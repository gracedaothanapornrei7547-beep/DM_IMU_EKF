/**
 ******************************************************************************
 * @file    prio_inv.c
 * @brief   优先级反转 / 优先级继承 教学 demo(Step 7b 配套)
 *
 * 三个任务共享一把锁,优先级 L(10) < M(13) < H(16):
 *   - DemoLow  :拿锁 -> 临界区内忙等 CRIT_MS(占 CPU)-> 放锁。
 *   - DemoMid  :不碰锁,纯忙等 HOG_MS 占 CPU —— 反转的"元凶"。
 *   - DemoHigh :测"申请锁->拿到锁"的等待时间(µs)。
 *
 * 用 osDelay 把三者错开成确定时序(每 PERIOD_MS 一轮):
 *   t=0   L 拿锁,开始 CRIT_MS 忙等;
 *   t=+20 H 申请锁 -> 阻塞(L 持锁);
 *   t=+40 M 开始 HOG_MS 忙等。
 *
 * 二值信号量(无优先级继承):t=+40 时 M(13)抢占 L(10)-> L 干不完、放不了锁
 *   -> H 被迫等在 M 之后 => H wait ≈ CRIT 剩余 + HOG(被拖长)= 反转。
 * 互斥锁(有优先级继承):H 阻塞瞬间 L 被提升到 16 -> M(13)压不住 L
 *   -> L 速速干完放锁 => H wait ≈ CRIT(不被 M 拖)= 继承救场。
 ******************************************************************************
 */
#include "prio_inv.h"

#if defined(PRIO_INV_DEMO)

#include "cmsis_os2.h"
#include "stm32h7xx.h"   /* DWT->CYCCNT, SystemCoreClock */
#include "bsp_dwt.h"     /* DWT_Delay(秒):DWT 忙等,占 CPU 不让出 */

/* ---- 可调参数(ms) ---- */
#define PERIOD_MS    1000u   /* 每轮演示周期 */
#define CRIT_MS       150u   /* L 持锁临界区忙等时长 */
#define HOG_MS        200u   /* M 不持锁、纯占 CPU 的忙等时长 */
#define H_OFFSET_MS    20u   /* H 比 L 晚这么久申请锁(确保 L 已持锁) */
#define M_OFFSET_MS    40u   /* M 比 L 晚这么久开始占 CPU(此时 H 已阻塞) */

/* H 测得的等锁延迟(µs);供 statsTask 打印 */
volatile uint32_t g_prioinv_wait_us     = 0;
volatile uint32_t g_prioinv_wait_max_us = 0;

/* ---- 锁:编译期 A/B 切换 ---- */
#if defined(PRIO_INV_USE_MUTEX)
  static osMutexId_t s_lock;
  static const osMutexAttr_t s_lock_attr = { .name = "prioInvLock" };
  #define LOCK_CREATE()   (s_lock = osMutexNew(&s_lock_attr))
  #define LOCK_ACQUIRE()  osMutexAcquire(s_lock, osWaitForever)
  #define LOCK_RELEASE()  osMutexRelease(s_lock)
  #define LOCK_KIND_STR   "MUTEX"
#else
  static osSemaphoreId_t s_lock;
  static const osSemaphoreAttr_t s_lock_attr = { .name = "prioInvLock" };
  /* 二值信号量当锁:max=1、初值=1(可用)。无所有权、无优先级继承 -> 会反转 */
  #define LOCK_CREATE()   (s_lock = osSemaphoreNew(1u, 1u, &s_lock_attr))
  #define LOCK_ACQUIRE()  osSemaphoreAcquire(s_lock, osWaitForever)
  #define LOCK_RELEASE()  osSemaphoreRelease(s_lock)
  #define LOCK_KIND_STR   "BIN-SEM"
#endif

static uint32_t cyc_to_us(uint32_t cyc)
{
    return cyc / (SystemCoreClock / 1000000u);   /* 240MHz -> /240 */
}

/* 低优:拿锁 -> 临界区忙等(占 CPU,不让出)-> 放锁 */
static void DemoLow(void *arg)
{
    (void)arg;
    uint32_t t = osKernelGetTickCount();
    for (;;)
    {
        LOCK_ACQUIRE();
        DWT_Delay(CRIT_MS / 1000.0f);   /* 忙等:持锁期间霸占 CPU */
        LOCK_RELEASE();
        t += PERIOD_MS;
        osDelayUntil(t);
    }
}

/* 中优:不碰锁,纯忙等占 CPU —— 反转的元凶 */
static void DemoMid(void *arg)
{
    (void)arg;
    osDelay(M_OFFSET_MS);
    uint32_t t = osKernelGetTickCount();
    for (;;)
    {
        DWT_Delay(HOG_MS / 1000.0f);
        t += PERIOD_MS;
        osDelayUntil(t);
    }
}

/* 高优:测"申请锁->拿到锁"的等待时间 */
static void DemoHigh(void *arg)
{
    (void)arg;
    osDelay(H_OFFSET_MS);
    uint32_t t = osKernelGetTickCount();
    for (;;)
    {
        uint32_t c0 = DWT->CYCCNT;
        LOCK_ACQUIRE();
        uint32_t wait = cyc_to_us(DWT->CYCCNT - c0);   /* < 17.9s,CYCCNT 不溢出,直接差值 */
        LOCK_RELEASE();
        g_prioinv_wait_us = wait;
        if (wait > g_prioinv_wait_max_us) g_prioinv_wait_max_us = wait;
        t += PERIOD_MS;
        osDelayUntil(t);
    }
}

static const osThreadAttr_t aL = { .name = "demoLow",  .stack_size = 256 * 4, .priority = (osPriority_t)osPriorityLow2 };
static const osThreadAttr_t aM = { .name = "demoMid",  .stack_size = 256 * 4, .priority = (osPriority_t)osPriorityLow5 };
static const osThreadAttr_t aH = { .name = "demoHigh", .stack_size = 256 * 4, .priority = (osPriority_t)osPriorityBelowNormal };

void PrioInvDemo_Init(void)
{
    LOCK_CREATE();
    osThreadNew(DemoLow,  NULL, &aL);
    osThreadNew(DemoMid,  NULL, &aM);
    osThreadNew(DemoHigh, NULL, &aH);
}

const char *PrioInvDemo_LockKind(void) { return LOCK_KIND_STR; }

#endif /* PRIO_INV_DEMO */
