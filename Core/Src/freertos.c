/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_usb.h"    /* usb_task() */
#include "vofa.h"       /* vofa_send_imu() */
#include "imu.h"        /* IMU_Update() */
#include "bsp_uart.h"   /* uart_send() -> USART1 */
#include "prio_inv.h"   /* 优先级反转 demo(默认关,PRIO_INV_DEMO) */
#include "menu_task.h"  /* MC02 LCD menu */
#include <string.h>
#include <stdio.h>     /* snprintf */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint32_t heartbeat_count = 0;

/* Step7b: 保护共享外设 USART1 的互斥锁。
 * statsTask 打印整张任务表、heartbeatTask 打印心跳行,两个低优先级任务抢同一个
 * huart1;uart_send 是阻塞式 HAL_UART_Transmit,没锁时会交错/撞 HAL_BUSY 丢字 ->
 * 任务表被 [HB] 行劈开、串口花屏。用互斥锁把"对 USART1 的一次完整打印"变成临界区:
 * 谁先拿到锁谁先打完,另一个等。互斥锁(非二值信号量):有所有权 + 优先级继承。 */
osMutexId_t uart1MutexHandle;
const osMutexAttr_t uart1Mutex_attributes = { .name = "uart1Mutex" };

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usbTask */
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
  .name = "usbTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for heartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
  .name = "heartbeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for statsTask */
osThreadId_t statsTaskHandle;
const osThreadAttr_t statsTask_attributes = {
  .name = "statsTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for attitudeQueue */
osMessageQueueId_t attitudeQueueHandle;
const osMessageQueueAttr_t attitudeQueue_attributes = {
  .name = "attitudeQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUsbTask(void *argument);
void StartHeartbeatTask(void *argument);
void StartImuTask(void *argument);
void StartStatsTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Step7b: 创建 USART1 互斥锁(必须在任务创建之前) */
  uart1MutexHandle = osMutexNew(&uart1Mutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of attitudeQueue */
  attitudeQueueHandle = osMessageQueueNew (4, sizeof(imu_msg_t), &attitudeQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of usbTask */
  usbTaskHandle = osThreadNew(StartUsbTask, NULL, &usbTask_attributes);

  /* creation of heartbeatTask */
  heartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &heartbeatTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

  /* creation of statsTask */
  statsTaskHandle = osThreadNew(StartStatsTask, NULL, &statsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  MenuTask_Init();
#ifdef PRIO_INV_DEMO
  PrioInvDemo_Init();   /* 创建 L/M/H 三个 demo 任务 + 共享锁(调度器启动前) */
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUsbTask */
/**
* @brief Function implementing the usbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbTask */
void StartUsbTask(void *argument)
{
  /* USER CODE BEGIN StartUsbTask */
  /* Infinite loop */
  uint32_t tx_div = 0;
  imu_msg_t m;
  uint8_t have = 0;
  for (;;)
  {
    usb_task();
    /* 排空队列只留最新一帧 */
    while (osMessageQueueGet(attitudeQueueHandle, &m, NULL, 0U) == osOK) { have = 1; }
    if (++tx_div >= 5)
    {
      tx_div = 0;
      if (have)
      {
        float ch[5] = { m.roll, m.pitch, m.yaw, m.yaw_total, m.temp };
        vofa_send_floats(ch, 5);   /* ★发队列取来的这帧,不再读全局 */
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartUsbTask */
}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
* @brief Function implementing the heartbeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
  /* USER CODE BEGIN StartHeartbeatTask */
  /* Infinite loop */
  for (;;)
  {
    heartbeat_count++;
    /* Step7b 演示:故意也往 USART1 打印,和 statsTask 抢同一个串口 */
    char hb[24];
    int n = snprintf(hb, sizeof(hb), "[HB %lu]\r\n", (unsigned long)heartbeat_count);
    /* Step7b: 同样拿锁再打印 -> 心跳行只会出现在两张表之间,绝不劈开表 */
    osMutexAcquire(uart1MutexHandle, osWaitForever);
    uart_send((const uint8_t *)hb, n);
    osMutexRelease(uart1MutexHandle);
    osDelay(2);     /* 演示:从 100 改到 2(~500Hz),让它在表格打印途中频繁插入 */

  }

  /* USER CODE END StartHeartbeatTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  /* Infinite loop */
  for (;;)
  {
    /* 阻塞等 TIM6 发来的通知(标志位bit0);没通知就一直睡,不占CPU */
    osThreadFlagsWait(0x01U, osFlagsWaitAny, osWaitForever);
    IMU_Update();    /* 收到通知 -> 算 EKF(原来在 TIM6 中断里,现在挪到任务) */

    /* 打包成一帧投进队列(满了丢,只保最新姿态) */
    imu_msg_t m = { imu_roll, imu_pitch, imu_yaw, imu_yaw_total, imu_temp };
    osMessageQueuePut(attitudeQueueHandle, &m, 0U, 0U);

  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartStatsTask */
/**
* @brief Function implementing the statsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStatsTask */
void StartStatsTask(void *argument)
{
  /* USER CODE BEGIN StartStatsTask */
  static TaskStatus_t cur[16];      /* 本次快照(static:放.bss,不占任务栈) */
  static TaskHandle_t prevH[16];    /* 上次各任务的句柄 */
  static uint32_t     prevRun[16];  /* 上次各任务的累计运行计数 */
  static UBaseType_t  prevN     = 0;
  static uint32_t     prevTotal = 0;
  char line[80];

  for (;;)
  {
    uint32_t    total = 0;
    UBaseType_t n = uxTaskGetSystemState(cur, 16, &total);  /* 一次拿全:名/态/优先级/栈/运行计数 */
    uint32_t    totalDelta = total - prevTotal;             /* 这≈1秒 CPU 总周期(无符号减,绕回安全) */
    if (totalDelta == 0u) totalDelta = 1u;                  /* 防首次除零 */

    /* Step7b: 拿锁 -> 整张表(表头/各行/IMU us 行)作为一个临界区一次打完,
     * 期间 heartbeatTask 想打印会阻塞在锁上等,打不进表中间 */
    osMutexAcquire(uart1MutexHandle, osWaitForever);

    const char *hdr = "\r\n== Task         St Pri Stk CPU% ==\r\n";
    uart_send((const uint8_t *)hdr, strlen(hdr));

    for (UBaseType_t i = 0; i < n; i++)
    {
      uint32_t prev = 0;                                    /* 在上次快照里按句柄找同一任务 */
      for (UBaseType_t j = 0; j < prevN; j++)
        if (prevH[j] == cur[i].xHandle) { prev = prevRun[j]; break; }

      uint32_t d   = cur[i].ulRunTimeCounter - prev;        /* 这任务这1秒用掉的周期 */
      uint32_t pct = (uint32_t)((uint64_t)d * 100u / totalDelta);

      char s;
      switch (cur[i].eCurrentState) {
        case eRunning:   s = 'X'; break;   /* 正在跑 */
        case eReady:     s = 'R'; break;   /* 就绪 */
        case eBlocked:   s = 'B'; break;   /* 阻塞(osDelay/等通知) */
        case eSuspended: s = 'S'; break;   /* 挂起 */
        default:         s = '?'; break;
      }
      int len = snprintf(line, sizeof(line), "%-12s %c  %2lu %4u %3lu\r\n",
                         cur[i].pcTaskName, s,
                         (unsigned long)cur[i].uxCurrentPriority,
                         (unsigned)cur[i].usStackHighWaterMark,
                         (unsigned long)pct);
      uart_send((const uint8_t *)line, len);
    }

    for (UBaseType_t i = 0; i < n; i++) { prevH[i] = cur[i].xHandle; prevRun[i] = cur[i].ulRunTimeCounter; }
    prevN = n; prevTotal = total;

    /* Step① 调试:打印 IMU_Update 三段耗时(µs),定位 SPI/EKF/PID 瓶颈 */
    {
      extern volatile uint32_t dbg_us_spi, dbg_us_ekf, dbg_us_pid;
      int dn = snprintf(line, sizeof(line), "IMU us: SPI=%lu EKF=%lu PID=%lu\r\n",
                        (unsigned long)dbg_us_spi, (unsigned long)dbg_us_ekf, (unsigned long)dbg_us_pid);
      uart_send((const uint8_t *)line, dn);
    }

#ifdef PRIO_INV_DEMO
    /* 优先级反转 demo:H 测得的等锁延迟(µs)。对比 BIN-SEM 与 MUTEX 两次烧录的数值 */
    {
      extern volatile uint32_t g_prioinv_wait_us, g_prioinv_wait_max_us;
      int pn = snprintf(line, sizeof(line), "PRIO-INV[%s] H wait=%lu us (max %lu)\r\n",
                        PrioInvDemo_LockKind(),
                        (unsigned long)g_prioinv_wait_us, (unsigned long)g_prioinv_wait_max_us);
      uart_send((const uint8_t *)line, pn);
    }
#endif

    osMutexRelease(uart1MutexHandle);                       /* Step7b: 出临界区,放行 heartbeatTask */

    osDelay(1000);                                          /* 1秒一刷,正好定义了"每秒CPU%" */
  }
  /* USER CODE END StartStatsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 运行时统计时基 = DWT 周期计数器(240MHz) */
void RunTimeStats_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能内核跟踪单元 */
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* 启动 CYCCNT 自由计数 */
}
uint32_t RunTimeStats_GetCount(void)
{
  return DWT->CYCCNT;
}



/* USER CODE END Application */

