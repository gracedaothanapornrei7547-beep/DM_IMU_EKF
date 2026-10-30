#ifndef APP_PRIO_INV_H
#define APP_PRIO_INV_H

/* 优先级反转 / 优先级继承 教学 demo(Step 7b 配套,学习用,默认关闭)
 *
 * 开关在 CMakeLists.txt:
 *   PRIO_INV_DEMO        定义=开启(创建 L/M/H 三个任务 + 一把共享锁)
 *   PRIO_INV_USE_MUTEX   定义=锁用互斥锁(有优先级继承);不定义=二值信号量(复现反转)
 *
 * 观测:H 测得的"申请锁->拿到锁"等待(µs)由 statsTask 经 USART1 打印:
 *   PRIO-INV[BIN-SEM] H wait=330000us (max 330000)   <- 反转,被中优任务拖长
 *   PRIO-INV[MUTEX]   H wait=130000us (max 130000)   <- 优先级继承,大幅缩短
 *
 * 学完整体删除:删本目录、CMakeLists 的源/开关、freertos.c 里两处 #ifdef 块。
 */

/* 在 MX_FREERTOS_Init 里(调度器启动前)创建 demo 的锁与三个任务 */
void PrioInvDemo_Init(void);

/* 当前锁类型字符串("MUTEX"/"BIN-SEM"),给打印用 */
const char *PrioInvDemo_LockKind(void);

#endif /* APP_PRIO_INV_H */
