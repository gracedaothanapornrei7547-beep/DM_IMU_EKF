#ifndef APP_IMU_H
#define APP_IMU_H

#include "stdint.h"

/* 最新一帧 IMU 数据（由 IMU_Task 更新）
 * gyro  : 角速度 rad/s   [x, y, z]
 * accel : 加速度 m/s^2   [x, y, z]
 * temp  : 芯片温度 °C
 */
extern float imu_gyro[3];
extern float imu_accel[3];
extern float imu_temp;

/* 恒温目标已到达标志（温度进入目标附近后置 1，可用于判断 IMU 是否可信） */
extern uint8_t imu_temp_ready;

/* 姿态解算输出（四元数 EKF），角度单位 deg */
extern float imu_yaw;          /* 航向角（无磁力计，长期缓慢漂移） */
extern float imu_pitch;        /* 俯仰角 */
extern float imu_roll;         /* 横滚角 */
extern float imu_yaw_total;    /* 连续航向角（累计、不跳变） */
extern float imu_q[4];         /* 姿态四元数 */

/* 一帧姿态消息(走队列从 imuTask 传给 usbTask,一次传一整帧避免撕裂读) */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float yaw_total;
    float temp;
} imu_msg_t;

/* BMI088 初始化结果：0=成功，非0=BMI088driver.h 里的错误码（最后一次尝试） */
extern uint8_t imu_init_status;

/* 最近一次 IMU_Update 的 dt(s)，离线调参采集原始数据时用 */
extern float imu_dt;

/* 在 main() 外设初始化之后、while(1) 之前调用一次。
 * 返回 0 表示成功；非0 为错误码。有限次重试后即返回，不再死等，
 * 这样即使 IMU 没接好，主循环的串口打印也能正常跑，方便排查。 */
uint8_t IMU_Init(void);

/* 在 main() 的 while(1) 里循环调用；内部靠 ACC_INT 数据就绪标志驱动 */
void IMU_Task(void);

/* 一次完整更新(读数据+EKF+温控)，由 TIM6 1kHz 定时器直接调用 */
void IMU_Update(void);

#endif /* APP_IMU_H */
