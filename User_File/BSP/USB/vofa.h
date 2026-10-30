#ifndef VOFA_H
#define VOFA_H

#include "stdint.h"

/* VOFA+ JustFloat 协议发送（配合上位机 VOFA+ 软件）
 * 通道顺序在 VOFA+ 里依次对应：ch0, ch1, ...
 */

/* 发送任意一组 float（≤16 通道）—— USB CDC */
void vofa_send_floats(const float *data, uint8_t num);
/* 发送任意一组 float —— USART1（干净可靠，推荐） */
void vofa_send_floats_uart(const float *data, uint8_t num);

/* 发送 IMU 姿态：ch0=roll ch1=pitch ch2=yaw ch3=yaw_total ch4=temp
 * (定义 IMU_RAW_LOG 时改发 7 通道原始 gyro/accel/dt 供离线调参) */
void vofa_send_imu(void);        /* USB CDC */
void vofa_send_imu_uart(void);   /* USART1（推荐）*/

#endif /* VOFA_H */
