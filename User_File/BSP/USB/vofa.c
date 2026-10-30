/**
 ******************************************************************************
 * @file    vofa.c
 * @brief   VOFA+ JustFloat 协议（移植自达妙官方例程 CtrBoard-H7_USB/vofa+）
 *
 * JustFloat 帧格式：N 个小端 float 数据 + 4 字节帧尾 0x00 0x00 0x80 0x7F。
 * 配合上位机 VOFA+ 软件（JustFloat 模式）实时绘图。
 * 通过 USB CDC 发送（usb_send 已做未枚举保护与非阻塞处理）。
 ******************************************************************************
 */
#include "vofa.h"
#include "bsp_usb.h"
#include "bsp_uart.h"
#include "imu.h"
#include <string.h>

#define VOFA_MAX_CH 16
static uint8_t s_frame[VOFA_MAX_CH * 4 + 4];

/* 构建 JustFloat 帧到 s_frame，返回总长度(数据+4字节帧尾) */
static uint16_t vofa_build(const float *data, uint8_t num)
{
    if (num > VOFA_MAX_CH)
    {
        num = VOFA_MAX_CH;
    }
    uint16_t idx = (uint16_t)num * 4u;
    memcpy(s_frame, data, idx);        /* Cortex-M 小端，直接拷贝即 byte0..byte3 顺序 */
    s_frame[idx++] = 0x00;             /* JustFloat 帧尾 */
    s_frame[idx++] = 0x00;
    s_frame[idx++] = 0x80;
    s_frame[idx++] = 0x7f;
    return idx;
}

/* 经 USB CDC 发送一组 float */
void vofa_send_floats(const float *data, uint8_t num)
{
    usb_send(s_frame, vofa_build(data, num));
}

/* 经 USART1 发送一组 float（阻塞、干净、不丢，VOFA+ 连串口） */
void vofa_send_floats_uart(const float *data, uint8_t num)
{
    uint16_t len = vofa_build(data, num);
    uart_send(s_frame, len);
}

/* 经 USB CDC 发送 IMU 数据（在主循环里调用，TinyUSB 要求与 tud_task 同上下文）
 * 正常: 5 通道姿态; 定义 IMU_RAW_LOG 时: 7 通道原始数据(供离线调参回放) */
void vofa_send_imu(void)
{
#if defined(IMU_RAW_LOG)
    extern float imu_dt;
    float ch[7] = {imu_gyro[0], imu_gyro[1], imu_gyro[2],
                   imu_accel[0], imu_accel[1], imu_accel[2], imu_dt};
    vofa_send_floats(ch, 7);
#else
    float ch[5] = {imu_roll, imu_pitch, imu_yaw, imu_yaw_total, imu_temp};
    vofa_send_floats(ch, 5);
#endif
}

/* 经 USART1 发送 IMU 姿态（5 通道：roll/pitch/yaw/yaw_total/temp）——推荐，干净可靠 */
void vofa_send_imu_uart(void)
{
    float ch[5] = {imu_roll, imu_pitch, imu_yaw, imu_yaw_total, imu_temp};
    vofa_send_floats_uart(ch, 5);
}
