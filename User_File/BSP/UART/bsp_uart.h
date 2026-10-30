#ifndef BSP_UART_H
#define BSP_UART_H

#include "stdint.h"

/* USART1 调试串口（PA9=TX, PA10=RX, 115200 8N1）
 * 用法：uart_printf("temp=%.2f\r\n", imu_temp);
 */
void uart_printf(const char *fmt, ...);

/* 发送原始字节（阻塞） */
void uart_send(const uint8_t *data, uint16_t len);

#endif /* BSP_UART_H */
