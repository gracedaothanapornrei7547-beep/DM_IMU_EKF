/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @brief   USART1 调试串口封装（格式化打印）
 *
 * 阻塞式发送，适合调试打印。波特率/引脚在 CubeMX 配置（USART1: PA9/PA10,
 * 115200 8N1），句柄 huart1 由 Core/Src/usart.c 提供。
 *
 * 注意：printf 浮点需链接选项 -u _printf_float（已在 CMakeLists.txt 加上），
 * 否则 %f 打印为空。
 ******************************************************************************
 */
#include "bsp_uart.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART_TX_BUF_SIZE 256

static uint8_t s_tx_buf[UART_TX_BUF_SIZE];

void uart_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)s_tx_buf, UART_TX_BUF_SIZE, fmt, ap);
    va_end(ap);

    if (len <= 0)
    {
        return;
    }
    if (len > UART_TX_BUF_SIZE)
    {
        len = UART_TX_BUF_SIZE;   /* 截断保护 */
    }
    HAL_UART_Transmit(&huart1, s_tx_buf, (uint16_t)len, HAL_MAX_DELAY);
}

void uart_send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, HAL_MAX_DELAY);
}
