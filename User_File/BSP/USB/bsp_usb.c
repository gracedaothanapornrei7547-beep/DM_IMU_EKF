/**
 ******************************************************************************
 * @file    bsp_usb.c
 * @brief   USB CDC 封装（基于 TinyUSB，根治 ST HAL 栈的 ITTXFE 高速发送死锁）
 *
 * 硬件层(时钟/电源/NVIC)复用 CubeMX 原 MspInit 的配置；协议栈用 TinyUSB。
 * DM02(STM32H723) 单 OTG_HS，TinyUSB 作为 rhport 1、全速(内置 FS PHY)。
 ******************************************************************************
 */
#include "bsp_usb.h"
#include "tusb.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define USB_TX_BUF_SIZE 256
static uint8_t s_tx_buf[USB_TX_BUF_SIZE];

/**
 * @brief USB 硬件初始化 + 启动 TinyUSB
 * @note  在 main() 外设初始化之后调用一次。时钟/电源/NVIC 同原 CubeMX MspInit；
 *        内置 FS PHY 不需要配 PA11/PA12 的 GPIO（ST 原流程也未配）。
 */
void usb_init(void)
{
    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    clk.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;   /* 48MHz from PLL */
    HAL_RCCEx_PeriphCLKConfig(&clk);

    HAL_PWREx_EnableUSBVoltageDetector();
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

    /* PA11=DM, PA12=DP 配成 OTG AF（TinyUSB dwc2 需要显式配置）*/
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);           /* 最高优先级 */
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);

    tud_init(BOARD_TUD_RHPORT);                        /* rhport 0 */
}

/**
 * @brief USB 协议栈轮询，必须在主循环里高频调用
 */
void usb_task(void)
{
    tud_task();
}

/**
 * @brief USB 中断处理（在 OTG_HS_IRQHandler 里调用，替代 HAL_PCD_IRQHandler）
 */
void usb_irq_handler(void)
{
    tud_int_handler(BOARD_TUD_RHPORT);
}

/* ---- 状态 ---- */
uint8_t usb_is_ready(void)
{
    return (tud_mounted()) ? 1u : 0u;
}

/* ---- 发送 ---- */
void usb_send(const uint8_t *data, uint16_t len)
{
    if (!tud_mounted())
    {
        return;
    }
    tud_cdc_write(data, len);
    tud_cdc_write_flush();
}

void usb_printf(const char *fmt, ...)
{
    if (!tud_mounted())
    {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)s_tx_buf, USB_TX_BUF_SIZE, fmt, ap);
    va_end(ap);
    if (len <= 0)
    {
        return;
    }
    if (len >= USB_TX_BUF_SIZE)
    {
        len = USB_TX_BUF_SIZE - 1;   /* vsnprintf 最多写 SIZE-1 有效字节 + '\0'，截断到此防越界 */
    }
    usb_send(s_tx_buf, (uint16_t)len);
}

/* ---- 接收（TinyUSB 内部已有缓冲，这里直接包装）---- */
uint16_t usb_available(void)
{
    return (uint16_t)tud_cdc_available();
}

uint16_t usb_read(uint8_t *buf, uint16_t maxlen)
{
    return (uint16_t)tud_cdc_read(buf, maxlen);
}

void usb_rx_flush(void)
{
    tud_cdc_read_flush();
}
