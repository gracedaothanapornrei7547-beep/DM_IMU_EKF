#ifndef BSP_USB_H
#define BSP_USB_H

#include "stdint.h"

/**
 ******************************************************************************
 * USB CDC 虚拟串口封装（基于 TinyUSB）
 *
 *   usb_init();                 // main 里初始化一次
 *   while(1){ usb_task(); ... } // 主循环高频调用(驱动协议栈)
 *
 *   发送： usb_printf("roll=%.2f\r\n", roll);  /  usb_send(buf, len);
 *   接收： if (usb_available()) { n = usb_read(buf, sizeof(buf)); ... }
 *   就绪： if (usb_is_ready()) { ... }          // 主机已打开端口
 ******************************************************************************
 */

void     usb_init(void);                              /* 硬件初始化 + 启动 TinyUSB */
void     usb_task(void);                              /* 主循环里高频调用(tud_task) */
void     usb_irq_handler(void);                       /* 在 OTG_HS_IRQHandler 里调用 */

uint8_t  usb_is_ready(void);                          /* 1=CDC 已连接(主机已打开) */

void     usb_send(const uint8_t *data, uint16_t len); /* 发送原始字节 */
void     usb_printf(const char *fmt, ...);            /* 格式化发送 */

uint16_t usb_available(void);                         /* 可读字节数 */
uint16_t usb_read(uint8_t *buf, uint16_t maxlen);     /* 读取接收数据 */
void     usb_rx_flush(void);                          /* 清空接收缓冲 */

#endif /* BSP_USB_H */
