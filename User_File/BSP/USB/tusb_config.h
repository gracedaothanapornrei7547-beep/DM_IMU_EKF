#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ---- MCU / OS ---- */
#define CFG_TUSB_MCU              OPT_MCU_STM32H7
#define CFG_TUSB_OS               OPT_OS_NONE

/* DM02(STM32H723) 只有一个 OTG_HS。TinyUSB 的 CDC 内部把 device 发送硬编码在
 * rhport 0(cdc_device.c)，故 device 必须用 rhport 0；H723 上 controller[0] 也指向
 * OTG_HS(0x40040000)、IRQ=OTG_HS_IRQn，所以 rhport 0 正确。全速(内置 FS PHY)。 */
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define BOARD_TUD_RHPORT          0
#define BOARD_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG            0
#endif

/* ---- Device ---- */
#define CFG_TUD_ENABLED           1
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED

#define CFG_TUD_ENDPOINT0_SIZE    64

/* ---- CDC ---- */
#define CFG_TUD_CDC               1
#define CFG_TUD_CDC_RX_BUFSIZE    512
#define CFG_TUD_CDC_TX_BUFSIZE    512
#define CFG_TUD_CDC_EP_BUFSIZE    64

/* 其余类关闭 */
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
