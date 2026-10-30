/**
 ******************************************************************************
 * @file    usb_descriptors.c
 * @brief   TinyUSB CDC-ACM 描述符（DM02 虚拟串口）
 ******************************************************************************
 */
#include "tusb.h"
#include <string.h>

#define USB_VID   0x0483       /* STMicroelectronics（Windows 自动识别 VCP；与原 ST 一致）*/
#define USB_PID   0x5740
#define USB_BCD   0x0200

/* ---------------- 设备描述符 ---------------- */
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,        /* IAD：CDC 组合设备 */
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

/* ---------------- 配置描述符 ---------------- */
enum
{
    ITF_NUM_CDC = 0,    /* CDC 通信接口 */
    ITF_NUM_CDC_DATA,   /* CDC 数据接口 */
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const desc_fs_configuration[] =
{
    /* config number, interface count, string index, total length, attribute, power(mA) */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    /* CDC: interface, string idx, notif EP, notif size, data OUT EP, data IN EP, EP size */
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_fs_configuration;
}

/* ---------------- 字符串描述符 ---------------- */
char const *string_desc_arr[] =
{
    (const char[]){0x09, 0x04},   /* 0: 语言 = English (0x0409) */
    "DM02",                        /* 1: 厂商 */
    "DM02 IMU Virtual ComPort",    /* 2: 产品 */
    "DM02-0001",                   /* 3: 序列号 */
    "DM02 CDC",                    /* 4: CDC 接口 */
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t chr_count;

    if (index == 0)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else
    {
        if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
        {
            return NULL;
        }
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31)
        {
            chr_count = 31;
        }
        for (uint8_t i = 0; i < chr_count; i++)
        {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
