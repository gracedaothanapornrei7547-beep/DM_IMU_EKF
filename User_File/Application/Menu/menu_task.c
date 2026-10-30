#include "menu_task.h"

#include "FreeRTOS.h"
#include "adc_key.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "imu.h"
#include "lcd.h"
#include "portable.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define MENU_ITEM_COUNT       6U
#define MENU_ITEMS_PER_PAGE   5U
#define MENU_SCAN_PERIOD_MS   20U
#define MENU_DRAW_PERIOD_MS   50U

typedef enum
{
    MENU_PAGE_ROOT = 0,
    MENU_PAGE_IMU_ATTITUDE,
    MENU_PAGE_IMU_RAW,
    MENU_PAGE_SYSTEM,
    MENU_PAGE_USB,
    MENU_PAGE_KEY_MONITOR,
    MENU_PAGE_ABOUT
} menu_page_t;

typedef struct
{
    const char *label;
    menu_page_t page;
} menu_item_t;

typedef struct
{
    bool inited;
    menu_page_t page;
    uint8_t menu_sel;

    float roll;
    float pitch;
    float yaw;
    float yaw_total;
    float temp;
    float gyro[3];
    float accel[3];
    uint8_t imu_status;
    uint8_t temp_ready;

    uint32_t uptime;
    uint32_t heartbeat;
    size_t free_heap;
    uint8_t usb_ready;
    uint16_t usb_rx;
    uint16_t key_adc;
    adc_key_t key_decoded;
} menu_render_cache_t;

static const menu_item_t s_items[MENU_ITEM_COUNT] = {
    {"1. IMU Attitude", MENU_PAGE_IMU_ATTITUDE},
    {"2. IMU Raw Data", MENU_PAGE_IMU_RAW},
    {"3. System Info",  MENU_PAGE_SYSTEM},
    {"4. USB Status",   MENU_PAGE_USB},
    {"5. Key Monitor",  MENU_PAGE_KEY_MONITOR},
    {"6. About MC02",   MENU_PAGE_ABOUT},
};

static osThreadId_t s_menu_task;
static menu_page_t s_page = MENU_PAGE_ROOT;
static uint8_t s_menu_sel;
static uint16_t s_latest_key_adc = UINT16_MAX;
static menu_render_cache_t s_cache;

static const osThreadAttr_t s_menu_task_attr = {
    .name = "lcdMenu",
    .stack_size = 768U * 4U,
    .priority = osPriorityBelowNormal,
};

static bool FloatChanged(float current, float cached, float epsilon)
{
    return !isfinite(current) || !isfinite(cached) ||
           fabsf(current - cached) > epsilon;
}

static void Menu_EnterPage(menu_page_t page)
{
    s_page = page;
    s_cache.inited = false;
}

static void Menu_HandleKey(adc_key_t key)
{
    if (key == ADC_KEY_NONE)
    {
        return;
    }

    if (s_page == MENU_PAGE_ROOT)
    {
        if (key == ADC_KEY_UP)
        {
            s_menu_sel = (s_menu_sel == 0U) ?
                         (MENU_ITEM_COUNT - 1U) : (uint8_t)(s_menu_sel - 1U);
        }
        else if (key == ADC_KEY_DOWN)
        {
            s_menu_sel = (uint8_t)((s_menu_sel + 1U) % MENU_ITEM_COUNT);
        }
        else if (key == ADC_KEY_LEFT || key == ADC_KEY_RIGHT)
        {
            if (s_menu_sel < MENU_ITEMS_PER_PAGE)
            {
                s_menu_sel = MENU_ITEM_COUNT - 1U;
            }
            else
            {
                s_menu_sel = 0U;
            }
        }
        else if (key == ADC_KEY_CENTER)
        {
            Menu_EnterPage(s_items[s_menu_sel].page);
        }
    }
    else if (key == ADC_KEY_LEFT ||
             (key == ADC_KEY_CENTER && s_page != MENU_PAGE_KEY_MONITOR))
    {
        Menu_EnterPage(MENU_PAGE_ROOT);
    }
}

static void Menu_DrawHeader(const char *title)
{
    LCD_Fill(0, 0, LCD_W, 38, BLACK);
    LCD_ShowString(14, 8, (const uint8_t *)title, CYAN, BLACK, 24, 0);
}

static void Menu_DrawFooter(const char *text)
{
    LCD_Fill(0, 216, LCD_W, LCD_H, GRAYBLUE);
    LCD_ShowString(8, 222, (const uint8_t *)text, WHITE, GRAYBLUE, 12, 0);
}

static void Menu_DrawLine(uint16_t y, uint16_t color, const char *text)
{
    LCD_Fill(10, y, LCD_W - 8U, y + 20U, BLACK);
    LCD_ShowString(14, y + 2U, (const uint8_t *)text, color, BLACK, 16, 0);
}

static void Menu_RenderRoot(void)
{
    bool force = !s_cache.inited;

    if (force)
    {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        Menu_DrawHeader("-- Main Menu --");
        Menu_DrawFooter("UP/DN move  L/R page  CTR enter");
        s_cache.menu_sel = UINT8_MAX;
    }

    if (force || s_cache.menu_sel != s_menu_sel)
    {
        uint8_t page = s_menu_sel / MENU_ITEMS_PER_PAGE;

        LCD_Fill(238, 8, LCD_W, 34, BLACK);
        LCD_ShowChar(238, 8, 'P', CYAN, BLACK, 24, 0);
        LCD_ShowChar(258, 8, (uint8_t)('1' + page), CYAN, BLACK, 24, 0);

        for (uint8_t row = 0; row < MENU_ITEMS_PER_PAGE; ++row)
        {
            uint8_t index = (uint8_t)(page * MENU_ITEMS_PER_PAGE + row);
            uint16_t y = (uint16_t)(48U + row * 32U);

            LCD_Fill(0, y, LCD_W, y + 28U, BLACK);
            if (index < MENU_ITEM_COUNT)
            {
                uint16_t color = (index == s_menu_sel) ? YELLOW : WHITE;
                uint8_t marker = (index == s_menu_sel) ? '>' : ' ';

                LCD_ShowChar(10, y, marker, color, BLACK, 24, 0);
                LCD_ShowString(34, y, (const uint8_t *)s_items[index].label,
                               color, BLACK, 24, 0);
            }
        }
        s_cache.menu_sel = s_menu_sel;
    }

    s_cache.inited = true;
}

static void Menu_RenderImuAttitude(void)
{
    bool force = !s_cache.inited;
    char line[34];

    if (force)
    {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        Menu_DrawHeader("-- IMU Attitude --");
        Menu_DrawFooter("LEFT / CENTER back");
    }

    if (force || FloatChanged(imu_roll, s_cache.roll, 0.05f))
    {
        snprintf(line, sizeof(line), "Roll      %9.2f deg", imu_roll);
        Menu_DrawLine(44, CYAN, line);
        s_cache.roll = imu_roll;
    }
    if (force || FloatChanged(imu_pitch, s_cache.pitch, 0.05f))
    {
        snprintf(line, sizeof(line), "Pitch     %9.2f deg", imu_pitch);
        Menu_DrawLine(70, CYAN, line);
        s_cache.pitch = imu_pitch;
    }
    if (force || FloatChanged(imu_yaw, s_cache.yaw, 0.05f))
    {
        snprintf(line, sizeof(line), "Yaw       %9.2f deg", imu_yaw);
        Menu_DrawLine(96, CYAN, line);
        s_cache.yaw = imu_yaw;
    }
    if (force || FloatChanged(imu_yaw_total, s_cache.yaw_total, 0.05f))
    {
        snprintf(line, sizeof(line), "Yaw total %9.2f deg", imu_yaw_total);
        Menu_DrawLine(122, WHITE, line);
        s_cache.yaw_total = imu_yaw_total;
    }
    if (force || FloatChanged(imu_temp, s_cache.temp, 0.05f))
    {
        snprintf(line, sizeof(line), "Temp      %9.2f C", imu_temp);
        Menu_DrawLine(148, YELLOW, line);
        s_cache.temp = imu_temp;
    }
    if (force || s_cache.imu_status != imu_init_status ||
        s_cache.temp_ready != imu_temp_ready)
    {
        snprintf(line, sizeof(line), "BMI088 %-3s  Heater %-3s",
                 imu_init_status == 0U ? "OK" : "ERR",
                 imu_temp_ready != 0U ? "OK" : "WAIT");
        Menu_DrawLine(174, imu_init_status == 0U ? GREEN : RED, line);
        s_cache.imu_status = imu_init_status;
        s_cache.temp_ready = imu_temp_ready;
    }

    s_cache.inited = true;
}

static void Menu_RenderImuRaw(void)
{
    bool force = !s_cache.inited;
    char line[34];

    if (force)
    {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        Menu_DrawHeader("-- IMU Raw Data --");
        Menu_DrawFooter("Gyro rad/s   Accel m/s^2");
    }

    for (uint8_t axis = 0; axis < 3U; ++axis)
    {
        if (force || FloatChanged(imu_gyro[axis], s_cache.gyro[axis], 0.005f))
        {
            snprintf(line, sizeof(line), "Gyro %c  %11.4f",
                     (char)('X' + axis), imu_gyro[axis]);
            Menu_DrawLine((uint16_t)(42U + axis * 25U), CYAN, line);
            s_cache.gyro[axis] = imu_gyro[axis];
        }
    }

    for (uint8_t axis = 0; axis < 3U; ++axis)
    {
        if (force || FloatChanged(imu_accel[axis], s_cache.accel[axis], 0.01f))
        {
            snprintf(line, sizeof(line), "Accel %c %11.4f",
                     (char)('X' + axis), imu_accel[axis]);
            Menu_DrawLine((uint16_t)(122U + axis * 25U), YELLOW, line);
            s_cache.accel[axis] = imu_accel[axis];
        }
    }

    s_cache.inited = true;
}

static void Menu_RenderSystem(void)
{
    extern volatile uint32_t heartbeat_count;
    bool force = !s_cache.inited;
    char line[34];
    uint32_t tick_frequency = osKernelGetTickFreq();
    uint32_t uptime = tick_frequency == 0U ?
                      0U : osKernelGetTickCount() / tick_frequency;
    uint32_t heartbeat = heartbeat_count;
    size_t free_heap = xPortGetFreeHeapSize();

    if (force)
    {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        Menu_DrawHeader("-- System Info --");
        Menu_DrawFooter("LEFT / CENTER back");
    }

    if (force || s_cache.uptime != uptime)
    {
        snprintf(line, sizeof(line), "Uptime       %9lu s",
                 (unsigned long)uptime);
        Menu_DrawLine(50, WHITE, line);
        s_cache.uptime = uptime;
    }
    if (force || s_cache.heartbeat != heartbeat)
    {
        snprintf(line, sizeof(line), "Heartbeat    %9lu",
                 (unsigned long)heartbeat);
        Menu_DrawLine(80, WHITE, line);
        s_cache.heartbeat = heartbeat;
    }
    if (force || s_cache.free_heap != free_heap)
    {
        snprintf(line, sizeof(line), "Free heap    %9lu B",
                 (unsigned long)free_heap);
        Menu_DrawLine(110, GREEN, line);
        s_cache.free_heap = free_heap;
    }
    if (force)
    {
        snprintf(line, sizeof(line), "CPU clock    %9lu MHz",
                 (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000U));
        Menu_DrawLine(140, CYAN, line);
        snprintf(line, sizeof(line), "RTOS tasks   %9lu",
                 (unsigned long)uxTaskGetNumberOfTasks());
        Menu_DrawLine(170, YELLOW, line);
    }

    s_cache.inited = true;
}

static void Menu_RenderUsb(void)
{
    bool force = !s_cache.inited;
    char line[34];
    uint8_t ready = usb_is_ready();
    uint16_t rx_bytes = usb_available();

    if (force)
    {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        Menu_DrawHeader("-- USB Status --");
        Menu_DrawFooter("LEFT / CENTER back");
        LCD_ShowString(20, 52, (const uint8_t *)"TinyUSB CDC",
                       WHITE, BLACK, 24, 0);
    }

    if (force || s_cache.usb_ready != ready)
    {
        snprintf(line, sizeof(line), "Mounted:    %-3s",
                 ready != 0U ? "YES" : "NO");
        Menu_DrawLine(98, ready != 0U ? GREEN : YELLOW, line);
        s_cache.usb_ready = ready;
    }
    if (force || s_cache.usb_rx != rx_bytes)
    {
        snprintf(line, sizeof(line), "RX pending: %5u bytes", rx_bytes);
        Menu_DrawLine(132, CYAN, line);
        s_cache.usb_rx = rx_bytes;
    }

    s_cache.inited = true;
}

static void Menu_RenderKeyMonitor(void)
{
    bool force = !s_cache.inited;
    char line[34];
    adc_key_t decoded = ADC_Key_Decode(s_latest_key_adc);

    if (force)
    {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        Menu_DrawHeader("-- Key Monitor --");
        Menu_DrawFooter("LEFT back / press all five keys");
        LCD_ShowString(14, 104, (const uint8_t *)"CENTER       <  2000",
                       LGRAY, BLACK, 12, 0);
        LCD_ShowString(14, 122, (const uint8_t *)"DOWN       8000..18000",
                       LGRAY, BLACK, 12, 0);
        LCD_ShowString(14, 140, (const uint8_t *)"UP        21000..31000",
                       LGRAY, BLACK, 12, 0);
        LCD_ShowString(14, 158, (const uint8_t *)"LEFT      33000..43000",
                       LGRAY, BLACK, 12, 0);
        LCD_ShowString(14, 176, (const uint8_t *)"RIGHT     46000..56000",
                       LGRAY, BLACK, 12, 0);
    }

    if (force || s_cache.key_adc != s_latest_key_adc)
    {
        snprintf(line, sizeof(line), "Raw ADC: %5u", s_latest_key_adc);
        Menu_DrawLine(48, CYAN, line);
        s_cache.key_adc = s_latest_key_adc;
    }
    if (force || s_cache.key_decoded != decoded)
    {
        snprintf(line, sizeof(line), "Decoded: %-6s", ADC_Key_Name(decoded));
        Menu_DrawLine(76, decoded == ADC_KEY_NONE ? WHITE : YELLOW, line);
        s_cache.key_decoded = decoded;
    }

    s_cache.inited = true;
}

static void Menu_RenderAbout(void)
{
    if (!s_cache.inited)
    {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        Menu_DrawHeader("-- About MC02 --");
        Menu_DrawFooter("LEFT / CENTER back");
        LCD_ShowString(58, 54, (const uint8_t *)"DAMIAO MC02",
                       CYAN, BLACK, 24, 0);
        LCD_ShowString(46, 96, (const uint8_t *)"STM32H723VGT6",
                       WHITE, BLACK, 16, 0);
        LCD_ShowString(46, 126, (const uint8_t *)"ST7789 280x240",
                       WHITE, BLACK, 16, 0);
        LCD_ShowString(46, 156, (const uint8_t *)"FreeRTOS + TinyUSB",
                       WHITE, BLACK, 16, 0);
        LCD_ShowString(46, 186, (const uint8_t *)"CMSIS-DAP / ST-Link",
                       LGRAY, BLACK, 16, 0);
        s_cache.inited = true;
    }
}

static void Menu_Render(void)
{
    if (s_cache.page != s_page)
    {
        memset(&s_cache, 0, sizeof(s_cache));
        s_cache.page = s_page;
    }

    switch (s_page)
    {
        case MENU_PAGE_ROOT:         Menu_RenderRoot();        break;
        case MENU_PAGE_IMU_ATTITUDE: Menu_RenderImuAttitude(); break;
        case MENU_PAGE_IMU_RAW:      Menu_RenderImuRaw();      break;
        case MENU_PAGE_SYSTEM:       Menu_RenderSystem();      break;
        case MENU_PAGE_USB:          Menu_RenderUsb();         break;
        case MENU_PAGE_KEY_MONITOR:  Menu_RenderKeyMonitor();  break;
        case MENU_PAGE_ABOUT:        Menu_RenderAbout();       break;
        default:                     Menu_EnterPage(MENU_PAGE_ROOT); break;
    }
}

static void MenuTask_Entry(void *argument)
{
    bool key_ready;
    uint32_t next_draw;

    (void)argument;
    LCD_Init();
    key_ready = ADC_Key_Init();
    memset(&s_cache, 0, sizeof(s_cache));
    s_cache.page = (menu_page_t)UINT8_MAX;
    next_draw = osKernelGetTickCount();

    for (;;)
    {
        uint32_t now;

        if (key_ready)
        {
            s_latest_key_adc = ADC_Key_Read();
            Menu_HandleKey(ADC_Key_Scan(s_latest_key_adc));
        }

        now = osKernelGetTickCount();
        if ((int32_t)(now - next_draw) >= 0)
        {
            Menu_Render();
            next_draw = now + pdMS_TO_TICKS(MENU_DRAW_PERIOD_MS);

            if (!key_ready && s_page == MENU_PAGE_ROOT)
            {
                Menu_DrawFooter("ADC1 PA5 init failed");
            }
        }

        osDelay(pdMS_TO_TICKS(MENU_SCAN_PERIOD_MS));
    }
}

void MenuTask_Init(void)
{
    if (s_menu_task == NULL)
    {
        s_menu_task = osThreadNew(MenuTask_Entry, NULL, &s_menu_task_attr);
    }
}
