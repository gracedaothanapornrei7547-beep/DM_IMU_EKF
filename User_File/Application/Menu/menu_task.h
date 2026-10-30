#ifndef DM02_MENU_TASK_H
#define DM02_MENU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Creates the LCD/menu FreeRTOS task. Call after osKernelInitialize(). */
void MenuTask_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* DM02_MENU_TASK_H */
