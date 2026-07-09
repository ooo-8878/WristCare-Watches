#ifndef __INCLUDES_H
#define __INCLUDES_H

/* 标准C库头文件 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* stm32头文件*/
#include "stm32f4xx.h"

/* 系统头文件 */
#include "sys.h"
#include "usart.h"
#include "delay.h"

/* 外部模块相关头文件 */
#include "tim.h"
#include "lcd_font.h"
#include "tft.h"
#include "bmp.h"
#include "touch.h"  
#include "my_rtc.h"
#include "dht11.h"
#include "vibration_motor.h"
#include "bt24.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "i2c.h"
#include "algorithm.h"
#include "max30102.h"

/* lvgl相关头文件 */
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "ui.h"

/* FreeRTOS相关头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"
#include "semphr.h"
#include "event_groups.h"

typedef struct __task_t{
    TaskFunction_t pxTaskCode;
    const char * const pcName;
    const configSTACK_DEPTH_TYPE usStackDepth;
    void * const pvParameters;
    UBaseType_t uxPriority;
    TaskHandle_t * const pxCreatedTask;
}task_t;

/* 事件标志组宏定义 */
#define EVENT_GROUPS_RTC_WAKEUP 0x01
#define EVENT_GROUPS_RTC_ALARM  0x02

/* 无操作熄屏值 */
#define SCREEN_OFF_TIME	 10

/* 单片机工作模式 */
#define CPU_MODE_RUN   1
#define CPU_MODE_SLEEP 0

extern SemaphoreHandle_t g_mutex_lvgl_handle;

extern EventGroupHandle_t g_event_group_handle;

extern QueueHandle_t  g_queue_bt24_handle;

/* FreeRTOS任务句柄 */
extern TaskHandle_t app_task_init_handle;
extern TaskHandle_t app_task_lvgl_handle;
extern TaskHandle_t app_task_max30102_handle;
extern TaskHandle_t app_task_rtc_handle;
extern TaskHandle_t app_task_dht11_handle;
extern TaskHandle_t app_task_motor_handle;
extern TaskHandle_t app_task_rtc_alarm_handlde;
extern TaskHandle_t app_task_ble_handlde;
extern TaskHandle_t app_task_icon_handlde;
extern TaskHandle_t app_task_mpu6050_handlde;
extern TaskHandle_t app_task_sedentary_remind_handle ;
extern TaskHandle_t app_task_istouch_lcd_handlde;

extern lv_disp_drv_t *g_disp_drvp;  

/* 全局变量 */
extern volatile uint32_t g_rtc_alarm_switch;
extern volatile uint32_t g_sedentary_switch;
extern volatile uint32_t g_raise_wrist_turn_on_lcd_switch;
extern volatile uint32_t g_lcd_display;
extern volatile uint32_t g_lcd_brightness;
extern volatile uint32_t g_lcd_self_turn_off_switch;
extern volatile uint32_t g_system_no_opreation_cnt;

extern void dgb_printf_safe(const char *format, ...);

#endif
