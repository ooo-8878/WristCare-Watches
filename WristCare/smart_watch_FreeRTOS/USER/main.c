/**
  ******************************************************************************
  * @file    main.c
  * @brief   智能手表 FreeRTOS 主程序文件
  *
  * @details 本文件是智能手表项目的主控程序，基于 STM32F1 + FreeRTOS + LVGL 架构，
  *          负责系统级初始化、所有业务任务的创建与调度，以及关键的同步/通信
  *          机制的管理。该模块在整个系统设计中扮演“应用调度中枢”的角色：
  *          它通过一个初始化任务完成所有硬件依赖与 FreeRTOS 资源的就绪，
  *          随后启动多个并行任务实现手表的完整功能。
  *
  *          【任务概览】
  *          - app_task_init             : 初始化任务，负责硬件初始化、创建所有其他
  *                                        任务、创建软件定时器，完成后自我销毁
  *          - app_task_lvgl             : LVGL 主任务，周期调用 lv_task_handler
  *                                        刷新 UI，使用互斥锁保护
  *          - app_task_app_task_max30102: 心率/血氧采集任务，使用 MAX30102
  *                                        传感器采样并通过算法计算心率与 SpO2
  *          - app_task_rtc              : RTC 时间日期显示任务，依赖事件组唤醒
  *          - app_task_dht11            : DHT11 温湿度采集任务，5s 周期更新
  *          - app_task_motor            : 震动马达控制任务，消费马达消息队列
  *          - app_task_rtc_alarm        : 闹钟任务，依赖事件组唤醒，触发马达震动
  *          - app_task_ble              : 蓝牙任务，解析串口指令并通过 strtok/atoi
  *                                        设置 RTC 时间/日期/闹钟
  *          - app_task_icon             : 主屏状态图标显示任务（闹钟/久坐/蓝牙）
  *          - app_task_mpu6050          : MPU6050 任务，实现抬腕亮屏与计步显示
  *          - app_task_sedentary_remind : 久坐提醒任务，对比步数变化触发震动
  *          - app_task_step_clear        : 跨日步数清零任务
  *          - app_task_istouch_lcd      : 触摸唤醒任务，检测触摸并点亮屏幕
  *
  *          【依赖硬件模块】
  *          STM32 标准外设：NVIC、SysTick、USART1、TIM3(PWM)、RTC(含闹钟A)
  *          外部传感器      ：MAX30102(心率血氧)、MPU6050(DMP计步/欧拉角)、
  *                          DHT11(温湿度)、BT24(蓝牙)
  *          人机交互        ：LCD(亮度可调PWM)、触摸屏、震动马达
  *          GUI 框架        ：LVGL v8
  *
  *          【FreeRTOS 资源设计思路】
  *          1. 互斥锁(Mutex)
  *             - g_mutex_printf_handle  : 保护 printf，避免多任务同时打印造成
  *                                       串口输出交错，封装在 dgb_printf_safe
  *             - g_mutex_sprintf_handle : 保护 sprintf，封装在 dgb_sprintf_safe
  *             - g_mutex_lvgl_handle    : 保护 LVGL API 调用，所有对 lv_xxx 控件
  *                                       的访问必须先获取此锁，避免 LVGL 内部
  *                                       数据结构被并发破坏
  *          2. 事件标志组(EventGroup)
  *             - g_event_group_handle   : 用于中断/任务向其他任务发送事件
  *             - 位 EVENT_GROUPS_RTC_WAKEUP : RTC 唤醒事件，触发时间显示任务刷新
  *             - 位 EVENT_GROUPS_RTC_ALARM : 闹钟触发事件，触发马达震动
  *          3. 消息队列(Queue)
  *             - g_queue_motor_handle : 马达控制队列，5 个元素，每个 8 字节，
  *                                     生产者(闹钟/久坐)发送 motor_t 结构，
  *                                     消费者 app_task_motor 接收执行
  *             - g_queue_bt24_handle  : 蓝牙串口数据队列，10 个元素，每个 32 字节，
  *                                     串口中断将接收到的字符串送入队列，
  *                                     app_task_ble 取出解析
  *          4. 计数信号量(Semaphore)
  *             - g_sem_motor_handle : 计数信号量(初值0)，用于马达任务与
  *                                  闹钟/久坐任务之间的同步，确保一次震动
  *                                  完成后才允许下一次震动请求
  *          5. 软件定时器(Timer)
  *             - soft_timer_handle : 周期 1s 自动重装定时器，用于实现
  *                                  自动熄屏计数和无操作时长统计
  *
  *          【任务间同步关系】
  *          - 闹钟/久坐任务 --(队列)--> 马达任务 --(信号量)--> 闹钟/久坐任务
  *            形成请求-执行-应答闭环，保证震动顺序执行
  *          - 串口中断 --(队列)--> 蓝牙任务 --(RTC寄存器)--> 系统时间更新
  *          - RTC中断 --(事件组)--> 时间任务/闹钟任务
  *          - 触摸任务/MPU6050任务 --(全局变量+临界区)--> 屏幕状态/电源状态
  *
  ******************************************************************************
  */

#include "includes.h"

/* 任务句柄：保存所有任务的句柄，便于挂起/恢复/删除等管理操作 */ 
TaskHandle_t app_task_init_handle             = NULL; /* 初始化任务句柄(运行后销毁) */
TaskHandle_t app_task_lvgl_handle             = NULL; /* LVGL 主任务句柄 */
TaskHandle_t app_task_max30102_handle         = NULL; /* MAX30102 心率血氧任务句柄 */
TaskHandle_t app_task_rtc_handle              = NULL; /* RTC 时间显示任务句柄 */
TaskHandle_t app_task_dht11_handle            = NULL; /* DHT11 温湿度任务句柄 */
TaskHandle_t app_task_motor_handle            = NULL; /* 震动马达任务句柄 */
TaskHandle_t app_task_rtc_alarm_handlde       = NULL; /* 闹钟震动任务句柄(拼写沿用原代码) */
TaskHandle_t app_task_ble_handlde             = NULL; /* 蓝牙指令解析任务句柄 */
TaskHandle_t app_task_icon_handlde            = NULL; /* 主屏状态图标显示任务句柄 */
TaskHandle_t app_task_mpu6050_handlde         = NULL; /* MPU6050 抬腕/计步任务句柄 */
TaskHandle_t app_task_sedentary_remind_handle = NULL; /* 久坐提醒任务句柄 */
TaskHandle_t app_task_step_clear_handle       = NULL; /* 步数跨日清零任务句柄 */
TaskHandle_t app_task_istouch_lcd_handlde     = NULL; /* 触摸唤醒任务句柄 */

/* 任务函数声明：所有任务统一签名 void (*)(void *)，便于 xTaskCreate 调用 */ 
static void app_task_init(void *pvParameters);
static void app_task_lvgl(void *pvParameters);
static void app_task_app_task_max30102(void *pvParameters);
static void app_task_rtc(void *pvParameters);
static void app_task_dht11(void *pvParameters);
static void app_task_motor(void *pvParameters);
static void app_task_rtc_alarm(void *pvParameters);
static void app_task_ble(void *pvParameters);
static void app_task_icon(void *pvParameters);
static void app_task_mpu6050(void *pvParameters);
static void app_task_sedentary_remind(void *pvParameters);
static void app_task_step_clear(void *pvParameters);
static void app_task_istouch_lcd(void *pvParameters);
static void timer_call_back(TimerHandle_t xTimer); /* 软件定时器回调函数声明 */

/* 互斥锁句柄：用于保护共享资源(printf/sprintf/LVGL)的并发访问 */
SemaphoreHandle_t g_mutex_printf_handle  = NULL; /* printf 互斥锁 */
SemaphoreHandle_t g_mutex_sprintf_handle = NULL; /* sprintf 互斥锁 */
SemaphoreHandle_t g_mutex_lvgl_handle    = NULL; /* LVGL API 互斥锁 */

/* 事件标志组句柄：用于 RTC 唤醒/闹钟事件通知 */
EventGroupHandle_t g_event_group_handle = NULL;

/* 消息队列句柄：用于任务间数据传递(马达/蓝牙) */
QueueHandle_t  g_queue_motor_handle = NULL; /* 马达控制消息队列 */
QueueHandle_t  g_queue_bt24_handle  = NULL; /* 蓝牙串口数据消息队列 */

/* 信号量句柄：用于马达任务与请求者之间的同步应答 */
SemaphoreHandle_t g_sem_motor_handle = NULL;

/* 软件定时器句柄：周期 1s 触发自动熄屏判断 */
static TimerHandle_t soft_timer_handle = NULL;

/* 全局变量：保存系统运行状态，多处任务通过临界区访问以避免竞争 */
volatile uint32_t g_rtc_alarm_switch = 0;  //闹钟开关，默认关
volatile uint32_t g_sedentary_switch = 0;  //久坐提醒开关，默认关
volatile uint32_t g_raise_wrist_turn_on_lcd_switch = 0; //抬腕亮屏开关，默认关
volatile uint32_t g_lcd_display = 1; //lcd是否打开，默认打开
volatile uint32_t g_lcd_brightness = 100;//lcd亮度，默认100
volatile uint32_t g_lcd_self_turn_off_switch = 0;//自动熄屏开关，默认关
volatile uint32_t g_system_no_opreation_cnt = 0; //系统无操作计数值，默认0
volatile uint32_t g_cpu_status = CPU_MODE_RUN;  //单片机工作状态，默认运行

/* 任务创建结构体数组：app_task_init 中循环遍历此表批量创建任务，
   最后一项 {0,0,0,0,0,0} 为哨兵项，表示任务表结束 */ 
static const task_t task_tab[]=
{
	{app_task_lvgl,					"app_task_lvgl",				2048, NULL,  6, &app_task_lvgl_handle      			}, /* LVGL 主任务，优先级6，2K 栈 */
	{app_task_app_task_max30102,	"app_task_app_task_max30102", 	2048, NULL,  7, &app_task_max30102_handle  			}, /* 心率血氧任务，优先级7(最高业务优先级) */
	{app_task_rtc,					"app_task_rtc",					512,  NULL,  5, &app_task_rtc_handle       			}, /* RTC 时间显示任务 */
	{app_task_dht11,				"app_task_dht11",				512,  NULL,  6, &app_task_dht11_handle     			}, /* 温湿度采集任务 */
	{app_task_motor,				"app_task_motor",				128,  NULL,  5, &app_task_motor_handle     			}, /* 马达震动任务 */
	{app_task_rtc_alarm,			"app_task_rtc_alarm",			128,  NULL,  5, &app_task_rtc_alarm_handlde			}, /* 闹钟震动任务 */
	{app_task_ble,					"app_task_ble",					512,  NULL,  5, &app_task_ble_handlde      			}, /* 蓝牙指令解析任务 */
	{app_task_icon,					"app_task_icon",				128,  NULL,  5, &app_task_icon_handlde     			}, /* 状态图标显示任务 */
	{app_task_mpu6050, 				"app_task_mpu6050", 			512,  NULL,  5, &app_task_mpu6050_handlde  			}, /* 抬腕亮屏/计步任务 */
	{app_task_sedentary_remind, 	"app_task_sedentary_remind",	512,  NULL,  5, &app_task_sedentary_remind_handle	}, /* 久坐提醒任务 */
	{app_task_step_clear, 	        "app_task_step_clear",	        128,  NULL,  5, &app_task_step_clear_handle      	}, /* 步数清零任务 */
	{app_task_istouch_lcd, 			"app_task_istouch_lcd",			128,  NULL,  6, &app_task_istouch_lcd_handlde  		}, /* 触摸唤醒任务 */
	{0,0,0,0,0,0},            /* 表结束哨兵，循环遇到此项即停止创建 */
};

/**
  * @brief  互斥锁封装的 printf 安全打印函数
  * @param  format  printf 风格的格式化字符串
  * @param  ...     可变参数列表
  * @retval None
  * @note   在多任务环境下，若直接调用 printf 会导致多个任务同时向串口
  *         写入，造成输出内容交错混乱。本函数通过获取 g_mutex_printf_handle
  *         互斥锁，保证任意时刻只有一个任务在打印。portMAX_DELAY 表示
  *         阻塞等待直到获取成功。
  */
/* 互斥锁封装printf */
void dgb_printf_safe(const char *format, ...)
{
	va_list args;                            /* 可变参数列表变量 */
	va_start(args, format);                  /* 初始化可变参数列表 */

	/* 获取互斥信号量：阻塞等待直到拿到锁，确保串口输出原子性 */
	xSemaphoreTake(g_mutex_printf_handle, portMAX_DELAY);

	vprintf(format, args);                   /* 实际执行格式化打印到串口 */

	/* 释放互斥信号量：允许其他等待打印的任务继续 */
	xSemaphoreGive(g_mutex_printf_handle);

	va_end(args);                            /* 结束可变参数处理 */

}

/**
  * @brief  互斥锁封装的 sprintf 安全格式化函数
  * @param  str    输出字符串缓冲区
  * @param  format printf 风格的格式化字符串
  * @param  ...    可变参数列表
  * @retval None
  * @note   在多任务环境下，vsprintf 不是可重入函数(依赖全局缓冲区)，
  *         多个任务同时调用会导致缓冲区数据相互覆盖。本函数使用
  *         g_mutex_sprintf_handle 互斥锁串行化 sprintf 调用，避免数据冲突。
  */
/* 互斥锁封装sprintf */
void dgb_sprintf_safe(char *str,const char *format, ...)
{
	va_list args;                            /* 可变参数列表变量 */
	va_start(args, format);                  /* 初始化可变参数列表 */

	/* 获取互斥信号量：阻塞等待直到拿到锁 */
	xSemaphoreTake(g_mutex_sprintf_handle, portMAX_DELAY);

	vsprintf(str,format, args);              /* 实际执行格式化到 str 缓冲 */

	/* 释放互斥信号量 */
	xSemaphoreGive(g_mutex_sprintf_handle);

	va_end(args);                            /* 结束可变参数处理 */

}

/**
  * @brief  主函数，系统启动入口
  * @retval int 程序不会返回，FreeRTOS 调度器接管
  * @note   执行顺序：NVIC 中断分组 -> SysTick 配置 -> 串口初始化 ->
  *         创建初始化任务 -> 启动调度器。所有其他任务由 app_task_init
  *         在运行时批量创建。
  */
// 主函数
int main(void)
{
	BaseType_t ret = 0;                      /* xTaskCreate 返回值，用于判断创建是否成功 */
	
	/* 中断分组4：4 位全用作抢占优先级，无子优先级，FreeRTOS 推荐配置 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	/* 1ms的滴答定时器：配置 SysTick 周期为 configTICK_RATE_HZ 对应的时钟分频 */
	SysTick_Config(SystemCoreClock/configTICK_RATE_HZ);	
	
	/* 串口1初始化 波特率为115200bps：用于 printf 调试输出 */
	usart1_init(115200);
	
	/* 创建初始化任务：该任务负责初始化所有外设并创建其他业务任务 */
	ret = xTaskCreate((TaskFunction_t )app_task_init,  		        /* 任务函数 */
			          (const char*    )"app_task_init",		        /* 任务名称 */
			          (uint16_t       )2048,  				        /* 任务堆栈大小 */
			          (void*          )NULL,			            /* 任务函数形参 */
			          (UBaseType_t    )6, 					        /* 任务优先级 */
			          (TaskHandle_t*  )&app_task_init_handle);	    /* 任务句柄 */ 

	if(ret != pdPASS)                       /* 创建任务失败 */
	{
		printf("app_task_init Create Error\r\n"); /* 输出错误信息 */
		while(1);                            /* 死循环，等待看门狗或手动复位 */
	}
	
	/* 开启任务调度器：此后 FreeRTOS 接管 CPU，初始化任务开始执行 */
	vTaskStartScheduler(); 
	
}

/**
  * @brief  初始化任务，初始化所有硬件设备，创建其他任务
  * @param  pvParameters 任务函数形参(本任务未使用)
  * @retval None
  * @note   该任务是 main() 中创建的唯一初始任务，运行优先级为 6。
  *         执行流程：先创建 FreeRTOS 同步原语(互斥锁/事件组/队列/信号量)，
  *         再初始化各硬件外设(传感器/LCD/蓝牙/RTC/DMP)，然后在临界区中
  *         批量创建所有业务任务并启动软件定时器，最后调用 vTaskDelete
  *         销毁自身释放栈空间。使用临界区保证任务创建过程的原子性，
  *         避免任务在被创建过程中被高优先级任务抢占导致句柄未就绪。
  */
static void app_task_init(void *pvParameters)
{
	uint32_t i = 0;                         /* 任务表循环计数器 */

	BaseType_t ret = 0;                     /* xTaskCreate/xTimerCreate 返回值 */

	printf("[app_task_init]:app_task_init...\r\n"); /* 打印启动信息(此时互斥锁未创建，可直接用 printf) */

	/* 创建互斥锁：3 个互斥锁分别保护 printf/sprintf/LVGL 的并发访问 */
	g_mutex_printf_handle  = xSemaphoreCreateMutex(); /* printf 串口打印互斥锁 */
	g_mutex_sprintf_handle = xSemaphoreCreateMutex(); /* sprintf 格式化互斥锁 */
	g_mutex_lvgl_handle    = xSemaphoreCreateMutex(); /* LVGL API 调用互斥锁 */

	/* 创建事件标志组：用于 RTC 唤醒事件与闹钟事件通知 */
	g_event_group_handle = xEventGroupCreate();

	/* 创建消息队列：队列大小(项数, 每项字节数) */
	g_queue_motor_handle = xQueueCreate(5,8);   /* 马达队列：5 项 * 8 字节(匹配 motor_t) */
	g_queue_bt24_handle  = xQueueCreate(10,32); /* 蓝牙队列：10 项 * 32 字节(完整指令字符串) */
	
	/* 创建信号量：计数信号量，最大计数1，初始值0
	   初始0表示马达未执行任何震动，请求者首次必须等待马达执行后释放 */
	g_sem_motor_handle = xSemaphoreCreateCounting(1,0);

	/* TIM3 PWM初始化：用于 LCD 亮度调节(背光 PWM) */
	tim3_init();

	/* 初始化lvgl：LVGL 核心初始化 */
	lv_init();
	
	/* 初始化lvgl显示设备：注册 LCD 驱动到 LVGL 显示缓冲 */
	lv_port_disp_init();

	/* 初始化lvgl输入设备：注册触摸屏驱动到 LVGL 输入设备 */
	lv_port_indev_init();
	
	/* lvgl界面初始化：构建 UI 控件树(主屏/心率/温湿度等) */
	ui_init();

	/* 蓝牙模块初始化：BT24 模块串口波特率 9600bps */
	bt24_init(9600);

	/* 震动模块初始化：马达 GPIO 配置 */
	vibration_motor_init();

	/* rtc初始化：STM32 内部 RTC 寄存器配置 */
	my_rtc_init();

	/* 闹钟初始化：RTC 闹钟 A 中断配置 */
	myrtc_alarm_init();
	
	/* DHT11初始化：温湿度传感器引脚初始化 */
	dht11_init();	

	/* MAX30102初始化：心率血氧传感器 I2C 与寄存器配置 */
	max30102_init();

	/* mpu6050初始化：MPU6050 基础 I2C 通信初始化 */
	MPU_Init();

	/* mpu6050dmp初始化：DMP 固件加载失败则重试，直到成功 */
	while(mpu_dmp_init())
	{
		printf("mpu6050 dmp init... \r\n");
		vTaskDelay(500);                     /* 500 tick 后重试，避免死循环占满 CPU */
	}

	/* 设置步数初值为0：清空 DMP 计步器寄存器 */
	while(dmp_set_pedometer_step_count(0))
	{
		printf("mpu6050 step init... \r\n");
		vTaskDelay(500);
	}

	/* 进入临界区：屏蔽任务切换与中断，保证批量任务创建原子性。
	   避免在创建任务表过程中，已创建任务提前被调度而引用到未就绪
	   的句柄或全局资源 */
	taskENTER_CRITICAL();
	
	for(i = 0; task_tab[i].pcName != 0; i++)  /* 遍历任务表直到哨兵项(任务名为0) */
	{
		/* 创建任务：从 task_tab 表读取参数调用 xTaskCreate */
		ret = xTaskCreate((TaskFunction_t )task_tab[i].pxTaskCode,      /* 任务函数 */
						  (const char*    )task_tab[i].pcName,		    /* 任务名称 */
						  (uint16_t       )task_tab[i].usStackDepth,    /* 任务堆栈大小 */
						  (void*          )task_tab[i].pvParameters,	/* 任务函数形参 */
						  (UBaseType_t    )task_tab[i].uxPriority, 		/* 任务优先级 */
						  (TaskHandle_t*  )task_tab[i].pxCreatedTask);	/* 任务句柄 */ 

		if(ret != pdPASS)                   /* 创建任务失败 */
		{
			printf("[app_task_init]:%s Create Error\r\n",task_tab[i].pcName);
			while(1);                        /* 死循环挂起系统，便于定位错误 */
		}
	}

	/* 创建一个软件定时器：周期 1000 tick(即 1s)自动重装，用于自动熄屏判定 */
	soft_timer_handle = xTimerCreate((const char*            )"soft_timer",         /* 软件定时器名称 */
                 				     (const TickType_t       )1000,					/* 定时周期 1000 tick = 1s */
                                     (const UBaseType_t      )pdTRUE,				/* pdTRUE 表示周期性自动重装 */
                                     (void * const           ) 1,					/* 定时器ID(可在回调中区分) */
                                     (TimerCallbackFunction_t)timer_call_back );    /* 定时器回调函数 */

	/* 开启软件定时器：参数 0 表示不等待，立即发送启动命令到定时器服务任务 */
	xTimerStart(soft_timer_handle,0);

	/* 销毁自己：参数 NULL 表示删除当前任务，释放该任务 TCB 与栈空间，
	   此处位于临界区，但因任务已删除不会执行后续的 taskEXIT_CRITICAL() */
	vTaskDelete(NULL);

	/* 退出临界区：因 vTaskDelete 已删除当前任务，此句实际上不会执行，
	   保留是为代码完整性 */
	taskEXIT_CRITICAL();
}

/**
  * @brief  LVGL 主任务，周期调用 lv_task_handler 刷新 UI
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：每 5 tick 调用一次 lv_task_handler 处理 LVGL 内部定时任务
  *         (动画/事件分发/重绘)。同步关系：调用前获取 g_mutex_lvgl_handle
  *         互斥锁，确保 LVGL 内部数据不被其他任务并发访问破坏；调用后
  *         释放锁。其他任务访问 lv_xxx 控件时也必须先获取同一把锁。
  */
static void app_task_lvgl(void *pvParameters)
{
	dgb_printf_safe("[app_task_lvgl]:app_task_lvgl start running...\r\n"); /* 任务启动日志 */
	
	for(;;)                                  /* 任务主循环(无限循环) */
	{
		/* 加锁：阻塞等待 LVGL 互斥锁，确保独占访问 LVGL 内部数据结构 */
		xSemaphoreTake(g_mutex_lvgl_handle,portMAX_DELAY);

		/* lvgl任务处理：执行 LVGL 内部定时任务(动画/事件/重绘等) */
		lv_task_handler();
		
		/* 解锁：允许其他任务访问 LVGL 控件 */
		xSemaphoreGive(g_mutex_lvgl_handle);

		vTaskDelay(5);                       /* 主动让出 CPU 5 tick，避免抢占过多 */
	}
}

/**
  * @brief  心率血氧任务，采集心率和血氧，输出到lvgl屏幕上
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：任务启动后立即挂起(vTaskSuspend)，由外部(如菜单按钮)
  *         恢复后开始采集。每次采集 100 个新样本并维持 500 个滑动窗口样本，
  *         通过 maxim_heart_rate_and_oxygen_saturation 算法计算心率与 SpO2，
  *         通过范围(60-100bpm / 95-100%)过滤异常值后刷新 LVGL 控件。
  *         LVGL 控件访问未单独加锁(可考虑后续优化使用 g_mutex_lvgl_handle)。
  */
static void app_task_app_task_max30102(void *pvParameters)
{
	uint32_t aun_ir_buffer[500];   //红外光数据
	int32_t n_ir_buffer_length;    //数据长度
	uint32_t aun_red_buffer[500];  //红光数据
	int32_t n_sp02;          //血氧
	int8_t ch_spo2_valid;   //血氧是否有效
	int32_t n_heart_rate;   //心率
	int8_t  ch_hr_valid;    //心率是否有效
	uint32_t un_min, un_max; //最大值和最小值
	int32_t i;
	uint8_t temp[6];

	dgb_printf_safe("[app_task_app_task_max30102]:app_task_app_task_max30102 start running...\r\n"); /* 任务启动日志 */

	/* 挂起自己：等待外部通过 vTaskResume 恢复后才进入采集循环 */
	vTaskSuspend(NULL);

	un_min=0x3FFFF;                          /* 信号最小值初始化为大值 */
	un_max=0;                                /* 信号最大值初始化为 0 */
	
	n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
	//read the first 500 samples, and determine the signal range
	
    for(i=0;i<n_ir_buffer_length;i++)        /* 首次采集 500 个样本(5 秒@100sps) */
    {
        while(MAX30102_INT==1)  //有数据拉低引脚
		{
			vTaskDelay(5);                  /* 等待 INT 引脚拉低(数据就绪)，5 tick 重试 */
		}
	
		max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp); /* 从 FIFO 读取 6 字节数据(红光+红外) */
	
		aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
		aun_ir_buffer[i] =   (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
            
        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //update signal min
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //update signal max
    }
	
	//un_prev_data=aun_red_buffer[i];
	//calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)

    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer,  /* 第一次心率血氧计算 */
										   n_ir_buffer_length,
										   aun_red_buffer,
										   &n_sp02,
										   &ch_spo2_valid,
										   &n_heart_rate,
										   &ch_hr_valid);

	for(;;)                                  /* 主循环：周期性采集与计算 */
	{
		un_min=0x3FFFF;                      /* 重置最大最小值 */
        un_max=0;
		n_ir_buffer_length=500;              /* 保留 500 样本窗口 */
		
		/* dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top

		   将前100组样本转储到存储器中，并将最后400组样本移到顶部
		*/

        for(i=100;i<500;i++)                 /* 滑动窗口：丢弃前 100 个旧样本，保留后 400 个 */
        {
            aun_red_buffer[i-100]=aun_red_buffer[i]; /* 红光数据前移 100 位 */
            aun_ir_buffer[i-100]=aun_ir_buffer[i];   /* 红外光数据前移 100 位 */

            /* update the signal min and max
			   更新信号最小值和最大值
			*/

            if(un_min>aun_red_buffer[i])
				un_min=aun_red_buffer[i];

            if(un_max<aun_red_buffer[i])
				un_max=aun_red_buffer[i];
        }

		/* take 100 sets of samples before calculating the heart rate

		   在计算心率之前采集100组样本
		*/

        for(i=400;i<500;i++)                  /* 采集新 100 个样本填入窗口尾部 */
        {
			while(MAX30102_INT==1)            /* 等待 INT 引脚拉低表示数据就绪 */
			{
				vTaskDelay(5);
			}

            max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp); /* 读取 FIFO 数据 */
			
			/* 组合值以获得实际数字 */
			aun_red_buffer[i] =  ((temp[0]&0x03)<<16) |(temp[1]<<8) | temp[2];
			aun_ir_buffer[i] =   ((temp[3]&0x03)<<16) |(temp[4]<<8) | temp[5];
	    }


		/* 计算心率和血氧饱和度 */
		maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer,
											   n_ir_buffer_length,
											   aun_red_buffer,
											   &n_sp02,
											   &ch_spo2_valid,
											   &n_heart_rate,
											   &ch_hr_valid);

		if((ch_hr_valid == 1) && (n_heart_rate >= 60) && (n_heart_rate <= 100))  /* 心率有效且在合理范围 */
		{
			/* 心率血氧测量屏幕更新显示心率数据 */
			lv_slider_set_value(ui_SliderHR,n_heart_rate,LV_ANIM_OFF); /* 设置心率滑块 */
			lv_label_set_text_fmt(ui_LabelHR,"%d",n_heart_rate);        /* 设置心率数值标签 */

			/* 主屏幕更新显示心率数据 */
			lv_arc_set_value(ui_ArcHeartRate,n_heart_rate);             /* 设置主屏心率圆弧 */
			lv_label_set_text_fmt(ui_LabelHeartRate,"%d",n_heart_rate); /* 设置主屏心率标签 */

			dgb_printf_safe("[app_task_app_task_max30102]:n_heart_rate = %d\r\n", n_heart_rate);
		}


		if((ch_spo2_valid == 1) && (n_sp02 >= 95) && (n_sp02 <= 100))  /* 血氧有效且在合理范围 */
		{
			/* 心率血氧测量屏幕更新显示血氧数据 */
			lv_slider_set_value(ui_SliderSPO2H,n_sp02,LV_ANIM_OFF);    /* 设置血氧滑块 */
			lv_label_set_text_fmt(ui_LabelSPO2H,"%d",n_sp02);          /* 设置血氧数值标签 */

			/* 主屏幕更新显示血氧数据 */
			lv_arc_set_value(ui_ArcBloodOxygen,n_sp02);                /* 设置主屏血氧圆弧 */
			lv_label_set_text_fmt(ui_LabelBloodOxygen,"%d",n_sp02);    /* 设置主屏血氧标签 */

			dgb_printf_safe("[app_task_app_task_max30102]:n_sp02 = %d\r\n", n_sp02);
		}

		vTaskDelay(1000);                    /* 1s 后再次采集 */
	}
}

/**
  * @brief  rtc任务，获取时间日期，输出到lvgl屏幕
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：阻塞等待事件组 EVENT_GROUPS_RTC_WAKEUP 标志位置位
  *         (由 RTC 唤醒中断触发)。事件置位后自动清除(pdTRUE)。
  *         同步关系：读取时间日期后获取 g_mutex_lvgl_handle 互斥锁，
  *         才能更新 LVGL 标签控件。
  */
static void app_task_rtc(void *pvParameters)
{
	RTC_DateTypeDef RTC_DateStruct;          /* RTC 日期结构体 */
	RTC_TimeTypeDef RTC_TimeStruct;          /* RTC 时间结构体 */

	dgb_printf_safe("[app_task_rtc]:app_task_rtc start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		/* 阻塞等待 RTC 唤醒事件位置位，置位后自动清除标志 */
		xEventGroupWaitBits(g_event_group_handle,          /* 事件标志组句柄 */
		                    EVENT_GROUPS_RTC_WAKEUP,       /* 等待置位的标志位 */
							pdTRUE,                        /* pdTRUE: 等待到时自动清除标志 */
							pdTRUE,						   /* pdTRUE: 所有指定标志都置位才返回(与逻辑) */
						    portMAX_DELAY                  /* portMAX_DELAY: 一直等待，永不超时 */
		                    );

		RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);//获取时间,必须先读取时间，否则修改日期会有问题
		RTC_GetDate(RTC_Format_BIN,&RTC_DateStruct);//获取日期
																					  										  
		/* 加锁：阻塞等待 LVGL 互斥锁，确保独占访问 LVGL 控件 */
		xSemaphoreTake(g_mutex_lvgl_handle,portMAX_DELAY);

		lv_label_set_text_fmt(ui_LabelDate,"20%d/%02d/%02d",RTC_DateStruct.RTC_Year, /* 格式化日期字符串 */
															RTC_DateStruct.RTC_Month,
															RTC_DateStruct.RTC_Date
															);

		lv_label_set_text_fmt(ui_LabelHour,"%02d",RTC_TimeStruct.RTC_Hours); /* 设置小时标签 */

		lv_label_set_text_fmt(ui_LabelMin,"%02d",RTC_TimeStruct.RTC_Minutes); /* 设置分钟标签 */

		switch (RTC_DateStruct.RTC_WeekDay)   /* 根据星期值显示对应英文缩写 */
		{
			case RTC_Weekday_Monday:
					lv_label_set_text(ui_LabelWeek,"Mon"); /* 星期一 */
				break;
			case RTC_Weekday_Tuesday:
					lv_label_set_text(ui_LabelWeek,"Tue"); /* 星期二 */
				break;
			case RTC_Weekday_Wednesday:
					lv_label_set_text(ui_LabelWeek,"Wed"); /* 星期三 */
				break;
			case RTC_Weekday_Thursday:
					lv_label_set_text(ui_LabelWeek,"Thu"); /* 星期四 */
				break;
			case RTC_Weekday_Friday:
					lv_label_set_text(ui_LabelWeek,"Fri"); /* 星期五 */
				break;
			case RTC_Weekday_Saturday:
					lv_label_set_text(ui_LabelWeek,"Sat"); /* 星期六 */
				break;
			case RTC_Weekday_Sunday:
					lv_label_set_text(ui_LabelWeek,"Sun"); /* 星期日 */
				break;
			default:
				break;                         /* 未知星期值不处理 */
		}

		/* 解锁：释放 LVGL 互斥锁 */
		xSemaphoreGive(g_mutex_lvgl_handle);

	}
}

/**
  * @brief  温湿度任务，将收集到的温湿度数据输出到lvgl屏幕上
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：每 5 秒周期采样一次 DHT11。同步关系：将温湿度数据格式化
  *         到缓冲区(使用 dgb_sprintf_safe 防止 sprintf 冲突)后，获取 LVGL
  *         互斥锁再更新温湿度标签与圆弧控件。
  */
static void app_task_dht11(void *pvParameters)
{
	/* 温湿度数据 */
	float tem = 0.0;
	uint8_t hum = 0;
	uint8_t buf1[32] = {0};                  /* 温度字符串缓冲 */
	uint8_t buf2[32] = {0};                  /* 湿度字符串缓冲 */

	dgb_printf_safe("[app_task_dht11]:app_task_dht11 start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		if(!dht11_get_tem_hum(&tem,&hum))    /* 读取 DHT11 温湿度，返回0表示成功 */
		{
			memset(buf1,0,sizeof(buf1));     /* 清空温度缓冲 */
			memset(buf2,0,sizeof(buf2));     /* 清空湿度缓冲 */

			dgb_sprintf_safe((char *)buf1,"%.1f",tem); /* 格式化温度(1 位小数) */
			dgb_sprintf_safe((char *)buf2,"%d",hum);    /* 格式化湿度(整数) */

			/* 加锁：阻塞等待 LVGL 互斥锁 */
			xSemaphoreTake(g_mutex_lvgl_handle,portMAX_DELAY);

			lv_label_set_text(ui_LabelTem,buf1); /* 设置温度标签 */
			lv_label_set_text(ui_LabelHum,buf2); /* 设置湿度标签 */

			lv_arc_set_value(ui_ArcTem, (int)tem); /* 设置温度圆弧 */
			lv_arc_set_value(ui_ArcHum, hum);       /* 设置湿度圆弧 */

			/* 解锁：释放 LVGL 互斥锁 */
			xSemaphoreGive(g_mutex_lvgl_handle);

			dgb_printf_safe("[app_task_dht11]:dht data:%s  %s\r\n",buf1,buf2); /* 打印温湿度 */
		}
		vTaskDelay(5000);                    /* 5 秒后再次采样 */
	}
}

/**
  * @brief  震动提醒任务，接收到其他任务的不同请求，震动提醒使用者
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：阻塞等待马达消息队列 g_queue_motor_handle，收到 motor_t
  *         结构后驱动 GPIO 震动指定时长。同步关系：震动完成后通过计数信号量
  *         g_sem_motor_handle 通知请求者(闹钟/久坐任务)，形成请求-执行-应答
  *         闭环，确保震动顺序执行不被覆盖。
  */
static void app_task_motor(void *pvParameters)
{
	motor_t motor;                           /* 马达控制结构(状态+时长) */
	BaseType_t ret = 0;                       /* 队列接收返回值 */

	dgb_printf_safe("[app_task_motor]:app_task_motor start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		/* 等待消息队列：阻塞等待 motor_t 结构，portMAX_DELAY 永久等待 */
		ret = xQueueReceive(g_queue_motor_handle,&motor,portMAX_DELAY);
		if(ret == pdFALSE)                   /* 接收失败 */
		{
			dgb_printf_safe("[app_task_motor]:app_task_motor receive queue error\r\n");
		}

		/* 震动状态有效：sta 非 0 才执行震动 */
		if(motor.sta)
		{
			VIBRATION_MOTOR(1);              /* 马达开启(置 GPIO 高) */

			vTaskDelay(motor.time);          /* 延时 motor.time 毫秒(震动时长) */

			VIBRATION_MOTOR(0);              /* 马达关闭 */

			motor.sta = 0;                   /* 清除状态标志 */

			/* 这一次震动完成，告诉别的任务可以执行下一次震动：释放计数信号量，
			   解除请求者(闹钟/久坐任务)在 xSemaphoreTake 上的阻塞 */
			xSemaphoreGive(g_sem_motor_handle);
		}
	}
}

/**
  * @brief  闹钟任务，接收到闹钟中断请求之后向震动任务发出震动要求
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：阻塞等待事件组 EVENT_GROUPS_RTC_ALARM 标志位置位
  *         (由 RTC 闹钟 A 中断触发)。同步关系：触发后向马达队列发送5次
  *         motor_t 震动请求，每次发送后阻塞等待 g_sem_motor_handle 信号量，
  *         确保上一次震动完成后再发起下一次，避免请求堆积。
  */
static void app_task_rtc_alarm(void *pvParameters)
{
	motor_t motor = {0};                     /* 马达请求结构(状态+时长) */
	uint8_t cnt = 5;                         /* 震动次数计数器 */
	BaseType_t ret = pdFALSE;                /* 队列发送返回值 */

	dgb_printf_safe("[app_task_rtc_alarm]:app_task_rtc_alarm start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		/* 等待设定闹钟触发：阻塞直到 EVENT_GROUPS_RTC_ALARM 事件置位，自动清除 */
		xEventGroupWaitBits(g_event_group_handle,          /* 事件标志组句柄 */
		                    EVENT_GROUPS_RTC_ALARM,        /* 等待置位的标志位 */
							pdTRUE,                        /* pdTRUE: 自动清除标志 */
							pdTRUE,						   /* pdTRUE: 与逻辑(全部置位才返回) */
						    portMAX_DELAY                  /* portMAX_DELAY: 一直等待 */
		                    );
		dgb_printf_safe("[app_task_rtc_alarm]:alarm is ringing\r\n"); /* 闹钟触发日志 */

		/* 震动模块震动5次：循环发送5次震动请求 */
		while(cnt--)
		{
			motor.sta = 1;                   /* 设置震动状态有效 */
			motor.time = 1000;               /* 设置震动时长 1000ms */

			ret = xQueueSend(g_queue_motor_handle,&motor,100); /* 发送马达请求(超时100 tick) */
			if(ret == pdFALSE)               /* 发送失败 */
			{
				dgb_printf_safe("[app_task_rtc_alarm]:app_task_rtc_alarm send queue error\r\n");
			}

			/* 等待震动模块执行完毕，避免没执行完又发出震动指令 ，同步任务：
			   阻塞等待 g_sem_motor_handle(由马达任务震动完成后释放) */
			xSemaphoreTake(g_sem_motor_handle,portMAX_DELAY);

			vTaskDelay(1000);                /* 两次震动间隔 1s */
		}
		cnt = 5;                             /* 重置震动次数计数器 */
	}
}

/**
  * @brief  蓝牙串口任务，根据不同指令修改时间日期 和 设置闹钟时间
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：阻塞等待蓝牙消息队列 g_queue_bt24_handle(由串口中断填入)，
  *         收到字符串后使用 strstr 判断指令类型，使用 strtok 按 "-" 分隔
  *         字段，atoi 转换为数值，最终调用 RTC_SetTime/RTC_SetDate/RTC_SetAlarm
  *         写入 RTC 寄存器。同步关系：串口中断为生产者，本任务为消费者。
  *         支持指令："TIME SET-HH-MM-SS" / "DATE SET-YYYY-MM-DD-W" / "ALARM SET-HH-MM-SS"
  */
static void app_task_ble(void *pvParameters)
{
	uint8_t buffer[32] = {0};                /* 蓝牙指令缓冲 */
	BaseType_t ret = pdFALSE;                /* 队列接收返回值 */

	char *p = NULL;                          /* strtok 返回的字段指针 */
	RTC_TimeTypeDef RTC_TimeStructure;       /* RTC 时间结构体 */
	RTC_DateTypeDef RTC_DateStructure;       /* RTC 日期结构体 */
	RTC_AlarmTypeDef RTC_AlarmStructure;     /* RTC 闹钟结构体 */

	dgb_printf_safe("[app_task_ble]:app_task_ble start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		/* 等待接收蓝牙串口消息：阻塞等待队列数据，由串口中断产生 */
		ret = xQueueReceive(g_queue_bt24_handle,buffer,portMAX_DELAY);
		if(ret == pdFALSE)                   /* 接收失败 */
		{
			dgb_printf_safe("[app_task_ble]:app_task_ble receivce queue error\r\n");
		}

		dgb_printf_safe("[app_task_ble]:app_task_ble recv:%s\r\n",buffer); /* 打印收到的指令 */

		/* 解析：根据关键字区分三种指令 */
		if(strstr((char *)buffer,"TIME SET") != NULL)//时间
		{
			p = strtok((char *)buffer,"-");  /* 第一次调用 strtok，分割出指令头 "TIME SET" */
			if(p == NULL)
			   goto error;                    /* 分割失败跳转错误处理 */

			/* 时 */
			p = strtok(NULL,"-");             /* 后续调用 strtok 第一参数为 NULL 继续分割 */
			if(p == NULL)
				goto error;

			RTC_TimeStructure.RTC_Hours = atoi(p); /* atoi 转换字符串为整数小时 */

			/* 判断上午还是下午 */
			if(atoi(p) < 12)
				RTC_TimeStructure.RTC_H12 = RTC_H12_AM; /* 小于12设为 AM */
			else
				RTC_TimeStructure.RTC_H12 = RTC_H12_PM; /* 大于等于12设为 PM */

			/* 分 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_TimeStructure.RTC_Minutes = atoi(p); /* 解析分钟 */

			/* 秒 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_TimeStructure.RTC_Seconds = atoi(p); /* 解析秒 */

			/* 设置RTC时间：将解析的时间结构写入 RTC 时间寄存器 */
			RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);

			dgb_printf_safe("[app_task_ble]: rtc set time ok\r\n"); /* 设置成功日志 */
		}
		else if(strstr((char *)buffer,"DATE SET") != NULL)//日期
		{
			p = strtok((char *)buffer,"-");  /* 分割指令头 "DATE SET" */
			if(p == NULL)
			   goto error;

			/* 年 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_DateStructure.RTC_Year = atoi(p)-2000; /* 减2000得到 RTC 相对年份 */

			/* 月 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_DateStructure.RTC_Month = atoi(p); /* 解析月份 */

			/* 日 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_DateStructure.RTC_Date = atoi(p); /* 解析日 */

			/* 星期 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_DateStructure.RTC_WeekDay = atoi(p); /* 解析星期(1-7) */

			/* 设置RTC日期：写入 RTC 日期寄存器 */
			RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);

			dgb_printf_safe("[app_task_ble]: rtc set date ok\r\n");
		}
		else if(strstr((char *)buffer,"ALARM SET") != NULL)
		{
			/* 关闭闹钟：修改前先禁能，避免修改过程触发 */
			RTC_AlarmCmd(RTC_Alarm_A,DISABLE);

			RTC_AlarmStructInit(&RTC_AlarmStructure);//先全部成员赋默认值，避免没有用上的成员没有赋值造成bug

			p = strtok((char *)buffer,"-");  /* 分割指令头 "ALARM SET" */
			if(p == NULL)
			   goto error;

			/* 时 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = atoi(p); /* 闹钟时 */

			/* 判断是下午还是上午 */
			if(atoi(p) < 12)
				RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_H12_AM;
			else
				RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_H12_PM;

			/* 分 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = atoi(p); /* 闹钟分 */

			/* 秒 */
			p = strtok(NULL,"-");
			if(p == NULL)
				goto error;

			RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = atoi(p); /* 闹钟秒 */

			/* 屏蔽日期和星期，每天生效*/
			RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;

			/* 设置RTC闹钟时间：写入 RTC 闹钟 A 寄存器 */
			RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_AlarmStructure);

			/* 判断原来的闹钟状态是关是开：根据全局开关决定是否使能 */
			if(g_rtc_alarm_switch)
				RTC_AlarmCmd(RTC_Alarm_A,ENABLE);   /* 开关打开则使能闹钟 */
			else
				RTC_AlarmCmd(RTC_Alarm_A,DISABLE);  /* 开关关闭则禁能闹钟 */

			dgb_printf_safe("[app_task_ble]: rtc set alarm ok\r\n");
		}
		else
		{
    error:                                  /* 解析失败跳转标签 */
			dgb_printf_safe("[app_task_ble]: command error\r\n"); /* 指令格式错误 */
		}

		/* 清空缓存数组：为下一次接收指令准备 */
		memset(buffer,0,sizeof(buffer));
	}
}

/**
  * @brief  主屏幕右上角图标显示任务
  * @param  pvParameters 任务函数形参(未使用)
  * @retval 不同的功能状态显示或者关闭图标
  * @note   调度策略：每 100ms 检测一次全局开关状态。同步关系：访问 LVGL 控件
  *         前获取 g_mutex_lvgl_handle 互斥锁，通过 lv_obj_clear_flag/add_flag
  *         显示/隐藏闹钟/久坐/蓝牙图标。
  */
static void app_task_icon(void *pvParameters)
{
	dgb_printf_safe("[app_task_icon]:app_task_icon start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		/* 加锁：阻塞等待 LVGL 互斥锁 */
		xSemaphoreTake(g_mutex_lvgl_handle,portMAX_DELAY);

		/* 闹钟图标 */
		if(g_rtc_alarm_switch)                /* 闹钟开关打开 */
			lv_obj_clear_flag(ui_ImageAlarm,LV_OBJ_FLAG_HIDDEN); /* 清除隐藏标志显示图标 */
		else
			lv_obj_add_flag(ui_ImageAlarm, LV_OBJ_FLAG_HIDDEN); /* 添加隐藏标志隐藏图标 */

		/* 久坐提醒图标 */
		if(g_sedentary_switch)                /* 久坐开关打开 */
			lv_obj_clear_flag(ui_ImageLongSit,LV_OBJ_FLAG_HIDDEN);
		else
			lv_obj_add_flag(ui_ImageLongSit, LV_OBJ_FLAG_HIDDEN);

		/* 蓝牙图标 */
		if(bt24_connect_status())             /* 查询蓝牙连接状态 */
			lv_obj_clear_flag(ui_Imageble,LV_OBJ_FLAG_HIDDEN);
		else
			lv_obj_add_flag(ui_Imageble,LV_OBJ_FLAG_HIDDEN);

		/* 解锁：释放 LVGL 互斥锁 */
		xSemaphoreGive(g_mutex_lvgl_handle);

		/* 100ms检测一次 */
		vTaskDelay(100);
	}
}

/**
  * @brief  mpu6050任务，检测角度达到一定条件实现抬腕亮屏，获取步数并输出到lvgl屏幕上
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：每 200ms 读取一次 DMP 欧拉角，若 roll 角超过 15° 且
  *         抬腕亮屏开关打开且屏幕处于熄灭状态，则点亮屏幕。每 5s (200ms*25)
  *         读取一次步数刷新 UI。同步关系：访问全局变量 g_lcd_display/
  *         g_cpu_status/g_system_no_opreation_cnt 时使用临界区保护，
  *         访问 LVGL 控件使用互斥锁保护。
  */
static void app_task_mpu6050(void *pvParameters)
{
	float pitch, roll, yaw; /* 欧拉角 */
	uint32_t step_count = 0; /* 步数 */
	uint32_t time_5s = 0;                     /* 5s 计时计数器 */

	dgb_printf_safe("[app_task_mpu6050]:app_task_mpu6050 start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		vTaskDelay(200);                     /* 200ms 周期 */
		time_5s++;                           /* 计数器+1 */

		if(!mpu_dmp_get_data(&pitch, &roll, &yaw)) /* 读取 DMP 欧拉角，返回0成功 */
		{
			/* 屏幕熄灭实现抬手亮屏：roll>15 且屏幕熄灭 且 抬腕亮屏开关打开 */
			if(roll > 15 && !g_lcd_display && g_raise_wrist_turn_on_lcd_switch)
			{
				lcd_set_brightness(g_lcd_brightness); /* 设置 LCD 亮度 */

				/* 进入临界区：保护对全局状态的修改，避免与软件定时器/触摸任务竞争 */
				taskENTER_CRITICAL();

				/* CPU进入工作状态 */
				g_cpu_status = CPU_MODE_RUN;

				/* 屏幕打开标志置1 */
				g_lcd_display = 1;

				/* 清零系统无操作计数值 */
				g_system_no_opreation_cnt = 0;

				/* 退出临界区：恢复任务调度 */
				taskEXIT_CRITICAL();

				dgb_printf_safe("[app_task_mpu6050]:roll:%.1f turn on lcd\r\n",roll);
			}
		}

		if(time_5s%25 == 0)                  /* 25 次 200ms = 5s，更新步数 */
		{
			/* 获取步数 */
			dmp_get_pedometer_step_count(&step_count);

			/* 更新步数显示 */

			/* 加锁：阻塞等待 LVGL 互斥锁 */
			xSemaphoreTake(g_mutex_lvgl_handle,portMAX_DELAY);

			lv_label_set_text_fmt(ui_LabelStepNum,"%d",step_count); /* 设置步数标签 */
			lv_arc_set_value(ui_ArcStepNum,step_count);              /* 设置步数圆弧 */

			/* 解锁：释放 LVGL 互斥锁 */
			xSemaphoreGive(g_mutex_lvgl_handle);

			dgb_printf_safe("[app_task_mpu6050]:step count:%d\r\n",step_count);
		}
	}
}

/**
  * @brief  久坐提醒任务
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：任务启动后立即挂起，由外部恢复后开始运行。每 5s 读取
  *         一次步数，若两次间步数增量小于5则视为久坐，触发5次震动提醒。
  *         同步关系：与闹钟任务类似，通过马达队列+计数信号量同步震动执行。
  */
static void app_task_sedentary_remind(void *pvParameters)
{
	uint32_t step_count_now = 0;              /* 当前步数 */
	uint32_t step_count_last = 0;             /* 上次步数 */
	uint8_t cnt = 5;                         /* 震动次数计数器 */
	BaseType_t ret = pdFALSE;                /* 队列发送返回值 */
	motor_t motor = {0};                     /* 马达请求结构 */

	dgb_printf_safe("[app_task_sedentary_remind]:app_task_sedentary_remind supend...\r\n");

	/* 挂起自己：等待外部(久坐开关打开时)通过 vTaskResume 恢复 */
	vTaskSuspend(NULL);

	for(;;)
	{
		/* 20s检测步数状态 */
		vTaskDelay(5000);

		/* 获取步数 */
		dmp_get_pedometer_step_count(&step_count_now); /* 读取 DMP 计步寄存器 */

		/* 与上次的步数比较：增量<5 表示久坐 */
		if(step_count_now - step_count_last < 5)
		{
			dgb_printf_safe("[app_task_sedentary_remind]:please get up and walk\r\n");

			/* 震动模块震动5次 */
			while(cnt--)
			{
				motor.sta = 1;               /* 设置震动状态有效 */
				motor.time = 1000;           /* 震动时长 1000ms */

				ret = xQueueSend(g_queue_motor_handle,&motor,100); /* 发送马达请求(超时100 tick) */
				if(ret == pdFALSE)           /* 发送失败 */
				{
					dgb_printf_safe("[app_task_sedentary_remind]:app_task_sedentary_remind send queue error\r\n");
				}

				/* 等待震动模块执行完毕，避免没执行完又发出震动指令 ，同步任务：
				   阻塞等待 g_sem_motor_handle 信号量(由马达任务震动完成后释放) */
				xSemaphoreTake(g_sem_motor_handle,portMAX_DELAY);

				vTaskDelay(1000);            /* 两次震动间隔 1s */
			}
			cnt = 5;                         /* 重置震动次数计数器 */
		}

		dgb_printf_safe("[app_task_sedentary_remind]:step_count_now = %d step_count_last = %d\r\n",step_count_now,step_count_last);
		/* 记录上一次步数 */
		step_count_last = step_count_now;
	}
}

/**
  * @brief  步数清零任务，判断时间到了第二天将昨天步数清空
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：每 60s 检查一次 RTC 时间，若小时数大于 23(即跨日)
  *         则调用 dmp_set_pedometer_step_count(0) 清空 DMP 计步寄存器。
  *         无同步关系，仅访问 RTC 寄存器与 DMP 接口。
  */
static void app_task_step_clear(void *pvParameters)
{
	RTC_DateTypeDef RTC_DateStruct;           /* RTC 日期结构体 */
	RTC_TimeTypeDef RTC_TimeStruct;           /* RTC 时间结构体 */

	dgb_printf_safe("[app_task_step_clear]:app_task_step_clear start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		/* 获取当前小时时间 */
		RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);//获取时间
		RTC_GetDate(RTC_Format_BIN,&RTC_DateStruct);//获取日期

		/*过了当日重置步数 */
		if(RTC_TimeStruct.RTC_Hours > 23)     /* 小时数>23 表示跨日 */
		{
			while(dmp_set_pedometer_step_count(0)) /* 重置 DMP 计步寄存器，失败则重试 */
			{
				dgb_printf_safe("[app_task_step_clear]:clear today step... \r\n");
				vTaskDelay(500);
			}

			dgb_printf_safe("[app_task_step_clear]:clear today step successful... \r\n");
		}

		/* 一分钟检查一次时间 */
		vTaskDelay(60000);
	}
}

/**
  * @brief  触摸屏幕点亮屏幕和唤醒CPU
  * @param  pvParameters 任务函数形参(未使用)
  * @retval None
  * @note   调度策略：每 20ms 读取触摸屏手指数量。检测到触摸时点亮屏幕，
  *         并将系统无操作计数清零，防止自动熄屏触发。同步关系：访问全局
  *         变量 g_lcd_display/g_cpu_status/g_system_no_opreation_cnt 时
  *         使用临界区保护，避免与软件定时器/MPU6050任务竞争。
  */
static void app_task_istouch_lcd(void *pvParameters)
{
	uint8_t tp_finger_num = 0;               /* 触摸手指数量 */

	dgb_printf_safe("[app_task_istouch_lcd]:app_task_istouch_lcd start running...\r\n"); /* 任务启动日志 */

	for(;;)                                  /* 任务主循环 */
	{
		tp_finger_num = tp_finger_num_get(); /* 读取触摸屏当前手指数量 */
		/* 检测是否触摸屏幕 */
		if(tp_finger_num == 1)                /* 1表示有手指触摸 */
		{
			if(!g_lcd_display)               /* 屏幕当前熄灭 */
			{
				lcd_set_brightness(g_lcd_brightness); /* 点亮屏幕 */

				/* 进入临界区：保护对全局状态的修改 */
				taskENTER_CRITICAL();

				g_cpu_status = CPU_MODE_RUN;  /* CPU 切换到运行状态 */
				g_lcd_display = 1;            /* 屏幕状态标志置1 */

				/* 退出临界区 */
				taskEXIT_CRITICAL();

				dgb_printf_safe("[app_task_istouch_lcd]:touch turn on lcd\r\n");
			}

			/* 进入临界区：保护无操作计数清零，避免与软件定时器回调竞争 */
			taskENTER_CRITICAL();

			g_system_no_opreation_cnt = 0;    /* 重置无操作计数，防止自动熄屏 */

			/* 退出临界区 */
			taskEXIT_CRITICAL();
		}

		vTaskDelay(20);                      /* 20ms 检测周期 */
	}
}

/**
  * @brief  软件定时器回调函数，周期 1s 触发
  * @param  xTimer 触发回调的软件定时器句柄
  * @retval None
  * @note   调度策略：在定时器服务任务上下文中执行(优先级较高)，每 1s 触发。
  *         职责：1) 检测无操作时间是否达到阈值且自动熄屏开关打开，
  *         若满足条件则熄灭屏幕并切换 CPU 到睡眠状态；2) 无操作计数自增。
  *         同步关系：访问全局变量 g_lcd_display/g_cpu_status/
  *         g_system_no_opreation_cnt 使用临界区保护，避免与触摸任务/MPU6050
  *         任务竞争。
  */
/* 软件定时器回调函数 */
void timer_call_back(TimerHandle_t xTimer)
{
	/* 系统无操作时间达到 屏幕状态为亮屏 自动熄屏开启 则灭屏 */
	if(g_system_no_opreation_cnt >= SCREEN_OFF_TIME && g_lcd_display && g_lcd_self_turn_off_switch)
	{
		/* 熄屏：将 LCD 亮度设为 0 */
		lcd_set_brightness(0);

		/* 进入临界区：保护对全局状态的修改，避免与触摸/MPU6050任务竞争 */
	    taskENTER_CRITICAL();

		g_cpu_status = CPU_MODE_SLEEP;        /* CPU 切换到睡眠状态(让空闲钩子进入 WFI) */
		g_lcd_display = 0;                    /* 屏幕状态标志置0 */

		/* 退出临界区 */
	    taskEXIT_CRITICAL();

		dgb_printf_safe("[timer_call_back]:lcd turn off\r\n");
	}

	/* 进入临界区：保护无操作计数自增，避免与触摸任务清零竞争 */
	taskENTER_CRITICAL();

	g_system_no_opreation_cnt++;//无操作系统计数+1

	/* 退出临界区 */
	taskEXIT_CRITICAL();
}

/**
  * @brief  空闲任务钩子函数，由 FreeRTOS 在空闲任务中调用
  * @retval None
  * @note   当 CPU 状态为睡眠时进入 WFI 指令，使 CPU 进入低功耗等待中断
  *         状态，由任意中断(触摸/RTC/串口等)唤醒，节省电量。
  */
void vApplicationIdleHook( void )
{
	 //进入睡眠模式
	if(g_cpu_status == CPU_MODE_SLEEP)       /* CPU 处于睡眠模式 */
	{
		__WFI();                             /* 等待中断唤醒指令，CPU 挂起直到中断 */
	}
}

/**
  * @brief  任务栈溢出钩子函数，当 FreeRTOS 检测到任务栈溢出时调用
  * @param  pxTask     发生栈溢出的任务句柄
  * @param  pcTaskName 发生栈溢出的任务名称
  * @retval None
  * @note   栈溢出通常是致命错误，本函数禁用中断并死循环以便调试器捕获。
  *         需在 FreeRTOSConfig.h 中开启 configCHECK_FOR_STACK_OVERFLOW。
  */
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;                     /* 消除未使用变量警告 */
	( void ) pxTask;                         /* 消除未使用变量警告 */

	taskDISABLE_INTERRUPTS();                /* 禁用所有中断，避免错误传播 */
	for( ;; );                               /* 死循环，等待复位或调试器介入 */
}

/**
  * @brief  Tick 钩子函数，由 FreeRTOS 在每个 SysTick 中断中调用
  * @retval None
  * @note   用于驱动 LVGL 心跳 lv_tick_inc，必须保持精准 1ms 间隔。
  *         需在 FreeRTOSConfig.h 中开启 configUSE_TICK_HOOK。
  */
/* Tick钩子函数 */
void vApplicationTickHook( void )
{
	/* lvgl心跳，需要精准时间间隔调用：每 tick(1ms) 调用一次 lv_tick_inc */
	lv_tick_inc(1);
}

/**
  * @brief  内存分配失败钩子函数，当 pvPortMalloc 分配失败时调用
  * @retval None
  * @note   内存不足是致命错误，本函数禁用中断并死循环以便调试器捕获。
  *         需在 FreeRTOSConfig.h 中开启 configUSE_MALLOC_FAILED_HOOK。
  */
void vApplicationMallocFailedHook( void )
{
	taskDISABLE_INTERRUPTS();                /* 禁用所有中断 */
	for( ;; );                               /* 死循环 */
}


