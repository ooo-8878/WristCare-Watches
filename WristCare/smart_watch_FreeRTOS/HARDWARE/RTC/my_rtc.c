#include "includes.h"

/**
 * @file    my_rtc.c
 * @brief   STM32 内部 RTC 驱动模块
 *
 * @details 模块功能：
 *          - 使用 STM32 内部 RTC 实现实时时钟、唤醒定时器(每秒一次)和闹钟功能
 *          - 唤醒中断(EXTI_Line22) 每 1 秒触发一次，置位事件组 EVENT_GROUPS_RTC_WAKEUP，
 *            唤醒 app_task_rtc 任务刷新屏幕显示时间
 *          - 闹钟中断(EXTI_Line17) 在闹钟时间到时触发，置位事件组 EVENT_GROUPS_RTC_ALARM，
 *            唤醒 app_task_rtc_alarm 任务执行震动提醒
 *
 *          硬件资源占用：
 *          - RTC：内部实时时钟外设，时钟源 LSE(32.768kHz)
 *          - PWR：电源时钟，用于使能备份域访问
 *          - EXTI_Line22：RTC 唤醒中断的外部中断线
 *          - EXTI_Line17：RTC 闹钟中断的外部中断线
 *          - NVIC：RTC_WKUP_IRQn、RTC_Alarm_IRQn 两个中断通道
 *          - RTC 备份寄存器 RTC_BKP_DR0：用于保存初始化标志 0x5678
 *
 *          与 FreeRTOS 的交互：
 *          - 在中断服务程序中调用 xEventGroupSetBitsFromISR 向事件组 g_event_group_handle 置位
 *          - 使用 taskENTER_CRITICAL_FROM_ISR / taskEXIT_CRITICAL_FROM_ISR 保护临界区
 *          - 中断优先级使用 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(可调用 FreeRTOS API 的最高优先级)
 */

/**
 * @brief   初始化 STM32 内部 RTC，包含时钟源选择、分频、初始时间日期、唤醒定时器、EXTI 与 NVIC 配置
 * @note    通过读取备份寄存器 RTC_BKP_DR0 判断是否已经初始化过，避免主供电断电后再次初始化时间
 *          唤醒定时器配置为每 1 秒触发一次中断(EXTI_Line22)，用于驱动 1Hz 的时间刷新任务
 */
void my_rtc_init(void)
{
	//相关配置结构体 /* RTC 初始化结构体，用于配置分频与小时格式 */
	RTC_InitTypeDef  RTC_InitStructure;
	RTC_DateTypeDef  RTC_DateStructure;   /* 日期结构体，用于设置年月日星期 */
	RTC_TimeTypeDef  RTC_TimeStructure;   /* 时间结构体，用于设置时分秒 */
	EXTI_InitTypeDef EXTI_InitStructure;  /* 外部中断配置结构体，用于配置 EXTI_Line22 唤醒中断线 */
	NVIC_InitTypeDef NVIC_InitStructure;  /* NVIC 配置结构体，用于配置 RTC 唤醒中断通道 */
	
	//使能电源时钟 /* PWR 时钟必须使能才能操作备份域 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	//允许访问RTC /* 解除备份域写保护，才能修改 RTC 和备份寄存器 */
	PWR_BackupAccessCmd(ENABLE);


	if(RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x5678)// 判断备份寄存器的值是否设置过为0x1234  /* 读取备份寄存器 DR0，判断是否已经初始化过 */
	{											     //	如果设置过代表时间日期已经初始化过一次了，不必再重置                                                //
		                                             // 否则将初始化设置时间日期
		//使能电源时钟 /* 在备份域内部再次使能 PWR 时钟(冗余写法) */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

		//允许访问RTC /* 再次使能备份域访问 */
		PWR_BackupAccessCmd(ENABLE);

#if 1 //选择LSE时钟 32.768kHz /* 编译开关：使用 LSE 外部 32.768kHz 晶振 */

		//使能LSE /* 开启外部低速晶振 LSE */
		RCC_LSEConfig(RCC_LSE_ON);

		//等待LSE就绪 /* 阻塞等待 LSE 起振稳定，否则后续 RTC 时钟会异常 */
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

		//选择LSE作为RTC时钟源 /* 将 LSE 配置为 RTC 时钟源 */
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

#else //选择LSI时钟 32KHz /* 备选方案：使用内部低速 LSI */

		//使能LSI /* 开启内部低速 LSI 时钟 */
		RCC_LSICmd(ENABLE);

		//检查该LSI是否有效  /* 等待 LSI 稳定 */
		while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

		//选择LSI作为RTC的硬件时钟源 /* 将 LSI 配置为 RTC 时钟源 */
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
#endif

		//使能RTC时钟 /* 使能 RTC 时钟，之后 RTC 寄存器才能开始工作 */
		RCC_RTCCLKCmd(ENABLE);

		//等待RTC相关寄存器就绪 /* 等待 RTC 寄存器与 APB 时钟同步，避免读取到旧值 */
		RTC_WaitForSynchro();

#if 1//选择LSE时钟 /* LSE 路径下的分频配置 */

		//配置RTC分频 RTC频率 = LSE/(RTC_AsynchPrediv+1)/(SynchPrediv+1) = 1Hz /* 32768/(127+1)/(255+1)=1Hz */
		RTC_InitStructure.RTC_AsynchPrediv = 127;   /* 异步分频 127+1=128 */
		RTC_InitStructure.RTC_SynchPrediv = 255;    /* 同步分频 255+1=256，最终得 1Hz */
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;//24小时格式
		RTC_Init(&RTC_InitStructure);                /* 写入 RTC 寄存器完成初始化 */

#else  //选择LSI时钟 /* LSI 路径下的分频配置，LSI 约 32kHz，故同步分频调整为 249 */

		//配置RTC分频 RTC频率 = LSI/(RTC_AsynchPrediv+1)/(SynchPrediv+1) = 1Hz /* 32000/128/250=1Hz */
		RTC_InitStructure.RTC_AsynchPrediv = 127;
		RTC_InitStructure.RTC_SynchPrediv = 249;
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;//24小时格式
		RTC_Init(&RTC_InitStructure);

#endif

		//配置日期 2024-10-14 星期1
		RTC_DateStructure.RTC_Year = 24;                  /* 年份 2024(只存后两位) */
		RTC_DateStructure.RTC_Month = RTC_Month_October;  /* 10 月 */
		RTC_DateStructure.RTC_Date = 14;                  /* 14 日 */
		RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Monday;/* 星期一 */
		RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);  /* 以 BIN 格式写入日期寄存器 */

		//配置时间 上午 10:30:00  /* 实际为 14:53:00(注释与代码不一致) */
		RTC_TimeStructure.RTC_H12 = RTC_H12_PM;          /* 下午 */
		RTC_TimeStructure.RTC_Hours = 14;                 /* 14 时 */
		RTC_TimeStructure.RTC_Minutes = 53;               /* 53 分 */
		RTC_TimeStructure.RTC_Seconds = 00;               /* 00 秒 */
		RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);  /* 以 BIN 格式写入时间寄存器 */

		RTC_WriteBackupRegister(RTC_BKP_DR0,0x5678); //第一次设置备份寄存器的值，主供电断开数据不丢失 /* 写入魔术字 0x5678，下次上电检测到此值则跳过初始化 */
	}

	//关闭唤醒功能 /* 修改唤醒定时器前必须先关闭，否则写不进去 */
	RTC_WakeUpCmd(DISABLE);

	//为唤醒功能选择RTC配置好的时钟源 1Hz /* CK_SPRE 即 1Hz 时钟，16 位计数器 */
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);

	//设置唤醒计数值为自动重载 0
	//这里频率选择RTC配置好的时钟源1Hz ,即每次唤醒时间为1s /* 重载值+1 秒触发一次，0 表示 1 秒 */
	RTC_SetWakeUpCounter(0);

	//清除RTC唤醒中断标志 /* 清除残留的中断标志，避免一打开就误触发 */
	RTC_ClearITPendingBit(RTC_IT_WUT);

	//使能RTC唤醒中断 /* 使能唤醒定时器中断 WUT */
	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	//使能唤醒功能 /* 启动唤醒定时器，每 1 秒进入一次 RTC_WKUP_IRQHandler */
	RTC_WakeUpCmd(ENABLE);

	//配置外部中断控制线22  /* RTC 唤醒信号通过 EXTI_Line22 路由到 NVIC */
	EXTI_InitStructure.EXTI_Line = EXTI_Line22;			//当前使用外部中断控制线22
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//上升沿触发中断
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			//使能外部中断控制线22
	EXTI_Init(&EXTI_InitStructure);                                  /* 写入 EXTI 配置 */

	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;		//允许RTC唤醒中断触发
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;	//抢占优先级 /* 必须大于等于该宏才能在中断里调用 FreeRTOS API */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能
	NVIC_Init(&NVIC_InitStructure);                                 /* 写入 NVIC 配置，使能中断通道 */
}


/**
 * @brief   初始化 RTC 闹钟 A，配置默认闹钟时间、屏蔽位、EXTI_Line17 与 NVIC 中断通道
 * @note    闹钟信号通过 EXTI_Line17 路由到 NVIC，触发 RTC_Alarm_IRQHandler
 *          本函数中通过 RTC_AlarmMask_DateWeekDay 屏蔽日期/星期位，使闹钟每天都会触发
 *          默认闹钟时间为 23:04:00(注释写的是 8:30，与代码不符)
 *          注意：函数末尾的 RTC_AlarmCmd(RTC_Alarm_A,ENABLE) 被注释掉，
 *                闹钟默认未使能，需要外部调用 myrtc_alarm_set_time 后再使能
 */
void myrtc_alarm_init(void)
{
	//相关配置结构体
	EXTI_InitTypeDef EXTI_InitStructure;            /* 外部中断配置结构体，用于配置 EXTI_Line17 */
	NVIC_InitTypeDef NVIC_InitStructure;            /* NVIC 配置结构体，用于配置 RTC_Alarm 中断通道 */
	RTC_AlarmTypeDef RTC_AlarmStructure;            /* 闹钟结构体，用于配置闹钟时间和屏蔽位 */

	//关闭闹钟 /* 修改闹钟寄存器前先关闭闹钟 A，避免配置过程中触发 */
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE);

	//设置闹钟时间 上午 8:30:00
	RTC_AlarmStructInit(&RTC_AlarmStructure);//先全部成员赋默认值，避免没有用上的成员没有赋值造成bug
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_H12_PM;       /* 下午 */
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = 23;              /* 23 时 */
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 04;            /* 04 分 */
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 00;            /* 00 秒 */
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;//屏蔽日期和星期，每天生效 /* 屏蔽日期/星期，使闹钟每天都会在此时触发 */
	RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_AlarmStructure); /* 以 BIN 格式写入闹钟 A 寄存器 */

	//使能闹钟A中断 /* 使能闹钟 A 的中断输出 */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	RTC_ClearFlag(RTC_FLAG_ALRAF);  /* 清除闹钟 A 标志位，避免历史标志触发 */

	/* 以下配置 EXTI_Line17：闹钟信号通过 EXTI_Line17 路由到 NVIC */
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;               /* RTC 闹钟对应的外部中断线 17 */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;       /* 中断模式(非事件模式) */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;   /* 上升沿触发 */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;                 /* 使能该外部中断线 */
	EXTI_Init(&EXTI_InitStructure);                           /* 写入 EXTI 配置 */

	/* Enable the RTC Alarm Interrupt */ /* 使能 RTC 闹钟中断 */
	RTC_ClearITPendingBit(RTC_IT_ALRA);  /* 清除 RTC 闹钟 A 中断挂起位 */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;     /* 选择 RTC 闹钟中断通道 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY; /* 抢占优先级，必须满足 FreeRTOS API 调用要求 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       /* 子优先级 0 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          /* 使能该通道 */
	NVIC_Init(&NVIC_InitStructure);                           /* 写入 NVIC 配置 */

	//使能闹钟A
	// RTC_AlarmCmd(RTC_Alarm_A,ENABLE); /* 注意：此处被注释，闹钟默认不使能 */

}

/**
 * @brief   重新设置闹钟 A 的触发时间
 * @param   rtc_h12  AM/PM 选择，取值 RTC_H12_AM 或 RTC_H12_PM
 * @param   hours    闹钟小时(24 小时制下直接传 0-23；12 小时制下传 1-12)
 * @param   minutes  闹钟分钟(0-59)
 * @note    本函数仅修改闹钟时间寄存器，秒固定为 0，日期/星期仍被屏蔽(每天触发)
 *          注意：本函数不使能闹钟，调用者需自行调用 RTC_AlarmCmd(RTC_Alarm_A,ENABLE)
 */
void myrtc_alarm_set_time(uint8_t rtc_h12,uint8_t hours,uint8_t minutes)
{
	RTC_AlarmTypeDef RTC_AlarmStructure;   /* 闹钟结构体 */

	//关闭闹钟 /* 修改闹钟寄存器前必须先关闭，否则可能写入失败 */
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE);

	RTC_AlarmStructInit(&RTC_AlarmStructure);//先全部成员赋默认值，避免没有用上的成员没有赋值造成bug
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = rtc_h12;        /* AM/PM 选择 */
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = hours;         /* 闹钟小时 */
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = minutes;     /* 闹钟分钟 */
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 00;          /* 闹钟秒固定为 0 */
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;//屏蔽日期和星期，每天生效 /* 屏蔽日期/星期位，使闹钟每天都会触发 */
	RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_AlarmStructure); /* 写入闹钟 A 寄存器 */
}

/**
 * @brief   RTC 唤醒定时器中断服务程序(每 1 秒触发一次)
 * @note    触发流程：RTC 唤醒定时器溢出 -> EXTI_Line22 上升沿 -> RTC_WKUP_IRQn
 *          在中断里向事件组 g_event_group_handle 置位 EVENT_GROUPS_RTC_WAKEUP，
 *          唤醒 app_task_rtc 任务刷新屏幕时间显示
 *          必须同时清除 RTC 中断标志和 EXTI 中断标志，否则会反复进入中断
 */
void RTC_WKUP_IRQHandler(void)
{
	uint32_t ulReturn;

	/* 进入中断临界区 */ /* FreeRTOS 中断中保护临界区，关闭可屏蔽中断 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();

	if(RTC_GetITStatus(RTC_IT_WUT) != RESET)   /* 判断是否为唤醒定时器中断 */
	{
		xEventGroupSetBitsFromISR(g_event_group_handle,EVENT_GROUPS_RTC_WAKEUP,NULL); /* 从中断向事件组置位 RTC 唤醒事件 */

		RTC_ClearITPendingBit(RTC_IT_WUT);       /* 清除 RTC 唤醒定时器中断标志 */
		EXTI_ClearITPendingBit(EXTI_Line22);      /* 清除 EXTI_Line22 挂起位 */
	}

	/* 退出中断临界区*/ /* 恢复之前的中断屏蔽状态 */
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}

/**
 * @brief   RTC 闹钟 A 中断服务程序(闹钟时间到时触发)
 * @note    触发流程：RTC 闹钟 A 匹配 -> EXTI_Line17 上升沿 -> RTC_Alarm_IRQn
 *          在中断里向事件组 g_event_group_handle 置位 EVENT_GROUPS_RTC_ALARM，
 *          唤醒 app_task_rtc_alarm 任务执行马达震动提醒
 *          必须同时清除 RTC 中断标志和 EXTI 中断标志
 */
void RTC_Alarm_IRQHandler(void)
{
	uint32_t ulReturn;

	/* 进入中断临界区 */ /* FreeRTOS 中断中保护临界区 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();

	if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)   /* 判断是否为闹钟 A 中断 */
	{
		xEventGroupSetBitsFromISR(g_event_group_handle,EVENT_GROUPS_RTC_ALARM,NULL); /* 从中断向事件组置位闹钟事件 */

		RTC_ClearITPendingBit(RTC_IT_ALRA);       /* 清除 RTC 闹钟 A 中断标志 */
		EXTI_ClearITPendingBit(EXTI_Line17);      /* 清除 EXTI_Line17 挂起位 */
	}

	/* 退出中断临界区*/ /* 恢复之前的中断屏蔽状态 */
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}