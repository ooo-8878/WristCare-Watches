#include "includes.h"

/**
 * @file    tim.c
 * @brief   TIM3 定时器驱动模块(PWM 输出 + 更新中断用于串口空闲检测)
 *
 * @details 模块功能：
 *          - TIM3 配置为 PWM 输出模式(OC1)，可输出可调占空比的 PWM 波形
 *          - 同时使能更新(溢出)中断，在 TIM3_IRQHandler 中通过比较两次中断之间
 *            USART1 接收字节计数 g_usart1_rx_cnt 是否变化，来判断串口接收是否完成
 *          - 当连续 10 次更新中断(约 1ms 周期 * 10)内 USART1 都没有新数据到达，
 *            即认为一帧接收完成，置位 g_usart1_rx_end 标志供上层任务处理
 *
 *          硬件资源占用：
 *          - TIM3：定时器外设，挂在 APB1 总线上
 *          - NVIC：TIM3_IRQn 中断通道
 *          - 共享全局变量：g_usart1_rx_cnt(USART1 接收计数)、g_usart1_rx_end(接收完成标志)
 *
 *          定时参数计算：
 *          - TIM3 时钟 = APB1 * 2 = 84MHz
 *          - 预分频 Prescaler = 840-1 -> 计数时钟 = 84MHz / 840 = 100kHz(10us)
 *          - 周期 Period = 100000/1000-1 = 99 -> 更新中断周期 = 100 * 10us = 1ms
 *          - 即每 1ms 进入一次 TIM3_IRQHandler
 *
 *          与 FreeRTOS 的交互：
 *          - 本模块不直接调用 FreeRTOS API，仅置位全局变量 g_usart1_rx_end
 *          - 中断优先级使用 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，
 *            以便后续若需要在中断中调用 FreeRTOS API 时不会冲突
 */

/**
 * @brief   初始化 TIM3：PWM1 模式 + 1ms 更新中断
 * @note    TIM3 时钟 84MHz，预分频 840 得 100kHz 计数时钟，Period=100 得 1ms 中断周期
 *          OC1 输出 PWM，初始比较值 100，低电平有效
 */
void tim3_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   /* 时基单元配置结构体 */
	TIM_OCInitTypeDef  TIM_OCInitStructure;           /* 输出比较配置结构体 */
	NVIC_InitTypeDef  NVIC_InitStructure;             /* NVIC 配置结构体 */

	//打开TIM3硬件时钟 /* TIM3 挂在 APB1 上，必须先使能时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 100000/1000-1;//计数值，用于定时时间的设置 /* 100000/1000-1=99，自动重装载值 ARR */
	TIM_TimeBaseStructure.TIM_Prescaler = 840-1;//预分频值的配置 /* 840-1=839，PSC，84MHz/840=100kHz */
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//F407没有，不需要配置 /* 时钟分频因子，F4 上无效 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数，就是0 -> TIM_Period，然后就会触发时间更新中断
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  /* 写入时基寄存器，1ms 周期确定 */

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1工作模式 即计数值小于比较值时是有效状态 大于比较值是失效状态
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//允许输出脉冲
	TIM_OCInitStructure.TIM_Pulse = 100; //初始化CCR /* 比较寄存器初值 100，可调整占空比 */
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //有效状态低电平 /* 有效电平为低，即 CNT<CCR 时输出低电平 */
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  /* 写入 OC1 寄存器，PWM 输出 */

	/* 定时器时间更新中断使能 */ /* 使能更新中断，溢出时进入 TIM3_IRQHandler */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	//配置NVIC，用于打开TIM3的中断请求通道
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	//TIM3的请求通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;//抢占优先级 /* 满足 FreeRTOS API 调用要求 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//打开其请求通道
	NVIC_Init(&NVIC_InitStructure);  /* 写入 NVIC 配置，使能 TIM3 中断通道 */

	/* 使能TIM3工作 */ /* 启动 TIM3 计数器，开始产生 PWM 与中断 */
	TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief   TIM3 更新中断服务程序(每 1ms 触发一次)
 * @note    利用 1ms 周期性中断检测 USART1 是否接收完成：
 *          - 每次中断 irq_cnt++；
 *          - 当 irq_cnt>=10(累计 10ms)且 g_usart1_rx_cnt>0 时进行一次判断：
 *              若本次接收计数 g_usart1_rx_cnt 与上次记录的 usart1_rx_cnt_last 相等，
 *              说明这 10ms 内没有新数据到达，认为一帧接收完毕，置位 g_usart1_rx_end；
 *              否则更新 usart1_rx_cnt_last 并继续等待
 *          - 该方法相当于用定时器实现串口空闲超时检测，避免串口无 IDLE 中断的场合
 */
void TIM3_IRQHandler(void)
{
	static uint32_t usart1_rx_cnt_last=0;   /* 上次记录的 USART1 接收字节数 */
	static uint32_t irq_cnt=0;              /* 中断计数，累计到 10 后做一次空闲判断 */

	//检测时间更新中断的标志位是否置位
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)   /* 判断是否为更新中断 */
	{
		irq_cnt++;   /* 中断计数加 1 */

		if(irq_cnt>=10 && g_usart1_rx_cnt)   /* 累计 10ms 且 USART1 已经开始接收数据 */
		{
			//若相等，则表示串口3目前接收数据完毕
			if(g_usart1_rx_cnt == usart1_rx_cnt_last)   /* 接收计数与上次相等，说明 10ms 内无新数据 */
			{
				g_usart1_rx_end=1;   /* 置位接收完成标志，通知上层任务处理数据 */
			}
			else
			{
				usart1_rx_cnt_last = g_usart1_rx_cnt;   /* 不相等，更新上次计数值，继续等待下一周期 */
			}

			irq_cnt=0;   /* 复位计数器，开始下一轮 10ms 检测 */
		}

		//清空标志位
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);   /* 清除更新中断标志，避免反复进入 */
	}
}
