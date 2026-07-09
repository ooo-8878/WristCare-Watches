/**
 ******************************************************************************
 * @file    usart.c
 * @brief   USART1 串口驱动模块
 *
 * @details 本文件实现 STM32F4 的 USART1 串口驱动,主要功能包括:
 *           1. USART1 初始化(GPIO/USART/NVIC 配置)
 *           2. printf 重定向(fputc),支持 C 标准库输出
 *           3. 阻塞式发送字节流(usart1_send_bytes)
 *           4. 接收中断处理(USART1_IRQHandler),配合 TIM3 完成接收
 *              帧结束检测(g_usart1_rx_cnt / g_usart1_rx_end)
 *
 * @hardware 硬件资源:
 *           - USART1:  挂载于 APB2 总线(最高 90MHz,支持高波特率)
 *           - GPIOA:   挂载于 AHB1 总线
 *           - PA9  (USART1_TX): 复用功能 AF7,推挽,高速
 *           - PA10 (USART1_RX): 复用功能 AF7,推挽,高速
 *           - NVIC:    USART1_IRQn 中断
 *
 * @protocol 通信协议:
 *           - 帧格式: 8 数据位 / 1 停止位 / 无校验 / 无硬件流控
 *           - 波特率: 由参数 baud 传入(常用 115200)
 *           - 接收超时检测: 由 TIM3 更新中断监测帧间间隔
 *             (TIM3 在每次收到字节时复位,长时间无数据视为一帧结束)
 *
 * @freertos 与 FreeRTOS 的交互方式:
 *           - 中断服务程序 USART1_IRQHandler 中调用
 *             taskENTER_CRITICAL_FROM_ISR / taskEXIT_CRITICAL_FROM_ISR
 *             保护共享缓冲区,避免与读取任务产生竞争。
 *           - NVIC 中断优先级设为 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
 *             保证可安全调用 FreeRTOS 的 *FromISR API。
 *           - 应用任务可读取 g_usart1_rx_end / g_usart1_rx_cnt 判断帧完整性,
 *             并通过任务通知/队列将数据投递给业务任务处理。
 *
 * @design 设计要点:
 *           - 关闭半主机模式(#pragma import(__use_no_semihosting_swi)),
 *             使 printf 可在无调试器环境下正常工作。
 *           - 接收缓冲区使用 volatile 修饰,保证中断与主循环访问一致性。
 *           - 缓冲区大小 1280 字节,满足一般调试/命令传输需求。
 *           - 注:本文件静态变量 USART_InitStructure 等存在但未直接使用,
 *             实际配置在 usart1_init 内部使用局部变量。
 ******************************************************************************
 */
#include "includes.h"      /* 包含工程统一头文件:STM32 标准库、FreeRTOS、配置宏等 */

static USART_InitTypeDef   		USART_InitStructure;   /* 静态 USART 初始化结构体(本工程实际未使用,保留) */
static GPIO_InitTypeDef 		GPIO_InitStructure;    /* 静态 GPIO 初始化结构体(本工程实际未使用,保留) */
static NVIC_InitTypeDef   		NVIC_InitStructure;    /* 静态 NVIC 初始化结构体(本工程实际未使用,保留) */

volatile uint8_t  g_usart1_rx_buf[1280];   /* USART1 接收环形缓冲区,1280 字节;volatile 防止编译器优化 */
volatile uint32_t g_usart1_rx_cnt=0;       /* 当前接收字节数计数,每收到一个字节自增 */
volatile uint32_t g_usart1_rx_end=0;       /* 接收完成标志:1 表示缓冲区满,由 TIM3/应用层复位 */

#pragma import(__use_no_semihosting_swi)   /* 禁用 C 库的半主机模式(SWI),避免 printf 卡死在软件中断 */

struct __FILE { int handle; /* Add whatever you need here */ };   /* 重定义标准 FILE 结构,供 stdio 使用 */
FILE __stdout;              /* 标准输出文件对象,printf 使用 */
FILE __stdin;               /* 标准输入文件对象,scanf 使用 */

/**
  * @brief  printf 字符输出重定向函数
  * @note   C 标准库 printf 最终会调用 fputc 输出单个字符,
  *         此处将字符通过 USART1 发送,实现 printf 调试输出。
  *         必须配合 #pragma import(__use_no_semihosting_swi) 使用。
  * @param  ch: 待发送的字符(ASCII)
  * @param  f:  文件指针(由标准库传入,本函数不使用)
  * @retval 返回发送的字符 ch
  */
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1,ch);            /* 将字符写入 USART1 数据寄存器(DR),启动发送 */
		
	//等待数据发送成功
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);   /* 轮询 TXE 标志,等待发送数据寄存器空 */
	USART_ClearFlag(USART1,USART_FLAG_TXE);                      /* 清除 TXE 标志,便于下次发送 */

	return ch;                           /* 返回发送的字符 */
}

/**
  * @brief  半主机模式退出钩子函数
  * @note   禁用半主机模式后必须提供 _sys_exit 的空实现,
  *         否则链接器会引用半主机符号导致链接失败。
  * @param  return_code: 退出码(本函数不使用)
  * @retval None
  */
void _sys_exit(int return_code) {

}


/**
  * @brief  USART1 初始化函数
  * @note   完成 GPIOA(PA9/PA10)复用配置、USART1 参数配置、
  *         接收中断使能、NVIC 优先级配置及串口使能。
  *         调用位置:系统初始化阶段,任务调度启动之前。
  * @param  baud: 串口波特率(如 115200)
  * @retval None
  */
void usart1_init(uint32_t baud)
{
	USART_InitTypeDef   USART_InitStructure;        /* USART1 初始化参数结构体(局部) */
	GPIO_InitTypeDef 	GPIO_InitStructure;         /* GPIOA 引脚初始化参数结构体(局部) */
    NVIC_InitTypeDef   	NVIC_InitStructure;         /* NVIC 中断配置结构体(局部) */

	//使能端口A硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);   /* 使能 GPIOA 所在 AHB1 总线时钟,PA9/PA10 工作前提 */
	
	//使能串口1硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  /* 使能 USART1 所在 APB2 总线时钟(USART1 挂 APB2) */
	
	
	//配置PA9、PA10为复用功能引脚
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_10;   /* 选择 PA9(TX) 与 PA10(RX) 两个引脚 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;             /* 复用功能模式,引脚交由 USART1 外设控制 */
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;          /* 高速(50MHz),满足高波特率翻转需求 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            /* 推挽输出,驱动能力强 */
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	        /* 无内部上下拉,由外部电路决定空闲电平 */
	GPIO_Init(GPIOA,&GPIO_InitStructure);                     /* 将上述配置写入 GPIOA 寄存器 */
	
	//将PA9、PA10连接到USART1的硬件
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);  /* 将 PA9 复用为 USART1_TX(AF7) */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  /* 将 PA10 复用为 USART1_RX(AF7) */
	
	
	//配置USART1的相关参数：波特率、数据位、校验位
	USART_InitStructure.USART_BaudRate = baud;//波特率                            /* 波特率,由参数传入 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位       /* 8 位数据位 */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位            /* 1 位停止位 */
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验               /* 无奇偶校验 */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制 /* 无 RTS/CTS 硬件流控 */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//允许串口发送和接收数据 /* 同时使能接收与发送 */
	USART_Init(USART1, &USART_InitStructure);                                     /* 将参数写入 USART1 寄存器 */
	
	
	//使能串口接收到数据触发中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);   /* 使能 RXNE(接收非空)中断,每收到 1 字节触发一次 */
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                                  /* 选择 USART1 中断通道 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY; /* 抢占优先级=最高允许调用 RTOS API 的级别 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                                  /* 子优先级 0 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                     /* 使能该中断通道 */
	NVIC_Init(&NVIC_InitStructure);                                                     /* 写入 NVIC 寄存器 */
	
	//使能串口1工作
	USART_Cmd(USART1,ENABLE);   /* 使能 USART1 外设,开始收发 */
}

/**
  * @brief  USART1 阻塞式发送字节数据
  * @note   通过轮询 TXE 标志逐字节发送,适用于少量数据发送。
  *         大数据量建议改用 DMA 发送以降低 CPU 占用。
  * @param  buf: 待发送数据缓冲区指针
  * @param  len: 待发送字节数
  * @retval None
  */
void usart1_send_bytes(uint8_t *buf,uint32_t len)
{
	uint8_t *p = buf;                /* 工作指针,逐字节推进 */
	
	while(len--)
	{
		USART_SendData(USART1,*p);    /* 将当前字节写入 USART1 DR 寄存器,启动发送 */
		
		p++;                          /* 指针指向下一个字节 */
		
		//等待数据发送成功
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);  /* 等待 TXE=1,表示数据已搬到移位寄存器 */
		USART_ClearFlag(USART1,USART_FLAG_TXE);                    /* 清除 TXE 标志,准备下次发送 */
	}
}



/**
  * @brief  USART1 接收中断服务函数
  * @note   每当 USART1 收到一个字节(RXNE 置位)即进入此中断。
  *         将字节存入 g_usart1_rx_buf,并自增 g_usart1_rx_cnt。
  *         缓冲区满时置位 g_usart1_rx_end 标志。
  *         实际应用中通常配合 TIM3 的更新中断做帧间空闲检测:
  *         每收到字节复位 TIM3,长时间无数据则触发 TIM3 中断
  *         置位帧结束标志,通知任务处理整帧数据。
  *         本函数使用 taskENTER/EXIT_CRITICAL_FROM_ISR 保护,
  *         确保与读取缓冲区的任务互斥访问。
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	uint8_t d=0;                    /* 临时存放本次接收到的字节 */
	uint32_t ulReturn;              /* 保存临界区嵌套计数返回值 */
		
	/* 进入中断临界区 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();   /* 进入 ISR 临界区,屏蔽低于阈值的中断,保护共享缓冲区 */
	
	//检测是否接收到数据
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)   /* 判断是否为接收非空中断 */
	{
		d=USART_ReceiveData(USART1);                      /* 读取 DR 寄存器获取字节(读 DR 同时会自动清 RXNE) */
		
		
		g_usart1_rx_buf[g_usart1_rx_cnt++]=d;             /* 存入缓冲区并后移写指针 */
	
		if(g_usart1_rx_cnt >= sizeof g_usart1_rx_buf)     /* 检查是否缓冲区已满 */
		{
			g_usart1_rx_end=1;                            /* 置位接收完成标志,通知任务处理 */
		}			

		//清空标志位，可以响应新的中断请求
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);   /* 清除 RXNE 中断挂起位,允许下次中断 */
	}
	
	/* 退出中断临界区*/
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);                /* 退出 ISR 临界区,恢复中断屏蔽状态 */
}




