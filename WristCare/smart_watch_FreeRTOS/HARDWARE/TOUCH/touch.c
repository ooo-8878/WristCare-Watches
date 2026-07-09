#include "includes.h"

/**
 * @file    touch.c
 * @brief   CST816 触摸屏驱动模块(软件 I2C + 外部中断)
 *
 * @details 模块功能：
 *          - 通过软件模拟 I2C 与 CST816 系列电容触摸芯片通信
 *          - 支持两套引脚定义：TP_PIN_DEF_1(PD14/PD0/PD4 + PF12 中断)
 *            和 TP_PIN_DEF_2(PD6/PD7 + PC6 复位 + PC8 中断)
 *          - 触摸芯片在检测到触摸时拉低 INT 引脚，触发 STM32 外部中断
 *            (EXTI_Line12 或 EXTI_Line8)，中断里仅置位 g_tp_event，
 *            真正读取坐标的工作放到任务上下文中调用 tp_read 完成
 *          - tp_read 内部根据屏幕方向 lcd_get_direction() 对坐标做旋转
 *
 *          硬件资源占用：
 *          - GPIOD/GPIOF/GPIOC：SCL、SDA、RST、INT 引脚
 *          - EXTI_Line12(TP_PIN_DEF_1) 或 EXTI_Line8(TP_PIN_DEF_2)
 *          - NVIC：EXTI15_10_IRQn(TP_PIN_DEF_1) 或 EXTI9_5_IRQn(TP_PIN_DEF_2)
 *          - 软件 I2C：使用 delay_us 控制时序，无硬件 I2C 占用
 *
 *          与 FreeRTOS 的交互：
 *          - 中断服务程序里使用 taskENTER_CRITICAL_FROM_ISR / taskEXIT_CRITICAL_FROM_ISR
 *            保护临界区，并通过全局变量 g_tp_event 通知任务有触摸事件
 *          - 任务侧通过轮询 g_tp_event 或事件组感知触摸，调用 tp_read 读取坐标
 */

volatile uint32_t g_tp_event=0;   /* 触摸事件标志：EXTI 中断中置 1，任务读取坐标后清 0 */

uint16_t g_tp_x,g_tp_y;            /* 最近一次读取到的触摸坐标(经方向旋转后) */
uint8_t  g_tp_finger_num=0;       /* 触摸点的手指数量(0 表示无触摸) */

/**
 * @brief   切换 I2C SDA 引脚的工作模式(输出/输入)
 * @param   pin_mode  目标模式，GPIO_Mode_OUT(主机输出) 或 GPIO_Mode_IN(主机读应答/读数据)
 * @note    软件 I2C 需要在发送和接收之间动态切换 SDA 方向：
 *          - 主机发送时为 OUT，主机接收前切为 IN
 *          - 使用开漏输出 + 外部上拉，符合 I2C 协议线与特性
 *          - 通过宏 TP_PIN_DEF 选择当前实际使用的引脚(PD14 或 PD7)
 */
void tp_sda_pin_mode(GPIOMode_TypeDef pin_mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#if TP_PIN_DEF == TP_PIN_DEF_1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		//第14根引脚
	GPIO_InitStructure.GPIO_Mode = pin_mode;	//设置输出/输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	//开漏模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//不需要上拉电阻
	GPIO_Init(GPIOD, &GPIO_InitStructure);


#endif


#if TP_PIN_DEF == TP_PIN_DEF_2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		//第7根引脚
	GPIO_InitStructure.GPIO_Mode = pin_mode;	//设置输出/输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	//开漏模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//不需要上拉电阻
	GPIO_Init(GPIOD, &GPIO_InitStructure);

#endif

}

/**
 * @brief   产生 I2C 起始信号 START
 * @note    时序：SCL=1 期间，SDA 由高变低；之后 SCL 拉低，准备开始数据传输
 */
void tp_i2c_start(void)
{
	//保证SDA引脚为输出模式
	tp_sda_pin_mode(GPIO_Mode_OUT);

	TP_SCL_W=1;   /* SCL 拉高 */
	TP_SDA_W=1;   /* SDA 拉高，空闲状态 */
	delay_us(1);  /* 保持 1us */

	TP_SDA_W=0;   /* SCL 高时 SDA 下降沿 = START 信号 */
	delay_us(1);

	TP_SCL_W=0;   /* 拉低 SCL，准备传输第一位 */
	delay_us(1);
}


/**
 * @brief   产生 I2C 停止信号 STOP
 * @note    时序：SCL=1 期间，SDA 由低变高，表示本次通信结束
 */
void tp_i2c_stop(void)
{
	//保证SDA引脚为输出模式
	tp_sda_pin_mode(GPIO_Mode_OUT);

	TP_SCL_W=1;   /* SCL 拉高 */
	TP_SDA_W=0;   /* SDA 先保持低 */
	delay_us(1);

	TP_SDA_W=1;   /* SCL 高时 SDA 上升沿 = STOP 信号，释放总线 */
	delay_us(1);
}

/**
 * @brief   I2C 主机发送一个字节(8 bit，MSB 在先)
 * @param   byte  要发送的字节数据
 * @note    每一位先在 SCL=0 期间输出到 SDA，然后 SCL 拉高让从机采样，
 *          SCL 再拉低进入下一位；共 8 个 SCL 脉冲
 */
void tp_i2c_send_byte(uint8_t byte)
{
	int32_t i;

	//保证SDA引脚为输出模式
	tp_sda_pin_mode(GPIO_Mode_OUT);

	TP_SCL_W=0;    /* SCL 拉低，准备传输 */
	TP_SDA_W=0;
	delay_us(1);

	for(i=7; i>=0; i--)   /* 从最高位 bit7 开始发送 */
	{
		if(byte & (1<<i))
			TP_SDA_W=1;   /* 该位为 1，SDA 输出高 */
		else
			TP_SDA_W=0;   /* 该位为 0，SDA 输出低 */

		delay_us(1);

		TP_SCL_W=1;       /* SCL 上升沿，从机采样 SDA */
		delay_us(1);

		TP_SCL_W=0;       /* SCL 下降沿，准备下一位 */
		delay_us(1);
	}
}

/**
 * @brief   主机发送应答/非应答位(ACK/NACK)
 * @param   ack  0=发送 ACK(继续读)，1=发送 NACK(读最后一字节)
 * @note    I2C 读操作时主机需要在每个字节后给出应答；最后一字节发 NACK 通知从机停止发送
 */
void tp_i2c_ack(uint8_t ack)
{
	//保证SDA引脚为输出模式
	tp_sda_pin_mode(GPIO_Mode_OUT);

	TP_SCL_W=0;    /* SCL 拉低，准备驱动 SDA */
	TP_SDA_W=0;
	delay_us(1);

	if(ack)
		TP_SDA_W=1;   /* ack=1 -> 输出 NACK(SDA 高) */
	else
		TP_SDA_W=0;   /* ack=0 -> 输出 ACK(SDA 低) */

	delay_us(1);

	TP_SCL_W=1;    /* SCL 上升沿，从机读取应答位 */
	delay_us(1);

	TP_SCL_W=0;    /* SCL 下降沿，结束应答 */
	delay_us(1);
}

/**
 * @brief   主机等待从机应答位(ACK/NACK)
 * @retval  1: 收到 NACK(SDA 为高，从机未应答)
 *          0: 收到 ACK(SDA 为低，从机正常应答)
 * @note    发送完一个字节后调用，在第 9 个 SCL 脉冲读取 SDA
 */
uint8_t tp_i2c_wait_ack(void)
{
	uint8_t ack;
	//保证SDA引脚为输入模式
	tp_sda_pin_mode(GPIO_Mode_IN);

	//紧接着第九个时钟周期，将SCL拉高
	TP_SCL_W=1;   /* SCL 拉高，读取从机应答 */
	delay_us(1);

	if(TP_SDA_R)
		ack=1;   /* SDA 高 -> NACK */
	else
		ack=0;   /* SDA 低 -> ACK */

	//继续保持占用总线
	TP_SCL_W=0;   /* SCL 拉低，结束应答时序 */
	delay_us(1);

	return ack;
}

/**
 * @brief   主机从 I2C 接收一个字节
 * @retval  接收到的 8 位数据(MSB 在先)
 * @note    主机在 SCL 高电平时读取 SDA，逐位拼装字节
 */
uint8_t tp_i2c_recv_byte(void)
{

	uint8_t d=0;
	int32_t i;


	//保证SDA引脚为输入模式
	tp_sda_pin_mode(GPIO_Mode_IN);


	for(i=7; i>=0; i--)   /* 从最高位 bit7 开始接收 */
	{
		TP_SCL_W=1;   /* SCL 拉高，从机驱动 SDA，主机采样 */
		delay_us(1);

		if(TP_SDA_R)
			d|=1<<i;   /* SDA 高 -> 该位置 1 */

		//继续保持占用总线
		TP_SCL_W=0;   /* SCL 拉低，准备下一位 */
		delay_us(1);
	}

	return d;
}




/**
 * @brief   触摸屏底层初始化：GPIO/EXTI/NVIC 配置
 * @note    根据 TP_PIN_DEF 选择两种引脚定义：
 *          - TP_PIN_DEF_1：SCL=PD0, SDA=PD14, RST=PD4, INT=PF12(EXTI_Line12)
 *          - TP_PIN_DEF_2：SCL=PD6, SDA=PD7, RST=PC6, INT=PC8(EXTI_Line8)
 *          INT 引脚配置为上拉输入，下降沿触发，触摸时 CST816 拉低 INT
 *          中断优先级使用 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 *          以便在中断中可以安全调用 FreeRTOS API
 */
void tp_lowlevel_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;

#if TP_PIN_DEF == TP_PIN_DEF_1

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  /* 使能 GPIOD 时钟(SCL/SDA/RST) */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);  /* 使能 GPIOF 时钟(INT=PF12) */

	//使能系统配置的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  /* 使能 SYSCFG，EXTI 配置需要 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_4|GPIO_Pin_14;		//第0 4 14根引脚 /* PD0=SCL, PD4=RST, PD14=SDA */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//设置输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;				//开漏模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//不需要上拉电阻
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	//只要是输出模式，有初始电平状态
	TP_SCL_W=1;   /* SCL 空闲拉高 */
	TP_SDA_W=1;   /* SDA 空闲拉高 */


	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     /* INT 引脚上拉，避免悬空误触发 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;     /* INT 配置为输入 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;         /* INT = PF12 */
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOF , &GPIO_InitStructure);

	/* 配置外部中断12相关的参数 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;	//外部中断12
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //检测按键的按下，使用下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能
	EXTI_Init(&EXTI_InitStructure);

	/* NVIC打开外部中断12的通道，并为它配置优先级 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应(子)优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//打开该通道
	NVIC_Init(&NVIC_InitStructure);
	//初始化
	TP_RST=1;   /* 复位引脚默认拉高，芯片不处于复位状态 */

#endif


#if TP_PIN_DEF == TP_PIN_DEF_2

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  /* 使能 GPIOC 时钟(RST/INT) */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  /* 使能 GPIOD 时钟(SCL/SDA) */

	//使能系统配置的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  /* 使能 SYSCFG */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;		//第6 7根引脚 /* PD6=SCL, PD7=SDA */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//设置输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;				//开漏模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//不需要上拉电阻
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	//只要是输出模式，有初始电平状态
	TP_SCL_W=1;   /* SCL 空闲拉高 */
	TP_SDA_W=1;   /* SDA 空闲拉高 */

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     /* RST 引脚用推挽输出 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;            /* RST = PC6 */
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC , &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       /* INT 引脚上拉 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       /* INT 配置为输入 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;            /* INT = PC8 */
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC , &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource8);  /* 把 EXTI_Line8 映射到 PC8 */

	/* 配置外部中断8相关的参数 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;	//外部中断8
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //检测按键的按下，使用下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能
	EXTI_Init(&EXTI_InitStructure);

	/* NVIC打开外部中断0的通道，并为它配置优先级 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应(子)优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//打开该通道
	NVIC_Init(&NVIC_InitStructure);

	//初始化
	TP_RST=1;   /* 复位引脚默认拉高，芯片不处于复位状态 */

#endif
}

/**
 * @brief   向 CST816 写入一个寄存器
 * @param   addr  寄存器地址
 * @param   data  指向待写入字节的指针
 * @note    设备写地址 0x2A(7 位地址 0x15 左移 1 位，最低位 0 表示写)
 *          时序：START -> 写地址(0x2A) -> 寄存器地址 -> 数据 -> STOP
 */
void tp_send_byte(uint8_t addr,uint8_t* data)
{
	tp_i2c_start();                                  /* START 信号 */
	tp_i2c_send_byte(0x2A);tp_i2c_wait_ack();         /* 发送设备写地址 0x2A，等待 ACK */
	tp_i2c_send_byte(addr);tp_i2c_wait_ack();         /* 发送寄存器地址，等待 ACK */
	tp_i2c_send_byte(*data);tp_i2c_wait_ack();        /* 发送数据字节，等待 ACK */
	tp_i2c_stop();                                    /* STOP 信号 */
}

/**
 * @brief   从 CST816 读取一个寄存器
 * @param   addr  寄存器地址
 * @param   data  指向存放读取结果的字节指针
 * @note    设备写地址 0x2A，设备读地址 0x2B
 *          时序：写寄存器地址后重新 START，切换为读模式读 1 字节并回 NACK
 */
void tp_recv_byte(uint8_t addr,uint8_t* data)
{
	tp_i2c_start();
	tp_i2c_send_byte(0x2A);tp_i2c_wait_ack();         /* 设备写地址 + 寄存器地址，告诉从机要读哪个寄存器 */
	tp_i2c_send_byte(addr);tp_i2c_wait_ack();
	tp_i2c_start();                                  /* 重复 START，切换为读方向 */
	tp_i2c_send_byte(0x2B);tp_i2c_wait_ack();         /* 发送设备读地址 0x2B */
	*data=tp_i2c_recv_byte();                         /* 读取一个字节 */
	tp_i2c_ack(1);                                    /* 回复 NACK，通知从机结束读 */
	tp_i2c_stop();                                    /* STOP 信号 */
}

/**
 * @brief   从 CST816 连续读取多个字节
 * @param   addr  起始寄存器地址
 * @param   data  存放读取结果的缓冲区
 * @param   len   要读取的字节数
 * @note    除最后一字节回 NACK 外，前面每字节都回 ACK(连续读)
 */
void tp_recv(uint8_t addr,uint8_t* data,uint32_t len)
{
	uint8_t *p=data;

	tp_i2c_start();
	tp_i2c_send_byte(0x2A);tp_i2c_wait_ack();         /* 设备写地址 */
	tp_i2c_send_byte(addr);tp_i2c_wait_ack();         /* 起始寄存器地址 */
	tp_i2c_start();                                  /* 重复 START 切换读方向 */
	tp_i2c_send_byte(0x2B);tp_i2c_wait_ack();         /* 设备读地址 */

	len=len-1;   /* 留出最后一字节稍后单独处理 */

	while(len--)
	{
		*p=tp_i2c_recv_byte();   /* 读取一个字节 */
		tp_i2c_ack(0);           /* 回复 ACK，继续读下一字节 */
		p++;

	}

	*p=tp_i2c_recv_byte();   /* 读取最后一字节 */
	tp_i2c_ack(1);           /* 回复 NACK，结束读操作 */

	tp_i2c_stop();           /* STOP 信号 */
}

/**
 * @brief   硬件复位 CST816 触摸芯片
 * @note    复位时序：RST 拉低保持 10ms -> 拉高 -> 等待 60ms 让芯片完成内部初始化
 */
void tp_reset(void)
{
	TP_RST=0;       /* 拉低复位引脚，进入复位状态 */
	delay_ms(10);   /* 保持 10ms */

	TP_RST=1;       /* 拉高复位引脚，释放复位 */
	delay_ms(60);   /* 等待 60ms 让 CST816 完成上电初始化 */
}

/**
 * @brief   触摸屏初始化：底层初始化 + 复位 + 读取 ChipID/FwVersion 用于校验
 * @note    通过寄存器 0xA7 读取 ChipID，0xA9 读取固件版本
 *          根据 ChipID 可识别具体型号(CST716/CST816S/CST816T/CST816D)
 */
void tp_init(void)
{
	uint8_t Data=0;
	uint8_t ChipID=0;
	uint8_t FwVersion=0;


	tp_lowlevel_init();   /* GPIO/EXTI/NVIC 初始化 */

	tp_reset();//芯片上电初始化
	tp_recv_byte(0xa7,&ChipID);     /* 读取芯片 ID 寄存器 0xA7 */
	tp_recv_byte(0xa9,&FwVersion);  /* 读取固件版本寄存器 0xA9 */

	printf("ChipID:%02X\r\n",ChipID);

	/*
		芯片 ID
		CST716 : 0x20
		CST816S : 0xB4
		CST816T : 0xB5
		CST816D : 0xB6

	*/

	printf("FwVersion:%02X\r\n",FwVersion);

}


/**
 * @brief   获取当前触摸手指数量
 * @retval  g_tp_finger_num 最近一次 tp_read 更新的手指数量
 */
uint8_t tp_finger_num_get(void)
{
	return g_tp_finger_num;
}

/**
 * @brief   读取触摸坐标和手势，并根据屏幕方向做坐标旋转
 * @param   screen_x  输出参数，旋转后的屏幕 X 坐标
 * @param   screen_y  输出参数，旋转后的屏幕 Y 坐标
 * @retval  手势代码 buf[1]：0x00 无手势 / 0x01 下滑 / 0x02 上滑 / 0x03 左滑 / 0x04 右滑
 *          / 0x05 单击 / 0x0B 双击 / 0x0C 长按；坐标越界时返回 0
 * @note    从寄存器 0x00 起连读 7 字节：[0]状态 [1]手势 [2]手指数
 *          [3][4] X 坐标(高 4 位在 buf[3] 低 4 位，低 8 位在 buf[4])
 *          [5][6] Y 坐标(同上)
 *          根据屏幕方向 1/2/3 对坐标做 90/180/270 度旋转
 */
uint8_t tp_read(uint16_t *screen_x,uint16_t *screen_y)
{
	uint8_t buf[7];
	uint16_t x=0,y=0,tmp;

	tp_recv(0,buf,7);   /* 从 0x00 起连读 7 字节 */

	x=(uint16_t)((buf[3]&0x0F)<<8)|buf[4];   /* 拼接 X 坐标(12 位) */
	y=(uint16_t)((buf[5]&0x0F)<<8)|buf[6];   /* 拼接 Y 坐标(12 位) */

	g_tp_finger_num = buf[2];   /* 更新手指数量 */

	if((x<g_lcd_width) && (y<g_lcd_height))   /* 坐标在屏幕范围内才有效 */
	{

		if(lcd_get_direction()==1)   /* 方向 1：屏幕旋转 90 度 */
		{
		   tmp= x;
			x = y;
			y = g_lcd_height-tmp;     /* X<->Y 互换，Y 翻转 */
		}

		if(lcd_get_direction()==2)   /* 方向 2：屏幕旋转 180 度 */
		{
			x = g_lcd_width-x;        /* X 翻转 */
			y = g_lcd_height-y;       /* Y 翻转 */
		}

		if(lcd_get_direction()==3)   /* 方向 3：屏幕旋转 270 度 */
		{
		   tmp= y;
			y = x;
			x = g_lcd_width-tmp;     /* X<->Y 互换，X 翻转 */
		}

		*screen_x=x;   /* 输出旋转后的 X */
		*screen_y=y;   /* 输出旋转后的 Y */

		/*
			0x00：无手势
			0x01：下滑
			0x02：上滑
			0x03：左滑
			0x04：右滑
			0x05：单击
			0x0B：双击
			0x0C：长按
		*/

		return buf[1];   /* 返回手势代码 */
	}

	return 0;   /* 坐标越界，返回 0 */
}


#if TP_PIN_DEF == TP_PIN_DEF_1
/**
 * @brief   外部中断 10-15 公共中断服务程序(TP_PIN_DEF_1 配置下使用 EXTI_Line12)
 * @note    CST816 检测到触摸时拉低 INT(PF12)，触发 EXTI_Line12 下降沿中断
 *          在中断里仅置位 g_tp_event=1，真正的 tp_read 放到任务上下文执行，
 *          避免软件 I2C 拉长中断响应时间，导致其他中断延迟
 */
void EXTI15_10_IRQHandler(void)
{
	uint32_t ulReturn;

	/* 进入中断临界区 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();

	//获取外部中断12是否有触发
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		/* 进行事件处理*/
		g_tp_event=1;   /* 置位触摸事件标志，通知任务读取坐标 */

		/* tp_read在中断调用实时性会更好，但会导致其他中断延迟 */
		//tp_read(&g_tp_x,&g_tp_y);

		/* 清除外部中断12的标志位，告诉CPU已经处理完毕 */
		EXTI_ClearITPendingBit(EXTI_Line12);   /* 清除 EXTI_Line12 挂起位 */
	}

	/* 退出中断临界区*/
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}
#endif

#if TP_PIN_DEF == TP_PIN_DEF_2
/**
 * @brief   外部中断 5-9 公共中断服务程序(TP_PIN_DEF_2 配置下使用 EXTI_Line8)
 * @note    CST816 检测到触摸时拉低 INT(PC8)，触发 EXTI_Line8 下降沿中断
 *          中断里仅置位 g_tp_event=1，真正的 tp_read 放到任务上下文执行
 */
void EXTI9_5_IRQHandler(void)
{
	uint32_t ulReturn;

	/* 进入中断临界区 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();

	//获取外部中断8是否有触发
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		/* 进行事件处理*/

		g_tp_event=1;   /* 置位触摸事件标志，通知任务读取坐标 */

		/* tp_read在中断调用实时性会更好，但会导致其他中断延迟 */
		//tp_read(&g_tp_x,&g_tp_y);

		/* 清除外部中断8的标志位，告诉CPU已经处理完毕 */
		EXTI_ClearITPendingBit(EXTI_Line8);   /* 清除 EXTI_Line8 挂起位 */
	}
	/* 退出中断临界区*/
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}
#endif