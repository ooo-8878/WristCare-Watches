/**
 * @file    i2c.c
 * @brief   MAX30102 心率血氧传感器所使用的软件模拟 I2C 通信驱动
 *
 * @details 本模块采用 GPIO 软件位带(bit-band)方式模拟标准 I2C 主机时序，
 *          专供 MAX30102 心率血氧传感器使用。不占用 STM32 硬件 I2C 外设资源。
 *
 *          硬件资源说明：
 *          - 通信端口：GPIOC
 *          - SCL(时钟线)：PC7   (宏 SCL_W = PCout(7)，位带输出)
 *          - SDA(数据线)：PC9   (宏 SDA_W = PCout(9)，位带输出；SDA_R = PCin(9)，位带输入)
 *          - GPIO 输出类型：开漏输出(GPIO_OType_OD)
 *          - 上下拉配置：无上下拉(GPIO_PuPd_NOPULL)，依赖外部上拉电阻实现高电平
 *          - 引脚速度：50MHz
 *
 *          I2C 协议时序要点：
 *          - 空闲状态：SCL=1，SDA=1(由外部上拉电阻拉高)
 *          - 起始信号：SCL 为高电平期间，SDA 由高变低
 *          - 停止信号：SCL 为高电平期间，SDA 由低变高
 *          - 数据传输：MSB(最高有效位)优先，每个 SCL 正脉冲期间采样 SDA
 *          - 应答机制：每发送 8 位数据后，主机释放 SDA 等待从机拉低应答(ACK)
 *
 *          与 FreeRTOS 的交互方式：
 *          - 本驱动为底层裸机时序，不直接调用 FreeRTOS API
 *          - 上层 MAX30102 采集任务通过队列/信号量保护对 I2C 总线的访问
 *          - 时序中使用 delay_us/delay_ms 进行延时，调用时需确保不被高优先级任务
 *            长时间抢占，否则会导致 I2C 时序异常
 *
 *          设计要点：
 *          - 采用开漏输出 + 外部上拉电阻符合 I2C 总线规范，支持"线与"功能
 *          - SDA 方向动态切换(sda_pin_mode)以实现主机发送/接收切换
 *          - delay_us 延时约控制 SCL 频率在 100kHz~400kHz 之间
 */
#include "includes.h"

/**
 * @brief   初始化软件模拟 I2C 的 GPIO 端口
 * @details 配置 PC7(SCL) 与 PC9(SDA) 为开漏输出模式，并将总线置于空闲高电平状态。
 *          开漏输出必须配合外部上拉电阻才能输出高电平，符合 I2C 总线规范。
 * @note    该函数仅需在系统启动时调用一次。开漏模式下输出"1"实际是释放总线，
 *          由外部上拉电阻拉高；输出"0"为主动拉低。
 */
void i2c_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//使能端口B的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* 使能 GPIOC 端口硬件时钟，PC7/PC9 才能工作 */

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_9;	//第 7 9个引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;		    //输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;		//引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;		//开漏
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;		//不需要上下拉电阻
	GPIO_Init(GPIOC,&GPIO_InitStructure);	/* 调用标准外设库完成 GPIOC 第7、9引脚的硬件配置 */
	
	
	//只要是输出模式，肯定会有初始电平状态，看时序图，空闲状态为高电平
	SCL_W=1;	/* SCL 拉高，开漏输出"1"即释放总线，由外部上拉电阻拉高 */
	SDA_W=1;	/* SDA 拉高，使总线进入空闲状态(空闲态 SCL=1,SDA=1) */

}

/**
 * @brief   动态切换 SDA(PC9) 引脚的工作方向
 * @details I2C 总线为半双工，主机在发送时 SDA 需为输出模式，在接收/等待应答时
 *          需切换为输入模式以释放 SDA 让从机驱动。本函数通过重新初始化 GPIO 实现
 *          方向切换，保持开漏输出特性不变。
 * @param   pin_mode  目标工作模式：GPIO_Mode_OUT(输出，用于发送) 或 GPIO_Mode_IN(输入，用于接收)
 * @note    每次 SDA 方向切换都会重新配置 GPIO 寄存器，开销略大于直接操作 MODER
 *          寄存器，但代码清晰且对时序影响在可接受范围内。
 */
void sda_pin_mode(GPIOMode_TypeDef pin_mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//配置硬件，配置GPIO，端口F，第9个引脚
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;			//第7 个引脚
	GPIO_InitStructure.GPIO_Mode=pin_mode;				//输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_25MHz;		//引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;		//开漏
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;		//不需要上下拉电阻
	GPIO_Init(GPIOC,&GPIO_InitStructure);	/* 仅对 PC9(SDA) 按传入模式重新初始化，SCL 不受影响 */
}


/**
 * @brief   产生 I2C 起始信号(START)
 * @details 起始信号时序：在 SCL 保持高电平期间，SDA 由高电平跳变为低电平。
 *          随后将 SCL 拉低，进入数据传输阶段(总线忙状态)。
 *          时序要求：SDA 下降沿必须发生在 SCL 高电平期间。
 * @note    调用前总线应处于空闲状态(SCL=1,SDA=1)。本函数会先把 SDA 切为输出模式。
 */
void i2c_start(void)
{
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);

	SDA_W=1;	/* 先把 SDA 拉高，确保起始条件前 SDA 为高电平 */
	SCL_W=1;	/* SCL 拉高，准备在 SCL 高电平期间产生 SDA 下降沿 */
	delay_us(1);	/* 保持 SCL 高电平一小段时间，使信号稳定 */

	SDA_W=0;	/* SCL 为高时，SDA 由高变低，产生起始信号 */
	delay_us(1);	/* 起始信号保持时间，满足 I2C 规范的建立时间 */

	SCL_W=0;//总线进入忙状态
	delay_us(1);	/* SCL 拉低，钳住总线，准备开始发送/接收数据 */
}


/**
 * @brief   产生 I2C 停止信号(STOP)
 * @details 停止信号时序：在 SCL 保持高电平期间，SDA 由低电平跳变为高电平。
 *          调用前需确保 SDA 为低电平，再拉高 SCL，最后将 SDA 拉高产生上升沿。
 *          时序要求：SDA 上升沿必须发生在 SCL 高电平期间。
 * @note    停止信号释放总线，使总线回到空闲状态(SCL=1,SDA=1)。
 */
void i2c_stop(void)
{
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);

	SDA_W=0;	/* 先将 SDA 拉低，为停止信号的上升沿做准备 */
	SCL_W=1;	/* SCL 拉高，准备在 SCL 高电平期间产生 SDA 上升沿 */
	delay_us(1);	/* 保持 SCL 高电平，使 SDA 低电平稳定 */

	SDA_W=1;	/* SCL 为高时，SDA 由低变高，产生停止信号，释放总线 */
	delay_us(1);	/* 停止信号保持时间，满足 I2C 规范 */

}


/**
 * @brief   通过 I2C 发送一个字节(8bit)
 * @details 采用 MSB(最高有效位)优先的方式逐位发送。每个比特的发送过程：
 *          1. SCL 为低时，将数据位写入 SDA；
 *          2. SCL 拉高，从机在此期间采样 SDA(数据有效)；
 *          3. SCL 拉低，准备发送下一位。
 *          循环 8 次完成一个字节的发送。发送完成后 SDA 仍保持最后一位状态，
 *          由调用方决定是否释放 SDA 等待应答。
 * @param   byte  待发送的字节数据
 * @note    本函数不处理应答，调用后需调用 i2c_wait_ack() 等待从机应答。
 */
void i2c_send_byte(uint8_t byte)
{
	int32_t i=0;

	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);

	SDA_W=0;	/* 发送前先将 SDA 拉低(清理总线状态) */
	SCL_W=0;	/* SCL 拉低，进入数据变更阶段 */
	delay_us(1);	/* 等待电平稳定 */

	//最高有效位优先传输，通过时序图观察到
	for(i=7; i>=0; i--)	/* i 从 7 递减到 0，即从 bit7(MSB) 到 bit0(LSB) 逐位发送 */
	{
		//检测对应的bit位是1还是0
		if(byte & (1<<i))	/* 通过掩码判断第 i 位是否为 1 */
			SDA_W=1;	/* 该位为 1，SDA 输出高电平 */
		else
			SDA_W=0;	/* 该位为 0，SDA 输出低电平 */

		delay_us(1);	/* SDA 数据保持时间，等待电平稳定 */

		//时钟线拉高，数据有效
		SCL_W=1;	/* SCL 拉高，从机在此期间锁存/采样 SDA 上的数据 */
		delay_us(1);	/* SCL 高电平保持时间，确保从机完成采样 */


		//时钟线拉低，数据变更
		SCL_W=0;	/* SCL 拉低，允许 SDA 数据变更，准备发送下一位 */
		delay_us(1);		/* SCL 低电平保持时间 */

	}
}


/**
 * @brief   通过 I2C 接收一个字节(8bit)
 * @details 采用 MSB(最高有效位)优先的方式逐位接收。每个比特的接收过程：
 *          1. 主机将 SCL 拉高，从机在此期间驱动 SDA 输出数据位；
 *          2. 主机读取 SDA 引脚电平并组装到结果中；
 *          3. 主机将 SCL 拉低，触发从机输出下一位。
 *          循环 8 次完成一个字节的接收。
 * @retval  接收到的 8bit 字节数据
 * @note    调用前 SDA 需切换为输入模式。本函数不发送应答，
 *          调用方应在接收后调用 i2c_ack() 进行应答/非应答。
 */
uint8_t i2c_recv_byte(void)
{
	uint8_t d=0;	/* 接收数据缓冲，初值为 0 */
	int32_t i;

	//保证SDA引脚为输入模式
	sda_pin_mode(GPIO_Mode_IN);	/* 切换为输入，释放 SDA 由从机驱动 */

	for(i=7; i>=0; i--)	/* 从 bit7(MSB) 到 bit0(LSB) 逐位接收 */
	{
		//时钟线拉高，数据有效
		SCL_W=1;	/* SCL 拉高，从机在此期间将数据位输出到 SDA */
		delay_us(1);	/* 等待电平稳定后采样 */

		//读取SDA引脚电平
		if(SDA_R)	/* 读取 PC9 引脚电平 */
			d|=1<<i;	/* SDA 为高，则将结果的第 i 位置 1 */


		//时钟线拉低，保持占用总线，总线是忙状态
		SCL_W=0;	/* SCL 拉低，触发从机输出下一位，同时钳住总线 */
		delay_us(1);	/* SCL 低电平保持时间 */

	}

	return d;	/* 返回组装完成的 8bit 字节 */
}


/**
 * @brief   主机向从机发送应答(ACK)或非应答(NACK)信号
 * @details 主机接收完一个字节后需向从机反馈应答状态：
 *          - 发送 ACK(ack=0)：将 SDA 拉低，表示继续接收下一字节；
 *          - 发送 NACK(ack=1)：将 SDA 拉高，表示接收结束，准备产生停止信号。
 *          应答时序：SCL 为低时设置 SDA，SCL 拉高产生应答脉冲，SCL 再拉低。
 * @param   ack  0:发送应答 ACK(继续读)；1:发送非应答 NACK(结束读)
 * @note    注意本函数参数语义：ack=1 表示 NACK，ack=0 表示 ACK。
 */
void i2c_ack(uint8_t ack)
{

	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);	/* 切换为输出，主机驱动 SDA 发送应答位 */

	SDA_W=0;	/* 先将 SDA 拉低作为默认准备状态 */
	SCL_W=0;	/* SCL 拉低，进入数据变更阶段 */
	delay_us(1);	/* 等待电平稳定 */


	if(ack)
		SDA_W=1;	/* ack=1，发送 NACK，SDA 拉高(非应答) */
	else
		SDA_W=0;	/* ack=0，发送 ACK，SDA 保持低电平(应答) */

	delay_us(1);	/* SDA 数据保持时间 */

	//时钟线拉高，数据有效
	SCL_W=1;	/* SCL 拉高，产生应答脉冲，从机在此期间采样 SDA */
	delay_us(1);	/* SCL 高电平保持时间 */


	//时钟线拉低，数据变更
	SCL_W=0;	/* SCL 拉低，结束应答脉冲 */
	delay_us(1);		/* SCL 低电平保持时间 */
}

/**
 * @brief   主机等待从机应答信号(ACK)
 * @details 主机发送完一个字节(含地址字节)后，需释放 SDA 并切换为输入模式，
 *          然后产生一个 SCL 正脉冲，在此期间采样 SDA：
 *          - SDA=0：从机应答(ACK)，返回 0；
 *          - SDA=1：从机非应答(NACK)，返回 1。
 * @retval  0:从机应答成功(ACK)；1:从机无应答(NACK)
 * @note    本函数无超时退出机制，若从机异常可能导致阻塞，调用方需注意。
 */
uint8_t i2c_wait_ack(void)
{
	uint8_t ack=0;
	//保证SDA引脚为输入模式
	sda_pin_mode(GPIO_Mode_IN);	/* 切换为输入，释放 SDA 由从机驱动 */

	//时钟线拉高，数据有效
	SCL_W=1;	/* SCL 拉高，产生应答检测脉冲 */
	delay_us(1);	/* 等待电平稳定后采样 */

	//读取SDA引脚电平
	if(SDA_R)	/* 读取 PC9(SDA) 引脚电平 */
		ack=1;//无应答
	else
		ack=0;//有应答

	//时钟线拉低，保持占用总线，总线是忙状态
	SCL_W=0;	/* SCL 拉低，结束应答脉冲，钳住总线 */
	delay_us(1);


	return ack;	/* 返回应答状态：0=ACK, 1=NACK */
}

/**
 * @brief   I2C 连续写入多个字节(用于 MAX30102 寄存器批量配置)
 * @details 完整写时序：START -> 发送器件写地址 -> 等待ACK -> 依次发送数据字节
 *          (每字节等待ACK) -> STOP。适用于向同一器件连续写入多字节配置数据。
 * @param   WriteAddr    器件地址(已含写方向位，即最低位为0)
 * @param   data         待写入数据缓冲区指针
 * @param   dataLength   待写入数据字节数
 * @note    写入完成后延时 10ms，确保从机内部完成数据存储/处理。
 */
void IIC_WriteBytes(u8 WriteAddr,u8* data,u8 dataLength)
{
	u8 i;
    i2c_start();	/* 产生起始信号，启动一次写事务 */

	i2c_send_byte(WriteAddr);	    //发送写命令
	i2c_wait_ack();	/* 等待从机应答器件地址 */

	for(i=0;i<dataLength;i++)	/* 循环发送 dataLength 个字节 */
	{
		i2c_send_byte(data[i]);	/* 发送第 i 个数据字节 */
		i2c_wait_ack();	/* 等待从机应答该字节 */
	}
    i2c_stop();//产生一个停止条件
	delay_ms(10);	/* 写入后延时 10ms，等待从机完成内部写入 */
}

/**
 * @brief   I2C 连续读取多个字节(用于 MAX30102 数据批量读取)
 * @details 完整读时序(含寄存器地址写入)：
 *          START -> 发送器件写地址 -> 等待ACK -> 发送寄存器地址 -> 等待ACK ->
 *          再次START(重复起始) -> 发送器件读地址 -> 等待ACK ->
 *          依次接收数据字节(前 N-1 个回ACK，最后一个回NACK) -> STOP。
 * @param   deviceAddr   器件地址(写地址，最低位为0，函数内部会|0x01转为读地址)
 * @param   writeAddr    待读取的起始寄存器地址
 * @param   data         读取数据存放缓冲区指针
 * @param   dataLength   待读取的字节数
 * @note    最后一个字节读取后发送 NACK，通知从机停止输出，随后产生停止信号。
 */
void IIC_ReadBytes(u8 deviceAddr, u8 writeAddr,u8* data,u8 dataLength)
{
	u8 i;
    i2c_start();	/* 产生起始信号，开始写阶段(设定寄存器地址) */

	i2c_send_byte(deviceAddr);	    //发送写命令
	i2c_wait_ack();	/* 等待从机应答器件写地址 */
	i2c_send_byte(writeAddr);	/* 发送待读取的寄存器地址 */
	i2c_wait_ack();	/* 等待从机应答寄存器地址 */
	i2c_send_byte(deviceAddr|0X01);//进入接收模式
	i2c_wait_ack();	/* 等待从机应答器件读地址 */

	for(i=0;i<dataLength-1;i++)	/* 读取前 dataLength-1 个字节 */
	{
		data[i] = i2c_recv_byte();	/* 接收第 i 个字节 */
		i2c_ack(0);	/* 发送 ACK，通知从机继续输出下一字节 */
	}
	data[dataLength-1] = i2c_recv_byte();	/* 接收最后一个字节 */
	i2c_ack(1);	/* 发送 NACK，通知从机读取结束 */
    i2c_stop();//产生一个停止条件
	delay_ms(10);	/* 读取后延时 10ms */
}

/**
 * @brief   I2C 读取单个字节(用于 MAX30102 单个寄存器读取)
 * @details 完整读单字节时序：
 *          START -> 发送器件写地址 -> 等待ACK -> 发送寄存器地址 -> 等待ACK ->
 *          重复START -> 发送器件读地址 -> 等待ACK -> 接收1字节(发NACK) -> STOP。
 * @param   daddr  器件地址(写地址，最低位为0，函数内部会|0x01转为读地址)
 * @param   addr   待读取的寄存器地址
 * @param   data   读取数据存放指针
 * @note    单字节读取后必须发送 NACK，再产生停止信号，以正确终止本次读事务。
 */
void IIC_Read_One_Byte(u8 daddr,u8 addr,u8* data)
{
    i2c_start();	/* 产生起始信号，开始写阶段(设定寄存器地址) */

	i2c_send_byte(daddr);	   //发送写命令
	i2c_wait_ack();	/* 等待从机应答器件写地址 */
	i2c_send_byte(addr);//发送地址
	i2c_wait_ack();	/* 等待从机应答寄存器地址 */
	i2c_start();	/* 重复起始信号，切换为读方向 */
	i2c_send_byte(daddr|0X01);//进入接收模式
	i2c_wait_ack();	/* 等待从机应答器件读地址 */
    *data = i2c_recv_byte();	/* 接收单字节数据 */
	i2c_ack(1);	/* 发送 NACK，通知从机只读一个字节 */
    i2c_stop();//产生一个停止条件
}

/**
 * @brief   I2C 写入单个字节(用于 MAX30102 单个寄存器配置)
 * @details 完整写单字节时序：
 *          START -> 发送器件写地址 -> 等待ACK -> 发送寄存器地址 -> 等待ACK ->
 *          发送数据字节 -> 等待ACK -> STOP。
 * @param   daddr  器件地址(写地址，最低位为0)
 * @param   addr   待写入的寄存器地址
 * @param   data   待写入的单字节数据
 * @note    写入完成后延时 10ms，确保从机完成内部寄存器写入操作。
 */
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data)
{
    i2c_start();	/* 产生起始信号，启动一次写事务 */

	i2c_send_byte(daddr);	    //发送写命令
	i2c_wait_ack();	/* 等待从机应答器件写地址 */
	i2c_send_byte(addr);//发送地址
	i2c_wait_ack();	/* 等待从机应答寄存器地址 */
	i2c_send_byte(data);     //发送字节
	i2c_wait_ack();	/* 等待从机应答数据字节 */
    i2c_stop();//产生一个停止条件
	delay_ms(10);	/* 写入后延时 10ms，等待从机完成内部写入 */
}
