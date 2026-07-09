/**
 * @file    MPU6050.c
 * @brief   MPU6050 六轴传感器(三轴加速度+三轴陀螺仪)驱动及软件模拟 I2C 时序实现
 *
 * @details 本模块包含两部分功能：
 *          1. 软件模拟 I2C 主机时序(IIC_Init/IIC_Start/IIC_Stop/IIC_Send_Byte 等)
 *          2. MPU6050 寄存器配置与数据读取(MPU_Init/MPU_Get_Gyroscope 等)
 *
 *          硬件资源说明：
 *          - 通信端口：GPIOB
 *          - SCL(时钟线)：PB8   (宏 IIC_SCL = PBout(8)，位带输出)
 *          - SDA(数据线)：PB9   (宏 IIC_SDA = PBout(9)，位带输出；READ_SDA = PBin(9)，位带输入)
 *          - GPIO 输出类型：开漏输出(GPIO_OType_OD)
 *          - 上下拉配置：上拉(GPIO_PuPd_UP)
 *          - 引脚速度：100MHz
 *          - MPU6050 器件地址：0x68(AD0 引脚接地时)
 *
 *          I2C 时序要点：
 *          - SDA 方向切换通过直接操作 GPIOB->MODER 寄存器实现(SDA_IN/SDA_OUT 宏)
 *            比 GPIO_Init 重新初始化更高效
 *          - 起始/停止信号：SCL 高电平期间 SDA 跳变
 *          - 数据传输：MSB 优先，SCL 高电平采样
 *          - 应答机制：带超时检测(250 次循环)防止死锁
 *
 *          与 FreeRTOS 的交互方式：
 *          - 本驱动为底层裸机时序，不直接调用 FreeRTOS API
 *          - 在 main.c 的 app_task_mpu6050 任务中被周期性调用
 *          - 实现两大功能：
 *            a) 抬腕亮屏：通过加速度计检测手腕抬起动作，唤醒屏幕
 *            b) 步数统计：通过加速度计/陀螺仪数据算法计步
 *          - 调用时应注意 I2C 时序不被高优先级任务抢占导致通信异常
 *
 *          设计要点：
 *          - 采用开漏输出 + 内部上拉，简化外部电路
 *          - IIC_Wait_Ack 带超时保护，避免从机异常导致任务永久阻塞
 *          - 初始化时通过读取 WHO_AM_I 寄存器(0x75)校验器件身份
 */
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "mpu6050.h"
#include "stdio.h"

//IO方向设置
/* SDA(PB9)方向切换宏：直接操作 GPIOB->MODER 寄存器，bit[19:18]控制 PB9 模式 */
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式

//IO操作函数
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA
#define READ_SDA   PBin(9)  //输入SDA 


/**
 * @brief   初始化 MPU6050 所用软件模拟 I2C 的 GPIO 端口
 * @details 配置 PB8(SCL) 与 PB9(SDA) 为开漏输出 + 内部上拉模式，并将总线置于
 *          空闲高电平状态。开漏输出配合上拉电阻符合 I2C 总线规范。
 * @note    该函数在 MPU_Init 内部被调用，正常使用无需单独调用。
 */
//初始化IIC
void IIC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	//GPIOB8,B9初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	IIC_SCL=1;	/* SCL 拉高(开漏输出1即释放总线)，进入空闲状态 */
	IIC_SDA=1;	/* SDA 拉高，总线空闲态：SCL=1,SDA=1 */

}
/**
 * @brief   产生 I2C 起始信号(START)
 * @details 起始信号时序：SCL 为高电平期间，SDA 由高电平跳变为低电平，
 *          随后将 SCL 拉低以钳住总线，准备开始数据传输。
 *          时序要求：SDA 下降沿必须发生在 SCL 高电平期间。
 * @note    调用前总线应处于空闲状态(SCL=1,SDA=1)。
 */
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	/* SDA 先拉高，确保起始条件前为高电平 */
	IIC_SCL=1;	/* SCL 拉高，准备在 SCL 高电平期间产生 SDA 下降沿 */
	delay_us(4);	/* 保持 SCL 高电平，等待信号稳定 */
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low
	delay_us(4);	/* 起始信号保持时间，满足建立时间要求 */
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
}	  
/**
 * @brief   产生 I2C 停止信号(STOP)
 * @details 停止信号时序：先将 SCL、SDA 拉低，再拉高 SCL，最后在 SCL 高电平期间
 *          将 SDA 由低拉高，产生上升沿。停止信号释放总线，使其回到空闲状态。
 *          时序要求：SDA 上升沿必须发生在 SCL 高电平期间。
 * @note    停止信号后总线恢复空闲(SCL=1,SDA=1)。
 */
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;	/* SCL 先拉低，准备产生停止信号 */
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);	/* 保持 SDA 低电平，等待信号稳定 */
	IIC_SCL=1;	/* SCL 拉高，准备在 SCL 高电平期间产生 SDA 上升沿 */
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);	/* 停止信号保持时间，满足 I2C 规范 */
}
/**
 * @brief   等待从机应答信号(ACK)
 * @details 主机发送完一个字节后，释放 SDA(切为输入)，产生一个 SCL 正脉冲，
 *          在此期间轮询采样 SDA：
 *          - SDA=0：从机应答(ACK)，拉低 SCL 后返回 0；
 *          - SDA=1：从机非应答，循环计数达 250 次后产生停止信号并返回 1(超时保护)。
 * @retval  1:接收应答失败(NACK 或超时)；0:接收应答成功(ACK)
 * @note    带超时保护机制(250 次 ~250us)，避免从机异常导致任务永久阻塞。
 */
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;	/* 超时计数器 */
	SDA_IN();      //SDA设置为输入
	IIC_SDA=1;delay_us(1);	/* 主机释放 SDA(开漏输出1)，等待从机驱动 */
	IIC_SCL=1;delay_us(1);	/* SCL 拉高，产生应答检测脉冲 */
	while(READ_SDA)	/* 循环检测 SDA 电平，直到为低(应答)或超时 */
	{
		ucErrTime++;	/* 超时计数自增 */
		delay_us(1);
		if(ucErrTime>250)	/* 超过 250 次仍未应答，判定为通信失败 */
		{
			IIC_Stop();	/* 产生停止信号，释放总线 */
			return 1;	/* 返回失败标志 */
		}
	}
	IIC_SCL=0;//时钟输出0
	return 0;	/* 返回成功标志：从机已应答 */
}
/**
 * @brief   产生 ACK 应答信号
 * @details 主机接收完一个字节后，向从机发送应答(ACK)，表示继续接收下一字节。
 *          时序：SCL 拉低 -> SDA 拉低(ACK) -> SCL 拉高产生应答脉冲 -> SCL 拉低。
 * @note    ACK 表示主机确认接收，从机将继续输出下一字节数据。
 */
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;	/* SCL 拉低，进入数据变更阶段 */
	SDA_OUT();	/* SDA 切为输出 */
	IIC_SDA=0;	/* SDA 拉低，表示 ACK 应答 */
	delay_us(2);	/* SDA 数据保持时间 */
	IIC_SCL=1;	/* SCL 拉高，产生应答脉冲，从机在此期间采样 SDA */
	delay_us(2);	/* SCL 高电平保持时间 */
	IIC_SCL=0;	/* SCL 拉低，结束应答脉冲 */
}
/**
 * @brief   产生 NACK 非应答信号
 * @details 主机接收完最后一个字节后，向从机发送非应答(NACK)，表示接收结束。
 *          时序：SCL 拉低 -> SDA 拉高(NACK) -> SCL 拉高产生应答脉冲 -> SCL 拉低。
 * @note    NACK 后通常紧跟停止信号(STOP)以终止读操作。
 */
//不产生ACK应答
void IIC_NAck(void)
{
	IIC_SCL=0;	/* SCL 拉低，进入数据变更阶段 */
	SDA_OUT();	/* SDA 切为输出 */
	IIC_SDA=1;	/* SDA 拉高，表示 NACK 非应答 */
	delay_us(2);	/* SDA 数据保持时间 */
	IIC_SCL=1;	/* SCL 拉高，产生应答脉冲，从机在此期间采样 SDA */
	delay_us(2);	/* SCL 高电平保持时间 */
	IIC_SCL=0;	/* SCL 拉低，结束应答脉冲 */
}					 				     
/**
 * @brief   I2C 发送一个字节(8bit)
 * @details 采用 MSB(最高有效位)优先方式逐位发送。每个比特发送过程：
 *          1. SCL 拉低，将数据最高位写入 SDA；
 *          2. SCL 拉高，从机在此期间采样 SDA(数据有效)；
 *          3. SCL 拉低，数据左移一位，准备发送下一比特。
 *          循环 8 次完成一个字节发送。
 * @param   txd  待发送的字节数据
 * @note    本函数不处理应答，调用后需调用 IIC_Wait_Ack() 等待从机应答。
 */
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(u8 txd)
{
    u8 t;
	SDA_OUT(); 	    /* SDA 切为输出模式 */
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)	/* 循环发送 8 个比特 */
    {
        IIC_SDA=(txd&0x80)>>7;	/* 取最高位 bit7 输出到 SDA */
        txd<<=1; 	    /* 数据左移一位，准备下一次取最高位 */
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;	/* SCL 拉高，从机在此期间采样 SDA */
		delay_us(2); 	/* SCL 高电平保持时间 */
		IIC_SCL=0;	/* SCL 拉低，允许 SDA 数据变更 */
		delay_us(2);	/* SCL 低电平保持时间 */
    }
} 	    
/**
 * @brief   I2C 接收一个字节(8bit)
 * @details 采用 MSB(最高有效位)优先方式逐位接收。每个比特接收过程：
 *          1. SCL 拉低，触发从机输出下一比特；
 *          2. SCL 拉高，主机采样 SDA 并组装到结果最高位；
 *          3. 结果左移一位，循环 8 次完成接收。
 *          接收完成后根据 ack 参数决定发送 ACK(继续读)或 NACK(结束读)。
 * @param   ack  1:接收后发送 ACK(继续读下一字节)；0:接收后发送 NACK(结束读)
 * @retval  接收到的 8bit 字节数据
 * @note    调用前 SDA 已切为输入模式。注意此处 ack 语义：ack=1 发 ACK，ack=0 发 NACK。
 */
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )	/* 循环接收 8 个比特 */
    {
        IIC_SCL=0; 	/* SCL 拉低，触发从机输出下一比特 */
        delay_us(2);
		IIC_SCL=1;	/* SCL 拉高，主机在此期间采样 SDA */
        receive<<=1;	/* 接收结果左移一位，为下一比特腾出最低位 */
        if(READ_SDA)receive++;   /* 若 SDA 为高，则最低位置 1 */
		delay_us(1); 	/* SCL 高电平保持时间 */
    }
    if (!ack)	/* ack=0，表示接收结束 */
        IIC_NAck();//发送nACK
    else	/* ack=1，表示继续接收 */
        IIC_Ack(); //发送ACK
    return receive;	/* 返回接收到的字节 */
}

/**
 * @brief   初始化 MPU6050 六轴传感器
 * @details 完整初始化序列：
 *          1. 初始化 I2C GPIO 端口(IIC_Init)
 *          2. 复位 MPU6050(写 0x80 到电源管理寄存器1)，延时 100ms 等待复位完成
 *          3. 唤醒 MPU6050(写 0x00 退出睡眠模式)
 *          4. 设置陀螺仪满量程 ±2000dps
 *          5. 设置加速度计满量程 ±2g
 *          6. 设置采样率 50Hz
 *          7. 关闭所有中断、关闭 I2C 主模式、开启全部 FIFO
 *          8. 设置 INT 引脚低电平有效
 *          9. 读取 WHO_AM_I 寄存器校验器件 ID(应为 0x68)
 *          10. 校验通过后设置时钟源为 PLL X 轴参考，全部传感器工作，采样率提升至 200Hz
 * @retval  0:初始化成功；1:器件 ID 校验失败(通信异常或器件不匹配)
 * @note    初始化失败时直接返回 1，调用方应根据返回值决定后续处理。
 */
u8 MPU_Init(void)
{
	u8 res;
	IIC_Init();//初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(100);	/* 等待 100ms 确保复位完成 */
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0XFF);	//FIFO全开
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);	/* 读取 WHO_AM_I 寄存器(0x75)获取器件 ID */
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(200);						//设置采样率为200Hz
 	}else return 1;	/* ID 校验失败，返回 1 表示初始化失败 */
	return 0;	/* 初始化成功 */
}
/**
 * @brief   设置 MPU6050 陀螺仪满量程范围
 * @details 通过写陀螺仪配置寄存器(0x1B)的 bit[4:3]设置满量程。
 *          参数 fsr 左移 3 位后写入，对应关系：
 *          - 0:±250dps  - 1:±500dps  - 2:±1000dps  - 3:±2000dps
 * @param   fsr  满量程档位(0~3)
 * @retval  0:设置成功；其他:设置失败(来自 I2C 写入返回值)
 */
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}
/**
 * @brief   设置 MPU6050 加速度计满量程范围
 * @details 通过写加速度计配置寄存器(0x1C)的 bit[4:3]设置满量程。
 *          参数 fsr 左移 3 位后写入，对应关系：
 *          - 0:±2g  - 1:±4g  - 2:±8g  - 3:±16g
 * @param   fsr  满量程档位(0~3)
 * @retval  0:设置成功；其他:设置失败(来自 I2C 写入返回值)
 */
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}
/**
 * @brief   设置 MPU6050 数字低通滤波器(LPF)
 * @details 通过写配置寄存器(0x1A)的 bit[2:0]设置低通滤波截止频率。
 *          根据传入频率分段映射为寄存器值，频率越高寄存器值越小。
 * @param   lpf  目标低通滤波频率(Hz)
 * @retval  0:设置成功；其他:设置失败(来自 I2C 写入返回值)
 * @note    LPF 可滤除高频噪声，频率设置过低会导致信号延迟，需折中选取。
 */
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;	/* 寄存器配置值 */
	if(lpf>=188)data=1;	/* >=188Hz，配置值为 1 */
	else if(lpf>=98)data=2;	/* >=98Hz，配置值为 2 */
	else if(lpf>=42)data=3;	/* >=42Hz，配置值为 3 */
	else if(lpf>=20)data=4;	/* >=20Hz，配置值为 4 */
	else if(lpf>=10)data=5;	/* >=10Hz，配置值为 5 */
	else data=6; 	/* <10Hz，配置值为 6(最低截止频率) */
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
}
/**
 * @brief   设置 MPU6050 采样率
 * @details 采样率分频器寄存器(0x19)：Sample_Rate = 1kHz / (1 + 分频值)。
 *          计算 data = 1000/rate - 1，并限制采样率在 4~1000Hz 范围内。
 *          同时自动将 LPF 设为采样率的一半，保证信号不失真。
 * @param   rate  目标采样率(Hz)，范围 4~1000
 * @retval  0:设置成功；其他:设置失败
 * @note    假定内部陀螺仪输出率 Fs=1kHz，因此分母基于 1kHz 计算。
 */
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;	/* 限制采样率上限为 1000Hz */
	if(rate<4)rate=4;	/* 限制采样率下限为 4Hz */
	data=1000/rate-1;	/* 计算分频值：Sample_Rate = 1kHz / (1 + data) */
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

/**
 * @brief   获取 MPU6050 温度值
 * @details 从温度寄存器(0x41,0x42)连续读取 2 字节，拼成 16 位有符号原始值。
 *          温度换算公式：Temp = 36.53 + raw / 340 (单位：摄氏度)。
 *          返回值放大 100 倍，便于整数传递。
 * @retval  温度值(扩大 100 倍，例如 3653 表示 36.53℃)
 */
//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 	/* 2 字节温度原始数据缓冲 */
    short raw;	/* 16 位有符号原始值 */
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 	/* 连续读取温度高、低字节 */
    raw=((u16)buf[0]<<8)|buf[1];  	/* 拼接为 16 位原始值(高字节在前) */
    temp=36.53+((double)raw)/340;  	/* 按公式换算为摄氏度 */
    return temp*100;;	/* 返回扩大 100 倍的温度值 */
}
/**
 * @brief   获取陀螺仪三轴原始数据
 * @details 从陀螺仪寄存器(0x43~0x48)连续读取 6 字节，每 2 字节拼成 16 位
 *          有符号值，分别对应 X/Y/Z 三轴角速度原始值。
 * @param   gx  陀螺仪 X 轴原始值输出指针
 * @param   gy  陀螺仪 Y 轴原始值输出指针
 * @param   gz  陀螺仪 Z 轴原始值输出指针
 * @retval  0:读取成功；其他:读取失败错误代码
 */
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res; 	/* 6 字节缓冲：XYZ 各 2 字节 */
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);	/* 从 0x43 起连续读 6 字节 */
	if(res==0)	/* 读取成功 */
	{
		*gx=((u16)buf[0]<<8)|buf[1];  	/* X 轴：高字节<<8 | 低字节 */
		*gy=((u16)buf[2]<<8)|buf[3];  	/* Y 轴：高字节<<8 | 低字节 */
		*gz=((u16)buf[4]<<8)|buf[5];	/* Z 轴：高字节<<8 | 低字节 */
	}
    return res;;	/* 返回读取结果：0=成功 */
}
/**
 * @brief   获取加速度计三轴原始数据
 * @details 从加速度寄存器(0x3B~0x40)连续读取 6 字节，每 2 字节拼成 16 位
 *          有符号值，分别对应 X/Y/Z 三轴加速度原始值。该数据用于抬腕亮屏和步数统计。
 * @param   ax  加速度 X 轴原始值输出指针
 * @param   ay  加速度 Y 轴原始值输出指针
 * @param   az  加速度 Z 轴原始值输出指针
 * @retval  0:读取成功；其他:读取失败错误代码
 */
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res; 	/* 6 字节缓冲：XYZ 各 2 字节 */
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);	/* 从 0x3B 起连续读 6 字节 */
	if(res==0)	/* 读取成功 */
	{
		*ax=((u16)buf[0]<<8)|buf[1];  	/* X 轴：高字节<<8 | 低字节 */
		*ay=((u16)buf[2]<<8)|buf[3];  	/* Y 轴：高字节<<8 | 低字节 */
		*az=((u16)buf[4]<<8)|buf[5];	/* Z 轴：高字节<<8 | 低字节 */
	}
    return res;;	/* 返回读取结果：0=成功 */
}
/**
 * @brief   I2C 连续写入多个字节(寄存器批量配置)
 * @details 完整写时序：START -> 发送器件地址(写) -> 等待ACK -> 发送寄存器地址
 *          -> 等待ACK -> 依次发送数据字节(每字节等待ACK) -> STOP。
 *          每个字节发送失败都会产生停止信号并返回错误。
 * @param   addr  器件地址(7位，函数内部左移1位并补写方向位0)
 * @param   reg   起始寄存器地址
 * @param   len   写入数据长度(字节数)
 * @param   buf   待写入数据缓冲区指针
 * @retval  0:写入成功；1:写入失败(应答失败)
 */
//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
    IIC_Start(); 	/* 产生起始信号 */
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();	/* 应答失败，产生停止信号释放总线 */
		return 1;	/* 返回失败 */
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)	/* 循环发送 len 个字节 */
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	/* 应答失败，产生停止信号 */
			return 1;	/* 返回失败 */
		}
	}
    IIC_Stop();	/* 全部发送完成，产生停止信号 */
	return 0;	/* 返回成功 */
}
/**
 * @brief   I2C 连续读取多个字节(数据批量读取)
 * @details 完整读时序(含寄存器地址写入)：
 *          START -> 发送器件地址(写) -> 等待ACK -> 发送寄存器地址 -> 等待ACK ->
 *          重复START -> 发送器件地址(读) -> 等待ACK ->
 *          依次接收数据字节(前 len-1 个发ACK，最后一个发NACK) -> STOP。
 * @param   addr  器件地址(7位，函数内部左移1位并补方向位)
 * @param   reg   起始寄存器地址
 * @param   len   读取数据长度(字节数)
 * @param   buf   读取数据存放缓冲区指针
 * @retval  0:读取成功；1:读取失败(应答失败)
 * @note    最后一个字节读取后发送 NACK，通知从机停止输出。
 */
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
 	IIC_Start(); 	/* 产生起始信号，开始写阶段(设定寄存器地址) */
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();	/* 应答失败，产生停止信号 */
		return 1;	/* 返回失败 */
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();	/* 重复起始信号，切换为读方向 */
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();		//等待应答
	while(len)	/* 循环读取 len 个字节 */
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK
		len--;	/* 剩余读取数减1 */
		buf++; 	/* 缓冲指针后移 */
	}
    IIC_Stop();	//产生一个停止条件
	return 0;	/* 返回成功 */
}
/**
 * @brief   I2C 写单个字节到 MPU6050 指定寄存器
 * @details 完整写单字节时序：START -> 发送器件地址(写) -> 等待ACK ->
 *          发送寄存器地址 -> 等待ACK -> 发送数据字节 -> 等待ACK -> STOP。
 *          每一步应答失败都会产生停止信号并返回错误。
 * @param   reg   目标寄存器地址
 * @param   data  待写入的单字节数据
 * @retval  0:写入成功；1:写入失败(应答失败)
 */
//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data)
{
    IIC_Start(); 	/* 产生起始信号 */
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();	/* 应答失败，产生停止信号 */
		return 1;	/* 返回失败 */
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	/* 应答失败，产生停止信号 */
		return 1;	/* 返回失败 */
	}
    IIC_Stop();	/* 写入完成，产生停止信号 */
	return 0;	/* 返回成功 */
}
/**
 * @brief   I2C 从 MPU6050 指定寄存器读取单个字节
 * @details 完整读单字节时序：
 *          START -> 发送器件地址(写) -> 等待ACK -> 发送寄存器地址 -> 等待ACK ->
 *          重复START -> 发送器件地址(读) -> 等待ACK -> 接收1字节(发NACK) -> STOP。
 * @param   reg  目标寄存器地址
 * @retval  读取到的单字节数据
 * @note    本函数不返回错误码，调用方应确保器件通信正常(如 MPU_Init 已校验 ID)。
 */
//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 	/* 产生起始信号，开始写阶段(设定寄存器地址) */
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
	IIC_Wait_Ack();		//等待应答
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();	/* 重复起始信号，切换为读方向 */
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();		//等待应答
	res=IIC_Read_Byte(0);//读取数据,发送nACK
    IIC_Stop();			//产生一个停止条件
	return res;		/* 返回读取到的字节 */
}
