/**
 * @file    dht11.c
 * @version V1.0
 * @date    2026-07-08
 * @brief   DHT11 单总线温湿度传感器驱动实现文件
 *
 * @section 模块功能
 *   本文件实现 DHT11 数字温湿度传感器的单总线通信驱动,提供传感器初始化、
 *   起始信号、应答检测、按位读取字节、温湿度数据获取(含校验和)等功能。
 *   DHT11 测量范围:湿度 20~90%RH,温度 0~50℃,精度 ±2%RH / ±2℃,
 *   适用于智能手表对环境温湿度的低频采集(单次采样周期 >= 1s)。
 *
 * @section 硬件资源
 *   - 通信总线: 单总线(One-Wire)协议,半双工,主机(MCU)轮询发起
 *   - 引脚映射:
 *       * PG9 -> DHT11 DATA 数据引脚
 *   - 时钟:
 *       * RCC_AHB1Periph_GPIOG (GPIOG 端口时钟)
 *   - 引脚工作模式:
 *       * 发送阶段: GPIO_Mode_OUT + GPIO_OType_OD (开漏输出)
 *       * 接收阶段: 由上层切换为输入模式(开漏 + 外部上拉或内部上拉)
 *       * 本驱动中通过 DHT11_Data_W()/DHT11_Data_R() 宏在 sys.h 中
 *         实现方向切换,默认上电配置为开漏输出并输出高电平(总线空闲)
 *
 * @section 通信协议(DHT11 单总线时序)
 *   1. 主机起始信号: 主机拉低 DATA >= 18ms(本驱动用 20ms),
 *      再释放总线(输出高),等待 20~40us。
 *   2. 从机应答: DHT11 拉低 80us,再拉高 80us 表示准备发送数据。
 *   3. 数据位传输: 每一位以 50us 低电平开始,随后高电平持续时间决定 0/1:
 *        - 高电平 26~28us -> 数据 "0"
 *        - 高电平 70us   -> 数据 "1"
 *      本驱动在低电平结束后延时 40us 再采样,>40us 仍为高即判 "1"。
 *   4. 数据格式: 共 40 bit = 5 字节,
 *        [湿度整数][湿度小数][温度整数][温度小数][校验和]
 *      校验和 = 前 4 字节算术和的低 8 位。
 *
 * @section 与 FreeRTOS 的交互方式
 *   - 本驱动为纯裸机时序实现,不直接调用 FreeRTOS API。
 *   - 由 USER/main.c 中的 app_task_dht11 任务周期性调用
 *     dht11_get_tem_hum() 采集数据(建议周期 >= 2s,避免传感器温升)。
 *   - delay_ms / delay_us 若基于 SysTick,在 FreeRTOS 启动后调用
 *     delay_ms 需注意不能阻塞调度器太久;本驱动单次采样最大耗时
 *     约 20ms + 4~5ms 数据传输,可接受。
 *
 * @section 设计要点
 *   1. 所有时序等待均带 time_out 超时保护(默认 100us),防止传感器
 *      掉线或时序异常导致任务卡死。
 *   2. 通过位拼装按 MSB 先行(0x80 >> i)方式组字节,与 DHT11 协议一致。
 *   3. 校验和失败返回 -3,数据读取失败返回 -2,应答失败返回 -1,
 *      成功返回 0;调用方可据此做异常重试。
 *   4. 单总线要求严格时序,delay_us 必须精确(基于 DWT 或定时器);
 *      采样期间应关闭任务调度或屏蔽中断以保证时序不被打断。
 *
 * @author  STM32工程师
 */

#include "includes.h"

/**
  * @brief  DHT11初始化
  * @param  无
  * @retval 无
  */
void dht11_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;/*定义结构体变量*/ /* GPIO 初始化结构体,用于配置 PG9 引脚 */

	/* 使能 GPIOG 端口时钟(DHT11 RCC 宏定义为 RCC_AHB1Periph_GPIOG) */
	RCC_AHB1PeriphClockCmd(DHT11_RCC,ENABLE);/*打开GPIOG时钟*/

	/* 配置 DHT11 数据引脚 PG9 为开漏输出,初始为高电平(总线空闲) */
	GPIO_InitStructure.GPIO_Pin = DHT11_Pin;/*DHT11通信引脚*/     /* 选中 PG9 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;/*设置为开漏输出模式*/ /* 输出模式,主机驱动总线 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                  /* 开漏输出,允许多方共享总线且支持"线与" */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;              /* 引脚翻转速度 50MHz */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;             /* 不启用内部上下拉,依赖外部上拉电阻 */
	GPIO_Init(DHT11_Port,&GPIO_InitStructure);                     /* 写入 GPIOG 寄存器,完成 PG9 配置 */

	/* 总线空闲状态:主机输出高电平(开漏 + 外部上拉 = 高),
	   等待 DHT11 上电稳定并可响应起始信号 */
	DHT11_Data_W() = 1;/*初始化让总线处于空闲状态*/
}


/**
  * @brief  DHT11开始信号
  * @param  无
  * @retval 无
  * @note   主机起始信号时序:拉低 >= 18ms(本驱动取 20ms,留余量),
  *         再释放总线拉高,等待 20~40us(本驱动取 30us)让 DHT11 检测到上升沿并响应。
  *         该时序要求传感器此前处于空闲态(总线高 >= 1s)。
  */
void dht11_start(void)
{
	DHT11_Data_W() = 0;/*主机拉低总线*/   /* 主机拉低 DATA,触发 DHT11 起始检测 */
	delay_ms(20);                          /* 保持低电平 20ms,满足 DHT11 起始信号最小 18ms 要求 */
	DHT11_Data_W() = 1;/*主机释放总线*/   /* 主机释放总线,由外部上拉电阻拉高 DATA */
	delay_us(30);                          /* 等待 30us,落在 DHT11 期望的 20~40us 响应窗口内 */
}

/**
  * @brief  DHT11应答
  * @param  无
  * @retval 0--应答 | -1--应答失败
  * @note   DHT11 应答时序:主机释放总线后,DHT11 拉低 DATA 约 80us,
  *         再拉高约 80us 表示准备发送数据。本函数分别等待两次电平跳变,
  *         各带 100us 超时保护,超时返回 -1。
  */
int8_t dht11_ack(void)
{
	uint8_t time_out = 0;/*计时，防止循环卡死*/ /* 超时计数器,1us 递增一次 */

	/* 等待 DHT11 拉低总线(应答开始),最长 100us */
	while(DHT11_Data_R() == 1)/*主机发出开始信号之后DHT11会拉低总线80us左右作为回应，这里用死循环等待*/
	{
		time_out++;                           /* 计数 +1,每轮约 1us */
		delay_us(1);                          /* 延时 1us,细化超时粒度 */

		if(time_out >=100)                    /* 超过 100us 仍未拉低,认为传感器无应答 */
			return -1;                        /* 返回 -1:应答失败 */
	}

	time_out = 0;                              /* 复位计数器,准备等待下一段时序 */

	/* 等待 DHT11 拉高总线(应答结束),最长 100us */
	while(DHT11_Data_R() == 0)/*随后DHT11将总线拉高80us左右告诉主机要发送数据*/
	{
		time_out++;                           /* 计数 +1 */
		delay_us(1);                          /* 延时 1us */

		if(time_out >=100)                    /* 超过 100us 仍未拉高,认为时序异常 */
			return -1;                        /* 返回 -1:应答失败 */
	}

	return 0;                                  /* 应答正常,返回 0 */
}

/**
  * @brief  DHT11读取一个字节的数据
  * @param  无
  * @retval  byte | -2 --- 读取失败
  * @note   单总线位时序:
  *         - 每一位以 50us 低电平开始
  *         - 随后高电平持续 26~28us 表示 "0",持续 70us 表示 "1"
  *         - 本函数在每位高电平开始后延时 40us 再采样:
  *           若仍为高,说明是 "1"(26~28us 的 "0" 已结束转低);
  *           若已变低,说明是 "0"
  *         - 按 MSB(高位)先行的方式拼装字节(0x80 >> i)
  *         - 每个电平等待均带 100us 超时保护,超时返回 -2
  */
int8_t dht11_receive_byte(void)
{
	uint8_t time_out = 0;/*计时，防止循环卡死*/ /* 超时计数器 */
	uint8_t byte = 0x00;/*初始值*/            /* 待返回的字节,初始为 0,逐位置 1 */
	uint8_t i;                                /* 位循环计数,共 8 位 */

	for(i = 0;i < 8;i++)                       /* 按高位在前的顺序读取 8 位 */
	{
		/* 等待当前位 50us 起始低电平到来(此时可能仍处于上一位的余高电平) */
		while(DHT11_Data_R() == 1)/*等待变为低电平，代表数据准备开始传输*/
		{
			time_out++;                       /* 超时计数 +1 */
			delay_us(1);                      /* 延时 1us */

			if(time_out >=100)                /* 等待超时(100us) */
				return -2;                    /* 返回 -2:读取失败 */
		}

		time_out = 0;                          /* 复位计数器 */

		/* 等待高电平到来(本位数据位开始),最长 100us */
		while(DHT11_Data_R() == 0)/*等待变为高电平，代表数据开始传输*/
		{
			time_out++;                       /* 超时计数 +1 */
			delay_us(1);                      /* 延时 1us */

			if(time_out >=100)                /* 等待超时 */
				return -2;                    /* 返回 -2:读取失败 */
		}

		time_out = 0;                          /* 复位计数器 */

		/* 关键采样点:延时 40us 后再读电平
		   - "0" 的高电平约 26~28us,40us 时已转低 -> 读到 0
		   - "1" 的高电平约 70us,40us 时仍为高  -> 读到 1 */
		delay_us(40);
		if(DHT11_Data_R() == 1)/*过了40us若还是高电平，代表数据是1，否则为0*/
		{
			/* 通过 0x80 >> i 实现按 MSB 先行写入:第 0 次循环写 bit7,第 7 次写 bit0 */
			byte |= (0x80 >> i);
		}
	}
	return byte;                              /* 返回组装好的一个字节 */
}

/**
  * @brief  获取DHT11的温度和湿度
  * @param  tem--温度 | hum--湿度
  * @param  0---数据读取成功 -1---应答失败  -2---数据读取失败  -3---校验失败
  * @retval DHT11发出数据格式为 8bit湿度整数+8bit湿度小数+8bit温度整数+8bit温度小数+8bit校验和
  * @note   完整的 DHT11 一次采集流程:
  *         1) dht11_start() 发起起始信号
  *         2) dht11_ack()   等待从机应答
  *         3) 连续读取 5 字节:湿度整数、湿度小数、温度整数、温度小数、校验和
  *         4) 校验和 = 前 4 字节之和的低 8 位
  *         5) 输出:湿度取整数部分,温度取整数+小数/10
  *         6) 结束后等待 50us 让 DHT11 释放总线
  *         注意:本函数为阻塞调用,采样期间不应被高优先级任务抢占,
  *         否则会破坏微秒级时序。
  */
int8_t dht11_get_tem_hum(float* tem,uint8_t* hum)
{
	uint8_t buffer[5] = {0};                  /* 接收缓冲:5 字节依次为 湿整/湿小/温整/温小/校验 */
	uint8_t i;                                /* 字节循环计数 */
	int8_t byte;                              /* 临时存放每次读到的字节(可能为 -2 表示失败) */

	dht11_start();/*发出开始信号告诉DHT11准备测量数据*/ /* 主机发起始信号 */
	if(dht11_ack() == 0)/*DHT11应答了*/        /* 若 DHT11 正常应答 */
	{
		for(i = 0;i < 5;i++)/*接收DHT11发出的5byte数据*/ /* 循环读取 5 字节 */
		{
			byte = dht11_receive_byte();     /* 读取 1 字节,返回值可能是数据或 -2 */

			if(byte != -2)                    /* 读取成功(非错误码) */
			{
				buffer[i] = byte;            /* 存入对应位置 */
			}
			else
			{
				return -2;                    /* 任一字节读取失败,立即返回 -2 */
			}
		}
	}
	else
	{
		return -1;                            /* 应答失败,返回 -1 */
	}
	/* 校验和检查:前 4 字节之和的低 8 位应等于第 5 字节 */
	if(buffer[0]+buffer[1]+buffer[2]+buffer[3] == buffer[4])/*校验和正确*/
	{
		*hum = buffer[0];/*8位整数湿度数据*/  /* 湿度整数(DHT11 小数位恒为 0) */
		*tem = buffer[2]+buffer[3]/10.0;/*8位整数温度数据+8位小数数据*/ /* 温度 = 整数部分 + 小数/10.0 */
	}
	else
	{
		return -3;                            /* 校验失败,返回 -3 */
	}

	delay_us(50);/*DHT11发送完一组数据之后会将总线拉低50us*/ /* 等待 DHT11 释放总线,保证下一次起始信号可靠 */

	return 0;                                  /* 数据读取成功,返回 0 */
}



