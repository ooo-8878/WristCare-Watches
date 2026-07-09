/**
 * @file    max30102.c
 * @brief   MAX30102 心率血氧传感器驱动实现
 *
 * @details 本模块为智能手表项目提供 Maxim(现 ADI) MAX30102 集成传感器的驱动：
 *          - 集成 红光/红外光 LED + 光电二极管 + ADC + 滤波电路
 *          - 用于 PPG 信号采集，进而计算心率(HR)和血氧饱和度(SpO2)
 *
 *          硬件资源占用：
 *          - 通信接口：软件 I2C（由 i2c.c 提供，bit-bang 模拟时序）
 *                      设备地址 7-bit=0x57（max30102_WR_address 8-bit=0xAE）
 *          - 中断引脚：MAX30102 的 INT 引脚连接到 STM32 的 PA4，开漏输出，
 *                      数据就绪时拉低触发，主机读取寄存器后自动释放
 *
 *          与 FreeRTOS 的交互：
 *          - 由 main.c 中的 app_task_max30102 任务周期性调用 maxim_max30102_read_fifo()
 *            读取红光/红外 ADC 数据，进行滤波/峰检得到心率和血氧
 *          - 软件延时与 I2C 阻塞，需保证任务优先级与延时合理
 *
 *          设计要点：
 *          - 提供两套 API：max30102_Bus_Write/Read 与 maxim_max30102_write/read_reg，
 *            前者使用本文件的软件 I2C 时序，后者调用 i2c.c 的 IIC_* 接口
 *          - FIFO 一次读取 6 字节：3 字节红光 + 3 字节红外，18 位精度
 *          - 上电后需复位、清中断、配置模式/采样率/脉冲宽度/LED 电流
 *
 * @note    本文件不含心率/血氧算法实现，仅做传感器数据采集
 */

#include "includes.h"

/**
 * @brief   向 MAX30102 寄存器写入一个字节（软件 I2C 时序）
 * @param   Register_Address 寄存器地址
 * @param   Word_Data        待写入的字节
 * @retval  1=成功，0=失败（无 ACK）
 * @note    I2C 写时序：START → 写地址+W → 寄存器地址 → 数据 → STOP
 *          任一步骤无应答即跳转 cmd_fail，发送 STOP 后返回 0
 */
u8 max30102_Bus_Write(u8 Register_Address, u8 Word_Data)
{
	/* 采用串行EEPROM随即读取指令序列，连续读取若干字节 */

	/* 第1步：发起I2C总线启动信号 */
	i2c_start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_send_byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址 */
	i2c_send_byte(Register_Address);
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	
	/* 第5步：开始写入数据 */
	i2c_send_byte(Word_Data);

	/* 第6步：发送ACK */
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 发送I2C总线停止信号 */
	i2c_stop();
	return 1;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_stop();
	return 0;
}



/**
 * @brief   从 MAX30102 寄存器读取一个字节（软件 I2C 时序）
 * @param   Register_Address 寄存器地址
 * @retval  读取到的字节；失败返回 0
 * @note    I2C 读时序（随机读）：
 *          START → 写地址+W → 寄存器地址 → RESTART → 写地址+R → 读1字节+NAK → STOP
 */
u8 max30102_Bus_Read(u8 Register_Address)
{
	u8  data;


	/* 第1步：发起I2C总线启动信号 */
	i2c_start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_send_byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	i2c_send_byte((uint8_t)Register_Address);
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	i2c_start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_send_byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	{
		data = i2c_recv_byte();	/* 读1个字节 */

		i2c_ack(1);	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
	}
	/* 发送I2C总线停止信号 */
	i2c_stop();
	return data;	/* 执行成功 返回data值 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_stop();
	return 0;
}


/**
 * @brief   从 MAX30102 FIFO 连续读取多个样本（按字读取）
 * @param   Register_Address FIFO 数据寄存器地址（REG_FIFO_DATA）
 * @param   Word_Data        输出缓冲区，二维数组 [count][2]，[i][0]=红光，[i][1]=红外
 * @param   count            要读取的样本数
 * @retval  无
 * @note    每个样本由 4 字节组成：红光 2 字节 + 红外 2 字节（实际芯片内部
 *          每个 LED 3 字节，此处为简化版本按 2 字节拼接）
 *          最后一个字节发送 NACK 表示读取结束
 */
void max30102_FIFO_ReadWords(u8 Register_Address,u16 Word_Data[][2],u8 count)
{
	u8 i=0;                            /* 样本索引 */
	u8 no = count;                     /* 剩余待读样本数 */
	u8 data1, data2;                   /* 临时存放读取的两个字节 */
	/* 第1步：发起I2C总线启动信号 */
	i2c_start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_send_byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	i2c_send_byte((uint8_t)Register_Address);
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	i2c_start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_send_byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	while (no)
	{
		data1 = i2c_recv_byte();	
		i2c_ack(0);
		data2 = i2c_recv_byte();
		i2c_ack(0);
		Word_Data[i][0] = (((u16)data1 << 8) | data2);  //

		
		data1 = i2c_recv_byte();	
		i2c_ack(0);
		data2 = i2c_recv_byte();
		if(1==no)
			i2c_ack(1);	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		else
			i2c_ack(0);
		Word_Data[i][1] = (((u16)data1 << 8) | data2); 

		no--;	
		i++;
	}
	/* 发送I2C总线停止信号 */
	i2c_stop();

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_stop();
}

/**
 * @brief   从 MAX30102 FIFO 读取一个样本（6 字节，红光3 + 红外3）
 * @param   Register_Address FIFO 寄存器地址（REG_FIFO_DATA）
 * @param   Data             输出缓冲区，至少 6 字节
 * @retval  无
 * @note    进入函数时先读取中断状态寄存器 1/2 清除中断标志，
 *          然后通过 I2C 连续读取 6 字节，最后字节发送 NACK 结束。
 *          Data[0..2]=红光样本，Data[3..5]=红外样本
 */
void max30102_FIFO_ReadBytes(u8 Register_Address,u8* Data)
{
	max30102_Bus_Read(REG_INTR_STATUS_1);   /* 读中断状态1，清除 PPG_RDY 中断 */
	max30102_Bus_Read(REG_INTR_STATUS_2);   /* 读中断状态2，清除其他中断 */
	
	/* 第1步：发起I2C总线启动信号 */
	i2c_start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_send_byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	i2c_send_byte((uint8_t)Register_Address);
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	i2c_start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_send_byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (i2c_wait_ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	Data[0] = i2c_recv_byte();i2c_ack(0);		
	Data[1] = i2c_recv_byte();i2c_ack(0);	
	Data[2] = i2c_recv_byte();i2c_ack(0);	
	Data[3] = i2c_recv_byte();i2c_ack(0);	
	Data[4] = i2c_recv_byte();i2c_ack(0);	
	Data[5] = i2c_recv_byte();i2c_ack(1);	
	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
	/* 发送I2C总线停止信号 */
	i2c_stop();

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_stop();

}

/**
 * @brief   MAX30102 传感器初始化
 * @retval  无
 * @note    完成以下工作：
 *          1) 配置 PA4 为输入，用于读取 MAX30102 INT 中断引脚电平
 *          2) 初始化软件 I2C（i2c_init）
 *          3) 复位 MAX30102
 *          4) 配置中断使能、FIFO、工作模式、采样率、脉冲宽度、LED 电流
 *          调用后传感器即开始按 100Hz 采样并填入 FIFO，等待主机读取
 */
void max30102_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//端口E硬件时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   /* 使能 GPIOA 时钟（INT 引脚在 PA4） */

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;//第4引脚        /* PA4 = MAX30102 INT 中断输入 */
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;			//复用功能模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;		//引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		//增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;		//不需要上下拉电阻
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	i2c_init();                                          /* 初始化软件 I2C GPIO */

	max30102_reset();                                    /* 软复位 MAX30102 */

//	max30102_Bus_Write(REG_MODE_CONFIG, 0x0b);  //mode configuration : temp_en[3]      MODE[2:0]=010 HR only enabled    011 SP02 enabled
//	max30102_Bus_Write(REG_INTR_STATUS_2, 0xF0); //open all of interrupt
//	max30102_Bus_Write(REG_INTR_STATUS_1, 0x00); //all interrupt clear
//	max30102_Bus_Write(REG_INTR_ENABLE_2, 0x02); //DIE_TEMP_RDY_EN
//	max30102_Bus_Write(REG_TEMP_CONFIG, 0x01); //SET   TEMP_EN

//	max30102_Bus_Write(REG_SPO2_CONFIG, 0x47); //SPO2_SR[4:2]=001  100 per second    LED_PW[1:0]=11  16BITS

//	max30102_Bus_Write(REG_LED1_PA, 0x47);
//	max30102_Bus_Write(REG_LED2_PA, 0x47);



	max30102_Bus_Write(REG_INTR_ENABLE_1,0xc0);	// INTR setting               /* 使能 PPG_RDY(数据就绪) 和 ALC（环境光） 中断 */
	max30102_Bus_Write(REG_INTR_ENABLE_2,0x00);                              /* 关闭温度/模拟就绪中断 */
	max30102_Bus_Write(REG_FIFO_WR_PTR,0x00);  	//FIFO_WR_PTR[4:0]            /* 清零 FIFO 写指针 */
	max30102_Bus_Write(REG_OVF_COUNTER,0x00);  	//OVF_COUNTER[4:0]            /* 清零溢出计数器 */
	max30102_Bus_Write(REG_FIFO_RD_PTR,0x00);  	//FIFO_RD_PTR[4:0]            /* 清零 FIFO 读指针 */
	max30102_Bus_Write(REG_FIFO_CONFIG,0x0f);  	//sample avg = 1, fifo rollover=false, fifo almost full = 17 /* 无样本平均、无回卷、AF=17 */
	max30102_Bus_Write(REG_MODE_CONFIG,0x03);  	//0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED /* SpO2 模式（红光+红外） */
	max30102_Bus_Write(REG_SPO2_CONFIG,0x27);  	// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  /* 4096nA范围/100sps/400us脉冲(18位) */
	max30102_Bus_Write(REG_LED1_PA,0x24);   	//Choose value for ~ 7mA for LED1 /* 红光 LED 电流约 7mA */
	max30102_Bus_Write(REG_LED2_PA,0x24);   	// Choose value for ~ 7mA for LED2 /* 红外 LED 电流约 7mA */
	max30102_Bus_Write(REG_PILOT_PA,0x7f);   	// Choose value for ~ 25mA for Pilot LED /* Pilot LED 电流约 25mA */



//	// Interrupt Enable 1 Register. Set PPG_RDY_EN (data available in FIFO)
//	max30102_Bus_Write(0x2, 1<<6);

//	// FIFO configuration register
//	// SMP_AVE: 16 samples averaged per FIFO sample
//	// FIFO_ROLLOVER_EN=1
//	//max30102_Bus_Write(0x8,  1<<4);
//	max30102_Bus_Write(0x8, (0<<5) | 1<<4);

//	// Mode Configuration Register
//	// SPO2 mode
//	max30102_Bus_Write(0x9, 3);

//	// SPO2 Configuration Register
//	max30102_Bus_Write(0xa,
//			(3<<5)  // SPO2_ADC_RGE 2 = full scale 8192 nA (LSB size 31.25pA); 3 = 16384nA
//			| (1<<2) // sample rate: 0 = 50sps; 1 = 100sps; 2 = 200sps
//			| (3<<0) // LED_PW 3 = 411μs, ADC resolution 18 bits
//	);

//	// LED1 (red) power (0 = 0mA; 255 = 50mA)
//	max30102_Bus_Write(0xc, 0xb0);

//	// LED (IR) power
//	max30102_Bus_Write(0xd, 0xa0);

}

/**
 * @brief   复位 MAX30102 传感器
 * @retval  无
 * @note    向 REG_MODE_CONFIG(0x09) 写 0x40，置 RESET 位，
 *          复位后所有寄存器恢复默认值。重复写两次以确保稳定。
 */
void max30102_reset(void)
{
	max30102_Bus_Write(REG_MODE_CONFIG,0x40);           /* 写 RESET=1，启动复位 */
	max30102_Bus_Write(REG_MODE_CONFIG,0x40);           /* 再次写入，确保复位生效 */
}






/**
 * @brief   向 MAX30102 寄存器写入一个字节（通过 i2c.c 的 IIC_* 接口）
 * @param   uch_addr 寄存器地址
 * @param   uch_data 待写入数据
 * @retval  无
 * @note    与 max30102_Bus_Write 等效，区别在于调用的是 i2c.c 中
 *          封装好的 IIC_Write_One_Byte。常用于 maxim_* 系列 API
 */
void maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
//  char ach_i2c_data[2];
//  ach_i2c_data[0]=uch_addr;
//  ach_i2c_data[1]=uch_data;
//
//  IIC_WriteBytes(I2C_WRITE_ADDR, ach_i2c_data, 2);
	IIC_Write_One_Byte(I2C_WRITE_ADDR,uch_addr,uch_data);   /* 调用 i2c.c 的单字节写接口 */
}

/**
 * @brief   从 MAX30102 寄存器读取一个字节（通过 i2c.c 的 IIC_* 接口）
 * @param   uch_addr  寄存器地址
 * @param   puch_data 输出指针，用于返回读取的字节
 * @retval  无
 * @note    读取结果通过出参 puch_data 返回
 */
void maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
//  char ch_i2c_data;
//  ch_i2c_data=uch_addr;
//  IIC_WriteBytes(I2C_WRITE_ADDR, &ch_i2c_data, 1);
//
//  i2c.read(I2C_READ_ADDR, &ch_i2c_data, 1);
//
//   *puch_data=(uint8_t) ch_i2c_data;
	IIC_Read_One_Byte(I2C_WRITE_ADDR,uch_addr,puch_data);   /* 调用 i2c.c 的单字节读接口 */
}

/**
 * @brief   读取 MAX30102 FIFO 一次样本（红光+红外各 18 位）
 * @param   pun_red_led 输出：红光通道 ADC 值
 * @param   pun_ir_led  输出：红外通道 ADC 值
 * @retval  无
 * @note    读取流程：
 *          1) 先读 REG_INTR_STATUS_1/2 清除中断标志（INT 引脚被释放）
 *          2) 通过 I2C 从 REG_FIFO_DATA 连续读取 6 字节
 *             - [0..2]：红光样本（高位在前，仅低 18 位有效）
 *             - [3..5]：红外样本（同上）
 *          3) 拼装为 32 位整数后用 0x03FFFF 掩码取低 18 位
 *          被 main.c 的 app_task_max30102 周期性调用
 */
void maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
	uint32_t un_temp;                              /* 临时存放拼接结果 */
	unsigned char uch_temp;                         /* 中断状态读取暂存 */
	char ach_i2c_data[6];                           /* FIFO 6 字节缓冲 */
	*pun_red_led=0;                                 /* 红光结果清零 */
	*pun_ir_led=0;                                  /* 红外结果清零 */


  //read and clear status register
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);   /* 读状态1，清中断 */
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);   /* 读状态2，清中断 */

  IIC_ReadBytes(I2C_WRITE_ADDR,REG_FIFO_DATA,(u8 *)ach_i2c_data,6); /* 一次读 6 字节 FIFO 数据 */

  /* —— 拼装红光样本（高字节在前）—— */
  un_temp=(unsigned char) ach_i2c_data[0];          /* 取 byte0 作为最高字节 */
  un_temp<<=16;                                    /* 左移 16 位 */
  *pun_red_led+=un_temp;                           /* 累加到红光结果 */
  un_temp=(unsigned char) ach_i2c_data[1];          /* 取 byte1 作为中间字节 */
  un_temp<<=8;                                      /* 左移 8 位 */
  *pun_red_led+=un_temp;                           /* 累加 */
  un_temp=(unsigned char) ach_i2c_data[2];          /* 取 byte2 作为最低字节 */
  *pun_red_led+=un_temp;                           /* 累加 */

  /* —— 拼装红外样本（高字节在前）—— */
  un_temp=(unsigned char) ach_i2c_data[3];          /* 取 byte3 作为最高字节 */
  un_temp<<=16;                                    /* 左移 16 位 */
  *pun_ir_led+=un_temp;                            /* 累加到红外结果 */
  un_temp=(unsigned char) ach_i2c_data[4];          /* 取 byte4 作为中间字节 */
  un_temp<<=8;                                      /* 左移 8 位 */
  *pun_ir_led+=un_temp;                            /* 累加 */
  un_temp=(unsigned char) ach_i2c_data[5];          /* 取 byte5 作为最低字节 */
  *pun_ir_led+=un_temp;                            /* 累加 */
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]      /* 掩码，仅保留低 18 位有效数据 */
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]       /* 掩码，仅保留低 18 位有效数据 */
}
