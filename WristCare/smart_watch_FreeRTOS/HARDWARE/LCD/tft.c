/**
 * @file    tft.c
 * @brief   ST7789V2 TFT-LCD 显示屏驱动实现
 *
 * @details 本模块为智能手表项目提供 ST7789V2 控制器的 1.69 英寸 TFT-LCD
 *          显示输出驱动，承担以下职责：
 *          - 初始化 LCD 控制器（含 GPIO/FSMC/SPI/复位序列）
 *          - 提供 LVGL 所需的显示接口（画点、填充、刷图）
 *          - 通过 SPI 向 LCD GRAM 写入像素数据
 *          - 通过 TIM3 PWM 实现背光调光
 *
 *          硬件资源占用：
 *          - 通信接口：SPI1（硬件 SPI，PB3=SCK, PB5=MOSI），或软件 SPI
 *                      （由宏 LCD_SOFT_SPI_ENABLE 选择，使用 PE7/PE9/PE11/PE13/PE15）
 *          - 控制引脚：LCD_DC（数据/命令选择）、LCD_RST（复位）、SPI_CS（片选）
 *          - 背光引脚：PB4 复用为 TIM3_CH1 输出 PWM 调光
 *          - DMA2_Stream3 + 通道3：用于 SPI1 TX 加速 LVGL 帧刷新
 *
 *          与 FreeRTOS 的交互：
 *          - DMA 发送完成中断 DMA2_Stream3_IRQHandler 中调用
 *            taskENTER_CRITICAL_FROM_ISR() / taskEXIT_CRITICAL_FROM_ISR()
 *            保护临界区，并通知 LVGL 缓冲区已刷完（lv_disp_flush_ready）
 *          - lcd_set_brightness 被 main.c 的 app_task_istouch_lcd /
 *            app_task_mpu6050 任务调用，实现抬腕亮屏/触摸亮屏
 *
 *          设计要点：
 *          - 软/硬件 SPI 通过宏切换，便于在不同 PCB 上复用
 *          - DMA 配合 SPI 提高刷新率，避免 CPU 阻塞
 *          - 支持横竖屏 4 方向旋转，并自动处理 X/Y 偏移
 *          - 背光低电平点亮，PWM 占空比控制亮度
 *
 * @note    本文件仅实现显示驱动，文字/图形/动画等上层绘制由 LVGL 完成
 */

#include "includes.h"

uint16_t g_lcd_width =LCD_WIDTH;     /* LCD 当前逻辑宽度（随方向变化） */
uint16_t g_lcd_height=LCD_HEIGHT;     /* LCD 当前逻辑高度（随方向变化） */
uint8_t g_lcd_direction=0;           /* LCD 当前显示方向：0=0°，1=90°，2=180°，3=270° */

/**
 * @brief   通过 SPI1 发送一个字节
 * @param   byte 待发送的字节数据
 * @retval  无
 * @note    根据宏 LCD_SOFT_SPI_ENABLE 选择软件模拟 SPI 或硬件 SPI：
 *          - 软件 SPI：手动控制 SCK/SDA 时序，MSB 先发
 *          - 硬件 SPI：使用 STM32 SPI1 外设，等待 TXE/RXNE 标志
 */
void spi1_send_byte(uint8_t byte)
{
#if LCD_SOFT_SPI_ENABLE
  unsigned char counter;                /* 位计数器，循环发送 8 位 */

  for (counter = 0; counter < 8; counter++)
  {
    SPI_SCK_0;                          /* 时钟拉低，准备数据 */
    if ((byte & 0x80) == 0)             /* 取最高位（MSB 先发） */
    {
      SPI_SDA_0;                        /* 该位为 0，数据线拉低 */
    }
    else
      SPI_SDA_1;                        /* 该位为 1，数据线拉高 */
    byte = byte << 1;                   /* 左移一位，准备下一位 */
    SPI_SCK_1;                          /* 时钟上升沿，从机采样数据 */
  }
  SPI_SCK_0;                            /* 8 位发完，时钟恢复低电平 */

#else
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)  /* 等待发送缓冲区空 */
		;
	SPI_I2S_SendData(SPI1, byte);                                   /* 写入 DR，启动发送 */

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) /* 等待接收完成（全双工） */
		;
	SPI_I2S_ReceiveData(SPI1);                                      /* 读取 DR 清 RXNE 标志 */

#endif

}

/**
 * @brief   向 LCD 发送一个命令字节
 * @param   cmd 命令字节（如 0x2A 列地址设置、0x29 开显示等）
 * @retval  无
 * @note    拉低 DC 选择命令模式，拉低 CS 使能 LCD，发送完后释放 CS
 */
void lcd_send_cmd(uint8_t cmd)
{
	SPI_CS_0;                /* 片选拉低，使能 LCD */
	LCD_DC_0;                /* DC=0 表示命令 */
	spi1_send_byte(cmd);     /* 发送命令字节 */
	SPI_CS_1;                /* 片选拉高，结束通信 */
}

/**
 * @brief   向液晶屏写一个8位数据
 * @param   dat 待写入的数据字节
 * @retval  无
 * @note    拉高 DC 选择数据模式，拉低 CS 使能 LCD，发送完后释放 CS
 */
void lcd_send_data(uint8_t dat)
{
	SPI_CS_0;                /* 片选拉低，使能 LCD */
	LCD_DC_1;                /* DC=1 表示数据 */
	spi1_send_byte(dat);     /* 发送数据字节 */
	SPI_CS_1;                /* 片选拉高，结束通信 */
}

/**
 * @brief   设置 LCD 显示窗口的行列地址范围
 * @param   x_s 起始列（X 坐标）
 * @param   y_s 起始行（Y 坐标）
 * @param   x_e 结束列（X 坐标）
 * @param   y_e 结束行（Y 坐标）
 * @retval  无
 * @note    部分屏幕存在内部 GRAM 偏移，需要根据显示方向叠加 X_OFFSET/Y_OFFSET
 *          随后向 LCD 发送 0x2A(列地址)、0x2B(行地址)、0x2C(写显存) 命令
 */
void lcd_addr_set(uint32_t x_s, uint32_t y_s, uint32_t x_e, uint32_t y_e)
{
	/* 部分tft屏需要偏移量 */

	if(lcd_get_direction()==0 || lcd_get_direction()==2)   /* 0°或180°方向：Y 方向需偏移 */
	{
		y_s=y_s+Y_OFFSET;                                  /* 起始行加偏移 */
		y_e=y_e+Y_OFFSET;                                   /* 结束行加偏移 */
	}

	if(lcd_get_direction()==1 || lcd_get_direction()==3)   /* 90°或270°方向：X 方向需偏移 */
	{
		x_s=x_s+X_OFFSET;                                  /* 起始列加偏移 */
		x_e=x_e+X_OFFSET;                                   /* 结束列加偏移 */

	}



	lcd_send_cmd(0x2a);	 		// 列地址设置
	lcd_send_data(x_s>> 8); 	// 起始列
	lcd_send_data(x_s);
	lcd_send_data(x_e >> 8); 	// 结束列
	lcd_send_data(x_e);

	lcd_send_cmd(0x2b);	 		// 行地址设置
	lcd_send_data(y_s>> 8); 	// 起始行
	lcd_send_data(y_s);
	lcd_send_data(y_e >> 8); 	// 结束行
	lcd_send_data(y_e);
	lcd_send_cmd(0x2C); 		// 写显存
}

/**
 * @brief   在指定矩形区域内填充同一颜色
 * @param   x_s   起始列坐标
 * @param   y_s   起始行坐标
 * @param   x_len 区域宽度
 * @param   y_len 区域高度
 * @param   color 填充颜色（RGB565）
 * @retval  无
 * @note    先设置地址窗口，再连续写入像素，避免每像素都设置地址，提高填充速度
 */
void lcd_fill(uint32_t x_s, uint32_t y_s, uint32_t x_len, uint32_t y_len,uint32_t color)
{

	uint32_t x, y;                                      /* 像素循环计数 */

	lcd_addr_set(x_s,y_s,x_s+x_len-1,y_s+y_len - 1);    /* 设置填充矩形窗口 */

	SPI_CS_0;                                          /* 片选拉低，连续传输 */
	LCD_DC_1;                                          /* DC=1 数据模式 */

	for (y = y_s; y < y_s+y_len; y++)                   /* 行循环 */
	{
		for (x = x_s; x < x_s+x_len; x++)               /* 列循环 */
		{

			spi1_send_byte(color >> 8);                /* 发送颜色高字节 */
			spi1_send_byte(color);                     /* 发送颜色低字节 */
		}
	}

	SPI_CS_1;                                          /* 片选拉高，结束传输 */
}

/**
 * @brief   用指定颜色清屏
 * @param   color 清屏颜色（RGB565）
 * @retval  无
 * @note    调用 lcd_fill 对全屏进行填充
 */
void lcd_clear(uint32_t color)
{
	lcd_fill(0,0,g_lcd_width,g_lcd_height,color);       /* 用全屏尺寸进行填充 */
}

/**
 * @brief   在指定区域绘制图片（RGB565 像素数组）
 * @param   x_s    起始列坐标
 * @param   y_s    起始行坐标
 * @param   width  图片宽度
 * @param   height 图片高度
 * @param   pic    图片像素数据指针（每个像素 2 字节，RGB565）
 * @retval  无
 * @note    LVGL 等上层框架刷帧时通过此函数把帧缓冲写入 LCD GRAM
 */
void lcd_draw_picture(uint32_t x_s, uint32_t y_s, uint32_t width, uint32_t height, const uint8_t *pic)
{

	const uint8_t *p = pic;                            /* 指向图片数据的指针 */
	uint32_t i = 0;                                    /* 字节计数器 */

	lcd_addr_set(x_s, y_s, x_s+width-1, y_s+height-1);/* 设置写图窗口 */

	SPI_CS_0;                                          /* 片选拉低 */
	LCD_DC_1;                                          /* DC=1 数据模式 */
	for (i = 0; i <width*height*2; i += 2)             /* 每像素 2 字节循环 */
	{

		spi1_send_byte(p[i]);                          /* 发送像素高字节 */
		spi1_send_byte(p[i + 1]);                      /* 发送像素低字节 */
	}
	SPI_CS_1;                                          /* 片选拉高 */

}

// void lcd_show_char(uint32_t x, uint32_t y,uint8_t ch,uint32_t fc,uint32_t bc,uint32_t font_size,uint32_t mode)
// {

// 	u8 temp,sizex,t,m=0;
// 	u16 i,TypefaceNum;						//一个字符所占字节大小
	
// 	u16 x0=x;
	
// 	sizex=font_size/2;
	
// 	TypefaceNum=(sizex/8+((sizex%8)?1:0))*font_size;
	
// 	ch=ch-' ';    						//得到偏移后的值
	
// 	lcd_addr_set(x,y,x+sizex-1,y+font_size-1);  //设置光标位置 
	
// 	for(i=0;i<TypefaceNum;i++)
// 	{ 
// 		if(font_size==12)temp=ascii_1206[ch][i];		      //调用6x12字体
// 		else if(font_size==16)temp=ascii_1608[ch][i];		 //调用8x16字体
// 		else if(font_size==24)temp=ascii_2412[ch][i];		 //调用12x24字体
// 		else if(font_size==32)temp=ascii_3216[ch][i];		 //调用16x32字体
// 		else return;
// 		for(t=0;t<8;t++)
// 		{
// 			if(!mode)//非叠加模式
// 			{
// 				if(temp&(0x01<<t))
// 				{
// 					lcd_send_data(fc>>8);
// 					lcd_send_data(fc);
// 				}
// 				else 
// 				{
// 					lcd_send_data(bc>>8);
// 					lcd_send_data(bc);
// 				}
// 				m++;
// 				if(m%sizex==0)
// 				{
// 					m=0;
// 					break;
// 				}
// 			}
// 			else//叠加模式
// 			{
// 				if(temp&(0x01<<t))lcd_draw_point(x,y,fc);//画一个点
// 				x++;
// 				if((x-x0)==sizex)
// 				{
// 					x=x0;
// 					y++;
// 					break;
// 				}
// 			}
// 		}
// 	}  
// }

// void lcd_show_string(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t font_size,uint8_t mode)
// {         
// 	while(*p!='\0')
// 	{       
// 		lcd_show_char(x,y,*p,fc,bc,font_size,mode);
// 		x+=font_size/2;
// 		p++;
// 	}  
// }

/******************************************************************************
*函数说明：显示数字
*入口数据：m底数，n指数
*返回值：  无
******************************************************************************/
/**
 * @brief   计算整数幂次方 m^n
 * @param   m 底数
 * @param   n 指数
 * @retval  m^n 的结果
 * @note    显示数字时用于取每一位（如 1234 取千位用 1234/1000%10）
 */
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	                            /* 初始化结果为 1 */
	while(n--)result*=m;                        /* 重复乘 m，共 n 次 */
	return result;                              /* 返回结果 */
}


/******************************************************************************
*函数说明：显示整数变量
*入口数据：x,y显示坐标
	num 要显示整数变量
	len 要显示的位数
	fc 字的颜色
	bc 字的背景色
	font_size 字号
*返回值：  无
******************************************************************************/
// void lcd_show_integer(uint32_t x,uint32_t y,uint32_t num,uint32_t len,uint32_t fc,uint32_t bc,uint32_t font_size)
// {
// 	u8 t,temp;
// 	u8 enshow=0;
// 	u8 sizex=font_size/2;
// 	for(t=0;t<len;t++)
// 	{
// 		temp=(num/mypow(10,len-t-1))%10;
// 		if(enshow==0&&t<(len-1))
// 		{
// 			if(temp==0)
// 			{
// 				lcd_show_char(x+t*sizex,y,' ',fc,bc,font_size,0);
// 				continue;
// 			}else enshow=1;

// 		}
// 	 	lcd_show_char(x+t*sizex,y,temp+48,fc,bc,font_size,0);
// 	}
// }


/******************************************************************************
*函数说明：显示两位小数变量
*入口数据：x,y显示坐标
		num 要显示小数变量
		len 要显示的位数
		fc 字的颜色
		bc 字的背景色
		font_size 字号
*返回值：  无
******************************************************************************/
// void lcd_show_float(uint32_t x,uint32_t y,float num,uint32_t len,uint32_t fc,uint32_t bc,uint32_t font_size)
// {
// 	uint32_t t,temp,sizex;
// 	uint32_t num1;
// 	sizex=font_size/2;
// 	num1=num*100;
// 	for(t=0;t<len;t++)
// 	{
// 		temp=(num1/mypow(10,len-t-1))%10;
// 		if(t==(len-2))
// 		{
// 			lcd_show_char(x+(len-2)*sizex,y,'.',fc,bc,font_size,0);
// 			t++;
// 			len+=1;
// 		}
// 	 	lcd_show_char(x+t*sizex,y,temp+48,fc,bc,font_size,0);
// 	}
// }

/**
 * @brief   通过PWM设置屏幕亮度(10~100)
 * @param   brightness 亮度值（占空比，0~100）
 * @retval  无
 * @note    该函数被 main.c 的 app_task_istouch_lcd / app_task_mpu6050 任务调用，
 *          实现"抬腕亮屏"和"触摸亮屏"功能。
 *          实际通过修改 TIM3 通道1 的 CCR 寄存器改变 PWM 占空比，
 *          PB4 引脚输出到 LCD 背光控制端（该屏幕背光为低电平点亮）。
 */
void lcd_set_brightness(uint8_t brightness)
{
	uint8_t ccr = brightness;        /* 占空比值赋给 CCR */

	TIM_SetCompare1(TIM3,ccr);       /* 设置 TIM3_CH1 的 CCR1，调节 PWM 占空比 */
}


/**
 * @brief   LCD 显示屏初始化（ST7789V2）
 * @retval  无
 * @note    完成以下工作：
 *          1) 配置 GPIO（SCK/MOSI/CS/DC/RST 以及背光 PWM 复用）
 *          2) 初始化 SPI1（或软件 SPI）
 *          3) 硬件复位 LCD
 *          4) 发送 ST7789V2 初始化序列（颜色格式、电源、Gamma、方向等）
 *          5) 开启显示
 *          该函数在系统启动后由 main.c 调用一次，之后 LVGL 即可使用其接口刷屏
 */
void lcd_init(void) ////ST7789V2
{
	GPIO_InitTypeDef GPIO_InitStructure;            /* GPIO 初始化结构体 */
	SPI_InitTypeDef  	SPI_InitStructure;            /* SPI 初始化结构体 */
	NVIC_InitTypeDef 	NVIC_InitStructure;           /* NVIC 初始化结构体（保留，本函数未使用） */

#if LCD_SOFT_SPI_ENABLE
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE); /* 使能 GPIOD/GPIOE 时钟 */

	// 引脚的配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度设置更高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出，Push Pull，使能了PMOS还有NMOS管
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 不使能上下拉电阻
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7| GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度设置更高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出，Push Pull，使能了PMOS还有NMOS管
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 不使能上下拉电阻
	GPIO_Init(GPIOE, &GPIO_InitStructure);

#else
	/* —— 硬件 SPI 分支：使用 SPI1，引脚 PB3(SCK)/PB5(MOSI)/PB4(背光PWM TIM3_CH1)，
	         PG6/PG7/PG8 作为 CS/DC/RST 控制引脚 —— */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);     /* 使能 GPIOB 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);     /* 使能 GPIOG 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);      /* 使能 SPI1 时钟 */

	// SCK=PB3,  MOSI=PB5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	 //复用功能模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; //引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 //增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //不需要上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	 //复用功能模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; //引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 //增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //不需要上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 //输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; //引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 //增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //不需要上下拉电阻
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	//PB4连接到TIM3 PWM通道1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

	// PB3 PB5连接到SPI1硬件
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);

	//关闭SPI1
	SPI_Cmd(SPI1, DISABLE);

	//设置SPI
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //全双工收发
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					   //设为主机
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;				   // 8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //空闲时时钟为低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;					   //第1个时钟沿捕获数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;						   // CS由SSI位控制（自控）
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //波特率为2分频
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //高位先传送
	SPI_InitStructure.SPI_CRCPolynomial = 0;						   //不使用CRC
	SPI_Init(SPI1, &SPI_InitStructure);								   //初始化SPI1

	//启动SPI1
	SPI_Cmd(SPI1, ENABLE);
	
#endif

	//该1.69英寸屏幕背光为低电平点亮
	//LCD_BLK_0;

	//SPI_SCK_0;

	SPI_SCK_1; // 特别注意！！                          /* CPOL=High，空闲拉高 SCK */
	LCD_RST_0;                                          /* 拉低复位脚 */
	delay_ms(100);                                      /* 保持 100ms 复位 */
	LCD_RST_1;                                          /* 释放复位 */
	delay_ms(100);                                      /* 等待 LCD 稳定 */
	lcd_send_cmd(0x11); // Sleep Out                    /* 退出睡眠模式 */
	delay_ms(120);		// DELAY120ms                  /* 退出睡眠后必须等待 */
	//-----------------------ST7789V Frame rate setting-----------------//
	//************************************************
	lcd_send_cmd(0x3A); // 65k mode                     /* 像素格式 16bpp(RGB565) */
	lcd_send_data(0x05);                                /* 0x05 = RGB565 */
	lcd_send_cmd(0xC5); // VCOM                         /* VCOM 电压控制 */
	lcd_send_data(0x1A);                                /* VCOM=1A 设置 */
	lcd_send_cmd(0x36); // 屏幕显示方向设置             /* 存储器访问控制 */
	lcd_send_data(0x00);                                /* 0°方向，无翻转 */
	//-------------ST7789V Frame rate setting-----------//
	lcd_send_cmd(0xb2); // Porch Setting                /* 前后肩/Porch 设置 */
	lcd_send_data(0x05);                                /* 垂直前肩 */
	lcd_send_data(0x05);                                /* 垂直后肩 */
	lcd_send_data(0x00);                                /* 空白 */
	lcd_send_data(0x33);                                /* 水平前肩 */
	lcd_send_data(0x33);                                /* 水平后肩 */

	lcd_send_cmd(0xb7);	 // Gate Control                /* 栅极驱动控制 */
	lcd_send_data(0x05); // 12.2v   -10.43v            /* VGH/VGL 设置 */
	//--------------ST7789V Power setting---------------//
	lcd_send_cmd(0xBB); // VCOM                          /* VCOM 设置 */
	lcd_send_data(0x3F);                                /* VCOM=0x3F */

	lcd_send_cmd(0xC0); // Power control                /* 电源控制 */
	lcd_send_data(0x2c);                                /* 开启电源 */

	lcd_send_cmd(0xC2); // VDV and VRH Command Enable   /* 启用 VDV/VRH 写入 */
	lcd_send_data(0x01);                                /* 写 VRH/VDV 来自命令 */

	lcd_send_cmd(0xC3);	 // VRH Set                     /* VRH 设置 */
	lcd_send_data(0x0F); // 4.3+( vcom+vcom offset+vdv) /* 电压参考 */

	lcd_send_cmd(0xC4);	 // VDV Set                     /* VDV 设置 */
	lcd_send_data(0x20); // 0v                          /* VDV=0x20 */

	lcd_send_cmd(0xC6);	 // Frame Rate Control in Normal Mode /* 帧率控制 */
	lcd_send_data(0X01); // 111Hz                       /* 111Hz 帧率 */

	lcd_send_cmd(0xd0); // Power Control 1              /* 电源控制1 */
	lcd_send_data(0xa4);                                /* 上电设置 */
	lcd_send_data(0xa1);                                /* AVDD/AVCL 配置 */

	lcd_send_cmd(0xE8); // Power Control 1             /* 电源控制2 */
	lcd_send_data(0x03);                                /* 设置 */

	lcd_send_cmd(0xE9); // Equalize time control        /* 均衡时间控制 */
	lcd_send_data(0x09);                                /* 前均衡 */
	lcd_send_data(0x09);                                /* 后均衡 */
	lcd_send_data(0x08);                                /* 偏移 */
	//---------------ST7789V gamma setting-------------//
	lcd_send_cmd(0xE0); // Set Gamma                    /* 正极性 Gamma */
	lcd_send_data(0xD0);
	lcd_send_data(0x05);
	lcd_send_data(0x09);
	lcd_send_data(0x09);
	lcd_send_data(0x08);
	lcd_send_data(0x14);
	lcd_send_data(0x28);
	lcd_send_data(0x33);
	lcd_send_data(0x3F);
	lcd_send_data(0x07);
	lcd_send_data(0x13);
	lcd_send_data(0x14);
	lcd_send_data(0x28);
	lcd_send_data(0x30);

	lcd_send_cmd(0XE1); // Set Gamma                    /* 负极性 Gamma */
	lcd_send_data(0xD0);
	lcd_send_data(0x05);
	lcd_send_data(0x09);
	lcd_send_data(0x09);
	lcd_send_data(0x08);
	lcd_send_data(0x03);
	lcd_send_data(0x24);
	lcd_send_data(0x32);
	lcd_send_data(0x32);
	lcd_send_data(0x3B);
	lcd_send_data(0x14);
	lcd_send_data(0x13);
	lcd_send_data(0x28);
	lcd_send_data(0x2F);

	lcd_send_cmd(0x21); // 反显                         /* 进入反显模式（黑底白字，常用于 OLED） */

	lcd_send_cmd(0x29); // 开启显示                     /* Display ON，正式点亮屏幕 */
	
}


// void lcd_show_chn(uint32_t x, uint32_t y,uint8_t no, uint32_t fc, uint32_t bc,uint32_t font_size)
// {
// 	uint32_t i,j;
// 	uint8_t tmp;

// 	lcd_addr_set(x, y, x + font_size-1, y + font_size-1);

// 	for (i = 0; i < (font_size*font_size/8); i++) // column loop
// 	{
// 		if(font_size==16)tmp = chinese_tbl_16[no][i];
// 		if(font_size==24)tmp = chinese_tbl_24[no][i];	
// 		if(font_size==32)tmp = chinese_tbl_32[no][i];
		
// 		for (j = 0;j < 8; j++)
// 		{
// 			if (tmp & (1<<j))
// 			{
// 				lcd_send_data(fc >> 8);
// 				lcd_send_data(fc);
// 			}
			
// 			else
// 			{
// 				lcd_send_data(bc);
// 				lcd_send_data(bc);
// 			}
// 		}
// 	}
// }

/**
 * @brief   在指定位置画一个点（像素）
 * @param   x     像素 X 坐标
 * @param   y     像素 Y 坐标
 * @param   color 颜色值（RGB565）
 * @retval  无
 * @note    通过设置单像素地址窗口，再写入 2 字节像素数据
 *          注意：每画一个点都要发地址命令，效率较低，仅适合少量像素操作
 */
void lcd_draw_point(uint32_t x, uint32_t y, uint32_t color)
{
	lcd_addr_set(x, y, x, y);                /* 设置单像素地址窗口 */

	lcd_send_data(color >> 8);                /* 发送颜色高字节 */
	lcd_send_data(color);                     /* 发送颜色低字节 */
}


// void lcd_draw_line(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
// {
// 	uint16_t t; 
// 	int xerr=0,yerr=0,delta_x,delta_y,distance;
// 	int incx,incy,uRow,uCol;
// 	delta_x=x2-x1; //计算坐标增量 
// 	delta_y=y2-y1;
// 	uRow=x1;//画线起点坐标
// 	uCol=y1;
// 	if(delta_x>0)incx=1; //设置单步方向 
// 	else if (delta_x==0)incx=0;//垂直线 
// 	else {incx=-1;delta_x=-delta_x;}
// 	if(delta_y>0)incy=1;
// 	else if (delta_y==0)incy=0;//水平线 
// 	else {incy=-1;delta_y=-delta_y;}
// 	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
// 	else distance=delta_y;
// 	for(t=0;t<distance+1;t++)
// 	{
// 		lcd_draw_point(uRow,uCol,color);//画点
// 		xerr+=delta_x;
// 		yerr+=delta_y;
// 		if(xerr>distance)
// 		{
// 			xerr-=distance;
// 			uRow+=incx;
// 		}
// 		if(yerr>distance)
// 		{
// 			yerr-=distance;
// 			uCol+=incy;
// 		}
// 	}
// }

// void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
// {
// 	lcd_draw_line(x1,y1,x2,y1,color);
// 	lcd_draw_line(x1,y1,x1,y2,color);
// 	lcd_draw_line(x1,y2,x2,y2,color);
// 	lcd_draw_line(x2,y1,x2,y2,color);
// }

// void lcd_draw_circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
// {
// 	int a,b;
	
// 	a=0;b=r;
	
// 	while(a<=b)
// 	{
// 		lcd_draw_point(x0-b,y0-a,color);             //3           
// 		lcd_draw_point(x0+b,y0-a,color);             //0           
// 		lcd_draw_point(x0-a,y0+b,color);             //1                
// 		lcd_draw_point(x0-a,y0-b,color);             //2             
// 		lcd_draw_point(x0+b,y0+a,color);             //4               
// 		lcd_draw_point(x0+a,y0-b,color);             //5
// 		lcd_draw_point(x0+a,y0+b,color);             //6 
// 		lcd_draw_point(x0-b,y0+a,color);             //7
// 		a++;
// 		if((a*a+b*b)>(r*r))//判断要画的点是否过远
// 		{
// 			b--;
// 		}
// 	}
// }


/**
 * @brief   设置 LCD 显示方向（0/90/180/270 度）
 * @param   dir 方向：0=0°，1=90°，2=180°，3=270°
 * @retval  无
 * @note    通过写 0x36 (MADCTL) 寄存器控制 GRAM 扫描方向：
 *          - bit7 MY  ：行地址顺序翻转
 *          - bit6 MX  ：列地址顺序翻转
 *          - bit5 MV  ：行列交换（横竖屏切换）
 *          - bit4 ML  ：垂直刷新方向翻转
 *          同时更新 g_lcd_width/g_lcd_height 全局变量，供上层使用
 */
void lcd_set_direction(uint32_t dir)
{
	g_lcd_direction = dir;                        /* 保存方向到全局变量 */

	/* 0°*/
	if(dir==0)
	{
		lcd_send_cmd(0x36);                       /* MADCTL 寄存器 */
		lcd_send_data(0x00);                      /* 不翻转，竖屏 */
		g_lcd_width=LCD_WIDTH;                    /* 宽=原始宽 */
		g_lcd_height=LCD_HEIGHT;                  /* 高=原始高 */
	}

	/* 90°*/
	if(dir==1)
	{
		lcd_send_cmd(0x36);                       /* MADCTL 寄存器 */
		lcd_send_data((1<<6)|(1<<5)|(1<<4));      /* MX|MV|ML，横屏 */
		g_lcd_width=LCD_HEIGHT;                   /* 宽高互换 */
		g_lcd_height=LCD_WIDTH;

	}


	/* 180°*/
	if(dir==2)
	{
		lcd_send_cmd(0x36);                       /* MADCTL 寄存器 */
		lcd_send_data((1<<7)|(1<<6));             /* MY|MX，180度翻转，竖屏 */
		g_lcd_width=LCD_WIDTH;
		g_lcd_height=LCD_HEIGHT;

	}

	/* 270°*/
	if(dir==3)
	{
		lcd_send_cmd(0x36);                       /* MADCTL 寄存器 */
		lcd_send_data((1<<7)|(0<<6)|(1<<5)|(1<<4)); /* MY|MV|ML，横屏 */
		g_lcd_width=LCD_HEIGHT;
		g_lcd_height=LCD_WIDTH;

	}

}

/**
 * @brief   lcd显示打开/关闭
 * @param   on 1=开显示，0=关显示
 * @retval  无
 * @note    0x29 = Display ON，0x28 = Display OFF
 */
void lcd_display_on(uint16_t on)
{
	if(on)
		lcd_send_cmd(0x29);	 	// 命令                         /* 开显示 */
	else
		lcd_send_cmd(0x28);	 	// 命令                         /* 关显示 */
}


/**
 * @brief   获取 LCD 当前显示方向
 * @retval  当前方向：0/1/2/3
 * @note    用于 lcd_addr_set 中判断偏移量方向
 */
uint32_t lcd_get_direction(void)
{
	return g_lcd_direction;                       /* 返回当前方向变量 */
}

/**
 * @brief   初始化 SPI1 TX DMA（DMA2_Stream3 + 通道3）
 * @param   DMA_Memory0BaseAddr  内存源基地址（要发送的缓冲区起始地址）
 * @param   DMA_BufferSize        传输数据量（单位：节点数）
 * @param   DMA_MemoryDataSize    内存数据宽度（Byte/HalfWord/Word）
 * @param   DMA_MemoryInc         内存是否自增（DMA_MemoryInc_Enable/Disable）
 * @retval  无
 * @note    该 DMA 把内存中 LVGL 的帧缓冲通过 SPI1 推送到 LCD GRAM，
 *          大幅降低 CPU 占用。配置完成后使能 DMA 并打开传输完成中断，
 *          中断中通知 LVGL 缓冲已刷完。
 */
void spi1_tx_dma_init(uint32_t DMA_Memory0BaseAddr, uint16_t DMA_BufferSize, uint32_t DMA_MemoryDataSize, uint32_t DMA_MemoryInc)
{
	NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef 	DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // DMA2时钟使能

	DMA_DeInit(DMA2_Stream3);

	// 等待DMA2_Stream1可配置
	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE)
		;

	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;							// 通道3 SPI1通道
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;			// 外设地址为:SPI1->DR
	DMA_InitStructure.DMA_Memory0BaseAddr = DMA_Memory0BaseAddr;			// DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// 存储器到外设模式
	DMA_InitStructure.DMA_BufferSize = DMA_BufferSize;						// 数据传输量
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		// 外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc;						// 存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize;				// 存储器数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// 正常模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						// 高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;					// 禁用FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;			//
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				// 外设突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		// 存储器突发单次传输
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);

	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);

	DMA_Cmd(DMA2_Stream3, ENABLE);


	/* 开启传输完成中断  */
    DMA_ITConfig(DMA2_Stream3,DMA_IT_TC,ENABLE);

    // 中断初始化
    /* DMA发送中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
    /* 抢断优先级 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
    /* 响应优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    /* 使能外部中断通道 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 配置NVIC */
    NVIC_Init(&NVIC_InitStructure);

}

/**
 * @brief   启动 SPI1 TX DMA 传输
 * @retval  无
 * @note    调用前需先由 spi1_tx_dma_init 配置好 DMA，并拉低 SPI_CS
 */
void spi1_tx_dma_start(void)
{
	DMA_Cmd(DMA2_Stream3, ENABLE);                       /* 使能 DMA2_Stream3，启动传输 */
}

/**
 * @brief   停止 SPI1 TX DMA 传输
 * @retval  无
 * @note    通常在传输完成中断后或需要中止传输时调用
 */
void spi1_tx_dma_stop(void)
{
	DMA_Cmd(DMA2_Stream3, DISABLE);                      /* 禁用 DMA2_Stream3，停止传输 */
}

extern lv_disp_drv_t *g_disp_drvp;                       /* LVGL 显示驱动指针，由 lv_port_disp.c 设置 */


/**
 * @brief   DMA2_Stream3 中断服务函数（SPI1 TX 完成中断）
 * @retval  无
 * @note    当 SPI1 DMA 发送完一帧数据后触发，主要工作：
 *          1) 进入 FreeRTOS 临界区，保护共享数据
 *          2) 清除 DMA TC 中断标志
 *          3) 通知 LVGL 该帧已刷完（lv_disp_flush_ready）
 *          4) 拉高 CS 结束本次 SPI 通信
 *          5) 退出临界区
 *          注意优先级设为 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，
 *          确保可调用 FreeRTOS 的 FromISR API
 */
void DMA2_Stream3_IRQHandler(void)
{
	uint32_t ulReturn;

	/* 进入中断临界区 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();            /* 进入 ISR 临界区，保存 BASEPRI */

    // DMA 发送完成
    if(DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3))
    {
        // 清除DMA发送完成标志
        DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);

		lv_disp_flush_ready(g_disp_drvp);               /* 通知 LVGL 当前 flush 完成，可继续下一帧 */

        // 片选拉高，数据发送完毕
        SPI_CS_1;                                       /* CS 拉高，结束本次 SPI 通信 */

    }

	/* 退出中断临界区*/
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);                /* 退出 ISR 临界区，恢复 BASEPRI */
}

