/**
 * @file    bt24.c
 * @version V1.0
 * @date    2026-07-08
 * @brief   BT24 蓝牙模块驱动实现文件
 *
 * @section 模块功能
 *   本文件实现 BT24 蓝牙模块(透传型串口蓝牙模块)与 STM32F4 的通信驱动,
 *   提供 AT 指令发送、透传数据收发、连接状态查询等功能,用于智能手表与
 *   手机端 App 的双向数据交互(如时间同步、消息推送、传感器数据上报等)。
 *
 * @section 硬件资源
 *   - 通信接口: USART3 (波特率可配置,默认由 bt24_init 入参指定)
 *   - 引脚映射:
 *       * PB10  -> USART3_TX (复用功能 AF7, 推挽输出, 100MHz)
 *       * PB11  -> USART3_RX (复用功能 AF7, 无上下拉)
 *       * PB7   -> 蓝牙连接状态输入引脚(普通输入,下拉,用于查询主从连接状态)
 *   - 时钟:
 *       * RCC_AHB1Periph_GPIOB   (GPIOB 端口时钟)
 *       * RCC_APB1Periph_USART3  (USART3 外设时钟, 挂在 APB1 总线)
 *   - 中断: USART3_IRQn (RXNE 接收中断 + IDLE 空闲中断)
 *
 * @section 通信协议
 *   - 串口参数: 8 位数据位 / 1 位停止位 / 无校验位 / 无硬件流控
 *   - 数据格式: 字符串指令(AT 指令)或二进制透传数据
 *   - 接收策略: 采用 "RXNE 逐字节接收 + IDLE 一帧结束" 组合中断方式,
 *               每当总线空闲时认为收到一帧完整数据
 *
 * @section 与 FreeRTOS 的交互方式
 *   - ISR 与任务解耦: USART3_IRQHandler 在 IDLE 中断中通过
 *                    xQueueSendFromISR() 把一帧数据投递到全局消息队列
 *                    g_queue_bt24_handle(在 USER/main.c 中创建, 长度 10,
 *                    每项 32 字节)。
 *   - 消费任务: USER/main.c 中的 app_task_ble 任务通过
 *              xQueueReceive(g_queue_bt24_handle, buffer, portMAX_DELAY)
 *              阻塞等待并解析蓝牙数据。
 *   - 临界区保护: 中断内部使用 taskENTER_CRITICAL_FROM_ISR() /
 *                taskEXIT_CRITICAL_FROM_ISR() 进入/退出临界区,
 *                保护 static 缓冲区 index 与队列发送操作的原子性。
 *
 * @section 设计要点
 *   1. 接收缓冲区设计为 32 字节 static 数组, 与队列项大小严格对应,
 *      避免越界写入;index 计数在到达数组容量时停止累加(防御性编程)。
 *   2. IDLE 中断后立即 "假读" 一次 USART3->DR, 再清 IDLE 标志,
 *      这是 STM32F4 标准的清除 IDLE 标志的推荐顺序。
 *   3. 中断优先级配置为 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
 *      保证可以安全调用 FreeRTOS 的 ...FromISR() API。
 *   4. 发送采用阻塞轮询 TXE 标志的方式,适用于指令量小、对实时性要求
 *      不高的场景;若需高速发送可改用 DMA + 中断方式。
 *   5. bt24_data 全局结构(rx_buf/rx_len/rx_flag)预留用于上层缓存,
 *      bt24_clear_struct() 提供清零接口。
 *
 * @author  STM32工程师
 */

#include "includes.h"

/* 全局蓝牙数据缓存结构体,定义在 bt24.h 中,包含 128 字节接收缓冲、
   接收长度、接收完成标志;供上层任务查询与处理 */
bt24_t bt24_data = {0};

/**
  * @brief  BT24 蓝牙模块初始化,完成 GPIO/USART3/NVIC 配置并使能外设
  * @param  baud: 串口波特率(常用 9600 / 115200,需与蓝牙模块出厂或 AT 配置一致)
  * @retval 无
  * @note   - PB7 配置为下拉输入,读取蓝牙模块的连接状态电平
  *         - PB10/PB11 复用为 USART3_TX/RX,需先选 AF 模式再调用 GPIO_PinAFConfig
  *         - 中断优先级设为 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
  *           确保 ISR 内可安全调用 FreeRTOS 的 FromISR 系列 API
  *         - 同时使能 RXNE(逐字节接收)与 IDLE(一帧结束)两类中断
  */
void bt24_init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStruct;             /* GPIO 初始化结构体,用于配置 PB7/PB10/PB11 */
	USART_InitTypeDef USART_InitStruct;           /* USART3 初始化结构体,配置波特率/校验/停止位等 */
	NVIC_InitTypeDef NVIC_InitStructure;         /* NVIC 嵌套向量中断控制器初始化结构体 */
	GPIO_InitTypeDef GPIO_InitStructure; /* 添加变量声明 */ /* 备用 GPIO 结构体(本函数实际未使用) */

	/* 开启时钟:GPIOB 挂在 AHB1,USART3 挂在 APB1 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   /* 使能 GPIOB 端口时钟,供 PB7/PB10/PB11 使用 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); /* 使能 USART3 外设时钟 */

	/* bt24 连接状态引脚 PB7:普通输入,下拉,读取蓝牙模块 STAT 引脚电平 */
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_7;              /* 选中 PB7 引脚 */
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;/*输入模式*/ /* 配置为通用输入模式,读取外部电平 */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;/*推挽*/    /* 输入模式下该字段无实际意义,保持默认 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;/*引脚速度50MHz*/ /* 引脚翻转速度 50MHz */
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;/*下拉*/  /* 内部下拉,无连接时默认低电平(未连接状态) */
	GPIO_Init(GPIOB,&GPIO_InitStruct);                   /* 写入 GPIOB 寄存器,完成 PB7 配置 */

	/* 配置 GPIO:PB10(TX)、PB11(RX) 复用为 USART3 */
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11; /* 同时选中 PB10 与 PB11 */
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;/*复用模式*/   /* 复用功能模式,引脚交由 USART3 外设控制 */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;/*推挽*/      /* TX 推挽输出(单端驱动),RX 配置无影响 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;/*引脚速度100MHz*/ /* 高速翻转,适应高波特率 */
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;/*无上拉下拉*/ /* 不启用内部上下拉,由外部电路决定 */
	GPIO_Init(GPIOB,&GPIO_InitStruct);                       /* 写入 GPIOB 寄存器,完成 PB10/PB11 配置 */

	/* 连接 PB10 和 PB11 到串口3 的复用功能(AF7 = USART3) */
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); /* PB10 -> USART3_TX */
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); /* PB11 -> USART3_RX */

	/* 配置串口3 通信参数 */
	USART_InitStruct.USART_BaudRate = baud;/*波特率*/                       /* 波特率由调用方传入 */
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;/*发送和接收模式*/ /* 同时使能发送与接收 */
	USART_InitStruct.USART_Parity = USART_Parity_No;/*无校验位*/            /* 不使用奇偶校验 */
	USART_InitStruct.USART_StopBits = USART_StopBits_1;/*1位停止位*/        /* 1 个停止位 */
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;/*8位数据长度*/  /* 8 位数据帧 */
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*无硬件控制流*/ /* 不使用 RTS/CTS 硬件流控 */
	USART_Init(USART3,&USART_InitStruct);                                   /* 写入 USART3 配置寄存器 */

	/* 配置 USART3 中断优先级并使能 NVIC 通道 */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; /* 选择 USART3 全局中断通道 */
	/* 抢占优先级设为 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(通常为 5/15),
	   这是允许调用 FreeRTOS FromISR API 的最高中断优先级数值上限 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* 子优先级 0,同抢占级内最先响应 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    /* 使能该中断通道 */
	NVIC_Init(&NVIC_InitStructure);                   /* 写入 NVIC 寄存器 */

	/* 使能串口接收中断:每收到 1 个字节触发一次 RXNE 中断 */
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);

    /* 使能串口空闲中断:总线空闲一帧时间后触发 IDLE,标志一帧接收结束 */
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);

	/* 使能串口3 外设,开始工作 */
	USART_Cmd(USART3,ENABLE);

	/* 清空全局接收缓存结构体,避免上电残留脏数据 */
	bt24_clear_struct(&bt24_data);
}

/**
  * @brief  通过 USART3 发送一个以 '\0' 结尾的字符串(常用于发送 AT 指令)
  * @param  str: 待发送的字符串指针(必须以 '\0' 结尾)
  * @retval 无
  * @note   - 本函数为阻塞式发送,在 FreeRTOS 任务中调用时会让出 CPU
  *           直到字符串全部发出,调用方需注意栈与任务优先级
  *         - 每发一个字节后轮询 TXE(发送数据寄存器空)标志,
  *           为 0 时表示上一字节仍在移位寄存器中,需等待
  *         - 注意此处轮询的是 TXE 而非 TC(发送完成),效率略高
  *           但下一字节写入后需保证移位寄存器可用,本场景可接受
  */
void bt24_sendstr(const char *str)
{
	while(*str != '\0')                       /* 遍历字符串直到结束符 */
	{
		USART_SendData(USART3,*str++);        /* 把当前字符写入 USART3 数据寄存器(DR),指针后移 */
		/* 轮询 TXE 标志:RESET 表示 DR 还没空(上一字节尚未搬入移位寄存器),继续等待 */
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);/*等待发送完成*/
	}
}

/**
  * @brief  通过 USART3 发送定长二进制缓冲区(用于透传二进制数据)
  * @param  buf: 待发送数据缓冲区指针
  * @param  len: 待发送的字节数
  * @retval 无
  * @note   - 与 bt24_sendstr 的区别:不以 '\0' 作为结束判定,按长度发送,
  *           因此可发送包含 0x00 的二进制帧
  *         - 同样为阻塞轮询 TXE 的方式,适用于小数据量场景
  *         - 长数据建议改用 DMA + TC 中断以降低 CPU 占用
  */
void bt24_sendbuf(const char *buf,uint16_t len)
{
	while(len--)                              /* 按长度递减循环,直至发送完毕 */
	{
		USART_SendData(USART3,*buf++);        /* 写入 DR 启动一次发送,buf 指针后移 */
		/* 等待 TXE 置位:确保上一字节已经从 DR 搬入移位寄存器,可以写下一字节 */
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);/*等待发送完成*/
	}
}

/**
  * @brief  读取 BT24 蓝牙模块的连接状态(已连接 / 未连接)
  * @param  无
  * @retval 1: 已建立蓝牙连接; 0: 未连接
  * @note   - 读取 PB7 引脚电平,该引脚在 bt24_init() 中配置为下拉输入
  *         - 蓝牙模块的 STAT/STATE 引脚通常在主从连接成功后输出高电平
  *         - 由于硬件下拉,模块未上电或未连接时电平稳定为 0,避免误判
  */
uint8_t bt24_connect_status(void)
{
	uint8_t status = 0;                       /* 默认状态:未连接 */

	status = PBin(7);                         /* 读取 PB7 输入电平并保存 */

	return status;                           /* 返回连接状态:1=已连接,0=未连接 */
}

/**
  * @brief  清空 bt24_t 接收缓存结构体(复位缓冲、长度、标志)
  * @param  data: 指向待清空的 bt24_t 结构体
  * @retval 无
  * @note   - 使用 memset 把 rx_buf 中的有效长度部分清零,
  *           清零长度取当前 rx_len(已接收字节数)
  *         - 随后复位 rx_len 与 rx_flag,供下一次接收使用
  *         - 在中断或任务中均可调用,但若多任务访问需加临界区保护
  */
void bt24_clear_struct(bt24_t *data)
{
	memset(data->rx_buf,0,data->rx_len);      /* 把接收缓冲中的有效字节清零 */
	data->rx_len = 0;                         /* 复位接收长度计数 */
	data->rx_flag = 0;                        /* 清除接收完成标志 */
}

/**
  * @brief  USART3 全局中断服务函数,处理蓝牙数据的接收
  * @param  无
  * @retval 无
  * @note   工作流程:
  *         1) RXNE 中断:每收到 1 字节,从 DR 读出并写入 static buffer,
  *            index 自增;index 到达 buffer 容量后停止累加(防越界)
  *         2) IDLE 中断:总线空闲表示一帧结束,把 buffer 通过
  *            xQueueSendFromISR() 投递给 app_task_ble 任务,
  *            然后清空 buffer 与 index,准备下一帧
  *         3) 整个 ISR 被 taskENTER/EXIT_CRITICAL_FROM_ISR 包裹,
  *            保护 static 变量与队列操作;同时屏蔽其他同优先级及以下中断
  *         4) IDLE 标志清除需先 "假读" SR 再读 DR,本处用 USART_ReceiveData
  *            完成 DR 读取步骤(STM32F4 标准 IDLE 清除序列)
  */
void USART3_IRQHandler(void)
{
	uint8_t Data;                              /* 临时存放从 DR 读出的当前字节 */
	static uint8_t buffer[32] = {0};/*缓冲数组*/ /* 一帧接收缓冲,32 字节;static 保证在多次中断间保留状态,与队列项大小一致 */
    static uint8_t index = 0;                 /* 当前一帧已写入 buffer 的字节数,跨中断保留 */
	uint32_t ulReturn;                        /* 保存临界区进入前的中断屏蔽状态,用于恢复 */

	/* 进入中断临界区:屏蔽所有可屏蔽中断,保护 static 变量与队列操作的原子性 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();

	/* ---------- RXNE 接收中断处理 ---------- */
	if(USART_GetITStatus(USART3,USART_IT_RXNE) == SET)/*接收中断*/ /* 判断是否为接收寄存器非空中断 */
	{
		Data = USART_ReceiveData(USART3);      /* 读取 DR,同时自动清除 RXNE 标志 */
        if(index < sizeof(buffer))            /* 防御性判断:确保不越界写入 buffer(32 字节) */
        {
            buffer[index++] = Data;           /* 将字节存入 buffer 并递增 index */
        }
        /* 注意:超出容量时直接丢弃,避免缓冲区溢出破坏内存 */
    
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);/*清除中断标志*/ /* 兜底清除 RXNE 挂起位(读 DR 通常已清,此处保险) */
	}

	/* ---------- IDLE 空闲中断处理:一帧接收完成 ---------- */
    if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET)/*空闲中断*/ /* 判断是否为总线空闲中断 */
	{
        /* 关键步骤:STM32F4 清除 IDLE 标志的标准序列是 "读 SR + 读 DR"。
           这里通过 USART_ReceiveData 再读一次 DR(假读),配合上面读 SR,
           确保 IDLE 标志被硬件清除,避免反复进中断 */
        USART_ReceiveData(USART3);/*假读*/    /* 假读 DR,完成 IDLE 清除序列 */

        /* 把本帧数据通过消息队列发送给 app_task_ble 任务(在 USER/main.c 中)
           - 第 1 个参数:队列句柄 g_queue_bt24_handle
           - 第 2 个参数:待发送数据起始地址(buffer)
           - 第 3 个参数:高优先级任务就绪标志,此处传 NULL 表示不需要任务切换请求
           注意:此处未传 pdFALSE 的高优先级唤醒指针,可能错过立即切换;
           若需低延迟,可改为传入 pxHigherPriorityTaskWoken 并在退出临界区前 portYIELD_FROM_ISR() */
        xQueueSendFromISR(g_queue_bt24_handle,buffer,NULL);
        
        memset(buffer,0,index);/*清空缓冲数组*/ /* 把本帧用过的字节清零,便于下次接收识别 */
        index = 0;                            /* 复位索引,准备接收下一帧 */

		USART_ClearITPendingBit(USART3,USART_IT_IDLE);/*清除中断标志*/ /* 兜底清除 IDLE 挂起位 */
	}

	/* 退出中断临界区:恢复进入前的中断屏蔽状态 */
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}

