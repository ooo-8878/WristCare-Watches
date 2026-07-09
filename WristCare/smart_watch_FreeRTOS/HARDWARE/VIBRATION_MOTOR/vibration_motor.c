/**
 ******************************************************************************
 * @file    vibration_motor.c
 * @brief   震动电机驱动模块
 *
 * @details 本文件实现智能手表震动电机的 GPIO 初始化与开关控制。
 *          电机通过一个数字引脚直接驱动(经三极管/MOS 管扩流),
 *          高电平开启震动,低电平关闭震动。
 *
 * @hardware 硬件资源:
 *           - GPIOB Pin_1:震动电机控制引脚(推挽输出)
 *           - 时钟总线:AHB1(GPIOB 挂载于 AHB1 总线)
 *           - 引脚配置:推挽输出,50MHz,无上下拉
 *
 * @control 控制方式:
 *           - 通过宏 VIBRATION_MOTOR(x) 控制电机开关,
 *             宏定义见头文件(通常展开为 PBout(1)=x)
 *             VIBRATION_MOTOR(1):开  VIBRATION_MOTOR(0):关
 *
 * @freertos 与 FreeRTOS 的交互方式:
 *           - 本驱动仅为底层 GPIO 操作,无阻塞,无 RTOS API 调用。
 *           - 由 main.c 中的 app_task_motor 任务调用 VIBRATION_MOTOR
 *             宏实现定时震动(如来电提醒、闹钟等)。
 *           - 任务可通过 vTaskDelay 控制震动持续时长与间隔。
 *
 * @design 设计要点:
 *           - 推挽输出可提供较强驱动能力,但仍需外接驱动管保护 GPIO;
 *           - 初始化完成后默认关闭电机,避免上电误动作;
 *           - 长时间震动需注意功耗,电池供电场景需控制时长。
 ******************************************************************************
 */
#include "includes.h"      /* 包含工程统一头文件,提供 GPIO 标准库函数及 VIBRATION_MOTOR 宏 */

/**
  * @brief  震动电机初始化函数
  * @note   配置 GPIOB Pin_1 为推挽输出,初始化完成后默认关闭电机。
  *         调用位置:系统初始化阶段(在 app_task_motor 创建之前),
  *         之后由 app_task_motor 任务通过 VIBRATION_MOTOR 宏控制开关。
  * @param  None
  * @retval None
  */
void vibration_motor_init(void)
{
	//传参结构体
	GPIO_InitTypeDef GPIO_InitStruct;          /* GPIO 初始化参数结构体,用于配置引脚工作模式等 */

	//开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   /* 使能 GPIOB 端口所在 AHB1 总线时钟,GPIOB 挂在 AHB1 上 */
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;                /* 选择 PB1 引脚作为电机控制引脚 */
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;//输出模式    /* 设置为通用输出模式(非复用、非模拟) */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽        /* 推挽输出,可输出高/低电平,驱动能力较强 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//引脚速度50MHz /* 引脚翻转速率 50MHz,满足 GPIO 快速开关需求 */
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;//无上下拉 /* 不启用内部上拉/下拉电阻,由外部电路决定电平 */

	GPIO_Init(GPIOB,&GPIO_InitStruct); //GPIO初始化          /* 将上述配置写入 GPIOB 寄存器,完成 PB1 初始化 */

	VIBRATION_MOTOR(0);//震动马达关                          /* 初始化完成后关闭电机,防止上电瞬间误震动 */
}