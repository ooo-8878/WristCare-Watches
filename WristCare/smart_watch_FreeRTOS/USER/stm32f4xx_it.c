/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   STM32F4xx 中断服务程序集中处理文件
 *
 * @details 本文件负责实现 Cortex-M4 内核异常(系统异常)处理函数,
 *          是 STM32 启动文件 startup_stm32f4xx.s 中断向量表所引用的
 *          C 语言入口函数集合。主要包括以下内容:
 *
 *          1. 系统异常处理函数(本文件实现):
 *             - NMI_Handler        不可屏蔽中断
 *             - HardFault_Handler  硬件错误(死循环便于调试)
 *             - MemManage_Handler  内存管理错误(MPU 违规)
 *             - BusFault_Handler   总线错误
 *             - UsageFault_Handler 使用错误(除零/未对齐等)
 *             - DebugMon_Handler   调试监控异常
 *
 *          2. 已被 FreeRTOS 占用的系统异常(本文件注释掉,避免符号冲突):
 *             - SVC_Handler        系统服务调用,FreeRTOS port.c 实现
 *             - PendSV_Handler     任务上下文切换,FreeRTOS port.c 实现
 *             - SysTick_Handler    系统节拍,FreeRTOS port.c 实现
 *
 * @hardware 本文件不直接操作外设,但与 FreeRTOS 的 port.c 紧密关联:
 *           - SysTick 提供 RTOS 调度节拍(由 configTICK_RATE_HZ 决定)
 *           - PendSV 用于任务切换
 *           - SVC 用于首次任务启动
 *
 * @freertos 与 FreeRTOS 的交互方式:
 *           - SVC/PendSV/SysTick 由 FreeRTOS 的 ARM_CM4F port 接管,
 *             本文件必须将三者注释,否则会出现符号重复定义链接错误。
 *           - 外设中断(如 USART1_IRQHandler)中可调用
 *             taskENTER_CRITICAL_FROM_ISR/taskEXIT_CRITICAL_FROM_ISR
 *             保护共享资源,并可通过 XFromISR API 与 RTOS 任务通信。
 *
 * @design   设计要点:
 *           - HardFault 等致命异常进入死循环,便于调试器捕获现场;
 *             生产环境可改为触发系统复位 NVIC_SystemReset()。
 *           - 本文件保持简洁,具体外设中断在各驱动文件中实现。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "includes.h"      /* 包含工程统一的头文件集合,涵盖 STM32F4xx 标准库、FreeRTOS、各硬件驱动 */
 

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/  /* 私有类型定义区(本文件未使用,空) */
/* Private define ------------------------------------------------------------*/ /* 私有宏定义区(本文件未使用,空) */
/* Private macro -------------------------------------------------------------*/ /* 私有宏函数区(本文件未使用,空) */
/* Private variables ---------------------------------------------------------*/ /* 私有变量定义区(本文件未使用,空) */
/* Private function prototypes -----------------------------------------------*/ /* 私有函数声明区(本文件未使用,空) */
/* Private functions ---------------------------------------------------------*/ /* 私有函数实现区(本文件未使用,空) */

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */  /* Cortex-M4 内核异常(系统异常)处理函数区 */
/******************************************************************************/

/**
  * @brief  NMI 不可屏蔽中断处理函数
  * @note   NMI(Non-Maskable Interrupt)优先级为 -2,无法被屏蔽,通常由看门狗、
  *         时钟安全系统(CSS)或 Flash 校验错误触发。此处为占位空实现,
  *         实际项目中可加入故障记录与系统保护逻辑。
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  硬件错误(HardFault)异常处理函数
  * @note   当内核发生严重故障(如取指失败、非法内存访问、未对齐访问等)且
  *         无法被 MemManage/BusFault/UsageFault 处理时进入此中断。
  *         调试时可通过此函数定位崩溃点,生产环境可加入复位或日志记录。
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */  /* 发生硬件错误时进入死循环,便于调试器捕获现场 */
  while (1)
  {
  }
}

/**
  * @brief  内存管理异常(MemManage)处理函数
  * @note   由 MPU(内存保护单元)违规访问或未对齐访问触发,
  *         例如访问未授权的内存区域。此处死循环便于调试定位。
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */  /* 内存管理异常发生时进入死循环 */
  while (1)
  {
  }
}

/**
  * @brief  总线错误(BusFault)异常处理函数
  * @note   由总线访问错误触发,例如访问无效外设地址、Flash 读取错误等。
  *         此处死循环便于调试时查看故障寄存器(CFSR/BFAR)。
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */  /* 总线错误发生时进入死循环 */
  while (1)
  {
  }
}

/**
  * @brief  使用错误(UsageFault)异常处理函数
  * @note   由除零、未对齐访问、协处理器错误或栈溢出等触发。
  *         此处死循环便于调试定位具体错误原因。
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */  /* 使用错误发生时进入死循环 */
  while (1)
  {
  }
}

/**
  * @brief  SVC(系统服务调用)异常处理函数
  * @note   FreeRTOS 中 SVC 指令用于触发系统服务调用,任务通过
  *         SVC 进入特权模式执行内核服务。本工程中该函数由 FreeRTOS
  *         的 portable/GCC/ARM_CM4F/port.c 文件实现,因此本文件中将其
  *         注释掉以避免符号重复定义,出现 "multiple definition" 链接错误。
  *         若使用裸机工程,可在此处实现 SVC 服务。
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  调试监控异常(DebugMon)处理函数
  * @note   用于软件调试,在单步/断点调试时被触发。
  *         正常运行时不使用,此处为空实现。
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  PendSV(可挂起系统调用)异常处理函数
  * @note   PendSV 是 FreeRTOS 任务切换的核心。FreeRTOS 在 port.c 中
  *         实现了 PendSV_Handler,用于在低优先级中断上下文中完成
  *         任务上下文保存与切换。若在此重复定义将导致链接错误,
  *         故注释掉。任务切换的实际逻辑见 FreeRTOS port.c。
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  SysTick 系统滴答定时器中断处理函数
  * @note   SysTick 是 FreeRTOS 的系统节拍源,用于产生任务调度节拍。
  *         FreeRTOS 在 port.c 中将 SysTick_Handler 实现为调用
  *         xPortSysTickHandler(),进而驱动 RTOS 任务调度与超时机制。
  *         若在此重复定义将导致符号冲突,故注释掉。
  *         节拍频率由 configTICK_RATE_HZ (FreeRTOSConfig.h) 决定。
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//	
//	
//	
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */  /* STM32F4 外设中断服务函数区 */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */  /* 在此添加外设中断处理函数,具体名称见启动文件 */
/*  available peripheral interrupt handler's name please refer to the startup */  /* 启动文件: startup_stm32f4xx.s */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  外设中断处理函数模板(示例,默认注释关闭)
  * @note   PPP 为占位符,实际使用时替换为具体外设名,如 USART1_IRQHandler、
  *         TIM3_IRQHandler 等。本工程中实际使用的中断服务函数分别在
  *         usart.c(USART1_IRQHandler)和定时器文件中实现。
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
