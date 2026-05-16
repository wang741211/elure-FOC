/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines. (中断服务子程序)
  * 本文件包含了系统所有的底层硬件中断入口。所有硬件触发的事件
  * （如ADC转换完成、定时器捕获、比较器越限等）都会首先进入这里的对应函数，
  * 然后由 HAL 库清理标志位，最终调用 main.c 中的弱回调函数(Callback)。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "FOC_Model.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern COMP_HandleTypeDef hcomp1;
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN EV */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];
/* USER CODE END EV */

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers          */
/* Cortex-M4 内核级中断与异常处理函数                                 */
/******************************************************************************/

/**
  * @brief This function handles Non maskable interrupt.
  * 不可屏蔽中断 (NMI)
  */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  * 硬件错误中断 (通常由内存越界、野指针等严重错误引起)
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  * 内存管理错误
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  * 总线错误 (如访问了无效的存储器地址)
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  * 用法错误 (如执行了未定义指令)
  */
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  * SVC 系统调用 (RTOS 常用于触发任务调度)
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Debug monitor.
  * 调试监视器中断
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  * PendSV 中断 (RTOS 常用于上下文切换)
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  * SysTick 系统滴答定时器 (每 1ms 触发一次，用于 HAL 库时间基准 `HAL_Delay()`)
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* 外设级硬件中断服务程序                                                       */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  * DMA1 通道1 中断 (本工程用于 USART3 的数据快速发送，实现虚拟示波器数据泵出)
  */
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  * 【核心频率调度源】：ADC1 和 ADC2 全局中断
  * 由 TIM1 的 CCR4 触发 ADC 注入组采样，采样完成后触发此中断。
  * 这是驱动整个 FOC 算法 (FOC_Model_step) 运行的心跳节拍 (Heartbeat)。
  */
void ADC1_2_IRQHandler(void)
{
  /* 分别调用 ADC1 和 ADC2 的 HAL 层处理函数，随后会触发 HAL_ADCEx_InjectedConvCpltCallback */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  * FDCAN1 接收中断，用于与上位机或其他控制器进行通讯
  */
void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* 从 FIFO0 中提取 CAN 接收到的数据 */
  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
}

/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  * TIM1 刹车(Break)中断。
  * 当发生极端的底层硬件过流、或者死区短路时，硬件 Break 机制会瞬间封锁 PWM，
  * 随后触发该中断，软件可以在此做电机保护状态机的切换或报警。
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  * TIM1 计数溢出/更新中断
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  * TIM1 捕获/比较中断
  */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles TIM4 global interrupt.
  * TIM4 全局中断，本工程配置为输入捕获模式 (Input Capture)，
  * 专门用于捕获霍尔传感器 (Hall) 引脚上的电平边沿跳变，测量转子位置和速度。
  */
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  * USART3 串口接收中断
  */
void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  * 外部中断线 10 到 15。
  * 工程中包含了 Button3 (按键3)，用于触发电机的开关机状态机翻转。
  */
void EXTI15_10_IRQHandler(void)
{
  /* 分发给外部引脚的中断回调 HAL_GPIO_EXTI_Callback */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
  * @brief This function handles COMP1, COMP2 and COMP3 interrupts through EXTI lines 21, 22 and 29.
  * 【硬件底层保护核心】：内部比较器 (COMP) 越限中断。
  * 相电流经过内部运算放大器放大后，通过 MCU 的内部模拟信号矩阵路由至 COMP1，
  * 与内部 DAC3 设定的动态阈值进行比对。一旦相电流异常飙升越过阈值，硬件内部
  * 连线会立刻触发 TIM1 Break 关断 PWM 发波，随后触发此中断通知软件系统。
  */
void COMP1_2_3_IRQHandler(void)
{
  HAL_COMP_IRQHandler(&hcomp1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
