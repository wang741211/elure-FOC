/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  * of all used GPIO pins. (基础通用输入输出引脚配置文件)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable (开启全部需用到的端口时钟) */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level 
   * 初始化时将两个 LED 状态指示灯默认拉低 (熄灭状态)
   */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin 
   * 按键引脚配置：挂载到 EXTI (外部中断/事件控制器)
   * 采用 RISING (上升沿触发) 模式，用于在按键按下时立刻翻转电机运行状态机 (main.c 中的 Motor_state)
   */
  GPIO_InitStruct.Pin = Button3_Pin|Button1_Pin|Button2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;                            /* 假设外部电路上已有物理下拉电阻 */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin 
   * LED 指示灯配置：推挽输出模式 (Push-Pull)，提供足够的电流驱动 LED 发光
   */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;                   /* LED 指示无需高速翻转，使用低速可降低 EMI */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init 
   * 注册并使能外部中断，优先级通常设低，不能打断电机正在执行的发波和采样
   */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 2 */
/* USER CODE END 2 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
