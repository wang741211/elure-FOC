/**
  ******************************************************************************
  * @file    opamp.c
  * @brief   This file provides code for the configuration
  * of the OPAMP instances. (内置运算放大器配置文件)
  * 核心作用：模拟前端 (AFE)，将微弱的相电流分流电阻压降放大并偏置，
  * 随后在 MCU 内部路由给 ADC(用于闭环控制) 和 COMP(用于过流保护)。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "opamp.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

/* OPAMP1 init function 
 * OPAMP1 初始化：负责处理 A 相电流 (Ia)
 */
void MX_OPAMP1_Init(void)
{
  /* USER CODE BEGIN OPAMP1_Init 0 */
  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */
  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  
  /* * 供电与模式配置
   * 工作在正常模式 (Normal Mode)，以获取最大的带宽和压摆率 (Slew Rate)，
   * 这对于捕捉高频 PWM 开关瞬间的电流变化至关重要。
   */
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  
  /* * 内部/外部增益配置
   * 配置为独立模式 (Standalone Mode)。
   * 物理意义：MCU 内部的 PGA (可编程增益) 电阻网络未启用，
   * 放大倍数完全由外部硬件 PCB 上连接到 VINM (PA3) 和 VOUT (PA2) 之间的反馈电阻决定。
   */
  hopamp1.Init.Mode = OPAMP_STANDALONE_MODE;
  
  /* 反相输入端 (VINM) 配置：连接到外部引脚 PA3 */
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  
  /* 正相输入端 (VINP) 配置：连接到外部引脚 PA1 (接入分流电阻的正端) */
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  
  /* 内部定时器控制的多路复用模式禁用 */
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  
  /* 偏置修整 (Trimming)：使用出厂校准值消除运放自身的失调电压 (Offset Voltage) */
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */
  /* USER CODE END OPAMP1_Init 2 */
}

/* OPAMP2 init function 
 * OPAMP2 初始化：负责处理 B 相电流 (Ib)
 */
void MX_OPAMP2_Init(void)
{
  /* USER CODE BEGIN OPAMP2_Init 0 */
  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */
  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp2.Init.Mode = OPAMP_STANDALONE_MODE;
  
  /* 反相输入端 (VINM)：外部引脚 PA5 */
  hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  
  /* 正相输入端 (VINP)：外部引脚 PA7 */
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */
  /* USER CODE END OPAMP2_Init 2 */
}

/* OPAMP3 init function 
 * OPAMP3 初始化：负责处理 C 相电流 (Ic)
 */
void MX_OPAMP3_Init(void)
{
  /* USER CODE BEGIN OPAMP3_Init 0 */
  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */
  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp3.Init.Mode = OPAMP_STANDALONE_MODE;
  
  /* 反相输入端与正相输入端根据具体硬件管脚映射 (常见为 PB0, PB2 等) */
  hopamp3.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  
  hopamp3.Init.InternalOutput = DISABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */
  /* USER CODE END OPAMP3_Init 2 */
}

/* 底层 MCU 引脚与模拟外设时钟初始化 */
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* opampHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(opampHandle->Instance==OPAMP1)
  {
  /* USER CODE BEGIN OPAMP1_MspInit 0 */
  /* USER CODE END OPAMP1_MspInit 0 */
  
    /* 使能 GPIOA 和 OPAMP 通用系统时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    
    /**OPAMP1 GPIO Configuration
    PA1     ------> OPAMP1_VINP (A相电流正相输入)
    PA2     ------> OPAMP1_VOUT (A相电流放大后输出，内部同时连至 ADC1_IN3)
    PA3     ------> OPAMP1_VINM (A相电流反相输入，接外部反馈电阻网络)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;            /* 全部设置为纯模拟模式，关闭数字输入缓冲防止漏电 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP1_MspInit 1 */
  /* USER CODE END OPAMP1_MspInit 1 */
  }
  else if(opampHandle->Instance==OPAMP2)
  {
  /* USER CODE BEGIN OPAMP2_MspInit 0 */
  /* USER CODE END OPAMP2_MspInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /**OPAMP2 GPIO Configuration
    PA5     ------> OPAMP2_VINM (B相电流反相输入)
    PA6     ------> OPAMP2_VOUT (B相电流放大后输出，内部同时连至 ADC2_IN3)
    PA7     ------> OPAMP2_VINP (B相电流正相输入)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP2_MspInit 1 */
  /* USER CODE END OPAMP2_MspInit 1 */
  }
  else if(opampHandle->Instance==OPAMP3)
  {
  /* USER CODE BEGIN OPAMP3_MspInit 0 */
  /* USER CODE END OPAMP3_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /**OPAMP3 GPIO Configuration (假设映射)
    PB0     ------> OPAMP3_VINP (C相电流正相输入)
    PB1     ------> OPAMP3_VOUT (C相电流放大后输出)
    PB2     ------> OPAMP3_VINM (C相电流反相输入)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP3_MspInit 1 */
  /* USER CODE END OPAMP3_MspInit 1 */
  }
}

void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* opampHandle)
{
  if(opampHandle->Instance==OPAMP1)
  {
  /* USER CODE BEGIN OPAMP1_MspDeInit 0 */
  /* USER CODE END OPAMP1_MspDeInit 0 */

    /**OPAMP1 GPIO Configuration */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN OPAMP1_MspDeInit 1 */
  /* USER CODE END OPAMP1_MspDeInit 1 */
  }
  else if(opampHandle->Instance==OPAMP2)
  {
  /* USER CODE BEGIN OPAMP2_MspDeInit 0 */
  /* USER CODE END OPAMP2_MspDeInit 0 */

    /**OPAMP2 GPIO Configuration */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN OPAMP2_MspDeInit 1 */
  /* USER CODE END OPAMP2_MspDeInit 1 */
  }
  else if(opampHandle->Instance==OPAMP3)
  {
  /* USER CODE BEGIN OPAMP3_MspDeInit 0 */
  /* USER CODE END OPAMP3_MspDeInit 0 */

    /**OPAMP3 GPIO Configuration */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);

  /* USER CODE BEGIN OPAMP3_MspDeInit 1 */
  /* USER CODE END OPAMP3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
