/**
  ******************************************************************************
  * @file    comp.c
  * @brief   This file provides code for the configuration
  * of the COMP instances. (模拟比较器配置文件)
  * 核心作用：实现微秒级的硬件过流断路保护 (Hardware OCP)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "comp.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

COMP_HandleTypeDef hcomp1;

/* COMP1 init function 
 * COMP1 初始化：过流保护阈值比较器
 */
void MX_COMP1_Init(void)
{
  /* USER CODE BEGIN COMP1_Init 0 */
  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */
  /* USER CODE END COMP1_Init 1 */
  
  hcomp1.Instance = COMP1;
  
  /* * 1. 正相输入端配置 (实际电流信号) 
   * 配置为 IO2，对应物理引脚 PB1。
   * 这里接入的是经过下桥臂采样电阻并可能经过外部运放放大后的电压信号，代表相电流大小。
   */
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  
  /* * 2. 反相输入端配置 (保护阈值参考)
   * 内部连接到 DAC3 的通道 1 输出。
   * 优势：不需要外部提供基准电压，软件可根据工况随时改变过流保护的动作点。
   */
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
  
  /* * 3. 比较器输出极性与特性配置 
   * NONINVERTED (同相)：当 V+ (实际电流) > V- (DAC阈值) 时，输出高电平。
   * 这个高电平在芯片内部被直接布线到了 TIM1 的 Break (刹车) 输入端。
   */
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;       /* 不使用滞回，追求最快的越限响应 */
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;    /* 不使用消隐窗口 (Blanking Window) */
  
  /* * 4. 中断触发模式配置 
   * RISING (上升沿触发)：不仅在硬件底层封锁 PWM，还要产生一次 NVIC 中断。
   * CPU 在主循环或控制算法中被中断唤醒，进而执行停机报错、记录故障代码等善后逻辑。
   */
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */
  /* USER CODE END COMP1_Init 2 */
}

/* 底层 MCU 引脚与中断初始化 */
void HAL_COMP_MspInit(COMP_HandleTypeDef* compHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(compHandle->Instance==COMP1)
  {
  /* USER CODE BEGIN COMP1_MspInit 0 */
  /* USER CODE END COMP1_MspInit 0 */

    /* 开启 GPIOB 时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /**COMP1 GPIO Configuration
    PB1     ------> COMP1_INP (比较器正相输入端，接相电流模拟量)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;           /* 模拟模式，断开数字输入施密特触发器以降低功耗 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;                /* 模拟引脚不需要上下拉 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* COMP1 interrupt Init 
     * 设置比较器越限(过流)中断优先级，并使能
     * 注意：由于过流是极其严重的故障，通常这个优先级会被设置得很高。
     */
    HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 0, 0);        /* 极高优先级 (0) */
    HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
  /* USER CODE BEGIN COMP1_MspInit 1 */
  /* USER CODE END COMP1_MspInit 1 */
  }
}

/* 比较器反初始化：重置引脚状态并关闭中断 */
void HAL_COMP_MspDeInit(COMP_HandleTypeDef* compHandle)
{
  if(compHandle->Instance==COMP1)
  {
  /* USER CODE BEGIN COMP1_MspDeInit 0 */
  /* USER CODE END COMP1_MspDeInit 0 */

    /**COMP1 GPIO Configuration
    PB1     ------> COMP1_INP
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    /* COMP1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(COMP1_2_3_IRQn);
  /* USER CODE BEGIN COMP1_MspDeInit 1 */
  /* USER CODE END COMP1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
