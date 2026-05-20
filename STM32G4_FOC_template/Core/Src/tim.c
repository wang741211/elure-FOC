/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  * of the TIM instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* TIM1 init function 
 * 高级控制定时器 TIM1 初始化
 * 作用：产生三相互补带死区的 SVPWM 波形，并在波形顶点触发 ADC 采样，同时接收比较器过流保护信号。
 */
void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;                                      /* 不分频，使用最高系统主频 (170MHz) */
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;         /* 采用中心对齐模式 1 (三角波计数)，保障电流波形对称 */
  htim1.Init.Period = 8499;                                        /* 载波周期计数值，决定了 PWM 的基准频率  */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;               /* 死区时间发生器的时钟分频 */
  htim1.Init.RepetitionCounter = 1;                                /* 重复计数器：设为 1 表示每经历 1 个完整上下溢出周期产生一次更新事件 */
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;   /* 禁用自动重装载预装载 */
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* *=============================================================================
   * 主从模式配置：将 CH4 的比较匹配 (OC4REF) 作为触发源 (TRGO)
   * 物理意义：ADC 将在这个 TRGO 信号的边缘瞬间进行注入组电流采样。
   * ==============================================================================
   */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* *=============================================================================
   * 刹车/断路保护 (Break) 输入配置 
   * 信号来源配置为：TIM_BREAKINPUTSOURCE_COMP1 (内部 MCU 模拟矩阵)
   * 取代了低效的外部参考电平比较方式，实现了基于内部运放和比较器的微秒级硬件级闭环保护。
   * ==============================================================================
   */
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH; /* 高电平触发刹车 */
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 动力输出通道 CH1, CH2, CH3 配置 (SVPWM 驱动上桥臂，互补驱动下桥臂) */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                                             /* 初始占空比为 0 */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;                      /* 正极性 */
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;                    /* 互补引脚极性也为正 */
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;                   /* 空闲时拉低 */
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;                 /* 互补端空闲时拉低 */
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* *=============================================================================
   * CH4 配置：充当触发 ADC 的独立发生器
   * 模式：PWM2。比较值：7998 (靠近 7999 顶点)。
   * 效果：在三角载波顶部产生一个短脉冲，无偏差地在此处触发 ADC 进行静音无噪采样。
   * ==============================================================================
   */
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 8498;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 死区时间与刹车控制寄存器配置 */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 120;                             /* 插入死区时间，防止同臂上下功率管直通短路炸管 */
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;              /* 开启主刹车功能 */
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 2;                            /* 插入数字滤波器抗干扰，防止误触发 */
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;           /* 禁用副刹车 */
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/* TIM4 init function 
 * 通用定时器 TIM4 初始化：专用霍尔传感器(Hall)测速与角度捕获接口
 */
void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  
  htim4.Instance = TIM4;
  /* 预分频器配置为 50-1，针对约 160MHz 总线，最终提供 3.2MHz 的计数频率 (匹配 main.c 中的 3200000 计数值) */
  htim4.Init.Prescaler = 50-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;                                       /* 最大捕获计数值 */
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 开启定时器专用硬件霍尔接口模式
   * 该模式下 CH1, CH2, CH3 输入信号经底层异或(XOR)，共同触发输入捕获复位机制
   */
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;                     /* 上升沿捕获 */
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;                                          /* 设置最大滤波常数，消除霍尔毛刺干扰 */
  sConfig.Commutation_Delay = 0;                                   /* 本工程直接在捕获中断处理角度，无需硬件延迟换相延时 */
  if (HAL_TIMEx_HallSensor_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
}

/* 底层 MCU 硬件级别的初始化，由 HAL_TIM_Base_Init 自动调用 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */
  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* 开启 TIM1 对应的中断：刹车中断、溢出更新中断、捕获比较中断 */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */
  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */
  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1 (霍尔 U/A 相)
    PB7     ------> TIM4_CH2 (霍尔 V/B 相)
    PB8-BOOT0     ------> TIM4_CH3 (霍尔 W/C 相)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */
  /* USER CODE END TIM4_MspInit 1 */
  }
}

/* 动力输出引脚配置 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */
  /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PB13     ------> TIM1_CH1N (下桥臂 U/A相)
    PB14     ------> TIM1_CH2N (下桥臂 V/B相)
    PB15     ------> TIM1_CH3N (下桥臂 W/C相)
    PA8      ------> TIM1_CH1  (上桥臂 U/A相)
    PA9      ------> TIM1_CH2  (上桥臂 V/B相)
    PA10     ------> TIM1_CH3  (上桥臂 W/C相)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */
  /* USER CODE END TIM1_MspPostInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */
  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */
  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */
  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    PB8-BOOT0     ------> TIM4_CH3
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8);

    /* TIM4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspDeInit 1 */
  /* USER CODE END TIM4_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
