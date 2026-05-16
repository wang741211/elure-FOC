/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  * of the ADC instances. (模数转换器配置文件)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function 
 * ADC1 初始化：负责采集 A相电流(注入组JDR1)、C相电流(注入组JDR2) 以及 外部电位器(规则组)
 */
void MX_ADC1_Init(void)
{
  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */
  
  /** Common config (基础公共配置) */
  hadc1.Instance = ADC1;
  /* 时钟分频：选择异步时钟 4 分频，确保 ADC 工作在最佳性能区间 */
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;               /* 12位分辨率 (0~4095) */
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;               /* 数据右对齐 */
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;                /* 开启扫描模式，允许一次触发扫描多个通道 */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;                  /* 关闭连续转换，采用单次触发 */
  hadc1.Init.NbrOfConversion = 1;                           /* 规则组通道数量为 1 (电位器) */
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;         /* 规则组由软件(main循环里)手动触发 */
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;              /* 数据溢出时保留旧数据 */
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure the ADC multi-mode (配置多模式：此处为独立模式，不与ADC2做主从交替等复杂联动) */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Regular Channel (配置规则组通道：用于低频模拟量如电位器给定) */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;          /* 采样时间设置为 2.5 个周期，快速采样 */
  sConfig.SingleDiff = ADC_SINGLE_ENDED;                    /* 单端输入 */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;                   /* 不使用硬件偏移，在代码中通过软件进行偏置消除 */
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** * Configure Injected Channel (配置注入组通道 1：采集 A 相电流) 
   * 注入组具有高优先级，配合硬件定时器可以精准捕获无噪时刻的电流
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;       /* 排名第一，数据存入 JDR1 */
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5; /* 增加采样时间至 6.5 周期，保证内部电容充分充电，提高精度 */
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;              /* 注入组共包含 2 个通道 (A相和C相) */
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  /* ！！！核心触发机制：注入组由 TIM1 通道4 的比较事件硬件触发 ！！！ */
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING; /* 上升沿触发 */
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Injected Channel (配置注入组通道 2：采集 C 相电流) */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;       /* 排名第二，数据存入 JDR2 */
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}

/* ADC2 init function 
 * ADC2 初始化：专门负责采集 B 相电流(注入组JDR1)，以及母线电压(规则组)
 */
void MX_ADC2_Init(void)
{
  /* USER CODE BEGIN ADC2_Init 0 */
  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */
  /* USER CODE END ADC2_Init 1 */
  
  /** Common config */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;               /* ADC2 的注入组只有1个通道，关闭扫描模式 */
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;                           /* 规则组通道数量为 1 (母线电压) */
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;         /* 规则组同样由软件手动触发 */
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Injected Channel (配置注入组通道 1：采集 B 相电流) */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;       /* 排名第一，数据存入 JDR1 */
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;              /* 仅采集 1 个通道 */
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  /* ！！！同样由 TIM1 通道4 硬件触发，确保与 ADC1 的电流采样绝对同步 ！！！ */
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Regular Channel (配置规则组通道：采集直流母线电压 Vbus) */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  /* USER CODE END ADC2_Init 2 */
}

/* 记录 ADC 时钟是否被启用的静态计数器 */
static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

/* 底层 MCU 引脚与中断初始化 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */
  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /**ADC1 GPIO Configuration
    PA2      ------> ADC1_IN3  (经过内部/外部放大的相电流信号 A相)
    PB1      ------> ADC1_IN12 (经过内部/外部放大的相电流信号 C相)
    PB12     ------> ADC1_IN11 (电位器等辅助输入)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 配置中断：将 ADC1 和 ADC2 的共享中断挂载并开启。
     * 当配置的注入通道转换完成后，该中断将唤醒 FOC_Model_step()
     */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);                 /* 设置为最高优先级(0)，确保电机控制的实时性不受其他低优先级任务干扰 */
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */
  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */
  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /**ADC2 GPIO Configuration
    PA0      ------> ADC2_IN1 (母线电压检测)
    PA6      ------> ADC2_IN3 (经过内部/外部放大的相电流信号 B相)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);                 /* 同样配置最高优先级 */
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */
  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */
  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration*/
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1|GPIO_PIN_12);

    /* ADC1 interrupt Deinit */
    /* 注释掉了对共享中断的强制关闭，以防影响 ADC2 */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE BEGIN ADC1_MspDeInit 1 */
  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */
  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration*/
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_6);

    /* ADC2 interrupt Deinit */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE BEGIN ADC2_MspDeInit 1 */
  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
