/**
  ******************************************************************************
  * @file    dac.c
  * @brief   This file provides code for the configuration
  * of the DAC instances. (数模转换器配置文件)
  * 核心作用：生成硬件过流保护的动态阈值参考电压，以及输出模拟调试波形。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dac.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac3;

/* DAC1 init function 
 * DAC1 初始化：主要用于对外输出模拟电压 (外部引脚 PA4)
 * 在电机控制工程中，DAC1 通常被用作“硬件级虚拟示波器”，
 * 可以在 FOC_Model_step() 算法中将内部的观测器角度、交直轴电流等变量，
 * 实时映射为 0~3.3V 的模拟电压输出到 PA4，用实体示波器抓取无延迟的纯净波形。
 */
void MX_DAC1_Init(void)
{
  /* USER CODE BEGIN DAC1_Init 0 */
  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */
  /* USER CODE END DAC1_Init 1 */
  
  /** DAC Initialization */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** DAC channel OUT1 config (DAC1 通道1 配置) */
  /* 自动配置高频接口模式，保证数据写入寄存器到模拟电压输出的极低延迟 */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;                      /* 使用无符号格式 (0~4095) */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;   /* 禁用采样保持 (用于低功耗模式，电机控制不需要) */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;                  /* 软件触发转换，直接写入寄存器即刻生效 */
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;      /* 开启输出缓冲 (Buffer)，增强引脚对外部负载的驱动能力 */
  
  /* ！！！重要：连接到外部物理引脚 PA4 ！！！ */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;         /* 使用出厂校准值消除失调误差 */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  /* USER CODE END DAC1_Init 2 */
}

/* DAC3 init function 
 * DAC3 初始化：内部动态阈值发生器 (保护核心)
 * 生成的电压不输出到外部，专门在 MCU 内部模拟矩阵中路由给 COMP1 的反相输入端。
 */
void MX_DAC3_Init(void)
{
  /* USER CODE BEGIN DAC3_Init 0 */
  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */
  /* USER CODE END DAC3_Init 1 */
  
  /** DAC Initialization */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** DAC channel OUT1 config (DAC3 通道1 配置) */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;     /* 关闭输出缓冲：因为信号只在芯片内部传输，关闭 Buffer 可以消除其带来的微小偏置误差和功耗 */
  
  /* *=================================================================================
   * ！！！核心模拟矩阵配置：DAC_CHIPCONNECT_INTERNAL ！！！
   * 物理意义：将 DAC3 的输出与外部引脚完全断开，专供内部外设（如比较器、运放）使用。
   * 这就是前面 main.c 中设定 3000 (约 2.41V) 时，直接形成 COMP1 翻转基准的底层原因。
   * ==================================================================================
   */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */
  /* USER CODE END DAC3_Init 2 */
}

/* 底层 MCU 引脚与时钟初始化 */
void HAL_DAC_MspInit(DAC_HandleTypeDef* dacHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(dacHandle->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspInit 0 */
  /* USER CODE END DAC1_MspInit 0 */
    /* 开启 DAC1 时钟 */
    __HAL_RCC_DAC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1 (外部模拟信号输出测试引脚)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC1_MspInit 1 */
  /* USER CODE END DAC1_MspInit 1 */
  }
  else if(dacHandle->Instance==DAC3)
  {
  /* USER CODE BEGIN DAC3_MspInit 0 */
  /* USER CODE END DAC3_MspInit 0 */
    /* 开启 DAC3 时钟 
     * 注意：由于 DAC3 配置为内部连接，不需要初始化任何 GPIO 引脚，
     * 避免了对宝贵外部 IO 资源的占用。
     */
    __HAL_RCC_DAC3_CLK_ENABLE();
  /* USER CODE BEGIN DAC3_MspInit 1 */
  /* USER CODE END DAC3_MspInit 1 */
  }
}

/* DAC 反初始化，用于关闭外设与释放资源 */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* dacHandle)
{
  if(dacHandle->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspDeInit 0 */
  /* USER CODE END DAC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC1_CLK_DISABLE();

    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

  /* USER CODE BEGIN DAC1_MspDeInit 1 */
  /* USER CODE END DAC1_MspDeInit 1 */
  }
  else if(dacHandle->Instance==DAC3)
  {
  /* USER CODE BEGIN DAC3_MspDeInit 0 */
  /* USER CODE END DAC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC3_CLK_DISABLE();
  /* USER CODE BEGIN DAC3_MspDeInit 1 */
  /* USER CODE END DAC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
