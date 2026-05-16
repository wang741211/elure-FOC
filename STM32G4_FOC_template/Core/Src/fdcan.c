/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  * of the FDCAN instances. (FDCAN总线配置文件)
  * 核心作用：工业级抗干扰通信，用于上位机调度电机启停和目标转速。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function 
 * FDCAN1 初始化：配置波特率、工作模式及帧格式
 */
void MX_FDCAN1_Init(void)
{
  /* USER CODE BEGIN FDCAN1_Init 0 */
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  
  /* *==============================================================================
   * 位时间 (Bit Timing) 与波特率配置数学模型：
   * 假设 FDCAN 时钟源频率为 f_canclk。
   * 波特率 = f_canclk / (ClockDivider * NominalPrescaler * (NominalSyncJumpWidth + NominalTimeSeg1 + NominalTimeSeg2))
   * 这里的参数决定了 CAN 总线在经典模式下的采样点位置，通常采样点放在 75% ~ 87.5% 处抗干扰能力最强。
   *==============================================================================
   */
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;                /* 兼容传统 CAN 2.0B 格式，不使用 FD 变波特率段 */
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;                         /* 正常收发模式 */
  hfdcan1.Init.AutoRetransmission = ENABLE;                      /* 开启自动重传：当总线受电机强电磁干扰导致发送失败时，硬件自动重发以保证指令必达 */
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  
  /* 标称位时间参数 (Nominal Bit Time) */
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  
  /* 数据段位时间参数 (Data Bit Time，仅在 FD 模式下生效，此处经典模式作为占位配置) */
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  
  hfdcan1.Init.StdFiltersNbr = 0;                                /* 标准帧 ID 滤波器数量 */
  hfdcan1.Init.ExtFiltersNbr = 1;                                /* 扩展帧 ID 滤波器数量 (代码在 main.c 中配置了 Range 滤波) */
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;        /* 发送缓冲区工作在 FIFO 先进先出模式 */
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  /* USER CODE END FDCAN1_Init 2 */
}

/* 底层引脚与时钟配置 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */
  /* USER CODE END FDCAN1_MspInit 0 */
  
    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX (接收引脚)
    PA12     ------> FDCAN1_TX (发送引脚)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;                 /* 复用映射为 FDCAN1 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init
     * 设置接收中断优先级，用于实时响应上位机下发的启停和转速指令
     */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);                 /* 优先级低于核心的 ADC 电流闭环中断，防止通讯阻塞 FOC */
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */
  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */
  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */
  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
