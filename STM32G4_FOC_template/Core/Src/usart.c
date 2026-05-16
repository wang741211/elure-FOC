/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  * of the USART instances. (串口配置文件)
  * 核心作用：配置上位机通讯与高速 DMA 数据泵（虚拟示波器数据通道）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;    /* 串口发送 DMA 句柄，用于高速非阻塞发数 */

/* USART3 init function 
 * USART3 初始化：主要用于 FOC 高频数据的实时上报与上位机指令接收
 */
void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  
  huart3.Instance = USART3;
  /* * 高速波特率配置：921600 bps
   * 这是为了尽量匹配 FOC 的高频执行周期，提高波形采样的还原度。
   */
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;           /* 8位数据位 */
  huart3.Init.StopBits = UART_STOPBITS_1;                /* 1位停止位 */
  huart3.Init.Parity = UART_PARITY_NONE;                 /* 无校验 */
  huart3.Init.Mode = UART_MODE_TX_RX;                    /* 收发双工模式 */
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;           /* 不使用硬件流控 (RTS/CTS) */
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;       /* 16倍过采样，增强抗噪声能力 */
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;      /* 串口时钟不分频 */
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 禁用与配置 FIFO，STM32G4 带有硬件 FIFO 机制以防数据溢出 */
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */
}

/* 串口外设的底层硬件配置 (GPIO、时钟与 DMA 绑定) */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */
  /* USER CODE END USART3_MspInit 0 */
  
    /* 使能 USART3 的外设时钟 */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX (发送引脚)
    PB11     ------> USART3_RX (接收引脚)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;              /* 复用推挽输出，确保高速信号质量 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;         /* 对于 1Mbps 以下的串口，LOW Speed 足够且 EMI 辐射小 */
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;         /* 引脚复用映射至 USART3 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* *==============================================================================
     * USART3 TX 的 DMA 配置通道 (极其关键)
     * ===============================================================================
     */
    hdma_usart3_tx.Instance = DMA1_Channel1;                       /* 挂载在 DMA1 的通道1 */
    hdma_usart3_tx.Init.Request = DMA_REQUEST_USART3_TX;           /* DMA 触发源为 USART3 的发送寄存器空中断 */
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;          /* 数据流向：内存 (SRAM) -> 外设 (USART TDR 寄存器) */
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;              /* 外设地址不递增 (永远指向同一个 TDR 发送寄存器) */
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;                  /* 内存地址递增 (按顺序读取待发送的数组) */
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; /* 外设数据位宽：单字节 (8 bit) */
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据位宽：单字节 (8 bit) */
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;                         /* 正常单次模式 (发完一帧就停止，等下一次软件触发) */
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;               /* DMA 优先级设置较低，把总线带宽让给内核和 FOC 运算 */
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将初始化好的 DMA 句柄与 USART3 的发送句柄链接起来 */
    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* 配置串口自身的全局中断，主要用于处理接收 (RXNE) 和错误 */
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);             /* 优先级低于 ADC 和 定时器，确保通信不影响控制 */
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */
  /* USER CODE END USART3_MspInit 1 */
  }
}

/* 串口外设反初始化 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */
  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */
  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
