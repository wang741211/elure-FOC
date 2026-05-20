/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  * of the ADC instances. (ģ��ת���������ļ�)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function 
 * ADC1 ��ʼ��������ɼ� A�����(ע����JDR1)��C�����(ע����JDR2) �Լ� �ⲿ��λ��(������)
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
  
  /** Common config (������������) */
  hadc1.Instance = ADC1;
  /* ʱ�ӷ�Ƶ��ѡ���첽ʱ�� 4 ��Ƶ��ȷ�� ADC ����������������� */
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;               /* 12λ�ֱ��� (0~4095) */
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;               /* �����Ҷ��� */
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;                /* ����ɨ��ģʽ������һ�δ���ɨ����ͨ�� */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;                  /* �ر�����ת�������õ��δ��� */
  hadc1.Init.NbrOfConversion = 1;                           /* ������ͨ������Ϊ 1 (��λ��) */
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;         /* ������������(mainѭ����)�ֶ����� */
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;              /* �������ʱ���������� */
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure the ADC multi-mode (���ö�ģʽ���˴�Ϊ����ģʽ������ADC2�����ӽ���ȸ�������) */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Regular Channel (���ù�����ͨ�������ڵ�Ƶģ�������λ������) */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;          /* ����ʱ������Ϊ 2.5 �����ڣ����ٲ��� */
  sConfig.SingleDiff = ADC_SINGLE_ENDED;                    /* �������� */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;                   /* ��ʹ��Ӳ��ƫ�ƣ��ڴ�����ͨ����������ƫ������ */
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** * Configure Injected Channel (����ע����ͨ�� 1���ɼ� A �����) 
   * ע������и����ȼ������Ӳ����ʱ�����Ծ�׼��������ʱ�̵ĵ���
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;       /* ������һ�����ݴ��� JDR1 */
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5; /* ���Ӳ���ʱ���� 6.5 ���ڣ���֤�ڲ����ݳ�ֳ�磬��߾��� */
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;              /* ע���鹲���� 2 ��ͨ�� (A���C��) */
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  /* ���������Ĵ������ƣ�ע������ TIM1 ͨ��4 �ıȽ��¼�Ӳ������ ������ */
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING; /* �����ش��� */
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Injected Channel (����ע����ͨ�� 2���ɼ� C �����) */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;       /* �����ڶ������ݴ��� JDR2 */
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}

/* ADC2 init function 
 * ADC2 ��ʼ����ר�Ÿ���ɼ� B �����(ע����JDR1)���Լ�ĸ�ߵ�ѹ(������)
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
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;               /* ADC2 ��ע����ֻ��1��ͨ�����ر�ɨ��ģʽ */
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;                           /* ������ͨ������Ϊ 1 (ĸ�ߵ�ѹ) */
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;         /* ������ͬ���������ֶ����� */
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Injected Channel (����ע����ͨ�� 1���ɼ� B �����) */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;       /* ������һ�����ݴ��� JDR1 */
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;              /* ���ɼ� 1 ��ͨ�� */
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  /* ������ͬ���� TIM1 ͨ��4 Ӳ��������ȷ���� ADC1 �ĵ�����������ͬ�� ������ */
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Regular Channel (���ù�����ͨ�����ɼ�ֱ��ĸ�ߵ�ѹ Vbus) */
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

/* ��¼ ADC ʱ���Ƿ����õľ�̬������ */
static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

/* �ײ� MCU �������жϳ�ʼ�� */
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
    PA2      ------> ADC1_IN3  (�����ڲ�/�ⲿ�Ŵ��������ź� A��)
    PB1      ------> ADC1_IN12 (�����ڲ�/�ⲿ�Ŵ��������ź� C��)
    PB12     ------> ADC1_IN11 (��λ���ȸ�������)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* �����жϣ��� ADC1 �� ADC2 �Ĺ����жϹ��ز�������
     * �����õ�ע��ͨ��ת����ɺ󣬸��жϽ����� FOC_Model_step()
     */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);                 /* ����Ϊ������ȼ�(0)��ȷ��������Ƶ�ʵʱ�Բ������������ȼ�������� */
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
    PA0      ------> ADC2_IN1 (ĸ�ߵ�ѹ���)
    PA6      ------> ADC2_IN3 (�����ڲ�/�ⲿ�Ŵ��������ź� B��)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);                 /* ͬ������������ȼ� */
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
    /* ע�͵��˶Թ����жϵ�ǿ�ƹرգ��Է�Ӱ�� ADC2 */
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
