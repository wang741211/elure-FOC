#include "sys_config.h"
#include "fdcan.h"
#include "usart.h"
#include "motor_app_fsm.h"
#include <string.h>

FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8] = {0};
uint8_t TxData[8] = {0};



/* ---------------- 串口接收私有变量 ---------------- */
#define RXBUFFERSIZE  256   
char RxBuffer[RXBUFFERSIZE];  
uint8_t aRxBuffer;             
uint8_t Uart1_Rx_Cnt = 0;    

/**
  * @brief  系统时钟配置 (170MHz - 基于 STM32G4 高性能配置)
  * 物理意义：配置核心 PLL 乘法器与分频器，为 FOC 算法提供确定的计算时基。
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* 配置电压调节级别以支持高速主频 */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    
    /* 1. 配置外部高速晶振 (HSE) 并开启 PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 2. 配置 AHB/APB 总线分频 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 3. 配置外设专用时钟 (ADC, FDCAN, UART) */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  FDCAN 滤波器与中断通知配置
  */
void FDCAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    extern FDCAN_HandleTypeDef hfdcan1;

    /* 激活 FIFO 接收新消息中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    
    /* 配置接收滤波器：接收所有 ID 范围内的扩展帧消息 */
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;  
    sFilterConfig.FilterIndex = 0;             
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;  
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x01ffffff;       
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    
    /* 预初始化发送帧头信息 */
    TxHeader.Identifier = 0x1B;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0x52;

    /* 启动 CAN 总线外设 */
    HAL_FDCAN_Start(&hfdcan1);
}

/**
  * @brief  串口接收完成中断回调 (低频事件)
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        if(Uart1_Rx_Cnt >= 255) {
            Uart1_Rx_Cnt = 0;
            memset(RxBuffer, 0x00, 256);
        } else {
            RxBuffer[Uart1_Rx_Cnt++] = aRxBuffer;
        }
        /* 重新启动接收中断 */
        HAL_UART_Receive_IT(huart, (uint8_t *)&aRxBuffer, 1);
    }
}

/**
  * @brief  外部按键中断回调 (逻辑解耦：仅调用 APP 层接口)
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* 只处理开机/停机按钮 */
    if(Button3_Pin == GPIO_Pin)
    {
        /* 这里的逻辑已经通过 APP 层接口完全抽象化 */
        if (Motor_APP_GetState() == APP_STATE_IDLE) {
            Motor_APP_Start();
        } else {
            Motor_APP_Stop();
        }
    }
}
