/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  * of all the requested memory to memory DMA transfers.
  * 核心作用：初始化底层 DMA 时钟与 DMAMUX 路由矩阵，并配置全局中断。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  * 初始化 DMA 控制器及其关联的路由矩阵
  */
void MX_DMA_Init(void)
{
  /* * =========================================================================
   * 1. 开启 DMAMUX1 时钟：STM32G4 的特色路由矩阵。
   * 它负责将外设 (如 USART3_TX) 的握手请求灵活重定向到任意物理 DMA 通道。
   * 2. 开启 DMA1 时钟：真实负责在 SRAM (内存) 和 APB/AHB (外设) 总线间搬运数据的硅核。
   * =========================================================================
   */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* * =========================================================================
   * DMA interrupt init (DMA 中断初始化)
   * DMA1_Channel1_IRQn interrupt configuration
   * 配置 DMA1 通道1 的全局中断。本工程中该通道已被分配给 USART3_TX (虚拟示波器数据泵)。
   * 此优先级配置为最高 (0, 0)，确保 DMA 搬运完成的瞬间能立即释放忙碌标志 (Busy Flag)，
   * 避免高频 FOC (10kHz) 在下一拍请求发送时发生堵塞。
   * =========================================================================
   */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/* USER CODE BEGIN 2 */
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
