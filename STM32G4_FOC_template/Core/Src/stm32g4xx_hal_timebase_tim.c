/*
 * ---------------------------------------------------------------------
 * 标准工程实践：HAL 库时基重定向 (Timebase Source Redirect)
 * 作用：拦截 HAL_Init() 内部的时基初始化过程，强行将 1ms 心跳分配给 TIM6。
 * ---------------------------------------------------------------------
 */
#include "stm32g4xx_hal.h"

/* 声明 TIM6 的句柄，后续中断服务函数也会用到它 */
TIM_HandleTypeDef htim6;

/**
  * @brief  初始化 HAL 库的时基定时器 (覆盖 HAL 库内部的弱函数)
  * @param  TickPriority: 滴答定时器的中断优先级
  * @retval HAL 状态
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;

  /* 1. 开启 TIM6 的硬件时钟 */
  __HAL_RCC_TIM6_CLK_ENABLE();

  /* 2. 获取当前系统的时钟配置，以计算准确的频率，防止硬编码导致移植出错 */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* 3. 动态获取 TIM6 的时钟源频率
   * 编码标准：STM32G4 的 TIM6 挂载在 APB1 总线上。
   * 根据硬件手册：如果 APB1 预分频器为 1，定时器时钟等于 APB1 时钟；
   * 否则等于 APB1 时钟的 2 倍。
   * (在 170MHz 满频运行时，这里的 uwTimclock 最终计算结果为 170,000,000 Hz)
   */
  uwTimclock = HAL_RCC_GetPCLK1Freq();
  if (clkconfig.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    uwTimclock *= 2;
  }

  /* 4. 计算预分频值 (Prescaler) 
   * 思路：将定时器的核心计数频率降维至 1MHz (即 1微秒递增一次)。
   * 公式：(输入频率 / 1000000) - 1
   */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* 5. 配置 TIM6 的基础参数 */
  htim6.Instance = TIM6;
  htim6.Init.Period = (1000U - 1U);          /* 重装载值：计数 1000 次，刚好是 1毫秒 */
  htim6.Init.Prescaler = uwPrescalerValue;
  htim6.Init.ClockDivision = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if(HAL_TIM_Base_Init(&htim6) == HAL_OK)
  {
    /* 6. 开启中断并设置优先级 
     * 架构技巧：HAL 库时基的中断优先级通常设为最高 (0)。
     * 这样可以保证你在其他高优先级外设中断里调用带超时的 HAL 库函数时，
     * TIM6 依然能抢占并更新时间，防止系统死锁。
     */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* 7. 启动定时器并使能更新中断 */
    return HAL_TIM_Base_Start_IT(&htim6);
  }

  return HAL_ERROR;
}

/**
  * @brief  挂起 HAL 库时基 (进入低功耗模式前调用)
  */
void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
  * @brief  恢复 HAL 库时基 (退出低功耗模式后调用)
  */
void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}
