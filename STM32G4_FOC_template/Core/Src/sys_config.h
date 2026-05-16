#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H

#include "main.h"
#include "rtwtypes.h"

/* 系统主时钟配置 (RCC/PLL) */
void SystemClock_Config(void);

/* FDCAN 通讯协议与滤波器配置 */
void FDCAN_Config(void);

/* 导出通讯相关的全局缓冲区，供 APP 层或调试使用 */
extern char RxBuffer[];
extern uint8_t Uart1_Rx_Cnt;

#endif /* SYS_CONFIG_H */
