#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>
#include "rtwtypes.h"

/* * 硬件抽象层初始化
 * 包含时钟、外设（ADC, PWM, OPAMP, COMP, DAC）的底层初始化调用
 */
void Motor_HAL_Init(void);

/* * 功率级使能与封锁
 * 对应开启/关闭定时器的 PWM 输出通道
 */
void Motor_HAL_EnablePWM(void);
void Motor_HAL_DisablePWM(void);

/* * 物理电流获取接口 (单位: A)
 * 屏蔽 ADC 原始数值、偏置校准 (Offset) 和硬件增益系数
 */
void Motor_HAL_GetPhaseCurrents(real32_T *ia, real32_T *ib, real32_T *ic);

/* * 母线电压获取接口 (单位: V)
 * 屏蔽分压电阻比例和 ADC 转换逻辑
 */
real32_T Motor_HAL_GetBusVoltage(void);

/* * PWM 占空比下发接口
 * @param dutyA/B/C: 标准化占空比，范围 0.0f ~ 1.0f
 * 屏蔽定时器计数值 (ARR) 和中心对齐模式的具体寄存器映射
 */
void Motor_HAL_SetPWM(real32_T dutyA, real32_T dutyB, real32_T dutyC);

/* * 零电流偏移校准
 * 在电机静止时调用，用于记录运放的静态中点电压
 */
void Motor_HAL_CalibrateOffsets(void);

#endif /* MOTOR_HAL_H */

