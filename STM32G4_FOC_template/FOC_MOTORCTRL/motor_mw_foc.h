#ifndef MOTOR_MW_FOC_H
#define MOTOR_MW_FOC_H

#include "rtwtypes.h"

/* * PI 控制器结构体
 * 包含比例、积分增益以及抗积分饱和（Anti-windup）所需的限幅值
 */
typedef struct {
    real32_T Kp;               /* 比例增益 */
    real32_T Ki;               /* 积分增益 */
    real32_T Integral;         /* 积分累加器 */
    real32_T OutMin;           /* 输出下限 */
    real32_T OutMax;           /* 输出上限 */
    real32_T Ts;               /* 采样周期 (s) */
} PI_Controller_t;

/* * FOC 算法输入结构体
 * 所有变量均为标准物理量
 */
typedef struct {
    real32_T Ia, Ib, Ic;       /* 反馈三相电流 (A) */
    real32_T Vbus;             /* 实时母线电压 (V) */
    real32_T Theta_Elec;       /* 当前电角度 (0 ~ 2Pi rad) */
    real32_T Id_Ref;           /* D 轴目标电流 (A) */
    real32_T Iq_Ref;           /* Q 轴目标电流 (A) */
} FOC_Input_t;

/* * FOC 算法输出结构体 */
typedef struct {
    real32_T Duty_A;           /* A 相标准化占空比 (0.0 ~ 1.0) */
    real32_T Duty_B;           /* B 相标准化占空比 (0.0 ~ 1.0) */
    real32_T Duty_C;           /* C 相标准化占空比 (0.0 ~ 1.0) */
    real32_T Vd;               /* D 轴控制电压 (V) */
    real32_T Vq;               /* Q 轴控制电压 (V) */
} FOC_Output_t;

/* 接口函数：初始化算法状态与 PI 参数 */
void Motor_MW_FOC_Init(void);

/* 接口函数：执行一步核心 FOC 计算 */
void Motor_MW_FOC_Step(const FOC_Input_t *in, FOC_Output_t *out);

#endif /* MOTOR_MW_FOC_H */

