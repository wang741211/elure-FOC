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
    real32_T V_alpha;          /* 定子电压 alpha (V) */
    real32_T V_beta;           /* 定子电压 beta (V) */
} FOC_Output_t;

// 定义磁链观测器(反电动势观测器)结构体
typedef struct {
    // === 1. 物理参数与常数 (初始化时配置) ===
    float Rs;           // 定子电阻 (Ohm)
    float Ls;           // 定子电感 (H)
    float Ts;           // 控制周期，例如 0.0001s (100us)
    float inv_Ls;
    float Ts_over_Ls;
    float ki_Ts;
    
    // === 2. 观测器 PI 控制器参数 ===
    float kp;           // 比例增益：决定观测器追踪真实电流的响应速度
    float ki;           // 积分增益：消除稳态误差
    
    // === 3. 内部状态变量 (需要记忆的历史状态) ===
    float i_alpha_est;  // 虚拟电机估算的上一拍电流 alpha轴
    float i_beta_est;   // 虚拟电机估算的上一拍电流 beta轴
    float e_alpha_int;  // PI控制器积分项的历史累加值 alpha轴
    float e_beta_int;   // PI控制器积分项的历史累加值 beta轴
    
    // === 4. 输出结果 ===
    float e_alpha_est;  // 最终观测出的估算反电动势 alpha轴
    float e_beta_est;   // 最终观测出的估算反电动势 beta轴
} BEMF_Observer_t;


/* 接口函数：初始化算法状态与 PI 参数 */
void Motor_MW_FOC_Init(void);

/* 接口函数：执行一步核心 FOC 计算 */
void Motor_MW_FOC_Step(const FOC_Input_t *in, FOC_Output_t *out);

void BEMF_Observer_Init(BEMF_Observer_t *obs, float Rs, float Ls, float kp, float ki, float Ts);
void BEMF_Observer_Step(BEMF_Observer_t *obs, float v_alpha, float v_beta, float i_alpha, float i_beta);



#endif /* MOTOR_MW_FOC_H */

 
