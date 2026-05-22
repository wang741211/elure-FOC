#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include <stdint.h>
#include <math.h>

/* 标准化 PI 控制器结构体 */
typedef struct {
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */
    float Integral;     /* 历史积分累计值 */
    float OutMax;       /* 输出抗饱和上限 */
    float OutMin;       /* 输出抗饱和下限 */
} PI_Controller_t;

/* 纯粹的 FOC 物理变量集合 (高内聚) */
typedef struct {
    /* 1. 输入物理量 */
    float Ia, Ib, Ic;   /* 采样的三相相电流 (A) */
    float Vbus;         /* 实时母线电压 (V) */
    float Theta_Elec;   /* 当前转子电角度 (rad, 0~2PI) */
    
    /* 2. 目标给定值 */
    float Id_Ref;       /* D轴目标励磁电流 (A) */
    float Iq_Ref;       /* Q轴目标转矩电流 (A) */
    
    /* 3. 坐标变换中间量 */
    float I_alpha, I_beta;
    float Id_Fdb, Iq_Fdb;
    float Vd_Out, Vq_Out;
    float V_alpha, V_beta;
    
    /* 4. 最终输出量 */
    float Duty_A;       /* A相占空比 (0.0f ~ 1.0f) */
    float Duty_B;       /* B相占空比 (0.0f ~ 1.0f) */
    float Duty_C;       /* C相占空比 (0.0f ~ 1.0f) */
} MC_Foc_Vars_t;

/* FOC 运算初始化 */
void Motor_FOC_Init(MC_Foc_Vars_t *foc, PI_Controller_t *pi_d, PI_Controller_t *pi_q);

/* FOC 核心步进函数 (10kHz调用) */
void Motor_FOC_Step(MC_Foc_Vars_t *foc, PI_Controller_t *pi_d, PI_Controller_t *pi_q);

#endif

