#ifndef MOTOR_MW_FOC_H
#define MOTOR_MW_FOC_H

#include "rtwtypes.h"
#include <math.h>

/* =========================================================================
 * 仅保留：磁链观测器 (反电动势观测器) 结构体及其实例化接口
 * 供后续第三步“观测器独立化”重构时使用
 * ========================================================================= */
typedef struct {
    float Rs;           
    float Ls;           
    float Ts;           
    float inv_Ls;
    float Ts_over_Ls;
    float ki_Ts;
    
    float kp;           
    float ki;           
    
    float i_alpha_est;  
    float i_beta_est;   
    float e_alpha_int;  
    float e_beta_int;   
    
    float e_alpha_est;  
    float e_beta_est;   
} BEMF_Observer_t;

void BEMF_Observer_Init(BEMF_Observer_t *obs, float Rs, float Ls, float kp, float ki, float Ts);
void BEMF_Observer_Step(BEMF_Observer_t *obs, float v_alpha, float v_beta, float i_alpha, float i_beta);

#endif /* MOTOR_MW_FOC_H */
