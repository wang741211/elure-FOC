#include "motor_sensor.h"
#include "motor_mw_foc.h" /* 包含刚才修剪过的 BEMF 结构体 */
#include "motor_pll.h"    /* 包含原有的 PLL 锁相环 */

/* =========================================================================
 * 内部静态私有变量 (对外不可见，严格封装)
 * ========================================================================= */
BEMF_Observer_t bemf_obs;
 SRF_PLL_t       bemf_pll;
static float sensorless_theta_offset = -1.5707963f; /* -PI/2 的相移补偿 */

/* =========================================================================
 * BEMF 观测器实现方法 (面向对象思想的私有方法)
 * ========================================================================= */

/**
  * @brief 初始化无感观测器与锁相环
  */
static void BEMF_Wrapper_Init(void) {
    /* 初始化反电动势观测器参数 (Rs, Ls, kp, ki, Ts)
     * Rs = 0.295 Ohm (相电阻)
     * Ls = 0.00033 H (相电感)
     * Kp = 0.5, Ki = 450.0 (基于 1500rad/s 带宽设计)
     * Ts = 0.0001s (10kHz 控制频率)
     */
    BEMF_Observer_Init(&bemf_obs, 0.295f, 0.00033f, 0.5f, 450.0f, 0.0001f);
    
    /* 锁相环参数保持不动，重点先让前端的 BEMF 稳定 */
    SRF_PLL_Init(&bemf_pll, 200.0f, 20000.0f, 0.0001f);
    SRF_PLL_Reset(&bemf_pll, -sensorless_theta_offset, 0.0f);
    
    Sensor_BEMF.is_ready = 0; 
}

/**
  * @brief 观测器步进运算，每拍 ADC 中断调用
  */
static void BEMF_Wrapper_Update(float v_alpha, float v_beta, float i_alpha, float i_beta) {
    /* 1. 通过电压方程估算反电动势 (e_alpha, e_beta) */
    BEMF_Observer_Step(&bemf_obs, v_alpha, v_beta, i_alpha, i_beta);
    
    /* 2. 将估算出的反电动势送入 PLL 提取角度和速度 */
    SRF_PLL_Step(&bemf_pll, bemf_obs.e_alpha_est, bemf_obs.e_beta_est);
    
    /* 如果 PLL 输出异常 (发散)，则触发硬件复位保护逻辑 */
    if (!isfinite(bemf_pll.Theta) || !isfinite(bemf_pll.Omega)) {
        SRF_PLL_Reset(&bemf_pll, 0.0f, 0.0f); 
    }
}

/**
  * @brief 获取滤波补偿后的实时电角度
  */
static float BEMF_Wrapper_GetTheta(void) {
    /* 补偿物理相移，并做 0~2Pi 的循环限幅 */
    float theta_elec = bemf_pll.Theta + sensorless_theta_offset;
    if (theta_elec < 0.0f) {
        theta_elec += 6.283185f;
    } else if (theta_elec >= 6.283185f) {
        theta_elec -= 6.283185f;
    }
    return theta_elec;
}

/**
  * @brief 获取估算电角速度
  */
static float BEMF_Wrapper_GetSpeed(void) {
    return bemf_pll.Omega;
}

/* =========================================================================
 * 暴露给外部的实体对象 (多态接口)
 * 其他文件只需要外部声明 extern MC_Position_Sensor_t Sensor_BEMF; 即可调用
 * ========================================================================= */
MC_Position_Sensor_t Sensor_BEMF = {
    .Init     = BEMF_Wrapper_Init,
    .Update   = BEMF_Wrapper_Update,
    .GetTheta = BEMF_Wrapper_GetTheta,
    .GetSpeed = BEMF_Wrapper_GetSpeed,
    .is_ready = 0
};
