#include "motor_foc.h"
#include "arm_math.h" /* 使用 CMSIS-DSP 库加速三角函数 */

/* 定义底层物理数学常数 */
#define MC_SQRT3_DIV_3   (0.577350269f)
#define MC_ONE_DIV_3     (0.333333333f)
#define MC_TWO_DIV_3     (0.666666667f)

/**
  * @brief  PI 控制器更新 (带抗积分饱和机制)
  * @note   使用前向欧拉法离散化积分项
  */
static float PI_Update(PI_Controller_t *pi, float ref, float fdb, float Ts)
{
    float error = ref - fdb;
    float output;

    /* 计算包含当前误差的输出值 */
    output = pi->Kp * error + pi->Integral;

    /* 抗积分饱和 (Anti-windup): 仅在未达到饱和边界时进行积分累加 */
    if (output >= pi->OutMax) {
        output = pi->OutMax;
    } else if (output <= pi->OutMin) {
        output = pi->OutMin;
    } else {
        pi->Integral += pi->Ki * error * Ts; 
    }

    return output;
}

void Motor_FOC_Init(MC_Foc_Vars_t *foc, PI_Controller_t *pi_d, PI_Controller_t *pi_q)
{
    /* 硬件安全基准线：上电时必须将 PWM 占空比严格初始化为 0.0f！
     * 坚决摒弃 Simulink 可能产生的 98% 默认高电平，防止启动瞬间的硬件短路或炸管危险。
     */
    foc->Duty_A = 0.0f;
    foc->Duty_B = 0.0f;
    foc->Duty_C = 0.0f;
    
    foc->Id_Ref = 0.0f;
    foc->Iq_Ref = 0.0f;
    
    pi_d->Integral = 0.0f;
    pi_q->Integral = 0.0f;
}

void Motor_FOC_Step(MC_Foc_Vars_t *foc, PI_Controller_t *pi_d, PI_Controller_t *pi_q)
{
    float sin_theta = arm_sin_f32(foc->Theta_Elec);
    float cos_theta = arm_cos_f32(foc->Theta_Elec);
    
    /* 保护母线电压防除零异常 */
    float vbus_safe = (foc->Vbus < 1.0f) ? 1.0f : foc->Vbus;

    /* 1. Clarke 变换：三相静止 (ABC) -> 两相静止 (Alpha-Beta) */
    foc->I_alpha = MC_TWO_DIV_3 * foc->Ia - MC_ONE_DIV_3 * (foc->Ib + foc->Ic);
    foc->I_beta  = MC_SQRT3_DIV_3 * (foc->Ib - foc->Ic);

    /* 2. Park 变换：两相静止 (Alpha-Beta) -> 两相旋转 (D-Q) */
    foc->Id_Fdb = foc->I_alpha * cos_theta + foc->I_beta * sin_theta;
    foc->Iq_Fdb = -foc->I_alpha * sin_theta + foc->I_beta * cos_theta;

    /* 3. 电流环 PI 调节：误差生成目标电压 (这里假设调度周期 Ts = 0.0001s) */
    foc->Vd_Out = PI_Update(pi_d, foc->Id_Ref, foc->Id_Fdb, 0.0001f);
    foc->Vq_Out = PI_Update(pi_q, foc->Iq_Ref, foc->Iq_Fdb, 0.0001f);

    /* 4. 逆 Park 变换：两相旋转 (D-Q) -> 两相静止 (Alpha-Beta) */
    foc->V_alpha = foc->Vd_Out * cos_theta - foc->Vq_Out * sin_theta;
    foc->V_beta  = foc->Vd_Out * sin_theta + foc->Vq_Out * cos_theta;

    /* 5. SVPWM 马鞍波发生器 (零序电压注入法) */
    float va_ref = foc->V_alpha;
    float vb_ref = -0.5f * foc->V_alpha + 0.8660254f * foc->V_beta;
    float vc_ref = -0.5f * foc->V_alpha - 0.8660254f * foc->V_beta;

    /* 提取三相参考电压的最值以计算零序电压分量 */
    float v_max = va_ref;
    if (vb_ref > v_max) v_max = vb_ref;
    if (vc_ref > v_max) v_max = vc_ref;

    float v_min = va_ref;
    if (vb_ref < v_min) v_min = vb_ref;
    if (vc_ref < v_min) v_min = vc_ref;

    float v_zero = -(v_max + v_min) * 0.5f;

    /* 注入零序电压并根据母线电压归一化为占空比 
     * 极性匹配：根据您的定时器中心对齐模式与极性设定进行符号调整
     */
    foc->Duty_A = (-(va_ref + v_zero) / vbus_safe) + 0.5f;
    foc->Duty_B = (-(vb_ref + v_zero) / vbus_safe) + 0.5f;
    foc->Duty_C = (-(vc_ref + v_zero) / vbus_safe) + 0.5f;
}
