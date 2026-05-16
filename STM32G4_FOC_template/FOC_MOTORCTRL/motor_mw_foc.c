#include "motor_mw_foc.h"
#include <math.h>

/* 内部数学常量定义 */
#define SQRT3_DIV3    0.577350269f  /* sqrt(3)/3 */
#define ONE_DIV3      0.333333333f  /* 1/3 */
#define TWO_DIV3      0.666666667f  /* 2/3 */

/* 实例化电流环 PI 控制器 */
static PI_Controller_t pi_id;
static PI_Controller_t pi_iq;

/**
  * @brief  PI 控制器更新函数 (带抗积分饱和)
  * 公式：u(k) = Kp * e(k) + Ki * sum(e(k) * Ts)
  */
static real32_T PI_Update(PI_Controller_t *pi, real32_T reference, real32_T feedback)
{
    real32_T error = reference - feedback;
    real32_T output;

    /* 比例项 */
    output = pi->Kp * error + pi->Integral;

    /* 积分项更新 (前向欧拉法) */
    pi->Integral += pi->Ki * error * pi->Ts;

    /* 抗积分饱和 (Output Saturation & Clamping) */
    if (output > pi->OutMax) {
        output = pi->OutMax;
        pi->Integral -= pi->Ki * error * pi->Ts; // 停止积分
    } else if (output < pi->OutMin) {
        output = pi->OutMin;
        pi->Integral -= pi->Ki * error * pi->Ts; // 停止积分
    }

    return output;
}

/**
  * @brief  初始化 FOC 算法参数
  */
void Motor_MW_FOC_Init(void)
{
    /* 初始化 D 轴电流环 */
    pi_id.Kp = 0.26f;
    pi_id.Ki = 35.0f;
    pi_id.Ts = 0.0001f; // 10kHz
    pi_id.OutMax = 12.47f;
    pi_id.OutMin = -12.47f;
    pi_id.Integral = 0.0f;

    /* 初始化 Q 轴电流环 */
    pi_iq.Kp = 0.26f;
    pi_iq.Ki = 35.0f;
    pi_iq.Ts = 0.0001f;
    pi_iq.OutMax = 12.47f;
    pi_iq.OutMin = -12.47f;
    pi_iq.Integral = 0.0f;
}

/**
  * @brief  FOC 核心步进算法
  */
void Motor_MW_FOC_Step(const FOC_Input_t *in, FOC_Output_t *out)
{
    real32_T sin_theta = sinf(in->Theta_Elec);
    real32_T cos_theta = cosf(in->Theta_Elec);

    /* * 1. Clarke 变换 (等幅值变换)
     * I_alpha = 2/3 * Ia - 1/3 * (Ib + Ic)
     * I_beta  = sqrt(3)/3 * (Ib - Ic)
     */
    real32_T i_alpha = TWO_DIV3 * in->Ia - ONE_DIV3 * (in->Ib + in->Ic);
    real32_T i_beta  = SQRT3_DIV3 * (in->Ib - in->Ic);

    /* * 2. Park 变换 (旋转变换)
     * Id = I_alpha * cos + I_beta * sin
     * Iq = -I_alpha * sin + I_beta * cos
     */
    real32_T id = i_alpha * cos_theta + i_beta * sin_theta;
    real32_T iq = -i_alpha * sin_theta + i_beta * cos_theta;

    /* * 3. 电流环 PI 控制
     * 得到 Vd 和 Vq 控制电压
     */
    out->Vd = PI_Update(&pi_id, in->Id_Ref, id);
    out->Vq = PI_Update(&pi_iq, in->Iq_Ref, iq);

    /* * 4. 逆 Park 变换 (Inverse Park)
     * V_alpha = Vd * cos - Vq * sin
     * V_beta  = Vd * sin + Vq * cos
     */
    real32_T v_alpha = out->Vd * cos_theta - out->Vq * sin_theta;
    real32_T v_beta  = out->Vd * sin_theta + out->Vq * cos_theta;

    /* * 5. SVPWM 实现 (零序电压注入/马鞍波生成)
     * 先计算三相参考电压
     */
    real32_T va_ref = v_alpha;
    real32_T vb_ref = -0.5f * v_alpha + 0.8660254f * v_beta; // 0.8660254 = sqrt(3)/2
    real32_T vc_ref = -0.5f * v_alpha - 0.8660254f * v_beta;

    /* 计算零序电压：Vzero = -(max(Va,Vb,Vc) + min(Va,Vb,Vc)) / 2 */
    real32_T v_max = va_ref;
    if (vb_ref > v_max) v_max = vb_ref;
    if (vc_ref > v_max) v_max = vc_ref;

    real32_T v_min = va_ref;
    if (vb_ref < v_min) v_min = vb_ref;
    if (vc_ref < v_min) v_min = vc_ref;

    real32_T v_zero = (v_max + v_min) * -0.5f;

    /* 注入零序并归一化到 0.0 ~ 1.0 占空比 
     * 公式：Duty = (V_ref + V_zero) / Vbus + 0.5
     * 这里的系数乘法与您的硬件极性对应
     */
    out->Duty_A = (- (va_ref + v_zero) / in->Vbus) + 0.5f;
    out->Duty_B = (- (vb_ref + v_zero) / in->Vbus) + 0.5f;
    out->Duty_C = (- (vc_ref + v_zero) / in->Vbus) + 0.5f;
}

