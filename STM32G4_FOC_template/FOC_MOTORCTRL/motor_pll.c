#include "motor_pll.h"
#include "arm_math.h"
#include <math.h>
#include <stdint.h>

#define TWO_PI 6.283185307179586f

static real32_T Wrap_0_2pi(real32_T theta)
{
    if (theta >= TWO_PI) theta -= TWO_PI;
    else if (theta < 0.0f) theta += TWO_PI;
    return theta;
}

static real32_T FastInvSqrt(real32_T x)
{
    union {
        real32_T f;
        uint32_t u;
    } v;
    v.f = x;
    v.u = 0x5f3759dfu - (v.u >> 1);
    real32_T y = v.f;
    y = y * (1.5f - 0.5f * x * y * y);
    return y;
}

void SRF_PLL_Init(SRF_PLL_t *pll, real32_T kp, real32_T ki, real32_T Ts)
{
    pll->Kp = kp;
    pll->Ki = ki;
    pll->Ts = Ts;

    pll->Theta = 0.0f;
    pll->Sin = 0.0f;
    pll->Cos = 1.0f;
    pll->Omega = 0.0f;
    pll->Integral = 0.0f;

    pll->OmegaMin = -20000.0f;
    pll->OmegaMax = 20000.0f;
}

void SRF_PLL_Reset(SRF_PLL_t *pll, real32_T theta, real32_T omega)
{
    pll->Theta = Wrap_0_2pi(theta);
    pll->Sin = arm_sin_f32(pll->Theta);
    pll->Cos = arm_cos_f32(pll->Theta);
    pll->Omega = omega;
    pll->Integral = 0.0f;
}

void SRF_PLL_Step(SRF_PLL_t *pll, real32_T x_alpha, real32_T x_beta)
{
    if (!isfinite(pll->Theta) || !isfinite(pll->Omega) || !isfinite(pll->Integral) ||
        !isfinite(pll->Sin) || !isfinite(pll->Cos)) {
        pll->Theta = 0.0f;
        pll->Omega = 0.0f;
        pll->Integral = 0.0f;
        pll->Sin = 0.0f;
        pll->Cos = 1.0f;
    }

    if (!isfinite(x_alpha) || !isfinite(x_beta)) {
        return;
    }

    real32_T x_d = x_alpha * pll->Cos + x_beta * pll->Sin;
    real32_T x_q = -x_alpha * pll->Sin + x_beta * pll->Cos;

    real32_T amp2 = x_d * x_d + x_q * x_q;
    real32_T inv_amp = (isfinite(amp2) && amp2 > 1e-12f) ? FastInvSqrt(amp2) : 0.0f;
    real32_T err = (inv_amp > 0.0f && isfinite(x_q)) ? (x_q * inv_amp) : 0.0f;

    pll->Integral += pll->Ki * err * pll->Ts;
    if (pll->Integral > pll->OmegaMax) pll->Integral = pll->OmegaMax;
    else if (pll->Integral < pll->OmegaMin) pll->Integral = pll->OmegaMin;

    pll->Omega = pll->Kp * err + pll->Integral;

    if (pll->Omega > pll->OmegaMax) pll->Omega = pll->OmegaMax;
    else if (pll->Omega < pll->OmegaMin) pll->Omega = pll->OmegaMin;

    real32_T delta = pll->Omega * pll->Ts;
    pll->Theta = pll->Theta + delta;

    if (pll->Theta >= TWO_PI || pll->Theta < 0.0f) {
        pll->Theta = Wrap_0_2pi(pll->Theta);
        pll->Sin = arm_sin_f32(pll->Theta);
        pll->Cos = arm_cos_f32(pll->Theta);
        return;
    }

    if (fabsf(delta) > 0.3f) {
        pll->Sin = arm_sin_f32(pll->Theta);
        pll->Cos = arm_cos_f32(pll->Theta);
        return;
    }

    real32_T d2 = delta * delta;
    real32_T sin_d = delta * (1.0f - d2 * (1.0f / 6.0f));
    real32_T cos_d = 1.0f - 0.5f * d2;

    real32_T sin_new = pll->Sin * cos_d + pll->Cos * sin_d;
    real32_T cos_new = pll->Cos * cos_d - pll->Sin * sin_d;

    real32_T sc2 = sin_new * sin_new + cos_new * cos_new;
    if (!isfinite(sc2)) {
        pll->Sin = arm_sin_f32(pll->Theta);
        pll->Cos = arm_cos_f32(pll->Theta);
        return;
    }

    real32_T k = 1.5f - 0.5f * sc2;
    pll->Sin = sin_new * k;
    pll->Cos = cos_new * k;
}

