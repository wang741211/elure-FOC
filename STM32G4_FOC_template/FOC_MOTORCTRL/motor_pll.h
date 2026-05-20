#ifndef MOTOR_PLL_H
#define MOTOR_PLL_H

#include "rtwtypes.h"

typedef struct {
    real32_T Kp;
    real32_T Ki;
    real32_T Ts;

    real32_T Theta;
    real32_T Sin;
    real32_T Cos;
    real32_T Omega;
    real32_T Integral;

    real32_T OmegaMin;
    real32_T OmegaMax;
} SRF_PLL_t;

void SRF_PLL_Init(SRF_PLL_t *pll, real32_T kp, real32_T ki, real32_T Ts);
void SRF_PLL_Reset(SRF_PLL_t *pll, real32_T theta, real32_T omega);
void SRF_PLL_Step(SRF_PLL_t *pll, real32_T x_alpha, real32_T x_beta);

#endif /* MOTOR_PLL_H */

