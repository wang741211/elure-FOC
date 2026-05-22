#include "motor_mw_foc.h"

/* =========================================================================
 * 仅保留：反电动势观测器的具体实现逻辑
 * ========================================================================= */
void BEMF_Observer_Init(BEMF_Observer_t *obs, float Rs, float Ls, float kp, float ki, float Ts)
{
    obs->Rs = Rs;
    obs->Ls = Ls;
    obs->kp = kp;
    obs->ki = ki;
    obs->Ts = Ts;
    obs->inv_Ls = (Ls > 0.0f) ? (1.0f / Ls) : 0.0f;
    obs->Ts_over_Ls = Ts * obs->inv_Ls;
    obs->ki_Ts = ki * Ts;

    obs->i_alpha_est = 0.0f;
    obs->i_beta_est = 0.0f;
    obs->e_alpha_int = 0.0f;
    obs->e_beta_int = 0.0f;
    obs->e_alpha_est = 0.0f;
    obs->e_beta_est = 0.0f;
}

void BEMF_Observer_Step(BEMF_Observer_t *obs, float v_alpha, float v_beta, float i_alpha, float i_beta)
{
    if (!isfinite(v_alpha) || !isfinite(v_beta) || !isfinite(i_alpha) || !isfinite(i_beta) ||
        !isfinite(obs->i_alpha_est) || !isfinite(obs->i_beta_est) ||
        !isfinite(obs->e_alpha_int) || !isfinite(obs->e_beta_int)) {
        obs->i_alpha_est = 0.0f;
        obs->i_beta_est = 0.0f;
        obs->e_alpha_int = 0.0f;
        obs->e_beta_int = 0.0f;
        obs->e_alpha_est = 0.0f;
        obs->e_beta_est = 0.0f;
        return;
    }

		float i_alpha_err = obs->i_alpha_est - i_alpha; 
		float i_beta_err  = obs->i_beta_est - i_beta;


    obs->e_alpha_int += obs->ki_Ts * i_alpha_err;
    obs->e_beta_int += obs->ki_Ts * i_beta_err;

    obs->e_alpha_est = obs->kp * i_alpha_err + obs->e_alpha_int;
    obs->e_beta_est = obs->kp * i_beta_err + obs->e_beta_int;

    obs->i_alpha_est += (v_alpha - obs->Rs * obs->i_alpha_est - obs->e_alpha_est) * obs->Ts_over_Ls;
    obs->i_beta_est += (v_beta - obs->Rs * obs->i_beta_est - obs->e_beta_est) * obs->Ts_over_Ls;
}
