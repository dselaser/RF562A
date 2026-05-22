/**
 * @file    adrc2.c
 * @brief   Second-order Linear ADRC - implementation
 */

#include "adrc2.h"
#include <math.h>


/* ---------------------------------------------------------------------
 * Discrete ESO coefficient design (current-injection / ZOH equivalent).
 *
 * Continuous 3-state LESO for d2y/dt2 = b0*u + f, triple pole at -wo:
 *     A_eso = [[-l1, 1, 0],
 *              [-l2, 0, 1],
 *              [-l3, 0, 0]],
 *     l1 = 3*wo,  l2 = 3*wo^2,  l3 = wo^3.
 *
 * Discretised with ZOH and the "current injection" structure
 * (Miklosovic, Radke, Gao 2006). Placing all three observer poles at
 * z = beta = exp(-wo*Ts) yields:
 *     L1 = 1 - beta^3
 *     L2 = (3/(2*Ts)) * (1 - beta)^2 * (1 + beta)
 *     L3 = (1/Ts^2) * (1 - beta)^3
 *
 * Numerically well-conditioned for any wo*Ts, unlike forward-Euler
 * with naive continuous gains.
 *
 * State feedback (double closed-loop pole at -wc):
 *     kp = wc^2,  kd = 2*wc
 *     u0 = kp*(r - z1) - kd*z2
 *     u  = (u0 - z3) / b0
 * --------------------------------------------------------------------- */
static void compute_coeffs(ADRC2_t *a)
{
    const float beta   = expf(-a->wo * a->Ts);
    const float omb    = 1.0f - beta;
    const float omb_sq = omb * omb;
    const float Ts     = a->Ts;

    a->L1         = 1.0f - beta * beta * beta;
    a->L2         = 1.5f * omb_sq * (1.0f + beta) / Ts;
    a->L3         = omb_sq * omb / (Ts * Ts);
    a->kp         = a->wc * a->wc;
    a->kd         = 2.0f * a->wc;
    a->inv_b0     = (a->b0 != 0.0f) ? (1.0f / a->b0) : 0.0f;
    a->Ts_half    = 0.5f * Ts;
    a->Ts_sq_half = 0.5f * Ts * Ts;
}


void ADRC2_Init(ADRC2_t *a,
                float b0, float wc, float wo, float Ts,
                float u_min, float u_max)
{
    a->b0    = b0;
    a->wc    = wc;
    a->wo    = wo;
    a->Ts    = Ts;
    a->u_min = u_min;
    a->u_max = u_max;
    compute_coeffs(a);
    ADRC2_Reset(a, 0.0f);
}


void ADRC2_Reset(ADRC2_t *a, float y0)
{
    a->z1         = y0;
    a->z2         = 0.0f;
    a->z3         = 0.0f;
    a->u_lim_prev = 0.0f;
}


void ADRC2_Retune(ADRC2_t *a, float wc, float wo, float b0)
{
    a->wc = wc;
    a->wo = wo;
    a->b0 = b0;
    compute_coeffs(a);
}


float ADRC2_Update(ADRC2_t *a, float r, float y)
{
    /* ---- 1. ESO predict (using previously SATURATED control) -------
     *      eff_u  = z3 + b0 * u_lim_prev
     *      z1_p   = z1 + Ts*z2 + (Ts^2/2) * eff_u
     *      z2_p   = z2 + Ts * eff_u
     *      z3_p   = z3
     *
     * Anti-windup: u_lim_prev (post-saturation) means the observer sees
     * what the actuator actually applied, so z3 never integrates a
     * phantom disturbance during saturation.
     * ---------------------------------------------------------------- */
    const float eff_u  = a->z3 + a->b0 * a->u_lim_prev;
    const float z1_pred = a->z1 + a->Ts * a->z2 + a->Ts_sq_half * eff_u;
    const float z2_pred = a->z2 + a->Ts * eff_u;
    const float err     = y - z1_pred;

    /* ---- 2. ESO correct with fresh measurement y ------------------- */
    a->z1 = z1_pred + a->L1 * err;
    a->z2 = z2_pred + a->L2 * err;
    a->z3 = a->z3   + a->L3 * err;

    /* ---- 3. State-error feedback + disturbance compensation -------- */
    const float u0 = a->kp * (r - a->z1) - a->kd * a->z2;
    float u = (u0 - a->z3) * a->inv_b0;

    /* ---- 4. Saturate ----------------------------------------------- */
    if      (u > a->u_max) u = a->u_max;
    else if (u < a->u_min) u = a->u_min;

    a->u_lim_prev = u;
    return u;
}
