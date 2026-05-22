/**
 * @file    adrc1.c
 * @brief   First-order Linear ADRC - implementation
 */

#include "adrc1.h"
#include <math.h>


/* ---------------------------------------------------------------------
 * Discrete ESO coefficient design (current-observer / ZOH equivalent).
 *
 * Continuous LESO with double pole at -wo:
 *     A_eso = [[-l1, 1],[-l2, 0]],  l1 = 2*wo,  l2 = wo^2
 *
 * Discretised with ZOH and "current injection" structure
 * (Miklosovic, Radke, Gao 2006), the pole at z = beta = exp(-wo*Ts)
 * is achieved by:
 *     L1 = 1 - beta^2
 *     L2 = (1 - beta)^2 / Ts
 *
 * This is numerically well-conditioned for ANY wo*Ts (no instability
 * from oversized continuous gains), unlike naive forward-Euler.
 * --------------------------------------------------------------------- */
static void compute_coeffs(ADRC1_t *a)
{
    const float beta = expf(-a->wo * a->Ts);
    a->L1     = 1.0f - beta * beta;
    a->L2     = (1.0f - beta) * (1.0f - beta) / a->Ts;
    a->kp     = a->wc;
    a->inv_b0 = (a->b0 != 0.0f) ? (1.0f / a->b0) : 0.0f;
}


void ADRC1_Init(ADRC1_t *a,
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
    ADRC1_Reset(a, 0.0f);
}


void ADRC1_Reset(ADRC1_t *a, float y0)
{
    a->z1          = y0;
    a->z2          = 0.0f;
    a->u_lim_prev  = 0.0f;
}


void ADRC1_Retune(ADRC1_t *a, float wc, float wo, float b0)
{
    a->wc = wc;
    a->wo = wo;
    a->b0 = b0;
    compute_coeffs(a);
}


float ADRC1_Update(ADRC1_t *a, float r, float y)
{
    /* ---- 1. ESO update (predict + current-injection correct) -------
     *  Predict:
     *      z1_p = z1 + Ts * ( z2 + b0 * u_lim_prev )
     *      z2_p = z2
     *  Correct with the FRESH measurement y:
     *      z1   = z1_p + L1 * (y - z1_p)
     *      z2   = z2_p + L2 * (y - z1_p)
     *
     *  Anti-windup: u_lim_prev is the saturated value of the previous
     *  step; feeding it back means the observer "knows" what the
     *  actuator actually applied, so z2 never integrates a phantom
     *  disturbance during saturation.
     * ---------------------------------------------------------------- */
    const float z1_pred = a->z1 + a->Ts * (a->z2 + a->b0 * a->u_lim_prev);
    const float err     = y - z1_pred;

    a->z1 = z1_pred + a->L1 * err;
    a->z2 = a->z2   + a->L2 * err;

    /* ---- 2. Linear state-error feedback + disturbance compensation -
     *      u0 = kp * ( r - z1 )                    (proportional law)
     *      u  = ( u0 - z2 ) / b0                   (cancel total disturbance)
     * ---------------------------------------------------------------- */
    float u = (a->kp * (r - a->z1) - a->z2) * a->inv_b0;

    /* ---- 3. Saturate -------------------------------------------- */
    if      (u > a->u_max) u = a->u_max;
    else if (u < a->u_min) u = a->u_min;

    a->u_lim_prev = u;
    return u;
}
