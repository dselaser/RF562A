/**
 * @file    adrc2.h
 * @brief   Second-order Linear Active Disturbance Rejection Controller
 *          (LADRC) for the RF562A VCA position / outer loop.
 *
 * Plant model assumed by the controller:
 *      d2y/dt2 = b0 * u + f(t)
 *   where f(t) absorbs ALL un-modelled dynamics + external disturbance
 *   (friction, gravity, payload, tissue contact force, etc.).
 *
 * For a current-driven VCA position loop:
 *      M * d2x/dt2 = Kt * i - F_disturbance(t)
 *   => b0 ≈ Kt / M,   u = i_ref [A],   y = x [mm or m]
 *      f  = -(F_disturbance + viscous + Coulomb + bias) / M
 *
 * Tuning (Gao 2003, bandwidth parameterization):
 *   - b0 : critical gain. Estimate b0 ≈ Kt/M, then UNDER-ESTIMATE by 30-50%
 *          for robustness. Sign matters (positive current → positive y motion).
 *   - wc : closed-loop bandwidth [rad/s]. Must be ≤ 1/5 of inner-loop wc
 *          for cascade stability.
 *   - wo : observer bandwidth [rad/s]. 3..10 × wc; reduce if measurement
 *          noisy (encoder quantization, ADS reading).
 *
 * Discrete-time realization:
 *   - 3-state ESO (z1≈y, z2≈dy/dt, z3≈f) with triple pole at z=exp(-wo*Ts).
 *   - Current-injection structure (Miklosovic, Radke, Gao 2006).
 *   - Anti-windup: saturated control fed back into the observer.
 *
 * Footprint (float32): ~18 floats per instance, ~40 MAC per Update().
 *
 * References:
 *   [1] Z. Gao, "Scaling and bandwidth-parameterization based controller
 *       tuning," Proc. ACC, 2003.
 *   [2] R. Miklosovic, A. Radke, Z. Gao, "Discrete implementation and
 *       generalization of the extended state observer," Proc. ACC, 2006.
 *   [3] G. Herbst, "A Minimum-Footprint Implementation of Discrete-Time
 *       ADRC," arXiv:2104.01943, 2021.
 */

#ifndef ADRC2_H
#define ADRC2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    /* ===== Tuning parameters (set via ADRC2_Init / ADRC2_Retune) ===== */
    float b0;            /* Critical gain (Kt/M estimate)                 */
    float wc;            /* Controller bandwidth     [rad/s]              */
    float wo;            /* Observer bandwidth       [rad/s]              */
    float Ts;            /* Sample period            [s]                  */
    float u_min;         /* Control output lower saturation (e.g. -I_MAX) */
    float u_max;         /* Control output upper saturation (e.g. +I_MAX) */

    /* ===== Pre-computed discrete coefficients ===== */
    float L1;            /* ESO gain - position state                     */
    float L2;            /* ESO gain - velocity state                     */
    float L3;            /* ESO gain - disturbance state                  */
    float kp;            /* State-feedback proportional gain (= wc^2)     */
    float kd;            /* State-feedback derivative   gain (= 2*wc)     */
    float inv_b0;        /* 1/b0, cached to avoid IRQ-time division       */
    float Ts_half;       /* Ts/2,  cached                                 */
    float Ts_sq_half;    /* Ts*Ts/2, cached                               */

    /* ===== Runtime states ===== */
    float z1;            /* ESO estimate of position y                    */
    float z2;            /* ESO estimate of velocity dy/dt                */
    float z3;            /* ESO estimate of total disturbance f           */
    float u_lim_prev;    /* Previous SATURATED control output             */
} ADRC2_t;


/**
 * @brief  Initialise controller and pre-compute discrete coefficients.
 *         Call once at startup, BEFORE enabling the control loop.
 *
 * @param  a        controller instance
 * @param  b0       critical gain (≈ Kt/M for VCA position loop)
 * @param  wc       controller bandwidth [rad/s] (≤ 1/5 of inner-loop wc)
 * @param  wo       observer bandwidth   [rad/s] (recommend 3..10 * wc)
 * @param  Ts       sample period [s] (e.g. 1e-3 for 1 kHz task)
 * @param  u_min    control lower limit in [A] (e.g. -I_MAX)
 * @param  u_max    control upper limit in [A] (e.g. +I_MAX)
 */
void  ADRC2_Init  (ADRC2_t *a,
                   float b0, float wc, float wo, float Ts,
                   float u_min, float u_max);

/**
 * @brief  Reset observer states (use on enable/disable transitions to
 *         bumplessly resume from current measurement y0).
 *         Velocity & disturbance estimates are zeroed.
 */
void  ADRC2_Reset (ADRC2_t *a, float y0);

/**
 * @brief  One control step.  Call at fixed period Ts from the position
 *         task (typically 1 kHz; the inner current loop runs faster).
 *
 * @param  r   reference position (setpoint) in same units as y
 * @param  y   measured position
 * @return     saturated control output (= current command i_ref, in A)
 */
float ADRC2_Update(ADRC2_t *a, float r, float y);

/**
 * @brief  Recompute coefficients after on-the-fly bandwidth change.
 *         NOT IRQ-safe: call from supervisor task with the control
 *         loop temporarily paused (or use double-buffering).
 */
void  ADRC2_Retune(ADRC2_t *a, float wc, float wo, float b0);

#ifdef __cplusplus
}
#endif

#endif /* ADRC2_H */
