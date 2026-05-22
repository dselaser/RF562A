/**
 * @file    adrc1.h
 * @brief   First-order Linear Active Disturbance Rejection Controller (LADRC)
 *          - Target: RF562 VCA current loop (STM32F446/F407, HAL + FreeRTOS)
 *
 * Plant model assumed by the controller:
 *      dy/dt = b0 * u + f(t)
 *   where f(t) absorbs ALL un-modelled dynamics + external disturbance.
 *
 * For a VCA current loop:
 *      di/dt = (1/L)*u - (R/L)*i - (Ke/L)*v
 *   => b0 ≈ 1/L,
 *      f  = -(R/L)*i - (Ke/L)*v + parameter drift + temperature effects
 *
 * Tuning (Gao 2003, bandwidth parameterization):
 *   - b0 : critical gain. Estimate from open-loop step,
 *          then UNDER-ESTIMATE by 30-50% for robustness.
 *   - wc : closed-loop bandwidth [rad/s]. Start at the existing PI cross-over.
 *   - wo : observer bandwidth    [rad/s]. 3~10 * wc; lower if measurement noisy.
 *
 * Discrete-time realization:
 *   - ESO poles placed at z = exp(-wo*Ts) (double pole, ZOH-equivalent).
 *   - Anti-windup: saturated control fed back into the observer.
 *
 * Footprint (float32): ~14 floats per instance, ~25 MAC per Update().
 *
 * References:
 *   [1] Z. Gao, "Scaling and bandwidth-parameterization based controller
 *       tuning," Proc. ACC, 2003.
 *   [2] R. Miklosovic, A. Radke, Z. Gao, "Discrete implementation and
 *       generalization of the extended state observer," Proc. ACC, 2006.
 *   [3] G. Herbst, "A simulative study on ADRC robustness," arXiv:1908.04596.
 *   [4] G. Herbst, "A Minimum-Footprint Implementation of Discrete-Time
 *       ADRC," arXiv:2104.01943, 2021.
 */

#ifndef ADRC1_H
#define ADRC1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    /* ===== Tuning parameters (set via ADRC1_Init / ADRC1_Retune) ===== */
    float b0;            /* Critical gain (plant input gain estimate)     */
    float wc;            /* Controller bandwidth        [rad/s]           */
    float wo;            /* Observer bandwidth          [rad/s]           */
    float Ts;            /* Sample period               [s]               */
    float u_min;         /* Control output lower saturation               */
    float u_max;         /* Control output upper saturation               */

    /* ===== Pre-computed discrete coefficients ===== */
    float L1;            /* ESO innovation gain - output state            */
    float L2;            /* ESO innovation gain - disturbance state       */
    float kp;            /* Controller proportional gain ( = wc )         */
    float inv_b0;        /* 1/b0, cached to avoid IRQ-time division       */

    /* ===== Runtime states ===== */
    float z1;            /* ESO estimate of output y                      */
    float z2;            /* ESO estimate of total disturbance f           */
    float u_lim_prev;    /* Previous SATURATED control output             */
} ADRC1_t;


/**
 * @brief  Initialise controller and pre-compute discrete coefficients.
 *         Call once at startup, BEFORE enabling the control interrupt.
 *
 * @param  a        controller instance
 * @param  b0       critical gain (≈ 1/L for current loop)
 * @param  wc       controller bandwidth [rad/s]
 * @param  wo       observer bandwidth   [rad/s]  (recommend 3..10 * wc)
 * @param  Ts       sample period [s]
 * @param  u_min    control lower limit (e.g. -1.0 for signed PWM duty)
 * @param  u_max    control upper limit (e.g. +1.0)
 */
void  ADRC1_Init  (ADRC1_t *a,
                   float b0, float wc, float wo, float Ts,
                   float u_min, float u_max);

/**
 * @brief  Reset observer states (use on enable/disable transitions to
 *         bumplessly resume from current measurement y0).
 */
void  ADRC1_Reset (ADRC1_t *a, float y0);

/**
 * @brief  One control step.  Call at fixed period Ts from the
 *         current-loop ISR (NOT from a normal FreeRTOS task at >1 kHz).
 *
 * @param  r   reference (setpoint) in same units as y
 * @param  y   measured output
 * @return     saturated control output
 */
float ADRC1_Update(ADRC1_t *a, float r, float y);

/**
 * @brief  Recompute coefficients after on-the-fly bandwidth change.
 *         NOT IRQ-safe: call from supervisor task with the control
 *         interrupt temporarily disabled (or use double-buffering).
 */
void  ADRC1_Retune(ADRC1_t *a, float wc, float wo, float b0);

#ifdef __cplusplus
}
#endif

#endif /* ADRC1_H */
