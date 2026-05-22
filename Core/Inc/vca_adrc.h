/**
 * @file    vca_adrc.h
 * @brief   RF562A VCA cascaded ADRC controller (scaffold)
 *
 *   Outer loop (position, slow ~ 1 kHz)
 *      r_pos -> ADRC2 -> i_ref [A]
 *   Inner loop (current, fast 5 kHz - TIM6 ISR)
 *      i_ref -> ADRC1 -> u_signed in [-1, +1] -> TLE9201
 *
 * Cascade stability rule:
 *      VCADRC_POS_WC_HZ <= VCADRC_CUR_WC_HZ / 5
 *
 * ALL TUNING CONSTANTS BELOW ARE PLACEHOLDERS.
 * They MUST be replaced with measured values (Phase 1 system ID) before
 * the loops are enabled on hardware. See docs/ADRC_BRINGUP.md.
 */

#ifndef VCA_ADRC_H
#define VCA_ADRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "adrc1.h"
#include "adrc2.h"


/* ============================================================
 *  Compile-time switch: select control law
 *     0 = legacy PI/PID (default - existing behaviour preserved)
 *     1 = ADRC cascade (new)
 *  Define from build options (-DUSE_ADRC=1) or change here.
 * ============================================================ */
#ifndef USE_ADRC
#define USE_ADRC                0
#endif


/* ============================================================
 *  Loop rates  (must match the hardware schedulers actually used)
 * ============================================================ */
#define VCADRC_CUR_RATE_HZ      5000.0f             /* TIM6 ISR        */
#define VCADRC_CUR_TS_S         (1.0f / VCADRC_CUR_RATE_HZ)

#define VCADRC_POS_RATE_HZ      1000.0f             /* FreeRTOS task   */
#define VCADRC_POS_TS_S         (1.0f / VCADRC_POS_RATE_HZ)


/* ============================================================
 *  PLACEHOLDER plant parameters - REPLACE after Phase-1 system ID
 *  (open-loop step on a single needle/coil, measured di/dt and a/i)
 * ============================================================ */
/* Inner (current) loop: b0_cur ≈ 1/L_coil */
#define VCADRC_L_HENRY          (0.0014f)               /* 1.4 mH PLACEHOLDER */
#define VCADRC_B0_CUR_NOMINAL   (1.0f / VCADRC_L_HENRY)
#define VCADRC_B0_CUR_DESIGN    (0.6f * VCADRC_B0_CUR_NOMINAL)

/* Outer (position) loop: b0_pos ≈ Kt / M_mover */
#define VCADRC_KT_N_PER_A       (7.6f)                  /* PLACEHOLDER N/A   */
#define VCADRC_M_KG             (0.05f)                 /* PLACEHOLDER 50 g  */
#define VCADRC_B0_POS_NOMINAL   (VCADRC_KT_N_PER_A / VCADRC_M_KG)
#define VCADRC_B0_POS_DESIGN    (0.6f * VCADRC_B0_POS_NOMINAL)


/* ============================================================
 *  Bandwidths - START LOW, raise after stable operation confirmed.
 *  See ref/ADRC_INTEGRATION.md Phase-2 procedure.
 * ============================================================ */
#define VCADRC_CUR_WC_HZ        (200.0f)                /* inner BW         */
#define VCADRC_CUR_WO_FACTOR    (3.0f)                  /* observer 3x wc   */

#define VCADRC_POS_WC_HZ        (30.0f)                 /* outer BW (≤ wc_cur/5) */
#define VCADRC_POS_WO_FACTOR    (5.0f)


/* ============================================================
 *  Saturation limits
 * ============================================================ */
#define VCADRC_U_MAX_SIGNED     (+1.0f)                 /* duty            */
#define VCADRC_U_MIN_SIGNED     (-1.0f)
#define VCADRC_I_MAX_A          (+8.0f)                 /* matches I_MAX_MA */
#define VCADRC_I_MIN_A          (-8.0f)


/* ============================================================
 *  API
 * ============================================================ */

/** Initialise both controllers with PLACEHOLDER parameters.
 *  Call once during application startup (before FreeRTOS tasks). */
void  VCAdrc_Init(void);

/** Replace plant gains after Phase-1 system identification.
 *  Re-applies cached bandwidths. Not IRQ-safe; call with loops disabled. */
void  VCAdrc_SetPlantParams(float b0_cur, float b0_pos);

/** Online bandwidth retune.  Not IRQ-safe; call with loops disabled. */
void  VCAdrc_RetuneCurrent (float wc_hz, float wo_hz);
void  VCAdrc_RetunePosition(float wc_hz, float wo_hz);

/** Enable / disable each loop.  Disable forces zero output and
 *  bumplessly re-seeds the observer state from current measurement
 *  on the next enable. */
void  VCAdrc_EnableCurrent (bool on);
void  VCAdrc_EnablePosition(bool on);
bool  VCAdrc_IsCurrentEnabled (void);
bool  VCAdrc_IsPositionEnabled(void);

/** Inner loop entry-point. CALL FROM 5 kHz CURRENT ISR.
 *  @param  i_meas_A  measured coil current, signed (PUSH +)
 *  @return signed duty in [-1, +1] (saturated). Map to TLE9201 dir+duty
 *          at the call site. If loop is disabled, returns 0.0f.
 */
float VCAdrc_CurrentStep(float i_meas_A);

/** Set current setpoint (called either by outer ADRC2 loop or directly
 *  from supervisor in current-only mode). Atomic float write on M33. */
void  VCAdrc_SetCurrentRef(float i_ref_A);

/** Outer loop entry-point. CALL FROM 1 kHz POSITION TASK.
 *  @param  pos_meas  measured position (same units as setpoint)
 *  @return current command in [A] (passed to inner ADRC1 via SetCurrentRef).
 *          If position loop disabled, returns 0.0f and does NOT modify
 *          the current reference.
 */
float VCAdrc_PositionStep(float pos_ref, float pos_meas);


/* ============================================================
 *  Telemetry (read-only from any context; not strictly atomic on float
 *  but adequate for SWV / RTT / DWIN display)
 * ============================================================ */
extern volatile float vcadrc_dbg_i_meas_A;
extern volatile float vcadrc_dbg_i_ref_A;
extern volatile float vcadrc_dbg_u_signed;
extern volatile float vcadrc_dbg_cur_z2_dist;     /* ADRC1 disturbance est */
extern volatile float vcadrc_dbg_pos_meas;
extern volatile float vcadrc_dbg_pos_ref;
extern volatile float vcadrc_dbg_pos_z2_vel;      /* ADRC2 velocity  est */
extern volatile float vcadrc_dbg_pos_z3_dist;     /* ADRC2 disturbance est */


#ifdef __cplusplus
}
#endif

#endif /* VCA_ADRC_H */
