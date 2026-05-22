/**
 * @file    vca_adrc.c
 * @brief   RF562A VCA cascaded ADRC controller - scaffold implementation.
 *
 *  - Inner loop : ADRC1, expected call rate VCADRC_CUR_RATE_HZ (5 kHz).
 *  - Outer loop : ADRC2, expected call rate VCADRC_POS_RATE_HZ (1 kHz).
 *
 *  The module ships DISABLED.  An explicit call to VCAdrc_EnableCurrent(true)
 *  and VCAdrc_EnablePosition(true) is required, AFTER the plant gains have
 *  been updated with measured values via VCAdrc_SetPlantParams().
 *
 *  Bringup checklist:
 *    1. VCAdrc_Init()  - safe; uses placeholder gains, both loops disabled.
 *    2. Run Phase-1 open-loop step (separate code) to measure L_coil, Kt/M.
 *    3. VCAdrc_SetPlantParams(1/L_meas * 0.6, Kt/M * 0.6).
 *    4. Optional VCAdrc_RetuneCurrent / Position to start from low BW.
 *    5. VCAdrc_EnableCurrent(true) - verify with i_ref = 0.5 A step.
 *    6. VCAdrc_EnablePosition(true) - verify with small position step.
 */

#include "vca_adrc.h"
#include <math.h>

#ifndef M_TWOPI
#define M_TWOPI 6.28318530717958647692f
#endif


/* ============================================================
 *  Module state
 * ============================================================ */
static ADRC1_t s_adrc_cur;
static ADRC2_t s_adrc_pos;

static volatile float s_i_ref_A      = 0.0f;
static volatile uint8_t s_cur_enable = 0;
static volatile uint8_t s_pos_enable = 0;

/* Cache of last configured bandwidths so SetPlantParams() can recompute. */
static float s_cur_wc_hz = VCADRC_CUR_WC_HZ;
static float s_cur_wo_hz = VCADRC_CUR_WC_HZ * VCADRC_CUR_WO_FACTOR;
static float s_pos_wc_hz = VCADRC_POS_WC_HZ;
static float s_pos_wo_hz = VCADRC_POS_WC_HZ * VCADRC_POS_WO_FACTOR;


/* Telemetry */
volatile float vcadrc_dbg_i_meas_A   = 0.0f;
volatile float vcadrc_dbg_i_ref_A    = 0.0f;
volatile float vcadrc_dbg_u_signed   = 0.0f;
volatile float vcadrc_dbg_cur_z2_dist = 0.0f;
volatile float vcadrc_dbg_pos_meas   = 0.0f;
volatile float vcadrc_dbg_pos_ref    = 0.0f;
volatile float vcadrc_dbg_pos_z2_vel  = 0.0f;
volatile float vcadrc_dbg_pos_z3_dist = 0.0f;


/* ============================================================
 *  API
 * ============================================================ */
void VCAdrc_Init(void)
{
    s_cur_enable = 0;
    s_pos_enable = 0;
    s_i_ref_A    = 0.0f;

    ADRC1_Init(&s_adrc_cur,
               VCADRC_B0_CUR_DESIGN,
               M_TWOPI * s_cur_wc_hz,
               M_TWOPI * s_cur_wo_hz,
               VCADRC_CUR_TS_S,
               VCADRC_U_MIN_SIGNED, VCADRC_U_MAX_SIGNED);

    ADRC2_Init(&s_adrc_pos,
               VCADRC_B0_POS_DESIGN,
               M_TWOPI * s_pos_wc_hz,
               M_TWOPI * s_pos_wo_hz,
               VCADRC_POS_TS_S,
               VCADRC_I_MIN_A, VCADRC_I_MAX_A);
}


void VCAdrc_SetPlantParams(float b0_cur, float b0_pos)
{
    /* Re-tune retains the current bandwidth settings. */
    ADRC1_Retune(&s_adrc_cur,
                 M_TWOPI * s_cur_wc_hz,
                 M_TWOPI * s_cur_wo_hz,
                 b0_cur);
    ADRC2_Retune(&s_adrc_pos,
                 M_TWOPI * s_pos_wc_hz,
                 M_TWOPI * s_pos_wo_hz,
                 b0_pos);
}


void VCAdrc_RetuneCurrent(float wc_hz, float wo_hz)
{
    s_cur_wc_hz = wc_hz;
    s_cur_wo_hz = wo_hz;
    ADRC1_Retune(&s_adrc_cur,
                 M_TWOPI * wc_hz,
                 M_TWOPI * wo_hz,
                 s_adrc_cur.b0);
}


void VCAdrc_RetunePosition(float wc_hz, float wo_hz)
{
    s_pos_wc_hz = wc_hz;
    s_pos_wo_hz = wo_hz;
    ADRC2_Retune(&s_adrc_pos,
                 M_TWOPI * wc_hz,
                 M_TWOPI * wo_hz,
                 s_adrc_pos.b0);
}


void VCAdrc_EnableCurrent(bool on)
{
    if (on && !s_cur_enable) {
        /* Bumpless: seed observer with last measured current.       */
        ADRC1_Reset(&s_adrc_cur, vcadrc_dbg_i_meas_A);
    }
    s_cur_enable = on ? 1u : 0u;
}


void VCAdrc_EnablePosition(bool on)
{
    if (on && !s_pos_enable) {
        ADRC2_Reset(&s_adrc_pos, vcadrc_dbg_pos_meas);
    }
    s_pos_enable = on ? 1u : 0u;
}


bool VCAdrc_IsCurrentEnabled(void)  { return s_cur_enable != 0u; }
bool VCAdrc_IsPositionEnabled(void) { return s_pos_enable != 0u; }


void VCAdrc_SetCurrentRef(float i_ref_A)
{
    if      (i_ref_A > VCADRC_I_MAX_A) i_ref_A = VCADRC_I_MAX_A;
    else if (i_ref_A < VCADRC_I_MIN_A) i_ref_A = VCADRC_I_MIN_A;
    s_i_ref_A = i_ref_A;
}


/* ----------- Inner loop (5 kHz ISR) ------------------------------- */
float VCAdrc_CurrentStep(float i_meas_A)
{
    vcadrc_dbg_i_meas_A = i_meas_A;

    if (!s_cur_enable) {
        vcadrc_dbg_u_signed = 0.0f;
        return 0.0f;
    }

    const float u = ADRC1_Update(&s_adrc_cur, s_i_ref_A, i_meas_A);

    vcadrc_dbg_i_ref_A     = s_i_ref_A;
    vcadrc_dbg_u_signed    = u;
    vcadrc_dbg_cur_z2_dist = s_adrc_cur.z2;
    return u;
}


/* ----------- Outer loop (1 kHz task) ------------------------------ */
float VCAdrc_PositionStep(float pos_ref, float pos_meas)
{
    vcadrc_dbg_pos_meas = pos_meas;

    if (!s_pos_enable) {
        return 0.0f;
    }

    const float i_cmd = ADRC2_Update(&s_adrc_pos, pos_ref, pos_meas);
    VCAdrc_SetCurrentRef(i_cmd);

    vcadrc_dbg_pos_ref     = pos_ref;
    vcadrc_dbg_pos_z2_vel  = s_adrc_pos.z2;
    vcadrc_dbg_pos_z3_dist = s_adrc_pos.z3;
    return i_cmd;
}
