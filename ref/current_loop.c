/**
 * @file    current_loop.c
 * @brief   RF562 VCA current loop integration example
 *          - 20 kHz current loop in ADC injected complete IRQ
 *          - FreeRTOS supervisor task for setpoint, retune, telemetry
 *
 * Hardware assumptions (adapt to RF562 schematic):
 *   - TIM1 : center-aligned complementary PWM, 20 kHz, TRGO triggers ADC
 *   - ADC1 : injected sequence sampling current sense (synced to TIM1)
 *   - 50 mΩ shunt + bidirectional sense (mid-rail = 2048 counts @ 12-bit)
 *
 *   ADC injected complete IRQ → ADRC1_Update → set PWM compare
 *   FreeRTOS task             → setpoint changes, gain retune at <1 kHz
 *
 * IMPORTANT: do NOT call ADRC1_Update from a FreeRTOS task at >1 kHz.
 * Task scheduling jitter (typically >50 µs even at highest priority on
 * F4 with HAL) will alias your 20 kHz controller; the IRQ path is
 * mandatory for the fast loop.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"          /* CubeMX-generated HAL handles */
#include "adrc1.h"
#include <stdint.h>
#include <math.h>


/* ============================================================
 * 1. Compile-time tuning (adjust to RF562 measurements)
 * ============================================================ */

/* --- Sampling --- */
#define CTRL_FREQ_HZ      20000.0f
#define CTRL_TS_S         (1.0f / CTRL_FREQ_HZ)        /* 50 µs */

/* --- VCA electrical parameters (MEASURE these on RF562!) --- 
 * Quick measurement procedure:
 *  L  : LCR meter at 1 kHz on coil leads (mover at rest)
 *  R  : 4-wire DMM
 *  Ke : back-EMF / velocity at constant velocity drive
 */
#define VCA_L_HENRY       0.0005f                       /* 0.5 mH placeholder */
#define VCA_R_OHM         4.0f
#define B0_NOMINAL        (1.0f / VCA_L_HENRY)          /* 2000  */
#define B0_DESIGN         (0.6f * B0_NOMINAL)           /* under-estimate 40% */

/* --- Closed-loop bandwidth target ---
 * Rule of thumb: wc_current ≤ 1/5 of switching freq (≤ 4 kHz here),
 * but limited by sensor noise. Start at the cross-over of your
 * existing PI; raise after stable operation is confirmed.
 */
#define WC_HZ             800.0f
#define WC_RAD_S          (2.0f * 3.14159265358979f * WC_HZ)
#define WO_RAD_S          (5.0f * WC_RAD_S)             /* observer 5× faster */

/* --- Actuator limits (normalized signed duty for H-bridge) --- */
#define U_MAX             (+1.0f)
#define U_MIN             (-1.0f)

/* --- Current sensor scaling --- */
#define ADC_FULL_SCALE    4096.0f
#define VREF_V            3.3f
#define SHUNT_OHM         0.05f
#define SENSE_AMP_GAIN    20.0f
#define I_SENSE_OFFSET    2048                           /* mid-rail */
#define I_SENSE_COUNT_TO_A  \
        ( VREF_V / ADC_FULL_SCALE / (SHUNT_OHM * SENSE_AMP_GAIN) )


/* ============================================================
 * 2. Globals
 * ============================================================ */

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

static ADRC1_t g_adrc_current;

/* Setpoint published by outer (position) loop; assumed 32-bit atomic on M4 */
static volatile float g_i_ref_A   = 0.0f;
static volatile uint8_t g_enabled = 0;

/* Telemetry (for SWV / segger RTT / DWIN display) */
volatile float dbg_i_meas_A = 0.0f;
volatile float dbg_u_out    = 0.0f;
volatile float dbg_z2_dist  = 0.0f;


/* ============================================================
 * 3. Low-level helpers (inlined - keep IRQ path lean)
 * ============================================================ */

static inline float read_current_A(void)
{
    int16_t raw = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1,
                                                      ADC_INJECTED_RANK_1);
    return ((float)(raw - I_SENSE_OFFSET)) * I_SENSE_COUNT_TO_A;
}

static inline void write_duty_signed(float u)
{
    /* Map u in [-1,+1] to complementary PWM:
     *   u =  +1 → full forward duty
     *   u =   0 → 50% duty (both bridges balanced, ≈ zero current)
     *   u =  -1 → full reverse duty
     * Dead-time inserted by TIM1 advanced timer hardware.
     */
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    float duty = 0.5f + 0.5f * u;
    if (duty < 0.0f) duty = 0.0f;
    else if (duty > 1.0f) duty = 1.0f;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          (uint32_t)(duty * (float)period));
}


/* ============================================================
 * 4. Fast loop: ADC injected complete IRQ (20 kHz)
 *    Override the HAL weak callback.
 * ============================================================ */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    if (!g_enabled) {
        write_duty_signed(0.0f);
        return;
    }

    const float i_meas = read_current_A();
    const float u      = ADRC1_Update(&g_adrc_current, g_i_ref_A, i_meas);
    write_duty_signed(u);

    /* lightweight telemetry */
    dbg_i_meas_A = i_meas;
    dbg_u_out    = u;
    dbg_z2_dist  = g_adrc_current.z2;
}


/* ============================================================
 * 5. Public API for outer loop / supervisor
 * ============================================================ */

void CurrentLoop_SetReference(float i_ref_A)
{
    g_i_ref_A = i_ref_A;
}

void CurrentLoop_Enable(void)
{
    /* Bumpless start: seed observer with present measurement */
    ADRC1_Reset(&g_adrc_current, read_current_A());
    g_i_ref_A = 0.0f;
    g_enabled = 1;
}

void CurrentLoop_Disable(void)
{
    g_enabled = 0;
    write_duty_signed(0.0f);
}

/* Online retune (e.g. per-needle gain schedule).
 * Disables IRQ briefly to keep coefficients consistent.
 */
void CurrentLoop_Retune(float wc_hz, float wo_hz, float b0_new)
{
    HAL_NVIC_DisableIRQ(ADC_IRQn);
    ADRC1_Retune(&g_adrc_current,
                 2.0f * 3.14159265358979f * wc_hz,
                 2.0f * 3.14159265358979f * wo_hz,
                 b0_new);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}


/* ============================================================
 * 6. FreeRTOS supervisor task (low rate)
 * ============================================================ */

void CurrentLoopSupervisor(void *argument)
{
    (void)argument;

    /* One-time controller initialisation */
    ADRC1_Init(&g_adrc_current,
               B0_DESIGN, WC_RAD_S, WO_RAD_S, CTRL_TS_S,
               U_MIN, U_MAX);

    /* Bring up hardware: start PWM first, then injected ADC,
     * then enable the loop. */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    CurrentLoop_Enable();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);   /* 100 Hz supervisor */

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        /* TODO:
         *   - over-current / over-temp checks → CurrentLoop_Disable on fault
         *   - publish telemetry to UI / log
         *   - apply gain schedule from position estimate (needle entering
         *     skin → boost wc; idle → reduce wo for noise)
         */
    }
}
