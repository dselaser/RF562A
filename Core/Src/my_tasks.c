/* ═══════════════════════════════════════════════════════════════════════════════
 *  ///my_tasks.c  –  Loop-1(TIM6, 5 kHz 전류제어) + Loop-2(FreeRTOS, 위치제어)
 *
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  Loop-2  (FreeRTOS HPSwitchTask, 1 ms)                                 │
 *  │    PA8 스위치 디바운스 → ADS8325 위치 읽기 → 상태머신(APPROACH/MOVE/   │
 *  │    HOLD/RETURN) → g_i_target_mA 를 루프1 에 넘겨줌                     │
 *  │                                                                         │
 *  │  Loop-1  (TIM6 ISR, 200 µs = 5 kHz)                                   │
 *  │    ADS7041 SPI3 로 전류 읽기 → PI 제어 → TIM8 PWM Duty 직접 수정      │
 *  │    g_i_target_mA (루프2 지령) + 포화 방지 + HOLD 절전                  │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 *  ─ 전류 감지 회로 ─────────────────────────────────────────────────────────
 *    Shunt        : R9 = 10 mΩ  (2512 1 %)
 *    INA240A1     : 게인 20 V/V
 *    REF1933      : 바이어스 1.647 V  (ADS7041 AINM 기준)
 *    ADS7041      : 12 bit, Vref = 3.3 V  → 1 LSB = 3.3/4096 V
 *
 *    ADC 전압 = 1.647 + I_vca × 0.010 × 20
 *    → I_vca (A) = (ADC_V − 1.647) / 0.20
 *    → I_vca (mA) = (ADC_code − BIAS_CODE) × (3300/4096) / 0.20
 *                 = (ADC_code − BIAS_CODE) × 4.028 mA/LSB
 *
 *  ─ TIM6 설정 지침 (CubeMX 에서 직접 추가) ─────────────────────────────────
 *    Prescaler     = 0  (카운터 클록 = APB1Tim = 50 MHz)
 *    Counter Period = 9999  → 50 MHz / (0+1) / (9999+1) = 5 000 Hz
 *    Enable TIM6 global interrupt, Priority = 5 (FreeRTOS 보다 높게)
 *    ※ CubeMX 에서 "TIM6 global interrupt" 를 활성화하면
 *      stm32h5xx_it.c 에 TIM6_DAC_IRQHandler 가 자동 생성됩니다.
 *      그 핸들러 안에 HAL_TIM_IRQHandler(&htim6) 호출이 들어갑니다.
 *
 *  ─ SPI3 설정 (CubeMX, ADS7041) ──────────────────────────────────────────
 *    이미 IOC 에 SPI3 이 설정되어 있음 (4.17 MHz, CPOL=0/CPHA=1, 16 bit)
 *    CS 핀 : PA15 (SPI3_CS, GPIO Output)
 * ═══════════════════════════════════════════════════════════════════════════*/

#include "my_tasks.h"
#include "usart.h"
#include "main.h"
#include "spi.h"
#include "tim.h"          // htim6, htim8 extern
#include "ads8325.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tle9201.h"
#include "motor_hal.h"
#include "vca_control.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdbool.h>
#include "firmware_info.h"
#include <math.h>
#include "switch.h"
#include "wdg.h"

#define RS485_MEASUREMENT_OUTPUT  1  /* 0=RS485 HP 슬레이브(Main 통신), 1=plotter 측정값 출력 (standalone 인증용) */

/* ── LED 광센서(ADS8325) 거리측정 검증 모드 ─────────────────────────────────
 *   1 = VCA 를 1Hz open-loop sine 으로 진동시키며 ADS8325 raw/filtered 값을
 *       UART1 으로 Serial Port Plotter 포맷 "$raw filt;" 으로 송출.
 *   0 = 정상 동작 (production state machine, Main↔HP RS485).
 *  활성화 시 자동으로 RS485_MEASUREMENT_OUTPUT=1 강제 → UART1 플로터 모드.   */
#define VCA_LED_SENSOR_TEST  0

#if VCA_LED_SENSOR_TEST
  #undef  RS485_MEASUREMENT_OUTPUT
  #define RS485_MEASUREMENT_OUTPUT  1
#endif

size_t UART_Cmd_GetRxQueueFree(void);
size_t UART_Cmd_GetRxQueueLength(void);

/* Loop-1 ISR 핸들러 – main.c 의 HAL_TIM_PeriodElapsedCallback 에서 호출 */
void Loop1_TIM6_ISR_Handler(void);

/* ═══════════════════════════════════════════════════════════════════════════
 *  공유 전역 변수
 * ═══════════════════════════════════════════════════════════════════════════*/

#if !USE_LINEAR_ACTUATOR
/* Loop-2 → Loop-1 지령 (단위: mA, 양수=PUSH, 음수=PULL) */
volatile int32_t g_i_target_mA = 0;

/* Loop-1 → Loop-2 모니터링 */
volatile int32_t  g_i_meas_mA   = 0;   // 실측 전류 (mA)
volatile uint16_t g_adc7041_raw = 0;   // ADS7041 원시값 (디버그)
volatile uint8_t  g_i_meas_updated = 0; // Loop2가 새 전류값 쓸 때 1, ISR이 읽은 후 0

/* Loop-2 위치 */
volatile uint16_t g_vca_pos_adc = 0;   /* 제어+표시용 (모터 중 동결) */
volatile uint16_t g_vca_pos_raw = 0;   /* median only  (디버그) */
#else  /* USE_LINEAR_ACTUATOR */
/* ── LA-T8 위치 제어 전역 변수 ── */
volatile float    g_la_pos_mm     = 0.0f;  /* 현재 위치 (mm)         */
volatile uint16_t g_la_pos_adc    = 0;     /* ADC1 raw (12-bit)      */
volatile float    g_la_target_mm  = 0.0f;  /* PID 목표 (mm)          */
volatile uint8_t  g_la_hard_limit = 0;     /* 1=하드 리밋 트리거     */
volatile float    g_la_pid_output = 0.0f;  /* PID 출력 (디버그)      */
volatile uint8_t  g_la_pid_active = 0;     /* 1=PID 동작 중          */
volatile uint16_t g_vca_pos_adc   = 0;     /* RS485 호환용           */
volatile float    g_la_home_offset_mm = 0.0f; /* 홈 위치 오프셋 (부팅 시 자동 캘리브) */
#endif /* USE_LINEAR_ACTUATOR */
volatile int32_t  g_vca_err     = 0;
volatile uint8_t  g_motor_active = 0;  /* 1=모터 PWM 중 → ADC 동결 */

/* 니들 삽입 깊이 — 3.5mm 고정 (GUI 슬라이더 잠금)
 *  사용자 요청: depth=3.5 에서 한번에 도달, 진동/오버슈트 차단
 *  RS485 @B/@N 등 외부 변경은 모두 3.5 로 강제                                 */
volatile float    g_needle_depth_mm = 3.5f;
/* PROBE 결과: 0=무부하, 1=유부하(피부 감지) */
volatile uint8_t  g_load_detected = 0U;

/* GUI READY/STBY 상태: 0=STBY(스위치 무시), 1=READY(스위치 허용) */
/* boot 시 STBY 가 default — Main 도 부트 후 @S 를 보내 동기화 */
volatile uint8_t  g_gui_ready = 0U;

/* Debug */
volatile float   g_u_dbg    = 0;
volatile float   g_duty_dbg = 0;
volatile int     g_dir_dbg  = 0;
volatile int     g_en_dbg   = 0;
/* ISR 호출 카운터 (TIM6 동작 확인용) */
volatile uint32_t g_isr_cnt  = 0;
volatile uint8_t  g_sw_state = 0;
volatile uint8_t  g_vca_state = 0;   /* 0=idle,1=fwd,2=target,3=ret,4=home */
volatile uint8_t  g_hp_error  = 0;   /* 에러코드 00=정상 */
volatile uint8_t  g_foot_state = 0;  /* 0=off, 1=on (Main→HP foot switch) */
volatile uint8_t  g_foot_mode  = 0;  /* 1=Foot 모드 (물리 스위치 무시) */
volatile uint8_t  g_hp_locked  = 0;  /* 1=잠금 (스위치 비활성+로고) */
#if VCA_LED_SENSOR_TEST
volatile uint8_t  g_vca_sine_mode = 1; /* LED 센서 검증: 부팅 즉시 open-loop sine 모드 */
#else
volatile uint8_t  g_vca_sine_mode = 0; /* 1=sine test, 0=production state machine (default) */
#endif

/* ── Main 명령 상태 추적 어레이 ────────────────────────────────────────────
 *  Main 보드에서 송신한 마지막 명령 상태를 기록.
 *  변경 감지 (edge) 로 의도적 동작만 트리거 → 잡음/중복 명령 무시.
 *  Index:
 *    [0] last_op_mode     0=HP, 1=Foot (mode 결정)
 *    [1] last_foot_state  0=released, 1=pressed (foot 트리거 상태)
 *    [2] last_ready       0=Standby, 1=Ready
 *    [3] last_locked      0=unlocked, 1=locked
 *    [4] last_power       0~9 (RF 파워)
 *    [5] last_depth_x10   05~35 (니들 깊이 ×10)
 *    [6] foot_active_ms   @F 마지막 수신 시각 (4 bytes packed)
 *    [7] reserved
 *  변경시 g_main_state_changed = 1 → 다음 cycle 에서 처리                    */
volatile uint8_t  g_main_cmd_state[8] = {0, 0, 1, 0, 5, 35, 0, 0};
volatile uint8_t  g_main_state_changed = 0;
volatile uint32_t g_last_foot_cmd_ms  = 0;
/* @L1 수신 시 production state machine 의 모든 static 변수 강제 reset 요청
 *  1-pin mode 갔다 와도 position EMA / state stuck 안 되도록                 */
volatile uint8_t  g_op_reset_request  = 0;
/* @L1 (1-pin → bipolar 복귀) 후 자동 PUSH (depth=3.5 까지 이동) 종료 시각
 *  cold motor break-away — depth ≤ 3mm 에서 미동작 회피.                    */
volatile uint32_t g_auto_push_until_ms = 0;
/* 치료 펄스 완료 후 강제 retract 플래그 — Main 이 @F0 송신 시 set
 *  HP/FOOT 모드 무관하게 needle home 으로 복귀 강제
 *  사용자 switch 릴리즈 검출되면 자동 clear                                  */
volatile uint8_t  g_retract_request   = 0;
volatile int32_t  g_blk_remain = 0;  /* release block 잔여시간 디버그 */
volatile uint16_t g_sm_pos    = 0;   /* 상태머신이 실제로 보는 pos */
volatile int      g_state_dbg = 0;

/* UART */
TaskHandle_t gUart2TxTaskHandle = NULL;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
static HAL_StatusTypeDef Uart1_Tx_IT(uint8_t *pData, uint16_t size);
static HAL_StatusTypeDef Uart2_Tx_IT(uint8_t *pData, uint16_t size);

/* ADS8325 (위치 ADC) */
#ifndef ADS8325_SAMPLES_PER_LINE
#define ADS8325_SAMPLES_PER_LINE  5
#endif
__attribute__((unused))
static uint16_t g_ads8325_samples[ADS8325_SAMPLES_PER_LINE];
static char     g_ads8325_buf[256];

#if !USE_LINEAR_ACTUATOR
/* ═══════════════════════════════════════════════════════════════════════════
 *  Loop-1 파라미터 – VCA 전류 제어 (TIM6 ISR)
 * ═══════════════════════════════════════════════════════════════════════════*/
#define ADC7041_BIAS_CODE    (640)
#define ADC7041_MA_PER_LSB_F (4.028f)
#define I_BOOST_MA           (8000)
#define I_HOLD_MA            (7000)
#define I_MAX_MA             (8000)
#define ILOOP_KP             (0.00040f)
#define ILOOP_KI             (0.00010f)
#define ILOOP_IMAX           (0.60f)

static volatile float  s_i_err_acc  = 0.0f;
static volatile float  s_duty_l1    = 0.0f;
static volatile int8_t s_l1_dir     = 1;
static volatile bool   s_l1_enable  = false;

#define BOOST_TICKS          (250U)
static volatile uint32_t s_boost_cnt  = 0;
static volatile bool     s_boosting   = false;

#define ADS7041_CS_PORT   GPIOA
#define ADS7041_CS_PIN    GPIO_PIN_15
extern SPI_HandleTypeDef hspi3;

/* Phase 2 이후 미사용 (DMA path가 g_i_meas_mA 갱신).
 *  레거시 fallback / debug 호출용으로 보존. SPI3 RX DMA 와 동시 호출 금지. */
__attribute__((unused))
static uint16_t ADS7041_ReadRaw(void)
{
    uint16_t tx = 0x0000U;
    uint16_t rx = 0x0000U;
    HAL_GPIO_WritePin(ADS7041_CS_PORT, ADS7041_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&tx, (uint8_t*)&rx, 1, 5);
    HAL_GPIO_WritePin(ADS7041_CS_PORT, ADS7041_CS_PIN, GPIO_PIN_SET);
    return (rx >> 4U);
}

#else  /* USE_LINEAR_ACTUATOR */
/* ═══════════════════════════════════════════════════════════════════════════
 *  LA-T8 위치 PID 파라미터 (TIM6 ISR, 1kHz 서브레이트)
 *
 *  회로: VCC_3V3 → R_pullup(2kΩ) → [ADC_NTC] → Pot(0~5kΩ) → AGND
 *  비선형 전압분배기: V = 3.3 × R_pot / (R_pullup + R_pot)
 *  변환: pos_mm = LA_POT_SCALE × adc / (4095 − adc)
 *        LA_POT_SCALE = stroke × R_pullup / R_pot_max = 10 × 2 / 5 = 4.0
 *  사용거리: 최대 3.5mm
 * ═══════════════════════════════════════════════════════════════════════════*/
#define LA_STROKE_MM       (10.0f)
#define LA_HARD_LIMIT_MM   (11.0f)    /* 절대 초과 금지 (raw 단위, depth 3.5→10.66 + 여유) */
#define LA_DECEL_ZONE_MM   (0.5f)     /* 감속 구간 */
#define LA_DECEL_START_MM  (LA_HARD_LIMIT_MM - LA_DECEL_ZONE_MM)

/* 비선형 ADC↔mm 변환 (2kΩ pullup + 5kΩ pot 전압분배기) */
#define LA_POT_SCALE       (4.0f)
static inline float la_adc_to_mm(uint16_t adc)
{
    if (adc >= 4090U) return LA_STROKE_MM;
    if (adc == 0U)    return 0.0f;
    return LA_POT_SCALE * (float)adc / (4095.0f - (float)adc);
}
static inline uint16_t la_mm_to_adc(float mm)
{
    if (mm <= 0.0f)        return 0;
    if (mm >= LA_STROKE_MM) return 4090;
    return (uint16_t)(4095.0f * mm / (LA_POT_SCALE + mm));
}

/* PID 게인 (1kHz) */
#define LA_PID_KP_INIT     (0.80f)    /* 1mm 오차 → 80% duty (고속 도달) */
#define LA_PID_KI_INIT     (0.20f)
#define LA_PID_KD_INIT     (0.03f)
#define LA_PID_DT          (0.001f)   /* 1kHz */
#define LA_PID_OUT_MAX     (0.95f)
#define LA_PID_I_MAX       (0.60f)
#define LA_ISR_DECIMATE    (5U)       /* 5kHz→1kHz */
#define LA_DEADBAND_MM     (0.15f)    /* 목표 ±0.15mm → 모터 OFF */
#define LA_DEADBAND_MS     (100U)     /* 데드밴드 내 100ms 유지 → 정지 */
#define LA_RESTART_MM      (0.30f)    /* IDLE에서 재시작 임계 (> DEADBAND) */

/* 위치 교정 (UART 실측 기반):
 *   물리 0.5mm 삽입 → pos_raw = 10.32 - 7.7 = 2.62  (off=7.7 실측)
 *   물리 3.5mm 삽입 → pos_raw = 12.37 - 7.7 = 4.67
 *   기울기 = (4.67-2.62)/(3.5-0.5) = 0.6833 raw/mm
 *   절편 = 2.62 - 0.5*0.6833  = 2.28  raw (홈→피부 기계적 오프셋)
 *   → target_raw = depth_mm * 0.6833 + 2.28
 * gain=1.0: pos_mm = raw 단위 그대로 사용 */
#define LA_CAL_GAIN        (1.0f)
#define LA_CAL_SLOPE       (0.6833f)  /* raw-mm per insertion-mm */
#define LA_SKIN_POS_MM     (9.98f)    /* 피부표면(0mm삽입) 절대 la_adc_to_mm 값
                                       * = 10.32 - 0.5*0.6833 (0.5mm 실측에서 역산) */

/* depth(삽입mm) → target(raw pos_mm) 변환
 * 절대 기준: target_abs = depth*slope + 9.98 (피부표면)
 * 상대 기준: target_rel = target_abs - home_offset (동적) */
static inline float la_depth_to_target(float depth_mm)
{
    float t = depth_mm * LA_CAL_SLOPE + LA_SKIN_POS_MM - g_la_home_offset_mm;
    if (t > (LA_HARD_LIMIT_MM - 0.1f)) t = LA_HARD_LIMIT_MM - 0.1f;
    if (t < 0.0f) t = 0.0f;
    return t;
}

static volatile float    s_la_pid_kp     = LA_PID_KP_INIT;
static volatile float    s_la_pid_ki     = LA_PID_KI_INIT;
static volatile float    s_la_pid_kd     = LA_PID_KD_INIT;
static volatile float    s_la_pid_i_acc  = 0.0f;
static volatile float    s_la_pid_prev_e = 0.0f;
static volatile uint32_t s_la_isr_div    = 0;
static volatile uint8_t  s_la_enable     = 0;

/* ADC EMA 필터: 노이즈 + 간헐적 ADC 스파이크 감쇠 (τ ≈ 10ms @ 5kHz) */
#define LA_FILT_ALPHA      (0.02f)
static volatile float    s_la_pos_filt = 0.0f;

/* ADC 이상 감지 */
static volatile uint8_t  s_la_adc_fault_cnt = 0;
#define LA_ADC_FAULT_TH    (50U)      /* 50틱 연속 0 또는 4095 → 센서 에러 */
#endif /* USE_LINEAR_ACTUATOR */

/* ─── 공통 방향 정의 (VCA/LA 모두 사용) ─────────────────────────────────*/
#ifndef DIR_PUSH
#define DIR_PUSH    (0)   /* LA: TLE9201 DIR LOW = PUSH (pos 증가) */
#endif
#ifndef DIR_PULL
#define DIR_PULL    (1)   /* LA: TLE9201 DIR HIGH = PULL (pos 감소) */
#endif

/* ─── TIM8 Duty 직접 설정 (ISR 에서 안전) ───────────────────────────────
 *  Phase 1: motor_hal_set_duty() wrapper로 위임. 이름은 호환 유지.
 * ─────────────────────────────────────────────────────────────────────────*/
static inline void TIM8_SetDutyDirect(float duty)
{
    motor_hal_set_duty(duty);
}

#if !USE_LINEAR_ACTUATOR
/* ═══════════════════════════════════════════════════════════════════════════
 *  Loop-1 : TIM6 5 kHz VCA 전류 제어 핸들러
 * ═══════════════════════════════════════════════════════════════════════════*/
void Loop1_TIM6_ISR_Handler(void)
{
    g_isr_cnt++;   /* TIM6 ISR 호출 확인용 – UART에서 증가하면 정상 */

    /* Phase 2: 매 tick(5kHz) DMA 캡처 시작 → 다음 tick에서 fresh raw 획득.
     *  motor_hal_iadc_start_capture() 는 ISR-safe (busy 시 자동 skip).
     *  Loop1 enable/disable 무관하게 항상 호출 → HPSwitchTask 도 fresh 값 수혜.
     *  RxCpltCallback (motor_hal.c) 가 g_adc7041_raw / g_i_meas_mA 갱신은
     *  ISR 본체에서 수행 (구조 분리 유지). */
    motor_hal_iadc_start_capture();

    /* DMA 캡처가 완료된 직전 tick의 raw를 mA로 변환하여 글로벌 갱신 */
    {
        uint16_t raw = motor_hal_get_iadc_raw();
        g_adc7041_raw    = raw;
        g_i_meas_mA      = (int32_t)(((int32_t)raw - ADC7041_BIAS_CODE)
                                       * ADC7041_MA_PER_LSB_F);
        g_i_meas_updated = 1;
    }

    /* ── 1. 루프 비활성화 시 → duty는 Loop-2가 관리, ISR은 손대지 않음 ── */
    if (!s_l1_enable) {
        s_i_err_acc = 0.0f;
        s_duty_l1   = 0.0f;
        return;
    }

    /* ── 3. 지령 전류 한계 적용 ─────────────────────────────────────── */
    int32_t i_tgt = g_i_target_mA;

    /* 방향 결정 */
    if (i_tgt >= 0) {
        s_l1_dir = 1;   /* PUSH */
    } else {
        s_l1_dir = -1;  /* PULL */
        i_tgt    = -i_tgt;
    }

    /* 부스트 카운터 관리 */
    if (s_boosting) {
        if (s_boost_cnt < BOOST_TICKS) {
            s_boost_cnt++;
        } else {
            s_boosting = false;  /* 부스트 종료 → HOLD 한계로 전환 */
        }
    }

    /* 전류 한계 선택 */
    int32_t i_limit = s_boosting ? I_BOOST_MA : I_HOLD_MA;
    if (i_tgt > i_limit) i_tgt = i_limit;
    if (i_tgt > I_MAX_MA) i_tgt = I_MAX_MA;  /* 절대 최대 */

    /* ── 4. PI 전류 제어 ─────────────────────────────────────────────── */
    /* 측정값도 방향 반영 (PUSH 방향 양수 기준) */
    /* ── 4. PI 전류 제어 — 새 측정값 있을 때만 계산 (1ms 갱신 대응) ── */
    if (g_i_meas_updated) {
        g_i_meas_updated = 0;  /* 플래그 클리어 */

        int32_t i_meas_dir = (s_l1_dir == 1) ? g_i_meas_mA : -g_i_meas_mA;
        float   err_f      = (float)(i_tgt - i_meas_dir);

        /* 비례항 */
        float up = ILOOP_KP * err_f;

        /* 적분항 (Anti-windup 포함) */
        s_i_err_acc += ILOOP_KI * err_f;
        if      (s_i_err_acc >  ILOOP_IMAX) s_i_err_acc =  ILOOP_IMAX;
        else if (s_i_err_acc < -ILOOP_IMAX) s_i_err_acc = -ILOOP_IMAX;

        float duty_cmd = up + s_i_err_acc;

        /* duty 클램프 */
        if (duty_cmd < 0.0f) duty_cmd = 0.0f;
        if (duty_cmd > 1.0f) duty_cmd = 1.0f;

        s_duty_l1  = duty_cmd;
        g_duty_dbg = duty_cmd;

        /* ── 5. PWM 적용 ──────────────────────────────────────────────── */
        TIM8_SetDutyDirect(duty_cmd);
    }
    /* 새 측정값 없으면 이전 duty 유지 (TIM8 건드리지 않음) */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Loop-1 제어 API  (Loop-2 에서 호출)
 * ═══════════════════════════════════════════════════════════════════════════*/

/**
 * @brief  전류 루프 시작. 부스트 모드(고전류 관통력) 포함
 * @param  i_target_mA  목표 전류 (양수=PUSH, 음수=PULL)
 * @param  boost        true = 처음 BOOST_TICKS 동안 I_BOOST_MA 허용
 */
static void Loop1_Start(int32_t i_target_mA, bool boost)
{
    s_i_err_acc  = 0.0f;
    s_duty_l1    = 0.0f;
    s_boost_cnt  = 0U;
    s_boosting   = boost;
    g_i_target_mA = i_target_mA;
    s_l1_enable  = true;
}

/**
 * @brief  전류 루프 정지 (PWM=0)
 */
static void Loop1_Stop(void)
{
    s_l1_enable   = false;
    g_i_target_mA = 0;
    s_i_err_acc   = 0.0f;
}

/**
 * @brief  목표 전류만 갱신 (부스트 상태는 유지)
 */
static void Loop1_SetTarget(int32_t i_target_mA)
{
    g_i_target_mA = i_target_mA;
}

#else  /* USE_LINEAR_ACTUATOR */
/* ═══════════════════════════════════════════════════════════════════════════
 *  LA-T8 : TIM6 5 kHz ISR (ADC읽기 + 하드리밋 + 위치 PID@1kHz)
 * ═══════════════════════════════════════════════════════════════════════════*/
#include "adc.h"

#ifndef TLE9201_DIS_GPIO_Port
#define TLE9201_DIS_GPIO_Port  GPIOC
#define TLE9201_DIS_Pin        GPIO_PIN_7
#endif

static volatile uint8_t s_la_adc_ready = 0;  /* HPSwitchTask에서 1로 설정 */

void Loop1_TIM6_ISR_Handler(void)
{
    g_isr_cnt++;

    /* ADC1 캘리브레이션 전이면 스킵 (HPSwitchTask 초기화 대기) */
    if (!s_la_adc_ready) return;

    /* ── 1. ADC1 중앙값 읽기 (3회 샘플 → median, PWM 스파이크 제거) ── */
    {
        uint16_t s[3];
        for (int i = 0; i < 3; i++) {
            ADC1->CR |= ADC_CR_ADSTART;
            uint32_t tmo = 100;
            while (!(ADC1->ISR & ADC_ISR_EOC)) {
                if (--tmo == 0) return;
            }
            s[i] = (uint16_t)(ADC1->DR & 0xFFFU);
        }
        /* 3개 정렬 → 중앙값 (PWM 전환 1회 스파이크 자동 제거) */
        if (s[0] > s[1]) { uint16_t t=s[0]; s[0]=s[1]; s[1]=t; }
        if (s[1] > s[2]) { uint16_t t=s[1]; s[1]=s[2]; s[2]=t; }
        if (s[0] > s[1]) { uint16_t t=s[0]; s[0]=s[1]; s[1]=t; }
        g_la_pos_adc = s[1];  /* median */
    }
    uint16_t adc_raw = g_la_pos_adc;
    g_vca_pos_adc = adc_raw;   /* RS485 호환 */

    float pos_raw = (la_adc_to_mm(adc_raw) - g_la_home_offset_mm) * LA_CAL_GAIN;
    if (pos_raw < 0.0f) pos_raw = 0.0f;

    /* EMA 필터: 모터 EMI → ADC 노이즈 제거 (τ ≈ 4ms @ 5kHz) */
    s_la_pos_filt += LA_FILT_ALPHA * (pos_raw - s_la_pos_filt);
    float pos_mm = s_la_pos_filt;   /* 제어에는 필터링 값 사용 */
    g_la_pos_mm = pos_mm;

    /* ── 2. ADC 이상 감지 ── */
    if (adc_raw == 0 || adc_raw >= 4095) {
        if (++s_la_adc_fault_cnt >= LA_ADC_FAULT_TH) {
            if (s_la_enable) TIM8_SetDutyDirect(0.0f);
            s_la_enable = 0;
            g_hp_error = 11;  /* 센서 에러 */
            return;
        }
    } else {
        s_la_adc_fault_cnt = 0;
    }

    /* ── 3. HARD LIMIT (매 틱, 5kHz — 안전은 raw 값 사용) ── */
    if (pos_raw >= LA_HARD_LIMIT_MM) {
        g_la_hard_limit = 1;
        if (!s_la_enable) {
            /* PID 비활성 → PWM 건드리지 않음 (task 직접 제어 보호) */
        } else if (g_la_target_mm < pos_raw) {
            /* target < pos: retract(홈 복귀) 중이므로 PID 계속 허용 */
        } else {
            /* PID 활성 + PUSH 방향 → 완전 차단 */
            TIM8_SetDutyDirect(0.0f);
            motor_hal_dis_set(MOTOR_HAL_BRIDGE_ENABLE);
            s_la_enable = 0;
            s_la_pid_i_acc = 0.0f;
            g_la_pid_active = 0;
            return;
        }
    }

    /* ── 4. PID 비활성 시 즉시 리턴 ── */
    if (!s_la_enable) {
        s_la_pid_i_acc = 0.0f;
        return;
    }

    /* ── 5. 1kHz 서브레이트 (매 5번째 틱) ── */
    if (++s_la_isr_div < LA_ISR_DECIMATE) return;
    s_la_isr_div = 0;

    float target = g_la_target_mm;
    float err    = target - pos_mm;

    /* ── 6-7. 비례 제어: 목표 접근 시 자연 감속, ±0.1mm 이내 정지 ── */
    float u;
    float err_abs = err;
    if (err_abs < 0.0f) err_abs = -err_abs;

    if (err_abs < LA_DEADBAND_MM) {
        u = 0.0f;                            /* 목표 도달 → 출력 0 */
    } else {
        /* 비례: 1mm당 0.80 duty → 자연 감속, 최소 0.12 (정지마찰 극복) */
        float duty = 0.80f * err_abs;
        if (duty > LA_PID_OUT_MAX) duty = LA_PID_OUT_MAX;
        if (duty < 0.12f)          duty = 0.12f;
        u = (err > 0.0f) ? duty : -duty;
    }

    g_la_pid_output = u;

    /* ── 8. 방향 + PWM 적용 (하드웨어 DIR 반전) ── */
    if (u >= 0.0f) {
        TLE9201_SetDir(DIR_PULL);   /* PID +: extend (위치 증가) */
        TIM8_SetDutyDirect(u);
    } else {
        TLE9201_SetDir(DIR_PUSH);   /* PID -: retract (위치 감소) */
        TIM8_SetDutyDirect(-u);
    }
    g_duty_dbg = (u >= 0.0f) ? u : -u;
    g_dir_dbg  = (u >= 0.0f) ? 1 : 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  LA PID 제어 API  (HPSwitchTask에서 호출)
 * ═══════════════════════════════════════════════════════════════════════════*/
static void LA_PID_Start(float target_mm)
{
    s_la_pid_i_acc  = 0.0f;
    s_la_pid_prev_e = 0.0f;
    s_la_isr_div    = 0;
    g_la_hard_limit = 0;
    g_la_pid_active = 1;
    if (target_mm > LA_HARD_LIMIT_MM) target_mm = LA_HARD_LIMIT_MM;
    if (target_mm < 0.0f)             target_mm = 0.0f;
    g_la_target_mm  = target_mm;
    s_la_enable     = 1;
}

static void LA_PID_Stop(void)
{
    s_la_enable     = 0;
    g_la_pid_active = 0;
    s_la_pid_i_acc  = 0.0f;
    TIM8_SetDutyDirect(0.0f);
}

static void LA_PID_SetTarget(float target_mm)
{
    if (target_mm > LA_HARD_LIMIT_MM) target_mm = LA_HARD_LIMIT_MM;
    if (target_mm < 0.0f)             target_mm = 0.0f;
    g_la_target_mm = target_mm;
}

void LA_PID_SetGains(float kp, float ki, float kd)
{
    s_la_pid_kp = kp;
    s_la_pid_ki = ki;
    s_la_pid_kd = kd;
}

void LA_PID_GetGains(float *kp, float *ki, float *kd)
{
    *kp = s_la_pid_kp;
    *ki = s_la_pid_ki;
    *kd = s_la_pid_kd;
}
#endif /* USE_LINEAR_ACTUATOR */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Loop-2 위치 제어 파라미터 (기존 코드 유지)
 * ═══════════════════════════════════════════════════════════════════════════*/

#ifndef HP_SW_GPIO_Port
#define HP_SW_GPIO_Port GPIOA
#endif
#ifndef HP_SW_Pin
#define HP_SW_Pin GPIO_PIN_8
#endif
#ifndef DIR_PUSH
#define DIR_PUSH    (1)   /* TLE9201 DIR HIGH = 실제 PUSH (pos 증가) ★최종확정★ */
#endif
#ifndef DIR_PULL
#define DIR_PULL    (0)   /* TLE9201 DIR LOW  = 실제 PULL (pos 감소) ★최종확정★ */
#endif

#define HP_SAMPLE_MS          (1U)
#if !USE_LINEAR_ACTUATOR       /* ── VCA Loop-2 파라미터 ── */
#define HP_DEBOUNCE_MS        (10U)
#define HP_PULL_KICK_MS       (60U)
#define HP_PULL_HOLD1_MS      (120U)
#define HP_PULL_HOLD1_DUTY    (0.12f)
#define HP_PULL_HOLD2_DUTY    (0.04f)
#define HP_PULL_SOFTSTART_MS  (8U)
#define HP_PULL_KICK_MIN_DUTY (0.60f)
#define HP_PULL_TOTAL_MAX_MS  (300U)
#define HP_PUSH_KICK_MS       (100U)
#define HP_PUSH_HOLD_DUTY     (0.20f)
#define DUTY_HOLD_MIN         0.040f

#define HP_STARTUP_PULL_ENABLE    (0)
#define HP_STARTUP_PULL_KICK_MS   (HP_PULL_KICK_MS)
#define HP_STARTUP_PULL_HOLD_MS   (120U)
#define HP_STARTUP_PULL_HOLD_DUTY (HP_PULL_HOLD1_DUTY)

/* ─── 전류 지령 매핑 ─────────────────────────────────────────────────────
 *  Loop-2 는 duty 대신 전류(mA)를 Loop-1 에 지령합니다.
 *  아래 매크로로 각 상태에서 목표 전류를 설정하세요.
 *
 *  VCA 스펙: Fp=44N, Kf=7.6 N/A, Fc=13.8N, I_cont≈1.8A
 *  피부 관통 시 순간 8A (I_BOOST_MA), 위치 유지 시 1~2A
 * ─────────────────────────────────────────────────────────────────────────*/
#define I_CMD_APPROACH_MA    (1000)   /* 미사용 */
#define I_CMD_MOVE_MA        (1000)   /* 미사용 (런타임 결정) */
#define I_CMD_BOOST_MA       (3000)   /* 미사용 */
#define I_CMD_HOLD_PUSH_MA    (400)   /* 미사용 */
#define I_CMD_HOLD_PULL_MA   (-800)   /* 미사용 */
#define I_CMD_RETURN_MA      (-2000)  /* 미사용 */

/* VCA_WRAP 상수 – VCA_StartupPushWrapToZeroOnce() 에서 사용 */
#ifndef VCA_WRAP_HIGH_TH
#define VCA_WRAP_HIGH_TH  (64000U)
#endif
#ifndef VCA_WRAP_LOW_TH
#define VCA_WRAP_LOW_TH   (2000U)
#endif
#ifndef VCA_WRAP_TARGET
#define VCA_WRAP_TARGET   (150U)
#endif

/* ── 위치 ADC 설정 ────────────────────────────────────────────────────────
 *  VCA_ADC_PER_MM : ★ 기구 실측 필수 ★
 *    HOME(pos≈3500)에서 캘리퍼로 1mm 이동 후 ADC 차이 측정
 * ─────────────────────────────────────────────────────────────────────────*/
/* HOME ADC: 부팅 시 auto-home 시퀀스에서 갱신 (반사판 위치별 매번 다름) */
volatile uint16_t g_vca_adc_home = 25000;        /* fallback 초기값 */
#define VCA_ADC_HOME          ((int32_t)g_vca_adc_home)
#define VCA_ADC_PER_MM        (6667)             /* 26000 ADC / 3.9mm 기구한계 */
#define VCA_DEPTH_MM_MIN      (0.5f)
#define VCA_DEPTH_MM_MAX      (3.5f)             /* 소프트 제한 (기구 3.9mm) */
#define VCA_ADC_MAX_SAFE      (VCA_ADC_HOME + (int32_t)(VCA_ADC_PER_MM * VCA_DEPTH_MM_MAX))
/* ── 깊이 → ADC 변환 (HOME은 런타임 캘리브레이션 값) ───────────────────── */
#define DEPTH_MM_TO_ADC(mm)   ((int32_t)(VCA_ADC_HOME + (mm) * VCA_ADC_PER_MM))

/* ── 측정값 기반 위치 교정 LUT ─────────────────────────────────────────
 *  실측 사이클 #2 (LCD 설정 → 측정 mm, depth_correction 적용 후):
 *    0.5→X(미도달)  1.0→0.81  1.5→0.89  2.0→2.0✓  2.5→3.79  3.0→3.87  3.5→3.85
 *  분석:
 *    - 모터 마찰 floor ~0.8mm — 그 아래 도달 불가 (LCD 0.5는 ~0.8로 매핑)
 *    - internal 1.7 = sweet spot (정확히 2.0mm)
 *    - internal 1.6 → 0.89, internal 1.9 → 3.79 (transition zone 비선형)
 *    - 2.5~3.5 사이 internal 1.7~1.85의 좁은 윈도우에 분포
 * ───────────────────────────────────────────────────────────────────── */
static float depth_correction(float user_mm)
{
    /* 사용자 원함 → motor 내부 target
     * identity 매핑 — pure-P + ramp 제어로 재교정 필요 (이전 LUT은 over-shoot 시절 inverse) */
    static const float lut_user[]     = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f};
    static const float lut_internal[] = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f};
    const int N = 7;
    if (user_mm <= lut_user[0])     return lut_internal[0];
    if (user_mm >= lut_user[N - 1]) return lut_internal[N - 1];
    for (int i = 0; i < N - 1; i++) {
        if (user_mm <= lut_user[i + 1]) {
            float t = (user_mm - lut_user[i]) / (lut_user[i + 1] - lut_user[i]);
            return lut_internal[i] + t * (lut_internal[i + 1] - lut_internal[i]);
        }
    }
    return lut_internal[N - 1];
}
#define VCA_ADC_TARGET        DEPTH_MM_TO_ADC(g_needle_depth_mm)

/* ══ 1단계: PROBE — 부하 판별 (~1mm PUSH 후 전류 측정) ══════════════════
 *  PROBE_DUTY      : PROBE duty (작게 — 부하 감지만)
 *  PROBE_ADC_END   : PROBE 끝 위치 (HOME + 1mm)
 *  PROBE_CURRENT_TH: 이 전류(mA) 이상이면 유부하(피부 있음) ★실측 조정★
 *  PROBE_TIMEOUT_MS: PROBE 최대 허용 시간
 * ═════════════════════════════════════════════════════════════════════════*/
#define PROBE_DUTY            (0.280f) /* PROBE duty — 유부하 저항 극복용 */
#define PROBE_ADC_END         ((int32_t)(VCA_ADC_HOME + 1 * VCA_ADC_PER_MM / 2))  /* 0.5mm */
#define PROBE_CURRENT_TH      (270)    /* mA 실측 확정: 무부하 max=240, 유부하=330 */
#define PROBE_TIMEOUT_MS      (300U)   /* 유부하 저항으로 느릴 수 있음 */

/* ══ 2단계: PUSH ═════════════════════════════════════════════════════════
 *  PUSH_I_NO_LOAD  : 무부하 전류 — 낮게 유지해야 오버슈트 없음
 *  PUSH_I_LOADED   : 유부하(피부) 관통 전류
 *  VCA_BRAKE_ZONE  : 목표까지 이 거리부터 PUSH_I_BRAKE로 낮춤
 *                    ★ 크게 잡을수록 일찍 감속 → 오버슈트 감소 ★
 *  PUSH_I_BRAKE    : 제동 구간 전류 (작을수록 부드러운 정지)
 *
 *  무부하 진동    → PUSH_I_NO_LOAD 낮춤, VCA_BRAKE_ZONE 키움
 *  부하 오버슈트  → PUSH_I_LOADED 낮춤, VCA_BRAKE_ZONE 키움
 *  부하 관통력 부족 → PUSH_I_LOADED 올림
 * ═════════════════════════════════════════════════════════════════════════*/
#define PUSH_I_NO_LOAD        (800)    /* mA: 무부하 — (현재 미사용, Loop2 duty 직접제어로 변경) */
#define NO_LOAD_MOVE_DUTY     (0.215f) /* 무부하 MOVE duty */
#define NO_LOAD_BRAKE_DUTY    (0.190f) /* 무부하 1차 제동 */
#define NO_LOAD_BRAKE2_DUTY   (0.165f) /* 무부하 2차 제동 */
#define NO_LOAD_BRAKE_ZONE2   (3000)   /* 2차 제동 시작 거리 */
#define PUSH_I_LOADED         (6000)   /* mA: 유부하 피부관통 전류 (참고용) */
#define PUSH_DUTY_LOADED      (0.550f) /* 유부하 PUSH duty (관통력 강화) */
#define VCA_BRAKE_ZONE        (8000)  /* ADC: 목표 8000 전부터 제동 */
#define PUSH_I_BRAKE          (300)    /* mA: 제동 전류 — 무부하 부드러운 정지 */

/* ══ 3단계: HOLD — Loop2 단독 duty 직접제어 ══════════════════════════════
 *  Loop1 완전 OFF. 발열 없는 최소 duty로 스프링 평형 유지.
 *
 *  HOLD_DUTY_FF : 스프링(Fc=13.8N) 버티는 duty
 *    pos 서서히 내려감 → 올림 (0.01 단위)
 *    pos 서서히 올라감 → 낮춤
 *  HOLD_KP      : 데드밴드 밖 P 보정 (작게)
 *  VCA_DB_HOLD  : 이 범위 안에서 FF duty만 → 진동 없음 (핵심)
 * ═════════════════════════════════════════════════════════════════════════*/
#define HOLD_DUTY_FF          (0.360f) /* 유부하 실리콘 반발력 극복 */
#define HOLD_DUTY_MAX         (0.60f)  /* HOLD/MOVE 최대 duty */
#define HOLD_KP               (0.000025f) /* P보정 */
#define VCA_DB_HOLD           (2000)      /* 데드밴드 좁혀서 빠른 보정 */
#define VCA_DB_MOVE           (400)

/* ══ 선형화 LUT: 원하는 깊이(mm) → 필요한 duty ═══════════════════════════
 *  3차 교정 (LCD 슬라이더 실측 2026-03-09):
 *    duty 0.1984 → 실측 1.80mm  (slider 1)
 *    duty 0.2079 → 실측 2.10mm  (slider 3)
 *    duty 0.2196 → 실측 2.51mm  (slider 4)
 *    duty 0.2316 → 실측 3.07mm  (slider 5)
 *    duty 0.2729 → 실측 3.84mm  (slider 6)
 *
 *  역변환: desired_depth → 보간(duty, actual) 으로 필요 duty 산출
 *  ★ 하한 ~1.8mm 이하는 VCA 스프링+PROBE 한계로 도달 불가
 * ═══════════════════════════════════════════════════════════════════════*/
#define LIN_LUT_N  8
static const float lut_depth[LIN_LUT_N] = {
    0.00f, 1.00f, 1.80f, 2.10f, 2.51f, 3.07f, 3.84f, 4.50f
};
static const float lut_duty[LIN_LUT_N] = {
    0.185f, 0.190f, 0.1984f, 0.2079f, 0.2196f, 0.2316f, 0.2729f, 0.30f
};

static float depth_to_duty(float depth_mm)
{
    /* 범위 밖 클램프 */
    if (depth_mm <= lut_depth[0])
        return lut_duty[0];
    if (depth_mm >= lut_depth[LIN_LUT_N - 1])
        return lut_duty[LIN_LUT_N - 1];

    /* 구간별 선형 보간 */
    for (int i = 0; i < LIN_LUT_N - 1; i++) {
        if (depth_mm <= lut_depth[i + 1]) {
            float t = (depth_mm - lut_depth[i])
                    / (lut_depth[i + 1] - lut_depth[i]);
            return lut_duty[i] + t * (lut_duty[i + 1] - lut_duty[i]);
        }
    }
    return lut_duty[LIN_LUT_N - 1];
}
#define MOVE_TIMEOUT_MS       (500)   /* MOVE 시간 제한: ADC 동결 중 위치 전환 불가 → 타이머로 HOLD 전환 */

/* ══ 4단계: RETURN — PULL 추출 → PUSH 속도비례 감속 → 무전류 안착 ═══════
 *  전략 (3단계, 홈 충돌 무소음):
 *    ① EXTRACT  : DIR_PULL + 능동 듀티 → 피부에서 능동 추출 (전류 흐름)
 *    ② BRAKE    : DIR_PUSH + 속도비례 듀티 → 홈 도착 직전 운동에너지 흡수
 *    ③ SILENT LAND : VCA OFF → 잔여 스프링력만으로 정지~극저속 안착
 *
 *  핵심: PULL 추출만으로는 관성+잔여 스프링력이 홈 스토퍼를 강타("탁")
 *        BRAKE 단계가 필수 — 도착 직전에 속도 ≈ 0 으로 줄여야 무소음
 *
 *             ┌────────┐
 *     HOLD    │        │
 *             │        └─╲                       ← ① PULL 추출 (전류)
 *                          ╲
 *     BRAKE────────────────┐                     ← ② PUSH 속도비례 감속
 *                            ╲╲╲
 *     STOP─────────────────────┐ ─ ─ ─ ─ ─      ← VCA OFF 전환점 (vel≈0)
 *                                ─ ─ ─
 *     HOME ──────────────────────────┘           ← ③ 잔여 스프링력 자연 안착
 * ═══════════════════════════════════════════════════════════════════════*/
#define RETURN_EXTRACT_DUTY        (0.22f)  /* 피부 추출 PULL duty (능동 추출 전류) */
#define RETURN_BRAKE_START_MM      (1.5f)   /* 이 mm 이하 → PUSH 감속 단계 진입 */
#define RETURN_BRAKE_GAIN          (0.008f) /* 속도비례 브레이크 게인 (duty/(ADC·ms⁻¹)) */
#define RETURN_BRAKE_DUTY_MIN      (0.18f)  /* 브레이크 최저 duty (스프링 평형 보장) */
#define RETURN_BRAKE_DUTY_MAX      (0.55f)  /* 브레이크 최대 duty (포화) */
#define RETURN_STOP_VEL            (3.0f)   /* 정지 판정 |vel| (ADC/ms ≈ 0.3 mm/s) */
#define RETURN_EXTRACT_TIMEOUT_MS  (500U)   /* 추출 단계 안전 타임아웃 */
#define RETURN_BRAKE_TIMEOUT_MS    (500U)   /* 감속 단계 안전 타임아웃 */
#define RETURN_STOP_POS            ((int32_t)(VCA_ADC_HOME + 1200))
#define RETURN_CUSHION_MS          (300U)   /* Release Gate 안착 판정 대기 (재사용) */

#define VCA_STABLE_MS         (60U)
#define VCA_PID_DT_MS         (HP_SAMPLE_MS)
#define VCA_PID_DT_S          (0.001f)
#define DUTY_MOVE_MAX         (0.560f) /* 15V 기준 (0.35×1.6) */
#define VCA_SINE_HOME_ADC     (4000U)  /* mechanical home 마진 — 무부하 bouncing 방지 */
#define VCA_SINE_PEAK_ADC     (33000U) /* 4mm 물리 진폭 */
#define VCA_SINE_FREQ_HZ      (1.0f)   /* 0.5 → 1.0 Hz: 한 주기 1초로 단축 */
/* ── 위치 PID + FF 폐루프 (직접 듀티) ──
 *   ADS8325 fresh 위치 읽기는 깨끗하므로 그것을 기준으로 위치 PID.
 *   limit-cycle 회피 핵심:
 *     ① FF가 듀티 80% 담당 (정적 duty-position 선형 근사)
 *     ② KP 매우 낮게 — 미세 보정만 → saturate 안 됨
 *     ③ KI는 누적 오차에만 천천히 — bias 보정용
 *     ④ KD = 0 (잡음 증폭 방지)
 *     ⑤ Loop1 우회 — 직접 VCA_SetDuty (Loop1 PI dynamics 제거)
 *     ⑥ 슬루 제한으로 듀티 변화 부드럽게.                                 */
/* HPSwitchTask 메인 루프 사이클 (sine 모드 + production controller 공용)
 *  변경 이력:
 *   10ms — sine 1Hz 시험모드용 음향 최적화 (100Hz update, 검증용 모드, 실사용X)
 *    1ms — production controller BURST 위치제어 강화 (모터 슬램 ~1-2ms 추적)
 *  영향 점검:
 *   - production controller HOLD: u_raw=ff_static 고정 → slew/EMA 무관 (영향 없음)
 *   - production PUSH: slew=1.0/ema=1.0 무력화, BURST 시간은 ms 단위 절대값 (무관)
 *   - sine 모드: TARGET_RAMP_RATE 가 cycle/ADC 단위라 효과 변하지만, sine 은 시험모드
 *   - vel 계산: (pos-prev) × (1000/CYCLE_MS) 자동 보정
 *  CPU 부담: ADS8325 16샘플 burst (~80us) × 1000Hz = 8% CPU (HW 여유 충분)        */
#define VCA_SINE_CYCLE_MS     (1U)
#define VCA_SINE_SAMPLE_N     (16U)
#define VCA_SINE_TRIM_K       (4U)      /* 양 끝 4개씩 절삭 → 8개 평균 (PWM ripple 강감쇠) */
#define VCA_SINE_POS_EMA_A    (0.20f)   /* 위치 EMA 필터 강도 (강하게 — 스파이크/노이즈 차단) */
#define VCA_SINE_U_EMA_A      (0.45f)   /* 듀티 EMA — 강한 평활로 진동 억제 */
/* 기계 끝점/스틱션 회피 — 선형 구간 안에서만 동작
 *   상단 PEAK 36000 ← 0.28 로 32000 도달 → 더 강하게 0.31 시도
 *   하단 HOME 8000  ← 0.16 으로 1500 까지 떨어짐 → 0.20 으로 강화
 *   FF 곡선이 비선형이라 (motor 강도 vs 스프링 압축) 약간 공격적으로 설정    */
#define VCA_SINE_FF_HOME_DUTY (0.05f)  /* HOME 정적 평형 — KP 피드백과 함께 사용 (was 0.03) */
#define VCA_SINE_FF_PEAK_DUTY (0.20f)  /* PEAK 정적 평형 — KP 피드백이 평형점 보정 (was 0.10) */
#define VCA_HOLD_KP           (3.0e-5f) /* HOLD 위치 피드백 게인 — 오버슈트 시 duty 감소, 언더슈트 시 증가 */
#define VCA_HOLD_DEADZONE     (500.0f)  /* HOLD err deadzone (ADC) ≈0.05mm — 센서 노이즈 chatter 방지 */
/* 위치 기반 KD 게이트 — smoothstep 전환으로 머뭇거림 제거
 *   pos > KD_GATE_HI (12000): KD = 0
 *   KD_GATE_LO (3000) ~ HI:  smoothstep(3x²-2x³) 으로 부드럽게 활성
 *   pos < KD_GATE_LO: KD 풀
 *   넓고 부드러운 게이트 + 낮은 KD = 머뭇거림 없는 자연스러운 브레이크       */
#define VCA_SINE_KD_GATE_HI   (12000.0f)
#define VCA_SINE_KD_GATE_LO   (3000.0f)
/* 속도 FF 끔 (1Hz 에서 하강 시 motor 너무 약화 → bouncing 원인)
 *   대신 KD 항으로 측정 속도에 비례한 댐핑 추가:
 *     u_kd = -vel_meas × KD  (하강 vel<0 → +KD: motor 더 가압 → 브레이크)
 *   KD 너무 크면 진폭 축소 + 노이즈 증폭. 0.5e-6 → ±0.047 max (적정).
 *   vel_meas EMA 필터 (α=0.3) 로 미분 노이즈 감쇠.                          */
#define VCA_SINE_VEL_FF_GAIN  (0.0f)
#define VCA_SINE_KD           (0.5e-6f) /* 속도 댐핑 — smoothstep 게이트로 머뭇거림 회피 */
#define VCA_SINE_VEL_EMA_A    (0.30f)   /* vel_meas EMA: 잡음 감쇠 */
/* 안정 우선 — 게인 대폭 축소, 무부하에서 부드러운 사인파 보장             */
#define VCA_SINE_KP           (1.2e-5f) /* 약한 KP — 진동 회피 */
#define VCA_SINE_KI           (0.0f)    /* I 끔 — 사이클간 wind-up 차단 */
#define VCA_SINE_I_LIMIT      (0.05f)   /* I 한도 (KI=0이라 무관) */
#define VCA_SINE_U_SLEW       (0.015f)  /* 100Hz × 0.015 = 1.5/s — 부드러운 변화 */
#define VCA_SINE_ERR_DEADZONE (8.0f)   /* deadzone — 작은 잡음 chatter 방지 */
#define VCA_SINE_DUTY_MAX     (0.45f)  /* 듀티 한도 작게 — 무부하 stability 우선 */
/* Cascade 전류 제어 — Loop1 (5kHz PI on ADS7041 10mΩ shunt) 재활성화
 *  외부 sine 루프가 듀티(0~1) 대신 전류 지령(mA) 을 Loop1 에 전달
 *  → 속도/back-EMF/외란 무관하게 일정 force 유지 → 강성 향상              */
#define VCA_SINE_I_MAX_MA     (5000)   /* u=1.0 매핑 — 5A (I_HOLD_MA 7A 안쪽) */
/* 보호: TLE9201 thermal/OC latch 자동 해제 + 주기적 쿨다운
 *   FAULT_CLR_CYC : 매 N 사이클마다 TLE9201 diag clear
 *   COOLDOWN_CYC  : 매 N 사이클마다 모터 완전 OFF (쿨다운)
 *   COOLDOWN_MS   : 쿨다운 지속시간 (모션 잠시 멈춤)                       */
#define VCA_SINE_FAULT_CLR_CYC (250U)   /* 5s @ 50Hz */
#define VCA_SINE_COOLDOWN_CYC  (1500U)  /* 30s @ 50Hz */
#define VCA_SINE_COOLDOWN_MS   (500U)
#define DUTY_HOLD_MAX         (0.160f) /* 15V 기준 (0.10×1.6) */
#define HOLD_COAST_ERR        (600)
#define DUTY_HOLD_MAX_PUSH    (DUTY_HOLD_MAX)
#define DUTY_HOLD_MAX_PULL    (0.02f)
#define VCA_ADC_AVG_N         (4U)
#define REV_INHIBIT_ERR       (800)
#define DUTY_SLEW_STEP        (0.01f)
#define HOLD_UPDATE_MS        (50U)
#define DIR_MIN_SWITCH_MS     (60U)
#define QUIET_SAMPLE_MS       (0U)
#define VCA_VEL_DAMP_K        (0.00015f)
#define HOLD_KI_HZ            (0.00003f)
#define HOLD_ERR_I_ENABLE     (2500)
#define HOLD_VEL_I_ENABLE     (60)
#define HOLD_VEL_DEADBAND     (40)
#define HOLD_I_LEAK           (0.985f)
#define HOLD_BIAS_ENABLE_ERR  (1000)
#define VCA_VEL_DAMP_ALPHA    (0.2f)
#define DUTY_ZERO_EPS         (0.01f)
#define DUTY_HARD_MAX         (1.0f)

#define VCA_ADC_SAT_THRESHOLD (60000)
#define VCA_ADC_EXIT_TARGET   (100)
#define VCA_INIT_PUSH_DUTY    (0.20f)
#define VCA_INIT_TIMEOUT_MS   (500)
#endif /* !USE_LINEAR_ACTUATOR -- VCA Loop-2 params */

/* 스위치 debounce 임계 — 10ms 루프 기준
 *  PRESS=4 → 40ms 검출 (HP 스위치 바운스 충분히 필터)
 *  RELEASE=8 → 80ms 검출 (잡음으로 인한 진동 방지)
 *  GRACE=100 → press 후 100ms release 무시 (확실한 누름 보장)            */
#define PRESS_DB_MS           (4U)
#define RELEASE_DB_MS         (8U)
#define RELEASE_GRACE_MS      (100U)
#define HOLD_ENTRY_BLOCK_MS   (200U)

/* ═══════════════════════════════════════════════════════════════════════════
 *  유틸리티 함수
 * ═══════════════════════════════════════════════════════════════════════════*/

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void PID_Reset(PID_t *p)
{
    p->i_acc   = 0.0f;
    p->prev_e  = 0.0f;
}

static inline float PID_Update(PID_t *p, float setpoint, float meas, float dt_s)
{
    float e  = setpoint - meas;
    float up = p->kp * e;
    p->i_acc += (p->ki * e * dt_s);
    float ud  = p->kd * ((e - p->prev_e) / dt_s);
    p->prev_e = e;
    float u   = up + p->i_acc + ud;
    if      (u > p->out_max) { u = p->out_max; if (p->i_acc > p->out_max) p->i_acc = p->out_max; }
    else if (u < p->out_min) { u = p->out_min; if (p->i_acc < p->out_min) p->i_acc = p->out_min; }
    return u;
}

static inline bool in_deadband_i32(int32_t e, int32_t db)
{
    return (e <= db) && (e >= -db);
}

static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

__attribute__((unused))
static uint16_t median5_u16(const uint16_t h[5])
{
    uint16_t v[5] = {h[0], h[1], h[2], h[3], h[4]};
    for (int i = 0; i < 5; i++)
        for (int j = i + 1; j < 5; j++)
            if (v[j] < v[i]) { uint16_t t = v[i]; v[i] = v[j]; v[j] = t; }
    return v[2];
}

#if !USE_LINEAR_ACTUATOR       /* ── VCA 유틸리티 함수 ── */
static inline uint16_t VCA_ReadPosADC(uint8_t avgN)
{
    (void)avgN;
    return g_vca_pos_adc;   /* ADS8325_AcqTask가 1ms마다 업데이트 */
}

/* ─── TLE9201 래퍼 (Loop-2 에서 방향만 제어, duty 는 Loop-1 이 담당) ────
 *  Loop-1 이 활성화된 동안에는 VCA_SetDuty() 를 Loop-2 가 직접
 *  호출하지 않습니다.  단, 방향(DIR)과 Enable 은 Loop-2 가 제어합니다.
 * ─────────────────────────────────────────────────────────────────────────*/

/* TLE9201 24bit 에러 클리어 – tle9201.c의 ClearErrors()는 16bit로 잘못됨
 * 실제 TLE9201: WR_DIAG(Clear) = 0x87 0x00 0x00 (24bit SPI)             */
__attribute__((unused))
static void TLE9201_ClearErrors24(void)
{
    extern SPI_HandleTypeDef hspi6;
    const uint8_t tx[3] = {0x87, 0x00, 0x00};
    uint8_t rx[3];
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi6, (uint8_t*)tx, rx, 3, 10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    osDelay(1);
    const uint8_t nop[3] = {0x00, 0x00, 0x00};
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi6, (uint8_t*)nop, rx, 3, 10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
}

/* Phase 1: CCR1 write를 motor_hal_set_ccr1_raw()로 위임.
 *   기존 0x40013400 + 0x34 직접 쓰기와 동일 동작 (TIM8->CCR1 write).
 *   BDTR MOE 보강은 그대로 유지 (타이머 설정 영역, Phase 2에서 정리). */
static inline void VCA_SetDuty(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    uint32_t arr = TIM8->ARR;
    uint32_t ccr = (uint32_t)((float)(arr + 1U) * duty);
    motor_hal_set_ccr1_raw(ccr);
    TIM8->BDTR |= TIM_BDTR_MOE;  /* MOE 항상 확인 */
}

static inline void vca_on(void)
{
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_ENABLE);
}

static inline void vca_off(void)
{
    Loop1_Stop();
    VCA_SetDuty(0.0f);
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);
}

/* ─── vca_apply_u : Loop-2 에서 방향/enable 제어 + Loop-1 전류 지령 ──────
 *  u > 0  → PUSH 방향, |u| 를 전류(mA)로 환산하여 Loop-1 에 전달
 *  u < 0  → PULL 방향
 * ─────────────────────────────────────────────────────────────────────────*/
static inline void vca_apply_u(float u, int32_t err)
{
    static bool    last_dir_push       = true;
    static float   duty_prev           = 0.0f;
    static uint32_t last_dir_change_ms = 0;
    (void)duty_prev;  /* Loop-1 이 duty 관리 */

    float au = fabsf(u);

    if (au < DUTY_ZERO_EPS) {
        /* 거의 0 → 전류 지령 최솟값 유지 (완전 OFF 는 vca_off) */
        Loop1_SetTarget(0);
        g_u_dbg    = u;
        g_duty_dbg = 0.0f;
        g_dir_dbg  = last_dir_push ? 1 : 0;
        g_en_dbg   = 1;
        return;
    }

    bool want_push = (u >= 0.0f);
    uint32_t now_ms = osKernelGetTickCount();

    /* 방향 반전 억제 */
    if (want_push != last_dir_push) {
        if (iabs32(err) < REV_INHIBIT_ERR) {
            want_push = last_dir_push;
            u = last_dir_push ? +au : -au;
        } else if ((now_ms - last_dir_change_ms) < DIR_MIN_SWITCH_MS) {
            want_push = last_dir_push;
            u = last_dir_push ? +fabsf(u) : -fabsf(u);
        } else {
            last_dir_change_ms = now_ms;
        }
    }

    /* 방향 설정 (Loop-2 담당) */
    last_dir_push = want_push;
    TLE9201_SetDir(want_push ? DIR_PUSH : DIR_PULL);

    /* 전류 지령 (duty → mA 변환: I_CMD_MOVE_MA 를 full scale 로 매핑) */
    int32_t i_cmd = (int32_t)(fabsf(u) * (float)I_CMD_MOVE_MA);
    if (!want_push) i_cmd = -i_cmd;

    Loop1_SetTarget(i_cmd);

    g_u_dbg    = u;
    g_duty_dbg = s_duty_l1;
    g_dir_dbg  = want_push ? 1 : 0;
    g_en_dbg   = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  상태 머신 열거형
 * ═══════════════════════════════════════════════════════════════════════════*/
typedef enum {
    VCA_HOME_OFF = 0,
    VCA_PROBE_LOAD,        /* 1a: ~1mm PUSH + 전류 측정 → 부하 판별 */
    VCA_APPROACH_TARGET,   /* 미사용 (호환 유지) */
    VCA_MOVE_TO_TARGET,    /* 1b: Loop1 전류제어 PUSH → 목표까지 */
    VCA_HOLD_TARGET,       /* 2 : Loop2 duty 직접제어 HOLD */
    VCA_RETURN_HOME        /* 3 : PULL 능동 추출 → 무전류 자연 안착 */
} VCA_State_t;

/* ─── Startup 래퍼 ────────────────────────────────────────────────────────*/
static inline void VCA_StartupPushWrapToZeroOnce(void)
{
    uint16_t p_prev = VCA_ReadPosADC(8);
    if (p_prev <= VCA_WRAP_LOW_TH) return;
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_ENABLE);
    TLE9201_SetDir(DIR_PUSH);
    const uint32_t on_ms = 12U, off_ms = 18U, max_pulses = 60U;
    const uint32_t t0 = HAL_GetTick();
    for (uint32_t i = 0; i < max_pulses; i++) {
        VCA_SetDuty(0.35f);  osDelay(on_ms);
        VCA_SetDuty(0.0f);   osDelay(off_ms);
        uint16_t p = VCA_ReadPosADC(8);
        if ((p_prev >= VCA_WRAP_HIGH_TH) && (p <= VCA_WRAP_LOW_TH)) break;
        if (p <= VCA_WRAP_TARGET) break;
        if ((HAL_GetTick() - t0) > 1200U) break;
        p_prev = p;
    }
    VCA_SetDuty(0.0f);
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE); VCA_SetDuty(0.0f);
}
#endif /* !USE_LINEAR_ACTUATOR -- VCA utilities */

#if !USE_LINEAR_ACTUATOR       /* ── VCA 상태 머신 + HPSwitchTask ── */
/* ═══════════════════════════════════════════════════════════════════════════
 *  HPSwitchTask  (Loop-2, 1 ms FreeRTOS)
 *
 *  변경점:
 *   - 상태 전환 시 Loop1_Start() / Loop1_SetTarget() 호출
 *   - APPROACH: 부스트 모드로 Loop1 시작 (피부 관통력 확보)
 *   - HOLD    : 저전류로 전환 (발열 최소화)
 *   - RETURN  : PULL 전류 지령
 *   - TLE9201 SetPWM_duty 는 Loop-1 이 관리하므로 직접 호출 금지
 *     (단, vca_on/vca_off 에서는 초기화 목적으로 0 설정 허용)
 * ═══════════════════════════════════════════════════════════════════════════*/
void HPSwitchTask_impl(void *argument)
{
    static float   s_hold_base_duty = 0.0f;  (void)s_hold_base_duty;
    static uint8_t s_hold_latched   = 0U;    (void)s_hold_latched;
    (void)argument;

    /* ★ 부팅 시 TLE9201 강제 OFF (IOC 초기값과 무관하게 확실히 비활성화) */
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE); /* DIS=HIGH=OFF */
    VCA_SetDuty(0.0f);
    osDelay(50);

    VCA_Init();   /* 내부에서 TLE9201_SetPWM_Frequency(40000) 호출 */
    /* PWM 주파수: 20kHz (가청대 밖, 무음)
     *  진단 결과 (스코프): duty 1.0 시 motor 양단 ~9.5V 인가 중 = HW 한계.
     *  PWM 주파수는 duty<1.0 일 때만 force 영향 → duty 1.0 BURST 에선 무관.
     *  HOLD (duty 0.35) 시 buzz 방지 위해 가청대 밖 20kHz 로 복원.            */
    extern void TLE9201_SetPWM_Frequency_pub(uint32_t);  /* tle9201.c에 없으면 직접 */
    __HAL_TIM_DISABLE(&htim8);
    __HAL_TIM_SET_PRESCALER(&htim8, 0);
    /* 20kHz: ARR = 50MHz/20kHz - 1 = 2499 */
    TIM8->ARR = 2499U;
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    __HAL_TIM_ENABLE(&htim8);
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);
    VCA_SetDuty(0.0f);

    /* CC1NE(complementary) 비활성화 - CH1만 사용 */
    TIM8->CCER &= ~TIM_CCER_CC1NE;
    TIM8->CCER |=  TIM_CCER_CC1E;
    TIM8->CCMR1 = (TIM8->CCMR1 & ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE))
                | (0x6U << TIM_CCMR1_OC1M_Pos);
    TIM8->BDTR  |= TIM_BDTR_MOE;
    motor_hal_set_ccr1_raw(0);  /* CCR1=0 (Phase 1: motor_hal wrapper 경유) */
    osDelay(10);

    /* Phase 2: ADS7041 SPI3 RX DMA 초기화 (HAL_SPI_Init(&hspi3) 후 1회).
     *  이후 TIM6 ISR(5kHz) 매 tick 에서 motor_hal_iadc_start_capture() 호출
     *  → DMA 백그라운드 캡처. motor_hal_get_iadc_raw() 로 fresh raw 획득. */
    motor_hal_iadc_init();

    /* ─── Auto-home calibration (boot 시 1회) ──────────────────────────
     *  PULL 방향으로 강한 duty 인가 → 위치값 안정되면 그 ADC를 g_vca_adc_home에 저장.
     *  HP 스위치 눌려있으면 skip (안전). Timeout 4초.                       */
    {
        bool sw_pressed_boot =
            (HAL_GPIO_ReadPin(HP_SW_GPIO_Port, HP_SW_Pin) == GPIO_PIN_RESET);
        if (!sw_pressed_boot) {
            /* ADS8325 핸들 초기화 (AcqTask가 먼저 했어도 idempotent) */
            ADS8325_Init(&g_ads8325, &hspi1, GPIOA, GPIO_PIN_4);

            /* 모터 power 인가 전 baseline 읽기 (진단용) */
            uint32_t base_sum = 0;
            for (int i = 0; i < 16; i++) base_sum += ADS8325_Read(&g_ads8325);
            uint16_t pos_baseline = (uint16_t)(base_sum / 16);
            (void)pos_baseline;
            g_vca_adc_home = pos_baseline;   /* 1차 추정: baseline 자체 (모터 미가동 상태) */

            motor_hal_dis_set(MOTOR_HAL_BRIDGE_ENABLE);
            TLE9201_SetDir(DIR_PULL);
            VCA_SetDuty(0.35f);                          /* 강한 PULL — 마찰 극복 */

            uint32_t t_start  = osKernelGetTickCount();
            uint16_t pos_last = ADS8325_Read(&g_ads8325);
            uint32_t t_stable = t_start;

            for (;;) {
                osDelay(10);
                uint16_t pos_now = ADS8325_Read(&g_ads8325);
                int32_t  d = (int32_t)pos_now - (int32_t)pos_last;
                if (d < 0) d = -d;
                if (d > 30) {                            /* 아직 움직이는 중 */
                    pos_last = pos_now;
                    t_stable = osKernelGetTickCount();
                }
                uint32_t now = osKernelGetTickCount();
                if ((now - t_stable) >= 300U) {          /* 300ms 안정 → 정착 */
                    g_vca_adc_home = pos_now;
                    break;
                }
                if ((now - t_start) >= 4000U) {          /* 안전 timeout 4초 */
                    g_vca_adc_home = pos_now;
                    break;
                }
            }

            VCA_SetDuty(0.0f);
            motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);
            osDelay(20);
        }
    }

    /* ─── 스위치 디바운스 변수 ─── */
    uint8_t raw_now = (HAL_GPIO_ReadPin(HP_SW_GPIO_Port, HP_SW_Pin) == GPIO_PIN_SET) ? 0U : 1U;
    static bool     s_latched_push      = false;
    static uint32_t s_press_cnt         = 0;
    static uint32_t s_release_cnt       = 0;
    static uint32_t s_release_block_until = 0;
    static bool     s_sw_init           = false;
    if (!s_sw_init) { s_latched_push = (raw_now == 1U); s_sw_init = true; }

    bool last_in_push = s_latched_push;
    uint32_t stable_t0   = 0; (void)stable_t0;
    static uint32_t s_push_t0   = 0;  (void)s_push_t0;
    static uint16_t s_push_pos0 = 0;  (void)s_push_pos0;
    static uint32_t release_t0  = 0;  /* 루프 밖: edge·gate 공유 */
    static int32_t  s_push_i_ma = 0;  /* PROBE 결과로 결정된 PUSH 전류 */
    static uint32_t s_probe_t0  = 0;  /* PROBE 타임아웃 기준 */
    static uint32_t s_move_t0   = 0;  /* MOVE 타임아웃 기준 */
    static uint16_t s_adc_at_home = 0; (void)s_adc_at_home; /* PROBE 진입 직전 실제 home ADC */
    static int32_t  s_pos_prev    = 0;
    static float    s_vel_lp      = 0.0f;
    static float    s_loaded_duty = 0.0f; (void)s_loaded_duty; /* 유부하 Loop2 전류제어 duty 누적 */
    static int32_t  s_burst_i_peak = 0;   /* 대안3: burst 중 전류 피크 추적 */

    /* RETURN 상태변수 (Release Gate에서 리셋) */
    static uint32_t s_ret_ramp_t0  = 0;     /* 추출 시작 시각 (안전 타임아웃) */
    static uint8_t  s_ret_phase    = 0;     /* 0=PULL 추출, 2=무전류 안착 */

    VCA_State_t s_state = last_in_push ? VCA_MOVE_TO_TARGET : VCA_HOME_OFF;

    static PID_t s_pid_move = {
        .kp = 0.000005f, .ki = 0.0f, .kd = 0.0f, .dt = 0.001f,
        .out_min = -DUTY_MOVE_MAX, .out_max = +DUTY_MOVE_MAX,
        .i_acc = 0.0f, .prev_e = 0.0f
    };
    static PID_t s_pid_hold = {
        .kp = 0.00016f, .ki = (HOLD_KI_HZ / 2.0f), .kd = 0.0f, .dt = 0.001f,
        .out_min = -DUTY_HOLD_MAX, .out_max = +DUTY_HOLD_MAX,
        .i_acc = 0.0f, .prev_e = 0.0f
    };
    PID_Reset(&s_pid_move);
    PID_Reset(&s_pid_hold);

    if (last_in_push && g_gui_ready) {
        vca_on();
        TLE9201_SetDir(DIR_PUSH);
        {   /* PROBE duty = min(PROBE_DUTY, 선형화 duty) */
            float pd = depth_to_duty(g_needle_depth_mm);
            if (pd > PROBE_DUTY) pd = PROBE_DUTY;
            VCA_SetDuty(pd);
        }
        s_push_i_ma = PUSH_I_NO_LOAD;
        s_probe_t0  = osKernelGetTickCount();
        s_adc_at_home = g_vca_pos_adc;  /* 모터 시작 전 실제 home ADC 캡처 */
        g_motor_active = 1;  /* PROBE 시작 → ADC 동결 (EMI 방지) */
        s_state = VCA_PROBE_LOAD;
    } else {
        vca_off();
        s_state = VCA_HOME_OFF;
    }

    /* ═══════════════════════ MAIN LOOP ═══════════════════════════════════ */
    for (;;)
    {
        Wdg_TaskAlive(WDG_TASK_HPSWITCH);
        static int s_prev_state = -1;
        if ((int)s_state != s_prev_state) {
            if (s_prev_state == (int)VCA_HOLD_TARGET &&
                (int)s_state  != (int)VCA_HOLD_TARGET) {
                s_hold_latched = 0U;
            }
            s_prev_state = (int)s_state;
        }
        uint32_t now = osKernelGetTickCount();

        /* ── 0. ADS7041 전류 읽기 ──────────────────────────────────────────
         *  Phase 2: blocking ADS7041_ReadRaw 폴링 제거.
         *  Loop1 TIM6 ISR(5kHz)이 SPI3 RX DMA로 g_i_meas_mA / g_adc7041_raw
         *  자동 갱신. SPI3 충돌 방지를 위해 HPSwitchTask 는 SPI 직접 접근 안 함.
         *  (Production controller / sine 모드의 ADS8325 SPI1 burst 와는 무관) */

        /* ── 1. 스위치 디바운스 ────────────────────────────────────────── */
        /* PA8 PULLUP: 누르면 LOW(0V) → pressed=1, 놓으면 HIGH → pressed=0 */
        uint8_t raw = (HAL_GPIO_ReadPin(HP_SW_GPIO_Port, HP_SW_Pin) == GPIO_PIN_SET) ? 0U : 1U;
        if (!s_latched_push) {
            if (raw == 1U) {   /* 눌림 */
                if (++s_press_cnt >= (PRESS_DB_MS / HP_SAMPLE_MS)) {
                    s_latched_push  = true;
                    s_press_cnt     = 0;
                    s_release_cnt   = 0;
                    s_release_block_until = now + RELEASE_GRACE_MS;
                }
            } else s_press_cnt = 0;
        } else {
            if (raw == 0U) {   /* 놓음 */
                if (now < s_release_block_until) {
                    s_release_cnt = 0;
                } else {
                    if (++s_release_cnt >= (RELEASE_DB_MS / HP_SAMPLE_MS)) {
                        s_latched_push  = false;
                        s_release_cnt   = 0;
                        s_press_cnt     = 0;
                    }
                }
            } else s_release_cnt = 0;
        }
        /* 입력 소스 선택:
         * g_foot_mode=1 → Foot 전용 (물리 스위치 무시)
         * g_foot_mode=0 → 물리 스위치 (Foot 무시)
         * g_hp_locked=1 → 모든 입력 차단
         * g_auto_push_until_ms 설정 → 시각 미만 동안 가상 switch press
         *   (@L1 1-pin → bipolar 복귀 후 cold motor break-away 용)        */
        bool in_push;
        if (g_hp_locked)
            in_push = false;
        else if (g_auto_push_until_ms != 0U &&
                 (int32_t)(g_auto_push_until_ms - osKernelGetTickCount()) > 0)
            in_push = true;          /* 자동 PUSH 활성 */
        else if (g_foot_mode)
            in_push = (g_foot_state != 0);
        else
            in_push = s_latched_push;
        /* 자동 PUSH 만료 시 클리어 */
        if (g_auto_push_until_ms != 0U &&
            (int32_t)(g_auto_push_until_ms - osKernelGetTickCount()) <= 0) {
            g_auto_push_until_ms = 0U;
        }
        g_sw_state = in_push ? 1U : 0U;
        g_blk_remain = (int32_t)s_release_block_until - (int32_t)now;

        /* ── 2. 위치 필터링 ────────────────────────────────────────────── */
        uint16_t pos_raw = VCA_ReadPosADC(VCA_ADC_AVG_N);
        static uint16_t s_hist[5]   = {0};
        static uint8_t  s_hi        = 0;
        static bool     s_hist_init = false;
        static float    s_pos_lp    = 0.0f;
        if (!s_hist_init) {
            for (int i = 0; i < 5; i++) s_hist[i] = pos_raw;
            s_pos_lp   = (float)pos_raw;
            s_hist_init = true;
        } else {
            s_hist[s_hi] = pos_raw;
            s_hi = (uint8_t)((s_hi + 1U) % 5U);
        }
        uint16_t pos_med = median5_u16(s_hist);
        const float alpha = 0.12f;
        s_pos_lp = s_pos_lp + alpha * ((float)pos_med - s_pos_lp);
        uint16_t pos = (uint16_t)(s_pos_lp + 0.5f);
        g_sm_pos = pos;   /* UART 모니터링용 */

        /* ── 2-b. 하드 리밋 제거 ─────────────────────────────────────────
         *  기존 38000 하드 리밋은 오버슈트 시 duty=0 → 전체 낙하 → 재push
         *  → 진동 유발. MOVE/HOLD 내 VCA_ADC_MAX_SAFE(39000) 체크로 충분 */

        /* 속도 추정 */
        int32_t dpos  = (int32_t)pos - s_pos_prev;
        s_pos_prev    = (int32_t)pos;
        s_vel_lp      = s_vel_lp + VCA_VEL_DAMP_ALPHA * ((float)dpos - s_vel_lp);

        /* ── 2-c. STBY 게이트 비활성화 ────────────────────────────────────
         *  Production controller 가 STANDBY 모드 (depth tracking) 직접 처리.
         *  sine 모드도 STBY 무관 (테스트). 따라서 게이트 불필요.             */

#if VCA_LED_SENSOR_TEST
        /* ── 3a-T. VCA free (수동 조작) — LED 광센서 거리측정 검증 ───────────
         *  H-bridge OFF → 코일 무여자, 무토크 → 손으로 자유롭게 이동 가능.
         *  ADS8325 burst 샘플 → raw(median) / trimmed-mean(filt) 을 g_vca_pos_*
         *  에 기록 → ADS8325_RS485_Task 가 100Hz 로 "$raw filt;" 송출.        */
        if (g_vca_sine_mode) {
            /* H-bridge & Loop1 강제 OFF — 매 사이클 재확인 (안전) */
            if (s_l1_enable) Loop1_Stop();
            VCA_SetDuty(0.0f);
            motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);  /* DIS=HIGH=OFF */
            g_motor_active = 0;
            g_duty_dbg = 0.0f;
            g_dir_dbg  = 0;

            /* ADS8325 burst (16 샘플) + insertion sort */
            uint16_t sbuf[VCA_SINE_SAMPLE_N];
            for (uint8_t i = 0; i < VCA_SINE_SAMPLE_N; i++) {
                sbuf[i] = ADS8325_Read(&g_ads8325);
            }
            for (int i = 1; i < (int)VCA_SINE_SAMPLE_N; i++) {
                uint16_t k = sbuf[i]; int j = i - 1;
                while (j >= 0 && sbuf[j] > k) { sbuf[j+1] = sbuf[j]; j--; }
                sbuf[j+1] = k;
            }
            /* trimmed mean (양 끝 TRIM_K 절삭) → filtered */
            uint32_t sum = 0;
            for (uint8_t i = VCA_SINE_TRIM_K;
                 i < (uint8_t)(VCA_SINE_SAMPLE_N - VCA_SINE_TRIM_K); i++) sum += sbuf[i];
            uint16_t pos_filt = (uint16_t)(sum
                / (uint32_t)(VCA_SINE_SAMPLE_N - 2U * VCA_SINE_TRIM_K));
            uint16_t pos_raw  = sbuf[VCA_SINE_SAMPLE_N / 2U];  /* median */
            g_vca_pos_raw = pos_raw;
            g_vca_pos_adc = pos_filt;
            g_sm_pos      = pos_filt;

            osDelay(VCA_SINE_CYCLE_MS);
            continue;
        }
#else  /* !VCA_LED_SENSOR_TEST — 기존 closed-loop sine 모드 */
        /* ── 3a. VCA closed-loop sine mode ───────────────────────────────────
         *  새 구조: 50ms 사이클당 한 번 PWM-OFF 윈도우(3ms)에서 ADS8325 직접
         *  읽기로 fresh 위치 캡처 → 47ms 연속 드라이브로 sine 추종.
         *  ① ringdown 1ms → ② SPI burst N=8 중앙값 → ③ 제어 갱신 → ④ 드라이브
         *  드라이브 듀티 94% 로 스프링 복원력 압도, 0.5Hz sine 추종.        */
        if (g_vca_sine_mode) {
            /* ── 진단용 open-loop bidirectional sine ──
             *  복잡한 PI 우회 — sin() 부호에 따라 DIR 토글 + |sin| 비례 duty.
             *  spring 약해도 motor가 양방향으로 active 구동 → 진동 가시화          */
            static uint32_t s_sine_start = 0;
            if (s_sine_start == 0) s_sine_start = now;

            const float t_s     = (float)(now - s_sine_start) * 0.001f;
            const float omega   = 2.0f * 3.14159265f * VCA_SINE_FREQ_HZ;
            const float sin_v   = sinf(omega * t_s);          /* -1 ~ +1 */
            const float duty_max_sine = 0.35f;                /* 최대 인가 듀티 */

            vca_on();
            if (sin_v >= 0.0f) {
                TLE9201_SetDir(DIR_PUSH);
                VCA_SetDuty(sin_v * duty_max_sine);
            } else {
                TLE9201_SetDir(DIR_PULL);
                VCA_SetDuty(-sin_v * duty_max_sine);
            }
            g_motor_active = 1;
            g_duty_dbg = sin_v >= 0.0f ? sin_v * duty_max_sine : -sin_v * duty_max_sine;
            g_dir_dbg  = sin_v >= 0.0f ? 1U : 0U;

            /* ADS8325 위치값 갱신 (plotter용) — burst 후 trimmed mean */
            uint16_t sbuf_s[VCA_SINE_SAMPLE_N];
            for (uint8_t i = 0; i < VCA_SINE_SAMPLE_N; i++) sbuf_s[i] = ADS8325_Read(&g_ads8325);
            for (int i = 1; i < (int)VCA_SINE_SAMPLE_N; i++) {
                uint16_t k = sbuf_s[i]; int j = i - 1;
                while (j >= 0 && sbuf_s[j] > k) { sbuf_s[j+1] = sbuf_s[j]; j--; }
                sbuf_s[j+1] = k;
            }
            uint32_t sum_s = 0;
            for (uint8_t i = VCA_SINE_TRIM_K; i < (uint8_t)(VCA_SINE_SAMPLE_N - VCA_SINE_TRIM_K); i++) sum_s += sbuf_s[i];
            uint16_t pos_s = (uint16_t)(sum_s / (uint32_t)(VCA_SINE_SAMPLE_N - 2U * VCA_SINE_TRIM_K));
            g_vca_pos_adc = pos_s;
            g_vca_pos_raw = pos_s;
            g_sm_pos = pos_s;

            osDelay(VCA_SINE_CYCLE_MS);
            continue;
        }
        /* ── 기존 closed-loop sine 코드 (현재 비활성) ── */
        if (0 && g_vca_sine_mode) {
            static uint32_t s_sine_start = 0;
            static float    s_i_acc      = 0.0f;
            static float    s_u_prev     = 0.0f;
            static uint32_t s_sine_cyc   = 0;
            static float    s_pos_prev_d = 0.0f;   /* KD 용 이전 위치 */
            static bool     s_pos_d_init = false;
            if (s_sine_start == 0) s_sine_start = now;
            s_sine_cyc++;

            /* 주기적 SPI fault clear / 쿨다운 제거 — hspi6 트래픽 제거하여
             *  hspi6 hang 가능성 차단. DIS 토글이 매 사이클 fault 자동 리셋을
             *  처리하므로 별도 SPI 명령 불필요. */

            const float two_pi_f = 2.0f * 3.14159265f * VCA_SINE_FREQ_HZ;
            /* sine 범위: g_vca_adc_home ~ home+8000 (≈1.2mm 진폭)
             *  motor는 DIR_PUSH 한 방향 + spring 복원으로 sine 추종 →
             *  진폭이 너무 크면 saturation. 1.2mm 정도가 적절.                  */
            const float sine_home = (float)VCA_ADC_HOME;
            const float sine_peak = sine_home + 8000.0f;
            const float center    = 0.5f * (sine_peak + sine_home);
            const float amplitude = 0.5f * (sine_peak - sine_home);
            const float pos_range = sine_peak - sine_home;

            g_state_dbg = 5;

            /* ── ① PWM 계속 ON 상태로 ADS8325 샘플링 ──
             *  소음 원인이던 매 사이클 PWM=0 펄스(50Hz 클릭) 제거.
             *  16샘플 SPI burst (~1ms) 동안 PWM 계속 동작 → 코일 전류 끊김 없음.
             *  PWM ripple 은 trim(4,4) + EMA 가 자연스럽게 평균 처리.          */

            uint16_t sbuf[VCA_SINE_SAMPLE_N];
            for (uint8_t i = 0; i < VCA_SINE_SAMPLE_N; i++) {
                sbuf[i] = ADS8325_Read(&g_ads8325);
            }
            /* 정렬 (insertion sort) */
            for (int i = 1; i < (int)VCA_SINE_SAMPLE_N; i++) {
                uint16_t k = sbuf[i]; int j = i - 1;
                while (j >= 0 && sbuf[j] > k) { sbuf[j+1] = sbuf[j]; j--; }
                sbuf[j+1] = k;
            }

            /* Spike/wrap 감지: ADS8325 신호선 EMI 결합 시 garbage(>60000) 발생
             *  → max>55000 인 경우만 spike 로 판정 (실제 wrap)
             *  분산 임계는 제거 — 빠른 모션도 진짜 위치값이므로 보존
             *  (이전 8000 임계가 빠른 하강을 차단해 그래프 진폭 축소시킴)    */
            uint16_t smin = sbuf[0];
            uint16_t smax = sbuf[VCA_SINE_SAMPLE_N - 1U];
            (void)smin;
            bool spike = (smax >= 55000U);

            static float    s_pos_ema = 0.0f;
            static bool     s_ema_init = false;
            uint16_t pos_trimmed;

            if (spike && s_ema_init) {
                /* EMI 노이즈 발견 → 이전 EMA 유지 (motor 멈춤 방지) */
                pos_trimmed = (uint16_t)(s_pos_ema + 0.5f);
                /* EMA 는 갱신 안 함 — 이전 깨끗한 값 보존 */
            } else {
                /* trimmed mean: 양 끝 TRIM_K 개씩 절삭 후 평균 */
                uint32_t sum = 0;
                for (uint8_t i = VCA_SINE_TRIM_K;
                     i < (uint8_t)(VCA_SINE_SAMPLE_N - VCA_SINE_TRIM_K); i++) {
                    sum += sbuf[i];
                }
                pos_trimmed = (uint16_t)(sum
                    / (uint32_t)(VCA_SINE_SAMPLE_N - 2U * VCA_SINE_TRIM_K));

                /* EMA 갱신 (정상 샘플만) */
                if (!s_ema_init) {
                    s_pos_ema = (float)pos_trimmed;
                    s_ema_init = true;
                } else {
                    s_pos_ema += VCA_SINE_POS_EMA_A
                               * ((float)pos_trimmed - s_pos_ema);
                }
            }
            uint16_t pos_clean = (uint16_t)(s_pos_ema + 0.5f);

            /* PWM ON 직전 fresh 위치값 표시 + 제어 입력
             *   raw = trimmed mean (필터 전), filtered = EMA 후 — 디버그 비교용 */
            g_vca_pos_adc = pos_clean;
            g_vca_pos_raw = pos_trimmed;
            g_sm_pos      = pos_clean;
            float pos_f = s_pos_ema;

            /* ── ② sine 타겟 + FF 듀티 (정적 + 속도 FF) ── */
            const float t_s   = (float)(now - s_sine_start) * 0.001f;
            const float phase = two_pi_f * t_s - (0.5f * 3.14159265f);
            const float target = center + amplitude * sinf(phase);
            const float vel_target = amplitude * two_pi_f * cosf(phase); /* ADC/s */

            /* 정적 FF: 타겟 위치 → HOME→PEAK 정적 평형 듀티 */
            float norm_t = (target - sine_home) / pos_range;
            if (norm_t < 0.0f) norm_t = 0.0f;
            if (norm_t > 1.0f) norm_t = 1.0f;
            float u_ff_static = VCA_SINE_FF_HOME_DUTY
                              + (VCA_SINE_FF_PEAK_DUTY - VCA_SINE_FF_HOME_DUTY) * norm_t;

            /* 속도 FF: sine 미분에 비례하여 duty 가/감산 → 추종 lag 보정 */
            float u_ff_vel = vel_target * VCA_SINE_VEL_FF_GAIN;

            float u_ff = u_ff_static + u_ff_vel;

            /* ── ③ PI 보정 (고정 게인) ───────────────────────────────── */
            float err_f = target - pos_f;
            g_vca_err = (int32_t)err_f;

            float err_for_pi = err_f;
            if (fabsf(err_for_pi) < VCA_SINE_ERR_DEADZONE) err_for_pi = 0.0f;

            s_i_acc += err_for_pi * VCA_SINE_KI;
            if (s_i_acc >  VCA_SINE_I_LIMIT) s_i_acc =  VCA_SINE_I_LIMIT;
            if (s_i_acc < -VCA_SINE_I_LIMIT) s_i_acc = -VCA_SINE_I_LIMIT;

            /* KD 항: 비대칭 (하강만 댐핑, 상승은 자유)
             *  하강 vel<0: +duty 가산 → motor 브레이크 → mech bottom 충돌 방지
             *  상승 vel>0: KD=0 → motor 자유 → peak 도달성 보장
             *  vel_meas 에 EMA 적용해 ADC 미분 노이즈 감쇠                     */
            static float s_vel_meas_ema = 0.0f;
            float vel_inst = 0.0f;
            if (!s_pos_d_init) {
                s_pos_prev_d = pos_f;
                s_vel_meas_ema = 0.0f;
                s_pos_d_init = true;
            } else {
                vel_inst = (pos_f - s_pos_prev_d)
                         * (1000.0f / (float)VCA_SINE_CYCLE_MS); /* ADC/s */
                s_pos_prev_d = pos_f;
                s_vel_meas_ema += VCA_SINE_VEL_EMA_A
                                  * (vel_inst - s_vel_meas_ema);
            }
            /* 위치 기반 KD 게이트 — smoothstep (3x²-2x³) 으로 부드러운 전환
             *  선형 ramp 의 끝점 미분 불연속 → 머뭇거림 → smoothstep 으로 해결  */
            float kd_gain;
            if (pos_f >= VCA_SINE_KD_GATE_HI) {
                kd_gain = 0.0f;
            } else if (pos_f <= VCA_SINE_KD_GATE_LO) {
                kd_gain = 1.0f;
            } else {
                float x = (VCA_SINE_KD_GATE_HI - pos_f)
                        / (VCA_SINE_KD_GATE_HI - VCA_SINE_KD_GATE_LO);
                kd_gain = x * x * (3.0f - 2.0f * x);  /* smoothstep */
            }
            float u_kd = (s_vel_meas_ema < 0.0f)
                       ? (-s_vel_meas_ema * VCA_SINE_KD * kd_gain)
                       : 0.0f;

            float u_pi = err_for_pi * VCA_SINE_KP + s_i_acc + u_kd;

            /* ── ④ 합산 + 단방향 클램프 + 슬루 ──
             *  단방향(PUSH only): u ≥ 0 만 허용 — 하강은 스프링이 처리
             *  → 방향 반전 시 발생하던 back-EMF 스파이크/OC 트립 원천 차단      */
            float u_raw = u_ff + u_pi;
            if (u_raw < 0.0f)              u_raw = 0.0f;
            if (u_raw > VCA_SINE_DUTY_MAX) u_raw = VCA_SINE_DUTY_MAX;

            float du = u_raw - s_u_prev;
            if (du >  VCA_SINE_U_SLEW) du =  VCA_SINE_U_SLEW;
            if (du < -VCA_SINE_U_SLEW) du = -VCA_SINE_U_SLEW;
            float u_slewed = s_u_prev + du;
            if (u_slewed < 0.0f) u_slewed = 0.0f;
            s_u_prev = u_slewed;

            /* 출력 EMA: 듀티 step 자체를 매끄럽게 → acoustic 고주파 제거 */
            static float s_u_ema = 0.0f;
            static bool  s_u_ema_init = false;
            if (!s_u_ema_init) { s_u_ema = u_slewed; s_u_ema_init = true; }
            else { s_u_ema += VCA_SINE_U_EMA_A * (u_slewed - s_u_ema); }
            float u = s_u_ema;
            if (u < 0.0f) u = 0.0f;

            /* ── ⑤ 직접 듀티 인가 (Loop1 우회) ──
             *  Cascade 전류 제어는 부호/감도 검증 후 재시도 예정.
             *  현재는 직접 듀티로 안정 동작 우선 — 강성은 KP/KI/FF 로.       */
            vca_on();
            TLE9201_SetDir(DIR_PUSH);
            if (s_l1_enable) Loop1_Stop();
            VCA_SetDuty(u);
            g_motor_active = 1;
            g_duty_dbg = u;
            g_dir_dbg  = 1U;

            /* 사이클 전체 osDelay (OFF 윈도우 제거됨) */
            osDelay(VCA_SINE_CYCLE_MS);
            continue;
        }
#endif /* VCA_LED_SENSOR_TEST */

        /* ════════════════════════════════════════════════════════════════
         *  3b. Production 상태 머신 (sine_mode=0 일 때, default)
         *      • OP_STANDBY (g_gui_ready=0): depth 슬라이더 추종 (약한 force)
         *      • OP_READY_IDLE (READY + 버튼 OFF): HOME 위치 유지
         *      • OP_READY_PUSH (READY + 버튼 ON, 미도달): 강력 push
         *      • OP_READY_HOLD (도달): 최소 전류로 위치 유지 (발열 감소)
         *      • 버튼 OFF → IDLE: 즉시 HOME 복귀 (스프링 자연 복원)
         *  RS485 g_vca_state 매핑: STANDBY=0, IDLE=4, PUSH=1, HOLD=2
         * ═══════════════════════════════════════════════════════════════ */
        {
            /* ── RETURN-with-BRAKE 파라미터 (HOLD/PUSH→IDLE 시, 홈 충돌음 제거) ──
             *  전략: 제어 루프 없이 상수 FF_HOME duty 만 인가 (진동 원천 차단)
             *        - duty = VCA_SINE_FF_HOME_DUTY (≈0.11) = 홈 위치 정적 평형 듀티
             *        - 평형점 = 홈 → 니들이 자연스럽게 홈 평형으로 수렴하며 감속
             *        - 깊은 위치: 스프링(0.26)>모터(0.11) → 홈 방향 가속
             *        - 홈 도착: 모터=스프링 평형 → 잔여 운동에너지 자연 소산
             *        - PWM 11% 의 LOW 구간(89%)은 low-side brake → 감쇠 추가 효과
             *        - 듀티가 절대 변동 안 함 → 진동/발진 원인 자체 없음
             *  단위: 위치=ADC(VCA_ADC_HOME=3300, /mm=10200)
             *  ─────────────────────────────────────────────────────────── */
            #define RETURN_PULL_DUTY            (0.18f)   /* 즉각 PULL duty — 스위치 OFF 시 능동 retract */
            #define RETURN_PULL_END_BAND_ADC    (5000)    /* PULL 종료 판정 (≈0.5mm) — BRAKE 단계 진입 */
            #define RETURN_PULL_TIMEOUT_MS      (400U)    /* PULL 단계 안전 타임아웃 */
            #define RETURN_BRAKE_DUTY           (0.0f)    /* BRAKE: PWM=0 → 저측 브레이크 (back-EMF 감쇠), 능동 PUSH 없음 — 강한 스프링이 자체 감속 */
            #define RETURN_HOME_BAND_ADC        (1500)    /* BRAKE 종료 판정 (≈0.15mm) — SEAT 단계 진입 */
            #define RETURN_BRAKE_TIMEOUT_MS_OP  (300U)    /* BRAKE 단계 안전 타임아웃 (PULL 후 짧게) */
            #define RETURN_SEAT_DUTY            (0.07f)   /* 안착 PULL duty — 정마찰 극복용 약한 힘 */
            #define RETURN_SEAT_MS              (200U)    /* 안착 단계 지속시간 — 슬램 없이 home 밀착 */

            enum { OP_STANDBY = 0, OP_READY_IDLE, OP_READY_PUSH, OP_READY_HOLD };
            static uint8_t s_op_state    = OP_STANDBY;
            static float   s_u_prev_op   = 0.0f;
            static float   s_u_ema_op    = 0.0f;
            static bool    s_u_init_op   = false;
            static float   s_pos_ema_op  = 0.0f;
            static bool    s_pos_init_op = false;

            /* RETURN 단계 상태:
             *   1 = PULL_FAST (DIR_PULL 0.18 — 즉각 능동 retract, 빠른 home 접근)
             *   2 = BRAKE     (DIR_PUSH 0.13 — home 직전 감속, 슬램 방지)
             *   3 = SEAT      (DIR_PULL 0.07 — home 정마찰 극복, 완전 밀착)
             *   0 = OFF       (안착 완료, 모터 완전 OFF)                          */
            static uint8_t  s_ret_brake_active = 0;
            static uint32_t s_ret_pull_t0      = 0;
            static uint32_t s_ret_brake_t0     = 0;
            static uint32_t s_ret_seat_t0      = 0;

            /* PUSH BURST 상태 — to_push 시각 기록, 시간 기반 burst 종료 */
            static uint32_t s_push_t0_op = 0;

            /* @L1 복귀 요청 처리 — 1-pin 진입/복귀는 "화면 모드 + 니들 home"
             *  만 하면 충분. 하드웨어 재초기화(VCA_Init/TIM8/SPI) 는 transient
             *  를 만들어 첫 PUSH 동작에 영향을 주므로 제거.
             *  IDLE 상태 진입만으로도 모터 off → 스프링이 home 으로 복귀.    */
            if (g_op_reset_request) {
                g_op_reset_request = 0;
                s_op_state    = OP_READY_IDLE;
                s_u_prev_op   = 0.0f;
                s_u_ema_op    = 0.0f;
            }

            /* ── ADS8325 fresh 위치 읽기 (sine 와 동일 trim+EMA+spike 보호) ── */
            uint16_t sbuf[VCA_SINE_SAMPLE_N];
            for (uint8_t i = 0; i < VCA_SINE_SAMPLE_N; i++) {
                sbuf[i] = ADS8325_Read(&g_ads8325);
            }
            for (int i = 1; i < (int)VCA_SINE_SAMPLE_N; i++) {
                uint16_t k = sbuf[i]; int j = i - 1;
                while (j >= 0 && sbuf[j] > k) { sbuf[j+1] = sbuf[j]; j--; }
                sbuf[j+1] = k;
            }
            /* Trim mean — 양 끝 K개 절삭 후 평균. trim 자체가 outlier 제거하므로
             *  spike rejection 추가 안 함 (이전 spike check 가 EMA stuck 유발) */
            uint32_t sum_op = 0;
            for (uint8_t i = VCA_SINE_TRIM_K;
                 i < (uint8_t)(VCA_SINE_SAMPLE_N - VCA_SINE_TRIM_K); i++) {
                sum_op += sbuf[i];
            }
            uint16_t pos_trim_op = (uint16_t)(sum_op
                / (uint32_t)(VCA_SINE_SAMPLE_N - 2U * VCA_SINE_TRIM_K));

            /* trim mean 이 valid range 밖이면 EMA 유지 + init 도 막음
             *  (이전: init 시 garbage 로 EMA 시작되어 stuck 가능)
             *  임계값 50000→55000: depth 1.1 등에서 garbage 판정으로 EMA stuck
             *  되는 사례 완화. mech stop 실측이 49-51k 이라 이 부근은 valid.   */
            if (pos_trim_op > 55000U) {
                /* garbage — init 보류, 기존 EMA 유지 (init=false 유지) */
            } else if (!s_pos_init_op) {
                s_pos_ema_op = (float)pos_trim_op;
                s_pos_init_op = true;
            } else {
                s_pos_ema_op += VCA_SINE_POS_EMA_A
                              * ((float)pos_trim_op - s_pos_ema_op);
            }
            uint16_t pos_op = (uint16_t)(s_pos_ema_op + 0.5f);
            g_vca_pos_adc = pos_op;
            g_vca_pos_raw = pos_trim_op;
            g_sm_pos      = pos_op;
            float pos_f_op = s_pos_ema_op;

            /* foot_mode auto-reset 제거 — 5초 timeout 이 foot mode 활성 중에도
             *  pedal 안 누르면 reset 시켜서 HP sw 가 잘못 작동하게 만들었음.
             *  1-pin 복구는 이미 @L1 의 hardware reset 으로 처리됨.            */

            /* 상태 변화 감지: 변경 시 production state 리셋 (오작동 방지) */
            if (g_main_state_changed) {
                g_main_state_changed = 0;
                /* state machine 자체는 in_push edge 로 재진입 */
            }

            /* ── 깊이 → ADC target 변환 (측정값 교정 LUT 적용) ── */
            float depth_user = g_needle_depth_mm;
            if (depth_user < VCA_DEPTH_MM_MIN) depth_user = VCA_DEPTH_MM_MIN;
            if (depth_user > VCA_DEPTH_MM_MAX) depth_user = VCA_DEPTH_MM_MAX;
            float depth_mm = depth_correction(depth_user);    /* user → motor 내부 */
            float target_depth_adc = (float)VCA_ADC_HOME + depth_mm * (float)VCA_ADC_PER_MM;

            /* ── Target trajectory ramp 제거됨 ──
             *  과거 cascaded ramp + KP 체이스 구조는 judder 원인 → 거리 기반
             *  open-loop PUSH 프로필로 대체. (s_target_ramp / TARGET_RAMP_RATE
             *  더 이상 사용 안 함.)                                              */

            /* ── 속도 추정 (상태 머신 전에 계산 — HOLD 진입 조건 사용용) ── */
            static float s_pos_prev_op_pre = 0.0f;
            static float s_vel_ema_op      = 0.0f;
            static bool  s_vel_init_op_pre = false;
            float vel_inst_pre;
            if (!s_vel_init_op_pre) {
                s_pos_prev_op_pre = pos_f_op;
                s_vel_ema_op      = 0.0f;
                vel_inst_pre      = 0.0f;
                s_vel_init_op_pre = true;
            } else {
                vel_inst_pre      = (pos_f_op - s_pos_prev_op_pre)
                                  * (1000.0f / (float)VCA_SINE_CYCLE_MS);
                s_pos_prev_op_pre = pos_f_op;
                s_vel_ema_op     += VCA_SINE_VEL_EMA_A * (vel_inst_pre - s_vel_ema_op);
            }

            /* ── 상태 결정 ────────────────────────────────────────────────
             *  핵심 robust 로직: 스위치 누름은 항상 작업 의도 = 자동 Ready 복원
             *  → 1-pin mode 후 Standby 로 stuck 되어도 스위치 누르면 즉시 동작  */
            static bool s_prev_in_push_op = false;
            bool rising_edge_op = (in_push && !s_prev_in_push_op);
            s_prev_in_push_op = in_push;

            /* Rising edge: 스위치 누름 = 작업 의도 — 단, READY 자동 복원은 제거
             *  사용자 요구: STBY 에서는 스위치 무시. TREAT 상태에서만 동작.
             *  (g_hp_locked 만 해제: 1-pin mode 등 stuck 회피용 유지)             */
            if (rising_edge_op) {
                g_hp_locked = 0;
            }

            uint8_t prev_state = s_op_state;
            uint8_t next_state;
            if (g_hp_locked) {
                /* 1-pin mode 등 HP locked 상태 → motor 강제 OFF (spring 이 home 으로) */
                next_state = OP_READY_IDLE;
            } else if (!g_gui_ready) {
                next_state = OP_STANDBY;  /* depth tracking */
            } else if (in_push) {
                /* PUSH → HOLD 전환은 시간 기반 (위치/속도 피드백 사용 안함)
                 *  PUSH 듀티 프로필이 끝나는 시점 (PUSH_TOTAL_MS) 에 HOLD 진입.
                 *  → position flicker 로 인한 PUSH↔HOLD 토글 차단.                 */
                #define PUSH_TOTAL_MS  (300U)
                if (s_op_state == OP_READY_HOLD) {
                    next_state = OP_READY_HOLD;
                } else if (s_op_state == OP_READY_PUSH &&
                           s_push_t0_op != 0 &&
                           (now - s_push_t0_op) >= PUSH_TOTAL_MS) {
                    next_state = OP_READY_HOLD;
                } else {
                    next_state = OP_READY_PUSH;
                }
            } else {
                next_state = OP_READY_IDLE;
            }

            /* HOLD → IDLE 전환 시 RS485 v=3(ret) 잠시 표시 */
            bool transitioning_return = (prev_state == OP_READY_HOLD &&
                                         next_state == OP_READY_IDLE);

            /* PUSH → HOLD 전환: 듀티 즉시 감소 (overshoot 방지) */
            bool push_to_hold = (prev_state == OP_READY_PUSH &&
                                 next_state == OP_READY_HOLD);

            /* → IDLE/STANDBY 전환: 즉시 듀티 0 reset (모터 free → 스프링 home) */
            bool to_idle = (prev_state != OP_READY_IDLE &&
                            prev_state != OP_STANDBY &&
                           (next_state == OP_READY_IDLE ||
                            next_state == OP_STANDBY));

            /* IDLE/STANDBY → PUSH 전환 검출 (트래젝토리 램프 init 용) */
            bool to_push = ((prev_state == OP_READY_IDLE || prev_state == OP_STANDBY)
                            && next_state == OP_READY_PUSH);
            s_op_state = next_state;

            /* Target ramp 블록 제거 — target_depth_adc 직접 사용 */

            /* RS485 g_vca_state 갱신 (Main board 응답용) */
            switch (s_op_state) {
                case OP_STANDBY:    g_vca_state = 0; break;  /* idle */
                case OP_READY_IDLE: g_vca_state = transitioning_return ? 3 : 4;
                                                              break;  /* ret/home */
                case OP_READY_PUSH: g_vca_state = 1; break;  /* fwd */
                case OP_READY_HOLD: g_vca_state = 2; break;  /* target */
            }
            g_state_dbg = s_op_state;

            /* ── 상태별 target / gains / duty 직접 결정 ──────────────────
             *  PUSH: bang-bang 개방루프 (KP 피드백이 진동 원인 → 제거)
             *  IDLE: motor 완전 OFF (스프링이 즉시 HOME 복귀)
             *  STANDBY/HOLD: FF + 약한 PI (정적 위치 유지)               */
            float target_pos_op;
            float duty_max_op;
            float slew_op;
            float ema_a_op;
            float kp_op;

            switch (s_op_state) {
                default:
                case OP_STANDBY:
                    /* STBY: depth 슬라이더 변경 후에도 모터는 HOME 유지.
                     *  자동 push 하면 depth 별 STBY duty 한계 차로 인해 스위치
                     *  눌렀을 때 이동량이 들쭉날쭉 보임 → IDLE 과 동일하게 free  */
                    target_pos_op = (float)VCA_ADC_HOME;
                    kp_op       = 0.0f;
                    duty_max_op = 0.0f;
                    slew_op     = 0.20f;
                    ema_a_op    = 0.95f;
                    break;
                case OP_READY_IDLE:
                    target_pos_op = (float)VCA_ADC_HOME;
                    kp_op       = 0.0f;        /* 피드백 없음 */
                    duty_max_op = 0.0f;        /* motor 완전 OFF — 스프링이 home 복귀 */
                    slew_op     = 0.20f;       /* 빠른 듀티 drop (즉시 retract) */
                    ema_a_op    = 0.95f;       /* 즉각 반응 */
                    break;
                case OP_READY_PUSH:
                    /* PUSH — open-loop 거리 프로필 (KP 체이스 제거)
                     *  duty 결정은 u_raw 단계에서 dist 기반 (BURST → taper → near)
                     *  slew/ema 는 약간 빠르게: snap 한 가속 + 평활 균형
                     *  target_pos_op 은 ff_static_op 정규화 계산용으로만 사용     */
                    target_pos_op = target_depth_adc;
                    kp_op       = 0.0f;        /* 미사용 (u_raw 가 KP 안 씀) */
                    duty_max_op = 1.00f;
                    slew_op     = 0.05f;
                    ema_a_op    = 0.35f;
                    break;
                case OP_READY_HOLD:
                    /* HOLD — target 정착 유지 (ff_static + 약한 P + D + deadzone)
                     *  target_pos = target_depth_adc 직결                            */
                    target_pos_op = target_depth_adc;
                    kp_op       = 0.0f;        /* HOLD u_raw 에서 HOLD_KP_STRONG 사용 */
                    duty_max_op = 1.00f;
                    slew_op     = 0.02f;
                    ema_a_op    = 0.30f;
                    break;
            }

            /* ── 정적 FF (작은 base) ──
             *  target_pos 에 따라 0.05 ~ 0.25 base duty.
             *  pure-P 는 undershoot 발생 (err=0 시 u=0 → spring 후퇴) →
             *  적당한 base duty 로 motor 가 항상 약간의 forward force 유지              */
            #define FF_BASE_HOME  (0.05f)
            #define FF_BASE_PEAK  (0.25f)
            float full_range_op = (float)VCA_ADC_MAX_SAFE - (float)VCA_ADC_HOME;
            float norm_t_op = (target_pos_op - (float)VCA_ADC_HOME) / full_range_op;
            if (norm_t_op < 0.0f) norm_t_op = 0.0f;
            if (norm_t_op > 1.0f) norm_t_op = 1.0f;
            float ff_static_op = FF_BASE_HOME + (FF_BASE_PEAK - FF_BASE_HOME) * norm_t_op;

            /* ── 위치 오차 ── */
            float err_op = target_pos_op - pos_f_op;
            g_vca_err = (int32_t)err_op;

            /* ── u_raw 결정 — 단순 2상태 (PUSH proportional / HOLD pure FF) ──
             *  PUSH: u_raw = ff_static + KP × err
             *        모터가 target 지나면 즉시 HOLD 로 전환되므로 추가 분기 불필요.
             *  HOLD: u_raw = ff_static (피드백 없음 → 진동 절대 불가)
             *        Spring + FF 평형 위치에서 자연 정착, 위치 변동에 반응 안 함.
             *  IDLE/STANDBY: motor off                                          */
            float u_raw_op;
            if (s_op_state == OP_READY_IDLE || s_op_state == OP_STANDBY) {
                u_raw_op = 0.0f;
            } else if (s_op_state == OP_READY_PUSH) {
                /* PUSH — 100% 시간 기반 open-loop 듀티 프로필
                 *  pos / vel 어떤 피드백도 사용 안함. s_push_t0_op 타이머만 참조.
                 *  Phase A (0 ~ DRIVE_MS): PUSH_DRIVE_DUTY 로 가속
                 *  Phase B (DRIVE_MS ~ DRIVE_MS+TAPER_MS): DRIVE → HOLD 듀티
                 *  Phase C (그 이후): HOLD 듀티 — 곧 HOLD 상태로 인계됨            */
                #define PUSH_DRIVE_DUTY   (0.40f)
                #define PUSH_HOLD_DUTY    (0.22f)   /* HOLD 와 동일 값 */
                #define PUSH_DRIVE_MS     (100U)
                #define PUSH_TAPER_MS     (200U)

                uint32_t elapsed = (s_push_t0_op != 0) ? (now - s_push_t0_op) : 0;
                float u_profile;
                if (elapsed < PUSH_DRIVE_MS) {
                    u_profile = PUSH_DRIVE_DUTY;
                } else if (elapsed < (PUSH_DRIVE_MS + PUSH_TAPER_MS)) {
                    float t = (float)(elapsed - PUSH_DRIVE_MS)
                            / (float)PUSH_TAPER_MS;
                    u_profile = PUSH_DRIVE_DUTY
                              + t * (PUSH_HOLD_DUTY - PUSH_DRIVE_DUTY);
                } else {
                    u_profile = PUSH_HOLD_DUTY;
                }
                u_raw_op = u_profile;
                if (u_raw_op > duty_max_op) u_raw_op = duty_max_op;
                if (u_raw_op < 0.0f) u_raw_op = 0.0f;
            } else {
                /* HOLD — 모든 피드백 제거. 고정 듀티 (PUSH_HOLD_DUTY 와 동일) */
                u_raw_op = PUSH_HOLD_DUTY;
                if (u_raw_op > duty_max_op) u_raw_op = duty_max_op;
                if (u_raw_op < 0.0f) u_raw_op = 0.0f;
            }
            if (u_raw_op < 0.0f) u_raw_op = 0.0f;

            /* PUSH→HOLD 전환: ramp가 이미 target에서 정착 → 자연 transition (강제 drop 불필요) */
            /* →IDLE 전환 즉시 듀티 0 → 빠른 retract (slew 우회) */
            if (to_idle) {
                s_u_prev_op = 0.0f;
                s_u_ema_op  = 0.0f;
                /* RETURN 진입 — Phase 1 PULL_FAST: 즉각 능동 retract 시작 */
                s_ret_brake_active = 1;
                s_ret_pull_t0      = now;
            }
            /* IDLE/STANDBY → PUSH 진입: BURST 시작 시각 기록 + duty 0 부터 */
            if (to_push) {
                s_u_prev_op = 0.0f;
                s_u_ema_op  = 0.0f;
                s_u_init_op = true;
                s_ret_brake_active = 0;  /* 다시 작업 진입 → 브레이크 강제 해제 */
                s_push_t0_op = now;       /* BURST 타이머 시작 */
            }
            /* @L1 hard-reset, lock 등에서도 브레이크 잔류 방지 */
            if (g_hp_locked) {
                s_ret_brake_active = 0;
            }

            /* ── Slew + 출력 EMA (sine 모드와 동일 — 모든 상태 통과) ── */
            float du_op = u_raw_op - s_u_prev_op;
            if (du_op >  slew_op) du_op =  slew_op;
            if (du_op < -slew_op) du_op = -slew_op;
            float u_slewed_op = s_u_prev_op + du_op;
            if (u_slewed_op < 0.0f) u_slewed_op = 0.0f;
            s_u_prev_op = u_slewed_op;

            if (!s_u_init_op) { s_u_ema_op = u_slewed_op; s_u_init_op = true; }
            else { s_u_ema_op += ema_a_op * (u_slewed_op - s_u_ema_op); }
            float u_op = s_u_ema_op;
            if (u_op < 0.0f) u_op = 0.0f;

            /* ── 직접 듀티 인가 (sine 모드 검증된 기법 적용) ──────────────
             *  Loop1 cascade 는 4A 전류 → 8g 가속도 → 10ms 동안 4mm 이동
             *  → mechanical stop 슬램. Loop1 우회하고 sine 1Hz 추종에서 검증된
             *  파라미터(낮은 DUTY_MAX, 느린 SLEW, KD 속도 댐핑) 사용.
             *  IDLE/STANDBY: motor free → home 복귀.
             *  PUSH/HOLD:    FF + KP×err - KD×vel + slew + EMA 직접 인가.       */
            if (s_op_state == OP_READY_IDLE || s_op_state == OP_STANDBY) {
                if (s_ret_brake_active == 1) {
                    /* ── Phase 1 PULL_FAST: 즉각 능동 retract ──
                     *  - 스위치 OFF 즉시 DIR_PULL + 0.18 duty 인가
                     *  - 스프링 + 모터가 동시에 home 방향 → 빠른 초기 가속
                     *  - 종료: pos ≤ HOME+5000 (≈0.5mm) 또는 400ms 타임아웃
                     *          → BRAKE 단계 진입 (높은 속도를 home 직전에 흡수)
                     *  ─────────────────────────────────────────────────────── */
                    uint32_t pt = now - s_ret_pull_t0;
                    bool pull_timeout = (pt >= RETURN_PULL_TIMEOUT_MS);
                    bool near_brake   = (pos_f_op <= (float)(VCA_ADC_HOME + RETURN_PULL_END_BAND_ADC));

                    if (near_brake || pull_timeout) {
                        s_ret_brake_active = 2;
                        s_ret_brake_t0     = now;
                    } else {
                        vca_on();
                        TLE9201_SetDir(DIR_PULL);
                        if (s_l1_enable) Loop1_Stop();
                        VCA_SetDuty(RETURN_PULL_DUTY);
                        g_motor_active = 1;
                        g_duty_dbg = RETURN_PULL_DUTY;
                        g_dir_dbg  = 0U;
                    }
                }
                if (s_ret_brake_active == 2) {
                    /* ── Phase 2 BRAKE: PUSH duty 로 home 직전 감속 ──
                     *  - PULL_FAST 단계에서 가속된 속도를 home 충돌 전 흡수
                     *  - DIR_PUSH + 0.13 duty: 스프링 거스르며 능동 감속
                     *  - 종료: pos ≤ HOME+1500 (≈0.15mm) 또는 300ms 타임아웃
                     *          → SEAT 단계 진입 (완전 밀착 보장)
                     *  ─────────────────────────────────────────────────────── */
                    uint32_t bt = now - s_ret_brake_t0;
                    bool brake_timeout = (bt >= RETURN_BRAKE_TIMEOUT_MS_OP);
                    bool near_home     = (pos_f_op <= (float)(VCA_ADC_HOME + RETURN_HOME_BAND_ADC));

                    if (near_home || brake_timeout) {
                        s_ret_brake_active = 3;
                        s_ret_seat_t0      = now;
                    } else {
                        vca_on();
                        TLE9201_SetDir(DIR_PUSH);
                        if (s_l1_enable) Loop1_Stop();
                        VCA_SetDuty(RETURN_BRAKE_DUTY);
                        g_motor_active = 1;
                        g_duty_dbg = RETURN_BRAKE_DUTY;
                        g_dir_dbg  = 1U;
                    }
                }
                if (s_ret_brake_active == 3) {
                    /* ── Phase 3 SEAT: 약한 PULL duty 로 home 정마찰 극복 ──
                     *  - 속도가 BRAKE 로 감속됨 → 슬램 없이 부드럽게 밀착
                     *  - DIR_PULL + 0.07 duty: 스프링 보조하여 home 스토퍼 안착
                     *  - 200ms 후 모터 완전 OFF
                     *  ─────────────────────────────────────────────────────── */
                    uint32_t st = now - s_ret_seat_t0;
                    if (st >= RETURN_SEAT_MS) {
                        s_ret_brake_active = 0;
                        if (s_l1_enable) Loop1_Stop();
                        VCA_SetDuty(0.0f);
                        vca_off();
                        g_motor_active = 0;
                        g_duty_dbg = 0.0f;
                        g_dir_dbg  = 0U;
                    } else {
                        vca_on();
                        TLE9201_SetDir(DIR_PULL);
                        if (s_l1_enable) Loop1_Stop();
                        VCA_SetDuty(RETURN_SEAT_DUTY);
                        g_motor_active = 1;
                        g_duty_dbg = RETURN_SEAT_DUTY;
                        g_dir_dbg  = 0U;
                    }
                }
                if (s_ret_brake_active == 0) {
                    /* 평상 IDLE/STANDBY — 기존 동작 (모터 완전 OFF) */
                    if (s_l1_enable) Loop1_Stop();
                    VCA_SetDuty(0.0f);
                    vca_off();
                    g_motor_active = 0;
                    g_duty_dbg = 0.0f;
                    g_dir_dbg  = 0U;
                }
            } else {
                vca_on();
                TLE9201_SetDir(DIR_PUSH);
                if (s_l1_enable) Loop1_Stop();
                VCA_SetDuty(u_op);
                g_motor_active = 1;
                g_duty_dbg = u_op;
                g_dir_dbg  = 1U;
            }

            /* [VCA] 디버그 출력 제거 — ADS8325 plotter 데이터만 UART1 단독 사용 */

            osDelay(VCA_SINE_CYCLE_MS);
            continue;
        }

        /* ── 3. 스위치 에지 이벤트 (legacy — 도달 안 됨) ────────────────── */
        if (in_push != last_in_push) {
            last_in_push = in_push;
            if (in_push && g_gui_ready) {
                /* 스위치 ON: READY 상태에서만 PROBE 진입 (STBY면 무시) */
                s_push_t0   = now;
                s_push_pos0 = (uint16_t)pos;
                PID_Reset(&s_pid_move);
                PID_Reset(&s_pid_hold);
                g_load_detected = 0U;
                s_push_i_ma = PUSH_I_NO_LOAD;
                vca_on();
                TLE9201_SetDir(DIR_PUSH);
                {   float pd = depth_to_duty(g_needle_depth_mm);
                    if (pd > PROBE_DUTY) pd = PROBE_DUTY;
                    VCA_SetDuty(pd);
                }
                s_probe_t0 = now;
                s_adc_at_home = g_vca_pos_adc;  /* home ADC 캡처 */
                g_motor_active = 1;  /* PROBE 재진입 → ADC 동결 */
                s_state    = VCA_PROBE_LOAD;
                s_release_block_until = now + RELEASE_GRACE_MS;
                release_t0 = 0U;
                stable_t0  = 0;
            }
            /* PULL(스위치 OFF)은 아래 Release Gate 처리 */
        }

        /* ── 4. Release Gate ────────────────────────────────────────────── */
        float u = 0.0f;  (void)u;

        if (!in_push && s_state == VCA_HOME_OFF) {
            osDelay(VCA_PID_DT_MS);
            continue;
        }

        if (!in_push) {
            /* 스위치 OFF: Loop1 즉시 차단, RETURN 상태로 전환 */
            if (release_t0 == 0U) {
                release_t0 = now;
                Loop1_Stop();
                VCA_SetDuty(0.0f);
                g_motor_active = 0;  /* 모터 정지 → ADC 동결 해제 */
                vca_on();  /* 하드 리밋이 DIS=HIGH 했을 수 있으므로 재활성화 */
                PID_Reset(&s_pid_move);
                PID_Reset(&s_pid_hold);
                s_ret_phase   = 0;      /* PULL 추출 단계로 진입 */
                s_ret_ramp_t0 = 0;
                s_state = VCA_RETURN_HOME;
            }
            /* HOME 착륙 완료(RETURN 상태에서만) 또는 타임아웃 1.5s */
            bool pos_at_home = (s_state == VCA_RETURN_HOME) &&
                                ((int32_t)pos <= (int32_t)RETURN_STOP_POS);
            /* 위치 도달 + 속도 거의 0 + 쿠션 시간 경과 후에만 OFF */
            bool vel_settled = (s_vel_lp > -2.0f && s_vel_lp < 2.0f);
            static uint32_t s_home_arrive_t = 0;
            if (pos_at_home && vel_settled) {
                if (s_home_arrive_t == 0) s_home_arrive_t = now;
            } else {
                s_home_arrive_t = 0;
            }
            bool home_reached = (s_home_arrive_t != 0) &&
                                ((now - s_home_arrive_t) >= RETURN_CUSHION_MS);
            bool timeout = ((now - release_t0) >= 3000U);
            if (home_reached || timeout) {
                VCA_SetDuty(0.0f);
                vca_off();
                s_state    = VCA_HOME_OFF;
                release_t0 = 0U;
                s_home_arrive_t = 0;
            }
            osDelay(VCA_PID_DT_MS);
            continue;
        } else {
            release_t0 = 0U;
        }

        /* ── 5. 상태 머신 ──────────────────────────────────────────────── */
        switch (s_state)
        {

        /* ══ 1a. PROBE: 부하 판별 ════════════════════════════════════════
         *  PROBE_DUTY 로 천천히 PUSH하면서 g_i_meas_mA 를 읽어
         *  PROBE_CURRENT_TH 이상이면 피부 있음(유부하) 판정.
         *  PROBE_ADC_END(1mm) 도달 또는 타임아웃 시 MOVE로 전환.
         *
         *  ★ PROBE_CURRENT_TH 튜닝 ★
         *  무부하 상태에서 전류값을 읽어 확인 후, 그 값의 2~3배로 설정.
         * ═══════════════════════════════════════════════════════════════*/
        case VCA_PROBE_LOAD: {
            g_state_dbg = 1;
            TLE9201_SetDir(DIR_PUSH);

            /* PROBE duty = min(PROBE_DUTY, 선형화 duty)
             *  얕은 목표: 선형화 duty < PROBE_DUTY → 선형화 duty 사용
             *  깊은 목표: 선형화 duty > PROBE_DUTY → PROBE_DUTY 사용
             *  → 오버슈트 방지: PROBE가 HOLD보다 강하게 밀지 않음 */
            {
                float pd = depth_to_duty(g_needle_depth_mm);
                if (pd > PROBE_DUTY) pd = PROBE_DUTY;
                VCA_SetDuty(pd);
            }
            g_motor_active = 1;  /* PROBE 중 ADC 동결 유지 */

            /* 전류 측정으로 부하 판별 (전류 ADC는 EMI 영향 적음) */
            int32_t i_now = g_i_meas_mA;
            if (i_now > (int32_t)PROBE_CURRENT_TH) {
                /* 유부하: 피부 감지 → 높은 전류로 관통 */
                g_load_detected = 1U;
                s_push_i_ma = PUSH_I_LOADED;
            }

            /* PROBE 완료: 목표 깊이에 비례한 시간 (최소 100ms 부하감지)
             *  G10(얕음): 100ms → 과도 push 방지
             *  G30(깊음): ~225ms → 충분한 이동 */
            #define PROBE_MIN_MS  50U
            uint32_t probe_limit;
            {
                int32_t tgt = DEPTH_MM_TO_ADC(g_needle_depth_mm);
                if (tgt < (int32_t)VCA_ADC_HOME) tgt = (int32_t)VCA_ADC_HOME;
                probe_limit = (uint32_t)(
                    (uint32_t)(tgt - (int32_t)VCA_ADC_HOME) * PROBE_TIMEOUT_MS
                    / (uint32_t)(VCA_ADC_MAX_SAFE - VCA_ADC_HOME));
                if (probe_limit < PROBE_MIN_MS) probe_limit = PROBE_MIN_MS;
                if (probe_limit > PROBE_TIMEOUT_MS) probe_limit = PROBE_TIMEOUT_MS;
            }
            bool probe_done = ((now - s_probe_t0) >= probe_limit);
            if (probe_done) {
                /* PROBE 끝 → MOVE로 전환
                 * 유부하/무부하 모두 Loop2 duty 직접제어 (Loop1 발진 문제) */
                if (g_load_detected != 0U) {
                    Loop1_Stop();
                    s_loaded_duty = PROBE_DUTY;  /* PROBE duty에서 연속 시작 */
                    TLE9201_SetDir(DIR_PUSH);
                } else {
                    Loop1_Stop();
                    TLE9201_SetDir(DIR_PUSH);
                    VCA_SetDuty(NO_LOAD_MOVE_DUTY);
                }
                s_state = VCA_MOVE_TO_TARGET;
                stable_t0 = 0;
                s_move_t0 = now;     /* MOVE 타이머 시작 */
                s_burst_i_peak = 0;  /* 대안3: 전류 피크 초기화 */
                g_motor_active = 1;  /* MOVE 진입 → ADC 동결 */
            }
            break;
        }

        /* ══ 1b. MOVE: Loop1 전류제어로 목표까지 PUSH ════════════════════
         *  3단계 제동:
         *  [HOME+1mm] ─ PUSH_I(결정된값) ─ [목표-BRAKE_ZONE] ─ PUSH_I_BRAKE ─ [목표-DB] ─ HOLD
         * ═══════════════════════════════════════════════════════════════*/
        case VCA_APPROACH_TARGET:   /* 호환 유지 — MOVE로 즉시 */
            Loop1_Start(s_push_i_ma, (g_load_detected != 0U));
            s_burst_i_peak = 0;  /* 대안3: 전류 피크 초기화 */
            s_state = VCA_MOVE_TO_TARGET;
            break;

        case VCA_MOVE_TO_TARGET: {
            g_state_dbg = 2;

            /* ── MOVE: 3단계 duty 제어 (대안1+2+3 조합) ──────────────
             *  Phase 1 (0~penetrate_ms): 깊이 비례 burst duty로 피부 관통
             *    ★ 대안1: burst duty = 깊이에 비례 (얕으면 약하게, 깊으면 강하게)
             *    ★ 대안3: 전류 감시 → 피크 대비 하강 시 burst 조기 종료
             *  Phase 2 (penetrate_ms~+TAPER_MS): burst→LUT 경사 하강
             *    ★ 대안2: 급격한 계단 대신 부드러운 테이퍼링
             *  Phase 3 (~SETTLE_MS): LUT duty로 평형 유지 → HOLD 전환
             * ──────────────────────────────────────────────────────── */
            #define PENETRATE_BASE_MS  30U    /* 최소 부스트 시간 (ms)   */
            #define PENETRATE_PER_MM   40U    /* mm당 추가 시간 (ms/mm)  */
            #define TAPER_MS           30U    /* 테이퍼 구간 시간 (ms)   */
            #define MOVE_SETTLE_MS     800U

            /* 대안1: 깊이 비례 burst duty
             *  얕은 깊이 → 낮은 burst (오버슈트 방지)
             *  깊은 깊이 → 높은 burst (관통력 확보)
             *    0.50mm → 0.350
             *    2.00mm → 0.457
             *    3.50mm → 0.550  */
            #define BURST_DUTY_MIN     (0.350f)  /* 최소 burst duty (0.5mm) */
            #define BURST_DUTY_MAX     (0.550f)  /* 최대 burst duty (3.5mm) */
            #define BURST_DEPTH_MAX    (3.50f)   /* burst duty 최대 깊이    */

            /* 대안3: 전류 감시 파라미터
             *  burst 중 전류 피크를 추적하고, 피크 대비 DROP_PCT% 이상
             *  하락하면 관통 완료로 판단하여 burst 조기 종료.
             *  GUARD_MS 동안은 전류 상승 중이므로 감시 안함. */
            #define CURRENT_GUARD_MS   20U    /* 초기 전류 상승 대기 (ms) */
            #define CURRENT_DROP_PCT   30U    /* 피크 대비 하락률 (%)     */
            #define CURRENT_PEAK_MIN   500    /* 최소 피크 전류 (mA) — 이하면 감시 안함 */

            float depth = g_needle_depth_mm;
            if (depth < 0.0f) depth = 0.0f;

            /* 깊이 비례 burst duty 계산 */
            float burst_depth = depth;
            if (burst_depth > BURST_DEPTH_MAX) burst_depth = BURST_DEPTH_MAX;
            float burst_duty = BURST_DUTY_MIN
                             + (burst_depth / BURST_DEPTH_MAX)
                               * (BURST_DUTY_MAX - BURST_DUTY_MIN);

            /* 깊이 비례 부스트 시간 */
            uint32_t penetrate_ms = PENETRATE_BASE_MS
                                  + (uint32_t)(depth * (float)PENETRATE_PER_MM);

            uint32_t move_elapsed = (s_move_t0 != 0) ? (now - s_move_t0) : 0;
            float lut_duty = depth_to_duty(depth);

            /* 대안3: 전류 감시 — burst 조기 종료 판정 */
            if (move_elapsed < penetrate_ms) {
                int32_t i_now = g_i_meas_mA;
                if (move_elapsed >= CURRENT_GUARD_MS) {
                    if (i_now > s_burst_i_peak)
                        s_burst_i_peak = i_now;

                    /* 피크 대비 DROP_PCT% 이상 하락 → 관통 완료 */
                    if (s_burst_i_peak >= CURRENT_PEAK_MIN) {
                        int32_t threshold = s_burst_i_peak
                                          * (int32_t)(100U - CURRENT_DROP_PCT) / 100;
                        if (i_now < threshold) {
                            penetrate_ms = move_elapsed;  /* burst 즉시 종료 */
                        }
                    }
                }
            }

            /* 3단계 duty 결정 */
            float move_duty;
            if (move_elapsed < penetrate_ms) {
                /* Phase 1: 깊이 비례 burst */
                move_duty = burst_duty;
            } else if (move_elapsed < penetrate_ms + TAPER_MS) {
                /* Phase 2 (대안2): burst → LUT 경사 하강 (테이퍼) */
                float t = (float)(move_elapsed - penetrate_ms) / (float)TAPER_MS;
                move_duty = burst_duty + t * (lut_duty - burst_duty);
            } else {
                /* Phase 3: LUT duty 평형 유지 */
                move_duty = lut_duty;
            }
            if (move_duty > HOLD_DUTY_MAX) move_duty = HOLD_DUTY_MAX;

            TLE9201_SetDir(DIR_PUSH);
            VCA_SetDuty(move_duty);

            g_motor_active = 1;  /* PWM 중 ADC 동결 (TIA 고임피던스 EMI) */

            /* SETTLE_MS 후 HOLD 전환 */
            if (s_move_t0 != 0 && move_elapsed >= MOVE_SETTLE_MS) {
                Loop1_Stop();
                s_state   = VCA_HOLD_TARGET;
                stable_t0 = 0;
                break;
            }

            g_duty_dbg = move_duty;
            g_dir_dbg  = 1;
            break;
        }

        /* ══ 2. HOLD: 깊이 비례 feedforward, 모터 계속 ON ═══════════════
         *  PWM 중 위치 ADC 읽기 불가 (TIA 200kΩ 고임피던스 EMI)
         *  → LUT 기반 duty만으로 깊이 유지 (HW 개선 후 위치 피드백 추가 예정)
         * ═══════════════════════════════════════════════════════════════*/
        case VCA_HOLD_TARGET: {
            g_state_dbg = 3;

            float depth = g_needle_depth_mm;
            if (depth < 0.0f) depth = 0.0f;

            float duty_hold = depth_to_duty(depth);
            if (duty_hold > HOLD_DUTY_MAX) duty_hold = HOLD_DUTY_MAX;

            TLE9201_SetDir(DIR_PUSH);
            VCA_SetDuty(duty_hold);
            g_motor_active = 1;  /* PWM 중 ADC 동결 (TIA 고임피던스 EMI) */
            g_vca_err  = 0;
            g_duty_dbg = duty_hold;
            g_en_dbg   = 1;
            break;
        }

        /* ══ 3. RETURN: PULL 추출 → PUSH 감속 → 무전류 안착 (3단계 무소음) ═
         *  ① 피부 추출  : DIR_PULL + EXTRACT_DUTY (cur_mm > BRAKE_START_MM)
         *  ② 능동 감속  : DIR_PUSH + 속도비례 듀티 (홈 충돌 운동에너지 흡수)
         *  ③ 무전류 안착: VCA OFF (vel≈0 도달 후, 스프링 약한 잔여력만)
         * ═══════════════════════════════════════════════════════════════*/
        case VCA_RETURN_HOME: {
            g_state_dbg = 4;
            if (s_l1_enable) Loop1_Stop();

            /* 첫 진입 시각 기록 (안전 타임아웃 기준) */
            if (s_ret_ramp_t0 == 0) {
                s_ret_ramp_t0 = now;
                if (s_ret_ramp_t0 == 0) s_ret_ramp_t0 = 1;
            }

            /* 현재 위치 (홈 기준 mm) 및 속도 */
            int32_t pos_dist = (int32_t)pos - (int32_t)VCA_ADC_HOME;
            if (pos_dist < 0) pos_dist = 0;
            float cur_mm = (float)pos_dist / (float)VCA_ADC_PER_MM;
            float vel = s_vel_lp;  /* ADC/ms; 음수 = 홈 접근 방향 */

            /* Phase 0: PULL 능동 추출 (피부에서 끌어냄, 전류 흐름) */
            if (s_ret_phase == 0) {
                uint32_t elapsed = now - s_ret_ramp_t0;
                bool past_skin       = (cur_mm <= RETURN_BRAKE_START_MM);
                bool extract_timeout = (elapsed >= RETURN_EXTRACT_TIMEOUT_MS);

                if (!past_skin && !extract_timeout) {
                    TLE9201_SetDir(DIR_PULL);
                    VCA_SetDuty(RETURN_EXTRACT_DUTY);
                } else {
                    /* 피부 통과 → 감속 단계 진입 */
                    s_ret_phase   = 1;
                    s_ret_ramp_t0 = now;
                }
            }

            /* Phase 1: PUSH 속도비례 감속 (홈 충돌 직전 운동에너지 흡수) */
            if (s_ret_phase == 1) {
                uint32_t elapsed     = now - s_ret_ramp_t0;
                bool brake_timeout   = (elapsed >= RETURN_BRAKE_TIMEOUT_MS);
                bool nearly_stopped  = (vel > -RETURN_STOP_VEL);  /* 거의 정지 */

                if (brake_timeout || nearly_stopped) {
                    /* 감속 완료 → VCA OFF, 잔여 스프링력만으로 자연 안착 */
                    VCA_SetDuty(0.0f);
                    s_ret_phase = 2;
                } else {
                    /* 속도가 클수록 강한 PUSH 브레이크 (스프링 거스름) */
                    float brake_duty = RETURN_BRAKE_GAIN * (-vel);
                    if (brake_duty < RETURN_BRAKE_DUTY_MIN) brake_duty = RETURN_BRAKE_DUTY_MIN;
                    if (brake_duty > RETURN_BRAKE_DUTY_MAX) brake_duty = RETURN_BRAKE_DUTY_MAX;
                    TLE9201_SetDir(DIR_PUSH);
                    VCA_SetDuty(brake_duty);
                }
            }

            /* Phase 2: 무전류 자연 안착 — 정지~극저속 안착 (탁 소리 없음) */
            if (s_ret_phase == 2) {
                VCA_SetDuty(0.0f);
            }

            g_duty_dbg = (s_ret_phase == 0) ? RETURN_EXTRACT_DUTY : 0.0f;
            g_dir_dbg  = (s_ret_phase == 1) ? 1 : 0;  /* 1=PUSH(brake), 0=PULL/OFF */
            break;
        }

        case VCA_HOME_OFF:
        default:
            Loop1_Stop();
            vca_off();
            break;
        }

        osDelay(VCA_PID_DT_MS);
    }
}  /* end HPSwitchTask */

#else  /* USE_LINEAR_ACTUATOR */
/* ═══════════════════════════════════════════════════════════════════════════
 *  LA-T8 상태 머신 열거형
 * ═══════════════════════════════════════════════════════════════════════════*/
typedef enum {
    LA_IDLE = 0,           /* 모터 OFF, home 위치 */
    LA_STBY_PREVIEW,       /* STBY 모드: depth 프리뷰 이동 */
    LA_MOVE_TO_TARGET,     /* PID 활성, 목표로 이동 */
    LA_HOLD_TARGET,        /* PID 활성, 위치 유지 */
    LA_RETURN_HOME         /* PID로 0mm 복귀 */
} LA_State_t;

#define LA_SETTLE_BAND_MM   (0.20f)  /* 목표 ±0.2mm = 모터 OFF */
#define LA_SETTLE_TIME_MS   (50U)    /* 50ms 연속 안정 → HOLD 전환 */
#define LA_RETURN_TARGET    (0.0f)   /* 홈 위치 = 0mm (retracted) */
#define LA_HOME_BAND_MM     (0.30f)  /* 홈 도착 판정 (ADC 노이즈 여유) */
#define LA_MOVE_TIMEOUT_MS  (2000U)  /* 2초 내 미도달 → 에러 */

static inline void la_motor_on(void)
{
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_ENABLE);
}

static inline void la_motor_off(void)
{
    LA_PID_Stop();
    TIM8_SetDutyDirect(0.0f);
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  LA-T8 HPSwitchTask  (1 ms FreeRTOS)
 *
 *  간소화된 상태 머신: IDLE → MOVE → HOLD → RETURN → IDLE
 *  위치 PID는 TIM6 ISR(1kHz)에서 실행, 여기서는 타겟만 관리
 * ═══════════════════════════════════════════════════════════════════════════*/
void HPSwitchTask_impl(void *argument)
{
    (void)argument;

    /* ★ TLE9201 초기화 (VCA/LA 공통 H-Bridge) */
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);
    TIM8_SetDutyDirect(0.0f);
    osDelay(50);
    VCA_Init();   /* TLE9201 SPI init + enable */

    /* PWM 20kHz 설정 */
    __HAL_TIM_DISABLE(&htim8);
    __HAL_TIM_SET_PRESCALER(&htim8, 0);
    TIM8->ARR = 2499U;  /* 50MHz / 20kHz - 1 */
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    __HAL_TIM_ENABLE(&htim8);
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);
    TIM8_SetDutyDirect(0.0f);

    /* TIM8 CH1 output enable */
    TIM8->CCER &= ~TIM_CCER_CC1NE;
    TIM8->CCER |=  TIM_CCER_CC1E;
    TIM8->CCMR1 = (TIM8->CCMR1 & ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE))
                 | (0x6U << TIM_CCMR1_OC1M_Pos);
    TIM8->BDTR |= TIM_BDTR_MOE;
    motor_hal_set_ccr1_raw(0);  /* CCR1=0 (Phase 1: motor_hal wrapper 경유) */
    osDelay(10);

    /* ADC1 캘리브레이션 (pot 읽기용 -- ISR에서 사용 전 1회) */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    /* ADC1 enable (레지스터 직접 접근 전 필수) */
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }

    /* ── 활성 호밍: 모터를 기계적 홈(완전 수축)으로 자동 이동 ── */
    osDelay(50);
    {
      /* 1단계: 현재 ADC 읽기 */
      ADC1->CR |= ADC_CR_ADSTART;
      while (!(ADC1->ISR & ADC_ISR_EOC)) { }
      uint16_t prev_adc = (uint16_t)(ADC1->DR & 0xFFFU);

      /* 2단계: 수축 방향(retract)으로 저속 구동 → 기계적 홈 탐색 */
      motor_hal_dis_set(MOTOR_HAL_BRIDGE_ENABLE);
      TLE9201_SetDir(DIR_PUSH);     /* retract 방향 (ISR과 동일) */
      TIM8_SetDutyDirect(0.20f);    /* 저속 수축 */

      uint32_t stable_ms = 0;
      for (int i = 0; i < 3000; i++) {   /* 최대 3초 */
          osDelay(1);
          ADC1->CR |= ADC_CR_ADSTART;
          while (!(ADC1->ISR & ADC_ISR_EOC)) { }
          uint16_t cur = (uint16_t)(ADC1->DR & 0xFFFU);
          int diff = (int)cur - (int)prev_adc;
          if (diff < 0) diff = -diff;
          if (diff < 15) {
              stable_ms++;
              if (stable_ms >= 300) break;  /* 300ms ADC 안정 → 기계적 홈 도달 */
          } else {
              stable_ms = 0;
          }
          prev_adc = cur;
      }

      /* 3단계: 모터 정지 */
      TIM8_SetDutyDirect(0.0f);
      motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);
      osDelay(100);   /* 진동 안정 대기 */

      /* 4단계: 홈 위치 ADC 평균 측정 */
      uint32_t sum = 0;
      for (int i = 0; i < 16; i++) {
        ADC1->CR |= ADC_CR_ADSTART;
        while (!(ADC1->ISR & ADC_ISR_EOC)) { }
        sum += (ADC1->DR & 0xFFFU);
        osDelay(2);
      }
      uint16_t home_adc = (uint16_t)(sum / 16);
      g_la_home_offset_mm = la_adc_to_mm(home_adc);
      s_la_pos_filt = 0.0f;              /* EMA 필터 초기화 (홈=0) */
#if !RS485_MEASUREMENT_OUTPUT
      char db[64]; int n = snprintf(db, sizeof(db),
          "[LA_CAL] home_adc=%u offset=%.2fmm\r\n", home_adc, g_la_home_offset_mm);
      HAL_UART_Transmit(&huart1, (uint8_t*)db, n, 10);
#endif
    }

    s_la_adc_ready = 1;   /* ISR에서 ADC 읽기 허용 */

    /* ── 스위치 디바운스 변수 (VCA와 동일) ── */
    uint8_t raw_now = (HAL_GPIO_ReadPin(HP_SW_GPIO_Port, HP_SW_Pin) == GPIO_PIN_SET) ? 0U : 1U;
    bool     s_latched_push        = (raw_now == 1U);
    uint32_t s_press_cnt           = 0;
    uint32_t s_release_cnt         = 0;
    uint32_t s_release_block_until = 0;
    bool     last_in_push          = s_latched_push;

    LA_State_t s_state   = LA_IDLE;
    uint32_t s_settle_t0 = 0;
    uint32_t s_move_t0   = 0;
    uint8_t  s_was_ready = g_gui_ready;
    float    s_stby_depth = g_needle_depth_mm;
    uint32_t s_ready_since = 0;  /* STBY→READY 디바운스 타이머 */

    /* ISR ADC 비활성 유지 — Task에서 ADC + 제어 수행 (진단과 동일 구조) */
    s_la_adc_ready = 0;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();

        /* ── 0. ADC 읽기 (100Hz) + 비례 제어 (1kHz) ── */
        {
            /* ADC: 10ms마다 1회 읽기 + 점프 거부 (100Hz)
             * 실제 이동 ~5counts/10ms, 노이즈 점프 600+counts → threshold 200 */
            static uint8_t  adc_div = 0;
            static uint16_t adc_prev = 0;
            if (++adc_div >= 10) {
                adc_div = 0;
                ADC1->CR |= ADC_CR_ADSTART;
                while (!(ADC1->ISR & ADC_ISR_EOC)) { }
                uint16_t adc_raw = (uint16_t)(ADC1->DR & 0xFFFU);

                /* 점프 거부: 10ms당 200카운트 이상 변화 = 노이즈 */
                if (adc_prev != 0) {
                    int adiff = (int)adc_raw - (int)adc_prev;
                    if (adiff < 0) adiff = -adiff;
                    if (adiff > 200) {
                        adc_raw = adc_prev;     /* 노이즈 → 이전 값 유지 */
                    } else {
                        adc_prev = adc_raw;
                    }
                } else {
                    adc_prev = adc_raw;
                }

                g_la_pos_adc = adc_raw;
                g_vca_pos_adc = adc_raw;

                float pos_raw = (la_adc_to_mm(adc_raw) - g_la_home_offset_mm) * LA_CAL_GAIN;
                if (pos_raw < 0.0f) pos_raw = 0.0f;
                s_la_pos_filt += LA_FILT_ALPHA * (pos_raw - s_la_pos_filt);
                g_la_pos_mm = s_la_pos_filt;

                /* Hard limit */
                if (pos_raw >= LA_HARD_LIMIT_MM) {
                    g_la_hard_limit = 1;
                    if (s_la_enable && g_la_target_mm >= pos_raw) {
                        TIM8_SetDutyDirect(0.0f);
                        s_la_enable = 0;
                        g_la_pid_active = 0;
                    }
                }
            }

            /* 비례 제어: 매 1ms, 최신 EMA 값 사용 */
            if (s_la_enable) {
                float target = g_la_target_mm;
                float err = target - s_la_pos_filt;
                float err_abs = err;
                if (err_abs < 0.0f) err_abs = -err_abs;
                float u;

                if (err_abs < LA_DEADBAND_MM) {
                    u = 0.0f;
                } else {
                    float duty = 0.80f * err_abs;
                    if (duty > LA_PID_OUT_MAX) duty = LA_PID_OUT_MAX;
                    if (duty < 0.12f)          duty = 0.12f;
                    u = (err > 0.0f) ? duty : -duty;
                }

                g_la_pid_output = u;
                if (u >= 0.0f) {
                    TLE9201_SetDir(DIR_PULL);
                    TIM8_SetDutyDirect(u);
                } else {
                    TLE9201_SetDir(DIR_PUSH);
                    TIM8_SetDutyDirect(-u);
                }
            }
        }

        /* ── 1. 스위치 디바운스 ── */
        uint8_t raw = (HAL_GPIO_ReadPin(HP_SW_GPIO_Port, HP_SW_Pin) == GPIO_PIN_SET) ? 0U : 1U;
        if (!s_latched_push) {
            if (raw == 1U) {
                if (++s_press_cnt >= (PRESS_DB_MS / HP_SAMPLE_MS)) {
                    s_latched_push = true;
                    s_press_cnt = 0;
                    s_release_cnt = 0;
                    s_release_block_until = now + RELEASE_GRACE_MS;
                }
            } else s_press_cnt = 0;
        } else {
            if (raw == 0U) {
                if (now < s_release_block_until)
                    s_release_cnt = 0;
                else if (++s_release_cnt >= (RELEASE_DB_MS / HP_SAMPLE_MS)) {
                    s_latched_push = false;
                    s_release_cnt = 0;
                    s_press_cnt = 0;
                }
            } else s_release_cnt = 0;
        }

        /* 입력 소스 선택 (Foot/Switch/Lock) */
        bool in_push;
        if (g_hp_locked)      in_push = false;
        else if (g_foot_mode) in_push = (g_foot_state != 0);
        else                  in_push = s_latched_push;
        g_sw_state = in_push ? 1U : 0U;

        /* ── 2. STBY→READY 디바운스 (RF407 토글 무시, 200ms 유지 시 전환) ── */
        if (g_gui_ready) {
            if (s_ready_since == 0) s_ready_since = now;
            /* STBY 프리뷰 완료(LA_IDLE) 후 홈 복귀 — 프리뷰 진행 중 역방향 방지.
             * 800ms 타임아웃: 프리뷰가 비정상적으로 길어질 경우 강제 복귀 */
            uint8_t preview_done = (s_state == LA_IDLE);
            uint8_t timed_out    = ((now - s_ready_since) >= 800U);
            if ((now - s_ready_since) >= 200U && (preview_done || timed_out)) {
                la_motor_on();
                LA_PID_Start(LA_RETURN_TARGET);
                s_state = LA_RETURN_HOME;
                g_vca_state = 3;
                s_settle_t0 = 0;
                s_ready_since = 0;
            }
        } else {
            s_ready_since = 0;
        }
        /* READY→STBY: 즉시 (READY 동작 중이면 홈 복귀) */
        if (!g_gui_ready && s_was_ready) {
            if (s_state == LA_MOVE_TO_TARGET || s_state == LA_HOLD_TARGET) {
                la_motor_on();
                LA_PID_Start(LA_RETURN_TARGET);
                s_state = LA_RETURN_HOME;
                g_vca_state = 3;
                s_settle_t0 = 0;
            }
        }
        s_was_ready = g_gui_ready;

        /* ── 3. 모드별 처리 ── */
        if (!g_gui_ready) {
            /* ── STBY 모드: 깊이 프리뷰 이동 ── */
            if (s_state == LA_IDLE) {
                float d = g_needle_depth_mm;
                float diff = d - s_stby_depth;  /* 사용자가 depth 슬라이더를 변경했을 때만 재시작 */
                if (diff < 0.0f) diff = -diff;
                if (diff > LA_RESTART_MM) {  /* DEADBAND보다 커야 재시작 */
                    la_motor_on();
                    LA_PID_Start(la_depth_to_target(d));
                    s_state = LA_STBY_PREVIEW;
                    s_stby_depth = d;
                    g_motor_active = 1;
                }
            } else if (s_state == LA_STBY_PREVIEW) {
                /* depth 슬라이더 변경 감지 → 새 타겟 */
                float d = g_needle_depth_mm;
                float diff = d - s_stby_depth;
                if (diff < 0) diff = -diff;
                if (diff > LA_RESTART_MM) {
                    la_motor_on();
                    LA_PID_Start(la_depth_to_target(d));
                    s_stby_depth = d;
                    g_motor_active = 1;
                    s_settle_t0 = 0;
                }
            } else {
                /* RETURN_HOME / MOVE / HOLD → STBY 모드에선 depth 타겟으로 전환 */
                float d = g_needle_depth_mm;
                la_motor_on();
                LA_PID_Start(la_depth_to_target(d));
                s_state = LA_STBY_PREVIEW;
                s_stby_depth = d;
                g_motor_active = 1;
                s_settle_t0 = 0;
            }
            last_in_push = in_push;
        } else {
            /* ── READY 모드: 스위치 레벨 처리 ── */
            /* 스위치 눌림 → MOVE 시작 (레벨 트리거, IDLE 상태에서만) */
            if (in_push && s_state == LA_IDLE) {
                la_motor_on();
                LA_PID_Start(la_depth_to_target(g_needle_depth_mm));
                s_state = LA_MOVE_TO_TARGET;
                g_vca_state = 1;
                s_settle_t0 = 0;
                s_move_t0 = now;
                g_motor_active = 1;
                s_release_block_until = now + RELEASE_GRACE_MS;
            }
            /* 스위치 릴리즈 → RETURN (레벨 트리거) */
            if (!in_push && s_state != LA_IDLE && s_state != LA_RETURN_HOME) {
                la_motor_on();
                LA_PID_Start(LA_RETURN_TARGET);  /* integral 리셋 필수 */
                s_state = LA_RETURN_HOME;
                g_vca_state = 3;
                s_settle_t0 = 0;
            }
            last_in_push = in_push;
        }

        /* ── 5. 상태 머신 ── */
        float pos = g_la_pos_mm;

        switch (s_state)
        {
        case LA_IDLE:
            g_vca_state = 0;
            break;

        case LA_STBY_PREVIEW: {
            /* ±0.2mm 도달 → 즉시 모터 OFF */
            float err_p = g_la_target_mm - pos;
            if (err_p < 0) err_p = -err_p;
            if (err_p < LA_DEADBAND_MM) {
                LA_PID_Stop();
                la_motor_off();
                g_motor_active = 0;
                s_state = LA_IDLE;
            }
            /* hard limit 체크 */
            if (g_la_hard_limit) {
                g_hp_error = 10;
                la_motor_on();
                LA_PID_Start(LA_RETURN_TARGET);
                s_state = LA_RETURN_HOME;
                g_vca_state = 3;
                s_settle_t0 = 0;
            }
            break;
        }

        case LA_MOVE_TO_TARGET: {
            float err = g_la_target_mm - pos;
            if (err < 0) err = -err;
            if (err < LA_DEADBAND_MM) {
                /* ±0.2mm 도달 → 즉시 모터 OFF */
                LA_PID_Stop();
                la_motor_off();
                g_motor_active = 0;
                s_state = LA_HOLD_TARGET;
                g_vca_state = 2;
            }
            /* 타임아웃 */
            if ((now - s_move_t0) >= LA_MOVE_TIMEOUT_MS && s_state == LA_MOVE_TO_TARGET) {
                g_hp_error = 12;  /* 이동 타임아웃 */
            }
            /* Hard limit 복구 */
            if (g_la_hard_limit) {
                g_hp_error = 10;
                la_motor_on();
                LA_PID_Start(LA_RETURN_TARGET);
                s_state = LA_RETURN_HOME;
                g_vca_state = 3;
                s_settle_t0 = 0;
            }
            break;
        }

        case LA_HOLD_TARGET:
            /* 모터 OFF 상태 유지 — 스위치 릴리즈 대기 */
            /* (lead screw 자기유지, PID 재시작 없음) */
            break;

        case LA_RETURN_HOME: {
            if (pos < LA_HOME_BAND_MM) {
                /* 홈 도착 → 즉시 모터 OFF */
                LA_PID_Stop();
                la_motor_off();
                g_motor_active = 0;
                s_state = LA_IDLE;
                s_stby_depth = g_la_pos_mm + 10.0f;
                g_vca_state = 0;
                g_la_hard_limit = 0;
                g_hp_error = 0;
            }
            break;
        }
        } /* switch */

        g_sm_pos = g_la_pos_adc;

        /* ── DEBUG: 1초 주기 상시 출력 ── */
#if !RS485_MEASUREMENT_OUTPUT
        { static uint32_t s_dbg_t = 0;
          if ((now - s_dbg_t) >= 1000U) {
            s_dbg_t = now;
            char db[120]; int n = snprintf(db, sizeof(db),
                "R=%d S=%d t=%.2f p=%.2f u=%.2f d=%.1f adc=%u off=%.1f\r\n",
                (int)g_gui_ready, (int)s_state, g_la_target_mm,
                g_la_pos_mm, g_la_pid_output, g_needle_depth_mm,
                (unsigned)g_la_pos_adc, g_la_home_offset_mm);
            HAL_UART_Transmit(&huart1, (uint8_t*)db, n, 10);
        }}
#endif

        osDelay(1);
    }
}  /* end LA HPSwitchTask */
#endif /* USE_LINEAR_ACTUATOR */

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADS8325 직접 폴링 ADC (TIM7 비사용)
 *
 *  ※ TIM7은 HAL timebase + LVGL tick (1kHz)로 사용 중이므로
 *     ADS8325 샘플링에 TIM7를 재설정할 수 없음.
 *     대신 Task 내에서 직접 20개 burst 읽기 (280μs/5ms ≈ 5.6%)
 *
 *  SPI1: PLL1Q/16 SCLK (CubeMX prescaler=16, 런타임 변경 금지)
 * ═══════════════════════════════════════════════════════════════════════════*/

/* ── ADC Task: 직접 폴링 burst 읽기 + 필터 ── */
#if !USE_LINEAR_ACTUATOR       /* ── VCA: ADS8325 외장 ADC ── */
void ADS8325_AcqTask_impl(void *argument)
{
    (void)argument;
    ADS8325_Init(&g_ads8325, &hspi1, GPIOA, GPIO_PIN_4);

    /* ── 처리 파라미터 ── */
    #define BATCH_COUNT   20    /* 5ms 주기당 직접 읽기 개수 */
    #define BATCH_MAX     24
    #define TRIM_PCT      25    /* 상하 25% 절삭 → 20개 중 10개만 평균 */
    #define IIR_ALPHA     (0.10f)  /* 강한 평활: τ≈50ms */
    #define SPIKE_TH      (3000)
    #define SETTLE_BATCHES 4    /* 모터 정지 후 20ms(4배치×5ms) 대기 */

    static uint8_t  s_adc_init   = 0;
    static uint8_t  s_settle_cnt = 0;
    static float    s_iir_acc    = 0.0f;

    for (;;) {
        Wdg_TaskAlive(WDG_TASK_ADS8325_ACQ);
        osDelay(5);  /* 5ms 주기 */

        /* HPSwitchTask (sine 모드 또는 production controller) 가 hspi1 단독 사용.
         *  AcqTask 는 항상 슬립 — SPI 경합 방지. */
        s_settle_cnt = SETTLE_BATCHES;
        continue;

        /* (도달 안 함 — 아래 batch 코드는 legacy 보존용) */
        if (g_motor_active) {
            s_settle_cnt = SETTLE_BATCHES;
            continue;
        }

        /* 모터 정지 직후 → back-EMF 대기 */
        if (s_settle_cnt > 0) {
            s_settle_cnt--;
            continue;
        }

        /* ── SPI burst 읽기 (Task 컨텍스트 — 안전) ── */
        uint16_t buf[BATCH_MAX];
        uint16_t n_req = BATCH_COUNT;
        for (uint16_t i = 0; i < n_req; i++) {
            buf[i] = ADS8325_Read(&g_ads8325);
        }

        /* ── 정렬 (insertion sort, N≤24) ── */
        uint16_t avail = n_req;
        for (int i = 1; i < (int)avail; i++) {
            uint16_t key = buf[i];
            int j = i - 1;
            while (j >= 0 && buf[j] > key) {
                buf[j + 1] = buf[j];
                j--;
            }
            buf[j + 1] = key;
        }

        /* ── wrap 감지 ── */
        uint16_t smin = buf[0];
        uint16_t smax = buf[avail - 1];
        if (smax >= 60000U && (smax - smin) > 15000U) {
            g_vca_pos_raw = 65535U;
            if (s_adc_init) {
                s_iir_acc = 65535.0f;
                g_vca_pos_adc = 65535U;
            }
            continue;
        }

        /* ── trimmed mean: 상하 TRIM_PCT% 절삭 ── */
        uint16_t trim_n = (uint16_t)((uint32_t)avail * TRIM_PCT / 100U);
        uint16_t lo = trim_n;
        uint16_t hi = avail - trim_n;
        if (hi <= lo) { lo = 0; hi = avail; }  /* 안전장치 */

        uint32_t sum = 0;
        for (uint16_t i = lo; i < hi; i++) sum += buf[i];
        uint16_t trimmed = (uint16_t)(sum / (hi - lo));

        g_vca_pos_raw = trimmed;  /* 디버그: 필터 전 값 */

        /* ── IIR 저역통과 + 스파이크 제거 ── */
        if (!s_adc_init) {
            s_iir_acc = (float)trimmed;
            s_adc_init = 1;
        } else {
            float diff = (float)trimmed - s_iir_acc;
            if (diff < 0.0f) diff = -diff;

            if (diff > (float)SPIKE_TH) {
                /* 스파이크 → IIR 유지 */
            } else {
                s_iir_acc = s_iir_acc * (1.0f - IIR_ALPHA)
                          + (float)trimmed * IIR_ALPHA;
            }
        }

        /* IIR → uint16_t */
        float out = s_iir_acc;
        if (out < 0.0f) out = 0.0f;
        if (out > 65535.0f) out = 65535.0f;
        g_vca_pos_adc = (uint16_t)(out + 0.5f);
    }
}
#else  /* USE_LINEAR_ACTUATOR */
/* LA 모드: ADS8325 불필요 — 위치는 TIM6 ISR에서 ADC1 직접 읽기 */
void ADS8325_AcqTask_impl(void *argument)
{
    (void)argument;
    for (;;) osDelay(10000);
}
#endif /* USE_LINEAR_ACTUATOR */

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADS8325_RS485_Task  (UART1 전송)
 * ═══════════════════════════════════════════════════════════════════════════*/

/* 0 = RS485 HP 슬레이브 프로토콜 (Main↔HP 동기화)
 * 1 = 기존 측정값 출력 (디버그 전용, Main 통신 불가) */
/* ── HP 로컬 상태 (GUI·RS485 공용) ── */
volatile uint8_t g_rf_power = 5;   /* PWR 슬라이더 값 0~9 */
volatile uint8_t g_ui_update_pending = 0;  /* RS485 → UI 업데이트 플래그 */

#if RS485_MEASUREMENT_OUTPUT
/* ── 기존 측정값 출력 모드 ──────────────────────────────────────────────── */
void ADS8325_RS485_Task_impl(void *argument)
{
    (void)argument;
    const TickType_t PERIOD = pdMS_TO_TICKS(10);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, PERIOD);

        /* WatchdogTask heartbeat — 빠지면 grace(3s) 후 IWDG 리셋 */
        Wdg_TaskAlive(WDG_TASK_RS485);

        /* ADS8325 raw/filtered position output for plotter */
#if !USE_LINEAR_ACTUATOR
        unsigned int raw = (unsigned int)g_vca_pos_raw;
#else
        unsigned int raw = (unsigned int)g_vca_pos_adc;
#endif
        unsigned int filtered = (unsigned int)g_vca_pos_adc;

        int len = snprintf(g_ads8325_buf, sizeof(g_ads8325_buf),
                           "$%u %u;\n",
                           raw,
                           filtered);
        HAL_UART_Transmit(&huart1, (uint8_t*)g_ads8325_buf, (uint16_t)len, 50);
    }
}

#else  /* ═══ RS485 HP 슬레이브 모드 (RX→응답 전용) ═══
 *
 *  이 장치 = HP(핸드피스, 슬레이브)
 *  Main 보드가 10 ms마다 @Q*51\n 등을 보내면,
 *  HP는 수신 후에만 응답 전송.  자율 송신 절대 금지 (충돌 방지).
 *
 *  [Main TX] ──→ RS485 bus ──→ [HP RX] → 파싱 → [HP TX 응답]
 *                              ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *                              이 코드가 담당하는 부분
 * ═══════════════════════════════════════════════════════════ */

/* ── XOR 패리티 계산 ── */
static uint8_t rs485_parity(const uint8_t *d, uint16_t len)
{
    uint8_t x = 0;
    for (uint16_t i = 0; i < len; i++) x ^= d[i];
    return x;
}

/* ── HEX 문자 → 4-bit 값 (-1 = 에러) ── */
static int8_t hex_val(uint8_t ch)
{
    if (ch >= '0' && ch <= '9') return (int8_t)(ch - '0');
    if (ch >= 'A' && ch <= 'F') return (int8_t)(ch - 'A' + 10);
    if (ch >= 'a' && ch <= 'f') return (int8_t)(ch - 'a' + 10);
    return -1;
}

/* ── 응답 프레임 빌드: @<payload>*XX\n ── */
static uint16_t rs485_resp(uint8_t *buf, const uint8_t *pl, uint16_t plen)
{
    static const char HEX[] = "0123456789ABCDEF";
    uint16_t i = 0;
    buf[i++] = '@';
    for (uint16_t j = 0; j < plen; j++) buf[i++] = pl[j];
    uint8_t par = rs485_parity(pl, plen);
    buf[i++] = '*';
    buf[i++] = HEX[(par >> 4) & 0x0F];
    buf[i++] = HEX[par & 0x0F];
    buf[i++] = '\n';
    return i;
}

/* ── 디버그: 상태 변경 시 UART1 출력 ── */
static uint8_t s_dbg_prev_ready = 0xFF;

/* ── 수신 프레임 파싱 & 응답 전송 ── */
static void rs485_process(const uint8_t *frm, uint16_t flen)
{
    /* 최소 @X*XX\n = 6 바이트 */
    if (flen < 6 || frm[0] != '@') return;

    /* '*' 위치 검색 */
    uint16_t star = 0;
    for (uint16_t i = 1; i < flen; i++) {
        if (frm[i] == '*') { star = i; break; }
    }
    if (star == 0 || star + 3 > flen) return;

    /* ── 패리티 검증 ── */
    int8_t hh = hex_val(frm[star + 1]);
    int8_t hl = hex_val(frm[star + 2]);
    if (hh < 0 || hl < 0) return;
    uint8_t rx_par = ((uint8_t)hh << 4) | (uint8_t)hl;

    const uint8_t *pl  = &frm[1];       /* payload */
    uint16_t       plen = star - 1;      /* payload 길이 */
    if (rs485_parity(pl, plen) != rx_par) return;   /* 불일치 → 무시 */

    /* ── 커맨드 처리 & 응답 페이로드 구성 ── */
    uint8_t  resp[64];
    uint16_t rlen = 0;

    switch (pl[0]) {
    case 'Q': {
        /* ── 현재 HP 상태 보고 ──
         *  응답: @B<btn><rdy>P<pp>N<nn>V<v>E<ee>*XX\n
         *   btn = 물리스위치 0/1
         *   rdy = R(READY) / S(STBY)
         *   pp  = RF Power  01~10
         *   nn  = Needle depth×10  05~35
         *   v   = VCA state 0~4 (0=idle,1=fwd,2=target,3=ret,4=home)
         *   ee  = Error code 00~99 (00=OK)                           */
        resp[rlen++] = 'B';
        resp[rlen++] = (g_sw_state && g_gui_ready) ? '1' : '0';  /* STBY면 항상 0 */
        resp[rlen++] = g_gui_ready ? 'R' : 'S';
        {
            uint8_t pwr = g_rf_power + 1;          /* 0~9 → 01~10 */
            resp[rlen++] = 'P';
            resp[rlen++] = (uint8_t)('0' + (pwr / 10) % 10);
            resp[rlen++] = (uint8_t)('0' + pwr % 10);

            uint8_t ndl = (uint8_t)(g_needle_depth_mm * 10.0f + 0.5f);
            resp[rlen++] = 'N';
            resp[rlen++] = (uint8_t)('0' + (ndl / 10) % 10);
            resp[rlen++] = (uint8_t)('0' + ndl % 10);

            resp[rlen++] = 'V';
            resp[rlen++] = (uint8_t)('0' + (g_vca_state % 10));

            resp[rlen++] = 'E';
            resp[rlen++] = (uint8_t)('0' + (g_hp_error / 10) % 10);
            resp[rlen++] = (uint8_t)('0' + g_hp_error % 10);
        }
        break;
    }
    case 'B': {
        /* ── Main → HP 상태 일괄 설정 + 자동 unlock ────────────────────
         *  Main 이 일괄 상태 송신 = 작업 의도 → HP 자동 operational.   */
        if (plen < 9 || pl[3] != 'P' || pl[6] != 'N') break;

        g_hp_locked = 0;                   /* @B 수신 = 작업 의도 → unlock */
        g_gui_ready = (pl[2] == 'R') ? 1 : 0;

        uint8_t pp = (uint8_t)((pl[4] - '0') * 10 + (pl[5] - '0'));
        g_rf_power = (pp > 0) ? (uint8_t)(pp - 1) : 0;
        g_main_cmd_state[4] = g_rf_power;

        /* Depth 는 3.5mm 잠금 — Main 이 보낸 nn 무시 */
        (void)pl[7]; (void)pl[8];
        g_needle_depth_mm = 3.5f;
        g_main_cmd_state[5] = 35;

        g_ui_update_pending = 1;

        if (s_dbg_prev_ready != g_gui_ready)
            s_dbg_prev_ready = g_gui_ready;

        /* ACK: echo full payload */
        for (uint16_t i = 0; i < plen && rlen < sizeof(resp); i++)
            resp[rlen++] = pl[i];
        break;
    }

    case 'R':                              /* Ready 지시 */
        g_hp_locked = 0;                   /* @R 수신 = 작업 의도 → 자동 unlock */
        g_gui_ready = 1;
        g_ui_update_pending = 1;
        g_foot_state = 0;                  /* 트리거 상태만 클리어 */
        if (s_dbg_prev_ready != 1) s_dbg_prev_ready = 1;
        resp[rlen++] = 'R';
        break;

    case 'S':                              /* Stop 지시 */
        g_hp_locked = 0;                   /* @S 도 통신 활성 = unlock (1-pin 후 stuck 방지) */
        g_gui_ready = 0;
        g_foot_state = 0;
        g_ui_update_pending = 1;
        if (s_dbg_prev_ready != 0) s_dbg_prev_ready = 0;
        resp[rlen++] = 'S';
        break;

    case 'P':                              /* RF Power 개별 설정 */
        if (plen >= 3) {
            uint8_t pp = (uint8_t)((pl[1] - '0') * 10 + (pl[2] - '0'));
            g_rf_power = (pp > 0) ? (uint8_t)(pp - 1) : 0;
            g_main_cmd_state[4] = g_rf_power;
        }
        g_hp_locked = 0;
        g_gui_ready = 0;                   /* 안전: 파라미터 변경 시 항상 STBY (HP 자체 슬라이더와 동일) */
        g_ui_update_pending = 1;
        resp[rlen++] = pl[0];
        if (plen >= 3) { resp[rlen++] = pl[1]; resp[rlen++] = pl[2]; }
        break;

    case 'N':                              /* Needle depth 개별 설정 — 3.5mm 잠금 */
        /* Main 의 depth 변경 요청 무시: g_needle_depth_mm 은 항상 3.5 유지 */
        g_needle_depth_mm = 3.5f;
        g_main_cmd_state[5] = 35;
        g_hp_locked = 0;
        g_gui_ready = 0;                   /* 안전: 파라미터 변경 시 항상 STBY */
        g_ui_update_pending = 1;
        resp[rlen++] = pl[0];
        if (plen >= 3) { resp[rlen++] = pl[1]; resp[rlen++] = pl[2]; }
        break;

    case 'W':                              /* 기타 값 커맨드 (ACK만) */
        resp[rlen++] = pl[0];
        if (plen >= 3) { resp[rlen++] = pl[1]; resp[rlen++] = pl[2]; }
        break;

    case 'U':                              /* @U: Main bootloader update entry */
    case 'X': {                            /* 펌웨어 업데이트 요청 → 부트로더 진입
                                              register 직접 송신 — IT/HAL blocking 모두 timing 이슈 가능
                                              하므로 가장 단순한 polled write 로 단일 byte echo 송신       */
        uint8_t echo_byte = (uint8_t)pl[0];   /* 'U' or 'X' */
        USART2->ICR = USART_ICR_TCCF;          /* TC flag clear */
        while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) { /* wait TXE */ }
        USART2->TDR = echo_byte;
        while (!(USART2->ISR & USART_ISR_TC)) { /* wait transmission complete */ }

        osDelay(20);                            /* DE off settle */

        HAL_PWR_EnableBkUpAccess();
        TAMP->BKP0R = 0xDEADBEEF;
        NVIC_SystemReset();
        break;
    }

    case 'V': {
        /* ── 버전 응답: App + Boot info ──
         * V<APP_NAME> V<x.y.z> <YYMMDD>,<BOOT_NAME> V<x.y.z> <YYMMDD> */
        uint32_t fv = firmware_info.fw_version;
        uint32_t fd = firmware_info.build_date;
        int n = snprintf((char *)resp, sizeof(resp), "V%s V%ld.%ld.%ld %02lu%02lu%02lu",
                         firmware_info.project_name,
                         (long)FWINFO_VER_MAJOR(fv), (long)FWINFO_VER_MINOR(fv), (long)FWINFO_VER_PATCH(fv),
                         (unsigned long)(fd / 10000 % 100),
                         (unsigned long)(fd / 100 % 100),
                         (unsigned long)(fd % 100));
        /* Append boot info from fixed flash address 0x08000400 */
        const FirmwareInfo_t *bi = (const FirmwareInfo_t *)0x08000400;
        if (FirmwareInfo_IsValid(bi)) {
            uint32_t bv = bi->fw_version;
            uint32_t bd = bi->build_date;
            n += snprintf((char *)resp + n, sizeof(resp) - n,
                          ",%s V%ld.%ld.%ld %02lu%02lu%02lu",
                          bi->project_name,
                          (long)FWINFO_VER_MAJOR(bv), (long)FWINFO_VER_MINOR(bv), (long)FWINFO_VER_PATCH(bv),
                          (unsigned long)(bd / 10000 % 100),
                          (unsigned long)(bd / 100 % 100),
                          (unsigned long)(bd % 100));
        }
        rlen = (uint16_t)n;
        break;
    }

    case 'M': {
        /* ── Operation Mode 명시적 설정 — 어레이 기록 + 상태 변경 감지 ── */
        if (plen >= 2) {
            uint8_t new_mode = (pl[1] == '1') ? 1 : 0;
            if (g_foot_mode != new_mode) {
                g_main_state_changed = 1;
            }
            g_foot_mode = new_mode;
            g_foot_state = 0;
            g_main_cmd_state[0] = new_mode;
            g_main_cmd_state[1] = 0;
        }
        g_hp_locked = 0;
        g_gui_ready = 0;                   /* 안전: OperType 변경 시 항상 STBY */
        g_ui_update_pending = 1;
        resp[rlen++] = 'M';
        resp[rlen++] = g_foot_mode ? '1' : '0';
        break;
    }

    case 'F': {
        /* ── Foot 명령 ────────────────────────────────────────────────
         *  @F1 (foot press): foot mode 활성화 + 발사 트리거
         *  @F0 (foot release): foot_state=0, foot_mode 보존
         *  HP 모드일 때 @F0 는 HP 로컬 switch 와 무관 (HP switch 우선)    */
        g_hp_locked  = 0;
        g_gui_ready  = 1;
        if (plen >= 2) {
            uint8_t new_state = (pl[1] == '1') ? 1 : 0;
            if (g_foot_state != new_state) {
                g_main_state_changed = 1;
            }
            if (new_state == 1) {
                /* @F1 — foot mode 진입 + 트리거 */
                g_foot_mode = 1;
                g_foot_state = 1;
            } else {
                /* @F0 — foot 릴리즈 (foot_mode 보존, HP 모드면 무시됨) */
                g_foot_state = 0;
            }
            g_main_cmd_state[1] = new_state;
        }
        g_last_foot_cmd_ms = osKernelGetTickCount();
        resp[rlen++] = 'F';
        resp[rlen++] = g_foot_state ? '1' : '0';
        break;
    }

    case 'L': {
        /* ── @L: RF_Needle 호환 — L1=HP On / L0=HP Off ──
         *  사용자 설정값 보존 — 1-pin mode 진입 시 Main 이 @N/@B/@P 등으로
         *  HP 설정값을 reset/변경 → @L1 시 이전 값 복원                       */
        static float   s_saved_depth_mm = 3.5f;
        static uint8_t s_saved_rf_power = 5;
        static uint8_t s_saved_foot_mode = 0;
        static uint8_t s_saved_gui_ready = 1;
        static bool    s_saved_valid    = false;

        if (plen >= 2 && pl[1] == '1') {
            /* HP On — lock 해제 + 상태 IDLE 로 복귀 + 사용자 설정값 복원
             *  하드웨어 재초기화는 하지 않음 (1-pin 진입/복귀는 화면 모드 +
             *  니들 home 만 하면 충분).
             *  ※ 모드 변경 시 니들이 움직이지 않아야 한다는 사용자 요청에 따라
             *    cold-motor break-away (강제 depth=3.5 + auto-push 1.5s) 제거.
             *    저장된 depth 가 있으면 복원만 하고, 모터는 트리거하지 않음.   */
            g_hp_locked = 0;
            g_op_reset_request = 1;       /* HPSwitchTask: state=IDLE 로 복귀 */

            if (s_saved_valid) {
                /* 백업된 값으로 복원 (니들 movement 트리거 없음)
                 *  depth 는 3.5 잠금 → 백업값 무시 */
                g_needle_depth_mm   = 3.5f;
                g_rf_power          = s_saved_rf_power;
                g_foot_mode         = s_saved_foot_mode;
                g_gui_ready         = s_saved_gui_ready;
                g_main_cmd_state[5] = 35;
                g_main_cmd_state[4] = s_saved_rf_power;
                g_main_cmd_state[0] = s_saved_foot_mode;
            } else {
                /* 백업 없음 → 현재 값 유지 (depth 강제 변경 안함) */
                g_gui_ready = 1;
                g_foot_mode = 0;
            }
            g_foot_state = 0;
            g_ui_update_pending = 1;
        } else {
            /* HP Off — 사용자 설정값 모두 백업 후 lock (depth 는 항상 3.5 잠금) */
            s_saved_depth_mm = 3.5f;
            s_saved_rf_power  = g_rf_power;
            s_saved_foot_mode = g_foot_mode;
            s_saved_gui_ready = g_gui_ready;
            s_saved_valid     = true;

            g_hp_locked = 1;
            g_gui_ready = 0;
            g_foot_mode = 0;
            g_foot_state = 0;
            g_ui_update_pending = 2;
        }
        resp[rlen++] = 'L';
        resp[rlen++] = g_hp_locked ? '0' : '1';
        break;
    }

    case 'T': {
        /* ── Task 통계: ACK 없이 raw text를 라인별 전송 ── */
        /* ACK @-frame을 보내면 MAX13487이 '@' 클리핑 → 잔여 바이트 누출 */
        #define HT_SEND(s, l) do { HAL_StatusTypeDef _s; \
            do { _s = Uart2_Tx_IT((uint8_t *)(s), (uint16_t)(l)); \
                 if (_s == HAL_BUSY) osDelay(5); \
            } while (_s == HAL_BUSY); osDelay(10); } while(0)

        osDelay(3);  /* 수신 완료 후 bus settle */

        static TaskStatus_t ts_buf[16];
        configRUN_TIME_COUNTER_TYPE totalRun = 0;
        UBaseType_t cnt = uxTaskGetNumberOfTasks();
        if (cnt > 16) cnt = 16;
        cnt = uxTaskGetSystemState(ts_buf, cnt, &totalRun);
        if (totalRun == 0) totalRun = 1;
        int len;

        /* 라인 1: 타이틀 (앞 \r\n = MAX13487 클리핑 패딩) */
        len = snprintf(g_ads8325_buf, sizeof(g_ads8325_buf),
                       "\r\n[Handpiece Task Status]\r\n");
        HT_SEND(g_ads8325_buf, len);

        /* 라인 2: 컬럼 헤더 */
        len = snprintf(g_ads8325_buf, sizeof(g_ads8325_buf),
                       "Name              Prio St   CPU%%   HWM(bytes)   StackFree\r\n");
        HT_SEND(g_ads8325_buf, len);

        /* 라인 3: 구분선 */
        len = snprintf(g_ads8325_buf, sizeof(g_ads8325_buf),
                       "--------------------------------------------------------------\r\n");
        HT_SEND(g_ads8325_buf, len);

        /* 태스크 라인 */
        for (UBaseType_t i = 0; i < cnt; i++) {
            uint32_t cpu10 = (uint32_t)((1000ULL * (uint64_t)ts_buf[i].ulRunTimeCounter) / (uint64_t)totalRun);
            char sc;
            switch (ts_buf[i].eCurrentState) {
                case eRunning:   sc = 'R'; break;
                case eReady:     sc = 'r'; break;
                case eBlocked:   sc = 'B'; break;
                case eSuspended: sc = 'S'; break;
                default:         sc = '?'; break;
            }
            len = snprintf(g_ads8325_buf, sizeof(g_ads8325_buf),
                           "%02u %-13s %4lu  %c %5lu.%1lu %7u %9u\r\n",
                           (unsigned)i, ts_buf[i].pcTaskName,
                           (unsigned long)ts_buf[i].uxCurrentPriority, sc,
                           (unsigned long)(cpu10/10), (unsigned long)(cpu10%10),
                           (unsigned)(ts_buf[i].usStackHighWaterMark * sizeof(StackType_t)),
                           (unsigned)ts_buf[i].usStackHighWaterMark);
            HT_SEND(g_ads8325_buf, len);
        }

        /* Heap 정보 (라인별 분리 — 한 burst에 2줄 보내면 UART1 flush 중 오버런) */
        len = snprintf(g_ads8325_buf, sizeof(g_ads8325_buf),
                       "Heap Free(bytes):       %lu\r\n",
                       (unsigned long)xPortGetFreeHeapSize());
        HT_SEND(g_ads8325_buf, len);

        len = snprintf(g_ads8325_buf, sizeof(g_ads8325_buf),
                       "Heap MinEverFree(bytes): %lu\r\n",
                       (unsigned long)xPortGetMinimumEverFreeHeapSize());
        HT_SEND(g_ads8325_buf, len);

        #undef HT_SEND
        return;  /* ACK 전송 스킵 */
    }

    default:
        return;                            /* 알 수 없는 커맨드 → 무응답 */
    }

    /* ── 응답 프레임 전송 (먼저! — printf 지연이 Main 타임아웃 유발 방지) ── */
    if (rlen == 0) return;
    uint16_t txlen = rs485_resp((uint8_t *)g_ads8325_buf, resp, rlen);

    HAL_StatusTypeDef st;
    do {
        st = Uart2_Tx_IT((uint8_t *)g_ads8325_buf, txlen);
        if (st == HAL_BUSY) osDelay(10);
    } while (st == HAL_BUSY);

    if (resp[0] == 'B' && s_dbg_prev_ready != g_gui_ready)
        s_dbg_prev_ready = g_gui_ready;
}

/* ═══════════════════════════════════════════════════════════
 *  ADS8325_RS485_Task — HP 슬레이브 메인 루프
 *
 *  Main 보드의 프레임을 수신 → 파싱 → 응답.
 *  자율 송신 없음 — Main 수신 후에만 응답.
 *
 *  USART2 FIFO 모드 활성화 (8-byte HW FIFO):
 *   - @Q(6B)는 FIFO에 담기지만 @B(15B)는 FIFO 오버플로우 발생
 *   - 프레임 수신 중에는 tight poll (osDelay 없음)으로 오버런 방지
 *   - FIFO 비어있고 프레임 미수신 중일 때만 osDelay(1)
 * ═══════════════════════════════════════════════════════════*/
void ADS8325_RS485_Task_impl(void *argument)
{
    (void)argument;
    gUart2TxTaskHandle = xTaskGetCurrentTaskHandle();

    uint8_t  rx_buf[32];
    uint16_t rx_idx = 0;
    uint8_t  in_frame = 0;          /* '@' 수신 후 프레임 수신 중 플래그 */
    USART_TypeDef *uart = huart2.Instance;

    for (;;) {
        Wdg_TaskAlive(WDG_TASK_RS485);
        /* ── 에러 플래그 클리어 (ORE) ── */
        uint32_t isr = uart->ISR;
        if (isr & USART_ISR_ORE)
            uart->ICR = USART_ICR_ORECF;

        /* ── FE/NE 바이트 필터: MAX13487 첫 바이트 클리핑 대응 ──
         *  MAX13487 auto-direction이 start bit를 잘라 FE 발생.
         *  FE/NE 바이트는 읽고 버림 (preamble 0xFF 또는 깨진 '@'). */
        if (isr & (USART_ISR_FE | USART_ISR_NE)) {
            uart->ICR = USART_ICR_FECF | USART_ICR_NECF;
            if (isr & USART_ISR_RXNE_RXFNE)
                (void)(uart->RDR);  /* FE/NE 바이트 버림 */
            continue;
        }

        /* ── RXNE/RXFNE: FIFO에 수신 데이터 있으면 읽기 ── */
        if (isr & USART_ISR_RXNE_RXFNE) {
            uint8_t ch = (uint8_t)(uart->RDR & 0xFF);

            if (ch == '@') {
                rx_idx = 0;         /* 프레임 시작 → 재동기화 */
                in_frame = 1;
            }

            if (rx_idx < sizeof(rx_buf)) {
                rx_buf[rx_idx++] = ch;
                if (ch == '\n' && rx_idx >= 6) {    /* 프레임 완성 */
                    rs485_process(rx_buf, rx_idx);
                    rx_idx = 0;
                    in_frame = 0;
                }
            } else {
                rx_idx = 0;
                in_frame = 0;
            }
            /* FIFO에 데이터 남아있을 수 있음 → delay 없이 즉시 재확인 */
        } else {
            /* ── FIFO 비어있음 ── */
            if (in_frame) {
                /* 프레임 수신 중: tight poll — osDelay 하면 나머지 바이트가
                 * FIFO 오버플로우(8B)로 유실됨. @B(15B) 등 긴 프레임 보호. */
                taskYIELD();
            } else {
                /* 프레임 미수신 중: CPU 양보 → LVGL/터치 응답성 보장 */
                osDelay(1);
            }
        }
    }
}
#endif /* RS485_MEASUREMENT_OUTPUT */

__attribute__((unused))
static HAL_StatusTypeDef Uart1_Tx_IT(uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(&huart1, pData, size);
    if (st != HAL_OK) return st;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) return HAL_TIMEOUT;
    return HAL_OK;
}

__attribute__((unused))
static HAL_StatusTypeDef Uart2_Tx_IT(uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(&huart2, pData, size);
    if (st != HAL_OK) return st;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) return HAL_TIMEOUT;
    return HAL_OK;
}
/* ═══════════════════════════════════════════════════════════════════════════
 *  LEDTask
 * ═══════════════════════════════════════════════════════════════════════════*/
void LEDTask(void *argument)
{
    (void)argument;
    for (;;) osDelay(1000);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SysMonTask
 * ═══════════════════════════════════════════════════════════════════════════*/
void SysMonTask(void *argument)
{
    (void)argument;

#if !RS485_MEASUREMENT_OUTPUT
    const size_t    HEAP_WARN  = 2048;
    const size_t    HEAP_CRIT  = 1024;
#endif
    const uint32_t  PERIOD_MS  = 1000;
    for (;;) {
#if !RS485_MEASUREMENT_OUTPUT
        size_t freeHeap    = xPortGetFreeHeapSize();
        size_t minEverHeap = xPortGetMinimumEverFreeHeapSize();
        size_t qFree  = UART_Cmd_GetRxQueueFree();
        size_t qTotal = UART_Cmd_GetRxQueueLength();
        if (minEverHeap < HEAP_CRIT) {
            char msg[128];
            snprintf(msg, sizeof(msg), "\r\n[ALARM] Heap CRITICAL! free=%u, minEver=%u\r\n",
                     (unsigned)freeHeap, (unsigned)minEverHeap);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        } else if (minEverHeap < HEAP_WARN) {
            char msg[128];
            snprintf(msg, sizeof(msg), "\r\n[WARN] Heap low. free=%u, minEver=%u\r\n",
                     (unsigned)freeHeap, (unsigned)minEverHeap);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        }
        static size_t   last_used  = 0;
        static uint32_t last_print = 0;
        if (qTotal > 0) {
            size_t used = qTotal - qFree;
            uint32_t t  = osKernelGetTickCount();
            if ((used * 4 >= qTotal * 3) && (t - last_print >= 5000) && (used != last_used)) {
                last_used  = used;
                last_print = t;
                char msg[128];
                snprintf(msg, sizeof(msg), "\r\n[WARN] UART RX queue high: used=%u / %u\r\n",
                         (unsigned)used, (unsigned)qTotal);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
            }
        }
#endif
        /* IWDG refresh 제거: SysMonTask 단독 refresh 는 다른 task 가 hang 되어도
         * watchdog 이 안 터지므로 무용지물. → wdg.c 의 WatchdogTask 가
         * critical task 4개 (LVGL/RS485/ADS8325_Acq/HPSwitch) 의 heartbeat 확인 후 refresh. */
        osDelay(PERIOD_MS);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  NTC ADC Task  –  PC4 (ADC1_INP4) 10 Hz 읽기 → UART1 출력
 *
 *  NTC 대신 2 kΩ 가변저항이 부착됨 (VCA Over heat detection 회로)
 *  ADC1, 12-bit, 3.3 V 기준 → 전압 = raw × 3.3 / 4095
 * ═══════════════════════════════════════════════════════════════════════════*/
#include "adc.h"

#if !USE_LINEAR_ACTUATOR       /* ── VCA: NTC 온도 모니터 ── */
void NTC_ADC_Task(void *argument)
{
    (void)argument;

    /* ADC1 캘리브레이션 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    char buf[64];
    for (;;)
    {
        /* 단일 변환 시작 → 완료 대기 → 값 읽기 */
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            uint32_t raw = HAL_ADC_GetValue(&hadc1);
            /* mV 단위 전압 = raw * 3300 / 4095 */
            uint32_t mV = raw * 3300 / 4095;
#if !RS485_MEASUREMENT_OUTPUT
            int len = snprintf(buf, sizeof(buf),
                               "NTC ADC: %4lu  (%lu.%03lu V)\r\n",
                               raw, mV / 1000, mV % 1000);
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
#endif
        }
        HAL_ADC_Stop(&hadc1);

        osDelay(100);   /* 100 ms = 10 Hz */
    }
}
#else  /* USE_LINEAR_ACTUATOR */
/* LA 모드: ADC1(PC4)은 TIM6 ISR에서 pot 위치 읽기에 사용 — NTC 비활성 */
void NTC_ADC_Task(void *argument)
{
    (void)argument;
    for (;;) osDelay(10000);
}
#endif /* USE_LINEAR_ACTUATOR */
