/*
 * motor_hal.h
 *
 *  Cascade control Phase 1: PWM/GPIO/SPI 레지스터 접근 격리 wrapper.
 *  모든 함수는 ISR/Task 양쪽에서 호출 가능 (FreeRTOS API 미사용).
 *
 *  Phase 1 범위:
 *    - TIM8 CCR1 write (PWM duty)
 *    - TLE9201 DIR pin write (BSRR direct)
 *    - TLE9201 DIS pin write (BSRR direct)
 *    - emergency stop
 */
#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>
#include "main.h"
#include "stm32h5xx_hal.h"
#include "tle9201.h"   /* TLE9201_DIR/DIS_GPIO_Port·Pin 매크로 */
#include "tim.h"       /* htim8 extern */

#ifdef __cplusplus
extern "C" {
#endif

/* ─── 내부 매크로 (사용자가 건드리지 말 것) ─────────────────────────────── */
/* BSRR: 상위 16비트 = reset, 하위 16비트 = set */
#define MOTOR_HAL_BSRR_SET(pin)    ((uint32_t)(pin))
#define MOTOR_HAL_BSRR_RESET(pin)  ((uint32_t)(pin) << 16)

/* DIS 핀 레벨 (raw pin level) ─────────────────────────────────────────────
 *  TLE9201 DIS 극성: HIGH → 출력 차단, LOW → enable.
 *  이름은 "pin level" 그대로 — 의미(enable/disable)는 BRIDGE_* 매크로 사용.
 */
#define MOTOR_HAL_DIS_HIGH  (1U)   /* DIS 핀 HIGH (= H-bridge disable) */
#define MOTOR_HAL_DIS_LOW   (0U)   /* DIS 핀 LOW  (= H-bridge enable)  */

/* 호출자가 의미 단위로 사용하는 별칭 (권장) */
#define MOTOR_HAL_BRIDGE_DISABLE  MOTOR_HAL_DIS_HIGH  /* H-bridge OFF */
#define MOTOR_HAL_BRIDGE_ENABLE   MOTOR_HAL_DIS_LOW   /* H-bridge ON  */

/* DIR 핀 의미는 호출처(my_tasks.c의 DIR_PUSH/DIR_PULL)에서 결정 */

/* ─── PWM duty 설정 (TIM8 CCR1 직접 쓰기) ────────────────────────────────
 *  ARR은 호출 시점의 TIM8->ARR 값을 사용 → 주파수 변경에 자동 추종.
 *  duty는 0.0~1.0 범위, 범위 외 값은 클램프. 변환은 truncate(소수점 절사).
 *  ISR-safe (FreeRTOS API 미사용, race-free single 32-bit register write).
 *  ISR-hot path(예: 5kHz Loop1)에서는 본 함수 대신
 *  motor_hal_set_ccr1_raw(uint32_t ccr)를 사용할 것 — FPU lazy stacking 비용 회피.
 */
static inline void motor_hal_set_duty(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    uint32_t arr = TIM8->ARR;
    TIM8->CCR1 = (uint32_t)(duty * (float)(arr + 1U));
}

/* CCR1 raw 쓰기 (caller가 ARR 기반 계산을 직접 한 경우용).
 *  ISR-safe, FPU 연산 없음. */
static inline void motor_hal_set_ccr1_raw(uint32_t ccr)
{
    TIM8->CCR1 = ccr;
}

/* ─── DIR pin (TLE9201) ───────────────────────────────────────────────────
 *  pin_level: 0 → RESET (LOW), 1 → SET (HIGH).
 *  pin_level 은 0/1 만 유효, 비제로 값은 1로 정규화됨.
 *  의미(PUSH/PULL)는 호출처가 결정.
 */
static inline void motor_hal_dir_set(uint8_t pin_level)
{
    pin_level = (uint8_t)(!!pin_level);
    if (pin_level) {
        TLE9201_DIR_GPIO_Port->BSRR = MOTOR_HAL_BSRR_SET(TLE9201_DIR_Pin);
    } else {
        TLE9201_DIR_GPIO_Port->BSRR = MOTOR_HAL_BSRR_RESET(TLE9201_DIR_Pin);
    }
}

/* ─── DIS pin (TLE9201) ───────────────────────────────────────────────────
 *  pin_level: 0=LOW(active=bridge enable), 1=HIGH(inactive=bridge disable).
 *  pin_level 은 0/1 만 유효, 비제로 값은 1로 정규화됨.
 *  사용 시 의미가 분명한 MOTOR_HAL_BRIDGE_ENABLE / MOTOR_HAL_BRIDGE_DISABLE
 *  매크로를 우선 사용할 것 (raw level은 MOTOR_HAL_DIS_LOW/HIGH).
 */
static inline void motor_hal_dis_set(uint8_t pin_level)
{
    pin_level = (uint8_t)(!!pin_level);
    if (pin_level) {
        TLE9201_DIS_GPIO_Port->BSRR = MOTOR_HAL_BSRR_SET(TLE9201_DIS_Pin);
    } else {
        TLE9201_DIS_GPIO_Port->BSRR = MOTOR_HAL_BSRR_RESET(TLE9201_DIS_Pin);
    }
}

/* ─── 비상 정지 ─────────────────────────────────────────────────────────────
 *  duty=0 + DIS HIGH로 H-bridge 완전 차단. ISR/Task 어디서든 호출 가능.
 *  hard-limit 경로(ISR)는 별도로 enable 유지(DIS LOW)하므로 이 함수와 다름.
 */
void motor_hal_emergency_stop(void);

/* ─── Phase 2: ADS7041 SPI3 DMA chained start ─────────────────────────────
 *  Loop1 (TIM6 ISR, 5kHz) 에서 fresh ADC 값을 사용할 수 있도록 DMA 백그라운드
 *  캡처. 동작:
 *    TIM6 ISR ──→ motor_hal_iadc_start_capture()
 *                       │ CS LOW + SPI3 RX DMA 시작 (~2us)
 *                       └─→ DMA RX complete IRQ (GPDMA1_Channel1)
 *                              │ CS HIGH + raw 값 저장
 *                              └─→ 다음 TIM6 tick 에서 motor_hal_get_iadc_raw()
 *
 *  motor_hal_iadc_init() 는 HAL_SPI_Init(&hspi3) 이후 1회만 호출.
 *  start_capture / get_iadc_raw 는 ISR-safe, atomic.
 *  GPDMA1_Channel1 사용 (Channel 0 은 SPI2_TX). NVIC priority 5.
 */
void     motor_hal_iadc_init(void);
void     motor_hal_iadc_start_capture(void);
uint16_t motor_hal_get_iadc_raw(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HAL_H */
