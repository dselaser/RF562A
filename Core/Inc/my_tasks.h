/*
 * my_task.h
 *
 *  Created on: Dec 13, 2025
 *      Author: hl3xs
 */


#ifndef MY_TASKS_H
#define MY_TASKS_H
#include "cmsis_os2.h" // osThreadId_t 타입 사용을 위해 포함
#define UART_TX_CPLT_NOTIFY_BIT (1U << 0)

/* ═══ 모터 모드 선택 ═══════════════════════════════════════
 *  1 = LA-T8 Linear Actuator (DC모터+기어, 10kΩ pot)
 *  0 = VCA (Voice Coil Actuator, ADS8325+ADS7041)
 * ═══════════════════════════════════════════════════════════*/
#ifndef USE_LINEAR_ACTUATOR
#define USE_LINEAR_ACTUATOR  0
#endif

#pragma once
#include "FreeRTOS.h"
#include "task.h"
extern TaskHandle_t gUart2TxTaskHandle;

/* GUI READY/STBY 상태: 0=STBY(스위치 무시), 1=READY(스위치 허용) */
extern volatile uint8_t g_gui_ready;

/* 물리 스위치 상태: 0=릴리즈, 1=푸시 */
extern volatile uint8_t g_sw_state;

/* ── RS485 HP 슬레이브 상태 ─────────────────────────────── */
/* HP 로컬 상태 (GUI에서 설정, Main의 @Q 폴링 응답에 사용)  */
extern volatile uint8_t g_rf_power;        /* PWR 슬라이더 0~9 */
extern volatile uint8_t g_ui_update_pending; /* RS485 → UI 업데이트 플래그 */
/* g_gui_ready, g_sw_state, g_needle_depth_mm 은 위에 선언됨 */

/* FreeRTOS Task entry prototypes */
void ADS8325_AcqTask(void *argument);

void LEDTask(void *argument);
void SysMonTask(void *argument);
void Loop1_TIM6_ISR_Handler(void);
void NTC_ADC_Task(void *argument);

#if USE_LINEAR_ACTUATOR
/* ── LA-T8 위치 PID 인터페이스 ── */
extern volatile float    g_la_pos_mm;       /* 현재 위치 (mm) */
extern volatile uint16_t g_la_pos_adc;      /* ADC1 raw 12-bit */
extern volatile float    g_la_target_mm;    /* PID 목표 (mm) */
extern volatile uint8_t  g_la_hard_limit;   /* 1=하드 리밋 트리거 */
extern volatile float    g_la_pid_output;   /* PID 출력 (디버그) */
extern volatile uint8_t  g_la_pid_active;   /* 1=PID 동작 중 */
void LA_PID_SetGains(float kp, float ki, float kd);
void LA_PID_GetGains(float *kp, float *ki, float *kd);
#endif

#endif /* MY_TASKS_H */
