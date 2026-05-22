/**
 * Per-task heartbeat watchdog.
 *
 * 각 task 의 메인 루프 상단에서 Wdg_TaskAlive() 호출.
 * 단일 WatchdogTask 가 주기적으로 모든 task heartbeat 확인 → 모두 살아있을 때만 IWDG refresh.
 * 어느 한 task 라도 hang 되면 IWDG (timeout 4초) 가 시스템 리셋.
 *
 * Surge/Burst 시험 시 통신·UI task hang 감지 목적.
 */
#ifndef WDG_H
#define WDG_H

#include <stdint.h>

typedef enum {
    WDG_TASK_LVGL = 0,
    WDG_TASK_RS485,
    WDG_TASK_ADS8325_ACQ,
    WDG_TASK_HPSWITCH,
    WDG_TASK_COUNT
} wdg_task_id_t;

void Wdg_TaskAlive(wdg_task_id_t id);
void WatchdogTask(void *argument);

/* 부트 카운터 — main 시작 시 1회 호출. TAMP->BKP30R 에 reset 누적 횟수 저장.
 *   - BKP0R(update magic), BKP31R(fault code) 충돌 회피
 *   - 30초 이상 안정 운영되면 WatchdogTask 가 0 으로 클리어
 *   - 5회 이상 누적되면 콘솔 경고 출력 */
void Wdg_BootCounter_Init(void);
uint32_t Wdg_BootCounter_Get(void);

#endif
