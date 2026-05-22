#include "wdg.h"
#include "stm32h5xx.h"
#include "cmsis_os2.h"

static volatile uint32_t s_counters[WDG_TASK_COUNT];

/* 부트 카운터 — TAMP->BKP30R. BKP0R(update magic), BKP31R(fault code) 충돌 회피.
 *   상위 16비트: 매직 0xB007
 *   하위 16비트: 부트 카운터 */
#define BOOT_CNT_MAGIC   0xB0070000U
#define BOOT_CNT_MASK    0x0000FFFFU

static volatile uint32_t s_boot_count_cached = 0;

void Wdg_BootCounter_Init(void)
{
    HAL_PWR_EnableBkUpAccess();
    uint32_t v = TAMP->BKP30R;
    uint32_t cnt;
    if ((v & 0xFFFF0000U) == BOOT_CNT_MAGIC) {
        cnt = (v & BOOT_CNT_MASK) + 1U;
    } else {
        cnt = 1U;
    }
    if (cnt > BOOT_CNT_MASK) cnt = BOOT_CNT_MASK;
    TAMP->BKP30R = BOOT_CNT_MAGIC | cnt;
    s_boot_count_cached = cnt;
}

uint32_t Wdg_BootCounter_Get(void)
{
    return s_boot_count_cached;
}

static void bootcnt_clear(void)
{
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP30R = BOOT_CNT_MAGIC | 0U;
}

void Wdg_TaskAlive(wdg_task_id_t id)
{
    if ((unsigned)id < (unsigned)WDG_TASK_COUNT) {
        s_counters[id]++;
    }
}

/* SerialPutString 가 RF562 에는 없으므로 huart1 직접 사용 */
extern UART_HandleTypeDef huart1;
static void wdg_log(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s,
                      (uint16_t)__builtin_strlen(s), 100);
}

void WatchdogTask(void *argument)
{
    (void)argument;

    if (s_boot_count_cached >= 5U) {
        wdg_log("[WDG] WARNING: repeated reset detected (>= 5)\r\n");
    }

    /* Startup grace: 다른 task 들이 첫 heartbeat 를 찍을 때까지 IWDG 를 직접 refresh.
     * LVGL splash, ui_init, ADS8325 시작 등 시간이 좀 걸리므로 3초 잡음. */
    const uint32_t STARTUP_GRACE_MS = 3000;
    uint32_t start = osKernelGetTickCount();
    while ((osKernelGetTickCount() - start) < STARTUP_GRACE_MS) {
        IWDG->KR = 0xAAAA;
        osDelay(100);
    }

    uint32_t last[WDG_TASK_COUNT];
    for (int i = 0; i < WDG_TASK_COUNT; i++) last[i] = s_counters[i];

    /* 안정 카운터 — STABLE_MS 동안 모든 task 가 heartbeat 정상이면 boot counter 클리어 */
    const uint32_t STABLE_MS = 30000U;
    uint32_t stable_elapsed = 0;
    int      bootcnt_cleared = 0;

    /* 500ms 마다 heartbeat 확인. IWDG timeout 4초 → 약 8회 기회 */
    const uint32_t CHECK_PERIOD_MS = 500;
    for (;;) {
        osDelay(CHECK_PERIOD_MS);
        int all_alive = 1;
        for (int i = 0; i < WDG_TASK_COUNT; i++) {
            if (s_counters[i] == last[i]) {
                all_alive = 0;
            }
            last[i] = s_counters[i];
        }
        if (all_alive) {
            IWDG->KR = 0xAAAA;
            if (!bootcnt_cleared) {
                stable_elapsed += CHECK_PERIOD_MS;
                if (stable_elapsed >= STABLE_MS) {
                    bootcnt_clear();
                    bootcnt_cleared = 1;
                }
            }
        } else {
            stable_elapsed = 0;
        }
        /* hang 되면 refresh 안 함 → ~4초 후 IWDG reset */
    }
}
