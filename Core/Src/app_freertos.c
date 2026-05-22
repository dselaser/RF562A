/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "uart_cmd.h"
#include "my_tasks.h"
#include "tim.h"

#include "cnnx_proj.h"
#include "lvgl/lvgl.h"
#include "lvgl/demos/lv_demos.h"
#include "ui.h"
#include "lv_port_indev.h"
#include "adxl345.h"
#include "ST7789V2.h"
#include "speaker.h"
#include "wdg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include "stm32h5xx.h"   // 디바이스 헤더에 맞게
#include "core_cm33.h"   // 보통 포함됨

void ADS8325_AcqTask(void *argument);

void vConfigureTimerForRunTimeStats(void)
{
  // Trace enable (TRCENA)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // (일부 코어/설정에서 필요할 수 있음) DWT unlock 레지스터가 있는 경우만
  #ifdef DWT_LAR
  DWT->LAR = 0xC5ACCE55;
  #endif

  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // enable
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

	// LEDTask
const osThreadAttr_t led_attr = {
	.name       = "01_LEDTask",
	.priority   = osPriorityNormal,
	.stack_size = 512
};

	 // SystemMonitorTask (Heap / Queue 상태 모니터링) - CubeMX 관리로 전환
__attribute__((unused))
static const osThreadAttr_t mon_attr = {
	.name       = "02_SysMonTask",
	.priority   = osPriorityLow,
	.stack_size = 512*2
};

// ADS7041 UART2 Task (비활성 - SPI3 HPSwitch와 충돌)
__attribute__((unused))
static const osThreadAttr_t ads7041TaskAttr = {
    .name       = "03_ADS7041_U2",
    .priority   = osPriorityNormal,
    .stack_size = 512*2
};

// ADXL345 엑셀로미터 UART2 Task (비활성 - UART2 RS485와 충돌)
__attribute__((unused))
static const osThreadAttr_t adxl345TaskAttr = {
    .name       = "04_ADXL345_U2",
    .priority   = osPriorityNormal,
    .stack_size = 512*2
};

// ADXL345 방향 측정 UART1 Task - CubeMX 관리로 전환
__attribute__((unused))
static const osThreadAttr_t adxl345Uart1Attr = {
    .name       = "05_ADXL345_U1",
    .priority   = osPriorityLow,
    .stack_size = 512*3      /* atan2f/sqrtf 사용 → 여유 확보 */
};

// Speaker 사운드 출력 Task - CubeMX 관리로 전환
__attribute__((unused))
static const osThreadAttr_t speakerAttr = {
    .name       = "09_Speaker",
    .priority   = osPriorityLow,
    .stack_size = 512
};

/*
osThreadId_t ads8325TaskHandle;
const osThreadAttr_t ads8325_attr = {
  .name = "5_ADS8325_UART2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512*2
};
*/

/* Definitions for ADS8325_Acq */
osThreadId_t ads8325AcqTaskHandle;
const osThreadAttr_t ads8325AcqTask_attr = {
  .name = "07_ADS8325_Acq",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 2
};

/* Definitions for ADS8325_RS485 */
osThreadId_t ads8325TaskHandle;
const osThreadAttr_t ads8325_attr = {
  .name = "06_RS485",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 1
};


// HPSwitchTask - CubeMX 관리로 전환
__attribute__((unused))
static const osThreadAttr_t hp_attr = {
  .name       = "08_HPSwitch",
  .priority   = osPriorityHigh,
  .stack_size = 512*6
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "10_Default",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for _lvglHandler */
osThreadId_t _lvglHandlerHandle;
const osThreadAttr_t _lvglHandler_attributes = {
  .name = "11_LVGL",
  /* Normal 로 올렸을 때 VCA 동작 중 LCD reboot 발생 — motor PWM EMI 와 LCD SPI traffic
   * 동시 발생으로 ST7789V2 controller 가 노이즈에 의해 리셋되는 것으로 추정.
   * Low 로 복원 — VCA 동작 중에는 LVGL 이 자연스럽게 양보됨 → SPI traffic 감소. */
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 2048 * 4
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "01_LEDTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for SysMonTask */
osThreadId_t SysMonTaskHandle;
const osThreadAttr_t SysMonTask_attributes = {
  .name = "02_SysMonTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for ADXL345_U1 */
osThreadId_t ADXL345_U1Handle;
const osThreadAttr_t ADXL345_U1_attributes = {
  .name = "05_ADXL345_U1",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 384 * 4
};
/* Definitions for RS485_Task */
osThreadId_t RS485_TaskHandle;
const osThreadAttr_t RS485_Task_attributes = {
  .name = "06_RS485",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for ADS8325_Acq */
osThreadId_t ADS8325_AcqHandle;
const osThreadAttr_t ADS8325_Acq_attributes = {
  .name = "07_ADS8325_Acq",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for HPSwitch */
osThreadId_t HPSwitchHandle;
const osThreadAttr_t HPSwitch_attributes = {
  .name = "08_HPSwitch",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 768 * 4
};
/* Definitions for SpeakerTask */
osThreadId_t SpeakerTaskHandle;
const osThreadAttr_t SpeakerTask_attributes = {
  .name = "09_Speaker",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for WatchdogTask — surge/burst hang 검출용 per-task heartbeat 감시 */
osThreadId_t WatchdogTaskHandle;
const osThreadAttr_t WatchdogTask_attributes = {
  .name = "13_Watchdog",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for _lvglSmphr */
osSemaphoreId_t _lvglSmphrHandle;
const osSemaphoreAttr_t _lvglSmphr_attributes = {
  .name = "_lvglSmphr"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LEDTask(void *argument);
void SysMonTask(void *argument);
void StartDefaultTask(void *argument);
void HPSwitchTask(void *argument);

void SysMonTask(void *argument);
void ADS8325_RS485_Task(void *argument);
void ADS8325_AcqTask(void *argument);
// ADS7041 / ADXL345 UART Task 프로토타입
void ADS7041_UART2_Task(void *argument);
void ADXL345_UART2_Task(void *argument);

/* TASK STATS auto-prime (uart_cmd.c) */
extern void UART_Cmd_PrimeTaskStatsBaseline(void);
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	 UART_Cmd_Init();      // RX IT 시작 + UARTCmd task 생성
	 Speaker_Init();

	 /* Task 생성은 CubeMX가 아래에서 관리 (osThreadNew 호출) */
	 /* 비활성 Task: ADS7041_U2 (SPI3 충돌), ADXL345_U2 (UART2 충돌) */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* creation of _lvglSmphr */
  _lvglSmphrHandle = osSemaphoreNew(1, 1, &_lvglSmphr_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of _lvglHandler */
  _lvglHandlerHandle = osThreadNew(lvglHandler, NULL, &_lvglHandler_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(LEDTask_Entry, NULL, &LEDTask_attributes);

  /* creation of SysMonTask */
  SysMonTaskHandle = osThreadNew(SysMonTask_Entry, NULL, &SysMonTask_attributes);

  /* creation of ADXL345_U1 */
  ADXL345_U1Handle = osThreadNew(ADXL345_UART1_Task, NULL, &ADXL345_U1_attributes);

  /* creation of RS485_Task */
  RS485_TaskHandle = osThreadNew(ADS8325_RS485_Task, NULL, &RS485_Task_attributes);

  /* creation of ADS8325_Acq */
  ADS8325_AcqHandle = osThreadNew(ADS8325_AcqTask, NULL, &ADS8325_Acq_attributes);

  /* creation of HPSwitch */
  HPSwitchHandle = osThreadNew(HPSwitchTask, NULL, &HPSwitch_attributes);

  /* creation of SpeakerTask */
  SpeakerTaskHandle = osThreadNew(SpeakerTask_Entry, NULL, &SpeakerTask_attributes);

  /* creation of WatchdogTask — heartbeat 검사 후 IWDG refresh */
  WatchdogTaskHandle = osThreadNew(WatchdogTask, NULL, &WatchdogTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* NTC ADC 태스크 — 미사용 (센서 미연결, UART1 출력만 어지럽힘) → 비활성 */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  // 리셋 후 자동 기준점 설정: 't'를 1번만 눌러도 stats 출력되게
  //osDelay(300);
 // UART_Cmd_PrimeTaskStatsBaseline();


  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  osDelay(100);

  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_lvglHandler */
/**
* @brief Function implementing the _lvglHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lvglHandler */
void lvglHandler(void *argument)
{
  /* USER CODE BEGIN _lvglHandler */
#ifdef   __CNNX_FREERTOS_ON
  // lvgl demos
  // lv_demo_widgets();
  //  Squareline Studio UI
  ui_init() ;

  /* ── Splash 화면을 LCD에 렌더링한 후 백라이트 ON (노이즈 방지) ── */
  if (osSemaphoreAcquire(_lvglSmphrHandle, osWaitForever) == osOK) {
      lv_timer_handler();                  // Splash 화면 첫 프레임 렌더링
      osSemaphoreRelease(_lvglSmphrHandle);
  }
  DEV_BL_PIN = 800 - 1;                   // 백라이트 80% ON

  lv_port_indev_enable() ;   // enable touch input after UI objects are created
#endif


  /* ── 화면 자동 회전 / 스위치 배경색 상태 추적 ──────────────── */
  extern volatile uint8_t g_hp_locked;
  static uint8_t last_rot = 0;
  static uint8_t last_sw  = 0xFF;   /* 초기값: 강제 첫 업데이트 */

  /* Infinite loop */
  for(;;)
  {
    Wdg_TaskAlive(WDG_TASK_LVGL);
    // Acquire GUI mutex if used
  	if( osSemaphoreAcquire(_lvglSmphrHandle, 2) == osOK ) {

      /* ── ADXL345 기반 180° 자동 회전 (Lock/STANDBY 포함) ──── */
      {
          uint8_t cur_rot = g_screen_rotated;
          if (cur_rot != last_rot) {
              ST7789V2_SetRotation(cur_rot);
              lv_obj_invalidate(lv_scr_act());
              last_rot = cur_rot;
          }
      }

      /* ── Lock 화면 전환 ──────────────────────────────────── */
      {
          static uint8_t s_lock_screen_active = 0;
          static lv_obj_t *s_lock_scr = NULL;
          if (g_hp_locked && !s_lock_screen_active) {
              /* Lock → 별도 스크린: 검은 배경 + 상태 텍스트 */
              s_lock_scr = lv_obj_create(NULL);
              lv_obj_set_style_bg_color(s_lock_scr, lv_color_black(), LV_PART_MAIN);
              lv_obj_set_style_bg_opa(s_lock_scr, LV_OPA_COVER, LV_PART_MAIN);
              lv_obj_clear_flag(s_lock_scr, LV_OBJ_FLAG_SCROLLABLE);
              lv_obj_t *lbl = lv_label_create(s_lock_scr);
              if (!g_gui_ready) {
                  /* 1PIN 등 Standby+Lock → "STANDBY" (H1 폰트) */
                  lv_label_set_text(lbl, "STANDBY");
                  lv_obj_set_style_text_font(lbl, &ui_font_H1, 0);
              } else {
                  /* 일반 Lock → "LOCK" */
                  lv_label_set_text(lbl, "LOCK");
                  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
              }
              lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
              lv_obj_center(lbl);
              lv_scr_load(s_lock_scr);
              s_lock_screen_active = 1;
          } else if (!g_hp_locked && s_lock_screen_active) {
              /* Unlock → 메인 화면 복원 */
              ui_Screen1_screen_init();
              lv_scr_load(ui_Screen1);
              if (s_lock_scr) { lv_obj_del(s_lock_scr); s_lock_scr = NULL; }
              s_lock_screen_active = 0;
              last_sw = 0xFF;  /* 배경색 강제 갱신 */
          }
      }

      /* ── 스위치 누름 → 배경색 흰/검 전환 (READY 모드 + Lock 아닐 때만) ──── */
      if (!g_hp_locked) {
          uint8_t cur_sw = (g_gui_ready && g_sw_state) ? 1 : 0;
          if (cur_sw != last_sw) {
              lv_color_t bg = cur_sw ? lv_color_white() : lv_color_black();
              lv_obj_set_style_bg_color(ui_Screen1, bg, LV_PART_MAIN | LV_STATE_DEFAULT);
              last_sw = cur_sw;
          }
      }

      lv_timer_handler(); // Handles LVGL tasks
      osSemaphoreRelease( _lvglSmphrHandle ) ; // Release mutex
  	}

    osDelay(1);
  }
  /* USER CODE END _lvglHandler */
}

/* USER CODE BEGIN Header_LEDTask_Entry */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask_Entry */
void LEDTask_Entry(void *argument)
{
  /* USER CODE BEGIN LEDTask */
  extern void LEDTask(void *argument);
  LEDTask(argument);
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_SysMonTask_Entry */
/**
* @brief Function implementing the SysMonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SysMonTask_Entry */
void SysMonTask_Entry(void *argument)
{
  /* USER CODE BEGIN SysMonTask */
  extern void SysMonTask(void *argument);
  SysMonTask(argument);
  /* USER CODE END SysMonTask */
}

/* USER CODE BEGIN Header_ADXL345_UART1_Task */
/**
* @brief Function implementing the ADXL345_U1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADXL345_UART1_Task */
void ADXL345_UART1_Task(void *argument)
{
  /* USER CODE BEGIN ADXL345_U1 */
  extern void ADXL345_UART1_Task_impl(void *argument);
  ADXL345_UART1_Task_impl(argument);
  /* USER CODE END ADXL345_U1 */
}

/* USER CODE BEGIN Header_ADS8325_RS485_Task */
/**
* @brief Function implementing the RS485_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADS8325_RS485_Task */
void ADS8325_RS485_Task(void *argument)
{
  /* USER CODE BEGIN RS485_Task */
  extern void ADS8325_RS485_Task_impl(void *argument);
  ADS8325_RS485_Task_impl(argument);
  /* USER CODE END RS485_Task */
}

/* USER CODE BEGIN Header_ADS8325_AcqTask */
/**
* @brief Function implementing the ADS8325_Acq thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADS8325_AcqTask */
void ADS8325_AcqTask(void *argument)
{
  /* USER CODE BEGIN ADS8325_Acq */
  extern void ADS8325_AcqTask_impl(void *argument);
  ADS8325_AcqTask_impl(argument);
  /* USER CODE END ADS8325_Acq */
}

/* USER CODE BEGIN Header_HPSwitchTask */
/**
* @brief Function implementing the HPSwitch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HPSwitchTask */
void HPSwitchTask(void *argument)
{
  /* USER CODE BEGIN HPSwitch */
  extern void HPSwitchTask_impl(void *argument);
  HPSwitchTask_impl(argument);
  /* USER CODE END HPSwitch */
}

/* USER CODE BEGIN Header_SpeakerTask_Entry */
/**
* @brief Function implementing the SpeakerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SpeakerTask_Entry */
void SpeakerTask_Entry(void *argument)
{
  /* USER CODE BEGIN SpeakerTask */
  extern void SpeakerTask(void *argument);
  SpeakerTask(argument);
  /* USER CODE END SpeakerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Stack overflow hook — UART1 콘솔에 태스크 이름 출력 후 정지 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    /* 최소한의 디버그 출력 (인터럽트 비활성 상태이므로 폴링 전송) */
    extern UART_HandleTypeDef huart1;
    const char msg[] = "\r\n!!! STACK OVERFLOW: ";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 50);
    HAL_UART_Transmit(&huart1, (uint8_t*)pcTaskName, 16, 50);
    const char nl[] = "\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)nl, 2, 50);
    for (;;) { __NOP(); }
}

/* USER CODE END Application */

