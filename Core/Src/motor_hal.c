/*
 * motor_hal.c
 *
 *  Cascade control Phase 1 — PWM/GPIO 레지스터 접근 격리.
 *  Phase 2 — ADS7041 SPI3 DMA chained start (5kHz fresh ADC).
 *  inline wrapper의 본체는 motor_hal.h에 있음.
 */
#include "motor_hal.h"
#include "spi.h"   /* hspi3 */

/* ─── 비상 정지 ─────────────────────────────────────────────────────────────
 *  ISR/Task 양쪽에서 호출 가능. FreeRTOS API 미사용.
 *  의미: 진짜 비상정지 = duty=0 + bridge 완전 차단(DIS HIGH).
 *  순서: PWM duty=0 먼저 → DIS HIGH (출력 차단)
 *      → 차단 시점에 모터 권선 전류가 이미 0에 가까운 상태가 되도록.
 *  hard-limit 경로(ISR)는 enable을 유지(DIS LOW)하므로 별도 처리.
 */
void motor_hal_emergency_stop(void)
{
    motor_hal_set_duty(0.0f);
    motor_hal_dis_set(MOTOR_HAL_BRIDGE_DISABLE);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Phase 2 — ADS7041 SPI3 RX DMA (GPDMA1 Channel 1)
 * ═══════════════════════════════════════════════════════════════════════════
 *  TIM6 ISR(5kHz) 매 tick:
 *    motor_hal_iadc_start_capture()  → CS LOW + DMA 시작
 *  DMA RX 완료 (~2us):
 *    HAL_SPI_RxCpltCallback         → CS HIGH + raw 값 저장
 *  다음 TIM6 tick:
 *    motor_hal_get_iadc_raw()       → 직전 캡처값 (1-tick latency, 200us)
 * ═══════════════════════════════════════════════════════════════════════════*/
DMA_HandleTypeDef       motor_hal_hdma_spi3_rx;   /* IT 핸들러에서 참조 (extern) */
DMA_HandleTypeDef       motor_hal_hdma_spi3_tx;   /* TX DMA (dummy 클럭 생성용) */
static uint16_t         s_iadc_dma_rx_buf;        /* DMA 수신 버퍼 (16비트) */
static const uint16_t   s_iadc_dma_tx_dummy = 0;  /* SCK 생성용 더미 TX */
static volatile uint16_t s_iadc_raw     = 0;      /* 12비트 raw, atomic read */
static volatile uint8_t  s_iadc_busy    = 0;      /* DMA 진행 중 플래그 */
static volatile uint8_t  s_iadc_initialized = 0;  /* init 완료 플래그 */

void motor_hal_iadc_init(void)
{
    /* GPDMA1 Channel 1: SPI3 RX, peripheral → memory, halfword(16비트) 1프레임 */
    motor_hal_hdma_spi3_rx.Instance                  = GPDMA1_Channel1;
    motor_hal_hdma_spi3_rx.Init.Request              = GPDMA1_REQUEST_SPI3_RX;
    motor_hal_hdma_spi3_rx.Init.BlkHWRequest         = DMA_BREQ_SINGLE_BURST;
    motor_hal_hdma_spi3_rx.Init.Direction            = DMA_PERIPH_TO_MEMORY;
    motor_hal_hdma_spi3_rx.Init.SrcInc               = DMA_SINC_FIXED;
    motor_hal_hdma_spi3_rx.Init.DestInc              = DMA_DINC_FIXED;
    motor_hal_hdma_spi3_rx.Init.SrcDataWidth         = DMA_SRC_DATAWIDTH_HALFWORD;
    motor_hal_hdma_spi3_rx.Init.DestDataWidth        = DMA_DEST_DATAWIDTH_HALFWORD;
    motor_hal_hdma_spi3_rx.Init.Priority             = DMA_HIGH_PRIORITY;
    motor_hal_hdma_spi3_rx.Init.SrcBurstLength       = 1;
    motor_hal_hdma_spi3_rx.Init.DestBurstLength      = 1;
    motor_hal_hdma_spi3_rx.Init.TransferAllocatedPort= DMA_SRC_ALLOCATED_PORT0
                                                     | DMA_DEST_ALLOCATED_PORT0;
    motor_hal_hdma_spi3_rx.Init.TransferEventMode    = DMA_TCEM_BLOCK_TRANSFER;
    motor_hal_hdma_spi3_rx.Init.Mode                 = DMA_NORMAL;

    if (HAL_DMA_Init(&motor_hal_hdma_spi3_rx) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_DMA_ConfigChannelAttributes(&motor_hal_hdma_spi3_rx,
                                        DMA_CHANNEL_NPRIV) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&hspi3, hdmarx, motor_hal_hdma_spi3_rx);

    /* GPDMA1 Channel 2: SPI3 TX (더미 0 송신 → SCK 생성 용도, master 2LINES 모드)
     *  Channel 0=SPI2_TX, Channel 1=SPI3_RX 이미 사용 중이므로 Channel 2 신규. */
    motor_hal_hdma_spi3_tx.Instance                  = GPDMA1_Channel2;
    motor_hal_hdma_spi3_tx.Init.Request              = GPDMA1_REQUEST_SPI3_TX;
    motor_hal_hdma_spi3_tx.Init.BlkHWRequest         = DMA_BREQ_SINGLE_BURST;
    motor_hal_hdma_spi3_tx.Init.Direction            = DMA_MEMORY_TO_PERIPH;
    motor_hal_hdma_spi3_tx.Init.SrcInc               = DMA_SINC_FIXED;  /* dummy 1개 반복 */
    motor_hal_hdma_spi3_tx.Init.DestInc              = DMA_DINC_FIXED;
    motor_hal_hdma_spi3_tx.Init.SrcDataWidth         = DMA_SRC_DATAWIDTH_HALFWORD;
    motor_hal_hdma_spi3_tx.Init.DestDataWidth        = DMA_DEST_DATAWIDTH_HALFWORD;
    motor_hal_hdma_spi3_tx.Init.Priority             = DMA_HIGH_PRIORITY;
    motor_hal_hdma_spi3_tx.Init.SrcBurstLength       = 1;
    motor_hal_hdma_spi3_tx.Init.DestBurstLength      = 1;
    motor_hal_hdma_spi3_tx.Init.TransferAllocatedPort= DMA_SRC_ALLOCATED_PORT0
                                                     | DMA_DEST_ALLOCATED_PORT0;
    motor_hal_hdma_spi3_tx.Init.TransferEventMode    = DMA_TCEM_BLOCK_TRANSFER;
    motor_hal_hdma_spi3_tx.Init.Mode                 = DMA_NORMAL;

    if (HAL_DMA_Init(&motor_hal_hdma_spi3_tx) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_DMA_ConfigChannelAttributes(&motor_hal_hdma_spi3_tx,
                                        DMA_CHANNEL_NPRIV) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&hspi3, hdmatx, motor_hal_hdma_spi3_tx);

    /* NVIC: 두 채널 + SPI3 에러 */
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(SPI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);

    /* CS HIGH 초기 상태 보장 */
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

    s_iadc_busy = 0;
    s_iadc_initialized = 1;
}

void motor_hal_iadc_start_capture(void)
{
    /* ISR-safe. 직전 transfer 진행 중이거나 init 미완료면 skip (드롭). */
    if (!s_iadc_initialized) return;
    if (s_iadc_busy) return;
    if (hspi3.State != HAL_SPI_STATE_READY) return;

    s_iadc_busy = 1;
    /* CS LOW → SPI3 TransmitReceive DMA: 더미 0 TX 로 SCK 생성 + RX 캡처
     *  ADS7041: 16 SCK 사이클 동안 12비트 데이터 출력 (MSB-aligned).
     *  master 2LINES 모드에서는 Receive_DMA 단독으로 SCK 생성 불가 →
     *  TransmitReceive_DMA (옛 blocking ADS7041_ReadRaw 와 동일 패턴). */
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(&hspi3,
                                    (uint8_t*)&s_iadc_dma_tx_dummy,
                                    (uint8_t*)&s_iadc_dma_rx_buf,
                                    1) != HAL_OK) {
        /* 시작 실패: CS 복원 + busy 해제 (다음 tick 재시도) */
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
        s_iadc_busy = 0;
    }
}

uint16_t motor_hal_get_iadc_raw(void)
{
    return s_iadc_raw;   /* 16비트 atomic read */
}

/* HAL DMA 완료 콜백 (HAL_DMA_IRQHandler → SPI 내부 → 이 함수)
 *  TransmitReceive_DMA 사용이므로 HAL_SPI_TxRxCpltCallback 가 호출됨.
 *  weak 선언이므로 여기서 정의하면 모든 SPI TxRx 완료 처리. hspi 매칭 필수. */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) {
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
        /* ADS7041: 16비트 프레임의 상위 12비트가 데이터 → 4비트 우향 시프트 */
        s_iadc_raw  = (uint16_t)((s_iadc_dma_rx_buf >> 4U) & 0x0FFFU);
        s_iadc_busy = 0;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) {
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
        s_iadc_busy = 0;
    }
}
