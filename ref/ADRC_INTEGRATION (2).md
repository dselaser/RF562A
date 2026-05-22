# RF562 VCA 전류 루프 ADRC 통합 가이드

## 1. 파일 배치

```
RF562/
├── Core/
│   ├── Inc/
│   │   └── adrc1.h              ← 추가
│   └── Src/
│       ├── adrc1.c              ← 추가
│       └── current_loop.c       ← 추가 (기존 PI 루프 대체)
└── CLAUDE.md                    ← 아래 항목 추가
```

## 2. CubeMX 설정 체크리스트

| 항목 | 권장 설정 |
|---|---|
| TIM1 | Center-aligned mode 1, ARR로 20 kHz, complementary PWM, dead-time ≥ 드라이버 사양 |
| TIM1 TRGO | Update event → ADC1 injected trigger |
| ADC1 | Injected group, TIM1 TRGO, 1 channel (current sense), DMA OFF (IRQ만), 12-bit |
| NVIC | ADC global IRQ priority = configMAX_SYSCALL_INTERRUPT_PRIORITY 보다 높게 (FreeRTOS API 호출 안 함) |
| FPU | Enabled (`-mfloat-abi=hard -mfpu=fpv4-sp-d16`) |
| 최적화 | `-O2` 권장. `-O0`로는 50 µs 루프에 빠듯함 |

## 3. 첫 운전 절차 (안전 우선)

**Phase 1 — 개루프 식별 (게인 측정)**
1. `g_enabled = 0` 상태로 시스템 가동
2. 디버거에서 `write_duty_signed(0.05f)` (5% duty 스텝)을 강제 호출
3. SWV/RTT로 `dbg_i_meas_A` 200 µs 간격 기록
4. 응답 초기 기울기 `a = di/dt` 측정 → `L_measured = u_volts / a`
5. `B0_NOMINAL = 1/L_measured`로 매크로 갱신, **`B0_DESIGN`은 그 60%**

**Phase 2 — 점진적 폐루프 활성화**
1. `WC_HZ = 200.0f` (기존 PI 대역폭의 1/4)로 시작
2. `WO_RAD_S = 3.0f * WC_RAD_S` (옵저버 보수적으로)
3. `CurrentLoop_Enable()` 후 `CurrentLoop_SetReference(0.5f)` (저전류)
4. 안정 확인 → `WO`를 5×, 7×, 10×로 단계적 상승, 측정 노이즈가 PWM에 보일 때 직전 값에서 정지
5. `WC_HZ`를 400, 800, 1200 Hz로 점진 상승, overshoot/oscillation 직전에서 정지

**Phase 3 — 외란 강건성 검증**
- 빈 핀 / 실리콘 팬텀 / 돼지 피부 샘플로 동일 게인 운전
- `dbg_z2_dist` 추적: 정상 동작 시 부하에 비례, 발산 시 게인 과대

## 4. 캐스케이드 통합 (위치 루프 추가 시)

```c
/* 외부 위치 루프 task에서 */
float i_cmd = ADRC2_Update(&g_adrc_position, pos_ref, pos_meas);
CurrentLoop_SetReference(i_cmd);
```

위치 루프 ADRC는 2차(`adrc2.c` - 향후 작성)로 구성하고, 외부 루프 `wc`는 **내부 전류 루프 `wc`의 1/5 이하**로 제한해야 캐스케이드가 안정합니다.

## 5. 튜닝 디버깅 표

| 증상 | 원인 후보 | 조치 |
|---|---|---|
| 정상상태 오차 잔존 | `b0` 과소 추정 너무 큼 | `B0_DESIGN`을 0.7~0.8 × `B0_NOMINAL`로 |
| 고주파 채터링 | `wo` 과대 → 노이즈 증폭 | `wo` 30% 감소, 전류 센스 필터 추가 |
| 스텝 응답 오버슈트 | `wc/wo` 비율 부적절 | `wo ≥ 5·wc` 보장 |
| 부하 외란 후 느린 회복 | `wo` 부족 | `wo` 단계적 상승 |
| 포화 후 비정상 회복 | anti-windup 미동작 | `u_lim_prev` 갱신 확인 (adrc1.c 마지막 줄) |
| 발산 | `b0` 부호 오류 | H-bridge 극성 또는 ADC 부호 점검 |

## 6. IEC 62304 관점 메모

- ADRC는 결정적 차분 방정식 → SOUP 아님, full validation 가능
- Risk file 추가 항목:
  - "ESO divergence → over-current" → 별도 HW 과전류 인터록 필수 (이미 존재 가정)
  - "Coefficient corruption" → 부팅 시 `compute_coeffs` 재계산 + sanity check 권장
- Verification: open-loop ID 결과 vs 명목 파라미터 비교를 IQ/OQ 절차에 포함
