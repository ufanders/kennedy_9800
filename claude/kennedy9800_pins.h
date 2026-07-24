/**
 * @file    kennedy9800_pins.h
 * @brief   Hardware pin, timer, and DMA assignments for the Kennedy Model 9800
 *          Digital Tape Transport interface on STM32F4xx (STM32F407 reference target).
 *
 * @details Physical interface summary (Kennedy 9800 Operation & Maintenance Manual,
 *          p/n 006-9801, Section 1.4–1.11):
 *
 *          J1 – Control connector (36-pin edge):
 *            Outputs to transport:  SFC, SRC, RWC, SLT, SWS, OFFC, OVW, DDS
 *            Inputs from transport: ONL, RWD, FPT, LDP, WEK, RDY, EOT, RNG
 *
 *          J2 – Write connector (36-pin edge):
 *            Outputs: WDS (strobe), WARS (reset pulse), WD0–WD7, WDP (parity)
 *
 *          J3 – Read connector (36-pin edge):
 *            Inputs:  RDS (strobe), RGAP (gap detect), RD0–RD7, RDP (parity), ACLD
 *
 *          Logic convention: zero-true (active-LOW) throughout (§1.5).
 *          Open-collector outputs from the Kennedy require external pull-ups.
 *          A 5V ↔ 3.3V level shifter is required between the Kennedy and STM32.
 *
 * ============================================================================
 * PIN ASSIGNMENT (edit this block to match your board layout)
 * ============================================================================
 *
 *  GPIO PORT A – Command outputs (J1), active-low
 *  -------------------------------------------------------
 *  PA0  SFC   Synchronous Forward Command
 *  PA1  SRC   Synchronous Reverse Command
 *  PA2  RWC   Rewind Command          (≥2 µs pulse)
 *  PA3  SLT   Transport Select
 *  PA4  SWS   Set Write Status
 *  PA5  OFFC  Off Line Command        (≥2 µs pulse)
 *  PA6  OVW   Overwrite               (optional)
 *  PA7  DDS   Data Density Select     (optional, dual-density models)
 *  PA8  WARS  Write Amplifier Reset   (≥2 µs pulse, TIM1_CH1 AF)
 *
 *  GPIO PORT C – Status inputs (J1), active-low, internal pull-up
 *  -------------------------------------------------------
 *  PC0  ONL   On Line
 *  PC1  RWD   Rewinding
 *  PC2  FPT   File Protect
 *  PC3  LDP   Load Point              (EXTI3)
 *  PC4  WEK   Write Enable
 *  PC5  RDY   Transport Ready
 *  PC6  EOT   End of Tape             (EXTI6)
 *  PC7  RNG   Tape Running            (EXTI7)
 *
 *  GPIO PORT E [8:0] – Write data bus (J2), active-low, DMA target
 *  -------------------------------------------------------
 *  PE0–PE7  WD0–WD7  Write Data channels 0–7
 *  PE8      WDP      Write Data Parity channel
 *  NOTE: PE[8:0] must be exclusively reserved for write data.
 *        DMA writes to GPIOE->ODR; no other signals on PE[8:0].
 *
 *  PB3  WDS  Write Data Strobe (J2)  → TIM2_CH2 alternate function (PWM output)
 *
 *  GPIO PORT D – Read data + control (J3), active-low, internal pull-up
 *  -------------------------------------------------------
 *  PD0  RDS   Read Data Strobe        (EXTI0, falling edge)
 *  PD1  RGAP  Read Gap Detect         (EXTI1, both edges)
 *  PD7  RDP   Read Data Parity channel
 *  PD8–PD15  RD0–RD7  Read Data channels 0–7
 *
 *  PB5  ACLD  Auto Clipping Level Disable (J3 output, active-low)
 *
 * ============================================================================
 * TIMER / DMA CONFIGURATION
 * ============================================================================
 *
 *  TIM2 – Write character-rate generator + WDS PWM (APB1 bus, 84 MHz on F407)
 *    PSC  = 83  → tick = 1 MHz (1 µs resolution)
 *    ARR  = 99  → period = 100 µs = 10 kHz  (800 cpi × 12.5 ips)
 *    CH2  = WDS PWM2 inverted: active-LOW from CCR2 to ARR
 *    CCR2 = 97  → WDS low for ticks 97–99 = 3 µs (≥2 µs required)
 *    DMA  = TIM2_UP → DMA1 Stream7 Ch3 → GPIOE->ODR (halfword, auto-increment src)
 *
 *  TIM1 – WARS one-pulse generator (APB2 bus, 168 MHz on F407)
 *    PSC  = 167 → tick = 1 MHz
 *    ARR  = 3   → 3 µs pulse (safe margin over 2 µs requirement)
 *    CH1  = one-pulse PWM2 inverted active-LOW, output on PA8
 *
 *  EXTI0 (PD0/RDS)   – falling edge → read byte ISR
 *  EXTI1 (PD1/RGAP)  – rising + falling edge → gap-detect ISR
 *  EXTI3 (PC3/LDP)   – both edges → load-point ISR
 *  EXTI6 (PC6/EOT)   – rising edge → end-of-tape ISR
 *
 * ============================================================================
 */

#pragma once
#include "stm32f4xx_hal.h"

/* --------------------------------------------------------------------------
 * J1 Command outputs  (STM32 → Kennedy, active-LOW)
 * -------------------------------------------------------------------------- */
#define K9800_SFC_PORT          GPIOA
#define K9800_SFC_PIN           GPIO_PIN_0

#define K9800_SRC_PORT          GPIOA
#define K9800_SRC_PIN           GPIO_PIN_1

#define K9800_RWC_PORT          GPIOA
#define K9800_RWC_PIN           GPIO_PIN_2

#define K9800_SLT_PORT          GPIOA
#define K9800_SLT_PIN           GPIO_PIN_3

#define K9800_SWS_PORT          GPIOA
#define K9800_SWS_PIN           GPIO_PIN_4

#define K9800_OFFC_PORT         GPIOA
#define K9800_OFFC_PIN          GPIO_PIN_5

#define K9800_OVW_PORT          GPIOA
#define K9800_OVW_PIN           GPIO_PIN_6

#define K9800_DDS_PORT          GPIOA
#define K9800_DDS_PIN           GPIO_PIN_7

/* WARS is driven by TIM1_CH1 alternate function on PA8.
 * For manual assertion (parity / test use) the GPIO is also accessible.  */
#define K9800_WARS_PORT         GPIOA
#define K9800_WARS_PIN          GPIO_PIN_8

/* --------------------------------------------------------------------------
 * J1 Status inputs  (Kennedy → STM32, active-LOW; configure pull-up)
 * -------------------------------------------------------------------------- */
#define K9800_ONL_PORT          GPIOC
#define K9800_ONL_PIN           GPIO_PIN_0

#define K9800_RWD_PORT          GPIOC
#define K9800_RWD_PIN           GPIO_PIN_1

#define K9800_FPT_PORT          GPIOC
#define K9800_FPT_PIN           GPIO_PIN_2

#define K9800_LDP_PORT          GPIOC
#define K9800_LDP_PIN           GPIO_PIN_3    /* EXTI3 */

#define K9800_WEK_PORT          GPIOC
#define K9800_WEK_PIN           GPIO_PIN_4

#define K9800_RDY_PORT          GPIOC
#define K9800_RDY_PIN           GPIO_PIN_5

#define K9800_EOT_PORT          GPIOC
#define K9800_EOT_PIN           GPIO_PIN_6    /* EXTI6 */

#define K9800_RNG_PORT          GPIOC
#define K9800_RNG_PIN           GPIO_PIN_7    /* EXTI7 */

/* --------------------------------------------------------------------------
 * J2 Write data bus  (STM32 → Kennedy, active-LOW)
 * PE[8:0] are EXCLUSIVELY used for DMA→GPIOE->ODR writes.
 * DO NOT assign other signals to PE[8:0].
 * -------------------------------------------------------------------------- */
#define K9800_WD_PORT           GPIOE
#define K9800_WD_MASK           ((uint16_t)0x01FFU)  /* bits [8:0] */

/* WDS is output by TIM2_CH2 on PB3 (AF1).  No GPIO macro needed at runtime. */
#define K9800_WDS_PORT          GPIOB
#define K9800_WDS_PIN           GPIO_PIN_3

/* --------------------------------------------------------------------------
 * J3 Read data + control  (Kennedy → STM32, active-LOW; configure pull-up)
 * -------------------------------------------------------------------------- */
#define K9800_RDS_PORT          GPIOD
#define K9800_RDS_PIN           GPIO_PIN_0    /* EXTI0, falling edge */

#define K9800_RGAP_PORT         GPIOD
#define K9800_RGAP_PIN          GPIO_PIN_1    /* EXTI1, both edges   */

/* RDP on PD7; RD0-RD7 on PD[15:8] (upper byte of GPIOD).
 * Raw read: idr = GPIOD->IDR
 *   data_byte = (uint8_t)(~(idr >> 8) & 0xFF)     (RD7=PD15 → bit7, ..., RD0=PD8 → bit0)
 *   parity    = (uint8_t)(~(idr >>  7) & 0x01)    (RDP on PD7)  */
#define K9800_RD_PORT           GPIOD
#define K9800_RDP_PORT          GPIOD
#define K9800_RDP_PIN           GPIO_PIN_7

/* ACLD output: holds Kennedy read electronics in normal clipping level */
#define K9800_ACLD_PORT         GPIOB
#define K9800_ACLD_PIN          GPIO_PIN_5

/* --------------------------------------------------------------------------
 * TIM2 – character-rate generator (APB1, 84 MHz on STM32F407 @ 168 MHz)
 * -------------------------------------------------------------------------- */
#define K9800_TIM2_INSTANCE     TIM2
#define K9800_TIM2_CLK_HZ       84000000UL
#define K9800_TIM2_PSC          (83U)          /* prescaler → 1 MHz tick   */
#define K9800_TIM2_ARR          (99U)          /* 100 µs period = 10 kHz   */
#define K9800_TIM2_WDS_CCR      (97U)          /* WDS active LOW ticks 97-99 */

/* --------------------------------------------------------------------------
 * TIM1 – WARS one-pulse generator (APB2, 168 MHz on STM32F407)
 * -------------------------------------------------------------------------- */
#define K9800_TIM1_INSTANCE     TIM1
#define K9800_TIM1_CLK_HZ       168000000UL
#define K9800_TIM1_PSC          (167U)         /* prescaler → 1 MHz tick   */
#define K9800_TIM1_ARR          (3U)           /* 3 µs one-pulse           */
#define K9800_TIM1_WARS_CCR     (0U)           /* no pre-delay             */

/* --------------------------------------------------------------------------
 * DMA – TIM2 update event → GPIOE->ODR write data path
 * STM32F4 Reference Manual Table 43: TIM2_UP → DMA1_Stream7_Channel3
 * -------------------------------------------------------------------------- */
#define K9800_DMA_STREAM        DMA1_Stream7
#define K9800_DMA_CHANNEL       DMA_CHANNEL_3

/* --------------------------------------------------------------------------
 * Timing constants derived from manual §1.8 (speed = 12.5 ips, 800 cpi)
 *
 *  tR  = 0.375 / speed_ips   = 30 ms   (ramp time)
 *  tGD = 0.075 / speed_ips   =  6 ms   (gap delay before data window)
 *  tWSD= tR + tGD             = 36 ms   (write start delay after SFC↑)
 *  tSD = 0.025 / speed_ips   =  2 ms   (write stop delay after last WDS)
 *  tRSD= 0.220 / speed_ips   = 17.6 ms (read stop delay, 9-track §1-8 fig)
 *  WARS delay (9-track): 8 character times after last data WDS = 800 µs
 *
 *  Character rate: 800 cpi × 12.5 ips = 10 000 chars/s → 100 µs / char
 * -------------------------------------------------------------------------- */
#define K9800_RAMP_TIME_MS          (30U)
#define K9800_GAP_DELAY_MS          (6U)
#define K9800_WRITE_START_DELAY_MS  (K9800_RAMP_TIME_MS + K9800_GAP_DELAY_MS)  /* 36 ms */
#define K9800_WRITE_STOP_DELAY_MS   (2U)
#define K9800_READ_STOP_DELAY_MS    (18U)
#define K9800_WARS_DELAY_US         (800U)      /* 8 char times after last WDS */
#define K9800_CHAR_PERIOD_US        (100U)      /* 10 kHz character rate */
#define K9800_OFFC_PULSE_US         (5U)        /* ≥2 µs required */
#define K9800_RWC_PULSE_US          (5U)        /* ≥2 µs required */
#define K9800_READY_TIMEOUT_MS      (5000U)
#define K9800_REWIND_TIMEOUT_MS     (60000U)    /* worst-case 1200 ft tape */
#define K9800_MAX_BLOCK_BYTES       (65535U)
