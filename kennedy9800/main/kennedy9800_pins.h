/**
 * @file  kennedy9800_pins.h
 * @brief Kennedy 9800 GPIO assignments for a custom ESP32-S3-N16R8 PCB.
 *
 * Pin strategy
 * ─────────────
 * The bare ESP32-S3 chip exposes GPIO 0-21 and 38-48 for general use when
 * 16 MB Octal-Flash and 8 MB OPI-PSRAM occupy the dedicated SPI0/SPI1 pins
 * (GPIO 26-37, connected internally on WROOM-1 modules).
 *
 * USB OTG uses GPIO 19 (D−) and GPIO 20 (D+) and must not be reassigned.
 * Console output is routed through USB-CDC (sdkconfig: CONSOLE_USB_CDC),
 * so GPIO 43/44 are freed from UART0 and used for the read data bus.
 *
 * GPIO 2 and 3 are strapping pins; with I2C pull-ups they sit HIGH at boot
 * (JTAG enabled, ROM log enabled) which is compatible with both functions.
 *
 * ── J2 Write bus ─────────────────────────────────────────────────────────
 *   GPIO  4–11  = WD0–WD7   (bits 4–11 of GPIO_OUT, written in one ISR shot)
 *   GPIO 12     = WDP        (write data parity, bit 12 of GPIO_OUT)
 *   GPIO 13     = WDS        (write data strobe, asserted LOW for 3 µs/char)
 *
 * ── J1 Command outputs ───────────────────────────────────────────────────
 *   GPIO  1 = WARS    GPIO 14 = SFC     GPIO 15 = SRC
 *   GPIO 16 = RWC     GPIO 17 = SLT     GPIO 18 = SWS
 *   GPIO 21 = OFFC
 *
 * ── J1 Status inputs (via MCP23017 on I2C) ───────────────────────────────
 *   GPIO  2 = I2C SDA      GPIO  3 = I2C SCL
 *   MCP23017 GPA0-GPA7 → ONL, RWD, FPT, LDP, WEK, RDY, EOT, RNG
 *   (polled every 20 ms; see app_config.h for MCP bit assignments)
 *
 * ── J3 Read bus ──────────────────────────────────────────────────────────
 *   GPIO 38–45 = RD0–RD7   (bits 6–13 of GPIO_IN1; read as a batch in ISR)
 *   GPIO 46    = RDP        (read data parity,  bit 14 of GPIO_IN1)
 *   GPIO 47    = RDS        (read data strobe,  EXTI falling edge → ISR)
 *   GPIO 48    = RGAP       (record gap detect, EXTI both edges   → ISR)
 */
#pragma once

/* ── J2 Write bus ────────────────────────────────────────────────────────── */
#define K9800_WD0_GPIO      4
#define K9800_WD1_GPIO      5
#define K9800_WD2_GPIO      6
#define K9800_WD3_GPIO      7
#define K9800_WD4_GPIO      8
#define K9800_WD5_GPIO      9
#define K9800_WD6_GPIO      10
#define K9800_WD7_GPIO      11
#define K9800_WDP_GPIO      12
#define K9800_WDS_GPIO      13

/* GPIO_OUT bitmasks for the write bus (all in bits 4-12) */
#define K9800_WBus_SHIFT    4U
#define K9800_WBus_MASK     (0x1FFU << K9800_WBus_SHIFT)
#define K9800_WDS_MASK      (1U << K9800_WDS_GPIO)

/* ── J1 Command outputs ──────────────────────────────────────────────────── */
#define K9800_WARS_GPIO     1
#define K9800_SFC_GPIO      14
#define K9800_SRC_GPIO      15
#define K9800_RWC_GPIO      16
#define K9800_SLT_GPIO      17
#define K9800_SWS_GPIO      18
#define K9800_OFFC_GPIO     21

/* ── I2C bus for MCP23017 status expander ────────────────────────────────── */
#define K9800_I2C_SDA_GPIO  2
#define K9800_I2C_SCL_GPIO  3

/* ── J3 Read bus ─────────────────────────────────────────────────────────── */
#define K9800_RD0_GPIO      38
#define K9800_RD1_GPIO      39
#define K9800_RD2_GPIO      40
#define K9800_RD3_GPIO      41
#define K9800_RD4_GPIO      42
#define K9800_RD5_GPIO      43
#define K9800_RD6_GPIO      44
#define K9800_RD7_GPIO      45
#define K9800_RDP_GPIO      46
#define K9800_RDS_GPIO      47
#define K9800_RGAP_GPIO     48

/*
 * GPIO_IN1 bitmask helper: GPIO38 = bit 6 of GPIO.in1.val (GPIO32 = bit 0).
 * RD0-RD7 land on bits 6-13, RDP on bit 14 — read all in one register load.
 */
#define K9800_RBus_IN1_SHIFT  6U              /* GPIO38 - GPIO32 */
#define K9800_RBus_IN1_MASK   (0x1FFU << K9800_RBus_IN1_SHIFT)  /* data+parity */
#define K9800_RDS_IN1_BIT     (1U << (K9800_RDS_GPIO  - 32U))   /* bit 15 */
#define K9800_RGAP_IN1_BIT    (1U << (K9800_RGAP_GPIO - 32U))   /* bit 16 */
