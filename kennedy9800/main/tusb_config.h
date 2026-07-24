/**
 * @file  tusb_config.h
 * @brief TinyUSB configuration for USB MSC device on ESP32-S3.
 */
#pragma once

/* ── Controller ──────────────────────────────────────────────────────────── */
#define CFG_TUSB_MCU             OPT_MCU_ESP32S3
#define CFG_TUSB_RHPORT0_MODE    (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_OS              OPT_OS_FREERTOS
#define CFG_TUSB_MEM_SECTION     /* default: no special placement */
#define CFG_TUSB_MEM_ALIGN       __attribute__((aligned(4)))
#define CFG_TUSB_DEBUG           0

/* ── Class: Mass Storage only ────────────────────────────────────────────── */
#define CFG_TUD_MSC              1
#define CFG_TUD_CDC              0
#define CFG_TUD_HID              0
#define CFG_TUD_MIDI             0
#define CFG_TUD_VENDOR           0

/* MSC endpoint buffer — must fit DISK_SECTOR_SIZE (512) */
#define CFG_TUD_MSC_EP_BUFSIZE   512
