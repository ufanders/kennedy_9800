/**
 * @file  app_config.h
 * @brief Master configuration — ESP32-S3-N16R8 Kennedy 9800 USB/LTFS firmware.
 */
#pragma once
#include <stdint.h>

/* ── System ──────────────────────────────────────────────────────────────── */
#define APP_CPU_HZ              240000000UL

/* ── PSRAM layout (8 MB OPI PSRAM, addressed via heap_caps SPIRAM) ────────
 * Only metadata lives in PSRAM; file content is never cached.
 *   FAT32 metadata : BPB + FSInfo + 2× FAT tables + root cluster  ≈ 530 KB
 *   LTFS index     : 256 entries × ~260 B                         ≈  66 KB
 *   Tape I/O buf   : one tape block (4 KB) for staging
 * ──────────────────────────────────────────────────────────────────────── */
#define PSRAM_FAT_SIZE          (560UL  * 1024UL)
#define PSRAM_LTFS_SIZE         (128UL  * 1024UL)
#define PSRAM_STAGING_SIZE      (4UL    * 1024UL)

/* ── USB virtual disk ────────────────────────────────────────────────────
 * 128 MB FAT32 volume.  Metadata LBAs → PSRAM.  Data LBAs → tape.
 * ──────────────────────────────────────────────────────────────────────── */
#define DISK_SECTOR_SIZE        512U
#define DISK_SECTOR_COUNT       (128UL * 1024UL * 2UL)   /* 262 144 sectors */

/* FAT32 geometry: 4 KB clusters, 32 reserved sectors */
#define FAT32_RESERVED_SECS     32U
#define FAT32_CLUSTER_SECS      8U          /* 4 096 B / 512 B = 8 sectors */
#define FAT32_ROOT_CLUSTER      2U
#define FAT32_DATA_CLUSTERS     ((DISK_SECTOR_COUNT - FAT32_RESERVED_SECS) \
                                  / FAT32_CLUSTER_SECS)
#define FAT32_FAT_SECS          ((FAT32_DATA_CLUSTERS * 4U + 511U) / 512U)
#define FAT32_DATA_LBA          (FAT32_RESERVED_SECS + 2U * FAT32_FAT_SECS)

/* ── LTFS limits ─────────────────────────────────────────────────────────── */
#define LTFS_MAX_FILES          256U
#define LTFS_FILENAME_MAX       240U
#define LTFS_BLOCK_BYTES        4096U
#define LTFS_TAPE_MAX_BLOCKS    32768U      /* ~128 MB at 4 KB/block */

/* On-tape block numbers */
#define LTFS_VOL_LABEL_BLOCK    0U
#define LTFS_INDEX_BLOCK        1U
#define LTFS_DATA_START_BLOCK   2U

/* ── Kennedy 9800 timing (identical to reference; tape-speed-dependent) ─── */
#define K9800_CHAR_PERIOD_US        100U    /* 10 kHz write rate */
#define K9800_RAMP_TIME_MS           30U
#define K9800_WRITE_START_DELAY_MS   36U
#define K9800_WRITE_STOP_DELAY_MS     2U
#define K9800_READ_STOP_DELAY_MS     18U
#define K9800_WARS_DELAY_US         800U
#define K9800_WARS_PULSE_US           3U
#define K9800_OFFC_PULSE_US           5U
#define K9800_RWC_PULSE_US            5U
#define K9800_READY_TIMEOUT_MS     5000U
#define K9800_REWIND_TIMEOUT_MS   60000U
#define K9800_MAX_BLOCK_BYTES      65535U
#define K9800_READ_RING_SIZE        4096U

/* ── FreeRTOS task priorities ────────────────────────────────────────────── */
#define PRI_USB             (configMAX_PRIORITIES - 1)
#define PRI_TAPE            (configMAX_PRIORITIES - 2)
#define PRI_SYNC            (configMAX_PRIORITIES - 3)

/* ── MCP23017 I2C expander (status inputs) ───────────────────────────────── */
#define MCP23017_I2C_ADDR       0x20        /* A2=A1=A0=GND */
#define MCP23017_I2C_PORT       I2C_NUM_0
#define MCP23017_I2C_FREQ_HZ    400000
#define MCP23017_IODIRA         0x00
#define MCP23017_GPPUA          0x0C
#define MCP23017_GPIOA          0x12

/* MCP23017 GPA bit positions for Kennedy status lines (all active LOW) */
#define MCP_BIT_ONL     0
#define MCP_BIT_RWD     1
#define MCP_BIT_FPT     2
#define MCP_BIT_LDP     3
#define MCP_BIT_WEK     4
#define MCP_BIT_RDY     5
#define MCP_BIT_EOT     6
#define MCP_BIT_RNG     7
