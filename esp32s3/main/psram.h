/**
 * @file  psram.h
 * @brief PSRAM region management.
 *
 * On ESP32-S3-N16R8 the 8 MB OPI PSRAM is transparent to the CPU via the
 * cache system.  We allocate three named regions at init time:
 *
 *   fat_meta  — FAT32 BPB, FSInfo, both FAT tables, root cluster
 *   ltfs_idx  — LTFS file index (256 entries)
 *   staging   — single tape-block staging buffer (4 KB)
 *
 * All three regions are allocated with MALLOC_CAP_SPIRAM so they land in
 * PSRAM; no region is ever swapped to flash.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handles returned by psram_region() */
typedef struct psram_region_s {
    void    *ptr;
    size_t   size;
    char     name[16];
} psram_region_t;

/**
 * Allocate all PSRAM regions.  Must be called once before any other psram_*
 * or ltfs_* or fat32_* function.  Returns ESP_OK on success.
 */
esp_err_t psram_init(void);

/**
 * Access region pointers allocated by psram_init().
 * Returned pointer is valid for the lifetime of the firmware.
 */
uint8_t *psram_fat_meta(void);   /* FAT32 metadata region */
uint8_t *psram_ltfs_idx(void);   /* LTFS index region     */
uint8_t *psram_staging(void);    /* tape I/O staging buf  */

size_t psram_fat_meta_size(void);
size_t psram_ltfs_idx_size(void);
size_t psram_staging_size(void);

#ifdef __cplusplus
}
#endif
