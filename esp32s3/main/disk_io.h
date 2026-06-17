/**
 * @file  disk_io.h
 * @brief LBA routing: metadata LBAs → PSRAM, data LBAs → tape.
 *
 * The USB MSC callbacks call disk_io_read() / disk_io_write() for every
 * host sector access.  This layer decides whether the LBA falls in the
 * metadata region (served from PSRAM) or the data region (routed to tape).
 *
 * Data LBA → tape mapping:
 *
 *   cluster = (lba - FAT32_DATA_LBA) / FAT32_CLUSTER_SECS + FAT32_ROOT_CLUSTER
 *   byte_offset_in_file = (via FAT chain walk in fat32_meta)
 *   tape_block = ltfs_entry.first_block + byte_offset / LTFS_BLOCK_BYTES
 *
 * Only complete LTFS_BLOCK_BYTES (4 KB) tape blocks are read or written.
 * Partial writes are assembled in the PSRAM staging buffer.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise disk I/O layer (call after psram_init, ltfs_init, fat32_meta_init). */
esp_err_t disk_io_init(void);

/**
 * Read 'count' sectors starting at 'lba' into 'buf'.
 * Returns ESP_OK or ESP_FAIL.
 */
esp_err_t disk_io_read(uint32_t lba, uint8_t *buf, uint32_t count);

/**
 * Write 'count' sectors starting at 'lba' from 'buf'.
 * Returns ESP_OK or ESP_FAIL.
 */
esp_err_t disk_io_write(uint32_t lba, const uint8_t *buf, uint32_t count);

/** True if the volume is currently ready (tape loaded and LTFS mounted). */
bool disk_io_ready(void);

/** Total sector count of the virtual disk. */
uint32_t disk_io_sector_count(void);

/** Sector size in bytes (always DISK_SECTOR_SIZE). */
uint32_t disk_io_sector_size(void);

#ifdef __cplusplus
}
#endif
