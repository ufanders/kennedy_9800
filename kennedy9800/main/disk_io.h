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
 *   (tape_block, block_off) = ltfs_locate(entry, byte_offset_in_file)
 *
 * A file's tape blocks may be split across multiple extents (see ltfs.h);
 * ltfs_locate() walks them to find the physical block for a given offset.
 *
 * Only complete LTFS_BLOCK_BYTES (4 KB) tape blocks are read or written. The
 * most recently touched block is held open in the PSRAM staging buffer
 * (disk_io's block cache) so that repeated sector-sized host accesses to the
 * same tape block only cost one physical tape transfer; the buffer is
 * flushed when the host moves to a different block or on explicit sync.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
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

/**
 * Flush any dirty tape-data block cached in PSRAM and the LTFS index (if
 * modified) out to tape. Call on SCSI SYNCHRONIZE CACHE, on host eject, and
 * periodically from a background task so writes don't sit unflushed
 * indefinitely.
 */
esp_err_t disk_io_flush(void);

/** True if the volume is currently ready (tape loaded and LTFS mounted). */
bool disk_io_ready(void);

/** Total sector count of the virtual disk. */
uint32_t disk_io_sector_count(void);

/** Sector size in bytes (always DISK_SECTOR_SIZE). */
uint32_t disk_io_sector_size(void);

/** Mark the volume ready/not-ready; resets the tape data-block cache and,
 * when becoming ready, re-derives the physical write high-water mark from
 * the LTFS index. */
void disk_io_set_ready(bool ready);

#ifdef __cplusplus
}
#endif
