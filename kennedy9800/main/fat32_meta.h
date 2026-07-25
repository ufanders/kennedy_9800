/**
 * @file  fat32_meta.h
 * @brief FAT32 metadata construction and sector access (backed by PSRAM).
 *
 * A 128 MB FAT32 volume is built entirely in PSRAM:
 *   Sectors 0       — Boot sector (BPB)
 *   Sector  1       — FSInfo
 *   Sectors 2-31    — Reserved (unused)
 *   Sectors 32 .. (32+FAT_SECS-1)               — FAT copy 1
 *   Sectors (32+FAT_SECS) .. (32+2*FAT_SECS-1)  — FAT copy 2
 *   Sector  FAT32_DATA_LBA                        — Root dir (cluster 2)
 *   Sectors FAT32_DATA_LBA+8 ..                   — Data region (→ tape)
 *
 * Sector reads/writes for LBAs below FAT32_DATA_LBA always hit PSRAM.
 * disk_io.c handles routing for LBAs at and beyond FAT32_DATA_LBA.
 *
 * After calling fat32_meta_init(), call fat32_meta_build() once to stamp
 * a freshly formatted FAT32 structure into PSRAM.  On subsequent boots,
 * call fat32_meta_rebuild_from_ltfs() to regenerate directory entries from
 * the live LTFS index without touching the tape.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FAT32_EOC   0x0FFFFFF8U    /* end-of-chain marker */
#define FAT32_FREE  0x00000000U

/**
 * Initialise the FAT32 metadata layer.  Must be called after psram_init().
 * Clears the PSRAM fat region.
 */
esp_err_t fat32_meta_init(void);

/**
 * Write a freshly formatted FAT32 volume structure into PSRAM.
 * Stamps BPB, FSInfo, both FAT tables (with volume ID cluster chain),
 * and an empty root directory cluster.
 * Call once on first use or after ltfs_format().
 */
esp_err_t fat32_meta_build(const char *volume_label);

/**
 * Rebuild the root directory and FAT entries from the current LTFS index
 * without reformatting.  Call at mount time after ltfs_mount() succeeds.
 */
esp_err_t fat32_meta_rebuild_from_ltfs(void);

/**
 * Read DISK_SECTOR_SIZE bytes from a metadata sector into dst.
 * lba must be < FAT32_DATA_LBA.
 */
esp_err_t fat32_meta_read_sector(uint32_t lba, uint8_t *dst);

/**
 * Write DISK_SECTOR_SIZE bytes from src into a metadata sector.
 * lba must be < FAT32_DATA_LBA.
 * Parses the written data to keep the in-PSRAM FAT consistent.
 */
esp_err_t fat32_meta_write_sector(uint32_t lba, const uint8_t *src);

/**
 * Look up the file that owns FAT32 cluster 'cluster'.
 * Walks the FAT chain from the root directory to find the matching file,
 * then returns the LTFS file entry pointer and byte offset within the file.
 * Returns false if the cluster is unallocated or belongs to no file.
 */
bool fat32_cluster_to_file(uint32_t cluster,
                            const char **out_filename,
                            uint32_t    *out_byte_offset);

/**
 * Allocate a cluster chain for a new file being written by the host.
 * first_cluster is the cluster already allocated in the FAT by the host.
 * Returns the total byte length of the chain.
 */
uint32_t fat32_chain_length(uint32_t first_cluster);

/**
 * Low-level: read the FAT entry for a given cluster number.
 */
uint32_t fat32_read_fat_entry(uint32_t cluster);

#ifdef __cplusplus
}
#endif
