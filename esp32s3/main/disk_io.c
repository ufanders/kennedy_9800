#include "disk_io.h"
#include "fat32_meta.h"
#include "ltfs.h"
#include "psram.h"
#include "kennedy9800.h"
#include "app_config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "disk_io";

static bool  s_ready        = false;
static bool  s_index_dirty  = false;    /* set when a write modifies LTFS index */

/* Staging buffer pointer (single LTFS block, in PSRAM) */
static inline uint8_t *staging(void) { return psram_staging(); }

/* ============================================================================
 * Data region: translate LBA → tape block and byte offset within it
 *
 * LBA is in the FAT32 data region (LBA >= FAT32_DATA_LBA).
 * cluster = (lba - FAT32_DATA_LBA) / FAT32_CLUSTER_SECS + FAT32_ROOT_CLUSTER
 *
 * The host always writes/reads whole clusters aligned on cluster boundaries
 * for file data.  We map each cluster run to consecutive tape blocks.
 * ============================================================================ */

/* Convert a data-region LBA to its LTFS file entry and tape block address.
 * Returns false if the LBA maps to an unallocated cluster (e.g. free space). */
static bool lba_to_tape(uint32_t lba,
                         const ltfs_file_entry_t **out_file,
                         uint32_t *out_tape_block,
                         uint32_t *out_block_byte_off)
{
    if (lba < FAT32_DATA_LBA) return false;

    uint32_t data_off_secs = lba - FAT32_DATA_LBA;
    uint32_t cluster       = data_off_secs / FAT32_CLUSTER_SECS + FAT32_ROOT_CLUSTER;

    const char *fname;
    uint32_t    byte_off_in_file;
    if (!fat32_cluster_to_file(cluster, &fname, &byte_off_in_file)) {
        return false;  /* unallocated / free space cluster */
    }

    /* Sector offset within the cluster */
    uint32_t sec_in_cluster = data_off_secs % FAT32_CLUSTER_SECS;
    uint32_t byte_in_cluster = sec_in_cluster * DISK_SECTOR_SIZE;
    uint32_t abs_byte_off   = byte_off_in_file + byte_in_cluster;

    const ltfs_file_entry_t *f = ltfs_find(fname);
    if (!f) return false;

    uint32_t tape_block    = f->first_block + abs_byte_off / LTFS_BLOCK_BYTES;
    uint32_t blk_byte_off  = abs_byte_off % LTFS_BLOCK_BYTES;

    if (tape_block >= f->first_block + f->block_count) return false;

    *out_file         = f;
    *out_tape_block   = tape_block;
    *out_block_byte_off = blk_byte_off;
    return true;
}

/* ============================================================================
 * Tape data read/write (uses PSRAM staging for sub-block access)
 * ============================================================================ */

static esp_err_t tape_data_read(uint32_t lba, uint8_t *dst)
{
    const ltfs_file_entry_t *file;
    uint32_t tape_block, blk_off;

    if (!lba_to_tape(lba, &file, &tape_block, &blk_off)) {
        /* Unallocated cluster: return zeros */
        memset(dst, 0, DISK_SECTOR_SIZE);
        return ESP_OK;
    }

    /* If the sector falls at the start of a full tape block, read directly
     * into the staging buffer and copy the needed sector out.             */
    uint32_t rlen;
    K9800_Error_t err = K9800_ReadBlock(staging(), LTFS_BLOCK_BYTES, &rlen);
    if (err != K9800_OK) {
        ESP_LOGW(TAG, "tape read block %u err=%d", (unsigned)tape_block, err);
        memset(dst, 0, DISK_SECTOR_SIZE);
        return ESP_FAIL;
    }
    memcpy(dst, staging() + blk_off, DISK_SECTOR_SIZE);
    return ESP_OK;
}

/* Write a 512-byte sector into a tape block.
 * Because the Kennedy 9800 writes whole blocks, we must read-modify-write
 * when the sector doesn't align to a full tape block.                     */
static esp_err_t tape_data_write(uint32_t lba, const uint8_t *src)
{
    const ltfs_file_entry_t *file;
    uint32_t tape_block, blk_off;

    if (!lba_to_tape(lba, &file, &tape_block, &blk_off)) {
        /* Writing to an unallocated cluster: allocate LTFS blocks on demand.
         * This happens on the first write to a freshly allocated FAT cluster. */
        /* For simplicity, log and skip — in practice the host writes the
         * directory entry (FAT metadata) first which triggers ltfs_alloc.  */
        ESP_LOGW(TAG, "write to unmapped LBA %u (no LTFS entry yet)", (unsigned)lba);
        return ESP_OK;
    }

    /* Read-modify-write cycle through staging */
    uint32_t rlen;
    if (blk_off != 0 || DISK_SECTOR_SIZE != LTFS_BLOCK_BYTES) {
        K9800_Error_t rerr = K9800_ReadBlock(staging(), LTFS_BLOCK_BYTES, &rlen);
        if (rerr != K9800_OK) {
            /* First write to this block: initialise staging to zeros */
            memset(staging(), 0, LTFS_BLOCK_BYTES);
        }
    } else {
        memset(staging(), 0, LTFS_BLOCK_BYTES);
    }

    memcpy(staging() + blk_off, src, DISK_SECTOR_SIZE);

    K9800_Error_t werr = K9800_WriteBlock(staging(), LTFS_BLOCK_BYTES);
    if (werr != K9800_OK) {
        ESP_LOGE(TAG, "tape write block %u err=%d", (unsigned)tape_block, werr);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ============================================================================
 * Metadata write: detect directory entry writes to sync LTFS index
 * ============================================================================ */

/* Check whether 'lba' is in the root directory cluster area */
static bool is_root_dir_lba(uint32_t lba)
{
    return (lba >= FAT32_DATA_LBA &&
            lba < FAT32_DATA_LBA + FAT32_CLUSTER_SECS);
}

/* Parse a 32-byte directory entry and create/update LTFS entry if valid */
static void sync_dirent_to_ltfs(const uint8_t *e)
{
    if (e[0] == 0x00 || e[0] == 0xE5) return;
    if (e[11] & 0x08) return;  /* volume label */
    if (e[11] & 0x10) return;  /* directory */
    if (e[11] == 0x0F) return; /* LFN entry */

    uint32_t size = (uint32_t)e[28] | ((uint32_t)e[29] << 8) |
                    ((uint32_t)e[30] << 16) | ((uint32_t)e[31] << 24);
    uint32_t fc   = (uint32_t)e[26] | ((uint32_t)e[27] << 8) |
                    ((uint32_t)e[20] << 16) | ((uint32_t)e[21] << 24);

    /* Build filename from 8.3 */
    char name[13];
    int n = 0;
    for (int i = 0; i < 8 && e[i] != 0x20; i++) name[n++] = (char)e[i];
    if (e[8] != 0x20) {
        name[n++] = '.';
        for (int i = 8; i < 11 && e[i] != 0x20; i++) name[n++] = (char)e[i];
    }
    name[n] = '\0';

    if (size == 0 || fc < 2) return;  /* empty or unallocated */

    ltfs_file_entry_t *lf = ltfs_find(name);
    if (!lf) {
        lf = ltfs_alloc_entry();
        if (!lf) { ESP_LOGW(TAG, "LTFS index full"); return; }
    }
    strncpy(lf->name, name, LTFS_FILENAME_MAX - 1);
    lf->size_bytes  = size;
    lf->block_count = (size + LTFS_BLOCK_BYTES - 1) / LTFS_BLOCK_BYTES;
    if (lf->first_block == 0) {
        lf->first_block = ltfs_alloc_blocks(lf->block_count);
    }
    lf->valid = 1;
    s_index_dirty = true;
    ESP_LOGI(TAG, "sync: file \"%s\" size=%u tape_block=%u",
             name, (unsigned)size, (unsigned)lf->first_block);
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t disk_io_init(void)
{
    s_ready       = false;
    s_index_dirty = false;
    return ESP_OK;
}

bool     disk_io_ready(void)        { return s_ready; }
uint32_t disk_io_sector_count(void) { return DISK_SECTOR_COUNT; }
uint32_t disk_io_sector_size(void)  { return DISK_SECTOR_SIZE; }

void disk_io_set_ready(bool ready)  { s_ready = ready; }

esp_err_t disk_io_read(uint32_t lba, uint8_t *buf, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++) {
        uint32_t cur_lba = lba + i;
        uint8_t *dst = buf + i * DISK_SECTOR_SIZE;

        if (cur_lba < FAT32_DATA_LBA) {
            /* Metadata region → PSRAM */
            esp_err_t err = fat32_meta_read_sector(cur_lba, dst);
            if (err != ESP_OK) return err;
        } else if (is_root_dir_lba(cur_lba)) {
            /* Root directory cluster → PSRAM */
            esp_err_t err = fat32_meta_read_sector(cur_lba, dst);
            if (err != ESP_OK) return err;
        } else {
            /* Data region → tape */
            esp_err_t err = tape_data_read(cur_lba, dst);
            if (err != ESP_OK) return err;
        }
    }
    return ESP_OK;
}

esp_err_t disk_io_write(uint32_t lba, const uint8_t *buf, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++) {
        uint32_t cur_lba = lba + i;
        const uint8_t *src = buf + i * DISK_SECTOR_SIZE;

        if (cur_lba < FAT32_DATA_LBA) {
            /* Metadata region → PSRAM */
            esp_err_t err = fat32_meta_write_sector(cur_lba, src);
            if (err != ESP_OK) return err;
        } else if (is_root_dir_lba(cur_lba)) {
            /* Root directory write → PSRAM + sync to LTFS */
            esp_err_t err = fat32_meta_write_sector(cur_lba, src);
            if (err != ESP_OK) return err;
            /* Scan the sector for valid directory entries */
            for (uint32_t d = 0; d < DISK_SECTOR_SIZE / 32U; d++) {
                sync_dirent_to_ltfs(src + d * 32U);
            }
        } else {
            /* Data region → tape */
            esp_err_t err = tape_data_write(cur_lba, src);
            if (err != ESP_OK) return err;
        }
    }

    /* Flush LTFS index to tape if modified */
    if (s_index_dirty) {
        ltfs_flush_index();
        s_index_dirty = false;
    }
    return ESP_OK;
}
