#include "disk_io.h"
#include "fat32_meta.h"
#include "ltfs.h"
#include "psram.h"
#include "kennedy9800.h"
#include "tape_io.h"
#include "app_config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "disk_io";

static bool  s_ready        = false;
static bool  s_index_dirty  = false;    /* set when a write modifies LTFS index */

/* Staging buffer pointer (single LTFS block, in PSRAM) */
static inline uint8_t *staging(void) { return psram_staging(); }

/* All-zero block used as a source buffer when we have to physically pad the
 * tape with filler blocks (see ensure_block_loaded()/flush_cache_block()). */
static const uint8_t s_zero_block[LTFS_BLOCK_BYTES] = { 0 };

/* ============================================================================
 * Tape data-block cache
 *
 * Host sector I/O is 512 B at a time, but a Kennedy 9800 tape block is
 * 4096 B (one FAT32 cluster). We hold the most recently touched tape block
 * open in the staging buffer so consecutive sector accesses within the same
 * cluster cost one physical tape transfer, not eight.
 *
 * s_commit_hwm tracks how much of the tape has actually been physically
 * written this mount cycle: the transport has no random-access addressing,
 * so a block can only ever be *positioned to* (for read or in-place rewrite)
 * if something has been recorded there before. Blocks at/above the high
 * water mark are "new" — reading one yields zero-filled content instead of
 * touching the tape, and flushing one extends the tape by exactly one block
 * (padding with zero blocks first if a gap ever opens up, e.g. because the
 * host wrote a later cluster of a file before an earlier one).
 * ============================================================================ */
static uint32_t s_cache_block;
static bool     s_cache_valid;
static bool     s_cache_dirty;
static uint32_t s_commit_hwm;

static esp_err_t flush_cache_block(void)
{
    if (!s_cache_valid || !s_cache_dirty) return ESP_OK;

    uint32_t block = s_cache_block;

    /* Pad any gap up to 'block' with physically-written zero blocks — the
     * transport can only extend the tape one block at a time from wherever
     * it has actually written so far. */
    while (s_commit_hwm < block) {
        K9800_Error_t werr = tape_io_write_block(s_commit_hwm, s_zero_block,
                                                  LTFS_BLOCK_BYTES);
        if (werr != K9800_OK) {
            ESP_LOGE(TAG, "gap-fill write at block %u failed, err=%d",
                     (unsigned)s_commit_hwm, werr);
            return ESP_FAIL;
        }
        s_commit_hwm++;
    }

    K9800_Error_t werr = tape_io_write_block(block, staging(), LTFS_BLOCK_BYTES);
    if (werr != K9800_OK) {
        ESP_LOGE(TAG, "tape write block %u failed, err=%d", (unsigned)block, werr);
        return ESP_FAIL;
    }
    if (block >= s_commit_hwm) s_commit_hwm = block + 1;
    s_cache_dirty = false;
    return ESP_OK;
}

/* Make sure 'block' is the one currently resident in the staging buffer,
 * flushing whatever was cached before if it was dirty and different. */
static esp_err_t ensure_block_loaded(uint32_t block)
{
    if (s_cache_valid && s_cache_block == block) return ESP_OK;

    esp_err_t err = flush_cache_block();
    if (err != ESP_OK) return err;

    if (block >= s_commit_hwm) {
        /* Nothing physically recorded here yet — present as zeroed. */
        memset(staging(), 0, LTFS_BLOCK_BYTES);
    } else {
        uint32_t rlen;
        K9800_Error_t rerr = tape_io_read_block(block, staging(), LTFS_BLOCK_BYTES, &rlen);
        if (rerr != K9800_OK && rerr != K9800_ERR_PARITY) {
            ESP_LOGE(TAG, "tape read block %u failed, err=%d", (unsigned)block, rerr);
            s_cache_valid = false;
            return ESP_FAIL;
        }
    }

    s_cache_block = block;
    s_cache_valid = true;
    s_cache_dirty = false;
    return ESP_OK;
}

/* ============================================================================
 * Data region: translate LBA → file + byte offset → tape block
 *
 * LBA is in the FAT32 data region (LBA >= FAT32_DATA_LBA).
 * cluster = (lba - FAT32_DATA_LBA) / FAT32_CLUSTER_SECS + FAT32_ROOT_CLUSTER
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
    uint32_t abs_byte_off   = byte_off_in_file + sec_in_cluster * DISK_SECTOR_SIZE;

    const ltfs_file_entry_t *f = ltfs_find(fname);
    if (!f) return false;

    uint32_t tape_block, blk_byte_off;
    if (!ltfs_locate(f, abs_byte_off, &tape_block, &blk_byte_off)) return false;

    *out_file           = f;
    *out_tape_block     = tape_block;
    *out_block_byte_off = blk_byte_off;
    return true;
}

/* ============================================================================
 * Tape data read/write (through the block cache)
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

    esp_err_t err = ensure_block_loaded(tape_block);
    if (err != ESP_OK) {
        memset(dst, 0, DISK_SECTOR_SIZE);
        return err;
    }
    memcpy(dst, staging() + blk_off, DISK_SECTOR_SIZE);
    return ESP_OK;
}

/* Write a 512-byte sector into the cached tape block. The block is only
 * physically written when the cache moves to a different block or is
 * explicitly flushed (see disk_io_flush()). */
static esp_err_t tape_data_write(uint32_t lba, const uint8_t *src)
{
    const ltfs_file_entry_t *file;
    uint32_t tape_block, blk_off;

    if (!lba_to_tape(lba, &file, &tape_block, &blk_off)) {
        /* Writing to an unallocated cluster: the host writes the directory
         * entry (FAT metadata) first, which triggers ltfs_alloc via
         * sync_dirent_to_ltfs() below, before it ever writes data here. */
        ESP_LOGW(TAG, "write to unmapped LBA %u (no LTFS entry yet)", (unsigned)lba);
        return ESP_OK;
    }

    esp_err_t err = ensure_block_loaded(tape_block);
    if (err != ESP_OK) return err;

    memcpy(staging() + blk_off, src, DISK_SECTOR_SIZE);
    s_cache_dirty = true;
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
        strncpy(lf->name, name, LTFS_FILENAME_MAX - 1);
    }

    /* Grow the file's tape allocation if the host has extended it beyond
     * what its current extents cover. Existing extents are never relocated
     * — a new extent is appended at the tape's current write position. */
    uint32_t needed_blocks = (size + LTFS_BLOCK_BYTES - 1) / LTFS_BLOCK_BYTES;
    uint32_t have_blocks   = ltfs_entry_blocks(lf);

    if (needed_blocks > have_blocks) {
        uint32_t grow = needed_blocks - have_blocks;
        if (lf->extent_count >= LTFS_MAX_EXTENTS_PER_FILE) {
            ESP_LOGE(TAG, "file \"%s\": out of extents, cannot grow to %u blocks",
                     name, (unsigned)needed_blocks);
        } else {
            ltfs_extent_t *ext = &lf->extents[lf->extent_count++];
            ext->first_block = ltfs_alloc_blocks(grow);
            ext->block_count = grow;
        }
    }

    lf->size_bytes = size;
    lf->valid      = 1;
    s_index_dirty  = true;
    ESP_LOGI(TAG, "sync: file \"%s\" size=%u extents=%u",
             name, (unsigned)size, (unsigned)lf->extent_count);
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t disk_io_init(void)
{
    s_ready        = false;
    s_index_dirty  = false;
    s_cache_valid  = false;
    s_cache_dirty  = false;
    s_commit_hwm   = LTFS_DATA_START_BLOCK;
    return ESP_OK;
}

bool     disk_io_ready(void)        { return s_ready; }
uint32_t disk_io_sector_count(void) { return DISK_SECTOR_COUNT; }
uint32_t disk_io_sector_size(void)  { return DISK_SECTOR_SIZE; }

void disk_io_set_ready(bool ready)
{
    s_ready = ready;
    if (ready) {
        /* Everything already allocated as of the last flush is assumed
         * physically present on tape; new allocations extend it further. */
        const ltfs_index_t *idx = ltfs_index();
        s_commit_hwm = idx ? idx->next_free_block : LTFS_DATA_START_BLOCK;
    } else {
        /* Tape is gone — nothing left to flush the cache to. */
        s_cache_dirty = false;
    }
    s_cache_valid = false;
}

esp_err_t disk_io_flush(void)
{
    esp_err_t err = flush_cache_block();
    if (err != ESP_OK) return err;

    if (s_index_dirty) {
        err = ltfs_flush_index();
        if (err != ESP_OK) return err;
        s_index_dirty = false;
    }
    return ESP_OK;
}

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
            /* Data region → tape (cached; not flushed here) */
            esp_err_t err = tape_data_write(cur_lba, src);
            if (err != ESP_OK) return err;
        }
    }

    /* Flush the LTFS index (cheap-ish, a few tape blocks) whenever a
     * directory entry changed. The much larger tape data-block cache is
     * flushed separately — see disk_io_flush(). */
    if (s_index_dirty) {
        esp_err_t err = ltfs_flush_index();
        if (err != ESP_OK) return err;
        s_index_dirty = false;
    }
    return ESP_OK;
}
