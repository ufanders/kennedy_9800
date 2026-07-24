#include "ltfs.h"
#include "psram.h"
#include "app_config.h"
#include "kennedy9800.h"
#include "tape_io.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ltfs";

/* Pointer into PSRAM for the live index */
static ltfs_index_t *s_idx;

/* Scratch buffer sized for one tape block (in PSRAM staging) — used only for
 * the small volume-label transfer; the index itself is transferred straight
 * out of/into its PSRAM home, one LTFS_BLOCK_BYTES chunk per tape block. */
static inline uint8_t *staging(void) { return psram_staging(); }

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t ltfs_init(void)
{
    uint8_t *psram = psram_ltfs_idx();
    if (!psram) return ESP_ERR_INVALID_STATE;
    if (LTFS_INDEX_BLOCKS * LTFS_BLOCK_BYTES > psram_ltfs_idx_size()) {
        ESP_LOGE(TAG, "PSRAM ltfs region too small: need %u, have %u",
                 (unsigned)(LTFS_INDEX_BLOCKS * LTFS_BLOCK_BYTES),
                 (unsigned)psram_ltfs_idx_size());
        return ESP_ERR_NO_MEM;
    }
    s_idx = (ltfs_index_t *)psram;
    memset(s_idx, 0, sizeof(ltfs_index_t));
    return ESP_OK;
}

esp_err_t ltfs_mount(void)
{
    if (!s_idx) return ESP_ERR_INVALID_STATE;
    tape_io_invalidate();   /* force rewind on first seek */

    /* ── Read volume label (block 0) ────────────────────────────────────── */
    uint32_t rlen;
    K9800_Error_t err = tape_io_read_block(LTFS_VOL_LABEL_BLOCK, staging(),
                                            LTFS_BLOCK_BYTES, &rlen);
    if (err != K9800_OK) {
        ESP_LOGE(TAG, "mount: failed to read vol label, err=%d", err);
        return ESP_FAIL;
    }

    ltfs_vol_label_t *vol = (ltfs_vol_label_t *)staging();
    if (memcmp(vol->magic, LTFS_MAGIC_VOL, 8) != 0) {
        ESP_LOGW(TAG, "mount: no valid volume label (blank or foreign tape)");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "mount: volume \"%s\", block_size=%u", vol->label,
             (unsigned)vol->block_size);

    /* ── Read index (blocks LTFS_INDEX_BLOCK .. +LTFS_INDEX_BLOCKS-1) ─────── */
    for (uint32_t i = 0; i < LTFS_INDEX_BLOCKS; i++) {
        uint8_t *dst = (uint8_t *)s_idx + (size_t)i * LTFS_BLOCK_BYTES;
        err = tape_io_read_block(LTFS_INDEX_BLOCK + i, dst, LTFS_BLOCK_BYTES, &rlen);
        if (err != K9800_OK && err != K9800_ERR_PARITY) {
            ESP_LOGE(TAG, "mount: failed to read index block %u, err=%d",
                     (unsigned)i, err);
            return ESP_FAIL;
        }
    }

    if (memcmp(s_idx->magic, LTFS_MAGIC_IDX, 8) != 0) {
        ESP_LOGE(TAG, "mount: index corrupt");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "mount: %u files, next_free_block=%u",
             (unsigned)s_idx->file_count,
             (unsigned)s_idx->next_free_block);
    return ESP_OK;
}

esp_err_t ltfs_flush_index(void)
{
    if (!s_idx) return ESP_ERR_INVALID_STATE;

    for (uint32_t i = 0; i < LTFS_INDEX_BLOCKS; i++) {
        const uint8_t *src = (const uint8_t *)s_idx + (size_t)i * LTFS_BLOCK_BYTES;
        K9800_Error_t err = tape_io_write_block(LTFS_INDEX_BLOCK + i, src,
                                                 LTFS_BLOCK_BYTES);
        if (err != K9800_OK) {
            ESP_LOGE(TAG, "flush_index: write of block %u failed, err=%d",
                     (unsigned)i, err);
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "flush_index: wrote index (%u files, %u blocks)",
             (unsigned)s_idx->file_count, (unsigned)LTFS_INDEX_BLOCKS);
    return ESP_OK;
}

esp_err_t ltfs_format(const char *label)
{
    if (!s_idx || !label) return ESP_ERR_INVALID_ARG;
    tape_io_invalidate();   /* format always rewinds; let tape_seek establish position */

    /* ── Write volume label ──────────────────────────────────────────────── */
    ltfs_vol_label_t *vol = (ltfs_vol_label_t *)staging();
    memset(vol, 0, sizeof(*vol));
    memcpy(vol->magic, LTFS_MAGIC_VOL, 8);
    vol->version     = LTFS_VERSION;
    vol->block_size  = LTFS_BLOCK_BYTES;
    vol->index_block = LTFS_INDEX_BLOCK;
    strncpy(vol->label, label, sizeof(vol->label) - 1);

    K9800_Error_t err = tape_io_write_block(LTFS_VOL_LABEL_BLOCK, staging(),
                                             sizeof(ltfs_vol_label_t));
    if (err != K9800_OK) return ESP_FAIL;

    /* ── Write empty index ───────────────────────────────────────────────── */
    memset(s_idx, 0, sizeof(ltfs_index_t));
    memcpy(s_idx->magic, LTFS_MAGIC_IDX, 8);
    s_idx->version         = LTFS_VERSION;
    s_idx->file_count      = 0;
    s_idx->next_free_block = LTFS_DATA_START_BLOCK;

    return ltfs_flush_index();
}

ltfs_file_entry_t *ltfs_find(const char *name)
{
    if (!s_idx || !name) return NULL;
    for (uint32_t i = 0; i < LTFS_MAX_FILES; i++) {
        ltfs_file_entry_t *e = &s_idx->files[i];
        if (e->valid && strncmp(e->name, name, LTFS_FILENAME_MAX) == 0) {
            return e;
        }
    }
    return NULL;
}

ltfs_file_entry_t *ltfs_alloc_entry(void)
{
    if (!s_idx) return NULL;
    for (uint32_t i = 0; i < LTFS_MAX_FILES; i++) {
        if (!s_idx->files[i].valid) {
            memset(&s_idx->files[i], 0, sizeof(ltfs_file_entry_t));
            s_idx->file_count++;
            return &s_idx->files[i];
        }
    }
    return NULL;
}

esp_err_t ltfs_remove(const char *name)
{
    ltfs_file_entry_t *e = ltfs_find(name);
    if (!e) return ESP_ERR_NOT_FOUND;
    e->valid = 0;
    s_idx->file_count--;
    return ESP_OK;
}

uint32_t ltfs_alloc_blocks(uint32_t count)
{
    if (!s_idx) return 0;
    uint32_t first = s_idx->next_free_block;
    s_idx->next_free_block += count;
    return first;
}

uint32_t ltfs_free_blocks(void)
{
    if (!s_idx) return 0;
    uint32_t used = s_idx->next_free_block;
    if (used >= LTFS_TAPE_MAX_BLOCKS) return 0;
    return LTFS_TAPE_MAX_BLOCKS - used;
}

uint32_t ltfs_entry_blocks(const ltfs_file_entry_t *f)
{
    if (!f) return 0;
    uint32_t total = 0;
    for (uint32_t i = 0; i < f->extent_count; i++) {
        total += f->extents[i].block_count;
    }
    return total;
}

bool ltfs_locate(const ltfs_file_entry_t *f, uint32_t byte_offset,
                  uint32_t *out_block, uint32_t *out_block_off)
{
    if (!f) return false;
    uint32_t rem = byte_offset;
    for (uint32_t i = 0; i < f->extent_count; i++) {
        uint32_t ext_bytes = f->extents[i].block_count * LTFS_BLOCK_BYTES;
        if (rem < ext_bytes) {
            if (out_block)     *out_block     = f->extents[i].first_block + rem / LTFS_BLOCK_BYTES;
            if (out_block_off) *out_block_off  = rem % LTFS_BLOCK_BYTES;
            return true;
        }
        rem -= ext_bytes;
    }
    return false;
}

const ltfs_index_t *ltfs_index(void) { return s_idx; }
