#include "ltfs.h"
#include "psram.h"
#include "app_config.h"
#include "kennedy9800.h"
#include "esp_log.h"
#include <string.h>
#include <time.h>

static const char *TAG = "ltfs";

/* Pointer into PSRAM for the live index */
static ltfs_index_t *s_idx;

/* Scratch buffer sized for one tape block (in PSRAM staging) */
static inline uint8_t *staging(void) { return psram_staging(); }

/* ============================================================================
 * Tape position tracker
 *
 * current_block is the block number the head is parked at — i.e. the next
 * K9800_ReadBlock/WriteBlock call will transfer block current_block.
 *
 * After a successful read or write of block N, current_block becomes N+1
 * (the head is now sitting at the IRG before block N+1).
 *
 * position_known is cleared on any transport error and restored after the
 * next successful rewind so we never act on a stale position.
 * ============================================================================ */
static struct {
    uint32_t current_block;
    bool     position_known;
} s_pos;

static void pos_reset(void)
{
    s_pos.current_block  = 0;
    s_pos.position_known = true;
}

static void pos_invalidate(void)
{
    s_pos.position_known = false;
}

static void pos_advance(uint32_t blocks)
{
    if (s_pos.position_known) s_pos.current_block += blocks;
}

/* ============================================================================
 * Tape positioning helpers
 * ============================================================================ */

/* Space forward exactly 'n' blocks by reading and discarding them.
 * RGAP transitions from K9800_ReadBlock() give us exact block counting. */
static K9800_Error_t tape_space_fwd_blocks(uint32_t n)
{
    static uint8_t discard[LTFS_BLOCK_BYTES];
    uint32_t dummy_len;
    for (uint32_t i = 0; i < n; i++) {
        K9800_Error_t err = K9800_ReadBlock(discard, sizeof(discard), &dummy_len);
        if (err != K9800_OK && err != K9800_ERR_PARITY) {
            pos_invalidate();
            return err;
        }
        pos_advance(1);
    }
    return K9800_OK;
}

/*
 * Space reverse exactly 'n' blocks using SRC + RGAP counting.
 *
 * The Kennedy 9800 asserts RGAP (active LOW) whenever the head is over an
 * inter-record gap regardless of tape direction, so gap edges are countable
 * in reverse.  We assert SRC and watch for n rising RGAP edges (gap exits).
 *
 * If your specific drive variant does not generate RGAP in reverse, replace
 * this body with a simple return K9800_ERR_NOT_INIT so that tape_seek() falls
 * back to rewind+forward for all backward moves.
 */
static K9800_Error_t tape_space_rev_blocks(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        /* Each SpaceReverse call moves back one block-worth of tape.
         * Block period at 800 cpi, 4096 B/block, 12.5 ips:
         *   data  = 4096 / 800 inches = 5.12 in → 5120/12.5 = 410 ms
         *   IRG   ≈ 0.6 in                       →  600/12.5 =  48 ms
         *   total ≈ 460 ms; use 600 ms with margin. */
        K9800_Error_t err = K9800_SpaceReverse(600U);
        if (err == K9800_ERR_BOT) {
            /* Hit load point before completing n steps — clamp to 0 */
            s_pos.current_block  = 0;
            s_pos.position_known = true;
            return K9800_OK;
        }
        if (err != K9800_OK) {
            pos_invalidate();
            return err;
        }
        if (s_pos.position_known && s_pos.current_block > 0) {
            s_pos.current_block--;
        }
    }
    return K9800_OK;
}

/*
 * Seek to tape block 'target' using the shortest available path:
 *
 *   1. Already there          → nothing to do.
 *   2. Target is ahead        → space forward Δ blocks.
 *   3. Target is behind
 *      a. Reverse Δ ≤ forward from BOT  → space reverse Δ blocks.
 *      b. Otherwise                      → rewind + space forward 'target' blocks.
 *
 * Crossover point: target < current/2  →  rewind is cheaper.
 */
static K9800_Error_t tape_seek(uint32_t target)
{
    /* If position is unknown we must rewind to re-establish it. */
    if (!s_pos.position_known) {
        K9800_Error_t err = K9800_Rewind(0);
        if (err != K9800_OK) return err;
        pos_reset();
    }

    if (target == s_pos.current_block) return K9800_OK;

    if (target > s_pos.current_block) {
        /* ── Case 2: head is behind target — space forward ─────────────── */
        return tape_space_fwd_blocks(target - s_pos.current_block);
    }

    /* ── Case 3: head is ahead of target ─────────────────────────────── */
    uint32_t rev_dist = s_pos.current_block - target;   /* blocks to reverse */
    uint32_t fwd_dist = target;                          /* blocks from BOT   */

    if (rev_dist <= fwd_dist) {
        /* Case 3a: reverse is shorter */
        return tape_space_rev_blocks(rev_dist);
    }

    /* Case 3b: rewind + forward is shorter */
    K9800_Error_t err = K9800_Rewind(0);
    if (err != K9800_OK) { pos_invalidate(); return err; }
    pos_reset();
    return tape_space_fwd_blocks(target);
}

/* ============================================================================
 * Read/write one logical LTFS tape block (fills/drains PSRAM staging buf)
 * ============================================================================ */
static K9800_Error_t tape_read_block(uint32_t block_num,
                                     uint8_t *buf, uint32_t *out_len)
{
    K9800_Error_t err = tape_seek(block_num);
    if (err != K9800_OK) return err;
    err = K9800_ReadBlock(buf, LTFS_BLOCK_BYTES, out_len);
    if (err == K9800_OK || err == K9800_ERR_PARITY) {
        pos_advance(1);   /* head is now parked before block_num+1 */
    } else {
        pos_invalidate();
    }
    return err;
}

static K9800_Error_t tape_write_block(uint32_t block_num,
                                      const uint8_t *buf, uint32_t len)
{
    K9800_Error_t err = tape_seek(block_num);
    if (err != K9800_OK) return err;
    err = K9800_WriteBlock(buf, len);
    if (err == K9800_OK) {
        pos_advance(1);
    } else {
        pos_invalidate();
    }
    return err;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t ltfs_init(void)
{
    uint8_t *psram = psram_ltfs_idx();
    if (!psram) return ESP_ERR_INVALID_STATE;
    s_idx = (ltfs_index_t *)psram;
    /* Zero-init the PSRAM region (calloc already does this, but be explicit) */
    memset(s_idx, 0, sizeof(ltfs_index_t));
    return ESP_OK;
}

esp_err_t ltfs_mount(void)
{
    if (!s_idx) return ESP_ERR_INVALID_STATE;
    pos_invalidate();   /* force rewind on first seek */

    /* ── Read volume label (block 0) ────────────────────────────────────── */
    uint32_t rlen;
    K9800_Error_t err = tape_read_block(LTFS_VOL_LABEL_BLOCK, staging(), &rlen);
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

    /* ── Read index (block 1) ────────────────────────────────────────────── */
    err = tape_read_block(LTFS_INDEX_BLOCK, staging(), &rlen);
    if (err != K9800_OK) {
        ESP_LOGE(TAG, "mount: failed to read index block, err=%d", err);
        return ESP_FAIL;
    }

    ltfs_index_t *on_tape = (ltfs_index_t *)staging();
    if (memcmp(on_tape->magic, LTFS_MAGIC_IDX, 8) != 0) {
        ESP_LOGE(TAG, "mount: index block corrupt");
        return ESP_FAIL;
    }
    memcpy(s_idx, on_tape, sizeof(ltfs_index_t));

    ESP_LOGI(TAG, "mount: %u files, next_free_block=%u",
             (unsigned)s_idx->file_count,
             (unsigned)s_idx->next_free_block);
    return ESP_OK;
}

esp_err_t ltfs_flush_index(void)
{
    if (!s_idx) return ESP_ERR_INVALID_STATE;

    /* Write index block to PSRAM staging, then to tape block 1 */
    memcpy(staging(), s_idx, sizeof(ltfs_index_t));
    K9800_Error_t err = tape_write_block(LTFS_INDEX_BLOCK,
                                         staging(), sizeof(ltfs_index_t));
    if (err != K9800_OK) {
        ESP_LOGE(TAG, "flush_index: write failed, err=%d", err);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "flush_index: wrote index (%u files)", (unsigned)s_idx->file_count);
    return ESP_OK;
}

esp_err_t ltfs_format(const char *label)
{
    if (!s_idx || !label) return ESP_ERR_INVALID_ARG;
    pos_invalidate();   /* format always rewinds; let tape_seek establish position */

    /* ── Write volume label ──────────────────────────────────────────────── */
    ltfs_vol_label_t *vol = (ltfs_vol_label_t *)staging();
    memset(vol, 0, sizeof(*vol));
    memcpy(vol->magic, LTFS_MAGIC_VOL, 8);
    vol->version     = LTFS_VERSION;
    vol->block_size  = LTFS_BLOCK_BYTES;
    vol->index_block = LTFS_INDEX_BLOCK;
    strncpy(vol->label, label, sizeof(vol->label) - 1);

    K9800_Error_t err = tape_write_block(LTFS_VOL_LABEL_BLOCK,
                                         staging(), sizeof(ltfs_vol_label_t));
    if (err != K9800_OK) return ESP_FAIL;

    /* ── Write empty index ───────────────────────────────────────────────── */
    memset(s_idx, 0, sizeof(ltfs_index_t));
    memcpy(s_idx->magic, LTFS_MAGIC_IDX, 8);
    s_idx->version        = LTFS_VERSION;
    s_idx->file_count     = 0;
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

const ltfs_index_t *ltfs_index(void) { return s_idx; }
