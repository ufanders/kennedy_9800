#include "tape_io.h"
#include "app_config.h"
#include "esp_log.h"

static const char *TAG = "tape_io";

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
 * Space reverse exactly 'n' blocks.
 *
 * The Kennedy 9800 asserts RGAP (active LOW) whenever the head is over an
 * inter-record gap regardless of tape direction, so gap edges are countable
 * in reverse. We assert SRC for one block-period at a time and rely on
 * K9800_SpaceReverse()'s own gap timing.
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
 * Public API
 * ============================================================================ */

void tape_io_invalidate(void)
{
    pos_invalidate();
}

K9800_Error_t tape_io_read_block(uint32_t block_num, uint8_t *buf,
                                  uint32_t buf_len, uint32_t *out_len)
{
    K9800_Error_t err = tape_seek(block_num);
    if (err != K9800_OK) return err;

    err = K9800_ReadBlock(buf, buf_len, out_len);
    if (err == K9800_OK || err == K9800_ERR_PARITY) {
        pos_advance(1);   /* head is now parked before block_num+1 */
    } else {
        ESP_LOGW(TAG, "read block %u failed, err=%d", (unsigned)block_num, err);
        pos_invalidate();
    }
    return err;
}

K9800_Error_t tape_io_write_block(uint32_t block_num, const uint8_t *buf,
                                   uint32_t len)
{
    K9800_Error_t err = tape_seek(block_num);
    if (err != K9800_OK) return err;

    err = K9800_WriteBlock(buf, len);
    if (err == K9800_OK) {
        pos_advance(1);
    } else {
        ESP_LOGW(TAG, "write block %u failed, err=%d", (unsigned)block_num, err);
        pos_invalidate();
    }
    return err;
}
