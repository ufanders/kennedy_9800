/**
 * @file  ltfs.h
 * @brief Simplified LTFS-inspired on-tape index, cached in PSRAM.
 *
 * On-tape layout (9-track, 800 cpi, 4 096-byte blocks):
 *
 *   Block 0                              — Volume label (ltfs_vol_label_t)
 *   Blocks 1 .. (1+LTFS_INDEX_BLOCKS-1)   — File index  (ltfs_index_t),
 *                                            spanning multiple 4 KB blocks
 *   Blocks LTFS_DATA_START_BLOCK+         — File data
 *
 * The index is loaded from tape into PSRAM at mount time and flushed back
 * when the volume is unmounted or modified. File content is never cached.
 *
 * Each file may occupy multiple extents (non-contiguous tape-block ranges).
 * This lets a file grow (e.g. the host appending to it) by allocating a new
 * extent at the current tape write position instead of requiring the whole
 * file to occupy one contiguous run decided at creation time — the tape
 * transport has no random-access addressing, so already-written extents are
 * never relocated, only appended to.
 *
 * Tape addressing uses sequential block numbers (0-based); positioning is
 * done by the shared tape_io module (spacing forward/backward from BOT).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "kennedy9800.h"
#include "app_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LTFS_MAGIC_VOL   "K9800VOL"
#define LTFS_MAGIC_IDX   "K9800IDX"
#define LTFS_VERSION     2U

/* ── On-tape structures (packed for tape I/O) ────────────────────────────── */

typedef struct __attribute__((packed)) {
    char     magic[8];          /* LTFS_MAGIC_VOL */
    uint32_t version;
    char     label[64];
    uint32_t block_size;        /* always LTFS_BLOCK_BYTES */
    uint32_t index_block;       /* always LTFS_INDEX_BLOCK */
    uint8_t  reserved[428];     /* pad to 512 bytes */
} ltfs_vol_label_t;

/* One contiguous run of tape blocks belonging to a file. */
typedef struct __attribute__((packed)) {
    uint32_t first_block;
    uint32_t block_count;
} ltfs_extent_t;

typedef struct __attribute__((packed)) {
    char          name[LTFS_FILENAME_MAX];  /* UTF-8 filename, null-terminated */
    uint32_t      size_bytes;
    uint32_t      mtime;                    /* Unix timestamp */
    uint8_t       valid;                    /* 1 = slot in use */
    uint8_t       extent_count;             /* valid entries in extents[] */
    uint8_t       pad[2];
    ltfs_extent_t extents[LTFS_MAX_EXTENTS_PER_FILE];
} ltfs_file_entry_t;

#define LTFS_INDEX_HEADER_SIZE  32U
typedef struct __attribute__((packed)) {
    char               magic[8];        /* LTFS_MAGIC_IDX */
    uint32_t           version;
    uint32_t           file_count;      /* number of valid entries */
    uint32_t           next_free_block; /* next available data block on tape */
    uint8_t            reserved[12];
    ltfs_file_entry_t  files[LTFS_MAX_FILES];
} ltfs_index_t;

/* The index (currently tens of KB) no longer fits in a single 4 KB tape
 * block, so it spans LTFS_INDEX_BLOCKS consecutive blocks starting at
 * LTFS_INDEX_BLOCK. File data starts right after it. */
#define LTFS_INDEX_BLOCKS \
    ((uint32_t)((sizeof(ltfs_index_t) + LTFS_BLOCK_BYTES - 1U) / LTFS_BLOCK_BYTES))
#define LTFS_DATA_START_BLOCK  (LTFS_INDEX_BLOCK + LTFS_INDEX_BLOCKS)

/* ── In-memory (PSRAM) index handle ─────────────────────────────────────── */

/**
 * Initialise the LTFS layer. Points the index at PSRAM (caller must have
 * already called psram_init()). Does NOT read from tape; call ltfs_mount()
 * for that.
 */
esp_err_t ltfs_init(void);

/**
 * Read the volume label and index from tape, populate the PSRAM index.
 * Rewinds to BOT first. Returns ESP_ERR_NOT_FOUND if no valid label exists
 * (blank tape).
 */
esp_err_t ltfs_mount(void);

/**
 * Write the in-PSRAM index back to tape (the volume label plus
 * LTFS_INDEX_BLOCKS index blocks). Data blocks are NOT disturbed.
 */
esp_err_t ltfs_flush_index(void);

/**
 * Format the tape: rewind, write a fresh volume label and empty index,
 * then set next_free_block = LTFS_DATA_START_BLOCK.
 */
esp_err_t ltfs_format(const char *label);

/* ── File-level operations ───────────────────────────────────────────────── */

/**
 * Look up a file by name. Returns pointer into PSRAM index (valid until
 * ltfs_flush_index() or ltfs_format()); NULL if not found.
 */
ltfs_file_entry_t *ltfs_find(const char *name);

/**
 * Allocate a new (zeroed) file entry. Returns NULL if the index is full.
 * Caller must fill name, size_bytes, mtime, add at least one extent via
 * ltfs_alloc_blocks(), and set valid=1.
 */
ltfs_file_entry_t *ltfs_alloc_entry(void);

/**
 * Remove a file entry by name (marks valid=0). Does NOT reclaim tape space.
 */
esp_err_t ltfs_remove(const char *name);

/**
 * Reserve tape blocks for a new extent. Returns the first block number
 * assigned and advances next_free_block. Blocks are always handed out in
 * increasing order and never reused until ltfs_format().
 */
uint32_t ltfs_alloc_blocks(uint32_t count);

/**
 * Return the number of free tape blocks remaining.
 */
uint32_t ltfs_free_blocks(void);

/**
 * Return the total number of tape blocks currently allocated to a file
 * across all of its extents.
 */
uint32_t ltfs_entry_blocks(const ltfs_file_entry_t *f);

/**
 * Translate a byte offset within a file's logical content to a tape block
 * number and byte offset within that block, walking the file's extent list.
 * Returns false if byte_offset lies beyond the file's allocated extents.
 */
bool ltfs_locate(const ltfs_file_entry_t *f, uint32_t byte_offset,
                  uint32_t *out_block, uint32_t *out_block_off);

/**
 * Read raw PSRAM index pointer (for disk_io tape-position lookups).
 */
const ltfs_index_t *ltfs_index(void);

#ifdef __cplusplus
}
#endif
