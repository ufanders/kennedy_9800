/**
 * @file  ltfs.h
 * @brief Simplified LTFS-inspired on-tape index, cached in PSRAM.
 *
 * On-tape layout (9-track, 800 cpi, 4 096-byte blocks):
 *
 *   Block 0  — Volume label  (struct ltfs_vol_label_t)
 *   Block 1  — File index    (struct ltfs_index_t)
 *   Blocks 2+ — File data, one or more consecutive blocks per file
 *
 * The index is loaded from tape into PSRAM at mount time and flushed back
 * when the volume is unmounted or modified.  File content is never cached.
 *
 * Tape addressing uses sequential block numbers (0-based); positioning is
 * done by spacing forward/backward the required number of blocks from BOT.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "kennedy9800.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LTFS_MAGIC_VOL   "K9800VOL"
#define LTFS_MAGIC_IDX   "K9800IDX"
#define LTFS_VERSION     1U

/* ── On-tape structures (packed for tape I/O) ────────────────────────────── */

typedef struct __attribute__((packed)) {
    char     magic[8];          /* LTFS_MAGIC_VOL */
    uint32_t version;
    char     label[64];
    uint32_t block_size;        /* always LTFS_BLOCK_BYTES */
    uint32_t index_block;       /* always 1 */
    uint8_t  reserved[428];     /* pad to 512 bytes */
} ltfs_vol_label_t;

typedef struct __attribute__((packed)) {
    char     name[240];         /* UTF-8 filename, null-terminated */
    uint32_t size_bytes;
    uint32_t first_block;       /* tape block number of first data block */
    uint32_t block_count;
    uint32_t mtime;             /* Unix timestamp */
    uint8_t  valid;             /* 1 = slot in use */
    uint8_t  pad[7];
} ltfs_file_entry_t;            /* 260 bytes each */

#define LTFS_INDEX_HEADER_SIZE  32U
typedef struct __attribute__((packed)) {
    char               magic[8];        /* LTFS_MAGIC_IDX */
    uint32_t           version;
    uint32_t           file_count;      /* number of valid entries */
    uint32_t           next_free_block; /* next available data block on tape */
    uint8_t            reserved[12];
    ltfs_file_entry_t  files[256];      /* fixed-size table, ≤256 files */
} ltfs_index_t;

/* ── In-memory (PSRAM) index handle ─────────────────────────────────────── */

/**
 * Initialise the LTFS layer.  Loads the index from PSRAM (caller must have
 * already called psram_init()).  Does NOT read from tape; call ltfs_mount()
 * for that.
 */
esp_err_t ltfs_init(void);

/**
 * Read the volume label and index from tape, populate the PSRAM index.
 * Rewinds to BOT first.  Returns ESP_ERR_NOT_FOUND if no valid label exists
 * (blank tape).
 */
esp_err_t ltfs_mount(void);

/**
 * Write the in-PSRAM index back to tape (blocks 0 and 1).
 * Rewinds to BOT, writes volume label, then writes index block.
 * Data blocks are NOT disturbed.
 */
esp_err_t ltfs_flush_index(void);

/**
 * Format the tape: rewind, write a fresh volume label and empty index,
 * then set next_free_block = LTFS_DATA_START_BLOCK.
 */
esp_err_t ltfs_format(const char *label);

/* ── File-level operations ───────────────────────────────────────────────── */

/**
 * Look up a file by name.  Returns pointer into PSRAM index (valid until
 * ltfs_flush_index() or ltfs_format()); NULL if not found.
 */
ltfs_file_entry_t *ltfs_find(const char *name);

/**
 * Allocate a new file entry.  Returns pointer to the new (zeroed) entry in
 * PSRAM, or NULL if the index is full.  Caller must fill name, size_bytes,
 * first_block, block_count, mtime and set valid=1.
 */
ltfs_file_entry_t *ltfs_alloc_entry(void);

/**
 * Remove a file entry by name (marks valid=0).  Does NOT reclaim tape space.
 */
esp_err_t ltfs_remove(const char *name);

/**
 * Reserve tape blocks for a new file.  Returns the first block number
 * assigned to the file and advances next_free_block.
 */
uint32_t ltfs_alloc_blocks(uint32_t count);

/**
 * Return the number of free tape blocks remaining.
 */
uint32_t ltfs_free_blocks(void);

/**
 * Read raw PSRAM index pointer (for disk_io tape-position lookups).
 */
const ltfs_index_t *ltfs_index(void);

#ifdef __cplusplus
}
#endif
