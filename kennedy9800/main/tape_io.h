/**
 * @file  tape_io.h
 * @brief Shared tape-block positioning layer used by both the LTFS index
 *        I/O (ltfs.c) and the FAT data I/O (disk_io.c).
 *
 * The Kennedy 9800, like any serial reel-to-reel transport, has no
 * random-access addressing: the only way to know where the head is is to
 * count inter-record gaps while spacing from a known reference point (BOT,
 * via rewind). This module owns that single position tracker so that index
 * reads/writes and file-data reads/writes interleave correctly instead of
 * each keeping (and fighting over) their own idea of where the head is.
 */
#pragma once

#include <stdint.h>
#include "kennedy9800.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Mark the current head position as unknown. The next tape_io_read_block()/
 * tape_io_write_block() call will rewind to BOT before seeking. Call this
 * after mount/format and whenever the transport reports an error that may
 * have desynchronized the position tracker (e.g. tape went offline).
 */
void tape_io_invalidate(void);

/**
 * Seek to logical block 'block_num' (0-based from BOT) and read up to
 * 'buf_len' bytes into 'buf'. Advances the internal position tracker by one
 * block on success (including a parity-error read, since the block was
 * still physically consumed).
 */
K9800_Error_t tape_io_read_block(uint32_t block_num, uint8_t *buf,
                                  uint32_t buf_len, uint32_t *out_len);

/**
 * Seek to logical block 'block_num' and write 'len' bytes from 'buf'.
 * Advances the internal position tracker by one block on success.
 */
K9800_Error_t tape_io_write_block(uint32_t block_num, const uint8_t *buf,
                                   uint32_t len);

#ifdef __cplusplus
}
#endif
