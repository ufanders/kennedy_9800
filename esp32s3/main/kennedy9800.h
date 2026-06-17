/**
 * @file  kennedy9800.h
 * @brief Public API for the Kennedy Model 9800 Digital Tape Transport.
 *
 * Thread-safe, blocking API.  All functions must be called from task context;
 * none may be called from ISR context.  Uses CMSIS-RTOS v2 for synchronisation
 * primitives so the implementation is portable across FreeRTOS targets.
 *
 * Active-LOW convention: all Kennedy interface lines are 5 V TTL, active LOW.
 * A bidirectional 3.3 V ↔ 5 V level-shifter is required on every line.
 * See kennedy9800_pins.h for GPIO assignments.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Error codes ─────────────────────────────────────────────────────────── */
typedef enum {
    K9800_OK                =  0,
    K9800_ERR_NOT_INIT      = -1,
    K9800_ERR_NOT_READY     = -2,
    K9800_ERR_WRITE_PROTECT = -3,
    K9800_ERR_EOT           = -4,
    K9800_ERR_BOT           = -5,
    K9800_ERR_TIMEOUT       = -6,
    K9800_ERR_PARITY        = -7,
    K9800_ERR_OVERRUN       = -8,
    K9800_ERR_PARAM         = -9,
    K9800_ERR_BUSY          = -10,
    K9800_ERR_ABORTED       = -11,
    K9800_ERR_DMA           = -12,   /* timer/DMA error on write path */
    K9800_ERR_I2C           = -13,   /* MCP23017 status expander error */
} K9800_Error_t;

/* ── Track / density configuration ──────────────────────────────────────── */
typedef enum { K9800_TRACK_9 = 0, K9800_TRACK_7 = 1 } K9800_TrackConfig_t;
typedef enum { K9800_DENSITY_800 = 0, K9800_DENSITY_1600 = 1 } K9800_DensityConfig_t;

/* ── Transport status snapshot ───────────────────────────────────────────── */
typedef struct {
    bool online;
    bool ready;
    bool tape_running;
    bool rewinding;
    bool at_load_point;
    bool at_eot;
    bool file_protect;
    bool write_enable;
} K9800_TransportStatus_t;

/* ── Initialisation configuration ───────────────────────────────────────── */
typedef struct {
    K9800_TrackConfig_t   tracks;
    K9800_DensityConfig_t density;
    bool                  overwrite;
    uint32_t              tape_speed_ips;
} K9800_Config_t;

#define K9800_DEFAULT_CONFIG  ((K9800_Config_t){ \
    .tracks         = K9800_TRACK_9,        \
    .density        = K9800_DENSITY_800,    \
    .overwrite      = false,                \
    .tape_speed_ips = 12U,                  \
})

/* ── Public API ──────────────────────────────────────────────────────────── */

K9800_Error_t K9800_Init(const K9800_Config_t *cfg);
void          K9800_DeInit(void);

K9800_Error_t K9800_GetStatus(K9800_TransportStatus_t *status);
K9800_Error_t K9800_WaitReady(uint32_t timeout_ms);
K9800_Error_t K9800_Select(bool select);

K9800_Error_t K9800_WriteBlock(const uint8_t *data, uint32_t len);
K9800_Error_t K9800_ReadBlock(uint8_t *buf, uint32_t buf_len, uint32_t *out_len);

K9800_Error_t K9800_Rewind(uint32_t timeout_ms);
K9800_Error_t K9800_SpaceForward(uint32_t duration_ms);
K9800_Error_t K9800_SpaceReverse(uint32_t duration_ms);
K9800_Error_t K9800_GoOffline(void);
K9800_Error_t K9800_Abort(void);

typedef void (*K9800_StatusCallback_t)(const K9800_TransportStatus_t *status);
void K9800_RegisterStatusCallback(K9800_StatusCallback_t cb);

#ifdef __cplusplus
}
#endif
