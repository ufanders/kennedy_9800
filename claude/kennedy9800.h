/**
 * @file    kennedy9800.h
 * @brief   Isolated API for the Kennedy Model 9800 Digital Tape Transport.
 *
 * @details Provides a blocking, thread-safe API for controlling the Kennedy 9800
 *          from a FreeRTOS application (CMSIS-RTOS v2 mode).  All public functions
 *          may be called from application tasks; none may be called from ISR context.
 *
 *          Typical write sequence:
 *          @code
 *          K9800_Init();
 *
 *          // Wait for operator to load tape and press ON LINE
 *          K9800_WaitReady(5000);
 *
 *          uint8_t buf[] = { 0x01, 0x02, 0x03 };
 *          K9800_WriteBlock(buf, sizeof(buf));
 *
 *          K9800_Rewind();
 *          @endcode
 *
 *          Typical read sequence:
 *          @code
 *          K9800_Init();
 *          K9800_WaitReady(5000);
 *
 *          uint8_t  rxbuf[K9800_MAX_BLOCK_BYTES];
 *          uint32_t rxlen = 0;
 *          K9800_ReadBlock(rxbuf, &rxlen);
 *          @endcode
 *
 * @note    Interface logic convention: zero-true (active-LOW) throughout.
 *          A 5 V ↔ 3.3 V bidirectional level shifter is required on all
 *          Kennedy interface lines.  See kennedy9800_pins.h for GPIO/timer/DMA
 *          assignments.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "kennedy9800_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Error / status codes
 * ============================================================================ */
typedef enum {
    K9800_OK                = 0,    /**< Operation succeeded                   */
    K9800_ERR_NOT_INIT      = -1,   /**< K9800_Init() not yet called           */
    K9800_ERR_NOT_READY     = -2,   /**< Transport not ready (not loaded/online) */
    K9800_ERR_WRITE_PROTECT = -3,   /**< Write-protect ring absent             */
    K9800_ERR_EOT           = -4,   /**< End-of-tape marker reached            */
    K9800_ERR_BOT           = -5,   /**< Load point / beginning-of-tape        */
    K9800_ERR_TIMEOUT       = -6,   /**< Operation timed out                   */
    K9800_ERR_PARITY        = -7,   /**< Read parity error                     */
    K9800_ERR_OVERRUN       = -8,   /**< Read buffer overrun                   */
    K9800_ERR_PARAM         = -9,   /**< Invalid parameter                     */
    K9800_ERR_BUSY          = -10,  /**< Another operation already in progress */
    K9800_ERR_ABORTED       = -11,  /**< Operation aborted by caller           */
    K9800_ERR_DMA           = -12,  /**< DMA transfer error                    */
} K9800_Error_t;

/* ============================================================================
 * Track / density configuration
 * ============================================================================ */
typedef enum {
    K9800_TRACK_9 = 0,  /**< 9-track (WD0–WD7 + WDP)  — default */
    K9800_TRACK_7 = 1,  /**< 7-track (WDA–WDG + WDP)            */
} K9800_TrackConfig_t;

typedef enum {
    K9800_DENSITY_800  = 0,  /**< 800 cpi  (NRZI) — default    */
    K9800_DENSITY_1600 = 1,  /**< 1600 cpi (Phase-Encoded)      */
} K9800_DensityConfig_t;

/* ============================================================================
 * Transport status snapshot
 * ============================================================================ */
typedef struct {
    bool online;         /**< Transport is on-line (ONL)        */
    bool ready;          /**< Transport ready to receive cmds   */
    bool tape_running;   /**< Capstan driving tape (RNG)        */
    bool rewinding;      /**< Rewind operation active (RWD)     */
    bool at_load_point;  /**< Tape at BOT load-point marker     */
    bool at_eot;         /**< End-of-tape marker under sensor   */
    bool file_protect;   /**< Supply reel has no write ring     */
    bool write_enable;   /**< Supply reel has write-enable ring */
} K9800_TransportStatus_t;

/* ============================================================================
 * Initialisation configuration
 * ============================================================================ */
typedef struct {
    K9800_TrackConfig_t   tracks;      /**< 7-track or 9-track         */
    K9800_DensityConfig_t density;     /**< 800 or 1600 cpi            */
    bool                  overwrite;   /**< Enable overwrite (OVW) mode */
    uint32_t              tape_speed_ips; /**< Tape speed (10–25 ips)  */
} K9800_Config_t;

/** Default configuration (9-track, 800 cpi, 12.5 ips, no overwrite) */
#define K9800_DEFAULT_CONFIG  ((K9800_Config_t){ \
    .tracks        = K9800_TRACK_9,  \
    .density       = K9800_DENSITY_800, \
    .overwrite     = false,          \
    .tape_speed_ips = 12U            \
})

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * @brief  Initialise the Kennedy 9800 driver, GPIO, timers, DMA, and FreeRTOS
 *         resources.  Must be called once before any other K9800_* function,
 *         typically from the main task before osKernelStart().
 *
 * @param  cfg  Transport configuration; pass NULL to use K9800_DEFAULT_CONFIG.
 * @return K9800_OK on success.
 */
K9800_Error_t K9800_Init(const K9800_Config_t *cfg);

/**
 * @brief  Deinitialise the driver, release all hardware and RTOS resources.
 *         Deasserts all command lines and stops timers.
 */
void K9800_DeInit(void);

/**
 * @brief  Retrieve a snapshot of all J1 status lines.
 * @param  status  Pointer to caller-supplied status structure.
 * @return K9800_OK.
 */
K9800_Error_t K9800_GetStatus(K9800_TransportStatus_t *status);

/**
 * @brief  Block until the transport reports Ready (RDY = true), meaning tape
 *         is loaded past the load point and not rewinding.
 *
 * @param  timeout_ms  Maximum wait in milliseconds.
 * @return K9800_OK when ready; K9800_ERR_TIMEOUT if the transport does not
 *         become ready within the timeout.
 */
K9800_Error_t K9800_WaitReady(uint32_t timeout_ms);

/**
 * @brief  Assert or deassert the Transport Select (SLT) line.
 *         SLT must be true (asserted) for the entire duration of any write
 *         sequence.  K9800_WriteBlock() and K9800_ReadBlock() manage SLT
 *         automatically; this function is provided for manual control.
 */
K9800_Error_t K9800_Select(bool select);

/**
 * @brief  Write a single inter-record block to tape.
 *
 * @details Performs the complete write sequence documented in §1.8 / Figure 1-6:
 *          1. Verifies write-enable ring present (WEK true).
 *          2. Asserts SLT, SWS, then SFC.
 *          3. Waits write-start delay (tWSD = 36 ms @ 12.5 ips).
 *          4. Clocks all bytes onto the write bus via TIM2-triggered DMA.
 *             WDS pulses are generated in hardware (TIM2_CH2 PWM).
 *          5. Appends CRCC byte (longitudinal XOR of record).
 *          6. After write-stop delay, asserts WARS to write LRCC.
 *          7. Deasserts SFC; waits for tape motion to cease.
 *          8. Deasserts SLT, SWS.
 *
 * @param  data  Pointer to data buffer (raw bytes, MSB = track 7).
 * @param  len   Number of bytes (1 – K9800_MAX_BLOCK_BYTES).
 * @return K9800_OK, K9800_ERR_WRITE_PROTECT, K9800_ERR_EOT, K9800_ERR_TIMEOUT,
 *         K9800_ERR_NOT_READY, or K9800_ERR_DMA.
 */
K9800_Error_t K9800_WriteBlock(const uint8_t *data, uint32_t len);

/**
 * @brief  Read a single inter-record block from tape.
 *
 * @details Performs the complete read sequence documented in §1.8 / Figure 1-7:
 *          1. Asserts SLT (SWS left deasserted → read mode).
 *          2. Asserts SFC; waits for read-gap-detect (RGAP) to go false
 *             (start of data block).
 *          3. For each rising RDS pulse, samples the read bus via EXTI ISR
 *             into a ring buffer which this task drains.
 *          4. When RGAP returns true (end of block), deasserts SFC.
 *          5. Verifies CRCC against received data.
 *
 * @param  buf      Caller-supplied receive buffer.
 * @param  buf_len  Size of buf in bytes.
 * @param  out_len  Receives the number of data bytes placed in buf
 *                  (excludes CRCC byte).
 * @return K9800_OK, K9800_ERR_PARITY, K9800_ERR_OVERRUN, K9800_ERR_EOT,
 *         K9800_ERR_TIMEOUT, or K9800_ERR_NOT_READY.
 */
K9800_Error_t K9800_ReadBlock(uint8_t *buf, uint32_t buf_len, uint32_t *out_len);

/**
 * @brief  Issue a Rewind Command (RWC pulse) to the transport.
 *         The transport will rewind past the load point and re-tension to BOT.
 *         Blocks until RWD goes false (rewind complete) or timeout expires.
 *
 * @param  timeout_ms  Maximum wait in milliseconds; pass 0 for default
 *                     (K9800_REWIND_TIMEOUT_MS = 60 000 ms).
 * @return K9800_OK or K9800_ERR_TIMEOUT.
 */
K9800_Error_t K9800_Rewind(uint32_t timeout_ms);

/**
 * @brief  Space forward (no data written/read) for an arbitrary tape move.
 *         Asserts SFC until either the desired time elapses or EOT is detected.
 *
 * @param  duration_ms  Time to run tape forward in milliseconds.
 * @return K9800_OK or K9800_ERR_EOT.
 */
K9800_Error_t K9800_SpaceForward(uint32_t duration_ms);

/**
 * @brief  Space in reverse for an arbitrary tape move.
 *         Asserts SRC until either the desired time elapses or BOT is detected.
 *
 * @param  duration_ms  Time to run tape in reverse in milliseconds.
 * @return K9800_OK or K9800_ERR_BOT.
 */
K9800_Error_t K9800_SpaceReverse(uint32_t duration_ms);

/**
 * @brief  Place the transport off-line via the OFFC pulse.
 *         After this call the transport ignores all remote commands until the
 *         operator presses ON LINE on the front panel.
 */
K9800_Error_t K9800_GoOffline(void);

/**
 * @brief  Abort any in-progress write or read operation.
 *         Deasserts SFC/SRC immediately; waits for tape to stop.
 *         Safe to call from a different task than the one currently blocked
 *         in K9800_WriteBlock() or K9800_ReadBlock().
 */
K9800_Error_t K9800_Abort(void);

/**
 * @brief  Register an optional callback invoked from the control task (NOT from ISR)
 *         whenever a status line changes.
 *
 * @param  cb   Callback function; pass NULL to deregister.
 */
typedef void (*K9800_StatusCallback_t)(const K9800_TransportStatus_t *status);
void K9800_RegisterStatusCallback(K9800_StatusCallback_t cb);

#ifdef __cplusplus
}
#endif
