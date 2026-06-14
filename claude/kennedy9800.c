/**
 * @file    kennedy9800.c
 * @brief   Kennedy Model 9800 Digital Tape Transport driver.
 *
 * @details Implementation notes:
 *
 *  WRITE PATH (DMA-driven)
 *  ───────────────────────
 *  K9800_WriteBlock() prepares a write_dma_buf[] of uint16_t values where each
 *  entry is the active-low GPIO word to be clocked onto GPIOE[8:0].  Odd parity
 *  (IBM 9-track NRZI) is computed and inserted into bit 8 (WDP channel).
 *
 *  TIM2 runs at 10 kHz (100 µs/character for 800 cpi @ 12.5 ips).
 *  On every TIM2 update event, DMA1_Stream7_Ch3 transfers one halfword from
 *  write_dma_buf[] to GPIOE->ODR, presenting the next character's data on the
 *  write bus.  TIM2_CH2 (PB3, AF1) generates the WDS pulse automatically in
 *  PWM2 mode: active-LOW for the last 3 µs of each 100 µs period.  The data
 *  bus is therefore stable for ≥97 µs before WDS fires (requirement: 0.5 µs).
 *
 *  After the final character + CRCC are clocked:
 *    1. DMA TC interrupt signals the write task via a semaphore.
 *    2. Write task busy-waits 800 µs (8 char times) using DWT.
 *    3. Write task fires TIM1 one-pulse (WARS) on PA8 to write LRCC.
 *    4. Write stop delay (2 ms) elapses; SFC is deasserted.
 *
 *  READ PATH (EXTI interrupt-driven)
 *  ──────────────────────────────────
 *  With SFC asserted and SWS deasserted (read mode), the Kennedy drives RDS
 *  as a 2 µs active-LOW pulse for each byte recovered from tape.  EXTI0 on PD0
 *  fires on the RDS falling edge.  The ISR:
 *    - Reads GPIOD->IDR atomically.
 *    - Extracts RD[7:0] from PD[15:8] and RDP from PD7.
 *    - Verifies odd parity; sets a parity-error flag on mismatch.
 *    - Pushes the byte into read_ring_buf[] (lock-free single-producer /
 *      single-consumer ring; ISR is producer, read task is consumer).
 *    - Signals read_sem semaphore so the blocked read task can drain the ring.
 *
 *  EXTI1 on PD1 (RGAP) detects the inter-record gap; its ISR signals
 *  rgap_event_flags to unblock the read task at block boundaries.
 *
 *  STATUS MONITORING
 *  ─────────────────
 *  A 20 ms FreeRTOS software timer drives k9800_status_timer_cb() which polls
 *  all J1 status pins, compares against the cached state, and notifies the
 *  application callback on change.  Critical events (EOT, LDP transition) are
 *  also captured immediately by EXTI ISRs.
 *
 *  THREAD SAFETY
 *  ─────────────
 *  All public K9800_* functions acquire api_mutex before modifying transport
 *  state; K9800_Abort() and K9800_GetStatus() are additionally safe to call
 *  concurrently because they only set a volatile abort flag or read a cached
 *  struct under short critical sections.
 */

#include "kennedy9800.h"
#include "kennedy9800_pins.h"

/* FreeRTOS / CMSIS-RTOS2 */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* STM32 HAL */
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdbool.h>

/* ============================================================================
 * Private type definitions
 * ============================================================================ */

/** Driver operational state (internal) */
typedef enum {
    K9800_ST_UNINIT = 0,
    K9800_ST_IDLE,         /**< Initialised, no operation in progress      */
    K9800_ST_WRITING,      /**< Write sequence active                      */
    K9800_ST_READING,      /**< Read sequence active                       */
    K9800_ST_SPACING,      /**< Tape motion without data (space fwd/rev)   */
    K9800_ST_REWINDING,    /**< Rewind sequence active                     */
    K9800_ST_ABORTING,     /**< Abort requested, unwinding in progress     */
} K9800_InternalState_t;

/* ============================================================================
 * Read ring-buffer – lock-free SPSC (ISR producer / task consumer)
 * ============================================================================ */
#define K9800_READ_RING_SIZE    (4096U)          /* must be power of two     */
#define K9800_READ_RING_MASK    (K9800_READ_RING_SIZE - 1U)

typedef struct {
    volatile uint8_t  buf[K9800_READ_RING_SIZE];
    volatile uint32_t head;   /* written by ISR                             */
    volatile uint32_t tail;   /* read by task                               */
    volatile bool     parity_err;
    volatile bool     overrun;
} K9800_ReadRing_t;

/* ============================================================================
 * Module-level state
 * ============================================================================ */
static struct {
    K9800_Config_t          cfg;
    K9800_InternalState_t   state;
    K9800_TransportStatus_t status_cache;
    K9800_StatusCallback_t  status_cb;

    /* Write path ---------------------------------------------------------- */
    uint16_t               *write_dma_buf;      /* heap-allocated            */
    uint32_t                write_dma_len;       /* chars including CRCC      */
    volatile bool           write_dma_done;
    volatile bool           write_dma_err;

    /* Read path ----------------------------------------------------------- */
    K9800_ReadRing_t        read_ring;

    /* Event / sync objects (CMSIS-RTOS2) ---------------------------------- */
    osMutexId_t             api_mutex;
    osSemaphoreId_t         write_dma_sem;   /* TIM2 DMA TC → write task     */
    osSemaphoreId_t         read_byte_sem;   /* RDS ISR → read task          */
    osEventFlagsId_t        rgap_flags;      /* RGAP edge events              */
    osEventFlagsId_t        status_flags;    /* EOT / LDP edge events         */
    osTimerId_t             poll_timer;      /* 20 ms status poll             */

    /* HAL handles --------------------------------------------------------- */
    TIM_HandleTypeDef       htim2;           /* character-rate / WDS PWM     */
    TIM_HandleTypeDef       htim1;           /* WARS one-pulse               */
    DMA_HandleTypeDef       hdma_tim2_up;    /* TIM2_UP → GPIOE->ODR         */

    volatile bool           abort_requested;
    volatile bool           eot_flag;
    volatile bool           ldp_flag;
} g;

/* RGAP event flag bits */
#define RGAP_WENT_LOW   (1U << 0)   /* gap ended, data starting              */
#define RGAP_WENT_HIGH  (1U << 1)   /* data ended, gap starting              */

/* Status event flag bits */
#define STATUS_EOT      (1U << 0)
#define STATUS_LDP      (1U << 1)

/* ============================================================================
 * Forward declarations of internal helpers
 * ============================================================================ */
static K9800_Error_t hw_gpio_init(void);
static K9800_Error_t hw_tim2_init(void);
static K9800_Error_t hw_tim1_init(void);
static K9800_Error_t hw_dma_init(void);

static void          cmd_assert  (GPIO_TypeDef *port, uint16_t pin);
static void          cmd_deassert(GPIO_TypeDef *port, uint16_t pin);
static void          cmd_pulse   (GPIO_TypeDef *port, uint16_t pin, uint32_t us);
static bool          status_read (GPIO_TypeDef *port, uint16_t pin);

static void          dwt_delay_us(uint32_t us);
static void          wars_pulse  (void);

static uint8_t       crcc_compute(const uint8_t *data, uint32_t len);
static uint16_t      data_to_gpio(uint8_t byte);

static K9800_Error_t write_block_internal(const uint8_t *data, uint32_t len);
static K9800_Error_t read_block_internal (uint8_t *buf, uint32_t buf_len,
                                          uint32_t *out_len);

static void k9800_poll_timer_cb(void *arg);
static void k9800_update_status_cache(void);

/* ============================================================================
 * DWT cycle-counter delay (1 µs resolution)
 * ============================================================================ */
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DEM_CR      (*(volatile uint32_t *)0xE000EDFC)
#define DEM_CR_TRCENA   (1U << 24)

static void dwt_init(void)
{
    DEM_CR  |= DEM_CR_TRCENA;
    DWT_CTRL|= 1U;           /* enable cycle counter */
}

static void dwt_delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((DWT_CYCCNT - start) < ticks) { __NOP(); }
}

/* ============================================================================
 * GPIO helpers – all signals are active-LOW (zero-true)
 * ============================================================================ */

/** Assert (drive LOW = TRUE) */
static inline void cmd_assert(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/** Deassert (drive HIGH = FALSE) */
static inline void cmd_deassert(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

/**
 * Generate a minimum-width active-LOW pulse using DWT busy-wait.
 * Suitable for OFFC, RWC (≥2 µs), and manual WARS (≥2 µs).
 * Must NOT be called from ISR context for durations > ~10 µs.
 */
static void cmd_pulse(GPIO_TypeDef *port, uint16_t pin, uint32_t us)
{
    cmd_assert(port, pin);
    dwt_delay_us(us);
    cmd_deassert(port, pin);
}

/** Read an active-LOW status line; returns true when the Kennedy asserts it */
static inline bool status_read(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

/* ============================================================================
 * WARS one-pulse via TIM1_CH1 (PA8, AF1)
 * ============================================================================ */
static void wars_pulse(void)
{
    /* One-pulse mode: asserting TIM1 counter start triggers a single
     * PWM2 active-LOW output for K9800_TIM1_ARR µs on PA8.           */
    __HAL_TIM_SET_COUNTER(&g.htim1, 0);
    HAL_TIM_OnePulse_Start(&g.htim1, TIM_CHANNEL_1);
    /* Wait for pulse completion (3 µs + margin) before returning.    */
    dwt_delay_us(K9800_TIM1_ARR + 2U);
    HAL_TIM_OnePulse_Stop(&g.htim1, TIM_CHANNEL_1);
    /* Ensure PA8 returns HIGH (deasserted) after one-pulse stops.    */
    cmd_deassert(K9800_WARS_PORT, K9800_WARS_PIN);
}

/* ============================================================================
 * Parity + data encoding
 * ============================================================================ */

/**
 * Encode one data byte for the GPIOE write bus.
 *
 * WD0–WD7 on GPIOE[7:0], WDP on GPIOE[8], all active-LOW.
 *   GPIOE bit = 0 → channel TRUE  → NRZI flux transition (logical 1)
 *   GPIOE bit = 1 → channel FALSE → no transition (logical 0)
 *
 * IBM 9-track NRZI odd parity: total 1-bits across all 9 channels is odd.
 *   __builtin_parity(byte) == 1  → byte already has odd count → WDP = 0
 *   __builtin_parity(byte) == 0  → even count → add WDP = 1 → GPIOE[8] = 0
 *
 * gpio[8] = ~__builtin_parity(byte) & 1  (0 when parity needed, 1 otherwise)
 *
 * Verification: byte = 0x03 (2 ones, even) →
 *   ~byte & 0xFF = 0xFC  (bits 0,1 = 0 = active)
 *   parity = 0 → gpio[8] = 1 – no wait, that's wrong, let me recheck:
 *   __builtin_parity(0x03) = 0 (even) → parity_bit_needed = 1
 *   gpio[8] = !parity_bit_needed << 8  ... no.
 *
 *   If parity_bit_needed = 1 → we need WDP = 1 → GPIOE[8] = 0 (active-low)
 *   gpio[8] = (parity_bit_needed == 1) ? 0 : (1<<8)
 *           = ~parity_bit_needed & 1 ... = !parity_bit_needed
 *   gpio[8] = __builtin_parity(byte)   (0 when parity bit required)
 *
 * Compact form: gpio_val = (~byte & 0xFF) | ((uint16_t)__builtin_parity(byte) << 8)
 */
static inline uint16_t data_to_gpio(uint8_t byte)
{
    return (uint16_t)((~(uint16_t)byte & 0x00FFU)
                    | ((uint16_t)__builtin_parity(byte) << 8));
}

/** Longitudinal XOR of the record (CRCC for 9-track NRZI). */
static uint8_t crcc_compute(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

/* ============================================================================
 * Status cache + callback
 * ============================================================================ */
static void k9800_update_status_cache(void)
{
    K9800_TransportStatus_t s;
    s.online        = status_read(K9800_ONL_PORT, K9800_ONL_PIN);
    s.rewinding     = status_read(K9800_RWD_PORT, K9800_RWD_PIN);
    s.file_protect  = status_read(K9800_FPT_PORT, K9800_FPT_PIN);
    s.at_load_point = status_read(K9800_LDP_PORT, K9800_LDP_PIN);
    s.write_enable  = status_read(K9800_WEK_PORT, K9800_WEK_PIN);
    s.ready         = status_read(K9800_RDY_PORT, K9800_RDY_PIN);
    s.at_eot        = status_read(K9800_EOT_PORT, K9800_EOT_PIN);
    s.tape_running  = status_read(K9800_RNG_PORT, K9800_RNG_PIN);

    bool changed = (memcmp(&s, &g.status_cache, sizeof(s)) != 0);
    g.status_cache = s;

    if (changed && g.status_cb != NULL) {
        g.status_cb(&g.status_cache);
    }
}

/** 20 ms software timer callback – polls all status lines */
static void k9800_poll_timer_cb(void *arg)
{
    (void)arg;
    k9800_update_status_cache();
}

/* ============================================================================
 * HAL initialisation helpers
 * ============================================================================ */

static K9800_Error_t hw_gpio_init(void)
{
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef io = {0};

    /* ── GPIOA: command outputs (all deasserted = HIGH on reset) ─────── */
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_MEDIUM;
    io.Pin   = K9800_SFC_PIN | K9800_SRC_PIN | K9800_RWC_PIN | K9800_SLT_PIN
             | K9800_SWS_PIN | K9800_OFFC_PIN | K9800_OVW_PIN | K9800_DDS_PIN;
    HAL_GPIO_Init(GPIOA, &io);
    /* Deassert all command outputs at init */
    HAL_GPIO_WritePin(GPIOA,
        K9800_SFC_PIN | K9800_SRC_PIN | K9800_RWC_PIN | K9800_SLT_PIN |
        K9800_SWS_PIN | K9800_OFFC_PIN | K9800_OVW_PIN | K9800_DDS_PIN,
        GPIO_PIN_SET);

    /* PA8 will be reconfigured as TIM1_CH1 AF in hw_tim1_init(). */

    /* ── GPIOB: WDS (TIM2_CH2 AF1 on PB3), ACLD output, WDS gpio ─────── */
    /* PB3 → TIM2_CH2 alternate function (configured in hw_tim2_init)     */
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    io.Pin   = K9800_ACLD_PIN;
    HAL_GPIO_Init(GPIOB, &io);
    /* ACLD deasserted (HIGH) = let Kennedy manage clipping automatically   */
    HAL_GPIO_WritePin(K9800_ACLD_PORT, K9800_ACLD_PIN, GPIO_PIN_SET);

    /* ── GPIOC: status inputs (pull-up, interrupt on key pins) ───────── */
    io.Mode  = GPIO_MODE_INPUT;
    io.Pull  = GPIO_PULLUP;
    io.Pin   = K9800_ONL_PIN | K9800_RWD_PIN | K9800_FPT_PIN
             | K9800_WEK_PIN | K9800_RDY_PIN;
    HAL_GPIO_Init(GPIOC, &io);

    /* LDP (PC3) – both edges for load-point detection                    */
    io.Mode  = GPIO_MODE_IT_RISING_FALLING;
    io.Pull  = GPIO_PULLUP;
    io.Pin   = K9800_LDP_PIN;
    HAL_GPIO_Init(GPIOC, &io);

    /* EOT (PC6) – falling edge (Kennedy asserts LOW = EOT)               */
    io.Mode  = GPIO_MODE_IT_FALLING;
    io.Pull  = GPIO_PULLUP;
    io.Pin   = K9800_EOT_PIN;
    HAL_GPIO_Init(GPIOC, &io);

    /* RNG (PC7) – both edges to detect tape motion state changes          */
    io.Mode  = GPIO_MODE_IT_RISING_FALLING;
    io.Pull  = GPIO_PULLUP;
    io.Pin   = K9800_RNG_PIN;
    HAL_GPIO_Init(GPIOC, &io);

    /* ── GPIOD: read bus inputs (pull-up), RDS + RGAP as EXTI ────────── */
    /* RDS (PD0) – falling edge (Kennedy drives LOW for each character)   */
    io.Mode  = GPIO_MODE_IT_FALLING;
    io.Pull  = GPIO_PULLUP;
    io.Pin   = K9800_RDS_PIN;
    HAL_GPIO_Init(GPIOD, &io);

    /* RGAP (PD1) – both edges                                            */
    io.Mode  = GPIO_MODE_IT_RISING_FALLING;
    io.Pull  = GPIO_PULLUP;
    io.Pin   = K9800_RGAP_PIN;
    HAL_GPIO_Init(GPIOD, &io);

    /* RDP (PD7) and RD0–RD7 (PD8–PD15) – plain inputs with pull-up      */
    io.Mode  = GPIO_MODE_INPUT;
    io.Pull  = GPIO_PULLUP;
    io.Pin   = K9800_RDP_PIN
             | GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 | GPIO_PIN_11
             | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &io);

    /* ── GPIOE [8:0]: write data bus – output push-pull, start HIGH ──── */
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_HIGH;  /* Must settle before WDS; high slew */
    io.Pin   = K9800_WD_MASK;
    HAL_GPIO_Init(GPIOE, &io);
    /* Deassert all write data lines (HIGH = inactive)                    */
    GPIOE->ODR = (GPIOE->ODR & ~K9800_WD_MASK) | K9800_WD_MASK;

    /* ── NVIC priorities (all below configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) */
    HAL_NVIC_SetPriority(EXTI0_IRQn,     6, 0);  /* RDS  – highest priority  */
    HAL_NVIC_SetPriority(EXTI1_IRQn,     7, 0);  /* RGAP                     */
    HAL_NVIC_SetPriority(EXTI3_IRQn,     9, 0);  /* LDP                      */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn,   9, 0);  /* EOT(6) + RNG(7)          */
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    return K9800_OK;
}

static K9800_Error_t hw_dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /*
     * DMA1_Stream7_Channel3 – TIM2_UP → GPIOE->ODR
     * Memory: write_dma_buf[] (halfword, increment)
     * Peripheral: GPIOE->ODR  (halfword, no increment)
     */
    g.hdma_tim2_up.Instance                 = K9800_DMA_STREAM;
    g.hdma_tim2_up.Init.Channel             = K9800_DMA_CHANNEL;
    g.hdma_tim2_up.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    g.hdma_tim2_up.Init.PeriphInc           = DMA_PINC_DISABLE;
    g.hdma_tim2_up.Init.MemInc              = DMA_MINC_ENABLE;
    g.hdma_tim2_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    g.hdma_tim2_up.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    g.hdma_tim2_up.Init.Mode                = DMA_NORMAL;
    g.hdma_tim2_up.Init.Priority            = DMA_PRIORITY_HIGH;
    g.hdma_tim2_up.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&g.hdma_tim2_up) != HAL_OK) {
        return K9800_ERR_DMA;
    }

    /* Link DMA handle to TIM2 update DMA request */
    __HAL_LINKDMA(&g.htim2, hdma[TIM_DMA_ID_UPDATE], g.hdma_tim2_up);

    /* DMA TC and TE interrupts */
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

    return K9800_OK;
}

static K9800_Error_t hw_tim2_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /*
     * TIM2 – 100 µs period (10 kHz) for 800 cpi @ 12.5 ips
     * APB1 timer clock = 84 MHz on STM32F407 @ 168 MHz
     * PSC = 83 → tick = 1 µs; ARR = 99 → 100 µs period
     */
    g.htim2.Instance               = TIM2;
    g.htim2.Init.Prescaler         = K9800_TIM2_PSC;
    g.htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    g.htim2.Init.Period            = K9800_TIM2_ARR;
    g.htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    g.htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&g.htim2) != HAL_OK) {
        return K9800_ERR_NOT_INIT;
    }

    /*
     * TIM2_CH2 → WDS output on PB3 (AF1)
     *
     * PWM mode 2, polarity LOW:
     *   Output = HIGH when CNT < CCR2 (inactive)
     *   Output = LOW  when CNT ≥ CCR2 (active = WDS asserted)
     *
     * CCR2 = 97 → WDS LOW for ticks 97–99 = 3 µs  (requirement ≥2 µs)
     * Data bus written at tick 0 (DMA on update event) → 97 µs setup time.
     */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM2;
    oc.Pulse        = K9800_TIM2_WDS_CCR;
    oc.OCPolarity   = TIM_OCPOLARITY_LOW;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCIdleState  = TIM_OCIDLESTATE_SET;   /* WDS HIGH (inactive) when idle */

    if (HAL_TIM_PWM_ConfigChannel(&g.htim2, &oc, TIM_CHANNEL_2) != HAL_OK) {
        return K9800_ERR_NOT_INIT;
    }

    /* Configure PB3 as TIM2_CH2 alternate function */
    GPIO_InitTypeDef io = {0};
    io.Pin       = GPIO_PIN_3;
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_NOPULL;
    io.Speed     = GPIO_SPEED_FREQ_HIGH;
    io.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &io);

    /* Timer interrupt for overflow (used only for abort checking)      */
    HAL_NVIC_SetPriority(TIM2_IRQn, 8, 0);
    /* Do NOT enable TIM2 IRQ here – started/stopped per write operation */

    return K9800_OK;
}

static K9800_Error_t hw_tim1_init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    /*
     * TIM1 – WARS one-pulse generator
     * APB2 timer clock = 168 MHz on STM32F407
     * PSC = 167 → tick = 1 µs; ARR = 3 → 3 µs pulse width
     */
    g.htim1.Instance               = TIM1;
    g.htim1.Init.Prescaler         = K9800_TIM1_PSC;
    g.htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    g.htim1.Init.Period            = K9800_TIM1_ARR;
    g.htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    g.htim1.Init.RepetitionCounter = 0;
    g.htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_OnePulse_Init(&g.htim1, TIM_OPMODE_SINGLE) != HAL_OK) {
        return K9800_ERR_NOT_INIT;
    }

    /*
     * TIM1_CH1 → WARS output on PA8 (AF1)
     * PWM2, polarity LOW: output LOW from CCR1 to ARR (active), HIGH otherwise.
     * CCR1 = 0 → pulse begins immediately: LOW for 3 µs.
     */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM2;
    oc.Pulse        = K9800_TIM1_WARS_CCR;
    oc.OCPolarity   = TIM_OCPOLARITY_LOW;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCIdleState  = TIM_OCIDLESTATE_SET;  /* WARS HIGH (inactive) at idle */
    oc.OCFastMode   = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&g.htim1, &oc, TIM_CHANNEL_1) != HAL_OK) {
        return K9800_ERR_NOT_INIT;
    }

    /* Configure PA8 as TIM1_CH1 AF */
    GPIO_InitTypeDef io = {0};
    io.Pin       = GPIO_PIN_8;
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_NOPULL;
    io.Speed     = GPIO_SPEED_FREQ_HIGH;
    io.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &io);

    return K9800_OK;
}

/* ============================================================================
 * Write path helpers
 * ============================================================================ */

/**
 * Prepare the DMA buffer.
 * Allocates (or reuses) write_dma_buf[], encodes data + CRCC.
 * Returns K9800_OK or K9800_ERR_PARAM.
 */
static K9800_Error_t write_prepare_buf(const uint8_t *data, uint32_t len)
{
    /* +1 for CRCC byte */
    uint32_t total = len + 1U;

    if (g.write_dma_buf != NULL) {
        vPortFree(g.write_dma_buf);
        g.write_dma_buf = NULL;
    }

    g.write_dma_buf = (uint16_t *)pvPortMalloc(total * sizeof(uint16_t));
    if (g.write_dma_buf == NULL) {
        return K9800_ERR_PARAM;
    }

    /* Encode each byte as active-low GPIO word with odd parity */
    for (uint32_t i = 0; i < len; i++) {
        g.write_dma_buf[i] = data_to_gpio(data[i]);
    }

    /* Append CRCC */
    uint8_t crc = crcc_compute(data, len);
    g.write_dma_buf[len] = data_to_gpio(crc);
    g.write_dma_len = total;

    return K9800_OK;
}

/**
 * Start the TIM2+DMA write sequence.
 * Returns once DMA is armed; completion is signalled via write_dma_sem.
 */
static K9800_Error_t write_start_dma(void)
{
    g.write_dma_done = false;
    g.write_dma_err  = false;

    /* Arm DMA: memory → GPIOE->ODR, g.write_dma_len halfword transfers.  */
    if (HAL_DMA_Start_IT(&g.hdma_tim2_up,
                         (uint32_t)g.write_dma_buf,
                         (uint32_t)&GPIOE->ODR,
                         g.write_dma_len) != HAL_OK) {
        return K9800_ERR_DMA;
    }

    /* Enable TIM2 update DMA request; start WDS PWM and base timer.     */
    __HAL_TIM_ENABLE_DMA(&g.htim2, TIM_DMA_UPDATE);
    __HAL_TIM_SET_COUNTER(&g.htim2, 0);
    HAL_TIM_PWM_Start(&g.htim2, TIM_CHANNEL_2);
    HAL_TIM_Base_Start(&g.htim2);

    return K9800_OK;
}

/** Stop TIM2/DMA and deassert write bus */
static void write_stop_dma(void)
{
    HAL_TIM_PWM_Stop(&g.htim2, TIM_CHANNEL_2);
    HAL_TIM_Base_Stop(&g.htim2);
    __HAL_TIM_DISABLE_DMA(&g.htim2, TIM_DMA_UPDATE);
    HAL_DMA_Abort(&g.hdma_tim2_up);

    /* Return write bus to inactive (all HIGH = deasserted) */
    GPIOE->ODR = (GPIOE->ODR & ~K9800_WD_MASK) | K9800_WD_MASK;
}

/* ============================================================================
 * Internal operation implementations
 * ============================================================================ */

static K9800_Error_t write_block_internal(const uint8_t *data, uint32_t len)
{
    K9800_Error_t ret;

    /* Verify write-enable ring */
    if (!status_read(K9800_WEK_PORT, K9800_WEK_PIN)) {
        return K9800_ERR_WRITE_PROTECT;
    }
    /* Verify transport ready */
    if (!status_read(K9800_RDY_PORT, K9800_RDY_PIN)) {
        return K9800_ERR_NOT_READY;
    }

    /* Prepare DMA buffer (data + CRCC) */
    ret = write_prepare_buf(data, len);
    if (ret != K9800_OK) { return ret; }

    /* ── Phase 1: Assert SLT and SWS, then SFC ── */
    cmd_assert(K9800_SLT_PORT, K9800_SLT_PIN);   /* Select transport        */
    cmd_assert(K9800_SWS_PORT, K9800_SWS_PIN);   /* Write mode              */
    /* SWS must be true at the leading edge of SFC (§1.9.3) */
    dwt_delay_us(5);
    cmd_assert(K9800_SFC_PORT, K9800_SFC_PIN);   /* Tape starts moving      */

    /* ── Phase 2: Write start delay tWSD = 36 ms ── */
    /* Tape ramps to speed (30 ms) plus gap delay (6 ms) per Figure 1-6.  */
    osDelay(K9800_WRITE_START_DELAY_MS);

    /* Check for abort or EOT during ramp */
    if (g.abort_requested || g.eot_flag) {
        cmd_deassert(K9800_SFC_PORT, K9800_SFC_PIN);
        cmd_deassert(K9800_SWS_PORT, K9800_SWS_PIN);
        cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);
        ret = g.abort_requested ? K9800_ERR_ABORTED : K9800_ERR_EOT;
        goto write_cleanup;
    }

    /* ── Phase 3: DMA write – all characters clocked by TIM2/DMA ── */
    g.state = K9800_ST_WRITING;
    ret = write_start_dma();
    if (ret != K9800_OK) {
        cmd_deassert(K9800_SFC_PORT, K9800_SFC_PIN);
        cmd_deassert(K9800_SWS_PORT, K9800_SWS_PIN);
        cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);
        goto write_cleanup;
    }

    /* Block until DMA transfer complete (or timeout) */
    uint32_t timeout_ticks = (g.write_dma_len * K9800_CHAR_PERIOD_US / 1000U)
                             + K9800_WRITE_STOP_DELAY_MS + 100U;
    osStatus_t stat = osSemaphoreAcquire(g.write_dma_sem,
                                         (uint32_t)timeout_ticks);

    write_stop_dma();   /* Stop TIM2/DMA regardless of outcome */

    if (stat != osOK) {
        ret = K9800_ERR_TIMEOUT;
        goto write_deassert;
    }
    if (g.write_dma_err) {
        ret = K9800_ERR_DMA;
        goto write_deassert;
    }
    if (g.abort_requested) {
        ret = K9800_ERR_ABORTED;
        goto write_deassert;
    }

    /* ── Phase 4: Write stop delay tSD = 2 ms, then WARS ── */
    /* The write-stop delay must elapse before asserting WARS (§1.9.3).  */
    osDelay(K9800_WRITE_STOP_DELAY_MS);

    /*
     * WARS leading edge: 8 character times (800 µs) after last data WDS.
     * The DMA completed at ~the last WDS event; 2 ms stop delay already
     * provides > 800 µs, but add an explicit 800 µs guard for clarity.
     */
    dwt_delay_us(K9800_WARS_DELAY_US);
    wars_pulse();   /* 3 µs active-LOW pulse on PA8 (TIM1_CH1) */

    ret = K9800_OK;

write_deassert:
    /* ── Phase 5: Deassert SFC; wait for tape to stop ── */
    cmd_deassert(K9800_SFC_PORT, K9800_SFC_PIN);
    cmd_deassert(K9800_SWS_PORT, K9800_SWS_PIN);

    /* Wait for RNG (tape running) to go false */
    uint32_t stop_deadline = osKernelGetTickCount() + K9800_RAMP_TIME_MS + 50U;
    while (status_read(K9800_RNG_PORT, K9800_RNG_PIN)) {
        if (osKernelGetTickCount() > stop_deadline) { break; }
        osDelay(2);
    }

    cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);   /* Deselect */

write_cleanup:
    g.state = K9800_ST_IDLE;
    if (g.write_dma_buf != NULL) {
        vPortFree(g.write_dma_buf);
        g.write_dma_buf = NULL;
    }
    return ret;
}

static K9800_Error_t read_block_internal(uint8_t *buf, uint32_t buf_len,
                                         uint32_t *out_len)
{
    if (buf == NULL || buf_len == 0 || out_len == NULL) {
        return K9800_ERR_PARAM;
    }
    if (!status_read(K9800_RDY_PORT, K9800_RDY_PIN)) {
        return K9800_ERR_NOT_READY;
    }

    /* Reset ring buffer and flags */
    g.read_ring.head      = 0;
    g.read_ring.tail      = 0;
    g.read_ring.parity_err = false;
    g.read_ring.overrun    = false;
    g.eot_flag             = false;
    *out_len = 0;

    /* Drain any stale semaphore tokens */
    while (osSemaphoreAcquire(g.read_byte_sem, 0) == osOK) {}
    osEventFlagsClear(g.rgap_flags, RGAP_WENT_LOW | RGAP_WENT_HIGH);

    /* ── Phase 1: Assert SLT (SWS left deasserted = read mode) ── */
    cmd_assert(K9800_SLT_PORT, K9800_SLT_PIN);
    cmd_deassert(K9800_SWS_PORT, K9800_SWS_PIN);
    dwt_delay_us(5);

    /* ── Phase 2: Assert SFC; wait for RGAP to go false (block start) ── */
    g.state = K9800_ST_READING;
    cmd_assert(K9800_SFC_PORT, K9800_SFC_PIN);

    /* RGAP is true in the inter-record gap; wait for it to go LOW
     * (Kennedy starts sending data bytes).                            */
    uint32_t flags = osEventFlagsWait(g.rgap_flags, RGAP_WENT_LOW,
                                      osFlagsWaitAny,
                                      K9800_READY_TIMEOUT_MS);
    if ((flags & osFlagsError) != 0U) {
        /* Timeout – no data block found within window */
        cmd_deassert(K9800_SFC_PORT, K9800_SFC_PIN);
        cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);
        g.state = K9800_ST_IDLE;
        return K9800_ERR_TIMEOUT;
    }

    /* ── Phase 3: Drain read ring buffer until RGAP goes HIGH again ── */
    bool block_done = false;
    K9800_Error_t ret = K9800_OK;

    while (!block_done) {
        /* Wait for either a byte in the ring or the end-of-block event.
         * Timeout = 10 × character period + generous margin.           */
        uint32_t ev = osEventFlagsWait(g.rgap_flags, RGAP_WENT_HIGH,
                                       osFlagsWaitAny | osFlagsNoClear, 1U);

        /* Drain all available bytes from ring buffer */
        uint32_t head = g.read_ring.head;   /* snapshot (ISR may update) */
        uint32_t tail = g.read_ring.tail;
        while (tail != head) {
            uint8_t b = g.read_ring.buf[tail & K9800_READ_RING_MASK];
            tail++;
            g.read_ring.tail = tail;

            if (*out_len < buf_len) {
                buf[(*out_len)++] = b;
            } else {
                g.read_ring.overrun = true;
            }
            head = g.read_ring.head;
        }

        if (g.abort_requested) {
            ret = K9800_ERR_ABORTED;
            block_done = true;
        } else if (g.eot_flag) {
            ret = K9800_ERR_EOT;
            block_done = true;
        } else if ((ev & RGAP_WENT_HIGH) != 0U) {
            /* End of block: RGAP back to true.  The last bytes in the
             * ring (including CRCC) may not have been drained yet.    */
            /* Give one more tick for the ISR to finish              */
            osDelay(1);
            head = g.read_ring.head;
            tail = g.read_ring.tail;
            while (tail != head) {
                uint8_t b = g.read_ring.buf[tail & K9800_READ_RING_MASK];
                tail++;
                g.read_ring.tail = tail;
                if (*out_len < buf_len) {
                    buf[(*out_len)++] = b;
                } else {
                    g.read_ring.overrun = true;
                }
                head = g.read_ring.head;
            }
            block_done = true;
        }
    }

    /* ── Phase 4: Deassert SFC; read stop delay ── */
    cmd_deassert(K9800_SFC_PORT, K9800_SFC_PIN);
    osDelay(K9800_READ_STOP_DELAY_MS);
    cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);

    /* Wait for tape to stop */
    uint32_t stop_deadline = osKernelGetTickCount() + K9800_RAMP_TIME_MS + 50U;
    while (status_read(K9800_RNG_PORT, K9800_RNG_PIN)) {
        if (osKernelGetTickCount() > stop_deadline) { break; }
        osDelay(2);
    }

    g.state = K9800_ST_IDLE;

    /* ── Phase 5: Check data integrity ── */
    if (g.read_ring.overrun) {
        return K9800_ERR_OVERRUN;
    }
    if (g.read_ring.parity_err) {
        return K9800_ERR_PARITY;
    }
    if (ret != K9800_OK) {
        return ret;
    }

    /* Verify and strip CRCC (last byte of received block) */
    if (*out_len >= 1U) {
        uint32_t data_len = *out_len - 1U;
        uint8_t  expected = crcc_compute(buf, data_len);
        uint8_t  received = buf[data_len];
        *out_len = data_len;
        if (expected != received) {
            return K9800_ERR_PARITY;
        }
    }

    return K9800_OK;
}

/* ============================================================================
 * Public API implementation
 * ============================================================================ */

K9800_Error_t K9800_Init(const K9800_Config_t *cfg)
{
    if (g.state != K9800_ST_UNINIT) {
        return K9800_OK;  /* idempotent */
    }

    /* Apply configuration */
    if (cfg != NULL) {
        g.cfg = *cfg;
    } else {
        g.cfg = K9800_DEFAULT_CONFIG;
    }

    dwt_init();

    /* Initialise hardware */
    K9800_Error_t ret;
    ret = hw_gpio_init();  if (ret != K9800_OK) { return ret; }
    ret = hw_dma_init();   if (ret != K9800_OK) { return ret; }
    ret = hw_tim2_init();  if (ret != K9800_OK) { return ret; }
    ret = hw_tim1_init();  if (ret != K9800_OK) { return ret; }

    /* Apply density select */
    if (g.cfg.density == K9800_DENSITY_1600) {
        cmd_assert(K9800_DDS_PORT, K9800_DDS_PIN);
    } else {
        cmd_deassert(K9800_DDS_PORT, K9800_DDS_PIN);
    }

    /* Apply overwrite mode */
    if (g.cfg.overwrite) {
        cmd_assert(K9800_OVW_PORT, K9800_OVW_PIN);
    }

    /* Create CMSIS-RTOS2 objects */
    static const osMutexAttr_t mutex_attr = {
        "K9800ApiMutex", osMutexRecursive | osMutexPrioInherit, NULL, 0
    };
    g.api_mutex = osMutexNew(&mutex_attr);
    if (g.api_mutex == NULL) { return K9800_ERR_NOT_INIT; }

    static const osSemaphoreAttr_t wdma_sem_attr = { "K9800WriteDmaSem", 0, NULL, 0 };
    g.write_dma_sem = osSemaphoreNew(1, 0, &wdma_sem_attr);
    if (g.write_dma_sem == NULL) { return K9800_ERR_NOT_INIT; }

    static const osSemaphoreAttr_t rbyte_sem_attr = { "K9800ReadByteSem", 0, NULL, 0 };
    g.read_byte_sem = osSemaphoreNew(K9800_READ_RING_SIZE, 0, &rbyte_sem_attr);
    if (g.read_byte_sem == NULL) { return K9800_ERR_NOT_INIT; }

    static const osEventFlagsAttr_t rgap_attr = { "K9800RgapFlags", 0, NULL, 0 };
    g.rgap_flags = osEventFlagsNew(&rgap_attr);
    if (g.rgap_flags == NULL) { return K9800_ERR_NOT_INIT; }

    static const osEventFlagsAttr_t status_attr = { "K9800StatusFlags", 0, NULL, 0 };
    g.status_flags = osEventFlagsNew(&status_attr);
    if (g.status_flags == NULL) { return K9800_ERR_NOT_INIT; }

    /* 20 ms status poll timer */
    static const osTimerAttr_t tmr_attr = { "K9800PollTimer", 0, NULL, 0 };
    g.poll_timer = osTimerNew(k9800_poll_timer_cb, osTimerPeriodic, NULL, &tmr_attr);
    if (g.poll_timer == NULL) { return K9800_ERR_NOT_INIT; }
    osTimerStart(g.poll_timer, 20U);  /* 20 ms */

    /* Take initial status snapshot */
    k9800_update_status_cache();

    g.state = K9800_ST_IDLE;
    return K9800_OK;
}

void K9800_DeInit(void)
{
    if (g.state == K9800_ST_UNINIT) { return; }

    K9800_Abort();

    osTimerStop(g.poll_timer);
    osTimerDelete(g.poll_timer);

    /* Deassert all command lines */
    HAL_GPIO_WritePin(GPIOA,
        K9800_SFC_PIN | K9800_SRC_PIN | K9800_RWC_PIN | K9800_SLT_PIN |
        K9800_SWS_PIN | K9800_OFFC_PIN | K9800_OVW_PIN | K9800_DDS_PIN,
        GPIO_PIN_SET);

    write_stop_dma();
    HAL_TIM_PWM_DeInit(&g.htim2);
    HAL_TIM_OnePulse_DeInit(&g.htim1);
    HAL_DMA_DeInit(&g.hdma_tim2_up);

    if (g.write_dma_buf) {
        vPortFree(g.write_dma_buf);
        g.write_dma_buf = NULL;
    }

    osMutexDelete(g.api_mutex);
    osSemaphoreDelete(g.write_dma_sem);
    osSemaphoreDelete(g.read_byte_sem);
    osEventFlagsDelete(g.rgap_flags);
    osEventFlagsDelete(g.status_flags);

    memset(&g, 0, sizeof(g));
    g.state = K9800_ST_UNINIT;
}

K9800_Error_t K9800_GetStatus(K9800_TransportStatus_t *status)
{
    if (status == NULL) { return K9800_ERR_PARAM; }
    if (g.state == K9800_ST_UNINIT) { return K9800_ERR_NOT_INIT; }
    /* Update cache immediately for freshest snapshot */
    k9800_update_status_cache();
    taskENTER_CRITICAL();
    *status = g.status_cache;
    taskEXIT_CRITICAL();
    return K9800_OK;
}

K9800_Error_t K9800_WaitReady(uint32_t timeout_ms)
{
    if (g.state == K9800_ST_UNINIT) { return K9800_ERR_NOT_INIT; }
    uint32_t deadline = osKernelGetTickCount() + timeout_ms;
    while (osKernelGetTickCount() < deadline) {
        k9800_update_status_cache();
        if (g.status_cache.ready) { return K9800_OK; }
        osDelay(50U);
    }
    return K9800_ERR_TIMEOUT;
}

K9800_Error_t K9800_Select(bool select)
{
    if (g.state == K9800_ST_UNINIT) { return K9800_ERR_NOT_INIT; }
    if (select) {
        cmd_assert(K9800_SLT_PORT, K9800_SLT_PIN);
    } else {
        cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);
    }
    return K9800_OK;
}

K9800_Error_t K9800_WriteBlock(const uint8_t *data, uint32_t len)
{
    if (g.state == K9800_ST_UNINIT)                   { return K9800_ERR_NOT_INIT; }
    if (data == NULL || len == 0 || len > K9800_MAX_BLOCK_BYTES)
                                                       { return K9800_ERR_PARAM; }
    if (osMutexAcquire(g.api_mutex, K9800_READY_TIMEOUT_MS) != osOK)
                                                       { return K9800_ERR_BUSY; }
    g.abort_requested = false;
    K9800_Error_t ret = write_block_internal(data, len);
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_ReadBlock(uint8_t *buf, uint32_t buf_len, uint32_t *out_len)
{
    if (g.state == K9800_ST_UNINIT)      { return K9800_ERR_NOT_INIT; }
    if (buf == NULL || buf_len == 0 || out_len == NULL)
                                          { return K9800_ERR_PARAM; }
    if (osMutexAcquire(g.api_mutex, K9800_READY_TIMEOUT_MS) != osOK)
                                          { return K9800_ERR_BUSY; }
    g.abort_requested = false;
    K9800_Error_t ret = read_block_internal(buf, buf_len, out_len);
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_Rewind(uint32_t timeout_ms)
{
    if (g.state == K9800_ST_UNINIT) { return K9800_ERR_NOT_INIT; }
    if (timeout_ms == 0U)           { timeout_ms = K9800_REWIND_TIMEOUT_MS; }

    if (osMutexAcquire(g.api_mutex, 1000U) != osOK) { return K9800_ERR_BUSY; }

    /* Transport must not be at load point to issue RWC (§1.9.2) */
    if (status_read(K9800_LDP_PORT, K9800_LDP_PIN)) {
        /* Already at BOT – nothing to rewind */
        osMutexRelease(g.api_mutex);
        return K9800_OK;
    }

    g.state = K9800_ST_REWINDING;
    cmd_pulse(K9800_RWC_PORT, K9800_RWC_PIN, K9800_RWC_PULSE_US);

    /* Wait for RWD to go true then false (rewind complete) */
    uint32_t deadline = osKernelGetTickCount() + timeout_ms;
    bool rwd_seen = false;
    K9800_Error_t ret = K9800_ERR_TIMEOUT;

    while (osKernelGetTickCount() < deadline) {
        bool rwd = status_read(K9800_RWD_PORT, K9800_RWD_PIN);
        if (!rwd_seen && rwd)  { rwd_seen = true; }
        if (rwd_seen && !rwd)  { ret = K9800_OK; break; }
        osDelay(100U);
    }

    g.state = K9800_ST_IDLE;
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_SpaceForward(uint32_t duration_ms)
{
    if (g.state == K9800_ST_UNINIT) { return K9800_ERR_NOT_INIT; }
    if (osMutexAcquire(g.api_mutex, 1000U) != osOK) { return K9800_ERR_BUSY; }

    g.state = K9800_ST_SPACING;
    g.eot_flag = false;
    cmd_assert(K9800_SLT_PORT, K9800_SLT_PIN);
    cmd_assert(K9800_SFC_PORT, K9800_SFC_PIN);

    uint32_t deadline = osKernelGetTickCount() + duration_ms;
    K9800_Error_t ret = K9800_OK;
    while (osKernelGetTickCount() < deadline) {
        if (g.eot_flag) { ret = K9800_ERR_EOT; break; }
        if (g.abort_requested) { ret = K9800_ERR_ABORTED; break; }
        osDelay(5U);
    }

    cmd_deassert(K9800_SFC_PORT, K9800_SFC_PIN);
    osDelay(K9800_RAMP_TIME_MS);
    cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);

    g.state = K9800_ST_IDLE;
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_SpaceReverse(uint32_t duration_ms)
{
    if (g.state == K9800_ST_UNINIT) { return K9800_ERR_NOT_INIT; }
    if (osMutexAcquire(g.api_mutex, 1000U) != osOK) { return K9800_ERR_BUSY; }

    g.state = K9800_ST_SPACING;
    cmd_assert(K9800_SLT_PORT, K9800_SLT_PIN);
    cmd_assert(K9800_SRC_PORT, K9800_SRC_PIN);

    uint32_t deadline = osKernelGetTickCount() + duration_ms;
    K9800_Error_t ret = K9800_OK;
    while (osKernelGetTickCount() < deadline) {
        if (status_read(K9800_LDP_PORT, K9800_LDP_PIN)) { ret = K9800_ERR_BOT; break; }
        if (g.abort_requested) { ret = K9800_ERR_ABORTED; break; }
        osDelay(5U);
    }

    cmd_deassert(K9800_SRC_PORT, K9800_SRC_PIN);
    osDelay(K9800_RAMP_TIME_MS);
    cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);

    g.state = K9800_ST_IDLE;
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_GoOffline(void)
{
    if (g.state == K9800_ST_UNINIT) { return K9800_ERR_NOT_INIT; }
    /* OFFC is gated only by SLT, not ON_LINE; it can interrupt a rewind */
    cmd_pulse(K9800_OFFC_PORT, K9800_OFFC_PIN, K9800_OFFC_PULSE_US);
    return K9800_OK;
}

K9800_Error_t K9800_Abort(void)
{
    if (g.state == K9800_ST_UNINIT) { return K9800_ERR_NOT_INIT; }

    g.abort_requested = true;

    /* Immediately deassert all motion commands */
    cmd_deassert(K9800_SFC_PORT, K9800_SFC_PIN);
    cmd_deassert(K9800_SRC_PORT, K9800_SRC_PIN);
    cmd_deassert(K9800_SWS_PORT, K9800_SWS_PIN);

    /* Stop DMA/timer regardless of current state */
    write_stop_dma();

    /* Deassert SLT after a short delay */
    osDelay(K9800_RAMP_TIME_MS + 10U);
    cmd_deassert(K9800_SLT_PORT, K9800_SLT_PIN);

    return K9800_OK;
}

void K9800_RegisterStatusCallback(K9800_StatusCallback_t cb)
{
    g.status_cb = cb;
}

/* ============================================================================
 * ISR-callable functions (called from kennedy9800_isr.c)
 * These functions execute entirely in interrupt context; they must be fast
 * and must only use ISR-safe RTOS calls (FromISR variants).
 * ============================================================================ */

/**
 * Called by EXTI0 handler (PD0 = RDS falling edge).
 * Reads one character from the read bus and pushes it into the ring buffer.
 * Must complete within the inter-character period (100 µs at 10 kHz).
 */
void K9800_ISR_RDS(void)
{
    /* Sample data bus atomically */
    uint32_t idr = GPIOD->IDR;

    /*
     * RD0–RD7 on PD[15:8], active-LOW → data bit = !GPIO bit
     * RDP on PD7, active-LOW → parity bit = !GPIO bit 7
     */
    uint8_t raw_data   = (uint8_t)(idr >> 8);           /* PD[15:8]        */
    uint8_t data_byte  = ~raw_data;                      /* invert: active-LOW → 1 */
    uint8_t gpio_parity = (uint8_t)((idr >> 7) & 0x01U); /* PD7 raw         */
    uint8_t parity_bit  = (uint8_t)(gpio_parity ^ 1U);   /* invert          */

    /* Verify IBM 9-track odd parity: total 1-bits must be odd */
    if ((__builtin_parity(data_byte) ^ parity_bit) != 1U) {
        g.read_ring.parity_err = true;
    }

    /* Push to ring buffer (lock-free: ISR is sole producer) */
    uint32_t next_head = (g.read_ring.head + 1U) & (K9800_READ_RING_SIZE - 1U);
    if (next_head == g.read_ring.tail) {
        g.read_ring.overrun = true;
        return;   /* Drop byte to avoid corrupting ring */
    }
    g.read_ring.buf[g.read_ring.head] = data_byte;
    g.read_ring.head = next_head;

    /* Signal the read task that data is available */
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)g.read_byte_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * Called by EXTI1 handler (PD1 = RGAP, both edges).
 * RGAP true  (HIGH) = inter-record gap (no data)
 * RGAP false (LOW)  = data block in progress
 */
void K9800_ISR_RGAP(void)
{
    BaseType_t woken = pdFALSE;
    if (HAL_GPIO_ReadPin(K9800_RGAP_PORT, K9800_RGAP_PIN) == GPIO_PIN_RESET) {
        /* RGAP went LOW: data block starting */
        xEventGroupSetBitsFromISR((EventGroupHandle_t)g.rgap_flags,
                                  RGAP_WENT_LOW, &woken);
    } else {
        /* RGAP went HIGH: data block ended (gap started) */
        xEventGroupSetBitsFromISR((EventGroupHandle_t)g.rgap_flags,
                                  RGAP_WENT_HIGH, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

/**
 * Called by EXTI3 handler (PC3 = LDP, both edges).
 */
void K9800_ISR_LDP(void)
{
    g.ldp_flag = status_read(K9800_LDP_PORT, K9800_LDP_PIN);
    BaseType_t woken = pdFALSE;
    xEventGroupSetBitsFromISR((EventGroupHandle_t)g.status_flags,
                              STATUS_LDP, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * Called by EXTI9_5 handler for EOT (PC6).
 */
void K9800_ISR_EOT(void)
{
    g.eot_flag = true;
    BaseType_t woken = pdFALSE;
    xEventGroupSetBitsFromISR((EventGroupHandle_t)g.status_flags,
                              STATUS_EOT, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * Called by the DMA1_Stream7 transfer-complete callback.
 * Releases write_dma_sem to unblock the write task.
 */
void K9800_ISR_DMA_TC(void)
{
    g.write_dma_done = true;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)g.write_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * Called by the DMA1_Stream7 transfer-error callback.
 */
void K9800_ISR_DMA_TE(void)
{
    g.write_dma_err = true;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)g.write_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
}
