/**
 * @file  kennedy9800.c
 * @brief Kennedy 9800 transport driver — ESP32-S3-N16R8 adaptation.
 *
 * Replaces STM32 TIM2-DMA write path with a GPTimer ISR driving GPIO directly.
 * Status inputs are read via MCP23017 I2C expander (polled every 20 ms).
 * Read data bus is sampled in a GPIO interrupt ISR triggered by RDS (GPIO47).
 * All timing-critical ISRs are placed in IRAM.
 *
 * Differences from the STM32 L4P5 reference driver:
 *   hw_gpio_init   — ESP-IDF gpio_config() instead of HAL_GPIO_Init()
 *   hw_timer_init  — GPTimer (gptimer API, IDF 5.x) instead of TIM2+DMA
 *   write path     — gptimer alarm ISR steps through write buffer, pulses WDS
 *   wars_pulse     — inline GPIO + esp_rom_delay_us(3)
 *   read path      — gpio_isr_handler on RDS GPIO47 fills ring buffer
 *   status         — MCP23017 polled over I2C; no EXTI on LDP/EOT lines
 *   delay          — esp_rom_delay_us() for µs, vTaskDelay() for ms
 *   RTOS           — CMSIS-RTOS v2 wrappers over FreeRTOS (same API as STM32)
 */

#include "kennedy9800.h"
#include "kennedy9800_pins.h"
#include "app_config.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "hal/gpio_ll.h"

#include <string.h>
#include <stdbool.h>

/* ============================================================================
 * Internal state
 * ============================================================================ */
typedef enum {
    K9800_ST_UNINIT = 0,
    K9800_ST_IDLE,
    K9800_ST_WRITING,
    K9800_ST_READING,
    K9800_ST_SPACING,
    K9800_ST_REWINDING,
} K9800_InternalState_t;

#define READ_RING_MASK  (K9800_READ_RING_SIZE - 1U)

typedef struct {
    volatile uint8_t  buf[K9800_READ_RING_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile bool     parity_err;
    volatile bool     overrun;
} ReadRing_t;

/* Write timer ISR shared state — kept in DRAM for ISR access */
static struct {
    const uint32_t  *buf;       /* pre-computed GPIO words */
    uint32_t         len;
    volatile uint32_t idx;
    volatile bool    done;
    volatile bool    err;
} s_wtx;

static struct {
    K9800_Config_t          cfg;
    K9800_InternalState_t   state;
    K9800_TransportStatus_t status_cache;
    K9800_StatusCallback_t  status_cb;

    uint32_t               *write_gpio_buf;  /* heap: GPIO words for write ISR */

    ReadRing_t              read_ring;

    osMutexId_t             api_mutex;
    osSemaphoreId_t         write_done_sem;
    osSemaphoreId_t         read_byte_sem;
    osEventFlagsId_t        rgap_flags;
    osTimerId_t             poll_timer;

    gptimer_handle_t        write_timer;
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t mcp_dev;

    volatile bool           abort_requested;
    volatile bool           eot_flag;
    volatile bool           ldp_flag;
} g;

#define RGAP_WENT_LOW   (1U << 0)
#define RGAP_WENT_HIGH  (1U << 1)

/* ============================================================================
 * GPIO helpers
 * ============================================================================ */

/* All command lines are active LOW (assert = drive to 0, deassert = drive to 1) */
static inline void cmd_assert(int gpio)
{
    gpio_set_level(gpio, 0);
}

static inline void cmd_deassert(int gpio)
{
    gpio_set_level(gpio, 1);
}

static void cmd_pulse(int gpio, uint32_t us)
{
    cmd_assert(gpio);
    esp_rom_delay_us(us);
    cmd_deassert(gpio);
}

/* Status lines on MCP23017 are active LOW; read returns inverted bit state */
static bool mcp_status_bit(uint8_t reg_val, int bit)
{
    return !(reg_val & (1U << bit));
}

/* ============================================================================
 * WARS one-shot pulse (3 µs, called from task context)
 * ============================================================================ */
static void wars_pulse(void)
{
    gpio_set_level(K9800_WARS_GPIO, 0);
    esp_rom_delay_us(K9800_WARS_PULSE_US);
    gpio_set_level(K9800_WARS_GPIO, 1);
}

/* ============================================================================
 * Write-bus GPIO word encoding (active LOW; 9 bits packed for GPIO_OUT)
 *
 * WD0-WD7 occupy GPIO 4-11 (shift left 4).
 * WDP occupies GPIO 12 (shift left 12).
 * Data bits and parity bit are inverted for active-LOW bus.
 * ============================================================================ */
static inline uint32_t data_to_gpio_word(uint8_t byte)
{
    uint32_t inv   = (~(uint32_t)byte) & 0xFFU;      /* WD0-WD7 inverted */
    uint32_t parity = (uint32_t)__builtin_parity(byte); /* WDP: 1 if odd parity */
    return (inv << K9800_WBus_SHIFT) | (parity << K9800_WDP_GPIO);
}

static uint8_t crcc_compute(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) { crc ^= data[i]; }
    return crc;
}

/* ============================================================================
 * MCP23017 read (I2C; returns raw GPA register byte, 0 on error)
 * ============================================================================ */
static uint8_t mcp_read_gpa(void)
{
    uint8_t reg = MCP23017_GPIOA;
    uint8_t val = 0xFF;  /* pull-up default = all deasserted */
    i2c_master_transmit_receive(g.mcp_dev, &reg, 1, &val, 1, 20);
    return val;
}

/* ============================================================================
 * Status cache update (called from poll timer callback and inline checks)
 * ============================================================================ */
static void k9800_update_status_cache(void)
{
    uint8_t gpa = mcp_read_gpa();
    K9800_TransportStatus_t s;
    s.online        = mcp_status_bit(gpa, MCP_BIT_ONL);
    s.rewinding     = mcp_status_bit(gpa, MCP_BIT_RWD);
    s.file_protect  = mcp_status_bit(gpa, MCP_BIT_FPT);
    s.at_load_point = mcp_status_bit(gpa, MCP_BIT_LDP);
    s.write_enable  = mcp_status_bit(gpa, MCP_BIT_WEK);
    s.ready         = mcp_status_bit(gpa, MCP_BIT_RDY);
    s.at_eot        = mcp_status_bit(gpa, MCP_BIT_EOT);
    s.tape_running  = mcp_status_bit(gpa, MCP_BIT_RNG);

    /* Track EOT/LDP transitions for in-progress operations */
    bool was_eot = g.status_cache.at_eot;
    bool was_ldp = g.status_cache.at_load_point;
    if (!was_eot && s.at_eot) { g.eot_flag = true; }
    if (was_ldp != s.at_load_point) { g.ldp_flag = s.at_load_point; }

    bool changed = (memcmp(&s, &g.status_cache, sizeof(s)) != 0);
    g.status_cache = s;
    if (changed && g.status_cb) { g.status_cb(&g.status_cache); }
}

static void poll_timer_cb(void *arg)
{
    (void)arg;
    k9800_update_status_cache();
}

/* ============================================================================
 * Write timer ISR — fires every 100 µs, clocks one byte to the write bus
 * ============================================================================ */
static bool IRAM_ATTR write_timer_isr(gptimer_handle_t timer,
                                      const gptimer_alarm_event_data_t *edata,
                                      void *user_ctx)
{
    BaseType_t woken = pdFALSE;

    if (s_wtx.idx >= s_wtx.len) {
        /* All bytes sent; stop alarm and signal task */
        s_wtx.done = true;
        xSemaphoreGiveFromISR((SemaphoreHandle_t)g.write_done_sem, &woken);
        return woken == pdTRUE;
    }

    /* Write 9-bit word (WD0-WD7 + WDP) to GPIO 4-12 in two atomic ops */
    uint32_t word = s_wtx.buf[s_wtx.idx++];
    GPIO.out_w1ts = word & K9800_WBus_MASK;
    GPIO.out_w1tc = (~word) & K9800_WBus_MASK;

    /* WDS strobe: assert low for 3 µs then deassert */
    GPIO.out_w1tc = K9800_WDS_MASK;
    esp_rom_delay_us(K9800_WARS_PULSE_US);   /* reuse 3 µs constant */
    GPIO.out_w1ts = K9800_WDS_MASK;

    return woken == pdTRUE;
}

/* ============================================================================
 * Read bus ISR — triggered on falling edge of RDS (GPIO47)
 * Samples GPIO38-45 (RD0-RD7) and GPIO46 (RDP) from GPIO.in1.val in one read.
 * ============================================================================ */
static void IRAM_ATTR rds_isr_handler(void *arg)
{
    /*
     * All read data lines are active LOW; GPIO.in1.val bit N represents GPIO(N+32).
     * RD0-RD7 = GPIO38-45 → bits 6-13 of in1.
     * RDP      = GPIO46   → bit 14 of in1.
     * Invert to recover logical levels.
     */
    uint32_t in1 = GPIO.in1.val;
    uint8_t data_byte  = (uint8_t)(~(in1 >> K9800_RBus_IN1_SHIFT) & 0xFFU);
    uint8_t parity_bit = (uint8_t)(~(in1 >> 14U) & 0x01U);

    if (__builtin_parity(data_byte) != parity_bit) {
        g.read_ring.parity_err = true;
    }

    uint32_t next = (g.read_ring.head + 1U) & READ_RING_MASK;
    if (next == g.read_ring.tail) {
        g.read_ring.overrun = true;
        return;
    }
    g.read_ring.buf[g.read_ring.head] = data_byte;
    g.read_ring.head = next;

    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)g.read_byte_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

/* RGAP ISR — both edges; signals gap start (LOW) and gap end (HIGH) */
static void IRAM_ATTR rgap_isr_handler(void *arg)
{
    BaseType_t woken = pdFALSE;
    /* RGAP is active LOW: pin LOW → in gap, pin HIGH → data block */
    uint32_t in1 = GPIO.in1.val;
    uint32_t bit = (in1 & K9800_RGAP_IN1_BIT) ? RGAP_WENT_HIGH : RGAP_WENT_LOW;
    xEventGroupSetBitsFromISR((EventGroupHandle_t)g.rgap_flags, bit, &woken);
    portYIELD_FROM_ISR(woken);
}

/* ============================================================================
 * Hardware initialisation
 * ============================================================================ */
static K9800_Error_t hw_gpio_init(void)
{
    /* ── Write bus outputs (GPIO 4-13) ──────────────────────────────────── */
    gpio_config_t oc = {
        .pin_bit_mask = (0xFFU << 4) | (1U << 12) | (1U << 13),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&oc);
    /* Deassert write bus (active LOW → drive HIGH) */
    GPIO.out_w1ts = K9800_WBus_MASK | K9800_WDS_MASK;

    /* ── Command outputs ─────────────────────────────────────────────────── */
    const int cmd_pins[] = {
        K9800_WARS_GPIO, K9800_SFC_GPIO, K9800_SRC_GPIO, K9800_RWC_GPIO,
        K9800_SLT_GPIO,  K9800_SWS_GPIO, K9800_OFFC_GPIO
    };
    for (int i = 0; i < (int)(sizeof(cmd_pins)/sizeof(*cmd_pins)); i++) {
        gpio_config_t c = {
            .pin_bit_mask = 1ULL << cmd_pins[i],
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&c);
        gpio_set_level(cmd_pins[i], 1);   /* deassert */
    }

    /* ── Read bus inputs (GPIO 38-46) + RDS (47) + RGAP (48) ────────────── */
    gpio_config_t ic = {
        .pin_bit_mask = (0x1FFULL << 38) | (1ULL << 47) | (1ULL << 48),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&ic);

    /* RDS: falling edge interrupt */
    gpio_set_intr_type(K9800_RDS_GPIO, GPIO_INTR_NEGEDGE);
    /* RGAP: both edges */
    gpio_set_intr_type(K9800_RGAP_GPIO, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(K9800_RDS_GPIO,  rds_isr_handler,  NULL);
    gpio_isr_handler_add(K9800_RGAP_GPIO, rgap_isr_handler, NULL);

    return K9800_OK;
}

static K9800_Error_t hw_i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = MCP23017_I2C_PORT,
        .scl_io_num        = K9800_I2C_SCL_GPIO,
        .sda_io_num        = K9800_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,  /* external pull-ups on PCB */
    };
    if (i2c_new_master_bus(&bus_cfg, &g.i2c_bus) != ESP_OK) {
        return K9800_ERR_I2C;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MCP23017_I2C_ADDR,
        .scl_speed_hz    = MCP23017_I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(g.i2c_bus, &dev_cfg, &g.mcp_dev) != ESP_OK) {
        return K9800_ERR_I2C;
    }

    /* Configure all GPA pins as inputs with pull-ups */
    uint8_t cfg[2] = { MCP23017_IODIRA, 0xFF };   /* all inputs */
    i2c_master_transmit(g.mcp_dev, cfg, 2, 20);
    cfg[0] = MCP23017_GPPUA;
    i2c_master_transmit(g.mcp_dev, cfg, 2, 20);   /* enable pull-ups */

    return K9800_OK;
}

static K9800_Error_t hw_write_timer_init(void)
{
    gptimer_config_t timer_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   /* 1 µs resolution */
    };
    if (gptimer_new_timer(&timer_cfg, &g.write_timer) != ESP_OK) {
        return K9800_ERR_DMA;
    }

    gptimer_event_callbacks_t cbs = { .on_alarm = write_timer_isr };
    gptimer_register_event_callbacks(g.write_timer, &cbs, NULL);
    gptimer_enable(g.write_timer);
    return K9800_OK;
}

/* ============================================================================
 * Write path helpers
 * ============================================================================ */
static K9800_Error_t write_prepare_buf(const uint8_t *data, uint32_t len)
{
    uint32_t total = len + 1U;  /* +1 for CRCC */
    if (g.write_gpio_buf) { free(g.write_gpio_buf); g.write_gpio_buf = NULL; }
    g.write_gpio_buf = malloc(total * sizeof(uint32_t));
    if (!g.write_gpio_buf) { return K9800_ERR_PARAM; }

    for (uint32_t i = 0; i < len; i++) {
        g.write_gpio_buf[i] = data_to_gpio_word(data[i]);
    }
    g.write_gpio_buf[len] = data_to_gpio_word(crcc_compute(data, len));

    s_wtx.buf  = g.write_gpio_buf;
    s_wtx.len  = total;
    s_wtx.idx  = 0;
    s_wtx.done = false;
    s_wtx.err  = false;
    return K9800_OK;
}

static K9800_Error_t write_start_timer(void)
{
    gptimer_alarm_config_t alarm = {
        .alarm_count               = K9800_CHAR_PERIOD_US,
        .reload_count              = 0,
        .flags.auto_reload_on_alarm = true,
    };
    if (gptimer_set_alarm_action(g.write_timer, &alarm) != ESP_OK) {
        return K9800_ERR_DMA;
    }
    gptimer_set_raw_count(g.write_timer, 0);
    gptimer_start(g.write_timer);
    return K9800_OK;
}

static void write_stop_timer(void)
{
    gptimer_stop(g.write_timer);
    /* Deassert write bus */
    GPIO.out_w1ts = K9800_WBus_MASK | K9800_WDS_MASK;
}

/* ============================================================================
 * Internal block operations
 * ============================================================================ */
static K9800_Error_t write_block_internal(const uint8_t *data, uint32_t len)
{
    k9800_update_status_cache();
    if (!g.status_cache.write_enable) return K9800_ERR_WRITE_PROTECT;
    if (!g.status_cache.ready)        return K9800_ERR_NOT_READY;

    K9800_Error_t ret = write_prepare_buf(data, len);
    if (ret != K9800_OK) return ret;

    cmd_assert(K9800_SLT_GPIO);
    cmd_assert(K9800_SWS_GPIO);
    esp_rom_delay_us(5);
    cmd_assert(K9800_SFC_GPIO);
    vTaskDelay(pdMS_TO_TICKS(K9800_WRITE_START_DELAY_MS));

    if (g.abort_requested || g.eot_flag) {
        cmd_deassert(K9800_SFC_GPIO);
        cmd_deassert(K9800_SWS_GPIO);
        cmd_deassert(K9800_SLT_GPIO);
        ret = g.abort_requested ? K9800_ERR_ABORTED : K9800_ERR_EOT;
        goto cleanup;
    }

    g.state = K9800_ST_WRITING;
    ret = write_start_timer();
    if (ret != K9800_OK) {
        cmd_deassert(K9800_SFC_GPIO);
        cmd_deassert(K9800_SWS_GPIO);
        cmd_deassert(K9800_SLT_GPIO);
        goto cleanup;
    }

    uint32_t timeout_ms = (len * K9800_CHAR_PERIOD_US / 1000U)
                        + K9800_WRITE_STOP_DELAY_MS + 200U;
    osStatus_t stat = osSemaphoreAcquire(g.write_done_sem,
                                         (uint32_t)timeout_ms);
    write_stop_timer();

    if (stat  != osOK)       { ret = K9800_ERR_TIMEOUT;  goto deassert; }
    if (s_wtx.err)           { ret = K9800_ERR_DMA;       goto deassert; }
    if (g.abort_requested)   { ret = K9800_ERR_ABORTED;   goto deassert; }

    vTaskDelay(pdMS_TO_TICKS(K9800_WRITE_STOP_DELAY_MS));
    esp_rom_delay_us(K9800_WARS_DELAY_US);
    wars_pulse();
    ret = K9800_OK;

deassert:
    cmd_deassert(K9800_SFC_GPIO);
    cmd_deassert(K9800_SWS_GPIO);
    {
        uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(K9800_RAMP_TIME_MS + 50U);
        while (xTaskGetTickCount() < deadline) {
            k9800_update_status_cache();
            if (!g.status_cache.tape_running) break;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    cmd_deassert(K9800_SLT_GPIO);

cleanup:
    g.state = K9800_ST_IDLE;
    if (g.write_gpio_buf) { free(g.write_gpio_buf); g.write_gpio_buf = NULL; }
    return ret;
}

static K9800_Error_t read_block_internal(uint8_t *buf, uint32_t buf_len,
                                          uint32_t *out_len)
{
    if (!buf || !buf_len || !out_len) return K9800_ERR_PARAM;
    k9800_update_status_cache();
    if (!g.status_cache.ready) return K9800_ERR_NOT_READY;

    g.read_ring.head = g.read_ring.tail = 0;
    g.read_ring.parity_err = g.read_ring.overrun = false;
    g.eot_flag = false;
    *out_len = 0;

    while (osSemaphoreAcquire(g.read_byte_sem, 0) == osOK) {}
    osEventFlagsClear(g.rgap_flags, RGAP_WENT_LOW | RGAP_WENT_HIGH);

    cmd_assert(K9800_SLT_GPIO);
    cmd_deassert(K9800_SWS_GPIO);
    esp_rom_delay_us(5);

    g.state = K9800_ST_READING;
    cmd_assert(K9800_SFC_GPIO);

    uint32_t flags = osEventFlagsWait(g.rgap_flags, RGAP_WENT_LOW,
                                      osFlagsWaitAny, K9800_READY_TIMEOUT_MS);
    if (flags & osFlagsError) {
        cmd_deassert(K9800_SFC_GPIO);
        cmd_deassert(K9800_SLT_GPIO);
        g.state = K9800_ST_IDLE;
        return K9800_ERR_TIMEOUT;
    }

    bool block_done = false;
    K9800_Error_t ret = K9800_OK;
    while (!block_done) {
        /* Non-blocking check for end-of-block */
        uint32_t ev = osEventFlagsWait(g.rgap_flags, RGAP_WENT_HIGH,
                                       osFlagsWaitAny | osFlagsNoClear, 1U);

        /* Drain ring buffer */
        uint32_t head = g.read_ring.head, tail = g.read_ring.tail;
        while (tail != head) {
            uint8_t b = g.read_ring.buf[tail & READ_RING_MASK];
            tail++;
            g.read_ring.tail = tail;
            if (*out_len < buf_len) buf[(*out_len)++] = b;
            else                    g.read_ring.overrun = true;
            head = g.read_ring.head;
        }

        if (g.abort_requested)   { ret = K9800_ERR_ABORTED; block_done = true; }
        else if (g.eot_flag)     { ret = K9800_ERR_EOT;     block_done = true; }
        else if (ev & RGAP_WENT_HIGH) {
            /* Drain any bytes that arrived during the RGAP transition */
            vTaskDelay(pdMS_TO_TICKS(1));
            head = g.read_ring.head; tail = g.read_ring.tail;
            while (tail != head) {
                uint8_t b = g.read_ring.buf[tail & READ_RING_MASK];
                tail++;
                g.read_ring.tail = tail;
                if (*out_len < buf_len) buf[(*out_len)++] = b;
                else                    g.read_ring.overrun = true;
                head = g.read_ring.head;
            }
            block_done = true;
        }
    }

    cmd_deassert(K9800_SFC_GPIO);
    vTaskDelay(pdMS_TO_TICKS(K9800_READ_STOP_DELAY_MS));
    cmd_deassert(K9800_SLT_GPIO);
    {
        uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(K9800_RAMP_TIME_MS + 50U);
        while (xTaskGetTickCount() < deadline) {
            k9800_update_status_cache();
            if (!g.status_cache.tape_running) break;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    g.state = K9800_ST_IDLE;

    if (g.read_ring.overrun)    return K9800_ERR_OVERRUN;
    if (g.read_ring.parity_err) return K9800_ERR_PARITY;
    if (ret != K9800_OK)        return ret;

    if (*out_len >= 1U) {
        uint32_t dl  = *out_len - 1U;
        uint8_t  exp = crcc_compute(buf, dl), got = buf[dl];
        *out_len = dl;
        if (exp != got) return K9800_ERR_PARITY;
    }
    return K9800_OK;
}

/* ============================================================================
 * Public API
 * ============================================================================ */
K9800_Error_t K9800_Init(const K9800_Config_t *cfg)
{
    if (g.state != K9800_ST_UNINIT) return K9800_OK;
    g.cfg = cfg ? *cfg : (K9800_Config_t)K9800_DEFAULT_CONFIG;

    K9800_Error_t ret;
    if ((ret = hw_gpio_init())         != K9800_OK) return ret;
    if ((ret = hw_i2c_init())          != K9800_OK) return ret;
    if ((ret = hw_write_timer_init())  != K9800_OK) return ret;

    static const osMutexAttr_t      mx  = { "K9800Mtx",  osMutexRecursive|osMutexPrioInherit, NULL, 0 };
    static const osSemaphoreAttr_t  ws  = { "K9800WDone", 0, NULL, 0 };
    static const osSemaphoreAttr_t  rs  = { "K9800RByte", 0, NULL, 0 };
    static const osEventFlagsAttr_t rg  = { "K9800Rgap",  0, NULL, 0 };
    static const osTimerAttr_t      tmr = { "K9800Poll",  0, NULL, 0 };

    if (!(g.api_mutex     = osMutexNew(&mx)))                               return K9800_ERR_NOT_INIT;
    if (!(g.write_done_sem= osSemaphoreNew(1, 0, &ws)))                    return K9800_ERR_NOT_INIT;
    if (!(g.read_byte_sem = osSemaphoreNew(K9800_READ_RING_SIZE, 0, &rs))) return K9800_ERR_NOT_INIT;
    if (!(g.rgap_flags    = osEventFlagsNew(&rg)))                          return K9800_ERR_NOT_INIT;
    if (!(g.poll_timer    = osTimerNew(poll_timer_cb, osTimerPeriodic, NULL, &tmr))) return K9800_ERR_NOT_INIT;

    osTimerStart(g.poll_timer, 20U);
    k9800_update_status_cache();
    g.state = K9800_ST_IDLE;
    return K9800_OK;
}

void K9800_DeInit(void)
{
    if (g.state == K9800_ST_UNINIT) return;
    K9800_Abort();
    osTimerStop(g.poll_timer);
    osTimerDelete(g.poll_timer);

    gptimer_stop(g.write_timer);
    gptimer_disable(g.write_timer);
    gptimer_del_timer(g.write_timer);

    gpio_isr_handler_remove(K9800_RDS_GPIO);
    gpio_isr_handler_remove(K9800_RGAP_GPIO);

    i2c_master_bus_rm_device(g.mcp_dev);
    i2c_del_master_bus(g.i2c_bus);

    /* Deassert all command outputs */
    const int cmd_pins[] = {
        K9800_SFC_GPIO, K9800_SRC_GPIO, K9800_RWC_GPIO, K9800_SLT_GPIO,
        K9800_SWS_GPIO, K9800_OFFC_GPIO, K9800_WARS_GPIO
    };
    for (int i = 0; i < (int)(sizeof(cmd_pins)/sizeof(*cmd_pins)); i++) {
        gpio_set_level(cmd_pins[i], 1);
    }

    if (g.write_gpio_buf) { free(g.write_gpio_buf); g.write_gpio_buf = NULL; }
    osMutexDelete(g.api_mutex);
    osSemaphoreDelete(g.write_done_sem);
    osSemaphoreDelete(g.read_byte_sem);
    osEventFlagsDelete(g.rgap_flags);

    memset(&g, 0, sizeof(g));
    g.state = K9800_ST_UNINIT;
}

K9800_Error_t K9800_GetStatus(K9800_TransportStatus_t *s)
{
    if (!s)                         return K9800_ERR_PARAM;
    if (g.state == K9800_ST_UNINIT) return K9800_ERR_NOT_INIT;
    k9800_update_status_cache();
    taskENTER_CRITICAL();
    *s = g.status_cache;
    taskEXIT_CRITICAL();
    return K9800_OK;
}

K9800_Error_t K9800_WaitReady(uint32_t timeout_ms)
{
    if (g.state == K9800_ST_UNINIT) return K9800_ERR_NOT_INIT;
    uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        k9800_update_status_cache();
        if (g.status_cache.ready) return K9800_OK;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return K9800_ERR_TIMEOUT;
}

K9800_Error_t K9800_Select(bool select)
{
    if (g.state == K9800_ST_UNINIT) return K9800_ERR_NOT_INIT;
    select ? cmd_assert(K9800_SLT_GPIO) : cmd_deassert(K9800_SLT_GPIO);
    return K9800_OK;
}

K9800_Error_t K9800_WriteBlock(const uint8_t *data, uint32_t len)
{
    if (g.state == K9800_ST_UNINIT)               return K9800_ERR_NOT_INIT;
    if (!data || !len || len > K9800_MAX_BLOCK_BYTES) return K9800_ERR_PARAM;
    if (osMutexAcquire(g.api_mutex, K9800_READY_TIMEOUT_MS) != osOK) return K9800_ERR_BUSY;
    g.abort_requested = false;
    K9800_Error_t ret = write_block_internal(data, len);
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_ReadBlock(uint8_t *buf, uint32_t buf_len, uint32_t *out_len)
{
    if (g.state == K9800_ST_UNINIT)    return K9800_ERR_NOT_INIT;
    if (!buf || !buf_len || !out_len)  return K9800_ERR_PARAM;
    if (osMutexAcquire(g.api_mutex, K9800_READY_TIMEOUT_MS) != osOK) return K9800_ERR_BUSY;
    g.abort_requested = false;
    K9800_Error_t ret = read_block_internal(buf, buf_len, out_len);
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_Rewind(uint32_t timeout_ms)
{
    if (g.state == K9800_ST_UNINIT) return K9800_ERR_NOT_INIT;
    if (!timeout_ms) timeout_ms = K9800_REWIND_TIMEOUT_MS;
    if (osMutexAcquire(g.api_mutex, 1000U) != osOK) return K9800_ERR_BUSY;

    k9800_update_status_cache();
    if (g.status_cache.at_load_point) { osMutexRelease(g.api_mutex); return K9800_OK; }

    g.state = K9800_ST_REWINDING;
    cmd_pulse(K9800_RWC_GPIO, K9800_RWC_PULSE_US);

    bool rwd_seen = false;
    K9800_Error_t ret = K9800_ERR_TIMEOUT;
    uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        k9800_update_status_cache();
        if (!rwd_seen && g.status_cache.rewinding)  rwd_seen = true;
        if (rwd_seen  && !g.status_cache.rewinding) { ret = K9800_OK; break; }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    g.state = K9800_ST_IDLE;
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_SpaceForward(uint32_t duration_ms)
{
    if (g.state == K9800_ST_UNINIT) return K9800_ERR_NOT_INIT;
    if (osMutexAcquire(g.api_mutex, 1000U) != osOK) return K9800_ERR_BUSY;
    g.state = K9800_ST_SPACING;
    g.eot_flag = false;
    cmd_assert(K9800_SLT_GPIO);
    cmd_assert(K9800_SFC_GPIO);

    K9800_Error_t ret = K9800_OK;
    uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
    while (xTaskGetTickCount() < deadline) {
        if (g.eot_flag)        { ret = K9800_ERR_EOT;    break; }
        if (g.abort_requested) { ret = K9800_ERR_ABORTED; break; }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    cmd_deassert(K9800_SFC_GPIO);
    vTaskDelay(pdMS_TO_TICKS(K9800_RAMP_TIME_MS));
    cmd_deassert(K9800_SLT_GPIO);
    g.state = K9800_ST_IDLE;
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_SpaceReverse(uint32_t duration_ms)
{
    if (g.state == K9800_ST_UNINIT) return K9800_ERR_NOT_INIT;
    if (osMutexAcquire(g.api_mutex, 1000U) != osOK) return K9800_ERR_BUSY;
    g.state = K9800_ST_SPACING;
    cmd_assert(K9800_SLT_GPIO);
    cmd_assert(K9800_SRC_GPIO);

    K9800_Error_t ret = K9800_OK;
    uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
    while (xTaskGetTickCount() < deadline) {
        k9800_update_status_cache();
        if (g.status_cache.at_load_point) { ret = K9800_ERR_BOT;    break; }
        if (g.abort_requested)             { ret = K9800_ERR_ABORTED; break; }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    cmd_deassert(K9800_SRC_GPIO);
    vTaskDelay(pdMS_TO_TICKS(K9800_RAMP_TIME_MS));
    cmd_deassert(K9800_SLT_GPIO);
    g.state = K9800_ST_IDLE;
    osMutexRelease(g.api_mutex);
    return ret;
}

K9800_Error_t K9800_GoOffline(void)
{
    if (g.state == K9800_ST_UNINIT) return K9800_ERR_NOT_INIT;
    cmd_pulse(K9800_OFFC_GPIO, K9800_OFFC_PULSE_US);
    return K9800_OK;
}

K9800_Error_t K9800_Abort(void)
{
    if (g.state == K9800_ST_UNINIT) return K9800_ERR_NOT_INIT;
    g.abort_requested = true;
    cmd_deassert(K9800_SFC_GPIO);
    cmd_deassert(K9800_SRC_GPIO);
    cmd_deassert(K9800_SWS_GPIO);
    write_stop_timer();
    vTaskDelay(pdMS_TO_TICKS(K9800_RAMP_TIME_MS + 10U));
    cmd_deassert(K9800_SLT_GPIO);
    return K9800_OK;
}

void K9800_RegisterStatusCallback(K9800_StatusCallback_t cb)
{
    g.status_cb = cb;
}
