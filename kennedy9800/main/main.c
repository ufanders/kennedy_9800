/**
 * @file  main.c
 * @brief Application entry point — Kennedy 9800 USB/LTFS bridge on ESP32-S3.
 *
 * Boot sequence:
 *   1. psram_init()            — allocate PSRAM regions (fat, ltfs, staging)
 *   2. K9800_Init()            — GPIO, I2C expander, write timer, RTOS objects
 *   3. fat32_meta_init()       — point FAT layer at PSRAM region
 *   4. disk_io_init()          — set up routing state
 *   5. K9800_WaitReady()       — block until tape loaded and transport ready
 *   6. ltfs_init()             — point LTFS at PSRAM ltfs region
 *   7. ltfs_mount()            — read tape blocks 0+1 into PSRAM index
 *       on ESP_OK             → fat32_meta_rebuild_from_ltfs()
 *       on ESP_ERR_NOT_FOUND  → fat32_meta_build() + ltfs_format() (blank tape)
 *   8. usb_msc_init()          — install TinyUSB, start USB task
 *   9. disk_io_set_ready(true) — allow host to access volume
 *  10. tape_sync_task          — background LTFS flush / status monitor
 */

#include "kennedy9800.h"
#include "psram.h"
#include "ltfs.h"
#include "fat32_meta.h"
#include "disk_io.h"
#include "usb_msc.h"
#include "app_config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_psram.h"

static const char *TAG = "main";

/* ── Tape sync / status monitor task ────────────────────────────────────── */

static void tape_sync_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "tape sync task started");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));   /* check every 5 s */

        if (disk_io_ready()) {
            /* Bound how long writes can sit only in the PSRAM block cache
             * before being committed to tape. */
            disk_io_flush();
        }

        K9800_TransportStatus_t st;
        if (K9800_GetStatus(&st) != K9800_OK) continue;

        if (!st.online) {
            /* Tape went offline — inform host */
            if (disk_io_ready()) {
                disk_io_set_ready(false);
                ESP_LOGW(TAG, "tape offline — volume unmounted from host");
            }
            continue;
        }

        if (st.online && st.ready && !disk_io_ready()) {
            /* Tape came back online — attempt re-mount */
            ESP_LOGI(TAG, "tape online again — attempting re-mount");
            esp_err_t err = ltfs_mount();
            if (err == ESP_OK) {
                fat32_meta_rebuild_from_ltfs();
                disk_io_set_ready(true);
                ESP_LOGI(TAG, "re-mount succeeded");
            } else {
                ESP_LOGW(TAG, "re-mount failed (err=%d), tape may be blank", err);
            }
        }
    }
}

/* ── app_main ────────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "Kennedy 9800 USB/LTFS bridge — ESP32-S3-N16R8");
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    /* ── 1. PSRAM ──────────────────────────────────────────────────────── */
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM not detected — check hardware and sdkconfig");
        /* Halt; cannot operate without PSRAM for FAT/index cache. */
        for (;;) vTaskDelay(portMAX_DELAY);
    }
    ESP_ERROR_CHECK(psram_init());

    /* ── 2. Transport driver ───────────────────────────────────────────── */
    const K9800_Config_t k9800_cfg = K9800_DEFAULT_CONFIG;
    K9800_Error_t kerr = K9800_Init(&k9800_cfg);
    if (kerr != K9800_OK) {
        ESP_LOGE(TAG, "K9800_Init failed: %d", kerr);
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    /* ── 3-4. FAT32 and disk_io layer ─────────────────────────────────── */
    ESP_ERROR_CHECK(fat32_meta_init());
    ESP_ERROR_CHECK(disk_io_init());

    /* ── 5. Wait for tape ─────────────────────────────────────────────── */
    ESP_LOGI(TAG, "waiting for tape transport ready...");
    kerr = K9800_WaitReady(K9800_READY_TIMEOUT_MS);
    if (kerr != K9800_OK) {
        ESP_LOGW(TAG, "transport not ready (err=%d) — starting in offline mode", kerr);
        /* Continue anyway: USB will appear with no media inserted */
    }

    /* ── 6-7. LTFS mount ──────────────────────────────────────────────── */
    ESP_ERROR_CHECK(ltfs_init());

    bool tape_ready = (kerr == K9800_OK);
    if (tape_ready) {
        esp_err_t merr = ltfs_mount();
        if (merr == ESP_OK) {
            ESP_LOGI(TAG, "LTFS mounted successfully");
            fat32_meta_rebuild_from_ltfs();
            disk_io_set_ready(true);
        } else if (merr == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "blank tape — formatting with label \"Kennedy 9800\"");
            fat32_meta_build("Kennedy 9800");
            ltfs_format("Kennedy 9800");
            disk_io_set_ready(true);
        } else {
            ESP_LOGE(TAG, "LTFS mount error %d — volume offline", merr);
            fat32_meta_build("Kennedy 9800");
            /* disk_io remains not-ready */
        }
    } else {
        /* No tape loaded; stamp an empty FAT so USB descriptors are valid */
        fat32_meta_build("Kennedy 9800");
    }

    /* ── 8. USB MSC ───────────────────────────────────────────────────── */
    ESP_ERROR_CHECK(usb_msc_init());

    /* ── 9. Background task ───────────────────────────────────────────── */
    xTaskCreate(tape_sync_task, "tape_sync", 8192, NULL, PRI_SYNC, NULL);

    ESP_LOGI(TAG, "init complete — USB MSC active");
    /* app_main returns; the USB task and tape_sync_task keep the firmware running */
}
