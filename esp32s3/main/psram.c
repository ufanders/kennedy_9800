#include "psram.h"
#include "app_config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "psram";

static uint8_t *s_fat_meta;
static uint8_t *s_ltfs_idx;
static uint8_t *s_staging;

esp_err_t psram_init(void)
{
    s_fat_meta = heap_caps_calloc(1, PSRAM_FAT_SIZE,    MALLOC_CAP_SPIRAM);
    s_ltfs_idx = heap_caps_calloc(1, PSRAM_LTFS_SIZE,   MALLOC_CAP_SPIRAM);
    s_staging  = heap_caps_calloc(1, PSRAM_STAGING_SIZE, MALLOC_CAP_SPIRAM);

    if (!s_fat_meta || !s_ltfs_idx || !s_staging) {
        ESP_LOGE(TAG, "PSRAM allocation failed (fat=%p ltfs=%p staging=%p)",
                 s_fat_meta, s_ltfs_idx, s_staging);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "PSRAM regions: fat_meta=%p (%u KB), ltfs_idx=%p (%u KB), "
             "staging=%p (%u B)",
             s_fat_meta, (unsigned)(PSRAM_FAT_SIZE  / 1024),
             s_ltfs_idx, (unsigned)(PSRAM_LTFS_SIZE / 1024),
             s_staging,  (unsigned) PSRAM_STAGING_SIZE);
    return ESP_OK;
}

uint8_t *psram_fat_meta(void)         { return s_fat_meta; }
uint8_t *psram_ltfs_idx(void)         { return s_ltfs_idx; }
uint8_t *psram_staging(void)          { return s_staging;  }
size_t   psram_fat_meta_size(void)    { return PSRAM_FAT_SIZE;    }
size_t   psram_ltfs_idx_size(void)    { return PSRAM_LTFS_SIZE;   }
size_t   psram_staging_size(void)     { return PSRAM_STAGING_SIZE; }
