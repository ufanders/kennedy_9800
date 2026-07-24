/**
 * @file  usb_msc.c
 * @brief TinyUSB MSC device callbacks + USB task for Kennedy 9800 USB bridge.
 *
 * The host OS sees a single removable FAT32 disk.  All MSC callbacks delegate
 * to disk_io_read() / disk_io_write() which route by LBA:
 *   Metadata LBAs  (<FAT32_DATA_LBA)  → PSRAM (fast, no tape)
 *   Data LBAs      (≥FAT32_DATA_LBA)  → tape via K9800_ReadBlock/WriteBlock
 *
 * USB descriptors describe a single Bulk-Only Transport MSC interface with
 * a vendor/product ID suitable for a "Tape Archive Drive" class device.
 */

#include "usb_msc.h"
#include "disk_io.h"
#include "kennedy9800.h"
#include "app_config.h"

#include "tusb.h"
#include "class/msc/msc_device.h"
#include "esp_private/usb_phy.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "usb_msc";

/* ── USB descriptor strings ──────────────────────────────────────────────── */
#define USB_VID     0x1234U   /* Replace with your licensed VID */
#define USB_PID     0x5678U   /* Replace with appropriate PID   */

static const tusb_desc_device_t s_desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

#define EPNUM_MSC_OUT   0x01
#define EPNUM_MSC_IN    0x81

/* Total config descriptor length: config(9) + interface(9) + 2×endpoint(7) */
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

static const uint8_t s_desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_MSC_DESCRIPTOR(0, 4, EPNUM_MSC_OUT, EPNUM_MSC_IN,
                       CFG_TUD_MSC_EP_BUFSIZE),
};

static const char *s_strings[] = {
    (const char[]){ 0x09, 0x04 },   /* 0: supported language = English */
    "Kennedy Tape Co.",              /* 1: manufacturer */
    "Kennedy 9800 USB Bridge",       /* 2: product */
    "K9800-0001",                    /* 3: serial number */
    "Kennedy 9800 Tape",             /* 4: MSC interface string */
};

/* ── TinyUSB descriptor callbacks ────────────────────────────────────────── */

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&s_desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return s_desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t desc_str[64];
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&desc_str[1], s_strings[0], 2);
        chr_count = 1;
    } else if (index < sizeof(s_strings) / sizeof(s_strings[0])) {
        const char *str = s_strings[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 63) chr_count = 63;
        for (uint8_t i = 0; i < chr_count; i++) {
            desc_str[1 + i] = str[i];
        }
    } else {
        return NULL;
    }

    desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return desc_str;
}

/* ── TinyUSB MSC callbacks ───────────────────────────────────────────────── */

/* Invoked when host requests SCSI INQUIRY */
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                         uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    memcpy(vendor_id,   "Kennedy ", 8);
    memcpy(product_id,  "9800 Tape Drive ", 16);
    memcpy(product_rev, "1.00", 4);
}

/* Invoked when host checks if device is ready (TEST UNIT READY) */
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    if (!disk_io_ready()) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY,
                          0x3A, 0x00);  /* Medium not present */
        return false;
    }
    return true;
}

/* Invoked when host requests medium capacity */
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = disk_io_sector_count();
    *block_size  = (uint16_t)disk_io_sector_size();
}

/* Invoked when host issues START/STOP UNIT */
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                             bool start, bool load_eject)
{
    (void)lun; (void)power_condition;
    if (load_eject) {
        if (!start) {
            /* Host requested ejection — flush pending writes, then go offline */
            disk_io_flush();
            K9800_GoOffline();
            disk_io_set_ready(false);
            ESP_LOGI(TAG, "host ejected volume");
        }
    }
    return true;
}

/* Invoked for every READ(10) command */
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                            void *buffer, uint32_t bufsize)
{
    (void)lun; (void)offset;
    uint32_t sectors = bufsize / DISK_SECTOR_SIZE;
    if (disk_io_read(lba, (uint8_t *)buffer, sectors) != ESP_OK) {
        return -1;
    }
    return (int32_t)bufsize;
}

/* Invoked for every WRITE(10) command */
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                             uint8_t *buffer, uint32_t bufsize)
{
    (void)lun; (void)offset;
    uint32_t sectors = bufsize / DISK_SECTOR_SIZE;
    if (disk_io_write(lba, buffer, sectors) != ESP_OK) {
        return -1;
    }
    return (int32_t)bufsize;
}

/* Invoked for SCSI commands not natively handled by TinyUSB's MSC layer */
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                          void *buffer, uint16_t bufsize)
{
    (void)buffer; (void)bufsize;

    if (scsi_cmd[0] == 0x35) {   /* SYNCHRONIZE CACHE (10) */
        if (disk_io_flush() != ESP_OK) {
            tud_msc_set_sense(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00);
            return -1;
        }
        return 0;
    }

    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST,
                       0x20, 0x00);  /* Invalid command */
    return -1;
}

/* Write protect: allow writes if tape has write ring */
bool tud_msc_is_writable_cb(uint8_t lun)
{
    (void)lun;
    K9800_TransportStatus_t st;
    if (K9800_GetStatus(&st) != K9800_OK) return false;
    return st.write_enable;
}

/* ── USB task ────────────────────────────────────────────────────────────── */

static void usb_device_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "USB MSC task started");
    while (1) {
        tud_task();   /* TinyUSB device task — must be called from a single task */
    }
}

/* ── USB init ─────────────────────────────────────────────────────────────
 * Raw TinyUSB core, not espressif/esp_tinyusb: that wrapper's MSC support
 * (>=2.0) only backs a storage instance with SPI-flash wear-levelling or an
 * SD/MMC card — no generic block-device callback API — so it can't host our
 * tape+PSRAM virtual disk. We bring up the USB PHY ourselves the same way
 * esp_tinyusb does internally, then drive tud_rhport_init()/tud_task() and
 * the tud_msc_ and tud_descriptor_ callbacks above directly.
 * ──────────────────────────────────────────────────────────────────────── */

esp_err_t usb_msc_init(void)
{
    const usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .target     = USB_PHY_TARGET_INT,
        .otg_mode   = USB_OTG_MODE_DEVICE,
        .otg_speed  = USB_PHY_SPEED_FULL,
    };
    usb_phy_handle_t phy_hdl;
    esp_err_t err = usb_new_phy(&phy_conf, &phy_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB PHY init failed: %s", esp_err_to_name(err));
        return err;
    }

    const tusb_rhport_init_t rh_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    if (!tud_rhport_init(0, &rh_init)) {
        ESP_LOGE(TAG, "TinyUSB device stack init failed");
        return ESP_FAIL;
    }

    xTaskCreate(usb_device_task, "usb_msc", 4096, NULL, PRI_USB, NULL);
    ESP_LOGI(TAG, "USB MSC initialised");
    return ESP_OK;
}
