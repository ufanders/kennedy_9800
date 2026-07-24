#include "fat32_meta.h"
#include "psram.h"
#include "ltfs.h"
#include "app_config.h"
#include "esp_log.h"
#include <string.h>
#include <time.h>

static const char *TAG = "fat32";

/* ── PSRAM layout within the fat_meta region ─────────────────────────────
 *  We treat the entire PSRAM fat region as a flat sector array.
 *  Sector N is at fat_meta_ptr + N * DISK_SECTOR_SIZE.
 *  Only sectors 0 .. FAT32_DATA_LBA+8-1 are stored (root cluster included).
 * ─────────────────────────────────────────────────────────────────────── */
static uint8_t *s_meta;   /* base pointer into PSRAM fat region */

/* FAT1 base and FAT2 base offsets (in bytes from s_meta) */
#define FAT1_OFFSET  ((FAT32_RESERVED_SECS) * DISK_SECTOR_SIZE)
#define FAT2_OFFSET  ((FAT32_RESERVED_SECS + FAT32_FAT_SECS) * DISK_SECTOR_SIZE)
#define ROOT_OFFSET  (FAT32_DATA_LBA * DISK_SECTOR_SIZE)

/* Bytes needed to store the full metadata region */
#define META_SECTORS  (FAT32_DATA_LBA + FAT32_CLUSTER_SECS)   /* include root cluster */
#define META_BYTES    (META_SECTORS * DISK_SECTOR_SIZE)

/* ── FAT32 BPB helpers ───────────────────────────────────────────────────── */
static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xFF; p[1] = v >> 8;
}
static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF; p[3] = v >> 24;
}
static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) |
           ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}

/* Write a FAT32 entry into both FAT copies */
static void fat_set(uint32_t cluster, uint32_t value)
{
    uint32_t off = cluster * 4U;
    put_le32(s_meta + FAT1_OFFSET + off, value & 0x0FFFFFFFU);
    put_le32(s_meta + FAT2_OFFSET + off, value & 0x0FFFFFFFU);
}

/* DOS time encoding (seconds since epoch → DOS date/time words) */
static void unix_to_dos(time_t t, uint16_t *date, uint16_t *time_w)
{
    struct tm tm;
    gmtime_r(&t, &tm);
    if (date)   *date   = (uint16_t)(((tm.tm_year - 80) << 9) | ((tm.tm_mon + 1) << 5) | tm.tm_mday);
    if (time_w) *time_w = (uint16_t)((tm.tm_hour << 11) | (tm.tm_min << 5) | (tm.tm_sec >> 1));
}

/* Write an 8.3 directory entry at 'slot' in the root cluster */
static void write_dirent(uint32_t slot, const char *name, uint32_t size,
                         uint32_t first_cluster, time_t mtime)
{
    uint8_t *root = s_meta + ROOT_OFFSET;
    uint8_t *e    = root + slot * 32U;
    memset(e, 0x20, 11);   /* fill name with spaces */

    /* Build short 8.3 name from the filename (truncate/uppercase) */
    const char *dot = strrchr(name, '.');
    int namelen = dot ? (int)(dot - name) : (int)strlen(name);
    if (namelen > 8) namelen = 8;
    for (int i = 0; i < namelen; i++) {
        char c = name[i];
        if (c >= 'a' && c <= 'z') c -= 32;
        e[i] = (uint8_t)c;
    }
    if (dot) {
        const char *ext = dot + 1;
        for (int i = 0; i < 3 && ext[i]; i++) {
            char c = ext[i];
            if (c >= 'a' && c <= 'z') c -= 32;
            e[8 + i] = (uint8_t)c;
        }
    }

    e[11] = 0x20;   /* attribute: archive */
    uint16_t dos_date, dos_time;
    unix_to_dos(mtime, &dos_date, &dos_time);
    put_le16(e + 22, dos_time);
    put_le16(e + 24, dos_date);
    put_le16(e + 26, (uint16_t)(first_cluster >> 16));  /* hi cluster */
    put_le16(e + 28, (uint16_t)(first_cluster & 0xFFFF)); /* lo cluster */
    put_le32(e + 28, size);
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t fat32_meta_init(void)
{
    s_meta = psram_fat_meta();
    if (!s_meta) return ESP_ERR_INVALID_STATE;
    if (META_BYTES > psram_fat_meta_size()) {
        ESP_LOGE(TAG, "PSRAM fat region too small: need %u, have %u",
                 (unsigned)META_BYTES, (unsigned)psram_fat_meta_size());
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t fat32_meta_build(const char *volume_label)
{
    if (!s_meta) return ESP_ERR_INVALID_STATE;
    memset(s_meta, 0, META_BYTES);

    /* ── Boot sector (sector 0) ──────────────────────────────────────────── */
    uint8_t *bs = s_meta;
    bs[0] = 0xEB; bs[1] = 0x58; bs[2] = 0x90;          /* jmp + NOP */
    memcpy(bs + 3, "MSDOS5.0", 8);                       /* OEM name */
    put_le16(bs + 11, DISK_SECTOR_SIZE);                  /* bytes/sector */
    bs[13] = FAT32_CLUSTER_SECS;                          /* sectors/cluster */
    put_le16(bs + 14, FAT32_RESERVED_SECS);               /* reserved sectors */
    bs[16] = 2;                                           /* num FATs */
    put_le16(bs + 17, 0);                                 /* root entry count (FAT32: 0) */
    put_le16(bs + 19, 0);                                 /* total sectors 16 (FAT32: 0) */
    bs[21] = 0xF8;                                        /* media type: fixed */
    put_le16(bs + 22, 0);                                 /* FAT size 16 (FAT32: 0) */
    put_le16(bs + 24, 63);                                /* sectors/track */
    put_le16(bs + 26, 255);                               /* num heads */
    put_le32(bs + 28, 0);                                 /* hidden sectors */
    put_le32(bs + 32, DISK_SECTOR_COUNT);                 /* total sectors 32 */
    put_le32(bs + 36, FAT32_FAT_SECS);                   /* FAT size 32 */
    put_le16(bs + 40, 0);                                 /* ext flags */
    put_le16(bs + 42, 0);                                 /* FS version 0.0 */
    put_le32(bs + 44, FAT32_ROOT_CLUSTER);                /* root cluster = 2 */
    put_le16(bs + 48, 1);                                 /* FSInfo sector = 1 */
    put_le16(bs + 50, 6);                                 /* backup boot sector */
    bs[64]  = 0x80;                                       /* drive number */
    bs[66]  = 0x29;                                       /* extended boot sig */
    put_le32(bs + 67, 0xDEADBEEFU);                      /* volume serial */
    /* Volume label (11 bytes, space-padded) */
    memset(bs + 71, 0x20, 11);
    if (volume_label) {
        int n = strlen(volume_label);
        if (n > 11) n = 11;
        memcpy(bs + 71, volume_label, n);
    }
    memcpy(bs + 82, "FAT32   ", 8);
    bs[510] = 0x55; bs[511] = 0xAA;

    /* ── FSInfo (sector 1) ───────────────────────────────────────────────── */
    uint8_t *fs = s_meta + DISK_SECTOR_SIZE;
    put_le32(fs + 0,   0x41615252U);   /* lead sig */
    put_le32(fs + 484, 0x61417272U);   /* struct sig */
    put_le32(fs + 488, 0xFFFFFFFFU);   /* free cluster count (unknown) */
    put_le32(fs + 492, FAT32_ROOT_CLUSTER + 1U); /* next free cluster hint */
    put_le32(fs + 508, 0xAA550000U);   /* trail sig */

    /* ── FAT tables (both copies) ────────────────────────────────────────── */
    fat_set(0, 0x0FFFFFF8U);  /* media byte */
    fat_set(1, 0x0FFFFFFFU);  /* reserved */
    fat_set(FAT32_ROOT_CLUSTER, FAT32_EOC); /* root dir chain, single cluster */

    ESP_LOGI(TAG, "build: FAT32 metadata stamped in PSRAM "
             "(data_lba=%u, fat_secs=%u)",
             (unsigned)FAT32_DATA_LBA, (unsigned)FAT32_FAT_SECS);
    return ESP_OK;
}

esp_err_t fat32_meta_rebuild_from_ltfs(void)
{
    if (!s_meta) return ESP_ERR_INVALID_STATE;

    /* Clear root directory cluster */
    memset(s_meta + ROOT_OFFSET, 0, FAT32_CLUSTER_SECS * DISK_SECTOR_SIZE);

    const ltfs_index_t *idx = ltfs_index();
    if (!idx) return ESP_ERR_INVALID_STATE;

    uint32_t next_cluster = FAT32_ROOT_CLUSTER + 1U;  /* cluster allocation cursor */
    uint32_t dirent_slot  = 0;

    for (uint32_t i = 0; i < LTFS_MAX_FILES; i++) {
        const ltfs_file_entry_t *f = &idx->files[i];
        if (!f->valid) continue;

        /* Clusters needed for this file */
        uint32_t file_bytes    = f->size_bytes;
        uint32_t cluster_bytes = FAT32_CLUSTER_SECS * DISK_SECTOR_SIZE;
        uint32_t num_clusters  = (file_bytes + cluster_bytes - 1) / cluster_bytes;
        if (num_clusters == 0) num_clusters = 1;

        uint32_t first_cluster = next_cluster;
        for (uint32_t c = 0; c < num_clusters; c++) {
            uint32_t this_c = next_cluster + c;
            uint32_t next_c = (c == num_clusters - 1) ? FAT32_EOC : (this_c + 1);
            fat_set(this_c, next_c);
        }
        next_cluster += num_clusters;

        write_dirent(dirent_slot++, f->name, f->size_bytes,
                     first_cluster, (time_t)f->mtime);

        if (dirent_slot >= (FAT32_CLUSTER_SECS * DISK_SECTOR_SIZE / 32U)) {
            ESP_LOGW(TAG, "root dir full at %u entries", (unsigned)dirent_slot);
            break;
        }
    }

    ESP_LOGI(TAG, "rebuild: %u file entries, next_cluster=%u",
             (unsigned)dirent_slot, (unsigned)next_cluster);
    return ESP_OK;
}

esp_err_t fat32_meta_read_sector(uint32_t lba, uint8_t *dst)
{
    if (!s_meta || !dst)        return ESP_ERR_INVALID_ARG;
    if (lba >= META_SECTORS)    return ESP_ERR_INVALID_ARG;
    memcpy(dst, s_meta + lba * DISK_SECTOR_SIZE, DISK_SECTOR_SIZE);
    return ESP_OK;
}

esp_err_t fat32_meta_write_sector(uint32_t lba, const uint8_t *src)
{
    if (!s_meta || !src)        return ESP_ERR_INVALID_ARG;
    if (lba >= META_SECTORS)    return ESP_ERR_INVALID_ARG;
    memcpy(s_meta + lba * DISK_SECTOR_SIZE, src, DISK_SECTOR_SIZE);
    return ESP_OK;
}

uint32_t fat32_read_fat_entry(uint32_t cluster)
{
    if (!s_meta) return 0;
    uint32_t off = FAT1_OFFSET + cluster * 4U;
    return get_le32(s_meta + off) & 0x0FFFFFFFU;
}

uint32_t fat32_chain_length(uint32_t first_cluster)
{
    uint32_t count = 0, c = first_cluster;
    while (c < 0x0FFFFFF8U && c != 0) {
        c = fat32_read_fat_entry(c);
        count++;
        if (count > FAT32_DATA_CLUSTERS) break;  /* guard against corrupt FAT */
    }
    return count * FAT32_CLUSTER_SECS * DISK_SECTOR_SIZE;
}

/* Build the 8.3 short name for a dirent, same convention as
 * disk_io.c's sync_dirent_to_ltfs(). 'out' must hold at least 13 bytes.
 * Not reentrant (uses no shared state, but relies on the caller consuming
 * the name immediately) — fine given the single-threaded USB task. */
static void dirent_short_name(const uint8_t *e, char *out)
{
    int n = 0;
    for (int i = 0; i < 8 && e[i] != 0x20; i++) out[n++] = (char)e[i];
    if (e[8] != 0x20) {
        out[n++] = '.';
        for (int i = 8; i < 11 && e[i] != 0x20; i++) out[n++] = (char)e[i];
    }
    out[n] = '\0';
}

bool fat32_cluster_to_file(uint32_t cluster,
                            const char **out_filename,
                            uint32_t    *out_byte_offset)
{
    if (!s_meta) return false;

    /* Walk root directory to find which file owns this cluster chain */
    uint8_t *root = s_meta + ROOT_OFFSET;
    uint32_t max_dirents = (FAT32_CLUSTER_SECS * DISK_SECTOR_SIZE) / 32U;
    uint32_t cluster_bytes = FAT32_CLUSTER_SECS * DISK_SECTOR_SIZE;
    static char name[13];

    for (uint32_t i = 0; i < max_dirents; i++) {
        uint8_t *e = root + i * 32U;
        if (e[0] == 0x00) break;     /* end of directory */
        if (e[0] == 0xE5) continue;  /* deleted */
        if (e[11] & 0x08) continue;  /* volume label */
        if (e[11] & 0x10) continue;  /* subdirectory (none expected) */

        uint32_t fc = ((uint32_t)e[26] | ((uint32_t)e[27] << 8) |
                       ((uint32_t)e[20] << 16) | ((uint32_t)e[21] << 24));
        if (fc == 0) continue;

        /* Walk this file's cluster chain to find 'cluster' */
        uint32_t c = fc, offset = 0;
        while (c < 0x0FFFFFF8U && c != 0) {
            if (c == cluster) {
                /* Found — this dirent owns 'cluster'; return its own name. */
                dirent_short_name(e, name);
                if (out_filename)    *out_filename    = name;
                if (out_byte_offset) *out_byte_offset = offset;
                return true;
            }
            c = fat32_read_fat_entry(c);
            offset += cluster_bytes;
            if (offset > 0x10000000U) break;  /* 256 MB sanity guard */
        }
    }
    return false;
}
