/* Independent UM980 BASEPOS monitoring and site-v1 wire contract.
 * Platform-neutral: production parsing/quality rules also run in host tests. */
#ifndef RTKDATA_GNSS_SITE_H
#define RTKDATA_GNSS_SITE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GNSS_SITE_LINE_MAX 768
#define GNSS_SITE_WINDOW 64
#define GNSS_SITE_MIN_SAMPLES 30
#define GNSS_SITE_MIN_SPAN_MS 60000U
#define GNSS_SITE_MAX_AGE_MS 5000U
#define GNSS_SITE_H_SIGMA_MAX_MM 3000U
#define GNSS_SITE_V_SIGMA_MAX_MM 6000U

typedef enum {
    GNSS_SITE_UNKNOWN = 0, GNSS_SITE_SURVEY = 1, GNSS_SITE_FIXED = 2,
    GNSS_SITE_HOLD = 3
} gnss_site_mode_t;

typedef struct {
    bool quality;
    uint64_t epoch_ms;
    uint32_t received_ms, solution_age_ms;
    double lat, lon, h;
    uint32_t h_sigma_mm, v_sigma_mm;
    uint8_t sats;
} gnss_site_sample_t;

typedef struct {
    char line[GNSS_SITE_LINE_MAX];
    size_t line_len;
    gnss_site_sample_t sample[GNSS_SITE_WINDOW];
    uint16_t count, head;
    uint32_t seq;
    uint64_t last_epoch_ms;
    bool epoch_seen;
    gnss_site_sample_t latest;
} gnss_site_monitor_t;

typedef struct {
    uint32_t boot_id, seq;
    bool valid;
    int32_t lat_e7, lon_e7, h_mm;
    uint32_t h_sigma_mm, v_sigma_mm;
    uint8_t sats;
    uint16_t samples;
    uint32_t span_ms, age_ms, desired_revision, applied_revision;
    gnss_site_mode_t mode;
    bool upload_allowed;
} gnss_site_status_t;

typedef struct {
    uint32_t boot_id, revision;
    gnss_site_mode_t mode;
    bool allow_upload;
    int64_t lat_e9, lon_e9;
    int32_t h_mm;
} gnss_site_control_t;

/* One complete desired-state blob in NVS, never an upload grant. */
#define GNSS_SITE_NVS_MAGIC 0x53495431U
typedef struct {
    uint32_t magic, revision, mode, reserved;
    int64_t lat_e9, lon_e9;
    int32_t h_mm;
    uint32_t reserved2;
} gnss_site_desired_t;

uint32_t gnss_site_crc32(const uint8_t *bytes, size_t len);
bool gnss_site_parse_basepos(const char *line, size_t len, gnss_site_sample_t *out);
void gnss_site_monitor_init(gnss_site_monitor_t *monitor);
/* True for complete CRC-valid BASEPOS, including no-fix records.
 * Repeated receiver epochs cannot contribute another quality sample. */
bool gnss_site_monitor_feed(gnss_site_monitor_t *monitor, const uint8_t *bytes,
                            size_t len, uint32_t now_ms);
void gnss_site_monitor_snapshot(const gnss_site_monitor_t *monitor, uint32_t now_ms,
                                gnss_site_status_t *out);
uint32_t gnss_site_horizontal_mm(double lat1, double lon1, double lat2, double lon2);
bool gnss_site_displaced(double lat, double lon, uint32_t h_sigma_mm,
                         double reference_lat, double reference_lon);
const char *gnss_site_mode_name(gnss_site_mode_t mode);
bool gnss_site_desired_valid(const gnss_site_desired_t *desired);
bool gnss_site_control_matches(const gnss_site_desired_t *desired,
                               const gnss_site_control_t *control);
int gnss_site_json(char *dst, size_t cap, const gnss_site_status_t *site);
/* Exact HMAC suffix after device_id|uptime|nonce|site-v1|. */
int gnss_site_canonical(char *dst, size_t cap, const gnss_site_status_t *site);

/* ESP-IDF runtime adapter (not linked into parser-only host tests). */
void gnss_site_runtime_init(void);
void gnss_site_receiver_reset(void);
void gnss_site_runtime_ingest(const uint8_t *bytes, size_t len);
void gnss_site_status(gnss_site_status_t *out);
bool gnss_site_upload_allowed(void);
bool gnss_site_needs_recovery(void);
bool gnss_site_output_expected(void);
/* Required-RTCM log repair may run: gnss_site_output_expected() with site-v1;
 * with CONFIG_RTK_SITE_LEGACY the mode warmup plus flowing observations. */
bool gnss_site_log_repair_expected(void);
uint32_t gnss_site_upload_generation(void);
bool gnss_site_get_desired(gnss_site_desired_t *out);
bool gnss_site_legacy_allowed(void);
void gnss_site_configured(gnss_site_mode_t mode, double lat, double lon,
                          uint32_t expected_revision, bool applied);
bool gnss_site_apply_control(const gnss_site_control_t *control);

/* Relocation guard, CONFIG_RTK_SITE_LEGACY only (no IE half exists there).
 * The receiver's own independent position, the median of a valid and fresh
 * BASEPOS window, judged against a coordinate with gnss_site_displaced(). */
typedef enum {
    GNSS_SITE_AGREEMENT_UNKNOWN = 0,   /* no valid, fresh window yet */
    GNSS_SITE_AGREEMENT_AGREE = 1,
    GNSS_SITE_AGREEMENT_DISAGREE = 2
} gnss_site_agreement_t;
/* A unit fixed on a coordinate it has left must not keep broadcasting it. */
#define GNSS_SITE_RELOCATE_MS 300000U
/* The receiver sends its 1005 every 10 s; older than this it is not "now". */
#define GNSS_SITE_ARP_MAX_AGE_MS 30000U
/* The reference point a complete, CRC-checked RTCM 1005/1006 frame
 * (0xD3 header to CRC) broadcasts, as WGS84 latitude/longitude in degrees.
 * Platform-neutral, host-tested against a recorded UM980 frame. */
bool gnss_site_parse_arp(const uint8_t *frame, size_t len, double *lat, double *lon);
/* Runtime: note a CRC-valid RTCM frame; only 1005/1006 are used (legacy). */
void gnss_site_runtime_note_frame(const uint8_t *frame, size_t len);
/* Legacy: an acknowledged FIXED or SURVEY MODE applied less than
 * GNSS_SITE_SETTLE_MAX_MS ago, or a SURVEY within 330 s of its first valid
 * position (late acquisition, L5). Its RTCM pause is deliberate and bounded;
 * an UNKNOWN mode never qualifies (OTA confirm, review 13.11. L4 #1). */
#define GNSS_SITE_SETTLE_MAX_MS 900000U
bool gnss_site_mode_settling(void);
gnss_site_agreement_t gnss_site_legacy_agreement(double lat, double lon,
                                                 uint32_t *distance_mm);
/* True once the receiver, fixed on its reference, has placed itself away
 * from it in every valid window for GNSS_SITE_RELOCATE_MS. */
bool gnss_site_legacy_relocated(uint32_t *distance_mm);
#endif
