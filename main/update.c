/*
 * This file is part of update.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "config.h"
#include "update.h"
#include "tasks.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "supervisor.h"
#include "gnss_site.h"
#include "interface/ntrip.h"
#include "esp_partition.h"
#include "nvs.h"
#include "uart.h"
#include "wifi.h"

static const char *TAG              = "OTA";
             int  total_file_size   = 0;
             int  total_file_write  = 0;

esp_event_loop_handle_t ota_event_loop = NULL;

ESP_EVENT_DEFINE_BASE(OTA_EVENT);

// send OTA update progress
void send_update_progress(int progress) {
    esp_event_post_to(ota_event_loop, OTA_EVENT, OTA_EVENT_UPDATE_PROGRESS, &progress, sizeof(progress), portMAX_DELAY);
}

// send OTA update started
void send_update_started() {
    esp_event_post_to(ota_event_loop, OTA_EVENT, OTA_EVENT_UPDATE_STARTED, NULL, 0, portMAX_DELAY);
}

// send OTA update completed
void send_update_completed() {
    esp_event_post_to(ota_event_loop, OTA_EVENT, OTA_EVENT_UPDATE_COMPLETED, NULL, 0, portMAX_DELAY);
}

// A download needs the full mbedtls TLS + flash buffers. With the data plane up
// (NTRIP sockets, the provisioning HTTPS heartbeat, the web server) only ~58 KB
// remained free and the OTA's TLS handshake OOM-PANICKED -> crash loop. The
// boot check runs before the data plane (~136 KB free); this guard is the
// belt-and-suspenders: if heap is ever below this, defer GRACEFULLY (return,
// keep running) instead of crashing.
#define MIN_OTA_HEAP (110 * 1024)

// Internal RAM is what TLS, lwIP and the flash driver actually run out of. On
// the Ethernet board the total above includes 2 MB of PSRAM, so it always
// passed and guarded nothing (review 2026-11-06, O8). On OnoLink (no PSRAM)
// total and internal are the same number and nothing changes.
#define MIN_OTA_INTERNAL_HEAP (32 * 1024)

// One file download may take at most this long. esp_http_client's 10 s
// timeout is per read, so a server that keeps sending a little held the boot
// OTA, and with it the whole station, for hours (review 2026-11-06, O4).
// 1.3 MB at 10 KB/s takes about 2 min.
#define OTA_DOWNLOAD_DEADLINE_US (10LL * 60 * 1000 * 1000)

static bool ota_heap_sufficient(void) {
    return esp_get_free_heap_size() >= MIN_OTA_HEAP &&
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) >= MIN_OTA_INTERNAL_HEAP;
}

/* Read up to len body bytes, never past the deadline. esp_http_client_read
 * loops internally until its buffer is full, waiting up to the client timeout
 * for EVERY transport read, so a peer (or a path) that trickles one byte at a
 * time held a single call for hours and the deadline check between calls
 * never ran (review R7). Small requests and a 1 s poll timeout make the
 * deadline bite on any normal or slow link. They are NOT a hard bound: the
 * TLS socket keeps esp-tls' own receive timeout and mbedTLS assembles a whole
 * record inside one call, so a peer or path that trickles a record byte by
 * byte can still hold a call (review R9). The hard bound is the boot OTA's
 * OTA_BOOT_HARD_LIMIT_MS. A read that times out with nothing is retried while
 * time is left: a pause on a slow link is not a failure. Returns bytes read,
 * 0 at the end of the body, -1 on error or when the deadline passed with
 * nothing read. */
#define OTA_READ_PIECE 64
#define OTA_READ_TIMEOUT_MS 1000

static int ota_read_bounded(esp_http_client_handle_t c, char *buf, int len, int64_t deadline_us) {
    int got = 0;
    while (got < len) {
        int64_t left_ms = (deadline_us - esp_timer_get_time()) / 1000;
        if (left_ms <= 0) return got > 0 ? got : -1;
        int to = left_ms > OTA_READ_TIMEOUT_MS ? OTA_READ_TIMEOUT_MS : (left_ms < 50 ? 50 : (int)left_ms);
        esp_http_client_set_timeout_ms(c, to);
        int want = len - got;
        if (want > OTA_READ_PIECE) want = OTA_READ_PIECE;
        int r = esp_http_client_read(c, buf + got, want);
        if (r == -ESP_ERR_HTTP_EAGAIN) continue;   // nothing within 1 s: wait on, deadline permitting
        if (r <= 0) return got > 0 ? got : (r == 0 ? 0 : -1);
        got += r;
        if (r < want) break;   // end of body, or a timeout with partial data
    }
    return got;
}

static bool ota_heap_ok(const char *what) {
    size_t total = esp_get_free_heap_size();
    size_t internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (total >= MIN_OTA_HEAP && internal >= MIN_OTA_INTERNAL_HEAP) return true;
    ESP_LOGE(TAG, "%s: free heap %u (internal %u) < %u (%u) needed -> deferring (no crash)",
             what, (unsigned)total, (unsigned)internal, (unsigned)MIN_OTA_HEAP,
             (unsigned)MIN_OTA_INTERNAL_HEAP);
    return false;
}

/* ---- anti-brick OTA state (NVS) ---------------------------------------
 * Two counters in NVS namespace "ota" guarantee a bad update can never leave the
 * device stuck in a crash-loop (the v1.0.1-era failure mode):
 *   fail_ver/fail_cnt : per-target-version DOWNLOAD-attempt counter. After
 *                       MAX_OTA_ATTEMPTS failed installs of a given version we
 *                       stop retrying it and boot the current FW (device usable).
 *   bootloop          : consecutive boots that never reached a healthy run.
 *                       MAX_BOOT_LOOPS in a row -> fall back to the FACTORY app.
 * Both are reset by ota_mark_valid_task once the semantic health contract holds.
 * Paired with CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE: a freshly-OTA'd image boots
 * PENDING_VERIFY and the bootloader auto-reverts it if it crashes before the
 * health task confirms it. */
#define OTA_NVS_NS       "ota"
#define OTA_KEY_FAILVER  "fail_ver"
#define OTA_KEY_FAILCNT  "fail_cnt"
#define OTA_KEY_BOOTLOOP "bootloop"
#define OTA_KEY_PLANNED  "planned_rb"   /* the next SW reset was intentional */
#define OTA_KEY_SKIPBOOT "skip_boot"    /* the last boot OTA hit its hard limit */
#define OTA_KEY_QUARANT  "quarantine"   /* version that crash-looped into factory */

/* The boot OTA blocks app_main (NTRIP, supervisor) until it is done. The read
 * deadlines bound the body; the header phase and the manifest fetch still
 * wait up to the client timeout per read, and a peer or path that trickles
 * bytes can stretch them (review R8). This is the backstop for all of it:
 * past the limit the unit restarts and skips the boot OTA once, so streaming
 * resumes; the daily check arms it again. At most one stall per day. */
#define OTA_BOOT_HARD_LIMIT_MS (14 * 60 * 1000)
#define MAX_OTA_ATTEMPTS 3
#define MAX_BOOT_LOOPS   5

/* Semantic health confirmation (review 2026-08-01, finding A2): a pure
 * elapsed-time confirm made rollback ineffective, because a live-but-broken
 * data plane (device up, receiver being reset, nothing reaching the caster)
 * still cancelled rollback after 60 s. The image is now confirmed only after
 * the data plane PROVED itself:
 *   - the parser produced CRC-valid RTCM frames (receiver alive and decoded),
 *   - the caster accepted a meaningful amount of bytes and the supervisor's
 *     caster watchdog is content,
 *   - no GNSS recovery fired between consecutive checks,
 * held for OTA_CONFIRM_STABLE_N consecutive checks. A device with NO enabled
 * uploader can never meet the caster condition; it confirms via the explicit
 * unprovisioned exception (long uptime + sane heap) so a bench/spare unit does
 * not oscillate between images. If health is never reached, the image stays
 * PENDING_VERIFY and the next reboot rolls back - which is the point. */
#define OTA_CONFIRM_CHECK_MS      (10 * 1000)
#define OTA_CONFIRM_STABLE_N      3
#define OTA_CONFIRM_DEADLINE_MS   (30 * 60 * 1000)
/* DELTAS per check interval, not cumulative totals since boot: a stream that
 * died after meeting a lifetime threshold once would otherwise keep passing
 * (review 2026-08-03, P0). */
#define OTA_CONFIRM_MIN_FRAME_DELTA 10ULL        /* new CRC-valid frames / 10 s */
#define OTA_CONFIRM_MIN_BYTE_DELTA  2048ULL      /* new bytes accepted / 10 s */
#define OTA_CONFIRM_UNPROV_MS     (10 * 60 * 1000)
#define OTA_CONFIRM_MIN_HEAP_B    20480
/* Pending image that may give way to a newer release (rescue, review 13.11.
 * L1 R3): unconfirmed for this long since its boot. */
#define OTA_RESCUE_PENDING_US     (6LL * 3600 * 1000 * 1000)
/* "did not crash" is a separate question from "data plane works". Long enough
 * that a deterministic late crash cannot keep resetting the counter. */
#define OTA_BOOTLOOP_CLEAR_MS     (45 * 60 * 1000)

// true iff 'cand' is a strictly newer dotted version than 'cur' ("1.0.2">"1.0.1").
// Prevents downgrades (channel older than the running build) re-flashing in a
// loop. Unparseable -> fall back to "any change" so an update is still possible.
static bool ota_version_is_newer(const char *cand, const char *cur) {
    int ca[3] = {0, 0, 0}, cu[3] = {0, 0, 0};
    int nca = sscanf(cand, "%d.%d.%d", &ca[0], &ca[1], &ca[2]);
    int ncu = sscanf(cur,  "%d.%d.%d", &cu[0], &cu[1], &cu[2]);
    if (nca < 1 || ncu < 1) return strcmp(cand, cur) != 0;
    for (int i = 0; i < 3; i++) {
        if (ca[i] > cu[i]) return true;
        if (ca[i] < cu[i]) return false;
    }
    return false;
}

// Gate an install attempt on the per-version failure counter. Returns true and
// records the attempt (BEFORE the download, so a crash mid-download still counts)
// when the version has failed fewer than MAX_OTA_ATTEMPTS times; false once it is
// exhausted, so the caller boots the current FW instead of re-trying forever.
// Fail-OPEN on any NVS error: never let a storage hiccup block a real update.
/* A release that crash-looped this unit into the factory app is never
 * installed again. Before, the factory app reinstalled it from the channel,
 * it confirmed (clearing its attempt counter), crashed again five boots
 * later, fell back, and the cycle repeated without end (review R10, O1). The
 * mark is cleared only by a different version being offered: a fixed release
 * always carries a new version number. */
static bool ota_version_quarantined(const char *version) {
    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    char q[24] = {0};
    size_t len = sizeof(q);
    bool hit = nvs_get_str(h, OTA_KEY_QUARANT, q, &len) == ESP_OK && strcmp(q, version) == 0;
    nvs_close(h);
    return hit;
}

static bool ota_attempt_allowed(const char *version) {
    if (ota_version_quarantined(version)) {
        ESP_LOGE(TAG, "version %s crash-looped this unit before -> quarantined, not installing", version);
        uart_nmea("$PESP,OTA,QUARANTINED,%s", version);
        return false;
    }
    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "ota nvs unavailable -> allowing attempt (fail-open)");
        return true;
    }
    char   stored[24] = {0};
    size_t len        = sizeof(stored);
    uint8_t cnt       = 0;
    if (nvs_get_str(h, OTA_KEY_FAILVER, stored, &len) == ESP_OK && strcmp(stored, version) == 0) {
        nvs_get_u8(h, OTA_KEY_FAILCNT, &cnt);
    } else {
        cnt = 0;   // a different (or no) version was recorded -> fresh start
    }
    if (cnt >= MAX_OTA_ATTEMPTS) {
        ESP_LOGE(TAG, "version %s failed %u/%u times -> NOT retrying (staying on %s, device usable)",
                 version, cnt, MAX_OTA_ATTEMPTS, FW_VERSION);
        uart_nmea("$PESP,OTA,GIVEUP,%s,%u", version, cnt);
        nvs_close(h);
        return false;
    }
    nvs_set_str(h, OTA_KEY_FAILVER, version);
    nvs_set_u8(h, OTA_KEY_FAILCNT, (uint8_t)(cnt + 1));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "OTA attempt %u/%u for version %s", cnt + 1, MAX_OTA_ATTEMPTS, version);
    return true;
}

// Read-only twin of ota_attempt_allowed(): "would an attempt still be allowed?"
// WITHOUT booking one. The daily poll needs this: it used to reboot purely on
// "a newer version is published", never consulting the budget, so a device that
// had already exhausted its three attempts kept rebooting every single night
// forever, losing ~70 s of corrections each time and never installing anything
// (found 2026-08-10). Fail-OPEN like its twin: an NVS hiccup must not block a
// real update.
static bool ota_attempts_exhausted(const char *version) {
    if (ota_version_quarantined(version)) return true;
    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    char    stored[24] = {0};
    size_t  len        = sizeof(stored);
    uint8_t cnt        = 0;
    if (nvs_get_str(h, OTA_KEY_FAILVER, stored, &len) == ESP_OK && strcmp(stored, version) == 0) {
        nvs_get_u8(h, OTA_KEY_FAILCNT, &cnt);
    }
    nvs_close(h);
    return cnt >= MAX_OTA_ATTEMPTS;
}

/* The version the manifest announced for the image being installed. The
 * identity gate refuses an image whose app descriptor says otherwise: a
 * manifest bumped without replacing the .bin installed the old image, booted
 * it, saw the newer manifest again and reinstalled it every day until the
 * attempt budget ran out (review 2026-11-06, O2). Empty = no check. */
static char s_expected_version[32];

esp_err_t ota_update_firmware(const char *url) {

    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "Beginn update file from url: %s", url);

    if (!ota_heap_ok("firmware OTA")) return ESP_FAIL;

    // start http client
    esp_http_client_config_t http_config = {
        .url                =   url,
        .timeout_ms         =   10000,
        .crt_bundle_attach  =   esp_crt_bundle_attach,
    };
        
    esp_http_client_handle_t http_client = esp_http_client_init(&http_config);
    
    if (http_client == NULL) {

        ESP_LOGE(TAG, "Failed to initialize HTTP client");


    }else{

        int64_t dl_deadline = esp_timer_get_time() + OTA_DOWNLOAD_DEADLINE_US;
        err = esp_http_client_open(http_client, 0);

        if (err != ESP_OK) {

            ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));

        }else{

            ESP_LOGI(TAG, "Looking for ota partition...");

            // start OTA-update
            esp_ota_handle_t ota_handle = 0;

            const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

            if (!update_partition) {

                ESP_LOGE(TAG, "No OTA partition found");
                err = ESP_FAIL;

            }else{

                ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%08" PRIx32,
                    update_partition->subtype, update_partition->address);

                int content_length = esp_http_client_fetch_headers(http_client);
                int http_status    = esp_http_client_get_status_code(http_client);

                if (http_status != 200) {

                    // Unchecked before: a 404/500 body was written into the OTA
                    // slot as if it were firmware (review 2026-08-03).
                    ESP_LOGE(TAG, "firmware HTTP status %d (expected 200)", http_status);
                    err = ESP_FAIL;

                }else if (content_length <= 0) {

                    ESP_LOGE(TAG, "Content length error");
                    err = ESP_FAIL;

                }else{

                    err = esp_ota_begin(update_partition, content_length, &ota_handle);

                    if (err != ESP_OK) {

                        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));

                    }else{

                        ESP_LOGI(TAG, "OTA begin successfull");

                        int     data_read;
                        char    buffer[1024];
                        bool    downloadBreak = false;
                        bool    identity_checked = false;
                        //download the firmware
                        while ((data_read = ota_read_bounded(http_client, buffer, sizeof(buffer), dl_deadline)) > 0) {

                            if (esp_timer_get_time() > dl_deadline) {
                                ESP_LOGE(TAG, "firmware download exceeded %lld s -> aborted",
                                         OTA_DOWNLOAD_DEADLINE_US / 1000000);
                                uart_nmea("$PESP,OTA,TIMEOUT");
                                downloadBreak = true;
                                break;
                            }

                            //write the firmware to the flash
                            err = esp_ota_write(ota_handle, (const void *)buffer, data_read);

                            if (err != ESP_OK) {

                                ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));

                                // No cleanup here: exactly ONE owner below.
                                // Ending the handle here and again in the
                                // downloadBreak branch used a freed handle
                                // (review 2026-08-03).
                                downloadBreak = true;

                                break;
                            }

                            total_file_write += data_read;

                            // Identity gate, once the app descriptor is on
                            // flash: an image built for the other board (other
                            // project name) must never become bootable. The
                            // Wi-Fi fleet and the Ethernet board share this
                            // code but not pins; the channels are separate
                            // paths, this catches a manifest pointing at the
                            // wrong .bin (2026-09-29).
                            if (!identity_checked &&
                                total_file_write >= (int)(sizeof(esp_image_header_t) +
                                                          sizeof(esp_image_segment_header_t) +
                                                          sizeof(esp_app_desc_t))) {

                                identity_checked = true;

                                esp_app_desc_t incoming;
                                const esp_app_desc_t *running = esp_app_get_description();

                                if (esp_ota_get_partition_description(update_partition, &incoming) != ESP_OK ||
                                    strncmp(incoming.project_name, running->project_name,
                                            sizeof(incoming.project_name)) != 0) {

                                    incoming.project_name[sizeof(incoming.project_name) - 1] = '\0';
                                    ESP_LOGE(TAG, "OTA image is for project '%s', this is '%s' -> refused",
                                             incoming.project_name, running->project_name);
                                    uart_nmea("$PESP,OTA,WRONGIMAGE,%s", incoming.project_name);
                                    downloadBreak = true;
                                    break;
                                }

                                incoming.version[sizeof(incoming.version) - 1] = '\0';
                                if (s_expected_version[0] &&
                                    strcmp(incoming.version, s_expected_version) != 0) {

                                    ESP_LOGE(TAG, "OTA image is version '%s', the manifest announced '%s' -> refused",
                                             incoming.version, s_expected_version);
                                    uart_nmea("$PESP,OTA,WRONGVERSION,%s,%s", incoming.version,
                                              s_expected_version);
                                    downloadBreak = true;
                                    break;
                                }
                            }

                            int progress = 0;

                            if(total_file_size > 0){

                                progress = (total_file_write * 100) / total_file_size;
                            }

                            send_update_progress(progress);
                        
                        }

                        // Exactly one cleanup owner, and the right call for the
                        // outcome: esp_ota_abort() on failure, esp_ota_end()
                        // only on success (ESP-IDF v5.3 OTA API).
                        if (data_read < 0 || downloadBreak) {

                            // The result used to be discarded here, so the
                            // earlier ESP_OK from esp_http_client_open survived
                            // and the caller treated an aborted download as a
                            // success: it then updated www and rebooted,
                            // leaving the OLD app with a NEW web partition
                            // (review 2026-08-03).
                            ESP_LOGE(TAG, "OTA download failed (read=%d, write_abort=%d)",
                                     data_read, (int)downloadBreak);

                            esp_ota_abort(ota_handle);
                            err = ESP_FAIL;

                        }else if (total_file_write != content_length) {

                            ESP_LOGE(TAG, "OTA short download: %d of %d bytes",
                                     total_file_write, content_length);
                            esp_ota_abort(ota_handle);
                            err = ESP_FAIL;

                        }else{

                            // ota write successfull end
                            err = esp_ota_end(ota_handle);

                            if (err != ESP_OK) {

                                ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));

                            }else{

                                // set ota boot partition
                                err = esp_ota_set_boot_partition(update_partition);

                                if (err != ESP_OK) {

                                    ESP_LOGE(TAG, "OTA set boot partition failed: %s", esp_err_to_name(err));

                                }
                            }

                        }

                    }

                }

            }

        }

        if (esp_http_client_get_transport_type(http_client) != HTTP_TRANSPORT_UNKNOWN) {
            
            esp_http_client_close(http_client);
        } 

        esp_http_client_cleanup(http_client);

        //http_client = NULL;
    }

    return err;
 
}

// Set while the LIVE www partition is erased and rewritten: the boot OTA's
// hard limit must not restart the unit in the middle of that copy, or the
// web UI is left half written with no later repair (review R9).
static volatile bool s_www_promoting = false;
// Once the hard limit decided to restart, no new promotion may begin; one
// already admitted is waited for. Both decisions under one lock (review R10).
static volatile bool s_restart_chosen = false;
static portMUX_TYPE s_promote_mux = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t copy_file_to_www_partition_impl(void);

// Function to copy the file from SPIFFS to the WWW partition
static esp_err_t copy_file_to_www_partition(void) {
    portENTER_CRITICAL(&s_promote_mux);
    bool admitted = !s_restart_chosen;
    if (admitted) s_www_promoting = true;
    portEXIT_CRITICAL(&s_promote_mux);
    if (!admitted) {
        ESP_LOGW(TAG, "restart pending -> live www left untouched");
        return ESP_FAIL;
    }
    esp_err_t err = copy_file_to_www_partition_impl();
    s_www_promoting = false;
    return err;
}

static esp_err_t copy_file_to_www_partition_impl(void) {

    esp_err_t   err         =   ESP_FAIL;

    const  esp_partition_t *www_partition    = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 
        ESP_PARTITION_SUBTYPE_ANY, WWW_PARTITION_LABEL);

     const esp_partition_t *spiffs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 
        ESP_PARTITION_SUBTYPE_ANY, SPIFFS_PARTITION_LABEL);
    
    if (!www_partition || !spiffs_partition) {

        ESP_LOGE(TAG, "Failed to find WWW  or SPIFFS partition for copy file.");

    }else{

        err = esp_partition_erase_range(www_partition, 0, www_partition->size);

        if (err != ESP_OK) {

            ESP_LOGE(TAG, "Failed to errase www partition: %s", WWW_PARTITION_LABEL);

        }else{

            ESP_LOGI(TAG, "Erasing WWW partition");
            ESP_LOGI(TAG, "Copying file to WWW partition");

            char    buffer[1024];
            size_t  offset = 0;

            while (offset < spiffs_partition->size) {

                    size_t read_size = 1024;

                    if (offset + read_size > spiffs_partition->size) {
                        read_size = spiffs_partition->size - offset;
                    }

                    err = esp_partition_read(spiffs_partition, offset, buffer, read_size);

                    if (err != ESP_OK) {

                        ESP_LOGE(TAG, "Error at read SPIFFS-Partition: %s", esp_err_to_name(err));
                        break;
                    }

                    err = esp_partition_write(www_partition, offset, buffer, read_size);

                    if (err != ESP_OK) {

                        ESP_LOGE(TAG, "Error at write WWW-Partition: %s", esp_err_to_name(err));
                        break;
                    }

                    offset += read_size;

                    //ESP_LOGI(TAG, "Move: %d/%d Bytes",(int)offset,(int)www_partition->size);
            }

            if(err == ESP_OK){

                    ESP_LOGI(TAG, "File copied from SPIFFS to WWW partition successfully");
            }

        }

    }

    return err;
}

esp_err_t update_SPIFFS(const char *url) {

    ESP_LOGI(TAG, "Beginn update spiffs file from url: %s", url);

    esp_err_t err = ESP_FAIL;

    if (!ota_heap_ok("SPIFFS OTA")) return ESP_FAIL;

    // start http client
    esp_http_client_config_t http_config = {
        .url                =   url,
        .timeout_ms         =   10000,
        .crt_bundle_attach  =   esp_crt_bundle_attach,
    };
        
    esp_http_client_handle_t http_client = esp_http_client_init(&http_config);
    
    if (http_client == NULL) {

        ESP_LOGE(TAG, "Failed to initialize HTTP client");

    }else{

        int64_t dl_deadline = esp_timer_get_time() + OTA_DOWNLOAD_DEADLINE_US;
        err = esp_http_client_open(http_client, 0);

        if (err != ESP_OK) {

            ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));

        }else{

            int content_length = esp_http_client_fetch_headers(http_client);
            int http_status    = esp_http_client_get_status_code(http_client);

            if (http_status != 200) {

                // Without this a 404 body was happily written over the staging
                // partition and then copied across the live web UI
                // (review 2026-08-03).
                ESP_LOGE(TAG, "www.bin HTTP status %d (expected 200)", http_status);
                err = ESP_FAIL;

            }else if (content_length <= 0) {

                ESP_LOGE(TAG, "Content length error");
                err = ESP_FAIL;

            }else{

                const esp_partition_t *partition = esp_partition_find_first(
                        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, SPIFFS_PARTITION_LABEL);

                if (partition == NULL) {

                    ESP_LOGE("PARTITION", "SPIFFS-Partition '%s' not found!", SPIFFS_PARTITION_LABEL);

                    err = ESP_FAIL;
                }else{

                        int     data_read       = 0;
                        int     offset          = 0;
                        char    buffer[1024];

                        // NO shadowing declaration here: a local `esp_err_t err`
                        // used to hide every erase/write/copy failure from the
                        // caller, which then rebooted onto a half-written web
                        // partition and reported success (review 2026-08-03).
                        err = esp_partition_erase_range(partition, 0, partition->size);

                        if (err != ESP_OK) {

                            ESP_LOGE(TAG, "Partition: %s not erase : ", esp_err_to_name(err));

                            err = ESP_FAIL;
                        }else{

                            ESP_LOGI(TAG, "Partition '%s' was successfull erase.", SPIFFS_PARTITION_LABEL);

                            while ((data_read = ota_read_bounded(http_client, buffer, sizeof(buffer), dl_deadline)) > 0) {

                                if (esp_timer_get_time() > dl_deadline) {
                                    ESP_LOGE(TAG, "www download exceeded %lld s -> aborted",
                                             OTA_DOWNLOAD_DEADLINE_US / 1000000);
                                    data_read = -1;
                                    break;
                                }

                                err = esp_partition_write(partition, offset, buffer, data_read);

                                if (err != ESP_OK) {

                                    ESP_LOGE(TAG, "Error write data to the partition: %s", esp_err_to_name(err));
                                    data_read = -1;

                                    break;
                                }

                                offset           += data_read;
                                total_file_write += data_read;

                                int progress = 0;

                                if(total_file_size > 0){

                                    progress = (total_file_write * 100) / total_file_size;
                                }

                                send_update_progress(progress);

                                //ESP_LOGI(TAG, "Write file: data %i, progress: %i",data_read,progress);
                            }

                            ESP_LOGI(TAG, "Finisched download the www.bin to spiffs.");

                            if (data_read < 0) {

                                ESP_LOGE(TAG, "File download to SPIFFS failed, data read %i",data_read);
                                err = ESP_FAIL;
                            }else if (offset != content_length) {

                                // Short download: the staging copy is truncated.
                                // Erasing the LIVE www partition for this would
                                // destroy the web UI with no way back (an app
                                // rollback does not restore a data partition).
                                ESP_LOGE(TAG, "www.bin truncated: %d of %d bytes; "
                                         "NOT touching the live www partition",
                                         offset, content_length);
                                err = ESP_FAIL;
                            }else{

                                err = copy_file_to_www_partition();

                                if(err != ESP_OK){

                                    ESP_LOGE(TAG, "Copy File to www parttion failed");

                                }else{

                                    ESP_LOGI(TAG, "File www.bin was successfully updated to %s", WWW_FINAL_FILE);
                                }

                            }

                        }

                }                 
               
            }
 
        }

        if (esp_http_client_get_transport_type(http_client) != HTTP_TRANSPORT_UNKNOWN) {
            
            esp_http_client_close(http_client);
        }

        esp_http_client_cleanup(http_client);

        //http_client = NULL;
    }

    return err;
 
}

bool isWWWBinFile(const char *url) {

    bool wwwbinfile = false;

    wwwbinfile = (strstr(url, "www.bin") != NULL);

    return wwwbinfile;
}

// Function to calculate the total size of all files
void calculate_total_file_size(cJSON *files_json) {

    total_file_size     =   0;
    total_file_write    =   0;

    if (!cJSON_IsArray(files_json)) {

        ESP_LOGE(TAG, "Provided JSON for total file size is not an array");

    }else{

        cJSON* file_url = NULL;
        
        cJSON_ArrayForEach(file_url, files_json) {
            
            if( cJSON_IsString(file_url)){

                // start http client
                esp_http_client_config_t http_config = {
                    .url                =   file_url->valuestring,
                    .timeout_ms         =   10000,
                    .crt_bundle_attach  =   esp_crt_bundle_attach,
                };
        
                esp_http_client_handle_t http_client = esp_http_client_init(&http_config);
    
                if (http_client == NULL) {

                    ESP_LOGE(TAG, "Failed to initialize HTTP client to: %s",(char*)file_url->valuestring);

                }else{

                    esp_err_t err = esp_http_client_open(http_client, 0);

                    if (err != ESP_OK) {

                        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
                    }else{

                        int content_length = esp_http_client_fetch_headers(http_client);

                        total_file_size += content_length;

                    }

                    if (esp_http_client_get_transport_type(http_client) != HTTP_TRANSPORT_UNKNOWN) {
            
                        esp_http_client_close(http_client);
                    }

                    esp_http_client_cleanup(http_client);

                    //http_client = NULL;

                }

            }

        }

    }

    ESP_LOGI(TAG,"Calculate total file size for download: %i",total_file_size);

}

// Download + apply every file in the manifest. Returns ESP_OK only if ALL files
// applied; on the FIRST failure it stops and returns ESP_FAIL WITHOUT rebooting.
// (The old version called esp_restart() unconditionally -> a failed download
// rebooted -> re-attempted the same download -> crash loop. Now: reboot only on
// success, so a failed install just falls through to the current, working FW.)
esp_err_t updateFirmware(cJSON *files_json, const char *expected_version) {

    ESP_LOGI(TAG, "Starting RTKdata OTA update...");

    // Every file must come over TLS: the certificate bundle authenticates the
    // server, plain HTTP would let anyone on the path supply the image
    // (review 2026-11-06, O6). Checked for ALL files before the first byte is
    // written, so a mixed manifest cannot install half an update.
    cJSON *u;
    cJSON_ArrayForEach(u, files_json) {
        if (!cJSON_IsString(u) || strncmp(u->valuestring, "https://", 8) != 0) {
            ESP_LOGE(TAG, "manifest file URL is not HTTPS -> update refused");
            uart_nmea("$PESP,OTA,NOTHTTPS");
            return ESP_FAIL;
        }
    }
    strlcpy(s_expected_version, expected_version ? expected_version : "",
            sizeof(s_expected_version));

    send_update_started();

    calculate_total_file_size(files_json);

    bool ok = true;
    cJSON* file_url;

    cJSON_ArrayForEach(file_url, files_json) {

        if( cJSON_IsString(file_url)){

            if(!isWWWBinFile(file_url->valuestring)){

                if (ota_update_firmware(file_url->valuestring) == ESP_OK) {

                    ESP_LOGI(TAG, "Firmware from %s was successfull updated.", file_url->valuestring);
                } else {

                    ESP_LOGE(TAG, "Error on firmware update from %s", file_url->valuestring);
                    ok = false;
                    break;
                }

            }else{

                if (update_SPIFFS(file_url->valuestring) == ESP_OK) {

                    ESP_LOGI(TAG, "Spiffs from %s was successfull updated.", file_url->valuestring);
                } else {

                    ESP_LOGE(TAG, "Error on spiffs update from %s", file_url->valuestring);
                    ok = false;
                    break;
                }

            }

        }

    }

    send_update_completed();

    if (ok) {
        ESP_LOGI(TAG, "OTA update completed -> restarting into the new image");
        uart_nmea("$PESP,OTA,INSTALLED,restarting");
        ota_note_planned_reboot();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();          // no return on success
    }

    // Failed: do NOT restart. Return so the caller continues on the current FW.
    ESP_LOGE(TAG, "OTA update FAILED -> staying on current FW %s (no reboot)", FW_VERSION);
    uart_nmea("$PESP,OTA,FAILED,%s", FW_VERSION);
    return ESP_FAIL;
}

cJSON* ota_fetch_json_from_url(){

    cJSON * jsonFile = NULL;

    // start http client
    esp_http_client_config_t http_config = {
        .url        = UPDATE_SERVER_URL VERSION_FILE,
        .timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t http_client = esp_http_client_init(&http_config);

    // Under heap pressure init returns NULL and every call below would
    // dereference it (review 2026-08-03).
    if (http_client == NULL) {
        ESP_LOGE(TAG, "manifest fetch: HTTP client init failed (heap?)");
        return NULL;
    }

    // GET HTTP-Request
    esp_http_client_set_method(http_client, HTTP_METHOD_GET);

    esp_err_t err = esp_http_client_open(http_client, 0);

    if (err == ESP_OK) {

        // check the answer
        int content_length  = esp_http_client_fetch_headers(http_client);

        if (content_length < 0) {

            ESP_LOGE(TAG, "HTTP client fetch headers failed");

        }else {
                            
            // read the release manifest from the RTKdata OTA channel
            char *response = malloc(content_length + 1);

            if (response == NULL) {

                // No cleanup here: the single cleanup at the end of the
                // function owns the handle. Doing both was a double free on
                // the OOM path (review 2026-08-03).
                ESP_LOGE(TAG, "Memory allocation failed for response");

            }else{

                int data_read  = esp_http_client_read_response(http_client, response, content_length);

                if (data_read >= 0) {

                    response[data_read] = '\0';
                                            
                    // parse JSON data
                    cJSON *json  = cJSON_Parse(response);

                    if (json == NULL) {

                        ESP_LOGE(TAG, "Failed to parse JSON response");

                    }else{

                        jsonFile = json;

                    }
                    
                } else {

                    ESP_LOGE(TAG, "Failed to read response");
                }

                free(response);
            }

        }
    
    } else {
                    
            ESP_LOGE(TAG, "Failed to perform HTTP request check new update: %s", esp_err_to_name(err));
            ESP_LOGE(TAG, "URL: %s",http_config.url);
    }

    esp_http_client_cleanup(http_client);

    return jsonFile;

}

// Synchronous one-shot OTA check, run at BOOT before the data plane (NTRIP /
// provisioning / web server) starts. At that point ~136 KB heap is free, so the
// heavy TLS download has room -- vs ~58 KB once the data plane is up, which
// OOM-crashed the in-task download. If a strictly newer version is published it
// downloads + applies + reboots (never returns); otherwise it returns and the
// caller proceeds to bring up the data plane.
void ota_boot_check(void) {

    ESP_LOGI(TAG, "boot OTA check (free heap=%u)", (unsigned)esp_get_free_heap_size());

    cJSON *jsonFile = ota_fetch_json_from_url();

    if (!jsonFile) {
        ESP_LOGW(TAG, "boot OTA check: no manifest reachable, skipping");
        return;
    }

    cJSON *version_json = cJSON_GetObjectItem(jsonFile, "version");
    cJSON *files_json   = cJSON_GetObjectItem(jsonFile, "update_files_urls");

    if (cJSON_IsString(version_json) && cJSON_IsArray(files_json)) {

        const char *new_version = version_json->valuestring;

        if (ota_version_is_newer(new_version, FW_VERSION)) {

            // The config AP (plus its DHCP server and captive DNS) is STILL UP
            // here: it auto-closes 15 min after boot, this check runs ~2 min in.
            // On 1.1.5 that left 109 700 bytes free against the 112 640 the TLS
            // download demands, so the install deferred on EVERY boot of EVERY
            // device (measured 2026-08-10, with no client on the AP). Ask the
            // Wi-Fi control task to close the AP now; it declines while a client
            // is mid-onboarding, which is the correct trade and just means we
            // defer once more.
            // The same predicate as the admission below (total AND internal):
            // with PSRAM the total alone always passed, the AP stayed up and
            // the internal check then deferred on every boot (review R7).
            if (!ota_heap_sufficient()) {
                ESP_LOGW(TAG, "free heap %u (internal %u) too low -> closing the config AP to make room",
                         (unsigned)esp_get_free_heap_size(),
                         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
                wifi_ap_stop_now();

                // POLL, do not sleep a fixed span: the teardown is asynchronous
                // (control task -> esp_wifi_set_mode -> DHCP/DNS teardown) and a
                // guessed delay is either too short (fix silently useless) or
                // wasted boot time on every device that is already fine.
                for (int i = 0; i < 30 && !ota_heap_sufficient(); i++) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                // Closing the AP changes the Wi-Fi mode on a live driver. If that
                // disturbed the STA association we must NOT walk into the
                // download: it would fail and burn a REAL attempt for a problem
                // we caused ourselves. No IP -> defer, uplink first.
                if (!wait_for_ip(5000)) {
                    ESP_LOGW(TAG, "no STA IP after closing the AP -> deferring %s (no attempt booked)",
                             new_version);
                    cJSON_Delete(jsonFile);
                    return;
                }
            }

            // Budget check BEFORE booking. ota_attempt_allowed() WRITES the
            // per-version counter and that counter is only cleared after a
            // CONFIRMED install, so booking here and then bailing out on the
            // heap pre-check inside ota_update_firmware() burned all three
            // attempts without downloading a single byte -- after which the
            // device refused that version forever (measured 2026-08-10:
            // "OTA attempt 3/3 for version 1.1.6", no download). A deferral is
            // not an attempt; retry on the next boot instead.
            {
                size_t heap_now = esp_get_free_heap_size();
                size_t internal_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
                if (heap_now < MIN_OTA_HEAP || internal_now < MIN_OTA_INTERNAL_HEAP) {
                    ESP_LOGW(TAG, "deferring %s: free heap %u < %u, no attempt booked, retry next boot",
                             new_version, (unsigned)heap_now, (unsigned)MIN_OTA_HEAP);
                    uart_nmea("$PESP,OTA,DEFER,%s,%u", new_version, (unsigned)heap_now);
                    cJSON_Delete(jsonFile);
                    return;
                }
            }

            if (!ota_attempt_allowed(new_version)) {
                // exhausted the retry budget for this version -> do NOT attempt
                // again; boot the current (working) FW instead of crash-looping.
                cJSON_Delete(jsonFile);
                return;
            }
            ESP_LOGI(TAG, "newer version %s > %s -> installing at boot (full heap)", new_version, FW_VERSION);
            esp_err_t up = updateFirmware(files_json, new_version);   // reboots on success; returns on failure
            ESP_LOGE(TAG, "boot OTA install did not succeed (%s) -> continuing on current FW %s",
                     esp_err_to_name(up), FW_VERSION);
        } else {
            ESP_LOGI(TAG, "no newer version (channel=%s current=%s)", new_version, FW_VERSION);
        }
    } else {
        ESP_LOGE(TAG, "boot OTA check: invalid manifest (missing version/urls)");
    }

    cJSON_Delete(jsonFile);
}

/* ---- anti-brick boot helpers ------------------------------------------ */

// Run the boot OTA check on a dedicated 16 KB-stack task and block until it is
// done. The synchronous TLS download + flash write needs ~16 KB of stack; doing
// it on the 3584-byte main task overflowed (0xa5a5a5a5 stack-smash -> crash
// loop). The task self-reboots on a successful install, so this only returns
// when there is nothing to install or the install failed (device keeps the
// current, working FW).
static SemaphoreHandle_t s_ota_boot_done = NULL;

static void ota_boot_check_task(void *arg) {
    (void)arg;
    ota_boot_check();
    if (s_ota_boot_done) xSemaphoreGive(s_ota_boot_done);
    vTaskDelete(NULL);
}

static bool ota_skip_boot_flag(bool clear) {
    nvs_handle_t h;
    uint8_t v = 0;
    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    nvs_get_u8(h, OTA_KEY_SKIPBOOT, &v);
    if (v && clear) {
        nvs_erase_key(h, OTA_KEY_SKIPBOOT);
        nvs_commit(h);
    }
    nvs_close(h);
    return v != 0;
}

static void ota_set_skip_boot(void) {
    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, OTA_KEY_SKIPBOOT, 1);
    nvs_commit(h);
    nvs_close(h);
}

void ota_boot_check_blocking(void) {
    // A freshly installed image that is still PENDING_VERIFY must reach its
    // health confirmation first. Running the boot OTA on it and hitting the
    // hard limit restarts the unit before confirmation, and the bootloader
    // then rolls a healthy image back because the OTA server was slow
    // (review R9). It has just installed the newest version anyway.
    {
        const esp_partition_t *run = esp_ota_get_running_partition();
        esp_ota_img_states_t st;
        if (run && esp_ota_get_state_partition(run, &st) == ESP_OK &&
            st == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "running image is pending verification -> boot OTA check skipped");
            return;
        }
    }
    if (ota_skip_boot_flag(true)) {
        ESP_LOGW(TAG, "previous boot OTA hit its %d min limit -> skipping it once",
                 OTA_BOOT_HARD_LIMIT_MS / 60000);
        uart_nmea("$PESP,OTA,BOOTSKIP");
        return;
    }
    s_ota_boot_done = xSemaphoreCreateBinary();
    if (!s_ota_boot_done) {
        ESP_LOGE(TAG, "no mem for OTA sync -> skipping boot OTA (device stays usable)");
        return;
    }
    if (xTaskCreate(ota_boot_check_task, "ota_boot", 16384, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "could not spawn OTA boot task (low mem) -> skipping boot OTA (device stays usable)");
        vSemaphoreDelete(s_ota_boot_done);
        s_ota_boot_done = NULL;
        return;
    }
    // the task reboots if it installs
    bool done = xSemaphoreTake(s_ota_boot_done, pdMS_TO_TICKS(OTA_BOOT_HARD_LIMIT_MS)) == pdTRUE;
    if (!done) {
        // Decide to restart and close the door to a new promotion in one
        // step; a copy already running is let finish.
        portENTER_CRITICAL(&s_promote_mux);
        s_restart_chosen = true;
        portEXIT_CRITICAL(&s_promote_mux);
        while (!done && s_www_promoting) {
            done = xSemaphoreTake(s_ota_boot_done, pdMS_TO_TICKS(1000)) == pdTRUE;
        }
    }
    if (!done) {
        ESP_LOGE(TAG, "boot OTA exceeded %d min -> restart, next boot skips it",
                 OTA_BOOT_HARD_LIMIT_MS / 60000);
        uart_nmea("$PESP,OTA,BOOTLIMIT");
        ota_set_skip_boot();
        ota_note_planned_reboot();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
    vSemaphoreDelete(s_ota_boot_done);
    s_ota_boot_done = NULL;
}

// Anti-brick boot-loop guard. Call ONCE early in app_main (after NVS init, before
// the risky bring-up). Counts consecutive boots that never reach a healthy run;
// after MAX_BOOT_LOOPS in a row it forces a boot into the FACTORY app (the
// bench-flashed known-good image) by erasing the OTA-select data, so the device
// can never stay stuck in a crash-loop regardless of cause. ota_mark_valid_task
// resets the counter once the semantic health contract holds. Fail-safe: any
// NVS error just returns (never blocks boot).
void ota_note_planned_reboot(void) {
    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, OTA_KEY_PLANNED, 1);
    nvs_commit(h);
    nvs_close(h);
}

void ota_boot_loop_guard(void) {
    // What counts as evidence that THIS image is broken:
    //   - crash resets (panic, watchdogs, brownout, unknown), and
    //   - software resets we did NOT plan.
    // The second half matters because the supervisor recovers via
    // esp_restart(), which produces ESP_RST_SW: excluding all SW resets
    // (as the previous version did) meant a reproducible supervisor reboot
    // loop never reached the factory fallback. Planned restarts (OTA install,
    // user action) leave an explicit NVS marker instead of being guessed from
    // the hardware reason (review 2026-08-03).
    esp_reset_reason_t rr = esp_reset_reason();

    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;

    uint8_t planned = 0;
    nvs_get_u8(h, OTA_KEY_PLANNED, &planned);
    if (planned) {                       // one-shot: consume it
        nvs_set_u8(h, OTA_KEY_PLANNED, 0);
        nvs_commit(h);
    }

    // A freshly installed image is PENDING_VERIFY on its first boot, which
    // means the reboot we just came from WAS an OTA install. The firmware that
    // performed it is the OLD one and cannot be expected to set our marker:
    // measured on the bench 2026-08-03, a clean 1.0.6 -> 1.1.4 update was
    // counted as "3 consecutive CRASH boot(s)". A pending-verify image is also
    // already covered by the bootloader's own rollback if it crashes, so the
    // boot-loop counter is not the safety net here.
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t rst;
    bool just_installed = run && esp_ota_get_state_partition(run, &rst) == ESP_OK &&
                          rst == ESP_OTA_IMG_PENDING_VERIFY;

    // A brownout is the supply, not the image: a weak PoE injector or a long
    // cable can brown out any firmware, factory included, and counting it
    // made five sags in 45 minutes quarantine a healthy release for good
    // (review 13.11. L1, R7). Not counted, the streak is left as it is.
    bool crash_like = (rr == ESP_RST_PANIC) || (rr == ESP_RST_INT_WDT) ||
                      (rr == ESP_RST_TASK_WDT) || (rr == ESP_RST_WDT) ||
                      (rr == ESP_RST_UNKNOWN) ||
                      (rr == ESP_RST_SW && !planned && !just_installed);

    uint8_t boots = 0;
    nvs_get_u8(h, OTA_KEY_BOOTLOOP, &boots);

    if (!crash_like) {
        nvs_close(h);
        ESP_LOGI(TAG, "boot-loop guard: reset reason %d%s%s is not a fault, "
                 "not counted (streak stays %u)", (int)rr,
                 planned ? " (planned)" : "",
                 just_installed ? " (fresh OTA image)" : "", boots);
        return;
    }

    boots++;
    nvs_set_u8(h, OTA_KEY_BOOTLOOP, boots);
    nvs_commit(h);
    nvs_close(h);

    ESP_LOGW(TAG, "boot-loop guard: %u consecutive CRASH boot(s) (reason %d)",
             boots, (int)rr);
    if (boots < MAX_BOOT_LOOPS) return;

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        ESP_LOGE(TAG, "crash-loop (%u) but already on FACTORY -> cannot fall back further", boots);
        return;
    }
    const esp_partition_t *otadata = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    if (!otadata) {
        ESP_LOGE(TAG, "crash-loop (%u) but no otadata partition -> cannot force factory", boots);
        return;
    }

    ESP_LOGE(TAG, "crash-loop (%u boots) -> erasing otadata, rebooting into FACTORY app", boots);
    uart_nmea("$PESP,OTA,FACTORYFALLBACK,%u", boots);

    // Erase FIRST, and only clear the counter once the erase actually
    // succeeded: clearing first meant a failed erase rebooted the same broken
    // app with a fresh counter, so the fallback could never happen
    // (review 2026-08-03).
    esp_err_t er = esp_partition_erase_range(otadata, 0, otadata->size);
    if (er != ESP_OK) {
        ESP_LOGE(TAG, "otadata erase failed (%s); keeping the counter so the next "
                 "boot retries the fallback", esp_err_to_name(er));
        return;
    }

    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {   // clean slate for factory
        nvs_set_u8(h, OTA_KEY_BOOTLOOP, 0);
        // This running image is the one that keeps crashing (the guard
        // returns early on the factory app): never reinstall this version.
        nvs_set_str(h, OTA_KEY_QUARANT, FW_VERSION);
        nvs_commit(h);
        nvs_close(h);
    }
    ota_note_planned_reboot();
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();   // bootloader now selects the factory app
}

// Anti-brick health-confirm task. Spawn ONCE at the end of app_main (after the
// data plane is up). Once the data plane meets the SEMANTIC health contract
// (see the OTA_CONFIRM_* block) it (1) resets the crash-loop +
// download-failure counters and (2) if the running image is a freshly-OTA'd
// one pending verification, confirms it valid so the bootloader keeps it.
// If health is never reached, nothing is confirmed: the bootloader rolls back
// the OTA image on the next reboot, and the boot-loop guard keeps counting.
void ota_mark_valid_task(void *pvParameter) {
    (void)pvParameter;

    const int64_t t0 = esp_timer_get_time();
    const char *path = "semantic";
    bool healthy = false;
    bool have_prev = false;
    bool outage_logged = false;
    uint32_t prev_rec_gnss = 0;
    uint64_t prev_frames = 0, prev_bytes = 0;
    int stable = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(OTA_CONFIRM_CHECK_MS));
        int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;

        ntrip_tx_totals_t t;
        ntrip_server_tx_totals(&t);
        supervisor_health_t h;
        supervisor_health(&h);
        ntrip_msg_freshness_t fresh;
        ntrip_server_msg_freshness(&fresh);

        uint64_t d_frames = (t.frames_ok        > prev_frames) ? t.frames_ok        - prev_frames : 0;
        uint64_t d_bytes  = (t.accepted_to_lwip > prev_bytes)  ? t.accepted_to_lwip - prev_bytes  : 0;
        bool no_new_rec   = have_prev && (h.recoveries_gnss == prev_rec_gnss);

        prev_frames   = t.frames_ok;
        prev_bytes    = t.accepted_to_lwip;
        prev_rec_gnss = h.recoveries_gnss;
        have_prev = true;

        // Health is: the receiver is producing the REQUIRED message set right
        // now, those frames are moving to the caster right now, and no
        // recovery fired in between. Cumulative totals and a caster timestamp
        // refreshed by bare handshakes both passed a dead stream before.
        bool gnss_ok     = (d_frames >= OTA_CONFIRM_MIN_FRAME_DELTA) && fresh.all_fresh;
        bool caster_good = (t.instances > 0) && (d_bytes >= OTA_CONFIRM_MIN_BYTE_DELTA);

        stable = (gnss_ok && no_new_rec && caster_good) ? stable + 1 : 0;

        if (stable >= OTA_CONFIRM_STABLE_N) {
            healthy = true;
            break;
        }

        // Explicit exception: no uploader configured (bench/spare unit). It can
        // never satisfy the caster leg; after a long stable-uptime window with
        // sane heap it confirms so it does not oscillate between images.
        // "No uploader" must mean none CONFIGURED: zero RUNNING with one
        // configured is a fault (tasks failed to start), and confirming then
        // kept a broken image (review 2026-11-06, N3).
        if (t.configured == 0 && t.instances == 0 && elapsed_ms >= OTA_CONFIRM_UNPROV_MS &&
            esp_get_free_heap_size() > OTA_CONFIRM_MIN_HEAP_B) {
            healthy = true;
            path = "unprovisioned";
            break;
        }

        if (elapsed_ms >= OTA_CONFIRM_DEADLINE_MS) {

            // Deadline reached without health. "Firmware is broken" and "the
            // outside world is unreachable" are DIFFERENT states (review
            // 2026-08-03): rolling back a locally sound image because the
            // caster was down for 30 minutes would turn an infrastructure
            // incident into a fleet-wide downgrade.
            //
            // LOCAL fault (the receiver is not producing the required RTCM
            // set) -> roll back now.
            // EXTERNAL fault (GNSS fine, only the network leg missing) -> keep
            // looping. The image stays pending-verify, so it is still rolled
            // back by a crash or reboot, but if the caster returns in minute
            // 31 this task is still here to confirm it. Exiting here made that
            // confirmation impossible forever.
            ntrip_msg_freshness_t fresh_final;
            ntrip_server_msg_freshness(&fresh_final);
            gnss_site_status_t site;
            gnss_site_status(&site);
            /* A verified antenna far from the restored fixed coordinate can
             * intentionally suppress UM980 RTCM until IE has fenced the old
             * site. Do not downgrade to firmware without relocation fencing
             * because that deliberate hold looks like missing RTCM. This is
             * only a wait exception, never an OTA confirmation. */
            bool verified_site_hold = !site.upload_allowed && site.valid;
#ifdef CONFIG_RTK_SITE_LEGACY
            /* A survey started by the relocation guard (or any MODE in its
             * warmup) has no RTCM for about five minutes by design, while
             * the receiver reports valid positions. Not a broken image
             * (review 13.11. L3 #4). Only an ACKNOWLEDGED mode, and only for
             * GNSS_SITE_SETTLE_MAX_MS after it: an UNKNOWN mode (failed
             * MODE, the case rollback exists for) never qualifies (L4 #1). */
            verified_site_hold = verified_site_hold ||
                (site.valid && gnss_site_mode_settling());
#endif
            bool local_fault = !fresh_final.all_fresh && !verified_site_hold;

            if (!local_fault) {
                if (!outage_logged) {
                    ESP_LOGW(TAG, "health contract not met within %d min, but GNSS is "
                             "healthy: treating as an EXTERNAL outage, staying "
                             "unconfirmed and retrying",
                             OTA_CONFIRM_DEADLINE_MS / 60000);
                    uart_nmea("$PESP,OTA,UNCONFIRMED,%s,0", FW_VERSION);
                    outage_logged = true;
                }
                continue;
            }

            ESP_LOGE(TAG, "local data-plane fault after %d min",
                     OTA_CONFIRM_DEADLINE_MS / 60000);
            uart_nmea("$PESP,OTA,UNCONFIRMED,%s,1", FW_VERSION);

            const esp_partition_t *run = esp_ota_get_running_partition();
            esp_ota_img_states_t st;
            if (run && esp_ota_get_state_partition(run, &st) == ESP_OK &&
                st == ESP_OTA_IMG_PENDING_VERIFY) {
                ESP_LOGE(TAG, "rolling back to the previous image");
                uart_nmea("$PESP,OTA,ROLLBACK,%s", FW_VERSION);
                vTaskDelay(pdMS_TO_TICKS(200));            // let the NMEA flush
                // Returns (instead of rebooting) when there is no rollback-able
                // image; that must not look like success.
                esp_err_t rb = esp_ota_mark_app_invalid_rollback_and_reboot();
                ESP_LOGE(TAG, "rollback did not happen: %s", esp_err_to_name(rb));
                uart_nmea("$PESP,OTA,ROLLBACKFAIL,%s", FW_VERSION);
            }
            // Nothing confirmed, nothing reset: the anti-brick counters keep
            // their meaning for the next boot.
            vTaskDelete(NULL);
            return;
        }
    }

    // Order matters: CONFIRM FIRST, clear the attempt counter only after the
    // confirmation actually succeeded. Clearing first meant a failing
    // mark-valid left the device with no retry budget for this version
    // (review 2026-08-03).
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    bool confirmed = false;

    if (running && esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err == ESP_OK) {
            confirmed = true;
            ESP_LOGI(TAG, "OTA image confirmed by %s health -> rollback cancelled", path);
            uart_nmea("$PESP,OTA,CONFIRMED,%s", FW_VERSION);
        } else {
            ESP_LOGE(TAG, "esp_ota_mark_app_valid failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGI(TAG, "firmware healthy (%s contract; not a pending-verify image, "
                 "nothing to confirm)", path);
    }

    // The per-version download-attempt counter may ONLY be cleared once THIS
    // version is confirmed valid. Clearing it while running a healthy OLD
    // image reset the 3-attempt protection every day, so a permanently broken
    // release artifact was re-downloaded forever (review 2026-08-03).
    if (confirmed) {
        nvs_handle_t h;
        if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_key(h, OTA_KEY_FAILVER);
            nvs_erase_key(h, OTA_KEY_FAILCNT);
            nvs_commit(h);
            nvs_close(h);
        }
    }
    vTaskDelete(NULL);
}

// Boot-loop counter clear, deliberately its OWN task on its OWN clock.
//
// It answers exactly one question: "did this image survive long enough that a
// crash loop is ruled out?" Tying it to data-plane health made it depend on
// the caster (a long outage plus a few reboots dropped a healthy device to
// factory), and clearing it on the healthy path let a regression that crashes
// at second 41 reset the counter on every boot, so the factory fallback was
// never reached. The window must therefore be longer than any plausible
// deterministic crash point AND independent of every other timer here.
void ota_bootloop_clear_task(void *pvParameter) {
    (void)pvParameter;
    vTaskDelay(pdMS_TO_TICKS(OTA_BOOTLOOP_CLEAR_MS));

    nvs_handle_t h;
    esp_err_t err = nvs_open(OTA_NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, OTA_KEY_BOOTLOOP, 0);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    // Reporting success on an ignored NVS error meant the counter silently
    // kept climbing (review 2026-08-03).
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "survived %d min without a crash -> boot-loop counter cleared",
                 OTA_BOOTLOOP_CLEAR_MS / 60000);
    } else {
        ESP_LOGE(TAG, "boot-loop counter NOT cleared: %s", esp_err_to_name(err));
    }
    vTaskDelete(NULL);
}

// Daily OTA poll. Does a LIGHTWEIGHT manifest fetch only (one small ~1 KB HTTPS
// GET, which runs fine even with the data plane up); it reboots ONLY when a
// strictly newer version is actually published, so the heavy download then runs
// at boot with full heap. No new version -> NO restart (zero reboots in normal
// operation; the device only reboots to install a real update).
void ota_schedule_check_newupdate(void *pvParameter){

    ESP_LOGI(TAG, "OTA daily-check scheduler started");

    bool checked_this_minute = false;

    while (true) {

        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        if (timeinfo.tm_hour == 0 && timeinfo.tm_min == 0) {

            if (!checked_this_minute) {
                checked_this_minute = true;
                ESP_LOGI(TAG, "daily OTA check (free heap=%u)", (unsigned)esp_get_free_heap_size());

                cJSON *jsonFile = ota_fetch_json_from_url();
                if (jsonFile) {
                    cJSON *v = cJSON_GetObjectItem(jsonFile, "version");
                    bool newer = cJSON_IsString(v) && ota_version_is_newer(v->valuestring, FW_VERSION);
                    // Never reboot for a version this device has already given up
                    // on: the boot check would refuse to install it anyway, so the
                    // restart is pure customer downtime, repeated every night.
                    if (newer && ota_attempts_exhausted(v->valuestring)) {
                        ESP_LOGW(TAG, "newer version %s available but its attempt budget is spent -> NOT rebooting",
                                 v->valuestring);
                        newer = false;
                    }
                    // A restart now would make the bootloader roll this still
                    // unconfirmed image back, healthy or not (review R10).
                    // Unless it has stayed unconfirmed for OTA_RESCUE_PENDING_US:
                    // GNSS healthy but no upload keeps an image pending for
                    // ever, and refusing every newer release then also refused
                    // the fix for that very upload (review 13.11. L1, R3).
                    // Back to the previous image, which installs the newer
                    // release with its own boot check.
                    bool rescue = false;
                    if (newer) {
                        const esp_partition_t *run = esp_ota_get_running_partition();
                        esp_ota_img_states_t st;
                        if (run && esp_ota_get_state_partition(run, &st) == ESP_OK &&
                            st == ESP_OTA_IMG_PENDING_VERIFY) {
                            if (esp_timer_get_time() >= OTA_RESCUE_PENDING_US) {
                                ESP_LOGW(TAG, "newer version %s available, this image unconfirmed for %u h -> "
                                         "back to the previous image, which installs it",
                                         v->valuestring, (unsigned)(esp_timer_get_time() / 3600000000LL));
                                uart_nmea("$PESP,OTA,RESCUE,%s", v->valuestring);
                                rescue = true;
                            } else {
                                ESP_LOGW(TAG, "newer version %s available, but this image is not confirmed yet -> not rebooting today",
                                         v->valuestring);
                            }
                            newer = false;
                        }
                    }
                    if (newer) {
                        ESP_LOGI(TAG, "newer version %s available -> restart to install at boot (full heap)", v->valuestring);
                    }
                    cJSON_Delete(jsonFile);
                    if (rescue) {
                        vTaskDelay(pdMS_TO_TICKS(300));   // let the NMEA flush
                        esp_err_t rb = esp_ota_mark_app_invalid_rollback_and_reboot();
                        ESP_LOGE(TAG, "rescue rollback did not happen: %s", esp_err_to_name(rb));
                    }
                    if (newer) {
                        ota_note_planned_reboot();
                        vTaskDelay(pdMS_TO_TICKS(300));
                        esp_restart();   // boot OTA check downloads + applies with full heap
                    }
                } else {
                    ESP_LOGW(TAG, "daily OTA check: manifest not reachable");
                }
            }
        } else {
            checked_this_minute = false;
        }

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void ota_update_init(){

    // Initialize SPIFFS
    esp_vfs_spiffs_conf_t spiffs_config = {
        .base_path              = SPIFFS_PARTITION_PATH,
        .partition_label        = SPIFFS_PARTITION_LABEL,
        .max_files              = 1,
        .format_if_mount_failed = true
    };

    if (esp_vfs_spiffs_register(&spiffs_config) != ESP_OK) {

        ESP_LOGE(TAG, "Failed to initialize temp partition for SPIFFS");
    }

    // OTA Event loop
    esp_event_loop_args_t loop_args = {
        .queue_size         = 10,
        .task_name          = "ota_event_task",
        .task_priority      = 5,         
        .task_stack_size    = 4096,     
        .task_core_id       = 0  
    };

    esp_err_t err = esp_event_loop_create(&loop_args, &ota_event_loop);

    if(err != ESP_OK){

        ESP_LOGE(TAG,"Failed to create custom event loop: %s", esp_err_to_name(err)); 
    }else{

        ESP_LOGI(TAG, "Custom event loop created successfully");
    }
}
