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
#include "esp_system.h"
#include "esp_spiffs.h"
#include "config.h"
#include "update.h"
#include "tasks.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h"
#include "supervisor.h"
#include "interface/ntrip.h"
#include "esp_partition.h"
#include "nvs.h"
#include "uart.h"

static const char *TAG              = "OTA";
static       bool checkUpdates      = true;
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
#define OTA_CONFIRM_MIN_FRAMES    60ULL          /* ~10 s of MSM at ~7 Hz */
#define OTA_CONFIRM_MIN_TX_BYTES  10240ULL       /* caster accepted >= 10 KiB */
#define OTA_CONFIRM_UNPROV_MS     (10 * 60 * 1000)
#define OTA_CONFIRM_MIN_HEAP_B    20480

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
static bool ota_attempt_allowed(const char *version) {
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

esp_err_t ota_update_firmware(const char *url) {

    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "Beginn update file from url: %s", url);

    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < MIN_OTA_HEAP) {
        ESP_LOGE(TAG, "free heap %u < %u needed for OTA -> deferring (no crash)",
                 (unsigned)free_heap, (unsigned)MIN_OTA_HEAP);
        return ESP_FAIL;
    }

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

            }else{

                ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%08" PRIx32,
                    update_partition->subtype, update_partition->address);

                int content_length = esp_http_client_fetch_headers(http_client);

                if (content_length <= 0) {

                    ESP_LOGE(TAG, "Content length error");

                }else{

                    err = esp_ota_begin(update_partition, content_length, &ota_handle);

                    if (err != ESP_OK) {

                        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));

                    }else{

                        ESP_LOGI(TAG, "OTA begin successfull");

                        int     data_read;
                        char    buffer[1024];
                        bool    downloadBreak = false;

                        //download the firmware
                        while ((data_read = esp_http_client_read(http_client, buffer, sizeof(buffer))) > 0) {

                            //write the firmware to the flash
                            err = esp_ota_write(ota_handle, (const void *)buffer, data_read);

                            if (err != ESP_OK) {

                                ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));

                                esp_ota_end(ota_handle);

                                downloadBreak = true;

                                break;
                            }

                            total_file_write += data_read;

                            int progress = 0;

                            if(total_file_size > 0){

                                progress = (total_file_write * 100) / total_file_size;
                            }

                            send_update_progress(progress);
                        
                        }

                        if (data_read < 0) {

                            ESP_LOGE(TAG, "OTA data read error");

                            esp_ota_end(ota_handle);

                        }else if(!downloadBreak){

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

// Function to copy the file from SPIFFS to the WWW partition
static esp_err_t copy_file_to_www_partition(void) {

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

    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < MIN_OTA_HEAP) {
        ESP_LOGE(TAG, "free heap %u < %u needed for SPIFFS OTA -> deferring (no crash)",
                 (unsigned)free_heap, (unsigned)MIN_OTA_HEAP);
        return ESP_FAIL;
    }

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

        err = esp_http_client_open(http_client, 0);

        if (err != ESP_OK) {

            ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));

        }else{

            int content_length = esp_http_client_fetch_headers(http_client);

            if (content_length <= 0) {

                ESP_LOGE(TAG, "Content length error");

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

                        esp_err_t err = esp_partition_erase_range(partition, 0, partition->size);

                        if (err != ESP_OK) {

                            ESP_LOGE(TAG, "Partition: %s not erase : ", esp_err_to_name(err));

                            err = ESP_FAIL;
                        }else{

                            ESP_LOGI(TAG, "Partition '%s' was successfull erase.", SPIFFS_PARTITION_LABEL);

                            while ((data_read = esp_http_client_read(http_client, buffer, sizeof(buffer))) > 0) {

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
esp_err_t updateFirmware(cJSON *files_json) {

    ESP_LOGI(TAG, "Starting RTKdata OTA update...");

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

                ESP_LOGE(TAG, "Memory allocation failed for response");

                esp_http_client_cleanup(http_client);
   
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

            if (!ota_attempt_allowed(new_version)) {
                // exhausted the retry budget for this version -> do NOT attempt
                // again; boot the current (working) FW instead of crash-looping.
                cJSON_Delete(jsonFile);
                return;
            }
            ESP_LOGI(TAG, "newer version %s > %s -> installing at boot (full heap)", new_version, FW_VERSION);
            esp_err_t up = updateFirmware(files_json);   // reboots on success; returns on failure
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

void ota_boot_check_blocking(void) {
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
    xSemaphoreTake(s_ota_boot_done, portMAX_DELAY);   // the task reboots if it installs
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
void ota_boot_loop_guard(void) {
    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint8_t boots = 0;
    nvs_get_u8(h, OTA_KEY_BOOTLOOP, &boots);
    boots++;
    nvs_set_u8(h, OTA_KEY_BOOTLOOP, boots);
    nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "boot-loop guard: %u consecutive boot(s) without a healthy run", boots);
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
    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {   // clean slate for factory
        nvs_set_u8(h, OTA_KEY_BOOTLOOP, 0);
        nvs_commit(h);
        nvs_close(h);
    }
    esp_partition_erase_range(otadata, 0, otadata->size);
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
    uint32_t prev_rec_gnss = 0;
    int stable = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(OTA_CONFIRM_CHECK_MS));
        int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;

        ntrip_tx_totals_t t;
        ntrip_server_tx_totals(&t);
        supervisor_health_t h;
        supervisor_health(&h);

        bool gnss_ok    = t.frames_ok >= OTA_CONFIRM_MIN_FRAMES;
        bool no_new_rec = have_prev && (h.recoveries_gnss == prev_rec_gnss);
        prev_rec_gnss = h.recoveries_gnss;
        have_prev = true;
        bool caster_good = (t.instances > 0) &&
                           (t.accepted_to_lwip >= OTA_CONFIRM_MIN_TX_BYTES) &&
                           h.caster_ok;

        stable = (gnss_ok && no_new_rec && caster_good) ? stable + 1 : 0;

        if (stable >= OTA_CONFIRM_STABLE_N) {
            healthy = true;
            break;
        }

        // Explicit exception: no uploader configured (bench/spare unit). It can
        // never satisfy the caster leg; after a long stable-uptime window with
        // sane heap it confirms so it does not oscillate between images.
        if (t.instances == 0 && elapsed_ms >= OTA_CONFIRM_UNPROV_MS &&
            esp_get_free_heap_size() > OTA_CONFIRM_MIN_HEAP_B) {
            healthy = true;
            path = "unprovisioned";
            break;
        }

        if (elapsed_ms >= OTA_CONFIRM_DEADLINE_MS) break;
    }

    if (!healthy) {
        // Deliberately: no counter reset, no confirmation. A pending-verify
        // image will be rolled back by the bootloader on the next reboot.
        ESP_LOGE(TAG, "data plane never met the health contract within %d min; "
                 "image stays unconfirmed (pending-verify would roll back)",
                 OTA_CONFIRM_DEADLINE_MS / 60000);
        uart_nmea("$PESP,OTA,UNCONFIRMED,%s", FW_VERSION);
        vTaskDelete(NULL);
        return;
    }

    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, OTA_KEY_BOOTLOOP, 0);
        nvs_erase_key(h, OTA_KEY_FAILVER);
        nvs_erase_key(h, OTA_KEY_FAILCNT);
        nvs_commit(h);
        nvs_close(h);
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (running && esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "OTA image confirmed by %s health -> rollback cancelled", path);
            uart_nmea("$PESP,OTA,CONFIRMED,%s", FW_VERSION);
        } else {
            ESP_LOGE(TAG, "esp_ota_mark_app_valid failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGI(TAG, "firmware healthy (%s contract; not a pending-verify image, "
                 "nothing to confirm)", path);
    }
    vTaskDelete(NULL);
}

void ota_check_newupdate(void *pvParameter) {

    while (true) {

        if(checkUpdates){

            ESP_LOGI(TAG, "chek for new updates");

            // read version and update files url from json
            cJSON *jsonFile = ota_fetch_json_from_url();

            if(jsonFile){

                cJSON *version_json     = cJSON_GetObjectItem(jsonFile, "version");
                cJSON *files_json       = cJSON_GetObjectItem(jsonFile, "update_files_urls");

                if (cJSON_IsString(version_json) && cJSON_IsArray(files_json)) {

                    const char *new_version = version_json->valuestring;

                    // compare the versions
                    if (strcmp(new_version, FW_VERSION) == 0) {

                        ESP_LOGI(TAG, "No new update available. Current version: %s", FW_VERSION);

                    } else {

                        ESP_LOGI(TAG, "New version available. Starting OTA update from current Version: %s to new: %s", FW_VERSION, new_version);

                        //start the update
                        updateFirmware(files_json);

                    }

                } else {

                    ESP_LOGE(TAG, "Invalid JSON format: Missing 'version' or 'update_url'");
                }

                cJSON_Delete(jsonFile);

            }else{
                
                ESP_LOGE(TAG, "No JSON file received from server");
            }

            checkUpdates = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

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
                    if (newer) {
                        ESP_LOGI(TAG, "newer version %s available -> restart to install at boot (full heap)", v->valuestring);
                    }
                    cJSON_Delete(jsonFile);
                    if (newer) {
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