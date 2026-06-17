/*
 * This file is part of update.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#include "esp_partition.h"

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

void updateFirmware(cJSON *files_json) {

    ESP_LOGI(TAG, "Starting RTKdata OTA update...");

    send_update_started();

    calculate_total_file_size(files_json);

    cJSON* file_url;

    cJSON_ArrayForEach(file_url, files_json) {

        if( cJSON_IsString(file_url)){

            if(!isWWWBinFile(file_url->valuestring)){
                
                if (ota_update_firmware(file_url->valuestring) == ESP_OK) {

                    ESP_LOGI(TAG, "Firmware from %s was successfull updated.", file_url->valuestring);
                    //free(fileUrl);
                } else {

                    ESP_LOGE(TAG, "Error on firmware update from %s", file_url->valuestring);
                    //free(fileUrl);
                    break;
                }

            }else{

                if (update_SPIFFS(file_url->valuestring) == ESP_OK) {

                    ESP_LOGI(TAG, "Spiffs from %s was successfull updated.", file_url->valuestring);
                    //free(fileUrl);
                } else {

                    ESP_LOGE(TAG, "Error on spiffs update from %s", file_url->valuestring);
                    //free(fileUrl);
                    break;
                }

            }

        }

    }

    send_update_completed();
    
    ESP_LOGI(TAG, "OTA update completed. Restarting...");

    esp_restart();

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

            ESP_LOGI(TAG, "newer version %s > %s -> updating at boot (full heap)", new_version, FW_VERSION);
            updateFirmware(files_json);   // downloads, applies, esp_restart() -> no return
        } else {
            ESP_LOGI(TAG, "no newer version (channel=%s current=%s)", new_version, FW_VERSION);
        }
    } else {
        ESP_LOGE(TAG, "boot OTA check: invalid manifest (missing version/urls)");
    }

    cJSON_Delete(jsonFile);
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