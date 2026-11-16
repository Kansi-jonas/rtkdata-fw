/*
 * This file is part of the ESP32-XBee distribution (https://github.com/nebkat/esp32-xbee).
 * Copyright (c) 2019 Nebojsa Cvetkovic.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <esp_http_server.h>
#include <esp_log.h>
#include <wifi.h>
#include <cJSON.h>
#include <sys/param.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>

#include <config.h>
#include <log.h>
#include <core_dump.h>
#include <util.h>

#include <esp_ota_ops.h>
#include <nvs.h>
#include <esp_heap_caps.h>
#include <esp_netif.h>
//#include <esp_netif_sta_list.h> makeItCompileable
#include <stream_stats.h>
#include <esp32/rom/crc.h>
#include <lwip/sockets.h>
#include "web_server.h"
#include "esp_timer.h" //makeItCompileable
#include "board.h"
#include "eth.h"
#include "esp_mac.h"
#include "update.h"
#include "state.h"
#include "provisioning.h"
#include "interface/ntrip.h"
#include "uart.h"
#include "board_test.h"

// Max length a file path can have on storage
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)
#define FILE_HASH_SUFFIX ".crc"

// #define BUFFER_SIZE 2048
#define BUFFER_SIZE 4096
static const char *TAG = "WEB";

static char *buffer;

enum auth_method {
    AUTH_METHOD_OPEN = 0,
    AUTH_METHOD_HOTSPOT = 1,
    AUTH_METHOD_BASIC = 2
};

static char *basic_authentication;
static enum auth_method auth_method;

static bool isFirmwareUpdating          = false;
static bool isFirmwareUpdatingCompleted = false;
static int  firmwareUpdateProgress      = 0;

static int  countWebHandler             = 0; 
static int  maxWebHandlerCount          = 16;

// Bounded: a name shorter than the extension used to index before the buffer.
#define IS_FILE_EXT(filename, ext) \
    (strlen(filename) >= sizeof(ext) - 1 && \
     strcasecmp(&(filename)[strlen(filename) - sizeof(ext) + 1], ext) == 0)

// OTA Update Status empfangen
static void handle_ota_events(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data) {

    switch (id) {

        case OTA_EVENT_UPDATE_STARTED:

            isFirmwareUpdating            = true;
            isFirmwareUpdatingCompleted   = false;  
            firmwareUpdateProgress        = 0;

            break;

        case OTA_EVENT_UPDATE_PROGRESS:

            int progress                  = *(int *)event_data;

            isFirmwareUpdating            = true;
            isFirmwareUpdatingCompleted   = false;
            firmwareUpdateProgress        = progress;

            break;

        case OTA_EVENT_UPDATE_COMPLETED:

            isFirmwareUpdating            = false;
            isFirmwareUpdatingCompleted   = true;
            firmwareUpdateProgress        = 0;

            break;

        default:
            break;
    }
}

static esp_err_t www_spiffs_init() {
    ESP_LOGD(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
            .base_path = WWW_PARTITION_PATH,
            .partition_label = WWW_PARTITION_LABEL,
            .max_files = 10,
            .format_if_mount_failed = false
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(WWW_PARTITION_LABEL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Partition size: total: %d, used: %d", total, used);
    return ESP_OK;
}

// Set HTTP response content type according to file extension
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".js")) {
        return httpd_resp_set_type(req, "application/javascript");
    } else if (IS_FILE_EXT(filename, ".css")) {
        return httpd_resp_set_type(req, "text/css");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    } else if (IS_FILE_EXT(filename, ".png")) {
        return httpd_resp_set_type(req, "image/png");
    } else if (IS_FILE_EXT(filename, ".svg")) {
        return httpd_resp_set_type(req, "image/svg+xml");
    } else if (IS_FILE_EXT(filename, ".json")) {
        return httpd_resp_set_type(req, "application/json");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        // Full path string won't fit into destination buffer
        return NULL;
    }

    // Construct full path (base + path)
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    // Return pointer to path, skipping the base
    return dest + base_pathlen;
}

static esp_err_t json_response(httpd_req_t *req, cJSON *root) {
    // Set mime type
    esp_err_t err = httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) return err;

    // Convert to string
    bool success = cJSON_PrintPreallocated(root, buffer, BUFFER_SIZE, false);
    cJSON_Delete(root);
    if (!success) {
        ESP_LOGE(TAG, "Not enough space in buffer to output JSON");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Not enough space in buffer to output JSON");
        return ESP_FAIL;
    }

    // Send as response
    err = httpd_resp_send(req, buffer, strlen(buffer));
    if (err != ESP_OK) return err;

    return ESP_OK;
}

/* "Basic <base64>": the scheme is case-insensitive (RFC 9110), the credential
 * is not. The header is read into a bounded stack buffer: the old code took a
 * missing header for a 1-byte value and compared uninitialised memory. */
static esp_err_t basic_auth(httpd_req_t *req) {
    char header[160];
    size_t len = httpd_req_get_hdr_value_len(req, "Authorization");
    bool authenticated = false;

    if (basic_authentication != NULL && len > 6 && len < sizeof(header) &&
        httpd_req_get_hdr_value_str(req, "Authorization", header, sizeof(header)) == ESP_OK) {
        authenticated = strncasecmp(header, "Basic ", 6) == 0 &&
                        strcmp(header + 6, basic_authentication + 6) == 0;
    }
    if (authenticated) return ESP_OK;

    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"RTKdata Station Config\"");
    httpd_resp_set_status(req, "401"); // Unauthorized
    const char *unauthorized = "401 Unauthorized - Incorrect or no password provided";
    httpd_resp_send(req, unauthorized, strlen(unauthorized));
    return ESP_FAIL;
}

/* Only clients on the setup hotspot: the connection must have arrived on the
 * AP interface, i.e. its LOCAL address is the AP address. The httpd socket is
 * dual-stack when IPv6 is on, so an IPv4 peer shows up IPv4-mapped. This mode
 * used to reject everyone (the check was commented out). */
static esp_err_t hotspot_auth(httpd_req_t *req) {
    int sock = httpd_req_to_sockfd(req);
    struct sockaddr_storage local;
    socklen_t socklen = sizeof(local);
    uint32_t local_ip = 0;

    if (getsockname(sock, (struct sockaddr *)&local, &socklen) == 0) {
        if (local.ss_family == AF_INET) {
            local_ip = ((struct sockaddr_in *)&local)->sin_addr.s_addr;
        }
#if CONFIG_LWIP_IPV6
        else if (local.ss_family == AF_INET6) {
            const uint32_t *a = ((struct sockaddr_in6 *)&local)->sin6_addr.un.u32_addr;
            if (a[0] == 0 && a[1] == 0 && a[2] == PP_HTONL(0x0000FFFFUL)) local_ip = a[3];
        }
#endif
    }

    // The peer as well: lwIP accepts a packet for ANY local address on ANY
    // interface, so a LAN host that routes the AP address via the station's
    // LAN IP reached the AP address from outside (review R6, 2026-11-06).
    // A hotspot client sits on the AP subnet; a LAN host does not.
    struct sockaddr_storage peer;
    socklen_t peerlen = sizeof(peer);
    uint32_t peer_ip = 0;
    if (getpeername(sock, (struct sockaddr *)&peer, &peerlen) == 0) {
        if (peer.ss_family == AF_INET) {
            peer_ip = ((struct sockaddr_in *)&peer)->sin_addr.s_addr;
        }
#if CONFIG_LWIP_IPV6
        else if (peer.ss_family == AF_INET6) {
            const uint32_t *a = ((struct sockaddr_in6 *)&peer)->sin6_addr.un.u32_addr;
            if (a[0] == 0 && a[1] == 0 && a[2] == PP_HTONL(0x0000FFFFUL)) peer_ip = a[3];
        }
#endif
    }

    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t info;
    if (local_ip != 0 && peer_ip != 0 && ap != NULL && esp_netif_get_ip_info(ap, &info) == ESP_OK &&
        info.ip.addr == local_ip &&
        (peer_ip & info.netmask.addr) == (info.ip.addr & info.netmask.addr)) {
        return ESP_OK;
    }

    httpd_resp_set_status(req, "401"); // Unauthorized
    const char *unauthorized = "401 Unauthorized - Configured to only accept connections from hotspot devices";
    httpd_resp_send(req, unauthorized, strlen(unauthorized));
    return ESP_FAIL;
}

static esp_err_t check_auth(httpd_req_t *req) {
    if (auth_method == AUTH_METHOD_HOTSPOT) return hotspot_auth(req);
    if (auth_method == AUTH_METHOD_BASIC) return basic_auth(req);
    return ESP_OK;
}

static esp_err_t log_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    httpd_resp_set_type(req, "text/plain");

    size_t length;
    void *log_data = log_receive(&length, 1);
    if (log_data == NULL) {
        httpd_resp_sendstr(req, "");

        return ESP_OK;
    }

    httpd_resp_send(req, log_data, length);

    log_return(log_data);

    return ESP_OK;
}

static esp_err_t core_dump_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    size_t core_dump_size = core_dump_available();
    if (core_dump_size == 0) {
        httpd_resp_sendstr(req, "No core dump available");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/octet-stream");

    const esp_app_desc_t *app_desc = esp_app_get_description();

    char elf_sha256[7];
    esp_app_get_elf_sha256(elf_sha256, sizeof(elf_sha256));

    time_t t = time(NULL);
    char date[20] = "\0";
    if (t > 315360000l) strftime(date, sizeof(date), "_%F_%T", localtime(&t));

    char content_disposition[128];
    snprintf(content_disposition, sizeof(content_disposition),
            "attachment; filename=\"esp32_xbee_%s_core_dump_%s%s.bin\"", app_desc->version, elf_sha256, date);
    httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);

    for (int offset = 0; offset < core_dump_size; offset += BUFFER_SIZE) {
        size_t read = core_dump_size - offset;
        if (read > BUFFER_SIZE) read = BUFFER_SIZE;

        core_dump_read(offset, buffer, read);
        httpd_resp_send_chunk(req, buffer, read);
    }

    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

static esp_err_t heap_info_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_DEFAULT);

    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "total_free_bytes", info.total_free_bytes);
    cJSON_AddNumberToObject(root, "total_allocated_bytes", info.total_allocated_bytes);
    cJSON_AddNumberToObject(root, "largest_free_block", info.largest_free_block);
    cJSON_AddNumberToObject(root, "minimum_free_bytes", info.minimum_free_bytes);
    cJSON_AddNumberToObject(root, "allocated_blocks", info.allocated_blocks);
    cJSON_AddNumberToObject(root, "free_blocks", info.free_blocks);
    cJSON_AddNumberToObject(root, "total_blocks", info.total_blocks);

    return json_response(req, root);
}

/* ETag = firmware build + file size. The web pages only change with a
 * firmware (or www) update, so this is as good as a content hash, and it
 * needs no state. The old scheme wrote a ".crc" file to flash after EVERY
 * served file (flash wear, and each write stalls the GNSS UART interrupt),
 * and leaked the If-None-Match buffer on every cache check. */
static void file_etag(char *etag, size_t etag_size, long size) {
    char sha[9];
    esp_app_get_elf_sha256(sha, sizeof(sha));
    snprintf(etag, etag_size, "\"%s-%ld\"", sha, size);
}

static bool file_etag_matches(httpd_req_t *req, const char *etag) {
    char inm[40];
    size_t len = httpd_req_get_hdr_value_len(req, "If-None-Match");
    if (len == 0 || len >= sizeof(inm)) return false;
    if (httpd_req_get_hdr_value_str(req, "If-None-Match", inm, sizeof(inm)) != ESP_OK) return false;
    return strcmp(inm, etag) == 0;
}

static esp_err_t file_get_handler(httpd_req_t *req) {

    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

#if !CONFIG_RTK_TEST_PAGE
    // An app-only update keeps the www partition, so a unit that once ran the
    // board test build may still hold test.html. Treat it as missing.
    if (strncmp(req->uri, "/test.html", strlen("/test.html")) == 0) {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
#endif

    char file_path[FILE_PATH_MAX];

    FILE *fd = NULL;
    struct stat file_stat;

    // Extract filename from URL
    char *file_name = get_path_from_uri(file_path, WWW_PARTITION_PATH, req->uri, sizeof(file_path));
    ERROR_ACTION(TAG, file_name == NULL, {
        httpd_resp_send_err(req, HTTPD_414_URI_TOO_LONG, "Filename too long");
        return ESP_FAIL;
    }, "Filename too long")

    // Trailing '/': serve the index page. The bound is the WHOLE path in
    // file_path (base path included); checking only the part after the base
    // let a 36-character directory URL write 8 bytes past the stack buffer.
    size_t name_len = strlen(file_name);
    if (name_len > 0 && file_name[name_len - 1] == '/') {
        if (strlen(file_path) + strlen("index.html") >= sizeof(file_path)) {
            httpd_resp_send_err(req, HTTPD_414_URI_TOO_LONG, "Filename too long");
            return ESP_FAIL;
        }
        strlcat(file_path, "index.html", sizeof(file_path));
    }

    set_content_type_from_file(req, file_name);

    // Check if file exists
    //ERROR_ACTION(TAG, stat(file_path, &file_stat) == -1, {
    //    httpd_resp_send_404(req);
    //    return ESP_FAIL;
    //}, "Could not stat file %s", file_path)

    if(stat(file_path, &file_stat) == -1){

        // file not exist go to the main "/index"
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/");
        httpd_resp_send(req, NULL, 0);

        return ESP_OK;

    }

    char etag[32];
    file_etag(etag, sizeof(etag), (long)file_stat.st_size);

    if (file_etag_matches(req, etag)) {
        httpd_resp_set_status(req, "304 Not Modified");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    httpd_resp_set_hdr(req, "ETag", etag);

    fd = fopen(file_path, "r");

    ERROR_ACTION(TAG, fd == NULL, {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not read file");
        return ESP_FAIL;
    }, "Could not read file %s", file_path)

    ESP_LOGI(TAG, "Sending file %s (%ld bytes)...", file_name, file_stat.st_size);

    size_t length;
    do {
        // Read file in chunks into the scratch buffer
        length = fread(buffer, 1, BUFFER_SIZE, fd);

        // Send the buffer contents as HTTP response chunk
        if (httpd_resp_send_chunk(req, buffer, length) != ESP_OK) {
            ESP_LOGE(TAG, "Failed sending file %s", file_name);
            httpd_resp_sendstr_chunk(req, NULL);

            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");

            fclose(fd);
            return ESP_FAIL;
        }
    } while (length != 0);

    fclose(fd);
    return ESP_OK;
}

static esp_err_t config_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    cJSON *root = cJSON_CreateObject();

    // const esp_app_desc_t *app_desc = esp_app_get_description();
    // cJSON_AddStringToObject(root, "version", app_desc->version);
    cJSON_AddStringToObject(root, "version", "v" FW_VERSION);
    int config_item_count;
    const config_item_t *config_items = config_items_get(&config_item_count);
    for (int i = 0; i < config_item_count; i++) {
        const config_item_t *item = &config_items[i];

        int64_t int64 = 0;
        uint64_t uint64 = 0;

        size_t length = 0;
        char *string = NULL;

        config_color_t color;
        esp_ip4_addr_t ip;

        switch (item->type) {
            case CONFIG_ITEM_TYPE_STRING:
            case CONFIG_ITEM_TYPE_BLOB:
                // Measure, allocate, read. Provisioning may store a longer
                // value in between: NVS then fails with INVALID_LENGTH and
                // REWRITES length, and the old code wrote the terminator past
                // the allocation. Keep the capacity separate and retry.
                for (int attempt = 0; attempt < 3 && string == NULL; attempt++) {
                    length = 0;
                    if (config_get_str_blob(item, NULL, &length) != ESP_OK) break;
                    size_t cap = length;
                    char *s = calloc(1, cap + 1);
                    if (s == NULL) break;
                    size_t got = cap;
                    esp_err_t e = config_get_str_blob(item, s, &got);
                    if (e == ESP_OK && got <= cap) {
                        s[got] = '\0';
                        string = s;
                    } else {
                        free(s);
                        if (e != ESP_ERR_NVS_INVALID_LENGTH) break;
                    }
                }
                break;
            case CONFIG_ITEM_TYPE_COLOR:
                // Convert to hex
                ESP_ERROR_CHECK_WITHOUT_ABORT(config_get_primitive(item, &color));
                asprintf(&string, "#%02x%02x%02x", color.values.red, color.values.green, color.values.blue);
                break;
            case CONFIG_ITEM_TYPE_IP:
                ESP_ERROR_CHECK_WITHOUT_ABORT(config_get_primitive(item, &ip));
                cJSON *ip_parts = cJSON_AddArrayToObject(root, item->key);
                for (int b = 0; b < 4; b++) {
                    cJSON_AddItemToArray(ip_parts, cJSON_CreateNumber(esp_ip4_addr_get_byte(&ip, b)));
                }

                break;
            case CONFIG_ITEM_TYPE_UINT8:
            case CONFIG_ITEM_TYPE_UINT16:
            case CONFIG_ITEM_TYPE_UINT32:
            case CONFIG_ITEM_TYPE_UINT64:
                ESP_ERROR_CHECK_WITHOUT_ABORT(config_get_primitive(item, &uint64));
                asprintf(&string, "%llu", uint64);
                break;
            case CONFIG_ITEM_TYPE_BOOL:
            case CONFIG_ITEM_TYPE_INT8:
            case CONFIG_ITEM_TYPE_INT16:
            case CONFIG_ITEM_TYPE_INT32:
            case CONFIG_ITEM_TYPE_INT64:
                ESP_ERROR_CHECK_WITHOUT_ABORT(config_get_primitive(item, &int64));
                asprintf(&string, "%lld", int64);
                break;
            default:
                string = calloc(1, 1);
                break;
        }

        if (string != NULL) {
            // Hide secret values that aren't empty
            char *value = item->secret && strlen(string) > 0 ? CONFIG_VALUE_UNCHANGED : string;
            cJSON_AddStringToObject(root, item->key, value);

            free(string);
        }
    }

    return json_response(req, root);
}

/* cJSON parses recursively with a nesting limit of 1000; a 4 KB body of '['
 * overflows the 4 KB httpd stack long before that. Our config bodies are one
 * flat object (IP values are one array deep), so reject anything deeper than
 * a few levels before cJSON sees it. Strings are skipped, escapes included. */
#define JSON_MAX_DEPTH 4
static bool json_depth_ok(const char *s) {
    int depth = 0;
    bool in_str = false;
    for (; *s; s++) {
        char c = *s;
        if (in_str) {
            if (c == '\\' && s[1] != '\0') s++;
            else if (c == '"') in_str = false;
        } else if (c == '"') {
            in_str = true;
        } else if (c == '{' || c == '[') {
            if (++depth > JSON_MAX_DEPTH) return false;
        } else if (c == '}' || c == ']') {
            depth--;
        }
    }
    return true;
}

static bool parse_json(cJSON **root, const char *buffer) {
    *root = NULL;
    if (!json_depth_ok(buffer)) {
        ESP_LOGE(TAG, "JSON rejected: nested deeper than %d", JSON_MAX_DEPTH);
        return false;
    }
    *root = cJSON_Parse(buffer);
    if (!*root || !cJSON_IsObject(*root)) {
        ESP_LOGE(TAG, "JSON parsing failed or root is not an object");
        cJSON_Delete(*root);
        *root = NULL;
        return false;
    }
    return true;
}

static bool parse_json_entry(const cJSON *root, const char *key, cJSON **entry) {
    if (!cJSON_HasObjectItem(root, key)) return false;

    *entry = cJSON_GetObjectItem(root, key);
    if (!*entry) {
        ESP_LOGE(TAG, "JSON entry '%s' missing or null", key);
        return false;
    }

    return true;
}

static esp_err_t write_config_value(const config_item_t *item, const cJSON *entry, size_t length){

    esp_err_t err = ESP_OK;

    if (!item || !entry) {

        err = ESP_ERR_INVALID_ARG;

    }else{

        switch (item->type) {

            case CONFIG_ITEM_TYPE_STRING:

                if (!cJSON_IsString(entry) || !entry->valuestring) {
                    
                    err = ESP_ERR_INVALID_ARG;
                    break;
                }

                err = config_set_str(item->key, entry->valuestring);
                break;

            case CONFIG_ITEM_TYPE_BLOB:

                if (!cJSON_IsString(entry) || !entry->valuestring) {
                    
                    err = ESP_ERR_INVALID_ARG;
                    break;
                }

                err = config_set_blob(item->key, entry->valuestring, length);
                break;

            case CONFIG_ITEM_TYPE_COLOR: {

                if (!cJSON_IsString(entry) || !entry->valuestring || strlen(entry->valuestring) != 7 || entry->valuestring[0] != '#') {
                
                    err = ESP_ERR_INVALID_ARG;
                    break;
                }

                config_color_t color;

                bool is_black = strcmp(entry->valuestring, "#000000") == 0;
                color.rgba    = strtoul(entry->valuestring + 1, NULL, 16) << 8u;

                if (!is_black && color.rgba == 0){

                    err = ESP_ERR_INVALID_ARG;

                }else if (!is_black){

                    color.values.alpha = item->def.color.values.alpha;

                    err = config_set_color(item->key, color);
                }

                break;
            }

            case CONFIG_ITEM_TYPE_IP: {

                if (!cJSON_IsArray(entry) || cJSON_GetArraySize(entry) != 4){
                    
                    err = ESP_ERR_INVALID_ARG;
                    break;
                }

                uint8_t a[4];

                bool valid = true;

                for (int i = 0; i < 4; ++i) {

                    cJSON *val = cJSON_GetArrayItem(entry, i);

                    if (!val || !cJSON_IsString(val) || !val->valuestring) {
                        valid = false;
                        break;
                    }

                    errno = 0;
                    long byte = strtol(val->valuestring, NULL, 10);

                    if (errno != 0 || byte < 0 || byte > 255) {
                        valid = false;
                        break;
                    }

                    a[i] = (uint8_t)byte;
                }

                if (valid) {

                    uint32_t ip = esp_netif_htonl(esp_netif_ip4_makeu32(a[0], a[1], a[2], a[3]));
                    err         = config_set_u32(item->key, ip);

                } else {
                    err         = ESP_ERR_INVALID_ARG;
                }

                break;

            }

            case CONFIG_ITEM_TYPE_BOOL:
            case CONFIG_ITEM_TYPE_INT8:
            case CONFIG_ITEM_TYPE_INT16:
            case CONFIG_ITEM_TYPE_INT32:
            case CONFIG_ITEM_TYPE_INT64: {

                if (!cJSON_IsString(entry) || !entry->valuestring) {

                    err = ESP_ERR_INVALID_ARG;
                    break;
                }

                int64_t int64 = strtol(entry->valuestring, NULL, 10);

                if (int64 == 0 && strcmp(entry->valuestring, "0") != 0){ 
                    
                    err = ESP_ERR_INVALID_ARG;

                }else{

                    err = config_set(item, &int64);
                }

                break;

            }

            case CONFIG_ITEM_TYPE_UINT8:
            case CONFIG_ITEM_TYPE_UINT16:
            case CONFIG_ITEM_TYPE_UINT32:
            case CONFIG_ITEM_TYPE_UINT64: {

                if (!cJSON_IsString(entry) || !entry->valuestring) {
                
                    err = ESP_ERR_INVALID_ARG;
                    break;
                }

                uint64_t uint64 = strtoul(entry->valuestring, NULL, 10);

                if (uint64 == 0 && strcmp(entry->valuestring, "0") != 0){
                    
                err =  ESP_ERR_INVALID_ARG;
                
                }else{

                    err = config_set(item, &uint64);
                }

                break;
            
            }

            default:
            
                err = ESP_FAIL;
        }

    }


    return err;
}

static esp_err_t apply_config_value(const config_item_t *item, const cJSON *entry) {

    esp_err_t err = ESP_OK;

    // For non-IP items: value and type validation
    bool   is_ip           = (item->type == CONFIG_ITEM_TYPE_IP);
    bool   is_valid_string = (entry && cJSON_IsString(entry) && entry->valuestring);
    size_t length          = is_valid_string ? strlen(entry->valuestring) : 0;

    // Invalid config item type
    if (item->type > CONFIG_ITEM_TYPE_MAX) {

        err = ESP_ERR_INVALID_ARG;
    }

    // Missing or invalid string for types that require it
    else if (!is_ip && !is_valid_string) {
        
        ESP_LOGE(TAG, "Expected string type for key: %s", item->key);
        err = ESP_ERR_INVALID_ARG;
    }

    // Empty string is allowed only for BLOB or STRING types
    else if (!is_ip && length == 0 && item->type != CONFIG_ITEM_TYPE_BLOB &&
            item->type != CONFIG_ITEM_TYPE_STRING) {

        err = ESP_OK;  // Skip processing
    }

    // Placeholder value indicates no change
    else if (is_valid_string && strcmp(entry->valuestring, CONFIG_VALUE_UNCHANGED) == 0) {

        err = ESP_OK;  // Skip processing
    }

    // Otherwise: proceed with configuration handling
    else {
        
        err = write_config_value(item, entry, length);
    }


    return err;
}

/* The setup page writes only the station Wi-Fi. Everything else (backend
 * host, device identity, station token, enroll key, casters, UART, admin
 * access) comes from USB provisioning or the backend. Accepting every key
 * from the open setup hotspot and from anyone on the LAN allowed redirecting
 * the station token to a foreign host, wiping the enroll key, or storing a
 * UART value that aborts every boot before the boot-loop guard runs. */
static const char *const s_post_writable[] = {
    KEY_CONFIG_WIFI_STA_ACTIVE, KEY_CONFIG_WIFI_STA_SSID, KEY_CONFIG_WIFI_STA_PASSWORD,
    KEY_CONFIG_WIFI_STA_STATIC, KEY_CONFIG_WIFI_STA_IP, KEY_CONFIG_WIFI_STA_GATEWAY,
    KEY_CONFIG_WIFI_STA_SUBNET, KEY_CONFIG_WIFI_STA_DNS_A, KEY_CONFIG_WIFI_STA_DNS_B,
};

static bool post_writable(const char *key) {
    for (size_t i = 0; i < sizeof(s_post_writable) / sizeof(s_post_writable[0]); i++) {
        if (strcmp(key, s_post_writable[i]) == 0) return true;
    }
    return false;
}

static bool decimal_ok(const char *s, long long lo, long long hi) {
    if (s == NULL || *s == '\0') return false;
    char *end = NULL;
    errno = 0;
    long long v = strtoll(s, &end, 10);
    return errno == 0 && end != NULL && *end == '\0' && v >= lo && v <= hi;
}

/* Everything is checked before anything is written, so a bad field cannot
 * leave half a configuration behind. */
static bool post_value_valid(const config_item_t *item, const cJSON *entry) {
    if (cJSON_IsString(entry) && entry->valuestring &&
        strcmp(entry->valuestring, CONFIG_VALUE_UNCHANGED) == 0) {
        return true;
    }
    switch (item->type) {
        case CONFIG_ITEM_TYPE_STRING:
        case CONFIG_ITEM_TYPE_BLOB: {
            if (!cJSON_IsString(entry) || !entry->valuestring) return false;
            size_t n = strlen(entry->valuestring);
            if (strcmp(item->key, KEY_CONFIG_WIFI_STA_SSID) == 0) return n <= 32;
            if (strcmp(item->key, KEY_CONFIG_WIFI_STA_PASSWORD) == 0) return n <= 64;
            return n <= 128;
        }
        case CONFIG_ITEM_TYPE_BOOL:
            return cJSON_IsString(entry) && decimal_ok(entry->valuestring, 0, 1);
        case CONFIG_ITEM_TYPE_UINT8:
            // the only writable UINT8 is the static-IP prefix length
            return strcmp(item->key, KEY_CONFIG_WIFI_STA_SUBNET) == 0 &&
                   cJSON_IsString(entry) && decimal_ok(entry->valuestring, 1, 32);
        case CONFIG_ITEM_TYPE_IP: {
            if (!cJSON_IsArray(entry) || cJSON_GetArraySize(entry) != 4) return false;
            for (int i = 0; i < 4; i++) {
                const cJSON *b = cJSON_GetArrayItem(entry, i);
                if (!cJSON_IsString(b) || !decimal_ok(b->valuestring, 0, 255)) return false;
            }
            return true;
        }
        default:
            return false;   // no other type is writable from the setup page
    }
}

static esp_err_t config_post_reply(httpd_req_t *req, const char *status, bool success, const char *error) {
    if (status) httpd_resp_set_status(req, status);
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "success", success);
    if (error) cJSON_AddStringToObject(resp, "error", error);
    return json_response(req, resp);
}

#define POST_BODY_DEADLINE_US (10LL * 1000 * 1000)

static esp_err_t config_post_handler(httpd_req_t *req) {

    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    // Only application/json: a cross-site form or text/plain POST (no CORS
    // preflight) from any web page the user has open must not reconfigure it.
    char ctype[48];
    if (httpd_req_get_hdr_value_str(req, "Content-Type", ctype, sizeof(ctype)) != ESP_OK ||
        strncasecmp(ctype, "application/json", strlen("application/json")) != 0) {
        return config_post_reply(req, "415 Unsupported Media Type", false, "content type must be application/json");
    }

    // Read the WHOLE body: TCP may deliver it in several pieces.
    size_t total = req->content_len;
    if (total == 0 || total > BUFFER_SIZE - 1) {
        return config_post_reply(req, "413 Payload Too Large", false, "body empty or too large");
    }
    // One absolute deadline for the whole body: the server has a single
    // task, and a client sending a byte every few seconds held it for hours
    // with per-read timeouts only (review R6, 2026-11-06).
    size_t got = 0;
    int timeouts = 0;
    int64_t body_t0 = esp_timer_get_time();
    while (got < total) {
        if (esp_timer_get_time() - body_t0 > POST_BODY_DEADLINE_US) {
            httpd_resp_send_408(req);
            return ESP_FAIL;
        }
        int r = httpd_req_recv(req, buffer + got, total - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 3) continue;
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req);
            return ESP_FAIL;
        }
        got += (size_t)r;
    }
    buffer[got] = '\0';

    cJSON *root = NULL;
    if (!parse_json(&root, buffer)) {
        return config_post_reply(req, "400 Bad Request", false, "malformed JSON");
    }

    int config_item_count = 0;
    const config_item_t *config_items = config_items_get(&config_item_count);

    // Pass 1: validate every field that names a config item.
    int writable = 0;
    for (int i = 0; i < config_item_count; i++) {
        const config_item_t *item = &config_items[i];
        cJSON *entry = NULL;
        if (!parse_json_entry(root, item->key, &entry)) continue;
        if (!post_writable(item->key)) {
            ESP_LOGW(TAG, "config POST: key %s is not writable here", item->key);
            cJSON_Delete(root);
            return config_post_reply(req, "403 Forbidden", false, "key not writable");
        }
        if (!post_value_valid(item, entry)) {
            ESP_LOGW(TAG, "config POST: invalid value for %s", item->key);
            cJSON_Delete(root);
            return config_post_reply(req, "400 Bad Request", false, "invalid value");
        }
        writable++;
    }
    if (writable == 0) {
        cJSON_Delete(root);
        return config_post_reply(req, "400 Bad Request", false, "nothing to change");
    }

    // Pass 2: write.
    bool ok = true;
    for (int i = 0; i < config_item_count; i++) {
        const config_item_t *item = &config_items[i];
        cJSON *entry = NULL;
        if (!parse_json_entry(root, item->key, &entry)) continue;
        esp_err_t err = apply_config_value(item, entry);
        if (err != ESP_OK) {
            ok = false;
            ESP_LOGE(TAG, "Error applying key=%s: err=%d (%s)", item->key, err, esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Applied config key=%s successfully", item->key);
        }
    }
    cJSON_Delete(root);

    if (config_commit() != ESP_OK) ok = false;
    if (!ok) {
        return config_post_reply(req, "500 Internal Server Error", false, "could not store the configuration");
    }

    esp_err_t result = config_post_reply(req, NULL, true, NULL);
    config_restart();
    return result;
}

static esp_err_t status_get_handler(httpd_req_t *req) {

    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    cJSON *root = cJSON_CreateObject();

    // Update status
    cJSON_AddBoolToObject(root,"firmwareupdate", isFirmwareUpdating);
    cJSON_AddNumberToObject(root,"firmwareupdateprogress",firmwareUpdateProgress);
    cJSON_AddBoolToObject(root,"firmwareupdatecompleted",isFirmwareUpdatingCompleted);

    // Uptime
    cJSON_AddNumberToObject(root, "uptime", (int) ((double) esp_timer_get_time() / 1000000));

    // Heap
    cJSON *heap = cJSON_AddObjectToObject(root, "heap");
    cJSON_AddNumberToObject(heap, "total", heap_caps_get_total_size(MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(heap, "free", heap_caps_get_free_size(MALLOC_CAP_8BIT));

    // Streams
    cJSON *streams = cJSON_AddObjectToObject(root, "streams");
    stream_stats_values_t values;
    for (stream_stats_handle_t stats = stream_stats_first(); stats != NULL; stats = stream_stats_next(stats)) {
        stream_stats_values(stats, &values);

        cJSON *stream = cJSON_AddObjectToObject(streams, values.name);
        cJSON *total = cJSON_AddObjectToObject(stream, "total");
        cJSON_AddNumberToObject(total, "in", values.total_in);
        cJSON_AddNumberToObject(total, "out", values.total_out);
        cJSON *rate = cJSON_AddObjectToObject(stream, "rate");
        cJSON_AddNumberToObject(rate, "in", values.rate_in);
        cJSON_AddNumberToObject(rate, "out", values.rate_out);
    }

    // Sockets
    cJSON *sockets = cJSON_AddArrayToObject(root, "sockets");
    for (int s = LWIP_SOCKET_OFFSET; s < LWIP_SOCKET_OFFSET + CONFIG_LWIP_MAX_SOCKETS; s++) {
        int err;

        int socktype;
        socklen_t socktype_len = sizeof(socktype);
        err = getsockopt(s, SOL_SOCKET, SO_TYPE, &socktype, &socktype_len);
        if (err < 0) continue;

        cJSON *socket = cJSON_CreateObject();

        cJSON_AddStringToObject(socket, "type", SOCKTYPE_NAME(socktype));

        struct sockaddr_in6 addr;
        socklen_t socklen = sizeof(addr);

        err = getsockname(s, (struct sockaddr *)&addr, &socklen);
        if (err == 0) cJSON_AddStringToObject(socket, "local", sockaddrtostr((struct sockaddr *) &addr));

        err = getpeername(s, (struct sockaddr *)&addr, &socklen);
        if (err == 0) cJSON_AddStringToObject(socket, "peer", sockaddrtostr((struct sockaddr *) &addr));

        cJSON_AddItemToArray(sockets, socket);
    }

    // WiFi
    wifi_ap_status_t ap_status;
    wifi_sta_status_t sta_status;

    wifi_ap_status(&ap_status);
    wifi_sta_status(&sta_status);

    cJSON *wifi = cJSON_AddObjectToObject(root, "wifi");

    cJSON *ap = cJSON_AddObjectToObject(wifi, "ap");
    cJSON_AddBoolToObject(ap, "active", ap_status.active);

    if (ap_status.active) {

        cJSON_AddStringToObject(ap, "ssid", (char *) ap_status.ssid);
        cJSON_AddStringToObject(ap, "authmode", wifi_auth_mode_name(ap_status.authmode));
        cJSON_AddNumberToObject(ap, "devices", ap_status.devices);

        char ip[40];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ap_status.ip4_addr));
        cJSON_AddStringToObject(ap, "ip4", ip);
        snprintf(ip, sizeof(ip), IPV6STR, IPV62STR(ap_status.ip6_addr));
        cJSON_AddStringToObject(ap, "ip6", ip);
    }

    cJSON *sta = cJSON_AddObjectToObject(wifi, "sta");
    cJSON_AddBoolToObject(sta, "active", sta_status.active);

    if (sta_status.active) {

        cJSON_AddBoolToObject(sta, "connected", sta_status.connected);

        if (sta_status.connected) {
            
            cJSON_AddStringToObject(sta, "ssid", (char *) sta_status.ssid);
            cJSON_AddStringToObject(sta, "authmode", wifi_auth_mode_name(sta_status.authmode));
            cJSON_AddNumberToObject(sta, "rssi", sta_status.rssi);

            char ip[40];
            snprintf(ip, sizeof(ip), IPSTR, IP2STR(&sta_status.ip4_addr));
            cJSON_AddStringToObject(sta, "ip4", ip);
            snprintf(ip, sizeof(ip), IPV6STR, IPV62STR(sta_status.ip6_addr));
            cJSON_AddStringToObject(sta, "ip6", ip);
        }
    }

#if BOARD_HAS_ETH
    {
        eth_status_t es;
        eth_status(&es);

        cJSON *eth = cJSON_AddObjectToObject(root, "eth");
        cJSON_AddBoolToObject(eth, "present", es.present);
        cJSON_AddBoolToObject(eth, "link", es.link);
        cJSON_AddBoolToObject(eth, "connected", es.has_ip);
        cJSON_AddNumberToObject(eth, "speed", es.speed_mbps);
        cJSON_AddBoolToObject(eth, "full_duplex", es.full_duplex);
        cJSON_AddNumberToObject(eth, "link_ups", es.link_ups);
        cJSON_AddNumberToObject(eth, "symbol_errors", es.symbol_errors);

        char buf[40];
        snprintf(buf, sizeof(buf), MACSTR, MAC2STR(es.mac));
        cJSON_AddStringToObject(eth, "mac", buf);
        if (es.has_ip) {
            snprintf(buf, sizeof(buf), IPSTR, IP2STR(&es.ip4_addr));
            cJSON_AddStringToObject(eth, "ip4", buf);
            snprintf(buf, sizeof(buf), IPSTR, IP2STR(&es.gw4_addr));
            cJSON_AddStringToObject(eth, "gw4", buf);
        }
        cJSON_AddBoolToObject(sta, "parked", sta_status.suspended);
    }
#endif

    // Which interface did THIS request arrive on: the captive SoftAP (setup)
    // or the LAN/STA side (dashboard)? The UI serves the onboarding wizard on
    // the AP and a status dashboard on the LAN IP. Detection is by the request
    // socket's LOCAL address: a request hitting the SoftAP gateway IP is "ap".
    {
        const char *via = "sta";
        int reqsock = httpd_req_to_sockfd(req);
        struct sockaddr_in6 la;
        socklen_t lalen = sizeof(la);
        if (ap_status.active && reqsock >= 0 &&
            getsockname(reqsock, (struct sockaddr *)&la, &lalen) == 0) {
            char apip[40];
            snprintf(apip, sizeof(apip), IPSTR, IP2STR(&ap_status.ip4_addr));
            const char *local = sockaddrtostr((struct sockaddr *)&la);
            if (local != NULL && strstr(local, apip) != NULL) via = "ap";
        }
        cJSON_AddStringToObject(root, "via", via);
    }

    return json_response(req, root);
}

static esp_err_t wifi_scan_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    uint16_t ap_count;
    wifi_ap_record_t *ap_records =  wifi_scan(&ap_count);
    if(ap_records == NULL)
    {
        ESP_LOGE(TAG,"no wifi found");
        return ESP_OK;
    }
    cJSON *root = cJSON_CreateArray();
    for (int i = 0; i < ap_count; i++) {
        wifi_ap_record_t *ap_record = &ap_records[i];
        cJSON *ap = cJSON_CreateObject();
        cJSON_AddItemToArray(root, ap);
        cJSON_AddStringToObject(ap, "ssid", (char *) ap_record->ssid);
        cJSON_AddNumberToObject(ap, "rssi", ap_record->rssi);
        cJSON_AddStringToObject(ap, "authmode", wifi_auth_mode_name(ap_record->authmode));
        ESP_LOGI(TAG,"ssid:%s",ap_record->ssid);
    }

    free(ap_records);

    return json_response(req, root);
}

static esp_err_t rtk_status_get_handler(httpd_req_t *req) {
    // Carries the station coordinate and identity: same access rule as /config.
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;
    cJSON *root = cJSON_CreateObject();
    provisioning_fill_status(root);
    return json_response(req, root);
}

// NTRIP send-path diagnostics (parser/ring/per-instance sender counters).
// Counter names are deliberate: accepted_to_lwip is what the stack took,
// not what reached the wire.
static esp_err_t ntrip_tx_stats_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    ntrip_ingest_stats_t ing;
    ntrip_server_ingest_stats(&ing);

    static ntrip_tx_stats_t txs[NTRIP_MAX_INSTANCES];
    size_t n = ntrip_server_tx_stats(txs, NTRIP_MAX_INSTANCES);

    cJSON *root = cJSON_CreateObject();

    cJSON *ingest = cJSON_AddObjectToObject(root, "ingest");
    cJSON_AddNumberToObject(ingest, "frames_ok", (double)ing.frames_ok);
    cJSON_AddNumberToObject(ingest, "bytes_ok", (double)ing.bytes_ok);
    cJSON_AddNumberToObject(ingest, "bytes_discarded", (double)ing.bytes_discarded);
    cJSON_AddNumberToObject(ingest, "crc_errors", (double)ing.crc_errors);
    cJSON_AddNumberToObject(ingest, "ring_pushed_frames", (double)ing.ring_pushed_frames);
    cJSON_AddNumberToObject(ingest, "ring_cum_bytes", (double)ing.ring_cum_bytes);
    cJSON_AddNumberToObject(ingest, "uart_event_post_drops", uart_event_post_drops());
    cJSON_AddNumberToObject(ingest, "uart_read_errors", uart_read_errors());
    cJSON_AddNumberToObject(ingest, "uart_rx_overflows", uart_rx_overflows());
    cJSON_AddNumberToObject(ingest, "uart_rx_line_errors", uart_rx_line_errors());

    // Required-set freshness: which production messages the receiver is
    // actually producing right now, per type. "any CRC-valid frame" is not
    // health (review 2026-08-03).
    ntrip_msg_freshness_t fresh;
    ntrip_server_msg_freshness(&fresh);
    cJSON *reqages = cJSON_AddObjectToObject(ingest, "required_age_ms");
    for (int i = 0; i < NTRIP_REQUIRED_MSG_COUNT; i++) {
        char key[8];
        snprintf(key, sizeof(key), "%u", (unsigned)fresh.type[i]);
        if (fresh.age_ms[i] == UINT32_MAX) {
            cJSON_AddNullToObject(reqages, key);
        } else {
            cJSON_AddNumberToObject(reqages, key, fresh.age_ms[i]);
        }
    }
    cJSON_AddBoolToObject(ingest, "required_all_fresh", fresh.all_fresh);

    cJSON *arr = cJSON_AddArrayToObject(root, "instances");
    for (size_t i = 0; i < n; i++) {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "index", txs[i].index);
        cJSON_AddBoolToObject(o, "connected", txs[i].connected);
        cJSON_AddNumberToObject(o, "accepted_to_lwip", (double)txs[i].accepted_to_lwip);
        cJSON_AddNumberToObject(o, "sent_frames", (double)txs[i].sent_frames);
        cJSON_AddNumberToObject(o, "copied_bytes", (double)txs[i].copied_bytes);
        cJSON_AddNumberToObject(o, "dropped_bytes", (double)txs[i].dropped_bytes);
        cJSON_AddNumberToObject(o, "skipped_bytes", (double)txs[i].skipped_bytes);
        cJSON_AddNumberToObject(o, "skipped_frames", txs[i].skipped_frames);
        cJSON_AddNumberToObject(o, "dropped_frames_stale", txs[i].dropped_frames_stale);
        cJSON_AddNumberToObject(o, "dropped_stale_bytes", (double)txs[i].dropped_stale_bytes);
        cJSON_AddNumberToObject(o, "eagain_count", (double)txs[i].eagain_count);
        cJSON_AddNumberToObject(o, "reconnects", txs[i].reconnects);
        cJSON_AddNumberToObject(o, "pending", txs[i].pending);
        cJSON_AddNumberToObject(o, "max_queue_age_ms", txs[i].max_queue_age_ms);
        cJSON_AddNumberToObject(o, "last_sock_errno", txs[i].last_sock_errno);
        cJSON_AddItemToArray(arr, o);
    }

    return json_response(req, root);
}

#if CONFIG_RTK_TEST_PAGE
static esp_err_t test_status_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;
    return board_test_status_handler(req);
}

static esp_err_t test_do_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;
    return board_test_do_handler(req);
}
#endif

static esp_err_t register_uri_handler(httpd_handle_t server, const char *path, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *r)) {
    httpd_uri_t uri_config_get = {
            .uri        = path,
            .method     = method,
            .handler    = handler
    };
    return httpd_register_uri_handler(server, &uri_config_get);
}

static httpd_handle_t web_server_start(void)
{
    // register the ota event handler
    esp_event_handler_register_with(ota_event_loop, OTA_EVENT, ESP_EVENT_ANY_ID, handle_ota_events, NULL);     

    config_get_primitive(CONF_ITEM(KEY_CONFIG_ADMIN_AUTH), &auth_method);
    if (auth_method == AUTH_METHOD_BASIC) {
        char *username, *password;
        config_get_str_blob_alloc(CONF_ITEM(KEY_CONFIG_ADMIN_USERNAME), (void **) &username);
        config_get_str_blob_alloc(CONF_ITEM(KEY_CONFIG_ADMIN_PASSWORD), (void **) &password);
        basic_authentication = http_auth_basic_header(username, password);
        free(username);
        free(password);
    }

    httpd_handle_t server = NULL;

    // The shared response buffer must exist BEFORE httpd runs a handler: it
    // used to be allocated after httpd_start(), so an early request (or a
    // failed allocation) used a NULL buffer. PSRAM when there is one: it is
    // only used for file reads and JSON output, and internal RAM is the
    // scarce pool on the Ethernet board.
    if (buffer == NULL) {
        buffer = heap_caps_malloc_prefer(BUFFER_SIZE, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT);
    }
    if (buffer == NULL) {
        ESP_LOGE(TAG, "no memory for the response buffer, web server not started");
        return NULL;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.uri_match_fn     = httpd_uri_match_wildcard;
    config.max_uri_handlers = maxWebHandlerCount;
    // lwIP has 10 sockets and the web server keeps two of its own. The
    // default 7 client slots let one browser with keep-alive connections take
    // the sockets the caster upload, the IE heartbeat and OTA need (review
    // 13.11. L1, R6). Four slots, the least recently used one is closed when
    // a fifth client connects, so a page load never locks the uplink out.
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;

    // Start the http server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK) {

        register_uri_handler(server, "/config", HTTP_GET, config_get_handler);
        countWebHandler++;

        register_uri_handler(server, "/config", HTTP_POST, config_post_handler);
        countWebHandler++;

        register_uri_handler(server, "/status", HTTP_GET, status_get_handler);
        countWebHandler++;

        register_uri_handler(server, "/log", HTTP_GET, log_get_handler);
        countWebHandler++;

        register_uri_handler(server, "/core_dump", HTTP_GET, core_dump_get_handler);
        countWebHandler++;

        register_uri_handler(server, "/heap_info", HTTP_GET, heap_info_get_handler);
        countWebHandler++;
        
        register_uri_handler(server, "/wifi/scan", HTTP_GET, wifi_scan_get_handler);
        countWebHandler++;

        register_uri_handler(server, "/rtk/status", HTTP_GET, rtk_status_get_handler);
        countWebHandler++;

        register_uri_handler(server, "/ntrip/tx_stats", HTTP_GET, ntrip_tx_stats_get_handler);
        countWebHandler++;

#if CONFIG_RTK_TEST_PAGE
        // Board test build only (-DRTK_TEST_PAGE=1), before the wildcard.
        register_uri_handler(server, "/test/status", HTTP_GET, test_status_get_handler);
        countWebHandler++;

        register_uri_handler(server, "/test/do", HTTP_GET, test_do_get_handler);
        countWebHandler++;
#endif

        register_uri_handler(server, "/*", HTTP_GET, file_get_handler);
        countWebHandler++;

    }

    if (server == NULL) {
        ESP_LOGE(TAG, "Could not start server");
        return NULL;
    }
    return server;
}

void check_count_webhandler(){

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if(countWebHandler > maxWebHandlerCount){

        ESP_LOGE(TAG, "too many registered webhandlers, %d < %d", config.max_uri_handlers, countWebHandler);
    } 

}

void web_server_init() {
    www_spiffs_init();
    web_server_start();
}