#ifndef ESP32_XBEE_CONFIG_H
#define ESP32_XBEE_CONFIG_H

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    CONFIG_ITEM_TYPE_BOOL = 0,
    CONFIG_ITEM_TYPE_INT8,
    CONFIG_ITEM_TYPE_INT16,
    CONFIG_ITEM_TYPE_INT32,
    CONFIG_ITEM_TYPE_INT64,
    CONFIG_ITEM_TYPE_UINT8,
    CONFIG_ITEM_TYPE_UINT16,
    CONFIG_ITEM_TYPE_UINT32,
    CONFIG_ITEM_TYPE_UINT64,
    CONFIG_ITEM_TYPE_STRING,
    CONFIG_ITEM_TYPE_BLOB,
    CONFIG_ITEM_TYPE_COLOR,
    CONFIG_ITEM_TYPE_IP,
    CONFIG_ITEM_TYPE_MAX
} config_item_type_t;

typedef union {
    struct values {
        uint8_t alpha;
        uint8_t blue;
        uint8_t green;
        uint8_t red;
    } values;
    uint32_t rgba;
} config_color_t;

typedef union {
    bool bool1;
    int8_t int8;
    int16_t int16;
    int32_t int32;
    int64_t int64;
    uint8_t uint8;
    uint16_t uint16;
    uint32_t uint32;
    uint64_t uint64;
    config_color_t color;
    char *str;
    struct blob {
        uint8_t *data;
        size_t length;
    } blob;

} config_item_value_t;

typedef struct config_item {
    char *key;
    config_item_type_t type;
    bool secret;
    config_item_value_t def;
} config_item_t;

#define CONFIG_VALUE_UNCHANGED "\x1a\x1a\x1a\x1a\x1a\x1a\x1a\x1a"

// Admin
#define KEY_CONFIG_ADMIN_AUTH "adm_auth"
#define KEY_CONFIG_ADMIN_USERNAME "adm_user"
#define KEY_CONFIG_ADMIN_PASSWORD "adm_pass"

// Bluetooth
#define KEY_CONFIG_BLUETOOTH_ACTIVE "bt_active"
#define KEY_CONFIG_BLUETOOTH_DEVICE_NAME "bt_dev_name"
#define KEY_CONFIG_BLUETOOTH_DEVICE_DISCOVERABLE "bt_dev_vis"
#define KEY_CONFIG_BLUETOOTH_PIN_CODE "bt_pin_code"

// NTRIP
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE        "ntr_srv_actv"
#define KEY_CONFIG_NTRIP_SERVER_COLOR         "ntr_srv_color"
#define KEY_CONFIG_NTRIP_SERVER_HOST          "ntr_srv_host"
#define KEY_CONFIG_NTRIP_SERVER_PORT          "ntr_srv_port"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT    "ntr_srv_mp"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME      "ntr_srv_user"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD      "ntr_srv_pass"

// --------------------
// NTRIP Server 1
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_1      "ntr_srv_actv_1"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_1       "ntr_srv_color_1"
#define KEY_CONFIG_NTRIP_SERVER_HOST_1        "ntr_srv_host_1"
#define KEY_CONFIG_NTRIP_SERVER_PORT_1        "ntr_srv_port_1"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_1  "ntr_srv_mp_1"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_1    "ntr_srv_user_1"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_1    "ntr_srv_pass_1"

// --------------------
// NTRIP Server 2
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_2      "ntr_srv_actv_2"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_2       "ntr_srv_color_2"
#define KEY_CONFIG_NTRIP_SERVER_HOST_2        "ntr_srv_host_2"
#define KEY_CONFIG_NTRIP_SERVER_PORT_2        "ntr_srv_port_2"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_2  "ntr_srv_mp_2"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_2    "ntr_srv_user_2"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_2    "ntr_srv_pass_2"

// --------------------
// NTRIP Server 3
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_3      "ntr_srv_actv_3"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_3       "ntr_srv_color_3"
#define KEY_CONFIG_NTRIP_SERVER_HOST_3        "ntr_srv_host_3"
#define KEY_CONFIG_NTRIP_SERVER_PORT_3        "ntr_srv_port_3"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_3  "ntr_srv_mp_3"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_3    "ntr_srv_user_3"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_3    "ntr_srv_pass_3"

// --------------------
// NTRIP Server 4
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_4      "ntr_srv_actv_4"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_4       "ntr_srv_color_4"
#define KEY_CONFIG_NTRIP_SERVER_HOST_4        "ntr_srv_host_4"
#define KEY_CONFIG_NTRIP_SERVER_PORT_4        "ntr_srv_port_4"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_4  "ntr_srv_mp_4"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_4    "ntr_srv_user_4"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_4    "ntr_srv_pass_4"

// --------------------
// NTRIP Server 5
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_5      "ntr_srv_actv_5"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_5       "ntr_srv_color_5"
#define KEY_CONFIG_NTRIP_SERVER_HOST_5        "ntr_srv_host_5"
#define KEY_CONFIG_NTRIP_SERVER_PORT_5        "ntr_srv_port_5"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_5  "ntr_srv_mp_5"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_5    "ntr_srv_user_5"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_5    "ntr_srv_pass_5"

// --------------------
// NTRIP Server 6
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_6      "ntr_srv_actv_6"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_6       "ntr_srv_color_6"
#define KEY_CONFIG_NTRIP_SERVER_HOST_6        "ntr_srv_host_6"
#define KEY_CONFIG_NTRIP_SERVER_PORT_6        "ntr_srv_port_6"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_6  "ntr_srv_mp_6"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_6    "ntr_srv_user_6"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_6    "ntr_srv_pass_6"

// --------------------
// NTRIP Server 7
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_7      "ntr_srv_actv_7"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_7       "ntr_srv_color_7"
#define KEY_CONFIG_NTRIP_SERVER_HOST_7        "ntr_srv_host_7"
#define KEY_CONFIG_NTRIP_SERVER_PORT_7        "ntr_srv_port_7"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_7  "ntr_srv_mp_7"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_7    "ntr_srv_user_7"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_7    "ntr_srv_pass_7"

// --------------------
// NTRIP Server 8
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_8      "ntr_srv_actv_8"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_8       "ntr_srv_color_8"
#define KEY_CONFIG_NTRIP_SERVER_HOST_8        "ntr_srv_host_8"
#define KEY_CONFIG_NTRIP_SERVER_PORT_8        "ntr_srv_port_8"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_8  "ntr_srv_mp_8"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_8    "ntr_srv_user_8"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_8    "ntr_srv_pass_8"

// --------------------
// NTRIP Server 9
// --------------------
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE_9      "ntr_srv_actv_9"
#define KEY_CONFIG_NTRIP_SERVER_COLOR_9       "ntr_srv_color_9"
#define KEY_CONFIG_NTRIP_SERVER_HOST_9        "ntr_srv_host_9"
#define KEY_CONFIG_NTRIP_SERVER_PORT_9        "ntr_srv_port_9"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT_9  "ntr_srv_mp_9"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME_9    "ntr_srv_user_9"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD_9    "ntr_srv_pass_9"

// Socket
#define KEY_CONFIG_SOCKET_SERVER_ACTIVE "sck_srv_active"
#define KEY_CONFIG_SOCKET_SERVER_COLOR "sck_srv_color"
#define KEY_CONFIG_SOCKET_SERVER_TCP_PORT "sck_srv_t_port"
#define KEY_CONFIG_SOCKET_SERVER_UDP_PORT "sck_srv_u_port"

#define KEY_CONFIG_SOCKET_CLIENT_ACTIVE "sck_cli_active"
#define KEY_CONFIG_SOCKET_CLIENT_COLOR "sck_cli_color"
#define KEY_CONFIG_SOCKET_CLIENT_HOST "sck_cli_host"
#define KEY_CONFIG_SOCKET_CLIENT_PORT "sck_cli_port"
#define KEY_CONFIG_SOCKET_CLIENT_TYPE_TCP_UDP "sck_cli_type"
#define KEY_CONFIG_SOCKET_CLIENT_CONNECT_MESSAGE "sck_cli_msg"

// UART
#define KEY_CONFIG_UART_NUM "uart_num"
#define KEY_CONFIG_UART_TX_PIN "uart_tx_pin"
#define KEY_CONFIG_UART_RX_PIN "uart_rx_pin"
#define KEY_CONFIG_UART_RTS_PIN "uart_rts_pin"
#define KEY_CONFIG_UART_CTS_PIN "uart_cts_pin"
#define KEY_CONFIG_UART_BAUD_RATE "uart_baud_rate"
#define KEY_CONFIG_UART_DATA_BITS "uart_data_bits"
#define KEY_CONFIG_UART_STOP_BITS "uart_stop_bits"
#define KEY_CONFIG_UART_PARITY "uart_parity"
#define KEY_CONFIG_UART_FLOW_CTRL_RTS "uart_fc_rts"
#define KEY_CONFIG_UART_FLOW_CTRL_CTS "uart_fc_cts"
#define KEY_CONFIG_UART_LOG_FORWARD "uart_log_fwd"

// WiFi
#define KEY_CONFIG_WIFI_AP_ACTIVE "w_ap_active"
#define KEY_CONFIG_WIFI_AP_COLOR "w_ap_color"
#define KEY_CONFIG_WIFI_AP_SSID "w_ap_ssid"
#define KEY_CONFIG_WIFI_AP_SSID_HIDDEN "w_ap_ssid_hid"
#define KEY_CONFIG_WIFI_AP_AUTH_MODE "w_ap_auth_mode"
#define KEY_CONFIG_WIFI_AP_PASSWORD "w_ap_pass"
#define KEY_CONFIG_WIFI_AP_GATEWAY "w_ap_gw"
#define KEY_CONFIG_WIFI_AP_SUBNET "w_ap_subnet"

#define KEY_CONFIG_WIFI_STA_ACTIVE "w_sta_active"
#define KEY_CONFIG_WIFI_STA_COLOR "w_sta_color"
#define KEY_CONFIG_WIFI_STA_SSID "w_sta_ssid"
#define KEY_CONFIG_WIFI_STA_PASSWORD "w_sta_pass"
#define KEY_CONFIG_WIFI_STA_SCAN_MODE_ALL "w_sta_scan_mode"
#define KEY_CONFIG_WIFI_STA_AP_FORWARD "w_sta_ap_fwd"
#define KEY_CONFIG_WIFI_STA_STATIC "w_sta_static"
#define KEY_CONFIG_WIFI_STA_IP "w_sta_ip"
#define KEY_CONFIG_WIFI_STA_GATEWAY "w_sta_gw"
#define KEY_CONFIG_WIFI_STA_SUBNET "w_sta_subnet"
#define KEY_CONFIG_WIFI_STA_DNS_A "w_sta_dns_a"
#define KEY_CONFIG_WIFI_STA_DNS_B "w_sta_dns_b"

// STATE
#define KEY_CONFIG_MODE_ACTIVE "mode_active"
#define KEY_CONFIG_MODE_COLOR  "mode_color"
#define KEY_CONFIG_MODE_PASSWORD "mode_pass"

// RTKdata provisioning (IE enrollment + state machine)
#define KEY_RTK_IE_HOST "rtk_ie_host"
#define KEY_RTK_DEVICE_ID "rtk_dev_id"
#define KEY_RTK_STATION_TOK "rtk_sta_tok"
#define KEY_RTK_STATE "rtk_state"
#define KEY_RTK_OTA_BASE "rtk_ota_base"
// Per-device enroll key (64-hex), provisioned into NVS at flash time from a
// server-only master. Replaces the baked fleet secret. See
// docs/enroll-per-device-key.md. NEVER in the .bin / repo.
#define KEY_RTK_ENROLL_KEY "rtk_enr_key"
#define KEY_RTK_KEY_VER "rtk_key_ver"

// Bump on every OTA release. update.c compares this against ota/release.json's
// "version" with an EXACT strcmp, so the published build MUST self-report the
// same version the manifest carries, otherwise the device re-flashes in a loop.
// release.yml sets the manifest version from the git tag (v1.0.1 -> "1.0.1"),
// so keep this in lockstep with the tag you push.
#define FW_VERSION "1.0.4"

#define UPDATE_SERVER_URL "https://raw.githubusercontent.com/Kansi-jonas/rtkdata-fw/main/ota/"
#define VERSION_FILE "release.json"

#define WWW_PARTITION_PATH      "/www"
#define WWW_PARTITION_LABEL     "www"
#define SPIFFS_PARTITION_PATH   "/spiffs"
#define SPIFFS_PARTITION_LABEL  "spiffs"
#define WWW_FINAL_FILE          "www.bin"

esp_err_t config_init();
esp_err_t config_reset();

const config_item_t *config_items_get(int *count);
const config_item_t * config_get_item(const char *key);

#define CONF_ITEM( key ) config_get_item(key)

bool config_get_bool1(const config_item_t *item);
int8_t config_get_i8(const config_item_t *item);
int16_t config_get_i16(const config_item_t *item);
int32_t config_get_i32(const config_item_t *item);
int64_t config_get_i64(const config_item_t *item);
uint8_t config_get_u8(const config_item_t *item);
uint16_t config_get_u16(const config_item_t *item);
uint32_t config_get_u32(const config_item_t *item);
uint64_t config_get_u64(const config_item_t *item);
config_color_t config_get_color(const config_item_t *item);

esp_err_t config_set(const config_item_t *item, void *value);
esp_err_t config_set_bool1(const char *key, bool value);
esp_err_t config_set_i8(const char *key, int8_t value);
esp_err_t config_set_i16(const char *key, int16_t value);
esp_err_t config_set_i32(const char *key, int32_t value);
esp_err_t config_set_i64(const char *key, int64_t value);
esp_err_t config_set_u8(const char *key, uint8_t value);
esp_err_t config_set_u16(const char *key, uint16_t value);
esp_err_t config_set_u32(const char *key, uint32_t value);
esp_err_t config_set_u64(const char *key, uint64_t value);
esp_err_t config_set_color(const char *key, config_color_t value);
esp_err_t config_set_str(const char *key, char *value);
esp_err_t config_set_blob(const char *key, char *value, size_t length);

esp_err_t config_get_str_blob_alloc(const config_item_t *item, void **out_value);
esp_err_t config_get_str_blob(const config_item_t *item, void *out_value, size_t *length);
esp_err_t config_get_primitive(const config_item_t *item, void *out_value);

esp_err_t config_commit();
void config_restart();

#endif //ESP32_XBEE_CONFIG_H
