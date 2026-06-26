#ifndef ESP32_XBEE_WIFI_H
#define ESP32_XBEE_WIFI_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_wifi.h>
#include <esp_netif_ip_addr.h>

#ifndef WIFI_SSID_STRLEN_MAX
#define WIFI_SSID_STRLEN_MAX 32
#endif

#ifndef WIFI_SSID_STRLEN_BUF
#define WIFI_SSID_STRLEN_BUF (WIFI_SSID_STRLEN_MAX + 1)
#endif

#ifndef WIFI_SCAN_MAX_APS
#define WIFI_SCAN_MAX_APS 32
#endif

typedef struct wifi_ap_status {

    bool active;                          

    char ssid[WIFI_SSID_STRLEN_BUF];      
    wifi_auth_mode_t authmode;            
    uint8_t devices;                      

    esp_ip4_addr_t ip4_addr;              
    esp_ip6_addr_t ip6_addr;              

} wifi_ap_status_t;

typedef struct wifi_sta_status {

    bool active;                          
    bool connected;                       

    char ssid[WIFI_SSID_STRLEN_BUF];      

    wifi_auth_mode_t authmode;  

    int8_t rssi;                          

    esp_ip4_addr_t ip4_addr;              
    esp_ip6_addr_t ip6_addr;              

} wifi_sta_status_t;

void net_init();

void wifi_init();

wifi_ap_record_t * wifi_scan(uint16_t *number);

wifi_sta_list_t *wifi_ap_sta_list();

void wifi_ap_status(wifi_ap_status_t *status);

void wifi_sta_status(wifi_sta_status_t *status);

bool wait_for_ip(uint32_t timeout_ms);   // bounded; true if STA got an IP, false on timeout (0 = forever)

void wifi_driver_restart(void);

const char * wifi_auth_mode_name(wifi_auth_mode_t auth_mode);

#endif //ESP32_XBEE_WIFI_H
