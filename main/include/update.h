#ifndef ESP32_XBEE_UPDATE_H
#define ESP32_XBEE_UPDATE_H

#include "cJSON.h"
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(OTA_EVENT);

void ota_update_init();
void ota_boot_check(void);
void ota_check_newupdate(void *pvParameter);
void ota_schedule_check_newupdate(void *pvParameter);


typedef enum {
    OTA_EVENT_UPDATE_STARTED,
    OTA_EVENT_UPDATE_PROGRESS,
    OTA_EVENT_UPDATE_COMPLETED,
} ota_event_t;

// Event Loop Handle
extern esp_event_loop_handle_t ota_event_loop;

#endif //ESP32_XBEE_UPDATE_H
