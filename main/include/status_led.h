#ifndef ESP32_XBEE_STATUS_LED_H
#define ESP32_XBEE_STATUS_LED_H

#include <sys/queue.h>
#include <stdint.h>     //makeItCompileable
#include <stdbool.h>    //makeItCompileable

typedef enum {
    STATUS_LED_STATIC = 0,
    STATUS_LED_FADE,
    STATUS_LED_BLINK
} status_led_flashing_mode_t;

typedef struct status_led_color_t *status_led_handle_t;
struct status_led_color_t {
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    status_led_flashing_mode_t flashing_mode;
    uint32_t interval;
    uint32_t duration;
    uint8_t expire;

    bool remove;
    bool active;

    SLIST_ENTRY(status_led_color_t) next;
};

status_led_handle_t status_led_add(uint32_t rgba, status_led_flashing_mode_t flashing_mode, uint32_t interval, uint32_t duration, uint8_t expire);
void status_led_remove(status_led_handle_t color);
void status_led_init();

// Ethernet board: start the task that derives the two LEDs from supervisor and
// uplink state. Call once the supervisor runs. No-op on the RGB (OnoLink) board.
void status_led_board_start(void);

// Board test page (Ethernet boards): force a firmware LED to a pattern.
// led: 0 = NTRIP, 1 = second LED (Wi-Fi on V4.1, status on V4.0).
// mode: -1 = back to the state-derived pattern, 0 off, 1 on, 2 slow, 3 fast.
// No-op on the RGB (OnoLink) board.
void status_led_test_set(int led, int mode);
int  status_led_test_get(int led);

// wifi leds
void rssi_led_set(uint8_t value);
void rssi_led_fade(uint8_t value, int max_fade_time_ms);
void rssi_led_blink();

#endif //ESP32_XBEE_STATUS_LED_H
