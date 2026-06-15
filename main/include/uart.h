#ifndef ESP32_XBEE_UART_H
#define ESP32_XBEE_UART_H

#include <esp_event.h>

ESP_EVENT_DECLARE_BASE(UART_EVENT_READ);
ESP_EVENT_DECLARE_BASE(UART_EVENT_WRITE);

#define UART_BUFFER_SIZE 4096

void uart_init();

int uart_log(char *buffer, size_t len);
int uart_nmea(const char *fmt, ...);
int uart_write(char *buffer, size_t len);
// for gnss main uart read or write action
void uart_register_read_handler(esp_event_handler_t event_handler);
void uart_register_write_handler(esp_event_handler_t event_handler);


typedef esp_err_t (* drv_uart_read_cb_t)(uint8_t *data, const uint16_t len);

extern esp_err_t drv_uart_usbc_init(void);
extern esp_err_t drv_uart_gnss_init(void);

extern esp_err_t drv_uart_usbc_read_set_cb(const drv_uart_read_cb_t cb);
extern int drv_uart_usbc_send(const uint8_t *data, const uint16_t len);
// uart irq -> queue->drv_task_uart->read_cb
extern esp_err_t drv_uart_gnss_read_set_cb(const drv_uart_read_cb_t cb);
extern int drv_uart_gnss_send(uint8_t *data, const uint32_t len);
// take data from buffer
extern int drv_uart_gnss_recv(uint8_t *data, const uint32_t len);
extern int drv_uart_gnss_recv_with_timeout(uint8_t *data, const uint32_t len, const uint32_t ms);
extern uint16_t drv_uart_gnss_buffered_rx_len(void);
extern void drv_uart_gnss_flush(void);
extern void drv_uart_gnss_config_rx_fifo_threshold(void);

extern void drv_uart_gnss_enable_buffer_cb(void);

extern esp_err_t drv_uart_gnss_set_baudrate(const uint32_t baud_rate);
extern esp_err_t drv_uart_gnss_clear_rx(void);

extern void drv_uart_usbc_printf(const char *fmt, ...);

extern void set_gnss_cmd_mode(bool flag);

#endif //ESP32_XBEE_UART_H
