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
 * MERCHANTuart_taskABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <string.h>
#include <protocol/nmea.h>
#include <stream_stats.h>

#include "uart.h"
#include "config.h"
#include "interface/socket_server.h"
#include "tasks.h"
#include "util.h"

#define RX_BUFF_SIZE (1024 * 4)
#define TX_BUFF_SIZE (1024)

#define USBC_PORT UART_NUM_0
#define USBC_TXD_PIN GPIO_NUM_1 // UART_PIN_NO_CHANGE //(GPIO_NUM_1)
#define USBC_RXD_PIN GPIO_NUM_3 // UART_PIN_NO_CHANGE //(GPIO_NUM_3)
#define USBC_BAUD 115200
//gnss com2 <-> eps uart1
#define UN_PORT UART_NUM_1
#define UN_TXD_PIN GPIO_NUM_18 //23->18
#define UN_RXD_PIN GPIO_NUM_5
// gnss com1 <-> esp32 uart2
#define GNSS_PORT UART_NUM_2
#define GNSS_TXD_PIN GPIO_NUM_17
#define GNSS_RXD_PIN GPIO_NUM_14
#define GNSS_BAUD 115200
#define GNSS_UART_RX_THRES (96) 
#define USBC_UART_RX_THRES (96)
#define UART_QUEUE_NUM (20)

static const char *TAG = "UART";

ESP_EVENT_DEFINE_BASE(UART_EVENT_READ);
ESP_EVENT_DEFINE_BASE(UART_EVENT_WRITE);

void uart_register_read_handler(esp_event_handler_t event_handler) {
    ESP_ERROR_CHECK(esp_event_handler_register(UART_EVENT_READ, ESP_EVENT_ANY_ID, event_handler, NULL));
}

void uart_unregister_read_handler(esp_event_handler_t event_handler) {
    ESP_ERROR_CHECK(esp_event_handler_unregister(UART_EVENT_READ, ESP_EVENT_ANY_ID, event_handler));
}

void uart_register_write_handler(esp_event_handler_t event_handler) {
    ESP_ERROR_CHECK(esp_event_handler_register(UART_EVENT_WRITE, ESP_EVENT_ANY_ID, event_handler, NULL));
}

void uart_unregister_write_handler(esp_event_handler_t event_handler) {
    ESP_ERROR_CHECK(esp_event_handler_unregister(UART_EVENT_WRITE, ESP_EVENT_ANY_ID, event_handler));
}

static int uart_port = -1;
static bool uart_log_forward = false;

static stream_stats_handle_t stream_stats;

static void uart_task(void *ctx);
static QueueHandle_t usbcQueue; // esp_uart0:for usbc
static QueueHandle_t gnssQueue; // esp_uart2:gnss com1

static SemaphoreHandle_t gnss_mutex_send;
static SemaphoreHandle_t usbc_mutex_send;

static drv_uart_read_cb_t usbc_read_cb_func = NULL;
static drv_uart_read_cb_t gnss_read_cb_func = NULL;
//static drv_uart_read_cb_t esp_dr_read_cb_func = NULL;

typedef volatile struct
{
    volatile uint16_t buffSize;
    volatile uint8_t *pHead;
    volatile uint8_t *pTail;
    volatile uint8_t *pBuff;
} DATA_QUEUE_TYPE;

/*static esp_err_t  handle_gnss_received_data(uint8_t *data, const uint16_t len)
{
    stream_stats_increment(stream_stats, len, 0);
    esp_event_post(UART_EVENT_READ, len, data, len, portMAX_DELAY);
    return ESP_OK;
}*/

static esp_err_t handle_usbc_received_data(uint8_t *data, const uint16_t len)
{
    return ESP_OK;
}

void uart_init() {
    uart_log_forward = config_get_bool1(CONF_ITEM(KEY_CONFIG_UART_LOG_FORWARD));
    
    // uart_port = config_get_u8(CONF_ITEM(KEY_CONFIG_UART_NUM));
    uart_port = GNSS_PORT;
    uart_hw_flowcontrol_t flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    // bool flow_ctrl_rts = config_get_bool1(CONF_ITEM(KEY_CONFIG_UART_FLOW_CTRL_RTS));
    // bool flow_ctrl_cts = config_get_bool1(CONF_ITEM(KEY_CONFIG_UART_FLOW_CTRL_CTS));
    // if (flow_ctrl_rts && flow_ctrl_cts) {
    //     flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
    // } else if (flow_ctrl_rts) {
    //     flow_ctrl = UART_HW_FLOWCTRL_RTS;
    // } else if (flow_ctrl_cts) {
    //     flow_ctrl = UART_HW_FLOWCTRL_CTS;
    // } else {
    //     flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    // }

    uart_config_t uart_config = {
            .baud_rate = config_get_u32(CONF_ITEM(KEY_CONFIG_UART_BAUD_RATE)),
            .data_bits = config_get_u8(CONF_ITEM(KEY_CONFIG_UART_DATA_BITS)),
            .parity = config_get_u8(CONF_ITEM(KEY_CONFIG_UART_PARITY)),
            .stop_bits = config_get_u8(CONF_ITEM(KEY_CONFIG_UART_STOP_BITS)),
            .flow_ctrl = flow_ctrl
    };
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(
            uart_port,
            // config_get_i8(CONF_ITEM(KEY_CONFIG_UART_TX_PIN)),
            // config_get_i8(CONF_ITEM(KEY_CONFIG_UART_RX_PIN)),
            // config_get_i8(CONF_ITEM(KEY_CONFIG_UART_RTS_PIN)),
            // config_get_i8(CONF_ITEM(KEY_CONFIG_UART_CTS_PIN))
            GNSS_TXD_PIN,GNSS_RXD_PIN,
            UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE
    ));
    ESP_ERROR_CHECK(uart_driver_install(uart_port, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0));
    
    stream_stats = stream_stats_new("uart");
    gnss_mutex_send = app_mutex_create();
    xTaskCreate(uart_task, "uart_task", 8192, NULL, TASK_PRIORITY_UART, NULL);
    drv_uart_usbc_init();
    // drv_uart_gnss_init();
    drv_uart_usbc_read_set_cb(handle_usbc_received_data);
    // drv_uart_gnss_read_set_cb(handle_gnss_received_data);
}
static volatile bool gnss_cmd_mode = false;
void set_gnss_cmd_mode(bool flag)
{
    gnss_cmd_mode = flag;
}
static void uart_task(void *ctx) {
    uint8_t buffer[UART_BUFFER_SIZE];

    while (true) {
        int32_t len = uart_read_bytes(uart_port, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
        if (len < 0) {
            ESP_LOGE(TAG, "Error reading from UART");
        } else if (len == 0) {
            continue;
        }

        stream_stats_increment(stream_stats, len, 0);
        esp_event_post(UART_EVENT_READ, len, &buffer, len, portMAX_DELAY);
        buffer[len] = '\0';
        if(gnss_cmd_mode)
        {
            //ESP_LOGI(TAG,"rx len %d:%s",len,buffer);makeItCompileable
            ESP_LOGI(TAG, "rx len %ld:%s", (long int)len, buffer);

        }
    }
}

// to  usbc 
int uart_log(char *buf, size_t len) {
    // if (!uart_log_forward) return 0;
    // return uart_write(buf, len);
    return drv_uart_usbc_send((uint8_t*)buf,len);
}
// to usbc
int uart_nmea(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char *nmea;
    nmea_vasprintf(&nmea, fmt, args);
    // int l = uart_write(nmea, strlen(nmea));
    int l = drv_uart_usbc_send((uint8_t*)nmea, strlen(nmea));
    free(nmea);

    va_end(args);

    return l;
}
// inject to gnss
int uart_write(char *buf, size_t len) {
    if (uart_port < 0) return 0;
    if (len == 0) return 0;

    int written = uart_write_bytes(uart_port, buf, len);
    // int written = drv_uart_gnss_send((uint8_t*)buf, len);
    if (written < 0) return written;

    stream_stats_increment(stream_stats, 0, len);

    esp_event_post(UART_EVENT_WRITE, len, buf, len, portMAX_DELAY);

    return written;
}

uint16_t data_que_create(DATA_QUEUE_TYPE *q, uint8_t *buff, uint16_t size)
{
    if(q->buffSize >0)
    {
        return 0;
    }

    uint8_t *pbuff = malloc(size);

    if (NULL == pbuff)
    {
        ESP_LOGE(TAG, "data_que_create fail for malloc %d for buffer", size);
    }
    else
    {
        memset(pbuff, 0, size);
    }

    q->buffSize = size;
    q->pBuff = pbuff;
    q->pHead = pbuff;
    q->pTail = pbuff;

    return 0;
}

int data_que_cleanup(DATA_QUEUE_TYPE *q)
{
    if (q->pBuff != NULL)
    {
        q->pHead = q->pBuff;
        q->pTail = q->pBuff;
        memset((void *)q->pBuff, 0, q->buffSize);
        return 0;
    }
    else
    {
        return -1;
    }
}

uint16_t data_que_destroy(DATA_QUEUE_TYPE *q)
{
    free((uint8_t *)q->pBuff);

    q->buffSize = 0;
    q->pBuff = NULL;
    q->pHead = NULL;
    q->pTail = NULL;

    return 0;
}

uint16_t data_que_size(DATA_QUEUE_TYPE *q)
{
    volatile uint8_t *pHead = NULL;
    volatile uint8_t *pTail = NULL;
    uint16_t size = 0;

    pHead = q->pHead;
    pTail = q->pTail;

    if (pTail - pHead >= 0)
    {
        size = pTail - pHead;
    }
    else
    {
        size = pTail - pHead + q->buffSize;
    }

    return size;
}

uint16_t data_que_push(DATA_QUEUE_TYPE *q, uint8_t byte)
{
    volatile uint8_t *pTail = NULL;

    pTail = q->pTail;

    if (++pTail >= (q->pBuff + q->buffSize)) // back to buffer area header
    {
        pTail = q->pBuff;
    }

    if (pTail == q->pHead) // que is full
    {
        return 0;
    }

    *(q->pTail) = byte;

    q->pTail = pTail;

    return 1;
}

uint16_t data_que_pop(DATA_QUEUE_TYPE *q)
{
    uint8_t byte = 0;

    if (q->pHead != q->pTail)
    {
        byte = *(q->pHead);
        q->pHead++;

        if (q->pHead >= q->pBuff + q->buffSize)
        {
            q->pHead = q->pBuff;
        }
    }

    return byte;
}

uint16_t data_que_packet_in(DATA_QUEUE_TYPE *q, uint8_t *buff, uint16_t len)
{
    volatile uint8_t *pTail = NULL;
    uint16_t idx = 0;

    pTail = q->pTail;

    for (idx = 0; idx < len; ++idx)
    {
        if (++pTail >= q->pBuff + q->buffSize)
        {
            pTail = q->pBuff;
        }
        if (pTail == q->pHead)
        {
            break;
        }

        *(q->pTail) = *(buff);
        buff++;

        q->pTail = pTail;
    }

    return idx;
}

uint16_t data_que_packet_out(DATA_QUEUE_TYPE *q, uint8_t *buff, uint16_t len)
{
    uint16_t idx = 0;

    while ((q->pHead != q->pTail) && (idx < len) && (idx < q->buffSize))
    {
        buff[idx++] = *(q->pHead);
        q->pHead++;

        if (q->pHead >= q->pBuff + q->buffSize)
        {
            q->pHead = q->pBuff;
        }
    }

    return idx;
}

void drv_uart_usbc_printf(const char *fmt, ...)
{
    char buff[128 + 1];
    size_t len;

    va_list va;
    va_start(va, fmt);
    len = vsnprintf(buff, 128, fmt, va);
    va_end(va);
    buff[len] = '\0';

    uart_write_bytes(USBC_PORT, (const char *)buff, len);
}

/*#define LOG_BUF_SIZE (256)
static int drv_uart_usbc_vprintf(const char *format, va_list args)
{
    char log_buffer[LOG_BUF_SIZE + 1];

    memset(log_buffer, 0, sizeof(log_buffer));
    size_t len = vsnprintf(log_buffer, LOG_BUF_SIZE, format, args);
    if (len > LOG_BUF_SIZE)
    {
        len = LOG_BUF_SIZE;
    }
    uart_write_bytes(USBC_PORT, (const char *)log_buffer, len);
    return len;
}*/

static IRAM_ATTR void drv_uart_event_task(void *pvParameters)
{
    int port = *(int *)pvParameters;
    QueueHandle_t *pQueue = NULL;
    drv_uart_read_cb_t *read_cb_func = NULL;
#define MAX_RX_SZ  100    
    uint8_t rxBuff[MAX_RX_SZ]; // less 100

    uart_event_t event;
    switch (port)
    {
    case USBC_PORT:
    {
        pQueue = &usbcQueue;
        read_cb_func = &usbc_read_cb_func;
        ESP_LOGI(TAG, "uart[%d] event task running in %d:", port, TASK_PRIORITY_UART);
        break;
    }
    // case GNSS_PORT:
    // {
    //     pQueue = &gnssQueue;
    //     read_cb_func = &gnss_read_cb_func;
    //     ESP_LOGI(TAG, "uart[%d] event task running in %d:", port, TASK_PRIORITY_UART);
    //     break;
    // }

    default:
    {
        ESP_LOGE(TAG, "Unknow port[%d],%s", port, pcTaskGetName(NULL));
        vTaskDelete(NULL);
        return;
    }
    }

    while (1)
    {
        // Waiting for UART event.
        //if (xQueueReceive(*pQueue, (void *)&event, (portTickType)portMAX_DELAY))makeItCompileable
        if (xQueueReceive(*pQueue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                if (NULL == *read_cb_func)
                {
                    ESP_LOGE(TAG, "invalid read_cb_func, lost %d bytes", event.size);
                    uart_flush_input(port);
                }
                else
                {
                    if (event.size > MAX_RX_SZ)
                    {
                        event.size = MAX_RX_SZ;
                    }
                    memset(rxBuff, 0, MAX_RX_SZ);
                    uart_read_bytes(port, rxBuff, event.size, portMAX_DELAY);
                    (*read_cb_func)(rxBuff, event.size);
                }
                break;
            }
            case UART_FIFO_OVF:
            {
                uart_flush(port);
                xQueueReset(*pQueue);
                break;
            }
            case UART_BUFFER_FULL:
            {
                ESP_LOGE(TAG, "ring buffer full");
                uart_flush(port);
                xQueueReset(*pQueue);
                break;
            }
            // Others
            default:
            {
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
            }
        }
    }

    vTaskDelete(NULL);
}

esp_err_t drv_uart_usbc_init(void)
{
    const uart_config_t usbc_config =
        {
            .baud_rate = USBC_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };
    usbc_mutex_send = app_mutex_create();
    /*init usb-console port*/
    uart_driver_install(USBC_PORT, RX_BUFF_SIZE, 0, UART_QUEUE_NUM, &usbcQueue, 0);
    uart_param_config(USBC_PORT, &usbc_config);
    uart_set_pin(USBC_PORT, USBC_TXD_PIN, USBC_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_rx_full_threshold(USBC_PORT, USBC_UART_RX_THRES);
    // esp_log_set_vprintf(drv_uart_usbc_vprintf);
    // Create a task to handler UART event from ISR
    int usbcPort = USBC_PORT;
    xTaskCreate(drv_uart_event_task, "uart_usbc", 1024 * 3, &usbcPort, TASK_PRIORITY_UART,
                NULL);

    return ESP_OK;
}

static DATA_QUEUE_TYPE gnss_uart_rx_que;
#define GNSS_UART_RX_BUFF_SIZE (1024)

esp_err_t drv_uart_gnss_buffer_data(uint8_t *data, const uint16_t len)
{
    data_que_packet_in(&gnss_uart_rx_que, data, len);
    return ESP_OK;
}

esp_err_t drv_uart_gnss_init(void)
{
    const uart_config_t gnss_config =
        {
            .baud_rate = GNSS_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };
    uart_param_config(GNSS_PORT, &gnss_config);
    gnss_mutex_send = app_mutex_create();

    int gnssPort = GNSS_PORT;
    // uart_driver_install(GNSS_PORT, RX_BUFF_SIZE, TX_BUFF_SIZE, UART_QUEUE_NUM, &gnssQueue, 0);
    uart_driver_install(GNSS_PORT, RX_BUFF_SIZE, 0, UART_QUEUE_NUM, &gnssQueue, 0);
    xTaskCreate(drv_uart_event_task, "uart_gnss", 1024 * 7, &gnssPort, TASK_PRIORITY_UART,
                NULL);

    uart_set_pin(GNSS_PORT, GNSS_TXD_PIN, GNSS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    drv_uart_gnss_config_rx_fifo_threshold();
    drv_uart_gnss_flush();

    return ESP_OK;
}

esp_err_t drv_uart_gnss_set_baudrate(const uint32_t baud_rate)
{
    if (0 == baud_rate)
    {
        return ESP_FAIL;
    }
    return uart_set_baudrate(GNSS_PORT, baud_rate);
}

esp_err_t drv_uart_gnss_clear_rx(void)
{
    xQueueReset(gnssQueue);

    return uart_flush(GNSS_PORT);
}

esp_err_t drv_uart_usbc_read_set_cb(const drv_uart_read_cb_t cb)
{
    usbc_read_cb_func = cb;
    return ESP_OK;
}

int drv_uart_usbc_send(const uint8_t *data, const uint16_t len)
{
    app_mutex_take(usbc_mutex_send, portMAX_DELAY);
    const int txBytes = uart_write_bytes(USBC_PORT, (const char *)data, len);

    app_mutex_give(usbc_mutex_send);
    return txBytes;
}

esp_err_t drv_uart_gnss_read_set_cb(const drv_uart_read_cb_t cb)
{
    gnss_read_cb_func = cb;
    return ESP_OK;
}

int drv_uart_gnss_send(uint8_t *data, const uint32_t len)
{
    app_mutex_take(gnss_mutex_send, portMAX_DELAY);

    const int txBytes = uart_write_bytes(GNSS_PORT, (const char *)data, len);

    if (txBytes != len)
    {
        ESP_LOGE(TAG, "txByte:%d", txBytes);
    }
    app_mutex_give(gnss_mutex_send);
    // ESP_LOGI(TAG, "end send len:%d", len);
    return txBytes;
}

int drv_uart_gnss_recv(uint8_t *data, const uint32_t len)
{
    while (0 == data_que_size(&gnss_uart_rx_que))
    {
        //vTaskDelay(1/portTICK_RATE_MS);makeItCompileable
        vTaskDelay(pdMS_TO_TICKS(1));
;
    }
    return data_que_packet_out(&gnss_uart_rx_que, data, len);
}

int drv_uart_gnss_recv_with_timeout(uint8_t *data, const uint32_t len, const uint32_t ms)
{
    return data_que_packet_out(&gnss_uart_rx_que, data, len);
}

uint16_t drv_uart_gnss_buffered_rx_len(void)
{
    return data_que_size(&gnss_uart_rx_que);
}

void drv_uart_gnss_flush(void)
{
    uart_flush(GNSS_PORT);
    data_que_cleanup(&gnss_uart_rx_que);
}

void drv_uart_gnss_config_rx_fifo_threshold(void)
{
    uart_set_rx_full_threshold(GNSS_PORT, GNSS_UART_RX_THRES);
}

void drv_uart_gnss_enable_buffer_cb(void)
{
    data_que_create(&gnss_uart_rx_que, NULL, GNSS_UART_RX_BUFF_SIZE);
    drv_uart_gnss_read_set_cb(drv_uart_gnss_buffer_data);
}
