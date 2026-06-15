#ifndef ESP32_XBEE_UTIL_H
#define ESP32_XBEE_UTIL_H

#include <esp_transport.h>
#include <sys/socket.h>
#include <freertos/FreeRTOS.h>

#include <uart.h>

#define PRINT_LINE printf("%s:%d %s\n", __FILE__, __LINE__, __func__)
#define UART_PRINT_LINE uart_nmea("$PESP,DBG,%s,%d,%s", __FILE__, __LINE__, __func__)

#define ERROR_ACTION(TAG, condition, action, format, ... ) if ((condition)) {             \
            ESP_LOGE(TAG, "%s:%d (%s): " format, __FILE__, __LINE__, __FUNCTION__,  ##__VA_ARGS__); \
            action; \
        }

#define SOCKTYPE_NAME(socktype) (socktype == SOCK_STREAM ? "TCP" : (socktype == SOCK_DGRAM ? "UDP" : (socktype == SOCK_RAW ? "RAW" : "???")))

#define CONNECT_SOCKET_ERROR_OPTS -3
#define CONNECT_SOCKET_ERROR_RESOLVE -2
#define CONNECT_SOCKET_ERROR_CONNECT -1

void destroy_socket(int *socket);
char *sockaddrtostr(struct sockaddr *a);

char *extract_http_header(const char *buffer, const char *key);

int connect_socket(char *host, int port, int socktype);
char *http_auth_basic_header(const char *username, const char *password);

typedef struct
{
    uint32_t msgID;
    uint8_t *pMsgData;
} APP_MSG_TYPE;
#define APP_MUTEX_BLOCK_WAIT portMAX_DELAY

#define app_msg_queue_create(queSize, dataSize) xQueueCreate((queSize), (dataSize))
#define app_msg_queue_delete(que) vQueueDelete(que)

#define app_mutex_create() xSemaphoreCreateMutex()
#define app_mutex_delete(mutexHdlr) vSemaphoreDelete((mutexHdlr))
//#define app_mutex_take(mutexHdlr, timeout) xSemaphoreTake((mutexHdlr), ((timeout) / portTICK_RATE_MS))makeItCompileable
#define app_mutex_take(mutexHdlr, timeout) xSemaphoreTake((mutexHdlr), pdMS_TO_TICKS(timeout))
#define app_mutex_give(mutexHdlr) xSemaphoreGive((mutexHdlr))

#endif //ESP32_XBEE_UTIL_H
