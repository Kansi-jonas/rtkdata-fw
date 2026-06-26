

#include <stdbool.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_event_base.h>
#include <sys/socket.h>
#include <sys/time.h>  
#include <wifi.h>
#include <tasks.h>
#include <status_led.h>
#include <retry.h>
#include <stream_stats.h>
#include <freertos/event_groups.h>
#include <freertos/task.h> 
#include <esp_ota_ops.h>
#include <errno.h>
#include "interface/ntrip.h"
#include "config.h"
#include "util.h"
#include "uart.h"
#include "supervisor.h"

static const char *TAG = "NTRIP_SERVER";

#define BUFFER_SIZE 512
#define MAX_NTRIP_SERVERS 10   // 0 … 9
#define NTRIP_SEND_TIMEOUT_MS 500

#define NTRIP_SLEEP_TASK_STACK      4096   // in Words (FreeRTOS-Stackeinheit)
#define NTRIP_SLEEP_STACK_WARN_WORDS 128   // Threshold: < 128 Words free -> Log-Warning

static const int CASTER_READY_BIT = BIT0;
static const int DATA_READY_BIT   = BIT1;
static const int DATA_SENT_BIT    = BIT2;

typedef struct {

    char                    suffix[4];       // "" or "_1" .. "_9"
    int                     index;           // 0..9 (only for Logging/Stats)
    EventGroupHandle_t      ev;              // EventGroup per instance
    TaskHandle_t            task_server;     // maintask (self)
    TaskHandle_t            task_sleep;      // Sleep/KeepAlive Task
    status_led_handle_t     led;             // LED Handle
    stream_stats_handle_t   stats;           // Stream-Stats
    retry_delay_handle_t    retry;           // Retry-Backoff
    int                     sock;            // Socket for this instance
    int                     data_keep_alive; // KeepAlive counter
    int                     blocked_sends;   // counter for blocked sends
    volatile bool           reconnect_req;   // supervisor / send-error -> reconnect
    
} ntrip_instance_t;

//global instance register

static ntrip_instance_t *g_instances[MAX_NTRIP_SERVERS] = {0};
static size_t            g_instance_count               = 0;
static SemaphoreHandle_t g_instances_mutex              = NULL;

static bool s_uart_handler_registered = false;

//helper

static inline void build_key(char *dst, size_t dstsz, const char *base, const char *suffix) {
    // base "ntr_srv_host", suffix "" or "_3"
    snprintf(dst, dstsz, "%s%s", base, suffix);
}

// -----------------------------------------------------------NTRIP-----------------------------------------------------------------//

// EventBits / DATA_READY / DATA_SENT / keep_alive
static bool ntrip_server_update_data_state(ntrip_instance_t *inst, EventBits_t bits) {

    bool dataState = false;

    if (inst && inst->ev) {

        if ((bits & DATA_READY_BIT) == 0) {

            xEventGroupSetBits(inst->ev, DATA_READY_BIT);

            if (bits & DATA_SENT_BIT) {

                ESP_LOGI(TAG, "[%d] UART data arrived; will reconnect if disconnected", inst->index);
            }
        }

        inst->data_keep_alive = 0;

        // caster is not ready -> nothing send
        if ((bits & CASTER_READY_BIT) == 0) {

            dataState = false;

        }else{

            // first time send data, set a flag
            if ((bits & DATA_SENT_BIT) == 0) {

                xEventGroupSetBits(inst->ev, DATA_SENT_BIT);
            }

            dataState = true;

        }

    }

    return dataState;
}

// update KeepAlive-counter
static void ntrip_server_update_keep_alive_counter(ntrip_instance_t *inst)
{
    if (inst) {

        if (inst->data_keep_alive < NTRIP_KEEP_ALIVE_THRESHOLD) {

            inst->data_keep_alive += NTRIP_KEEP_ALIVE_THRESHOLD / 10;

            if (inst->data_keep_alive > NTRIP_KEEP_ALIVE_THRESHOLD) {

                inst->data_keep_alive = NTRIP_KEEP_ALIVE_THRESHOLD;

            }

        }

    }

}

// Check timeout and DATA_READY_BIT delete + warning
static void ntrip_server_handle_keep_alive_timeout(ntrip_instance_t *inst)
{
    if (inst && inst->ev) {

        // Once we have reached or crossed the threshold and still
        // If the data is marked as "DATA_READY", then delete the bit and log once.
        if (inst->data_keep_alive >= NTRIP_KEEP_ALIVE_THRESHOLD) {

            EventBits_t bits = xEventGroupGetBits(inst->ev);

            if (bits & DATA_READY_BIT) {

                xEventGroupClearBits(inst->ev, DATA_READY_BIT);

                ESP_LOGW(TAG,"[%d] No data received by UART in %d seconds, will not " 
                    "reconnect to caster if disconnected", inst->index, NTRIP_KEEP_ALIVE_THRESHOLD);
            }
        }
    }

}

// Periodic monitoring (Heap/Stack/Bits) every X seconds
static void ntrip_server_periodic_monitor(ntrip_instance_t *inst,int *monitor_elapsed_sec,
    int monitor_interval_sec){

    if (inst && monitor_elapsed_sec) {

        //Accumulate time
        *monitor_elapsed_sec += NTRIP_KEEP_ALIVE_THRESHOLD / 10;

        if (*monitor_elapsed_sec >= monitor_interval_sec) {

            //Interval reached -> Log status
            *monitor_elapsed_sec = 0;

            size_t free_heap = esp_get_free_heap_size();
            UBaseType_t server_stack_hw = 0;
            UBaseType_t sleep_stack_hw  = uxTaskGetStackHighWaterMark(NULL); // dieser Task

            if (inst->task_server) {

                server_stack_hw = uxTaskGetStackHighWaterMark(inst->task_server);
            }

            EventBits_t bits = inst->ev ? xEventGroupGetBits(inst->ev) : 0;

            ESP_LOGI(TAG,"[%d] Periodic status: free_heap=%u bytes, ""server_stack_min_free=%u words, "
                    "sleep_stack_min_free=%u words, bits=0x%02x, data_keep_alive=%d",inst->index,
                    (unsigned)free_heap, (unsigned)server_stack_hw, (unsigned)sleep_stack_hw,
                    (unsigned)bits,inst->data_keep_alive);
            }

    }

}

static void ntrip_server_sleep_task(void *ctx)
{
    ntrip_instance_t *inst = (ntrip_instance_t *)ctx;

    // Nur zur Orientierung im Log
    ESP_LOGI(TAG, "[%d] Sleep task started, suspending until first UART data", inst->index);

    //Activated by ntrip_server_task after the first data reception
    vTaskSuspend(NULL);

    ESP_LOGI(TAG, "[%d] Sleep task resumed (KeepAlive/Monitor aktiv)", inst->index);


    const int monitor_interval_sec = 3600;   // every 1 hour
    int monitor_elapsed_sec = 0;

    // Stack-Monitoring: wir merken uns das bisherige Minimum
    UBaseType_t last_hw_mark = uxTaskGetStackHighWaterMark(NULL);

    while (true) {

        // --- Stack-Monitoring (minimal) ---
        UBaseType_t cur_hw = uxTaskGetStackHighWaterMark(NULL);
        if (cur_hw < last_hw_mark) {
            last_hw_mark = cur_hw;

            if (cur_hw < NTRIP_SLEEP_STACK_WARN_WORDS) {
                ESP_LOGW(TAG,
                         "[%d] Sleep task stack low: min_free=%u words",
                         inst->index, (unsigned)cur_hw);
            } else {
                ESP_LOGI(TAG,
                         "[%d] Sleep task stack updated: min_free=%u words",
                         inst->index, (unsigned)cur_hw);
            }
        }
        // -----------------------------------

        //Update KeepAlive-counter
        ntrip_server_update_keep_alive_counter(inst);

        //Timeout handle (DATA_READY_BIT delete + warning)
        ntrip_server_handle_keep_alive_timeout(inst);

        //periodic monitoring (heap/stack/bits)
        ntrip_server_periodic_monitor(inst, &monitor_elapsed_sec, monitor_interval_sec);

        vTaskDelay(pdMS_TO_TICKS(NTRIP_KEEP_ALIVE_THRESHOLD / 10));
    }
}

// Init: EventGroup, Sleep-Task, UART-Handler, LED, Stats, Retry
static void ntrip_server_init_task_context(ntrip_instance_t *inst, char *key,size_t key_size,
    char *stats_name, size_t stats_size){

    config_color_t color;

    // EventGroup & SleepTask
    inst->ev = xEventGroupCreate();

    if (!inst->ev) {

        ESP_LOGE(TAG, "[%d] Failed to create EventGroup", inst->index);
        vTaskDelete(NULL);
    }

    if (xTaskCreate(ntrip_server_sleep_task,"ntrip_server_sleep_task",NTRIP_SLEEP_TASK_STACK,inst,
            TASK_PRIORITY_INTERFACE,&inst->task_sleep) != pdPASS) {

        ESP_LOGE(TAG, "[%d] Failed to create sleep task", inst->index);
        vTaskDelete(NULL);
    }

    // load LED-color
    build_key(key, key_size, "ntr_srv_color", inst->suffix);

    color = config_get_color(CONF_ITEM(key));

    if (color.rgba != 0) {

        inst->led = status_led_add(color.rgba, STATUS_LED_FADE, 500, 2000, 0);
    }

    if (inst->led) {

        inst->led->active = false;
    }

    // Stats-name
    if (inst->index == 0) {

        snprintf(stats_name, stats_size, "ntrip_server_0");

    } else {

        snprintf(stats_name, stats_size, "ntrip_server_%d", inst->index);

    }

    inst->stats = stream_stats_new(stats_name);

    if (!inst->stats) {

        ESP_LOGW(TAG, "[%d] Failed to create stream stats", inst->index);
    }

    // Retry
    inst->retry = retry_init(true, 5, 2000, 0);

    if (!inst->retry) {

        ESP_LOGE(TAG, "[%d] Failed to init retry, falling back to fixed 2s delay", inst->index);
    }
}

// Waiting phase before connection establishment: Retry delay, waiting for UART data,
// Sleep-Task start, auf IP warten
static void ntrip_server_wait_for_start(ntrip_instance_t *inst){

    // Retry / Fallback-Delay
    if (inst->retry) {

        retry_delay(inst->retry);

    } else {

        vTaskDelay(pdMS_TO_TICKS(2000));

    }

    // wait for first UART data
    if (inst->ev && ((xEventGroupGetBits(inst->ev) & DATA_READY_BIT) == 0)) {

        ESP_LOGI(TAG, "[%d] Waiting for UART input to connect to caster", inst->index);
        uart_nmea("$PESP,NTRIP,SRV,WAITING,%d", inst->index);

        xEventGroupWaitBits(inst->ev, DATA_READY_BIT, true, false, portMAX_DELAY);
    }

    // push sleep/keep alive-task
    if (inst->task_sleep) {

        vTaskResume(inst->task_sleep);
    }

    wait_for_ip(0);   // block (forever) until the STA has an IP before (re)connecting
}

// config (Host, Port, Passwort, Mountpoint)
static void ntrip_server_load_config_from_storage(ntrip_instance_t *inst, char *key, size_t key_size,
    char **host, uint16_t *port, char **pwd, char **mp){

    if (inst && key && host && port && pwd && mp) {
        
        *host = NULL;
        *pwd  = NULL;
        *mp   = NULL;
        *port = 2101;

        // Host
        build_key(key, key_size, "ntr_srv_host", inst->suffix);
        config_get_str_blob_alloc(CONF_ITEM(key), (void **)host);

        // Port
        build_key(key, key_size, "ntr_srv_port", inst->suffix);
        config_get_primitive(CONF_ITEM(key), port);

        // Passwort
        build_key(key, key_size, "ntr_srv_pass", inst->suffix);
        config_get_str_blob_alloc(CONF_ITEM(key), (void **)pwd);

        // Mountpoint
        build_key(key, key_size, "ntr_srv_mp", inst->suffix);
        config_get_str_blob_alloc(CONF_ITEM(key), (void **)mp);

    }

}

// Socket send timeout setzen
static void ntrip_server_set_socket_timeout(ntrip_instance_t *inst)
{
    struct timeval tv;

    tv.tv_sec  = NTRIP_SEND_TIMEOUT_MS / 1000;
    tv.tv_usec = (NTRIP_SEND_TIMEOUT_MS % 1000) * 1000;

    if (setsockopt(inst->sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {

        ESP_LOGW(TAG, "[%d] Failed to set SO_SNDTIMEO: %d %s", inst->index, 
            errno, strerror(errno));
    }
}

// Monitoring after successfull connect
static void ntrip_server_log_connect_monitoring(ntrip_instance_t *inst){

    size_t free_heap = esp_get_free_heap_size();

    UBaseType_t server_stack_hw = uxTaskGetStackHighWaterMark(NULL);  // current task
    UBaseType_t sleep_stack_hw  = 0;

    if (inst->task_sleep) {

        sleep_stack_hw = uxTaskGetStackHighWaterMark(inst->task_sleep);
    }

    // HighWaterMark is in "Words", not Bytes
    ESP_LOGI(TAG, "[%d] Connect status: free_heap=%u bytes, server_stack_min_free=%u words, "
        "sleep_stack_min_free=%u words", inst->index, (unsigned)free_heap, (unsigned)server_stack_hw,
        (unsigned)sleep_stack_hw);
}

// Disconnect-Handling (Bits, LED, Logs, NMEA)
static void ntrip_server_handle_disconnect(ntrip_instance_t *inst, const char *host, uint16_t port,
    const char *mp){

    if (inst->ev) {

        xEventGroupClearBits(inst->ev, CASTER_READY_BIT | DATA_SENT_BIT);
    }

    if (inst->led) {

        inst->led->active = false;
    }

    ESP_LOGW(TAG, "[%d] Disconnected from %s:%u/%s", inst->index, host ? host : "", port, mp ? mp : "");

    uart_nmea("$PESP,NTRIP,SRV,DISCONNECTED,%d,%s:%u,%s", inst->index, host ? host : "", port, mp ? mp : "");
}

// Monitoring after disconnect
static void ntrip_server_log_disconnect_monitoring(ntrip_instance_t *inst){

    size_t free_heap_disc = esp_get_free_heap_size();

    UBaseType_t server_stack_hw_disc = uxTaskGetStackHighWaterMark(NULL);
    UBaseType_t sleep_stack_hw_disc  = 0;

    if (inst->task_sleep) {
        sleep_stack_hw_disc = uxTaskGetStackHighWaterMark(inst->task_sleep);
    }

    ESP_LOGI(TAG, "[%d] Disconnect status: free_heap=%u bytes, server_stack_min_free=%u words, "
        "sleep_stack_min_free=%u words", inst->index, (unsigned)free_heap_disc,
        (unsigned)server_stack_hw_disc, (unsigned)sleep_stack_hw_disc);

}

// Cleanup at the end of an iteration (error OR normal disconnect)
static void ntrip_server_cleanup_iteration(ntrip_instance_t *inst, char *txbuf, char *host,
    char *mp, char *pwd){

    if (inst && inst->task_sleep) {

        vTaskSuspend(inst->task_sleep);
    }

    if (inst) {

        destroy_socket(&inst->sock);
    }

    free(txbuf);
    free(host);
    free(mp);
    free(pwd);
}

static void ntrip_server_task(void *ctx){

    ntrip_instance_t *inst = (ntrip_instance_t *)ctx;

    char key[64];
    char stats_name[32];

    ntrip_server_init_task_context(inst, key, sizeof(key), stats_name, sizeof(stats_name));


    while (true) {

        inst->blocked_sends = 0;

        char *txbuf = NULL;
        char *host  = NULL;
        char *mp    = NULL;
        char *pwd   = NULL;

        uint16_t port = 2101;

        ntrip_server_wait_for_start(inst);

        ntrip_server_load_config_from_storage(inst, key, sizeof(key), &host, &port, &pwd, &mp);

        ESP_LOGI(TAG, "[%d] Connecting to %s:%u/%s", inst->index, host ? host : "", port, mp ? mp : "");

        uart_nmea("$PESP,NTRIP,SRV,CONNECTING,%d,%s:%u,%s", inst->index, host ? host : "", port, mp ? mp : "");

        inst->sock = connect_socket(host, port, SOCK_STREAM);

        ERROR_ACTION(TAG, inst->sock == CONNECT_SOCKET_ERROR_RESOLVE, goto _error,
                     "Host resolve failed");

        ERROR_ACTION(TAG, inst->sock == CONNECT_SOCKET_ERROR_CONNECT, goto _error,
                     "Connect failed");

        ntrip_server_set_socket_timeout(inst);

        txbuf = malloc(BUFFER_SIZE);

        if (!txbuf) {

            goto _error;
        }

        snprintf(txbuf, BUFFER_SIZE, "SOURCE %s /%s" NEWLINE "Source-Agent: NTRIP %s/%s" NEWLINE NEWLINE,
                (pwd ? pwd : ""), (mp ? mp : ""), NTRIP_SERVER_NAME,
                 &esp_app_get_description()->version[1]);

        int err = write(inst->sock, txbuf, strlen(txbuf));

        ERROR_ACTION(TAG, err < 0, goto _error, "Send request failed: %d %s", errno, strerror(errno));

        int len = read(inst->sock, txbuf, BUFFER_SIZE - 1);

        ERROR_ACTION(TAG, len <= 0, goto _error, "Recv resp failed: %d %s", errno, strerror(errno));

        txbuf[len] = '\0';

        char *status = extract_http_header(txbuf, "");

        ERROR_ACTION(TAG, status == NULL || !ntrip_response_ok(status), free(status); goto _error,
                    "Mountpoint connect error: %s", status == NULL ? "HTTP malformed" : status);

        free(status);

        ESP_LOGI(TAG, "[%d] Connected to %s:%u/%s", inst->index, host ? host : "", port, mp ? mp : "");

        uart_nmea("$PESP,NTRIP,SRV,CONNECTED,%d,%s:%u,%s", inst->index, host ? host : "", port, mp ? mp : "");

        if (inst->retry) {

            retry_reset(inst->retry);
        }

        if (inst->led) {

            inst->led->active = true;
        }

        // Instance is connected
        if (inst->ev) {

            xEventGroupSetBits(inst->ev, CASTER_READY_BIT);
        }

        // Monitoring after connected
        ntrip_server_log_connect_monitoring(inst);

        // Hold the connection up, but actively detect a half-open socket instead of
        // suspending forever (the stock bug: a GNSS stall meant no send ever failed,
        // so a dead caster socket was never noticed -> the station went dark).
        inst->reconnect_req = false;
        while (!inst->reconnect_req) {
            char probe;
            int r = recv(inst->sock, &probe, 1, MSG_DONTWAIT | MSG_PEEK);
            if (r == 0) { ESP_LOGW(TAG, "[%d] caster closed connection", inst->index); break; }
            if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
                ESP_LOGW(TAG, "[%d] socket probe error %d", inst->index, errno); break;
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        // Disconnect handling (Bits/LED/Logs)
        ntrip_server_handle_disconnect(inst, host, port, mp);

        // Monitoring after disconnect
        ntrip_server_log_disconnect_monitoring(inst);

    _error:

        ntrip_server_cleanup_iteration(inst, txbuf, host, mp, pwd);
    }
}

// -----------------------------------------------------------UART-----------------------------------------------------------------//

// Send data to a instance
static void ntrip_server_send_data(ntrip_instance_t *inst, int32_t length, void *buffer) {

    int sent = write(inst->sock, buffer, length);

    if (sent < 0) {

        int errsv = errno;

        if (errsv == EWOULDBLOCK || errsv == EAGAIN) {

            // Send buffer is full or timeout -> Paket drop, no reaction
            ESP_LOGW(TAG,"[%d] send would block/timeout, dropping %d bytes",
                     inst->index, (int)length);

            inst->blocked_sends++;         

            if (inst->blocked_sends > 20) {

                ESP_LOGW(TAG, "[%d] too many blocked sends, closing socket/reconnecting", inst->index);

                inst->reconnect_req = true;
                destroy_socket(&inst->sock);

                if (inst->task_server) {

                    vTaskResume(inst->task_server);
                }
            }

        } else {

            ESP_LOGW(TAG,"[%d] send error (%d: %s), closing socket",
                     inst->index, errsv, strerror(errsv));

            inst->reconnect_req = true;
            destroy_socket(&inst->sock);

            if (inst->task_server) {

                vTaskResume(inst->task_server);
            }
        }

    } else if (sent > 0) {

        inst->blocked_sends = 0;
        supervisor_note_caster_tx(sent);

        if (inst->stats) {
            stream_stats_increment(inst->stats, 0, sent);
        }
    }

    // sent == 0 -> ignore
}

// complete processing for one instance
static void ntrip_server_handle_uart_instance(ntrip_instance_t *inst,int32_t length, void *buffer) {

    if (inst && inst->ev) {

        EventBits_t bits = xEventGroupGetBits(inst->ev);

        // EventBits & KeepAlive update, check if we should send
        if (ntrip_server_update_data_state(inst, bits)) {
            
            ntrip_server_send_data(inst, length, buffer);
        }

    }

}

static void ntrip_server_uart_handler(void* handler_args,esp_event_base_t base,int32_t length,
    void* buffer) {

    (void)handler_args;
    (void)base;

    if (g_instances_mutex){

        xSemaphoreTake(g_instances_mutex, portMAX_DELAY);

        for (size_t k = 0; k < g_instance_count; ++k) {

            ntrip_instance_t *inst = g_instances[k];

            ntrip_server_handle_uart_instance(inst, length, buffer);
        }

        xSemaphoreGive(g_instances_mutex);

    }

}

static void ensure_uart_handler_registered(void) {

    if (!g_instances_mutex) {

        g_instances_mutex = xSemaphoreCreateMutex();

        if (!g_instances_mutex) {

            ESP_LOGE(TAG, "Failed to create g_instances_mutex in ensure_uart_handler_registered");

        }
    }

    if(g_instances_mutex){

        // Protection against race conditions when registering the handler
        xSemaphoreTake(g_instances_mutex, portMAX_DELAY);

        if (!s_uart_handler_registered) {

            uart_register_read_handler(ntrip_server_uart_handler);

            s_uart_handler_registered = true;

            ESP_LOGI(TAG, "UART read handler for NTRIP registered");
        }

        xSemaphoreGive(g_instances_mutex);

    }


}

void ntrip_server_init() {

    if (!g_instances_mutex){

        g_instances_mutex = xSemaphoreCreateMutex();

        if (!g_instances_mutex) {

            ESP_LOGE(TAG, "Failed to create g_instances_mutex");

            return;
        }

    }

    // UART-Handler EINMAL registrieren
    ensure_uart_handler_registered();

    for (int i = 0; i < MAX_NTRIP_SERVERS; i++) {

        char suffix[4] = "";

        if (i > 0) {

            snprintf(suffix, sizeof(suffix), "_%d", i);
        }

        char key[64];

        build_key(key, sizeof(key), "ntr_srv_actv", suffix);

        if (config_get_bool1(CONF_ITEM(key))) {

            // create instance
            ntrip_instance_t *inst = calloc(1, sizeof(*inst));

            if (!inst) continue;

            strncpy(inst->suffix, suffix, sizeof(inst->suffix));

            inst->suffix[sizeof(inst->suffix) - 1] = '\0';
            inst->index = i;
            inst->sock  = -1;

            bool registered = false;

            // register
            xSemaphoreTake(g_instances_mutex, portMAX_DELAY);

            if (g_instance_count < MAX_NTRIP_SERVERS) {

                g_instances[g_instance_count++] = inst;

                registered = true;
            }

            xSemaphoreGive(g_instances_mutex);

            if (!registered) {

                ESP_LOGE(TAG, "Max NTRIP instances reached, skipping index %d", i);

                free(inst);

                continue;
            }
            

            // start task
            char task_name[32];

            if (i == 0) snprintf(task_name, sizeof(task_name), "ntrip_server_task_0");
            else        snprintf(task_name, sizeof(task_name), "ntrip_server_task_%d", i);

            ESP_LOGI(TAG, "Starting NTRIP server task for index %d (suffix '%s')", i, inst->suffix);

            BaseType_t res = xTaskCreate(ntrip_server_task, task_name, 4096, inst, 
                TASK_PRIORITY_INTERFACE, &inst->task_server);

            if (res != pdPASS) {

                    ESP_LOGE(TAG, "Failed to create NTRIP server task for index %d", i);

                    // Instance remove from global array
                    xSemaphoreTake(g_instances_mutex, portMAX_DELAY);

                    for (size_t k = 0; k < g_instance_count; ++k) {

                        if (g_instances[k] == inst) {

                            for (size_t j = k + 1; j < g_instance_count; ++j) {

                                g_instances[j - 1] = g_instances[j];
                            }

                            g_instances[--g_instance_count] = NULL;

                            break;
                        }
                    }

                    xSemaphoreGive(g_instances_mutex);

                    free(inst);
            }
        }
    }
}

// RTKdata: ask every NTRIP instance to drop its socket and reconnect (picks up
// new credentials from provisioning, and is the supervisor's caster recovery).
void ntrip_server_reconnect_all(void) {
    if (!g_instances_mutex) return;
    xSemaphoreTake(g_instances_mutex, portMAX_DELAY);
    for (size_t k = 0; k < g_instance_count; ++k) {
        ntrip_instance_t *inst = g_instances[k];
        if (inst) inst->reconnect_req = true;
    }
    xSemaphoreGive(g_instances_mutex);
}