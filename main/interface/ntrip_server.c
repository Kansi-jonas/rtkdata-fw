

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
#include <esp_timer.h>
#include <errno.h>
#include <fcntl.h>
#include "interface/ntrip.h"
#include "interface/ntrip_tx_core.h"
#include "config.h"
#include "util.h"
#include "uart.h"
#include "supervisor.h"

static const char *TAG = "NTRIP_SERVER";

#define BUFFER_SIZE 512
#define MAX_NTRIP_SERVERS NTRIP_MAX_INSTANCES   // 0 … 9
#define NTRIP_SEND_TIMEOUT_MS 500
#define NTRIP_RECV_TIMEOUT_MS 10000
#define NTRIP_DRAIN_WAIT_MS 200
#define NTRIP_PROBE_INTERVAL_MS 2000
#define NTRIP_DRAIN_MAX_SPINS 64

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
    ntrip_tx_t              tx;              // sender state + counters (tx core)
    // Reconnect request as a generation counter, not a clearable boolean: a
    // request that arrives during DNS/handshake must survive until the drain
    // loop can honor it (review 2026-08-01).
    volatile uint32_t       reconnect_gen;

} ntrip_instance_t;

//global instance register

static ntrip_instance_t *g_instances[MAX_NTRIP_SERVERS] = {0};
static size_t            g_instance_count               = 0;
static SemaphoreHandle_t g_instances_mutex              = NULL;

// Shared RTCM ingest: one parser (single GNSS UART source) feeding one frame
// ring, all statically allocated. Only CRC-valid complete frames enter the
// ring; the per-instance server tasks drain it. The UART path touches no
// socket anywhere.
static rtcm_parser_t     g_rtcm_parser;
static ntrip_ring_t      g_frame_ring;
static SemaphoreHandle_t g_ring_mutex = NULL;

static void ring_lock_hook(void *ctx)   { xSemaphoreTake((SemaphoreHandle_t)ctx, portMAX_DELAY); }
static void ring_unlock_hook(void *ctx) { xSemaphoreGive((SemaphoreHandle_t)ctx); }

static inline uint32_t ntrip_now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

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

// Remove an instance from the global register. Under the mutex, so the UART
// ingest path can never see (or notify) a half-torn-down instance.
static void ntrip_server_deregister_instance(ntrip_instance_t *inst) {

    if (!g_instances_mutex || !inst) return;

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
}

// Init failed mid-way: deregister BEFORE this task deletes itself, otherwise
// the ingest path keeps notifying a dead task handle (review 2026-08-01).
static void ntrip_server_abort_task_init(ntrip_instance_t *inst) {

    ntrip_server_deregister_instance(inst);

    if (inst->ev) {
        vEventGroupDelete(inst->ev);
    }

    free(inst);
    vTaskDelete(NULL);
}

// Init: EventGroup, Sleep-Task, LED, Stats, Retry
static void ntrip_server_init_task_context(ntrip_instance_t *inst, char *key,size_t key_size,
    char *stats_name, size_t stats_size){

    config_color_t color;

    // EventGroup & SleepTask
    inst->ev = xEventGroupCreate();

    if (!inst->ev) {

        ESP_LOGE(TAG, "[%d] Failed to create EventGroup, disabling this instance", inst->index);
        ntrip_server_abort_task_init(inst);
    }

    if (xTaskCreate(ntrip_server_sleep_task,"ntrip_server_sleep_task",NTRIP_SLEEP_TASK_STACK,inst,
            TASK_PRIORITY_INTERFACE,&inst->task_sleep) != pdPASS) {

        ESP_LOGE(TAG, "[%d] Failed to create sleep task, disabling this instance", inst->index);
        ntrip_server_abort_task_init(inst);
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

// Handshake socket timeouts. The SOURCE request and its response run on a
// blocking socket; SO_RCVTIMEO bounds the response wait (the old code could
// block in read() forever). After the handshake the socket goes non-blocking
// and neither timeout applies to the data path.
static void ntrip_server_set_handshake_timeouts(ntrip_instance_t *inst)
{
    struct timeval tv;

    tv.tv_sec  = NTRIP_SEND_TIMEOUT_MS / 1000;
    tv.tv_usec = (NTRIP_SEND_TIMEOUT_MS % 1000) * 1000;

    if (setsockopt(inst->sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {

        ESP_LOGW(TAG, "[%d] Failed to set SO_SNDTIMEO: %d %s", inst->index,
            errno, strerror(errno));
    }

    tv.tv_sec  = NTRIP_RECV_TIMEOUT_MS / 1000;
    tv.tv_usec = (NTRIP_RECV_TIMEOUT_MS % 1000) * 1000;

    if (setsockopt(inst->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {

        ESP_LOGW(TAG, "[%d] Failed to set SO_RCVTIMEO: %d %s", inst->index,
            errno, strerror(errno));
    }
}

// The send callback for the tx core. Runs exclusively in this instance's own
// server task on a non-blocking socket: nothing here can stall another
// instance, the UART path, or the event loop.
static int ntrip_server_sock_send(void *ctx, const uint8_t *buf, size_t len, int *out_errno)
{
    ntrip_instance_t *inst = (ntrip_instance_t *)ctx;

    int n = send(inst->sock, buf, len, 0);

    if (n > 0) {

        supervisor_note_caster_tx(n);

        if (inst->stats) {
            stream_stats_increment(inst->stats, 0, n);
        }

    } else if (n < 0) {

        *out_errno = errno;
    }

    return n;
}

static bool ntrip_server_make_nonblocking(ntrip_instance_t *inst)
{
    int fl = fcntl(inst->sock, F_GETFL, 0);

    if (fl < 0 || fcntl(inst->sock, F_SETFL, fl | O_NONBLOCK) < 0) {

        ESP_LOGW(TAG, "[%d] fcntl O_NONBLOCK failed: %d %s", inst->index,
            errno, strerror(errno));
        return false;
    }

    return true;
}

// Detect a closed/broken connection; consumes and discards caster chatter so
// the receive window can never fill up. Returns false when the connection is
// gone.
static bool ntrip_server_probe_socket(ntrip_instance_t *inst)
{
    char scratch[64];

    int r = recv(inst->sock, scratch, sizeof(scratch), MSG_DONTWAIT);

    if (r == 0) {
        ESP_LOGW(TAG, "[%d] caster closed connection", inst->index);
        return false;
    }

    if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
        ESP_LOGW(TAG, "[%d] socket probe error %d", inst->index, errno);
        return false;
    }

    return true;
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

        char *txbuf = NULL;
        char *host  = NULL;
        char *mp    = NULL;
        char *pwd   = NULL;

        uint16_t port = 2101;

        ntrip_server_wait_for_start(inst);

        ntrip_server_load_config_from_storage(inst, key, sizeof(key), &host, &port, &pwd, &mp);

        // Snapshot the reconnect generation BEFORE dialing: a request that
        // arrives during DNS/handshake ends the new connection promptly
        // instead of being lost (the old boolean was cleared unconditionally).
        uint32_t conn_gen = inst->reconnect_gen;

        ESP_LOGI(TAG, "[%d] Connecting to %s:%u/%s", inst->index, host ? host : "", port, mp ? mp : "");

        uart_nmea("$PESP,NTRIP,SRV,CONNECTING,%d,%s:%u,%s", inst->index, host ? host : "", port, mp ? mp : "");

        inst->sock = connect_socket(host, port, SOCK_STREAM);

        ERROR_ACTION(TAG, inst->sock == CONNECT_SOCKET_ERROR_RESOLVE, goto _error,
                     "Host resolve failed");

        ERROR_ACTION(TAG, inst->sock == CONNECT_SOCKET_ERROR_CONNECT, goto _error,
                     "Connect failed");

        ntrip_server_set_handshake_timeouts(inst);

        txbuf = malloc(BUFFER_SIZE);

        if (!txbuf) {

            goto _error;
        }

        snprintf(txbuf, BUFFER_SIZE, "SOURCE %s /%s" NEWLINE "Source-Agent: NTRIP %s/%s" NEWLINE NEWLINE,
                (pwd ? pwd : ""), (mp ? mp : ""), NTRIP_SERVER_NAME,
                 &esp_app_get_description()->version[1]);

        // The request must go out completely; a short write here would desync
        // the handshake exactly like it desynced RTCM frames on the data path.
        size_t req_len = strlen(txbuf);
        size_t req_off = 0;

        while (req_off < req_len) {

            int werr = write(inst->sock, txbuf + req_off, req_len - req_off);

            if (werr <= 0) break;

            req_off += (size_t)werr;
        }

        ERROR_ACTION(TAG, req_off != req_len, goto _error, "Send request failed: %d %s", errno, strerror(errno));

        int len = read(inst->sock, txbuf, BUFFER_SIZE - 1);

        ERROR_ACTION(TAG, len <= 0, goto _error, "Recv resp failed: %d %s", errno, strerror(errno));

        txbuf[len] = '\0';

        char *status = extract_http_header(txbuf, "");

        ERROR_ACTION(TAG, status == NULL || !ntrip_response_ok(status), free(status); goto _error,
                    "Mountpoint connect error: %s", status == NULL ? "HTTP malformed" : status);

        free(status);

        ERROR_ACTION(TAG, !ntrip_server_make_nonblocking(inst), goto _error,
                     "Could not switch socket to non-blocking");

        // ORDER MATTERS (review 2026-08-01): cursor/accounting/notifications
        // are initialized BEFORE CASTER_READY_BIT is visible. The other way
        // round, a frame pushed in between was skipped without being counted.
        ntrip_tx_reset_conn(&inst->tx, &g_frame_ring, ntrip_now_ms());
        ulTaskNotifyTake(pdTRUE, 0);   // clear notifications from before this connect

        // Instance is connected
        if (inst->ev) {

            xEventGroupSetBits(inst->ev, CASTER_READY_BIT);
        }

        ESP_LOGI(TAG, "[%d] Connected to %s:%u/%s", inst->index, host ? host : "", port, mp ? mp : "");

        uart_nmea("$PESP,NTRIP,SRV,CONNECTED,%d,%s:%u,%s", inst->index, host ? host : "", port, mp ? mp : "");

        if (inst->led) {

            inst->led->active = true;
        }

        // Arm the caster watchdog from the handshake on; the backoff reset
        // deliberately waits for the first ACCEPTED bytes (a handshake that
        // dies right after must keep backing off, or the fleet hammers a
        // struggling caster in lockstep).
        supervisor_note_caster_tx(0);

        // Monitoring after connected
        ntrip_server_log_connect_monitoring(inst);

        // This task owns the socket exclusively from here on. The UART path
        // only parses into the shared frame ring and notifies us; we drain
        // complete RTCM frames to the caster on a non-blocking socket. The
        // probe (recv) keeps detecting a half-open socket like before (the
        // stock bug: a GNSS stall meant no send ever failed, so a dead caster
        // socket was never noticed -> the station went dark).
        bool made_progress = false;
        uint32_t last_probe_ms = ntrip_now_ms();

        while (inst->reconnect_gen == conn_gen) {

            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(NTRIP_DRAIN_WAIT_MS));

            uint32_t now = ntrip_now_ms();

            // Drain until the ring is empty or the socket pushes back. The
            // spin cap only bounds one wake-up; the outer loop continues.
            ntrip_tx_poll_result_t pr;
            int spins = 0;

            do {
                pr = ntrip_tx_poll(&inst->tx, &g_frame_ring, now,
                                   NTRIP_TX_MAX_SENDS_PER_POLL,
                                   NTRIP_TX_STALE_DROP_MS,
                                   ntrip_server_sock_send, inst);

                if (pr == NTRIP_TX_POLL_PROGRESS && !made_progress) {

                    // First accepted bytes on this connection: NOW the retry
                    // backoff may reset (not at handshake, see above).
                    made_progress = true;

                    if (inst->retry) {
                        retry_reset(inst->retry);
                    }
                }
            } while (pr == NTRIP_TX_POLL_PROGRESS && ++spins < NTRIP_DRAIN_MAX_SPINS);

            if (pr == NTRIP_TX_POLL_ERROR) {
                ESP_LOGW(TAG, "[%d] send error (%d: %s), closing connection",
                         inst->index, inst->tx.last_sock_errno,
                         strerror(inst->tx.last_sock_errno));
                break;
            }

            if (ntrip_tx_stalled(&inst->tx, now, NTRIP_TX_PROGRESS_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "[%d] no send progress for %d ms with %u bytes pending, "
                         "reconnecting", inst->index, NTRIP_TX_PROGRESS_TIMEOUT_MS,
                         (unsigned)ntrip_tx_pending(&inst->tx));
                break;
            }

            // A frame that keeps trickling but is past its absolute age limit
            // ends the CONNECTION; the remainder is never dropped mid-stream
            // (review 2026-08-01: byte-drip kept the stall clock reset while
            // the frame grew arbitrarily old).
            if (ntrip_tx_frame_overdue(&inst->tx, now, NTRIP_TX_FRAME_DEADLINE_MS)) {
                ESP_LOGW(TAG, "[%d] partially sent frame older than %d ms, closing "
                         "connection", inst->index, NTRIP_TX_FRAME_DEADLINE_MS);
                break;
            }

            now = ntrip_now_ms();

            if ((uint32_t)(now - last_probe_ms) >= NTRIP_PROBE_INTERVAL_MS) {

                last_probe_ms = now;

                if (!ntrip_server_probe_socket(inst)) break;
            }
        }

        // Account whatever was still pending; the next connection starts with
        // a fresh, complete frame, never the rest of this one.
        ntrip_tx_abort(&inst->tx);
        inst->tx.reconnects++;

        // Disconnect handling (Bits/LED/Logs)
        ntrip_server_handle_disconnect(inst, host, port, mp);

        // Monitoring after disconnect
        ntrip_server_log_disconnect_monitoring(inst);

    _error:

        ntrip_server_cleanup_iteration(inst, txbuf, host, mp, pwd);
    }
}

// -----------------------------------------------------------UART-----------------------------------------------------------------//

static void ntrip_server_on_frame(void *ctx, const uint8_t *frame, uint16_t len,
    uint32_t now) {

    (void)ctx;

    // A CRC-valid frame is the ONLY thing that counts as "the receiver is
    // alive": command echoes and serial garbage must not feed the watchdog
    // (review 2026-08-01). No valid frame for TH_GNSS -> supervisor re-runs
    // the UM980 config; still nothing -> wedge rung reboots. That removes the
    // customer's manual power cycle.
    supervisor_note_gnss_rx();

    ntrip_ring_push(&g_frame_ring, frame, len, now);
}

// Called SYNCHRONOUSLY from uart_task for every UART chunk. This is the whole
// RTCM data path: parse, push complete CRC-valid frames into the shared ring,
// wake the connected instances. No socket, no allocation, no event-loop heap
// copy whose failure could silently swallow a chunk (review 2026-08-01,
// esp_event_post returns ESP_ERR_NO_MEM under heap pressure and the old
// handler path ignored it). The esp_event path still exists for the other
// UART consumers; RTCM no longer depends on it.
void ntrip_server_ingest_uart(const uint8_t *data, size_t len) {

    if (!g_ring_mutex || !data || len == 0) return;   // not initialized yet

    uint32_t frames_before = g_frame_ring.next_seq;   // sole writer: this task

    rtcm_parser_feed(&g_rtcm_parser, data, len,
                     ntrip_now_ms(), ntrip_server_on_frame, NULL);

    // DATA_READY / keep-alive / wake-ups are driven by complete valid frames,
    // not by raw bytes: garbage must neither hold a mountpoint open nor wake
    // the drain tasks (review 2026-08-01).
    if (g_frame_ring.next_seq == frames_before) return;

    if (g_instances_mutex){

        xSemaphoreTake(g_instances_mutex, portMAX_DELAY);

        for (size_t k = 0; k < g_instance_count; ++k) {

            ntrip_instance_t *inst = g_instances[k];

            if (inst && inst->ev) {

                EventBits_t bits = xEventGroupGetBits(inst->ev);

                // EventBits & KeepAlive update; wake the drain task if the
                // instance is connected
                if (ntrip_server_update_data_state(inst, bits) && inst->task_server) {

                    xTaskNotifyGive(inst->task_server);
                }
            }
        }

        xSemaphoreGive(g_instances_mutex);

    }

}

// RTCM ingest (parser + ring). Idempotent. MUST run BEFORE uart_init() spawns
// uart_task (which feeds ntrip_server_ingest_uart synchronously), and long
// BEFORE the data plane: GNSS liveness is noted per valid frame in this
// path, and the supervisor's 30 s boot grace must never expire just because
// the OTA window delayed ntrip_server_init(). (Found 2026-08-01 via the new
// heartbeat telemetry: a slow OTA check let the supervisor hardware-reset a
// perfectly healthy receiver four times in a row.)
void ntrip_server_ingest_init(void) {

    if (g_ring_mutex) return;

    SemaphoreHandle_t mtx = xSemaphoreCreateMutex();

    if (!mtx) {

        ESP_LOGE(TAG, "Failed to create g_ring_mutex, RTCM ingest disabled");

        return;
    }

    rtcm_parser_init(&g_rtcm_parser);
    ntrip_ring_init(&g_frame_ring, ring_lock_hook, ring_unlock_hook, mtx);

    // Publish LAST: g_ring_mutex doubles as the ready flag that
    // ntrip_server_ingest_uart() checks, so parser + ring must be fully
    // initialized before it becomes non-NULL. Release ordering keeps the
    // compiler from hoisting the store above the init calls; main.c also
    // sequences ingest_init before uart_init as the primary guarantee
    // (v1.1.2 audit 2026-08-01: mutex-first init raced uart_task against
    // the parser/ring memset).
    __atomic_store_n(&g_ring_mutex, mtx, __ATOMIC_RELEASE);

    // uart_task feeds ntrip_server_ingest_uart() directly from here on;
    // there is deliberately NO esp_event handler in the RTCM path.
    ESP_LOGI(TAG, "RTCM ingest ready (direct UART feed, %d ring slots)",
             RTCM_RING_SLOTS);
}

void ntrip_server_init() {

    if (!g_instances_mutex){

        g_instances_mutex = xSemaphoreCreateMutex();

        if (!g_instances_mutex) {

            ESP_LOGE(TAG, "Failed to create g_instances_mutex");

            return;
        }

    }

    // Normally already done right after uart_init(); harmless if not.
    ntrip_server_ingest_init();

    if (!g_ring_mutex) return;   // ingest init failed; no instances without it

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

                    ntrip_server_deregister_instance(inst);

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
        if (inst) inst->reconnect_gen++;   // requests survive DNS/handshake
    }
    xSemaphoreGive(g_instances_mutex);
}

// Diagnostics snapshot for the web UI (/ntrip/tx_stats). The 64-bit counters
// are written by other tasks without a lock, so a value can tear on this
// 32-bit target; that is acceptable for diagnostics and keeps the hot path
// free of extra locking.
size_t ntrip_server_tx_stats(ntrip_tx_stats_t *out, size_t max) {

    size_t n = 0;

    if (!g_instances_mutex || !out) return 0;

    xSemaphoreTake(g_instances_mutex, portMAX_DELAY);

    for (size_t k = 0; k < g_instance_count && n < max; ++k) {

        ntrip_instance_t *inst = g_instances[k];

        if (!inst) continue;

        const ntrip_tx_t *tx = &inst->tx;
        ntrip_tx_stats_t *s = &out[n++];

        s->index               = inst->index;
        s->connected           = inst->ev &&
                                 (xEventGroupGetBits(inst->ev) & CASTER_READY_BIT);
        s->accepted_to_lwip    = tx->accepted_to_lwip;
        s->sent_frames         = tx->sent_frames;
        s->copied_bytes        = tx->copied_bytes;
        s->dropped_bytes       = tx->dropped_bytes;
        s->skipped_bytes       = tx->skipped_bytes;
        s->skipped_frames      = tx->skipped_frames;
        s->dropped_frames_stale = tx->dropped_frames_stale;
        s->dropped_stale_bytes = tx->dropped_stale_bytes;
        s->eagain_count        = tx->eagain_count;
        s->reconnects          = tx->reconnects;
        s->pending             = ntrip_tx_pending(tx);
        s->max_queue_age_ms    = tx->max_queue_age_ms;
        s->last_sock_errno     = tx->last_sock_errno;
    }

    xSemaphoreGive(g_instances_mutex);

    return n;
}

// Heartbeat summary: ingest + sender totals. Same tearing caveat as above,
// acceptable for diagnostics. Mutexes are taken sequentially, never nested.
void ntrip_server_tx_totals(ntrip_tx_totals_t *out) {

    if (!out) return;

    memset(out, 0, sizeof(*out));

    ntrip_ingest_stats_t ing;
    ntrip_server_ingest_stats(&ing);
    out->frames_ok       = ing.frames_ok;
    out->bytes_ok        = ing.bytes_ok;
    out->bytes_discarded = ing.bytes_discarded;
    out->crc_errors      = ing.crc_errors;

    if (!g_instances_mutex) return;

    xSemaphoreTake(g_instances_mutex, portMAX_DELAY);

    for (size_t k = 0; k < g_instance_count; ++k) {

        ntrip_instance_t *inst = g_instances[k];

        if (!inst) continue;

        const ntrip_tx_t *tx = &inst->tx;

        out->instances++;
        if (inst->ev && (xEventGroupGetBits(inst->ev) & CASTER_READY_BIT)) {
            out->connected++;
        }
        out->accepted_to_lwip    += tx->accepted_to_lwip;
        out->sent_frames         += tx->sent_frames;
        out->dropped_bytes       += tx->dropped_bytes;
        out->skipped_bytes       += tx->skipped_bytes;
        out->dropped_stale_bytes += tx->dropped_stale_bytes;
        out->eagain_count        += tx->eagain_count;
        out->reconnects          += tx->reconnects;
        if (tx->max_queue_age_ms > out->max_queue_age_ms) {
            out->max_queue_age_ms = tx->max_queue_age_ms;
        }
    }

    xSemaphoreGive(g_instances_mutex);
}

void ntrip_server_ingest_stats(ntrip_ingest_stats_t *out) {

    if (!out) return;

    memset(out, 0, sizeof(*out));

    if (g_ring_mutex) xSemaphoreTake(g_ring_mutex, portMAX_DELAY);

    out->frames_ok          = g_rtcm_parser.frames_ok;
    out->bytes_ok           = g_rtcm_parser.bytes_ok;
    out->bytes_discarded    = g_rtcm_parser.bytes_discarded;
    out->crc_errors         = g_rtcm_parser.crc_errors;
    out->ring_pushed_frames = g_frame_ring.pushed_frames;
    out->ring_cum_bytes     = g_frame_ring.cum_bytes;

    if (g_ring_mutex) xSemaphoreGive(g_ring_mutex);
}