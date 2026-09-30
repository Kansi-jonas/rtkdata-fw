/*
 * RTKdata reference firmware - data-path health supervisor.
 *
 * One high-priority task owns end-to-end liveness of the GNSS -> caster path.
 * Three monotonic liveness timestamps are stamped from the hot paths
 * (GNSS RX, successful caster TX, STA IP up). The task evaluates them once a
 * second and walks an escalation ladder, where each rung only fires once the
 * subsystem has been healthy at least once (so we never "recover" something
 * that was never configured, e.g. the caster during pre-enroll onboarding).
 *
 * Rungs (cheapest first):
 *   no GNSS byte    > 10 s  -> reset + reconfigure the UM980
 *   no caster send  > 30 s  -> force-close socket + reconnect
 *   no STA IP       > 60 s  -> restart the Wi-Fi driver
 *   total wedge     > 120 s -> esp_restart() (last resort)
 *
 * Concrete recovery actions are injected by main.c via supervisor_set_recovery()
 * so this module stays decoupled from gnss/ntrip/wifi.
 *
 * License: GPLv3 (see LICENSE).
 */
#include <stdbool.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include "sdkconfig.h"

#include "supervisor.h"
#include "gnss_site.h"
#include "interface/ntrip.h"
#include "uart.h"

#define TAG "SUPERVISOR"

#define SUPERVISOR_TASK_PRIO   6      /* above interface(5), below UART(10) */
#define SUPERVISOR_TASK_STACK  3072
#define TICK_MS                1000

/* boot grace: let config/uart/wifi/ntrip settle before we judge anything */
#define GRACE_S                30

/* fault thresholds (seconds) */
#define TH_GNSS_S              10
#define TH_CASTER_S            30
#define TH_LINK_S              60
#define TH_WEDGE_S             120

/* per-rung debounce: min seconds between repeated recovery attempts */
#define DEB_GNSS_S             20
#define DEB_CASTER_S           15
#define DEB_LINK_S             30

/* heap-exhaustion guard: if free heap stays below HEAP_FLOOR_B for
 * HEAP_LOW_S consecutive seconds, reboot cleanly before a hard OOM fault
 * drops every session uncontrolled. The floor sits well under the ~70 KB
 * steady-state idle heap, so it only fires on genuine exhaustion. */
#define HEAP_FLOOR_B           (12 * 1024)
#define HEAP_LOW_S             15

/* With PSRAM (Ethernet board) the total free heap includes megabytes of
 * PSRAM and would never trip the floor while the INTERNAL RAM that DMA, the
 * EMAC, Wi-Fi and the task stacks need runs dry. Guard the internal part. */
static inline size_t guarded_free_heap(void) {
#if CONFIG_SPIRAM
    return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
    return esp_get_free_heap_size();
#endif
}

/* survives a soft reboot (not power loss) so flapping is visible to the IE */
static RTC_DATA_ATTR uint32_t s_supervised_reboots;

/* liveness timestamps in microseconds (esp_timer monotonic) */
static volatile int64_t s_t_gnss   = 0;
static volatile int64_t s_t_caster = 0;
static volatile int64_t s_t_ip     = 0;

/* "has this subsystem ever been healthy" gates */
static volatile bool s_gnss_seen   = false;
static volatile bool s_caster_seen = false;
static volatile bool s_caster_progress = false; // handshake alone proves no RTCM progress
static volatile bool s_ip_seen     = false;
static volatile bool s_link_up     = false;

/* recovery counters (since boot) */
static uint32_t s_rec_gnss   = 0;
static uint32_t s_rec_caster = 0;
static uint32_t s_rec_wifi   = 0;

static supervisor_recovery_fn s_recover_gnss   = NULL;
static supervisor_recovery_fn s_recover_caster = NULL;
static supervisor_recovery_fn s_recover_wifi   = NULL;

static inline int64_t now_us(void) { return esp_timer_get_time(); }
static inline uint32_t age_s(int64_t t) { return (uint32_t)((now_us() - t) / 1000000); }

/* ---- hot-path notes ---------------------------------------------------- */

void supervisor_note_gnss_rx(void) { s_t_gnss = now_us(); s_gnss_seen = true; }

void supervisor_note_caster_tx(int bytes) {
    /* bytes>0 is progress and refreshes the liveness timestamp.
     *
     * bytes==0 is the handshake ARMING the watchdog: it makes "no caster
     * progress" judgeable from now on (before, a connection that never sent a
     * byte was a permanent blind spot). It must NOT refresh the timestamp on
     * an already-armed watchdog: a reconnect loop of bare handshakes would
     * otherwise keep the caster looking alive forever while zero payload
     * moved (review 2026-08-03). */
    if (bytes > 0) {
        s_t_caster = now_us();
        s_caster_seen = true;
        s_caster_progress = true;
    } else if (!s_caster_seen) {
        s_t_caster = now_us();
        s_caster_seen = true;
    }
}

void supervisor_note_sta_ip(bool up) {
    if (up) { s_t_ip = now_us(); s_ip_seen = true; s_link_up = true; }
    else    { s_link_up = false; }
}

void supervisor_set_recovery(supervisor_recovery_fn gnss_reset,
                             supervisor_recovery_fn caster_reconnect,
                             supervisor_recovery_fn wifi_restart) {
    s_recover_gnss   = gnss_reset;
    s_recover_caster = caster_reconnect;
    s_recover_wifi   = wifi_restart;
}

/* ---- health snapshot --------------------------------------------------- */

static bool gnss_observation_fault(void) {
    return gnss_site_output_expected() &&
        ntrip_server_observations_silent_ms() >= TH_GNSS_S * 1000U;
}

void supervisor_health(supervisor_health_t *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->uptime_s        = (uint32_t)(now_us() / 1000000);
    out->gnss_silent_s   = s_gnss_seen   ? age_s(s_t_gnss)   : out->uptime_s;
    out->caster_silent_s = s_caster_seen ? age_s(s_t_caster) : 0; /* 0 = n/a yet */
    out->ip_down_s       = s_link_up ? 0 : (s_ip_seen ? age_s(s_t_ip) : 0);
    uint32_t obs_ms = ntrip_server_observations_silent_ms();
    out->obs_silent_s = obs_ms == UINT32_MAX ? -1 : (int32_t)(obs_ms / 1000U);
    out->obs_expected = gnss_site_output_expected();
    out->gnss_ok         = s_gnss_seen && out->gnss_silent_s < TH_GNSS_S &&
                          !gnss_site_needs_recovery() && !gnss_observation_fault();
    out->caster_ok       = gnss_site_upload_allowed() && s_caster_progress &&
                          out->caster_silent_s < TH_CASTER_S;
    out->link_ok         = s_link_up || !s_ip_seen;
    out->recoveries_gnss   = s_rec_gnss;
    out->recoveries_caster = s_rec_caster;
    out->recoveries_wifi   = s_rec_wifi;
    out->reboots_supervised = s_supervised_reboots;
}

/* ---- the supervisor task ----------------------------------------------- */

static void supervisor_task(void *ctx) {
    (void)ctx;
    int64_t last_gnss_act = 0, last_caster_act = 0, last_link_act = 0;
    int heap_low_streak = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));

        uint32_t uptime = (uint32_t)(now_us() / 1000000);
        if (uptime < GRACE_S) continue;

        /* Rung 5: heap-exhaustion guard (self-healing against a slow leak or
         * fragmentation that would otherwise hard-fault). Sustained low heap
         * -> clean reboot, counted so flapping is visible to the IE. */
        if (guarded_free_heap() < HEAP_FLOOR_B) {
            if (++heap_low_streak >= HEAP_LOW_S) {
                s_supervised_reboots++;
                ESP_LOGE(TAG, "heap exhausted (free=%u < %u for %ds) -> reboot #%u",
                         (unsigned)guarded_free_heap(), (unsigned)HEAP_FLOOR_B,
                         HEAP_LOW_S, (unsigned)s_supervised_reboots);
                uart_nmea("$PESP,RTK,SUPERVISOR,HEAP,%u", (unsigned)guarded_free_heap());
                vTaskDelay(pdMS_TO_TICKS(150));
                esp_restart();
            }
        } else {
            heap_low_streak = 0;
        }

        /* GNSS: silent if never seen after grace, or stale */
        uint32_t gnss_silent = s_gnss_seen ? age_s(s_t_gnss) : uptime;
        bool basepos_fault = gnss_site_needs_recovery();
        bool observation_fault = gnss_observation_fault();
        bool gnss_fault = gnss_silent >= TH_GNSS_S || basepos_fault || observation_fault;

        /* Caster: only judged once it has sent at least once */
        uint32_t caster_silent = s_caster_seen ? age_s(s_t_caster) : 0;
        bool caster_fault = gnss_site_upload_allowed() && s_caster_seen &&
                            caster_silent >= TH_CASTER_S;

        /* Link: only judged once we have ever had an IP (skip pure-AP setup) */
        uint32_t ip_down = (!s_link_up && s_ip_seen) ? age_s(s_t_ip) : 0;
        bool link_fault = s_ip_seen && !s_link_up && ip_down >= TH_LINK_S;

        /* Rung 4: total wedge -> reboot. Conservative: caster (the product's
         * whole purpose) dead AND an underlying cause dead, sustained. */
        if (caster_fault && caster_silent >= TH_WEDGE_S &&
            (gnss_silent >= TH_WEDGE_S || ip_down >= TH_WEDGE_S)) {
            s_supervised_reboots++;
            ESP_LOGE(TAG, "data path wedged (gnss_silent=%us caster_silent=%us ip_down=%us) "
                          "-> reboot #%u", (unsigned)gnss_silent, (unsigned)caster_silent,
                          (unsigned)ip_down, (unsigned)s_supervised_reboots);
            uart_nmea("$PESP,RTK,SUPERVISOR,REBOOT,%u,%u,%u",
                      gnss_silent, caster_silent, ip_down);
            vTaskDelay(pdMS_TO_TICKS(150)); /* let the NMEA flush */
            esp_restart();
        }

        /* Rung 3: link down -> restart Wi-Fi driver (don't wait stock 5 min) */
        if (link_fault && s_recover_wifi && age_s(last_link_act) >= DEB_LINK_S) {
            ESP_LOGW(TAG, "STA link down %us -> wifi driver restart", (unsigned)ip_down);
            uart_nmea("$PESP,RTK,SUPERVISOR,WIFI,%u", ip_down);
            last_link_act = now_us();
            s_rec_wifi++;
            s_recover_wifi();
        }

        /* Rung 2: caster silent -> force reconnect */
        if (caster_fault && s_recover_caster && age_s(last_caster_act) >= DEB_CASTER_S) {
            ESP_LOGW(TAG, "no caster send %us -> reconnect", (unsigned)caster_silent);
            uart_nmea("$PESP,RTK,SUPERVISOR,CASTER,%u", caster_silent);
            last_caster_act = now_us();
            s_rec_caster++;
            s_recover_caster();
        }

        /* Rung 1: no GNSS bytes -> reset + reconfigure the UM980 */
        if (gnss_fault && s_recover_gnss && age_s(last_gnss_act) >= DEB_GNSS_S) {
            const char *reason = observation_fault ? "observations" : basepos_fault ? "basepos" : "receiver";
            uint32_t obs_ms = ntrip_server_observations_silent_ms();
            int32_t obs_seconds = obs_ms == UINT32_MAX ? -1 : (int32_t)(obs_ms / 1000U);
            ESP_LOGW(TAG, "GNSS recovery reason=%s receiver_silent=%us observation_silent=%lds (-1=never)",
                     reason, (unsigned)gnss_silent, (long)obs_seconds);
            uart_nmea("$PESP,RTK,SUPERVISOR,GNSS,%u,%s,%ld",
                      gnss_silent, reason, (long)obs_seconds);
            last_gnss_act = now_us();
            s_rec_gnss++;
            s_recover_gnss();
        }
    }
}

void supervisor_init(void) {
    ESP_LOGI(TAG, "starting (grace=%us gnss=%us caster=%us link=%us wedge=%us, prior supervised reboots=%u)",
             GRACE_S, TH_GNSS_S, TH_CASTER_S, TH_LINK_S, TH_WEDGE_S,
             (unsigned)s_supervised_reboots);
    xTaskCreate(supervisor_task, "rtk_supervisor", SUPERVISOR_TASK_STACK, NULL,
                SUPERVISOR_TASK_PRIO, NULL);
}
