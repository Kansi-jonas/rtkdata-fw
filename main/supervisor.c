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
#include "wifi.h"
#include "eth.h"

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
#define ENSURE_S               30     /* re-check for missing uploaders */
#define REQ_STALE_S            30     /* required RTCM stale this long -> repair */
#define DEB_REPAIR_S           120

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

/* Liveness timestamps in MILLISECONDS, 32 bit. They are written from the hot
 * paths on one core and read here on the other: a 64-bit store is two 32-bit
 * stores on the ESP32 and could be read half-old/half-new (once every 71.6
 * min at the low-word rollover) and fake a huge age. An aligned 32-bit word
 * cannot tear, and unsigned subtraction stays right across its 49.7-day
 * wrap for any age below that. */
static volatile uint32_t s_t_gnss   = 0;
static volatile uint32_t s_t_caster = 0;
static volatile uint32_t s_t_ip     = 0;  // when the uplink went down (see note_sta_ip)

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
static supervisor_recovery_fn s_repair_logs    = NULL;

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline uint32_t age_s(uint32_t t_ms) { return (now_ms() - t_ms) / 1000U; }

/* ---- hot-path notes ---------------------------------------------------- */

void supervisor_note_gnss_rx(void) { s_t_gnss = now_ms(); s_gnss_seen = true; }

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
        s_t_caster = now_ms();
        s_caster_seen = true;
        s_caster_progress = true;
    } else if (!s_caster_seen) {
        s_t_caster = now_ms();
        s_caster_seen = true;
    }
}

/* s_t_ip is the moment the uplink was LOST, so ip_down counts the outage. It
 * used to be set on "up" only, which made ip_down the time since the IP was
 * acquired: after a minute online, any blip already exceeded TH_LINK_S and
 * restarted the Wi-Fi driver at the next tick instead of after 60 s down
 * (bench, 2026-11-06: one cable re-plug dropped the setup hotspot). */
void supervisor_note_sta_ip(bool up) {
    if (up) {
        s_ip_seen = true;
        s_link_up = true;
    } else {
        if (s_link_up) s_t_ip = now_ms();   // only the up -> down edge starts the clock
        s_link_up = false;
    }
}

void supervisor_set_recovery(supervisor_recovery_fn gnss_reset,
                             supervisor_recovery_fn caster_reconnect,
                             supervisor_recovery_fn wifi_restart) {
    s_recover_gnss   = gnss_reset;
    s_recover_caster = caster_reconnect;
    s_recover_wifi   = wifi_restart;
}

void supervisor_set_log_repair(supervisor_recovery_fn repair_logs) {
    s_repair_logs = repair_logs;
}

/* ---- health snapshot --------------------------------------------------- */

static bool gnss_observation_fault(void) {
    return gnss_site_output_expected() &&
        ntrip_server_observations_silent_ms() >= TH_GNSS_S * 1000U;
}

void supervisor_health(supervisor_health_t *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->uptime_s        = (uint32_t)(esp_timer_get_time() / 1000000);
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
    uint32_t last_gnss_act = 0, last_caster_act = 0, last_link_act = 0;
    /* Grace timers: an explicit armed flag, never a sentinel value. With
     * "now | 1" as the start, an age computed in the same even millisecond
     * underflowed to 4294967 s and the grace expired at once (R6). */
    uint32_t eth_nolease_since = 0;
    bool     eth_nolease_armed = false;
    uint32_t last_ensure = 0;
    uint32_t req_stale_since = 0, last_repair_act = 0;
    bool     req_stale_armed = false;
    int heap_low_streak = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));

        uint32_t uptime = (uint32_t)(esp_timer_get_time() / 1000000);

        /* Memory telemetry, also during the grace period: the internal heap is
         * the budget that ran out on the first Ethernet hardware run (15.10.),
         * PSRAM is where Wi-Fi/lwIP and most allocations go now. */
        if (uptime % 10 == 0) {
            ESP_LOGI(TAG, "heap: internal free %u min %u largest %u, psram free %u",
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        }

        /* The selected uplink's DNS servers, put back if another
         * interface's DHCP start cleared lwIP's global table (eth.c). */
        eth_dns_guard();

        if (uptime < GRACE_S) continue;

        /* Uploaders: start any configured one that is not running (a task
         * that failed to start at boot, or one activated by provisioning). */
        if (age_s(last_ensure) >= ENSURE_S) {
            last_ensure = now_ms();
            ntrip_server_ensure_instances();
        }

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

        /* Is there an uplink this unit could get back by itself? A configured
         * STA, or an Ethernet carrier. Without either (cable pulled, no Wi-Fi
         * set up) a driver restart or a reboot cannot help, and rebooting only
         * kicked setup-hotspot clients and reset a healthy receiver. */
        bool uplink_recoverable = wifi_sta_configured() || eth_link_up();

        /* Rung 4a: the receiver path is dead. GNSS bytes flowed, then stopped
         * for TH_WEDGE_S although rung 1 already reset the receiver. The
         * cause is local (UART driver or task, receiver power), so a reboot
         * can fix it, and it must not hinge on the caster: the upload gate
         * closes within seconds without BASEPOS, which made caster_fault
         * false and this escalation unreachable. s_gnss_seen is cleared by the
         * reboot, so a receiver that never comes back gets one reboot, not a
         * loop. */
        if (s_gnss_seen && gnss_silent >= TH_WEDGE_S && s_rec_gnss > 0) {
            s_supervised_reboots++;
            ESP_LOGE(TAG, "GNSS receiver path dead (%us silent after %u resets) -> reboot #%u",
                     (unsigned)gnss_silent, (unsigned)s_rec_gnss, (unsigned)s_supervised_reboots);
            uart_nmea("$PESP,RTK,SUPERVISOR,REBOOT,%u,%u,%u", gnss_silent, caster_silent, ip_down);
            vTaskDelay(pdMS_TO_TICKS(150));
            esp_restart();
        }

        /* Rung 4b: total wedge -> reboot. Conservative: caster (the product's
         * whole purpose) dead AND an underlying cause dead, sustained. A dead
         * uplink only counts when it is recoverable at all. */
        if (caster_fault && caster_silent >= TH_WEDGE_S &&
            (gnss_silent >= TH_WEDGE_S || (uplink_recoverable && ip_down >= TH_WEDGE_S))) {
            s_supervised_reboots++;
            ESP_LOGE(TAG, "data path wedged (gnss_silent=%us caster_silent=%us ip_down=%us) "
                          "-> reboot #%u", (unsigned)gnss_silent, (unsigned)caster_silent,
                          (unsigned)ip_down, (unsigned)s_supervised_reboots);
            uart_nmea("$PESP,RTK,SUPERVISOR,REBOOT,%u,%u,%u",
                      gnss_silent, caster_silent, ip_down);
            vTaskDelay(pdMS_TO_TICKS(150)); /* let the NMEA flush */
            esp_restart();
        }

        /* Rung 3: link down -> restart Wi-Fi driver (don't wait stock 5 min).
         * Skipped when nothing is recoverable: it did nothing, yet logged and
         * counted a recovery every 30 s. */
        if (link_fault && uplink_recoverable && s_recover_wifi && age_s(last_link_act) >= DEB_LINK_S) {
            ESP_LOGW(TAG, "uplink down %us -> link recovery", (unsigned)ip_down);
            uart_nmea("$PESP,RTK,SUPERVISOR,WIFI,%u", ip_down);
            last_link_act = now_ms();
            s_rec_wifi++;
            s_recover_wifi();
        }

        /* Rung 3b (Ethernet): carrier but no lease for TH_LINK_S, also before
         * the FIRST lease. The rung above needs s_ip_seen, so a DHCP client
         * that failed to start at boot (allocation failure) was never kicked
         * and the unit stayed offline with the cable in. The recovery call
         * restarts the DHCP client in exactly this state (eth_link_recover). */
        /* Not while the Wi-Fi STA carries the uplink: every DHCP client start
         * clears lwIP's global DNS table (IDF 5.3.1), which then broke the
         * working Wi-Fi uplink's name resolution, and lwIP's DHCP client keeps
         * asking on its own anyway. The rung is for a client that never
         * started, with no other uplink (review R9), or for a client that is
         * not running at all, which no DHCP retry can bring back (R10). */
        if (eth_link_up() && !eth_has_ip() && (!s_link_up || !eth_dhcp_client_running())) {
            if (!eth_nolease_armed) { eth_nolease_since = now_ms(); eth_nolease_armed = true; }
            if (!link_fault && age_s(eth_nolease_since) >= TH_LINK_S &&
                s_recover_wifi && age_s(last_link_act) >= DEB_LINK_S) {
                ESP_LOGW(TAG, "Ethernet carrier without lease for %us -> link recovery",
                         (unsigned)age_s(eth_nolease_since));
                uart_nmea("$PESP,RTK,SUPERVISOR,ETHDHCP,%u", (unsigned)age_s(eth_nolease_since));
                last_link_act = now_ms();
                s_rec_wifi++;
                s_recover_wifi();
            }
        } else {
            eth_nolease_armed = false;
        }

        /* Rung 2: caster silent -> force reconnect */
        if (caster_fault && s_recover_caster && age_s(last_caster_act) >= DEB_CASTER_S) {
            ESP_LOGW(TAG, "no caster send %us -> reconnect", (unsigned)caster_silent);
            uart_nmea("$PESP,RTK,SUPERVISOR,CASTER,%u", caster_silent);
            last_caster_act = now_ms();
            s_rec_caster++;
            s_recover_caster();
        }

        /* Rung 0: a required RTCM type (1005/1033/1077/1087) stays stale
         * although the stream is expected and the receiver is otherwise fine.
         * The fix is to re-enable that log, not to reset a receiver that is
         * streaming (review 2026-11-06, G4). Past each type's own budget
         * (30 s for 1005/1033) plus REQ_STALE_S, then at most every
         * DEB_REPAIR_S. */
        ntrip_msg_freshness_t fresh;
        ntrip_server_msg_freshness(&fresh);
        if (!gnss_fault && !fresh.all_fresh && gnss_site_log_repair_expected()) {
            if (!req_stale_armed) { req_stale_since = now_ms(); req_stale_armed = true; }
            if (s_repair_logs && age_s(req_stale_since) >= REQ_STALE_S &&
                age_s(last_repair_act) >= DEB_REPAIR_S) {
                ESP_LOGW(TAG, "required RTCM stale for %us -> log repair",
                         (unsigned)age_s(req_stale_since));
                last_repair_act = now_ms();
                s_repair_logs();
            }
        } else {
            req_stale_armed = false;
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
            last_gnss_act = now_ms();
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
