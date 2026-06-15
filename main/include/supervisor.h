/*
 * RTKdata reference firmware - data-path health supervisor.
 *
 * Owns end-to-end liveness of the GNSS -> caster path. The stock firmware only
 * had local recoveries (Wi-Fi reconnect, socket reconnect on a send error) and
 * nothing that supervised "is RTCM actually reaching the caster", which is why
 * stations went dark and needed a manual restart. This module closes that gap
 * with three monotonic liveness timestamps and an escalation ladder.
 *
 * Decoupling: the supervisor does not include the gnss/ntrip/wifi modules. The
 * concrete recovery actions are registered as callbacks by main.c.
 *
 * License: GPLv3 (see LICENSE).
 */
#ifndef RTKDATA_SUPERVISOR_H
#define RTKDATA_SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t uptime_s;
    uint32_t gnss_silent_s;    // seconds since last GNSS byte (0 = fresh)
    uint32_t caster_silent_s;  // seconds since last successful caster send
    uint32_t ip_down_s;        // seconds STA has had no IP (0 = up)
    bool     gnss_ok;
    bool     caster_ok;
    bool     link_ok;
    uint32_t recoveries_gnss;
    uint32_t recoveries_caster;
    uint32_t recoveries_wifi;
    uint32_t reboots_supervised; // count persisted across boots (NVS)
} supervisor_health_t;

typedef void (*supervisor_recovery_fn)(void);

// Start the supervisor task. Call after config/uart/wifi/ntrip are up.
void supervisor_init(void);

// Hot-path liveness notes. Cheap (single timestamp store), safe to call often.
void supervisor_note_gnss_rx(void);          // from the GNSS UART read path
void supervisor_note_caster_tx(int bytes);   // from a *successful* caster send
void supervisor_note_sta_ip(bool up);        // from the Wi-Fi STA IP up/down events

// Register concrete recovery actions (any may be NULL to skip that rung).
void supervisor_set_recovery(supervisor_recovery_fn gnss_reset,
                             supervisor_recovery_fn caster_reconnect,
                             supervisor_recovery_fn wifi_restart);

// Snapshot for the provisioning heartbeat / local status endpoint.
void supervisor_health(supervisor_health_t *out);

#endif // RTKDATA_SUPERVISOR_H
