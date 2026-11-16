/*
 * RTKdata reference firmware - UM980 GNSS configuration.
 *
 * SOTA reference-base config: ACK-gated (reads the UM980 "$command,...,response: OK"
 * echo and retries) instead of the stock blind open loop, with 1005 + 1033 + 1230
 * and MSM7 observables. See docs/UM980-config-research.md for the rationale + sources.
 *
 * License: GPLv3 (see LICENSE).
 */
#ifndef RTKDATA_GNSS_H
#define RTKDATA_GNSS_H

#include <stdbool.h>
#include <stddef.h>
#include "gnss_site.h"

// Hardware-reset the UM980 and apply the reference-base config. Called at boot.
void gnss_init(void);

// Apply the reference-base RTCM config (ACK-gated). Provisional survey-in base.
void config_gnss_base(void);

// Query (VERSIONA) + log the UM980 firmware version/build. Called at boot for
// inventory; the response carries the model + firmware build string.
void gnss_log_version(void);

// Same VERSIONA query, answer copied into out (CR/LF as spaces). Returns the
// number of bytes the receiver sent (0 = no answer). Used by the test page.
size_t gnss_query_version(char *out, size_t out_sz);

// Board test page: hardware reset pulse + reference-base config, regardless
// of the supervisor's health verdict.
void gnss_test_reset(void);

// Switch the base from survey-in to a FIXED position computed by the IE
// (lat/lon in degrees, height in metres, ITRF2020). Returns true if the UM980
// acknowledged. Called by the provisioning client once the position converges.
bool gnss_set_fixed_base(double lat_deg, double lon_deg, double height_m);
bool gnss_start_site_survey(void);
bool gnss_apply_site_revision(const gnss_site_desired_t *desired);

// CONFIG_RTK_SITE_LEGACY only. Periodic: relocation drop (fixed coordinate
// left for 5 min -> forget it, survey-in) and unknown-mode reconfiguration.
void gnss_legacy_tick(void);
// Metres between the receiver's own position and the coordinate that was
// last refused or dropped; 0 while the unit is where its coordinate says.
uint32_t gnss_legacy_moved_m(void);

// Synchronous command/response capture, called from uart_task for every UART
// chunk. Cheap no-op unless a transaction is capturing.
void gnss_ingest_uart(const uint8_t *data, size_t len);

// Hardware-reset + reconfigure the UM980. Registered as the supervisor's GNSS
// recovery action (fires when no GNSS bytes have arrived for a while).
void gnss_recover(void);

// Reconfigure after the receiver was reset from OUTSIDE the firmware (the
// K_EVK_V4 button resets the UM980 in hardware on every press).
void gnss_after_external_reset(void);

// Re-enable required RTCM messages (1005/1033/1077/1087) that the receiver is
// not producing while the stream is expected. Plain log enables: no MODE, no
// reset. The supervisor's log-repair action.
void gnss_repair_required_logs(void);

#endif // RTKDATA_GNSS_H
