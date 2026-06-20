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

// Hardware-reset the UM980 and apply the reference-base config. Called at boot.
void gnss_init(void);

// Apply the reference-base RTCM config (ACK-gated). Provisional survey-in base.
void config_gnss_base(void);

// Query (VERSIONA) + log the UM980 firmware version/build. Called at boot for
// inventory; the response carries the model + firmware build string.
void gnss_log_version(void);

// Switch the base from survey-in to a FIXED position computed by the IE
// (lat/lon in degrees, height in metres, ITRF2020). Returns true if the UM980
// acknowledged. Called by the provisioning client once the position converges.
bool gnss_set_fixed_base(double lat_deg, double lon_deg, double height_m);

// Hardware-reset + reconfigure the UM980. Registered as the supervisor's GNSS
// recovery action (fires when no GNSS bytes have arrived for a while).
void gnss_recover(void);

#endif // RTKDATA_GNSS_H
