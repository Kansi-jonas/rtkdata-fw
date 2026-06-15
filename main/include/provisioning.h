/*
 * RTKdata reference firmware - zero-touch provisioning client.
 *
 * Enrolls the device with the Integrity Engine (IE), pulls its caster
 * credentials into NVS, and heartbeats its health so the IE can run the
 * 48h DB-gating (a station is created only after ~48h of clean data and a
 * converged, server-computed position). See ARCHITECTURE.md section 4.
 *
 * License: GPLv3 (see LICENSE).
 */
#ifndef RTKDATA_PROVISIONING_H
#define RTKDATA_PROVISIONING_H

#include "cJSON.h"

// Start the enroll/heartbeat task. Call after wifi + ntrip + supervisor are up.
void provisioning_init(void);

// Fill the local /rtk/status object (last IE heartbeat reply + local liveness),
// consumed by the onboarding UI. Safe to call from the web server task.
void provisioning_fill_status(cJSON *root);

#endif // RTKDATA_PROVISIONING_H
