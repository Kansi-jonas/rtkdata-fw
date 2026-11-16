/*
 * Board test page (/test.html) for the Ethernet boards. Built only with
 * -DRTK_TEST_PAGE=1 (CONFIG_RTK_TEST_PAGE): it can reset the GNSS, force LEDs
 * and restart the unit, so it never ships in a customer image. Without the
 * option the web server registers neither handler and board_test_init() is
 * empty.
 *
 * License: GPLv3 (see LICENSE).
 */
#ifndef RTKDATA_BOARD_TEST_H
#define RTKDATA_BOARD_TEST_H

#include <esp_http_server.h>

// Count this boot per reset reason (NVS). Call right after config_init(), so
// a crash during the bring-up that follows is counted too.
void board_test_count_boot(void);

// PPS and button counters, revision ID, worker task. Call once in app_main,
// after the button task has configured its GPIO.
void board_test_init(void);

// GET /test/status: one JSON snapshot of everything the page shows.
esp_err_t board_test_status_handler(httpd_req_t *req);

// GET /test/do?a=<action>: gnss_version, gnss_reset, atecc (run on the worker
// task, result in the next status), led (&led=0|1&m=-1..3), clear_boots,
// restart.
esp_err_t board_test_do_handler(httpd_req_t *req);

#endif // RTKDATA_BOARD_TEST_H
