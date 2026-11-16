/*
 * RTKdata reference firmware - UM980 response capture.
 *
 * The UM980 sends command responses ("$command,...*xx") and ASCII logs
 * ("#VERSIONA,...*xxxxxxxx") on the same UART as the RTCM stream. The capture
 * keeps only complete ASCII lines that start with '$' or '#', so a burst of
 * binary RTCM can no longer push a response out before it is read. When the
 * buffer is full the oldest whole lines are dropped, never the whole buffer.
 *
 * No ESP-IDF dependency: host-tested in tests/host/test_gnss_capture.c. The
 * caller provides the locking.
 *
 * License: GPLv3 (see LICENSE).
 */
#ifndef RTKDATA_GNSS_CAPTURE_H
#define RTKDATA_GNSS_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GNSS_CAP_SZ        1024   // committed lines, NUL-terminated
#define GNSS_CAP_LINE_MAX  384    // longest line kept; #VERSIONA is about 230

typedef struct {
    char     buf[GNSS_CAP_SZ];
    size_t   len;
    char     line[GNSS_CAP_LINE_MAX];
    size_t   line_len;
    bool     in_line;
    uint32_t lines_dropped;       // too long, or evicted to make room
} gnss_cap_t;

// Empty the buffer and forget any line in progress.
void gnss_cap_reset(gnss_cap_t *c);

// Feed raw UART bytes (RTCM and ASCII mixed). Bounded work: O(n) plus at most
// one buffer shift per committed line.
void gnss_cap_feed(gnss_cap_t *c, const uint8_t *data, size_t n);

#endif // RTKDATA_GNSS_CAPTURE_H
