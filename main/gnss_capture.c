/* See gnss_capture.h. License: GPLv3 (see LICENSE). */

#include <string.h>

#include "gnss_capture.h"

void gnss_cap_reset(gnss_cap_t *c) {
    c->len = 0;
    c->buf[0] = '\0';
    c->line_len = 0;
    c->in_line = false;
}

/* Append one complete line. If it does not fit, drop the oldest whole lines:
 * the parsers look for record starts, so a cut-off first line would only be
 * noise, and clearing everything (the old behaviour) lost the very response
 * a command was waiting for. */
static void commit_line(gnss_cap_t *c) {
    size_t need = c->line_len;
    if (need == 0) return;
    size_t room = GNSS_CAP_SZ - 1;            // keep one byte for the NUL
    if (c->len + need > room) {
        size_t drop = c->len + need - room;
        // Round up to the end of the line that contains byte drop - 1.
        const char *nl = memchr(c->buf + drop - 1, '\n', c->len - (drop - 1));
        drop = nl ? (size_t)(nl - c->buf) + 1 : c->len;
        memmove(c->buf, c->buf + drop, c->len - drop);
        c->len -= drop;
        c->lines_dropped++;
    }
    memcpy(c->buf + c->len, c->line, need);
    c->len += need;
    c->buf[c->len] = '\0';
}

void gnss_cap_feed(gnss_cap_t *c, const uint8_t *data, size_t n) {
    if (c == NULL || data == NULL) return;
    for (size_t i = 0; i < n; i++) {
        uint8_t b = data[i];
        if (b == '$' || b == '#') {
            // A record start always begins a new line: this also resyncs
            // after a '$' or '#' that was really an RTCM payload byte.
            c->line[0] = (char)b;
            c->line_len = 1;
            c->in_line = true;
        } else if (!c->in_line) {
            continue;                          // RTCM or noise between lines
        } else if (b == '\n') {
            c->line[c->line_len++] = '\n';     // room is reserved below
            commit_line(c);
            c->line_len = 0;
            c->in_line = false;
        } else if (b == '\r' || (b >= 0x20 && b <= 0x7E)) {
            if (c->line_len < GNSS_CAP_LINE_MAX - 1) {
                c->line[c->line_len++] = (char)b;
            } else {
                c->in_line = false;            // too long to be a response
                c->line_len = 0;
                c->lines_dropped++;
            }
        } else {
            c->in_line = false;                // binary: this was no line
            c->line_len = 0;
        }
    }
}
