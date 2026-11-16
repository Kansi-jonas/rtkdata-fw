#include "gnss_site.h"
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t gnss_site_crc32(const uint8_t *bytes, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? UINT32_C(0xedb88320) : 0);
    }
    return crc;
}

static int split(char *text, char **fields, size_t capacity) {
    size_t n = 0;
    do {
        if (n == capacity) return -1;
        fields[n++] = text;
        char *comma = strchr(text, ',');
        if (!comma) break;
        *comma = '\0';
        text = comma + 1;
    } while (true);
    return (int)n;
}

static bool unsigned_value(const char *s, uint32_t limit, uint32_t *out) {
    if (!s || !*s) return false;
    uint32_t value = 0;
    for (; *s; ++s) {
        if (*s < '0' || *s > '9') return false;
        uint32_t digit = (uint32_t)(*s - '0');
        if (digit > limit || value > (limit - digit) / 10) return false;
        value = value * 10 + digit;
    }
    *out = value;
    return true;
}

static bool real_value(const char *s, double minimum, double maximum, double *out) {
    if (!s || !*s || *s == ' ' || *s == '\t') return false;
    errno = 0;
    char *end;
    double value = strtod(s, &end);
    if (errno == ERANGE || end == s || *end || !isfinite(value) ||
        value < minimum || value > maximum) return false;
    *out = value;
    return true;
}

bool gnss_site_parse_basepos(const char *line, size_t len, gnss_site_sample_t *out) {
    if (!line || !out || len >= GNSS_SITE_LINE_MAX) return false;
    while (len && (line[len-1] == '\r' || line[len-1] == '\n')) --len;
    if (len < 20 || memcmp(line, "#BASEPOSA,", 10)) return false;
    size_t star = len - 9;
    if (line[star] != '*') return false;
    uint32_t wanted = 0;
    for (size_t i = star + 1; i < len; ++i) {
        unsigned char ch = (unsigned char)line[i];
        unsigned digit;
        if (ch >= '0' && ch <= '9') digit = ch - '0';
        else if (ch >= 'a' && ch <= 'f') digit = ch - 'a' + 10;
        else if (ch >= 'A' && ch <= 'F') digit = ch - 'A' + 10;
        else return false;
        wanted = (wanted << 4) | digit;
    }
    if (gnss_site_crc32((const uint8_t *)line + 1, star - 1) != wanted) return false;
    char copy[GNSS_SITE_LINE_MAX];
    memcpy(copy, line + 1, star - 1);
    copy[star - 1] = '\0';
    char *body = strchr(copy, ';');
    if (!body) return false;
    *body++ = '\0';
    char *header[12], *field[40];
    int header_count = split(copy, header, 12), count = split(body, field, 40);
    if (header_count != 10 || count < 30 || strcmp(header[0], "BASEPOSA") ||
        strcmp(header[2], "GPS")) return false;
    gnss_site_sample_t value = {0};
    uint32_t week, tow, satellites;
    double hgt, undulation, lat_sigma, lon_sigma, hgt_sigma, age;
    if (!unsigned_value(header[4], 65535, &week) ||
        !unsigned_value(header[5], 604799999, &tow)) return false;
    value.epoch_ms = (uint64_t)week * UINT64_C(604800000) + tow;
    /* No-solution records can contain unavailable numeric fields. Their
     * checksummed, correctly framed status proves logging is alive, without
     * contributing any coordinate/quality evidence. */
    if (strcmp(field[0], "SOL_COMPUTED")) {
        *out = value;
        return true;
    }
    if (
        !unsigned_value(field[14], 255, &satellites) ||
        !real_value(field[2], -90, 90, &value.lat) ||
        !real_value(field[3], -180, 180, &value.lon) ||
        !real_value(field[4], -1000, 10000, &hgt) ||
        !real_value(field[5], -1000, 1000, &undulation) ||
        !real_value(field[7], 0, 100000, &lat_sigma) ||
        !real_value(field[8], 0, 100000, &lon_sigma) ||
        !real_value(field[9], 0, 100000, &hgt_sigma) ||
        !real_value(field[12], 0, 1000000, &age)) return false;
    value.h = hgt + undulation;
    value.h_sigma_mm = (uint32_t)ceil(hypot(lat_sigma, lon_sigma) * 1000);
    value.v_sigma_mm = (uint32_t)ceil(hgt_sigma * 1000);
    value.solution_age_ms = (uint32_t)ceil(age * 1000);
    value.sats = (uint8_t)satellites;
    value.quality = !strcmp(header[3], "FINE") && week > 0 &&
        !strcmp(field[0], "SOL_COMPUTED") && !strcmp(field[1], "SINGLE") &&
        !strcmp(field[6], "WGS84") && !(value.lat == 0 && value.lon == 0) &&
        value.h >= -500 && value.h <= 9000 && satellites >= 6 &&
        lat_sigma > 0 && lon_sigma > 0 && hgt_sigma > 0 &&
        value.h_sigma_mm <= GNSS_SITE_H_SIGMA_MAX_MM &&
        value.v_sigma_mm <= GNSS_SITE_V_SIGMA_MAX_MM &&
        value.solution_age_ms <= GNSS_SITE_MAX_AGE_MS;
    *out = value;
    return true;
}

void gnss_site_monitor_init(gnss_site_monitor_t *monitor) {
    memset(monitor, 0, sizeof(*monitor));
}

static void accept_sample(gnss_site_monitor_t *monitor, gnss_site_sample_t sample,
                           uint32_t now_ms) {
    if (monitor->epoch_seen && sample.epoch_ms <= monitor->last_epoch_ms) return;
    if (monitor->epoch_seen &&
        ((uint32_t)(now_ms - monitor->latest.received_ms) > GNSS_SITE_MAX_AGE_MS ||
         sample.epoch_ms - monitor->last_epoch_ms > GNSS_SITE_MAX_AGE_MS))
        monitor->count = monitor->head = 0;
    monitor->epoch_seen = true;
    monitor->last_epoch_ms = sample.epoch_ms;
    sample.received_ms = now_ms;
    monitor->latest = sample;
    if (++monitor->seq == 0) ++monitor->seq;
    if (!sample.quality) {
        monitor->count = monitor->head = 0;
        return;
    }
    monitor->sample[monitor->head] = sample;
    monitor->head = (uint16_t)((monitor->head + 1) % GNSS_SITE_WINDOW);
    if (monitor->count < GNSS_SITE_WINDOW) ++monitor->count;
}

bool gnss_site_monitor_feed(gnss_site_monitor_t *monitor, const uint8_t *bytes,
                            size_t len, uint32_t now_ms) {
    bool received = false;
    if (!monitor || !bytes) return false;
    for (size_t i = 0; i < len; ++i) {
        unsigned char ch = bytes[i];
        if (ch == '#') {
            monitor->line_len = 0;
            monitor->line[monitor->line_len++] = '#';
        } else if (!monitor->line_len) {
            continue;
        } else if (ch == '\n') {
            gnss_site_sample_t sample;
            if (gnss_site_parse_basepos(monitor->line, monitor->line_len, &sample)) {
                received = true;
                accept_sample(monitor, sample, now_ms);
            }
            monitor->line_len = 0;
        } else if ((ch < 32 && ch != '\r') || ch > 126 ||
                   monitor->line_len >= sizeof(monitor->line) - 1) {
            monitor->line_len = 0;
        } else {
            monitor->line[monitor->line_len++] = (char)ch;
        }
    }
    return received;
}

uint32_t gnss_site_horizontal_mm(double lat1, double lon1, double lat2, double lon2) {
    const double rad = 0.017453292519943295;
    double p = (lat2 - lat1) * rad / 2, l = (lon2 - lon1) * rad / 2;
    double a = sin(p)*sin(p) + cos(lat1*rad)*cos(lat2*rad)*sin(l)*sin(l);
    if (a > 1) a = 1;
    if (a < 0) a = 0;
    double mm = 12742000000.0 * atan2(sqrt(a), sqrt(1-a));
    return mm >= UINT32_MAX ? UINT32_MAX : (uint32_t)ceil(mm);
}

static uint64_t rtcm_bits(const uint8_t *p, unsigned pos, unsigned n) {
    uint64_t v = 0;
    for (unsigned i = pos; i < pos + n; ++i) v = (v << 1) | ((p[i >> 3] >> (7 - (i & 7))) & 1u);
    return v;
}

static int64_t rtcm_sbits(const uint8_t *p, unsigned pos, unsigned n) {
    uint64_t v = rtcm_bits(p, pos, n);
    return (v >> (n - 1)) ? (int64_t)(v - (UINT64_C(1) << n)) : (int64_t)v;
}

/* RTCM 10403.3 message 1005/1006: DF002 type (12), DF003 station (12),
 * DF021 ITRF year (6), DF022-DF024 and DF141 (4 x 1), DF025 X (38, 0.1 mm),
 * DF142, DF001 (2 x 1), DF026 Y (38), DF364 (2), DF027 Z (38): 152 bits. */
bool gnss_site_parse_arp(const uint8_t *frame, size_t len, double *lat, double *lon) {
    if (!frame || !lat || !lon || len < 6 || frame[0] != 0xD3) return false;
    size_t payload = ((size_t)(frame[1] & 0x03) << 8) | frame[2];
    if (payload + 6 != len || payload < 19) return false;
    const uint8_t *p = frame + 3;
    unsigned type = (unsigned)rtcm_bits(p, 0, 12);
    /* 1005 is 152 bits, 1006 adds the 16-bit antenna height (L4) */
    if (!(type == 1005 && payload == 19) && !(type == 1006 && payload == 21)) return false;
    double x = rtcm_sbits(p, 34, 38) * 1e-4;
    double y = rtcm_sbits(p, 74, 38) * 1e-4;
    double z = rtcm_sbits(p, 114, 38) * 1e-4;
    double r = sqrt(x*x + y*y + z*z);
    if (r < 6300000.0 || r > 6400000.0) return false;   /* not on the Earth's surface */
    const double a = 6378137.0, e2 = 6.69437999014e-3;   /* WGS84 */
    double horizontal = hypot(x, y);
    double phi = atan2(z, horizontal * (1 - e2));
    for (int i = 0; i < 6; ++i) {
        double s = sin(phi), n = a / sqrt(1 - e2 * s * s);
        double h = horizontal > 1.0 ? horizontal / cos(phi) - n : fabs(z) - n * (1 - e2);
        phi = atan2(z, horizontal * (1 - e2 * n / (n + h)));
    }
    *lat = phi * 57.29577951308232;
    *lon = atan2(y, x) * 57.29577951308232;
    return isfinite(*lat) && isfinite(*lon);
}

bool gnss_site_displaced(double lat, double lon, uint32_t sigma,
                         double ref_lat, double ref_lon) {
    uint32_t threshold = sigma > UINT32_MAX / 5 ? UINT32_MAX : sigma * 5;
    if (threshold < 10000) threshold = 10000;
    return gnss_site_horizontal_mm(lat, lon, ref_lat, ref_lon) > threshold;
}

static double median(double *values, unsigned count) {
    for (unsigned i = 1; i < count; ++i) {
        double v = values[i];
        unsigned j = i;
        while (j && values[j-1] > v) { values[j] = values[j-1]; --j; }
        values[j] = v;
    }
    return count % 2 ? values[count/2] : (values[count/2-1] + values[count/2])/2;
}

void gnss_site_monitor_snapshot(const gnss_site_monitor_t *monitor, uint32_t now_ms,
                                gnss_site_status_t *out) {
    memset(out, 0, sizeof(*out));
    out->seq = monitor->seq;
    out->age_ms = UINT32_MAX;
    if (!monitor->epoch_seen) return;
    uint64_t age = (uint32_t)(now_ms - monitor->latest.received_ms);
    age += monitor->latest.solution_age_ms;
    out->age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    out->lat_e7 = (int32_t)llround(monitor->latest.lat * 1e7);
    out->lon_e7 = (int32_t)llround(monitor->latest.lon * 1e7);
    out->h_mm = (int32_t)llround(monitor->latest.h * 1000);
    out->h_sigma_mm = monitor->latest.h_sigma_mm;
    out->v_sigma_mm = monitor->latest.v_sigma_mm;
    out->sats = monitor->latest.sats;
    out->samples = monitor->count;
    if (!monitor->count) return;
    double values[GNSS_SITE_WINDOW];
    for (unsigned i = 0; i < monitor->count; ++i) values[i] = monitor->sample[i].lat;
    double lat = median(values, monitor->count);
    for (unsigned i = 0; i < monitor->count; ++i) {
        double d = monitor->sample[i].lon - monitor->latest.lon;
        if (d > 180) d -= 360;
        if (d < -180) d += 360;
        values[i] = monitor->latest.lon + d;
    }
    double lon = median(values, monitor->count);
    if (lon > 180) lon -= 360;
    if (lon < -180) lon += 360;
    for (unsigned i = 0; i < monitor->count; ++i) values[i] = monitor->sample[i].h;
    double h = median(values, monitor->count);
    out->lat_e7 = (int32_t)llround(lat * 1e7);
    out->lon_e7 = (int32_t)llround(lon * 1e7);
    out->h_mm = (int32_t)llround(h * 1000);
    for (unsigned i = 0; i < monitor->count; ++i) {
        const gnss_site_sample_t *sample = &monitor->sample[i];
        uint32_t span = (uint32_t)(monitor->latest.received_ms - sample->received_ms);
        if (span > out->span_ms) out->span_ms = span;
        uint32_t spread = gnss_site_horizontal_mm(lat, lon, sample->lat, sample->lon);
        if (spread > out->h_sigma_mm) out->h_sigma_mm = spread;
        if (sample->h_sigma_mm > out->h_sigma_mm) out->h_sigma_mm = sample->h_sigma_mm;
        if (sample->v_sigma_mm > out->v_sigma_mm) out->v_sigma_mm = sample->v_sigma_mm;
        if (sample->sats < out->sats) out->sats = sample->sats;
    }
    out->valid = monitor->latest.quality && out->samples >= GNSS_SITE_MIN_SAMPLES &&
        out->span_ms >= GNSS_SITE_MIN_SPAN_MS && out->age_ms <= GNSS_SITE_MAX_AGE_MS &&
        out->h_sigma_mm <= GNSS_SITE_H_SIGMA_MAX_MM && out->v_sigma_mm <= GNSS_SITE_V_SIGMA_MAX_MM;
}

const char *gnss_site_mode_name(gnss_site_mode_t mode) {
    switch (mode) {
        case GNSS_SITE_SURVEY: return "survey";
        case GNSS_SITE_FIXED: return "fixed";
        case GNSS_SITE_HOLD: return "hold";
        default: return "unknown";
    }
}

bool gnss_site_desired_valid(const gnss_site_desired_t *d) {
    if (!d || d->magic != GNSS_SITE_NVS_MAGIC || d->revision == 0 ||
        (d->mode != GNSS_SITE_SURVEY && d->mode != GNSS_SITE_FIXED)) return false;
    if (d->mode == GNSS_SITE_SURVEY)
        return d->lat_e9 == 0 && d->lon_e9 == 0 && d->h_mm == 0;
    return d->lat_e9 >= INT64_C(-90000000000) && d->lat_e9 <= INT64_C(90000000000) &&
        d->lon_e9 >= INT64_C(-180000000000) && d->lon_e9 <= INT64_C(180000000000) &&
        (d->lat_e9 != 0 || d->lon_e9 != 0) && d->h_mm >= -500000 && d->h_mm <= 9000000;
}

bool gnss_site_control_matches(const gnss_site_desired_t *d, const gnss_site_control_t *c) {
    return gnss_site_desired_valid(d) && d->revision == c->revision &&
        d->mode == (uint32_t)c->mode && d->lat_e9 == c->lat_e9 &&
        d->lon_e9 == c->lon_e9 && d->h_mm == c->h_mm;
}

int gnss_site_canonical(char *dst, size_t cap, const gnss_site_status_t *s) {
    int n = snprintf(dst, cap,
        "%" PRIu32 "|%" PRIu32 "|%u|%" PRId32 "|%" PRId32 "|%" PRId32
        "|%" PRIu32 "|%" PRIu32 "|%u|%u|%" PRIu32 "|%" PRIu32
        "|%" PRIu32 "|%" PRIu32 "|%s|%u",
        s->boot_id, s->seq, s->valid ? 1U : 0U, s->lat_e7, s->lon_e7, s->h_mm,
        s->h_sigma_mm, s->v_sigma_mm, (unsigned)s->sats, (unsigned)s->samples,
        s->span_ms, s->age_ms, s->desired_revision, s->applied_revision,
        gnss_site_mode_name(s->mode), s->upload_allowed ? 1U : 0U);
    return n < 0 || (size_t)n >= cap ? -1 : n;
}

int gnss_site_json(char *dst, size_t cap, const gnss_site_status_t *s) {
    int n = snprintf(dst, cap,
        "{\"v\":1,\"boot_id\":%" PRIu32 ",\"seq\":%" PRIu32 ",\"valid\":%s,"
        "\"lat_e7\":%" PRId32 ",\"lon_e7\":%" PRId32 ",\"h_mm\":%" PRId32 ","
        "\"h_sigma_mm\":%" PRIu32 ",\"v_sigma_mm\":%" PRIu32 ",\"sats\":%u,"
        "\"samples\":%u,\"span_ms\":%" PRIu32 ",\"age_ms\":%" PRIu32 ","
        "\"desired_revision\":%" PRIu32 ",\"applied_revision\":%" PRIu32 ","
        "\"mode\":\"%s\",\"upload_allowed\":%s}",
        s->boot_id, s->seq, s->valid ? "true" : "false", s->lat_e7, s->lon_e7, s->h_mm,
        s->h_sigma_mm, s->v_sigma_mm, (unsigned)s->sats, (unsigned)s->samples,
        s->span_ms, s->age_ms, s->desired_revision, s->applied_revision,
        gnss_site_mode_name(s->mode), s->upload_allowed ? "true" : "false");
    return n < 0 || (size_t)n >= cap ? -1 : n;
}
