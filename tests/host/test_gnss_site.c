#include "gnss_site.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int checks;
#define CHECK(c) do { ++checks; if (!(c)) { printf("FAIL %d: %s\n", __LINE__, #c); return 1; } } while (0)

static size_t sentence(char *out, unsigned tow, const char *status, const char *type,
                       double lat, double lon, double hs, double vs, unsigned sats) {
    int n = snprintf(out, GNSS_SITE_LINE_MAX,
        "#BASEPOSA,96,GPS,FINE,2440,%u,0,0,18,20;"
        "%s,%s,%.11f,%.11f,40.0000,2.0000,WGS84,%.4f,%.4f,%.4f,"
        "\"0\",0.000,0.000,30,%u,28,0,16,12,01,51,"
        "SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0044,52.887930,0.0082,0.0205,0.0116",
        tow, status, type, lat, lon, hs, hs, vs, sats);
    unsigned crc = gnss_site_crc32((const unsigned char *)out + 1, (size_t)n - 1);
    return (size_t)n + (size_t)snprintf(out+n, GNSS_SITE_LINE_MAX-(size_t)n, "*%08x\r\n", crc);
}

int main(void) {
    /* Manufacturer BASEPOS example: printed +8.4923 is a typo; -8.4923
     * matches the independently supplied CRC80a5f451. No checksum relaxation. */
    const char *vendor = "#BASEPOSA,96,GPS,FINE,2207,289028000,0,0,18,20;"
        "SOL_COMPUTED,SINGLE,40.07899984715,116.23661761328,64.8315,-8.4923,"
        "WGS84,2.8968,2.0472,6.2202,\"0\",0.000,0.000,55,28,28,0,16,12,01,51,"
        "SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0044,52.887930,0.0082,0.0205,0.0116*80a5f451\r\n";
    gnss_site_sample_t sample;
    CHECK(gnss_site_parse_basepos(vendor, strlen(vendor), &sample));
    CHECK(fabs(sample.lat-40.07899984715) < 1e-10);
    CHECK(fabs(sample.h-56.3392) < 1e-8 && sample.sats == 28);
    CHECK(!sample.quality); /* conservative quality limits, not the parser */
    char buf[GNSS_SITE_LINE_MAX];
    size_t len = sentence(buf, 1000, "SOL_COMPUTED", "SINGLE", 52, 13, 1, 2, 12);
    CHECK(gnss_site_parse_basepos(buf, len, &sample) && sample.quality);
    CHECK(sample.h == 42 && sample.h_sigma_mm == 1415 && sample.v_sigma_mm == 2000);
    buf[30] ^= 1;
    CHECK(!gnss_site_parse_basepos(buf, len, &sample));
    buf[30] ^= 1;
    CHECK(!gnss_site_parse_basepos(buf, len-5, &sample));
    len = sentence(buf, 1000, "SOL_COMPUTED", "FIXEDPOS", 52, 13, 1, 2, 12);
    CHECK(gnss_site_parse_basepos(buf, len, &sample) && !sample.quality);
    len = sentence(buf, 1000, "INSUFFICIENT_OBS", "SINGLE", 52, 13, 1, 2, 12);
    CHECK(gnss_site_parse_basepos(buf, len, &sample) && !sample.quality);
    len = sentence(buf, 1000, "SOL_COMPUTED", "SINGLE", 52, 13, 1, 2, 5);
    CHECK(gnss_site_parse_basepos(buf, len, &sample) && !sample.quality);
    len = sentence(buf, 1000, "SOL_COMPUTED", "SINGLE", 52, 13, 4, 2, 12);
    CHECK(gnss_site_parse_basepos(buf, len, &sample) && !sample.quality);
    len = sentence(buf, 1000, "SOL_COMPUTED", "SINGLE", NAN, 13, 1, 2, 12);
    CHECK(!gnss_site_parse_basepos(buf, len, &sample));
    len = sentence(buf, 1000, "SOL_COMPUTED", "SINGLE", 91, 13, 1, 2, 12);
    CHECK(!gnss_site_parse_basepos(buf, len, &sample));

    gnss_site_monitor_t monitor;
    gnss_site_status_t status;
    gnss_site_monitor_init(&monitor);
    len = sentence(buf, 1000, "SOL_COMPUTED", "SINGLE", 52, 13, 1, 2, 12);
    const unsigned char noise[] = {0xd3, 0, 4, '#', 'x', 0, '\n'};
    CHECK(!gnss_site_monitor_feed(&monitor, noise, sizeof(noise), 0));
    for (size_t i = 0; i < len; ++i)
        gnss_site_monitor_feed(&monitor, (const unsigned char *)buf+i, 1, 1000);
    CHECK(monitor.count == 1 && monitor.seq == 1);
    gnss_site_monitor_feed(&monitor, (const unsigned char *)buf, len, 2000);
    CHECK(monitor.count == 1 && monitor.seq == 1); /* same epoch cannot satisfy window */
    for (unsigned i = 2; i <= 61; ++i) {
        len = sentence(buf, i*1000, "SOL_COMPUTED", "SINGLE", 52, 13, 1, 2, 12);
        CHECK(gnss_site_monitor_feed(&monitor, (const unsigned char *)buf, len, i*1000));
        if (i == 60) {
            gnss_site_monitor_snapshot(&monitor, i*1000, &status);
            CHECK(!status.valid && status.span_ms == 59000);
        }
    }
    gnss_site_monitor_snapshot(&monitor, 61000, &status);
    CHECK(status.valid && status.samples == 61 && status.span_ms == 60000);
    CHECK(status.lat_e7 == 520000000 && status.lon_e7 == 130000000 && status.h_mm == 42000);
    gnss_site_monitor_snapshot(&monitor, 66001, &status);
    CHECK(!status.valid);
    /* A quality outlier invalidates the stable window and triggers immediate displacement. */
    len = sentence(buf, 62000, "SOL_COMPUTED", "SINGLE", 52.001, 13, 1, 2, 12);
    gnss_site_monitor_feed(&monitor, (const unsigned char *)buf, len, 62000);
    gnss_site_monitor_snapshot(&monitor, 62000, &status);
    CHECK(!status.valid && status.h_sigma_mm > 3000);
    CHECK(gnss_site_displaced(monitor.latest.lat, monitor.latest.lon,
                              monitor.latest.h_sigma_mm, 52, 13));
    CHECK(!gnss_site_displaced(52.00001, 13, 1415, 52, 13));
    /* New stable location eventually replaces every old sample. */
    for (unsigned i = 63; i <= 126; ++i) {
        len = sentence(buf, i*1000, "SOL_COMPUTED", "SINGLE", 52.001, 13, 1, 2, 12);
        gnss_site_monitor_feed(&monitor, (const unsigned char *)buf, len, i*1000);
    }
    gnss_site_monitor_snapshot(&monitor, 126000, &status);
    CHECK(status.valid && status.samples == GNSS_SITE_WINDOW);
    /* Lost epochs and no-fix results cannot preserve a previously qualified window. */
    len = sentence(buf, 140000, "SOL_COMPUTED", "SINGLE", 52.001, 13, 1, 2, 12);
    gnss_site_monitor_feed(&monitor, (const unsigned char *)buf, len, 140000);
    CHECK(monitor.count == 1);
    len = sentence(buf, 141000, "INSUFFICIENT_OBS", "NONE", 52.001, 13, 1, 2, 12);
    gnss_site_monitor_feed(&monitor, (const unsigned char *)buf, len, 141000);
    CHECK(monitor.count == 0);
    gnss_site_monitor_snapshot(&monitor, 141000, &status);
    CHECK(!status.valid);
    CHECK(gnss_site_horizontal_mm(0, 179.999, 0, -179.999) < 223000);

    gnss_site_desired_t desired = {GNSS_SITE_NVS_MAGIC, 7, GNSS_SITE_SURVEY, 0, 0, 0, 0, 0};
    CHECK(gnss_site_desired_valid(&desired));
    gnss_site_control_t control = {.boot_id=5,.revision=7,.mode=GNSS_SITE_SURVEY};
    CHECK(gnss_site_control_matches(&desired, &control));
    control.revision = 6;
    CHECK(!gnss_site_control_matches(&desired, &control));
    desired.mode = GNSS_SITE_FIXED;
    CHECK(!gnss_site_desired_valid(&desired)); /* Null Island */
    desired.lat_e9 = INT64_C(52000000000);
    desired.lon_e9 = INT64_C(13000000000);
    desired.h_mm = 42000;
    CHECK(gnss_site_desired_valid(&desired));
    desired.h_mm = 9000001;
    CHECK(!gnss_site_desired_valid(&desired));

    status = (gnss_site_status_t){.boot_id=123,.seq=61,.valid=true,
        .lat_e7=520000000,.lon_e7=130000000,.h_mm=42000,.h_sigma_mm=1415,
        .v_sigma_mm=2000,.sats=12,.samples=61,.span_ms=60000,.age_ms=4,
        .desired_revision=7,.applied_revision=7,.mode=GNSS_SITE_FIXED,.upload_allowed=true};
    CHECK(gnss_site_canonical(buf, sizeof(buf), &status) > 0);
    CHECK(!strcmp(buf, "123|61|1|520000000|130000000|42000|1415|2000|12|61|60000|4|7|7|fixed|1"));
    CHECK(gnss_site_json(buf, sizeof(buf), &status) > 0);
    CHECK(strstr(buf, "\"upload_allowed\":true") && strstr(buf, "\"v\":1"));
    CHECK(gnss_site_json(buf, 5, &status) < 0);
    CHECK(gnss_site_canonical(buf, 5, &status) < 0);
    /* RTCM 1005 reference point (review 13.11. L3 #2). Golden: a frame
     * recorded from the Saarland edge box (UM980, station 1008). pyrtcm
     * decodes it to X 4125951.0083, Y 495896.9049, Z 4822713.6169, which is
     * 49.4393264 N 6.8534891 E, the box's known site. */
    {
        static const uint8_t arp[] = {0xd3,0x00,0x13,0x3e,0xd3,0xf0,0x03,0x89,0x9b,0x42,0x29,0x43,0x81,0x27,0x93,0xdc,0xd9,0x0b,0x3a,0x8f,0xb2,0xa9,0x88,0x94,0x86};
        double lat = 0, lon = 0;
        CHECK(gnss_site_parse_arp(arp, sizeof(arp), &lat, &lon));
        CHECK(fabs(lat - 49.43932637) < 1e-7 && fabs(lon - 6.85348914) < 1e-7);
        CHECK(!gnss_site_parse_arp(arp, sizeof(arp) - 1, &lat, &lon));  /* length */
        uint8_t other[sizeof(arp)];
        memcpy(other, arp, sizeof(arp));
        other[4] ^= 0x10;                                               /* type 1005 -> 1004 */
        CHECK(!gnss_site_parse_arp(other, sizeof(other), &lat, &lon));
        memcpy(other, arp, sizeof(arp));
        memset(other + 7, 0, 15);                                       /* ECEF zero */
        CHECK(!gnss_site_parse_arp(other, sizeof(other), &lat, &lon));
        other[0] = 0xD2;
        CHECK(!gnss_site_parse_arp(other, sizeof(other), &lat, &lon));
        /* 1006 = 1005 plus 16 bits antenna height: 21 payload bytes. A
         * 19-byte "1006" is malformed (L4). */
        memcpy(other, arp, sizeof(arp));
        other[4] = (uint8_t)((other[4] & 0x0F) | 0xE0);                 /* type 1006 */
        CHECK(!gnss_site_parse_arp(other, sizeof(other), &lat, &lon));
        uint8_t m1006[sizeof(arp) + 2];
        memcpy(m1006, other, 3 + 19);
        m1006[2] = 21;
        m1006[22] = 0x01; m1006[23] = 0x02;                             /* height */
        memcpy(m1006 + 24, arp + 22, 3);                                /* CRC bytes (not checked here) */
        CHECK(gnss_site_parse_arp(m1006, sizeof(m1006), &lat, &lon));
        CHECK(fabs(lat - 49.43932637) < 1e-7 && fabs(lon - 6.85348914) < 1e-7);
        m1006[4] = (uint8_t)((m1006[4] & 0x0F) | 0xD0);                 /* a 21-byte "1005" */
        CHECK(!gnss_site_parse_arp(m1006, sizeof(m1006), &lat, &lon));
    }
    printf("%d checks passed\n", checks);
    return 0;
}
