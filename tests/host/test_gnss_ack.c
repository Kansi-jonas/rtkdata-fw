/* Host tests for the UM980 command-response parser. Every case here is taken
 * from a real bench capture or from a defect that shipped:
 *   - the documented reject grammar puts PARSING FAIL BEFORE the command, so a
 *     "verdict must follow the echo" matcher missed every reject;
 *   - a global "response: OK" search let a late OK for command A confirm B;
 *   - binary RTCM shares the buffer, so NUL bytes must not end the search.
 * gcc -Imain/include tests/host/test_gnss_ack.c main/gnss_ack.c */

#include <stdio.h>
#include <string.h>

#include "gnss_ack.h"

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { g_fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_V(buf, n, cmd, want) do { \
    gnss_ack_verdict_t _v = gnss_ack_scan((buf), (n), (cmd)); \
    g_checks++; \
    if (_v != (want)) { \
        g_fails++; \
        printf("FAIL %s:%d: scan(%s) = %d, want %d\n", __FILE__, __LINE__, (cmd), _v, (want)); \
    } \
} while (0)

/* Real captures (bench, UM980 R4.10Build11833, 2026-08-03). */
static const char OK_VERSIONA[] =
    "$command,VERSIONA,response: OK*45\r\n";
static const char OK_RTCM1077[] =
    "$command,rtcm1077,com1,1,response: OK*7a\r\n";
/* VERBATIM from the device, including the double space and the uppercased
 * trailing token. Note the command IS echoed before ",response:" here, which
 * contradicts docs/UM980-config-research.md. The hardware wins. */
static const char FAIL_RTCM1230[] =
    "$command,rtcm1230,com1,10,response: PARSING FAILD NO MATCHING FUNC  RTCM1230*11\r\n";
/* The documented-but-unobserved form, supported as a fallback. */
static const char FAIL_DOCFORM[] =
    "$command,response: PARSING FAILD NO MATCHING FUNC rtcm1230,com1,10*3b\r\n";

static void test_success_grammar(void) {
    CHECK_V(OK_VERSIONA, strlen(OK_VERSIONA), "VERSIONA", GNSS_ACK_OK);
    CHECK_V(OK_RTCM1077, strlen(OK_RTCM1077), "rtcm1077,com1,1", GNSS_ACK_OK);
    /* an OK for a DIFFERENT command is not our verdict */
    CHECK_V(OK_RTCM1077, strlen(OK_RTCM1077), "rtcm1087,com1,1", GNSS_ACK_NONE);
    CHECK_V(OK_VERSIONA, strlen(OK_VERSIONA), "rtcm1077,com1,1", GNSS_ACK_NONE);
}

static void test_failure_grammar(void) {
    /* The regression that shipped twice: this MUST be a reject, not a timeout. */
    CHECK_V(FAIL_RTCM1230, strlen(FAIL_RTCM1230), "rtcm1230,com1,10", GNSS_ACK_REJECTED);
    /* and it is not a verdict about some other command */
    CHECK_V(FAIL_RTCM1230, strlen(FAIL_RTCM1230), "rtcm1077,com1,1", GNSS_ACK_NONE);

    /* the documented form still parses, for other firmware builds */
    CHECK_V(FAIL_DOCFORM, strlen(FAIL_DOCFORM), "rtcm1230,com1,10", GNSS_ACK_REJECTED);
    CHECK_V(FAIL_DOCFORM, strlen(FAIL_DOCFORM), "rtcm1077,com1,1", GNSS_ACK_NONE);

    /* a reject must never read as OK just because "OK" appears elsewhere */
    const char fail_with_ok_word[] =
        "$command,MASK 10.0,response: PARSING FAILD NO MATCHING FUNC  MASKOK*01\r\n";
    CHECK_V(fail_with_ok_word, strlen(fail_with_ok_word), "MASK 10.0", GNSS_ACK_REJECTED);
}

static void test_cross_command_confusion(void) {
    /* FAIL(B) followed by a late OK(A): B must stay rejected, A must be OK,
     * and neither may leak into the other. */
    char buf[512];
    int n = snprintf(buf, sizeof(buf), "%s%s", FAIL_RTCM1230, OK_VERSIONA);
    CHECK_V(buf, (size_t)n, "rtcm1230,com1,10", GNSS_ACK_REJECTED);
    CHECK_V(buf, (size_t)n, "VERSIONA", GNSS_ACK_OK);

    /* reverse order, same requirement */
    n = snprintf(buf, sizeof(buf), "%s%s", OK_VERSIONA, FAIL_RTCM1230);
    CHECK_V(buf, (size_t)n, "rtcm1230,com1,10", GNSS_ACK_REJECTED);
    CHECK_V(buf, (size_t)n, "VERSIONA", GNSS_ACK_OK);
}

static void test_binary_interleaving(void) {
    /* RTCM bytes including NUL in front of the response: a strstr-based
     * matcher stopped at the NUL and reported a timeout. */
    char buf[512];
    size_t n = 0;
    const unsigned char rtcm[] = { 0xD3, 0x00, 0x13, 0x3E, 0xD7, 0x00, 0x00, 0x42 };
    memcpy(buf, rtcm, sizeof(rtcm)); n += sizeof(rtcm);
    memcpy(buf + n, OK_RTCM1077, strlen(OK_RTCM1077)); n += strlen(OK_RTCM1077);
    CHECK_V(buf, n, "rtcm1077,com1,1", GNSS_ACK_OK);

    /* NUL inside the record's own tail must not matter either */
    n = 0;
    memcpy(buf + n, FAIL_RTCM1230, strlen(FAIL_RTCM1230)); n += strlen(FAIL_RTCM1230);
    memcpy(buf + n, rtcm, sizeof(rtcm)); n += sizeof(rtcm);
    CHECK_V(buf, n, "rtcm1230,com1,10", GNSS_ACK_REJECTED);
}

static void test_partial_records(void) {
    /* A record without its checksum delimiter is still arriving: no verdict. */
    const char partial_ok[] = "$command,rtcm1077,com1,1,response: OK";
    CHECK_V(partial_ok, strlen(partial_ok), "rtcm1077,com1,1", GNSS_ACK_NONE);

    const char partial_fail[] = "$command,response: PARSING FAIL";
    CHECK_V(partial_fail, strlen(partial_fail), "rtcm1230,com1,10", GNSS_ACK_NONE);

    /* every prefix of a real reject either yields NONE or the right verdict,
     * never a wrong one */
    size_t full = strlen(FAIL_RTCM1230);
    for (size_t k = 0; k <= full; k++) {
        gnss_ack_verdict_t v = gnss_ack_scan(FAIL_RTCM1230, k, "rtcm1230,com1,10");
        if (v != GNSS_ACK_NONE && v != GNSS_ACK_REJECTED) {
            CHECK(0 && "prefix produced a wrong verdict");
            return;
        }
    }
    g_checks++;

    /* same for the success grammar */
    full = strlen(OK_RTCM1077);
    for (size_t k = 0; k <= full; k++) {
        gnss_ack_verdict_t v = gnss_ack_scan(OK_RTCM1077, k, "rtcm1077,com1,1");
        if (v != GNSS_ACK_NONE && v != GNSS_ACK_OK) {
            CHECK(0 && "prefix produced a wrong verdict");
            return;
        }
    }
    g_checks++;
}

static void test_prefix_commands_not_confused(void) {
    /* "rtcm1077,com1,1" must not be satisfied by a record for
     * "rtcm1077,com1,10" (exact span comparison, not a prefix match). */
    const char ok10[] = "$command,rtcm1077,com1,10,response: OK*11\r\n";
    CHECK_V(ok10, strlen(ok10), "rtcm1077,com1,1", GNSS_ACK_NONE);
    CHECK_V(ok10, strlen(ok10), "rtcm1077,com1,10", GNSS_ACK_OK);

    const char fail10[] =
        "$command,response: PARSING FAILD NO MATCHING FUNC rtcm1077,com1,10*22\r\n";
    CHECK_V(fail10, strlen(fail10), "rtcm1077,com1,1", GNSS_ACK_NONE);
    CHECK_V(fail10, strlen(fail10), "rtcm1077,com1,10", GNSS_ACK_REJECTED);
}

static void test_last_verdict_wins(void) {
    /* A retry that first failed and then succeeded must read as OK. */
    char buf[512];
    int n = snprintf(buf, sizeof(buf),
        "$command,response: PARSING FAILD NO MATCHING FUNC MASK 10.0*01\r\n"
        "$command,MASK 10.0,response: OK*02\r\n");
    CHECK_V(buf, (size_t)n, "MASK 10.0", GNSS_ACK_OK);
}

static void test_degenerate_inputs(void) {
    CHECK_V(NULL, 0, "x", GNSS_ACK_NONE);
    CHECK_V("", 0, "x", GNSS_ACK_NONE);
    CHECK(gnss_ack_scan(OK_VERSIONA, strlen(OK_VERSIONA), NULL) == GNSS_ACK_NONE);
    const char noise[] = "random serial noise without any command record";
    CHECK_V(noise, strlen(noise), "VERSIONA", GNSS_ACK_NONE);
}

int main(void) {
    test_success_grammar();
    test_failure_grammar();
    test_cross_command_confusion();
    test_binary_interleaving();
    test_partial_records();
    test_prefix_commands_not_confused();
    test_last_verdict_wins();
    test_degenerate_inputs();
    printf("%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
