/*
 * RTKdata reference firmware - zero-touch provisioning client.
 *
 * Flow (state machine persisted in NVS, see ARCHITECTURE.md section 4):
 *   UNCLAIMED --enroll--> PROVISIONING --(IE: clean stream)--> MATURING
 *             --(IE: 48h + converged position)--> ACTIVE
 *
 * On first STA-online the device enrolls with the IE (HMAC-signed), writes the
 * returned caster credentials into the inherited NVS NTRIP keys, and triggers the
 * NTRIP server to reconnect. It then heartbeats its supervisor health; the IE
 * replies with {state, progress_pct, matures_in_s, position?, config_delta?}. When
 * the IE returns a converged position the firmware sets a fixed base; when it
 * returns a config_delta it re-applies caster credentials.
 *
 * License: GPLv3 (see LICENSE).
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <esp_http_client.h>
#include <esp_crt_bundle.h>
#include <mbedtls/md.h>
#include <cJSON.h>

#include "provisioning.h"
#include "config.h"
#include "uart.h"
#include "tasks.h"
#include "wifi.h"
#include "gnss.h"
#include "supervisor.h"
#include "interface/ntrip.h"

#define TAG "PROVISION"

/* Enrollment auth: the enroll request is HMAC-signed with a PER-DEVICE key that
 * is provisioned into NVS at flash time (key KEY_RTK_ENROLL_KEY, 64-hex), derived
 * on the flash host from a master key that lives ONLY on the host + the IE, never
 * in firmware. No fleet-wide secret is baked into the .bin (a public .bin / flash
 * dump must not leak a key usable for the whole fleet). The per-station token
 * returned by enroll is used for subsequent heartbeats. See
 * docs/enroll-per-device-key.md for the byte-exact FW<->IE contract. */

#define DEFAULT_POLL_S   30
#define MIN_POLL_S       10

enum rtk_state { RTK_UNCLAIMED = 0, RTK_PROVISIONING = 1, RTK_MATURING = 2, RTK_ACTIVE = 3 };

static char  s_device_id[40] = {0};
static char  s_mac[18]       = {0};
static char  s_ie_host[96]   = {0};
static char  s_token[96]     = {0};
static char  s_enroll_key[80] = {0};   // per-device 64-hex enroll key from NVS
static uint8_t s_key_ver      = 1;     // which master version derived it
static int   s_poll_s        = DEFAULT_POLL_S;
static bool  s_fixed_base_set = false;

/* last status (for /rtk/status), guarded by a mutex */
static SemaphoreHandle_t s_mtx;
static struct {
    char     state[16];
    int      progress_pct;
    int      matures_in_s;
    char     station_id[40];
    int      q_sats;
    int      q_fix;
    bool     pos_valid;
    double   pos_lat, pos_lon, pos_h;
    bool     enrolled;
} s_st;

/* ------------------------------------------------------------------ utils */

static void hmac_sha256_hex(const char *key, const char *msg, char out[65]) {
    unsigned char mac[32];
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!info || mbedtls_md_hmac(info, (const unsigned char *)key, strlen(key),
                                 (const unsigned char *)msg, strlen(msg), mac) != 0) {
        out[0] = '\0';
        return;
    }
    for (int i = 0; i < 32; i++) sprintf(out + i * 2, "%02x", mac[i]);
    out[64] = '\0';
}

/* HTTP POST JSON, returns malloc'd response body (caller frees) or NULL. */
static char *http_post_json(const char *url, const char *body, int *out_status) {
    esp_http_client_config_t cfg = {
        .url               = url,
        .method            = HTTP_METHOD_POST,
        .timeout_ms        = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli) return NULL;

    char *resp = NULL;
    esp_http_client_set_header(cli, "Content-Type", "application/json");

    esp_err_t err = esp_http_client_open(cli, strlen(body));
    if (err == ESP_OK && esp_http_client_write(cli, body, strlen(body)) >= 0) {
        int clen = esp_http_client_fetch_headers(cli);
        if (out_status) *out_status = esp_http_client_get_status_code(cli);

        /* Read the whole body even when the server replies chunked. The IE
         * (Next.js) sends Transfer-Encoding: chunked with no Content-Length, so
         * fetch_headers() returns 0 here; grow the buffer as data arrives rather
         * than trusting clen, or the 200 reply (token + caster creds) is lost. */
        int cap = clen > 0 ? clen + 1 : 512;
        int len = 0;
        resp = malloc(cap);
        if (resp) {
            for (;;) {
                if (len + 1 >= cap) {
                    char *grown = realloc(resp, cap * 2);
                    if (!grown) { free(resp); resp = NULL; break; }
                    resp = grown;
                    cap *= 2;
                }
                int r = esp_http_client_read(cli, resp + len, cap - len - 1);
                if (r <= 0) break;   /* 0 = body complete, <0 = closed/error */
                len += r;
            }
            if (resp) resp[len] = '\0';
        }
    } else {
        ESP_LOGW(TAG, "POST %s failed: %s", url, esp_err_to_name(err));
    }

    esp_http_client_close(cli);
    esp_http_client_cleanup(cli);
    return resp;
}

static void set_str(const char *key, const char *val) {
    if (val) config_set_str((char *)key, (char *)val);
}

/* Write the IE-issued caster credentials into the inherited NTRIP NVS keys. */
static void write_caster_creds(cJSON *caster) {
    cJSON *host = cJSON_GetObjectItem(caster, "host");
    cJSON *port = cJSON_GetObjectItem(caster, "port");
    cJSON *mp   = cJSON_GetObjectItem(caster, "mountpoint");
    cJSON *user = cJSON_GetObjectItem(caster, "username");
    cJSON *pass = cJSON_GetObjectItem(caster, "password");
    if (cJSON_IsString(host)) set_str(KEY_CONFIG_NTRIP_SERVER_HOST, host->valuestring);
    if (cJSON_IsNumber(port)) config_set_u16(KEY_CONFIG_NTRIP_SERVER_PORT, (uint16_t)port->valueint);
    if (cJSON_IsString(mp))   set_str(KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT, mp->valuestring);
    if (cJSON_IsString(user)) set_str(KEY_CONFIG_NTRIP_SERVER_USERNAME, user->valuestring);
    if (cJSON_IsString(pass)) set_str(KEY_CONFIG_NTRIP_SERVER_PASSWORD, pass->valuestring);
    config_set_bool1(KEY_CONFIG_NTRIP_SERVER_ACTIVE, true);
    config_commit();
}

/* True if any caster field in the reply differs from what is already stored, so
 * a repeated config_delta (the IE includes the authoritative caster on every
 * heartbeat) only rewrites NVS + bounces the NTRIP server on an actual change. */
static bool caster_creds_differ(cJSON *caster) {
    char *cur = NULL;
    bool differ = false;

    cJSON *host = cJSON_GetObjectItem(caster, "host");
    if (cJSON_IsString(host)) {
        config_get_str_blob_alloc(CONF_ITEM(KEY_CONFIG_NTRIP_SERVER_HOST), (void **)&cur);
        if (!cur || strcmp(cur, host->valuestring) != 0) differ = true;
        free(cur); cur = NULL;
    }
    cJSON *mp = cJSON_GetObjectItem(caster, "mountpoint");
    if (!differ && cJSON_IsString(mp)) {
        config_get_str_blob_alloc(CONF_ITEM(KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT), (void **)&cur);
        if (!cur || strcmp(cur, mp->valuestring) != 0) differ = true;
        free(cur); cur = NULL;
    }
    cJSON *user = cJSON_GetObjectItem(caster, "username");
    if (!differ && cJSON_IsString(user)) {
        config_get_str_blob_alloc(CONF_ITEM(KEY_CONFIG_NTRIP_SERVER_USERNAME), (void **)&cur);
        if (!cur || strcmp(cur, user->valuestring) != 0) differ = true;
        free(cur); cur = NULL;
    }
    cJSON *pass = cJSON_GetObjectItem(caster, "password");
    if (!differ && cJSON_IsString(pass)) {
        config_get_str_blob_alloc(CONF_ITEM(KEY_CONFIG_NTRIP_SERVER_PASSWORD), (void **)&cur);
        if (!cur || strcmp(cur, pass->valuestring) != 0) differ = true;
        free(cur); cur = NULL;
    }
    cJSON *port = cJSON_GetObjectItem(caster, "port");
    if (!differ && cJSON_IsNumber(port)) {
        if (config_get_u16(CONF_ITEM(KEY_CONFIG_NTRIP_SERVER_PORT)) != (uint16_t)port->valueint) differ = true;
    }
    return differ;
}

/* ------------------------------------------------------------- identity/cfg */

static void load_identity(void) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_mac, sizeof(s_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    char *dev = NULL;
    config_get_str_blob_alloc(CONF_ITEM(KEY_RTK_DEVICE_ID), (void **)&dev);
    if (dev && dev[0]) {
        strlcpy(s_device_id, dev, sizeof(s_device_id));
    } else {
        snprintf(s_device_id, sizeof(s_device_id), "rtk-%02x%02x%02x%02x%02x%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        config_set_str(KEY_RTK_DEVICE_ID, s_device_id);
        config_commit();
    }
    free(dev);

    char *host = NULL;
    config_get_str_blob_alloc(CONF_ITEM(KEY_RTK_IE_HOST), (void **)&host);
    strlcpy(s_ie_host, (host && host[0]) ? host : "integrity-engine.onrender.com", sizeof(s_ie_host));
    free(host);

    char *tok = NULL;
    config_get_str_blob_alloc(CONF_ITEM(KEY_RTK_STATION_TOK), (void **)&tok);
    if (tok) strlcpy(s_token, tok, sizeof(s_token));
    free(tok);

    char *ek = NULL;
    config_get_str_blob_alloc(CONF_ITEM(KEY_RTK_ENROLL_KEY), (void **)&ek);
    if (ek) strlcpy(s_enroll_key, ek, sizeof(s_enroll_key));
    free(ek);
    s_key_ver = config_get_u8(CONF_ITEM(KEY_RTK_KEY_VER));
    if (s_key_ver == 0) s_key_ver = 1;
}

/* ------------------------------------------------------------------ enroll */

static bool enroll(void) {
    if (!s_enroll_key[0]) {
        // No per-device enroll key in NVS -> unprovisioned. Do NOT fall back to
        // any baked secret (there is none). Stays UNCLAIMED until flashed with a key.
        ESP_LOGW(TAG, "unprovisioned: no per-device enroll key (NVS rtk_enr_key); cannot enroll");
        return false;
    }
    uint32_t nonce = esp_random();
    char canon[256];
    snprintf(canon, sizeof(canon), "%s|%s|%s|um980|%u", s_device_id, s_mac, FW_VERSION, (unsigned)nonce);
    char hmac[65];
    hmac_sha256_hex(s_enroll_key, canon, hmac);

    char body[512];
    snprintf(body, sizeof(body),
        "{\"device_id\":\"%s\",\"mac\":\"%s\",\"fw_version\":\"%s\",\"hw\":\"um980\",\"nonce\":%u,\"key_version\":%u,\"hmac\":\"%s\"}",
        s_device_id, s_mac, FW_VERSION, (unsigned)nonce, (unsigned)s_key_ver, hmac);

    char url[160];
    snprintf(url, sizeof(url), "https://%s/api/edge/enroll", s_ie_host);

    int status = 0;
    char *resp = http_post_json(url, body, &status);
    if (!resp || status != 200) {
        ESP_LOGW(TAG, "enroll failed (status=%d)", status);
        free(resp);
        return false;
    }

    cJSON *root = cJSON_Parse(resp);
    free(resp);
    if (!root) return false;

    bool ok = false;
    cJSON *caster = cJSON_GetObjectItem(root, "caster");
    cJSON *tok    = cJSON_GetObjectItem(root, "station_token");
    if (cJSON_IsObject(caster) && cJSON_IsString(tok)) {
        write_caster_creds(caster);
        strlcpy(s_token, tok->valuestring, sizeof(s_token));
        config_set_str(KEY_RTK_STATION_TOK, s_token);
        cJSON *poll = cJSON_GetObjectItem(root, "ie_poll_s");
        if (cJSON_IsNumber(poll) && poll->valueint >= MIN_POLL_S) s_poll_s = poll->valueint;
        config_set_u8(KEY_RTK_STATE, RTK_PROVISIONING);
        config_commit();
        ok = true;
        ESP_LOGI(TAG, "enrolled as %s, caster creds stored", s_device_id);
        uart_nmea("$PESP,RTK,ENROLL,OK,%s", s_device_id);
        ntrip_server_reconnect_all();   // pick up the new credentials now
    } else {
        ESP_LOGW(TAG, "enroll reply missing caster/token");
    }
    cJSON_Delete(root);
    return ok;
}

/* --------------------------------------------------------------- heartbeat */

static void apply_reply(cJSON *root) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);

    cJSON *state = cJSON_GetObjectItem(root, "state");
    if (cJSON_IsString(state)) strlcpy(s_st.state, state->valuestring, sizeof(s_st.state));
    cJSON *prog = cJSON_GetObjectItem(root, "progress_pct");
    if (cJSON_IsNumber(prog)) s_st.progress_pct = prog->valueint;
    cJSON *mat = cJSON_GetObjectItem(root, "matures_in_s");
    if (cJSON_IsNumber(mat)) s_st.matures_in_s = mat->valueint;
    cJSON *sid = cJSON_GetObjectItem(root, "station_id");
    if (cJSON_IsString(sid)) strlcpy(s_st.station_id, sid->valuestring, sizeof(s_st.station_id));

    cJSON *q = cJSON_GetObjectItem(root, "quality");
    if (cJSON_IsObject(q)) {
        cJSON *sats = cJSON_GetObjectItem(q, "sats");
        cJSON *fix  = cJSON_GetObjectItem(q, "fix");
        if (cJSON_IsNumber(sats)) s_st.q_sats = sats->valueint;
        if (cJSON_IsNumber(fix))  s_st.q_fix  = fix->valueint;
    }

    cJSON *pos = cJSON_GetObjectItem(root, "position");
    bool want_fix = false;
    double lat = 0, lon = 0, h = 0;
    if (cJSON_IsObject(pos)) {
        cJSON *v = cJSON_GetObjectItem(pos, "valid");
        cJSON *la = cJSON_GetObjectItem(pos, "lat");
        cJSON *lo = cJSON_GetObjectItem(pos, "lon");
        cJSON *he = cJSON_GetObjectItem(pos, "h");
        if (cJSON_IsBool(v) && cJSON_IsTrue(v) && cJSON_IsNumber(la) && cJSON_IsNumber(lo)) {
            s_st.pos_valid = true;
            s_st.pos_lat = lat = la->valuedouble;
            s_st.pos_lon = lon = lo->valuedouble;
            s_st.pos_h   = h   = cJSON_IsNumber(he) ? he->valuedouble : 0.0;
            want_fix = !s_fixed_base_set;   // only push the fixed base once
        }
    }
    xSemaphoreGive(s_mtx);

    /* IE returned a converged position -> set the fixed base (outside the lock) */
    if (want_fix) {
        if (gnss_set_fixed_base(lat, lon, h)) {
            s_fixed_base_set = true;
            ESP_LOGI(TAG, "fixed base applied from IE position");
        }
    }

    /* IE re-issued credentials (re-host, go-live mountpoint, ...) */
    cJSON *delta = cJSON_GetObjectItem(root, "config_delta");
    if (cJSON_IsObject(delta)) {
        cJSON *caster = cJSON_GetObjectItem(delta, "caster");
        if (cJSON_IsObject(caster) && caster_creds_differ(caster)) {
            write_caster_creds(caster);
            ntrip_server_reconnect_all();
            ESP_LOGI(TAG, "applied IE config_delta (caster re-pointed)");
        }
    }

    /* persist promoted state */
    cJSON *st = cJSON_GetObjectItem(root, "state");
    if (cJSON_IsString(st)) {
        uint8_t v = RTK_PROVISIONING;
        if (!strcmp(st->valuestring, "maturing")) v = RTK_MATURING;
        else if (!strcmp(st->valuestring, "active")) v = RTK_ACTIVE;
        config_set_u8(KEY_RTK_STATE, v);
        config_commit();
    }
}

static void heartbeat(void) {
    supervisor_health_t h;
    supervisor_health(&h);

    uint32_t nonce = esp_random();
    char canon[160];
    snprintf(canon, sizeof(canon), "%s|%u|%u", s_device_id, (unsigned)h.uptime_s, (unsigned)nonce);
    char hmac[65];
    // Heartbeat is keyed with the per-station token issued at enroll; it only
    // runs once state >= PROVISIONING, so s_token is always set here.
    hmac_sha256_hex(s_token, canon, hmac);

    char body[640];
    snprintf(body, sizeof(body),
        "{\"device_id\":\"%s\",\"station_token\":\"%s\",\"uptime\":%u,"
        "\"health\":{\"gnss_silent_s\":%u,\"caster_silent_s\":%u,\"ip_down_s\":%u,"
        "\"gnss_ok\":%s,\"caster_ok\":%s,\"link_ok\":%s,"
        "\"rec_gnss\":%u,\"rec_caster\":%u,\"rec_wifi\":%u,\"reboots\":%u},"
        "\"nonce\":%u,\"hmac\":\"%s\"}",
        s_device_id, s_token, (unsigned)h.uptime_s,
        (unsigned)h.gnss_silent_s, (unsigned)h.caster_silent_s, (unsigned)h.ip_down_s,
        h.gnss_ok ? "true" : "false", h.caster_ok ? "true" : "false", h.link_ok ? "true" : "false",
        (unsigned)h.recoveries_gnss, (unsigned)h.recoveries_caster, (unsigned)h.recoveries_wifi,
        (unsigned)h.reboots_supervised, (unsigned)nonce, hmac);

    char url[160];
    snprintf(url, sizeof(url), "https://%s/api/edge/heartbeat", s_ie_host);

    int status = 0;
    char *resp = http_post_json(url, body, &status);
    if (resp && status == 200) {
        cJSON *root = cJSON_Parse(resp);
        if (root) { apply_reply(root); cJSON_Delete(root); }
    } else {
        ESP_LOGW(TAG, "heartbeat failed (status=%d)", status);
    }
    free(resp);
}

/* ------------------------------------------------------------------- task */

static void provisioning_task(void *ctx) {
    (void)ctx;
    wait_for_ip();
    load_identity();

    uint8_t state = config_get_u8(CONF_ITEM(KEY_RTK_STATE));

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    strlcpy(s_st.state, state >= RTK_MATURING ? "maturing" : "provisioning", sizeof(s_st.state));
    s_st.enrolled = (state >= RTK_PROVISIONING);
    xSemaphoreGive(s_mtx);

    for (;;) {
        if (state < RTK_PROVISIONING) {
            if (enroll()) {
                state = RTK_PROVISIONING;
                xSemaphoreTake(s_mtx, portMAX_DELAY);
                s_st.enrolled = true;
                xSemaphoreGive(s_mtx);
            } else {
                vTaskDelay(pdMS_TO_TICKS(15000));   // retry enroll
                continue;
            }
        } else {
            heartbeat();
            state = config_get_u8(CONF_ITEM(KEY_RTK_STATE));
        }
        vTaskDelay(pdMS_TO_TICKS(s_poll_s * 1000));
    }
}

/* ------------------------------------------------------------------- api */

void provisioning_fill_status(cJSON *root) {
    supervisor_health_t h;
    supervisor_health(&h);

    if (!s_mtx) {                                 // status hit before provisioning_init()
        cJSON_AddStringToObject(root, "state", "provisioning");
        cJSON_AddBoolToObject(root, "enrolled", false);
        return;
    }

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    cJSON_AddStringToObject(root, "state", s_st.state[0] ? s_st.state : "provisioning");
    cJSON_AddNumberToObject(root, "progress_pct", s_st.progress_pct);
    cJSON_AddNumberToObject(root, "matures_in_s", s_st.matures_in_s);
    if (s_st.station_id[0]) cJSON_AddStringToObject(root, "station_id", s_st.station_id);
    cJSON_AddBoolToObject(root, "enrolled", s_st.enrolled);

    cJSON *q = cJSON_AddObjectToObject(root, "quality");
    cJSON_AddNumberToObject(q, "sats", s_st.q_sats);
    cJSON_AddNumberToObject(q, "fix", s_st.q_fix);

    if (s_st.pos_valid) {
        cJSON *p = cJSON_AddObjectToObject(root, "position");
        cJSON_AddBoolToObject(p, "valid", true);
        cJSON_AddNumberToObject(p, "lat", s_st.pos_lat);
        cJSON_AddNumberToObject(p, "lon", s_st.pos_lon);
        cJSON_AddNumberToObject(p, "h", s_st.pos_h);
    }
    xSemaphoreGive(s_mtx);

    /* local liveness the device is sure of */
    cJSON_AddBoolToObject(root, "data_ok", h.caster_ok);
    cJSON_AddNumberToObject(root, "uptime_s", h.uptime_s);

    /* self-healing supervisor state (so the dashboard can show the watchdog
     * is alive and how often each subsystem has been auto-recovered). */
    cJSON *sh = cJSON_AddObjectToObject(root, "self_heal");
    cJSON_AddBoolToObject(sh, "gnss_ok", h.gnss_ok);
    cJSON_AddBoolToObject(sh, "caster_ok", h.caster_ok);
    cJSON_AddBoolToObject(sh, "link_ok", h.link_ok);
    cJSON_AddNumberToObject(sh, "rec_gnss", h.recoveries_gnss);
    cJSON_AddNumberToObject(sh, "rec_caster", h.recoveries_caster);
    cJSON_AddNumberToObject(sh, "rec_wifi", h.recoveries_wifi);
    cJSON_AddNumberToObject(sh, "reboots", h.reboots_supervised);
}

void provisioning_init(void) {
    s_mtx = xSemaphoreCreateMutex();
    memset(&s_st, 0, sizeof(s_st));
    strlcpy(s_st.state, "provisioning", sizeof(s_st.state));
    xTaskCreate(provisioning_task, "rtk_provision", 8192, NULL, TASK_PRIORITY_PROVISIONING, NULL);
    ESP_LOGI(TAG, "provisioning client started");
}
