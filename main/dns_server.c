/*
 * Minimal captive-portal DNS hijack for the RTKdata setup AP.
 *
 * Answers EVERY A query with the SoftAP IP (default 192.168.4.1) so a phone or
 * laptop that joins the setup network gets its OS captive-portal popup: the OS
 * connectivity-check hostname (captive.apple.com / connectivitycheck.gstatic.com
 * / msftconnecttest.com) resolves to us, hits the web server, and the existing
 * 302 -> "/" serves the setup page. Runs ONLY while the AP is up during
 * onboarding (started from WIFI_EVENT_AP_START, stopped on WIFI_EVENT_AP_STOP),
 * and only when NAT/internet passthrough is off (that mode needs real DNS).
 *
 * AAAA and non-A queries get an empty NOERROR answer so dual-stack clients fall
 * back to IPv4 and still reach the portal. Single-question queries only (every
 * real resolver sends exactly one); anything else is ignored.
 */
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "dns_server.h"

static const char *TAG = "DNS_HIJACK";

#define DNS_PORT     53
#define DNS_RX_MAX   512
#define DNS_TX_MAX   (DNS_RX_MAX + 16)

static TaskHandle_t  s_dns_task = NULL;
static volatile bool s_run      = false;
static int           s_sock     = -1;

typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t flags;
    uint16_t qd_count;
    uint16_t an_count;
    uint16_t ns_count;
    uint16_t ar_count;
} dns_header_t;

// Return the SoftAP IPv4 (network byte order). Falls back to 192.168.4.1.
static uint32_t ap_ip_be(void) {
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t info;
    if (ap != NULL && esp_netif_get_ip_info(ap, &info) == ESP_OK && info.ip.addr != 0) {
        return info.ip.addr;
    }
    return esp_netif_htonl(esp_netif_ip4_makeu32(192, 168, 4, 1));
}

static void dns_task(void *arg) {
    (void)arg;

    uint8_t rx[DNS_RX_MAX];
    uint8_t tx[DNS_TX_MAX];

    struct sockaddr_in server = {0};
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    server.sin_port        = htons(DNS_PORT);

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        s_dns_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    if (bind(s_sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
        ESP_LOGE(TAG, "bind(:53) failed");
        close(s_sock);
        s_sock     = -1;
        s_dns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "captive DNS hijack up on :53");

    while (s_run) {
        struct sockaddr_in src;
        socklen_t          slen = sizeof(src);

        int n = recvfrom(s_sock, rx, sizeof(rx), 0, (struct sockaddr *)&src, &slen);
        if (n < 0) break;                       // socket closed by dns_hijack_stop()
        if (!s_run) break;
        if (n < (int)sizeof(dns_header_t)) continue;

        dns_header_t *qh = (dns_header_t *)rx;
        if (ntohs(qh->flags) & 0x8000) continue; // a response, not a query
        if (ntohs(qh->qd_count) != 1) continue;  // only the universal single-question case

        // Walk the single QNAME to find the end of the question section.
        int p = sizeof(dns_header_t);
        bool bad = false;
        while (p < n && rx[p] != 0) {
            if (rx[p] & 0xC0) { bad = true; break; }   // compression/reserved in a query QNAME
            p += rx[p] + 1;
        }
        if (bad || p >= n) continue;
        p += 1;                                  // skip the zero-length root label
        if (p + 4 > n) continue;                 // need QTYPE + QCLASS
        uint16_t qtype = (uint16_t)((rx[p] << 8) | rx[p + 1]);
        p += 4;                                  // skip QTYPE + QCLASS
        int qend = p;                            // end of question = insert point for the answer

        if (qend + 16 > (int)sizeof(tx)) continue;

        memcpy(tx, rx, qend);                    // header + the one question only (drop any EDNS/OPT)
        dns_header_t *rh = (dns_header_t *)tx;
        rh->flags    = htons(0x8180);            // response, recursion available, NOERROR
        rh->qd_count = htons(1);
        rh->ns_count = 0;
        rh->ar_count = 0;

        int pos = qend;
        if (qtype == 1) {                        // A: answer with the AP IP
            uint32_t ip = ap_ip_be();
            rh->an_count = htons(1);
            tx[pos++] = 0xC0; tx[pos++] = 0x0C;  // NAME -> pointer to the question at offset 12
            tx[pos++] = 0x00; tx[pos++] = 0x01;  // TYPE  A
            tx[pos++] = 0x00; tx[pos++] = 0x01;  // CLASS IN
            tx[pos++] = 0x00; tx[pos++] = 0x00; tx[pos++] = 0x00; tx[pos++] = 0x1E; // TTL 30s
            tx[pos++] = 0x00; tx[pos++] = 0x04;  // RDLENGTH 4
            memcpy(&tx[pos], &ip, 4); pos += 4;  // RDATA: AP IP (network order)
        } else {
            rh->an_count = 0;                    // AAAA/other -> empty answer -> client falls back to A
        }

        sendto(s_sock, tx, pos, 0, (struct sockaddr *)&src, sizeof(src));
    }

    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
    ESP_LOGI(TAG, "captive DNS hijack stopped");
    s_dns_task = NULL;
    vTaskDelete(NULL);
}

void dns_hijack_start(void) {
    if (s_dns_task != NULL) return;              // already running
    s_run = true;
    if (xTaskCreate(dns_task, "dns_hijack", 4096, NULL, 5, &s_dns_task) != pdPASS) {
        s_run      = false;
        s_dns_task = NULL;
        ESP_LOGE(TAG, "failed to start dns_hijack task");
    }
}

void dns_hijack_stop(void) {
    if (s_dns_task == NULL) return;
    s_run = false;
    int sock = s_sock;
    s_sock = -1;
    if (sock >= 0) close(sock);                  // unblocks recvfrom -> task exits

    // Wait (bounded) for the task to actually exit, so a fast AP flap (stop then
    // immediate start) can't find s_dns_task still set and silently skip the
    // restart, nor race the just-closed fd. The task nulls s_dns_task right
    // before vTaskDelete, so this is normally a single short delay.
    for (int i = 0; i < 50 && s_dns_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
