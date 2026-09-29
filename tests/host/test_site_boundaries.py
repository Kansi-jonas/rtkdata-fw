"""Fault-inject production JSON-control and final socket-send boundaries."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from test_station_health import function

ROOT = Path(__file__).resolve().parents[2]
PRELUDE = r'''
#include "gnss_site.h"
#include "interface/ntrip_tx_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
typedef struct cJSON {
    const char *key;
    int type;
    const char *valuestring;
    double valuedouble;
    struct cJSON *child, *next;
} cJSON;
enum { J_NULL, J_NUMBER, J_STRING, J_FALSE, J_TRUE, J_OBJECT };
static int cJSON_IsNumber(const cJSON *i) { return i && i->type == J_NUMBER; }
static int cJSON_IsString(const cJSON *i) { return i && i->type == J_STRING; }
static int cJSON_IsBool(const cJSON *i) { return i && (i->type == J_FALSE || i->type == J_TRUE); }
static int cJSON_IsTrue(const cJSON *i) { return i && i->type == J_TRUE; }
static int cJSON_IsObject(const cJSON *i) { return i && i->type == J_OBJECT; }
static cJSON *cJSON_GetObjectItemCaseSensitive(const cJSON *object, const char *key) {
    for (cJSON *i = object ? object->child : NULL; i; i = i->next)
        if (!strcmp(i->key,key)) return i;
    return NULL;
}
typedef struct { int sock; uint32_t site_generation; void *stats; } ntrip_instance_t;
static bool allowed;
static uint32_t generation;
static unsigned sent_calls, sent_bytes, health_bytes, checks;
bool gnss_site_upload_allowed(void) { return allowed; }
uint32_t gnss_site_upload_generation(void) { return generation; }
static int mock_send(int sock, const void *buf, size_t len, int flags) {
    (void)sock; (void)buf; (void)flags; sent_calls++; sent_bytes += (unsigned)len;
    return (int)len;
}
#define send mock_send
static void supervisor_note_caster_tx(int bytes) { health_bytes += (unsigned)bytes; }
static void stream_stats_increment(void *stats, int rx, int tx) { (void)stats; (void)rx; (void)tx; }
static char s_token[32]="dummy";
static struct { bool enrolled, pos_valid; char state[16], site_phase[20]; } s_st;
static int s_mtx;
static unsigned auth_writes, auth_commits, auth_holds;
#define portMAX_DELAY 0
#define KEY_RTK_STATION_TOK "station_token"
#define KEY_RTK_STATE "state"
#define RTK_UNCLAIMED 0
static void xSemaphoreTake(int mutex, int timeout) { (void)mutex; (void)timeout; }
static void xSemaphoreGive(int mutex) { (void)mutex; }
static void config_set_str(const char *key, const char *value) {
    if (strcmp(key,KEY_RTK_STATION_TOK) || value[0]) abort();
    auth_writes++;
}
static void config_set_u8(const char *key, uint8_t value) {
    if (strcmp(key,KEY_RTK_STATE) || value != RTK_UNCLAIMED) abort();
    auth_writes++;
}
static void config_commit(void) { auth_commits++; }
void gnss_site_status(gnss_site_status_t *out) { *out=(gnss_site_status_t){.boot_id=123}; }
bool gnss_site_apply_control(const gnss_site_control_t *c) {
    if (c->boot_id!=123 || c->mode!=GNSS_SITE_HOLD || c->allow_upload) abort();
    auth_holds++; allowed=false; return true;
}
static size_t test_strlcpy(char *out, const char *value, size_t size) {
    size_t len=strlen(value);
    if (size) snprintf(out,size,"%s",value);
    return len;
}
#define strlcpy test_strlcpy
#define CHECK(c) do { checks++; if (!(c)) { \
    printf("FAIL %d: %s\n", __LINE__, #c); return 1; } } while (0)
'''

TESTS = r'''
int main(void) {
    cJSON fields[] = {
        {.key="v",.type=J_NUMBER,.valuedouble=1},
        {.key="boot_id",.type=J_NUMBER,.valuedouble=4294967295.0},
        {.key="revision",.type=J_NUMBER,.valuedouble=4294967295.0},
        {.key="mode",.type=J_STRING,.valuestring="fixed"},
        {.key="allow_upload",.type=J_TRUE},
        {.key="lat_e9",.type=J_NUMBER,.valuedouble=52000000000.0},
        {.key="lon_e9",.type=J_NUMBER,.valuedouble=-13000000000.0},
        {.key="h_mm",.type=J_NUMBER,.valuedouble=42000},
    };
    for (unsigned i=0; i<7; i++) fields[i].next=&fields[i+1];
    cJSON object = {.key="site_control",.type=J_OBJECT,.child=fields};
    cJSON root = {.type=J_OBJECT,.child=&object};
    gnss_site_control_t control;
    CHECK(parse_site_control(&root,&control));
    CHECK(control.boot_id == UINT32_MAX && control.revision == UINT32_MAX &&
          control.lat_e9 == INT64_C(52000000000) && control.lon_e9 == INT64_C(-13000000000));
    fields[1].valuedouble=0;
    CHECK(!parse_site_control(&root,&control));
    fields[1].valuedouble=123.5;
    CHECK(!parse_site_control(&root,&control));
    fields[1].valuedouble=123;
    fields[2].valuedouble=4294967296.0;
    CHECK(!parse_site_control(&root,&control));
    fields[2].valuedouble=2;
    fields[5].type=J_NULL;
    CHECK(!parse_site_control(&root,&control));
    fields[5].type=J_NUMBER; fields[5].valuedouble=NAN;
    CHECK(!parse_site_control(&root,&control));
    fields[5].valuedouble=90000000001.0;
    CHECK(!parse_site_control(&root,&control));
    fields[5].valuedouble=52000000000.5;
    CHECK(!parse_site_control(&root,&control));
    fields[5].valuedouble=52000000000.0;
    fields[7].valuedouble=-500001;
    CHECK(!parse_site_control(&root,&control));
    fields[7].valuedouble=42000;
    fields[4].type=J_NUMBER; fields[4].valuedouble=1;
    CHECK(!parse_site_control(&root,&control));
    fields[4].type=J_TRUE; fields[3].valuestring="rover";
    CHECK(!parse_site_control(&root,&control));
    fields[3].valuestring="hold";
    CHECK(!parse_site_control(&root,&control));
    fields[4].type=J_FALSE;
    CHECK(parse_site_control(&root,&control) && control.mode==GNSS_SITE_HOLD);
    fields[3].valuestring="survey"; fields[4].type=J_TRUE;
    CHECK(parse_site_control(&root,&control) && control.mode==GNSS_SITE_SURVEY &&
          control.lat_e9==0 && control.lon_e9==0 && control.h_mm==0);
    CHECK(!parse_site_control(NULL,&control));
    // The final socket callback rejects bytes from a copied old partial frame,
    // even after a newer site has reopened the global gate.
    ntrip_instance_t instance = {.sock=1,.site_generation=7};
    uint8_t bytes[16]={0}; int error=0; generation=7;
    CHECK(ntrip_server_sock_send(&instance,bytes,sizeof(bytes),&error)==-1);
    CHECK(error==ECANCELED && sent_calls==0 && health_bytes==0);
    allowed=true; generation=8;
    CHECK(ntrip_server_sock_send(&instance,bytes,sizeof(bytes),&error)==-1 && sent_calls==0);
    instance.site_generation=8;
    CHECK(ntrip_server_sock_send(&instance,bytes,sizeof(bytes),&error)==16);
    CHECK(sent_calls==1 && sent_bytes==16 && health_bytes==16);
    allowed=false;
    CHECK(ntrip_server_sock_send(&instance,bytes,sizeof(bytes),&error)==-1 && sent_calls==1);
    // Only authenticated rejection classes repair enrollment; network/server
    // faults leave durable credentials intact, and 403 cannot bypass disable.
    handle_heartbeat_auth_failure(0); handle_heartbeat_auth_failure(500);
    CHECK(auth_writes==0 && auth_holds==0 && !strcmp(s_token,"dummy"));
    allowed=true; handle_heartbeat_auth_failure(403);
    CHECK(!allowed && auth_holds==1 && auth_writes==0 && !strcmp(s_token,"dummy"));
    s_st.enrolled=s_st.pos_valid=true; allowed=true;
    handle_heartbeat_auth_failure(401);
    CHECK(!allowed && auth_holds==2 && auth_writes==2 && auth_commits==1);
    CHECK(!s_token[0] && !s_st.enrolled && !s_st.pos_valid &&
          !strcmp(s_st.site_phase,"checking") && !strcmp(s_st.state,"provisioning"));
    strcpy(s_token,"dummy"); allowed=true;
    handle_heartbeat_auth_failure(404);
    CHECK(!allowed && !s_token[0] && auth_holds==3 && auth_writes==4 && auth_commits==2);
    printf("%u JSON/socket boundary checks, 0 failures\n",checks);
    return 0;
}
'''

def main():
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise SystemExit("Host C compiler required")
    provision = (ROOT / "main/provisioning.c").read_text(encoding="utf-8")
    ntrip = (ROOT / "main/interface/ntrip_server.c").read_text(encoding="utf-8")
    actual = "\n".join([function(provision, "json_integer"),
                        function(provision, "parse_site_control"),
                        function(provision, "handle_heartbeat_auth_failure"),
                        function(ntrip, "ntrip_server_sock_send")])
    with tempfile.TemporaryDirectory(prefix="rtkdata-site-boundaries-") as folder:
        source = Path(folder) / "test.c"
        binary = Path(folder) / ("test.exe" if os.name == "nt" else "test")
        source.write_text(PRELUDE + actual + TESTS, encoding="utf-8")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        f"-I{ROOT / 'main/include'}", str(source), "-lm", "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    main()
