#include "rtcm_observation_fixture.h"
#include <stdio.h>
static unsigned checks;
#define CHECK(c) do { ++checks; if (!(c)) { printf("FAIL %d: %s\n",__LINE__,#c); return 1; } } while(0)
int main(void) {
    rtcm_observation_monitor_t m;
    uint8_t frame[128];
    rtcm_observation_init(&m);
    CHECK(rtcm_observation_silent_ms(&m,1000)==UINT32_MAX);
    const uint16_t families[] = {1077,1087,1097,1117,1127,1137};
    for (unsigned i=0; i<6; ++i) {
        size_t len=observation_frame(frame,families[i],1000,OBS_VALID);
        CHECK(rtcm_observation_note(&m,frame,len,1000));
    }
    CHECK(rtcm_observation_silent_ms(&m,9000)==8000);
    size_t len=observation_frame(frame,1077,1000,OBS_VALID);
    CHECK(!rtcm_observation_note(&m,frame,len,9000)); /* repeated receiver epoch */
    CHECK(rtcm_observation_silent_ms(&m,9000)==8000);
    const uint16_t metadata[]={1005,1033,1230};
    for (unsigned i=0;i<3;++i) {
        len=observation_frame(frame,metadata[i],2000,OBS_VALID);
        CHECK(!rtcm_observation_note(&m,frame,len,9000));
    }
    for (int kind=OBS_EMPTY;kind<=OBS_BAD_PHASE;++kind) {
        len=observation_frame(frame,1077,2000,kind);
        CHECK(!rtcm_observation_note(&m,frame,len,9000));
        CHECK(rtcm_observation_silent_ms(&m,9000)==8000);
    }
    len=observation_frame(frame,1077,2000,OBS_ONE_VALID);
    CHECK(rtcm_observation_note(&m,frame,len,9000));
    len=observation_frame(frame,1077,1000,OBS_VALID);
    CHECK(!rtcm_observation_note(&m,frame,len,9500)); /* backwards */
    len=observation_frame(frame,1077,3000,OBS_VALID); frame[len-1]^=1;
    CHECK(!rtcm_observation_note(&m,frame,len,9500));
    CHECK(rtcm_observation_silent_ms(&m,9500)==500);
    // Week rollover for all families; GLONASS carries day + milliseconds-of-day.
    for (unsigned i=0;i<6;++i) {
        rtcm_observation_init(&m);
        uint32_t end=i==1 ? (6U<<27)|86399000U : 604799000U;
        len=observation_frame(frame,families[i],end,OBS_VALID);
        CHECK(rtcm_observation_note(&m,frame,len,1000));
        len=observation_frame(frame,families[i],0,OBS_VALID);
        CHECK(rtcm_observation_note(&m,frame,len,2000));
        CHECK(!rtcm_observation_note(&m,frame,len,3000));
        len=observation_frame(frame,families[i],end,OBS_VALID);
        CHECK(!rtcm_observation_note(&m,frame,len,3000));
    }
    rtcm_observation_init(&m);
    len=observation_frame(frame,1087,(1U<<27)|86399000U,OBS_VALID);
    CHECK(rtcm_observation_note(&m,frame,len,1000));
    len=observation_frame(frame,1087,(2U<<27),OBS_VALID);
    CHECK(rtcm_observation_note(&m,frame,len,2000)); /* day rollover */
    len=observation_frame(frame,1087,(7U<<27),OBS_VALID);
    CHECK(!rtcm_observation_note(&m,frame,len,3000));
    len=observation_frame(frame,1077,604800000U,OBS_VALID);
    CHECK(!rtcm_observation_note(&m,frame,len,3000));
    rtcm_observation_init(&m);
    len=observation_frame(frame,1077,1000,OBS_VALID);
    CHECK(rtcm_observation_note(&m,frame,len,UINT32_MAX-100));
    CHECK(rtcm_observation_silent_ms(&m,50)==151); /* host uptime wraps */
    rtcm_observation_init(&m);
    CHECK(rtcm_observation_silent_ms(&m,50)==UINT32_MAX);
    CHECK(rtcm_observation_note(&m,frame,len,50)); /* receiver reset permits fresh epoch */
    printf("%u observation progress checks, 0 failures\n",checks);
    return 0;
}
