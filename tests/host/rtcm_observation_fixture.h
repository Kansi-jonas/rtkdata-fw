#ifndef RTCM_OBSERVATION_FIXTURE_H
#define RTCM_OBSERVATION_FIXTURE_H
#include "interface/ntrip_tx_core.h"
#include <string.h>
enum { OBS_VALID, OBS_EMPTY, OBS_NO_SIGNALS, OBS_NO_CELLS, OBS_SHORT,
       OBS_BAD_ROUGH, OBS_BAD_CODE, OBS_BAD_PHASE, OBS_ONE_VALID };
static void obs_bits(uint8_t *p, unsigned bit, unsigned width, uint32_t value) {
    for (unsigned i = 0; i < width; ++i) {
        uint8_t mask = (uint8_t)(1U << (7-(bit+i)%8));
        if (value & (UINT32_C(1) << (width-1-i))) p[(bit+i)/8] |= mask;
        else p[(bit+i)/8] &= (uint8_t)~mask;
    }
}
/* Two satellites, one signal, two cells; both use finite code/phase residuals. */
static size_t observation_frame(uint8_t *frame, uint16_t type, uint32_t epoch, int kind) {
    memset(frame, 0, 128);
    uint8_t *p = frame+3;
    unsigned payload = (169+2+36*2+80*2+7)/8;
    obs_bits(p,0,12,type); obs_bits(p,12,12,100); obs_bits(p,24,30,epoch);
    if (kind != OBS_EMPTY) {
        obs_bits(p,73,1,1); obs_bits(p,74,1,1);
        if (kind != OBS_NO_SIGNALS) obs_bits(p,137,1,1);
        if (kind != OBS_NO_CELLS) { obs_bits(p,169,1,1); obs_bits(p,170,1,1); }
        obs_bits(p,171,8,kind==OBS_BAD_ROUGH ? 255 : 80);
        obs_bits(p,179,8,kind==OBS_BAD_ROUGH ? 255 : 81);
        if (kind==OBS_BAD_CODE) { obs_bits(p,243,20,0x80000); obs_bits(p,263,20,0x80000); }
        if (kind==OBS_BAD_PHASE || kind==OBS_ONE_VALID) obs_bits(p,283,24,0x800000);
        if (kind==OBS_BAD_PHASE) obs_bits(p,307,24,0x800000);
    } else {
        payload = 22;
    }
    if (kind == OBS_SHORT) payload = 25;
    frame[0]=0xd3; frame[1]=(uint8_t)(payload>>8); frame[2]=(uint8_t)payload;
    uint32_t crc=rtcm_crc24q(frame,payload+3);
    frame[payload+3]=(uint8_t)(crc>>16);
    frame[payload+4]=(uint8_t)(crc>>8);
    frame[payload+5]=(uint8_t)crc;
    return payload+6;
}
#endif
