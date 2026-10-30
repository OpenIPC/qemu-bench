/*
 * Records and plays through libimp's IMP_AI / IMP_AO, for
 * qemu-boot/test-ingenic-audio.sh: the vendor audio.ko, the AIC and the
 * PDMA end to end.
 *
 *   audio-test RATE RECORD_MS PLAY_HZ PLAY_MS FILE [dmic|amic [PLAY_RATE]]
 *
 * Records RECORD_MS of 16-bit mono at RATE from AI device 1 (the
 * analogue microphone) -- or, with "dmic" (built with -DAUDIO_DMIC, the
 * T40 SDK's IMP_DMIC), from DMIC device 0 -- into FILE, raw
 * little-endian (after the first 200 ms, which the vendor driver's
 * start-up leaves partly zero), then plays a sine of PLAY_HZ for PLAY_MS
 * on AO device 0 (at PLAY_RATE, RATE by default), and prints
 * "AUDIOTEST PASS" if every call succeeded.
 *
 * With -DAUDIO_NO_DL (a static uClibc link, whose libdl does not link)
 * dlopen fails here: libimp then runs without its optional
 * libaudioProcess.so (noise suppression, AEC), which this does not use.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include <imp/imp_audio.h>
#ifdef AUDIO_DMIC
#include <imp/imp_dmic.h>
#endif

#ifdef AUDIO_NO_DL
void *dlopen(const char *file, int mode)
{
    return NULL;
}

void *dlsym(void *handle, const char *name)
{
    return NULL;
}

char *dlerror(void)
{
    return (char *)"no dynamic loading";
}

int dlclose(void *handle)
{
    return 0;
}
#endif

#define TRY(call) do { \
        int r_ = (call); \
        if (r_) { \
            printf("AUDIOTEST FAIL %s = %d\n", #call, r_); \
            fflush(stdout); \
            return 1; \
        } \
    } while (0)

static int save_raw(const char *path, int16_t *buf, int want)
{
    FILE *f = fopen(path, "wb");

    if (!f || fwrite(buf, 2, want, f) != (size_t)want || fclose(f)) {
        printf("AUDIOTEST FAIL writing %s\n", path);
        return 1;
    }
    return 0;
}

static int record(int rate, int ms, const char *path)
{
    IMPAudioIOAttr attr;
    IMPAudioIChnParam chn;
    int per = rate / 50;                /* 20 ms frames */
    int want = (int)((long long)rate * ms / 1000), skip = rate / 5, got = 0;
    int16_t *buf = malloc((want + skip + per) * 2);

    memset(&attr, 0, sizeof(attr));
    attr.samplerate = rate;
    attr.bitwidth = AUDIO_BIT_WIDTH_16;
    attr.soundmode = AUDIO_SOUND_MODE_MONO;
    attr.frmNum = 20;
    attr.numPerFrm = per;
    attr.chnCnt = 1;
    TRY(IMP_AI_SetPubAttr(1, &attr));
    TRY(IMP_AI_Enable(1));
    memset(&chn, 0, sizeof(chn));
    chn.usrFrmDepth = 20;
    TRY(IMP_AI_SetChnParam(1, 0, &chn));
    TRY(IMP_AI_EnableChn(1, 0));
    TRY(IMP_AI_SetVol(1, 0, 60));
    while (got < want + skip) {
        IMPAudioFrame frm;

        TRY(IMP_AI_PollingFrame(1, 0, 2000));
        TRY(IMP_AI_GetFrame(1, 0, &frm, BLOCK));
        memcpy(buf + got, frm.virAddr, frm.len);
        got += frm.len / 2;
        TRY(IMP_AI_ReleaseFrame(1, 0, &frm));
    }
    TRY(IMP_AI_DisableChn(1, 0));
    TRY(IMP_AI_Disable(1));
    TRY(save_raw(path, buf + skip, want));
    free(buf);
    return 0;
}

#ifdef AUDIO_DMIC
static int record_dmic(int rate, int ms, const char *path)
{
    IMPDmicAttr attr;
    IMPDmicChnParam chn;
    IMPDmicChnFrame frm;
    int per = rate / 25;                /* 40 ms frames */
    int want = (int)((long long)rate * ms / 1000), skip = rate / 5, got = 0;
    int16_t *buf = malloc((want + skip + per) * 2);

    TRY(IMP_DMIC_SetUserInfo(0, 1, 0));
    memset(&attr, 0, sizeof(attr));
    attr.samplerate = rate;
    attr.bitwidth = DMIC_BIT_WIDTH_16;
    attr.soundmode = DMIC_SOUND_MODE_MONO;
    attr.chnCnt = 1;
    attr.frmNum = 40;
    attr.numPerFrm = per;
    TRY(IMP_DMIC_SetPubAttr(0, &attr));
    TRY(IMP_DMIC_Enable(0));
    memset(&chn, 0, sizeof(chn));
    chn.usrFrmDepth = 40;
    TRY(IMP_DMIC_SetChnParam(0, 0, &chn));
    TRY(IMP_DMIC_EnableChn(0, 0));
    TRY(IMP_DMIC_SetVol(0, 0, 60));
    /* as the SDK's sample-dmic.c: libimp's gain is otherwise unset */
    TRY(IMP_DMIC_SetGain(0, 0, 22));
    while (got < want + skip) {
        int n;

        TRY(IMP_DMIC_PollingFrame(0, 0, 2000));
        TRY(IMP_DMIC_GetFrame(0, 0, &frm, BLOCK));
        n = MIN(frm.rawFrame.len / 2, want + skip - got);
        memcpy(buf + got, frm.rawFrame.virAddr, n * 2);
        got += n;
        TRY(IMP_DMIC_ReleaseFrame(0, 0, &frm));
    }
    TRY(IMP_DMIC_DisableChn(0, 0));
    TRY(IMP_DMIC_Disable(0));
    TRY(save_raw(path, buf + skip, want));
    free(buf);
    return 0;
}
#endif

static int play(int rate, int hz, int ms)
{
    IMPAudioIOAttr attr;
    int per = rate / 50, n = (int)((long long)rate * ms / 1000), i, k;
    int16_t *buf = malloc(per * 2);

    memset(&attr, 0, sizeof(attr));
    attr.samplerate = rate;
    attr.bitwidth = AUDIO_BIT_WIDTH_16;
    attr.soundmode = AUDIO_SOUND_MODE_MONO;
    attr.frmNum = 20;
    attr.numPerFrm = per;
    attr.chnCnt = 1;
    TRY(IMP_AO_SetPubAttr(0, &attr));
    TRY(IMP_AO_Enable(0));
    TRY(IMP_AO_EnableChn(0, 0));
    TRY(IMP_AO_SetVol(0, 0, 60));
    for (k = 0; k < n; k += per) {
        IMPAudioFrame frm;

        for (i = 0; i < per; i++) {
            buf[i] = (int16_t)lrint(12000 * sin(2 * M_PI * hz * (k + i) /
                                                rate));
        }
        memset(&frm, 0, sizeof(frm));
        frm.virAddr = (uint32_t *)buf;
        frm.len = per * 2;
        TRY(IMP_AO_SendFrame(0, 0, &frm, BLOCK));
    }
    IMP_AO_FlushChnBuf(0, 0);
    TRY(IMP_AO_DisableChn(0, 0));
    TRY(IMP_AO_Disable(0));
    free(buf);
    return 0;
}

int main(int argc, char **argv)
{
    int rate = argc > 1 ? atoi(argv[1]) : 16000;
    int rec_ms = argc > 2 ? atoi(argv[2]) : 500;
    int hz = argc > 3 ? atoi(argv[3]) : 440;
    int play_ms = argc > 4 ? atoi(argv[4]) : 2000;
    const char *path = argc > 5 ? argv[5] : "/tmp/rec.raw";
    int dmic = argc > 6 && !strcmp(argv[6], "dmic");
    int play_rate = argc > 7 ? atoi(argv[7]) : rate;

    setvbuf(stdout, NULL, _IOLBF, 0);
#ifdef AUDIO_DMIC
    if (dmic && rec_ms > 0 && record_dmic(rate, rec_ms, path)) {
        return 1;
    }
#else
    if (dmic) {
        printf("AUDIOTEST FAIL built without IMP_DMIC\n");
        return 1;
    }
#endif
    if (!dmic && rec_ms > 0 && record(rate, rec_ms, path)) {
        return 1;
    }
    printf("AUDIOTEST recorded\n");
    if (play_ms > 0 && play(play_rate, hz, play_ms)) {
        return 1;
    }
    printf("AUDIOTEST PASS\n");
    return 0;
}
