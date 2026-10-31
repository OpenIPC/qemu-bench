/*
 * The vendor encoder path end to end on an emulated T21/T23 camera: the
 * SDK samples' sequence (sample-common.c, sample-Encoder-video.c,
 * sample-Encoder-jpeg.c) -- ISP open, sensor added and enabled,
 * IMP_System_Init, tuning on, FrameSource channel 1 scaled to 640x360,
 * an encoder group with an H.264 channel (0) and a JPEG channel (3),
 * FrameSource bound to the group, stream on -- then N H.264 frames and a
 * JPEG snapshot through IMP_Encoder_GetStream.  The guest half of
 * qemu/tests/test-ingenic-vpu-e2e.sh.
 *
 *   ingenic-vpu-enc frames out sensor i2c-addr [width height]
 *
 * @out is the SD card's block device: the H.264 stream goes to offset 0
 * behind a 512-byte header ("H264", its length), the JPEG to 8 MiB behind
 * one ("JPEG", its length).  The sensor's size (width x height, default
 * 1920x1080) is FrameSource channel 0's, which the ISP needs even when
 * only channel 1 is encoded.
 *
 * Built like qemu-boot/ingenic-isp-snap.c: the T31 SDK's headers for the
 * ISP, system and FrameSource calls (that client's IMPFSChnAttr fix-up
 * included), the T23 SDK's imp_encoder.h, whose IMPEncoderCHNAttr is the
 * layout T21's libimp copies too (160 bytes; T23's adds bEnableIvdc), and
 * dynamically against the rootfs's libimp.so and musl.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <ctype.h>
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_isp.h>
#include <imp/imp_framesource.h>
#include IMP_ENCODER_H

/* ---- what a uClibc libimp wants from a musl program --------------------- */

int __libc_start_main(int (*m)(int, char **, char **), int argc, char **argv,
                      void (*init)(void), void (*fini)(void),
                      void (*ldso_fini)(void));

int __uClibc_main(int (*m)(int, char **, char **), int argc, char **argv,
                  void (*init)(void), void (*fini)(void),
                  void (*rtld_fini)(void), void *stack_end)
{
    return __libc_start_main(m, argc, argv, init, fini, rtld_fini);
}

/* uClibc's off_t is 32 bits, musl's 64: libimp's calls go to the kernel */
void *mmap(void *addr, size_t len, int prot, int flags, int fd, off_t off)
{
    return (void *)syscall(__NR_mmap2, addr, len, prot, flags, fd,
                           (unsigned long)off >> 12);
}

int ftruncate(int fd, off_t len)
{
    return syscall(__NR_ftruncate, fd, len);
}

/* T20/T21's libalog.so reads uClibc's C-locale ctype tables directly */
static __ctype_mask_t ctype_b[384];
static __ctype_touplow_t ctype_lower[384], ctype_upper[384];
const __ctype_mask_t *__ctype_b = ctype_b + 128;
const __ctype_touplow_t *__ctype_tolower = ctype_lower + 128;
const __ctype_touplow_t *__ctype_toupper = ctype_upper + 128;

static void __attribute__((constructor)) ctype_init(void)
{
    int c;

    for (c = -128; c < 256; c++) {
        __ctype_mask_t m = 0;
        int up = c >= 'A' && c <= 'Z', lo = c >= 'a' && c <= 'z';
        int dig = c >= '0' && c <= '9';

        ctype_lower[c + 128] = up ? c + 32 : c;
        ctype_upper[c + 128] = lo ? c - 32 : c;
        if (c < 0 || c > 127) {
            continue;
        }
        m |= up ? _ISupper : 0;
        m |= lo ? _ISlower : 0;
        m |= up || lo ? _ISalpha | _ISalnum : 0;
        m |= dig ? _ISdigit | _ISalnum | _ISxdigit : 0;
        m |= (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F') ? _ISxdigit : 0;
        m |= c == ' ' || (c >= 9 && c <= 13) ? _ISspace : 0;
        m |= c == ' ' || c == 9 ? _ISblank : 0;
        m |= c < 32 || c == 127 ? _IScntrl : 0;
        m |= c >= 32 && c < 127 ? _ISprint : 0;
        m |= c > 32 && c < 127 ? _ISgraph : 0;
        m |= c > 32 && c < 127 && !up && !lo && !dig ? _ISpunct : 0;
        ctype_b[c + 128] = m;
    }
}

void __assert(const char *a, const char *file, unsigned line, const char *fn)
{
    printf("assertion %s failed: %s:%u %s\n", a, file, line, fn);
    abort();
}

int __fgetc_unlocked(FILE *fp)
{
    return fgetc(fp);
}

void __pthread_register_cancel(__pthread_unwind_buf_t *buf)
{
}

void __pthread_unregister_cancel(__pthread_unwind_buf_t *buf)
{
}

void __pthread_unwind_next(__pthread_unwind_buf_t *buf)
{
    pthread_exit((void *)-1);
}

/* ---- the camera ---------------------------------------------------------- */

#define W 640
#define H 360

/* the IMP 1.1.6 IMPFSChnAttr ends with a front crop the headers lack */
static struct {
    IMPFSChnAttr a;
    IMPFSChnCrop fcrop;
} attrs[2] = {
    { {
        .pixFmt = PIX_FMT_NV12, .outFrmRateNum = 25, .outFrmRateDen = 1,
        .nrVBs = 2, .type = FS_PHY_CHANNEL,
        .picWidth = 1920, .picHeight = 1080,
    } }, { {
        .pixFmt = PIX_FMT_NV12, .outFrmRateNum = 25, .outFrmRateDen = 1,
        .nrVBs = 2, .type = FS_PHY_CHANNEL,
        .scaler = { .enable = 1, .outwidth = W, .outheight = H },
        .picWidth = W, .picHeight = H,
    } },
};

/* the stream, gathered in memory and written once (O_DIRECT, aligned) */
static unsigned char stream[4 << 20] __attribute__((aligned(4096)));

static int save(const char *dev, unsigned long long pos, const char *tag,
                const unsigned char *p, unsigned len)
{
    static unsigned char hdr[512] __attribute__((aligned(4096)));
    unsigned long long res;
    int fd = open(dev, O_WRONLY | O_DIRECT), ok;

    if (fd < 0) {
        return -1;
    }
    memset(hdr, 0, sizeof(hdr));
    memcpy(hdr, tag, 4);
    memcpy(hdr + 4, &len, 4);
    ok = syscall(__NR__llseek, fd, (unsigned long)(pos >> 32),
                 (unsigned long)pos, &res, SEEK_SET) >= 0 &&
         write(fd, hdr, 512) == 512 &&
         write(fd, p, (len + 511) & ~511u) >= 0;
    close(fd);
    return ok ? 0 : -1;
}

/* one picture's packs, appended to @buf at @len */
static int get_stream(int chn, unsigned char *buf, unsigned *len,
                      unsigned cap)
{
    IMPEncoderStream st;
    unsigned i;

    if (IMP_Encoder_PollingStream(chn, 3000) < 0) {
        printf("chn%d: no stream\n", chn);
        return -1;
    }
    if (IMP_Encoder_GetStream(chn, &st, 1) < 0) {
        printf("chn%d: GetStream failed\n", chn);
        return -1;
    }
    for (i = 0; i < st.packCount; i++) {
        IMPEncoderPack *pk = &st.pack[i];

        if (*len + pk->length <= cap) {
            memcpy(buf + *len, (void *)pk->virAddr, pk->length);
            *len += pk->length;
        }
    }
    printf("chn%d: seq %u, %u packs, %u bytes so far\n", chn, st.seq,
           st.packCount, *len);
    IMP_Encoder_ReleaseStream(chn, &st);
    return 0;
}

int main(int argc, char **argv)
{
    IMPSensorInfo sensor;
    IMPEncoderCHNAttr ca;
    IMPCell fs = { DEV_ID_FS, 1, 0 }, enc = { DEV_ID_ENC, 0, 0 };
    int frames = argc > 1 ? atoi(argv[1]) : 10;
    const char *dev = argc > 2 ? argv[2] : "/dev/mmcblk0";
    const char *name = argc > 3 ? argv[3] : "jxf37";
    unsigned len = 0, jlen = 0, i;
    int ch;

    memset(&sensor, 0, sizeof(sensor));
    strncpy(sensor.name, name, sizeof(sensor.name) - 1);
    sensor.cbus_type = TX_SENSOR_CONTROL_INTERFACE_I2C;
    strncpy(sensor.i2c.type, name, sizeof(sensor.i2c.type) - 1);
    sensor.i2c.addr = argc > 4 ? strtol(argv[4], NULL, 0) : 0x40;
    if (argc > 6) {
        attrs[0].a.picWidth = atoi(argv[5]);
        attrs[0].a.picHeight = atoi(argv[6]);
    }
    if (IMP_ISP_Open() < 0 || IMP_ISP_AddSensor(&sensor) < 0 ||
        IMP_ISP_EnableSensor() < 0 || IMP_System_Init() < 0 ||
        IMP_ISP_EnableTuning() < 0) {
        printf("ISP / system setup failed\n");
        return 1;
    }
    for (ch = 0; ch < 2; ch++) {
        if (IMP_FrameSource_CreateChn(ch, &attrs[ch].a) < 0 ||
            IMP_FrameSource_SetChnAttr(ch, &attrs[ch].a) < 0) {
            printf("fs%d: CreateChn failed\n", ch);
            return 1;
        }
    }
    if (IMP_Encoder_CreateGroup(0) < 0) {
        printf("CreateGroup failed\n");
        return 1;
    }
    /* H.264, the samples' CBR settings */
    memset(&ca, 0, sizeof(ca));
    ca.encAttr.enType = PT_H264;
    ca.encAttr.bufSize = W * H * 3 / 4;
    ca.encAttr.profile = 1;
    ca.encAttr.picWidth = W;
    ca.encAttr.picHeight = H;
    ca.rcAttr.outFrmRate.frmRateNum = 25;
    ca.rcAttr.outFrmRate.frmRateDen = 1;
    ca.rcAttr.maxGop = 50;
    ca.rcAttr.attrRcMode.rcMode = ENC_RC_MODE_CBR;
    ca.rcAttr.attrRcMode.attrH264Cbr.outBitRate = 600;
    ca.rcAttr.attrRcMode.attrH264Cbr.maxQp = 45;
    ca.rcAttr.attrRcMode.attrH264Cbr.minQp = 15;
    ca.rcAttr.attrRcMode.attrH264Cbr.frmQPStep = 3;
    ca.rcAttr.attrRcMode.attrH264Cbr.gopQPStep = 15;
    ca.rcAttr.attrHSkip.hSkipAttr.skipType = IMP_Encoder_STYPE_N1X;
    ca.rcAttr.attrHSkip.maxHSkipType = IMP_Encoder_STYPE_N1X;
    if (IMP_Encoder_CreateChn(0, &ca) < 0 || IMP_Encoder_RegisterChn(0, 0) < 0) {
        printf("H.264 channel failed\n");
        return 1;
    }
    /* JPEG */
    memset(&ca, 0, sizeof(ca));
    ca.encAttr.enType = PT_JPEG;
    ca.encAttr.bufSize = W * H * 3 / 4;
    ca.encAttr.profile = 1;
    ca.encAttr.picWidth = W;
    ca.encAttr.picHeight = H;
    if (IMP_Encoder_CreateChn(3, &ca) < 0 || IMP_Encoder_RegisterChn(0, 3) < 0) {
        printf("JPEG channel failed\n");
        return 1;
    }
    if (IMP_System_Bind(&fs, &enc) < 0) {
        printf("Bind failed\n");
        return 1;
    }
    for (ch = 0; ch < 2; ch++) {
        if (IMP_FrameSource_EnableChn(ch) < 0) {
            printf("fs%d: EnableChn failed\n", ch);
            return 1;
        }
    }
    if (IMP_Encoder_StartRecvPic(0) < 0) {
        printf("StartRecvPic failed\n");
        return 1;
    }
    printf("streaming\n");
    for (i = 0; i < (unsigned)frames; i++) {
        if (get_stream(0, stream, &len, sizeof(stream) / 2) < 0) {
            return 1;
        }
    }
    IMP_Encoder_StopRecvPic(0);
    /* a snapshot */
    if (IMP_Encoder_StartRecvPic(3) < 0 ||
        get_stream(3, stream + sizeof(stream) / 2, &jlen,
                   sizeof(stream) / 2) < 0) {
        printf("JPEG snapshot failed\n");
        return 1;
    }
    IMP_Encoder_StopRecvPic(3);
    if (save(dev, 0, "H264", stream, len) < 0 ||
        save(dev, 8ull << 20, "JPEG", stream + sizeof(stream) / 2,
             jlen) < 0) {
        printf("cannot write %s\n", dev);
        return 1;
    }
    printf("ingenic-vpu-enc: %u H.264 bytes, %u JPEG bytes: PASS\n", len,
           jlen);
    return 0;
}
