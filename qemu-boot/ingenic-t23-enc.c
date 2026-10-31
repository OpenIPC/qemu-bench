/*
 * Encode the emulated sensors' frames on T23 through libimp's IMP_Encoder
 * and the HELIX model (hw/misc/ingenic-vpu.c) -- the guest half of
 * qemu-boot/test-ingenic-t23-enc.sh.  The sequence is the T23 SDK's
 * sample-Encoder-Double-Ivdc.c (and its single-sensor siblings): ISP
 * open, one or two sensors added (the second as sensor_id 1), tuning on,
 * FrameSource channel 0 at the main sensor's size and channel 1 scaled to
 * 640x360, and with a second sensor channels 3 and 4 likewise; encoder
 * groups 0 (channel 0) and 1 (channel 3, the second sensor's main
 * stream) take H.264 channels 0 and 1, groups 2 and 3 (channels 1 and 4)
 * H.264 channels 4 and 5 -- the bindings the sample's direct_mode needs.
 * With @direct the main streams' encoder channels ask for the IVDC
 * (bEnableIvdc): tx-isp-t23 then queues the ISP's channel 0 (and 3) the
 * IVDC's store, which HELIX reads.
 *
 *   ingenic-t23-enc frames direct sensor addr [sensor1 addr1 [w h]]
 *
 * @frames pictures of each stream, from five seconds in, the AE frozen
 * after four (T23ENC_SETTLE; T23ENC_AE_RUN keeps it running), at fixed QP
 * 30 and a GOP of 10.  Streams go, as a 4-byte length and the bytes, into 4 MiB
 * slots -- slot 0 the main sensor's main stream, 1 its 640x360 stream, 2
 * and 3 the second sensor's -- at SNAP_DEVMEM (a physical address, for
 * the host's pmemsave) and in /tmp/slot<n>.bin.
 *
 * Built against the T23 SDK's IMP headers (its IMPSensorInfo carries a
 * sensor ID, its IMPEncoderCHNAttr bEnableIvdc): statically with the
 * SDK's uClibc libimp.a, or with -DSNAP_MUSL dynamically against the
 * rootfs's libimp.so and musl (the shims are ingenic-avpu-enc.c's).
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
#include <sys/mman.h>
#ifdef SNAP_MUSL
#include <sys/syscall.h>
#include <ctype.h>
#endif
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_isp.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>

#ifdef SNAP_MUSL
int __libc_start_main(int (*m)(int, char **, char **), int argc, char **argv,
                      void (*init)(void), void (*fini)(void),
                      void (*ldso_fini)(void));

int __uClibc_main(int (*m)(int, char **, char **), int argc, char **argv,
                  void (*init)(void), void (*fini)(void),
                  void (*rtld_fini)(void), void *stack_end)
{
    return __libc_start_main(m, argc, argv, init, fini, rtld_fini);
}

void *mmap(void *addr, size_t len, int prot, int flags, int fd, off_t off)
{
    return (void *)syscall(__NR_mmap2, addr, len, prot, flags, fd,
                           (unsigned long)off >> 12);
}

int ftruncate(int fd, off_t len)
{
    return syscall(__NR_ftruncate, fd, len);
}

/*
 * The older libalog.so (T20/T21) reads uClibc's C-locale ctype tables
 * directly; these are those tables, -128..255, ASCII only.
 */
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
#endif

#define NSTREAM 4
#define CAP ((4 << 20) - 4)            /* a slot: 4 MiB with the length */

static int nframes = 10;
static int nstream = 2;
static unsigned char *buf[NSTREAM];
static unsigned len[NSTREAM];
static int ok[NSTREAM];

/* stream n: FrameSource channel, encoder group and encoder channel */
static const int fs_of[NSTREAM] = { 0, 1, 3, 4 };
static const int grp_of[NSTREAM] = { 0, 2, 1, 3 };
static const int enc_of[NSTREAM] = { 0, 4, 1, 5 };

static void add(int n, const unsigned char *p, unsigned k)
{
    if (len[n] + k <= CAP) {
        memcpy(buf[n] + len[n], p, k);
        len[n] += k;
    }
}

static void *pull(void *arg)
{
    int n = (int)(long)arg, i, k;

    for (i = 0; i < nframes; i++) {
        IMPEncoderStream st;
        unsigned before = len[n];

        if (IMP_Encoder_PollingStream(enc_of[n], 20000) < 0) {
            printf("stream%d: no picture %d\n", n, i);
            return NULL;
        }
        if (IMP_Encoder_GetStream(enc_of[n], &st, 1) < 0) {
            printf("stream%d: GetStream %d failed\n", n, i);
            return NULL;
        }
        for (k = 0; k < (int)st.packCount; k++) {
            add(n, (const unsigned char *)st.pack[k].virAddr,
                st.pack[k].length);
        }
        printf("stream%d picture %d: seq %u, %u packs, %u bytes\n", n, i,
               st.seq, st.packCount, len[n] - before);
        IMP_Encoder_ReleaseStream(enc_of[n], &st);
    }
    ok[n] = 1;
    return NULL;
}

static void sensor_info(IMPSensorInfo *s, const char *name, int addr, int id)
{
    memset(s, 0, sizeof(*s));
    strncpy(s->name, name, sizeof(s->name) - 1);
    s->sensor_id = id;
    s->cbus_type = TX_SENSOR_CONTROL_INTERFACE_I2C;
    strncpy(s->i2c.type, name, sizeof(s->i2c.type) - 1);
    s->i2c.addr = addr;
    s->i2c.i2c_adapter_id = 0;
}

int main(int argc, char **argv)
{
    IMPSensorInfo sen[2];
    IMPFSChnAttr fa[NSTREAM];
    pthread_t tid[NSTREAM];
    int direct = argc > 2 ? atoi(argv[2]) : 0;
    int w = 1920, h = 1080, n, all = 1;

    if (argc < 5) {
        fprintf(stderr, "usage: %s frames direct sensor addr "
                "[sensor1 addr1 [w h]]\n", argv[0]);
        return 2;
    }
    nframes = atoi(argv[1]);
    sensor_info(&sen[0], argv[3], strtol(argv[4], NULL, 0), 0);
    if (argc > 6 && strcmp(argv[5], "-")) {
        sensor_info(&sen[1], argv[5], strtol(argv[6], NULL, 0), 1);
        nstream = 4;
    }
    if (argc > 8) {
        w = atoi(argv[7]);
        h = atoi(argv[8]);
    }
    if (IMP_ISP_Open() < 0 || IMP_ISP_AddSensor(&sen[0]) < 0 ||
        (nstream == 4 && IMP_ISP_AddSensor(&sen[1]) < 0) ||
        IMP_ISP_EnableSensor() < 0) {
        printf("ISP / sensor setup failed\n");
        return 1;
    }
    if (IMP_System_Init() < 0 || IMP_ISP_EnableTuning() < 0) {
        printf("IMP_System_Init / EnableTuning failed\n");
        return 1;
    }
    IMP_ISP_Tuning_SetISPRunningMode(IMPISP_RUNNING_MODE_DAY);
    for (n = 0; n < nstream; n++) {
        IMPFSChnAttr *a = &fa[n];
        IMPEncoderCHNAttr ea;
        IMPCell fs = { DEV_ID_FS, fs_of[n], 0 };
        IMPCell enc = { DEV_ID_ENC, grp_of[n], 0 };
        int main_stream = !(n & 1);

        memset(a, 0, sizeof(*a));
        a->pixFmt = PIX_FMT_NV12;
        a->outFrmRateNum = 25;
        a->outFrmRateDen = 1;
        a->nrVBs = 2;
        a->type = FS_PHY_CHANNEL;
        a->picWidth = main_stream ? w : 640;
        a->picHeight = main_stream ? h : 360;
        if (!main_stream) {
            a->scaler.enable = 1;
            a->scaler.outwidth = 640;
            a->scaler.outheight = 360;
        }
        buf[n] = malloc(CAP);
        if (IMP_FrameSource_CreateChn(fs_of[n], a) < 0 ||
            IMP_FrameSource_SetChnAttr(fs_of[n], a) < 0) {
            printf("fs%d: CreateChn failed\n", fs_of[n]);
            return 1;
        }
        memset(&ea, 0, sizeof(ea));
        ea.encAttr.enType = PT_H264;
        ea.encAttr.profile = 1;
        ea.encAttr.picWidth = a->picWidth;
        ea.encAttr.picHeight = a->picHeight;
        ea.rcAttr.outFrmRate.frmRateNum = 25;
        ea.rcAttr.outFrmRate.frmRateDen = 1;
        ea.rcAttr.maxGop = 10;
        ea.rcAttr.attrRcMode.rcMode = ENC_RC_MODE_FIXQP;
        ea.rcAttr.attrRcMode.attrH264FixQp.qp = 30;
        ea.rcAttr.attrHSkip.hSkipAttr.skipType = IMP_Encoder_STYPE_N1X;
        ea.rcAttr.attrHSkip.maxHSkipType = IMP_Encoder_STYPE_N1X;
        ea.bEnableIvdc = direct && main_stream;
        if (IMP_Encoder_CreateGroup(grp_of[n]) < 0 ||
            IMP_Encoder_CreateChn(enc_of[n], &ea) < 0 ||
            IMP_Encoder_RegisterChn(grp_of[n], enc_of[n]) < 0 ||
            IMP_System_Bind(&fs, &enc) < 0) {
            printf("stream%d: encoder setup failed\n", n);
            return 1;
        }
    }
    for (n = 0; n < nstream; n++) {
        if (IMP_FrameSource_EnableChn(fs_of[n]) < 0) {
            printf("fs%d: EnableChn failed\n", fs_of[n]);
            return 1;
        }
    }
    /*
     * The kernel's AE settles meanwhile; then it is frozen.  On this scene
     * T23's AE keeps flipping the sensor between two gains a step apart
     * (the ISP model's trace: 2272 / 2683 per 256, mean Y 39 / 44, a
     * five-frame cycle), and the HELIX model codes a changed macroblock of
     * a P picture as I_PCM: a whole-frame change makes a picture bigger
     * than libimp's stream buffer takes (0.75 byte a pixel).
     */
    sleep(getenv("T23ENC_SETTLE") ? atoi(getenv("T23ENC_SETTLE")) : 4);
    if (!getenv("T23ENC_AE_RUN")) {
        IMP_ISP_Tuning_SetAeFreeze(IMPISP_TUNING_OPS_MODE_ENABLE);
        sleep(1);
    }
    for (n = 0; n < nstream; n++) {
        if (IMP_Encoder_StartRecvPic(enc_of[n]) < 0) {
            printf("stream%d: StartRecvPic failed\n", n);
            return 1;
        }
    }
    printf("encoding %d streams, direct %d\n", nstream, direct);
    for (n = 0; n < nstream; n++) {
        pthread_create(&tid[n], NULL, pull, (void *)(long)n);
    }
    for (n = 0; n < nstream; n++) {
        pthread_join(tid[n], NULL);
        all &= ok[n];
    }
    for (n = 0; n < nstream; n++) {
        char path[64];
        int fd;

        snprintf(path, sizeof(path), "/tmp/slot%d.bin", n);
        fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            if (write(fd, &len[n], 4) != 4 ||
                write(fd, buf[n], len[n]) != (ssize_t)len[n]) {
                printf("cannot write %s\n", path);
            }
            close(fd);
        }
    }
    if (getenv("SNAP_DEVMEM")) {
        unsigned long base = strtoul(getenv("SNAP_DEVMEM"), NULL, 0);
        int mfd = open("/dev/mem", O_RDWR | O_SYNC);

        for (n = 0; mfd >= 0 && n < nstream; n++) {
            unsigned char *dst = mmap(NULL, 4 << 20, PROT_READ | PROT_WRITE,
                                      MAP_SHARED, mfd, base + (n << 22));

            if (dst == MAP_FAILED) {
                printf("devmem slot %d: no mapping\n", n);
                continue;
            }
            memcpy(dst, &len[n], 4);
            memcpy(dst + 4, buf[n], len[n]);
            munmap(dst, 4 << 20);
        }
        if (mfd >= 0) {
            close(mfd);
        }
    }
    printf("t23-enc: %s\n", all ? "PASS" : "FAIL");
    fflush(stdout);
    close(open("/tmp/t23-enc.done", O_WRONLY | O_CREAT, 0644));
    return all ? 0 : 1;
}
