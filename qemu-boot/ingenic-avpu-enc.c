/*
 * Encode the emulated sensor's frames through libimp's IMP_Encoder -- the
 * guest half of qemu-boot/test-ingenic-avpu.sh, for the AVPU model
 * (hw/misc/ingenic-avpu.c) on T31, T40 and T41.  The ISP and FrameSource
 * setup is ingenic-isp-snap.c's (channel 0 at the sensor's size, channel
 * 1 scaled to 640x360); each FrameSource channel is bound to an encoder
 * group with one channel, as the SDK samples (sample-common.c) do.
 *
 *   ingenic-avpu-enc h264|h265|jpeg frames outdir sensor i2c-addr w h
 *
 * Video: @frames pictures of each channel, from four seconds into the
 * stream, once the AE has settled (fixed QP 30, a GOP of 10, so
 * there are I and P pictures), the packs of every IMPEncoderStream
 * appended to an in-memory stream (they may wrap at the end of the stream
 * buffer).  JPEG: one snapshot of each channel.  The streams go to
 * <outdir>/enc<n>.<ext> and, as a 4-byte length and the bytes, to
 * <outdir>/slot<n>.bin (for the SD card), and with SNAP_DEVMEM=<physical
 * address> also into 4 MiB slots of memory there for the host to read
 * with pmemsave: T40's SDHCI finds no card.
 *
 * With -DENC_OSD (T41; T40 builds too) and AVPU_OSD=1 in the environment, channel 0
 * goes through an OSD group with a red rectangle (6 pixels wide) and a
 * green horizontal line near its bottom right corner -- drawn by the
 * DRAWBOX through libimp's OSD -- and "osd: x y w h lx ly ll" reports
 * where, for the host to check in the decoded pictures.
 *
 * Built like ingenic-isp-snap.c: -DSNAP_MUSL against the rootfs's own
 * libimp.so and musl (the shims below are that file's), -DSNAP_T40 for
 * T40's IMP 1.2 sensor API; T41 adds -DSNAP_I2C_ADAPTER=0
 * -DSNAP_MCLK=IMPISP_SENSOR_MCLK0 and links the SDK's libimp.a statically.
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
#include <sys/mman.h>
#ifdef SNAP_MUSL
#include <ctype.h>
#endif
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_isp.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>
#ifdef ENC_OSD
#include <imp/imp_osd.h>
#endif

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

#define NCHN 2
#define CAP ((4 << 20) - 4)            /* a slot: 4 MiB with the length */

static int nframes = 10;
static int jpeg;
static unsigned char *buf[NCHN];
static unsigned len[NCHN];
static int chn_ok[NCHN];

/* as ingenic-isp-snap.c: IMP 1.1.6's IMPFSChnAttr ends with a front crop */
static struct {
    IMPFSChnAttr a;
    IMPFSChnCrop fcrop;
} attrs[NCHN] = {
    { {
        .pixFmt = PIX_FMT_NV12, .outFrmRateNum = 25, .outFrmRateDen = 1,
        .nrVBs = 2, .type = FS_PHY_CHANNEL,
        .picWidth = 1920, .picHeight = 1080,
    } }, { {
        .pixFmt = PIX_FMT_NV12, .outFrmRateNum = 25, .outFrmRateDen = 1,
        .nrVBs = 2, .type = FS_PHY_CHANNEL,
        .scaler = { .enable = 1, .outwidth = 640, .outheight = 360 },
        .picWidth = 640, .picHeight = 360,
    } },
};
#define attr(n) (attrs[n].a)

static void add(int ch, const unsigned char *p, unsigned n)
{
    if (len[ch] + n <= CAP) {
        memcpy(buf[ch] + len[ch], p, n);
        len[ch] += n;
    }
}

/* the encoder channel of FrameSource channel ch: JPEG's from 4, as the
 * SDK samples have them (T40's libimp faults on a JPEG channel 0) */
#define ENC(ch) (jpeg ? 4 + (ch) : (ch))

static void *pull(void *arg)
{
    int ch = (int)(long)arg, i, k;
    const int want = jpeg ? 1 : nframes;

    for (i = 0; i < want; i++) {
        IMPEncoderStream st;
        unsigned before = len[ch];

        if (IMP_Encoder_PollingStream(ENC(ch), 20000) < 0) {
            printf("enc%d: no stream %d\n", ch, i);
            return NULL;
        }
        if (IMP_Encoder_GetStream(ENC(ch), &st, 1) < 0) {
            printf("enc%d: GetStream %d failed\n", ch, i);
            return NULL;
        }
        for (k = 0; k < (int)st.packCount; k++) {
            const IMPEncoderPack *pk = &st.pack[k];
            const unsigned char *v = (const unsigned char *)st.virAddr;

            if (pk->offset + pk->length > st.streamSize) {
                unsigned first = st.streamSize - pk->offset;

                add(ch, v + pk->offset, first);
                add(ch, v, pk->length - first);
            } else {
                add(ch, v + pk->offset, pk->length);
            }
        }
        printf("enc%d picture %d: seq %u, %u packs, %u bytes\n", ch, i,
               st.seq, st.packCount, len[ch] - before);
        IMP_Encoder_ReleaseStream(ENC(ch), &st);
    }
    chn_ok[ch] = 1;
    return NULL;
}

#ifdef ENC_OSD
/* OSD group 0 between FrameSource channel 0 and encoder group 0 */
static int osd_setup(int w, int h)
{
    int x = w - 400, y = h - 320, lx = w - 400, ly = h - 120;
    IMPOSDRgnAttr ra;
    IMPOSDGrpRgnAttr ga;
    IMPRgnHandle rect, line;

    if (IMP_OSD_CreateGroup(0) < 0) {
        return -1;
    }
    rect = IMP_OSD_CreateRgn(NULL);
    line = IMP_OSD_CreateRgn(NULL);
    if (rect < 0 || line < 0 || IMP_OSD_RegisterRgn(rect, 0, NULL) < 0 ||
        IMP_OSD_RegisterRgn(line, 0, NULL) < 0) {
        return -1;
    }
    memset(&ra, 0, sizeof(ra));
    ra.type = OSD_REG_RECT;
    ra.rect.p0.x = x;
    ra.rect.p0.y = y;
    ra.rect.p1.x = x + 256 - 1;
    ra.rect.p1.y = y + 128 - 1;
    ra.fmt = PIX_FMT_MONOWHITE;
    ra.data.lineRectData.color = OSD_RED;
    ra.data.lineRectData.linewidth = 6;
    if (IMP_OSD_SetRgnAttr(rect, &ra) < 0) {
        return -1;
    }
    memset(&ra, 0, sizeof(ra));
    ra.type = OSD_REG_HORIZONTAL_LINE;
    ra.line.p0.x = lx;
    ra.line.p0.y = ly;
    ra.data.lineRectData.color = OSD_GREEN;
    ra.data.lineRectData.linewidth = 6;
    ra.data.lineRectData.linelength = 256;
    if (IMP_OSD_SetRgnAttr(line, &ra) < 0) {
        return -1;
    }
    memset(&ga, 0, sizeof(ga));
    ga.show = 1;
    ga.layer = 1;
    ga.scalex = 1;
    ga.scaley = 1;
    if (IMP_OSD_SetGrpRgnAttr(rect, 0, &ga) < 0 ||
        IMP_OSD_SetGrpRgnAttr(line, 0, &ga) < 0 || IMP_OSD_Start(0) < 0) {
        return -1;
    }
    printf("osd: %d %d 256 128 %d %d 256\n", x, y, lx, ly);
    return 0;
}
#endif

int main(int argc, char **argv)
{
    IMPSensorInfo sensor;
    IMPEncoderChnAttr ea;
    const char *codec = argc > 1 ? argv[1] : "h264";
    const char *outdir = argc > 3 ? argv[3] : "/tmp";
    const char *name = argc > 4 ? argv[4] : "gc2053";
    IMPEncoderProfile prof = IMP_ENC_PROFILE_AVC_MAIN;
    pthread_t tid[NCHN];
    int ch;

    if (argc > 2) {
        nframes = atoi(argv[2]);
    }
    if (!strcmp(codec, "h265")) {
        prof = IMP_ENC_PROFILE_HEVC_MAIN;
    } else if (!strcmp(codec, "jpeg")) {
        prof = IMP_ENC_PROFILE_JPEG;
        jpeg = 1;
    } else if (!strcmp(codec, "h264b")) {
        prof = IMP_ENC_PROFILE_AVC_BASELINE;
    }
    memset(&sensor, 0, sizeof(sensor));
    strncpy(sensor.name, name, sizeof(sensor.name) - 1);
    sensor.cbus_type = TX_SENSOR_CONTROL_INTERFACE_I2C;
    strncpy(sensor.i2c.type, name, sizeof(sensor.i2c.type) - 1);
    sensor.i2c.addr = argc > 5 ? strtol(argv[5], NULL, 0) : 0x37;
    if (argc > 7) {
        attr(0).picWidth = atoi(argv[6]);
        attr(0).picHeight = atoi(argv[7]);
    }
#ifdef SNAP_T40
#ifndef SNAP_I2C_ADAPTER
#define SNAP_I2C_ADAPTER 1          /* T40: I2C1; T41: 0 */
#endif
#ifndef SNAP_MCLK
#define SNAP_MCLK IMPISP_SENSOR_MCLK1
#endif
    sensor.i2c.i2c_adapter_id = SNAP_I2C_ADAPTER;
    sensor.rst_gpio = -1;
    sensor.pwdn_gpio = -1;
    sensor.power_gpio = -1;
    sensor.video_interface = IMPISP_SENSOR_VI_MIPI_CSI0;
    sensor.mclk = SNAP_MCLK;
    if (IMP_ISP_Open() < 0 || IMP_ISP_AddSensor(IMPVI_MAIN, &sensor) < 0 ||
        IMP_ISP_EnableSensor(IMPVI_MAIN, &sensor) < 0) {
#else
    if (IMP_ISP_Open() < 0 || IMP_ISP_AddSensor(&sensor) < 0 ||
        IMP_ISP_EnableSensor() < 0) {
#endif
        printf("ISP / sensor setup failed\n");
        return 1;
    }
    if (IMP_System_Init() < 0 || IMP_ISP_EnableTuning() < 0) {
        printf("IMP_System_Init / EnableTuning failed\n");
        return 1;
    }
#ifndef SNAP_T40
    IMP_ISP_Tuning_SetISPRunningMode(IMPISP_RUNNING_MODE_DAY);
#endif
    for (ch = 0; ch < NCHN; ch++) {
        IMPCell fs = { DEV_ID_FS, ch, 0 }, enc = { DEV_ID_ENC, ch, 0 };
        int osd = 0;

        buf[ch] = malloc(CAP);
        if (IMP_FrameSource_CreateChn(ch, &attr(ch)) < 0 ||
            IMP_FrameSource_SetChnAttr(ch, &attr(ch)) < 0) {
            printf("fs%d: CreateChn failed\n", ch);
            return 1;
        }
        memset(&ea, 0, sizeof(ea));
        if (IMP_Encoder_CreateGroup(ch) < 0 ||
            IMP_Encoder_SetDefaultParam(&ea, prof, IMP_ENC_RC_MODE_FIXQP,
                                        attr(ch).picWidth,
                                        attr(ch).picHeight, 25, 1,
                                        jpeg ? 0 : 10, 2, jpeg ? 25 : 30,
                                        0) < 0 ||
#ifdef ENC_IVPU
            /* T41: JPEG on the HELIX JPEG engine (/dev/soc_vpu) */
            ((jpeg && getenv("AVPU_IVPU") &&
              (ea.encAttr.encVputype = IMP_ENC_TPYE_IVPU)), 0) ||
#endif
            IMP_Encoder_CreateChn(ENC(ch), &ea) < 0 ||
            IMP_Encoder_RegisterChn(ch, ENC(ch)) < 0) {
            printf("enc%d: setup failed\n", ch);
            return 1;
        }
#ifdef ENC_OSD
        if (ch == 0 && getenv("AVPU_OSD")) {
            IMPCell oc = { DEV_ID_OSD, 0, 0 };

            if (osd_setup(attr(0).picWidth, attr(0).picHeight) < 0 ||
                IMP_System_Bind(&fs, &oc) < 0 ||
                IMP_System_Bind(&oc, &enc) < 0) {
                printf("osd: setup failed\n");
                return 1;
            }
            osd = 1;
        }
#endif
        if (!osd && IMP_System_Bind(&fs, &enc) < 0) {
            printf("enc%d: bind failed\n", ch);
            return 1;
        }
    }
    for (ch = 0; ch < NCHN; ch++) {
        if (IMP_FrameSource_EnableChn(ch) < 0) {
            printf("ch%d: EnableChn failed\n", ch);
            return 1;
        }
    }
    /* the kernel's AE settles in the first seconds of the stream */
    sleep(4);
    for (ch = 0; ch < NCHN; ch++) {
        if (IMP_Encoder_StartRecvPic(ENC(ch)) < 0) {
            printf("ch%d: StartRecvPic failed\n", ch);
            return 1;
        }
    }
    printf("encoding %s\n", codec);
    for (ch = 0; ch < NCHN; ch++) {
        pthread_create(&tid[ch], NULL, pull, (void *)(long)ch);
    }
    for (ch = 0; ch < NCHN; ch++) {
        pthread_join(tid[ch], NULL);
    }
    for (ch = 0; ch < NCHN; ch++) {
        char path[128];
        int fd;

        snprintf(path, sizeof(path), "%s/enc%d.%s", outdir, ch,
                 jpeg ? "jpg" : prof == IMP_ENC_PROFILE_HEVC_MAIN ? "h265"
                                                                : "h264");
        fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0 || write(fd, buf[ch], len[ch]) != (ssize_t)len[ch]) {
            printf("cannot write %s\n", path);
        }
        if (fd >= 0) {
            close(fd);
        }
        snprintf(path, sizeof(path), "%s/slot%d.bin", outdir, ch);
        fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0 || write(fd, &len[ch], 4) != 4 ||
            write(fd, buf[ch], len[ch]) != (ssize_t)len[ch]) {
            printf("cannot write %s\n", path);
        }
        if (fd >= 0) {
            close(fd);
        }
    }
    if (getenv("SNAP_DEVMEM")) {
        unsigned long base = strtoul(getenv("SNAP_DEVMEM"), NULL, 0);
        int mfd = open("/dev/mem", O_RDWR | O_SYNC);

        for (ch = 0; mfd >= 0 && ch < NCHN; ch++) {
            unsigned char *dst = mmap(NULL, 4 << 20, PROT_READ | PROT_WRITE,
                                      MAP_SHARED, mfd, base + (ch << 22));

            if (dst == MAP_FAILED) {
                printf("devmem slot %d: no mapping\n", ch);
                continue;
            }
            memcpy(dst, &len[ch], 4);
            memcpy(dst + 4, buf[ch], len[ch]);
            munmap(dst, 4 << 20);
        }
        if (mfd >= 0) {
            close(mfd);
        }
    }
    printf("avpu-enc: %s %s\n", codec,
           chn_ok[0] && chn_ok[1] ? "PASS" : "FAIL");
    fflush(stdout);
    close(open("/tmp/avpu-enc.done", O_WRONLY | O_CREAT, 0644));
    /* no teardown: T40's IMP can hang in it, and the test powers off */
    return chn_ok[0] && chn_ok[1] ? 0 : 1;
}
