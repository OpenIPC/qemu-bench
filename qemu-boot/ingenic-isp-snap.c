/*
 * Capture NV12 frames through libimp's FrameSource on an Ingenic T31 --
 * the guest half of qemu-boot/test-ingenic-isp.sh.  Follows the T31 SDK
 * 1.1.5 samples (sample-common.c, sample-Framesource.c): ISP open, add and
 * enable the sensor, IMP_System_Init, tuning on, then FrameSource
 * channel 0 at the sensor's 1920x1080 and channel 1 scaled to 640x360.
 *
 *   ingenic-isp-snap [frames [outdir [sensor [i2c-addr [width height]]]]]
 *
 * (defaults 10, /tmp, gc2053, 0x37, 1920 1080: channel 0's size)
 *
 * Each channel's thread gets @frames frames (default 10), prints a line
 * per frame (sequence, timestamp, a sum of the Y plane) and writes frames
 * @frames - 6 and @frames - 1 (late: the kernel's AE settles in the first
 * twenty or so) to <outdir>/ch<n>-<k>.nv12 -- or, when @outdir is a
 * block device (the SD card; a 1080p frame does not fit in the RAM of a
 * 64 MiB camera beside libimp), into 8 MiB slots on it: channel 0's two
 * frames in slots 0 and 1, channel 1's in 2 and 3.  Exits 0 when every
 * channel got all of its frames.
 *
 * T40 (built with SNAP_DUAL) with SNAP_SENSOR1=<name>:<i2c-addr> in the
 * environment: two sensors
 * (SDK sample-common.c, IMPISP_TOTAL_TWO in the all-cached dual mode), the
 * second on I2C3 through CSI1, and the two channels are 0 (the first
 * sensor's main stream) and 3 (the second's), both at the sensor size;
 * channel 3's frames go where channel 1's would.
 *
 * With SNAP_WDR=1 in the environment the sensor runs its two-exposure
 * (DOL) WDR mode: IMP_ISP_WDR_ENABLE before the sensor is added, as the
 * SDKs' headers ask, and on T40 / T41 the sensor's WDR setting
 * (default_boot 1); on T31 the sensor module is loaded with data_type=2
 * (TX_SENSOR_DATA_TYPE_WDR_DOL) by the harness.
 *
 * Built two ways (test-ingenic-isp.sh does both):
 *   - static, against the SDK's uClibc libimp.a (IMP 1.1.5);
 *   - with -DSNAP_MUSL, dynamically against the rootfs's own libimp.so
 *     (IMP 1.1.6, the build that matches its tx-isp-t31.ko) and musl.
 *     That libimp is a uClibc build, which majestic runs by exporting the
 *     few uClibc-only symbols it calls; this file does the same, and turns
 *     the uClibc crt1.o's call to __uClibc_main into musl's
 *     __libc_start_main (the same first six arguments).  mmap and
 *     ftruncate take a 32-bit off_t in uClibc and a 64-bit one in musl
 *     (in a register pair on o32): libimp's calls would land in musl's
 *     with the arguments shifted, so they go to the system calls here.
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

static int nframes = 10;
static const char *outdir = "/tmp";
static int chn_ok[2];
static int chn_id[2] = { 0, 1 };    /* the FrameSource channel of each */
static int joint;                   /* T40 joint mode: channel 3 on too */

/*
 * The IMP 1.1.6 IMPFSChnAttr (the rootfs's libimp) ends with a front crop
 * that the 1.1.5 headers lack; libimp reads it past the 1.1.5 struct and
 * hands it to the kernel, which then crops channel 0 to whatever follows
 * in memory.  Each attribute gets a zeroed one.
 */
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
        .scaler = { .enable = 1, .outwidth = 640, .outheight = 360 },
        .picWidth = 640, .picHeight = 360,
    } },
};
#define attr(n) (attrs[n].a)
static IMPFSChnAttr joint_attr;     /* channel 3 in a joint mode */

/* O_DIRECT wants aligned buffers: copy through one, 64 KiB at a time */
static int save(int ch, int fd, const unsigned char *p, unsigned size)
{
    static unsigned char buf[2][65536] __attribute__((aligned(4096)));
    unsigned char *b = buf[ch];
    unsigned n;

    while (size) {
        n = size < sizeof(buf[0]) ? size : sizeof(buf[0]);
        memcpy(b, p, n);
        if (write(fd, b, (n + 511) & ~511u) < 0) {
            return -1;
        }
        p += n;
        size -= n;
    }
    return 0;
}

/*
 * SNAP_WDR: the ISP's WDR mode, between IMP_ISP_Open and the sensor
 * (built with SNAP_WDR_API: the T10..T23 libimps have no WDR call)
 */
static int snap_wdr(void)
{
    if (!getenv("SNAP_WDR")) {
        return 0;
    }
#ifndef SNAP_WDR_API
    printf("isp-snap: no WDR in this libimp\n");
    return -1;
#elif defined(SNAP_T40)
    {
        IMPISPTuningOpsMode m = IMPISP_TUNING_OPS_MODE_ENABLE;

        if (IMP_ISP_WDR_ENABLE(IMPVI_MAIN, &m) < 0) {
            printf("isp-snap: IMP_ISP_WDR_ENABLE failed\n");
            return -1;
        }
    }
#else
    if (IMP_ISP_WDR_ENABLE(IMPISP_TUNING_OPS_MODE_ENABLE) < 0) {
        printf("isp-snap: IMP_ISP_WDR_ENABLE failed\n");
        return -1;
    }
#endif
    printf("isp-snap: WDR enabled\n");
    return 0;
}

#ifdef SNAP_LDC
#include <stdint.h>
#include <sys/ioctl.h>
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/*
 * SNAP_LDC=<k1>,<k2>: the ISP's lens distortion correction (HLDC) on.
 * The stock tx-isp-t41.ko (H20230630a) builds IMP_ISP_LDC_*'s LDC block
 * out (TX_ISP_LDC_ENABLE), and nothing in it calls tisp_hldc_set_attr;
 * what reaches the HLDC registers (0x15000..0x15044) is the tuning
 * device /dev/tisp: _IO('t', 1) with module 21 (LDC), sub-module 0 and
 * the 16 halfwords tisp_hldc_write_reg writes.  They are computed here
 * as tisp_hldc_set_attr / tisp_hldc_calc_para would (k1 / k2 of the
 * radial model for luma and chroma, the zoom that keeps the picture
 * filled, the inverse of the largest squared radius, the centre's
 * offset); then IMP_ISP_Tuning_SetModuleControl clears bitBypassHLDC.
 */
static int ldc_pending;

#define TISP_SET_PAR    0x20007401      /* _IO('t', 1), MIPS encoding */
/* tisp_set_par_process's module number (its table index + 1) for the
 * HLDC: 21 in tx-isp-t41.ko, 20 in tx-isp-t40.ko, whose pipe is word 3 */
#ifdef SNAP_T40
#define TISP_LDC        20
#else
#define TISP_LDC        21
#endif
#define TISP_PAR_SIZE   20492

static void hldc_calc(int k1, int k2, int w, int h, int cx, int cy,
                      int32_t *out)
{
    int dxa = cx - 1, dxb = w - cx - 1, dy = MAX(cy - 1, h - cy - 1);
    int dxmax = MAX(dxa, dxb), dxmin = MIN(dxa, dxb);
    int32_t maxr2 = dxmax * dxmax + dy * dy;
    int32_t inv = 0x3fffffff / maxr2;
    int64_t r = (int64_t)maxr2 * inv, r2 = (int64_t)dxmin * dxmin * inv;
    int64_t f1 = ((r * r) >> 30) * k2 + (256LL << 32) - (int64_t)k1 * r;
    int64_t f2 = ((r2 * r2) >> 30) * k2 + (256LL << 32) - (int64_t)k1 * r2;
    int64_t f = MIN(f1, f2);
    int32_t scale = (int32_t)((0x80000LL << 32) / f) - 5;

    out[0] = inv;
    out[1] = scale > 0 ? scale : 1;
    out[2] = cx - w / 2;
    out[3] = cy - h / 2;
}

static int snap_ldc(int w, int h)
{
    static uint32_t par[TISP_PAR_SIZE / 4];
#ifdef SNAP_T40
    uint16_t *p = (uint16_t *)&par[4];      /* par[3]: the pipe, 0 */
#else
    uint16_t *p = (uint16_t *)&par[3];
#endif
    int32_t y[4], uv[4];
    int k1 = 0, k2 = 0, fd;

    if (!getenv("SNAP_LDC")) {
        return 0;
    }
    sscanf(getenv("SNAP_LDC"), "%d,%d", &k1, &k2);
    hldc_calc(k1, k2, w, h, w / 2, h / 2, y);
    hldc_calc(k1, k2, w / 2, h / 2, w / 4, h / 4, uv);
    par[0] = TISP_LDC;
    par[2] = 0;
    p[0] = 1;                   /* the tuning's own (both 1) */
    p[1] = 1;
    p[2] = k1;
    p[3] = k2;
    p[4] = k1;
    p[5] = k2;
    p[6] = y[1];
    p[7] = uv[1];
    p[8] = y[0];
    p[9] = uv[0];
    p[10] = y[2];
    p[11] = y[3];
    p[12] = uv[2];
    p[13] = uv[3];
    fd = open("/dev/tisp", O_RDWR);
    if (fd < 0 || ioctl(fd, TISP_SET_PAR, par) < 0) {
        printf("isp-snap: /dev/tisp LDC parameters failed\n");
        return -1;
    }
    close(fd);
    printf("isp-snap: LDC k1 %d k2 %d: zoom %d / %d, inverse r2 %d / %d\n",
           k1, k2, y[1], uv[1], y[0], uv[0]);
    ldc_pending = 1;
    return 0;
}

/*
 * The stage on (bitBypassHLDC clear) once the first frames are saved:
 * the host compares a picture without the correction with one through it
 */
static void ldc_enable(void)
{
    IMPISPModuleCtl ctl;

    ldc_pending = 0;
    if (IMP_ISP_Tuning_GetModuleControl(IMPVI_MAIN, &ctl) < 0) {
        printf("isp-snap: GetModuleControl failed\n");
        return;
    }
    ctl.bitBypassHLDC = 0;
    if (IMP_ISP_Tuning_SetModuleControl(IMPVI_MAIN, &ctl) < 0) {
        printf("isp-snap: SetModuleControl failed\n");
        return;
    }
    printf("isp-snap: LDC on\n");
}
#else
static int ldc_pending;

static int snap_ldc(int w, int h)
{
    (void)w;
    (void)h;
    return getenv("SNAP_LDC") ? -1 : 0;
}

static void ldc_enable(void)
{
}
#endif

#ifdef SNAP_AF
/*
 * The AF statistics' sharpness metric, sampled while the AE settles: the
 * kernel's AF reads the zones the model fills from the scene
 */
static void *af_watch(void *arg)
{
    int i;

    for (i = 0; i < 12; i++) {
#ifdef SNAP_T40
        IMPISPAFMetricsInfo info;

        usleep(500 * 1000);
        memset(&info, 0, sizeof(info));
        if (IMP_ISP_Tuning_GetAFMetricesInfo(IMPVI_MAIN, &info) == 0) {
            printf("isp-snap: af metric %u alt %u\n", info.af_metrics,
                   info.af_metrics_alt);
        }
#else
        unsigned int metric = 0;

        usleep(500 * 1000);
        if (IMP_ISP_Tuning_GetAFMetrices(&metric) == 0) {
            printf("isp-snap: af metric %u\n", metric);
        }
#endif
    }
    return NULL;
}
#endif

static void *grab(void *arg)
{
    int k = (int)(long)arg, ch = chn_id[k], i;
    IMPFrameInfo *f;
    char path[128];

    if (IMP_FrameSource_SetFrameDepth(ch, attr(k).nrVBs * 2) < 0) {
        printf("ch%d: SetFrameDepth failed\n", ch);
        return NULL;
    }
    for (i = 0; i < nframes; i++) {
        unsigned long long sum = 0;
        unsigned j;

        if (IMP_FrameSource_GetFrame(ch, &f) < 0) {
            printf("ch%d: GetFrame %d failed\n", ch, i);
            return NULL;
        }
        for (j = 0; j < f->width * f->height; j += 61) {
            sum += ((unsigned char *)f->virAddr)[j];
        }
        printf("ch%d frame %d: seq %u %ux%u fmt 0x%x size %u phy 0x%08x "
               "ts %llu ysum %llu\n", ch, i, f->index, f->width, f->height,
               f->pixfmt, f->size, f->phyAddr,
               (unsigned long long)f->timeStamp, sum);
        if (i == nframes - 6 || i == nframes - 1) {
            int fd;

            if (getenv("SNAP_DEVMEM")) {
                /*
                 * SNAP_DEVMEM=<physical address>: the frame into 8 MiB
                 * slot 2 ch + (i == nframes - 1) of memory there (rmem's
                 * top half; libimp's buffers are at its bottom), for a
                 * host with no SD card to read with pmemsave.  /dev/mem's
                 * write() refuses memory past mem=, its mmap() does not.
                 */
                unsigned long base = strtoul(getenv("SNAP_DEVMEM"), NULL, 0);
                int slot = 2 * k + (i == nframes - 1);
                /* SNAP_SLOT_MB: bigger slots for a joint picture */
                unsigned mb = getenv("SNAP_SLOT_MB") ?
                              atoi(getenv("SNAP_SLOT_MB")) : 8;
                int mfd = open("/dev/mem", O_RDWR | O_SYNC);
                void *dst = mfd < 0 ? MAP_FAILED :
                            mmap(NULL, mb << 20, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, mfd,
                                 base + ((unsigned long)slot * mb << 20));

                if (dst == MAP_FAILED) {
                    printf("ch%d: cannot map slot %d\n", ch, slot);
                } else {
                    memcpy(dst, (void *)f->virAddr,
                           f->size < (mb << 20) ? f->size : mb << 20);
                    munmap(dst, mb << 20);
                }
                if (mfd >= 0) {
                    close(mfd);
                }
                fd = -2;
            } else if (!strncmp(outdir, "/dev/", 5)) {
                unsigned long long pos = (2 * k + (i == nframes - 1)) << 23;
                unsigned long long res;

                snprintf(path, sizeof(path), "%s", outdir);
                /* past the page cache, which has no room for a frame */
                fd = open(path, O_WRONLY | O_DIRECT);
                /* _llseek: no off_t in sight, whichever libc this is */
                if (fd >= 0 && syscall(__NR__llseek, fd,
                                       (unsigned long)(pos >> 32),
                                       (unsigned long)pos, &res,
                                       SEEK_SET) < 0) {
                    close(fd);
                    fd = -1;
                }
            } else {
                snprintf(path, sizeof(path), "%s/ch%d-%d.nv12", outdir, ch,
                         i);
                fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
            }
            if (fd == -2) {
                /* written to memory above */
            } else if (fd < 0 ||
                       save(k, fd, (void *)f->virAddr, f->size) < 0) {
                printf("ch%d: cannot write %s\n", ch, path);
            }
            if (fd >= 0) {
                close(fd);
            }
        }
        IMP_FrameSource_ReleaseFrame(ch, f);
        /* both channels' first frames are out by then */
        if (k == 0 && i == nframes - 5 && ldc_pending) {
            ldc_enable();
        }
    }
    IMP_FrameSource_SetFrameDepth(ch, 0);
    chn_ok[k] = 1;
    return NULL;
}

#ifdef SNAP_DUAL
/*
 * The raw snapshots: an RGGB mosaic of the scene, so the navy background
 * (most of the picture) has more blue than red, and green between
 */
static int snap_raw_check(int w, int h, int want)
{
    int k, good = 0;

    for (k = 0; k < 8; k++) {
        char path[32];
        FILE *f;
        long size, n[4] = { 0 };
        double sum[4] = { 0 };
        uint16_t px;
        long i;

        snprintf(path, sizeof(path), "/tmp/snap%d.raw", k);
        f = fopen(path, "rb");
        if (!f) {
            continue;
        }
        fseek(f, 0, SEEK_END);
        size = ftell(f);
        fseek(f, 0, SEEK_SET);
        for (i = 0; i < size / 2 && fread(&px, 2, 1, f) == 1; i++) {
            int x = i % w, y = i / w, c = (y & 1) * 2 + (x & 1);

            sum[c] += px;
            n[c]++;
        }
        fclose(f);
        printf("isp-snap: %s: %ld bytes (%dx%d raw16: %ld), R %.0f Gr %.0f "
               "Gb %.0f B %.0f\n", path, size, w, h,
               2L * w * h, n[0] ? sum[0] / n[0] : 0, n[1] ? sum[1] / n[1] : 0,
               n[2] ? sum[2] / n[2] : 0, n[3] ? sum[3] / n[3] : 0);
        if (size == 2L * w * h && n[0] && sum[3] / n[3] > sum[1] / n[1] &&
            sum[1] / n[1] > sum[0] / n[0]) {
            good++;
        }
    }
    printf("isp-snap: snapraw %d of %d raw frames as expected: %s\n", good,
           want, good == want ? "PASS" : "FAIL");
    return good == want;
}
#endif

int main(int argc, char **argv)
{
    IMPSensorInfo sensor;
#ifdef SNAP_DUAL
    IMPSensorInfo sensor1, sensor2;
    const char *s1 = getenv("SNAP_SENSOR1");
    const char *s2 = getenv("SNAP_SENSOR2");
#endif
    const char *name = "gc2053";
    pthread_t tid[2];
    int ch;

    if (argc > 1) {
        nframes = atoi(argv[1]);
    }
    if (argc > 2) {
        outdir = argv[2];
    }
    if (argc > 3) {
        name = argv[3];
    }
    memset(&sensor, 0, sizeof(sensor));
    strncpy(sensor.name, name, sizeof(sensor.name) - 1);
    sensor.cbus_type = TX_SENSOR_CONTROL_INTERFACE_I2C;
    strncpy(sensor.i2c.type, name, sizeof(sensor.i2c.type) - 1);
    sensor.i2c.addr = argc > 4 ? strtol(argv[4], NULL, 0) : 0x37;
    if (argc > 6) {
        attr(0).picWidth = atoi(argv[5]);
        attr(0).picHeight = atoi(argv[6]);
    }

#ifdef SNAP_T40
    /* T40's IMP takes the sensor's VI number, adapter and interface */
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
    if (getenv("SNAP_WDR")) {
        sensor.default_boot = 1;        /* the sensor's WDR setting */
    }
#ifdef SNAP_DUAL
    if (s1) {
        /* sample-common.c: the second sensor, its input mode, channel 3 */
        IMPISPCameraInputMode mode = {
            .sensor_num = IMPISP_TOTAL_TWO,
            .dual_mode = IMPISP_DUALSENSOR_DUAL_ALLCACHED_MODE,
            .joint_mode = IMPISP_NOT_JOINT,
        };
        if (getenv("SNAP_DUAL_MODE")) {
            mode.dual_mode = atoi(getenv("SNAP_DUAL_MODE"));
        }
        if (getenv("SNAP_JOINT")) {
            mode.joint_mode = atoi(getenv("SNAP_JOINT"));
        }
        if (getenv("SNAP_SWITCH")) {
            /* the frame pattern a cached pipe switches its sensors by */
            mode.dual_mode_switch.en = IMPISP_TUNING_OPS_MODE_ENABLE;
            mode.dual_mode_switch.switch_con = strtol(getenv("SNAP_SWITCH"),
                                                      NULL, 0);
            mode.dual_mode_switch.switch_con_num = 2;
        }
        if (s2) {
            mode.sensor_num = IMPISP_TOTAL_THR;
        }
        const char *colon = strchr(s1, ':');
        size_t len = colon ? (size_t)(colon - s1) : strlen(s1);

        memcpy(&sensor1, &sensor, sizeof(sensor1));
        memset(sensor1.name, 0, sizeof(sensor1.name));
        memset(sensor1.i2c.type, 0, sizeof(sensor1.i2c.type));
        memcpy(sensor1.name, s1, len < sizeof(sensor1.name) ?
               len : sizeof(sensor1.name) - 1);
        memcpy(sensor1.i2c.type, sensor1.name, sizeof(sensor1.i2c.type) - 1);
        sensor1.i2c.addr = colon ? strtol(colon + 1, NULL, 0) : sensor.i2c.addr;
        sensor1.i2c.i2c_adapter_id = 3;
        sensor1.video_interface = IMPISP_SENSOR_VI_MIPI_CSI1;
        sensor1.mclk = IMPISP_SENSOR_MCLK2;
        sensor1.sensor_id = 1;
        if (mode.joint_mode) {
            /*
             * a joint picture on channel 0, twice the sensor's size
             * across (main left / right) or down (above / under); the
             * second stream is channel 1, pipe 0's scaled one
             * (ispcore_pad_event_handle queues channel 0's buffer to
             * both pipes, the secondary's part at its offset)
             */
            joint_attr = attr(0);
            joint = 1;
            if (mode.joint_mode <= IMPISP_MAIN_ON_THE_RIGHT) {
                attr(0).picWidth *= 2;
            } else {
                attr(0).picHeight *= 2;
            }
        } else {
            chn_id[1] = 3;
            attr(1) = attr(0);
        }
        if (s2) {
            /* the third sensor: VIC2 on the DVP port (two CSI hosts), I2C0 */
            colon = strchr(s2, ':');
            len = colon ? (size_t)(colon - s2) : strlen(s2);
            memcpy(&sensor2, &sensor1, sizeof(sensor2));
            memset(sensor2.name, 0, sizeof(sensor2.name));
            memset(sensor2.i2c.type, 0, sizeof(sensor2.i2c.type));
            memcpy(sensor2.name, s2, len < sizeof(sensor2.name) ?
                   len : sizeof(sensor2.name) - 1);
            memcpy(sensor2.i2c.type, sensor2.name, sizeof(sensor2.i2c.type) - 1);
            sensor2.i2c.addr = colon ? strtol(colon + 1, NULL, 0) :
                               sensor.i2c.addr;
            sensor2.i2c.i2c_adapter_id = getenv("SNAP_I2C2") ?
                                         atoi(getenv("SNAP_I2C2")) : 0;
            sensor2.video_interface = getenv("SNAP_VI2") ?
                                      atoi(getenv("SNAP_VI2")) :
                                      IMPISP_SENSOR_VI_DVP;
            sensor2.mclk = IMPISP_SENSOR_MCLK2;
            sensor2.sensor_id = 2;
        }
        printf("isp-snap: input mode: %d sensors, dual mode %d, joint %d\n",
               mode.sensor_num, mode.dual_mode, mode.joint_mode);
        if (IMP_ISP_Open() < 0 || IMP_ISP_SetCameraInputMode(&mode) < 0 ||
            IMP_ISP_AddSensor(IMPVI_MAIN, &sensor) < 0 ||
            IMP_ISP_AddSensor(IMPVI_SEC, &sensor1) < 0 ||
            (s2 && IMP_ISP_AddSensor(IMPVI_THR, &sensor2) < 0) ||
            IMP_ISP_EnableSensor(IMPVI_MAIN, &sensor) < 0 ||
            IMP_ISP_EnableSensor(IMPVI_SEC, &sensor1) < 0 ||
            (s2 && IMP_ISP_EnableSensor(IMPVI_THR, &sensor2) < 0)) {
            printf("ISP / dual sensor setup failed\n");
            return 1;
        }
        printf("isp-snap: two sensors, %s and %s\n", sensor.name,
               sensor1.name);
    } else
#endif
    if (IMP_ISP_Open() < 0 || snap_wdr() < 0 ||
        IMP_ISP_AddSensor(IMPVI_MAIN, &sensor) < 0 ||
        IMP_ISP_EnableSensor(IMPVI_MAIN, &sensor) < 0) {
#else
    if (IMP_ISP_Open() < 0 || snap_wdr() < 0 ||
        IMP_ISP_AddSensor(&sensor) < 0 || IMP_ISP_EnableSensor() < 0) {
#endif
        printf("ISP / sensor setup failed\n");
        return 1;
    }
    if (IMP_System_Init() < 0) {
        printf("IMP_System_Init failed\n");
        return 1;
    }
    if (IMP_ISP_EnableTuning() < 0) {
        printf("IMP_ISP_EnableTuning failed\n");
        return 1;
    }
    if (snap_ldc(attr(0).picWidth, attr(0).picHeight) < 0) {
        return 1;
    }
#ifndef SNAP_T40
    IMP_ISP_Tuning_SetISPRunningMode(IMPISP_RUNNING_MODE_DAY);
#endif
    for (ch = 0; ch < 2; ch++) {
        if (IMP_FrameSource_CreateChn(chn_id[ch], &attr(ch)) < 0 ||
            IMP_FrameSource_SetChnAttr(chn_id[ch], &attr(ch)) < 0) {
            printf("ch%d: CreateChn failed\n", chn_id[ch]);
            return 1;
        }
    }
    if (joint) {
        /*
         * the secondary pipe's channel, created and enabled as well (its
         * scaler writes the secondary's part of channel 0's buffer); its
         * frames come through channel 0
         */
        if (IMP_FrameSource_CreateChn(3, &joint_attr) < 0 ||
            IMP_FrameSource_SetChnAttr(3, &joint_attr) < 0) {
            printf("ch3: CreateChn failed\n");
            return 1;
        }
    }
    for (ch = 0; ch < 2; ch++) {
        if (IMP_FrameSource_EnableChn(chn_id[ch]) < 0) {
            printf("ch%d: EnableChn failed\n", chn_id[ch]);
            return 1;
        }
    }
    if (joint && IMP_FrameSource_EnableChn(3) < 0) {
        printf("ch3: EnableChn failed\n");
        return 1;
    }
    printf("streaming\n");
#ifdef SNAP_AF
    {
        pthread_t af;

        pthread_create(&af, NULL, af_watch, NULL);
        pthread_detach(af);
    }
#endif
    for (ch = 0; ch < 2; ch++) {
        pthread_create(&tid[ch], NULL, grab, (void *)(long)ch);
    }
    for (ch = 0; ch < 2; ch++) {
        pthread_join(tid[ch], NULL);
    }
#ifdef SNAP_DUAL
    if (getenv("SNAP_SNAPRAW")) {
        /*
         * T40's raw snapshot, while streaming: the driver's VIC MDMA
         * stores N raw frames of sensor S, which it saves as
         * /tmp/snap<k>.raw (16 bits a pixel)
         */
        char cmd[96], *u;

        /* "S_N": the harness passes words without spaces */
        snprintf(cmd, sizeof(cmd), "echo snapraw %s > /proc/jz/isp/isp-w02",
                 getenv("SNAP_SNAPRAW"));
        while ((u = strchr(cmd, '_'))) {
            *u = ' ';
        }
        printf("isp-snap: %s: %d\n", cmd, system(cmd));
        u = strchr(getenv("SNAP_SNAPRAW"), '_');
        if (!snap_raw_check(attr(0).picWidth, attr(0).picHeight,
                            u ? atoi(u + 1) : 1)) {
            chn_ok[0] = 0;
        }
    }
#endif
    /* the verdict before the teardown, which T40's IMP can hang in */
    printf("isp-snap: frames %s\n", chn_ok[0] && chn_ok[1] ? "PASS" : "FAIL");
    fflush(stdout);
    {
        int dfd = open("/tmp/isp-snap.done", O_WRONLY | O_CREAT, 0644);

        if (dfd >= 0) {
            if (write(dfd, chn_ok[0] && chn_ok[1] ? "PASS\n" : "FAIL\n",
                      5) < 0) {
                printf("cannot write /tmp/isp-snap.done\n");
            }
            close(dfd);
        }
    }
    for (ch = 0; ch < 2; ch++) {
        IMP_FrameSource_DisableChn(chn_id[ch]);
    }
    for (ch = 0; ch < 2; ch++) {
        IMP_FrameSource_DestroyChn(chn_id[ch]);
    }
#ifdef SNAP_T40
    /* sample-common.c's order: the sensors, then tuning */
    IMP_System_Exit();
    IMP_ISP_DisableSensor(IMPVI_MAIN);
#ifdef SNAP_DUAL
    if (s1) {
        IMP_ISP_DisableSensor(IMPVI_SEC);
    }
    if (s1 && s2) {
        IMP_ISP_DisableSensor(IMPVI_THR);
    }
#endif
    IMP_ISP_DelSensor(IMPVI_MAIN, &sensor);
#ifdef SNAP_DUAL
    if (s1) {
        IMP_ISP_DelSensor(IMPVI_SEC, &sensor1);
    }
    if (s1 && s2) {
        IMP_ISP_DelSensor(IMPVI_THR, &sensor2);
    }
#endif
    IMP_ISP_DisableTuning();
#else
    IMP_ISP_DisableTuning();
    IMP_System_Exit();
    IMP_ISP_DisableSensor();
    IMP_ISP_DelSensor(&sensor);
#endif
    IMP_ISP_Close();
    printf("isp-snap: %s\n", chn_ok[0] && chn_ok[1] ? "PASS" : "FAIL");
    return chn_ok[0] && chn_ok[1] ? 0 : 1;
}
