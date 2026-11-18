/*
 * Feed libimp's own H.264 encoder (the x264 port behind IMP_Encoder on
 * T21/T23: i264e_*) with pictures from memory and write what it returns,
 * for checking the ingenic-vpu model end to end without an ISP.
 *
 * libimp keeps its encoder exported: i264e_param_default / i264e_init /
 * i264e_encode / i264e_get_bitstream / i264e_release_bitstream.  The
 * parameter and picture fields set here are the ones IMP_Encoder_CreateChn
 * and do_channel_process_h264e set (T23 SDK 1.1.0 libimp.so, which is the
 * library OpenIPC ships for T23; offsets checked against T21's):
 *
 *   param +4 soc (the library's row in pf_array: 5), +8 b_tlb 0, +12 1,
 *         +20 0 (no user stream buffer), +36 -1, +40 i_threads 1,
 *         +56/+60 width/height, +64 i_csp 4 (NV12),
 *         +68 profile_idc, +200 rc method 0 (fixed QP; +204 QP) or 1
 *         (CBR: +208/+212 min/max QP, +228 kbit/s, +244/+248 QP steps),
 *         +408/+412 fps,
 *         +440.. malloc(chn, size, align) / free / virt_to_phys /
 *         phys_to_virt: libimp's buffers come from these.
 *   pic   +8 force IDR, +16 pts, +40 i_csp, +44 planes, +48/+52 strides,
 *         +64/+68 lines, +96/+100 Y / CbCr (virtual).
 *
 * Memory is a /dev/mem window over the reserved RAM the kernel leaves
 * alone (mem=40M: 0x2800000 up), so physical addresses are known.
 *
 * Built against OpenIPC's musl libc.so with the uClibc toolchain:
 * uClibc's crt1.o calls __uClibc_main, defined below on top of musl's
 * __libc_start_main; the other uClibc-only symbols libimp wants are
 * defined here too (OpenIPC's majestic does the same).
 *
 *   ingenic-vpu-feed OUT [frames [WxH [qp]]]
 *
 * Environment: FEED_LIB t20|t21|t23|t30 (the library's layout; default
 * t23), FEED_CBR=kbit/s (CBR instead of fixed QP), FEED_JPEG=1 (JPEG),
 * FEED_H265=1 (T30's H.265), FEED_SOC (pf_array row), FEED_DEBUG.
 * qemu/tests/test-ingenic-vpu-feed.py builds it into each SoC's rootfs,
 * runs it and decodes what it writes.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>

/* musl's mmap takes a 64-bit off_t; uClibc's headers say 32 */
void *musl_mmap(void *a, size_t n, int prot, int flags, int fd,
                long long off) __asm__("mmap");

/* ---- what libimp expects from the program ------------------------------ */

int __libc_start_main(int (*)(int, char **, char **), int, char **,
                      void (*)(void), void (*)(void), void (*)(void));

int __uClibc_main(int (*m)(int, char **, char **), int argc, char **argv,
                  void (*init)(void), void (*fini)(void),
                  void (*rtld_fini)(void), void *stack_end)
{
    (void)stack_end;
    return __libc_start_main(m, argc, argv, init, fini, rtld_fini);
}

int IMP_Log_Get_Option(void)
{
    return 0;
}

void imp_log_fun(int level, int option, int type, const char *tag,
                 const char *file, int line, const char *func,
                 const char *fmt, ...)
{
    va_list ap;

    (void)option; (void)type; (void)file; (void)func;
    fprintf(stderr, "[%d %s:%d] ", level, tag ? tag : "", line);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

void __assert(const char *a, const char *f, unsigned l, const char *fn)
{
    fprintf(stderr, "assert %s %s:%u %s\n", a, f, l, fn ? fn : "");
    abort();
}

int __fgetc_unlocked(FILE *f)
{
    return fgetc(f);
}

void __pthread_register_cancel(__pthread_unwind_buf_t *b) { (void)b; }
void __pthread_unregister_cancel(__pthread_unwind_buf_t *b) { (void)b; }
void __pthread_unwind_next(__pthread_unwind_buf_t *b) { (void)b; abort(); }

/* SIGILL/SIGSEGV: say where (the kernel's MIPS sigaction, raw) */
long syscall(long n, ...);

static void feed_fault(int sig, void *si, void *uc)
{
    /* ucontext: mcontext at 24, sc_pc (64-bit) at +8, ra (sc_regs[31]) */
    uint32_t pc = *(uint32_t *)((uint8_t *)uc + 32);
    uint32_t ra = *(uint32_t *)((uint8_t *)uc + 40 + 31 * 8);

    (void)si;
    fprintf(stderr, "feed: ra %08x\n", ra);
    fprintf(stderr, "feed: signal %d at %08x: %08x\n", sig, pc,
            sig == 4 ? *(uint32_t *)pc : 0);
    _exit(128 + sig);
}

static void feed_catch(int sig)
{
    struct { unsigned flags; void *h; unsigned long mask[4]; } sa = {
        .flags = 8 /* SA_SIGINFO */, .h = (void *)feed_fault,
    };

    syscall(4194 /* rt_sigaction */, sig, &sa, NULL, 16);
}

/* ---- libimp's encoder ---------------------------------------------------- */

int i264e_param_default(void *param);
void *i264e_init(void *param);
int i264e_encode(void *h, void *pic);
int i264e_get_bitstream(void *h, void **nal, int *nnal, void **pic_out,
                        void **a, void **b, void **frame);
int i264e_release_bitstream(void *h, void *nal, void *frame);

/*
 * IMP's EncoderInit allocates the engine's bitstream buffer, shared by
 * every channel and guarded by bsbufsem (an IMPAlloc: +128 virtual,
 * +132 physical, +136 size; +96.. in the T20/T30 library).
 */
#include <semaphore.h>
extern uint8_t alloc_vpu_bs[];
extern sem_t bsbufsem;

/* T30's H.265 encoder (an x265 port, C++ inside, C entry points) */
#define WEAK __attribute__((weak))
WEAK int i265e_param_default(void *param);
WEAK void *i265e_init(void *param);
WEAK int i265e_encode(void *h, void *pic);
WEAK int i265e_get_bitstream(void *h, void **nal, int *nnal, void **a,
                             void **b, void **c, void **task);
WEAK int i265e_release_bitstream(void *h, void *nal, void *task);

/* libimp's JPEG encoder: SOI..SOS and EOI in software around the scan */
int ijpege_param_default(void *param);
void *ijpege_init(void *param);
int ijpege_encode(void *h, void **nal, int *nnal, void *pic, void *x);

/* ---- physical memory ----------------------------------------------------- */

#define PHYS_BASE   0x02800000u
#define PHYS_SIZE   (24u << 20)

static uint8_t *mem;
static uint32_t brk_off;

static void *feed_malloc(int chn, int size, int align)
{
    uint32_t a = align > 0 ? (uint32_t)align : 32;
    uint32_t off = (brk_off + a - 1) / a * a;

    (void)chn;
    if (off + (uint32_t)size > PHYS_SIZE) {
        fprintf(stderr, "feed: out of memory (%d)\n", size);
        return NULL;
    }
    brk_off = off + size;
    memset(mem + off, 0, size);
    return mem + off;
}

static void feed_free(void *p) { (void)p; }

static uint32_t feed_v2p(void *v)
{
    return PHYS_BASE + (uint32_t)((uint8_t *)v - mem);
}

static void *feed_p2v(uint32_t p)
{
    return mem + (p - PHYS_BASE);
}

#define P32(b, o)   (*(int32_t *)((uint8_t *)(b) + (o)))
#define PP(b, o)    (*(void **)((uint8_t *)(b) + (o)))

/* a test picture: colour bars over a gradient, and a square that moves */
static void draw(uint8_t *y, uint8_t *uv, int w, int h, int f)
{
    static const uint8_t bars[8][3] = {
        { 235, 128, 128 }, { 210, 16, 146 }, { 170, 166, 16 },
        { 145, 54, 34 }, { 106, 202, 222 }, { 81, 90, 240 },
        { 41, 240, 110 }, { 16, 128, 128 },
    };
    int r, x, sx = (f * 16) % (w - 64), sy = h / 2;

    for (r = 0; r < h; r++) {
        for (x = 0; x < w; x++) {
            const uint8_t *b = bars[x * 8 / w];
            int v = r < h / 2 ? b[0] : 16 + 219 * x / w;

            if (x >= sx && x < sx + 64 && r >= sy && r < sy + 64) {
                v = 235;
            }
            y[r * w + x] = v;
            if (!(r & 1) && !(x & 1)) {
                uv[(r / 2) * w + x] = r < h / 2 ? b[1] : 128;
                uv[(r / 2) * w + x + 1] = r < h / 2 ? b[2] : 128;
            }
        }
    }
}

/*
 * H.265 (T30): i265e_param_t (i265e_param_dump names it) +0 socType 3,
 * +4 bUserNalbuf, +8/+12 task/thread counts, +20/+24 fps, +28 GOP,
 * +36 internalCsp 4 (NV12), +40..+52 size and maximum size, +72 rc mode
 * (0 fixed QP), +76 QP; +584.. malloc / free / virt_to_phys /
 * phys_to_virt as IMP_Encoder_CreateChn passes them.  i265e_pic_t:
 * +0 pts, +28 i_csp, +36/+40 strides, +52/+56 lines, +84/+88 planes.
 */
static int feed_h265(const char *out, int frames, int w, int h, int qp,
                     uint8_t *y, uint8_t *uv)
{
    static uint8_t param[2048], pic[1024];
    void *enc;
    FILE *fo;
    int f, total = 0, ws = (w + 15) & ~15, hs = (h + 15) & ~15;

    if (!i265e_init) {
        fprintf(stderr, "feed: this libimp has no H.265 encoder\n");
        return 1;
    }
    i265e_param_default(param);
    P32(param, 0) = 3;
    param[4] = 0;
    param[5] = 0;
    P32(param, 8) = 1;
    P32(param, 12) = 1;
    P32(param, 20) = 25;
    P32(param, 24) = 1;
    P32(param, 28) = 25;
    P32(param, 36) = 4;
    P32(param, 40) = w;
    P32(param, 44) = h;
    P32(param, 48) = w;
    P32(param, 52) = h;
    P32(param, 72) = 0;
    P32(param, 76) = qp;
    PP(param, 584) = (void *)feed_malloc;
    PP(param, 588) = (void *)feed_free;
    PP(param, 592) = (void *)feed_v2p;
    PP(param, 596) = (void *)feed_p2v;
    enc = i265e_init(param);
    if (!enc) {
        fprintf(stderr, "feed: i265e_init failed\n");
        return 1;
    }
    fo = fopen(out, "wb");
    for (f = 0; f < frames; f++) {
        void *nal = NULL, *a = NULL, *b = NULL, *c = NULL, *task = NULL;
        int nnal = 0, i, ret;

        draw(y, uv, ws, h, f);
        memset(pic, 0, sizeof(pic));
        P32(pic, 0) = f;
        P32(pic, 28) = 4;
        P32(pic, 32) = 2;
        P32(pic, 36) = ws;
        P32(pic, 40) = ws;
        P32(pic, 52) = hs;
        P32(pic, 56) = hs / 2;
        PP(pic, 84) = y;
        PP(pic, 88) = uv;
        ret = i265e_encode(enc, pic);
        if (ret < 0) {
            fprintf(stderr, "feed: i265e_encode %d: %d\n", f, ret);
            return 1;
        }
        ret = i265e_get_bitstream(enc, &nal, &nnal, &a, &b, &c, &task);
        if (ret < 0) {
            fprintf(stderr, "feed: i265e_get_bitstream %d: %d\n", f, ret);
            return 1;
        }
        for (i = 0; i < nnal; i++) {
            uint32_t *n = (uint32_t *)nal + 3 * i;

            if (getenv("FEED_DEBUG")) {
                fprintf(stderr, "feed: frame %d nal %d: %08x %08x %08x\n",
                        f, i, n[0], n[1], n[2]);
            }
            /* i265e_nal_t: type, size, payload */
            fwrite((void *)n[2], 1, n[1], fo);
            total += n[1];
        }
        i265e_release_bitstream(enc, nal, task);
    }
    fclose(fo);
    printf("feed: %d frames, %d bytes -> %s\n", frames, total, out);
    return 0;
}

/*
 * JPEG: the encoder's parameters (ijpege_param_default; T23 offsets, the
 * older libraries' 4 bytes in front up to +16 and 8 after, T30's 4 again
 * from the allocator on) +4 soc,
 * +8 0, +12/+16/+20 1, +24 -1, +28/+32 size, +36 i_csp, +52.. the
 * allocator; the picture +8 i_csp, +16/+20 strides, +32/+36 lines,
 * +64/+68 planes, +120 where the output buffer's address is, +124 its
 * size: the buffer gets the whole JFIF.
 */
static int feed_jpeg(const char *out, int frames, int w, int h, int lay,
                     int csp, uint8_t *y, uint8_t *uv)
{
    static uint8_t param[256], pic[512];
    const uint32_t cap = w * h;
    uint8_t *buf = feed_malloc(0, cap, 64);
    static void *bufp;
    void *enc;
    FILE *fo;
    int f, total = 0, ws = (w + 15) & ~15, hs = (h + 15) & ~15;
#define JO(o)   ((o) - (lay == 0 ? 0 : (o) <= 16 ? 4 : \
                         (lay == 3 && (o) >= 52) ? 4 : 8))

    ijpege_param_default(param);
    P32(param, 0) = 0;
    P32(param, JO(4)) = getenv("FEED_SOC") ? atoi(getenv("FEED_SOC")) :
                        lay == 0 ? 5 : lay == 1 ? 4 : lay == 2 ? 2 : 3;
    P32(param, JO(8)) = 0;
    P32(param, JO(12)) = 1;
    if (lay == 0) {
        P32(param, 20) = 1;
    }
    P32(param, JO(28)) = w;
    P32(param, JO(32)) = h;
    P32(param, JO(36)) = csp;
    PP(param, JO(52)) = (void *)feed_malloc;
    PP(param, JO(56)) = (void *)feed_free;
    PP(param, JO(60)) = (void *)feed_v2p;
    PP(param, JO(64)) = (void *)feed_p2v;
    bufp = buf;
    enc = ijpege_init(param);
    if (!enc) {
        fprintf(stderr, "feed: ijpege_init failed\n");
        return 1;
    }
    fo = fopen(out, "wb");
    for (f = 0; f < frames; f++) {
        void *nal = NULL;
        int nnal = 0, ret, x = 0;

        draw(y, uv, ws, h, f);
        memset(pic, 0, sizeof(pic));
        P32(pic, 0) = f;
        P32(pic, 8) = csp;
        P32(pic, 12) = 2;
        P32(pic, 16) = ws;
        P32(pic, 20) = ws;
        P32(pic, 32) = hs;
        P32(pic, 36) = hs / 2;
        PP(pic, 64) = y;
        PP(pic, 68) = uv;
        PP(pic, 120) = &bufp;               /* where the buffer's address is */
        P32(pic, 124) = cap;
        ret = ijpege_encode(enc, &nal, &nnal, pic, &x);
        if (ret < 0 || !nal) {
            fprintf(stderr, "feed: ijpege_encode %d: %d\n", f, ret);
            return 1;
        }
        if (getenv("FEED_DEBUG")) {
            fprintf(stderr, "feed: jpeg %d: nnal %d %08x %08x %08x\n", f,
                    nnal, P32(nal, 0), P32(nal, 4), P32(nal, 8));
        }
        fwrite(PP(nal, 4), 1, P32(nal, 0), fo);
        total += P32(nal, 0);
    }
    fclose(fo);
    printf("feed: %d JPEGs, %d bytes -> %s\n", frames, total, out);
    return 0;
}

int main(int argc, char **argv)
{
    const char *out = argc > 1 ? argv[1] : "/tmp/feed.h264";
    int frames = argc > 2 ? atoi(argv[2]) : 10;
    int w = 640, h = 360, qp = argc > 4 ? atoi(argv[4]) : 30;
    static uint8_t param[1024], pic[1024];
    uint8_t *y, *uv;
    void *enc;
    FILE *fo;
    int fd, f, total = 0;
    /* the library's parameter layout: 0 T23, 1 T21, 2 T20, 3 T30 */
    const char *ls = getenv("FEED_LIB") ? getenv("FEED_LIB") : "t23";
    int lay = !strcmp(ls, "t21") ? 1 : !strcmp(ls, "t20") ? 2 :
              !strcmp(ls, "t30") ? 3 : 0;
    const int csp = lay == 2 ? 1 : 4;   /* the library's number for NV12 */

    if (argc > 3) {
        sscanf(argv[3], "%dx%d", &w, &h);
    }
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("/dev/mem");
        return 1;
    }
    mem = musl_mmap(NULL, PHYS_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
               PHYS_BASE);
    if (mem == MAP_FAILED) {
        perror("mmap");
        return 1;
    }
    y = feed_malloc(0, ((w + 15) & ~15) * ((h + 15) & ~15) * 3 / 2, 256);
    uv = y + ((w + 15) & ~15) * ((h + 15) & ~15);

    feed_catch(4);
    feed_catch(11);
    feed_catch(10);
    if (getenv("FEED_DEBUG")) {
        char buf[256];
        FILE *m = fopen("/proc/self/maps", "r");

        while (m && fgets(buf, sizeof(buf), m)) {
            fputs(buf, stderr);
        }
        if (m) {
            fclose(m);
        }
    }
    {
        uint32_t n = 0x258000, o = lay == 2 ? 96 : 128;
        uint8_t *bs = feed_malloc(0, n, 4096);

        *(void **)(alloc_vpu_bs + o) = bs;
        *(uint32_t *)(alloc_vpu_bs + o + 4) = feed_v2p(bs);
        *(uint32_t *)(alloc_vpu_bs + o + 8) = n;
        *(uint32_t *)(alloc_vpu_bs + o + 12) = n;
        sem_init(&bsbufsem, 0, 1);
    }
    if (getenv("FEED_JPEG")) {
        return feed_jpeg(out, frames, w, h, lay, csp, y, uv);
    }
    if (getenv("FEED_H265")) {
        return feed_h265(out, frames, w, h, qp, y, uv);
    }
    i264e_param_default(param);
    /*
     * The T23 library's offsets (o), one word further in than T21's; the
     * T20/T30 library has the fields from the size on 12 bytes in front
     * of T23's (i264e_param2string in each names them).
     */
#define PO(o)   ((o) - (lay == 0 ? 0 : ((o) <= 40 || lay == 1) ? 4 : 12))
    if (lay == 0) {
        P32(param, 0) = 0;
    }
    P32(param, PO(4)) = getenv("FEED_SOC") ? atoi(getenv("FEED_SOC")) :
                        lay == 0 ? 5 : lay == 1 ? 4 : lay == 2 ? 2 : 3;
    P32(param, PO(8)) = 0;              /* b_tlb: physical addresses */
    P32(param, PO(12)) = 1;
    P32(param, PO(20)) = 0;             /* the encoder allocates its stream */
    P32(param, PO(24)) = 0;
    P32(param, PO(36)) = -1;            /* i_devid */
    P32(param, PO(40)) = 1;             /* one thread: synchronous */
    P32(param, PO(56)) = w;
    P32(param, PO(60)) = h;
    P32(param, PO(64)) = csp;           /* NV12 */
    P32(param, PO(68)) = 100;           /* High */
    if (getenv("FEED_CBR")) {           /* CBR at FEED_CBR kbit/s */
        P32(param, PO(200)) = 1;
        P32(param, PO(208)) = 15;       /* min QP */
        P32(param, PO(212)) = 45;       /* max QP */
        P32(param, PO(228)) = atoi(getenv("FEED_CBR"));
        P32(param, PO(244)) = 3;        /* frame QP step */
        P32(param, PO(248)) = 15;       /* GOP QP step */
    } else {
        P32(param, PO(200)) = 0;        /* fixed QP */
        P32(param, PO(204)) = qp;
    }
    P32(param, PO(408)) = 25;
    P32(param, PO(412)) = 1;
    PP(param, PO(440)) = (void *)feed_malloc;
    PP(param, PO(444)) = (void *)feed_free;
    PP(param, PO(448)) = (void *)feed_v2p;
    PP(param, PO(452)) = (void *)feed_p2v;
    enc = i264e_init(param);
    if (!enc) {
        fprintf(stderr, "feed: i264e_init failed\n");
        return 1;
    }
    fo = fopen(out, "wb");
    for (f = 0; f < frames; f++) {
        void *nal = NULL, *po = NULL, *pa = NULL, *pb = NULL, *fr = NULL;
        int nnal = 0, i, ret;

        draw(y, uv, (w + 15) & ~15, h, f);
        memset(pic, 0, sizeof(pic));
        P32(pic, 8) = f == 0;
        P32(pic, 16) = f;
        P32(pic, 32) = f * 40000;
        P32(pic, 40) = csp;
        P32(pic, 44) = 2;
        P32(pic, 48) = (w + 15) & ~15;
        P32(pic, 52) = (w + 15) & ~15;
        P32(pic, 64) = (h + 15) & ~15;
        P32(pic, 68) = ((h + 15) & ~15) / 2;
        PP(pic, 96) = y;
        PP(pic, 100) = uv;
        ret = i264e_encode(enc, pic);
        if (ret < 0) {
            fprintf(stderr, "feed: i264e_encode %d: %d\n", f, ret);
            return 1;
        }
        ret = i264e_get_bitstream(enc, &nal, &nnal, &po, &pa, &pb, &fr);
        if (ret < 0) {
            fprintf(stderr, "feed: i264e_get_bitstream %d: %d\n", f, ret);
            return 1;
        }
        for (i = 0; i < nnal; i++) {
            uint8_t *n = (uint8_t *)nal + 24 * i;
            int len = P32(n, 12);
            uint8_t *p = PP(n, 16);

            if (getenv("FEED_DEBUG")) {
                fprintf(stderr, "feed: frame %d nal %d: %08x %08x %08x "
                        "len %d @%p %08x\n", f, i, P32(n, 0), P32(n, 4),
                        P32(n, 8), len, p, P32(n, 20));
            }
            fwrite(p, 1, len, fo);
            total += len;
        }
        i264e_release_bitstream(enc, nal, fr);
    }
    fclose(fo);
    printf("feed: %d frames, %d bytes -> %s\n", frames, total, out);
    return 0;
}
