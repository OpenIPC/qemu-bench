/*
 * T40's BSCALER under Ingenic's own BSCALER code from libvenus
 * (magik-toolkit InferenceKit/nna1: bscaler_api, bscaler_hal,
 * bscaler_segmentation, matrix): resizes (BGRA, NV12, NV12 to BGRA,
 * a multi-tile upscale), affine and perspective warps on the frmc unit,
 * and NV12 to BGRA / RGBA on the frmt unit, on buffers in reserved
 * memory (mapped uncached through /dev/mem), each result checked against
 * what the operation means -- linear ramps sampled at the half-pixel
 * source positions, BT.601 colours, a square moved by the forward
 * matrix.  The library drives the vendor kernel driver (/dev/bscaler0)
 * and the registers at 0x13090000.
 *
 *   bscaler-test PHYS    (4 MiB of reserved memory at PHYS)
 *
 * This file stands in for the rest of libvenus (its __aie_mmap, DDR
 * allocator and cache maintenance, which need the NNA runtime): the
 * library's "DDR" is the reserved memory, mapped uncached.
 *
 * The bs_* structures, read from the library: an image is { y, uv,
 * format, channels, w, h, stride, - } (addresses virtual, in "DDR";
 * format 0 NV12, 1 BGRA, 11 RGBA, 27 ARGB); a resize box { x, y, w, h,
 * -, - }; a warp box { x, y, w, h, the forward 3x3 matrix as floats, -,
 * value outside the source }; a covert step { 0x807060, lines, dst
 * bytes }.
 */
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

struct img { void *y, *uv; uint32_t fmt, ch, w, h, stride, rsv; };
struct rbox { uint32_t x, y, w, h, rsv[2]; };
struct wbox { uint32_t x, y, w, h; float m[9]; uint32_t rsv, pad; };

int bs_resize_start(struct img *, int, struct img *, struct rbox *,
                    uint32_t *, uint32_t *);
int bs_resize_wait(void);
int bs_affine_start(struct img *, int, struct img *, struct wbox *,
                    uint32_t *, uint32_t *);
int bs_affine_wait(void);
int bs_perspective_start(struct img *, int, struct img *, struct wbox *,
                         uint32_t *, uint32_t *);
int bs_perspective_wait(void);
int bs_covert_cfg(struct img *, struct img *, uint32_t *, uint32_t *,
                  uint32_t *);
int bs_covert_step_start(uint32_t *, void *, int);
int bs_covert_step_wait(void);

#define POOL    (4 << 20)

static uint8_t *v;
static unsigned phys, pool_top = 3 << 20;
static int fails;

/* --- what the rest of libvenus provides ------------------------------- */

void *__ddr_vbase, *__ddr_pbase, *__oram_vbase, *__bscaler_io_vbase;
int __ddr_vbase_cache_attr;

void *ddr_memalign(unsigned align, unsigned size)
{
    pool_top = (pool_top + align - 1) & ~(align - 1);
    if (pool_top + size > POOL) {
        return NULL;
    }
    pool_top += size;
    return v + pool_top - size;
}

void ddr_free(void *p) { }
void *oram_memalign(unsigned align, unsigned size) { return NULL; }
void oram_free(void *p) { }
void __aie_flushcache(void *p, unsigned n) { }
void __aie_flushcache_dir(void *p, unsigned n, int dir) { }
int __aie_mmap(void) { return 0; }      /* bscaler_init's; not called */
/* libvenus's timing helper, AutoTime::AutoTime(int, const char *) / dtor */
void autotime_ctor(void *t, int l, const char *n) __asm__("_ZN8AutoTimeC1EiPKc");
void autotime_ctor(void *t, int l, const char *n) { }
void autotime_dtor(void *t) __asm__("_ZN8AutoTimeD1Ev");
void autotime_dtor(void *t) { }

/* ------------------------------------------------------------------------ */

static void __attribute__((format(printf, 3, 4)))
check(const char *what, int ok, const char *fmt, ...)
{
    va_list ap;

    printf("BSCTEST %s: %s (", ok ? "PASS" : "FAIL", what);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf(")\n");
    fails += !ok;
}

static uint32_t coef[9] = {          /* libvenus's default: rows B, G, R */
    1220542, 2116026, 0, 1220542, 409993, 852492, 1220542, 0, 1673527,
};
static uint32_t offs[2] = { 16, 128 };

/* the linear ramps of the sources: luma, the two chroma, and BGRA */
static double fy(double x, double y) { return 16 + x * 0.25 + y * 0.1; }
static double fu(double x, double y) { return 60 + x * 0.25; }
static double fv(double x, double y) { return 200 - y * 0.4; }
static double fc(int c, double x, double y)
{
    return 10 + c * 30 + x * 0.5 + y * 0.3;
}

#define SRC 0x000000
#define DST1 0x100000
#define DST2 0x200000

static void make_nv12(int w, int h)
{
    int x, y;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            v[SRC + y * w + x] = lround(fy(x, y));
        }
    }
    for (y = 0; y < h / 2; y++) {
        for (x = 0; x < w / 2; x++) {
            /* chroma sites at luma (2x + 0.5, 2y + 0.5) */
            v[SRC + w * h + y * w + 2 * x] = lround(fu(2 * x + .5, 2 * y + .5));
            v[SRC + w * h + y * w + 2 * x + 1] =
                lround(fv(2 * x + .5, 2 * y + .5));
        }
    }
}

static void make_bgra(int w, int h)
{
    int x, y, c;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            for (c = 0; c < 4; c++) {
                v[SRC + (y * w + x) * 4 + c] = lround(fc(c, x, y));
            }
        }
    }
}

static void set_img(struct img *i, unsigned off, int fmt, int w, int h)
{
    memset(i, 0, sizeof(*i));
    i->y = v + off;
    i->uv = fmt ? NULL : v + off + w * h;
    i->fmt = fmt;
    i->ch = fmt ? 4 : 1;
    i->w = w;
    i->h = h;
    i->stride = fmt ? w * 4 : w;
}

/* the source position of destination pixel @d at scale @s (half-pixel) */
static double spos(int d, double s, double lim)
{
    double p = (d + .5) * s - .5;

    return fmin(fmax(p, 0), lim - 1);
}

static void test_resize_bgra(int sw, int sh, int dw, int dh, const char *name)
{
    struct img src, dst;
    struct rbox box = { 0, 0, sw, sh, { 0, 0 } };
    double e = 0, sx = (double)sw / dw, sy = (double)sh / dh;
    int x, y, c;

    make_bgra(sw, sh);
    set_img(&src, SRC, 1, sw, sh);
    set_img(&dst, DST1, 1, dw, dh);
    memset(v + DST1, 0xee, dw * dh * 4);
    bs_resize_start(&src, 1, &dst, &box, coef, offs);
    bs_resize_wait();
    for (y = 0; y < dh; y++) {
        for (x = 0; x < dw; x++) {
            for (c = 0; c < 4; c++) {
                double d = v[DST1 + (y * dw + x) * 4 + c] -
                           fc(c, spos(x, sx, sw), spos(y, sy, sh));

                e = fmax(e, fabs(d));
            }
        }
    }
    check(name, e <= 2, "worst error %.2f", e);
}

static void test_resize_nv12(void)
{
    struct img src, dst;
    struct rbox box = { 0, 0, 256, 128, { 0, 0 } };
    double e = 0, d;
    int x, y;

    make_nv12(256, 128);
    set_img(&src, SRC, 0, 256, 128);
    set_img(&dst, DST1, 0, 128, 64);
    memset(v + DST1, 0, 128 * 96);
    bs_resize_start(&src, 1, &dst, &box, coef, offs);
    bs_resize_wait();
    for (y = 0; y < 64; y++) {
        for (x = 0; x < 128; x++) {
            d = v[DST1 + y * 128 + x] - fy(spos(x, 2, 256), spos(y, 2, 128));
            e = fmax(e, fabs(d));
        }
    }
    for (y = 0; y < 32; y++) {
        for (x = 0; x < 64; x++) {
            double cx = spos(2 * x, 2, 256) + 1, cy = spos(2 * y, 2, 128) + 1;

            d = v[DST1 + 128 * 64 + y * 128 + 2 * x] - fu(cx, cy);
            e = fmax(e, fabs(d));
            d = v[DST1 + 128 * 64 + y * 128 + 2 * x + 1] - fv(cx, cy);
            e = fmax(e, fabs(d));
        }
    }
    check("C: NV12 256x128 -> 128x64 follows the ramps (Y and CbCr)",
          e <= 2, "worst error %.2f", e);
}

static void bt601(int y, int u, int v_, int rgb[3])
{
    double yy = 1.164 * (y - 16);

    rgb[0] = lround(fmin(255, fmax(0, yy + 1.596 * (v_ - 128))));
    rgb[1] = lround(fmin(255, fmax(0, yy - 0.391 * (u - 128) -
                                   0.813 * (v_ - 128))));
    rgb[2] = lround(fmin(255, fmax(0, yy + 2.018 * (u - 128))));
}

/* 64x32 NV12 of four colour blocks */
static const uint8_t yuv[4][3] = {
    { 81, 90, 240 }, { 145, 54, 34 }, { 41, 240, 110 }, { 180, 128, 128 },
};

static void make_blocks(void)
{
    int x, y;

    for (y = 0; y < 32; y++) {
        for (x = 0; x < 64; x++) {
            int b = x / 16;

            v[SRC + y * 64 + x] = yuv[b][0];
            if (!(y & 1) && !(x & 1)) {
                v[SRC + 64 * 32 + (y / 2) * 64 + x] = yuv[b][1];
                v[SRC + 64 * 32 + (y / 2) * 64 + x + 1] = yuv[b][2];
            }
        }
    }
}

/* the worst colour error of a 4-byte picture of the blocks, inner pixels */
static int blocks_err(unsigned off, int w, int h, int scale, const int pos[3])
{
    int x, y, c, rgb[3], worst = 0;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint8_t *p = v + off + (y * w + x) * 4;
            int sx = x * scale, b = sx / 16;

            if (sx % 16 < 2 || sx % 16 > 13) {
                continue;               /* a block edge: mixed colours */
            }
            bt601(yuv[b][0], yuv[b][1], yuv[b][2], rgb);
            for (c = 0; c < 3; c++) {
                worst = abs(p[pos[c]] - rgb[c]) > worst ?
                        abs(p[pos[c]] - rgb[c]) : worst;
            }
        }
    }
    return worst;
}

static const struct { int fmt; const char *name; int pos[3]; } ord[] = {
    { 1, "BGRA", { 2, 1, 0 } },     /* byte positions of R, G, B */
    { 11, "RGBA", { 0, 1, 2 } },
    { 27, "ARGB", { 1, 2, 3 } },
};

static void test_resize_cvt(void)
{
    struct img src, dst;
    struct rbox box = { 0, 0, 64, 32, { 0, 0 } };
    char what[80];
    int i, worst;

    make_blocks();
    for (i = 0; i < 3; i++) {
        set_img(&src, SRC, 0, 64, 32);
        set_img(&dst, DST1, ord[i].fmt, 32, 16);
        memset(v + DST1, 0xee, 32 * 16 * 4);
        bs_resize_start(&src, 1, &dst, &box, coef, offs);
        bs_resize_wait();
        worst = blocks_err(DST1, 32, 16, 2, ord[i].pos);
        snprintf(what, sizeof(what), "C: NV12 64x32 -> %s 32x16, BT.601",
                 ord[i].name);
        check(what, worst <= 3, "worst error %d, first pixel %02x %02x %02x "
              "%02x", worst, v[DST1], v[DST1 + 1], v[DST1 + 2], v[DST1 + 3]);
    }
}

static void test_covert(void)
{
    struct img src, dst;
    uint32_t step[3];
    char what[80];
    int i, worst;

    make_blocks();
    for (i = 0; i < 2; i++) {
        set_img(&src, SRC, 0, 64, 32);
        set_img(&dst, DST1, ord[i].fmt, 64, 32);
        memset(v + DST1, 0xee, 64 * 32 * 4);
        step[0] = 0x807060;
        step[1] = 32;
        step[2] = 64 * 4 * 32;
        bs_covert_cfg(&src, &dst, coef, offs, step);
        bs_covert_step_start(step, v + DST1, 0);
        bs_covert_step_wait();
        worst = blocks_err(DST1, 64, 32, 1, ord[i].pos);
        snprintf(what, sizeof(what), "T: NV12 -> %s by lines, BT.601",
                 ord[i].name);
        check(what, worst <= 2, "worst error %d, first pixel %02x %02x %02x "
              "%02x", worst, v[DST1], v[DST1 + 1], v[DST1 + 2], v[DST1 + 3]);
    }
}

/* the inverse of a 3x3 matrix */
static void inverse3(const double m[9], double r[9])
{
    double d = m[0] * (m[4] * m[8] - m[5] * m[7]) -
               m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);

    r[0] = (m[4] * m[8] - m[5] * m[7]) / d;
    r[1] = (m[2] * m[7] - m[1] * m[8]) / d;
    r[2] = (m[1] * m[5] - m[2] * m[4]) / d;
    r[3] = (m[5] * m[6] - m[3] * m[8]) / d;
    r[4] = (m[0] * m[8] - m[2] * m[6]) / d;
    r[5] = (m[2] * m[3] - m[0] * m[5]) / d;
    r[6] = (m[3] * m[7] - m[4] * m[6]) / d;
    r[7] = (m[1] * m[6] - m[0] * m[7]) / d;
    r[8] = (m[0] * m[4] - m[1] * m[3]) / d;
}

static void test_warp(int persp)
{
    /* forward: dst = 0.5 src + (10, 5), with a projective term for one */
    double fwd[9] = { .5, 0, 10, 0, .5, 5, persp ? .002 : 0, 0, 1 }, inv[9];
    struct img src, dst;
    struct wbox box;
    int x, y, i, inside = 0, outside = 0, skipped = 0, bad = 0;
    const char *name = persp ? "C: perspective (a projective matrix)" :
                               "C: affine";

    /* BGRA 128x128, black with a white square at (40..59, 30..49) */
    memset(v + SRC, 0, 128 * 128 * 4);
    for (y = 30; y < 50; y++) {
        for (x = 40; x < 60; x++) {
            memset(v + SRC + (y * 128 + x) * 4, 0xff, 4);
        }
    }
    set_img(&src, SRC, 1, 128, 128);
    set_img(&dst, DST1, 1, 96, 96);
    memset(&box, 0, sizeof(box));
    box.w = 128;
    box.h = 128;
    for (i = 0; i < 9; i++) {
        box.m[i] = fwd[i];
    }
    box.pad = 0x40;
    inverse3(fwd, inv);
    memset(v + DST1, 0xee, 96 * 96 * 4);
    if (persp) {
        bs_perspective_start(&src, 1, &dst, &box, coef, offs);
        bs_perspective_wait();
    } else {
        bs_affine_start(&src, 1, &dst, &box, coef, offs);
        bs_affine_wait();
    }
    for (y = 0; y < 96; y++) {
        for (x = 0; x < 96; x++) {
            uint8_t g = v[DST1 + (y * 96 + x) * 4 + 1];
            /* the source position of this pixel, by the inverse */
            double w = inv[6] * x + inv[7] * y + inv[8];
            double sx = (inv[0] * x + inv[1] * y + inv[2]) / w;
            double sy = (inv[3] * x + inv[4] * y + inv[5]) / w;

            if (sx >= 42 && sx <= 57 && sy >= 32 && sy <= 47) {
                inside++;
                bad += g < 0xf0;
            } else if (sx < -2 || sy < -2 || sx > 129 || sy > 129) {
                /*
                 * the pad value -- or nothing: the library emits no job
                 * for a tile that maps wholly outside the source
                 */
                outside += g == 0x40;
                skipped += g == 0xee;
                bad += g != 0x40 && g != 0xee;
            } else if (sx > 0 && sy > 0 && sx < 127 && sy < 127 &&
                       (sx < 37 || sx > 62 || sy < 27 || sy > 52)) {
                bad += g > 0x10;
            }
        }
    }
    check(name, !bad && inside > 30 && outside > 100,
          "%d pixels in the moved square, %d outside the source at the pad "
          "value, %d in tiles not run, %d wrong", inside, outside, skipped,
          bad);
}

int main(int argc, char **argv)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);

    phys = argc > 1 ? strtoul(argv[1], NULL, 0) : 0;
    if (!phys || fd < 0) {
        printf("BSCTEST FAIL: usage / /dev/mem (%d)\n", fd);
        return 1;
    }
    v = mmap(NULL, POOL, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys);
    __bscaler_io_vbase = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED,
                              fd, 0x13090000);
    if (v == MAP_FAILED || __bscaler_io_vbase == MAP_FAILED) {
        printf("BSCTEST FAIL: mmap 0x%x / the registers\n", phys);
        return 1;
    }
    if (access("/dev/bscaler0", F_OK)) {
        printf("BSCTEST FAIL: no /dev/bscaler0 (the kernel driver)\n");
        return 1;
    }
    __ddr_vbase = v;
    __ddr_pbase = (void *)(uintptr_t)phys;
    test_resize_bgra(64, 32, 32, 16, "C: BGRA 64x32 -> 32x16 follows the ramps");
    test_resize_bgra(32, 16, 160, 80,
                     "C: BGRA 32x16 -> 160x80, many tiles, follows the ramps");
    test_resize_nv12();
    test_resize_cvt();
    test_warp(0);
    test_warp(1);
    test_covert();
    printf("BSCTEST DONE %d failure(s)\n", fails);
    return fails != 0;
}
