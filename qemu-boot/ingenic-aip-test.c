/*
 * T41's AIP under Ingenic's own libaip (magik-toolkit InferenceKit/nna2):
 * resize (unit F, one box and a two-box chain), NV12 to BGRA / RGBA /
 * ARGB (unit T) and affine / perspective warps (unit P), on buffers in
 * reserved memory (mapped uncached through /dev/mem), each result checked
 * against what the operation means -- a linear ramp sampled at the
 * half-pixel source positions, BT.601 colours, a square moved by the
 * forward matrix.  The library drives the vendor kernel driver
 * (/dev/jzaip_t, _f, _p) and the registers at 0x13090000.
 *
 *   aip-test PHYS        (4 MiB of reserved memory at PHYS)
 *
 * The bs_* structures, read from the library: an image is { y, uv,
 * format, -, w, h, stride, order } words (format 0 NV12, 1 BGRA, 11
 * RGBA, 27 ARGB); a resize box { x, y, w, h, -, - }; a warp box { x, y,
 * w, h, the forward 3x3 matrix as floats, -, value outside the box }.
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

struct img { uint32_t y, uv, fmt, rsv, w, h, stride, order; };
struct rbox { uint32_t x, y, w, h, rsv[2]; };
struct wbox { uint32_t x, y, w, h; float m[9]; uint32_t rsv, pad; };

int bs_resize_start(struct img *, int, struct img *, struct rbox *);
int bs_resize_wait(void);
int bs_covert_cfg(struct img *, struct img *, uint32_t *, uint32_t *);
int bs_covert_step_wait(void);
int bs_covert_step_exit(void);
int bs_affine_start(struct img *, int, struct img *, struct wbox *,
                    uint32_t *, uint32_t *);
int bs_affine_wait(void);
int bs_perspective_start(struct img *, int, struct img *, struct wbox *,
                         uint32_t *, uint32_t *);
int bs_perspective_wait(void);

static uint8_t *v;
static unsigned phys;
static int fails;

#define P(off)  (phys + (off))
#define V(off)  (v + (off))

static void __attribute__((format(printf, 3, 4)))
check(const char *what, int ok, const char *fmt, ...)
{
    va_list ap;

    printf("AIPTEST %s: %s (", ok ? "PASS" : "FAIL", what);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf(")\n");
    fails += !ok;
}

static uint32_t coef[9] = {          /* BT.601, Q20 (libvenus's default) */
    1220542, 0, 1673527, 1220542, 409993, 852492, 1220542, 2116026, 0,
};
static uint32_t offs[2] = { 16, 128 };

/* the linear ramps of the NV12 source: luma and the two chroma planes */
static double fy(double x, double y) { return 16 + x * 0.25 + y * 0.1; }
static double fu(double x, double y) { return 60 + x * 0.25; }
static double fv(double x, double y) { return 200 - y * 0.4; }

#define SW 640
#define SH 360
#define SRC 0x000000
#define DST1 0x100000
#define DST2 0x140000

static void make_nv12(void)
{
    int x, y;

    for (y = 0; y < SH; y++) {
        for (x = 0; x < SW; x++) {
            V(SRC)[y * SW + x] = lround(fy(x, y));
        }
    }
    for (y = 0; y < SH / 2; y++) {
        for (x = 0; x < SW / 2; x++) {
            /* chroma sites at luma (2x + 0.5, 2y + 0.5) */
            V(SRC + SW * SH)[y * SW + 2 * x] = lround(fu(2 * x + .5, 2 * y + .5));
            V(SRC + SW * SH)[y * SW + 2 * x + 1] =
                lround(fv(2 * x + .5, 2 * y + .5));
        }
    }
}

static void nv12_img(struct img *i, unsigned off, int w, int h, int stride)
{
    memset(i, 0, sizeof(*i));
    i->y = P(off);
    i->uv = P(off) + stride * h;
    i->fmt = 0;
    i->w = w;
    i->h = h;
    i->stride = stride;
}

/* the worst error of a scaled NV12 picture against the ramps */
static double resize_err(unsigned off, int dw, int dh, double bx, double by,
                         double s)
{
    double e = 0, d;
    int x, y;

    for (y = 0; y < dh; y++) {
        for (x = 0; x < dw; x++) {
            d = V(off)[y * dw + x] - fy(bx + (x + .5) * s - .5,
                                        by + (y + .5) * s - .5);
            e = fmax(e, fabs(d));
        }
    }
    for (y = 0; y < dh / 2; y++) {
        for (x = 0; x < dw / 2; x++) {
            double sx = bx + (2 * x + 1) * s - .5, sy = by + (2 * y + 1) * s - .5;

            d = V(off + dw * dh)[y * dw + 2 * x] - fu(sx, sy);
            e = fmax(e, fabs(d));
            d = V(off + dw * dh)[y * dw + 2 * x + 1] - fv(sx, sy);
            e = fmax(e, fabs(d));
        }
    }
    return e;
}

static void test_resize(void)
{
    struct img src, dst[2];
    struct rbox box[2];
    double e1, e2;

    nv12_img(&src, SRC, SW, SH, SW);
    nv12_img(&dst[0], DST1, 320, 180, 320);
    memset(box, 0, sizeof(box));
    box[0].w = SW;
    box[0].h = SH;
    memset(V(DST1), 0, 320 * 270);
    bs_resize_start(&src, 1, dst, box);
    bs_resize_wait();
    e1 = resize_err(DST1, 320, 180, 0, 0, 2);
    check("F: NV12 640x360 -> 320x180 follows the ramps", e1 <= 2,
          "worst error %.2f", e1);

    /* two boxes: a chain of two nodes */
    nv12_img(&dst[1], DST2, 128, 64, 128);
    box[1].x = 64;
    box[1].y = 32;
    box[1].w = 256;
    box[1].h = 128;
    memset(V(DST1), 0, 320 * 270);
    memset(V(DST2), 0, 128 * 96);
    bs_resize_start(&src, 2, dst, box);
    bs_resize_wait();
    e1 = resize_err(DST1, 320, 180, 0, 0, 2);
    e2 = resize_err(DST2, 128, 64, 64, 32, 2);
    check("F: a two-box chain, both outputs", e1 <= 2 && e2 <= 2,
          "worst errors %.2f, %.2f", e1, e2);
}

static void bt601(int y, int u, int v, int rgb[3])
{
    double yy = 1.164 * (y - 16);

    rgb[0] = lround(fmin(255, fmax(0, yy + 1.596 * (v - 128))));
    rgb[1] = lround(fmin(255, fmax(0, yy - 0.391 * (u - 128) -
                                   0.813 * (v - 128))));
    rgb[2] = lround(fmin(255, fmax(0, yy + 2.018 * (u - 128))));
}

static void test_covert(void)
{
    static const struct { int fmt; const char *name; int pos[4]; } ord[] = {
        /* byte positions of R, G, B, A */
        { 1, "BGRA", { 2, 1, 0, 3 } },
        { 11, "RGBA", { 0, 1, 2, 3 } },
        { 27, "ARGB", { 1, 2, 3, 0 } },
    };
    struct img src, dst;
    int i, x, y, rgb[3], worst;

    /* 64x32: four colour blocks */
    static const uint8_t yuv[4][3] = {
        { 81, 90, 240 }, { 145, 54, 34 }, { 41, 240, 110 }, { 180, 128, 128 },
    };
    for (y = 0; y < 32; y++) {
        for (x = 0; x < 64; x++) {
            int b = x / 16;

            V(SRC)[y * 64 + x] = yuv[b][0];
            if (!(y & 1) && !(x & 1)) {
                V(SRC + 64 * 32)[(y / 2) * 64 + x] = yuv[b][1];
                V(SRC + 64 * 32)[(y / 2) * 64 + x + 1] = yuv[b][2];
            }
        }
    }
    for (i = 0; i < 3; i++) {
        nv12_img(&src, SRC, 64, 32, 64);
        memset(&dst, 0, sizeof(dst));
        dst.y = P(DST1);
        dst.fmt = ord[i].fmt;
        dst.w = 64;
        dst.h = 32;
        dst.stride = 64 * 4;
        memset(V(DST1), 0xee, 64 * 32 * 4);
        bs_covert_cfg(&src, &dst, coef, offs);
        bs_covert_step_wait();
        bs_covert_step_exit();
        worst = 0;
        for (y = 0; y < 32; y++) {
            for (x = 0; x < 64; x++) {
                uint8_t *p = V(DST1) + (y * 64 + x) * 4;
                int b = x / 16, c;

                bt601(yuv[b][0], yuv[b][1], yuv[b][2], rgb);
                for (c = 0; c < 3; c++) {
                    worst = abs(p[ord[i].pos[c]] - rgb[c]) > worst ?
                            abs(p[ord[i].pos[c]] - rgb[c]) : worst;
                }
            }
        }
        {
            char what[64];

            snprintf(what, sizeof(what), "T: NV12 -> %s, BT.601", ord[i].name);
            check(what, worst <= 2, "worst error %d, first pixel %02x %02x "
                  "%02x %02x", worst, V(DST1)[0], V(DST1)[1], V(DST1)[2],
                  V(DST1)[3]);
        }
    }
}

static void test_warp(int persp)
{
    struct img src, dst;
    struct wbox box;
    int x, y, inside = 0, outside = 0, bad = 0;
    const char *name = persp ? "P: perspective" : "P: affine";

    /* BGRA 128x128, black with a white square at (40..59, 30..49) */
    memset(V(SRC), 0, 128 * 128 * 4);
    for (y = 30; y < 50; y++) {
        for (x = 40; x < 60; x++) {
            memset(V(SRC) + (y * 128 + x) * 4, 0xff, 4);
        }
    }
    memset(&src, 0, sizeof(src));
    src.y = P(SRC);
    src.fmt = 1;
    src.w = 128;
    src.h = 128;
    src.stride = 128 * 4;
    memset(&dst, 0, sizeof(dst));
    dst.y = P(DST1);
    dst.fmt = 1;
    dst.w = 96;
    dst.h = 96;
    dst.stride = 96 * 4;
    memset(&box, 0, sizeof(box));
    box.w = 128;
    box.h = 128;
    /* forward: dst = 0.5 src + (10, 5); the square lands at (30..40, 20..30) */
    box.m[0] = .5f; box.m[1] = 0; box.m[2] = 10;
    box.m[3] = 0; box.m[4] = .5f; box.m[5] = 5;
    box.m[6] = 0; box.m[7] = 0; box.m[8] = 1;
    box.pad = 0x40;
    memset(V(DST1), 0xee, 96 * 96 * 4);
    if (persp) {
        bs_perspective_start(&src, 1, &dst, &box, coef, offs);
        bs_perspective_wait();
    } else {
        bs_affine_start(&src, 1, &dst, &box, coef, offs);
        bs_affine_wait();
    }
    for (y = 0; y < 96; y++) {
        for (x = 0; x < 96; x++) {
            uint8_t g = V(DST1)[(y * 96 + x) * 4 + 1];
            /* the source position of this pixel */
            double sx = (x - 10) * 2.0, sy = (y - 5) * 2.0;

            if (sx >= 42 && sx <= 57 && sy >= 32 && sy <= 47) {
                inside++;
                bad += g < 0xf0;
            } else if (sx < 0 || sy < 0 || sx > 127 || sy > 127) {
                outside++;
                bad += g != 0x40;
            } else if ((sx < 37 || sx > 62 || sy < 27 || sy > 52)) {
                bad += g > 0x10;
            }
        }
    }
    check(name, !bad && inside > 30 && outside > 100,
          "%d pixels in the moved square, %d outside the source at the pad "
          "value, %d wrong", inside, outside, bad);
}

int main(int argc, char **argv)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);

    phys = argc > 1 ? strtoul(argv[1], NULL, 0) : 0;
    if (!phys || fd < 0) {
        printf("AIPTEST FAIL: usage / /dev/mem (%d)\n", fd);
        return 1;
    }
    v = mmap(NULL, 4 << 20, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys);
    if (v == MAP_FAILED) {
        printf("AIPTEST FAIL: mmap 0x%x\n", phys);
        return 1;
    }
    make_nv12();
    test_resize();
    test_covert();
    test_warp(0);
    test_warp(1);
    printf("AIPTEST DONE %d failure(s)\n", fails);
    return fails != 0;
}
