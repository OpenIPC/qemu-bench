/*
 * Rockchip RGA2 ("rockchip,rga2"; RV1126 / RV1109 0xFFAF0000), the 2D
 * engine, as the vendor driver (develop-4.19 drivers/video/rockchip/rga2:
 * rga2_drv.c, rga2_reg_info.c, rga2_mmu_info.c) programs it.
 *
 *   0x000 SYS_CTRL   bit 0 soft reset (reads back 0: done)
 *   0x004 CMD_CTRL   bit 0 starts the command at CMD_BASE
 *   0x008 CMD_BASE   physical address of the 32-word command
 *   0x00c STATUS     0: idle
 *   0x010 INT        status bits 0..3 (2: all commands done), cleared by
 *                    writing bits 4..7; bits 8..10 enable status 0..2
 *   0x028 VERSION    0 (the driver then calls it 2.00)
 *
 * The command (rga2_reg_info.h offsets): MODE_CTRL (0x00) bits 2:0 the
 * render mode -- 0 bitblt, 2 colour fill; SRC_INFO (0x04) format bits
 * 3:0, R/B swap 4, alpha swap 5, U/V swap 6, YUV->RGB mode 9:8; SRC_BASE0
 * .. 2 (0x08..0x10) Y / RGB, U or UV, V; SRC_VIR_INFO (0x18) stride in
 * words; SRC_ACT_INFO (0x1c) (w - 1) | (h - 1) << 16; SRC_FG_COLOR (0x2c)
 * the fill colour; DST_INFO (0x38) format 3:0, swaps 4..6, RGB->YUV mode
 * 17:16, Y only 24; DST_BASE0..2 (0x3c..0x44); DST_VIR_INFO (0x48);
 * DST_ACT_INFO (0x4c); MMU_CTRL1 (0x6c) bit 0 / 8 the source's /
 * destination's MMU on, with its page table at MMU_SRC_BASE / MMU_DST_BASE
 * (0x70 / 0x78) << 4: one physical page address per 4 KiB page of the
 * buffer, the base registers then being offsets into it.
 *
 * Formats: 0 RGBA8888, 1 RGBX8888, 2 RGB888, 4 RGB565, 5 RGBA5551,
 * 6 RGBA4444 (bytes R, G, B, A; B first with R/B swap, A first with alpha
 * swap); 8 YUV422 SP, 9 422 P, 0xa 420 SP, 0xb 420 P (chroma rows the
 * luma stride wide, half for planar); source 7 packed 4:2:2 (Y first with
 * R/B swap, V first with U/V swap); destination 0xc / 0xe packed 4:2:2
 * UYVY / YUYV (0xd / 0xf their 4:2:0, written as 4:2:2).  The copy scales
 * by nearest neighbour; YUV <-> RGB is BT.601 limited (mode 1), full (2)
 * or BT.709 limited (3).  SRC_INFO bits 11:10 rotate the source 90, 180
 * or 270 degrees clockwise (the destination's size is the rotated one),
 * bits 12 / 13 mirror it in x / y.  With ALPHA_CTRL0 (0x50) bit 0 set the
 * source goes over what the destination holds, weighted by the source's
 * own alpha (none: opaque); its Porter-Duff modes and global alphas are
 * not decoded.  ROPs, palettes and patterns are not done: those jobs
 * copy (or fill) straight.
 *
 * A job runs on a timer JOB_NS after it starts, then raises "all done".
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_RK_RGA2 "rk-rga2"
OBJECT_DECLARE_SIMPLE_TYPE(RkRga2State, RK_RGA2)

#define REGS_SIZE       0x1000
#define R_SYS_CTRL      0x000
#define R_CMD_CTRL      0x004
#define R_CMD_BASE      0x008
#define R_STATUS        0x00c
#define R_INT           0x010

#define INT_ALL_DONE    (1u << 2)
#define JOB_NS          500000
#define MAX_DIM         8192

/* the command's words */
#define C_MODE          (0x00 / 4)
#define C_SRC_INFO      (0x04 / 4)
#define C_SRC_BASE0     (0x08 / 4)
#define C_SRC_VIR       (0x18 / 4)
#define C_SRC_ACT       (0x1c / 4)
#define C_FG_COLOR      (0x2c / 4)
#define C_DST_INFO      (0x38 / 4)
#define C_DST_BASE0     (0x3c / 4)
#define C_DST_VIR       (0x48 / 4)
#define C_DST_ACT       (0x4c / 4)
#define C_ALPHA_CTRL0   (0x50 / 4)
#define C_MMU_CTRL1     (0x6c / 4)
#define C_MMU_SRC       (0x70 / 4)
#define C_MMU_DST       (0x78 / 4)

struct RkRga2State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[REGS_SIZE / 4];
    uint32_t ints;              /* INT status bits 3:0 */
    bool trace;                 /* property: log jobs and interrupts */
};

#define RGA_TRACE(s, fmt, ...) do {                                     \
    if ((s)->trace) {                                                   \
        qemu_log("rk-rga2 %" PRId64 " " fmt "\n",                        \
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000, ## __VA_ARGS__); \
    }                                                                   \
} while (0)

#define REG(s, off) ((s)->regs[(off) / 4])

/* One side of the copy: a picture in memory, maybe through the RGA's MMU */
typedef struct RgaPic {
    uint32_t base[3];
    uint32_t stride;            /* bytes */
    uint32_t w, h;
    uint32_t fmt;
    bool rb_swap, a_swap, uv_swap;
    int csc;                    /* 0 none, 1 601 limited, 2 601 full, 3 709 */
    bool y_only;
    hwaddr mmu;                 /* page table, 0: physical */
} RgaPic;

static void rk_rga2_update(RkRga2State *s)
{
    REG(s, R_INT) = (REG(s, R_INT) & ~0xfu) | s->ints;
    qemu_set_irq(s->irq, !!(s->ints & (REG(s, R_INT) >> 8) & 7));
}

static void rga_mem(const RgaPic *p, uint32_t a, void *buf, uint32_t len,
                    bool wr)
{
    uint8_t *q = buf;

    while (len) {
        uint32_t n = MIN(len, 0x1000 - (a & 0xfff));
        hwaddr pa = a;

        if (p->mmu) {
            /*
             * The RGA2's own table holds plain page addresses, no valid
             * bit (the kernel's rga2_mmu_info.c); a table read that fails
             * maps nothing: a read gives zeros, a write is dropped.
             */
            uint32_t pte = 0;

            if (dma_memory_read(&address_space_memory,
                                p->mmu + (a >> 12) * 4, &pte, 4,
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                if (!wr) {
                    memset(q, 0, n);
                }
                q += n;
                a += n;
                len -= n;
                continue;
            }
            pa = (le32_to_cpu(pte) & ~0xfffu) | (a & 0xfff);
        }
        if (wr) {
            dma_memory_write(&address_space_memory, pa, q, n,
                             MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_read(&address_space_memory, pa, q, n,
                            MEMTXATTRS_UNSPECIFIED);
        }
        q += n;
        a += n;
        len -= n;
    }
}

static bool rga_is_yuv(uint32_t fmt)
{
    return fmt >= 7;
}

static int rga_bpp(uint32_t fmt)
{
    static const int bpp[16] = { 4, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 2, 2,
                                 2, 2 };
    return bpp[fmt & 0xf];
}

static uint8_t clip8(double v)
{
    return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)(v + 0.5);
}

static void rga_yuv2rgb(int csc, const uint8_t yuv[3], uint8_t rgb[3])
{
    double y = yuv[0], u = yuv[1] - 128.0, v = yuv[2] - 128.0;

    switch (csc) {
    case 2:                                     /* BT.601 full */
        rgb[0] = clip8(y + 1.402 * v);
        rgb[1] = clip8(y - 0.344136 * u - 0.714136 * v);
        rgb[2] = clip8(y + 1.772 * u);
        break;
    case 3:                                     /* BT.709 limited */
        y = (y - 16) * 1.164384;
        rgb[0] = clip8(y + 1.792741 * v);
        rgb[1] = clip8(y - 0.213249 * u - 0.532909 * v);
        rgb[2] = clip8(y + 2.112402 * u);
        break;
    default:                                    /* BT.601 limited */
        y = (y - 16) * 1.164384;
        rgb[0] = clip8(y + 1.596027 * v);
        rgb[1] = clip8(y - 0.391762 * u - 0.812968 * v);
        rgb[2] = clip8(y + 2.017232 * u);
        break;
    }
}

static void rga_rgb2yuv(int csc, const uint8_t rgb[3], uint8_t yuv[3])
{
    double r = rgb[0], g = rgb[1], b = rgb[2];

    switch (csc) {
    case 2:
        yuv[0] = clip8(0.299 * r + 0.587 * g + 0.114 * b);
        yuv[1] = clip8(128 - 0.168736 * r - 0.331264 * g + 0.5 * b);
        yuv[2] = clip8(128 + 0.5 * r - 0.418688 * g - 0.081312 * b);
        break;
    case 3:
        yuv[0] = clip8(16 + 0.182586 * r + 0.614231 * g + 0.062007 * b);
        yuv[1] = clip8(128 - 0.100644 * r - 0.338572 * g + 0.439216 * b);
        yuv[2] = clip8(128 + 0.439216 * r - 0.398942 * g - 0.040274 * b);
        break;
    default:
        yuv[0] = clip8(16 + 0.256788 * r + 0.504129 * g + 0.097906 * b);
        yuv[1] = clip8(128 - 0.148223 * r - 0.290993 * g + 0.439216 * b);
        yuv[2] = clip8(128 + 0.439216 * r - 0.367788 * g - 0.071427 * b);
        break;
    }
}

/* An RGB pixel's bytes <-> R, G, B, A */
static void rga_unpack_rgb(const RgaPic *p, const uint8_t *q, uint8_t c[4])
{
    uint16_t v = q[0] | q[1] << 8;

    switch (p->fmt) {
    case 0:
    case 1:
        if (p->a_swap) {
            c[3] = q[0];
            q++;
        } else {
            c[3] = q[3];
        }
        c[0] = q[0];
        c[1] = q[1];
        c[2] = q[2];
        if (p->fmt == 1) {
            c[3] = 255;
        }
        break;
    case 2:
    case 3:
        c[0] = q[0];
        c[1] = q[1];
        c[2] = q[2];
        c[3] = 255;
        break;
    case 4:
        c[0] = (v >> 11) * 255 / 31;
        c[1] = ((v >> 5) & 63) * 255 / 63;
        c[2] = (v & 31) * 255 / 31;
        c[3] = 255;
        break;
    case 5:
        c[0] = ((v >> 11) & 31) * 255 / 31;
        c[1] = ((v >> 6) & 31) * 255 / 31;
        c[2] = ((v >> 1) & 31) * 255 / 31;
        c[3] = (v & 1) * 255;
        break;
    default:
        c[0] = ((v >> 12) & 15) * 17;
        c[1] = ((v >> 8) & 15) * 17;
        c[2] = ((v >> 4) & 15) * 17;
        c[3] = (v & 15) * 17;
        break;
    }
    if (p->rb_swap) {
        uint8_t t = c[0];

        c[0] = c[2];
        c[2] = t;
    }
}

static void rga_pack_rgb(const RgaPic *p, const uint8_t c0[4], uint8_t *q)
{
    uint8_t c[4] = { c0[0], c0[1], c0[2], c0[3] };
    uint16_t v;

    if (p->rb_swap) {
        c[0] = c0[2];
        c[2] = c0[0];
    }
    switch (p->fmt) {
    case 0:
    case 1:
        if (p->a_swap) {
            q[0] = c[3];
            q[1] = c[0];
            q[2] = c[1];
            q[3] = c[2];
        } else {
            q[0] = c[0];
            q[1] = c[1];
            q[2] = c[2];
            q[3] = c[3];
        }
        return;
    case 2:
    case 3:
        q[0] = c[0];
        q[1] = c[1];
        q[2] = c[2];
        return;
    case 4:
        v = (c[0] >> 3) << 11 | (c[1] >> 2) << 5 | c[2] >> 3;
        break;
    case 5:
        v = (c[0] >> 3) << 11 | (c[1] >> 3) << 6 | (c[2] >> 3) << 1 |
            (c[3] >> 7);
        break;
    default:
        v = (c[0] >> 4) << 12 | (c[1] >> 4) << 8 | (c[2] >> 4) << 4 |
            c[3] >> 4;
        break;
    }
    q[0] = v;
    q[1] = v >> 8;
}

/* Source row @y as Y, U, V or R, G, B, A per pixel (4 bytes each) */
static void rga_read_row(const RgaPic *p, uint32_t y, uint8_t *row,
                         uint8_t *t0, uint8_t *t1, uint8_t *t2)
{
    uint32_t x, w = p->w;

    if (!rga_is_yuv(p->fmt)) {
        int bpp = rga_bpp(p->fmt);

        rga_mem(p, p->base[0] + y * p->stride, t0, w * bpp, false);
        for (x = 0; x < w; x++) {
            rga_unpack_rgb(p, t0 + x * bpp, row + 4 * x);
        }
        return;
    }
    if (p->fmt == 7) {                          /* packed 4:2:2 */
        rga_mem(p, p->base[0] + y * p->stride, t0, w * 2 + 4, false);
        for (x = 0; x < w; x++) {
            const uint8_t *q = t0 + (x & ~1u) * 2;
            uint8_t yy = p->rb_swap ? q[(x & 1) * 2] : q[(x & 1) * 2 + 1];
            uint8_t c0 = p->rb_swap ? q[1] : q[0];
            uint8_t c1 = p->rb_swap ? q[3] : q[2];

            row[4 * x] = yy;
            row[4 * x + 1] = p->uv_swap ? c1 : c0;
            row[4 * x + 2] = p->uv_swap ? c0 : c1;
        }
        return;
    }
    {
        bool planar = p->fmt == 9 || p->fmt == 0xb;
        uint32_t cy = (p->fmt >= 0xa) ? y / 2 : y;
        uint32_t cs = planar ? p->stride / 2 : p->stride;

        rga_mem(p, p->base[0] + y * p->stride, t0, w, false);
        if (planar) {
            rga_mem(p, p->base[1] + cy * cs, t1, w / 2 + 1, false);
            rga_mem(p, p->base[2] + cy * cs, t2, w / 2 + 1, false);
        } else {
            rga_mem(p, p->base[1] + cy * cs, t1, (w & ~1u) + 2, false);
        }
        for (x = 0; x < w; x++) {
            uint8_t u, v;

            if (planar) {
                u = t1[x / 2];
                v = t2[x / 2];
            } else {
                u = t1[x & ~1u];
                v = t1[(x & ~1u) + 1];
            }
            row[4 * x] = t0[x];
            row[4 * x + 1] = p->uv_swap ? v : u;
            row[4 * x + 2] = p->uv_swap ? u : v;
        }
    }
}

/* Destination row @y from pixels in the destination's space (YUV or RGBA) */
static void rga_write_row(const RgaPic *p, uint32_t y, const uint8_t *row,
                          uint8_t *t0, uint8_t *t1, uint8_t *t2)
{
    uint32_t x, w = p->w;

    if (!rga_is_yuv(p->fmt)) {
        int bpp = rga_bpp(p->fmt);

        for (x = 0; x < w; x++) {
            rga_pack_rgb(p, row + 4 * x, t0 + x * bpp);
        }
        rga_mem(p, p->base[0] + y * p->stride, t0, w * bpp, true);
        return;
    }
    if (p->fmt >= 0xc) {                        /* packed */
        bool yfirst = p->fmt >= 0xe;

        for (x = 0; x < w; x += 2) {
            const uint8_t *a = row + 4 * x;
            const uint8_t *b = row + 4 * MIN(x + 1, w - 1);
            uint8_t u = a[1], v = a[2], *q = t0 + 2 * x;

            if (!p->uv_swap) {
                uint8_t t = u;

                u = v;
                v = t;
            }
            q[yfirst ? 0 : 1] = a[0];
            q[yfirst ? 2 : 3] = b[0];
            q[yfirst ? 1 : 0] = u;
            q[yfirst ? 3 : 2] = v;
        }
        rga_mem(p, p->base[0] + y * p->stride, t0, w * 2, true);
        return;
    }
    {
        bool planar = p->fmt == 9 || p->fmt == 0xb;
        bool c420 = p->fmt >= 0xa;
        uint32_t cs = planar ? p->stride / 2 : p->stride;

        for (x = 0; x < w; x++) {
            t0[x] = row[4 * x];
        }
        rga_mem(p, p->base[0] + y * p->stride, t0, w, true);
        if (p->y_only || (c420 && (y & 1))) {
            return;
        }
        for (x = 0; x < w; x += 2) {
            uint8_t u = row[4 * x + 1], v = row[4 * x + 2];

            if (planar) {
                t1[x / 2] = u;
                t2[x / 2] = v;
            } else {
                t1[x] = p->uv_swap ? v : u;
                t1[x + 1] = p->uv_swap ? u : v;
            }
        }
        y = c420 ? y / 2 : y;
        if (planar) {
            rga_mem(p, p->base[1] + y * cs, t1, (w + 1) / 2, true);
            rga_mem(p, p->base[2] + y * cs, t2, (w + 1) / 2, true);
        } else {
            rga_mem(p, p->base[1] + y * cs, t1, w & ~1u, true);
        }
    }
}

static void rga_pic(const uint32_t *cmd, bool dst, RgaPic *p)
{
    uint32_t info = cmd[dst ? C_DST_INFO : C_SRC_INFO];
    uint32_t act = cmd[dst ? C_DST_ACT : C_SRC_ACT];
    uint32_t mmu = cmd[C_MMU_CTRL1] >> (dst ? 8 : 0);
    int i;

    for (i = 0; i < 3; i++) {
        p->base[i] = cmd[(dst ? C_DST_BASE0 : C_SRC_BASE0) + i];
    }
    p->stride = (cmd[dst ? C_DST_VIR : C_SRC_VIR] & 0xffff) * 4;
    p->w = (act & 0x1fff) + 1;
    p->h = ((act >> 16) & 0x1fff) + 1;
    p->fmt = info & 0xf;
    p->rb_swap = info & (1u << 4);
    p->a_swap = info & (1u << 5);
    p->uv_swap = info & (1u << 6);
    p->csc = dst ? (info >> 16) & 3 : (info >> 8) & 3;
    p->y_only = dst && (info & (1u << 24));
    p->mmu = (mmu & 1) ? (hwaddr)cmd[dst ? C_MMU_DST : C_MMU_SRC] << 4 : 0;
}

/* a source pixel in the destination's space (YUV or RGBA) */
static void rga_convert(const RgaPic *src, const RgaPic *dst,
                        const uint8_t *a, uint8_t *b)
{
    if (rga_is_yuv(src->fmt) == rga_is_yuv(dst->fmt)) {
        memcpy(b, a, 4);
    } else if (rga_is_yuv(src->fmt)) {
        rga_yuv2rgb(src->csc, a, b);
        b[3] = 255;
    } else {
        rga_rgb2yuv(dst->csc, a, b);
    }
}

/*
 * The copy with rotation, mirroring or blending: the whole source in, each
 * destination pixel taken back through the rotation and mirror
 */
static void rga_blit_general(const uint32_t *cmd, const RgaPic *src,
                             const RgaPic *dst, uint8_t *in, uint8_t *out,
                             uint8_t *t0, uint8_t *t1, uint8_t *t2)
{
    int rot = (cmd[C_SRC_INFO] >> 10) & 3;
    bool mx = cmd[C_SRC_INFO] & (1u << 12), my = cmd[C_SRC_INFO] & (1u << 13);
    bool blend = cmd[C_ALPHA_CTRL0] & 1;
    bool src_alpha = !rga_is_yuv(src->fmt) && src->fmt != 1 && src->fmt != 2 &&
                     src->fmt != 3 && src->fmt != 4;
    g_autofree uint8_t *pic = NULL, *under = NULL;
    uint32_t x, y;

    if ((uint64_t)src->w * src->h > 4096 * 4096) {
        return;
    }
    pic = g_malloc((size_t)src->w * src->h * 4);
    for (y = 0; y < src->h; y++) {
        rga_read_row(src, y, pic + (size_t)y * src->w * 4, t0, t1, t2);
    }
    if (blend) {
        under = g_malloc0(4 * MAX_DIM);
    }
    for (y = 0; y < dst->h; y++) {
        if (blend) {
            rga_read_row(dst, y, under, t0, t1, t2);
        }
        for (x = 0; x < dst->w; x++) {
            /* where in the source, rotated clockwise by rot * 90 */
            uint64_t u = x, v = y, sx, sy;

            switch (rot) {
            case 0:
                sx = u * src->w / dst->w;
                sy = v * src->h / dst->h;
                break;
            case 1:                 /* 90: dst x runs up the source */
                sx = v * src->w / dst->h;
                sy = src->h - 1 - u * src->h / dst->w;
                break;
            case 2:
                sx = src->w - 1 - u * src->w / dst->w;
                sy = src->h - 1 - v * src->h / dst->h;
                break;
            default:                /* 270 */
                sx = src->w - 1 - v * src->w / dst->h;
                sy = u * src->h / dst->w;
                break;
            }
            if (mx) {
                sx = src->w - 1 - sx;
            }
            if (my) {
                sy = src->h - 1 - sy;
            }
            const uint8_t *a = pic + ((size_t)sy * src->w + sx) * 4;
            uint8_t *b = out + 4 * x;

            rga_convert(src, dst, a, b);
            if (blend) {
                unsigned al = src_alpha ? a[3] : 255, i;
                uint8_t *c = under + 4 * x;

                for (i = 0; i < 3; i++) {
                    b[i] = (b[i] * al + c[i] * (255 - al) + 127) / 255;
                }
                b[3] = al + c[3] * (255 - al) / 255;
            }
        }
        rga_write_row(dst, y, out, t0, t1, t2);
    }
}

/*
 * NV12 to NV12, scaled: what the general path below does pixel by pixel
 * (the nearest source pixel, its chroma pair), a row of bytes at a time.
 * The XM R5's 1920x1080 -> 2880x1616 upscale took 14 ms that way.
 */
static void rga_scale_nv12(const RgaPic *src, const RgaPic *dst,
                           uint8_t *sy_row, uint8_t *sc_row, uint8_t *o)
{
    g_autofree uint32_t *xm = g_new(uint32_t, dst->w);
    bool swap = !!src->uv_swap != !!dst->uv_swap;
    uint32_t x, y, last = UINT32_MAX, lastc = UINT32_MAX;
    uint32_t cw = dst->w & ~1u;

    for (x = 0; x < dst->w; x++) {
        xm[x] = (uint64_t)x * src->w / dst->w;
    }
    for (y = 0; y < dst->h; y++) {
        uint32_t sy = (uint64_t)y * src->h / dst->h;

        if (sy != last) {
            rga_mem(src, src->base[0] + sy * src->stride, sy_row, src->w,
                    false);
            last = sy;
        }
        for (x = 0; x < dst->w; x++) {
            o[x] = sy_row[xm[x]];
        }
        rga_mem(dst, dst->base[0] + y * dst->stride, o, dst->w, true);
        if (dst->y_only || (y & 1)) {
            continue;
        }
        if (sy / 2 != lastc) {
            rga_mem(src, src->base[1] + (sy / 2) * src->stride, sc_row,
                    (src->w & ~1u) + 2, false);
            lastc = sy / 2;
        }
        for (x = 0; x < cw; x += 2) {
            const uint8_t *c = sc_row + (xm[x] & ~1u);

            o[x] = c[swap];
            o[x + 1] = c[!swap];
        }
        rga_mem(dst, dst->base[1] + (y / 2) * dst->stride, o, cw, true);
    }
}

static void rk_rga2_run(RkRga2State *s)
{
    uint32_t cmd[32];
    RgaPic src, dst;
    g_autofree uint8_t *in = NULL, *out = NULL, *t0 = NULL, *t1 = NULL,
                       *t2 = NULL;
    uint32_t mode, x, y, last = UINT32_MAX;
    int i;

    dma_memory_read(&address_space_memory, REG(s, R_CMD_BASE), cmd,
                    sizeof(cmd), MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < 32; i++) {
        cmd[i] = le32_to_cpu(cmd[i]);
    }
    mode = cmd[C_MODE] & 7;
    rga_pic(cmd, false, &src);
    rga_pic(cmd, true, &dst);
    /* with the MMU on, 0 is an offset into the pages like any other */
    if (dst.w > MAX_DIM || src.w > MAX_DIM || (!dst.base[0] && !dst.mmu)) {
        return;
    }
    in = g_new0(uint8_t, 4 * MAX_DIM);
    out = g_new0(uint8_t, 4 * MAX_DIM);
    t0 = g_new0(uint8_t, 4 * MAX_DIM + 8);
    t1 = g_new0(uint8_t, 4 * MAX_DIM + 8);
    t2 = g_new0(uint8_t, 4 * MAX_DIM + 8);

    if (mode == 2) {                            /* colour fill */
        uint32_t fg = cmd[C_FG_COLOR];
        uint8_t c[4] = { fg, fg >> 8, fg >> 16, fg >> 24 };

        if (rga_is_yuv(dst.fmt)) {
            uint8_t yuv[3];

            rga_rgb2yuv(dst.csc, c, yuv);
            memcpy(c, yuv, 3);
        }
        for (x = 0; x < dst.w; x++) {
            memcpy(out + 4 * x, c, 4);
        }
        for (y = 0; y < dst.h; y++) {
            rga_write_row(&dst, y, out, t0, t1, t2);
        }
        return;
    }
    if (mode != 0) {
        qemu_log_mask(LOG_UNIMP, "rk-rga2: render mode %u\n", mode);
        return;
    }
    if (((cmd[C_SRC_INFO] >> 10) & 0xf) || (cmd[C_ALPHA_CTRL0] & 1)) {
        rga_blit_general(cmd, &src, &dst, in, out, t0, t1, t2);
        return;
    }
    if (src.fmt == 0xa && dst.fmt == 0xa) {
        rga_scale_nv12(&src, &dst, t0, t1, t2);
        return;
    }
    for (y = 0; y < dst.h; y++) {
        uint32_t sy = (uint64_t)y * src.h / dst.h;

        if (sy != last) {
            rga_read_row(&src, sy, in, t0, t1, t2);
            last = sy;
        }
        for (x = 0; x < dst.w; x++) {
            const uint8_t *a = in + 4 * ((uint64_t)x * src.w / dst.w);
            uint8_t *b = out + 4 * x;

            if (rga_is_yuv(src.fmt) == rga_is_yuv(dst.fmt)) {
                memcpy(b, a, 4);
            } else if (rga_is_yuv(src.fmt)) {
                rga_yuv2rgb(src.csc, a, b);
                b[3] = 255;
            } else {
                rga_rgb2yuv(dst.csc, a, b);
            }
        }
        rga_write_row(&dst, y, out, t0, t1, t2);
    }
}

static void rk_rga2_done(void *opaque)
{
    RkRga2State *s = opaque;
    int64_t t0 = g_get_monotonic_time();

    rk_rga2_run(s);
    t0 = g_get_monotonic_time() - t0;
    REG(s, R_STATUS) = 0;
    s->ints |= INT_ALL_DONE;
    if (s->trace) {
        uint32_t c[32];

        dma_memory_read(&address_space_memory, REG(s, R_CMD_BASE), c,
                        sizeof(c), MEMTXATTRS_UNSPECIFIED);
        RGA_TRACE(s, "done mode %u src_info 0x%x alpha0 0x%x src %ux%u dst "
                  "%ux%u dst0 0x%x, int en 0x%x, %" PRId64 " us",
                  le32_to_cpu(c[0]) & 7, le32_to_cpu(c[1]),
                  le32_to_cpu(c[0x50 / 4]),
                  (le32_to_cpu(c[0x1c / 4]) & 0x1fff) + 1,
                  ((le32_to_cpu(c[0x1c / 4]) >> 16) & 0x1fff) + 1,
                  (le32_to_cpu(c[0x4c / 4]) & 0x1fff) + 1,
                  ((le32_to_cpu(c[0x4c / 4]) >> 16) & 0x1fff) + 1,
                  le32_to_cpu(c[0x3c / 4]), REG(s, R_INT), t0);
        RGA_TRACE(s, "  src0 0x%x vir %u mmu 0x%x", le32_to_cpu(c[0x08 / 4]),
                  le32_to_cpu(c[0x18 / 4]) & 0xffff,
                  le32_to_cpu(c[0x6c / 4]));
    }
    rk_rga2_update(s);
}

static uint64_t rk_rga2_read(void *opaque, hwaddr off, unsigned size)
{
    RkRga2State *s = opaque;

    return s->regs[off / 4];
}

static void rk_rga2_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    RkRga2State *s = opaque;

    switch (off) {
    case R_SYS_CTRL:
        REG(s, off) = val & ~1u;            /* soft reset: done at once */
        if (val & 1) {
            timer_del(s->timer);
            REG(s, R_STATUS) = 0;
        }
        return;
    case R_CMD_CTRL:
        if (val & 1) {
            RGA_TRACE(s, "start cmd 0x%x%s", REG(s, R_CMD_BASE),
                      timer_pending(s->timer) ? " (one still running)" : "");
            REG(s, R_STATUS) = 1;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      JOB_NS);
        }
        return;
    case R_STATUS:
        return;
    case R_INT:
        RGA_TRACE(s, "int wr 0x%x (status 0x%x)", (uint32_t)val, s->ints);
        s->ints &= ~((val >> 4) & 0xf);
        REG(s, R_INT) = val & ~0xffu;
        rk_rga2_update(s);
        return;
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps rk_rga2_ops = {
    .read = rk_rga2_read,
    .write = rk_rga2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_rga2_reset(DeviceState *dev)
{
    RkRga2State *s = RK_RGA2(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ints = 0;
    timer_del(s->timer);
    rk_rga2_update(s);
}

static void rk_rga2_init(Object *obj)
{
    RkRga2State *s = RK_RGA2(obj);

    memory_region_init_io(&s->iomem, obj, &rk_rga2_ops, s, TYPE_RK_RGA2,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_rga2_done, s);
}

static const VMStateDescription rk_rga2_vmstate = {
    .name = TYPE_RK_RGA2,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkRga2State, REGS_SIZE / 4),
        VMSTATE_UINT32(ints, RkRga2State),
        VMSTATE_TIMER_PTR(timer, RkRga2State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property rk_rga2_props[] = {
    DEFINE_PROP_BOOL("trace", RkRga2State, trace, false),
};

static void rk_rga2_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_rga2_reset);
    device_class_set_props(dc, rk_rga2_props);
    dc->vmsd = &rk_rga2_vmstate;
}

static const TypeInfo rk_rga2_info = {
    .name = TYPE_RK_RGA2,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkRga2State),
    .instance_init = rk_rga2_init,
    .class_init = rk_rga2_class_init,
};

static void rk_rga2_register_types(void)
{
    type_register_static(&rk_rga2_info);
}

type_init(rk_rga2_register_types)
