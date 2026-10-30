/*
 * Ingenic IPU ("jz-ipu", 0x13080000, INTC source 6 = Linux IRQ 14): the
 * image processing unit the vendor libimp blends its OSD regions with.
 * The same block (the kernels' jz_ipu_v13 / ingenic_ipu_v13 driver,
 * identical register files) sits at the same address and interrupt on
 * T10, T20, T21, T23, T30, T31, T40 and T41.
 *
 * The kernel driver takes a struct ipu_param from libimp through
 * IOCTL_IPU_START, programs one operation, starts it and waits up to two
 * seconds for the interrupt:
 *
 *   OSD: the background, an NV12 (or NV21) frame (IPU_OSD_CH_BK_PARA
 *     [12:11] = 2 / 3, bit 0 enable), is IN_FM_GS (w << 16 | h) at
 *     Y_ADDR / U_ADDR with Y_STRIDE and UV_STRIDE[31:16]; it is written to
 *     OUT_ADDR / NV_OUT_ADDR (OUT_STRIDE, NV_OUT_STRIDE) -- the same
 *     buffer, in place.  Up to four OSD channels n are laid over it in
 *     channel order, each enabled by bit 0 of its PARA (0x11C + 4 n):
 *       PARA [2:1] alpha: 0 per pixel, 1 global, 2 per pixel x global;
 *            [10:3] the global alpha; [13:11] the picture: 0 ARGB (32
 *            bits, [17:14] the byte order, CH_BK_ARGB_TYPE_*), 2 NV12,
 *            3 NV21, 4 (5551, the driver's isrgb == 2) 16-bit ARGB1555;
 *            bit 18 premultiplied colour, bit 19 mask: the channel's
 *            BAK_ARGB colour instead of the picture, under its alpha.
 *       GS (0xFC + 4 n): width in BYTES << 16 | height (4 w for ARGB);
 *       POS (0x10C + 4 n): x << 16 | y on the background;
 *       Y/UV ADDR and STRIDE at 0xBC + 0x10 n;
 *       the RGB -> YUV matrix at 0x148 + 0x18 n: C0 = Ry | Gy << 20,
 *       C1 = By | Ru << 20, C2 = Gu | Bu << 20, C3 = Rv | Gv << 20,
 *       C4 = Bv (x 1024, U = -Ru R - Gu G + Bu B, V = Rv R - Gv G - Bv B),
 *       OFSET_PARA = Y offset | UV offset << 16.
 *   CSC (cmd IPU_CMD_CSC): the NV12 background converted to NV21
 *     (D_FMT [2:0] = 3) or to 32-bit RGB (D_FMT 0x40, BGRA bytes) at
 *     OUT_ADDR, or to HSV (D_FMT 0x4: three planes, ipu_csc_hsv()).
 *
 * TRIGGER (0x74) bit 3 resets the unit: every register to 0 (the driver
 * resets before it programs an operation, and writes only the channels
 * it uses, so stale channel enables must not survive); bit 0 runs it.
 * The picture is written at once and STATUS bit 0 (OUT_END) set a
 * virtual 100 us later; the interrupt is OUT_END while GLB_CTRL bit 0
 * (IRQ_EN) is set -- the handler clears IRQ_EN, the next start clears
 * OUT_END.
 *
 * "-global ingenic-ipu.debug=on" logs every operation.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_INGENIC_IPU "ingenic-ipu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIpuState, INGENIC_IPU)

#define IPU_SIZE        0x10000
#define IPU_NREGS       (0x200 / 4)

#define R_FM_CTRL       0x000
#define R_STATUS        0x004
#define R_D_FMT         0x008
#define R_Y_ADDR        0x00C
#define R_U_ADDR        0x010
#define R_IN_FM_GS      0x018
#define R_Y_STRIDE      0x01C
#define R_UV_STRIDE     0x020
#define R_OUT_ADDR      0x024
#define R_OUT_STRIDE    0x02C
#define R_OUT_V_ADDR    0x030
#define R_OUT_V_STRIDE  0x034
#define R_TRIGGER       0x074
#define R_GLB_CTRL      0x07C
#define R_NV_OUT_ADDR   0x0B4
#define R_NV_OUT_STRIDE 0x0B8
#define R_OSD_Y_ADDR(n)     (0x0BC + 0x10 * (n))
#define R_OSD_Y_STRIDE(n)   (0x0C0 + 0x10 * (n))
#define R_OSD_UV_ADDR(n)    (0x0C4 + 0x10 * (n))
#define R_OSD_UV_STRIDE(n)  (0x0C8 + 0x10 * (n))
#define R_OSD_GS(n)     (0x0FC + 4 * (n))
#define R_OSD_POS(n)    (0x10C + 4 * (n))
#define R_OSD_PARA(n)   (0x11C + 4 * (n))
#define R_OSD_BK_PARA   0x12C
#define R_OSD_BAK_ARGB(n)   (0x130 + 4 * (n))
#define R_CSC(n, i)     (0x148 + 0x18 * (n) + 4 * (i))

#define STATUS_OUT_END  (1u << 0)
#define TRIG_RUN        (1u << 0)
#define TRIG_RESET      (1u << 3)
#define GLB_IRQ_EN      (1u << 0)

#define PIC_ARGB        0
#define PIC_NV12        2
#define PIC_NV21        3
#define PIC_ARGB1555    4

#define IPU_MAX_W       4096
#define IPU_MAX_H       4096
#define IPU_DONE_NS     100000

struct IngenicIpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[IPU_NREGS];
    bool debug;
};

static inline uint32_t R(IngenicIpuState *s, hwaddr off)
{
    return s->regs[off >> 2];
}

static void ipu_update_irq(IngenicIpuState *s)
{
    qemu_set_irq(s->irq, (R(s, R_STATUS) & STATUS_OUT_END) &&
                         (R(s, R_GLB_CTRL) & GLB_IRQ_EN));
}

static void ipu_read(hwaddr a, void *buf, size_t n)
{
    dma_memory_read(&address_space_memory, a, buf, n, MEMTXATTRS_UNSPECIFIED);
}

static void ipu_write(hwaddr a, const void *buf, size_t n)
{
    dma_memory_write(&address_space_memory, a, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
}

static inline int clamp255(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* one OSD pixel as Y, U, V and an alpha 0..255 */
typedef struct OsdPix {
    int y, u, v, a;
} OsdPix;

typedef struct OsdChan {
    int n, pic, w, h, x0, y0, amode, galpha, bpp;
    bool prem, mask;
    uint32_t para, bak;
    hwaddr ya, uva;
    uint32_t ys, uvs;
    int c[9], yoff, uvoff;          /* Ry Gy By Ru Gu Bu Rv Gv Bv */
    uint8_t *line;                  /* one source line */
    uint8_t *uvline;
    int cur_line, cur_uvline;
} OsdChan;

static void osd_rgb2yuv(const OsdChan *c, int r, int g, int b, OsdPix *p)
{
    p->y = clamp255(((c->c[0] * r + c->c[1] * g + c->c[2] * b) >> 10) +
                    c->yoff);
    p->u = clamp255(((-c->c[3] * r - c->c[4] * g + c->c[5] * b) >> 10) +
                    c->uvoff);
    p->v = clamp255(((c->c[6] * r - c->c[7] * g - c->c[8] * b) >> 10) +
                    c->uvoff);
}

/*
 * CH_BK_ARGB_TYPE: which byte (in memory order) holds A, R, G, B.  The
 * codes name the components from the first byte in memory: 0 ARGB is
 * B, G, R, A in memory on this little-endian part (a 32-bit 0xAARRGGBB
 * word), as libimp's OSD_PIX_FMT_BGRA buffers are.
 */
static void osd_argb_order(int t, int *ia, int *ir, int *ig, int *ib)
{
    static const char *orders[16] = {
        "ARGB", "ARBG", "AGRB", "AGBR", "ABRG", "ABGR", NULL, NULL,
        "RGBA", "RBGA", "GRBA", "GBRA", "BRGA", "BGRA", NULL, NULL,
    };
    const char *o = orders[t & 15] ? orders[t & 15] : "ARGB";
    int i;

    /* the name is the 32-bit word from its top byte down */
    for (i = 0; i < 4; i++) {
        int byte = 3 - i;

        switch (o[i]) {
        case 'A': *ia = byte; break;
        case 'R': *ir = byte; break;
        case 'G': *ig = byte; break;
        default:  *ib = byte; break;
        }
    }
}

static void osd_pixel(OsdChan *c, int x, int y, OsdPix *p)
{
    int a = 255;

    if (c->mask) {
        uint32_t k = c->bak;

        a = k >> 24;
        osd_rgb2yuv(c, (k >> 16) & 0xff, (k >> 8) & 0xff, k & 0xff, p);
    } else if (c->pic == PIC_NV12 || c->pic == PIC_NV21) {
        if (c->cur_line != y) {
            ipu_read(c->ya + (hwaddr)y * c->ys, c->line, c->w);
            c->cur_line = y;
        }
        if (c->cur_uvline != y / 2) {
            ipu_read(c->uva + (hwaddr)(y / 2) * c->uvs, c->uvline,
                     (c->w + 1) & ~1);
            c->cur_uvline = y / 2;
        }
        p->y = c->line[x];
        p->u = c->uvline[(x & ~1) + (c->pic == PIC_NV21)];
        p->v = c->uvline[(x & ~1) + (c->pic == PIC_NV12)];
    } else {
        uint8_t *px;

        if (c->cur_line != y) {
            ipu_read(c->ya + (hwaddr)y * c->ys, c->line, c->w * c->bpp);
            c->cur_line = y;
        }
        px = c->line + x * c->bpp;
        if (c->bpp == 2) {
            uint16_t v = px[0] | px[1] << 8;
            int r = (v >> 10) & 31, g = (v >> 5) & 31, b = v & 31;

            a = v & 0x8000 ? 255 : 0;
            osd_rgb2yuv(c, r << 3 | r >> 2, g << 3 | g >> 2, b << 3 | b >> 2,
                        p);
        } else {
            int ia = 3, ir = 2, ig = 1, ib = 0;
            int r, g, b;

            osd_argb_order((c->para >> 14) & 15, &ia, &ir, &ig, &ib);
            a = px[ia];
            r = px[ir];
            g = px[ig];
            b = px[ib];
            if (c->prem && a) {
                r = MIN(255, r * 255 / a);
                g = MIN(255, g * 255 / a);
                b = MIN(255, b * 255 / a);
            }
            osd_rgb2yuv(c, r, g, b, p);
        }
    }
    switch (c->amode) {
    case 1:
        a = c->galpha;
        break;
    case 2:
        a = a * c->galpha / 255;
        break;
    default:
        break;
    }
    p->a = a;
}

static bool osd_chan_setup(IngenicIpuState *s, int n, OsdChan *c)
{
    uint32_t gs = R(s, R_OSD_GS(n)), pos = R(s, R_OSD_POS(n));
    int i;

    memset(c, 0, sizeof(*c));
    c->n = n;
    c->para = R(s, R_OSD_PARA(n));
    if (!(c->para & 1)) {
        return false;
    }
    c->pic = (c->para >> 11) & 7;
    c->amode = (c->para >> 1) & 3;
    c->galpha = (c->para >> 3) & 0xff;
    c->prem = c->para & (1u << 18);
    c->mask = c->para & (1u << 19);
    c->bak = R(s, R_OSD_BAK_ARGB(n));
    c->bpp = c->pic == PIC_ARGB ? 4 : c->pic == PIC_ARGB1555 ? 2 : 1;
    if (c->pic != PIC_ARGB && c->pic != PIC_NV12 && c->pic != PIC_NV21 &&
        c->pic != PIC_ARGB1555) {
        qemu_log_mask(LOG_UNIMP, "ingenic-ipu: OSD %d picture type %d\n",
                      n, c->pic);
        return false;
    }
    c->w = (gs >> 16) / c->bpp;
    c->h = gs & 0xffff;
    c->x0 = pos >> 16;
    c->y0 = pos & 0xffff;
    c->ya = R(s, R_OSD_Y_ADDR(n));
    c->uva = R(s, R_OSD_UV_ADDR(n));
    c->ys = R(s, R_OSD_Y_STRIDE(n));
    c->uvs = R(s, R_OSD_UV_STRIDE(n));
    if (!c->ys) {
        c->ys = c->w * c->bpp;
    }
    if (!c->uvs) {
        c->uvs = c->w;
    }
    if (c->w <= 0 || c->h <= 0 || c->w > IPU_MAX_W || c->h > IPU_MAX_H) {
        return false;
    }
    for (i = 0; i < 4; i++) {
        uint32_t v = R(s, R_CSC(n, i));

        c->c[2 * i] = v & 0xfff;
        c->c[2 * i + 1] = (v >> 20) & 0xfff;
    }
    c->c[8] = R(s, R_CSC(n, 4)) & 0xfff;
    c->yoff = R(s, R_CSC(n, 5)) & 0xff;
    c->uvoff = (R(s, R_CSC(n, 5)) >> 16) & 0xff;
    c->line = g_malloc(c->w * c->bpp);
    c->uvline = g_malloc((c->w + 2) & ~1);
    c->cur_line = c->cur_uvline = -1;
    return true;
}

static void ipu_osd(IngenicIpuState *s, int w, int h)
{
    uint32_t bk = R(s, R_OSD_BK_PARA);
    int bpic = (bk >> 11) & 3;
    hwaddr ya = R(s, R_Y_ADDR), uva = R(s, R_U_ADDR);
    hwaddr oya = R(s, R_OUT_ADDR), ouva = R(s, R_NV_OUT_ADDR);
    uint32_t ys = R(s, R_Y_STRIDE), uvs = R(s, R_UV_STRIDE) >> 16;
    uint32_t oys = R(s, R_OUT_STRIDE), ouvs = R(s, R_NV_OUT_STRIDE);
    int uoff = bpic == PIC_NV21, voff = bpic != PIC_NV21;
    OsdChan ch[4];
    int nch = 0, i, x, y;
    g_autofree uint8_t *yl = g_malloc(w * 2);
    g_autofree uint8_t *uvl = g_malloc(w + 2);

    if (bpic != PIC_NV12 && bpic != PIC_NV21) {
        qemu_log_mask(LOG_UNIMP, "ingenic-ipu: background type %d\n", bpic);
        return;
    }
    ys = ys ? ys : w;
    uvs = uvs ? uvs : w;
    oys = oys ? oys : w;
    ouvs = ouvs ? ouvs : w;
    for (i = 0; i < 4; i++) {
        if (osd_chan_setup(s, i, &ch[nch])) {
            nch++;
        }
    }
    if (s->debug) {
        qemu_log("ingenic-ipu: OSD %dx%d %s %08" HWADDR_PRIx "/%08"
                 HWADDR_PRIx " -> %08" HWADDR_PRIx ", %d channel(s)\n",
                 w, h, bpic == PIC_NV21 ? "NV21" : "NV12", ya, uva, oya, nch);
        for (i = 0; i < nch; i++) {
            qemu_log("ingenic-ipu:   ch%d %dx%d at %d,%d type %d para %08x "
                     "src %08" HWADDR_PRIx " stride %u\n", ch[i].n, ch[i].w,
                     ch[i].h, ch[i].x0, ch[i].y0, ch[i].pic, ch[i].para,
                     ch[i].ya, ch[i].ys);
        }
    }

    /* two background lines at a time: one chroma line covers both */
    for (y = 0; y < h; y += 2) {
        int rows = MIN(2, h - y);
        bool touched = false;

        for (i = 0; i < nch; i++) {
            if (y + rows > ch[i].y0 && y < ch[i].y0 + ch[i].h) {
                touched = true;
            }
        }
        if (!touched && oya == ya && ouva == uva && oys == ys &&
            ouvs == uvs) {
            continue;               /* in place and nothing over it */
        }
        ipu_read(ya + (hwaddr)y * ys, yl, w);
        if (rows > 1) {
            ipu_read(ya + (hwaddr)(y + 1) * ys, yl + w, w);
        }
        ipu_read(uva + (hwaddr)(y / 2) * uvs, uvl, (w + 1) & ~1);

        for (i = 0; i < nch; i++) {
            OsdChan *c = &ch[i];
            int xs = MAX(0, c->x0), xe = MIN(w, c->x0 + c->w);

            if (y + rows <= c->y0 || y >= c->y0 + c->h) {
                continue;
            }
            for (x = xs & ~1; x < xe; x += 2) {
                int usum = 0, vsum = 0, asum = 0, k;

                for (k = 0; k < rows * 2; k++) {
                    int px = x + (k & 1), py = y + (k >> 1);
                    OsdPix p;
                    uint8_t *dy;

                    if (px < xs || px >= xe || py < c->y0 ||
                        py >= c->y0 + c->h || px >= w) {
                        continue;
                    }
                    osd_pixel(c, px - c->x0, py - c->y0, &p);
                    dy = &yl[(k >> 1) * w + px];
                    *dy = (p.y * p.a + *dy * (255 - p.a) + 127) / 255;
                    usum += p.u * p.a;
                    vsum += p.v * p.a;
                    asum += p.a;
                }
                if (asum) {
                    /* the block's mean alpha over its mean colour */
                    int n = rows * 2, a = asum / n;
                    uint8_t *du = &uvl[x + uoff], *dv = &uvl[x + voff];

                    *du = (usum / n + *du * (255 - a) + 127) / 255;
                    *dv = (vsum / n + *dv * (255 - a) + 127) / 255;
                }
            }
        }
        ipu_write(oya + (hwaddr)y * oys, yl, w);
        if (rows > 1) {
            ipu_write(oya + (hwaddr)(y + 1) * oys, yl + w, w);
        }
        ipu_write(ouva + (hwaddr)(y / 2) * ouvs, uvl, (w + 1) & ~1);
    }
    for (i = 0; i < nch; i++) {
        g_free(ch[i].line);
        g_free(ch[i].uvline);
    }
}

/* the conversion mode: NV12 -> NV21 or 32-bit BGRA */
/*
 * NV12 to HSV in three planes (the driver's _ipu_set_bg_buffer for
 * HAL_PIXEL_FORMAT_HSV, libimp's ipu_csc_nv12_to_hsv, the T23 SDK's
 * sample-EXTFS-Encoder-h264.c): H, 16 bits a pixel, at NV_OUT_ADDR
 * (stride NV_OUT_STRIDE, 2 w); S at OUT_ADDR (OUT_STRIDE, w) and V at
 * OUT_V_ADDR (OUT_V_STRIDE, w), a byte each.  H is in degrees, 0..359;
 * S and V are 0..255 -- the usual scales, not confirmed on silicon.
 */
static void ipu_csc_hsv(IngenicIpuState *s, int w, int h, uint8_t *yl,
                        uint8_t *uvl)
{
    hwaddr ya = R(s, R_Y_ADDR), uva = R(s, R_U_ADDR);
    hwaddr ha = R(s, R_NV_OUT_ADDR), sa = R(s, R_OUT_ADDR);
    hwaddr va = R(s, R_OUT_V_ADDR);
    uint32_t ys = R(s, R_Y_STRIDE), uvs = R(s, R_UV_STRIDE) >> 16;
    uint32_t hs = R(s, R_NV_OUT_STRIDE), ss = R(s, R_OUT_STRIDE);
    uint32_t vs = R(s, R_OUT_V_STRIDE);
    g_autofree uint16_t *hl = g_new(uint16_t, w);
    g_autofree uint8_t *sl = g_malloc(w), *vl = g_malloc(w);
    int x, y;

    ys = ys ? ys : w;
    uvs = uvs ? uvs : w;
    hs = hs ? hs : 2 * w;
    ss = ss ? ss : w;
    vs = vs ? vs : w;
    for (y = 0; y < h; y++) {
        ipu_read(ya + (hwaddr)y * ys, yl, w);
        ipu_read(uva + (hwaddr)(y / 2) * uvs, uvl, (w + 1) & ~1);
        for (x = 0; x < w; x++) {
            int c = (yl[x] - 16) * 1192;
            int d = uvl[x & ~1] - 128, e = uvl[(x & ~1) + 1] - 128;
            int r = clamp255((c + 1634 * e) >> 10);
            int g = clamp255((c - 401 * d - 833 * e) >> 10);
            int b = clamp255((c + 2066 * d) >> 10);
            int mx = MAX(r, MAX(g, b)), mn = MIN(r, MIN(g, b));
            int dl = mx - mn, hue = 0;

            if (dl) {
                if (mx == r) {
                    hue = 60 * (g - b) / dl;
                } else if (mx == g) {
                    hue = 120 + 60 * (b - r) / dl;
                } else {
                    hue = 240 + 60 * (r - g) / dl;
                }
                hue = (hue + 360) % 360;
            }
            hl[x] = cpu_to_le16(hue);
            sl[x] = mx ? 255 * dl / mx : 0;
            vl[x] = mx;
        }
        ipu_write(ha + (hwaddr)y * hs, (uint8_t *)hl, 2 * w);
        ipu_write(sa + (hwaddr)y * ss, sl, w);
        ipu_write(va + (hwaddr)y * vs, vl, w);
    }
}

static void ipu_csc(IngenicIpuState *s, int w, int h)
{
    uint32_t dfmt = R(s, R_D_FMT);
    hwaddr ya = R(s, R_Y_ADDR), uva = R(s, R_U_ADDR);
    hwaddr oya = R(s, R_OUT_ADDR), ouva = R(s, R_NV_OUT_ADDR);
    uint32_t ys = R(s, R_Y_STRIDE), uvs = R(s, R_UV_STRIDE) >> 16;
    uint32_t oys = R(s, R_OUT_STRIDE);
    g_autofree uint8_t *yl = g_malloc(w);
    g_autofree uint8_t *uvl = g_malloc(w + 2);
    g_autofree uint8_t *out = g_malloc(w * 4);
    int x, y;

    ys = ys ? ys : w;
    uvs = uvs ? uvs : w;
    if (s->debug) {
        qemu_log("ingenic-ipu: CSC %dx%d D_FMT %08x %08" HWADDR_PRIx
                 " -> %08" HWADDR_PRIx "\n", w, h, dfmt, ya, oya);
    }
    if ((dfmt & 7) == 3) {
        uint32_t ouvs = R(s, R_NV_OUT_STRIDE);

        oys = oys ? oys : w;
        ouvs = ouvs ? ouvs : w;
        for (y = 0; y < h; y++) {
            ipu_read(ya + (hwaddr)y * ys, yl, w);
            ipu_write(oya + (hwaddr)y * oys, yl, w);
            if (!(y & 1)) {
                ipu_read(uva + (hwaddr)(y / 2) * uvs, uvl, w & ~1);
                for (x = 0; x + 1 < w; x += 2) {
                    uint8_t t = uvl[x];

                    uvl[x] = uvl[x + 1];
                    uvl[x + 1] = t;
                }
                ipu_write(ouva + (hwaddr)(y / 2) * ouvs, uvl, w & ~1);
            }
        }
        return;
    }
    if (dfmt & 4) {
        ipu_csc_hsv(s, w, h, yl, uvl);
        return;
    }
    oys = oys ? oys : w * 4;
    for (y = 0; y < h; y++) {
        ipu_read(ya + (hwaddr)y * ys, yl, w);
        ipu_read(uva + (hwaddr)(y / 2) * uvs, uvl, (w + 1) & ~1);
        for (x = 0; x < w; x++) {
            int c = (yl[x] - 16) * 1192;
            int d = uvl[x & ~1] - 128, e = uvl[(x & ~1) + 1] - 128;

            out[4 * x + 0] = clamp255((c + 2066 * d) >> 10);
            out[4 * x + 1] = clamp255((c - 401 * d - 833 * e) >> 10);
            out[4 * x + 2] = clamp255((c + 1634 * e) >> 10);
            out[4 * x + 3] = 0xff;
        }
        ipu_write(oya + (hwaddr)y * oys, out, w * 4);
    }
}

static void ipu_run(IngenicIpuState *s)
{
    uint32_t gs = R(s, R_IN_FM_GS);
    int w = (gs >> 16) & 0x1fff, h = gs & 0x1fff;

    if (w <= 0 || h <= 0 || w > IPU_MAX_W || h > IPU_MAX_H) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-ipu: bad size %dx%d\n", w, h);
    } else if (R(s, R_OSD_BK_PARA) & (1u << 20)) {
        /* CH_BK_WHOLE_PIC_EN: the driver's CSC set-up (0x141000) */
        ipu_csc(s, w, h);
    } else {
        ipu_osd(s, w, h);
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + IPU_DONE_NS);
}

static void ipu_done(void *opaque)
{
    IngenicIpuState *s = opaque;

    s->regs[R_STATUS >> 2] |= STATUS_OUT_END;
    ipu_update_irq(s);
}

static uint64_t ingenic_ipu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIpuState *s = opaque;

    if (off == R_TRIGGER) {
        return 0;                   /* self-clearing */
    }
    return off < IPU_NREGS * 4 ? s->regs[off >> 2] : 0;
}

static void ingenic_ipu_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicIpuState *s = opaque;

    if (off >= IPU_NREGS * 4) {
        return;
    }
    if (off == R_TRIGGER) {
        if (val & TRIG_RESET) {
            timer_del(s->timer);
            memset(s->regs, 0, sizeof(s->regs));
        }
        if (val & TRIG_RUN) {
            ipu_run(s);
        }
    } else {
        s->regs[off >> 2] = val;
    }
    ipu_update_irq(s);
}

static const MemoryRegionOps ingenic_ipu_ops = {
    .read = ingenic_ipu_read,
    .write = ingenic_ipu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_ipu_reset(DeviceState *dev)
{
    IngenicIpuState *s = INGENIC_IPU(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    ipu_update_irq(s);
}

static void ingenic_ipu_init(Object *obj)
{
    IngenicIpuState *s = INGENIC_IPU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_ipu_ops, s,
                          TYPE_INGENIC_IPU, IPU_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ipu_done, s);
}

static void ingenic_ipu_finalize(Object *obj)
{
    IngenicIpuState *s = INGENIC_IPU(obj);

    timer_free(s->timer);
}

static const Property ingenic_ipu_props[] = {
    DEFINE_PROP_BOOL("debug", IngenicIpuState, debug, false),
};

static const VMStateDescription vmstate_ingenic_ipu = {
    .name = TYPE_INGENIC_IPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, IngenicIpuState),
        VMSTATE_UINT32_ARRAY(regs, IngenicIpuState, IPU_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ipu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_ipu_reset);
    device_class_set_props(dc, ingenic_ipu_props);
    dc->vmsd = &vmstate_ingenic_ipu;
}

static const TypeInfo ingenic_ipu_info = {
    .name = TYPE_INGENIC_IPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicIpuState),
    .instance_init = ingenic_ipu_init,
    .instance_finalize = ingenic_ipu_finalize,
    .class_init = ingenic_ipu_class_init,
};

static void ingenic_ipu_register_types(void)
{
    type_register_static(&ingenic_ipu_info);
}

type_init(ingenic_ipu_register_types)
