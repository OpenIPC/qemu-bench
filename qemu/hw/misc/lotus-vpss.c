/*
 * XMedia "lotus" SPC030 VPSS (Goke GK7206 / xmorca), as the stock
 * Xiongmai firmware's xm_vpss.ko drives it behind VIPROC, online.
 *
 * VPSS (0x11400000, SPI 41 "vpss0").  Like VIPROC it runs register images
 * ("nodes") the driver writes to DDR; online it starts with the VIPROC
 * task (lotus-vi's "vpss-start" / "vpss-early" / "vpss-done" GPIOs).
 * From xm_vpss.ko hal_vpss_isr_run_main / hal_reg_vpss_* and a
 * GK7206V11A camera streaming (devmem):
 *   0x0010  SYS_MASK   1 = masked, not bit-aligned with the status:
 *                      bit 15 masks frame start, 16 frame end, 17 list
 *                      end ("frame done"); hal_vpss_set_int_mask writes
 *                      0xfffe7fff, clean 0xffffffff
 *   0x0014  SYS_INT    write 1 to clear: bits 0..11 early 0..2 of
 *                      ch0 / ch1 / ch3 / start (the ISR takes 0 / 3 / 6:
 *                      low delay, the online encoder may start on the
 *                      lines out), 12 frame start (the ISR latches the
 *                      node id), 13 frame end, 14 list end = frame done
 *                      (VIPROC -> VPSS rendezvous via the slave finish
 *                      notify: the task completes)
 *   0x0018 / 0x001c    overflow mask / status, 0x0020 / 0x0024 error
 *                      mask / status (kept 0); bit-aligned, 1 = masked
 *   0x01e4  EARLY_MASK bits 0..11 mask SYS_INT bits 0..11, 1 = masked
 *                      (hal_vpss_set_ochn_lowdelay writes ~enabled;
 *                      the node's: 0xffe, early 0 only)
 *   0x0160  stage update enables, 0x0168 node id, 0x016c cnfg base
 *           address = node image + 0x160 (the image holds every register
 *           at its own offset), 0x0170 coefficient base, 0x0174 / 0x0178
 *           working / finished node id
 * The picture registers in a node image (hal_reg_vpss_set_*, vpss_node_cfg):
 *   0x1a0  bits 0..2: output channel 0 / 1 / 2 writes luma, 3..5 chroma
 *          (com_chan_switch; channel 2 is "ch3" in the register names)
 *   0x1a4  bit 0 input, 1 / 2 / 3 channel 0 / 1 / 2 UV inversion: set
 *          when the channel's format is YUV_SEMIPLANAR_420 (NV12, format
 *          0); the hardware's own order is NV21
 *   0x1a8  input width bits 11:0, height 27:16
 *   0x290 / 0x2b0 / 0x2d0  channel 0 / 1 / 2 scaler output width bits
 *          12:0, height 28:16
 *   wch00 (channel 0) 0x480 bit 0 flip, 1 mirror, 2 tile, 17:4 Y stride
 *        (bits 17:4 in 8-byte units; a tiled channel's is per 4-line tile
 *        row), 0x484 / 0x488 Y loop (wrap) buffer start / end, 0x48c Y
 *        address, 0x4ac full height (bits 29:16, tile rows when tiled),
 *        0x4c0.. the same for C (0x4c4 / 0x4c8 loop, 0x4cc address)
 *   wch01 (channel 1)  0x500 Y stride, 0x504 Y address, 0x518 height,
 *        0x520 / 0x524 C stride / address
 *   wch03 (channel 2)  0x540 / 0x544 / 0x558 / 0x560 / 0x564
 * The stock firmware runs channel 0 at 3840x2160 TILE_32x4 + TILE_COMPACT
 * through a wrap buffer to the online H.265 encoder (stride 0x2d00 a tile
 * row, ~575 lines of loop) and channel 2 at 800x448 linear NV21 (the sub
 * stream, and VGS's source for ext channels 3 (352x288) and 5 (1920x1080),
 * NV12); channel 1 is off.
 *
 * At the end of each frame every channel the node writes gets the picture:
 * the scene (hisi-scene) as the ISP renders it with the frame's exposure
 * and white balance, at the channel's size, mirrored / flipped as it is
 * set, NV21 or NV12 as the channel is.  The exposure is what the ISP model
 * last told the scene (hisi_scene_set_frame()); when no ISP model does,
 * VPSS takes it from the sensor's registers itself (lines x gain x the
 * scene's light) with an ideal white balance.  Per channel the scene is
 * kept as a grid of reflectance steps and a new exposure re-renders it
 * through hisi_scene_display_lut() (about 20 ms for 4K, which is drawn at
 * half resolution: 2x2 blocks); while nothing changes the picture is only
 * copied, so a still scene keeps the encoder's frames identical.
 * Channels are written linear NV21 / NV12 (a compressed, tiled one too:
 * the encoder model reads them that way).  A channel in wrap mode gets the
 * lines that fit in its loop buffer, top first, at its strides; DDR never
 * holds such a picture whole (the real one streams through the loop), so
 * the encoder model asks for it with lotus_vpss_picture().
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/i2c/image-sensor.h"
#include "hw/misc/hisi-scene.h"
#include "hw/misc/lotus-vpss.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "system/dma.h"
#include "system/address-spaces.h"

#define TYPE_LOTUS_VPSS "lotus-vpss"
OBJECT_DECLARE_SIMPLE_TYPE(LotusVpssState, LOTUS_VPSS)

#define VPSS_SIZE           0x10000
#define VPSS_SYS_MASK       0x010
#define VPSS_SYS_INT        0x014
#define VPSS_OVF_MASK       0x018
#define VPSS_OVF_INT        0x01c
#define VPSS_ERR_MASK       0x020
#define VPSS_ERR_INT        0x024
#define VPSS_EARLY_MASK     0x1e4
#define VPSS_NODE_ID        0x168
#define VPSS_CTRL_CNFG      0x16c
#define VPSS_WORK_NODE      0x174
#define VPSS_FINISH_NODE    0x178
#define VPSS_INT_EARLY0     (1u << 0)
#define VPSS_INT_START      (1u << 12)
#define VPSS_INT_DONE       (1u << 14)
#define VPSS_NODE_CNFG_OFF  0x160
#define VPSS_CHAN_SWITCH    0x1a0
#define VPSS_UV_INV         0x1a4
#define VPSS_IN_SIZE        0x1a8
#define VPSS_NCHN           3
#define VPSS_PIC_MAX        8192

/* a channel's picture as it was last rendered */
typedef struct VpssPic {
    int w, h;
    bool nv12, mirror, flip;
    uint32_t gen;
    double level, tint[3], wb[3];
    int f, gw, gh;              /* the grid: a pixel is f x f of the picture */
    uint16_t *q;                /* its reflectance, VPSS_LUT_N steps */
    uint8_t *y, *c;             /* w x h luma, w x h / 2 chroma */
} VpssPic;

/* reflectance steps (square-root spaced) and the brightest one kept */
#define VPSS_LUT_N          4096
#define VPSS_LUT_PMAX       4.0

struct LotusVpssState {
    SysBusDevice parent_obj;

    MemoryRegion mr;
    qemu_irq irq;
    uint32_t *regs;
    uint32_t node;          /* the running frame's node image (+0x160) */
    uint32_t node_id;
    uint32_t frames;
    uint32_t scene_seq;     /* hisi_scene_frame_seq() at the last frame */
    bool own_exposure;      /* no ISP model describes the frames */
    VpssPic pic[VPSS_NCHN];
    char geom[VPSS_NCHN][160];
    int wrap_chn;           /* the channel in wrap mode, -1 none */
    uint32_t wrap_ya, wrap_ca, wrap_ls, wrap_le, wrap_cls, wrap_cle;
    HisiSceneState *scene;
    Object *sensor;
    bool paint;
    bool trace;
    bool verbose;
};

static LotusVpssState *lotus_vpss_dev;   /* lotus_vpss_picture() */

static const struct {
    uint16_t ys, ya, cs, ca, yh, size, yls, yle, cls, cle;
} vpss_wch[VPSS_NCHN] = {
    { 0x480, 0x48c, 0x4c0, 0x4cc, 0x4ac, 0x290, 0x484, 0x488, 0x4c4, 0x4c8 },
    { 0x500, 0x504, 0x520, 0x524, 0x518, 0x2b0 },
    { 0x540, 0x544, 0x560, 0x564, 0x558, 0x2d0 },
};

static void vpss_trace(LotusVpssState *s, char rw, hwaddr off, uint64_t val)
{
    if (s->trace) {
        qemu_log("%" PRId64 " lotus-vpss %c %05" HWADDR_PRIx " %08" PRIx64
                 "\n", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), rw, off, val);
    }
}

static void vpss_update(LotusVpssState *s)
{
    uint32_t *r = s->regs;
    /* SYS_INT 12..14 are masked by SYS_MASK 15..17, 0..11 by EARLY_MASK */
    uint32_t sys_on = (~r[VPSS_EARLY_MASK >> 2] & 0xfff) |
                      (~r[VPSS_SYS_MASK >> 2] >> 3 & 0x7000);

    qemu_set_irq(s->irq, !!((r[VPSS_SYS_INT >> 2] & sys_on) |
                            (r[VPSS_OVF_INT >> 2] & ~r[VPSS_OVF_MASK >> 2]) |
                            (r[VPSS_ERR_INT >> 2] & ~r[VPSS_ERR_MASK >> 2])));
}

static uint32_t vpss_node_reg(LotusVpssState *s, uint32_t off)
{
    uint32_t v = 0;

    dma_memory_read(&address_space_memory,
                    (hwaddr)s->node - VPSS_NODE_CNFG_OFF + off, &v, 4,
                    MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

/*
 * The frame's exposure.  An ISP model that measures the scene tells it
 * each frame (hisi_scene_set_frame()); while none does, the sensor's
 * registers give the level and the white balance is taken as ideal.
 */
static void vpss_frame_exposure(LotusVpssState *s)
{
    ImageSensorExposure e;
    double tint[3], light;
    const uint32_t seq = hisi_scene_frame_seq(s->scene);

    /* the frame described since the last one: someone else's */
    s->own_exposure = seq == s->scene_seq;
    if (!s->own_exposure) {
        s->scene_seq = seq;
        return;
    }
    light = hisi_scene_light(s->scene, tint);
    if (s->sensor && image_sensor_exposure(s->sensor, &e)) {
        light *= (double)e.lines * e.gain / 1024;
        if (s->verbose && s->frames % 100 == 0) {
            qemu_log("lotus-vpss: exposure lines %u gain %u vts %u -> "
                     "level %.0f\n", e.lines, e.gain, e.vts, light);
        }
    } else {
        light = 65535.0 / 2;
    }
    hisi_scene_set_frame(s->scene, light, tint, 1.0 / tint[0],
                         1.0 / tint[2]);
    s->scene_seq = hisi_scene_frame_seq(s->scene);
}

/*
 * A channel's grid: the scene at the channel's size (at half of it each
 * way for a picture wider than 2048: each grid pixel is then a 2x2 block
 * with one chroma sample), mirrored / flipped as the channel is, kept as
 * reflectance steps so a new exposure is only a table lookup per pixel.
 */
static void vpss_grid(LotusVpssState *s, VpssPic *p)
{
    const float *grid;
    int r, x, c;

    p->f = p->w > 2048 ? 2 : 1;
    p->gw = MAX(p->w / p->f, 1);
    p->gh = MAX(p->h / p->f, 1);
    grid = hisi_scene_grid(s->scene, p->gw, p->gh);
    g_free(p->q);
    p->q = g_new(uint16_t, (size_t)p->gw * p->gh * 3);
    for (r = 0; r < p->gh; r++) {
        const float *g = grid +
                         (size_t)(p->flip ? p->gh - 1 - r : r) * p->gw * 3;
        uint16_t *o = p->q + (size_t)r * p->gw * 3;

        for (x = 0; x < p->gw; x++) {
            const float *px = g + (p->mirror ? p->gw - 1 - x : x) * 3;

            for (c = 0; c < 3; c++) {
                const double v = MIN(MAX(px[c], 0.0f), VPSS_LUT_PMAX);

                o[x * 3 + c] = lrint(sqrt(v / VPSS_LUT_PMAX) *
                                     (VPSS_LUT_N - 1));
            }
        }
    }
}

static inline uint8_t vpss_clip(int v, int lo, int hi)
{
    return MIN(MAX(v, lo), hi);
}

/* the grid through the frame's look (hisi_scene_display_lut()) */
static void vpss_render(LotusVpssState *s, VpssPic *p)
{
    g_autofree int32_t *lutmem = g_new(int32_t, 3 * VPSS_LUT_N * 3);
    int32_t (*lut)[3] = (int32_t (*)[3])lutmem;
    const int f = p->f, cw = p->w & ~1;
    int r, x;

    hisi_scene_display_lut(s->scene, VPSS_LUT_N, VPSS_LUT_PMAX, lut);
    for (r = 0; r < p->gh; r++) {
        const uint16_t *q = p->q + (size_t)r * p->gw * 3;
        uint8_t *y = p->y + (size_t)r * f * p->w;
        /* the chroma row of a 2x2 grid row, or of an even 1x1 one */
        uint8_t *c = (f == 2 || !(r & 1)) && r * f / 2 < p->h / 2 ?
                     p->c + (size_t)(r * f / 2) * p->w : NULL;

        for (x = 0; x < p->gw; x++) {
            const int32_t *l0 = lut[q[x * 3]];
            const int32_t *l1 = lut[VPSS_LUT_N + q[x * 3 + 1]];
            const int32_t *l2 = lut[2 * VPSS_LUT_N + q[x * 3 + 2]];
            const uint8_t yv = 16 + ((l0[0] + l1[0] + l2[0] + 32768) >> 16);

            if (f == 2) {
                y[2 * x] = y[2 * x + 1] = yv;
            } else {
                y[x] = yv;
            }
            if (c && (f == 2 || !(x & 1)) && f * x + 1 < p->w) {
                const int v = (l0[1] + l1[1] + l2[1] + 32768) >> 16;
                const int u = (l0[2] + l1[2] + l2[2] + 32768) >> 16;
                uint8_t *d = c + f * x;

                d[p->nv12] = 128 + vpss_clip(v, -112, 112);
                d[!p->nv12] = 128 + vpss_clip(u, -112, 112);
            }
        }
        if (f == 2 && r * 2 + 1 < p->h) {
            memcpy(y + p->w, y, p->w);
        }
        if (p->w > p->gw * f) {
            /* an odd last column */
            y[p->w - 1] = y[p->w - 2];
            if (f == 2 && r * 2 + 1 < p->h) {
                y[p->w + p->w - 1] = y[p->w - 2];
            }
        }
        if (c && cw < p->w) {
            c[p->w - 1] = 128;
        }
    }
    /* the odd last line(s) a coarser grid leaves */
    for (r = p->gh * f; r < p->h; r++) {
        memcpy(p->y + (size_t)r * p->w, p->y + (size_t)(r - 1) * p->w, p->w);
    }
    for (r = DIV_ROUND_UP(p->gh * f, 2); r < p->h / 2; r++) {
        memcpy(p->c + (size_t)r * p->w, p->c + (size_t)(r - 1) * p->w, p->w);
    }
}

/* the picture for a channel's geometry and this frame's look */
static VpssPic *vpss_pic(LotusVpssState *s, int i, int w, int h, bool nv12,
                         bool mirror, bool flip)
{
    VpssPic *p = &s->pic[i];
    double level = 0, tint[3] = { 0 }, wb[3] = { 0 };
    const uint32_t gen = hisi_scene_generation(s->scene);
    const bool geom = !p->y || p->w != w || p->h != h || p->nv12 != nv12 ||
                      p->mirror != mirror || p->flip != flip || p->gen != gen;

    level = hisi_scene_get_frame(s->scene, tint, wb);
    if (geom) {
        p->w = w;
        p->h = h;
        p->nv12 = nv12;
        p->mirror = mirror;
        p->flip = flip;
        p->gen = gen;
        g_free(p->y);
        g_free(p->c);
        p->y = g_malloc0((size_t)w * h);
        p->c = g_malloc0((size_t)w * MAX(h / 2, 1));
        vpss_grid(s, p);
    }
    if (geom || p->level != level || memcmp(p->tint, tint, sizeof(tint)) ||
        memcmp(p->wb, wb, sizeof(wb))) {
        int64_t t0 = get_clock_realtime();

        p->level = level;
        memcpy(p->tint, tint, sizeof(tint));
        memcpy(p->wb, wb, sizeof(wb));
        vpss_render(s, p);
        if (s->verbose && (geom || s->frames % 100 == 0)) {
            qemu_log("lotus-vpss: ch%d %dx%d%s rendered in %" PRId64
                     " us (level %.0f)\n", i, w, h, nv12 ? " nv12" : "",
                     (get_clock_realtime() - t0) / 1000, level);
        }
    }
    return p;
}

/* no scene: flat grey */
static VpssPic *vpss_pic_grey(LotusVpssState *s, int i, int w, int h)
{
    VpssPic *p = &s->pic[i];

    if (!p->y || p->w != w || p->h != h) {
        p->w = w;
        p->h = h;
        g_free(p->y);
        g_free(p->c);
        p->y = g_malloc((size_t)w * h);
        p->c = g_malloc((size_t)w * MAX(h / 2, 1));
        memset(p->y, 126, (size_t)w * h);
        memset(p->c, 128, (size_t)w * MAX(h / 2, 1));
    }
    return p;
}

static bool vpss_ram(uint32_t a, uint64_t len)
{
    return a >= 0x40000000 && len && len <= 64 * MiB &&
           (uint64_t)a + len <= 0x100000000ull;
}

/* a plane: rows of w bytes from src, stride apart from a */
static void vpss_write_plane(uint32_t a, uint32_t stride, const uint8_t *src,
                             int w, int rows)
{
    if (!vpss_ram(a, (uint64_t)stride * rows)) {
        return;
    }
    if (stride == (uint32_t)w) {
        dma_memory_write(&address_space_memory, a, src, (size_t)w * rows,
                         MEMTXATTRS_UNSPECIFIED);
        return;
    }
    for (int r = 0; r < rows; r++) {
        dma_memory_write(&address_space_memory, a + (uint64_t)r * stride,
                         src + (size_t)r * w, w, MEMTXATTRS_UNSPECIFIED);
    }
}

static void vpss_paint(LotusVpssState *s)
{
    uint32_t sw, inv;

    if (!s->paint || s->node < VPSS_NODE_CNFG_OFF || s->node == 0xffffffff) {
        return;
    }
    if (s->scene) {
        vpss_frame_exposure(s);
    }
    sw = vpss_node_reg(s, VPSS_CHAN_SWITCH);
    inv = vpss_node_reg(s, VPSS_UV_INV);
    s->frames++;
    for (int i = 0; i < VPSS_NCHN; i++) {
        const uint32_t ysr = vpss_node_reg(s, vpss_wch[i].ys);
        const uint32_t ys = ((ysr >> 4) & 0x3fff) * 8;
        const uint32_t cs = ((vpss_node_reg(s, vpss_wch[i].cs) >> 4) &
                             0x3fff) * 8;
        const uint32_t size = vpss_node_reg(s, vpss_wch[i].size);
        /* the scaler's output (a tiled channel's full height is in tiles) */
        const uint32_t h = (size >> 16) & 0x1fff;
        const uint32_t ya = vpss_node_reg(s, vpss_wch[i].ya);
        const uint32_t ca = vpss_node_reg(s, vpss_wch[i].ca);
        const bool nv12 = inv & (2u << i);
        int w = size & 0x1fff;
        VpssPic *p;
        char g[160];

        if (s->verbose) {
            snprintf(g, sizeof(g), "ch%d sw %02x inv %x in %08x out %08x "
                     "ys %08x cs %08x h %08x ya %08x ca %08x loop %08x..%08x",
                     i, sw, inv, vpss_node_reg(s, VPSS_IN_SIZE), size, ysr,
                     vpss_node_reg(s, vpss_wch[i].cs),
                     vpss_node_reg(s, vpss_wch[i].yh), ya, ca,
                     vpss_wch[i].yls ? vpss_node_reg(s, vpss_wch[i].yls) : 0,
                     vpss_wch[i].yle ? vpss_node_reg(s, vpss_wch[i].yle) : 0);
            if ((strcmp(g, s->geom[i]) && s->frames < 50) ||
                s->frames % 500 == 1) {
                pstrcpy(s->geom[i], sizeof(s->geom[i]), g);
                qemu_log("lotus-vpss: frame %u %s\n", s->frames, g);
            }
        }
        if (!(sw & (1u << i)) || !ya || !ys || !h || w < 2 ||
            w > VPSS_PIC_MAX || h > VPSS_PIC_MAX) {
            continue;
        }
        w = MIN((uint32_t)w, ys);
        p = s->scene ? vpss_pic(s, i, w, h, nv12, ysr & 2, ysr & 1)
                     : vpss_pic_grey(s, i, w, h);
        if (vpss_wch[i].yls) {
            s->wrap_ls = vpss_node_reg(s, vpss_wch[i].yls);
            s->wrap_le = vpss_node_reg(s, vpss_wch[i].yle);
            s->wrap_cls = vpss_node_reg(s, vpss_wch[i].cls);
            s->wrap_cle = vpss_node_reg(s, vpss_wch[i].cle);
            if (s->wrap_ls && s->wrap_le > s->wrap_ls) {
                /*
                 * A wrap buffer holds a few lines of the picture at a
                 * time, while the encoder reads them; here only the lines
                 * that fit (from the top, at the channel's strides), the
                 * whole picture through lotus_vpss_picture().
                 */
                vpss_write_plane(ya, ys, p->y, w,
                                 MIN(h, (s->wrap_le - ya + 1) / ys));
                if (ca && (sw & (8u << i)) && s->wrap_cle > ca && cs) {
                    vpss_write_plane(ca, cs, p->c, w,
                                     MIN(h / 2, (s->wrap_cle - ca + 1) / cs));
                }
                s->wrap_ya = ya;
                s->wrap_ca = ca;
                s->wrap_chn = i;
                continue;
            }
        }
        vpss_write_plane(ya, ys, p->y, w, h);
        if (ca && (sw & (8u << i))) {
            vpss_write_plane(ca, cs ? cs : ys, p->c, w, h / 2);
        }
    }
}

bool lotus_vpss_picture(uint32_t y_addr, uint32_t c_addr, int *w, int *h,
                        const uint8_t **y, const uint8_t **c, bool *nv12)
{
    LotusVpssState *s = lotus_vpss_dev;
    VpssPic *p;

    if (!s || s->wrap_chn < 0 || !s->wrap_ya || y_addr != s->wrap_ya ||
        (c_addr && c_addr != s->wrap_ca)) {
        return false;
    }
    p = &s->pic[s->wrap_chn];
    if (!p->y) {
        return false;
    }
    *w = p->w;
    *h = p->h;
    *y = p->y;
    *c = p->c;
    *nv12 = p->nv12;
    return true;
}

static void vpss_vi_start(void *opaque, int n, int level)
{
    LotusVpssState *s = opaque;

    if (level) {
        s->node = s->regs[VPSS_CTRL_CNFG >> 2];
        s->node_id = s->regs[VPSS_NODE_ID >> 2];
        s->regs[VPSS_WORK_NODE >> 2] = s->node_id;
        vpss_trace(s, 'S', 0, s->node);
        s->regs[VPSS_SYS_INT >> 2] |= VPSS_INT_START;
        vpss_update(s);
    }
}

static void vpss_vi_early(void *opaque, int n, int level)
{
    LotusVpssState *s = opaque;

    if (level) {
        vpss_trace(s, 'E', 0, 0);
        s->regs[VPSS_SYS_INT >> 2] |= VPSS_INT_EARLY0;
        vpss_update(s);
    }
}

static void vpss_vi_done(void *opaque, int n, int level)
{
    LotusVpssState *s = opaque;

    if (level) {
        vpss_trace(s, 'D', 0, s->node);
        vpss_paint(s);
        s->regs[VPSS_FINISH_NODE >> 2] = s->node_id;
        s->regs[VPSS_SYS_INT >> 2] |= VPSS_INT_DONE;
        vpss_update(s);
    }
}

static uint64_t vpss_read(void *opaque, hwaddr off, unsigned size)
{
    LotusVpssState *s = opaque;
    uint32_t v = s->regs[off >> 2];

    vpss_trace(s, 'R', off, v);
    return v;
}

static void vpss_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    LotusVpssState *s = opaque;

    vpss_trace(s, 'W', off, val);
    if (off == VPSS_SYS_INT || off == VPSS_OVF_INT || off == VPSS_ERR_INT) {
        s->regs[off >> 2] &= ~val;
        vpss_update(s);
        return;
    }
    s->regs[off >> 2] = val;
    if (off == VPSS_SYS_MASK || off == VPSS_OVF_MASK ||
        off == VPSS_ERR_MASK || off == VPSS_EARLY_MASK) {
        vpss_update(s);
    }
}

static const MemoryRegionOps vpss_ops = {
    .read = vpss_read,
    .write = vpss_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void lotus_vpss_reset(DeviceState *dev)
{
    LotusVpssState *s = LOTUS_VPSS(dev);

    memset(s->regs, 0, VPSS_SIZE);
    s->node = s->node_id = 0;
    s->wrap_chn = -1;
    s->wrap_ya = s->wrap_ca = 0;
    qemu_irq_lower(s->irq);
}

static void lotus_vpss_realize(DeviceState *dev, Error **errp)
{
    LotusVpssState *s = LOTUS_VPSS(dev);

    s->regs = g_new0(uint32_t, VPSS_SIZE / 4);
    memory_region_init_io(&s->mr, OBJECT(s), &vpss_ops, s, "lotus-vpss",
                          VPSS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mr);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    qdev_init_gpio_in_named(dev, vpss_vi_start, "vi-start", 1);
    qdev_init_gpio_in_named(dev, vpss_vi_early, "vi-early", 1);
    qdev_init_gpio_in_named(dev, vpss_vi_done, "vi-done", 1);
    s->wrap_chn = -1;
    lotus_vpss_dev = s;
}

static void lotus_vpss_init(Object *obj)
{
    LotusVpssState *s = LOTUS_VPSS(obj);

    object_property_add_link(obj, "scene", TYPE_HISI_SCENE,
                             (Object **)&s->scene,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    /* the machine creates the sensor after the media blocks */
    object_property_add_link(obj, "sensor", TYPE_OBJECT, &s->sensor,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
}

static void lotus_vpss_finalize(Object *obj)
{
    LotusVpssState *s = LOTUS_VPSS(obj);

    for (int i = 0; i < VPSS_NCHN; i++) {
        g_free(s->pic[i].y);
        g_free(s->pic[i].c);
        g_free(s->pic[i].q);
    }
    g_free(s->regs);
}

static const Property lotus_vpss_properties[] = {
    DEFINE_PROP_BOOL("paint", LotusVpssState, paint, true),
    DEFINE_PROP_BOOL("trace", LotusVpssState, trace, false),
    DEFINE_PROP_BOOL("verbose", LotusVpssState, verbose, false),
};

static void lotus_vpss_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = lotus_vpss_realize;
    device_class_set_legacy_reset(dc, lotus_vpss_reset);
    device_class_set_props(dc, lotus_vpss_properties);
    dc->desc = "XMedia lotus SPC030 VPSS";
}

static const TypeInfo lotus_vpss_info = {
    .name = TYPE_LOTUS_VPSS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LotusVpssState),
    .instance_init = lotus_vpss_init,
    .instance_finalize = lotus_vpss_finalize,
    .class_init = lotus_vpss_class_init,
};

static void lotus_vpss_register(void)
{
    type_register_static(&lotus_vpss_info);
}
type_init(lotus_vpss_register)
