/*
 * Ingenic DPU: the T40/T41 LCD controller at 0x13050000 ("ingenic,t41-dpu"
 * in t41.dtsi, "ingenic,t40-dpu" in the 4.4 kernel's fbdev/ingenic/fb_v12,
 * the /dev/fb0 behind the SDK's sample-LCD).
 *
 * The driver describes each frame in memory and lets the composer walk
 * the chain:
 *
 *   0x0000 FRM_CFG_ADDR   the first frame descriptor
 *   0x0004 FRM_CFG_CTRL   bit 0 START: run the chain
 *   0x2000 CTRL           bit 0 quick stop, bit 3 stop after the frame
 *   0x2004 ST             bit 0 WORKING, 6 stop acknowledge, 9 CMP_END,
 *                         10..13 layer 0..3 end, 14 CMP_START, 17 DISP_END
 *   0x2008 CLR_ST         write 1 to clear those
 *   0x200C INTC           interrupt enables, ST's bit positions
 *   0x2010 INT_FLAG       ST & INTC, what the interrupt handler reads
 *   0x2100 FRM_DES        the descriptor being shown
 *   0x9008 / 0x900C TFT_HDE / VDE, and the rest: storage
 *
 * A frame descriptor is 11 words: next descriptor, size (width [11:0],
 * height [27:16]), control (bit 0: stop after this frame; write-back is
 * not modelled), write-back address and stride, layer 0..3 descriptor
 * addresses, layer enables (bits 4..7) with each layer's z order (2 bits
 * from bit 8, 3 on top), interrupt controls.  A layer descriptor is 12
 * words: size, config (format [19:16] -- 0 RGB555, 1 ARGB1555, 2 RGB565,
 * 4 RGB888 in 32 bits, 5 ARGB8888, 8 NV12, 9 NV21; channel order [12:10],
 * 0 RGB from the top bits down to 5 BGR; global alpha [7:0] with bit 13),
 * buffer, scale target size, rotation, -, position (x [11:0], y
 * [27:16]), the resize coefficients, stride in pixels, the UV buffer and
 * its stride.
 *
 * Each frame period ("fps", 60) the current descriptor's enabled layers
 * are composed in z order onto the display -- a scaled layer drawn at its
 * target size, nearest neighbour; ARGB layers and global alpha blended --
 * then CMP_START, the layers' ends, CMP_END and DISP_END are raised, and
 * the next descriptor follows unless this one said stop.  The "frames"
 * property counts the frames shown.
 *
 * Neither SDK kernel here builds the driver (both carry only the device
 * tree node), so the model follows fb_v12's contract and is checked by
 * qtest.
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
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_INGENIC_DPU "ingenic-dpu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDpuState, INGENIC_DPU)

#define DPU_SIZE        0x10000

#define R_FRM_CFG_ADDR  0x0000
#define R_FRM_CFG_CTRL  0x0004
#define R_CTRL          0x2000
#define R_ST            0x2004
#define R_CLR_ST        0x2008
#define R_INTC          0x200C
#define R_INT_FLAG      0x2010
#define R_FRM_DES       0x2100

#define FRM_START       (1u << 0)
#define CTRL_QCK_STP    (1u << 0)
#define CTRL_GEN_STP    (1u << 3)

#define ST_WORKING      (1u << 0)
#define ST_STOP_ACK     (1u << 6)
#define ST_CMP_END      (1u << 9)
#define ST_LAY_END(n)   (1u << (10 + (n)))
#define ST_CMP_START    (1u << 14)
#define ST_DISP_END     (1u << 17)
#define ST_W1C          0x8003FFC6u

#define MAX_DIM         4096

enum {
    FMT_RGB555 = 0, FMT_ARGB1555 = 1, FMT_RGB565 = 2, FMT_RGB888 = 4,
    FMT_ARGB8888 = 5, FMT_NV12 = 8, FMT_NV21 = 9,
};

struct IngenicDpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    QemuConsole *con;
    uint32_t regs[DPU_SIZE / 4];
    uint32_t cur;                   /* the descriptor to show next */
    bool stopping;                  /* a general stop: after this frame */
    uint32_t fps;
    uint32_t frames;
};

#define DREG(s, off)    ((s)->regs[(off) >> 2])

static void dpu_update(IngenicDpuState *s)
{
    DREG(s, R_INT_FLAG) = DREG(s, R_ST) & DREG(s, R_INTC);
    qemu_set_irq(s->irq, DREG(s, R_INT_FLAG) != 0);
}

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static void dpu_words(uint32_t addr, uint32_t *w, int n)
{
    int i;

    dma_memory_read(&address_space_memory, addr, w, 4 * n,
                    MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < n; i++) {
        w[i] = le32_to_cpu(w[i]);
    }
}

/* r, g, b out of three 8-bit fields, top first, in channel order @order */
static void dpu_order(uint32_t order, unsigned hi, unsigned mid, unsigned lo,
                      unsigned *r, unsigned *g, unsigned *b)
{
    static const uint8_t pos[6][3] = {     /* r, g, b: 0 top .. 2 bottom */
        { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 2, 0, 1 }, { 1, 2, 0 },
        { 2, 1, 0 },
    };
    const unsigned v[3] = { hi, mid, lo };
    const uint8_t *p = pos[order < 6 ? order : 0];

    *r = v[p[0]];
    *g = v[p[1]];
    *b = v[p[2]];
}

/* One source pixel of a layer as r, g, b, a */
static void dpu_pixel(uint32_t fmt, uint32_t order, const uint8_t *line,
                      const uint8_t *uvline, int x, unsigned *r, unsigned *g,
                      unsigned *b, unsigned *a)
{
    uint32_t v;

    *a = 255;
    switch (fmt) {
    case FMT_RGB555:
    case FMT_ARGB1555:
        v = lduw_le_p(line + 2 * x);
        dpu_order(order, ((v >> 10) & 31) * 255 / 31,
                  ((v >> 5) & 31) * 255 / 31, (v & 31) * 255 / 31, r, g, b);
        if (fmt == FMT_ARGB1555) {
            *a = v & 0x8000 ? 255 : 0;
        }
        break;
    case FMT_RGB565:
        v = lduw_le_p(line + 2 * x);
        dpu_order(order, ((v >> 11) & 31) * 255 / 31,
                  ((v >> 5) & 63) * 255 / 63, (v & 31) * 255 / 31, r, g, b);
        break;
    case FMT_RGB888:
    case FMT_ARGB8888:
        v = ldl_le_p(line + 4 * x);
        dpu_order(order, (v >> 16) & 255, (v >> 8) & 255, v & 255, r, g, b);
        if (fmt == FMT_ARGB8888) {
            *a = v >> 24;
        }
        break;
    case FMT_NV12:
    case FMT_NV21: {
        int yy = line[x] - 16;
        int u = uvline[x & ~1] - 128, vv = uvline[(x & ~1) + 1] - 128;

        if (fmt == FMT_NV21) {
            int t = u;

            u = vv;
            vv = t;
        }
        *r = clamp8((298 * yy + 409 * vv + 128) >> 8);
        *g = clamp8((298 * yy - 100 * u - 208 * vv + 128) >> 8);
        *b = clamp8((298 * yy + 516 * u + 128) >> 8);
        break;
    }
    default:
        *r = *g = *b = 0;
        break;
    }
}

static int dpu_bpp(uint32_t fmt)
{
    switch (fmt) {
    case FMT_RGB555: case FMT_ARGB1555: case FMT_RGB565:
        return 2;
    case FMT_RGB888: case FMT_ARGB8888:
        return 4;
    case FMT_NV12: case FMT_NV21:
        return 1;
    }
    return 0;
}

/* Layer descriptor @addr onto the frame @fb of @fw x @fh */
static void dpu_layer(uint32_t addr, uint32_t *fb, int fw, int fh)
{
    uint32_t d[12];
    uint32_t sw, sh, tw, th, fmt, order, ga, stride, uvstride;
    int px, py, x, y, bpp;
    bool galpha;
    g_autofree uint8_t *line = NULL, *uvline = NULL;

    dpu_words(addr, d, 12);
    sw = d[0] & 0xFFF;
    sh = (d[0] >> 16) & 0xFFF;
    fmt = (d[1] >> 16) & 15;
    order = (d[1] >> 10) & 7;
    galpha = d[1] & (1u << 13);
    ga = d[1] & 255;
    tw = d[3] & 0xFFF;
    th = (d[3] >> 16) & 0xFFF;
    px = d[6] & 0xFFF;
    py = (d[6] >> 16) & 0xFFF;
    stride = d[9];
    uvstride = d[11];
    bpp = dpu_bpp(fmt);
    if (!sw || !sh || !bpp) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-dpu: bad layer %ux%u fmt %u\n",
                      sw, sh, fmt);
        return;
    }
    if (!tw || !th) {
        tw = sw;
        th = sh;
    }
    line = g_malloc((size_t)stride * bpp + 4 * sw);
    uvline = g_malloc((size_t)uvstride + 2 * sw);
    for (y = 0; y < (int)th && py + y < fh; y++) {
        uint32_t sy = (uint64_t)y * sh / th;
        uint32_t *row = fb + (size_t)(py + y) * fw;

        dma_memory_read(&address_space_memory,
                        d[2] + (hwaddr)sy * stride * bpp, line,
                        (size_t)sw * bpp, MEMTXATTRS_UNSPECIFIED);
        if (bpp == 1) {
            dma_memory_read(&address_space_memory,
                            d[10] + (hwaddr)(sy / 2) * uvstride, uvline,
                            (sw + 1) & ~1, MEMTXATTRS_UNSPECIFIED);
        }
        for (x = 0; x < (int)tw && px + x < fw; x++) {
            unsigned r, g, b, a, dr, dg, db;
            uint32_t o = row[px + x];

            dpu_pixel(fmt, order, line, uvline, (uint64_t)x * sw / tw,
                      &r, &g, &b, &a);
            if (galpha) {
                a = a * ga / 255;
            }
            dr = (o >> 16) & 255;
            dg = (o >> 8) & 255;
            db = o & 255;
            r = (r * a + dr * (255 - a)) / 255;
            g = (g * a + dg * (255 - a)) / 255;
            b = (b * a + db * (255 - a)) / 255;
            row[px + x] = (r << 16) | (g << 8) | b;
        }
    }
}

/* Show the current frame descriptor and move on */
static void dpu_frame(void *opaque)
{
    IngenicDpuState *s = opaque;
    uint32_t f[11];
    int w, h, z, n;
    DisplaySurface *surf;
    g_autofree uint32_t *fb = NULL;

    if (!(DREG(s, R_ST) & ST_WORKING)) {
        return;
    }
    dpu_words(s->cur, f, 11);
    DREG(s, R_FRM_DES) = s->cur;
    w = f[1] & 0xFFF;
    h = (f[1] >> 16) & 0xFFF;
    DREG(s, R_ST) |= ST_CMP_START;
    if (w && h && w <= MAX_DIM && h <= MAX_DIM) {
        fb = g_new0(uint32_t, (size_t)w * h);
        for (z = 0; z < 4; z++) {
            for (n = 0; n < 4; n++) {
                if ((f[9] & (1u << (4 + n))) &&
                    extract32(f[9], 8 + 2 * n, 2) == z) {
                    dpu_layer(f[5 + n], fb, w, h);
                    DREG(s, R_ST) |= ST_LAY_END(n);
                }
            }
        }
        surf = qemu_console_surface(s->con);
        if (!surf || surface_width(surf) != w || surface_height(surf) != h) {
            qemu_console_resize(s->con, w, h);
            surf = qemu_console_surface(s->con);
        }
        for (n = 0; n < h; n++) {
            memcpy(surface_data(surf) + n * surface_stride(surf),
                   fb + (size_t)n * w, 4 * w);
        }
        dpy_gfx_update_full(s->con);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-dpu: bad frame %dx%d\n", w, h);
    }
    s->frames++;
    DREG(s, R_ST) |= ST_CMP_END | ST_DISP_END;
    if ((f[2] & 1) || s->stopping) {
        DREG(s, R_ST) &= ~ST_WORKING;
        if (s->stopping) {
            DREG(s, R_ST) |= ST_STOP_ACK;
        }
        s->stopping = false;
    } else {
        s->cur = f[0];
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
    }
    dpu_update(s);
}

static uint64_t ingenic_dpu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicDpuState *s = opaque;

    return DREG(s, off);
}

static void ingenic_dpu_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicDpuState *s = opaque;

    switch (off) {
    case R_FRM_CFG_CTRL:
        DREG(s, off) = val & ~FRM_START;
        if (val & FRM_START) {
            s->cur = DREG(s, R_FRM_CFG_ADDR);
            s->stopping = false;
            DREG(s, R_ST) |= ST_WORKING;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
        }
        break;
    case R_CTRL:
        DREG(s, off) = 0;
        if (val & CTRL_QCK_STP) {
            timer_del(s->timer);
            DREG(s, R_ST) &= ~ST_WORKING;
        } else if ((val & CTRL_GEN_STP) && (DREG(s, R_ST) & ST_WORKING)) {
            s->stopping = true;
        }
        break;
    case R_ST:
    case R_INT_FLAG:
        break;                          /* read only */
    case R_CLR_ST:
        DREG(s, R_ST) &= ~(val & ST_W1C);
        break;
    default:
        DREG(s, off) = val;
        break;
    }
    dpu_update(s);
}

static const MemoryRegionOps ingenic_dpu_ops = {
    .read = ingenic_dpu_read,
    .write = ingenic_dpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const GraphicHwOps ingenic_dpu_gfx_ops = {
};

static void ingenic_dpu_reset(DeviceState *dev)
{
    IngenicDpuState *s = INGENIC_DPU(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->cur = 0;
    s->stopping = false;
    s->frames = 0;
    dpu_update(s);
}

static void ingenic_dpu_realize(DeviceState *dev, Error **errp)
{
    IngenicDpuState *s = INGENIC_DPU(dev);

    s->con = graphic_console_init(dev, 0, &ingenic_dpu_gfx_ops, s);
    qemu_console_resize(s->con, 640, 480);
}

static void ingenic_dpu_init(Object *obj)
{
    IngenicDpuState *s = INGENIC_DPU(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_dpu_ops, s,
                          TYPE_INGENIC_DPU, DPU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dpu_frame, s);
    object_property_add_uint32_ptr(obj, "frames", &s->frames,
                                   OBJ_PROP_FLAG_READ);
}

static const Property ingenic_dpu_props[] = {
    DEFINE_PROP_UINT32("fps", IngenicDpuState, fps, 60),
};

static const VMStateDescription vmstate_ingenic_dpu = {
    .name = TYPE_INGENIC_DPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicDpuState, DPU_SIZE / 4),
        VMSTATE_UINT32(cur, IngenicDpuState),
        VMSTATE_BOOL(stopping, IngenicDpuState),
        VMSTATE_UINT32(frames, IngenicDpuState),
        VMSTATE_TIMER_PTR(timer, IngenicDpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_dpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_dpu_realize;
    device_class_set_legacy_reset(dc, ingenic_dpu_reset);
    device_class_set_props(dc, ingenic_dpu_props);
    dc->vmsd = &vmstate_ingenic_dpu;
}

static const TypeInfo ingenic_dpu_info = {
    .name = TYPE_INGENIC_DPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDpuState),
    .instance_init = ingenic_dpu_init,
    .class_init = ingenic_dpu_class_init,
};

static void ingenic_dpu_register_types(void)
{
    type_register_static(&ingenic_dpu_info);
}

type_init(ingenic_dpu_register_types)
