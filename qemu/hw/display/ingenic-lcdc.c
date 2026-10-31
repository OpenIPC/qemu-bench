/*
 * Ingenic LCDC ("tiny" LCD controller): the XBurst1 parts' display
 * controller at 0x13050000 (T21, T23, T31 on INTC 31, T30 on 5), as the
 * 3.10 kernels' drivers/video/jz_fb_v14 (jz-fb, /dev/fb0; the SDKs'
 * sample_slcd) drive it, TFT or smart (SLCD) panel.
 *
 *   0x0000 CHAIN_ADDR  the first frame descriptor
 *   0x0004 GLB_CFG     [18:16] format: 0 RGB555, 1 RGB565, 2 RGB888 in
 *                      32 bits, 4 NV12, 5 NV21; [21:19] colour order
 *                      (0 RGB .. 5 BGR, from the top bits down)
 *   0x1000 CTRL        bit 0 CHAIN_START (load CHAIN_ADDR), 1 QCK_STOP,
 *                      3 GEN_STOP (after the frame), 4 SLCD_START,
 *                      5 TFT_START
 *   0x1004 ST          0 WORKING, 1 FRM_END, 2 DMA_END, 3 TFT_UNDER,
 *                      5 GEN_STOPM, 6 QCK_STOPM
 *   0x1008 CSR         write 1 to clear ST's bits
 *   0x100C INTC        their interrupt enables; 0x101C INT_FLAG ST & INTC
 *   0x2000 CHAIN_SITE / 0x2004 DMA_SITE  the descriptor / buffer shown
 *   0x8000 DISP_COMMON bit 0 a smart panel (else TFT)
 *   0x9008 / 0x900C    TFT HDE / VDE: start << 16 | end
 *   0xA00C SLCD_FRM_SIZE  height << 16 | width
 *   0xA014 SLCD_CMD    a command (bit 31), parameter or data word for
 *                      the panel; 0xA018 SLCD_ST bit 0 busy (never)
 *   the rest           storage (timings, CSC, panel configuration)
 *
 * A frame descriptor is 7 words: next, buffer (RGB or Y), stride in
 * pixels, chain config (bit 0: go on to the next -- the driver's looping
 * chain sets it, its single-frame mode leaves it 0: inferred), interrupt
 * control, the UV buffer and its stride.  Running (CHAIN_START, or a
 * panel start with the chain loaded), each frame time ("fps", 60) shows
 * the current descriptor's buffer on the display at the panel's size,
 * raises FRM_END and DMA_END, and goes on unless the chain config says
 * stop.  The panel commands are counted ("slcd-cmds") and the last kept
 * ("slcd-last").  The XBurst1 OpenIPC and SDK kernels build no display
 * driver, so the model follows jz_fb_v14's contract and is checked by
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

#define TYPE_INGENIC_LCDC "ingenic-lcdc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicLcdcState, INGENIC_LCDC)

#define LCDC_SIZE       0x10000

#define R_CHAIN_ADDR    0x0000
#define R_GLB_CFG       0x0004
#define R_CTRL          0x1000
#define R_ST            0x1004
#define R_CSR           0x1008
#define R_INTC          0x100C
#define R_INT_FLAG      0x101C
#define R_CHAIN_SITE    0x2000
#define R_DMA_SITE      0x2004
#define R_DISP_COMMON   0x8000
#define R_TFT_HDE       0x9008
#define R_TFT_VDE       0x900C
#define R_SLCD_FRM_SIZE 0xA00C
#define R_SLCD_CMD      0xA014
#define R_SLCD_ST       0xA018

#define CTRL_CHAIN_START (1u << 0)
#define CTRL_QCK_STOP   (1u << 1)
#define CTRL_GEN_STOP   (1u << 3)
#define CTRL_SLCD_START (1u << 4)
#define CTRL_TFT_START  (1u << 5)

#define ST_WORKING      (1u << 0)
#define ST_FRM_END      (1u << 1)
#define ST_DMA_END      (1u << 2)
#define ST_GEN_STOPM    (1u << 5)
#define ST_QCK_STOPM    (1u << 6)
#define ST_W1C          0x6Eu

#define CC_CONTINUE     (1u << 0)
#define MAX_DIM         2048

struct IngenicLcdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    QemuConsole *con;
    uint32_t regs[LCDC_SIZE / 4];
    uint32_t cur;                   /* the descriptor to show next */
    bool loaded;                    /* CHAIN_START has loaded the chain */
    bool stopping;
    uint32_t fps;
    uint32_t frames;
    uint32_t slcd_cmds, slcd_last;
};

#define LREG(s, off)    ((s)->regs[(off) >> 2])

static void lcdc_update(IngenicLcdcState *s)
{
    LREG(s, R_INT_FLAG) = LREG(s, R_ST) & LREG(s, R_INTC);
    qemu_set_irq(s->irq, LREG(s, R_INT_FLAG) != 0);
}

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* the panel's size: SLCD_FRM_SIZE for a smart panel, else the TFT's DE */
static void lcdc_size(IngenicLcdcState *s, int *w, int *h)
{
    uint32_t hde = LREG(s, R_TFT_HDE), vde = LREG(s, R_TFT_VDE);

    if (LREG(s, R_DISP_COMMON) & 1) {
        *w = LREG(s, R_SLCD_FRM_SIZE) & 0xFFF;
        *h = (LREG(s, R_SLCD_FRM_SIZE) >> 16) & 0xFFF;
    } else {
        *w = (int)(hde & 0xFFFF) - (int)(hde >> 16);
        *h = (int)(vde & 0xFFFF) - (int)(vde >> 16);
    }
}

/* r, g, b out of three fields, top first, in colour order @order */
static void lcdc_order(uint32_t order, unsigned hi, unsigned mid,
                       unsigned lo, unsigned *r, unsigned *g, unsigned *b)
{
    static const uint8_t pos[6][3] = {     /* RGB RBG GRB BRG GBR BGR */
        { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 },
        { 2, 1, 0 },
    };
    const unsigned v[3] = { hi, mid, lo };
    const uint8_t *p = pos[order < 6 ? order : 0];

    *r = v[p[0]];
    *g = v[p[1]];
    *b = v[p[2]];
}

static void lcdc_frame(void *opaque)
{
    IngenicLcdcState *s = opaque;
    uint32_t d[7], fmt, order, stride, uvstride;
    int w, h, x, y, bpp;
    DisplaySurface *surf;
    g_autofree uint8_t *line = NULL, *uvline = NULL;

    if (!(LREG(s, R_ST) & ST_WORKING)) {
        return;
    }
    dma_memory_read(&address_space_memory, s->cur, d, sizeof(d),
                    MEMTXATTRS_UNSPECIFIED);
    for (x = 0; x < 7; x++) {
        d[x] = le32_to_cpu(d[x]);
    }
    LREG(s, R_CHAIN_SITE) = s->cur;
    LREG(s, R_DMA_SITE) = d[1];
    lcdc_size(s, &w, &h);
    fmt = (LREG(s, R_GLB_CFG) >> 16) & 7;
    order = (LREG(s, R_GLB_CFG) >> 19) & 7;
    bpp = fmt == 2 ? 4 : fmt >= 4 ? 1 : 2;
    stride = d[2] ? d[2] : w;
    uvstride = d[6] ? d[6] : w;
    if (w > 0 && h > 0 && w <= MAX_DIM && h <= MAX_DIM && fmt != 3 &&
        fmt <= 5) {
        surf = qemu_console_surface(s->con);
        if (!surf || surface_width(surf) != w || surface_height(surf) != h) {
            qemu_console_resize(s->con, w, h);
            surf = qemu_console_surface(s->con);
        }
        line = g_malloc((size_t)w * bpp);
        uvline = g_malloc(w + 2);
        for (y = 0; y < h; y++) {
            uint32_t *row = (uint32_t *)(surface_data(surf) +
                                         y * surface_stride(surf));

            dma_memory_read(&address_space_memory,
                            d[1] + (hwaddr)y * stride * bpp, line,
                            (size_t)w * bpp, MEMTXATTRS_UNSPECIFIED);
            if (bpp == 1) {
                dma_memory_read(&address_space_memory,
                                d[5] + (hwaddr)(y / 2) * uvstride, uvline,
                                (w + 1) & ~1, MEMTXATTRS_UNSPECIFIED);
            }
            for (x = 0; x < w; x++) {
                unsigned r, g, b;
                uint32_t v;

                switch (fmt) {
                case 0:
                    v = lduw_le_p(line + 2 * x);
                    lcdc_order(order, ((v >> 10) & 31) * 255 / 31,
                               ((v >> 5) & 31) * 255 / 31,
                               (v & 31) * 255 / 31, &r, &g, &b);
                    break;
                case 1:
                    v = lduw_le_p(line + 2 * x);
                    lcdc_order(order, ((v >> 11) & 31) * 255 / 31,
                               ((v >> 5) & 63) * 255 / 63,
                               (v & 31) * 255 / 31, &r, &g, &b);
                    break;
                case 2:
                    v = ldl_le_p(line + 4 * x);
                    lcdc_order(order, (v >> 16) & 255, (v >> 8) & 255,
                               v & 255, &r, &g, &b);
                    break;
                default: {          /* NV12 / NV21, BT.601 */
                    int yy = line[x] - 16;
                    int u = uvline[x & ~1] - 128;
                    int vv = uvline[(x & ~1) + 1] - 128;

                    if (fmt == 5) {
                        int t = u;

                        u = vv;
                        vv = t;
                    }
                    r = clamp8((298 * yy + 409 * vv + 128) >> 8);
                    g = clamp8((298 * yy - 100 * u - 208 * vv + 128) >> 8);
                    b = clamp8((298 * yy + 516 * u + 128) >> 8);
                    break;
                }
                }
                row[x] = (r << 16) | (g << 8) | b;
            }
        }
        dpy_gfx_update_full(s->con);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-lcdc: bad frame %dx%d "
                      "format %u\n", w, h, fmt);
    }
    s->frames++;
    LREG(s, R_ST) |= ST_FRM_END | ST_DMA_END;
    if (!(d[3] & CC_CONTINUE) || s->stopping) {
        LREG(s, R_ST) &= ~ST_WORKING;
        if (s->stopping) {
            LREG(s, R_ST) |= ST_GEN_STOPM;
        }
        s->stopping = false;
        s->cur = d[0];
    } else {
        s->cur = d[0];
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
    }
    lcdc_update(s);
}

static void lcdc_run(IngenicLcdcState *s)
{
    s->stopping = false;
    LREG(s, R_ST) |= ST_WORKING;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
}

static uint64_t ingenic_lcdc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicLcdcState *s = opaque;

    if (off == R_SLCD_ST) {
        return 0;                       /* never busy */
    }
    return LREG(s, off);
}

static void ingenic_lcdc_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicLcdcState *s = opaque;

    switch (off) {
    case R_CTRL:
        LREG(s, off) = 0;
        if (val & CTRL_QCK_STOP) {
            timer_del(s->timer);
            if (LREG(s, R_ST) & ST_WORKING) {
                LREG(s, R_ST) |= ST_QCK_STOPM;
            }
            LREG(s, R_ST) &= ~ST_WORKING;
            break;
        }
        if ((val & CTRL_GEN_STOP) && (LREG(s, R_ST) & ST_WORKING)) {
            s->stopping = true;
        }
        if (val & CTRL_CHAIN_START) {
            s->cur = LREG(s, R_CHAIN_ADDR);
            s->loaded = true;
            lcdc_run(s);
        } else if ((val & (CTRL_SLCD_START | CTRL_TFT_START)) &&
                   !(LREG(s, R_ST) & ST_WORKING)) {
            if (!s->loaded) {
                s->cur = LREG(s, R_CHAIN_ADDR);
                s->loaded = true;
            }
            lcdc_run(s);
        }
        break;
    case R_ST:
    case R_INT_FLAG:
    case R_CHAIN_SITE:
    case R_DMA_SITE:
        break;                          /* read only */
    case R_CSR:
        LREG(s, R_ST) &= ~(val & ST_W1C);
        break;
    case R_SLCD_CMD:
        LREG(s, off) = val;
        s->slcd_cmds++;
        s->slcd_last = val;
        break;
    default:
        LREG(s, off) = val;
        break;
    }
    lcdc_update(s);
}

static const MemoryRegionOps ingenic_lcdc_ops = {
    .read = ingenic_lcdc_read,
    .write = ingenic_lcdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const GraphicHwOps ingenic_lcdc_gfx_ops = {
};

static void ingenic_lcdc_reset(DeviceState *dev)
{
    IngenicLcdcState *s = INGENIC_LCDC(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->cur = 0;
    s->loaded = s->stopping = false;
    s->frames = s->slcd_cmds = s->slcd_last = 0;
    lcdc_update(s);
}

static void ingenic_lcdc_realize(DeviceState *dev, Error **errp)
{
    IngenicLcdcState *s = INGENIC_LCDC(dev);

    s->con = graphic_console_init(dev, 0, &ingenic_lcdc_gfx_ops, s);
    qemu_console_resize(s->con, 320, 240);
}

static void ingenic_lcdc_init(Object *obj)
{
    IngenicLcdcState *s = INGENIC_LCDC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_lcdc_ops, s,
                          TYPE_INGENIC_LCDC, LCDC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lcdc_frame, s);
    object_property_add_uint32_ptr(obj, "frames", &s->frames,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "slcd-cmds", &s->slcd_cmds,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "slcd-last", &s->slcd_last,
                                   OBJ_PROP_FLAG_READ);
}

static const Property ingenic_lcdc_props[] = {
    DEFINE_PROP_UINT32("fps", IngenicLcdcState, fps, 60),
};

static const VMStateDescription vmstate_ingenic_lcdc = {
    .name = TYPE_INGENIC_LCDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicLcdcState, LCDC_SIZE / 4),
        VMSTATE_UINT32(cur, IngenicLcdcState),
        VMSTATE_BOOL(loaded, IngenicLcdcState),
        VMSTATE_BOOL(stopping, IngenicLcdcState),
        VMSTATE_UINT32(frames, IngenicLcdcState),
        VMSTATE_UINT32(slcd_cmds, IngenicLcdcState),
        VMSTATE_UINT32(slcd_last, IngenicLcdcState),
        VMSTATE_TIMER_PTR(timer, IngenicLcdcState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_lcdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_lcdc_realize;
    device_class_set_legacy_reset(dc, ingenic_lcdc_reset);
    device_class_set_props(dc, ingenic_lcdc_props);
    dc->vmsd = &vmstate_ingenic_lcdc;
}

static const TypeInfo ingenic_lcdc_info = {
    .name = TYPE_INGENIC_LCDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicLcdcState),
    .instance_init = ingenic_lcdc_init,
    .class_init = ingenic_lcdc_class_init,
};

static void ingenic_lcdc_register_types(void)
{
    type_register_static(&ingenic_lcdc_info);
}

type_init(ingenic_lcdc_register_types)
