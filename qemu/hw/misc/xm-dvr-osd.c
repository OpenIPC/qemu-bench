/*
 * Xiongmai DVR OSD engine (XM8521DV200 / XM8536E xm_rgn.ko: 0x34000000,
 * 4 KiB, IRQ 89): blends region bitmaps into frames for the encoders.
 *
 * xm_rgn.ko's osd_process: +0x408 0 then 1 resets, +0xF8 1 enables the
 * interrupt, 0 to +0xF4 clears the done flags; region n at +0x100 * n
 * (+0x00 enable, colours at +0x10.., size +0x2C/+0x30, bitmap and frame
 * addresses +0x40..+0x54); a job is +0x064 (the last stage) and +0x068
 * (their order), then 1 to +0x060, which starts it.  It then sleeps until
 * +0xF0 reads non-zero (done), osd_int seeing +0xFC.  Without completions
 * every OSD update times out ("osd: timeout"), which stalls Sofia's
 * start-up long enough that its main thread applies the motion-detect
 * config before the capture thread has created the detectors, and Sofia
 * crashes on the NULL handle.
 *
 * Stage n (0 .. +0x064, in 0x100 blocks; +0x068 is the order, 0x3210)
 * overlays one region in place: +0x00 / +0x04 the mode, 0 / 1 ARGB1555
 * (alpha +0x20: [7:0] for A=1, [15:8] for A=0), 1 / 0 one bit a pixel
 * (AYUV colours +0x08 for 1, +0x0C for 0), 1 / 4 two bits (AYUV palette
 * +0x10..+0x1C); the bitmap at +0x40, +0x30 bytes a line, scaled by
 * +0x24 / +0x28 (+1) to +0x4C x (+0x34 + 1); frame stride +0x2C, source
 * Y / C +0x44 / +0x48, destination +0x50 / +0x54, physical addresses.
 * Sofia uses ARGB1555 on the XM8520D, two bits on the XM8521DV200.  The
 * other modes (2, 3, 4: planar RGB conversions for snapshots) are not
 * modelled.  A job is drawn at its start and completes 100 us later.
 * Every word is RAM-backed.  Read from the vendor module (no public source).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qom/object.h"

#define TYPE_XM_DVR_OSD "xm-dvr-osd"
OBJECT_DECLARE_SIMPLE_TYPE(XmDvrOsdState, XM_DVR_OSD)

#define OSD_SIZE        0x1000
#define OSD_ACK         0xF4
#define OSD_STATUS      0xFC
#define OSD_DONE        0xF0
#define OSD_START       0x060
#define OSD_JOB_NS      (100 * SCALE_US)

struct XmDvrOsdState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[OSD_SIZE / 4];
    unsigned dbg_n;
};

static void xm_dvr_osd_done(void *opaque)
{
    XmDvrOsdState *s = opaque;

    s->regs[OSD_DONE / 4] = 1;
    s->regs[OSD_STATUS / 4] = 1;
    if (s->regs[0xF8 / 4]) {
        qemu_irq_raise(s->irq);
    }
}


static void xm_dvr_osd_log(XmDvrOsdState *s)
{
    unsigned n = s->regs[0x64 / 4] & 3;

    if (s->dbg_n >= 4) {
        return;
    }
    s->dbg_n++;
    for (unsigned st = 0; st <= n; st++) {
        const uint32_t *b = &s->regs[st * 0x40];
        g_autoptr(GString) g = g_string_new(NULL);

        for (unsigned i = 0; i < 0x18; i++) {
            if (b[i]) {
                g_string_append_printf(g, " %02x=%x", i * 4, b[i]);
            }
        }
        qemu_log_mask(LOG_UNIMP, "xm-dvr-osd: job %u stage %u/%u order %x:%s\n",
                      s->dbg_n, st, n, s->regs[0x68 / 4], g->str);
    }
}

static uint8_t osd_mix(uint8_t bg, uint8_t fg, unsigned a)
{
    return bg + ((int)fg - bg) * (int)a / 255;
}

/* RGB555 to AYUV (BT.601, limited range) */
static uint32_t osd_1555(uint16_t px, uint32_t alpha)
{
    const int r = ((px >> 10) & 31) * 255 / 31;
    const int g = ((px >> 5) & 31) * 255 / 31;
    const int b = (px & 31) * 255 / 31;
    const unsigned a = px & 0x8000 ? alpha & 0xff : (alpha >> 8) & 0xff;
    const int y = (66 * r + 129 * g + 25 * b + 128) / 256 + 16;
    const int u = (-38 * r - 74 * g + 112 * b + 128) / 256 + 128;
    const int v = (112 * r - 94 * g - 18 * b + 128) / 256 + 128;

    return a << 24 | y << 16 | u << 8 | v;
}

/* One overlay stage: the bitmap (+0x40, +0x30 bytes a line) scaled by
 * (+0x24 + 1, +0x28 + 1) to +0x4C x (+0x34 + 1) pixels, blended over the
 * frame (stride +0x2C) at Y +0x44 / C +0x48 into Y +0x50 / C +0x54. */
static void xm_dvr_osd_stage(XmDvrOsdState *s, const uint32_t *b)
{
    const uint32_t mode = b[0], fmt = b[1];
    const unsigned hs = (b[0x24 / 4] & 0xf) + 1, vs = (b[0x28 / 4] & 0xf) + 1;
    const unsigned stride = b[0x2c / 4], bstride = b[0x30 / 4];
    const unsigned w = b[0x4c / 4], h = b[0x34 / 4] + 1;
    const hwaddr bmp_pa = b[0x40 / 4];
    const hwaddr sy_pa = b[0x44 / 4], sc_pa = b[0x48 / 4];
    const hwaddr dy_pa = b[0x50 / 4], dc_pa = b[0x54 / 4];
    unsigned bpp;

    if (mode == 0 && fmt == 1) {
        bpp = 16;                       /* ARGB1555, global alpha +0x20 */
    } else if (mode == 1 && fmt == 0) {
        bpp = 1;                        /* colours +0x08 (1), +0x0C (0) */
    } else if (mode == 1 && fmt == 4) {
        bpp = 2;                        /* palette +0x10.. +0x1C */
    } else {
        qemu_log_mask(LOG_UNIMP, "xm-dvr-osd: mode %u format %u\n", mode, fmt);
        return;
    }
    if (!w || !stride || w > stride || h > 4096 || !bstride ||
        bstride > 16384) {
        return;
    }

    g_autofree uint8_t *bmp = g_malloc(bstride);
    g_autofree uint8_t *y = g_malloc(w);
    g_autofree uint8_t *uv = g_malloc(w + 2);
    unsigned last = UINT_MAX;

    for (unsigned oy = 0; oy < h; oy++) {
        const unsigned by = oy / vs;
        const bool chroma = !(oy & 1);

        if (by != last) {
            dma_memory_read(&address_space_memory, bmp_pa + by * bstride,
                            bmp, bstride, MEMTXATTRS_UNSPECIFIED);
            last = by;
        }
        dma_memory_read(&address_space_memory, sy_pa + oy * stride, y, w,
                        MEMTXATTRS_UNSPECIFIED);
        if (chroma) {
            dma_memory_read(&address_space_memory, sc_pa + oy / 2 * stride,
                            uv, w, MEMTXATTRS_UNSPECIFIED);
        }
        for (unsigned ox = 0; ox < w; ox++) {
            const unsigned bx = ox / hs;
            uint32_t c;

            if (bpp == 16) {
                if (bx * 2 + 1 >= bstride) {
                    break;
                }
                c = osd_1555(lduw_le_p(&bmp[bx * 2]), b[0x20 / 4]);
            } else if (bpp == 1) {
                if (bx / 8 >= bstride) {
                    break;
                }
                c = b[(bmp[bx / 8] >> (7 - bx % 8)) & 1 ? 2 : 3];
            } else {
                if (bx / 4 >= bstride) {
                    break;
                }
                c = b[4 + ((bmp[bx / 4] >> (6 - 2 * (bx % 4))) & 3)];
            }
            if (!(c >> 24)) {
                continue;
            }
            y[ox] = osd_mix(y[ox], c >> 16, c >> 24);
            if (chroma) {
                uint8_t *p = &uv[ox & ~1u];

                p[0] = osd_mix(p[0], c >> 8, c >> 24);
                p[1] = osd_mix(p[1], c, c >> 24);
            }
        }
        dma_memory_write(&address_space_memory, dy_pa + oy * stride, y, w,
                         MEMTXATTRS_UNSPECIFIED);
        if (chroma) {
            dma_memory_write(&address_space_memory, dc_pa + oy / 2 * stride,
                             uv, w, MEMTXATTRS_UNSPECIFIED);
        }
    }
}

static void xm_dvr_osd_run(XmDvrOsdState *s)
{
    const unsigned n = s->regs[0x64 / 4] & 3;

    for (unsigned st = 0; st <= n; st++) {
        xm_dvr_osd_stage(s, &s->regs[st * 0x40]);
    }
}

static uint64_t xm_dvr_osd_read(void *opaque, hwaddr off, unsigned size)
{
    XmDvrOsdState *s = opaque;

    return s->regs[off / 4];
}

static void xm_dvr_osd_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    XmDvrOsdState *s = opaque;

    if (off == OSD_ACK) {
        /* the interrupt only: the waiter still has to see +0xF0 */
        s->regs[OSD_STATUS / 4] = 0;
        qemu_irq_lower(s->irq);
        return;
    }
    s->regs[off / 4] = val;
    if (off == OSD_START && (val & 1)) {
        xm_dvr_osd_log(s);
        xm_dvr_osd_run(s);
        s->regs[OSD_DONE / 4] = 0;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + OSD_JOB_NS);
    }
}

static const MemoryRegionOps xm_dvr_osd_ops = {
    .read = xm_dvr_osd_read,
    .write = xm_dvr_osd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void xm_dvr_osd_reset(DeviceState *dev)
{
    XmDvrOsdState *s = XM_DVR_OSD(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timer);
    qemu_irq_lower(s->irq);
}

static void xm_dvr_osd_init(Object *obj)
{
    XmDvrOsdState *s = XM_DVR_OSD(obj);

    memory_region_init_io(&s->iomem, obj, &xm_dvr_osd_ops, s,
                          TYPE_XM_DVR_OSD, OSD_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_dvr_osd_done, s);
}

static void xm_dvr_osd_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_dvr_osd_reset);
}

static const TypeInfo xm_dvr_osd_info = {
    .name = TYPE_XM_DVR_OSD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmDvrOsdState),
    .instance_init = xm_dvr_osd_init,
    .class_init = xm_dvr_osd_class_init,
};

static void xm_dvr_osd_register(void)
{
    type_register_static(&xm_dvr_osd_info);
}

type_init(xm_dvr_osd_register)
