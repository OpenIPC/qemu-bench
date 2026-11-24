/*
 * Grain Media GM8126 video capture (FTCAP020, the driver's "fcap200").
 *
 * Two instances, 0x90B00000 and 0x90C00000, two output paths each.  The
 * vendor driver (fcap_common.ko, link-list mode) puts one list of 32-bit
 * words per frame in DRAM and the head of each path's chain in 0x280 /
 * 0x284 (address >> 2):
 *
 *   word 0           status, written by the hardware
 *   01 aaaa...       extra DMA address (multi-DMA; ignored)
 *   10 aaaa...       DMA address >> 2; three of them, Y / U / V
 *   00 be<<16 off    register update with byte enables, data in next word
 *   11 aaaa...       next list >> 2, end of this one
 *
 * At each frame start the head list's updates are applied, the frame is
 * captured, the status word gets frame-done (bits 31 and 28) and the head
 * moves on.  A dummy list that names itself keeps the chain alive.
 *
 *   0x00  control, bit 31 enable
 *   0x04  path enables   0x08 bit 28 software reset (self clearing)
 *   0x38  per path (byte) the DMA order / planar format
 *   0x40  per path (halfword): low five bits all ones = path off
 *   0x10 / 0x14 / 0x18 (+0x10 per path) scaled size, crop position and
 *         crop size, w | h << 16: the crop size is what is written
 *   0x54 / 0x58 (+0x10 per path) the active window of the input
 *   0x218 / 0x21C  line pitch / 4
 *   0x3E0 interrupt status (w1c), 0x3E4 mask (1 = masked): bit 4 frame
 *         start, the driver's clock -- it looks at the statuses there
 *   0x3E8 frame count, 0x3EC machine status (bit 28 busy, kept clear)
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include <math.h>
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include "aw-vinc-logo.h"
#include "gm-pattern.c.inc"

#define TYPE_GM_CAP020 "gm-cap020"
OBJECT_DECLARE_SIMPLE_TYPE(GmCap020State, GM_CAP020)

#define CAP_SIZE        0x1000
#define CAP_NREGS       (0x400 / 4)
#define CAP_PATHS       2
#define CAP_MAX_WORDS   256

#define R_CTRL          0x000
#define R_RESET         0x008
#define R_FORMAT        0x038
#define R_PATH_OFF      0x040
#define R_LIST          0x280
#define R_INT           0x3E0
#define R_MASK          0x3E4
#define R_FRAMES        0x3E8
#define R_MACH          0x3EC

#define INT_FRAME_START (1u << 4)
#define ST_DONE         ((1u << 31) | (1u << 28) | (1u << 25)) /* frame */

struct GmCap020State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t fps;

    uint32_t regs[CAP_NREGS];
    uint32_t frames;
};

static void cap_update(GmCap020State *s)
{
    qemu_set_irq(s->irq, (s->regs[R_INT / 4] & ~s->regs[R_MASK / 4]) != 0);
}

static uint32_t cap_ld(uint32_t a)
{
    uint32_t v = 0;

    dma_memory_read(&address_space_memory, a, &v, 4, MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

static void cap_st(uint32_t a, uint32_t v)
{
    v = cpu_to_le32(v);
    dma_memory_write(&address_space_memory, a, &v, 4, MEMTXATTRS_UNSPECIFIED);
}

static void cap_out(hwaddr a, const void *p, size_t n)
{
    dma_memory_write(&address_space_memory, a, p, n, MEMTXATTRS_UNSPECIFIED);
}

/* Y, Cb, Cr of picture pixel (x, y) of a w x h picture */
static void cap_pixel(GmCap020State *s, int x, int y, int w, int h,
                      uint8_t *yuv)
{
    const uint32_t t = (s->frames * 8) % (2 * PAT_DOT_RANGE);
    const int dot = PAT_DOT_X0 + (t < PAT_DOT_RANGE ? t
                                                    : 2 * PAT_DOT_RANGE - t);

    gm_vcap_pixel((int64_t)x * PAT_W / w, (int64_t)y * PAT_H / h, dot, yuv);
}

static void cap_paint(GmCap020State *s, int p, const uint32_t *dma)
{
    const uint32_t size = s->regs[(0x18 + 0x10 * p) / 4];
    const uint32_t fmt = extract32(s->regs[R_FORMAT / 4], 8 * p, 8);
    const uint32_t pitch = s->regs[(0x218 + 4 * p) / 4] * 4;
    const int w = extract32(size, 0, 12) & ~1, h = extract32(size, 16, 12);
    int x, y, i, j;

    if (w < 2 || h < 2 || !dma[0]) {
        return;
    }
    if (!gm_vcap_logo) {
        gm_vcap_logo_init();
    }
    if ((fmt & 7) == 7 || (fmt & 7) == 6) {     /* 4:2:0 macroblock tiles */
        const bool three = (fmt & 7) == 6;
        int mbw = w / 16, mbp = MAX(pitch / 16, mbw);

        for (y = 0; y + 16 <= h; y += 16) {
            for (x = 0; x + 16 <= w; x += 16) {
                uint8_t luma[256], c[128];
                uint32_t mb = (y / 16) * mbp + x / 16;

                for (j = 0; j < 16; j++) {
                    for (i = 0; i < 16; i++) {
                        uint8_t v[3];

                        cap_pixel(s, x + i, y + j, w, h, v);
                        luma[j * 16 + i] = v[0];
                        if (!(i & 1) && !(j & 1)) {
                            int k = (j / 2) * 8 + i / 2;

                            if (three) {
                                c[k] = v[1];
                                c[64 + k] = v[2];
                            } else {
                                c[(j / 2) * 16 + i] = v[1];
                                c[(j / 2) * 16 + i + 1] = v[2];
                            }
                        }
                    }
                }
                cap_out(dma[0] + mb * 256, luma, 256);
                if (three) {
                    cap_out(dma[1] + mb * 64, c, 64);
                    cap_out(dma[2] + mb * 64, c + 64, 64);
                } else {
                    cap_out(dma[1] + mb * 128, c, 128);
                }
            }
        }
    } else {                                    /* packed 4:2:2, UYVY */
        g_autofree uint8_t *line = g_malloc(w * 2);

        for (y = 0; y < h; y++) {
            for (x = 0; x + 1 < w; x += 2) {
                uint8_t a[3], b[3];

                cap_pixel(s, x, y, w, h, a);
                cap_pixel(s, x + 1, y, w, h, b);
                line[2 * x] = (a[1] + b[1]) / 2;
                line[2 * x + 1] = a[0];
                line[2 * x + 2] = (a[2] + b[2]) / 2;
                line[2 * x + 3] = b[0];
            }
            cap_out(dma[0] + (hwaddr)y * MAX(pitch, w * 2), line, w * 2);
        }
    }
}

static void cap_path(GmCap020State *s, int p)
{
    uint32_t list = s->regs[R_LIST / 4 + p] << 2;
    uint32_t dma[3] = { 0 };
    int i, n = 0;

    if (!list) {
        return;
    }
    if (cap_ld(list) & (1u << 31)) {
        return;                                 /* not collected yet */
    }
    for (i = 1; i < CAP_MAX_WORDS; i++) {
        uint32_t w = cap_ld(list + 4 * i);

        switch (w >> 30) {
        case 0: {
            uint32_t v = cap_ld(list + 4 * ++i);
            uint32_t off = w & 0xFFFC, be = extract32(w, 16, 4), m = 0;
            int b;

            for (b = 0; b < 4; b++) {
                m |= be & (1u << b) ? 0xFFu << (8 * b) : 0;
            }
            if (off / 4 < CAP_NREGS) {
                s->regs[off / 4] = (s->regs[off / 4] & ~m) | (v & m);
            }
            break;
        }
        case 1:
            break;
        case 2:
            if (n < 3) {
                dma[n++] = w << 2;
            }
            break;
        case 3:
            if ((extract32(s->regs[R_PATH_OFF / 4], 16 * p, 5)) != 0x1F) {
                cap_paint(s, p, dma);
            }
            cap_st(list, ST_DONE);
            s->regs[R_LIST / 4 + p] = w & 0x3FFFFFFF;
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "gm-cap020: unterminated list 0x%08x\n",
                  list);
}

static void cap_tick(void *opaque)
{
    GmCap020State *s = opaque;

    if (!(s->regs[R_CTRL / 4] & (1u << 31))) {
        return;
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / s->fps);
    cap_path(s, 0);
    cap_path(s, 1);
    s->frames++;
    s->regs[R_FRAMES / 4] = s->frames & 0xFFFF;
    s->regs[R_INT / 4] |= INT_FRAME_START;
    cap_update(s);
}

static uint64_t cap_read(void *opaque, hwaddr off, unsigned size)
{
    GmCap020State *s = opaque;

    return off / 4 < CAP_NREGS ? s->regs[off / 4] : 0;
}

static void cap_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    GmCap020State *s = opaque;

    if (off / 4 >= CAP_NREGS) {
        return;
    }
    switch (off) {
    case R_CTRL:
        s->regs[0] = val;
        if ((val & (1u << 31)) && !timer_pending(s->timer)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      NANOSECONDS_PER_SECOND / s->fps);
        }
        break;
    case R_RESET:
        s->regs[off / 4] = val & ~(1u << 28);
        if (val & (1u << 28)) {
            s->regs[R_INT / 4] = 0;
        }
        break;
    case R_INT:
        s->regs[off / 4] &= ~val;
        break;
    case R_FRAMES:
    case R_MACH:
        break;
    default:
        s->regs[off / 4] = val;
    }
    cap_update(s);
}

static const MemoryRegionOps cap_ops = {
    .read = cap_read,
    .write = cap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void cap_reset(DeviceState *dev)
{
    GmCap020State *s = GM_CAP020(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_MASK / 4] = 0xFFFFFFFF;
    s->frames = 0;
    cap_update(s);
}

static void cap_realize(DeviceState *dev, Error **errp)
{
    GmCap020State *s = GM_CAP020(dev);

    if (!s->fps || s->fps > 120) {
        error_setg(errp, "gm-cap020: fps must be 1..120");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &cap_ops, s, TYPE_GM_CAP020,
                          CAP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cap_tick, s);
}

static const Property cap_props[] = {
    DEFINE_PROP_UINT32("fps", GmCap020State, fps, 30),
};

static const VMStateDescription vmstate_cap = {
    .name = TYPE_GM_CAP020,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmCap020State, CAP_NREGS),
        VMSTATE_UINT32(frames, GmCap020State),
        VMSTATE_TIMER_PTR(timer, GmCap020State),
        VMSTATE_END_OF_LIST()
    }
};

static void cap_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = cap_realize;
    device_class_set_legacy_reset(dc, cap_reset);
    device_class_set_props(dc, cap_props);
    dc->vmsd = &vmstate_cap;
}

static const TypeInfo cap_info = {
    .name = TYPE_GM_CAP020,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GmCap020State),
    .class_init = cap_class_init,
};

static void cap_register_types(void)
{
    type_register_static(&cap_info);
}

type_init(cap_register_types)
