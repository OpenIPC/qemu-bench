/*
 * Ingenic I2D ("ingenic,t40-i2d" / "t41-i2d", 0x130B0000, core INTC
 * source 45 on T40 and T41): rotates, mirrors and flips a picture into
 * another buffer.  libimp uses it for a frame channel's rotation
 * (IMP_FrameSource_SetI2dAttr, i2d_init from IMP_FrameSource_EnableChn);
 * the kernel driver is drivers/video/ingenic_i2d/ingenic_i2d.c, /dev/i2d,
 * whose register programming is the same on both parts.
 *
 *   0x00 RTL_VERSION        0x08 SHD_CTRL (storage)
 *   0x10 CTRL     bit 0 START (clears when done), 4 SAFE_RESET and
 *                 16 RESET (clear at once)
 *   0x14 IMG_SIZE width [31:16] | height [15:0], the source's
 *   0x18 IMG_MODE [5:4] data: 0 NV12, 1 RAW8, 2 RGB565, 3 ARGB8888;
 *                 [3:2] rotation 0, 90, 180, 270 degrees clockwise;
 *                 bit 1 mirror (left-right), bit 0 flip (top-bottom)
 *   0x20/0x24 SRC_ADDR_Y/UV  0x28/0x2C SRC_Y/UV_STRID (bytes)
 *   0x30/0x34 DST_ADDR_Y/UV  0x38/0x3C DST_Y/UV_STRID (bytes)
 *   0x80 IRQ_STATE bit 0 frame done, 1 timeout; 0x84 IRQ_CLEAR (write 1);
 *   0x88 IRQ_MASK (1 masks); 0x90.. timeout, clock gate: storage
 *
 * START transforms the picture at once (NV12: the Y plane as 1-byte and
 * the CbCr plane as 2-byte elements at half size) and raises frame done
 * a virtual 100 us later.  The destination is h x w for 90 and 270.  The
 * mirror and flip apply to the rotated picture; the driver sets either a
 * rotation or a mirror/flip, so their order is not exercised.
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
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_INGENIC_I2D "ingenic-i2d"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicI2dState, INGENIC_I2D)

#define R_VERSION       0x00
#define R_CTRL          0x10
#define R_IMG_SIZE      0x14
#define R_IMG_MODE      0x18
#define R_SRC_Y         0x20
#define R_SRC_UV        0x24
#define R_SRC_Y_STRIDE  0x28
#define R_SRC_UV_STRIDE 0x2C
#define R_DST_Y         0x30
#define R_DST_UV        0x34
#define R_DST_Y_STRIDE  0x38
#define R_DST_UV_STRIDE 0x3C
#define R_IRQ_STATE     0x80
#define R_IRQ_CLEAR     0x84
#define R_IRQ_MASK      0x88
#define I2D_REGS        0x100

#define CTRL_START      (1u << 0)
#define CTRL_SAFE_RESET (1u << 4)
#define CTRL_RESET      (1u << 16)
#define IRQ_DONE        (1u << 0)

#define DONE_NS         (100 * SCALE_US)
#define MAX_DIM         4096

struct IngenicI2dState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[I2D_REGS / 4];
};

static uint32_t *i2d_reg(IngenicI2dState *s, hwaddr off)
{
    return &s->regs[off >> 2];
}

static void i2d_update(IngenicI2dState *s)
{
    qemu_set_irq(s->irq, (*i2d_reg(s, R_IRQ_STATE) &
                          ~*i2d_reg(s, R_IRQ_MASK) & 3) != 0);
}

/*
 * One plane of w x h elements of es bytes: source row r, column c goes
 * to the destination position the rotation, mirror and flip give it.
 */
static void i2d_plane(hwaddr src, uint32_t sstride, hwaddr dst,
                      uint32_t dstride, int w, int h, int es, int rot,
                      bool mirror, bool flip)
{
    int dw = (rot & 1) ? h : w, dh = (rot & 1) ? w : h;
    g_autofree uint8_t *in = g_malloc((size_t)w * es);
    g_autofree uint8_t *out = g_malloc0((size_t)dw * dh * es);
    int r, c, x, y;

    for (r = 0; r < h; r++) {
        dma_memory_read(&address_space_memory, src + (hwaddr)r * sstride, in,
                        (size_t)w * es, MEMTXATTRS_UNSPECIFIED);
        for (c = 0; c < w; c++) {
            switch (rot) {
            case 0:
                x = c;
                y = r;
                break;
            case 1:                     /* 90 clockwise */
                x = h - 1 - r;
                y = c;
                break;
            case 2:
                x = w - 1 - c;
                y = h - 1 - r;
                break;
            default:                    /* 270 clockwise */
                x = r;
                y = w - 1 - c;
                break;
            }
            if (mirror) {
                x = dw - 1 - x;
            }
            if (flip) {
                y = dh - 1 - y;
            }
            memcpy(out + ((size_t)y * dw + x) * es, in + (size_t)c * es, es);
        }
    }
    for (y = 0; y < dh; y++) {
        dma_memory_write(&address_space_memory, dst + (hwaddr)y * dstride,
                         out + (size_t)y * dw * es, (size_t)dw * es,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

static void i2d_run(IngenicI2dState *s)
{
    uint32_t size = *i2d_reg(s, R_IMG_SIZE), mode = *i2d_reg(s, R_IMG_MODE);
    int w = size >> 16, h = size & 0xffff;
    int type = extract32(mode, 4, 2), rot = extract32(mode, 2, 2);
    bool mirror = mode & 2, flip = mode & 1;
    static const int es[] = { 1, 1, 2, 4 };

    if (!w || !h || w > MAX_DIM || h > MAX_DIM) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-i2d: bad size %dx%d\n", w, h);
        return;
    }
    i2d_plane(*i2d_reg(s, R_SRC_Y), *i2d_reg(s, R_SRC_Y_STRIDE),
              *i2d_reg(s, R_DST_Y), *i2d_reg(s, R_DST_Y_STRIDE),
              w, h, es[type], rot, mirror, flip);
    if (type == 0) {
        i2d_plane(*i2d_reg(s, R_SRC_UV), *i2d_reg(s, R_SRC_UV_STRIDE),
                  *i2d_reg(s, R_DST_UV), *i2d_reg(s, R_DST_UV_STRIDE),
                  w / 2, h / 2, 2, rot, mirror, flip);
    }
}

static void i2d_done(void *opaque)
{
    IngenicI2dState *s = opaque;

    *i2d_reg(s, R_CTRL) &= ~CTRL_START;
    *i2d_reg(s, R_IRQ_STATE) |= IRQ_DONE;
    i2d_update(s);
}

static uint64_t ingenic_i2d_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicI2dState *s = opaque;

    if (off == R_VERSION) {
        return 1;
    }
    return off < I2D_REGS && off != R_IRQ_CLEAR ? *i2d_reg(s, off) : 0;
}

static void ingenic_i2d_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicI2dState *s = opaque;

    if (off >= I2D_REGS) {
        return;
    }
    switch (off) {
    case R_VERSION:
    case R_IRQ_STATE:
        break;
    case R_IRQ_CLEAR:
        *i2d_reg(s, R_IRQ_STATE) &= ~val;
        break;
    case R_CTRL:
        if (val & (CTRL_RESET | CTRL_SAFE_RESET)) {
            timer_del(s->timer);
            *i2d_reg(s, R_IRQ_STATE) = 0;
            *i2d_reg(s, R_CTRL) = 0;
            break;
        }
        if ((val & CTRL_START) && !(*i2d_reg(s, R_CTRL) & CTRL_START)) {
            *i2d_reg(s, R_CTRL) = val;
            i2d_run(s);
            timer_mod(s->timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DONE_NS);
        }
        break;
    default:
        *i2d_reg(s, off) = val;
        break;
    }
    i2d_update(s);
}

static const MemoryRegionOps ingenic_i2d_ops = {
    .read = ingenic_i2d_read,
    .write = ingenic_i2d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_i2d_reset(DeviceState *dev)
{
    IngenicI2dState *s = INGENIC_I2D(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    i2d_update(s);
}

static void ingenic_i2d_init(Object *obj)
{
    IngenicI2dState *s = INGENIC_I2D(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_i2d_ops, s,
                          TYPE_INGENIC_I2D, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, i2d_done, s);
}

static const VMStateDescription vmstate_ingenic_i2d = {
    .name = TYPE_INGENIC_I2D,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicI2dState, I2D_REGS / 4),
        VMSTATE_TIMER_PTR(timer, IngenicI2dState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_i2d_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_i2d_reset);
    dc->vmsd = &vmstate_ingenic_i2d;
}

static const TypeInfo ingenic_i2d_info = {
    .name = TYPE_INGENIC_I2D,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicI2dState),
    .instance_init = ingenic_i2d_init,
    .class_init = ingenic_i2d_class_init,
};

static void ingenic_i2d_register_types(void)
{
    type_register_static(&ingenic_i2d_info);
}

type_init(ingenic_i2d_register_types)
