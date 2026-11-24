/*
 * Grain Media GM8126 scaler (FTSCAL010, the driver's "fscaler" V3).
 *
 * Memory to memory, one job at a time: the driver programs the source
 * window and the destination, then sets PIMG_EN in the enable register.
 *
 *   0x000 enable: bit 31 the block, bit 24 fire (self clearing here)
 *   0x008 bit 28 software reset   0x3EC bit 24 busy calculating (never)
 *   0x028 output size  0x068 source size (w | h << 16)
 *   0x204 bits 13:12 DMA order (3: 16x16 tiles), 9:8 planes (0: one,
 *         1: three, 2: two)
 *   0x20C source >> 2, 0x210 its pitch / 4; 0x250 / 0x260 / 0x270
 *         destinations >> 2, 0x21C their pitch / 4
 *   0x3E0 status (w1c): bit 24 frame done, bit 8 bus idle; 0x3E4 mask
 *
 * Packed 4:2:2 (UYVY) in; packed 4:2:2 or the encoders' 4:2:0 tiles out;
 * nearest neighbour.  The on-screen display is not drawn.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_GM_SCAL010 "gm-scal010"
OBJECT_DECLARE_SIMPLE_TYPE(GmScal010State, GM_SCAL010)

#define SCAL_SIZE   0x1000
#define SCAL_NREGS  (0x400 / 4)
#define SCAL_JOB_NS 3000000

#define R_EN        0x000
#define R_RST       0x008
#define R_INT       0x3E0
#define R_MASK      0x3E4

#define EN_FIRE     (1u << 24)
#define ST_DONE     ((1u << 24) | (1u << 8))

struct GmScal010State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[SCAL_NREGS];
};

static void scal_update(GmScal010State *s)
{
    qemu_set_irq(s->irq, (s->regs[R_INT / 4] & ~s->regs[R_MASK / 4]) != 0);
}

static void scal_io(hwaddr a, void *p, size_t n, bool write)
{
    if (write) {
        dma_memory_write(&address_space_memory, a, p, n,
                         MEMTXATTRS_UNSPECIFIED);
    } else {
        dma_memory_read(&address_space_memory, a, p, n,
                        MEMTXATTRS_UNSPECIFIED);
    }
}

static void scal_run(GmScal010State *s)
{
    const uint32_t in = s->regs[0x068 / 4], out = s->regs[0x028 / 4];
    const int sw = extract32(in, 0, 13), sh = extract32(in, 16, 13);
    const int dw = extract32(out, 0, 13) & ~1, dh = extract32(out, 16, 13);
    const uint32_t udma = s->regs[0x204 / 4];
    const int split = extract32(udma, 8, 2);
    const hwaddr src = (hwaddr)(s->regs[0x20C / 4] & 0x3FFFFFFF) << 2;
    const uint32_t spit = s->regs[0x210 / 4] * 4;
    const uint32_t dpit = s->regs[0x21C / 4] * 4;
    hwaddr dst[3];
    g_autofree uint8_t *row = NULL, *img = NULL;
    int x, y, i, j;

    for (i = 0; i < 3; i++) {
        dst[i] = (hwaddr)(s->regs[(0x250 + 0x10 * i) / 4] & 0x3FFFFFFF) << 2;
    }
    if (sw < 2 || sh < 1 || dw < 2 || dh < 1 || sw > 4096 || sh > 4096 ||
        dw > 4096 || dh > 4096) {
        return;
    }
    row = g_malloc0(sw * 2 + 4);
    img = g_malloc(dw * dh * 2);                /* scaled, UYVY */
    for (y = 0; y < dh; y++) {
        uint8_t *o = img + y * dw * 2;

        scal_io(src + (hwaddr)(y * sh / dh) * spit, row, sw * 2, false);
        for (x = 0; x < dw; x += 2) {
            const int xa = x * sw / dw, xb = (x + 1) * sw / dw;
            const uint8_t *c = row + (xa & ~1) * 2;

            o[x * 2] = c[0];
            o[x * 2 + 1] = row[xa * 2 + 1];
            o[x * 2 + 2] = c[2];
            o[x * 2 + 3] = row[xb * 2 + 1];
        }
    }
    if (!split) {
        for (y = 0; y < dh; y++) {
            scal_io(dst[0] + (hwaddr)y * MAX(dpit, dw * 2),
                    img + y * dw * 2, dw * 2, true);
        }
        return;
    }
    for (y = 0; y + 16 <= dh; y += 16) {
        for (x = 0; x + 16 <= dw; x += 16) {
            const uint32_t mb = (y / 16) * MAX(dpit / 16, dw / 16) +
                                x / 16;
            uint8_t l[256], c[128];

            for (j = 0; j < 16; j++) {
                const uint8_t *p = img + (y + j) * dw * 2 + x * 2;

                for (i = 0; i < 16; i++) {
                    l[j * 16 + i] = p[i * 2 + 1];
                    if (!(i & 1) && !(j & 1)) {
                        const int k = (j / 2) * 8 + i / 2;

                        if (split == 2) {
                            c[k * 2] = p[i * 2];
                            c[k * 2 + 1] = p[i * 2 + 2];
                        } else {
                            c[k] = p[i * 2];
                            c[64 + k] = p[i * 2 + 2];
                        }
                    }
                }
            }
            scal_io(dst[0] + mb * 256, l, 256, true);
            if (split == 2) {
                scal_io(dst[1] + mb * 128, c, 128, true);
            } else {
                scal_io(dst[1] + mb * 64, c, 64, true);
                scal_io(dst[2] + mb * 64, c + 64, 64, true);
            }
        }
    }
}

static void scal_done(void *opaque)
{
    GmScal010State *s = opaque;

    scal_run(s);
    s->regs[R_EN / 4] &= ~EN_FIRE;
    s->regs[R_INT / 4] |= ST_DONE;
    scal_update(s);
}

static uint64_t scal_read(void *opaque, hwaddr off, unsigned size)
{
    GmScal010State *s = opaque;

    return off / 4 < SCAL_NREGS ? s->regs[off / 4] : 0;
}

static void scal_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    GmScal010State *s = opaque;

    if (off / 4 >= SCAL_NREGS) {
        return;
    }
    switch (off) {
    case R_INT:
        s->regs[R_INT / 4] &= ~val;
        break;
    case R_RST:
        if (val & (1u << 28)) {
            timer_del(s->timer);
            s->regs[R_EN / 4] &= ~EN_FIRE;
        }
        s->regs[off / 4] = val & ~(1u << 28);
        break;
    case R_EN:
        s->regs[off / 4] = val;
        if ((val & EN_FIRE) && (val & (1u << 31))) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SCAL_JOB_NS);
        }
        break;
    default:
        s->regs[off / 4] = val;
    }
    scal_update(s);
}

static const MemoryRegionOps scal_ops = {
    .read = scal_read,
    .write = scal_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void scal_reset(DeviceState *dev)
{
    GmScal010State *s = GM_SCAL010(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_MASK / 4] = 0xFFFFFFFF;
    scal_update(s);
}

static void scal_realize(DeviceState *dev, Error **errp)
{
    GmScal010State *s = GM_SCAL010(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &scal_ops, s,
                          TYPE_GM_SCAL010, SCAL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, scal_done, s);
}

static const VMStateDescription vmstate_scal = {
    .name = TYPE_GM_SCAL010,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmScal010State, SCAL_NREGS),
        VMSTATE_TIMER_PTR(timer, GmScal010State),
        VMSTATE_END_OF_LIST()
    }
};

static void scal_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = scal_realize;
    device_class_set_legacy_reset(dc, scal_reset);
    dc->vmsd = &vmstate_scal;
}

static const TypeInfo scal_info = {
    .name = TYPE_GM_SCAL010,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GmScal010State),
    .class_init = scal_class_init,
};

static void scal_register_types(void)
{
    type_register_static(&scal_info);
}

type_init(scal_register_types)
