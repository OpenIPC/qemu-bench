/*
 * Xiongmai XM232 memory-to-memory DMA controller (0x20020000, SPI 15), as
 * dma.ko drives it for /dev/dma's copy ioctl -- which Sofia uses to fetch
 * the VI frames its person detector reads (xm_ai_getframe_doublecam*).
 * One channel, a block at a time (dma.ko copies 16 KiB a block and starts
 * the next from its interrupt):
 *
 *   +0x010   1 (bit 0): controller on (dma.ko writes 3, and 0 first)
 *   +0x018   channel enable: bit 0 on, bit 8 its write enable (as on a
 *            DesignWare DMAC); it reads back 0 once the block is done
 *   +0x100   source, +0x108 destination (physical)
 *   +0x110   the block's 8-byte transfers - 1
 *   +0x118 / +0x11C / +0x124   control (dma.ko: 0x3fc45b00, 0x24,
 *            0x20000; read back only)
 *   +0x180   written 2 per block; +0x190 the done mask (2)
 *   +0x188   done status, raised & +0x190 (the interrupt); dma.ko's
 *            handler writes it back to +0x198 to clear it
 *
 * A block completes a few microseconds after the enable.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/dma.h"

#define TYPE_XM_DMAC "xm-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(XmDmacState, XM_DMAC)

#define DMAC_SIZE   0x1000
#define DMAC_CFG    0x010
#define DMAC_CHEN   0x018
#define DMAC_SRC    0x100
#define DMAC_DST    0x108
#define DMAC_COUNT  0x110
#define DMAC_STATUS 0x188
#define DMAC_MASK   0x190
#define DMAC_CLEAR  0x198
#define DMAC_DONE   2
#define DMAC_MAX    (1 << 20)   /* a block's bytes, at most */

struct XmDmacState {
    SysBusDevice parent_obj;
    MemoryRegion io;
    uint32_t regs[DMAC_SIZE / 4];
    QEMUTimer *timer;
    qemu_irq irq;
};

#define R(off) (s->regs[(off) / 4])

static void xm_dmac_update(XmDmacState *s)
{
    qemu_set_irq(s->irq, (R(DMAC_STATUS) & R(DMAC_MASK)) != 0);
}

static void xm_dmac_done(void *opaque)
{
    XmDmacState *s = XM_DMAC(opaque);
    const uint64_t n = ((uint64_t)R(DMAC_COUNT) + 1) * 8;

    if (n > DMAC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-dmac: %" PRIu64 "-byte block\n",
                      n);
    } else {
        g_autofree uint8_t *buf = g_malloc(n);

        dma_memory_read(&address_space_memory, R(DMAC_SRC), buf, n,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory, R(DMAC_DST), buf, n,
                         MEMTXATTRS_UNSPECIFIED);
    }
    R(DMAC_CHEN) &= ~1u;
    R(DMAC_STATUS) |= DMAC_DONE;
    xm_dmac_update(s);
}

static uint64_t xm_dmac_read(void *opaque, hwaddr off, unsigned size)
{
    XmDmacState *s = XM_DMAC(opaque);

    return s->regs[off / 4];
}

static void xm_dmac_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    XmDmacState *s = XM_DMAC(opaque);

    switch (off) {
    case DMAC_CHEN:
        /* bits 8.. say which of bits 0.. the write changes */
        R(DMAC_CHEN) = (R(DMAC_CHEN) & ~((val >> 8) & 0xff)) |
                       (val & (val >> 8) & 0xff);
        if ((val & 0x101) == 0x101 && (R(DMAC_CFG) & 1)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      10 * SCALE_US);
        }
        break;
    case DMAC_CLEAR:
        R(DMAC_STATUS) &= ~(uint32_t)val;
        break;
    case DMAC_STATUS:
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
    xm_dmac_update(s);
}

static const MemoryRegionOps xm_dmac_ops = {
    .read = xm_dmac_read,
    .write = xm_dmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_dmac_reset(DeviceState *dev)
{
    XmDmacState *s = XM_DMAC(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    xm_dmac_update(s);
}

static void xm_dmac_init(Object *obj)
{
    XmDmacState *s = XM_DMAC(obj);

    memory_region_init_io(&s->io, obj, &xm_dmac_ops, s, "xm-dmac",
                          DMAC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->io);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_dmac_done, s);
}

static void xm_dmac_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_dmac_reset);
}

static const TypeInfo xm_dmac_info = {
    .name          = TYPE_XM_DMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmDmacState),
    .instance_init = xm_dmac_init,
    .class_init    = xm_dmac_class_init,
};

static void xm_dmac_register_types(void)
{
    type_register_static(&xm_dmac_info);
}

type_init(xm_dmac_register_types)
