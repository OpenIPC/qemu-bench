/*
 * SigmaStar Ifado audio ("BACH"), its capture DMA.
 *
 * sstar_mi.ko reaches BACH through HalBachReadReg/HalBachWriteReg with a
 * bank index and a byte offset: index 0 is RIU bank 0x1502, 1 is 0x1503,
 * 2 is 0x1504 and 3 is 0x1034, 16-bit registers at even offsets (offset
 * o is register o / 2 of the bank).  The DMAs (HalAudDma*) are in bank
 * 0x1502; the AI writer, DMA 0 to the driver (DMA 7, at 0x80-0x8E, is
 * the AO reader), is (byte offsets):
 *
 *   0x80  bit 11 enables the threshold interrupt, bit 14 the overrun
 *         one (HalAudDmaIntEnable); writing bit 9 clears the overrun
 *         (HalAudDmaClearInt)
 *   0x92  [11:0] the buffer's address [14:3]; bit 12 latches the level
 *         count into 0x9E; a change of bit 13 takes the count in 0x98 as
 *         read (HalAudDmaTrigLevelCnt); bit 15 runs the DMA
 *   0x94  [14:0] the buffer's address [29:15]
 *   0x96  the buffer's size in 8-byte units
 *   0x98  bytes read, in 8-byte units
 *   0x9A  the threshold, 8-byte units (HalAudDmaAiSetThreshold)
 *   0x9E  the level count, 8-byte units (HalAudDmaGetLevelCnt)
 *   0x90  flags (HalAudDmaGetFlags): bit 1 overrun, bit 5 the level at
 *         or above the threshold, bit 6 full
 *
 * The model's microphone hears a 1 kHz tone: while the DMA runs the
 * ring fills at "rate" bytes a second of 16-bit samples ("sample-rate"
 * Hz), the level count growing until the buffer is full; samples
 * that arrive then are dropped, which latches the overrun until 0x80
 * bit 9 clears it.  _AudDmaIrq acts only on a flag whose enable is set (bit 5
 * with bit 11, bit 1 with bit 14) and masks it, so the interrupt is the
 * OR of exactly those pairs; a line raised on any other condition is
 * never masked and storms the CPU.  Everything else in the banks is
 * storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SSTAR_BACH "sstar-bach"
OBJECT_DECLARE_SIMPLE_TYPE(SstarBachState, SSTAR_BACH)

#define BACH_REGS           0x80        /* one RIU bank */
#define R(off)              ((off) >> 1)
#define AI_IRQEN            R(0x80)
#define EN_THRESH           (1u << 11)
#define EN_OVERRUN          (1u << 14)
#define CLR_OVERRUN         (1u << 9)
#define AI_ADDR_LO          R(0x92)
#define AI_LATCH            (1u << 12)
#define AI_TRIG             (1u << 13)
#define AI_RUN              (1u << 15)
#define AI_ADDR_HI          R(0x94)
#define AI_SIZE             R(0x96)
#define AI_READ             R(0x98)
#define AI_THRESH           R(0x9A)
#define AI_LEVEL            R(0x9E)
#define AI_FLAGS            R(0x90)
#define FLAG_OVERRUN        (1u << 1)
#define FLAG_HIGH           (1u << 5)
#define FLAG_FULL           (1u << 6)
#define TICK_NS             (10 * SCALE_MS)
#define SS_DRAM_BASE        0x20000000

struct SstarBachState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *tick;
    uint16_t regs[BACH_REGS];
    uint32_t level;             /* bytes in the ring not yet read */
    uint32_t wptr;              /* where the next byte goes */
    uint64_t sample;            /* samples written so far */
    int64_t last;

    uint32_t rate;
    uint32_t sample_rate;
    bool trace;
};

static uint64_t bach_base(SstarBachState *s)
{
    return ((uint64_t)(s->regs[AI_ADDR_HI] & 0x7FFF) << 15) |
           ((uint64_t)(s->regs[AI_ADDR_LO] & 0xFFF) << 3);
}

static uint32_t bach_size(SstarBachState *s)
{
    return (uint32_t)s->regs[AI_SIZE] << 3;
}

static void bach_update(SstarBachState *s)
{
    uint32_t thresh = (uint32_t)s->regs[AI_THRESH] << 3;
    uint16_t f = s->regs[AI_FLAGS] & ~(FLAG_HIGH | FLAG_FULL);
    uint16_t en = s->regs[AI_IRQEN];
    bool on = s->regs[AI_ADDR_LO] & AI_RUN;

    if (s->level >= bach_size(s) && bach_size(s)) {
        f |= FLAG_FULL;
    }
    if (thresh && s->level >= thresh) {
        f |= FLAG_HIGH;
    }
    s->regs[AI_FLAGS] = f;
    qemu_set_irq(s->irq, on && (((f & FLAG_HIGH) && (en & EN_THRESH)) ||
                                ((f & FLAG_OVERRUN) && (en & EN_OVERRUN))));
}

static void bach_fill(SstarBachState *s, uint32_t n)
{
    uint32_t size = bach_size(s);
    uint64_t base = bach_base(s);
    uint8_t buf[512];

    while (n >= 2) {
        uint32_t chunk = MIN(MIN(n, sizeof(buf)), size - s->wptr) & ~1u;
        uint32_t i;
        hwaddr a = base + s->wptr;

        if (!chunk) {
            break;
        }
        for (i = 0; i < chunk; i += 2) {
            int16_t v = (int16_t)(8000 * sin(2 * M_PI * 1000 * s->sample++ /
                                             MAX(s->sample_rate, 1)));

            buf[i] = v & 0xFF;
            buf[i + 1] = (v >> 8) & 0xFF;
        }
        if (a < SS_DRAM_BASE) {
            a += SS_DRAM_BASE;
        }
        dma_memory_write(&address_space_memory, a, buf, chunk,
                         MEMTXATTRS_UNSPECIFIED);
        s->wptr = (s->wptr + chunk) % size;
        n -= chunk;
    }
}

static void bach_tick(void *opaque)
{
    SstarBachState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t size = bach_size(s);

    if (!(s->regs[AI_ADDR_LO] & AI_RUN) || !size) {
        return;
    }
    if (s->last) {
        uint64_t want = (uint64_t)(now - s->last) * s->rate /
                        NANOSECONDS_PER_SECOND;
        uint64_t n = MIN(want, (uint64_t)(size - s->level)) & ~7ull;

        if (n) {
            bach_fill(s, n);
            s->level += n;
            s->last += (int64_t)(n * NANOSECONDS_PER_SECOND /
                                 MAX(s->rate, 1));
        }
        if (s->level >= size && want > n) {
            /* full: the microphone drops samples, which is the overrun */
            s->regs[AI_FLAGS] |= FLAG_OVERRUN;
            s->last = now;
        }
    } else {
        s->last = now;
    }
    bach_update(s);
    timer_mod(s->tick, now + TICK_NS);
}

static uint64_t sstar_bach_read(void *opaque, hwaddr off, unsigned size)
{
    SstarBachState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->regs[(off >> 2) & (BACH_REGS - 1)];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_bach_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SstarBachState *s = opaque;
    int reg = (off >> 2) & (BACH_REGS - 1);
    uint16_t mask, v, old;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        mask = off & 1 ? 0xFF00 : 0x00FF;
        v = off & 1 ? (val & 0xFF) << 8 : val & 0xFF;
    } else {
        mask = 0xFFFF;
        v = val;
    }
    if (reg == AI_LEVEL || reg == AI_FLAGS) {
        return;
    }
    old = s->regs[reg];
    s->regs[reg] = (old & ~mask) | (v & mask);
    if (s->trace) {
        qemu_log("sstar-bach: W 0x%02x = 0x%04x\n", reg << 1, s->regs[reg]);
    }
    switch (reg) {
    case AI_IRQEN:
        if (s->regs[reg] & CLR_OVERRUN) {
            s->regs[AI_FLAGS] &= ~FLAG_OVERRUN;
        }
        break;
    case AI_ADDR_LO:
        if ((s->regs[reg] & AI_RUN) && !(old & AI_RUN)) {
            s->regs[AI_FLAGS] &= ~FLAG_OVERRUN;
            s->level = s->wptr = 0;
            s->last = 0;
            bach_tick(s);
        }
        if (s->regs[reg] & AI_LATCH) {
            s->regs[AI_LEVEL] = MIN(s->level >> 3, 0xFFFF);
        }
        if ((s->regs[reg] ^ old) & AI_TRIG) {
            uint32_t n = (uint32_t)s->regs[AI_READ] << 3;

            s->level -= MIN(n, s->level);
        }
        break;
    }
    bach_update(s);
}

static const MemoryRegionOps sstar_bach_ops = {
    .read = sstar_bach_read,
    .write = sstar_bach_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_bach_reset(DeviceState *dev)
{
    SstarBachState *s = SSTAR_BACH(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->level = s->wptr = 0;
    s->sample = 0;
    s->last = 0;
    timer_del(s->tick);
    qemu_set_irq(s->irq, 0);
}

static void sstar_bach_realize(DeviceState *dev, Error **errp)
{
    SstarBachState *s = SSTAR_BACH(dev);

    s->tick = timer_new_ns(QEMU_CLOCK_VIRTUAL, bach_tick, s);
}

static void sstar_bach_init(Object *obj)
{
    SstarBachState *s = SSTAR_BACH(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_bach_ops, s,
                          TYPE_SSTAR_BACH, BACH_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sstar_bach_props[] = {
    DEFINE_PROP_UINT32("rate", SstarBachState, rate, 32000),
    DEFINE_PROP_UINT32("sample-rate", SstarBachState, sample_rate, 16000),
    DEFINE_PROP_BOOL("trace", SstarBachState, trace, false),
};

static const VMStateDescription vmstate_sstar_bach = {
    .name = TYPE_SSTAR_BACH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarBachState, BACH_REGS),
        VMSTATE_UINT32(level, SstarBachState),
        VMSTATE_UINT32(wptr, SstarBachState),
        VMSTATE_UINT64(sample, SstarBachState),
        VMSTATE_INT64(last, SstarBachState),
        VMSTATE_TIMER_PTR(tick, SstarBachState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_bach_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_bach_realize;
    device_class_set_legacy_reset(dc, sstar_bach_reset);
    device_class_set_props(dc, sstar_bach_props);
    dc->vmsd = &vmstate_sstar_bach;
}

static const TypeInfo sstar_bach_info = {
    .name           = TYPE_SSTAR_BACH,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarBachState),
    .instance_init  = sstar_bach_init,
    .class_init     = sstar_bach_class_init,
};

static void sstar_bach_register_types(void)
{
    type_register_static(&sstar_bach_info);
}

type_init(sstar_bach_register_types)
