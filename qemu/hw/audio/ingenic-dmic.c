/*
 * Ingenic DMIC: the digital microphone interface at 0x10034000 (T31,
 * T40), as the vendor audio.ko runs it (oss2 devices/dmic/dmic_hal.[ch]
 * on T31, oss3 host/audio_dmic.[ch] on T40; INTC 12, PDMA request type
 * 13, DMIC_RX):
 *
 *   0x00 CR0     31 RESET (pulse: empties the FIFO), 30 RESET_TRI,
 *                18:16 CHNUM (microphones - 1), 13 UNPACK_MSB,
 *                12 UNPACK_DIS, 11 SW_LR, 10 SPLIT_DI, 8 PACK_EN,
 *                7:6 SR (0 8 kHz, 1 16 kHz, 2 48 kHz), 3 LP, 2 HPF1,
 *                1 TRI_EN, 0 EN
 *   0x04 GCR     4:0 gain (storage)
 *   0x08 IMR     interrupt masks, 1 = masked (5:0)
 *   0x0C INTCR   interrupt flags: 4 FIFO threshold, 3 empty, 2 full,
 *                1 pre-read, 0 trigger; write 1 to clear
 *   0x10..0x20   TRICR, THRH, THRL, TRIMMAX, TRINMAX: voice trigger,
 *                storage
 *   0x30 DR      the FIFO: a read takes one 32-bit entry
 *   0x34 FTHR    31 RDMS (DMA), 5:0 request threshold
 *   0x38 FSR     19 full, 18 trigger, 17 pre-read, 16 empty, 6:0 level
 *   0x50 CGDIS   storage
 *
 * While EN is set, frames come at the SR rate, one 16-bit sample per
 * microphone; a FIFO entry is two samples with PACK_EN (the earlier in
 * bits 15:0), one otherwise.  Every microphone hears the same sine of
 * "tone-hz" (1 kHz; 0 for silence) at a quarter of full scale.  The FIFO
 * holds 64 entries; "drq" (PDMA request 13) is high while RDMS is set
 * and the FIFO holds one.  The drivers mask every interrupt and run on
 * DMA only; the flags are still kept.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_DMIC "ingenic-dmic"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDmicState, INGENIC_DMIC)

#define DMIC_REGS       0x60

#define R_CR0           0x00
#define R_IMR           0x08
#define R_INTCR         0x0C
#define R_DR            0x30
#define R_FTHR          0x34
#define R_FSR           0x38

#define CR0_EN          (1u << 0)
#define CR0_PACK        (1u << 8)
#define CR0_RESET       (1u << 31)
#define FTHR_RDMS       (1u << 31)

#define INT_THR         (1u << 4)
#define INT_EMPTY       (1u << 3)
#define INT_FULL        (1u << 2)

#define FSR_FULL        (1u << 19)
#define FSR_EMPTY       (1u << 16)

#define DEPTH           64
#define TICK_NS         (1 * SCALE_MS)

struct IngenicDmicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq drq;
    QEMUTimer *timer;

    uint32_t regs[DMIC_REGS / 4];
    uint32_t fifo[DEPTH];
    uint32_t head, count;
    uint32_t half;              /* a packed entry's first sample, + 1 */
    int64_t last_ns;
    uint64_t frac;
    uint64_t phase;

    uint32_t tone_hz;
};

static uint32_t *dmic_reg(IngenicDmicState *s, hwaddr off)
{
    return &s->regs[off >> 2];
}

static uint32_t dmic_rate(IngenicDmicState *s)
{
    static const uint32_t rates[4] = { 8000, 16000, 48000, 48000 };

    return rates[extract32(*dmic_reg(s, R_CR0), 6, 2)];
}

static void dmic_update(IngenicDmicState *s)
{
    uint32_t *intcr = dmic_reg(s, R_INTCR);
    uint32_t thr = extract32(*dmic_reg(s, R_FTHR), 0, 6);

    *intcr &= ~(INT_THR | INT_EMPTY);
    *intcr |= (s->count >= thr ? INT_THR : 0) | (!s->count ? INT_EMPTY : 0);
    *dmic_reg(s, R_FSR) = (s->count >= DEPTH ? FSR_FULL : 0) |
                          (!s->count ? FSR_EMPTY : 0) | s->count;
    qemu_set_irq(s->irq, (*intcr & ~*dmic_reg(s, R_IMR) & 0x3f) != 0);
    qemu_set_irq(s->drq, (*dmic_reg(s, R_FTHR) & FTHR_RDMS) && s->count);
}

static void dmic_push(IngenicDmicState *s, uint32_t v)
{
    if (s->count >= DEPTH) {
        *dmic_reg(s, R_INTCR) |= INT_FULL;
        return;
    }
    s->fifo[(s->head + s->count) % DEPTH] = v;
    s->count++;
}

static void dmic_frame(IngenicDmicState *s)
{
    uint32_t cr0 = *dmic_reg(s, R_CR0);
    int mics = extract32(cr0, 16, 3) + 1, i;
    int16_t v = 0;

    if (s->tone_hz) {
        double t = (double)s->phase / dmic_rate(s);

        v = (int16_t)lrint(8192 * sin(2 * M_PI * s->tone_hz * t));
    }
    s->phase++;
    for (i = 0; i < mics; i++) {
        if (!(cr0 & CR0_PACK)) {
            dmic_push(s, (uint16_t)v);
        } else if (s->half) {
            dmic_push(s, (s->half - 1) | ((uint32_t)(uint16_t)v << 16));
            s->half = 0;
        } else {
            s->half = (uint16_t)v + 1;
        }
    }
}

static void dmic_advance(IngenicDmicState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t dt = MIN(now - s->last_ns, 100 * TICK_NS);
    uint64_t n;

    s->last_ns = now;
    if (dt <= 0 || !(*dmic_reg(s, R_CR0) & CR0_EN)) {
        return;
    }
    s->frac += (uint64_t)dt * dmic_rate(s);
    n = s->frac / NANOSECONDS_PER_SECOND;
    s->frac %= NANOSECONDS_PER_SECOND;
    while (n--) {
        dmic_frame(s);
        dmic_update(s);         /* the PDMA takes what it can now */
    }
}

static void dmic_tick(void *opaque)
{
    IngenicDmicState *s = opaque;

    dmic_advance(s);
    if (*dmic_reg(s, R_CR0) & CR0_EN) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TICK_NS);
    }
}

static void dmic_flush(IngenicDmicState *s)
{
    s->head = s->count = s->half = 0;
}

static uint64_t ingenic_dmic_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicDmicState *s = opaque;
    uint32_t v = 0;

    if (off >= DMIC_REGS) {
        return 0;
    }
    if (off == R_DR) {
        if (s->count) {
            v = s->fifo[s->head];
            s->head = (s->head + 1) % DEPTH;
            s->count--;
            dmic_update(s);
        }
        return v;
    }
    return *dmic_reg(s, off);
}

static void ingenic_dmic_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicDmicState *s = opaque;
    bool was_on = *dmic_reg(s, R_CR0) & CR0_EN;

    if (off >= DMIC_REGS) {
        return;
    }
    switch (off) {
    case R_CR0:
        if (val & CR0_RESET) {
            dmic_flush(s);
        }
        *dmic_reg(s, off) = val & ~(CR0_RESET | (1u << 30));
        if ((val & CR0_EN) && !was_on) {
            s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->frac = 0;
            timer_mod(s->timer, s->last_ns + TICK_NS);
        } else if (!(val & CR0_EN)) {
            timer_del(s->timer);
        }
        break;
    case R_INTCR:
        *dmic_reg(s, off) &= ~val;
        break;
    case R_DR: case R_FSR:
        return;                         /* read only */
    default:
        *dmic_reg(s, off) = val;
        break;
    }
    dmic_update(s);
}

static const MemoryRegionOps ingenic_dmic_ops = {
    .read = ingenic_dmic_read,
    .write = ingenic_dmic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_dmic_reset(DeviceState *dev)
{
    IngenicDmicState *s = INGENIC_DMIC(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    *dmic_reg(s, R_IMR) = 0x3f;
    dmic_flush(s);
    s->phase = 0;
    dmic_update(s);
}

static void ingenic_dmic_init(Object *obj)
{
    IngenicDmicState *s = INGENIC_DMIC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_dmic_ops, s,
                          TYPE_INGENIC_DMIC, 0x1000);
    /* a register write may start the PDMA, which reads DR back here */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->drq, "drq", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dmic_tick, s);
}

static const Property ingenic_dmic_props[] = {
    DEFINE_PROP_UINT32("tone-hz", IngenicDmicState, tone_hz, 1000),
};

static const VMStateDescription vmstate_ingenic_dmic = {
    .name = TYPE_INGENIC_DMIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicDmicState, DMIC_REGS / 4),
        VMSTATE_UINT32_ARRAY(fifo, IngenicDmicState, DEPTH),
        VMSTATE_UINT32(head, IngenicDmicState),
        VMSTATE_UINT32(count, IngenicDmicState),
        VMSTATE_UINT32(half, IngenicDmicState),
        VMSTATE_INT64(last_ns, IngenicDmicState),
        VMSTATE_UINT64(frac, IngenicDmicState),
        VMSTATE_UINT64(phase, IngenicDmicState),
        VMSTATE_TIMER_PTR(timer, IngenicDmicState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_dmic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_dmic_reset);
    device_class_set_props(dc, ingenic_dmic_props);
    dc->vmsd = &vmstate_ingenic_dmic;
}

static const TypeInfo ingenic_dmic_info = {
    .name = TYPE_INGENIC_DMIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDmicState),
    .instance_init = ingenic_dmic_init,
    .class_init = ingenic_dmic_class_init,
};

static void ingenic_dmic_register_types(void)
{
    type_register_static(&ingenic_dmic_info);
}

type_init(ingenic_dmic_register_types)
