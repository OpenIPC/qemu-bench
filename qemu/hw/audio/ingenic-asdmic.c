/*
 * Ingenic T41 DMIC: the digital microphones of T41's audio system at
 * 0x134D0000, as the T41 SDK's audio.ko runs them with dmic_enable=1
 * (oss3 host/T41/dmic: as-dma, as-dbus, as-fmtcov, as-dmic; the devices
 * are its own platform devices, boards/t41_platform.c -- t41.dtsi's
 * "ingenic,dmic" node stays disabled; core INTC 33).  One 64 KiB block:
 *
 *   0x0000 + 0x18 n  DMA channel n (0..9): DBA (the bus address; it
 *                    advances as data lands, the driver's hardware
 *                    pointer), DTC, DCR (1 RESET, pulse; 0 CTE), DSR (5
 *                    TT_INT, 4 LTT_INT, 3 TT, 2 LTT, 1 RST_EN, 0 LINK;
 *                    write 1 to clear), DCM (31 NDES, 8 BAI, 6:4 TSZ,
 *                    1 TIE, 0 LTIE), DDA (the descriptor's address)
 *   0x0100 DGRR (0 RESET)  0x0104 DGER (0 DMA_EN)  0x0108 AEER
 *   0x010C AESR  0x0110 AIPR (9:0 channel n's interrupt)
 *   0x1000 + 0x10 n  FIFO n: FAS (depth), FCR (2 FLUSH, 0 FIFO_EN), FFR
 *                    (threshold), FSR (28:16 level)
 *   0x2000 + 4 n     format converter DFCR n (15:12 channels, 10:8
 *                    sample size, 2 PACKEN, 0 ENABLE)
 *   0x4000..0x402C   the data bus' timeslot routing (BTSET, BTCLR, BTSR,
 *                    BFSR, BFCR0..2, BST0..2, BTT0..1)
 *   0xA000 CR0 (31 RESET, pulse; 22 HPF2, 21 LPF, 20 HPF1; 19:16 CHNUM,
 *          microphones - 1; 13:12 OSS, 0 16-bit, 1 24-bit; 11 SW_LR;
 *          8:6 SR, 0 8 kHz, 1 16, 2 48, 3 96; 0 EN)
 *   0xA004 GCR (4:0 gain)  0xA008 IER  0xA00C ICR  0xA010 TCR
 *
 * The driver routes the DMIC to FIFO 5 over the bus and converts and
 * moves it through channel 5: a closed ring of 4-word descriptors (word
 * 0: 0 LTIE, 1 TIE, 2 LINK, 3 LTE, 6:4 TSZ, 8 BAI; 1 DBA; 2 the next
 * descriptor's bus address; 3 DTC, bursts of TSZ: 0 one word, n 2^(n+1)
 * words), which never interrupts (TIE clear) -- the DSP work queue polls
 * DBA.  Here, while CR0.EN, channel 5's CTE and FIFO 5's FIFO_EN are set,
 * frames come at the SR rate, a sample per microphone: 16-bit with OSS 0
 * (two to a word), else 32-bit words holding 24 bits in 31:8; they are
 * written at DBA, which counts DTC down through the descriptor and then
 * follows the link.  Every microphone hears the same sine of "tone-hz"
 * (1 kHz; 0 for silence) at a quarter of full scale.  A descriptor with
 * TIE / LTIE raises TT_INT / LTT_INT, its AIPR bit and the interrupt.
 * Routing, the converter and the gain (GCR, the driver's default 4) are
 * storage: the samples come out at that level whatever the gain.
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
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_INGENIC_ASDMIC "ingenic-asdmic"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAsDmicState, INGENIC_ASDMIC)

#define AS_SIZE         0x10000
#define NCH             10
#define CH              5               /* the DMIC's DMA channel and FIFO */

#define DMA_REGS        0x114
#define FIFO_BASE       0x1000
#define FIFO_REGS       0x100
#define FMT_BASE        0x2000
#define FMT_REGS        0x28
#define DBUS_BASE       0x4000
#define DBUS_REGS       0x30
#define DMIC_BASE       0xA000
#define DMIC_REGS       0x14

#define DBA(n)          (0x00 + 0x18 * (n))
#define DTC(n)          (0x04 + 0x18 * (n))
#define DCR(n)          (0x08 + 0x18 * (n))
#define DSR(n)          (0x0C + 0x18 * (n))
#define DCM(n)          (0x10 + 0x18 * (n))
#define DDA(n)          (0x14 + 0x18 * (n))
#define DGRR            0x100
#define AIPR            0x110

#define DCR_RESET       (1u << 1)
#define DCR_CTE         (1u << 0)
#define DSR_TT_INT      (1u << 5)
#define DSR_LTT_INT     (1u << 4)
#define DSR_TT          (1u << 3)
#define DSR_LTT         (1u << 2)

#define FCR(n)          (0x04 + 0x10 * (n))
#define FCR_FIFO_EN     (1u << 0)

#define CR0             0x00
#define CR0_RESET       (1u << 31)
#define CR0_EN          (1u << 0)

#define TICK_NS         (1 * SCALE_MS)

struct IngenicAsDmicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t dma[DMA_REGS / 4];
    uint32_t fifo[FIFO_REGS / 4];
    uint32_t fmt[FMT_REGS / 4];
    uint32_t dbus[DBUS_REGS / 4];
    uint32_t dmic[DMIC_REGS / 4];

    uint32_t tone_hz;
    uint64_t frame;                     /* frames made since EN */
    int64_t start_ns;                   /* when the stream started */
    uint32_t left;                      /* bytes left in the descriptor */
    uint32_t half;                      /* a 16-bit sample waiting for its
                                           pair, | 0x10000 */
};

static bool asdmic_running(IngenicAsDmicState *s)
{
    return (s->dmic[CR0 / 4] & CR0_EN) && (s->dma[DCR(CH) / 4] & DCR_CTE) &&
           (s->fifo[FCR(CH) / 4] & FCR_FIFO_EN);
}

static uint32_t asdmic_rate(IngenicAsDmicState *s)
{
    static const uint32_t rates[] = { 8000, 16000, 48000, 96000 };

    return rates[extract32(s->dmic[CR0 / 4], 6, 2)];
}

static void asdmic_update(IngenicAsDmicState *s)
{
    qemu_set_irq(s->irq, (s->dma[AIPR / 4] & 0x3ff) != 0);
}

/* channel 5's descriptor at DDA: its address, count and flags */
static void asdmic_load(IngenicAsDmicState *s)
{
    uint32_t d[4];
    unsigned tsz;

    dma_memory_read(&address_space_memory, s->dma[DDA(CH) / 4], d,
                    sizeof(d), MEMTXATTRS_UNSPECIFIED);
    d[0] = le32_to_cpu(d[0]);
    tsz = extract32(d[0], 4, 3);
    s->dma[DCM(CH) / 4] = d[0] & 0x173;
    s->dma[DBA(CH) / 4] = le32_to_cpu(d[1]);
    s->dma[DTC(CH) / 4] = le32_to_cpu(d[3]) & 0xffffff;
    s->dma[DSR(CH) / 4] = (s->dma[DSR(CH) / 4] & ~1u) | !!(d[0] & 0x4);
    s->left = s->dma[DTC(CH) / 4] * 4 * (tsz ? 1u << (tsz + 1) : 1);
    /* the next one, for when this one ends */
    s->dma[DDA(CH) / 4] = (d[0] & 0x4) ? le32_to_cpu(d[2]) : 0;
}

/* a descriptor done: its flags, and the next one if it links */
static void asdmic_desc_end(IngenicAsDmicState *s)
{
    uint32_t dcm = s->dma[DCM(CH) / 4], *dsr = &s->dma[DSR(CH) / 4];
    bool link = *dsr & 1;

    *dsr |= DSR_TT;
    if (dcm & 2) {
        *dsr |= DSR_TT_INT;
        s->dma[AIPR / 4] |= 1u << CH;
    }
    if (!link) {
        *dsr |= DSR_LTT;
        if (dcm & 1) {
            *dsr |= DSR_LTT_INT;
            s->dma[AIPR / 4] |= 1u << CH;
        }
        s->dma[DCR(CH) / 4] &= ~DCR_CTE;
    } else if (s->dma[DDA(CH) / 4]) {
        asdmic_load(s);
    }
    asdmic_update(s);
}

static void asdmic_put32(IngenicAsDmicState *s, uint32_t w)
{
    uint32_t le = cpu_to_le32(w);

    if (!s->left) {
        return;
    }
    dma_memory_write(&address_space_memory, s->dma[DBA(CH) / 4], &le, 4,
                     MEMTXATTRS_UNSPECIFIED);
    s->dma[DBA(CH) / 4] += 4;
    s->left -= 4;
    if (!s->left) {
        asdmic_desc_end(s);
    }
}

static int16_t asdmic_sample(IngenicAsDmicState *s)
{
    double t;

    if (!s->tone_hz) {
        return 0;
    }
    t = (double)s->frame / asdmic_rate(s);
    return (int16_t)lrint(8192 * sin(2 * M_PI * s->tone_hz * t));
}

static void asdmic_make(IngenicAsDmicState *s, uint64_t frames)
{
    unsigned mics = extract32(s->dmic[CR0 / 4], 16, 4) + 1;
    bool b24 = extract32(s->dmic[CR0 / 4], 12, 2) != 0;

    while (frames-- && asdmic_running(s)) {
        int16_t v = asdmic_sample(s);
        unsigned m;

        for (m = 0; m < mics; m++) {
            if (b24) {
                asdmic_put32(s, (uint32_t)(int32_t)v << 16);
            } else if (s->half) {
                asdmic_put32(s, (s->half & 0xffff) | ((uint16_t)v << 16));
                s->half = 0;
            } else {
                s->half = 0x10000 | (uint16_t)v;
            }
        }
        s->frame++;
    }
}

static void asdmic_tick(void *opaque)
{
    IngenicAsDmicState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t due;

    if (!asdmic_running(s)) {
        return;
    }
    due = muldiv64(now - s->start_ns, asdmic_rate(s), NANOSECONDS_PER_SECOND);
    if (due > s->frame) {
        asdmic_make(s, due - s->frame);
    }
    if (asdmic_running(s)) {
        timer_mod(s->timer, now + TICK_NS);
    }
}

/* the stream may have started or stopped */
static void asdmic_kick(IngenicAsDmicState *s, bool was)
{
    bool run = asdmic_running(s);

    if (run && !was) {
        s->frame = 0;
        s->half = 0;
        s->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (s->dma[DDA(CH) / 4]) {
            asdmic_load(s);
        }
        timer_mod(s->timer, s->start_ns + TICK_NS);
    } else if (!run && was) {
        timer_del(s->timer);
    }
}

static uint32_t *asdmic_reg(IngenicAsDmicState *s, hwaddr off)
{
    if (off < DMA_REGS) {
        return &s->dma[off / 4];
    }
    if (off >= FIFO_BASE && off < FIFO_BASE + FIFO_REGS) {
        return &s->fifo[(off - FIFO_BASE) / 4];
    }
    if (off >= FMT_BASE && off < FMT_BASE + FMT_REGS) {
        return &s->fmt[(off - FMT_BASE) / 4];
    }
    if (off >= DBUS_BASE && off < DBUS_BASE + DBUS_REGS) {
        return &s->dbus[(off - DBUS_BASE) / 4];
    }
    if (off >= DMIC_BASE && off < DMIC_BASE + DMIC_REGS) {
        return &s->dmic[(off - DMIC_BASE) / 4];
    }
    return NULL;
}

static uint64_t ingenic_asdmic_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicAsDmicState *s = opaque;
    uint32_t *r;

    if (off == DBA(CH) && asdmic_running(s)) {
        asdmic_tick(s);                 /* bring the pointer up to date */
    }
    r = asdmic_reg(s, off);
    return r ? *r : 0;
}

static void ingenic_asdmic_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    IngenicAsDmicState *s = opaque;
    bool was = asdmic_running(s);
    uint32_t *r = asdmic_reg(s, off);

    if (!r) {
        return;
    }
    if (off < DMA_REGS && (off % 0x18) == 0x0C && off < 0x18 * NCH) {
        *r &= ~(val & 0x3c);            /* DSR: the flags clear */
        if (off == DSR(CH) && !(*r & (DSR_TT_INT | DSR_LTT_INT))) {
            s->dma[AIPR / 4] &= ~(1u << CH);
        }
        asdmic_update(s);
        return;
    }
    if (off == AIPR) {
        *r &= ~val;
        asdmic_update(s);
        return;
    }
    *r = val;
    if (off < DMA_REGS && (off % 0x18) == 0x08 && off < 0x18 * NCH) {
        *r &= ~DCR_RESET;               /* DCR's reset pulse */
    } else if (off == DGRR) {
        *r = 0;
    } else if (off == DMIC_BASE + CR0) {
        *r &= ~CR0_RESET;
    } else if (off >= FIFO_BASE && off < FIFO_BASE + FIFO_REGS &&
               ((off - FIFO_BASE) % 0x10) == 4) {
        *r &= ~4u;                      /* FCR's flush pulse */
    }
    asdmic_kick(s, was);
}

static const MemoryRegionOps ingenic_asdmic_ops = {
    .read = ingenic_asdmic_read,
    .write = ingenic_asdmic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_asdmic_reset(DeviceState *dev)
{
    IngenicAsDmicState *s = INGENIC_ASDMIC(dev);

    timer_del(s->timer);
    memset(s->dma, 0, sizeof(s->dma));
    memset(s->fifo, 0, sizeof(s->fifo));
    memset(s->fmt, 0, sizeof(s->fmt));
    memset(s->dbus, 0, sizeof(s->dbus));
    memset(s->dmic, 0, sizeof(s->dmic));
    s->frame = 0;
    s->left = 0;
    s->half = 0;
    asdmic_update(s);
}

static void ingenic_asdmic_init(Object *obj)
{
    IngenicAsDmicState *s = INGENIC_ASDMIC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_asdmic_ops, s,
                          TYPE_INGENIC_ASDMIC, AS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, asdmic_tick, s);
}

static const Property ingenic_asdmic_props[] = {
    DEFINE_PROP_UINT32("tone-hz", IngenicAsDmicState, tone_hz, 1000),
};

static const VMStateDescription vmstate_ingenic_asdmic = {
    .name = TYPE_INGENIC_ASDMIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(dma, IngenicAsDmicState, DMA_REGS / 4),
        VMSTATE_UINT32_ARRAY(fifo, IngenicAsDmicState, FIFO_REGS / 4),
        VMSTATE_UINT32_ARRAY(fmt, IngenicAsDmicState, FMT_REGS / 4),
        VMSTATE_UINT32_ARRAY(dbus, IngenicAsDmicState, DBUS_REGS / 4),
        VMSTATE_UINT32_ARRAY(dmic, IngenicAsDmicState, DMIC_REGS / 4),
        VMSTATE_UINT64(frame, IngenicAsDmicState),
        VMSTATE_INT64(start_ns, IngenicAsDmicState),
        VMSTATE_UINT32(left, IngenicAsDmicState),
        VMSTATE_UINT32(half, IngenicAsDmicState),
        VMSTATE_TIMER_PTR(timer, IngenicAsDmicState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_asdmic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_asdmic_reset);
    device_class_set_props(dc, ingenic_asdmic_props);
    dc->vmsd = &vmstate_ingenic_asdmic;
}

static const TypeInfo ingenic_asdmic_info = {
    .name = TYPE_INGENIC_ASDMIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAsDmicState),
    .instance_init = ingenic_asdmic_init,
    .class_init = ingenic_asdmic_class_init,
};

static void ingenic_asdmic_register_types(void)
{
    type_register_static(&ingenic_asdmic_info);
}

type_init(ingenic_asdmic_register_types)
