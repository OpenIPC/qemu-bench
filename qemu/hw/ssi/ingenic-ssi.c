/*
 * Ingenic SSI: the T-series SPI master (SSI0 at 0x10043000, INTC 9;
 * SSI1 at 0x10044000, INTC 8, on T21, T30, T31, T40, T41).
 *
 * As drivers/spi/jz_spi.[ch] of the 3.10 kernels (and ingenic_spi.c of
 * 4.4, the same block) use it:
 *
 *   0x00 DR    a write queues a frame to send, a read takes one received
 *   0x04 CR0   15 SSIE (enable), 14 TIE / 13 RIE (FIFO half empty / full
 *              interrupts), 12 TEIE / 11 REIE (underrun / overrun ones),
 *              10 LOOP, 7 EACLRUN, 6 FSEL (CE2 else CE), 2 TFLUSH /
 *              1 RFLUSH (pulses), 0 DISREV (do not keep what comes in);
 *              19:16 bit orders (MSB first here)
 *   0x08 CR1   23 UNFIN (keep CE asserted when the TX FIFO runs dry),
 *              19:16 TTRG, 11:8 RTRG (FIFO triggers, 8 entries a step),
 *              7:3 FLEN (frame bits - 2), 30 FRMHL, 1 PHA, 0 POL
 *   0x0C SR    23:16 TX entries, 15:8 RX entries, 7 END, 6 BUSY, 5 TFF,
 *              4 RFE, 3 TFHE, 2 RFHF, 1 UNDR, 0 OVER (UNDR and OVER
 *              cleared by writing them 0)
 *   0x10 ITR / 0x14 ICR / 0x18 GR (clock) / 0x1C RCNT   storage
 *
 * A frame goes out on the SSI bus as soon as it is queued (the FIFO
 * never holds one), and what comes back -- the frame itself in LOOP --
 * is kept in the 128-entry RX FIFO unless DISREV.  The TX FIFO running
 * dry with UNFIN set is an underrun (UNDR): the driver's PIO transfers
 * end on that interrupt; with UNFIN clear the transfer ends there (END)
 * and CE is released.  CE (or CE2 with FSEL) is the chip select of the
 * bus's peripheral with cs 0 (1), so "-device w25q80,bus=ssi0" puts a
 * flash on SSI0's CE.  TFHE is the TX FIFO at or below its trigger (it
 * always is), RFHF the RX FIFO above its own; the interrupt is those and
 * UNDR / OVER under their enables.
 *
 * DMA: "drq-tx" (PDMA request type 0x16 for SSI0, 0x18 for SSI1) is high
 * while enabled and the RX FIFO has room for what a frame brings back,
 * "drq-rx" (0x17 / 0x19) while the RX FIFO holds a frame.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_SSI "ingenic-ssi"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicSsiState, INGENIC_SSI)

#define R_DR            0x00
#define R_CR0           0x04
#define R_CR1           0x08
#define R_SR            0x0C
#define R_ITR           0x10
#define R_ICR           0x14
#define R_GR            0x18
#define R_RCNT          0x1C

#define CR0_SSIE        (1u << 15)
#define CR0_TIE         (1u << 14)
#define CR0_RIE         (1u << 13)
#define CR0_TEIE        (1u << 12)
#define CR0_REIE        (1u << 11)
#define CR0_LOOP        (1u << 10)
#define CR0_FSEL        (1u << 6)
#define CR0_TFLUSH      (1u << 2)
#define CR0_RFLUSH      (1u << 1)
#define CR0_DISREV      (1u << 0)

#define CR1_UNFIN       (1u << 23)

#define SR_END          (1u << 7)
#define SR_RFE          (1u << 4)
#define SR_TFHE         (1u << 3)
#define SR_RFHF         (1u << 2)
#define SR_UNDR         (1u << 1)
#define SR_OVER         (1u << 0)

#define FIFO_DEPTH      128

struct IngenicSsiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *bus;
    qemu_irq irq, drq_tx, drq_rx;
    char *bus_name;

    uint32_t cr0, cr1, itr, icr, gr, rcnt;
    uint32_t flags;                 /* SR_UNDR | SR_OVER | SR_END */
    uint32_t rx[FIFO_DEPTH];
    uint32_t rx_head, rx_count;
    int32_t cs_on;                  /* the CE asserted, -1: none */
};

static uint32_t ssi_frame_mask(IngenicSsiState *s)
{
    int bits = extract32(s->cr1, 3, 5) + 2;

    return bits >= 32 ? UINT32_MAX : (1u << bits) - 1;
}

/* assert CE (or CE2) of the bus's peripheral, or release them all */
static void ssi_set_cs(IngenicSsiState *s, int cs)
{
    int i;

    if (cs == s->cs_on) {
        return;
    }
    for (i = 0; i < 2; i++) {
        DeviceState *dev = ssi_get_cs(s->bus, i);

        if (dev && (i == s->cs_on || i == cs)) {
            /* the peripherals' chip selects are active low */
            qemu_set_irq(qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0),
                         i != cs);
        }
    }
    s->cs_on = cs;
}

static uint32_t ssi_sr(IngenicSsiState *s)
{
    uint32_t sr = s->flags | SR_TFHE;
    uint32_t rtrg = extract32(s->cr1, 8, 4) * 8;

    sr |= (s->rx_count & 0xff) << 8;
    if (!s->rx_count) {
        sr |= SR_RFE;
    }
    if (s->rx_count > rtrg) {
        sr |= SR_RFHF;
    }
    return sr;
}

static void ssi_update(IngenicSsiState *s)
{
    uint32_t sr = ssi_sr(s);
    bool on = s->cr0 & CR0_SSIE;

    qemu_set_irq(s->irq, on && (((s->cr0 & CR0_TIE) && (sr & SR_TFHE)) ||
                                ((s->cr0 & CR0_RIE) && (sr & SR_RFHF)) ||
                                ((s->cr0 & CR0_TEIE) && (sr & SR_UNDR)) ||
                                ((s->cr0 & CR0_REIE) && (sr & SR_OVER))));
    qemu_set_irq(s->drq_tx, on && ((s->cr0 & CR0_DISREV) ||
                                   s->rx_count < FIFO_DEPTH));
    qemu_set_irq(s->drq_rx, on && s->rx_count);
}

/* the TX FIFO is empty again: an underrun, or the end of the transfer */
static void ssi_tx_empty(IngenicSsiState *s)
{
    if (s->cr1 & CR1_UNFIN) {
        s->flags |= SR_UNDR;
    } else {
        s->flags |= SR_END;
        ssi_set_cs(s, -1);
    }
}

static void ssi_send(IngenicSsiState *s, uint32_t v)
{
    uint32_t in;

    if (!(s->cr0 & CR0_SSIE)) {
        return;
    }
    v &= ssi_frame_mask(s);
    s->flags &= ~SR_END;
    ssi_set_cs(s, (s->cr0 & CR0_FSEL) ? 1 : 0);
    in = ssi_transfer(s->bus, v);
    if (s->cr0 & CR0_LOOP) {
        in = v;
    }
    if (!(s->cr0 & CR0_DISREV)) {
        if (s->rx_count < FIFO_DEPTH) {
            s->rx[(s->rx_head + s->rx_count++) % FIFO_DEPTH] =
                in & ssi_frame_mask(s);
        } else {
            s->flags |= SR_OVER;
        }
    }
    ssi_tx_empty(s);
}

static uint64_t ingenic_ssi_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicSsiState *s = opaque;
    uint32_t v = 0;

    switch (off) {
    case R_DR:
        if (s->rx_count) {
            v = s->rx[s->rx_head];
            s->rx_head = (s->rx_head + 1) % FIFO_DEPTH;
            s->rx_count--;
            ssi_update(s);
        }
        return v;
    case R_CR0:
        return s->cr0;
    case R_CR1:
        return s->cr1;
    case R_SR:
        return ssi_sr(s);
    case R_ITR:
        return s->itr;
    case R_ICR:
        return s->icr;
    case R_GR:
        return s->gr;
    case R_RCNT:
        return s->rcnt;
    }
    return 0;
}

static void ingenic_ssi_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicSsiState *s = opaque;

    switch (off) {
    case R_DR:
        ssi_send(s, val);
        break;
    case R_CR0:
        s->cr0 = val & ~(CR0_TFLUSH | CR0_RFLUSH);
        if (val & CR0_RFLUSH) {
            s->rx_head = s->rx_count = 0;
        }
        if (!(s->cr0 & CR0_SSIE)) {
            ssi_set_cs(s, -1);
        }
        break;
    case R_CR1:
        s->cr1 = val;
        if (!(s->cr1 & CR1_UNFIN) && s->cs_on >= 0) {
            /* the FIFO is empty: the transfer ends with UNFIN */
            s->flags |= SR_END;
            ssi_set_cs(s, -1);
        }
        break;
    case R_SR:
        /* UNDR and OVER clear when written 0 */
        s->flags &= val | ~(SR_UNDR | SR_OVER);
        break;
    case R_ITR:
        s->itr = val;
        break;
    case R_ICR:
        s->icr = val;
        break;
    case R_GR:
        s->gr = val;
        break;
    case R_RCNT:
        s->rcnt = val;
        break;
    }
    ssi_update(s);
}

static const MemoryRegionOps ingenic_ssi_ops = {
    .read = ingenic_ssi_read,
    .write = ingenic_ssi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_ssi_reset(DeviceState *dev)
{
    IngenicSsiState *s = INGENIC_SSI(dev);

    s->cr0 = s->cr1 = s->itr = s->icr = s->gr = s->rcnt = 0;
    s->flags = SR_END;
    s->rx_head = s->rx_count = 0;
    s->cs_on = -1;
    ssi_update(s);
}

static void ingenic_ssi_realize(DeviceState *dev, Error **errp)
{
    IngenicSsiState *s = INGENIC_SSI(dev);

    s->bus = ssi_create_bus(dev, s->bus_name ? s->bus_name : "ssi");
}

static void ingenic_ssi_init(Object *obj)
{
    IngenicSsiState *s = INGENIC_SSI(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_ssi_ops, s,
                          TYPE_INGENIC_SSI, 0x1000);
    /* the PDMA reads and writes DR from inside a DR access */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->drq_tx, "drq-tx", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->drq_rx, "drq-rx", 1);
}

static const Property ingenic_ssi_props[] = {
    DEFINE_PROP_STRING("bus-name", IngenicSsiState, bus_name),
};

static const VMStateDescription vmstate_ingenic_ssi = {
    .name = TYPE_INGENIC_SSI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr0, IngenicSsiState),
        VMSTATE_UINT32(cr1, IngenicSsiState),
        VMSTATE_UINT32(itr, IngenicSsiState),
        VMSTATE_UINT32(icr, IngenicSsiState),
        VMSTATE_UINT32(gr, IngenicSsiState),
        VMSTATE_UINT32(rcnt, IngenicSsiState),
        VMSTATE_UINT32(flags, IngenicSsiState),
        VMSTATE_UINT32_ARRAY(rx, IngenicSsiState, FIFO_DEPTH),
        VMSTATE_UINT32(rx_head, IngenicSsiState),
        VMSTATE_UINT32(rx_count, IngenicSsiState),
        VMSTATE_INT32(cs_on, IngenicSsiState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ssi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_ssi_realize;
    device_class_set_legacy_reset(dc, ingenic_ssi_reset);
    device_class_set_props(dc, ingenic_ssi_props);
    dc->vmsd = &vmstate_ingenic_ssi;
}

static const TypeInfo ingenic_ssi_info = {
    .name = TYPE_INGENIC_SSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicSsiState),
    .instance_init = ingenic_ssi_init,
    .class_init = ingenic_ssi_class_init,
};

static void ingenic_ssi_register_types(void)
{
    type_register_static(&ingenic_ssi_info);
}

type_init(ingenic_ssi_register_types)
