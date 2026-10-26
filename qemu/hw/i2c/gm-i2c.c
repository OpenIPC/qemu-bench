/*
 * Grain Media FTI2C010 I2C controller.
 *
 * Faraday's byte-at-a-time I2C master at 0x91200000 on the GM813x
 * (drivers/i2c/busses/i2c-ftiic010-new-intr-mode.c and ftiic010.h of the
 * GM 3.3 kernel; U-Boot's CONFIG_I2C_GM):
 *
 *   0x00 CR   0 reset (self clearing), 1 enable, 2 SCL enable, 4 START,
 *             5 STOP, 6 NAK the byte received, 7 transfer a byte,
 *             8 / 9 / 10 / 11 interrupt on byte sent / byte received /
 *             bus error (NAK) / STOP
 *   0x04 SR   1 ACK, 2 busy, 4 byte transmitted, 5 byte received, 6 bus
 *             error -- no ACK -- , 7 STOP done; cleared by reading
 *   0x08 CDR clock divider   0x0C DR data   0x14 TGSR glitch / setup
 *   0x18 BMR bus levels (SDA, SCL)   0x30 revision   0x34 features
 *
 * A write of CR with "transfer a byte" set moves DR: with START it is the
 * address byte (bit 0 = read), otherwise a data byte out or, in a read,
 * a byte in -- NAKed when CR says so -- and STOP ends the transfer.  The
 * byte takes a few microseconds, after which SR and the interrupt report
 * it: the kernel's interrupt mode sends the next byte from the handler.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_I2C "gm-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(GmI2cState, GM_I2C)

#define GM_I2C_SIZE         0x1000
#define GM_I2C_BYTE_NS      10000       /* 100 kHz: ~90 us; be quicker */

#define R_CR                0x00
#define R_SR                0x04
#define R_CDR               0x08
#define R_DR                0x0C
#define R_SAR               0x10
#define R_TGSR              0x14
#define R_BMR               0x18
#define R_REV               0x30
#define R_FEAT              0x34

#define CR_RST              (1u << 0)
#define CR_START            (1u << 4)
#define CR_STOP             (1u << 5)
#define CR_NAK              (1u << 6)
#define CR_TB               (1u << 7)
#define CR_DTI              (1u << 8)
#define CR_DRI              (1u << 9)
#define CR_BERRI            (1u << 10)
#define CR_STOPI            (1u << 11)

#define SR_RW               (1u << 0)
#define SR_ACK              (1u << 1)
#define SR_BUSY             (1u << 2)
#define SR_DT               (1u << 4)
#define SR_DR               (1u << 5)
#define SR_BERR             (1u << 6)
#define SR_STOP             (1u << 7)

struct GmI2cState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t cr;
    uint32_t sr;
    uint32_t pending;           /* SR bits the byte in flight will set */
    uint32_t cdr;
    uint32_t dr;
    uint32_t sar;
    uint32_t tgsr;
    bool active;                /* a transfer is open on the bus */
    bool reading;
};

static void gm_i2c_update(GmI2cState *s)
{
    bool level = ((s->sr & SR_DT) && (s->cr & CR_DTI)) ||
                 ((s->sr & SR_DR) && (s->cr & CR_DRI)) ||
                 ((s->sr & SR_BERR) && (s->cr & CR_BERRI)) ||
                 ((s->sr & SR_STOP) && (s->cr & CR_STOPI));

    qemu_set_irq(s->irq, level);
}

static void gm_i2c_done(void *opaque)
{
    GmI2cState *s = opaque;

    s->sr = (s->sr & ~SR_BUSY) | s->pending;
    s->pending = 0;
    s->cr &= ~(CR_TB | CR_START | CR_STOP | CR_NAK);
    gm_i2c_update(s);
}

static void gm_i2c_stop(GmI2cState *s)
{
    if (s->active) {
        i2c_end_transfer(s->bus);
        s->active = false;
    }
    s->pending |= SR_STOP;
}

/* Move one byte as CR says. */
static void gm_i2c_transfer(GmI2cState *s, uint32_t cr)
{
    uint8_t byte = s->dr & 0xFF;

    s->pending = 0;
    if (cr & CR_START) {
        if (s->active) {
            i2c_end_transfer(s->bus);   /* a repeated START */
        }
        s->reading = byte & 1;
        s->active = true;
        if (i2c_start_transfer(s->bus, byte >> 1, s->reading)) {
            i2c_end_transfer(s->bus);
            s->active = false;
            s->pending = SR_BERR;
        } else {
            s->pending = SR_DT | SR_ACK | (s->reading ? SR_RW : 0);
        }
    } else if (!s->active) {
        s->pending = SR_BERR;
    } else if (s->reading) {
        s->dr = i2c_recv(s->bus);
        if (cr & CR_NAK) {
            i2c_nack(s->bus);
        }
        s->pending = SR_DR | SR_RW;
    } else if (i2c_send(s->bus, byte)) {
        s->pending = SR_BERR;
    } else {
        s->pending = SR_DT | SR_ACK;
    }
    if ((cr & CR_STOP) || (s->pending & SR_BERR)) {
        gm_i2c_stop(s);
    }
    s->sr |= SR_BUSY;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        GM_I2C_BYTE_NS);
}

static uint64_t gm_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    GmI2cState *s = opaque;
    uint32_t v;

    switch (off) {
    case R_CR:
        return s->cr;
    case R_SR:
        v = s->sr;
        s->sr &= SR_BUSY;       /* the events clear on read */
        gm_i2c_update(s);
        return v;
    case R_CDR:
        return s->cdr;
    case R_DR:
        return s->dr;
    case R_SAR:
        return s->sar;
    case R_TGSR:
        return s->tgsr;
    case R_BMR:
        return 3;               /* SDA and SCL released */
    case R_REV:
        return 0x00010601;
    case R_FEAT:
        return 0;
    }
    return 0;
}

static void gm_i2c_reset_ctrl(GmI2cState *s)
{
    timer_del(s->timer);
    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    s->active = false;
    s->reading = false;
    s->cr = 0;
    s->sr = 0;
    s->pending = 0;
    s->dr = 0;
    gm_i2c_update(s);
}

static void gm_i2c_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmI2cState *s = opaque;

    switch (off) {
    case R_CR:
        if (val & CR_RST) {
            gm_i2c_reset_ctrl(s);
            return;
        }
        s->cr = val;
        if (val & CR_TB) {
            gm_i2c_transfer(s, val);
        } else if (val & CR_STOP) {
            gm_i2c_stop(s);
            s->sr |= SR_BUSY;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                GM_I2C_BYTE_NS);
        }
        gm_i2c_update(s);
        break;
    case R_CDR:
        s->cdr = val & 0x3FFFF;
        break;
    case R_DR:
        s->dr = val & 0xFF;
        break;
    case R_SAR:
        s->sar = val;
        break;
    case R_TGSR:
        s->tgsr = val & 0x1FFF;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: write 0x%" HWADDR_PRIx " = 0x%"
                      PRIx64 "\n", __func__, off, val);
        break;
    }
}

static const MemoryRegionOps gm_i2c_ops = {
    .read = gm_i2c_read,
    .write = gm_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_i2c_reset(DeviceState *dev)
{
    GmI2cState *s = GM_I2C(dev);

    gm_i2c_reset_ctrl(s);
    s->cdr = 0;
    s->sar = 0;
    s->tgsr = 0x405;
}

static void gm_i2c_realize(DeviceState *dev, Error **errp)
{
    GmI2cState *s = GM_I2C(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_i2c_ops, s,
                          TYPE_GM_I2C, GM_I2C_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(dev, "i2c");
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_i2c_done, s);
}

static const VMStateDescription vmstate_gm_i2c = {
    .name = TYPE_GM_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmI2cState),
        VMSTATE_UINT32(cr, GmI2cState),
        VMSTATE_UINT32(sr, GmI2cState),
        VMSTATE_UINT32(pending, GmI2cState),
        VMSTATE_UINT32(cdr, GmI2cState),
        VMSTATE_UINT32(dr, GmI2cState),
        VMSTATE_UINT32(sar, GmI2cState),
        VMSTATE_UINT32(tgsr, GmI2cState),
        VMSTATE_BOOL(active, GmI2cState),
        VMSTATE_BOOL(reading, GmI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_i2c_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_i2c_realize;
    device_class_set_legacy_reset(dc, gm_i2c_reset);
    dc->vmsd = &vmstate_gm_i2c;
}

static const TypeInfo gm_i2c_info = {
    .name           = TYPE_GM_I2C,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmI2cState),
    .class_init     = gm_i2c_class_init,
};

static void gm_i2c_register_types(void)
{
    type_register_static(&gm_i2c_info);
}

type_init(gm_i2c_register_types)
