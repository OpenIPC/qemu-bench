/*
 * Ingenic I2C: a Synopsys DW_apb_i2c master under Ingenic register names
 * (0x10050000 + 0x1000 n; drivers/i2c/busses/i2c-v12-jz.c on 3.10,
 * i2c-ingenic.c on 4.4).
 *
 *   0x00 CON  0x04 TAR  0x10 DC (DATA_CMD: 8 read, 9 stop, 10 restart)
 *   0x2C INTST (masked)  0x30 INTM  0x34 RAW  0x38 RXTL  0x3C TXTL
 *   0x40.. read-to-clear: CINTR 0x40 (all), CRXUF 0x44, CRXOF 0x48,
 *     CTXOF 0x4C, CRDREQ 0x50, CTXABRT 0x54, CRXDN 0x58, CACT 0x5C,
 *     CSTP 0x60, CSTT 0x64, CGC 0x68
 *   0x6C ENB  0x70 STA  0x74 TXFLR  0x78 RXFLR  0x80 TXABRT  0x9C ENSTA
 *
 * Every DC write happens on the bus at once, so the TX FIFO is always
 * empty (STA TFE|TFNF, RAW TXEMP).  The first command after a stop sends
 * START and the address from TAR, as does a change of direction or the
 * restart bit; an address nobody acknowledges aborts the transfer
 * (TXABRT bit 0, RAW TXABT) and ends it with a stop, which is what the
 * drivers see from an absent sensor.  Reads fill a 64-byte RX FIFO;
 * RXFL is raised while RXFLR > RXTL.  ENSTA follows ENB: both drivers
 * wait for it on every enable and disable.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/fifo8.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_I2C "ingenic-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicI2cState, INGENIC_I2C)

#define R_CON       0x00
#define R_TAR       0x04
#define R_DC        0x10
#define R_INTST     0x2C
#define R_INTM      0x30
#define R_RAW       0x34
#define R_RXTL      0x38
#define R_TXTL      0x3C
#define R_CINTR     0x40
#define R_CRXUF     0x44
#define R_CRXOF     0x48
#define R_CTXOF     0x4C
#define R_CRDREQ    0x50
#define R_CTXABRT   0x54
#define R_CRXDN     0x58
#define R_CACT      0x5C
#define R_CSTP      0x60
#define R_CSTT      0x64
#define R_CGC       0x68
#define R_ENB       0x6C
#define R_STA       0x70
#define R_TXFLR     0x74
#define R_RXFLR     0x78
#define R_TXABRT    0x80
#define R_ENSTA     0x9C
#define I2C_REGS    0x100

#define DC_READ     (1u << 8)
#define DC_STOP     (1u << 9)
#define DC_RESTART  (1u << 10)

#define INT_RXUF    (1u << 0)
#define INT_RXOF    (1u << 1)
#define INT_RXFL    (1u << 2)
#define INT_TXOF    (1u << 3)
#define INT_TXEMP   (1u << 4)
#define INT_RDREQ   (1u << 5)
#define INT_TXABT   (1u << 6)
#define INT_RXDN    (1u << 7)
#define INT_ACT     (1u << 8)
#define INT_STP     (1u << 9)
#define INT_STT     (1u << 10)
#define INT_GC      (1u << 11)

#define STA_TFNF    (1u << 1)
#define STA_TFE     (1u << 2)
#define STA_RFNE    (1u << 3)
#define STA_RFF     (1u << 4)

#define ABRT_7B_ADDR_NOACK  (1u << 0)
#define ABRT_TXDATA_NOACK   (1u << 3)

#define FIFO_LEN    64

struct IngenicI2cState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;

    uint32_t regs[I2C_REGS / 4];
    uint32_t raw, abrt;
    Fifo8 rx;
    bool active, reading, aborted;
};

static uint32_t *i2c_reg(IngenicI2cState *s, hwaddr off)
{
    return &s->regs[off >> 2];
}

static void i2c_update(IngenicI2cState *s)
{
    if (fifo8_num_used(&s->rx) > *i2c_reg(s, R_RXTL)) {
        s->raw |= INT_RXFL;
    } else {
        s->raw &= ~INT_RXFL;
    }
    if (*i2c_reg(s, R_ENB) & 1) {
        s->raw |= INT_TXEMP;
    } else {
        s->raw &= ~INT_TXEMP;
    }
    qemu_set_irq(s->irq, (s->raw & *i2c_reg(s, R_INTM)) != 0);
}

static void i2c_stop(IngenicI2cState *s)
{
    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    s->active = false;
    s->raw |= INT_STP;
}

static void i2c_command(IngenicI2cState *s, uint32_t dc)
{
    bool read = dc & DC_READ;
    uint8_t addr = *i2c_reg(s, R_TAR) & 0x7f;

    if (!(*i2c_reg(s, R_ENB) & 1)) {
        return;
    }
    if (s->aborted) {
        /* the FIFO is flushed after an abort until TXABRT is cleared */
        return;
    }
    if (!s->active || (dc & DC_RESTART) || read != s->reading) {
        s->raw |= INT_STT | INT_ACT;
        if (i2c_start_transfer(s->bus, addr, read)) {
            s->abrt |= ABRT_7B_ADDR_NOACK;
            s->raw |= INT_TXABT;
            s->aborted = true;
            s->active = false;
            s->raw |= INT_STP;
            return;
        }
        s->active = true;
        s->reading = read;
    }
    if (read) {
        uint8_t b = i2c_recv(s->bus);

        if (fifo8_is_full(&s->rx)) {
            s->raw |= INT_RXOF;
        } else {
            fifo8_push(&s->rx, b);
        }
    } else if (i2c_send(s->bus, dc & 0xff)) {
        s->abrt |= ABRT_TXDATA_NOACK;
        s->raw |= INT_TXABT;
        s->aborted = true;
        i2c_stop(s);
        return;
    }
    if (dc & DC_STOP) {
        i2c_stop(s);
    }
}

/* read-to-clear interrupt bits */
static uint32_t i2c_clear(IngenicI2cState *s, hwaddr off)
{
    static const struct { hwaddr off; uint32_t bits; } clr[] = {
        { R_CRXUF, INT_RXUF }, { R_CRXOF, INT_RXOF }, { R_CTXOF, INT_TXOF },
        { R_CRDREQ, INT_RDREQ }, { R_CTXABRT, INT_TXABT },
        { R_CRXDN, INT_RXDN }, { R_CACT, INT_ACT }, { R_CSTP, INT_STP },
        { R_CSTT, INT_STT }, { R_CGC, INT_GC },
    };
    int i;

    if (off == R_CINTR) {
        s->raw &= INT_RXFL | INT_TXEMP;
        s->abrt = 0;
        s->aborted = false;
        return 0;
    }
    for (i = 0; i < ARRAY_SIZE(clr); i++) {
        if (clr[i].off == off) {
            s->raw &= ~clr[i].bits;
            if (off == R_CTXABRT) {
                s->abrt = 0;
                s->aborted = false;
            }
        }
    }
    return 0;
}

static uint64_t ingenic_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicI2cState *s = opaque;
    uint32_t v;

    if (off >= I2C_REGS) {
        return 0;
    }
    switch (off) {
    case R_DC:
        if (fifo8_is_empty(&s->rx)) {
            s->raw |= INT_RXUF;
            v = 0;
        } else {
            v = fifo8_pop(&s->rx);
        }
        break;
    case R_INTST:
        v = s->raw & *i2c_reg(s, R_INTM);
        break;
    case R_RAW:
        v = s->raw;
        break;
    case R_CINTR ... R_CGC:
        v = i2c_clear(s, off);
        break;
    case R_STA:
        v = STA_TFNF | STA_TFE;
        if (!fifo8_is_empty(&s->rx)) {
            v |= STA_RFNE;
        }
        if (fifo8_is_full(&s->rx)) {
            v |= STA_RFF;
        }
        break;
    case R_TXFLR:
        v = 0;
        break;
    case R_RXFLR:
        v = fifo8_num_used(&s->rx);
        break;
    case R_TXABRT:
        v = s->abrt;
        break;
    case R_ENSTA:
        v = *i2c_reg(s, R_ENB) & 1;
        break;
    default:
        v = *i2c_reg(s, off);
        break;
    }
    i2c_update(s);
    return v;
}

static void ingenic_i2c_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicI2cState *s = opaque;

    if (off >= I2C_REGS) {
        return;
    }
    switch (off) {
    case R_DC:
        i2c_command(s, val);
        break;
    case R_ENB:
        *i2c_reg(s, off) = val & 1;
        if (!(val & 1)) {
            if (s->active) {
                i2c_end_transfer(s->bus);
                s->active = false;
            }
            fifo8_reset(&s->rx);
            s->aborted = false;
        }
        break;
    case R_INTST:
    case R_RAW:
    case R_STA:
    case R_TXFLR:
    case R_RXFLR:
    case R_TXABRT:
    case R_ENSTA:
        break;
    default:
        *i2c_reg(s, off) = val;
        break;
    }
    i2c_update(s);
}

static const MemoryRegionOps ingenic_i2c_ops = {
    .read = ingenic_i2c_read,
    .write = ingenic_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_i2c_reset(DeviceState *dev)
{
    IngenicI2cState *s = INGENIC_I2C(dev);

    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    memset(s->regs, 0, sizeof(s->regs));
    s->raw = s->abrt = 0;
    s->active = s->reading = s->aborted = false;
    fifo8_reset(&s->rx);
    i2c_update(s);
}

static void ingenic_i2c_init(Object *obj)
{
    IngenicI2cState *s = INGENIC_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_i2c_ops, s,
                          TYPE_INGENIC_I2C, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
    fifo8_create(&s->rx, FIFO_LEN);
}

static const VMStateDescription vmstate_ingenic_i2c = {
    .name = TYPE_INGENIC_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicI2cState, I2C_REGS / 4),
        VMSTATE_UINT32(raw, IngenicI2cState),
        VMSTATE_UINT32(abrt, IngenicI2cState),
        VMSTATE_FIFO8(rx, IngenicI2cState),
        VMSTATE_BOOL(active, IngenicI2cState),
        VMSTATE_BOOL(reading, IngenicI2cState),
        VMSTATE_BOOL(aborted, IngenicI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_i2c_reset);
    dc->vmsd = &vmstate_ingenic_i2c;
}

static const TypeInfo ingenic_i2c_info = {
    .name = TYPE_INGENIC_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicI2cState),
    .instance_init = ingenic_i2c_init,
    .class_init = ingenic_i2c_class_init,
};

static void ingenic_i2c_register_types(void)
{
    type_register_static(&ingenic_i2c_info);
}

type_init(ingenic_i2c_register_types)
