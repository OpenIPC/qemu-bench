/*
 * Fullhan I2C: a DesignWare APB I2C master.
 *
 * As the kernel's drivers/i2c/busses/i2c_fh_interrupt.c (a fork of
 * i2c-designware) drives it:
 *
 *   0x00 CON        0x04 TAR (7-bit target)       0x10 DATA_CMD
 *   0x14-0x28 SCL counts                           0x2C INTR_STAT
 *   0x30 INTR_MASK  0x34 RAW_INTR_STAT             0x38 RX_TL  0x3C TX_TL
 *   0x40 CLR_INTR, 0x44-0x68 CLR_<one interrupt> (reads clear)
 *   0x6C ENABLE     0x70 STATUS   0x74 TXFLR  0x78 RXFLR  0x7C SDA_HOLD
 *   0x80 TX_ABRT_SOURCE           0x9C ENABLE_STATUS
 *   0xF4 COMP_PARAM_1 (FIFO depths)  0xF8 COMP_VERSION  0xFC COMP_TYPE
 *
 * DATA_CMD: bits 7:0 the byte to send, bit 8 read instead, bit 9 STOP
 * after this byte, bit 10 RESTART before it.  The first command after
 * enabling starts a transfer to TAR in its direction; a change of
 * direction is a repeated start.  A target that does not acknowledge its
 * address or a byte aborts the transfer (TX_ABRT, with the reason in
 * TX_ABRT_SOURCE) and STOP_DET follows, as on the bus.
 *
 * Bytes move at once: the TX FIFO is always empty, so TX_EMPTY stands
 * while the controller is enabled (the driver unmasks it to be called to
 * fill the FIFO, and masks it once its messages are queued), and
 * RX_FULL follows the RX FIFO against RX_TL.  The driver reads the FIFO
 * depths from COMP_PARAM_1.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/fifo8.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_FH_I2C "fh-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(FhI2cState, FH_I2C)

#define FH_I2C_SIZE         0x1000
#define FH_I2C_FIFO         16

#define R_CON               0x00
#define R_TAR               0x04
#define R_SAR               0x08
#define R_DATA_CMD          0x10
#define R_INTR_STAT         0x2C
#define R_INTR_MASK         0x30
#define R_RAW_INTR_STAT     0x34
#define R_RX_TL             0x38
#define R_TX_TL             0x3C
#define R_CLR_INTR          0x40
#define R_CLR_RX_UNDER      0x44
#define R_CLR_RX_OVER       0x48
#define R_CLR_TX_OVER       0x4C
#define R_CLR_RD_REQ        0x50
#define R_CLR_TX_ABRT       0x54
#define R_CLR_RX_DONE       0x58
#define R_CLR_ACTIVITY      0x5C
#define R_CLR_STOP_DET      0x60
#define R_CLR_START_DET     0x64
#define R_CLR_GEN_CALL      0x68
#define R_ENABLE            0x6C
#define R_STATUS            0x70
#define R_TXFLR             0x74
#define R_RXFLR             0x78
#define R_TX_ABRT_SOURCE    0x80
#define R_ENABLE_STATUS     0x9C
#define R_COMP_PARAM_1      0xF4
#define R_COMP_VERSION      0xF8
#define R_COMP_TYPE         0xFC

#define INTR_RX_UNDER       (1u << 0)
#define INTR_RX_OVER        (1u << 1)
#define INTR_RX_FULL        (1u << 2)
#define INTR_TX_OVER        (1u << 3)
#define INTR_TX_EMPTY       (1u << 4)
#define INTR_RD_REQ         (1u << 5)
#define INTR_TX_ABRT        (1u << 6)
#define INTR_RX_DONE        (1u << 7)
#define INTR_ACTIVITY       (1u << 8)
#define INTR_STOP_DET       (1u << 9)
#define INTR_START_DET      (1u << 10)
#define INTR_GEN_CALL       (1u << 11)
#define INTR_LATCHED        (INTR_RX_UNDER | INTR_RX_OVER | INTR_TX_OVER | \
                             INTR_RD_REQ | INTR_TX_ABRT | INTR_RX_DONE | \
                             INTR_ACTIVITY | INTR_STOP_DET | \
                             INTR_START_DET | INTR_GEN_CALL)

#define CMD_READ            (1u << 8)
#define CMD_STOP            (1u << 9)
#define CMD_RESTART         (1u << 10)

#define ABRT_7B_ADDR_NOACK  (1u << 0)
#define ABRT_TXDATA_NOACK   (1u << 3)

#define STATUS_TFNF         (1u << 1)
#define STATUS_TFE          (1u << 2)
#define STATUS_RFNE         (1u << 3)
#define STATUS_RFF          (1u << 4)

enum { XFER_IDLE, XFER_SEND, XFER_RECV, XFER_ABORTED };

struct FhI2cState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;
    Fifo8 rx;

    uint32_t regs[FH_I2C_SIZE / 4];    /* the plain storage ones */
    uint32_t raw;               /* latched interrupts */
    uint32_t abrt_source;
    uint32_t enable;
    uint8_t xfer;
    uint8_t xfer_addr;          /* the target of the transfer in progress */
};

static uint32_t fh_i2c_raw(FhI2cState *s)
{
    uint32_t raw = s->raw;

    if (s->enable) {
        raw |= INTR_TX_EMPTY;
    }
    if (fifo8_num_used(&s->rx) > s->regs[R_RX_TL >> 2]) {
        raw |= INTR_RX_FULL;
    }
    return raw;
}

static void fh_i2c_update(FhI2cState *s)
{
    qemu_set_irq(s->irq, (fh_i2c_raw(s) & s->regs[R_INTR_MASK >> 2]) != 0);
}

static void fh_i2c_stop(FhI2cState *s)
{
    if (s->xfer == XFER_SEND || s->xfer == XFER_RECV) {
        i2c_end_transfer(s->bus);
    }
    s->xfer = XFER_IDLE;
    s->raw |= INTR_STOP_DET;
}

static void fh_i2c_abort(FhI2cState *s, uint32_t why)
{
    s->abrt_source |= why;
    s->raw |= INTR_TX_ABRT;
    fifo8_reset(&s->rx);
    fh_i2c_stop(s);
    /* The rest of the commands of this transfer are flushed. */
    s->xfer = XFER_ABORTED;
}

static void fh_i2c_data_cmd(FhI2cState *s, uint32_t cmd)
{
    bool rd = cmd & CMD_READ;
    uint8_t want = rd ? XFER_RECV : XFER_SEND;
    uint8_t addr = s->regs[R_TAR >> 2] & 0x7F;

    if (!s->enable || s->xfer == XFER_ABORTED) {
        return;
    }
    if (s->xfer != want || (cmd & CMD_RESTART)) {
        /*
         * A repeated start: QEMU's bus keeps the addressed target across
         * it, so one to another address must let go of the first.
         */
        if (s->xfer != XFER_IDLE && s->xfer_addr != addr) {
            i2c_end_transfer(s->bus);
        }
        s->raw |= INTR_START_DET | INTR_ACTIVITY;
        if (i2c_start_transfer(s->bus, addr, rd)) {
            fh_i2c_abort(s, ABRT_7B_ADDR_NOACK);
            return;
        }
        s->xfer = want;
        s->xfer_addr = addr;
    }
    if (rd) {
        uint8_t b = i2c_recv(s->bus);

        if (fifo8_is_full(&s->rx)) {
            s->raw |= INTR_RX_OVER;
        } else {
            fifo8_push(&s->rx, b);
        }
    } else if (i2c_send(s->bus, cmd & 0xFF)) {
        fh_i2c_abort(s, ABRT_TXDATA_NOACK);
        return;
    }
    if (cmd & CMD_STOP) {
        fh_i2c_stop(s);
    }
}

static uint64_t fh_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    FhI2cState *s = opaque;
    uint32_t v;

    switch (off) {
    case R_DATA_CMD:
        if (fifo8_is_empty(&s->rx)) {
            s->raw |= INTR_RX_UNDER;
            v = 0;
        } else {
            v = fifo8_pop(&s->rx);
        }
        break;
    case R_INTR_STAT:
        return fh_i2c_raw(s) & s->regs[R_INTR_MASK >> 2];
    case R_RAW_INTR_STAT:
        return fh_i2c_raw(s);
    case R_CLR_INTR:
        s->raw &= ~INTR_LATCHED;
        s->abrt_source = 0;
        v = 0;
        break;
    case R_CLR_RX_UNDER:  s->raw &= ~INTR_RX_UNDER;  v = 0; break;
    case R_CLR_RX_OVER:   s->raw &= ~INTR_RX_OVER;   v = 0; break;
    case R_CLR_TX_OVER:   s->raw &= ~INTR_TX_OVER;   v = 0; break;
    case R_CLR_RD_REQ:    s->raw &= ~INTR_RD_REQ;    v = 0; break;
    case R_CLR_TX_ABRT:
        s->raw &= ~INTR_TX_ABRT;
        s->abrt_source = 0;
        if (s->xfer == XFER_ABORTED) {
            s->xfer = XFER_IDLE;
        }
        v = 0;
        break;
    case R_CLR_RX_DONE:   s->raw &= ~INTR_RX_DONE;   v = 0; break;
    case R_CLR_ACTIVITY:  s->raw &= ~INTR_ACTIVITY;  v = 0; break;
    case R_CLR_STOP_DET:  s->raw &= ~INTR_STOP_DET;  v = 0; break;
    case R_CLR_START_DET: s->raw &= ~INTR_START_DET; v = 0; break;
    case R_CLR_GEN_CALL:  s->raw &= ~INTR_GEN_CALL;  v = 0; break;
    case R_ENABLE:
        return s->enable;
    case R_STATUS:
        v = STATUS_TFNF | STATUS_TFE;
        if (!fifo8_is_empty(&s->rx)) {
            v |= STATUS_RFNE;
        }
        if (fifo8_is_full(&s->rx)) {
            v |= STATUS_RFF;
        }
        return v;
    case R_TXFLR:
        return 0;
    case R_RXFLR:
        return fifo8_num_used(&s->rx);
    case R_TX_ABRT_SOURCE:
        return s->abrt_source;
    case R_ENABLE_STATUS:
        return s->enable;
    case R_COMP_PARAM_1:
        /* FIFO depths - 1, 32-bit APB, fast mode, interrupts */
        return ((FH_I2C_FIFO - 1) << 16) | ((FH_I2C_FIFO - 1) << 8) |
               (1 << 4) | (2 << 2) | 2;
    case R_COMP_VERSION:
        return 0x3131302A;      /* "110*" */
    case R_COMP_TYPE:
        return 0x44570140;
    default:
        return s->regs[off >> 2];
    }
    fh_i2c_update(s);
    return v;
}

static void fh_i2c_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhI2cState *s = opaque;

    switch (off) {
    case R_DATA_CMD:
        fh_i2c_data_cmd(s, val);
        break;
    case R_ENABLE:
        s->enable = val & 1;
        if (!s->enable) {
            if (s->xfer == XFER_SEND || s->xfer == XFER_RECV) {
                i2c_end_transfer(s->bus);
            }
            s->xfer = XFER_IDLE;
            fifo8_reset(&s->rx);
        }
        break;
    case R_INTR_STAT:
    case R_RAW_INTR_STAT:
    case R_CLR_INTR ... R_CLR_GEN_CALL:
    case R_STATUS:
    case R_TXFLR:
    case R_RXFLR:
    case R_TX_ABRT_SOURCE:
    case R_ENABLE_STATUS:
    case R_COMP_PARAM_1 ... R_COMP_TYPE:
        return;
    default:
        s->regs[off >> 2] = val;
        break;
    }
    fh_i2c_update(s);
}

static const MemoryRegionOps fh_i2c_ops = {
    .read = fh_i2c_read,
    .write = fh_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_i2c_reset(DeviceState *dev)
{
    FhI2cState *s = FH_I2C(dev);

    if (s->xfer == XFER_SEND || s->xfer == XFER_RECV) {
        i2c_end_transfer(s->bus);
    }
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_CON >> 2] = 0x7F;     /* master, fast, restart, no slave */
    s->regs[R_TAR >> 2] = 0x55;
    s->raw = 0;
    s->abrt_source = 0;
    s->enable = 0;
    s->xfer = XFER_IDLE;
    fifo8_reset(&s->rx);
    fh_i2c_update(s);
}

static void fh_i2c_init(Object *obj)
{
    FhI2cState *s = FH_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &fh_i2c_ops, s, TYPE_FH_I2C,
                          FH_I2C_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
    fifo8_create(&s->rx, FH_I2C_FIFO);
}

static const VMStateDescription vmstate_fh_i2c = {
    .name = TYPE_FH_I2C,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, FhI2cState, FH_I2C_SIZE / 4),
        VMSTATE_UINT32(raw, FhI2cState),
        VMSTATE_UINT32(abrt_source, FhI2cState),
        VMSTATE_UINT32(enable, FhI2cState),
        VMSTATE_UINT8(xfer, FhI2cState),
        VMSTATE_UINT8_V(xfer_addr, FhI2cState, 2),
        VMSTATE_FIFO8(rx, FhI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_i2c_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, fh_i2c_reset);
    dc->vmsd = &vmstate_fh_i2c;
}

static const TypeInfo fh_i2c_info = {
    .name           = TYPE_FH_I2C,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhI2cState),
    .instance_init  = fh_i2c_init,
    .class_init     = fh_i2c_class_init,
};

static void fh_i2c_register_types(void)
{
    type_register_static(&fh_i2c_info);
}

type_init(fh_i2c_register_types)
