/*
 * Xiongmai I2C controller (XM530: 0x10000000), as xm_i2c.ko drives it.
 *
 * A byte FIFO in front of the bus, one transaction per start command:
 *
 *   +0x00 CTRL    bit 6 pulses a TX FIFO reset, bit 7 an RX FIFO reset;
 *                 0x0B starts a write of WCNT bytes from the TX FIFO,
 *                 0x803 a read of RCNT bytes into the RX FIFO,
 *                 0x903 both in one transaction: WCNT bytes out, a
 *                 repeated start, RCNT bytes in (the XM530V200 / XM533 /
 *                 XM535 i2c.ko reads registers this way)
 *   +0x0C ADDR    7-bit slave address (ipctool passes 0x34 >> 1 for Sony)
 *   +0x10 TXDATA  one byte per write: register address MSB first, then data
 *   +0x14 RXDATA  one byte per read, in bus order
 *   +0x18 WCNT    bytes to write
 *   +0x1C RCNT    bytes to read
 *   +0x44 CLR     write 1s to clear ST bits
 *   +0x48 ST      bit 4 write done, bit 5 read done
 *
 * A register read is a write of the register address (0x0B, wait for
 * bit 4) followed by a read (0x803, wait for bit 5).  A slave that does not
 * ACK never sets the done bit, so the driver times out ("i2c_write timeout")
 * the way it does on an empty bus -- which is how sensor probing tells a
 * missing sensor from a present one.  Transfers finish at once.
 *
 * Read from the vendor xm_i2c.ko (i2c_write/i2c_read); no public source.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/sysbus.h"
#include "qemu/fifo8.h"
#include "qemu/log.h"
#include "qom/object.h"

#define TYPE_XM_I2C "xm-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(XmI2cState, XM_I2C)

#define R_CTRL      0x00
#define R_ADDR      0x0C
#define R_TXDATA    0x10
#define R_RXDATA    0x14
#define R_WCNT      0x18
#define R_RCNT      0x1C
#define R_CLR       0x44
#define R_ST        0x48

#define CTRL_TX_RESET   (1u << 6)
#define CTRL_RX_RESET   (1u << 7)
#define CTRL_START_WR   0x00B
#define CTRL_START_RD   0x803
#define CTRL_START_WRRD 0x903
#define CTRL_START_MASK 0x003
#define ST_WR_DONE      (1u << 4)
#define ST_RD_DONE      (1u << 5)

#define XM_I2C_FIFO     64
#define XM_I2C_SIZE     0x1000

struct XmI2cState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus *bus;
    Fifo8 tx, rx;
    uint32_t ctrl, addr, wcnt, rcnt, st;
};

/*
 * A transfer that cannot move its whole programmed count is not done: the
 * driver then times out as it would on silicon, rather than taking a short
 * register-address write for a complete one and reading the wrong register.
 * (The driver queues one byte past WCNT, a terminator, which is not sent.)
 */
static void xm_i2c_write_xfer(XmI2cState *s)
{
    uint32_t n = s->wcnt;
    bool ok;

    if (n > fifo8_num_used(&s->tx)) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-i2c: write of %u bytes with %u "
                      "queued\n", n, fifo8_num_used(&s->tx));
        fifo8_reset(&s->tx);
        return;
    }
    ok = !i2c_start_send(s->bus, s->addr & 0x7f);

    while (ok && n--) {
        ok = !i2c_send(s->bus, fifo8_pop(&s->tx));
    }
    i2c_end_transfer(s->bus);
    fifo8_reset(&s->tx);
    if (ok) {
        s->st |= ST_WR_DONE;
    }
}

static void xm_i2c_read_xfer(XmI2cState *s)
{
    uint32_t n = s->rcnt;

    fifo8_reset(&s->rx);
    if (n > XM_I2C_FIFO) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-i2c: read of %u bytes, FIFO is %u\n",
                      n, XM_I2C_FIFO);
        return;
    }
    if (i2c_start_recv(s->bus, s->addr & 0x7f)) {
        i2c_end_transfer(s->bus);
        return;
    }
    while (n--) {
        fifo8_push(&s->rx, i2c_recv(s->bus));
    }
    i2c_nack(s->bus);
    i2c_end_transfer(s->bus);
    s->st |= ST_RD_DONE;
}

/* 0x903: the register address out, a repeated start, the data in.  Only
 * read-done is set, which is all the driver waits for. */
static void xm_i2c_write_read_xfer(XmI2cState *s)
{
    uint32_t n = s->wcnt;
    bool ok;

    fifo8_reset(&s->rx);
    if (n > fifo8_num_used(&s->tx) || s->rcnt > XM_I2C_FIFO) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-i2c: write-read of %u+%u bytes "
                      "with %u queued\n", n, s->rcnt, fifo8_num_used(&s->tx));
        fifo8_reset(&s->tx);
        return;
    }
    ok = !i2c_start_send(s->bus, s->addr & 0x7f);
    while (ok && n--) {
        ok = !i2c_send(s->bus, fifo8_pop(&s->tx));
    }
    fifo8_reset(&s->tx);
    if (ok) {
        ok = !i2c_start_recv(s->bus, s->addr & 0x7f);
    }
    if (ok) {
        for (n = s->rcnt; n; n--) {
            fifo8_push(&s->rx, i2c_recv(s->bus));
        }
        i2c_nack(s->bus);
    }
    i2c_end_transfer(s->bus);
    if (ok) {
        s->st |= ST_RD_DONE;
    }
}

static uint64_t xm_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    XmI2cState *s = XM_I2C(opaque);

    switch (off) {
    case R_CTRL:
        return s->ctrl;
    case R_ADDR:
        return s->addr;
    case R_RXDATA:
        return fifo8_is_empty(&s->rx) ? 0 : fifo8_pop(&s->rx);
    case R_WCNT:
        return s->wcnt;
    case R_RCNT:
        return s->rcnt;
    case R_ST:
        return s->st;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-i2c: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void xm_i2c_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    XmI2cState *s = XM_I2C(opaque);

    switch (off) {
    case R_CTRL:
        if ((val & CTRL_TX_RESET) && !(s->ctrl & CTRL_TX_RESET)) {
            fifo8_reset(&s->tx);
        }
        if ((val & CTRL_RX_RESET) && !(s->ctrl & CTRL_RX_RESET)) {
            fifo8_reset(&s->rx);
        }
        s->ctrl = val;
        if (val == CTRL_START_WR) {
            xm_i2c_write_xfer(s);
        } else if (val == CTRL_START_RD) {
            xm_i2c_read_xfer(s);
        } else if (val == CTRL_START_WRRD) {
            xm_i2c_write_read_xfer(s);
        }
        /* The start bits clear when the transfer ends: the driver pulses
         * the FIFO resets as "old | bit" then "old", and old would
         * otherwise read back as a start command and launch another. */
        s->ctrl &= ~CTRL_START_MASK;
        break;
    case R_ADDR:
        s->addr = val;
        break;
    case R_TXDATA:
        if (!fifo8_is_full(&s->tx)) {
            fifo8_push(&s->tx, val);
        }
        break;
    case R_WCNT:
        s->wcnt = val;
        break;
    case R_RCNT:
        s->rcnt = val;
        break;
    case R_CLR:
        s->st &= ~val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-i2c: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        break;
    }
}

static const MemoryRegionOps xm_i2c_ops = {
    .read = xm_i2c_read,
    .write = xm_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_i2c_reset(DeviceState *dev)
{
    XmI2cState *s = XM_I2C(dev);

    fifo8_reset(&s->tx);
    fifo8_reset(&s->rx);
    s->ctrl = s->addr = s->wcnt = s->rcnt = s->st = 0;
}

static void xm_i2c_init(Object *obj)
{
    XmI2cState *s = XM_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &xm_i2c_ops, s, TYPE_XM_I2C,
                          XM_I2C_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
    fifo8_create(&s->tx, XM_I2C_FIFO);
    fifo8_create(&s->rx, XM_I2C_FIFO);
}

static void xm_i2c_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_i2c_reset);
}

static const TypeInfo xm_i2c_info = {
    .name          = TYPE_XM_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmI2cState),
    .instance_init = xm_i2c_init,
    .class_init    = xm_i2c_class_init,
};

static void xm_i2c_register_types(void)
{
    type_register_static(&xm_i2c_info);
}

type_init(xm_i2c_register_types)
