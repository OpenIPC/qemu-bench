/*
 * Allwinner "TWI driver" engine: the packet machine at +0x200 of the
 * sun8iw19 (V831/V833) TWI blocks, which the vendor Linux i2c-sunxi.c
 * uses when the DT says twi_drv_used.  The legacy mv64xxx-style registers
 * below it are upstream's allwinner.i2c-sun6i, mapped separately.
 *
 *   0x200 CTRL   bit 0 enable, 1 reset, 23:16 status code of a failure,
 *                27:24 result, 28 read mode, 31 start
 *   0x204 CFG    15:0 packets          0x208 SLV  15:9 address, 8 read
 *   0x20C FMT    23:16 address bytes, 15:0 data bytes (per packet)
 *   0x214 INTC   pending 3:0 (write 1 to clear), enables 19:16:
 *                complete, error, TX request, RX request
 *   0x218 DMAC   RX trigger 21:16 ...   0x21C FIFOC  TX count 5:0, TX
 *                clear 6, RX count 21:16, RX clear 22
 *   0x300 SENDF  0x304 RECVF
 *
 * A packet: START, the address bytes (a register number) from SENDF,
 * then -- read: repeated START with the read bit and the data bytes into
 * RECVF; write: the data bytes from SENDF -- and STOP.  Read mode with no
 * address bytes is a plain read.  START comes before the driver fills
 * SENDF, so the packet waits for its bytes.  A chip that does not answer
 * its address fails the transfer at once with the status code the driver
 * decodes (0x20 address + write, 0x48 address + read not acknowledged).
 * A packet longer than the FIFOs stops when SENDF runs dry or RECVF fills
 * and raises the TX or RX request for the driver to serve.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/fifo8.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AW_TWI "aw-twi"
OBJECT_DECLARE_SIMPLE_TYPE(AwTwiState, AW_TWI)

#define AW_TWI_SIZE         0x200       /* mapped at the block's +0x200 */
#define AW_TWI_FIFO         32

#define REG_CTRL            0x000
#define REG_CFG             0x004
#define REG_SLV             0x008
#define REG_FMT             0x00C
#define REG_BUSC            0x010
#define REG_INTC            0x014
#define REG_DMAC            0x018
#define REG_FIFOC           0x01C
#define REG_SENDF           0x100
#define REG_RECVF           0x104

#define CTRL_EN             (1u << 0)
#define CTRL_RST            (1u << 1)
#define CTRL_STA_SHIFT      16
#define CTRL_RESULT_SHIFT   24
#define CTRL_READ           (1u << 28)
#define CTRL_START          (1u << 31)
#define SLV_RD              (1u << 8)
#define INT_COM             (1u << 0)
#define INT_ERR             (1u << 1)
#define INT_TX_REQ          (1u << 2)
#define INT_RX_REQ          (1u << 3)
#define FIFOC_TX_CLR        (1u << 6)
#define FIFOC_RX_CLR        (1u << 22)

typedef enum {
    PH_IDLE,
    PH_ADDR,        /* sending the address bytes */
    PH_DATA,        /* data bytes, out or in */
} AwTwiPhase;

struct AwTwiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;

    uint32_t ctrl, cfg, slv, fmt, busc, intc, dmac;
    Fifo8 tx, rx;
    /* the transfer in progress */
    uint8_t phase;
    uint32_t pkt, left;
};

static void aw_twi_update_irq(AwTwiState *s)
{
    qemu_set_irq(s->irq, !!(s->intc & (s->intc >> 16) & 0xF));
}

static void aw_twi_finish(AwTwiState *s, uint8_t code)
{
    i2c_end_transfer(s->bus);
    s->phase = PH_IDLE;
    s->ctrl &= ~(CTRL_START | (0xFFu << CTRL_STA_SHIFT) |
                 (0xFu << CTRL_RESULT_SHIFT));
    if (code) {
        s->ctrl |= (uint32_t)code << CTRL_STA_SHIFT |
                   1u << CTRL_RESULT_SHIFT;
        s->intc |= INT_ERR;
    } else {
        s->ctrl |= 0xF8u << CTRL_STA_SHIFT;
        s->intc |= INT_COM;
        /* The driver collects what was read only if RX request is up. */
        if (!fifo8_is_empty(&s->rx)) {
            s->intc |= INT_RX_REQ;
        }
    }
    aw_twi_update_irq(s);
}

static bool aw_twi_reading(AwTwiState *s)
{
    return (s->slv & SLV_RD) || (s->ctrl & CTRL_READ);
}

/* Address the chip; false (transfer failed) when it does not answer. */
static bool aw_twi_start(AwTwiState *s, bool recv)
{
    uint8_t addr = (s->slv >> 9) & 0x7F;

    if (i2c_start_transfer(s->bus, addr, recv)) {
        aw_twi_finish(s, recv ? 0x48 : 0x20);
        return false;
    }
    return true;
}

/* Begin packet s->pkt. */
static bool aw_twi_packet(AwTwiState *s)
{
    uint32_t abytes = (s->fmt >> 16) & 0xFF;

    if (abytes) {
        s->phase = PH_ADDR;
        s->left = abytes;
        return aw_twi_start(s, false);
    }
    s->phase = PH_DATA;
    s->left = s->fmt & 0xFFFF;
    return aw_twi_start(s, aw_twi_reading(s));
}

/* Move the transfer on as far as the FIFOs let it. */
static void aw_twi_step(AwTwiState *s)
{
    while (s->phase != PH_IDLE) {
        if (s->phase == PH_ADDR) {
            if (!s->left) {
                s->phase = PH_DATA;
                s->left = s->fmt & 0xFFFF;
                if (aw_twi_reading(s) && !aw_twi_start(s, true)) {
                    return;
                }
                continue;
            }
            if (fifo8_is_empty(&s->tx)) {
                return;
            }
            if (i2c_send(s->bus, fifo8_pop(&s->tx))) {
                aw_twi_finish(s, 0x30);
                return;
            }
            s->left--;
            continue;
        }
        /* PH_DATA */
        if (!s->left) {
            i2c_end_transfer(s->bus);
            if (++s->pkt >= MAX(s->cfg & 0xFFFF, 1)) {
                aw_twi_finish(s, 0);
                return;
            }
            if (!aw_twi_packet(s)) {
                return;
            }
            continue;
        }
        if (aw_twi_reading(s)) {
            if (fifo8_is_full(&s->rx)) {
                return;
            }
            fifo8_push(&s->rx, i2c_recv(s->bus));
        } else {
            if (fifo8_is_empty(&s->tx)) {
                return;
            }
            if (i2c_send(s->bus, fifo8_pop(&s->tx))) {
                aw_twi_finish(s, 0x30);
                return;
            }
        }
        s->left--;
    }
}

/*
 * Run the transfer; when it stops for the FIFOs, ask the driver to refill
 * SENDF (TX request) or drain RECVF (RX request).
 */
static void aw_twi_run(AwTwiState *s)
{
    aw_twi_step(s);
    if (s->phase == PH_IDLE) {
        return;
    }
    if (s->phase == PH_DATA && aw_twi_reading(s)) {
        if (fifo8_is_full(&s->rx)) {
            s->intc |= INT_RX_REQ;
        }
    } else if (fifo8_is_empty(&s->tx)) {
        s->intc |= INT_TX_REQ;
    }
    aw_twi_update_irq(s);
}

static uint64_t aw_twi_read(void *opaque, hwaddr addr, unsigned size)
{
    AwTwiState *s = opaque;
    uint64_t v;

    switch (addr) {
    case REG_CTRL:
        return s->ctrl;
    case REG_CFG:
        return s->cfg;
    case REG_SLV:
        return s->slv;
    case REG_FMT:
        return s->fmt;
    case REG_BUSC:
        return s->busc;
    case REG_INTC:
        return s->intc;
    case REG_DMAC:
        return s->dmac;
    case REG_FIFOC:
        return fifo8_num_used(&s->tx) | fifo8_num_used(&s->rx) << 16;
    case REG_RECVF:
        v = fifo8_is_empty(&s->rx) ? 0 : fifo8_pop(&s->rx);
        aw_twi_run(s);
        return v;
    }
    return 0;
}

static void aw_twi_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    AwTwiState *s = opaque;

    switch (addr) {
    case REG_CTRL:
        if (val & CTRL_RST) {
            i2c_end_transfer(s->bus);
            s->phase = PH_IDLE;
            fifo8_reset(&s->tx);
            fifo8_reset(&s->rx);
            val &= ~CTRL_RST;
        }
        s->ctrl = (s->ctrl & ~(CTRL_EN | CTRL_READ)) |
                  (val & (CTRL_EN | CTRL_READ));
        if ((val & CTRL_START) && s->phase == PH_IDLE) {
            s->ctrl |= CTRL_START;
            s->pkt = 0;
            if (aw_twi_packet(s)) {
                aw_twi_run(s);
            }
        }
        break;
    case REG_CFG:
        s->cfg = val;
        break;
    case REG_SLV:
        s->slv = val;
        break;
    case REG_FMT:
        s->fmt = val;
        break;
    case REG_BUSC:
        s->busc = val;
        break;
    case REG_INTC:
        s->intc = (val & 0xF0000) | (s->intc & 0xF & ~val);
        aw_twi_update_irq(s);
        break;
    case REG_DMAC:
        s->dmac = val;
        break;
    case REG_FIFOC:
        if (val & FIFOC_TX_CLR) {
            fifo8_reset(&s->tx);
        }
        if (val & FIFOC_RX_CLR) {
            fifo8_reset(&s->rx);
        }
        break;
    case REG_SENDF:
        if (!fifo8_is_full(&s->tx)) {
            fifo8_push(&s->tx, val);
        }
        aw_twi_run(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "aw-twi: write 0x%03x = 0x%08x\n",
                      (unsigned)addr + 0x200, (uint32_t)val);
        break;
    }
}

static const MemoryRegionOps aw_twi_ops = {
    .read = aw_twi_read,
    .write = aw_twi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void aw_twi_reset_hold(Object *obj, ResetType type)
{
    AwTwiState *s = AW_TWI(obj);

    if (s->phase != PH_IDLE) {
        i2c_end_transfer(s->bus);
    }
    s->ctrl = 0x00F80000;
    s->cfg = 0x10000001;
    s->slv = 0;
    s->fmt = 0x00010001;
    s->busc = 0xC0;
    s->intc = 0;
    s->dmac = 0x00100010;
    fifo8_reset(&s->tx);
    fifo8_reset(&s->rx);
    s->phase = PH_IDLE;
}

static void aw_twi_reset_exit(Object *obj, ResetType type)
{
    aw_twi_update_irq(AW_TWI(obj));
}

static void aw_twi_init(Object *obj)
{
    AwTwiState *s = AW_TWI(obj);

    memory_region_init_io(&s->iomem, obj, &aw_twi_ops, s, TYPE_AW_TWI,
                          AW_TWI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
    fifo8_create(&s->tx, AW_TWI_FIFO);
    fifo8_create(&s->rx, AW_TWI_FIFO);
}

static void aw_twi_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_twi_reset_hold;
    rc->phases.exit = aw_twi_reset_exit;
}

static const TypeInfo aw_twi_info = {
    .name           = TYPE_AW_TWI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwTwiState),
    .instance_init  = aw_twi_init,
    .class_init     = aw_twi_class_init,
};

static void aw_twi_register_types(void)
{
    type_register_static(&aw_twi_info);
}

type_init(aw_twi_register_types)
