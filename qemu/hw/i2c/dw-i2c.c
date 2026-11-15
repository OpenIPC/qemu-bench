/*
 * Generic Synopsys DesignWare I2C controller (master mode).
 *
 * This is the vanilla "snps,designware-i2c" register layout that the
 * mainline Linux i2c-designware-common.c/i2c-designware-master.c
 * drivers drive directly -- DISTINCT from this tree's own
 * hisi-i2c-dw.c, which (despite its similar name) models a
 * HiSilicon-specific "AUTO mode" register protocol used by the
 * drivers/i2c/busses/i2c-hisilicon.c vendor driver on CV200-family
 * SoCs; the two are not register-compatible, confirmed by comparing
 * i2c-hisilicon.c's AUTO/TX_RX single-shot transaction registers
 * against i2c-designware-core.h's real DW_IC_* offsets (e.g. this
 * device's DATA_CMD lives at 0x10 with FIFO semantics, not a single
 * TX_RX trigger register at 0xB4). Needed when an SoC's devicetree
 * uses the generic "snps,designware-i2c" compatible string and the
 * guest kernel's own generic driver (not a vendor-specific one) drives
 * it -- confirmed live on Axera AX620E: without DW_IC_COMP_TYPE
 * reading back the expected magic, i2c_dw_set_reg_access() in the
 * real driver returns -ENODEV outright and no /dev/i2c-N node is ever
 * created, which in turn starves any sensor on that bus (its
 * userspace driver's own open("/dev/i2c-N") fails).
 *
 * All register offsets/values below are taken directly from the real
 * kernel's own drivers/i2c/busses/i2c-designware-core.h.
 *
 * Processing model: each DATA_CMD write is executed synchronously
 * against the guest's actual I2C bus (QEMU's i2c_start_send/
 * i2c_start_recv/i2c_send/i2c_recv/i2c_end_transfer) at the moment
 * it's written, rather than modelling the real FIFO's pipelining --
 * the real driver only ever checks TXFLR/RXFLR to decide how many
 * more bytes it may still push/pop in its current loop iteration, and
 * is content to see "space/data available immediately" (TXFLR always
 * reports empty since we drain complete immediately; RXFLR reports
 * however many received bytes are queued and not yet read back). A
 * START is issued when a transfer begins from idle or when the
 * driver's own RESTART bit (DATA_CMD bit 10) is set (exactly the
 * cases where the real driver intends a fresh/repeated START); a STOP
 * is issued when the driver's own STOP bit (bit 9) is set on the
 * write that completes a message.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_DW_I2C "dw-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(DwI2cState, DW_I2C)

#define DW_I2C_REG_SIZE  0x100

/* Register offsets (real DW_IC_* layout) */
#define R_CON            0x00
#define R_TAR            0x04
#define R_SAR            0x08
#define R_DATA_CMD       0x10
#define R_SS_SCL_HCNT    0x14
#define R_SS_SCL_LCNT    0x18
#define R_FS_SCL_HCNT    0x1c
#define R_FS_SCL_LCNT    0x20
#define R_HS_SCL_HCNT    0x24
#define R_HS_SCL_LCNT    0x28
#define R_INTR_STAT      0x2c
#define R_INTR_MASK      0x30
#define R_RAW_INTR_STAT  0x34
#define R_RX_TL          0x38
#define R_TX_TL          0x3c
#define R_CLR_INTR       0x40
#define R_CLR_RX_UNDER   0x44
#define R_CLR_RX_OVER    0x48
#define R_CLR_TX_OVER    0x4c
#define R_CLR_RD_REQ     0x50
#define R_CLR_TX_ABRT    0x54
#define R_CLR_RX_DONE    0x58
#define R_CLR_ACTIVITY   0x5c
#define R_CLR_STOP_DET   0x60
#define R_CLR_START_DET  0x64
#define R_CLR_GEN_CALL   0x68
#define R_ENABLE         0x6c
#define R_STATUS         0x70
#define R_TXFLR          0x74
#define R_RXFLR          0x78
#define R_SDA_HOLD       0x7c
#define R_TX_ABRT_SOURCE 0x80
#define R_ENABLE_STATUS  0x9c
#define R_CLR_RESTART_DET 0xa8
#define R_COMP_PARAM_1   0xf4
#define R_COMP_VERSION   0xf8
#define R_COMP_TYPE      0xfc

#define COMP_TYPE_VALUE   0x44570140
#define COMP_VERSION_VALUE 0x3131312a /* "111*" -- matches a real IP rev */

/* DATA_CMD command bits */
#define DATA_CMD_READ    (1U << 8)
#define DATA_CMD_STOP    (1U << 9)
#define DATA_CMD_RESTART (1U << 10)

/* INTR bits */
#define INTR_RX_UNDER  0x001
#define INTR_RX_OVER   0x002
#define INTR_RX_FULL   0x004
#define INTR_TX_OVER   0x008
#define INTR_TX_EMPTY  0x010
#define INTR_RD_REQ    0x020
#define INTR_TX_ABRT   0x040
#define INTR_RX_DONE   0x080
#define INTR_ACTIVITY  0x100
#define INTR_STOP_DET  0x200
#define INTR_START_DET 0x400
#define INTR_GEN_CALL  0x800

#define STATUS_ACTIVITY 0x1
#define STATUS_TFE      (1U << 2)
#define STATUS_MST_ACT  (1U << 5)

#define ENABLE_EN (1U << 0)

#define ABRT_7B_ADDR_NOACK (1U << 0)

#define RX_FIFO_DEPTH 32

struct DwI2cState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus      *bus;
    qemu_irq     irq;

    uint32_t con;
    uint32_t tar;
    uint32_t sar;
    uint32_t intr_mask;
    uint32_t raw_intr_stat;
    uint32_t rx_tl;
    uint32_t tx_tl;
    uint32_t sda_hold;
    uint32_t tx_abrt_source;
    uint32_t enable;

    bool     in_transfer;
    bool     cur_is_read;

    uint8_t  rx_fifo[RX_FIFO_DEPTH];
    uint32_t rx_head, rx_count;
};

static uint32_t dw_i2c_rx_pending(DwI2cState *s)
{
    return s->rx_count;
}

/*
 * TX_EMPTY is a level flag on real hardware, continuously reflecting
 * "transmit FIFO occupancy <= TX_TL" -- not a latched/sticky event
 * (confirmed from the real driver's own i2c_dw_xfer_msg(), which only
 * ever silences it by clearing DW_IC_INTR_MASK's bit, never via
 * DW_IC_CLR_INTR). Since every DATA_CMD write here completes
 * synchronously, the transmit FIFO is *always* logically empty, so
 * this bit is always live. This matters at adapter-enable time: real
 * hardware asserts TX_EMPTY the instant IC_ENABLE is set (FIFO starts
 * empty), and i2c_dw_xfer_init() enables the adapter and unmasks
 * TX_EMPTY *before* ever touching DATA_CMD itself -- the real
 * driver's i2c_dw_xfer_msg() (which performs the actual DATA_CMD
 * writes) only ever runs from inside the ISR, kicked off by that
 * very first TX_EMPTY interrupt. Treating TX_EMPTY as a stored,
 * write-triggered bit (set only once a DATA_CMD write had already
 * happened) is a chicken-and-egg deadlock: that first DATA_CMD write
 * never happens because the kickoff interrupt that's supposed to
 * cause it never fires. Computing it live here, independent of any
 * write having happened yet, is what breaks that deadlock.
 */
static uint32_t dw_i2c_effective_raw(DwI2cState *s)
{
    uint32_t v = s->raw_intr_stat | INTR_TX_EMPTY;

    if (dw_i2c_rx_pending(s) > s->rx_tl) {
        v |= INTR_RX_FULL;
    }
    return v;
}

static void dw_i2c_update_irq(DwI2cState *s)
{
    qemu_set_irq(s->irq, (dw_i2c_effective_raw(s) & s->intr_mask) != 0);
}

static void dw_i2c_rx_push(DwI2cState *s, uint8_t byte)
{
    if (s->rx_count < RX_FIFO_DEPTH) {
        s->rx_fifo[(s->rx_head + s->rx_count) % RX_FIFO_DEPTH] = byte;
        s->rx_count++;
    }
    /* Real hardware would raise RX_OVER past depth; harmless to drop
     * silently here since no real driver on this bus issues reads
     * deep enough to overflow a 32-entry FIFO between two register
     * pokes. */
}

static uint8_t dw_i2c_rx_pop(DwI2cState *s)
{
    uint8_t byte = 0;

    if (s->rx_count > 0) {
        byte = s->rx_fifo[s->rx_head];
        s->rx_head = (s->rx_head + 1) % RX_FIFO_DEPTH;
        s->rx_count--;
    }
    return byte;
}

static void dw_i2c_do_data_cmd(DwI2cState *s, uint32_t value)
{
    bool is_read = (value & DATA_CMD_READ) != 0;
    bool want_restart = (value & DATA_CMD_RESTART) != 0;
    bool want_stop = (value & DATA_CMD_STOP) != 0;
    uint8_t data = value & 0xFF;
    uint8_t addr7 = s->tar & 0x7F;
    int rc;

    if (!(s->enable & ENABLE_EN)) {
        return;
    }

    if (!s->in_transfer || want_restart || is_read != s->cur_is_read) {
        if (s->in_transfer) {
            i2c_end_transfer(s->bus);
        }
        rc = is_read ? i2c_start_recv(s->bus, addr7)
                     : i2c_start_send(s->bus, addr7);
        if (rc != 0) {
            s->tx_abrt_source |= ABRT_7B_ADDR_NOACK;
            s->raw_intr_stat |= INTR_TX_ABRT;
            s->in_transfer = false;
            dw_i2c_update_irq(s);
            return;
        }
        s->in_transfer = true;
        s->cur_is_read = is_read;
        s->raw_intr_stat |= INTR_START_DET;
    }

    if (is_read) {
        dw_i2c_rx_push(s, i2c_recv(s->bus));
    } else {
        i2c_send(s->bus, data);
    }

    if (want_stop) {
        i2c_end_transfer(s->bus);
        s->in_transfer = false;
        s->raw_intr_stat |= INTR_STOP_DET | INTR_ACTIVITY;
    }

    dw_i2c_update_irq(s);
}

static uint64_t dw_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    DwI2cState *s = DW_I2C(opaque);

    switch (offset) {
    case R_CON:           return s->con;
    case R_TAR:           return s->tar;
    case R_SAR:           return s->sar;

    case R_DATA_CMD: {
        /* RX_FULL is computed live from FIFO occupancy (see
         * dw_i2c_effective_raw()), so popping a byte here can make it
         * go away -- re-evaluate the IRQ line to match, exactly like
         * every other FIFO/status-mutating path already does. Without
         * this, the real driver's own kernel ISR (triggered while
         * RX_FULL was asserted) drains the FIFO via this exact read,
         * but the GIC input line was last explicitly driven high by
         * dw_i2c_update_irq() and nothing ever tells it to go low
         * again: the level stays stuck asserted forever even though
         * every *register* read would now correctly report nothing
         * pending, so the kernel's level-triggered IRQ keeps
         * re-firing indefinitely (confirmed live: millions of
         * ENABLE/RAW_INTR_STAT/INTR_STAT re-reads per second, 100%
         * host CPU, zero forward progress, with the stuck level
         * traced back to the single update_irq call before this
         * exact pop and none after it). */
        uint64_t val = dw_i2c_rx_pop(s);
        dw_i2c_update_irq(s);
        return val;
    }

    case R_SS_SCL_HCNT:
    case R_SS_SCL_LCNT:
    case R_FS_SCL_HCNT:
    case R_FS_SCL_LCNT:
    case R_HS_SCL_HCNT:
    case R_HS_SCL_LCNT:
        return 0;

    case R_INTR_STAT:
        return dw_i2c_effective_raw(s) & s->intr_mask;
    case R_INTR_MASK:
        return s->intr_mask;
    case R_RAW_INTR_STAT:
        return dw_i2c_effective_raw(s);

    case R_RX_TL: return s->rx_tl;
    case R_TX_TL: return s->tx_tl;

    case R_CLR_INTR:
        s->raw_intr_stat = 0;
        s->tx_abrt_source = 0;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_RX_UNDER:
        s->raw_intr_stat &= ~INTR_RX_UNDER;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_RX_OVER:
        s->raw_intr_stat &= ~INTR_RX_OVER;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_TX_OVER:
        s->raw_intr_stat &= ~INTR_TX_OVER;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_RD_REQ:
        s->raw_intr_stat &= ~INTR_RD_REQ;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_TX_ABRT:
        /* Real hardware clears TX_ABRT_SOURCE on this read -- the
         * driver reads TX_ABRT_SOURCE first, then this, per its own
         * comment ("preserve it beforehand"). */
        s->raw_intr_stat &= ~INTR_TX_ABRT;
        s->tx_abrt_source = 0;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_RX_DONE:
        s->raw_intr_stat &= ~INTR_RX_DONE;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_ACTIVITY:
        s->raw_intr_stat &= ~INTR_ACTIVITY;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_STOP_DET:
        s->raw_intr_stat &= ~INTR_STOP_DET;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_START_DET:
        s->raw_intr_stat &= ~INTR_START_DET;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_GEN_CALL:
        s->raw_intr_stat &= ~INTR_GEN_CALL;
        dw_i2c_update_irq(s);
        return 0;
    case R_CLR_RESTART_DET:
        return 0;

    case R_ENABLE:        return s->enable;

    case R_STATUS: {
        uint32_t v = STATUS_TFE;
        if (s->in_transfer) {
            v |= STATUS_ACTIVITY | STATUS_MST_ACT;
        }
        return v;
    }

    case R_TXFLR: return 0;
    case R_RXFLR: return dw_i2c_rx_pending(s);

    case R_SDA_HOLD:      return s->sda_hold;
    case R_TX_ABRT_SOURCE: return s->tx_abrt_source;
    case R_ENABLE_STATUS: return s->enable & ENABLE_EN;

    case R_COMP_PARAM_1:
        return ((RX_FIFO_DEPTH - 1) << 16) | ((RX_FIFO_DEPTH - 1) << 8);
    case R_COMP_VERSION: return COMP_VERSION_VALUE;
    case R_COMP_TYPE:    return COMP_TYPE_VALUE;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "dw-i2c: bad read offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void dw_i2c_write(void *opaque, hwaddr offset,
                         uint64_t value, unsigned size)
{
    DwI2cState *s = DW_I2C(opaque);

    switch (offset) {
    case R_CON:  s->con = value; break;
    case R_TAR:  s->tar = value & 0x1FFF; break;
    case R_SAR:  s->sar = value & 0x3FF; break;

    case R_DATA_CMD:
        dw_i2c_do_data_cmd(s, value);
        break;

    case R_SS_SCL_HCNT:
    case R_SS_SCL_LCNT:
    case R_FS_SCL_HCNT:
    case R_FS_SCL_LCNT:
    case R_HS_SCL_HCNT:
    case R_HS_SCL_LCNT:
        /* Timing dividers -- no real bus delay modelled here. */
        break;

    case R_INTR_MASK:
        s->intr_mask = value;
        dw_i2c_update_irq(s);
        break;

    case R_RX_TL: s->rx_tl = value & 0xFF; break;
    case R_TX_TL: s->tx_tl = value & 0xFF; break;

    case R_ENABLE:
        s->enable = value & ENABLE_EN;
        if (!(s->enable & ENABLE_EN) && s->in_transfer) {
            i2c_end_transfer(s->bus);
            s->in_transfer = false;
        }
        break;

    case R_SDA_HOLD: s->sda_hold = value; break;

    case R_INTR_STAT:
    case R_RAW_INTR_STAT:
    case R_CLR_INTR:
    case R_CLR_RX_UNDER:
    case R_CLR_RX_OVER:
    case R_CLR_TX_OVER:
    case R_CLR_RD_REQ:
    case R_CLR_TX_ABRT:
    case R_CLR_RX_DONE:
    case R_CLR_ACTIVITY:
    case R_CLR_STOP_DET:
    case R_CLR_START_DET:
    case R_CLR_GEN_CALL:
    case R_CLR_RESTART_DET:
    case R_STATUS:
    case R_TXFLR:
    case R_RXFLR:
    case R_TX_ABRT_SOURCE:
    case R_ENABLE_STATUS:
    case R_COMP_PARAM_1:
    case R_COMP_VERSION:
    case R_COMP_TYPE:
        /* Read-only. */
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "dw-i2c: bad write offset 0x%x val=0x%x\n",
                      (int)offset, (unsigned)value);
        break;
    }
}

static const MemoryRegionOps dw_i2c_ops = {
    .read  = dw_i2c_read,
    .write = dw_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void dw_i2c_reset(DeviceState *dev)
{
    DwI2cState *s = DW_I2C(dev);

    s->con = 0;
    s->tar = 0;
    s->sar = 0;
    s->intr_mask = 0;
    s->raw_intr_stat = 0;
    s->rx_tl = 0;
    s->tx_tl = 0;
    s->sda_hold = 0;
    s->tx_abrt_source = 0;
    s->enable = 0;
    s->in_transfer = false;
    s->cur_is_read = false;
    s->rx_head = 0;
    s->rx_count = 0;
}

static void dw_i2c_init(Object *obj)
{
    DwI2cState *s = DW_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dw_i2c_ops, s,
                          TYPE_DW_I2C, DW_I2C_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static void dw_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, dw_i2c_reset);
}

static const TypeInfo dw_i2c_info = {
    .name          = TYPE_DW_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DwI2cState),
    .instance_init = dw_i2c_init,
    .class_init    = dw_i2c_class_init,
};

static void dw_i2c_register_types(void)
{
    type_register_static(&dw_i2c_info);
}

type_init(dw_i2c_register_types)
