/*
 * HiSilicon HiBVT I2C controller emulation.
 *
 * Drives a QEMU I2C bus from the vendor "hibvt" register-level
 * interface used on V2+ SoCs (CV200 family, AV100, CV300, EV300,
 * CV500, CV610, ...).  The Linux driver (drivers/i2c/busses/i2c-hibvt.c)
 * programs a command queue at CMD_BASE (offsets 0x30..0xAC, 32 slots of
 * 4 bytes each) and then sets CTRL1.START; the hardware walks the
 * queue, generating START/STOP/data bytes accordingly.
 *
 * Command opcodes (see vendor driver):
 *   0x00 EXIT       end of program
 *   0x01 TX_S       emit START
 *   0x04 TX_D1_2    transmit high byte of DATA1 (10-bit addr only)
 *   0x05 TX_D1_1    transmit low byte of DATA1 (slave addr + R/W)
 *   0x09 TX_FIFO    transmit one byte from TX FIFO
 *   0x12 RX_FIFO    receive one byte into RX FIFO
 *   0x13 RX_ACK     expect ACK from slave (abort on NACK)
 *   0x15 IGN_ACK    don't abort on NACK
 *   0x16 TX_ACK     transmit ACK (master-to-slave on reads, non-last)
 *   0x17 TX_NACK    transmit NACK (last byte of a read)
 *   0x18 JMP1       loop-back: if LOOP1 > 0, decrement & jump to DST1
 *   0x19 JMP2       the same with LOOP2 / DST2: the V5 (Linux 5.10
 *                   i2c-bsp) driver repeats a whole multi-register write
 *                   program with it
 *   0x1D UP_TXF     update TX FIFO pointer (DMA-only; no-op here)
 *   0x1E TX_RS      emit REPEATED START
 *   0x1F TX_P       emit STOP
 *
 * In QEMU's I2C model, START+address are merged into a single
 * i2c_start_send()/i2c_start_recv() call, so TX_S / TX_RS just set a
 * pending-start flag and the subsequent TX_D1_1 actually opens the
 * bus.  ACK/NACK opcodes are status flow-control on real silicon; we
 * treat them as no-ops because the bus abstraction handles them.
 *
 * Execution model: when the guest sets CTRL1.START, the program runs
 * synchronously until it either terminates (EXIT / end of slots) or
 * stalls at a TX_FIFO opcode with an empty TX FIFO.  A subsequent
 * write to TXF resumes execution.  RX_FIFO opcodes push bytes the
 * guest later drains via RXF reads.
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

#define TYPE_HISI_I2C "hisi-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(HisiI2cState, HISI_I2C)

#define HISI_I2C_REG_SIZE   0x1000

/* Register offsets */
#define R_GLB           0x000
#define R_SCL_H         0x004
#define R_SCL_L         0x008
#define R_DATA1         0x010
#define R_TXF           0x020
#define R_RXF           0x024
#define R_CMD_BASE      0x030     /* 32 command slots, 4 bytes each */
#define R_CMD_END       0x0B0
#define R_LOOP1         0x0B0
#define R_DST1          0x0B4
#define R_LOOP2         0x0B8
#define R_DST2          0x0BC
#define R_TX_WATER      0x0C8
#define R_RX_WATER      0x0CC
#define R_CTRL1         0x0D0
#define R_CTRL2         0x0D4
#define R_STAT          0x0D8
#define R_INTR_RAW      0x0E0
#define R_INTR_EN       0x0E4
#define R_INTR_STAT     0x0E8

/* GLB register */
#define GLB_EN          (1U << 0)

/* STAT register */
#define STAT_RXF_NOE    (1U << 16)
#define STAT_TXF_NOF    (1U << 19)

/* CTRL1 register */
#define CTRL1_CMD_START (1U << 0)

/* CTRL2 register */
#define CTRL2_SDA_IN    (1U << 16)   /* read-back SDA line level */

/* INTR bits (shared across RAW/EN/STAT) */
#define INTR_ABORT_MASK ((1U << 0) | (1U << 11))
#define INTR_RX_MASK    (1U << 2)
#define INTR_TX_MASK    (1U << 4)
#define INTR_CMD_DONE   (1U << 12)

/* Command opcodes */
#define CMD_EXIT        0x00
#define CMD_TX_S        0x01
#define CMD_TX_D1_2     0x04
#define CMD_TX_D1_1     0x05
#define CMD_TX_FIFO     0x09
#define CMD_RX_FIFO     0x12
#define CMD_RX_ACK      0x13
#define CMD_IGN_ACK     0x15
#define CMD_TX_ACK      0x16
#define CMD_TX_NACK     0x17
#define CMD_JMP1        0x18
#define CMD_JMP2        0x19
#define CMD_UP_TXF      0x1D
#define CMD_TX_RS       0x1E
#define CMD_TX_P        0x1F

#define CMD_SLOTS       32

#define FIFO_DEPTH      64

struct HisiI2cState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus      *bus;
    qemu_irq     irq;

    uint32_t     glb;
    uint32_t     scl_h;
    uint32_t     scl_l;
    uint32_t     data1;
    uint32_t     loop1;
    uint32_t     dst1;
    uint32_t     loop2;
    uint32_t     dst2;
    uint32_t     tx_water;
    uint32_t     rx_water;
    uint32_t     ctrl1;
    uint32_t     ctrl2;
    uint32_t     intr_raw;
    uint32_t     intr_en;

    uint32_t     cmds[CMD_SLOTS];

    /* FIFOs (kept as simple byte buffers, size bounded by FIFO_DEPTH) */
    uint8_t      txf[FIFO_DEPTH];
    uint32_t     txf_len;
    uint8_t      rxf[FIFO_DEPTH];
    uint32_t     rxf_head;   /* next byte to dequeue on RXF read */
    uint32_t     rxf_len;    /* bytes queued */

    /* Command-queue execution state */
    bool         running;
    bool         active;     /* an i2c_start_* has been issued and not ended */
    bool         is_read;
    bool         pending_start;
    bool         pending_restart;
    uint32_t     pc;
};

static void hisi_i2c_update_irq(HisiI2cState *s)
{
    qemu_set_irq(s->irq, (s->intr_raw & s->intr_en) != 0);
}

static void hisi_i2c_run(HisiI2cState *s);

static uint8_t hisi_i2c_txf_pop(HisiI2cState *s)
{
    uint8_t v = s->txf[0];
    memmove(s->txf, s->txf + 1, s->txf_len - 1);
    s->txf_len--;
    return v;
}

static void hisi_i2c_rxf_push(HisiI2cState *s, uint8_t v)
{
    if (s->rxf_len < FIFO_DEPTH) {
        s->rxf[(s->rxf_head + s->rxf_len) % FIFO_DEPTH] = v;
        s->rxf_len++;
    }
}

static void hisi_i2c_do_start(HisiI2cState *s, bool restart)
{
    uint8_t addr = (s->data1 >> 1) & 0x7F;
    bool want_read = (s->data1 & 1) != 0;
    int ret;

    if (s->active && restart) {
        i2c_end_transfer(s->bus);
        s->active = false;
    }

    ret = want_read ? i2c_start_recv(s->bus, addr)
                    : i2c_start_send(s->bus, addr);
    if (ret == 0) {
        s->active = true;
        s->is_read = want_read;
    } else {
        /*
         * No slave acked — flag abort for guest.  Drain any TX FIFO
         * bytes the guest queued for this aborted transaction;
         * otherwise they pile up across consecutive failed probes
         * (Sofia walks ~10 sensor addresses at startup, most NACK)
         * and eventually corrupt the TX stream of subsequent
         * successful transactions when the FIFO overflows.  Real
         * hardware drivers typically reset the FIFO after an abort
         * via a soft-reset register; clearing it on abort here gives
         * equivalent behaviour without modelling that register.
         */
        s->intr_raw |= INTR_ABORT_MASK;
        s->active = false;
        s->txf_len = 0;
    }
}

static void hisi_i2c_finish_program(HisiI2cState *s)
{
    s->running = false;
    s->intr_raw |= INTR_CMD_DONE;
    s->ctrl1 &= ~CTRL1_CMD_START;
    hisi_i2c_update_irq(s);
}

static void hisi_i2c_run(HisiI2cState *s)
{
    if (!s->running) {
        return;
    }

    while (s->pc < CMD_SLOTS) {
        uint32_t cmd = s->cmds[s->pc] & 0x1F;
        bool advance = true;

        switch (cmd) {
        case CMD_EXIT:
            hisi_i2c_finish_program(s);
            return;

        case CMD_TX_S:
            s->pending_start = true;
            s->pending_restart = false;
            break;

        case CMD_TX_RS:
            s->pending_restart = true;
            s->pending_start = false;
            break;

        case CMD_TX_D1_2:
            /* 10-bit addressing — high byte of DATA1 gets emitted after
             * the start sequence on real silicon.  We only support 7-bit
             * addressing in QEMU's I2C model; skip. */
            break;

        case CMD_TX_D1_1:
            if (s->pending_start || s->pending_restart) {
                hisi_i2c_do_start(s, s->pending_restart);
                s->pending_start = false;
                s->pending_restart = false;
                if (s->intr_raw & INTR_ABORT_MASK) {
                    hisi_i2c_finish_program(s);
                    return;
                }
            } else if (s->active && !s->is_read) {
                i2c_send(s->bus, s->data1 & 0xFF);
            }
            break;

        case CMD_TX_FIFO:
            if (s->txf_len == 0) {
                /* Stall until guest refills TXF. */
                return;
            }
            if (s->active && !s->is_read) {
                i2c_send(s->bus, hisi_i2c_txf_pop(s));
            } else {
                /* Drain regardless to keep FIFO accounting consistent. */
                (void)hisi_i2c_txf_pop(s);
            }
            if (s->txf_len < s->tx_water) {
                s->intr_raw |= INTR_TX_MASK;
            }
            break;

        case CMD_RX_FIFO:
            if (s->active && s->is_read) {
                hisi_i2c_rxf_push(s, i2c_recv(s->bus));
                if (s->rxf_len >= s->rx_water) {
                    s->intr_raw |= INTR_RX_MASK;
                }
            }
            break;

        case CMD_RX_ACK:
        case CMD_IGN_ACK:
        case CMD_TX_ACK:
        case CMD_TX_NACK:
            /* Flow-control markers — QEMU's I2C bus abstraction handles
             * ACK/NACK implicitly through i2c_start_*() return codes. */
            break;

        case CMD_JMP1:
            if (s->loop1 > 0) {
                s->loop1--;
                s->pc = s->dst1;
                advance = false;
            }
            break;

        case CMD_JMP2:
            if (s->loop2 > 0) {
                s->loop2--;
                s->pc = s->dst2;
                advance = false;
            }
            break;

        case CMD_UP_TXF:
            /* DMA-mode TX FIFO update — no-op for PIO. */
            break;

        case CMD_TX_P:
            if (s->active) {
                i2c_end_transfer(s->bus);
                s->active = false;
            }
            break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-i2c: unknown command 0x%x at pc=%u\n",
                          cmd, s->pc);
            s->intr_raw |= INTR_ABORT_MASK;
            hisi_i2c_finish_program(s);
            return;
        }

        if (advance) {
            s->pc++;
        }
    }

    /* Fell off the end of the command array — treat as EXIT. */
    hisi_i2c_finish_program(s);
}

static uint64_t hisi_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiI2cState *s = HISI_I2C(opaque);

    if (offset >= R_CMD_BASE && offset < R_CMD_END) {
        return s->cmds[(offset - R_CMD_BASE) >> 2];
    }

    switch (offset) {
    case R_GLB:
        return s->glb;
    case R_SCL_H:
        return s->scl_h;
    case R_SCL_L:
        return s->scl_l;
    case R_DATA1:
        return s->data1;
    case R_TXF:
        return 0;
    case R_RXF: {
        uint8_t v;
        if (s->rxf_len == 0) {
            return 0;
        }
        v = s->rxf[s->rxf_head];
        s->rxf_head = (s->rxf_head + 1) % FIFO_DEPTH;
        s->rxf_len--;
        /* Reading may have drained the FIFO and allowed the program to
         * progress, but RX_FIFO opcodes are self-contained so no resume
         * needed here. */
        return v;
    }
    case R_LOOP1:
        return s->loop1;
    case R_DST1:
        return s->dst1;
    case R_LOOP2:
        return s->loop2;
    case R_DST2:
        return s->dst2;
    case R_TX_WATER:
        return s->tx_water;
    case R_RX_WATER:
        return s->rx_water;
    case R_CTRL1:
        return s->ctrl1;
    case R_CTRL2:
        /* Mirror SDA high when idle so the rescue path doesn't spin. */
        return s->ctrl2 | CTRL2_SDA_IN;
    case R_STAT: {
        uint32_t v = 0;
        if (s->rxf_len > 0) {
            v |= STAT_RXF_NOE;
        }
        if (s->txf_len < FIFO_DEPTH) {
            v |= STAT_TXF_NOF;
        }
        return v;
    }
    case R_INTR_RAW:
        return s->intr_raw;
    case R_INTR_EN:
        return s->intr_en;
    case R_INTR_STAT:
        return s->intr_raw & s->intr_en;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-i2c: bad read offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void hisi_i2c_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    HisiI2cState *s = HISI_I2C(opaque);

    if (offset >= R_CMD_BASE && offset < R_CMD_END) {
        s->cmds[(offset - R_CMD_BASE) >> 2] = value & 0x1F;
        return;
    }

    switch (offset) {
    case R_GLB:
        s->glb = value;
        break;
    case R_SCL_H:
        s->scl_h = value;
        break;
    case R_SCL_L:
        s->scl_l = value;
        break;
    case R_DATA1:
        /*
         * The kernel hibvt-i2c driver writes DATA1 once per transaction
         * (before queueing TXF bytes and setting CTRL1.START).  Clear
         * any TX FIFO leftovers from a previous program so this fresh
         * transaction sees only its own bytes.  Without this, Sofia's
         * sensor probe sequence (which intentionally queues 1 extra
         * byte per write program — likely an SDK quirk that real HW
         * tolerates because the FIFO is reset between transactions)
         * gradually corrupts subsequent transactions.
         */
        s->data1 = value & 0x3FF;
        s->txf_len = 0;
        break;
    case R_TXF:
        if (s->txf_len < FIFO_DEPTH) {
            s->txf[s->txf_len++] = value & 0xFF;
        }
        /* A write to TXF may satisfy a stalled TX_FIFO opcode. */
        hisi_i2c_run(s);
        break;
    case R_LOOP1:
        s->loop1 = value;
        break;
    case R_DST1:
        s->dst1 = value & (CMD_SLOTS - 1);
        break;
    case R_LOOP2:
        s->loop2 = value;
        break;
    case R_DST2:
        s->dst2 = value & (CMD_SLOTS - 1);
        break;
    case R_TX_WATER:
        s->tx_water = value & 0x7F;
        break;
    case R_RX_WATER:
        s->rx_water = value & 0x7F;
        break;
    case R_CTRL1:
        s->ctrl1 = value & ~CTRL1_CMD_START;
        if (value & CTRL1_CMD_START) {
            /*
             * Kick off a fresh program run.  The active/is_read state
             * carries over from the previous program: the kernel
             * emits one program per I2C message and uses CMD_TX_RS in
             * subsequent messages to restart the bus without an
             * intervening STOP, so we must not drop the open transfer
             * here.
             */
            s->running = true;
            s->pending_start = false;
            s->pending_restart = false;
            s->pc = 0;
            hisi_i2c_run(s);
        }
        break;
    case R_CTRL2:
        s->ctrl2 = value;
        break;
    case R_INTR_RAW:
        /* write-1-to-clear */
        s->intr_raw &= ~value;
        hisi_i2c_update_irq(s);
        break;
    case R_INTR_EN:
        s->intr_en = value;
        hisi_i2c_update_irq(s);
        break;
    case R_RXF:
    case R_STAT:
    case R_INTR_STAT:
        /* Read-only. */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-i2c: bad write offset 0x%x val=0x%x\n",
                      (int)offset, (unsigned)value);
        break;
    }
}

static const MemoryRegionOps hisi_i2c_ops = {
    .read  = hisi_i2c_read,
    .write = hisi_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_i2c_reset(DeviceState *dev)
{
    HisiI2cState *s = HISI_I2C(dev);

    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    memset(s->cmds, 0, sizeof(s->cmds));
    s->glb = 0;
    s->scl_h = 0;
    s->scl_l = 0;
    s->data1 = 0;
    s->loop1 = 0;
    s->dst1 = 0;
    s->loop2 = 0;
    s->dst2 = 0;
    s->tx_water = 32;
    s->rx_water = 1;
    s->ctrl1 = 0;
    s->ctrl2 = 0;
    s->intr_raw = 0;
    s->intr_en = 0;
    s->txf_len = 0;
    s->rxf_head = 0;
    s->rxf_len = 0;
    s->running = false;
    s->active = false;
    s->is_read = false;
    s->pending_start = false;
    s->pending_restart = false;
    s->pc = 0;
    hisi_i2c_update_irq(s);
}

static void hisi_i2c_init(Object *obj)
{
    HisiI2cState *s = HISI_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_i2c_ops, s,
                          TYPE_HISI_I2C, HISI_I2C_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static void hisi_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_i2c_reset);
}

static const TypeInfo hisi_i2c_info = {
    .name          = TYPE_HISI_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiI2cState),
    .instance_init = hisi_i2c_init,
    .class_init    = hisi_i2c_class_init,
};

static void hisi_i2c_register_types(void)
{
    type_register_static(&hisi_i2c_info);
}

type_init(hisi_i2c_register_types)
