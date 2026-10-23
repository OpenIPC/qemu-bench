/*
 * SigmaStar / MStar MIIC I2C master (0x1F223000, 0x1F223200, 0x1F222E00).
 *
 * 16-bit registers on the RIU's 4-byte stride (the vendor
 * drivers/sstar/i2c/<chip>/hal_iic_reg.h; Maruko SDK, infinity6c):
 *
 *   0x00 config: bit 0 reset, 1 DMA, 2 interrupt enable
 *   0x01 bit 0 START, bit 8 STOP (each byte written on its own)
 *   0x02 write data [7:0]; bit 8 reads back the slave's NACK
 *   0x03 read data [7:0]; writing bit 8 reads a byte, bit 9 NACKs it
 *   0x04 bit 0 interrupt flag: set when a step completes, write 1 clears
 *   0x05 status: [4:0] state (0 idle), bit 8 start, 9 stop, 10 RX done,
 *        11 TX done
 *   0x08-0x12 SCL/SDA timing counts: stored
 *   0x20 DMA config: bit 1 soft reset, 2 interrupt enable, 3 MIU reset
 *   0x21-0x23 [3:0] the buffer's MIU address; 0x23 bit 5 no STOP at the
 *        end, bit 6 read (else write)
 *   0x24 bit 0 DMA done, write 1 clears
 *   0x25-0x28 command bytes sent first, 0x29 [3:0] their count
 *   0x2A/0x2B data length, 0x2C/0x2D bytes moved
 *   0x2E [9:0] slave address (7-bit unless bit 10)
 *   0x2F bit 0 DMA trigger
 *
 * A DMA job sends START, the address, the command bytes and then writes
 * the buffer, or (repeated START) reads into it; STOP unless 0x23 bit 5.
 * It completes at once.  The interrupt line is the DMA done flag with
 * DMA interrupts on, or the byte-mode flag with interrupts on.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_SSTAR_I2C "sstar-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(SstarI2cState, SSTAR_I2C)

#define R_CONFIG            0x00
#define R_START_STOP        0x01
#define R_WDATA             0x02
#define R_RDATA             0x03
#define R_INT_FLAG          0x04
#define R_STATUS            0x05
#define R_DMA_CONFIG        0x20
#define R_MIU_LO            0x21
#define R_MIU_HI            0x22
#define R_DMA_CTRL          0x23
#define R_DMA_DONE          0x24
#define R_CMD_DATA          0x25        /* 4 registers, 8 bytes */
#define R_CMD_LEN           0x29
#define R_LEN_LO            0x2A
#define R_LEN_HI            0x2B
#define R_TC_LO             0x2C
#define R_TC_HI             0x2D
#define R_SLAVE             0x2E
#define R_DMA_TRIGGER       0x2F

#define CONFIG_INT_EN       (1u << 2)
#define WDATA_NACK          (1u << 8)
#define RDATA_READ          (1u << 8)
#define RDATA_NACK          (1u << 9)
#define ST_START            (1u << 8)
#define ST_STOP             (1u << 9)
#define ST_RX_DONE          (1u << 10)
#define ST_TX_DONE          (1u << 11)
#define DMACFG_INT_EN       (1u << 2)
#define DMACTRL_NO_STOP     (1u << 5)
#define DMACTRL_READ        (1u << 6)

#define SS_DRAM_BASE        0x20000000
#define DMA_MAX_LEN         0x10000

struct SstarI2cState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;
    uint16_t regs[0x80];
    bool want_addr;             /* START sent; the next byte is the address */
    bool in_xfer;               /* an address was acked */
    bool recv;
};

static void sstar_i2c_update_irq(SstarI2cState *s)
{
    bool dma = (s->regs[R_DMA_DONE] & 1) &&
               (s->regs[R_DMA_CONFIG] & DMACFG_INT_EN);
    bool riu = (s->regs[R_INT_FLAG] & 1) &&
               (s->regs[R_CONFIG] & CONFIG_INT_EN);

    qemu_set_irq(s->irq, dma || riu);
}

/* A byte-mode step done: flag it with its status. */
static void sstar_i2c_step(SstarI2cState *s, uint16_t status)
{
    s->regs[R_STATUS] = status;
    s->regs[R_INT_FLAG] |= 1;
    sstar_i2c_update_irq(s);
}

static void sstar_i2c_stop(SstarI2cState *s)
{
    if (s->in_xfer) {
        i2c_end_transfer(s->bus);
    }
    s->in_xfer = false;
    s->want_addr = false;
}

/* true if acked */
static bool sstar_i2c_start(SstarI2cState *s, uint8_t addr, bool recv)
{
    if (s->in_xfer) {
        /* a repeated START */
        i2c_end_transfer(s->bus);
        s->in_xfer = false;
    }
    if (i2c_start_transfer(s->bus, addr, recv)) {
        return false;
    }
    s->in_xfer = true;
    s->recv = recv;
    return true;
}

/* ---- byte mode ------------------------------------------------------ */

static void sstar_i2c_write_byte(SstarI2cState *s, uint8_t b)
{
    bool ack;

    if (s->want_addr) {
        s->want_addr = false;
        ack = sstar_i2c_start(s, b >> 1, b & 1);
    } else if (s->in_xfer && !s->recv) {
        ack = !i2c_send(s->bus, b);
    } else {
        ack = false;
    }
    s->regs[R_WDATA] = b | (ack ? 0 : WDATA_NACK);
    sstar_i2c_step(s, ST_TX_DONE);
}

static void sstar_i2c_read_byte(SstarI2cState *s, bool nack)
{
    uint8_t b = 0xFF;

    if (s->in_xfer && s->recv) {
        b = i2c_recv(s->bus);
        if (nack) {
            i2c_nack(s->bus);
        }
    }
    s->regs[R_RDATA] = (s->regs[R_RDATA] & 0xFF00) | b;
    sstar_i2c_step(s, ST_RX_DONE);
}

/* ---- DMA ------------------------------------------------------------ */

static void sstar_i2c_dma(SstarI2cState *s)
{
    uint32_t len = s->regs[R_LEN_LO] | (s->regs[R_LEN_HI] << 16);
    /* 0x23 [3:0], MIU address bits 35:32, are beyond this 32-bit map */
    uint32_t addr = s->regs[R_MIU_LO] | (s->regs[R_MIU_HI] << 16);
    uint8_t slave = s->regs[R_SLAVE] & 0x7F;
    int cmd_len = s->regs[R_CMD_LEN] & 0xF;
    bool read = s->regs[R_DMA_CTRL] & DMACTRL_READ;
    bool ten = s->regs[R_SLAVE] & (1u << 10);
    g_autofree uint8_t *buf = NULL;
    uint32_t done = 0;
    int i;

    if (ten) {
        /* not modelled: no slave answers, as a NACK of the address */
        qemu_log_mask(LOG_UNIMP, "sstar-i2c: 10-bit addressing\n");
    }
    if (addr < SS_DRAM_BASE) {
        addr += SS_DRAM_BASE;
    }
    len = MIN(len, DMA_MAX_LEN);
    buf = g_malloc0(len ? len : 1);

    /* address and command bytes, written; then the data either way */
    if (!ten && sstar_i2c_start(s, slave, read && !cmd_len)) {
        bool ok = true;

        if (cmd_len) {
            for (i = 0; i < MIN(cmd_len, 8) && ok; i++) {
                uint16_t w = s->regs[R_CMD_DATA + i / 2];

                ok = !i2c_send(s->bus, i & 1 ? w >> 8 : w & 0xFF);
            }
            if (ok && read) {
                ok = sstar_i2c_start(s, slave, true);
            }
        }
        if (ok && read) {
            for (done = 0; done < len; done++) {
                buf[done] = i2c_recv(s->bus);
            }
            if (len) {
                i2c_nack(s->bus);
            }
            dma_memory_write(&address_space_memory, addr, buf, len,
                             MEMTXATTRS_UNSPECIFIED);
        } else if (ok) {
            dma_memory_read(&address_space_memory, addr, buf, len,
                            MEMTXATTRS_UNSPECIFIED);
            for (done = 0; done < len; done++) {
                if (i2c_send(s->bus, buf[done])) {
                    break;
                }
            }
        }
    }
    if (!(s->regs[R_DMA_CTRL] & DMACTRL_NO_STOP)) {
        sstar_i2c_stop(s);
    }
    s->regs[R_TC_LO] = done & 0xFFFF;
    s->regs[R_TC_HI] = done >> 16;
    s->regs[R_DMA_DONE] |= 1;
    sstar_i2c_update_irq(s);
}

/* ---- MMIO ----------------------------------------------------------- */

static uint64_t sstar_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI2cState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->regs[(off >> 2) & 0x7F];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_i2c_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    SstarI2cState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    uint16_t mask, v;

    if ((off & 3) >= 2) {
        return;
    }
    /* which byte lanes this access wrote */
    if (size == 1) {
        mask = off & 1 ? 0xFF00 : 0x00FF;
        v = off & 1 ? (val & 0xFF) << 8 : val & 0xFF;
    } else {
        mask = 0xFFFF;
        v = val;
    }

    switch (reg) {
    case R_START_STOP:
        if ((mask & 0x00FF) && (v & 1)) {
            /* START, or a repeated START */
            s->want_addr = true;
            sstar_i2c_step(s, ST_START);
        }
        if ((mask & 0xFF00) && (v & (1u << 8))) {
            sstar_i2c_stop(s);
            sstar_i2c_step(s, ST_STOP);
        }
        break;
    case R_WDATA:
        if (mask & 0x00FF) {
            sstar_i2c_write_byte(s, v & 0xFF);
        }
        break;
    case R_RDATA:
        if ((mask & 0xFF00) && (v & RDATA_READ)) {
            sstar_i2c_read_byte(s, v & RDATA_NACK);
        }
        break;
    case R_INT_FLAG:
        if (v & mask & 1) {
            s->regs[R_INT_FLAG] &= ~1;
            sstar_i2c_update_irq(s);
        }
        break;
    case R_STATUS:
    case R_TC_LO:
    case R_TC_HI:
        break;
    case R_DMA_DONE:
        if (v & mask & 1) {
            s->regs[R_DMA_DONE] &= ~1;
            sstar_i2c_update_irq(s);
        }
        break;
    case R_DMA_TRIGGER:
        if (v & mask & 1) {
            sstar_i2c_dma(s);
        }
        break;
    case R_CONFIG:
        s->regs[reg] = (s->regs[reg] & ~mask) | (v & mask);
        if (s->regs[reg] & 1) {
            sstar_i2c_stop(s);
        }
        sstar_i2c_update_irq(s);
        break;
    default:
        s->regs[reg] = (s->regs[reg] & ~mask) | (v & mask);
        if (reg == R_DMA_CONFIG) {
            sstar_i2c_update_irq(s);
        }
        break;
    }
}

static const MemoryRegionOps sstar_i2c_ops = {
    .read = sstar_i2c_read,
    .write = sstar_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_i2c_reset(DeviceState *dev)
{
    SstarI2cState *s = SSTAR_I2C(dev);

    if (s->in_xfer) {
        i2c_end_transfer(s->bus);
    }
    memset(s->regs, 0, sizeof(s->regs));
    s->want_addr = s->in_xfer = s->recv = false;
    sstar_i2c_update_irq(s);
}

static void sstar_i2c_init(Object *obj)
{
    SstarI2cState *s = SSTAR_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_i2c_ops, s,
                          TYPE_SSTAR_I2C, 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const VMStateDescription vmstate_sstar_i2c = {
    .name = TYPE_SSTAR_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarI2cState, 0x80),
        VMSTATE_BOOL(want_addr, SstarI2cState),
        VMSTATE_BOOL(in_xfer, SstarI2cState),
        VMSTATE_BOOL(recv, SstarI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_i2c_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_i2c_reset);
    dc->vmsd = &vmstate_sstar_i2c;
}

static const TypeInfo sstar_i2c_info = {
    .name           = TYPE_SSTAR_I2C,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarI2cState),
    .instance_init  = sstar_i2c_init,
    .class_init     = sstar_i2c_class_init,
};

static void sstar_i2c_register_types(void)
{
    type_register_static(&sstar_i2c_info);
}

type_init(sstar_i2c_register_types)
