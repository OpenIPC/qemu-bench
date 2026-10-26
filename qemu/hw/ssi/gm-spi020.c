/*
 * Grain Media FTSPI020 SPI flash controller.
 *
 * Faraday's command-based serial flash controller at 0x92300000 on the
 * GM813x parts (drivers/spi/ftspi020.c of the GM 3.3 kernel, U-Boot
 * drivers/mtd/spi/spi020.c, nsboot spi020.c).  A command is four
 * registers, written in order, the last one starting it:
 *
 *   0x00 CMD0  flash address
 *   0x04 CMD1  2:0 address bytes, 23:16 dummy cycles, 25:24 opcode
 *              bytes, 28 continuous read mode (send CMD3[23:16])
 *   0x08 CMD2  data bytes
 *   0x0C CMD3  31:24 opcode, 9:8 chip select, 7:5 I/O mode, 3 status
 *              read by software (else polled by hardware until the busy
 *              bit is clear), 2 this is a status read, 1 write, 0
 *              interrupt on completion
 *
 * then 0x10 CTRL (1:0 clock divider, 4 SPI mode 3, 8 abort -- self
 * clearing, 18:16 busy bit position), 0x14 AC timing, 0x18 STATUS (0 TX
 * FIFO ready, 1 RX FIFO ready), 0x20 interrupt control (0 DMA handshake,
 * 1 command complete interrupt), 0x24 interrupt status (0 command
 * complete, write 1 to clear), 0x28 the status a status-read command got,
 * 0x50 revision, 0x54 feature (FIFO depths) and the data port at 0x100.
 *
 * The data phase moves bytes through the data port, by PIO (nsboot polls
 * RX ready; the kernel without DMA does too) or by the AHB DMA, which the
 * kernel and U-Boot use: with handshake enabled the controller holds
 * "dma-req" high while it has data for the DMA (or wants data from it),
 * so the DMA moves the port at its pace.  A read clocks the flash into a
 * 128-byte RX FIFO and completes once the last byte is in it -- U-Boot
 * waits for completion before it drains a short read such as the JEDEC
 * ID -- while a write completes with its last byte.  Flash chips sit on
 * the SSI bus behind four chip selects; QEMU's m25p80 counts fast-read
 * dummies in cycles, one transfer each, which is what CMD1 holds.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_SPI020 "gm-spi020"
OBJECT_DECLARE_SIMPLE_TYPE(GmSpi020State, GM_SPI020)

#define GM_SPI020_SIZE      0x1000
#define GM_SPI020_NUM_CS    4
#define GM_SPI020_FIFO      128

#define R_CMD0              0x00
#define R_CMD1              0x04
#define R_CMD2              0x08
#define R_CMD3              0x0C
#define R_CTRL              0x10
#define R_AC_TIMING         0x14
#define R_STATUS            0x18
#define R_ICR               0x20
#define R_ISR               0x24
#define R_READ_STS          0x28
#define R_TX_CNT            0x30
#define R_RX_CNT            0x34
#define R_REVISION          0x50
#define R_FEATURE           0x54
#define R_DATA              0x100

#define CMD3_INTR           (1u << 0)
#define CMD3_WRITE          (1u << 1)
#define CMD3_RD_STS         (1u << 2)
#define CMD3_STS_SW         (1u << 3)

#define CTRL_ABORT          (1u << 8)

#define STATUS_TFR          (1u << 0)
#define STATUS_RFR          (1u << 1)

#define ICR_DMA             (1u << 0)
#define ICR_INT             (1u << 1)

#define ISR_CMD_CMPL        (1u << 0)

struct GmSpi020State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *ssi;
    qemu_irq irq;
    qemu_irq dma_req;
    qemu_irq cs[GM_SPI020_NUM_CS];

    uint32_t cmd[4];
    uint32_t ctrl;
    uint32_t ac_timing;
    uint32_t icr;
    uint32_t isr;
    uint32_t read_sts;
    uint32_t remaining;         /* data phase bytes still to clock */
    uint32_t moved;
    bool active;                /* chip selected, command running */
    bool writing;
    uint8_t cur_cs;
    uint8_t rx_fifo[GM_SPI020_FIFO];
    uint32_t rx_head;
    uint32_t rx_count;
};

static void gm_spi020_update(GmSpi020State *s)
{
    bool data = s->rx_count || (s->active && s->writing && s->remaining);

    qemu_set_irq(s->irq, (s->isr & ISR_CMD_CMPL) && (s->icr & ICR_INT));
    qemu_set_irq(s->dma_req, data && (s->icr & ICR_DMA));
}

static void gm_spi020_select(GmSpi020State *s, int cs, bool on)
{
    qemu_set_irq(s->cs[cs], !on);
}

static void gm_spi020_complete(GmSpi020State *s)
{
    if (s->active) {
        gm_spi020_select(s, s->cur_cs, false);
    }
    s->active = false;
    s->remaining = 0;
    s->isr |= ISR_CMD_CMPL;
    gm_spi020_update(s);
}

/* Clock the flash into the RX FIFO; the last byte completes the read. */
static void gm_spi020_fill(GmSpi020State *s)
{
    while (s->active && !s->writing && s->remaining &&
           s->rx_count < GM_SPI020_FIFO) {
        s->rx_fifo[(s->rx_head + s->rx_count) % GM_SPI020_FIFO] =
            ssi_transfer(s->ssi, 0);
        s->rx_count++;
        s->remaining--;
        s->moved++;
    }
    if (s->active && !s->writing && !s->remaining) {
        gm_spi020_complete(s);
    }
}

static void gm_spi020_start(GmSpi020State *s)
{
    uint32_t cmd1 = s->cmd[1], cmd3 = s->cmd[3];
    unsigned ins_len = extract32(cmd1, 24, 2);
    unsigned addr_len = extract32(cmd1, 0, 3);
    unsigned dummies = extract32(cmd1, 16, 8);
    uint8_t op = extract32(cmd3, 24, 8);
    int i;

    if (s->active) {
        gm_spi020_select(s, s->cur_cs, false);
        s->active = false;
    }
    s->rx_head = s->rx_count = 0;
    s->cur_cs = extract32(cmd3, 8, 2);
    s->moved = 0;
    gm_spi020_select(s, s->cur_cs, true);
    s->active = true;

    for (i = 0; i < ins_len; i++) {
        ssi_transfer(s->ssi, op);
    }
    for (i = addr_len; i > 0; i--) {
        ssi_transfer(s->ssi, extract32(s->cmd[0], (i - 1) * 8, 8));
    }
    if (cmd1 & (1u << 28)) {
        ssi_transfer(s->ssi, extract32(cmd3, 16, 8));
    }
    for (i = 0; i < dummies; i++) {
        ssi_transfer(s->ssi, 0);
    }

    if (cmd3 & CMD3_RD_STS) {
        /*
         * A status read: software reads the byte from READ_STS, or the
         * controller polls it until the busy bit clears.  QEMU's flash
         * finishes programs and erases at once, so one read does both.
         */
        s->read_sts = ssi_transfer(s->ssi, 0) & 0xFF;
        gm_spi020_complete(s);
        return;
    }
    s->writing = cmd3 & CMD3_WRITE;
    s->remaining = s->cmd[2];
    if (!s->remaining) {
        gm_spi020_complete(s);
        return;
    }
    gm_spi020_fill(s);
    gm_spi020_update(s);
}

static uint64_t gm_spi020_read(void *opaque, hwaddr off, unsigned size)
{
    GmSpi020State *s = opaque;
    uint32_t v = 0;
    int i;

    if (off >= R_DATA && off < R_DATA + 4) {
        if (!s->rx_count) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: RX FIFO empty\n", __func__);
            return 0;
        }
        for (i = 0; i < size && s->rx_count; i++) {
            v |= s->rx_fifo[s->rx_head] << (i * 8);
            s->rx_head = (s->rx_head + 1) % GM_SPI020_FIFO;
            s->rx_count--;
        }
        gm_spi020_fill(s);
        gm_spi020_update(s);
        return v;
    }
    switch (off) {
    case R_CMD0:
    case R_CMD1:
    case R_CMD2:
    case R_CMD3:
        return s->cmd[off / 4];
    case R_CTRL:
        return s->ctrl;
    case R_AC_TIMING:
        return s->ac_timing;
    case R_STATUS:
        /* the FIFOs never hold the guest up */
        return STATUS_TFR | (s->rx_count ? STATUS_RFR : 0);
    case R_ICR:
        return s->icr;
    case R_ISR:
        return s->isr;
    case R_READ_STS:
        return s->read_sts;
    case R_TX_CNT:
        return s->writing ? s->moved : 0;
    case R_RX_CNT:
        return s->writing ? 0 : s->moved;
    case R_REVISION:
        return 0x00010001;
    case R_FEATURE:
        /* clock mode and DTR, 2 queued commands, 32-word FIFOs */
        return (1u << 25) | (1u << 24) | (2u << 16) | (32u << 8) | 32u;
    }
    qemu_log_mask(LOG_UNIMP, "%s: read 0x%" HWADDR_PRIx "\n", __func__, off);
    return 0;
}

static void gm_spi020_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    GmSpi020State *s = opaque;
    int i;

    if (off >= R_DATA && off < R_DATA + 4) {
        if (!s->active || !s->writing) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: data port write with no write in progress\n",
                          __func__);
            return;
        }
        for (i = 0; i < size && s->remaining; i++) {
            ssi_transfer(s->ssi, extract32(val, i * 8, 8));
            s->remaining--;
            s->moved++;
        }
        if (!s->remaining) {
            gm_spi020_complete(s);
        }
        return;
    }
    switch (off) {
    case R_CMD0:
    case R_CMD1:
    case R_CMD2:
        s->cmd[off / 4] = val;
        break;
    case R_CMD3:
        s->cmd[3] = val;
        gm_spi020_start(s);
        break;
    case R_CTRL:
        if (val & CTRL_ABORT) {
            if (s->active) {
                gm_spi020_select(s, s->cur_cs, false);
            }
            s->active = false;
            s->remaining = 0;
            s->rx_head = s->rx_count = 0;
            gm_spi020_update(s);
        }
        s->ctrl = val & ~CTRL_ABORT;
        break;
    case R_AC_TIMING:
        s->ac_timing = val;
        break;
    case R_ICR:
        s->icr = val;
        gm_spi020_update(s);
        break;
    case R_ISR:
        s->isr &= ~val;
        gm_spi020_update(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: write 0x%" HWADDR_PRIx " = 0x%"
                      PRIx64 "\n", __func__, off, val);
        break;
    }
}

static const MemoryRegionOps gm_spi020_ops = {
    .read = gm_spi020_read,
    .write = gm_spi020_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void gm_spi020_reset(DeviceState *dev)
{
    GmSpi020State *s = GM_SPI020(dev);
    int i;

    memset(s->cmd, 0, sizeof(s->cmd));
    s->ctrl = 0;
    s->ac_timing = 0;
    s->icr = 0;
    s->isr = 0;
    s->read_sts = 0;
    s->remaining = 0;
    s->moved = 0;
    s->active = false;
    s->writing = false;
    s->rx_head = s->rx_count = 0;
    for (i = 0; i < GM_SPI020_NUM_CS; i++) {
        qemu_set_irq(s->cs[i], 1);
    }
    gm_spi020_update(s);
}

static void gm_spi020_realize(DeviceState *dev, Error **errp)
{
    GmSpi020State *s = GM_SPI020(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_spi020_ops, s,
                          TYPE_GM_SPI020, GM_SPI020_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->ssi = ssi_create_bus(dev, "ssi");
    qdev_init_gpio_out_named(dev, s->cs, SSI_GPIO_CS, GM_SPI020_NUM_CS);
    qdev_init_gpio_out_named(dev, &s->dma_req, "dma-req", 1);
}

static const VMStateDescription vmstate_gm_spi020 = {
    .name = TYPE_GM_SPI020,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cmd, GmSpi020State, 4),
        VMSTATE_UINT32(ctrl, GmSpi020State),
        VMSTATE_UINT32(ac_timing, GmSpi020State),
        VMSTATE_UINT32(icr, GmSpi020State),
        VMSTATE_UINT32(isr, GmSpi020State),
        VMSTATE_UINT32(read_sts, GmSpi020State),
        VMSTATE_UINT32(remaining, GmSpi020State),
        VMSTATE_UINT32(moved, GmSpi020State),
        VMSTATE_BOOL(active, GmSpi020State),
        VMSTATE_BOOL(writing, GmSpi020State),
        VMSTATE_UINT8(cur_cs, GmSpi020State),
        VMSTATE_UINT8_ARRAY(rx_fifo, GmSpi020State, GM_SPI020_FIFO),
        VMSTATE_UINT32(rx_head, GmSpi020State),
        VMSTATE_UINT32(rx_count, GmSpi020State),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_spi020_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_spi020_realize;
    device_class_set_legacy_reset(dc, gm_spi020_reset);
    dc->vmsd = &vmstate_gm_spi020;
}

static const TypeInfo gm_spi020_info = {
    .name           = TYPE_GM_SPI020,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmSpi020State),
    .class_init     = gm_spi020_class_init,
};

static void gm_spi020_register_types(void)
{
    type_register_static(&gm_spi020_info);
}

type_init(gm_spi020_register_types)
