/*
 * Fullhan SPI controller: a DesignWare SSI with Fullhan's "SPIC" additions.
 *
 * DesignWare registers (U-Boot drivers/spi/fh_spi.h, the kernel's
 * drivers/spi/fh_spi.c):
 *
 *   0x00 CTRLR0   frame size 3:0 (DFS - 1), mode 7:6, TMOD 9:8 (0 TX+RX,
 *                 1 TX only, 2 RX only, 3 EEPROM), DFS_32 20:16
 *   0x04 CTRLR1   frames to receive - 1 in RX-only mode
 *   0x08 SSIENR   0x10 SER   0x14 BAUDR   0x18 TXFTLR   0x1C RXFTLR
 *   0x20 TXFLR    0x24 RXFLR 0x28 SR      0x2C IMR      0x30 ISR
 *   0x34 RISR     0x38-0x48 interrupt clears   0x4C DMACR (bit 0 RX,
 *   bit 1 TX)     0x50 DMATDLR   0x54 DMARDLR   0x58 IDR   0x5C version
 *   0x60 DR       (to 0xEC)      0xF0 RX_SAMPLE_DLY
 *
 * SPIC, on SPI0 only (fh_spi_extend.c, fh_spi_multi_wire.c):
 *
 *   0xF4 CCFGR    1:0 wire mode of the data phase (0 single, 1 dual,
 *                 2 quad), 12 byte swap, 16 32-bit frames, 15 clock mask
 *   0xF8 OPCR, 0xFC TIMCR, 0x100/0x104 BBAR0/1: XIP and timing, storage
 *   SR bit 9 / 10: the AHB (XIP) TX FIFO empty / RX FIFO not empty
 *   0x1000-0x1FFF a data window: every word in it is DR, so a DMA burst
 *                 can increment its source address
 *
 * Wider frames are how the drivers speed up flash reads: in RX-only mode
 * they switch to 16 or 32-bit frames with the byte swap on, so a frame is
 * the next bytes off the wire in memory order and the DMA (or the CPU)
 * stores whole frames.  Without the swap a frame is shifted MSB first,
 * the first byte in its top bits, as on any DW SSI.  Multi-wire data
 * phases are transparent: the flash model answers dual and quad reads
 * byte by byte.
 *
 * The chip selects are GPIOs on every Fullhan board (cs0_gpio / cs1_gpio
 * in the DTS, gpio_direction_output() around each transfer); SER is set
 * too, but toggling SSIENR between the command and data phases, as both
 * drivers do, would drop a hardware select.  So the flash's select is
 * the "cs-gpio" input, wired from the GPIO block by the machine.
 *
 * QEMU's flash models count fast-read dummy cycles one per transfer (a
 * Winbond, Macronix or GigaDevice part wants 8 for 0x0B / 0x3B / 0x6B),
 * where these drivers send one dummy byte.  The controller follows the
 * command from the select on and clocks a dummy byte as its 8 cycles.
 *
 * Frames move at once.  In RX-only mode writing DR starts the reception
 * of CTRLR1 + 1 frames; the RX FIFO is kept full from the flash as it is
 * read, so a DMA drains a whole transfer while its request line is up.
 * The request lines ("dma-rx-req", "dma-tx-req") follow DMACR and the
 * FIFO levels, for the AXI DMAC's hardware handshake.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_FH_SPI "fh-spi"
OBJECT_DECLARE_SIMPLE_TYPE(FhSpiState, FH_SPI)

#define FH_SPI_SIZE         0x4000
#define FH_SPI_WINDOW       0x1000
#define FH_SPI_FIFO_MAX     256

#define R_CTRLR0            0x00
#define R_CTRLR1            0x04
#define R_SSIENR            0x08
#define R_MWCR              0x0C
#define R_SER               0x10
#define R_BAUDR             0x14
#define R_TXFTLR            0x18
#define R_RXFTLR            0x1C
#define R_TXFLR             0x20
#define R_RXFLR             0x24
#define R_SR                0x28
#define R_IMR               0x2C
#define R_ISR               0x30
#define R_RISR              0x34
#define R_TXOICR            0x38
#define R_RXOICR            0x3C
#define R_RXUICR            0x40
#define R_MSTICR            0x44
#define R_ICR               0x48
#define R_DMACR             0x4C
#define R_DMATDLR           0x50
#define R_DMARDLR           0x54
#define R_IDR               0x58
#define R_VERSION           0x5C
#define R_DR                0x60
#define R_DR_END            0xEC
#define R_SAMPLE_DLY        0xF0
#define R_CCFGR             0xF4
#define R_OPCR              0xF8
#define R_TIMCR             0xFC
#define R_BBAR0             0x100
#define R_BBAR1             0x104

#define TMOD_TXRX           0
#define TMOD_TX             1
#define TMOD_RX             2
#define TMOD_EEPROM         3

#define SR_TFNF             (1u << 1)
#define SR_TFE              (1u << 2)
#define SR_RFNE             (1u << 3)
#define SR_RFF              (1u << 4)
#define SR_AHB_TFE          (1u << 9)

#define INT_TXE             (1u << 0)
#define INT_TXO             (1u << 1)
#define INT_RXU             (1u << 2)
#define INT_RXO             (1u << 3)
#define INT_RXF             (1u << 4)

#define DMACR_RDMAE         (1u << 0)
#define DMACR_TDMAE         (1u << 1)

#define CCFGR_SWAP          (1u << 12)
#define CCFGR_32BIT         (1u << 16)

struct FhSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *bus;
    qemu_irq irq;
    qemu_irq cs;
    qemu_irq dma_rx_req;
    qemu_irq dma_tx_req;

    uint32_t ctrlr0;
    uint32_t ctrlr1;
    uint32_t ssienr;
    uint32_t mwcr;
    uint32_t ser;
    uint32_t baudr;
    uint32_t txftlr;
    uint32_t rxftlr;
    uint32_t imr;
    uint32_t risr;
    uint32_t dmacr;
    uint32_t dmatdlr;
    uint32_t dmardlr;
    uint32_t sample_dly;
    uint32_t ccfgr;
    uint32_t opcr;
    uint32_t timcr;
    uint32_t bbar[2];

    uint32_t rx_fifo[FH_SPI_FIFO_MAX];
    uint32_t rx_head;
    uint32_t rx_count;
    uint32_t rx_pending;        /* RX-only frames still to clock in */
    bool cs_level;              /* the GPIO select, low = selected */
    uint32_t cmd_pos;           /* bytes since the select went low */
    uint8_t cmd;                /* their first: the flash command */

    uint32_t fifo_depth;
    uint32_t idr;
};

static unsigned fh_spi_frame_bytes(FhSpiState *s)
{
    unsigned bits;

    if (s->ccfgr & CCFGR_32BIT) {
        bits = ((s->ctrlr0 >> 16) & 0x1F) + 1;
    } else {
        bits = (s->ctrlr0 & 0xF) + 1;
    }
    return bits > 16 ? 4 : bits > 8 ? 2 : 1;
}

static void fh_spi_update(FhSpiState *s)
{
    uint32_t risr = s->risr & (INT_TXO | INT_RXU | INT_RXO);

    if (s->ssienr) {
        risr |= INT_TXE;        /* the TX FIFO is always empty */
        if (s->rx_count > s->rxftlr) {
            risr |= INT_RXF;
        }
    }
    s->risr = risr;
    qemu_set_irq(s->irq, (risr & s->imr) != 0);
    qemu_set_irq(s->dma_rx_req,
                 s->ssienr && (s->dmacr & DMACR_RDMAE) && s->rx_count);
    qemu_set_irq(s->dma_tx_req, s->ssienr && (s->dmacr & DMACR_TDMAE));
}

/* Where the dummy byte of a fast read falls, or 0. */
static uint32_t fh_spi_dummy_pos(uint8_t cmd)
{
    switch (cmd) {
    case 0x0B:                  /* FAST_READ */
    case 0x3B:                  /* DOR */
    case 0x6B:                  /* QOR */
        return 4;
    case 0x0C:                  /* their 4-byte address forms */
    case 0x3C:
    case 0x6C:
        return 5;
    default:
        return 0;
    }
}

static uint8_t fh_spi_byte(FhSpiState *s, uint8_t tx)
{
    uint32_t pos = s->cmd_pos++;
    uint8_t rx;
    int i;

    if (pos == 0) {
        s->cmd = tx;
    } else if (pos == fh_spi_dummy_pos(s->cmd)) {
        for (i = 0; i < 7; i++) {
            ssi_transfer(s->bus, tx);
        }
    }
    rx = ssi_transfer(s->bus, tx);
    return rx;
}

/* One frame on the wire: tx out, the received frame back. */
static uint32_t fh_spi_xfer_frame(FhSpiState *s, uint32_t tx)
{
    unsigned n = fh_spi_frame_bytes(s);
    bool swap = s->ccfgr & CCFGR_SWAP;
    uint32_t rx = 0;
    unsigned i;

    for (i = 0; i < n; i++) {
        unsigned shift = swap ? 8 * i : 8 * (n - 1 - i);
        uint32_t b = fh_spi_byte(s, (tx >> shift) & 0xFF);

        rx |= b << shift;
    }
    return rx;
}

static void fh_spi_rx_push(FhSpiState *s, uint32_t v)
{
    if (s->rx_count >= s->fifo_depth) {
        s->risr |= INT_RXO;
        return;
    }
    s->rx_fifo[(s->rx_head + s->rx_count) % s->fifo_depth] = v;
    s->rx_count++;
}

static void fh_spi_rx_fill(FhSpiState *s)
{
    while (s->rx_pending && s->rx_count < s->fifo_depth) {
        uint32_t dummy = (s->ccfgr & CCFGR_32BIT) ? 0xFFFFFFFF : 0xFFFF;

        fh_spi_rx_push(s, fh_spi_xfer_frame(s, dummy));
        s->rx_pending--;
    }
}

static void fh_spi_flush(FhSpiState *s)
{
    s->rx_head = 0;
    s->rx_count = 0;
    s->rx_pending = 0;
}

static void fh_spi_dr_write(FhSpiState *s, uint32_t val)
{
    uint32_t rx;

    if (!s->ssienr) {
        return;
    }
    switch ((s->ctrlr0 >> 8) & 3) {
    case TMOD_RX:
        /* Any write starts the reception; the data is not sent. */
        if (!s->rx_pending && !s->rx_count) {
            s->rx_pending = s->ctrlr1 + 1;
            fh_spi_rx_fill(s);
        }
        return;
    case TMOD_TX:
        fh_spi_xfer_frame(s, val);
        return;
    case TMOD_EEPROM:
        /*
         * Command bytes out, then CTRLR1 + 1 frames in once the TX FIFO
         * runs dry -- which, frames moving at once, is after every write;
         * a further write of the same command phase restarts the count.
         */
        fh_spi_xfer_frame(s, val);
        s->rx_pending = s->ctrlr1 + 1;
        fh_spi_rx_fill(s);
        return;
    default:
        rx = fh_spi_xfer_frame(s, val);
        fh_spi_rx_push(s, rx);
        return;
    }
}

static uint32_t fh_spi_dr_read(FhSpiState *s)
{
    uint32_t v;

    if (!s->rx_count) {
        s->risr |= INT_RXU;
        return 0;
    }
    v = s->rx_fifo[s->rx_head];
    s->rx_head = (s->rx_head + 1) % s->fifo_depth;
    s->rx_count--;
    fh_spi_rx_fill(s);
    return v;
}

static uint64_t fh_spi_read_reg(FhSpiState *s, hwaddr off)
{
    uint32_t v = 0;

    if (off >= FH_SPI_WINDOW && off < 2 * FH_SPI_WINDOW) {
        v = fh_spi_dr_read(s);
        fh_spi_update(s);
        return v;
    }
    switch (off) {
    case R_CTRLR0:   return s->ctrlr0;
    case R_CTRLR1:   return s->ctrlr1;
    case R_SSIENR:   return s->ssienr;
    case R_MWCR:     return s->mwcr;
    case R_SER:      return s->ser;
    case R_BAUDR:    return s->baudr;
    case R_TXFTLR:   return s->txftlr;
    case R_RXFTLR:   return s->rxftlr;
    case R_TXFLR:    return 0;
    case R_RXFLR:    return s->rx_count;
    case R_SR:
        v = SR_TFNF | SR_TFE | SR_AHB_TFE;
        if (s->rx_count) {
            v |= SR_RFNE;
        }
        if (s->rx_count >= s->fifo_depth) {
            v |= SR_RFF;
        }
        return v;
    case R_IMR:      return s->imr;
    case R_ISR:      return s->risr & s->imr;
    case R_RISR:     return s->risr;
    case R_TXOICR:
        s->risr &= ~INT_TXO;
        break;
    case R_RXOICR:
        s->risr &= ~INT_RXO;
        break;
    case R_RXUICR:
        s->risr &= ~INT_RXU;
        break;
    case R_MSTICR:
        break;
    case R_ICR:
        s->risr &= ~(INT_TXO | INT_RXU | INT_RXO);
        break;
    case R_DMACR:    return s->dmacr;
    case R_DMATDLR:  return s->dmatdlr;
    case R_DMARDLR:  return s->dmardlr;
    case R_IDR:      return s->idr;
    case R_VERSION:  return 0x3332332A;  /* "323*" */
    case R_DR ... R_DR_END:
        v = fh_spi_dr_read(s);
        break;
    case R_SAMPLE_DLY: return s->sample_dly;
    case R_CCFGR:    return s->ccfgr;
    case R_OPCR:     return s->opcr;
    case R_TIMCR:    return s->timcr;
    case R_BBAR0:    return s->bbar[0];
    case R_BBAR1:    return s->bbar[1];
    default:
        qemu_log_mask(LOG_UNIMP, "fh-spi: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
    fh_spi_update(s);
    return v;
}

/*
 * Narrow accesses: DR and the window pop one frame whatever the width
 * and address; elsewhere they read the bytes of the register they fall in.
 */
static uint64_t fh_spi_read(void *opaque, hwaddr off, unsigned size)
{
    bool dr = (off >= R_DR && off <= R_DR_END + 3) ||
              (off >= FH_SPI_WINDOW && off < 2 * FH_SPI_WINDOW);
    uint32_t v = fh_spi_read_reg(opaque, dr ? R_DR : off & ~3);

    /* A DMA source walking the window by its item size reads frames. */
    return extract32(v, dr ? 0 : (off & 3) * 8, size * 8);
}

static void fh_spi_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhSpiState *s = opaque;

    off &= ~3;
    if (off >= FH_SPI_WINDOW && off < 2 * FH_SPI_WINDOW) {
        fh_spi_dr_write(s, val);
        fh_spi_update(s);
        return;
    }
    switch (off) {
    case R_CTRLR0:
        s->ctrlr0 = val;
        break;
    case R_CTRLR1:
        s->ctrlr1 = val & 0xFFFF;
        break;
    case R_SSIENR:
        s->ssienr = val & 1;
        if (!s->ssienr) {
            fh_spi_flush(s);
            s->risr = 0;
        }
        break;
    case R_MWCR:
        s->mwcr = val;
        break;
    case R_SER:
        s->ser = val;
        break;
    case R_BAUDR:
        s->baudr = val & 0xFFFF;
        break;
    case R_TXFTLR:
        /* Wider than the FIFO is refused: the kernel sizes it this way. */
        if (val < s->fifo_depth) {
            s->txftlr = val;
        }
        break;
    case R_RXFTLR:
        if (val < s->fifo_depth) {
            s->rxftlr = val;
        }
        break;
    case R_IMR:
        s->imr = val & 0x3F;
        break;
    case R_DMACR:
        s->dmacr = val & 3;
        break;
    case R_DMATDLR:
        s->dmatdlr = val;
        break;
    case R_DMARDLR:
        s->dmardlr = val;
        break;
    case R_DR ... R_DR_END:
        fh_spi_dr_write(s, val);
        break;
    case R_SAMPLE_DLY:
        s->sample_dly = val;
        break;
    case R_CCFGR:
        s->ccfgr = val;
        break;
    case R_OPCR:
        s->opcr = val;
        break;
    case R_TIMCR:
        s->timcr = val;
        break;
    case R_BBAR0:
        s->bbar[0] = val;
        break;
    case R_BBAR1:
        s->bbar[1] = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "fh-spi: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        return;
    }
    fh_spi_update(s);
}

static const MemoryRegionOps fh_spi_ops = {
    .read = fh_spi_read,
    .write = fh_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void fh_spi_cs_gpio(void *opaque, int n, int level)
{
    FhSpiState *s = opaque;

    if (s->cs_level != !!level) {
        s->cs_level = !!level;
        s->cmd_pos = 0;
        qemu_set_irq(s->cs, s->cs_level);
    }
}

static void fh_spi_reset(DeviceState *dev)
{
    FhSpiState *s = FH_SPI(dev);

    s->ctrlr0 = 0x7;            /* 8-bit Motorola frames, TX+RX */
    s->ctrlr1 = 0;
    s->ssienr = 0;
    s->mwcr = 0;
    s->ser = 0;
    s->baudr = 0;
    s->txftlr = 0;
    s->rxftlr = 0;
    /*
     * The core resets with every interrupt enabled, but the kernel only
     * masks them after request_irq() and its handler acks nothing, so
     * start masked rather than storm on the always-empty TX FIFO.
     */
    s->imr = 0;
    s->risr = 0;
    s->dmacr = 0;
    s->dmatdlr = 0;
    s->dmardlr = 0;
    s->sample_dly = 0;
    s->ccfgr = 0;
    s->opcr = 0;
    s->timcr = 0;
    s->bbar[0] = s->bbar[1] = 0;
    fh_spi_flush(s);
    fh_spi_update(s);
}

static void fh_spi_realize(DeviceState *dev, Error **errp)
{
    FhSpiState *s = FH_SPI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->fifo_depth < 2 || s->fifo_depth > FH_SPI_FIFO_MAX) {
        error_setg(errp, "fh-spi: fifo-depth must be 2..%d",
                   FH_SPI_FIFO_MAX);
        return;
    }
    s->bus = ssi_create_bus(dev, "ssi");
    memory_region_init_io(&s->iomem, OBJECT(s), &fh_spi_ops, s,
                          TYPE_FH_SPI, FH_SPI_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, &s->cs, SSI_GPIO_CS, 1);
    qdev_init_gpio_out_named(dev, &s->dma_rx_req, "dma-rx-req", 1);
    qdev_init_gpio_out_named(dev, &s->dma_tx_req, "dma-tx-req", 1);
    qdev_init_gpio_in_named(dev, fh_spi_cs_gpio, "cs-gpio", 1);
    s->cs_level = true;
}

static const Property fh_spi_props[] = {
    DEFINE_PROP_UINT32("fifo-depth", FhSpiState, fifo_depth, 64),
    DEFINE_PROP_UINT32("idr", FhSpiState, idr, 0),
};

static const VMStateDescription vmstate_fh_spi = {
    .name = TYPE_FH_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrlr0, FhSpiState),
        VMSTATE_UINT32(ctrlr1, FhSpiState),
        VMSTATE_UINT32(ssienr, FhSpiState),
        VMSTATE_UINT32(mwcr, FhSpiState),
        VMSTATE_UINT32(ser, FhSpiState),
        VMSTATE_UINT32(baudr, FhSpiState),
        VMSTATE_UINT32(txftlr, FhSpiState),
        VMSTATE_UINT32(rxftlr, FhSpiState),
        VMSTATE_UINT32(imr, FhSpiState),
        VMSTATE_UINT32(risr, FhSpiState),
        VMSTATE_UINT32(dmacr, FhSpiState),
        VMSTATE_UINT32(dmatdlr, FhSpiState),
        VMSTATE_UINT32(dmardlr, FhSpiState),
        VMSTATE_UINT32(sample_dly, FhSpiState),
        VMSTATE_UINT32(ccfgr, FhSpiState),
        VMSTATE_UINT32(opcr, FhSpiState),
        VMSTATE_UINT32(timcr, FhSpiState),
        VMSTATE_UINT32_ARRAY(bbar, FhSpiState, 2),
        VMSTATE_UINT32_ARRAY(rx_fifo, FhSpiState, FH_SPI_FIFO_MAX),
        VMSTATE_UINT32(rx_head, FhSpiState),
        VMSTATE_UINT32(rx_count, FhSpiState),
        VMSTATE_UINT32(rx_pending, FhSpiState),
        VMSTATE_BOOL(cs_level, FhSpiState),
        VMSTATE_UINT32(cmd_pos, FhSpiState),
        VMSTATE_UINT8(cmd, FhSpiState),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_spi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = fh_spi_realize;
    device_class_set_legacy_reset(dc, fh_spi_reset);
    device_class_set_props(dc, fh_spi_props);
    dc->vmsd = &vmstate_fh_spi;
}

static const TypeInfo fh_spi_info = {
    .name           = TYPE_FH_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhSpiState),
    .class_init     = fh_spi_class_init,
};

static void fh_spi_register_types(void)
{
    type_register_static(&fh_spi_info);
}

type_init(fh_spi_register_types)
