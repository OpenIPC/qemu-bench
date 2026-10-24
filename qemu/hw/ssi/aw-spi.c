/*
 * Allwinner sun6i-style SPI controller (V831/V833 SPI0..2; the H6 and
 * sun50i parts have the same block).
 *
 *   0x04 GCR   bit 0 EN, 1 master, 7 transmit pause, 31 soft reset
 *   0x08 TCR   bits 5:4 SS_SEL, 6 SS_OWNER (software CS), 7 SS_LEVEL,
 *              8 DHB (discard what arrives while transmitting), 31 XCH
 *   0x10 IER / 0x14 ISR (write 1 to clear)
 *   0x18 FCR   bits 7:0 RX trigger, 8 RX DRQ, 15 RX reset,
 *              23:16 TX trigger, 24 TX DRQ, 31 TX reset
 *   0x1C FSR   bits 7:0 RX count, 23:16 TX count
 *   0x30 MBC   bursts in the exchange (counts down to 0 as they go; boot0
 *              checks it is 0 at the end), 0x34 MTC  of them transmitted,
 *   0x38 BCC   bits 23:0 single-mode transmit count, 27:24 dummy bursts,
 *              28 dual RX, 29 quad RX
 *   0x200 TXD, 0x300 RXD
 *
 * An exchange (XCH) clocks MBC bytes: the MTC transmit bytes from the TX
 * FIFO, then DBC dummies, then reads.  The flash model sees one byte per
 * clock-group whatever the wire width, which is what the drivers assume
 * when they count dummies in bytes.  The exchange waits whenever the
 * FIFOs say so, as the clock does on silicon: for the driver (or DMA) to
 * refill the TX FIFO before MTC bytes have gone, and to drain the 64-byte
 * RX FIFO when it is full -- U-Boot reads 128 KiB in one exchange.
 *
 * Chip select: automatic (asserted for the exchange) or, with SS_OWNER,
 * the SS_LEVEL bit.
 *
 * DRQ outputs for the DMA controller: 0 RX (RX DRQ on and data waiting),
 * 1 TX (TX DRQ on and room in the FIFO).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/fifo8.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AW_SPI "aw-spi"
OBJECT_DECLARE_SIMPLE_TYPE(AwSpiState, AW_SPI)

#define AW_SPI_SIZE         0x1000
#define AW_SPI_FIFO         64
#define AW_SPI_RX_MAX       AW_SPI_FIFO
#define AW_SPI_NUM_CS       4

#define REG_VER             0x00
#define REG_GCR             0x04
#define REG_TCR             0x08
#define REG_IER             0x10
#define REG_ISR             0x14
#define REG_FCR             0x18
#define REG_FSR             0x1C
#define REG_WCR             0x20
#define REG_CCR             0x24
#define REG_MBC             0x30
#define REG_MTC             0x34
#define REG_BCC             0x38
#define REG_TXD             0x200
#define REG_RXD             0x300

#define GCR_SRST            (1u << 31)
#define TCR_SS_SEL(v)       (((v) >> 4) & 3)
#define TCR_SS_OWNER        (1u << 6)
#define TCR_SS_LEVEL        (1u << 7)
#define TCR_DHB             (1u << 8)
#define TCR_XCH             (1u << 31)
#define FCR_RX_DRQ          (1u << 8)
#define FCR_RX_RST          (1u << 15)
#define FCR_TX_DRQ          (1u << 24)
#define FCR_TX_RST          (1u << 31)

#define ISR_RX_RDY          (1u << 0)
#define ISR_RX_EMP          (1u << 1)
#define ISR_RX_FULL         (1u << 2)
#define ISR_TX_ERQ          (1u << 4)
#define ISR_TX_EMP          (1u << 5)
#define ISR_TX_FULL         (1u << 6)
#define ISR_TC              (1u << 12)
#define ISR_LIVE            (ISR_RX_RDY | ISR_RX_EMP | ISR_RX_FULL | \
                             ISR_TX_ERQ | ISR_TX_EMP | ISR_TX_FULL)

struct AwSpiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    SSIBus *bus;
    qemu_irq irq;
    qemu_irq cs[AW_SPI_NUM_CS];
    qemu_irq drq[2];

    uint32_t gcr, tcr, ier, isr, fcr, wcr, ccr, mbc, mtc, bcc;
    Fifo8 tx;
    uint8_t *rx;
    uint32_t rx_head, rx_count;
    /* the exchange in progress: bursts done of mbc */
    bool busy;
    uint32_t done;
    int cs_active;              /* -1: none */
};

static uint32_t aw_spi_tx_trig(AwSpiState *s)
{
    return (s->fcr >> 16) & 0xFF;
}

static uint32_t aw_spi_rx_trig(AwSpiState *s)
{
    return s->fcr & 0xFF;
}

static uint32_t aw_spi_isr(AwSpiState *s)
{
    uint32_t v = s->isr & ~ISR_LIVE;
    uint32_t txn = fifo8_num_used(&s->tx);

    if (s->rx_count && s->rx_count >= MAX(aw_spi_rx_trig(s), 1)) {
        v |= ISR_RX_RDY;
    }
    if (!s->rx_count) {
        v |= ISR_RX_EMP;
    }
    if (s->rx_count >= AW_SPI_FIFO) {
        v |= ISR_RX_FULL;
    }
    if (txn <= aw_spi_tx_trig(s)) {
        v |= ISR_TX_ERQ;
    }
    if (!txn) {
        v |= ISR_TX_EMP;
    }
    if (txn == AW_SPI_FIFO) {
        v |= ISR_TX_FULL;
    }
    return v;
}

static void aw_spi_update(AwSpiState *s)
{
    qemu_set_irq(s->irq, !!(aw_spi_isr(s) & s->ier));
    qemu_set_irq(s->drq[0], (s->fcr & FCR_RX_DRQ) && s->rx_count);
    qemu_set_irq(s->drq[1], (s->fcr & FCR_TX_DRQ) &&
                 fifo8_num_free(&s->tx) > 0 && s->busy);
}

static void aw_spi_set_cs(AwSpiState *s, int cs)
{
    if (s->cs_active == cs) {
        return;
    }
    if (s->cs_active >= 0) {
        qemu_set_irq(s->cs[s->cs_active], 1);
    }
    s->cs_active = cs;
    if (cs >= 0) {
        qemu_set_irq(s->cs[cs], 0);
    }
}

/* Software chip select follows SS_LEVEL; automatic is the exchange. */
static void aw_spi_update_cs(AwSpiState *s)
{
    if (s->tcr & TCR_SS_OWNER) {
        aw_spi_set_cs(s, (s->tcr & TCR_SS_LEVEL) ? -1 : TCR_SS_SEL(s->tcr));
    } else {
        aw_spi_set_cs(s, s->busy ? TCR_SS_SEL(s->tcr) : -1);
    }
}

static void aw_spi_rx_push(AwSpiState *s, uint8_t b)
{
    if (s->rx_count == AW_SPI_RX_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "aw-spi: RX overflow\n");
        return;
    }
    s->rx[(s->rx_head + s->rx_count) % AW_SPI_RX_MAX] = b;
    s->rx_count++;
}

/* Clock bursts until the exchange ends or waits for TX data. */
static void aw_spi_run(AwSpiState *s)
{
    uint32_t mtc = s->mtc & 0xFFFFFF;
    uint32_t dbc = (s->bcc >> 24) & 0xF;
    uint32_t mbc = s->mbc & 0xFFFFFF;

    while (s->busy && s->done < mbc) {
        bool tx_phase = s->done < mtc;
        uint8_t out = 0;
        uint8_t in;

        if (tx_phase) {
            if (fifo8_is_empty(&s->tx)) {
                break;
            }
            out = fifo8_pop(&s->tx);
        } else if (s->rx_count == AW_SPI_FIFO) {
            break;
        }
        in = ssi_transfer(s->bus, out);
        if (!tx_phase && s->done >= mtc + dbc) {
            aw_spi_rx_push(s, in);
        } else if (tx_phase && !(s->tcr & TCR_DHB)) {
            aw_spi_rx_push(s, in);
        }
        s->done++;
    }
    if (s->busy && s->done >= mbc) {
        s->busy = false;
        s->mbc = 0;
        s->tcr &= ~TCR_XCH;
        s->isr |= ISR_TC;
        aw_spi_update_cs(s);
    }
    aw_spi_update(s);
}

static void aw_spi_soft_reset(AwSpiState *s)
{
    s->gcr = 0x80;              /* transmit pause enabled */
    s->tcr = 0x87;              /* SS_LEVEL, SPOL, CPOL, CPHA */
    s->ier = 0;
    s->isr = 0;
    s->fcr = 0x00400001;
    s->wcr = 0;
    s->ccr = 2;
    s->mbc = s->mtc = s->bcc = 0;
    fifo8_reset(&s->tx);
    s->rx_head = s->rx_count = 0;
    s->busy = false;
    s->done = 0;
    aw_spi_set_cs(s, -1);
}

static uint64_t aw_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    AwSpiState *s = opaque;
    uint64_t v = 0;
    unsigned i;

    switch (addr) {
    case REG_VER:
        return 0x00010001;
    case REG_GCR:
        return s->gcr;
    case REG_TCR:
        return s->tcr;
    case REG_IER:
        return s->ier;
    case REG_ISR:
        return aw_spi_isr(s);
    case REG_FCR:
        return s->fcr;
    case REG_FSR:
        return s->rx_count |
               (fifo8_num_used(&s->tx) << 16);
    case REG_WCR:
        return s->wcr;
    case REG_CCR:
        return s->ccr;
    case REG_MBC:
        return s->busy ? s->mbc - s->done : s->mbc;
    case REG_MTC:
        return s->mtc;
    case REG_BCC:
        return s->bcc;
    case REG_RXD ... REG_RXD + 3:
        for (i = 0; i < size; i++) {
            if (!s->rx_count) {
                qemu_log_mask(LOG_GUEST_ERROR, "aw-spi: RX underflow\n");
                break;
            }
            v |= (uint64_t)s->rx[s->rx_head] << (8 * i);
            s->rx_head = (s->rx_head + 1) % AW_SPI_RX_MAX;
            s->rx_count--;
        }
        if (s->busy) {
            aw_spi_run(s);
        } else {
            aw_spi_update(s);
        }
        return v;
    }
    return 0;
}

static void aw_spi_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    AwSpiState *s = opaque;
    unsigned i;

    switch (addr) {
    case REG_GCR:
        if (val & GCR_SRST) {
            aw_spi_soft_reset(s);
            val &= ~GCR_SRST;
        }
        s->gcr = val;
        break;
    case REG_TCR:
        s->tcr = val & ~TCR_XCH;
        if ((val & TCR_XCH) && !s->busy) {
            s->tcr |= TCR_XCH;
            s->busy = true;
            s->done = 0;
            aw_spi_update_cs(s);
            aw_spi_run(s);
            return;
        }
        if (s->busy) {
            s->tcr |= TCR_XCH;
        }
        aw_spi_update_cs(s);
        break;
    case REG_IER:
        s->ier = val;
        break;
    case REG_ISR:
        s->isr &= ~val;
        break;
    case REG_FCR:
        if (val & FCR_TX_RST) {
            fifo8_reset(&s->tx);
        }
        if (val & FCR_RX_RST) {
            s->rx_head = s->rx_count = 0;
        }
        s->fcr = val & ~(FCR_TX_RST | FCR_RX_RST);
        break;
    case REG_WCR:
        s->wcr = val;
        break;
    case REG_CCR:
        s->ccr = val;
        break;
    case REG_MBC:
        s->mbc = val & 0xFFFFFF;
        break;
    case REG_MTC:
        s->mtc = val & 0xFFFFFF;
        break;
    case REG_BCC:
        s->bcc = val;
        break;
    case REG_TXD ... REG_TXD + 3:
        for (i = 0; i < size; i++) {
            if (fifo8_is_full(&s->tx)) {
                qemu_log_mask(LOG_GUEST_ERROR, "aw-spi: TX overflow\n");
                break;
            }
            fifo8_push(&s->tx, val >> (8 * i));
        }
        if (s->busy) {
            aw_spi_run(s);
            return;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "aw-spi: write 0x%03x = 0x%08x\n",
                      (unsigned)addr, (uint32_t)val);
        return;
    }
    aw_spi_update(s);
}

static const MemoryRegionOps aw_spi_ops = {
    .read = aw_spi_read,
    .write = aw_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void aw_spi_reset_hold(Object *obj, ResetType type)
{
    aw_spi_soft_reset(AW_SPI(obj));
}

static void aw_spi_init(Object *obj)
{
    AwSpiState *s = AW_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &aw_spi_ops, s, TYPE_AW_SPI,
                          AW_SPI_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = ssi_create_bus(DEVICE(obj), "ssi");
    qdev_init_gpio_out_named(DEVICE(obj), s->cs, SSI_GPIO_CS, AW_SPI_NUM_CS);
    qdev_init_gpio_out_named(DEVICE(obj), s->drq, "drq", 2);
    fifo8_create(&s->tx, AW_SPI_FIFO);
    s->rx = g_malloc0(AW_SPI_RX_MAX);
    s->cs_active = -1;
}

static void aw_spi_finalize(Object *obj)
{
    AwSpiState *s = AW_SPI(obj);

    fifo8_destroy(&s->tx);
    g_free(s->rx);
}

static void aw_spi_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_spi_reset_hold;
}

static const TypeInfo aw_spi_info = {
    .name              = TYPE_AW_SPI,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(AwSpiState),
    .instance_init     = aw_spi_init,
    .instance_finalize = aw_spi_finalize,
    .class_init        = aw_spi_class_init,
};

static void aw_spi_register_types(void)
{
    type_register_static(&aw_spi_info);
}

type_init(aw_spi_register_types)
