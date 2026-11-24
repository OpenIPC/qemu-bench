/*
 * Faraday FTSSP010 as an SPI master, the Grain Media GM8126's flash
 * interface (0x98200000).
 *
 * 0x00 CR0, 0x04 CR1 (the frame length - 1 at 20:16), 0x08 CR2 (bit 0
 * enable, 2 / 3 clear the receive / transmit FIFO), 0x0C status (bit 1
 * transmit FIFO not full, 8:4 received words), 0x18 data, 0x64 the FIFO
 * depths, 0x10 ICR (bits 4 / 5 receive / transmit DMA requests "dma-rx",
 * "dma-tx", which nsboot reads the flash with).  Each data write shifts one frame, most significant bit first,
 * and queues what came back.  The chip select is a GPIO.
 *
 * The same block carries I2S on other ports; that is gm-ssp.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_SSP_SPI "gm-ssp-spi"
OBJECT_DECLARE_SIMPLE_TYPE(GmSspSpiState, GM_SSP_SPI)

#define SSP_SIZE    0x1000
#define SSP_NREGS   (0x70 / 4)
#define SSP_FIFO    16

#define R_CR1       0x04
#define R_CR2       0x08
#define R_STATUS    0x0C
#define R_ICR       0x10
#define R_ISR       0x14
#define R_DATA      0x18
#define R_FEATURE   0x64

struct GmSspSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dma_rx, dma_tx;
    SSIBus *bus;
    uint32_t regs[SSP_NREGS];
    uint32_t rx[SSP_FIFO];
    uint32_t rx_len, rx_pos;
};

static void ssp_spi_update(GmSspSpiState *s)
{
    const uint32_t icr = s->regs[R_ICR / 4];

    qemu_set_irq(s->dma_rx, (icr & (1u << 4)) && s->rx_len);
    qemu_set_irq(s->dma_tx, (icr & (1u << 5)) && s->rx_len < SSP_FIFO);
}

static uint64_t ssp_spi_read(void *opaque, hwaddr off, unsigned size)
{
    GmSspSpiState *s = opaque;
    uint32_t v;

    switch (off) {
    case R_STATUS:
        return (1u << 1) | (s->rx_len << 4) | (s->rx_len == SSP_FIFO);
    case R_DATA:
        if (!s->rx_len) {
            return 0;
        }
        v = s->rx[s->rx_pos];
        s->rx_pos = (s->rx_pos + 1) % SSP_FIFO;
        s->rx_len--;
        ssp_spi_update(s);
        return v;
    case R_FEATURE:     /* SPI, 16-word FIFOs, 32-bit words */
        return (1u << 26) | (SSP_FIFO - 1) << 16 | (SSP_FIFO - 1) << 8 | 31;
    }
    return off / 4 < SSP_NREGS ? s->regs[off / 4] : 0;
}

static void ssp_spi_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmSspSpiState *s = opaque;

    if (off / 4 >= SSP_NREGS) {
        return;
    }
    switch (off) {
    case R_DATA: {
        const int bytes = extract32(s->regs[R_CR1 / 4], 16, 5) / 8 + 1;
        uint32_t r = 0;
        int i;

        for (i = bytes - 1; i >= 0; i--) {
            r = r << 8 | (ssi_transfer(s->bus, extract32(val, 8 * i, 8)) &
                          0xFF);
        }
        if (s->rx_len < SSP_FIFO) {
            s->rx[(s->rx_pos + s->rx_len++) % SSP_FIFO] = r;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "gm-ssp-spi: receive overrun\n");
        }
        ssp_spi_update(s);
        return;
    }
    case R_CR2:
        if (val & (1u << 2)) {
            s->rx_len = 0;
        }
        val &= ~0xCu;
        break;
    case R_ISR:
        return;
    }
    s->regs[off / 4] = val;
    ssp_spi_update(s);
}

static const MemoryRegionOps ssp_spi_ops = {
    .read = ssp_spi_read,
    .write = ssp_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ssp_spi_reset(DeviceState *dev)
{
    GmSspSpiState *s = GM_SSP_SPI(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->rx_len = s->rx_pos = 0;
    ssp_spi_update(s);
}

static void ssp_spi_realize(DeviceState *dev, Error **errp)
{
    GmSspSpiState *s = GM_SSP_SPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ssp_spi_ops, s,
                          TYPE_GM_SSP_SPI, SSP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    qdev_init_gpio_out_named(dev, &s->dma_rx, "dma-rx", 1);
    qdev_init_gpio_out_named(dev, &s->dma_tx, "dma-tx", 1);
    s->bus = ssi_create_bus(dev, "ssi");
}

static const VMStateDescription vmstate_ssp_spi = {
    .name = TYPE_GM_SSP_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmSspSpiState, SSP_NREGS),
        VMSTATE_UINT32_ARRAY(rx, GmSspSpiState, SSP_FIFO),
        VMSTATE_UINT32(rx_len, GmSspSpiState),
        VMSTATE_UINT32(rx_pos, GmSspSpiState),
        VMSTATE_END_OF_LIST()
    }
};

static void ssp_spi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ssp_spi_realize;
    device_class_set_legacy_reset(dc, ssp_spi_reset);
    dc->vmsd = &vmstate_ssp_spi;
}

static const TypeInfo ssp_spi_info = {
    .name = TYPE_GM_SSP_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GmSspSpiState),
    .class_init = ssp_spi_class_init,
};

static void ssp_spi_register_types(void)
{
    type_register_static(&ssp_spi_info);
}

type_init(ssp_spi_register_types)
