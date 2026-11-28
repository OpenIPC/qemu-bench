/*
 * Novatek NA51055 / NA51089 SPI controller ("nvt,nvt_spi").
 *
 * drivers/spi/spi-na51055.c and plat-na51089/spi_reg.h.  The NT9856x has
 * three of these controllers, at 0xF0230000, 0xF0320000 and 0xF0340000,
 * on interrupts 35, 36 and 37.  They are for external chips, a lens-motor
 * driver for one.  The driver sets a bus up and then moves every transfer
 * through the data registers, polling:
 *
 *   0x00 CTRL    bit 0 SPI_EN, bit 1 SPICS_VALUE (the level forced on the
 *                CS pin: 0 selects).  nvt_spi_setup() rewrites SPI_EN
 *                until it reads back set and nvt_spi_set_npcs() re-reads
 *                until SPICS_VALUE matches what it wrote, so both must
 *                read back; with nothing here they read 0 and setup never
 *                returns.  Bits 2..7 start / abort the DMA, flash-status
 *                and gyro modes and [18:16] are their status: not
 *                modelled, the strobes read back 0 and the modes idle
 *   0x08 CONFIG  bit 0 LSB first, bit 2 PKTLEN (0: a packet is one byte,
 *                1: two), [5:4] PKT_CNT (packets per TDR write, less one)
 *   0x18 STS     bit 0 TDR_EMPTY, bit 1 RDR_FULL, both live; bits 2..9
 *                the mode events, write 1 to clear
 *   0x1C INTEN   the same bits; the interrupt is STS & INTEN
 *   0x20 RDR     the packets received, packet i in the bits packet i of
 *                TDR went out from; reading it drops RDR_FULL
 *   0x24 TDR     a write shifts PKT_CNT packets out, the first from the
 *                low bits (nvt_spi_transfer_polled() packs tx_data[0]
 *                there)
 *   0x04 IO, 0x0C TIMING, 0x10 FLASH_CTRL, 0x14 DLY_CHAIN, 0x30.. DMA,
 *   0x40.. gyro: storage
 *
 * A TDR write completes at once.  TDR_EMPTY always reads set, and
 * RDR_FULL reads set from the write until RDR is read.  spi_wait_tdre()
 * and spi_wait_rdrf() poll for exactly these two bits.  Each packet is
 * one word on the SSI bus, 8 or 16 bits wide, with the value the driver
 * wrote.  Clock mode (IO), bit order and sample delay are stored for the
 * driver to read back and for a slave model to look up, and otherwise
 * ignored.  An empty bus reads 0.  "cs" follows SPICS_VALUE.  The driver
 * also drives a cs-gpios pad, which the GPIO model handles.
 *
 * The reset values are not in the driver (setup writes every register it
 * depends on); CTRL resets with SPICS_VALUE set, the CS pin idle.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_SPI "nvt-spi"
OBJECT_DECLARE_SIMPLE_TYPE(NvtSpiState, NVT_SPI)

#define SPI_REGS_SIZE   0x100
#define SPI_CTRL        0x00
#define SPI_CONFIG      0x08
#define SPI_STS         0x18
#define SPI_INTEN       0x1C
#define SPI_RDR         0x20
#define SPI_TDR         0x24

#define CTRL_EN         (1u << 0)
#define CTRL_CS         (1u << 1)
#define CTRL_KEPT       (CTRL_EN | CTRL_CS)
/* DMA_EN, RDSTS_EN, GYRO_EN: the modes with no model */
#define CTRL_MODE_START ((1u << 2) | (1u << 4) | (1u << 6))
#define CONFIG_PKTLEN   (1u << 2)
#define STS_TDR_EMPTY   (1u << 0)
#define STS_RDR_FULL    (1u << 1)
#define STS_W1C         0x3FC
#define STS_MASK        0x3FF

struct NvtSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq cs;
    SSIBus *bus;
    uint32_t regs[SPI_REGS_SIZE / 4];
};

static uint32_t *reg(NvtSpiState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static uint32_t nvt_spi_status(NvtSpiState *s)
{
    return *reg(s, SPI_STS) | STS_TDR_EMPTY;
}

static void nvt_spi_update(NvtSpiState *s)
{
    qemu_set_irq(s->irq, nvt_spi_status(s) & *reg(s, SPI_INTEN) & STS_MASK);
    qemu_set_irq(s->cs, !!(*reg(s, SPI_CTRL) & CTRL_CS));
}

/* One TDR write: PKT_CNT packets out, as many back into RDR. */
static void nvt_spi_shift(NvtSpiState *s, uint32_t tdr)
{
    uint32_t cfg = *reg(s, SPI_CONFIG);
    int bits = cfg & CONFIG_PKTLEN ? 16 : 8;
    /* TDR holds 32 bits: four one-byte packets, two two-byte ones */
    int n = MIN(extract32(cfg, 4, 2) + 1, 32 / bits), i;
    uint32_t rdr = 0;

    for (i = 0; i < n; i++) {
        rdr = deposit32(rdr, i * bits, bits,
                        ssi_transfer(s->bus, extract32(tdr, i * bits, bits)));
    }
    *reg(s, SPI_RDR) = rdr;
    *reg(s, SPI_STS) |= STS_RDR_FULL;
}

static uint64_t nvt_spi_read(void *opaque, hwaddr off, unsigned size)
{
    NvtSpiState *s = opaque;

    switch (off) {
    case SPI_STS:
        return nvt_spi_status(s);
    case SPI_RDR:
        *reg(s, SPI_STS) &= ~STS_RDR_FULL;
        nvt_spi_update(s);
        break;
    }
    return *reg(s, off);
}

static void nvt_spi_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    NvtSpiState *s = opaque;

    switch (off) {
    case SPI_CTRL:
        if (val & CTRL_MODE_START) {
            qemu_log_mask(LOG_UNIMP, "nvt-spi: DMA / flash-status / gyro "
                          "mode (CTRL 0x%08x)\n", (uint32_t)val);
        }
        *reg(s, off) = val & CTRL_KEPT;
        break;
    case SPI_STS:
        *reg(s, off) &= ~(val & STS_W1C);
        break;
    case SPI_RDR:
        return;
    case SPI_TDR:
        *reg(s, off) = val;
        nvt_spi_shift(s, val);
        break;
    default:
        *reg(s, off) = val;
        break;
    }
    nvt_spi_update(s);
}

static const MemoryRegionOps nvt_spi_ops = {
    .read = nvt_spi_read,
    .write = nvt_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_spi_reset(DeviceState *dev)
{
    NvtSpiState *s = NVT_SPI(dev);

    memset(s->regs, 0, sizeof(s->regs));
    *reg(s, SPI_CTRL) = CTRL_CS;
    nvt_spi_update(s);
}

static void nvt_spi_init(Object *obj)
{
    NvtSpiState *s = NVT_SPI(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_spi_ops, s, TYPE_NVT_SPI,
                          SPI_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->cs, "cs", 1);
    s->bus = ssi_create_bus(DEVICE(obj), "spi");
}

static const VMStateDescription vmstate_nvt_spi = {
    .name = TYPE_NVT_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtSpiState, SPI_REGS_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_spi_reset);
    dc->vmsd = &vmstate_nvt_spi;
}

static const TypeInfo nvt_spi_info = {
    .name = TYPE_NVT_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtSpiState),
    .instance_init = nvt_spi_init,
    .class_init = nvt_spi_class_init,
};

static void nvt_spi_register_types(void)
{
    type_register_static(&nvt_spi_info);
}

type_init(nvt_spi_register_types)
