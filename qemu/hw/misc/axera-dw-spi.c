/*
 * Synopsys DesignWare APB SSI (SPI) controller, as used for Axera's
 * axera-spi2 (SPI-attached display panel on the Sipeed NanoKVM-Pro/
 * MaixCam board).
 *
 * Grounded in the real vendor U-Boot driver
 * (maix_ax620e_sdk/boot/uboot/u-boot-2020.04/drivers/spi/
 * designware_spi.c): poll_transfer() is a bare "do { dw_writer();
 * dw_reader(); } while (rx_end > rx)" loop with NO timeout -- it only
 * terminates once DW_SPI_RXFLR (the RX FIFO level register) reports as
 * many bytes available as were written, since the driver's tmode is
 * always TR (transmit+receive) even when the caller only wants to
 * write. A plain storage regbank can never satisfy that (TXFLR/RXFLR
 * never change), so with no real panel attached the driver spins in
 * this loop forever with no way to time out, past console init, 100%
 * silent. This model implements just enough of the real IP's FIFO
 * semantics (loop every written byte straight back into a receive
 * queue) for that loop to terminate -- as real silicon would too, since
 * a DW SPI controller's own FIFO state updates independent of whether
 * anything is actually listening on the bus.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_DW_SPI "axera-dw-spi"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraDwSpiState, AXERA_DW_SPI)

#define DW_SPI_SIZE             0x100
#define DW_SPI_TXFLR            0x20
#define DW_SPI_RXFLR            0x24
#define DW_SPI_SR               0x28
#define DW_SPI_DR               0x60

#define SR_BUSY                 BIT(0)
#define SR_TF_NOT_FULL          BIT(1)
#define SR_TF_EMPT              BIT(2)
#define SR_RF_NOT_EMPT          BIT(3)
#define SR_RF_FULL              BIT(4)

#define DW_SPI_RX_QUEUE_MAX     4096

struct AxeraDwSpiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint8_t *mem;
    uint32_t rxq[DW_SPI_RX_QUEUE_MAX];
    unsigned int rxq_head;
    unsigned int rxq_len;
};

static uint64_t axera_dw_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraDwSpiState *s = opaque;
    uint64_t val;

    switch (offset) {
    case DW_SPI_TXFLR:
        return 0;
    case DW_SPI_RXFLR:
        return s->rxq_len;
    case DW_SPI_SR:
        return SR_TF_NOT_FULL | SR_TF_EMPT |
               (s->rxq_len ? SR_RF_NOT_EMPT : 0);
    case DW_SPI_DR:
        if (s->rxq_len == 0) {
            return 0;
        }
        val = s->rxq[s->rxq_head];
        s->rxq_head = (s->rxq_head + 1) % DW_SPI_RX_QUEUE_MAX;
        s->rxq_len--;
        return val;
    default:
        memcpy(&val, s->mem + offset, size);
        return val;
    }
}

static void axera_dw_spi_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    AxeraDwSpiState *s = opaque;

    switch (offset) {
    case DW_SPI_TXFLR:
    case DW_SPI_RXFLR:
    case DW_SPI_SR:
        /* Read-only hardware status; ignore writes like real silicon. */
        return;
    case DW_SPI_DR:
        if (s->rxq_len < DW_SPI_RX_QUEUE_MAX) {
            s->rxq[(s->rxq_head + s->rxq_len) % DW_SPI_RX_QUEUE_MAX] = value;
            s->rxq_len++;
        }
        return;
    default:
        memcpy(s->mem + offset, &value, size);
        return;
    }
}

static const MemoryRegionOps axera_dw_spi_ops = {
    .read = axera_dw_spi_read,
    .write = axera_dw_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_dw_spi_realize(DeviceState *dev, Error **errp)
{
    AxeraDwSpiState *s = AXERA_DW_SPI(dev);

    s->mem = g_malloc0(DW_SPI_SIZE);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_dw_spi_ops, s,
                          TYPE_AXERA_DW_SPI, DW_SPI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

/*
 * A system reset (e.g. the watchdog's) must not hand the next boot any
 * loopback bytes still queued from an interrupted transfer, nor stale
 * controller configuration: back to the all-zero power-on state.
 */
static void axera_dw_spi_reset(DeviceState *dev)
{
    AxeraDwSpiState *s = AXERA_DW_SPI(dev);

    memset(s->mem, 0, DW_SPI_SIZE);
    s->rxq_head = 0;
    s->rxq_len = 0;
}

static void axera_dw_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_dw_spi_realize;
    device_class_set_legacy_reset(dc, axera_dw_spi_reset);
}

static const TypeInfo axera_dw_spi_info = {
    .name = TYPE_AXERA_DW_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraDwSpiState),
    .class_init = axera_dw_spi_class_init,
};

static void axera_dw_spi_register_types(void)
{
    type_register_static(&axera_dw_spi_info);
}

type_init(axera_dw_spi_register_types)
