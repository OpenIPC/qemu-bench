/*
 * Anyka AK39xx SPI master.
 *
 * Registers (U-Boot drivers/spi/ak39_spi.h, kernel drivers/spi/spi-anyka.c):
 *
 *   0x00 CON    6 enable, 5 chip select asserted, 4 master, 3 CPHA, 2 CPOL,
 *               1 ARRM (TX only: received bits dropped), 0 TGDM (RX only:
 *               the master clocks out filler), 17:16 data lines, 15:8 divider
 *   0x04 STA    8 transfer done, 6 RX FIFO holds a word, 5 RX FIFO full,
 *               4 RX FIFO empty, 2 TX FIFO half empty, 0 TX FIFO empty
 *   0x08 INT    interrupt enables, the STA layout
 *   0x0C CNT    bytes to clock; writing it starts the transfer, it reads
 *               what is left
 *   0x10 EXTX / 0x14 EXRX  bit 0 (+ bit 16 for DMA): TX / RX data go
 *               through the L2 buffer the driver assigned to this
 *               controller, not the FIFOs
 *   0x18 OUT / 0x1C IN     the 8-byte TX / RX FIFOs, a word at a time
 *
 * Chip select is CON bit 5, or a GPIO for kernels that drive it that way
 * (the EV330's, cs-gpios): the "cs-gpio" input, active low, is ORed in.
 *
 * The master clocks a byte whenever it has one to send (or TGDM) and room
 * for the one it receives (or ARRM): the drivers' PIO loops write OUT on
 * "half empty" and read IN on "holds a word", the L2 path waits on the
 * buffer's block counter (hw/misc/ak-l2.c).  The data line count changes
 * nothing for the attached flash model, which sees bytes.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/fifo8.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/misc/ak-l2.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_SPI "ak-spi"
OBJECT_DECLARE_SIMPLE_TYPE(AkSpiState, AK_SPI)

#define AK_SPI_FIFO     8

#define R_CON           0x00
#define R_STA           0x04
#define R_INT           0x08
#define R_CNT           0x0C
#define R_EXTX          0x10
#define R_EXRX          0x14
#define R_OUT           0x18
#define R_IN            0x1C

#define CON_CS          (1u << 5)
#define CON_ARRM        (1u << 1)
#define CON_TGDM        (1u << 0)

#define STA_TRANSF      (1u << 8)
#define STA_RXHFULL     (1u << 6)
#define STA_RXFULL      (1u << 5)
#define STA_RXEMP       (1u << 4)
#define STA_TXHEMP      (1u << 2)
#define STA_TXEMP       (1u << 0)

#define EX_BUFEN        (1u << 0)

struct AkSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *bus;
    qemu_irq irq;
    qemu_irq cs;
    AkL2State *l2;

    uint32_t con;
    uint32_t inten;
    uint32_t cnt;
    uint32_t extx;
    uint32_t exrx;
    bool done;
    bool cs_gpio;           /* level of the cs-gpio input (1 = idle) */

    Fifo8 txfifo;
    Fifo8 rxfifo;

    /* The L2 block being sent / filled. */
    uint8_t txblk[AK_L2_BLOCK];
    uint8_t txblk_len;
    uint8_t txblk_pos;
    uint8_t rxblk[AK_L2_BLOCK];
    uint8_t rxblk_len;

    /* BUFASSIGN numbers of this controller's buffers. */
    uint32_t l2_rx_dev;
    uint32_t l2_tx_dev;
};

static uint32_t ak_spi_status(AkSpiState *s)
{
    uint32_t st = 0;
    uint32_t rx = fifo8_num_used(&s->rxfifo);
    uint32_t tx = fifo8_num_used(&s->txfifo);

    if (s->done) {
        st |= STA_TRANSF;
    }
    /* A last, short word counts as a word once nothing more will come. */
    if (rx >= 4 || (rx && !s->cnt)) {
        st |= STA_RXHFULL;
    }
    if (rx >= AK_SPI_FIFO) {
        st |= STA_RXFULL;
    }
    if (!rx) {
        st |= STA_RXEMP;
    }
    if (tx <= AK_SPI_FIFO / 2) {
        st |= STA_TXHEMP;
    }
    if (!tx) {
        st |= STA_TXEMP;
    }
    return st;
}

static void ak_spi_update_irq(AkSpiState *s)
{
    qemu_set_irq(s->irq, !!(ak_spi_status(s) & s->inten));
}

static void ak_spi_update_cs(AkSpiState *s)
{
    qemu_set_irq(s->cs, !(s->con & CON_CS) && s->cs_gpio);
}

static void ak_spi_set_cs_gpio(void *opaque, int n, int level)
{
    AkSpiState *s = opaque;

    s->cs_gpio = level;
    ak_spi_update_cs(s);
}

static bool ak_spi_tx_l2(AkSpiState *s)
{
    return s->l2 && (s->extx & EX_BUFEN);
}

static bool ak_spi_rx_l2(AkSpiState *s)
{
    return s->l2 && (s->exrx & EX_BUFEN);
}

/* Hand a completed (or final) RX block to L2; false if it has no room. */
static bool ak_spi_flush_rxblk(AkSpiState *s)
{
    int buf;

    if (!s->rxblk_len) {
        return true;
    }
    buf = ak_l2_dev_buf(s->l2, s->l2_rx_dev);
    if (buf < 0 || !ak_l2_push(s->l2, buf, s->rxblk, s->rxblk_len)) {
        return false;
    }
    s->rxblk_len = 0;
    return true;
}

/* Clock as many bytes as the FIFOs / buffers allow.  True on progress. */
static bool ak_spi_run(AkSpiState *s)
{
    bool tx_only = s->con & CON_ARRM;
    bool rx_only = s->con & CON_TGDM;
    bool progress = false;

    while (s->cnt) {
        uint8_t out = 0xFF, in;

        /*
         * Room for what comes back first: nothing is taken off the TX
         * side for a byte that cannot be clocked.
         */
        if (ak_spi_rx_l2(s) && !tx_only && s->rxblk_len == AK_L2_BLOCK &&
            !ak_spi_flush_rxblk(s)) {
            break;
        }
        if (!tx_only && !ak_spi_rx_l2(s) && fifo8_is_full(&s->rxfifo)) {
            break;
        }
        if (!rx_only) {
            if (ak_spi_tx_l2(s)) {
                if (s->txblk_pos == s->txblk_len) {
                    int buf = ak_l2_dev_buf(s->l2, s->l2_tx_dev);

                    if (buf < 0 ||
                        !ak_l2_pop(s->l2, buf, s->txblk, AK_L2_BLOCK)) {
                        break;
                    }
                    s->txblk_len = MIN(s->cnt, AK_L2_BLOCK);
                    s->txblk_pos = 0;
                }
                out = s->txblk[s->txblk_pos++];
            } else {
                if (fifo8_is_empty(&s->txfifo)) {
                    break;
                }
                out = fifo8_pop(&s->txfifo);
            }
        }
        in = ssi_transfer(s->bus, out);
        s->cnt--;
        progress = true;

        if (!tx_only) {
            if (ak_spi_rx_l2(s)) {
                s->rxblk[s->rxblk_len++] = in;
            } else {
                fifo8_push(&s->rxfifo, in);
            }
        }
    }
    if (!s->cnt && ak_spi_rx_l2(s) && !tx_only) {
        progress |= s->rxblk_len && ak_spi_flush_rxblk(s);
    }
    if (!s->cnt && !s->done && !s->rxblk_len) {
        s->done = true;
        /* What the last OUT word carried past the count is not sent. */
        fifo8_reset(&s->txfifo);
        s->txblk_len = s->txblk_pos = 0;
        progress = true;
    }
    ak_spi_update_irq(s);
    return progress;
}

static bool ak_spi_l2_pump(void *opaque)
{
    AkSpiState *s = opaque;

    if (!s->cnt && !s->rxblk_len) {
        return false;
    }
    return ak_spi_run(s);
}

static void ak_spi_kick(AkSpiState *s)
{
    if (s->l2) {
        ak_l2_kick(s->l2);      /* calls ak_spi_l2_pump() */
    } else {
        ak_spi_run(s);
    }
}

static uint64_t ak_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    AkSpiState *s = opaque;
    uint32_t v = 0;
    int i;

    switch (offset) {
    case R_CON:
        return s->con;
    case R_STA:
        return ak_spi_status(s);
    case R_INT:
        return s->inten;
    case R_CNT:
        return s->cnt;
    case R_EXTX:
        return s->extx;
    case R_EXRX:
        return s->exrx;
    case R_IN:
        for (i = 0; i < 4 && !fifo8_is_empty(&s->rxfifo); i++) {
            v |= (uint32_t)fifo8_pop(&s->rxfifo) << (8 * i);
        }
        ak_spi_kick(s);
        return v;
    default:
        qemu_log_mask(LOG_UNIMP, "ak-spi: read of 0x%02" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void ak_spi_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    AkSpiState *s = opaque;
    int i;

    switch (offset) {
    case R_CON:
        s->con = val;
        ak_spi_update_cs(s);
        break;
    case R_INT:
        s->inten = val;
        break;
    case R_CNT:
        s->cnt = val & 0xFFFF;
        s->done = !s->cnt;
        fifo8_reset(&s->txfifo);
        fifo8_reset(&s->rxfifo);
        s->txblk_len = s->txblk_pos = 0;
        s->rxblk_len = 0;
        ak_spi_kick(s);
        return;
    case R_EXTX:
        s->extx = val;
        break;
    case R_EXRX:
        s->exrx = val;
        break;
    case R_OUT:
        for (i = 0; i < 4 && !fifo8_is_full(&s->txfifo); i++) {
            fifo8_push(&s->txfifo, val >> (8 * i));
        }
        ak_spi_kick(s);
        return;
    default:
        qemu_log_mask(LOG_UNIMP, "ak-spi: write 0x%08" PRIx64 " to 0x%02"
                      HWADDR_PRIx "\n", val, offset);
        return;
    }
    ak_spi_update_irq(s);
}

static const MemoryRegionOps ak_spi_ops = {
    .read = ak_spi_read,
    .write = ak_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_spi_reset(DeviceState *dev)
{
    AkSpiState *s = AK_SPI(dev);

    s->con = 0;
    s->inten = 0;
    s->cnt = 0;
    s->extx = 0;
    s->exrx = 0;
    s->done = false;
    fifo8_reset(&s->txfifo);
    fifo8_reset(&s->rxfifo);
    s->txblk_len = s->txblk_pos = 0;
    s->rxblk_len = 0;
    ak_spi_update_cs(s);
    ak_spi_update_irq(s);
}

static void ak_spi_realize(DeviceState *dev, Error **errp)
{
    AkSpiState *s = AK_SPI(dev);

    fifo8_create(&s->txfifo, AK_SPI_FIFO);
    fifo8_create(&s->rxfifo, AK_SPI_FIFO);
    if (s->l2) {
        ak_l2_register_client(s->l2, ak_spi_l2_pump, s);
    }
}

static void ak_spi_init(Object *obj)
{
    AkSpiState *s = AK_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ak_spi_ops, s, TYPE_AK_SPI,
                          0x8000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = ssi_create_bus(DEVICE(obj), "ssi");
    qdev_init_gpio_out_named(DEVICE(obj), &s->cs, SSI_GPIO_CS, 1);
    qdev_init_gpio_in_named(DEVICE(obj), ak_spi_set_cs_gpio, "cs-gpio", 1);
    s->cs_gpio = true;
}

static const Property ak_spi_properties[] = {
    DEFINE_PROP_LINK("l2", AkSpiState, l2, TYPE_AK_L2, AkL2State *),
    DEFINE_PROP_UINT32("l2-rx-dev", AkSpiState, l2_rx_dev,
                       AK_L2_DEV_SPI0_RX),
    DEFINE_PROP_UINT32("l2-tx-dev", AkSpiState, l2_tx_dev,
                       AK_L2_DEV_SPI0_TX),
};

static int ak_spi_post_load(void *opaque, int version_id)
{
    AkSpiState *s = opaque;

    ak_spi_update_cs(s);
    ak_spi_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_ak_spi = {
    .name = TYPE_AK_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ak_spi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(con, AkSpiState),
        VMSTATE_UINT32(inten, AkSpiState),
        VMSTATE_UINT32(cnt, AkSpiState),
        VMSTATE_UINT32(extx, AkSpiState),
        VMSTATE_UINT32(exrx, AkSpiState),
        VMSTATE_BOOL(done, AkSpiState),
        VMSTATE_BOOL(cs_gpio, AkSpiState),
        VMSTATE_FIFO8(txfifo, AkSpiState),
        VMSTATE_FIFO8(rxfifo, AkSpiState),
        VMSTATE_UINT8_ARRAY(txblk, AkSpiState, AK_L2_BLOCK),
        VMSTATE_UINT8(txblk_len, AkSpiState),
        VMSTATE_UINT8(txblk_pos, AkSpiState),
        VMSTATE_UINT8_ARRAY(rxblk, AkSpiState, AK_L2_BLOCK),
        VMSTATE_UINT8(rxblk_len, AkSpiState),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ak_spi_realize;
    dc->vmsd = &vmstate_ak_spi;
    device_class_set_legacy_reset(dc, ak_spi_reset);
    device_class_set_props(dc, ak_spi_properties);
}

static const TypeInfo ak_spi_info = {
    .name          = TYPE_AK_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkSpiState),
    .instance_init = ak_spi_init,
    .class_init    = ak_spi_class_init,
};

static void ak_spi_register_types(void)
{
    type_register_static(&ak_spi_info);
}

type_init(ak_spi_register_types)
