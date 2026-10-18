/*
 * Anyka AK39xx UART.
 *
 * Not a 16550: the UART has no data register.  Its TX and RX buffers are
 * fixed 128-byte areas of L2 memory (UART n at 0x48001000 + 0x100 n and
 * + 0x80, hw/misc/ak-l2.c), and the registers only start and account for
 * transfers (kernel drivers/tty/serial/ak_uart.[ch], U-Boot serial_ak39.c):
 *
 *   0x00 CONF1  15:0 baud divider, 21 enable; 28 / 29 clear the TX / RX
 *               state (29 also restarts the RX ring at 0), 30 / 31 the
 *               RX / TX address
 *   0x04 CONF2  status and enables: 0 TX empty, 1 RX buffer full, 2 RX
 *               timeout, 3 RX error, 15:4 bytes to send, 16 start sending
 *               them, 19 TX done; enables 21 RX full, 22 timeout, 23 error,
 *               24 TX empty, 27 TX done, 28 RX threshold, 29 TX threshold;
 *               30 / 31 RX / TX threshold reached.  1, 2, 3, 30 and 31 are
 *               write-1-to-clear
 *   0x08 DATA   19:13 where the next received byte goes in the RX ring,
 *               12:0 bytes still to send
 *   0x0C THRESH 4:0 RX threshold, 5 clear the RX count, 31 receive on
 *   0x18 STOP / TIMEOUT    stored
 *
 * TX done (19) is a live "transmitter idle" flag, not a latched event:
 * U-Boot clears the TX state, starts a byte and waits for 19; the 3.4
 * kernel's start_tx() only enables the TX-done interrupt and expects it to
 * fire straight away when idle -- also after set_termios() has cleared the
 * TX state, which would otherwise stall the tty for good.  A send completes
 * at once here, so the transmitter is always idle.
 *
 * Received bytes go into the ring, with the timeout status raised (the
 * drivers' signal to drain it) and the threshold one once enough have
 * come.  U-Boot's tstc() looks only at the
 * threshold status and nothing ever clears it there, so a read of DATA --
 * which is how both U-Boot and the kernel find the new bytes -- clears it
 * too; the kernel acknowledges it explicitly anyway.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "chardev/char-fe.h"
#include "hw/irq.h"
#include "hw/misc/ak-l2.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_UART "ak-uart"
OBJECT_DECLARE_SIMPLE_TYPE(AkUartState, AK_UART)

#define R_CONF1         0x00
#define R_CONF2         0x04
#define R_DATA          0x08
#define R_THRESH        0x0C
#define R_RXBUF         0x10
#define R_STOP          0x18

#define CONF1_RX_CLR    (1u << 29)
#define CONF1_RXADDR_CLR (1u << 30)
#define CONF1_CMDS      0xF0000000u

#define CONF2_TX_EMPTY  (1u << 0)
#define CONF2_RX_FULL   (1u << 1)
#define CONF2_TIMEOUT   (1u << 2)
#define CONF2_RX_ERR    (1u << 3)
#define CONF2_TX_CNT(v) (((v) >> 4) & 0xFFF)
#define CONF2_TX_GO     (1u << 16)
#define CONF2_TX_END    (1u << 19)
#define CONF2_EN_RXFULL (1u << 21)
#define CONF2_EN_TMO    (1u << 22)
#define CONF2_EN_ERR    (1u << 23)
#define CONF2_EN_TXEMP  (1u << 24)
#define CONF2_EN_TXEND  (1u << 27)
#define CONF2_EN_RXTH   (1u << 28)
#define CONF2_EN_TXTH   (1u << 29)
#define CONF2_RX_TH     (1u << 30)
#define CONF2_TX_TH     (1u << 31)
#define CONF2_W1C       (CONF2_RX_FULL | CONF2_TIMEOUT | CONF2_RX_ERR | \
                         CONF2_RX_TH | CONF2_TX_TH)
#define CONF2_STATUS    (CONF2_W1C | CONF2_TX_EMPTY | CONF2_TX_END)

#define THRESH_RX_CLR   (1u << 5)
#define THRESH_RX_ON    (1u << 31)

#define AK_UART_TX_SIZE 128

struct AkUartState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq;
    AkL2State *l2;

    uint32_t conf1;
    uint32_t conf2;
    uint32_t thresh;
    uint32_t stop;
    uint32_t rx_pos;        /* producer index in the RX ring */
    uint32_t rx_count;      /* bytes since the RX count was cleared */
    uint32_t index;
};

static void ak_uart_update_irq(AkUartState *s)
{
    uint32_t c = s->conf2;
    bool level = ((c & CONF2_RX_TH) && (c & CONF2_EN_RXTH)) ||
                 ((c & CONF2_TX_END) && (c & CONF2_EN_TXEND)) ||
                 ((c & CONF2_TIMEOUT) && (c & CONF2_EN_TMO)) ||
                 ((c & CONF2_RX_ERR) && (c & CONF2_EN_ERR)) ||
                 ((c & CONF2_RX_FULL) && (c & CONF2_EN_RXFULL)) ||
                 ((c & CONF2_TX_EMPTY) && (c & CONF2_EN_TXEMP)) ||
                 ((c & CONF2_TX_TH) && (c & CONF2_EN_TXTH));

    qemu_set_irq(s->irq, level);
}

static void ak_uart_send(AkUartState *s, unsigned count)
{
    const uint8_t *tx = ak_l2_mem(s->l2) + AK_L2_UART_TX(s->index);
    uint8_t buf[AK_UART_TX_SIZE];
    unsigned i;

    for (i = 0; i < count; i++) {
        buf[i % AK_UART_TX_SIZE] = tx[i % AK_UART_TX_SIZE];
        if (i % AK_UART_TX_SIZE == AK_UART_TX_SIZE - 1 || i == count - 1) {
            qemu_chr_fe_write_all(&s->chr, buf, i % AK_UART_TX_SIZE + 1);
        }
    }
}

static int ak_uart_can_receive(void *opaque)
{
    AkUartState *s = opaque;

    return (s->thresh & THRESH_RX_ON) ? AK_L2_UART_RX_SIZE / 4 : 0;
}

static void ak_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    AkUartState *s = opaque;
    uint8_t *rx = ak_l2_mem(s->l2) + AK_L2_UART_RX(s->index);
    uint32_t th = MAX(s->thresh & 0x1F, 1);
    int i;

    for (i = 0; i < size; i++) {
        rx[s->rx_pos] = buf[i];
        s->rx_pos = (s->rx_pos + 1) % AK_L2_UART_RX_SIZE;
    }
    s->rx_count += size;
    s->conf2 |= CONF2_TIMEOUT;
    if (s->rx_count >= th) {
        s->conf2 |= CONF2_RX_TH;
    }
    ak_uart_update_irq(s);
}

static uint64_t ak_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    AkUartState *s = opaque;

    switch (offset) {
    case R_CONF1:
        return s->conf1;
    case R_CONF2:
        return s->conf2;
    case R_DATA:
        if (s->conf2 & CONF2_RX_TH) {
            s->conf2 &= ~CONF2_RX_TH;
            ak_uart_update_irq(s);
        }
        return s->rx_pos << 13;
    case R_THRESH:
        return (s->thresh & ~(0xFFFu << 12)) |
               (MIN(s->rx_count, 0xFFF) << 12);
    case R_STOP:
        return s->stop;
    case R_RXBUF:
        return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "ak-uart: read of 0x%02" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void ak_uart_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    AkUartState *s = opaque;

    switch (offset) {
    case R_CONF1:
        s->conf1 = val & ~CONF1_CMDS;
        if (val & (CONF1_RX_CLR | CONF1_RXADDR_CLR)) {
            s->rx_pos = 0;
            s->rx_count = 0;
            s->conf2 &= ~(CONF2_TIMEOUT | CONF2_RX_TH | CONF2_RX_FULL);
        }
        break;
    case R_CONF2:
        s->conf2 = (s->conf2 & CONF2_STATUS & ~(val & CONF2_W1C)) |
                   (val & ~(CONF2_STATUS | CONF2_TX_GO));
        if (val & CONF2_TX_GO) {
            ak_uart_send(s, CONF2_TX_CNT(val));
        }
        break;
    case R_THRESH:
        s->thresh = val & ~THRESH_RX_CLR;
        if (val & THRESH_RX_CLR) {
            s->rx_count = 0;
        }
        if (val & THRESH_RX_ON) {
            qemu_chr_fe_accept_input(&s->chr);
        }
        break;
    case R_STOP:
        s->stop = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ak-uart: write 0x%08" PRIx64 " to 0x%02"
                      HWADDR_PRIx "\n", val, offset);
        return;
    }
    ak_uart_update_irq(s);
}

static const MemoryRegionOps ak_uart_ops = {
    .read = ak_uart_read,
    .write = ak_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_uart_reset(DeviceState *dev)
{
    AkUartState *s = AK_UART(dev);

    s->conf1 = 0;
    s->conf2 = CONF2_TX_EMPTY | CONF2_TX_END;
    s->thresh = 0;
    s->stop = 0;
    s->rx_pos = 0;
    s->rx_count = 0;
    ak_uart_update_irq(s);
}

static void ak_uart_realize(DeviceState *dev, Error **errp)
{
    AkUartState *s = AK_UART(dev);

    if (!s->l2) {
        error_setg(errp, "ak-uart: the 'l2' link is required");
        return;
    }
    qemu_chr_fe_set_handlers(&s->chr, ak_uart_can_receive, ak_uart_receive,
                             NULL, NULL, s, NULL, true);
}

static void ak_uart_init(Object *obj)
{
    AkUartState *s = AK_UART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ak_uart_ops, s, TYPE_AK_UART,
                          0x8000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property ak_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", AkUartState, chr),
    DEFINE_PROP_LINK("l2", AkUartState, l2, TYPE_AK_L2, AkL2State *),
    DEFINE_PROP_UINT32("index", AkUartState, index, 0),
};

static int ak_uart_post_load(void *opaque, int version_id)
{
    ak_uart_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_ak_uart = {
    .name = TYPE_AK_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ak_uart_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf1, AkUartState),
        VMSTATE_UINT32(conf2, AkUartState),
        VMSTATE_UINT32(thresh, AkUartState),
        VMSTATE_UINT32(stop, AkUartState),
        VMSTATE_UINT32(rx_pos, AkUartState),
        VMSTATE_UINT32(rx_count, AkUartState),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ak_uart_realize;
    dc->vmsd = &vmstate_ak_uart;
    device_class_set_legacy_reset(dc, ak_uart_reset);
    device_class_set_props(dc, ak_uart_properties);
}

static const TypeInfo ak_uart_info = {
    .name          = TYPE_AK_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkUartState),
    .instance_init = ak_uart_init,
    .class_init    = ak_uart_class_init,
};

static void ak_uart_register_types(void)
{
    type_register_static(&ak_uart_info);
}

type_init(ak_uart_register_types)
