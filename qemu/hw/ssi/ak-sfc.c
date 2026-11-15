/*
 * Anyka AK3918AV130 SPI flash controller ("SFC v2", sfc@20120000).
 *
 * Modelled on what the AV130's U-Boot 2019.10.0-V4.0.x driver does with it
 * (its flash, owner report r-5uwdebux on openipc.org; no datasheet).  The
 * boot NOR hangs off its SSI bus (hw/ssi/ak-spinor.c), and the controller
 * runs a list of command banks against it:
 *
 *   0x0C0  FIFO thresholds: 7:0 TX, 15:8 RX (words)
 *   0x0D0  status: 2 TX FIFO has room, 3 RX FIFO at its threshold, 4 done
 *          (latched, write 1 to clear), 5 idle
 *   0x0D4  interrupt enable, the status bits
 *   0x100  work mode: 0 command-mode request, 1 its acknowledgement (read
 *          only), 2 XIP enable, 4 FIFO reset (self-clearing), 6 data
 *          lengths count 2-byte units
 *   0x104, 0x108 (clock divider 9:0), 0x11C   stored
 *   0x10C  dummy cycles - 1, for whichever bank has a dummy phase
 *   0x180  transfer: 3:0 last bank, 29 DMA towards the flash, 30 DMA
 *          (through L2 common buffer, BUFASSIGN device 0), 31 start
 *   0x184  data FIFO, little-endian, for transfers without DMA
 *   0x190  DMA byte count (stored)
 *   0x200 + 0x40 n  command bank n:
 *          +0x00 opcode  +0x04 address  +0x08 mode byte(s)
 *          +0x0C data, up to 4 bytes, when interface bit 28 is set
 *          +0x18 match mask (1 = ignore)  +0x1C match data
 *          +0x20 interface: 1:0 / 3:2 / 5:4 lanes (opcode / address, mode
 *                and dummy / data, 0 = x1, 1 = x2, 2 = x4); phases 6
 *                opcode, 7 address, 8 mode, 9 dummy, 10 data; 14 data
 *                towards the flash; 27 repeat until the data matches;
 *                28 data through +0x0C
 *          +0x24 lengths: 3:2 address bytes - 1, 5:4 mode bytes - 1,
 *                31:6 data bytes - 1
 *          +0x2C poll interval (stored)
 *
 * Each bank is one chip-select frame.  The NOR answers byte by byte, so
 * lanes only scale the dummy phase (cycles x lanes / 8 bytes); the data a
 * bank moves is the same on one wire or four.  A status poll (bit 27)
 * reads one byte per try; ak-spinor is never busy, so the first try
 * matches.  The driver starts a FIFO write before it fills the FIFO, and
 * reads only the done bit after a DMA read, never L2: a bank writing from
 * the FIFO waits for all its bytes, and a DMA read is done only once L2
 * has put the last block in DDR.  The XIP window at 0x30000000 reads the
 * flash with 0x03; this U-Boot never uses it.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/misc/ak-l2.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_SFC "ak-sfc"
OBJECT_DECLARE_SIMPLE_TYPE(AkSfcState, AK_SFC)

#define AK_SFC_REGS_SIZE    0x1000
#define AK_SFC_XIP_SIZE     (16 * MiB)
#define AK_SFC_BANKS        16
#define AK_SFC_FIFO         (64 * KiB)      /* what a FIFO transfer may hold */
#define AK_SFC_POLL_TRIES   10000

#define R_THRESH    (0x0C0 / 4)
#define R_STATUS    (0x0D0 / 4)
#define R_INTEN     (0x0D4 / 4)
#define R_MODE      (0x100 / 4)
#define R_DUMMY     (0x10C / 4)
#define R_XFER      (0x180 / 4)
#define R_FIFO      (0x184 / 4)
#define BANK(n)     ((0x200 + 0x40 * (n)) / 4)
#define B_INST      0
#define B_ADDR      1
#define B_MODEB     2
#define B_DATA      3
#define B_MASK      6
#define B_MATCH     7
#define B_IFACE     8
#define B_LEN       9

#define ST_TXROOM   (1u << 2)
#define ST_RXTH     (1u << 3)
#define ST_DONE     (1u << 4)
#define ST_IDLE     (1u << 5)

#define MODE_REQ    (1u << 0)
#define MODE_ACK    (1u << 1)
#define MODE_FRESET (1u << 4)
#define MODE_DDRLEN (1u << 6)

#define XFER_TODEV  (1u << 29)
#define XFER_DMA    (1u << 30)
#define XFER_START  (1u << 31)

#define IF_INST     (1u << 6)
#define IF_ADDR     (1u << 7)
#define IF_MODE     (1u << 8)
#define IF_DUMMY    (1u << 9)
#define IF_DATA     (1u << 10)
#define IF_TX       (1u << 14)
#define IF_POLL     (1u << 27)
#define IF_REGDATA  (1u << 28)

#define AK_SFC_L2_DEV   0       /* BUFASSIGN device */

struct AkSfcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion xip;
    SSIBus *bus;
    qemu_irq cs;
    qemu_irq irq;
    AkL2State *l2;
    uint32_t regs[AK_SFC_REGS_SIZE / 4];

    /* The running transfer. */
    bool busy;
    bool dma;
    uint32_t bank;          /* bank being run */
    uint32_t last;
    bool in_frame;          /* chip select held, header sent */
    uint32_t data_left;     /* bytes the bank's data phase still moves */
    uint32_t data_done;
    uint8_t blk[AK_L2_BLOCK];
    uint32_t blk_len, blk_pos;

    /* FIFO transfers: what the CPU wrote, what it has to read. */
    uint8_t *fifo;
    uint32_t tx_len, tx_pos;
    uint32_t rx_len, rx_pos;
};

static void ak_sfc_update_irq(AkSfcState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R_STATUS] & s->regs[R_INTEN] &
                            (ST_DONE | ST_RXTH | ST_TXROOM)));
}

static uint32_t ak_sfc_status(AkSfcState *s)
{
    uint32_t rxth = 4 * MAX((s->regs[R_THRESH] >> 8) & 0xFF, 1);
    uint32_t v = s->regs[R_STATUS] & ST_DONE;

    v |= ST_TXROOM;
    if (s->rx_len - s->rx_pos >= rxth) {
        v |= ST_RXTH;
    }
    if (!s->busy) {
        v |= ST_IDLE;
    }
    return v;
}

static uint32_t ak_sfc_bank(AkSfcState *s, unsigned n, unsigned field)
{
    return s->regs[BANK(n) + field];
}

static uint32_t ak_sfc_data_len(AkSfcState *s, unsigned n)
{
    uint32_t len = (ak_sfc_bank(s, n, B_LEN) >> 6) + 1;

    return s->regs[R_MODE] & MODE_DDRLEN ? 2 * len : len;
}

static void ak_sfc_fifo_reset(AkSfcState *s)
{
    s->tx_len = s->tx_pos = 0;
    s->rx_len = s->rx_pos = 0;
}

/* Chip select, opcode, address, mode and dummy of bank n. */
static void ak_sfc_header(AkSfcState *s, unsigned n)
{
    uint32_t iface = ak_sfc_bank(s, n, B_IFACE);
    uint32_t len = ak_sfc_bank(s, n, B_LEN);
    uint32_t i, k;

    qemu_set_irq(s->cs, 0);
    if (iface & IF_INST) {
        ssi_transfer(s->bus, ak_sfc_bank(s, n, B_INST) & 0xFF);
    }
    if (iface & IF_ADDR) {
        uint32_t addr = ak_sfc_bank(s, n, B_ADDR);

        for (i = ((len >> 2) & 3) + 1; i; i--) {
            ssi_transfer(s->bus, (addr >> (8 * (i - 1))) & 0xFF);
        }
    }
    if (iface & IF_MODE) {
        uint32_t mode = ak_sfc_bank(s, n, B_MODEB);

        for (i = ((len >> 4) & 3) + 1; i; i--) {
            ssi_transfer(s->bus, (mode >> (8 * (i - 1))) & 0xFF);
        }
    }
    if (iface & IF_DUMMY) {
        k = ((s->regs[R_DUMMY] & 0xFF) + 1) << ((iface >> 2) & 3);
        for (i = DIV_ROUND_UP(k, 8); i; i--) {
            ssi_transfer(s->bus, 0xFF);
        }
    }
}

static void ak_sfc_end_frame(AkSfcState *s)
{
    qemu_set_irq(s->cs, 1);
    s->in_frame = false;
}

static void ak_sfc_finish(AkSfcState *s)
{
    s->busy = false;
    s->regs[R_XFER] &= ~(XFER_START | XFER_DMA);
    s->regs[R_STATUS] |= ST_DONE;
    ak_sfc_update_irq(s);
}

/*
 * The data phase of bank n, as far as the FIFO or L2 lets it go.  True
 * when it is complete.
 */
static bool ak_sfc_data(AkSfcState *s, unsigned n)
{
    uint32_t iface = ak_sfc_bank(s, n, B_IFACE);
    bool tx = iface & IF_TX;

    if (iface & IF_REGDATA) {
        uint32_t data = ak_sfc_bank(s, n, B_DATA), in = 0;
        uint32_t i;

        for (i = 0; i < s->data_left; i++) {
            uint8_t b = ssi_transfer(s->bus, i < 4 ? data >> (8 * i) : 0);

            if (i < 4) {
                in |= (uint32_t)b << (8 * i);
            }
        }
        if (!tx) {
            s->regs[BANK(n) + B_DATA] = in;
        }
        s->data_left = 0;
        return true;
    }
    if (s->dma && s->l2) {
        int buf = ak_l2_dev_buf(s->l2, AK_SFC_L2_DEV);

        if (buf < 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "ak-sfc: DMA without an L2 buffer assigned\n");
            return false;
        }
        while (s->data_left || s->blk_len) {
            if (tx) {
                if (s->blk_pos == s->blk_len) {
                    if (!ak_l2_pop(s->l2, buf, s->blk, AK_L2_BLOCK)) {
                        return false;
                    }
                    s->blk_len = MIN(s->data_left, AK_L2_BLOCK);
                    s->blk_pos = 0;
                }
                while (s->blk_pos < s->blk_len) {
                    ssi_transfer(s->bus, s->blk[s->blk_pos++]);
                    s->data_left--;
                }
                s->blk_len = s->blk_pos = 0;
            } else {
                if (!s->blk_len) {
                    s->blk_len = MIN(s->data_left, AK_L2_BLOCK);
                    for (s->blk_pos = 0; s->blk_pos < s->blk_len;
                         s->blk_pos++) {
                        s->blk[s->blk_pos] = ssi_transfer(s->bus, 0xFF);
                    }
                    s->data_left -= s->blk_len;
                }
                if (!ak_l2_push(s->l2, buf, s->blk, s->blk_len)) {
                    return false;
                }
                s->blk_len = s->blk_pos = 0;
            }
        }
        return true;
    }
    if (tx) {
        while (s->data_left) {
            if (s->tx_pos == s->tx_len) {
                return false;   /* the CPU has not written it yet */
            }
            ssi_transfer(s->bus, s->fifo[s->tx_pos++]);
            s->data_left--;
        }
        return true;
    }
    while (s->data_left) {
        uint8_t b = ssi_transfer(s->bus, 0xFF);

        if (s->rx_len < AK_SFC_FIFO) {
            s->fifo[s->rx_len++] = b;
        }
        s->data_left--;
    }
    return true;
}

/* A status poll: the bank again until the byte it reads matches. */
static void ak_sfc_poll(AkSfcState *s, unsigned n)
{
    uint32_t mask = ~ak_sfc_bank(s, n, B_MASK);
    uint32_t want = ak_sfc_bank(s, n, B_MATCH) & mask;
    uint32_t i, got = 0;

    for (i = 0; i < AK_SFC_POLL_TRIES; i++) {
        ak_sfc_header(s, n);
        got = ssi_transfer(s->bus, 0xFF) & 0xFF;
        qemu_set_irq(s->cs, 1);
        if ((got & mask) == want) {
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ak-sfc: bank %u poll never matched "
                  "(0x%02x, mask 0x%08x, want 0x%08x)\n", n, got, mask, want);
}

/* Run banks as far as the data lets; true on progress. */
static bool ak_sfc_run(AkSfcState *s)
{
    bool progress = false;

    while (s->busy && s->bank <= s->last) {
        unsigned n = s->bank;
        uint32_t iface = ak_sfc_bank(s, n, B_IFACE);

        if (iface & IF_POLL) {
            ak_sfc_poll(s, n);
            s->bank++;
            progress = true;
            continue;
        }
        if (!s->in_frame) {
            /* A FIFO write waits whole for its bytes, header and all. */
            if ((iface & (IF_DATA | IF_TX | IF_REGDATA)) == (IF_DATA | IF_TX)
                && !s->dma &&
                s->tx_len - s->tx_pos < ak_sfc_data_len(s, n)) {
                break;
            }
            ak_sfc_header(s, n);
            s->in_frame = true;
            s->data_left = (iface & IF_DATA) ? ak_sfc_data_len(s, n) : 0;
            s->blk_len = s->blk_pos = 0;
            progress = true;
        }
        if (s->data_left && !ak_sfc_data(s, n)) {
            break;
        }
        ak_sfc_end_frame(s);
        s->bank++;
        progress = true;
    }
    if (s->busy && s->bank > s->last) {
        /* A DMA read is over when L2 has moved the last block to DDR. */
        if (s->dma && s->l2) {
            int buf = ak_l2_dev_buf(s->l2, AK_SFC_L2_DEV);

            if (buf >= 0 && (s->l2->buf[buf].status ||
                             s->l2->buf[buf].dma_left)) {
                return progress;
            }
        }
        ak_sfc_finish(s);
        progress = true;
    }
    return progress;
}

static bool ak_sfc_l2_pump(void *opaque)
{
    AkSfcState *s = opaque;

    return s->busy && s->dma && ak_sfc_run(s);
}

static void ak_sfc_kick(AkSfcState *s)
{
    if (s->dma && s->l2) {
        ak_l2_kick(s->l2);          /* runs ak_sfc_l2_pump() */
    } else {
        ak_sfc_run(s);
    }
}

static void ak_sfc_start(AkSfcState *s)
{
    uint32_t x = s->regs[R_XFER];

    if (s->busy) {
        qemu_log_mask(LOG_GUEST_ERROR, "ak-sfc: start while busy\n");
        return;
    }
    s->busy = true;
    s->dma = x & XFER_DMA;
    s->bank = 0;
    s->last = MIN(x & 0xF, AK_SFC_BANKS - 1);
    s->in_frame = false;
    s->rx_len = s->rx_pos = 0;
    s->regs[R_STATUS] &= ~ST_DONE;
    qemu_log_mask(LOG_UNIMP, "ak-sfc: start, banks 0-%u, %s, bank 0 op 0x%02x"
                  " addr 0x%x len %u\n", s->last, s->dma ? "DMA" : "FIFO",
                  ak_sfc_bank(s, 0, B_INST) & 0xFF,
                  ak_sfc_bank(s, 0, B_ADDR), ak_sfc_data_len(s, 0));
    ak_sfc_kick(s);
}

static uint64_t ak_sfc_read(void *opaque, hwaddr offset, unsigned size)
{
    AkSfcState *s = opaque;
    uint32_t v = 0;
    unsigned i;

    switch (offset / 4) {
    case R_STATUS:
        return ak_sfc_status(s);
    case R_MODE:
        v = s->regs[R_MODE] & ~(MODE_ACK | MODE_FRESET);
        return v | ((v & MODE_REQ) ? MODE_ACK : 0);
    case R_FIFO:
        /* An empty FIFO reads harmlessly: the driver reads one too many. */
        for (i = 0; i < 4 && s->rx_pos < s->rx_len; i++) {
            v |= (uint32_t)s->fifo[s->rx_pos++] << (8 * i);
        }
        if (s->rx_pos == s->rx_len) {
            s->rx_pos = s->rx_len = 0;
        }
        ak_sfc_update_irq(s);
        return v;
    }
    return s->regs[offset / 4];
}

static void ak_sfc_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    AkSfcState *s = opaque;
    unsigned i;

    switch (offset / 4) {
    case R_STATUS:
        s->regs[R_STATUS] &= ~(val & ST_DONE);
        ak_sfc_update_irq(s);
        return;
    case R_MODE:
        if (val & MODE_FRESET) {
            ak_sfc_fifo_reset(s);
        }
        s->regs[R_MODE] = val & ~(MODE_ACK | MODE_FRESET);
        return;
    case R_XFER:
        s->regs[R_XFER] = val;
        if (val & XFER_START) {
            ak_sfc_start(s);
        }
        return;
    case R_FIFO:
        for (i = 0; i < 4; i++) {
            if (s->tx_len < AK_SFC_FIFO) {
                s->fifo[s->tx_len++] = val >> (8 * i);
            }
        }
        if (s->busy) {
            ak_sfc_kick(s);
        }
        return;
    case R_INTEN:
        s->regs[R_INTEN] = val;
        ak_sfc_update_irq(s);
        return;
    }
    s->regs[offset / 4] = val;
}

static const MemoryRegionOps ak_sfc_ops = {
    .read = ak_sfc_read,
    .write = ak_sfc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* A read of the XIP window: a 0x03 read of the same address on the bus. */
static uint64_t ak_sfc_xip_read(void *opaque, hwaddr addr, unsigned size)
{
    AkSfcState *s = opaque;
    uint64_t v = 0;
    unsigned i;

    qemu_set_irq(s->cs, 0);
    ssi_transfer(s->bus, 0x03);
    ssi_transfer(s->bus, (addr >> 16) & 0xFF);
    ssi_transfer(s->bus, (addr >> 8) & 0xFF);
    ssi_transfer(s->bus, addr & 0xFF);
    for (i = 0; i < size; i++) {
        v |= (uint64_t)(ssi_transfer(s->bus, 0xFF) & 0xFF) << (8 * i);
    }
    qemu_set_irq(s->cs, 1);
    return v;
}

static void ak_sfc_xip_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR, "ak-sfc: write to the XIP window at 0x%"
                  HWADDR_PRIx "\n", addr);
}

static const MemoryRegionOps ak_sfc_xip_ops = {
    .read = ak_sfc_xip_read,
    .write = ak_sfc_xip_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void ak_sfc_reset(DeviceState *dev)
{
    AkSfcState *s = AK_SFC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->busy = s->in_frame = false;
    ak_sfc_fifo_reset(s);
    qemu_set_irq(s->cs, 1);
    qemu_set_irq(s->irq, 0);
}

static void ak_sfc_realize(DeviceState *dev, Error **errp)
{
    AkSfcState *s = AK_SFC(dev);

    s->fifo = g_malloc0(AK_SFC_FIFO);
    if (s->l2) {
        ak_l2_register_client(s->l2, ak_sfc_l2_pump, s);
    }
}

static void ak_sfc_init(Object *obj)
{
    AkSfcState *s = AK_SFC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ak_sfc_ops, s, TYPE_AK_SFC,
                          AK_SFC_REGS_SIZE);
    memory_region_init_io(&s->xip, obj, &ak_sfc_xip_ops, s, "ak-sfc.xip",
                          AK_SFC_XIP_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->xip);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = ssi_create_bus(DEVICE(obj), "ssi");
    qdev_init_gpio_out_named(DEVICE(obj), &s->cs, SSI_GPIO_CS, 1);
}

static const Property ak_sfc_properties[] = {
    DEFINE_PROP_LINK("l2", AkSfcState, l2, TYPE_AK_L2, AkL2State *),
};

/*
 * Only an idle controller migrates: a transfer runs to completion within
 * one register write, or stalls on L2 or the FIFO, which the next write
 * resumes; the FIFO contents are not saved.
 */
static int ak_sfc_pre_save(void *opaque)
{
    AkSfcState *s = opaque;

    return s->busy ? -EBUSY : 0;
}

static const VMStateDescription vmstate_ak_sfc = {
    .name = TYPE_AK_SFC,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = ak_sfc_pre_save,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AkSfcState, AK_SFC_REGS_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_sfc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ak_sfc_realize;
    dc->vmsd = &vmstate_ak_sfc;
    device_class_set_legacy_reset(dc, ak_sfc_reset);
    device_class_set_props(dc, ak_sfc_properties);
}

static const TypeInfo ak_sfc_info = {
    .name          = TYPE_AK_SFC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkSfcState),
    .instance_init = ak_sfc_init,
    .class_init    = ak_sfc_class_init,
};

static void ak_sfc_register_types(void)
{
    type_register_static(&ak_sfc_info);
}

type_init(ak_sfc_register_types)
