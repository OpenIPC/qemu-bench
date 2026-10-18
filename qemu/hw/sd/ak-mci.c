/*
 * Anyka AK39xx MCI (SD / MMC host).
 *
 * A PL180-like controller with its own bit layout (U-Boot port
 * drivers/mmc/ak39_mci.c, vendor drivers/mmc/host/akmci):
 *
 *   0x04 CLOCK     divider 15:0, 16 enable, 20 controller enable
 *   0x08 ARGUMENT  0x0C COMMAND: 0 go, 6:1 index, 7 response, 8 long, 11
 *                  with data      0x14..0x20 RESPONSE0..3 (0 = bits 127:96)
 *   0x24 DATATIMER 0x28 DATALENGTH 0x2C DATACTRL: 0 go, 1 read
 *   0x34 STATUS    0 resp CRC, 1 data CRC, 2 resp timeout, 3 data timeout,
 *                  4 response end, 5 command sent, 6 data end, 10 / 11 TX /
 *                  RX active, 12 FIFO full (a word to read), 13 FIFO empty
 *                  (room to write); the event bits clear when read
 *   0x38 MASK      interrupt enables, the STATUS layout
 *   0x3C DMACTRL   L2 DMA set-up (stored)        0x40 FIFO
 *
 * Commands go to the card on the SD bus at once; without a card every
 * command with a response times out, which is how the vendor U-Boot and
 * kernel find an empty slot.  Data moves through the FIFO a word at a
 * time; the kernels' L2 DMA path is not modelled.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_MCI "ak-mci"
OBJECT_DECLARE_SIMPLE_TYPE(AkMciState, AK_MCI)

#define R_CLOCK         0x04
#define R_ARG           0x08
#define R_CMD           0x0C
#define R_RESP0         0x14
#define R_DATATIMER     0x24
#define R_DATALEN       0x28
#define R_DATACTRL      0x2C
#define R_RESPCMD       0x10
#define R_DATACNT       0x30
#define R_STATUS        0x34
#define R_MASK          0x38
#define R_DMACTRL       0x3C
#define R_FIFO          0x40

#define CMD_GO          (1u << 0)
#define CMD_INDEX(v)    (((v) >> 1) & 0x3F)
#define CMD_RESP        (1u << 7)
#define CMD_LONG        (1u << 8)

#define DCTRL_GO        (1u << 0)
#define DCTRL_READ      (1u << 1)

#define ST_RESPTIMEOUT  (1u << 2)
#define ST_RESPEND      (1u << 4)
#define ST_CMDSENT      (1u << 5)
#define ST_DATAEND      (1u << 6)
#define ST_TXACTIVE     (1u << 10)
#define ST_RXACTIVE     (1u << 11)
#define ST_FIFOFULL     (1u << 12)
#define ST_FIFOEMPTY    (1u << 13)

struct AkMciState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;

    uint32_t clock;
    uint32_t arg;
    uint32_t cmd;
    uint32_t resp[4];
    uint32_t datatimer;
    uint32_t datalen;
    uint32_t datactrl;
    uint32_t events;
    uint32_t mask;
    uint32_t dmactrl;
    uint32_t left;          /* data bytes still to move */
};

static uint32_t ak_mci_status(AkMciState *s);

static void ak_mci_update_irq(AkMciState *s)
{
    qemu_set_irq(s->irq, !!(ak_mci_status(s) & s->mask));
}

static void ak_mci_command(AkMciState *s)
{
    SDRequest req = {
        .cmd = CMD_INDEX(s->cmd),
        .arg = s->arg,
    };
    uint8_t r[16];
    int n;

    memset(s->resp, 0, sizeof(s->resp));
    n = sdbus_do_command(&s->sdbus, &req, r, sizeof(r));
    if (!(s->cmd & CMD_RESP)) {
        s->events |= ST_CMDSENT;
        return;
    }
    if (n <= 0 || ((s->cmd & CMD_LONG) ? n != 16 : n != 4)) {
        s->events |= ST_RESPTIMEOUT;
        return;
    }
    if (n == 16) {
        int i;

        for (i = 0; i < 4; i++) {
            s->resp[i] = ldl_be_p(r + 4 * i);
        }
    } else {
        s->resp[0] = ldl_be_p(r);
    }
    s->events |= ST_RESPEND;
}

static uint32_t ak_mci_status(AkMciState *s)
{
    uint32_t st = s->events;
    bool active = s->left && (s->datactrl & DCTRL_GO);

    if (active && (s->datactrl & DCTRL_READ)) {
        st |= ST_RXACTIVE | ST_FIFOFULL;
    } else if (active) {
        st |= ST_TXACTIVE | ST_FIFOEMPTY;
    } else {
        st |= ST_FIFOEMPTY;
    }
    return st;
}

static void ak_mci_data_step(AkMciState *s, unsigned n)
{
    s->left -= MIN(n, s->left);
    if (!s->left) {
        s->events |= ST_DATAEND;
    }
}

static uint64_t ak_mci_read(void *opaque, hwaddr offset, unsigned size)
{
    AkMciState *s = opaque;
    uint32_t v = 0;
    unsigned i;

    switch (offset) {
    case R_CLOCK:
        return s->clock;
    case R_ARG:
        return s->arg;
    case R_CMD:
        return s->cmd;
    case R_RESPCMD:
        return CMD_INDEX(s->cmd);
    case R_DATACNT:
        return s->left;
    case R_MASK:
        return s->mask;
    case R_DMACTRL:
        return s->dmactrl;
    case R_RESP0 ... R_RESP0 + 12:
        return s->resp[(offset - R_RESP0) / 4];
    case R_DATATIMER:
        return s->datatimer;
    case R_DATALEN:
        return s->datalen;
    case R_DATACTRL:
        return s->datactrl;
    case R_STATUS:
        v = ak_mci_status(s);
        s->events = 0;
        ak_mci_update_irq(s);
        return v;
    case R_FIFO:
        if (!(s->datactrl & DCTRL_READ) || !s->left) {
            return 0;
        }
        for (i = 0; i < 4 && i < s->left; i++) {
            v |= (uint32_t)sdbus_read_byte(&s->sdbus) << (8 * i);
        }
        ak_mci_data_step(s, 4);
        ak_mci_update_irq(s);
        return v;
    default:
        qemu_log_mask(LOG_UNIMP, "ak-mci: read of 0x%02" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void ak_mci_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    AkMciState *s = opaque;
    unsigned i;

    switch (offset) {
    case R_CLOCK:
        s->clock = val;
        break;
    case R_ARG:
        s->arg = val;
        break;
    case R_CMD:
        s->cmd = val;
        if (val & CMD_GO) {
            ak_mci_command(s);
        }
        break;
    case R_DATATIMER:
        s->datatimer = val;
        break;
    case R_MASK:
        s->mask = val;
        break;
    case R_DMACTRL:
        s->dmactrl = val;
        break;
    case R_DATALEN:
        s->datalen = val & 0xFFFF;
        break;
    case R_DATACTRL:
        s->datactrl = val;
        s->left = (val & DCTRL_GO) ? s->datalen : 0;
        break;
    case R_FIFO:
        if ((s->datactrl & DCTRL_READ) || !s->left) {
            break;
        }
        for (i = 0; i < 4 && i < s->left; i++) {
            sdbus_write_byte(&s->sdbus, val >> (8 * i));
        }
        ak_mci_data_step(s, 4);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ak-mci: write 0x%08" PRIx64 " to 0x%02"
                      HWADDR_PRIx "\n", val, offset);
        break;
    }
    ak_mci_update_irq(s);
}

static const MemoryRegionOps ak_mci_ops = {
    .read = ak_mci_read,
    .write = ak_mci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_mci_reset(DeviceState *dev)
{
    AkMciState *s = AK_MCI(dev);

    s->clock = s->arg = s->cmd = 0;
    memset(s->resp, 0, sizeof(s->resp));
    s->datatimer = s->datalen = s->datactrl = 0;
    s->events = 0;
    s->mask = 0;
    s->dmactrl = 0;
    s->left = 0;
    ak_mci_update_irq(s);
}

static void ak_mci_init(Object *obj)
{
    AkMciState *s = AK_MCI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ak_mci_ops, s, TYPE_AK_MCI,
                          0x8000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_SD_BUS, DEVICE(obj),
              "sd-bus");
}

static const VMStateDescription vmstate_ak_mci = {
    .name = TYPE_AK_MCI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(clock, AkMciState),
        VMSTATE_UINT32(arg, AkMciState),
        VMSTATE_UINT32(cmd, AkMciState),
        VMSTATE_UINT32_ARRAY(resp, AkMciState, 4),
        VMSTATE_UINT32(datatimer, AkMciState),
        VMSTATE_UINT32(datalen, AkMciState),
        VMSTATE_UINT32(datactrl, AkMciState),
        VMSTATE_UINT32(events, AkMciState),
        VMSTATE_UINT32(mask, AkMciState),
        VMSTATE_UINT32(dmactrl, AkMciState),
        VMSTATE_UINT32(left, AkMciState),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_mci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ak_mci;
    device_class_set_legacy_reset(dc, ak_mci_reset);
}

static const TypeInfo ak_mci_info = {
    .name          = TYPE_AK_MCI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkMciState),
    .instance_init = ak_mci_init,
    .class_init    = ak_mci_class_init,
};

static void ak_mci_register_types(void)
{
    type_register_static(&ak_mci_info);
}

type_init(ak_mci_register_types)
