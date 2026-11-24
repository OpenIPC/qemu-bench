/*
 * Faraday FTSDC010 SD host of the Grain Media GM8126 (0x92800000).
 *
 *   0x00 command (bits 5:0 index, 6 response, 7 long, 9 go, 10 reset)
 *   0x04 argument   0x08-0x14 response   0x1C data control (3:0 block
 *   size log2, 4 write, 5 DMA, 6 enable)   0x24 data length   0x28 status
 *   0x2C clear   0x30 interrupt mask   0x3C bus width (bit 3: 4 bits can)
 *   0x40 data window   0x44 feature (the FIFO's depth in words)
 *
 * Data is moved by the CPU, 64 bytes each time status bit 9 (read) or 8
 * (write) says the FIFO wants it, or by the DMA controller, which "dma-req"
 * paces. Status bit 11 is a card in the slot: the polarity of a micro-SD
 * slot, what r-nz9qyrtg's firmware loads the vendor driver for. Bit 17,
 * DAT0 idle, always reads set.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_SDC010 "gm-sdc010"
OBJECT_DECLARE_SIMPLE_TYPE(GmSdc010State, GM_SDC010)

#define TYPE_GM_SDC010_BUS "gm-sdc010-bus"

#define SDC_SIZE    0x1000
#define SDC_NREGS   (0x100 / 4)
#define SDC_FIFO    64

#define R_CMD       0x00
#define R_ARG       0x04
#define R_RESP      0x08
#define R_RESPCMD   0x18
#define R_DCR       0x1C
#define R_DLEN      0x24
#define R_STATUS    0x28
#define R_CLEAR     0x2C
#define R_MASK      0x30
#define R_BUSW      0x3C
#define R_DATA      0x40
#define R_FEATURE   0x44

#define CMD_RSP     (1u << 6)
#define CMD_LONG    (1u << 7)
#define CMD_GO      (1u << 9)
#define CMD_RESET   (1u << 10)

#define DCR_WRITE   (1u << 4)
#define DCR_DMA     (1u << 5)
#define DCR_EN      (1u << 6)

#define ST_RSP_TIMEOUT  (1u << 2)
#define ST_DATA_TIMEOUT (1u << 3)
#define ST_RSP_OK       (1u << 4)
#define ST_DATA_OK      (1u << 5)
#define ST_CMD_SENT     (1u << 6)
#define ST_DATA_END     (1u << 7)
#define ST_URUN         (1u << 8)
#define ST_ORUN         (1u << 9)
#define ST_CHANGE       (1u << 10)
#define ST_CARD         (1u << 11)
#define ST_WP           (1u << 12)
#define ST_DAT0         (1u << 17)

struct GmSdc010State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;
    qemu_irq dma_req;
    uint32_t regs[SDC_NREGS];
    uint32_t left;              /* bytes of the data phase to come */
    uint32_t moved;
    bool running;               /* the command has started the data phase */
};

static void sdc_update(GmSdc010State *s)
{
    qemu_set_irq(s->irq,
                 (s->regs[R_STATUS / 4] & s->regs[R_MASK / 4]) != 0);
    qemu_set_irq(s->dma_req, s->running && s->left &&
                 (s->regs[R_DCR / 4] & DCR_DMA));
}

/* the FIFO has, or has room for, the next 64 bytes */
static void sdc_fifo(GmSdc010State *s)
{
    if (s->left && !(s->regs[R_DCR / 4] & DCR_DMA)) {
        s->regs[R_STATUS / 4] |=
            s->regs[R_DCR / 4] & DCR_WRITE ? ST_URUN : ST_ORUN;
    }
}

static void sdc_moved(GmSdc010State *s)
{
    s->moved += 4;
    s->left -= MIN(s->left, 4);
    if (!s->left) {
        s->running = false;
        s->regs[R_STATUS / 4] |= ST_DATA_OK | ST_DATA_END;
    } else if (s->moved % SDC_FIFO == 0) {
        sdc_fifo(s);
    }
    sdc_update(s);
}

static void sdc_command(GmSdc010State *s, uint32_t val)
{
    SDRequest req = { .cmd = val & 0x3F, .arg = s->regs[R_ARG / 4] };
    uint8_t resp[16];
    size_t n = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));
    int i;

    s->regs[R_RESPCMD / 4] = req.cmd;
    if (!(val & CMD_RSP)) {
        s->regs[R_STATUS / 4] |= ST_CMD_SENT;
    } else if (n == 16 && (val & CMD_LONG)) {
        for (i = 0; i < 4; i++) {
            s->regs[R_RESP / 4 + i] = ldl_be_p(resp + 12 - 4 * i);
        }
        s->regs[R_STATUS / 4] |= ST_RSP_OK;
    } else if (n == 4) {
        s->regs[R_RESP / 4] = ldl_be_p(resp);
        s->regs[R_STATUS / 4] |= ST_RSP_OK;
    } else {
        s->regs[R_STATUS / 4] |= ST_RSP_TIMEOUT;
    }

    if (s->regs[R_DCR / 4] & DCR_EN) {
        const bool wr = s->regs[R_DCR / 4] & DCR_WRITE;

        if (wr ? sdbus_receive_ready(&s->sdbus) : sdbus_data_ready(&s->sdbus)) {
            s->running = true;
            s->left = s->regs[R_DLEN / 4] & 0xFFFFFF;
            s->moved = 0;
            sdc_fifo(s);
        }
    }
}

static uint64_t sdc_read(void *opaque, hwaddr off, unsigned size)
{
    GmSdc010State *s = opaque;
    uint32_t v = 0;
    int i;

    switch (off) {
    case R_STATUS:
        return s->regs[R_STATUS / 4] | ST_DAT0 |
               (sdbus_get_inserted(&s->sdbus) ? ST_CARD : 0) |
               (sdbus_get_readonly(&s->sdbus) ? ST_WP : 0);
    case R_BUSW:
        return s->regs[R_BUSW / 4] | (1u << 3);
    case R_FEATURE:
        return SDC_FIFO / 4;
    case R_DATA:
        if (!s->running || !s->left || (s->regs[R_DCR / 4] & DCR_WRITE)) {
            return 0;
        }
        for (i = 0; i < 4 && i < s->left; i++) {
            v |= (uint32_t)sdbus_read_byte(&s->sdbus) << (8 * i);
        }
        sdc_moved(s);
        return v;
    }
    return off / 4 < SDC_NREGS ? s->regs[off / 4] : 0;
}

static void sdc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    GmSdc010State *s = opaque;
    int i;

    switch (off) {
    case R_CMD:
        s->regs[R_CMD / 4] = val & ~(CMD_GO | CMD_RESET);
        if (val & CMD_RESET) {
            s->regs[R_STATUS / 4] = 0;
            s->regs[R_DCR / 4] = 0;
            s->running = false;
            s->left = 0;
        } else if (val & CMD_GO) {
            sdc_command(s, val);
        }
        break;
    case R_DCR:
        s->regs[R_DCR / 4] = val;
        if (!(val & DCR_EN)) {
            s->running = false;
            s->left = 0;
        }
        break;
    case R_DATA:
        if (s->running && s->left && (s->regs[R_DCR / 4] & DCR_WRITE)) {
            for (i = 0; i < 4 && i < s->left; i++) {
                sdbus_write_byte(&s->sdbus, val >> (8 * i));
            }
            sdc_moved(s);
        }
        break;
    case R_STATUS:
    case R_FEATURE:
        break;
    case R_CLEAR:
        s->regs[R_STATUS / 4] &= ~val;
        break;
    case R_BUSW:
        s->regs[R_BUSW / 4] = val & 7;
        break;
    default:
        if (off / 4 < SDC_NREGS) {
            s->regs[off / 4] = val;
        }
    }
    sdc_update(s);
}

static const MemoryRegionOps sdc_ops = {
    .read = sdc_read,
    .write = sdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void sdc_set_inserted(DeviceState *dev, bool inserted)
{
    GmSdc010State *s = GM_SDC010(dev);

    s->regs[R_STATUS / 4] |= ST_CHANGE;
    sdc_update(s);
}

static void sdc_reset(DeviceState *dev)
{
    GmSdc010State *s = GM_SDC010(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->left = s->moved = 0;
    s->running = false;
    sdc_update(s);
}

static void sdc_realize(DeviceState *dev, Error **errp)
{
    GmSdc010State *s = GM_SDC010(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sdc_ops, s, TYPE_GM_SDC010,
                          SDC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    qdev_init_gpio_out_named(dev, &s->dma_req, "dma-req", 1);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_GM_SDC010_BUS, dev, "sd-bus");
}

static const VMStateDescription vmstate_sdc = {
    .name = TYPE_GM_SDC010,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmSdc010State, SDC_NREGS),
        VMSTATE_UINT32(left, GmSdc010State),
        VMSTATE_UINT32(moved, GmSdc010State),
        VMSTATE_BOOL(running, GmSdc010State),
        VMSTATE_END_OF_LIST()
    }
};

static void sdc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sdc_realize;
    device_class_set_legacy_reset(dc, sdc_reset);
    dc->vmsd = &vmstate_sdc;
}

static void sdc_bus_class_init(ObjectClass *oc, const void *data)
{
    SD_BUS_CLASS(oc)->set_inserted = sdc_set_inserted;
}

static const TypeInfo sdc_types[] = {
    {
        .name = TYPE_GM_SDC010,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(GmSdc010State),
        .class_init = sdc_class_init,
    }, {
        .name = TYPE_GM_SDC010_BUS,
        .parent = TYPE_SD_BUS,
        .class_init = sdc_bus_class_init,
    },
};

DEFINE_TYPES(sdc_types)
