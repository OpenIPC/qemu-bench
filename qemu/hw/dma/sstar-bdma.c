/*
 * SigmaStar / MStar BDMA (RIU bank 0x1002, 0x1F200400).
 *
 * Four byte-DMA channels, 0x20 RIU registers (0x80 bytes) apart
 * (drivers/sstar/bdma/infinity2m/kernel_bdma.h):
 *
 *   +0x00 trigger (bit 0, self-clearing)
 *   +0x01 status, write 1 to clear: bit 1 busy, bit 2 interrupt,
 *         bit 3 done, bit 4 result
 *   +0x02 path: [3:0] source, [11:8] destination
 *   +0x03 bit 1 interrupt enable, [13:12] which memory the MIU ports
 *         reach (2 = IMI)
 *   +0x04/05 source, +0x06/07 destination, +0x08/09 length
 *   +0x0A/0B the fill pattern (source 4)
 *
 * Ports: 0/1 MIU (DRAM from bus address 0, or IMI), 4 memory fill
 * (source), 5 the SPI NOR (source), 11 the FSP's outside write buffer
 * (destination).  The boot ROM copies the IPL from flash to IMI with
 * source 5; U-Boot programs pages with MIU -> FSP (HAL_FSP_Write_BDMA:
 * path 0x2B40) and fires the FSP after triggering the channel.
 * Transfers complete inside the trigger write.
 *
 * The port decoding and the IMI select follow Daniel Palmer's mstar_bdma.c
 * (github.com/fifteenhex/qemu, mstar-clean), checked against the vendor
 * register map above.
 *
 * Copyright (c) 2026 Daniel Palmer <daniel@thingy.jp>
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ssi/sstar-fsp.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_SSTAR_BDMA "sstar-bdma"
OBJECT_DECLARE_SIMPLE_TYPE(SstarBdmaState, SSTAR_BDMA)

#define BDMA_NUM_CHAN       4
#define BDMA_CHAN_REGS      0x20

#define R_TRIG              0x00
#define R_STATUS            0x01
#define R_PATH              0x02
#define R_MISC              0x03
#define R_SRC_L             0x04
#define R_DST_L             0x06
#define R_SIZE_L            0x08
#define R_PATTERN_L         0x0A

#define ST_BUSY             (1u << 1)
#define ST_INT              (1u << 2)
#define ST_DONE             (1u << 3)
#define ST_RESULT           (1u << 4)
#define MISC_INT_EN         (1u << 1)
#define MISC_MEM_IMI        2

#define PORT_MIU0           0
#define PORT_MIU1           1
#define PORT_FILL           4
#define PORT_SPI            5
#define PORT_FSP            11

#define DRAM_BASE           0x20000000
#define IMI_BASE            0xA0000000

struct SstarBdmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SstarFspState *fsp;
    uint16_t regs[BDMA_NUM_CHAN][BDMA_CHAN_REGS];
    qemu_irq irq[BDMA_NUM_CHAN];
};

static uint32_t reg32(uint16_t *r, int lo)
{
    return r[lo] | ((uint32_t)r[lo + 1] << 16);
}

static hwaddr bdma_mem_addr(uint16_t *r, uint32_t a)
{
    /* Bus addresses: DRAM from 0, as the drivers program them. */
    if (((r[R_MISC] >> 12) & 0x3) == MISC_MEM_IMI) {
        return IMI_BASE + (a & 0xFFFF);
    }
    return DRAM_BASE + (a & 0x1FFFFFFF);
}

static void bdma_update_irq(SstarBdmaState *s, int ch)
{
    uint16_t *r = s->regs[ch];

    qemu_set_irq(s->irq[ch], (r[R_STATUS] & ST_INT) &&
                 (r[R_MISC] & MISC_INT_EN));
}

static void bdma_run(SstarBdmaState *s, int ch)
{
    uint16_t *r = s->regs[ch];
    int src = r[R_PATH] & 0xF;
    int dst = (r[R_PATH] >> 8) & 0xF;
    uint32_t sa = reg32(r, R_SRC_L);
    uint32_t da = reg32(r, R_DST_L);
    uint32_t len = reg32(r, R_SIZE_L);
    uint8_t buf[4096];

    while (len) {
        uint32_t n = MIN(len, sizeof(buf));
        uint32_t i;

        switch (src) {
        case PORT_MIU0:
        case PORT_MIU1:
            dma_memory_read(&address_space_memory, bdma_mem_addr(r, sa), buf,
                            n, MEMTXATTRS_UNSPECIFIED);
            break;
        case PORT_FILL:
            for (i = 0; i < n; i++) {
                buf[i] = reg32(r, R_PATTERN_L) >> (8 * ((sa + i) & 3));
            }
            break;
        case PORT_SPI:
            if (s->fsp) {
                sstar_fsp_read_flash(s->fsp, sa, buf, n);
            } else {
                memset(buf, 0xFF, n);
            }
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "sstar-bdma: ch%d source port %d\n",
                          ch, src);
            goto done;
        }

        switch (dst) {
        case PORT_MIU0:
        case PORT_MIU1:
            dma_memory_write(&address_space_memory, bdma_mem_addr(r, da),
                             buf, n, MEMTXATTRS_UNSPECIFIED);
            break;
        case PORT_FSP:
            if (s->fsp) {
                sstar_fsp_push_outside(s->fsp, buf, n);
            }
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "sstar-bdma: ch%d destination port "
                          "%d\n", ch, dst);
            goto done;
        }
        sa += n;
        da += n;
        len -= n;
    }
done:
    r[R_STATUS] = (r[R_STATUS] & ~ST_BUSY) | ST_INT | ST_DONE | ST_RESULT;
    bdma_update_irq(s, ch);
}

static uint64_t sstar_bdma_read(void *opaque, hwaddr off, unsigned size)
{
    SstarBdmaState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->regs[reg / BDMA_CHAN_REGS][reg % BDMA_CHAN_REGS];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_bdma_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SstarBdmaState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    int ch = reg / BDMA_CHAN_REGS;
    uint16_t *r = s->regs[ch];
    uint16_t v = val;

    reg %= BDMA_CHAN_REGS;
    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = (off & 1) ? (r[reg] & 0x00FF) | ((val & 0xFF) << 8)
                      : (r[reg] & 0xFF00) | (val & 0xFF);
        /* A byte write to the status high half clears nothing. */
        if (reg == R_STATUS && (off & 1)) {
            return;
        }
    }
    switch (reg) {
    case R_TRIG:
        r[R_TRIG] = v & ~1;
        if (v & 1) {
            bdma_run(s, ch);
        }
        break;
    case R_STATUS:
        r[R_STATUS] &= ~(v & 0x1F);
        bdma_update_irq(s, ch);
        break;
    default:
        r[reg] = v;
        if (reg == R_MISC) {
            bdma_update_irq(s, ch);
        }
        break;
    }
}

static const MemoryRegionOps sstar_bdma_ops = {
    .read = sstar_bdma_read,
    .write = sstar_bdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_bdma_reset(DeviceState *dev)
{
    SstarBdmaState *s = SSTAR_BDMA(dev);
    int ch;

    memset(s->regs, 0, sizeof(s->regs));
    for (ch = 0; ch < BDMA_NUM_CHAN; ch++) {
        qemu_set_irq(s->irq[ch], 0);
    }
}

static void sstar_bdma_init(Object *obj)
{
    SstarBdmaState *s = SSTAR_BDMA(obj);
    int ch;

    memory_region_init_io(&s->iomem, obj, &sstar_bdma_ops, s, "sstar-bdma",
                          0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (ch = 0; ch < BDMA_NUM_CHAN; ch++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[ch]);
    }
}

static const Property sstar_bdma_props[] = {
    DEFINE_PROP_LINK("fsp", SstarBdmaState, fsp, TYPE_SSTAR_FSP,
                     SstarFspState *),
};

static const VMStateDescription vmstate_sstar_bdma = {
    .name = TYPE_SSTAR_BDMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_2DARRAY(regs, SstarBdmaState, BDMA_NUM_CHAN,
                               BDMA_CHAN_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_bdma_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_bdma_reset);
    device_class_set_props(dc, sstar_bdma_props);
    dc->vmsd = &vmstate_sstar_bdma;
}

static const TypeInfo sstar_bdma_info = {
    .name           = TYPE_SSTAR_BDMA,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarBdmaState),
    .instance_init  = sstar_bdma_init,
    .class_init     = sstar_bdma_class_init,
};

static void sstar_bdma_register_types(void)
{
    type_register_static(&sstar_bdma_info);
}

type_init(sstar_bdma_register_types)
