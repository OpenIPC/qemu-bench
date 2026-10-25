/*
 * Fullhan AXI DMA controller: a DesignWare DW_axi_dmac, linked-list mode.
 *
 * As U-Boot's drivers/dma/fh_axi_dma.c and the kernel's
 * drivers/dma/fh_axi_dma_driver.c drive it.  64-bit registers 8 bytes
 * apart, of which only the low words matter:
 *
 *   0x10 CFG      bit 0 DMAC enable, bit 1 interrupt enable
 *   0x18 CHEN     7:0 channel enable, 15:8 their write enables, 23:16
 *                 suspend, 31:24 its write enables -- a bit only changes
 *                 with its write enable set
 *   0x30 INTSTATUS (one bit per channel)        0x58 RESET (self-clearing)
 *
 * and per channel at 0x100 + ch * 0x100: SAR 0x00, DAR 0x08, BLOCK_TS
 * 0x10, CTL 0x18, CFG 0x20, LLP 0x28, STATUS 0x30, BLK_TFR_RESU 0x48,
 * INTSTATUS_EN 0x80, INTSTATUS 0x88, INTSIGNAL_EN 0x90, INTCLEAR 0x98.
 *
 * Both drivers put the channel in linked-list mode (CFG_LO = 0xF) and
 * point LLP at 64-byte descriptors: SAR, DAR, BLOCK_TS (items - 1), LLP,
 * CTL.  CTL_LO: SRC_INC 4 / DST_INC 6 (1 = fixed), source and destination
 * item widths 10:8 / 13:11; CTL_HI: 26 interrupt on block done, 30 last
 * descriptor, 31 valid.  CFG_HI: TT_FC 2:0 (0 memory to memory, 1 memory
 * to peripheral, 2 peripheral to memory), SRC_PER 10:7, DST_PER 15:12.
 *
 * Items move in a bottom half, never inside the CHEN write: the kernel
 * counts channels by enabling each with nothing programmed and reading
 * CHEN back.  A channel enabled outside linked-list mode, or with no
 * list, stays enabled and does nothing.  A peripheral end goes only
 * while its handshake request ("dma-req" input N) is up -- the SPI
 * controllers raise theirs while their RX FIFO holds data.  The end of a
 * descriptor raises BLOCK_TFR if it asks for it, the last one DMA_TFR,
 * and the channel's enable drops.  Descriptors are not written back:
 * neither driver reads them.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_FH_AXI_DMAC "fh-axi-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(FhAxiDmacState, FH_AXI_DMAC)

#define FH_DMAC_SIZE        0x1000
#define FH_DMAC_MAX_CH      8
#define FH_DMAC_NUM_REQ     16

#define G_ID                0x00
#define G_COMPVER           0x08
#define G_CFG               0x10
#define G_CHEN              0x18
#define G_INTSTATUS         0x30
#define G_RESET             0x58

#define C_SAR               0x00
#define C_DAR               0x08
#define C_BLOCK_TS          0x10
#define C_CTL_LO            0x18
#define C_CTL_HI            0x1C
#define C_CFG_LO            0x20
#define C_CFG_HI            0x24
#define C_LLP               0x28
#define C_STATUS            0x30
#define C_BLK_TFR_RESU      0x48
#define C_INTSTATUS_EN      0x80
#define C_INTSTATUS         0x88
#define C_INTSIGNAL_EN      0x90
#define C_INTCLEAR          0x98

#define CFG_DMAC_EN         (1u << 0)
#define CFG_INT_EN          (1u << 1)

#define CTL_SRC_FIX         (1u << 4)
#define CTL_DST_FIX         (1u << 6)
#define CTLH_IOC_BLKTFR     (1u << 26)
#define CTLH_LLI_LAST       (1u << 30)
#define CTLH_LLI_VALID      (1u << 31)

#define IRQ_BLOCK_TFR       (1u << 0)
#define IRQ_DMA_TFR         (1u << 1)
#define IRQ_SRC_DEC_ERR     (1u << 5)
#define IRQ_DST_DEC_ERR     (1u << 6)
#define IRQ_SUSPENDED       (1u << 29)
#define IRQ_DISABLED        (1u << 30)

#define TT_M2M              0
#define TT_M2P              1
#define TT_P2M              2

typedef struct FhAxiDmacChan {
    uint32_t sar;
    uint32_t dar;
    uint32_t block_ts;
    uint32_t ctl_lo;
    uint32_t ctl_hi;
    uint32_t cfg_lo;
    uint32_t cfg_hi;
    uint32_t llp;
    uint32_t intstatus_en;
    uint32_t intstatus;
    uint32_t intsignal_en;
    uint32_t done;              /* items moved of the current block */
    bool en;
    bool susp;
    bool active;                /* a descriptor is loaded */
} FhAxiDmacChan;

struct FhAxiDmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *bh;
    AddressSpace *as;

    uint32_t cfg;
    uint32_t req;               /* handshake request levels */
    FhAxiDmacChan ch[FH_DMAC_MAX_CH];
    uint32_t num_ch;
};

static void fh_dmac_update(FhAxiDmacState *s)
{
    bool level = false;
    int i;

    if (s->cfg & CFG_INT_EN) {
        for (i = 0; i < s->num_ch; i++) {
            if (s->ch[i].intstatus & s->ch[i].intsignal_en) {
                level = true;
            }
        }
    }
    qemu_set_irq(s->irq, level);
}

static void fh_dmac_event(FhAxiDmacChan *c, uint32_t ev)
{
    c->intstatus |= ev & c->intstatus_en;
}

/* Load the descriptor at LLP; false ends the transfer. */
static bool fh_dmac_fetch(FhAxiDmacState *s, FhAxiDmacChan *c)
{
    uint32_t d[10];
    int i;

    if (!c->llp) {
        return false;
    }
    if (dma_memory_read(s->as, c->llp, d, sizeof(d),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "fh-axi-dmac: bad LLI 0x%08x\n",
                      c->llp);
        return false;
    }
    for (i = 0; i < 10; i++) {
        d[i] = le32_to_cpu(d[i]);
    }
    if (!(d[9] & CTLH_LLI_VALID)) {
        return false;
    }
    c->sar = d[0];
    c->dar = d[2];
    c->block_ts = d[4];
    c->llp = d[6];
    c->ctl_lo = d[8];
    c->ctl_hi = d[9];
    c->done = 0;
    c->active = true;
    return true;
}

static void fh_dmac_stop(FhAxiDmacChan *c)
{
    c->en = false;
    c->active = false;
}

static bool fh_dmac_req_ok(FhAxiDmacState *s, FhAxiDmacChan *c)
{
    switch (c->cfg_hi & 7) {
    case TT_M2P:
        return s->req & (1u << ((c->cfg_hi >> 12) & 0xF));
    case TT_P2M:
        return s->req & (1u << ((c->cfg_hi >> 7) & 0xF));
    default:
        return true;
    }
}

/* Move one source item to the destination: the error to report, or 0. */
static uint32_t fh_dmac_item(FhAxiDmacState *s, FhAxiDmacChan *c)
{
    unsigned sw = 1u << ((c->ctl_lo >> 8) & 7);
    unsigned dw = 1u << ((c->ctl_lo >> 11) & 7);
    uint8_t buf[8] = { 0 };
    MemTxResult r = MEMTX_OK;
    unsigned off;

    sw = MIN(sw, 8);
    dw = MIN(dw, 8);
    if (dma_memory_read(s->as, c->sar, buf, sw,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return IRQ_SRC_DEC_ERR;
    }
    if (!(c->ctl_lo & CTL_SRC_FIX)) {
        c->sar += sw;
    }
    if (!(c->ctl_lo & CTL_DST_FIX)) {
        r = dma_memory_write(s->as, c->dar, buf, sw, MEMTXATTRS_UNSPECIFIED);
        c->dar += sw;
    } else {
        /* A peripheral register takes the bytes in its own access width. */
        for (off = 0; off < sw; off += dw) {
            r |= dma_memory_write(s->as, c->dar, buf + off,
                                  MIN(dw, sw - off), MEMTXATTRS_UNSPECIFIED);
        }
    }
    return r == MEMTX_OK ? 0 : IRQ_DST_DEC_ERR;
}

static void fh_dmac_run(FhAxiDmacState *s, FhAxiDmacChan *c)
{
    unsigned budget = 1 << 20;  /* items, before yielding to the guest */
    uint32_t err;

    while (c->en && c->active && !c->susp && budget) {
        budget--;               /* a chain of empty blocks spends it too */
        if (c->done > c->block_ts) {
            if (c->ctl_hi & CTLH_IOC_BLKTFR) {
                fh_dmac_event(c, IRQ_BLOCK_TFR);
            }
            if ((c->ctl_hi & CTLH_LLI_LAST) || !fh_dmac_fetch(s, c)) {
                fh_dmac_event(c, IRQ_DMA_TFR);
                fh_dmac_stop(c);
            }
            continue;
        }
        if (!fh_dmac_req_ok(s, c)) {
            return;
        }
        err = fh_dmac_item(s, c);
        if (err) {
            qemu_log_mask(LOG_GUEST_ERROR, "fh-axi-dmac: bus error at "
                          "0x%08x -> 0x%08x\n", c->sar, c->dar);
            fh_dmac_event(c, err);
            fh_dmac_stop(c);
            break;
        }
        c->done++;
    }
    if (!budget) {
        qemu_bh_schedule(s->bh);
    }
}

static void fh_dmac_bh(void *opaque)
{
    FhAxiDmacState *s = opaque;
    int i;

    if (s->cfg & CFG_DMAC_EN) {
        for (i = 0; i < s->num_ch; i++) {
            fh_dmac_run(s, &s->ch[i]);
        }
    }
    fh_dmac_update(s);
}

static void fh_dmac_set_req(void *opaque, int n, int level)
{
    FhAxiDmacState *s = opaque;
    uint32_t old = s->req;

    s->req = deposit32(s->req, n, 1, !!level);
    if (level && !(old & (1u << n))) {
        qemu_bh_schedule(s->bh);
    }
}

static void fh_dmac_enable(FhAxiDmacState *s, FhAxiDmacChan *c)
{
    c->en = true;
    c->susp = false;
    c->active = false;
    /* Linked list on both sides (MULTBLK_TYPE 3), the drivers' mode. */
    if ((c->cfg_lo & 0xF) == 0xF && fh_dmac_fetch(s, c)) {
        qemu_bh_schedule(s->bh);
    }
}

static void fh_dmac_chen_write(FhAxiDmacState *s, uint32_t val)
{
    int i;

    for (i = 0; i < s->num_ch; i++) {
        FhAxiDmacChan *c = &s->ch[i];

        if (val & (1u << (8 + i))) {
            bool on = val & (1u << i);

            if (on && !c->en) {
                fh_dmac_enable(s, c);
            } else if (!on && c->en) {
                fh_dmac_stop(c);
                fh_dmac_event(c, IRQ_DISABLED);
            }
        }
        if (val & (1u << (24 + i))) {
            bool susp = val & (1u << (16 + i));

            if (susp && c->en && !c->susp) {
                c->susp = true;
                fh_dmac_event(c, IRQ_SUSPENDED);
            } else if (!susp && c->susp) {
                c->susp = false;
                qemu_bh_schedule(s->bh);
            }
        }
    }
}

static void fh_dmac_reset_all(FhAxiDmacState *s)
{
    s->cfg = 0;
    memset(s->ch, 0, sizeof(s->ch));
}

static uint64_t fh_dmac_read(void *opaque, hwaddr off, unsigned size)
{
    FhAxiDmacState *s = opaque;
    FhAxiDmacChan *c;
    uint32_t v = 0;
    int i;

    if (off < 0x100) {
        switch (off) {
        case G_ID:
            return 0;
        case G_COMPVER:
            return 0x3130302A;  /* "100*" */
        case G_CFG:
            return s->cfg;
        case G_CHEN:
            for (i = 0; i < s->num_ch; i++) {
                v |= s->ch[i].en << i;
                v |= s->ch[i].susp << (16 + i);
            }
            return v;
        case G_INTSTATUS:
            for (i = 0; i < s->num_ch; i++) {
                if (s->ch[i].intstatus) {
                    v |= 1u << i;
                }
            }
            return v;
        default:
            return 0;
        }
    }
    i = (off >> 8) - 1;
    if (i >= s->num_ch) {
        return 0;
    }
    c = &s->ch[i];
    switch (off & 0xFF) {
    case C_SAR:          return c->sar;
    case C_DAR:          return c->dar;
    case C_BLOCK_TS:     return c->block_ts;
    case C_CTL_LO:       return c->ctl_lo;
    case C_CTL_HI:       return c->ctl_hi;
    case C_CFG_LO:       return c->cfg_lo;
    case C_CFG_HI:       return c->cfg_hi;
    case C_LLP:          return c->llp;
    case C_STATUS:       return c->done;
    case C_INTSTATUS_EN: return c->intstatus_en;
    case C_INTSTATUS:    return c->intstatus;
    case C_INTSIGNAL_EN: return c->intsignal_en;
    default:             return 0;
    }
}

static void fh_dmac_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    FhAxiDmacState *s = opaque;
    FhAxiDmacChan *c;
    int i;

    if (off < 0x100) {
        switch (off) {
        case G_CFG:
            s->cfg = val & 3;
            qemu_bh_schedule(s->bh);
            break;
        case G_CHEN:
            fh_dmac_chen_write(s, val);
            break;
        case G_RESET:
            if (val & 1) {
                fh_dmac_reset_all(s);
            }
            break;
        }
        fh_dmac_update(s);
        return;
    }
    i = (off >> 8) - 1;
    if (i >= s->num_ch) {
        return;
    }
    c = &s->ch[i];
    switch (off & 0xFF) {
    case C_SAR:          c->sar = val; break;
    case C_DAR:          c->dar = val; break;
    case C_BLOCK_TS:     c->block_ts = val; break;
    case C_CTL_LO:       c->ctl_lo = val; break;
    case C_CTL_HI:       c->ctl_hi = val; break;
    case C_CFG_LO:       c->cfg_lo = val; break;
    case C_CFG_HI:       c->cfg_hi = val; break;
    case C_LLP:          c->llp = val & ~0x3Fu; break;
    case C_BLK_TFR_RESU:
        qemu_bh_schedule(s->bh);
        break;
    case C_INTSTATUS_EN: c->intstatus_en = val; break;
    case C_INTSIGNAL_EN: c->intsignal_en = val; break;
    case C_INTCLEAR:     c->intstatus &= ~val; break;
    }
    fh_dmac_update(s);
}

static const MemoryRegionOps fh_dmac_ops = {
    .read = fh_dmac_read,
    .write = fh_dmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_dmac_reset(DeviceState *dev)
{
    FhAxiDmacState *s = FH_AXI_DMAC(dev);

    fh_dmac_reset_all(s);
    fh_dmac_update(s);
}

static void fh_dmac_realize(DeviceState *dev, Error **errp)
{
    FhAxiDmacState *s = FH_AXI_DMAC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (!s->num_ch || s->num_ch > FH_DMAC_MAX_CH) {
        error_setg(errp, "fh-axi-dmac: num-channels must be 1..%d",
                   FH_DMAC_MAX_CH);
        return;
    }
    s->as = &address_space_memory;
    s->bh = qemu_bh_new(fh_dmac_bh, s);
    memory_region_init_io(&s->iomem, OBJECT(s), &fh_dmac_ops, s,
                          TYPE_FH_AXI_DMAC, FH_DMAC_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, fh_dmac_set_req, "dma-req", FH_DMAC_NUM_REQ);
}

static const Property fh_dmac_props[] = {
    DEFINE_PROP_UINT32("num-channels", FhAxiDmacState, num_ch, 8),
};

static const VMStateDescription vmstate_fh_dmac_chan = {
    .name = "fh-axi-dmac-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(sar, FhAxiDmacChan),
        VMSTATE_UINT32(dar, FhAxiDmacChan),
        VMSTATE_UINT32(block_ts, FhAxiDmacChan),
        VMSTATE_UINT32(ctl_lo, FhAxiDmacChan),
        VMSTATE_UINT32(ctl_hi, FhAxiDmacChan),
        VMSTATE_UINT32(cfg_lo, FhAxiDmacChan),
        VMSTATE_UINT32(cfg_hi, FhAxiDmacChan),
        VMSTATE_UINT32(llp, FhAxiDmacChan),
        VMSTATE_UINT32(intstatus_en, FhAxiDmacChan),
        VMSTATE_UINT32(intstatus, FhAxiDmacChan),
        VMSTATE_UINT32(intsignal_en, FhAxiDmacChan),
        VMSTATE_UINT32(done, FhAxiDmacChan),
        VMSTATE_BOOL(en, FhAxiDmacChan),
        VMSTATE_BOOL(susp, FhAxiDmacChan),
        VMSTATE_BOOL(active, FhAxiDmacChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_fh_dmac = {
    .name = TYPE_FH_AXI_DMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, FhAxiDmacState),
        VMSTATE_UINT32(req, FhAxiDmacState),
        VMSTATE_STRUCT_ARRAY(ch, FhAxiDmacState, FH_DMAC_MAX_CH, 1,
                             vmstate_fh_dmac_chan, FhAxiDmacChan),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_dmac_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = fh_dmac_realize;
    device_class_set_legacy_reset(dc, fh_dmac_reset);
    device_class_set_props(dc, fh_dmac_props);
    dc->vmsd = &vmstate_fh_dmac;
}

static const TypeInfo fh_dmac_info = {
    .name           = TYPE_FH_AXI_DMAC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhAxiDmacState),
    .class_init     = fh_dmac_class_init,
};

static void fh_dmac_register_types(void)
{
    type_register_static(&fh_dmac_info);
}

type_init(fh_dmac_register_types)
