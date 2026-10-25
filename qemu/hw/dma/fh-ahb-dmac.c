/*
 * Fullhan AHB DMA controller: a DesignWare DW_ahb_dmac, 8 channels.
 *
 * The older Fullhan dies' DMA (FH8852 / FH8856 V100, FH8632, FH8626V100),
 * as the kernel's drivers/dma/fh_dmac.c (a dw_dmac fork) and U-Boot's
 * drivers/dma/fh_dma.c drive it.  64-bit registers 8 bytes apart:
 *
 *   per channel, ch * 0x58: SAR 0x00, DAR 0x08, LLP 0x10, CTL 0x18,
 *     SSTAT 0x20, DSTAT 0x28, SSTATAR 0x30, DSTATAR 0x38, CFG 0x40,
 *     SGR 0x48, DSR 0x50
 *   0x2C0 RAW, 0x2E8 STATUS, 0x310 MASK, 0x338 CLEAR: each the five
 *     interrupt kinds 8 bytes apart (XFER, BLOCK, SRC_TRAN, DST_TRAN,
 *     ERR), one bit per channel; MASK bits change only with their write
 *     enable (bits 15:8)
 *   0x360 STATUS_INT  0x398 CFG (bit 0 enable)  0x3A0 CH_EN (bits 7:0,
 *     write enables 15:8)  0x3A8 ID
 *   0x3CC FHC_PARAMS[8] (last channel first), 0x3EC MULTI_BLK_TYPE,
 *     0x3F0 MAX_BLK_SIZE (a nibble per channel), 0x3F4 FH_PARAMS: the
 *     encoded parameters the kernel sizes its channels from (bit 28 says
 *     they are there)
 *
 * CTL_LO: INT_EN 0, DST_WIDTH 3:1, SRC_WIDTH 6:4, DINC 8:7 / SINC 10:9
 * (0 increment, 1 decrement, else fixed), TT_FC 22:20 (0 memory to memory,
 * 1 memory to peripheral, 2 peripheral to memory), LLP_D_EN 27, LLP_S_EN
 * 28.  CTL_HI: BLOCK_TS 11:0, DONE 12.  CFG_HI: SRC_PER 10:7, DST_PER
 * 14:11.  With LLP enabled in CTL and LLP set, enabling a channel fetches
 * the descriptor there first -- {sar, dar, llp, ctllo, ctlhi} -- and each
 * block that leaves LLP enabled fetches the next; DONE goes back into the
 * descriptor's ctlhi.  The end of a block raises BLOCK, the end of the
 * transfer XFER, and the channel's enable drops.
 *
 * As in fh-axi-dmac, items move in a bottom half, and a peripheral end
 * waits for its handshake request ("dma-req" input N).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_FH_AHB_DMAC "fh-ahb-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(FhAhbDmacState, FH_AHB_DMAC)

#define FH_AHB_SIZE         0x1000
#define FH_AHB_CH           8
#define FH_AHB_CH_STRIDE    0x58
#define FH_AHB_NUM_REQ      16

#define C_SAR               0x00
#define C_DAR               0x08
#define C_LLP               0x10
#define C_CTL_LO            0x18
#define C_CTL_HI            0x1C
#define C_CFG_LO            0x40
#define C_CFG_HI            0x44

#define R_RAW               0x2C0
#define R_STATUS            0x2E8
#define R_MASK              0x310
#define R_CLEAR             0x338
#define R_STATUS_INT        0x360
#define R_CFG               0x398
#define R_CH_EN             0x3A0
#define R_ID                0x3A8
#define R_FHC_PARAMS        0x3CC
#define R_MULTI_BLK_TYPE    0x3EC
#define R_MAX_BLK_SIZE      0x3F0
#define R_FH_PARAMS         0x3F4

#define IRQ_KINDS           5   /* XFER, BLOCK, SRC_TRAN, DST_TRAN, ERR */
#define K_XFER              0
#define K_BLOCK             1

#define CTL_INT_EN          (1u << 0)
#define CTL_LLP_D_EN        (1u << 27)
#define CTL_LLP_S_EN        (1u << 28)
#define CTLH_DONE           (1u << 12)

#define TT_M2P              1
#define TT_P2M              2

typedef struct FhAhbChan {
    uint32_t sar, dar, llp;
    uint32_t ctl_lo, ctl_hi;
    uint32_t cfg_lo, cfg_hi;
    uint32_t lli;               /* where the current descriptor came from */
    uint32_t done;              /* items moved of the current block */
    bool en;
    bool active;
} FhAhbChan;

struct FhAhbDmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *bh;
    AddressSpace *as;

    uint32_t cfg;
    uint32_t req;
    uint32_t raw[IRQ_KINDS];
    uint32_t mask[IRQ_KINDS];
    FhAhbChan ch[FH_AHB_CH];
};

static void fh_ahb_update(FhAhbDmacState *s)
{
    bool level = false;
    int k;

    if (s->cfg & 1) {
        for (k = 0; k < IRQ_KINDS; k++) {
            if (s->raw[k] & s->mask[k]) {
                level = true;
            }
        }
    }
    qemu_set_irq(s->irq, level);
}

static bool fh_ahb_fetch(FhAhbDmacState *s, FhAhbChan *c)
{
    uint32_t d[5];
    int i;

    if (!c->llp) {
        return false;
    }
    if (dma_memory_read(s->as, c->llp, d, sizeof(d),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "fh-ahb-dmac: bad LLI 0x%08x\n",
                      c->llp);
        return false;
    }
    for (i = 0; i < 5; i++) {
        d[i] = le32_to_cpu(d[i]);
    }
    c->lli = c->llp;
    c->sar = d[0];
    c->dar = d[1];
    c->llp = d[2];
    c->ctl_lo = d[3];
    c->ctl_hi = d[4] & ~CTLH_DONE;
    c->done = 0;
    c->active = true;
    return true;
}

static bool fh_ahb_req_ok(FhAhbDmacState *s, FhAhbChan *c)
{
    switch ((c->ctl_lo >> 20) & 7) {
    case TT_M2P:
        return s->req & (1u << ((c->cfg_hi >> 11) & 0xF));
    case TT_P2M:
        return s->req & (1u << ((c->cfg_hi >> 7) & 0xF));
    default:
        return true;
    }
}

static void fh_ahb_step(uint32_t *addr, unsigned inc, unsigned w)
{
    if (inc == 0) {
        *addr += w;
    } else if (inc == 1) {
        *addr -= w;
    }
}

static void fh_ahb_item(FhAhbDmacState *s, FhAhbChan *c)
{
    unsigned sw = MIN(1u << ((c->ctl_lo >> 4) & 7), 8);
    unsigned dw = MIN(1u << ((c->ctl_lo >> 1) & 7), 8);
    unsigned sinc = (c->ctl_lo >> 9) & 3, dinc = (c->ctl_lo >> 7) & 3;
    uint8_t buf[8] = { 0 };
    unsigned off;

    dma_memory_read(s->as, c->sar, buf, sw, MEMTXATTRS_UNSPECIFIED);
    fh_ahb_step(&c->sar, sinc, sw);
    if (dinc == 0 || dinc == 1) {
        /* Memory: the bytes as they came. */
        dma_memory_write(s->as, c->dar, buf, sw, MEMTXATTRS_UNSPECIFIED);
        fh_ahb_step(&c->dar, dinc, sw);
        return;
    }
    for (off = 0; off < sw; off += dw) {
        dma_memory_write(s->as, c->dar, buf + off, MIN(dw, sw - off),
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/* The block ended: report it, then chain on or finish. */
static void fh_ahb_block_done(FhAhbDmacState *s, FhAhbChan *c, int n)
{
    uint32_t bit = 1u << n;

    c->ctl_hi |= CTLH_DONE;
    if (c->lli) {
        uint32_t hi = cpu_to_le32(c->ctl_hi);

        dma_memory_write(s->as, c->lli + 16, &hi, 4, MEMTXATTRS_UNSPECIFIED);
    }
    if (c->ctl_lo & CTL_INT_EN) {
        s->raw[K_BLOCK] |= bit;
    }
    if ((c->ctl_lo & (CTL_LLP_S_EN | CTL_LLP_D_EN)) && fh_ahb_fetch(s, c)) {
        return;
    }
    s->raw[K_XFER] |= bit;
    c->en = false;
    c->active = false;
}

static void fh_ahb_run(FhAhbDmacState *s, int n)
{
    FhAhbChan *c = &s->ch[n];
    unsigned budget = 1 << 20;

    while (c->en && c->active && budget) {
        if (c->done >= (c->ctl_hi & 0xFFF)) {
            fh_ahb_block_done(s, c, n);
            continue;
        }
        if (!fh_ahb_req_ok(s, c)) {
            return;
        }
        fh_ahb_item(s, c);
        c->done++;
        budget--;
    }
    if (!budget) {
        qemu_bh_schedule(s->bh);
    }
}

static void fh_ahb_bh(void *opaque)
{
    FhAhbDmacState *s = opaque;
    int n;

    if (s->cfg & 1) {
        for (n = 0; n < FH_AHB_CH; n++) {
            fh_ahb_run(s, n);
        }
    }
    fh_ahb_update(s);
}

static void fh_ahb_set_req(void *opaque, int n, int level)
{
    FhAhbDmacState *s = opaque;
    uint32_t old = s->req;

    s->req = deposit32(s->req, n, 1, !!level);
    if (level && !(old & (1u << n))) {
        qemu_bh_schedule(s->bh);
    }
}

static void fh_ahb_enable(FhAhbDmacState *s, FhAhbChan *c)
{
    c->en = true;
    c->lli = 0;
    c->done = 0;
    if (c->ctl_lo & (CTL_LLP_S_EN | CTL_LLP_D_EN)) {
        c->active = fh_ahb_fetch(s, c);
    } else {
        c->active = true;       /* a single block from the registers */
    }
    qemu_bh_schedule(s->bh);
}

static uint64_t fh_ahb_read(void *opaque, hwaddr off, unsigned size)
{
    FhAhbDmacState *s = opaque;
    uint32_t v = 0;
    int i;

    if (off < FH_AHB_CH * FH_AHB_CH_STRIDE) {
        FhAhbChan *c = &s->ch[off / FH_AHB_CH_STRIDE];

        switch (off % FH_AHB_CH_STRIDE) {
        case C_SAR:     return c->sar;
        case C_DAR:     return c->dar;
        case C_LLP:     return c->llp;
        case C_CTL_LO:  return c->ctl_lo;
        case C_CTL_HI:  return c->ctl_hi;
        case C_CFG_LO:  return c->cfg_lo | (1u << 9);   /* FIFO empty */
        case C_CFG_HI:  return c->cfg_hi;
        default:        return 0;
        }
    }
    switch (off) {
    case R_RAW ... R_RAW + 0x20:
        return (off & 7) ? 0 : s->raw[(off - R_RAW) >> 3];
    case R_STATUS ... R_STATUS + 0x20:
        if (off & 7) {
            return 0;
        }
        i = (off - R_STATUS) >> 3;
        return s->raw[i] & s->mask[i];
    case R_MASK ... R_MASK + 0x20:
        return (off & 7) ? 0 : s->mask[(off - R_MASK) >> 3];
    case R_STATUS_INT:
        for (i = 0; i < IRQ_KINDS; i++) {
            if (s->raw[i] & s->mask[i]) {
                v |= 1u << i;
            }
        }
        return v;
    case R_CFG:
        return s->cfg;
    case R_CH_EN:
        for (i = 0; i < FH_AHB_CH; i++) {
            v |= s->ch[i].en << i;
        }
        return v;
    case R_ID:
        return 0;
    case R_FHC_PARAMS ... R_FHC_PARAMS + 4 * (FH_AHB_CH - 1):
        return 1u << 11;        /* multi-block transfers */
    case R_MULTI_BLK_TYPE:
        return 0;
    case R_MAX_BLK_SIZE:
        return 0xAAAAAAAA;      /* 4 << 10: 4095-item blocks */
    case R_FH_PARAMS:
        /* encoded, 8 channels, 2 AHB masters, 32-bit data */
        return (1u << 28) | ((FH_AHB_CH - 1) << 8) | (1u << 11);
    default:
        return 0;
    }
}

static void fh_ahb_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhAhbDmacState *s = opaque;
    int i;

    if (off < FH_AHB_CH * FH_AHB_CH_STRIDE) {
        FhAhbChan *c = &s->ch[off / FH_AHB_CH_STRIDE];

        switch (off % FH_AHB_CH_STRIDE) {
        case C_SAR:     c->sar = val; break;
        case C_DAR:     c->dar = val; break;
        case C_LLP:     c->llp = val & ~3u; break;
        case C_CTL_LO:  c->ctl_lo = val; break;
        case C_CTL_HI:  c->ctl_hi = val; break;
        case C_CFG_LO:  c->cfg_lo = val & ~(1u << 9); break;
        case C_CFG_HI:  c->cfg_hi = val; break;
        }
        return;
    }
    switch (off) {
    case R_MASK ... R_MASK + 0x20:
        if (!(off & 7)) {
            i = (off - R_MASK) >> 3;
            s->mask[i] = (s->mask[i] & ~(val >> 8)) | (val & (val >> 8));
        }
        break;
    case R_CLEAR ... R_CLEAR + 0x20:
        if (!(off & 7)) {
            s->raw[(off - R_CLEAR) >> 3] &= ~val;
        }
        break;
    case R_CFG:
        s->cfg = val & 1;
        if (!s->cfg) {
            for (i = 0; i < FH_AHB_CH; i++) {
                s->ch[i].en = s->ch[i].active = false;
            }
        }
        qemu_bh_schedule(s->bh);
        break;
    case R_CH_EN:
        for (i = 0; i < FH_AHB_CH; i++) {
            FhAhbChan *c = &s->ch[i];

            if (!(val & (1u << (8 + i)))) {
                continue;
            }
            if ((val & (1u << i)) && !c->en) {
                fh_ahb_enable(s, c);
            } else if (!(val & (1u << i))) {
                c->en = c->active = false;
            }
        }
        break;
    }
    fh_ahb_update(s);
}

static const MemoryRegionOps fh_ahb_ops = {
    .read = fh_ahb_read,
    .write = fh_ahb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_ahb_reset(DeviceState *dev)
{
    FhAhbDmacState *s = FH_AHB_DMAC(dev);

    s->cfg = 0;
    memset(s->raw, 0, sizeof(s->raw));
    memset(s->mask, 0, sizeof(s->mask));
    memset(s->ch, 0, sizeof(s->ch));
    fh_ahb_update(s);
}

static void fh_ahb_realize(DeviceState *dev, Error **errp)
{
    FhAhbDmacState *s = FH_AHB_DMAC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->as = &address_space_memory;
    s->bh = qemu_bh_new(fh_ahb_bh, s);
    memory_region_init_io(&s->iomem, OBJECT(s), &fh_ahb_ops, s,
                          TYPE_FH_AHB_DMAC, FH_AHB_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, fh_ahb_set_req, "dma-req", FH_AHB_NUM_REQ);
}

static const VMStateDescription vmstate_fh_ahb_chan = {
    .name = "fh-ahb-dmac-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(sar, FhAhbChan),
        VMSTATE_UINT32(dar, FhAhbChan),
        VMSTATE_UINT32(llp, FhAhbChan),
        VMSTATE_UINT32(ctl_lo, FhAhbChan),
        VMSTATE_UINT32(ctl_hi, FhAhbChan),
        VMSTATE_UINT32(cfg_lo, FhAhbChan),
        VMSTATE_UINT32(cfg_hi, FhAhbChan),
        VMSTATE_UINT32(lli, FhAhbChan),
        VMSTATE_UINT32(done, FhAhbChan),
        VMSTATE_BOOL(en, FhAhbChan),
        VMSTATE_BOOL(active, FhAhbChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_fh_ahb = {
    .name = TYPE_FH_AHB_DMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, FhAhbDmacState),
        VMSTATE_UINT32(req, FhAhbDmacState),
        VMSTATE_UINT32_ARRAY(raw, FhAhbDmacState, IRQ_KINDS),
        VMSTATE_UINT32_ARRAY(mask, FhAhbDmacState, IRQ_KINDS),
        VMSTATE_STRUCT_ARRAY(ch, FhAhbDmacState, FH_AHB_CH, 1,
                             vmstate_fh_ahb_chan, FhAhbChan),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_ahb_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = fh_ahb_realize;
    device_class_set_legacy_reset(dc, fh_ahb_reset);
    dc->vmsd = &vmstate_fh_ahb;
}

static const TypeInfo fh_ahb_info = {
    .name           = TYPE_FH_AHB_DMAC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhAhbDmacState),
    .class_init     = fh_ahb_class_init,
};

static void fh_ahb_register_types(void)
{
    type_register_static(&fh_ahb_info);
}

type_init(fh_ahb_register_types)
