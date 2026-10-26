/*
 * Grain Media FTDMAC020 AHB DMA controller.
 *
 * Faraday's eight-channel AHB DMA at 0x92600000 on the GM813x parts
 * (arch/arm/mach-GM/ftdmac020.c and include/mach/ftdmac020.h, U-Boot's
 * GM_AHBDMA, nsboot's ahb_dma.c):
 *
 *   0x00 ISR       0x04 TCISR     0x08 TCICR     0x0C EAISR    0x10 EAICR
 *   0x14 TCRAW     0x18 EARAW     0x1C CH_EN     0x20 CH_BUSY  0x24 CR
 *   0x28 SYNC      0x2C REVISION  0x30 FEATURE
 *
 * and per channel n at 0x100 + n * 0x20: CCR, CFG, SRC, DST, LLP, CYC.
 * CCR: 0 enable, 1/2 dst/src AHB master, 4:3 / 6:5 dst/src address
 * control (increment, decrement, fixed), 7 hardware handshake, 10:8 /
 * 13:11 dst/src width, 15 abort, 18:16 burst, 31 mask the terminal count.
 * CFG: 0-2 mask TC / error / abort interrupts, 6:3 + 7 the source's
 * request line and its enable, 12:9 + 13 the destination's.  CYC counts
 * source-width units.  LLP chains 20-byte descriptors {src, dst, next,
 * ctrl, cycles} in memory, ctrl laid out as 16/17 masters, 19:18 / 21:20
 * address control, 24:22 / 27:25 widths, 28 mask TC.
 *
 * The SPI flash controller paces transfers with request line 8: a
 * handshaken side moves data only while its "dma-req" input is high.
 * Channels run from a bottom half, so a request raised inside a device's
 * register write (the SPI controller's, when a command starts) does not
 * re-enter that device; a transfer that runs out of requests waits for
 * the next one.  A finished chain clears the channel's enable and sets
 * its TC status unless masked; an abort sets the abort status.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "qom/object.h"

#define TYPE_GM_DMAC "gm-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(GmDmacState, GM_DMAC)

#define GM_DMAC_CHANS       8
#define GM_DMAC_REQS        16
#define GM_DMAC_SIZE        0x1000

#define R_ISR               0x00
#define R_TCISR             0x04
#define R_TCICR             0x08
#define R_EAISR             0x0C
#define R_EAICR             0x10
#define R_TCRAW             0x14
#define R_EARAW             0x18
#define R_CH_EN             0x1C
#define R_CH_BUSY           0x20
#define R_CR                0x24
#define R_SYNC              0x28
#define R_REVISION          0x2C
#define R_FEATURE           0x30

#define CH_BASE             0x100
#define CH_STRIDE           0x20
#define C_CCR               0x00
#define C_CFG               0x04
#define C_SRC               0x08
#define C_DST               0x0C
#define C_LLP               0x10
#define C_CYC               0x14

#define CR_ENABLE           (1u << 0)

#define CCR_ENABLE          (1u << 0)
#define CCR_HANDSHAKE       (1u << 7)
#define CCR_ABORT           (1u << 15)
#define CCR_MASK_TC         (1u << 31)

#define CFG_MASK_TC         (1u << 0)
#define CFG_MASK_ERR        (1u << 1)
#define CFG_MASK_ABT        (1u << 2)
#define CFG_SRC_REQ_EN      (1u << 7)
#define CFG_DST_REQ_EN      (1u << 13)
#define CFG_BUSY            (1u << 8)

#define CYC_MASK            0x3FFFFF

typedef struct GmDmacChan {
    uint32_t ccr;
    uint32_t cfg;
    uint32_t src;
    uint32_t dst;
    uint32_t llp;
    uint32_t cyc;
} GmDmacChan;

struct GmDmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *bh;
    AddressSpace *as;

    GmDmacChan ch[GM_DMAC_CHANS];
    uint32_t cr;
    uint32_t sync;
    uint32_t tc_raw;
    uint32_t ea_raw;
    uint32_t req;               /* request line levels */
};

static uint32_t gm_dmac_tc_status(GmDmacState *s)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < GM_DMAC_CHANS; i++) {
        if ((s->tc_raw & (1u << i)) && !(s->ch[i].cfg & CFG_MASK_TC)) {
            v |= 1u << i;
        }
    }
    return v;
}

static uint32_t gm_dmac_ea_status(GmDmacState *s)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < GM_DMAC_CHANS; i++) {
        if ((s->ea_raw & (1u << i)) && !(s->ch[i].cfg & CFG_MASK_ERR)) {
            v |= 1u << i;
        }
        if ((s->ea_raw & (1u << (16 + i))) &&
            !(s->ch[i].cfg & CFG_MASK_ABT)) {
            v |= 1u << (16 + i);
        }
    }
    return v;
}

static void gm_dmac_update(GmDmacState *s)
{
    qemu_set_irq(s->irq, gm_dmac_tc_status(s) || gm_dmac_ea_status(s));
}

static int gm_dmac_width(uint32_t field)
{
    return 1 << MIN(field & 7, 2);
}

static bool gm_dmac_ld(GmDmacState *s, hwaddr a, int w, uint32_t *v)
{
    MemTxResult r;

    switch (w) {
    case 1:
        *v = address_space_ldub(s->as, a, MEMTXATTRS_UNSPECIFIED, &r);
        break;
    case 2:
        *v = address_space_lduw_le(s->as, a, MEMTXATTRS_UNSPECIFIED, &r);
        break;
    default:
        *v = address_space_ldl_le(s->as, a, MEMTXATTRS_UNSPECIFIED, &r);
        break;
    }
    return r == MEMTX_OK;
}

static bool gm_dmac_st(GmDmacState *s, hwaddr a, int w, uint32_t v)
{
    MemTxResult r;

    switch (w) {
    case 1:
        address_space_stb(s->as, a, v, MEMTXATTRS_UNSPECIFIED, &r);
        break;
    case 2:
        address_space_stw_le(s->as, a, v, MEMTXATTRS_UNSPECIFIED, &r);
        break;
    default:
        address_space_stl_le(s->as, a, v, MEMTXATTRS_UNSPECIFIED, &r);
        break;
    }
    return r == MEMTX_OK;
}

static hwaddr gm_dmac_step(hwaddr a, unsigned ctl, int w)
{
    switch (ctl & 3) {
    case 0:
        return a + w;
    case 1:
        return a - w;
    default:
        return a;
    }
}

/* May the channel move data now?  Not while a paced side has no request. */
static bool gm_dmac_ready(GmDmacState *s, GmDmacChan *c)
{
    if (!(c->ccr & CCR_HANDSHAKE)) {
        return true;
    }
    if ((c->cfg & CFG_SRC_REQ_EN) &&
        !(s->req & (1u << extract32(c->cfg, 3, 4)))) {
        return false;
    }
    if ((c->cfg & CFG_DST_REQ_EN) &&
        !(s->req & (1u << extract32(c->cfg, 9, 4)))) {
        return false;
    }
    return true;
}

static void gm_dmac_finish(GmDmacState *s, int n, bool error)
{
    GmDmacChan *c = &s->ch[n];

    c->ccr &= ~CCR_ENABLE;
    c->cfg &= ~CFG_BUSY;
    if (error) {
        s->ea_raw |= 1u << n;
    }
}

/* Load the next descriptor of the chain into the channel registers. */
static bool gm_dmac_next_desc(GmDmacState *s, GmDmacChan *c)
{
    hwaddr a = c->llp & ~3u;
    uint32_t d[5];
    int i;

    for (i = 0; i < 5; i++) {
        if (!gm_dmac_ld(s, a + i * 4, 4, &d[i])) {
            return false;
        }
    }
    c->src = d[0];
    c->dst = d[1];
    c->llp = d[2];
    c->cyc = d[4] & CYC_MASK;
    c->ccr &= ~(CCR_MASK_TC | (7u << 11) | (7u << 8) | (3u << 5) |
                (3u << 3) | (3u << 1));
    c->ccr |= extract32(d[3], 16, 1) << 1 | extract32(d[3], 17, 1) << 2 |
              extract32(d[3], 18, 2) << 3 | extract32(d[3], 20, 2) << 5 |
              extract32(d[3], 22, 3) << 8 | extract32(d[3], 25, 3) << 11 |
              (extract32(d[3], 28, 1) ? CCR_MASK_TC : 0);
    return true;
}

/*
 * Run channel n as far as its requests allow.  CYC, SRC and DST track the
 * progress, so a paused transfer resumes where it stopped.
 */
static void gm_dmac_run_chan(GmDmacState *s, int n)
{
    GmDmacChan *c = &s->ch[n];

    while (c->ccr & CCR_ENABLE) {
        int sw = gm_dmac_width(extract32(c->ccr, 11, 3));
        int dw = gm_dmac_width(extract32(c->ccr, 8, 3));
        unsigned sctl = extract32(c->ccr, 5, 2);
        unsigned dctl = extract32(c->ccr, 3, 2);

        if (c->ccr & CCR_ABORT) {
            c->ccr &= ~(CCR_ENABLE | CCR_ABORT);
            c->cfg &= ~CFG_BUSY;
            s->ea_raw |= 1u << (16 + n);
            return;
        }
        while (c->cyc) {
            uint8_t buf[8];
            int chunk = MAX(sw, dw), i;
            uint32_t v;

            if (!gm_dmac_ready(s, c)) {
                return;
            }
            /* a short tail moves at the source width */
            if ((uint64_t)c->cyc * sw < chunk) {
                chunk = c->cyc * sw;
            }
            for (i = 0; i < chunk; i += sw) {
                if (!gm_dmac_ld(s, c->src, sw, &v)) {
                    gm_dmac_finish(s, n, true);
                    return;
                }
                stl_le_p(buf + i, v);   /* i + 4 <= 8 */
                c->src = gm_dmac_step(c->src, sctl, sw);
                c->cyc--;
            }
            for (i = 0; i < chunk; i += MIN(dw, chunk - i)) {
                int w = MIN(dw, chunk - i);

                if (!gm_dmac_st(s, c->dst, w, ldl_le_p(buf + i))) {
                    gm_dmac_finish(s, n, true);
                    return;
                }
                c->dst = gm_dmac_step(c->dst, dctl, w);
            }
        }
        /* block done: its terminal count, unless the block masks it */
        if (!(c->ccr & CCR_MASK_TC)) {
            s->tc_raw |= 1u << n;
        }
        if (!c->llp) {
            gm_dmac_finish(s, n, false);
            return;
        }
        if (!gm_dmac_next_desc(s, c)) {
            gm_dmac_finish(s, n, true);
            return;
        }
    }
}

static void gm_dmac_run(void *opaque)
{
    GmDmacState *s = opaque;
    int n;

    if (s->cr & CR_ENABLE) {
        for (n = 0; n < GM_DMAC_CHANS; n++) {
            gm_dmac_run_chan(s, n);
        }
    }
    gm_dmac_update(s);
}

static void gm_dmac_set_req(void *opaque, int n, int level)
{
    GmDmacState *s = opaque;

    if (level) {
        s->req |= 1u << n;
        qemu_bh_schedule(s->bh);
    } else {
        s->req &= ~(1u << n);
    }
}

static uint32_t gm_dmac_ch_enabled(GmDmacState *s)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < GM_DMAC_CHANS; i++) {
        if (s->ch[i].ccr & CCR_ENABLE) {
            v |= 1u << i;
        }
    }
    return v;
}

static uint64_t gm_dmac_read(void *opaque, hwaddr off, unsigned size)
{
    GmDmacState *s = opaque;
    GmDmacChan *c;

    switch (off) {
    case R_ISR: {
        uint32_t ea = gm_dmac_ea_status(s);

        return (gm_dmac_tc_status(s) | ea | ea >> 16) & 0xFF;
    }
    case R_TCISR:
        return gm_dmac_tc_status(s);
    case R_EAISR:
        return gm_dmac_ea_status(s);
    case R_TCRAW:
        return s->tc_raw;
    case R_EARAW:
        return s->ea_raw;
    case R_CH_EN:
    case R_CH_BUSY:
        return gm_dmac_ch_enabled(s);
    case R_CR:
        return s->cr;
    case R_SYNC:
        return s->sync;
    case R_REVISION:
        return 0x00011106;
    case R_FEATURE:
        /* 8 channels, linked lists, two AHB masters */
        return (GM_DMAC_CHANS << 12) | (1 << 8) | (1 << 9) | 4;
    }
    if (off >= CH_BASE && off < CH_BASE + GM_DMAC_CHANS * CH_STRIDE) {
        c = &s->ch[(off - CH_BASE) / CH_STRIDE];
        switch ((off - CH_BASE) % CH_STRIDE) {
        case C_CCR:
            return c->ccr;
        case C_CFG:
            return c->cfg;
        case C_SRC:
            return c->src;
        case C_DST:
            return c->dst;
        case C_LLP:
            return c->llp;
        case C_CYC:
            return c->cyc;
        }
    }
    return 0;
}

static void gm_dmac_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmDmacState *s = opaque;
    GmDmacChan *c;

    switch (off) {
    case R_TCICR:
        s->tc_raw &= ~val;
        gm_dmac_update(s);
        return;
    case R_EAICR:
        s->ea_raw &= ~val;
        gm_dmac_update(s);
        return;
    case R_CR:
        s->cr = val & 7;
        qemu_bh_schedule(s->bh);
        return;
    case R_SYNC:
        s->sync = val & 0xFF;
        return;
    }
    if (off < CH_BASE || off >= CH_BASE + GM_DMAC_CHANS * CH_STRIDE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, off);
        return;
    }
    c = &s->ch[(off - CH_BASE) / CH_STRIDE];
    switch ((off - CH_BASE) % CH_STRIDE) {
    case C_CCR:
        if (val & CCR_ABORT) {
            /*
             * Cyclic (audio) channels are stopped this way, idle or not:
             * the driver then waits for the abort status.
             */
            c->ccr = val & ~(CCR_ENABLE | CCR_ABORT);
            c->cfg &= ~CFG_BUSY;
            s->ea_raw |= 1u << (16 + ((off - CH_BASE) / CH_STRIDE));
            gm_dmac_update(s);
            break;
        }
        if (val & CCR_ENABLE) {
            c->cfg |= CFG_BUSY;
        }
        c->ccr = val;
        qemu_bh_schedule(s->bh);
        break;
    case C_CFG:
        c->cfg = (val & ~CFG_BUSY) | (c->cfg & CFG_BUSY);
        gm_dmac_update(s);
        break;
    case C_SRC:
        c->src = val;
        break;
    case C_DST:
        c->dst = val;
        break;
    case C_LLP:
        c->llp = val;
        break;
    case C_CYC:
        c->cyc = val & CYC_MASK;
        break;
    }
}

static const MemoryRegionOps gm_dmac_ops = {
    .read = gm_dmac_read,
    .write = gm_dmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_dmac_reset(DeviceState *dev)
{
    GmDmacState *s = GM_DMAC(dev);

    memset(s->ch, 0, sizeof(s->ch));
    s->cr = 0;
    s->sync = 0;
    s->tc_raw = 0;
    s->ea_raw = 0;
    gm_dmac_update(s);
}

static void gm_dmac_realize(DeviceState *dev, Error **errp)
{
    GmDmacState *s = GM_DMAC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_dmac_ops, s,
                          TYPE_GM_DMAC, GM_DMAC_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, gm_dmac_set_req, "dma-req", GM_DMAC_REQS);
    s->as = &address_space_memory;
    s->bh = aio_bh_new_guarded(qemu_get_aio_context(), gm_dmac_run, s,
                               &dev->mem_reentrancy_guard);
}

static const VMStateDescription vmstate_gm_dmac_chan = {
    .name = "gm-dmac-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ccr, GmDmacChan),
        VMSTATE_UINT32(cfg, GmDmacChan),
        VMSTATE_UINT32(src, GmDmacChan),
        VMSTATE_UINT32(dst, GmDmacChan),
        VMSTATE_UINT32(llp, GmDmacChan),
        VMSTATE_UINT32(cyc, GmDmacChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_gm_dmac = {
    .name = TYPE_GM_DMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, GmDmacState, GM_DMAC_CHANS, 1,
                             vmstate_gm_dmac_chan, GmDmacChan),
        VMSTATE_UINT32(cr, GmDmacState),
        VMSTATE_UINT32(sync, GmDmacState),
        VMSTATE_UINT32(tc_raw, GmDmacState),
        VMSTATE_UINT32(ea_raw, GmDmacState),
        VMSTATE_UINT32(req, GmDmacState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_dmac_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_dmac_realize;
    device_class_set_legacy_reset(dc, gm_dmac_reset);
    dc->vmsd = &vmstate_gm_dmac;
}

static const TypeInfo gm_dmac_info = {
    .name           = TYPE_GM_DMAC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmDmacState),
    .class_init     = gm_dmac_class_init,
};

static void gm_dmac_register_types(void)
{
    type_register_static(&gm_dmac_info);
}

type_init(gm_dmac_register_types)
