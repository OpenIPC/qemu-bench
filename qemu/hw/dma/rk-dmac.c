/*
 * Rockchip DMA controller ("rockchip,dma": RV1103B / RV1106B at
 * 0x20740000, RV1126B, RK3576), the linked-list engine that replaces the
 * PL330 on the newer dies.
 *
 * Register file, from the vendor kernel's drivers/dma/rockchip-dma.c
 * (develop-5.10).  Control registers with a "hiword" note take a write
 * mask in bits 31:16.
 *
 *   0x000 CMN_VER      major 31:16, minor 15:0
 *   0x004 CMN_CFG      hiword: bit 0 enable, bit 1 reset, bit 2 IRQ enable
 *   0x018 CMN_IS0 / 0x01c CMN_IS1   one bit per logical channel, W1C
 *   0x030 CMN_CAP0     logical channels - 1 in 5:0, physical - 1 in 11:6
 *   0x034 CMN_CAP1     log2 AXI bytes in 2:0, burst length - 1 in 10:3
 *   0x040 + n * 0x40   logical channel n ("LCH"; the request number):
 *         +0x00 CTL (hiword, bit 0 enable)   +0x04 BIND
 *         +0x08 CMDBA: the command entry     +0x0c TRF_CMD (hiword):
 *               bit 0 start, bit 1 kill
 *         +0x14 IS (W1C) / +0x18 IE: bit 0 DMA done, bit 1 block done
 *   then the physical channels, 0x40 each: a register file here.
 *
 * Descriptors are 64 bytes: TRF_CTL0 (bit 1 last, bit 3 interrupt on
 * completion), TRF_CTL1 (bit 0 read address increments, bit 16 write
 * address increments, 4:2 element size as log2 bytes), SAR, DAR,
 * BLOCK_TS (bytes), LLP_NXT, LLP_CUR, TRF_CFG (9:8 direction:
 * 0 mem-to-mem, 1 mem-to-dev, 2 dev-to-mem) ...  CMDBA points at a
 * command entry whose LLP_NXT is the first block; the engine writes the
 * current SAR / DAR back into it, which is how the driver reads the
 * residue.  A cyclic transfer's last LLP_NXT leads back to the first
 * block.
 *
 * A block moves at once: memory runs straight, a fixed (non-incrementing)
 * address side is read or written element by element, so a peripheral
 * FIFO sees every element.  Completions are timed: memory-to-memory
 * blocks at "mem-rate" bytes per microsecond, blocks to or from a
 * peripheral at "dev-rate" bytes per second -- the request pacing of a
 * peripheral that is not modelled (default 192000: 48 kHz stereo 16-bit
 * audio), without which a cyclic audio buffer would spin.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_RK_DMAC "rk-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(RkDmacState, RK_DMAC)

#define NLCH            16
#define NPCH            8
#define REGS_SIZE       0x2000
#define GRP             0x40
#define LCH_BASE        GRP
#define PCH_BASE        (GRP + NLCH * GRP)

#define R_VER           0x00
#define R_CFG           0x04
#define R_IS0           0x18
#define R_IS1           0x1c
#define R_CAP0          0x30
#define R_CAP1          0x34

#define L_CTL           0x00
#define L_BIND          0x04
#define L_CMDBA         0x08
#define L_TRF_CMD       0x0c
#define L_TRF_PARA      0x10
#define L_IS            0x14
#define L_IE            0x18
#define L_DBG           0x1c

#define CFG_IE          (1u << 2)
#define CMD_START       (1u << 0)
#define CMD_KILL        (1u << 1)
#define IS_DMA_DONE     (1u << 0)
#define IS_BLOCK_DONE   (1u << 1)
#define IS_LLI_RD_ERR   (1u << 10)
#define CTL0_LAST       (1u << 1)
#define CTL0_IOC        (1u << 3)
#define CTL1_ARINCR     (1u << 0)
#define CTL1_AWINCR     (1u << 16)

typedef struct RkDmacChan {
    uint32_t regs[GRP / 4];
    uint32_t cur;           /* the block in flight, 0: idle */
    QEMUTimer *timer;
    struct RkDmacState *s;
    int n;
} RkDmacChan;

struct RkDmacState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t cfg;
    uint64_t is;
    RkDmacChan lch[NLCH];
    uint32_t pch[NPCH][GRP / 4];
    uint32_t mem_rate;      /* bytes per microsecond */
    uint32_t dev_rate;      /* bytes per second */
};

static void rk_dmac_update_irq(RkDmacState *s)
{
    qemu_set_irq(s->irq, (s->cfg & CFG_IE) && s->is);
}

static void rk_dmac_chan_irq(RkDmacState *s, RkDmacChan *c, uint32_t bits)
{
    c->regs[L_IS / 4] |= bits;
    if (c->regs[L_IS / 4] & c->regs[L_IE / 4]) {
        s->is |= 1ull << c->n;
    }
    rk_dmac_update_irq(s);
}

/* Move one block now; return its length, or -1 on a bad descriptor. */
static int64_t rk_dmac_block(RkDmacState *s, RkDmacChan *c, uint32_t d[16])
{
    AddressSpace *as = &address_space_memory;
    uint32_t len = d[4] & 0x1ffffff;
    uint32_t ctl1 = d[1];
    int width = 1 << ((ctl1 >> 2) & 7);
    hwaddr src = d[2], dst = d[3];
    uint32_t cmd = c->regs[L_CMDBA / 4];
    uint32_t pos[2];

    if (width > 16) {
        width = 16;
    }
    if ((ctl1 & CTL1_ARINCR) && (ctl1 & CTL1_AWINCR)) {
        g_autofree uint8_t *buf = g_malloc(len);

        dma_memory_read(as, src, buf, len, MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(as, dst, buf, len, MEMTXATTRS_UNSPECIFIED);
    } else {
        uint8_t e[16];
        uint32_t i;

        for (i = 0; i + width <= len; i += width) {
            dma_memory_read(as, (ctl1 & CTL1_ARINCR) ? src + i : src, e,
                            width, MEMTXATTRS_UNSPECIFIED);
            dma_memory_write(as, (ctl1 & CTL1_AWINCR) ? dst + i : dst, e,
                             width, MEMTXATTRS_UNSPECIFIED);
        }
    }
    /* the command entry tracks the position: the driver's residue */
    pos[0] = cpu_to_le32((ctl1 & CTL1_ARINCR) ? src + len : src);
    pos[1] = cpu_to_le32((ctl1 & CTL1_AWINCR) ? dst + len : dst);
    dma_memory_write(as, cmd + 8, pos, 8, MEMTXATTRS_UNSPECIFIED);
    return len;
}

static bool rk_dmac_read_lli(uint32_t addr, uint32_t d[16])
{
    int i;

    if (!addr || dma_memory_read(&address_space_memory, addr, d, 64,
                                 MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    for (i = 0; i < 16; i++) {
        d[i] = le32_to_cpu(d[i]);
    }
    return true;
}

/* Run the block at c->cur and time its completion. */
static void rk_dmac_run(RkDmacState *s, RkDmacChan *c)
{
    uint32_t d[16];
    int64_t len, ns;
    bool dev;

    if (!rk_dmac_read_lli(c->cur, d)) {
        c->cur = 0;
        rk_dmac_chan_irq(s, c, IS_LLI_RD_ERR);
        return;
    }
    len = rk_dmac_block(s, c, d);
    dev = (d[7] >> 8) & 3;
    if (dev) {
        ns = len * NANOSECONDS_PER_SECOND / MAX(s->dev_rate, 1);
    } else {
        ns = len * 1000 / MAX(s->mem_rate, 1);
    }
    timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns + 1000);
}

static void rk_dmac_block_done(void *opaque)
{
    RkDmacChan *c = opaque;
    RkDmacState *s = c->s;
    uint32_t d[16];

    if (!c->cur || !rk_dmac_read_lli(c->cur, d)) {
        return;
    }
    if (d[0] & CTL0_IOC) {
        rk_dmac_chan_irq(s, c, IS_BLOCK_DONE);
    }
    if ((d[0] & CTL0_LAST) || !d[5]) {
        c->cur = 0;
        rk_dmac_chan_irq(s, c, IS_DMA_DONE);
        return;
    }
    c->cur = d[5];
    rk_dmac_run(s, c);
}

static void rk_dmac_start(RkDmacState *s, RkDmacChan *c)
{
    uint32_t cmd[16];

    if (!rk_dmac_read_lli(c->regs[L_CMDBA / 4], cmd)) {
        rk_dmac_chan_irq(s, c, IS_LLI_RD_ERR);
        return;
    }
    c->cur = cmd[5];            /* the command entry's LLP_NXT */
    rk_dmac_run(s, c);
}

static uint32_t hiword(uint32_t old, uint64_t val)
{
    uint32_t mask = val >> 16;

    return (old & ~mask) | (val & mask);
}

static uint64_t rk_dmac_read(void *opaque, hwaddr off, unsigned size)
{
    RkDmacState *s = opaque;

    if (off >= PCH_BASE && off < PCH_BASE + NPCH * GRP) {
        return s->pch[(off - PCH_BASE) / GRP][(off % GRP) / 4];
    }
    if (off >= LCH_BASE && off < PCH_BASE) {
        RkDmacChan *c = &s->lch[(off - LCH_BASE) / GRP];

        if ((off % GRP) == L_DBG) {
            return c->cur ? 4 : 0;          /* DMA_LCH_TRANS / idle */
        }
        return c->regs[(off % GRP) / 4];
    }
    switch (off) {
    case R_VER:
        return 0x00010000;
    case R_CFG:
        return s->cfg;
    case R_IS0:
        return s->is;
    case R_IS1:
        return s->is >> 32;
    case R_CAP0:
        return (NLCH - 1) | (NPCH - 1) << 6;
    case R_CAP1:
        return 3 | (16 - 1) << 3;           /* 8-byte AXI, bursts of 16 */
    }
    return 0;
}

static void rk_dmac_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    RkDmacState *s = opaque;

    if (off >= PCH_BASE && off < PCH_BASE + NPCH * GRP) {
        uint32_t *r = &s->pch[(off - PCH_BASE) / GRP][(off % GRP) / 4];

        *r = (off % GRP) == 0 ? hiword(*r, val) : val;
        return;
    }
    if (off >= LCH_BASE && off < PCH_BASE) {
        RkDmacChan *c = &s->lch[(off - LCH_BASE) / GRP];

        switch (off % GRP) {
        case L_CTL:
            c->regs[L_CTL / 4] = hiword(c->regs[L_CTL / 4], val);
            if (!(c->regs[L_CTL / 4] & 1)) {
                timer_del(c->timer);
                c->cur = 0;
            }
            break;
        case L_TRF_CMD: {
            uint32_t set = val & (val >> 16);   /* bits written as 1 */

            c->regs[L_TRF_CMD / 4] = hiword(c->regs[L_TRF_CMD / 4], val);
            if (set & CMD_KILL) {
                timer_del(c->timer);
                c->cur = 0;
            } else if (set & CMD_START) {
                timer_del(c->timer);
                rk_dmac_start(s, c);
            }
            c->regs[L_TRF_CMD / 4] &= ~(CMD_START | CMD_KILL);
            break;
        }
        case L_IS:
            c->regs[L_IS / 4] &= ~val;
            break;
        default:
            c->regs[(off % GRP) / 4] = val;
            break;
        }
        return;
    }
    switch (off) {
    case R_CFG:
        s->cfg = hiword(s->cfg, val) & ~2u;     /* reset self-clears */
        break;
    case R_IS0:
        s->is &= ~(uint64_t)(uint32_t)val;
        break;
    case R_IS1:
        s->is &= ~((uint64_t)val << 32);
        break;
    default:
        break;
    }
    rk_dmac_update_irq(s);
}

static const MemoryRegionOps rk_dmac_ops = {
    .read = rk_dmac_read,
    .write = rk_dmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_dmac_reset(DeviceState *dev)
{
    RkDmacState *s = RK_DMAC(dev);
    int i;

    s->cfg = 0;
    s->is = 0;
    memset(s->pch, 0, sizeof(s->pch));
    for (i = 0; i < NLCH; i++) {
        timer_del(s->lch[i].timer);
        memset(s->lch[i].regs, 0, sizeof(s->lch[i].regs));
        s->lch[i].cur = 0;
    }
}

static void rk_dmac_init(Object *obj)
{
    RkDmacState *s = RK_DMAC(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &rk_dmac_ops, s, TYPE_RK_DMAC,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    for (i = 0; i < NLCH; i++) {
        s->lch[i].s = s;
        s->lch[i].n = i;
        s->lch[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       rk_dmac_block_done, &s->lch[i]);
    }
}

static const Property rk_dmac_properties[] = {
    DEFINE_PROP_UINT32("mem-rate", RkDmacState, mem_rate, 400),
    DEFINE_PROP_UINT32("dev-rate", RkDmacState, dev_rate, 192000),
};

static const VMStateDescription rk_dmac_chan_vmstate = {
    .name = "rk-dmac-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkDmacChan, GRP / 4),
        VMSTATE_UINT32(cur, RkDmacChan),
        VMSTATE_TIMER_PTR(timer, RkDmacChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription rk_dmac_vmstate = {
    .name = TYPE_RK_DMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, RkDmacState),
        VMSTATE_UINT64(is, RkDmacState),
        VMSTATE_STRUCT_ARRAY(lch, RkDmacState, NLCH, 1,
                             rk_dmac_chan_vmstate, RkDmacChan),
        VMSTATE_UINT32_2DARRAY(pch, RkDmacState, NPCH, GRP / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_dmac_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_dmac_reset);
    device_class_set_props(dc, rk_dmac_properties);
    dc->vmsd = &rk_dmac_vmstate;
}

static const TypeInfo rk_dmac_info = {
    .name = TYPE_RK_DMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkDmacState),
    .instance_init = rk_dmac_init,
    .class_init = rk_dmac_class_init,
};

static void rk_dmac_register_types(void)
{
    type_register_static(&rk_dmac_info);
}

type_init(rk_dmac_register_types)
