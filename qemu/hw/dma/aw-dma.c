/*
 * Allwinner sun8i/sun50i DMA controller, H6 register layout (V831/V833:
 * 16 channels at 0x03002000).
 *
 *   0x00/0x04 IRQ_EN0/1   per channel 4 bits: half, package, queue
 *   0x10/0x14 IRQ_PEND0/1 (write 1 to clear)
 *   0x30      STA         channel busy bits
 *   0x100 + 0x40 * ch:    EN, PAUSE, DESC_ADDR, CFG, CUR_SRC, CUR_DST,
 *                         BCNT_LEFT, PARA, ..., MODE (+0x28),
 *                         FDESC_ADDR (+0x2C), PKG_NUM (+0x30)
 *
 * A channel runs a chain of six-word descriptors in DRAM: cfg, src, dst,
 * length, para, next (0xFFFFF800 ends the chain).  cfg per side (source
 * in bits 15:0, destination in 31:16): DRQ port 5:0, burst 7:6, IO
 * (fixed-address) mode 8, width 10:9.  An IO-mode side is a device's
 * FIFO register, and the transfer waits for the device: its DRQ input
 * must be high for each unit to move.  A linear side is memory and needs
 * no handshake, whatever its port number (DRAM is port 1 on the V831).
 * A port number names a device both as a source and as a destination
 * (SPI0 RX and TX are both 22), so the inputs are two sets:
 * gpio "drq-src" (the device has data) and "drq-dst" (it has room).
 * Units are the device side's width; memory-to-memory moves the whole
 * length at once.  A DRQ that rises runs the channels from a bottom half:
 * it rises inside the device's own register access, and the transfer
 * reads that device again.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_AW_DMA "aw-dma"
OBJECT_DECLARE_SIMPLE_TYPE(AwDmaState, AW_DMA)

#define AW_DMA_SIZE         0x1000
#define AW_DMA_CHANS        16
#define AW_DMA_PORTS        64
#define AW_DMA_LINK_END     0xFFFFF800u

#define REG_IRQ_EN0         0x00
#define REG_IRQ_EN1         0x04
#define REG_IRQ_PEND0       0x10
#define REG_IRQ_PEND1       0x14
#define REG_SECU            0x20
#define REG_GATE            0x28
#define REG_STA             0x30
#define REG_VER             0x40
#define REG_CHAN            0x100

#define CH_EN               0x00
#define CH_PAUSE            0x04
#define CH_DESC             0x08
#define CH_CFG              0x0C
#define CH_SRC              0x10
#define CH_DST              0x14
#define CH_LEFT             0x18
#define CH_PARA             0x1C
#define CH_MODE             0x28
#define CH_FDESC            0x2C
#define CH_PKG_NUM          0x30

#define IRQ_HALF            1
#define IRQ_PKG             2
#define IRQ_QUEUE           4

typedef struct AwDmaChan {
    uint32_t en, pause, desc, mode;
    /* the descriptor being run */
    uint32_t cfg, src, dst, left, para, next, len;
    uint32_t fdesc, pkg_num;
    bool active;            /* a descriptor is loaded */
} AwDmaChan;

struct AwDmaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    AddressSpace *as;
    QEMUBH *bh;

    uint32_t irq_en[2], irq_pend[2], secu, gate;
    AwDmaChan ch[AW_DMA_CHANS];
    uint64_t drq[2];        /* DRQ levels, one bit per port: src, dst */
    bool running;
    bool rerun;
};

static void aw_dma_update_irq(AwDmaState *s)
{
    qemu_set_irq(s->irq, !!((s->irq_en[0] & s->irq_pend[0]) |
                            (s->irq_en[1] & s->irq_pend[1])));
}

static void aw_dma_raise(AwDmaState *s, int n, uint32_t bits)
{
    s->irq_pend[n / 8] |= bits << ((n % 8) * 4);
}

static bool aw_dma_load(AwDmaState *s, int n, uint32_t addr)
{
    AwDmaChan *c = &s->ch[n];
    uint32_t d[6];
    int i;

    if (dma_memory_read(s->as, addr, d, sizeof(d),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "aw-dma: ch%d: bad descriptor address 0x%08x\n",
                      n, addr);
        return false;
    }
    for (i = 0; i < 6; i++) {
        d[i] = le32_to_cpu(d[i]);
    }
    c->fdesc = addr;
    c->cfg = d[0];
    c->src = d[1];
    c->dst = d[2];
    c->len = c->left = d[3];
    c->para = d[4];
    c->next = d[5];
    c->desc = d[5];
    c->active = true;
    return true;
}

/* Move what the channel can; false when it has to wait for a DRQ. */
static bool aw_dma_step(AwDmaState *s, int n)
{
    AwDmaChan *c = &s->ch[n];
    uint32_t sport = c->cfg & 0x3F, dport = (c->cfg >> 16) & 0x3F;
    bool sio = c->cfg & (1u << 8), dio = c->cfg & (1u << 24);
    unsigned sw = 1u << ((c->cfg >> 9) & 3);
    unsigned dw = 1u << ((c->cfg >> 25) & 3);
    unsigned unit = sio ? sw : dio ? dw : 0;
    uint8_t buf[8];

    if (!unit) {
        /* SDRAM to SDRAM: all of it. */
        g_autofree uint8_t *tmp = g_malloc(c->left);

        dma_memory_read(s->as, c->src, tmp, c->left,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(s->as, c->dst, tmp, c->left,
                         MEMTXATTRS_UNSPECIFIED);
        c->src += c->left;
        c->dst += c->left;
        c->left = 0;
        return true;
    }
    while (c->left) {
        unsigned u = MIN(unit, c->left);

        if ((sio && !(s->drq[0] & (1ull << sport))) ||
            (dio && !(s->drq[1] & (1ull << dport)))) {
            return false;
        }
        address_space_read(s->as, c->src, MEMTXATTRS_UNSPECIFIED, buf, u);
        address_space_write(s->as, c->dst, MEMTXATTRS_UNSPECIFIED, buf, u);
        if (!sio) {
            c->src += u;
        }
        if (!dio) {
            c->dst += u;
        }
        c->left -= u;
    }
    return true;
}

static void aw_dma_run(AwDmaState *s)
{
    int n;

    if (s->running) {
        s->rerun = true;
        return;
    }
    s->running = true;
    do {
        s->rerun = false;
        for (n = 0; n < AW_DMA_CHANS; n++) {
            AwDmaChan *c = &s->ch[n];

            while (c->en && !c->pause) {
                if (!c->active) {
                    if (c->desc == AW_DMA_LINK_END ||
                        !aw_dma_load(s, n, c->desc)) {
                        c->en = 0;
                        break;
                    }
                }
                if (!aw_dma_step(s, n)) {
                    break;
                }
                c->active = false;
                c->pkg_num++;
                aw_dma_raise(s, n, IRQ_HALF | IRQ_PKG);
                if (c->next == AW_DMA_LINK_END) {
                    aw_dma_raise(s, n, IRQ_QUEUE);
                    c->desc = AW_DMA_LINK_END;
                    c->en = 0;
                } else {
                    c->desc = c->next;
                }
            }
        }
    } while (s->rerun);
    s->running = false;
    aw_dma_update_irq(s);
}

static void aw_dma_set_drq(AwDmaState *s, int side, int port, int level)
{
    uint64_t bit = 1ull << port;

    if (level) {
        if (!(s->drq[side] & bit)) {
            s->drq[side] |= bit;
            qemu_bh_schedule(s->bh);
        }
    } else {
        s->drq[side] &= ~bit;
    }
}

static void aw_dma_set_drq_src(void *opaque, int port, int level)
{
    aw_dma_set_drq(opaque, 0, port, level);
}

static void aw_dma_set_drq_dst(void *opaque, int port, int level)
{
    aw_dma_set_drq(opaque, 1, port, level);
}

static void aw_dma_bh(void *opaque)
{
    aw_dma_run(opaque);
}

static uint32_t aw_dma_sta(AwDmaState *s)
{
    uint32_t v = 0;
    int n;

    for (n = 0; n < AW_DMA_CHANS; n++) {
        if (s->ch[n].en) {
            v |= 1u << n;
        }
    }
    return v;
}

static uint64_t aw_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    AwDmaState *s = opaque;

    if (addr >= REG_CHAN && addr < REG_CHAN + AW_DMA_CHANS * 0x40) {
        AwDmaChan *c = &s->ch[(addr - REG_CHAN) / 0x40];

        switch ((addr - REG_CHAN) % 0x40) {
        case CH_EN:
            return c->en;
        case CH_PAUSE:
            return c->pause;
        case CH_DESC:
            return c->desc;
        case CH_CFG:
            return c->cfg;
        case CH_SRC:
            return c->src;
        case CH_DST:
            return c->dst;
        case CH_LEFT:
            return c->left;
        case CH_PARA:
            return c->para;
        case CH_MODE:
            return c->mode;
        case CH_FDESC:
            return c->fdesc;
        case CH_PKG_NUM:
            return c->pkg_num;
        }
        return 0;
    }
    switch (addr) {
    case REG_IRQ_EN0:
    case REG_IRQ_EN1:
        return s->irq_en[(addr - REG_IRQ_EN0) / 4];
    case REG_IRQ_PEND0:
    case REG_IRQ_PEND1:
        return s->irq_pend[(addr - REG_IRQ_PEND0) / 4];
    case REG_SECU:
        return s->secu;
    case REG_GATE:
        return s->gate;
    case REG_STA:
        return aw_dma_sta(s);
    }
    return 0;
}

static void aw_dma_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    AwDmaState *s = opaque;

    if (addr >= REG_CHAN && addr < REG_CHAN + AW_DMA_CHANS * 0x40) {
        AwDmaChan *c = &s->ch[(addr - REG_CHAN) / 0x40];

        switch ((addr - REG_CHAN) % 0x40) {
        case CH_EN:
            c->en = val & 1;
            if (c->en) {
                c->active = false;
                c->pkg_num = 0;
            }
            break;
        case CH_PAUSE:
            c->pause = val & 1;
            break;
        case CH_DESC:
            c->desc = val;
            break;
        case CH_MODE:
            c->mode = val;
            break;
        }
        aw_dma_run(s);
        return;
    }
    switch (addr) {
    case REG_IRQ_EN0:
    case REG_IRQ_EN1:
        s->irq_en[(addr - REG_IRQ_EN0) / 4] = val;
        break;
    case REG_IRQ_PEND0:
    case REG_IRQ_PEND1:
        s->irq_pend[(addr - REG_IRQ_PEND0) / 4] &= ~val;
        break;
    case REG_SECU:
        s->secu = val;
        break;
    case REG_GATE:
        s->gate = val;
        break;
    }
    aw_dma_update_irq(s);
}

static const MemoryRegionOps aw_dma_ops = {
    .read = aw_dma_read,
    .write = aw_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_dma_reset_hold(Object *obj, ResetType type)
{
    AwDmaState *s = AW_DMA(obj);

    s->irq_en[0] = s->irq_en[1] = 0;
    s->irq_pend[0] = s->irq_pend[1] = 0;
    s->secu = s->gate = 0;
    memset(s->ch, 0, sizeof(s->ch));
}

static void aw_dma_init(Object *obj)
{
    AwDmaState *s = AW_DMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &aw_dma_ops, s, TYPE_AW_DMA,
                          AW_DMA_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), aw_dma_set_drq_src, "drq-src",
                            AW_DMA_PORTS);
    qdev_init_gpio_in_named(DEVICE(obj), aw_dma_set_drq_dst, "drq-dst",
                            AW_DMA_PORTS);
    s->as = &address_space_memory;
    s->bh = qemu_bh_new(aw_dma_bh, s);
}

static void aw_dma_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_dma_reset_hold;
}

static const TypeInfo aw_dma_info = {
    .name           = TYPE_AW_DMA,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwDmaState),
    .instance_init  = aw_dma_init,
    .class_init     = aw_dma_class_init,
};

static void aw_dma_register_types(void)
{
    type_register_static(&aw_dma_info);
}

type_init(aw_dma_register_types)
