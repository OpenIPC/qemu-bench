/*
 * SigmaStar / MStar EMAC: 10/100 Ethernet MAC with its internal PHY.
 *
 * An Atmel AT91RM9200-style EMAC (register numbering G below as in the
 * AT91 manual) seen through three windows (vendor DTS emac0 node):
 *
 *   0x1F2A2000  0x800  RIU view: each 32-bit register G is two 16-bit RIU
 *                      words, bits 15:0 at +2G and 31:16 at +2G+4.  Every
 *                      access, whatever its width, carries one half;
 *                      bytes are byte 0/1 of a half.  U-Boot uses only
 *                      this view, low half first, so side effects fire on
 *                      the high-half write.  G >= 0x100 are MStar's
 *                      "JULIAN" extras.
 *   0x1F343C00  0x600  X32 view: register G as one 32-bit word at +2G;
 *                      the vendor kernel's view of G < 0x100.
 *   0x1F006200  0x600  the internal ("Albany") PHY, RIU banks 0x31-0x33:
 *                      PHY register N as the 16-bit RIU word at +4N,
 *                      beside its analog trims.  The kernel reads and
 *                      writes it there; U-Boot over MDIO.  The bank is
 *                      the RIU's storage ("riu" link) with the PHY's
 *                      fixed registers answered here and in sstar-riu,
 *                      so both paths see one PHY.
 *
 * Transmit: write the frame's bus address to TAR (0x0C), then its length
 * to TCR (0x10); TSR (0x14) reports every FIFO slot idle (0x1F98) and the
 * kernel's TX-queue count byte at 0x1F2A2290 reads 0.
 *
 * Receive: a ring of 8-byte descriptors at RBQP (0x18): { addr: bit 0
 * DONE, bit 1 WRAP, buffer from bit 2; size: length incl. the 4-byte FCS
 * in [10:0], bit 29 unicast, 30 multicast, 31 broadcast }.  The kernel
 * takes 4 off every length and drops anything under 64, so frames are
 * padded to 60 and an FCS appended.  Writing RBQP restarts at entry 0:
 * U-Boot rebuilds its ring on every eth_init.  RX interrupts are MStar's
 * "RX delay" interrupt: J104 (0x104) bit 7 enables it, J108 (0x108) bit
 * 15 is pending and clears on read; RCOM is never enabled in IER.
 *
 * ISR (0x24) clears on read (per half in the RIU view), IMR (0x30) reads
 * 1 for masked, IER/IDR (0x28/0x2C) unmask/mask.  MDIO through MAN (0x34)
 * reaches the internal PHY at address 0, the same registers as the
 * Albany window; SR (0x08) bit 2 (MDIO idle) is always set.  Statistics
 * (0x40-0x84) read 0: the kernel accumulates them.  DMA addresses are MIU
 * bus addresses, DRAM from 0.
 *
 * Sources: the vendor U-Boot (drivers/mstar/emac, ALBANY internal PHY,
 * SOFTWARE_DESCRIPTOR) and kernel 4.9 (drivers/sstar/emac, hal/
 * infinity2m, drivers/sstar/netphy) drivers.  The register core follows
 * Daniel Palmer's mstar_emac.c (github.com/fifteenhex/qemu,
 * mstar-miyoo-working), written against mainline's macb driver.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "net/eth.h"
#include "hw/net/mii.h"
#include "hw/misc/sstar-riu.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include <zlib.h>

#define TYPE_SSTAR_EMAC "sstar-emac"
OBJECT_DECLARE_SIMPLE_TYPE(SstarEmacState, SSTAR_EMAC)

#define EMAC_RIU_SIZE       0x800
#define EMAC_X32_SIZE       0x600
#define EMAC_NREGS          (EMAC_RIU_SIZE / 8)     /* G / 4 */
#define EMAC_PHY_BANK       0x0031      /* ALBANY0, 0x1F006200 */

#define DRAM_BASE           0x20000000

/* register G */
#define G_CTL               0x00
#define G_CFG               0x04
#define G_SR                0x08
#define G_TAR               0x0C
#define G_TCR               0x10
#define G_TSR               0x14
#define G_RBQP              0x18
#define G_RSR               0x20
#define G_ISR               0x24
#define G_IER               0x28
#define G_IDR               0x2C
#define G_IMR               0x30
#define G_MAN               0x34
#define G_RBRP              0x38
#define G_STATS_FIRST       0x40
#define G_STATS_LAST        0x84
#define G_J104              0x104
#define G_J108              0x108
#define G_TXQ               0x148       /* byte 0x1F2A2290: queued TX */

#define CTL_RE              (1u << 2)
#define SR_IDLE             (1u << 2)
#define TSR_ALL_FREE        0x1F98
#define RSR_REC             (1u << 1)
#define RSR_BNA             (1u << 3)
#define ISR_RCOM            (1u << 1)
#define ISR_RBNA            (1u << 2)
#define ISR_TCOM            (1u << 7)
#define J104_RXDLY_EN       (1u << 7)
#define J108_RXDLY          (1u << 15)

#define RXD_DONE            (1u << 0)
#define RXD_WRAP            (1u << 1)
#define RXD_UNICAST         (1u << 29)
#define RXD_MULTICAST       (1u << 30)
#define RXD_BROADCAST       (1u << 31)
#define RX_BUF_MAX          0x600
#define RX_RING_MAX         1024

struct SstarEmacState {
    SysBusDevice parent_obj;

    MemoryRegion riu;
    MemoryRegion x32;
    NICState *nic;
    NICConf conf;
    qemu_irq irq;

    uint32_t regs[EMAC_NREGS];
    SstarRiuState *phy_store;   /* the RIU storage behind ALBANY0 */
    uint32_t rx_idx;
    uint32_t pend_g;
    uint16_t pend_lo;
};

/* ---- PHY ------------------------------------------------------------ */

static uint16_t emac_phy_read(SstarEmacState *s, unsigned reg)
{
    /* The fixed registers are answered by sstar-riu's status hook too. */
    return sstar_riu_read16(s->phy_store, EMAC_PHY_BANK, reg);
}

static void emac_phy_write(SstarEmacState *s, unsigned reg, uint16_t v)
{
    if (reg == MII_BMCR) {
        v &= ~(MII_BMCR_RESET | MII_BMCR_ANRESTART);
    }
    sstar_riu_set(s->phy_store, EMAC_PHY_BANK, reg, v);
}

/* ---- interrupts, RX --------------------------------------------------- */

static void emac_update_irq(SstarEmacState *s)
{
    bool level = (s->regs[G_ISR / 4] & ~s->regs[G_IMR / 4]) ||
                 ((s->regs[G_J108 / 4] & J108_RXDLY) &&
                  (s->regs[G_J104 / 4] & J104_RXDLY_EN));

    qemu_set_irq(s->irq, level);
}

static hwaddr emac_desc_addr(SstarEmacState *s, uint32_t idx)
{
    return DRAM_BASE + (s->regs[G_RBQP / 4] & 0x1FFFFFF8) + idx * 8;
}

static bool emac_can_receive(NetClientState *nc)
{
    SstarEmacState *s = qemu_get_nic_opaque(nc);
    uint32_t addr;

    if (!(s->regs[G_CTL / 4] & CTL_RE) || !s->regs[G_RBQP / 4]) {
        return false;
    }
    if (dma_memory_read(&address_space_memory, emac_desc_addr(s, s->rx_idx),
                        &addr, 4, MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    return !(le32_to_cpu(addr) & RXD_DONE);
}

static ssize_t emac_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    SstarEmacState *s = qemu_get_nic_opaque(nc);
    uint8_t frame[RX_BUF_MAX];
    hwaddr da = emac_desc_addr(s, s->rx_idx);
    uint32_t desc[2], len, flags, crc;

    if (!(s->regs[G_CTL / 4] & CTL_RE)) {
        return size;            /* receiver off: dropped on the wire */
    }
    if (dma_memory_read(&address_space_memory, da, desc, 8,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return size;
    }
    desc[0] = le32_to_cpu(desc[0]);
    if (desc[0] & RXD_DONE) {
        /* Ring full: the frame waits, the driver hears about it. */
        s->regs[G_RSR / 4] |= RSR_BNA;
        s->regs[G_ISR / 4] |= ISR_RBNA;
        emac_update_irq(s);
        return 0;
    }

    len = MIN(size, RX_BUF_MAX - 4);
    memcpy(frame, buf, len);
    if (len < ETH_ZLEN) {
        memset(frame + len, 0, ETH_ZLEN - len);
        len = ETH_ZLEN;
    }
    crc = cpu_to_le32(crc32(0, frame, len));
    memcpy(frame + len, &crc, 4);
    len += 4;
    dma_memory_write(&address_space_memory,
                     DRAM_BASE + (desc[0] & 0x1FFFFFFC), frame, len,
                     MEMTXATTRS_UNSPECIFIED);

    if (is_broadcast_ether_addr(buf)) {
        flags = RXD_BROADCAST;
    } else if (is_multicast_ether_addr(buf)) {
        flags = RXD_MULTICAST;
    } else {
        flags = RXD_UNICAST;
    }
    desc[1] = cpu_to_le32(len | flags);
    desc[0] = cpu_to_le32(desc[0] | RXD_DONE);
    dma_memory_write(&address_space_memory, da + 4, &desc[1], 4,
                     MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, da, &desc[0], 4,
                     MEMTXATTRS_UNSPECIFIED);

    if ((le32_to_cpu(desc[0]) & RXD_WRAP) || s->rx_idx + 1 >= RX_RING_MAX) {
        s->rx_idx = 0;
    } else {
        s->rx_idx++;
    }
    s->regs[G_RSR / 4] |= RSR_REC;
    s->regs[G_ISR / 4] |= ISR_RCOM;
    s->regs[G_J108 / 4] |= J108_RXDLY;
    emac_update_irq(s);
    return size;
}

static void emac_flush_rx(SstarEmacState *s)
{
    if (emac_can_receive(qemu_get_queue(s->nic))) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

/* ---- TX, MDIO -------------------------------------------------------- */

static void emac_transmit(SstarEmacState *s)
{
    uint8_t frame[2048];
    uint32_t len = s->regs[G_TCR / 4] & 0x7FF;

    if (len) {
        dma_memory_read(&address_space_memory,
                        DRAM_BASE + (s->regs[G_TAR / 4] & 0x1FFFFFFF),
                        frame, len, MEMTXATTRS_UNSPECIFIED);
        qemu_send_packet(qemu_get_queue(s->nic), frame, len);
    }
    s->regs[G_ISR / 4] |= ISR_TCOM;
    emac_update_irq(s);
}

static void emac_mdio(SstarEmacState *s)
{
    uint32_t man = s->regs[G_MAN / 4];
    unsigned op = (man >> 28) & 3;
    unsigned phy = (man >> 23) & 0x1F;
    unsigned reg = (man >> 18) & 0x1F;

    if (op == 2) {
        uint16_t v = phy == 0 ? emac_phy_read(s, reg) : 0xFFFF;

        s->regs[G_MAN / 4] = (man & 0xFFFF0000) | v;
    } else if (op == 1 && phy == 0) {
        emac_phy_write(s, reg, man & 0xFFFF);
    }
}

/* ---- the register file ------------------------------------------------ */

/* A read of register G, @mask the bits the access covers. */
static uint32_t emac_reg_read(SstarEmacState *s, uint32_t g, uint32_t mask)
{
    uint32_t *r = &s->regs[g / 4];
    uint32_t v;

    switch (g) {
    case G_SR:
        return *r | SR_IDLE;
    case G_TSR:
        return TSR_ALL_FREE;
    case G_ISR:
        v = *r;
        *r &= ~mask;            /* read to clear */
        emac_update_irq(s);
        emac_flush_rx(s);
        return v;
    case G_J108:
        v = *r;
        *r &= ~(mask & J108_RXDLY);
        emac_update_irq(s);
        emac_flush_rx(s);
        return v;
    case G_TXQ:
        return *r & ~0x7F;      /* nothing pending in the TX queue */
    case G_STATS_FIRST ... G_STATS_LAST:
        return 0;
    default:
        return *r;
    }
}

/* A write of register G whose last part has just landed. */
static void emac_reg_commit(SstarEmacState *s, uint32_t g, uint32_t v)
{
    uint32_t *r = &s->regs[g / 4];

    switch (g) {
    case G_TSR:
    case G_ISR:
    case G_IMR:
        break;
    case G_RSR:
        *r &= ~v;               /* write 1 to clear */
        emac_flush_rx(s);       /* the driver frees descriptors, then this */
        break;
    case G_IER:
        s->regs[G_IMR / 4] &= ~v;
        emac_update_irq(s);
        emac_flush_rx(s);
        break;
    case G_IDR:
        s->regs[G_IMR / 4] |= v;
        emac_update_irq(s);
        break;
    case G_TCR:
        *r = v;
        emac_transmit(s);
        break;
    case G_MAN:
        *r = v;
        emac_mdio(s);
        break;
    case G_RBQP:
        *r = v;
        s->rx_idx = 0;
        emac_flush_rx(s);
        break;
    case G_CTL:
    case G_RBRP:
    case G_J104:
        *r = v;
        emac_update_irq(s);
        emac_flush_rx(s);
        break;
    default:
        *r = v;
        break;
    }
}

/* Registers written as commands rather than stored. */
static bool emac_is_command(uint32_t g)
{
    switch (g) {
    case G_TCR:
    case G_MAN:
    case G_IER:
    case G_IDR:
    case G_RSR:
    case G_ISR:
        return true;
    default:
        return false;
    }
}

/*
 * The RIU view: 16-bit word w is half (w & 1) of register G = (w / 2) * 4.
 * Byte accesses reach byte (off & 1) of a word; bytes 2 and 3 of each
 * 4-byte slot are not there.
 */
static uint64_t emac_riu_read(void *opaque, hwaddr off, unsigned size)
{
    SstarEmacState *s = opaque;
    uint32_t w = off >> 2, g = (w >> 1) * 4, shift = (w & 1) * 16;
    uint32_t half;

    if ((off & 3) >= 2) {
        return 0;
    }
    half = (emac_reg_read(s, g, 0xFFFFu << shift) >> shift) & 0xFFFF;
    if (off & 1) {
        return half >> 8;
    }
    return size == 1 ? half & 0xFF : half;
}

static void emac_riu_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    SstarEmacState *s = opaque;
    uint32_t w = off >> 2, g = (w >> 1) * 4, shift = (w & 1) * 16;
    uint32_t cur = s->regs[g / 4], half = (cur >> shift) & 0xFFFF;
    uint32_t v;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        half = (off & 1) ? (half & 0x00FF) | ((val & 0xFF) << 8)
                         : (half & 0xFF00) | (val & 0xFF);
    } else {
        half = val & 0xFFFF;
    }
    if (emac_is_command(g)) {
        /*
         * Command registers act on the whole word: the low half waits for
         * its high half, which U-Boot always writes second.
         */
        if (!(w & 1)) {
            s->pend_g = g;
            s->pend_lo = half;
            return;
        }
        v = (s->pend_g == g ? s->pend_lo : 0) | (half << 16);
        s->pend_g = UINT32_MAX;
        emac_reg_commit(s, g, v);
        return;
    }
    emac_reg_commit(s, g, (cur & ~(0xFFFFu << shift)) | (half << shift));
}

static const MemoryRegionOps emac_riu_ops = {
    .read = emac_riu_read,
    .write = emac_riu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* The X32 view: register G as a 32-bit word at 2G. */
static uint64_t emac_x32_read(void *opaque, hwaddr off, unsigned size)
{
    SstarEmacState *s = opaque;

    if (off & 7) {
        return 0;
    }
    return emac_reg_read(s, off / 2, 0xFFFFFFFF);
}

static void emac_x32_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    SstarEmacState *s = opaque;

    if (off & 7) {
        return;
    }
    emac_reg_commit(s, off / 2, val);
}

static const MemoryRegionOps emac_x32_ops = {
    .read = emac_x32_read,
    .write = emac_x32_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static NetClientInfo net_sstar_emac_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = emac_can_receive,
    .receive = emac_receive,
};

static void sstar_emac_reset(DeviceState *dev)
{
    SstarEmacState *s = SSTAR_EMAC(dev);
    const uint8_t *m = s->conf.macaddr.a;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[G_IMR / 4] = 0xFFFFFFFF;
    /* SA1, as a boot loader leaves it; the kernel reads it back. */
    s->regs[0x98 / 4] = m[0] | (m[1] << 8) | (m[2] << 16) |
                        ((uint32_t)m[3] << 24);
    s->regs[0x9C / 4] = m[4] | (m[5] << 8);
    s->rx_idx = 0;
    s->pend_g = UINT32_MAX;
    qemu_set_irq(s->irq, 0);
}

static void sstar_emac_realize(DeviceState *dev, Error **errp)
{
    SstarEmacState *s = SSTAR_EMAC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (!s->phy_store) {
        error_setg(errp, "sstar-emac: the riu link is not set");
        return;
    }

    memory_region_init_io(&s->riu, OBJECT(s), &emac_riu_ops, s,
                          "sstar-emac.riu", EMAC_RIU_SIZE);
    memory_region_init_io(&s->x32, OBJECT(s), &emac_x32_ops, s,
                          "sstar-emac.x32", EMAC_X32_SIZE);
    sysbus_init_mmio(sbd, &s->riu);
    sysbus_init_mmio(sbd, &s->x32);
    sysbus_init_irq(sbd, &s->irq);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_sstar_emac_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static const Property sstar_emac_props[] = {
    DEFINE_NIC_PROPERTIES(SstarEmacState, conf),
    DEFINE_PROP_LINK("riu", SstarEmacState, phy_store, TYPE_SSTAR_RIU,
                     SstarRiuState *),
};

static const VMStateDescription vmstate_sstar_emac = {
    .name = TYPE_SSTAR_EMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, SstarEmacState, EMAC_NREGS),
        VMSTATE_UINT32(rx_idx, SstarEmacState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_emac_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_emac_realize;
    device_class_set_legacy_reset(dc, sstar_emac_reset);
    device_class_set_props(dc, sstar_emac_props);
    dc->vmsd = &vmstate_sstar_emac;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo sstar_emac_info = {
    .name           = TYPE_SSTAR_EMAC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarEmacState),
    .class_init     = sstar_emac_class_init,
};

static void sstar_emac_register_types(void)
{
    type_register_static(&sstar_emac_info);
}

type_init(sstar_emac_register_types)
