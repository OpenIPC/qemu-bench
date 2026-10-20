/*
 * Novatek NA51055 / NA51089 Ethernet: a Synopsys DWC Ethernet QoS (GMAC
 * 4.x) core with Novatek glue and an embedded PHY.
 *
 * drivers/net/ethernet/novatek/na51089 (the Synopsys "DWC_ETH_QOS"
 * reference driver, register offsets in DWC_ETH_QOS_yregacc.h) and
 * U-Boot's eth_na51089.c.  The block is 16 KiB:
 *
 *   0x0000 MAC   0x00 MCR (bit 0 RE, bit 1 TE), 0x08 packet filter,
 *                0xB0/0xB4 interrupt status/mask, 0x110 version,
 *                0x11C..0x124 HW features: MII, MDIO (HFR0 bit 5, without
 *                which the driver skips its PHY), no checksum offload,
 *                HFR2 = 0: one queue each way,
 *                0x200/0x204 MDIO address/data, 0x300/0x304 MAC address
 *   0x0C00 MTL   operation modes, queue debug (empty queues read 0)
 *   0x1000 DMA   0x00 mode (bit 0 software reset), 0x08 interrupt
 *                summary (bit 0 channel 0), 0x0C debug status 0 (RPS0
 *                [11:8], TPS0 [15:12]); channel 0 from 0x1100: control
 *                (DSL [20:18]), Tx control (bit 0 ST), Rx control (bit 0
 *                SR, RBSZ [14:1]), ring bases 0x14/0x1C, tail pointers
 *                0x20/0x28, ring lengths 0x2C/0x30, interrupt enable 0x34,
 *                current descriptors 0x44/0x4C, status 0x60 (write 1 to
 *                clear)
 *   0x3000       Novatek glue (RMII / clock selection): storage, but for
 *                0x3008 ETH_ISR, the core's interrupt line in bit 0 --
 *                the ISR returns IRQ_NONE unless its low 6 bits are set
 *   0x3800       embedded PHY analog block (power-on sequence): storage
 *
 * Descriptors are four words, OWN in DES3 bit 31.  Tx: TDES0 buffer 1,
 * TDES1 buffer 2, TDES2 lengths [13:0] / [29:16] and IOC bit 31, TDES3 FD
 * bit 29, LD bit 28, CTXT bit 30 (context descriptors are skipped); on
 * write-back TDES3 keeps FD / LD only, its low half being error flags.  Rx
 * read format: RDES0 buffer, RDES3 OWN, IOC bit 30; write-back: RDES3 FD,
 * LD, packet length [14:0] -- without the FCS, which the driver does not
 * strip (DWC_ETH_QOS_clean_rx_irq).  yinit() polls DSR0 until RPS0 reads
 * 3 (running, waiting for a packet) and re-initialises the core until it
 * does.
 *
 * The PHY is a generic 100 Mb/s MII PHY at MDIO address 1, link up: the
 * driver scans for the first address whose BMSR is neither 0 nor 0xFFFF
 * and hands it to phylib.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "net/eth.h"
#include "hw/net/mii.h"
#include "qom/object.h"
#include "system/dma.h"

#define TYPE_NVT_EQOS "nvt-eqos"
OBJECT_DECLARE_SIMPLE_TYPE(NvtEqosState, NVT_EQOS)

#define EQOS_SIZE           0x4000

#define MAC_MCR             0x0000
#define MAC_ISR             0x00B0
#define MAC_VR              0x0110
#define MAC_HFR0            0x011C
#define MAC_HFR1            0x0120
#define MAC_HFR2            0x0124
#define MAC_MDIO_ADDR       0x0200
#define MAC_MDIO_DATA       0x0204
#define MAC_ADDR0_HI        0x0300
#define MAC_ADDR0_LO        0x0304
#define MTL_Q0TOMR          0x0D00
#define DMA_MODE            0x1000
#define DMA_ISR             0x1008
#define DMA_DSR0            0x100C
#define DMA_CH0             0x1100
#define DMA_CH0_END         0x1180
#define ETH_ISR             0x3008      /* Novatek glue: interrupt summary */

/* channel 0, relative to DMA_CH0 */
#define CH_CR               0x00
#define CH_TCR              0x04
#define CH_RCR              0x08
#define CH_TDLAR            0x14
#define CH_RDLAR            0x1C
#define CH_TDTP             0x20
#define CH_RDTP             0x28
#define CH_TDRLR            0x2C
#define CH_RDRLR            0x30
#define CH_IER              0x34
#define CH_RIWTR            0x38
#define CH_CHTDR            0x44
#define CH_CHRDR            0x4C
#define CH_CHTBAR           0x54
#define CH_CHRBAR           0x5C
#define CH_SR               0x60

#define MCR_RE              (1u << 0)
#define MCR_TE              (1u << 1)
#define MDIO_GB             (1u << 0)
#define MDIO_GOC_READ       (3u << 2)
#define DMA_MODE_SWR        (1u << 0)
#define TCR_ST              (1u << 0)
#define RCR_SR              (1u << 0)
#define MTL_FTQ             (1u << 0)

#define SR_TI               (1u << 0)
#define SR_TBU              (1u << 2)
#define SR_RI               (1u << 6)
#define SR_RBU              (1u << 7)
#define SR_AIS              (1u << 14)
#define SR_NIS              (1u << 15)
#define SR_NORMAL           (SR_TI | SR_TBU | SR_RI | (1u << 11))
#define SR_ABNORMAL         (0x3FFFu & ~SR_NORMAL & ~SR_AIS)

#define DES3_OWN            (1u << 31)
#define DES3_CTXT           (1u << 30)
#define DES3_FD             (1u << 29)
#define DES3_LD             (1u << 28)
#define RDES3_IOC           (1u << 30)
#define TDES2_IOC           (1u << 31)

/* DSR0 process states */
#define RPS_STOPPED         0
#define RPS_WAITING         3
#define RPS_SUSPENDED       4
#define TPS_STOPPED         0
#define TPS_SUSPENDED       6

#define PHY_ADDR            1
#define EQOS_MAX_FRAME      9018

struct NvtEqosState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    uint32_t regs[EQOS_SIZE / 4];
    uint32_t cur_tx, cur_rx;            /* descriptor indices */
    uint32_t rps;                       /* Rx process state, DSR0 [11:8] */
    bool irq_level;
    uint16_t phy[32];
};

static uint32_t *reg(NvtEqosState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static uint32_t *ch(NvtEqosState *s, hwaddr off)
{
    return &s->regs[(DMA_CH0 + off) / 4];
}

/* ---- PHY ----------------------------------------------------------- */

static void nvt_eqos_phy_reset(NvtEqosState *s)
{
    memset(s->phy, 0, sizeof(s->phy));
    s->phy[MII_BMCR] = MII_BMCR_AUTOEN | MII_BMCR_SPEED100 | MII_BMCR_FD;
    s->phy[MII_BMSR] = MII_BMSR_100TX_FD | MII_BMSR_100TX_HD |
                       MII_BMSR_10T_FD | MII_BMSR_10T_HD | MII_BMSR_MFPS |
                       MII_BMSR_AN_COMP | MII_BMSR_AUTONEG |
                       MII_BMSR_LINK_ST | MII_BMSR_EXTCAP;
    s->phy[MII_PHYID1] = 0x001C;                /* generic: genphy binds */
    s->phy[MII_PHYID2] = 0xC800;
    s->phy[MII_ANAR] = MII_ANAR_TXFD | MII_ANAR_TX | MII_ANAR_10FD |
                       MII_ANAR_10 | MII_ANAR_CSMACD;
    s->phy[MII_ANLPAR] = MII_ANLPAR_ACK | MII_ANLPAR_TXFD | MII_ANLPAR_TX |
                         MII_ANLPAR_10FD | MII_ANLPAR_10 | MII_ANLPAR_CSMACD;
}

static void nvt_eqos_mdio(NvtEqosState *s, uint32_t val)
{
    const int pa = (val >> 21) & 0x1F, ra = (val >> 16) & 0x1F;

    if ((val & MDIO_GOC_READ) == MDIO_GOC_READ) {
        *reg(s, MAC_MDIO_DATA) = pa == PHY_ADDR ? s->phy[ra] : 0xFFFF;
    } else if (pa == PHY_ADDR) {
        uint16_t v = *reg(s, MAC_MDIO_DATA);

        switch (ra) {
        case MII_BMCR:
            if (v & MII_BMCR_RESET) {
                nvt_eqos_phy_reset(s);
            } else {
                s->phy[ra] = v & ~MII_BMCR_ANRESTART;
            }
            break;
        case MII_BMSR:
        case MII_PHYID1:
        case MII_PHYID2:
        case MII_ANLPAR:
            break;
        default:
            s->phy[ra] = v;
            break;
        }
    }
    *reg(s, MAC_MDIO_ADDR) = val & ~MDIO_GB;    /* done at once */
}

/* ---- DMA ----------------------------------------------------------- */

static void nvt_eqos_update_irq(NvtEqosState *s)
{
    uint32_t sr = *ch(s, CH_SR), ier = *ch(s, CH_IER);
    bool level = false;

    sr &= ~(SR_NIS | SR_AIS);
    if (sr & SR_NORMAL) {
        sr |= SR_NIS;
    }
    if (sr & SR_ABNORMAL) {
        sr |= SR_AIS;
    }
    *ch(s, CH_SR) = sr;
    if ((ier & SR_NIS) && (sr & ier & SR_NORMAL)) {
        level = true;
    }
    if ((ier & SR_AIS) && (sr & ier & SR_ABNORMAL)) {
        level = true;
    }
    *reg(s, DMA_ISR) = (*reg(s, DMA_ISR) & ~1u) | (sr ? 1 : 0);
    s->irq_level = level;
    qemu_set_irq(s->irq, level);
}

static uint32_t nvt_eqos_desc_size(NvtEqosState *s)
{
    return 16 + ((*ch(s, CH_CR) >> 18) & 7) * 8;
}

static dma_addr_t nvt_eqos_desc(NvtEqosState *s, bool rx, uint32_t idx)
{
    return *ch(s, rx ? CH_RDLAR : CH_TDLAR) + idx * nvt_eqos_desc_size(s);
}

static uint32_t nvt_eqos_ring_len(NvtEqosState *s, bool rx)
{
    return (*ch(s, rx ? CH_RDRLR : CH_TDRLR) & 0x3FF) + 1;
}

static void nvt_eqos_read_desc(dma_addr_t a, uint32_t d[4])
{
    int i;

    dma_memory_read(&address_space_memory, a, d, 16, MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < 4; i++) {
        d[i] = le32_to_cpu(d[i]);
    }
}

static void nvt_eqos_write_desc(dma_addr_t a, const uint32_t d[4])
{
    uint32_t le[4];
    int i;

    for (i = 0; i < 4; i++) {
        le[i] = cpu_to_le32(d[i]);
    }
    dma_memory_write(&address_space_memory, a, le, 16,
                     MEMTXATTRS_UNSPECIFIED);
}

static void nvt_eqos_set_state(NvtEqosState *s)
{
    uint32_t dsr = *reg(s, DMA_DSR0) & ~0xFF00u;

    if (*ch(s, CH_RCR) & RCR_SR) {
        dsr |= s->rps << 8;
    }
    if (*ch(s, CH_TCR) & TCR_ST) {
        dsr |= TPS_SUSPENDED << 12;
    }
    *reg(s, DMA_DSR0) = dsr;
}

static void nvt_eqos_tx(NvtEqosState *s)
{
    uint8_t frame[EQOS_MAX_FRAME];
    uint32_t len = 0, d[4], n = 0, l1, l2;
    const uint32_t ring = nvt_eqos_ring_len(s, false);
    dma_addr_t a;

    if (!(*ch(s, CH_TCR) & TCR_ST) || !(*reg(s, MAC_MCR) & MCR_TE)) {
        return;
    }
    while (n++ < ring) {
        a = nvt_eqos_desc(s, false, s->cur_tx);
        if (a == *ch(s, CH_TDTP)) {
            break;                  /* caught up with the tail pointer */
        }
        nvt_eqos_read_desc(a, d);
        if (!(d[3] & DES3_OWN)) {
            *ch(s, CH_SR) |= SR_TBU;
            break;
        }
        if (!(d[3] & DES3_CTXT)) {
            if (d[3] & DES3_FD) {
                len = 0;
            }
            l1 = d[2] & 0x3FFF;
            l2 = (d[2] >> 16) & 0x3FFF;
            if (len + l1 + l2 <= sizeof(frame)) {
                dma_memory_read(&address_space_memory, d[0], frame + len,
                                l1, MEMTXATTRS_UNSPECIFIED);
                dma_memory_read(&address_space_memory, d[1],
                                frame + len + l1, l2,
                                MEMTXATTRS_UNSPECIFIED);
                len += l1 + l2;
            }
            if (d[3] & DES3_LD) {
                qemu_send_packet(qemu_get_queue(s->nic), frame, len);
                len = 0;
            }
            if (d[2] & TDES2_IOC) {
                *ch(s, CH_SR) |= SR_TI;
            }
        }
        /* Write-back format: FD / LD kept, no error bits, no timestamp. */
        d[3] &= DES3_FD | DES3_LD;
        d[0] = d[1] = 0;
        nvt_eqos_write_desc(a, d);
        *ch(s, CH_CHTDR) = a;
        s->cur_tx = (s->cur_tx + 1) % ring;
    }
    nvt_eqos_update_irq(s);
}

static bool nvt_eqos_can_receive(NetClientState *nc)
{
    NvtEqosState *s = qemu_get_nic_opaque(nc);
    uint32_t d[4];

    if (!(*ch(s, CH_RCR) & RCR_SR) || !(*reg(s, MAC_MCR) & MCR_RE)) {
        return false;
    }
    nvt_eqos_read_desc(nvt_eqos_desc(s, true, s->cur_rx), d);
    return d[3] & DES3_OWN;
}

static ssize_t nvt_eqos_receive(NetClientState *nc, const uint8_t *buf,
                                size_t size)
{
    NvtEqosState *s = qemu_get_nic_opaque(nc);
    const uint32_t bufsz = *ch(s, CH_RCR) & 0x7FFE;
    uint32_t d[4], len;
    dma_addr_t a;

    if (!nvt_eqos_can_receive(nc)) {
        s->rps = RPS_SUSPENDED;
        *ch(s, CH_SR) |= SR_RBU;
        nvt_eqos_update_irq(s);
        return 0;
    }
    a = nvt_eqos_desc(s, true, s->cur_rx);
    nvt_eqos_read_desc(a, d);
    len = MIN(size, bufsz ? bufsz : 2048);
    dma_memory_write(&address_space_memory, d[0], buf, len,
                     MEMTXATTRS_UNSPECIFIED);
    /* IOC, or the Rx watchdog (RIWT), which the driver uses for the
     * descriptors it leaves IOC clear on; it is taken to expire at once. */
    if ((d[3] & RDES3_IOC) || (*ch(s, CH_RIWTR) & 0xFF)) {
        *ch(s, CH_SR) |= SR_RI;
    }
    d[1] = 0;
    d[2] = 0;
    d[3] = DES3_FD | DES3_LD | len;
    nvt_eqos_write_desc(a, d);
    *ch(s, CH_CHRDR) = a;
    *ch(s, CH_CHRBAR) = d[0];
    s->cur_rx = (s->cur_rx + 1) % nvt_eqos_ring_len(s, true);
    nvt_eqos_update_irq(s);
    return size;
}

/* ---- registers ----------------------------------------------------- */

static void nvt_eqos_soft_reset(NvtEqosState *s)
{
    uint32_t glue[(EQOS_SIZE - 0x3000) / 4];

    /* The software reset is the core's; the Novatek blocks keep theirs. */
    memcpy(glue, &s->regs[0x3000 / 4], sizeof(glue));
    memset(s->regs, 0, sizeof(s->regs));
    memcpy(&s->regs[0x3000 / 4], glue, sizeof(glue));
    *reg(s, MAC_VR) = 0x00001041;               /* SNPSVER 4.10 */
    /* MII, half duplex, MDIO; no checksum offload, MMC or timestamps. */
    *reg(s, MAC_HFR0) = (1u << 0) | (1u << 2) | (1u << 5);
    *reg(s, MAC_HFR1) = 5 | (5 << 6);           /* 4 KiB Rx / Tx FIFOs */
    *reg(s, MAC_HFR2) = 0;                      /* one Tx, one Rx queue */
    *reg(s, MAC_ADDR0_HI) = (1u << 31) | s->conf.macaddr.a[4] |
                            (s->conf.macaddr.a[5] << 8);
    *reg(s, MAC_ADDR0_LO) = s->conf.macaddr.a[0] |
                            (s->conf.macaddr.a[1] << 8) |
                            (s->conf.macaddr.a[2] << 16) |
                            ((uint32_t)s->conf.macaddr.a[3] << 24);
    s->cur_tx = s->cur_rx = 0;
    s->rps = RPS_STOPPED;
    s->irq_level = false;
    qemu_irq_lower(s->irq);
}

static uint64_t nvt_eqos_read(void *opaque, hwaddr off, unsigned size)
{
    NvtEqosState *s = opaque;

    switch (off) {
    case DMA_DSR0:
        nvt_eqos_set_state(s);
        break;
    case ETH_ISR:
        return s->irq_level;
    }
    return *reg(s, off);
}

static void nvt_eqos_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    NvtEqosState *s = opaque;

    switch (off) {
    case MAC_MCR:
        *reg(s, off) = val;
        nvt_eqos_tx(s);
        if (val & MCR_RE) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        return;
    case MAC_MDIO_ADDR:
        if (val & MDIO_GB) {
            nvt_eqos_mdio(s, val);
        } else {
            *reg(s, off) = val;
        }
        return;
    case MAC_ISR:
    case MAC_VR:
    case MAC_HFR0:
    case MAC_HFR1:
    case MAC_HFR2:
    case DMA_DSR0:
        return;
    case MTL_Q0TOMR:
        *reg(s, off) = val & ~MTL_FTQ;          /* flushed at once */
        return;
    case DMA_MODE:
        if (val & DMA_MODE_SWR) {
            nvt_eqos_soft_reset(s);
            return;
        }
        break;
    case DMA_ISR:
        return;
    }

    if (off >= DMA_CH0 && off < DMA_CH0_END) {
        switch (off - DMA_CH0) {
        case CH_SR:
            *ch(s, CH_SR) &= ~val;
            nvt_eqos_update_irq(s);
            return;
        case CH_IER:
            *ch(s, CH_IER) = val;
            nvt_eqos_update_irq(s);
            return;
        case CH_TDLAR:
            *ch(s, CH_TDLAR) = val;
            s->cur_tx = 0;
            return;
        case CH_RDLAR:
            *ch(s, CH_RDLAR) = val;
            s->cur_rx = 0;
            return;
        case CH_TCR:
            *ch(s, CH_TCR) = val;
            nvt_eqos_tx(s);
            return;
        case CH_TDTP:
            *ch(s, CH_TDTP) = val;
            nvt_eqos_tx(s);
            return;
        case CH_RCR:
        case CH_RDTP:
            *reg(s, off) = val;
            if (*ch(s, CH_RCR) & RCR_SR) {
                s->rps = RPS_WAITING;
                qemu_flush_queued_packets(qemu_get_queue(s->nic));
            }
            return;
        }
    }
    *reg(s, off) = val;
}

static const MemoryRegionOps nvt_eqos_ops = {
    .read = nvt_eqos_read,
    .write = nvt_eqos_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static NetClientInfo nvt_eqos_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = nvt_eqos_can_receive,
    .receive = nvt_eqos_receive,
};

static void nvt_eqos_reset(DeviceState *dev)
{
    NvtEqosState *s = NVT_EQOS(dev);

    memset(&s->regs[0x3000 / 4], 0, EQOS_SIZE - 0x3000);
    nvt_eqos_soft_reset(s);
    nvt_eqos_phy_reset(s);
}

static void nvt_eqos_realize(DeviceState *dev, Error **errp)
{
    NvtEqosState *s = NVT_EQOS(dev);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&nvt_eqos_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void nvt_eqos_init(Object *obj)
{
    NvtEqosState *s = NVT_EQOS(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_eqos_ops, s, TYPE_NVT_EQOS,
                          EQOS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_nvt_eqos = {
    .name = TYPE_NVT_EQOS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtEqosState, EQOS_SIZE / 4),
        VMSTATE_UINT32(cur_tx, NvtEqosState),
        VMSTATE_UINT32(cur_rx, NvtEqosState),
        VMSTATE_UINT32(rps, NvtEqosState),
        VMSTATE_BOOL(irq_level, NvtEqosState),
        VMSTATE_UINT16_ARRAY(phy, NvtEqosState, 32),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_eqos_properties[] = {
    DEFINE_NIC_PROPERTIES(NvtEqosState, conf),
};

static void nvt_eqos_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_eqos_realize;
    device_class_set_legacy_reset(dc, nvt_eqos_reset);
    device_class_set_props(dc, nvt_eqos_properties);
    dc->vmsd = &vmstate_nvt_eqos;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo nvt_eqos_info = {
    .name = TYPE_NVT_EQOS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtEqosState),
    .instance_init = nvt_eqos_init,
    .class_init = nvt_eqos_class_init,
};

static void nvt_eqos_register_types(void)
{
    type_register_static(&nvt_eqos_info);
}

type_init(nvt_eqos_register_types)
