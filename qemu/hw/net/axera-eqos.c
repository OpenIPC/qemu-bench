/*
 * Axera AX620E Ethernet: a Synopsys DWC Ethernet QoS / dwmac-4.10a core
 * with an external RMII PHY on the shared MDIO bus.
 *
 * ethernet@104c0000 in the real devicetree: "axera,dwmac-4.10a", reg
 * <0x104c0000 0x10000>, interrupts "macirq"/"eth_wake_irq"/"eth_lpi"
 * (only macirq is modelled -- the mainline stmmac driver this dwmac-4.10a
 * compatible string binds to treats the other two as optional), phy-mode
 * "rmii", phy-handle pointing at an "ethernet-phy-id0044.1400" child of
 * this same node's mdio bus (reg 1). "dwmac-4.10a" is mainline Linux's own
 * exact IP-revision string for this Synopsys core generation (see
 * stmmac_platform.c's compatible-string table) -- register-for-register
 * the same silicon Synopsys calls "DWC_ETH_QOS" in its own reference
 * driver and documentation, which is what hw/net/nvt-eqos.c's own header
 * comment describes; this file is that same register map (MAC at 0x0000,
 * MTL at 0x0C00, DMA at 0x1000, channel 0 at 0x1100, four-word OWN-bit
 * descriptors) with the Novatek-specific embedded-PHY glue block (its
 * 0x3000/0x3800 ranges) dropped, since Axera's PHY is an ordinary
 * external RMII chip reached over MDIO, not embedded silicon needing its
 * own analog power-on registers.
 *
 * The PHY's register bank reports MII_PHYID1/2 as 0x0044/0x1400, matching
 * the real devicetree's "ethernet-phy-idAAAA.BBBB" compatible string
 * exactly (that string IS the PHY_ID the kernel's generic OF-PHY match
 * expects to read back over MDIO), rather than reusing nvt-eqos.c's own
 * stand-in ID for a different, unrelated embedded PHY.
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

#define TYPE_AXERA_EQOS "axera-eqos"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraEqosState, AXERA_EQOS)

#define EQOS_SIZE           0x4000

#define MAC_MCR             0x0000
#define MCR_ACS             (1u << 20)  /* auto pad/CRC stripping */
#define MCR_CST             (1u << 21)  /* CRC stripping, Type frames */
#define MAC_MPFR            0x0008
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
#define TDES3_ES            (1u << 15)      /* write-back: error summary */
#define TDES3_JT            (1u << 14)      /* write-back: jabber timeout */
#define MPFR_PR             (1u << 0)
#define MPFR_HUC            (1u << 1)
#define MPFR_HMC            (1u << 2)
#define MPFR_PM             (1u << 4)
#define MPFR_DBF            (1u << 5)
#define MPFR_RA             (1u << 31)
#define MAC_ADDR_AE         (1u << 31)
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

struct AxeraEqosState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    uint32_t regs[EQOS_SIZE / 4];
    uint32_t cur_tx, cur_rx;            /* descriptor indices */
    uint32_t rps;                       /* Rx process state, DSR0 [11:8] */
    bool irq_level;
    /* the Tx frame being gathered, kept across tail-pointer writes */
    uint8_t tx_frame[EQOS_MAX_FRAME];
    uint32_t tx_len;
    bool tx_overflow;
    uint16_t phy[32];
};

static uint32_t *reg(AxeraEqosState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static uint32_t *ch(AxeraEqosState *s, hwaddr off)
{
    return &s->regs[(DMA_CH0 + off) / 4];
}

/* ---- PHY ----------------------------------------------------------- */

static void axera_eqos_phy_reset(AxeraEqosState *s)
{
    memset(s->phy, 0, sizeof(s->phy));
    s->phy[MII_BMCR] = MII_BMCR_AUTOEN | MII_BMCR_SPEED100 | MII_BMCR_FD;
    s->phy[MII_BMSR] = MII_BMSR_100TX_FD | MII_BMSR_100TX_HD |
                       MII_BMSR_10T_FD | MII_BMSR_10T_HD | MII_BMSR_MFPS |
                       MII_BMSR_AN_COMP | MII_BMSR_AUTONEG |
                       MII_BMSR_LINK_ST | MII_BMSR_EXTCAP;
    s->phy[MII_PHYID1] = 0x0044;        /* matches the real devicetree's */
    s->phy[MII_PHYID2] = 0x1400;        /* "ethernet-phy-id0044.1400" */
    s->phy[MII_ANAR] = MII_ANAR_TXFD | MII_ANAR_TX | MII_ANAR_10FD |
                       MII_ANAR_10 | MII_ANAR_CSMACD;
    s->phy[MII_ANLPAR] = MII_ANLPAR_ACK | MII_ANLPAR_TXFD | MII_ANLPAR_TX |
                         MII_ANLPAR_10FD | MII_ANLPAR_10 | MII_ANLPAR_CSMACD;
}

static void axera_eqos_mdio(AxeraEqosState *s, uint32_t val)
{
    const int pa = (val >> 21) & 0x1F, ra = (val >> 16) & 0x1F;

    if ((val & MDIO_GOC_READ) == MDIO_GOC_READ) {
        *reg(s, MAC_MDIO_DATA) = pa == PHY_ADDR ? s->phy[ra] : 0xFFFF;
    } else if (pa == PHY_ADDR) {
        uint16_t v = *reg(s, MAC_MDIO_DATA);

        switch (ra) {
        case MII_BMCR:
            if (v & MII_BMCR_RESET) {
                axera_eqos_phy_reset(s);
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

static void axera_eqos_update_irq(AxeraEqosState *s)
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

static uint32_t axera_eqos_desc_size(AxeraEqosState *s)
{
    return 16 + ((*ch(s, CH_CR) >> 18) & 7) * 8;
}

static dma_addr_t axera_eqos_desc(AxeraEqosState *s, bool rx, uint32_t idx)
{
    return *ch(s, rx ? CH_RDLAR : CH_TDLAR) + idx * axera_eqos_desc_size(s);
}

static uint32_t axera_eqos_ring_len(AxeraEqosState *s, bool rx)
{
    return (*ch(s, rx ? CH_RDRLR : CH_TDRLR) & 0x3FF) + 1;
}

static void axera_eqos_read_desc(dma_addr_t a, uint32_t d[4])
{
    int i;

    dma_memory_read(&address_space_memory, a, d, 16, MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < 4; i++) {
        d[i] = le32_to_cpu(d[i]);
    }
}

static void axera_eqos_write_desc(dma_addr_t a, const uint32_t d[4])
{
    uint32_t le[4];
    int i;

    for (i = 0; i < 4; i++) {
        le[i] = cpu_to_le32(d[i]);
    }
    dma_memory_write(&address_space_memory, a, le, 16,
                     MEMTXATTRS_UNSPECIFIED);
}

static void axera_eqos_set_state(AxeraEqosState *s)
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

static void axera_eqos_tx(AxeraEqosState *s)
{
    uint32_t d[4], n = 0, l1, l2;
    const uint32_t ring = axera_eqos_ring_len(s, false);
    dma_addr_t a;

    if (!(*ch(s, CH_TCR) & TCR_ST) || !(*reg(s, MAC_MCR) & MCR_TE)) {
        return;
    }
    while (n++ < ring) {
        a = axera_eqos_desc(s, false, s->cur_tx);
        if (a == *ch(s, CH_TDTP)) {
            break;                  /* caught up with the tail pointer */
        }
        axera_eqos_read_desc(a, d);
        if (!(d[3] & DES3_OWN)) {
            *ch(s, CH_SR) |= SR_TBU;
            break;
        }
        if (d[3] & DES3_CTXT) {
            d[3] = 0;
        } else {
            if (d[3] & DES3_FD) {
                s->tx_len = 0;
                s->tx_overflow = false;
            }
            l1 = d[2] & 0x3FFF;
            l2 = (d[2] >> 16) & 0x3FFF;
            if (s->tx_overflow || s->tx_len + l1 + l2 > EQOS_MAX_FRAME) {
                s->tx_overflow = true;
            } else {
                dma_memory_read(&address_space_memory, d[0],
                                s->tx_frame + s->tx_len, l1,
                                MEMTXATTRS_UNSPECIFIED);
                dma_memory_read(&address_space_memory, d[1],
                                s->tx_frame + s->tx_len + l1, l2,
                                MEMTXATTRS_UNSPECIFIED);
                s->tx_len += l1 + l2;
            }
            /* Write-back format: FD / LD kept, error flags in the low
             * half, no timestamp.  A frame too long to gather is dropped
             * as a jabber timeout rather than sent cut short. */
            d[3] &= DES3_FD | DES3_LD;
            if (d[3] & DES3_LD) {
                if (s->tx_overflow) {
                    d[3] |= TDES3_ES | TDES3_JT;
                } else {
                    qemu_send_packet(qemu_get_queue(s->nic), s->tx_frame,
                                     s->tx_len);
                }
                s->tx_len = 0;
                s->tx_overflow = false;
            }
            if (d[2] & TDES2_IOC) {
                *ch(s, CH_SR) |= SR_TI;
            }
        }
        d[0] = d[1] = 0;
        axera_eqos_write_desc(a, d);
        *ch(s, CH_CHTDR) = a;
        s->cur_tx = (s->cur_tx + 1) % ring;
    }
    axera_eqos_update_irq(s);
}

static uint32_t axera_eqos_rx_bufsz(AxeraEqosState *s)
{
    uint32_t bufsz = (*ch(s, CH_RCR) >> 1) & 0x3FFF;   /* RBSZ [14:1] */

    return bufsz ? bufsz & ~3u : 2048;
}

/*
 * How many descriptors from the current one the DMA may fill: gated
 * purely on each descriptor's own OWN bit, NOT on the tail pointer.
 *
 * An earlier version of this function also stopped the instant
 * cur_rx's address reached CH_RDTP (reasoning: "the driver points it
 * at the last descriptor it refilled, so that's the hard boundary").
 * That's wrong, confirmed live: the real stmmac driver's own RX
 * refill (stmmac_rx_refill(), called unconditionally at the end of
 * every NAPI poll regardless of how many new packets that poll
 * found) recomputes the tail as dma_rx_phy + dirty_rx*desc_size,
 * where dirty_rx is "how many descriptors this poll has personally
 * refilled so far" -- NOT "how many of the ring's buf_alloc_num
 * descriptors are still genuinely owned by DMA". After the very
 * first packet is processed, this rewrites tail to point at the
 * NEXT descriptor (cur_rx itself) even though that descriptor's OWN
 * bit is still legitimately set from the ring's original full
 * allocation (buf_alloc_num descriptors, all marked owned, at
 * interface-open time) and was never touched by this refill.
 * Confirmed live (temporary instrumentation, since reverted): at the
 * exact point every further incoming packet was being silently
 * dropped, cur_rx's own descriptor read back DES3_OWN=1 (genuinely
 * available) while sitting exactly AT the just-rewritten tail
 * address -- the old tail check was discarding real, driver-granted
 * capacity for no reason, which turned one quiet moment early in
 * boot into a permanent dead end (nothing could ever arrive again to
 * trigger the interrupt that's the ONLY thing that makes the driver
 * advance the tail further). Respecting only OWN bit matches what
 * the driver is actually asking for: "use any descriptor I've marked
 * ready", with no separate, stricter boundary.
 */
static uint32_t axera_eqos_rx_avail(AxeraEqosState *s, uint32_t want)
{
    const uint32_t ring = axera_eqos_ring_len(s, true);
    uint32_t n, d[4];
    dma_addr_t a;

    if (!(*ch(s, CH_RCR) & RCR_SR) || !(*reg(s, MAC_MCR) & MCR_RE)) {
        return 0;
    }
    for (n = 0; n < want && n < ring; n++) {
        a = axera_eqos_desc(s, true, (s->cur_rx + n) % ring);
        axera_eqos_read_desc(a, d);
        if (!(d[3] & DES3_OWN)) {
            break;
        }
    }
    return n;
}

static bool axera_eqos_can_receive(NetClientState *nc)
{
    AxeraEqosState *s = qemu_get_nic_opaque(nc);

    return axera_eqos_rx_avail(s, 1) > 0;
}

/* The MAC packet filter: promiscuous / receive-all, broadcast, multicast
 * (all, or through the hash filter, taken as passing everything), and
 * unicast against the enabled MAC address registers. */
static bool axera_eqos_accept(AxeraEqosState *s, const uint8_t *buf,
                              size_t size)
{
    const uint32_t mpfr = *reg(s, MAC_MPFR);
    uint32_t hi, lo;
    int i;

    if (size < 6 || (mpfr & (MPFR_PR | MPFR_RA))) {
        return true;
    }
    if (is_broadcast_ether_addr(buf)) {
        return !(mpfr & MPFR_DBF);
    }
    if (is_multicast_ether_addr(buf)) {
        return mpfr & (MPFR_PM | MPFR_HMC);
    }
    if (mpfr & MPFR_HUC) {
        return true;
    }
    for (i = 0; i < 32; i++) {
        hi = *reg(s, MAC_ADDR0_HI + 8 * i);
        lo = *reg(s, MAC_ADDR0_LO + 8 * i);
        if (i && !(hi & MAC_ADDR_AE)) {
            continue;
        }
        if (ldl_le_p(buf) == lo && lduw_le_p(buf + 4) == (hi & 0xFFFF)) {
            return true;
        }
    }
    return false;
}

static ssize_t axera_eqos_receive(NetClientState *nc, const uint8_t *pkt,
                                  size_t pkt_size)
{
    AxeraEqosState *s = qemu_get_nic_opaque(nc);
    const uint32_t bufsz = axera_eqos_rx_bufsz(s);
    const uint32_t ring = axera_eqos_ring_len(s, true);
    g_autofree uint8_t *fbuf = NULL;
    const uint8_t *buf = pkt;
    size_t size = pkt_size;
    uint32_t need, done = 0, i, chunk, d[4];
    dma_addr_t a;

    if (!axera_eqos_accept(s, pkt, pkt_size)) {
        return pkt_size;            /* filtered out: not for us */
    }
    /*
     * Real silicon hands the FCS to software unless MAC_Configuration
     * ACS/CST ask it to strip it, and stmmac (core >= 4.00) always
     * subtracts ETH_FCS_LEN from the descriptor's packet length on that
     * basis. Without the 4 bytes every received IP packet came out 4
     * bytes short: IpExt InTruncatedPkts counted them all and nothing
     * ever reached TCP/UDP/ICMP (no DNS/DHCP replies, no SYN-ACK to an
     * RTSP client).
     */
    if (!(*reg(s, MAC_MCR) & (MCR_ACS | MCR_CST))) {
        uint32_t fcs = net_crc32_le(pkt, pkt_size);

        fbuf = g_malloc(pkt_size + 4);
        memcpy(fbuf, pkt, pkt_size);
        stl_le_p(fbuf + pkt_size, ~fcs);
        buf = fbuf;
        size = pkt_size + 4;
    }
    need = DIV_ROUND_UP(size, bufsz);
    if (size == 0 || need >= ring) {
        return pkt_size;            /* can never fit the ring: dropped */
    }
    if (axera_eqos_rx_avail(s, need) < need) {
        s->rps = RPS_SUSPENDED;
        *ch(s, CH_SR) |= SR_RBU;
        axera_eqos_update_irq(s);
        return 0;                   /* queued until the guest refills */
    }

    /* Spread over as many buffers as it takes: FD on the first, LD and
     * the frame length (FCS included, see above) on the last. */
    for (i = 0; i < need; i++) {
        a = axera_eqos_desc(s, true, s->cur_rx);
        axera_eqos_read_desc(a, d);
        chunk = MIN(size - done, bufsz);
        dma_memory_write(&address_space_memory, d[0], buf + done, chunk,
                         MEMTXATTRS_UNSPECIFIED);
        done += chunk;
        /* IOC, or the Rx watchdog (RIWT), which the driver uses for the
         * descriptors it leaves IOC clear on; it is taken to expire at
         * once. */
        if ((d[3] & RDES3_IOC) || (*ch(s, CH_RIWTR) & 0xFF)) {
            *ch(s, CH_SR) |= SR_RI;
        }
        d[1] = 0;
        d[2] = 0;
        d[3] = (i == 0 ? DES3_FD : 0) |
               (i == need - 1 ? DES3_LD | size : 0);
        axera_eqos_write_desc(a, d);
        *ch(s, CH_CHRDR) = a;
        *ch(s, CH_CHRBAR) = d[0];
        s->cur_rx = (s->cur_rx + 1) % ring;
    }
    axera_eqos_update_irq(s);
    return pkt_size;
}

/* ---- registers ----------------------------------------------------- */

static void axera_eqos_soft_reset(AxeraEqosState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
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
    s->tx_len = 0;
    s->tx_overflow = false;
    s->rps = RPS_STOPPED;
    s->irq_level = false;
    qemu_irq_lower(s->irq);
}

static uint64_t axera_eqos_read(void *opaque, hwaddr off, unsigned size)
{
    AxeraEqosState *s = opaque;

    switch (off) {
    case DMA_DSR0:
        axera_eqos_set_state(s);
        break;
    }
    return *reg(s, off);
}

static void axera_eqos_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    AxeraEqosState *s = opaque;

    switch (off) {
    case MAC_MCR:
        *reg(s, off) = val;
        axera_eqos_tx(s);
        if (val & MCR_RE) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        return;
    case MAC_MDIO_ADDR:
        if (val & MDIO_GB) {
            axera_eqos_mdio(s, val);
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
            axera_eqos_soft_reset(s);
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
            axera_eqos_update_irq(s);
            return;
        case CH_IER:
            *ch(s, CH_IER) = val;
            axera_eqos_update_irq(s);
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
            axera_eqos_tx(s);
            return;
        case CH_TDTP:
            *ch(s, CH_TDTP) = val;
            axera_eqos_tx(s);
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

static const MemoryRegionOps axera_eqos_ops = {
    .read = axera_eqos_read,
    .write = axera_eqos_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static NetClientInfo axera_eqos_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = axera_eqos_can_receive,
    .receive = axera_eqos_receive,
};

static void axera_eqos_reset(DeviceState *dev)
{
    AxeraEqosState *s = AXERA_EQOS(dev);

    axera_eqos_soft_reset(s);
    axera_eqos_phy_reset(s);
}

static void axera_eqos_realize(DeviceState *dev, Error **errp)
{
    AxeraEqosState *s = AXERA_EQOS(dev);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&axera_eqos_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void axera_eqos_init(Object *obj)
{
    AxeraEqosState *s = AXERA_EQOS(obj);

    memory_region_init_io(&s->iomem, obj, &axera_eqos_ops, s,
                          TYPE_AXERA_EQOS, EQOS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_axera_eqos = {
    .name = TYPE_AXERA_EQOS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AxeraEqosState, EQOS_SIZE / 4),
        VMSTATE_UINT32(cur_tx, AxeraEqosState),
        VMSTATE_UINT32(cur_rx, AxeraEqosState),
        VMSTATE_UINT32(rps, AxeraEqosState),
        VMSTATE_BOOL(irq_level, AxeraEqosState),
        VMSTATE_UINT8_ARRAY(tx_frame, AxeraEqosState, EQOS_MAX_FRAME),
        VMSTATE_UINT32(tx_len, AxeraEqosState),
        VMSTATE_BOOL(tx_overflow, AxeraEqosState),
        VMSTATE_UINT16_ARRAY(phy, AxeraEqosState, 32),
        VMSTATE_END_OF_LIST()
    }
};

static const Property axera_eqos_properties[] = {
    DEFINE_NIC_PROPERTIES(AxeraEqosState, conf),
};

static void axera_eqos_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_eqos_realize;
    device_class_set_legacy_reset(dc, axera_eqos_reset);
    device_class_set_props(dc, axera_eqos_properties);
    dc->vmsd = &vmstate_axera_eqos;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo axera_eqos_info = {
    .name = TYPE_AXERA_EQOS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraEqosState),
    .instance_init = axera_eqos_init,
    .class_init = axera_eqos_class_init,
};

static void axera_eqos_register_types(void)
{
    type_register_static(&axera_eqos_info);
}

type_init(axera_eqos_register_types)
