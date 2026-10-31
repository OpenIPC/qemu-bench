/*
 * Ingenic T-series Ethernet: a Synopsys DesignWare GMAC 3.x core
 * (0x134B0000, INTC 55) with an IP101G-like RMII PHY on its MDIO bus.
 *
 * All the drivers are the Synopsys reference driver (synopGMAC_Dev.c)
 * under an Ingenic wrapper -- jz_mac.c ("V12": T10/T20/T21/T30) and
 * jz_mac_v13.c (T23/T31) in the 3.10 kernels, ingenic_mac.c in the T40/
 * T41 4.4 ones, drivers/net/jz4775-9161.c in U-Boot -- and every one is
 * built with -DENH_DESC -DENH_DESC_8W (drivers/net/ethernet/ingenic/
 * Makefile) and programs DmaBusMode = PBL32 | DSL 2 | ATDS | FB | AAL
 * (jz_mac_configure, jz_mac_v13.c:1657; ingenic_mac.c:1604; U-Boot
 * jz47xx_mac_configure).  So the descriptors are the 8-word enhanced ones
 * with two driver words after them: 40 bytes apart in a ring.
 *
 * MAC block at +0x0000 (synopGMAC_Dev.h enum GmacRegisters):
 *   0x00 Config (RE 2, TE 3, ACS 7 pad/CRC strip, IPC 10, CST 25)
 *   0x04 FrameFilter (PR 0, HUC 1, HMC 2, PM 4, DBF 5, RA 31)
 *   0x08/0x0C hash, 0x18 flow control, 0x1C VLAN
 *   0x10 GmiiAddr: GB 0 busy, GW 1 write, CR 5:2, GR 10:6, PA 15:11
 *   0x14 GmiiData
 *   0x20 Version (read and stored, never checked; synopGMAC_Dev.c:192)
 *   0x38 interrupt status (read in the ISR), 0x3C mask
 *   0x40.. Addr0..15 High/Low, 0x800.. Addr16..31
 *   0x100.. MMC: control and interrupt masks; the counters read zero
 *   0x700.. IEEE 1588 timestamp block (plain registers)
 *
 * DMA block at +0x1000 (enum DmaRegisters):
 *   0x00 BusMode: SWR 0 (self-clearing; the drivers poll it for ~10 ms
 *        and fail probe or open if it sticks, synopGMAC_Dev.c:206-231),
 *        DSL 6:2 (words skipped between ring descriptors), ATDS 7
 *   0x04/0x08 TX/RX poll demand, 0x0C/0x10 RX/TX list base
 *   0x14 Status: TI 0, TPS 1, TU 2, UNF 5, RI 6, RU 7, RPS 8, FBI 13,
 *        AIS 15, NIS 16 (write 1 to clear), RS 19:17 and TS 22:20 states
 *   0x18 OpMode: SR 1, ST 13, FTF 20 (self-clearing)
 *   0x1C interrupt enable (NIE 16, AIE 15), 0x20 missed frames (read
 *        clears), 0x48..0x54 current descriptor and buffer pointers,
 *   0x58 HW feature
 *
 * Enhanced descriptors (8 words; or 4 without ATDS): des0 status/control
 * (OWN 31; TX IC 30, LS 29, FS 28, CIC 23:22, TER 21, TCH 20), des1 sizes
 * (13 bits each, buffer 2 at 28:16; RX also DIC 31, RER 15, RCH 14), des2
 * buffer 1, des3 buffer 2 or the next descriptor.  A core built with
 * normal descriptors (enh-desc=off) has TX control in des1 (IC 31, LS 30,
 * FS 29, TER 25, TCH 24) and 11-bit sizes; RX end of ring 25, chain 24.
 *
 * Receive frames carry their FCS and the length counts it: every driver
 * subtracts 4 (jz_mac_v13.c:1353 "length -= 4", U-Boot jz_recv) and none
 * sets ACS or CST (synopGMAC_pad_crc_strip_disable); with those set it is
 * stripped the way the core does.  Short frames are padded to 60 bytes.
 *
 * The PHY: the drivers scan all 32 MDIO addresses and take the first one
 * whose ID registers are not 0xffff (mii_probe, jz_mac_v13.c:848-870;
 * U-Boot synopGMAC_search_phy), so the address is free (phy-addr).  It
 * has negotiated 100 Mbit/s full duplex; BMCR reset and restart-autoneg
 * self-clear -- jz_mac_open() spins on BMCR.RESET with no timeout
 * (jz_mac_v13.c:1833, ingenic_mac.c:1733).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "net/eth.h"
#include "net/checksum.h"
#include "qom/object.h"
#include "system/dma.h"

#include <zlib.h>

#define TYPE_INGENIC_GMAC "ingenic-gmac"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicGmacState, INGENIC_GMAC)

#define GMAC_MMIO_SIZE  0x2000
#define MAC_REGS        (0x1000 / 4)
#define DMA_OFF         0x1000

/* MAC */
#define MAC_CONFIG      0x00
#define MAC_FILTER      0x04
#define MAC_GMII_ADDR   0x10
#define MAC_GMII_DATA   0x14
#define MAC_VERSION     0x20
#define MAC_DEBUG       0x24
#define MAC_INT_STATUS  0x38
#define MAC_ADDR0_HIGH  0x40
#define MAC_ADDR0_LOW   0x44
#define MAC_RGMII_CTRL  0xD8
#define MMC_CNTRL       0x100
#define MMC_INTR_MASK_RX 0x10C
#define MMC_INTR_MASK_TX 0x110
#define MMC_IPC_INTR_MASK_RX 0x200
#define MMC_START       0x100
#define MMC_END         0x300

#define CONFIG_RE       (1u << 2)
#define CONFIG_TE       (1u << 3)
#define CONFIG_ACS      (1u << 7)
#define CONFIG_CST      (1u << 25)

#define FILTER_PR       (1u << 0)
#define FILTER_HUC      (1u << 1)
#define FILTER_DBF      (1u << 5)
#define FILTER_RA       (1u << 31)

#define ADDR_HIGH_AE    (1u << 31)
#define ADDR_HIGH_SA    (1u << 30)

#define GMII_GB         (1u << 0)
#define GMII_GW         (1u << 1)

/* DMA, relative to DMA_OFF */
#define DMA_BUS_MODE    0x00
#define DMA_TX_POLL     0x04
#define DMA_RX_POLL     0x08
#define DMA_RX_BASE     0x0C
#define DMA_TX_BASE     0x10
#define DMA_STATUS      0x14
#define DMA_OPMODE      0x18
#define DMA_INTR_ENA    0x1C
#define DMA_MISSED      0x20
#define DMA_RIWT        0x24
#define DMA_AXI_BUS     0x28
#define DMA_AHB_STATUS  0x2C
#define DMA_CUR_TX_DESC 0x48
#define DMA_CUR_RX_DESC 0x4C
#define DMA_CUR_TX_BUF  0x50
#define DMA_CUR_RX_BUF  0x54
#define DMA_HW_FEATURE  0x58

#define BUS_SWR         (1u << 0)
#define BUS_DSL(v)      (((v) >> 2) & 0x1f)
#define BUS_ATDS        (1u << 7)
#define BUS_RESET_VALUE 0x00020100      /* PBL 1, after reset */

#define OP_SR           (1u << 1)
#define OP_ST           (1u << 13)
#define OP_FTF          (1u << 20)

#define ST_TI           (1u << 0)
#define ST_TPS          (1u << 1)
#define ST_TU           (1u << 2)
#define ST_TJT          (1u << 3)
#define ST_OVF          (1u << 4)
#define ST_UNF          (1u << 5)
#define ST_RI           (1u << 6)
#define ST_RU           (1u << 7)
#define ST_RPS          (1u << 8)
#define ST_RWT          (1u << 9)
#define ST_ETI          (1u << 10)
#define ST_FBI          (1u << 13)
#define ST_ERI          (1u << 14)
#define ST_AIS          (1u << 15)
#define ST_NIS          (1u << 16)
#define ST_NORMAL       (ST_TI | ST_TU | ST_RI | ST_ERI)
#define ST_ABNORMAL     (ST_TPS | ST_TJT | ST_OVF | ST_UNF | ST_RU | ST_RPS | \
                         ST_RWT | ST_ETI | ST_FBI)
#define ST_W1C          0x0001ffff
#define ST_RS_SHIFT     17
#define ST_TS_SHIFT     20
#define RS_WAITING      3
#define RS_SUSPENDED    4
#define TS_SUSPENDED    6

/* descriptors */
#define DES0_OWN        (1u << 31)
/* enhanced TDES0 */
#define ETDES0_IC       (1u << 30)
#define ETDES0_LS       (1u << 29)
#define ETDES0_FS       (1u << 28)
#define ETDES0_CIC(v)   (((v) >> 22) & 3)
#define ETDES0_TER      (1u << 21)
#define ETDES0_TCH      (1u << 20)
#define ETDES0_STATUS   0x0003ffff
/* enhanced RDES1 */
#define ERDES1_RER      (1u << 15)
#define ERDES1_RCH      (1u << 14)
/* normal TDES1 */
#define NTDES1_IC       (1u << 31)
#define NTDES1_LS       (1u << 30)
#define NTDES1_FS       (1u << 29)
#define NTDES1_CIC(v)   (((v) >> 27) & 3)
#define NTDES1_TER      (1u << 25)
#define NTDES1_TCH      (1u << 24)
/* normal RDES1 */
#define NRDES1_RER      (1u << 25)
#define NRDES1_RCH      (1u << 24)
#define RDES1_DIC       (1u << 31)
/* RDES0 status */
#define RDES0_FL_SHIFT  16
#define RDES0_ES        (1u << 15)
#define RDES0_DE        (1u << 14)
#define RDES0_FS        (1u << 9)
#define RDES0_LS        (1u << 8)
#define RDES0_FT        (1u << 5)

#define FRAME_MAX       16384           /* jumbo frames: U-Boot enables JE */
#define RX_RETRY_NS     (1000 * 1000)   /* re-fetch while RX is suspended */
#define DESC_GUARD      256

/* the PHY */
#define PHY_BMCR_RESET  0x8000
#define PHY_BMCR_PDOWN  0x0800
#define PHY_BMCR_ANRST  0x0200
#define PHY_BMCR_DEF    0x3100          /* AN on, 100 Mbit/s, full duplex */
#define PHY_BMSR        0x7849          /* 100/10 FD/HD, MF pre. sup., AN */
#define PHY_BMSR_LINK   0x0004
#define PHY_BMSR_ANDONE 0x0020

typedef struct {
    uint32_t des0, des1, des2, des3;
} GmacDesc;

struct IngenicGmacState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;
    QEMUTimer *rx_retry;

    uint32_t mac[MAC_REGS];
    uint32_t bus_mode, rx_base, tx_base, status, opmode, intr_ena, missed;
    uint32_t riwt, axi_bus;
    uint32_t cur_tx, cur_rx, cur_tx_buf, cur_rx_buf;
    bool tx_suspended, rx_suspended;

    uint16_t phy[32];

    uint8_t frame[FRAME_MAX];
    uint8_t rx_frame[FRAME_MAX + 4];
    uint32_t frame_len;
    bool frame_overflow;

    /* properties */
    uint32_t version;
    uint32_t phy_addr;
    uint32_t phy_id;
    bool enh_desc;
};

/* --- interrupts --------------------------------------------------------- */

/* NIS and AIS are the OR of their enabled sources (DMA register 7). */
static void gmac_update_irq(IngenicGmacState *s)
{
    s->status &= ~(ST_NIS | ST_AIS);
    if (s->status & s->intr_ena & ST_NORMAL) {
        s->status |= ST_NIS;
    }
    if (s->status & s->intr_ena & ST_ABNORMAL) {
        s->status |= ST_AIS;
    }
    qemu_set_irq(s->irq, ((s->status & s->intr_ena) & (ST_NIS | ST_AIS)) != 0);
}

static uint32_t gmac_status(IngenicGmacState *s)
{
    uint32_t v = s->status;

    if (s->opmode & OP_SR) {
        v |= (s->rx_suspended ? RS_SUSPENDED : RS_WAITING) << ST_RS_SHIFT;
    }
    if (s->opmode & OP_ST) {
        v |= TS_SUSPENDED << ST_TS_SHIFT;
    }
    return v;
}

/* --- PHY ---------------------------------------------------------------- */

static void gmac_phy_reset(IngenicGmacState *s)
{
    memset(s->phy, 0, sizeof(s->phy));
    s->phy[0] = PHY_BMCR_DEF;
    s->phy[4] = 0x01e1;                 /* ANAR: 100/10 FD/HD, CSMA */
}

static uint16_t gmac_phy_read(IngenicGmacState *s, int reg)
{
    const NetClientState *nc = qemu_get_queue(s->nic);
    const bool link = !nc->link_down && !(s->phy[0] & PHY_BMCR_PDOWN);

    switch (reg) {
    case 1:     /* BMSR */
        return PHY_BMSR | (link ? PHY_BMSR_LINK | PHY_BMSR_ANDONE : 0);
    case 2:
        return s->phy_id >> 16;
    case 3:
        return s->phy_id & 0xffff;
    case 5:     /* ANLPAR: the partner does the same, acknowledged */
        return link ? 0x45e1 : 0;
    case 6:     /* ANER: partner is AN able */
        return 0x0001;
    default:
        return s->phy[reg];
    }
}

static void gmac_phy_write(IngenicGmacState *s, int reg, uint16_t val)
{
    switch (reg) {
    case 0:
        if (val & PHY_BMCR_RESET) {
            gmac_phy_reset(s);          /* done at once: RESET reads 0 */
        } else {
            s->phy[0] = val & ~PHY_BMCR_ANRST;
        }
        break;
    case 1: case 2: case 3: case 5: case 6:
        break;                          /* read-only */
    default:
        s->phy[reg] = val;
        break;
    }
}

static void gmac_mdio(IngenicGmacState *s, uint32_t val)
{
    const int pa = (val >> 11) & 0x1f;
    const int gr = (val >> 6) & 0x1f;

    s->mac[MAC_GMII_ADDR / 4] = val & ~GMII_GB;   /* transfers finish now */
    if (!(val & GMII_GB)) {
        return;                                    /* CR update only */
    }
    if (val & GMII_GW) {
        if (pa == s->phy_addr) {
            gmac_phy_write(s, gr, s->mac[MAC_GMII_DATA / 4] & 0xffff);
        }
    } else {
        s->mac[MAC_GMII_DATA / 4] =
            pa == s->phy_addr ? gmac_phy_read(s, gr) : 0xffff;
    }
}

/* --- descriptors -------------------------------------------------------- */

static bool gmac_enh(IngenicGmacState *s)
{
    return s->enh_desc || (s->bus_mode & BUS_ATDS);
}

static void gmac_read_desc(hwaddr addr, GmacDesc *d)
{
    dma_memory_read(&address_space_memory, addr, d, sizeof(*d),
                    MEMTXATTRS_UNSPECIFIED);
    d->des0 = le32_to_cpu(d->des0);
    d->des1 = le32_to_cpu(d->des1);
    d->des2 = le32_to_cpu(d->des2);
    d->des3 = le32_to_cpu(d->des3);
}

static void gmac_write_des0(hwaddr addr, uint32_t des0)
{
    des0 = cpu_to_le32(des0);
    dma_memory_write(&address_space_memory, addr, &des0, 4,
                     MEMTXATTRS_UNSPECIFIED);
}

/* Ring stride: 4 or 8 (ATDS) words, then DSL words of the driver's. */
static uint32_t gmac_ring_next(IngenicGmacState *s, uint32_t cur)
{
    return cur + ((s->bus_mode & BUS_ATDS) ? 32 : 16) +
           4 * BUS_DSL(s->bus_mode);
}

static uint32_t gmac_size1(IngenicGmacState *s, uint32_t des1)
{
    return gmac_enh(s) ? des1 & 0x1fff : des1 & 0x7ff;
}

static uint32_t gmac_size2(IngenicGmacState *s, uint32_t des1)
{
    return gmac_enh(s) ? (des1 >> 16) & 0x1fff : (des1 >> 11) & 0x7ff;
}

/* --- transmit ----------------------------------------------------------- */

static void gmac_tx(IngenicGmacState *s)
{
    const bool enh = gmac_enh(s);
    int guard = DESC_GUARD;

    if (!(s->opmode & OP_ST)) {
        return;
    }
    s->tx_suspended = false;
    while (guard--) {
        GmacDesc d;
        const uint32_t at = s->cur_tx;

        gmac_read_desc(at, &d);
        if (!(d.des0 & DES0_OWN)) {
            s->status |= ST_TU;
            s->tx_suspended = true;
            break;
        }
        const uint32_t ctl = enh ? d.des0 : d.des1;
        const bool fs = ctl & (enh ? ETDES0_FS : NTDES1_FS);
        const bool ls = ctl & (enh ? ETDES0_LS : NTDES1_LS);
        const bool ic = ctl & (enh ? ETDES0_IC : NTDES1_IC);
        const bool ter = ctl & (enh ? ETDES0_TER : NTDES1_TER);
        const bool tch = ctl & (enh ? ETDES0_TCH : NTDES1_TCH);
        const int cic = enh ? ETDES0_CIC(d.des0) : NTDES1_CIC(d.des1);

        if (fs) {
            s->frame_len = 0;
            s->frame_overflow = false;
        }
        const uint32_t len1 = gmac_size1(s, d.des1);
        const uint32_t len2 = tch ? 0 : gmac_size2(s, d.des1);
        if (s->frame_len + len1 + len2 > FRAME_MAX) {
            s->frame_overflow = true;
        }
        if (!s->frame_overflow) {
            dma_memory_read(&address_space_memory, d.des2,
                            s->frame + s->frame_len, len1,
                            MEMTXATTRS_UNSPECIFIED);
            s->frame_len += len1;
            if (len2) {
                dma_memory_read(&address_space_memory, d.des3,
                                s->frame + s->frame_len, len2,
                                MEMTXATTRS_UNSPECIFIED);
                s->frame_len += len2;
            }
        }
        s->cur_tx_buf = d.des2;
        if (ls) {
            if (s->frame_overflow) {
                qemu_log_mask(LOG_GUEST_ERROR, "ingenic-gmac: TX frame over "
                              "%d bytes dropped\n", FRAME_MAX);
            } else if (s->mac[MAC_CONFIG / 4] & CONFIG_TE) {
                if (cic) {
                    net_checksum_calculate(s->frame, s->frame_len,
                                           cic == 1 ? CSUM_IP : CSUM_ALL);
                }
                qemu_send_packet(qemu_get_queue(s->nic), s->frame,
                                 s->frame_len);
            }
            s->frame_len = 0;
            s->frame_overflow = false;
            if (ic) {
                s->status |= ST_TI;
            }
        }
        /* Write back: OWN and the status bits clear, no errors. */
        gmac_write_des0(at, enh ? d.des0 & ~(DES0_OWN | ETDES0_STATUS) : 0);
        if (ter) {
            s->cur_tx = s->tx_base;
        } else if (tch) {
            s->cur_tx = d.des3;
        } else {
            s->cur_tx = gmac_ring_next(s, at);
        }
    }
    gmac_update_irq(s);
}

/* --- receive ------------------------------------------------------------ */

static bool gmac_rx_on(IngenicGmacState *s)
{
    return (s->opmode & OP_SR) && (s->mac[MAC_CONFIG / 4] & CONFIG_RE);
}

static void gmac_rx_suspend(IngenicGmacState *s)
{
    if (!s->rx_suspended) {
        s->rx_suspended = true;
        s->status |= ST_RU;
        gmac_update_irq(s);
    }
    /* On silicon a suspended receiver re-fetches its descriptor when the
     * next frame arrives; the drivers count on it (none enables RU), and
     * QEMU holds that frame until can_receive says yes, so look again. */
    timer_mod(s->rx_retry, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              RX_RETRY_NS);
}

static bool gmac_can_receive(NetClientState *nc)
{
    IngenicGmacState *s = qemu_get_nic_opaque(nc);
    GmacDesc d;

    if (!gmac_rx_on(s)) {
        return false;
    }
    gmac_read_desc(s->cur_rx, &d);
    if (!(d.des0 & DES0_OWN)) {
        gmac_rx_suspend(s);
        return false;
    }
    s->rx_suspended = false;
    return true;
}

static void gmac_rx_retry(void *opaque)
{
    IngenicGmacState *s = opaque;

    if (gmac_can_receive(qemu_get_queue(s->nic))) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

static bool gmac_addr_match(IngenicGmacState *s, const uint8_t *da)
{
    for (int i = 0; i < 32; i++) {
        const int hi = i < 16 ? 0x40 + 8 * i : 0x800 + 8 * (i - 16);
        const uint32_t h = s->mac[hi / 4], l = s->mac[hi / 4 + 1];
        const uint8_t a[ETH_ALEN] = { l, l >> 8, l >> 16, l >> 24, h, h >> 8 };

        if (i && (!(h & ADDR_HIGH_AE) || (h & ADDR_HIGH_SA))) {
            continue;
        }
        if (!memcmp(da, a, ETH_ALEN)) {
            return true;
        }
    }
    return false;
}

/* Perfect filter for unicast; broadcast unless DBF; every multicast (the
 * hash would only narrow it, and the stack filters again). */
static bool gmac_accept(IngenicGmacState *s, const uint8_t *da)
{
    const uint32_t flt = s->mac[MAC_FILTER / 4];

    if (flt & (FILTER_PR | FILTER_RA)) {
        return true;
    }
    if (is_broadcast_ether_addr(da)) {
        return !(flt & FILTER_DBF);
    }
    if (da[0] & 1) {
        return true;
    }
    return (flt & FILTER_HUC) || gmac_addr_match(s, da);
}

static ssize_t gmac_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    IngenicGmacState *s = qemu_get_nic_opaque(nc);
    const bool enh = gmac_enh(s);
    const uint32_t cfg = s->mac[MAC_CONFIG / 4];
    uint8_t *frame = s->rx_frame;
    size_t len, done = 0;
    bool first = true;
    int guard = DESC_GUARD;

    if (!gmac_rx_on(s) || size < ETH_HLEN || size > FRAME_MAX ||
        !gmac_accept(s, buf)) {
        return size;
    }
    /* The wire frame: padded to the minimum, then its FCS, which the
     * core keeps unless CST (all frames) or ACS (802.3 length frames). */
    memcpy(frame, buf, size);
    len = size;
    if (len < ETH_ZLEN) {
        memset(frame + len, 0, ETH_ZLEN - len);
        len = ETH_ZLEN;
    }
    const uint16_t type = lduw_be_p(frame + 12);
    if (!((cfg & CONFIG_CST) || ((cfg & CONFIG_ACS) && type < 0x600))) {
        stl_le_p(frame + len, crc32(0, frame, len));
        len += 4;
    }

    while (done < len && guard--) {
        GmacDesc d;
        const uint32_t at = s->cur_rx;

        gmac_read_desc(at, &d);
        if (!(d.des0 & DES0_OWN)) {
            /* Out of descriptors mid-frame: the rest is lost. */
            s->missed++;
            gmac_rx_suspend(s);
            return size;
        }
        const bool rer = d.des1 & (enh ? ERDES1_RER : NRDES1_RER);
        const bool rch = d.des1 & (enh ? ERDES1_RCH : NRDES1_RCH);
        const uint32_t room1 = gmac_size1(s, d.des1);
        const uint32_t room2 = rch ? 0 : gmac_size2(s, d.des1);
        uint32_t n = MIN(room1, len - done);

        dma_memory_write(&address_space_memory, d.des2, frame + done, n,
                         MEMTXATTRS_UNSPECIFIED);
        done += n;
        if (done < len && room2) {
            n = MIN(room2, len - done);
            dma_memory_write(&address_space_memory, d.des3, frame + done, n,
                             MEMTXATTRS_UNSPECIFIED);
            done += n;
        }
        s->cur_rx_buf = d.des2;

        uint32_t des0 = 0;
        if (first) {
            des0 |= RDES0_FS;
            first = false;
        }
        if (done >= len) {
            des0 |= RDES0_LS | ((uint32_t)len << RDES0_FL_SHIFT);
            if (type >= 0x600) {
                des0 |= RDES0_FT;
            }
            if (!(d.des1 & RDES1_DIC)) {
                s->status |= ST_RI;
            }
        }
        gmac_write_des0(at, des0);
        if (rer) {
            s->cur_rx = s->rx_base;
        } else if (rch) {
            s->cur_rx = d.des3;
        } else {
            s->cur_rx = gmac_ring_next(s, at);
        }
        if (!room1 && !room2) {
            break;                      /* a zero-sized ring: give up */
        }
    }
    if (done < len) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-gmac: RX frame of %zu bytes "
                      "did not fit the descriptors, dropped\n", size);
        s->missed++;
    }
    gmac_update_irq(s);
    return size;
}

/* --- registers ---------------------------------------------------------- */

static void gmac_mac_defaults(IngenicGmacState *s)
{
    const uint8_t *m = s->conf.macaddr.a;

    memset(s->mac, 0, sizeof(s->mac));
    for (int i = 1; i < 16; i++) {
        s->mac[(0x40 + 8 * i) / 4] = 0x0000ffff;
        s->mac[(0x44 + 8 * i) / 4] = 0xffffffff;
    }
    for (int i = 0; i < 16; i++) {
        s->mac[(0x800 + 8 * i) / 4] = 0x0000ffff;
        s->mac[(0x804 + 8 * i) / 4] = 0xffffffff;
    }
    /* ADDR0 starts as the NIC's address, as if a boot loader left it. */
    s->mac[MAC_ADDR0_HIGH / 4] = ADDR_HIGH_AE | m[4] | (m[5] << 8);
    s->mac[MAC_ADDR0_LOW / 4] = m[0] | (m[1] << 8) | (m[2] << 16) |
                                ((uint32_t)m[3] << 24);
}

/* DmaBusMode.SWR: DMA and MAC back to reset values, done at once. */
static void gmac_soft_reset(IngenicGmacState *s)
{
    timer_del(s->rx_retry);
    gmac_mac_defaults(s);
    s->bus_mode = BUS_RESET_VALUE;
    s->rx_base = s->tx_base = 0;
    s->cur_tx = s->cur_rx = s->cur_tx_buf = s->cur_rx_buf = 0;
    s->status = s->opmode = s->intr_ena = s->missed = 0;
    s->riwt = s->axi_bus = 0;
    s->tx_suspended = s->rx_suspended = false;
    s->frame_len = 0;
    s->frame_overflow = false;
    gmac_update_irq(s);
}

static uint32_t gmac_hw_feature(IngenicGmacState *s)
{
    /* MII (10/100), half duplex, 64-bit hash, extra MAC addresses, MMC,
     * enhanced descriptors if built so, active interface RMII (4). */
    return (1u << 0) | (1u << 2) | (1u << 4) | (1u << 5) | (1u << 11) |
           (s->enh_desc ? 1u << 24 : 0) | (4u << 28);
}

static uint64_t gmac_mac_read(IngenicGmacState *s, hwaddr off)
{
    switch (off) {
    case MAC_VERSION:
        return s->version;
    case MAC_DEBUG:
    case MAC_INT_STATUS:
    case MAC_RGMII_CTRL:
        return 0;
    case MAC_ADDR0_HIGH:
        return s->mac[off / 4] | ADDR_HIGH_AE;
    case MMC_START ... MMC_END - 1:
        if (off == MMC_CNTRL || off == MMC_INTR_MASK_RX ||
            off == MMC_INTR_MASK_TX || off == MMC_IPC_INTR_MASK_RX) {
            return s->mac[off / 4];
        }
        return 0;                       /* counters and their status */
    default:
        return s->mac[off / 4];
    }
}

static void gmac_mac_write(IngenicGmacState *s, hwaddr off, uint32_t val)
{
    switch (off) {
    case MAC_GMII_ADDR:
        gmac_mdio(s, val);
        break;
    case MAC_GMII_DATA:
        s->mac[off / 4] = val & 0xffff;
        break;
    case MAC_VERSION:
    case MAC_DEBUG:
    case MAC_INT_STATUS:
    case MAC_RGMII_CTRL:
        break;
    case MMC_START ... MMC_END - 1:
        if (off == MMC_CNTRL || off == MMC_INTR_MASK_RX ||
            off == MMC_INTR_MASK_TX || off == MMC_IPC_INTR_MASK_RX) {
            s->mac[off / 4] = val & (off == MMC_CNTRL ? ~0x1u : ~0u);
        }
        break;
    case MAC_CONFIG: {
        const bool rx_was_on = gmac_rx_on(s);
        s->mac[off / 4] = val;
        if (!rx_was_on && gmac_rx_on(s)) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    }
    default:
        s->mac[off / 4] = val;
        break;
    }
}

static uint64_t gmac_dma_read(IngenicGmacState *s, hwaddr off)
{
    switch (off) {
    case DMA_BUS_MODE:
        return s->bus_mode;
    case DMA_RX_BASE:
        return s->rx_base;
    case DMA_TX_BASE:
        return s->tx_base;
    case DMA_STATUS:
        return gmac_status(s);
    case DMA_OPMODE:
        return s->opmode;
    case DMA_INTR_ENA:
        return s->intr_ena;
    case DMA_MISSED: {
        const uint32_t v = MIN(s->missed, 0xffff) |
                           (s->missed > 0xffff ? 1u << 16 : 0);
        s->missed = 0;
        return v;
    }
    case DMA_RIWT:
        return s->riwt;
    case DMA_AXI_BUS:
        return s->axi_bus;
    case DMA_CUR_TX_DESC:
        return s->cur_tx;
    case DMA_CUR_RX_DESC:
        return s->cur_rx;
    case DMA_CUR_TX_BUF:
        return s->cur_tx_buf;
    case DMA_CUR_RX_BUF:
        return s->cur_rx_buf;
    case DMA_HW_FEATURE:
        return gmac_hw_feature(s);
    case DMA_TX_POLL:
    case DMA_RX_POLL:
    case DMA_AHB_STATUS:
        return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic-gmac: read DMA+0x%" HWADDR_PRIx
                      "\n", off);
        return 0;
    }
}

static void gmac_dma_write(IngenicGmacState *s, hwaddr off, uint32_t val)
{
    switch (off) {
    case DMA_BUS_MODE:
        if (val & BUS_SWR) {
            gmac_soft_reset(s);
        } else {
            s->bus_mode = val;
        }
        break;
    case DMA_TX_POLL:
        gmac_tx(s);
        break;
    case DMA_RX_POLL:
        if (gmac_can_receive(qemu_get_queue(s->nic))) {
            s->rx_suspended = false;
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case DMA_RX_BASE:
        s->rx_base = s->cur_rx = val;
        break;
    case DMA_TX_BASE:
        s->tx_base = s->cur_tx = val;
        break;
    case DMA_STATUS:
        s->status &= ~(val & ST_W1C);
        gmac_update_irq(s);
        break;
    case DMA_OPMODE: {
        const uint32_t old = s->opmode;

        s->opmode = val & ~OP_FTF;      /* the flush is done at once */
        if ((old & OP_ST) && !(val & OP_ST)) {
            s->status |= ST_TPS;
            s->tx_suspended = false;
        }
        if ((old & OP_SR) && !(val & OP_SR)) {
            s->status |= ST_RPS;
            s->rx_suspended = false;
            timer_del(s->rx_retry);
        }
        if (val & OP_ST) {
            gmac_tx(s);
        }
        if (!(old & OP_SR) && (val & OP_SR)) {
            s->rx_suspended = false;
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        gmac_update_irq(s);
        break;
    }
    case DMA_INTR_ENA:
        s->intr_ena = val & 0x0001e7ff;
        gmac_update_irq(s);
        break;
    case DMA_RIWT:
        s->riwt = val & 0xff;
        break;
    case DMA_AXI_BUS:
        s->axi_bus = val;
        break;
    case DMA_MISSED:
    case DMA_AHB_STATUS:
    case DMA_CUR_TX_DESC ... DMA_HW_FEATURE:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic-gmac: write DMA+0x%" HWADDR_PRIx
                      " = 0x%08x\n", off, val);
        break;
    }
}

static uint64_t gmac_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicGmacState *s = opaque;

    return off < DMA_OFF ? gmac_mac_read(s, off & ~3)
                         : gmac_dma_read(s, (off - DMA_OFF) & ~3);
}

static void gmac_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    IngenicGmacState *s = opaque;

    if (off < DMA_OFF) {
        gmac_mac_write(s, off & ~3, val);
    } else {
        gmac_dma_write(s, (off - DMA_OFF) & ~3, val);
    }
}

static const MemoryRegionOps gmac_ops = {
    .read = gmac_read,
    .write = gmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* --- device ------------------------------------------------------------- */

static NetClientInfo gmac_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = gmac_can_receive,
    .receive = gmac_receive,
};

static void gmac_realize(DeviceState *dev, Error **errp)
{
    IngenicGmacState *s = INGENIC_GMAC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->phy_addr > 31) {
        error_setg(errp, "ingenic-gmac: phy-addr must be 0..31");
        return;
    }
    memory_region_init_io(&s->mmio, OBJECT(s), &gmac_ops, s,
                          TYPE_INGENIC_GMAC, GMAC_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    s->rx_retry = timer_new_ns(QEMU_CLOCK_VIRTUAL, gmac_rx_retry, s);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&gmac_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void gmac_reset(DeviceState *dev)
{
    IngenicGmacState *s = INGENIC_GMAC(dev);

    gmac_phy_reset(s);
    gmac_soft_reset(s);
}

static const VMStateDescription vmstate_ingenic_gmac = {
    .name = TYPE_INGENIC_GMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(mac, IngenicGmacState, MAC_REGS),
        VMSTATE_UINT32(bus_mode, IngenicGmacState),
        VMSTATE_UINT32(rx_base, IngenicGmacState),
        VMSTATE_UINT32(tx_base, IngenicGmacState),
        VMSTATE_UINT32(status, IngenicGmacState),
        VMSTATE_UINT32(opmode, IngenicGmacState),
        VMSTATE_UINT32(intr_ena, IngenicGmacState),
        VMSTATE_UINT32(missed, IngenicGmacState),
        VMSTATE_UINT32(riwt, IngenicGmacState),
        VMSTATE_UINT32(axi_bus, IngenicGmacState),
        VMSTATE_UINT32(cur_tx, IngenicGmacState),
        VMSTATE_UINT32(cur_rx, IngenicGmacState),
        VMSTATE_UINT32(cur_tx_buf, IngenicGmacState),
        VMSTATE_UINT32(cur_rx_buf, IngenicGmacState),
        VMSTATE_BOOL(tx_suspended, IngenicGmacState),
        VMSTATE_BOOL(rx_suspended, IngenicGmacState),
        VMSTATE_UINT16_ARRAY(phy, IngenicGmacState, 32),
        VMSTATE_UINT32(frame_len, IngenicGmacState),
        VMSTATE_BOOL(frame_overflow, IngenicGmacState),
        VMSTATE_BUFFER(frame, IngenicGmacState),
        VMSTATE_TIMER_PTR(rx_retry, IngenicGmacState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property gmac_props[] = {
    DEFINE_NIC_PROPERTIES(IngenicGmacState, conf),
    /* GmacVersion: Synopsys 3.7a; ATDS needs 3.50a or later */
    DEFINE_PROP_UINT32("version", IngenicGmacState, version, 0x00001037),
    /* the PHY's MDIO address; the drivers take whatever answers first */
    DEFINE_PROP_UINT32("phy-addr", IngenicGmacState, phy_addr, 0),
    /* IC+ IP101G, what the boards carry (U-Boot isvp_t31.h) */
    DEFINE_PROP_UINT32("phy-id", IngenicGmacState, phy_id, 0x02430c54),
    /* enhanced descriptors even with 4-word ones (no ATDS) */
    DEFINE_PROP_BOOL("enh-desc", IngenicGmacState, enh_desc, true),
};

static void gmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = gmac_realize;
    device_class_set_legacy_reset(dc, gmac_reset);
    device_class_set_props(dc, gmac_props);
    dc->vmsd = &vmstate_ingenic_gmac;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo gmac_info = {
    .name          = TYPE_INGENIC_GMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicGmacState),
    .class_init    = gmac_class_init,
};

static void gmac_register_types(void)
{
    type_register_static(&gmac_info);
}

type_init(gmac_register_types)
