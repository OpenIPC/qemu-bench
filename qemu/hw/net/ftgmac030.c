/*
 * Faraday FTGMAC030 Gigabit Ethernet MAC emulation.
 *
 * Found on Goke GK7206 ("xmorca", vendor-tagged "lotus") at 0x100d0000,
 * driven by the vendor Linux 5.10 driver drivers/net/ethernet/ftgmac030/
 * (compatible "lotus,ftgmac030").  Register map, descriptor format and
 * interrupt bits follow that driver's regs.h.  Structure loosely follows
 * QEMU's ftgmac100 model (same Faraday family, different register map).
 *
 * Modelled:
 *   - MAC registers (MACCR with self-clearing SW_RST, MAC address, MAHT
 *     multicast hash, RBSR, TPAFCR, FEAR fifo sizes, DMAFIFOS reporting
 *     both FIFOs empty so the driver's down() polls terminate).
 *   - Normal-priority TX ring (NPTXR_BADR / NPTXPD poll demand), 16-byte
 *     descriptors, multi-segment frames (FTS..LTS), EDOTR wrap, TX
 *     checksum offload (TXDES1 IP/TCP/UDP) and VLAN tag insertion.
 *   - RX ring (RXR_BADR), 16-byte descriptors, RBSR-sized buffers with
 *     multi-descriptor frames, EDORR wrap, 4-byte FCS appended (the driver
 *     strips it), unicast / broadcast / multicast-hash / promiscuous
 *     filtering.
 *   - ISR (write-1-to-clear) / IER, one level IRQ line.
 *   - MDIO via PHYCR/PHYDATA inside the MAC block (the DT gives no separate
 *     mdio resource) with the internal 10/100 ePHY at address 0: link up,
 *     autoneg complete, 100 Mb/s full duplex link partner.  Clause-45-over-22
 *     (regs 13/14) reads return 0 so the driver leaves EEE off.
 *
 * Not modelled: high-priority TX ring, interrupt mitigation timers, PTP
 * (FEAR reports no PTP), flow control, wake-on-LAN, statistics counters.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/host-utils.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "net/net.h"
#include "net/eth.h"
#include "net/checksum.h"
#include "system/dma.h"
#include <zlib.h> /* crc32 */

#define TYPE_FTGMAC030 "ftgmac030"
OBJECT_DECLARE_SIMPLE_TYPE(Ftgmac030State, FTGMAC030)

#define FTGMAC030_MMIO_SIZE     0x1000

/* Registers */
#define REG_ISR                 0x00
#define REG_IER                 0x04
#define REG_MAC_MADR            0x08
#define REG_MAC_LADR            0x0c
#define REG_MAHT0               0x10
#define REG_MAHT1               0x14
#define REG_NPTXPD              0x18
#define REG_RXPD                0x1c
#define REG_NPTXR_BADR          0x20
#define REG_RXR_BADR            0x24
#define REG_HPTXPD              0x28
#define REG_HPTXR_BADR          0x2c
#define REG_DBLAC               0x3c
#define REG_DMAFIFOS            0x40
#define REG_TPAFCR              0x48
#define REG_RBSR                0x4c
#define REG_MACCR               0x50
#define REG_MACSR               0x54
#define REG_PHYCR               0x60
#define REG_PHYDATA             0x64
#define REG_NPTXR_PTR           0x90
#define REG_HPTXR_PTR           0x94
#define REG_RXR_PTR             0x98
#define REG_REVR                0xf4
#define REG_FEAR                0xf8

/* ISR / IER */
#define INT_RPKT_BUF            (1 << 0)
#define INT_RPKT_FIFO           (1 << 1)
#define INT_NO_RXBUF            (1 << 2)
#define INT_RPKT_LOST           (1 << 3)
#define INT_XPKT_ETH            (1 << 4)
#define INT_XPKT_FIFO           (1 << 5)
#define INT_NO_NPTXBUF          (1 << 6)
#define INT_XPKT_LOST           (1 << 7)
#define INT_AHB_ERR             (1 << 8)

/* DMAFIFOS */
#define DMAFIFOS_RXFIFO_EMPTY   (1 << 26)
#define DMAFIFOS_TXFIFO_EMPTY   (1 << 27)

/* MACCR */
#define MACCR_TXDMA_EN          (1 << 0)
#define MACCR_RXDMA_EN          (1 << 1)
#define MACCR_TXMAC_EN          (1 << 2)
#define MACCR_RXMAC_EN          (1 << 3)
#define MACCR_RX_ALL            (1 << 8)
#define MACCR_HT_MULTI_EN       (1 << 9)
#define MACCR_RX_MULTIPKT       (1 << 10)
#define MACCR_RX_BROADPKT       (1 << 11)
#define MACCR_JUMBO_LF          (1 << 13)
#define MACCR_SW_RST            (1u << 31)

/* PHYCR / PHYDATA */
#define PHYCR_PHYAD(v)          (((v) >> 16) & 0x1f)
#define PHYCR_REGAD(v)          (((v) >> 21) & 0x1f)
#define PHYCR_MIIRD             (1 << 26)
#define PHYCR_MIIWR             (1 << 27)

/* TX descriptor */
#define TXDES0_TXBUF_SIZE(x)    ((x) & 0x3fff)
#define TXDES0_EDOTR            (1 << 15)
#define TXDES0_LTS              (1 << 28)
#define TXDES0_FTS              (1 << 29)
#define TXDES0_TXDMA_OWN        (1u << 31)
#define TXDES1_VLANTAG_CI(x)    ((x) & 0xffff)
#define TXDES1_INS_VLANTAG      (1 << 16)
#define TXDES1_TCP_CHKSUM       (1 << 17)
#define TXDES1_UDP_CHKSUM       (1 << 18)
#define TXDES1_IP_CHKSUM        (1 << 19)
#define TXDES1_TX2FIC           (1 << 30)
#define TXDES1_TXIC             (1u << 31)

/* RX descriptor */
#define RXDES0_VDBC             0x3fff
#define RXDES0_EDORR            (1 << 15)
#define RXDES0_MULTICAST        (1 << 16)
#define RXDES0_BROADCAST        (1 << 17)
#define RXDES0_FTL              (1 << 20)
#define RXDES0_LRS              (1 << 28)
#define RXDES0_FRS              (1 << 29)
#define RXDES0_RXPKT_RDY        (1u << 31)

#define DESC_SIZE               16
#define MAX_RING_WALK           4096    /* driver max ring size */
#define MAX_FRAME               10240   /* jumbo (9234) + slack */

/* Reset values */
#define FEAR_RESET              0x00000033  /* 16 KiB TX/RX FIFO, no PTP */
#define REVR_RESET              0x00030000
#define RBSR_RESET              0x640
#define DBLAC_RESET             0x00022f00

/*
 * The SoC's internal 10/100 PHY, "ePHY" 0x0243991f (the SDK kernel's
 * lotus/drivers/net/phy/ephy_phy.c, CONFIG_LOTUS_PHY).  That driver is
 * genphy plus LED / trim writes through regs 0x12..0x14, which read 0 and
 * drop writes here.  ID1 must not be 0: U-Boot's ftgmac030 driver scans
 * MII_PHYSID1 and takes 0x0000 / 0xffff for "no PHY present".
 */
#define PHY_ID1                 0x0243
#define PHY_ID2                 0x991f
#define BMCR_RESET              0x8000
#define BMCR_ANRESTART          0x0200
#define BMCR_PDOWN              0x0800
#define BMCR_DEFAULT            0x3100      /* 100M, AN enabled, FD */
#define BMSR_LSTATUS            0x0004
#define BMSR_ANEGCOMPLETE       0x0020
#define BMSR_BASE               0x7809      /* 10/100 HD/FD, AN able, ext */

typedef struct Ftgmac030State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    uint32_t isr;
    uint32_t ier;
    uint32_t madr, ladr;
    uint32_t maht0, maht1;
    uint32_t nptxr_badr, rxr_badr, hptxr_badr;
    uint32_t tx_ptr, rx_ptr;
    uint32_t maccr;
    uint32_t phycr, phydata;
    uint32_t rbsr;
    uint32_t fear;
    uint32_t regs[FTGMAC030_MMIO_SIZE / 4];   /* plain storage for the rest */

    uint8_t phy_addr;
    uint16_t phy_bmcr;
    uint16_t phy_anar;
    uint16_t phy_mmd_ctrl;

    uint8_t frame[MAX_FRAME];
} Ftgmac030State;

static void ftgmac030_update_irq(Ftgmac030State *s)
{
    qemu_set_irq(s->irq, !!(s->isr & s->ier));
}

static bool ftgmac030_rx_enabled(Ftgmac030State *s)
{
    uint32_t m = MACCR_RXDMA_EN | MACCR_RXMAC_EN;

    return (s->maccr & m) == m;
}

/* ---- PHY ---- */

static void ftgmac030_phy_reset(Ftgmac030State *s)
{
    s->phy_bmcr = BMCR_DEFAULT;
    s->phy_anar = 0x01e1;
    s->phy_mmd_ctrl = 0;
}

static uint16_t ftgmac030_phy_read(Ftgmac030State *s, int reg)
{
    bool link = !(s->phy_bmcr & BMCR_PDOWN);

    switch (reg) {
    case 0:     /* BMCR */
        return s->phy_bmcr;
    case 1:     /* BMSR */
        return BMSR_BASE | (link ? (BMSR_LSTATUS | BMSR_ANEGCOMPLETE) : 0);
    case 2:
        return PHY_ID1;
    case 3:
        return PHY_ID2;
    case 4:     /* ANAR */
        return s->phy_anar;
    case 5:     /* ANLPAR: 10/100 HD/FD, ACK */
        return link ? 0x45e1 : 0;
    case 6:     /* ANER: link partner AN able */
        return 0x0001;
    case 13:    /* MMD access control */
        return s->phy_mmd_ctrl;
    case 14:    /* MMD data: no EEE, nothing behind it */
    default:
        return 0;
    }
}

static void ftgmac030_phy_write(Ftgmac030State *s, int reg, uint16_t val)
{
    switch (reg) {
    case 0:
        if (val & BMCR_RESET) {
            ftgmac030_phy_reset(s);
        } else {
            s->phy_bmcr = val & ~BMCR_ANRESTART;
        }
        break;
    case 4:
        s->phy_anar = val;
        break;
    case 13:
        s->phy_mmd_ctrl = val;
        break;
    default:
        break;
    }
}

static void ftgmac030_mdio(Ftgmac030State *s)
{
    int phyad = PHYCR_PHYAD(s->phycr);
    int reg = PHYCR_REGAD(s->phycr);

    if (s->phycr & PHYCR_MIIRD) {
        uint16_t v = (phyad == s->phy_addr) ? ftgmac030_phy_read(s, reg)
                                            : 0xffff;
        s->phydata = (s->phydata & 0xffff) | ((uint32_t)v << 16);
        s->phycr &= ~PHYCR_MIIRD;
    }
    if (s->phycr & PHYCR_MIIWR) {
        if (phyad == s->phy_addr) {
            ftgmac030_phy_write(s, reg, s->phydata & 0xffff);
        }
        s->phycr &= ~PHYCR_MIIWR;
    }
}

/* ---- TX ---- */

static int ftgmac030_insert_vlan(Ftgmac030State *s, int len, uint16_t tci)
{
    if (len + 4 > sizeof(s->frame) || len < 12) {
        return len;
    }
    memmove(s->frame + 16, s->frame + 12, len - 12);
    s->frame[12] = 0x81;
    s->frame[13] = 0x00;
    s->frame[14] = tci >> 8;
    s->frame[15] = tci & 0xff;
    return len + 4;
}

static void ftgmac030_do_tx(Ftgmac030State *s)
{
    uint32_t m = MACCR_TXDMA_EN | MACCR_TXMAC_EN;
    int frame_size = 0;
    uint32_t first_flags = 0;
    int n;

    if ((s->maccr & m) != m) {
        return;
    }

    for (n = 0; n < MAX_RING_WALK; n++) {
        uint32_t d[4];
        uint32_t len;

        if (dma_memory_read(&address_space_memory, s->tx_ptr, d, sizeof(d),
                            MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR, "ftgmac030: bad TX desc @0x%x\n",
                          s->tx_ptr);
            s->isr |= INT_AHB_ERR;
            break;
        }
        d[0] = le32_to_cpu(d[0]);
        d[1] = le32_to_cpu(d[1]);
        d[3] = le32_to_cpu(d[3]);

        if (!(d[0] & TXDES0_TXDMA_OWN)) {
            break;
        }
        if (d[0] & TXDES0_FTS) {
            frame_size = 0;
            first_flags = d[1];
        }

        len = TXDES0_TXBUF_SIZE(d[0]);
        if (frame_size + len > sizeof(s->frame)) {
            qemu_log_mask(LOG_GUEST_ERROR, "ftgmac030: TX frame too big\n");
            s->isr |= INT_XPKT_LOST;
            len = sizeof(s->frame) - frame_size;
        }
        if (dma_memory_read(&address_space_memory, d[3],
                            s->frame + frame_size, len,
                            MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR, "ftgmac030: bad TX buf @0x%x\n",
                          d[3]);
            s->isr |= INT_AHB_ERR;
            break;
        }
        frame_size += len;

        if (d[0] & TXDES0_LTS) {
            int csum = 0;

            if ((first_flags & TXDES1_INS_VLANTAG) && frame_size >= 14 &&
                lduw_be_p(s->frame + 12) != ETH_P_VLAN) {
                frame_size = ftgmac030_insert_vlan(s, frame_size,
                                    TXDES1_VLANTAG_CI(first_flags));
            }
            if (first_flags & TXDES1_IP_CHKSUM) {
                csum |= CSUM_IP;
            }
            if (first_flags & TXDES1_TCP_CHKSUM) {
                csum |= CSUM_TCP;
            }
            if (first_flags & TXDES1_UDP_CHKSUM) {
                csum |= CSUM_UDP;
            }
            if (csum) {
                net_checksum_calculate(s->frame, frame_size, csum);
            }
            qemu_send_packet(qemu_get_queue(s->nic), s->frame, frame_size);
            frame_size = 0;
            if (d[1] & TXDES1_TXIC) {
                s->isr |= INT_XPKT_ETH;
            }
        }
        if (d[1] & TXDES1_TX2FIC) {
            s->isr |= INT_XPKT_FIFO;
        }

        /* Hand the descriptor back: only OWN changes, size/flags stay. */
        d[0] &= ~TXDES0_TXDMA_OWN;
        d[0] = cpu_to_le32(d[0]);
        if (dma_memory_write(&address_space_memory, s->tx_ptr, &d[0], 4,
                             MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "ftgmac030: TX desc writeback failed @0x%x\n",
                          s->tx_ptr);
            s->isr |= INT_AHB_ERR;
            break;
        }

        if (le32_to_cpu(d[0]) & TXDES0_EDOTR) {
            s->tx_ptr = s->nptxr_badr;
        } else {
            s->tx_ptr += DESC_SIZE;
        }
    }

    ftgmac030_update_irq(s);
}

/* ---- RX ---- */

static uint32_t ftgmac030_rx_bufsz(Ftgmac030State *s)
{
    uint32_t sz = s->rbsr & 0x3fff;

    sz &= ~7u;          /* hardware requires 8-byte multiples */
    return sz ? sz : RBSR_RESET;
}

/*
 * Reads the whole descriptor, so a free one is known to be fully readable
 * before receive() consumes its buffer address.  *bus_err (if given) tells
 * a failed read apart from a descriptor the guest still owns.
 */
static bool ftgmac030_rx_desc_free(Ftgmac030State *s, uint32_t addr,
                                   uint32_t *des0, bool *bus_err)
{
    uint32_t d[4];
    uint32_t v;

    if (bus_err) {
        *bus_err = false;
    }
    if (dma_memory_read(&address_space_memory, addr, d, sizeof(d),
                        MEMTXATTRS_UNSPECIFIED)) {
        if (bus_err) {
            *bus_err = true;
        }
        return false;
    }
    v = le32_to_cpu(d[0]);
    if (des0) {
        *des0 = v;
    }
    return !(v & RXDES0_RXPKT_RDY);
}

static bool ftgmac030_can_receive(NetClientState *nc)
{
    Ftgmac030State *s = FTGMAC030(qemu_get_nic_opaque(nc));
    bool bus_err;

    if (!ftgmac030_rx_enabled(s)) {
        return true;    /* receive() drops it, like a disabled MAC */
    }
    if (ftgmac030_rx_desc_free(s, s->rx_ptr, NULL, &bus_err)) {
        return true;
    }
    return bus_err;     /* receive() reports the bus error and drops */
}

/* Returns 0 = drop, else RXDES0 BROADCAST/MULTICAST flags | 1 */
static uint32_t ftgmac030_filter(Ftgmac030State *s, const uint8_t *buf)
{
    static const uint8_t bcast[ETH_ALEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };
    uint8_t mac[ETH_ALEN];

    if (!memcmp(buf, bcast, ETH_ALEN)) {
        return (s->maccr & (MACCR_RX_BROADPKT | MACCR_RX_ALL)) ?
               (RXDES0_BROADCAST | 1) : 0;
    }
    if (buf[0] & 1) {
        if (s->maccr & (MACCR_RX_MULTIPKT | MACCR_RX_ALL)) {
            return RXDES0_MULTICAST | 1;
        }
        if (s->maccr & MACCR_HT_MULTI_EN) {
            /* Driver hash: ~ether_crc, bytes bit-reversed, top 6 bits */
            uint32_t crc = ~net_crc32(buf, ETH_ALEN);
            uint32_t idx;

            crc = revbit8(crc & 0xff) |
                  (revbit8((crc >> 8) & 0xff) << 8) |
                  (revbit8((crc >> 16) & 0xff) << 16) |
                  ((uint32_t)revbit8((crc >> 24) & 0xff) << 24);
            idx = crc >> 26;
            if ((idx < 32 ? s->maht0 : s->maht1) & (1u << (idx & 31))) {
                return RXDES0_MULTICAST | 1;
            }
        }
        return 0;
    }
    if (s->maccr & MACCR_RX_ALL) {
        return 1;
    }
    mac[0] = s->madr >> 8;
    mac[1] = s->madr;
    mac[2] = s->ladr >> 24;
    mac[3] = s->ladr >> 16;
    mac[4] = s->ladr >> 8;
    mac[5] = s->ladr;
    return memcmp(buf, mac, ETH_ALEN) ? 0 : 1;
}

static ssize_t ftgmac030_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t len)
{
    Ftgmac030State *s = FTGMAC030(qemu_get_nic_opaque(nc));
    uint32_t bufsz = ftgmac030_rx_bufsz(s);
    uint32_t max_frame = (s->maccr & MACCR_JUMBO_LF) ? 9216 + 4 : 1518 + 4;
    uint32_t flags, crc, addr, des0, total, done, ftl = 0;
    uint8_t pkt[MAX_FRAME];
    bool bus_err;
    int i, ndesc;

    if (!ftgmac030_rx_enabled(s) || len < 14) {
        return len;
    }
    flags = ftgmac030_filter(s, buf);
    if (!flags) {
        return len;
    }
    flags &= ~1u;

    /* Pad runts to the 60-byte minimum, then append the FCS. */
    total = MAX(len, 60);
    if (total + 4 > max_frame) {
        total = max_frame - 4;
        ftl = RXDES0_FTL;
    }
    memset(pkt, 0, total);
    memcpy(pkt, buf, MIN(len, total));
    crc = cpu_to_be32(crc32(~0, pkt, total));
    memcpy(pkt + total, &crc, 4);
    total += 4;

    /* Make sure the whole frame fits before touching anything. */
    ndesc = DIV_ROUND_UP(total, bufsz);
    addr = s->rx_ptr;
    for (i = 0; i < ndesc; i++) {
        if (!ftgmac030_rx_desc_free(s, addr, &des0, &bus_err)) {
            if (bus_err) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "ftgmac030: bad RX desc @0x%x\n", addr);
                s->isr |= INT_AHB_ERR;
                ftgmac030_update_irq(s);
                return len;     /* drop: nothing written, ring unchanged */
            }
            s->isr |= INT_NO_RXBUF;
            ftgmac030_update_irq(s);
            return 0;   /* queue; flushed when the guest frees buffers */
        }
        addr = (des0 & RXDES0_EDORR) ? s->rxr_badr : addr + DESC_SIZE;
    }

    for (done = 0, i = 0; i < ndesc; i++) {
        uint32_t d[4];
        uint32_t chunk = MIN(bufsz, total - done);

        if (dma_memory_read(&address_space_memory, s->rx_ptr, d, sizeof(d),
                            MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR, "ftgmac030: bad RX desc @0x%x\n",
                          s->rx_ptr);
            s->isr |= INT_AHB_ERR;
            ftgmac030_update_irq(s);
            return len; /* never DMA to an unread buffer address */
        }
        des0 = le32_to_cpu(d[0]);
        if (dma_memory_write(&address_space_memory, le32_to_cpu(d[3]),
                             pkt + done, chunk, MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR, "ftgmac030: bad RX buf @0x%x\n",
                          le32_to_cpu(d[3]));
            s->isr |= INT_AHB_ERR;
        }
        done += chunk;

        d[1] = 0;   /* PROT_NONIP: let the stack verify checksums */
        d[0] = (des0 & RXDES0_EDORR) | chunk | RXDES0_RXPKT_RDY;
        if (i == 0) {
            d[0] |= RXDES0_FRS | flags;
        }
        if (i == ndesc - 1) {
            d[0] |= RXDES0_LRS | ftl;
        }
        d[0] = cpu_to_le32(d[0]);
        d[1] = cpu_to_le32(d[1]);
        /* rxdes1 first, then hand rxdes0 (ownership) over */
        dma_memory_write(&address_space_memory, s->rx_ptr + 4, &d[1], 4,
                         MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory, s->rx_ptr, &d[0], 4,
                         MEMTXATTRS_UNSPECIFIED);

        s->rx_ptr = (des0 & RXDES0_EDORR) ? s->rxr_badr
                                          : s->rx_ptr + DESC_SIZE;
    }

    s->isr |= INT_RPKT_BUF | INT_RPKT_FIFO;
    ftgmac030_update_irq(s);
    return len;
}

static void ftgmac030_flush_rx(Ftgmac030State *s)
{
    if (ftgmac030_rx_enabled(s)) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

/* ---- MMIO ---- */

static void ftgmac030_soft_reset(Ftgmac030State *s)
{
    s->isr = 0;
    s->ier = 0;
    s->maht0 = s->maht1 = 0;
    s->nptxr_badr = s->rxr_badr = s->hptxr_badr = 0;
    s->tx_ptr = s->rx_ptr = 0;
    s->maccr = 0;
    s->phycr = 0;
    s->phydata = 0;
    s->rbsr = RBSR_RESET;
    s->fear = FEAR_RESET;
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[REG_DBLAC / 4] = DBLAC_RESET;
    s->regs[REG_REVR / 4] = REVR_RESET;
    ftgmac030_update_irq(s);
}

static uint64_t ftgmac030_read(void *opaque, hwaddr addr, unsigned size)
{
    Ftgmac030State *s = opaque;

    switch (addr) {
    case REG_ISR:
        return s->isr;
    case REG_IER:
        return s->ier;
    case REG_MAC_MADR:
        return s->madr;
    case REG_MAC_LADR:
        return s->ladr;
    case REG_MAHT0:
        return s->maht0;
    case REG_MAHT1:
        return s->maht1;
    case REG_NPTXPD:
    case REG_RXPD:
    case REG_HPTXPD:
        return 0;
    case REG_NPTXR_BADR:
        return s->nptxr_badr;
    case REG_RXR_BADR:
        return s->rxr_badr;
    case REG_HPTXR_BADR:
        return s->hptxr_badr;
    case REG_DMAFIFOS:
        return DMAFIFOS_RXFIFO_EMPTY | DMAFIFOS_TXFIFO_EMPTY;
    case REG_RBSR:
        return s->rbsr;
    case REG_MACCR:
        return s->maccr;
    case REG_PHYCR:
        return s->phycr;
    case REG_PHYDATA:
        return s->phydata;
    case REG_NPTXR_PTR:
        return s->tx_ptr;
    case REG_HPTXR_PTR:
        return s->hptxr_badr;
    case REG_RXR_PTR:
        return s->rx_ptr;
    case REG_FEAR:
        return s->fear;
    default:
        if (addr < FTGMAC030_MMIO_SIZE) {
            return s->regs[addr / 4];
        }
        return 0;
    }
}

static void ftgmac030_write(void *opaque, hwaddr addr, uint64_t val64,
                            unsigned size)
{
    Ftgmac030State *s = opaque;
    uint32_t val = val64;

    switch (addr) {
    case REG_ISR:
        s->isr &= ~val;
        ftgmac030_update_irq(s);
        ftgmac030_flush_rx(s);
        break;
    case REG_IER:
        s->ier = val;
        ftgmac030_update_irq(s);
        ftgmac030_flush_rx(s);
        break;
    case REG_MAC_MADR:
        s->madr = val & 0xffff;
        break;
    case REG_MAC_LADR:
        s->ladr = val;
        break;
    case REG_MAHT0:
        s->maht0 = val;
        break;
    case REG_MAHT1:
        s->maht1 = val;
        break;
    case REG_NPTXPD:
        ftgmac030_do_tx(s);
        break;
    case REG_RXPD:
        ftgmac030_flush_rx(s);
        break;
    case REG_HPTXPD:
        qemu_log_mask(LOG_UNIMP, "ftgmac030: high-priority TX ring\n");
        break;
    case REG_NPTXR_BADR:
        s->nptxr_badr = val & ~0xfu;
        s->tx_ptr = s->nptxr_badr;
        break;
    case REG_RXR_BADR:
        s->rxr_badr = val & ~0xfu;
        s->rx_ptr = s->rxr_badr;
        break;
    case REG_HPTXR_BADR:
        s->hptxr_badr = val & ~0xfu;
        break;
    case REG_DMAFIFOS:
    case REG_NPTXR_PTR:
    case REG_HPTXR_PTR:
    case REG_RXR_PTR:
    case REG_FEAR:
    case REG_REVR:
        break;      /* read-only */
    case REG_RBSR:
        s->rbsr = val & 0x3fff;
        break;
    case REG_MACCR:
        if (val & MACCR_SW_RST) {
            ftgmac030_soft_reset(s);    /* self-clearing */
            break;
        }
        s->maccr = val;
        ftgmac030_do_tx(s);
        ftgmac030_flush_rx(s);
        break;
    case REG_PHYCR:
        s->phycr = val;
        ftgmac030_mdio(s);
        break;
    case REG_PHYDATA:
        s->phydata = (s->phydata & 0xffff0000) | (val & 0xffff);
        break;
    default:
        if (addr < FTGMAC030_MMIO_SIZE) {
            s->regs[addr / 4] = val;
        }
        break;
    }
}

static const MemoryRegionOps ftgmac030_ops = {
    .read = ftgmac030_read,
    .write = ftgmac030_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static NetClientInfo ftgmac030_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ftgmac030_can_receive,
    .receive = ftgmac030_receive,
};

static void ftgmac030_init(Object *obj)
{
    Ftgmac030State *s = FTGMAC030(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ftgmac030_ops, s,
                          TYPE_FTGMAC030, FTGMAC030_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ftgmac030_realize(DeviceState *dev, Error **errp)
{
    Ftgmac030State *s = FTGMAC030(dev);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&ftgmac030_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void ftgmac030_reset(DeviceState *dev)
{
    Ftgmac030State *s = FTGMAC030(dev);
    const uint8_t *m = s->conf.macaddr.a;

    ftgmac030_soft_reset(s);
    /* Power-on MAC address registers from the NIC config. */
    s->madr = (m[0] << 8) | m[1];
    s->ladr = ((uint32_t)m[2] << 24) | (m[3] << 16) | (m[4] << 8) | m[5];
    ftgmac030_phy_reset(s);
}

static const Property ftgmac030_properties[] = {
    DEFINE_NIC_PROPERTIES(Ftgmac030State, conf),
    /*
     * The xmorca ASIC's PHY answers at MDIO address 0: the SDK's U-Boot
     * (include/configs/xmorca.h SFV_PHY_U, 1 only on the FPGA) and kernel
     * DTS (xmorca.dts phy-addr = <0>, which OpenIPC's kernel keeps) both
     * say so.  XM's stock DTB has no phy-addr and takes the first PHY it
     * finds, so 0 serves both; at 1 OpenIPC's kernel finds no PHY.
     */
    DEFINE_PROP_UINT8("phy-addr", Ftgmac030State, phy_addr, 0),
};

static void ftgmac030_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ftgmac030_realize;
    device_class_set_legacy_reset(dc, ftgmac030_reset);
    device_class_set_props(dc, ftgmac030_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "Faraday FTGMAC030 Gigabit Ethernet MAC";
}

static const TypeInfo ftgmac030_info = {
    .name          = TYPE_FTGMAC030,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Ftgmac030State),
    .instance_init = ftgmac030_init,
    .class_init    = ftgmac030_class_init,
};

static void ftgmac030_register_types(void)
{
    type_register_static(&ftgmac030_info);
}

type_init(ftgmac030_register_types)
