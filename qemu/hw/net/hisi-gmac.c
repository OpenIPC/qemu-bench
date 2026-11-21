/*
 * HiSilicon Gigabit Ethernet MAC (HiGMAC) emulation.
 *
 * Emulates the HiGMAC V200 found in Hi3516AV100 and Hi3519V101 SoCs.
 * Compatible with kernel drivers:
 *   - drivers/net/ethernet/hisilicon/higmac/   (MAC + DMA)
 *   - drivers/net/phy/mdio-hisi-gemac.c        (MDIO bus)
 *
 * The device exposes a 4 KiB MMIO region containing:
 *   0x0000-0x004F  MAC configuration (station addr, port mode/enable)
 *   0x0064-0x006F  Receive filter, multicast address
 *   0x0500-0x05CF  DMA descriptor ring control (RX_FQ, RX_BQ, TX_BQ, TX_RQ)
 *   0x05C0-0x05E8  Interrupt and control registers
 *   0x03C0-0x03D4  MDIO registers (separate kernel driver maps this sub-region)
 *
 * DMA uses four descriptor rings:
 *   RX_FQ: SW pushes empty buffers, HW pops for receiving
 *   RX_BQ: HW pushes received packets, SW pops and processes
 *   TX_BQ: SW pushes packets to send, HW pops and transmits
 *   TX_RQ: HW pushes completed TX descriptors, SW reclaims
 *
 * Descriptor format (4-word, 16 bytes):
 *   word0: data_buff_addr (physical)
 *   word1: buffer_len[10:0] | reserved[15:11] | data_len[26:16] |
 *          reserved[28:27] | fl[30:29] | descvid[31]
 *   word2: rxhash
 *   word3: reserved[7:0] | l3_hash[8] | has_hash[9] | skb_id[23:10] | reserved[31:24]
 *
 * MDIO: integrated PHY stub at address 1 presenting standard MII
 * registers.  Reports 100 Mbps full-duplex link permanently up.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "net/net.h"
#include "net/eth.h"
#include "net/checksum.h"
#include "system/dma.h"

/* ── MAC registers ──────────────────────────────────────────────────── */

#define STATION_ADDR_LOW        0x0000
#define STATION_ADDR_HIGH       0x0004
#define MAC_DUPLEX_HALF_CTRL    0x0008
#define PORT_MODE               0x0040
#define PORT_EN                 0x0044
#define  BITS_TX_EN             BIT(2)
#define  BITS_RX_EN             BIT(1)
#define REC_FILT_CONTROL        0x0064
#define PORT_MC_ADDR_LOW        0x0068
#define PORT_MC_ADDR_HIGH       0x006C
#define MODE_CHANGE_EN          0x01B4
#define CRF_MIN_PACKET          0x0210
#define CONTROL_WORD            0x0214
#define TSO_COE_CTRL            0x02E8

/* ── DMA descriptor ring registers ──────────────────────────────────── */

#define RX_FQ_START_ADDR        0x0500
#define RX_FQ_DEPTH             0x0504
#define RX_FQ_WR_ADDR           0x0508
#define RX_FQ_RD_ADDR           0x050C
#define RX_FQ_VLDDESC_CNT       0x0510
#define RX_FQ_ALEMPTY_TH        0x0514
#define RX_FQ_REG_EN            0x0518
#define RX_FQ_ALFULL_TH         0x051C

#define RX_BQ_START_ADDR        0x0520
#define RX_BQ_DEPTH             0x0524
#define RX_BQ_WR_ADDR           0x0528
#define RX_BQ_RD_ADDR           0x052C
#define RX_BQ_FREE_DESC_CNT     0x0530
#define RX_BQ_ALEMPTY_TH        0x0534
#define RX_BQ_REG_EN            0x0538
#define RX_BQ_ALFULL_TH         0x053C

#define TX_BQ_START_ADDR        0x0580
#define TX_BQ_DEPTH             0x0584
#define TX_BQ_WR_ADDR           0x0588
#define TX_BQ_RD_ADDR           0x058C
#define TX_BQ_VLDDESC_CNT       0x0590
#define TX_BQ_ALEMPTY_TH        0x0594
#define TX_BQ_REG_EN            0x0598
#define TX_BQ_ALFULL_TH         0x059C

#define TX_RQ_START_ADDR        0x05A0
#define TX_RQ_DEPTH             0x05A4
#define TX_RQ_WR_ADDR           0x05A8
#define TX_RQ_RD_ADDR           0x05AC
#define TX_RQ_FREE_DESC_CNT     0x05B0
#define TX_RQ_ALEMPTY_TH        0x05B4
#define TX_RQ_REG_EN            0x05B8
#define TX_RQ_ALFULL_TH         0x05BC

/* ── Interrupt / control registers ──────────────────────────────────── */

#define RAW_PMU_INT             0x05C0
#define ENA_PMU_INT             0x05C4
#define DESC_WR_RD_ENA          0x05CC
#define STOP_CMD                0x05E8

#define RX_BQ_IN_INT            BIT(17)
#define TX_RQ_IN_INT            BIT(19)
#define RX_BQ_IN_TIMEOUT_INT    BIT(28)
#define TX_RQ_IN_TIMEOUT_INT    BIT(29)

/* ── MDIO registers (at offset 0x3C0 from GMAC base) ───────────────── */

#define MDIO_BASE               0x03C0
#define MDIO_SINGLE_CMD         0x0000
#define MDIO_SINGLE_DATA        0x0004
#define MDIO_RDATA_STATUS       0x0010
#define MDIO_START              BIT(20)
#define MDIO_READ               BIT(17)
#define MDIO_WRITE              BIT(16)
#define BIT_PHY_ADDR_OFFSET     8

/* ── Descriptor format ──────────────────────────────────────────────── */

/* Descriptors can be 4-word (16 bytes) or 8-word (32 bytes) depending
 * on kernel config.  We detect the size from the depth register value
 * at runtime.  WR/RD pointer registers and depth register all use the
 * same byte-shift encoding, so we use raw register values for ring
 * wrapping and only convert to byte addresses for DMA. */
#define DESC_VLD_FREE           0
#define DESC_VLD_BUSY           1
#define DESC_FL_FULL            3

/* ── PHY stub ───────────────────────────────────────────────────────── */

#define PHY_ADDR                1
#define PHY_ID1                 0x0044
#define PHY_ID2                 0x6161

/* ── Device state ───────────────────────────────────────────────────── */

#define TYPE_HISI_GMAC "hisi-gmac"
OBJECT_DECLARE_SIMPLE_TYPE(HisiGmacState, HISI_GMAC)

#define MMIO_SIZE               0x1000

struct HisiGmacState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq     irq;
    NICState    *nic;
    NICConf      conf;

    /* MAC registers */
    uint32_t station_addr_low;
    uint32_t station_addr_high;
    uint32_t port_mode;
    uint32_t port_en;
    uint32_t rec_filt;
    uint32_t mc_addr_low;
    uint32_t mc_addr_high;
    uint32_t control_word;

    /* Descriptor ring base addresses and pointers */
    uint32_t rx_fq_start;
    uint32_t rx_fq_depth;
    uint32_t rx_fq_wr;      /* software writes (pushes free bufs) */
    uint32_t rx_fq_rd;      /* hardware reads  (pops free bufs)   */

    uint32_t rx_bq_start;
    uint32_t rx_bq_depth;
    uint32_t rx_bq_wr;      /* hardware writes (pushes recv pkts) */
    uint32_t rx_bq_rd;      /* software reads  (pops recv pkts)   */

    uint32_t tx_bq_start;
    uint32_t tx_bq_depth;
    uint32_t tx_bq_wr;      /* software writes (pushes tx pkts)   */
    uint32_t tx_bq_rd;      /* hardware reads  (pops tx pkts)     */

    uint32_t tx_rq_start;
    uint32_t tx_rq_depth;
    uint32_t tx_rq_wr;      /* hardware writes (pushes tx done)   */
    uint32_t tx_rq_rd;      /* software reads  (reclaims tx done) */

    /* Interrupt state */
    uint32_t raw_pmu_int;
    uint32_t ena_pmu_int;
    uint32_t desc_wr_rd_ena;

    /* MDIO state */
    uint32_t mdio_cmd;
    uint32_t mdio_data;

    /* PHY registers */
    uint16_t phy_regs[32];

    /* Descriptor size in bytes (16 or 32; CONFIG_HIGMAC_DESC_4_WORD). */
    uint32_t desc_size_bytes;

    /* MDIO address of the single emulated PHY.  Most HiSilicon dies strap it
     * at 1, but some vendor device trees place it elsewhere — e.g. the
     * hi3536cv100 NVR DTs (mdio@100a03c0/ethernet-phy@2) use address 2, and
     * the kernel's higmacv300 never finds a PHY at 1 there, so no netdev
     * comes up.  Configurable via "phy-addr". */
    uint8_t phy_addr;

    /* Offset into each RX buffer at which the MAC DMAs the frame.  The newer
     * "gmac-v5" vendor driver (Hi3519DV500) allocates buffers at offset 0 and
     * skb_reserve(NET_IP_ALIGN) on RX, so it expects the frame 2 bytes in; the
     * older hi_gmac_v200 driver's buffer address already includes the
     * alignment, so it wants offset 0.  Configurable via "rx-pkt-offset". */
    uint32_t rx_pkt_offset;

    /* Backing register array for misc registers */
    uint32_t regs[MMIO_SIZE / 4];
};

/* ── PHY emulation ──────────────────────────────────────────────────── */

static void hisi_gmac_phy_init(HisiGmacState *s)
{
    s->phy_regs[0]  = 0x3100;  /* BMCR: auto-neg, 100M, full-duplex */
    s->phy_regs[1]  = 0x786D;  /* BMSR: link up, auto-neg complete */
    s->phy_regs[2]  = PHY_ID1;
    s->phy_regs[3]  = PHY_ID2;
    s->phy_regs[4]  = 0x01E1;  /* ANAR */
    s->phy_regs[5]  = 0x45E1;  /* ANLPAR: partner 100M FD */
    s->phy_regs[6]  = 0x0001;  /* ANER */
    s->phy_regs[17] = 0x4000;  /* Vendor: speed indication 100M */
}

static uint16_t hisi_gmac_phy_read(HisiGmacState *s, int reg)
{
    if (reg < 32) {
        return s->phy_regs[reg];
    }
    return 0xFFFF;
}

static void hisi_gmac_phy_write(HisiGmacState *s, int reg, uint16_t val)
{
    if (reg == 0) {
        /* BMCR — accept writes but keep link up.  RESET (bit15) and
         * ANRESTART (bit9) are self-clearing in real PHYs: genphy sets
         * ANRESTART to restart auto-negotiation and then polls BMCR until it
         * clears, so we must clear both or the link never comes up. */
        s->phy_regs[0] = val & ~(BIT(15) | BIT(9));
        s->phy_regs[1] |= BIT(2) | BIT(5); /* link up, auto-neg complete */
    } else if (reg < 32 && reg >= 4) {
        s->phy_regs[reg] = val;
    }
}

/* ── MDIO command execution ─────────────────────────────────────────── */

static void hisi_gmac_mdio_exec(HisiGmacState *s)
{
    uint32_t cmd = s->mdio_cmd;
    int phy_addr = (cmd >> BIT_PHY_ADDR_OFFSET) & 0x1F;
    int reg = cmd & 0x1F;

    if (phy_addr != s->phy_addr) {
        /* No chip at this address */
        s->mdio_data = 0xFFFF << 16;
        s->mdio_cmd &= ~MDIO_START;
        return;
    }

    if (cmd & MDIO_WRITE) {
        hisi_gmac_phy_write(s, reg, s->mdio_data & 0xFFFF);
    } else if (cmd & MDIO_READ) {
        s->mdio_data = (uint32_t)hisi_gmac_phy_read(s, reg) << 16;
    }

    s->mdio_cmd &= ~MDIO_START;
}

/* ── IRQ handling ───────────────────────────────────────────────────── */

static void hisi_gmac_update_irq(HisiGmacState *s)
{
    bool level = (s->raw_pmu_int & s->ena_pmu_int) != 0;
    qemu_set_irq(s->irq, level);
}

/* ── Descriptor ring helpers ────────────────────────────────────────── */

/*
 * Descriptor ring layout.
 *
 * The driver writes depth_reg = num_descriptors * (desc_size_bytes / 4)
 * — i.e. queue extent measured in 32-bit words.  The WR/RD pointers
 * are BYTE offsets into the queue and advance by DESC_SIZE per
 * descriptor, wrapping at queue_size_bytes.
 *
 * Descriptor size is 8 words (32 bytes) on AV100 / 3519V101 / hi3536dv100
 * (vendor `drivers/net/.../higmac.h` `DESC_WORD_SHIFT=3`).  It's 4 words
 * (16 bytes) when `CONFIG_HIGMAC_DESC_4_WORD` is set — e.g. hi3536cv100
 * (`drivers/net/higmacv300/higmac.h` with `DESC_WORD_SHIFT=2`).  Word 0
 * is the buffer DMA address and word 1 is the length/flags bitfield;
 * later words are reserved padding, so the read/write path doesn't
 * change — only the stride between descriptors does.  Exposed as the
 * `desc-size` property (default 32, set to 16 for hi3536cv100).
 *
 * Auto-detect: the same SoC can run two firmwares built with different
 * `CONFIG_HIGMAC_DESC_4_WORD` — e.g. the OpenIPC Hi3519V101 ships a
 * U-Boot built with 4-word descriptors but a kernel built with 8-word.
 * When SW writes a *_WR_ADDR pointer for the first time after the ring
 * is empty, that value IS one descriptor stride, so we latch it.  This
 * keeps both firmwares working without per-SoC config changes.
 */
#define DESC_SIZE_BYTES_MAX     32
#define DESC_SIZE_BYTES_DEFAULT 32

/* Latch a smaller descriptor stride when SW reveals it via the first
 * non-zero wr-ptr write to an empty ring.  No-op if the ring isn't
 * empty (SW queued multiple descriptors at once, which we can't safely
 * disambiguate) or if the value isn't a smaller power-of-two stride. */
static inline void hisi_gmac_maybe_detect_stride(HisiGmacState *s,
                                                  uint32_t wr, uint32_t rd)
{
    if (wr == 0 || rd != 0 || wr >= s->desc_size_bytes) {
        return;
    }
    if (wr == 16) {
        s->desc_size_bytes = 16;
    }
}

/* Queue size in bytes from the depth register value (queue size in
 * words, so * 4).  0 if the driver hasn't programmed depth yet. */
static inline uint32_t queue_size_bytes(uint32_t depth_reg)
{
    return depth_reg * 4;
}

/* Advance a byte-offset ring pointer by one descriptor, wrapping at
 * queue_size_bytes.  Returns 0 if the queue is unsized. */
static inline uint32_t ring_next_ptr(HisiGmacState *s, uint32_t ptr,
                                     uint32_t depth_reg)
{
    uint32_t qsz = queue_size_bytes(depth_reg);
    if (!qsz) {
        return 0;
    }
    ptr += s->desc_size_bytes;
    if (ptr >= qsz) {
        ptr = 0;
    }
    return ptr;
}

/* ── TX path: process packets from TX_BQ ring ───────────────────────── */

static void hisi_gmac_tx(HisiGmacState *s)
{
    uint32_t rd_ptr, wr_ptr;
    uint8_t buf[2048];

    if (!s->tx_bq_depth || !(s->port_en & BITS_TX_EN) || !s->desc_wr_rd_ena) {
        return;
    }

    rd_ptr = s->tx_bq_rd;
    wr_ptr = s->tx_bq_wr;

    while (rd_ptr != wr_ptr) {
        uint32_t desc_addr = s->tx_bq_start + rd_ptr;
        uint32_t desc[DESC_SIZE_BYTES_MAX / 4];

        dma_memory_read(&address_space_memory, desc_addr,
                        desc, s->desc_size_bytes, MEMTXATTRS_UNSPECIFIED);

        uint32_t data_addr = desc[0];
        uint32_t w1 = desc[1];
        uint32_t data_len = (w1 >> 16) & 0x7FF;
        uint32_t descvid = (w1 >> 31) & 1;

        if (descvid != DESC_VLD_BUSY || data_len == 0 ||
            data_len > sizeof(buf)) {
            break;
        }

        dma_memory_read(&address_space_memory, data_addr,
                        buf, data_len, MEMTXATTRS_UNSPECIFIED);

        net_checksum_calculate(buf, data_len, CSUM_ALL);
        qemu_send_packet(qemu_get_queue(s->nic), buf, data_len);

        /* Write completion descriptor to TX_RQ ring */
        if (s->tx_rq_depth) {
            uint32_t rq_addr = s->tx_rq_start + s->tx_rq_wr;

            desc[1] = (w1 & ~(1u << 31));
            dma_memory_write(&address_space_memory, rq_addr,
                             desc, s->desc_size_bytes, MEMTXATTRS_UNSPECIFIED);

            s->tx_rq_wr = ring_next_ptr(s, s->tx_rq_wr, s->tx_rq_depth);
        }

        rd_ptr = ring_next_ptr(s, rd_ptr, s->tx_bq_depth);
    }

    s->tx_bq_rd = rd_ptr;

    /* Raise TX completion interrupt */
    if (s->tx_rq_wr != s->tx_rq_rd) {
        s->raw_pmu_int |= TX_RQ_IN_INT;
        hisi_gmac_update_irq(s);
    }
}

/* ── RX path: receive packet from network ───────────────────────────── */

static bool hisi_gmac_can_receive(NetClientState *nc)
{
    HisiGmacState *s = qemu_get_nic_opaque(nc);

    if (!(s->port_en & BITS_RX_EN) || !s->desc_wr_rd_ena || !s->rx_fq_depth) {
        return false;
    }

    return s->rx_fq_rd != s->rx_fq_wr;
}

static ssize_t hisi_gmac_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    HisiGmacState *s = qemu_get_nic_opaque(nc);

    if (!s->rx_fq_depth || !s->rx_bq_depth) {
        return -1;
    }
    if (!(s->port_en & BITS_RX_EN) || !s->desc_wr_rd_ena) {
        return -1;
    }
    if (s->rx_fq_rd == s->rx_fq_wr) {
        return -1;
    }

    /* Pop a descriptor from RX_FQ */
    uint32_t fq_addr = s->rx_fq_start + s->rx_fq_rd;
    uint32_t desc[DESC_SIZE_BYTES_MAX / 4];
    dma_memory_read(&address_space_memory, fq_addr,
                    desc, s->desc_size_bytes, MEMTXATTRS_UNSPECIFIED);

    uint32_t data_addr = desc[0];
    uint32_t w1 = desc[1];
    uint32_t buffer_len = (w1 & 0x7FF) + 1;

    /* DMA the frame at the configured RX offset (rx-pkt-offset): the gmac-v5
     * driver skb_reserve()s NET_IP_ALIGN and expects the frame 2 bytes into
     * the buffer; hi_gmac_v200's buffer address already includes it (offset
     * 0).  Wrong offset => every frame delivered shifted and dropped. */
    if (size + s->rx_pkt_offset > buffer_len) {
        return -1;
    }

    dma_memory_write(&address_space_memory, data_addr + s->rx_pkt_offset,
                     buf, size, MEMTXATTRS_UNSPECIFIED);

    /* Advance RX_FQ read pointer */
    s->rx_fq_rd = ring_next_ptr(s, s->rx_fq_rd, s->rx_fq_depth);

    /* Push descriptor to RX_BQ with received length */
    uint32_t bq_addr = s->rx_bq_start + s->rx_bq_wr;

    w1 = (w1 & 0x7FF) |
         ((uint32_t)size << 16) |
         (DESC_FL_FULL << 29) |
         (DESC_VLD_BUSY << 31);
    desc[1] = w1;

    dma_memory_write(&address_space_memory, bq_addr,
                     desc, s->desc_size_bytes, MEMTXATTRS_UNSPECIFIED);

    s->rx_bq_wr = ring_next_ptr(s, s->rx_bq_wr, s->rx_bq_depth);

    /* Raise RX interrupt */
    s->raw_pmu_int |= RX_BQ_IN_INT;
    hisi_gmac_update_irq(s);

    return size;
}

/* ── MMIO read ──────────────────────────────────────────────────────── */

static uint64_t hisi_gmac_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiGmacState *s = HISI_GMAC(opaque);

    /* MDIO registers (separate kernel driver at base+0x3C0) */
    if (offset >= MDIO_BASE && offset < MDIO_BASE + 0x20) {
        switch (offset - MDIO_BASE) {
        case MDIO_SINGLE_CMD:   return s->mdio_cmd;
        case MDIO_SINGLE_DATA:  return s->mdio_data;
        case MDIO_RDATA_STATUS: return 0; /* valid */
        default: return 0;
        }
    }

    switch (offset) {
    case STATION_ADDR_LOW:   return s->station_addr_low;
    case STATION_ADDR_HIGH:  return s->station_addr_high;
    case PORT_MODE:          return s->port_mode;
    case PORT_EN:            return s->port_en;
    case REC_FILT_CONTROL:   return s->rec_filt;
    case PORT_MC_ADDR_LOW:   return s->mc_addr_low;
    case PORT_MC_ADDR_HIGH:  return s->mc_addr_high;
    case CONTROL_WORD:       return s->control_word;

    /* RX_FQ */
    case RX_FQ_START_ADDR:   return s->rx_fq_start;
    case RX_FQ_DEPTH:        return s->rx_fq_depth;
    case RX_FQ_WR_ADDR:      return s->rx_fq_wr;
    case RX_FQ_RD_ADDR:      return s->rx_fq_rd;
    case RX_FQ_REG_EN:       return 0;

    /* RX_BQ */
    case RX_BQ_START_ADDR:   return s->rx_bq_start;
    case RX_BQ_DEPTH:        return s->rx_bq_depth;
    case RX_BQ_WR_ADDR:      return s->rx_bq_wr;
    case RX_BQ_RD_ADDR:      return s->rx_bq_rd;
    case RX_BQ_REG_EN:       return 0;

    /* TX_BQ */
    case TX_BQ_START_ADDR:   return s->tx_bq_start;
    case TX_BQ_DEPTH:        return s->tx_bq_depth;
    case TX_BQ_WR_ADDR:      return s->tx_bq_wr;
    case TX_BQ_RD_ADDR:      return s->tx_bq_rd;
    case TX_BQ_REG_EN:       return 0;

    /* TX_RQ */
    case TX_RQ_START_ADDR:   return s->tx_rq_start;
    case TX_RQ_DEPTH:        return s->tx_rq_depth;
    case TX_RQ_WR_ADDR:      return s->tx_rq_wr;
    case TX_RQ_RD_ADDR:      return s->tx_rq_rd;
    case TX_RQ_REG_EN:       return 0;

    /* Interrupts */
    case RAW_PMU_INT:        return s->raw_pmu_int;
    case ENA_PMU_INT:        return s->ena_pmu_int;
    case DESC_WR_RD_ENA:     return s->desc_wr_rd_ena;
    case STOP_CMD:           return 0;

    default:
        if (offset < MMIO_SIZE) {
            return s->regs[offset / 4];
        }
        return 0;
    }
}

/* ── MMIO write ─────────────────────────────────────────────────────── */

static void hisi_gmac_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    HisiGmacState *s = HISI_GMAC(opaque);

    /* MDIO registers */
    if (offset >= MDIO_BASE && offset < MDIO_BASE + 0x20) {
        switch (offset - MDIO_BASE) {
        case MDIO_SINGLE_CMD:
            s->mdio_cmd = val;
            if (val & MDIO_START) {
                hisi_gmac_mdio_exec(s);
            }
            break;
        case MDIO_SINGLE_DATA:
            s->mdio_data = val;
            break;
        }
        return;
    }

    switch (offset) {
    case STATION_ADDR_LOW:   s->station_addr_low = val; break;
    case STATION_ADDR_HIGH:  s->station_addr_high = val; break;
    case PORT_MODE:          s->port_mode = val; break;
    case PORT_EN:
        s->port_en = val;
        if (val & BITS_RX_EN) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case REC_FILT_CONTROL:   s->rec_filt = val; break;
    case PORT_MC_ADDR_LOW:   s->mc_addr_low = val; break;
    case PORT_MC_ADDR_HIGH:  s->mc_addr_high = val; break;
    case CONTROL_WORD:       s->control_word = val; break;

    /* RX_FQ — writes gated by REG_EN */
    case RX_FQ_REG_EN:
        /* REG_EN latches one write then auto-clears */
        break;
    case RX_FQ_START_ADDR:   s->rx_fq_start = val; break;
    case RX_FQ_DEPTH:        s->rx_fq_depth = val; break;
    case RX_FQ_WR_ADDR:
        hisi_gmac_maybe_detect_stride(s, val, s->rx_fq_rd);
        s->rx_fq_wr = val;
        /* New free buffers available — check if we can receive now */
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case RX_FQ_RD_ADDR:      s->rx_fq_rd = val; break;

    /* RX_BQ */
    case RX_BQ_REG_EN:       break;
    case RX_BQ_START_ADDR:   s->rx_bq_start = val; break;
    case RX_BQ_DEPTH:        s->rx_bq_depth = val; break;
    case RX_BQ_WR_ADDR:      s->rx_bq_wr = val; break;
    case RX_BQ_RD_ADDR:      s->rx_bq_rd = val; break;

    /* TX_BQ */
    case TX_BQ_REG_EN:       break;
    case TX_BQ_START_ADDR:   s->tx_bq_start = val; break;
    case TX_BQ_DEPTH:        s->tx_bq_depth = val; break;
    case TX_BQ_WR_ADDR:
        hisi_gmac_maybe_detect_stride(s, val, s->tx_bq_rd);
        s->tx_bq_wr = val;
        /* New TX packets queued — process them */
        hisi_gmac_tx(s);
        break;
    case TX_BQ_RD_ADDR:      s->tx_bq_rd = val; break;

    /* TX_RQ */
    case TX_RQ_REG_EN:       break;
    case TX_RQ_START_ADDR:   s->tx_rq_start = val; break;
    case TX_RQ_DEPTH:        s->tx_rq_depth = val; break;
    case TX_RQ_WR_ADDR:      s->tx_rq_wr = val; break;
    case TX_RQ_RD_ADDR:      s->tx_rq_rd = val; break;

    /* Interrupts */
    case RAW_PMU_INT:
        s->raw_pmu_int &= ~val; /* W1C */
        hisi_gmac_update_irq(s);
        break;
    case ENA_PMU_INT:
        s->ena_pmu_int = val;
        hisi_gmac_update_irq(s);
        break;
    case DESC_WR_RD_ENA:
        s->desc_wr_rd_ena = val;
        if (val) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case STOP_CMD:
        if (val & BIT(0)) { /* RX stop */
            s->port_en &= ~BITS_RX_EN;
        }
        if (val & BIT(1)) { /* TX stop */
            s->port_en &= ~BITS_TX_EN;
        }
        break;

    default:
        if (offset < MMIO_SIZE) {
            s->regs[offset / 4] = val;
        }
        break;
    }
}

static const MemoryRegionOps hisi_gmac_ops = {
    .read = hisi_gmac_read,
    .write = hisi_gmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── NIC callbacks ──────────────────────────────────────────────────── */

static void hisi_gmac_set_link(NetClientState *nc)
{
    HisiGmacState *s = qemu_get_nic_opaque(nc);
    if (nc->link_down) {
        s->phy_regs[1] &= ~BIT(2); /* link down */
    } else {
        s->phy_regs[1] |= BIT(2);  /* link up */
    }
}

static NetClientInfo hisi_gmac_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = hisi_gmac_can_receive,
    .receive = hisi_gmac_receive,
    .link_status_changed = hisi_gmac_set_link,
};

/* ── Device lifecycle ───────────────────────────────────────────────── */

static void hisi_gmac_init(Object *obj)
{
    HisiGmacState *s = HISI_GMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_gmac_ops, s,
                          "hisi-gmac", MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void hisi_gmac_realize(DeviceState *dev, Error **errp)
{
    HisiGmacState *s = HISI_GMAC(dev);

    hisi_gmac_phy_init(s);

    s->nic = qemu_new_nic(&hisi_gmac_net_info, &s->conf,
                           object_get_typename(OBJECT(dev)),
                           dev->id, &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static const Property hisi_gmac_properties[] = {
    DEFINE_NIC_PROPERTIES(HisiGmacState, conf),
    /* Descriptor stride: 32 bytes by default (8-word descriptor), 16 for
     * SoCs whose vendor higmacv300 sets CONFIG_HIGMAC_DESC_4_WORD — e.g.
     * hi3536cv100.  Word 0 (buffer DMA addr) and word 1 (length/flags)
     * sit at the same offsets in both layouts, only the per-descriptor
     * stride differs. */
    DEFINE_PROP_UINT32("desc-size", HisiGmacState, desc_size_bytes,
                       DESC_SIZE_BYTES_DEFAULT),
    /* RX DMA offset into each buffer (NET_IP_ALIGN).  0 for hi_gmac_v200,
     * 2 for the gmac-v5 driver (Hi3519DV500). */
    DEFINE_PROP_UINT32("rx-pkt-offset", HisiGmacState, rx_pkt_offset, 0),
    /* MDIO address of the emulated PHY; strapped at 1 on most dies, 2 on the
     * hi3536cv100 NVR device trees. */
    DEFINE_PROP_UINT8("phy-addr", HisiGmacState, phy_addr, PHY_ADDR),
};

static void hisi_gmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = hisi_gmac_realize;
    device_class_set_props(dc, hisi_gmac_properties);
}

static const TypeInfo hisi_gmac_info = {
    .name          = TYPE_HISI_GMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiGmacState),
    .instance_init = hisi_gmac_init,
    .class_init    = hisi_gmac_class_init,
};

static void hisi_gmac_register_types(void)
{
    type_register_static(&hisi_gmac_info);
}

type_init(hisi_gmac_register_types)
