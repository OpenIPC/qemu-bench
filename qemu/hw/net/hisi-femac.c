/*
 * HiSilicon Fast Ethernet MAC (FEMAC) emulation.
 *
 * Emulates the FEMAC found in Hi3516CV300 and Hi3516EV300 SoCs.
 * Compatible with kernel drivers:
 *   - drivers/net/ethernet/hisilicon/hisi_femac.c  (MAC + DMA)
 *   - drivers/net/phy/mdio-hisi-femac.c            (MDIO bus)
 *
 * The device exposes a single 8 KiB MMIO region containing three
 * register blocks at fixed offsets from the base address:
 *
 *   Offset  Size   Block
 *   0x0000  0x1000 Port registers (MAC control, DMA queues, status)
 *   0x1100  0x0010 MDIO registers (PHY management interface)
 *   0x1300  0x0200 GLB registers  (IRQs, MAC address, forwarding)
 *
 * The kernel maps each block via separate platform resources
 * (port_base, glb_base, mdio membase) that all fall within our
 * single MMIO region.
 *
 * TX: guest writes DMA address to EQ_ADDR, then frame length to
 * EQFRM_LEN which triggers immediate DMA read + qemu_send_packet().
 *
 * RX: guest pushes buffer DMA addresses via IQ_ADDR writes into a
 * 64-entry ring.  When a packet arrives, we pop a buffer, DMA-write
 * the payload, set IQFRM_DES (length + 4 for FCS), and raise IRQ.
 *
 * MDIO: integrated PHY stub at address 1 presenting standard MII
 * registers.  Reports 100 Mbps full-duplex link permanently up.
 * The Generic PHY driver binds to it via PHY ID 0x00446161.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "net/net.h"
#include "net/eth.h"
#include "net/checksum.h"
#include "system/dma.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "hw/core/cpu.h"

#define TYPE_HISI_FEMAC "hisi-femac"
OBJECT_DECLARE_SIMPLE_TYPE(HisiFemacState, HISI_FEMAC)

/* Port registers */
#define REG_MAC_PORTSEL         0x0200
#define REG_MAC_PORTSET         0x0208
#define REG_MAC_SET             0x0210
#define REG_RX_COALESCE_SET     0x0340
#define REG_QLEN_SET            0x0344
#define REG_FC_LEVEL            0x0348
#define REG_IQFRM_DES           0x0354
#define REG_IQ_ADDR             0x0358
#define REG_EQ_ADDR             0x0360
#define REG_EQFRM_LEN           0x0364
#define REG_ADDRQ_STAT          0x036C
#define REG_RX_COE_CTRL         0x0380

/* U-Boot per-port MDIO registers (upstream port) */
#define REG_U_MDIO_PHYADDR     0x0108
#define REG_U_MDIO_RO_STAT     0x010C
#define REG_U_MDIO_ANEG_CTRL   0x0110
#define REG_U_MDIO_IRQENA      0x0114

/* MAC TX/RX IPG control (U-Boot uses these) */
#define REG_U_MAC_TX_IPGCTRL   0x0218

/*
 * V1/V2 hieth-sf TX descriptor queue.  CV100's 3.0.8 kernel uses the
 * older hieth-sf driver which pushes TX descriptors via 0x488 and arms
 * the queue via 0x490.  Our V3/V4 FEMAC model handles TX instantaneously
 * through REG_EQ_ADDR/REG_EQFRM_LEN, so reads of 0x488 returning 0 ("queue
 * empty") let hieth-sf exit its busy-poll immediately; writes are no-ops.
 */
#define REG_HIETH_TX_Q_STAT    0x0488
#define REG_HIETH_TX_Q_CTRL    0x0490

/*
 * Vendor init/enable register at 0x3a4.  Not in upstream hisi_femac.c;
 * vendor BSPs for CV200/CV300/CV500 do a one-shot read-modify-write that
 * sets bit 31 during port bring-up.  Model as a plain RAM register.
 */
#define REG_PORT_VENDOR_INIT   0x03a4

/* Linux kernel MDIO registers (separate MDIO base) */
#define REG_MDIO_RWCTRL         0x1100
#define REG_MDIO_RO_DATA        0x1104

/* GLB registers */
#define REG_GLB_HOSTMAC_L32     0x1300
#define REG_GLB_HOSTMAC_H16     0x1304
#define REG_GLB_SOFT_RESET      0x1308
#define REG_GLB_FWCTRL          0x1310
#define REG_GLB_MACTCTRL        0x1314
#define REG_GLB_ENDIAN_MOD      0x1318
#define REG_GLB_IRQ_STAT        0x1330
#define REG_GLB_IRQ_ENA         0x1334
#define REG_GLB_IRQ_RAW         0x1338
#define REG_MAC_FILTER_BASE     0x1400

/* GLB_SOFT_RESET bits (hisi_femac.c: SOFT_RESET_ALL BIT(0)) */
#define SOFT_RESET_ALL          (1 << 0)

/* IRQ bits */
#define IRQ_RX_RDY              (1 << 0)
#define IRQ_TX_PER_PACKET       (1 << 1)
#define IRQ_TX_FIFO_EMPTY       (1 << 6)
#define IRQ_MULTI_RXRDY         (1 << 7)
#define IRQ_ENA_PORT0           (1 << 18)
#define IRQ_ENA_ALL             (1 << 19)

/* MDIO RWCTRL bits */
#define MDIO_RW_FINISH          (1 << 15)
#define MDIO_RW_WRITE           (1 << 13)
#define MDIO_PHY_ADDR_SHIFT     8
#define MDIO_PHY_ADDR_MASK      0x1f
#define MDIO_REG_ADDR_SHIFT     0  /* actually bits 4:0 per driver but offset varies */
#define MDIO_REG_ADDR_MASK      0x1f

/* ADDRQ_STAT bits */
#define ADDRQ_TX_READY          (1 << 24)

/* PHY address we respond to */
#define FEMAC_PHY_ADDR          1

/* RX ring size */
#define RX_RING_SIZE            64

/*
 * MAC_SET bits 10:0 hold the largest frame the MAC will deliver
 * (hisi_femac.c: MAX_FRAME_SIZE_MASK GENMASK(10, 0)), and the driver both
 * programs it and sizes every posted RX buffer to the same MAX_FRAME_SIZE.
 */
#define FEMAC_MAX_FRAME_SIZE_MASK       0x7ff
#define FEMAC_MAX_FRAME_SIZE_DEFAULT    1600

struct HisiFemacState {
    SysBusDevice parent;
    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    /* Port registers */
    uint32_t mac_portsel;
    uint32_t mac_portset;
    uint32_t mac_set;
    uint32_t rx_coalesce_set;
    uint32_t qlen_set;
    uint32_t fc_level;
    uint32_t rx_coe_ctrl;

    /* TX */
    uint32_t eq_addr;

    /* RX ring of buffer DMA addresses */
    uint32_t rx_ring[RX_RING_SIZE];
    uint32_t rx_ring_head;
    uint32_t rx_ring_tail;
    uint32_t rx_ring_count;

    /* RX frame descriptor FIFO (mirrors the DMA buffer ring) */
    uint32_t iqfrm_des_fifo[RX_RING_SIZE];
    uint32_t iqfrm_des_head;
    uint32_t iqfrm_des_tail;
    uint32_t iqfrm_des_count;

    /* MDIO */
    uint32_t mdio_rwctrl;
    uint32_t mdio_ro_data;
    uint32_t u_mdio_phyaddr;
    uint32_t u_mdio_aneg_ctrl;
    uint32_t u_mdio_irqena;
    uint16_t phy_bmcr;
    uint16_t phy_bmsr;
    uint16_t phy_anar;
    uint16_t phy_anlpar;
    uint8_t  phy_addr;  /* MDIO address the integrated PHY responds at */

    /* GLB */
    uint32_t hostmac_l32;
    uint32_t hostmac_h16;
    uint32_t soft_reset;
    uint32_t fwctrl;
    uint32_t mactctrl;
    uint32_t irq_ena;
    uint32_t irq_raw;
    uint32_t endian_mod;
    uint32_t tx_ipgctrl;
    uint32_t port_vendor_init;
    uint32_t mac_filter[16];

    /* Periodic RX poll timer for polled (non-IRQ) drivers like U-Boot */
    QEMUTimer *rx_poll_timer;
};

static void hisi_femac_update_irq(HisiFemacState *s)
{
    bool level = (s->irq_raw & s->irq_ena) != 0;
    qemu_set_irq(s->irq, level);
}

/*
 * Periodic RX poll timer — ensures the main loop processes network
 * backends (SLIRP, TAP) even when the vCPU is busy-polling MMIO
 * registers.  This is needed for ARM926 SoCs where U-Boot's udelay
 * doesn't trigger an event loop iteration.  The timer auto-disables
 * once the RX ring is empty (no pending receive buffers).
 */
#define FEMAC_RX_POLL_NS  (100000)  /* 100 us */

static void hisi_femac_rx_poll(void *opaque)
{
    HisiFemacState *s = opaque;

    /* Deliver any packets queued by the network backend */
    qemu_flush_queued_packets(qemu_get_queue(s->nic));

    if (s->rx_ring_count > 0) {
        timer_mod(s->rx_poll_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + FEMAC_RX_POLL_NS);
        /*
         * Force the vCPU out of cpu_exec() so the main loop can poll
         * SLIRP/TAP sockets.  Without this, ARM926 SoCs (no arch_timer)
         * never yield and network replies are never received.
         */
        CPUState *cpu = first_cpu;
        if (cpu) {
            cpu->exit_request = 1;
            qemu_cpu_kick(cpu);
        }
    }
}

/* ── MDIO PHY stub ─────────────────────────────────────────────────── */

static uint16_t hisi_femac_phy_read(HisiFemacState *s, uint8_t reg)
{
    switch (reg) {
    case 0: /* BMCR */
        return s->phy_bmcr;
    case 1: /* BMSR */
        return s->phy_bmsr;
    case 2: /* PHYID1 */
        return 0x0044;
    case 3: /* PHYID2 */
        return 0x6161;
    case 4: /* ANAR */
        return s->phy_anar;
    case 5: /* ANLPAR */
        return s->phy_anlpar;
    default:
        return 0;
    }
}

static void hisi_femac_phy_write(HisiFemacState *s, uint8_t reg, uint16_t val)
{
    switch (reg) {
    case 0: /* BMCR */
        /* bit 15 (reset) and bit 9 (restart autoneg) are self-clearing */
        s->phy_bmcr = val & ~(0x8000 | 0x0200);
        break;
    case 4: /* ANAR */
        s->phy_anar = val;
        break;
    default:
        break;
    }
}

static void hisi_femac_mdio_access(HisiFemacState *s, uint32_t val)
{
    uint8_t phy_addr = (val >> MDIO_PHY_ADDR_SHIFT) & MDIO_PHY_ADDR_MASK;
    uint8_t reg_addr = val & MDIO_REG_ADDR_MASK;
    bool is_write = val & MDIO_RW_WRITE;

    s->mdio_rwctrl = val | MDIO_RW_FINISH;

    /*
     * Respond to PHY address 0 (U-Boot default) and the configured PHY
     * address (Linux DTB / driver default — 1 for femac, 3 for hieth-sf).
     * Real hardware has one internal PHY; the address depends on the
     * driver configuration.
     */
    if (phy_addr != s->phy_addr && phy_addr != 0) {
        s->mdio_ro_data = 0xffff;
        return;
    }

    if (is_write) {
        hisi_femac_phy_write(s, reg_addr, (val >> 16) & 0xffff);
    } else {
        s->mdio_ro_data = hisi_femac_phy_read(s, reg_addr);
    }
}

/* ── TX path ───────────────────────────────────────────────────────── */

static void hisi_femac_do_tx(HisiFemacState *s, uint32_t pkt_info)
{
    /*
     * EQFRM_LEN register format (from hisi-femac kernel driver):
     *   bits [10:0]  = frame length including 4-byte FCS
     *   bits [15:11] = nr_frags (scatter-gather fragment count)
     *   bits [19:16] = protocol header length
     *   bits [23:20] = IP header length
     *   bit  26      = scatter-gather flag
     *   bit  27      = TX checksum offload
     *   bit  28      = UDP flag
     *   bit  29      = IPv6 flag
     *   bit  30      = VLAN flag
     *   bit  31      = TSO flag
     */
    bool is_sg = (pkt_info >> 26) & 1;
    uint32_t nr_frags = (pkt_info >> 11) & 0x1f;
    uint32_t len = pkt_info & 0x7FF;
    uint8_t buf[65536]; /* large enough for assembled SG packet */
    uint32_t real_len;

    if (is_sg) {
        /*
         * Scatter-gather mode: EQ_ADDR points to a tx_desc descriptor:
         *   Word0: total_len (bits 16:0)
         *   Word1: ipv6_id
         *   Word2: linear_addr (DMA address of head data)
         *   Word3: linear_len (bits 15:0)
         *   Then per-fragment pairs: { addr(32), size(16)|reserved(16) }
         */
        /* 4 header words + 2 words per fragment (max 31 frags) */
        uint32_t desc[4 + 2 * 31];
        uint32_t desc_words = 4 + 2 * nr_frags;
        if (dma_memory_read(&address_space_memory, s->eq_addr,
                            desc, desc_words * 4,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-femac: TX SG desc read failed at 0x%08x\n",
                          s->eq_addr);
            return;
        }

        /* Read linear (head) data */
        uint32_t linear_addr = desc[2];
        uint32_t linear_len = desc[3] & 0xFFFF;
        uint32_t offset = 0;

        if (linear_len > sizeof(buf)) {
            return;
        }
        if (dma_memory_read(&address_space_memory, linear_addr,
                            buf, linear_len,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return;
        }
        offset = linear_len;

        /* Read each fragment */
        for (uint32_t i = 0; i < nr_frags; i++) {
            uint32_t frag_addr = desc[4 + i * 2];
            uint32_t frag_len = desc[5 + i * 2] & 0xFFFF;
            if (offset + frag_len > sizeof(buf)) {
                return;
            }
            if (dma_memory_read(&address_space_memory, frag_addr,
                                buf + offset, frag_len,
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                return;
            }
            offset += frag_len;
        }
        real_len = offset;
    } else {
        /* Non-SG: EQ_ADDR points directly to packet data */
        /* Driver includes 4-byte FCS in length; strip it */
        if (len <= 4 || len > 2048 + 4) {
            return;
        }
        real_len = len - 4;

        if (dma_memory_read(&address_space_memory, s->eq_addr, buf, real_len,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-femac: TX DMA read failed at 0x%08x\n",
                          s->eq_addr);
            return;
        }
    }

    /*
     * TX checksum offload: when BIT_FLAG_TXCSUM (bit 27) is set, the
     * guest kernel left the IP/TCP/UDP checksums for hardware to compute.
     * Real FEMAC hardware fills them in; we must do the same.
     */
    if ((pkt_info >> 27) & 1) {
        net_checksum_calculate(buf, real_len, CSUM_ALL);
    }

    qemu_send_packet(qemu_get_queue(s->nic), buf, real_len);

    s->irq_raw |= IRQ_TX_PER_PACKET | IRQ_TX_FIFO_EMPTY;
    hisi_femac_update_irq(s);
}

/* ── RX path ───────────────────────────────────────────────────────── */

static bool hisi_femac_can_receive(NetClientState *nc)
{
    HisiFemacState *s = qemu_get_nic_opaque(nc);
    /*
     * Accept packets only when DMA buffers are available AND the
     * descriptor FIFO isn't full.  If we accept packets when the FIFO
     * is full, the driver can't process them fast enough (NAPI budget)
     * and the ring drains permanently — TCP never works because all
     * buffers get consumed by broadcast traffic during boot.
     */
    return s->rx_ring_count > 0 && s->iqfrm_des_count < RX_RING_SIZE;
}

static ssize_t hisi_femac_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    HisiFemacState *s = qemu_get_nic_opaque(nc);

    if (s->rx_ring_count == 0) {
        return -1;
    }

    /*
     * Bound the frame by the MAX_FRAME_SIZE field the driver programs into
     * MAC_SET, because the RX ring carries bare addresses -- no length -- so
     * this is the only size contract there is.  hisi_femac.c posts every RX
     * buffer at exactly MAX_FRAME_SIZE (1600) and writes the same value into
     * MAC_SET bits 10:0, and the MAC will not deliver a longer frame; a
     * backend that hands us one (a tap with a large MTU, a jumbo frame) was
     * written straight past the end of an skb, silently corrupting whatever
     * the guest had after it.  That is the worst shape of bug this model can
     * have: the damage lands later, in unrelated code, nowhere near here.
     * hisi-gmac already bounds its RX this way -- its descriptors carry a
     * buffer length, so it had something obvious to check against.
     *
     * Drop it as the MAC does, and report the frame consumed: returning -1
     * means "no room right now", and the net layer would re-offer the same
     * oversized frame forever, wedging the queue behind it.
     */
    uint32_t max_frame = s->mac_set & FEMAC_MAX_FRAME_SIZE_MASK;
    if (max_frame == 0) {
        /* Before the driver programs MAC_SET (U-Boot, early probe). */
        max_frame = FEMAC_MAX_FRAME_SIZE_DEFAULT;
    }
    if (size > max_frame) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-femac: dropping %zu-byte frame, MAC_SET caps RX "
                      "at %u bytes\n", size, max_frame);
        return size;
    }

    /* Pop buffer address from ring */
    uint32_t dma_addr = s->rx_ring[s->rx_ring_head];
    s->rx_ring_head = (s->rx_ring_head + 1) % RX_RING_SIZE;
    s->rx_ring_count--;

    /* Write packet data to guest memory */
    if (dma_memory_write(&address_space_memory, dma_addr, buf, size,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-femac: RX DMA write failed at 0x%08x\n", dma_addr);
        return -1;
    }

    /*
     * Push frame descriptor into FIFO: size + 4 for FCS.  RX_FRAME_LEN is
     * 12 bits; with the cap above, size + 4 can no longer reach 0x1000, so
     * the mask cannot silently report a frame shorter than the bytes that
     * were just written.
     */
    s->iqfrm_des_fifo[s->iqfrm_des_tail] = (size + 4) & 0xfff;
    s->iqfrm_des_tail = (s->iqfrm_des_tail + 1) % RX_RING_SIZE;
    s->iqfrm_des_count++;

    s->irq_raw |= IRQ_RX_RDY | IRQ_MULTI_RXRDY;
    hisi_femac_update_irq(s);

    return size;
}

/* ── MMIO ──────────────────────────────────────────────────────────── */

static uint64_t hisi_femac_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiFemacState *s = opaque;

    switch (offset) {
    /* Port */
    case REG_MAC_PORTSEL:
        return s->mac_portsel;
    case REG_MAC_PORTSET:
        return s->mac_portset;
    case REG_MAC_SET:
        return s->mac_set;
    case REG_RX_COALESCE_SET:
        return s->rx_coalesce_set;
    case REG_QLEN_SET:
        return s->qlen_set;
    case REG_FC_LEVEL:
        return s->fc_level;
    case REG_IQFRM_DES:
        if (s->iqfrm_des_count > 0) {
            uint32_t des = s->iqfrm_des_fifo[s->iqfrm_des_head];
            s->iqfrm_des_head = (s->iqfrm_des_head + 1) % RX_RING_SIZE;
            s->iqfrm_des_count--;
            return des;
        }
        return 0;
    case REG_EQFRM_LEN:
        /* U-Boot reads this register (TX frame length / status).
         * Return 0 to indicate TX queue is empty (complete). */
        return 0;
    case REG_ADDRQ_STAT:
        /* TX_CNT_INUSE (bits 5:0) = 0 (instant completion),
         * RX_PKT_CNT   (bits 13:8) = pending RX descriptors in FIFO,
         * BIT_TX_READY  (bit 24) always set,
         * BIT_RX_READY  (bit 25) set if ring has room for buffers */
        return ADDRQ_TX_READY |
               ((s->iqfrm_des_count & 0x3F) << 8) |
               ((s->rx_ring_count < RX_RING_SIZE) ? (1 << 25) : 0);
    case REG_RX_COE_CTRL:
        return s->rx_coe_ctrl;
    case REG_U_MAC_TX_IPGCTRL:
        return s->tx_ipgctrl;

    /* hieth-sf TX queue: instantaneous TX model ⇒ always "empty" */
    case REG_HIETH_TX_Q_STAT:
        return 0;

    case REG_PORT_VENDOR_INIT:
        return s->port_vendor_init;

    /* U-Boot per-port MDIO config registers */
    case REG_U_MDIO_PHYADDR:
        return s->u_mdio_phyaddr;
    case REG_U_MDIO_RO_STAT:
        return 0; /* MDIO status: always ready */
    case REG_U_MDIO_ANEG_CTRL:
        return s->u_mdio_aneg_ctrl;
    case REG_U_MDIO_IRQENA:
        return s->u_mdio_irqena;

    /* Linux kernel MDIO registers */
    case REG_MDIO_RWCTRL:
        return s->mdio_rwctrl;
    case REG_MDIO_RO_DATA:
        return s->mdio_ro_data;

    /* GLB */
    case REG_GLB_HOSTMAC_L32:
        return s->hostmac_l32;
    case REG_GLB_HOSTMAC_H16:
        return s->hostmac_h16;
    case REG_GLB_SOFT_RESET:
        return s->soft_reset;
    case REG_GLB_FWCTRL:
        return s->fwctrl;
    case REG_GLB_MACTCTRL:
        return s->mactctrl;
    case REG_GLB_ENDIAN_MOD:
        return s->endian_mod;
    case REG_GLB_IRQ_STAT:
        return s->irq_raw & s->irq_ena;
    case REG_GLB_IRQ_ENA:
        return s->irq_ena;
    case REG_GLB_IRQ_RAW:
        /*
         * When the guest polls for RX_RDY and it's not set, kick the
         * vCPU to force cpu_exec() to return. This lets the main loop
         * run SLIRP/TAP and deliver pending network replies.
         * Essential for ARM926 SoCs without arch_timer.
         */
        if (!(s->irq_raw & IRQ_RX_RDY) && s->rx_ring_count > 0) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        return s->irq_raw;
    default:
        if (offset >= REG_MAC_FILTER_BASE &&
            offset < REG_MAC_FILTER_BASE + 16 * 4) {
            return s->mac_filter[(offset - REG_MAC_FILTER_BASE) / 4];
        }
        qemu_log_mask(LOG_UNIMP,
                      "hisi-femac: read from unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void hisi_femac_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    HisiFemacState *s = opaque;

    switch (offset) {
    /* Port */
    case REG_MAC_PORTSEL:
        s->mac_portsel = val;
        break;
    case REG_MAC_PORTSET:
        s->mac_portset = val;
        break;
    case REG_MAC_SET:
        s->mac_set = val;
        break;
    case REG_RX_COALESCE_SET:
        s->rx_coalesce_set = val;
        break;
    case REG_QLEN_SET:
        s->qlen_set = val;
        break;
    case REG_FC_LEVEL:
        s->fc_level = val;
        break;
    case REG_IQ_ADDR:
        /* Push RX buffer address into ring */
        if (s->rx_ring_count < RX_RING_SIZE) {
            s->rx_ring[s->rx_ring_tail] = val;
            s->rx_ring_tail = (s->rx_ring_tail + 1) % RX_RING_SIZE;
            s->rx_ring_count++;
        }
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        /* Arm RX poll timer for polled drivers (U-Boot on ARM926) */
        if (s->rx_ring_count > 0 &&
            !timer_pending(s->rx_poll_timer)) {
            timer_mod(s->rx_poll_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + FEMAC_RX_POLL_NS);
        }
        break;
    case REG_EQ_ADDR:
        s->eq_addr = val;
        break;
    case REG_EQFRM_LEN:
        hisi_femac_do_tx(s, val);
        break;
    case REG_RX_COE_CTRL:
        s->rx_coe_ctrl = val;
        break;
    case REG_U_MAC_TX_IPGCTRL:
        s->tx_ipgctrl = val;
        break;

    /* hieth-sf TX queue writes are no-ops: TX is instantaneous */
    case REG_HIETH_TX_Q_STAT:
    case REG_HIETH_TX_Q_CTRL:
        break;

    case REG_PORT_VENDOR_INIT:
        s->port_vendor_init = val;
        break;

    /* U-Boot per-port MDIO config registers */
    case REG_U_MDIO_PHYADDR:
        s->u_mdio_phyaddr = val;
        break;
    case REG_U_MDIO_RO_STAT:
        break; /* read-only */
    case REG_U_MDIO_ANEG_CTRL:
        s->u_mdio_aneg_ctrl = val;
        break;
    case REG_U_MDIO_IRQENA:
        s->u_mdio_irqena = val;
        break;

    /* Linux kernel MDIO registers */
    case REG_MDIO_RWCTRL:
        hisi_femac_mdio_access(s, val);
        break;

    /* GLB */
    case REG_GLB_HOSTMAC_L32:
        s->hostmac_l32 = val;
        break;
    case REG_GLB_HOSTMAC_H16:
        s->hostmac_h16 = val;
        break;
    case REG_GLB_SOFT_RESET:
        /*
         * Asserting SOFT_RESET_ALL resets the MAC, and a reset MAC has
         * forgotten every RX buffer the driver had posted to it.  Dropping
         * them here is not housekeeping -- it is the whole point.
         *
         * hisi_femac_net_close() calls hisi_femac_free_skb_rings(), which
         * dma_unmap_single()s and frees every posted skb, and the driver
         * brackets open/close with hisi_femac_port_reset().  Keeping the
         * addresses across the reset left us holding kernel pointers the
         * guest had freed and the page allocator had handed to something
         * else; the next frame to arrive was DMA'd straight into them.
         *
         * The damage lands wherever those pages went, so the guest dies
         * later in whatever walks them -- anon_vma_interval_tree_remove,
         * __rb_insert_augmented, unmap_vmas, set_page_dirty -- with nothing
         * pointing back at the NIC.  Removing the interface made it go away
         * (0/8 panics with -net none, 5/8 with the NIC attached).
         */
        if ((val & SOFT_RESET_ALL) && !(s->soft_reset & SOFT_RESET_ALL)) {
            s->rx_ring_head = 0;
            s->rx_ring_tail = 0;
            s->rx_ring_count = 0;
            s->iqfrm_des_head = 0;
            s->iqfrm_des_tail = 0;
            s->iqfrm_des_count = 0;
            s->irq_raw &= ~(IRQ_RX_RDY | IRQ_MULTI_RXRDY);
            hisi_femac_update_irq(s);
        }
        s->soft_reset = val;
        break;
    case REG_GLB_FWCTRL:
        s->fwctrl = val;
        break;
    case REG_GLB_MACTCTRL:
        s->mactctrl = val;
        break;
    case REG_GLB_ENDIAN_MOD:
        s->endian_mod = val;
        break;
    case REG_GLB_IRQ_ENA:
        s->irq_ena = val;
        hisi_femac_update_irq(s);
        break;
    case REG_GLB_IRQ_RAW:
        /* W1C: clear bits written as 1 */
        s->irq_raw &= ~val;
        /* Re-assert RX_RDY if unprocessed descriptors remain in FIFO */
        if ((val & IRQ_RX_RDY) && s->iqfrm_des_count > 0) {
            s->irq_raw |= IRQ_RX_RDY;
        }
        hisi_femac_update_irq(s);
        break;
    default:
        if (offset >= REG_MAC_FILTER_BASE &&
            offset < REG_MAC_FILTER_BASE + 16 * 4) {
            s->mac_filter[(offset - REG_MAC_FILTER_BASE) / 4] = val;
            break;
        }
        qemu_log_mask(LOG_UNIMP,
                      "hisi-femac: write to unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", offset, val);
        break;
    }
}

static const MemoryRegionOps hisi_femac_ops = {
    .read = hisi_femac_read,
    .write = hisi_femac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static NetClientInfo hisi_femac_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = hisi_femac_can_receive,
    .receive = hisi_femac_receive,
};

static void hisi_femac_init(Object *obj)
{
    HisiFemacState *s = HISI_FEMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_femac_ops, s,
                          "hisi-femac", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void hisi_femac_realize(DeviceState *dev, Error **errp)
{
    HisiFemacState *s = HISI_FEMAC(dev);

    s->nic = qemu_new_nic(&hisi_femac_net_info, &s->conf,
                           object_get_typename(OBJECT(dev)),
                           dev->id, &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    s->rx_poll_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                     hisi_femac_rx_poll, s);
}

static void hisi_femac_reset(DeviceState *dev)
{
    HisiFemacState *s = HISI_FEMAC(dev);

    s->mac_portsel = 0;
    s->mac_portset = 0;
    s->mac_set = 0;
    s->rx_coalesce_set = 0;
    s->qlen_set = 0;
    s->fc_level = 0;
    s->rx_coe_ctrl = 0;
    s->eq_addr = 0;
    s->rx_ring_head = 0;
    s->rx_ring_tail = 0;
    s->rx_ring_count = 0;
    s->iqfrm_des_head = 0;
    s->iqfrm_des_tail = 0;
    s->iqfrm_des_count = 0;
    s->mdio_rwctrl = MDIO_RW_FINISH; /* idle = ready */
    s->mdio_ro_data = 0;
    s->hostmac_l32 = 0;
    s->hostmac_h16 = 0;
    s->soft_reset = 0;
    s->fwctrl = 0;
    s->mactctrl = 0;
    s->irq_ena = 0;
    s->irq_raw = 0;
    memset(s->mac_filter, 0, sizeof(s->mac_filter));

    /* PHY defaults: 100M FD, autoneg, link up, autoneg complete */
    s->phy_bmcr = 0x3100;  /* 100Mbps, full duplex, autoneg enable */
    s->phy_bmsr = 0x786d;  /* link up, autoneg complete, 100TX-FD capable */
    s->phy_anar = 0x01e1;  /* 100TX-FD/HD, 10TX-FD/HD */
    s->phy_anlpar = 0x45e1; /* partner: 100TX-FD */
}

static const Property hisi_femac_properties[] = {
    DEFINE_NIC_PROPERTIES(HisiFemacState, conf),
    DEFINE_PROP_UINT8("phy-addr", HisiFemacState, phy_addr, FEMAC_PHY_ADDR),
};

static void hisi_femac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hisi_femac_realize;
    device_class_set_legacy_reset(dc, hisi_femac_reset);
    device_class_set_props(dc, hisi_femac_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo hisi_femac_info = {
    .name          = TYPE_HISI_FEMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiFemacState),
    .instance_init = hisi_femac_init,
    .class_init    = hisi_femac_class_init,
};

static void hisi_femac_register_types(void)
{
    type_register_static(&hisi_femac_info);
}

type_init(hisi_femac_register_types)
