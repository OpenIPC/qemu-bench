/*
 * Xiongmai XM510 Ethernet: a Synopsys DWMAC 10/100 core ("xmmac").
 *
 * The vendor driver is a fork of stmmac (drivers/net/xmmac in the 3.0.101
 * kernel), configured as the 10/100 core with normal descriptors
 * (xm510_gmac_platdata: has_gmac = 0, enh_desc = 0) and an RMII PHY at
 * MDIO address 1 (xm510_phy_platdata).  Xiongmai rearranged the 10/100
 * core's MAC block, so the offsets are the ones in its dwmac100.h, not
 * upstream stmmac's:
 *
 *   MAC_CONTROL 0x00  FRAME_FLT 0x04  HASH_HIGH 0x08  HASH_LOW 0x0C
 *   MII_ADDR    0x10  MII_DATA  0x14  FLOW_CTRL 0x18
 *   ADDR0_HIGH  0x40  ADDR0_LOW 0x44
 *
 * The DMA block is the standard Synopsys one at +0x1000 (dwmac_dma.h):
 * BUS_MODE, TX/RX poll demand, RX/TX list bases, STATUS (write 1 to clear),
 * CONTROL (ST/SR), INTR_ENA, and the current-descriptor/buffer pointers.
 *
 * Normal descriptors, four words: des0 status (OWN bit 31), des1 control
 * (buf1 size 10:0, buf2 size 21:11, chained 24, end of ring 25; TX also
 * first 29, last 30, IRQ-on-completion 31), des2 buf1, des3 buf2 or next.
 * In a ring, BUS_MODE's descriptor skip length (DSL, 6:2) puts that many
 * words between one descriptor and the next: 0 for xmmac, 2 for the Anyka
 * AK39 driver, whose descriptors carry two software words.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "net/eth.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/dma.h"

#include <zlib.h>

#define TYPE_XM_DWMAC "xm-dwmac"
OBJECT_DECLARE_SIMPLE_TYPE(XmDwmacState, XM_DWMAC)

#define MAC_CONTROL     0x00
#define MAC_FRAME_FLT   0x04
#define MAC_HASH_HIGH   0x08
#define MAC_HASH_LOW    0x0C
#define MAC_MII_ADDR    0x10
#define MAC_MII_DATA    0x14
#define MAC_FLOW_CTRL   0x18
#define MAC_ADDR0_HIGH  0x40
#define MAC_ADDR0_LOW   0x44

#define MAC_CONTROL_TE  (1u << 3)
#define MAC_CONTROL_RE  (1u << 2)
#define MII_BUSY        (1u << 0)
#define MII_WRITE       (1u << 1)

#define DMA_BUS_MODE    0x1000
#define DMA_TX_POLL     0x1004
#define DMA_RX_POLL     0x1008
#define DMA_RX_BASE     0x100C
#define DMA_TX_BASE     0x1010
#define DMA_STATUS      0x1014
#define DMA_CONTROL     0x1018
#define DMA_INTR_ENA    0x101C
#define DMA_MISSED      0x1020
#define DMA_CUR_TX_DESC 0x1048
#define DMA_CUR_RX_DESC 0x104C
#define DMA_CUR_TX_BUF  0x1050
#define DMA_CUR_RX_BUF  0x1054

#define DMA_BUS_SWR     (1u << 0)
#define DMA_BUS_DSL(v)  (((v) >> 2) & 0x1f)
#define DMA_CONTROL_SR  (1u << 1)
#define DMA_CONTROL_ST  (1u << 13)

#define ST_TI           (1u << 0)
#define ST_TU           (1u << 2)
#define ST_RI           (1u << 6)
#define ST_RU           (1u << 7)
#define ST_AIS          (1u << 15)
#define ST_NIS          (1u << 16)
#define ST_NORMAL       (ST_TI | ST_RI)
#define ST_ABNORMAL     (ST_TU | ST_RU)

#define DES0_OWN        (1u << 31)
#define DES1_TCH        (1u << 24)
#define DES1_TER        (1u << 25)
#define DES1_TFS        (1u << 29)
#define DES1_TLS        (1u << 30)
#define DES1_TIC        (1u << 31)
#define RDES0_LS        (1u << 8)
#define RDES0_FS        (1u << 9)
#define RDES0_FL_SHIFT  16

#define PHY_ADDR        1
#define FRAME_MAX       1536
/* FRAME_FLT bits, as xmmac's dwmac100.h names them (MAC_CONTROL_*) */
#define FLT_PR          (1u << 0)       /* promiscuous */
#define FLT_DBF         (1u << 11)      /* disable broadcast frames */
#define RX_START_DELAY_NS  (1000 * 1000)   /* 1 ms; see DMA_CONTROL */
#define REGS_SIZE       0x10000

typedef struct {
    uint32_t des0, des1, des2, des3;
} XmDesc;

struct XmDwmacState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    uint32_t mac_ctrl, frame_flt, hash_hi, hash_lo, mii_addr, mii_data;
    uint32_t flow_ctrl, addr_hi, addr_lo;
    uint32_t bus_mode, rx_base, tx_base, status, control, intr_ena, missed;
    uint32_t cur_tx, cur_rx, cur_tx_buf, cur_rx_buf;

    uint16_t phy_bmcr;
    uint32_t regs_size;
    bool rx_fcs;

    uint8_t frame[FRAME_MAX];
    uint32_t frame_len;
    bool frame_overflow;

    /* Receive DMA has started fetching descriptors; see DMA_CONTROL. */
    bool rx_running;
    QEMUTimer *rx_start;
};

/* NIS and AIS summarise the normal and abnormal bits that are set; the line
 * is up when a summary is set and its enable, and one of its sources'
 * enables, are on (DMA_INTR_ENA: NIE 16, AIE 15). */
static void xm_dwmac_update_irq(XmDwmacState *s)
{
    s->status &= ~(ST_NIS | ST_AIS);
    if (s->status & ST_NORMAL) {
        s->status |= ST_NIS;
    }
    if (s->status & ST_ABNORMAL) {
        s->status |= ST_AIS;
    }
    const bool normal = (s->intr_ena & ST_NIS) &&
                        (s->status & s->intr_ena & ST_NORMAL);
    const bool abnormal = (s->intr_ena & ST_AIS) &&
                          (s->status & s->intr_ena & ST_ABNORMAL);
    qemu_set_irq(s->irq, normal || abnormal);
}

/* An RMII PHY that has always negotiated 100 Mbit/s full duplex. */
static uint16_t xm_dwmac_phy_read(XmDwmacState *s, int reg)
{
    switch (reg) {
    case 0:  /* BMCR */
        return s->phy_bmcr;
    case 1:  /* BMSR: 100/10 FD/HD, AN capable and complete, link up */
        return 0x7809 | 0x0020 | 0x0004;
    case 2:  /* PHYID1 */
        return 0x0007;
    case 3:  /* PHYID2 */
        return 0xc0f1;
    case 4:  /* ANAR */
        return 0x01e1;
    case 5:  /* ANLPAR: partner does the same, acknowledged */
        return 0x45e1;
    case 6:  /* ANER: the partner negotiates (Anyka's driver keys on it) */
        return 0x0001;
    default:
        return 0;
    }
}

static void xm_dwmac_mii(XmDwmacState *s, uint32_t val)
{
    const int phy = (val >> 11) & 0x1f;
    const int reg = (val >> 6) & 0x1f;

    s->mii_addr = val & ~MII_BUSY;   /* every transfer is done at once */
    if (val & MII_WRITE) {
        if (phy == PHY_ADDR && reg == 0) {
            /* Reset and restart-autoneg self-clear. */
            s->phy_bmcr = s->mii_data & ~(0x8000 | 0x0200);
        }
        return;
    }
    s->mii_data = phy == PHY_ADDR ? xm_dwmac_phy_read(s, reg) : 0xffff;
}

static void xm_dwmac_read_desc(hwaddr addr, XmDesc *d)
{
    dma_memory_read(&address_space_memory, addr, d, sizeof(*d),
                    MEMTXATTRS_UNSPECIFIED);
    d->des0 = le32_to_cpu(d->des0);
    d->des1 = le32_to_cpu(d->des1);
    d->des2 = le32_to_cpu(d->des2);
    d->des3 = le32_to_cpu(d->des3);
}

static void xm_dwmac_write_des0(hwaddr addr, uint32_t des0)
{
    des0 = cpu_to_le32(des0);
    dma_memory_write(&address_space_memory, addr, &des0, 4,
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * The next descriptor: chained via des3, the list base at end of ring, or
 * the next one in the ring, DSL words past this one's end.
 */
static uint32_t xm_dwmac_next(XmDwmacState *s, uint32_t cur, const XmDesc *d,
                              uint32_t base)
{
    if (d->des1 & DES1_TER) {
        return base;
    }
    if (d->des1 & DES1_TCH) {
        return d->des3;
    }
    return cur + sizeof(XmDesc) + 4 * DMA_BUS_DSL(s->bus_mode);
}

static void xm_dwmac_tx(XmDwmacState *s)
{
    int guard = 256;

    if (!(s->control & DMA_CONTROL_ST)) {
        return;
    }
    while (guard--) {
        XmDesc d;
        const uint32_t at = s->cur_tx;

        xm_dwmac_read_desc(at, &d);
        if (!(d.des0 & DES0_OWN)) {
            s->status |= ST_TU;
            break;
        }
        if (d.des1 & DES1_TFS) {
            s->frame_len = 0;
            s->frame_overflow = false;
        }
        const uint32_t len1 = d.des1 & 0x7ff;
        const uint32_t len2 = (d.des1 & DES1_TCH) ? 0 : (d.des1 >> 11) & 0x7ff;
        /* Past FRAME_MAX the whole frame is dropped, never its prefix sent. */
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
        if (d.des1 & DES1_TLS) {
            if (s->frame_overflow) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "xm-dwmac: TX frame over %d bytes dropped\n",
                              FRAME_MAX);
            } else if (s->mac_ctrl & MAC_CONTROL_TE) {
                qemu_send_packet(qemu_get_queue(s->nic), s->frame,
                                 s->frame_len);
            }
            s->frame_len = 0;
            s->frame_overflow = false;
            if (d.des1 & DES1_TIC) {
                s->status |= ST_TI;
            }
        }
        xm_dwmac_write_des0(at, d.des0 & ~DES0_OWN);
        s->cur_tx = xm_dwmac_next(s, at, &d, s->tx_base);
    }
    xm_dwmac_update_irq(s);
}

static bool xm_dwmac_can_receive(NetClientState *nc)
{
    XmDwmacState *s = qemu_get_nic_opaque(nc);
    XmDesc d;

    if (!s->rx_running || !(s->mac_ctrl & MAC_CONTROL_RE)) {
        return false;
    }
    xm_dwmac_read_desc(s->cur_rx, &d);
    return d.des0 & DES0_OWN;
}

/* Perfect filter on ADDR0 for unicast; multicast is let through whatever
 * the hash says, which only hands the guest a superset to filter. */
static bool xm_dwmac_accept(XmDwmacState *s, const uint8_t *da)
{
    static const uint8_t bcast[ETH_ALEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };
    const uint8_t own[ETH_ALEN] = {
        s->addr_lo, s->addr_lo >> 8, s->addr_lo >> 16, s->addr_lo >> 24,
        s->addr_hi, s->addr_hi >> 8,
    };

    if (s->frame_flt & FLT_PR) {
        return true;
    }
    if (!memcmp(da, bcast, ETH_ALEN)) {
        return !(s->frame_flt & FLT_DBF);
    }
    if (da[0] & 1) {
        return true;
    }
    return !memcmp(da, own, ETH_ALEN);
}

static ssize_t xm_dwmac_receive(NetClientState *nc, const uint8_t *buf,
                                size_t size)
{
    XmDwmacState *s = qemu_get_nic_opaque(nc);
    size_t done = 0;
    bool first = true;
    int guard = 64;

    if (!s->rx_running || !(s->mac_ctrl & MAC_CONTROL_RE)) {
        return size;
    }
    if (size < ETH_ALEN || !xm_dwmac_accept(s, buf)) {
        return size;
    }
    /* The frame length excludes the FCS: with checksum offload off the
     * core drops it, and both vendor drivers are built not to subtract it
     * (EDIT_ZENGCHUANJIE in xmmac/stmmac_main.c).  A core that keeps it
     * (rx-fcs: Goke, whose kernel driver subtracts 4) stores it after the
     * frame and counts it in the length. */
    uint8_t fcs_frame[FRAME_MAX + 4];
    const uint8_t *data = buf;
    size_t len = size;
    if (s->rx_fcs && size > FRAME_MAX) {
        /* Past what the core takes with its FCS: dropped, not delivered
         * under the other length convention. */
        s->missed++;
        return size;
    }
    if (s->rx_fcs) {
        memcpy(fcs_frame, buf, size);
        stl_le_p(fcs_frame + size, crc32(0, buf, size));
        data = fcs_frame;
        len = size + 4;
    }
    const size_t total = len;

    while (done < total && guard--) {
        XmDesc d;
        const uint32_t at = s->cur_rx;

        xm_dwmac_read_desc(at, &d);
        if (!(d.des0 & DES0_OWN)) {
            s->status |= ST_RU;
            s->missed++;
            xm_dwmac_update_irq(s);
            return size;
        }
        const uint32_t room = d.des1 & 0x7ff;
        const uint32_t n = MIN(room, total - done);
        const uint32_t from_buf = done < len ? MIN(n, len - done) : 0;
        if (from_buf) {
            dma_memory_write(&address_space_memory, d.des2, data + done,
                             from_buf, MEMTXATTRS_UNSPECIFIED);
        }
        done += n;
        s->cur_rx_buf = d.des2;

        uint32_t des0 = 0;
        if (first) {
            des0 |= RDES0_FS;
            first = false;
        }
        if (done >= total) {
            des0 |= RDES0_LS | ((uint32_t)total << RDES0_FL_SHIFT);
        }
        xm_dwmac_write_des0(at, des0);
        s->cur_rx = xm_dwmac_next(s, at, &d, s->rx_base);
    }
    if (done < total) {
        /* Ran out of guard before the frame did: like a ring with no free
         * descriptor, the frame is lost rather than half-delivered. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "xm-dwmac: RX frame of %zu bytes spans over 64 "
                      "descriptors, dropped\n", size);
        s->status |= ST_RU;
        s->missed++;
        xm_dwmac_update_irq(s);
        return size;
    }
    s->status |= ST_RI;
    xm_dwmac_update_irq(s);
    return size;
}

static void xm_dwmac_rx_start(void *opaque)
{
    XmDwmacState *s = opaque;

    s->rx_running = true;
    qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

static void xm_dwmac_soft_reset(XmDwmacState *s)
{
    timer_del(s->rx_start);
    s->rx_running = false;
    s->bus_mode = 0;
    s->status = 0;
    s->control = 0;
    s->intr_ena = 0;
    s->missed = 0;
    s->rx_base = s->tx_base = 0;
    s->cur_tx = s->cur_rx = 0;
    s->frame_len = 0;
    s->frame_overflow = false;
    xm_dwmac_update_irq(s);
}

static uint64_t xm_dwmac_read(void *opaque, hwaddr off, unsigned size)
{
    XmDwmacState *s = opaque;

    switch (off) {
    case MAC_CONTROL:
        return s->mac_ctrl;
    case MAC_FRAME_FLT:
        return s->frame_flt;
    case MAC_HASH_HIGH:
        return s->hash_hi;
    case MAC_HASH_LOW:
        return s->hash_lo;
    case MAC_MII_ADDR:
        return s->mii_addr;
    case MAC_MII_DATA:
        return s->mii_data;
    case MAC_FLOW_CTRL:
        return s->flow_ctrl;
    case MAC_ADDR0_HIGH:
        return s->addr_hi;
    case MAC_ADDR0_LOW:
        return s->addr_lo;
    case DMA_BUS_MODE:
        return s->bus_mode;
    case DMA_RX_BASE:
        return s->rx_base;
    case DMA_TX_BASE:
        return s->tx_base;
    case DMA_STATUS:
        return s->status;
    case DMA_CONTROL:
        return s->control;
    case DMA_INTR_ENA:
        return s->intr_ena;
    case DMA_MISSED: {
        const uint32_t v = s->missed & 0xffff;
        s->missed = 0;        /* clear on read */
        return v;
    }
    case DMA_CUR_TX_DESC:
        return s->cur_tx;
    case DMA_CUR_RX_DESC:
        return s->cur_rx;
    case DMA_CUR_TX_BUF:
        return s->cur_tx_buf;
    case DMA_CUR_RX_BUF:
        return s->cur_rx_buf;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-dwmac: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void xm_dwmac_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    XmDwmacState *s = opaque;

    switch (off) {
    case MAC_CONTROL:
        s->mac_ctrl = val;
        if (val & MAC_CONTROL_RE) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case MAC_FRAME_FLT:
        s->frame_flt = val;
        break;
    case MAC_HASH_HIGH:
        s->hash_hi = val;
        break;
    case MAC_HASH_LOW:
        s->hash_lo = val;
        break;
    case MAC_MII_ADDR:
        xm_dwmac_mii(s, val);
        break;
    case MAC_MII_DATA:
        s->mii_data = val & 0xffff;
        break;
    case MAC_FLOW_CTRL:
        s->flow_ctrl = val;
        break;
    case MAC_ADDR0_HIGH:
        s->addr_hi = val;
        break;
    case MAC_ADDR0_LOW:
        s->addr_lo = val;
        break;
    case DMA_BUS_MODE:
        if (val & DMA_BUS_SWR) {
            xm_dwmac_soft_reset(s);   /* SWR reads back clear: done */
        } else {
            s->bus_mode = val;
        }
        break;
    case DMA_TX_POLL:
        xm_dwmac_tx(s);
        break;
    case DMA_RX_POLL:
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case DMA_RX_BASE:
        s->rx_base = s->cur_rx = val;
        break;
    case DMA_TX_BASE:
        s->tx_base = s->cur_tx = val;
        break;
    case DMA_STATUS:
        s->status &= ~(val & 0x0001ffff);
        xm_dwmac_update_irq(s);
        break;
    case DMA_CONTROL:
        /*
         * Starting receive DMA does not hand the guest a frame in the same
         * instruction: the engine fetches its first descriptor first.  The
         * vendor stmmac_open() depends on that gap -- it sets SR before
         * napi_enable(), and an RX interrupt in between masks DMA
         * interrupts for good (_stmmac_schedule disables them, then
         * napi_schedule is a no-op on a disabled NAPI).  QEMU queues
         * frames while SR is clear, so flushing them on the SR write
         * itself hit that window every time on a busy TAP link.
         */
        if ((val & DMA_CONTROL_SR) && !(s->control & DMA_CONTROL_SR)) {
            timer_mod(s->rx_start, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      RX_START_DELAY_NS);
        } else if (!(val & DMA_CONTROL_SR)) {
            timer_del(s->rx_start);
            s->rx_running = false;
        }
        s->control = val;
        if (val & DMA_CONTROL_ST) {
            xm_dwmac_tx(s);
        }
        break;
    case DMA_INTR_ENA:
        s->intr_ena = val;
        xm_dwmac_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-dwmac: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        break;
    }
}

static const MemoryRegionOps xm_dwmac_ops = {
    .read = xm_dwmac_read,
    .write = xm_dwmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static NetClientInfo xm_dwmac_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = xm_dwmac_can_receive,
    .receive = xm_dwmac_receive,
};

static void xm_dwmac_realize(DeviceState *dev, Error **errp)
{
    XmDwmacState *s = XM_DWMAC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &xm_dwmac_ops, s,
                          "xm-dwmac", s->regs_size);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    s->rx_start = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_dwmac_rx_start, s);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&xm_dwmac_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void xm_dwmac_reset(DeviceState *dev)
{
    XmDwmacState *s = XM_DWMAC(dev);
    const uint8_t *m = s->conf.macaddr.a;

    s->mac_ctrl = s->frame_flt = s->hash_hi = s->hash_lo = 0;
    s->mii_addr = s->mii_data = s->flow_ctrl = 0;
    /* U-Boot leaves the address it was given in ADDR0; with none the
     * driver says "no valid MAC address", so start from the NIC's. */
    s->addr_hi = m[4] | (m[5] << 8);
    s->addr_lo = m[0] | (m[1] << 8) | (m[2] << 16) | ((uint32_t)m[3] << 24);
    s->phy_bmcr = 0x3100;           /* AN enabled, 100 Mbit/s, full duplex */
    xm_dwmac_soft_reset(s);
}

static const Property xm_dwmac_props[] = {
    DEFINE_NIC_PROPERTIES(XmDwmacState, conf),
    /* The window the SoC decodes: 64 KiB on Xiongmai; the Goke GK710x
     * has only the MAC and DMA pages before its VIC2. */
    DEFINE_PROP_UINT32("regs-size", XmDwmacState, regs_size, REGS_SIZE),
    DEFINE_PROP_BOOL("rx-fcs", XmDwmacState, rx_fcs, false),
};

static void xm_dwmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xm_dwmac_realize;
    device_class_set_legacy_reset(dc, xm_dwmac_reset);
    device_class_set_props(dc, xm_dwmac_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo xm_dwmac_info = {
    .name          = TYPE_XM_DWMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmDwmacState),
    .class_init    = xm_dwmac_class_init,
};

static void xm_dwmac_register_types(void)
{
    type_register_static(&xm_dwmac_info);
}

type_init(xm_dwmac_register_types)
