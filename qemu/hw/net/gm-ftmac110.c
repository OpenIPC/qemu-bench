/*
 * Faraday FTMAC110 10/100 Ethernet (the FTMAC100 register and descriptor
 * layout) with an internal PHY, as on the Grain Media GM8126.
 *
 *   0x00 ISR (read clears)  0x04 IMR   0x08 / 0x0C station address
 *   0x18 transmit poll   0x20 / 0x24 transmit / receive ring base
 *   0x88 MACCR           0x90 PHYCR    0x94 PHYWDATA
 *
 * Descriptors are 16 bytes: word 0 bit 31 the owner (set: the MAC), word 1
 * bit 31 the ring's end and bits 10:0 the buffer size, word 2 the buffer.
 * Transmit: word 1 bits 28 / 27 first / last segment.  Receive: word 0
 * bits 29 / 28 first / last segment, bits 10:0 the frame length.  A frame
 * starts at a half-word that is not word aligned: the address' bit 1 is
 * forced on (Faraday's U-Boot driver says the receive DMA takes no other;
 * the GM8126 kernel driver gives word addresses and reads two bytes in).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "net/eth.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_GM_FTMAC110 "gm-ftmac110"
OBJECT_DECLARE_SIMPLE_TYPE(GmFtmac110State, GM_FTMAC110)

#define FTMAC_SIZE      0x1000
#define FTMAC_NREGS     (0x100 / 4)
#define FTMAC_BUF       2048
#define FTMAC_MAX_DESC  1024

#define R_ISR           0x00
#define R_IMR           0x04
#define R_MADR          0x08
#define R_LADR          0x0C
#define R_TXPD          0x18
#define R_RXPD          0x1C
#define R_TXBASE        0x20
#define R_RXBASE        0x24
#define R_MACCR         0x88
#define R_PHYCR         0x90
#define R_PHYWDATA      0x94

#define INT_RX          (1u << 0)
#define INT_NORXBUF     (1u << 1)
#define INT_XPKT_OK     (1u << 4)

#define CR_XDMA         (1u << 0)
#define CR_RDMA         (1u << 1)
#define CR_RESET        (1u << 2)
#define CR_XMT          (1u << 5)
#define CR_RCV          (1u << 8)
#define CR_HT_MULTI     (1u << 9)
#define CR_RCV_ALL      (1u << 12)
#define CR_MULTI        (1u << 17)
#define CR_BROAD        (1u << 18)

#define PHYCR_READ      (1u << 26)
#define PHYCR_WRITE     (1u << 27)

#define DES_OWN         (1u << 31)
#define DES_END         (1u << 31)

struct GmFtmac110State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;
    uint32_t regs[FTMAC_NREGS];
    uint16_t phy[32];
    uint32_t tx, rx;            /* current descriptors */
    uint32_t txlen;
    uint8_t txbuf[FTMAC_BUF];
};

static void ftmac_update(GmFtmac110State *s)
{
    qemu_set_irq(s->irq, (s->regs[R_ISR / 4] & s->regs[R_IMR / 4]) != 0);
}

static void ftmac_desc(hwaddr a, uint32_t *d, bool write)
{
    int i;

    if (!write) {
        dma_memory_read(&address_space_memory, a, d, 12,
                        MEMTXATTRS_UNSPECIFIED);
        for (i = 0; i < 3; i++) {
            d[i] = le32_to_cpu(d[i]);
        }
    } else {
        uint32_t v = cpu_to_le32(d[0]);

        dma_memory_write(&address_space_memory, a, &v, 4,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

static void ftmac_transmit(GmFtmac110State *s)
{
    int n;

    if ((s->regs[R_MACCR / 4] & (CR_XDMA | CR_XMT)) != (CR_XDMA | CR_XMT)) {
        return;
    }
    for (n = 0; n < FTMAC_MAX_DESC; n++) {
        uint32_t d[3], len;

        ftmac_desc(s->tx, d, false);
        if (!(d[0] & DES_OWN)) {
            break;
        }
        len = d[1] & 0x7FF;
        if (d[1] & (1u << 28)) {
            s->txlen = 0;
        }
        if (s->txlen + len <= FTMAC_BUF) {
            dma_memory_read(&address_space_memory, d[2], s->txbuf + s->txlen,
                            len, MEMTXATTRS_UNSPECIFIED);
            s->txlen += len;
        }
        if (d[1] & (1u << 27)) {
            uint32_t l = s->txlen;

            if (l < 60) {
                memset(s->txbuf + l, 0, 60 - l);
                l = 60;
            }
            s->txlen = 0;
            qemu_send_packet(qemu_get_queue(s->nic), s->txbuf, l);
            s->regs[R_ISR / 4] |= INT_XPKT_OK;
        }
        d[0] = 0;
        ftmac_desc(s->tx, d, true);
        s->tx = d[1] & DES_END ? s->regs[R_TXBASE / 4] : s->tx + 16;
    }
    ftmac_update(s);
}

static bool ftmac_can_receive(NetClientState *nc)
{
    GmFtmac110State *s = qemu_get_nic_opaque(nc);
    uint32_t d[3];

    if ((s->regs[R_MACCR / 4] & (CR_RDMA | CR_RCV)) != (CR_RDMA | CR_RCV)) {
        return false;
    }
    ftmac_desc(s->rx, d, false);
    return d[0] & DES_OWN;
}

static bool ftmac_wanted(GmFtmac110State *s, const uint8_t *buf)
{
    const uint32_t cr = s->regs[R_MACCR / 4];
    uint8_t mac[6];

    if (cr & CR_RCV_ALL) {
        return true;
    }
    if (is_broadcast_ether_addr(buf)) {
        return cr & CR_BROAD;
    }
    if (buf[0] & 1) {
        return cr & (CR_MULTI | CR_HT_MULTI);
    }
    stw_be_p(mac, s->regs[R_MADR / 4]);
    stl_be_p(mac + 2, s->regs[R_LADR / 4]);
    return !memcmp(mac, buf, 6);
}

static ssize_t ftmac_receive(NetClientState *nc, const uint8_t *buf,
                             size_t size)
{
    GmFtmac110State *s = qemu_get_nic_opaque(nc);
    size_t len = MAX(size, 60) + 4, done = 0;   /* with the CRC */
    uint8_t frame[FTMAC_BUF] = { 0 };
    bool first = true;

    if ((s->regs[R_MACCR / 4] & (CR_RDMA | CR_RCV)) != (CR_RDMA | CR_RCV) ||
        size > 1518 || size < ETH_HLEN || !ftmac_wanted(s, buf)) {
        return size;
    }
    memcpy(frame, buf, size);
    while (done < len) {
        uint32_t d[3], n, pad;

        ftmac_desc(s->rx, d, false);
        if (!(d[0] & DES_OWN)) {
            s->regs[R_ISR / 4] |= INT_NORXBUF;
            ftmac_update(s);
            return first ? 0 : size;
        }
        n = d[1] & 0x7FF;
        pad = first && !(d[2] & 2) ? 2 : 0;
        if (n <= pad) {
            return size;
        }
        n = MIN(len - done, n - pad);
        dma_memory_write(&address_space_memory, d[2] + pad, frame + done, n,
                         MEMTXATTRS_UNSPECIFIED);
        done += n;
        d[0] = (first ? 1u << 29 : 0) |
               (done >= len ? (1u << 28) | (uint32_t)len : 0) |
               (is_broadcast_ether_addr(buf) ? 1u << 17 :
                buf[0] & 1 ? 1u << 16 : 0);
        ftmac_desc(s->rx, d, true);
        s->rx = d[1] & DES_END ? s->regs[R_RXBASE / 4] : s->rx + 16;
        first = false;
    }
    s->regs[R_ISR / 4] |= INT_RX;
    ftmac_update(s);
    return size;
}

static void ftmac_phy_reset(GmFtmac110State *s)
{
    memset(s->phy, 0, sizeof(s->phy));
    s->phy[0] = 0x3100;                 /* autonegotiate, 100, full */
    s->phy[1] = 0x782D;                 /* complete, link up */
    s->phy[2] = 0x0181;
    s->phy[3] = 0xB8A0;
    s->phy[4] = 0x01E1;
    s->phy[5] = 0x45E1;
}

static uint64_t ftmac_read(void *opaque, hwaddr off, unsigned size)
{
    GmFtmac110State *s = opaque;
    uint32_t v;

    if (off / 4 >= FTMAC_NREGS) {
        return 0;
    }
    v = s->regs[off / 4];
    if (off == R_ISR) {
        s->regs[R_ISR / 4] = 0;
        ftmac_update(s);
    }
    return v;
}

static void ftmac_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    GmFtmac110State *s = opaque;

    if (off / 4 >= FTMAC_NREGS) {
        return;
    }
    switch (off) {
    case R_ISR:
        return;
    case R_TXBASE:
        s->tx = val;
        break;
    case R_RXBASE:
        s->rx = val;
        break;
    case R_MACCR:
        if (val & CR_RESET) {
            s->tx = s->regs[R_TXBASE / 4];
            s->rx = s->regs[R_RXBASE / 4];
            s->regs[R_ISR / 4] = 0;
            val = 0;
        }
        break;
    case R_PHYCR:
        if (val & PHYCR_READ) {
            /* the one internal PHY, whatever the address */
            uint32_t reg = extract32(val, 21, 5);

            val = (val & 0xFFFF0000u & ~PHYCR_READ) | s->phy[reg];
        } else if (val & PHYCR_WRITE) {
            uint32_t reg = extract32(val, 21, 5);

            if (reg == 0 && (s->regs[R_PHYWDATA / 4] & 0x8000)) {
                ftmac_phy_reset(s);
            } else if (reg == 0 || reg == 4 || reg >= 16) {
                s->phy[reg] = s->regs[R_PHYWDATA / 4] & ~0x0200;
            }
            val &= ~PHYCR_WRITE;
        }
        break;
    }
    s->regs[off / 4] = val;
    if (off == R_TXPD) {
        ftmac_transmit(s);
    } else if (off == R_RXPD || off == R_MACCR) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
    ftmac_update(s);
}

static const MemoryRegionOps ftmac_ops = {
    .read = ftmac_read,
    .write = ftmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static NetClientInfo ftmac_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ftmac_can_receive,
    .receive = ftmac_receive,
};

static void ftmac_reset(DeviceState *dev)
{
    GmFtmac110State *s = GM_FTMAC110(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->tx = s->rx = s->txlen = 0;
    ftmac_phy_reset(s);
    ftmac_update(s);
}

static void ftmac_realize(DeviceState *dev, Error **errp)
{
    GmFtmac110State *s = GM_FTMAC110(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ftmac_ops, s,
                          TYPE_GM_FTMAC110, FTMAC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&ftmac_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static const Property ftmac_props[] = {
    DEFINE_NIC_PROPERTIES(GmFtmac110State, conf),
};

static const VMStateDescription vmstate_ftmac = {
    .name = TYPE_GM_FTMAC110,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmFtmac110State, FTMAC_NREGS),
        VMSTATE_UINT16_ARRAY(phy, GmFtmac110State, 32),
        VMSTATE_UINT32(tx, GmFtmac110State),
        VMSTATE_UINT32(rx, GmFtmac110State),
        VMSTATE_UINT32(txlen, GmFtmac110State),
        VMSTATE_UINT8_ARRAY(txbuf, GmFtmac110State, FTMAC_BUF),
        VMSTATE_END_OF_LIST()
    }
};

static void ftmac_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ftmac_realize;
    device_class_set_legacy_reset(dc, ftmac_reset);
    device_class_set_props(dc, ftmac_props);
    dc->vmsd = &vmstate_ftmac;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo ftmac_info = {
    .name = TYPE_GM_FTMAC110,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GmFtmac110State),
    .class_init = ftmac_class_init,
};

static void ftmac_register_types(void)
{
    type_register_static(&ftmac_info);
}

type_init(ftmac_register_types)
