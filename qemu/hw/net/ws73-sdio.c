/*
 * HiSilicon WS73 SDIO Wi-Fi (vendor 0x12D1, product 0x0073) as XM's
 * battery cameras carry it (G2, RV1103B): the host side is XM's
 * syschannel.ko, HiSilicon's hcc transport (public in the WS73 Linux SDK,
 * driver/platform/hcc/host/hcc_sdio_host.c) under a "syschannel" service.
 * The chip runs its own firmware -- the host downloads nothing -- and
 * reports itself ready ("bsp ready") before the host brings wlan0 up.
 *
 * The card: CCCR, the common and function 1 CIS (MANFID 0x12D1 / 0x0073),
 * function 1 enable and block size; function 1's registers
 * (hcc_sdio_host.c):
 *   0x00 the FIFO (CMD53)
 *   0x08 INT_STATUS (bit 0 data ready, 2 message)  0x09 INT_ENABLE
 *   0x24 WRITE_MSG (host -> chip, a bit a message)
 *   0x30 the extend area, read on every interrupt: int_stat, msg_stat,
 *        xfer_count, credit_info, comm_reg[] (the RX queue, then the
 *        packets' lengths in 32-byte units)
 * XM's syschannel takes the chip's interrupt on a GPIO (Linux GPIO 70,
 * rising), the "wake" output, not on DAT1.
 *
 * Packets both ways are hcc ones: a 4-byte header (service 7:4, sub-type
 * 3:0, the queue, the length with the header) and the payload, 32-byte
 * aligned; the host's writes start with a 64-byte descriptor.  XM's
 * queues: 0 syschannel messages (service 0), 1 wlan0's Ethernet frames
 * (service 1).  Both carry a big-endian length, two bytes, then the data:
 *   - messages hold XM's App protocol (UartControlWlan, "7B len cmd ...
 *     sum"); the chip's firmware would join the AP and run DHCP, so on
 *     the App's first command the model reports the address (cmd 8:
 *     slirp's 10.0.2.15/24, gateway 10.0.2.2, DNS 10.0.2.3) and "DHCP
 *     SUCESS" (cmd 1, 1), which is what the App waits for;
 *   - wlan0's frames go to the netdev and back.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_WS73_SDIO "ws73-sdio"
OBJECT_DECLARE_SIMPLE_TYPE(Ws73State, WS73_SDIO)

#define CMD_GO_IDLE             0
#define CMD_SEND_RELATIVE_ADDR  3
#define CMD_IO_SEND_OP_COND     5
#define CMD_SELECT_CARD         7
#define CMD_SEND_IF_COND        8
#define CMD_IO_RW_DIRECT        52
#define CMD_IO_RW_EXTENDED      53

#define CCCR_SIZE       0x20
#define CCCR_IOEX       0x02
#define CCCR_IENX       0x04
#define CCCR_ABORT      0x06
#define ABORT_RES       0x08
#define FBR1            0x100
#define CIS_COMMON      0x1000
#define CIS_FUNC1       0x1100
#define FN0_WINDOW      0x2000

#define WS73_VID        0x12d1
#define WS73_PID        0x0073

#define F1_FIFO         0x00
#define F1_INT_STATUS   0x08
#define F1_INT_ENABLE   0x09
#define F1_WRITE_MSG    0x24
#define F1_EXTEND       0x30        /* int_stat, msg_stat, xfer_count, credit */
#define F1_SIZE         0x100

#define INT_DREADY      (1u << 0)
#define INT_MFARM       (1u << 2)   /* a message from the chip */

#define D2H_BSP_READY   8           /* syschannel: the chip is up */

#define TX_DESCR        64          /* the host's descriptor before packets */
#define HCC_HDR         4           /* sub:4 service:4, queue, u16 length */
#define RX_ALIGN        32          /* comm_reg lengths are in 32-byte units */
#define RXQ_MAX         32
#define PKT_MAX         1600
#define HCC_USER_MSG    0x01        /* syschannel: a message for the App */
#define HCC_WIFI_DATA   0x10        /* service 1: wlan0's Ethernet frames */

#define XFER_MAX        (64 * 1024)

struct Ws73State {
    DeviceState parent_obj;

    SDBus *sdbus;
    qemu_irq wake;              /* the chip's interrupt pin, rising */
    qemu_irq host_wake;         /* wakes the sleeping SoC (a key line) */
    QEMUTimer *host_wake_timer;
    bool host_asleep;           /* the host suspended syschannel (AOV) */
    int64_t host_wake_at;       /* the last wake pulse, ns */
    bool pir;                   /* the PIR sensor's output (QOM "pir") */
    char cloud[48];             /* the App's cloud server, host:port */
    uint16_t rca;

    uint8_t fn0[FN0_WINDOW];
    uint8_t fn_enable, int_enable, bus_width;
    uint16_t fn1_block_size;
    uint8_t f1[F1_SIZE];

    /* a pending CMD53 data phase */
    bool     io_rw;
    uint8_t  io_fn;
    uint32_t io_addr;
    uint32_t io_left, xfer_pos, xfer_len;
    uint8_t  xfer[XFER_MAX];

    /* chip -> host, read (and cleared) through the extend area */
    uint32_t int_stat, msg_stat, xfer_count, credit;
    uint8_t comm_reg[64];

    /* chip -> host packets, served one at a time through the FIFO */
    uint8_t rxq[RXQ_MAX][PKT_MAX + 64];
    uint32_t rxq_len[RXQ_MAX], rxq_head, rxq_count;
    uint8_t rxq_queue[RXQ_MAX];

    bool trace;
    bool ip_sent;
    uint8_t host_mac[6];        /* what the host's wlan0 sends from */
    uint8_t battery;            /* property: the charge left, % */
    NICConf conf;
    NICState *nic;
};

static void update_irq(Ws73State *s)
{
    bool level = (s->int_enable & 3) == 3 &&
                 (s->f1[F1_INT_STATUS] & s->f1[F1_INT_ENABLE]);

    sdbus_set_sdio_irq(s->sdbus, level);
    /*
     * XM's syschannel takes the chip's interrupt on a GPIO, not on DAT1
     * (g_sdio_intr_mode 1, Linux GPIO 70, rising edge): high while
     * anything is pending, whatever the function's enables say
     */
    qemu_set_irq(s->wake, !!(s->int_stat & (INT_DREADY | INT_MFARM)));
}

/* ---- function 1 -------------------------------------------------------- */

/* a message bit for the host: the message interrupt */
static void d2h_msg(Ws73State *s, int bit)
{
    s->msg_stat |= 1u << bit;
    s->int_stat |= INT_MFARM;
    s->f1[F1_INT_STATUS] |= INT_MFARM;
    if (s->trace) {
        qemu_log("ws73: D2H msg %d\n", bit);
    }
}

/* the packet at the head of the queue, offered to the host */
static void rx_offer(Ws73State *s)
{
    uint32_t len;

    if (!s->rxq_count || (s->int_stat & INT_DREADY)) {
        return;
    }
    len = s->rxq_len[s->rxq_head];
    memset(s->comm_reg, 0, sizeof(s->comm_reg));
    s->comm_reg[0] = s->rxq_queue[s->rxq_head];  /* the RX queue */
    s->comm_reg[1] = len / RX_ALIGN;
    s->xfer_count = len < 512 ? ROUND_UP(len, 4) : ROUND_UP(len, 512);
    s->int_stat |= INT_DREADY;
    s->f1[F1_INT_STATUS] |= INT_DREADY;
    update_irq(s);
}

/* queue an hcc packet: header, then the payload */
static void d2h_packet(Ws73State *s, uint8_t type, const uint8_t *pay,
                       uint32_t n)
{
    uint8_t *b;
    uint32_t i;

    if (s->rxq_count >= RXQ_MAX || n + HCC_HDR > PKT_MAX) {
        return;
    }
    i = (s->rxq_head + s->rxq_count) % RXQ_MAX;
    b = s->rxq[i];
    memset(b, 0, sizeof(s->rxq[i]));
    b[0] = type;
    /* XM's queues: 0 messages (service 0), 1 Wi-Fi data (service 1) */
    b[1] = type >> 4;
    s->rxq_queue[i] = type >> 4;
    stw_le_p(b + 2, ROUND_UP(HCC_HDR + n, RX_ALIGN));   /* header included */
    memcpy(b + HCC_HDR, pay, n);
    s->rxq_len[i] = ROUND_UP(HCC_HDR + n, RX_ALIGN);
    s->rxq_count++;
    rx_offer(s);
}

/*
 * A frame of XM's App protocol (UartControlWlan) for the App: "7B len
 * cmd data... sum", the sum ~XOR of the bytes before it, inside a
 * syschannel message (big-endian length, "Ap", the frame)
 */
static void d2h_frame(Ws73State *s, uint8_t cmd, const uint8_t *data,
                      uint32_t n)
{
    uint8_t m[256 + 4], *f = m + 4, x = 0;
    uint32_t len = n + 4, i;

    if (len > 255) {
        return;
    }
    f[0] = 0x7b;
    f[1] = len;
    f[2] = cmd;
    memcpy(f + 3, data, n);
    for (i = 0; i < len - 1; i++) {
        x ^= f[i];
    }
    f[len - 1] = ~x;
    m[0] = len >> 8;
    m[1] = len;
    m[2] = 0x41;                                /* as the host sends */
    m[3] = 0x70;
    if (s->trace) {
        qemu_log("ws73: D2H frame cmd 0x%02x len %u\n", cmd, len);
    }
    d2h_packet(s, HCC_USER_MSG, m, len + 4);
}

/* a frame from the App (a syschannel message's payload) */
/* pull the sleeping host's wake-up line, at most every 3 s */
static void ws73_wake_host(Ws73State *s, const char *why)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (now - s->host_wake_at <= 3 * NANOSECONDS_PER_SECOND) {
        return;
    }
    if (s->trace) {
        qemu_log("ws73: %s while the host sleeps: wake it\n", why);
    }
    s->host_wake_at = now;
    qemu_set_irq(s->host_wake, 1);
    timer_mod(s->host_wake_timer, now + 200 * SCALE_MS);
}

static void h2d_frame(Ws73State *s, const uint8_t *f, uint32_t n)
{
    if (n < 4 || f[0] != 0x7b || f[1] > n) {
        return;
    }
    if (s->trace) {
        uint32_t i;

        qemu_log("ws73: H2D frame cmd 0x%02x len %u:", f[2], f[1]);
        for (i = 3; i + 1 < f[1]; i++) {
            qemu_log(" %02x", f[i]);
        }
        qemu_log("\n");
    }
    if (!s->ip_sent) {
        /*
         * The App's first command means it is listening: the station is
         * up, as the chip's firmware would report after joining the AP.
         * cmd 9 its MAC (the App sets wlan0's to it), cmd 8 the address
         * (ip, mask, gateway, dns1, dns2: slirp's -nic user defaults),
         * cmd 1, 1 "DHCP SUCESS", which the App's connected state needs.
         */
        static const uint8_t ip[20] = {
            10, 0, 2, 15,  255, 255, 255, 0,  10, 0, 2, 2,
            10, 0, 2, 3,   0, 0, 0, 0,
        };
        static const uint8_t dhcp_ok = 1;

        s->ip_sent = true;
        d2h_frame(s, 0x09, s->conf.macaddr.a, 6);
        d2h_frame(s, 0x08, ip, sizeof(ip));
        d2h_frame(s, 0x01, &dhcp_ok, 1);
        if (s->pir) {
            /* what woke it, if the PIR did: the sensor still sees it */
            static const uint8_t on = 1;

            d2h_frame(s, 0xa3, &on, 1);
        }
    }
    switch (f[2]) {
    case 0x95: {
        /*
         * The charge state, which the chip's MCU side knows: cmd 0xa4,
         * bit 0 a charger, bit 1 full.  On battery, as the App waits
         * half a second for an answer every time it asks.
         */
        static const uint8_t on_battery;

        d2h_frame(s, 0xa4, &on_battery, 1);
        break;
    }
    case 0x01:
        /*
         * The App's cloud server, for the chip to keep a TCP keep-alive
         * with while the SoC sleeps (and to be woken through): a
         * little-endian port, then the address as text (34 bytes)
         */
        if (f[1] >= 4 + 3) {
            char host[35] = "";
            uint32_t m = MIN(f[1] - 4 - 2, sizeof(host) - 1);

            memcpy(host, f + 5, m);
            host[m] = 0;
            snprintf(s->cloud, sizeof(s->cloud), "%s:%u", host,
                     lduw_le_p(f + 3));
            if (s->trace) {
                qemu_log("ws73: cloud server %s\n", s->cloud);
            }
        }
        break;
    case 0x06: {
        /* the PIR's state, which the MCU side reads: cmd 0xa3 */
        uint8_t v = s->pir;

        d2h_frame(s, 0xa3, &v, 1);
        break;
    }
    case 0x13: {
        /*
         * GetMcuPower: the MCU's ADC result, cmd 0x0b ("Get ADC Sample
         * Result From MCU"): 0x31, then the percentage in the third byte
         */
        uint8_t adc[3] = { 0x31, 0, s->battery };

        d2h_frame(s, 0x0b, adc, sizeof(adc));
        break;
    }
    }
}

/*
 * The station's MAC is the chip's: what goes on the air carries it,
 * whatever the host's wlan0 has (XM's syschannel refuses the App's
 * "ifconfig hw ether" while wlan0 is up, so on the RV1103B it stays
 * 00:00:00:00:00:00).  The chip puts its MAC in as the source (and as
 * ARP's sender), and hands frames for it back to the host's MAC.
 */
static void ws73_tx_mac(Ws73State *s, uint8_t *f, uint32_t n)
{
    memcpy(s->host_mac, f + 6, 6);
    memcpy(f + 6, s->conf.macaddr.a, 6);
    if (n >= 14 + 28 && lduw_be_p(f + 12) == 0x0806 &&
        !memcmp(f + 14 + 8, s->host_mac, 6)) {
        memcpy(f + 14 + 8, s->conf.macaddr.a, 6);   /* ARP sender */
    }
}

static void ws73_rx_mac(Ws73State *s, uint8_t *f, uint32_t n)
{
    if (!memcmp(f, s->conf.macaddr.a, 6)) {
        memcpy(f, s->host_mac, 6);
    }
    if (n >= 14 + 28 && lduw_be_p(f + 12) == 0x0806 &&
        !memcmp(f + 14 + 18, s->conf.macaddr.a, 6)) {
        memcpy(f + 14 + 18, s->host_mac, 6);        /* ARP target */
    }
}

/* the netdev's frames: wlan0's receive, while the queue has room */
static bool ws73_can_receive(NetClientState *nc)
{
    Ws73State *s = qemu_get_nic_opaque(nc);

    return s->host_asleep || s->rxq_count < RXQ_MAX;
}

static ssize_t ws73_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    Ws73State *s = qemu_get_nic_opaque(nc);
    uint8_t m[PKT_MAX];

    /*
     * syschannel oopses on a data frame before wlan0 is set up (slirp's
     * router advertisements come early): nothing until the App talks
     */
    if (s->host_asleep) {
        /*
         * The SoC sleeps (XM's AOV) and the chip keeps the link: a
         * connection to the station wakes it, as the chip pulls the
         * host's wake-up key line; the frame itself is dropped, the
         * client's retransmission finds the host back up.  At most a
         * pulse every 3 s.
         */
        bool syn = size >= 14 + 20 + 20 && lduw_be_p(buf + 12) == 0x0800 &&
                   buf[14 + 9] == 6 &&
                   (buf[14 + ((buf[14] & 15) * 4) + 13] & 0x12) == 0x02;

        if (syn) {
            ws73_wake_host(s, "a connection");
        }
        return size;
    }
    if (!s->ip_sent || size + 4 + HCC_HDR > PKT_MAX) {
        return size;
    }
    if (s->rxq_count >= RXQ_MAX) {
        return 0;
    }
    stw_be_p(m, size);
    m[2] = m[3] = 0;
    memcpy(m + 4, buf, size);
    ws73_rx_mac(s, m + 4, size);
    d2h_packet(s, HCC_WIFI_DATA, m, size + 4);
    return size;
}

static NetClientInfo ws73_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ws73_can_receive,
    .receive = ws73_receive,
};

/* the host's FIFO write: a descriptor, then hcc packets */
static void h2d_packets(Ws73State *s, uint8_t *b, uint32_t len)
{
    uint32_t off = TX_DESCR;

    while (off + HCC_HDR <= len) {
        uint32_t tot = lduw_le_p(b + off + 2), pay;

        if (tot <= HCC_HDR || off + tot > len) {
            if (s->trace && tot) {
                qemu_log("ws73: H2D stop: tot %u at %u of %u\n", tot, off, len);
            }
            break;
        }
        pay = tot - HCC_HDR;
        if (b[off] == HCC_USER_MSG && pay > 4) {
            uint32_t mlen = lduw_be_p(b + off + HCC_HDR);

            h2d_frame(s, b + off + HCC_HDR + 4, MIN(mlen, pay - 4));
        } else if (b[off] == HCC_WIFI_DATA && pay > 4) {
            /* big-endian length, two bytes, the frame */
            uint32_t flen = lduw_be_p(b + off + HCC_HDR);

            if (s->trace) {
                qemu_log("ws73: H2D data tot %u flen %u at %u of %u\n",
                         tot, flen, off, len);
            }
            if (flen >= 14 && flen <= pay - 4) {
                ws73_tx_mac(s, b + off + HCC_HDR + 4, flen);
                qemu_send_packet(qemu_get_queue(s->nic),
                                 b + off + HCC_HDR + 4, flen);
            }
        } else if (s->trace) {
            uint32_t j;

            qemu_log("ws73: H2D packet type 0x%02x len %u:", b[off], pay);
            for (j = 0; j < MIN(pay, 96); j++) {
                qemu_log(" %02x", b[off + HCC_HDR + j]);
            }
            qemu_log("\n");
        }
        off += ROUND_UP(tot, RX_ALIGN);
    }
}

/*
 * A message from the host (WRITE_MSG, a bit a message).  XM's syschannel
 * sends 14 when it waits for the chip (syschannel_host_wait_ready, which
 * takes BSP ready, 8) and 16 before the SoC sleeps (wait_suspend_ack:
 * 15); an App that gets no ack thinks the channel stuck and reboots.
 */
#define H2D_WAIT_READY  14
#define H2D_SUSPEND     16
#define D2H_SUSPEND_ACK 15

static void h2d_msg(Ws73State *s, uint32_t bits)
{
    if (s->trace) {
        qemu_log("ws73: H2D msg 0x%08x\n", bits);
    }
    if (bits & (1u << H2D_WAIT_READY)) {
        /*
         * A host (re)starting: after XM's AOV the App binds syschannel
         * again and waits for the station's state anew
         */
        s->ip_sent = false;
        s->host_asleep = false;
        s->rxq_head = s->rxq_count = 0;
        d2h_msg(s, D2H_BSP_READY);
    }
    if (bits & (1u << H2D_SUSPEND)) {
        d2h_msg(s, D2H_SUSPEND_ACK);
        s->host_asleep = true;
    }
}

static uint8_t f1_read(Ws73State *s, uint32_t addr)
{
    uint8_t v = addr < F1_SIZE ? s->f1[addr] : 0;

    if (s->trace) {
        qemu_log("ws73: F1 rd 0x%02x = 0x%02x\n", addr, v);
    }
    return v;
}

static void f1_write(Ws73State *s, uint32_t addr, uint8_t v)
{
    if (s->trace) {
        qemu_log("ws73: F1 wr 0x%02x = 0x%02x\n", addr, v);
    }
    if (addr == F1_INT_STATUS) {
        s->f1[addr] &= ~v;                  /* write 1 to clear */
    } else if (addr < F1_SIZE) {
        s->f1[addr] = v;
    }
    update_irq(s);
}

static void f1_read_begin(Ws73State *s, uint32_t len)
{
    uint32_t i;

    if (s->trace) {
        qemu_log("ws73: F1 CMD53 rd 0x%02x len %u\n", s->io_addr, len);
    }
    memset(s->xfer, 0, len);
    if (s->io_addr == F1_EXTEND && len >= 16) {
        /* struct hcc_sdio_extend_func; reading it takes the events */
        stl_le_p(s->xfer, s->int_stat);
        stl_le_p(s->xfer + 4, s->msg_stat);
        stl_le_p(s->xfer + 8, s->xfer_count);
        stl_le_p(s->xfer + 12, s->credit);
        memcpy(s->xfer + 16, s->comm_reg, MIN(len - 16, sizeof(s->comm_reg)));
        s->int_stat &= ~INT_MFARM;
        s->msg_stat = 0;
        s->f1[F1_INT_STATUS] &= ~INT_MFARM;
        update_irq(s);
        return;
    }
    if (s->io_addr == F1_FIFO && (s->int_stat & INT_DREADY)) {
        /* the offered packet; reading it takes it off the queue */
        memcpy(s->xfer, s->rxq[s->rxq_head],
               MIN(len, s->rxq_len[s->rxq_head]));
        s->rxq_head = (s->rxq_head + 1) % RXQ_MAX;
        s->rxq_count--;
        s->int_stat &= ~INT_DREADY;
        s->f1[F1_INT_STATUS] &= ~INT_DREADY;
        s->xfer_count = 0;
        memset(s->comm_reg, 0, sizeof(s->comm_reg));
        update_irq(s);
        rx_offer(s);
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        return;
    }
    if (s->io_addr != F1_FIFO) {
        for (i = 0; i < len && s->io_addr + i < F1_SIZE; i++) {
            s->xfer[i] = s->f1[s->io_addr + i];
        }
    }
}

static void f1_write_end(Ws73State *s)
{
    uint32_t i;

    if (s->trace) {
        qemu_log("ws73: F1 CMD53 wr 0x%02x len %u:", s->io_addr, s->xfer_len);
        i = s->io_addr == F1_FIFO && s->xfer_len > TX_DESCR ? TX_DESCR : 0;
        for (; i < MIN(s->xfer_len, 192); i++) {
            qemu_log(" %02x", s->xfer[i]);
        }
        qemu_log("\n");
    }
    if (s->io_addr != F1_FIFO) {
        for (i = 0; i < s->xfer_len && s->io_addr + i < F1_SIZE; i++) {
            f1_write(s, s->io_addr + i, s->xfer[i]);
        }
    } else {
        h2d_packets(s, s->xfer, s->xfer_len);
    }
    if (s->io_addr == F1_WRITE_MSG && s->xfer_len >= 4) {
        h2d_msg(s, ldl_le_p(s->xfer));
        update_irq(s);
    }
}

/* ---- SDIO card --------------------------------------------------------- */

static void build_cis(Ws73State *s)
{
    uint8_t *c = s->fn0 + CIS_COMMON;
    int i = 0;

    c[i++] = 0x20; c[i++] = 4;                  /* CISTPL_MANFID */
    c[i++] = WS73_VID & 0xff; c[i++] = WS73_VID >> 8;
    c[i++] = WS73_PID & 0xff; c[i++] = WS73_PID >> 8;
    c[i++] = 0x22; c[i++] = 4;                  /* CISTPL_FUNCE, common */
    c[i++] = 0x00;
    c[i++] = 0x00; c[i++] = 0x02;               /* fn0 max block 512 */
    c[i++] = 0x32;                              /* 25 MHz */
    c[i++] = 0xff;

    c = s->fn0 + CIS_FUNC1;
    memset(c, 0, 0x40);
    c[0] = 0x22; c[1] = 0x2a;                   /* CISTPL_FUNCE, function */
    c[2] = 0x01;
    c[2 + 12] = 0x00; c[2 + 13] = 0x02;         /* max block 512 */
    c[2 + 0x2a] = 0xff;
}

static void cccr_reset(Ws73State *s)
{
    memset(s->fn0, 0, sizeof(s->fn0));
    s->fn0[0x00] = 0x43;
    s->fn0[0x08] = 0xc2;                        /* 4BLS, LSC, SMB */
    s->fn0[0x09] = CIS_COMMON & 0xff;
    s->fn0[0x0a] = (CIS_COMMON >> 8) & 0xff;
    s->fn0[0x0b] = (CIS_COMMON >> 16) & 0xff;
    s->fn0[FBR1 + 0x00] = 0x07;                 /* class: WLAN */
    s->fn0[FBR1 + 0x09] = CIS_FUNC1 & 0xff;
    s->fn0[FBR1 + 0x0a] = (CIS_FUNC1 >> 8) & 0xff;
    s->fn0[FBR1 + 0x0b] = (CIS_FUNC1 >> 16) & 0xff;
    s->fn_enable = s->int_enable = s->bus_width = 0;
    s->fn1_block_size = 0;
    memset(s->f1, 0, sizeof(s->f1));
    s->int_stat = s->msg_stat = s->xfer_count = s->credit = 0;
    memset(s->comm_reg, 0, sizeof(s->comm_reg));
    s->rxq_head = s->rxq_count = 0;
    build_cis(s);
}

static uint8_t fn0_read(Ws73State *s, uint32_t addr)
{
    switch (addr) {
    case CCCR_IOEX: return s->fn_enable;
    case 0x03: return s->fn_enable;             /* IORx: ready at once */
    case CCCR_IENX: return s->int_enable;
    case 0x05:                                  /* INTx: F1 pending */
        return (s->f1[F1_INT_STATUS] & s->f1[F1_INT_ENABLE]) ? 2 : 0;
    case 0x07: return s->bus_width;
    case 0x110: return s->fn1_block_size & 0xff;
    case 0x111: return s->fn1_block_size >> 8;
    }
    return addr < FN0_WINDOW ? s->fn0[addr] : 0;
}

static void fn0_write(Ws73State *s, uint32_t addr, uint8_t v)
{
    switch (addr) {
    case CCCR_IOEX:
        if (s->trace && ((s->fn_enable ^ v) & 2)) {
            qemu_log("ws73: F1 %s\n", v & 2 ? "enabled" : "disabled");
        }
        s->fn_enable = v;
        return;
    case CCCR_IENX:
        s->int_enable = v;
        update_irq(s);
        return;
    case CCCR_ABORT:
        if (v & ABORT_RES) {
            cccr_reset(s);
        }
        return;
    case 0x07: s->bus_width = v; return;
    case 0x110: s->fn1_block_size = (s->fn1_block_size & 0xff00) | v; return;
    case 0x111: s->fn1_block_size = (s->fn1_block_size & 0xff) | v << 8; return;
    }
    if (s->trace && addr >= CCCR_SIZE && addr < 0x100) {
        qemu_log("ws73: CCCR wr 0x%02x = 0x%02x\n", addr, v);
    }
    if (addr >= CCCR_SIZE && addr < FN0_WINDOW) {
        s->fn0[addr] = v;
    }
}

static size_t resp32(uint8_t *resp, uint32_t v)
{
    resp[0] = v >> 24; resp[1] = v >> 16; resp[2] = v >> 8; resp[3] = v;
    return 4;
}

static size_t do_command(SDState *sd, SDRequest *req, uint8_t *resp,
                         size_t respsz)
{
    Ws73State *s = WS73_SDIO(sd);
    uint32_t arg = req->arg;

    s->io_left = 0;
    switch (req->cmd) {
    case CMD_GO_IDLE:
    case CMD_SEND_IF_COND:
        return 0;
    case CMD_IO_SEND_OP_COND:
        return resp32(resp, (arg ? 0x80000000 : 0) | (1u << 28) | 0x00ff8000);
    case CMD_SEND_RELATIVE_ADDR:
        s->rca = 1;
        return resp32(resp, (uint32_t)s->rca << 16);
    case CMD_SELECT_CARD:
        return resp32(resp, 0x1e00);
    case CMD_IO_RW_DIRECT: {
        bool write = arg & (1u << 31);
        uint8_t fn = (arg >> 28) & 7, data = arg & 0xff, out = 0;
        uint32_t addr = (arg >> 9) & 0x1ffff;

        if (fn == 0) {
            if (write) {
                fn0_write(s, addr, data);
            }
            out = fn0_read(s, addr);
        } else if (fn == 1) {
            if (write) {
                f1_write(s, addr, data);
            }
            out = f1_read(s, addr);
        }
        return resp32(resp, (0x10 << 8) | out);
    }
    case CMD_IO_RW_EXTENDED: {
        bool block = arg & (1u << 27);
        uint32_t count = arg & 0x1ff, bytes;

        s->io_rw = arg & (1u << 31);
        s->io_fn = (arg >> 28) & 7;
        s->io_addr = (arg >> 9) & 0x1ffff;
        bytes = block ? (count ? count : 512) * MAX(s->fn1_block_size, 1)
                      : (count ? count : 512);
        if (bytes > XFER_MAX) {
            bytes = XFER_MAX;
        }
        s->io_left = bytes;
        s->xfer_pos = 0;
        s->xfer_len = bytes;
        if (!s->io_rw) {
            if (s->io_fn == 1) {
                f1_read_begin(s, bytes);
            } else {
                uint32_t i;

                for (i = 0; i < bytes; i++) {
                    s->xfer[i] = fn0_read(s, s->io_addr + i);
                }
            }
        }
        return resp32(resp, 0x10 << 8);
    }
    default:
        qemu_log_mask(LOG_UNIMP, "ws73: CMD%d arg 0x%08x\n", req->cmd, arg);
        return 0;
    }
}

static void write_byte(SDState *sd, uint8_t value)
{
    Ws73State *s = WS73_SDIO(sd);

    if (!s->io_left || !s->io_rw) {
        return;
    }
    if (s->xfer_pos < XFER_MAX) {
        s->xfer[s->xfer_pos++] = value;
    }
    if (!--s->io_left) {
        if (s->io_fn == 1) {
            f1_write_end(s);
        } else {
            uint32_t i;

            for (i = 0; i < s->xfer_len; i++) {
                fn0_write(s, s->io_addr + i, s->xfer[i]);
            }
        }
    }
}

static uint8_t read_byte(SDState *sd)
{
    Ws73State *s = WS73_SDIO(sd);
    uint8_t v = 0;

    if (!s->io_left || s->io_rw) {
        return 0;
    }
    if (s->xfer_pos < XFER_MAX) {
        v = s->xfer[s->xfer_pos++];
    }
    s->io_left--;
    return v;
}

static bool data_ready(SDState *sd)
{
    return WS73_SDIO(sd)->io_left > 0;
}

static void set_voltage(SDState *sd, uint16_t millivolts)
{
}

static bool get_inserted(SDState *sd)
{
    return true;
}

static bool get_readonly(SDState *sd)
{
    return false;
}

static void ws73_host_wake_end(void *opaque)
{
    Ws73State *s = opaque;

    qemu_set_irq(s->host_wake, 0);
}

/*
 * QOM "pir": the PIR sensor, which hangs off the chip's MCU side.  A
 * rising edge wakes a sleeping host (XM's AOV) or, awake, is reported
 * at once (cmd 0xa3, 1); the falling edge reports 0.
 */
static bool ws73_get_pir(Object *obj, Error **errp)
{
    return WS73_SDIO(obj)->pir;
}

static void ws73_set_pir(Object *obj, bool v, Error **errp)
{
    Ws73State *s = WS73_SDIO(obj);
    uint8_t d = v;

    if (v == s->pir) {
        return;
    }
    s->pir = v;
    if (s->trace) {
        qemu_log("ws73: PIR %s\n", v ? "on" : "off");
    }
    if (s->host_asleep) {
        if (v) {
            ws73_wake_host(s, "the PIR");
        }
    } else if (s->ip_sent) {
        d2h_frame(s, 0xa3, &d, 1);
    }
}

/*
 * QOM "cloud-server" (read only): what the App asked the chip to keep
 * alive.  "cloud-wake": the server's wake-up through that connection --
 * the chip keeps no real TCP session here, so this stands in for it: a
 * sleeping host is woken as for a local connection.
 */
static char *ws73_get_cloud(Object *obj, Error **errp)
{
    return g_strdup(WS73_SDIO(obj)->cloud);
}

static bool ws73_get_false(Object *obj, Error **errp)
{
    return false;
}

static void ws73_set_cloud_wake(Object *obj, bool v, Error **errp)
{
    Ws73State *s = WS73_SDIO(obj);

    if (v && s->host_asleep) {
        ws73_wake_host(s, "the cloud");
    }
}

static void ws73_realize(DeviceState *dev, Error **errp)
{
    Ws73State *s = WS73_SDIO(dev);

    s->sdbus = SD_BUS(qdev_get_parent_bus(dev));
    qdev_init_gpio_out_named(dev, &s->wake, "wake", 1);
    qdev_init_gpio_out_named(dev, &s->host_wake, "host-wake", 1);
    object_property_add_bool(OBJECT(dev), "pir", ws73_get_pir, ws73_set_pir);
    object_property_add_str(OBJECT(dev), "cloud-server", ws73_get_cloud, NULL);
    object_property_add_bool(OBJECT(dev), "cloud-wake", ws73_get_false,
                             ws73_set_cloud_wake);
    s->host_wake_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      ws73_host_wake_end, s);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&ws73_net_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    cccr_reset(s);
}

static void ws73_reset(DeviceState *dev)
{
    Ws73State *s = WS73_SDIO(dev);

    s->rca = 0;
    s->io_left = 0;
    s->ip_sent = false;
    s->host_asleep = false;
    s->cloud[0] = 0;
    s->rxq_head = s->rxq_count = 0;
    cccr_reset(s);
}

static const Property ws73_props[] = {
    DEFINE_PROP_BOOL("trace", Ws73State, trace, false),
    DEFINE_PROP_UINT8("battery", Ws73State, battery, 90),
    DEFINE_NIC_PROPERTIES(Ws73State, conf),
};

static void ws73_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = (SDCardClass *)klass;

    dc->realize = ws73_realize;
    device_class_set_legacy_reset(dc, ws73_reset);
    device_class_set_props(dc, ws73_props);
    dc->bus_type = TYPE_SD_BUS;
    dc->user_creatable = true;
    sc->do_command = do_command;
    sc->write_byte = write_byte;
    sc->read_byte = read_byte;
    sc->data_ready = data_ready;
    sc->receive_ready = data_ready;
    sc->set_voltage = set_voltage;
    sc->get_inserted = get_inserted;
    sc->get_readonly = get_readonly;
}

static const TypeInfo ws73_info = {
    .name          = TYPE_WS73_SDIO,
    .parent        = TYPE_SD_BUS_CARD,
    .instance_size = sizeof(Ws73State),
    .class_size    = sizeof(SDCardClass),
    .class_init    = ws73_class_init,
};

static void ws73_register_types(void)
{
    type_register_static(&ws73_info);
}

type_init(ws73_register_types)
