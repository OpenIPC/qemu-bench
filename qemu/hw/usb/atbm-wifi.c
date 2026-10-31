/*
 * AltoBeam ATBM60xx USB Wi-Fi (Xiongmai's "XM713G"), as the vendor's
 * atbm_wlan driver (xm713g.ko, hal_apollo) drives it.
 *
 * The chip is a softmac with its own LMAC CPU.  The driver talks to it:
 *   - over EP0 vendor requests, wValue / wIndex the low / high half of a
 *     chip address: 0 / 1 read / write registers (4 bytes), 4 / 5 read /
 *     write memory (up to 1 KiB: the firmware download into ICCM and
 *     DCCM), 3 jumps the CPU to the firmware, 6 resets parts of the chip;
 *   - over a bulk OUT / IN pair, WSM messages (the ST-Ericsson CW1200
 *     host interface the LMAC firmware speaks) once the firmware runs.
 *
 * Here no firmware runs: the device answers WSM itself.  It reads the chip
 * version (0x0acc017c) as ARES_B, keeps what the driver writes, and when
 * the driver releases the CPU (0x16101000 bit 8 cleared, or request 3)
 * sends the startup indication.
 *
 * With "psk" the access point is WPA2-PSK (atbm-wpa.c, shared with the
 * SDIO chip): the driver installs the CCMP keys in the chip with
 * WSM_ADD_KEY, so frames between it and the "firmware" stay plaintext.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qemu/cutils.h"
#include "hw/qdev-properties.h"
#include "hw/usb.h"
#include "desc.h"
#include "qom/object.h"
#include "net/net.h"
#include "../net/atbm-wpa.h"

#define TYPE_ATBM_WIFI "atbm-wifi"
OBJECT_DECLARE_SIMPLE_TYPE(AtbmState, ATBM_WIFI)

#define ATBM_VID            0x007a
#define ATBM_PID            0x8888

#define REQ_HW_READ         0
#define REQ_HW_WRITE        1
#define REQ_CPU_JUMP        3
#define REQ_SW_READ         4
#define REQ_SW_WRITE        5
#define REQ_HW_RESET        6
#define REQ_EP0_CMD         7

#define REG_CHIP_VERSION    0x0acc017c
#define CHIP_ARES_B         0x49
#define REG_CPU_CTRL        0x16101000      /* bit 8: CPU held in reset */
/* 0x0acc017c's low byte: 0x49 ARES_B (xm713g.ko: XM713G), 0x4a AresLite
 * (6012B_x.ko, which only carries AresLite firmware: ATBM6012B-X) */
#define CHIP_ARES_LITE      0x4a
#define CPU_HOLD            (1u << 8)

#define EP_IN               1
#define EP_OUT              2

#define PAGE_BITS           12
#define PAGE_SIZE_          (1u << PAGE_BITS)

typedef struct AtbmMsg {
    QSIMPLEQ_ENTRY(AtbmMsg) next;
    size_t len;
    uint8_t data[];
} AtbmMsg;

struct AtbmState {
    USBDevice dev;
    GHashTable *mem;            /* page number -> PAGE_SIZE_ bytes */
    QSIMPLEQ_HEAD(, AtbmMsg) rxq;
    unsigned rxq_len;           /* messages queued: at most RXQ_MAX */
    QEMUTimer *start_timer;
    bool fw_running;
    uint8_t rx_seq;             /* WSM sequence of device -> host messages */
    NICConf conf;
    NICState *nic;
    char *ssid;
    char *psk;                  /* WPA2-PSK passphrase; unset: open */
    AtbmWpa wpa;
    uint8_t sta_if;             /* the driver interface the STA joined on */
    uint8_t channel;
    QEMUTimer *scan_timer;
    uint8_t scan_if;
    bool associated;
    uint8_t sta[6];             /* the camera's address, once associated */
    uint8_t bssid[6];
    uint16_t ap_seq;
    uint16_t pid;
    USBDesc desc;               /* atbm_desc with the product ID */
    uint8_t chip_version;
    bool trace;
};

/* ---- chip memory ------------------------------------------------------- */

static uint8_t *atbm_page(AtbmState *s, uint32_t addr, bool alloc)
{
    gpointer key = GUINT_TO_POINTER(addr >> PAGE_BITS);
    uint8_t *p = g_hash_table_lookup(s->mem, key);

    if (!p && alloc) {
        p = g_malloc0(PAGE_SIZE_);
        g_hash_table_insert(s->mem, key, p);
    }
    return p;
}

static void atbm_mem_write(AtbmState *s, uint32_t addr, const uint8_t *buf,
                           size_t len)
{
    while (len) {
        size_t off = addr & (PAGE_SIZE_ - 1), n = MIN(len, PAGE_SIZE_ - off);

        memcpy(atbm_page(s, addr, true) + off, buf, n);
        addr += n;
        buf += n;
        len -= n;
    }
}

static void atbm_mem_read(AtbmState *s, uint32_t addr, uint8_t *buf,
                          size_t len)
{
    while (len) {
        size_t off = addr & (PAGE_SIZE_ - 1), n = MIN(len, PAGE_SIZE_ - off);
        uint8_t *p = atbm_page(s, addr, false);

        if (p) {
            memcpy(buf, p + off, n);
        } else {
            memset(buf, 0, n);
        }
        addr += n;
        buf += n;
        len -= n;
    }
}

static uint32_t atbm_reg(AtbmState *s, uint32_t addr)
{
    uint8_t b[4];

    atbm_mem_read(s, addr, b, 4);
    return ldl_le_p(b);
}

/* ---- device -> host ---------------------------------------------------- */

/* queue one WSM message: id with the next sequence number in bits 13..15 */
/* what the device holds for a guest that stops reading bulk IN */
#define RXQ_MAX     256

static void atbm_send(AtbmState *s, uint16_t id, const void *body, size_t n)
{
    AtbmMsg *m;

    if (s->rxq_len >= RXQ_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "atbm: bulk IN not read, 0x%04x "
                      "dropped\n", id);
        return;
    }
    m = g_malloc0(sizeof(*m) + 4 + n);

    m->len = 4 + n;
    stw_le_p(m->data, 4 + n);
    stw_le_p(m->data + 2, (id & 0x1fff) | (s->rx_seq << 13));
    s->rx_seq = (s->rx_seq + 1) & 7;
    memcpy(m->data + 4, body, n);
    if (s->trace) {
        qemu_log("atbm: -> 0x%04x len %zu\n", id, m->len);
    }
    QSIMPLEQ_INSERT_TAIL(&s->rxq, m, next);
    s->rxq_len++;
}

#define WSM_STARTUP_IND     0x0801
#define WSM_CNF             0x0400
#define WSM_FIRMWARE_CHECK  0x0002
#define WSM_TX              0x0004
#define WSM_READ_MIB        0x0005
#define WSM_CONFIGURATION   0x0009
#define WSM_START_SCAN      0x0007
#define WSM_STOP_SCAN       0x0008
#define WSM_JOIN            0x000B
#define WSM_RX_IND          0x0804
#define WSM_SCAN_DONE_IND   0x0806
#define WSM_NEED_TX_CONFIRM (1u << 8)   /* in wsm_tx.htTxParameters */

/* struct wsm_tx after the header (the old rate policy: no TxRateRetry):
 * packetID, maxTxRate, queueId, more, flags, reserved, expireTime,
 * htTxParameters, then the 802.11 frame */
#define TX_PACKET_ID        12
#define TX_HT_PARAMS        28
#define TX_FRAME            32

/* firmwareCap: data frames are not confirmed one by one (the driver is
 * built with CONFIG_TX_NO_CONFIRM and refuses an LMAC without it) */
#define CAP_NO_CONFIRM      (1u << 3)

#define MIB_DOT11_STATION_ID    0x0000

/* wsm_startup_indication: the capabilities of the LMAC */
static void atbm_startup(void *opaque)
{
    AtbmState *s = opaque;
    uint8_t b[2 * 10 + 128 + 4 * 4 + 2 * 4] = { 0 };

    stw_le_p(b + 0, 16);        /* numInpChBufs */
    stw_le_p(b + 2, 1632);      /* sizeInpChBuf */
    stw_le_p(b + 4, 0);         /* hardwareId */
    stw_le_p(b + 6, 0);         /* hardwareSubId */
    stw_le_p(b + 8, 0);         /* status: ok */
    stw_le_p(b + 10, CAP_NO_CONFIRM);   /* firmwareCap */
    stw_le_p(b + 12, 2);        /* firmwareType: WSM */
    stw_le_p(b + 14, 0);        /* firmwareApiVer */
    stw_le_p(b + 16, 1);        /* firmwareBuildNumber */
    stw_le_p(b + 18, 1);        /* firmwareVersion */
    pstrcpy((char *)b + 20, 128, "qemu atbm");
    stl_le_p(b + 148, 1 << 16 | 8);     /* Config[0]: interfaces, stations */
    stl_le_p(b + 160, 4);               /* Config[3]: NumOfHwXmitedAddr */
    s->fw_running = true;
    atbm_send(s, WSM_STARTUP_IND, b, sizeof(b));
}

static void atbm_cpu_release(AtbmState *s)
{
    if (s->trace) {
        qemu_log("atbm: CPU released\n");
    }
    s->rx_seq = 0;
    timer_mod(s->start_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 20);
}

/* ---- the access point ---------------------------------------------------- */

#define FC_ASSOC_REQ        0x00
#define FC_ASSOC_RESP       0x10
#define FC_REASSOC_REQ      0x20
#define FC_REASSOC_RESP     0x30
#define FC_PROBE_REQ        0x40
#define FC_PROBE_RESP       0x50
#define FC_BEACON           0x80
#define FC_DISASSOC         0xa0
#define FC_AUTH             0xb0
#define FC_DEAUTH           0xc0
#define FC_DATA             0x08
#define FC_QOS_DATA         0x88

static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static const uint8_t llc_snap[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };

/* a frame from the AP to the driver: wsm_rx, then the 802.11 frame */
static void atbm_rx(AtbmState *s, const uint8_t *f, size_t n, uint32_t flags,
                    uint8_t if_id)
{
    g_autofree uint8_t *b = g_malloc0(16 + n);

    stw_le_p(b + 4, s->channel);    /* channelNumber */
    b[6] = 0;                       /* rxedRate: 1 Mb/s */
    b[7] = (uint8_t)-40;            /* RSSI, dBm */
    stl_le_p(b + 8, flags);
    memcpy(b + 16, f, n);
    atbm_send(s, WSM_RX_IND | if_id << 6, b, 16 + n);
}

/* a management header from the AP: to da, from and with the BSSID */
static size_t ap_hdr(AtbmState *s, uint8_t *f, uint8_t fc, const uint8_t *da)
{
    memset(f, 0, 24);
    f[0] = fc;
    memcpy(f + 4, da, 6);
    memcpy(f + 10, s->bssid, 6);
    memcpy(f + 16, s->bssid, 6);
    stw_le_p(f + 22, s->ap_seq++ << 4);
    return 24;
}

static size_t ap_rates(uint8_t *f)
{
    static const uint8_t r[] = {
        0x01, 8, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,
        0x32, 4, 0x30, 0x48, 0x60, 0x6c,
    };

    memcpy(f, r, sizeof(r));
    return sizeof(r);
}

static uint16_t ap_capability(AtbmState *s)
{
    return 0x0401 | atbm_wpa_capability(&s->wpa);  /* ESS, short slot */
}

/* WPA2-PSK (atbm-wpa.c): the authenticator's frames to the station */
static void wpa_rx(void *opaque, const uint8_t *f, size_t n)
{
    AtbmState *s = opaque;

    atbm_rx(s, f, n, 0, s->sta_if);
}

static void wpa_deauth(void *opaque, uint16_t reason)
{
    AtbmState *s = opaque;
    uint8_t f[32];
    size_t n = ap_hdr(s, f, FC_DEAUTH, s->sta);

    stw_le_p(f + n, reason);
    atbm_rx(s, f, n + 2, 0, s->sta_if);
    s->associated = false;
}

/* a beacon, or a probe response to da */
static void ap_beacon(AtbmState *s, const uint8_t *da, uint8_t if_id)
{
    uint8_t f[256];
    size_t n = ap_hdr(s, f, da ? FC_PROBE_RESP : FC_BEACON, da ? da : bcast);
    const size_t len = MIN(strlen(s->ssid), 32);

    stq_le_p(f + n, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    stw_le_p(f + n + 8, 100);           /* beacon interval, TU */
    stw_le_p(f + n + 10, ap_capability(s));     /* ESS, short slot, privacy */
    n += 12;
    f[n++] = 0;                         /* SSID */
    f[n++] = len;
    memcpy(f + n, s->ssid, len);
    n += len;
    n += ap_rates(f + n);
    f[n++] = 3;                         /* DS parameter set */
    f[n++] = 1;
    f[n++] = s->channel;
    f[n++] = 0x2a;                      /* ERP */
    f[n++] = 1;
    f[n++] = 0;
    n += atbm_wpa_ie(&s->wpa, f + n);
    if (!da) {
        static const uint8_t tim[] = { 5, 4, 0, 1, 0, 0 };

        memcpy(f + n, tim, sizeof(tim));
        n += sizeof(tim);
    }
    atbm_rx(s, f, n, da ? 0 : 1u << 7, if_id);
}

static void ap_auth(AtbmState *s, const uint8_t *sta, uint8_t if_id)
{
    uint8_t f[64];
    size_t n = ap_hdr(s, f, FC_AUTH, sta);

    stw_le_p(f + n, 0);                 /* open system */
    stw_le_p(f + n + 2, 2);             /* transaction 2 */
    stw_le_p(f + n + 4, 0);             /* success */
    atbm_rx(s, f, n + 6, 0, if_id);
}

static void ap_assoc(AtbmState *s, const uint8_t *sta, bool re, uint8_t if_id)
{
    uint8_t f[64];
    size_t n = ap_hdr(s, f, re ? FC_REASSOC_RESP : FC_ASSOC_RESP, sta);

    stw_le_p(f + n, ap_capability(s));  /* capabilities */
    stw_le_p(f + n + 2, 0);             /* success */
    stw_le_p(f + n + 4, 0xc001);        /* AID 1 */
    n += 6;
    n += ap_rates(f + n);
    memcpy(s->sta, sta, 6);
    s->associated = true;
    s->sta_if = if_id;
    if (s->trace) {
        qemu_log("atbm: %02x:%02x:%02x:%02x:%02x:%02x associated\n",
                 sta[0], sta[1], sta[2], sta[3], sta[4], sta[5]);
    }
    atbm_rx(s, f, n, 0, if_id);
    atbm_wpa_associated(&s->wpa, s->bssid, sta);
}

/* the end of a scan: the AP heard on its channel, then the indication */
static void atbm_scan_done(void *opaque)
{
    AtbmState *s = opaque;
    uint8_t b[4 + 2] = { 0 };

    ap_beacon(s, s->conf.macaddr.a, s->scan_if);
    ap_beacon(s, NULL, s->scan_if);
    b[5] = 14;                          /* channels done */
    atbm_send(s, WSM_SCAN_DONE_IND | s->scan_if << 6, b, sizeof(b));
}

/*
 * a data frame to the distribution system: out of the netdev (with WPA2,
 * EAPOL goes to the authenticator and nothing else passes before the
 * handshake ends)
 */
static void ap_data_from_sta(AtbmState *s, const uint8_t *f, size_t n)
{
    const size_t hl = (f[0] == FC_QOS_DATA) ? 26 : 24;
    uint8_t eth[2048];
    size_t p, end;

    if (!s->associated || n < hl || !(f[1] & 1) ||
        atbm_wpa_from_sta(&s->wpa, f, n, hl, &p, &end) != ATBM_WPA_PASS ||
        end - p - 6 + 12 > sizeof(eth)) {
        return;                         /* not ToDS, closed, or no SNAP */
    }
    memcpy(eth, f + 16, 6);             /* DA: addr3 */
    memcpy(eth + 6, f + 10, 6);         /* SA: addr2 */
    memcpy(eth + 12, f + p + 6, end - p - 6);
    qemu_send_packet(qemu_get_queue(s->nic), eth, 12 + end - p - 6);
}

/* a frame the driver transmits */
static void ap_tx(AtbmState *s, const uint8_t *f, size_t n, uint8_t if_id)
{
    if (n < 24) {
        return;
    }
    if (s->trace) {
        qemu_log("atbm: tx fc 0x%02x%02x len %zu\n", f[1], f[0], n);
    }
    if (memcmp(f + 4, s->bssid, 6) && memcmp(f + 4, bcast, 6)) {
        return;                         /* not for the AP */
    }
    switch (f[0]) {
    case FC_PROBE_REQ:
        ap_beacon(s, f + 10, if_id);
        break;
    case FC_AUTH:
        ap_auth(s, f + 10, if_id);
        break;
    case FC_ASSOC_REQ:
    case FC_REASSOC_REQ:
        ap_assoc(s, f + 10, f[0] == FC_REASSOC_REQ, if_id);
        break;
    case FC_DEAUTH:
    case FC_DISASSOC:
        s->associated = false;
        atbm_wpa_reset(&s->wpa);
        break;
    case FC_DATA:
    case FC_QOS_DATA:
        ap_data_from_sta(s, f, n);
        break;
    }
}

/* the netdev's frames, to the camera from the AP */
static bool atbm_can_receive(NetClientState *nc)
{
    AtbmState *s = qemu_get_nic_opaque(nc);

    return s->fw_running;
}

static ssize_t atbm_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    AtbmState *s = qemu_get_nic_opaque(nc);
    /* the 802.11 header, the CCMP header, LLC/SNAP, then the frame from
     * its type on, then the MIC room */
    uint8_t f[24 + 8 + 6 + 2 + 2048 + 8];
    const bool group = buf[0] & 1;
    uint32_t flags = group ? 1u << 9 : 0;
    size_t n = 24, mic;

    if (size < 14) {
        return size;
    }
    if (s->trace) {
        qemu_log("atbm: net rx %zu to %02x:%02x:%02x:%02x:%02x:%02x type "
                 "%02x%02x%s\n", size, buf[0], buf[1], buf[2], buf[3], buf[4],
                 buf[5], buf[12], buf[13], s->associated ? "" : " (dropped)");
    }
    if (!s->associated || 24 + 8 + 8 + size - 12 + 8 > sizeof(f) ||
        (!group && memcmp(buf, s->sta, 6)) || !atbm_wpa_port_open(&s->wpa)) {
        return size;
    }
    memset(f, 0, 24);
    f[0] = FC_DATA;
    f[1] = 0x02;                        /* FromDS */
    memcpy(f + 4, buf, 6);              /* DA */
    memcpy(f + 10, s->bssid, 6);
    memcpy(f + 16, buf + 6, 6);         /* SA */
    stw_le_p(f + 22, s->ap_seq++ << 4);
    mic = atbm_wpa_to_sta(&s->wpa, group, f + n, &flags);
    if (mic) {
        f[1] |= 0x40;                   /* Protected */
        n += mic;
    }
    memcpy(f + n, llc_snap, 6);
    memcpy(f + n + 6, buf + 12, size - 12);
    n += 6 + size - 12;
    if (mic) {
        memset(f + n, 0, ATBM_WPA_CCMP_MIC);
        n += ATBM_WPA_CCMP_MIC;
    }
    atbm_rx(s, f, n, flags, 0);
    return size;
}

static NetClientInfo atbm_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = atbm_can_receive,
    .receive = atbm_receive,
};

/* ---- host -> device ---------------------------------------------------- */

/*
 * One message from the driver: struct wsm_hdr_tx, built with USB_BUS_BUG,
 *   +0 flag, +4 usb_len (1538 at least) / usb_id, +8 len / id,
 * then the body; len counts from +0.  The id's low 6 bits are the
 * command, bits 6..9 the interface, 13..15 the driver's sequence.
 */
#define TX_HDR              12

static void atbm_confirm(AtbmState *s, uint16_t cmd, const void *body,
                         size_t n)
{
    g_autofree uint8_t *b = g_malloc0(4 + n);

    memcpy(b + 4, body, n);     /* status 0: WSM_STATUS_SUCCESS */
    atbm_send(s, WSM_CNF | cmd, b, 4 + n);
}

static void atbm_wsm_out(AtbmState *s, const uint8_t *d, size_t n)
{
    uint16_t len, id, cmd;

    if (n < TX_HDR) {
        return;
    }
    len = MIN(lduw_le_p(d + 8), n);
    id = lduw_le_p(d + 10);
    cmd = id & 0x3f;
    if (s->trace) {
        GString *hex = qemu_hexdump_line(NULL, d + TX_HDR,
                                         MIN(len > TX_HDR ? len - TX_HDR : 0,
                                             32), 4, 0);

        qemu_log("atbm: <- 0x%04x len %u: %s\n", id, len, hex->str);
        g_string_free(hex, true);
    }
    d += TX_HDR;
    len = len > TX_HDR ? len - TX_HDR : 0;

    switch (cmd) {
    case WSM_FIRMWARE_CHECK:
        break;
    case WSM_TX:
        if (len + TX_HDR > TX_FRAME) {
            const uint8_t *w = d - TX_HDR;

            ap_tx(s, w + TX_FRAME, len + TX_HDR - TX_FRAME, (id >> 6) & 0xf);
            if (ldl_le_p(w + TX_HT_PARAMS) & WSM_NEED_TX_CONFIRM) {
                uint8_t b[4 + 4 + 4 + 4 + 4] = { 0 };

                memcpy(b, w + TX_PACKET_ID, 4);
                atbm_send(s, WSM_CNF | WSM_TX, b, sizeof(b));
            }
        }
        break;
    case WSM_ADD_KEY:
        atbm_wpa_add_key(&s->wpa, d, len);
        atbm_confirm(s, cmd, NULL, 0);
        break;
    case WSM_START_SCAN:
        s->scan_if = (id >> 6) & 0xf;
        atbm_confirm(s, cmd, NULL, 0);
        timer_mod(s->scan_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
        break;
    case WSM_JOIN: {
        uint8_t b[8] = { 0 };

        stl_le_p(b + 4, 200);           /* max power level */
        atbm_confirm(s, cmd, b, sizeof(b));
        break;
    }
    case WSM_CONFIGURATION: {
        uint8_t b[6 + 2 + 4 + 2 * 12] = { 0 };

        memcpy(b, s->conf.macaddr.a, 6);
        b[6] = 1;                       /* 2.4 GHz */
        stl_le_p(b + 8, 0x3fff);        /* 1..54 Mb/s, MCS 0..7 */
        for (int i = 0; i < 2; i++) {
            stl_le_p(b + 12 + 12 * i, 0);       /* min, dBm * 10 */
            stl_le_p(b + 16 + 12 * i, 200);     /* max */
            stl_le_p(b + 20 + 12 * i, 10);      /* step */
        }
        atbm_confirm(s, cmd, b, sizeof(b));
        break;
    }
    case WSM_READ_MIB: {
        const uint16_t mib = len >= 2 ? lduw_le_p(d) : 0;
        uint8_t b[4 + 64] = { 0 };

        stw_le_p(b, mib);
        stw_le_p(b + 2, 64);
        if (mib == MIB_DOT11_STATION_ID) {
            memcpy(b + 4, s->conf.macaddr.a, 6);
        }
        atbm_confirm(s, cmd, b, sizeof(b));
        break;
    }
    default:
        atbm_confirm(s, cmd, NULL, 0);
        break;
    }
}

static void atbm_handle_control(USBDevice *dev, USBPacket *p, int request,
                                int value, int index, int length,
                                uint8_t *data)
{
    AtbmState *s = ATBM_WIFI(dev);
    const uint32_t addr = (uint32_t)index << 16 | (value & 0xffff);
    int ret;

    ret = usb_desc_handle_control(dev, p, request, value, index, length,
                                  data);
    if (ret >= 0) {
        return;
    }
    switch (request) {
    case VendorDeviceRequest | REQ_HW_READ:
    case VendorDeviceRequest | REQ_SW_READ:
        atbm_mem_read(s, addr, data, length);
        if (addr == REG_CHIP_VERSION && length >= 4) {
            stl_le_p(data, s->chip_version);
        }
        if (s->trace) {
            qemu_log("atbm: rd %d 0x%08x/%d = 0x%08x\n", request & 0xff,
                     addr, length, length >= 4 ? ldl_le_p(data) : 0);
        }
        p->actual_length = length;
        break;
    case VendorDeviceOutRequest | REQ_HW_WRITE:
    case VendorDeviceOutRequest | REQ_SW_WRITE: {
        const uint32_t old = atbm_reg(s, REG_CPU_CTRL);

        atbm_mem_write(s, addr, data, length);
        if (s->trace && (length <= 8 || (addr & 0xfff) == 0)) {
            qemu_log("atbm: wr %d 0x%08x/%d = 0x%08x\n", request & 0xff,
                     addr, length, length >= 4 ? ldl_le_p(data) : 0);
        }
        if (addr == REG_CPU_CTRL && (old & CPU_HOLD) &&
            !(atbm_reg(s, REG_CPU_CTRL) & CPU_HOLD)) {
            atbm_cpu_release(s);
        }
        break;
    }
    case VendorDeviceOutRequest | REQ_CPU_JUMP:
        atbm_cpu_release(s);
        break;
    case VendorDeviceOutRequest | REQ_HW_RESET:
    case VendorDeviceOutRequest | REQ_EP0_CMD:
        if (s->trace) {
            qemu_log("atbm: req %d value 0x%x index 0x%x len %d\n",
                     request & 0xff, value, index, length);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "atbm: control 0x%04x value 0x%x index "
                      "0x%x len %d\n", request, value, index, length);
        p->status = USB_RET_STALL;
        break;
    }
}

static void atbm_handle_data(USBDevice *dev, USBPacket *p)
{
    AtbmState *s = ATBM_WIFI(dev);

    switch (p->pid) {
    case USB_TOKEN_IN:
        if (p->ep->nr != EP_IN) {
            p->status = USB_RET_STALL;
            return;
        }
        /*
         * NAK until there is a message (the host controller retries).
         * Parking the packet (USB_RET_ASYNC) would be cheaper, but the
         * driver kills its IN URBs on recovery and the DWC2 model reuses
         * a halted channel's packet while it is still in flight.
         */
        if (!QSIMPLEQ_EMPTY(&s->rxq)) {
            AtbmMsg *m = QSIMPLEQ_FIRST(&s->rxq);

            QSIMPLEQ_REMOVE_HEAD(&s->rxq, next);
            s->rxq_len--;
            usb_packet_copy(p, m->data, MIN(m->len, p->iov.size));
            g_free(m);
            return;
        }
        p->status = USB_RET_NAK;
        break;
    case USB_TOKEN_OUT: {
        g_autofree uint8_t *buf = g_malloc(p->iov.size);

        if (p->ep->nr != EP_OUT) {
            p->status = USB_RET_STALL;
            return;
        }
        usb_packet_copy(p, buf, p->iov.size);
        atbm_wsm_out(s, buf, p->iov.size);
        break;
    }
    default:
        p->status = USB_RET_STALL;
        break;
    }
}

static void atbm_flush(AtbmState *s)
{
    AtbmMsg *m;

    while ((m = QSIMPLEQ_FIRST(&s->rxq))) {
        QSIMPLEQ_REMOVE_HEAD(&s->rxq, next);
        g_free(m);
    }
    s->rxq_len = 0;
}

static void atbm_handle_reset(USBDevice *dev)
{
    AtbmState *s = ATBM_WIFI(dev);

    timer_del(s->start_timer);
    timer_del(s->scan_timer);
    atbm_flush(s);
    s->associated = false;
    atbm_wpa_reset(&s->wpa);
    g_hash_table_remove_all(s->mem);
    s->fw_running = false;
    s->rx_seq = 0;
}

/* ---- descriptors ------------------------------------------------------- */

enum { STR_MANUFACTURER = 1, STR_PRODUCT, STR_SERIAL };

static const USBDescStrings atbm_strings = {
    [STR_MANUFACTURER] = "AltoBeam",
    [STR_PRODUCT]      = "ATBM USB Wi-Fi",
    [STR_SERIAL]       = "1",
};

#define ATBM_IFACE(mps) {                                       \
    .bInterfaceNumber   = 0,                                    \
    .bNumEndpoints      = 2,                                    \
    .bInterfaceClass    = USB_CLASS_VENDOR_SPEC,                \
    .bInterfaceSubClass = 0xff,                                 \
    .bInterfaceProtocol = 0xff,                                 \
    .eps = (USBDescEndpoint[]) {                                \
        {                                                       \
            .bEndpointAddress = USB_DIR_IN | EP_IN,             \
            .bmAttributes     = USB_ENDPOINT_XFER_BULK,         \
            .wMaxPacketSize   = mps,                            \
        }, {                                                    \
            .bEndpointAddress = USB_DIR_OUT | EP_OUT,           \
            .bmAttributes     = USB_ENDPOINT_XFER_BULK,         \
            .wMaxPacketSize   = mps,                            \
        },                                                      \
    },                                                          \
}

static const USBDescIface atbm_iface_full = ATBM_IFACE(64);
static const USBDescIface atbm_iface_high = ATBM_IFACE(512);

#define ATBM_DEVICE(iface) {                                    \
    .bcdUSB             = 0x0200,                               \
    .bMaxPacketSize0    = 64,                                   \
    .bNumConfigurations = 1,                                    \
    .confs = (USBDescConfig[]) {                                \
        {                                                       \
            .bNumInterfaces      = 1,                           \
            .bConfigurationValue = 1,                           \
            .bmAttributes        = USB_CFG_ATT_ONE,             \
            .bMaxPower           = 250,                         \
            .nif = 1,                                           \
            .ifs = &(iface),                                    \
        },                                                      \
    },                                                          \
}

static const USBDescDevice atbm_dev_full = ATBM_DEVICE(atbm_iface_full);
static const USBDescDevice atbm_dev_high = ATBM_DEVICE(atbm_iface_high);

static const USBDesc atbm_desc = {
    .id = {
        .idVendor          = ATBM_VID,
        .idProduct         = ATBM_PID,
        .bcdDevice         = 0x0100,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIAL,
    },
    .full = &atbm_dev_full,
    .high = &atbm_dev_high,
    .str  = atbm_strings,
};

static void atbm_realize(USBDevice *dev, Error **errp)
{
    AtbmState *s = ATBM_WIFI(dev);

    if (!s->ssid) {
        s->ssid = g_strdup("OpenIPC");
    }
    s->desc = atbm_desc;
    s->desc.id.idProduct = s->pid;
    dev->usb_desc = &s->desc;
    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    s->mem = g_hash_table_new_full(NULL, NULL, NULL, g_free);
    QSIMPLEQ_INIT(&s->rxq);
    s->start_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_startup, s);
    s->scan_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_scan_done, s);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    /* the AP's BSSID: the NIC's address, locally administered */
    memcpy(s->bssid, s->conf.macaddr.a, 6);
    s->bssid[0] = 0x02;
    s->bssid[5] ^= 0x80;
    if (!atbm_wpa_init(&s->wpa, s->psk, s->ssid, "atbm", s->trace, s,
                       wpa_rx, wpa_deauth, &s->ap_seq, errp)) {
        return;
    }
    s->nic = qemu_new_nic(&atbm_net_info, &s->conf,
                          object_get_typename(OBJECT(s)),
                          DEVICE(s)->id, &DEVICE(s)->mem_reentrancy_guard, s);
}

static void atbm_unrealize(USBDevice *dev)
{
    AtbmState *s = ATBM_WIFI(dev);

    timer_free(s->start_timer);
    timer_free(s->scan_timer);
    atbm_wpa_free(&s->wpa);
    qemu_del_nic(s->nic);
    atbm_flush(s);
    g_hash_table_destroy(s->mem);
}

static const Property atbm_props[] = {
    DEFINE_NIC_PROPERTIES(AtbmState, conf),
    DEFINE_PROP_STRING("ssid", AtbmState, ssid),
    /* a WPA2-PSK passphrase (8..63 characters); unset, the network is open */
    DEFINE_PROP_STRING("psk", AtbmState, psk),
    /* 0x8888: xm713g.ko (XM530V200) and 6132cu.ko (XM535D2);
     * 0x888b with chip-version 0x4a: 6012B_x.ko (XM535W1 / D1) */
    DEFINE_PROP_UINT16("pid", AtbmState, pid, ATBM_PID),
    DEFINE_PROP_UINT8("chip-version", AtbmState, chip_version, CHIP_ARES_B),
    DEFINE_PROP_UINT8("channel", AtbmState, channel, 6),
    DEFINE_PROP_BOOL("trace", AtbmState, trace, false),
};

static bool atbm_get_selftest(Object *obj, Error **errp)
{
    return atbm_wpa_selftest();
}

static void atbm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->realize        = atbm_realize;
    uc->unrealize      = atbm_unrealize;
    uc->product_desc   = "AltoBeam ATBM USB Wi-Fi";
    uc->usb_desc       = &atbm_desc;
    uc->handle_attach  = usb_desc_attach;
    uc->handle_reset   = atbm_handle_reset;
    uc->handle_control = atbm_handle_control;
    uc->handle_data    = atbm_handle_data;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    device_class_set_props(dc, atbm_props);
    object_class_property_add_bool(klass, "selftest", atbm_get_selftest,
                                   NULL);
}

static const TypeInfo atbm_info = {
    .name          = TYPE_ATBM_WIFI,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(AtbmState),
    .class_init    = atbm_class_init,
};

static void atbm_register_types(void)
{
    type_register_static(&atbm_info);
}

type_init(atbm_register_types)
