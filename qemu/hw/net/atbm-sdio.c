/*
 * AltoBeam ATBM6031/Apollo SDIO Wi-Fi, as the vendor's atbm_wlan driver
 * (hal_apollo, PLATFORM_INGENICT31/T41) drives it on the Ingenic MSC.
 *
 * The chip is an SDIO function-1 device with its own LMAC CPU.  The driver
 *   - enumerates it over the SD bus (CMD5/CMD3/CMD7, then CCCR/FBR/CIS and
 *     CMD52/CMD53 to function 1);
 *   - reaches the HIF registers through function-1 CMD53, by a 17-bit
 *     address (buf_id << 6 | reg << 2): CONFIG(0), CONTROL(1), the in/out
 *     queue(2), the AHB and SRAM data ports(3,5) with their shared base
 *     address register(4);
 *   - wakes the chip (CONTROL wakeup -> ready), downloads firmware into
 *     the chip's ITCM/DTCM over the AHB port, releases the CPU (CONFIG CPU
 *     reset cleared), and then speaks WSM (the ST-Ericsson CW1200 host
 *     interface) over the queue, the chip raising the SDIO interrupt when
 *     it has a message.
 *
 * Here no firmware runs: the device answers WSM itself and emulates an
 * access point, bridging data frames to a QEMU netdev.  It is the SDIO
 * sibling of hw/usb/atbm-wifi.c and shares its WSM / AP engine.
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
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qom/object.h"
#include "atbm-wpa.h"

#define TYPE_ATBM_SDIO "atbm-sdio"
OBJECT_DECLARE_TYPE(AtbmSdioState, AtbmSdioClass, ATBM_SDIO)

struct AtbmSdioClass {
    SDCardClass parent_class;
};

/* SDIO command set */
#define CMD_GO_IDLE             0
#define CMD_SEND_RELATIVE_ADDR  3
#define CMD_IO_SEND_OP_COND     5
#define CMD_SELECT_CARD         7
#define CMD_SEND_IF_COND        8
#define CMD_IO_RW_DIRECT        52
#define CMD_IO_RW_EXTENDED      53

/* CCCR (function 0) */
#define CCCR_SIZE       0x20
#define FBR1            0x100
#define CIS_COMMON      0x1000
#define CIS_FUNC1       0x1100
#define FN0_WINDOW      0x2000          /* CCCR + FBR + CIS storage */

#define ATBM_VID        0x007a
#define ATBM_PID        0x6011

/* HIF registers (reg = (addr >> 2) & 7) */
#define HIF_CONFIG      0
#define HIF_CONTROL     1
#define HIF_QUEUE       2
#define HIF_AHB_DPORT   3
#define HIF_SRAM_BASE   4
#define HIF_SRAM_DPORT  5

#define CONFIG_CPU_RESET    (1u << 14)
#define CONFIG_AHB_PFETCH   (1u << 11)  /* indirect prefetch, self-clearing */
#define CONFIG_PREFETCH     (1u << 13)

#define CONTROL_RDY     (1u << 13)
#define CONTROL_LEN_LSB 0x0fff          /* next message length, in words */

/* special chip addresses the driver reads during bring-up */
#define REG_CHIP_VERSION    0x0acc017c
#define CHIP_ARES_B         0x49
#define REG_STATUS_5        0x16101028  /* before_load polls for 5 */
#define REG_EFUSE_DATA      0x16b00004
#define REG_HW_XMITED       0x09000000  /* Config[3]: TX frames consumed */

#define XFER_MAX        (16 * 1024)
#define RXQ_MAX         256
#define PAGE_BITS       12
#define PAGE_SIZE_      (1u << PAGE_BITS)

enum atbm_sd_state { ATBM_IDLE, ATBM_READY, ATBM_STANDBY, ATBM_TRANSFER };

typedef struct AtbmMsg {
    QSIMPLEQ_ENTRY(AtbmMsg) next;
    size_t len;
    uint8_t data[];
} AtbmMsg;

struct AtbmSdioState {
    DeviceState parent_obj;

    SDBus *sdbus;
    enum atbm_sd_state state;
    uint16_t rca;

    uint8_t fn0[FN0_WINDOW];
    uint8_t fn_enable, int_enable, bus_width;
    uint16_t fn1_block_size;

    /* a pending CMD53 data phase */
    bool     io_rw;             /* true: write */
    uint8_t  io_fn, io_reg;
    uint32_t io_left, xfer_pos, xfer_len;
    uint8_t  xfer[XFER_MAX];

    /* HIF */
    uint32_t config;
    uint32_t sram_base;         /* AHB/SRAM data-port address latch */
    GHashTable *mem;            /* chip memory: page -> bytes */
    uint32_t tx_consumed;       /* TX frames the "firmware" has taken */

    /* device -> host message queue */
    QSIMPLEQ_HEAD(, AtbmMsg) rxq;
    unsigned rxq_len;
    uint8_t rx_seq;

    /* firmware / AP */
    QEMUTimer *start_timer, *scan_timer;
    bool fw_running;
    NICConf conf;
    NICState *nic;
    char *ssid;
    uint8_t channel, scan_if;
    bool associated;
    uint8_t sta[6], bssid[6];
    uint16_t ap_seq;
    uint8_t sta_if;             /* the driver interface the STA joined on */

    /* WPA2-PSK (with "psk"): the access point's authenticator */
    char *psk;
    AtbmWpa wpa;
    bool trace;
};

/* ---- chip memory ------------------------------------------------------- */

static uint8_t *atbm_page(AtbmSdioState *s, uint32_t addr, bool alloc)
{
    gpointer key = GUINT_TO_POINTER(addr >> PAGE_BITS);
    uint8_t *p = g_hash_table_lookup(s->mem, key);

    if (!p && alloc) {
        p = g_malloc0(PAGE_SIZE_);
        g_hash_table_insert(s->mem, key, p);
    }
    return p;
}

static void atbm_mem_write(AtbmSdioState *s, uint32_t addr, const uint8_t *b,
                           size_t n)
{
    while (n) {
        size_t off = addr & (PAGE_SIZE_ - 1), k = MIN(n, PAGE_SIZE_ - off);

        memcpy(atbm_page(s, addr, true) + off, b, k);
        addr += k; b += k; n -= k;
    }
}

static void atbm_mem_read(AtbmSdioState *s, uint32_t addr, uint8_t *b,
                          size_t n)
{
    /* the registers the driver polls during bring-up */
    if (n == 4) {
        uint32_t v;

        switch (addr) {
        case REG_CHIP_VERSION: v = CHIP_ARES_B; break;
        case REG_STATUS_5:     v = 5; break;
        case REG_EFUSE_DATA:   v = 0; break;
        case REG_HW_XMITED:    v = s->tx_consumed; break;
        default: goto mem;
        }
        stl_le_p(b, v);
        return;
    }
mem:
    while (n) {
        size_t off = addr & (PAGE_SIZE_ - 1), k = MIN(n, PAGE_SIZE_ - off);
        uint8_t *p = atbm_page(s, addr, false);

        if (p) {
            memcpy(b, p + off, k);
        } else {
            memset(b, 0, k);
        }
        addr += k; b += k; n -= k;
    }
}

/* ---- device -> host ---------------------------------------------------- */

static void atbm_irq(AtbmSdioState *s)
{
    sdbus_set_sdio_irq(s->sdbus, !QSIMPLEQ_EMPTY(&s->rxq));
}

static void atbm_send(AtbmSdioState *s, uint16_t id, const void *body,
                      size_t n)
{
    AtbmMsg *m;

    if (s->rxq_len >= RXQ_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "atbm-sdio: queue full, 0x%04x "
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
        qemu_log("atbm-sdio: -> 0x%04x len %zu\n", id, m->len);
    }
    QSIMPLEQ_INSERT_TAIL(&s->rxq, m, next);
    s->rxq_len++;
    atbm_irq(s);
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
#define WSM_NEED_TX_CONFIRM (1u << 8)

/* wsm_tx offsets from the { len, id } header */
#define TX_PACKET_ID        4
#define TX_HT_PARAMS        24
#define TX_FRAME            28
#define CAP_NO_CONFIRM      (1u << 3)
#define MIB_DOT11_STATION_ID    0x0000

static void atbm_startup(void *opaque)
{
    AtbmSdioState *s = opaque;
    uint8_t b[2 * 10 + 128 + 4 * 4 + 2 * 4] = { 0 };

    stw_le_p(b + 0, 16);        /* numInpChBufs */
    stw_le_p(b + 2, 1632);      /* sizeInpChBuf */
    stw_le_p(b + 10, CAP_NO_CONFIRM);   /* firmwareCap */
    stw_le_p(b + 12, 2);        /* firmwareType: WSM */
    stw_le_p(b + 16, 1);        /* firmwareBuildNumber */
    stw_le_p(b + 18, 1);        /* firmwareVersion */
    pstrcpy((char *)b + 20, 128, "qemu atbm");
    stl_le_p(b + 148, 1 << 16 | 8);     /* Config[0]: interfaces, stations */
    stl_le_p(b + 160, REG_HW_XMITED);   /* Config[3]: NumOfHwXmitedAddr */
    s->fw_running = true;
    atbm_send(s, WSM_STARTUP_IND, b, sizeof(b));
}

static void atbm_cpu_release(AtbmSdioState *s)
{
    if (s->trace) {
        qemu_log("atbm-sdio: CPU released\n");
    }
    s->rx_seq = 0;
    timer_mod(s->start_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 20);
}

/* ---- the access point -------------------------------------------------- */

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

static void atbm_rx(AtbmSdioState *s, const uint8_t *f, size_t n,
                    uint32_t flags, uint8_t if_id)
{
    g_autofree uint8_t *b = g_malloc0(16 + n);

    stw_le_p(b + 4, s->channel);
    b[6] = 0;
    b[7] = (uint8_t)-40;
    stl_le_p(b + 8, flags);
    memcpy(b + 16, f, n);
    atbm_send(s, WSM_RX_IND | if_id << 6, b, 16 + n);
}

static size_t ap_hdr(AtbmSdioState *s, uint8_t *f, uint8_t fc,
                     const uint8_t *da)
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

/* WPA2-PSK (atbm-wpa.c): the authenticator's frames to the station */
static void wpa_rx(void *opaque, const uint8_t *f, size_t n)
{
    AtbmSdioState *s = opaque;

    atbm_rx(s, f, n, 0, s->sta_if);
}

static void wpa_deauth(void *opaque, uint16_t reason)
{
    AtbmSdioState *s = opaque;
    uint8_t f[32];
    size_t n = ap_hdr(s, f, FC_DEAUTH, s->sta);

    stw_le_p(f + n, reason);
    atbm_rx(s, f, n + 2, 0, s->sta_if);
    s->associated = false;
}

static uint16_t ap_capability(AtbmSdioState *s)
{
    return 0x0401 | atbm_wpa_capability(&s->wpa);  /* ESS, short slot */
}

static void ap_beacon(AtbmSdioState *s, const uint8_t *da, uint8_t if_id)
{
    uint8_t f[256];
    size_t n = ap_hdr(s, f, da ? FC_PROBE_RESP : FC_BEACON, da ? da : bcast);
    const size_t len = MIN(strlen(s->ssid), 32);

    stq_le_p(f + n, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    stw_le_p(f + n + 8, 100);
    stw_le_p(f + n + 10, ap_capability(s));
    n += 12;
    f[n++] = 0;
    f[n++] = len;
    memcpy(f + n, s->ssid, len);
    n += len;
    n += ap_rates(f + n);
    f[n++] = 3;
    f[n++] = 1;
    f[n++] = s->channel;
    f[n++] = 0x2a;
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

static void ap_auth(AtbmSdioState *s, const uint8_t *sta, uint8_t if_id)
{
    uint8_t f[64];
    size_t n = ap_hdr(s, f, FC_AUTH, sta);

    stw_le_p(f + n, 0);
    stw_le_p(f + n + 2, 2);
    stw_le_p(f + n + 4, 0);
    atbm_rx(s, f, n + 6, 0, if_id);
}

static void ap_assoc(AtbmSdioState *s, const uint8_t *sta, bool re,
                     uint8_t if_id)
{
    uint8_t f[64];
    size_t n = ap_hdr(s, f, re ? FC_REASSOC_RESP : FC_ASSOC_RESP, sta);

    stw_le_p(f + n, ap_capability(s));
    stw_le_p(f + n + 2, 0);
    stw_le_p(f + n + 4, 0xc001);
    n += 6;
    n += ap_rates(f + n);
    memcpy(s->sta, sta, 6);
    s->associated = true;
    s->sta_if = if_id;
    if (s->trace) {
        qemu_log("atbm-sdio: %02x:%02x:%02x:%02x:%02x:%02x associated\n",
                 sta[0], sta[1], sta[2], sta[3], sta[4], sta[5]);
    }
    atbm_rx(s, f, n, 0, if_id);
    atbm_wpa_associated(&s->wpa, s->bssid, sta);
}

static void atbm_scan_done(void *opaque)
{
    AtbmSdioState *s = opaque;
    uint8_t b[4 + 2] = { 0 };

    ap_beacon(s, s->conf.macaddr.a, s->scan_if);
    ap_beacon(s, NULL, s->scan_if);
    b[5] = 14;
    atbm_send(s, WSM_SCAN_DONE_IND | s->scan_if << 6, b, sizeof(b));
}

static void ap_data_from_sta(AtbmSdioState *s, const uint8_t *f, size_t n)
{
    const size_t hl = (f[0] == FC_QOS_DATA) ? 26 : 24;
    uint8_t eth[2048];
    size_t p, end;

    if (!s->associated || !(f[1] & 1) ||
        atbm_wpa_from_sta(&s->wpa, f, n, hl, &p, &end) != ATBM_WPA_PASS ||
        end - p - 6 + 12 > sizeof(eth)) {
        return;
    }
    memcpy(eth, f + 16, 6);
    memcpy(eth + 6, f + 10, 6);
    memcpy(eth + 12, f + p + 6, end - p - 6);
    qemu_send_packet(qemu_get_queue(s->nic), eth, 12 + end - p - 6);
}

static void ap_tx(AtbmSdioState *s, const uint8_t *f, size_t n, uint8_t if_id)
{
    if (n < 24) {
        return;
    }
    if (s->trace) {
        qemu_log("atbm-sdio: tx fc 0x%02x%02x len %zu\n", f[1], f[0], n);
    }
    if (memcmp(f + 4, s->bssid, 6) && memcmp(f + 4, bcast, 6)) {
        return;
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

static bool atbm_can_receive(NetClientState *nc)
{
    return ATBM_SDIO(qemu_get_nic_opaque(nc))->fw_running;
}

static ssize_t atbm_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    AtbmSdioState *s = qemu_get_nic_opaque(nc);
    uint8_t f[24 + 8 + 6 + 2 + 2048 + 8];
    const bool group = buf[0] & 1;
    uint32_t flags = group ? 1u << 9 : 0;
    size_t n = 24, mic;

    if (size < 14) {
        return size;
    }
    if (!s->associated || 24 + 8 + 8 + size - 12 + 8 > sizeof(f) ||
        (!group && memcmp(buf, s->sta, 6)) || !atbm_wpa_port_open(&s->wpa)) {
        return size;
    }
    memset(f, 0, 24);
    f[0] = FC_DATA;
    f[1] = 0x02;                /* FromDS */
    memcpy(f + 4, buf, 6);
    memcpy(f + 10, s->bssid, 6);
    memcpy(f + 16, buf + 6, 6);
    stw_le_p(f + 22, s->ap_seq++ << 4);
    mic = atbm_wpa_to_sta(&s->wpa, group, f + n, &flags);
    if (mic) {
        f[1] |= 0x40;           /* Protected */
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

/* ---- WSM from the driver ---------------------------------------------- */

/*
 * Over SDIO every message goes out with a 28-byte prefix ahead of its
 * { len, id } header -- an 8-byte bus header (total length, checksum
 * mark, flags) and a 20-byte cipher header -- and len counts the prefix.
 * The driver packs several messages into one queue write, each starting
 * on a 256-byte (block) boundary.
 */
#define TX_PREFIX       28
#define TX_ALIGN        256

static void atbm_confirm(AtbmSdioState *s, uint16_t cmd, const void *body,
                         size_t n)
{
    g_autofree uint8_t *b = g_malloc0(4 + n);

    memcpy(b + 4, body, n);     /* status 0: WSM_STATUS_SUCCESS */
    atbm_send(s, WSM_CNF | cmd, b, 4 + n);
}

/* one WSM message from the queue: { len16, id16 } then body */
static void atbm_wsm_out(AtbmSdioState *s, const uint8_t *d, size_t n)
{
    uint16_t len, id, cmd;

    if (n < 4) {
        return;
    }
    len = MIN(lduw_le_p(d), n);
    id = lduw_le_p(d + 2);
    cmd = id & 0x3f;
    if (s->trace) {
        qemu_log("atbm-sdio: <- 0x%04x len %u\n", id, len);
    }

    switch (cmd) {
    case WSM_FIRMWARE_CHECK:
        break;
    case WSM_TX:
        if (len > TX_FRAME) {
            ap_tx(s, d + TX_FRAME, len - TX_FRAME, (id >> 6) & 0xf);
            if (ldl_le_p(d + TX_HT_PARAMS) & WSM_NEED_TX_CONFIRM) {
                uint8_t b[20] = { 0 };

                memcpy(b, d + TX_PACKET_ID, 4);
                atbm_send(s, WSM_CNF | WSM_TX, b, sizeof(b));
            }
        }
        break;
    case WSM_ADD_KEY:
        atbm_wpa_add_key(&s->wpa, d + 4, len - 4);
        atbm_confirm(s, cmd, NULL, 0);
        break;
    case WSM_START_SCAN:
        s->scan_if = (id >> 6) & 0xf;
        atbm_confirm(s, cmd, NULL, 0);
        timer_mod(s->scan_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
        break;
    case WSM_JOIN: {
        uint8_t b[8] = { 0 };

        stl_le_p(b + 4, 200);
        atbm_confirm(s, cmd, b, sizeof(b));
        break;
    }
    case WSM_CONFIGURATION: {
        uint8_t b[6 + 2 + 4 + 2 * 12] = { 0 };

        memcpy(b, s->conf.macaddr.a, 6);
        b[6] = 1;
        stl_le_p(b + 8, 0x3fff);
        for (int i = 0; i < 2; i++) {
            stl_le_p(b + 16 + 12 * i, 200);
            stl_le_p(b + 20 + 12 * i, 10);
        }
        atbm_confirm(s, cmd, b, sizeof(b));
        break;
    }
    case WSM_READ_MIB: {
        const uint16_t mib = len >= 4 + 2 ? lduw_le_p(d + 4) : 0;
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

/* ---- HIF register transport ------------------------------------------- */

static uint16_t atbm_control(AtbmSdioState *s)
{
    uint16_t c = CONTROL_RDY;

    if (!QSIMPLEQ_EMPTY(&s->rxq)) {
        AtbmMsg *m = QSIMPLEQ_FIRST(&s->rxq);

        c |= (m->len / 2) & CONTROL_LEN_LSB;    /* next length, in words */
    }
    return c;
}

/* prepare @len bytes for a function-1 read that just started */
static void atbm_fn1_read_begin(AtbmSdioState *s, uint32_t len)
{
    len = MIN(len, XFER_MAX);
    memset(s->xfer, 0, len);
    switch (s->io_reg) {
    case HIF_CONFIG:
        stl_le_p(s->xfer, s->config);
        break;
    case HIF_CONTROL:
        stw_le_p(s->xfer, atbm_control(s));
        break;
    case HIF_QUEUE: {
        /* the head message, then the next CONTROL in the last two bytes */
        AtbmMsg *m = QSIMPLEQ_FIRST(&s->rxq);

        if (m) {
            memcpy(s->xfer, m->data, MIN(m->len, len));
            QSIMPLEQ_REMOVE_HEAD(&s->rxq, next);
            s->rxq_len--;
            g_free(m);
        }
        if (len >= 2) {
            stw_le_p(s->xfer + len - 2, atbm_control(s));
        }
        atbm_irq(s);
        break;
    }
    case HIF_AHB_DPORT:
    case HIF_SRAM_DPORT:
        atbm_mem_read(s, s->sram_base, s->xfer, len);
        s->sram_base += len;
        break;
    }
}

/* a completed function-1 write of xfer_len bytes */
static void atbm_fn1_write_end(AtbmSdioState *s)
{
    uint32_t len = s->xfer_len;

    switch (s->io_reg) {
    case HIF_CONFIG: {
        uint32_t old = s->config;

        if (len >= 4) {
            s->config = ldl_le_p(s->xfer);
        }
        /* the prefetch bits clear once the chip has fetched the word */
        s->config &= ~(CONFIG_AHB_PFETCH | CONFIG_PREFETCH);
        if ((old & CONFIG_CPU_RESET) && !(s->config & CONFIG_CPU_RESET)) {
            atbm_cpu_release(s);
        }
        break;
    }
    case HIF_CONTROL:
        break;                  /* wakeup bit etc.: nothing to do */
    case HIF_SRAM_BASE:
        if (len >= 4) {
            s->sram_base = ldl_le_p(s->xfer);
        }
        break;
    case HIF_AHB_DPORT:
    case HIF_SRAM_DPORT:
        atbm_mem_write(s, s->sram_base, s->xfer, len);
        s->sram_base += len;
        break;
    case HIF_QUEUE: {
        uint32_t off = 0;

        while (off + TX_PREFIX + 4 <= len) {
            uint32_t mlen = lduw_le_p(s->xfer + off + TX_PREFIX);

            if (mlen < TX_PREFIX + 4 || off + mlen > len) {
                break;
            }
            s->tx_consumed++;
            atbm_wsm_out(s, s->xfer + off + TX_PREFIX, mlen - TX_PREFIX);
            off += ROUND_UP(mlen, TX_ALIGN);
        }
        break;
    }
    }
}

/* ---- SDIO card methods ------------------------------------------------- */

static void atbm_build_cis(AtbmSdioState *s)
{
    /* common CIS: MANFID, then a function-0 extension (block size, speed) */
    uint8_t *c = s->fn0 + CIS_COMMON;
    int i = 0;

    c[i++] = 0x20; c[i++] = 4;                  /* CISTPL_MANFID */
    c[i++] = ATBM_VID & 0xff; c[i++] = ATBM_VID >> 8;
    c[i++] = ATBM_PID & 0xff; c[i++] = ATBM_PID >> 8;
    c[i++] = 0x22; c[i++] = 4;                  /* CISTPL_FUNCE, common */
    c[i++] = 0x00;                              /* type 0: function 0 */
    c[i++] = 0x00; c[i++] = 0x02;               /* fn0 max block size 512 */
    c[i++] = 0x32;                              /* max transfer speed 25 MHz */
    c[i++] = 0xff;                              /* CISTPL_END */

    /* function-1 CIS: a function extension whose body holds the max block
     * size at offset 12 (sdio_cis.c wants at least 28 bytes) */
    c = s->fn0 + CIS_FUNC1;
    memset(c, 0, 0x40);
    c[0] = 0x22; c[1] = 0x2a;                   /* CISTPL_FUNCE, 42 bytes */
    c[2] = 0x01;                                /* type 1: a function */
    c[2 + 12] = 0x00; c[2 + 13] = 0x02;        /* max block size 512 */
    c[2 + 0x2a] = 0xff;                         /* CISTPL_END */
}

static void atbm_cccr_reset(AtbmSdioState *s)
{
    memset(s->fn0, 0, sizeof(s->fn0));
    s->fn0[0x00] = 0x43;
    s->fn0[0x08] = 0xc2;
    s->fn0[0x09] = CIS_COMMON & 0xff;
    s->fn0[0x0a] = (CIS_COMMON >> 8) & 0xff;
    s->fn0[0x0b] = (CIS_COMMON >> 16) & 0xff;
    s->fn0[FBR1 + 0x00] = 0x0c;
    s->fn0[FBR1 + 0x09] = CIS_FUNC1 & 0xff;
    s->fn0[FBR1 + 0x0a] = (CIS_FUNC1 >> 8) & 0xff;
    s->fn0[FBR1 + 0x0b] = (CIS_FUNC1 >> 16) & 0xff;
    s->fn_enable = s->int_enable = s->bus_width = 0;
    s->fn1_block_size = 0;
    atbm_build_cis(s);
}

static uint8_t atbm_fn0_read(AtbmSdioState *s, uint32_t addr)
{
    switch (addr) {
    case 0x02: return s->fn_enable;
    case 0x03: return s->fn_enable;     /* ready mirrors enable */
    case 0x04: return s->int_enable;
    case 0x07: return s->bus_width;
    case 0x110: return s->fn1_block_size & 0xff;
    case 0x111: return s->fn1_block_size >> 8;
    }
    return addr < FN0_WINDOW ? s->fn0[addr] : 0;
}

static void atbm_fn0_write(AtbmSdioState *s, uint32_t addr, uint8_t v)
{
    switch (addr) {
    case 0x02: s->fn_enable = v; return;
    case 0x04: s->int_enable = v; return;
    case 0x06: return;
    case 0x07: s->bus_width = v; return;
    case 0x110: s->fn1_block_size = (s->fn1_block_size & 0xff00) | v; return;
    case 0x111: s->fn1_block_size = (s->fn1_block_size & 0xff) | v << 8; return;
    }
    if (addr >= CCCR_SIZE && addr < FN0_WINDOW) {
        s->fn0[addr] = v;
    }
}

static size_t atbm_resp32(uint8_t *resp, uint32_t v)
{
    resp[0] = v >> 24; resp[1] = v >> 16; resp[2] = v >> 8; resp[3] = v;
    return 4;
}

static size_t atbm_do_command(SDState *sd, SDRequest *req, uint8_t *resp,
                              size_t respsz)
{
    AtbmSdioState *s = ATBM_SDIO(sd);
    uint32_t arg = req->arg;

    if (s->trace) {
        qemu_log("atbm-sdio: CMD%d arg 0x%08x\n", req->cmd, arg);
    }
    s->io_left = 0;
    switch (req->cmd) {
    case CMD_GO_IDLE:
        s->state = ATBM_IDLE;
        return 0;
    case CMD_SEND_IF_COND:
        return 0;
    case CMD_IO_SEND_OP_COND:
        /* the inquiry (arg 0) reports the OCR and function count but is
         * not yet ready; once the host sends a voltage the card is ready */
        s->state = ATBM_READY;
        return atbm_resp32(resp, (arg ? 0x80000000 : 0) |
                           (1u << 28) | 0x00ff8000);
    case CMD_SEND_RELATIVE_ADDR:
        s->rca = 1;
        s->state = ATBM_STANDBY;
        return atbm_resp32(resp, (uint32_t)s->rca << 16);
    case CMD_SELECT_CARD:
        s->state = (arg >> 16) == s->rca ? ATBM_TRANSFER : ATBM_STANDBY;
        return atbm_resp32(resp, 0x1e00);
    case CMD_IO_RW_DIRECT: {
        bool write = arg & (1u << 31);
        uint8_t fn = (arg >> 28) & 7;
        uint32_t addr = (arg >> 9) & 0x1ffff;
        uint8_t data = arg & 0xff, out;

        if (fn == 0) {
            if (write) {
                atbm_fn0_write(s, addr, data);
            }
            out = atbm_fn0_read(s, addr);
        } else {
            /* a one-byte HIF access (rare): one byte of a register */
            s->io_reg = (addr >> 2) & 7;
            if (write) {
                s->xfer[0] = data;
                s->xfer_len = 1;
                atbm_fn1_write_end(s);
                out = data;
            } else {
                atbm_fn1_read_begin(s, 1);
                out = s->xfer[0];
            }
        }
        return atbm_resp32(resp, (0x10 << 8) | out);
    }
    case CMD_IO_RW_EXTENDED: {
        bool block = arg & (1u << 27);
        uint32_t count = arg & 0x1ff, addr = (arg >> 9) & 0x1ffff, bytes;

        s->io_rw = arg & (1u << 31);
        s->io_fn = (arg >> 28) & 7;
        s->io_reg = (addr >> 2) & 7;
        bytes = block ? (count ? count : 512) * s->fn1_block_size
                      : (count ? count : 512);
        bytes = MIN(bytes, XFER_MAX);
        s->io_left = bytes;
        s->xfer_pos = 0;
        s->xfer_len = bytes;
        if (!s->io_rw && s->io_fn == 1) {
            atbm_fn1_read_begin(s, bytes);
        }
        return atbm_resp32(resp, (0x10 << 8));
    }
    default:
        qemu_log_mask(LOG_UNIMP, "atbm-sdio: CMD%d arg 0x%08x\n",
                      req->cmd, arg);
        return 0;
    }
}

static void atbm_write_byte(SDState *sd, uint8_t value)
{
    AtbmSdioState *s = ATBM_SDIO(sd);

    if (!s->io_left || !s->io_rw) {
        return;
    }
    if (s->io_fn == 1 && s->xfer_pos < XFER_MAX) {
        s->xfer[s->xfer_pos++] = value;
    }
    if (!--s->io_left && s->io_fn == 1) {
        atbm_fn1_write_end(s);
    }
}

static uint8_t atbm_read_byte(SDState *sd)
{
    AtbmSdioState *s = ATBM_SDIO(sd);
    uint8_t v = 0;

    if (!s->io_left || s->io_rw) {
        return 0;
    }
    if (s->io_fn == 1 && s->xfer_pos < XFER_MAX) {
        v = s->xfer[s->xfer_pos++];
    }
    s->io_left--;
    return v;
}

static bool atbm_data_ready(SDState *sd)
{
    return ATBM_SDIO(sd)->io_left > 0;
}

static bool atbm_receive_ready(SDState *sd)
{
    return ATBM_SDIO(sd)->io_left > 0;
}

static bool atbm_get_inserted(SDState *sd)
{
    return true;
}

static bool atbm_get_readonly(SDState *sd)
{
    return false;
}

static void atbm_sdio_realize(DeviceState *dev, Error **errp)
{
    AtbmSdioState *s = ATBM_SDIO(dev);

    s->sdbus = SD_BUS(qdev_get_parent_bus(dev));
    if (!s->ssid) {
        s->ssid = g_strdup("OpenIPC");
    }
    s->mem = g_hash_table_new_full(NULL, NULL, NULL, g_free);
    QSIMPLEQ_INIT(&s->rxq);
    s->start_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_startup, s);
    s->scan_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_scan_done, s);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    memcpy(s->bssid, s->conf.macaddr.a, 6);
    s->bssid[0] = 0x02;
    s->bssid[5] ^= 0x80;
    if (!atbm_wpa_init(&s->wpa, s->psk, s->ssid, "atbm-sdio", s->trace, s,
                       wpa_rx, wpa_deauth, &s->ap_seq, errp)) {
        return;
    }
    s->nic = qemu_new_nic(&atbm_net_info, &s->conf,
                          object_get_typename(OBJECT(s)),
                          dev->id, &dev->mem_reentrancy_guard, s);
    atbm_cccr_reset(s);
}

static void atbm_flush(AtbmSdioState *s)
{
    AtbmMsg *m;

    while ((m = QSIMPLEQ_FIRST(&s->rxq))) {
        QSIMPLEQ_REMOVE_HEAD(&s->rxq, next);
        g_free(m);
    }
    s->rxq_len = 0;
}

static void atbm_sdio_reset(DeviceState *dev)
{
    AtbmSdioState *s = ATBM_SDIO(dev);

    timer_del(s->start_timer);
    timer_del(s->scan_timer);
    atbm_wpa_reset(&s->wpa);
    atbm_flush(s);
    g_hash_table_remove_all(s->mem);
    s->state = ATBM_IDLE;
    s->rca = 0;
    s->io_left = 0;
    s->config = 0;
    s->sram_base = 0;
    s->tx_consumed = 0;
    s->rx_seq = 0;
    s->fw_running = false;
    s->associated = false;
    atbm_cccr_reset(s);
    sdbus_set_sdio_irq(s->sdbus, false);
}

static void atbm_sdio_unrealize(DeviceState *dev)
{
    AtbmSdioState *s = ATBM_SDIO(dev);

    timer_free(s->start_timer);
    timer_free(s->scan_timer);
    atbm_wpa_free(&s->wpa);
    qemu_del_nic(s->nic);
    atbm_flush(s);
    g_hash_table_destroy(s->mem);
}

/* the firmware, the AP state and the queue are rebuilt by the driver
 * reloading the chip, so migration keeps only the SDIO plumbing */
static const VMStateDescription vmstate_atbm_sdio = {
    .name = TYPE_ATBM_SDIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(state, AtbmSdioState),
        VMSTATE_UINT16(rca, AtbmSdioState),
        VMSTATE_UINT8(fn_enable, AtbmSdioState),
        VMSTATE_UINT8(int_enable, AtbmSdioState),
        VMSTATE_UINT8(bus_width, AtbmSdioState),
        VMSTATE_UINT16(fn1_block_size, AtbmSdioState),
        VMSTATE_UINT32(config, AtbmSdioState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property atbm_sdio_props[] = {
    DEFINE_NIC_PROPERTIES(AtbmSdioState, conf),
    DEFINE_PROP_STRING("ssid", AtbmSdioState, ssid),
    DEFINE_PROP_STRING("psk", AtbmSdioState, psk),
    DEFINE_PROP_UINT8("channel", AtbmSdioState, channel, 6),
    DEFINE_PROP_BOOL("trace", AtbmSdioState, trace, false),
};

static bool atbm_get_selftest(Object *obj, Error **errp)
{
    return atbm_wpa_selftest();
}

static void atbm_sdio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = (SDCardClass *)klass;

    dc->realize = atbm_sdio_realize;
    dc->unrealize = atbm_sdio_unrealize;
    device_class_set_legacy_reset(dc, atbm_sdio_reset);
    dc->vmsd = &vmstate_atbm_sdio;
    dc->bus_type = TYPE_SD_BUS;
    device_class_set_props(dc, atbm_sdio_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    object_class_property_add_bool(klass, "selftest", atbm_get_selftest,
                                   NULL);

    sc->do_command = atbm_do_command;
    sc->write_byte = atbm_write_byte;
    sc->read_byte = atbm_read_byte;
    sc->data_ready = atbm_data_ready;
    sc->receive_ready = atbm_receive_ready;
    sc->get_inserted = atbm_get_inserted;
    sc->get_readonly = atbm_get_readonly;
}

static const TypeInfo atbm_sdio_info = {
    .name = TYPE_ATBM_SDIO,
    .parent = TYPE_SD_BUS_CARD,
    .instance_size = sizeof(AtbmSdioState),
    .class_size = sizeof(AtbmSdioClass),
    .class_init = atbm_sdio_class_init,
};

static void atbm_sdio_register_types(void)
{
    type_register_static(&atbm_sdio_info);
}

type_init(atbm_sdio_register_types)
