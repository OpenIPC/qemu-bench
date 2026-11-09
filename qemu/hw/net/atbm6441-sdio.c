/*
 * AltoBeam ATBM6441 SDIO Wi-Fi in "IoT mode", as the atbm6441 Linux
 * driver (hal_apollo built for HERA, PROJ_TYPE 11) drives it on the Ingenic
 * MSC -- the chip of battery doorbells such as the Merkury MI-CW074-125W.
 *
 * Unlike the ATBM6031 (atbm-sdio.c) the 6441 is a whole Wi-Fi SoC: its
 * own CPU runs the firmware from its own flash, keeps the credentials,
 * scans, associates, does the WPA handshake and DHCP before the host has
 * booted.  The host driver downloads nothing; it
 *   - enumerates F1 and talks to the same HIF registers (CMD53, address
 *     = reg << 2: CONFIG 0x00, CONTROL 0x04, the queue port 0x08 | buf_id
 *     << 6 | force << 5, the AHB data port 0x0c and its address 0x10);
 *   - sets WUP in CONTROL, then discards the first four messages it reads
 *     (bh_sdio.c atbm_rx_directly, "soft reset bug") and waits for the
 *     startup indication, which must carry sequence 4;
 *   - asks GET_CONFIG for the MAC and the link the chip already has
 *     (bconnect, HostConnectEvent), creates wlan0 with that MAC and, once
 *     it is up, a station entry for the BSSID, and tells the chip
 *     NETWORK_OK;
 *   - exchanges data as raw 802.11 (ToDS out, FromDS in, LLC/SNAP), the
 *     chip forwarding to the host what its own stack does not consume;
 *   - sends AT commands (/dev/atbm_ioctl, WSM 0x0023) to the chip.
 * TX flow control is a credit count the chip returns in event 3
 * (CONFIG_TX_NO_CONFIRM): cumulative messages consumed plus the window.
 *
 * Here the chip is already associated to an open access point (ssid=,
 * the BSSID derived from the MAC) and holds the address slirp leases
 * (ip=, gw=, dns=, reported in the connect event); frames go both ways to
 * a QEMU netdev.  The AT command set of the real firmware is not visible
 * from the driver: every command is logged (trace) and answered "OK" in
 * a CUSTOMER_PRIVATE event, which is what /dev/atbm_ioctl readers get.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_ATBM6441 "atbm6441-sdio"
OBJECT_DECLARE_TYPE(Atbm6441State, Atbm6441Class, ATBM6441)

struct Atbm6441Class {
    SDCardClass parent_class;
};

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

#define ATBM_VID        0x007a
#define ATBM_PID        0x6441

/* HIF register windows (CMD53 address & 0x1f) */
#define HIF_CONFIG      0x00
#define HIF_CONTROL     0x04
#define HIF_QUEUE       0x08
#define HIF_AHB_DPORT   0x0c
#define HIF_SRAM_BASE   0x10

#define CONFIG_ACCESS_MODE  (1u << 10)
#define CONFIG_AHB_PFETCH   (1u << 11)
#define CONFIG_PREFETCH     (1u << 13)
#define CONFIG_DATA_IRQ     (1u << 16)

#define CONTROL_WUP     (1u << 12)
#define CONTROL_RDY     (1u << 13)

#define XFER_MAX        (32 * 1024)
#define RXQ_MAX         256
#define RX_ALIGN        256     /* SDIO_BLOCK_SIZE: reads are rounded to it */
#define TX_ALIGN        256

/* WSM */
#define WSM_CNF             0x0400
#define WSM_STARTUP_IND     0x0801
#define WSM_TRACE_IND       0x0802
#define WSM_RX_IND          0x0804
#define WSM_EVENT_IND       0x0805

#define WSM_GET_CONFIG      0x0002
#define WSM_TX              0x0004
#define WSM_GET_SCAN_INFO   0x000b
#define WSM_GET_AP_CFG      0x0021
#define WSM_AT_CMD          0x0023
#define WSM_IPC_DATA        0x0028
#define WSM_CHECK_ALIVE     0x003a
#define WSM_DIRECT_BUFFER   0x003b
#define WSM_NETWORK_OK      0x003d

#define EVENT_INPUTCHAN     3
#define EVENT_CUSTOMER_PRIV 0x4321
#define EVENT_BUF           504     /* firmware >= 12802 */

#define FW_VERSION          13679   /* >= 13679: the host sends NETWORK_OK */
#define NUM_INP_BUFS        16
#define SIZE_INP_BUF        2048

#define GETCONFIG_LEN       162
#define CONEVT_OFF          32

/* wsm_tx: { flag32, len16, id16 } then packetID .. htTxParameters */
#define TX_HDR              8
#define TX_FLAGS            15
#define TX_2BYTES_SHIFT     0x80
#define TX_FRAME            28

#define FC_DATA             0x08
#define FC_QOS_DATA         0x88

enum { ST_IDLE, ST_READY, ST_STANDBY, ST_TRANSFER };

typedef struct Msg {
    QSIMPLEQ_ENTRY(Msg) next;
    size_t len;
    uint8_t data[];
} Msg;

struct Atbm6441State {
    DeviceState parent_obj;

    SDBus *sdbus;
    uint32_t state;
    uint16_t rca;

    uint8_t fn0[FN0_WINDOW];
    uint8_t fn_enable, int_enable, bus_width;
    uint16_t fn1_block_size;

    /* a pending CMD53 data phase */
    bool     io_rw;
    uint8_t  io_fn, io_reg;
    uint32_t io_left, xfer_pos, xfer_len;
    uint8_t  xfer[XFER_MAX];

    /* HIF */
    uint32_t config;
    bool     wup;
    uint32_t sram_base;
    GHashTable *ahb;            /* AHB word address -> value */

    /* chip -> host: queued messages and the read image being served */
    QSIMPLEQ_HEAD(, Msg) rxq;
    unsigned rxq_len;
    uint8_t  rx_seq;
    uint8_t  img[XFER_MAX];
    uint32_t img_len, img_pos;

    /* host -> chip */
    uint32_t consumed;          /* messages taken off the queue port */
    bool     network_ok;        /* the host sent NETWORK_OK */

    /* the link the chip's own stack holds */
    NICConf conf;
    NICState *nic;
    char *ssid, *ip, *gw, *dns;
    uint8_t channel;
    bool wmm;
    uint8_t bssid[6];
    uint16_t seq;
    bool trace;
};

/* ---- chip -> host ------------------------------------------------------ */

static size_t rx_image_len(size_t len)
{
    return ROUND_UP(ROUND_UP(len, 2) + 2, RX_ALIGN);
}

static uint16_t control_for(Atbm6441State *s, const Msg *m)
{
    uint16_t c = CONTROL_RDY | (s->wup ? CONTROL_WUP : 0);

    if (m) {
        uint32_t words = (m->len + 1) / 2;

        c |= (words & 0x0fff) | ((words >> 12) & 3) << 14;
    }
    return c;
}

static void update_irq(Atbm6441State *s)
{
    bool level = (s->int_enable & 3) == 3 && (s->config & CONFIG_DATA_IRQ) &&
                 !QSIMPLEQ_EMPTY(&s->rxq);

    sdbus_set_sdio_irq(s->sdbus, level);
}

static void send_msg(Atbm6441State *s, uint16_t id, const void *body,
                     size_t n)
{
    Msg *m;

    if (s->rxq_len >= RXQ_MAX || 4 + n > XFER_MAX - RX_ALIGN) {
        qemu_log_mask(LOG_GUEST_ERROR, "atbm6441: queue full, 0x%04x "
                      "dropped\n", id);
        return;
    }
    m = g_malloc0(sizeof(*m) + 4 + n);
    m->len = 4 + n;
    stw_le_p(m->data, 4 + n);
    stw_le_p(m->data + 2, (id & 0x1fff) | (s->rx_seq << 13));
    s->rx_seq = (s->rx_seq + 1) & 7;
    if (n) {
        memcpy(m->data + 4, body, n);
    }
    if (s->trace && id != WSM_RX_IND) {
        qemu_log("atbm6441: -> 0x%04x len %zu\n", id, m->len);
    }
    QSIMPLEQ_INSERT_TAIL(&s->rxq, m, next);
    s->rxq_len++;
    update_irq(s);
}

static void send_event(Atbm6441State *s, uint32_t event, const void *buf,
                       size_t n)
{
    uint8_t b[4 + EVENT_BUF] = { 0 };

    stl_le_p(b, event);
    memcpy(b + 4, buf, MIN(n, EVENT_BUF - 1));     /* NUL-terminated */
    send_msg(s, WSM_EVENT_IND, b, sizeof(b));
}

static void send_credits(Atbm6441State *s)
{
    uint8_t b[8];

    stl_le_p(b, EVENT_INPUTCHAN);
    stl_le_p(b + 4, s->consumed + NUM_INP_BUFS);
    send_msg(s, WSM_EVENT_IND, b, sizeof(b));
}

static void flush_rxq(Atbm6441State *s)
{
    Msg *m;

    while ((m = QSIMPLEQ_FIRST(&s->rxq))) {
        QSIMPLEQ_REMOVE_HEAD(&s->rxq, next);
        g_free(m);
    }
    s->rxq_len = 0;
    s->img_len = s->img_pos = 0;
}

/*
 * The host set WUP: the chip, running all along, starts talking.  The
 * driver drops the first four messages unread and arms its sequence check
 * at 4, so four trace indications go first.
 */
static void chip_start(Atbm6441State *s)
{
    uint8_t b[166] = { 0 };
    int i;

    flush_rxq(s);
    s->rx_seq = 0;
    s->consumed = 0;
    s->network_ok = false;
    for (i = 0; i < 4; i++) {
        send_msg(s, WSM_TRACE_IND, NULL, 0);
    }
    stw_le_p(b + 0, NUM_INP_BUFS);
    stw_le_p(b + 2, SIZE_INP_BUF);
    stw_le_p(b + 4, 0x6441);            /* hardwareId */
    stw_le_p(b + 12, 1);                /* firmwareType */
    stw_le_p(b + 16, 1);                /* firmwareBuildNumber */
    stw_le_p(b + 18, FW_VERSION);
    pstrcpy((char *)b + 20, 128, "qemu atbm6441 IoT");
    send_msg(s, WSM_STARTUP_IND, b, sizeof(b));
    if (s->trace) {
        qemu_log("atbm6441: WUP set, startup indication queued\n");
    }
}

/* chip reset: F1 disabled, CCCR RES, power-on */
static void hif_reset(Atbm6441State *s)
{
    flush_rxq(s);
    s->config = CONFIG_ACCESS_MODE;
    s->wup = false;
    s->sram_base = 0;
    s->rx_seq = 0;
    s->consumed = 0;
    s->network_ok = false;
    update_irq(s);
}

/* ---- the link ---------------------------------------------------------- */

static void put_ip(uint8_t *p, const char *a)
{
    unsigned v[4];

    if (a && sscanf(a, "%u.%u.%u.%u", &v[0], &v[1], &v[2], &v[3]) == 4) {
        for (int i = 0; i < 4; i++) {
            p[i] = v[i];
        }
    }
}

static void get_config(Atbm6441State *s, uint8_t *b)
{
    uint8_t *ev = b + CONEVT_OFF;
    const size_t len = MIN(strlen(s->ssid), 32);

    memset(b, 0, GETCONFIG_LEN);
    memcpy(b + 4, s->conf.macaddr.a, 6);
    b[10] = 1;                          /* bconnect */
    b[16] = NUM_INP_BUFS;
    b[17] = 0;                          /* WifiMode: station */
    stl_le_p(b + 24, FW_VERSION);
    memcpy(ev, s->ssid, len);
    ev[32] = len;
    memcpy(ev + 33, s->bssid, 6);
    ev[39] = 0;                         /* crypto_pairwise: open */
    ev[40] = 0;                         /* crypto_group */
    ev[42] = 7;                         /* bgnMode: b, g, n */
    ev[43] = s->wmm ? 1 : 0;            /* connect_flag: WMM */
    put_ip(ev + 44, s->ip);
    put_ip(ev + 48, "255.255.255.0");
    put_ip(ev + 52, s->gw);
    put_ip(ev + 56, s->dns);
}

static const uint8_t llc_snap[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };

/* a host data frame: 802.11 ToDS, LLC/SNAP -> Ethernet to the netdev */
static void host_data(Atbm6441State *s, const uint8_t *d, size_t len)
{
    size_t off = TX_FRAME, hl;
    const uint8_t *f;
    uint8_t eth[2048];
    size_t n;

    if (len > TX_FLAGS && (d[TX_FLAGS] & TX_2BYTES_SHIFT)) {
        off += 2;
    }
    if (len < off + 24) {
        return;
    }
    f = d + off;
    n = len - off;
    hl = f[0] == FC_QOS_DATA ? 26 : 24;
    if ((f[0] != FC_DATA && f[0] != FC_QOS_DATA) || !(f[1] & 1) ||
        (f[1] & 0x40) || n < hl + 8 || memcmp(f + hl, llc_snap, 6) ||
        n - hl - 6 + 12 > sizeof(eth)) {
        qemu_log_mask(LOG_GUEST_ERROR, "atbm6441: tx frame fc %02x%02x "
                      "len %zu not forwarded\n", f[1], f[0], n);
        return;
    }
    memcpy(eth, f + 16, 6);             /* DA = addr3 */
    memcpy(eth + 6, f + 10, 6);         /* SA = addr2 */
    memcpy(eth + 12, f + hl + 6, n - hl - 6);
    qemu_send_packet(qemu_get_queue(s->nic), eth, 12 + n - hl - 6);
}

static bool can_receive(NetClientState *nc)
{
    return true;
}

/* the netdev's frames the chip hands the host: 802.11 FromDS */
static ssize_t receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    Atbm6441State *s = qemu_get_nic_opaque(nc);
    g_autofree uint8_t *b = NULL;
    uint8_t *f;

    if (size < 14 || size > 2048 || !s->network_ok ||
        (!(buf[0] & 1) && memcmp(buf, s->conf.macaddr.a, 6))) {
        return size;
    }
    b = g_malloc0(16 + 24 + 6 + size - 12);
    stw_le_p(b + 4, s->channel);
    b[6] = 7;                           /* rxedRate: 54 Mbit/s */
    b[7] = (uint8_t)-40;
    f = b + 16;
    f[0] = FC_DATA;
    f[1] = 0x02;                        /* FromDS */
    memcpy(f + 4, buf, 6);
    memcpy(f + 10, s->bssid, 6);
    memcpy(f + 16, buf + 6, 6);
    stw_le_p(f + 22, s->seq++ << 4);
    memcpy(f + 24, llc_snap, 6);
    memcpy(f + 30, buf + 12, size - 12);
    send_msg(s, WSM_RX_IND, b, 16 + 24 + 6 + size - 12);
    return size;
}

static NetClientInfo net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = can_receive,
    .receive = receive,
};

/* ---- host -> chip ------------------------------------------------------ */

static void confirm(Atbm6441State *s, uint16_t cmd, const void *b, size_t n)
{
    send_msg(s, WSM_CNF | cmd, b, n);
}

static void at_command(Atbm6441State *s, const uint8_t *p, size_t n)
{
    char cmd[257] = { 0 };
    char out[EVENT_BUF];
    uint8_t st[4] = { 0 };

    if (n > 4) {
        memcpy(cmd, p + 4, MIN(n - 4, 256));
    }
    g_strchomp(cmd);
    qemu_log_mask(LOG_UNIMP, "atbm6441: AT command '%s'\n", cmd);
    if (s->trace) {
        qemu_log("atbm6441: AT command '%s'\n", cmd);
    }
    confirm(s, WSM_AT_CMD, st, sizeof(st));
    snprintf(out, sizeof(out), "%s\r\nOK\r\n", cmd);
    send_event(s, EVENT_CUSTOMER_PRIV, out, strlen(out));
}

/* one message from the queue port: { flag32, len16, id16 } + payload */
static void host_msg(Atbm6441State *s, const uint8_t *d, size_t len)
{
    const uint16_t id = lduw_le_p(d + 6), cmd = id & 0x3f;
    const uint8_t *p = d + TX_HDR;
    const size_t n = len - TX_HDR;
    uint32_t flag = ldl_le_p(d), sum = (len & 0xff) + ((len >> 8) & 0xff);

    sum = (sum & 0x100) ? (sum + 1) & 0xff : sum & 0xff;
    if (flag != ((len & 0xffff) | sum << 24)) {
        qemu_log_mask(LOG_GUEST_ERROR, "atbm6441: message 0x%04x flag "
                      "0x%08x, want 0x%08x\n", id, flag,
                      (uint32_t)((len & 0xffff) | sum << 24));
    }
    if (s->trace && cmd != WSM_TX) {
        qemu_log("atbm6441: <- 0x%04x len %zu\n", id, len);
    }

    switch (cmd) {
    case WSM_TX:
        host_data(s, d, len);
        break;
    case WSM_NETWORK_OK:                /* no confirm */
        if (s->trace && !s->network_ok) {
            qemu_log("atbm6441: host network ok, forwarding\n");
        }
        s->network_ok = true;
        break;
    case WSM_IPC_DATA:                  /* no confirm */
        break;
    case WSM_GET_CONFIG: {
        uint8_t b[GETCONFIG_LEN];

        get_config(s, b);
        confirm(s, cmd, b, sizeof(b));
        break;
    }
    case WSM_AT_CMD:
        at_command(s, p, n);
        break;
    case WSM_CHECK_ALIVE: {
        uint8_t b[8] = { 0 };

        stl_le_p(b + 4, 2121);
        confirm(s, cmd, b, sizeof(b));
        break;
    }
    case WSM_GET_SCAN_INFO:
    case WSM_DIRECT_BUFFER: {
        /* the whole request comes back, status 0 */
        g_autofree uint8_t *b = g_malloc0(n);

        confirm(s, cmd, b, n);
        break;
    }
    default:
        if (n == 2) {
            /* a "get": the host copies exactly the size it asked for */
            uint16_t want = MIN(lduw_le_p(p), 1604);
            g_autofree uint8_t *b = g_malloc0(want);

            confirm(s, cmd, b, want);
        } else {
            uint8_t st[4] = { 0 };     /* WSM_STATUS_SUCCESS */

            confirm(s, cmd, st, sizeof(st));
        }
        break;
    }
}

/* a queue-port write: messages packed on 256-byte boundaries */
static void queue_write(Atbm6441State *s, const uint8_t *b, uint32_t total)
{
    uint32_t off = 0, taken = 0;

    while (off + TX_HDR <= total) {
        uint32_t len = lduw_le_p(b + off + 4);

        if (len < TX_HDR || off + len > total) {
            break;
        }
        s->consumed++;
        taken++;
        host_msg(s, b + off, len);
        off += ROUND_UP(MAX(len, 16), TX_ALIGN);
    }
    if (taken) {
        send_credits(s);
    }
}

/* ---- HIF register transport ------------------------------------------- */

static uint32_t ahb_read(Atbm6441State *s, uint32_t addr)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(s->ahb,
                                                GUINT_TO_POINTER(addr)));
}

/* the next byte of the queue port: the head message's read image */
static void queue_read(Atbm6441State *s, uint8_t *out, uint32_t len)
{
    while (len) {
        uint32_t k;

        if (s->img_pos >= s->img_len) {
            Msg *m = QSIMPLEQ_FIRST(&s->rxq);

            if (!m) {
                memset(out, 0, len);
                return;
            }
            QSIMPLEQ_REMOVE_HEAD(&s->rxq, next);
            s->rxq_len--;
            s->img_len = rx_image_len(m->len);
            s->img_pos = 0;
            memset(s->img, 0, s->img_len);
            memcpy(s->img, m->data, m->len);
            g_free(m);
            /* piggyback: CONTROL for the next message */
            stw_le_p(s->img + s->img_len - 2,
                     control_for(s, QSIMPLEQ_FIRST(&s->rxq)));
        }
        k = MIN(len, s->img_len - s->img_pos);
        memcpy(out, s->img + s->img_pos, k);
        s->img_pos += k;
        out += k;
        len -= k;
    }
    update_irq(s);
}

static void fn1_read_begin(Atbm6441State *s, uint32_t len)
{
    memset(s->xfer, 0, len);
    switch (s->io_reg) {
    case HIF_CONFIG:
        stl_le_p(s->xfer, s->config);
        break;
    case HIF_CONTROL:
        stl_le_p(s->xfer, control_for(s, s->img_pos < s->img_len ? NULL :
                                         QSIMPLEQ_FIRST(&s->rxq)));
        break;
    case HIF_QUEUE:
        queue_read(s, s->xfer, len);
        break;
    case HIF_AHB_DPORT:
        stl_le_p(s->xfer, ahb_read(s, s->sram_base));
        break;
    case HIF_SRAM_BASE:
        stl_le_p(s->xfer, s->sram_base);
        break;
    }
}

static void fn1_write_end(Atbm6441State *s)
{
    uint32_t len = s->xfer_len, v = len >= 4 ? ldl_le_p(s->xfer) : 0;

    switch (s->io_reg) {
    case HIF_CONFIG:
        /* the prefetch bits clear once the word is latched */
        s->config = v & ~(CONFIG_AHB_PFETCH | CONFIG_PREFETCH);
        update_irq(s);
        break;
    case HIF_CONTROL: {
        bool wup = v & CONTROL_WUP;

        if (wup && !s->wup) {
            s->wup = true;
            chip_start(s);
        } else if (!wup && s->wup) {
            s->wup = false;
            if (s->trace) {
                qemu_log("atbm6441: WUP cleared\n");
            }
        }
        break;
    }
    case HIF_SRAM_BASE:
        s->sram_base = v;
        break;
    case HIF_AHB_DPORT:
        g_hash_table_insert(s->ahb, GUINT_TO_POINTER(s->sram_base),
                            GUINT_TO_POINTER(v));
        break;
    case HIF_QUEUE:
        queue_write(s, s->xfer, len);
        break;
    }
}

/* ---- SDIO card --------------------------------------------------------- */

static void build_cis(Atbm6441State *s)
{
    uint8_t *c = s->fn0 + CIS_COMMON;
    int i = 0;

    c[i++] = 0x20; c[i++] = 4;                  /* CISTPL_MANFID */
    c[i++] = ATBM_VID & 0xff; c[i++] = ATBM_VID >> 8;
    c[i++] = ATBM_PID & 0xff; c[i++] = ATBM_PID >> 8;
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

static void cccr_reset(Atbm6441State *s)
{
    memset(s->fn0, 0, sizeof(s->fn0));
    s->fn0[0x00] = 0x43;
    s->fn0[0x08] = 0xc2;                        /* 4BLS, LSC, SMB */
    s->fn0[0x09] = CIS_COMMON & 0xff;
    s->fn0[0x0a] = (CIS_COMMON >> 8) & 0xff;
    s->fn0[0x0b] = (CIS_COMMON >> 16) & 0xff;
    s->fn0[FBR1 + 0x00] = 0x0c;
    s->fn0[FBR1 + 0x09] = CIS_FUNC1 & 0xff;
    s->fn0[FBR1 + 0x0a] = (CIS_FUNC1 >> 8) & 0xff;
    s->fn0[FBR1 + 0x0b] = (CIS_FUNC1 >> 16) & 0xff;
    s->fn_enable = s->int_enable = s->bus_width = 0;
    s->fn1_block_size = 0;
    build_cis(s);
}

static uint8_t fn0_read(Atbm6441State *s, uint32_t addr)
{
    switch (addr) {
    case CCCR_IOEX: return s->fn_enable;
    case 0x03: return s->fn_enable;            /* IORx: ready at once */
    case CCCR_IENX: return s->int_enable;
    case 0x05:                                  /* INTx: F1 pending */
        return (s->int_enable & 3) == 3 && (s->config & CONFIG_DATA_IRQ) &&
               !QSIMPLEQ_EMPTY(&s->rxq) ? 2 : 0;
    case 0x07: return s->bus_width;
    case 0x110: return s->fn1_block_size & 0xff;
    case 0x111: return s->fn1_block_size >> 8;
    }
    return addr < FN0_WINDOW ? s->fn0[addr] : 0;
}

static void fn0_write(Atbm6441State *s, uint32_t addr, uint8_t v)
{
    switch (addr) {
    case CCCR_IOEX:
        if ((s->fn_enable ^ v) & 2) {
            hif_reset(s);
            if (s->trace) {
                qemu_log("atbm6441: F1 %s\n", v & 2 ? "enabled" : "disabled");
            }
        }
        s->fn_enable = v;
        return;
    case CCCR_IENX:
        s->int_enable = v;
        update_irq(s);
        return;
    case CCCR_ABORT:
        if (v & ABORT_RES) {
            hif_reset(s);
        }
        return;
    case 0x07: s->bus_width = v; return;
    case 0x110: s->fn1_block_size = (s->fn1_block_size & 0xff00) | v; return;
    case 0x111: s->fn1_block_size = (s->fn1_block_size & 0xff) | v << 8; return;
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
    Atbm6441State *s = ATBM6441(sd);
    uint32_t arg = req->arg;

    s->io_left = 0;
    switch (req->cmd) {
    case CMD_GO_IDLE:
        s->state = ST_IDLE;
        return 0;
    case CMD_SEND_IF_COND:
        return 0;
    case CMD_IO_SEND_OP_COND:
        s->state = ST_READY;
        return resp32(resp, (arg ? 0x80000000 : 0) | (1u << 28) | 0x00ff8000);
    case CMD_SEND_RELATIVE_ADDR:
        s->rca = 1;
        s->state = ST_STANDBY;
        return resp32(resp, (uint32_t)s->rca << 16);
    case CMD_SELECT_CARD:
        s->state = (arg >> 16) == s->rca ? ST_TRANSFER : ST_STANDBY;
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
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "atbm6441: CMD52 to F%d 0x%x\n",
                          fn, addr);
        }
        return resp32(resp, (0x10 << 8) | out);
    }
    case CMD_IO_RW_EXTENDED: {
        bool block = arg & (1u << 27);
        uint32_t count = arg & 0x1ff, addr = (arg >> 9) & 0x1ffff, bytes;

        s->io_rw = arg & (1u << 31);
        s->io_fn = (arg >> 28) & 7;
        s->io_reg = addr & 0x1f;
        bytes = block ? (count ? count : 512) * s->fn1_block_size
                      : (count ? count : 512);
        if (bytes > XFER_MAX) {
            qemu_log_mask(LOG_GUEST_ERROR, "atbm6441: CMD53 of %u bytes\n",
                          bytes);
            bytes = XFER_MAX;
        }
        s->io_left = bytes;
        s->xfer_pos = 0;
        s->xfer_len = bytes;
        if (!s->io_rw && s->io_fn == 1) {
            fn1_read_begin(s, bytes);
        }
        return resp32(resp, 0x10 << 8);
    }
    default:
        qemu_log_mask(LOG_UNIMP, "atbm6441: CMD%d arg 0x%08x\n",
                      req->cmd, arg);
        return 0;
    }
}

static void write_byte(SDState *sd, uint8_t value)
{
    Atbm6441State *s = ATBM6441(sd);

    if (!s->io_left || !s->io_rw) {
        return;
    }
    if (s->io_fn == 1 && s->xfer_pos < XFER_MAX) {
        s->xfer[s->xfer_pos++] = value;
    }
    if (!--s->io_left && s->io_fn == 1) {
        fn1_write_end(s);
    }
}

static uint8_t read_byte(SDState *sd)
{
    Atbm6441State *s = ATBM6441(sd);
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

static bool data_ready(SDState *sd)
{
    return ATBM6441(sd)->io_left > 0;
}

static bool get_inserted(SDState *sd)
{
    return true;
}

static bool get_readonly(SDState *sd)
{
    return false;
}

static void atbm6441_realize(DeviceState *dev, Error **errp)
{
    Atbm6441State *s = ATBM6441(dev);

    s->sdbus = SD_BUS(qdev_get_parent_bus(dev));
    if (!s->ssid) {
        s->ssid = g_strdup("OpenIPC");
    }
    s->ahb = g_hash_table_new(NULL, NULL);
    QSIMPLEQ_INIT(&s->rxq);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    memcpy(s->bssid, s->conf.macaddr.a, 6);
    s->bssid[0] = 0x02;
    s->bssid[5] ^= 0x80;
    s->nic = qemu_new_nic(&net_info, &s->conf, object_get_typename(OBJECT(s)),
                          dev->id, &dev->mem_reentrancy_guard, s);
    cccr_reset(s);
}

static void atbm6441_reset(DeviceState *dev)
{
    Atbm6441State *s = ATBM6441(dev);

    s->state = ST_IDLE;
    s->rca = 0;
    s->io_left = 0;
    g_hash_table_remove_all(s->ahb);
    cccr_reset(s);
    hif_reset(s);
}

static void atbm6441_unrealize(DeviceState *dev)
{
    Atbm6441State *s = ATBM6441(dev);

    qemu_del_nic(s->nic);
    flush_rxq(s);
    g_hash_table_destroy(s->ahb);
}

/* the chip's messages are rebuilt by the driver reloading (WUP), so
 * migration keeps only the SDIO plumbing */
static const VMStateDescription vmstate_atbm6441 = {
    .name = TYPE_ATBM6441,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(state, Atbm6441State),
        VMSTATE_UINT16(rca, Atbm6441State),
        VMSTATE_UINT8(fn_enable, Atbm6441State),
        VMSTATE_UINT8(int_enable, Atbm6441State),
        VMSTATE_UINT8(bus_width, Atbm6441State),
        VMSTATE_UINT16(fn1_block_size, Atbm6441State),
        VMSTATE_UINT32(config, Atbm6441State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property atbm6441_props[] = {
    DEFINE_NIC_PROPERTIES(Atbm6441State, conf),
    DEFINE_PROP_STRING("ssid", Atbm6441State, ssid),
    DEFINE_PROP_STRING("ip", Atbm6441State, ip),
    DEFINE_PROP_STRING("gw", Atbm6441State, gw),
    DEFINE_PROP_STRING("dns", Atbm6441State, dns),
    DEFINE_PROP_UINT8("channel", Atbm6441State, channel, 6),
    DEFINE_PROP_BOOL("wmm", Atbm6441State, wmm, false),
    DEFINE_PROP_BOOL("trace", Atbm6441State, trace, false),
};

static void atbm6441_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = (SDCardClass *)klass;

    dc->realize = atbm6441_realize;
    dc->unrealize = atbm6441_unrealize;
    device_class_set_legacy_reset(dc, atbm6441_reset);
    dc->vmsd = &vmstate_atbm6441;
    dc->bus_type = TYPE_SD_BUS;
    device_class_set_props(dc, atbm6441_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->do_command = do_command;
    sc->write_byte = write_byte;
    sc->read_byte = read_byte;
    sc->data_ready = data_ready;
    sc->receive_ready = data_ready;
    sc->get_inserted = get_inserted;
    sc->get_readonly = get_readonly;
}

static const TypeInfo atbm6441_info = {
    .name = TYPE_ATBM6441,
    .parent = TYPE_SD_BUS_CARD,
    .instance_size = sizeof(Atbm6441State),
    .class_size = sizeof(Atbm6441Class),
    .class_init = atbm6441_class_init,
};

static void atbm6441_register_types(void)
{
    type_register_static(&atbm6441_info);
}

type_init(atbm6441_register_types)
