/*
 * Rockchip USB OTG controller (Synopsys DWC3) in device mode, with the
 * USB host it is plugged into built in.
 *
 * The core reports itself device-only (GHWPARAMS0 mode 0), so the kernel's
 * dwc3 driver (drivers/usb/dwc3: core.c, gadget.c, ep0.c) runs it as a
 * gadget whatever dr_mode says, and whatever gadget is bound (configfs,
 * g_cdc, g_ether, ...) sees a host enumerate it.  The registers:
 *
 *   0xc100.. globals   GSNPSID 0xc120, GHWPARAMS0..7 0xc140.., GTXFIFOSIZn
 *                      0xc300 + 4n, GRXFIFOSIZ0 0xc380; the rest storage
 *   0xc400 GEVNTADRLO  0xc404 ADRHI  0xc408 GEVNTSIZ (15:0 size, bit 31
 *                      interrupt mask)  0xc40c GEVNTCOUNT: bytes of events
 *                      pending; a write subtracts what the driver handled
 *   0xc700 DCFG (device address 9:3)  0xc704 DCTL (bit 31 run, bit 30
 *          soft reset, self-clearing)  0xc708 DEVTEN  0xc70c DSTS (bit 22
 *          halted while not running; speed 2:0, high speed)
 *   0xc714 DGCMD  0xc720 DALEPENA
 *   0xc800 + 0x10 n   endpoint n (physical: 2 x number + IN): DEPCMDPAR2,
 *          PAR1, PAR0, DEPCMD (3:0 command, bit 8 interrupt on completion,
 *          bit 10 active -- cleared at once, 31:16 parameter)
 *
 * The interrupt is up while events are pending and not masked.  Events are
 * 32-bit words in the ring at GEVNTADR: device events (bit 0 set, type in
 * 11:8: 1 reset, 2 connect done) and endpoint events (endpoint in 5:1,
 * type in 9:6: 1 XferComplete, 2 XferInProgress, 3 XferNotReady, 7 command
 * complete; status 15:12; parameters 31:16).
 *
 * Transfers are TRB lists in memory (16 bytes: buffer low / high, size
 * with the status in 31:28, control: bit 0 HWO, 1 LST, 2 CHN, 9:4 type --
 * 8 a link to the address in the buffer field --, 10 ISP_IMI, 11 IOC).
 * Start Transfer points an endpoint at its first TRB; the hardware works
 * through those the driver hands it (HWO), clears HWO, leaves in size what
 * was not transferred, and reports per IOC (XferInProgress) or at the
 * last TRB (XferComplete); Update Transfer says more were added.
 *
 * The host: when the driver sets RUN, it resets the bus (reset, then
 * connect done at high speed) and enumerates over endpoint 0 as the
 * driver's control state machine expects -- the SETUP packet into the
 * setup TRB and XferComplete; the data stage on the TRBs the driver starts
 * at once on the data direction; XferNotReady(STATUS) on the status
 * direction, then XferComplete of the status TRB.  It reads the device and
 * configuration descriptors, sets address 1 and the configuration, and if
 * the configuration has a CDC Ethernet (ECM) function, selects its data
 * interface's alternate setting 1 and bridges its bulk endpoints to the
 * network backend ("netdev"): each request the gadget sends IN is a frame
 * out, each frame in fills the next OUT TRBs.  An RNDIS function instead
 * gets REMOTE_NDIS_INITIALIZE and a packet filter (SEND_ENCAPSULATED_
 * COMMAND, each answer read with GET_ENCAPSULATED_RESPONSE), which turns
 * its carrier on, and its frames travel in REMOTE_NDIS_PACKET_MSGs.
 * A CDC ACM function gets SET_CONTROL_LINE_STATE (DTR, RTS) and its bulk
 * pair is the serial port "chardev": what the gadget writes to ttyGSn
 * comes out there, what comes in there the gadget reads.  An adb function
 * (FunctionFS, interface ff/42/01) is a byte stream the same way, on
 * "adb-chardev": adb's TCP transport carries the same messages as its USB
 * one, so "adb connect" to a socket chardev reaches adbd.  Notifications
 * are read and dropped.  Other functions enumerate and stay idle.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "chardev/char-fe.h"
#include "hw/qdev-properties-system.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_RK_DWC3 "rk-dwc3"
OBJECT_DECLARE_SIMPLE_TYPE(RkDwc3State, RK_DWC3)

#define REGS_SIZE       0x10000
#define R_GSNPSID       0xc120
#define R_GHWPARAMS0    0xc140
#define R_GHWPARAMS1    0xc144
#define R_GHWPARAMS3    0xc14c
#define R_GTXFIFOSIZ(n) (0xc300 + 4 * (n))
#define R_GRXFIFOSIZ0   0xc380
#define R_GEVNTADRLO    0xc400
#define R_GEVNTADRHI    0xc404
#define R_GEVNTSIZ      0xc408
#define R_GEVNTCOUNT    0xc40c
#define R_DCFG          0xc700
#define R_DCTL          0xc704
#define R_DSTS          0xc70c
#define R_DGCMD         0xc714
#define R_DEP(n)        (0xc800 + 0x10 * (n))

#define DCTL_RUN        (1u << 31)
#define DCTL_CSFTRST    (1u << 30)
#define DSTS_HALTED     (1u << 22)
#define CMDACT          (1u << 10)
#define CMDIOC          (1u << 8)
#define EVNTSIZ_MASK    (1u << 31)

#define CMD_SETEPCONFIG 1
#define CMD_SETSTALL    4
#define CMD_START       6
#define CMD_UPDATE      7
#define CMD_END         8

#define EV_COMPLETE     1
#define EV_INPROGRESS   2
#define EV_NOTREADY     3
#define EV_CMDCMPLT     7
#define EVST_SHORT      (1u << 1)
#define EVST_IOC        (1u << 2)
#define EVST_LST        (1u << 3)
#define EVST_CTRL_STATUS 2
#define DEVEV_RESET     1
#define DEVEV_CONNDONE  2

#define TRB_HWO         (1u << 0)
#define TRB_LST         (1u << 1)
#define TRB_CHN         (1u << 2)
#define TRB_ISP_IMI     (1u << 10)
#define TRB_IOC         (1u << 11)
#define TRB_TYPE(c)     (((c) >> 4) & 0x3f)
#define TRBT_LINK       8

#define NEPS            16          /* physical endpoints */
#define TICK_NS         100000      /* the host's pace: 100 us */
#define MAX_FRAME       2048
#define RXQ             16

typedef struct RkDwc3Ep {
    bool started;
    bool stalled;
    uint32_t trb;               /* the TRB the hardware is at */
} RkDwc3Ep;

/* The host's enumeration, one control transfer at a time */
enum {
    H_IDLE,                     /* not connected */
    H_SETUP,                    /* waiting for the setup TRB */
    H_DATA,                     /* waiting for the data stage TRBs */
    H_STATUS,                   /* XferNotReady sent, waiting for status */
    H_RUN,                      /* enumerated */
};

enum {
    STEP_DEV_DESC, STEP_SET_ADDR, STEP_CFG9, STEP_CFG, STEP_SET_CFG,
    STEP_SET_IF, STEP_RNDIS_INIT, STEP_RNDIS_INIT_RSP, STEP_RNDIS_FILTER,
    STEP_RNDIS_FILTER_RSP, STEP_ACM_DTR, STEP_DONE,
};

/* A function's bulk pair as a byte stream to a chardev */
typedef struct RkDwc3Pipe {
    int in, out;                /* physical endpoints, -1: none */
    CharFrontend chr;
    uint8_t rx[65536];          /* bytes for the gadget */
    uint32_t rx_n;
} RkDwc3Pipe;

struct RkDwc3State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *tick;
    NICState *nic;
    NICConf conf;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t ev_head;           /* where the next event goes in the ring */
    uint32_t ev_count;          /* bytes pending */
    RkDwc3Ep ep[NEPS];

    int hstate, step;
    uint8_t setup[8];
    uint8_t ctl[1024];          /* the control transfer's data */
    uint32_t ctl_len;
    bool ctl_in;
    uint8_t cfg_value;
    int ecm_data_if;            /* -1: none */
    int rndis_if;               /* RNDIS control interface, -1: none */
    int ep_in, ep_out;          /* the network's bulk pair, -1: none */
    int ep_notify[4];           /* interrupt IN endpoints: read, dropped */
    int n_notify;
    int acm_if;                 /* CDC ACM control interface, -1: none */
    RkDwc3Pipe acm, adb;        /* byte streams: ACM's bulk pair, adb's */

    uint8_t tx[MAX_FRAME];      /* the frame the gadget is sending */
    uint32_t tx_len;
    uint8_t rxq[RXQ][MAX_FRAME];    /* frames for the gadget */
    uint32_t rxq_len[RXQ];
    int rxq_head, rxq_n;
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_dwc3_update(RkDwc3State *s)
{
    qemu_set_irq(s->irq, s->ev_count && !(REG(s, R_GEVNTSIZ) & EVNTSIZ_MASK));
}

static void rk_dwc3_event(RkDwc3State *s, uint32_t ev)
{
    uint32_t size = REG(s, R_GEVNTSIZ) & 0xfffc;
    uint64_t base = REG(s, R_GEVNTADRLO) |
                    (uint64_t)REG(s, R_GEVNTADRHI) << 32;
    uint32_t le = cpu_to_le32(ev);

    if (!size || s->ev_count + 4 > size) {
        qemu_log_mask(LOG_GUEST_ERROR, "rk-dwc3: event ring full\n");
        return;
    }
    dma_memory_write(&address_space_memory, base + s->ev_head, &le, 4,
                     MEMTXATTRS_UNSPECIFIED);
    s->ev_head = (s->ev_head + 4) % size;
    s->ev_count += 4;
    rk_dwc3_update(s);
}

static void rk_dwc3_ep_event(RkDwc3State *s, int ep, int type, int status,
                             uint32_t param)
{
    rk_dwc3_event(s, (ep << 1) | (type << 6) | (status << 12) |
                     (param << 16));
}

static void rk_dwc3_trb_rd(uint32_t a, uint32_t t[4])
{
    int i;

    dma_memory_read(&address_space_memory, a, t, 16, MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < 4; i++) {
        t[i] = le32_to_cpu(t[i]);
    }
}

static void rk_dwc3_trb_wr(uint32_t a, const uint32_t t[4])
{
    uint32_t le[2] = { cpu_to_le32(t[2]), cpu_to_le32(t[3]) };

    dma_memory_write(&address_space_memory, a + 8, le, 8,
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * The endpoint's next TRB the driver handed over (following links), or
 * false.  @t gets it, *at its address.
 */
static bool rk_dwc3_next_trb(RkDwc3State *s, int n, uint32_t t[4],
                             uint32_t *at)
{
    RkDwc3Ep *e = &s->ep[n];
    int hops;

    if (!e->started || e->stalled) {
        return false;
    }
    for (hops = 0; hops < 4; hops++) {
        rk_dwc3_trb_rd(e->trb, t);
        if (!(t[3] & TRB_HWO)) {
            return false;
        }
        if (TRB_TYPE(t[3]) != TRBT_LINK) {
            *at = e->trb;
            return true;
        }
        e->trb = t[0];
    }
    return false;
}

/*
 * The TRB at @at is done with @done bytes moved: HWO off, the rest in
 * size; an event if it asks for one or ends the transfer.
 */
static void rk_dwc3_trb_done(RkDwc3State *s, int n, uint32_t t[4],
                             uint32_t at, uint32_t done, bool is_short)
{
    uint32_t len = t[2] & 0xffffff;
    int st = 0;

    t[2] = (t[2] & ~0xffffffu) | (len - MIN(done, len));
    t[3] &= ~TRB_HWO;
    rk_dwc3_trb_wr(at, t);
    s->ep[n].trb = at + 16;
    if (is_short) {
        st |= EVST_SHORT;
    }
    if (t[3] & TRB_LST) {
        s->ep[n].started = false;
        rk_dwc3_ep_event(s, n, EV_COMPLETE, st | EVST_LST, 0);
    } else if ((t[3] & TRB_IOC) || (is_short && (t[3] & TRB_ISP_IMI))) {
        rk_dwc3_ep_event(s, n, EV_INPROGRESS, st | EVST_IOC, 0);
    }
}

/* Move up to @len bytes between @buf and the endpoint's TRBs: in = to us */
static int rk_dwc3_xfer(RkDwc3State *s, int n, uint8_t *buf, uint32_t len,
                        bool to_host, bool *ended)
{
    uint32_t t[4], at, moved = 0;

    *ended = false;
    while (rk_dwc3_next_trb(s, n, t, &at)) {
        uint32_t room = t[2] & 0xffffff, k = MIN(room, len - moved);
        uint64_t a = t[0] | (uint64_t)t[1] << 32;
        bool chn = t[3] & TRB_CHN;

        if (to_host) {
            dma_memory_read(&address_space_memory, a, buf + moved, k,
                            MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_write(&address_space_memory, a, buf + moved, k,
                             MEMTXATTRS_UNSPECIFIED);
        }
        moved += k;
        /*
         * A short packet ends the transfer: less than the TRB holds, or
         * the data running out mid-chain on a partial packet (the host's
         * last packet then is short, as a 24-byte adb header is)
         */
        if (chn && (k < room || (moved >= len && (moved % 512)))) {
            /*
             * A short packet ends the chain (CSP): this TRB is done
             * quietly, the rest of the chain is passed over and its last
             * TRB completes -- the driver's bounce TRB after an unaligned
             * OUT request carries the IOC.
             */
            uint32_t u[4], ua;

            t[2] = (t[2] & ~0xffffffu) | (room - k);
            t[3] &= ~TRB_HWO;
            rk_dwc3_trb_wr(at, t);
            for (ua = at + 16;; ua += 16) {
                rk_dwc3_trb_rd(ua, u);
                if (TRB_TYPE(u[3]) == TRBT_LINK) {
                    ua = u[0] - 16;
                    continue;
                }
                if (!(u[3] & TRB_CHN)) {
                    rk_dwc3_trb_done(s, n, u, ua, 0, true);
                    break;
                }
                u[3] &= ~TRB_HWO;
                rk_dwc3_trb_wr(ua, u);
            }
        } else {
            rk_dwc3_trb_done(s, n, t, at, k, k < room);
        }
        if (!chn || k < room || moved >= len) {
            *ended = true;
            break;
        }
    }
    return moved;
}

/* -- the host's control transfers -- */

/*
 * A control data stage: the TRBs up to the last one (the driver may chain
 * a bounce TRB after the real one), @len bytes at most, one XferComplete.
 */
static uint32_t rk_dwc3_ctl_data(RkDwc3State *s, int n, uint8_t *buf,
                                 uint32_t len, bool to_host)
{
    uint32_t t[4], at, moved = 0;

    while (rk_dwc3_next_trb(s, n, t, &at)) {
        uint32_t room = t[2] & 0xffffff, k = MIN(room, len - moved);
        uint64_t a = t[0] | (uint64_t)t[1] << 32;

        if (to_host) {
            dma_memory_read(&address_space_memory, a, buf + moved, k,
                            MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_write(&address_space_memory, a, buf + moved, k,
                             MEMTXATTRS_UNSPECIFIED);
        }
        moved += k;
        t[2] = (t[2] & ~0xffffffu) | (room - k);
        t[3] &= ~TRB_HWO;
        rk_dwc3_trb_wr(at, t);
        s->ep[n].trb = at + 16;
        if (!(t[3] & TRB_CHN)) {
            break;
        }
    }
    s->ep[n].started = false;
    rk_dwc3_ep_event(s, n, EV_COMPLETE, 0, 0);
    return moved;
}

static void rk_dwc3_ctl_start(RkDwc3State *s, uint8_t type, uint8_t req,
                              uint16_t value, uint16_t index, uint16_t len)
{
    uint8_t p[8] = { type, req, value, value >> 8, index, index >> 8,
                     len, len >> 8 };

    memcpy(s->setup, p, 8);
    s->ctl_in = type & 0x80;
    s->ctl_len = 0;
    s->hstate = H_SETUP;
}

/* A class request with an OUT data stage: @len bytes of @data */
static void rk_dwc3_ctl_out(RkDwc3State *s, uint8_t type, uint8_t req,
                            uint16_t value, uint16_t index,
                            const uint32_t *data, int words)
{
    int i;

    rk_dwc3_ctl_start(s, type, req, value, index, words * 4);
    for (i = 0; i < words; i++) {
        stl_le_p(&s->ctl[4 * i], data[i]);
    }
    s->ctl_len = words * 4;
}

/*
 * The configuration's functions: CDC ECM / RNDIS (a network) and CDC ACM
 * (a serial port), each a control interface with its notification
 * endpoint and a data interface after it with the bulk pair.
 */
static void rk_dwc3_scan(RkDwc3State *s)
{
    enum { K_NONE, K_ECM, K_RNDIS, K_ACM, K_ADB } kind = K_NONE;
    int data = K_NONE;
    uint32_t i = 0, len = s->ctl_len;
    int cur_if = -1, comm_if = -1;

    s->cfg_value = len > 5 ? s->ctl[5] : 1;
    s->ecm_data_if = s->rndis_if = s->acm_if = -1;
    s->ep_in = s->ep_out = s->acm.in = s->acm.out = -1;
    s->adb.in = s->adb.out = -1;
    s->n_notify = 0;
    while (i + 2 <= len && s->ctl[i] >= 2) {
        const uint8_t *d = &s->ctl[i];

        if (d[1] == 4 && d[0] >= 9) {               /* interface */
            cur_if = d[2];
            data = K_NONE;
            if (d[5] == 2 && d[6] == 6) {
                kind = K_ECM;
                comm_if = cur_if;
            } else if ((d[5] == 0xe0 && d[6] == 1 && d[7] == 3) ||
                       (d[5] == 2 && d[6] == 2 && d[7] == 0xff)) {
                kind = K_RNDIS;
                comm_if = cur_if;
                if (s->rndis_if < 0 && s->ecm_data_if < 0) {
                    s->rndis_if = cur_if;
                }
            } else if (d[5] == 2 && d[6] == 2) {
                kind = K_ACM;
                comm_if = cur_if;
                if (s->acm_if < 0) {
                    s->acm_if = cur_if;
                }
            } else if (d[5] == 0xff && d[6] == 0x42 && d[7] == 1 &&
                       s->adb.in < 0) {
                data = K_ADB;                       /* adb (FunctionFS) */
            } else if (d[5] == 0x0a && comm_if >= 0) {
                /* ECM's endpoints are in alternate setting 1, the
                 * others' in 0 */
                if (kind == K_ECM && d[3] == 1 && s->ecm_data_if < 0 &&
                    s->rndis_if < 0) {
                    s->ecm_data_if = cur_if;
                    data = K_ECM;
                } else if (kind == K_RNDIS && d[3] == 0 &&
                           comm_if == s->rndis_if) {
                    data = K_RNDIS;
                } else if (kind == K_ACM && d[3] == 0 &&
                           comm_if == s->acm_if) {
                    data = K_ACM;
                }
            }
        } else if (d[1] == 5 && d[0] >= 7) {        /* endpoint */
            int pn = ((d[2] & 0xf) << 1) | (d[2] >> 7);
            bool in = d[2] & 0x80;

            if ((d[3] & 3) == 3 && in && s->n_notify < 4) {
                s->ep_notify[s->n_notify++] = pn;
            } else if ((d[3] & 3) == 2 && (data == K_ECM ||
                                           data == K_RNDIS)) {
                *(in ? &s->ep_in : &s->ep_out) = pn;
            } else if ((d[3] & 3) == 2 && data == K_ACM) {
                *(in ? &s->acm.in : &s->acm.out) = pn;
            } else if ((d[3] & 3) == 2 && data == K_ADB) {
                *(in ? &s->adb.in : &s->adb.out) = pn;
            }
        }
        i += d[0];
    }
}

/* The next request of the enumeration, or the end of it */
static void rk_dwc3_enum_next(RkDwc3State *s)
{
    switch (s->step) {
    case STEP_DEV_DESC:
        rk_dwc3_ctl_start(s, 0x80, 6, 0x0100, 0, 64);
        break;
    case STEP_SET_ADDR:
        rk_dwc3_ctl_start(s, 0x00, 5, 1, 0, 0);
        break;
    case STEP_CFG9:
        rk_dwc3_ctl_start(s, 0x80, 6, 0x0200, 0, 9);
        break;
    case STEP_CFG: {
        uint16_t total = s->ctl_len >= 4 ? s->ctl[2] | s->ctl[3] << 8 : 9;

        rk_dwc3_ctl_start(s, 0x80, 6, 0x0200, 0,
                          MIN(total, sizeof(s->ctl)));
        break;
    }
    case STEP_SET_CFG:
        rk_dwc3_scan(s);
        rk_dwc3_ctl_start(s, 0x00, 9, s->cfg_value, 0, 0);
        break;
    case STEP_SET_IF:
        if (s->ecm_data_if >= 0 && s->rndis_if < 0) {
            rk_dwc3_ctl_start(s, 0x01, 11, 1, s->ecm_data_if, 0);
            break;
        }
        s->step++;
        /* fall through */
    case STEP_RNDIS_INIT:
        if (s->rndis_if >= 0) {
            /* REMOTE_NDIS_INITIALIZE_MSG: v1.0, 16 KiB transfers */
            static const uint32_t init[] = { 2, 24, 1, 1, 0, 0x4000 };

            rk_dwc3_ctl_out(s, 0x21, 0, 0, s->rndis_if, init, 6);
            break;
        }
        s->step = STEP_ACM_DTR;
        /* fall through */
    case STEP_ACM_DTR:
        if (s->acm_if >= 0 && s->step == STEP_ACM_DTR) {
            /* SET_CONTROL_LINE_STATE: DTR and RTS, the port opened */
            rk_dwc3_ctl_start(s, 0x21, 0x22, 3, s->acm_if, 0);
            break;
        }
        s->step = STEP_DONE;
        s->hstate = H_RUN;
        break;
    case STEP_RNDIS_INIT_RSP:
    case STEP_RNDIS_FILTER_RSP:
        /* GET_ENCAPSULATED_RESPONSE: the gadget queued it at once */
        rk_dwc3_ctl_start(s, 0xa1, 1, 0, s->rndis_if, sizeof(s->ctl));
        break;
    case STEP_RNDIS_FILTER: {
        /* REMOTE_NDIS_SET_MSG OID_GEN_CURRENT_PACKET_FILTER: directed,
         * multicast, all multicast, broadcast -- the gadget's carrier on */
        static const uint32_t set[] = { 5, 32, 2, 0x0001010e, 4, 20, 0,
                                        0x0f };

        rk_dwc3_ctl_out(s, 0x21, 0, 0, s->rndis_if, set, 8);
        break;
    }
    default:
        s->hstate = H_RUN;
        break;
    }
}

/* Endpoint 0: one step of the current control transfer, if it can go */
static void rk_dwc3_ctl_step(RkDwc3State *s)
{
    uint16_t wlen = s->setup[6] | s->setup[7] << 8;
    uint32_t t[4], at;
    int sn;

    switch (s->hstate) {
    case H_SETUP:
        if (!rk_dwc3_next_trb(s, 0, t, &at)) {
            return;
        }
        dma_memory_write(&address_space_memory,
                         t[0] | (uint64_t)t[1] << 32, s->setup, 8,
                         MEMTXATTRS_UNSPECIFIED);
        t[2] &= ~0xffffffu;
        t[3] &= ~TRB_HWO;
        rk_dwc3_trb_wr(at, t);
        s->ep[0].started = false;
        s->hstate = wlen ? H_DATA : H_STATUS;
        rk_dwc3_ep_event(s, 0, EV_COMPLETE, 0, 0);
        if (!wlen) {
            /* two stages: the status is IN */
            rk_dwc3_ep_event(s, 1, EV_NOTREADY, EVST_CTRL_STATUS, 0);
        }
        return;
    case H_DATA:
        sn = s->ctl_in ? 1 : 0;
        if (!rk_dwc3_next_trb(s, sn, t, &at)) {
            return;
        }
        if (s->ctl_in) {
            s->ctl_len = rk_dwc3_ctl_data(s, sn, s->ctl,
                                          MIN(wlen, sizeof(s->ctl)), true);
        } else {
            rk_dwc3_ctl_data(s, sn, s->ctl, MIN(s->ctl_len, wlen), false);
        }
        s->hstate = H_STATUS;
        rk_dwc3_ep_event(s, s->ctl_in ? 0 : 1, EV_NOTREADY,
                         EVST_CTRL_STATUS, 0);
        return;
    case H_STATUS:
        sn = (wlen && s->ctl_in) ? 0 : 1;
        if (!rk_dwc3_next_trb(s, sn, t, &at)) {
            return;
        }
        t[3] &= ~TRB_HWO;
        rk_dwc3_trb_wr(at, t);
        s->ep[sn].started = false;
        rk_dwc3_ep_event(s, sn, EV_COMPLETE, 0, 0);
        if (s->setup[1] == 5 && !s->setup[0]) {
            REG(s, R_DCFG) = (REG(s, R_DCFG) & ~(0x7f << 3)) |
                             (s->setup[2] & 0x7f) << 3;
        }
        s->step++;
        rk_dwc3_enum_next(s);
        return;
    }
}

/* RNDIS: the frames in the REMOTE_NDIS_PACKET_MSGs of one transfer */
static void rk_dwc3_rndis_tx(RkDwc3State *s)
{
    uint32_t off = 0;

    while (off + 44 <= s->tx_len) {
        const uint8_t *m = &s->tx[off];
        uint32_t mlen = ldl_le_p(m + 4), doff = ldl_le_p(m + 8);
        uint32_t dlen = ldl_le_p(m + 12);

        if (ldl_le_p(m) != 1 || mlen < 44 || off + mlen > s->tx_len) {
            break;
        }
        if (8 + doff + dlen <= mlen) {
            qemu_send_packet(qemu_get_queue(s->nic), m + 8 + doff, dlen);
        }
        off += mlen;
    }
}

/* A byte stream: what the gadget sends out, what waits for it in */
static void rk_dwc3_pipe_step(RkDwc3State *s, RkDwc3Pipe *p)
{
    uint8_t b[4096];
    uint32_t n, t[4], at;
    bool ended;

    if (p->in >= 0) {
        while ((n = rk_dwc3_xfer(s, p->in, b, sizeof(b), true, &ended)) > 0) {
            qemu_chr_fe_write_all(&p->chr, b, n);
        }
    }
    while (p->out >= 0 && p->rx_n && rk_dwc3_next_trb(s, p->out, t, &at)) {
        /* one transfer per request: no more than its first TRB holds (a
         * chained TRB after it is the driver's bounce buffer) */
        n = rk_dwc3_xfer(s, p->out, p->rx, MIN(p->rx_n, t[2] & 0xffffff),
                         false, &ended);
        memmove(p->rx, p->rx + n, p->rx_n - n);
        p->rx_n -= n;
        qemu_chr_fe_accept_input(&p->chr);
        if (!n) {
            break;
        }
    }
}

/* Bulk data, once enumerated: frames both ways, notifications dropped */
static void rk_dwc3_data_step(RkDwc3State *s)
{
    uint8_t junk[64];
    bool ended;

    int k;

    for (k = 0; k < s->n_notify; k++) {
        rk_dwc3_xfer(s, s->ep_notify[k], junk, sizeof(junk), true, &ended);
    }
    rk_dwc3_pipe_step(s, &s->acm);
    rk_dwc3_pipe_step(s, &s->adb);
    if (s->ep_in >= 0) {
        for (;;) {
            uint32_t n = rk_dwc3_xfer(s, s->ep_in, s->tx + s->tx_len,
                                      MAX_FRAME - s->tx_len, true, &ended);
            s->tx_len += n;
            if (!ended) {
                break;
            }
            if (s->tx_len && s->nic && s->rndis_if >= 0) {
                rk_dwc3_rndis_tx(s);
            } else if (s->tx_len && s->nic) {
                qemu_send_packet(qemu_get_queue(s->nic), s->tx, s->tx_len);
            }
            s->tx_len = 0;
        }
    }
    while (s->ep_out >= 0 && s->rxq_n) {
        uint32_t t[4], at;

        if (!rk_dwc3_next_trb(s, s->ep_out, t, &at)) {
            break;
        }
        rk_dwc3_xfer(s, s->ep_out, s->rxq[s->rxq_head],
                     s->rxq_len[s->rxq_head], false, &ended);
        s->rxq_head = (s->rxq_head + 1) % RXQ;
        s->rxq_n--;
    }
    if (s->nic && s->rxq_n < RXQ) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

static void rk_dwc3_tick(void *opaque)
{
    RkDwc3State *s = opaque;

    if (!(REG(s, R_DCTL) & DCTL_RUN)) {
        return;
    }
    if (s->hstate == H_RUN) {
        rk_dwc3_data_step(s);
    } else if (s->hstate != H_IDLE) {
        rk_dwc3_ctl_step(s);
    }
    timer_mod(s->tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TICK_NS);
}

/* RUN set: the host resets the bus and starts over */
static void rk_dwc3_connect(RkDwc3State *s)
{
    s->tx_len = 0;
    s->rxq_n = 0;
    s->step = STEP_DEV_DESC;
    rk_dwc3_event(s, 1 | DEVEV_RESET << 8);
    rk_dwc3_event(s, 1 | DEVEV_CONNDONE << 8);
    rk_dwc3_enum_next(s);
    timer_mod(s->tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TICK_NS);
}

static void rk_dwc3_ep_cmd(RkDwc3State *s, int n, uint32_t val)
{
    RkDwc3Ep *e = &s->ep[n];
    uint32_t ret = val & ~CMDACT & ~(0xf << 12);

    switch (val & 0xf) {
    case CMD_START:
        e->trb = REG(s, R_DEP(n) + 4);      /* PAR1: the TRB, low half */
        e->started = true;
        e->stalled = false;
        ret = (ret & 0xffff) | (uint32_t)(n + 1) << 16;   /* resource */
        break;
    case CMD_UPDATE:
        break;
    case CMD_END:
        e->started = false;
        if (val & CMDIOC) {
            rk_dwc3_ep_event(s, n, EV_CMDCMPLT, 0, CMD_END << 8);
        }
        break;
    case CMD_SETSTALL:
        if (n < 2 && s->hstate != H_RUN && s->hstate != H_IDLE) {
            /* the control transfer failed: on to the next request */
            s->step++;
            rk_dwc3_enum_next(s);
        } else {
            e->stalled = true;
        }
        break;
    case 5:                                     /* CLEARSTALL */
        e->stalled = false;
        break;
    }
    REG(s, R_DEP(n) + 0xc) = ret;
    if (e->started && s->hstate != H_IDLE) {
        timer_mod(s->tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TICK_NS);
    }
}

static uint64_t rk_dwc3_read(void *opaque, hwaddr off, unsigned size)
{
    RkDwc3State *s = opaque;

    switch (off) {
    case R_GEVNTCOUNT:
        return s->ev_count;
    case R_DSTS:
        return (REG(s, R_DCTL) & DCTL_RUN) ? 0 : DSTS_HALTED;
    }
    return s->regs[off / 4];
}

static void rk_dwc3_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    RkDwc3State *s = opaque;
    uint32_t was;

    switch (off) {
    case R_GSNPSID:
    case R_GHWPARAMS0 ... R_GHWPARAMS0 + 0x1c:
    case R_DSTS:
        return;
    case R_GEVNTCOUNT:
        s->ev_count -= MIN(s->ev_count, val & 0xfffc);
        rk_dwc3_update(s);
        return;
    case R_GEVNTADRLO:
        s->ev_head = 0;
        s->ev_count = 0;
        break;
    case R_DCTL:
        was = REG(s, R_DCTL);
        REG(s, R_DCTL) = val & ~DCTL_CSFTRST;
        if ((val & DCTL_RUN) && !(was & DCTL_RUN)) {
            rk_dwc3_connect(s);
        } else if (!(val & DCTL_RUN)) {
            s->hstate = H_IDLE;
        }
        return;
    case R_DGCMD:
        REG(s, off) = val & ~CMDACT;
        return;
    }
    s->regs[off / 4] = val;
    if (off >= R_DEP(0) && off < R_DEP(NEPS) && (off & 0xf) == 0xc &&
        (val & CMDACT)) {
        rk_dwc3_ep_cmd(s, (off - R_DEP(0)) / 0x10, val);
    }
    if (off == R_GEVNTSIZ) {
        rk_dwc3_update(s);
    }
}

static const MemoryRegionOps rk_dwc3_ops = {
    .read = rk_dwc3_read,
    .write = rk_dwc3_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static bool rk_dwc3_can_receive(NetClientState *nc)
{
    RkDwc3State *s = qemu_get_nic_opaque(nc);

    return s->hstate == H_RUN && s->ep_out >= 0 && s->rxq_n < RXQ;
}

static ssize_t rk_dwc3_receive(NetClientState *nc, const uint8_t *buf,
                               size_t len)
{
    RkDwc3State *s = qemu_get_nic_opaque(nc);
    int k;

    if (!rk_dwc3_can_receive(nc)) {
        return 0;
    }
    if (len > MAX_FRAME) {
        return len;                             /* dropped */
    }
    k = (s->rxq_head + s->rxq_n) % RXQ;
    if (s->rndis_if >= 0) {
        /* REMOTE_NDIS_PACKET_MSG: the data 36 bytes after DataOffset */
        if (len + 44 > MAX_FRAME) {
            return len;
        }
        memset(s->rxq[k], 0, 44);
        stl_le_p(s->rxq[k], 1);
        stl_le_p(s->rxq[k] + 4, 44 + len);
        stl_le_p(s->rxq[k] + 8, 36);
        stl_le_p(s->rxq[k] + 12, len);
        memcpy(s->rxq[k] + 44, buf, len);
        s->rxq_len[k] = 44 + len;
    } else {
        memcpy(s->rxq[k], buf, len);
        s->rxq_len[k] = len;
    }
    s->rxq_n++;
    return len;
}

static NetClientInfo rk_dwc3_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = rk_dwc3_can_receive,
    .receive = rk_dwc3_receive,
};

static void rk_dwc3_reset(DeviceState *dev)
{
    RkDwc3State *s = RK_DWC3(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    REG(s, R_GSNPSID) = 0x5533290a;             /* 2.90a */
    REG(s, R_GHWPARAMS0) = 0x40 << 8;           /* device only, 64-bit */
    REG(s, R_GHWPARAMS1) = 1 << 24;
    REG(s, R_GHWPARAMS3) = 8 << 18 | NEPS << 12;
    for (i = 0; i < 8; i++) {
        REG(s, R_GTXFIFOSIZ(i)) = 0x184;
    }
    REG(s, R_GRXFIFOSIZ0) = 0x285;
    REG(s, R_DSTS) = DSTS_HALTED;
    memset(s->ep, 0, sizeof(s->ep));
    s->ev_head = s->ev_count = 0;
    s->hstate = H_IDLE;
    s->ep_in = s->ep_out = s->ecm_data_if = -1;
    s->rndis_if = s->acm_if = s->acm.in = s->acm.out = -1;
    s->adb.in = s->adb.out = -1;
    s->adb.rx_n = 0;
    s->n_notify = 0;
    s->acm.rx_n = 0;
    s->tx_len = 0;
    s->rxq_n = s->rxq_head = 0;
    timer_del(s->tick);
    rk_dwc3_update(s);
}

/* A pipe's input: as much as its buffer takes */
static int rk_dwc3_chr_can_read(void *opaque)
{
    RkDwc3Pipe *p = opaque;

    return p->out >= 0 ? sizeof(p->rx) - p->rx_n : 0;
}

static void rk_dwc3_chr_read(void *opaque, const uint8_t *buf, int size)
{
    RkDwc3Pipe *p = opaque;

    size = MIN(size, (int)(sizeof(p->rx) - p->rx_n));
    memcpy(p->rx + p->rx_n, buf, size);
    p->rx_n += size;
}

static void rk_dwc3_realize(DeviceState *dev, Error **errp)
{
    RkDwc3State *s = RK_DWC3(dev);

    qemu_chr_fe_set_handlers(&s->acm.chr, rk_dwc3_chr_can_read,
                             rk_dwc3_chr_read, NULL, NULL, &s->acm, NULL, true);
    qemu_chr_fe_set_handlers(&s->adb.chr, rk_dwc3_chr_can_read,
                             rk_dwc3_chr_read, NULL, NULL, &s->adb, NULL, true);

    memory_region_init_io(&s->iomem, OBJECT(s), &rk_dwc3_ops, s,
                          TYPE_RK_DWC3, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->tick = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_dwc3_tick, s);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&rk_dwc3_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
}

static const Property rk_dwc3_properties[] = {
    DEFINE_NIC_PROPERTIES(RkDwc3State, conf),
    DEFINE_PROP_CHR("chardev", RkDwc3State, acm.chr),
    DEFINE_PROP_CHR("adb-chardev", RkDwc3State, adb.chr),
};

static const VMStateDescription rk_dwc3_vmstate = {
    .name = TYPE_RK_DWC3,
    .unmigratable = 1,
};

static void rk_dwc3_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_dwc3_realize;
    device_class_set_legacy_reset(dc, rk_dwc3_reset);
    device_class_set_props(dc, rk_dwc3_properties);
    dc->vmsd = &rk_dwc3_vmstate;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo rk_dwc3_info = {
    .name = TYPE_RK_DWC3,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkDwc3State),
    .class_init = rk_dwc3_class_init,
};

static void rk_dwc3_register_types(void)
{
    type_register_static(&rk_dwc3_info);
}

type_init(rk_dwc3_register_types)
