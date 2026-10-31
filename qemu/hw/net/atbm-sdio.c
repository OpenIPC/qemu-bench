/*
 * AltoBeam ATBM6031/Apollo SDIO Wi-Fi, as the vendor's atbm_wlan driver
 * (hal_apollo, PLATFORM_INGENICT31/T41) drives it on the Ingenic MSC.
 *
 * The chip is an SDIO function-1 device with its own LMAC CPU.  The driver
 *   - enumerates it over the SD bus (CMD5/CMD3/CMD7, then CCCR/FBR/CIS and
 *     CMD52/CMD53 to function 1);
 *   - reaches eight HIF registers through function-1 CMD53, by a 17-bit
 *     address (buf_id << 6 | reg << 2): CONFIG, CONTROL, the in/out queue,
 *     the AHB and SRAM data ports with their base-address registers;
 *   - downloads firmware into the chip's ITCM/DTCM over the AHB port,
 *     releases the CPU, and then speaks WSM (the ST-Ericsson CW1200 host
 *     interface) over the queue.
 *
 * Here no firmware runs: the device answers WSM itself and emulates an
 * access point, bridging data frames to a QEMU netdev.  This is the SDIO
 * sibling of hw/usb/atbm-wifi.c and shares its WSM/AP engine.  Stage 1 of
 * the model (this file as first committed) implements only the SDIO
 * transport and enumeration; the HIF registers and WSM come next.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_ATBM_SDIO "atbm-sdio"
OBJECT_DECLARE_TYPE(AtbmSdioState, AtbmSdioClass, ATBM_SDIO)

struct AtbmSdioClass {
    SDCardClass parent_class;
};

/* SDIO command set (on top of the SD physical commands) */
#define CMD_GO_IDLE             0
#define CMD_SEND_RELATIVE_ADDR  3
#define CMD_IO_SEND_OP_COND     5
#define CMD_SELECT_CARD         7
#define CMD_SEND_IF_COND        8
#define CMD_IO_RW_DIRECT        52
#define CMD_IO_RW_EXTENDED      53

/* CCCR (function 0) */
#define CCCR_SIZE       0x20
#define FBR1            0x100       /* function 1 basic registers */
#define CIS_COMMON      0x1000
#define CIS_FUNC1       0x1100
#define FN1_WINDOW      0x10000     /* function-1 I/O space, 17-bit */

#define ATBM_VID        0x007a
#define ATBM_PID        0x6011

enum atbm_sd_state {
    ATBM_IDLE,
    ATBM_READY,
    ATBM_STANDBY,
    ATBM_TRANSFER,
};

struct AtbmSdioState {
    DeviceState parent_obj;

    SDBus *sdbus;               /* the bus this card sits on */
    enum atbm_sd_state state;
    uint16_t rca;
    bool powered;

    uint8_t cccr[CCCR_SIZE];
    uint8_t fn_enable;          /* CCCR 0x02 */
    uint8_t fn_ready;           /* CCCR 0x03 */
    uint8_t int_enable;         /* CCCR 0x04 */
    uint8_t bus_width;          /* CCCR 0x07 */
    uint16_t fn1_block_size;    /* FBR1 0x110/0x111 */

    /* a pending CMD53 data phase */
    bool     io_rw;             /* true: write */
    uint8_t  io_fn;
    bool     io_incr;
    uint32_t io_addr;
    uint32_t io_left;           /* bytes still to move */

    uint8_t fn1[FN1_WINDOW];    /* function-1 I/O space (storage for now) */
};

/* ---- CIS: the card information structure the SDIO core walks --------- */

static void atbm_build_cis(AtbmSdioState *s)
{
    /* Common CIS at CIS_COMMON: MANFID then END */
    uint8_t *c = s->fn1 + CIS_COMMON;
    int i = 0;

    c[i++] = 0x20;              /* CISTPL_MANFID */
    c[i++] = 4;
    c[i++] = ATBM_VID & 0xff;
    c[i++] = ATBM_VID >> 8;
    c[i++] = ATBM_PID & 0xff;
    c[i++] = ATBM_PID >> 8;
    c[i++] = 0xff;             /* CISTPL_END */

    /* Function-1 CIS at CIS_FUNC1: FUNCE (max block size 512) then END */
    c = s->fn1 + CIS_FUNC1;
    i = 0;
    c[i++] = 0x22;             /* CISTPL_FUNCE */
    c[i++] = 6;
    c[i++] = 0x01;             /* extended data: function extension */
    c[i++] = 0x00;             /* wakeup, etc. */
    c[i++] = 0x00;
    c[i++] = 0x02;             /* max block size 512 (0x0200) */
    c[i++] = 0x00;
    c[i++] = 0x00;
    c[i++] = 0xff;             /* CISTPL_END */
}

static void atbm_cccr_reset(AtbmSdioState *s)
{
    memset(s->cccr, 0, sizeof(s->cccr));
    s->cccr[0x00] = 0x43;       /* CCCR/SDIO revision 3.00 */
    s->cccr[0x08] = 0xc2;       /* card capability: 4-bit, low-speed... */
    /* common CIS pointer (0x09..0x0b) */
    s->cccr[0x09] = CIS_COMMON & 0xff;
    s->cccr[0x0a] = (CIS_COMMON >> 8) & 0xff;
    s->cccr[0x0b] = (CIS_COMMON >> 16) & 0xff;
    s->fn_enable = s->fn_ready = s->int_enable = 0;
    s->bus_width = 0;
    s->fn1_block_size = 0;
    /* FBR for function 1 */
    s->fn1[FBR1 + 0x00] = 0x0c; /* standard SDIO function, class 12 */
    s->fn1[FBR1 + 0x09] = CIS_FUNC1 & 0xff;
    s->fn1[FBR1 + 0x0a] = (CIS_FUNC1 >> 8) & 0xff;
    s->fn1[FBR1 + 0x0b] = (CIS_FUNC1 >> 16) & 0xff;
}

/* ---- function-0 (CCCR/FBR/CIS) register access ----------------------- */

static uint8_t atbm_fn0_read(AtbmSdioState *s, uint32_t addr)
{
    switch (addr) {
    case 0x02: return s->fn_enable;
    case 0x03: return s->fn_ready;
    case 0x04: return s->int_enable;
    case 0x07: return s->bus_width;
    case 0x110: return s->fn1_block_size & 0xff;
    case 0x111: return s->fn1_block_size >> 8;
    }
    if (addr < CCCR_SIZE) {
        return s->cccr[addr];
    }
    if (addr < FN1_WINDOW) {
        return s->fn1[addr];
    }
    return 0;
}

static void atbm_fn0_write(AtbmSdioState *s, uint32_t addr, uint8_t val)
{
    switch (addr) {
    case 0x02:
        s->fn_enable = val;
        s->fn_ready = val;      /* a function enables at once */
        return;
    case 0x04:
        s->int_enable = val;
        return;
    case 0x06:                  /* I/O abort / reset */
        return;
    case 0x07:
        s->bus_width = val;
        return;
    case 0x110:
        s->fn1_block_size = (s->fn1_block_size & 0xff00) | val;
        return;
    case 0x111:
        s->fn1_block_size = (s->fn1_block_size & 0xff) | (val << 8);
        return;
    }
    if (addr >= CCCR_SIZE && addr < FN1_WINDOW) {
        s->fn1[addr] = val;
    }
}

/* ---- function-1 (the chip's I/O window) access ----------------------- */
/* Stage 1: plain storage.  The HIF/WSM engine replaces this next. */

static uint8_t atbm_fn1_read(AtbmSdioState *s, uint32_t addr)
{
    return addr < FN1_WINDOW ? s->fn1[addr] : 0;
}

static void atbm_fn1_write(AtbmSdioState *s, uint32_t addr, uint8_t val)
{
    if (addr < FN1_WINDOW) {
        s->fn1[addr] = val;
    }
}

static uint8_t atbm_io_read(AtbmSdioState *s, uint8_t fn, uint32_t addr)
{
    return fn == 0 ? atbm_fn0_read(s, addr) : atbm_fn1_read(s, addr);
}

static void atbm_io_write(AtbmSdioState *s, uint8_t fn, uint32_t addr,
                          uint8_t val)
{
    if (fn == 0) {
        atbm_fn0_write(s, addr, val);
    } else {
        atbm_fn1_write(s, addr, val);
    }
}

/* ---- SD bus card methods -------------------------------------------- */

/* Build a 4-byte big-endian response from a 32-bit value. */
static size_t atbm_resp32(uint8_t *resp, uint32_t v)
{
    resp[0] = v >> 24;
    resp[1] = v >> 16;
    resp[2] = v >> 8;
    resp[3] = v;
    return 4;
}

static size_t atbm_do_command(SDState *sd, SDRequest *req, uint8_t *resp,
                              size_t respsz)
{
    AtbmSdioState *s = ATBM_SDIO(sd);
    uint32_t arg = req->arg;

    s->io_left = 0;
    switch (req->cmd) {
    case CMD_GO_IDLE:
        s->state = ATBM_IDLE;
        return 0;
    case CMD_SEND_IF_COND:
        return 0;               /* not an SD memory card */
    case CMD_IO_SEND_OP_COND:
        s->powered = true;
        s->state = ATBM_READY;
        /* R4: ready, one function, the voltage window we accept */
        return atbm_resp32(resp, 0x80000000 | (1u << 28) | 0x00300000);
    case CMD_SEND_RELATIVE_ADDR:
        s->rca = 1;
        s->state = ATBM_STANDBY;
        /* R6: RCA, status */
        return atbm_resp32(resp, (uint32_t)s->rca << 16);
    case CMD_SELECT_CARD:
        s->state = (arg >> 16) == s->rca ? ATBM_TRANSFER : ATBM_STANDBY;
        return atbm_resp32(resp, 0x1e00);       /* R1b */
    case CMD_IO_RW_DIRECT: {
        /* arg: 31 R/W, 30:28 fn, 27 RAW, 25:9 addr, 7:0 data */
        bool write = arg & (1u << 31);
        uint8_t fn = (arg >> 28) & 7;
        uint32_t addr = (arg >> 9) & 0x1ffff;
        uint8_t data = arg & 0xff;
        uint8_t out;

        if (write) {
            atbm_io_write(s, fn, addr, data);
            out = (arg & (1u << 27)) ? atbm_io_read(s, fn, addr) : data;
        } else {
            out = atbm_io_read(s, fn, addr);
        }
        /* R5: flags (IO current state 1) in [15:8], data in [7:0] */
        return atbm_resp32(resp, (0x10 << 8) | out);
    }
    case CMD_IO_RW_EXTENDED: {
        /* arg: 31 R/W, 30:28 fn, 27 block, 26 incr, 25:9 addr, 8:0 count */
        bool block = arg & (1u << 27);
        uint32_t count = arg & 0x1ff;
        uint32_t bytes;

        s->io_rw = arg & (1u << 31);
        s->io_fn = (arg >> 28) & 7;
        s->io_incr = arg & (1u << 26);
        s->io_addr = (arg >> 9) & 0x1ffff;
        if (block) {
            bytes = (count ? count : 512) * s->fn1_block_size;
        } else {
            bytes = count ? count : 512;
        }
        s->io_left = bytes;
        return atbm_resp32(resp, (0x10 << 8));  /* R5 */
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
    atbm_io_write(s, s->io_fn, s->io_addr, value);
    if (s->io_incr) {
        s->io_addr++;
    }
    s->io_left--;
}

static uint8_t atbm_read_byte(SDState *sd)
{
    AtbmSdioState *s = ATBM_SDIO(sd);
    uint8_t v;

    if (!s->io_left || s->io_rw) {
        return 0;
    }
    v = atbm_io_read(s, s->io_fn, s->io_addr);
    if (s->io_incr) {
        s->io_addr++;
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
    atbm_cccr_reset(s);
    atbm_build_cis(s);
}

static void atbm_sdio_reset(DeviceState *dev)
{
    AtbmSdioState *s = ATBM_SDIO(dev);

    s->state = ATBM_IDLE;
    s->rca = 0;
    s->powered = false;
    s->io_left = 0;
    memset(s->fn1, 0, sizeof(s->fn1));
    atbm_cccr_reset(s);
    atbm_build_cis(s);
}

static const VMStateDescription vmstate_atbm_sdio = {
    .name = TYPE_ATBM_SDIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(state, AtbmSdioState),
        VMSTATE_UINT16(rca, AtbmSdioState),
        VMSTATE_UINT8(fn_enable, AtbmSdioState),
        VMSTATE_UINT8(fn_ready, AtbmSdioState),
        VMSTATE_UINT8(int_enable, AtbmSdioState),
        VMSTATE_UINT8(bus_width, AtbmSdioState),
        VMSTATE_UINT16(fn1_block_size, AtbmSdioState),
        VMSTATE_UINT8_ARRAY(fn1, AtbmSdioState, FN1_WINDOW),
        VMSTATE_END_OF_LIST()
    }
};

static void atbm_sdio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = (SDCardClass *)klass;

    dc->realize = atbm_sdio_realize;
    device_class_set_legacy_reset(dc, atbm_sdio_reset);
    dc->vmsd = &vmstate_atbm_sdio;
    dc->bus_type = TYPE_SD_BUS;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

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
