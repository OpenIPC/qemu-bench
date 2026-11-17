/*
 * Xiongmai / XMedia "lotus,aiao": the audio in/out DMA of the GK7206V11A
 * (XM SPC030 / xmorca die), 64 KiB at 0x100e0000, one interrupt (SPI 47,
 * "AIO Interrupt").  The vendor's xm_aio.ko HAL (hal_ai_* / hal_ao_*)
 * programs it, xm_ai.ko / xm_ao.ko take the interrupts.  Not HiSilicon's
 * V4 AIAO (hisi-aiao.c): one capture and one playback channel, buffer
 * pointers are bus addresses, other offsets throughout.
 *
 * Global block:
 *   +0x000  version (reads 0x20230430 on a GK7206V11A)
 *   +0x004  enables: bit n capture channel n, bit 8 + n playback channel n,
 *           bit 16 capture, bit 17 playback
 *   +0x008  interrupt status (read only): bit n capture channel n has a
 *           masked bit pending, bit 8 + n playback channel n
 *   +0x020, +0x024  slot maps (bits 0-3 written by the HAL)
 *   +0x100 + 8n capture channel n's clocks, +0x140 + 8n playback's:
 *           first word bits 0-26 an NCO, linear in the sample rate
 *           (capture 0x44444 = 8 kHz ... 0x199999 = 48 kHz, the HAL's
 *           table; playback reads half that, 0x22222 at 8 kHz), second word
 *           bits 0-3 / 4-7 bit clock and frame dividers, bits 8-10 gates
 * Channel block, capture n at +0x1000 + 0x100n, playback at +0x2000:
 *   +0x00  interface (bits 5-6 sample width: 1 is 16-bit, 2 is 32-bit;
 *          bits 8-17 a slot setting; bits 21-23 track mode)
 *   +0x04  bits 0-2 start (the HAL sets all three, clears them to stop)
 *   +0x08  buffer bus address, +0x0c high word (0), +0x10 buffer size
 *   +0x18  write pointer, +0x1c read pointer: BUS ADDRESSES inside the
 *          buffer.  Capture: hardware moves the write pointer, the driver
 *          the read pointer; playback the other way round.
 *   +0x24  bytes a transfer (one interrupt each)
 *   +0x28  hardware's last transfer boundary (bus address)
 *   +0x2c  interrupt enable (the HAL writes 0x2b), +0x30 raw status,
 *          +0x34 masked status, +0x38 write 1 to clear.  Bit 0 a transfer
 *          done; bit 1 capture buffer full / playback buffer empty;
 *          bits 3 and 5 errors (only counted by the drivers).
 *   +0x3c  bit 0 idle: after a stop the HAL polls it to 1 (10 ms steps,
 *          gives up after ~1 s); running it reads 0x18 (capture) / 0x2
 *
 * Seen on a real GK7206V11A running the stock XM app: 8 kHz mono, 32-bit
 * ADC samples, 1280 bytes a transfer, a 3840-byte capture and 5120-byte
 * playback buffer, an interrupt per channel every 40 ms; playback keeps
 * interrupting when there is nothing to play.
 *
 * Model: a transfer at a time, every transfer / byte rate of virtual time.
 * Capture writes a sine of "tone" Hz (0 = silence); playback consumes
 * (and drops) a transfer when one is there, else raises "empty".
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include <math.h>

#define TYPE_XM_AIAO "xm-aiao"
OBJECT_DECLARE_SIMPLE_TYPE(XmAiaoState, XM_AIAO)

#define AIAO_SIZE       0x10000
#define G_VERSION       0x000
#define G_ENABLE        0x004
#define G_INT_STA       0x008
#define G_RX_CRG        0x100
#define G_TX_CRG        0x140
#define RX_BASE         0x1000
#define TX_BASE         0x2000

#define CH_ATTR         0x00
#define CH_CTRL         0x04
#define CH_ADDR         0x08
#define CH_ADDR_HI      0x0c
#define CH_SIZE         0x10
#define CH_WPTR         0x18
#define CH_RPTR         0x1c
#define CH_TRANS        0x24
#define CH_LAST         0x28
#define CH_INT_ENA      0x2c
#define CH_INT_RAW      0x30
#define CH_INT_STA      0x34
#define CH_INT_CLR      0x38
#define CH_BUSY         0x3c

#define CTRL_START      (1u << 0)
#define INT_TRANS       (1u << 0)
#define INT_FULL        (1u << 1)     /* capture full / playback empty */

#define XFER_MAX        65536
#define NCO_48K         0x199999

typedef struct XmAiaoState XmAiaoState;

typedef struct XmAiaoChn {
    XmAiaoState *s;
    hwaddr base;
    hwaddr crg;
    bool tx;
    uint32_t sta_bit;           /* in G_INT_STA */
    QEMUTimer *timer;
    uint32_t rate;
    uint32_t width;             /* bytes a sample */
    uint64_t frames;
} XmAiaoChn;

struct XmAiaoState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[AIAO_SIZE / 4];
    XmAiaoChn chn[2];           /* capture 0, playback 0 */
    uint32_t tone;
    bool trace;
};

#define REG(s, off)     ((s)->regs[(off) / 4])
#define CREG(c, off)    REG((c)->s, (c)->base + (off))

static uint32_t xm_aiao_sta(XmAiaoState *s)
{
    uint32_t sta = 0;

    for (int n = 0; n < 2; n++) {
        XmAiaoChn *c = &s->chn[n];

        if (CREG(c, CH_INT_RAW) & CREG(c, CH_INT_ENA)) {
            sta |= c->sta_bit;
        }
    }
    return sta;
}

static void xm_aiao_update(XmAiaoState *s)
{
    qemu_set_irq(s->irq, xm_aiao_sta(s) != 0);
}

static uint32_t xm_aiao_rate(XmAiaoChn *c)
{
    const uint64_t nco = extract32(REG(c->s, c->crg), 0, 27);
    /* playback's NCO runs at half capture's for the same rate: a real
     * GK7206V11A at 8 kHz both ways reads 0x44444 / 0x22222 */
    const uint64_t scale = c->tx ? 96000 : 48000;
    const uint64_t fs = (nco * scale + NCO_48K / 2) / NCO_48K;

    if (fs < 1000 || fs > 192000) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-aiao: %s NCO 0x%" PRIx64
                      " gives %" PRIu64 " Hz, using 8000\n",
                      c->tx ? "playback" : "capture", nco, fs);
        return 8000;
    }
    return fs;
}

static uint32_t xm_aiao_width(XmAiaoChn *c)
{
    static const uint32_t widths[4] = { 1, 2, 4, 4 };

    return widths[extract32(CREG(c, CH_ATTR), 5, 2)];
}

static int64_t xm_aiao_period(XmAiaoChn *c)
{
    const uint32_t trans = CREG(c, CH_TRANS);

    return MAX(NANOSECONDS_PER_SECOND * trans /
               ((uint64_t)c->rate * c->width), 100000);
}

static bool xm_aiao_running(XmAiaoChn *c)
{
    const uint32_t trans = CREG(c, CH_TRANS);
    const uint32_t size = CREG(c, CH_SIZE);

    return (CREG(c, CH_CTRL) & CTRL_START) && trans && trans <= XFER_MAX &&
           size && trans <= size;
}

/* a pointer register as an offset in the buffer; out of range is 0 */
static uint32_t xm_aiao_off(XmAiaoChn *c, hwaddr reg)
{
    const uint32_t a = CREG(c, reg) - CREG(c, CH_ADDR);

    return a < CREG(c, CH_SIZE) ? a : 0;
}

static void xm_aiao_capture(XmAiaoChn *c)
{
    const uint32_t size = CREG(c, CH_SIZE);
    const uint32_t trans = CREG(c, CH_TRANS);
    const uint32_t addr = CREG(c, CH_ADDR);
    const uint32_t wptr = xm_aiao_off(c, CH_WPTR);
    const uint32_t rptr = xm_aiao_off(c, CH_RPTR);
    const uint32_t used = (wptr + size - rptr) % size;
    g_autofree uint8_t *buf = NULL;
    const uint32_t w = c->width;

    if (used + trans >= size) {
        CREG(c, CH_INT_RAW) |= INT_FULL;
        return;
    }
    buf = g_malloc0(trans);
    for (uint32_t off = 0; off + w <= trans; off += w) {
        int16_t v = 0;

        if (c->s->tone) {
            v = (int16_t)lrint(8000.0 * sin(2 * M_PI * c->s->tone *
                                            (double)c->frames / c->rate));
        }
        c->frames++;
        switch (w) {
        case 1:
            buf[off] = (uint8_t)(v >> 8);
            break;
        case 2:
            stw_le_p(buf + off, v);
            break;
        default:
            stl_le_p(buf + off, (uint32_t)(int32_t)v << 16);
            break;
        }
    }
    for (uint32_t done = 0; done < trans;) {
        const uint32_t pos = (wptr + done) % size;
        const uint32_t n = MIN(trans - done, size - pos);

        dma_memory_write(&address_space_memory, addr + pos, buf + done, n,
                         MEMTXATTRS_UNSPECIFIED);
        done += n;
    }
    CREG(c, CH_LAST) = addr + wptr;
    CREG(c, CH_WPTR) = addr + (wptr + trans) % size;
    CREG(c, CH_INT_RAW) |= INT_TRANS;
}

static void xm_aiao_playback(XmAiaoChn *c)
{
    const uint32_t size = CREG(c, CH_SIZE);
    const uint32_t trans = CREG(c, CH_TRANS);
    const uint32_t addr = CREG(c, CH_ADDR);
    const uint32_t wptr = xm_aiao_off(c, CH_WPTR);
    const uint32_t rptr = xm_aiao_off(c, CH_RPTR);
    const uint32_t avail = (wptr + size - rptr) % size;

    if (avail < trans) {
        CREG(c, CH_INT_RAW) |= INT_FULL;
        return;
    }
    CREG(c, CH_LAST) = addr + rptr;
    CREG(c, CH_RPTR) = addr + (rptr + trans) % size;
    CREG(c, CH_INT_RAW) |= INT_TRANS;
}

static void xm_aiao_tick(void *opaque)
{
    XmAiaoChn *c = opaque;

    if (!xm_aiao_running(c)) {
        return;
    }
    if (c->tx) {
        xm_aiao_playback(c);
    } else {
        xm_aiao_capture(c);
    }
    xm_aiao_update(c->s);
    timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              xm_aiao_period(c));
}

static void xm_aiao_start(XmAiaoChn *c)
{
    const uint32_t addr = CREG(c, CH_ADDR);

    c->rate = xm_aiao_rate(c);
    c->width = xm_aiao_width(c);
    c->frames = 0;
    /* the hardware's own pointer starts at the buffer */
    CREG(c, c->tx ? CH_RPTR : CH_WPTR) = addr;
    if (!c->tx) {
        CREG(c, CH_RPTR) = addr + xm_aiao_off(c, CH_RPTR);
    }
    if (c->s->trace) {
        qemu_log("xm-aiao: %s start %u Hz, %u-byte samples, buffer 0x%x+0x%x"
                 ", 0x%x a transfer\n", c->tx ? "playback" : "capture",
                 c->rate, c->width, addr, CREG(c, CH_SIZE),
                 CREG(c, CH_TRANS));
    }
    if (xm_aiao_running(c)) {
        timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  xm_aiao_period(c));
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-aiao: %s started with buffer "
                      "0x%x transfer 0x%x; it moves nothing\n",
                      c->tx ? "playback" : "capture", CREG(c, CH_SIZE),
                      CREG(c, CH_TRANS));
    }
}

static XmAiaoChn *xm_aiao_chn(XmAiaoState *s, hwaddr off, hwaddr *reg)
{
    if (off >= RX_BASE && off < RX_BASE + 0x100) {
        *reg = off - RX_BASE;
        return &s->chn[0];
    }
    if (off >= TX_BASE && off < TX_BASE + 0x100) {
        *reg = off - TX_BASE;
        return &s->chn[1];
    }
    return NULL;
}

static uint64_t xm_aiao_read(void *opaque, hwaddr off, unsigned size)
{
    XmAiaoState *s = opaque;
    hwaddr reg;
    XmAiaoChn *c = xm_aiao_chn(s, off & ~3, &reg);
    uint32_t v = REG(s, off & ~3);

    if (off == G_INT_STA) {
        v = xm_aiao_sta(s);
    } else if (c) {
        switch (reg) {
        case CH_INT_STA:
            v = CREG(c, CH_INT_RAW) & CREG(c, CH_INT_ENA);
            break;
        case CH_INT_CLR:
            v = 0;
            break;
        case CH_BUSY:
            v = (CREG(c, CH_CTRL) & CTRL_START) ? (c->tx ? 0x2 : 0x18) : 0x1;
            break;
        }
    }
    return extract32(v, (off & 3) * 8, size * 8);
}

static void xm_aiao_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    XmAiaoState *s = opaque;
    const hwaddr word = off & ~3;
    hwaddr reg;
    XmAiaoChn *c = xm_aiao_chn(s, word, &reg);
    uint32_t old = REG(s, word);
    uint32_t v = deposit32(old, (off & 3) * 8, size * 8, val);

    if (word == G_VERSION || word == G_INT_STA) {
        return;
    }
    if (!c) {
        REG(s, word) = v;
        return;
    }
    switch (reg) {
    case CH_INT_RAW:
    case CH_INT_STA:
    case CH_BUSY:
        return;
    case CH_INT_CLR:
        CREG(c, CH_INT_RAW) &= ~v;
        xm_aiao_update(s);
        return;
    case CH_INT_ENA:
        CREG(c, CH_INT_ENA) = v;
        xm_aiao_update(s);
        return;
    case CH_CTRL:
        CREG(c, CH_CTRL) = v;
        if ((v & CTRL_START) && !(old & CTRL_START)) {
            xm_aiao_start(c);
        } else if (!(v & CTRL_START) && (old & CTRL_START)) {
            timer_del(c->timer);
            if (s->trace) {
                qemu_log("xm-aiao: %s stop\n", c->tx ? "playback" : "capture");
            }
        }
        return;
    default:
        REG(s, word) = v;
        return;
    }
}

static const MemoryRegionOps xm_aiao_ops = {
    .read = xm_aiao_read,
    .write = xm_aiao_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void xm_aiao_reset(DeviceState *dev)
{
    XmAiaoState *s = XM_AIAO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    /* as a GK7206V11A reads them */
    REG(s, G_VERSION) = 0x20230430;
    REG(s, 0x14) = 0xffffffff;
    REG(s, 0x18) = 0xffffffff;
    REG(s, 0x20) = 0x76543210;
    REG(s, 0x24) = 0xfedcba98;
    for (int n = 0; n < 2; n++) {
        timer_del(s->chn[n].timer);
        s->chn[n].frames = 0;
    }
    xm_aiao_update(s);
}

static void xm_aiao_init(Object *obj)
{
    XmAiaoState *s = XM_AIAO(obj);

    memory_region_init_io(&s->iomem, obj, &xm_aiao_ops, s, TYPE_XM_AIAO,
                          AIAO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    for (int n = 0; n < 2; n++) {
        XmAiaoChn *c = &s->chn[n];

        c->s = s;
        c->tx = n == 1;
        c->base = c->tx ? TX_BASE : RX_BASE;
        c->crg = c->tx ? G_TX_CRG : G_RX_CRG;
        c->sta_bit = c->tx ? 1u << 8 : 1u << 0;
        c->rate = 8000;
        c->width = 2;
        c->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_aiao_tick, c);
    }
}

static const Property xm_aiao_props[] = {
    DEFINE_PROP_UINT32("tone", XmAiaoState, tone, 1000),
    DEFINE_PROP_BOOL("trace", XmAiaoState, trace, false),
};

static void xm_aiao_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_aiao_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_aiao_props);
}

static const TypeInfo xm_aiao_info = {
    .name          = TYPE_XM_AIAO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmAiaoState),
    .instance_init = xm_aiao_init,
    .class_init    = xm_aiao_class_init,
};

static void xm_aiao_register_types(void)
{
    type_register_static(&xm_aiao_info);
}

type_init(xm_aiao_register_types)
