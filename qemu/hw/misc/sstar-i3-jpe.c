/*
 * MStar infinity3 JPE: the JPEG encoder (vendor driver drivers/mstar/jpe2,
 * "mstar,jpe", 0x1F264000, GIC SPI 61, INT_IRQ_JPE).
 *
 * Register-driven, no firmware.  16-bit registers on the RIU's 4-byte
 * stride, bank 0x1320 (0x200 bytes).  What jpe2's _EncFire()
 * (hal/src/hal_jpe_ios.c in the I3 SDK MSC3XX_SDK_V_1711291828) does per
 * picture, register indices from hal/pub/hal_jpe_ios.h:
 *
 *   - 0x1C irq mask <- 0, then 0x00 bit 0 (frame_start, write one to
 *     start) with whatever 0x00 held from the last picture; then 0x00 bit 1
 *     (soft_rstz) low five times and high again.  That first start is
 *     stale, so the model only arms its encode on frame_start and drops it
 *     again on soft_rstz = 0: only a start after the reset encodes.
 *   - 0x3B bit 0 set_adr / bit 10 update_adr (write one), 0x3B bit 9
 *     obuf_hw_en = 0 (single buffer); the output buffer through 0x3C (low)
 *     and 0x3D [10:0] (high) in 4-byte units, 0x3D [15:14] naming it
 *     (0 = s0 start, 1 = e0 end, inclusive; 2/3 the second buffer), latched
 *     by 0x3E bit 0.
 *   - 0x1D [8:0] clears status bits (write one, reads back 0); 0x01
 *     width, 0x02 height (rounded up to 8); 0x5E [7:0]/[15:8] the luma and
 *     chroma pitch in 16-pixel units; 0x00 bit 3 the source format
 *     (0 = NV12 4:2:0, 1 = YUYV 4:2:2) and 0x5F bit 15 chroma swap (NV21,
 *     YVYU); 0x18 bit 2 frame buffer mode.
 *   - the quantization tables through the table port: 0x03 bit 6
 *     tbc_mode = 0 (software access), and per entry 0x4A the byte, 0x49
 *     {bit 0 rw = 1, [15:8] address} and 0x48 bit 12 tbc_en (write one).
 *     It polls 0x49 bit 2 (done) at most ten times, clears it with 0x49
 *     bit 1, then reads the entry back (rw = 0, tbc_en, done, 0x4B) and
 *     fails the picture (JPE_IOC_RET_BAD_QTABLE) on a mismatch.  Entry i
 *     of the luma table (0-63; chroma 64-127) is the raster table's
 *     [(i & 7) * 8 + (i >> 3)]: the table is held transposed.
 *   - 0x06/0x07 and 0x0A/0x0B the source luma and chroma, MIU address
 *     >> 8; 0x1C <- 0 and frame_start.
 *
 * The engine writes the entropy-coded segment only, 0xFF stuffed, ended by
 * EOI (FF D9); the application writes SOI .. SOS itself (jpegenc_marker.c:
 * the Annex K Huffman tables, Y 2x2 (NV12) or 2x1 (YUYV) and Cb, Cr 1x1,
 * DQT 0 for Y and 1 for Cb/Cr), so the scan is coded the same way.  0x1E
 * [7:0] is the status past the mask: bit 0 frame done, bit 1 output buffer
 * 0 full, bit 3 buffer 1 full; the line is high while it is non-zero.
 * _IsrFunc() takes the size from the write pointer 0x44/0x45 (4-byte
 * units) and 0x42 [1:0], the index of the last byte in its word:
 * size = wptr * 4 - s0 + byte_idx + 1, where it reads s0 from the same
 * 0x44/0x45 with 0x3B bit 13 (rchk_en) set and [12:11] = 0.  The driver
 * waits for the interrupt without a timeout (CamOsTsemWait), so every
 * start raises one; a scan larger than the buffer is cut at its end and
 * raises buffer full instead of frame done.
 *
 * The source is read from DRAM; when its MIU address is outside DRAM
 * (0x5F bit 14, the IMI path from the scaler) it is the test pattern of
 * sstar-pattern.h.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/sstar-jpeg.h"
#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_SSTAR_I3_JPE "sstar-i3-jpe"
OBJECT_DECLARE_SIMPLE_TYPE(SstarI3JpeState, SSTAR_I3_JPE)

#define JPE_REGS            0x80

#define R_CTRL              0x00
#define R_WIDTH             0x01
#define R_HEIGHT            0x02
#define R_TBC_MODE          0x03
#define R_Y0_LO             0x06
#define R_C0_LO             0x0A
#define R_IRQ_MASK          0x1C
#define R_IRQ_CLR           0x1D
#define R_IRQ_STATUS        0x1E
#define R_OBUF_CTRL         0x3B
#define R_OBUF_LO           0x3C
#define R_OBUF_HI           0x3D
#define R_OBUF_STROBE       0x3E
#define R_BYTE_IDX          0x42
#define R_WPTR_LO           0x44
#define R_WPTR_HI           0x45
#define R_TBC_EN            0x48
#define R_TBC_CTRL          0x49
#define R_TBC_WDATA         0x4A
#define R_TBC_RDATA         0x4B
#define R_PITCH             0x5E
#define R_YUVLDR            0x5F

#define CTRL_START          (1u << 0)
#define CTRL_RSTZ           (1u << 1)
#define CTRL_YUV422         (1u << 3)
#define OBUF_SET_ADR        (1u << 0)
#define OBUF_TOGGLE         (3u << 7)
#define OBUF_UPDATE_ADR     (1u << 10)
#define OBUF_RCHK_EN        (1u << 13)
#define TBC_EN              (1u << 12)
#define TBC_RW              (1u << 0)
#define TBC_DONE_CLR        (1u << 1)
#define TBC_DONE            (1u << 2)
#define YUVLDR_SWAP         (1u << 15)
#define IRQ_FRAME_DONE      (1u << 0)
#define IRQ_OBUF0_FULL      (1u << 1)

#define DRAM_BASE           0x20000000
#define ENCODE_NS           (2 * SCALE_MS)
#define OUT_MAX             (8 * MiB)

struct SstarI3JpeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint16_t regs[JPE_REGS];
    uint8_t raw;                    /* status before the mask */
    uint8_t qtab[256];              /* the table port's SRAM */
    uint32_t obuf[4];               /* s0, e0, s1, e1: MIU byte addresses */
    uint32_t frames;
    uint64_t dram_size;
    bool trace;
};

static void jpe_update(SstarI3JpeState *s)
{
    qemu_set_irq(s->irq, !!(s->raw & ~s->regs[R_IRQ_MASK] & 0xFF));
}

static uint32_t jpe_addr(SstarI3JpeState *s, int reg)
{
    return (s->regs[reg] | (uint32_t)(s->regs[reg + 1] & 0x1F) << 16) << 8;
}

static bool jpe_in_dram(SstarI3JpeState *s, uint32_t miu, uint64_t len)
{
    return (uint64_t)miu + len <= s->dram_size;
}

/*
 * The source picture as planes, chroma at half width (and half height for
 * 4:2:0), padded to whole MCUs by repeating the last column and row.
 */
typedef struct JpePic {
    int w, h, cw, ch;
    uint8_t *p[3];
} JpePic;

static void jpe_load(SstarI3JpeState *s, JpePic *pic, int w, int h,
                     bool yuv422)
{
    uint32_t ya = jpe_addr(s, R_Y0_LO), ca = jpe_addr(s, R_C0_LO);
    uint32_t ypitch = (s->regs[R_PITCH] & 0xFF) * 16;
    uint32_t cpitch = (s->regs[R_PITCH] >> 8) * 16;
    bool swap = s->regs[R_YUVLDR] & YUVLDR_SWAP;
    int x, y;

    pic->w = w;
    pic->h = h;
    pic->cw = DIV_ROUND_UP(w, 2);
    pic->ch = yuv422 ? h : DIV_ROUND_UP(h, 2);
    pic->p[0] = g_malloc0((size_t)w * h);
    pic->p[1] = g_malloc0((size_t)pic->cw * pic->ch);
    pic->p[2] = g_malloc0((size_t)pic->cw * pic->ch);
    if (!ypitch) {
        ypitch = ROUND_UP(w, 16);
    }
    if (!cpitch) {
        cpitch = ypitch;
    }
    if (yuv422) {
        ypitch *= 2;                /* YUYV: two bytes a pixel */
    }

    if (!jpe_in_dram(s, ya, (uint64_t)ypitch * h) ||
        (!yuv422 && !jpe_in_dram(s, ca, (uint64_t)cpitch * h / 2))) {
        /* fed through IMI by the scaler: the picture it would carry */
        uint8_t px[3];

        for (y = 0; y < h; y++) {
            for (x = 0; x < w; x++) {
                sstar_pattern_pixel(s->frames, w, h, x, y, px);
                pic->p[0][y * w + x] = px[0];
                if (!(x & 1) && (yuv422 || !(y & 1))) {
                    int cy = yuv422 ? y : y / 2;

                    pic->p[1][cy * pic->cw + x / 2] = px[1];
                    pic->p[2][cy * pic->cw + x / 2] = px[2];
                }
            }
        }
        return;
    }
    {
        g_autofree uint8_t *line = g_malloc(MAX(ypitch, cpitch));

        for (y = 0; y < h; y++) {
            address_space_read(&address_space_memory,
                               DRAM_BASE + ya + (hwaddr)ypitch * y,
                               MEMTXATTRS_UNSPECIFIED, line, ypitch);
            if (yuv422) {
                for (x = 0; x < w; x++) {
                    pic->p[0][y * w + x] = line[2 * MIN(x, ypitch / 2 - 1)];
                }
                for (x = 0; x < pic->cw; x++) {
                    int o = 4 * MIN(x, ypitch / 4 - 1);

                    pic->p[1][y * pic->cw + x] = line[o + (swap ? 3 : 1)];
                    pic->p[2][y * pic->cw + x] = line[o + (swap ? 1 : 3)];
                }
            } else {
                for (x = 0; x < w; x++) {
                    pic->p[0][y * w + x] = line[MIN((uint32_t)x, ypitch - 1)];
                }
            }
        }
        for (y = 0; !yuv422 && y < pic->ch; y++) {
            address_space_read(&address_space_memory,
                               DRAM_BASE + ca + (hwaddr)cpitch * y,
                               MEMTXATTRS_UNSPECIFIED, line, cpitch);
            for (x = 0; x < pic->cw; x++) {
                int o = 2 * MIN((uint32_t)x, cpitch / 2 - 1);

                pic->p[1][y * pic->cw + x] = line[o + (swap ? 1 : 0)];
                pic->p[2][y * pic->cw + x] = line[o + (swap ? 0 : 1)];
            }
        }
    }
}

static uint8_t jpe_sample(const JpePic *pic, int c, int x, int y)
{
    int w = c ? pic->cw : pic->w, h = c ? pic->ch : pic->h;

    return pic->p[c][MIN(y, h - 1) * w + MIN(x, w - 1)];
}

/* The scan: interleaved MCUs, Y 2x2 (4:2:0) or 2x1 (4:2:2), Cb, Cr. */
static size_t jpe_scan(SstarI3JpeState *s, const JpePic *pic, bool yuv422,
                       uint8_t *buf, size_t cap, bool *full)
{
    SstarJpegBits o = { .buf = buf, .cap = cap };
    g_autofree SstarJpegTables *t = g_new0(SstarJpegTables, 1);
    int mw = 16, mh = yuv422 ? 8 : 16, mx, my, c, bx, by, i, j;
    int pred[3] = { 0 };
    uint8_t px[64];

    t->w = pic->w;
    t->h = pic->h;
    t->ncomp = 3;
    t->hs[0] = 2;
    t->vs[0] = yuv422 ? 1 : 2;
    t->hs[1] = t->vs[1] = t->hs[2] = t->vs[2] = 1;
    t->tq[1] = t->tq[2] = 1;
    t->td[1] = t->td[2] = t->ta[1] = t->ta[2] = 1;
    sstar_jpeg_annex_k(t);
    /* the SRAM holds the raster tables transposed; DQT order is zigzag */
    for (i = 0; i < 64; i++) {
        int z = sstar_jpeg_zigzag[i];
        int hw = (z & 7) * 8 + (z >> 3);

        t->q[0][i] = s->qtab[hw];
        t->q[1][i] = s->qtab[64 + hw];
    }

    for (my = 0; my < DIV_ROUND_UP(pic->h, mh) && !o.full; my++) {
        for (mx = 0; mx < DIV_ROUND_UP(pic->w, mw) && !o.full; mx++) {
            for (c = 0; c < 3; c++) {
                for (by = 0; by < t->vs[c]; by++) {
                    for (bx = 0; bx < t->hs[c]; bx++) {
                        int x0 = (mx * t->hs[c] + bx) * 8;
                        int y0 = (my * t->vs[c] + by) * 8;

                        for (i = 0; i < 8; i++) {
                            for (j = 0; j < 8; j++) {
                                px[i * 8 + j] = jpe_sample(pic, c, x0 + j,
                                                           y0 + i);
                            }
                        }
                        sstar_jpeg_block(&o, t, c, px, &pred[c]);
                    }
                }
            }
        }
    }
    sstar_jpeg_flush(&o);
    /* EOI; the scan is cut at the buffer's end when it does not fit */
    if (!o.full && o.len + 2 <= cap) {
        buf[o.len++] = 0xFF;
        buf[o.len++] = 0xD9;
    } else {
        o.full = true;
    }
    *full = o.full;
    return o.len;
}

static void jpe_encode(void *opaque)
{
    SstarI3JpeState *s = opaque;
    bool yuv422 = s->regs[R_CTRL] & CTRL_YUV422;
    int w = s->regs[R_WIDTH] & 0xFFF, h = s->regs[R_HEIGHT] & 0xFFF;
    uint32_t s0 = s->obuf[0], e0 = s->obuf[1];
    size_t cap = e0 >= s0 ? (size_t)(e0 - s0) + 1 : 0, len = 0;
    uint32_t last;
    bool full = true;

    cap = MIN(cap, OUT_MAX);
    if (w && h && cap) {
        g_autofree uint8_t *buf = g_malloc(cap);
        JpePic pic;

        jpe_load(s, &pic, w, h, yuv422);
        len = jpe_scan(s, &pic, yuv422, buf, cap, &full);
        g_free(pic.p[0]);
        g_free(pic.p[1]);
        g_free(pic.p[2]);
        if (jpe_in_dram(s, s0, len)) {
            address_space_write(&address_space_memory, DRAM_BASE + s0,
                                MEMTXATTRS_UNSPECIFIED, buf, len);
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-i3-jpe: picture %dx%d, "
                      "output buffer 0x%x-0x%x\n", w, h, s0, e0);
    }

    /* the last byte written: wptr in words and its index in the word */
    last = s0 + MAX(len, 1) - 1;
    s->regs[R_WPTR_LO] = (last >> 2) & 0xFFFF;
    s->regs[R_WPTR_HI] = (last >> 18) & 0x7FF;
    s->regs[R_BYTE_IDX] = (s->regs[R_BYTE_IDX] & ~3) | (last & 3);
    s->raw |= full ? IRQ_OBUF0_FULL : IRQ_FRAME_DONE;
    if (s->trace) {
        qemu_log("sstar-i3-jpe: %dx%d %s src 0x%x, %zu bytes at 0x%x%s\n",
                 w, h, yuv422 ? "YUYV" : "NV12", jpe_addr(s, R_Y0_LO), len,
                 s0, full ? " (buffer full)" : "");
    }
    s->frames++;
    jpe_update(s);
}

static uint64_t jpe_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3JpeState *s = opaque;
    int reg = (off >> 2) & (JPE_REGS - 1);
    uint16_t v = s->regs[reg];

    if ((off & 3) >= 2) {
        return 0;
    }
    switch (reg) {
    case R_IRQ_STATUS:
        v = s->raw & ~s->regs[R_IRQ_MASK] & 0xFF;
        break;
    case R_WPTR_LO:
    case R_WPTR_HI:
        /* rchk_en: the latched buffer address [12:11] picks instead */
        if (s->regs[R_OBUF_CTRL] & OBUF_RCHK_EN) {
            uint32_t a = s->obuf[(s->regs[R_OBUF_CTRL] >> 11) & 3] >> 2;

            v = reg == R_WPTR_LO ? a & 0xFFFF : (a >> 16) & 0x7FF;
        }
        break;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void jpe_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarI3JpeState *s = opaque;
    int reg = (off >> 2) & (JPE_REGS - 1);
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    switch (reg) {
    case R_CTRL:
        if (!(v & CTRL_RSTZ)) {
            /* soft reset: drops a start not yet done (the stale one) */
            timer_del(s->timer);
            s->raw = 0;
        } else if (v & CTRL_START) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      ENCODE_NS);
        }
        v &= ~CTRL_START;
        break;
    case R_IRQ_CLR:
        s->raw &= ~v;
        v = 0;
        break;
    case R_IRQ_STATUS:
    case R_BYTE_IDX:
    case R_WPTR_LO:
    case R_WPTR_HI:
    case R_TBC_RDATA:
        return;
    case R_OBUF_CTRL:
        v &= ~(OBUF_SET_ADR | OBUF_TOGGLE | OBUF_UPDATE_ADR);
        break;
    case R_OBUF_STROBE:
        if (v & 1) {
            int id = s->regs[R_OBUF_HI] >> 14;

            s->obuf[id] = (s->regs[R_OBUF_LO] |
                           (uint32_t)(s->regs[R_OBUF_HI] & 0x7FF) << 16) << 2;
        }
        v = 0;
        break;
    case R_TBC_EN:
        if (v & TBC_EN) {
            int a = s->regs[R_TBC_CTRL] >> 8;

            if (s->regs[R_TBC_CTRL] & TBC_RW) {
                s->qtab[a] = s->regs[R_TBC_WDATA] & 0xFF;
            } else {
                s->regs[R_TBC_RDATA] = s->qtab[a];
            }
            s->regs[R_TBC_CTRL] |= TBC_DONE;
        }
        v &= ~TBC_EN;
        break;
    case R_TBC_CTRL:
        /* done is the engine's; done_clr (write one) clears it */
        v = (v & ~(TBC_DONE | TBC_DONE_CLR)) |
            (v & TBC_DONE_CLR ? 0 : s->regs[reg] & TBC_DONE);
        break;
    }
    s->regs[reg] = v;
    jpe_update(s);
}

static const MemoryRegionOps jpe_ops = {
    .read = jpe_read,
    .write = jpe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_i3_jpe_reset(DeviceState *dev)
{
    SstarI3JpeState *s = SSTAR_I3_JPE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->qtab, 0, sizeof(s->qtab));
    memset(s->obuf, 0, sizeof(s->obuf));
    s->regs[R_CTRL] = CTRL_RSTZ;
    s->regs[R_IRQ_MASK] = 0xFF;
    s->raw = 0;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void sstar_i3_jpe_init(Object *obj)
{
    SstarI3JpeState *s = SSTAR_I3_JPE(obj);

    memory_region_init_io(&s->iomem, obj, &jpe_ops, s, TYPE_SSTAR_I3_JPE,
                          JPE_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, jpe_encode, s);
}

static const Property sstar_i3_jpe_props[] = {
    DEFINE_PROP_UINT64("dram-size", SstarI3JpeState, dram_size, 64 * MiB),
    DEFINE_PROP_BOOL("trace", SstarI3JpeState, trace, false),
};

static const VMStateDescription vmstate_sstar_i3_jpe = {
    .name = TYPE_SSTAR_I3_JPE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarI3JpeState, JPE_REGS),
        VMSTATE_UINT8(raw, SstarI3JpeState),
        VMSTATE_UINT8_ARRAY(qtab, SstarI3JpeState, 256),
        VMSTATE_UINT32_ARRAY(obuf, SstarI3JpeState, 4),
        VMSTATE_UINT32(frames, SstarI3JpeState),
        VMSTATE_TIMER_PTR(timer, SstarI3JpeState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_i3_jpe_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_i3_jpe_reset);
    device_class_set_props(dc, sstar_i3_jpe_props);
    dc->vmsd = &vmstate_sstar_i3_jpe;
}

static const TypeInfo sstar_i3_jpe_info = {
    .name           = TYPE_SSTAR_I3_JPE,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarI3JpeState),
    .instance_init  = sstar_i3_jpe_init,
    .class_init     = sstar_i3_jpe_class_init,
};

static void sstar_i3_jpe_register_types(void)
{
    type_register_static(&sstar_i3_jpe_info);
}

type_init(sstar_i3_jpe_register_types)
