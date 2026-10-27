/*
 * SigmaStar Ifado JPEG encoder (JPE), at its frame handshake.
 *
 * RIU bank 0x1237 (0x1F246E00, the DTB's "sstar,cedric-jpe"), 16-bit
 * registers on a 4-byte stride (sstar_mi.ko HalJpeIos*; register
 * indices):
 *
 *   0x00        bit 0 reg_opr_jpeg_enc_en fires a frame
 *               (HalJpeIosEncFire); [7:4] the frame index
 *   0x08..0x0B  the source picture's luma and chroma
 *   0x20        reg_jpe_g_irq_mask, 1 = masked (HalJpeIosMaskIrq)
 *   0x21        reg_jpe_g_irq_clr: the bits to clear (_HalJpeIosClearIrq)
 *   0x22        reg_jpe_g_irq_cpu, the status (HalJpeIosIsr; the SDK's
 *               E_JPE_HAL_IRQ_*): bit 0 frame done, bit 1 output buffer
 *               full, bit 2 its reset, bits 3-6 errors
 *   0x51..0x54  reg_jpeg_wr_base/end_addr: the output buffer, MIU
 *               addresses in 16-byte units, the end inclusive
 *   0x63..0x6C  read only: the frame's output start and end (16-byte
 *               units), its size, the write's end and its byte count
 *
 * The names are the Ifliegen SDK's (sgs_mi.ko's debug information,
 * hal_jpe_ios.c), whose JPE is this one.
 *
 * Here a fired frame completes "done-us" later with a placeholder in the
 * output buffer: the end-of-image marker, no picture.  Everything else in
 * the bank is storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/sstar-jpeg.h"
#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SSTAR_JPE "sstar-jpe"
OBJECT_DECLARE_SIMPLE_TYPE(SstarJpeState, SSTAR_JPE)

#define JPE_REGS            0x80
#define R_CTRL              0x00
#define CTRL_FIRE           (1u << 0)
#define R_WR_BASE_LO        0x51
#define R_WR_BASE_HI        0x52
#define R_INT_MASK          0x20
#define R_INT_CLEAR         0x21
#define R_INT_STATUS        0x22
#define R_RO_OBUF_SADR      0x63        /* lo, hi */
#define R_RO_OBUF_EADR      0x65
#define R_RO_OBUF_SIZE      0x67
#define R_RO_W2MI_END       0x69
#define R_RO_WBYTE_CNT      0x6B
#define INT_BITS            0x3F
#define INT_FRAME_DONE      (1u << 0)   /* E_JPE_HAL_IRQ_FRAME_DONE */

#define JPE_BYTES           16          /* the placeholder frame */
#define R_HDR_CTRL          0x1C        /* [0] copy, [14:8] size - 1 */
#define R_HDR_LO            0x1D
#define R_HDR_HI            0x1E
#define R_WR_END_LO         0x53
#define R_WR_END_HI         0x54
#define HDR_MAX             2048
#define OUT_MAX             (8 * MiB)   /* a scan larger is not coded */
#define DIM_MAX             8192
#define SS_DRAM_BASE        0x20000000

struct SstarJpeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;
    uint16_t regs[JPE_REGS];

    uint32_t done_us;
    bool trace;
    uint32_t frames;
};

/*
 * A baseline JPEG scan (sstar-jpeg.h), coded with the tables of the
 * header the driver gives the engine (DQT, SOF0, DHT, SOS), of the test
 * pattern of sstar-pattern.h (the OpenIPC logo on navy, its dot) at the
 * header's size and sampling.
 */
typedef SstarJpegTables JpeTables;
typedef SstarJpegBits JpeBits;

static bool jpe_parse(const uint8_t *b, size_t n, JpeTables *t, size_t *hdr)
{
    size_t i = 2;
    int c, k;

    memset(t, 0, sizeof(*t));
    if (n < 4 || b[0] != 0xFF || b[1] != 0xD8) {
        return false;
    }
    while (i + 4 <= n) {
        const uint8_t *d = b + i + 4;
        uint8_t m;
        size_t l, dl;

        if (b[i] != 0xFF) {
            return false;
        }
        m = b[i + 1];
        l = b[i + 2] << 8 | b[i + 3];
        if (l < 2 || i + 2 + l > n) {
            return false;
        }
        dl = l - 2;

        if (m == 0xDB) {                /* DQT, 8-bit tables */
            for (k = 0; k + 65 <= (int)dl; k += 65) {
                memcpy(t->q[d[k] & 3], d + k + 1, 64);
            }
        } else if (m == 0xC0) {         /* SOF0 */
            if (dl < 6 || d[5] != 3 || dl < 6 + 3 * 3) {
                return false;
            }
            t->h = d[1] << 8 | d[2];
            t->w = d[3] << 8 | d[4];
            t->ncomp = 3;
            for (c = 0; c < t->ncomp; c++) {
                t->hs[c] = d[7 + c * 3] >> 4;
                t->vs[c] = d[7 + c * 3] & 15;
                t->tq[c] = d[8 + c * 3] & 3;
                if (t->hs[c] < 1 || t->hs[c] > 4 ||
                    t->vs[c] < 1 || t->vs[c] > 4) {
                    return false;
                }
            }
            if (t->w > DIM_MAX || t->h > DIM_MAX) {
                return false;
            }
        } else if (m == 0xC4) {         /* DHT */
            size_t o = 0;

            while (o + 17 <= dl) {
                int tc = d[o] >> 4, th = d[o] & 15;
                size_t used;

                if (tc > 1 || th > 3) {     /* baseline: DC/AC, 0-3 */
                    return false;
                }
                used = sstar_jpeg_huff_build(t, tc * 4 + th, d + o + 1,
                                             dl - o - 1);
                if (!used) {
                    return false;
                }
                o += 1 + used;
            }
        } else if (m == 0xDA) {         /* SOS: the header ends here */
            if (dl < 1 || d[0] != 3 || dl < 1 + 3 * 2) {
                return false;
            }
            for (c = 0; c < 3; c++) {
                t->td[c] = d[2 + c * 2] >> 4;
                t->ta[c] = d[2 + c * 2] & 15;
                if (t->td[c] > 3 || t->ta[c] > 3) {
                    return false;
                }
            }
            *hdr = i + 2 + l;
            return t->w > 0 && t->h > 0 && t->ncomp == 3;
        }
        i += 2 + l;
    }
    return false;
}

/* The test picture's sample of component c at (x, y), full resolution. */
static uint8_t jpe_pixel(int c, int x, int y, int w, int h, uint32_t frame)
{
    uint8_t yuv[3];

    sstar_pattern_pixel(frame, w, h, x, y, yuv);
    return yuv[c];
}

static size_t jpe_encode(const JpeTables *t, uint32_t frame, uint8_t *buf,
                         size_t cap)
{
    JpeBits o = { .buf = buf, .cap = cap };
    int hmax = 1, vmax = 1, c, mx, my, bx, by, i, j;
    int pred[3] = { 0 };
    uint8_t px[64];

    for (c = 0; c < 3; c++) {
        hmax = MAX(hmax, t->hs[c]);
        vmax = MAX(vmax, t->vs[c]);
    }
    for (my = 0; my < DIV_ROUND_UP(t->h, 8 * vmax) && !o.full; my++) {
        for (mx = 0; mx < DIV_ROUND_UP(t->w, 8 * hmax) && !o.full; mx++) {
            for (c = 0; c < 3; c++) {
                int sx = hmax / MAX(t->hs[c], 1), sy = vmax / MAX(t->vs[c], 1);

                for (by = 0; by < t->vs[c]; by++) {
                    for (bx = 0; bx < t->hs[c]; bx++) {
                        for (i = 0; i < 8; i++) {
                            for (j = 0; j < 8; j++) {
                                int x = ((mx * t->hs[c] + bx) * 8 + j) * sx;
                                int y = ((my * t->vs[c] + by) * 8 + i) * sy;

                                px[i * 8 + j] = jpe_pixel(c, MIN(x, t->w - 1),
                                                          MIN(y, t->h - 1),
                                                          t->w, t->h, frame);
                            }
                        }
                        sstar_jpeg_block(&o, t, c, px, &pred[c]);
                    }
                }
            }
        }
    }
    sstar_jpeg_flush(&o);               /* pad with ones */
    if (o.len + 2 > cap) {
        return 0;
    }
    buf[o.len++] = 0xFF;
    buf[o.len++] = 0xD9;
    return o.full ? 0 : o.len;
}

static void sstar_jpe_update_irq(SstarJpeState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R_INT_STATUS] & ~s->regs[R_INT_MASK] &
                            INT_BITS));
}

static hwaddr jpe_dram(uint32_t miu16)
{
    hwaddr a = (hwaddr)miu16 << 4;

    return a < SS_DRAM_BASE ? SS_DRAM_BASE + a : a;
}

static void jpe_set32(SstarJpeState *s, int reg, uint32_t v)
{
    s->regs[reg] = v & 0xFFFF;
    s->regs[reg + 1] = v >> 16;
}

static void sstar_jpe_frame_done(void *opaque)
{
    SstarJpeState *s = opaque;
    uint32_t out = s->regs[R_WR_BASE_LO] |
                   (uint32_t)s->regs[R_WR_BASE_HI] << 16;
    uint32_t lim = s->regs[R_WR_END_LO] | (uint32_t)s->regs[R_WR_END_HI] << 16;
    uint32_t ha = s->regs[R_HDR_LO] | (uint32_t)s->regs[R_HDR_HI] << 16;
    size_t hlen = (((s->regs[R_HDR_CTRL] >> 8) & 0x7F) + 1) * 16;
    size_t cap = lim >= out ? ((size_t)(lim - out) + 1) * 16 : 0;
    size_t len = 0, hdr = 0;
    g_autofree uint8_t *buf = NULL;
    JpeTables t;

    /*
     * The driver's header (SOI .. SOS) is copied in front of the scan
     * (reg_header_copy_enable); a frame that cannot be coded (a header
     * that is not 3-component baseline up to 8192x8192), or does not fit
     * the output buffer or OUT_MAX, is the end-of-image marker alone.
     */
    cap = MIN(cap, OUT_MAX);
    if (out && cap > HDR_MAX && (s->regs[R_HDR_CTRL] & 1) && ha &&
        hlen <= HDR_MAX) {
        buf = g_malloc(cap);
        if (dma_memory_read(&address_space_memory, jpe_dram(ha), buf, hlen,
                            MEMTXATTRS_UNSPECIFIED) == MEMTX_OK &&
            jpe_parse(buf, hlen, &t, &hdr) && hdr <= hlen) {
            size_t n = jpe_encode(&t, s->frames, buf + hlen, cap - hlen);

            len = n ? hlen + n : 0;
        }
    }
    if (!len) {
        static const uint8_t eoi[JPE_BYTES] = {
            [JPE_BYTES - 2] = 0xFF, [JPE_BYTES - 1] = 0xD9,
        };

        g_free(buf);
        buf = g_memdup2(eoi, sizeof(eoi));
        len = sizeof(eoi);
    }
    if (out) {
        dma_memory_write(&address_space_memory, jpe_dram(out), buf, len,
                         MEMTXATTRS_UNSPECIFIED);
    }
    s->frames++;
    jpe_set32(s, R_RO_OBUF_SADR, out);
    jpe_set32(s, R_RO_OBUF_EADR, out + DIV_ROUND_UP(len, 16));
    jpe_set32(s, R_RO_OBUF_SIZE, len);
    jpe_set32(s, R_RO_W2MI_END, out + DIV_ROUND_UP(len, 16));
    jpe_set32(s, R_RO_WBYTE_CNT, len);
    s->regs[R_INT_STATUS] |= INT_FRAME_DONE;
    if (s->trace) {
        qemu_log("sstar-jpe: frame done, %zu bytes at 0x%08" PRIx64 "\n",
                 len, (uint64_t)jpe_dram(out));
    }
    sstar_jpe_update_irq(s);
}

static uint64_t sstar_jpe_read(void *opaque, hwaddr off, unsigned size)
{
    SstarJpeState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->regs[(off >> 2) & (JPE_REGS - 1)];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_jpe_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    SstarJpeState *s = opaque;
    int reg = (off >> 2) & (JPE_REGS - 1);
    uint16_t mask, v;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        mask = off & 1 ? 0xFF00 : 0x00FF;
        v = off & 1 ? (val & 0xFF) << 8 : val & 0xFF;
    } else {
        mask = 0xFFFF;
        v = val;
    }
    if (s->trace) {
        qemu_log("sstar-jpe: W reg 0x%02x = 0x%04x/0x%04x\n", reg, v, mask);
    }
    if (reg == R_INT_STATUS) {
        return;
    }
    s->regs[reg] = (s->regs[reg] & ~mask) | (v & mask);
    switch (reg) {
    case R_CTRL:
        if (v & mask & CTRL_FIRE) {
            timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      (int64_t)s->done_us * 1000);
        }
        break;
    case R_INT_CLEAR:
        s->regs[R_INT_STATUS] &= ~(v & mask & INT_BITS);
        sstar_jpe_update_irq(s);
        break;
    case R_INT_MASK:
        sstar_jpe_update_irq(s);
        break;
    }
}

static const MemoryRegionOps sstar_jpe_ops = {
    .read = sstar_jpe_read,
    .write = sstar_jpe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_jpe_reset(DeviceState *dev)
{
    SstarJpeState *s = SSTAR_JPE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_INT_MASK] = INT_BITS;
    timer_del(s->done);
    qemu_set_irq(s->irq, 0);
}

static void sstar_jpe_realize(DeviceState *dev, Error **errp)
{
    SstarJpeState *s = SSTAR_JPE(dev);

    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, sstar_jpe_frame_done, s);
}

static void sstar_jpe_init(Object *obj)
{
    SstarJpeState *s = SSTAR_JPE(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_jpe_ops, s,
                          TYPE_SSTAR_JPE, JPE_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sstar_jpe_props[] = {
    DEFINE_PROP_UINT32("done-us", SstarJpeState, done_us, 2000),
    DEFINE_PROP_BOOL("trace", SstarJpeState, trace, false),
};

static const VMStateDescription vmstate_sstar_jpe = {
    .name = TYPE_SSTAR_JPE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarJpeState, JPE_REGS),
        VMSTATE_TIMER_PTR(done, SstarJpeState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_jpe_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_jpe_realize;
    device_class_set_legacy_reset(dc, sstar_jpe_reset);
    device_class_set_props(dc, sstar_jpe_props);
    dc->vmsd = &vmstate_sstar_jpe;
}

static const TypeInfo sstar_jpe_info = {
    .name           = TYPE_SSTAR_JPE,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarJpeState),
    .instance_init  = sstar_jpe_init,
    .class_init     = sstar_jpe_class_init,
};

static void sstar_jpe_register_types(void)
{
    type_register_static(&sstar_jpe_info);
}

type_init(sstar_jpe_register_types)
