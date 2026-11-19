/*
 * Fullhan FH8626V100 PAE H.264 encoder at 0xE7000000, as OpenIPC's 4.19
 * drivers/media/platform/fh8626/fh8626-pae-hw.c drives it: Main profile,
 * CABAC, one reference, a slice a picture.  There is no SDK or manual for
 * the part; that driver is the description.
 *
 * core (0xE7000000, 16 KiB)
 *   0x08 IRQ_STATUS   bit 0 the slice is done, 1 the output window is
 *                     full; write one to clear     0x0C IRQ_ENABLE (IRQ 4)
 *   0x18 CTRL         bit 16 resets the engine, which 0x20 acknowledges
 *   0x3000 TAE        0xB0 / 0xD0 the slice header FIFO: 0x01011000 on
 *                     0xD0 empties it, 2 x bits - 1 pushes that many bits
 *                     of 0xB0, the last ones; 0x10000000 starts the
 *                     picture.  0x110 the bits written.
 * working (0xE7010000) the QP and policy tables: storage
 * SRAM (0x10000000, the machine's) 0x7E00 the frame's parameters as the
 *                     driver lays them out: coded size (0x04 / 0x06), the
 *                     intra / inter QP (0x0C / 0x0D), the deblocking
 *                     (0x10-0x13), the slice type (0xC3: 2 I, 0 P), the
 *                     input planes (0x134 / 0x138), the reference ring
 *                     (0x140) and the output ring (0x14C, its size 0x15A),
 *                     addresses in 1 KiB units
 * The engine sits on a bus whose DMA addresses start at DDR ("dma-base",
 * 0xA0000000: the DT's pae-bus dma-ranges), hence the driver's 29 bits.
 *
 * A picture is the slice NAL's payload, after the 00 00 00 01 and NAL
 * header byte the driver puts in the ring's bytes 11-15: the header bits
 * the driver pushed, the slice data, emulation prevention bytes, written
 * from the ring's byte 16 on, and its length in bits at TAE 0x110.  The
 * slice data is the shared encoder's (hantro-h264.c: I_16x16, P_Skip,
 * P_L0_16x16 with motion search, mb_qp_delta to fit).  The reference --
 * the deblocked reconstruction, as the decoder has it -- is kept here, by
 * the context's reference ring address, so contexts sharing the engine
 * each predict from their own; the engine's own reference, motion and
 * macroblock buffers in guest memory are not written.  CABAC state the
 * driver saves at SRAM 0x7A00 is left as it is: each slice starts from
 * cabac_init_idc 0, as its header says.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hantro-h264.h"

#define TYPE_FH_PAE "fh-pae"
OBJECT_DECLARE_SIMPLE_TYPE(FhPaeState, FH_PAE)

#define FH_PAE_SIZE         0x4000
#define FH_PAE_WORK_SIZE    0x20000

#define R_IRQ_STATUS        0x08
#define R_IRQ_ENABLE        0x0C
#define R_CTRL              0x18
#define R_RESET_DONE        0x20
#define R_TAE               0x3000
#define R_TAE_FIFO          (R_TAE + 0xB0)
#define R_TAE_CMD           (R_TAE + 0xD0)
#define R_TAE_BITS          (R_TAE + 0x110)

#define CTRL_RESET          (1u << 16)
#define IRQ_SLICE           1u
#define CMD_HDR_RESET       0x01011000
#define CMD_START           0x10000000

#define PARAMS_ADDR         0x10007E00
#define PARAMS_SIZE         0x200
#define HDR_MAX             64
#define CONTEXTS            4
#define MAX_MB              (1920 / 16)
/* a picture takes this long to come out */
#define ENCODE_NS           (10 * SCALE_MS)

typedef struct FhPaeRef {
    uint32_t key;               /* the context's reference ring */
    int mbw, mbh;
    uint8_t *y, *uv;            /* NV12 at stride mbw * 16 */
    uint64_t used;
} FhPaeRef;

struct FhPaeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion work;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[FH_PAE_SIZE / 4];
    uint32_t status;
    uint8_t hdr[HDR_MAX];
    uint32_t hdr_bits;

    FhPaeRef ref[CONTEXTS];
    uint64_t pictures;
    uint32_t dma_base;
};

static uint32_t fh_pae_r(FhPaeState *s, hwaddr off)
{
    return s->regs[off >> 2];
}

static void fh_pae_update_irq(FhPaeState *s)
{
    qemu_set_irq(s->irq, !!(s->status & fh_pae_r(s, R_IRQ_ENABLE)));
}

static void fh_pae_push_bits(FhPaeState *s, uint32_t val, unsigned bits)
{
    unsigned i;

    for (i = 0; i < bits; i++) {
        unsigned bit = (val >> (bits - 1 - i)) & 1;

        if (s->hdr_bits >= HDR_MAX * 8) {
            qemu_log_mask(LOG_GUEST_ERROR, "fh-pae: slice header over %d "
                          "bytes\n", HDR_MAX);
            return;
        }
        if (bit) {
            s->hdr[s->hdr_bits / 8] |= 0x80 >> (s->hdr_bits % 8);
        } else {
            s->hdr[s->hdr_bits / 8] &= ~(0x80 >> (s->hdr_bits % 8));
        }
        s->hdr_bits++;
    }
}

/* the reference of the context whose ring is at key, made fit for mbw x mbh */
static FhPaeRef *fh_pae_ref(FhPaeState *s, uint32_t key, int mbw, int mbh,
                            bool fresh)
{
    FhPaeRef *r = NULL, *old = &s->ref[0];
    int i;

    for (i = 0; i < CONTEXTS; i++) {
        if (s->ref[i].y && s->ref[i].key == key) {
            r = &s->ref[i];
            break;
        }
        if (!s->ref[i].y || s->ref[i].used < old->used) {
            old = &s->ref[i];
        }
    }
    if (!r || r->mbw != mbw || r->mbh != mbh) {
        if (!fresh) {
            return NULL;        /* no reference: code the P slice intra */
        }
        r = r ? r : old;
        g_free(r->y);
        g_free(r->uv);
        r->key = key;
        r->mbw = mbw;
        r->mbh = mbh;
        r->y = g_malloc0(mbw * mbh * 256);
        r->uv = g_malloc0(mbw * mbh * 128);
    }
    r->used = ++s->pictures;
    return r;
}

/* RBSP to NAL payload: 0x03 after two zeros that a byte <= 3 follows */
static size_t fh_pae_escape(const uint8_t *in, size_t n, uint8_t *out,
                            size_t cap)
{
    size_t i, o = 0;
    int zeros = 0;

    for (i = 0; i < n && o < cap; i++) {
        if (zeros == 2 && in[i] <= 3) {
            out[o++] = 3;
            zeros = 0;
            if (o >= cap) {
                break;
            }
        }
        out[o++] = in[i];
        zeros = in[i] ? 0 : zeros + 1;
    }
    return o;
}

static void fh_pae_encode(FhPaeState *s)
{
    uint8_t p[PARAMS_SIZE];
    int w, h, mbw, mbh, qp;
    bool idr;
    uint32_t y, uv, ring, ring_bytes, key;
    size_t hdr_bytes, n, len, cap;
    g_autofree uint8_t *in = NULL, *rbsp = NULL, *nal = NULL;
    H264Deblock db = { 0 };
    H264Pic *pic;
    FhPaeRef *r;

    s->regs[R_TAE_BITS >> 2] = 0;
    dma_memory_read(&address_space_memory, PARAMS_ADDR, p, sizeof(p),
                    MEMTXATTRS_UNSPECIFIED);
    w = lduw_le_p(p + 0x04);
    h = lduw_le_p(p + 0x06);
    idr = p[0xC3] == 2;
    qp = idr ? p[0x0C] : p[0x0D];
    y = s->dma_base + (ldl_le_p(p + 0x134) << 10);
    uv = s->dma_base + (ldl_le_p(p + 0x138) << 10);
    key = ldl_le_p(p + 0x140);
    ring = s->dma_base + (ldl_le_p(p + 0x14C) << 10);
    ring_bytes = lduw_le_p(p + 0x15A) << 10;
    mbw = w / 16;
    mbh = h / 16;
    if (!mbw || !mbh || (w & 15) || (h & 15) || mbw > MAX_MB ||
        mbh > MAX_MB || qp > 51 || ring_bytes <= 16 || (s->hdr_bits & 7)) {
        qemu_log_mask(LOG_GUEST_ERROR, "fh-pae: %dx%d QP %d, ring of %u, "
                      "header of %u bits\n", w, h, qp, ring_bytes,
                      s->hdr_bits);
        return;
    }
    if (p[0x10]) {
        db.idc = p[0x11] & 3;
        db.alpha_off = 2 * (((int8_t)(p[0x12] << 4)) >> 4);
        db.beta_off = 2 * (((int8_t)(p[0x13] << 4)) >> 4);
    }

    in = g_malloc((size_t)w * h * 3 / 2);
    dma_memory_read(&address_space_memory, y, in, (size_t)w * h,
                    MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, uv, in + (size_t)w * h,
                    (size_t)w * h / 2, MEMTXATTRS_UNSPECIFIED);
    r = fh_pae_ref(s, key, mbw, mbh, idr);
    pic = h264_pic_new(mbw, mbh, in, w, in + (size_t)w * h, w,
                       !idr && r ? r->y : NULL, !idr && r ? r->uv : NULL,
                       0, false);

    cap = ring_bytes - 16;
    hdr_bytes = s->hdr_bits / 8;
    rbsp = g_malloc(cap);
    memcpy(rbsp, s->hdr, MIN(hdr_bytes, cap));
    n = h264_pic_slice_cabac(pic, !idr, 0, mbh, qp, 0, rbsp + hdr_bytes,
                             cap - hdr_bytes, NULL);
    if (!r) {
        r = fh_pae_ref(s, key, mbw, mbh, true);
    }
    h264_pic_finish(pic, &db, r->y, r->uv);
    h264_pic_free(pic);
    if (hdr_bytes + n > cap) {
        qemu_log_mask(LOG_GUEST_ERROR, "fh-pae: the %u-byte ring is too "
                      "small\n", ring_bytes);
        n = cap - hdr_bytes;
    }
    nal = g_malloc(cap);
    len = fh_pae_escape(rbsp, hdr_bytes + n, nal, cap);
    dma_memory_write(&address_space_memory, ring + 16, nal, len,
                     MEMTXATTRS_UNSPECIFIED);
    s->regs[R_TAE_BITS >> 2] = len * 8;
}

static void fh_pae_done(void *opaque)
{
    FhPaeState *s = opaque;

    s->status |= IRQ_SLICE;
    fh_pae_update_irq(s);
}

static uint64_t fh_pae_read(void *opaque, hwaddr off, unsigned size)
{
    FhPaeState *s = opaque;

    if (off == R_IRQ_STATUS) {
        return s->status;
    }
    return s->regs[off >> 2];
}

static void fh_pae_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhPaeState *s = opaque;

    switch (off) {
    case R_IRQ_STATUS:
        s->status &= ~val;
        fh_pae_update_irq(s);
        return;
    case R_RESET_DONE:
    case R_TAE_BITS:
        return;
    }
    s->regs[off >> 2] = val;
    switch (off) {
    case R_IRQ_ENABLE:
        fh_pae_update_irq(s);
        break;
    case R_CTRL:
        if (val & CTRL_RESET) {
            timer_del(s->timer);
            s->status = 0;
            s->hdr_bits = 0;
            s->regs[R_RESET_DONE >> 2] = 1;
            fh_pae_update_irq(s);
        }
        break;
    case R_TAE_CMD:
        if (val == CMD_HDR_RESET) {
            s->hdr_bits = 0;
            memset(s->hdr, 0, sizeof(s->hdr));
        } else if (val >= 1 && val <= 63 && (val & 1)) {
            fh_pae_push_bits(s, fh_pae_r(s, R_TAE_FIFO), (val + 1) / 2);
        } else if (val == CMD_START) {
            fh_pae_encode(s);
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      ENCODE_NS);
        }
        break;
    }
}

static const MemoryRegionOps fh_pae_ops = {
    .read = fh_pae_read,
    .write = fh_pae_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_pae_reset(DeviceState *dev)
{
    FhPaeState *s = FH_PAE(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    s->status = 0;
    s->hdr_bits = 0;
    for (i = 0; i < CONTEXTS; i++) {
        g_free(s->ref[i].y);
        g_free(s->ref[i].uv);
    }
    memset(s->ref, 0, sizeof(s->ref));
    timer_del(s->timer);
    fh_pae_update_irq(s);
}

static void fh_pae_init(Object *obj)
{
    FhPaeState *s = FH_PAE(obj);

    memory_region_init_io(&s->iomem, obj, &fh_pae_ops, s, TYPE_FH_PAE,
                          FH_PAE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fh_pae_done, s);
}

static void fh_pae_realize(DeviceState *dev, Error **errp)
{
    FhPaeState *s = FH_PAE(dev);

    if (!memory_region_init_ram(&s->work, OBJECT(dev), "fh-pae.working",
                                FH_PAE_WORK_SIZE, errp)) {
        return;
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->work);
}

/*
 * The references are not migrated: after a migration the next P picture
 * of a context is coded intra, which every decoder takes.
 */
static int fh_pae_post_load(void *opaque, int version_id)
{
    fh_pae_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_fh_pae = {
    .name = TYPE_FH_PAE,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = fh_pae_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, FhPaeState, FH_PAE_SIZE / 4),
        VMSTATE_UINT32(status, FhPaeState),
        VMSTATE_UINT8_ARRAY(hdr, FhPaeState, HDR_MAX),
        VMSTATE_UINT32(hdr_bits, FhPaeState),
        VMSTATE_TIMER_PTR(timer, FhPaeState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property fh_pae_props[] = {
    DEFINE_PROP_UINT32("dma-base", FhPaeState, dma_base, 0xA0000000),
};

static void fh_pae_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, fh_pae_props);
    device_class_set_legacy_reset(dc, fh_pae_reset);
    dc->realize = fh_pae_realize;
    dc->vmsd = &vmstate_fh_pae;
}

static const TypeInfo fh_pae_info = {
    .name          = TYPE_FH_PAE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FhPaeState),
    .instance_init = fh_pae_init,
    .class_init    = fh_pae_class_init,
};

static void fh_pae_register_types(void)
{
    type_register_static(&fh_pae_info);
}

type_init(fh_pae_register_types)
