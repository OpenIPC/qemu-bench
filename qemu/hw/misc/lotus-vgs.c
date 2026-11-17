/*
 * XMedia "lotus" SPC030 VGS (Goke GK7206 / xmorca): the scaler the stock
 * firmware's VPSS uses for its extended channels (ext ochn 3 / 5, scaled
 * from ochn 2), as xm_vgs.ko drives it.
 *
 * VGS (0x11300000, SPI 44 "VGS"; xm_vgs hal_vgs_*, vgs_isr_main): a job
 * is a list of register images ("nodes") in DDR, started in list mode:
 *   0x010  CTRL     bit 0 ctl_up_finish, 4 filter_up_finish (start, read
 *                   back 0 when done), 8 list mode
 *   0x014  INT_MASK 1 = masked, bit-aligned with INT from bit 4 up
 *                   (hal_vgs_set_int_mask; hal_vgs_startup_task writes
 *                   0x703f: list done and the errors enabled)
 *   0x018  INT      write the read value back to clear: bit 0 / 1 / 2
 *                   early 0 / 1 / 2 (masked by 0x280 bits 0 / 4 / 8, not
 *                   by 0x014), 4 frame start, 5 frame end, 6 list done
 *                   (the job is finished: event 2 to VPSS), 12..14 early
 *                   overflow, 21 / 22 AXI write / read error, 23 / 24
 *                   blank / valid timeout
 *   0x01c           AXI outstanding (hal_vgs_set_ost, 0x1004 / 0x1008),
 *                   not an interrupt register
 *   0x184  node id, 0x188.. node addresses, 0x194 / 0x198 working /
 *          finished node id
 * A node block (hal_vgs_set_next_node_addr): the register image starts at
 * its +0x180 (the cnfg base the driver puts in 0x188 holds register 0x180
 * at offset 0), and each image's own 0x188 links the next one
 * (0xffffffff ends the job).  The scaler's registers in it (hal_reg_vgs_*):
 *   0x200  bits 2:0 mode (0: scale / copy), 3 core type, 8 crop in,
 *          9 crop out, 10 / 11 packed 422 in / out, 12 / 13 swap U and V
 *          in / out (the stock VPSS asks for NV12 from its NV21 that way),
 *          17 RGB out
 *   0x210 / 0x214  ch0 (luma) in: bit 0 enable, 6 flip, 31:16 stride in
 *                  8 bytes; address
 *   0x218 / 0x21c  ch1 (chroma) in: also 8 / 9 half width / height
 *   0x228 / 0x22c, 0x230 / 0x234  ch0 / ch1 out, likewise (+ bit 7 mirror)
 *   0x250  input width bits 13:0, height 29:16; 0x254 crop start x / y,
 *   0x258  the valid (cropped) input size, 0x25c output crop start,
 *   0x260  output size
 * When the job ends the scaled picture is written (bilinear, semi-planar
 * 4:2:0 or 4:2:2 as the chroma channels' half-height bits say).  A job
 * completes 4 ms after it starts (the real scaler took ~4.1 ms for
 * 800x448 -> 352x288, /proc/umap/vgs hw_time).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "system/dma.h"
#include "system/address-spaces.h"

#define TYPE_LOTUS_VGS "lotus-vgs"
OBJECT_DECLARE_SIMPLE_TYPE(LotusVgsState, LOTUS_VGS)

#define VGS_SIZE        0x10000
#define VGS_CTRL        0x010
#define VGS_CTRL_START  0x11u
#define VGS_INT_MASK    0x014
#define VGS_INT         0x018
#define VGS_EARLY_MASK  0x280
#define VGS_NODE_ID     0x184
#define VGS_WORK_NODE   0x194
#define VGS_FINISH_NODE 0x198
#define VGS_INT_DONE    (1u << 6)
#define VGS_JOB_NS      (4 * SCALE_MS)
#define VGS_CNFG_BASE   0x188
#define VGS_IMG_OFF     0x180       /* the image holds registers from here */
#define VGS_MAX_NODES   64
#define VGS_PIC_MAX     8192

struct LotusVgsState {
    SysBusDevice parent_obj;

    MemoryRegion mr;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t *regs;
    bool trace;
    bool scale;
    bool verbose;
    uint32_t logged;
};

/* a node image's register */
static uint32_t vgs_reg(const uint32_t *img, uint32_t off)
{
    return le32_to_cpu(img[(off - VGS_IMG_OFF) / 4]);
}

static bool vgs_ram(uint32_t a, uint64_t len)
{
    return a >= 0x40000000 && len && len <= 64 * MiB &&
           (uint64_t)a + len <= 0x100000000ull;
}

/*
 * Scale a plane of n-byte pixels (1: luma, 2: a chroma pair) bilinearly:
 * sw x sh at src (stride ss) to dw x dh rows of dst (stride ds).
 */
static void vgs_scale_plane(uint32_t src, uint32_t ss, int sw, int sh,
                            uint32_t dst, uint32_t ds, int dw, int dh, int n,
                            bool swap, bool mirror, bool flip)
{
    g_autofree uint8_t *in = NULL, *out = NULL;
    int x, y, k;

    if (!vgs_ram(src, (uint64_t)ss * sh) || !vgs_ram(dst, (uint64_t)ds * dh) ||
        ss < (uint32_t)sw * n || ds < (uint32_t)dw * n) {
        return;
    }
    in = g_malloc((size_t)ss * sh);
    out = g_malloc((size_t)dw * n);
    dma_memory_read(&address_space_memory, src, in, (size_t)ss * sh,
                    MEMTXATTRS_UNSPECIFIED);
    for (y = 0; y < dh; y++) {
        /* centres aligned: (y + 0.5) * sh / dh - 0.5, 16.16 */
        const int64_t fy = MAX(((2 * y + 1) * (int64_t)sh * 65536 / dh -
                                65536) / 2, 0);
        const int y0 = MIN(fy >> 16, sh - 1), y1 = MIN(y0 + 1, sh - 1);
        const int wy = (fy & 0xffff) >> 8;
        const uint8_t *r0 = in + (size_t)y0 * ss, *r1 = in + (size_t)y1 * ss;

        for (x = 0; x < dw; x++) {
            const int64_t fx = MAX(((2 * x + 1) * (int64_t)sw * 65536 / dw -
                                    65536) / 2, 0);
            const int x0 = MIN(fx >> 16, sw - 1), x1 = MIN(x0 + 1, sw - 1);
            const int wx = (fx & 0xffff) >> 8;
            const int ox = mirror ? dw - 1 - x : x;

            for (k = 0; k < n; k++) {
                const int a = r0[x0 * n + k] * (256 - wx) + r0[x1 * n + k] * wx;
                const int b = r1[x0 * n + k] * (256 - wx) + r1[x1 * n + k] * wx;
                const int ok = swap ? n - 1 - k : k;

                out[ox * n + ok] = (a * (256 - wy) + b * wy + 32768) >> 16;
            }
        }
        dma_memory_write(&address_space_memory,
                         dst + (uint64_t)(flip ? dh - 1 - y : y) * ds, out,
                         (size_t)dw * n, MEMTXATTRS_UNSPECIFIED);
    }
}

/* one node: a scale (or copy) of a semi-planar YUV picture */
static void vgs_node(LotusVgsState *s, const uint32_t *img)
{
    const uint32_t mode = vgs_reg(img, 0x200);
    const uint32_t yin = vgs_reg(img, 0x210), cin = vgs_reg(img, 0x218);
    const uint32_t yout = vgs_reg(img, 0x228), cout = vgs_reg(img, 0x230);
    const uint32_t isz = vgs_reg(img, 0x250), osz = vgs_reg(img, 0x260);
    const uint32_t crop = vgs_reg(img, 0x254), vsz = vgs_reg(img, 0x258);
    int iw = isz & 0x3fff, ih = (isz >> 16) & 0x3fff;
    const int ow = osz & 0x3fff, oh = (osz >> 16) & 0x3fff;
    int cx = 0, cy = 0;
    uint32_t ya = vgs_reg(img, 0x214), ca = vgs_reg(img, 0x21c);
    const uint32_t ys = (yin >> 16) * 8, cs = (cin >> 16) * 8;
    const uint32_t oys = (yout >> 16) * 8, ocs = (cout >> 16) * 8;
    const bool mirror = yout & (1u << 7);
    const bool flip = ((yout >> 6) ^ (yin >> 6)) & 1;

    if (s->verbose && s->logged < 32) {
        s->logged++;
        qemu_log("lotus-vgs: node mode %08x in %dx%d crop %08x/%08x y %08x@%08x"
                 " c %08x@%08x out %dx%d y %08x@%08x c %08x@%08x\n", mode, iw,
                 ih, crop, vsz, yin, ya, cin, ca, ow, oh, yout,
                 vgs_reg(img, 0x22c), cout, vgs_reg(img, 0x234));
    }
    /* scaling of YUV only: no packed, RGB, or other operations */
    if ((mode & 7) || (mode & ((3u << 10) | (1u << 17) | (1u << 9))) ||
        !(yin & 1) || !(yout & 1) || iw < 2 || ih < 2 || ow < 2 || oh < 2 ||
        iw > VGS_PIC_MAX || ih > VGS_PIC_MAX || ow > VGS_PIC_MAX ||
        oh > VGS_PIC_MAX) {
        return;
    }
    if (mode & (1u << 8)) {
        cx = crop & 0x3fff;
        cy = (crop >> 16) & 0x3fff;
        iw = MIN(vsz & 0x3fff, iw - cx);
        ih = MIN((vsz >> 16) & 0x3fff, ih - cy);
        if (iw < 2 || ih < 2) {
            return;
        }
    }
    vgs_scale_plane(ya + cy * ys + cx, ys, iw, ih, vgs_reg(img, 0x22c), oys,
                    ow, oh, 1, false, mirror, flip);
    if ((cin & 1) && (cout & 1)) {
        /* chroma: half width (pairs); half height as each side has it */
        const int icy = cin & (1u << 9) ? 2 : 1, ocy = cout & (1u << 9) ? 2 : 1;

        ca += (cy / icy) * cs + (cx & ~1);
        vgs_scale_plane(ca, cs, iw / 2, ih / icy, vgs_reg(img, 0x234), ocs,
                        ow / 2, oh / ocy, 2, ((mode >> 12) ^ (mode >> 13)) & 1,
                        mirror, flip);
    }
}

/* the job the driver started: its node list */
static void vgs_run(LotusVgsState *s)
{
    uint32_t a = s->regs[VGS_CNFG_BASE >> 2];
    uint32_t img[(0x300 - VGS_IMG_OFF) / 4];

    for (int n = 0; n < VGS_MAX_NODES && a && a != 0xffffffff; n++) {
        if (!vgs_ram(a, sizeof(img))) {
            break;
        }
        dma_memory_read(&address_space_memory, a, img, sizeof(img),
                        MEMTXATTRS_UNSPECIFIED);
        vgs_node(s, img);
        a = vgs_reg(img, VGS_CNFG_BASE);
    }
}

static void vgs_trace(LotusVgsState *s, char rw, hwaddr off, uint64_t val)
{
    if (s->trace) {
        qemu_log("%" PRId64 " lotus-vgs %c %05" HWADDR_PRIx " %08" PRIx64
                 "\n", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), rw, off, val);
    }
}

static void vgs_update(LotusVgsState *s)
{
    uint32_t em = s->regs[VGS_EARLY_MASK >> 2];
    uint32_t on = (~s->regs[VGS_INT_MASK >> 2] & ~7u) |
                  (~em & 1) | (~em >> 3 & 2) | (~em >> 6 & 4);

    qemu_set_irq(s->irq, !!(s->regs[VGS_INT >> 2] & on));
}

static void vgs_done(void *opaque)
{
    LotusVgsState *s = opaque;

    vgs_trace(s, 'D', 0, s->regs[VGS_NODE_ID >> 2]);
    if (s->scale) {
        vgs_run(s);
    }
    s->regs[VGS_CTRL >> 2] &= ~VGS_CTRL_START;
    s->regs[VGS_WORK_NODE >> 2] = s->regs[VGS_NODE_ID >> 2];
    s->regs[VGS_FINISH_NODE >> 2] = s->regs[VGS_NODE_ID >> 2];
    s->regs[VGS_INT >> 2] |= VGS_INT_DONE;
    vgs_update(s);
}

static uint64_t vgs_read(void *opaque, hwaddr off, unsigned size)
{
    LotusVgsState *s = opaque;
    uint32_t v = s->regs[off >> 2];

    vgs_trace(s, 'R', off, v);
    return v;
}

static void vgs_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    LotusVgsState *s = opaque;

    vgs_trace(s, 'W', off, val);
    if (off == VGS_INT) {
        s->regs[off >> 2] &= ~val;
        vgs_update(s);
        return;
    }
    if (off == VGS_CTRL && (val & VGS_CTRL_START) &&
        !(s->regs[off >> 2] & VGS_CTRL_START)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  VGS_JOB_NS);
    }
    s->regs[off >> 2] = val;
    if (off == VGS_INT_MASK || off == VGS_EARLY_MASK) {
        vgs_update(s);
    }
}

static const MemoryRegionOps vgs_ops = {
    .read = vgs_read,
    .write = vgs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void lotus_vgs_reset(DeviceState *dev)
{
    LotusVgsState *s = LOTUS_VGS(dev);

    timer_del(s->timer);
    memset(s->regs, 0, VGS_SIZE);
    qemu_irq_lower(s->irq);
}

static void lotus_vgs_realize(DeviceState *dev, Error **errp)
{
    LotusVgsState *s = LOTUS_VGS(dev);

    s->regs = g_new0(uint32_t, VGS_SIZE / 4);
    memory_region_init_io(&s->mr, OBJECT(s), &vgs_ops, s, "lotus-vgs",
                          VGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mr);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vgs_done, s);
}

static const Property lotus_vgs_properties[] = {
    DEFINE_PROP_BOOL("trace", LotusVgsState, trace, false),
    DEFINE_PROP_BOOL("scale", LotusVgsState, scale, true),
    DEFINE_PROP_BOOL("verbose", LotusVgsState, verbose, false),
};

static void lotus_vgs_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = lotus_vgs_realize;
    device_class_set_legacy_reset(dc, lotus_vgs_reset);
    device_class_set_props(dc, lotus_vgs_properties);
    dc->desc = "XMedia lotus SPC030 VGS";
}

static const TypeInfo lotus_vgs_info = {
    .name = TYPE_LOTUS_VGS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LotusVgsState),
    .class_init = lotus_vgs_class_init,
};

static void lotus_vgs_register(void)
{
    type_register_static(&lotus_vgs_info);
}
type_init(lotus_vgs_register)
