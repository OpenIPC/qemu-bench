/*
 * Grain Media scaler300 (FTSCALER300).
 *
 * The scaler at 0x96200000 on the GM813x (module/scaler300:
 * fscaler300_hw.c, fscaler300_drv.c, fscaler300_vg.c), which the IP camera
 * graph uses for frames the capture cannot produce, clear-window and
 * border fills.  Registers: per-job parameters at 0x000-0x1AF, globals at
 * 0x5000-0x55FF, link-list memory at 0x40000 (13 blocks of 0x100 and a
 * dummy block at 0x40D00).  A job, for each of up to four output paths i:
 *
 *   0x000      byte i: bit 0 bypass, 1 scale, 2 target crop
 *   0x010+4i   output width | height << 16
 *   0x160+8i   target crop x start | x end << 16, +4: y start | y end << 16
 *   0x180+8i   destination, 0x184+8i its line gap in 4-pixel units
 *   0x1A0      source (at the crop origin), 0x1A4 its line gap in pixels
 *   0x5200     source width | height << 16, bit 31 fill with 0x5204
 *   0x5204     the fill colour as packed Cb Y0 Cr Y1
 *
 * Pixels are packed 4:2:2 (UYVY).  The driver writes a dummy job into the
 * registers, the first chained job's block offset into 0x1A8, and fires
 * (0x5404 bit 28, which the engine clears).  The engine runs the register
 * job, then the list at 0x40000 + 0x1A8: 64-bit commands, low word data,
 * high word command by bits 31:29 --
 *
 *   001  status slot             010  register write, byte enables 22:19,
 *   100  end of job, next at       register 15:0
 *        bits 18:0              101  the job continues at bits 18:0
 *   111  continue at the data    000  (all zero) end of the chain
 *
 * At each job end the job is executed and its status slot gets 0x20000001
 * | the finished outputs << 4; the chain's end raises list done (0x55B0
 * bit 0, mask 0x5530, write 1 to clear) on interrupt 12.  A driver that
 * extends a stopped chain by patching its end gets it run too.  Scaling
 * is nearest-neighbour.
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
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_GM_SCALER "gm-scaler"
OBJECT_DECLARE_SIMPLE_TYPE(GmScalerState, GM_SCALER)

#define GM_SCL_SIZE         0x48000
#define GM_SCL_NREGS        (GM_SCL_SIZE / 4)
#define GM_SCL_LL_BASE      0x40000
#define GM_SCL_JOB_NS       1000000
#define GM_SCL_MAX_CMDS     4096
#define GM_SCL_PATHS        4

#define R_PATH_CTRL         0x000
#define R_OUT_SIZE(i)       (0x010 + 4 * (i))
#define R_TC_X(i)           (0x160 + 8 * (i))
#define R_TC_Y(i)           (0x164 + 8 * (i))
#define R_DST(i)            (0x180 + 8 * (i))
#define R_DST_GAP(i)        (0x184 + 8 * (i))
#define R_SRC               0x1A0
#define R_SRC_GAP           0x1A4
#define R_NEXT              0x1A8
#define R_SRC_SIZE          0x5200
#define R_FILL              0x5204
#define R_GLOBAL            0x5404
#define R_STS_ERR           0x55A8
#define R_STS_DONE          0x55B0
#define R_STS_FAIL          0x55B8

#define GLOBAL_FIRE         (1u << 28)

struct GmScalerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_SCL_NREGS];
    bool running;
    bool stopped;           /* at the chain end at pc */
    uint32_t pc;
    uint32_t status_slot;
};

static void gm_scaler_update(GmScalerState *s)
{
    const uint32_t done = s->regs[R_STS_DONE / 4] &
                          ~s->regs[(R_STS_DONE - 0x80) / 4];
    const uint32_t err = s->regs[R_STS_ERR / 4] &
                         ~s->regs[(R_STS_ERR - 0x80) / 4];

    qemu_set_irq(s->irq, done || err);
}

static void gm_scaler_path(GmScalerState *s, int i, const uint8_t *src,
                           int sw, int sh, int sstride, bool fill)
{
    const uint32_t ctl = extract32(s->regs[R_PATH_CTRL / 4], i * 8, 8);
    const uint32_t osz = s->regs[R_OUT_SIZE(i) / 4];
    int ow = extract32(osz, 0, 13), oh = extract32(osz, 16, 13);
    int x0 = 0, x1, y0 = 0, y1, x, y, w, h;
    g_autofree uint8_t *line = NULL;
    hwaddr dst = s->regs[R_DST(i) / 4];
    size_t dstride;

    if (ctl & 1) {                          /* bypass: the source size */
        ow = sw;
        oh = sh;
    }
    if (!ow || !oh) {
        return;
    }
    x1 = ow - 1;
    y1 = oh - 1;
    if (ctl & 4) {                          /* target crop */
        x0 = extract32(s->regs[R_TC_X(i) / 4], 0, 13);
        x1 = extract32(s->regs[R_TC_X(i) / 4], 16, 13);
        y0 = extract32(s->regs[R_TC_Y(i) / 4], 0, 13);
        y1 = extract32(s->regs[R_TC_Y(i) / 4], 16, 13);
    }
    w = (x1 - x0 + 1) & ~1;
    h = y1 - y0 + 1;
    if (w <= 0 || h <= 0 || w > 8192 || h > 8192) {
        return;
    }
    dstride = (size_t)w * 2 + extract32(s->regs[R_DST_GAP(i) / 4], 0, 13) * 8;
    line = g_malloc(w * 2);
    for (y = 0; y < h; y++) {
        const int sy = sh ? MIN((int64_t)(y + y0) * sh / oh, sh - 1) : 0;

        for (x = 0; x < w; x += 2) {
            uint8_t *d = line + x * 2;

            if (fill || !src) {
                stl_le_p(d, s->regs[R_FILL / 4]);
            } else {
                /* nearest source pixel pair, keeping its chroma */
                const int sx = MIN((int64_t)(x + x0) * sw / ow, sw - 1) & ~1;
                memcpy(d, src + (size_t)sy * sstride + sx * 2, 4);
            }
        }
        dma_memory_write(&address_space_memory, dst + y * dstride, line, w * 2,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/* Execute the job in the registers; returns the finished outputs. */
static uint32_t gm_scaler_job(GmScalerState *s)
{
    const uint32_t ssz = s->regs[R_SRC_SIZE / 4];
    const int sw = extract32(ssz, 0, 13), sh = extract32(ssz, 16, 13);
    const bool fill = ssz & (1u << 31);
    const size_t sstride = ((size_t)sw + extract32(s->regs[R_SRC_GAP / 4], 0,
                                                   16)) * 2;
    g_autofree uint8_t *src = NULL;
    uint32_t outs = 0;
    int i;

    if (!fill && sw && sh && sstride * sh <= 64 * MiB) {
        src = g_malloc0(sstride * sh + 4);  /* an odd width's last pair */
        dma_memory_read(&address_space_memory, s->regs[R_SRC / 4], src,
                        sstride * sh, MEMTXATTRS_UNSPECIFIED);
    }
    for (i = 0; i < GM_SCL_PATHS; i++) {
        const uint32_t ctl = extract32(s->regs[R_PATH_CTRL / 4], i * 8, 8);

        if (ctl & 3) {
            gm_scaler_path(s, i, src, sw, sh, sstride, fill);
            if (ctl & 2) {
                outs |= 1u << i;
            }
        }
    }
    return outs;
}

static void gm_scaler_finish_job(GmScalerState *s)
{
    const uint32_t outs = gm_scaler_job(s);

    if (s->status_slot) {
        s->regs[s->status_slot / 4] = 0;
        s->regs[s->status_slot / 4 + 1] = 0x20000001 | (outs << 4);
        s->status_slot = 0;
    }
}

static void gm_scaler_run(GmScalerState *s)
{
    int n;

    for (n = 0; n < GM_SCL_MAX_CMDS; n++) {
        uint32_t lo, hi, reg, be, v;
        int b;

        if (s->pc < GM_SCL_LL_BASE || s->pc + 8 > GM_SCL_SIZE) {
            break;
        }
        lo = s->regs[s->pc / 4];
        hi = s->regs[s->pc / 4 + 1];
        if (!hi) {                              /* end of the chain */
            gm_scaler_finish_job(s);
            s->stopped = true;
            s->regs[R_STS_DONE / 4] |= 1;
            gm_scaler_update(s);
            return;
        }
        switch (hi >> 29) {
        case 1:                                 /* status slot */
            s->status_slot = s->pc;
            s->pc += 8;
            break;
        case 2:                                 /* register write */
            reg = hi & 0xFFFC;
            be = extract32(hi, 19, 4);
            v = s->regs[reg / 4];
            for (b = 0; b < 4; b++) {
                if (be & (1u << b)) {
                    v = deposit32(v, b * 8, 8, extract32(lo, b * 8, 8));
                }
            }
            s->regs[reg / 4] = v;
            s->pc += 8;
            break;
        case 4:                                 /* end of job */
            gm_scaler_finish_job(s);
            s->pc = hi & 0x7FFF8;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                GM_SCL_JOB_NS);
            return;
        case 5:                                 /* job continues */
            s->pc = hi & 0x7FFF8;
            break;
        case 7:                                 /* continue at the data */
            s->pc = GM_SCL_LL_BASE + (lo & 0x7FF8);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "gm-scaler: bad command 0x%08x at 0x%x\n", hi, s->pc);
            s->regs[R_STS_FAIL / 4] |= 1;
            s->running = false;
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "gm-scaler: runaway list at 0x%x\n", s->pc);
    s->running = false;
}

static void gm_scaler_step(void *opaque)
{
    GmScalerState *s = opaque;

    if (s->running && !s->stopped) {
        gm_scaler_run(s);
    }
}

static void gm_scaler_fire(GmScalerState *s)
{
    gm_scaler_job(s);                           /* the job in the registers */
    s->running = true;
    s->stopped = false;
    s->status_slot = 0;
    s->pc = GM_SCL_LL_BASE + (s->regs[R_NEXT / 4] & 0x7FF8);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        GM_SCL_JOB_NS);
}

static uint64_t gm_scaler_read(void *opaque, hwaddr off, unsigned size)
{
    GmScalerState *s = opaque;

    return s->regs[off / 4];
}

static void gm_scaler_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    GmScalerState *s = opaque;

    switch (off) {
    case R_GLOBAL:
        s->regs[off / 4] = val & ~GLOBAL_FIRE;
        if (val & GLOBAL_FIRE) {
            gm_scaler_fire(s);
        }
        return;
    case R_STS_ERR:
    case R_STS_DONE:
    case R_STS_FAIL:
        s->regs[off / 4] &= ~val;
        gm_scaler_update(s);
        return;
    }
    s->regs[off / 4] = val;
    if (off >= 0x5500 && off < 0x5580) {        /* the masks */
        gm_scaler_update(s);
    }
    /* a chain extended at the end the engine stopped on */
    if (s->running && s->stopped && off == s->pc + 4 && (val >> 29) == 4) {
        /* that job already ran: carry on with the next one */
        s->stopped = false;
        s->pc = val & 0x7FFF8;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            GM_SCL_JOB_NS);
    }
}

static const MemoryRegionOps gm_scaler_ops = {
    .read = gm_scaler_read,
    .write = gm_scaler_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_scaler_reset(DeviceState *dev)
{
    GmScalerState *s = GM_SCALER(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->running = s->stopped = false;
    s->pc = GM_SCL_LL_BASE;
    s->status_slot = 0;
    gm_scaler_update(s);
}

static void gm_scaler_realize(DeviceState *dev, Error **errp)
{
    GmScalerState *s = GM_SCALER(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_scaler_ops, s,
                          TYPE_GM_SCALER, GM_SCL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_scaler_step, s);
}

static const VMStateDescription vmstate_gm_scaler = {
    .name = TYPE_GM_SCALER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmScalerState),
        VMSTATE_UINT32_ARRAY(regs, GmScalerState, GM_SCL_NREGS),
        VMSTATE_BOOL(running, GmScalerState),
        VMSTATE_BOOL(stopped, GmScalerState),
        VMSTATE_UINT32(pc, GmScalerState),
        VMSTATE_UINT32(status_slot, GmScalerState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_scaler_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_scaler_realize;
    device_class_set_legacy_reset(dc, gm_scaler_reset);
    dc->vmsd = &vmstate_gm_scaler;
}

static const TypeInfo gm_scaler_info = {
    .name           = TYPE_GM_SCALER,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmScalerState),
    .class_init     = gm_scaler_class_init,
};

static void gm_scaler_register_types(void)
{
    type_register_static(&gm_scaler_info);
}

type_init(gm_scaler_register_types)
