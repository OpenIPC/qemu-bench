/*
 * Rockchip VICAP ("rockchip,rv1106-cif", 0xFFA10000), the RV1106's video
 * capture front end, as far as its online path to the ISP goes.  With the
 * sensor feeding VICAP (not the ISP's own MIPI), the vendor 3A library
 * (rkaiq CamHwIsp20) takes its start-of-frame events from the CSI-2
 * subdevice, and the rkcif driver queues those from VICAP's "to ISP frame
 * start" interrupt (capture.c rkcif_irq_handle_toisp -> rkcif_send_sof).
 * Without them the library's frame-sync count stays at 0, every
 * statistics buffer is "delta > 3" ahead and skipped, and AE never runs.
 *
 *   0x008 GLB_INTST   write 1 to clear (the driver writes back what it
 *                     read): bit 14 + 3n TOISPn FS ch0, 20 + 3n END ch0
 *   0x780 TOISP0_CH_CTRL  0x78c TOISP1_CH_CTRL: bit 0 enable (the driver
 *                     checks the channel's source ID in bits 7:3 itself)
 *
 * The ISP model drives "frame-start" / "frame-end" (GPIO inputs) at its
 * own frame start and end; each sets the bits of every enabled TOISP
 * channel and raises the interrupt (SPI 106).  "toisp-on" (GPIO out) is
 * the level of any TOISP channel enabled: an online ISP sees frames only
 * while VICAP sends them, so its frame count and VICAP's start together
 * -- the 3A library pairs statistics with frame syncs by those counts.
 *
 * With more than one sensor on the ISP the driver runs it from memory
 * instead (the ISP's "Isp Read" mode, rkcif RKISP_VICAP_RDBK_AUTO): each
 * MIPI host captures its ID0 channel to DDR, ping-pong between two
 * buffers, and the driver hands each full buffer to the ISP to read back.
 * The hosts' blocks are at 0x100 and 0x300 (CSI_MIPI0_*):
 *   +0x00 ID0_CTRL0   bit 0 capture enable
 *   +0x24 FRM0_ADDR_Y_ID0, +0x28 FRM1_ADDR_Y_ID0: a buffer address makes
 *                     it a DMA capture (online TOISP capture leaves 0)
 *   +0x74 INTEN, +0x78 INTSTAT (write 1 to clear): bit 0 + p frame p
 *                     start, bit 8 + p frame p end, for ID0
 *   +0x9c FRAME_NUM_VC0: counts frame starts
 * An enabled DMA capture runs at "frame-ns" (30 fps): start into buffer
 * p, end FRAME_ACTIVE of the period later, then the other buffer.  The
 * raw frame isn't written: the ISP draws its own picture on read-back.
 * Everything else is storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_VICAP "rk-vicap"
OBJECT_DECLARE_SIMPLE_TYPE(RkVicapState, RK_VICAP)

#define REGS_SIZE       0x10000
#define R_GLB_INTST     0x008
#define R_TOISP0_CTRL   0x780
#define R_TOISP1_CTRL   0x78c

#define MIPI_HOSTS      2
#define MIPI_BASE(h)    (0x100 + 0x200 * (h))
#define M_ID0_CTRL0     0x00
#define M_FRM0_ADDR     0x24
#define M_FRM1_ADDR     0x28
#define M_INTEN         0x74
#define M_INTSTAT       0x78
#define M_FRAME_NUM     0x9c
#define MREG(s, h, r)   REG(s, MIPI_BASE(h) + (r))
#define CSI_FS(p)       (1u << (p))
#define CSI_FE(p)       (1u << (8 + (p)))

#define TOISP_FS(n)     (1u << (14 + 3 * (n)))
#define TOISP_END(n)    (1u << (20 + 3 * (n)))

struct RkVicapState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq toisp_on;

    uint32_t regs[REGS_SIZE / 4];
    QEMUTimer *timer[MIPI_HOSTS];
    bool in_frame[MIPI_HOSTS];
    uint32_t phase[MIPI_HOSTS];     /* the buffer being written: 0 or 1 */
    uint64_t frame_ns;              /* property: a frame's period */
};



#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_vicap_update(RkVicapState *s)
{
    bool level = REG(s, R_GLB_INTST);
    int h;

    for (h = 0; h < MIPI_HOSTS; h++) {
        level |= !!(MREG(s, h, M_INTSTAT) & MREG(s, h, M_INTEN));
    }
    qemu_set_irq(s->irq, level);
}

/* the active part of a frame; the rest is vertical blanking */
#define FRAME_ACTIVE(ns)    ((ns) * 9 / 10)

static bool rk_vicap_dma_on(RkVicapState *s, int h)
{
    return (MREG(s, h, M_ID0_CTRL0) & 1) && MREG(s, h, M_FRM0_ADDR);
}

static void rk_vicap_host_tick(RkVicapState *s, int h)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!rk_vicap_dma_on(s, h)) {
        s->in_frame[h] = false;
        return;
    }
    if (!s->in_frame[h]) {
        MREG(s, h, M_INTSTAT) |= CSI_FS(s->phase[h]);
        MREG(s, h, M_FRAME_NUM)++;
        s->in_frame[h] = true;
        timer_mod(s->timer[h], now + FRAME_ACTIVE(s->frame_ns));
    } else {
        MREG(s, h, M_INTSTAT) |= CSI_FE(s->phase[h]);
        s->phase[h] ^= 1;
        s->in_frame[h] = false;
        timer_mod(s->timer[h], now + s->frame_ns - FRAME_ACTIVE(s->frame_ns));
    }
    rk_vicap_update(s);
}

static void rk_vicap_tick0(void *opaque)
{
    rk_vicap_host_tick(opaque, 0);
}

static void rk_vicap_tick1(void *opaque)
{
    rk_vicap_host_tick(opaque, 1);
}

/* a register write may start a host's capture */
static void rk_vicap_host_check(RkVicapState *s, int h)
{
    if (rk_vicap_dma_on(s, h) && !timer_pending(s->timer[h])) {
        s->in_frame[h] = false;
        s->phase[h] = 0;
        timer_mod(s->timer[h], qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  s->frame_ns - FRAME_ACTIVE(s->frame_ns));
    }
}

/* frame start (end) at the ISP: each enabled TOISP channel reports it */
static void rk_vicap_frame(RkVicapState *s, bool end)
{
    static const uint32_t ctrl[2] = { R_TOISP0_CTRL, R_TOISP1_CTRL };
    int n;

    for (n = 0; n < 2; n++) {
        if (REG(s, ctrl[n]) & 1) {
            REG(s, R_GLB_INTST) |= end ? TOISP_END(n) : TOISP_FS(n);
        }
    }
    rk_vicap_update(s);
}

static void rk_vicap_fs_in(void *opaque, int n, int level)
{
    if (level) {
        rk_vicap_frame(opaque, false);
    }
}

static void rk_vicap_fe_in(void *opaque, int n, int level)
{
    if (level) {
        rk_vicap_frame(opaque, true);
    }
}

static uint64_t rk_vicap_read(void *opaque, hwaddr off, unsigned size)
{
    RkVicapState *s = opaque;

    return s->regs[off / 4];
}

static void rk_vicap_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkVicapState *s = opaque;
    int h;

    if (off == R_GLB_INTST) {
        REG(s, R_GLB_INTST) &= ~val;
        rk_vicap_update(s);
        return;
    }
    for (h = 0; h < MIPI_HOSTS; h++) {
        if (off == MIPI_BASE(h) + M_INTSTAT) {
            MREG(s, h, M_INTSTAT) &= ~val;
            rk_vicap_update(s);
            return;
        }
    }
    s->regs[off / 4] = val;
    for (h = 0; h < MIPI_HOSTS; h++) {
        if (off >= MIPI_BASE(h) && off < MIPI_BASE(h) + 0x100) {
            rk_vicap_host_check(s, h);
            rk_vicap_update(s);
        }
    }
    if (off == R_TOISP0_CTRL || off == R_TOISP1_CTRL) {
        qemu_set_irq(s->toisp_on, ((REG(s, R_TOISP0_CTRL) |
                                    REG(s, R_TOISP1_CTRL)) & 1));
    }
}

static const MemoryRegionOps rk_vicap_ops = {
    .read = rk_vicap_read,
    .write = rk_vicap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_vicap_reset(DeviceState *dev)
{
    RkVicapState *s = RK_VICAP(dev);

    int h;

    memset(s->regs, 0, sizeof(s->regs));
    for (h = 0; h < MIPI_HOSTS; h++) {
        timer_del(s->timer[h]);
        s->in_frame[h] = false;
        s->phase[h] = 0;
    }
    rk_vicap_update(s);
    qemu_set_irq(s->toisp_on, 0);
}

static void rk_vicap_init(Object *obj)
{
    RkVicapState *s = RK_VICAP(obj);

    memory_region_init_io(&s->iomem, obj, &rk_vicap_ops, s, TYPE_RK_VICAP,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), rk_vicap_fs_in, "frame-start", 1);
    qdev_init_gpio_in_named(DEVICE(obj), rk_vicap_fe_in, "frame-end", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->toisp_on, "toisp-on", 1);
    s->timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_vicap_tick0, s);
    s->timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_vicap_tick1, s);
}

static const Property rk_vicap_properties[] = {
    DEFINE_PROP_UINT64("frame-ns", RkVicapState, frame_ns,
                       NANOSECONDS_PER_SECOND / 30),
};

static const VMStateDescription rk_vicap_vmstate = {
    .name = TYPE_RK_VICAP,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkVicapState, REGS_SIZE / 4),
        VMSTATE_TIMER_PTR_ARRAY(timer, RkVicapState, MIPI_HOSTS),
        VMSTATE_BOOL_ARRAY(in_frame, RkVicapState, MIPI_HOSTS),
        VMSTATE_UINT32_ARRAY(phase, RkVicapState, MIPI_HOSTS),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_vicap_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_vicap_reset);
    device_class_set_props(dc, rk_vicap_properties);
    dc->vmsd = &rk_vicap_vmstate;
}

static const TypeInfo rk_vicap_info = {
    .name = TYPE_RK_VICAP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkVicapState),
    .instance_init = rk_vicap_init,
    .class_init = rk_vicap_class_init,
};

static void rk_vicap_register_types(void)
{
    type_register_static(&rk_vicap_info);
}

type_init(rk_vicap_register_types)
