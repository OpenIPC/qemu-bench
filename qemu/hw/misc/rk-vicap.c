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
 * Everything else is storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_VICAP "rk-vicap"
OBJECT_DECLARE_SIMPLE_TYPE(RkVicapState, RK_VICAP)

#define REGS_SIZE       0x10000
#define R_GLB_INTST     0x008
#define R_TOISP0_CTRL   0x780
#define R_TOISP1_CTRL   0x78c

#define TOISP_FS(n)     (1u << (14 + 3 * (n)))
#define TOISP_END(n)    (1u << (20 + 3 * (n)))

struct RkVicapState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq toisp_on;

    uint32_t regs[REGS_SIZE / 4];
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_vicap_update(RkVicapState *s)
{
    qemu_set_irq(s->irq, !!REG(s, R_GLB_INTST));
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

    if (off == R_GLB_INTST) {
        REG(s, R_GLB_INTST) &= ~val;
        rk_vicap_update(s);
        return;
    }
    s->regs[off / 4] = val;
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

    memset(s->regs, 0, sizeof(s->regs));
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
}

static const VMStateDescription rk_vicap_vmstate = {
    .name = TYPE_RK_VICAP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkVicapState, REGS_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_vicap_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_vicap_reset);
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
