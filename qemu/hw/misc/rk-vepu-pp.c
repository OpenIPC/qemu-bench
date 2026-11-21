/*
 * Rockchip RKVENC-PP ("rockchip,rkvenc-pp-rv1106", 0xFFA60000), the
 * encoder's pre-processor: smear, weighted prediction, motion and
 * occlusion detection for the smart features (rockit's IVS).  As the
 * kernel driver drives it (kmpp mpp/mpp_rkvenc_pp.c, public in
 * ticklab/rockchip-kmpp):
 *
 *   0x010 ENC_STRT   1 starts the job configured below
 *   0x020 INT_EN  0x024 INT_MSK  0x028 INT_CLR (write 1 to clear)
 *   0x02c INT_STA    bit 0 the job done (the driver's
 *                    PP_REG_RO_VALID_INT_STS), 4:2 bus errors
 *   0x034 ENC_PIC_FMT  0x038 ENC_PIC_RSL
 *   0x03c..0x054 reference / statistics buffers, 0x100.. the source
 *   0x200 smear, 0x400 motion detection, 0x500 occlusion detection,
 *   0x800.. results (read back by the driver)
 *
 * A job is done "job-ns" after it starts: INT_STA bit 0, the interrupt
 * while enabled and not masked.  Nothing is computed -- the motion and
 * occlusion outputs stay as they were (no motion, nothing covered); the
 * pipeline behind it runs instead of timing out.  Everything else is
 * storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_VEPU_PP "rk-vepu-pp"
OBJECT_DECLARE_SIMPLE_TYPE(RkVepuPpState, RK_VEPU_PP)

#define REGS_SIZE       0x1000
#define R_ENC_STRT      0x010
#define R_INT_EN        0x020
#define R_INT_MSK       0x024
#define R_INT_CLR       0x028
#define R_INT_STA       0x02c
#define INT_DONE        (1u << 0)

struct RkVepuPpState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t job_ns;            /* property: a job's time */
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_vepu_pp_update(RkVepuPpState *s)
{
    qemu_set_irq(s->irq, !!(REG(s, R_INT_STA) & REG(s, R_INT_EN) &
                            ~REG(s, R_INT_MSK)));
}

static void rk_vepu_pp_done(void *opaque)
{
    RkVepuPpState *s = opaque;

    REG(s, R_INT_STA) |= INT_DONE;
    rk_vepu_pp_update(s);
}

static uint64_t rk_vepu_pp_read(void *opaque, hwaddr off, unsigned size)
{
    RkVepuPpState *s = opaque;

    return s->regs[off / 4];
}

static void rk_vepu_pp_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    RkVepuPpState *s = opaque;

    switch (off) {
    case R_INT_CLR:
        REG(s, R_INT_STA) &= ~val;
        rk_vepu_pp_update(s);
        return;
    case R_INT_STA:
        return;                     /* read-only */
    case R_ENC_STRT:
        if (val & 1) {
            timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      s->job_ns);
        }
        return;
    }
    s->regs[off / 4] = val;
    rk_vepu_pp_update(s);
}

static const MemoryRegionOps rk_vepu_pp_ops = {
    .read = rk_vepu_pp_read,
    .write = rk_vepu_pp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_vepu_pp_reset(DeviceState *dev)
{
    RkVepuPpState *s = RK_VEPU_PP(dev);

    timer_del(s->done);
    memset(s->regs, 0, sizeof(s->regs));
    rk_vepu_pp_update(s);
}

static void rk_vepu_pp_init(Object *obj)
{
    RkVepuPpState *s = RK_VEPU_PP(obj);

    memory_region_init_io(&s->iomem, obj, &rk_vepu_pp_ops, s,
                          TYPE_RK_VEPU_PP, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_vepu_pp_done, s);
}

static const Property rk_vepu_pp_properties[] = {
    DEFINE_PROP_UINT32("job-ns", RkVepuPpState, job_ns, 500000),
};

static const VMStateDescription rk_vepu_pp_vmstate = {
    .name = TYPE_RK_VEPU_PP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkVepuPpState, REGS_SIZE / 4),
        VMSTATE_TIMER_PTR(done, RkVepuPpState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_vepu_pp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_vepu_pp_reset);
    device_class_set_props(dc, rk_vepu_pp_properties);
    dc->vmsd = &rk_vepu_pp_vmstate;
}

static const TypeInfo rk_vepu_pp_info = {
    .name = TYPE_RK_VEPU_PP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkVepuPpState),
    .instance_init = rk_vepu_pp_init,
    .class_init = rk_vepu_pp_class_init,
};

static void rk_vepu_pp_register_types(void)
{
    type_register_static(&rk_vepu_pp_info);
}

type_init(rk_vepu_pp_register_types)
