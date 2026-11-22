/*
 * The DDR controller (Synopsys DW uMCTL2, the RV1103B's at 0x20C00000) as
 * far as the kernel's suspend code drives it.  Before sleep it takes the
 * DRAM out of the controller's own low-power modes (PWRCTL 0x30 bits 1:0
 * off) and waits for STAT (0x04) to read "normal"; it then asks for
 * self-refresh by software (PWRCTL bit 5, selfref_sw) and waits for
 * "self refresh" (operating mode 3, selfref_type 2 "under software
 * control"), and on the way back clears it and waits for normal again.
 * The DRAM is always there, so STAT follows the request at once.
 * Everything else is storage.
 *
 *   0x04 STAT    bits 2:0 operating mode (1 normal, 3 self refresh),
 *                bits 5:4 selfref_type
 *   0x30 PWRCTL  bit 5 selfref_sw
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_DDRC "rk-ddrc"
OBJECT_DECLARE_SIMPLE_TYPE(RkDdrcState, RK_DDRC)

#define DDRC_SIZE       0x4000
#define R_STAT          0x04
#define R_PWRCTL        0x30
#define PWRCTL_SELFREF_SW   (1u << 5)
#define STAT_NORMAL         1
#define STAT_SELFREF_SW     (3 | 2 << 4)

struct RkDdrcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[DDRC_SIZE / 4];
};

static uint64_t rk_ddrc_read(void *opaque, hwaddr off, unsigned size)
{
    RkDdrcState *s = opaque;

    if (off == R_STAT) {
        return s->regs[R_PWRCTL / 4] & PWRCTL_SELFREF_SW ? STAT_SELFREF_SW
                                                          : STAT_NORMAL;
    }
    return s->regs[off / 4];
}

static void rk_ddrc_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    RkDdrcState *s = opaque;

    s->regs[off / 4] = val;
}

static const MemoryRegionOps rk_ddrc_ops = {
    .read = rk_ddrc_read,
    .write = rk_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_ddrc_reset(DeviceState *dev)
{
    RkDdrcState *s = RK_DDRC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void rk_ddrc_init(Object *obj)
{
    RkDdrcState *s = RK_DDRC(obj);

    memory_region_init_io(&s->iomem, obj, &rk_ddrc_ops, s, TYPE_RK_DDRC,
                          DDRC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription rk_ddrc_vmstate = {
    .name = TYPE_RK_DDRC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkDdrcState, DDRC_SIZE / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void rk_ddrc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_ddrc_reset);
    dc->vmsd = &rk_ddrc_vmstate;
}

static const TypeInfo rk_ddrc_info = {
    .name = TYPE_RK_DDRC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkDdrcState),
    .instance_init = rk_ddrc_init,
    .class_init = rk_ddrc_class_init,
};

static void rk_ddrc_register_types(void)
{
    type_register_static(&rk_ddrc_info);
}

type_init(rk_ddrc_register_types)
