/*
 * Ambarella S3L RCT (reset and clock tree) at 0xEC170000.
 *
 * Software reads the rates back out of the PLL control words, so those
 * reset to what a real S3L's boot loader leaves.  The figures are the ones
 * /proc/ambarella/clock shows on the IP4M-1026E: core 720 MHz (gclk_core,
 * gclk_ahb and gclk_apb are /2, /4 and /8 of it, so the timers run at
 * 90 MHz), Cortex 816 MHz, gclk_ddr 840 MHz, IDSP 348 MHz.  A PLL word is
 * intp [30:24], sout [19:16] and sdiv [15:12]: ref * (intp + 1) *
 * (sdiv + 1) / (sout + 1), from a 24 MHz reference.
 *
 * The rest is storage.  Writing SOFT_OR_DLL_RESET's bit 0 back to 1 after 0
 * is how the kernel restarts.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "system/runstate.h"

#define TYPE_AMB_RCT "amb-rct"
OBJECT_DECLARE_SIMPLE_TYPE(AmbRctState, AMB_RCT)

#define AMB_RCT_SIZE    0x1000
#define R_PLL_CORE      0x000
#define R_PLL_LOCK      0x02c
#define R_CG_UART       0x038
#define R_SOFT_RESET    0x068
#define R_PLL_DDR       0x0dc
#define R_PLL_IDSP      0x0e4
#define R_PLL_CORTEX    0x264

#define PLL(intp, sout) (((uint32_t)(intp) - 1) << 24 | ((sout) - 1) << 16)

struct AmbRctState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[AMB_RCT_SIZE / 4];
};

static uint64_t amb_rct_read(void *opaque, hwaddr off, unsigned size)
{
    AmbRctState *s = opaque;

    if (off == R_PLL_LOCK) {
        return 0xffffffff;
    }
    return s->regs[off / 4];
}

static void amb_rct_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    AmbRctState *s = opaque;

    if (off == R_SOFT_RESET && !(s->regs[off / 4] & 1) && (val & 1)) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps amb_rct_ops = {
    .read = amb_rct_read,
    .write = amb_rct_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void amb_rct_reset(DeviceState *dev)
{
    AmbRctState *s = AMB_RCT(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_PLL_CORE / 4] = PLL(30, 1);       /* 720 MHz */
    s->regs[R_PLL_CORTEX / 4] = PLL(34, 1);     /* 816 MHz */
    s->regs[R_PLL_DDR / 4] = PLL(70, 1);        /* gclk_ddr is half: 840 MHz */
    s->regs[R_PLL_IDSP / 4] = PLL(29, 2);       /* 348 MHz */
    s->regs[R_CG_UART / 4] = 1;                 /* the 24 MHz reference */
    s->regs[R_SOFT_RESET / 4] = 3;
}

static void amb_rct_init(Object *obj)
{
    AmbRctState *s = AMB_RCT(obj);

    memory_region_init_io(&s->iomem, obj, &amb_rct_ops, s, TYPE_AMB_RCT,
                          AMB_RCT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription amb_rct_vmstate = {
    .name = TYPE_AMB_RCT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AmbRctState, AMB_RCT_SIZE / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void amb_rct_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, amb_rct_reset);
    dc->vmsd = &amb_rct_vmstate;
}

static const TypeInfo amb_rct_info = {
    .name = TYPE_AMB_RCT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AmbRctState),
    .instance_init = amb_rct_init,
    .class_init = amb_rct_class_init,
};

static void amb_rct_register(void)
{
    type_register_static(&amb_rct_info);
}

type_init(amb_rct_register)
