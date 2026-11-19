/*
 * Fullhan FH8626V100 MIPI CSI-2 receiver: a DesignWare CSI-2 host at
 * 0xF1100000 and Fullhan's wrapper at 0xF1000000, as OpenIPC's 4.19
 * drivers/media/platform/fh8626/fh8626-csi.c drives them.
 *
 *   host    0x04 N_LANES   0x08 CSI2_RESETN   0x40 PHY_SHUTDOWNZ
 *           0x44 DPHY_RSTZ 0x50 / 0x54 PHY_TEST_CTRL0 / 1
 *           0xE4-0x134 the error masks
 *   wrapper 0x00 CTRL (1:0 mode, 2 raw type)  0x04 INT_EN
 *
 * Storage, as far as the driver can tell: it only writes, but for the
 * wrapper's read-modify-write.  What matters is the GPIO "up": the host,
 * its PHY and the D-PHY all out of reset, which the ISP model takes as
 * the receiver passing the sensor's frames on.  The packet and error
 * status, the PHY test interface's read-back and the lane timing are
 * not modelled: the driver reads none of them.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_FH_CSI "fh-csi"
OBJECT_DECLARE_SIMPLE_TYPE(FhCsiState, FH_CSI)

#define FH_CSI_HOST_SIZE    0x1000
#define FH_CSI_WRAP_SIZE    0x100

#define CSI_RESETN          0x08
#define CSI_PHY_SHUTDOWNZ   0x40
#define CSI_DPHY_RSTZ       0x44

struct FhCsiState {
    SysBusDevice parent_obj;

    MemoryRegion host_mr;
    MemoryRegion wrap_mr;
    qemu_irq up;

    uint32_t host[FH_CSI_HOST_SIZE / 4];
    uint32_t wrap[FH_CSI_WRAP_SIZE / 4];
};

static void fh_csi_update(FhCsiState *s)
{
    qemu_set_irq(s->up, (s->host[CSI_RESETN / 4] & 1) &&
                        (s->host[CSI_PHY_SHUTDOWNZ / 4] & 1) &&
                        (s->host[CSI_DPHY_RSTZ / 4] & 1));
}

static uint64_t fh_csi_host_read(void *opaque, hwaddr off, unsigned size)
{
    FhCsiState *s = opaque;

    return s->host[off / 4];
}

static void fh_csi_host_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    FhCsiState *s = opaque;

    s->host[off / 4] = val;
    fh_csi_update(s);
}

static uint64_t fh_csi_wrap_read(void *opaque, hwaddr off, unsigned size)
{
    FhCsiState *s = opaque;

    return s->wrap[off / 4];
}

static void fh_csi_wrap_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    FhCsiState *s = opaque;

    s->wrap[off / 4] = val;
}

static const MemoryRegionOps fh_csi_host_ops = {
    .read = fh_csi_host_read,
    .write = fh_csi_host_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps fh_csi_wrap_ops = {
    .read = fh_csi_wrap_read,
    .write = fh_csi_wrap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_csi_reset(DeviceState *dev)
{
    FhCsiState *s = FH_CSI(dev);

    memset(s->host, 0, sizeof(s->host));
    memset(s->wrap, 0, sizeof(s->wrap));
    fh_csi_update(s);
}

static void fh_csi_init(Object *obj)
{
    FhCsiState *s = FH_CSI(obj);

    memory_region_init_io(&s->host_mr, obj, &fh_csi_host_ops, s,
                          "fh-csi.host", FH_CSI_HOST_SIZE);
    memory_region_init_io(&s->wrap_mr, obj, &fh_csi_wrap_ops, s,
                          "fh-csi.wrap", FH_CSI_WRAP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->host_mr);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->wrap_mr);
    qdev_init_gpio_out_named(DEVICE(obj), &s->up, "up", 1);
}

static int fh_csi_post_load(void *opaque, int version_id)
{
    fh_csi_update(opaque);
    return 0;
}

static const VMStateDescription vmstate_fh_csi = {
    .name = TYPE_FH_CSI,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = fh_csi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(host, FhCsiState, FH_CSI_HOST_SIZE / 4),
        VMSTATE_UINT32_ARRAY(wrap, FhCsiState, FH_CSI_WRAP_SIZE / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void fh_csi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, fh_csi_reset);
    dc->vmsd = &vmstate_fh_csi;
}

static const TypeInfo fh_csi_info = {
    .name          = TYPE_FH_CSI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FhCsiState),
    .instance_init = fh_csi_init,
    .class_init    = fh_csi_class_init,
};

static void fh_csi_register_types(void)
{
    type_register_static(&fh_csi_info);
}

type_init(fh_csi_register_types)
