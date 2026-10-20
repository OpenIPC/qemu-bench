/*
 * Novatek NA51089 (NT9856x) TOP block: pinmux, pad, chip version.
 *
 * Storage for the pinmux and GPIO-function registers the vendor pinctrl
 * driver (drivers/pinctrl/novatek/na51089) and the loader program, plus
 * the read-only version word:
 *
 *   0xF0 TOP_VERSION_REG  bits [31:16] CHIP_ID, 0x7021 on NA51089
 *                         (plat-na51089/top.h, CHIP_NA51089); the low
 *                         half is the die revision and reads 0.
 *
 * nvt_get_chip_id() reads it through the "nvt,nvt_top" node's first reg
 * entry (drivers/pinctrl/novatek/na51089/na51089_pinmux_host.c), and
 * nvt_serial branches on it; U-Boot's nvt_get_chip_id() (mach-novatek/nvt_na51089_a32/soc.c)
 * reads 0xF00100F0 directly.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_TOP "nvt-top"
OBJECT_DECLARE_SIMPLE_TYPE(NvtTopState, NVT_TOP)

#define NVT_TOP_SIZE        0x2000
#define NVT_TOP_VERSION     0xF0

struct NvtTopState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[NVT_TOP_SIZE / 4];
    uint32_t chip_id;
};

static uint64_t nvt_top_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtTopState *s = opaque;

    if (offset == NVT_TOP_VERSION) {
        return s->chip_id << 16;
    }
    return s->regs[offset / 4];
}

static void nvt_top_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    NvtTopState *s = opaque;

    if (offset == NVT_TOP_VERSION) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-top: write to TOP_VERSION\n");
        return;
    }
    s->regs[offset / 4] = val;
}

static const MemoryRegionOps nvt_top_ops = {
    .read = nvt_top_read,
    .write = nvt_top_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_top_reset(DeviceState *dev)
{
    NvtTopState *s = NVT_TOP(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void nvt_top_init(Object *obj)
{
    NvtTopState *s = NVT_TOP(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_top_ops, s, TYPE_NVT_TOP,
                          NVT_TOP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_nvt_top = {
    .name = TYPE_NVT_TOP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtTopState, NVT_TOP_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_top_properties[] = {
    DEFINE_PROP_UINT32("chip-id", NvtTopState, chip_id, 0x7021),
};

static void nvt_top_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_top_reset);
    device_class_set_props(dc, nvt_top_properties);
    dc->vmsd = &vmstate_nvt_top;
}

static const TypeInfo nvt_top_info = {
    .name = TYPE_NVT_TOP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtTopState),
    .instance_init = nvt_top_init,
    .class_init = nvt_top_class_init,
};

static void nvt_top_register_types(void)
{
    type_register_static(&nvt_top_info);
}

type_init(nvt_top_register_types)
