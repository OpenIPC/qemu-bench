/*
 * Anyka AK39xx DDR2 controller.
 *
 * Programmed by the boot header's register script (hw/arm/anyka.c) and,
 * on the EV330, again by the first stage in L2 memory.  Its timing and mode
 * registers are kept and mean nothing here; what code waits on is 0x24:
 * the scripts write 1 (EV200 / EV300) or 0x81 (EV330) there to start the
 * DDR initialisation, and the EV330 stage polls bit 1 for "done" and then
 * reads bit 2 as "training failed, retry" (its 0x48000444 loop).  Done at
 * once and never failing, as on a board that boots.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_DDRC "ak-ddrc"
OBJECT_DECLARE_SIMPLE_TYPE(AkDdrcState, AK_DDRC)

#define AK_DDRC_NREGS       (0x100 / 4)
#define R_INIT              (0x24 / 4)
#define INIT_DONE           (1u << 1)
#define INIT_FAILED         (1u << 2)

struct AkDdrcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[AK_DDRC_NREGS];
};

static uint64_t ak_ddrc_read(void *opaque, hwaddr offset, unsigned size)
{
    AkDdrcState *s = opaque;
    uint32_t v;

    if (offset >= sizeof(s->regs)) {
        return 0;
    }
    v = s->regs[offset / 4];
    if (offset / 4 == R_INIT) {
        v = (v | INIT_DONE) & ~INIT_FAILED;
    }
    return v;
}

static void ak_ddrc_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    AkDdrcState *s = opaque;

    if (offset < sizeof(s->regs)) {
        s->regs[offset / 4] = val;
    }
}

static const MemoryRegionOps ak_ddrc_ops = {
    .read = ak_ddrc_read,
    .write = ak_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_ddrc_reset(DeviceState *dev)
{
    AkDdrcState *s = AK_DDRC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void ak_ddrc_init(Object *obj)
{
    AkDdrcState *s = AK_DDRC(obj);

    memory_region_init_io(&s->iomem, obj, &ak_ddrc_ops, s, TYPE_AK_DDRC,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ak_ddrc = {
    .name = TYPE_AK_DDRC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AkDdrcState, AK_DDRC_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_ddrc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ak_ddrc;
    device_class_set_legacy_reset(dc, ak_ddrc_reset);
}

static const TypeInfo ak_ddrc_info = {
    .name          = TYPE_AK_DDRC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkDdrcState),
    .instance_init = ak_ddrc_init,
    .class_init    = ak_ddrc_class_init,
};

static void ak_ddrc_register_types(void)
{
    type_register_static(&ak_ddrc_info);
}

type_init(ak_ddrc_register_types)
