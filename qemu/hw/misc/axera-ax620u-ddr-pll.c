/*
 * Axera AX620U's DDR PLL/clock-enable block (ddr_pll @ 0x00210000): a
 * real, separate instance from both hw/misc/axera-pllc.c's CPU/VPLL0
 * handshake and hw/misc/axera-ddr-pll.c's own DPLL (different SoC
 * family, different base address, different register layout again).
 *
 * Grounded in a live gdb trace of this real dump's own BL1-equivalent
 * (no source available, no symbols): its ddr-init routine reads
 * ENABLE_OFF (+0x64) once per retry, extracts a mode bit (bit 1) and a
 * 5-bit selector (bits 6:2), and only when mode==1 does it write a
 * frequency value (read from a small fixed config table) to +0x0 and
 * then strobe ENABLE_OFF's bit 0 (set, then immediately clear --
 * same "disable/reconfigure/enable" shape already modelled for
 * axera-pllc.c and axera-ddr-pll.c). It then reads STATUS_OFF (+0x38)
 * and checks bit 13 ("ready"); if clear, it decrements a 16M retry
 * counter and loops back to re-read ENABLE_OFF, bailing out (and
 * printing "ddr init timeout!", confirmed from the real dump's own
 * string table) once either the counter hits 0 or axera-bl1-timer.c's
 * now-real tick counter shows more than one second elapsed.
 *
 * Against a plain storage regbank, ENABLE_OFF's mode bit reads back 0
 * (its zero-initialized default) on every single retry -- this real
 * dump's own mode/selector strapping apparently already reads as
 * "nothing to (re)configure" -- so the strobe-and-set-ready dance this
 * device still models for completeness is never actually exercised;
 * only STATUS_OFF's ready bit being set from power-on (not merely on a
 * strobe that never arrives) gets real BL1 code past this poll at all.
 * Confirmed empirically: the exact same real dump that printed
 * "ddr init timeout!" (and then an unconditional panic loop) every
 * single run against a passive regbank here prints "ddr init
 * success!" and keeps making forward progress once STATUS_OFF's ready
 * bit is live from reset.
 *
 * Everything else in this window is plain storage, same as
 * hisi-regbank -- this device exists only to add the one real side
 * effect a passive regbank can't express.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/misc/axera-ax620u-ddr-pll.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraAx620uDdrPllState, AXERA_AX620U_DDR_PLL)

#define DDR_PLL_SIZE            0x10000
#define STATUS_OFF              0x38
#define ENABLE_OFF              0x64
#define READY_BIT               BIT(13)
#define ENABLE_BIT              BIT(0)

struct AxeraAx620uDdrPllState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint8_t *mem;
};

static uint64_t axera_ax620u_ddr_pll_read(void *opaque, hwaddr offset,
                                          unsigned size)
{
    AxeraAx620uDdrPllState *s = opaque;
    uint64_t val = 0;

    memcpy(&val, s->mem + offset, size);
    return val;
}

static void axera_ax620u_ddr_pll_write(void *opaque, hwaddr offset,
                                       uint64_t value, unsigned size)
{
    AxeraAx620uDdrPllState *s = opaque;

    memcpy(s->mem + offset, &value, size);

    if (offset == ENABLE_OFF && (value & ENABLE_BIT)) {
        uint32_t *sts = (uint32_t *)(s->mem + STATUS_OFF);

        *sts |= READY_BIT;
    }
}

static const MemoryRegionOps axera_ax620u_ddr_pll_ops = {
    .read = axera_ax620u_ddr_pll_read,
    .write = axera_ax620u_ddr_pll_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_ax620u_ddr_pll_realize(DeviceState *dev, Error **errp)
{
    AxeraAx620uDdrPllState *s = AXERA_AX620U_DDR_PLL(dev);
    uint32_t *sts;

    s->mem = g_malloc0(DDR_PLL_SIZE);
    sts = (uint32_t *)(s->mem + STATUS_OFF);
    *sts |= READY_BIT;

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_ax620u_ddr_pll_ops, s,
                          TYPE_AXERA_AX620U_DDR_PLL, DDR_PLL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void axera_ax620u_ddr_pll_class_init(ObjectClass *klass,
                                            const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_ax620u_ddr_pll_realize;
}

static const TypeInfo axera_ax620u_ddr_pll_info = {
    .name = TYPE_AXERA_AX620U_DDR_PLL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraAx620uDdrPllState),
    .class_init = axera_ax620u_ddr_pll_class_init,
};

static void axera_ax620u_ddr_pll_register_types(void)
{
    type_register_static(&axera_ax620u_ddr_pll_info);
}

type_init(axera_ax620u_ddr_pll_register_types)
