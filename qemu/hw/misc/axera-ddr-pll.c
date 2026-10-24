/*
 * Axera's DDR system DPLL (dpll_ctrl_glb @ 0x00265000): the PLL that
 * clocks the DDR controller/PHY, enabled by BL1 before DDR training
 * can even start -- a real, separate PLL instance from the CPU/VPLL0
 * one hw/misc/axera-pllc.c already models (different base address,
 * different register layout: this one's ready status lives at two
 * separate offsets, not one).
 *
 * Grounded directly in the real vendor BL1 source
 * (maix_ax620e_sdk/boot/bl1/driver/ddr/ddr/dpll_ctrl_glb.h): BL1
 * writes DPLL_CFG1 (+0x10), then sets bit 0 of DPLL_ON_CFG (+0x44) to
 * enable the PLL, then spins reading DPLL_STS (+0x0) waiting for bit 0
 * (locked), then spins again reading PLL_RDY_STS (+0x40) waiting for
 * bit 0 -- two separate poll loops, confirmed from a real deployed
 * AX620E camera's own SPI NOR dump by gdb-tracing a silent flash-file=
 * boot straight into the first of those loops (tbz w0, #0 on a readl()
 * of this base address, offset 0). A plain storage regbank can never
 * satisfy either poll -- nothing ever sets bit 0 -- so BL1 spins here
 * forever with no UART output (this happens before console init) and
 * no -d unimp trace (it's reading storage that exists, just never
 * changes, the same shape as axera-pllc.c's own problem). This model
 * sets both ready bits the moment the enable write lands.
 *
 * Everything else in this window (DPLL_CFG0/CFG1, PLL_RE_OPEN, and any
 * offset this file doesn't know about) is plain storage, same as
 * hisi-regbank -- this device exists only to add the one real side
 * effect a passive regbank can't express.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_DDR_PLL "axera-ddr-pll"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraDdrPllState, AXERA_DDR_PLL)

#define DDR_PLL_SIZE            0x1000
#define DPLL_STS_OFF            0x00
#define DPLL_ON_CFG_OFF         0x44
#define PLL_RDY_STS_OFF         0x40
#define PLL_RDY_BIT             BIT(0)

struct AxeraDdrPllState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint8_t *mem;
};

static uint64_t axera_ddr_pll_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraDdrPllState *s = opaque;
    uint64_t val = 0;

    memcpy(&val, s->mem + offset, size);
    return val;
}

static void axera_ddr_pll_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    AxeraDdrPllState *s = opaque;

    memcpy(s->mem + offset, &value, size);

    if (offset == DPLL_ON_CFG_OFF && (value & BIT(0))) {
        uint32_t *sts = (uint32_t *)(s->mem + DPLL_STS_OFF);
        uint32_t *rdy = (uint32_t *)(s->mem + PLL_RDY_STS_OFF);

        *sts |= PLL_RDY_BIT;
        *rdy |= PLL_RDY_BIT;
    }
}

static const MemoryRegionOps axera_ddr_pll_ops = {
    .read = axera_ddr_pll_read,
    .write = axera_ddr_pll_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_ddr_pll_realize(DeviceState *dev, Error **errp)
{
    AxeraDdrPllState *s = AXERA_DDR_PLL(dev);

    s->mem = g_malloc0(DDR_PLL_SIZE);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_ddr_pll_ops, s,
                          TYPE_AXERA_DDR_PLL, DDR_PLL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void axera_ddr_pll_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_ddr_pll_realize;
}

static const TypeInfo axera_ddr_pll_info = {
    .name = TYPE_AXERA_DDR_PLL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraDdrPllState),
    .class_init = axera_ddr_pll_class_init,
};

static void axera_ddr_pll_register_types(void)
{
    type_register_static(&axera_ddr_pll_info);
}

type_init(axera_ddr_pll_register_types)
