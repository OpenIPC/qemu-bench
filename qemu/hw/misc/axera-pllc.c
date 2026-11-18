/*
 * Axera AX620E/AX630C PLLC (clk_pllc @ 0x02210000): CPU PLL and VPLL0
 * enable/ready handshake.
 *
 * Grounded directly in the real vendor BL1 source
 * (maix_ax620e_sdk/boot/bl1/board/board.c's cpupll_config()/
 * vpll0_config(), addresses from boot/bl1/common/include/chip_reg.h):
 * BL1 writes bit 0 of *_ON_CFG_SET_ADDR to enable a PLL, then spins
 * reading GRP_PLL_RDY_STS_ADDR waiting for that PLL's ready bit (CPU
 * PLL: bit 3; VPLL0: bit 1) to come up. A plain storage regbank can
 * never satisfy that -- nothing ever sets the ready bit -- so BL1
 * spins here forever with no UART output and no -d unimp trace (it's
 * reading storage that exists, just never changes). This model sets
 * the matching ready bit the moment the enable write lands, which is
 * both the simplest correct behaviour for an emulator (there is no
 * real PLL lock delay worth modelling) and exactly what let BL1 get
 * past this point.
 *
 * Everything else in this 64 KiB window (CFG0/CFG1 fractional/integer
 * multiplier registers, and any offset this file doesn't know about)
 * is plain storage, same as hisi-regbank -- this device exists only to
 * add the one real side effect regbank can't express.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_PLLC "axera-pllc"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraPllcState, AXERA_PLLC)

#define PLLC_SIZE               0x10000
#define GRP_PLL_RDY_STS_OFF     0x1c0
#define CPUPLL_ON_CFG_SET_OFF   0x1ec
#define CPUPLL_ON_CFG_CLR_OFF   0x1f0
#define CPUPLL_RDY_BIT          BIT(3)
#define VPLL0_ON_CFG_SET_OFF    0x1d4
#define VPLL0_ON_CFG_CLR_OFF    0x1d8
#define VPLL0_RDY_BIT           BIT(1)

struct AxeraPllcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint8_t *mem;
};

static uint64_t axera_pllc_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraPllcState *s = opaque;
    uint64_t val = 0;

    memcpy(&val, s->mem + offset, size);
    return val;
}

static void axera_pllc_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AxeraPllcState *s = opaque;
    uint32_t *ready = (uint32_t *)(s->mem + GRP_PLL_RDY_STS_OFF);

    memcpy(s->mem + offset, &value, size);

    switch (offset) {
    case CPUPLL_ON_CFG_SET_OFF:
        if (value & BIT(0)) {
            *ready |= CPUPLL_RDY_BIT;
        }
        break;
    case CPUPLL_ON_CFG_CLR_OFF:
        if (value & BIT(0)) {
            *ready &= ~CPUPLL_RDY_BIT;
        }
        break;
    case VPLL0_ON_CFG_SET_OFF:
        if (value & BIT(0)) {
            *ready |= VPLL0_RDY_BIT;
        }
        break;
    case VPLL0_ON_CFG_CLR_OFF:
        if (value & BIT(0)) {
            *ready &= ~VPLL0_RDY_BIT;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps axera_pllc_ops = {
    .read = axera_pllc_read,
    .write = axera_pllc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_pllc_realize(DeviceState *dev, Error **errp)
{
    AxeraPllcState *s = AXERA_PLLC(dev);

    s->mem = g_malloc0(PLLC_SIZE);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_pllc_ops, s,
                          TYPE_AXERA_PLLC, PLLC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void axera_pllc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_pllc_realize;
}

static const TypeInfo axera_pllc_info = {
    .name = TYPE_AXERA_PLLC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraPllcState),
    .class_init = axera_pllc_class_init,
};

static void axera_pllc_register_types(void)
{
    type_register_static(&axera_pllc_info);
}

type_init(axera_pllc_register_types)
