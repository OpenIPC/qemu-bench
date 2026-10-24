/*
 * Axera AX620E/AX630C DDR PHY register block (D_DDRPHY_* in the real
 * vendor BL1 source, boot/bl1/driver/ddr/ddr/ddrphy_reg_addr_def.h):
 * channel A at 0x110000, channel B at the same layout + DDR_PHY_OFFSET
 * (0x10000) = 0x120000. One instance of this device covers one
 * channel; axera.c maps two.
 *
 * Confirmed by tracing the real BL1 injected via flash-file=: it spun
 * forever (12M+ accesses) inside ddrphy_zqc_seq() (boot/bl1/driver/
 * ddr/ddr_init.c), a bounded successive-approximation search against
 * GEN_CFG1's "ZQ calibration compare result" bits (17 for PMOS, 1 for
 * NMOS) -- NOT an unbounded hardware poll. Each step writes a new
 * candidate code into GEN_CFG1 (bits 20:25 for PMOS, 4:9 for NMOS) and
 * reads the compare bit back to decide which way to adjust next; a
 * plain unimplemented-device stub never reflects the write, so the
 * read-back code is always 0 and the search can never converge.
 *
 * This models the real register shape faithfully -- a comparator that
 * flips once the candidate code crosses a threshold, same as real
 * ZQ-cal silicon compares a candidate driver-impedance code against a
 * reference resistor -- just with an arbitrary fixed threshold instead
 * of a real resistor's value, since QEMU's RAM has no real electrical
 * characteristics to calibrate against (same trade-off already made
 * for axera-pllc.c/axera-ddr-pll.c's ready bits). Verified against the
 * real decrement/increment loop shapes in ddrphy_zqc_seq(): with
 * PMOS_THRESHOLD == NMOS_THRESHOLD, both the down-from-0xf and
 * up-from-0x0 passes converge to code==threshold (+-1), well inside
 * the real code's own delta<=3 sanity check -- not the explicit
 * while(1) hardware-failure trap it takes if the two passes disagree.
 *
 * Every other offset (GEN_CFG0, the TMG/CFG timing registers, DS0/DS1
 * lane-scan registers, etc.) is a plain RAM-backed readback, the same
 * read-modify-write shape as hisi-regbank -- BL1 reads back plenty of
 * these elsewhere in ddr_init.c/ddr_scan.c and a stub that discards
 * writes breaks all of them the same way it broke GEN_CFG1.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_DDR_PHY "axera-ddr-phy"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraDdrPhyState, AXERA_DDR_PHY)

#define AXERA_DDR_PHY_SIZE      0x4000
#define GEN_CFG1_OFFSET         0x4

#define PMOS_COMPARE_BIT        17
#define PMOS_CODE_LSB           20
#define NMOS_COMPARE_BIT        1
#define NMOS_CODE_LSB           4
#define CODE_WIDTH_MASK         0x3f

/*
 * Picked within the real code's own working range (it bounds-checks
 * failure at 0x0/0xf on either side), away from both edges so neither
 * SAR pass bottoms/tops out before the comparator flips.
 */
#define PMOS_THRESHOLD          8
#define NMOS_THRESHOLD          8

struct AxeraDdrPhyState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[AXERA_DDR_PHY_SIZE / 4];
};

static uint64_t axera_ddr_phy_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    AxeraDdrPhyState *s = opaque;
    uint32_t val = s->mem[offset / 4];
    uint32_t pmos_code, nmos_code;

    if (offset == GEN_CFG1_OFFSET) {
        pmos_code = (val >> PMOS_CODE_LSB) & CODE_WIDTH_MASK;
        nmos_code = (val >> NMOS_CODE_LSB) & CODE_WIDTH_MASK;

        val &= ~(1U << PMOS_COMPARE_BIT);
        if (pmos_code > PMOS_THRESHOLD) {
            val |= 1U << PMOS_COMPARE_BIT;
        }

        val &= ~(1U << NMOS_COMPARE_BIT);
        if (nmos_code <= NMOS_THRESHOLD) {
            val |= 1U << NMOS_COMPARE_BIT;
        }
    }

    return val;
}

static void axera_ddr_phy_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    AxeraDdrPhyState *s = opaque;

    s->mem[offset / 4] = value;
}

static const MemoryRegionOps axera_ddr_phy_ops = {
    .read = axera_ddr_phy_read,
    .write = axera_ddr_phy_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_ddr_phy_realize(DeviceState *dev, Error **errp)
{
    AxeraDdrPhyState *s = AXERA_DDR_PHY(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_ddr_phy_ops, s,
                          TYPE_AXERA_DDR_PHY, AXERA_DDR_PHY_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void axera_ddr_phy_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_ddr_phy_realize;
}

static const TypeInfo axera_ddr_phy_info = {
    .name = TYPE_AXERA_DDR_PHY,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraDdrPhyState),
    .class_init = axera_ddr_phy_class_init,
};

static void axera_ddr_phy_register_types(void)
{
    type_register_static(&axera_ddr_phy_info);
}

type_init(axera_ddr_phy_register_types)
