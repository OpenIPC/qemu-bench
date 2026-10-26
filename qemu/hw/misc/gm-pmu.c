/*
 * Grain Media FTPMU010 power management unit.
 *
 * The GM813x system controller at 0x90C00000: chip ID, boot strap,
 * PLLs, clock gating, pin mux and the frequency change sequence
 * (arch/arm/mach-GM/platform-GM8136/pmu.c, the nsboot loader's entry.c
 * and U-Boot board/gm/gm8136/gm8136.c).  Registers the model gives
 * meaning to:
 *
 *   0x00 ID       bits 31:16 0x0813 / 15:8 0x61 (0x6100 is revision B),
 *                 bits 6:2 the package: 0 GM8136, 0x14/0x1C GM8136S,
 *                 others GM8135(S)
 *   0x04 strap    bit 7 boots from SPI NAND (else SPI NOR), bit 23 a
 *                 4-byte-address NOR
 *   0x0C PMODE    bit 2 starts a frequency change (FCS)
 *   0x20 status   bit 17 FCS done, write 1 to clear
 *   0x30 PLL1     bits 10:4 N, 15:11 M (30 MHz * N / M), 3:2 range,
 *                 31:16 the clock selection: bits 19:18 AHB (PLL1 / 3,
 *                 PLL2 / 3, PLL2 / 4), 22 the CPU from PLL2
 *   0x34 PLL2/3   PLL2 as PLL1's low half, PLL3 in bits 26:20 / 31:27
 *
 * nsboot changes the CPU clock by writing PLL1, setting PMODE bit 2 and
 * waiting in WFI for interrupt 8, whose handler clears the status bit;
 * here the change completes at once and raises "fcs-irq".  PCLK -- AHB
 * / 2, the timers' and watchdog's clock -- is the "pclk" output, which
 * follows the PLL registers.  The other registers hold what is written
 * (gating, pin mux, dividers): the drivers only read back their own
 * settings.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/clock.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_PMU "gm-pmu"
OBJECT_DECLARE_SIMPLE_TYPE(GmPmuState, GM_PMU)

#define GM_PMU_SIZE         0x1000
#define GM_PMU_NREGS        (0x100 / 4)
#define GM_OSC_HZ           30000000

#define R_ID                0x00
#define R_STRAP             0x04
#define R_PMODE             0x0C
#define R_STATUS            0x20
#define R_PLL1              0x30
#define R_PLL23             0x34

#define PMODE_FCS           (1u << 2)
#define STATUS_FCS          (1u << 17)

struct GmPmuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq fcs_irq;
    Clock *pclk;
    uint32_t regs[GM_PMU_NREGS];

    uint32_t chip_id;
    uint32_t strap;
    uint32_t pll1;
    uint32_t pll23;
};

static uint64_t gm_pmu_pll_hz(uint32_t v)
{
    uint32_t n = extract32(v, 4, 7);
    uint32_t m = extract32(v, 11, 5);

    return m ? (uint64_t)GM_OSC_HZ * n / m : 0;
}

static void gm_pmu_update_clocks(GmPmuState *s)
{
    uint64_t pll1 = gm_pmu_pll_hz(s->regs[R_PLL1 / 4]);
    uint64_t pll2 = gm_pmu_pll_hz(s->regs[R_PLL23 / 4]);
    uint64_t ahb;

    switch (extract32(s->regs[R_PLL1 / 4], 18, 2)) {
    case 0:
        ahb = pll1 / 3;
        break;
    case 1:
        ahb = pll2 / 3;
        break;
    default:
        ahb = pll2 / 4;
        break;
    }
    clock_update_hz(s->pclk, ahb / 2);
}

static void gm_pmu_update_irq(GmPmuState *s)
{
    qemu_set_irq(s->fcs_irq, !!(s->regs[R_STATUS / 4] & STATUS_FCS));
}

static uint64_t gm_pmu_read(void *opaque, hwaddr off, unsigned size)
{
    GmPmuState *s = opaque;

    if (off >= GM_PMU_NREGS * 4) {
        return 0;
    }
    return extract32(s->regs[off / 4], (off & 3) * 8, size * 8);
}

static void gm_pmu_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmPmuState *s = opaque;

    if (off >= GM_PMU_NREGS * 4) {
        return;
    }
    /* byte and halfword stores merge into the register */
    if (size < 4) {
        unsigned shift = (off & 3) * 8;

        off &= ~3;
        if (off == R_STATUS) {
            val <<= shift;      /* write-1-to-clear: only these bytes */
        } else {
            val = deposit32(s->regs[off / 4], shift, size * 8, val);
        }
    }
    switch (off) {
    case R_ID:
    case R_STRAP:
        return;
    case R_STATUS:
        s->regs[R_STATUS / 4] &= ~val;
        gm_pmu_update_irq(s);
        return;
    case R_PMODE:
        if ((val & PMODE_FCS) && !(s->regs[R_PMODE / 4] & PMODE_FCS)) {
            s->regs[R_STATUS / 4] |= STATUS_FCS;
        }
        s->regs[R_PMODE / 4] = val;
        gm_pmu_update_irq(s);
        return;
    }
    s->regs[off / 4] = val;
    if (off == R_PLL1 || off == R_PLL23) {
        gm_pmu_update_clocks(s);
    }
}

static const MemoryRegionOps gm_pmu_ops = {
    .read = gm_pmu_read,
    .write = gm_pmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void gm_pmu_reset(DeviceState *dev)
{
    GmPmuState *s = GM_PMU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_ID / 4] = s->chip_id;
    s->regs[R_STRAP / 4] = s->strap;
    s->regs[R_PLL1 / 4] = s->pll1;
    s->regs[R_PLL23 / 4] = s->pll23;
    /* UART clock divider: PLL2 / 2 / 12 = 25 MHz, as nsboot leaves it */
    s->regs[0x70 / 4] = 11u << 16;
    /*
     * ADDA308 clock divider, 0x74 bits 29:24: what adda308.ko sets
     * (/(10 + 1)); irda.ko divides by it before the codec is loaded
     */
    s->regs[0x74 / 4] = 0xAu << 24;
    gm_pmu_update_clocks(s);
    gm_pmu_update_irq(s);
}

static void gm_pmu_realize(DeviceState *dev, Error **errp)
{
    GmPmuState *s = GM_PMU(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_pmu_ops, s,
                          TYPE_GM_PMU, GM_PMU_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_out_named(dev, &s->fcs_irq, "fcs-irq", 1);
}

static void gm_pmu_init(Object *obj)
{
    GmPmuState *s = GM_PMU(obj);

    s->pclk = qdev_init_clock_out(DEVICE(obj), "pclk");
}

static int gm_pmu_post_load(void *opaque, int version_id)
{
    gm_pmu_update_clocks(opaque);
    return 0;
}

static const Property gm_pmu_props[] = {
    DEFINE_PROP_UINT32("chip-id", GmPmuState, chip_id, 0x08136100),
    DEFINE_PROP_UINT32("strap", GmPmuState, strap, 0),
    /*
     * PLL1 860 MHz (30 * 86 / 3), AHB = PLL2 / 3: what a Longse camera's
     * nsboot sets (its GM8136 mode 5), so a -kernel boot that skips it
     * sees the same.  The audio codec driver (adda308) panics unless
     * PLL1 is a rate in its MCLK table: 860, 590, 762, 885, 712.5 or
     * 442.5 MHz.
     */
    DEFINE_PROP_UINT32("pll1", GmPmuState, pll1, 0x62161D6C),
    /* PLL2 600 MHz, PLL3 540 MHz */
    DEFINE_PROP_UINT32("pll23", GmPmuState, pll23, 0x092C128C),
};

static const VMStateDescription vmstate_gm_pmu = {
    .name = TYPE_GM_PMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = gm_pmu_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmPmuState, GM_PMU_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_pmu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_pmu_realize;
    device_class_set_legacy_reset(dc, gm_pmu_reset);
    device_class_set_props(dc, gm_pmu_props);
    dc->vmsd = &vmstate_gm_pmu;
}

static const TypeInfo gm_pmu_info = {
    .name           = TYPE_GM_PMU,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmPmuState),
    .instance_init  = gm_pmu_init,
    .class_init     = gm_pmu_class_init,
};

static void gm_pmu_register_types(void)
{
    type_register_static(&gm_pmu_info);
}

type_init(gm_pmu_register_types)
