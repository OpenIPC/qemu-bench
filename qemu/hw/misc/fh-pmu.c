/*
 * Fullhan PMU: the clock, reset, pad and chip-ID block at 0xF0000000.
 *
 * Mostly storage -- PLLs, dividers, gates and the pad mux the vendor code
 * read-modify-writes (mach-fh/<soc>/chip.h REG_PMU_*).  What reads back
 * differently:
 *
 *   0x000 CHIP_ID   platform ID, the first key of mach-fh/fh_chipid.c's
 *   0x004 IP_VER    table; the package ID its second key for the V100
 *                   generations
 *   0x33C CHIP_INFO the package ID from FH885xV200 / FH865x on, over IP_VER
 *   0x334 DDR_SIZE  DDR in Mbit (fh_pmu_get_ddrsize())
 *   0x330 BOOT_MODE the boot strap the ROM latched: 1 SPI NAND, 2 SPI NOR,
 *                   3 / 4 eMMC on SD0 / SD1 (U-Boot's common.h FH_*_FLASH;
 *                   its environment is looked for on that medium)
 *   0x040-0x04C SWRST_*  write 0 bits to reset blocks; the resets finish
 *                   at once, so they read all ones (U-Boot's fh_mci_reset
 *                   polls for that).  Clearing SWRST_MAIN_CTRL bit 31 is
 *                   the system reset both U-Boot and fh-reboot.c use, and
 *                   its bit 18 resets the timers ("tmr-reset", the FH8632
 *                   kernel's way of reloading them).  FH8626V100 has the
 *                   four at 0x04C-0x058 ("swrst-base").
 *   0x22C / 0x230 PTSLO / PTSHI  a 64-bit 1 MHz count of time since power
 *                   on, latched by writing 1 to PTSLO
 *   0x1FC WDT_CTRL  bit 8 pauses the watchdog's counter (GPIO "wdt-pause")
 *
 * The clock tree the kernels compute (mach-fh/clock.c).  On FH8632 and
 * FH8626V100 ("pll-scheme" 2) the timers run from PLL1 = 24 MHz * M / N /
 * 2^OD (M 7:0, N 11:8, OD 17:16), resetting to 600 MHz, / 10 / (CLK_DIV5
 * [23:16] + 1) -- FH8632's tmr0_clk prediv; FH8626V100's kernel has none
 * ("tmr-prediv" 1) and runs its timers at 10 MHz.  On the later dies, from the system PLL's
 * P output, VCO = 24 MHz * M / (N + 1), P = VCO / (P + 1) / DIVVCO (its
 * control register's bits 11:8), is PLL2 (0x64, control 0x68) on
 * FH885xV200 / FH865x and PLL0 (0x10, control 0x18) on the V100 dies
 * ("pll-scheme" 1).  It resets to 1200 MHz, which CLK_DIV5[27:24] + 1 = 12
 * takes to the 100 MHz the dividers below assume.  The "tmr-clk" output is
 * that / (CLK_DIV5[23:16] + 1): 1 MHz from the reset value U-Boot also
 * writes, and whatever a kernel programs -- the V100 and FH865x kernels set
 * 50 MHz through it.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/clock.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/runstate.h"

#define TYPE_FH_PMU "fh-pmu"
OBJECT_DECLARE_SIMPLE_TYPE(FhPmuState, FH_PMU)

#define FH_PMU_SIZE         0x4000

#define PMU_CHIP_ID         0x000
#define PMU_IP_VER          0x004
#define PMU_PLL0            0x010
#define PMU_PLL1            0x014
#define PMU_PLL0_CTRL       0x018
#define PMU_CLK_DIV5        0x038
#define PMU_SWRST_BASE      0x040
#define PMU_SPC_IO_STATUS   0x054
#define PMU_PLL2            0x064
#define PMU_PLL2_CTRL       0x068
#define PMU_WDT_CTRL        0x1FC
#define PMU_PTSLO           0x22C
#define PMU_PTSHI           0x230
#define PMU_BOOT_MODE       0x330
#define PMU_DDR_SIZE        0x334
#define PMU_CHIP_INFO       0x33C

struct FhPmuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq wdt_pause;
    qemu_irq tmr_reset;
    Clock *tmr_clk;
    uint32_t regs[FH_PMU_SIZE / 4];
    uint64_t pts_latch;

    uint32_t chip_id;
    uint32_t ip_ver;
    uint32_t chip_info;
    uint32_t ddr_mbit;
    uint32_t boot_mode;
    uint32_t pll_scheme;
    uint32_t tmr_prediv;
    uint32_t swrst_base;
    uint32_t spc_io_status;
};

static hwaddr fh_pmu_sys_pll(FhPmuState *s)
{
    switch (s->pll_scheme) {
    case 1:  return PMU_PLL0;
    case 2:  return PMU_PLL1;
    default: return PMU_PLL2;
    }
}

static hwaddr fh_pmu_sys_pll_ctrl(FhPmuState *s)
{
    return s->pll_scheme == 1 ? PMU_PLL0_CTRL : PMU_PLL2_CTRL;
}

/* mach-fh/clock.c fh_clk_get_pll_p_rate() */
static uint64_t fh_pmu_pll_p(FhPmuState *s)
{
    uint32_t reg = s->regs[fh_pmu_sys_pll(s) >> 2];
    uint32_t m = reg & 0x7F, n = (reg >> 8) & 0x1F, p = (reg >> 16) & 0x3F;
    uint32_t sel = (s->regs[fh_pmu_sys_pll_ctrl(s) >> 2] >> 8) & 0xF;
    uint32_t divvco;

    if (m < 4) {
        m += 128;
    }
    if (m == 0xB) {
        m = 0xA;
    }
    switch (sel) {
    case 0x8: divvco = 2; break;
    case 0xC: divvco = 4; break;
    case 0xD: divvco = 8; break;
    case 0xE: divvco = 16; break;
    case 0xF: divvco = 32; break;
    default:  divvco = 1; break;
    }
    return 24000000ULL * m / (n + 1) / (p + 1) / divvco;
}

/* mach-fh/clock.c fh_clk_get_pll_rate(), the FH8632 generation's */
static uint64_t fh_pmu_pll1_v1(FhPmuState *s)
{
    uint32_t reg = s->regs[PMU_PLL1 >> 2];
    uint32_t m = reg & 0xFF, n = (reg >> 8) & 0xF, od = (reg >> 16) & 3;

    return n ? 24000000ULL * m / n >> od : 0;
}

static void fh_pmu_update_clocks(FhPmuState *s)
{
    uint32_t div5 = s->regs[PMU_CLK_DIV5 >> 2];
    uint64_t src;

    if (s->pll_scheme == 2) {
        src = fh_pmu_pll1_v1(s) / (s->tmr_prediv ? s->tmr_prediv : 1);
    } else {
        src = fh_pmu_pll_p(s) / (((div5 >> 24) & 0xF) + 1);
    }
    clock_update_hz(s->tmr_clk, src / (((div5 >> 16) & 0xFF) + 1));
}

static bool fh_pmu_is_swrst(FhPmuState *s, hwaddr off)
{
    return off >= s->swrst_base && off <= s->swrst_base + 0xC;
}

static uint64_t fh_pmu_pts(void)
{
    return qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
}

static uint64_t fh_pmu_read(void *opaque, hwaddr off, unsigned size)
{
    FhPmuState *s = opaque;

    if (fh_pmu_is_swrst(s, off)) {
        return 0xFFFFFFFF;
    }
    switch (off) {
    case PMU_PTSLO:
        return (uint32_t)s->pts_latch;
    case PMU_PTSHI:
        return s->pts_latch >> 32;
    }
    return s->regs[off >> 2];
}

static void fh_pmu_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhPmuState *s = opaque;

    switch (off) {
    case PMU_CHIP_ID:
    case PMU_IP_VER:
    case PMU_BOOT_MODE:
    case PMU_DDR_SIZE:
    case PMU_CHIP_INFO:
        return;
    }
    if (fh_pmu_is_swrst(s, off)) {
        if (off == s->swrst_base) {
            if (!(val & (1u << 31))) {
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            }
            if (!(val & (1u << 18))) {
                qemu_irq_pulse(s->tmr_reset);
            }
        }
        return;
    }
    switch (off) {
    case PMU_PTSLO:
        if (val & 1) {
            s->pts_latch = fh_pmu_pts();
        }
        return;
    }
    s->regs[off >> 2] = val;
    if (off == PMU_CLK_DIV5 || off == fh_pmu_sys_pll(s) ||
        off == fh_pmu_sys_pll_ctrl(s)) {
        fh_pmu_update_clocks(s);
    }
    if (off == PMU_WDT_CTRL) {
        qemu_set_irq(s->wdt_pause, (val >> 8) & 1);
    }
}

static const MemoryRegionOps fh_pmu_ops = {
    .read = fh_pmu_read,
    .write = fh_pmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_pmu_reset(DeviceState *dev)
{
    FhPmuState *s = FH_PMU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[PMU_CHIP_ID >> 2] = s->chip_id;
    s->regs[PMU_IP_VER >> 2] = s->ip_ver;
    s->regs[PMU_CHIP_INFO >> 2] = s->chip_info;
    s->regs[PMU_DDR_SIZE >> 2] = s->ddr_mbit;
    s->regs[PMU_BOOT_MODE >> 2] = s->boot_mode;
    s->regs[PMU_PLL1 >> 2] = 0x0200003A;
    if (!fh_pmu_is_swrst(s, PMU_SPC_IO_STATUS)) {
        s->regs[PMU_SPC_IO_STATUS >> 2] = s->spc_io_status;
    }
    if (s->pll_scheme == 2) {
        /* M, N, OD: PLL0 480 MHz as the DDR script leaves it, PLL1 600 */
        s->regs[PMU_PLL0 >> 2] = 0x00010128;
        s->regs[PMU_PLL1 >> 2] = 0x00000232;
        s->regs[PMU_PLL0_CTRL >> 2] = 0x00010232;   /* PLL2 here, 300 */
    } else if (s->pll_scheme == 1) {
        s->regs[PMU_PLL0 >> 2] = 0x01000032;        /* P 1200, R 600 */
    } else {
        /* The DDR PLL the 2BL script leaves, then the system PLL. */
        s->regs[PMU_PLL0 >> 2] = 0x01010058;
        s->regs[PMU_PLL0_CTRL >> 2] = 0x88198819;
        s->regs[PMU_PLL2 >> 2] = 0x01000032;        /* P 1200, R 600 */
    }
    /* sys / 12 = 100 MHz; timers 100 / 100, the watchdog 100 / 200 */
    s->regs[PMU_CLK_DIV5 >> 2] = 0x0B63C700;
    if (s->pll_scheme == 2) {
        s->regs[PMU_CLK_DIV5 >> 2] = 0x003BC700;    /* 600 / 10 / 60 */
    }
    s->pts_latch = 0;
    qemu_set_irq(s->wdt_pause, 0);
    fh_pmu_update_clocks(s);
}

static void fh_pmu_init(Object *obj)
{
    FhPmuState *s = FH_PMU(obj);

    memory_region_init_io(&s->iomem, obj, &fh_pmu_ops, s, TYPE_FH_PMU,
                          FH_PMU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->wdt_pause, "wdt-pause", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->tmr_reset, "tmr-reset", 1);
    s->tmr_clk = qdev_init_clock_out(DEVICE(obj), "tmr-clk");
}

static const Property fh_pmu_props[] = {
    DEFINE_PROP_UINT32("chip-id", FhPmuState, chip_id, 0),
    DEFINE_PROP_UINT32("ip-ver", FhPmuState, ip_ver, 0),
    DEFINE_PROP_UINT32("chip-info", FhPmuState, chip_info, 0),
    DEFINE_PROP_UINT32("ddr-mbit", FhPmuState, ddr_mbit, 0),
    DEFINE_PROP_UINT32("boot-mode", FhPmuState, boot_mode, 2),
    DEFINE_PROP_UINT32("pll-scheme", FhPmuState, pll_scheme, 0),
    DEFINE_PROP_UINT32("tmr-prediv", FhPmuState, tmr_prediv, 10),
    DEFINE_PROP_UINT32("swrst-base", FhPmuState, swrst_base, PMU_SWRST_BASE),
    /* FH8632: its variant in bits 5:0 (fh_chipid.c) */
    DEFINE_PROP_UINT32("spc-io-status", FhPmuState, spc_io_status, 0),
};

static const VMStateDescription vmstate_fh_pmu = {
    .name = TYPE_FH_PMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, FhPmuState, FH_PMU_SIZE / 4),
        VMSTATE_UINT64(pts_latch, FhPmuState),
        VMSTATE_CLOCK(tmr_clk, FhPmuState),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_pmu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, fh_pmu_reset);
    device_class_set_props(dc, fh_pmu_props);
    dc->vmsd = &vmstate_fh_pmu;
}

static const TypeInfo fh_pmu_info = {
    .name           = TYPE_FH_PMU,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhPmuState),
    .instance_init  = fh_pmu_init,
    .class_init     = fh_pmu_class_init,
};

static void fh_pmu_register_types(void)
{
    type_register_static(&fh_pmu_info);
}

type_init(fh_pmu_register_types)
