/*
 * Novatek NA51089 (NT9856x) clock generator (CG).
 *
 * Storage with reset values a booted loader leaves behind, so the clock
 * framework (drivers/clk/novatek/na51089-clk.c, nvt-im-clk-fr.c) computes
 * sane rates.  Offsets are plat-na51089/cg-reg.h:
 *
 *   0x00 PLL_EN          one bit per PLL
 *   0x04 PLL_STAT        lock status: reads back PLL_EN
 *   0x10 SYS_CLK_RATE    [1:0] CPU clock: 0 80 MHz, 1 PLL8, 2 480 MHz
 *   0x70..0x7C CLK_EN    gates; reset 0 would leave every block stopped
 *   0x80..0x88 SYS_RESET 1 = out of reset.  Each bit is also a GPIO line,
 *                        "sys-reset" n = (offset - 0x80) * 8 + bit, high
 *                        while the block is held in reset: the clock
 *                        framework pulses a block's bit when it prepares
 *                        its clock (nvt_composite_gate_clk_prepare,
 *                        DO_RESET), and U-Boot does it by hand -- e.g.
 *                        0x84 bits 2 / 3 / 14 for the SDIO hosts
 *   PLLn base + 0x20/0x24/0x28  ratio bytes 0/1/2; rate = 12 MHz *
 *                        ratio / 131072 (nvt_pll_clk_recalc_rate)
 *
 * The A9 global timer and TWD are clocked by periph_clk, which the kernel
 * derives as cpu_clk / 8 and U-Boot as get_cpu_clk() / 8.  QEMU's
 * a9mpcore ticks both at a fixed 100 MHz, so PLL8 is reset to 800 MHz
 * (ratio 0x855555) rather than the silicon's 960 MHz (0xA00000): with
 * the real rate every delay and jiffy would run 20% slow.
 *
 * "layout" picks where the PLL ratio registers are -- the one thing the
 * NT9852x die moves (plat-na51055/cg-reg.h; the NT9852x kernel's
 * na51055-clk.c picks its table by chip ID):
 *   0  NA51089: PLLn base + 0x20
 *   1  NA51055 (NT98520/525): its own table from 0x1188
 *   2  NA51084 (NT98528/529): the CG528_* table from 0x4400, where the
 *      kernel takes PLL8 as an eighth of the CPU clock, so PLL8 is 100 MHz
 *   3  NA51068 (NT98321/323, plat-na51068/cg-reg.h): a different block
 *      altogether, 128 KiB.  The PLLs' ratio bytes sit at PLLn RATE0 +
 *      0x20 (nvt-im-clk-bk.c), PLLn RATE0 from 0x10400; CPU_CKCTRL
 *      (0x100) bit 31 selects the CPU APLL, PLL16 x 8, so PLL16 is
 *      100 MHz, and writes to it are dropped, U-Boot's included.  Its
 *      resets (0x4C..0x5C) read as released; nothing else
 *      is preset, and the NA51089 gate and reset bits do not apply.
 *   4  NA51090 (NT98633, plat-na51090_a64/cg-reg.h): NA51089's block, but
 *      PLL0 and PLL2..PLL20 at 0x4420 + 0x40 n (PLL2 at 0x44A0), with
 *      na51090-clk.c's ratios.  NA51089's table would put its 470 MHz PLL3
 *      on PLL2, the 400 MHz that the OSG and hwcopy clocks may not exceed.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "hw/misc/nvt-cg.h"

#define TYPE_NVT_CG "nvt-cg"
OBJECT_DECLARE_SIMPLE_TYPE(NvtCgState, NVT_CG)

#define NVT_CG_SIZE         0x20000     /* NA51068's; the others map 0x10000 */
#define CG_PLL_EN           0x00
#define CG_PLL_STAT         0x04
#define CG_SYS_CLK_RATE     0x10
#define CG_SLEEP_MODE       0xA0
#define CG_CLK_EN0          0x70
#define CG_CLK_EN3          0x7C
#define CG_SYS_RESET0       0x80
#define CG_SYS_RESET2       0x88
#define CG_RESET_LINES      96

#define CPU_CLK_SEL_PLL8    1

struct NvtCgState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[NVT_CG_SIZE / 4];
    qemu_irq sys_reset[CG_RESET_LINES];
    uint8_t layout;
    uint32_t sleep_reg;
};

/* PLL ratio registers (RATE0; RATE1 / RATE2 follow) and the ratios the
 * vendor loader programs (na51089-clk.c / na51055-clk.c
 * DEFAULT[528]_PLLn_RATIO), PLL8 excepted -- see above. */
#define CG_LAYOUT_NA51089   0
#define CG_LAYOUT_NA51055   1
#define CG_LAYOUT_NA51084   2
#define CG_LAYOUT_NA51068   3
#define CG_LAYOUT_NA51090   4

typedef struct {
    uint32_t rate0;
    uint32_t ratio;
} NvtCgPll;

static const NvtCgPll nvt_cg_plls_na51089[] = {
    { 0x4420, 0x855555 },   /* PLL8: CPU, 800 MHz */
    { 0x4460, 0x500000 },   /* PLL11: DSI, 480 MHz */
    { 0x44A0, 0x4E5555 },   /* PLL3: DMA, 470 MHz */
    { 0x44E0, 0x320000 },   /* PLL6: IDE / ETH, 300 MHz */
    { 0x4520, 0x2EAAAA },   /* PLL9: IDE / ETH, 280 MHz */
    { 0x4560, 0x280000 },   /* PLL5: sensor, 240 MHz */
    { 0x45A0, 0x280000 },   /* PLL12: sensor 2, 240 MHz */
    { 0x45E0, 0x395810 },   /* PLL7: audio, 344.064 MHz */
    { 0 }
};

static const NvtCgPll nvt_cg_plls_na51055[] = {
    { 0x1188, 0x855555 },   /* PLL8: CPU, 800 MHz */
    { 0x1268, 0x500000 },   /* PLL11: DSI, 480 MHz */
    { 0x1208, 0x4E5555 },   /* PLL3: DMA, 470 MHz */
    { 0x1288, 0x320000 },   /* PLL6: IDE / ETH, 300 MHz */
    { 0x134C, 0x2EAAAA },   /* PLL9: IDE / ETH, 280 MHz */
    { 0x13D0, 0x280000 },   /* PLL5: sensor, 240 MHz */
    { 0x1340, 0x280000 },   /* PLL12: sensor 2, 240 MHz */
    { 0x12E8, 0x395810 },   /* PLL7: audio, 344.064 MHz */
    { 0 }
};

static const NvtCgPll nvt_cg_plls_na51084[] = {
    { 0x47E0, 0x10AAAA },   /* PLL8: CPU / 8, 100 MHz */
    { 0x47A0, 0x500000 },   /* PLL11: DSI, 480 MHz */
    { 0x4420, 0x26E000 },   /* PLL3: DMA, 233.25 MHz */
    { 0x4520, 0x320000 },   /* PLL6: IDE / ETH, 300 MHz */
    { 0x4560, 0x320000 },   /* PLL9: IDE / ETH, 300 MHz */
    { 0x4620, 0x280000 },   /* PLL5: sensor, 240 MHz */
    { 0x4660, 0x280000 },   /* PLL12: sensor 2, 240 MHz */
    { 0x46E0, 0x395810 },   /* PLL7: audio, 344.064 MHz */
    { 0x44A0, 0x640000 },   /* PLL10: CNN, 600 MHz */
    { 0 }
};

/* na51090-clk.c DEFAULT_PLLn_RATIO */
static const NvtCgPll nvt_cg_plls_na51090[] = {
    { 0x4420, 0x535555 },   /* PLL0: AXI0, 500 MHz */
    { 0x44A0, 0x42AAAA },   /* PLL2: AXI1, 400 MHz */
    { 0x44E0, 0x215555 },   /* PLL3: DRAM, 200 MHz */
    { 0x4520, 0x320000 },   /* PLL4: AHB, 300 MHz */
    { 0x4560, 0x10AAAA },   /* PLL5: SATA PHY, 100 MHz */
    { 0x45A0, 0x215555 },   /* PLL6: SDC, 200 MHz */
    { 0x45E0, 0x29AAAA },   /* PLL7: MAC, 250 MHz */
    { 0x4620, 0x640000 },   /* PLL8: CNN, 600 MHz */
    { 0x4660, 0x318000 },   /* PLL9: display 0, 297 MHz */
    { 0x46A0, 0x29AAAA },   /* PLL10: display 1, 250 MHz */
    { 0x46E0, 0x090000 },   /* PLL11: display 2, 54 MHz */
    { 0x4720, 0x3D70A3 },   /* PLL12: SSP, 368.64 MHz */
    { 0x4760, 0x3A5555 },   /* PLL13: AXI2, 350 MHz */
    { 0x47A0, 0x3F5555 },   /* PLL14: H.26x encoder, 380 MHz */
    { 0x47E0, 0x535555 },   /* PLL15: IPP, 500 MHz */
    { 0x4820, 0x1D2AAA },   /* PLL16: CPU / 8, 175 MHz */
    { 0x4860, 0x215555 },   /* PLL17: DRAM2, 200 MHz */
    { 0x48A0, 0x4B0000 },   /* PLL18: decoder, 450 MHz */
    { 0x48E0, 0x4B0000 },   /* PLL19: capture, 450 MHz */
    { 0x4920, 0x535555 },   /* PLL20: USB3 PHY, 500 MHz */
    { 0 }
};

/* na51068-clk.c DEFAULT_PLLn_RATIO, PLL16 excepted -- see above */
static const NvtCgPll nvt_cg_plls_na51068[] = {
    { 0x10420, 0x42AAAA },  /* PLL0: AXI0, 400 MHz */
    { 0x10460, 0x42AAAA },  /* PLL1: AXI1, 400 MHz */
    { 0x104A0, 0x535555 },  /* PLL2: CNN, 500 MHz */
    { 0x104E0, 0x215555 },  /* PLL3: DRAM, 200 MHz */
    { 0x10520, 0x320000 },  /* PLL4: AHB, 300 MHz */
    { 0x10820, 0x10AAAA },  /* PLL5: SATA PHY, 100 MHz */
    { 0x10560, 0x215555 },  /* PLL6: SDC, 200 MHz */
    { 0x105A0, 0x29AAAA },  /* PLL7: ETH, 250 MHz */
    { 0x105E0, 0x2AAAAA },  /* PLL8: UART, 256 MHz */
    { 0x10620, 0x318000 },  /* PLL9: DISP0, 297 MHz */
    { 0x10660, 0x29AAAA },  /* PLL10: DISP1, 250 MHz */
    { 0x106A0, 0x090000 },  /* PLL11: DISP2, 54 MHz */
    { 0x106E0, 0x395810 },  /* PLL12: SSP, 344.064 MHz */
    { 0x10720, 0x3A5555 },  /* PLL13: AXI2, 350 MHz */
    { 0x10760, 0x3F5555 },  /* PLL14: H.264 / H.265, 380 MHz */
    { 0x107A0, 0x640000 },  /* PLL15: DSP, 600 MHz */
    { 0x107E0, 0x10AAAA },  /* PLL16: CPU / 8, 100 MHz */
    { 0 }
};
#define CG51068_CPU_CKCTRL      0x100
#define CG51068_CPU_APLL        (1u << 31)
#define CG51068_RESET_FIRST     0x4C
#define CG51068_RESET_LAST      0x5C
#define CG51068_PLL16_RATE0     0x107E0     /* ratio bytes 0..2 */
#define CG51068_PLL16_RATE2     0x107E8

static const NvtCgPll *nvt_cg_plls(NvtCgState *s)
{
    switch (s->layout) {
    case CG_LAYOUT_NA51068:
        return nvt_cg_plls_na51068;
    case CG_LAYOUT_NA51055:
        return nvt_cg_plls_na51055;
    case CG_LAYOUT_NA51084:
        return nvt_cg_plls_na51084;
    case CG_LAYOUT_NA51090:
        return nvt_cg_plls_na51090;
    default:
        return nvt_cg_plls_na51089;
    }
}

/*
 * Audio: PLL7 (ratio at 0x45E0 / E4 / E8) divided by the eacdac divider
 * (0x38 [31:24]) and the DAI's (0x38 [23:16]) is the DAI clock, 256 x fs
 * (na51089-clk.c; Audio.o aud_set_codec_samplerate).  The kernel's clock
 * setup leaves 0x1B000100: 12.288 MHz, 48 kHz.
 */
#define CG_AUDIO_DIV        0x38

unsigned nvt_cg_audio_rate(DeviceState *dev)
{
    static const unsigned rates[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
    };
    NvtCgState *s = NVT_CG(dev);
    uint32_t pll7 = s->layout == CG_LAYOUT_NA51055 ? 0x12E8 :
                    s->layout == CG_LAYOUT_NA51084 ? 0x46E0 : 0x45E0;
    uint64_t ratio = (s->regs[pll7 / 4] & 0xFF) |
                     (s->regs[pll7 / 4 + 1] & 0xFF) << 8 |
                     (s->regs[pll7 / 4 + 2] & 0xFF) << 16;
    uint32_t div = s->regs[CG_AUDIO_DIV / 4];
    uint64_t fs = 12000000 * ratio / 131072 / (((div >> 24) & 0xFF) + 1) /
                  (((div >> 16) & 0xFF) + 1) / 256;
    unsigned best = 48000, i;

    for (i = 0; i < ARRAY_SIZE(rates); i++) {
        if (llabs((int64_t)fs - rates[i]) < llabs((int64_t)fs - best)) {
            best = rates[i];
        }
    }
    return best;
}

static uint64_t nvt_cg_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtCgState *s = opaque;

    if (offset == CG_PLL_STAT) {
        return s->regs[CG_PLL_EN / 4];
    }
    /*
     * SLEEP_MODE (0xA0, NA51089 / NA51055; 0xB0 on NA51103, "sleep-reg"):
     * the loader's DDR bring-up sets bits 2 and 4 and waits for bit 3, the
     * mode switch done.
     */
    if (offset == s->sleep_reg && s->layout != CG_LAYOUT_NA51068) {
        uint32_t v = s->regs[offset / 4];

        return v | ((v & 0x14) ? 0x8 : 0);
    }
    return s->regs[offset / 4];
}

static void nvt_cg_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    NvtCgState *s = opaque;
    uint32_t changed = s->regs[offset / 4] ^ val;
    int bit, line;

    /*
     * U-Boot's clock.c sets PLL16 to the tree's cpu freq (1400 MHz on the
     * EVB boards) before booting Linux, which would then take periph_clk as
     * 175 MHz against the a9mpcore's fixed 100 MHz: PLL16 stays 800 MHz.
     */
    if (s->layout == CG_LAYOUT_NA51068 && offset >= CG51068_PLL16_RATE0 &&
        offset <= CG51068_PLL16_RATE2) {
        return;
    }
    s->regs[offset / 4] = val;
    if (s->layout == CG_LAYOUT_NA51068 ||
        offset < CG_SYS_RESET0 || offset > CG_SYS_RESET2) {
        return;
    }
    for (bit = 0; bit < 32; bit++) {
        if (changed & (1u << bit)) {
            line = (offset - CG_SYS_RESET0) * 8 + bit;
            qemu_set_irq(s->sys_reset[line], !(val & (1u << bit)));
        }
    }
}

static const MemoryRegionOps nvt_cg_ops = {
    .read = nvt_cg_read,
    .write = nvt_cg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_cg_reset(DeviceState *dev)
{
    NvtCgState *s = NVT_CG(dev);
    const NvtCgPll *pll;
    hwaddr off;
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[CG_PLL_EN / 4] = 0xFFFFFFFF;
    for (pll = nvt_cg_plls(s); pll->rate0; pll++) {
        off = pll->rate0;
        s->regs[off / 4] = pll->ratio & 0xFF;
        s->regs[off / 4 + 1] = (pll->ratio >> 8) & 0xFF;
        s->regs[off / 4 + 2] = (pll->ratio >> 16) & 0xFF;
    }
    if (s->layout == CG_LAYOUT_NA51068) {
        s->regs[CG51068_CPU_CKCTRL / 4] = CG51068_CPU_APLL;
        for (off = CG51068_RESET_FIRST; off <= CG51068_RESET_LAST; off += 4) {
            s->regs[off / 4] = 0xFFFFFFFF;
        }
        return;
    }
    s->regs[CG_SYS_CLK_RATE / 4] = CPU_CLK_SEL_PLL8;
    for (off = CG_CLK_EN0; off <= CG_CLK_EN3; off += 4) {
        s->regs[off / 4] = 0xFFFFFFFF;
    }
    for (off = CG_SYS_RESET0; off <= CG_SYS_RESET2; off += 4) {
        s->regs[off / 4] = 0xFFFFFFFF;
    }
    for (i = 0; i < CG_RESET_LINES; i++) {
        qemu_irq_lower(s->sys_reset[i]);
    }
}

static void nvt_cg_realize(DeviceState *dev, Error **errp)
{
    NvtCgState *s = NVT_CG(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &nvt_cg_ops, s, TYPE_NVT_CG,
                          s->layout == CG_LAYOUT_NA51068 ? NVT_CG_SIZE :
                          0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void nvt_cg_init(Object *obj)
{
    NvtCgState *s = NVT_CG(obj);

    qdev_init_gpio_out_named(DEVICE(obj), s->sys_reset, "sys-reset",
                             CG_RESET_LINES);
}

static const VMStateDescription vmstate_nvt_cg = {
    .name = TYPE_NVT_CG,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtCgState, NVT_CG_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_cg_properties[] = {
    DEFINE_PROP_UINT8("layout", NvtCgState, layout, 0),
    DEFINE_PROP_UINT32("sleep-reg", NvtCgState, sleep_reg, CG_SLEEP_MODE),
};

static void nvt_cg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_cg_realize;
    device_class_set_legacy_reset(dc, nvt_cg_reset);
    device_class_set_props(dc, nvt_cg_properties);
    dc->vmsd = &vmstate_nvt_cg;
}

static const TypeInfo nvt_cg_info = {
    .name = TYPE_NVT_CG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtCgState),
    .instance_init = nvt_cg_init,
    .class_init = nvt_cg_class_init,
};

static void nvt_cg_register_types(void)
{
    type_register_static(&nvt_cg_info);
}

type_init(nvt_cg_register_types)
