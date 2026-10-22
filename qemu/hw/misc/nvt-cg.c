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
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "hw/misc/nvt-cg.h"

#define TYPE_NVT_CG "nvt-cg"
OBJECT_DECLARE_SIMPLE_TYPE(NvtCgState, NVT_CG)

#define NVT_CG_SIZE         0x10000
#define CG_PLL_EN           0x00
#define CG_PLL_STAT         0x04
#define CG_SYS_CLK_RATE     0x10
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
};

/* PLL ratio-register bases and the ratios the vendor loader programs
 * (na51089-clk.c DEFAULT_PLLn_RATIO), PLL8 excepted -- see above. */
static const struct {
    uint32_t base;
    uint32_t ratio;
} nvt_cg_plls[] = {
    { 0x4400, 0x855555 },   /* PLL8: CPU, 800 MHz */
    { 0x4440, 0x500000 },   /* PLL11: DSI, 480 MHz */
    { 0x4480, 0x4E5555 },   /* PLL3: DMA, 470 MHz */
    { 0x44C0, 0x320000 },   /* PLL6: IDE / ETH, 300 MHz */
    { 0x4500, 0x2EAAAA },   /* PLL9: IDE / ETH, 280 MHz */
    { 0x4540, 0x280000 },   /* PLL5: sensor, 240 MHz */
    { 0x4580, 0x280000 },   /* PLL12: sensor 2, 240 MHz */
    { 0x45C0, 0x395810 },   /* PLL7: audio, 344.064 MHz */
};

/*
 * Audio: PLL7 (ratio at 0x45E0 / E4 / E8) divided by the eacdac divider
 * (0x38 [31:24]) and the DAI's (0x38 [23:16]) is the DAI clock, 256 x fs
 * (na51089-clk.c; Audio.o aud_set_codec_samplerate).  The kernel's clock
 * setup leaves 0x1B000100: 12.288 MHz, 48 kHz.
 */
#define CG_AUDIO_DIV        0x38
#define CG_PLL7_RATIO       0x45E0

unsigned nvt_cg_audio_rate(DeviceState *dev)
{
    static const unsigned rates[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
    };
    NvtCgState *s = NVT_CG(dev);
    uint64_t ratio = (s->regs[CG_PLL7_RATIO / 4] & 0xFF) |
                     (s->regs[CG_PLL7_RATIO / 4 + 1] & 0xFF) << 8 |
                     (s->regs[CG_PLL7_RATIO / 4 + 2] & 0xFF) << 16;
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
    return s->regs[offset / 4];
}

static void nvt_cg_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    NvtCgState *s = opaque;
    uint32_t changed = s->regs[offset / 4] ^ val;
    int bit, line;

    s->regs[offset / 4] = val;
    if (offset < CG_SYS_RESET0 || offset > CG_SYS_RESET2) {
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
    hwaddr off;
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[CG_PLL_EN / 4] = 0xFFFFFFFF;
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
    for (i = 0; i < ARRAY_SIZE(nvt_cg_plls); i++) {
        off = nvt_cg_plls[i].base + 0x20;
        s->regs[off / 4] = nvt_cg_plls[i].ratio & 0xFF;
        s->regs[off / 4 + 1] = (nvt_cg_plls[i].ratio >> 8) & 0xFF;
        s->regs[off / 4 + 2] = (nvt_cg_plls[i].ratio >> 16) & 0xFF;
    }
}

static void nvt_cg_init(Object *obj)
{
    NvtCgState *s = NVT_CG(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_cg_ops, s, TYPE_NVT_CG,
                          NVT_CG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), s->sys_reset, "sys-reset",
                             CG_RESET_LINES);
}

static const VMStateDescription vmstate_nvt_cg = {
    .name = TYPE_NVT_CG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtCgState, NVT_CG_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_cg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_cg_reset);
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
