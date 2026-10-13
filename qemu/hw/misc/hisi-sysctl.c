/*
 * HiSilicon System Controller (SysCtrl) emulation.
 *
 * Provides SoC identification, system reset, and general-purpose
 * registers for all supported HiSilicon SoC generations (V2/V3/V4).
 *
 * Copyright (c) 2020-2021, 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "system/runstate.h"

#define TYPE_HISI_SYSCTL "hisi-sysctl"
OBJECT_DECLARE_SIMPLE_TYPE(HisiSysctlState, HISI_SYSCTL)

/*
 * Register space is 64K to cover the full range mapped by firmware.
 * V2 DTS maps 0x20050000 size 0x10000; vendor modules do write-then-read
 * cycles across the full range — a 4K window loses writes beyond 0x1000
 * and causes poll loops to hang.
 */
#define HISI_SYSCTL_MMIO_SIZE   0x10000

/* Number of 32-bit general-purpose storage words */
#define HISI_SYSCTL_NREGS       (HISI_SYSCTL_MMIO_SIZE / 4)

/*
 * SC_CTRL bit 8: set by the mask-ROM's final handoff thunk to drop the
 * reset-time alias of the mask-ROM at address 0 (openhisilicon
 * bootrom/hi3516av300/re/start.S, clear_remap_start).  Cleared by reset.
 */
#define SC_CTRL_REMAP_CLEAR     (1 << 8)

/*
 * SC_CTRL[2:0] requests a system clock mode (2 = slow, 4 = normal) and
 * SC_CTRL[6:3] reports the mode the clock tree is in.  There is no PLL to
 * wait for here, so the status follows the request at once.  The V1 3.0.8
 * kernel's tcm_slow_reset (mach-hi3518/tcm_reset.S) requests slow mode and
 * polls the status for it before writing SC_SYSRES.
 */
#define SC_CTRL_MODE_CTRL_MASK      0x7
#define SC_CTRL_MODE_STATUS_SHIFT   3
#define SC_CTRL_MODE_STATUS_MASK    (0xf << SC_CTRL_MODE_STATUS_SHIFT)

struct HisiSysctlState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t soc_id;
    bool byte_layout;
    uint8_t chip_variant;
    /*
     * V1-family chip identification registers.  ipctool's get_chip_V1()
     * reads SCSYSID0 (giving family ID 0x35180100) and then disambiguates
     * the four V1 variants via two non-standard registers:
     *   0x88 — read-only chip ID byte (1 = 3516CV100, 2 = 3518EV100,
     *          3 = 3518AV100); driver writes 3 first then reads back,
     *          but real silicon ignores the write.
     *   0x8C — read-only field with bits[14:8] holding a chip-specific
     *          tag (0x10 = 3518CV100, 0x57 = 3518EV100); takes priority
     *          over 0x88 when matched.
     * Both default to 0 (= "match nothing"); per-SoC machine init sets
     * the value that lets ipctool identify the chip.
     */
    uint32_t v1_chip_id_88;
    uint32_t v1_chip_id_8c;
    uint32_t regs[HISI_SYSCTL_NREGS];
    /*
     * Power-on image of regs[]: whatever machine init stored (SYSSTAT boot
     * straps, per-SoC preset words), captured at the first system reset,
     * which runs after machine init and before the guest.  Later resets
     * restore it, discarding every word the guest wrote.
     */
    uint32_t por_regs[HISI_SYSCTL_NREGS];
    bool por_valid;
    qemu_irq remap_clear;       /* SC_CTRL bit 8 level */
};

static uint64_t hisi_sysctl_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiSysctlState *s = HISI_SYSCTL(opaque);

    switch (offset) {
    case 0x00: /* SC_CTRL */
        return (s->regs[0] & ~SC_CTRL_MODE_STATUS_MASK) |
               ((s->regs[0] & SC_CTRL_MODE_CTRL_MASK)
                << SC_CTRL_MODE_STATUS_SHIFT);
    case 0x04: /* SC_SYSRES — system reset (write-only, read returns 0) */
        return 0;
    case 0x88: /* V1 chip ID register (read-only, write-ignored) */
        return s->v1_chip_id_88;
    case 0x8C: /* V1 chip ID + REG_SYSSTAT (boot mode) — bits[14:8] hold
                * the chip tag (0x10 / 0x57); other bits unused (= 0 =
                * SPI NOR boot mode). */
        return s->v1_chip_id_8c | s->regs[0x8C / 4];
    /*
     * SCSYSID0..3 chip identification block. Two layouts coexist:
     *
     *   word layout (V4+, e.g. EV200/EV300/CV500): SCSYSID0 returns the
     *     full 32-bit packed ID (e.g. 0x3516E300); SCSYSID1..3 unused.
     *     Vendor V4 sys.o SYS_HAL_GetChipID reads SCSYSID0 as one u32.
     *
     *   byte layout (V1/V2/V2A/V3/V3A): each register returns one byte
     *     of the family ID, low byte first; SCSYSID0 also carries the
     *     chip sub-variant byte at bits 24-31. ipctool's
     *     hisi_detect_cpu() and the vendor V3 sys.ko's SYS_HAL_GetChipID
     *     rely on this — the latter reads SCSYSID0 >> 24 as the variant
     *     (0 = CV300, 1 = CV200, 4 = EV100, etc.).
     *
     * The active layout is chosen per-SoC via the byte-layout-id
     * property (default = word); chip-variant supplies the sub-variant
     * byte and defaults to 0.
     */
    case 0xEE0: /* SCSYSID0 */
        return s->byte_layout
            ? (((uint32_t)s->chip_variant << 24) | (s->soc_id & 0xff))
            : s->soc_id;
    case 0xEE4: /* SCSYSID1 */
        return s->byte_layout ? ((s->soc_id >> 8) & 0xff) : 0;
    case 0xEE8: /* SCSYSID2 */
        return s->byte_layout ? ((s->soc_id >> 16) & 0xff) : 0;
    case 0xEEC: /* SCSYSID3 */
        return s->byte_layout ? ((s->soc_id >> 24) & 0xff) : 0;
    default:
        if (offset < HISI_SYSCTL_MMIO_SIZE) {
            return s->regs[offset / 4];
        }
        return 0;
    }
}

static void hisi_sysctl_write(void *opaque, hwaddr offset,
                               uint64_t val, unsigned size)
{
    HisiSysctlState *s = HISI_SYSCTL(opaque);

    switch (offset) {
    case 0x04: /* SC_SYSRES — system reset */
        qemu_log_mask(LOG_UNIMP,
                      "hisi-sysctl: SC_SYSRES reset triggered (val=0x%08x)\n",
                      (uint32_t)val);
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        break;
    case 0x00: /* SC_CTRL */
        s->regs[0] = (uint32_t)val;
        qemu_set_irq(s->remap_clear, !!(val & SC_CTRL_REMAP_CLEAR));
        break;
    case 0x88: /* V1 chip ID register — read-only, drop writes */
        break;
    default:
        if (offset < HISI_SYSCTL_MMIO_SIZE) {
            s->regs[offset / 4] = (uint32_t)val;
        }
        break;
    }
}

static const MemoryRegionOps hisi_sysctl_ops = {
    .read = hisi_sysctl_read,
    .write = hisi_sysctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_sysctl_init(Object *obj)
{
    HisiSysctlState *s = HISI_SYSCTL(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_sysctl_ops, s,
                          TYPE_HISI_SYSCTL, HISI_SYSCTL_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->remap_clear, "remap-clear", 1);
}

static void hisi_sysctl_reset(DeviceState *dev)
{
    HisiSysctlState *s = HISI_SYSCTL(dev);

    /*
     * SC_SYSRES is a whole-chip reset: every word returns to its power-on
     * value, not just the boot remap.  The vendor 4.9 GIC driver depends
     * on that: it stamps 0x47444946 ("GDIF") at +0x130 after initialising
     * the distributor and skips gic_dist_init() when the stamp is already
     * there, so a stamp that survived the reset leaves the freshly reset
     * GIC disabled and the rebooted kernel without a timer interrupt.
     * Whoever consumes "remap-clear" restores its own reset state.
     */
    if (!s->por_valid) {
        memcpy(s->por_regs, s->regs, sizeof(s->por_regs));
        s->por_valid = true;
    } else {
        memcpy(s->regs, s->por_regs, sizeof(s->regs));
    }
    s->regs[0] &= ~SC_CTRL_REMAP_CLEAR;
}

static const Property hisi_sysctl_properties[] = {
    DEFINE_PROP_UINT32("soc-id", HisiSysctlState, soc_id, 0),
    DEFINE_PROP_BOOL("byte-layout-id", HisiSysctlState, byte_layout, false),
    DEFINE_PROP_UINT8("chip-variant", HisiSysctlState, chip_variant, 0),
    DEFINE_PROP_UINT32("v1-chip-id-88", HisiSysctlState, v1_chip_id_88, 0),
    DEFINE_PROP_UINT32("v1-chip-id-8c", HisiSysctlState, v1_chip_id_8c, 0),
};

static void hisi_sysctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, hisi_sysctl_properties);
    device_class_set_legacy_reset(dc, hisi_sysctl_reset);
}

static const TypeInfo hisi_sysctl_info = {
    .name          = TYPE_HISI_SYSCTL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiSysctlState),
    .instance_init = hisi_sysctl_init,
    .class_init    = hisi_sysctl_class_init,
};

static void hisi_sysctl_register_types(void)
{
    type_register_static(&hisi_sysctl_info);
}

type_init(hisi_sysctl_register_types)
