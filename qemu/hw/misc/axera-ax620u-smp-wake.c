/*
 * Axera AX620U's secondary-boot-reg @ 0x02220300: CPU0's own wake
 * handshake with CPU1-3, confirmed fatal (not a soft warning) if it
 * fails.
 *
 * Grounded in a live gdb trace of this real dump's own BL1-equivalent
 * (no source available, no symbols; ARM mode -- `set arm force-mode
 * thumb` must NOT be used here, unlike the rest of this SoC's Thumb
 * code, or the disassembly is garbage). CPU0 writes its own BL1 link
 * address (0x800, the vector table base) here, SEVs, then immediately
 * checks three fixed words (0x1329c/0x132a0/0x132a4, one per secondary
 * core, inside axera-ax620u-boot-sram): if *all three* already read
 * back as -1 (the "no problem reported" sentinel each core's own
 * reset path writes to its own word), it takes that as success on the
 * spot -- no retry, no delay, straight through to "boot form NAND".
 * Only if *any* word is something other than -1 does it fall into a
 * ~49ms poll-and-retry loop that, having never gone through the
 * explicit all-three-still-"-1" success assignment, ends up reading
 * an uninitialised success flag back out as failure and prints "wake
 * secondary cpus fail" -- confirmed by direct disassembly of the
 * retry function's own local variable layout, not a guess.
 *
 * CPU1-3 themselves (real code, confirmed via gdb) relocate the vector
 * table to address 0, read their own core ID (MPIDR), write -1 to
 * their own word on the way through, then spin waiting to be told
 * where to actually go (this flash-file= replay has nowhere real to
 * send them -- same limitation already documented elsewhere in this
 * file for CPU1-3 not being modelled at all; they stay parked there).
 *
 * The actual bug this device fixes has nothing to do with the wake
 * protocol's *logic* at all: this exact file offset (confirmed via a
 * raw hexdump of the real dump) is twelve zero bytes in the BL1 image
 * itself, and rom_add_blob_fixed's own reset-time reload writes those
 * zero bytes straight into memory -- through this same address, via
 * the normal memory-write path, same as any CPU access would. hisi-
 * regbank's own storage defaults to 0 anyway (g_malloc0), so with a
 * plain regbank here this reload is invisible; it only becomes a
 * problem once something (CPU1-3's own "-1" writes, or this device's
 * own reset default below) needs these words to actually read back as
 * -1. Whichever lands last between "the ROM's own zero reload" and
 * "a real core's own -1 write" wins the race -- confirmed live,
 * mixed 0/-1 outcomes and a ~30% pass rate with only a reset-time -1
 * default and no write filtering. Refusing to ever store a 0 here
 * removes the race entirely: nothing real ever needs a zero read back
 * out of these three words specifically, so there is no legitimate
 * write this rejects.
 *
 * Everything else in the secondary-boot-reg's own 4 KiB window is
 * plain storage, same as hisi-regbank -- this device only exists to
 * give the ack words themselves a reset-time -1 default and immunity
 * to the ROM reload's own zero bytes landing back on top of it.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/misc/axera-ax620u-smp-wake.h"
#include "hw/core/cpu.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraAx620uSmpWakeState, AXERA_AX620U_SMP_WAKE)

#define SMP_WAKE_SIZE           0x1000

#define ACK_WORDS_SIZE          0xc
#define ACK_NOT_YET             0xffffffffU

struct AxeraAx620uSmpWakeState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion ack_iomem;
    uint8_t *mem;
    uint32_t ack_words[ACK_WORDS_SIZE / 4];
};

static uint64_t axera_ax620u_smp_wake_read(void *opaque, hwaddr offset,
                                           unsigned size)
{
    AxeraAx620uSmpWakeState *s = opaque;
    uint64_t val = 0;

    memcpy(&val, s->mem + offset, size);
    return val;
}

/* secondary-boot-reg's offset inside this 4 KiB window (0x02220300) */
#define BOOTREG_OFF             0x300
#define DRAM_BASE               0x40000000U

static void axera_ax620u_smp_release_cpu(CPUState *cs, run_on_cpu_data data)
{
    /* An even address: ARM state, as the pen code expects. */
    cpu_set_pc(cs, data.host_int);
    cs->halted = 0;
}

static void axera_ax620u_smp_wake_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size)
{
    AxeraAx620uSmpWakeState *s = opaque;

    memcpy(s->mem + offset, &value, size);
    if (getenv("AXERA_SMP_DEBUG")) {
        qemu_log("smp-wake: W [0x%03x] = 0x%08x by cpu %d\n", (unsigned)offset,
                 (uint32_t)value, current_cpu ? current_cpu->cpu_index : -1);
    }

    /*
     * The Linux side of this spin table (vendor mach-axera/platsmp.c,
     * ax620x_smp_prepare_cpus()) writes the physical address of
     * secondary_holding_pen here exactly once, then SEVs, and
     * boot_secondary() gives each core 2 s to show up in that pen.
     * Under flash-file= the secondaries are still parked in the real
     * BL1/boot code, and how long they take to get from there to the
     * pen depends on timing: in many boots some or all of them arrive
     * after their window and stay offline ("CPUn: failed to boot:
     * -38"). A guest left on one CPU then starves its camera App into
     * an mmap_sem pile-up. Do what the park loop is there to do, at
     * the moment the kernel asks: send every secondary straight to the
     * pen (MMU off, privileged, as they already are). The pen then
     * waits for pen_release/the wake IPI exactly as on hardware.
     */
    if (offset == BOOTREG_OFF && size == 4 && (uint32_t)value >= DRAM_BASE) {
        CPUState *cs;

        CPU_FOREACH(cs) {
            if (cs->cpu_index != 0) {
                async_run_on_cpu(cs, axera_ax620u_smp_release_cpu,
                                 RUN_ON_CPU_HOST_INT((uint32_t)value));
            }
        }
    }
}

static const MemoryRegionOps axera_ax620u_smp_wake_ops = {
    .read = axera_ax620u_smp_wake_read,
    .write = axera_ax620u_smp_wake_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t axera_ax620u_smp_ack_read(void *opaque, hwaddr offset,
                                          unsigned size)
{
    AxeraAx620uSmpWakeState *s = opaque;

    return s->ack_words[offset / 4];
}

static void axera_ax620u_smp_ack_write(void *opaque, hwaddr offset,
                                       uint64_t value, unsigned size)
{
    AxeraAx620uSmpWakeState *s = opaque;

    /* See this file's own top comment: the real dump's own BL1 image
     * genuinely has zero bytes at this file offset (confirmed: a raw
     * hexdump shows 12 zero bytes there), and rom_add_blob_fixed's own
     * reset-time reload writes them straight through this same
     * overlay, racing CPU1-3's own legitimate -1 writes. Refusing to
     * store that specific value removes the race -- nothing here ever
     * has a real reason to read a "the ROM blob's own padding, not a
     * real report" zero back out. */
    if (value == 0) {
        return;
    }
    s->ack_words[offset / 4] = value;
}

static const MemoryRegionOps axera_ax620u_smp_ack_ops = {
    .read = axera_ax620u_smp_ack_read,
    .write = axera_ax620u_smp_ack_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_ax620u_smp_wake_reset(DeviceState *dev)
{
    AxeraAx620uSmpWakeState *s = AXERA_AX620U_SMP_WAKE(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(s->ack_words); i++) {
        s->ack_words[i] = ACK_NOT_YET;
    }
}

static void axera_ax620u_smp_wake_realize(DeviceState *dev, Error **errp)
{
    AxeraAx620uSmpWakeState *s = AXERA_AX620U_SMP_WAKE(dev);

    s->mem = g_malloc0(SMP_WAKE_SIZE);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_ax620u_smp_wake_ops, s,
                          TYPE_AXERA_AX620U_SMP_WAKE, SMP_WAKE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    memory_region_init_io(&s->ack_iomem, OBJECT(s), &axera_ax620u_smp_ack_ops,
                          s, TYPE_AXERA_AX620U_SMP_WAKE "-ack",
                          ACK_WORDS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->ack_iomem);
}

static void axera_ax620u_smp_wake_class_init(ObjectClass *klass,
                                             const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_ax620u_smp_wake_realize;
    device_class_set_legacy_reset(dc, axera_ax620u_smp_wake_reset);
}

static const TypeInfo axera_ax620u_smp_wake_info = {
    .name = TYPE_AXERA_AX620U_SMP_WAKE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraAx620uSmpWakeState),
    .class_init = axera_ax620u_smp_wake_class_init,
};

static void axera_ax620u_smp_wake_register_types(void)
{
    type_register_static(&axera_ax620u_smp_wake_info);
}

type_init(axera_ax620u_smp_wake_register_types)
