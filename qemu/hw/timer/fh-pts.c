/*
 * Fullhan PTS: the media block's presentation time stamp counter.
 *
 * A free-running 32-bit count of microseconds at PAE + 0x40 (0xE7000040),
 * which the FH8632 kernel takes as its clocksource
 * (CONFIG_USE_PTS_AS_CLOCKSOURCE; mach-fh/time.c sets pts_clk to 1 MHz)
 * and the media drivers stamp frames with.  The rest of the PAE window is
 * the ARC coprocessor's and reads as zero.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_FH_PTS "fh-pts"
OBJECT_DECLARE_SIMPLE_TYPE(FhPtsState, FH_PTS)

#define FH_PTS_SIZE         0x1000
#define FH_PTS_COUNT        0x40

struct FhPtsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
};

static uint64_t fh_pts_read(void *opaque, hwaddr off, unsigned size)
{
    if (off == FH_PTS_COUNT) {
        return (uint32_t)qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    }
    return 0;
}

static void fh_pts_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
}

static const MemoryRegionOps fh_pts_ops = {
    .read = fh_pts_read,
    .write = fh_pts_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_pts_init(Object *obj)
{
    FhPtsState *s = FH_PTS(obj);

    memory_region_init_io(&s->iomem, obj, &fh_pts_ops, s, TYPE_FH_PTS,
                          FH_PTS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const TypeInfo fh_pts_info = {
    .name           = TYPE_FH_PTS,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhPtsState),
    .instance_init  = fh_pts_init,
};

static void fh_pts_register_types(void)
{
    type_register_static(&fh_pts_info);
}

type_init(fh_pts_register_types)
