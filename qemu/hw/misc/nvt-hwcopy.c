/*
 * Novatek NA51103 / NA51090 hardware copy engine ("nvt,nvt_hwcopy",
 * 0x2_F0330000, SPI 70): memory fills and copies for the HDAL
 * (kdrv_hwcopy.ko, as decompiled: hwcopy_request_gop, hwcopy_trigger,
 * copy_isr and its own self-test, emu_hwcopy_auto).
 *
 *   0x00 START     write 1 to run the job
 *   0x04 MODE      [1:0] 0 linear set, 1 block set, 2 linear copy,
 *                  3 block copy; bit 6 cleared at open
 *   0x08 INT_EN    bit 0: the done flag interrupts
 *   0x0C INT_STS   bit 0 done; write 1 to clear
 *   0x14           block copy: the source's line offset (bytes)
 *   0x18 / 0x1C    block: the lines and the bytes a line
 *   0x24           block: the destination's line offset
 *   0x28           linear: the length in bytes
 *   0x2C           set: the 32-bit pattern, little-endian in memory
 *   0x40 / 0x44    the source address, bits 31:0 / 35:32
 *   0x48 / 0x4C    the destination address, bits 31:0 / 35:32
 *
 * A job runs when it is started; its done flag follows a little later, as
 * the driver waits for the interrupt.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_NVT_HWCOPY "nvt-hwcopy"
OBJECT_DECLARE_SIMPLE_TYPE(NvtHwcopyState, NVT_HWCOPY)

#define HWCP_SIZE       0x100
#define HWCP_START      0x00
#define HWCP_MODE       0x04
#define HWCP_INT_EN     0x08
#define HWCP_INT_STS    0x0C
#define HWCP_SRC_LOFS   0x14
#define HWCP_LINES      0x18
#define HWCP_WIDTH      0x1C
#define HWCP_DST_LOFS   0x24
#define HWCP_LEN        0x28
#define HWCP_PATTERN    0x2C
#define HWCP_SRC_LO     0x40
#define HWCP_SRC_HI     0x44
#define HWCP_DST_LO     0x48
#define HWCP_DST_HI     0x4C

#define MODE_LINE_SET   0
#define MODE_BLOCK_SET  1
#define MODE_LINE_COPY  2
#define MODE_BLOCK_COPY 3

#define HWCP_JOB_NS     20000       /* a job's done flag, 20 us later */
#define HWCP_MAX        (64 * MiB)  /* the 26-bit length */

struct NvtHwcopyState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;
    uint32_t regs[HWCP_SIZE / 4];
};

static uint32_t *reg(NvtHwcopyState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static hwaddr hwcp_addr(NvtHwcopyState *s, hwaddr lo)
{
    return *reg(s, lo) | (hwaddr)(*reg(s, lo + 4) & 0xF) << 32;
}

static void hwcp_update_irq(NvtHwcopyState *s)
{
    qemu_set_irq(s->irq, *reg(s, HWCP_INT_STS) & *reg(s, HWCP_INT_EN) & 1);
}

/* Fill n bytes at addr with the pattern, aligned to the line start. */
static void hwcp_set(hwaddr addr, uint32_t pattern, uint32_t n, uint8_t *buf)
{
    uint32_t i;

    for (i = 0; i < n; i++) {
        buf[i] = pattern >> (8 * (i % 4));
    }
    dma_memory_write(&address_space_memory, addr, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
}

static void hwcp_run(NvtHwcopyState *s)
{
    uint32_t mode = *reg(s, HWCP_MODE) & 3;
    hwaddr src = hwcp_addr(s, HWCP_SRC_LO), dst = hwcp_addr(s, HWCP_DST_LO);
    uint32_t len = *reg(s, HWCP_LEN) & 0x3FFFFFF;
    uint32_t lines = *reg(s, HWCP_LINES) & 0x3FFF;
    uint32_t width = *reg(s, HWCP_WIDTH) & 0x3FFF;
    uint32_t slofs = *reg(s, HWCP_SRC_LOFS) & 0x1FFFF;
    uint32_t dlofs = *reg(s, HWCP_DST_LOFS) & 0x1FFFF;
    g_autofree uint8_t *buf = NULL;
    uint32_t y;

    switch (mode) {
    case MODE_LINE_SET:
    case MODE_LINE_COPY:
        if (!len || len > HWCP_MAX) {
            break;
        }
        buf = g_malloc(len);
        if (mode == MODE_LINE_SET) {
            hwcp_set(dst, *reg(s, HWCP_PATTERN), len, buf);
        } else {
            dma_memory_read(&address_space_memory, src, buf, len,
                            MEMTXATTRS_UNSPECIFIED);
            dma_memory_write(&address_space_memory, dst, buf, len,
                             MEMTXATTRS_UNSPECIFIED);
        }
        break;
    default:
        if (!width) {
            break;
        }
        buf = g_malloc(width);
        for (y = 0; y < lines; y++) {
            if (mode == MODE_BLOCK_SET) {
                hwcp_set(dst + (hwaddr)y * dlofs, *reg(s, HWCP_PATTERN),
                         width, buf);
            } else {
                dma_memory_read(&address_space_memory,
                                src + (hwaddr)y * slofs, buf, width,
                                MEMTXATTRS_UNSPECIFIED);
                dma_memory_write(&address_space_memory,
                                 dst + (hwaddr)y * dlofs, buf, width,
                                 MEMTXATTRS_UNSPECIFIED);
            }
        }
        break;
    }
}

static void hwcp_done(void *opaque)
{
    NvtHwcopyState *s = opaque;

    *reg(s, HWCP_INT_STS) |= 1;
    hwcp_update_irq(s);
}

static uint64_t nvt_hwcopy_read(void *opaque, hwaddr off, unsigned size)
{
    NvtHwcopyState *s = opaque;

    return off == HWCP_START ? 0 : *reg(s, off);
}

static void nvt_hwcopy_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    NvtHwcopyState *s = opaque;

    switch (off) {
    case HWCP_START:
        if (val & 1) {
            hwcp_run(s);
            timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      HWCP_JOB_NS);
        }
        return;
    case HWCP_INT_STS:
        *reg(s, off) &= ~val;
        break;
    default:
        *reg(s, off) = val;
        break;
    }
    hwcp_update_irq(s);
}

static const MemoryRegionOps nvt_hwcopy_ops = {
    .read = nvt_hwcopy_read,
    .write = nvt_hwcopy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_hwcopy_reset(DeviceState *dev)
{
    NvtHwcopyState *s = NVT_HWCOPY(dev);

    timer_del(s->done);
    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void nvt_hwcopy_init(Object *obj)
{
    NvtHwcopyState *s = NVT_HWCOPY(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_hwcopy_ops, s,
                          TYPE_NVT_HWCOPY, HWCP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, hwcp_done, s);
}

static const VMStateDescription vmstate_nvt_hwcopy = {
    .name = TYPE_NVT_HWCOPY,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtHwcopyState, HWCP_SIZE / 4),
        VMSTATE_TIMER_PTR(done, NvtHwcopyState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_hwcopy_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_hwcopy_reset);
    dc->vmsd = &vmstate_nvt_hwcopy;
}

static const TypeInfo nvt_hwcopy_info = {
    .name = TYPE_NVT_HWCOPY,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtHwcopyState),
    .instance_init = nvt_hwcopy_init,
    .class_init = nvt_hwcopy_class_init,
};

static void nvt_hwcopy_register_types(void)
{
    type_register_static(&nvt_hwcopy_info);
}

type_init(nvt_hwcopy_register_types)
