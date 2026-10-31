/*
 * Ingenic NNDMA: the T40/T41 neural accelerator's DMA at 0x12500000.
 *
 * It moves 64-byte-aligned blocks between DDR and the NNA's ORAM
 * (0x12600000, the machine's RAM there).  Ingenic's AI library (libvenus
 * of the public magik-toolkit, dma_func.cc: w_d2o_orasetrun,
 * aie_run_nndma_rdchn / wrchn) drives it from user space through /dev/mem;
 * the kernel's soc-nna only answers its version ioctl:
 *
 *   0x0000..0x3FFF  descriptor RAM, 2048 descriptors of two words:
 *                     word 0  DDR byte address (bits 31:6); bit 0 set while
 *                             another descriptor follows; bits 5:1 the
 *                             length's bits 16:12
 *                     word 1  bits 31:20 the length's bits 11:0; bits
 *                             19:0 the ORAM offset from 0x12600000 in
 *                             64-byte units
 *                   the length is in 64-byte units, minus one
 *   0x8000 + 4 n    read channel n (0..3): DDR to ORAM
 *   0x8010 + 4 n    write channel n (0..3): ORAM to DDR
 *                   a write of bit 31 | (bytes / 64) << 12 | first
 *                   descriptor starts the channel; it walks the chain until
 *                   a descriptor without bit 0 (or the byte count, when not
 *                   0, is reached)
 *
 * The library waits for a channel with rdhwr $8/$9/$10 (read channels
 * 0..2) and $11/$12 (write channels 0..1), XBurst2 hardware registers the
 * CPU model answers (patch 0022).  A transfer completes when it is
 * started, so those reads find every channel done.  The rest of the window
 * (the kernel soc-nna driver's RCFG/WCFG registers at 0x2000, inside the
 * descriptor RAM, which no library uses) is storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_NNDMA "ingenic-nndma"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicNndmaState, INGENIC_NNDMA)

#define NNDMA_WINDOW    0x10000
#define DESRAM_SIZE     0x4000
#define NDES            (DESRAM_SIZE / 8)
#define R_CHAN          0x8000
#define NCHAN           8               /* 4 read, 4 write */
#define ORAM_PADDR      0x12600000u

struct IngenicNndmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t mem[NNDMA_WINDOW / 4];     /* descriptor RAM, channels, rest */
    uint64_t bytes[NCHAN];              /* moved, per channel (diagnostic) */
};

static void nndma_run(IngenicNndmaState *s, int ch, uint32_t val)
{
    bool to_oram = ch < 4;
    uint32_t idx = val & (NDES - 1);
    uint64_t limit = (uint64_t)extract32(val, 12, 19) << 6, done = 0;
    uint8_t buf[4096];
    int n;

    for (n = 0; n < NDES; n++) {
        uint32_t w0 = s->mem[idx * 2], w1 = s->mem[idx * 2 + 1];
        uint32_t len = ((extract32(w0, 1, 5) << 12) | (w1 >> 20)) + 1;
        hwaddr ddr = w0 & ~0x3fu;
        hwaddr oram = ORAM_PADDR + ((hwaddr)(w1 & 0xfffff) << 6);
        uint64_t left = (uint64_t)len << 6;

        while (left) {
            uint32_t k = MIN(left, sizeof(buf));
            hwaddr src = to_oram ? ddr : oram, dst = to_oram ? oram : ddr;

            dma_memory_read(&address_space_memory, src, buf, k,
                            MEMTXATTRS_UNSPECIFIED);
            dma_memory_write(&address_space_memory, dst, buf, k,
                             MEMTXATTRS_UNSPECIFIED);
            ddr += k;
            oram += k;
            left -= k;
        }
        done += (uint64_t)len << 6;
        if (!(w0 & 1) || (limit && done >= limit)) {
            break;
        }
        idx = (idx + 1) & (NDES - 1);
    }
    s->bytes[ch] += done;
}

static uint64_t ingenic_nndma_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicNndmaState *s = opaque;
    uint32_t v = s->mem[off >> 2];

    if (off >= R_CHAN && off < R_CHAN + 4 * NCHAN) {
        v &= ~(1u << 31);               /* every transfer has completed */
    }
    return v;
}

static void ingenic_nndma_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    IngenicNndmaState *s = opaque;

    s->mem[off >> 2] = val;
    if (off >= R_CHAN && off < R_CHAN + 4 * NCHAN && (val & (1u << 31))) {
        nndma_run(s, (off - R_CHAN) >> 2, val);
    }
}

static const MemoryRegionOps ingenic_nndma_ops = {
    .read = ingenic_nndma_read,
    .write = ingenic_nndma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
};

static void ingenic_nndma_reset(DeviceState *dev)
{
    IngenicNndmaState *s = INGENIC_NNDMA(dev);

    memset(s->mem, 0, sizeof(s->mem));
    memset(s->bytes, 0, sizeof(s->bytes));
}

static void ingenic_nndma_init(Object *obj)
{
    IngenicNndmaState *s = INGENIC_NNDMA(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_nndma_ops, s,
                          TYPE_INGENIC_NNDMA, NNDMA_WINDOW);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ingenic_nndma = {
    .name = TYPE_INGENIC_NNDMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(mem, IngenicNndmaState, NNDMA_WINDOW / 4),
        VMSTATE_UINT64_ARRAY(bytes, IngenicNndmaState, NCHAN),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_nndma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_nndma_reset);
    dc->vmsd = &vmstate_ingenic_nndma;
}

static const TypeInfo ingenic_nndma_info = {
    .name = TYPE_INGENIC_NNDMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicNndmaState),
    .instance_init = ingenic_nndma_init,
    .class_init = ingenic_nndma_class_init,
};

static void ingenic_nndma_register_types(void)
{
    type_register_static(&ingenic_nndma_info);
}

type_init(ingenic_nndma_register_types)
