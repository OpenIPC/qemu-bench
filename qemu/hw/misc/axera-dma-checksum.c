/*
 * Axera DMADIM: a hardware word-checksum DMA engine at 0x10460000
 * (boot/bl1/driver/axdma/axdma_checksum.c). Confirmed from a real
 * flash-file= boot of the AX620E SPI NOR dump spinning forever (290K+
 * accesses to DMADIM_REG_INT_RAW) once the earlier flash-read gap was
 * fixed: boot.c's calc_word_chksum() first tries this hardware engine
 * (axi_dma_word_checksum()) and only falls back to a plain CPU word-sum
 * loop if __ax_dma_checksum() itself reports failure -- it does NOT
 * fall back on a slow/never-finishing hardware poll, since that poll's
 * own 200ms software timeout (measuring against axera-bl1-timer.c's
 * down-counter) needs the exact same "two reads of inert catch-all
 * memory are always equal" problem already fixed for that timer, which
 * a plain unimplemented INT_RAW can't satisfy either: it never reports
 * done OR error, so __ax_dma_checksum()'s own poll just spins.
 *
 * Real register shape: software builds a small 5-word (40-byte) LLI
 * descriptor -- {u64 sar; u64 reserved; u64 size_in_8byte_units;
 * u64 magic_mode_word; u64 reserved} -- in its OWN memory, writes only
 * that descriptor's address into DMADIM_REG_LLI_BASE_H/L, then writes
 * DMADIM_REG_START to tell real hardware "go read that descriptor and
 * checksum what it describes". This model does exactly that: read the
 * descriptor back out of guest memory, sum the described byte range as
 * 32-bit words (the exact algorithm boot.c's own CPU fallback uses,
 * since both have to produce compatible results for software to
 * trust either one), store it in DMADIM_REG_CHECKSUME, and set
 * DMADIM_XFER_DONE in INT_RAW for the poll to see.
 *
 * Every other register (CH, CTRL, TRIG, INT_MASK) is plain RAM-backed
 * readback -- the real driver only ever writes them once per call with
 * no readback dependency.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_DMA_CHECKSUM "axera-dma-checksum"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraDmaChecksumState, AXERA_DMA_CHECKSUM)

#define AXERA_DMA_CHECKSUM_SIZE 0x200

#define DMADIM_START            0x0c
#define DMADIM_LLI_BASE_H       0x14
#define DMADIM_LLI_BASE_L       0x18
#define DMADIM_CHECKSUME        0x20
#define DMADIM_INT_CLR          0xfc
#define DMADIM_INT_RAW          0x100

#define DMADIM_XFER_DONE        BIT(0)

/*
 * BL1 only checksums boot images (tens of MiB at most); a descriptor
 * asking for more than this is treated like an unreadable range. The
 * range is summed through a fixed bounce buffer, never allocated whole.
 */
#define DMADIM_MAX_LEN          (256 * MiB)
#define DMADIM_CHUNK            (4 * KiB)

struct AxeraDmaChecksumState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[AXERA_DMA_CHECKSUM_SIZE / 4];
};

static void axera_dma_checksum_do_run(AxeraDmaChecksumState *s)
{
    uint64_t lli_addr;
    uint8_t desc[40];
    uint64_t sar, size_units, byte_len;
    uint8_t buf[DMADIM_CHUNK];
    uint32_t sum = 0;
    uint64_t done = 0;

    lli_addr = ((uint64_t)s->mem[DMADIM_LLI_BASE_H / 4] << 32) |
               s->mem[DMADIM_LLI_BASE_L / 4];

    if (address_space_read(&address_space_memory, lli_addr,
                           MEMTXATTRS_UNSPECIFIED, desc, sizeof(desc)) !=
        MEMTX_OK) {
        return;
    }

    sar = ldq_le_p(desc + 0);
    size_units = ldq_le_p(desc + 16);

    if (size_units > DMADIM_MAX_LEN / 8) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "axera-dma-checksum: length 0x%" PRIx64 " * 8 exceeds "
                      "the %u-byte limit\n", size_units,
                      (unsigned)DMADIM_MAX_LEN);
        byte_len = 0;
    } else {
        byte_len = size_units * 8;
    }

    /* byte_len is a multiple of 8, so every chunk holds whole words. */
    while (done < byte_len) {
        uint32_t n = MIN(byte_len - done, DMADIM_CHUNK);
        uint32_t i;

        if (address_space_read(&address_space_memory, sar + done,
                               MEMTXATTRS_UNSPECIFIED, buf, n) != MEMTX_OK) {
            /* Same result as before for an unreadable range: sum 0. */
            sum = 0;
            break;
        }
        for (i = 0; i < n; i += 4) {
            sum += ldl_le_p(buf + i);
        }
        done += n;
    }

    s->mem[DMADIM_CHECKSUME / 4] = sum;
    s->mem[DMADIM_INT_RAW / 4] |= DMADIM_XFER_DONE;
}

static void axera_dma_checksum_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    AxeraDmaChecksumState *s = opaque;

    if (offset == DMADIM_START) {
        axera_dma_checksum_do_run(s);
        return;
    }
    if (offset == DMADIM_INT_CLR) {
        s->mem[DMADIM_INT_RAW / 4] &= ~value;
        return;
    }
    if (offset < AXERA_DMA_CHECKSUM_SIZE) {
        s->mem[offset / 4] = value;
    }
}

static uint64_t axera_dma_checksum_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    AxeraDmaChecksumState *s = opaque;

    if (offset < AXERA_DMA_CHECKSUM_SIZE) {
        return s->mem[offset / 4];
    }
    return 0;
}

static const MemoryRegionOps axera_dma_checksum_ops = {
    .read = axera_dma_checksum_read,
    .write = axera_dma_checksum_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_dma_checksum_realize(DeviceState *dev, Error **errp)
{
    AxeraDmaChecksumState *s = AXERA_DMA_CHECKSUM(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_dma_checksum_ops, s,
                          TYPE_AXERA_DMA_CHECKSUM, AXERA_DMA_CHECKSUM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void axera_dma_checksum_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_dma_checksum_realize;
}

static const TypeInfo axera_dma_checksum_info = {
    .name = TYPE_AXERA_DMA_CHECKSUM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraDmaChecksumState),
    .class_init = axera_dma_checksum_class_init,
};

static void axera_dma_checksum_register_types(void)
{
    type_register_static(&axera_dma_checksum_info);
}

type_init(axera_dma_checksum_register_types)
