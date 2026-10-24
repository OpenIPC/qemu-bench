/*
 * Axera AX_DMAPER: a generic LLI-descriptor memory-to-memory DMA engine
 * at 0x048a0000 (boot/bl1/driver/dmaper/dmaper.h, dmaper.c). Confirmed
 * from a real flash-file= boot of the AX620E SPI NOR dump spinning
 * forever (25M+ accesses in a 40s run) inside axi_dma_wait_xfer_done()'s
 * `while (!readl(AX_DMAPER_INT_STA(ch))) ;` -- a real hardware
 * completion poll with NO timeout of its own, same shape as every other
 * "write enable/start, poll a ready/done bit" gap this project has
 * already hit (axera-pllc.c, axera-ddr-pll.c, the AX620U clock regbank),
 * except this one genuinely needs to move real bytes: BL1 stages a real
 * SPI NOR read into a scratch buffer, then uses this DMA engine to copy
 * it into the image's final DRAM destination (confirmed empirically --
 * before this device existed, that final destination stayed all-zero
 * forever even though the real file's bytes checksum correctly at the
 * scratch location, and BL1's own axi_dma_xfer_start()/
 * axi_dma_wait_xfer_done() pair is the only thing between the two).
 *
 * Real register shape (dmaper.c's ax_dma_per_set_lli()): software
 * builds an "LLI" descriptor -- {u64 sar; u64 dar; a 64-bit ctrl
 * bitfield; a 64-bit data bitfield with block_ts:22 at bit 37} -- in
 * its OWN memory, then writes only that descriptor's address into
 * AX_DMAPER_CHN_LLI_H/L(ch). Writing AX_DMAPER_START's channel bit is
 * real hardware being told "go fetch that descriptor and do the
 * transfer it describes" -- so this model does exactly that: read the
 * 32-byte descriptor back out of guest memory, pull src/dst/length out
 * of it (xfer_len = block_ts << ctrl.src_tr_wridth, the exact inverse
 * of how ax_dma_per_set_lli() computed block_ts), copy that many bytes
 * guest-memory-to-guest-memory, then set the channel's completion bit
 * (AX_DMAPER_INT_STA(ch) bit 0, CHN_TRANSF_DONE) for the poll to see.
 * AX_DMAPER_INT_CLR(ch) clears it back, matching
 * axi_dma_wait_xfer_done()'s own cleanup write.
 *
 * Every other register (CHN_EN, CTRL, clock/reset helpers, priority,
 * the other interrupt-mask/raw/error registers) is plain RAM-backed
 * readback -- BL1's own driver only read-modify-writes them, never
 * polls a hardware-driven bit in any of them.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_DMAPER "axera-dmaper"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraDmaperState, AXERA_DMAPER)

#define AXERA_DMAPER_SIZE       0x400

#define DMAPER_START            0x0c
#define DMAPER_CHN_LLI_H(ch)    (0x20 + (ch) * 8)
#define DMAPER_CHN_LLI_L(ch)    (0x24 + (ch) * 8)
#define DMAPER_INT_CLR(ch)      (0xf8 + (ch) * 0x10)
#define DMAPER_INT_STA(ch)      (0x100 + (ch) * 0x10)

#define DMAPER_MAX_CHN          16
#define CHN_TRANSF_DONE         BIT(0)

struct AxeraDmaperState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[AXERA_DMAPER_SIZE / 4];
};

static void axera_dmaper_do_transfer(AxeraDmaperState *s, unsigned ch)
{
    uint64_t lli_addr;
    uint8_t desc[32];
    uint64_t sar, dar, ctrl, data;
    uint32_t src_tr_wridth, block_ts;
    uint64_t xfer_len;
    bool sinc, dinc;
    uint8_t *buf;
    uint64_t i;

    lli_addr = ((uint64_t)s->mem[DMAPER_CHN_LLI_H(ch) / 4] << 32) |
               s->mem[DMAPER_CHN_LLI_L(ch) / 4];

    if (address_space_read(&address_space_memory, lli_addr,
                           MEMTXATTRS_UNSPECIFIED, desc, sizeof(desc)) !=
        MEMTX_OK) {
        return;
    }

    sar = ldq_le_p(desc + 0);
    dar = ldq_le_p(desc + 8);
    ctrl = ldq_le_p(desc + 16);
    data = ldq_le_p(desc + 24);

    src_tr_wridth = (ctrl >> 27) & 0x7;
    sinc = (ctrl >> 31) & 1;
    dinc = (ctrl >> 30) & 1;
    block_ts = (data >> 37) & 0x3fffff;
    xfer_len = (uint64_t)block_ts << src_tr_wridth;

    if (xfer_len == 0) {
        return;
    }

    /*
     * ax_dma_per_set_lli()'s DMA_DEV_TO_MEM case sets sinc=0: the real
     * source here is a device register's own FIFO (the SPI controller's
     * DR, for every real transfer this model has seen), which drains
     * one real byte per 4-byte-wide bus access -- not `xfer_len`
     * sequential source addresses -- so a non-incrementing side has to
     * be read (or written, for the symmetric MEM_TO_DEV case) one
     * fixed-address access at a time, same as real silicon stepping a
     * single peripheral register through the transfer instead of
     * walking memory.
     */
    buf = g_malloc(xfer_len);

    if (sinc) {
        if (address_space_read(&address_space_memory, sar,
                               MEMTXATTRS_UNSPECIFIED, buf, xfer_len) !=
            MEMTX_OK) {
            g_free(buf);
            return;
        }
    } else {
        for (i = 0; i < xfer_len; i++) {
            uint32_t word = 0;

            address_space_read(&address_space_memory, sar,
                               MEMTXATTRS_UNSPECIFIED, &word, sizeof(word));
            buf[i] = word & 0xff;
        }
    }

    if (dinc) {
        address_space_write(&address_space_memory, dar,
                            MEMTXATTRS_UNSPECIFIED, buf, xfer_len);
    } else {
        for (i = 0; i < xfer_len; i++) {
            uint32_t word = buf[i];

            address_space_write(&address_space_memory, dar,
                                MEMTXATTRS_UNSPECIFIED, &word, sizeof(word));
        }
    }

    g_free(buf);

    s->mem[DMAPER_INT_STA(ch) / 4] |= CHN_TRANSF_DONE;
}

static void axera_dmaper_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    AxeraDmaperState *s = opaque;
    unsigned ch;

    if (offset == DMAPER_START) {
        for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
            if (value & BIT(ch)) {
                axera_dmaper_do_transfer(s, ch);
            }
        }
        return;
    }

    for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
        if (offset == DMAPER_INT_CLR(ch)) {
            s->mem[DMAPER_INT_STA(ch) / 4] &= ~value;
            return;
        }
    }

    if (offset < AXERA_DMAPER_SIZE) {
        s->mem[offset / 4] = value;
    }
}

static uint64_t axera_dmaper_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraDmaperState *s = opaque;

    if (offset < AXERA_DMAPER_SIZE) {
        return s->mem[offset / 4];
    }
    return 0;
}

static const MemoryRegionOps axera_dmaper_ops = {
    .read = axera_dmaper_read,
    .write = axera_dmaper_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_dmaper_realize(DeviceState *dev, Error **errp)
{
    AxeraDmaperState *s = AXERA_DMAPER(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_dmaper_ops, s,
                          TYPE_AXERA_DMAPER, AXERA_DMAPER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void axera_dmaper_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_dmaper_realize;
}

static const TypeInfo axera_dmaper_info = {
    .name = TYPE_AXERA_DMAPER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraDmaperState),
    .class_init = axera_dmaper_class_init,
};

static void axera_dmaper_register_types(void)
{
    type_register_static(&axera_dmaper_info);
}

type_init(axera_dmaper_register_types)
