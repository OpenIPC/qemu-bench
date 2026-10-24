/*
 * Axera GZIPD: the real hardware decompression engine for Axera's own
 * proprietary "AXGZIP" ("z20e") format, at 0x10410000 (boot/bl1/driver/
 * gzipd/ax_gzipd_reg.h, ax_gzipd_drv.c). boot.c's read_image_data()
 * routes every non-DDRINIT flash image (confirmed: real U-Boot images
 * on this project's real AX620E NOR dump) through this engine via
 * gzip_pipeline_flash_read() -- this is not optional for a NOR boot
 * once SUPPPORT_GZIPD is compiled in, and it is real, deliberately-used
 * decompression (the real dump's U-Boot body genuinely starts with the
 * "20" container magic), not a coincidence.
 *
 * AXGZIP is Axera's own LZ77 + canonical-Huffman codec -- same
 * algorithmic family as DEFLATE, a different bitstream, not a wrapper
 * around zlib/LZ4/zstd. No source for the algorithm itself exists
 * anywhere (checked the vendor SDK and both GitHub mirrors of it,
 * Sipeed's and Axera's own official one -- every copy ships only the
 * compiled host-side packing tool, tools/ax_gzip_tool/ax_gzip,
 * committed as a binary from its very first commit). The format below
 * was reverse-engineered against that real tool as a validation oracle
 * (round-tripped a 61-file black-box test corpus plus fresh test
 * vectors, cross-checked the Huffman canonical-code assignment against
 * the tool's own live execution under gdb) and independently verified
 * here against the real AX620E dump's own U-Boot image: the decoded
 * output starts with real AArch64 code (a branch + nop pair) whose own
 * embedded text_offset field reads exactly 0x5c000400 -- this project's
 * own UBOOT_IMG_HEADER_BASE + sizeof(img_header), the real link address
 * BL1 injects it at.
 *
 * Container (as BL1's own CPU reads it from flash, before the hardware
 * engine is ever touched -- see below): a 16-byte file header {"20",
 * blk_num:u16, osize:u32, isize:u32, icrc32:u32}, then blk_num blocks.
 * Each block: a 16-byte sub-header {chead0 (data length - 1), chead1
 * (CRC-32/MPEG-2 of the data region), two CRC checkpoint words not
 * needed for decoding}, a 720-byte canonical-Huffman table region
 * (664B literal/length table, alphabet 512 -- symbols 0-255 are
 * literal bytes, 256-511 are match lengths 3-258 -- capped at 22 bits;
 * 55B distance table, alphabet 24, DEFLATE's own distance base/extra-
 * bits table truncated to a 4096-byte window, capped at 15 bits; 1
 * CRC-8/SMBus byte over the whole 720), then chead0+1 bytes of
 * MSB-first Huffman-coded bitstream data. Each table is stored as its
 * canonical-code (firstcode-indexed-by-length, JPEG-Annex-F-style)
 * parameters followed by a flat rank->symbol array. The distance
 * table's 16th stored entry (unused by the canonical code itself,
 * which only needs 15) is repurposed to hold this block's own token
 * count: ntok = (raw_20bit_value + 1) * 4096 -- confirmed against the
 * real tool's own execution via gdb register tracing. A non-final
 * block decodes exactly ntok literal/length symbols; the final block
 * instead decodes until total output reaches osize.
 *
 * Real register sequence (ax_gzipd_drv.c) -- and the one real surprise
 * found wiring this up against the actual dump: gzip_pipeline_flash_
 * read() parses the 16-byte file header itself, in software, straight
 * off the staged flash read (gzipd_dev_get_header_info()) BEFORE ever
 * touching a GZIPD register -- the hardware engine never sees that
 * header at all. gzipd_dev_cfg() then writes WDMA_CFG1 (osize-1),
 * WDMA_CFG0 (dest_addr>>3), and RDMA_CFG0 ((tile_cnt-1) | (0x10<<11) |
 * (blk_num<<16) | bypass<<31) once per image -- RDMA_CFG0's blk_num
 * field is this model's only source for it, since the in-stream
 * header isn't visible to hardware. gzipd_dev_run()/_run_last_tile()
 * then write RDMA_CFG1 (tile addr>>3) immediately followed by
 * RDMA_CFG2 (tile_size-1) once per compressed-data TILE, starting
 * right after the 16-byte header (tile_addr = img_compressed_addr +
 * sizeof(header_info)) -- a buffer-management chunking BL1 uses to
 * stream from flash through a small staging area, unrelated to this
 * format's own block boundaries (confirmed directly: the first tile's
 * real address was off by exactly 16 bytes from where this model
 * first assumed the container header would be).
 *
 * This model doesn't need to replicate the real FIFO-depth/tile-
 * batching timing: it accumulates each tile's real bytes (read via
 * address_space_read) in arrival order on every RDMA_CFG2 write --
 * this buffer holds block data only, offset 0 is the first block's
 * own sub-header, there is no leading file header to skip -- and
 * after every append, tries a cheap dry run "walking" blk_num blocks
 * forward using only each block's own chead0 length field (no real
 * decode work) to tell whether enough bytes have arrived yet. Once
 * that walk succeeds, it decodes the whole thing in one shot and
 * writes the result to the WDMA destination -- same "complete
 * synchronously inside the MMIO write handler" simplification already
 * used throughout this project's other DMA-shaped devices
 * (axera-dmaper.c, axera-dma-checksum.c).
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_GZIPD "axera-gzipd"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraGzipdState, AXERA_GZIPD)

#define AXERA_GZIPD_SIZE        0x100

#define GZIPD_CTRL_CFG_REG      0x04
#define GZIPD_WDMA_CFG0_REG     0x08
#define GZIPD_WDMA_CFG1_REG     0x0c
#define GZIPD_RDMA_CFG0_REG     0x10
#define GZIPD_RDMA_CFG1_REG     0x14
#define GZIPD_RDMA_CFG2_REG     0x18
#define GZIPD_INTR_CLR_REG      0x1c
#define GZIPD_STATUS0_REG       0x20
#define GZIPD_STATUS1_REG       0x24

#define GZIP_COMPLETE_INTR      BIT(0)

#define LIT_CAP         22
#define LIT_DATABIT     9
#define LIT_ALPHA       512
#define LIT_ENTRY_W     32
#define LIT_N_ENTRIES   22
#define LIT_REGION_LEN  664

#define DIST_CAP        15
#define DIST_DATABIT    5
#define DIST_ALPHA      24
#define DIST_ENTRY_W    20
#define DIST_N_ENTRIES  16
#define DIST_REGION_LEN 55

#define TABLE_REGION_LEN (LIT_REGION_LEN + DIST_REGION_LEN + 1)
#define BLOCK_HDR_LEN   16

static const uint32_t dist_base[DIST_ALPHA] = {
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193,
    257, 385, 513, 769, 1025, 1537, 2049, 3073,
};

struct AxeraGzipdState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[AXERA_GZIPD_SIZE / 4];

    uint8_t *accum;
    size_t accum_len;
    size_t accum_cap;
    uint32_t osize;
    uint32_t blk_num;
};

/* MSB-first N-bit field starting at bit offset bit_off within buf. */
static uint32_t extract_bits(const uint8_t *buf, size_t bit_off, int n)
{
    uint32_t val = 0;
    int i;

    for (i = 0; i < n; i++) {
        size_t bo = bit_off + i;
        uint8_t byte = buf[bo / 8];
        int bit = (byte >> (7 - (bo % 8))) & 1;
        val = (val << 1) | bit;
    }
    return val;
}

typedef struct {
    const uint8_t *data;
    size_t pos; /* bit position */
    size_t nbits;
} BitReader;

static int br_read_bit(BitReader *br)
{
    uint8_t byte;
    int bit;

    if (br->pos >= br->nbits) {
        return 0;
    }
    byte = br->data[br->pos >> 3];
    bit = (byte >> (7 - (br->pos & 7))) & 1;
    br->pos++;
    return bit;
}

static uint32_t br_read_bits(BitReader *br, int n)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < n; i++) {
        v = (v << 1) | br_read_bit(br);
    }
    return v;
}

typedef struct {
    uint32_t firstcode[LIT_CAP > DIST_CAP ? LIT_CAP : DIST_CAP];
    uint32_t valptr[LIT_CAP > DIST_CAP ? LIT_CAP : DIST_CAP];
    uint16_t table1[LIT_ALPHA > DIST_ALPHA ? LIT_ALPHA : DIST_ALPHA];
} HuffTable;

static void parse_tree(const uint8_t *region, int databit, int alpha,
                       int entry_w, int n_entries, HuffTable *t)
{
    int i;
    size_t t1_start_bits = (size_t)n_entries * entry_w;

    for (i = 0; i < n_entries; i++) {
        uint32_t val = extract_bits(region, (size_t)i * entry_w, entry_w);

        t->firstcode[i] = val >> databit;
        t->valptr[i] = val & ((1u << databit) - 1);
    }
    for (i = 0; i < alpha; i++) {
        t->table1[i] = extract_bits(region, t1_start_bits + (size_t)i * databit,
                                    databit);
    }
}

static int decode_symbol(BitReader *br, const HuffTable *t, int cap, int alpha)
{
    uint32_t code = 0;
    int i;

    for (i = 0; i < cap; i++) {
        code = (code << 1) | br_read_bit(br);

        uint32_t count_this;
        if (i + 1 < cap) {
            count_this = ((t->firstcode[i + 1] + 1) / 2) - t->firstcode[i];
        } else {
            count_this = alpha - t->valptr[i];
        }
        uint32_t offset = code - t->firstcode[i];

        if (code >= t->firstcode[i] && offset < count_this) {
            return t->table1[t->valptr[i] + offset];
        }
    }
    return -1;
}

/*
 * Checks whether blk_num blocks' worth of sub-headers/tables/data fit
 * within the first avail_len bytes of buf, without doing any real
 * Huffman decode work -- just enough to know a decode attempt won't
 * run off the end of what has arrived so far. Returns true and sets
 * *total_len to the exact byte length of all blk_num blocks together
 * if they fit; false (don't touch *total_len) if more data is needed.
 */
static bool blocks_fit(const uint8_t *buf, size_t avail_len, uint32_t blk_num,
                       size_t *total_len)
{
    size_t addr = 0;
    uint32_t blk;

    for (blk = 0; blk < blk_num; blk++) {
        if (addr + BLOCK_HDR_LEN + TABLE_REGION_LEN > avail_len) {
            return false;
        }
        uint32_t chead0 = ldl_le_p(buf + addr);
        uint32_t data_len = chead0 + 1;
        uint32_t block_size = BLOCK_HDR_LEN + TABLE_REGION_LEN + data_len;

        if (addr + block_size > avail_len) {
            return false;
        }
        addr += block_size;
    }
    *total_len = addr;
    return true;
}

/*
 * Decodes blk_num blocks of block-data (no leading file header -- see
 * this file's own top comment for why) starting at data[0], into
 * *out_len freshly g_malloc'd bytes. Returns NULL on any structural
 * failure.
 */
static uint8_t *decode_axgzip(const uint8_t *data, uint32_t blk_num,
                              uint32_t osize, size_t *out_len)
{
    uint8_t *out;
    size_t out_cap, out_used;
    size_t addr;
    uint32_t blk;

    out_cap = osize + 1024;
    out = g_malloc(out_cap);
    out_used = 0;

    addr = 0;
    for (blk = 0; blk < blk_num; blk++) {
        size_t block_start = addr;
        uint32_t chead0 = ldl_le_p(data + addr);
        uint32_t data_len = chead0 + 1;
        const uint8_t *table_region = data + addr + BLOCK_HDR_LEN;
        const uint8_t *lit_region = table_region;
        const uint8_t *dist_region = table_region + LIT_REGION_LEN;

        HuffTable lit_t, dist_t;
        parse_tree(lit_region, LIT_DATABIT, LIT_ALPHA, LIT_ENTRY_W,
                  LIT_N_ENTRIES, &lit_t);
        parse_tree(dist_region, DIST_DATABIT, DIST_ALPHA, DIST_ENTRY_W,
                  DIST_N_ENTRIES, &dist_t);

        uint32_t entry15_raw = extract_bits(dist_region, 300, 20);
        uint32_t ntok = (entry15_raw + 1) * 4096;

        const uint8_t *data_bytes = data + addr + BLOCK_HDR_LEN
                                    + TABLE_REGION_LEN;
        BitReader br = { .data = data_bytes, .pos = 0,
                         .nbits = (size_t)data_len * 8 };
        bool is_final = (blk == blk_num - 1);
        uint32_t token_i = 0;

        while (is_final ? (out_used < osize) : (token_i < ntok)) {
            int sym = decode_symbol(&br, &lit_t, LIT_CAP, LIT_ALPHA);
            if (sym < 0) {
                goto fail;
            }
            token_i++;

            if (sym < 256) {
                if (out_used >= out_cap) {
                    out_cap *= 2;
                    out = g_realloc(out, out_cap);
                }
                out[out_used++] = sym;
            } else {
                uint32_t length = (sym - 256) + 3;
                int dcode = decode_symbol(&br, &dist_t, DIST_CAP, DIST_ALPHA);
                if (dcode < 0 || dcode >= DIST_ALPHA) {
                    goto fail;
                }
                uint32_t extra_bits = (dcode < 4) ? 0 : (dcode - 2) / 2;
                uint32_t extra = extra_bits ? br_read_bits(&br, extra_bits) : 0;
                uint32_t dist = dist_base[dcode] + extra;

                if (dist == 0 || dist > out_used) {
                    goto fail;
                }
                while (out_cap < out_used + length) {
                    out_cap *= 2;
                    out = g_realloc(out, out_cap);
                }
                uint32_t k;
                for (k = 0; k < length; k++) {
                    out[out_used] = out[out_used - dist];
                    out_used++;
                }
            }
        }

        uint32_t block_size = BLOCK_HDR_LEN + TABLE_REGION_LEN + data_len;
        addr = block_start + block_size;
    }

    if (out_used < osize) {
        goto fail;
    }
    *out_len = osize;
    return out;

fail:
    g_free(out);
    return NULL;
}

static void axera_gzipd_reset_accum(AxeraGzipdState *s)
{
    g_free(s->accum);
    s->accum = NULL;
    s->accum_len = 0;
    s->accum_cap = 0;
    s->osize = 0;
    s->blk_num = 0;
}

static void axera_gzipd_append(AxeraGzipdState *s, hwaddr addr, uint32_t size)
{
    if (s->accum_len + size > s->accum_cap) {
        s->accum_cap = (s->accum_len + size) * 2 + 4096;
        s->accum = g_realloc(s->accum, s->accum_cap);
    }
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       s->accum + s->accum_len, size);
    s->accum_len += size;

    if (s->blk_num == 0) {
        return; /* RDMA_CFG0 (which carries blk_num) not written yet */
    }

    size_t total_len;
    if (blocks_fit(s->accum, s->accum_len, s->blk_num, &total_len)) {
        size_t out_len = 0;
        uint8_t *out = decode_axgzip(s->accum, s->blk_num, s->osize, &out_len);

        if (out) {
            hwaddr dest = ((uint64_t)s->mem[GZIPD_WDMA_CFG0_REG / 4]) << 3;

            address_space_write(&address_space_memory, dest,
                               MEMTXATTRS_UNSPECIFIED, out, out_len);
            g_free(out);
            s->mem[GZIPD_STATUS1_REG / 4] |= GZIP_COMPLETE_INTR;
        }
        axera_gzipd_reset_accum(s);
    }
}

static void axera_gzipd_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    AxeraGzipdState *s = opaque;

    switch (offset) {
    case GZIPD_WDMA_CFG1_REG:
        /*
         * A fresh gzipd_dev_cfg() call: new image, drop any leftovers.
         * This is genuinely the first register the real driver writes
         * for a new image (osize, then WDMA_CFG0/RDMA_CFG0) -- resetting
         * on WDMA_CFG0 instead (written second) wiped the osize this
         * same call had just set.
         */
        axera_gzipd_reset_accum(s);
        s->osize = value + 1;
        s->mem[offset / 4] = value;
        return;
    case GZIPD_WDMA_CFG0_REG:
        s->mem[offset / 4] = value;
        return;
    case GZIPD_RDMA_CFG0_REG:
        s->blk_num = (value >> 16) & 0x7fff;
        s->mem[offset / 4] = value;
        return;
    case GZIPD_RDMA_CFG2_REG: {
        hwaddr tile_addr = ((uint64_t)s->mem[GZIPD_RDMA_CFG1_REG / 4]) << 3;
        uint32_t tile_size = value + 1;

        s->mem[offset / 4] = value;
        axera_gzipd_append(s, tile_addr, tile_size);
        return;
    }
    case GZIPD_INTR_CLR_REG:
        s->mem[GZIPD_STATUS1_REG / 4] &= ~value;
        return;
    case GZIPD_STATUS0_REG:
    case GZIPD_STATUS1_REG:
        return;
    default:
        if (offset < AXERA_GZIPD_SIZE) {
            s->mem[offset / 4] = value;
        }
        return;
    }
}

static uint64_t axera_gzipd_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraGzipdState *s = opaque;

    if (offset < AXERA_GZIPD_SIZE) {
        return s->mem[offset / 4];
    }
    return 0;
}

static const MemoryRegionOps axera_gzipd_ops = {
    .read = axera_gzipd_read,
    .write = axera_gzipd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_gzipd_realize(DeviceState *dev, Error **errp)
{
    AxeraGzipdState *s = AXERA_GZIPD(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_gzipd_ops, s,
                          TYPE_AXERA_GZIPD, AXERA_GZIPD_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void axera_gzipd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_gzipd_realize;
}

static const TypeInfo axera_gzipd_info = {
    .name = TYPE_AXERA_GZIPD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraGzipdState),
    .class_init = axera_gzipd_class_init,
};

static void axera_gzipd_register_types(void)
{
    type_register_static(&axera_gzipd_info);
}

type_init(axera_gzipd_register_types)
