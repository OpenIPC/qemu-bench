/*
 * HiSilicon Hardware GZIP Decompressor emulation.
 *
 * Found on Hi3516EV300 and similar V4/V3.5 SoCs at 0x11310000 (EV300)
 * or 0x11310000 (CV500-era).  Used by U-Boot's hw_compressed first-stage
 * loader to decompress the real U-Boot binary from SPI NOR flash into DDR.
 *
 * The hardware is a DMA engine that reads gzip-compressed data from a
 * source address, decompresses it, and writes the result to a destination
 * address.  U-Boot polls the interrupt status register for completion.
 *
 * Register map derived from u-boot-2016.11/lib/hw_dec/hw_decompress_v1.h.
 *
 * Copyright (c) 2026 OpenIPC.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "system/dma.h"
#include <zlib.h>

/* ── Register offsets ──────────────────────────────────────────────── */

/* EMAR interface config */
#define EAMR_WORK_EN        0x0100
#define EAMR_RID            0x0108
#define EAMR_ROSD           0x010C
#define EAMR_WID            0x0110
#define EAMR_WOSD           0x0114

/* Interrupt registers */
#define INT_STATUS          0x0124
#define INT_EN              0x0128
#define INT_CLEAR           0x0130

/* Decompress output (direct address mode) */
#define DPRS_DATA_RTN_BADDR  0x2020
#define DPRS_DATA_RTN_LEN    0x2024

/* Decompress output (page mode) */
#define DPRS_DATA_INFO_BADDR 0x2028
#define DPRS_DATA_INFO_LEN   0x202C

#define DPRS_DATA_CRC32     0x2030

/* Decompress input */
#define DPRS_DATA_SRC_BADDR 0x2040
#define DPRS_DATA_SRC_LEN   0x2044

/* Status / result */
#define BUF_INFO            0x2080
#define DPRS_RTN_INFO       0x2084
#define DPRS_RTN_LEN        0x2088
#define BUF_INFO_CLR        0x2090
#define RLT_INFO_CLR        0x2094

/* CRC */
#define CRC_CHECK_EN        0x4000

/* Register region size */
#define GZIP_REG_SIZE       0x8000

/*
 * Register layouts.  Goke GK7206 (xmorca) has the same engine with its
 * registers moved and the interrupt bits at [11:10] -- task at 11, block
 * at 10 -- rather than [1:0] (task at 0, block at 1): the SDK's
 * u-boot-2020.01 lotus/hw_dec/hw_decompress_v1.h.  Its offsets are
 * translated to the canonical ones above.
 */
#define GZIP_LAYOUT_V4      0
#define GZIP_LAYOUT_XMORCA  1
#define XMORCA_INT_TASK     (1u << 11)
#define XMORCA_INT_BLOCK    (1u << 10)

static hwaddr hisi_gzip_xmorca_to_canonical(hwaddr off)
{
    switch (off) {
    case 0x0110: return EAMR_RID;
    case 0x0114: return EAMR_ROSD;
    case 0x0118: return EAMR_WORK_EN;
    case 0x0128: return EAMR_WID;
    case 0x012C: return EAMR_WOSD;
    case 0x0130: return INT_EN;
    case 0x013C: return INT_STATUS;
    case 0x0148: return INT_CLEAR;
    case 0x2030: return DPRS_DATA_INFO_BADDR;
    case 0x2034: return DPRS_DATA_INFO_LEN;
    case 0x2038: return DPRS_DATA_RTN_BADDR;
    case 0x203C: return DPRS_DATA_RTN_LEN;
    case 0x2040: return DPRS_DATA_CRC32;
    case 0x2058: return DPRS_DATA_SRC_BADDR;
    case 0x205C: return DPRS_DATA_SRC_LEN;
    case 0x2090: return DPRS_RTN_LEN;
    case 0x2098: return BUF_INFO;
    case 0x209C: return DPRS_RTN_INFO;
    case 0x20A8: return BUF_INFO_CLR;
    case 0x20AC: return RLT_INFO_CLR;
    case 0x4018: return CRC_CHECK_EN;
    default:     return off | 0x10000;   /* no canonical twin: unknown */
    }
}

/* ── Device state ──────────────────────────────────────────────────── */

#define TYPE_HISI_GZIP "hisi-gzip"
OBJECT_DECLARE_SIMPLE_TYPE(HisiGzipState, HISI_GZIP)

struct HisiGzipState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* EMAR config */
    uint32_t work_en;
    uint32_t eamr_rid;
    uint32_t eamr_rosd;
    uint32_t eamr_wid;
    uint32_t eamr_wosd;

    /* Interrupt */
    uint32_t int_en;
    uint32_t int_status;

    /* Decompress parameters */
    uint32_t dst_baddr;       /* output base address (direct mode) */
    uint32_t dst_len;         /* output max length */
    uint32_t info_baddr;      /* page mode base addr */
    uint32_t info_len;        /* page mode length */
    uint32_t src_baddr;       /* input base address */
    uint32_t crc32_val;
    uint32_t crc_en;

    /* Result */
    uint32_t buf_info;
    uint32_t rtn_info;        /* bit 31 = avail, bits [7:0] = err */
    uint32_t rtn_len;         /* decompressed length */

    /*
     * Maximum sliding-window size the hardware decoder can use, in
     * bytes.  Real V4 / Goke V4 silicon has only 8 KiB of window
     * memory; deflate streams compressed with a larger WSIZE will
     * contain back-references > 8192 that cannot be reconstructed
     * and the chip aborts decompression.  The OpenIPC u-boot tree
     * ships a patched gzip-1.8 (WSIZE = 0x2000) for that reason.
     *
     * We feed this to zlib's inflateInit2() as windowBits — any
     * back-reference beyond 2^windowBits then makes inflate()
     * return Z_DATA_ERROR, matching real-silicon behaviour.
     *
     * Property "max-window-bytes" overrides at -global time.  Must
     * be a power of two in [256, 32768].  Defaults to 8192 since
     * every SoC currently using this device has the 8 KiB hardware.
     */
    uint32_t max_window_bytes;

    uint32_t layout;          /* GZIP_LAYOUT_V4 or GZIP_LAYOUT_XMORCA */
};

/* ── Decompress engine ─────────────────────────────────────────────── */

static void hisi_gzip_do_decompress(HisiGzipState *s, uint32_t src_len_reg)
{
    uint32_t src_len = src_len_reg & 0x0FFFFFFF;
    bool is_direct = (src_len_reg >> 31) & 1;  /* bit 31: 1=direct, 0=page */
    uint32_t dst_addr, dst_max;

    if (is_direct) {
        dst_addr = s->dst_baddr;
        dst_max  = s->dst_len;
    } else {
        dst_addr = s->info_baddr;
        dst_max  = s->info_len;
    }

    if (src_len == 0 || dst_max == 0) {
        s->rtn_info = (1u << 31) | 1;  /* error */
        s->rtn_len = 0;
        s->int_status |= 1;  /* task interrupt */
        return;
    }

    /* Read compressed data from guest memory */
    uint8_t *src_buf = g_malloc(src_len);
    dma_memory_read(&address_space_memory, s->src_baddr, src_buf, src_len,
                    MEMTXATTRS_UNSPECIFIED);

    /* Allocate output buffer */
    uint8_t *dst_buf = g_malloc(dst_max);

    /* Decompress using zlib (gzip format).
     *
     * windowBits = log2(max_window_bytes) | 32 → gzip+zlib auto-detect
     * with the hardware-imposed window cap.  zlib returns Z_DATA_ERROR
     * if the stream contains a back-reference beyond the cap (matches
     * real silicon's "Uncompress Fail!" path).  See openhisilicon#85.
     */
    uint32_t wb_bytes = s->max_window_bytes ? s->max_window_bytes : 8192;
    int wb_log2 = 0;
    while ((1u << wb_log2) < wb_bytes) wb_log2++;
    if (wb_log2 < 8) wb_log2 = 8;
    if (wb_log2 > 15) wb_log2 = 15;
    int windowBits = wb_log2 | 32;

    z_stream strm = {0};
    strm.next_in = src_buf;
    strm.avail_in = src_len;
    strm.next_out = dst_buf;
    strm.avail_out = 0;

    int ret = inflateInit2(&strm, windowBits);
    if (ret != Z_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-gzip: inflateInit2(%d) failed: %d\n",
                      windowBits, ret);
        s->rtn_info = (1u << 31) | 1;
        s->rtn_len = 0;
        s->int_status |= 1;
        g_free(src_buf);
        g_free(dst_buf);
        return;
    }

    /*
     * Chunk the inflate loop with Z_NO_FLUSH and a small avail_out.
     *
     * Calling inflate(Z_FINISH) with the entire output buffer
     * pre-allocated takes zlib's inflate_fast path, where the
     * back-distance check `if (op > whave)` is guarded by `if (dist >
     * op)` — and `op` is the current output-buffer position.  With a
     * full-size output buffer, every back-reference resolves with
     * `dist <= op` (intra-buffer copy) and the whave check never
     * fires.  Result: a stream encoded with a 32 KiB window
     * decompresses fine even though we asked for a 13-bit window cap.
     *
     * Forcing avail_out <= chunk_size makes zlib call updatewindow()
     * between chunks; the slow path then uses `whave` (capped at
     * `wsize = 1<<wbits`) for the back-distance check, matching
     * real-silicon behaviour where back-references beyond the
     * 8 KiB-of-window-memory boundary cannot be resolved.
     *
     * chunk_size = 1024 keeps the over-permissiveness slop (the
     * effective max valid distance is `whave + position_in_buffer
     * = wsize + chunk_size`) at 1 KiB above the cap — small enough
     * that streams with > 8 KiB back-distances reliably trip it,
     * while not blocking edge-case streams whose distances are
     * exactly at 8 KiB.  See openhisilicon#85.
     */
    const uint32_t chunk_size = 1024;
    size_t total_out = 0;
    const char *zmsg = NULL;
    do {
        uint32_t want = dst_max - total_out;
        if (want > chunk_size) want = chunk_size;
        if (want == 0) {
            /* output buffer full */
            break;
        }
        strm.next_out = dst_buf + total_out;
        strm.avail_out = want;

        ret = inflate(&strm, Z_NO_FLUSH);
        total_out += (want - strm.avail_out);

        if (ret == Z_STREAM_END) break;
        if (ret != Z_OK) {
            zmsg = strm.msg;
            break;
        }
        if (strm.avail_in == 0 && strm.avail_out > 0) {
            /* zlib wants more input but we have none — treat as error
             * the same way Z_FINISH would have surfaced it. */
            ret = Z_DATA_ERROR;
            zmsg = strm.msg ? strm.msg : "unexpected end of input";
            break;
        }
    } while (1);
    uint32_t out_len = (uint32_t)total_out;
    inflateEnd(&strm);

    if (ret != Z_STREAM_END && ret != Z_OK) {
        if (ret == Z_DATA_ERROR && zmsg &&
            strstr(zmsg, "distance too far back")) {
            /* Stream uses a back-reference beyond our window cap —
             * this is exactly the case the WSIZE=0x2000 patched gzip
             * was meant to avoid.  Surface it loudly so test authors
             * don't repeat the archaeology in #85. */
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-gzip: deflate back-distance exceeds %u-byte "
                          "window (zlib: %s) — stream needs WSIZE<=%u "
                          "(rebuild u-boot-z.bin with patched gzip; see "
                          "openhisilicon#85)\n",
                          1u << wb_log2, zmsg, 1u << wb_log2);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-gzip: inflate failed: %d (%s) "
                          "(produced %u bytes)\n",
                          ret, zmsg ? zmsg : "no msg", out_len);
        }
        /* Mark failure unconditionally for these errors — real silicon
         * raises Uncompress Fail!, no partial-output tolerance. */
        s->rtn_info = (1u << 31) | 1;
        s->rtn_len = 0;
        s->int_status |= 1;
        g_free(src_buf);
        g_free(dst_buf);
        return;
    }

    /* Write decompressed data to guest memory */
    dma_memory_write(&address_space_memory, dst_addr, dst_buf, out_len,
                     MEMTXATTRS_UNSPECIFIED);

    s->rtn_len = out_len;
    s->rtn_info = (1u << 31);  /* avail=1, err=0 => success */
    s->int_status |= 1;        /* task interrupt */

    qemu_log("hisi-gzip: decompressed %u -> %u bytes "
             "(src=0x%08x dst=0x%08x)\n",
             src_len, out_len, s->src_baddr, dst_addr);

    g_free(src_buf);
    g_free(dst_buf);
}

/* ── Register access ───────────────────────────────────────────────── */

static uint64_t hisi_gzip_read_canonical(HisiGzipState *s, hwaddr offset);

static uint64_t hisi_gzip_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiGzipState *s = HISI_GZIP(opaque);
    uint64_t val;

    if (s->layout != GZIP_LAYOUT_XMORCA) {
        return hisi_gzip_read_canonical(s, offset);
    }
    offset = hisi_gzip_xmorca_to_canonical(offset);
    val = hisi_gzip_read_canonical(s, offset);
    if (offset == INT_STATUS || offset == INT_EN) {
        val = ((val & 1) ? XMORCA_INT_TASK : 0) |
              ((val & 2) ? XMORCA_INT_BLOCK : 0);
    }
    return val;
}

static uint64_t hisi_gzip_read_canonical(HisiGzipState *s, hwaddr offset)
{
    switch (offset) {
    case EAMR_WORK_EN:      return s->work_en;
    case EAMR_RID:          return s->eamr_rid;
    case EAMR_ROSD:         return s->eamr_rosd;
    case EAMR_WID:          return s->eamr_wid;
    case EAMR_WOSD:         return s->eamr_wosd;

    case INT_STATUS:        return s->int_status;
    case INT_EN:            return s->int_en;

    case DPRS_DATA_RTN_BADDR: return s->dst_baddr;
    case DPRS_DATA_RTN_LEN:   return s->dst_len;
    case DPRS_DATA_INFO_BADDR: return s->info_baddr;
    case DPRS_DATA_INFO_LEN:   return s->info_len;
    case DPRS_DATA_CRC32:     return s->crc32_val;
    case DPRS_DATA_SRC_BADDR: return s->src_baddr;
    case DPRS_DATA_SRC_LEN:   return 0;

    case BUF_INFO:          return s->buf_info;
    case DPRS_RTN_INFO:     return s->rtn_info;
    case DPRS_RTN_LEN:      return s->rtn_len;

    case CRC_CHECK_EN:      return s->crc_en;

    case INT_CLEAR:         return 0;  /* write-only, reads as 0 */

    default:
        qemu_log_mask(LOG_UNIMP,
                      "hisi-gzip: read from unknown reg 0x%04x\n",
                      (unsigned)(offset & 0xffff));
        return 0;
    }
}

static void hisi_gzip_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    HisiGzipState *s = HISI_GZIP(opaque);

    if (s->layout == GZIP_LAYOUT_XMORCA) {
        offset = hisi_gzip_xmorca_to_canonical(offset);
        if (offset == INT_EN || offset == INT_CLEAR) {
            value = ((value & XMORCA_INT_TASK) ? 1 : 0) |
                    ((value & XMORCA_INT_BLOCK) ? 2 : 0);
        }
    }

    switch (offset) {
    case EAMR_WORK_EN:      s->work_en = value; break;
    case EAMR_RID:          s->eamr_rid = value; break;
    case EAMR_ROSD:         s->eamr_rosd = value; break;
    case EAMR_WID:          s->eamr_wid = value; break;
    case EAMR_WOSD:         s->eamr_wosd = value; break;

    case INT_EN:            s->int_en = value; break;
    case INT_CLEAR:
        if (value & 1) s->int_status &= ~1;  /* task_intrpt_clr */
        if (value & 2) s->int_status &= ~2;  /* block_intrpt_clr */
        break;

    case DPRS_DATA_RTN_BADDR: s->dst_baddr = value; break;
    case DPRS_DATA_RTN_LEN:   s->dst_len = value; break;
    case DPRS_DATA_INFO_BADDR: s->info_baddr = value; break;
    case DPRS_DATA_INFO_LEN:   s->info_len = value; break;
    case DPRS_DATA_CRC32:     s->crc32_val = value; break;
    case CRC_CHECK_EN:        s->crc_en = value; break;

    case DPRS_DATA_SRC_BADDR:
        s->src_baddr = value;
        break;

    case DPRS_DATA_SRC_LEN:
        /* Writing src_len triggers the decompress operation */
        hisi_gzip_do_decompress(s, value);
        break;

    case BUF_INFO_CLR:
        s->buf_info = 0;
        break;

    case RLT_INFO_CLR:
        s->rtn_info = 0;
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "hisi-gzip: write 0x%08x to unknown reg 0x%04x\n",
                      (unsigned)value, (unsigned)(offset & 0xffff));
        break;
    }
}

static const MemoryRegionOps hisi_gzip_ops = {
    .read  = hisi_gzip_read,
    .write = hisi_gzip_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── Device lifecycle ──────────────────────────────────────────────── */

static void hisi_gzip_init(Object *obj)
{
    HisiGzipState *s = HISI_GZIP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_gzip_ops, s,
                          "hisi-gzip", GZIP_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const Property hisi_gzip_properties[] = {
    /* HW sliding-window cap.  Real V4 / Goke V4 silicon has 8 KiB;
     * future SoCs with bigger HW windows can override at -global time
     * to anything in [256, 32768] (clamped to power-of-two log2 in the
     * decompress path).  See openhisilicon#85. */
    DEFINE_PROP_UINT32("max-window-bytes", HisiGzipState,
                       max_window_bytes, 8192),
    /* 0 = V4 / V5 register map, 1 = Goke GK7206 (xmorca) moved map. */
    DEFINE_PROP_UINT32("layout", HisiGzipState, layout, GZIP_LAYOUT_V4),
};

static void hisi_gzip_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, hisi_gzip_properties);
}

static const TypeInfo hisi_gzip_info = {
    .name          = TYPE_HISI_GZIP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiGzipState),
    .instance_init = hisi_gzip_init,
    .class_init    = hisi_gzip_class_init,
};

static void hisi_gzip_register(void)
{
    type_register_static(&hisi_gzip_info);
}

type_init(hisi_gzip_register)
