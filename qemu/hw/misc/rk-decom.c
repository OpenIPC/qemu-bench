/*
 * Rockchip hardware decompressor ("decom", RV1106 / RV1126 / RK3588 ...).
 *
 * Thunder-boot SPLs and U-Boots unpack gzip'd FIT images with it
 * (CONFIG_SPL_MISC_DECOMPRESS; "decompress error, ret=-110" without it).
 * The driver is U-Boot's drivers/misc/rockchip_decompress.c:
 *
 *   0x00 CTRL   mode: bit 0 deflate, + bit 4 gzip / bit 5 zlib; 0 = LZ4
 *   0x04 ENR    1 starts a job
 *   0x08 RADDR  0x0c WADDR   source / destination in DRAM
 *   0x24 STAT   bit 0 COMPLETE
 *   0x28 ISR    bit 0 done (write 1 to clear); 0x2c IEN
 *   0x30 AXI_STAT  bit 4 idle
 *   0x34 / 0x38 TSIZEL / H  bytes written
 *   0x50 / 0x54 LMTSL / H   output limit
 *
 * Jobs take time, as on silicon: the input is decoded at the ENR write,
 * but the output lands in DRAM and COMPLETE / the interrupt come only
 * after size / rate (property "rate", bytes per microsecond, default 200)
 * of virtual time.  Thunder-boot SPLs start the kernel's decompression,
 * go on loading the rest, and only then poll STAT; the polling path is
 * the one silicon takes, so it is the one modelled.
 *
 * Deflate goes through zlib, which QEMU always links.  LZ4 (CTRL mode 0)
 * is the LZ4 frame format (magic 0x184D2204), decoded here, with the
 * checks CTRL asks for: bit 1 the header checksum, bit 2 the block
 * checksums, bit 3 the content checksum (all xxHash32; U-Boot sets all
 * three).  A bad frame ends with ISR bit 1 (header error), a bad
 * checksum with bit 13 / 14 / 15 (header / block / content), a bad
 * sequence with bit 3 (SQME) -- the drivers report any of them.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qemu/bitops.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qom/object.h"
#include <zlib.h>

#define TYPE_RK_DECOM "rk-decom"
OBJECT_DECLARE_SIMPLE_TYPE(RkDecomState, RK_DECOM)

#define R_CTRL      0x00
#define R_ENR       0x04
#define R_RADDR     0x08
#define R_WADDR     0x0c
#define R_STAT      0x24
#define R_ISR       0x28
#define R_IEN       0x2c
#define R_AXI_STAT  0x30
#define R_TSIZEL    0x34
#define R_TSIZEH    0x38
#define R_LMTSL     0x50
#define R_LMTSH     0x54
#define REGS_SIZE   0x100

#define ISR_DONE    (1u << 0)
#define ISR_HDERR   (1u << 1)
#define ISR_SQMERR  (1u << 3)
#define ISR_HCCERR  (1u << 13)
#define ISR_BCCERR  (1u << 14)
#define ISR_CCCERR  (1u << 15)
#define CTRL_LZ4_HC (1u << 1)
#define CTRL_LZ4_BC (1u << 2)
#define CTRL_LZ4_CC (1u << 3)
#define LZ4_MAGIC   0x184D2204u
#define CHUNK       (64 * 1024)

struct RkDecomState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t rate;
    uint32_t regs[REGS_SIZE / 4];

    /* the job in flight: its output, written when the timer fires */
    uint8_t *out;
    uint64_t out_len;
    bool error;
    uint32_t err_isr;           /* the ISR bit for an error */
};

static void rk_decom_update_irq(RkDecomState *s)
{
    qemu_set_irq(s->irq, (s->regs[R_ISR / 4] & s->regs[R_IEN / 4]) != 0);
}

/* Inflate from RADDR into s->out, at most limit bytes; false on error. */
static bool rk_decom_inflate(RkDecomState *s, int window_bits, uint64_t limit)
{
    AddressSpace *as = &address_space_memory;
    hwaddr src = s->regs[R_RADDR / 4];
    g_autofree uint8_t *in = g_malloc(CHUNK);
    uint64_t cap = CHUNK;
    z_stream zs = { 0 };
    int ret = Z_OK;

    if (inflateInit2(&zs, window_bits) != Z_OK) {
        return false;
    }
    s->out = g_malloc(cap);
    s->out_len = 0;
    while (ret != Z_STREAM_END && s->out_len < limit) {
        if (!zs.avail_in) {
            dma_memory_read(as, src, in, CHUNK, MEMTXATTRS_UNSPECIFIED);
            src += CHUNK;
            zs.next_in = in;
            zs.avail_in = CHUNK;
        }
        if (s->out_len == cap) {
            cap *= 2;
            s->out = g_realloc(s->out, cap);
        }
        zs.next_out = s->out + s->out_len;
        zs.avail_out = MIN(cap - s->out_len, limit - s->out_len);
        {
            uint32_t room = zs.avail_out;

            ret = inflate(&zs, Z_NO_FLUSH);
            s->out_len += room - zs.avail_out;
        }
        if (ret != Z_OK && ret != Z_STREAM_END) {
            qemu_log_mask(LOG_GUEST_ERROR, "rk-decom: inflate: %d (%s) at "
                          "0x%" PRIx64 " bytes out\n", ret,
                          zs.msg ? zs.msg : "", s->out_len);
            inflateEnd(&zs);
            return false;
        }
    }
    inflateEnd(&zs);
    return true;
}

/* ---- LZ4 frames ---------------------------------------------------- */

#define XXH_P1 2654435761u
#define XXH_P2 2246822519u
#define XXH_P3 3266489917u
#define XXH_P4 668265263u
#define XXH_P5 374761393u

static uint32_t xxh_round(uint32_t acc, uint32_t in)
{
    return rol32(acc + in * XXH_P2, 13) * XXH_P1;
}

static uint32_t xxh32(const uint8_t *p, size_t len, uint32_t seed)
{
    const uint8_t *end = p + len;
    uint32_t h;

    if (len >= 16) {
        uint32_t v1 = seed + XXH_P1 + XXH_P2, v2 = seed + XXH_P2;
        uint32_t v3 = seed, v4 = seed - XXH_P1;

        do {
            v1 = xxh_round(v1, ldl_le_p(p));
            v2 = xxh_round(v2, ldl_le_p(p + 4));
            v3 = xxh_round(v3, ldl_le_p(p + 8));
            v4 = xxh_round(v4, ldl_le_p(p + 12));
            p += 16;
        } while (p + 16 <= end);
        h = rol32(v1, 1) + rol32(v2, 7) + rol32(v3, 12) + rol32(v4, 18);
    } else {
        h = seed + XXH_P5;
    }
    h += len;
    for (; p + 4 <= end; p += 4) {
        h = rol32(h + ldl_le_p(p) * XXH_P3, 17) * XXH_P4;
    }
    for (; p < end; p++) {
        h = rol32(h + *p * XXH_P5, 11) * XXH_P1;
    }
    h ^= h >> 15;
    h *= XXH_P2;
    h ^= h >> 13;
    h *= XXH_P3;
    h ^= h >> 16;
    return h;
}

/* Room for n more bytes of output, within limit. */
static bool rk_decom_room(RkDecomState *s, uint64_t *cap, uint64_t n,
                          uint64_t limit)
{
    if (s->out_len + n > limit) {
        return false;
    }
    while (s->out_len + n > *cap) {
        *cap *= 2;
        s->out = g_realloc(s->out, *cap);
    }
    return true;
}

/* One LZ4 block, appended to s->out; matches may reach earlier blocks. */
static bool rk_decom_lz4_block(RkDecomState *s, const uint8_t *ip,
                               uint32_t len, uint64_t *cap, uint64_t limit)
{
    const uint8_t *end = ip + len;

    while (ip < end) {
        uint8_t token = *ip++;
        uint64_t lit = token >> 4, ml, off;

        if (lit == 15) {
            uint8_t b;

            do {
                if (ip >= end) {
                    return false;
                }
                b = *ip++;
                lit += b;
            } while (b == 255);
        }
        if (lit > (uint64_t)(end - ip) || !rk_decom_room(s, cap, lit, limit)) {
            return false;
        }
        memcpy(s->out + s->out_len, ip, lit);
        s->out_len += lit;
        ip += lit;
        if (ip == end) {
            return true;            /* the last sequence has no match */
        }
        if (end - ip < 2) {
            return false;
        }
        off = lduw_le_p(ip);
        ip += 2;
        ml = token & 15;
        if (ml == 15) {
            uint8_t b;

            do {
                if (ip >= end) {
                    return false;
                }
                b = *ip++;
                ml += b;
            } while (b == 255);
        }
        ml += 4;
        if (!off || off > s->out_len || !rk_decom_room(s, cap, ml, limit)) {
            return false;
        }
        while (ml--) {              /* byte by byte: matches overlap */
            s->out[s->out_len] = s->out[s->out_len - off];
            s->out_len++;
        }
    }
    return true;
}

/* An LZ4 frame from RADDR into s->out; false with s->err_isr set. */
static bool rk_decom_lz4(RkDecomState *s, uint32_t ctrl, uint64_t limit)
{
    AddressSpace *as = &address_space_memory;
    hwaddr src = s->regs[R_RADDR / 4];
    uint8_t hdr[19];
    uint64_t cap = CHUNK;
    int hlen = 7;
    uint8_t flg;

    s->err_isr = ISR_HDERR;
    dma_memory_read(as, src, hdr, sizeof(hdr), MEMTXATTRS_UNSPECIFIED);
    flg = hdr[4];
    if (ldl_le_p(hdr) != LZ4_MAGIC || (flg >> 6) != 1) {
        qemu_log_mask(LOG_GUEST_ERROR, "rk-decom: not an LZ4 frame\n");
        return false;
    }
    hlen += (flg & 0x08) ? 8 : 0;           /* content size */
    hlen += (flg & 0x01) ? 4 : 0;           /* dictionary ID */
    if ((ctrl & CTRL_LZ4_HC) &&
        ((xxh32(hdr + 4, hlen - 5, 0) >> 8) & 0xff) != hdr[hlen - 1]) {
        s->err_isr = ISR_HCCERR;
        return false;
    }
    src += hlen;
    s->out = g_malloc(cap);
    s->out_len = 0;
    for (;;) {
        uint32_t bsize, raw;
        g_autofree uint8_t *blk = NULL;

        dma_memory_read(as, src, &bsize, 4, MEMTXATTRS_UNSPECIFIED);
        bsize = le32_to_cpu(bsize);
        src += 4;
        if (!bsize) {
            break;                          /* end mark */
        }
        raw = bsize & 0x80000000u;
        bsize &= 0x7fffffffu;
        if (bsize > 4 * MiB) {
            return false;
        }
        blk = g_malloc(bsize);
        dma_memory_read(as, src, blk, bsize, MEMTXATTRS_UNSPECIFIED);
        src += bsize;
        if (flg & 0x10) {                   /* block checksum */
            uint32_t sum;

            dma_memory_read(as, src, &sum, 4, MEMTXATTRS_UNSPECIFIED);
            src += 4;
            if ((ctrl & CTRL_LZ4_BC) && xxh32(blk, bsize, 0) != le32_to_cpu(sum)) {
                s->err_isr = ISR_BCCERR;
                return false;
            }
        }
        if (raw) {
            if (!rk_decom_room(s, &cap, bsize, limit)) {
                s->err_isr = ISR_SQMERR;
                return false;
            }
            memcpy(s->out + s->out_len, blk, bsize);
            s->out_len += bsize;
        } else if (!rk_decom_lz4_block(s, blk, bsize, &cap, limit)) {
            s->err_isr = ISR_SQMERR;
            return false;
        }
    }
    if (flg & 0x04) {                       /* content checksum */
        uint32_t sum;

        dma_memory_read(as, src, &sum, 4, MEMTXATTRS_UNSPECIFIED);
        if ((ctrl & CTRL_LZ4_CC) &&
            xxh32(s->out, s->out_len, 0) != le32_to_cpu(sum)) {
            s->err_isr = ISR_CCCERR;
            return false;
        }
    }
    return true;
}

static void rk_decom_done(void *opaque)
{
    RkDecomState *s = opaque;

    if (s->out) {
        dma_memory_write(&address_space_memory, s->regs[R_WADDR / 4], s->out,
                         s->out_len, MEMTXATTRS_UNSPECIFIED);
    }
    s->regs[R_STAT / 4] |= 1;
    s->regs[R_ISR / 4] |= s->error ? s->err_isr : ISR_DONE;
    s->regs[R_TSIZEL / 4] = s->out_len;
    s->regs[R_TSIZEH / 4] = s->out_len >> 32;
    g_free(s->out);
    s->out = NULL;
    rk_decom_update_irq(s);
}

static void rk_decom_run(RkDecomState *s)
{
    uint32_t ctrl = s->regs[R_CTRL / 4];
    uint64_t limit = (uint64_t)s->regs[R_LMTSH / 4] << 32 |
                     s->regs[R_LMTSL / 4];

    g_free(s->out);
    s->out = NULL;
    s->out_len = 0;
    s->error = true;
    s->err_isr = ISR_HDERR;
    /*
     * No limit, or one beyond any DDR here: what a kernel or ramdisk
     * unpacks to fits in 256 MiB; a guest's bomb stops there.
     */
    if (!limit || limit > 256 * MiB) {
        limit = 256 * MiB;
    }
    if (ctrl & 1) {
        /* gzip: 16 + 15; zlib: 15; raw deflate: -15 */
        s->error = !rk_decom_inflate(s, (ctrl & 0x10) ? 31 :
                                        (ctrl & 0x20) ? 15 : -15, limit);
    } else {
        s->error = !rk_decom_lz4(s, ctrl, limit);
    }
    if (s->error) {
        s->out_len = 0;
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (s->out_len / MAX(s->rate, 1) + 1) * SCALE_US);
}

static uint64_t rk_decom_read(void *opaque, hwaddr off, unsigned size)
{
    RkDecomState *s = opaque;

    if (off == R_AXI_STAT) {
        return 1u << 4;         /* idle: jobs finish inside the ENR write */
    }
    return s->regs[off / 4];
}

static void rk_decom_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkDecomState *s = opaque;

    switch (off) {
    case R_ENR:
        s->regs[R_ENR / 4] = val & 1;
        if (val & 1) {
            s->regs[R_STAT / 4] &= ~1;
            rk_decom_run(s);
        } else if (s->out) {
            timer_del(s->timer);        /* stopped: the job is dropped */
            g_free(s->out);
            s->out = NULL;
        }
        return;
    case R_ISR:
        s->regs[R_ISR / 4] &= ~val;
        rk_decom_update_irq(s);
        return;
    case R_STAT:
    case R_TSIZEL:
    case R_TSIZEH:
        return;                 /* read-only */
    default:
        s->regs[off / 4] = val;
        if (off == R_IEN) {
            rk_decom_update_irq(s);
        }
    }
}

static const MemoryRegionOps rk_decom_ops = {
    .read = rk_decom_read,
    .write = rk_decom_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_decom_reset(DeviceState *dev)
{
    RkDecomState *s = RK_DECOM(dev);

    timer_del(s->timer);
    g_free(s->out);
    s->out = NULL;
    memset(s->regs, 0, sizeof(s->regs));
    rk_decom_update_irq(s);
}

static void rk_decom_realize(DeviceState *dev, Error **errp)
{
    RkDecomState *s = RK_DECOM(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &rk_decom_ops, s,
                          TYPE_RK_DECOM, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_decom_done, s);
}

static const Property rk_decom_props[] = {
    DEFINE_PROP_UINT32("rate", RkDecomState, rate, 200),
};

static void rk_decom_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_decom_realize;
    device_class_set_legacy_reset(dc, rk_decom_reset);
    device_class_set_props(dc, rk_decom_props);
    dc->desc = "Rockchip hardware decompressor (gzip / zlib)";
}

static const TypeInfo rk_decom_info = {
    .name = TYPE_RK_DECOM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkDecomState),
    .class_init = rk_decom_class_init,
};

static void rk_decom_register_types(void)
{
    type_register_static(&rk_decom_info);
}

type_init(rk_decom_register_types)
