/*
 * Novatek GZIP decoder engine (NS02501 at 0xF0100000), as the loader's
 * GZ_Uncompress() drives it: the loader keeps its main part deflated and
 * unpacks it with this engine before it runs.
 *
 *   0x00 CTRL    bit 0 enable, bit 1 start (a run completes at once)
 *   0x04 MODE    4 for the loader's streams (storage)
 *   0x10 INTEN   0x14 STS  bit 0 done; write 1 to clear
 *   0x20 SRC     0x28 DST  0x30 LEN  the input's address and size
 *   0x38 OUTSZ   the bytes written
 *
 * The input is a gzip, zlib or raw deflate stream (tried in that order);
 * the output goes to DST, at most "max-out" bytes.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include <zlib.h>

#define TYPE_NVT_GZIP "nvt-gzip"
OBJECT_DECLARE_SIMPLE_TYPE(NvtGzipState, NVT_GZIP)

#define GZ_CTRL         0x00
#define GZ_INTEN        0x10
#define GZ_STS          0x14
#define GZ_SRC          0x20
#define GZ_DST          0x28
#define GZ_LEN          0x30
#define GZ_OUTSZ        0x38
#define GZ_NREGS        (0x40 / 4)
#define GZ_CTRL_START   (1u << 1)
#define GZ_STS_DONE     (1u << 0)
#define GZ_MAX_IN       (64 * MiB)

struct NvtGzipState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[GZ_NREGS];
    uint32_t max_out;
};

static void nvt_gzip_update(NvtGzipState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[GZ_STS / 4] & s->regs[GZ_INTEN / 4]));
}

/* Inflate with the given window bits; the bytes made, or -1 */
static long nvt_gzip_inflate(const uint8_t *in, uint32_t inlen, uint8_t *out,
                             uint32_t outlen, int wbits)
{
    z_stream zs = { 0 };
    long n;
    int r;

    if (inflateInit2(&zs, wbits) != Z_OK) {
        return -1;
    }
    zs.next_in = (Bytef *)in;
    zs.avail_in = inlen;
    zs.next_out = out;
    zs.avail_out = outlen;
    r = inflate(&zs, Z_FINISH);
    n = (r == Z_STREAM_END || (r == Z_BUF_ERROR && zs.total_out)) ?
        (long)zs.total_out : -1;
    inflateEnd(&zs);
    return n;
}

static void nvt_gzip_run(NvtGzipState *s)
{
    uint32_t src = s->regs[GZ_SRC / 4], dst = s->regs[GZ_DST / 4];
    uint32_t len = s->regs[GZ_LEN / 4];
    g_autofree uint8_t *in = NULL, *out = NULL;
    long n = -1;

    if (len && len <= GZ_MAX_IN) {
        in = g_malloc(len);
        out = g_malloc(s->max_out);
        if (!dma_memory_read(&address_space_memory, src, in, len,
                             MEMTXATTRS_UNSPECIFIED)) {
            n = nvt_gzip_inflate(in, len, out, s->max_out, 16 + MAX_WBITS);
            if (n < 0) {
                n = nvt_gzip_inflate(in, len, out, s->max_out, MAX_WBITS);
            }
            if (n < 0) {
                n = nvt_gzip_inflate(in, len, out, s->max_out, -MAX_WBITS);
            }
        }
    }
    if (n < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-gzip: 0x%x bytes at 0x%x do not "
                      "inflate\n", len, src);
        n = 0;
    } else {
        dma_memory_write(&address_space_memory, dst, out, n,
                         MEMTXATTRS_UNSPECIFIED);
    }
    s->regs[GZ_OUTSZ / 4] = n;
    s->regs[GZ_STS / 4] |= GZ_STS_DONE;
}

static uint64_t nvt_gzip_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtGzipState *s = opaque;

    return offset < GZ_NREGS * 4 ? s->regs[offset / 4] : 0;
}

static void nvt_gzip_write(void *opaque, hwaddr offset, uint64_t val,
                           unsigned size)
{
    NvtGzipState *s = opaque;

    if (offset >= GZ_NREGS * 4 || offset == GZ_OUTSZ) {
        return;
    }
    if (offset == GZ_STS) {
        s->regs[GZ_STS / 4] &= ~val;
    } else {
        s->regs[offset / 4] = val;
        if (offset == GZ_CTRL && (val & GZ_CTRL_START)) {
            s->regs[GZ_CTRL / 4] &= ~GZ_CTRL_START;
            nvt_gzip_run(s);
        }
    }
    nvt_gzip_update(s);
}

static const MemoryRegionOps nvt_gzip_ops = {
    .read = nvt_gzip_read,
    .write = nvt_gzip_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_gzip_reset(DeviceState *dev)
{
    NvtGzipState *s = NVT_GZIP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void nvt_gzip_init(Object *obj)
{
    NvtGzipState *s = NVT_GZIP(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_gzip_ops, s, TYPE_NVT_GZIP,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property nvt_gzip_properties[] = {
    DEFINE_PROP_UINT32("max-out", NvtGzipState, max_out, 16 * MiB),
};

static const VMStateDescription vmstate_nvt_gzip = {
    .name = TYPE_NVT_GZIP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtGzipState, GZ_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_gzip_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_gzip_reset);
    device_class_set_props(dc, nvt_gzip_properties);
    dc->vmsd = &vmstate_nvt_gzip;
}

static const TypeInfo nvt_gzip_info = {
    .name          = TYPE_NVT_GZIP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtGzipState),
    .instance_init = nvt_gzip_init,
    .class_init    = nvt_gzip_class_init,
};

static void nvt_gzip_register_types(void)
{
    type_register_static(&nvt_gzip_info);
}

type_init(nvt_gzip_register_types)
