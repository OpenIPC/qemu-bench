/*
 * Goke GK710xS DMA controller (Ambarella-style), at the offsets the
 * vendor drivers use, which the S HAL passes through
 * (mach-gk710xs/include/mach/dma.h, mach-gk710xs/dma.c):
 *
 *   0x304           INT       bit n: channel n has status to report
 *   0x308 + 4n      DA(n)     descriptor address (descriptor mode)
 *   0x31C + 16n     CTR(n)    31 EN, 30 D (descriptor mode), 29 WM, 28 RM,
 *                             27 NI, 26:24 block, 23:22 transfer size,
 *                             21:0 byte count (plain mode)
 *   0x320 + 16n     SRC(n)    plain-mode source
 *   0x324 + 16n     DST(n)    plain-mode destination
 *   0x328 + 16n     STA(n)    31 DM ... 22 DN (done), 21:0 count
 *
 * A descriptor (gk_dma_req_t) is six words: src, dst, next, rpt (where the
 * controller stores the descriptor's status), xfr_count and attr
 * (24 EOC, 23 WM, 22 RM, 21 NI, 20:19 transfer size, 18:16 block,
 * 2 ID, 1 IE, 0 ST).  WM / RM: the destination / source is memory and
 * advances; otherwise it is a peripheral FIFO read or written in place.
 *
 * Setting EN runs the whole transfer at once: in descriptor mode each
 * descriptor of the chain from DA, DA following each one's next until one
 * has EOC, as the vendor U-Boot of the GK7102C cameras checks at start
 * (it runs one descriptor from the I2S FIFO and expects DA to have moved
 * on to that descriptor's next).  EN then drops, STA reports done and the
 * channel's INT bit raises the interrupt until STA is written.
 *
 * Counts are 22 bits, as the CTR field and the kernel's limit on
 * xfr_count (0x3FFFFF, mach-gk710xs dma.c) have them; higher bits of a
 * descriptor's count are ignored.  A count that is not a whole number of
 * transfer units has its tail moved a byte at a time, so the reported
 * count is what was moved.
 *
 * A descriptor that cannot be fetched, or a source or destination that
 * cannot be accessed, stops the channel there: DA is left at the failing
 * descriptor, DD / DN are not set, and STA gets BE (bit 25).  The vendor
 * headers name the STA bits but not their meaning; BE is taken as the
 * bus error.  The INT bit then reports that status as for any other.
 *
 * The other registers (the FIO channel at 0x380 up) are plain storage.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_GK_DMA "gk-dma"
OBJECT_DECLARE_SIMPLE_TYPE(GkDmaState, GK_DMA)

#define GK_DMA_SIZE     0x1000
#define GK_DMA_CHANS    4

#define DMA_INT         0x304
#define DMA_DA(n)       (0x308 + 4 * (n))
#define DMA_CTR(n)      (0x31C + 16 * (n))
#define DMA_SRC(n)      (0x320 + 16 * (n))
#define DMA_DST(n)      (0x324 + 16 * (n))
#define DMA_STA(n)      (0x328 + 16 * (n))

#define CTR_EN          (1u << 31)
#define CTR_D           (1u << 30)
#define CTR_WM          (1u << 29)
#define CTR_RM          (1u << 28)
#define CTR_TS(c)       (((c) >> 22) & 3)
#define CTR_COUNT(c)    ((c) & 0x3FFFFF)

#define DESC_EOC        (1u << 24)
#define DESC_WM         (1u << 23)
#define DESC_RM         (1u << 22)
#define DESC_TS(a)      (((a) >> 19) & 3)

#define STA_DD          (1u << 28)      /* descriptor done */
#define STA_BE          (1u << 25)      /* bus error: see the top */
#define STA_DN          (1u << 22)      /* transfer done */
#define DMA_COUNT_MASK  0x3FFFFF

/* A chain longer than this is a loop: stop and report a done. */
#define GK_DMA_MAX_DESC 4096

struct GkDmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[GK_DMA_SIZE / 4];
};

static uint32_t *gk_dma_reg(GkDmaState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void gk_dma_update(GkDmaState *s)
{
    uint32_t pending = 0;
    int n;

    for (n = 0; n < GK_DMA_CHANS; n++) {
        if (*gk_dma_reg(s, DMA_STA(n))) {
            pending |= 1u << n;
        }
    }
    *gk_dma_reg(s, DMA_INT) = pending;
    qemu_set_irq(s->irq, pending != 0);
}

/*
 * Move count (at most DMA_COUNT_MASK) bytes in units of 1 << ts, the tail
 * that is not a whole unit a byte at a time; a side that is not memory is
 * a FIFO, read or written at the same address every time.  False if an
 * access failed: the transfer stops there.
 */
static bool gk_dma_move(uint32_t src, uint32_t dst, uint32_t count,
                        unsigned ts, bool src_inc, bool dst_inc)
{
    uint8_t buf[8];

    count &= DMA_COUNT_MASK;
    while (count) {
        const unsigned unit = count >= (1u << ts) ? 1u << ts : 1;

        if (address_space_read(&address_space_memory, src,
                               MEMTXATTRS_UNSPECIFIED, buf, unit) != MEMTX_OK ||
            address_space_write(&address_space_memory, dst,
                                MEMTXATTRS_UNSPECIFIED, buf, unit) != MEMTX_OK) {
            return false;
        }
        count -= unit;
        if (src_inc) {
            src += unit;
        }
        if (dst_inc) {
            dst += unit;
        }
    }
    return true;
}

static void gk_dma_run(GkDmaState *s, int n)
{
    uint32_t ctr = *gk_dma_reg(s, DMA_CTR(n));
    uint32_t sta;

    if (ctr & CTR_D) {
        uint32_t da = *gk_dma_reg(s, DMA_DA(n));
        uint32_t d[6];
        int i;

        sta = STA_DD | STA_DN;
        for (i = 0; i < GK_DMA_MAX_DESC; i++) {
            if (dma_memory_read(&address_space_memory, da, d, sizeof(d),
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR, "gk-dma: channel %d: no "
                              "descriptor at 0x%08x\n", n, da);
                sta = STA_BE;
                break;
            }
            for (int w = 0; w < 6; w++) {
                d[w] = le32_to_cpu(d[w]);
            }
            if (!gk_dma_move(d[0], d[1], d[4], DESC_TS(d[5]),
                             d[5] & DESC_RM, d[5] & DESC_WM)) {
                qemu_log_mask(LOG_GUEST_ERROR, "gk-dma: channel %d: "
                              "transfer of descriptor 0x%08x failed\n", n, da);
                sta = STA_BE;
                break;
            }
            if (d[3]) {
                uint32_t rpt = cpu_to_le32(STA_DD | STA_DN |
                                           (d[4] & DMA_COUNT_MASK));

                address_space_write(&address_space_memory, d[3],
                                    MEMTXATTRS_UNSPECIFIED, &rpt, 4);
            }
            da = d[2];
            if (d[5] & DESC_EOC) {
                break;
            }
        }
        *gk_dma_reg(s, DMA_DA(n)) = da;
    } else if (gk_dma_move(*gk_dma_reg(s, DMA_SRC(n)),
                           *gk_dma_reg(s, DMA_DST(n)), CTR_COUNT(ctr),
                           CTR_TS(ctr), ctr & CTR_RM, ctr & CTR_WM)) {
        sta = STA_DN | CTR_COUNT(ctr);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "gk-dma: channel %d: transfer "
                      "failed\n", n);
        sta = STA_BE;
    }
    *gk_dma_reg(s, DMA_CTR(n)) = ctr & ~CTR_EN;
    *gk_dma_reg(s, DMA_STA(n)) = sta;
    gk_dma_update(s);
}

static uint64_t gk_dma_read(void *opaque, hwaddr off, unsigned size)
{
    GkDmaState *s = opaque;

    return *gk_dma_reg(s, off & ~3);
}

static void gk_dma_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GkDmaState *s = opaque;
    int n;

    off &= ~3;
    if (off == DMA_INT) {
        return;                         /* read only */
    }
    *gk_dma_reg(s, off) = val;
    for (n = 0; n < GK_DMA_CHANS; n++) {
        if (off == DMA_CTR(n) && (val & CTR_EN)) {
            gk_dma_run(s, n);
            return;
        }
        if (off == DMA_STA(n)) {
            gk_dma_update(s);
            return;
        }
    }
}

static const MemoryRegionOps gk_dma_ops = {
    .read = gk_dma_read,
    .write = gk_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void gk_dma_reset(DeviceState *dev)
{
    GkDmaState *s = GK_DMA(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static const VMStateDescription vmstate_gk_dma = {
    .name = TYPE_GK_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GkDmaState, GK_DMA_SIZE / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void gk_dma_init(Object *obj)
{
    GkDmaState *s = GK_DMA(obj);

    memory_region_init_io(&s->iomem, obj, &gk_dma_ops, s, TYPE_GK_DMA,
                          GK_DMA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void gk_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, gk_dma_reset);
    dc->vmsd = &vmstate_gk_dma;
}

static const TypeInfo gk_dma_info = {
    .name          = TYPE_GK_DMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GkDmaState),
    .instance_init = gk_dma_init,
    .class_init    = gk_dma_class_init,
};

static void gk_dma_register_types(void)
{
    type_register_static(&gk_dma_info);
}

type_init(gk_dma_register_types)
