/*
 * Goke GK7101S / GK7102S register layouts over the GK7101 / GK7102 models.
 *
 * The S die (GK7101S, GK7102S and the later GK7102C) runs the same vendor
 * drivers as the GK710x, but its kernel reaches almost every block
 * directly: in mach-gk710xs/include/mach/gk710xs.h only the UART and SSI
 * accessors go through the HAL (*_HAL_MODE 1), the VIC, timers, watchdog,
 * IDC, GMAC, DMA, GPIO, RCT and the rest are plain loads and stores at the
 * driver's addresses and offsets.  So on the S die each of those blocks is
 * at 0x90000000 / 0xA0000000 in the drivers' own register layout.
 *
 * On the GK710x the same driver accesses all go through the HAL, which
 * moves the blocks and reorders their registers and bits: the GK710x
 * models implement what that HAL makes of them (include/hw/arm/goke.h).
 * Same drivers, same IP -- so an S block is a GK710x model with the HAL
 * applied in front of it.  gk-remap is that front: for a GK710x model
 * (its MMIO region 0, the "target") it decodes the drivers' layout and
 * forwards each register where the GK710x HAL would send it, with its bits
 * permuted the same way.  w[] is the model bit a driver bit is written to,
 * r[] the driver bit a model bit is read as (-1: not passed).
 *
 * The tables were taken from the GK710x HAL (20160913, board/goke/gk7101/
 * gk7101_hal_data.h), run under Unicorn for every register of each block
 * (qemu-boot/gk710x-halmap.py).  Registers not listed pass through; a
 * model register of 0xFF is one that HAL never reaches (RAZ/WI).  The S
 * register layouts are the GK710x driver ones -- mach-gk710xs and
 * mach-gk710x define the same offsets and bits for these blocks -- except
 * the timer control word, where the S timer.h has each timer's overflow-
 * interrupt and clock-select bits swapped (OFn = 0x40 >> n, CSLn =
 * 0x04 >> n); its row is composed with that swap.
 *
 * The UARTs need no remap -- the S kernel drives them through its HAL,
 * which makes the same standard 16550 of them as the GK710x HAL does --
 * and neither do the blocks the GK710x HAL passes through unchanged
 * (sflash, the internal PHY, GPIO, DDRC, USB): the GK710x model sits at
 * the S address as it is.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "exec/memattrs.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "system/memory.h"

#define TYPE_GK_REMAP "gk-remap"
OBJECT_DECLARE_SIMPLE_TYPE(GkRemapState, GK_REMAP)

#define GK_REMAP_NONE   0xFF

typedef struct GkRemapReg {
    uint16_t off;           /* driver (S) register */
    uint16_t model;         /* GK710x model register */
    bool perm;
    int8_t w[32];           /* driver bit i -> model bit w[i] */
    int8_t r[32];           /* model bit i -> driver bit r[i] */
} GkRemapReg;

/* Generated from the GK710x HAL 20160913. */
static const GkRemapReg gk_remap_timer[] = {
    { 0x0000, 0x0000 },
    { 0x0004, 0x0008 },
    { 0x0008, 0x000c },
    { 0x000c, 0x0030, true,
      { 9, 5, 1, -1, 10, 6, 2, -1, 8, 4, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      { 10, 2, 6, -1, 9, 1, 5, -1, 8, 0, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { 0x0010, 0x00ff },
    { 0x0014, 0x0010 },
    { 0x0018, 0x0018 },
    { 0x001c, 0x001c },
    { 0x0020, 0x0020 },
    { 0x0024, 0x0028 },
    { 0x0028, 0x002c },
    { 0x002c, 0x00ff },
    { 0x0030, 0x0004 },
    { 0x0034, 0x0014 },
    { 0x0038, 0x0024 },
    { 0x003c, 0x003c },
};

static const GkRemapReg gk_remap_wdt[] = {
    { 0x0000, 0x000c, true,
      { 0, 2, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      { 0, 2, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { 0x0004, 0x0010 },
    { 0x0008, 0x0014 },
    { 0x000c, 0x0000 },
    { 0x0010, 0x0004 },
    { 0x0014, 0x0008 },
    { 0x0018, 0x0018 },
};

static const GkRemapReg gk_remap_idc[] = {
    { 0x0000, 0x0010, true,
      { 0, 1, 2, 3, 4, 5, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      { 0, 1, 2, 3, 4, 5, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { 0x0004, 0x0014, true,
      { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { 0x0008, 0x0018, true,
      { 3, 2, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      { -1, 4, 1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { 0x000c, 0x0004, true,
      { 3, 2, 0, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      { 2, 3, 1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { 0x0010, 0x000c },
    { 0x0014, 0x0000 },
    { 0x0018, 0x0008 },
    { 0x001c, 0x001c },
    { 0x0020, 0x0020 },
    { 0x0024, 0x0024 },
    { 0x0028, 0x0028 },
    { 0x002c, 0x002c },
    { 0x0030, 0x0030 },
    { 0x0034, 0x0034 },
    { 0x0038, 0x0038 },
    { 0x003c, 0x003c },
};

static const GkRemapReg gk_remap_gmac[] = {
    { 0x0004, 0x0010 },
    { 0x0008, 0x0014 },
    { 0x000c, 0x0004 },
    { 0x0010, 0x0008 },
    { 0x0014, 0x000c },
    { 0x0058, 0x0020 },
    { 0x1000, 0x1004 },
    { 0x1004, 0x1008 },
    { 0x1008, 0x1000 },
    { 0x1018, 0x1020 },
    { 0x1020, 0x1018 },
    { 0x1040, 0x1050 },
    { 0x1044, 0x1054 },
};

static const GkRemapReg gk_remap_vic[] = {
    { 0x0000, 0x0024, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0004, 0x002c, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0008, 0x0028, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x000c, 0x000c, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0010, 0x0010, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0014, 0x0014, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0018, 0x0008, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x001c, 0x0018, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0020, 0x001c, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0024, 0x0020 },
    { 0x0028, 0x0030 },
    { 0x002c, 0x0034 },
    { 0x0030, 0x0000, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0034, 0x0004, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x0038, 0x0038, true,
      { 26, 27, 28, 29, 30, 31, 0, 1, 18, 19, 20, 21, 22, 23, 24, 25, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9 },
      { 6, 7, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5 } },
    { 0x003c, 0x003c },
    { 0x0040, 0x0040 },
    { 0x0044, 0x0044 },
    { 0x0048, 0x0048 },
    { 0x004c, 0x004c },
    { 0x0050, 0x0050 },
    { 0x0054, 0x0054 },
    { 0x0058, 0x0058 },
    { 0x005c, 0x005c },
};

static const struct {
    const char *name;
    const GkRemapReg *regs;
    int num_regs;
} gk_remap_blocks[] = {
    { "timer", gk_remap_timer, ARRAY_SIZE(gk_remap_timer) },
    { "wdt",   gk_remap_wdt,   ARRAY_SIZE(gk_remap_wdt) },
    { "idc",   gk_remap_idc,   ARRAY_SIZE(gk_remap_idc) },
    { "gmac",  gk_remap_gmac,  ARRAY_SIZE(gk_remap_gmac) },
    { "vic",   gk_remap_vic,   ARRAY_SIZE(gk_remap_vic) },
};

struct GkRemapState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    char *layout;
    uint32_t size;
    SysBusDevice *target;

    MemoryRegion *target_mr;
    const GkRemapReg *regs;
    int num_regs;
};

static const GkRemapReg *gk_remap_find(GkRemapState *s, hwaddr off)
{
    int i;

    for (i = 0; i < s->num_regs; i++) {
        if (s->regs[i].off == off) {
            return &s->regs[i];
        }
    }
    return NULL;
}

static uint32_t gk_remap_bits(const int8_t map[32], uint32_t v)
{
    uint32_t out = 0;
    int i;

    for (i = 0; i < 32; i++) {
        if ((v >> i) & 1 && map[i] >= 0) {
            out |= 1u << map[i];
        }
    }
    return out;
}

static uint64_t gk_remap_read(void *opaque, hwaddr off, unsigned size)
{
    GkRemapState *s = opaque;
    const GkRemapReg *r = gk_remap_find(s, off);
    hwaddr model = r ? r->model : off;
    uint64_t v = 0;

    if (model == GK_REMAP_NONE) {
        return 0;
    }
    memory_region_dispatch_read(s->target_mr, model, &v, MO_32,
                                MEMTXATTRS_UNSPECIFIED);
    return r && r->perm ? gk_remap_bits(r->r, v) : v;
}

static void gk_remap_write(void *opaque, hwaddr off, uint64_t v,
                           unsigned size)
{
    GkRemapState *s = opaque;
    const GkRemapReg *r = gk_remap_find(s, off);
    hwaddr model = r ? r->model : off;

    if (model == GK_REMAP_NONE) {
        return;
    }
    if (r && r->perm) {
        v = gk_remap_bits(r->w, v);
    }
    memory_region_dispatch_write(s->target_mr, model, v, MO_32,
                                 MEMTXATTRS_UNSPECIFIED);
}

static const MemoryRegionOps gk_remap_ops = {
    .read = gk_remap_read,
    .write = gk_remap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* The HAL has byte and halfword accessors for every block. */
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void gk_remap_realize(DeviceState *dev, Error **errp)
{
    GkRemapState *s = GK_REMAP(dev);
    int i;

    if (!s->target) {
        error_setg(errp, "gk-remap: no target");
        return;
    }
    for (i = 0; i < ARRAY_SIZE(gk_remap_blocks); i++) {
        if (s->layout && !strcmp(s->layout, gk_remap_blocks[i].name)) {
            s->regs = gk_remap_blocks[i].regs;
            s->num_regs = gk_remap_blocks[i].num_regs;
            break;
        }
    }
    if (!s->regs) {
        error_setg(errp, "gk-remap: unknown layout '%s'",
                   s->layout ? s->layout : "");
        return;
    }
    s->target_mr = sysbus_mmio_get_region(s->target, 0);
    memory_region_init_io(&s->iomem, OBJECT(s), &gk_remap_ops, s,
                          TYPE_GK_REMAP, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static const Property gk_remap_properties[] = {
    DEFINE_PROP_STRING("layout", GkRemapState, layout),
    DEFINE_PROP_UINT32("size", GkRemapState, size, 0x1000),
    DEFINE_PROP_LINK("target", GkRemapState, target, TYPE_SYS_BUS_DEVICE,
                     SysBusDevice *),
};

static void gk_remap_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = gk_remap_realize;
    device_class_set_props(dc, gk_remap_properties);
}

static const TypeInfo gk_remap_info = {
    .name          = TYPE_GK_REMAP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GkRemapState),
    .class_init    = gk_remap_class_init,
};

static void gk_remap_register_types(void)
{
    type_register_static(&gk_remap_info);
}

type_init(gk_remap_register_types)
