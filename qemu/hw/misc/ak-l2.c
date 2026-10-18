/*
 * Anyka AK39xx L2 buffer controller and L2 memory.
 *
 * The L2 memory is 8 KiB of SRAM at 0x48000000: eight 512-byte common
 * buffers (0x0000-0x0FFF) that the SPI, MMC, ADC/DAC and USB controllers
 * stream through, then a 256-byte TX + RX pair per UART at 0x1000 + 0x100 n.
 * It decodes only the low 13 address bits: the 3.4 kernel maps it at
 * 0x4800C000 (mach-ak39 map.h) while its decompressor and U-Boot use
 * 0x48001000 for the same UART buffer, so the 64 KiB window is a mirror.
 *
 * A common buffer is a ring of eight 64-byte blocks with a "filled blocks"
 * counter (BUFSTAT1, 4 bits a buffer) shared by its two ends.  The CPU side
 * signals a block by touching its last word: a write to block + 0x3C fills
 * it, a read of block + 0x3C drains it (plat-anyka/l2.c l2_cpu() writes or
 * reads that word for a short block on purpose).  The DMA engine is the
 * other way to work the memory end, a block at a time; a "fraction" DMA
 * moves the odd tail of up to 64 bytes.  Registers (offsets in 0x20140000):
 *
 *   0x00 + 4n  DMA DDR address of buffer n   0x40 + 4n  DMA count:
 *              blocks = (bits 7:0) * (bits 23:16 + 1)
 *   0x80  DMAREQ: bit 24+n starts / is busy for buffer n; bit 9 starts a
 *         fraction DMA of bits 15:10 + 1 bytes to L2 block bits 7:1
 *         (buffer << 3 | block), bit 8 = towards L2
 *   0x84  fraction DMA DDR address (bits 27:0, DDR bits 29:28 in 31:30)
 *   0x88  buffer 0-7: bit n DMA enable, 8+n direction (1 = DDR to L2),
 *         16+n buffer enable, 24+n clear (self-clearing)
 *   0x8C  UART buffers 8-15: 16+n clear; bit 3 with 2:0 / 7:4 forces the
 *         status of common buffer (2:0) to (7:4) (l2_set_status())
 *   0x90 / 0x94  BUFASSIGN: 3 bits a device, the common buffer it uses
 *   0x9C  interrupt enables: 9+n buffer n DMA done, 0 fraction DMA done
 *   0xA0  BUFSTAT1        0xA8  BUFSTAT2 (UART buffers, always 0 here)
 *
 * Code runs from L2 memory at boot (the EV330 mask ROM loads its first
 * stage to 0x48000000), so the SRAM is a ROM device: plain RAM to reads
 * while no common buffer is enabled, MMIO (to see the block handshakes)
 * while one is.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/misc/ak-l2.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define AK_L2_CTRL_SIZE     0x10000

#define R_ADDR(n)           ((0x00 + 4 * (n)) / 4)
#define R_CON(n)            ((0x40 + 4 * (n)) / 4)
#define R_DMAREQ            (0x80 / 4)
#define R_FRACADDR          (0x84 / 4)
#define R_CONBUF0_7         (0x88 / 4)
#define R_CONBUF8_15        (0x8C / 4)
#define R_ASSIGN1           (0x90 / 4)
#define R_ASSIGN2           (0x94 / 4)
#define R_INTEN             (0x9C / 4)
#define R_BUFSTAT1          (0xA0 / 4)
#define R_BUFSTAT2          (0xA8 / 4)

#define DMAREQ_BUF(n)       (1u << (24 + (n)))
#define DMAREQ_FRAC         (1u << 9)
#define DMAREQ_FRAC_TO_L2   (1u << 8)
#define CONBUF_DIR(n)       (1u << (8 + (n)))
#define CONBUF_VLD(n)       (1u << (16 + (n)))
#define CONBUF_VLD_ALL      0x00FF0000u
#define CONBUF_CLR(n)       (1u << (24 + (n)))
#define CONBUF8_SEL_EN      (1u << 3)
#define INTEN_FRAC          (1u << 0)
#define INTEN_BUF(n)        (1u << (9 + (n)))

/* The L2 DMA master reaches DDR with 30 address bits. */
#define AK_DDR_BASE         0x80000000u

static void ak_l2_update_irq(AkL2State *s)
{
    qemu_set_irq(s->irq, s->irq_pending);
}

static void ak_l2_update_romd(AkL2State *s)
{
    memory_region_rom_device_set_romd(&s->mem,
        !(s->regs[R_CONBUF0_7] & CONBUF_VLD_ALL));
}

static uint8_t *ak_l2_block(AkL2State *s, int buf, int blk)
{
    return ak_l2_mem(s) + buf * AK_L2_COMBUF_SIZE + blk * AK_L2_BLOCK;
}

static void ak_l2_clear(AkL2State *s, int n)
{
    AkL2Buf *b = &s->buf[n];

    b->status = 0;
    b->dev_blk = 0;
    b->mem_blk = 0;
}

uint8_t *ak_l2_mem(AkL2State *s)
{
    return memory_region_get_ram_ptr(&s->mem);
}

void ak_l2_register_client(AkL2State *s, AkL2PumpFn pump, void *opaque)
{
    assert(s->num_clients < AK_L2_MAX_CLIENTS);
    s->pump[s->num_clients] = pump;
    s->pump_opaque[s->num_clients] = opaque;
    s->num_clients++;
}

int ak_l2_dev_buf(AkL2State *s, int dev)
{
    uint32_t v = dev < 10 ? s->regs[R_ASSIGN1] >> (3 * dev)
                          : s->regs[R_ASSIGN2] >> (3 * (dev - 10));
    int n = v & 7;

    /*
     * The field is just a buffer number: the kernel's l2_alloc() never
     * hands out buffer 0, but the EV330 first stage streams U-Boot
     * through it.
     */
    if (!(s->regs[R_CONBUF0_7] & CONBUF_VLD(n))) {
        return -1;
    }
    return n;
}

bool ak_l2_push(AkL2State *s, int n, const uint8_t *data, int len)
{
    AkL2Buf *b = &s->buf[n];

    if (b->status >= AK_L2_COMBUF_BLOCKS) {
        return false;
    }
    memcpy(ak_l2_block(s, n, b->dev_blk), data, len);
    b->dev_blk = (b->dev_blk + 1) % AK_L2_COMBUF_BLOCKS;
    b->status++;
    return true;
}

bool ak_l2_pop(AkL2State *s, int n, uint8_t *data, int len)
{
    AkL2Buf *b = &s->buf[n];

    if (!b->status) {
        return false;
    }
    memcpy(data, ak_l2_block(s, n, b->dev_blk), MIN(len, AK_L2_BLOCK));
    b->dev_blk = (b->dev_blk + 1) % AK_L2_COMBUF_BLOCKS;
    b->status--;
    return true;
}

/* One block of buffer n's DMA, if the buffer lets it. */
static bool ak_l2_dma_step(AkL2State *s, int n)
{
    AkL2Buf *b = &s->buf[n];
    uint8_t *blk = ak_l2_block(s, n, b->mem_blk);

    if (!b->dma_left) {
        return false;
    }
    if (s->regs[R_CONBUF0_7] & CONBUF_DIR(n)) {
        if (b->status >= AK_L2_COMBUF_BLOCKS) {
            return false;
        }
        dma_memory_read(&address_space_memory, b->dma_addr, blk,
                        AK_L2_BLOCK, MEMTXATTRS_UNSPECIFIED);
        b->status++;
    } else {
        if (!b->status) {
            return false;
        }
        dma_memory_write(&address_space_memory, b->dma_addr, blk,
                         AK_L2_BLOCK, MEMTXATTRS_UNSPECIFIED);
        b->status--;
    }
    b->mem_blk = (b->mem_blk + 1) % AK_L2_COMBUF_BLOCKS;
    b->dma_addr += AK_L2_BLOCK;
    if (!--b->dma_left) {
        s->regs[R_DMAREQ] &= ~DMAREQ_BUF(n);
        if (s->regs[R_INTEN] & INTEN_BUF(n)) {
            s->irq_pending = true;
        }
    }
    return true;
}

static bool ak_l2_frac_step(AkL2State *s)
{
    uint32_t req = s->regs[R_DMAREQ];
    uint32_t fa = s->regs[R_FRACADDR];
    unsigned l2addr = (req >> 1) & 0x7F;
    unsigned len = ((req >> 10) & 0x3F) + 1;
    hwaddr ddr = AK_DDR_BASE | ((fa >> 2) & 0x30000000) | (fa & 0x0FFFFFFF);
    bool covers_last = len > 0x3C;
    uint8_t *p;
    int n;

    if (!(req & DMAREQ_FRAC)) {
        return false;
    }
    if (l2addr >= 0x40) {
        /* UART buffers: two 64-byte halves each. */
        p = ak_l2_mem(s) + 0x1000 + (l2addr - 0x40) * AK_L2_BLOCK;
        n = -1;
    } else {
        n = l2addr >> 3;
        p = ak_l2_block(s, n, l2addr & 7);
    }
    if (req & DMAREQ_FRAC_TO_L2) {
        dma_memory_read(&address_space_memory, ddr, p, len,
                        MEMTXATTRS_UNSPECIFIED);
        if (n >= 0 && covers_last &&
            s->buf[n].status < AK_L2_COMBUF_BLOCKS) {
            s->buf[n].status++;
        }
    } else {
        if (n >= 0 && !s->buf[n].status) {
            return false;       /* the tail is not in yet */
        }
        dma_memory_write(&address_space_memory, ddr, p, len,
                         MEMTXATTRS_UNSPECIFIED);
        if (n >= 0 && covers_last) {
            s->buf[n].status--;
        }
    }
    s->regs[R_DMAREQ] &= ~DMAREQ_FRAC;
    if (s->regs[R_INTEN] & INTEN_FRAC) {
        s->irq_pending = true;
    }
    return true;
}

void ak_l2_kick(AkL2State *s)
{
    bool progress;
    int n;

    if (s->running) {
        s->rerun = true;
        return;
    }
    s->running = true;
    do {
        progress = false;
        s->rerun = false;
        for (n = 0; n < s->num_clients; n++) {
            progress |= s->pump[n](s->pump_opaque[n]);
        }
        for (n = 0; n < AK_L2_COMBUF_NUM; n++) {
            while (ak_l2_dma_step(s, n)) {
                progress = true;
            }
        }
        progress |= ak_l2_frac_step(s);
    } while (progress || s->rerun);
    s->running = false;
    ak_l2_update_irq(s);
}

/* CPU access to a common buffer's handshake word. */
static void ak_l2_cpu_handshake(AkL2State *s, hwaddr addr, unsigned size,
                                bool write)
{
    int n = addr / AK_L2_COMBUF_SIZE;
    unsigned off = addr % AK_L2_BLOCK;
    AkL2Buf *b;

    if (addr >= AK_L2_COMBUF_NUM * AK_L2_COMBUF_SIZE ||
        off > 0x3C || off + size <= 0x3C ||
        !(s->regs[R_CONBUF0_7] & CONBUF_VLD(n))) {
        return;
    }
    b = &s->buf[n];
    if (write) {
        if (b->status < AK_L2_COMBUF_BLOCKS) {
            b->status++;
        }
    } else if (b->status) {
        b->status--;
    }
    ak_l2_kick(s);
}

static uint64_t ak_l2_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    AkL2State *s = opaque;
    uint64_t val = ldn_le_p(ak_l2_mem(s) + addr, size);

    ak_l2_cpu_handshake(s, addr, size, false);
    return val;
}

static void ak_l2_mem_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    AkL2State *s = opaque;

    stn_le_p(ak_l2_mem(s) + addr, size, val);
    ak_l2_cpu_handshake(s, addr, size, true);
}

static const MemoryRegionOps ak_l2_mem_ops = {
    .read = ak_l2_mem_read,
    .write = ak_l2_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static uint64_t ak_l2_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    AkL2State *s = opaque;
    uint32_t v = 0;
    int n;

    if (offset >= sizeof(s->regs)) {
        qemu_log_mask(LOG_UNIMP, "ak-l2: read of 0x%04" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
    switch (offset / 4) {
    case R_DMAREQ:
        /* The interrupt handler's one read: it acknowledges. */
        if (s->irq_pending) {
            s->irq_pending = false;
            ak_l2_update_irq(s);
        }
        return s->regs[R_DMAREQ];
    case R_BUFSTAT1:
        for (n = 0; n < AK_L2_COMBUF_NUM; n++) {
            v |= (uint32_t)s->buf[n].status << (4 * n);
        }
        return v;
    case R_BUFSTAT2:
        return 0;
    default:
        return s->regs[offset / 4];
    }
}

static void ak_l2_ctrl_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    AkL2State *s = opaque;
    int n;

    if (offset >= sizeof(s->regs)) {
        qemu_log_mask(LOG_UNIMP, "ak-l2: write 0x%08" PRIx64 " to 0x%04"
                      HWADDR_PRIx "\n", val, offset);
        return;
    }
    switch (offset / 4) {
    case R_DMAREQ:
        /*
         * 1 starts a request; a request in flight is not cancelled by the
         * read-modify-writes that start the others.
         */
        for (n = 0; n < AK_L2_COMBUF_NUM; n++) {
            if ((val & DMAREQ_BUF(n)) &&
                !(s->regs[R_DMAREQ] & DMAREQ_BUF(n))) {
                uint32_t con = s->regs[R_CON(n)];
                AkL2Buf *b = &s->buf[n];

                b->dma_left = (con & 0xFF) * (((con >> 16) & 0xFF) + 1);
                b->dma_addr = s->regs[R_ADDR(n)];
                if (!(b->dma_addr & AK_DDR_BASE)) {
                    b->dma_addr |= AK_DDR_BASE;
                }
                if (b->dma_left) {
                    s->regs[R_DMAREQ] |= DMAREQ_BUF(n);
                }
            }
        }
        s->regs[R_DMAREQ] = (s->regs[R_DMAREQ] & 0xFFFF0000u) |
                            (val & 0xFFFF);
        break;
    case R_CONBUF0_7:
        for (n = 0; n < AK_L2_COMBUF_NUM; n++) {
            if (val & CONBUF_CLR(n)) {
                ak_l2_clear(s, n);
            }
        }
        s->regs[R_CONBUF0_7] = val & 0x00FFFFFF;
        ak_l2_update_romd(s);
        break;
    case R_CONBUF8_15:
        if (val & CONBUF8_SEL_EN) {
            s->buf[val & 7].status = MIN((val >> 4) & 0xF,
                                         AK_L2_COMBUF_BLOCKS);
        }
        s->regs[R_CONBUF8_15] = val & ~0x00FF0000u;
        break;
    case R_BUFSTAT1:
    case R_BUFSTAT2:
        return;
    default:
        s->regs[offset / 4] = val;
        break;
    }
    ak_l2_kick(s);
}

static const MemoryRegionOps ak_l2_ctrl_ops = {
    .read = ak_l2_ctrl_read,
    .write = ak_l2_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_l2_reset(DeviceState *dev)
{
    AkL2State *s = AK_L2(dev);
    int n;

    memset(s->regs, 0, sizeof(s->regs));
    for (n = 0; n < AK_L2_COMBUF_NUM; n++) {
        ak_l2_clear(s, n);
        s->buf[n].dma_left = 0;
    }
    s->irq_pending = false;
    ak_l2_update_romd(s);
    ak_l2_update_irq(s);
}

static void ak_l2_realize(DeviceState *dev, Error **errp)
{
    AkL2State *s = AK_L2(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int n;

    if (!memory_region_init_rom_device(&s->mem, OBJECT(dev), &ak_l2_mem_ops,
                                       s, "ak-l2.mem", AK_L2_MEM_SIZE,
                                       errp)) {
        return;
    }
    /* The window: the SRAM, then its mirrors. */
    sysbus_init_mmio(sbd, &s->mem);
    for (n = 0; n < ARRAY_SIZE(s->mem_alias); n++) {
        memory_region_init_alias(&s->mem_alias[n], OBJECT(dev),
                                 "ak-l2.mirror", &s->mem, 0,
                                 AK_L2_MEM_SIZE);
        sysbus_init_mmio(sbd, &s->mem_alias[n]);
    }
}

static void ak_l2_init(Object *obj)
{
    AkL2State *s = AK_L2(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ctrl, obj, &ak_l2_ctrl_ops, s, "ak-l2.ctrl",
                          AK_L2_CTRL_SIZE);
    sysbus_init_mmio(sbd, &s->ctrl);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_ak_l2_buf = {
    .name = "ak-l2-buf",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(status, AkL2Buf),
        VMSTATE_UINT8(dev_blk, AkL2Buf),
        VMSTATE_UINT8(mem_blk, AkL2Buf),
        VMSTATE_UINT32(dma_left, AkL2Buf),
        VMSTATE_UINT32(dma_addr, AkL2Buf),
        VMSTATE_END_OF_LIST()
    }
};

static int ak_l2_post_load(void *opaque, int version_id)
{
    ak_l2_update_romd(opaque);
    return 0;
}

static const VMStateDescription vmstate_ak_l2 = {
    .name = TYPE_AK_L2,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ak_l2_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AkL2State, 0x100 / 4),
        VMSTATE_STRUCT_ARRAY(buf, AkL2State, AK_L2_COMBUF_NUM, 1,
                             vmstate_ak_l2_buf, AkL2Buf),
        VMSTATE_BOOL(irq_pending, AkL2State),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_l2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ak_l2_realize;
    dc->vmsd = &vmstate_ak_l2;
    device_class_set_legacy_reset(dc, ak_l2_reset);
}

static const TypeInfo ak_l2_info = {
    .name          = TYPE_AK_L2,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkL2State),
    .instance_init = ak_l2_init,
    .class_init    = ak_l2_class_init,
};

static void ak_l2_register_types(void)
{
    type_register_static(&ak_l2_info);
}

type_init(ak_l2_register_types)
