/*
 * Axera dw_dmac@48b0000, "axera,axi-dma-1.01a": a real Synopsys
 * DesignWare AXI DMA Controller, confirmed present in the real
 * devicetree with its own real register layout (drivers/dma/dw-axi-
 * dmac/dw-axi-dmac-platform.c, confirmed directly from this project's
 * own openipc-linux-ax620e source tree) -- a genuinely different
 * physical IP block from AX_DMAPER (hw/misc/axera-dmaper.c @
 * 0x048a0000), which BL1/U-Boot use.
 *
 * Added while chasing the real kernel's own SPI-NOR driver timing
 * out trying to mount the squashfs rootfs:
 *   spi-nor spi_nor.0: DMA transaction timed out
 *   SPI transfer failed: -110
 *   Kernel panic - not syncing: VFS: Unable to mount root fs
 * on the assumption that spi@1A00000's own "dmas = <0x09 0x01 0x09
 * 0x00>;" pointed at this block's phandle -- it doesn't: checking
 * phandle 0x09's own definition in the decompiled dtb (not just
 * proximity to the spi node) shows it actually resolves to
 * dma_per@48a0000 (AX_DMAPER) instead -- the real fix (AX_DMAPER
 * needed a real IRQ for the kernel's own dmaengine wait to ever wake
 * up, not a missing register write at all) landed on that device
 * instead, see hw/misc/axera-dmaper.c's own comment. Kept and wired
 * up anyway since it's a real IP block this SoC genuinely has (some
 * other, not yet identified consumer may legitimately need it).
 *
 * Models a single CH_SAR/CH_DAR/CH_BLOCK_TS/CH_CTL-described block
 * transfer per channel, completed synchronously the instant
 * DMAC_CHEN's write-enable convention (writing (1<<ch)|(1<<(ch+8)) to
 * actually enable channel ch, the real hardware's own "which bits of
 * this write to actually apply" idiom) requests it, raising this
 * block's own real IRQ line and clearing the channel back to disabled
 * (real silicon auto-clears CHEN on a non-cyclic transfer's
 * completion, which dw-axi-dmac-platform.c's own
 * dma_chan_terminate_all() already polls for on exactly this bit).
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "system/memory.h"
#include "system/address-spaces.h"

#define TYPE_AXERA_AXI_DMAC "axera-axi-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraAxiDmacState, AXERA_AXI_DMAC)

#define AXERA_AXI_DMAC_SIZE     0x1000
#define AXERA_AXI_DMAC_NR_CHAN  8

#define COMMON_REG_LEN          0x100
#define CHAN_REG_LEN            0x100

#define DMAC_CFG                0x010
#define DMAC_CHEN               0x018
#define DMAC_COMMON_INTCLEAR    0x038

#define CH_SAR                  0x000
#define CH_DAR                  0x008
#define CH_BLOCK_TS             0x010
#define CH_CTL_L                0x018
#define CH_CTL_H                0x01c
#define CH_CFG_L                0x020
#define CH_CFG_H                0x024
#define CH_LLP                  0x028
#define CH_STATUS               0x030
#define CH_INTSTATUS_ENA        0x080
#define CH_INTSTATUS            0x088
#define CH_INTSIGNAL_ENA        0x090
#define CH_INTCLEAR             0x098

#define CH_CTL_L_DST_INC_POS    6
#define CH_CTL_L_SRC_INC_POS    4
#define DWAXIDMAC_CH_CTL_L_NOINC 1

#define DMAC_CHAN_EN_SHIFT      0
#define DMAC_CHAN_EN_WE_SHIFT   8

#define DWAXIDMAC_IRQ_DMA_TRF   BIT(1)

struct AxeraAxiDmacState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t common[COMMON_REG_LEN / 4];
    uint32_t chan[AXERA_AXI_DMAC_NR_CHAN][CHAN_REG_LEN / 4];
};

static void axera_axi_dmac_update_irq(AxeraAxiDmacState *s)
{
    int ch;
    bool pending = false;

    for (ch = 0; ch < AXERA_AXI_DMAC_NR_CHAN; ch++) {
        if (s->chan[ch][CH_INTSTATUS / 4]) {
            pending = true;
            break;
        }
    }
    qemu_set_irq(s->irq, pending);
}

/*
 * One single-block transfer, the only shape drivers/spi/spi-dw-dma.c
 * ever submits: xfer_len bytes from CH_SAR to CH_DAR, each side
 * either auto-incrementing (normal memory) or pinned at a fixed
 * address (the real SPI controller's own DR FIFO register, drained
 * or filled one access at a time) per CH_CTL_L's own SRC_INC/DST_INC
 * bits -- same shape hw/misc/axera-dmaper.c's own real transfer
 * already has to honour for the exact same real-FIFO reason.
 */
static void axera_axi_dmac_do_transfer(AxeraAxiDmacState *s, int ch)
{
    uint64_t sar = ((uint64_t)s->chan[ch][CH_SAR / 4 + 1] << 32) |
                   s->chan[ch][CH_SAR / 4];
    uint64_t dar = ((uint64_t)s->chan[ch][CH_DAR / 4 + 1] << 32) |
                   s->chan[ch][CH_DAR / 4];
    uint32_t block_ts = s->chan[ch][CH_BLOCK_TS / 4] & 0x3fffff;
    uint32_t ctl_l = s->chan[ch][CH_CTL_L / 4];
    bool src_noinc = ((ctl_l >> CH_CTL_L_SRC_INC_POS) & 0x3) ==
                      DWAXIDMAC_CH_CTL_L_NOINC;
    bool dst_noinc = ((ctl_l >> CH_CTL_L_DST_INC_POS) & 0x3) ==
                      DWAXIDMAC_CH_CTL_L_NOINC;
    uint32_t xfer_len = block_ts + 1;
    uint32_t i;

    if (getenv("AXERA_DMAC_DEBUG")) {
        qemu_log("axi-dmac: ch%d sar %" PRIx64 " dar %" PRIx64 " ts %u ctl %08x:%08x cfg %08x:%08x llp %08x:%08x\n",
                 ch, sar, dar, block_ts, ctl_l, s->chan[ch][CH_CTL_H / 4],
                 s->chan[ch][CH_CFG_L / 4], s->chan[ch][CH_CFG_H / 4],
                 s->chan[ch][CH_LLP / 4], s->chan[ch][CH_LLP / 4 + 1]);
    }

    for (i = 0; i < xfer_len; i++) {
        uint8_t byte;

        address_space_read(&address_space_memory,
                           src_noinc ? sar : sar + i,
                           MEMTXATTRS_UNSPECIFIED, &byte, 1);
        address_space_write(&address_space_memory,
                            dst_noinc ? dar : dar + i,
                            MEMTXATTRS_UNSPECIFIED, &byte, 1);
    }

    s->chan[ch][CH_INTSTATUS / 4] |= DWAXIDMAC_IRQ_DMA_TRF;
    /* Real silicon auto-clears CHEN on a non-cyclic transfer's own
     * completion; dma_chan_terminate_all()'s own poll on this exact
     * bit already assumes it. */
    s->common[DMAC_CHEN / 4] &= ~(1U << ch);
    axera_axi_dmac_update_irq(s);
}

static void axera_axi_dmac_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    AxeraAxiDmacState *s = opaque;

    if (offset < COMMON_REG_LEN) {
        switch (offset) {
        case DMAC_CHEN: {
            uint32_t old = s->common[DMAC_CHEN / 4];
            uint32_t ch;

            for (ch = 0; ch < AXERA_AXI_DMAC_NR_CHAN; ch++) {
                if (!(value & (1U << (ch + DMAC_CHAN_EN_WE_SHIFT)))) {
                    continue; /* write-enable bit for this channel unset */
                }
                if (value & (1U << (ch + DMAC_CHAN_EN_SHIFT))) {
                    old |= (1U << ch);
                } else {
                    old &= ~(1U << ch);
                }
            }
            s->common[DMAC_CHEN / 4] = old;
            for (ch = 0; ch < AXERA_AXI_DMAC_NR_CHAN; ch++) {
                if (old & (1U << ch)) {
                    axera_axi_dmac_do_transfer(s, ch);
                }
            }
            return;
        }
        case DMAC_COMMON_INTCLEAR:
            return;
        default:
            s->common[offset / 4] = value;
            return;
        }
    }

    {
        hwaddr rel = offset - COMMON_REG_LEN;
        int ch = rel / CHAN_REG_LEN;
        hwaddr creg = rel % CHAN_REG_LEN;

        if (ch >= AXERA_AXI_DMAC_NR_CHAN) {
            return;
        }
        switch (creg) {
        case CH_INTCLEAR:
            s->chan[ch][CH_INTSTATUS / 4] &= ~value;
            axera_axi_dmac_update_irq(s);
            return;
        default:
            s->chan[ch][creg / 4] = value;
            return;
        }
    }
}

static uint64_t axera_axi_dmac_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    AxeraAxiDmacState *s = opaque;

    if (offset < COMMON_REG_LEN) {
        return s->common[offset / 4];
    }

    {
        hwaddr rel = offset - COMMON_REG_LEN;
        int ch = rel / CHAN_REG_LEN;
        hwaddr creg = rel % CHAN_REG_LEN;

        if (ch >= AXERA_AXI_DMAC_NR_CHAN) {
            return 0;
        }
        return s->chan[ch][creg / 4];
    }
}

static const MemoryRegionOps axera_axi_dmac_ops = {
    .read = axera_axi_dmac_read,
    .write = axera_axi_dmac_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_axi_dmac_realize(DeviceState *dev, Error **errp)
{
    AxeraAxiDmacState *s = AXERA_AXI_DMAC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_axi_dmac_ops, s,
                          TYPE_AXERA_AXI_DMAC, AXERA_AXI_DMAC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void axera_axi_dmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_axi_dmac_realize;
}

static const TypeInfo axera_axi_dmac_info = {
    .name = TYPE_AXERA_AXI_DMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraAxiDmacState),
    .class_init = axera_axi_dmac_class_init,
};

static void axera_axi_dmac_register_types(void)
{
    type_register_static(&axera_axi_dmac_info);
}

type_init(axera_axi_dmac_register_types)
