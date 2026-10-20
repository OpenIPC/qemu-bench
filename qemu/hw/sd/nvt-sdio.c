/*
 * Novatek NA51055 / NA51089 SDIO host, with no card in the slot.
 *
 * U-Boot's drivers/mmc/nvt_ivot_mmc.c and nvt_ivot_mmcreg.h:
 *
 *   0x00 CMD       [5:0] index, bit 6 response wanted, bit 9 send,
 *                  bit 10 controller reset (clears when done)
 *   0x08..0x14 RSP0..3
 *   0x28 STATUS    bit 2 response timeout, bit 3 data timeout, bit 6
 *                  command sent, bit 7 data end; write 1 to clear
 *   0x30 INT_MASK  the same bits
 *   0x4C PHY       bit 0 PHY reset (clears when done)
 *   0x108 FIFO_STS bit 8 empty
 *   0x10C FIFO_CTRL bit 0 enable
 *
 * The drivers spin on these, so an empty slot must answer: every command
 * is "sent" and, if it wanted a response, times out, which is what an
 * empty slot gives on silicon.  U-Boot probes the slot for an update image
 * at boot, finds none and carries on.  A card is not modelled.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_SDIO "nvt-sdio"
OBJECT_DECLARE_SIMPLE_TYPE(NvtSdioState, NVT_SDIO)

#define SDIO_REGS_SIZE      0x200
#define SDIO_CMD            0x00
#define SDIO_STATUS         0x28
#define SDIO_INT_MASK       0x30
#define SDIO_PHY            0x4C
#define SDIO_FIFO_STS       0x108

#define CMD_NEED_RSP        (1u << 6)
#define CMD_EN              (1u << 9)
#define CMD_SDC_RST         (1u << 10)
#define STS_RSP_TIMEOUT     (1u << 2)
#define STS_CMD_SENT        (1u << 6)
#define PHY_SW_RST          (1u << 0)
#define FIFO_EMPTY          (1u << 8)

struct NvtSdioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[SDIO_REGS_SIZE / 4];
};

static void nvt_sdio_update_irq(NvtSdioState *s)
{
    qemu_set_irq(s->irq, s->regs[SDIO_STATUS / 4] &
                 s->regs[SDIO_INT_MASK / 4]);
}

static uint64_t nvt_sdio_read(void *opaque, hwaddr off, unsigned size)
{
    NvtSdioState *s = opaque;

    if (off == SDIO_FIFO_STS) {
        return FIFO_EMPTY;
    }
    return off < SDIO_REGS_SIZE ? s->regs[off / 4] : 0;
}

static void nvt_sdio_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    NvtSdioState *s = opaque;

    switch (off) {
    case SDIO_CMD:
        s->regs[off / 4] = val & ~(CMD_EN | CMD_SDC_RST);
        if (val & CMD_SDC_RST) {
            s->regs[SDIO_STATUS / 4] = 0;
        }
        if (val & CMD_EN) {
            s->regs[SDIO_STATUS / 4] |= STS_CMD_SENT |
                                        (val & CMD_NEED_RSP ?
                                         STS_RSP_TIMEOUT : 0);
        }
        nvt_sdio_update_irq(s);
        break;
    case SDIO_STATUS:
        s->regs[off / 4] &= ~val;
        nvt_sdio_update_irq(s);
        break;
    case SDIO_INT_MASK:
        s->regs[off / 4] = val;
        nvt_sdio_update_irq(s);
        break;
    case SDIO_PHY:
        s->regs[off / 4] = val & ~PHY_SW_RST;
        break;
    default:
        if (off < SDIO_REGS_SIZE) {
            s->regs[off / 4] = val;
        }
        break;
    }
}

static const MemoryRegionOps nvt_sdio_ops = {
    .read = nvt_sdio_read,
    .write = nvt_sdio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_sdio_reset(DeviceState *dev)
{
    NvtSdioState *s = NVT_SDIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void nvt_sdio_init(Object *obj)
{
    NvtSdioState *s = NVT_SDIO(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_sdio_ops, s, TYPE_NVT_SDIO,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_nvt_sdio = {
    .name = TYPE_NVT_SDIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtSdioState, SDIO_REGS_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_sdio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_sdio_reset);
    dc->vmsd = &vmstate_nvt_sdio;
}

static const TypeInfo nvt_sdio_info = {
    .name = TYPE_NVT_SDIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtSdioState),
    .instance_init = nvt_sdio_init,
    .class_init = nvt_sdio_class_init,
};

static void nvt_sdio_register_types(void)
{
    type_register_static(&nvt_sdio_info);
}

type_init(nvt_sdio_register_types)
