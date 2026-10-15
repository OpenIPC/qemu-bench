/*
 * HiSilicon V1 (Hi3516CV100 / Hi3518A/C/EV100) DDR controller and PHY.
 *
 * Real V1 DDR does not work until something programs the controller.
 * On a flash boot that something is U-Boot's reg_info table: start.S
 * runs in place from the SPI NOR window and walks the table at image
 * offset 0x40 (arch/arm/cpu/hi3518/start.S, init_registers) before it
 * copies itself to DDR.  The table's DDR part is the same in every
 * table we have (the OpenIPC cv100 one and the vendor DS-I203 one from
 * OpenIPC/u-boot-hi3516cv100#6):
 *
 *   DDRC 0x20110000 +0x1c..+0xf4  timing and geometry, then +0x04 = 0
 *   poll DDRC +0x00 bit 2 == 0    controller idle
 *   PHY  0x20120400 +0x04 = 0x80000000
 *   poll PHY +0x0c bit 0 == 1     PHY ready
 *   PHY  +0x08..+0x200            impedance and timing
 *   PHY  +0x04 = 0xff             bit 0 starts DRAM init and training
 *   poll PHY +0x0c bit 0 == 1     init done
 *
 * An image whose table is empty never touches either block, and on
 * silicon it hangs on its first DDR access with nothing on the UART
 * (OpenIPC/firmware#2593).  This model makes the same image fail here:
 *
 *   region 0  DDRC registers, RAM-backed
 *   region 1  PHY window (0x400 bytes), RAM-backed, +0x0c bit 0 always
 *             reads 1: init and training complete as soon as they start
 *   region 2  "untrained DDR", the size of RAM, mapped over it at a
 *             higher priority: reads 0, drops writes, logs the first
 *             access as a guest error.  Disabled by the first PHY +0x04
 *             write with bit 0 set, enabled again by reset.
 *
 * Region 2 is only created when the "ram-size" property is set; the
 * machine sets it for flash boots only, since a -kernel boot has no
 * table to run and needs DDR from the first instruction.
 *
 * Copyright (c) 2026 OpenIPC.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_HISI_DDRC_V1 "hisi-ddrc-v1"
OBJECT_DECLARE_SIMPLE_TYPE(HisiDdrcV1State, HISI_DDRC_V1)

#define DDRC_MMIO_SIZE      0x10000
#define PHY_MMIO_SIZE       0x400

#define PHY_INITCTRL        0x04    /* bit 0: start DRAM init + training */
#define PHY_INITSTATUS      0x0c    /* bit 0: done */
#define PHY_INIT_START      (1u << 0)
#define PHY_INIT_DONE       (1u << 0)

struct HisiDdrcV1State {
    SysBusDevice parent_obj;
    MemoryRegion ctrl_iomem;
    MemoryRegion phy_iomem;
    MemoryRegion untrained_iomem;
    uint32_t ctrl[DDRC_MMIO_SIZE / 4];
    uint32_t phy[PHY_MMIO_SIZE / 4];
    uint64_t ram_size;
    bool trained;
    bool untrained_logged;
};

static void hisi_ddrc_v1_set_trained(HisiDdrcV1State *s, bool trained)
{
    s->trained = trained;
    if (s->ram_size) {
        memory_region_set_enabled(&s->untrained_iomem, !trained);
    }
}

static uint64_t hisi_ddrc_v1_ctrl_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    HisiDdrcV1State *s = HISI_DDRC_V1(opaque);

    return s->ctrl[offset / 4];
}

static void hisi_ddrc_v1_ctrl_write(void *opaque, hwaddr offset,
                                    uint64_t val, unsigned size)
{
    HisiDdrcV1State *s = HISI_DDRC_V1(opaque);

    s->ctrl[offset / 4] = val;
}

static const MemoryRegionOps hisi_ddrc_v1_ctrl_ops = {
    .read = hisi_ddrc_v1_ctrl_read,
    .write = hisi_ddrc_v1_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t hisi_ddrc_v1_phy_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    HisiDdrcV1State *s = HISI_DDRC_V1(opaque);
    uint32_t val = s->phy[offset / 4];

    if (offset == PHY_INITSTATUS) {
        val |= PHY_INIT_DONE;
    }
    return val;
}

static void hisi_ddrc_v1_phy_write(void *opaque, hwaddr offset,
                                   uint64_t val, unsigned size)
{
    HisiDdrcV1State *s = HISI_DDRC_V1(opaque);

    s->phy[offset / 4] = val;
    if (offset == PHY_INITCTRL && (val & PHY_INIT_START) && !s->trained) {
        hisi_ddrc_v1_set_trained(s, true);
    }
}

static const MemoryRegionOps hisi_ddrc_v1_phy_ops = {
    .read = hisi_ddrc_v1_phy_read,
    .write = hisi_ddrc_v1_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_ddrc_v1_untrained_log(HisiDdrcV1State *s, hwaddr offset,
                                       bool is_write)
{
    if (!s->untrained_logged) {
        s->untrained_logged = true;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-ddrc-v1: DDR %s at +0x%" HWADDR_PRIx
                      " before the DDR PHY was initialised "
                      "(empty reg_info table?)\n",
                      is_write ? "write" : "read", offset);
    }
}

static uint64_t hisi_ddrc_v1_untrained_read(void *opaque, hwaddr offset,
                                            unsigned size)
{
    hisi_ddrc_v1_untrained_log(HISI_DDRC_V1(opaque), offset, false);
    return 0;
}

static void hisi_ddrc_v1_untrained_write(void *opaque, hwaddr offset,
                                         uint64_t val, unsigned size)
{
    hisi_ddrc_v1_untrained_log(HISI_DDRC_V1(opaque), offset, true);
}

static const MemoryRegionOps hisi_ddrc_v1_untrained_ops = {
    .read = hisi_ddrc_v1_untrained_read,
    .write = hisi_ddrc_v1_untrained_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void hisi_ddrc_v1_realize(DeviceState *dev, Error **errp)
{
    HisiDdrcV1State *s = HISI_DDRC_V1(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->ctrl_iomem, OBJECT(dev), &hisi_ddrc_v1_ctrl_ops,
                          s, "hisi-ddrc-v1", DDRC_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->ctrl_iomem);

    memory_region_init_io(&s->phy_iomem, OBJECT(dev), &hisi_ddrc_v1_phy_ops,
                          s, "hisi-ddrc-v1.phy", PHY_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->phy_iomem);

    if (s->ram_size) {
        memory_region_init_io(&s->untrained_iomem, OBJECT(dev),
                              &hisi_ddrc_v1_untrained_ops, s,
                              "hisi-ddrc-v1.untrained", s->ram_size);
        sysbus_init_mmio(sbd, &s->untrained_iomem);
    }
}

static void hisi_ddrc_v1_reset(DeviceState *dev)
{
    HisiDdrcV1State *s = HISI_DDRC_V1(dev);

    memset(s->ctrl, 0, sizeof(s->ctrl));
    memset(s->phy, 0, sizeof(s->phy));
    s->untrained_logged = false;
    hisi_ddrc_v1_set_trained(s, false);
}

static const Property hisi_ddrc_v1_properties[] = {
    DEFINE_PROP_UINT64("ram-size", HisiDdrcV1State, ram_size, 0),
};

static void hisi_ddrc_v1_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hisi_ddrc_v1_realize;
    device_class_set_props(dc, hisi_ddrc_v1_properties);
    device_class_set_legacy_reset(dc, hisi_ddrc_v1_reset);
}

static const TypeInfo hisi_ddrc_v1_info = {
    .name          = TYPE_HISI_DDRC_V1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiDdrcV1State),
    .class_init    = hisi_ddrc_v1_class_init,
};

static void hisi_ddrc_v1_register_types(void)
{
    type_register_static(&hisi_ddrc_v1_info);
}

type_init(hisi_ddrc_v1_register_types)
