/*
 * Novatek mask ROM model (hw/arm/nvt-bootrom.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_NVT_BOOTROM_H
#define HW_ARM_NVT_BOOTROM_H

#include "hw/sysbus.h"
#include "system/memory.h"
#include "hw/ssi/nvt-flash.h"
#include "qom/object.h"

#define TYPE_NVT_BOOTROM "nvt-bootrom"
OBJECT_DECLARE_SIMPLE_TYPE(NvtBootRomState, NVT_BOOTROM)

struct NvtBootRomState {
    SysBusDevice parent_obj;

    MemoryRegion svc;
    MemoryRegion rom;           /* the ROM at its home, rom-base */
    uint8_t *image;
    uint8_t *loader;
    char *loader_file;          /* what the ROM got by TFTP or serial */
    NvtFlashState *flash;
    MemoryRegion *mem;          /* the loader's view (default: system) */
    AddressSpace as;
    uint64_t sram_base;
    uint64_t sram_size;
    uint64_t uart;
    uint64_t sp;
    uint64_t svc_base;
    uint64_t rom_base;
    uint8_t layout;     /* 0 NA51089's table, 1 NA51055's, 2 NS02501 (none) */
    uint32_t args[6];
    uint32_t ret;
};

#endif /* HW_ARM_NVT_BOOTROM_H */
