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
#include "qemu/timer.h"

#define TYPE_NVT_BOOTROM "nvt-bootrom"

/*
 * The ROM's Ethernet boot drives the MAC (nvt-eqos) itself before any
 * loader runs: attached, it gets every frame instead of the DMA rings.
 */
typedef void (*NvtEqosRomRx)(void *opaque, const uint8_t *buf, size_t len);
void nvt_eqos_rom_attach(DeviceState *eqos, NvtEqosRomRx rx, void *opaque);
void nvt_eqos_rom_send(DeviceState *eqos, const uint8_t *buf, size_t len);
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

    /*
     * Recovery (nvt-bootrom.c, "the ROM's fallback"): with no loader in
     * flash, or straps not at flash, the CPU parks while the ROM looks for
     * a loader on its other sources.
     */
    bool fallback;              /* run it when the flash has no loader */
    bool fast;                  /* timeouts / 10, for tests */
    uint8_t strap;              /* TOP+0 BS[3:0] */
    uint64_t top;               /* TOP, for the chip word at +0xF0 */
    QEMUTimer *timer;           /* the current stage's timeout */
    QEMUTimer *poll;            /* the UART receive poll */
    int source;                 /* the stage running, NVT_SRC_* */
    int tries;
    /* UART download (the ROM's CDB engine) */
    uint8_t chunk[16];
    int nchunk;
    bool storing;               /* a WRLOADER has started the store */
    bool echo_due;              /* the WRLOADER's echo, once size is in */
    uint8_t wr_cdb[16];
    uint32_t dest, size, count;
    /* Ethernet boot: DHCP, then loader.bin over TFTP */
    DeviceState *eth;           /* nvt-eqos, if the machine has one */
    uint8_t mac[6];
    int eth_state;
    uint32_t xid, my_ip, srv_ip, mask;
    uint8_t srv_mac[6];
    uint16_t srv_port, block;
    uint32_t got;
};

#endif /* HW_ARM_NVT_BOOTROM_H */
