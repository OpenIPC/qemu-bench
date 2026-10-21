/*
 * Novatek NA51055 / NA51089 SPI flash controller (hw/ssi/nvt-flash.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_SSI_NVT_FLASH_H
#define HW_SSI_NVT_FLASH_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_NVT_FLASH "nvt-flash"
OBJECT_DECLARE_SIMPLE_TYPE(NvtFlashState, NVT_FLASH)

#define NVT_FLASH_REGS_SIZE 0x300
#define NVT_FLASH_BUF       0x1000

/* The SPI NAND: 2 KiB pages with 64 bytes of OOB, 64 pages a block. */
#define NVT_NAND_PAGE       2048
#define NVT_NAND_OOB        64
#define NVT_NAND_RAW        (NVT_NAND_PAGE + NVT_NAND_OOB)
#define NVT_NAND_BLOCK      64

struct NvtFlashState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[NVT_FLASH_REGS_SIZE / 4];
    bool busy;

    /* PIO: bytes read for the data port, or still to write through it */
    uint8_t buf[NVT_FLASH_BUF];
    uint32_t buf_len, buf_pos;
    uint32_t wr_left;
    uint32_t rd_left;           /* PIO read bytes not yet clocked in */

    /* the flash: a SPI NOR, or a SPI NAND whose array holds each page
     * followed by its OOB */
    uint8_t *array;
    uint32_t array_size;
    uint32_t size;              /* data bytes, OOB not counted */
    uint32_t jedec;
    char *flash_file;
    int fd;
    Error *migration_blocker;
    uint8_t sr[2];
    bool wel;
    bool addr4;
    uint8_t cmd;
    uint32_t addr;
    uint32_t pos;
    bool selected;
    uint32_t dirty_lo, dirty_hi;

    bool nand;
    bool file_oob;              /* flash-file holds the OOB after each page */
    uint32_t pages;
    uint8_t cache[NVT_NAND_RAW];
    uint8_t feat[3];            /* A0 protection, B0 configuration, C0 status */
};

/* What the flash holds at off, for the machine's boot ROM; on a NAND, off
 * counts data bytes only. */
void nvt_flash_read(NvtFlashState *s, uint32_t off, void *buf, uint32_t len);

#endif /* HW_SSI_NVT_FLASH_H */
