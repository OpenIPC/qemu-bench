/*
 * SigmaStar / MStar SPI NOR controller (hw/ssi/sstar-fsp.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_SSI_SSTAR_FSP_H
#define HW_SSI_SSTAR_FSP_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_SSTAR_FSP "sstar-fsp"
OBJECT_DECLARE_SIMPLE_TYPE(SstarFspState, SSTAR_FSP)

#define SSTAR_FSP_OUTSIDE_MAX   0x400

struct SstarFspState {
    SysBusDevice parent_obj;

    MemoryRegion isp_io;        /* RIU bank 0x0008 */
    MemoryRegion fsp_io;        /* RIU bank 0x0016 */
    MemoryRegion qspi_io;       /* RIU bank 0x0017 */
    MemoryRegion xiu;           /* memory-mapped read window */
    qemu_irq irq;

    uint16_t isp[0x80];
    uint16_t fsp[0x80];
    uint16_t qspi[0x80];
    uint8_t rbf[16];
    bool done;
    bool fire_wait;             /* fired; the outside bytes are to come */

    /* the SPI NOR */
    uint8_t *array;
    uint32_t size;
    uint32_t jedec;
    bool sw_cs;                 /* Ifado/Iford FSP-QSPI */
    char *flash_file;
    int fd;
    Error *migration_blocker;
    uint8_t sr[3];
    uint8_t ear;                /* extended address register, read back */
    bool wel;
    uint8_t cmd;
    uint32_t addr;
    uint32_t pos;
    bool selected;
    uint32_t dirty_lo, dirty_hi;
    uint8_t isp_rdata;

    /* bytes BDMA has queued for the FSP's outside write buffer */
    uint8_t outside[SSTAR_FSP_OUTSIDE_MAX];
    uint32_t outside_len;
};

/* BDMA channel with the FSP as destination (MIU -> FSP page program). */
void sstar_fsp_push_outside(SstarFspState *s, const uint8_t *buf,
                            uint32_t len);
/* BDMA channel with the SPI flash as source. */
void sstar_fsp_read_flash(SstarFspState *s, uint32_t off, uint8_t *buf,
                          uint32_t len);

#endif /* HW_SSI_SSTAR_FSP_H */
