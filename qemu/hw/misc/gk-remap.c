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
 * The SD controllers (driver layout: arch-gk7101s gk7101_reg.h REG_SDIO_*,
 * halfword and word registers) are a standard SDHCI under the GK710x HAL,
 * generic-sdhci here; their rows carry the register width.
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
#include "qemu/bitops.h"
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
    uint8_t size;           /* 1, 2 or 4 bytes, the same on both sides
                             * (0: 4); a 1-byte row is used only by byte
                             * accesses */
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

static const GkRemapReg gk_remap_sdio[] = {
    { .off = 0x00, .model = 0x00, .size = 4 },	/* SDMA address */
    /* S sd.h: bit 4 MUL_SEL, bit 5 CTH_SEL (card to host) -- the GK710x
     * one has them the other way round, which this row composes in. */
    { .off = 0x04, .model = 0x0c, .size = 2, .perm = true,	/* MOD: transfer mode */
      .w = { 1, 2, 0, -1, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 2, 0, 1, -1, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x06, .model = 0x38, .size = 2 },	/* NOR_INT_SIG_EN */
    { .off = 0x08, .model = 0x3a, .size = 2, .perm = true,	/* ERR_INT_SIG_EN */
      .w = { 0, 2, 3, 4, 1, 5, 6, 7, 8, -1, -1, -1, -1, -1, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 0, 4, 1, 2, 3, 5, 6, 7, 8, -1, -1, -1, -1, -1, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x0a, .model = 0x06, .size = 2 },	/* BLK_COU: block count */
    { .off = 0x0c, .model = 0x04, .size = 2 },	/* BLK_SIZ: block size */
    { .off = 0x0e, .model = 0x34, .size = 2 },	/* NOR_INT_STA_EN */
    { .off = 0x10, .model = 0x36, .size = 2, .perm = true,	/* ERR_INT_STA_EN */
      .w = { 0, 2, 1, 3, 5, 4, 6, 7, 8, -1, -1, -1, -1, -1, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 0, 2, 1, 3, 5, 4, 6, 7, 8, -1, -1, -1, -1, -1, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x12, .model = 0x30, .size = 2, .perm = true,	/* NOR_INT_STA */
      .w = { 0, 2, 3, 1, 4, 6, 5, 7, 8, 9, -1, -1, -1, -1, -1, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 0, 3, 1, 2, 4, 6, 5, 7, 8, 9, -1, -1, -1, -1, -1, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x14, .model = 0x32, .size = 2 },	/* ERR_INT_STA */
    { .off = 0x16, .model = 0x0e, .size = 2, .perm = true,	/* COMM: command */
      .w = { 0, 1, -1, 3, 5, 4, 6, 7, 8, 9, 10, 11, 12, 13, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 0, 1, -1, 3, 5, 4, 6, 7, 8, 9, 10, 11, 12, 13, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x18, .model = 0x2e, .size = 2 },	/* SF_RST_CTRL: timeout, reset */
    { .off = 0x1a, .model = 0x2c, .size = 2 },	/* CLK_CTRL */
    { .off = 0x1c, .model = 0x10, .size = 4 },	/* RESP0 */
    { .off = 0x20, .model = 0x14, .size = 4 },	/* RESP1 */
    { .off = 0x24, .model = 0x18, .size = 4 },	/* RESP2 */
    { .off = 0x28, .model = 0x1c, .size = 4 },	/* RESP3 */
    { .off = 0x2c, .model = 0x28, .size = 4, .perm = true,	/* CONTROL00: host control */
      .w = { 0, 1, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 0, 1, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x30, .model = 0x24, .size = 4, .perm = true,	/* PRESENTSTATE */
      .w = { 0, 2, 1, -1, -1, -1, -1, -1, 9, 10, 8, 11, -1, -1, -1, -1, 16, 18, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, -1, -1, -1 },
      .r = { 0, 2, 1, -1, -1, -1, -1, -1, 10, 8, 9, 11, -1, -1, -1, -1, 16, 18, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, -1, -1, -1 } },
    { .off = 0x34, .model = 0x08, .size = 4 },	/* ARG */
    { .off = 0x38, .model = 0x40, .size = 4, .perm = true,	/* CAP */
      .w = { 0, 1, 2, 3, 4, 5, -1, 7, 8, 9, 10, 11, 12, 13, -1, -1, 16, 17, 18, -1, -1, 21, 23, 22, 24, 25, 26, 27, -1, -1, -1, -1 },
      .r = { 0, 1, 2, 3, 4, 5, -1, 7, 8, 9, 10, 11, 12, 13, -1, -1, 16, 17, 18, -1, -1, 21, 23, 22, 24, 25, 26, 27, -1, -1, -1, -1 } },
    { .off = 0x3c, .model = 0x3c, .size = 4, .perm = true,	/* AUTOCMD12ERRSTATUS */
      .w = { 1, 2, 3, 0, 4, -1, -1, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 3, 0, 1, 2, 4, -1, -1, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x40, .model = 0x20, .size = 4 },	/* BUFFERDATAPORT */
    /*
     * The byte registers inside them, as byte accesses reach them: the
     * GK710x HAL's byte accessors decode these on their own, with their
     * own bit orders, and the driver uses them for the timeout, the
     * resets (CMD 1, ALL 2, DAT 4 on the vendor's side), host and power
     * control (sd.h "Byte" registers).
     */
    { .off = 0x18, .model = 0x2e, .size = 1 },	/* TMO: data timeout */
    { .off = 0x19, .model = 0x2f, .size = 1, .perm = true,	/* RESET (CMD 1, ALL 2, DAT 4) */
      .w = { 1, 0, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 1, 0, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x2c, .model = 0x28, .size = 1, .perm = true,	/* HOST: host control */
      .w = { 0, 1, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 0, 1, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x2d, .model = 0x29, .size = 1 },	/* PWR: power control */
    { .off = 0x2e, .model = 0x2a, .size = 1, .perm = true,	/* GAP: block gap control */
      .w = { 0, 2, 1, 3, 7, 5, 6, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 0, 2, 1, 3, 7, 5, 6, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
    { .off = 0x2f, .model = 0x2b, .size = 1, .perm = true,	/* WAK: wakeup control */
      .w = { 1, 0, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
      .r = { 1, 0, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
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
    { "sdio",  gk_remap_sdio,  ARRAY_SIZE(gk_remap_sdio) },
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

/*
 * The register holding byte off: a table row, or else the aligned word it
 * is in, passed through at the same offset.
 */
typedef struct GkRemapHit {
    const GkRemapReg *r;    /* NULL: pass-through */
    hwaddr off;             /* the register's driver offset */
    hwaddr model;
    unsigned size;
} GkRemapHit;

static GkRemapHit gk_remap_find(GkRemapState *s, hwaddr off, unsigned width)
{
    GkRemapHit h = { NULL, off & ~3, off & ~3, 4 };
    int i;

    for (i = 0; width == 1 && i < s->num_regs; i++) {
        const GkRemapReg *r = &s->regs[i];

        if (r->size == 1 && r->off == off) {
            h.r = r;
            h.off = off;
            h.model = r->model;
            h.size = 1;
            return h;
        }
    }
    for (i = 0; i < s->num_regs; i++) {
        const GkRemapReg *r = &s->regs[i];
        unsigned size = r->size ? r->size : 4;

        if (size > 1 && off >= r->off && off < r->off + size) {
            h.r = r;
            h.off = r->off;
            h.model = r->model;
            h.size = size;
            break;
        }
    }
    return h;
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

/* One whole driver register, through the table. */
static uint32_t gk_remap_read_reg(GkRemapState *s, const GkRemapHit *h)
{
    uint64_t v = 0;

    if (h->model == GK_REMAP_NONE) {
        return 0;
    }
    memory_region_dispatch_read(s->target_mr, h->model, &v,
                                size_memop(h->size), MEMTXATTRS_UNSPECIFIED);
    return h->r && h->r->perm ? gk_remap_bits(h->r->r, v) : v;
}

static void gk_remap_write_reg(GkRemapState *s, const GkRemapHit *h,
                               uint64_t v)
{
    if (h->model == GK_REMAP_NONE) {
        return;
    }
    if (h->r && h->r->perm) {
        v = gk_remap_bits(h->r->w, v);
    }
    memory_region_dispatch_write(s->target_mr, h->model, v,
                                 size_memop(h->size), MEMTXATTRS_UNSPECIFIED);
}

/*
 * An access is split over the registers it covers: the vendor drivers use
 * byte, halfword and word accessors, and the SD block's registers are
 * halfwords and words.  A read takes each register's bytes out of it.  A
 * write of a whole register writes it; a write of part of one merges its
 * bytes into the register's current value (a read of it, then a write of
 * the whole register).  The merge read is plain for the timer, watchdog,
 * IDC, GMAC and VIC models; on the SD controller a partial write of a
 * write-1-to-clear status register or of the data port would have the
 * read's effect too, which the vendor driver, using the registers' own
 * widths, never asks for.
 */
static uint64_t gk_remap_read(void *opaque, hwaddr off, unsigned size)
{
    GkRemapState *s = opaque;
    uint64_t v = 0;
    hwaddr pos;

    for (pos = off; pos < off + size; ) {
        GkRemapHit h = gk_remap_find(s, pos, size);
        unsigned in = pos - h.off;
        unsigned n = MIN(h.size - in, off + size - pos);

        v |= extract64(gk_remap_read_reg(s, &h), 8 * in, 8 * n)
             << (8 * (pos - off));
        pos += n;
    }
    return v;
}

static void gk_remap_write(void *opaque, hwaddr off, uint64_t v,
                           unsigned size)
{
    GkRemapState *s = opaque;
    hwaddr pos;

    for (pos = off; pos < off + size; ) {
        GkRemapHit h = gk_remap_find(s, pos, size);
        unsigned in = pos - h.off;
        unsigned n = MIN(h.size - in, off + size - pos);
        uint64_t part = extract64(v, 8 * (pos - off), 8 * n);

        if (n < h.size) {
            part = deposit64(gk_remap_read_reg(s, &h), 8 * in, 8 * n, part);
        }
        gk_remap_write_reg(s, &h, part);
        pos += n;
    }
}

static const MemoryRegionOps gk_remap_ops = {
    .read = gk_remap_read,
    .write = gk_remap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* The HAL has byte and halfword accessors for every block. */
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
    .impl.min_access_size = 1,
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
