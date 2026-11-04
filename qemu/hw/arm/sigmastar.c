/*
 * SigmaStar (MStar) ARMv7 SoC machines.
 *
 * Table-driven like hisilicon.c, xiongmai.c, goke.c and anyka.c;
 * include/hw/arm/sigmastar.h has the family.  Boots a board's SPI NOR
 * image (flash-file=) through its own IPL and U-Boot, the way the mask
 * ROM does, or a kernel with -kernel.
 *
 * Copyright (c) 2026 Daniel Palmer <daniel@thingy.jp>
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/sigmastar.h"
#include "system/runstate.h"
#include "hw/boards.h"
#include "hw/cpu/a15mpcore.h"
#include "hw/intc/arm_gicv3_common.h"
#include "hw/arm/bsa.h"
#include "qobject/qlist.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "hw/ide/ahci.h"
#include "hw/ide/ahci-sysbus.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/sstar-riu.h"
#include "hw/misc/sstar-xzdec.h"
#include "hw/misc/unimp.h"
#include "hw/or-irq.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/ssi/sstar-fsp.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/cpu.h"
#include "target/arm/cpregs.h"

#define SS_BOOTROM_BASE     0x00000000
#define SS_BOOTROM_SIZE     (16 * KiB)
#define SS_XIU_BASE         0x14000000
#define SS_PERIPHBASE       0x16000000
#define SS_DRAM_BASE        0x20000000
#define SS_DRAM_WINDOW      (512 * MiB)
#define SS_DRAM_ALIASED     (2 * SS_DRAM_WINDOW)    /* to 0x60000000 */
#define SS_IMI_BASE         0xA0000000
#define SS_IMI_SIZE         (64 * KiB)

#define SS_MAX_CPUS         2
#define SS_GIC_NUM_SPI      128
#define SS_GICV3_NUM_IRQ    256         /* Iford: MS_FIQ ends at SPI 191 */
#define SS_GICV3_REDIST     0x16040000  /* Iford DTS */
#define SS_CNTFRQ_HZ        6000000     /* arm,armv7-timer clock-frequency */

/* RIU blocks with models; bank numbers as the vendor code names them. */
#define SS_ISP_BASE         SSTAR_RIU_ADDR(0x0008, 0)
#define SS_FSP_BASE         SSTAR_RIU_ADDR(0x0016, 0)
#define SS_QSPI_BASE        SSTAR_RIU_ADDR(0x0017, 0)
#define SS_TIMER_BASE       SSTAR_RIU_ADDR(0x0030, 0)   /* wdt + 3 timers */
#define SS_BDMA_BASE        SSTAR_RIU_ADDR(0x1002, 0)
#define SS_INTC_BASE        SSTAR_RIU_ADDR(0x1009, 0)
#define SS_SMPCTRL_BASE     SSTAR_RIU_ADDR(0x1020, 0)

/*
 * The smpctrl mailbox (bank 0x1020): CPU1 runs the mask ROM and parks
 * until the kernel writes its entry address to 0x13 (high half) and
 * 0x14 (low half) and the magic 0xBABE to 0x16 (smp_platform.c:
 * SECOND_START_ADDR_HI/LO, SECOND_MAGIC_NUMBER_ADDR).  Here CPU1 is
 * powered off instead and the magic powers it on at that address, as
 * Daniel Palmer's model does.  The bank otherwise reads back.
 *
 * Iford keeps the address (smp_platform.c) but no magic: 0x15 takes two bits
 * per core, bit 2n wake and bit 2n + 1 Non-secure.  Nothing on the flash
 * looks at them (the IPL brings up CPU0's GIC only), so what parks CPU1 is
 * the ROM's or a secure firmware's, and QEMU stands in for it: the core comes
 * up at that address, in the state asked for, as arm_set_cpu_on() leaves it.
 */
#define SMPCTRL_BOOT_HI     0x13
#define SMPCTRL_BOOT_LO     0x14
#define SMPCTRL_WAKE        0x15
#define SMPCTRL_WAKE_CPU1   (1u << 2)
#define SMPCTRL_NS_CPU1     (1u << 3)
#define SMPCTRL_UNLOCK      0x16
#define SMPCTRL_MAGIC       0xBABE

/*
 * Iford's cores are Cortex-A32s, which QEMU does not have.  An A7 runs its
 * code; its SMF stub only adds these two A32 implementation registers.
 */
static const ARMCPRegInfo ss_a32_cp_reginfo[] = {
    { .name = "CPUACTLR", .cp = 15, .opc1 = 0, .crm = 15,
      .type = ARM_CP_CONST | ARM_CP_64BIT, .access = PL1_RW, .resetvalue = 0 },
    { .name = "CPUECTLR", .cp = 15, .opc1 = 1, .crm = 15,
      .type = ARM_CP_CONST | ARM_CP_64BIT, .access = PL1_RW, .resetvalue = 0 },
};

/* GIC SPIs, which are also the vendor DTS interrupt numbers. */
#define SS_SPI_FSP          37
#define SS_SPI_GOP_VSYNC    52
#define SS_SPI_SATA         50
#define SS_SPI_GE           57
#define SS_SPI_EMAC         58
#define SS_SPI_VPU          53
#define SS_SPI_UART0        66
#define SS_SPI_UART1        67
#define SS_SPI_UART2        71
#define SS_SPI_FUART        79
#define SS_SPI_BDMA0        72
#define SS_SPI_BDMA1        73
#define SS_SPI_BDMA2        93
#define SS_SPI_BDMA3        94
#define SS_SPI_DISP_TOP     82
#define SS_SPI_UHC2         63
#define SS_SPI_UHC1         65
#define SS_SPI_UHC0         87
#define SS_SPI_VENC         53          /* Ifado; the VPU's on infinity2m */
#define SS_SPI_ISP          57          /* Ifado */
#define SS_SPI_VIF          89          /* Ifado: the DTS vif's first */
#define SS_SPI_CMDQ         49          /* Ifado: cmdq0-3 */
#define SS_SPI_JPE          61          /* Ifado: the DTS "jpe-irq0" */
#define SS_SPI_AUDIO        74          /* Ifado sstar,audio; I3 BACH */
#define SS_SPI_PM_SLEEP     34          /* infinity3: INT_IRQ_PM_SLEEP */
#define SS_SPI_MIIC0        68          /* Ifado */
#define SS_SPI_MIIC1        69
#define SS_SPI_MIIC2        92
#define SS_SPI_FURDMA       80          /* Ifado */
#define SS_SPI_URDMA0       93          /* Ifado; BDMA2 on infinity2m */
#define SS_SPI_URDMA1       94          /* Ifado; BDMA3 on infinity2m */
#define SS_SPI_RTC          76          /* infinity3: INT_IRQ_RTC */
#define SS_SPI_I3_SCL       52          /* infinity3: INT_IRQ_SC_TOP */
#define SS_SPI_I3_ISP       57          /* infinity3: INT_IRQ_ISP */
#define SS_SPI_I3_CSI       65          /* infinity3: INT_IRQ_MIPI_CSI2 */
#define SS_SPI_I3_MFE       60          /* infinity3: INT_IRQ_MFE */
#define SS_SPI_I3_CMDQ      49          /* infinity3: INT_IRQ_CMDQ */
#define SS_SPI_I3_JPE       61          /* infinity3: INT_IRQ_JPE */
#define SS_SPI_I3_VHE       53          /* infinity3: INT_IRQ_VHE */
#define SS_SPI_I3_SDIO      51          /* infinity3: INT_IRQ_SDIO */
/* Iford's numbers: MS_IRQ from SPI 32, MS_FIQ from 160 (irqs.h) */
#define SS_IPU_BASE         0x19000000
#define SS_IPU_SIZE         0x390000
#define SS_SPI_IFORD_ISP_DMA 56         /* INT_IRQ_ISP0_DMA */
#define SS_SPI_IFORD_RTC    40          /* INT_IRQ_RTC0 */
#define SS_SPI_IFORD_UART1  69
#define SS_SPI_IFORD_UR0    67          /* the _MERGE lines */
#define SS_SPI_IFORD_UR1    92
#define SS_SPI_IFORD_EMAC   51
#define SS_SPI_IFORD_PM_MIIC 43
#define SS_SPI_IFORD_XZDEC  86
#define SS_SPI_IFORD_VENC   58          /* INT_IRQ_MHE_0 */
#define SS_SPI_IFORD_UHC    63
#define SS_SPI_IFORD_SD     50
#define SS_SPI_IFORD_TIMER  160         /* INT_FIQ_TIMER: all three */
#define SS_SPI_IFORD_WDT    161
#define SS_SPI_TIMER0       96
#define SS_SPI_TIMER1       97
#define SS_SPI_WDT          98
#define SS_SPI_TIMER2       108

/*
 * Reset values the boot chain reads back without writing first, captured
 * on an SSD202D with contrib/mstarpoker by Daniel Palmer (mstar-clean,
 * hw/arm/mstarv7.c).  Same die family as the SSR621Q.
 */
static const SstarRegDefault ss_i2m_defaults[] = {
    /* clkgen (0x1F207000) */
    { 0x1F207008, 0x0001 }, { 0x1F20700C, 0x0001 }, { 0x1F2070CC, 0x1101 },
    { 0x1F2070D0, 0x0211 }, { 0x1F2070D4, 0x45A9 }, { 0x1F2070DC, 0x0101 },
    { 0x1F207108, 0x0001 }, { 0x1F207114, 0x0001 }, { 0x1F207118, 0x0101 },
    { 0x1F207144, 0x0001 }, { 0x1F207148, 0x0001 }, { 0x1F20714C, 0x0101 },
    { 0x1F207150, 0x0001 }, { 0x1F207154, 0x0101 }, { 0x1F207158, 0x0101 },
    { 0x1F20715C, 0x0001 }, { 0x1F207180, 0x0001 }, { 0x1F207184, 0x0001 },
    { 0x1F20718C, 0x0009 }, { 0x1F2071A8, 0x0001 }, { 0x1F2071B0, 0x0100 },
    { 0x1F2071B4, 0x003D }, { 0x1F2071B8, 0x0001 }, { 0x1F2071BC, 0x0001 },
    { 0x1F2071C4, 0xFFFF }, { 0x1F2071CC, 0x0A40 }, { 0x1F2071F8, 0x0101 },
    { 0x1F2071FC, 0xFFFF },
    /* PM clkgen (0x1F001C00) */
    { 0x1F001C00, 0x0003 }, { 0x1F001C20, 0x00FF }, { 0x1F001C50, 0x000F },
    { 0x1F001C58, 0x003F }, { 0x1F001C70, 0x0300 }, { 0x1F001C7C, 0x1800 },
    { 0x1F001CB8, 0x00FF }, { 0x1F001CC0, 0x0003 }, { 0x1F001CD4, 0x0008 },
    { 0x1F001CDC, 0xFFFF }, { 0x1F001CEC, 0xFFFF }, { 0x1F001CF0, 0xFFFF },
    { 0x1F001CF4, 0x003F }, { 0x1F001D24, 0x0020 }, { 0x1F001D28, 0x000F },
    { 0x1F001D30, 0x000F }, { 0x1F001D58, 0x003F }, { 0x1F001D84, 0x3C3F },
    { 0x1F001D88, 0x4000 }, { 0x1F001D8C, 0x0600 }, { 0x1F001D90, 0x2000 },
    { 0x1F001D94, 0x01FF }, { 0x1F001D98, 0x0006 }, { 0x1F001DBC, 0x0003 },
    /* chiptop (0x1F203C00), the bond strap at +0x120 aside */
    { 0x1F203C04, 0x0200 }, { 0x1F203C74, 0x7FF2 }, { 0x1F203C78, 0x800D },
    { 0x1F203C80, 0xFFFF }, { 0x1F203C88, 0xFFFF }, { 0x1F203CC4, 0x7FFF },
    { 0x1F203CC8, 0x7F00 }, { 0x1F203CE0, 0xFFFF }, { 0x1F203CE4, 0x0FFF },
    { 0x1F203CE8, 0xFFFF }, { 0x1F203CEC, 0x0FFF }, { 0x1F203CF0, 0xFFFF },
    { 0x1F203CF4, 0x0FFF }, { 0x1F203D00, 0x0003 }, { 0x1F203D10, 0xFFFF },
    { 0x1F203D14, 0xFFFF }, { 0x1F203D18, 0xFFFF }, { 0x1F203D50, 0x0054 },
    { 0x1F203D94, 0x1006 }, { 0x1F203DCC, 0xFFFF }, { 0x1F203DF0, 0xFFFF },
    { 0x1F203DF4, 0x0003 }, { 0x1F203DF8, 0x000F },
    /* chipver (0x1F003C00): +0x19C revision, read by the GOP code */
    { 0x1F003D9C, 0x0100 },
    /*
     * efuse (0x1F004000): the DDR ZQ / drive-strength calibration and the
     * USB/ETH/MIPI/HDMI analog trims the IPL reads (read_config_word)
     */
    { 0x1F004014, 0x0809 }, { 0x1F004018, 0xE148 }, { 0x1F00401C, 0x000D },
    { 0x1F004020, 0x0010 }, { 0x1F004024, 0x0591 }, { 0x1F004028, 0x4210 },
    { 0x1F00402C, 0x0F70 }, { 0x1F004058, 0x10BD }, { 0x1F00405C, 0x741A },
    { 0x1F004060, 0x84D1 }, { 0x1F004068, 0x5B65 }, { 0x1F00406C, 0x254A },
    { 0x1F004070, 0x4385 }, { 0x1F004074, 0x85CC },
    /* USB PLL, clock gate, UTMI PHYs (powered down) and USBC */
    { 0x1F283E00, 0x11B2 }, { 0x1F284000, 0x11B2 },
    { 0x1F284200, 0xFF05 }, { 0x1F284204, 0x9080 }, { 0x1F284210, 0x04AF },
    { 0x1F284600, 0x0200 },
    { 0x1F284A00, 0xFF05 }, { 0x1F284A04, 0x9080 }, { 0x1F284A10, 0x04AF },
    { 0x1F284E00, 0x0200 },
    { 0x1F285200, 0xFF05 }, { 0x1F285204, 0x9080 }, { 0x1F285210, 0x04AF },
    { 0x1F286200, 0x0200 },
    /* MIPI/DSI analog */
    { 0x1F2A4A00, 0x1000 }, { 0x1F2A4F40, 0x8002 },
    /* DID key: the boot-media strap, 0x20 = SPI NOR */
    { 0x1F0071C0, 0x0020 },
    /*
     * MIU DDR PLL frequency set: what the IPL programs, for a -kernel
     * boot that skips it (the kernel's MIU clock driver divides by it)
     */
    { 0x1F202060, 0x8000 }, { 0x1F202064, 0x0029 },
    { 0 }
};

/*
 * SSR621Q: the infinity2m die in Xiongmai's 4/8/16-channel NVRs, with
 * SATA, HDMI and the video decoder bonded out and DDR3 in the package.
 * The vendor DTS (model "INFINITY2M SSC010A-S01A-S") has memory at
 * 0x20000000 and the U-Boot bootargs LX_MEM=0xFF00000: 256 MiB.
 */
static const SsSoCConfig ssr621q_soc = {
    .desc               = "SigmaStar SSR621Q (infinity2m, 2x Cortex-A7)",
    .num_cpus           = 2,
    .ram_size_default   = 256 * MiB,
    .chip_id            = 0xF0,
    .bond               = 0x1E,
    .flash_jedec        = 0xC22018,         /* MX25L12835F, 16 MiB */
};

/* SSD202D: 128 MiB DDR3 in the package (bond 0x1E). */
static const SsSoCConfig ssd202d_soc = {
    .desc               = "SigmaStar SSD202D (infinity2m, 2x Cortex-A7)",
    .num_cpus           = 2,
    .ram_size_default   = 128 * MiB,
    .chip_id            = 0xF0,
    .bond               = 0x1E,
    .flash_jedec        = 0xEF4018,         /* W25Q128 */
};

/* SSD201: 64 MiB DDR2 in the package (bond 0x1D). */
static const SsSoCConfig ssd201_soc = {
    .desc               = "SigmaStar SSD201 (infinity2m, 2x Cortex-A7)",
    .num_cpus           = 2,
    .ram_size_default   = 64 * MiB,
    .chip_id            = 0xF0,
    .bond               = 0x1D,
    .flash_jedec        = 0xEF4018,
};

/*
 * SSC375 (die "Ifado", Linux "sstar,ifado"): one Cortex-A7 at 1 GHz with
 * DDR in a QFN88, the infinity map with a later FSP-QSPI.  Sources: the
 * kernel DTB and boot chain in a Dahua IPC-HDW1433V-A-IL2 NOR (model
 * "IFADO SSC032B-S01A", Linux 4.9.84, U-Boot 2021.10).  Its IPL picks
 * the DRAM profile by the bond strap, 0x10-0x19 (QFN88 WB512M/ESMT512M/
 * ETRON512M DDR2, NTC1G/WB1G/PM1G DDR3); 0x15, WB1G DDR3, gives the
 * board's mem=128M.  It runs from IMI past 64 KiB (stack 0xA00155E0).
 */
static const SstarRegDefault ss_ifado_defaults[] = {
    /*
     * PM clkgen +0x7C [14:10], the boot device strap: U-Boot's flash
     * driver loads the CIS's NRI records only when (x & 0x19) is 0x18
     * or 0x19, SPI NOR.  The infinity2m value, 0x1800, reads as none.
     */
    { 0x1F001C7C, 0x7800 },
    /*
     * CPU PLL (bank 0x1032): the current LPF word, regs 0x60 and 0x61
     * [7:0], which the clock driver reads while no frequency has been set
     * (ms_cpuclk_recalc_rate in the Maruko SDK's infinity6c clock driver;
     * zero makes the kernel divide by it), and post divider - 1 in reg 0x19.
     * 432 MHz * 2^19 * 16 / 0x374BC6 is the 1 GHz the IPL reports.
     */
    { 0x1F206464, 0x0001 },
    { 0x1F206580, 0x4BC6 }, { 0x1F206584, 0x0037 },
    /*
     * IPU PLL (bank 0x1035): what the kernel's hal_clk_ipupll_init writes
     * when reg 0x11 bit 8 says the PLL is off.  The clock framework works
     * out its rate from the LPF word in regs 0x60/0x61 before that, and
     * divides by it.
     */
    { 0x1F206A44, 0x0180 }, { 0x1F206A48, 0x1088 }, { 0x1F206A64, 0x0001 },
    { 0x1F206B80, 0x6666 }, { 0x1F206B84, 0x0026 }, { 0x1F206B88, 0x0001 },
    { 0 }
};

static const SsSoCConfig ssc375_soc = {
    .desc               = "SigmaStar SSC375 (ifado, 1x Cortex-A7)",
    .die                = SS_DIE_IFADO,
    .num_cpus           = 1,
    .ram_size_default   = 128 * MiB,
    .chip_id            = 0xF0,
    .bond               = 0x15,
    .flash_jedec        = 0xEF4018,         /* W25Q128 */
    .imi_size           = 128 * KiB,
    .fsp_sw_cs          = true,
    .smf_ack            = true,
    .defaults           = ss_ifado_defaults,
    .urdma              = true,
    .miic               = 3,
    .miic_sensor_bus    = 1,
    .default_sensor     = "sc431hai",
    .ipc                = true,
    .gpio_bank          = 0x103E,
};

/*
 * MSC313E (die "infinity3", Linux "mstar,infinity3"): one Cortex-A7 with
 * 64 MiB DDR2 in a QFN.  Sources: the vendor I3 SDK
 * (MSC3XX_SDK_V_1711291828-I3: U-Boot 2015.01, Linux 3.18.30, the
 * prebuilt image_QFN64M-MSC313E flash image) and the MSC313E datasheet.
 * Its mask ROM looks for the IPL at NOR 0x4000 (linux-chenxing.org
 * blobs/bootrom.md); the chip ID is 0xC2.  The IPL sizes DRAM by the wrap
 * and reads no bond strap.  Also booted: the 8 MiB W25Q64 (0xEF4017) NOR
 * of a retail camera with the same chain.
 */
static const SsSoCConfig msc313e_soc = {
    .desc               = "MStar MSC313E (infinity3, 1x Cortex-A7)",
    .die                = SS_DIE_INFINITY3,
    .num_cpus           = 1,
    .ram_size_default   = 64 * MiB,
    .chip_id            = 0xC2,
    .flash_jedec        = 0xEF4018,         /* W25Q128 */
    .ipl_offset         = 0x4000,
    .ipc                = true,
    /*
     * MIIC0/1 only (DTS i2c0@0, i2c1@1); the F22 on MIIC1, the bus the
     * camera's U-Boot sets ("Setting bus to 1").
     */
    .miic               = 2,
    .miic_sensor_bus    = 1,
    .default_sensor     = "jxf22",
};

static const SstarRegDefault ss_iford_defaults[] = {
    /*
     * MIU: DRAM size, log2 of MiB, which the kernel's
     * reg_miu_dram_get_size and so mi_sys read.  Nothing in the boot
     * chain writes it here: on silicon it is presumably strapped or set
     * by the DRAM init this model skips.
     */
    { SSTAR_RIU_ADDR(0x1659, 0x0D), 7 },
    { 0 }
};

/*
 * SSC029 (die "iford", Linux "sstar,iford"): two Cortex-A32s, run here by
 * A7s (see ss_a32_cp_reginfo), a GICv3, and its blocks moved from where
 * the older dies have them.  Sources: the vendor Iford SDK (IMD00V2.1.0:
 * U-Boot 2021.10, Linux 5.10.117) and its prebuilt NOR image
 * ssz029c.128.qfn128_ddr3.  The boot flash is on the non-PM FSP-QSPI,
 * banks 0x100D/0x100E (hal_fsp_qspi_is_boot_storage).
 *
 * Not from the SDK: the chip ID, which nothing in that image reads, and
 * the bond, which the IPL takes as its DRAM profile (0-5 are valid; 0
 * works for this QFN128 DDR3 image).  IMI is the kernel's IMI_SIZE
 * (registers.h), 240 KiB.
 */
static const SsSoCConfig iford_soc = {
    .desc               = "SigmaStar SSC029 (iford, 2x Cortex-A32)",
    .die                = SS_DIE_IFORD,
    .num_cpus           = 2,
    .ram_size_default   = 128 * MiB,
    .chip_id            = 0xF0,
    .bond               = 0,
    .flash_jedec        = 0xEF4018,         /* W25Q128 */
    .imi_size           = 240 * KiB,
    .fsp_sw_cs          = true,
    .fsp_bank           = 0x100D,
    .qspi_bank          = 0x100E,
    .defaults           = ss_iford_defaults,
    .smf_ack            = true,
    .urdma              = true,
    .miic               = 3,
    .miic_sensor_bus    = 1,                /* DTS snr0_mipi_i2c */
    .default_sensor     = "os04d10",
    .ipc                = true,
    .gpio_bank          = 0x103E,
};

#define TYPE_SS_MACHINE MACHINE_TYPE_NAME("sigmastar")
OBJECT_DECLARE_SIMPLE_TYPE(SsMachineState, SS_MACHINE)

struct SsMachineState {
    MachineState parent_obj;

    char *flash_file;
    uint32_t flash_jedec;
    char *sensor;
    char *rtc;
    uint64_t uuid;
    bool night;
    DeviceState *i3_gpio;
    ARMCPU *cpus[SS_MAX_CPUS];
    MemoryRegion smpctrl;
    uint16_t smpctrl_regs[0x80];
    bool iford;
    /* what bus masters see: MIU bus addresses, DRAM from 0 */
    MemoryRegion miu_root;
    MemoryRegion miu_sys;
    MemoryRegion miu_low;
    MemoryRegion miu_hi;
    AddressSpace miu_as;
    MemoryRegion sata_win;
};

static char *ss_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(SS_MACHINE(obj)->flash_file);
}

static void ss_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    SsMachineState *s = SS_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static char *ss_machine_get_rtc(Object *obj, Error **errp)
{
    return g_strdup(SS_MACHINE(obj)->rtc);
}

static void ss_machine_set_rtc(Object *obj, const char *value, Error **errp)
{
    SsMachineState *s = SS_MACHINE(obj);

    g_free(s->rtc);
    s->rtc = g_strdup(value);
}

static char *ss_machine_get_sensor(Object *obj, Error **errp)
{
    return g_strdup(SS_MACHINE(obj)->sensor);
}

static void ss_machine_set_sensor(Object *obj, const char *value,
                                  Error **errp)
{
    SsMachineState *s = SS_MACHINE(obj);

    g_free(s->sensor);
    s->sensor = g_strdup(value);
}

static void ss_machine_get_flash_jedec(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    visit_type_uint32(v, name, &SS_MACHINE(obj)->flash_jedec, errp);
}

static void ss_machine_set_flash_jedec(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    visit_type_uint32(v, name, &SS_MACHINE(obj)->flash_jedec, errp);
}

static void ss_machine_get_uuid(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    visit_type_uint64(v, name, &SS_MACHINE(obj)->uuid, errp);
}

static void ss_machine_set_uuid(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    visit_type_uint64(v, name, &SS_MACHINE(obj)->uuid, errp);
}

static char *ss_machine_get_daynight(Object *obj, Error **errp)
{
    return g_strdup(SS_MACHINE(obj)->night ? "night" : "day");
}

/* Before init it is the board's start; afterwards, a change of light. */
static void ss_machine_set_daynight(Object *obj, const char *value,
                                    Error **errp)
{
    SsMachineState *s = SS_MACHINE(obj);

    if (strcmp(value, "day") && strcmp(value, "night")) {
        error_setg(errp, "daynight is day or night");
        return;
    }
    s->night = !strcmp(value, "night");
    if (s->i3_gpio) {
        object_property_set_bool(OBJECT(s->i3_gpio), "night", s->night,
                                 errp);
    }
}

static uint64_t ss_smpctrl_read(void *opaque, hwaddr off, unsigned size)
{
    SsMachineState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->smpctrl_regs[(off >> 2) & 0x7F];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void ss_smpctrl_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SsMachineState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    uint16_t *r = &s->smpctrl_regs[reg];

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        *r = (off & 1) ? (*r & 0x00FF) | ((val & 0xFF) << 8)
                       : (*r & 0xFF00) | (val & 0xFF);
    } else {
        *r = val;
    }
    if (s->cpus[1] && (s->iford ?
                       reg == SMPCTRL_WAKE && (*r & SMPCTRL_WAKE_CPU1) :
                       reg == SMPCTRL_UNLOCK && *r == SMPCTRL_MAGIC)) {
        uint32_t entry = s->smpctrl_regs[SMPCTRL_BOOT_LO] |
                         ((uint32_t)s->smpctrl_regs[SMPCTRL_BOOT_HI] << 16);
        /* Released from the mask ROM, so in the Secure state. */
        int el = arm_feature(&s->cpus[1]->env, ARM_FEATURE_EL3) &&
                 !(s->iford && (*r & SMPCTRL_NS_CPU1)) ? 3 : 1;
        int ret;

        if (s->iford) {
            /* the IPL's GIC init, for this core: its lines are Non-secure */
            MemTxAttrs secure = { .secure = 1 };

            address_space_stl_le(&address_space_memory,
                                 SS_GICV3_REDIST + 0x20000 +
                                 0x10000 + 0x80, UINT32_MAX, secure, NULL);
        }
        ret = arm_set_cpu_on(s->cpus[1]->mp_affinity, entry, 0, el, false);

        if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS &&
            ret != QEMU_ARM_POWERCTL_ALREADY_ON) {
            qemu_log_mask(LOG_GUEST_ERROR, "sigmastar: cannot start CPU1 "
                          "at 0x%08x (%d)\n", entry, ret);
        }
    }
}

static const MemoryRegionOps ss_smpctrl_ops = {
    .read = ss_smpctrl_read,
    .write = ss_smpctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * The IPL header at the start of the SPI NOR (and of the IPL image the
 * ROM copies to IMI):
 *
 *   +0x00  branch over the header      +0x04  "IPL_"
 *   +0x08  image size (16 bits)        +0x0C  checksum
 *
 * The mask ROM checks the magic through the XIU window, copies the
 * image into IMI at 0xA0000000 with BDMA and enters it there, the stack
 * in IMI at 0xA000D000 (Daniel Palmer's ROM notes, docs/system/arm/
 * mstarv7/bootrom.rst).  The IPL then brings up DDR, loads IPL_CUST,
 * which unpacks U-Boot.
 */
#define SS_IPL_MAGIC        0x5F4C5049      /* "IPL_" */
#define SS_IPL_STACK        0xA000D000

/*
 * What stands in for the mask ROM: the writes the ROM makes before it
 * enters the IPL -- watchdog off, timer 0 free-running from 0 to
 * 0xFFFFFFFF, the UART0 pad mux and clock -- then sp and a jump to the
 * IPL.  The same {address, value} table runner as anyka.c:
 *
 *     ldr   sp, [pc, #20]
 *     add   r2, pc, #20        @ r2 = table
 *  1: ldmia r2!, {r0, r1}
 *     cmp   r0, #0
 *     strne r1, [r0]
 *     bne   1b
 *     ldr   pc, [r2]
 *     .word SS_IPL_STACK
 */
static const uint32_t ss_bootrom_stub[] = {
    0xE59FD014, 0xE28F2014, 0xE8B20003, 0xE3500000, 0x15801000, 0x1AFFFFFB,
    0xE592F000, SS_IPL_STACK,
};

static const uint32_t ss_bootrom_script[] = {
    0x1F006010, 0x0000, 0x1F006014, 0x0000,     /* WDT MAX_PRD = 0: off */
    0x1F006040, 0x0000,                         /* timer 0 stopped */
    0x1F006048, 0xFFFF, 0x1F00604C, 0xFFFF,     /* MAX = 0xFFFFFFFF */
    0x1F006040, 0x0001,                         /* EN */
    0x1F2070C4, 0x0009,                         /* clk_uart */
    0x1F203D4C, 0x3210,                         /* UART pad mux */
};

static bool ss_boot_rom_setup(const uint8_t *img, gsize len, const char *path,
                              uint32_t ipl_offset, bool monitor)
{
    GArray *rom = g_array_new(false, false, sizeof(uint32_t));
    const uint32_t end[3] = { 0, 0, SS_IMI_BASE };
    const uint32_t cps_mon = 0xF1020016;        /* cps #22 */
    uint32_t size;
    guint i;

    if (len < ipl_offset + 0x10) {
        error_report("sigmastar: %s is too short for an IPL at 0x%x",
                     path, ipl_offset);
        return false;
    }
    img += ipl_offset;
    len -= ipl_offset;
    if (ldl_le_p(img + 4) != SS_IPL_MAGIC) {
        error_report("sigmastar: %s has no IPL header (\"IPL_\" at 0x%x)",
                     path, ipl_offset + 4);
        return false;
    }
    size = lduw_le_p(img + 8);
    if (size < 0x10 || size > len) {
        error_report("sigmastar: %s: IPL size 0x%x does not fit IMI",
                     path, size);
        return false;
    }
    /*
     * Iford's IPL toggles SCR.NS around its ICC_HSRE write, which only a
     * core in Monitor mode survives: the ROM must leave it there.  An
     * inference from the IPL: there is no dump of this ROM.
     */
    if (monitor) {
        g_array_append_val(rom, cps_mon);
    }
    g_array_append_vals(rom, ss_bootrom_stub, ARRAY_SIZE(ss_bootrom_stub));
    g_array_append_vals(rom, ss_bootrom_script,
                        ARRAY_SIZE(ss_bootrom_script));
    g_array_append_vals(rom, end, ARRAY_SIZE(end));
    for (i = 0; i < rom->len; i++) {
        g_array_index(rom, uint32_t, i) =
            cpu_to_le32(g_array_index(rom, uint32_t, i));
    }
    rom_add_blob_fixed("sigmastar.bootrom", rom->data, rom->len * 4,
                       SS_BOOTROM_BASE);
    g_array_free(rom, true);
    rom_add_blob_fixed("sigmastar.ipl", img, size, SS_IMI_BASE);
    return true;
}

/* A reset parks CPU1 again: the mailbox forgets its magic and address. */
static void ss_smpctrl_reset(void *opaque)
{
    SsMachineState *s = opaque;

    memset(s->smpctrl_regs, 0, sizeof(s->smpctrl_regs));
}

/* Secondaries wait on smpctrl, never on boot.c's pen. */
static void ss_write_secondary_boot(ARMCPU *cpu,
                                    const struct arm_boot_info *info)
{
}

static void ss_reset_secondary(ARMCPU *cpu, const struct arm_boot_info *info)
{
}

/*
 * Iford's MIU MMU seen from the CPU: mi_sys maps its buffers at 0x60000000
 * on, MIU 0x40000000, the MMU's VPA region, and the CPU's accesses go
 * through the table as a bus master's do (sstar_miu_xlate).
 */
#define SS_VPA_BASE         0x60000000
#define SS_VPA_SIZE         (512 * MiB)

static MemTxResult ss_vpa_read(void *opaque, hwaddr off, uint64_t *data,
                               unsigned size, MemTxAttrs attrs)
{
    uint64_t pa = sstar_miu_xlate(SS_VPA_BASE - SS_DRAM_BASE + off);
    uint64_t v = 0;
    MemTxResult r = MEMTX_OK;

    /* a miss leaves the address in this window: nothing behind it */
    if (pa < SS_VPA_BASE - SS_DRAM_BASE) {
        r = address_space_read(&address_space_memory, SS_DRAM_BASE + pa,
                               attrs, &v, size);
    }
    *data = le64_to_cpu(v);
    return r;
}

static MemTxResult ss_vpa_write(void *opaque, hwaddr off, uint64_t data,
                                unsigned size, MemTxAttrs attrs)
{
    uint64_t pa = sstar_miu_xlate(SS_VPA_BASE - SS_DRAM_BASE + off);
    uint64_t v = cpu_to_le64(data);

    if (pa >= SS_VPA_BASE - SS_DRAM_BASE) {
        return MEMTX_OK;
    }
    return address_space_write(&address_space_memory, SS_DRAM_BASE + pa,
                               attrs, &v, size);
}

static const MemoryRegionOps ss_vpa_ops = {
    .read_with_attrs = ss_vpa_read,
    .write_with_attrs = ss_vpa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .impl.unaligned = true,
};

static qemu_irq ss_spi(DeviceState *gic, DeviceState *intc, int spi)
{
    /*
     * SPI 32 up are the main-intc's MS_IRQ and MS_FIQ groups: to 127, and
     * on Iford, whose first group is twice the size, to 191.
     */
    if (spi >= 32 &&
        spi < (object_property_get_bool(OBJECT(intc), "iford",
                                        &error_abort) ? 192 : 128)) {
        return qdev_get_gpio_in(intc, spi - 32);
    }
    return qdev_get_gpio_in(gic, spi);
}

static void sigmastar_common_init(MachineState *machine, const SsSoCConfig *c)
{
    SsMachineState *sms = SS_MACHINE(machine);
    static struct arm_boot_info binfo;
    static SstarRegDefault chip_id[2];
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *imi = g_new(MemoryRegion, 1);
    MemoryRegion *bootrom = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    gsize img_len = 0;
    DeviceState *gic, *intc, *riu, *fsp, *dev, *i3_isp = NULL, *isp0 = NULL;
    const int ncpus = MIN(machine->smp.cpus, c->num_cpus);

    assert(c->num_cpus <= SS_MAX_CPUS);
    uint64_t off;
    int n;

    /* The MIU decodes a power of two; the window repeats it. */
    if (machine->ram_size > SS_DRAM_WINDOW ||
        !is_power_of_2(machine->ram_size)) {
        error_report("sigmastar: DRAM must be a power of two up to %" PRIu64
                     " MiB", (uint64_t)SS_DRAM_WINDOW / MiB);
        exit(1);
    }
    if (sms->flash_file) {
        GError *gerr = NULL;

        if (!g_file_get_contents(sms->flash_file, (gchar **)&img, &img_len,
                                 &gerr)) {
            error_report("sigmastar: cannot read %s: %s", sms->flash_file,
                         gerr->message);
            g_error_free(gerr);
            exit(1);
        }
    }

    for (n = 0; n < ncpus; n++) {
        Object *o = object_new(machine->cpu_type);

        object_property_set_int(o, "reset-cbar", SS_PERIPHBASE,
                                &error_fatal);
        /*
         * 6 MHz, as the vendor DTS pins arm,armv7-timer: the kernel
         * scales the counter by it.
         */
        object_property_set_int(o, "cntfrq", SS_CNTFRQ_HZ, &error_fatal);
        /* CPU1 parks in the mask ROM until smpctrl releases it. */
        object_property_set_bool(o, "start-powered-off", n > 0,
                                 &error_fatal);
        qdev_realize(DEVICE(o), NULL, &error_fatal);
        sms->cpus[n] = ARM_CPU(o);
        if (c->die == SS_DIE_IFORD) {
            define_arm_cp_regs(sms->cpus[n], ss_a32_cp_reginfo);
        }
    }

    /*
     * DRAM, mirrored through the whole 512 MiB MIU0 window as the address
     * decode wraps: the IPL sizes DRAM by looking for the wrap.  The next
     * 512 MiB repeat it again: the Xiongmai U-Boot converts its logo into
     * "dstAddr + 0x20000000" with dstAddr already a CPU address (0x2DD00000
     * becomes 0x4DD00000), and the NVR shows it.
     */
    memory_region_add_subregion(sysmem, SS_DRAM_BASE, machine->ram);
    for (off = machine->ram_size; off + machine->ram_size <= SS_DRAM_ALIASED;
         off += machine->ram_size) {
        MemoryRegion *alias = g_new(MemoryRegion, 1);

        memory_region_init_alias(alias, NULL, "sigmastar.dram-mirror",
                                 machine->ram, 0, machine->ram_size);
        memory_region_add_subregion(sysmem, SS_DRAM_BASE + off, alias);
    }

    if (c->die == SS_DIE_IFORD) {
        MemoryRegion *vpa = g_new(MemoryRegion, 1);

        memory_region_init_io(vpa, NULL, &ss_vpa_ops, NULL,
                              "sigmastar.miu-vpa", SS_VPA_SIZE);
        memory_region_add_subregion(sysmem, SS_VPA_BASE, vpa);
    }

    memory_region_init_ram(imi, NULL, "sigmastar.imi",
                           c->imi_size ? c->imi_size : SS_IMI_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SS_IMI_BASE, imi);
    memory_region_init_rom(bootrom, NULL, "sigmastar.bootrom",
                           SS_BOOTROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, SS_BOOTROM_BASE, bootrom);

    if (c->die == SS_DIE_IFORD) {
        /* GICv3: the IPL itself reaches for ICC_MSRE. */
        QList *redist = qlist_new();

        qlist_append_int(redist, ncpus);
        gic = qdev_new(gicv3_class_name());
        qdev_prop_set_uint32(gic, "revision", 3);
        qdev_prop_set_uint32(gic, "num-cpu", ncpus);
        qdev_prop_set_uint32(gic, "num-irq", SS_GICV3_NUM_IRQ);
        qdev_prop_set_bit(gic, "has-security-extensions", true);
        qdev_prop_set_array(gic, "redist-region-count", redist);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gic), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(gic), 0, SS_PERIPHBASE);
        sysbus_mmio_map(SYS_BUS_DEVICE(gic), 1, SS_GICV3_REDIST);
        for (n = 0; n < ncpus; n++) {
            DeviceState *cpu = DEVICE(sms->cpus[n]);
            int base = SS_GICV3_NUM_IRQ - GIC_INTERNAL + n * GIC_INTERNAL;
            static const int timer_ppi[] = {
                [GTIMER_PHYS] = ARCH_TIMER_NS_EL1_IRQ,
                [GTIMER_VIRT] = ARCH_TIMER_VIRT_IRQ,
                [GTIMER_HYP]  = ARCH_TIMER_NS_EL2_IRQ,
                [GTIMER_SEC]  = ARCH_TIMER_S_EL1_IRQ,
            };
            int t;

            for (t = 0; t < ARRAY_SIZE(timer_ppi); t++) {
                qdev_connect_gpio_out(cpu, t, qdev_get_gpio_in(gic,
                                      base + timer_ppi[t]));
            }
            qdev_connect_gpio_out_named(cpu, "gicv3-maintenance-interrupt",
                                        0, qdev_get_gpio_in(gic,
                                        base + ARCH_GIC_MAINT_IRQ));
        }
    } else {
        /* GIC-400 in the A7 private region: distributor +0x1000, CPU +0x2000 */
        gic = qdev_new(TYPE_A15MPCORE_PRIV);
        qdev_prop_set_uint32(gic, "num-cpu", ncpus);
        qdev_prop_set_uint32(gic, "num-irq", SS_GIC_NUM_SPI + 32);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gic), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(gic), 0, SS_PERIPHBASE);
    }
    for (n = 0; n < ncpus; n++) {
        DeviceState *cpu = DEVICE(sms->cpus[n]);
        SysBusDevice *sbd = SYS_BUS_DEVICE(gic);

        sysbus_connect_irq(sbd, n, qdev_get_gpio_in(cpu, ARM_CPU_IRQ));
        sysbus_connect_irq(sbd, n + ncpus,
                           qdev_get_gpio_in(cpu, ARM_CPU_FIQ));
        sysbus_connect_irq(sbd, n + 2 * ncpus,
                           qdev_get_gpio_in(cpu, ARM_CPU_VIRQ));
        sysbus_connect_irq(sbd, n + 3 * ncpus,
                           qdev_get_gpio_in(cpu, ARM_CPU_VFIQ));
    }
    create_unimplemented_device("sigmastar.periphbase", SS_PERIPHBASE,
                                0x8000);

    /* Storage under every RIU bank, and the boot chain's status bits. */
    riu = qdev_new(TYPE_SSTAR_RIU);
    qdev_prop_set_uint16(riu, "bond", c->bond);
    qdev_prop_set_bit(riu, "smf-ack", c->smf_ack);
    qdev_prop_set_bit(riu, "infinity3", c->die == SS_DIE_INFINITY3);
    qdev_prop_set_bit(riu, "iford", c->die == SS_DIE_IFORD);
    sms->iford = c->die == SS_DIE_IFORD;
    qdev_prop_set_uint16(riu, "gpio-bank", c->gpio_bank);
    qdev_prop_set_uint64(riu, "uuid", sms->uuid);
    sstar_riu_add_defaults(SSTAR_RIU(riu), ss_i2m_defaults);
    if (c->defaults) {
        sstar_riu_add_defaults(SSTAR_RIU(riu), c->defaults);
    }
    /* chipver +0x00: the chip ID byte U-Boot and the kernel identify by */
    chip_id[0].addr = SSTAR_RIU_ADDR(0x001E, 0x00);
    chip_id[0].val = c->chip_id;
    sstar_riu_add_defaults(SSTAR_RIU(riu), chip_id);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(riu), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(riu), 0, SSTAR_RIU_BASE, -2);

    intc = qdev_new("sstar-intc");
    qdev_prop_set_bit(intc, "iford", c->die == SS_DIE_IFORD);
    object_property_add_child(OBJECT(machine), "intc", OBJECT(intc));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(intc), 0, SS_INTC_BASE);
    for (n = 0; n < (c->die == SS_DIE_IFORD ? 160 : 96); n++) {
        qdev_connect_gpio_out(intc, n, qdev_get_gpio_in(gic, 32 + n));
    }

    dev = qdev_new("sstar-timer");
    object_property_set_link(OBJECT(dev), "riu", OBJECT(riu), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SS_TIMER_BASE);
    if (c->die == SS_DIE_IFORD) {
        /* one line for the timers: the first; the others go unconnected */
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_IFORD_TIMER));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 2,
                           ss_spi(gic, intc, SS_SPI_IFORD_WDT));
    } else {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_TIMER0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                           ss_spi(gic, intc, SS_SPI_TIMER1));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 2,
                           ss_spi(gic, intc, SS_SPI_WDT));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 3,
                           ss_spi(gic, intc, SS_SPI_TIMER2));
    }

    /* UART0 is the console (DTS alias serial0), then uart1, fuart, uart2 */
    for (n = 0; n < 4; n++) {
        static const hwaddr base[4] = {
            0x1F221000, 0x1F221200, 0x1F220400, 0x1F221400,
        };
        static const int spi[4] = {
            SS_SPI_UART0, SS_SPI_UART1, SS_SPI_FUART, SS_SPI_UART2,
        };
        static const int iford_spi[4] = {
            SS_SPI_UART0, SS_SPI_IFORD_UART1, SS_SPI_FUART, SS_SPI_UART2,
        };
        /* Ifado DTS: each UART's URDMA block and line; UART2 has none */
        static const hwaddr ur_base[3] = {
            0x1F220C00, 0x1F220E00, 0x1F220600,
        };
        static const int ur_spi[3] = {
            SS_SPI_URDMA0, SS_SPI_URDMA1, SS_SPI_FURDMA,
        };
        static const hwaddr iford_ur_base[3] = {
            0x1F220E00, 0x1F221C00, 0x1F220600,
        };
        static const int iford_ur_spi[3] = {
            SS_SPI_IFORD_UR0, SS_SPI_IFORD_UR1, SS_SPI_FURDMA,
        };
        bool iford = c->die == SS_DIE_IFORD;
        bool ur = c->urdma && n < 3;

        dev = qdev_new("sstar-uart");
        qdev_prop_set_chr(dev, "chardev", serial_hd(n));
        qdev_prop_set_bit(dev, "urdma", ur);
        qdev_prop_set_bit(dev, "urdma-tx-v2", ur);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, iford ? iford_spi[n] : spi[n]));
        if (ur) {
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1,
                            iford ? iford_ur_base[n] : ur_base[n]);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                               ss_spi(gic, intc,
                                      iford ? iford_ur_spi[n] : ur_spi[n]));
        }
    }

    fsp = qdev_new(TYPE_SSTAR_FSP);
    if (sms->flash_file) {
        qdev_prop_set_string(fsp, "flash-file", sms->flash_file);
    }
    {
        /*
         * Without flash-jedec=, the board's part in the size of the image:
         * the same maker and type, the capacity byte log2 of the size
         * (W25Q128 0xEF4018 dumped as 8 MiB is a W25Q64, 0xEF4017).
         */
        uint32_t jedec = c->flash_jedec;

        if (sms->flash_jedec) {
            jedec = sms->flash_jedec;
        } else if (img_len && is_power_of_2(img_len)) {
            jedec = (jedec & ~0xFFu) | ctz64(img_len);
        }
        qdev_prop_set_uint32(fsp, "jedec", jedec);
    }
    qdev_prop_set_bit(fsp, "sw-cs", c->fsp_sw_cs);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(fsp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(fsp), 0, SS_ISP_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(fsp), 1, c->fsp_bank ?
                    SSTAR_RIU_ADDR(c->fsp_bank, 0) : SS_FSP_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(fsp), 2, c->qspi_bank ?
                    SSTAR_RIU_ADDR(c->qspi_bank, 0) : SS_QSPI_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(fsp), 3, SS_XIU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(fsp), 0, ss_spi(gic, intc, SS_SPI_FSP));

    /*
     * The infinity/infinity3 PM RTC, bank 0x0012 (DTS "mstar,infinity-rtc"
     * at 0x1F002400).  infinity2m and later have the RTCPWC instead.
     */
    if (c->die == SS_DIE_IFORD) {
        /*
         * The IPU's space (MI_IPU_Mapping_Riu_ChipTop_IPU_Space): SRAM and
         * registers, as plain storage.  A STUB: the driver can power it up
         * and clear it, and nothing behind it computes.
         */
        MemoryRegion *ipu = g_new(MemoryRegion, 1);

        memory_region_init_ram(ipu, NULL, "sigmastar.ipu-stub", SS_IPU_SIZE,
                               &error_fatal);
        memory_region_add_subregion(sysmem, SS_IPU_BASE, ipu);

        dev = qdev_new("sstar-rtcpwc");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x0034, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_IFORD_RTC));
    }
    if (c->die == SS_DIE_INFINITY3) {
        dev = qdev_new("sstar-rtc");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x0012, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_RTC));

        /* the scaler VIP's YUV gamma SRAM port, SCNR bank regs 0x40-0x4F */
        dev = qdev_new("sstar-vip-gamma");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1226, 0x40));
        /* and its RGB gamma SRAM ports, HVSP1 bank regs 0x7A-0x7F */
        dev = qdev_new("sstar-rgb-gamma");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1211, 0x7A));

        /*
         * The video input: the ISP top (bank 0x1302), its menuload (0x1306)
         * and the CSI MAC (0x1204) are the frame clock, which drives the
         * scaler's frame interrupts and DMA clients (SCL0 bank 0x1218),
         * and the pixel pipeline, which renders the sensor's picture (the
         * "sensor" link, set below) and hands it to the scaler.
         */
        {
            DeviceState *isp = qdev_new("sstar-i3-isp");

            i3_isp = isp;
            object_property_add_child(OBJECT(machine), "isp", OBJECT(isp));
            object_property_set_link(OBJECT(isp), "riu", OBJECT(riu),
                                     &error_fatal);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(isp), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(isp), 0,
                            SSTAR_RIU_ADDR(0x1302, 0));
            sysbus_mmio_map(SYS_BUS_DEVICE(isp), 1,
                            SSTAR_RIU_ADDR(0x1306, 0));
            sysbus_mmio_map(SYS_BUS_DEVICE(isp), 2,
                            SSTAR_RIU_ADDR(0x1204, 0));
            sysbus_connect_irq(SYS_BUS_DEVICE(isp), 0,
                               ss_spi(gic, intc, SS_SPI_I3_ISP));
            sysbus_connect_irq(SYS_BUS_DEVICE(isp), 1,
                               ss_spi(gic, intc, SS_SPI_I3_CSI));

            dev = qdev_new("sstar-i3-scl");
            object_property_set_link(OBJECT(dev), "riu", OBJECT(riu),
                                     &error_fatal);
            object_property_set_link(OBJECT(dev), "isp", OBJECT(isp),
                                     &error_fatal);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0,
                            SSTAR_RIU_ADDR(0x1218, 0));
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               ss_spi(gic, intc, SS_SPI_I3_SCL));
            qdev_connect_gpio_out(isp, 0, qdev_get_gpio_in(dev, 0));
            qdev_connect_gpio_out(isp, 1, qdev_get_gpio_in(dev, 1));
        }

        /*
         * The GPIO pads (PADGPIO 0x103C, PM GPIO 0x000F), the SAR ADC
         * (0x0014) and the PM-sleep interrupt controller (PMSLEEP 0x000E
         * regs 0x00-0x0F), cascaded into the main controller's MS_IRQ 2.
         */
        dev = qdev_new("sstar-i3-gpio");
        object_property_set_bool(OBJECT(dev), "night", sms->night,
                                 &error_fatal);
        /* PM_SD_CDZ reads "card in" when -drive if=sd gives one */
        object_property_set_bool(OBJECT(dev), "sd-card",
                                 drive_get(IF_SD, 0, 0) != NULL,
                                 &error_fatal);
        object_property_add_child(OBJECT(machine), "gpio", OBJECT(dev));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x000E, 0));
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, SSTAR_RIU_ADDR(0x000F, 0));
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, SSTAR_RIU_ADDR(0x103C, 0));
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 3, SSTAR_RIU_ADDR(0x0014, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_PM_SLEEP));
        sms->i3_gpio = dev;

        /*
         * The SD slot: SDIO0 (banks 0x1410-0x1411), an FCIE5 on its SDIO
         * port, with the card from -drive if=sd
         */
        {
            DriveInfo *di = drive_get(IF_SD, 0, 0);

            dev = sysbus_create_simple("sstar-fcie", SSTAR_RIU_ADDR(0x1410, 0),
                                       ss_spi(gic, intc, SS_SPI_I3_SDIO));
            if (di) {
                DeviceState *card = qdev_new(TYPE_SD_CARD);

                qdev_prop_set_drive_err(card, "drive",
                                        blk_by_legacy_dinfo(di), &error_fatal);
                qdev_realize_and_unref(card,
                                       qdev_get_child_bus(dev, "sd-bus"),
                                       &error_fatal);
            }
        }

        /*
         * Audio: BACH's DMAs (bank 0x1502) and its interrupt enable (bank
         * 0x1503); playback to the machine's audiodev, if it has one.
         */
        dev = qdev_new("sstar-i3-bach");
        if (machine->audiodev) {
            qdev_prop_set_string(dev, "audiodev", machine->audiodev);
        }
        object_property_add_child(OBJECT(machine), "audio", OBJECT(dev));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1502, 0));
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, SSTAR_RIU_ADDR(0x1503, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_AUDIO));

        /*
         * The scaler's command queue, one engine at bank 0x1120: it
         * applies scaling changes at a frame end while streaming.
         */
        dev = qdev_new("sstar-cmdq");
        qdev_prop_set_bit(dev, "infinity3", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1120, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_I3_CMDQ));

        /* the H.264 encoder, banks 0x1324-0x1325 */
        dev = qdev_new("sstar-mfe");
        qdev_prop_set_uint64(dev, "dram-size", machine->ram_size);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1F264800);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_I3_MFE));

        /* the JPEG encoder, bank 0x1320 */
        dev = qdev_new("sstar-i3-jpe");
        qdev_prop_set_uint64(dev, "dram-size", machine->ram_size);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1F264000);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_I3_JPE));

        /*
         * The H.265 encoder (Hantro H2 v3): its conf bank 0x1328 and the
         * Hantro registers, two RIU words each, in banks 0x1329-0x132B.
         */
        dev = qdev_new("sstar-vhe");
        qdev_prop_set_uint64(dev, "dram-size", machine->ram_size);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1F265000);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x1F265200);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_I3_VHE));
    }

    /* SHA and RSA (and the AES DMA's registers), bank 0x1122 */
    dev = qdev_new("sstar-crypto");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1122, 0));

    dev = qdev_new("sstar-bdma");
    object_property_set_link(OBJECT(dev), "fsp", OBJECT(fsp), &error_fatal);
    if (c->die == SS_DIE_IFORD) {
        DeviceState *xzd = qdev_new(TYPE_SSTAR_XZDEC);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(xzd), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(xzd), 0, SSTAR_RIU_ADDR(0x11D7, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(xzd), 0,
                           ss_spi(gic, intc, SS_SPI_IFORD_XZDEC));
        object_property_set_link(OBJECT(dev), "xzdec", OBJECT(xzd),
                                 &error_fatal);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SS_BDMA_BASE);
    {
        static const int spi[4] = {
            SS_SPI_BDMA0, SS_SPI_BDMA1, SS_SPI_BDMA2, SS_SPI_BDMA3,
        };

        /* On Ifado SPIs 93/94 are URDMA's; its DTS gives BDMA 72/73 */
        for (n = 0; n < (c->urdma ? 2 : 4); n++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), n,
                               ss_spi(gic, intc, spi[n]));
        }
    }

    /*
     * Ifado's video encoder, at its firmware interface: the RISC control
     * block at 0x1F34C800 and the encoder's at 0x1F34CC00 (sstar_mi.ko's
     * HalVencIos*; the DTS "venc" node, "mhe-irq").
     */
    if (c->die == SS_DIE_IFADO || c->die == SS_DIE_IFORD) {
        bool iford = c->die == SS_DIE_IFORD;

        dev = qdev_new("sstar-venc");
        qdev_prop_set_bit(dev, "iford", iford);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1F34C800);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, iford ? SS_SPI_IFORD_VENC
                                                   : SS_SPI_VENC));

        /* the ISP0 top, bank 0x1302: its interrupts and the frame clock */
        dev = qdev_new("sstar-isp");
        isp0 = dev;
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1302, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_ISP));

        /* VIF channels 0-3, bank 0x1308: their frame interrupts */
        dev = qdev_new("sstar-vif");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1308, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_VIF));

        /* the JPEG encoder, bank 0x1237 (the DTS "sstar,cedric-jpe") */
        dev = qdev_new("sstar-jpe");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1237, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_JPE));

        /* the audio DMAs, BACH bank 0x1502: the microphone's capture */
        dev = qdev_new("sstar-bach");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1502, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_AUDIO));

        /* the four CMDQ engines (sstar_mi.ko g_halRegCmdCtlBase) */
        dev = qdev_new("sstar-cmdq");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        {
            /* g_halRegCmdCtlBase; Iford has all eight */
            static const uint32_t bank[8] = {
                0x1120, 0x1123, 0x1124, 0x1125, 0x1126, 0x1127, 0x1128, 0x1129,
            };

            for (n = 0; n < (iford ? 8 : 4); n++) {
                sysbus_mmio_map(SYS_BUS_DEVICE(dev), n,
                                SSTAR_RIU_ADDR(bank[n], 0));
            }
        }
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_CMDQ));
    }

    /*
     * MIIC0-2 (Ifado DTS i2c@1f223000, @1f223200, @1f222e00); infinity3
     * has the first two, at the same addresses and SPIs.
     */
    if (c->miic) {
        static const hwaddr base[3] = { 0x1F223000, 0x1F223200, 0x1F222E00 };
        static const int spi[3] = {
            SS_SPI_MIIC0, SS_SPI_MIIC1, SS_SPI_MIIC2,
        };
        static const hwaddr iford_base[3] = {
            0x1F222800, 0x1F222A00, 0x1F007C00,
        };
        /* i2c0 and i2c1 share INT_IRQ_MIIC; the OR gate below is theirs */
        static const int iford_spi[3] = {
            SS_SPI_MIIC0, SS_SPI_MIIC0, SS_SPI_IFORD_PM_MIIC,
        };
        qemu_irq iford_miic = NULL;
        bool iford = c->die == SS_DIE_IFORD;
        const char *sensor = sms->sensor ? sms->sensor : c->default_sensor;
        I2CBus *bus[3];

        assert(c->miic <= 3 && c->miic_sensor_bus < c->miic);
        for (n = 0; n < c->miic; n++) {
            dev = qdev_new("sstar-i2c");
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0,
                            iford ? iford_base[n] : base[n]);
            if (iford && n < 2) {
                if (!iford_miic) {
                    DeviceState *or = qdev_new(TYPE_OR_IRQ);

                    qdev_prop_set_uint16(or, "num-lines", 2);
                    qdev_realize_and_unref(or, NULL, &error_fatal);
                    qdev_connect_gpio_out(or, 0, ss_spi(gic, intc,
                                                        iford_spi[n]));
                    iford_miic = qdev_get_gpio_in(or, 0);
                    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, iford_miic);
                    iford_miic = qdev_get_gpio_in(or, 1);
                } else {
                    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, iford_miic);
                }
            } else {
                sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                                   ss_spi(gic, intc,
                                          iford ? iford_spi[n] : spi[n]));
            }
            bus[n] = I2C_BUS(qdev_get_child_bus(dev, "i2c"));
        }
        /*
         * A board's battery RTC, on the first bus: the Raysharp app reads
         * a DS1307-family part at 0xD0 (8-bit), which the SoC has no twin of.
         */
        if (sms->rtc && strcmp(sms->rtc, "none")) {
            if (strcmp(sms->rtc, "ds1338")) {
                error_report("sigmastar: unknown rtc '%s' (none, ds1338)",
                             sms->rtc);
                exit(1);
            }
            i2c_slave_create_simple(bus[0], "ds1338", 0x68);
        }

        dev = image_sensor_create(bus[c->miic_sensor_bus], sensor);
        if (dev && iford) {
            DeviceState *scl = qdev_new("sstar-iford-scl");
            DeviceState *pipe = qdev_new("sstar-iford-isp");

            object_property_set_link(OBJECT(pipe), "sensor", OBJECT(dev),
                                     &error_fatal);
            object_property_set_link(OBJECT(pipe), "riu", OBJECT(riu),
                                     &error_fatal);
            object_property_add_child(OBJECT(machine), "isp", OBJECT(pipe));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(pipe), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(pipe), 0,
                            SSTAR_RIU_ADDR(0x1358, 0));
            sysbus_mmio_map(SYS_BUS_DEVICE(pipe), 1,
                            SSTAR_RIU_ADDR(0x130C, 0));
            sysbus_connect_irq(SYS_BUS_DEVICE(pipe), 0,
                               ss_spi(gic, intc, SS_SPI_IFORD_ISP_DMA));
            qdev_connect_gpio_out_named(isp0, "frame", 0,
                                        qdev_get_gpio_in(pipe, 0));
            object_property_set_link(OBJECT(scl), "isp", OBJECT(pipe),
                                     &error_fatal);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(scl), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(scl), 0,
                            SSTAR_RIU_ADDR(0x1253, 0));
        }
        /* the infinity3 ISP renders its picture from the sensor's */
        if (dev && i3_isp) {
            object_property_set_link(OBJECT(i3_isp), "sensor", OBJECT(dev),
                                     &error_fatal);
        }
    } else if (sms->sensor && strcmp(sms->sensor, "none")) {
        error_report("sigmastar: this machine has no sensor bus");
        exit(1);
    } else if (sms->rtc && strcmp(sms->rtc, "none")) {
        error_report("sigmastar: this machine has no I2C bus for an rtc");
        exit(1);
    }

    /*
     * EMAC: RIU view, X32 view (vendor DTS emac0); its internal PHY's
     * registers are in the RIU storage at 0x1F006200.
     */
    dev = qdev_new("sstar-emac");
    qemu_configure_nic_device(dev, true, NULL);
    if (c->die == SS_DIE_IFORD) {
        qdev_prop_set_uint32(dev, "phy-bank", 0x1518);  /* 0x1F2A3000 */
    }
    object_property_set_link(OBJECT(dev), "riu", OBJECT(riu), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1F2A2000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1,
                    c->die == SS_DIE_IFORD ? 0x1F304200 : 0x1F343C00);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       ss_spi(gic, intc, c->die == SS_DIE_IFORD ?
                              SS_SPI_IFORD_EMAC : SS_SPI_EMAC));

    /*
     * The bus masters' view of memory (sstar_ahci_sata_bus_address()):
     * MIU bus addresses below 512 MiB are DRAM; above, physical.
     */
    memory_region_init(&sms->miu_root, OBJECT(sms), "sigmastar.miu",
                       UINT64_MAX);
    memory_region_init_alias(&sms->miu_sys, OBJECT(sms), "sigmastar.miu-sys",
                             sysmem, 0, UINT64_MAX);
    memory_region_init_alias(&sms->miu_low, OBJECT(sms), "sigmastar.miu-dram",
                             sysmem, SS_DRAM_BASE, SS_DRAM_WINDOW);
    memory_region_add_subregion_overlap(&sms->miu_root, 0, &sms->miu_sys, 0);
    memory_region_add_subregion_overlap(&sms->miu_root, 0, &sms->miu_low, 1);
    /*
     * The SATA driver puts its PRD high words through the same
     * phys-to-bus subtraction: 0 becomes 0xE0000000.  The 32-bit MIU
     * ignores them; QEMU's AHCI does not, so the bus repeats there.
     */
    memory_region_init_alias(&sms->miu_hi, OBJECT(sms), "sigmastar.miu-hi",
                             &sms->miu_root, 0, 4 * GiB);
    memory_region_add_subregion(&sms->miu_root, 0xE0000000ULL << 32,
                                &sms->miu_hi);
    address_space_init(&sms->miu_as, &sms->miu_root, "sigmastar.miu");

    /*
     * The NVR die's SATA, display, video decoder and GE.  An IPC die has
     * other blocks at those addresses (Ifado's video encoder is at
     * 0x1F34D000, inside the VPU's window), so it gets none of them.
     */
    if (!c->ipc) {
        /*
         * SATA: a one-port AHCI at 0x1F345000 (HBA, port 0 at +0x100), its
         * PHY and misc banks (0x1525-0x1527) RIU storage.  The vendor driver
         * hands it MIU bus addresses.  The disk is -drive if=ide,index=0.
         */
        dev = qdev_new(TYPE_SYSBUS_AHCI);
        qdev_prop_set_uint32(dev, "num-ports", 1);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        SYSBUS_AHCI(dev)->ahci.as = &sms->miu_as;
        memory_region_init_alias(&sms->sata_win, OBJECT(sms), "sigmastar.sata",
                                 sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0),
                                 0, 0x200);
        memory_region_add_subregion(sysmem, SSTAR_RIU_ADDR(0x1A28, 0),
                                    &sms->sata_win);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_SATA));
        {
            DriveInfo *hd[1] = { drive_get(IF_IDE, 0, 0) };

            ahci_ide_create_devs(&SYSBUS_AHCI(dev)->ahci, hd);
        }

        /*
         * The display: GOP 1 (banks 0x1234/0x1235, the plane fbdev uses), the
         * display top (frame timing, vsync) and the MOP video plane, scanned
         * out to a graphic console.
         */
        dev = qdev_new("mstar-disp");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1234, 0));
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, SSTAR_RIU_ADDR(0x1128, 0));
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, SSTAR_RIU_ADDR(0x1405, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_DISP_TOP));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                           ss_spi(gic, intc, SS_SPI_GOP_VSYNC));

        /*
         * The video decoder, a Chips&Media WAVE5 VPU (mi_vdec: ioremap of
         * 0x1F344000, registers from +0x800).  The vendor driver reaches WAVE5
         * offsets up to the backbone at 0xFE00, through the kernel's static
         * map of the RIU, so the window spans 64 KiB: under every other model,
         * over the RIU's storage.
         */
        dev = qdev_new("sstar-vpu");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, 0x1F344800, -1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_VPU));

        /* GE 2D engine: MI_GFX composites the GUI through it */
        dev = qdev_new("mstar-ge");
        qdev_prop_set_uint32(dev, "dram-base", SS_DRAM_BASE);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1409, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, SS_SPI_GE));
    }

    /*
     * The three EHCI hosts (UHC0-2, DTS Sstar-ehci-1..3), each beside its
     * UTMI PHY and USBC wrapper banks (ehci-mstar-chip.h).  -device
     * usb-tablet / usb-kbd / usb-storage attach as USB 2.0 devices.
     */
    for (n = 0; n < (c->die == SS_DIE_IFORD ? 1 : 3); n++) {
        static const uint32_t bank[3] = { 0x1424, 0x1428, 0x1432 };
        static const int spi[3] = { SS_SPI_UHC0, SS_SPI_UHC1, SS_SPI_UHC2 };

        dev = qdev_new("sstar-uhc");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(bank[n], 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           ss_spi(gic, intc, c->die == SS_DIE_IFORD ?
                                  SS_SPI_IFORD_UHC : spi[n]));
    }

    /* Iford's one SD slot: FCIE at bank 0x1413, CIFD above (DTS sdmmc0) */
    if (c->die == SS_DIE_IFORD) {
        DriveInfo *di = drive_get(IF_SD, 0, 0);

        dev = sysbus_create_simple("sstar-fcie", SSTAR_RIU_ADDR(0x1413, 0),
                                   ss_spi(gic, intc, SS_SPI_IFORD_SD));
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    memory_region_init_io(&sms->smpctrl, OBJECT(sms), &ss_smpctrl_ops, sms,
                          "sigmastar.smpctrl", 0x200);
    memory_region_add_subregion(sysmem, SS_SMPCTRL_BASE, &sms->smpctrl);
    qemu_register_reset(ss_smpctrl_reset, sms);

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = SS_DRAM_BASE;
    /*
     * The vendor kernel is entered by U-Boot in the Secure state and
     * does Secure-only setup (ACTLR.SMP, the L2 controls).
     */
    binfo.secure_boot = true;
    binfo.write_secondary_boot = ss_write_secondary_boot;
    binfo.secondary_cpu_reset_hook = ss_reset_secondary;
    arm_load_kernel(sms->cpus[0], machine, &binfo);

    if (machine->kernel_filename) {
        return;
    }
    if (machine->firmware) {
        /* A real mask ROM dump: it loads the IPL itself, through BDMA. */
        if (load_image_targphys(machine->firmware, SS_BOOTROM_BASE,
                                SS_BOOTROM_SIZE, NULL) < 0) {
            error_report("sigmastar: cannot load ROM %s", machine->firmware);
            exit(1);
        }
    } else if (img && !ss_boot_rom_setup(img, img_len, sms->flash_file,
                                         c->ipl_offset,
                                         c->die == SS_DIE_IFORD)) {
        exit(1);
    }
}

/* The RTCPWC's alarm ends a power-down: the CPUs start over, DRAM stays. */
static void ss_machine_wakeup(MachineState *machine)
{
    qemu_system_reset(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void ss_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "sigmastar.ram";
    mc->wakeup = ss_machine_wakeup;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");
    /*
     * 4 KiB target pages, as on virt.  With QEMU's default 1 KiB pages a
     * region starting inside one 1 KiB page and covering the next (the
     * MOP at 0x1F280A00, the EMAC PHY bank at 0x1F006200) is mapped with a
     * misaligned offset that corrupts its TLB entry ("iotlb_to_section:
     * Assertion `section_index < d->map.sections_nb'").  The RIU's blocks
     * are 512-byte aligned; none spans a whole 4 KiB page from inside one.
     */
    mc->minimum_page_bits = 12;
    object_class_property_add_str(oc, "flash-file",
        ss_machine_get_flash_file, ss_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as dumped from the board (IPL at 0, at 0x4000 on "
        "infinity3); erases and programs are written back to it");
    object_class_property_add(oc, "flash-jedec", "uint32",
        ss_machine_get_flash_jedec, ss_machine_set_flash_jedec, NULL, NULL);
    object_class_property_set_description(oc, "flash-jedec",
        "JEDEC ID the SPI NOR answers with (e.g. 0xEF4018); by default "
        "the board's part, its capacity byte taken from the image size");
    object_class_property_add_str(oc, "rtc",
        ss_machine_get_rtc, ss_machine_set_rtc);
    object_class_property_set_description(oc, "rtc",
        "Battery RTC on the first I2C bus: ds1338 (a DS1307 at 0x68), "
        "or none, the default");
    object_class_property_add_str(oc, "sensor",
        ss_machine_get_sensor, ss_machine_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "Image sensor on the sensor I2C bus, by its catalogue name "
        "(hw/i2c/image-sensor.c; ssc375: sc431hai, the "
        "default; msc313e: jxf22, the default; or none)");
    object_class_property_add_str(oc, "daynight",
        ss_machine_get_daynight, ss_machine_set_daynight);
    object_class_property_set_description(oc, "daynight",
        "msc313e: the light the camera sees, day (the default) or night: "
        "PM_IRIN's level and the light sensor on SAR channel 2; "
        "qom-set /machine daynight changes it while running");
    machine_add_audiodev_property(mc);
    object_class_property_add(oc, "uuid", "uint64",
        ss_machine_get_uuid, ss_machine_set_uuid, NULL, NULL);
    object_class_property_set_description(oc, "uuid",
        "The die's unique ID in OTP2 (msys_read_uuid), as read from the "
        "board; vendor licences are bound to it");
}

static const TypeInfo ss_machine_info = {
    .name           = TYPE_SS_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(SsMachineState),
    .class_init     = ss_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void ss_machine_register_types(void)
{
    type_register_static(&ss_machine_info);
}

type_init(ss_machine_register_types)

#define DEFINE_SS_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        sigmastar_common_init(machine, &config);                     \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_ram_size = config.ram_size_default;              \
        mc->max_cpus = config.num_cpus;                              \
        mc->default_cpus = config.num_cpus;                          \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, SS_MACHINE, SsMachineState,     \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_SS_MACHINE("ssr621q", ssr621q, ssr621q_soc)
DEFINE_SS_MACHINE("ssc375", ssc375, ssc375_soc)
DEFINE_SS_MACHINE("ssd202d", ssd202d, ssd202d_soc)
DEFINE_SS_MACHINE("ssd201", ssd201, ssd201_soc)
DEFINE_SS_MACHINE("msc313e", msc313e, msc313e_soc)
DEFINE_SS_MACHINE("ssc029", ssc029, iford_soc)
