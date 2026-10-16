# HiSilicon IP Camera SoC Landscape & QEMU Architecture Plan

## 1. SoC Family Classification

Based on cross-referencing OpenIPC firmware build matrix, ipctool chip detection,
U-Boot repositories, and SDK documentation, here is the definitive family tree:

### Generation Timeline & Hardware Summary

| Gen | ipctool ID | Family Name | SoC Members | CPU | Kernel | U-Boot | MPP | IRQ | UART0 Base |
|-----|-----------|-------------|-------------|-----|--------|--------|-----|-----|------------|
| V1 | `0x35180100` | hi3516cv100 | hi3516cv100, hi3518cv100, hi3518ev100, hi3518av100 | ARM926 540MHz | 3.0.8 | 2010.06 | v1 | VIC | 0x20080000 |
| V2 | `0x3518E200` | hi3516cv200 | hi3516cv200, hi3518ev200, hi3518ev201 | ARM926 800MHz | 4.9.37¹ | 2010.06 | v2 | VIC | 0x20080000 |
| V2A | `0x3516A100` | hi3516av100 | hi3516av100, hi3516dv100 | Cortex-A7 600MHz | 4.9.37¹ | 2010.06 | v2 | GIC | 0x20080000 |
| V3 | `0x3516C300` | hi3516cv300 | hi3516cv300, hi3516ev100 | ARM926 800MHz | 3.18.20 | 2010.06 | v3 | VIC | 0x12100000 |
| V3A | `0x35190101` | hi3519v101 | hi3519v101, hi3516av200, hi3559v100, hi3556v100 | A17+A7 big.LITTLE | 3.18.20 | 2016.11 | v3 | GIC | 0x12100000 |
| V4 | `0x3516E300` | hi3516ev200 | hi3516ev200, hi3516ev300, hi3518ev300, hi3516dv200 | Cortex-A7 900MHz | 4.9.37 | 2016.11 | v4 | GIC | 0x12040000 |
| V4A | `0x3516C500` | hi3516cv500 | hi3516cv500, hi3516av300, hi3516dv300 | Cortex-A7 900MHz (NNIE) | 4.9.37 | 2016.11 | v4 | GIC | 0x120a0000 |
| — | `0x3536C100` | hi3536cv100 | hi3536cv100 (NVR SoC) | Cortex-A7 | 3.18.20 | — | — | GIC | 0x12080000 |
| — | `0x3536D100` | hi3536dv100 | hi3536dv100 (NVR SoC) | Cortex-A7 | 4.9.37 | — | — | GIC | 0x12080000 |
| OT | `0x3516C610` | hi3516cv610 | hi3516cv610, hi3516cv608, hi3516cv613, hi3516dv500, hi3519dv500 | Cortex-A7? | 5.10? | 2021.04 | v5? | GIC | 0x11040000 |

¹ OpenIPC uses 4.9.37 for V2/V2A, but original HiSilicon SDK used 3.4.35

### Goke Equivalents (Silicon Clones)

Goke chips are die-identical HiSilicon V4 silicon with different branding:

| Goke Chip | HiSilicon Equivalent | ipctool Family ID |
|-----------|---------------------|-------------------|
| gk7205v200 | hi3516ev200 | `0x72050200` |
| gk7205v210 | (new, ~ev200) | `0x72050210` |
| gk7205v300 | hi3516ev300 | `0x72050300` |
| gk7202v300 | hi3518ev300 | `0x72020300` |
| gk7605v100 | hi3516dv200 | `0x76050100` |
| gk7201v200 | (~ev200 stripped) | `0x72010200` |
| gk7201v300 | (~ev200 stripped, re-ID of gk7201v200) | `0x72010300` |
| gk7202v330 | (~18ev300 variant) | `0x72020330` |
| gk7205v500 | (new gen) | `0x72050500` |
| gk7205v510 | (new gen) | `0x72050510` |
| gk7205v530 | (new gen) | `0x72050530` |

All Goke V4 chips share the **same memory map, peripherals, and drivers** as hi3516ev200 family.

---

## 2. Hardware Architecture Comparison (IP Blocks)

### Memory Map Eras

There are essentially **3 distinct address map generations**:

#### Era 1: "0x200xxxxx" (V1, V2, V2A)
```
SysCtrl:    0x20050000
CRG:        0x20030000
UART0:      0x20080000
Timer:      0x20000000
VIC/GIC:    0x10040000 (V1/V2 VIC) or GIC (V2A)
FMC:        0x10000000 / 0x14000000 (or SPI flash via SFC)
GPIO:       0x20140000+
DMA:        0x20030000 (PL080)
RAM:        0x80000000
SRAM:       0x04010000
FEMAC:      0x10050000 (V1/V2 only, V2A has GMAC)
```

#### Era 2: "0x120xxxxx" (V3, V3A)
```
SysCtrl:    0x12020000
CRG:        0x12010000
UART0:      0x12100000 (V3) / 0x12100000 (V3A)
Timer:      0x12000000
VIC/GIC:    0x10040000 (V3 VIC) or GIC (V3A)
FMC:        0x10000000 / 0x14000000
GPIO:       0x12140000+
DMA:        0x10030000 (PL080)
RAM:        0x80000000 (V3) / 0x40000000 (V3A)
SRAM:       0x04010000
FEMAC:      0x10050000
MMC:        0x100c0000+ (himciv200)
```

#### Era 3: "0x120xxxxx shifted" (V4, V4A)
```
SysCtrl:    0x12020000 (V4) / 0x12020000 (V4A)
CRG:        0x12010000
UART0:      0x12040000 (V4) / 0x120a0000 (V4A)
Timer:      0x12000000
GIC Dist:   0x10301000
GIC CPU:    0x10302000
FMC:        0x10000000 / 0x14000000
GPIO:       0x120b0000+ (V4) / 0x120d0000+ (V4A)
RAM:        0x40000000 (V4) / 0x40000000 (V4A)
SRAM:       0x04010000
FEMAC:      0x10040000 (V4) / none (V4A uses GMAC)
SDHCI:      0x10010000+ (V4) / 0x10010000+ (V4A)
```

### Common IP Blocks Across All Families

| IP Block | Implementation | Variations |
|----------|---------------|------------|
| UART | PL011 | All families, 2-3 ports, addresses vary |
| Timer | SP804 dual-timer | All families, 2 blocks (4 timers), freq varies |
| GPIO | PL061 | 8-12 ports, addresses vary per era |
| SPI | PL022 | 1-2 ports |
| DMA | PL080 (V1-V3) / HIDMA (V4+) | V4+ uses HiSilicon's own DMA |
| Flash | HiFMC V100 (V3+) / SFC (V1/V2) | SPI NOR + SPI NAND via same controller |
| IRQ | PL190 VIC (ARM926 SoCs) / GICv2 (A7+) | Split by CPU type |
| Ethernet | FEMAC (V1-V4) / GMAC (V2A, V4A) | FEMAC = fast 100Mbit, GMAC = gigabit |
| SD/MMC | himciv200 (V1-V3) / SDHCI (V4+) | Custom DW MMC vs standard SDHCI |
| Watchdog | SP805 | All families |
| I2C | HiSilicon custom | For sensor access |
| USB | EHCI/OHCI (V1-V3) / DWC3 (V4+) | |

### CPU & Interrupt Controller Matrix

| CPU Core | IRQ Controller | Families |
|----------|---------------|----------|
| ARM926EJ-S (ARMv5) | PL190 VIC (32 IRQs) | V1, V2, V3 |
| Cortex-A7 (ARMv7-A) | GICv2 | V2A, V4, V4A, 3536x |
| Cortex-A17+A7 (ARMv7-A) | GICv2 | V3A |

### V4 Intra-Family Differences (Media Pipeline, Not System Peripherals)

Within the V4 family (EV200, EV300, 18EV300, DV200), all system-level
peripherals (UARTs, timers, GIC, SPI, SDHCI, FMC, FEMAC) share identical
addresses and IRQs.  The differences are entirely in the media processing
pipeline and I/O sizing — relevant only if we ever emulate the ISP/VENC:

| Feature | EV200 | EV300 | 18EV300 | DV200 |
|---------|-------|-------|---------|-------|
| **Max video input** | 2304x1296 | 2688x1520 | same as EV200 | same as EV300 |
| **H.264/H.265 encode** | 2304x1296@20fps | 2688x1520@25fps | same as EV200 | same as EV300 |
| **ISP WDR** | 2F-WDR frame | 2F-WDR line+frame | same as EV200 | same as EV300 |
| **Audio** | mono | stereo | same as EV200 | same as EV300 |
| **LCD output** | no | 16-bit parallel | same as EV200 | same as EV300 |
| **eMMC bus width** | 4-bit only | 8-bit, HS400 | 4-bit only | 8-bit, HS400 |
| **MIPI Rx lanes** | 2 | 4 | 2 | 4 |
| **GPIO groups** | 8 | 10 | 8 | 10 |
| **Integrated FE PHY** | yes | yes | **no** | yes |
| **On-package DDR** | 512Mb DDR2 | 1Gb DDR3L | 512Mb DDR2 | up to 512MB DDR3 |
| **LSADC channels** | 2 | 4 | 2 | 4 |

Source: HiSilicon SDK `Hi3516EV200R001C01SPC012` difference documents:
- `Hi3516EV300与Hi3516EV200 开发包差异说明.pdf`
- `Hi3518EV300与Hi3516EV200 开发包差异说明.pdf`
- `Hi3516EV300与Hi3516DV200 开发包差异说明.pdf`

Goke equivalents (gk7205v200/v300, gk7202v300, gk7605v100) are
pin-compatible, register-compatible silicon — only the SoC ID at
SCSYSID (0x72xxxxxx) differs.

---

## 3. SDK & Software Stack

### U-Boot Repositories (OpenIPC)

| U-Boot Repo | Base Version | Covers Families |
|-------------|-------------|-----------------|
| u-boot-hi3516cv100 | 2010.06 | V1: hi3516cv100, hi3518cv100, hi3518ev100 |
| u-boot-hi3516av100 | 2010.06 | V2A: hi3516av100, hi3516dv100 |
| u-boot-hi3516cv200 | 2010.06 | V2: hi3516cv200, hi3518ev200 |
| u-boot-hi3516cv300 | 2010.06 | V3: hi3516cv300, hi3516ev100 |
| u-boot-hi3519v101 | 2016.11 | V3A: hi3519v101, hi3516av200 |
| u-boot-hi3516ev200 | 2016.11 | V4: hi3516ev200/ev300/dv200, hi3518ev300 |
| u-boot-hi3516cv500 | 2016.11 | V4A: hi3516cv500, hi3516av300, hi3516dv300 |
| u-boot-gk7205v200 | 2016.11 | Goke V4: all gk72xx/76xx chips |

Local repos:
- `~/git/u-boot` = U-Boot 2021.04-rc5 (mainline, for reference)
- `~/git/u-boot.ev200` = U-Boot 2016.11 (HiSilicon V4 vendor fork)

### Kernel Versions

| Family | Original SDK Kernel | OpenIPC Kernel |
|--------|-------------------|----------------|
| V1 (cv100) | 3.0.8 | 3.0.8 |
| V2 (cv200) | 3.4.35 | 4.9.37 |
| V2A (av100) | 3.4.35 | 4.9.37 |
| V3 (cv300) | 3.18.20 | 3.18.20 |
| V3A (3519v101) | 3.18.20 | 3.18.20 |
| V4 (ev200) | 4.9.37 | 4.9.37 |
| V4A (cv500) | 4.9.37 | 4.9.37 |

### MPP (Media Processing Platform) Versions

| MPP Version | Families | Key Capabilities |
|-------------|----------|-----------------|
| v1 | V1 | Basic H.264, 720p |
| v2 | V2, V2A | H.264, up to 5MP |
| v3 | V3, V3A | H.264/H.265, 1080p-12MP |
| v4 | V4, V4A | H.264/H.265, AI/NNIE, 3MP-8MP |

### Toolchains

| Toolchain | Families |
|-----------|----------|
| arm-hisiv300-linux (uclibc) | V1, V2 |
| arm-hisiv500-linux (glibc) | V1, V2, V2A |
| arm-hisiv510-linux (uclibc) | V3 |
| arm-hisiv600-linux (glibc) | V3, V3A |
| arm-himix100-linux | V4, V4A |
| arm-himix200-linux | V4A (glibc) |
| OpenIPC uses musl + GCC 12 | All (modern builds) |

---

## 4. QEMU Architecture Proposal

### Design Principles

1. **Single source tree** — all families in `qemu/hw/arm/hisilicon.c` + `hisilicon.h`
2. **Shared peripheral devices** — common IP blocks (FMC, FEMAC, sysctl, CRG) implemented once with parameterization
3. **Per-family machine definitions** — each `-machine` type wires peripherals at correct addresses
4. **Gradual addition** — only add families when users request them; start with existing V3+V4, extend on demand
5. **SoC ID driven** — the sysctl device returns the correct SCSYSID registers so U-Boot/kernel auto-detect the exact chip variant

### Proposed Machine Hierarchy

```
QEMU -machine options:
  hi3516cv300    (V3, ARM926+VIC)     ← EXISTING
  hi3516ev300    (V4, A7+GIC)         ← EXISTING
  hi3516ev200    (V4, A7+GIC)         ← future, trivial (same as ev300 with different SoC ID + minor diffs)
  hi3518ev300    (V4, A7+GIC)         ← future, trivial (ev200 with fewer peripherals)
  hi3516dv200    (V4, A7+GIC)         ← future, trivial
  gk7205v200    (V4, A7+GIC)          ← future, alias with goke SoC ID
  gk7205v300    (V4, A7+GIC)          ← future, alias with goke SoC ID
  hi3516cv200    (V2, ARM926+VIC)     ← future, needs Era 1 memory map
  hi3518ev200    (V2, ARM926+VIC)     ← future
  hi3516av100    (V2A, A7+GIC)        ← future, Era 1 memory map + GIC
  hi3516cv100    (V1, ARM926+VIC)     ← future, oldest era
  hi3516cv500    (V4A, A7+GIC)        ← future, Era 3 with GMAC instead of FEMAC
  hi3519v101     (V3A, A17+A7+GIC)    ← future, big.LITTLE
```

### Source File Organization

```
qemu/hw/arm/hisilicon.c          — Machine definitions (all families)
qemu/include/hw/arm/hisilicon.h  — Memory map constants (all families)
qemu/hw/misc/hisi-sysctl.c       — SysCtrl: SoC ID, misc ctrl, reset
qemu/hw/misc/hisi-crg.c          — CRG: clock/reset gate stub
qemu/hw/misc/hisi-fmc.c          — HiFMC V100 flash controller
qemu/hw/net/hisi-femac.c         — FEMAC 100M Ethernet + MDIO
qemu/hw/misc/hisi-himci.c        — himciv200 SD/MMC controller (V1-V3)
                                    Full DW MMC with IDMAC + ADMA3 DMA
                                    (V4+ uses built-in SDHCI)
```

### Proposed Future Files (add only when needed)

```
qemu/hw/net/hisi-gmac.c          — GMAC Gigabit Ethernet (for V2A, V4A)
qemu/hw/misc/hisi-wdt.c          — Watchdog (SP805 compatible, just wire it)
qemu/hw/misc/hisi-misc.c         — MISC_CTRL registers (temperature, die ID)
```

### Parameterization Strategy

The key to supporting all variants from shared code:

#### 1. SysCtrl Device — already parameterized by `soc-id` property
- Add SCSYSID0-3 register emulation returning correct chip family + variant bytes
- Current: returns family ID at offset 0xEE0
- Needed: return per-chip variant byte at SCSYSID0[31:24] for sub-model detection

#### 2. Machine Init — use a shared helper pattern
```c
/* Proposed refactoring pattern */
typedef struct HisiSoCConfig {
    const char      *name;
    const char      *cpu_type;        /* "arm926" or "cortex-a7" */
    uint32_t        soc_id;           /* SCSYSID family ID */
    uint8_t         soc_variant;      /* SCSYSID0[31:24] sub-model */
    hwaddr          ram_base;
    ram_addr_t      ram_size_default;
    hwaddr          sram_base;
    size_t          sram_size;

    /* Interrupt controller */
    enum { HISI_IC_VIC, HISI_IC_GIC } irq_type;
    hwaddr          vic_base;         /* for VIC */
    hwaddr          gic_dist_base;    /* for GIC */
    hwaddr          gic_cpu_base;
    int             gic_num_spi;

    /* Peripherals */
    hwaddr          sysctl_base;
    hwaddr          crg_base;
    hwaddr          uart_bases[3];
    int             uart_irqs[3];
    hwaddr          timer_bases[2];
    int             timer_irqs[2];
    uint32_t        timer_freq;
    hwaddr          fmc_ctrl_base;
    hwaddr          fmc_mem_base;
    hwaddr          gpio_base;
    int             gpio_count;
    int             gpio_irq_start;   /* shared IRQ for VIC, per-port for GIC */
    hwaddr          spi_bases[2];
    int             spi_irqs[2];
    hwaddr          dma_base;
    int             dma_irq;

    /* Ethernet */
    enum { HISI_ETH_NONE, HISI_ETH_FEMAC, HISI_ETH_GMAC } eth_type;
    hwaddr          eth_base;
    int             eth_irq;

    /* SD/MMC */
    enum { HISI_MMC_NONE, HISI_MMC_HIMCI, HISI_MMC_SDHCI } mmc_type;
    hwaddr          mmc_bases[3];
    int             mmc_count;
} HisiSoCConfig;

/* One shared init function driven by config struct */
static void hisilicon_common_init(MachineState *machine, const HisiSoCConfig *cfg);
```

#### 3. Adding a New V4 Variant (Example: hi3516ev200)
```c
/* Trivial — just a new config struct + machine registration */
static const HisiSoCConfig hi3516ev200_config = {
    .name           = "hi3516ev200",
    .cpu_type       = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id         = 0x3516E200,
    .soc_variant    = 0,
    .ram_base       = 0x40000000,
    .ram_size_default = 64 * MiB,
    /* ... same as ev300 for everything except soc_id ... */
};
DEFINE_MACHINE_ARM("hi3516ev200", hi3516ev200_class_init)
```

#### 4. Adding Goke Variants
Goke chips are identical hardware with different SoC IDs. The sysctl device
already supports arbitrary soc-id values. A Goke machine is literally:
```c
static const HisiSoCConfig gk7205v300_config = {
    /* Copy hi3516ev300_config, change soc_id to 0x72050300 */
};
```

### Migration Path (Current → Table-Driven)

1. **Phase 0 (current)**: Two hand-coded machines (cv300, ev300) — working now
2. **Phase 1**: Extract `HisiSoCConfig` struct, refactor cv300+ev300 to use it
3. **Phase 2**: Add V4 variants (ev200, 18ev300, dv200) — trivial config changes
4. **Phase 3**: Add Goke aliases (gk7205v200/v300, gk7202v300, gk7605v100)
5. **Phase 4**: Add V2 family (cv200, 18ev200) — needs Era 1 address map
6. **Phase 5**: Add V2A family (av100, dv100) — Era 1 + GIC
7. **Phase 6**: Add V1 family (cv100, 18cv100, 18ev100) — oldest, lowest priority
8. **Phase 7**: Add V4A family (cv500, av300, dv300) — needs GMAC device
9. **Phase 8**: Add V3A family (3519v101, av200) — needs big.LITTLE

### What Each Phase Requires

| Phase | New IP Blocks Needed | Effort |
|-------|---------------------|--------|
| 1 (refactor) | None — just restructure | Medium |
| 2 (V4 variants) | None — config-only | Trivial |
| 3 (Goke) | None — config-only | Trivial |
| 4 (V2) | SFC flash controller (replaces HiFMC) | Medium |
| 5 (V2A) | GMAC Ethernet device | Medium-High |
| 6 (V1) | SFC + oldest mem map | Medium |
| 7 (V4A) | GMAC (reuse from Phase 5) | Low |
| 8 (V3A) | big.LITTLE CPU setup | High |

---

## 5. Key Findings & Observations

### SoC ID Detection Mechanism
- All HiSilicon SoCs have a System Controller with SCSYSID registers at offset 0xEE0
- The SC_CTRL base address varies by era:
  - V1/V2/V2A: 0x20050000
  - V3/V3A/V4/V4A: 0x12020000
  - OT (V5): 0x11020000
- Family ID is constructed from SCSYSID0-3 registers (4 bytes, one per register)
- Sub-model variant is in SCSYSID0[31:24]

### Goke Is Just Rebranded HiSilicon
- Goke V4 chips are pin-compatible, register-compatible silicon
- Same drivers, same memory maps, same everything
- Only difference: SoC ID register returns 0x72xxxxxx instead of 0x3516xxxx
- OpenIPC firmware build system treats them as a separate vendor but same family

### OpenIPC Firmware Grouping Matches Hardware Reality
The build matrix groups (comments in build.yml) align perfectly with:
- U-Boot repositories (one per group)
- Shared kernel configs (one per SOC_FAMILY)
- ipctool chip_generation values

### NVR SoCs (3536cv100, 3536dv100, 3520dv200)
These are video recorder chips, not cameras. Different use case but similar
IP blocks. Low priority for emulation unless specifically requested.

### Xiongmai Is Not HiSilicon
XM510/XM530/XM550 kernels started from HiSilicon BSP templates, but the
silicon is not a clone the way Goke is:
- **Memory map:** Xiongmai's own. The VIC is at 0x20010000 and the peripherals
  sit at 0x100x0000 and 0x200x0000.
- **Flash controller:** xmsfc, not HiFMC.
- **Ethernet:** a Synopsys DWMAC with its MAC block rearranged, not FEMAC.
- **GPIO:** one register per pin.

That is why they are a separate family: `hw/arm/xiongmai.c` with
`XmSoCConfig` (`include/hw/arm/xiongmai.h`), rather than more `HisiSoCConfig`
slots that would sit empty. The generic ARM IP comes from upstream QEMU, as it
does for HiSilicon. `hisi-regbank` is reused for unmodelled blocks. The
hardware reference with kernel-source citations is `docs/xiongmai.md`.
