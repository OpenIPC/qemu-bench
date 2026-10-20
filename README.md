# QEMU HiSilicon SoC Emulator (IPC + DVR/NVR + STB)

QEMU machine definitions for the HiSilicon surveillance and STB SoC lineup,
targeting QEMU v10.2.0. Boots unmodified [OpenIPC](https://openipc.org/) firmware
and vendor SDK kernels to a full Linux userspace on all supported platforms.

Builds two QEMU targets:
- `qemu-system-arm` — 31 IPC + 11 DVR/NVR + 1 STB (ARMv7-A) + 3 Xiongmai + 4 Goke GK710x/GK710xS (ARM1176) + 6 Anyka AK39xx (ARM926) + 3 SigmaStar (Cortex-A7)
- `qemu-system-aarch64` — 3 IPC + 1 STB (ARMv8-A, Cortex-A53/A55)

## Supported Machines (61 total)

### IPC family — V1 through V5 (`qemu-system-arm`)

| Machine | Generation | CPU | IRQ | Kernel | Boot tested |
|---------|-----------|-----|-----|--------|-------------|
| `hi3516cv100` | V1 | ARM926EJ-S | VIC | 3.0.8 | yes |
| `hi3518av100` | V1 | ARM926EJ-S | VIC | 3.0.8 | yes |
| `hi3518cv100` | V1 | ARM926EJ-S | VIC | 3.0.8 | yes |
| `hi3518ev100` | V1 | ARM926EJ-S | VIC | 3.0.8 | yes |
| `hi3516cv200` | V2 | ARM926EJ-S | VIC | 4.9.37 | yes |
| `hi3516av100` | V2A | Cortex-A7 | GIC | 4.9.37 | yes |
| `hi3516dv100` | V2A | Cortex-A7 | GIC | 4.9.37 | yes |
| `hi3516cv300` | V3 | ARM926EJ-S | VIC | 3.18.20 | yes |
| `hi3516cv500` | V3.5 | Cortex-A7 ×2 | GIC | 4.9.37 | yes |
| `hi3516dv300` | V4A | Cortex-A7 ×2 | GIC | 4.9.37 | yes |
| `hi3516av300` | V4A | Cortex-A7 ×2 | GIC | 4.9.37 | yes |
| `hi3519v101` | V3A | Cortex-A7 | GIC | 3.18.20 | yes |
| `hi3516av200` | V3A | Cortex-A7 | GIC | 3.18.20 | yes |
| `hi3516ev300` | V4 | Cortex-A7 | GIC | 4.9.37 | yes |
| `hi3516ev200` | V4 | Cortex-A7 | GIC | — | — |
| `hi3518ev300` | V4 | Cortex-A7 | GIC | — | — |
| `hi3516dv200` | V4 | Cortex-A7 | GIC | — | — |
| `gk7205v200` | V4/Goke | Cortex-A7 | GIC | — | — |
| `gk7201v200` | V4/Goke | Cortex-A7 | GIC | — | — |
| `gk7205v300` | V4/Goke | Cortex-A7 | GIC | — | — |
| `gk7202v300` | V4/Goke | Cortex-A7 | GIC | — | — |
| `gk7605v100` | V4/Goke | Cortex-A7 | GIC | — | — |
| `gk7205v500` | V4/Goke V500 | Cortex-A7 | GIC | 4.9.37 | yes |
| `gk7205v510` | V4/Goke V500 | Cortex-A7 | GIC | 4.9.37 | yes |
| `gk7205v530` | V4/Goke V500 | Cortex-A7 | GIC | — | —¹ |
| `gk7202v330` | V4/Goke V500 | Cortex-A7 | GIC | — | —¹ |
| `gk7206v10` | Goke/XMedia | Cortex-A7 MP2 | GIC | 5.10 | yes² |
| `gk7206v12a` | Goke/XMedia | Cortex-A7 MP2 | GIC | 5.10 | yes² |
| `hi3516cv608` | V5 | Cortex-A7 MP2 | GIC | 5.10 | — |
| `hi3516cv610` | **V5** | Cortex-A7 MP2 | GIC | **5.10** | yes |
| `hi3516cv613` | V5 | Cortex-A7 MP2 | GIC | 5.10 | — |

¹ No OpenIPC release exists for these two yet, so there is nothing to boot
them with; they build and register. The four Goke V500 parts share the
`xmedia,sp804` timer wiring (`hisi-xmsp804.c`) that the V500 kernel needs.

² `gk7206v10` and `gk7206v12a` (codename "xmorca"; same die, 64 vs 256 MiB
embedded DDR) boot the **vendor** XMedia SDK kernel
(linux-5.10.y, `xmorca_defconfig`) and busybox userspace, not OpenIPC — via
`qemu-boot/run-gk7206.sh` for an initramfs boot or `FLASH=1` for a SPI-NOR
boot with a jffs2 root. No vendor artifacts are committed here; see the script
header for how to build them from the SDK.

The three dual-core Cortex-A7 parts (`hi3516cv500`, `hi3516dv300`,
`hi3516av300`) boot both cores with `-smp 2`, from `-kernel` and from a
`flash-file` image alike. As on silicon, the kernel writes a reset trampoline
at physical 0 and clears CPU1's soft-reset bit in CRG `REG_CPU_SRST_CRG`
(0x78), and that bit drives CPU1's power state. The default is still one CPU.
The other "MP2" parts leave CPU1 offline: their vendor release methods are not
modelled yet.

### DVR/NVR family — surveillance back-end SoCs (`qemu-system-arm`)

| Machine | CPU | Kernel | Boot tested | Notes |
|---------|-----|--------|-------------|-------|
| `hi3520dv200` | Cortex-A9 | 4.9.37 backported from vendor 3.0 | yes | V1-era 0x20xxxxxx layout, 2013 legacy DVR |
| `hi3520dv300` | Cortex-A7 | 4.9.37 (Hi3521A artifact reuse) | yes | shares Hi3521A SDK base per LKML |
| `hi3520dv400` | Cortex-A7 | 3.18.20 vendor | yes | H.265 single, mobile DVR |
| `hi3521a` | Cortex-A7 | 4.9.37 from RichStrong | yes | hybrid DVR/NVR + NVP6124B I2C-stub |
| `hi3521dv100` | Cortex-A7 | 3.18.20 vendor | yes | H.265 dual-A7 sibling of Hi3521A |
| `hi3531a` | Cortex-A9 | 4.9.37 from RichStrong | yes | first A9 SMP-class (a9mpcore_priv) + XHCI |
| `hi3531dv100` | Cortex-A9 | 3.18.20 vendor | yes | H.265 sibling of Hi3531A |
| `hi3535` | Cortex-A9 | 4.9.37 (Hi3531A artifact reuse) | yes | A9 dual NVR-only, 2× GbE stub |
| `hi3536` | Cortex-A17 (a15) | 4.9.37 backported from vendor 3.10 | yes | flagship — quad A17 + A7 video coproc |
| `hi3536cv100` | Cortex-A7 | 3.18.20 vendor | yes | A7 dual entry-4K NVR |
| `hi3536dv100` | Cortex-A7 | 4.9.37 from RichStrong | yes | first DVR/NVR-class machine added |

### STB family

| Machine | Target | CPU | Kernel | Boot tested |
|---------|--------|-----|--------|-------------|
| `hi3796mv100` | `qemu-system-arm` | Cortex-A7 quad | 4.9.37 backported from vendor 3.10 (HiSTB) | yes |
| `hi3798cv200` | `qemu-system-aarch64` | **Cortex-A53 quad** (ARMv8) | mainline Linux 7.1 + Poplar DT | yes |

### Xiongmai family (`qemu-system-arm`)

| Machine | CPU | IRQ | Kernel | Boot tested |
|---------|-----|-----|--------|-------------|
| `xm510` | Cortex-A5 (A9 stand-in) | PL190 VIC | 3.0.101 vendor | yes |
| `xm530` | Cortex-A5 ×2 (A9 stand-in) | GIC | 3.10.103 vendor | yes |
| `xm550` | Cortex-A5 ×2 (A9 stand-in), 128 MB | GIC | 3.10.103 vendor | yes |

Not HiSilicon parts, although their BSP kernels started from HiSilicon
templates: the memory map and the flash controller (xmsfc) are Xiongmai's own,
the rest is licensed IP (PrimeCell PL011/SP804/SP805/PL190, Synopsys DWMAC,
Arasan SDHCI). So the family has its own table and init in
`hw/arm/xiongmai.c` rather than `HisiSoCConfig` slots; addresses and their
sources are in [docs/xiongmai.md](docs/xiongmai.md). `xm510` boots the OpenIPC
`xm510_lite` release with `qemu-boot/run-xm510.sh`, and boots a full camera
flash dump through its own U-Boot with `-M xm510,flash-file=dump.bin`.

CI covers:
- login;
- guest reboot;
- the network: DHCP, ICMP, SSH and routed DNS;
- SPI-NOR erase/program;
- the firmware's flash paths, each run by the firmware's own SD-card hook
  and checked on the host:
  - `sysupgrade --archive` (`test-sysupgrade.sh`);
  - `firstboot` (`test-firstboot.sh`);
  - init keeping an overlay this kernel cannot mount (`test-overlay-keep.sh`,
    firmware#2613).

`xm530` and `xm550` boot the `xm530_lite` release with `-smp 2`: CPU1 is
released through `REG_SC_REMAP` and `REG_CPU1_SOFT_RST` as on silicon.
An XM550 camera's full flash dump boots through its own U-Boot to `login:`
with `-M xm550,flash-file=dump.bin -smp 2`.

CI covers login, both CPUs, guest reboot and clock on both. On `xm530` it
also runs the network check and the flash tests: erase/program,
sysupgrade and firstboot.

Boot them with `mem=` as the **first** kernel argument, as OpenIPC's U-Boot
does (`mem=32M`). `load_xiongmai` sizes MMZ from the first argument. If it
is missing, MMZ covers all of RAM, kernel included. majestic then Oopses
in `mmz` and never stops the `xm_wdt` the script started, and the guest
resets a minute later.

### Goke GK710x family (`qemu-system-arm`)

| Machine | CPU | IRQ | Kernel | Boot tested |
|---------|-----|-----|--------|-------------|
| `gk7101` | ARM1176, 128 MB | 2× VIC (PL190-shaped) | 3.4.43 vendor | —¹ |
| `gk7102` | ARM1176, 64 MB in package | 2× VIC (PL190-shaped) | 3.4.43 vendor | yes |
| `gk7101s` | ARM1176, 128 MB | 2× VIC | 3.4.43 vendor | —¹ |
| `gk7102s` | ARM1176, 64 MB (GK7102S, GK7102C) | 2× VIC | 3.4.43 vendor | yes |

Goke's own first SoC, unrelated to the HiSilicon-derived GK7205 line above:
Ambarella A5s IP (VIC, timers, watchdog, IDC I2C), a 16550, a Synopsys GMAC
and Goke's register-driven SPI-NOR controller, modelled in
`hw/arm/goke.c`. The vendor U-Boot and kernel reach the hardware only
through a HAL blob that relocates, reorders and bit-scrambles every register;
the models are of the silicon underneath, mapped by executing that HAL
(`qemu-boot/gk710x-halmap.py`). Addresses, the boot header and the hand-off
are in [docs/goke-gk710x.md](docs/goke-gk710x.md).

A camera's full SPI-NOR dump boots through its own U-Boot and kernel to the
vendor application and a shell, network included:
`-M gk7102,flash-file=dump.bin` (or `FLASH=dump.bin
qemu-boot/run-gk7102.sh`). The OpenIPC `gk7102` release boots to `login:`
with `-kernel`/`-initrd` given a vendor HAL to load —
`-M gk7102,hal-file=<vendor U-Boot or dump>`, see
`qemu-boot/run-gk7102.sh` — since the kernel calls into it. `sensor=jxh42`
(or `jxh62`, `f37`) puts an SOI sensor on I2C 0.

`gk7101s`/`gk7102s` are the later S die (the GK7102C cameras are this
family): the same IP at the drivers' own 0x9000_0000 / 0xA000_0000 map and
register layouts, which the vendor kernel uses directly. They reuse the
GK710x models behind `gk-remap`, plus a DMA controller the GK7102C U-Boot
tests at start. A GK7102C camera dump boots through its U-Boot and kernel
to the vendor application and `login:` with
`-M gk7102s,flash-file=<image in chip byte order>`
(`qemu-boot/run-gk7102s.sh` converts a software dump).

¹ `gk7101`/`gk7101s` are the same dies and models with 128 MiB; no image of
either was at hand.

### Anyka AK39xx family (`qemu-system-arm`)

| Machine | CPU | IRQ | Kernel | Boot tested |
|---------|-----|-----|--------|-------------|
| `ak3918ev200` | ARM926EJ-S, 64 MB in package | SYSCTRL (2-level) | 3.4.35 vendor | yes |
| `ak3916ev300` | ARM926EJ-S, 64 MB in package | SYSCTRL (2-level) | 3.4.35 vendor | yes¹ |
| `ak3918ev300` | ARM926EJ-S, 64 MB in package | SYSCTRL (2-level) | 3.4.35 vendor | yes |
| `ak3919ev300` | ARM926EJ-S, 128 MB in package | SYSCTRL (2-level) | 3.4.35 vendor | —² |
| `ak3918ev330` | ARM926EJ-S, 64 MB in package | SYSCTRL (2-level) | 4.4.192 vendor (DT) | yes |
| `ak3919ev330` | ARM926EJ-S, 128 MB in package | SYSCTRL (2-level) | 4.4.192 vendor (DT) | yes |

Anyka's camera SoCs, unrelated to HiSilicon: one memory map from the
AK3918EV200 through AnyCloud39EV300 to AnyCloud V500 (EV330), in
`hw/arm/anyka.c`. The UART and the SPI master move their data through the
L2 buffer SRAM rather than FIFOs, which `hw/misc/ak-l2.c` models with its
block handshakes and DMA. A camera's SPI-NOR dump boots the way the mask ROM
does -- boot-header register script, then U-Boot (EV200 / EV300) or the
EV330's L2-resident first stage -- through the vendor U-Boot and kernel to
the vendor application and `login:`: `-M ak3918ev300,flash-file=dump.bin`.
Flash writes go back to the file. Ethernet (the Synopsys GMAC, on
`xm-dwmac`) works from both the vendor U-Boot and kernels: `-nic user`.
The SoC list from the SDKs, the map, the boot header and the dumps tried
are in [docs/anyka.md](docs/anyka.md).

¹ An AK3916EV301 dump.  ² Same die and models as a tested sibling; no image
was at hand.

### SigmaStar / MStar family (`qemu-system-arm`)

| Machine | CPU | IRQ | Kernel | Boot tested |
|---------|-----|-----|--------|-------------|
| `ssr621q` | Cortex-A7 ×2, 256 MB in package | GIC + main-intc | 4.9.84 vendor | yes |
| `ssd202d` | Cortex-A7 ×2, 128 MB in package | GIC + main-intc | — | —¹ |
| `ssd201` | Cortex-A7 ×2, 64 MB in package | GIC + main-intc | — | —¹ |

SigmaStar's (formerly MStar's) infinity2m dies, in `hw/arm/sigmastar.c`:
every peripheral behind the RIU's 16-bit register banks at `0x1F000000`. An
SSR621Q NVR's SPI-NOR dump boots the way the mask ROM does -- the board's own
IPL into IMI, `IPL_CUST`, U-Boot, the kernel from the romfs squashfs -- to
Xiongmai's Sofia GUI on the emulated HDMI output, with the mouse and keyboard
on USB and the HDD on SATA:
`FLASH=nvr.bin HDD=disk.img qemu-boot/run-ssr621q.sh`. Ethernet works from
both U-Boot and the kernel. The display and 2D engine models are ported from
Daniel Palmer's MStar QEMU work ([fifteenhex/qemu](https://github.com/fifteenhex/qemu)),
as are the SSD202D register findings the boot chain relies on. Map, boot,
devices and the Xiongmai `xmuart` console switch are in
[docs/sigmastar.md](docs/sigmastar.md).

¹ Same die and models as `ssr621q`; no image was at hand.

### IPC family — V5 aarch64 (`qemu-system-aarch64`)

| Machine | Generation | CPU | IRQ | Boot tested |
|---------|-----------|-----|-----|-------------|
| `hi3519dv500` | V5 (HISI_OT) | **Cortex-A55** (ARMv8) | GIC | U-Boot¹ |
| `hi3516dv500` | V5 (HISI_OT) | **Cortex-A55** (ARMv8) | GIC | U-Boot¹ |
| `gk7606v1` | Goke/XMedia | **Cortex-A55** (ARMv8) | GICv3 | no² |

The two Hi35xxDV500 parts share the V5/SS626 0x11xxxxxx peripheral map with
CV610 but are 64-bit and boot the gzip self-extracting vendor U-Boot
(`u-boot-z.bin`) via the on-die HW gzip engine — run locally with
`qemu-boot/run-hi3519dv500.sh <u-boot-z.bin>`.

¹ Boot-tested by OpenIPC/u-boot-hi3519dv500's `qemu_smoke` CI, which builds the
U-Boot image fresh and boots it on this machine (so no vendor binary is vendored
here). This repo's CI only checks the machines register.

² `gk7606v1` (codename "xmfalcon", the Cortex-A55 sibling of GK7206) is a
placeholder: it builds and registers, but does not boot yet. Much of its config
is inherited from GK7206 and several values — DDR size among them — are still
unverified against the vendor SDK.

All 46 machines build and register. Every machine marked "yes" above boots
OpenIPC or vendor firmware to a shell prompt (artifacts/scripts staged in
`qemu-boot/run-<machine>.sh`); `-machine help` on either binary is the
authoritative list.

### V5 Model Suffix → Chip ID Mapping

From datasheet Section 1.2.14 and lab identification:

| Model | Chip | SoC ID | NPU | Max Res | DDR | Package |
|-------|------|--------|-----|---------|-----|---------|
| — | Hi3516CV608 | 0x3516C608 | 0.2 TOPS | 3M | DDR2 512Mb | QFN |
| 10B | Hi3516CV610 | 0x3516C610 | 0.5 TOPS | 5M | DDR2 512Mb | QFN |
| 20S | Hi3516CV613 | 0x3516C613 | 1 TOPS | 4K | DDR3 1Gb | QFN |
| 00S | unknown | — | 1 TOPS | 4K | DDR3 1Gb | QFN |
| 20G | unknown | — | 1 TOPS | 4K | DDR3 1Gb | QFN, GB35114 |
| 00G | unknown | — | 1 TOPS | 4K | ext DDR3 4Gb | BGA, GB35114 |

Hi3516DV500 (`0x3516D500`) and Hi3519DV500 (`0x3519D500`) are the `HISI_OT`
aarch64 die (Cortex-A55) — same V5 0x11xxxxxx peripheral map, now emulated as the
`hi3519dv500` / `hi3516dv500` machines (confirmed against the SPC011 SDK U-Boot).

## Peripheral Support Matrix

| Peripheral | CV100 | CV200 | AV100 | CV300 | CV500 | 3519V101 | V4 (×8) | **V5 (×3)** |
|---|---|---|---|---|---|---|---|---|
| **Flash** | SFC350 | HiFMC | SFC350 | HiFMC | HiFMC | HiFMC | HiFMC | HiFMC |
| **Ethernet** | FEMAC | FEMAC | — | FEMAC | FEMAC | — | FEMAC | FEMAC |
| **SD/MMC** | himci×1 | himci×2 | himci×2 | himci×3 | himci×3 | himci×3 | SDHCI×2 | SDHCI×2 |
| **UARTs** | 3 | 3 | 4 | 3 | 3 | 5 | 3 | 3 |
| **SPI (PL022)** | — | 2 | 2 | 2 | 3 | 4 | 2 | 2 |
| **GPIO (PL061)** | 12 | 9 | 15 | 9 | 11 | 17 | 8-10 | 11 |
| **I2C** | — | 3 | 3 | 2 | 7 | 4 | 3 | 3 |
| **DMA (PL080)** | yes | yes | — | yes | — | — | — | — |
| **MIPI RX** | — | — | stub | — | yes | stub | yes | yes |
| **RTC** | — | — | yes | — | yes | yes | yes | yes |
| **VEDU/JPGE** | — | — | stub | — | yes | stub | yes | yes |
| **Watchdog** | yes | yes | yes | yes | yes | yes | yes | yes |
| **Sensor I2C** | — | — | — | — | — | — | yes | — |
| **IVE (18 ops)** | — | — | — | — | — | — | **yes** | — |
| **NPU** | — | — | — | — | — | — | — | stub |

Notes: `stub` = regbank stub only. V5 (×3) = CV608/CV610/CV613 (same die, different feature
tiers). AV100/3519V101 use GMAC (gigabit) — not emulated, boot without networking.
IVE = Intelligent Video Engine with 18 operations validated against real IVE silicon.

## Project Structure

```
qemu/
├── hw/arm/hisilicon.c           # Machine definitions (all 40 SoCs)
├── hw/misc/hisi-sysctl.c        # SysCtrl (SoC ID, reset, general registers)
├── hw/misc/hisi-crg.c           # CRG clock/reset stub
├── hw/misc/hisi-fmc.c           # HiFMC V100 flash controller (CV200+, flash-file for dumps)
├── hw/misc/hisi-gzip.c          # Hardware GZIP decompressor (U-Boot hw_compressed)
├── hw/misc/hisi-sfc350.c        # HISFC350 flash controller (CV100, AV100)
├── hw/misc/hisi-ddrc-v1.c       # V1 DDR controller/PHY: DDR dead until reg_info runs
├── hw/misc/hisi-himci.c         # DW MMC (himciv200) SD/MMC controller
├── hw/misc/hisi-regbank.c       # Generic RAM-backed register bank
├── hw/misc/hisi-ive.c           # IVE: 18 ops (SAD, CCL, Sub, Thresh, Erode...)
├── hw/misc/hisi-fastboot.c      # Boot ROM fastboot serial protocol (defib compat)
├── hw/misc/hisi-vedu.c          # Video encoder stub (VEDU + JPGE)
├── hw/misc/hisi-mipi-rx.c       # MIPI RX controller stub
├── hw/misc/hisi-rtc.c           # SPI-bridge RTC device
├── hw/net/hisi-femac.c          # Fast Ethernet MAC + MDIO PHY stub
├── hw/i2c/hisi-i2c.c            # HiBVT I2C controller
├── hw/i2c/hisi-imx335.c         # IMX335 image sensor I2C device
├── include/hw/arm/hisilicon.h   # SoC config structure + constants
├── hw/arm/xiongmai.c            # Xiongmai XM510/XM530/XM550 machines
├── hw/misc/xm-sysctl.c          # XM SYS_CTRL: soft reset, PLL lock
├── hw/misc/xm-ddrc.c            # XM510 uPCTL DDR controller (U-Boot)
├── hw/misc/xm-sfc.c             # xmsfc v1/v2 SPI-NOR controller (flash-file)
├── hw/net/xm-dwmac.c            # xmmac: DWMAC 10/100, XM MAC layout
├── include/hw/arm/xiongmai.h    # XmSoCConfig
├── hw/arm/goke.c                # Goke GK7101/GK7102 machines, boot ROM, HAL hand-off
├── hw/misc/gk-vic.c             # Ambarella-style VIC (edge latch, sense/event)
├── hw/misc/gk-timer.c           # Three Ambarella interval timers
├── hw/misc/gk-wdt.c             # Ambarella watchdog (reboot path)
├── hw/misc/gk-sflash.c          # Goke SPI-NOR controller (flash-file)
├── hw/misc/gk-ephy.c            # Internal Ethernet PHY, memory-mapped MII
├── hw/i2c/gk-idc.c              # Ambarella IDC I2C master
├── hw/misc/gk-remap.c           # GK710xS register layouts over the GK710x models
├── hw/misc/gk-dma.c             # GK710xS DMA controller (descriptor chains)
├── hw/arm/anyka.c               # Anyka AK39xx machines, synthetic mask ROM
├── hw/misc/ak-sysctrl.c         # AK39 SYSCTRL: chip ID, IRQ controller, timers, WDT, RTC
├── hw/misc/ak-l2.c              # L2 buffer SRAM + DMA (UART / SPI data path)
├── hw/misc/ak-ddrc.c            # DDR2 controller (init-done poll)
├── hw/char/ak-uart.c            # UART on L2 rings
├── hw/ssi/ak-spi.c              # SPI master, PIO + L2
├── hw/ssi/ak-spinor.c           # Byte-wise SPI NOR (flash-file, any JEDEC ID)
├── hw/gpio/ak-gpio.c            # GPIO (pull-up inputs, flash CS on EV330)
├── hw/i2c/ak-i2c.c              # I2C, EV200/EV300 and EV330 layouts
├── hw/usb/ak-usb.c              # MUSB-style OTG, nothing plugged in
├── hw/sd/ak-mci.c               # MCI SD/MMC host (sd-bus, FIFO)
├── include/hw/arm/anyka.h       # AkSoCConfig
├── hw/arm/sigmastar.c           # SigmaStar infinity2m machines, synthetic mask ROM
├── hw/misc/sstar-riu.c          # RIU register file under the models, boot status bits
├── hw/intc/sstar-intc.c         # Main interrupt controller (masks on the way to the GIC)
├── hw/timer/sstar-timer.c       # PIU timers + watchdog
├── hw/char/sstar-uart.c         # 16550 on the RIU stride + DesignWare USR
├── hw/ssi/sstar-fsp.c           # ISP/FSP/QSPI SPI-NOR controller, XIU window (flash-file)
├── hw/dma/sstar-bdma.c          # BDMA (flash -> IMI, MIU -> FSP page program)
├── hw/net/sstar-emac.c          # EMAC + internal PHY
├── hw/usb/sstar-uhc.c           # EHCI hosts behind the RIU (fusbh200-ehci-usb)
├── hw/display/mstar-disp.c      # GOP/MOP scan-out, display-top vsync (ported)
├── hw/display/mstar-ge.c        # GE 2D engine (ported)
├── include/hw/arm/sigmastar.h   # SsSoCConfig
├── include/hw/arm/goke.h        # GkSoCConfig
└── setup.sh                     # Clone QEMU v10.2.0, copy sources, build
qemu-boot/
├── run-cv100.sh                 # Run scripts for each boot-tested platform
├── run-cv200.sh
├── run-av100.sh
├── run-cv300.sh
├── run-cv500.sh
├── run-3519v101.sh
├── run-ev300.sh
├── run-ev300-flash.sh           # Boot from SPI NOR flash dump (no -kernel needed)
├── mk-ev300-nand.sh             # Assemble OpenIPC ev300 SPI-NAND/UBI image
├── run-ev300-nand.sh            # Boot from SPI NAND image (UBI rootfs)
├── run-gk7205v510-nand.sh       # Boot a GD5F1GM7 SPI-NAND camera dump
├── run-cv610.sh
├── run-xm510.sh                 # Xiongmai XM510, OpenIPC xm510_lite
├── run-gk7102.sh                # Goke GK7102: flash dump, or OpenIPC + vendor HAL
├── run-ssr621q.sh               # SigmaStar SSR621Q NVR: flash dump to the GUI
├── sstar-env.py                 # Set U-Boot env vars in a SigmaStar flash image
├── gk710x-halmap.py             # Run the GK710x HAL under Unicorn, print its map
├── test-sysupgrade.sh           # sysupgrade --archive on emulated NOR
├── test-firstboot.sh            # firstboot wipes the overlay, camera boots
├── test-overlay-keep.sh         # firmware#2613: unmountable overlay survives
├── test-ive-init.c              # IVE basic test (hw_id, dma, sad, ccl)
├── test-ive-ops.c               # IVE operations test for QEMU (register-level)
├── test-ive-mpi.c               # IVE operations test for real board (MPI API)
├── test-ive-video-mpi.c         # IVE video processing for real board (MPI API, 352×288)
├── test-ive.c                   # IVE test (standalone)
├── test-ive-video.c             # IVE motion detection on video frames (QEMU)
├── test-ive-abandoned.c         # IVE abandoned object detection (QEMU)
├── test-fastboot.sh             # End-to-end fastboot test (QEMU + defib)
└── test-fastboot-protocol.py    # Protocol-level fastboot test (no defib)
demo/
├── generate_scene.py            # Synthetic CCTV scene generator
├── generate_abandoned.py        # Synthetic abandoned bag scene generator
├── ive_demo.py                  # Host reference SAD+CCL + visualization
├── abandoned_demo.py            # Host reference abandoned object + visualization
└── run_demo.sh                  # End-to-end demo orchestration
docs/
├── ive-registers.md             # IVE register map (from live EV300 capture)
├── xiongmai.md                  # XM510/XM530 memory map, with kernel sources
├── goke-gk710x.md               # GK7101/GK7102: the HAL, real memory map, boot
└── nnie-vs-npu.md               # NNIE vs SVP_NPU architecture comparison
qemu-src/                        # (generated by setup.sh, not committed)
```

## Quick Start

```bash
# Build QEMU
bash qemu/setup.sh
```

### Method 1: Boot from flash image (recommended)

The simplest way to get started. Uses a single binary file — either a pre-built
full flash image from [OpenIPC](https://openipc.org/) or a raw dump from a real
IP camera's SPI NOR flash chip.

```bash
# Option A: Download a pre-built OpenIPC full flash image
#   Go to https://openipc.org/cameras/vendors/hisilicon/socs/hi3516ev300
#   and click "Download full image" (8MB NOR, lite)
#   Save as e.g. openipc-hi3516ev300-lite-8mb.bin

# Option B: Read flash from a real camera using a programmer
#   flashrom, CH341A, or similar → flash_dump.bin

# Boot (U-Boot and kernel load from flash, just like real hardware)
bash qemu-boot/run-ev300-flash.sh openipc-hi3516ev300-lite-8mb.bin

# Or with a camera flash dump
bash qemu-boot/run-ev300-flash.sh /tmp/flash_dump_ev300.bin

# Or pass the flash image directly via the machine property — works
# for any SoC, regardless of which flash controller it instantiates:
qemu-system-arm -M hi3516ev300,flash-file=$flash -nographic
qemu-system-arm -M hi3516av100,flash-file=$flash -nographic   # also works
qemu-system-arm -M gk7205v200,flash-file=$flash  -nographic   # also works
```

The emulator includes a boot ROM that copies U-Boot from the flash memory
window to DDR and jumps to it — the same boot sequence as real silicon.
A hardware GZIP decompressor (`hisi-gzip`) handles U-Boot's compressed
first-stage loader. No separate kernel or rootfs files needed.

The machine-level `flash-file` property forwards to whichever flash
controller (`hisi-fmc` on V3+/Goke, `hisi-sfc350` on V1/V2A) the SoC
instantiates, so consumers don't need to know that detail. The legacy
device-level globals (`-global hisi-fmc.flash-file=…` /
`-global hisi-sfc350.flash-file=…`) still work for backward compat.

Default login: `root` / `12345`

#### Factory-locked NOR (recovery testing)

XM-flashed Winbond W25Q128s ship from the factory with `SR3.WPS = 1`
and every individual block-lock bit set. WPS is non-volatile, so it
survives power-cycle and even a fresh-U-Boot via the boot ROM —
every `sf erase` / `sf write` from a recovery agent silently no-ops
until firmware issues Winbond Global Block Unlock (`0x98`) and
clears `SR3.WPS`. The runtime kernel/U-Boot/agent has to do this
unlock dance to make the chip writable
([OpenIPC kernel workaround](https://github.com/OpenIPC/linux/commit/3961fada5)).

To exercise that recovery-unlock path under QEMU, use
`hisi-fmc.nor-wps-locked=on` (default `off`):

```bash
qemu-system-arm -M hi3516ev200 -kernel u-boot.bin \
    -global hisi-fmc.nor-wps-locked=on \
    -global hisi-fmc.flash-file=blank-flash.img
```

When on, the emulator starts the chip in the as-shipped state
(SR3.WPS=1, all blocks locked, none ever-unlocked) and logs:

```
hisi-fmc: factory-locked NOR (SR3.WPS=1, all 128 blocks locked);
firmware must Global-Unlock 0x98 + clear SR3.WPS before erase/program
will succeed
```

A test passes only if the firmware actually issues the unlock
sequence — without the knob, the same firmware silently "works"
against an emulator that never had the lock to begin with.

#### Single-I/O-only NOR (dual-read fallback testing)

Some Xiongmai hi3518ev200 boards carry a `0xc22017` NOR (sold as
MX25L6406E/MX25L6436F) that answers only plain single-I/O reads: every
dual- or quad-I/O read comes back as zeros, while writes, erases and
status reads keep working. Stock U-Boot then fails with
`Wrong Image Format for bootm command`, and a kernel booted another way
panics with `VFS: Unable to mount root fs`
([OpenIPC/firmware#646](https://github.com/OpenIPC/firmware/issues/646)).

`hisi-fmc.nor-single-io-only=on` (default `off`) reproduces that chip:
a DMA read whose `OP_CFG` interface type is anything but standard
returns zeros and logs a `guest_errors` line.

```bash
qemu-system-arm -M hi3516cv200,flash-file=$flash -m 64M -nographic \
    -global hisi-fmc.flash-jedec=0xc22017 \
    -global hisi-fmc.nor-single-io-only=on
```

#### SPI-NAND flash boot (UBI/UBIFS)

`hisi-fmc` also models a SPI-NAND chip (GigaDevice GD5F1GM7 by default:
2 KiB page, 128 KiB block, 128 MiB), so NAND/UBI firmware boots straight
from a flash image — boot ROM → U-Boot → UBI → kernel → UBIFS/ubiblock
root, the same path as silicon. A `flash-file` larger than 16 MiB is
auto-detected as SPI-NAND, and the SoC's SYSSTAT boot-strap reports the
NAND boot device so U-Boot takes the NAND path instead of probing NOR.

```bash
# Goke GK7205V510 — boot a real 128 MiB GD5F1GM7 camera dump:
bash qemu-boot/run-gk7205v510-nand.sh nand-dump.bin

# OpenIPC hi3516ev300 NAND build — assemble image, then flash-boot it:
bash qemu-boot/mk-ev300-nand.sh        # u-boot + release rootfs.ubi (UBIFS, /boot/fitImage)
bash qemu-boot/run-ev300-nand.sh
```

`nand-jedec` overrides the READ-ID and `nand-oob-size` the spare-area size
(128 for GD5F1GM7, 64 for W25N01GV) for firmware whose ID table differs —
e.g. the OpenIPC ev300 U-Boot wants W25N01GV
(`-global hisi-fmc.nand-jedec=0xEFAA21 -global hisi-fmc.nand-oob-size=64`).
Dual NOR+NAND boards (NOR boot/env, NAND rootfs) attach the second chip
with `-global hisi-fmc.nand-file=…`.

### Method 2: Boot with separate kernel and rootfs

Useful for development when you build kernel and rootfs independently and
want to iterate without reflashing.

```bash
# Download OpenIPC firmware components
cd qemu-boot
for soc in hi3516cv100 hi3516cv200 hi3516av100 hi3516cv300 hi3516cv500 hi3519v101 hi3516ev300; do
    curl -sL "https://github.com/OpenIPC/firmware/releases/download/latest/openipc.${soc}-nor-lite.tgz" \
        -o "openipc.${soc}-nor-lite.tgz"
    tar xzf "openipc.${soc}-nor-lite.tgz"
done
cd ..

# Boot any platform (pass kernel + rootfs via QEMU -kernel/-initrd)
bash qemu-boot/run-cv300.sh
bash qemu-boot/run-ev300.sh
bash qemu-boot/run-cv500.sh
# etc.
```

Default login: `root` / `12345`

### CV610 Boot (FIT image)

CV610 uses a FIT image (kernel + DTB in one file). Extract components first:

```bash
# Extract kernel, DTB, and rootfs from firmware
cd qemu-boot
dumpimage -T flat_dt -p 0 -o cv610.dtb firmware.bin.hi3516cv610
dumpimage -T flat_dt -p 1 -o kernel.hi3516cv610 firmware.bin.hi3516cv610
# rootfs is at offset 0x2A0000 in the firmware blob
dd if=firmware.bin.hi3516cv610 of=rootfs.squashfs.hi3516cv610 bs=1 skip=$((0x2A0000))

# Boot
bash run-cv610.sh
```

## Architecture

The emulator uses a table-driven design. Each SoC is a `HisiSoCConfig` struct
with addresses, IRQs, and peripheral counts. A single `hisilicon_common_init()`
function handles all platforms — adding a new SoC variant requires only a new
config struct and a one-line `DEFINE_HISI_MACHINE()` macro call.

### Memory Map Eras

| Era | SoCs | SysCtrl | UARTs | RAM | IRQ | GIC base |
|-----|------|---------|-------|-----|-----|----------|
| V1/V2 (0x20xx) | CV100, CV200 | 0x20050000 | 0x20080000 | 0x80M | VIC | — |
| V2A (0x20xx+GIC) | AV100 | 0x20050000 | 0x20080000 | 0x80M | GIC | 0x20301000 |
| V3 (0x12xx) | CV300 | 0x12020000 | 0x12100000 | 0x80M | VIC | — |
| V3.5 (unique) | CV500 | 0x12020000 | 0x120A0000 | 0x80M | GIC | 0x10301000 |
| V3A (0x12xx+GIC) | 3519V101 | 0x12020000 | 0x12100000 | 0x80M | GIC | 0x10301000 |
| V4 (0x12xx) | EV300, Goke | 0x12020000 | 0x12040000 | 0x40M | GIC | 0x10301000 |
| **V5 (0x11xx)** | **CV610** | **0x11020000** | **0x11040000** | **0x40M** | **GIC** | **0x12401000** |

### Appended DTB Auto-Patching

OpenIPC uImages for DT-based kernels have an appended device tree. The emulator
automatically patches it at boot:

- Adds `chosen/stdout-path` for console output
- Inserts padding for kernel's `atags_to_fdt()` initrd patching
- Unifies SP804 split clocks (e.g., AV100's 50 MHz/3 MHz → 3 MHz) to prevent timer storms
- Disables NAND controllers whose register-level polling hangs on stubs

This runs transparently — unmodified firmware artifacts work as-is.
CV610 uses a separate DTB file (`-dtb` flag) instead of appended DTB.

## SD Card Emulation

```bash
# Create a 64 MB image with FAT32
dd if=/dev/zero of=sdcard.img bs=1M count=64
echo -e "o\nn\np\n1\n\n\nt\nc\nw\n" | fdisk sdcard.img
LOOP=$(sudo losetup --find --show --partscan sdcard.img)
sudo mkfs.vfat -F 32 -n SDCARD "${LOOP}p1"
sudo losetup -d "$LOOP"

# Attach to any platform
bash qemu-boot/run-ev300.sh -drive file=sdcard.img,if=sd,format=raw
```

## Ethernet

The FEMAC device provides a single 8 KiB MMIO region with port, MDIO, and GLB
register blocks. An integrated PHY stub at MDIO address 1 reports 100 Mbps
full-duplex (PHY ID `0x00446161`). TX uses DMA read + `qemu_send_packet()`;
RX uses a 64-entry ring buffer with IRQ notification.

Run scripts pass `-nic user` for QEMU's SLIRP networking (NAT to host,
gateway `10.0.2.2`). PHY link-up takes a few seconds — DHCP may need a retry.

Note: AV100 and 3519V101 use GMAC (gigabit) which is not yet emulated;
those SoCs boot without networking.

## IVE (Intelligent Video Engine)

The IVE device (`hisi-ive`) provides hardware-accelerated image processing
for V4 SoCs. Register map reverse-engineered from live EV300 hardware capture
(554 register changes, see `docs/ive-registers.md`). 18 operations implemented,
validated against real IVE silicon on EV300 board:

| Category | Operations | Status |
|----------|-----------|--------|
| Memory | DMA (copy with stride) | ✓ HW verified |
| Motion detection | SAD (4×4 block diff + threshold), CCL (connected components) | ✓ HW verified |
| Pixel-wise | Sub, Add (weighted blend), And, Or, Xor | ✓ HW verified |
| Thresholding | Thresh (binary), Hist (256-bin histogram) | ✓ HW verified |
| Convolution | Filter (5×5 kernel), Sobel (3×3 edge detection) | ✓ HW verified |
| Morphology | Dilate (5×5 max), Erode (5×5 min) | ✓ HW verified |
| Analysis | Integ (integral image), Map (LUT), NCC (cross-correlation) | ✓ verified |
| Background | GMM2 (Gaussian Mixture Model, stateful per-pixel) | ✓ verified |

### Validation against real hardware

Two test binaries validate the same algorithms through different paths:

| Test | Platform | Path | Image size |
|------|----------|------|-----------|
| `test-ive-mpi` | Real EV300 board | MPI API → libmpi.so → kernel module → IVE silicon | 64×64 |
| `test-ive-ops` | QEMU | Register writes → hisi-ive.c device | 64×64 |

Per-pixel output values match between real IVE hardware and QEMU emulation:
```
            Real HW                    QEMU
sub:        [18,22,26,30]              [18,22,26,30]       ✓
and:        [13,0,17,0]                [13,0,17,0]         ✓
or:         [31,62,63,98]              [31,62,63,98]       ✓
xor:        [18,62,46,98]              [18,62,46,98]       ✓
thresh:     [0,0,0,0]                  [0,0,0,0]           ✓
sobel:      [28,56,56,56]              [28,56,56,56]       ✓
dilate:     [223,255,255,255]          [223,255,255,255]    ✓
erode:      [0,0,0,0]                  [0,0,0,0]           ✓
```

```bash
# QEMU test (register-level, runs as init in initramfs)
CC=path/to/arm-openipc-linux-musleabi-gcc
$CC -static -O2 -o /tmp/init qemu-boot/test-ive-ops.c
mkdir -p /tmp/ive && cp /tmp/init /tmp/ive/init
cd /tmp/ive && find . | cpio -oH newc | gzip > /tmp/ive.gz
qemu-system-arm -M hi3516ev300 -m 128M \
    -kernel qemu-boot/uImage.hi3516ev300 -initrd /tmp/ive.gz \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 mem=128M root=/dev/ram0 rdinit=/init"

# Real board test (MPI API, needs SDK libs)
$CC -o test-ive-mpi qemu-boot/test-ive-mpi.c -I$SDK/include -L$LIBS -lmpi -live ...
# Upload and run: load_hisilicon -i; killall majestic; ./test-ive-mpi
```

### Full Hardware Pipeline

The real board test (`test-ive-video-mpi`) uses a **full hardware pipeline** with
zero CPU pixel loops. Reads Y4M (YUV4MPEG2 Cmono) or legacy raw .bin files.
Resolution is parsed from the Y4M header — follows Majestic's approach:
`MD_SIZE(x) = (x/2) & ~0xF` (half sensor resolution, 16-byte aligned).

| Platform | Binary | IVE path | Notes |
|----------|--------|----------|-------|
| Real EV300 board | `test-ive-video-mpi` | MPI API → libmpi.so → kernel module → IVE silicon | Ground truth |
| QEMU | `test-ive-video` | Register writes → hisi-ive.c device | Emulation |
| Host | `ive_demo.py` / `abandoned_demo.py` | Python reference + visualization | Demo video |

#### Motion Detection: SAD → CCL (2 IVE ops)

```
Frame (960×528) → SAD 4×4 blocks → threshold output (240×132) → CCL → blob regions
```

```bash
# Convert 1080p source to Majestic-equivalent resolution
ffmpeg -y -i source.mp4 -vf "scale=960:528,format=gray" -r 10 \
    -pix_fmt gray -f yuv4mpegpipe output.y4m

# Run on real IVE hardware
ssh root@ev300-board 'killall majestic; /utils/ive-test/test-ive-video-mpi \
    /utils/ive-test/meva-school-vehicles.y4m md'
# → FRAME 210: (840,316)-(848,328) area=5  ... vehicle motion detected
```

#### Abandoned Object Detection: 9-step IVE pipeline

```
Frame → Sub(cur,ref) → Thresh → Erode → Dilate → SAD(mask,zero) → blockify
     → SAD(cur,prev) → Thresh(invert) → AND(fg,stationary) → CCL → blob regions
```

CPU only reads the CCL blob struct (~3 KB) per frame — no pixel scanning.
The `SAD(mask, zero_image)` trick reduces the full-frame binary mask (960×528)
to block-level (240×132), staying within CCL's 64-720 pixel size constraint.

```bash
ssh root@ev300-board 'killall majestic; /utils/ive-test/test-ive-video-mpi \
    /utils/ive-test/meva-abandon-package.y4m abandoned'
# → FRAME 105: ABANDONED (804,212)-(848,256) area=82 dur=30
# → ... sustained 70+ frames (package left at bus stop)
```

#### License Plate Region Detection: 6-step IVE pipeline

```
Frame → Sobel(vertical) → 16BitTo8Bit(abs) → Thresh → Dilate → Erode → CCL
                                                                         ↓
                                               CPU: filter by aspect ratio 2.0-6.0
```

Detects plate-shaped rectangular regions with dense edges. Evaluated on CCPD
dataset (100 images): P=0.065, R=0.560, F1=0.116. High recall when it hits
(median IoU 0.67) but too many false positives from non-plate edge regions.
Needs NN assistance for production use (see `docs/ive-applications.md`).

```bash
ssh root@ev300-board 'killall majestic; /utils/ive-test/test-ive-video-mpi \
    /utils/ive-test/parking.y4m lpr'
# → FRAME 1: PLATE (452,500)-(516,520) area=32 ratio=3.2
```

#### Benchmark: 960×528 on real EV300 IVE silicon

| Component | Motion detection | Abandoned detection | Plate region |
|-----------|-----------------|-------------------|--------------|
| **IVE hardware** | **2.4 ms/frame** | **6.7 ms/frame** | **9.1 ms/frame** |
| CPU (blob parse) | 0.0 ms/frame | 0.1 ms/frame | 0.2 ms/frame |
| I/O (NFS read) | 40.8 ms/frame | 9.9 ms/frame | 33.3 ms/frame |
| **IVE capacity** | **416 fps** | **149 fps** | **110 fps** |
| Total (with NFS) | 23 fps | 60 fps | 24 fps |

In a real camera, VPSS feeds frames via DMA (zero I/O overhead), so the IVE
pipeline easily exceeds the 10 fps target. NFS is the bottleneck in the test setup.

### Quantitative Evaluation (MEVA + VIRAT Ground Truth)

Evaluated on **3132 motion events** and **131 abandoned-object events** from the
[MEVA](https://mevadata.org/) (CC BY 4.0) and [VIRAT Ground 2.0](https://viratdata.org/)
datasets. All processing on **real EV300 IVE silicon** at Majestic-equivalent resolution
(960×528 for 1080p, 640×352 for 720p sources).

#### Event-level metrics

| Algorithm | Precision | Recall | F1 | Events | Dataset |
|-----------|-----------|--------|------|--------|---------|
| **Motion detection** | **0.998** | **1.000** | **0.999** | 3132 | MEVA+VIRAT |
| **Abandoned object** | **0.361** | **0.673** | **0.470** | 131 | MEVA+VIRAT |
| **Plate region** | **0.065** | **0.560** | **0.116** | 100 | CCPD |

**Motion detection** is near-perfect: all motion events detected (vehicles, people
walking, carrying objects) with only 6 false alarms across 3132 test events.

**Abandoned object detection** catches 67% of events (bags, packages, unloaded objects)
but has false positives from stationary scene elements (parked cars, furniture) that
differ from the learned reference background. Threshold sweep (diff_thr 30-60,
sad_thr 100-300, area_thr 4-16) confirmed this is an inherent limitation of
background subtraction — the algorithm cannot distinguish "newly abandoned bag" from
"car that was parked during reference learning." Production-grade precision requires
owner-object tracking or semantic classification (see `docs/ive-applications.md`).

**Plate region detection** has decent recall (56%) but very low precision (6.5%) —
edge-based detection cannot distinguish plates from bumpers, signs, and window frames.
Needs neural network assistance for production use.

#### Dataset composition

Sources: MEVA examples (121 pre-trimmed 1080p clips) + VIRAT Ground 2.0 (83 HD clips).
Activities used as ground truth:

| Eval task | Positives | Negatives | Activity types |
|-----------|-----------|-----------|---------------|
| Motion | 3126 events | 6 static clips | walking, vehicle_moving, carrying, riding, ... |
| Abandoned | 52 events | 79 clips | Abandon_Package, SetDown, Unloading, Drop |

```bash
# Reproduce evaluation
python3 scripts/eval_download_virat.py       # download VIRAT videos (~18 GB)
python3 scripts/eval_build_dataset.py        # convert to Y4M
bash scripts/eval_run_ive.sh both            # run on real EV300 board
python3 scripts/eval_metrics.py              # compute P/R/F1
```

See `docs/meva-dataset-spec.md` for dataset structure, annotation format, and
evaluation methodology.

See `docs/ive-applications.md` for a roadmap of 9 CV applications
(tamper detection, line crossing, zone intrusion, loitering, etc.)
that can be built with the existing IVE operations.

## Fastboot Protocol (Serial Boot ROM Emulation)

When QEMU is started **without** `-kernel`, the machine emulates the HiSilicon
boot ROM serial download protocol. The [defib](https://github.com/OpenIPC/defib)
tool can then load firmware over UART — the same way real hardware is recovered.

```bash
# Terminal 1: start QEMU in fastboot mode (no -kernel)
qemu-system-arm -M hi3516ev300 -m 64M -nographic \
    -chardev socket,id=ser0,path=/tmp/qemu-hisi.sock,server=on,wait=off \
    -serial chardev:ser0

# Terminal 2: load firmware via defib
defib burn -c hi3516ev300 -p socket:///tmp/qemu-hisi.sock
```

Defib auto-downloads OpenIPC U-Boot for the chip. To use a local file:

```bash
defib burn -c hi3516ev300 -p socket:///tmp/qemu-hisi.sock -f u-boot.bin
```

The protocol implements three sequential transfers (DDR init → SPL → U-Boot)
with CRC-16/CCITT validation on every frame. After the final transfer, QEMU
hands the serial port to a PL011 UART and starts the CPU at the U-Boot entry
point — U-Boot output appears on the same connection.

Supported chips: all Standard protocol SoCs (Hi3516CV300, Hi3516EV200/EV300,
Hi3518EV300, GK7205V200/V300, and others). V500 and CV6xx protocols are not
yet implemented.

### Automated test

```bash
# Protocol-level test (no defib needed)
qemu-system-arm -M hi3516ev300 -m 64M -display none -monitor none \
    -chardev socket,id=ser0,path=/tmp/hisi-fb,server=on,wait=on \
    -serial chardev:ser0 -d unimp &
python3 qemu-boot/test-fastboot-protocol.py /tmp/hisi-fb

# End-to-end test with defib
bash qemu-boot/test-fastboot.sh
```

## References

- OpenIPC firmware: https://github.com/openipc/firmware
- ipctool: https://github.com/openipc/ipctool
- SDK device trees:
  - CV100: `Hi3518_SDK_V1.0.B.0/.../mach-hi3518/`
  - CV200: `Hi3518E_SDK_V1.0.4.0/.../hi3518ev20x.dtsi`
  - AV100: `Hi3516A_SDK_V1.0.8.0/.../hi3516a.dtsi`
  - CV300: `Hi3516CV300_SDK_V1.0.4.0/.../hi3516cv300.dtsi`
  - CV500: `Hi3516CV500_SDK_V2.0.2.1/.../hi3516cv500.dtsi`
  - 3519V101: `Hi3519V101_SDK_V1.0.5.0/...patch (hi3519v101.dtsi)`
  - EV300: `Hi3516EV200_SDK_V1.0.1.2/.../hi3516ev300.dtsi`
  - CV610: `Hi3516CV610R001C01SPC020/...tgz (hi3516cv610.dtsi)`
