# Hi3516CV300 / Hi3516EV300 Hardware Reference

All addresses and IRQ numbers are from the official SDK device trees:
- CV300: `Hi3516CV300_SDK_V1.0.4.0/.../hi3516cv300.dtsi`
- EV300: `Hi3516EV200_SDK_V1.0.1.2/.../hi3516ev300.dtsi`

---

## 1. Summary Table

| Peripheral | CV300 Address | CV300 IRQ | EV300 Address | EV300 IRQ (SPI) | IP Core | Emulation |
|------------|--------------|-----------|---------------|-----------------|---------|-----------|
| CPU | — | — | — | — | ARM926EJ-S / Cortex-A7 | Emulated |
| IRQ Controller | Built-in | — | 0x10301000/0x10302000 | — | PL190 / GICv2 | Emulated |
| Timer 0/1 | 0x12000000 | 3 | 0x12000000 | 5 | SP804 | Emulated |
| Timer 2/3 | 0x12010000 | 4 | 0x12001000 | 6 | SP804 | Emulated |
| ARM Timer | N/A | — | — | PPI 13/14 | armv7-timer | Emulated |
| UART0 | 0x12100000 | 5 | 0x12040000 | 7 | PL011 | Emulated |
| UART1 | 0x12101000 | 30 | 0x12041000 | 8 | PL011 | Emulated |
| UART2 | 0x12102000 | 25 | 0x12042000 | 9 | PL011 | Emulated |
| SPI0 | 0x12120000 | 6 | 0x12070000 | 14 | PL022 | Emulated |
| SPI1 | 0x12121000 | 7 | 0x12071000 | 15 | PL022 | Emulated |
| GPIO0 | 0x12140000 | 31 | 0x120b0000 | 16 | PL061 | Emulated |
| GPIO1 | 0x12141000 | 31 | 0x120b1000 | 17 | PL061 | Emulated |
| GPIO2 | 0x12142000 | 31 | 0x120b2000 | 18 | PL061 | Emulated |
| GPIO3 | 0x12143000 | 31 | 0x120b3000 | 19 | PL061 | Emulated |
| GPIO4 | 0x12144000 | 31 | 0x120b4000 | 20 | PL061 | Emulated |
| GPIO5 | 0x12145000 | 31 | 0x120b5000 | 21 | PL061 | Emulated |
| GPIO6 | 0x12146000 | 31 | 0x120b6000 | 22 | PL061 | Emulated |
| GPIO7 | 0x12147000 | 31 | 0x120b7000 | 23 | PL061 | Emulated |
| GPIO8 | 0x12148000 | 31 | 0x120b8000 | 24 | PL061 | Emulated |
| GPIO9 | — | — | 0x120b9000 | 25 | PL061 | Emulated |
| DMA | 0x10030000 | 14 | 0x100B0000 | 38 | PL080 / hiedmacv310 | Emulated (CV300 only) |
| SysCtrl | 0x12020000 | — | 0x12020000 | — | HiSilicon | Stubbed |
| CRG | 0x12010000 | — | 0x12010000 | — | HiSilicon | Stubbed |
| I2C0 | 0x12110000 | — | 0x12060000 | — | hibvt-i2c | Missing |
| I2C1 | 0x12112000 | — | 0x12061000 | — | hibvt-i2c | Missing |
| I2C2 | — | — | 0x12062000 | — | hibvt-i2c | Missing |
| FMC | 0x10000000 | — | 0x10000000 | — | HiSilicon | Missing |
| MMC0 | 0x100c0000 | 18 | 0x10010000 | 30 | himciv200 / hisi-sdhci | Blacklisted |
| MMC1 | 0x100d0000 | 27 | 0x10020000 | 31 | himciv200 / hisi-sdhci | Blacklisted |
| MMC2 (eMMC) | 0x100e0000 | 11 | — | — | himciv200 | Blacklisted |
| MMC3 | 0x100f0000 | 27 | — | — | himciv200 | Blacklisted |
| Ethernet | 0x10050000 | 12 | 0x10040000 | 33 | hisi-femac-v2 | Blacklisted |
| USB EHCI | 0x10120000 | 15 | — | — | EHCI | Missing |
| USB OHCI | 0x10110000 | 16 | — | — | OHCI | Missing |
| USB UDC | 0x10130000 | 10 | — | — | HiSilicon | Missing |
| USB DWC3 | — | — | 0x10030000 | 39 | Synopsys DWC3 | Missing |
| Watchdog | 0x12080000 | — | 0x12030000 | 2 | hi_wdg | Missing |
| RTC | 0x12090000 | 2 | 0x120e0000 | 0 | hi_rtc / hi35xx-rtc | Missing |
| PWM | 0x12130000 | — | 0x12080000 | — | hi3516cv300-pwm | `hisi-pwm` |
| ADC (LSADC) | — | — | 0x120a0000 | 4 | hisi-lsadc | Missing |
| Cipher | — | — | 0x10050000 | 34 | hisi-cipher | Missing |
| MISC | — | — | 0x12028000 | — | HiSilicon | Missing |
| IR | — | 19 | — | — | — | Missing |
| PMU | — | — | — | PPI 58 | ARM PMU | Missing |

---

## 2. Core — CPU, Interrupt Controller, Timers

### CV300

- **CPU**: ARM926EJ-S (ARMv5TEJ), single core
- **Interrupt Controller**: PL190 VIC — 32 flat IRQ lines
- **Timers**: 2× SP804 dual-channel timers
  - Timer 0/1: 0x12000000, IRQ 3
  - Timer 2/3: 0x12010000, IRQ 4

### EV300

- **CPU**: Cortex-A7 (ARMv7-A), single core
- **Interrupt Controller**: GICv2 — 160 IRQs (128 SPI + 16 SGI + 16 PPI)
  - Distributor: 0x10301000
  - CPU Interface: 0x10302000
- **Timers**: 2× SP804 dual-channel timers
  - Timer 0/1: 0x12000000, SPI 5
  - Timer 2/3: 0x12001000, SPI 6
- **ARM Generic Timer**: 50 MHz
  - Secure Physical Timer: PPI 13
  - Non-Secure Physical Timer: PPI 14
- **PMU**: SPI 58

---

## 3. System Control & Clocks

### SysCtrl — 0x12020000 (both SoCs)

- CV300 compatible: `hi3516cv300-sys,syscon`
- EV300 compatible: `hisilicon,sysctrl`
- Reboot: write 0xdeadbeef to offset 0x4
- SCSYSID registers: 0xEE0–0xEEC (SoC identification)

### CRG (Clock & Reset Generator) — 0x12010000 (both SoCs)

- CV300 compatible: `hi3516cv300-crg` (0x1000 region)
- EV300 compatible: `hi3516ev300-clock,syscon` (0x1000 region)
- PLL lock detection: bit 28
- Clock trees:
  - CV300: 28 fixed clocks (3 MHz – 400 MHz)
  - EV300: 29+ fixed clocks (100 kHz – 1500 MHz)

### MISC Controller — 0x12028000 (EV300 only)

Pad control, FEPHY configuration, USB configuration.

---

## 4. UARTs (PL011)

3 channels per SoC, ARM PrimeCell PL011.

| Channel | CV300 Address | CV300 IRQ | EV300 Address | EV300 SPI |
|---------|--------------|-----------|---------------|-----------|
| UART0 | 0x12100000 | 5 | 0x12040000 | 7 |
| UART1 | 0x12101000 | 30 | 0x12041000 | 8 |
| UART2 | 0x12102000 | 25 | 0x12042000 | 9 |

---

## 5. I2C (hibvt-i2c) — HiSilicon Proprietary

| Bus | CV300 Address | EV300 Address |
|-----|--------------|---------------|
| I2C0 | 0x12110000 | 0x12060000 |
| I2C1 | 0x12112000 | 0x12061000 |
| I2C2 | — | 0x12062000 |

HiSilicon proprietary I2C controller (`hibvt-i2c`). CV300 has 2 buses, EV300 has 3.

---

## 6. SPI (PL022)

2 channels per SoC, ARM PrimeCell PL022.

| Channel | CV300 Address | CV300 IRQ | EV300 Address | EV300 SPI |
|---------|--------------|-----------|---------------|-----------|
| SPI0 | 0x12120000 | 6 | 0x12070000 | 14 |
| SPI1 | 0x12121000 | 7 | 0x12071000 | 15 |

- CV300 CS mux register: 0x12030000
- EV300 CS mux register: 0x12028000

---

## 7. GPIO (PL061)

ARM PrimeCell PL061, 8 lines per port.

### CV300 — 9 ports, all sharing IRQ 31

| Port | Address |
|------|---------|
| GPIO0 | 0x12140000 |
| GPIO1 | 0x12141000 |
| GPIO2 | 0x12142000 |
| GPIO3 | 0x12143000 |
| GPIO4 | 0x12144000 |
| GPIO5 | 0x12145000 |
| GPIO6 | 0x12146000 |
| GPIO7 | 0x12147000 |
| GPIO8 | 0x12148000 |

### EV300 — 10 ports, individual IRQs (SPI 16–25)

| Port | Address | SPI |
|------|---------|-----|
| GPIO0 | 0x120b0000 | 16 |
| GPIO1 | 0x120b1000 | 17 |
| GPIO2 | 0x120b2000 | 18 |
| GPIO3 | 0x120b3000 | 19 |
| GPIO4 | 0x120b4000 | 20 |
| GPIO5 | 0x120b5000 | 21 |
| GPIO6 | 0x120b6000 | 22 |
| GPIO7 | 0x120b7000 | 23 |
| GPIO8 | 0x120b8000 | 24 |
| GPIO9 | 0x120b9000 | 25 |

---

## 8. DMA

### CV300: PL080 / hisi-dmac @ 0x10030000, IRQ 14

Alternative drivers: ARM PL080 DMA controller or HiSilicon `hisi-dmac`. Same base address.

### EV300: hiedmacv310 @ 0x100B0000, SPI 38

HiSilicon EDMAC v310. 4 channels, 32 peripheral request lines.

---

## 9. Flash Memory Controller (FMC) — HiSilicon Proprietary

Both SoCs share the same architecture:
- Control registers: 0x10000000
- Memory-mapped flash: 0x14000000

Sub-controllers:
- **CV300**: `hisi-fmc` with `hisi-sfc` (SPI NOR) and `hisi-spi-nand` (SPI NAND)
- **EV300**: `hisi-fmc` with `fmc-spi-nor` and `fmc-spi-nand`

---

## 10. MMC/SD

### CV300 — himciv200 (4 controllers)

| Instance | Address | IRQ | Function |
|----------|---------|-----|----------|
| mmc0 | 0x100c0000 | 18 | SD card |
| mmc1 | 0x100d0000 | 27 | SD card |
| mmc2 | 0x100e0000 | 11 | eMMC (HS200, 1.8V) |
| mmc3 | 0x100f0000 | 27 | SD card |

### EV300 — hisi-sdhci (2 controllers)

| Instance | Address | SPI | Function |
|----------|---------|-----|----------|
| mmc0 | 0x10010000 | 30 | eMMC (HS400, enhanced strobe) |
| mmc1 | 0x10020000 | 31 | SD card |

---

## 11. Ethernet (FEMAC) — HiSilicon Proprietary

Both SoCs use `hisi-femac-v2`.

| | CV300 | EV300 |
|-|-------|-------|
| MAC | 0x10050000 | 0x10040000 |
| GLB (MAC base + 0x1300) | 0x10051300 | 0x10041300 |
| MDIO | 0x10051100 | 0x10041100 |
| IRQ | 12 | SPI 33 |
| PHY | External | Integrated FEPHY |

---

## 12. USB

### CV300 — Standard EHCI/OHCI + HiSilicon UDC

| Controller | Address | IRQ |
|------------|---------|-----|
| EHCI | 0x10120000 | 15 |
| OHCI | 0x10110000 | 16 |
| UDC (device) | 0x10130000 | 10 |

PHY: Inno USB2 PHY (HiSilicon-specific).

### EV300 — Synopsys DWC3

| Controller | Address | SPI |
|------------|---------|-----|
| DWC3 | 0x10030000 | 39 |

Compatible: `snps,dwc3`. High-speed only. PHY: `hixvp-usb2-phy`.

---

## 13. Watchdog

| | CV300 | EV300 |
|-|-------|-------|
| Address | 0x12080000 | 0x12030000 |
| IRQ | — | SPI 2 |
| Compatible | `hi_wdg` | `hi_wdg` |

---

## 14. RTC

| | CV300 | EV300 |
|-|-------|-------|
| Address | 0x12090000 | 0x120e0000 |
| IRQ | 2 | SPI 0 |
| Compatible | `hi_rtc` | `hi35xx-rtc` (hibvt_rtc driver) |

---

## 15. PWM

Modelled by `hisi-pwm` (`qemu/hw/misc/hisi-pwm.c`) on every IPC machine:

- The block reads 0 while its CRG word gates it or holds it in reset.
- It counts at the rate the word's source field selects.
- A duty latches on a CTRL write, and on V1 only on the enable edge.
- Each channel's output is published over QOM: `/machine/pwm`, properties
  `chN-enabled`, `chN-period-ns`, `chN-high-ns` and `clock-hz`.

The rows below give the block, the channel count and the CRG word, with the
bit meanings and rates.

| family | block | ch | CRG word | bits | rates |
|---|---|---|---|---|---|
| V1 cv100 / 18ev100 / 18av100 / 18cv100 | 0x20130000 | 3 | +0x38 | bit 1 gate | 3 MHz |
| V2 cv200 / 18ev200 | 0x20130000 | 4 | +0x38 | bit 1 gate, bit 2 source | 3 / 50 MHz |
| V2A av100 / dv100 | 0x20130000, ch 4-7 at +0x1000 | 8 | +0x38 | as V2 | 3 / 50 MHz |
| V3 cv300 / ev100 | 0x12130000 | 4 | +0x38 | as V2 | 3 / 50 MHz |
| V3A 3519v101 / av200 | 0x12130000 | 8 | +0xdc | as V2 | 3 / 50 MHz |
| V4A cv500 / av300 / dv300 | 0x12070000 | 2 | +0x1bc | bit 7 gate, bit 6 reset, [9:8] source | 3 / 50 / 24 / 24 MHz |
| V4 ev200 / ev300 / gk7205v200 family | 0x12080000 | 4 | +0x1bc | as V4A | as V4A |
| gk7205v500 / v510 / v530 / gk7202v330 | 0x12080000 | 12 | +0x1bc | as V4A | as V4A |
| V5 cv608 / cv610 / cv613 | 0x11080000, stride 0x100, minus-one | 8 | +0x4598 | bit 4 gate, bit 0 reset, [13:12] source | 1 / 24 / 198 MHz |

The sources:

- **V4A**: measured on a lab hi3516av300. Stock `0x01802802` reads 0; bit 7
  wakes the block; sources 0 / 1 / 2 measured 2997703 / 50026950 /
  24011375 Hz; only channels 0 and 1 decode.
- **V1, V2, V3, V4**: measured on lab boards, by sampling a bounded run's
  remaining-periods count twice (the method in OpenIPC/majestic#324).
- **V2A, V3A, V5**: from the vendor drivers, openhisilicon `kernel/pwm/` and
  the kernels' `pwm-bsp.c` / `clk_*.c`.

---

## 16. ADC (LSADC)

- **EV300 only**: 0x120a0000, SPI 4, compatible `hisi-lsadc`
- CV300: Not present

---

## 17. Cipher/Crypto

- **EV300 only**: 0x10050000, SPI 34, compatible `hisi-cipher`
- CV300: Not present

---

## 18. Pinmux

### CV300

- `pinctrl-single` @ 0x12040000 (0x108 region) — function muxing
- `pinconf` @ 0x12040800 (0x130 region) — drive strength / pull configuration

### EV300

- `iocfg-controller` @ 0x100C0000 (syscon) — combined mux and configuration

---

## 19. Media Pipeline

All media blocks are HiSilicon proprietary and require the vendor kernel.

### CV300 Media Peripherals

| Block | Address | IRQ | Function |
|-------|---------|-----|----------|
| MIPI RX | 0x11300000 | 28 | CSI-2 receiver |
| VIU | 0x11380000 | 22 | Video input unit |
| ISP | 0x11392200 | 22 | Image signal processor |
| VPSS | 0x11250000 | 17 | Video post-processing subsystem |
| VEDU | 0x11260000 | 24 | Video encoder (H.264/H.265) |
| JPGE | 0x11220000 | 26 | JPEG encoder |
| VGS | 0x11240000 | 29 | Video graphics subsystem |
| VOU | 0x11400000 | 23 | Video output unit |
| AIAO | 0x11310000 | 9 | Audio I/O |
| Audio Codec | 0x11320000 | — | Internal audio codec |
| IVE | 0x11230000 | 21 | Intelligent video engine |

### EV300 Media Peripherals

| Block | Address | SPI | Function |
|-------|---------|-----|----------|
| MIPI RX | 0x11240000 | 45 | CSI-2 receiver |
| VI CAP | 0x11000000 | 43 | Video input capture |
| VI PROC | 0x11000000 | 44 | Video input processing |
| ISP | 0x11220000 | 43 | Image signal processor |
| VPSS | 0x11400000 | 46 | Video post-processing subsystem |
| VEDU | 0x11410000 | 47 | Video encoder |
| JPGE | 0x11420000 | 48 | JPEG encoder |
| VGS | 0x11300000 | 49 | Video graphics subsystem |
| VO | 0x11280000 | 40 | Video output |
| HIFB | 0x11280000 | 41 | Framebuffer overlay |
| AIAO | 0x100e0000 | 42 | Audio I/O |
| Audio Codec | 0x100f0000 | — | Internal audio codec |
| IVE | 0x11320000 | 51 | Intelligent video engine |
| GZIP | 0x11310000 | 50 | Hardware compression |

---

## 20. Complete IRQ Maps

### CV300 — PL190 VIC (flat IRQ numbers 0–31)

| IRQ | Peripheral |
|-----|------------|
| 2 | RTC |
| 3 | Timer 0/1 |
| 4 | Timer 2/3 |
| 5 | UART0 |
| 6 | SPI0 |
| 7 | SPI1 |
| 9 | AIAO (Audio) |
| 10 | USB UDC |
| 11 | eMMC (mmc2) |
| 12 | Ethernet |
| 14 | DMA |
| 15 | USB EHCI |
| 16 | USB OHCI |
| 17 | VPSS |
| 18 | SD (mmc0) |
| 19 | IR |
| 21 | IVE |
| 22 | VIU / ISP |
| 23 | VOU |
| 24 | VEDU |
| 25 | UART2 |
| 26 | JPGE |
| 27 | SD (mmc1, mmc3) |
| 28 | MIPI RX |
| 29 | VGS |
| 30 | UART1 |
| 31 | GPIO (all 9 ports shared) |

### EV300 — GIC SPI Numbers

| SPI | Peripheral |
|-----|------------|
| 0 | RTC |
| 2 | Watchdog |
| 4 | ADC (LSADC) |
| 5 | Timer 0/1 |
| 6 | Timer 2/3 |
| 7 | UART0 |
| 8 | UART1 |
| 9 | UART2 |
| 14 | SPI0 |
| 15 | SPI1 |
| 16 | GPIO0 |
| 17 | GPIO1 |
| 18 | GPIO2 |
| 19 | GPIO3 |
| 20 | GPIO4 |
| 21 | GPIO5 |
| 22 | GPIO6 |
| 23 | GPIO7 |
| 24 | GPIO8 |
| 25 | GPIO9 |
| 30 | MMC0 (eMMC) |
| 31 | MMC1 (SD) |
| 33 | Ethernet (FEMAC) |
| 34 | Cipher |
| 38 | DMA (hiedmacv310) |
| 39 | USB (DWC3) |
| 40 | VO (Video Output) |
| 41 | HIFB (Framebuffer) |
| 42 | AIAO (Audio) |
| 43 | VI CAP / ISP |
| 44 | VI PROC |
| 45 | MIPI RX |
| 46 | VPSS |
| 47 | VEDU |
| 48 | JPGE |
| 49 | VGS |
| 50 | GZIP |
| 51 | IVE |
| 58 | PMU |

### EV300 — PPI Numbers

| PPI | Peripheral |
|-----|------------|
| 13 | Secure Physical Timer |
| 14 | Non-Secure Physical Timer |

---

## 21. IP Core Provenance

| IP Block | Vendor | Notes |
|----------|--------|-------|
| ARM926EJ-S / Cortex-A7 | ARM | CPU cores |
| PL190 VIC | ARM | CV300 interrupt controller |
| GICv2 | ARM | EV300 interrupt controller |
| PL011 UART | ARM | Both SoCs |
| SP804 Timer | ARM | Both SoCs |
| PL061 GPIO | ARM | Both SoCs |
| PL022 SPI | ARM | Both SoCs |
| PL080 DMA | ARM | CV300 only |
| DWC3 USB | Synopsys | EV300 USB controller |
| System Controller | **HiSilicon** | Proprietary, HiSilicon SoCs only |
| CRG (Clock/Reset) | **HiSilicon** | Proprietary, HiSilicon SoCs only |
| hibvt-i2c | **HiSilicon** | Proprietary I2C controller |
| FMC (Flash) | **HiSilicon** | Proprietary flash memory controller |
| himciv200 / hisi-sdhci | **HiSilicon** | Proprietary MMC/SD controllers |
| hisi-femac-v2 | **HiSilicon** | Proprietary fast Ethernet MAC |
| USB PHY (Inno/hixvp) | **HiSilicon** | Proprietary USB PHY |
| hiedmacv310 | **HiSilicon** | Proprietary EDMAC (EV300) |
| ISP / VI / VPSS / VEDU | **HiSilicon** | Proprietary media pipeline |
| VGS / VOU / IVE / GZIP | **HiSilicon** | Proprietary media pipeline |
| AIAO + Codec | **HiSilicon** | Proprietary audio subsystem |
| hisi-cipher | **HiSilicon** | Proprietary crypto engine |

---

## 22. QEMU Emulation Status

| Peripheral | CV300 | EV300 | Status |
|------------|-------|-------|--------|
| CPU | ARM926EJ-S | Cortex-A7 | **Emulated** |
| Interrupt Controller | PL190 | GICv2 | **Emulated** |
| UART ×3 | PL011 | PL011 | **Emulated** |
| Timer ×2 | SP804 | SP804 | **Emulated** |
| ARM Generic Timer | N/A | PPI 13/14 | **Emulated** |
| GPIO ×9/10 | PL061 | PL061 | **Emulated** |
| SPI ×2 | PL022 | PL022 | **Emulated** |
| DMA | PL080 | — | **Emulated** (CV300 only) |
| SysCtrl | 0x12020000 | 0x12020000 | **Stubbed** — SoC ID + reboot only |
| CRG | 0x12010000 | 0x12010000 | **Stubbed** — PLL lock bits only |
| I2C | — | — | Missing |
| FMC (Flash) | — | — | Missing |
| MMC/SD | — | — | **Blacklisted** (`himci_init`) |
| Ethernet | — | — | **Blacklisted** (`hisi_femac_driver_init`) |
| USB | — | — | Missing |
| Watchdog | — | — | Missing |
| RTC | — | — | Missing |
| PWM | — | — | `hisi-pwm` |
| ADC | — | — | Missing |
| Cipher | — | — | Missing |
| Pinmux | — | — | Missing |
| Media Pipeline | — | — | Missing (vendor kernel drivers only) |

---

## 23. Known Issues & Debug Log Findings

### SysCtrl Unimplemented Offsets
- **0x8c** (CV300) — eMMC pad configuration, accessed during boot
- **0x150** (CV300) — eMMC 1.8V drive strength
- **0x130** (EV300) — unknown register, accessed during boot

### PL022 Extended Registers
- Offsets **0x28** and **0x2c** accessed on EV300 — HiSilicon extensions to PL022 not present in QEMU's PL022 model

### CV300 Boot Issues
- Full `ipctool` run hangs — MDIO/I2C polling via `/dev/mem` too slow on emulated ARM926
- Full init hangs at `mmz` vendor kernel module — requires unimplemented media memory zone hardware

### Blacklisted Drivers
Kernel modules blacklisted in QEMU run scripts to avoid hangs on unimplemented hardware:
- `himci_init` — MMC/SD controller init
- `hisi_femac_driver_init` — Ethernet MAC init
