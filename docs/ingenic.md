# Ingenic T-series (`t10`, `t20`, `t21`, `t23`, `t30`, `t31`, `t40`, `t41`)

The Ingenic T-series is the only non-ARM SoC family in this repository: little-endian MIPS32 camera SoCs built into `qemu-system-mipsel` (`qemu/setup.sh` adds `mipsel-softmmu` to QEMU's target list). One machine file, `qemu/hw/mips/ingenic.c`, defines a table entry per SoC, while the individual peripheral device models live in `qemu/hw/*/ingenic-*.c`.

## Contents

- [Reference sources](#reference-sources)
- [Machines](#machines)
- [Memory map](#memory-map)
- [CPUs](#cpus)
- [Boot](#boot)
- [System and peripheral devices](#system-and-peripheral-devices)
- [Storage, networking and Wi-Fi](#storage-networking-and-wi-fi)
- [Audio](#audio-ingenic-pdma-ingenic-aic-ingenic-dmic-and-ingenic-asdmic)
- [Image pipeline](#image-pipeline-sensor-ingenic-isp-ingenic-isp-apical-ingenic-ivdc-and-ingenic-ipu)
- [Video encoders, display and NNA accelerators](#video-encoders-display-and-nna-accelerators)
- [Tested](#tested)
- [Not modelled](#not-modelled)

## Reference sources

| Source | What it provided |
|---|---|
| OpenIPC `linux` branches `ingenic-t31` (3.10.14: T10, T20, T21, T30, T31), `ingenic-t23`, and `ingenic-t40` (4.4.94) | MMIO base addresses, IRQ numbers, `arch/mips/xburst*/soc-*/` platform setup, and every boot-time hardware poll |
| OpenIPC `u-boot-ingenic` (`isvp_t*`) and the T40/T41 vendor U-Boot trees | Mask ROM SPL header, SPL/U-Boot register polls, and flash partition layouts |
| Vendor SDKs T23 1.1.0, T31 1.1.5, T40 1.2.0, and T41 1.2.0 | T23 N/LP/HP variants, T41 4.4.94 kernel and drivers, and full flash images |
| Camera dumps: T10, T20, T21 x2, T31 x3 (one a Wyze Cam v3), and T40XP | Vendor boot chains end to end |

All of these SoCs boot cleanly; see [Tested](#tested).

---

## Machines

| Machine | CPU core | Default RAM | Kernel | SFC | MSC | Notes |
|---|---|---|---|---|---|---|
| `t10` | XBurst1 | 64 MiB | 3.10 | v1 | `jzmmc` | DWC DDR PHY; GPIO port stride `0x100` |
| `t20` | XBurst1 | 64 MiB | 3.10 | v1 | `jzmmc` | Same base layout as T10 (use `-m 128M` for T20X) |
| `t21` | XBurst1 (16 KiB L1 caches on silicon) | 64 MiB | 3.10 | v1 | `jzmmc` | "+1" PLL encoding; SD card detect on `PB20` |
| `t23` | XBurst1 (16 KiB L1 caches) | 64 MiB | 3.10 | v1 | `jzmmc` | Two PLLs; N, LP, and HP variants differ only in clocks |
| `t30` | XBurst1 | 64 MiB | 3.10 | v1 | `jzmmc` | "+1" PLL encoding plus a fourth PLL (`EPLL`) |
| `t31` | XBurst1 | 64 MiB | 3.10 | v1 | `jzmmc` | Reference part (use `-m 128M` for T31X/T31A) |
| `t40` | 2 x XBurst2 | 128 MiB | 4.4 (built-in DT) | v2 | SDHCI | CCU, per-core INTC and OST (use `-m 256M` for T40XP) |
| `t41` | 2 x XBurst2 | 128 MiB | 4.4 (built-in DT) | v2 | SDHCI | Dedicated T41 PLL encoding; RTC moved to `0x10003000` |

Variants within each family (`T31N`/`L`/`X`/`A`/`AL`/`LC`, `T23N`/`LP`/`HP`, `T40N`/`XP`/`A`, `T41LQ`/`NQ`/`XQ`) differ only in DRAM capacity and clock settings, which the bootloader programs and the kernel reads back. A single QEMU machine serves all variants when combined with `-m` and `variant=`. Supported DRAM is at most 256 MiB, all mapped below the I/O space.

### SoC identification and variants

Guest software (following `ipctool` `src/hal/ingenic.c` and Ingenic's `libsysutils`) distinguishes dies and grades using three registers:

- **SoC ID at `0x1300002C` (`ingenic-harb`)**: Bits `31:28` equal `1`, bits `19:12` identify the family (`0x21`, `0x23`, `0x30`, `0x31`, `0x40`, `0x41`; except bits `27:12` equal `0x0005` on T10 and `0x2000` on T20), and the lower bits hold the revision.
- **`CPM CPPSR` low byte**: Reads `0x10` on the `L` parts of T10/T20/T30, and `1` otherwise.
- **eFuse grade code**: Stored in the upper halfword of `SUBSOCTYPE1` (`0x13540238`) on T10–T31, and `SUBSOCTYPE2` (`0x13540250`) on T40/T41.

Use `-M <soc>,variant=<grade>` to configure `CPPSR` and the eFuse code (pass `-m` separately for DRAM size). Running `ipctool` in the guest then reports the configured grade (`T31X`, `T40XP`, etc.):

- `t10`: `t10`, `t10l`
- `t20`: `t20`, `t20l`, `t20x`
- `t21`: `t21n`, `t21l`, `t21x`, `t21z`
- `t23`: `t23n`, `t23x`
- `t30`: `t30n`, `t30l`, `t30x`, `t30a`, `t30z`
- `t31`: `t31n`, `t31l`, `t31x`, `t31a`, `t31zl`, `t31zx`, `t31al`, `t31zc`, `t31lc`
- `t40`: `t40n`, `t40nn`, `t40xp`, `t40a`
- `t41`: `t41nq`, `t41n`, `t41l`, `t41lq`, `t41lc`, `t41zl`, `t41zn`, `t41zx`, `t41a`

The first entry listed for each SoC is its default.

The T41 SDK SPL and current `ipctool` identify subtype `0x6666` as T41ZX and `0x4444` as T41A. Older `ipctool` builds, including the one in the T40 release root used for the T41 Majestic test, report those codes as `T41X` and `unknown`, respectively.

Use `variant=t41zx` for T41XQ images. The XQ U-Boot uses the ZX configuration with different DDR skew settings, and its subtype code is unknown. Set `-m` to at least the size the SDK U-Boot expects for the grade. L/LQ need 64 MiB, N/NQ/A need 128 MiB, and ZX/XQ need 256 MiB. The T41ZX SPL repeatedly resets at 64 MiB.

---

## Memory map

All base addresses below are physical addresses; guest software accesses I/O through uncached `KSEG1` (`0xA0000000 +`). Any unimplemented address inside `0x10000000`–`0x13FFFFFF` is handled by `ingenic.io`, which reads as zero and logs accesses when `-d unimp` is enabled.

| Base | Block | QEMU model / Backing |
|---|---|---|
| `0x00000000` | DDR | RAM |
| `0x10000000` | CPM: PLLs, dividers, clock gates, resets | `ingenic-cpm` |
| `0x10001000` | INTC: 64 sources routed to CPU `IP2` (XBurst1) | `ingenic-intc` |
| `0x10002000` | TCU: 8 timer/PWM channels, watchdog, bootloader OST | `ingenic-tcu` |
| `0x10003000` | RTC / hibernation (at `0x132A0000` on T40); IRQ `32` (`3` on T41) | `ingenic-rtc` |
| `0x10004000` | T41 always-on RTC memory (4 KiB) | RAM |
| `0x10010000` | GPIO: ports every `0x1000` (`0x100` on T10/T20) plus shadow port Z | `ingenic-gpio` |
| `0x10020000` / `0x10021000` | AIC (I2S, IRQ `1`) and internal audio codec | `ingenic-aic` (`/machine/aic`) |
| `0x10022000` / `0x10023000` | MIPI D-PHY / CSI-2 host (T23, T31; CSI1 on T40); see DMIC below for T10/T20/T30 | `ingenic-isp` |
| `0x10030000 + 0x1000*n` | UART0..3 (UART0..5 on T41; UART4/5 on IRQs `57`/`56`), 16550 with 4-byte stride and line-rate PDMA pacing | QEMU `serial-mm` (`0021`, `0023`) |
| `0x10034000` | T31, T40 DMIC (digital microphones), IRQ `12`, PDMA `13`; on T10/T20: `0x10022000`, IRQ `0`; on T30: `0x10023000`, IRQ `40` (both PDMA `0x21`) | `ingenic-dmic` (`/machine/dmic`) |
| `0x10043000` / `0x10044000` | SSI0 / SSI1 SPI masters (SSI1 on T21, T30, T31, T40, T41), IRQ `9` / `8`, buses `ssi0` / `ssi1` | `ingenic-ssi` |
| `0x10050000 + 0x1000*n` | I2C0..3 (`DW_apb_i2c`), PDMA requests `0x24 + 2*n` / `0x25 + 2*n` | `ingenic-i2c` |
| `0x10060000` | OTG PHY byte registers | RAM |
| `0x10061000` | DES / triple-DES engine (all SoCs), PDMA `0x2E` / `0x2F` | `ingenic-des` |
| `0x10070000` | SADC: 8 AUX channels, 12-bit, IRQ `18`, PDMA request `9` | `ingenic-sadc` (`/machine/sadc`) |
| `0x10072000` | DTRNG, IRQ `34` (T21, T23, T31, T40, T41) | `ingenic-dtrng` |
| `0x12000000` | System OST (`IP4`, XBurst1); global OST (XBurst2) | `ingenic-ost` |
| `0x12100000 + 0x100*n` | Core OST routed to `IP4` (XBurst2) | `ingenic-ost` (`layout` 2) |
| `0x12200000` | CCU: core reset/entry and mailboxes routed to `IP3` (XBurst2) | `ingenic-ccu` |
| `0x12300000 + 0x100*n` | Core INTC routed to `IP2` (XBurst2) | `ingenic-ccu` |
| `0x12500000` | T40, T41 NNDMA (descriptor RAM + channels at `+0x8000`) | `ingenic-nndma` (`/machine/nndma`) |
| `0x12600000` | XBurst2 ORAM: 1 MiB NNA SRAM (T41 SPL places its core-1 stub at `0x12660000`) | RAM |
| `0x12B00000` (alias `0x13090000`) | T41 AIP: NNA image pre-processor (convert, resize, warp), IRQ `40` / `39` / `38` | `ingenic-aip` (`/machine/aip`) |
| `0x13000000` | AHB0 arbiter (SoC ID at `+0x2C`) | `ingenic-harb` |
| `0x13010000` | DDR PHY (`+0x1000`) and DDRC APB (`+0x2000`) | `ingenic-ddr` |
| `0x13050000` | T40, T41 DPU (frame/layer LCD compositor on display `dpu`), IRQ `20`; T21, T23, T30, T31 LCDC ("tiny" TFT/smart panel controller on display `lcdc`), IRQ `31` (T30: `5`) | `ingenic-dpu` (`/machine/dpu`) / `ingenic-lcdc` (`/machine/lcdc`) |
| `0x13060000` / `0x13070000` | MSC0 / MSC1 (XBurst2): SDHCI 3.0 | QEMU `generic-sdhci` |
| `0x13080000` | IPU: `libimp` OSD blender and CSC (all SoCs), IRQ `14` | `ingenic-ipu` |
| `0x13090000` | T40 BSCALER: NNA resize/affine/perspective and NV12->color engine, IRQ `40` (T) / `39` (C) | `ingenic-bscaler` (`/machine/bscaler`) |
| `0x130A0000` | T40, T41 AXI/AHB bus MONITOR, IRQ `53` | `ingenic-monitor` |
| `0x130B0000` | T40, T41 I2D: rotate/mirror/flip engine, IRQ `45` (on T30: MSCA scaler) | `ingenic-i2d` / `ingenic-isp` |
| `0x130C0000` | T41 VO: NV12 out over BT.656 / BT.1120 to display, IRQ `43` | `ingenic-vo` (`/machine/vo`) |
| `0x130D0000` | T40, T41 DRAWBOX: OSD rectangles and lines, IRQ `42` | `ingenic-drawbox` |
| `0x13100000` | T23 HELIX (IRQ `48`); T41 AVPU (from `+0`, IRQ `70`) | `ingenic-vpu` / `ingenic-avpu` |
| `0x13200000` | Video encoder: VPU (T10/T20), HELIX (T21/T30; T23 at `0x13100000`), IRQ `70` (T23: `48`); AVPU from `+0x8000` (T31, T40), IRQ `70`; or T41/T23 IVDC registers, core INTC `13` (T23: INTC `38`) | `ingenic-vpu` / `ingenic-avpu` / `ingenic-ivdc` |
| `0x13280000` | T41 JPEG engine (HELIX JPEG half, renumbered units), core INTC `0` | `ingenic-vpu` (`layout` 1, `/machine/jpeg`) |
| `0x13300000` | ISP core (T31 up to `0x1338FFFF`; T21, T23: 64 KiB; T40, T41: 512 KiB, T41 scaler at `0x133F0000`; T10/T20/T30 Apical core at `0x13380000`) | `ingenic-isp`, `ingenic-isp-apical` |
| `0x133E0000` | VIC (T21: `0x13310000`; T40, T41: `0x13380000`; T40 VIC1/VIC2 at `0x13390000` / `0x133A0000`) | `ingenic-isp` |
| `0x13420000` | PDMA: 32 descriptor channels, IRQ `10` | `ingenic-pdma` |
| `0x13430000` | AES engine (all SoCs), IRQ `23` | `ingenic-aes` |
| `0x13440000` | SFC + SPI NOR or SPI NAND | `ingenic-sfc` + `m25p80` / `ingenic-spinand` |
| `0x13450000` | T41 SFC1, IRQ `19` (`-drive if=mtd,index=1`), or XBurst1 MSC0 | `ingenic-sfc` / `ingenic-msc` |
| `0x13460000` | XBurst1 MSC1, or T41 8-channel PWM controller | `ingenic-msc` / `ingenic-pwm` (`/machine/pwm`) |
| MSC1 SDIO | ATBM6031 Wi-Fi (`-M t31,wifi=atbm`; T40/T41 on SDHCI MSC1) | `atbm-sdio` |
| MSC1 SDIO | ATBM6441 IoT Wi-Fi (`-M t31,wifi=atbm6441`) | `atbm6441-sdio` |
| `0x13480000` | HASH engine (MD5, SHA-1/2 on T21, T23, T31, T40, T41), IRQ `22` | `ingenic-hash` |
| `0x134B0000` | Synopsys GMAC with IP101G PHY | `ingenic-gmac` |
| `0x134C0000` | RSA 1024/2048-bit engine, polled (T23, T31, T40, T41) | `ingenic-rsa` |
| `0x134D0000` | T41 audio system DMA, FIFOs, data bus, and DMIC (core INTC `33`) | `ingenic-asdmic` (`/machine/dmic`) |
| `0x134F0000` | DDRC controller | `ingenic-ddr` |
| `0x13500000` | DWC2 USB OTG host controller (A-device, 16 host channels) | QEMU `dwc2-usb` (`otg-a-device`, `host-channels=16`) |
| `0x13540000` | eFuse: blank except grade code | `ingenic-efuse` |
| `0x13600000` | RADIX: T30 H.265 encoder, IRQ `53` | `ingenic-vpu` (`kind` 1) |
| `0x1F000000` | Cache-as-RAM window (32 KiB) | RAM |
| `0x1FC00000` | Boot ROM mapped at reset vector `0xBFC00000` | ROM |
| `0x80000000` | T41, T23 IVDC frame store addressed by `direct_mode` (32 MiB) | RAM |

### Interrupt map

Hardware interrupt controller (INTC) source numbers are shared across the family (Linux IRQ numbers add `+8`, so ISP INTC `29` is Linux IRQ `37`, VIC `30` is `38`, and AVPU `62` is `70`):

- AIC `1`, SFC `7`, SSI1 `8`, SSI0 `9`, PDMA `10`, DMIC `12` (`40` on T30, `0` on T10/T20), GPIO A `17`, B `16`, C `15`, D `14`, SADC `18`, OTG `21`, HASH `22`, AES `23`, TCU `27`/`26`/`25`, DTRNG `34`, MSC0/1 `37`/`36`, UART0..3 `51..48`, GMAC `55`, I2C0..3 `60..57`, ISP `29`, VIC `30` (T40: VIC1 `29`, VIC2 `28`), AVPU `62`.
- On T40/T41: DRAWBOX `42`, I2D `45`, MONITOR `53`.
- On T40: BSCALER `40` (T) / `39` (C).
- On T41: AIP `40` (T) / `39` (F) / `38` (P), VO `43`, IVDC `13` (T23 IVDC `38`), JPEG `0`, ASDMIC `33`.

---

## CPUs

QEMU's upstream MIPS models lack several Ingenic CP0 features and SIMD units required by these kernels and vendor binaries, so `qemu/patches/0015-mips-ingenic-xburst-cpus.patch` adds two dedicated CPU types.

### XBurst1 (T10–T31)

- **Identity and caches**: `PRId` is `0x00d00100` (`0x00d00101` on T20, matching camera boot logs). Models a 32-bit FPU (the kernels hard-code FPU presence and OpenIPC userspace is hard-float), 32 TLB entries (the kernel forces `tlbsize = 32` regardless of `Config1`), and 32 KiB 8-way caches. T21 and T23 have 16 KiB L1 caches; the machine sets `l1-kib` and T20's `prid` via `qemu/patches/0017-mips-prid-l1-size-properties.patch`. T23 reports its 128 KiB L2 (8 ways of 32-byte lines) in `Config2` (`0x80000347`); other XBurst1 chips do not report an L2 geometry.
- **ISA, FPU, and idle behavior**: Implements MIPS32R2 decoding so `mfc0/mtc0 $12, 2` (the SPL's second instruction) executes without error, while 3.10 R1 kernels are unaffected. Sets `FIR.F64` (keeping `Status.FR = 0`) so QEMU decodes `COP1X` instructions (`MADD.D`, `MSUB.D`, ...) used by OpenIPC's `musl` `libm`, such as `logf` in `libimp` rate control. Enables `Config7.WII` so QEMU wakes from `wait` when the kernel idles with interrupts disabled (`jz_wait_irqoff`).
- **MXU2 SIMD (`qemu/patches/0018-mips-ingenic-mxu2.patch`)**: Adds Ingenic's 128-bit SIMD unit (32 x 128-bit registers; note that T-series hardware includes MXU2 rather than MXU1, despite older manual summaries). `Config1.C2` is set and `Status.CU2` is writable. Every MXU2 instruction traps with Coprocessor Unusable until `CU2` is enabled, letting the kernel perform lazy trap-based context save/restore using MXU2's own loads and stores. Instruction encodings come from the Ingenic toolchain (`as -mmxu2`), and semantics come from intrinsic names and MSA equivalents (uncertain cases are marked `GUESS` in source). `qemu/tests/test-ingenic-mxu2.sh` verifies every instruction, lazy enable, context switch, and `libimp`'s MXU2 `jz_memcpy` (requires the Ingenic toolchain; skipped otherwise).

### XBurst2 (T40, T41)

- **Identity, caches, and SMP**: `PRId` is `0x00132100` (company `0x13`, bits `15:13 = 1`, which Linux 4.4 identifies as XBurst2). Includes FPU, `RI`/`XI`, `UserLocal`, and `Config4.IE = 2` (without which single-instruction `tlbinvf` TLB flushes become no-ops). T40 and T41 report a 128 KiB L2 (T40 product briefs and U-Boot `isvp_t40.h`: 8 ways of 64-byte lines) in `Config2`, with CCU `0x60` bits `12:10` reading `1`, enabling Linux L2 detection and maintenance. Core 1 starts powered off until `boot_secondary()` writes its entry address to CCU `RER` and clears its `CSRR` bit (`-smp 1` runs single-core).
- **MXU3 and MXU3.1 SIMD (`qemu/patches/0019-mips-ingenic-mxu3.patch`)**: Adds the 512-bit SIMD unit (32 x 512-bit vector registers, four 512-bit accumulators, 32 scalar registers, plus T41 MXU3.1's three extra shuffles), gated by lazy `CU2` trapping. Encodings come from the Ingenic GCC 7.2 toolchain's opcode table, and semantics come from RTL expansions in `mxu3.h`. Out of 496 mnemonics, 265 are exact; the remainder (reductions, strided and gathered loads, sub-byte forms, and several opaque float and shuffle intrinsics) are approximations or leave their destination untouched so code completes without crashing even if results are approximate.
  - Under `-d unimp`, the first use of any approximated mnemonic is logged. Setting `QEMU_MXU3_STATS=1` (or `QEMU_MXU3_STATS=all` for every MXU3 mnemonic) prints execution counts at exit.
  - `qemu/tests/test-ingenic-mxu3.sh` verifies representative exact groups, lazy `CU2` enabling, and two processes pinned to core 0 (the T40 kernel enables MXU3 only on core 0). It runs from initramfs on T40, and as `/init` inside a NOR SquashFS under the SDK kernel on T41 (since the SDK kernel lacks initramfs), where `-mmxu3.1` also checks decoding and retirement of `gshufvb`, `gshufwb_1`, and `gshufwb_2`. Both the T40XP camera app and the T41 SDK image start cleanly without `SIGILL`, exercising MXU3 per video frame (`libimp` motion/resize and `libvenus`).

### TLB shadow entries (`tlb-shadow`)

When `tlbwr` replaces a TLB slot, upstream QEMU normally retains the evicted mapping in a hidden "shadow" translation cache as long as the guest uses `tlbp`, `tlbwi`, or `tlbinv(f)`. On dual-core T40/T41 Linux 4.4, user processes intermittently read stale data from a remapped page while the kernel read the current bytes for the same virtual address (for instance, `pow()` returned garbage in a static T40 client, making `libimp` DMIC volume calculate `inf` or `0` and producing constant `0x7FFF` or silent recordings in roughly 40% of runs). `qemu/patches/0020-mips-tlb-shadow-property.patch` adds the `tlb-shadow` CPU property, which the Ingenic machine disables so `tlbwr` drops the replaced entry like real hardware.

### XBurst2 custom `rdhwr` registers

On CPUs with MXU3, `qemu/patches/0022-mips-xburst2-hwr.patch` handles Ingenic's custom `rdhwr` registers:
- **`$4`**: Physical address of the most recent `ll` instruction (used by vendor kernels via `ll; rdhwr $4` to translate user addresses to physical addresses).
- **`$8`..`$10` and `$11`, `$12`**: Wait hooks for the NNA DMA read and write channels used directly from userspace by `libvenus`. `HWREna` holds bits `4` and `8..12` (the `soc-nna` driver's `open()` sets all ones in `HWREna` and enables `Status.CU2`).

---

## Boot

### Mask ROM and SPI NOR

Using `-M <soc>,flash-file=image.bin` without `-kernel` emulates the on-chip Mask ROM NOR boot sequence:

1. **SPL header check**: At flash offset `0`, verifies the SPL magic (`06 05 04 03 02 55 aa 55 aa`), checks the CRC7 of `[0x800, length)` against byte `9`, and reads the SPL length (`<= 0x6800`) at `0xC` (emitting a warning on CRC mismatch).
2. **Cache-as-RAM (CAR) window**: Copies the SPL to `0x80001000` and enters `0x80001800` with `sp = 0x80008000` (the SPL never sets its own stack). Using `qemu/patches/0016-mips-cache-as-ram-window.patch` (`car-size`/`car-phys`), QEMU maps `KSEG0` `0x80000000`–`0x80007FFF` onto the 32 KiB SRAM at `0x1F000000` from reset until the first instruction fetch outside that range (the jump to U-Boot or the kernel). This mirrors hardware locked cache lines and stops T41's SPL from overwriting itself when it writes test patterns across all of DDR through `KSEG1`.
3. **Bootloader handoff and persistence**: The SPL initializes pinmux, UART1, TCU OST delays, CPM PLLs/dividers (every lock and busy poll succeeds immediately in `ingenic-cpm`), and DDR (`ingenic-ddr`). It then loads U-Boot from flash offset `0x6800` to `0x80100000` (LZO-compressed in OpenIPC builds, raw in vendor builds), and U-Boot boots Linux. Flash writes and erases write back to the image file so U-Boot's saved environment and JFFS2 overlay persist across boots.

Use `-drive if=mtd,file=...` to attach NOR flash without booting from it. Default NOR parts are `w25q64` (8 MiB), `mx25l12805d` (16 MiB; QEMU has no `w25q128`, and JEDEC ID `0xC22018` is in all vendor tables), and `w25q256` (32 MiB), or choose explicitly with `flash-type=` (including Zbit `zb25vq64` and `zb25vq128`).

### SPI NAND

Any `flash-file` sized for SPI NAND attaches to `SFC0` (SFC0 is standard across all parts including T41 NAND boards; `SFC1` is optional). Raw page images keep OOB in RAM initialized to `0xFF` (all blocks good) on boot; full images with OOB read/write OOB in the file. Image size picks the part, or `flash-type=` overrides it:

| Image size | Part (JEDEC ID) | Geometry |
|---|---|---|
| 128 MiB, 132 MiB with OOB | `w25n01gv` (`EF AA 21`) | 2048 + 64 bytes, 64 pages/block, 1024 blocks |
| 136 MiB with OOB (128 MiB via `flash-type=`) | `gd5f1gq4ub` (`C8 D1`) | 2048 + 128 bytes, 1024 blocks |
| 256 MiB, 272 MiB with OOB | `gd5f2gq4ub` (`C8 D2`) | 2048 + 128 bytes, 2048 blocks |
| 264 MiB with OOB | `mx35lf2ge4ab` (`C2 22`) | 2048 + 64 bytes, 2048 blocks, two planes |

All four parts exist in SPL, U-Boot, and both kernel tables, and writes/erases persist to disk.

Mask ROM, U-Boot, and partition layouts:
- **Mask ROM NAND header**: Same format as NOR except byte `8 = 00` (NOR: `aa`), byte `10 = pages_per_block / 32`, and byte `11 = (page + OOB) / 1024` (for example, `2112` bytes becomes `2` in `start.S`). Up to 28 KiB of SPL loads into CAR above `0x80001000` (length at `0xC`, CRC7 at `9`).
- **SPL and U-Boot (`spl_sfc_nand.c`, `jz_sfc_nand`)**: Probes ID, unlocks protection, sets ECC/BUF/QE, reads U-Boot raw from data offset `0x6C00` (27 KiB) via page reads + `0x0B` cache reads + status polls, and stores redundant environments at `0xC0000` and `0xE0000`.
- **3.10 burner partition table & `qemu-boot/ingenic-nand-image.sh`**: The 3.10 `jz_sfc_nand` driver creates no MTD partitions unless the vendor burner's partition table (magic `nand`, count, `{name[32], size, offset, mask_flags, manager_mode}`) appears at data offset `0x6800` inside SPL padding. `ingenic-nand-image.sh` builds a full image matching that layout:

```text
0x000000  u-boot-with-spl.bin (SPL padded to 27 KiB + raw u-boot.bin)
0x006800  Partition table: 1M(boot),8M(kernel),40M(rootfs),-(data)
0x0C0000  U-Boot environment (blank; U-Boot saves on first boot)
0x100000  uImage
0x900000  UBI: "rootfs" = static SquashFS (ubi.block=0,0, root=/dev/ubiblock0_0)
               "rootfs_data" = UBIFS overlay mounted by OpenIPC /init (holds /etc/fw_env.config)
```

- **OpenIPC SPI NAND fixes**: Stock `isvp_<soc>_sfcnand` builds fail because `isvp_common.h` overwrites NAND settings with NOR values (producing a 230 KiB U-Boot that SPL only partially loads, `sf` boot commands, and a `0x40000` env offset inside U-Boot). `qemu-boot/ingenic-uboot-sfcnand.patch` restores the NAND config. Bundling `/etc/fw_env.config` prevents OpenIPC's `fw_printenv` from scanning all 128 MiB via `mtdblock` for minutes. `qemu-boot/test-ingenic-nand.sh` boots twice and verifies persisted U-Boot env and UBIFS files.
- **T41 vendor NAND layout (`ingenic-nand-image.sh --layout t41`)**: Vendor `isvp_t41*_sfc0_nand` U-Boot puts `uImage` at 1 MiB (3 MiB) and UBIFS `rootfs` at 4 MiB, passing `mtdparts=sfc0_nand:` on cmdline (4.4 reads the burner table at `0`, finds the SPL, and falls back to `mtdparts=`); `--env` writes the environment. Because T41 SPL `epc_debug.c` copies a core-1 stub to `0x12660000` inside NNA ORAM (`0x12600000`) and starts core 1 via CCU, ORAM must be backed by RAM or core 1 jumps into boot ROM exception vectors and hangs the SPL at random. Because SDK 4.4 `isvp_tomcat_nand_defconfig` kernels suffer an OST interrupt storm on core 1 (`"ost couldn't find ostfr"`), the T41 NAND test boots with `maxcpus=1`.
- **Direct `-kernel` NAND defaults**: Pairing `-kernel` with a NAND image automatically supplies `ubi.mtd=2 ubi.block=0,0 root=/dev/ubiblock0_0` and `mtdparts=sfc_nand:` (`sfc0_nand:` on T41).

### Direct `-kernel` boot

`-kernel uImage` (LZMA, gzip, or uncompressed) or ELF sets UART1 `LCR` non-zero (early console picks the first UART with non-zero `LCR`) and enters the kernel using U-Boot arguments (`a0 = 2`, `a1 = argv`, `a2 = envp`). With `-dtb`, QEMU switches to the MIPS UHI boot protocol (`a0 = -2`, `a1` pointing to the FDT in `KSEG0`, updating `/chosen`'s `bootargs`, `linux,initrd-start`, and `linux,initrd-end`).

- Unless `auto-mem=off` is set, missing `mem=` triggers default U-Boot `mem=` and `rmem=` values for the configured `-m` size, and `-initrd` loads at 16 MiB with `rd_start=` / `rd_size=`.
- For Device Tree kernels using reusable CMA, pass `-M t23,auto-mem=off` and omit `mem=`, `rmem=`, and `SDKMem=` from `-append` so DT `/memory` and `/reserved-memory` take effect unaltered.
- With no `-append`, the default OpenIPC command line is used: `console=ttyS1,115200n8`, `root=/dev/ram0` (with initrd) or `/dev/mtdblock3`, and OpenIPC `mtdparts` on `jz_sfc` (`sfc0_nor` on T41).

```bash
./qemu-src/build/qemu-system-mipsel -M t31 -m 64M -nographic \
    -kernel uImage.t31 -initrd rootfs.squashfs.t31 -nic user,restrict=on
```

### OpenIPC images

OpenIPC ships `u-boot-<soc>-nor.bin` (`u-boot-t40n-universal.bin` on T40) and `-nor-lite` kernel/rootfs binaries without pre-assembled full flash images. Placing U-Boot at `0`, `uImage` at `0x50000`, and SquashFS at `0x250000` matches U-Boot's environment and boots via `flash-file=`; `qemu-boot/test-ingenic-boot.sh --mode flash` downloads OpenIPC U-Boot and builds/boots that image. `--mode split` boots the split layout instead: the release's `firmware.bin.<soc>` (from `openipc.<soc>-nor-lite-split.tgz`, OpenIPC/firmware#2912), a uImage and then its SquashFS at the next 64 KiB, written whole at `0x50000` under the release's U-Boot. The kernel divides it at boot (OpenIPC/linux `split_layout.c`, `CONFIG_MTD_SPLIT_LAYOUT`), and the test checks for its "split NOR layout" line, the root on the rootfs it split off (`31:4`, not the `mtdblock3` the command line names), and the JFFS2 overlay from the end of `firmware.bin`. A release without the package is a SKIP. T41's U-Boot, which the firmware release does not carry, comes from the OpenIPC/u-boot `ingenic` release.

### Vendor SDK images

T41, T40, and T30 SDKs ship full NOR flash images (`resource/image_t*/<GRADE>_gcc*_<libc>_<8|16>M_*.img` accompanied by a `.txt` naming the U-Boot model and drivers): SPL at `0`, U-Boot at `0x6800`, environment at `0x3C000`, `uImage` at `0x40000`, root SquashFS at `0x2C0000`, followed by JFFS2 `appfs` (`/system`: kernel modules, `carrier-server`, and `start.sh`).

Pass `-m` and `variant=` matching the image grade (`T41LQ`: 64 MiB with `mem=32M rmem=32M`; `T41NQ`/`T40N`/`T30X`: 128 MiB; `T41XQ`/`T40XP`: 256 MiB). These images have passwordless root via `telnetd` and no DHCP client: `app_init.sh` configures static `eth0` (`193.169.4.151` on most images; `10.3.33.235` on T41LQ and 8 MiB T41NQ), and `start.sh` loads ISP/audio/sensor modules based on `sinfo` and launches `carrier-server`, a live555 RTSP server (`rtsp://<address>:8554/main`, `/second`).

`qemu-boot/test-ingenic-sdk-boot.sh --image <img>` extracts grade, RAM, and sensor metadata from the image, rebuilds the rootfs so `rcS` invokes `app_init.sh` with test networking (and without IVDC `direct_mode` when `--no-direct-mode` is passed), forwards telnet and RTSP over slirp user networking, logs in over telnet (`qemu-boot/ingenic-telnet.py`), pulls and decodes the stream, and reboots once over telnet.

---

## System and peripheral devices

### Clocks and power (`ingenic-cpm`)

Most CPM registers act as read/write storage, except clock and power handshakes (none of which have software timeouts, so incorrect readback hangs boot):
- PLL `ON` bits (and `LOCK` bits where present) immediately follow their enable bits.
- `CPCCR` change bits read back clear and `CPCSR` busy bits read `0`. Any divider written with `CE` clears `BUSY` immediately (bit `28`, or bit `27` for T21–T31 SSI/SFC and LPC dividers). I2S fractional `M`/`N` dividers (with `M` in bits `28:20`) remain unmodified.
- Reset values match what vendor U-Boot leaves in each chip's PLL encoding so early kernel init does not oops on divider mux `3` or `DDRCDR = 0`.
- `CPPSR` and `CPSPPR` preserve reboot signatures checked by reboot-recovery code.
- `LCR` power domains immediately acknowledge state changes (bit `24 + m` mirrors bit `28 + m`) for the 3.10 suspend path (`soc-t31/common/pm.c` `DISABLE`/`ENABLE_LCR_MODULES`, though no shipped kernel enables PM).

| PLL encoding | SoCs | Rate |
|---|---|---|
| `M[31:20] N[19:14] OD1[13:11] OD0[10:8]` | T10, T20, T23, T31, T40 | `24 MHz x M / N / OD1 / OD0` |
| `M[28:20] N[19:14] OD[13:11] RNG[7:5]` | T21, T30 | `24 MHz x 2(M + 1) / (N + 1) / 2^OD` |
| `M[28:20] N[19:14] OD0[13:11] OD1[10:7] RG[6:4]` | T41 | `24 MHz x 2(M + 1) / (N + 1) / 2^OD0 / (OD1 + 1)` |

### Timers, watchdog and PWM (`ingenic-ost`, `ingenic-tcu`, `ingenic-pwm`)

- **OST (`ingenic-ost`, `0x12000000`)**: Clocked at `EXTAL / 4^n`. Channel 1 counts to `T1DFR`, sets `TFR` bit `0`, wraps to `0`, and interrupts `IP4` when unmasked (clockevent). Channel 2 is a 64-bit free-running clocksource whose low-word read latches the high word into a shadow buffer. XBurst2 splits these into global OST (counter 2) and per-core OST (`0x12100000 + 0x100*n`, counter 1 behind `CCU OIMR`).
  - *XBurst1 vs XBurst2 gating*: XBurst1 system OST gates its interrupt with `TER` bit `0` in addition to `TFR & ~TMR`, whereas XBurst2 core OST drives `TFR & ~TMR` even while stopped: T41 Linux 4.4's `set_next_event` and shutdown stop the counter before its handler clears `TFR`, and suppressing the line while stopped caused core 1 to storm `"ost couldn't find ostfr"` (as seen on the SDK NAND kernel and SDK image under host load).
- **TCU (`ingenic-tcu`, `0x10002000`)**:
  - *Bootloader OST (`0x10002000 + 0xE0`)*: Enabled via `TESR` bit `15` (`EXTAL/4`); required for SPL and U-Boot `mdelay`.
  - *Watchdog*: Counts while `TCER` bit `0` is set and `TSR` bit `16` is clear, resetting the SoC at `TDR`. U-Boot and every kernel (including T40, whose own restart hook is empty) reboot through it. On XBurst2, `TSR` bit `16` does not stop the watchdog and `TCSR` bit `10` zeroes the counter: Linux 4.4 probes with the stop bit set without clearing it, and restart writes `TCSR` bit `10`, `TDR`, clock, and `TCER` while spinning on `"check wdt."`.
  - *Timers & PWM*: Each channel counts from `0` at its `TCSR` clock, setting half flag (`TFR` bit `n + 16`) at `TDHR` and full flag (bit `n`) at `TDFR` before wrapping (`TCU0` = ch0, `TCU1` = ch1, `TCU2` = ch2..7). Linux 3.10 `jz_tcu` (`jz_tcu.c`) demuxes all channels off `TCU2`, driving OpenIPC `motor.ko` and SDK `sample_motor` on channel 2's full interrupt (`test-ingenic-motor.sh`). With PWM enabled (`TCSR` bit `7` on, bit `8` initial level, bit `9` shutdown; OpenIPC `jz_tcu.c` / SDK `sample_pwm`), the pin starts at the initial level, toggles at `TDHR`, and ends at `TDFR`. `qom-get /machine/tcu pwm2` reports `high/period` ticks, constant `0`/`1`, or `off`.
  - *Warm-reset TB flush (`ingenic_tb_reset` in `hw/mips/ingenic.c`)*: Watchdog and `system_reset` flush all QEMU translated blocks. Without flushing, ASLR shared-library mappings from boot 1 accumulated up to 64k cached blocks per physical page on T40, slowing boot 2's overlay mount from 3 s to 32–43 s and causing RCU stalls when running `majestic`. Verified by `test-ingenic-boot.sh --reset` comparing two consecutive mount times in one QEMU process.
- **T41 PWM (`ingenic-pwm`, `0x13460000`, `pwm-ingenic.c` 4.4)**: Models `ENS`/`ENC` over `EN`, `WCFG` updates on `UPDATE`, clear `BUSYR`, `LVL`, `OEN`, and DMA register storage. Read pin state via `qom-get /machine/pwm chan2` (`high/period`, `0`, or `1`). Other SoCs use TCU channels for PWM.

### Interrupt controllers and CCU (`ingenic-intc`, `ingenic-ccu`)

- **`ingenic-intc`**: Two banks of `ISR`/`IMR`/`IMSR`/`IMCR`/`IPR` registers, all masked out of reset, raising `IP2` whenever an unmasked source is pending.
- **`ingenic-ccu`**: On XBurst2, mirrors core INTC views over shared interrupt sources alongside `PIMR`/`MIMR`/`OIMR` (`IP2`/`IP3`/`IP4` masks — core 0's INTC and OST bits start enabled at reset because guest code never sets them), IPI mailboxes, `CSSR` (bit `n` indicates core `n` halted), and core reset/entry registers.

### DDR controller (`ingenic-ddr`)

Acts as storage except for polled PHY/DDRC initialization status bits reading done: DWC PHY `PGSR` (T10/T20), Inno PHY PLL lock/init/calibration registers (exact `3` on T21–T31), T40/T41 calibration offsets, and self-clearing `DDRC LMR` start bit. Remap registers remap nothing; DRAM size comes from `-m`.

### GPIO (`ingenic-gpio`)

Models `S`/`C` set/clear aliases on all registers, shadow port Z (`GID2LD` applying masks), output level readback, pull-high undriven inputs, and one IRQ per port. External pin levels can be driven live via `/machine/gpio` properties (`pa`..`pf`), e.g. `qom-set /machine/gpio pc 0xfffffffe` pulls `PC0` low and triggers configured edge/level interrupts (`test-ingenic-gpio.sh` tests both edges on T31 sysfs).

### UARTs (paced 16550 and PDMA)

All UARTs (`0x10030000 + 0x1000*n`, UART0..3 or UART0..5 on T41) use `serial-mm` with 64-byte FIFOs, 4-byte strides, `+0x20` (`ISR`/`UMR`/`UACR`) storage, PDMA requests `0x14 - 2*n` (TX) / `0x15 - 2*n` (RX), and `EXTAL / 16` (`1.5 MHz` baud base; divisor `13` for `115200`):

- **Line-rate TX pacing (`qemu/patches/0021-...`)**: On T40/T41 Linux 4.4, `ingenic_uart.c`'s IRQ handler sends and stops TX without holding the port lock, racing `start_tx` on the second CPU core. Instantaneous QEMU writes returned `THRE` immediately and lost this race within a few hundred lines (`test-ingenic-audio.sh --soc t41 --console-load` stalled after lines 26, 101, and 279 in 3/3 runs). Enabling `paced` mode (`tx-fifo=64`, where the driver loads 32 bytes per IRQ) drains characters at real baud intervals (~11.5 KB/s), passing 3/3 500-line stress runs and 10/11 OpenIPC T40 boots (one still went quiet after app start, matching silicon's race window). Boot takes ~2 s longer and a 420 KB console flood takes 41 s vs 6 s; harnesses wait for drain before power-off.
- **Line-rate RX pacing and PDMA (`qemu/patches/0023-...`)**: Incoming bytes arrive at character intervals (~`2.8 ms` to fill a 32-byte burst at `115200` baud, keeping the window between reading DMA residue and halting the channel as narrow as on silicon) using `jz47xx` 64-byte RX FIFO trigger levels (`1`, `16`, `32`, `60`), standalone `IER` bit `4` (`RTOIE`), and `FCR` `DMS` mode (TX PDMA asserted whenever TX FIFO has room; RX PDMA asserted in bursts equal to the trigger level, holding shorter tails for `RTOIE`). Matches `jz_uart.c` under `CONFIG_SERIAL_JZ47XX_UARTn_DMA` (trigger `32`, 32-byte PDMA bursts into two 1 KiB buffers, `IER = 0x14`, no `RDI`). `test-ingenic-uart-dma.sh` passes 32 KiB loopback on `ttyS0` (3/3 runs), and exposed two kernel bugs: T31's `jzdma_pdata` only defines channels for UART0 and UART1 (so `UART2_DMA` dereferences a null channel), and driver close leaks/frees RX buffer pages improperly (`"Bad page state"` at close, so the test keeps the port open).

### I2C, SPI, ADC and stepper motors (`ingenic-i2c`, `ingenic-ssi`, `ingenic-sadc`, `motor=`)

- **`ingenic-i2c`**: `DW_apb_i2c` master with 8/16/32-bit accesses, instant command completion, `TXABRT` bit `0` on unaddressed devices, `ENSTA` tracking `ENB`, and PDMA handshakes (`DMACR` bit `0` `RDMAE` / bit `1` `TDMAE`, `DMATDLR`/`DMARDLR` at `0x88..0x90`, named and printed by both vendor drivers including `i2c-v12-jz.c`: TX asserted while `TDMAE` is set; RX asserted above `DMARDLR` for `DMARDLR + 1` `DATA_CMD` reads; checked by qtest since no driver enables I2C DMA). Image sensors sit on `I2C0` (or `I2C1` on T40).
- **`ingenic-ssi`**: SPI master (`jz_spi.c`, `ingenic_spi.c`) sending `FLEN + 2` bits MSB-first into a 128-entry RX FIFO (`CR0 LOOP` / `DISREV`, `CR1 UNFIN` raising `UNDR` vs `END` + CE release, `CR0 FSEL` selecting `CE`/`CE2` on cs `0`/`1`, e.g. `-device w25q80,bus=ssi0`, PDMA `0x16`/`0x17` SSI0 and `0x18`/`0x19` SSI1). Unused by shipped configs. The SSI slave (`0x10040000`) is unmodelled: although `JZ_SPI_SLAVE` appears in Kconfig and T31 platform definitions (and T40 datasheets list pins `SSI_SLV_CLK`/`CE0`/`DT`/`DR`), no 3.10/4.4 kernel, U-Boot, SDK sample, or any of the 51 vendor PDFs contains a driver or register contract.
- **`ingenic-sadc` (`jz_adc_v13.c`, `ingenic_adc_v13.c`, `jz_adc.h`)**: `ENA` bit `15` powers down; setting bit `n` converts AUX channel `n` in `20 us`, clears bit `n`, sets `STATUS` bit `n` (W1C), and interrupts unless masked by `CTRL`. Runtime levels (`0..4095`, `1.8 V` full scale, mid-scale default) are exposed at `/machine/sadc` as `auxN` (`qom-set /machine/sadc aux0 1234`). Inferred continuous DMA mode (`ADCFG` bit `15` `DMA_EN`, PDMA `SADC_RX` request `9`) scans enabled channels every `20 us` into an 8-word queue at port `0x10` (`AUX1:AUX0`, `AUX3:AUX2`, ...), dropping scans if full.
- **Stepper motors (`motor=`)**: Attaches `/machine/motor-chip` (inferred from SDK samples, verified by qtest; no shipped kernel enables `sample_motor3`'s SPI driver):
  - `ms41929` (`ingenic-ms41929`, `sample_motor3`'s `ms419xx_spi_dev.c` / `motor.c`) on `SSI0` CE: 1-byte address (bit `6` = read) + 16-bit LE register data (`0x20..0x2F`). Falling edges on `VD`/`VDFZ` (`PB25`) step channels `AB` (`0x24`) and `CD` (`0x29`) by `PSUM` (`bits 7:0`, direction bit `8`, channel-on bit `10`); `PB26` low resets. Read `pos-ab` / `pos-cd` via QOM.
  - `ms32006` (`ingenic-ms32006`, `sample_motor_i2c`) at `0x10` on `I2C1`: byte writes (`reg`, `val`; supports pointer write + no-stop data write); `0x00` chip-on (`bit 0`) and pause `A`/`B` (`bits 2`/`1`), `0x01..0x04` motor A 14-bit speed (`speed x 128` 24 MHz cycles) and 12-bit steps (`bit 15` output on, `bit 14` CCW), `0x05..0x08` motor B, `0x09` run `A`/`B` (`bits 7`/`6`, self-clearing) and IR-cut (`bits 3:2`). Counts down in virtual time; read `pos-a`, `pos-b`, and `ircut` via QOM.

### RTC, eFuse and OTG PHY (`ingenic-rtc`, `ingenic-efuse`)

- **`ingenic-rtc`**: `WRDY` always `1`, `WEN` follows `0xA55A` unlock (polled without timeout on power-off), tracks host seconds, `HCR.PD` powers off, `RTCCR.1HZ` sets every second, `RTCSR == RTCSAR` sets `AF` when `AE` is set, IRQ (`32` on T10–T31, `3` on T41, none named on T40) tracks `(AF & AIE) | (1HZ & 1HZIE)` (`rtc-jz.h`), and `HWCR.EALM` latches `HWRSR.ALM` to wake suspend (`sample-alarm`). T41 includes 4 KiB always-on RTC RAM at `0x10004000` (`pm_fastboot.c`).
- **`ingenic-efuse`**: Blank fuses with instant reads/programs, keeping 3.10's binary MXU license check permissive. Offset `0` (96-bit chip ID returned by `SU_Base_GetDevID`, `sample-chipid`, `/proc/jz/efuse/efuse_chip_id`) contains the SoC ID followed by `"OpenIPC!"` unless overridden via `-global ingenic-efuse.chipid0..2=`.
- **OTG PHY (`0x10060000`)**: Byte storage for 4.4 `cpm_usb.c`.

### Crypto and RNG engines (`ingenic-aes`, `ingenic-hash`, `ingenic-rsa`, `ingenic-dtrng`, `ingenic-des`)

Follow `sample_aes`, `sample_hash`, `sample_rsa`, `sample_dtrng`, `sample_des`, and Linux 4.4 crypto / `hw_random` drivers cleanly, completing immediately unless noted:
- **`ingenic-aes`**: Supports ECB and CBC modes with 128-, 192-, or 256-bit keys (`AES-128` verified against FIPS-197 and SP 800-38A) via DMA (`ASSA`, `ASDA`, `ASTC` in 16-byte blocks) or single-block `ASDI`/`ASDO`. Keys, IVs, and data words are big-endian (matching Linux 4.4's word swap), raising `KEYD`, `AESD`, and `DMAD` interrupts under `ASINTM`.
- **`ingenic-hash`**: Computes MD5, SHA-1, SHA-224, SHA-256, SHA-384, and SHA-512 over driver-padded DMA blocks (`HSTC` in 64-byte units), preserving intermediate state until `HSCR` bit `4` reloads it and outputting digests as little-endian words from `HSDO`. CPU mode via `HSDI` is inferred identically: memory-order writes raise `HSSR` bit `0` every 64 bytes (`128` bytes for SHA-384/512) and bit `1` after `HSTC` 64-byte units (no Linux 4.4 code uses `HASH_FLAGS_CPU`, `sample_hash` only prints `HSDI`, no U-Boot references `HASH_BASE`, and the 51 vendor PDFs omit it). `HSCR` bits `8:7` (byte order) are ignored because 4.4 writes `3`/`0` (SHA/MD5) while `sample_hash` writes `1`/`2` and both expect standard digests.
- **`ingenic-rsa`**: Computes $M^E \bmod N$ (`RSA-1024/2048`, selected by `RSAC` bit `7`) with MSW-first input and LSW-first output matching `sample_rsa`'s `test_rsa.h` vectors; polled (no IRQ wired).
- **`ingenic-dtrng`**: Populates host entropy `10 us` after `CFG` bit `0`; `CFG` bit `12` clears ready and bit `11` masks IRQ `34`.
- **`ingenic-des` (`0x10061000`, all SoCs)**: DES and triple DES (EDE with `K1`, `K2`, `K3`) in ECB/CBC (`DESCR2` bits `2`, `1`; bit `3` decrypts). Setting `DESCR1` work bit clears FIFOs and loads CBC IV; every second `DESDIN` word pushes a ciphered block into the 16-word output FIFO (`DESSR` bit `0` empty). PDMA (`DESCR2` bit `0`) uses `0x2E` (TX) / `0x2F` (RX). Keys and IV preserve word order (`L` high), while data words byte-swap under `DESCR2` bit `4` ("little endian") so FIPS 46's example (`85e813540f0ab405`) passes under `sample_des`.

---

## Storage, networking and Wi-Fi

### SPI flash controller (`ingenic-sfc`) and `ingenic-spinand`

`ingenic-sfc` models v1 (T10–T31) and v2 (T40, T41) in one device. `TRIG.START` executes command, 0..7 address bytes, dummy clocks, and data phases. SPL/U-Boot use PIO (`+0x1000` FIFO polling `RREQ`/`TREQ` and `END`), 3.10 uses interrupt-driven DMA, and 4.4 uses v2 command tables (`+0x800`, `LINK`, `COL`/`ROW`/`STA0`/`STA1`) + descriptor DMA. Key hardware details:

- **`END` vs FIFO drain**: `END` asserts when remaining read data equals one `GLB THRESHOLD` in words (at least 1), not when CPU drains it. Vendor U-Boots read desired words then wait for `END` (a new `START` discards unused trailing bytes), whereas T41's `jz_sfc_v1` takes one `SR` snapshot, reads `THRESHOLD` words on `RREQ`, and exits immediately if `END` is set — meaning an early `END` lost the last 64 bytes of every 2 KiB NAND page.
- **29-bit DMA addressing**: DMA addresses mask modulo 512 MiB because T41 passes uncached `virt_to_phys()` descriptor addresses (`0x2xxxxxxx` for `0x0xxxxxxx` RAM).
- **NOR flash (`m25p80`, patch `0024`)**: Adds 4 KiB (`20h`) and 32 KiB (`52h`) erase support (`W25Q64`, `W25Q256`, and `0xC22018` shared by `MX25L12805D/12835F/12845E`), `DMYBITS` fast-read clocks, and data reads past 6-byte probe addresses used by T41 SPL. Patch `0029` adds the Zbit ZB25VQ64 and ZB25VQ128 parts.
- **`POLL_EN` phase**: Polls status (`OIP`) and leaves the final byte in `DEV_STA_RT` (`0x0C`) for `P_FAIL`, `E_FAIL`, and ECC extraction.

**`ingenic-spinand` (`hw/block/ingenic-spinand.c`)**: One QOM type per part (`w25n01gv`, `gd5f1gq4ub`, `gd5f2gq4ub`, `mx35lf2ge4ab`, `gd5f1gq5ue`):
- **Commands**: `READ ID` (with or without dummy byte), page read to cache (`0x13`), cache reads (`0x03`/`0x0B`/`0x3B`/`0x6B` with 2-byte column + plane-select bit ignored by 1-plane chips despite 4.4 setting it from page number + 8 dummy clocks), program load (`0x02`/`0x32` clearing cache to `0xFF`), random load (`0x84`/`0x34`), program execute (`0x10`, requires `WEL`), block erase (`0xD8`), features `A0` (block lock on at power-up; any `BP` bit causes `P_FAIL`/`E_FAIL`), `B0` (`ECC`/`BUF`/`QE`), `C0` (status), `D0`/`F0`, write enable/disable, and reset. `busy-polls=n` keeps `OIP` set for `n` status reads.
- **ECC fault injection (`ecc-fail=`)**: Sets status ECC bits `5:4` to `10b` (uncorrectable) while ECC (`B0` bit `4`) is enabled for listed pages/ranges (`4470,16190-16191`), still transferring cache data. Matches OpenIPC/ipctool#283 where 3.10 `jz_sfc_nand` returns `-EBADMSG` from `mtd_read` (for `C8 51` and `D1..D4` GigaDevice chips; `B1..B4` expect `111b`), causing `mtdblock` `Buffer I/O error` while `/dev/mtdN` `read()` ignores it:

```bash
./qemu-src/build/qemu-system-mipsel -M t31,flash-file=nand.bin,flash-type=gd5f1gq5ue \
    -global gd5f1gq5ue.ecc-fail=4470,16190 -kernel uImage -initrd rootfs.squashfs ...
```

`gd5f1gq5ue` (`C8 51`, `GD5F1GQ5UEYIGR`, 136 MiB image size matching `gd5f1gq4ub`) is in Linux 3.10's table but absent from SPL and U-Boot, so it boots only via `-kernel` with `flash-type=`.

### SD/MMC controller (`ingenic-msc`)

XBurst1 `jzmmc` (`-drive if=sd,index=0|1`) supports U-Boot FIFO PIO and `jzmmc_v12` SDMA descriptors. Empty slots fail commands immediately so U-Boot passes `mmc` init quickly. `MSC0` card detect connects via GPIO (`PB27`, `PB20` on T21, or `msc0-cd=<gpio>` such as `61` for `PB29`), supporting monitor `eject`/`change` hotplug; `fatls` requires a partitioned disk image. XBurst2 uses SDHCI hosts.

### Ethernet MAC (`ingenic-gmac`)

Synopsys GMAC 3.x supporting V12, V13, T40 `ingenic_mac`, mainline `stmmac`, and U-Boot with 8-word enhanced descriptors (`DSL 2`, 40-byte spacing), self-clearing soft/MDIO/PHY resets, IP101G PHY (`0x0243/0x0C54`) at address `0` (`100 Mbit/s` full-duplex), appended RX FCS, and `DMA_RIWT` (`0x1024`) RX interrupt watchdog timer for mainline `stmmac` `RDES1_DIC` coalescing.

#### PHY power-down and reboot

To model OpenIPC/firmware#2839 accurately, the PHY retains its registers across TCU watchdog resets (`phy-cold-reset=on` resets it with the SoC instead) and `DmaBusMode.SWR` requires PHY RMII `REF_CLK`:
- Because T40 `ingenic_mac_close()` sets `BMCR.PDOWN` during `ifdown`, a standard `reboot` leaves `SWR` stuck on the next boot (`Bus Mode Reg after reset: 0x00020101` x10, `PROB:synopGMAC_reset timeout`, missing `eth0`), whereas `reboot -f` (skipping `ifdown`) keeps `eth0`.
- Writing `BMCR` without `PDOWN`, setting `BMCR.RESET`, or releasing reset GPIO clears pending `SWR` (`swr-needs-phy-clk=off` bypasses this check).
- Fixed kernels pulse `ingenic,rst-gpio` (`PC7`, active low) before `SWR`. Pass `-global ingenic-gmac.phy-rst-gpio=71` to wire `PC7` to PHY reset (default `-1` matches unpatched hardware). `qemu-boot/test-ingenic-eth-reboot.sh` checks both `reboot` and `reboot -f` (with `--expect-fixed` and `--phy-rst-gpio 71` for fixed kernels).

### SDIO Wi-Fi: AltoBeam ATBM6031 (`atbm-sdio`)

Attach AltoBeam ATBM6031/Apollo on MSC1 via `-M t31,wifi=atbm -nic user,model=atbm-sdio` (`/machine/atbm`), matching vendor `atbm_wlan` (`hal_apollo`, `PLATFORM_INGENICT31`). Uses patch `0026` on `jzmmc` (raising `IFLG` bit `7`) and patch `0027` on XBurst2 SDHCI ("Card Interrupt" level status):

- **SDIO / HIF / WSM**: Implements `CMD5/CMD3/CMD7`, CCCR, FBR1, ATBM CIS tuples, and `CMD52/CMD53` over 8 HIF registers (`CONFIG`, `CONTROL`, in/out queues, AHB/SRAM ports with base latch). Accepts firmware download into SRAM, releases the "CPU" when `CONFIG` reset clears, and runs the WSM AP + netdev bridge (`hw/usb/atbm-wifi.c` shared engine). Each SDIO WSM write packs a 28-byte prefix (8-byte bus + 20-byte cipher headers included in `len`) ahead of `{len, id}` on 256-byte boundaries and advances `NumOfHwXmitedAddr`.
- **Network & security (`hw/net/atbm-wpa.c`, shared with `atbm-wifi`)**: Machine options `wifi-ssid=` (card `ssid=`, default `OpenIPC`) and `wifi-psk=` (`8..63` chars) configure open or WPA2-PSK access:
  - *WPA2-PSK (CCMP)*: Beacons/probes set privacy + RSN IE and run IEEE 802.11-2016 12.7 4-way handshake (`PMK = PBKDF2-HMAC-SHA1(passphrase, SSID, 4096, 256 bits)`, `PTK = PRF-384(...)`, EAPOL-Key v2 `HMAC-SHA1-128` MICs under KCK, AES-key-wrapped GTK key ID `1` under KEK). Wrong MICs drop msgs 2/4 and deauthenticate after 4 retries (`1 s` interval, reason `15`). Verifies `WSM_ADD_KEY` keys match TK/GTK, strips TX CCMP 8-byte header + 8-byte MIC, and builds RX CCMP frames with rising PN flagged `AES-decrypted`.
  - *Card properties* (`-global atbm-sdio.X=` or `-device atbm-wifi,X=`):
    - `proto=wpa`: WPA1 vendor IE (`00:50:f2:1`), EAPOL-Key type `254`, plain IE in msg 3, separate GTK handshake after msg 4.
    - `cipher=tkip`: 512-bit PTK, 32-byte GTK, EAPOL-Key v1 (`HMAC-MD5` MIC, RC4 key wrap dropping the first 256 bytes of keystream), `WSM` key types `3`/`2`, strips 8-byte IV + 12-byte `icv_len` on TX, and prepends a TKIP IV (`TSC1`, WEP seed, `TSC0`, ExtIV + key ID, `TSC2..5`), a real Michael MIC, and 4 bytes of ICV room flagged `TKIP-decrypted` on RX.
    - `pmf=optional|required`: 802.11w `MFPC`/`MFPR`, `BIP-CMAC-128` group management cipher, IGTK (key ID `4`, `WSM` key `8`), and SA Query answers (no group robust management frames sent; requires RSN + CCMP).
    - `rekey=N`: Pairwise-protected GTK rekey every `N` seconds alternating key IDs `1`/`2`.
    - `selftest`: Runs KATs for PBKDF2 (`802.11 J.4.2`), PRF (`J.3`), AES key wrap (`RFC 3394 4.1`), HMAC-MD5 (`RFC 2202`), RC4, and Michael (`802.11i`).
- **Power save**: Handles `WSM_SET_PM` followed by indication `0x0809` while remaining awake (`iwconfig power` fails as `CONFIG_ATBM_SMPS` is off; use `iwpriv wlan0 common power_save,1` for mode `0x81`).
- **End-to-end & build notes (`test-ingenic-wifi.sh`)**: Passes open/WPA2/`--psk`/`--wrong-psk`/`--proto wpa`/`--cipher tkip`/`--pmf`/`--rekey`/`--powersave` with `mmc1: new SDIO card`, `wpa_supplicant` (`key_mgmt=WPA-PSK`), `udhcpc` (`10.0.2.15`), and `ping 10.0.2.2`. Build T31 driver with `platform=PLATFORM_INGENICT31 ATBM_WIFI__EXT_CCFLAGS=-DATBM_WIFI_PLATFORM=24` (`--driver`). For `--soc t40`, rebuild OpenIPC `t40.generic.config` with MSC1 enabled (`status = "okay"`, `ingenic,sdio_clk` for `sdhci-ingenic` `jzmmc_manual_detect()`, no `non-removable`), stub the three `CONFIG_WIRELESS_EXT` tx-power ioctls in `PLATFORM_INGENICT41`, and include the driver's `wireless/cfg80211.ko`. Verified by `test_sdio` and `test_sdio_sdhci` qtests too.

### SDIO IoT Wi-Fi: AltoBeam ATBM6441 (`atbm6441-sdio`)

Models ATBM6441 IoT mode for battery doorbells such as the Merkury MI-CW074-125W (OpenIPC/firmware#1974), where credentials and DHCP lease persist in the module across host sleeps (`-M t31,wifi=atbm6441 -nic user,model=atbm6441-sdio`, `/machine/atbm6441`, matching `gtxaspec/atbm6441` `ATBM_WIFI_PLATFORM=22`):

- **HIF & startup**: Same HIF addressing (`register << 2`) without firmware download. Setting `CONTROL WUP` emits 4 trace indications discarded by the driver before startup indication seq `4` (firmware `13679`, 16 x 2048 B input buffers). Queue reads pad to 256 bytes with next `CONTROL` in the last two bytes.
- **Framing & flow control**: Host `{flag, len, id}` packets (flag checksum checked) align to 256 bytes; event `3` returns consumed buffer credits + window (only synchronous commands get confirmations, not data/`NETWORK_OK`/IPC data).
- **Network & AT commands**: `GET_CONFIG` returns MAC, STA link on `wifi-ssid=`, derived BSSID, open network, and `ip=`/`gw=`/`dns=` properties (which the driver exposes on `/dev/atbm_ioctl`). After `NETWORK_OK`, raw `ToDS`/`FromDS` 802.11 LLC/SNAP frames bridge to the netdev. `/dev/atbm_ioctl` `ATBM_FW_AT_CMD` (`WSM 0x0023`) logs under `-d unimp`/`trace=on` and emits synthetic `<command>\r\nOK\r\n` events.
- **Tests (`qemu-boot/test-ingenic-wifi-6441.sh`)**: Runs `S96wifi`'s `jzmmc_v1.2.1/present` scan (`cfg80211` alone or `--mac80211`), ping/TCP fetch, `lsmod`, AT ioctl via `qemu-boot/atbm6441-at.c`, and `rmmod`/`insmod` reload. Qtest checks PID `0x6441` and rejects `wifi-psk=`.

### USB OTG (`dwc2-usb`) and USB Wi-Fi (`atbm-wifi`)

Uses `hw/usb/hcd-dwc2.c` configured for Ingenic `drivers/usb/dwc2`:
- **`otg-a-device`**: Clears `ConIDSts` and raises a single connector ID status change `100 ms` after first soft reset (long ID debounce), enabling `cpm_usb.c` to bring up port power and root hub polling.
- **`host-channels=16`**: Vendor DWC2 ignores `GHWCFG2` and dispatches across `MAX_EPS_CHANNELS = 16` round-robin (QEMU's default 8 dropped every 9th transfer: `"unable to read config index 0 descriptor/start: -145"`).

Works with `atbm-wifi` (`docs/xiongmai.md`), e.g. `-device atbm-wifi,bus=usb-bus.0,netdev=w,pid=0x888b,chip-version=0x4a,efuse=0x0500` reproducing OpenIPC/firmware#1655 (`ATBM6012B-X` on T31L). `qemu-boot/test-ingenic-wifi-usb.sh` validates `atbm6012b_wifi_usb.ko` (model `6012B`, `t31_ultimate`, open/`--psk` WPA2-PSK, `10.0.2.15`, ping/TCP, `lsmod`). The issue's other driver, `atbm60xx` built for the 603x over USB (`atbm603x_wifi_usb.ko`, `--module`), loads and never binds because its ID table lists only `007a:8888`, leaving `wlan0` absent just as reported.

---

## Audio: `ingenic-pdma`, `ingenic-aic`, `ingenic-dmic` and `ingenic-asdmic`

Vendor `audio.ko` (`drivers/audio/oss2` on T10–T31, `oss3` on T23/T40/T41; `/dev/dsp` for `IMP_AI`/`IMP_AO`/`IMP_DMIC`) transfers FIFO audio using cyclic PDMA descriptors. Attach `-audiodev wav,id=snd,path=out.wav -M t31,audiodev=snd` to record guest playback.

### Programmable DMA (`ingenic-pdma`)

32 channels (`0x20*n` at `0x13420000`: `DSA`, `DTA`, `DTC`, `DRT`, `DCS` [`NDES`, `DES8`, `CDOA`, `AR`, `TT`, `HLT`, `CTE`], `DCM` [`SAI`/`DAI`, port widths, `TSZ` including 4.4 `auto` byte count, `TIE`, `LINK`], `DDA`; plus `DMAC`, `DIRQP` W1C, `DDRS`, `DIP`/`DIC`) following `jz_dma.c` (3.10) and `ingenic_dma.c` (4.4):

- Fetches 8-word descriptors from `DDA`. At `DTC == 0`, linked descriptors load the next descriptor (`DDA` page + `DOA` in `DTC` top byte) without interrupting (continuous audio rings); unlinked ones set `TT` and `DIRQP` if `TIE` is set.
- Request `8` (`AUTO`) copies memory at once; peripheral requests pace every access: AIC (`6` play, `7` record, `5` loop echo ref), DMIC (`13`), UART (`0x14 - 2*n` / `0x15 - 2*n`), SADC (`9`), I2Cs (`0x24 + 2*n` / `0x25 + 2*n`), SSI, and DES.
- Masks addresses to 29 bits (`0x20000000` alias reaches DDR, since `jz_uart.c` passes `virt_to_phys` of uncached KSEG1 RX buffers like T41 SFC). MSC uses SDMA (`jzmmc_v12.c`), I2C/SADC/SSI slave have no kernel DMA client, and no PCM driver exists in the vendor 3.10/4.4 trees.

### AIC and internal codec (`ingenic-aic`)

Models `AICFR` (`ENB`, `RST`), `AICCR` (`EREC`/`ERPL`, flushes, `M2S`, `MONOCTR`, `TDMS`/`RDMS`, `ISS`/`OSS`, `TLDMS`/`ETFL`, `CHANNEL`, `PACK16`), `AICSR` (levels, `TUR`/`ROR`), `AICDR`, `AICLR`, 64-entry TX FIFO, and 32-entry RX/loop FIFOs:

- **Clocking & pacing**: On T23, T31, T40, and T41 (AIC I2S master), CPM speaker (`I2SSPKCDR`/`I2STCDR` `0x70`) and mic (`0x84`) dividers run at `256 * fs` (`parent PLL * M / N`, snapped within 3% to standard rates). On T10–T30 or with CPM clock gated, codec `CSRR` (`0x110[2:0]`, `8`–`96 kHz`) sets both directions.
- **Audio flow**: While `ENB` is set, frames pace in virtual time: TX writes 1 (`M2S`) or `CHANNEL + 1` samples per frame to `audiodev` (16-bit stereo) and copies TX into `loop` (`ETFL`). RX receives 1 (`MONOCTR`, right channel on T21/T31) or 2 samples: quarter-scale sine (`-global ingenic-aic.tone-hz=N`, default `1000`, `0` silence) or host audio (`-global ingenic-aic.rx-source=audiodev`). Requests trigger for TX (`TDMS` + space), RX (`RDMS` + data), and loop (`TLDMS` + data).

Codec register bank (`0x10021000`, byte-per-word, `1.5 dB` analogue gain steps, muted when off, supporting `IMP_AI_SetGain`/`IMP_AO_SetGain`; digital gains scale in software via `IMP_AI/AO_SetVol`):

| SoCs | Playback enable & gain (`unity`) | Record enable, ALC & gain (`unity`) | Digital vol reset (`0 dB`, 0.5 dB steps) |
|---|---|---|---|
| T10–T30 (`jz_t10_codec.c`) | `CAR` `0x94[3:0]`; `CHCR` `0x9C[4:0]` (`0x18`) | `CAMPCR` `0x90` bit `7`; `CACR2` `0x8C[4:0]` (`0x10`) | — |
| T23, T31 (`inner_codecs/T23/codec.c`) | `CHR` `0xA0` bit `7`; `CHR` `0xA0[4:0]` (`0x18`) | `CMICCR` `0x8C` bit `7` (bit `4` ALC holds level); `CALCGR` `0x94[4:0]` (`0x10`) | — |
| T40, T41 (`inner_codecs/t40`, `T41/codec.c`) | `CHR` `0xA8` bit `6`; `CHR_2b`/`CHPOUTLGR` `0xAC[4:0]` (`0x18`) | `CMICCR` `0x8C` bit `7` (T40 right ADC `0x0E` bit `3`); `CGAINR` `0x28` bit `5` (T40) / `4` (T41); `CANACR_26`/`CALCGR` `0x98[4:0]` (T40 right `0x9C`; unity `0x10` T40 / `0x1F` T41) | `CDGSR` `0x18` (`0xF1`); `CLAVR` `0x20` (T40 right `0x24`, `0xC3`, `0` mutes; assumed undocumented reset) |

### Digital microphones (`ingenic-dmic` and `ingenic-asdmic`)

- **`ingenic-dmic` (T31, T40)**: Matches `audio_dmic.c` (`oss3`, `dmic_enable=1` on T40) and `oss2 devices/dmic` (`CONFIG_JZ_TS_DMIC`, off in shipped kernels). Supports `CR0` enable, rate (`8`/`16`/`48 kHz`), mic count, `PACK_EN` (`bits 15:0` earlier sample), 64-entry `DR` FIFO, `FTHR RDMS` (PDMA `13`), `FSR` (`FULL`/`EMPTY`), `INTCR`/`IMR`. Synthesizes `-global ingenic-dmic.tone-hz=N` (`1 kHz`) at peak `tone-amp` (`8192`, adjustable via `qom-set /machine/dmic tone-amp N`). Other XBurst1 chips have no DMIC driver.
  - *Low-power voice trigger (`drivers/char/voice_wakeup`)*: With `EN` + `TRI_EN`, compares samples against `THRL`: `TRICR` mode `0` fires immediately, mode `2` after exceeding `TRINMAX` (other modes act as `0`), setting `INTCR TRIFLG` (bit `0`, IRQ unless `IMR` bit `0`) and `FSR TRIGS` (bit `18`) until `TRI_CLR` or self-clearing `RESET_TRI`. `SR 3` ("data path off") continues clocking trigger logic at `16 kHz` without pushing FIFO data (`THRH`/`TRIMMAX` storage; T41 ASDMIC has empty `TCR` and no trigger).
- **`ingenic-asdmic` (T41 `0x134D0000`, core INTC `33`, `host/T41/dmic` `dmic_enable=1`)**: Registers as platform device (`ingenic,dmic` DT disabled). Connects DMIC (`+0xA000`, `8/16/48/96 kHz`, `16/24-bit`, `GCR` storage), data bus (`+0x4000`), format converter (`+0x2000`, storage), FIFO 5 (`+0x1000`), and DMA ch5 (`+0x0` closed ring of 4-word `{flags, DBA, next, DTC}` descriptors polled or interrupting on `TIE`/`LTIE`, `AIPR`). Synthesizes `-global ingenic-asdmic.tone-hz=N` (`1 kHz`).

---

## Image pipeline: `sensor=`, `ingenic-isp`, `ingenic-isp-apical`, `ingenic-ivdc` and `ingenic-ipu`

### Supported sensors

Pass `-M <soc>,sensor=<name>` (on `I2C0`; `I2C1` on T40; `sensor-bus=<n>` overrides `-1` default for custom boards like OpenIPC/firmware#1884). On T40, `sensor1=` attaches pipe 1's sensor on `I2C3` and `sensor2=` attaches `VIC2`'s sensor on `I2C0`. T10 uses `i2c-gpio` (`PA12` SDA, `PA13` SCL) wired open-drain through `ingenic-gpio` drive outputs onto `gpio_i2c`. Sensors respond without waiting for `MCLK` or `PA18` reset so `sinfo.ko` (`/proc/jz/sinfo/info`) always identifies `sensor: gc2053`.

| Sensor(s) | I2C address | Model / driver ID | Notes |
|---|---|---|---|
| `gc2053`, `gc2083`, `gc1084` | `0x37` | `galaxycore-sensor` | `gc2083` and `gc1084` use 16-bit registers; `gc1084` is T23's 720p part (openingenic `t23/gc1084.c`) |
| `gc4653`, `gc4663` | `0x29` | `galaxycore-sensor` | 16-bit registers; T40 (`I2C1`) and T41 |
| `sc2336`, `sc2335`, `sc1a4t`, `sc2336p`, `sc2337p` | `0x30` | `hisi-smartsens` | Single/main sensor |
| `sc1a4ts1`, `sc2336ps1`, `sc2337ps1` | `0x32` | `hisi-smartsens` | Secondary sensor (`sensor1=`) |
| `jxf37`, `jxf22`, `jxf23`, `jxq03` | `0x40` | `hisi-f37`, `hisi-jxf22`, `hisi-jxf23`, `hisi-jxq03` | `JX-F22` (`jxf22`), `JX-Q03` (`jxq03`, T31 `2304x1296` MIPI) |
| `jxh62` | `0x30` | `hisi-jxh62` | T10 sensor |
| `ov2735b`, `ov2735` | `0x3C` | `ingenic-ov2735` | Paged 8-bit registers; page `0` reg `0x04` selects variant |

*OpenIPC `load_ingenic` bug*: Release `sinfo.ko` prints `sensor: gc2053` with a space after `:`, making `${SENSOR_INFO#*:}` evaluate to `" gc2053"` and fail `modprobe sensor_` on boot 1 (boot 2 works using saved env). Pre-seeding `sensor=` in U-Boot's environment works around this (`qemu-boot/test-ingenic-isp.sh`).

### T21, T23, T31, T40 and T41 ISP (`ingenic-isp`)

Matches `tx-isp-t31.ko` (`H20221206a` + SDK 1.1.5 symbols), `tx-isp-t23.ko` (SDK 1.1.0 + OpenIPC `H20240204a`), `tx-isp-t21.ko` (`H20200509a`), `tx-isp-t40.ko` (`H20220606a`), and `tx-isp-t41.ko` (`H20230630a`). Rather than reading sensor frames, each frame renders the synthetic test pattern (`sstar-pattern.h`, `ingenic-isp-paint.h`: OpenIPC logo on navy gradient, sliding dot, 6-patch grey step wedge bottom-left, 6-digit top-left frame counter on black box) into queued NV12 buffers.

- **CPM handshake**: Sets `SRBC` (`0xC4`) bit `21` and polls bit `20` (immediately satisfied).
- **ISP core & MSCA DMA-out channels**:

| Feature | T21 | T31 | T23 (`0x90000` core) | T40 (pipe 0 / pipe 1) | T41 |
|---|---|---|---|---|---|
| IRQ en / sta / clr | `0x80` / `0x84` / `0x88` | `0xB0` / `0xB4` / `0xB8` | `0xB0` / `0xB4` / `0xB8` (`0xC0`-`0xC8` dual) | `0x40020`/`28`/`24`, `0x40030`/`0x40038`/`0x40034` | `0x40020` / `0x40028` / `0x40024` |
| MSCA enable & ch base | `0x2304` (`0x2308` stop=0); `0x2400 + 0x100*n` | `0x9804`; `0x9900 + 0x100*n` | `0xD040` (`0xD050` NV21/mirror/flip); `0xD100 + 0x100*n` | `0x16008`/`0x17008`; `0x16100`/`0x17100` | `0xF0008`; `0xF0100 + 0x100*n` (`+0xF0000` = `0x133F0000`, mmio 4) |
| QBUF push Y / UV | `C+0x6C` / `C+0x84` | `C+0x6C` / `C+0x84` | `C+0x30` / `C+0x50` | `C+0x6C` / `C+0x84` | `C+0x6C` / `C+0x84` |
| ISR pop Y / empty / meta | `C+0x74` (`C+0x78` fnum) / `C+0x7C` | `C+0x74` (`C+0x8C`/`90`) / `C+0x7C` | `C+0x38`/`C+0x3C`, `C+0x58`/`C+0x5C` | T31 offsets | T31 offsets |
| Size / strides | `C+0x2C` / `C+0x80`, `C+0x98` | `C+0x2C` / `C+0x80`, `C+0x98` | `C+0x0C` / `C+0x40`, `C+0x60` | T31 offsets | T31 offsets |

On T31, writing `C+0x9C..C+0xA8` clears channel FIFOs; status bit `n` marks channel `n` done (`0x1000` frame start).

- **VIC & CSI**: Commands `2` and `4` to `0x00` clear at once; `1` starts input. Status / mask (`1` = masked) / W1C sit at `0x1E0/0x1E8/0x1F0` (+4 bank 2; T21 `0x100/0x104/0x108`), incrementing `/proc/jz/isp/isp-w02` on bit `0` frame done for `libimp`'s "video drop" watchdog (T40 VIC0 bit `4` triggers sensor/AE work). On T23, `VIC_PORT_SEL` (`0x204`) locks `VIC_PORT_STATUS` (`0x20C`), and `VIC_MDMA` packs 10-bit BGGR (`<< 6`). CSI and D-PHY (`0x10023000`/`0x10022000`, `0x10054000` T40 pipe 0) act as storage reading `0` on `ERR1`/`ERR2` (T21 is DVP-only without CSI/PHY).
- **Frame clock & lighting**: Runs at sensor frame time (`HTS * VTS / pclk`) or `-global ingenic-isp.fps=` (`25`) with `-global ingenic-isp.expo-delay=<n>` (or `qom-set /machine/isp expo-delay <n>`, `0..8`). Scene brightness follows `light` (`100`%), `light-r`, and `light-b` (linear scaling before gamma `2.2` encoding and white clipping); T23/T31 apply the WB gains at `0x1804`/`0x1808` to output chroma. Statistics use the scene before white balance.
- **Tracing**: `-global ingenic-isp.trace=on` (MSCA/3A + frame summary), `trace-regs=on` (`ingenic-isp: core [0x...] = 0x...`), and `-global galaxycore-sensor.trace=on`.

### Closed-loop 3A (AE, AWB, AF) and extended statistics

Exposure scales linear scene brightness by `sensor rows x gain` (normalized against `1000` rows at gain `1.0`) multiplied by the ISP digital gain:

- **Sensor exposure registers** (`include/hw/i2c/ingenic-sensor.h`): `gc2053` uses rows `0x03`/`0x04` and gain registers `0xB8`/`0xB9`; `gc2083` uses `0x0D03`/`0x0D04`; `gc4653` and `gc4663` use `0x0202`/`0x0203` and `0x02B8`/`0x02B9`; `sc2336` uses `0x3E00`–`0x3E02` and `0x3E07`/`0x3E09`; `jxf37` uses `0x01`/`0x02` and `0x00`; and `ov2735b` uses page `1` registers `0x03`:`0x04` and `x16` gain at `0x24`.
- **ISP digital gain**:
  - On T23 and T31, `tisp_ae0_process_impl` (`JZ_Isp_Ae_Dg2reg`) writes fractional gain to `0x1030`/`0x1034` (`G` sits in the upper halfword of `0x1030`, with `0x400 = 1.0`). Without this feedback, `gc2053`'s coarse `1.18x` analogue gain steps hunted every 5 frames between `2272/256` (`1.21x` digital gain) and `2683/256` (`1.0x`); with it, AE locks smoothly at `2283/256` for 153 frames of a 200-frame run.
  - On T21, `tisp_ae_process_impl` writes `0x408`/`0x40C`, holding `129/256` (mean `8`) from frame 65 onward.
  - T40 and T41 apply digital gain elsewhere and converge directly.

#### AE0 and AWB DMA statistics

`tisp_init` configures the 15 x 15 (`225`-zone) AE0 and AWB statistics blocks using shared grid registers:

- **Common layout**: `+0x04` holds the zone grid (`[31:28]` columns, `[15:12]` rows = 15 x 15); `+0x08`–`0x14` and `+0x18`–`0x24` store per-zone widths and heights in `2x2-pixel` quads (one byte each); `+0x4C` holds bank counts; `+0x50` holds the last bank written (AE zone bank in `[5:4]`, histogram bank in `[1:0]`).
- **AE0 specific registers**: `+0x28` stores dark (`[7:0]`) and bright (`[15:8]`) thresholds, `+0x2C`–`0x38` hold four zone banks, and `+0x3C`–`0x48` hold four 256-bin histogram banks containing `21-bit` quad luma counts (stored twice). `ae0_weight_mean2` divides each zone's sums by `width x height`.
- **Per-SoC differences**:

| Feature | T21 (`0x84` bits `16/17/18`) | T31, T23 (`0xB4` bits `26/27/30`) | T40, T41 (`0x40028` bits `4/5/3`) |
|---|---|---|---|
| AE0 / AWB base | `0x700` / `0x800` | `0xA000` / `0xB000` | `0x19000` / `0x18000` |
| AE0 format | `16` B/zone, 20-bit histogram; quarter-scaled sums (settles at mean `7` vs T31 `10`, preventing T21 from sticking at linear mean `2` of `255` regardless of light) | `16` B/zone (21-bit dark/mid/bright sums + 13-bit counts), `8x6` sampling | 22-bit sums, 14-bit counts |
| AWB zone record | `16` B (`R, G, B` sums + count at bit `63`; no IR sum) | Banks at `0x3C`–`0x48`; `16` B (`R, G, B, IR` 21-bit sums + 13-bit count) | `128-byte` zones: 4 pairs of `16-byte` records read by `tisp_awb_get_statistics` (`R, G, B, 4th` 22-bit sums at bit `0/22/44/66`, 14-bit count at bit `88`). Tuning bytes `3274`/`3276` select a pair; AWB reads the second record (filtered quads) while the model also fills the first (all quads). |

#### AWB pixel filters and settled gains

- **T21 / T23 / T31 filter (`tiziano_awb_set_hardware_param`)**: Programs T21 `0x828`–`0x838` or T23/T31 `0xB028`–`0xB038` with Q8 limits (`min` `[15:0]`, `max` `[31:16]`) on `R/G` (`+0x28`), `B/G` (`+0x2C`), diagonal bounds `k * R/G / 256 + B/G` (`+0x30`/`+0x34`), and `(R + 2G + B)/4` luma (`+0x38`, `[7:0]` low / `[15:8]` high), vs the driver's all-pass defaults (`R/G 1..4095`, bounds `0` and `65535`).
- **T40 / T41 filter (`0x18028`)**: `tisp_awb_set_regional_threshold` and `tisp_awb_set_lum_th_freq` program tighter EV-interpolated windows (three luma bytes + frequency in `+0x38`): T41 uses `R/G 60..245`, `B/G 30..200`, `k = 256` bounds `170`/`320`, luma `10..230`; T40 uses `R/G 53..284`, `B/G 29..304`, bounds `167`/`394`, luma `0..230`.
- **T21 `Tiziano_awb_fpga` / `Tiziano_Awb_Ct_Detect`**: Reconstructed from `swapper_pg_dir` gdbstub inspection: `_AwbPointPos[1] = 0` runs every frame with static tuning gains `1526`/`1583` (Q10), drops zones with count `<= _pixel_cnt_th` (`25`), projects zones onto the CT band (`R/G 180-400`, `B/G 260-148`) above luma floor `50`, and uses fallback `R/G 167, B/G 340`.
- **Grey wedge & light tracking**: Because metering for the dark navy background (`Y ~90` on T40/T41, 12x past logo full scale vs `1.2%` navy) clips the logo outside the R/G and luma limits, the scene's bottom-left 6-patch neutral grey step wedge ensures valid white zones. Out-of-window lights (such as `R 0.35 / B 0.9`) hold default start gains (`2212 / 1856`). On T21, `10%`, `100%`, and `1000%` light all settle at mean `7`; on T31, `light 30%` and `100%` both converge to navy `Y 39` by frame 40.

| SoC | Settled AWB gains (`per 1024`) |
|---|---|
| **T21** (`JX-F37`) | Daylight (`550/600`): `R 1860 / B 1712` · Warm (`R 0.9 / B 0.35`): `R 1140 / B 2740` · Cool (`R 0.45 / B 0.75`): `R 2272 / B 1360` |
| **T31** | Daylight: `1860 / 1704` · Warm: `1136 / 2916` · Cool: `2280 / 1364` |
| **T23** | Daylight: `1856 / 1704` |
| **T40** | Daylight (`460/550`): `2220 / 1864` · (`500/430`): `2048 / 2380` · Warm (`700/280`): `1468 / 3636` · Cool (`380/600`): `2688 / 1712` (within 1–3% of `1024/R`, `1024/B`) |
| **T41** | Daylight (`460/550`): `2228 / 1856` · (`500/430`): `2056 / 2388` · Warm (`700/280`): `1468 / 3704` · Cool (`380/600`): `2716 / 1704` |

#### Extended statistics (`ingenic_isp_xstats()` / `xstat 3`)

| Block | T31 / T23 (`0xB4` bit) | T40 / T41 (`0x40028` bit) | Structure and driver behavior |
|---|---|---|---|
| **AE1** | `0xA800`, bits `28`, `29` (`0x0C` bit `0` clear) | Inside WDR block | Short-frame AE (`system_reg_write_ae(2)` latch bit `0`). Note: if `ae1_interrupt_hist` fires while T31 is in linear mode, it freezes AE/AWB at frame 10. |
| **AF** | Grid `0xB804` (`238x134` at 1080p `8x8`), `0xB8A8`, last `0xB8BC`; bit `31` | Grid `0x1A004`, widths `0x1A008` (u16), heights `0x1A028` (u8), banks `0x1A0CC`, cnt `0x1A0DC`, last `0x1A0E0`; bit `6` | T31 packs 16 B/zone (`FIR0/1`, `IIR0/1` 22b, luma 23b, bright 15b; `IMP_ISP_Tuning_GetAFMetrices` ~`1.6M` falling to `0.9M` when warm/clipped). T41 packs 136b/zone (row stride `2 * {3,5,7,9,11,13,15,17,20,22,24,26,28,30,32}[cols-1]` words; `IMP_ISP_Tuning_GetAFMetricesInfo` ~`3.4M`, alt `~1.8M`). T40 single-sensor `tisp_af_get_metric` faults in module space. |
| **ADR** | `0x4494` / `0x44B0` (`0x0C` bit `7` bypass); bit `18` | Banks `0x9514`, cnt `0x9510`, last `0x9540`; bit `9` | T31: `6x8` blocks + 512-bin (`21b`) hist, writes `0x4270`–`0x43A4`. T40/T41: `4x6` blocks of 56 B (`29b`, 20-bin `18-bit` counts, `12b` min/max) + 512-bin hist at `+0x548`, writing `0x50024`–`0x5040C` (fields inferred). |
| **Defog** | `0x5B84` / `0x5BA4` (`0x0C` bit `11` bypass); bit `20` | Banks `0xC290` (`2 KiB`), cnt `0xC28C`, last `0xC2B4`; bit `10` | Starts at `+8` with 180 blocks: `B, G, R` 24b + 3x16b (meaning guessed) on T31 (`0x58000` output); 8 B `21-bit` sums on T40/T41 (`0xC0xx` output). |
| **WDR** | `0x2010` (`8 KiB`), last `0x2680` (`0x0C` bit `0` clear); bit `11` | `0x6010..` (`6 KiB`), last `0x6030`; bit `11` | T31 uses eight `516-byte` rows `1 KiB` apart; see WDR below for T40/T41. |
| **GSM** | — | Write `0x201` to ports `0x501A0`/`0x501B0`, read `0x501A8`/`0x501B8`, flags `0x1B004`/`0x1B044`; bit `7` | `tisp_gsm_single_get_hist` / `tisp_gsm_process`: two 256-bin histograms required to sum to `width x height` (`0x80`). |
| **LCE** | — | Ports `0x501C0`/`0x501C8` (`45x32` words) + `0x50200`/`0x50208`, 2nd set `+0x10`, `0x0E014` bit `0` ready / bit `8` set; bit `8` | `tisp_lce_get_data` / `lce_std_hist_transform`: `9x5` blocks x 32-bin quad luma histograms + block means. |
| **TMO** | — | Banks `0x1E010`/`0x1E014` (`16 KiB`), last `0x1E028`; bit `17` | T41 only: `3750` halfword pairs (`75x50` mean and brightest 12-bit luma). |

#### WDR: two exposures

Two-exposure DOL line-interleaved WDR runs on T31 (`data_type=2` / `TX_SENSOR_DATA_TYPE_WDR_DOL` plus `IMP_ISP_WDR_ENABLE` between `IMP_ISP_Open` and `IMP_ISP_AddSensor`), T40, and T41 (`IMP_ISP_WDR_ENABLE(IMPVI_MAIN, &mode)` plus `default_boot 1`).

- **Sensor and ISP setup**: `tisp_init` clears bypass bit `0` (`0x0C`: `0xa1fc0202`, `0x40`: `0xff9100e0` in WDR vs `0xffd00049` in linear mode). `galaxycore_exposure()` reads both the long (`0x0202:0x0203`) and short (`0x0200[5:0]:0x0201` via `gc*_set_integration_time_short`, shared gain) registers on `gc4653` and `gc4663`.
- **Scene fusion and statistics**: `ingenic-isp` illuminates both exposures independently, feeds short-exposure statistics to AE1 and the WDR block, and blends unclipped long pixels with ratio-scaled short pixels through an extended Reinhard curve (`isp_wdr_tone()`). On T40/T41, `tisp_wdr_get_data` (driven by `tisp_wdr_interrupt_static` / `tisp_wdr_main_interrupt_static` on `0x40028` bit `11`) reads:
  - `456` 24-bit values (`15x15` long + short blocks + 450 threshold),
  - four 12-bit and two `23-bit` values at `+1352`/`+1360` (mean luma, bright and dark counts),
  - `160` packed words at `+1368` unpacked into an unused `640-byte` buffer (kept zero), and
  - long and short RGB histograms at `+1848` and `+3896` (256 x 3 `21-bit` counts).
- **Convergence and results**: T31 updates 134 of 229 WDR registers (`0x2000`–`0x28FF`) per frame; T41 writes ~29 registers/frame. `tisp_ae1_process` hands statistics to `tisp_wdr_rx_ae1_infm`, while `tisp_ae1_expt` (`ae1_ev_list`) drives short exposure (`7`–`94` rows on T41; `gc4653-t31.bin` holds 310 rows across `1486` down to `4` long rows). Logo pixel clipping drops from `7.5%` in linear mode to `0.4%` in WDR (`test-ingenic-isp.sh --wdr`). T23 WDR sensors (`imx327`, `sc301IoT`) are not modelled.

#### HLDC

T41 (`tx-isp-t41.ko` `tiziano-hldc.c`, `tisp_hldc_write_reg`) and T40 (`tisp_hldc_con_par_cfg`, `tisp_s_module_control` `(pipe+16) << 2`) enable hardware lens distortion correction when `0x40` (`0x44` on pipe 1) bit `20` (`IMPISPModuleCtl.bitBypassHLDC`) is clear (`_IO('t', 1)` via `/dev/tisp`: module `21` sub-module `0` on T41 since stock omits `TX_ISP_LDC_ENABLE` / `tisp_hldc_set_attr`; module `20`, pipe word `3`, 16 halfwords from word `4` on T40).

| Offset (pipe 0) | Parameters | Notes |
|---|---|---|
| `0x15000` | `p1` (`[31:16]`), `p0` (`[15:0]`) | Stored as `1, 1` (`0x2A0 << 7 = 0x15000` on T40) |
| `0x15004`, `0x15008` | `k2` (`[31:16]`), `k1` (`[15:0]`) | Radial distortion coefficients |
| `0x1500C`, `0x15010` | Zoom (`Q11`: `2^19 / min(f(1), f_side) - 5`) | Output scale factor |
| `0x15014`, `0x15018` | `0x3fffffff / r2max` | `r2_rep = 498` at `2560x1440` |
| `0x1501C`, `0x15020` | `offy` (`[31:16]`), `offx` (`[15:0]`) | Optical center offset |
| `0x15024`, `0x15028`, `0x15044` | `p14`, `p15`, `3` | Pipe 1 sits `+0x80` on (`0x15080`); T40 inputs at `0x80`/`0x84` |

`tisp_hldc_calc_para` computes `rho = r^2 / r2max` and `f(rho) = 256 - k1*rho/4 + k2*rho^2/4` (`Q8`). The inferred sampler reads bilinearly at `d * f(rho)/256 * zoom/2048`. Verified by `test-ingenic-isp.sh --ldc` (`95%` match on T41 vs 157 uncorrected levels, `100%` on T40 vs 111 levels, and `95%` on T40 dual-pipe `--sensor1 gc4653` across `ch0`/`ch3`).

### Multi-sensor modes and VIC raw capture

- **T40 dual-sensor (`sensor1=`)**: Attaches a second sensor on `I2C3` and `CSI1` (`0x10023000`, behaving like `CSI0` storage) for `IMPVI_SEC` / `IMPISP_TOTAL_TWO` / `DUALSENSOR_MODE` (`tisp_sec_*`, `sensor_gc4653s1_t40.ko`, and `/etc/sensor/gc4653s1-t40.bin` via `tisp_sec_init`). `VIC1` (`0x13390000`, `INTC 29`, queuing `sec_fs_work`) mirrors `VIC0` 64 KiB past it (the driver only maps `VIC0`'s 64 KiB and indexes past it based on the incoming IRQ). Pipe 1 interrupts use `0x40038`/`0x40034`/`0x40030` (core ISR `0x72f10` polls `0x40028` then `0x40038`, while `0x40068`/`0x400A8` stay `0`), MSCA channels `3`–`5` sit at `0x17100 + 0x100*n` (`0x17008` enable), and AE/AWB banks sit at `0x19200`/`0x18200`. Pipe 1 renders `+500000` frames ahead (`test-ingenic-isp.sh --soc t40 --sensor1 gc4653`).
- **T40 third sensor (`sensor2=`)**: Attaches on `I2C0` for `IMPVI_THR` / `IMPISP_TOTAL_THR` using DVP `VIC2` (`0x133A0000`, `INTC 28`) after `csi_phy_start` logs `"Can not support more than 2(3) mipi sensor"` and `"The third sensor init"`. There is no third ISP pipe or tuning (`/proc/jz/isp/isp-w02` shows `"vic index: 0, 1, 2"`). `VIC2` raises frame-done interrupts while pipes 0 and 1 stream (`test-ingenic-isp.sh` aliases `s1` to `s2`).
- **T40 `snapraw` (`VIC_MDMA`)**: `echo snapraw S N > /proc/jz/isp/isp-w02` captures `/tmp/snapK.raw` via `vic_mdma_enable` (`0x304` `w<<16|h`, `0x308=1`, strides `0x310`/`0x314`, buffer slots `0x318+4k` / `0x340+4k`, `0x300` count `<= 8`, `0x380` buffer index, and secondary status `0x1E4`/`0x1EC`/`0x1F4` triggering `vic_mdma_irq_function0`). Packs a 10-bit-in-16 RGGB mosaic (`R 186, G 216, B 335`). `ispvic_frame_channel_qbuf` is unused.
- **T40 split-joint mode (`IMPISPDualSensorSplitJoint`, `ispcore_pad_event_handle`)**: Modes `3` and `4` (vertical above/under stacking of channel 0 and channel 3 into a `2560x2880` frame) pass (`--joint 3` / `--joint 4`). Horizontal modes `1` and `2` exceed the scaler limit with two `2560px` sensors (`5120px` total width: `"Image width cannot exceed 3840!"`).
- **T23 dual-sensor (`tx-isp-t23-double` `H20230726b`, `libt23-firmware.a`, `sensor1=gc2053s1` at `I2C0 0x3f`)**: Alternates both sensors through T23's single pipe via pass-end `0xC8` bit `15` (enabled by `0xC0 = 0x8000`, acknowledged by `0xC4`; single-sensor firmware never touches `0xC0`–`0xC8`). On each pass end, `ispcore_interrupt_service_routine` advances `vi_num = count & 1`, reloads registers (`tisp_rcfg_cfg_trig`), queues buffers, enables channels (`tisp_msca_chx_en`), and restarts VIC (`tisp_msca_channel_start(n)` maps frame channels `0-2` and `3-5` to `n % 3`). The model alternates passes starting with the main sensor (inferred) and treats `tisp_ae_multi_sensor_malloc_cfg` (`+0x4C = 0x10010000`) bit `16` as the AE enable (since `0xA000` is never written), converging both AEs (`mean Y` `0` to `32` in 60 passes). HELIX distinguishes each stream's reference by chain addresses (`MCE 0x5006C` / `DBLK 0x60008`) instead of picture dimensions (`test-ingenic-t23-enc.sh --sensor1 gc2053s1`: four H.264 1080p/360p streams).

### IVDC direct mode (`ingenic-ivdc` on T41 and T23)

ISP-to-encoder direct buffer mode (`direct_mode=1` in `tx-isp`, `--dm 1` / `bEnableIvdc` in `libimp`, `"T41 IVDC Module Application Guide"`) replaces DRAM frame buffers with the fixed pair `0x80000000` / `0x81000000` (pitch `4096`), backed by 32 MiB of RAM at `0x80000000`-`0x81FFFFFF`. The model's buffer never overflows, so spill line counts at `+0xEC`/`+0xF0` remain `0` in `/proc/jz/isp/isp-ivdc`.

- **T41 (`0x13200000`, core INTC `13`, `tx-isp`'s `isp-ivdc` IRQ vs DT `jpeg0` IRQ)**: T41's `start_param` script loads `tx-isp-t41.ko` with `direct_mode=1` and runs the camera app with `--dm 1`. The ISP writes `0x80000000`, signals frame source on `int_sta` bit `20` (and bit `13` when `ivdc_vb_type = 1`), and AVPU reads `0x80000000`-`0x81FFFFFF` unmasked. `ingenic-ivdc` auto-clears `0x00` bit `0`, mirrors `0x200` bit `0` in `0x204` (`ispcore_video_s_stream` / `ispcore_suspend_module`), and sets `+0x44` bits `17` and `16` under `+0x40 = 0x244FC0FF` (`ivdc_core_ops_init`, cleared via `+0x54`), recording ~`102` frames in `test-ingenic-sdk-boot.sh`.
- **T23 (`tx-isp-ivdc.c`, `0x13200000`–`0x1320FFFF`, Linux IRQ `46` = INTC `38`, build `0x65cf8`)**: Uses the same flow except frame start (`0x1000`, or `0x2000` with `ivdc_vb_type = 1`) signals frame source, and `0xD060` + the top 3 bits of `0x1C` configure HELIX reads. Verified by `test-ingenic-t23-enc.sh --direct 1` (`~100` frames, `T23ENC_AE_FREEZE`). Passing `--sdk` (`tx-isp-t23.ko H20230720a`) requires `2.10` tuning files (`gc2053-t23.bin`) because rootfs `2.20` files trigger a `kfree` BUG in `tiziano_load_parameters`.
- **T23 double direct (`direct_mode=2`, `--direct 2`)**: When streaming two sensors directly through IVDC, the model pulses IVDC frame-done first and ISP pass-end `0.5 ms` later so `INTC 37` (`ispcore_interrupt_service_routine`) does not advance `vi_num` before `INTC 46` (`ivdc_core_interrupt_service_routine`) delivers `0x80000000` (preventing `"vb is null, buf get:0x80000000"` and unset `isp_index` errors). Sensor 0 main (`stream 0`) and both sub-streams pass `10/10` frames; sensor 1 main (`framesource 3`) still stalls because `IMP_FrameSource_EnableChn` never queues buffers (`buf_qcnt 0` in `/proc/jz/isp/isp-fs`) despite `enc_chn->index=1, direct_mode = 2` and `ispcore_pad_event_handle` setup. ISP register writes to `0x20000`–`0x7FFFF` past 64 KiB are dropped.

### Apical ISP (`ingenic-isp-apical` on T10, T20, T30)

Models OpenIPC `tx-isp-t20.ko` (`H201709151519` / Apical `H01-380`), `tx-isp-t10.ko` (VIC at `0x20`, config `0x00`, size `0x0C`, `1280x960` max), and T30 `tx-isp-t30.ko` (`H20190725b` with MSCA at `0x130B0000` IRQ `27` and three CPM resets in `0xAC`):

- **Core (`0x13380000`–`0x133FFFFF`)**: Storage (`0x1B000` mapped via `/dev/mem`) plus 16 interrupt slots (`0x80 + 4*(slot/2)`, acked via `0xA0`/`0xA4`: frame start `24`, end `23`, DMA writers `14`/`15`/`1`, `DS1`/`DS2` `50`/`53`) and handshake state `0x130` -> `0x134`.
- **Three DMA writers (`0xB00 + 0x100*n`, CbCr `+0x80`)**: Size at `+0x04`, five bank addresses at `+0x08`, bank count at `+0x1C`, stride at `+0x20`, control at `+0x24` (`2` active vs `3` drop; current bank in `[10:8]`).
- **VIC (`0x13300000`), combiner (`0x13320000`, IRQ `37`), CSI (`0x10040000`, `PHY_STATE 0x14 = 0x630`)**: Combines event slots (`15:0`) and VIC frame-done (`16`) through `0x0C`/`0x04`/`0x14`/`0x1C`.
- **3A statistics and firmware algorithms**: `apical_fw_interrupts_init` assigns AE to event `11` (`256-bin` floating histogram at `0x10000 + 4*b`, decoded by `ae_read_full_histogram_data` as `m` when `e=0` else `(m | 0x1000) << (e - 1)`) and AWB to event `9` (`15x15` = `225` zones at `0x8740 + 8*z`, read by `awb_read_statistics` from `AWB_fsm_clear`: R/G `[11:0]`, B/G `[27:16]`, weight `+4`). AE settles by frame 67 on T10 (still rising at frame 75 on T20/T30); AWB gains at `0x300`/`0x30C` track light color from `B 1.00 / R 2.75` (daylight) to `B 1.16 / R 2.24` (warm), traceable with `-global ingenic-isp-apical.trace-core=on`.
  - *AF / Iridix / DIS*: AF (`event 14` slot `5`, `af_status = 2`) and `af_mode`/`af_range_*`/`af_roi` act as stub storage. Iridix (`iridix_fsm_process_interrupt`, `0x2C0` bit `7`) adjusts `0x39C`/`0x3DC` from exposure steps; DIS (`event 15`, `dis_update_stats`, `0x95C0 + 8*i` for the `0x908` `8x4` grid) reads `0` motion vectors.
- **OpenIPC `tx-isp-t30.ko` upstream fault**: Although both SDK and OpenIPC report `H20190725b`, SDK `opensource/drivers/tx-isp-t30` skips `apical-isp/apical_isp_io.c` and links `apical_isp_io.c.o` from `apical-isp/libapical_isp.a`, where `APICAL_READ_32(addr)` routes `addr < 0x1FFFF` to `system_isp_read_32()` and `0x50100`–`0x501F7` to `apical_ext_sytem_mem` using `.bss+0x1B98` (initialized by `system_isp_set_base_address()`). OpenIPC (GCC `13.3`) compiled `apical_isp_io.c` instead, whose `APICAL_READ_32` (`.text+0xF60`) reads `apical_io_base` (`.bss+0x490`, only written by uncalled `apical_iobase_init()`) while probe stores the base in `.bss+0x35F8`, oopsing at `00014000` (`epc : APICAL_WRITE_32+0xc/0x18 [tx_isp_t30]`). Work around this with the SDK module (`test-ingenic-isp.sh --soc t30 --isp-ko <SDK>/opensource/drivers/tx-isp-t30/tx-isp-t30.ko` + `sensor_ov2735b_t30.ko`), or fix upstream by dropping `apical_isp_io.o` (calling `apical_iobase_init(base)` alone still lacks the `0x501xx` RAM window).

### Image Processing Unit (`ingenic-ipu`) and ISP test harnesses

- **`ingenic-ipu` (`0x13080000`, INTC `6` / IRQ `14`, `jz_ipu_v13` / `ingenic_ipu_v13`)**: `IOCTL_IPU_START` resets (`TRIGGER` bit `3`), configures the background frame (`IN_FM_GS`, `Y/U_ADDR`, strides, `OSD_CH_BK_PARA`) and OSD channels (`PARA` at `0x11C + 4*n`, supporting ARGB8888 `[17:14]`, ARGB1555 `[13:11]=4`, NV12/NV21, `POS`/`GS`, alpha `[10:3]`/`[2:1]`, CSC `0x148 + 0x18*n`), enables `GLB_CTRL IRQ_EN`, starts (`TRIGGER` bit `0`), and raises `OUT_END` `100 us` later (`-global ingenic-ipu.debug=on`). Also performs NV12->NV21, RGB (`D_FMT` `0x00` ARGB, `0x28` ABGR, `0x40` RGBA, `0x68` BGRA with `CH_BK_GLOBAL_ALPHA`), and HSV conversions (`D_FMT 0x4` / `PIX_FMT_HSV`: 16-bit H at `NV_OUT_ADDR`, 8-bit S/V at `OUT_ADDR`/`OUT_V_ADDR`, value ranges unconfirmed).
- **`qemu-boot/test-ingenic-t30-rtsp.sh [IMAGE]`**: Boots T30X SDK `1.0.5` (`8 MiB`, `193.169.4.151`, forwarding `8554`), captures `rtsp://127.0.0.1:<port>/main` (`1920x1080` + IPU timestamp OSD) and `/second` (`640x360`) using `ffmpeg -rtsp_transport tcp -err_detect explode`, and decodes offline after capture so host decode latency never drops LIVE555 P frames.
- **`qemu-boot/test-ingenic-isp.sh`**: Boots OpenIPC NOR flash (`mem=40M rmem=24M`), runs `qemu-boot/ingenic-isp-snap.c` (capturing `1920x1080` and `640x360` NV12 frames via SD on XBurst1, or `/dev/mem` + monitor `pmemsave` on T40 before IMP teardown hangs), and verifies frames using `qemu-boot/ingenic-isp-check.py` (`--png` saves images). Compiles against OpenIPC uClibc `libimp.so` (`1.1.6` T31, `1.1.0` T23, `1.0.33` T21, `1.2.1` T20; GCC 7.2 NaN2008 `IMPVI_NUM` on T40) on musl with `mmap`/`ftruncate` (`off_t`) + ctype shims, `/tmp/closesimd`, and per-SoC structs (`IMPFSChnAttr` front crop in `1.1.6`; T23 `IMPSensorInfo`).

### majestic on T40 and T41: a digital twin

`qemu-boot/test-ingenic-majestic.sh` runs the streamer OpenIPC ships, or a
build of it (`--majestic BIN`), to reproduce what an owner reports without
the camera:

- `--soc t40` (the default): a T40N with a GC4653
  (`t40,variant=t40n,sensor=gc4653 -m 128M`) on the `t40` lite release's
  kernel and root.
- `--soc t41`: a T41NQ with a GC4663 (`t41,variant=t41nq,sensor=gc4663
  -m 128M`), in the memory split of the T41NQ in OpenIPC/majestic#376
  (`mem=74M@0x0 rmem=45M@0x4A00000 nmem=9M@0x7700000`). That camera's
  sensor is a GC5603, which the twin does not model and the SDK image has
  no driver for: the GC4663 is the twin's sensor, not the owner's, and
  the GC5603 is not covered. OpenIPC has no T41 release, so the
  root is still the `t40` lite release's -- the same musl userland, and
  T41's libimp is uClibc-built like T40's -- with the T41 SDK 1.2.0's
  `libimp`, `libalog` and `libsysutils`, and the vendor modules and the
  sensor setting out of its 16 MiB T41NQ image (`--sdk`, or `T41_SDK`; the
  test skips without it). `load_ingenic` is a stand-in that loads them
  with the parameters measured on that camera (`tx-isp-t41.ko
  clk_name=mpll isp_clk=300000000 direct_mode=0`). majestic must be a
  `t41` build.

The camera boots as a camera does. OpenIPC's `/init` mounts the JFFS2
`rootfs_data` partition as the overlay, busybox init runs the release's rc
scripts, `S70vendor` loads the vendor modules, and `S95majestic` starts
majestic under its supervisor with a `majestic.yaml` the test writes (H.264
on both channels, JPEG, `colorToGray`, the IR-cut pins, and a light sensor
on PB17). What majestic saves is saved, and survives a power cycle
(`--reboot`). T41 needs a kernel with overlayfs for that, and the SDK's
has none: OpenIPC's `/init` then leaves the root read-only, which no
camera does. `qemu-boot/build-ingenic-t41-kernel.sh` builds the SDK's own
4.4.94 (`isvp_tomcat_defconfig`) with `CONFIG_OVERLAY_FS` and nothing else
changed, so the SDK's prebuilt modules still load; the test builds it on
first use. The twin adds three things: `S00twin` gives seedrng a
creditable seed on every boot and turns libimp's MXU2 paths off,
`rc.local` copies the syslog lines the checks look for (`majestic -s`
logs there) to the console, and root has a password, since an unclaimed
camera streams nothing. Only those lines: with the whole syslog on the
console the twin froze about one boot in three. The SDK 4.4 kernel's
`ingenic_uart.c` turns its transmit interrupt off from the interrupt
handler without the port lock, racing the other core's start; a race lost
with the tty buffer full leaves the console stopped for good (UART1 `IER`
`0x15`, `LSR` `0x60`, text queued), `logread` blocks on it, syslogd on
`logread`, and majestic and dropbear on syslogd until the watchdog resets
the camera. The race is the driver's, on silicon as well; two vCPU threads
lose it far more often than two cores do, and running both on one
(`-accel tcg,thread=single`) left the twin too slow to stream.

The checks:

- majestic brings the SDK up -- with `--audio`, with the microphone on,
  which OpenIPC/majestic#257 was about (the T40 build stopped at
  `IMP_AI_SetChnParam() failed`; a release from before the fix fails this
  check here the same way, and CI runs the test with `--audio`);
- with `--audio` on `t41`, the speaker as well: `audio.ko spk_gpio=-1
  spk_level=-1` is loaded as on the owner's T41NQ, the speaker plays into a
  wav audiodev, and a 1 s 440 Hz tone POSTed to `/play_audio` must come out
  for at least 0.8 s at 440 Hz. A build from before OpenIPC/majestic#376's
  audio fix fails it: every frame refused, and the clip silent. The
  microphone too: `/audio.pcm` must stream for at least 3.5 of 4 s, with
  the codec's 1 kHz tone in every half second of it,
  and the vendor driver must never have timed a read out (`get mic
  timeout!`; libimp records nothing after one). That timeout was the
  encoder model's: coding a 2560x1440 picture under the iothread lock froze
  both CPUs for up to 0.8 s while the virtual clock ran on, past the 800 ms
  the driver waits (the AVPU section below), and 2 of 3 runs failed until
  the coding moved to a thread of its own. The check cannot tell IMP_AI
  device 0 from 1: the twin's `audio.ko` records the codec on both (a build
  that records on device 0 passes it), where the owner's camera gave
  silence on 0. T40 keeps its speaker off: its
  `audio.ko` stops recording after about a second, and with the output on
  too its streams stalled after a power cycle;
- 50 pictures of each RTSP stream decode -- ffmpeg's own count, exit
  status 0, nothing at error level -- and `/image.jpg` is a JPEG of the
  main stream's size;
- the host pulls PB17 low (`qom-set /machine/gpio pb 0xfffdffff`), and
  majestic changes to night; it lets it go, and majestic changes back.
  The test reads that from `night_enabled` in `/metrics`, not from the
  console, which carries only the lines the checks look for, and whose
  `logread -f` stops following once syslogd's ring laps it, as the twin's
  `audio.ko` stack dumps make it do on T40 with `--audio`;
- the change reaches the ISP. `-global ingenic-isp.trace-regs=on` logs
  every write to the ISP core (and the VIC) on stderr, and the test counts
  the core registers the night window writes with values the ten seconds
  of day before it did not: about 980 on the current release, the night
  tuning set. A build whose switch never reached the ISP
  (OpenIPC/majestic#258: the call handed back the mode it had just read)
  logs "Change mode to NIGHT" all the same and writes 8 to 15, the
  per-frame curves' drift. `--min-night-regs` (100) is the line. T41
  writes about 560;
- a setting written through the API reads back, and with `--reboot` it
  is still there after QEMU is killed and booted again from the same
  flash. The setting is `rtsp.keyframeOnSkip`, which majestic applies
  without rebuilding anything, so what this tests is the flash: a
  setting that rebuilds the pipeline would hang on T40, where the IMP
  teardown hangs here (majestic logs the reload and never starts the SDK
  again, and the HTTP server stops answering; T41 tears down and starts
  again).

A T41NQ can report the T40 family's SoC ID: an owner's reads
`0x1300002c = 0x10040003` with subtype `0xAAAA`, and ipctool then takes it
down its T40 branch, where `0xAAAA` has no name. `variant=t41nq` reports
that ID; the other T41 grades keep `0x10041003`.

The picture itself does not turn grey: the model paints the scene from
the exposure alone (above), so the twin shows that the night settings are
written, not what they do to the colour.

The machine gathers almost no entropy (`jitterentropy` refuses it), and
the kernel's pool never initialises on its own: majestic would wait in
`getrandom()` before it reads its configuration, for good. `S00twin`
gives the release's busybox `seedrng` a creditable seed on every boot --
the seed seedrng saves for the next one is not creditable here, so after
a power cycle the pool would never be ready; it is ready about a minute
into the boot, and the SDK a second after that.

With the microphone on, the release's `audio.ko` and `libimp.so` record
for about a second -- majestic gets 20 ms frames of the codec's 1 kHz tone
(peak 9191 at 8 kHz) -- and then the driver prints `get mic timeout!` and
`Error: Not Find any irq`, and libimp's record thread ends (below, "Not
modelled").

---

## Video encoders, display and NNA accelerators

### VPU, HELIX and RADIX (`ingenic-vpu`)

Models T10/T20 `jz-vpu.0`, T21/T23/T30 HELIX, T30 RADIX (`kind 1`), and T41 JPEG (`layout 1`) via `/dev/soc_vpu` (`drivers/video/soc_vpu/`, documented with evidence in `tmp/soc-vpu-contract.md`):

- **Execution**: The kernel resets via CPM `SRBC` (bits `30`/`29`/`31` for VPU/HELIX, `27`/`26`/`28` for RADIX, and `28`/`27`/`26` for T41 JPEG) and submits a VDMA chain of `{value, VLD | TERM | register}` entries (written to `0x84` on T21/T23, VDMA `0x10008` on T20/T30, and CFGC `0x08` on RADIX) terminated by `EFE_CTRL RUN` (VDMA `0x8008` on RADIX). Jobs complete `1 ms` later in virtual time, retaining all chain writes in the ~40 statistics registers read back by `libimp` (`H264E_T21_SliceInit`, etc.).

| Codec | Software (`libimp`) | Hardware model (`ingenic-vpu`) |
|---|---|---|
| H.264 | Writes `SPS`, `PPS`, slice header, `cabac_alignment_one_bits`, and NAL escaping | Writes unescaped CABAC `slice_data()` + flush at `SDE_CFG4` (`SDE_CFG9` byte count) |
| H.265 | Writes `VPS`, `SPS`, `PPS`, slice header, `byte_alignment()`, and whole-NAL escaping | Writes unescaped `slice_segment_data()` + trailing bits at `SDE 0x68008` (`0x68024` byte count) |
| JPEG | Writes `SOI`..`SOS` and `EOI` markers | Writes entropy-coded scan at `JPGC_BSA` (`JPGC_STAT[23:0]` byte count) |

- **Coders and inter prediction**: Uses `hantro-h264.c`, `hantro-h264-inter.c.inc`, `hevc-enc.c`, `hevc-enc-inter.c.inc`, and `jpeg-enc.c` (`QMEM` reciprocals), taking QP and slice type from `EFE_CTRL` (RADIX `0x20040`/`0x20000`). `libimp`'s `x264` requires CABAC (`i264e_init` rejects CAVLC).
  - *H.264 P slices*: Selects `P_Skip`, `P_L0_16x16` (testing predicted, `P_Skip`, neighbor, and zero vectors -> `4-pixel` grid over `+-16` pixels -> diamond search -> quarter-pel luma H.264 `8.4.2.2.1` and `6-tap` / averaging eighth-pel chroma bilinear interpolation over `num_ref_idx_l0_active = 1` using `8.4.1` vectors, followed by a `4x4-transform` residual + CBP against the deblocked `disable_deblocking_filter_idc = 0` reference), or `I_16x16 DC` on scene cuts. Honors High-profile PPS `transform_8x8_mode_flag` (`transform_size_8x8_flag = 0`; override with `-global ingenic-vpu.transform-8x8=off`), retries at `+6` QP on `SDE_CFG5` overflow (`mb_qp_delta`), and uses DC residuals on AE steps so CBR lowers QP (`31 -> 25` over 20 frames in `test-ingenic-vpu-e2e.sh`) instead of stalling at QP 51 `I_PCM`.
  - *H.265 P slices (`i265e`)*: Matches T30 streams (CTB `64`, `16x16` `cu_qp_delta`, chroma QP `-3`, luma-only SAO, no scaling lists, no TMVP, `max_transform_hierarchy_depth_inter = 0`, no sign hiding, 2 merge candidates via `five_minus_max_num_merge_cand = 3`), retrying `+6` QP above 1 byte/pixel down to all-skipped (`test-ingenic-t30-rtsp.sh`).
- **DMMU TLB (`IOCTL_CHANNEL_PRIVATE_TLB`, `libdmmu.c`)**: Reads the page directory base from `SCH_TLBA` (`0x30`), gated by `GLBC` bits `26:21` (`JPGC`, `DBLK`, `SDE`, `EFE`, `VDMA`, `MCE`; bit `30` on older layouts) or RADIX `CFGC 0x10`/`0x1C` (bits `16:21`: `EFE`, `VDMA`, `ODMA`, `DBLK`, `MCE`, `SDE`). Walks a 1024-entry page directory (`[31:22]`) to 1024-entry page tables (`[21:12]`, bit `0` valid, PA `[31:12]`); unmapped addresses raise `SCH_STAT` bit `10`, read as `0`, and drop writes.
- **JPEG decode (`IMP_Decoder` on T23 & T30)**: `ijpegd_decode_header` submits `T20_JPEGD_SliceInit` ending in `JPGC_TRIG = 7` (without EFE), outputting `T420` `16x16` tiles (`P0A + 256*k`, `P1A + 128*k` of 8 Cb then 8 Cr rows; non-4:2:0 MCU shapes inferred) before `c_copy_frame_t420_to_nv12` zeroes partial trailing rows:

| Register | Offset | Role |
|---|---|---|
| `GLBI` | `0xE0004` | Components - 1 in `[5:4]`, restart markers every `NRSM + 1` MCUs (`bit 2`) |
| `BSA` | `0xE000C` | Byte-stuffed entropy scan (retaining restart markers and `EOI`) |
| `P0A`, `P1A` | `0xE0010`, `0xE0014` | Output luma (`P0A`) and chroma (`P1A`) `T420` tile buffers |
| `NMCU` | `0xE0028` | Number of `16x16` MCUs minus `1` |
| `P0C`..`P3C` | `0xE0030..` | Per-component blocks/MCU - 1 (`[7:4]`), QT (`[3:2]`), AC (`[1]`), DC (`[0]`) table selectors |
| `MCUS` | `0xE0064` | Non-zero decoded block count; raises `JPGC_STAT` / `SCH_STAT` bit `4` |
| `HUFB` / `HUFM` / `QMEM` / `HUFS` | `0xE1200` / `0xE1300` / `0xE1400` / `0xE1800` | `9-bit` `HUFS` starts (`HUFB`), `36-bit` top-packed canonical codes (`HUFM`: `AC0`, `DC0`, `AC1`, `DC1`), `4 x 64` zigzag quantizers (`QMEM`), and Huffman symbols (`HUFS`: `AC0` @ `0`, `AC1` @ `174`, `DC0`/`DC1` nibbles @ `162` with `DC0` low) |

- **T41 standalone HELIX JPEG (`0x13280000`, `ingenic,t41-jpeg`, `drivers/video/ingenic_vpu/helix`, core INTC `0`)**: Selected via `IMP_ENC_TPYE_IVPU` (`ijpege_*`, instead of `AVPU` core 1; reset via SRBC `28`/`27`/`26`). Maps renumbered 64 KiB `helix.h` `HID_*` units (`SCH = 0`, `VDMA = 1`, `EFE = 2`, `JPGC = 3`, remainder `7`, address bit `19` set) back to HELIX offsets (`JPGC_STAT` at `+0x30008`). Tested via `test-ingenic-avpu.sh --soc t41 --ivpu` (`2560x1440` + `640x360`).
- **Tests**: `qemu/tests/test-ingenic-vpu.py`, `qemu/tests/test-ingenic-vpu-feed.py` (`i264e_*`, `i265e_*`, `ijpege_*` via `qemu/tests/ingenic-vpu-feed.c`), `qemu/tests/test-ingenic-vpu-e2e.sh` (T21 FrameSource -> `IMP_Encoder`), and `qemu-boot/test-ingenic-jpegd.sh --soc t23|t30` (`qemu-boot/ingenic-jpegd-test.c`).

### Allegro AVPU (`ingenic-avpu` on T31, T40, T41)

An Allegro DVT video/JPEG encoder proxied by `avpu.ko` (`opensource/drivers/avpu`, IRQ `70` = INTC `62`) and scheduled in userspace `libimp.a` (`CoreManager.c`, `EncSliceParam.c`, `SliceHdrWriter.c`; contract in `tmp/media-contract.md` section 3). Only 4 KiB is mapped from `reg-base` (`+0x8000` on T31/T40 since T40's RTC occupies `0x132A0000`; `+0` on T41). Core `c` sits at `+0x200*c` (`0` = video, `1` = JPEG), and UID+`0x8004` returns `hw-id` (`0x72000460` T31, `0x72000760` T40, `0x71000260` T41) to satisfy `"Board ID mismatch: ... instead of 72000460"`.

Encoding runs on a worker thread beside the vCPUs. Coding a `2560x1440` frame under the iothread lock held both CPUs for up to `0.8 s` while virtual time advanced, exceeding the vendor `audio.ko` microphone read timeout of `800 ms`. The worker accesses guest RAM only: register reads return zero and register writes are dropped and logged as guest errors. Device callbacks from the worker could otherwise deadlock with a vCPU waiting for that batch.

- **Gen 1 register interface (T31, T40)**:
  - `0x14` / `0x18`: Interrupt mask and W1C status (`4c+0` = Enc1 or JPEG done; `4c+2` = Enc2 done).
  - `0x30` / `0x3F8`: Ticking `800 MHz` counter (polled non-zero) and busy flag (always reads idle).
  - `0x3E0` / `0x3E4`: 29-bit-masked command list (`0x200-byte` entries ending on word `0` bit `31`, up to 2 Enc1, 8 Enc2, or 1 JPEG entry reading `R0`–`R10` at `0x200`–`0x228` and writing byte count at `0x234`) and launch trigger. Jobs finish after at least `200 us`, once coding completes, with non-zero RC counters and tracking of `pcm-budget`. Supports both Enc1-only slicing (`W8` bit `11`, `WC0`–`WCC`, length `0x104`, offset `0x220`) and two-pass Enc2 (`WF0`–`WFC`, length `0x1E4`).
- **T41 Gen 2 interface (`gen=2`, `libimp` 1.2 `EncCmdRegs.c`)**:
  - Uses per-core 4 KiB pages at `0x1000*(c + 1)` (`Enc1` at `+0`, `Enc2` at `+0x800`): command list lo/hi (`+0`), status area (`+8`), control (`+0x10`), start (`+0x18`: `2` video, `1` JPEG), busy (`+0x1C`), and IRQ done bits `2c` (Enc1/JPEG) / `2c+1` (Enc2).
  - Each `4 KiB` command entry places Enc1 status at `0x5C0` (`gen 1` `0x104`–`0x174` shifted by `-0x118`) and Enc2 status at `0xDC0`:

| Gen 2 offset | Mapping / Fields | Gen 2 JPEG (`core 1`) |
|---|---|---|
| `W0` (`+0x00`), `W4` (`+0x04`) | `W0`: codec `[1:0]`, `log2(LCU) - 4` `[26:24]`, last `[31]`; `W4`: `width/8 - 1` `[10:0]`, `height/8 - 1` `[26:16]` | `0x40` `R0`, `0x44` size |
| `0x0C`, `0x60`/`0x64`/`0x68` | Maps to Gen 1 `W8` (CABAC, `cabac_init`, Enc1 slice) and `WC`/`W10`/`W14` | `0x58` tables, `0x60`/`0x68`/`0x6C`/`0x70` stream |
| `0xA0`/`0xA8`, `0xD0`, `0x1C0`, `0x108` | Source luma/chroma (`0xA0`/`0xA8`), pitch (`0xD0`), reconstruction (`0x1C0`), reference (`0x108`) | `0x74` pitch, `0x80`/`0x88` source |
| `0x220`/`0x22C`/`0x230`/`0x234`, `0x800`–`0x814` | Stream base/size/offset/room and Enc2 entries | `0x604` byte count written |

  - `avpu_g2_entry()` maps Gen 2 entries back to Gen 1 fields and initializes T41 HEVC SPS/PPS defaults (scaling lists on, transform skip off, intra transform depth `1`).
- **Bitstream headers and inter/B coding**: `libimp` writes `SPS`/`PPS`/`VPS`/`SEI` and the slice header after encoding completes, and the hardware model writes the slice payload starting at `stream base + offset` (re-emitting the `0x78`/`0x7C` header tail with EPB). The coders recover parameter sets only from consecutive complete `SPS + PPS` headers preceding the stream offset so stale fragment buffers (such as T40's `transform_skip_enabled_flag = 0` leftovers) are ignored.
  - *H.264 (`hantro-h264-inter.c.inc`)*: Supports Main-profile CABAC (`cabac_init_idc = 1`, multi-slice rows via `W14`, `W10` alpha `[13:8]` / beta `[4:0]` deblocking with `disable_deblocking_filter_idc = 0`, `W90`/`WA0` references) and Baseline CAVLC (`mb_skip_run`, Exp-Golomb `mb_type`, `coded_block_pattern` Table `9-4`, `residual_block_cavlc` with neighboring `nC` across 4 `coeff_token` tables). B slices (`W3[29:28] = 0`, `RefA = WA0`, `RefB = WA8` from `ChannelMngr.c` `encode1` `TSliceBuffersEnc`) code `B_L0_16x16`, `B_L1_16x16`, `B_Bi_16x16`, or `I_16x16 DC` (never `B_Skip`/`B_Direct_16x16`, with `num_ref_idx_active = 1/1`), verified by qtest because `PYRAMIDAL` `Codec_Encode_SetGopParam` fails inside 64 MiB `libimp`.
  - *HEVC (`hevc-enc-inter.c.inc`)*: Uses CTB `32`, intra depth `1`, transform skip (T31/T40), and `cabac_init_flag`. P slices enable real `inter` motion search (whole/half/quarter-pel with `8.5.3.3.3` 8-tap luma / 4-tap chroma filters, 5-candidate merge list with TMVP, AMVP `rqt_root_cbf`, default `8x8`/`4x4` scaling lists, `residual_coding()`, and QP capped `<= 22` instead of rate control climbing into the `40s` so 6-sample counter cells stay sharp). B slices (`hevc_enc_slice_data_b`) use bi-predicted 14-bit AMVP or intra DC with `mvd_l1_zero_flag = 0`, `slice_temporal_mvp_enabled_flag = 0`, and `initType = 2`. Overflow retries higher QP down to all-skipped so byte-count bit `30` stays clear. JPEG de-zigzags `R6` quantizers.
- **Metrics and debug**: `-global ingenic-avpu.dump=DIR` saves streams and `avpu-<w>x<h>.<ext>.rec`/`.src` NV12 images; `trace=1|2` logs launches or register writes. Over 30 frames on T31: H.264 1080p averages `8.9 KB` I / `1.2 KB` P (`47.4 dB`), 640x360 averages `2.6 KB` / `0.65 KB` (`42.0 dB`); HEVC 1080p averages `18.3 KB` / `2.6 KB` (`43.6 dB`), 640x360 averages `3.2 KB` / `0.9 KB` (`36.6 dB`), versus `21 KB` P frames under `I_PCM`.
- **Tests**: `python3 qemu/tests/test-ingenic-avpu.py` validates bit-exact reconstruction across CAVLC, CABAC, multi-slice, and `I0 P2 B1 P4 B3` sequences (`41-43 dB` on B frames) on T31/T40/T41 with `ffmpeg` or Docker `jrottenberg/ffmpeg`. End-to-end boot tests run via `bash qemu-boot/test-ingenic-avpu.sh [--soc t40|t31|t41]` (`qemu-boot/ingenic-avpu-enc.c`, SD extract on T31, top 8 MiB `pmemsave` on T40/T41, JPEG channels 4/5 to avoid T40 channel 0 fault, static `libimp.a` + `--no-direct` on T41 SDK image, and full-range YUV check on T40 HEVC).

### Display and 2D graphics (`ingenic-lcdc`, `ingenic-dpu`, `ingenic-vo`, `ingenic-drawbox`, `ingenic-i2d`)

- **`ingenic-lcdc` (T21, T23, T30, T31 `drivers/video/jz_fb_v14`, `jz-fb`, `sample_slcd`)**: Reads looping or single-frame 7-word descriptors from `CHAIN_ADDR` (`next`, buffer address, pixel stride, chain config bit `0` continue, IRQ control, UV buffer and stride) onto display `lcdc` (`screendump device=lcdc`, `fps=60`). Frame dimensions come from `SLCD_FRM_SIZE` (smart panel when `DISP_COMMON` bit `0` is set) or TFT `HDE`/`VDE`, with `GLB_CFG[18:16]` choosing `RGB555`, `RGB565`, `RGB888-in-32`, `NV12`, or `NV21` (`[21:19]` color order). Control bits cover `CHAIN_START`, `SLCD_START`/`TFT_START`, `QCK_STOP`/`GEN_STOP` (`QCK_STOPM`/`GEN_STOPM`), `FRM_END`/`DMA_END` (`INT_FLAG = ST & INTC` cleared via `CSR`), and `SLCD_CMD` word counters (`slcd-cmds`, `slcd-last`, clear `SLCD_ST`). Verified by qtest.
- **`ingenic-dpu` (T40, T41 `fbdev/ingenic/fb_v12`, `t40-dpu`/`t41-dpu`, `sample-LCD` `/dev/fb0`)**: Follows the kernel driver contract (not enabled in shipped configs). `FRM_CFG_CTRL` bit `0` reads frame descriptors from `FRM_CFG_ADDR` (`next`, size, control bit `0` stop, write-back address and stride, four layer descriptor pointers, enables, and z-order) onto display `dpu` (`screendump device=dpu`, `frames`, `fps=60`):
  - *Composition*: Supports `RGB555`, `ARGB1555`, `RGB565`, `RGB888`, and `ARGB8888` (`0..5` channel order), `NV12`/`NV21` (`BT.601`), nearest-neighbor scaling, per-pixel/global alpha, 2-level DMMU (`TLB_GLBC[3:0]`, `TLB_TLBA`, `JZFB_USE_TLB`), `CMP_START`/layer ends/`CMP_END`/`DISP_END` (`INT_FLAG = ST & INTC` -> IRQ `20`, cleared by `CLR_ST`), and quick/general stop (`STOP_DISP_ACK`).
  - *Write-back & dither*: Control bit `1` writes composed frames back in `[18:16]` formats `0/1/2/3/4/6` (`ARGB8888`, `RGB565`, `RGB555`, packed YUV422 `Y0 U Y1 V` studio `BT.601` inferred, `Y`, `RGB888-in-32`), raising `WDMA_END`. Control bits `4`/`5` (`[25:20]`) and `DISP_COM` (`0x8000` bit `4`, `[21:16]`) apply 4x4 ordered dithering.
  - *Simple RDMA (`CONFIG_INGENIC_FB_SIMPLE_RDMA`)*: Control bit `3` or `RDMA_CHAIN_CTRL` bit `0` reads 5-word descriptors from `RDMA_CHAIN_ADDR` (`0x1000`) with ctrl `[18:16]`/`[22:19]`, raising `SRD_START`/`SRD_END`/`SRD_WORKING` (`DC_CTRL` bits `1`/`4` stop).
- **`ingenic-vo` (T41 `drivers/video/ingenic_vo`, `/dev/jz-vo`, disabled in DT)**: Models DMA reset (`TOP_STATUS` bit `0` W1C), `SIZE`, `STRIDE`, `CON_PAR`, blanking, `INT_EN`, `TRAN_EN`, frame queueing (`ADDR_INFO` bit `3`), `screendump` display output (`fps=30`), `INT_VAL`/`INT_CLC`, and the `frames` counter.
- **`ingenic-drawbox` (`ingenic_drawbox` / `ingenic_drawbox.c` on T40, T41, `/dev/dbox`)**: Driven by `libimp` `OSD_Draw_Layer_Cover_Pic_RectLine` -> `dbox_osd`. Loads a 4-entry `COLOR_Y`/`U`/`V` palette, targets an NV12 (`YB`, `CB`, `STRIDE`, `IMG_WH`) or BGRA frame (`COLOR_MASK` for luma-only), executes 2-word box RAM entries (rectangles, corners, horizontal/vertical lines), clears state on `RESET`, and raises `CTRL` bit `4` `50 us` after `START` unless `IRQ_MASK` is set (line length and corner arm widths inferred).
- **`ingenic-i2d` (`ingenic_i2d` / `ingenic_i2d.c` on T40, T41, `/dev/i2d`)**: Used by `IMP_FrameSource_SetI2dAttr` to transform `NV12`, `RAW8`, `RGB565`, or `ARGB8888` images (`IMG_MODE`) with `0`/`90`/`180`/`270` degree clockwise rotation, mirror, and flip onto `DST`, setting `IRQ_STATE` bit `0` `100 us` later (cleared via `IRQ_CLEAR`).

### NNA pre-processors, DMA and bus monitor (`ingenic-aip`, `ingenic-bscaler`, `ingenic-nndma`, `ingenic-monitor`)

- **`ingenic-aip` (T41 `drivers/video/ingenic_aip`, `/dev/jzaip_t`, `_f`, `_p` at `0x12B00000` & `0x13090000`)**: Reverse-engineered from `libaip` (`magik-toolkit InferenceKit/nna2`: `warp_*`, `crop_*`, `img_format_convert`). All three units share `CTRL`, `STAT`, `CFG`, and IRQ enable registers at `0x00`–`0x0C` (`50 us` completion; alpha zero-fill and `CFG` bit `0` `Cb/Cr` swap inferred):
  - *Unit `T` (`+0`)*: Converts NV12 to 4-byte RGB/BGR using a signed Q20 BT.601 matrix (`1.164, 1.596, 0.391, 0.813, 2.018`, offsets `16, 128`). Byte order bits `[2:0]` encode `BGR..RGB` (`0..5`), and bit `3` moves alpha first (`BGRA = 0`, `RGBA = 5`, `ARGB = 13`).
  - *Unit `F` (`+0x200`)*: Performs chained bilinear box resizes with Q16 scale steps, `CFG[9:7]` element widths (`1`, `4`, `8`, `16`, or `32` bytes per pixel, or dual-plane NV12).
  - *Unit `P` (`+0x300`)*: Evaluates 15-word `3x3` inverse affine/perspective warps with out-of-bounds padding and NV12-to-4B conversion (or 1-, 2-, and 4-byte pixel copy). Chains contain `(value, reg)` pairs terminated by address bit `31`.
- **`ingenic-bscaler` (T40 `drivers/video/ingenic_bscaler`, `/dev/bscaler0` at `0x13090000`)**: Reverse-engineered by running `libvenus` (`InferenceKit/nna1`: `bscaler_api`, `bscaler_hal`, `bscaler_segmentation`, `warp_*`, `crop_*`, `warp_covert_nv2bgr`) under user-mode QEMU:
  - *Unit `C` (`0x00`–`0x7F`, IRQ `39`)*: Executes up to `64x64`-pixel tiled NV12 or 4-byte resize, affine, and perspective warps (`0x18` dest FIFO, `0x40`/`0x44` origins, `0x48`–`0x68` Q16 matrix, `0x04` pixel formats/pad, `0x00 = 0x201` run, `(value, reg)` chain via `0x34`/`0x38`/`0x30`, `50 us` completion, bilinear edge-repeat sampling with `128` CbCr pad >1 pixel outside). `0x28`/`0x2C`, `0x04` bits `1`/`2`/`16..18`, `0x30` bit `1`, and `0xA0`/`0xA8`/`0xF0`/`0xF4` act as storage.
  - *Unit `T` (`0x80`–`0xFF`, IRQ `40`)*: Line-by-line NV12->4B converter (`0x84 = lines << 16 | 1` raising bit `2`; `0x80` bit `0`/`8`; `0xAC`), sharing the `0xC0..0xE0` signed Q20 `YUV->BGR` matrix (`0xE4` offsets/orders).
  - *LZMA mode (`CONFIG_LZMADEVICE`, `drivers/lzma/lzma.c`, `jz_lzma_decompress` / `hardware_lzma.c`)*: Writing `0x00` bit `31` switches the window into LZMA mode until a C unit reset (`0x30` bit `16`), decoding full `.lzma` streams (`13-byte` header, up to `16 MiB` output) from `0x04`/`0x08` -> `0x0C` into `0x1C FINAL` (`0x10`/`0x14`/`0x18` storage; inferred from unused driver code). T41 `ingenic,t41-lzma` nodes (`0x13C00000`/`0x13D00000`) are disabled.
- **`ingenic-nndma` (T40, T41 `0x12500000`)**: Mapped directly in userspace via `__aie_mmap` (`dma_func.cc`: `w_d2o_orasetrun`, `aie_run_nndma_rdchn`, `wrchn`; `soc-nna.ko` implements only the version ioctl). `0x0000`–`0x3FFF` holds 2048 2-word descriptors (`word 0`: DDR `[31:6]`, chain bit `0`, len `[16:12]` in `5:1`; `word 1`: len `[11:0]` in `31:20`, ORAM 64-byte offset in `19:0`). Writing `bit 31 | bytes/64 << 12 | first_desc` to `0x8000 + 4*n` (DDR->ORAM) or `0x8010 + 4*n` (ORAM->DDR) executes synchronously so `rdhwr` waits complete immediately (`0x2000` `CPU_SIMULATOR` `RCFG`/`WCFG` stays storage).
- **`ingenic-monitor` (T40, T41 `0x130A0000`, `sample_bus_monitor`, IRQ `53`)**: Implements 6 AXI channels (`0..4, 6` at `0x100` stride) plus AHB channel `5` (`0x500`) with `INT_STA`, `INT_MASK`, and `INT_CLR`. Advances virtual cycles at `clock-hz` (`300 MHz`) to raise `TIME_SLOT_REACH` (`traffic` and `watch` counters stay `0`).

---

## Tested

### Boot and flash persistence

| Image / Configuration | Target & Path | Result |
|---|---|---|
| OpenIPC `t10`, `t20`, `t21`, `t23`, `t30`, `t31`, `t40`, `t41` | Own machine, `-kernel` + NOR, flash & split NOR layout (`test-ingenic-boot.sh`) | Login, DHCP lease, U-Boot environment, and JFFS2 overlay persist across boots; `--reset` verifies fast warm reboot (`ingenic_tb_reset`) |
| OpenIPC `t40` PHY power-down on reboot | `t40`, `test-ingenic-eth-reboot.sh` | `reboot -f` keeps `eth0`; `reboot` reproduces `firmware#2839` SWR timeout; `--phy-rst-gpio 71` (with `--expect-fixed`) keeps `eth0` across both |
| OpenIPC T31 SPI NAND (`w25n01gv`) | U-Boot `isvp_t31_sfcnand` (+ patch), Linux 3.10 (`MTD_JZ_SFC_NAND`), UBI, `test-ingenic-nand.sh` | SPL, U-Boot environment save, UBI SquashFS `ubiblock` root, UBIFS overlay, and login; written files persist on reboot |
| OpenIPC T41 SPI NAND (`gd5f1gq4ub` with OOB) | Vendor `isvp_t41n_sfc0_nand`, SDK 4.4 `isvp_tomcat_nand_defconfig` (+ `UBI_BLOCK`, `maxcpus=1`), `test-ingenic-nand.sh --soc t41` | SPL, U-Boot, Linux 4.4 CDT + descriptor DMA, UBIFS root, and login; written files persist on reboot |
| Vendor SDK 1.2.0 (`t41lq`, `t41nq`, `t41xq`; `t40,variant=t40n/t40xp`) & 1.0.5 (`t30,variant=t30x`) | Full 8 / 16 MiB flash (`-m 64M`/`128M`/`256M`), `test-ingenic-sdk-boot.sh` | Dual-core 4.4 boot (`t40`/`t41`), `Ingenic-uc1_1 login:`, telnet over GMAC, `carrier-server` H.265 RTSP scene (`gc4663`/`gc2053`, 20 frames copied before host decode; counter read from bottom four rows beneath OSD), WDT reboot back to login |
| Camera dumps (`t10` APK-Link, `t20` Jabsco, `t21` Jabsco/CamHi, `t31` Wyze Cam v3, `t40xp`) | Own machine, `flash` (`t40xp` `-m 256M`) | Vendor SPL -> U-Boot -> kernel -> login -> app start / ISP probe / DHCP (`t21` WDT reboots cleanly) |

### Networking and Wi-Fi

| Image / Configuration | Target & Path | Result |
|---|---|---|
| ATBM6031 SDIO (`t31`, `t40`, open & `wifi-psk=`) | `test-ingenic-wifi.sh [--soc t40] [--psk P [--wrong-psk]]` | SDIO enumeration, firmware download, `wlan0`, 4-way handshake with TK/GTK verification, DHCP `10.0.2.15`, ping; wrong PSK rejected at message 2 |
| ATBM6031 AP modes (`wpa`, `tkip`, `pmf`, `rekey`, `powersave`) | `-global atbm-sdio.proto= / cipher= / pmf= / rekey=` | RSN/TKIP and WPA1/TKIP (station Michael MIC checked), 3x alternating GTK rekeys, PMF IGTK verified, power-save mode `0x81` traffic |
| ATBM6441 IoT SDIO (`t31`) | `test-ingenic-wifi-6441.sh` | `0x6441` MAC/DHCP/ping/TCP fetch, `/dev/atbm_ioctl` AT command round-trip, `rmmod`/`insmod` reload |
| ATBM6012B-X USB (`t31_ultimate`) | `test-ingenic-wifi-usb.sh` | `atbm6012b_wifi_usb.ko` joins open/WPA2-PSK (`10.0.2.15`, ping/TCP); `atbm603x_wifi_usb.ko` (`atbm60xx`) ignores `0x888b` as reported |

### Image pipeline, WDR, HLDC and multi-sensor

| Image / Configuration | Target & Path | Result |
|---|---|---|
| Single-sensor ISP (`t10`..`t41`, `t30` + SDK `tx-isp-t30.ko`) | Respective sensors (`jxh62`, `jxf22`, `jxf37`, `gc2053`, `ov2735b`, `gc4653`, `gc4663`) | `test-ingenic-isp.sh`: 1080p/1440p/720p and 360p NV12 frames with advancing counter, logo, and moving dot (bit-banged I2C on `t10`) |
| DOL WDR (`t31 -m 128M`, `t40`, `t41`, `--wdr`) | `gc4653`, `gc4663`, `test-ingenic-isp.sh --wdr` | Short exposure active and converging; logo clipping under `2%` (`0.4%` vs `7.5%`) |
| T41 & T40 HLDC (`--soc t41 --ldc 200,40`, `--soc t40 [--sensor1] --ldc`) | `/dev/tisp` + libimp bypass clear | Corrected frames match host remap (`95%` T41 vs `157` levels, `100%` T40 vs `111`, `95%` dual-pipe `ch0`+`ch3`) |
| T40 2- & 3-sensor + `snapraw` + Joint | `sensor1=gc4653`, `sensor2=gc4653`, `--snap-env SNAP_SNAPRAW=0_1`, `--joint 3` / `4` | Dual `2560x1440` AE convergence, `VIC2` IRQ, `/tmp/snap0.raw` (`2560x1440x2` RGGB mosaic), `2560x2880` vertical joint |
| T23 `direct_mode` & Dual-sensor | `tx-isp-t23-double`, `gc2053s1`, `test-ingenic-t23-enc.sh` | `--direct 1` (`~100` IVDC frames); `--sensor1 gc2053s1` (4 H.264 1080p/360p streams, counters `0..` & `500000..`); `--direct 2` (ch0 + sub-streams) |

### Video encoders, decoders and RTSP

| Image / Configuration | Target & Path | Result |
|---|---|---|
| `ingenic-vpu` (`t20`, `t21`, `t23`, `t30`) | `test-ingenic-vpu.py`, `test-ingenic-vpu-feed.py`, `test-ingenic-vpu-e2e.sh` | Register-level, reverse-TLB, FrameSource, and `libimp` H.264/HEVC/JPEG passes; P-slice motion (`~1/50` I_PCM); `ffmpeg -err_detect explode` PSNR verified |
| JPEG decode (`t23`, `t30`) | `test-ingenic-jpegd.sh --soc t23` / `t30 [--jpeg FILE]` | `640x480`, `1280x720`, and T21 `640x360` snapshots decoded (`80.8`, `81.6`, `70.5 dB` Y PSNR against `ffmpeg`) |
| `ingenic-avpu` (`t31`, `t40`, `t41`) + SDK 1.1.5 manual boot | `test-ingenic-avpu.py`, `test-ingenic-avpu.sh [--osd] [--ivpu]`, `init=/bin/sh` | CAVLC/CABAC/multi-slice/B-frame H.264, I/P/B HEVC, JPEG, T41 gen2, `IMP_OSD` DRAWBOX, standalone HELIX JPEG (`--ivpu`), SDK `sample-Encoder-video`/`-jpeg` (`init=/bin/sh`), bit-exact `rec` match (`41-43 dB`) |
| T30X SDK live RTSP + OSD | `t30,variant=t30x -m 128M`, `jxf22`, `test-ingenic-t30-rtsp.sh` | `sample-RTSPServer` `/main` 1080p & `/second` 360p H.265 over RTSP/TCP, 200 decoded frames each with scene and IPU timestamp OSD |

### Audio, digital twin, NNA, display and crypto

| Image / Configuration | Target & Path | Result |
|---|---|---|
| OpenIPC `majestic` on T40 digital twin | `t40n`, `gc4653`, `test-ingenic-majestic.sh [--audio]` | Mic starts cleanly (`#257`); `PB17` day/night toggles write ~980 new ISP night registers vs 8–15 drift (`#258`) |
| AIC audio (`t10`..`t41`), gains & DMIC (`t40`, `t41`) | `test-ingenic-audio.sh --soc <soc> [--gains AI:AO] [--dmic]`, `--rate 8000 --play-rate 16000` | `1000.0 Hz` record (`rms 5793`; `11585` T10/T20/T30); `2.00 s` `440.0 Hz` play (T21 `6/6` alone; `dsp_ioctl_flush_ao_stream` truncated an earlier `1.08 s` loaded run); T40 `-6.0/-6.0 dB` (`2903`, `4242/8464`), T41 `-6.0/+6.0 dB` (`2903`, `16889`); DMIC `1500.0 Hz` (`24/24` without TLB shadows vs `5/12` failures with them) |
| Drawbox & I2D (`t40`, `t41`) | `ingenic_drawbox`, `ingenic_i2d`, `test-ingenic-xb2blocks.sh --image` | Draws rectangle/line through `/dev/dbox` and rotates `320x192` NV12 90 degrees through `/dev/i2d` (every pixel checked) |
| AIP & BSCALER (`t41`, `t40`) | `ingenic_aip` + `libaip.a` (`test-ingenic-aip.sh`), `ingenic_bscaler` + nna1 `libvenus.a` (`test-ingenic-bscaler.sh`) | `bs_*` calls: AIP `640x360->320x180` (`< 0.8`), 2-box chain, BT.601, warps; BSCALER `64x32->32x16`, `32x16->160x80`, `256x128->128x64` (`< 0.7`), warps |
| NNDMA (`t40`) | `soc-nna.ko` + `libvenus.a` `dma_func`, `test-ingenic-nna.sh` | Version ioctl; `w_d2o_orasetrun` moves DDR<->ORAM verified byte-for-byte with `rdhwr $9`/`$11` plus `ll; rdhwr $4` translation |
| UART0 PDMA loopback (`t31`) | `CONFIG_SERIAL_JZ47XX_UART0_DMA`, `test-ingenic-uart-dma.sh --kernel` | 32 KiB random TX/RX loopback via 1 KiB PDMA buffers intact across `3/3` runs |
| Crypto & RNG (`t31`) | `sample_aes`, `sample_hash`, `sample_rsa`, `sample_dtrng`, `sample_des`, `test-ingenic-crypto.sh` | `AES-128` ECB/CBC (FIPS-197, SP 800-38A); HASH vs `hashlib`; `RSA-1024/2048` (`test_rsa.h`); DTRNG polled/IRQ; DES FIPS 46 (`85e813540f0ab405`) + 4 KiB 3DES CBC PDMA |
| Qtest suite (`python3 qemu/tests/test-ingenic.py`) | No guest (`qtest`, `t31`/`t41`/3-sensor `t40` migration) | ISP/VIC/MSCA across layouts (`clock_step`); T40 dual pipe, `VIC_MDMA` (`0x380`, `status2` bit `0`, RGGB), and `VIC2`; T10/T20 Apical; DPU (`test_dpu`); HASH CPU (`test_hash_cpu`); I2C PDMA (`test_i2c_dma`); SADC continuous PDMA (`test_sadc_dma`); PDMA/AIC/DMIC triggers/SSI `w25q80`/UART DMA/AES/RSA/DTRNG/TCU PWM |

---

## Not modelled

These features and blocks remain unimplemented (reads return zero or register blocks act as storage):

- **T40 `audio.ko` capture past 1 second**: Using OpenIPC `t40`'s release `audio.ko` and `libimp.so` (`8 kHz`), capture records ~50 x `20 ms` frames before `pdma` and `pdmam` interrupts stay `0`, causing `get mic timeout!` and `Error: Not Find any irq`.
- **Wi-Fi (`atbm-sdio`, `atbm-wifi`)**:
  - *WPA3-SAE & 802.1X/EAP*: Guest `wpa_supplicant` builds parse `sae_*` options without SAE code (and lack EAP methods); no SAE test vectors or auth server exist here.
  - *BIP / SA Query / BLE*: No group-addressed robust management frames are sent, station SA Query handling is implemented but untested, and BLE is unmodelled (`wifi=atbm` attaches on MSC1 across all machines, tested end-to-end on T31 and T40).
- **ISP & VIC gaps**: OpenIPC's broken `tx-isp-t30.ko` (uninitialized `apical_io_base`), T40 left/right horizontal split-joint (`>3840` scaler width), VIC raw frame channel `ispvic_frame_channel_qbuf`, T23 WDR (no WDR T23 sensor modelled), Apical DIS non-zero motion vectors, and T23 `direct_mode=2` sensor 1 main stream (`framesource 3`, never queued by `libimp`).
- **Peripheral gaps**: SSI slave (`0x10040000`, no driver, sample, or PDF register map in any SDK tree), trigger logic on T41 ASDMIC and `ingenic-dmic` trigger modes other than `0`/`2`, and DPU TLB retry/verification registers (storage).
- **Remaining T40/T41 video and compute blocks (SDK 4.4 / `libimp` survey)**:
  - **T41 LDC block (`0x13040000`, `ingenic,t41-ldc`, core INTC `35`)**: `tx-isp-t41.ko` (`H20230630a`, `tx-isp-ldc.c`) probes its resource, `div_ldc`/`gate_ldc` clocks, and pads without register programming; `IMP_ISP_LDC_INIT` / `IMP_ISP_LDC_SetAttr` / `IMPISPLDCParams` (`LDC_ADDR_RES_SIZE`, `X_CORF`, `BROCF`, `R2_REP`, `LEN_COEF`, `ldclut.h`) target an unshipped `"LDC-Beta"` (`sample-Encoder-ldc-video.c`) build. (ISP HLDC is modelled.)
  - **NCU of T10/T20/T30 (`0x13090000`) & T31 bus monitor**: `jz_nvpu` configures `jz_ncu.h` alongside VPU runs without polling; storage passes frames through un-denoised. T31 `base.h` marks `MONITOR_IOBASE` (`0x130F0000`) `"T31 delete"`, and its `sample_bus_monitor` reuses T40 `t40-irq.h`, so no T31 bus monitor is mapped.
  - **T40/T41 NNA compute instructions**: While ORAM and NNDMA are modelled, `libvenus` executes neural networks with proprietary coprocessor-2 (`COP2`) MAC instructions (`57` opcode classes by `rs`/function; `121881` instructions in T40 `libvenus`, `48450` in T41) undocumented in public toolchain `libopcodes`. Executing one raises Reserved Instruction (`SIGILL`).
  - **T40/T41 RISC-V co-processor (`LEP`, T41 CCU `0x12A00000`)**: No SDK here includes firmware, drivers, or libraries to start it.
