# Ingenic T-series (`t10`, `t20`, `t21`, `t23`, `t30`, `t31`, `t40`, `t41`)

The only non-ARM family here: little-endian MIPS32 camera SoCs, built
into `qemu-system-mipsel` (`qemu/setup.sh` adds `mipsel-softmmu` to the
target list). One machine file, `qemu/hw/mips/ingenic.c`, with a table
entry per SoC; the device models are `qemu/hw/*/ingenic-*.c`.

## What the sources cover

| Source | What it gave |
|---|---|
| OpenIPC `linux` branches `ingenic-t31` (3.10.14: T10, T20, T21, T30, T31), `ingenic-t23`, `ingenic-t40` (4.4.94) | addresses, IRQs, every boot-time poll (`arch/mips/xburst*/soc-*/`, drivers) |
| OpenIPC `u-boot-ingenic` (`isvp_t*`), the T40/T41 vendor U-Boots | SPL header, SPL and U-Boot polls, flash layout |
| Vendor SDKs T23 1.1.0, T31 1.1.5, T40 1.2.0, T41 1.2.0 | the T23 N/LP/HP variants, T41's kernel 4.4.94 and full images |
| Camera dumps: T10, T20, T21 ×2, T31 ×3 (one a Wyze Cam v3), T40XP | the vendor boot chains, end to end |

All of them boot (see [Tested](#tested)).

## Machines

| Machine | Core | RAM default | Kernel | SFC | MSC | Notes |
|---|---|---|---|---|---|---|
| `t10` | XBurst1 | 64 MiB | 3.10 | v1 | jzmmc | DWC DDR PHY, GPIO ports at 0x100 |
| `t20` | XBurst1 | 64 MiB | 3.10 | v1 | jzmmc | as T10 (T20X: `-m 128M`) |
| `t21` | XBurst1, 16 KiB caches on silicon | 64 MiB | 3.10 | v1 | jzmmc | "+1" PLL encoding, card detect PB20 |
| `t23` | XBurst1 | 64 MiB | 3.10 | v1 | jzmmc | two PLLs; N, LP, HP differ only in clocks |
| `t30` | XBurst1 | 64 MiB | 3.10 | v1 | jzmmc | "+1" PLL encoding, a fourth PLL (EPLL) |
| `t31` | XBurst1 | 64 MiB | 3.10 | v1 | jzmmc | the reference part (T31X/A: `-m 128M`) |
| `t40` | 2 × XBurst2 | 128 MiB | 4.4 (built-in DT) | v2 | SDHCI | CCU, per-core INTC and OST (T40XP: `-m 256M`) |
| `t41` | 2 × XBurst2 | 128 MiB | 4.4 (built-in DT) | v2 | SDHCI | T41's own PLL encoding, RTC at 0x10003000 |

The parts within a family (T31N/L/X/A/AL/LC, T23N/LP/HP, T40N/XP/A,
T41LQ/NQ/XQ) differ in DRAM and clock settings, which the boot loader
programs and the kernel reads; one machine serves them all, with `-m`
for the DRAM. RAM is at most 256 MiB, all of it below the I/O space.

## Memory map

Physical addresses; software uses them through KSEG1 (`0xA0000000 +`).

| Base | Block | Model |
|---|---|---|
| 0x00000000 | DDR | RAM |
| 0x10000000 | CPM: PLLs, dividers, gates, resets | `ingenic-cpm` |
| 0x10001000 | INTC, 64 sources → IP2 (XBurst1) | `ingenic-intc` |
| 0x10002000 | TCU: 8 channels, watchdog, boot loader OST | `ingenic-tcu` |
| 0x10003000 | RTC / hibernation (T40: 0x132A0000) | `ingenic-rtc` |
| 0x10010000 | GPIO, ports at 0x1000 (T10/T20: 0x100), shadow port Z | `ingenic-gpio` |
| 0x10030000 + 0x1000 n | UART0..3 (T41: UART0..5, UART4/5 on 57/56), 16550 with a 4-byte stride, paced at the line rate both ways, 64-byte FIFOs, PDMA requests 0x14 - 2 n / 0x15 - 2 n | QEMU `serial-mm` (patches 0021, 0023) |
| 0x10050000 + 0x1000 n | I2C0..3 (DW_apb_i2c), PDMA requests 0x24 + 2 n / 0x25 + 2 n | `ingenic-i2c` |
| 0x12000000 | system OST → IP4 (XBurst1); global OST (XBurst2) | `ingenic-ost` |
| 0x12100000 + 0x100 n | core OST → IP4 (XBurst2) | `ingenic-ost` layout 2 |
| 0x12200000 | CCU: core reset/entry, mailboxes → IP3 (XBurst2) | `ingenic-ccu` |
| 0x12300000 + 0x100 n | core INTC → IP2 (XBurst2) | `ingenic-ccu` |
| 0x13010000 | DDR PHY (+0x1000) and DDRC APB (+0x2000) | `ingenic-ddr` |
| 0x134F0000 | DDRC | `ingenic-ddr` |
| 0x13440000 | SFC + SPI NOR or SPI NAND | `ingenic-sfc` + m25p80 / `ingenic-spinand` |
| 0x13450000 | T41: SFC1, IRQ 19 (`-drive if=mtd,index=1`) | `ingenic-sfc` |
| 0x13450000 / 0x13460000 | MSC0/1 (XBurst1) | `ingenic-msc` |
| MSC1 SDIO | ATBM6031 Wi-Fi (`-M t31,wifi=atbm`) | `atbm-sdio` |
| 0x13060000 / 0x13070000 | MSC0/1 (XBurst2): SDHCI 3.0 | QEMU `generic-sdhci` |
| 0x134B0000 | Synopsys GMAC, IP101G PHY | `ingenic-gmac` |
| 0x13500000 | DWC2 USB OTG, host | QEMU `dwc2-usb` |
| 0x13540000 | EFUSE: blank but for the grade code | `ingenic-efuse` |
| 0x13300000 | ISP core (T31: to 0x1338FFFF; T21, T23: 64 KiB; T40, T41: 512 KiB, T41's scaler at 0x133F0000; T10/T20/T30's Apical core at 0x13380000) | `ingenic-isp`, `ingenic-isp-apical` |
| 0x133E0000 | VIC (T21: 0x13310000; T40, T41: 0x13380000, T40's VIC1 / VIC2 at 0x13390000 / 0x133A0000) | `ingenic-isp` |
| 0x10023000 / 0x10022000 | MIPI CSI-2 host / D-PHY (T23, T31) | `ingenic-isp` |
| 0x13000000 | AHB0 arbiter, the SoC ID at +0x2C | `ingenic-harb` |
| 0x13200000 | video encoder: VPU (T10/T20), HELIX (T21/T30; T23: 0x13100000), IRQ 70 (T23: 48) | `ingenic-vpu` |
| 0x13600000 | RADIX, T30's H.265 encoder, IRQ 53 | `ingenic-vpu` kind 1 |
| 0x13080000 | IPU, libimp's OSD blender (every part), IRQ 14 | `ingenic-ipu` |
| 0x13280000 | T41: the JPEG engine (HELIX's JPEG half, units renumbered), core INTC 0 (`/machine/jpeg`) | `ingenic-vpu` layout 1 |
| 0x130B0000 | T40, T41: I2D, rotate / mirror / flip (frame channel rotation), IRQ 45 | `ingenic-i2d` |
| 0x130D0000 | T40, T41: DRAWBOX, libimp's OSD rectangles and lines, IRQ 42 | `ingenic-drawbox` |
| 0x130C0000 | T41: VO, NV12 out over BT.656 / BT.1120, shown on the display, IRQ 43 (`/machine/vo`) | `ingenic-vo` |
| 0x13050000 | T40, T41: DPU, the LCD controller (frame and layer descriptors, composed onto a display), IRQ 20 (`/machine/dpu`, display id `dpu`) | `ingenic-dpu` |
| 0x130A0000 | T40, T41: bus MONITOR (AXI/AHB bandwidth and address watch), IRQ 53 | `ingenic-monitor` |
| 0x12B00000 (+ alias 0x13090000) | T41: AIP, the NNA's image pre-processor (convert, resize, warp), IRQ 40 / 39 / 38 (`/machine/aip`) | `ingenic-aip` |
| 0x13090000 | T40: BSCALER, the NNA's image engine (resize / affine / perspective, NV12 to colour), IRQ 40 (T) / 39 (C) (`/machine/bscaler`) | `ingenic-bscaler` |
| 0x10070000 | SADC: 8 AUX channels, 12 bits, IRQ 18, PDMA request 9 (`/machine/sadc`) | `ingenic-sadc` |
| 0x10020000 / 0x10021000 | AIC (I2S, IRQ 1) / internal audio codec (`/machine/aic`) | `ingenic-aic` |
| 0x13420000 | PDMA: 32 channels, descriptors, IRQ 10 | `ingenic-pdma` |
| 0x10034000 | T31, T40: DMIC, digital microphones, IRQ 12 (`/machine/dmic`) | `ingenic-dmic` |
| 0x134D0000 | T41: the audio system's DMA, FIFOs, data bus and DMIC, core INTC 33 (`/machine/dmic`) | `ingenic-asdmic` |
| 0x13460000 | T41: PWM controller, 8 channels (`/machine/pwm`) | `ingenic-pwm` |
| 0x10043000 / 0x10044000 | SSI0 / SSI1 SPI masters (SSI1: T21, T30, T31, T40, T41), IRQ 9 / 8, buses `ssi0` / `ssi1` | `ingenic-ssi` |
| 0x10060000 | OTG PHY registers (bytes) | RAM |
| 0x10072000 | DTRNG, IRQ 34 (T21, T23, T31, T40, T41) | `ingenic-dtrng` |
| 0x13430000 | AES engine, IRQ 23 (every part) | `ingenic-aes` |
| 0x10061000 | DES / triple-DES engine, PDMA 0x2E / 0x2F (every part) | `ingenic-des` |
| 0x13480000 | HASH engine (MD5, SHA-1/2), IRQ 22 (T21, T23, T31, T40, T41) | `ingenic-hash` |
| 0x134C0000 | RSA engine, 1024/2048 bits, polled (T23, T31, T40, T41) | `ingenic-rsa` |
| 0x13200000 | AVPU video encoder, registers from +0x8000 (T31, T40; T41: 0x13100000 from +0), IRQ 70 | `ingenic-avpu` |
| 0x12600000 | XBurst2 ORAM, the NNA's memory (1 MiB; the T41 SPL's core-1 stub at 0x12660000) | RAM |
| 0x12500000 | T40, T41: NNDMA, the NNA's DMA (descriptor RAM, channels at +0x8000) (`/machine/nndma`) | `ingenic-nndma` |
| 0x80000000 | T41: the IVDC's frame store as `direct_mode` addresses it (32 MiB) | RAM |
| 0x13200000 | T41: the IVDC's registers, core INTC 13 | `ingenic-ivdc` |
| 0x1F000000 | cache-as-RAM window (32 KiB) | RAM |
| 0x1FC00000 | boot ROM (the reset vector, 0xBFC00000) | ROM |

Everything else in 0x10000000-0x13FFFFFF reads as zero (`ingenic.io`, an
unimplemented device, logged with `-d unimp`). Interrupt numbers are the
same on every part: AIC 1, SSI1 8, SSI0 9, SFC 7, PDMA 10, DMIC 12, GPIO A 17 (B 16, C 15, D 14), OTG 21,
SADC 18, HASH 22, AES 23, DTRNG 34, TCU 27/26/25, MSC0/1 37/36, UART0..3 51..48, GMAC 55, I2C0..3 60..57,
ISP 29, VIC 30 (T40: VIC1 29, VIC2 28), AVPU 62, on T40/T41 DRAWBOX 42 and I2D 45, T41's AIP 40 (T), 39 (F), 38 (P) and VO 43, T40's BSCALER 40 (T), 39 (C), and the MONITOR 53 (Linux
numbers are these + 8: the ISP's 37 and 38, the AVPU's 70).

## CPUs

QEMU's own Ingenic models are not what these kernels need, so
`qemu/patches/0015-mips-ingenic-xburst-cpus.patch` adds two:

- `XBurst1` (T10..T31): PRId 0x00d00100 (T20: 0x00d00101; what the
  cameras' boot logs print), a 32-bit FPU (the kernels
  hard-code one and OpenIPC userspace is hard-float), 32 TLB entries (the
  kernel sets `tlbsize = 32` whatever Config1 says), 32 KiB 8-way caches,
  and **Config7.WII**: the kernel idles in `wait` with interrupts disabled
  (`jz_wait_irqoff`), which QEMU wakes from only with WII.  The ISA is R2
  so that `mfc0/mtc0 $12, 2`, the SPL's second instruction, decodes; the
  kernels are R1 and never notice. FIR.F64 is set (Status.FR stays 0):
  QEMU decodes COP1X (MADD.D, MSUB.D...) on R2 only with it, and
  OpenIPC's musl libm uses them -- libimp's rate control calls `logf`.
  It has **MXU2**, Ingenic's 128-bit SIMD unit (32 × 128-bit registers;
  the core manual's feature table gives the T-series MXU2 and not MXU1),
  in `qemu/patches/0018-mips-ingenic-mxu2.patch`: Config1.C2 set, Status.CU2
  writable, every instruction Coprocessor Unusable until CU2 is on -- the
  kernels enable it lazily from that trap and save and restore the
  registers with MXU2's own loads and stores. Encodings come from the
  Ingenic toolchain's opcode table (`as -mmxu2`), semantics from the
  intrinsic names and their MSA counterparts; the few that are guesses
  are marked GUESS in the code. `qemu/tests/test-ingenic-mxu2.sh` checks
  every instruction, the lazy enable and a context switch, and runs
  libimp's MXU2 `jz_memcpy` (it needs the Ingenic toolchain and skips
  without it).
- `XBurst2` (T40, T41): PRId 0x00132100 (company 0x13, bits 15:13 = 1 is
  what the 4.4 kernel calls XBurst2), FPU, RI/XI, UserLocal and **Config4.IE = 2**: the kernel flushes
  the TLB with one `tlbinvf`, a no-op in QEMU without it.
  It has **MXU3**, the 512-bit SIMD unit (32 × 512-bit registers, four
  512-bit accumulators, 32 scalar registers; T41's MXU3.1 adds three
  shuffles), in `qemu/patches/0019-mips-ingenic-mxu3.patch`, gated by
  CU2 the same way. Encodings come from the Ingenic GCC 7.2 toolchain's
  opcode table, semantics from the RTL its `mxu3.h` builtins expand to.
  265 of the 496 mnemonics are exact; the rest -- reductions, strided
  and gathered loads, the sub-byte forms and a few float and shuffle
  ops, whose builtins are opaque to the compiler -- are approximations
  or keep their destination, so code using them runs but may compute
  wrong values. `qemu/tests/test-ingenic-mxu3.sh` checks representatives
  of the exact groups, the lazy enable and two processes (the T40
  kernel enables MXU3 on core 0 only; the test pins itself there), on
  T40 from an initramfs and on T41 -- whose SDK kernel has no initramfs
  -- as `/init` of a SquashFS on the NOR, under the SDK image's own
  kernel; the T41 build (`-mmxu3.1`) also runs the three MXU3.1 shuffles
  (`gshufvb`, `gshufwb_1`, `gshufwb_2`), whose builtins are opaque, so
  there the check is that they decode, retire and are reported. The
  T40XP camera's application and the T41 SDK image start without
  SIGILL; their MXU3 code (libimp's motion detection and resize, the
  libvenus networks) runs per video frame, so it is exercised once the
  T40/T41 ISP feeds frames. The first execution of each approximated
  mnemonic is logged under `-d unimp`, and `QEMU_MXU3_STATS=1` (`=all`
  for every MXU3 mnemonic) prints execution counts at exit.

The T21 and T23 have 16 KiB L1 caches where the others have 32 KiB; the
machine sets that, and the T20's PRId, through the CPU's `l1-kib` and
`prid` properties (`qemu/patches/0017-mips-prid-l1-size-properties.patch`).

The second XBurst2 core starts powered off. The kernel's
`boot_secondary()` writes the entry to CCU RER and clears the core's CSRR
bit, and the CCU starts it there (`-smp 1` gives one core).

The machine also turns off QEMU's TLB "shadow" entries
(`qemu/patches/0020-mips-tlb-shadow-property.patch`, CPU property
`tlb-shadow`): when `tlbwr` replaces an entry QEMU normally keeps the old
one as a hidden translation, valid as long as the guest drops mappings
through `tlbp`, `tlbwi` or `tlbinv(f)`. On two cores the T40/T41 4.4
kernel can tell: a user program now and then kept reading stale data
through one of its pages for the rest of its life (a static T40 client's
`pow()` returned garbage, so libimp's DMIC volume came out `inf` or 0 and
recordings were constant 0x7FFF or silence in about 40 % of runs) while
the kernel read the right bytes for the same page. Without the shadows
`tlbwr` drops the old entry as a real TLB does.

XBurst2 answers `rdhwr` for hardware registers MIPS leaves undefined
(`qemu/patches/0022-mips-xburst2-hwr.patch`, on CPUs with MXU3): `$4` is
the physical address the last `ll` touched -- the vendor kernels turn user
addresses into physical ones with `ll; rdhwr $4` -- and `$8`..`$10` /
`$11`, `$12` wait for the NNA DMA's read / write channels, which Ingenic's
libvenus does from user space. HWREna keeps bits 4 and 8..12 (the
`soc-nna` driver's `open()` writes it all ones and sets Status.CU2).

## Identification

Software tells the dies and their grades apart by three registers
(ipctool `src/hal/ingenic.c`, after the vendor's libsysutils):

- the SoC ID at 0x1300002C (`ingenic-harb`): bits 31:28 = 1, the family
  in bits 19:12 (0x21, 0x23, 0x30, 0x31, 0x40, 0x41; bits 27:12 = 0x0005
  on T10, 0x2000 on T20), a revision below;
- CPM CPPSR's low byte: 0x10 on the L parts of T10/T20/T30, 1 otherwise;
- a grade code in the upper half of the EFUSE's SUBSOCTYPE1 (0x13540238)
  on T10..T31, SUBSOCTYPE2 (0x13540250) on T40/T41.

`-M <soc>,variant=<grade>` sets CPPSR and the code: `t10 t10l`, `t20
t20l t20x`, `t21n t21l t21x t21z`, `t23n t23x`, `t30n t30l t30x t30a t30z`,
`t31n t31l t31x t31a t31zl t31zx t31al t31zc t31lc`, `t40n t40nn t40xp
t40a`, `t41nq t41n t41l t41lq t41lc t41zl t41zn t41x`; the first of each
SoC is its default. DRAM stays `-m`. `ipctool` run in the guest reports
the grade (T31X, T40XP, ...).

## Boot

### The mask ROM

`-M <soc>,flash-file=image.bin` (and no `-kernel`) boots a NOR image the
way the mask ROM does:

1. The SPL header at flash 0: `06 05 04 03 02 55 aa 55 aa`, a CRC7 of
   bytes `[0x800, length)` in byte 9, the length (≤ 0x6800) at 0xC. The
   CRC matches on every image we have; a mismatch is a warning.
2. The SPL is copied to 0x80001000 and entered at 0x80001800 with
   `sp = 0x80008000`: the SPL never sets its stack.
3. On silicon that window is cache-as-RAM, locked cache lines with no
   DDR behind them. Here too (`qemu/patches/0016-mips-cache-as-ram-window.patch`):
   the CPU's `car-size`/`car-phys` properties send KSEG0
   0x80000000-0x80007FFF to the 32 KiB at 0x1F000000 from reset until the
   first instruction fetch from KSEG0 outside it -- the jump to U-Boot or
   the kernel. The T41 SPL writes a pattern across all of DDR through
   KSEG1 underneath itself, which would overwrite an SPL in DDR.

The SPL then does what it does on a camera: pinmux, UART1, the TCU's OST
for its delays, PLLs and dividers (`ingenic-cpm` answers every lock and
busy poll), DDR init (`ingenic-ddr` answers the PHY's), then loads U-Boot
from flash 0x6800 to 0x80100000 -- LZO-packed in OpenIPC builds, raw in
the vendor ones. U-Boot boots the kernel from flash. Erases and programs
go back to the image, so the environment U-Boot saves on its first boot,
and the JFFS2 overlay, persist.

`-drive if=mtd,file=...` gives the NOR without booting from it. The NOR
is a `w25q64` (8 MiB), `mx25l12805d` (16 MiB; QEMU has no W25Q128, and
0xC22018 is in every vendor table) or `w25q256` (32 MiB), or
`flash-type=`.

### SPI NAND

A flash image of NAND size is a SPI NAND on the same SFC (SFC0 on every
part: the T41 vendor boards and DT put NAND there too, SFC1 is optional).
The image format is the page data of the whole part, or every page
followed by its OOB; the size picks the part, `flash-type=` overrides:

| Image size | Part (ID) | Geometry |
|---|---|---|
| 128 MiB, 132 MiB with OOB | `w25n01gv` (EF AA 21) | 2048 + 64, 64 pages/block, 1024 blocks |
| 136 MiB with OOB (128 MiB: `flash-type=`) | `gd5f1gq4ub` (C8 D1) | 2048 + 128, 1024 blocks |
| 256 MiB, 272 MiB with OOB | `gd5f2gq4ub` (C8 D2) | 2048 + 128, 2048 blocks |
| 264 MiB with OOB | `mx35lf2ge4ab` (C2 22) | 2048 + 64, 2048 blocks, two planes |

All four are in the SPL's, U-Boot's and both kernels' tables. Without
OOB in the file the OOB is RAM and reads FF (all blocks good) on every
start; programs and erases go back to the file either way.

The mask ROM's NAND path is the NOR one with a different header: byte 8
is 00 (NOR: aa), byte 10 is pages per block / 32 and byte 11 bytes per
page (page + OOB) / 1024 (`start.S`; 2112 bytes -> 2). The SPL is in
the data area of the first pages, length at 0xC, CRC7 at 9, up to 28 KiB
(the cache-as-RAM above 0x80001000). The SPL (`spl_sfc_nand.c`) probes
the ID, unlocks, sets ECC/BUF/QE, and loads U-Boot raw from data offset
0x6C00 (27 KiB) with page reads, `0x0B` reads from the cache and status
polls; U-Boot's `jz_sfc_nand` reads the environment at 0xC0000/0xE0000
(redundant) and saves it with erase and program.

The 3.10 `jz_sfc_nand` registers no MTD at all without the burner's
partition table at data offset 0x6800 (magic `nand`, a count, then
`{name[32], size, offset, mask_flags, manager_mode}`), which the vendor
burner writes into the SPL's padding. `qemu-boot/ingenic-nand-image.sh`
builds a whole image the way the burner lays it out:

    0         u-boot-with-spl.bin (SPL padded to 27 KiB, raw u-boot.bin)
    0x6800    partition table: 1M(boot),8M(kernel),40M(rootfs),-(data)
    0xC0000   U-Boot environment (blank; U-Boot saves it on first boot)
    0x100000  uImage
    0x900000  UBI: "rootfs" = the SquashFS, static (ubi.block=0,0,
              root=/dev/ubiblock0_0); "rootfs_data" = UBIFS, the overlay
              OpenIPC's /init mounts, holding /etc/fw_env.config

OpenIPC's `isvp_<soc>_sfcnand` builds do not work as they are:
`isvp_common.h` overrides the NAND settings with the NOR ones (a 230 KiB
U-Boot that the NAND SPL loads only partly, `sf` boot commands, the
environment at 0x40000 inside U-Boot). `qemu-boot/ingenic-uboot-sfcnand.patch`
restores them. And OpenIPC's `fw_printenv` without a config reads every
partition through mtdblock looking for the environment, which takes
minutes on 128 MiB of emulated NAND; the image's overlay carries the
config. `qemu-boot/test-ingenic-nand.sh` boots such an image twice (SPL,
U-Boot saving its environment, UBI, the SquashFS root, the UBIFS overlay,
login, a file written) and checks the second boot sees the environment
and the file.

On T41 the vendor U-Boot (`isvp_t41*_sfc0_nand`) has its own layout,
`ingenic-nand-image.sh --layout t41`: the uImage at 1 MiB (3 MiB), UBI
at 4 MiB with a UBIFS volume `rootfs`, `mtdparts=sfc0_nand:` on the
command line (the 4.4 driver looks for the burner table at offset 0, per
the DT, finds the SPL and falls back to `mtdparts=`); `--env` writes the
environment. The T41 SPL's `epc_debug.c` copies a stub to the on-chip
RAM at 0x12660000 (inside the NNA's ORAM) and runs core 1 on it through the CCU, so that RAM is
modelled: without it core 1 ran off the I/O space into the boot ROM's
exception vector and back into the SPL, and the SPL hung at random. A
4.4 kernel built from the SDK tree (`isvp_tomcat_nand_defconfig`) storms
on core 1's OST interrupt ("ost couldn't find ostfr"), NAND or not; the
tested T41 NAND image boots it with `maxcpus=1`.

`-kernel` with a NAND image boots with `ubi.mtd=2 ubi.block=0,0
root=/dev/ubiblock0_0` and `mtdparts=sfc_nand:` (T41: `sfc0_nand:`) by
default.

### `-kernel`

`-kernel uImage` (LZMA, gzip or none) or an ELF, with `-initrd` and
`-append`, boots the way U-Boot's `bootm` does: a stub in the boot ROM
sets UART1's LCR (the kernels' early console takes the first UART whose
LCR is not zero) and calls the entry with `a0 = 2`, `a1 = argv`
(`argv[1]` the command line), `a2 = envp`. The kernels size memory from
`mem=` alone -- no DT memory node, no DDR registers -- so a command line
without one gets the U-Boot default for the RAM size, `mem=` and the
`rmem=` the ISP driver takes. `-initrd` goes at 16 MiB and is passed as
`rd_start=`/`rd_size=`.

```bash
./qemu-src/build/qemu-system-mipsel -M t31 -m 64M -nographic \
    -kernel uImage.t31 -initrd rootfs.squashfs.t31 -nic user,restrict=on
```

With no `-append` the command line is OpenIPC's: `console=ttyS1,115200n8`,
`root=/dev/ram0` with an initrd (else `/dev/mtdblock3`), and the OpenIPC
`mtdparts` on `jz_sfc` (`sfc0_nor` on T41).

### OpenIPC images

OpenIPC releases a `u-boot-<soc>-nor.bin` (T40: `u-boot-t40n-universal.bin`)
and the `-nor-lite` kernel and rootfs, but no full image. Laid out as
the U-Boot environment expects -- U-Boot at 0, the uImage at 0x50000,
the SquashFS at 0x250000 -- they boot from flash;
`qemu-boot/test-ingenic-boot.sh --mode flash` builds that image and
fetches the U-Boot.

### Vendor SDK images

The T41, T40 and T30 SDKs ship full NOR images
(`resource/image_t*/<GRADE>_gcc*_<libc>_<8|16>M_*.img`, with a `.txt`
naming the U-Boot model and the drivers): SPL at 0, U-Boot at 0x6800,
its environment at 0x3C000, the uImage at 0x40000, the root SquashFS at
0x2C0000 and a JFFS2 `appfs` (`/system`: modules, `carrier-server`,
`start.sh`) after it. They boot from `flash-file=` as they are. The
grade decides the DRAM, which the bootargs split: T41LQ 64 MiB
(`mem=32M rmem=32M`), T41NQ / T40N 128 MiB, T41XQ / T40XP 256 MiB, T30X
128 MiB; pass that `-m` and `variant=`. The userspace has no DHCP
client: rcS starts telnetd, `app_init.sh` gives eth0 a static address
(193.169.4.151 on most images, 10.3.33.235 on the T41LQ ones and the
T41NQ 8 MiB one) and runs `start.sh`, which loads the ISP, audio and
sensor modules for the sensor sinfo finds and starts `carrier-server`, a
live555 RTSP server (`rtsp://<address>:8554/main`, `/second`). Root has
no password. `qemu-boot/test-ingenic-sdk-boot.sh --image <img>` takes
the grade, RAM and sensor from the image, rebuilds the root so that rcS
runs `app_init.sh` at the test's address (and, with a sensor and
`--no-direct-mode`, without IVDC's `direct_mode`), puts QEMU's user network on that /24 with host
forwards for telnet and RTSP, logs in over telnet
(`qemu-boot/ingenic-telnet.py`), pulls and decodes the stream, and
reboots the camera over telnet once.

## Devices

### `ingenic-cpm`

Storage plus the handshakes, none of which have a timeout that would
rescue a wrong answer: a PLL's ON bit (and LOCK, where the layout has
one) follows its enable; CPCCR's change bits read back clear and CPCSR's
busy bits read 0; a divider written with CE completes at once, so BUSY
reads 0 -- bit 28, or bit 27 in the T21..T31 SSI/SFC and LPC dividers.
The I2S dividers are M/N pairs with M in bits 28:20 and are left alone.
The reset values are what the vendor U-Boot leaves, in each SoC's PLL
encoding, because the kernels' clock code oopses before the console if a
divider's mux reads 3 or DDRCDR's reads 0:

| Encoding | SoCs | Rate |
|---|---|---|
| M[31:20] N[19:14] OD1[13:11] OD0[10:8] | T10, T20, T23, T31, T40 | 24 MHz × M / N / OD1 / OD0 |
| M[28:20] N[19:14] OD[13:11] RNG[7:5] | T21, T30 | 24 MHz × 2(M+1) / (N+1) / 2^OD |
| M[28:20] N[19:14] OD0[13:11] OD1[10:7] RG[6:4] | T41 | 24 MHz × 2(M+1) / (N+1) / 2^OD0 / (OD1+1) |

CPPSR/CPSPPR keep what is written: the reboot code writes a signature,
and "reboot recovery" waits to read it back.

### Timers: `ingenic-ost` and `ingenic-tcu`

The kernels' clock is the OST at 0x12000000: counter 1 counts to T1DFR,
sets TFR bit 0, starts again from 0 and interrupts IP4 directly while
unmasked (the clock event); counter 2 is a 64-bit free-running clock
source whose low half latches the high one into a buffer. On XBurst2 the
two are separate blocks under other names: a global OST (counter 2) and
one core OST per core (counter 1) behind the CCU's OIMR. Both run from
EXTAL through a 4^n prescaler. The line follows TFR & ~TMR whether or
not the counter runs: the T41 kernel's `set_next_event` and shutdown
stop the counter with the flag up and its handler clears the flag after,
and a line left high there (the model did) made core 1 storm with "ost
couldn't find ostfr" -- what the SDK NAND kernel did, and the SDK image
too under host load.

The SPL and U-Boot use a different counter: the OST inside the TCU at
0x10002000 + 0xE0, enabled by TESR bit 15, EXTAL/4. If it does not count,
the first `mdelay` hangs. The TCU's watchdog counts while TCER bit 0 is set
and TSR bit 16 is clear, and resets the machine at TDR: that is how
U-Boot and every kernel here reboot (the T40 kernel's own restart hook is
empty). On XBurst2 TSR bit 16 does not stop the watchdog and TCSR bit 10
zeroes its counter: the 4.4 kernels' watchdog driver sets the stop bit at
probe and nothing clears it, and their restart writes TCSR bit 10, TDR,
the clock and TCER and spins ("check wdt."). The PWM channels count and
flag without interrupts. Each channel's PWM pin (TCSR bit 7 output on,
bit 8 initial level, bit 9 shut down; OpenIPC's `jz_tcu.c` and the SDKs'
`sample_pwm`) starts a period at the initial level and flips at TDHR,
until TDFR: `qom-get /machine/tcu pwm2` reads it as `high/period` counter
ticks while the channel counts, `0` / `1` for a level that stays, or
`off` while the TCU does not drive the pin.

### `ingenic-intc`, `ingenic-ccu`

The INTC is two banks of ISR/IMR/IMSR/IMCR/IPR, everything masked out of
reset, any pending source raising IP2. The XBurst2 core INTC is the same
once per core over shared source levels; the CCU adds the per-core
IP2/IP3/IP4 enables (PIMR/MIMR/OIMR -- core 0's INTC and OST bits are set
at reset, since nothing sets them), the mailboxes that carry the IPIs,
CSSR (bit n while core n is halted) and the core reset/entry pair.

### `ingenic-sfc`

The flash controller of every part, v1 (T10..T31) and v2 (T40, T41) in
one model. A transfer is a list of phases (command, 0..7 address bytes,
dummy clocks, data) run on TRIG.START. The SPL and U-Boot move data by
PIO through the FIFO at +0x1000, polling RREQ/TREQ and END; the 3.10
kernels take the same flags as a level interrupt and DMA everything but
the JEDEC ID. v2 adds the command table at +0x800 (entries chained with
LINK, each taking its address from COL/ROW/STA0/STA1, status-poll
entries) and descriptor DMA, which the 4.4 kernels use; the SPL and U-Boot
keep the single-command path. Three details matter:

- END means the data is all in the FIFO, not that the CPU has drained it:
  it is raised once what is left of a read is one RREQ's worth, the GLB
  THRESHOLD in words (at least one). The vendor U-Boots read the words
  they want and then wait for END, leaving part of a word behind on a
  two-byte status read; a new START closes whatever the last read left.
  The T41 U-Boot's `jz_sfc_v1` loop goes the other way: one SR snapshot,
  THRESHOLD words on RREQ, stop on END in the same snapshot -- an END
  any earlier loses the words past the threshold (it did: the last 64
  bytes of every 2 KiB NAND page).
- DMA addresses are taken modulo 512 MiB. The T41 kernel hands the
  controller `virt_to_phys()` of its uncached descriptor buffer --
  0x2xxxxxxx for RAM at 0x0xxxxxxx -- so the silicon evidently decodes
  29 bits.
- The NOR is an m25p80 on the SSI bus. It counts fast-read dummies in
  clocks, one transfer each, which is what DMYBITS holds; an address
  longer than the part's (the T41 SPL probes with 6 bytes) reads data,
  as on a real chip.
- A POLL_EN phase (the NAND drivers' wait on OIP, also as a CDT entry)
  reads the status until it matches and leaves the last value in
  DEV_STA_RT (0x0C): the drivers take P_FAIL, E_FAIL and the ECC bits
  from there.

### `ingenic-spinand`

A SPI NAND on the SSI bus (`hw/block/ingenic-spinand.c`), one QOM type
per part (`w25n01gv`, `gd5f1gq4ub`, `gd5f2gq4ub`, `mx35lf2ge4ab`): READ ID
(the ID after one dummy byte; the drivers try without and with it), page
read to the cache (0x13), read from cache 0x03/0x0B/0x3B/0x6B (two column
bytes, 8 dummy clocks; the bit above the column is a plane select that a
one-plane part ignores -- the 4.4 driver sets it from the page number
for every part), program load 0x02/0x32 (cache to FF first) and random
load 0x84/0x34, program execute 0x10 (page &= cache, needs WEL), block
erase 0xD8, get/set feature A0 (protection; locked at power-up, and
program/erase fail with P_FAIL/E_FAIL while any BP bit is set), B0
(ECC enable, BUF, QE), C0 (status), D0/F0, write enable/disable, reset.
Operations finish at once; `busy-polls=n` keeps OIP up for the next n
status reads after each one. On-die ECC never reports an error.

### `ingenic-msc`

The jzmmc host of T10..T31 on an SD bus (`-drive if=sd,index=0|1`):
commands, responses, PIO through the FIFOs (U-Boot) and SDMA descriptors
(the kernels' `jzmmc_v12`). With no card, a command times out at once,
which is what lets U-Boot get past `mmc` init. MSC0's card detect is a
GPIO the host drives itself -- PB27, PB20 on T21 -- so monitor
`eject`/`change` are hot-plug. An SD image for U-Boot's `fatls` needs a
partition table. XBurst2 has SDHCI hosts instead.

### `atbm-sdio`

The AltoBeam ATBM6031/Apollo SDIO Wi-Fi the camera boards carry on MSC1,
as the vendor `atbm_wlan` driver (`hal_apollo`, `PLATFORM_INGENICT31`)
drives it; `-M t31,wifi=atbm` attaches it, with a `-nic`/`-netdev`.  It is
an SDIO function-1 card: patch 0026 lets a non-SD/MMC card sit on the SD
bus and raise its interrupt, and the MSC reflects that as IFLG bit 7.
The card answers CMD5/CMD3/CMD7, the CCCR, the function-1 FBR and the CIS
tuples (the ATBM vendor/product id), and function-1 CMD52/CMD53 to the
eight HIF registers -- CONFIG, CONTROL, the in/out queue, the AHB/SRAM
data ports with their base-address latch.  It accepts the firmware the
driver downloads into chip memory, releases the "CPU" when CONFIG's reset
bit clears, and then answers the WSM protocol itself, emulating an access
point and bridging data frames to the netdev (the WSM/AP engine it shares
with `hw/usb/atbm-wifi.c`): a scan reports an SSID (`ssid=`, default
OpenIPC), and an open-network join associates.

The driver enumerates the card, reads the CIS and brings the chip up over
the HIF registers (verified: the guest's `mmc1` reports "new SDIO card",
`atbm_wlan` probes and starts the firmware download).  Driving a full
association end to end additionally needs the MSC to complete a CMD53
data transfer as the asynchronous, multi-interrupt sequence
`jzmmc_v12`'s state machine expects (the command response and the
write/DMA completion as separate interrupts, ordered after the host
enters its data-wait state); the MSC's present synchronous completion
suffices for enumeration but not yet for the firmware download's bulk
writes.  A qtest (`test_sdio`) drives the enumeration; the guest test is
`test-ingenic-wifi.sh` (skips without the built driver).

### `ingenic-gmac`

The Synopsys GMAC 3.x every driver here (V12, V13, the T40 `ingenic_mac`,
U-Boot) runs with 8-word enhanced descriptors and DSL 2 -- 40 bytes apart
in a ring. Soft reset, MDIO and the PHY's BMCR reset self-clear (the
kernels' `open` spins on the last with no timeout); the PHY is an IP101G
(0x0243/0x0C54) at address 0 with a 100 Mbit/s full-duplex link. Received
frames carry their FCS, which the drivers subtract.

### The video encoders: `ingenic-vpu`

T10/T20's VPU (`jz-vpu.0`), the HELIX of T21/T23/T30 (H.264, JPEG) and
T30's RADIX (H.265), which OpenIPC's vendor `libimp` drives through
`/dev/soc_vpu` (`drivers/video/soc_vpu/` in the 3.10 trees; the register
and task contract, with its evidence, is `tmp/soc-vpu-contract.md`).
The kernel only resets the engine through CPM SRBC (STP bit, its
acknowledge, the SR pulse -- 30/29/31 for HELIX and the VPU, 27/26/28 for
RADIX; `ingenic-cpm` answers the handshake and passes the reset bits on
as GPIOs), enables the interrupt and gives the engine one VDMA chain:
{value, VLD | TERM | register} pairs that libimp's `H264E_T21_SliceInit`
& co. build and the engine writes into its own registers. The chain's
address goes to 0x84 on T21/T23, to VDMA 0x10008 on T20/T30, to CFGC
0x08 on RADIX; its last entry (EFE_CTRL with RUN; VDMA 0x8008 on RADIX)
starts the job, and the model runs it a virtual millisecond later.

What libimp keeps of the engine's output decides the split:

| | software (libimp) | `ingenic-vpu` |
|---|---|---|
| H.264 | SPS, PPS, slice header and `cabac_alignment_one_bits`, after the job; escapes the NAL | CABAC `slice_data()` with the flush, unescaped, at SDE_CFG4; length SDE_CFG9 |
| H.265 | VPS, SPS, PPS, slice header and `byte_alignment()`; escapes the whole NAL | `slice_segment_data()` with its trailing bits, unescaped, at SDE 0x68008; length 0x68024 |
| JPEG | SOI..SOS, EOI | the entropy-coded scan at JPGC_BSA; length JPGC_STAT[23:0] |

The coders are the repo's: `hantro-h264.c` (I_16x16 DC intra; P slices
I_PCM where the source changed, P_Skip elsewhere), `hevc-enc.c` (8x8 DC
intra, skip where the reference still matches; extended for libimp's CTB
64, 16x16 `cu_qp_delta` groups, chroma QP offset -3 and luma-only SAO) and
`jpeg-enc.c` with the quantisers taken back from the JPGC's QMEM
reciprocals. The slice QP and type come from the chain (EFE_CTRL, RADIX
EFE 0x20040/0x20000), so libimp's rate control (fixed QP, CBR) and its
slice headers agree with the slices. libimp's x264 port is CABAC-only
(`i264e_init` refuses CAVLC). The ~40 statistics registers libimp reads
back after a task read what its chain left there.

The engines' TLB: libimp addresses the engines physically, but the
driver can hand a process its DMMU table (`IOCTL_CHANNEL_PRIVATE_TLB`,
`libdmmu.c`), and with a table base set and a unit's enable on, that
unit's addresses are virtual: a 4 KiB page directory indexed by VA bits
31:22 whose entries (bit 0 valid) point to 1024-entry page tables
indexed by bits 21:12 (bit 0 valid, the physical page in 31:12). HELIX
and the VPU take the base at SCH_TLBA (0x30) and the enables in GLBC
(bits 26..21: JPGC, DBLK, SDE, EFE, VDMA, MCE; bit 30 all of them on the
older layout); RADIX at CFGC 0x10 and 0x1C (bits 16..21: EFE, VDMA, ODMA,
DBLK, MCE, SDE). The model translates the EFE's source reads, the SDE /
JPGC bitstream writes and the VDMA chain a page at a time; an unmapped
page logs a TLB error, reads as zero, drops writes and, on HELIX / the
VPU, sets SCH_STAT bit 10.

`qemu/tests/test-ingenic-vpu.py` plays the driver and libimp at register
level on every SoC (H.264, JPEG, H.265 on T30) and decodes the result
with ffmpeg, and runs the JPEG (and T30's H.265 IDR picture) again
through the TLB, over pages laid out in reverse, for the same bytes; `qemu/tests/test-ingenic-vpu-feed.py` runs libimp's own
encoders (`i264e_*`, `i265e_*`, `ijpege_*`; `qemu/tests/ingenic-vpu-feed.c`)
from a picture in memory on the OpenIPC kernel and rootfs of each SoC; `qemu/tests/test-ingenic-vpu-e2e.sh` runs the
SDK samples' FrameSource -> `IMP_Encoder` path on T21 with the ISP model
and its sensor.

T41 keeps HELIX's JPEG half as its own engine at 0x13280000
(`ingenic,t41-jpeg`, the 4.4 kernel's `drivers/video/ingenic_vpu/helix`,
`/dev/soc_vpu`, core INTC source 0): libimp encodes a JPEG channel on it,
with its own `ijpege_*`, when the channel's `encVputype` is
`IMP_ENC_TPYE_IVPU` (the default, AVPU, takes core 1 of the AVPU). The
units are renumbered -- SCH 0, VDMA 1, EFE 2, JPGC 3 at 64 KiB each,
the rest folded on 7 (`helix.h`'s HID_*) -- and the chain sets bit 19 of
every address; the model (`layout` 1) maps them back onto HELIX's
offsets, so the kernel's JPGC_STAT read at +0x30008 and the chain's
EFE/JPGC words land where the JPEG coder reads them. The reset is SRBC
28 (stop 27, acknowledge 26). `test-ingenic-avpu.sh --soc t41 --ivpu`
encodes both channels this way through libimp and the kernel driver
(2560x1440 and 640x360, decoded by ffmpeg, the scene checked);
`test-ingenic-vpu.py` has a register-level T41 case.

### `ingenic-avpu`

The T31/T40/T41 encoder is an Allegro DVT core with no MCU: `avpu.ko`
(SDK `opensource/drivers/avpu`) only proxies register reads/writes and
the interrupt (IRQ 70 = INTC 62), and libimp runs Allegro's scheduler
on the CPU (`CoreManager.c`, `EncSliceParam.c`, `SliceHdrWriter.c` in
the SDK's `libimp.a`; contract: `tmp/media-contract.md` section 3). The
`reg-base` property puts the 4 KiB register file at +0x8000 of the 1 MiB
block (T31, T40) or at +0 (T41). Core c's registers are at +0x200 c:
libimp encodes video on core 0 and JPEG on core 1.

- 0x14 / 0x18: interrupt mask / W1C status, bit 4c+0 Enc1 or JPEG done,
  4c+2 Enc2 done; 0x30 a ticking 800 MHz counter (libimp spins until it is
  non-zero); busy (0x3F8) always reads idle.
- 0x3E0 / 0x3E4: a command list (0x200-byte entries, word 0 bit 31 the
  last) and its start, 2 Enc1, 8 Enc2, 1 JPEG from R0-R10 (0x200-0x228;
  bytes written at 0x234). Jobs complete 200 us after the start, never
  inside the write. Addresses are masked to 29 bits.
- Bitstream split: libimp writes SPS/PPS/VPS/SEI and the slice header,
  the hardware the slice data from stream base + offset (re-emitting the
  0x78/0x7C header tail with emulation prevention); libimp writes the
  headers after the picture is done. T31 libimp sets Enc1 W8 bit 11 (Enc1
  codes the slice; stream buffer in WC0-WCC, bytes in 0x104), offset
  0x220, tail n = 0; the separate Enc2 pass (WF0-WFC, 0x1E4) works too.
- Slice data: H.264 from `hantro-h264.c` (CAVLC or CABAC by W8 bit 10;
  I slices I_16x16 DC, P slices I_PCM where the source changed against
  the model's own reference -- kept per reconstruction address, looked up
  by the reference address -- and P_Skip elsewhere, at most `pcm-budget`
  bytes of PCM a picture, 256 KiB); HEVC from `hevc-enc.c` with libimp's
  CTB 32, intra transform depth 1, transform skip and cabac_init_flag;
  JPEG from `jpeg-enc.c` (the whole file, quantisers de-zigzagged from
  R6). Rate-control counters in the entries are made up and non-zero.
- `-global ingenic-avpu.dump=DIR` writes every channel's stream and each
  JPEG to DIR; `trace=1` logs every start and command entry (`-d unimp`),
  `trace=2` every register write.

T41's core is the next Allegro generation (`gen=2`, libimp 1.2's
`CoreManager.c`, `EncCmdRegs.c`, `EncSliceParam.c`): the common
registers as above (mask 0x14, status 0x18, timer 0x30), then a 4 KiB
page per core at 0x1000 (c + 1) -- Enc1 at +0, Enc2 at +0x800: the
command list address (low, high) at +0, its status area at +8, control
at +0x10, start at +0x18 (2; 1 for JPEG), busy at +0x1C -- and done bits
2c (Enc1, JPEG) / 2c + 1 (Enc2). A list entry is 4 KiB: Enc1's words at
0, its status at 0x5C0 (gen 1's 0x104-0x174 minus 0x118), Enc2's words
at 0x800 and status at 0xDC0. The words that matter, read off libimp
1.2's entries: W0 codec [1:0], log2 LCU - 4 [26:24], last [31]; W4
width / 8 - 1 [10:0], height / 8 - 1 [26:16]; 0x0C gen 1's W8 (CABAC,
cabac_init, Enc1 codes the slice); 0x60 / 0x64 / 0x68 gen 1's WC / W10 /
W14; source luma / chroma 0xA0 / 0xA8, pitch 0xD0; reconstruction
0x1C0, reference 0x108; stream base / size / offset / room 0x220 /
0x22C / 0x230 / 0x234; Enc2 0x800-0x814. libimp 1.2 has Enc1 code every
slice, as T31's does. Its HEVC SPS / PPS differ from T31's (scaling
lists on with the defaults, no transform skip, depth 1), which the model
assumes for a channel's first picture. JPEG runs on core 1 from the
page's 0x40-0x88 (R0 0x40, size 0x44, tables 0x58, stream 0x60 / 0x68 /
0x6C / 0x70, pitch 0x74, source 0x80 / 0x88), bytes back at 0x604.
`avpu_g2_entry()` puts the words back in gen 1's places for the coders.

`python3 qemu/tests/test-ingenic-avpu.py` plays libimp's register and
command-list sequence under qtest (I + P in CAVLC, CABAC, Enc1-only and
Enc2 forms, HEVC, JPEG, the PCM budget, the T40/T41 bases) and decodes
the output with ffmpeg (or the `jrottenberg/ffmpeg` docker image).
`bash qemu-boot/test-ingenic-avpu.sh [--soc t40|t31|t41]` runs it end to
end: OpenIPC's kernel and rootfs, `load_ingenic` (avpu.ko among the
modules), `sensor=` and `ingenic-isp`, and an IMP_Encoder client
(`qemu-boot/ingenic-avpu-enc.c`, linked against the rootfs's own
libimp.so) that encodes the sensor's size and 640x360 in H.264, H.265 and
JPEG, one boot each. The streams come back through the SD card (T31) or
pmemsave of the top 8 MiB of RAM (T40, T41, whose SDHCI finds no card); every
picture must decode under `ffmpeg -err_detect explode` and show the
ISP's counter, logo and moving dot (`ingenic-isp-check.py`; at 640x360
HEVC's 8x8 DC blocks are coarser than the counter, so there only the logo
and the dot). The register block is 4 KiB from `reg-base`, not the MiB
avpu.ko claims: T40's RTC sits at 0x132A0000, inside it. T40's libimp
faults on a JPEG encoder channel 0; the client uses channels 4 and 5, as
the SDK samples do. T41 boots the SDK image the way the ISP test does
(below), the client linked statically against the SDK's `libimp.a`, and
loads `tx-isp-t41.ko` without `direct_mode`.

### The rest

- `ingenic-ddr`: storage, with the bits each DDR init polls reading
  done -- the DWC PHY's PGSR on T10/T20; the Inno PHY's PLL lock, init and
  calibration on T21..T31 (exact values, 3); T40's and T41's at their own
  offsets; the DDRC LMR start bit self-clearing. The remap registers remap
  nothing. DRAM size is the machine's `-m`.
- `ingenic-gpio`: S/C aliases on every register, the shadow port's
  collected set/clear masks applied by GID2LD, outputs reading back their
  level and undriven inputs reading high; one interrupt line per port.
- `ingenic-i2c`: a DW_apb_i2c master moving each command at once; an
  address no device answers aborts with TXABRT bit 0, and ENSTA follows
  ENB (both drivers wait for it). The image sensor (`sensor=`) is on
  I2C0. DMA follows the DW_apb_i2c handshake on DMACR (0 RDMAE, 1 TDMAE),
  DMATDLR and DMARDLR (0x88..0x90, named in both vendor drivers, which
  only print them): the TX request is up while TDMAE is set (commands
  leave at once, so the TX FIFO never passes DMATDLR); the RX request
  rises once the RX FIFO holds more than DMARDLR bytes and stays up for a
  burst of DMARDLR + 1 DATA_CMD reads. No driver here moves I2C data by
  DMA, so this is checked by qtest only.
- `ingenic-rtc`: WRDY always set, WEN after the 0xA55A unlock (the
  power-off path waits for both without a timeout), seconds from the
  host's clock, HCR.PD powers off.
- `ingenic-efuse`: blank fuses with every read and program done at once
  (the vendor kernels poll both and CTRL's enable). Blank keeps the
  binary-only MXU licence check in the 3.10 kernels permissive.
- `ingenic-sadc`: the AUX ADC of `jz_adc_v13.c` (3.10) and
  `ingenic_adc_v13.c` (4.4). ENA bit 15 powers it down; setting bit n
  converts AUX n 20 us later, clears the bit and raises STATUS bit n
  (write 1 to clear), interrupting unless CTRL masks it. Each pin's level
  is the `auxN` property (raw 0..4095, 1.8 V full scale; mid-scale by
  default), settable at run time: `qom-set /machine/sadc aux0 1234`.
  DMA is inferred: `jz_adc.h` names ADCFG bit 15 DMA_EN and every request
  table has SADC_RX (9), but no driver uses either. With DMA_EN the
  enabled channels convert continuously (ENA keeps its bits, a scan every
  20 us); each scan queues a word per channel pair with an enabled
  channel (AUX1:AUX0, AUX3:AUX2, ... as the data registers hold them) at
  the 0x10 port, read 32 bits at a time; request 9 is up while the queue
  (8 words) holds one, and a full queue drops the scan.
- `ingenic-pwm`: T41's PWM block (`pwm-ingenic.c`, 4.4): ENS/ENC over EN,
  WCFG taken on UPDATE, BUSYR clear, LVL and OEN; DMA mode is storage.
  `qom-get /machine/pwm chan2` reads a pin as `high/period` cycles or a
  fixed `0`/`1`. The other parts' PWM is the TCU's channels.
- `ingenic-drawbox` (T40, T41; `ingenic_drawbox.c`, `/dev/dbox`): libimp
  draws its OSD rectangles, corner marks and lines with it
  (`OSD_Draw_Layer_Cover_Pic_RectLine` -> `dbox_osd`). The driver loads a
  four-entry YUV palette (COLOR_Y/U/V, a byte each), the NV12 picture
  (YB, CB, STRIDE, IMG_WH) and two RAM words per box -- x, y, w, h, colour,
  mode (rectangle, corners, horizontal or vertical line), line width,
  corner length -- and sets START; the boxes are drawn into the picture
  in place (luma only with COLOR_MASK; BGRA pictures too) and the done
  bit (CTRL bit 4) interrupts 50 us later unless IRQ_MASK. RESET clears
  itself and empties the RAM. A line's length is its box width or
  height, a corner arm the length field: inferred from the driver.
- `ingenic-i2d` (T40, T41; `ingenic_i2d.c`, `/dev/i2d`): libimp rotates a
  frame channel with it (`IMP_FrameSource_SetI2dAttr`). IMG_MODE picks
  NV12, RAW8, RGB565 or ARGB8888, a rotation of 0/90/180/270 degrees
  clockwise, mirror and flip; START writes the transformed picture (NV12:
  the CbCr pairs moved as units at half size) to DST with its strides and
  sets frame done (IRQ_STATE bit 0, cleared through IRQ_CLEAR) 100 us
  later.
- `ingenic-vo` (T41; `drivers/video/ingenic_vo`, `/dev/jz-vo`; the
  device tree disables it, so no shipped kernel probes it): NV12 frames
  out over BT.656 / BT.1120. The driver's sequence: a DMA reset whose
  acknowledge (TOP_STATUS bit 0, write 1 to clear) follows a moment
  later, SIZE, STRIDE, CON_PAR, the blanking, INT_EN, TRAN_EN, then Y / UV
  addresses until ADDR_INFO bit 3 says the frame is queued. Each queued
  frame goes out over a frame time (`fps`, 30) onto the machine's display
  -- a QEMU graphic console, so `screendump` captures it -- and raises
  INT_VAL bit 0 (cleared through INT_CLC); `frames` counts them.
- `ingenic-dpu` (T40, T41; the 4.4 kernel's `fbdev/ingenic/fb_v12`, which
  binds `ingenic,t40-dpu`; T41's device tree calls it `t41-dpu`; the
  `/dev/fb0` of the SDK's `sample-LCD`). Neither SDK kernel builds the
  driver, so it follows the driver's contract. START (FRM_CFG_CTRL bit 0)
  takes the frame descriptor at FRM_CFG_ADDR -- next descriptor, size,
  control (bit 0: stop after this frame), write-back (not modelled), four
  layer descriptor addresses, the layer enables with each one's z order --
  and every frame time (`fps`, 60) composes the enabled layers bottom to
  top onto the display: RGB555 / ARGB1555 / RGB565, RGB888 and ARGB8888
  in 32 bits (channel order 0 RGB from the top bits down .. 5 BGR, as
  fb_v12's colour modes), NV12 / NV21 (BT.601), at their positions, a
  layer with a scale target drawn at that size (nearest neighbour), ARGB
  and global alpha blended. Then ST gets CMP_START, the layers' ends,
  CMP_END and DISP_END, INT_FLAG = ST & INTC drives IRQ 20 (CLR_ST clears),
  and the chain goes on unless the descriptor said stop; DC_CTRL's quick
  stop ends it at once, its general stop after the frame with
  STOP_DISP_ACK. `screendump device=dpu` captures it; `frames` counts.
- `ingenic-monitor` (T40, T41; the SDKs' `sample_bus_monitor`): six AXI
  channels (0..4, 6; 0x100 each) and the AHB one (5, at 0x500), each with
  CTRL, a counting window, read / write counters, a watch range and
  INT_STA / INT_MASK / INT_CLR. In monitor mode the cycle counters follow
  virtual time at `clock-hz` (300 MHz) and the window's end raises
  TIME_SLOT_REACH; capture / lock / clear latch and zero them. QEMU's
  accesses pass no bus here, so the traffic counters stay 0 and a watched
  range never matches.
- `ingenic-aip` (T41; `drivers/video/ingenic_aip`, `/dev/jzaip_t`, `_f`,
  `_p`): the NNA's image pre-processor. The kernel driver only maps it,
  hands out a chain buffer per unit and waits for the interrupt; the
  programming is Ingenic's libaip (magik-toolkit InferenceKit/nna2, under
  libvenus's `warp_*`, `crop_*`, `img_format_convert`), which maps the
  same units at 0x13090000 through `/dev/mem` and writes its chains with
  those addresses, so the machine maps them there too. The contract is
  read from that library. Three units, each with CTRL (bit 0 one job
  from the registers, 1 reset, 2 the chain, T's 4 a step), status (0
  job done, 1 chain done; write 1 to clear), CFG and an interrupt
  enable at 0x00..0x0C:
  - T (+0): NV12 to 4-byte colour: Y, CbCr, destination, strides, size,
    a signed Q20 3x3 YUV->RGB matrix (libvenus's default is BT.601:
    1.164, 1.596, 0.391, 0.813, 2.018) and offsets (16, 128) with the
    byte order.
  - F (+0x200): bilinear resize of a box: Q16 steps and half-pixel start
    phases, source / destination planes, box and output sizes, strides;
    CFG [9:7] the format -- 8-bit tensors of 1, 4, 8, 16 or 32 bytes a
    pixel, or NV12 (both planes); the output keeps it. Chains of nodes
    (one per box) run from its chain address and length.
  - P (+0x300): affine and perspective warps: the dst->src 3x3 matrix
    (libaip inverts the caller's forward one) in fixed point split over
    15 words, perspective divide, the box's pad value outside it; NV12
    comes out as 4-byte colour by the same matrix as T, 1-, 2- and
    4-byte pixels as they are.

  The 4-byte orders: bits [2:0] the order of B, G, R, lexicographic (0
  BGR .. 5 RGB), bit 3 alpha first -- libvenus's BGRA is 0, RGBA 5, ARGB
  13. A chain is (value, register) word pairs; a pair marked by bit 31 of
  its address runs the node's job. Jobs complete at once and signal 50 us
  later. Alpha is written 0 and the source order bits of CFG (bit 0 swaps
  T's Cb/Cr) are inferred, not checked against silicon.
- `ingenic-bscaler` (T40; `drivers/video/ingenic_bscaler`, `/dev/bscaler0`):
  the NNA's image engine. The kernel driver only enables its clocks, waits
  for its two interrupts and clears their status bits; the programming is
  in libvenus (magik-toolkit InferenceKit/nna1: bscaler_api, bscaler_hal,
  bscaler_segmentation, under its `warp_*`, `crop_*` and
  `warp_covert_nv2bgr`), which maps the registers through `/dev/mem`. The
  contract is read from that library -- its register writes and chains
  logged while it ran on the host under a user-mode MIPS QEMU, with the
  register access and memory helpers stood in for -- and its code. Two
  units in one 256-byte window:
  - C (0x00..0x7F, IRQ 39): warps -- resize, affine, perspective -- of NV12
    or 4-byte pixels, NV12 coming out as colour on the way. The library
    splits each box into tiles of at most 64x64 output pixels and chains
    one job per tile: source Y / CbCr, tile size and stride, the
    destination planes pushed through 0x18 (Y, then CbCr), its size and
    stride, the tile's source and destination origins (0x40 / 0x44, in
    whole-picture coordinates), and the destination -> source 3x3
    matrix (0x48..0x68, signed Q16: for a resize the scale and the
    half-pixel phase, for warps the inverse of the caller's forward
    matrix, with the perspective divide); 0x04 bit 0 / 3 say 4-byte
    source / destination and [15:8] is the value outside the source;
    0x00 = 0x201 runs a job. The chain -- (value, register) word pairs,
    bit 31 marking a job's last -- runs from 0x34 for 0x38 bytes on 0x30
    bit 0; at its end 0x30 bit 4 and 0x00 bit 8 are set and the
    interrupt raised. Tiles that map wholly outside the source get no
    job from the library, so the hardware leaves them untouched.
  - T (0x80..0xFF, IRQ 40): NV12 to colour by lines: source Y / CbCr,
    size, strides, destination, 0xAC's 4-byte bits, 0x84 = lines << 16 |
    1 converts the next lines and sets 0x84 bit 2; 0x80 bit 0 stays set
    while the frame is in progress (the library holds its lock until it
    reads it clear) and drops, with bit 8 set, at the last line.
  - Shared: the YUV->colour matrix at 0xC0..0xE0 (signed Q20, rows B, G,
    R, G's U and V terms subtracted -- libvenus's defaults are all
    positive) and 0xE4 with the CbCr and Y offsets and C's / T's output
    orders, coded as the AIP's.

  Sampling is bilinear with the edges repeated; a source position more
  than a pixel outside the tile takes the pad value (128 for CbCr). Jobs
  complete at once and signal 50 us later. Stored, not acted on: the
  per-column byte table at 0x28 / 0x2C, 0x04 bits 1, 2 and 16..18, T's
  0xA0 / 0xA8 / 0xF0 / 0xF4, and the chain-in-ORAM bit (0x30 bit 1);
  alpha is written 0.
- `ingenic-nndma` (T40, T41): the NNA's DMA between DDR and its ORAM
  (0x12600000). Ingenic's libvenus (magik-toolkit, `dma_func.cc`:
  `w_d2o_orasetrun`, `aie_run_nndma_rdchn` / `wrchn`, mapped by
  `__aie_mmap` through `/dev/mem`) drives it from user space; OpenIPC's
  `soc-nna.ko` only answers its version ioctl, enables the hardware
  registers and CU2. The descriptor RAM (0x0000..0x3FFF) holds 2048
  descriptors of two words: word 0 the DDR address (bits 31:6), bit 0
  set while another follows, bits 5:1 the length's bits 16:12; word 1 the
  length's bits 11:0 in 31:20 and the ORAM offset from 0x12600000 in
  64-byte units in 19:0 (length in 64-byte units, minus one). A write of
  bit 31 | bytes / 64 << 12 | first descriptor to read channel n
  (0x8000 + 4 n) moves DDR to ORAM, to write channel n (0x8010 + 4 n) ORAM
  to DDR, following the chain (or the byte count when not 0); it
  completes at once, so the library's `rdhwr` waits return. The kernel
  driver's own RCFG / WCFG path (0x2000, a 64-bit descriptor format,
  `CPU_SIMULATOR` in its source) lies inside the descriptor RAM and no
  library uses it: storage.
- `ingenic-ssi`: the SPI master of `jz_spi.c` (3.10) and `ingenic_spi.c`
  (4.4). A queued frame (CR1 FLEN + 2 bits, MSB first) goes out on the
  bus at once; the reply, or the frame itself with CR0 LOOP, lands in the
  128-entry RX FIFO unless DISREV. The TX FIFO running dry with CR1 UNFIN
  set raises UNDR -- the driver's PIO transfers end on that interrupt --
  and with UNFIN clear the transfer ends (END) and CE is released. CE /
  CE2 (CR0 FSEL) are the chip selects of the bus's peripherals with cs 0 /
  1: `-device w25q80,bus=ssi0` puts a flash on SSI0. The PDMA serves it on
  request types 0x16 / 0x17 (SSI0) and 0x18 / 0x19 (SSI1). No shipped
  kernel enables the driver. The SSI slave (0x10040000) is not
  modelled: the 3.10 kernel has its Kconfig entries (`JZ_SPI_SLAVE`) and
  T31's platform device, but no tree here carries the driver, so there
  is no register contract to follow. Also searched: every 3.10 and 4.4
  kernel and U-Boot tree in the SDKs, the SDK samples, and the 51 vendor
  PDFs (the T40 data sheets list only its pins, SSI_SLV_CLK/CE0/DT/DR).
- Security blocks, as the SDKs' `sample_aes`, `sample_hash`, `sample_rsa`,
  `sample_dtrng` and `sample_des` and the 4.4 kernel's crypto and
  hw_random drivers use them; everything completes at once:
  - `ingenic-aes`: ECB / CBC with 128/192/256-bit keys, by DMA (ASSA,
    ASDA, ASTC in 16-byte blocks) or one block through ASDI/ASDO; key,
    IV and data words are big-endian (the 4.4 driver byte-swaps every
    data word before and after); KEYD/AESD/DMAD under ASINTM.
  - `ingenic-hash`: MD5, SHA-1/224/256/384/512 over blocks the driver has
    padded, by DMA (HSTC in 64-byte units), the state carried across
    runs until HSCR bit 4 reloads it; HSDO gives the digest four bytes at
    a time as little-endian words (what the 4.4 driver hands its
    self-tests). HSDI (CPU mode) is inferred: no driver or sample in any
    tree here feeds it (the 4.4 driver defines HASH_FLAGS_CPU and never
    uses it; `sample_hash` only prints HSDI; no U-Boot uses HASH_BASE;
    none of the 51 vendor PDFs describes the block). It is modelled as
    the DMA path fed by the CPU: with the engine on and DMA off, each
    HSDI word is four message bytes in memory order (a load from the
    DMA's buffer), each 64 bytes (128 for SHA-384/512) run a block and
    raise HSSR bit 0, and after HSTC 64-byte units bit 1, as a DMA run
    ends; loading the state empties a part-filled block. HSCR bits 8:7
    (byte order) are acted on in neither mode: the 4.4 driver sets 3 for
    SHA and 0 for MD5, `sample_hash` 1 for SHA and 2 for MD5, and both
    expect the standard digest.
  - `ingenic-rsa`: M^E mod N, 1024 or 2048 bits (RSAC bit 7); operands
    written most significant word first, the result read least
    significant first -- the order the sample's own `test_rsa.h` vectors
    decrypt with. Polled: its interrupt is not wired.
  - `ingenic-dtrng`: a number ready 10 us after CFG bit 0, from the
    host's entropy; CFG bit 12 clears ready, bit 11 masks the interrupt.
  - `ingenic-des` (0x10061000, every part): DES and triple DES (EDE with
    K1, K2, K3), ECB or CBC (DESCR2 bits 2, 1; bit 3 decrypts). Setting
    DESCR1's work bit empties the FIFOs and loads the CBC chain from the
    IV; every second DESDIN word completes a block, ciphered into a
    16-word output FIFO that DESSR bit 0 reports empty. With DESCR2 bit 0
    the PDMA moves it: request 0x2E feeds DESDIN while the output has
    room for a block, 0x2F drains DESDOUT. Keys and IV are taken as
    written (L the high word); data words are byte-swapped with DESCR2
    bit 4 (the driver's "little endian"), so that a buffer's bytes form
    the block -- the order under which FIPS 46's worked example comes out
    of `sample_des`; nothing else pins it down.
- The OTG PHY's byte registers at 0x10060000 (`cpm_usb.c`, the 4.4 inno
  PHY driver): storage, nothing is polled.

### Sound: `ingenic-aic`, `ingenic-dmic` and `ingenic-pdma`

The vendor `audio.ko` (SDK `drivers/audio/oss2` on T10..T31,
`oss3` on T23, T40 and T41; `/dev/dsp`, which libimp's IMP_AI, IMP_AO
and IMP_DMIC drive) plays and records through the AIC, and on T40 also
records through the DMIC, their FIFOs moved by the PDMA in cyclic
transfers.

- `ingenic-pdma`: the channels of `jz_dma.c` (3.10) and `ingenic_dma.c`
  (4.4) at 0x20 n: DSA, DTA, DTC, DRT, DCS (NDES, DES8, CDOA, AR, TT, HLT,
  CTE), DCM (SAI/DAI, port widths, TSZ -- 4.4's "auto" counts bytes --,
  TIE, LINK), DDA; DMAC, DIRQP (a write keeps the bits written as 1),
  DDRS doorbells, DIP/DIC. A started channel fetches its 8-word
  descriptor at DDA; at DTC 0 one with LINK loads the next (DDA's page +
  the DOA in DTC's top byte) without an interrupt -- the drivers' cyclic
  rings never end -- and one without sets TT and, with TIE, DIRQP. Request
  type 8 (AUTO) is a memory copy done at once; the others wait on the
  peripheral's request line for every access to it: the AIC's 6 (play),
  7 (record) and 5 (loop record, the echo canceller's reference), the
  DMIC's 13, the UARTs' 0x14 - 2 n (TX) / 0x15 - 2 n (RX), the SADC's 9,
  the I2Cs' 0x24 + 2 n (TX) / 0x25 + 2 n (RX), the SSIs' and the DES's
  (below). The bus decodes 29 address bits: an address 0x20000000
  high still reaches DDR -- `jz_uart.c` maps its uncached (KSEG1) receive
  buffers through virt_to_phys, as T41's SFC driver does its descriptors.
  Of the request types the drivers' tables list, the MSC moves its data by
  its own descriptor DMA (`jzmmc_v12.c`'s SDMA, above), no driver gives
  the I2C (`i2c-v12-jz.c` only prints DMACR / DMATDLR / DMARDLR), the SADC
  or the SSI slave a channel -- the I2C and SADC sides are modelled anyway
  (above), the SSI slave has no register map -- and there is no PCM
  driver.
- `ingenic-aic`: AICFR (ENB, RST), AICCR (EREC/ERPL, the flushes, M2S,
  MONOCTR, TDMS/RDMS, ISS/OSS, TLDMS/ETFL, CHANNEL, PACK16), AICSR (levels,
  TUR/ROR), AICDR and AICLR over the hardware's FIFOs (64 to play, 32 to
  record and loop), and the codec at 0x10021000: bytes, one per word, read
  back as written. The sample rates: on T23, T31, T40 and T41 the AIC is
  the I2S master, and the drivers set the CPM's speaker clock
  (I2SSPKCDR / I2STCDR 0x70) to 256 times the play rate and its
  microphone clock (0x84) to 256 times the record rate, each when that
  side starts; the AIC paces each side by its own clock (parent PLL x M /
  N, a rate within 3 % of a standard one taken as that). On T10..T30, or
  with that clock gated, the codec clocks the AIC and its CSRR (0x110
  bits 2:0, 8 kHz to 96 kHz) is the rate. While ENB is set, frames move at
  those rates in virtual time:
  played samples (one per frame with M2S, CHANNEL + 1 otherwise) leave the
  TX FIFO for the machine's audiodev as 16-bit stereo; the RX FIFO gets
  one sample per frame with MONOCTR (T21, T31: the right channel) or two:
  a 1 kHz sine at a quarter of full scale (`-global
  ingenic-aic.tone-hz=N`, 0 for silence) or, with
  `-global ingenic-aic.rx-source=audiodev`, the host's input; the loop
  FIFO gets what is played. Requests: TX while TDMS and the TX FIFO has
  room, RX while RDMS and it holds a sample, loop while TLDMS and it does.

The codec's analogue paths follow the SDKs' codec drivers
(`jz_t10_codec.c`, `inner_codecs/T23/codec.c`): on T10, T20, T21 and T30
the playback gain is CHCR (0x9C) bits 4:0 with the output on while CAR
(0x94) bits 3:0 are, the record gain CACR2 (0x8C) bits 4:0 with the ADC
on by CAMPCR (0x90) bit 7; on T23 and T31 playback is CHR (0xA0) bits 4:0
with the headphone output on by CHR bit 7, record CALCGR (0x94) bits 4:0
with the ADC on by CMICCR (0x8C) bit 7, and CMICCR bit 4 (ALC) holds the
level whatever the gain. 1.5 dB a step, unity at the drivers' defaults
(0x18 to play, 0x10 to record); a path that is off is silent. T40 and
T41 (`inner_codecs/t40` and `T41/codec.c` of their SDKs) share the
offsets: playback CHR_2b / CHPOUTLGR (0xAC) bits 4:0 (unity 0x18) with
the driver's output on by CHR (0xA8) bit 6; record CANACR_26 / CALCGR
(0x98) bits 4:0 (unity at the drivers' 0x10 on T40, 0x1F on T41) with
the ADC on by CMICCR (0x8C) bit 7, the ALC holding the level with CGAINR
(0x28) bit 5 / 4; T40 records mono through its right ADC (CMICCR 0x0E:
bit 3, gain 0x9C, ALC bit 4). Their digital volumes, 0.5 dB a step, play
CDGSR (0x18; 0xF1 is 0 dB, 0 mutes) and record CLAVR (0x20, T40's right
0x24; 0xC3 is 0 dB): no driver writes them -- libimp's IMP_AI/AO_SetVol
scale in software -- so they come out of reset at 0 dB, an assumption
(the reset values are not documented). IMP_AI_SetGain / IMP_AO_SetGain
reach the analogue gains.

- `ingenic-dmic` (T31, T40): `audio_dmic.c` (oss3, T40's `audio.ko` with
  `dmic_enable=1`) and `devices/dmic` (oss2, T31, behind
  `CONFIG_JZ_TS_DMIC`, off in the shipped kernels). CR0's enable, rate (8,
  16, 48 kHz), microphone count and PACK_EN (two 16-bit samples to a FIFO
  entry, the earlier in bits 15:0); a 64-entry FIFO read at DR; FTHR's
  RDMS raises request 13 while it holds an entry; FSR's level and
  FULL/EMPTY, INTCR's flags under IMR (the drivers mask them all). Every
  microphone hears a sine of `-global ingenic-dmic.tone-hz=N` (1 kHz)
  at the peak `tone-amp` (8192; settable at run time,
  `qom-set /machine/dmic tone-amp N`, 0 for silence).
  The other parts' builds drive none but T41's.
  The voice trigger (the data sheet's "low power voice trigger", with
  pre-fetch while the data interface is off) follows the voice-wakeup
  firmware of the same block (`drivers/char/voice_wakeup`; no T-series
  audio driver sets TRI_EN): with CR0's EN and TRI_EN each sample is
  compared with THRL; TRICR mode 0 fires on the first sample over it,
  mode 2 once more than TRINMAX have been (other modes are not described
  anywhere here and act as mode 0). Firing sets INTCR's TRIFLG (bit 0,
  interrupting unless IMR bit 0) and FSR's TRIGS (bit 18) and holds until
  TRICR's TRI_CLR or CR0's RESET_TRI (self-clearing) rearms it. SR 3,
  the firmware's "data path off", still clocks the trigger at 16 kHz and
  fills no FIFO. THRH and TRIMMAX are storage. T41's audio-system DMIC
  defines its TCR but no field of it, and is not given a trigger.
- `ingenic-asdmic` (T41): T41's audio system at 0x134D0000 (core INTC
  33), which the T41 SDK's `audio.ko` registers as its own platform
  devices and drives with `dmic_enable=1` (oss3 `host/T41/dmic`; the
  device tree's `ingenic,dmic` node stays disabled): the DMIC at +0xA000
  (CR0 enable, rate 8/16/48/96 kHz, microphones, 16/24-bit; GCR gain,
  storage), routed over the data bus (+0x4000) and the format converter
  (+0x2000, storage) to FIFO 5 (+0x1000) and DMA channel 5 (+0x0):
  a closed ring of 4-word descriptors (flags, DBA, the next descriptor's
  bus address, DTC in bursts), which DBA follows as the samples land --
  the driver's work queue polls it; no interrupt unless a descriptor asks
  (TIE / LTIE, AIPR). Every microphone hears `-global
  ingenic-asdmic.tone-hz=N` (1 kHz).

`-audiodev wav,id=snd,path=out.wav -M t31,audiodev=snd` records what the
guest plays.

The UARTs transmit at the line rate. On T40/T41 (4.4, `ingenic_uart.c`)
the driver's interrupt handler sends and stops the transmitter without
the port lock, racing `start_tx` on the other core, and a lost race
leaves bytes queued with the THR interrupt off -- the console stops for
good. QEMU's 16550 sends a whole burst at once, so THRE (and the
interrupt) came back after every write and the race was lost within a
few hundred lines of heavy output: with a client printing 500 lines of
hex from a thread while IMP_AO plays (`test-ingenic-audio.sh --soc t41
--console-load`), the console stopped after 26, 101 and 279 lines in 3
of 3 runs. Patch 0021 adds a `paced` mode to QEMU's serial -- a byte
leaves the shift register a character time after it entered it, THRE
comes when the FIFO really drains, TEMT a character later -- and a
`tx-fifo` depth; the machine sets both (64 bytes, the driver's
`fifosize`, which loads half of it per interrupt) with a baud base of
EXTAL / 16 (1.5 MHz: the drivers program the divisor for 24 MHz, 13 for
115200). The same test then prints all 500 lines in 3 of 3 runs. It
narrows the window to silicon's rather than closing it: 10 of 11 OpenIPC
T40 boots passed paced, one went quiet after its application started.
The cost is the real rate, some 11.5 KB a second: an OpenIPC boot takes
about 2 s longer, a 420 KB console flood 41 s instead of 6, and the test
harnesses wait for the console to drain before they power off.

Bytes come in at the line rate too (patch 0023, with `paced`): one a
character time. The UARTs have the jz47xx's 64-byte receive FIFO, whose
FCR trigger levels are 1, 16, 32 and 60, and IER bit 4 (RTOIE) enables
the receive timeout interrupt by itself. And they request the PDMA in
FCR's DMA mode (DMS): TX while the transmit FIFO has room, RX once the
receive FIFO reaches its trigger level, held for that many bytes -- one
request, one burst -- so a shorter tail stays for the timeout interrupt.
That is what `jz_uart.c` does with `CONFIG_SERIAL_JZ47XX_UARTn_DMA`
(trigger 32, 32-byte PDMA units into two 1 KiB buffers, IER 0x14: the
timeout and line status, no RDI; the tail by the timeout interrupt, the
residue and PIO). No shipped config sets it; with it, on a T31 kernel
built so (`test-ingenic-uart-dma.sh`), 32 KiB come in on UART0 and go
back out intact. Two of the driver's own faults show there: T31's PDMA
table (`jzdma_pdata`) has channels for UART0 and UART1 only, so with
UART2_DMA the driver dereferences a channel it never got; and its
shutdown frees the receive buffer's pages wrongly ("Bad page state" at
close), so the test keeps the port open throughout. Without paced input
the host's bytes all arrived at once and the driver lost data in the
window between reading a channel's residue and stopping it; at the line
rate a burst takes 2.8 ms to fill and the window is as narrow as on
silicon.

### The image pipeline: `sensor=` and `ingenic-isp`

`-M <soc>,sensor=<name>` puts an image sensor on I2C0 (I2C1 on T40) at
the address and with the ID that OpenIPC's `sinfo.ko` table probes:
`gc2053` (0x37, `galaxycore-sensor`), `gc2083` (0x37, 16-bit registers),
`sc2336` and `sc2335` (0x30, `hisi-smartsens`), `jxf37` and `jxf22`
(0x40, `hisi-f37`, `hisi-jxf22`), `ov2735b` and `ov2735` (0x3C,
`ingenic-ov2735`: 8-bit registers in pages, the revision in page 0's
0x04 telling the two apart), `gc4653` (0x29, 16-bit registers; T40, on
I2C1), `gc4663` (T41, the same ID), `jxh62` (0x30, `hisi-jxh62`; T10).
The default is none. On T40 `sensor1=<name>` (the same names) adds a
second sensor on I2C3, the second pipe's, and `sensor2=<name>` a third
on I2C0, VIC2's (below). T10's kernel bit-bangs the sensor bus (`i2c-gpio`,
SDA PA12, SCL PA13), so there the sensor hangs off QEMU's `gpio_i2c`,
wired open-drain to those pins through `ingenic-gpio`'s new "drive"
outputs (a pin's level as the SoC drives it: low for a GPIO output at
0, else released). The models answer whether or not MCLK
runs and the PA18 reset is pulsed, so sinfo's detection needs nothing
else; `/proc/jz/sinfo/info` reads `sensor: gc2053`.

`ingenic-isp` is what the vendor `tx-isp-<soc>.ko` drives on T31, T23
and T21 (read off the modules: `tx-isp-t31.ko` H20221206a with the SDK
1.1.5 build's symbols, `tx-isp-t23.ko` from the T23 SDK 1.1.0 build with
symbols and OpenIPC's H20240204a, `tx-isp-t21.ko` H20200509a). The
three share one design and differ in where the blocks are. It has no sensor data: each
frame it paints a test picture into the buffers the driver queued.

- ISP core: storage, but for the interrupt enable / status / clear
  (T31 0xB0/0xB4/0xB8, T21 0x80/0x84/0x88; status bit n is channel n
  done, 0x1000 frame start on T31) and the MSCA scaler's three DMA-out
  channels at 0x9900 + 0x100 n (T21 0x2400): QBUF pushes the Y and UV
  addresses of an empty buffer (C+0x6C, C+0x84), the ISR pops filled
  ones (C+0x74, with the UV address and a frame number at C+0x8C/C+0x90,
  T21 C+0x78) until the done FIFO's empty bit (C+0x7C) is set; C+0x2C is
  the size the channel writes, C+0x80/C+0x98 the strides; 0x9804 (0x2304)
  bit n enables channel n; a write to C+0x9C..C+0xA8 empties the FIFOs
  (T31). T21's channel-stop poll on 0x2308 reads 0. T23 moved the
  channels to 0xD100 + 0x100 n with other offsets (push C+0x30/C+0x50,
  pop C+0x38/C+0x58, empty C+0x3C, size C+0x0C, strides C+0x40/C+0x60,
  enable 0xD040).
- VIC: the commands 2 and 4 written to 0x00 read back 0 at once (T31
  spins on it with no timeout), 1 starts the input. Status / mask (1 =
  masked) / write-1-to-clear at 0x1E0/0x1E8/0x1F0 (+4 for the second
  word; T21 0x100/0x104/0x108), bit 0 frame done -- the counter
  `/proc/jz/isp/isp-w02` prints and libimp's "video drop" watchdog reads.
- CSI host and D-PHY: storage (the driver only writes them and reads
  ERR1/ERR2 as 0). T21 has none; it is DVP only. T23's IVDC (the direct
  ISP-to-encoder path at 0x13200000) is used only with the module's
  `direct_mode`, off by default, and is left unmodelled.
- `ingenic-cpm`: before resetting the ISP the driver sets SRBC (0xC4)
  bit 21 and waits a second for bit 20, which now follows it.

Frames run at `-global ingenic-isp.fps=` (25) while the VIC is started
and the ISP's interrupts are enabled: frame start, then for every enabled
channel with a queued buffer the picture at its size and strides, the
buffer into the done FIFO and its status bit, then VIC frame done. The
picture is the OpenIPC logo on a navy gradient with a dot sliding beneath
it (`sstar-pattern.h`, as `aw-vinc` and the SigmaStar models draw it) and
a six-digit frame counter at the top left.

The 3A loop is closed (T31, T23, T40, T41; read off `tx-isp-t31.ko` and
`tx-isp-t23.ko` of the SDKs, with symbols, and the T40/T41 SDK builds).
The scene is lit by what the sensor model's registers say the kernel's
AE programmed -- its rows times its gain, over 1000 rows at gain 1.0,
times `light` (percent, 100) -- read the way each part's SDK sensor
driver writes them: GC2053 page 0 0x03/0x04 and 0xB8/0xB9, GC2083
0x0D03/0x0D04, GC4653/GC4663 0x0202/0x0203 and 0x02B8/0x02B9, SC2336
0x3E00-0x3E02 and 0x3E07/0x3E09 (`include/hw/i2c/ingenic-sensor.h`).
The painted scene counts as display light (gamma 2.2): the exposure
scales its linear light, white clips, and the picture is gamma-encoded
again, so a longer exposure lifts the navy while the logo stays white.
At each frame's end the model
writes the AE and AWB statistics of that picture by DMA into the banks
`tisp_init` gave the hardware, and raises their status bits:

| | T31, T23 | T40, T41 |
|---|---|---|
| AE0 block, AWB block | 0xA000, 0xB000 | 0x19000, 0x18000 |
| status bits (AE zones, AE histogram, AWB) | 26, 27, 30 of 0xB4 | 4, 5, 3 of 0x40028 |
| AE zone, 16 bytes | dark, middle, bright luma sums (21 bits), dark and bright counts (13) | the same, 22 and 14 bits |
| AWB zone | 16 bytes: R, G, B, IR sums (21 bits), count (13) | 128 bytes: 4 pairs of records of R, G, B, IR (22), count (14): every quad, then the filtered ones |

In both blocks +0x04 is the zone grid (columns [31:28], rows [15:12]:
15 x 15), +0x08-0x14 and +0x18-0x24 the zone widths and heights a byte
each in 2x2-pixel quads, +0x4C the bank counts and +0x50 the bank last
written (AE: zones [5:4], histogram [1:0]); AE +0x28 holds the dark and
bright thresholds ([7:0], [15:8]), +0x2C-0x38 four zone banks and
+0x3C-0x48 four histogram banks (256 21-bit counts of the quads by luma,
twice); AWB +0x3C-0x48 four zone banks. The kernel's `ae0_weight_mean2`
takes a zone's three sums over its width x height as its mean. Zones are
sampled 8 x 6 from the scene, in linear light (8 bits), as a sensor's
statistics are; the light's colour scales the linear R and B, so that
a white reads R `light-r` and B `light-b` per mille of G -- by default
the SoC's daylight raw white, where its tuning's AWB looks for one (T21,
T23, T31: 550, 600; T40, T41: 460, 550, the white their start gains
2212 / 1856 make neutral). The AWB block sums only the quads inside the pixel
filter `tiziano_awb_set_hardware_param` programs after the zone sizes
(T21 0x828-0x838, T31 and T23 0xB028-0xB038): +0x28 a raw R/G window
and +0x2C a B/G window (Q8, min in [15:0], max in [31:16]), +0x30 and
+0x34 lower and upper bounds on k R/G / 256 + B/G (k in [15:0], bound
in [31:16]), +0x38 a luma window ([7:0] low, [15:8] high; luma taken
as (R + 2 G + B) / 4) -- the driver's all-pass values (R/G 1..4095,
bounds 0 and 65535) read so -- and a zone's count is of the quads that
passed. T40 and T41 program the same form at 0x18028
(`tisp_awb_set_regional_threshold`, bounds interpolated by EV: on the
T41 SDK image R/G 60..245, B/G 30..200, k 256 with bounds 170 and 320;
T40's wider, R/G 53..284, B/G 29..304, bounds 167 and 394), but +0x38
holds three luma thresholds in bytes 0..2 and a frequency in byte 3
(`tisp_awb_set_lum_th_freq`): the window is bytes 0 and 2 (T41 10..230,
T40 0..230). Their 128-byte zones are four pairs of 16-byte records
(`tisp_awb_get_statistics`: R, G, B and a fourth sum in 22 bits from
bits 0, 22, 44, 66, the count in 14 from bit 88), a pair per
statistics position; the tuning picks the pair read (its block's bytes
3274 / 3276). Of a pair the AWB's gains follow the second record
alone; the model fills the first with every quad and the second with
the filtered ones.

The scene has a grey step wedge for the AWB to find a white in
(`ingenic-isp-paint.h`: six neutral patches below the dot, bottom
left): with only the navy, the logo and the dot, an AE that exposes for
the navy -- T40's and T41's do -- leaves the white saturated, past the
luma window and the R/G limit, and no zone inside the filter, as on a
camera pointed at such a scene; the AWB then holds its start gains.
`light`, `light-r` and `light-b` can change while running
(`qom-set /machine/isp light 30`). The kernel's AE starts high and
settles over a second or two -- on T31 a light of 30 % and of 100 % give
the same picture by frame 40 (the navy at Y 39) -- and, metering linear
light with this dark scene, exposes for the navy (Y ~90 on the T40 and
T41 SDK images). The AWB gains follow the light's colour (per 1024,
once settled; warm R 0.9 / B 0.35, cool R 0.45 / B 0.75):

| | daylight | warm | cool |
|---|---|---|---|
| T21 | R 1860 / B 1712 | R 1140 / B 2740 | R 2272 / B 1360 |
| T31 | R 1860 / B 1704 | R 1136 / B 2916 | R 2280 / B 1364 |
| T23 | R 1856 / B 1704 | | |

T40 and T41 settle on the gains that make the light's white neutral,
1024 / R and 1024 / B to within 1-3 % (daylight 460 / 550; a white of
500 / 430, warm 700 / 280, cool 380 / 600, all inside both SoCs'
windows):

| | daylight | 500 / 430 | warm | cool |
|---|---|---|---|---|
| T40 | R 2220 / B 1864 | R 2048 / B 2380 | R 1468 / B 3636 | R 2688 / B 1712 |
| T41 | R 2228 / B 1856 | R 2056 / B 2388 | R 1468 / B 3704 | R 2716 / B 1704 |

A light outside a tuning's windows (R 0.35 / B 0.9 is below both
R/G minima; on T40/T41 a light whose R/G + B/G falls under the lower
bound) leaves no quad to sum, and the AWB keeps its defaults, as it
would on a camera. The white logo itself saturates once the AE brings
the navy up -- a full-scale reflector against a 1.2 % one, some twelve
times past full scale -- and fails the luma window on silicon too; the
wedge's darker patches are what the AWBs settle on (T23, which held its
defaults before, now settles at daylight's 1856 / 1704).

T31 and T23 also produce their other statistics
(`ingenic_isp_xstats()`), each into the banks `tisp_init` gives it:

| | block, banks | status bit | layout |
|---|---|---|---|
| AE1 | 0xA800, AE0's form; only with 0x0C bit 0 clear (a WDR mode) | 28, 29 | AE0's zones and histogram, of the short frame |
| AF | grid 0xB804 (pixels; 8 x 8 zones of 238 x 134 at 1080p), banks 0xB8A8, last bank number 0xB8BC | 31 | 16 bytes a zone: FIR 0 / 1, IIR 0 / 1 (22 bits each), luma sum (23), bright count (15) |
| ADR | banks 0x4494, last bank's address 0x44B0; 0x0C bit 7 bypasses | 18 | 6 x 8 blocks of luma sum and a five-bin histogram, then a 512-bin luma histogram (three 21-bit counts per 8 bytes) |
| defog | banks 0x5B84, last bank's address 0x5BA4; 0x0C bit 11 bypasses | 20 | from +8, 180 blocks of B, G, R sums (24 bits) and three halfwords (here the dark channel, brightest and mean luma) |
| WDR (T31) | banks of 8 KiB at 0x2010, last address 0x2680; only with 0x0C bit 0 clear | 11 | long and short frame R, G, B histograms (256 x 21 bits), luma counts and sums, in eight 516-byte rows 1 KiB apart |

AE1 is the WDR short frame's AE. Its block's bit 0 is a latch
(`system_reg_write_ae(2)` sets it after every AE1 update), not an
enable; raised in a linear mode, `ae1_interrupt_hist` stores its
histogram over the AE's static data and pushes the AE1 process, and
T31's AE and AWB stop where they are (the gains frozen from frame 10),
so like WDR it runs only while 0x0C bit 0 is clear.

#### WDR: two exposures

The sensors' two-exposure (DOL, line-interleaved) WDR modes run end to
end. T31 enters one with the sensor module's `data_type=2`
(`TX_SENSOR_DATA_TYPE_WDR_DOL`) and `IMP_ISP_WDR_ENABLE` between
`IMP_ISP_Open` and `IMP_ISP_AddSensor`; T40 and T41 with
`IMP_ISP_WDR_ENABLE(IMPVI_MAIN, &mode)` before the sensor is added and
the sensor's WDR setting (`default_boot` 1). `tisp_init` then clears
bit 0 of the bypasses (T31, T23: 0x0C, 0xa1fc0202 on the OpenIPC image;
T40, T41: 0x40, 0xff9100e0 on the T41 SDK image against 0xffd00049
linear), which is what the model takes as the WDR mode. The sensor
gets both exposures over I2C: the GC4653's and GC4663's long rows at
0x0202:0x0203 as in a linear mode, the short frame's at
0x0200[5:0]:0x0201 (`gc*_set_integration_time_short`), the gain shared
(`galaxycore_exposure()` reports both).

Each frame the model lights the scene with each exposure: AE1's zones
and histogram (T31, T23) and the WDR statistics' short-frame half come
from the short one, and the picture is the fusion -- the long frame
where it does not clip, the short frame scaled by the exposure ratio
where it does, then an extended Reinhard curve whose white is that
ratio (`isp_wdr_tone()`), so a white the long frame clips keeps its
shading. T40 and T41 (`tisp_wdr_interrupt_static`, T40's
`tisp_wdr_main_interrupt_static`, bit 11 of 0x40028 on both) take four
6 KiB banks at 0x6010.., 0x6030 the last one's number;
`tisp_wdr_get_data` reads 456 24-bit values (the first 450: 15 rows of
15 long-frame then 15 short-frame blocks -- here luma sums), four
12-bit and two 23-bit values at +1352 / +1360 (here the frames' mean
luma, bright and dark counts), 160 values from +1368 (left zero, not
identified) and, when the tuning asks, the long and the short frame's
R, G, B histograms from +1848 and +3896 (256 entries of three 21-bit
counts).

The kernels' WDR processes run on them: on T31 134 of the 229 WDR
registers (0x2000..0x28FF) change from frame to frame; on T41 its
process writes some 29 registers a frame. AE1 does not close a loop of
its own: `tisp_ae1_process` hands its zone means and histogram to the
WDR module (`tisp_wdr_rx_ae1_infm`), and `tisp_ae1_expt` derives the
short exposure from the long one's (rows, analog and digital gain)
through the tuning's `ae1_ev_list` tables. On T40 and T41 it follows
the long exposure (T41: 7 to 94 rows as the AE settles); with the
OpenIPC T31 tuning (`gc4653-t31.bin`) it stays at the sensor's longest
short exposure, 310 rows, for every long exposure from 1486 rows down
to 4. On the scene at the AE's exposure, a linear frame clips 7.5 % of
its pixels (the logo), the WDR frame 0.4 % (T31, T40, T41;
`test-ingenic-isp.sh --wdr`). T23's WDR-capable drivers (imx327,
sc301IoT) are for sensors not modelled here.

T40 and T41 produce theirs too (`xstat` 3; their symbols):

| | block, banks | status bit | layout |
|---|---|---|---|
| AF | grid 0x1A004 (T31's word), widths as halfwords from 0x1A008, heights as bytes from 0x1A028; banks 0x1A0CC, count 0x1A0DC, last bank number 0x1A0E0 | 6 | zones of 136 bits end to end: FIR 0 / 1, IIR 0 / 1 (23 bits each), luma sum (24), bright count (16); a row every 2 x {3, 5, 7, 9, 11, 13, 15, 17, 20, 22, 24, 26, 28, 30, 32}[columns - 1] words |
| ADR | banks 0x9514, count 0x9510, last bank's address 0x9540 | 9 | 4 x 6 blocks of 56 bytes (two 29-bit words, twenty 18-bit counts, two 12-bit fields: here luma sum, quads, a 20-bin histogram, darkest and brightest luma), then the 512-bin histogram from +0x548 |
| defog | banks 0xC290 (2 KiB), count 0xC28C, last bank's address 0xC2B4 | 10 | from +8, 180 blocks of 8 bytes: B, G, R sums (21 bits) |
| GSM (T41) | RAMs read through ports: 0x201 to 0x501A0 / 0x501B0, then 256 reads of 0x501A8 / 0x501B8; flags 0x1B004 / 0x1B044 | 7 | two 256-bin luma histograms that add up to exactly width x height (0x80; `tisp_gsm_process` checks the total) |
| LCE (T41) | ports 0x501C0 / 0x501C8 (45 x 32 words) and 0x50200 / 0x50208 (45), the second set at +0x10; 0x0E014 bit 0 ready, bit 8 the set | 8 | 9 x 5 blocks of a 32-bin luma histogram of quads (`lce_std_hist_transform` rejects a block that counts more than it has), then the blocks' means |
| TMO (T41) | banks 0x1E010 / 0x1E014 (16 KiB), last bank 0x1E028 | 17 | 3750 halfword pairs: here a 75 x 50 grid of mean and brightest luma (12 bits) |

T41's `IMP_ISP_Tuning_GetAFMetricesInfo` reads about 3.4 million (the
alternate about 1.8 million), 0 without; ADR rewrites its outputs
(0x50024..0x5040C) about three times as often and defog its (0xC0xx)
half again as often as with no statistics. Which reader field means
what (ADR's, defog's order, TMO's pair) is the model's reading of their
widths. T40 reads its GSM and LCE through other paths and has no TMO;
those are T41's only. T40's `tisp_af_get_metric`, asked for the AF
metric from a single-sensor run once AF statistics arrive, faults on a
pointer of its own (a TLB load in module space); the tests do not ask
it there.

The AF filters are the luma's differences to the next pixel and the
one after, along the row (FIR) and down the column (IIR), summed over
the zone. With them `IMP_ISP_Tuning_GetAFMetrices` on T31 reads about
1.6 million, and 0 without; when the light quadruples and turns warm it
drops to about 0.9 million as the logo clips. With ADR and defog fed,
their algorithms run every frame and write their outputs (ADR
0x4270..0x43A4, defog's per-block transmission and air light from
0x58000) where without statistics only their init values are written
once; on the static scene those outputs settle and stay, also across a
change of light, and which of defog's three halfwords means what is a
guess.

The tests that check pictures take them once the AE has
settled (`test-ingenic-isp.sh` the last frames of 40, the encoder client
after four seconds), and `ingenic-isp-check.py` judges the logo and the
dot against the background's level, and the dot's place by the frame
counter, which is drawn on a black box so that it reads at any
exposure.

T21 (`tx-isp-t21.ko` H20200509a, with symbols) has the same blocks at
0x700 (AE0) and 0x800 (AWB), banks set by `tisp_init`, its callbacks on
bits 16 (AE zones), 17 (AE histogram) and 18 (AWB) of 0x84, and no
enable bits: a block runs once it has its banks. Its AE zones are T31's;
its histogram counts in 20 bits, and its AWB zone has no IR sum (R, G,
B, then the count at bit 63). Its sensor, JX-F37, is decoded as the
other SOI parts (rows in 0x01/0x02, gain in 0x00). With T31's scale its
AE settled at a linear mean of 2 of 255 -- too dark to see the scene --
whatever the light; its target is a table in the sensor's tuning file,
so the zone sums are given at a quarter scale, a calibration rather than
something read off the module, and the AE settles at 7 (T31: 10) with
the light at 10 %, 100 % or 1000 %. Its AWB (`Tiziano_awb_fpga`, run
every frame -- `_AwbPointPos[1]`, the change it waits for, is 0 in the
JX-F37 tuning) takes each zone's R/G and B/G times the tuning's static
gains (1526 / 1583, Q10), drops the zones whose count is at most
`_pixel_cnt_th` (25), and hands them to `Tiziano_Awb_Ct_Detect`, which
returns the white point (Q8): a zone on the tuning's colour-temperature
band, about R/G 180-400 against B/G 260-148, comes back as itself, one
off it is pulled to the band's nearest end, and all zones weigh in. The
navy background -- most of the scene, B/G over 5 -- outvoted the white
of the logo and pinned the result at the band's blue end (R/G 167, B/G
340) whatever the light, until the AWB block's own pixel filter (above)
left it out, as the silicon's does: its quads fail the B/G window and
the luma floor (50). Read off the module by running the detector on
zones captured from a guest (gdbstub, the module's pages walked through
`swapper_pg_dir`) in an emulator.

`-global ingenic-isp.trace=on` logs the MSCA and 3A register writes and
each frame with its light, mean luma and white-balance gains; `-global
galaxycore-sensor.trace=on` the sensor's register writes.

T20 and T10 are the older design, an Apical ISP core, and have their own
model, `ingenic-isp-apical` (read off the OpenIPC `tx-isp-t20.ko`,
software H201709151519 with the Apical firmware H01-380; `tx-isp-t10.ko`
is the same software, and differs only in its VIC, whose control is at
0x20 -- 0x00 is configuration, 0x0C the size -- and in taking at most
1280x960 from the sensor, a limit in the driver):

- core 0x13380000-0x133FFFFF: storage (libimp maps 0x1B000 of it through
  `/dev/mem` and tunes it directly), but for the 16 interrupt slots:
  status 0xA0, write-1-to-clear 0xA4, and each slot's event in 6 bits of
  0x80 + 4 (slot / 2), which the driver programs (frame start 24, frame
  end 23, the three DMA writers' done 14 / 15 / 1, DS1 / DS2 end 50 / 53).
  0x134 reports the run state 0x130 asks for (the stop path spins on it).
- the three DMA writers at 0xB00 + 0x100 n (CbCr at +0x80): enable bit 9
  of +0x00, size +0x04 (w | h << 16), up to five bank addresses from
  +0x08, the bank count in +0x1C, the line offset +0x20, and +0x24: the
  driver writes 2 or 3 as the next bank does or does not hold a buffer,
  and reads the current bank in [10:8]. It refills empty banks from its
  frame start and frame end ISRs, so a frame takes the next bank when
  the last control was 2 and is dropped when it was 3.
- VIC 0x13300000 (0x00: 2 reads back 0, 1 starts) and the interrupt
  block at 0x13320000 that puts both on IRQ 37: status 0x0C (the core's
  slots in 15:0, VIC frame done in 16), ack 0x04, enable 0x14, mask
  0x1C. CSI at 0x10040000, whose PHY_STATE (0x14) reads 0x630, the stop
  state the init polls for.
- 3A statistics (the same in `tx-isp-t10.ko`, `-t20.ko` and the T30
  SDK's `-t30.ko`, all with symbols): `apical_fw_interrupts_init` puts
  the AE statistics on event 11 and the AWB statistics on event 9, and
  their handlers read them straight from the core. The AE takes a
  256-bin histogram at 0x10000 + 4 b (`ae_read_full_histogram_data`:
  a 12-bit mantissa and a 4-bit exponent, m for e = 0, else (m | 0x1000)
  << (e - 1)); the AWB 15 x 15 zones at 0x8740 + 8 z
  (`awb_read_statistics`, 225 zones from `AWB_fsm_clear`): R/G in
  [11:0] and B/G in [27:16], 1.0 = 256, before the white balance, and
  the zone's weight at +4. At each frame end the model fills both from
  the scene at the first enabled writer's size, lit as on the later
  parts (the sensor link, `light`, `light-r`, `light-b` at
  `/machine/isp`; OV2735B decoded from page 1, rows 0x03:0x04 and gain
  x16 in 0x24), and raises the two events. The firmware's AE starts
  from the sensor's init exposure -- a nearly black frame -- and climbs
  to the scene over two or three seconds (T20 and T30: still rising at
  frame 75; T10 settles at frame 67); with three times the light T20
  reaches the same picture on a third of the exposure. The AWB's gains,
  which it writes to 0x300 (B on the BGGR JX-F22) and 0x30C (R), go
  from B 1.00 / R 2.75 in daylight to B 1.16 / R 2.24 under a warm
  light. `-global ingenic-isp-apical.trace-core=on` logs every core
  register the firmware changes.
- No other statistics are read. The Apical firmware has no autofocus:
  `af_status` returns 2 with nothing measured and `af_mode`, `af_range_*`
  and `af_roi` only keep values, in all three modules; event 14 (slot 5)
  has no algorithm behind it. Iridix (the DRC) reads no statistics block:
  `iridix_fsm_process_interrupt` runs on frame end from the exposure set
  and core 0x2C0 bit 7, so its outputs follow the AE -- on T20 the
  strength in 0x39C goes 0x1000 -> 0x800 and the gain in 0x3DC moves
  every few frames as the exposure settles. The DIS (`dis_update_stats`,
  event 15, the second DMA writer's done) reads motion vectors at 0x95C0
  + 8 i for the 0x908 grid (8 x 4) only when the stabiliser is armed; the
  core reads zero there, no motion, which is right for a fixed camera.

T40 (`tx-isp-t40.ko` H20220606a, read off the T40 SDK 1.2.0 build with
symbols) is the T31 design again, rearranged: the core at 0x13300000 is
512 KiB with pipe 0's interrupts at 0x40020 / 0x40028 / 0x40024 (enable,
status, clear; IRQ 39 = INTC 31), the MSCA channels at 0x16100 + 0x100 n
with T31's offsets and their enable at 0x16008; VIC0 at 0x13380000 (IRQ
38) is T31's but for its frame start, bit 4 of 0x1E0, which starts the
sensor / AE work; the single sensor's CSI host is the one at 0x10054000.
Nothing is polled.

T40's second pipe takes a second sensor (`sensor1=`, I2C3, CSI1 at
0x10023000: storage, as CSI0) -- libimp's `IMPVI_SEC`, the SDK samples'
`IMPISP_TOTAL_TWO` in a `DUALSENSOR_MODE`, the driver's `tisp_sec_*`,
and a module of the sensor's own (`sensor_gc4653s1_t40.ko`, the gc4653
driver under another name, and `/etc/sensor/gc4653s1-t40.bin` for
`tisp_sec_init`). It is pipe 0 again at fixed distances: VIC1 is VIC0's
register file 64 KiB on (0x13390000; VIC2 at 0x133A0000, storage) --
the module maps only VIC0's 64 KiB but reaches the others past it, its
VIC ISR picking the window by the interrupt it came in on: INTC 30 VIC0,
29 VIC1, else VIC2 -- and VIC1's frame start queues `sec_fs_work`;
pipe 1's status is 0x40038, cleared through 0x40034, enabled at 0x40030
(the core ISR, 0x72f10, reads 0x40028 and 0x40038 in turn), with pipe
0's bits: channels 3..5 on bits 0..2 and the 3A banks on 3..5; its MSCA
channels 3..5 are at 0x17100 + 0x100 n with the T31 offsets, enable
0x17008; its AE and AWB blocks are pipe 0's plus 0x200 (0x19200,
0x18200), their banks set by `tisp_sec_init`. The model runs a pipe
while its VIC is started and its interrupt enable is not zero, lights
pipe 1 by the second sensor's exposure, and paints it with the frame
counter 500000 ahead so that its pictures tell themselves apart. The
interrupt status 0x40068 and 0x400A8 the ISR also reads stay 0.

A third sensor (`sensor2=`, I2C0: the T40 kernel registers I2C0, 1 and
3 only) is libimp's `IMPVI_THR` with `IMPISP_TOTAL_THR`: the driver
takes it on VIC2 through the DVP port (`csi_phy_start` refuses a third
MIPI sensor, "Can not support more than 2(3) mipi sensor"), prints "The
third sensor init" and gives it no tuning (only indices 0 and 1 get
`tisp_init` / `tisp_sec_init`) and no pipe: `/proc/jz/isp/isp-w02` shows
"vic index: 0, 1, 2". The model runs VIC2 once started -- frame done,
INTC 28; no frame start, for which the driver's VIC ISR has no work and
only prints -- and a three-sensor libimp client still gets both pipes'
frames. The SDK ships no driver for a third sensor; the test renames the
gc4653 "s1" module to "s2" (the two differ only in their names).

The VIC's MDMA stores raw frames in DDR. Only the `snapraw` debug
command uses it ("echo snapraw S N > /proc/jz/isp/isp-w02", saved as
`/tmp/snapK.raw`): `vic_mdma_enable` writes 0x304 = width << 16 |
height, 0x308 = 1, the strides 0x310 / 0x314 (2 bytes a pixel), five
buffers per channel at 0x318 + 4 k (channel 0) and 0x340 + 4 k (channel
1), and 0x300 = bit 31 | frames << 16 | 0x20 (at most 8; the driver tops
it up). The VIC ISR reads a second status, 0x1E4 under 0x1EC, acked at
0x1F4: bit 0 is channel 0's frame, bit 1 channel 1's; on each,
`vic_mdma_irq_function0` moves the next buffer slot on and completes the
command when its count is out. The model stores each of its frames
in the next slot as an RGGB mosaic of the scene, 10 bits in 16
(undocumented: the driver only copies the bytes), sets 0x380 to the
buffer written, counts 0x300's frames down and raises bit 0. The same
raw path behind a VIC frame channel (`ispvic_frame_channel_qbuf`, the
module's `isp-w02` raw dequeue) is not driven by anything here.

libimp's joint mode (`IMPISPDualSensorSplitJoint`: 1 main left, 2 right,
3 above, 4 under) puts both sensors in one picture on channel 0, twice
the sensor's size across or down; the secondary pipe's channel 3 is
created and enabled with the sensor's size, and its frames come through
channel 0. `ispcore_pad_event_handle` (`tx-isp-t40.ko`) queues channel
0's buffer to both pipes' scalers: one gets it as it is, the other at an
offset -- across, the channel's width less the part's (main left) or the
part's width (main right), in bytes for both planes; down, width x (the
channel's height less the part's) (main above) or width x the part's
height (main under) for luma and half that for the CbCr plane -- and the
ISR pops both pipes' done FIFOs for the one "buffer done" it sends. The
model paints each pipe at the address and size its scaler was given, so
each sensor's scene, lit by its own sensor and 3A, lands in its part.
Left and right joins of two 2560x1440 sensors are 5120 wide, past the
scaler's limit ("Image width cannot exceed 3840!"), which the driver
enforces; above and under are tested (`test-ingenic-isp.sh --sensor1
gc4653 --joint 3|4`: the main part reads pipe 0's counter, the
secondary's pipe 1's, 500000 ahead).

T41's HLDC, the ISP's lens distortion correction (`tiziano-hldc.c` of
`tx-isp-t41.ko`), runs while 0x40 bit 20 (`IMPISPModuleCtl.bitBypassHLDC`)
is clear. `tisp_hldc_write_reg` writes its registers from 16 halfwords --
the tuning's at init (1, 1 and zeros), or the tuning device `/dev/tisp`'s
(`_IO('t', 1)` with module 21, sub-module 0; the stock module builds
`IMP_ISP_LDC_*`'s LDC block out, `TX_ISP_LDC_ENABLE`, and calls no
`tisp_hldc_set_attr`):

| | |
|---|---|
| 0x15000 | p1 << 16 \| p0 (1, 1 from every tuning; stored) |
| 0x15004 / 0x15008 | k2 << 16 \| k1, luma / chroma |
| 0x1500C / 0x15010 | the zoom, luma / chroma |
| 0x15014 / 0x15018 | 0x3fffffff / r2max, luma / chroma |
| 0x1501C / 0x15020 | the centre's offset, offy << 16 \| offx, luma / chroma |
| 0x15024 / 0x15028, 0x15044 | p14 / p15 (stored); 3 |

`tisp_hldc_calc_para` gives their meaning for a plane of w x h: r2max is
the squared distance from the centre to the farthest corner, rho = r^2 /
r2max, the radial gain f(rho) = 256 - k1 rho / 4 + k2 rho^2 / 4 (Q8), the
zoom 2^19 / min(f(1), f at the nearer side) - 5 (Q11); its 1 / r2max for
2560x1440, 498, is the SDK's own `r2_rep` for that size. How the stage
applies them is inferred: an output pixel at d from the centre shows the
picture at d x f(rho) / 256 x zoom / 2048 -- a barrel correction for k1
> 0 that keeps the corners and pulls the middle in -- sampled
bilinearly, luma and chroma with their own registers, on the sensor's
picture (0x80) before the scaler, each channel mapped by its scale.
`test-ingenic-isp.sh --soc t41 --ldc K1,K2` writes the parameters
through `/dev/tisp`, saves frames, clears the bypass through libimp and
saves more: the later frames match the earlier remapped on the host from
the same registers (95 % of pixels identical, against 157 levels apart
without the remap). T40's HLDC (`tisp_hldc_con_par_cfg`) is not
modelled.

T41 (`tx-isp-t41.ko` H20230630a of the T41 SDK 1.2.0) is T40's core and
VIC with one pipe and one VIC, the CSI host at 0x10023000, and the scaler
moved to core + 0xF0000: 0x133F0000, past the VIC, so `ingenic-isp`
maps it as a window of its own (mmio 4) onto the same register file;
channels from 0xF0100 with the T31 offsets, enable 0xF0008. OpenIPC has
no T41 release, so its test boots the SDK's own T41NQ 16 MiB flash image
(`--image`), kernel through `-kernel` and a rebuilt root as a fifth
partition, with the vendor modules from the image's JFFS2 and the client
linked statically against the SDK's `libimp.a`. The T41 `start_param`
loads the ISP with `direct_mode=1` and the application with `--dm 1`:
IVDC, the ISP-to-encoder path ("T41 IVDC Module Application Guide";
its driver is part of `tx-isp-t41.ko`). In that mode tx-isp queues
channel 0 the pair 0x80000000 / 0x81000000 instead of DRAM buffers
(pitch 4096), the scaler writes the picture into the IVDC's 288 KiB
store (spilling to DRAM lines it configures at IVDC +0x34 / +0x38), and
the encoder is handed the same pair as its source. The ISR sends the
frame source its "buffer done" on int_sta bit 20, the MSCA's first line
(bit 13, the vertical blank, with `ivdc_vb_type` 1), and on bit 0 queues
the pair again instead of reading the done FIFO. Here 32 MiB of RAM at
0x80000000 stands in for the store: the scaler paints a frame queued
the pair into it and raises bit 20 without touching the done FIFO, and
the AVPU reads 0x80000000-0x81FFFFFF unmasked. The IVDC's own registers
(`ingenic-ivdc`, 0x13200000) hold the sizes and DRAM spill lines the
driver writes; 0x00 bit 0 is a reset that reads back 0 at once (the
driver spins on it); the status at +0x44, cleared through +0x54, gets
ISP frame start and frame done (bits 17 and 16) for each picture the
scaler writes into the store, and interrupts on core INTC source 13
(tx-isp's `isp-ivdc`; the device tree's node names 0, `jpeg0`'s line)
under the enable mask at +0x40 (0x244FC0FF from `ivdc_core_ops_init`).
The driver counts them in `/proc/jz/isp/isp-ivdc`, and at frame done
moves its state to running. `test-ingenic-sdk-boot.sh` keeps the
image's `direct_mode` (`--no-direct-mode` strips it): the stream
decodes with the scene, and the IVDC counters read 102 ISP frame starts
and dones and no errors by the time of the check. The core's 0x204 bit 0 follows 0x200
bit 0: `ispcore_video_s_stream` and `ispcore_suspend_module` set the one
and spin on the other without a timeout.

T30 couples the same Apical core with a T31-style MSCA scaler of its own
(0x130B0000, IRQ 27), which `ingenic-isp`'s "t30" layout models, and
three CPM reset handshakes in 0xAC. With the vendor's own module it
streams; OpenIPC's `tx-isp-t30.ko` cannot work on any hardware as built.

Both modules say H20190725b, but they are linked differently. The SDK's
Makefile (opensource/drivers/tx-isp-t30) does not compile
`apical-isp/apical_isp_io.c`: the Apical accessors come from the
`apical_isp_io.c.o` member of the prebuilt `apical-isp/libapical_isp.a`,
where `APICAL_READ_32(addr)` sends `addr < 0x1FFFF` to
`system_isp_read_32()` (and 0x50100-0x501F7 to an array in RAM,
`apical_ext_sytem_mem`), and `system_isp_*` add the base that the
core's probe hands to `system_isp_set_base_address()` -- one variable
(.bss+0x1B98 in the SDK's prebuilt module, identical to the one in
the image's /system). OpenIPC's module (GCC 13.3, Buildroot) compiled
the source `apical_isp_io.c` instead, whose definitions win over the
archive member: its `APICAL_READ_32` (at .text+0xF60) adds the offset to
`apical_io_base` (.bss+0x490), which only `apical_iobase_init()` sets,
and nothing calls that -- no call, no relocation, no export; the base
from the probe goes into .bss+0x35F8, which no accessor reads. So the
first Apical write, at libimp's first call, oopses:

    Unable to handle kernel paging request at virtual address 00014000
    epc : APICAL_WRITE_32+0xc/0x18 [tx_isp_t30]

A model cannot change that (the pointer is NULL). Replacing only the
module with the SDK's prebuilt one, on OpenIPC's kernel, libimp and
`sensor_ov2735b_t30.ko`, streams: `test-ingenic-isp.sh --soc t30
--isp-ko <SDK>/opensource/drivers/tx-isp-t30/tx-isp-t30.ko`. The fix
upstream is to drop `apical_isp_io.o` from the module's object list
(and link `libapical_isp.a`), or to call `apical_iobase_init(base)`
where the probe calls `system_isp_set_base_address(base)` -- the latter
still lacks the archive's 0x501xx RAM window.

The IPU (0x13080000, INTC source 6 = Linux IRQ 14 on T10, T20, T21,
T23, T30, T31; the T40/T41 device trees the same) is the block libimp
blends its OSD regions into the frames with, through the kernel's
`jz_ipu_v13` (`ingenic_ipu_v13` on 4.4; one register file on all):
`IOCTL_IPU_START` resets the unit (TRIGGER bit 3), programs the NV12
background (IN_FM_GS, Y/U_ADDR, strides, OSD_CH_BK_PARA) and the
OSD channels it uses, sets GLB_CTRL's IRQ_EN, runs it (TRIGGER bit 0)
and waits two seconds for STATUS bit 0. `ingenic-ipu` blends in place:
each channel enabled in bit 0 of its PARA (0x11C + 4 n), its picture
ARGB8888 (byte order [17:14]), ARGB1555 ([13:11] = 4; what T30's
libimp uses for its timestamp), NV12 or NV21, at POS (x << 16 | y),
GS (width in bytes << 16 | height), alpha per pixel, global ([10:3]) or
both ([2:1]), RGB through the channel's CSC matrix at 0x148 + 0x18 n;
OUT_END and the interrupt follow a virtual 100 us later. The conversion
mode is there too: NV12 to NV21, to BGRA, or to HSV (D_FMT 0x4, what
libimp's frame source gives a `PIX_FMT_HSV` channel): H in degrees, 16
bits a pixel, at NV_OUT_ADDR; S and V a byte each at OUT_ADDR and
OUT_V_ADDR (the planes as the kernel driver and the T23 SDK's EXTFS
sample lay them out; the value scales are the usual ones, unconfirmed).
`-global ingenic-ipu.debug=on` logs each operation.

`qemu-boot/test-ingenic-t30-rtsp.sh [IMAGE]` boots the T30 SDK 1.0.5's
full image (T30X, 8 MiB) as it ships: its kernel, module and
`sample-RTSPServer` (LIVE555), which encodes H.265 1920x1080 and
640x360 with the timestamp OSD. The image sets eth0 to 193.169.4.151,
so the user network takes that subnet and forwards 8554; the host pulls
rtsp://127.0.0.1:<port>/main and /second with `ffmpeg -rtsp_transport
tcp -err_detect explode`, and checks the decoded pictures' scene, the
counter and dot advancing, and on /main the timestamp box brighter
than the background beside it and changing (the H.265 model codes 8x8
DC blocks, so the text is legible only as blocks).

OpenIPC's `load_ingenic` takes the sensor from the U-Boot environment if
it is set and otherwise from sinfo, but the release `sinfo.ko` prints
`sensor: gc2053` (a space after the colon) and the script's
`${SENSOR_INFO#*:}` then yields " gc2053": the first boot fails at
`modprobe sensor_` (the second works, from the environment the first one
saved). Preset `sensor=<name>` in the environment, as
`qemu-boot/test-ingenic-isp.sh` does.

`qemu-boot/test-ingenic-isp.sh --soc t31|t21|t23|t20|t10|t40|t41` boots the release kernel
with the rootfs on a NOR image (a RAM disk takes the memory a 1080p
capture needs at `mem=40M rmem=24M`), lets `load_ingenic` load the
vendor modules, and runs `qemu-boot/ingenic-isp-snap.c`, an IMP
FrameSource client built from the SDK samples: channel 0 at the sensor's
1920x1080 and channel 1 at 640x360, NV12, ten frames each. It writes four
frames to an SD card image, and `qemu-boot/ingenic-isp-check.py` reads
their frame counters, finds the logo and the dot, and checks that both
channels advance (`--png` keeps pictures). The client is linked against
the rootfs's own `libimp.so` (IMP 1.1.6 on T31, 1.1.0 on T23, 1.0.33 on
T21, 1.2.1 on T20; T40's, with a 64-bit FPU and NaN2008, needs the GCC
7.2 Ingenic toolchain and its own IMP API, `IMPVI_NUM` and all), a uClibc build that OpenIPC runs on musl: like majestic, it
exports the few uClibc-only symbols the library calls (and mmap /
ftruncate, whose `off_t` differs, and the older libalog's ctype tables),
and it is compiled against the SoC's SDK headers where the structs
differ (T23's `IMPSensorInfo`). IMP 1.1.6's `IMPFSChnAttr` ends with a
front crop the 1.1.5 headers lack -- left out, the kernel crops channel
0 to whatever follows in memory. The test keeps libimp's MXU2 paths off
with `/tmp/closesimd`, as the first runs did before the CPU decoded
MXU2. On T23 the MSC driver is one of the modules `load_ingenic` loads,
so the test waits for the card. T40's SDHCI finds no card, so there the
client copies the frames into rmem through `/dev/mem` and the host reads
them with the monitor's `pmemsave`; T40's IMP also hangs in its
teardown, so the verdict comes first.

## Tested

| Image | Machine | Path | Result |
|---|---|---|---|
| OpenIPC `t10`, `t20`, `t21`, `t23`, `t30`, `t31`, `t40` | own | `-kernel` + NOR root; flash (release SPL + U-Boot) | login, DHCP lease, U-Boot environment and JFFS2 overlay written back (`test-ingenic-boot.sh`) |
| T41LQ / NQ / XQ SDK 1.2.0 full images (8 and 16 MiB) | `t41,variant=t41lq/nq/x`, 64 / 128 / 256 MiB | flash | SPL (DDR test in cache-as-RAM), U-Boot, kernel on two cores, `Ingenic-uc1_1 login:`, telnet over the GMAC, the vendor `carrier-server` with `sensor=gc4663` and its RTSP H.265 stream pulled and decoded, `reboot` through the watchdog back to login (`test-ingenic-sdk-boot.sh`) |
| T40N / T40XP SDK 1.2.0 full images (glibc and uClibc, 8 and 16 MiB), T30X SDK 1.0.5 | `t40,variant=t40n/t40xp` (`sensor=gc2053`), `t30,variant=t30x` | flash | the same boot, telnet and reboot cycle; on T40 `carrier-server` streams 1080p H.265 over RTSP through the IPU and AVPU; 20 pulled frames show the ISP scene (`test-ingenic-sdk-boot.sh`) |
| T10 camera (APK-Link) | `t10` | flash | vendor U-Boot, kernel, login, the camera application |
| T20 camera (Jabsco) | `t20` | flash | login |
| T21 cameras (Jabsco, CamHi) | `t21` | flash | login, the application (whose watchdog then reboots it) |
| T31 cameras, Wyze Cam v3 | `t31` | flash | login, the ISP driver probes, DHCP lease |
| OpenIPC `t31` + vendor modules, `sensor=gc2053` | `t31` | `-kernel` + NOR root | 1920x1080 and 640x360 NV12 frames through libimp's FrameSource (`test-ingenic-isp.sh`) |
| OpenIPC `t21` + vendor modules, `sensor=jxf37` | `t21` | `-kernel` + NOR root | the same, DVP (`test-ingenic-isp.sh --soc t21`) |
| OpenIPC `t23` + vendor modules, `sensor=gc2053` | `t23` | `-kernel` + NOR root | the same (`test-ingenic-isp.sh --soc t23`) |
| OpenIPC `t20` + vendor modules, `sensor=jxf22` | `t20` | `-kernel` + NOR root | the same, through the Apical core's DMA writers (`test-ingenic-isp.sh --soc t20`) |
| OpenIPC `t40` + vendor modules, `sensor=gc4653` | `t40 -m 128M` | `-kernel` + NOR root | 2560x1440 and 640x360 (`test-ingenic-isp.sh --soc t40`) |
| OpenIPC `t31` / `t40` + vendor modules, `sensor=gc4653`; T41NQ SDK image, `gc4663`; the sensors' DOL WDR | `t31 -m 128M` / `t40` / `t41` | as above | frames in the WDR mode, the short exposure written (T40, T41: moving), under 2 % of the last frame clipped (`test-ingenic-isp.sh --wdr`) |
| OpenIPC `t40` + vendor modules + the SDK's `sensor_gc4653s1_t40.ko`, two sensors | `t40,sensor=gc4653,sensor1=gc4653 -m 256M` | `-kernel` + NOR root | libimp in the all-cached dual mode: channel 0 (pipe 0) and channel 3 (pipe 1) at 2560x1440, 40 frames each, both pipes' AE converging on their own sensors, the scene and counter checked per pipe (`test-ingenic-isp.sh --soc t40 --sensor1 gc4653`) |
| OpenIPC `t40` + vendor modules + the gc4653 "s1" module twice (as `s1` and `s2`), three sensors | `t40,sensor=gc4653,sensor1=gc4653,sensor2=gc4653 -m 256M` | `-kernel` + NOR root | libimp with `IMPISP_TOTAL_THR`, the third on VIC2 / DVP: both pipes' frames as above, VIC2 interrupting; the `snapraw` command while streaming saves `/tmp/snap0.raw` (2560x1440 x 2 bytes, an RGGB mosaic of the navy scene: R 186, G 216, B 335 on average) through the VIC MDMA (`test-ingenic-isp.sh --soc t40 --sensor1 gc4653 --sensor2 gc4653 --snap-env SNAP_SNAPRAW=0_1`) |
| T41NQ SDK 1.2.0 image + its modules, `sensor=gc4663` | `t41,variant=t41nq -m 128M` | `-kernel` + NOR root | 2560x1440 and 640x360 (`test-ingenic-isp.sh --soc t41`) |
| the same, HLDC parameters through `/dev/tisp`, the bypass cleared through libimp | `t41,variant=t41nq` | as above | the corrected frames match the uncorrected ones remapped on the host from the registers (`test-ingenic-isp.sh --soc t41 --ldc 200,40`) |
| OpenIPC `t40` + vendor modules, two gc4653, libimp's joint mode | `t40,sensor=gc4653,sensor1=gc4653` | as above | 2560x2880 on channel 0, the main sensor's part with pipe 0's counter and the secondary's with pipe 1's, above and under (`test-ingenic-isp.sh --soc t40 --sensor1 gc4653 --joint 3`, `4`) |
| OpenIPC `t10` + vendor modules, `sensor=jxh62` | `t10` | `-kernel` + NOR root | 1280x720 and 640x360, the sensor on bit-banged I2C (`test-ingenic-isp.sh --soc t10`) |
| T40XP camera | `t40 -m 256M` | flash | login, the vendor application starts |
| U-Boot `isvp_t31_sfcnand` (+ patch), 3.10 with `MTD_JZ_SFC_NAND`, UBI, OpenIPC t31 rootfs | `t31` | SPI NAND (`w25n01gv`) | SPL, U-Boot saves its env, UBI, SquashFS root on ubiblock, UBIFS overlay, login; a file written persists (`test-ingenic-nand.sh`) |
| Vendor U-Boot `isvp_t41n_sfc0_nand`, SDK 4.4 `isvp_tomcat_nand_defconfig` (+ UBI_BLOCK, cmdline from U-Boot), OpenIPC t40 rootfs as UBIFS | `t41` | SPI NAND (`gd5f1gq4ub` with OOB) | SPL, U-Boot, the 4.4 driver by command table + descriptor DMA, UBIFS root, login; a file written persists (`test-ingenic-nand.sh --soc t41`, `maxcpus=1`) |
| OpenIPC `t40` / `t31` + their libimp.so, `sensor=gc4653` / `gc2053` | `t40` / `t31` | NOR root, `test-ingenic-avpu.sh` | 2560x1440 (T40) or 1920x1080 (T31) + 640x360: 10 H.264 and 10 H.265 pictures and a JPEG per channel, all decoded with `-err_detect explode` and checked against the ISP's scene |
| T41NQ SDK image + SDK 1.2.0 `libimp.a`, `sensor=gc4663` | `t41,variant=t41nq` | `-kernel` + NOR root, `test-ingenic-avpu.sh --soc t41` | the same at 2560x1440 + 640x360, through the gen 2 AVPU |
| OpenIPC `t31` + SDK 1.1.5 `sample-Encoder-video` / `-jpeg` (static, `libimp.a`), `sensor=gc2053` | `t31` | `-kernel`, `init=/bin/sh`, vendor modules by hand | 20 frames (150 for main profile) of 1920x1080 + 640x360 H.264 (baseline, main, high) and H.265, 1080p and 360p JPEG snapshots; all decode with ffmpeg without errors |
| OpenIPC `t30` + the T30 SDK's `tx-isp-t30.ko`, `sensor=ov2735b` | `t30` | `-kernel` + NOR root | 1920x1080 and 640x360 (`test-ingenic-isp.sh --soc t30 --isp-ko ...`) |
| T30X SDK 1.0.5 full image (8 MiB), `sensor=jxf22` | `t30,variant=t30x -m 128M` | flash, user net | vendor kernel and modules, sample-RTSPServer: /main 1920x1080 and /second 640x360 H.265 over RTSP/TCP, 200 pictures each decoded without errors, the scene and the IPU's timestamp OSD (`test-ingenic-t30-rtsp.sh`) |
| OpenIPC `t20`, `t21`, `t23`, `t30` libimp | own | `-kernel`, `test-ingenic-vpu-feed.py` | libimp's H.264 (fixed QP; CBR on T21/T23/T30) and JPEG encoders, T30's H.265: streams ffmpeg decodes with `-err_detect explode`, PSNR against the source picture |
| OpenIPC `t10`, `t20`, `t21`, `t23`, `t30`, `t31`, `t40` (their `audio.ko`) / T41NQ SDK image (its `audio.ko`) + SDK `libimp.a` | own / `t41,variant=t41nq`, `audiodev=` wav | `-kernel` + NOR root, `test-ingenic-audio.sh --soc <soc>` | IMP_AI records 16 kHz mono: the 1 kHz tone on all eight (FFT peak 1000.0 Hz; rms 5793, 11585 on T10, T20, T30); IMP_AO plays a 440 Hz sine: 2.00 s of it in the wav (peak 440.0 Hz) on all eight, T21 included (6 runs alone; an earlier 1.08 s on T21 did not come back -- the driver's flush (`dsp_ioctl_flush_ao_stream`) zeroes whatever is still queued, so a guest that falls behind on a loaded host plays less); `--rate 8000 --play-rate 16000` (two clocks): 440.0 Hz on T10, T23, T31, T40 |
| OpenIPC `t40` / T41NQ SDK image, `--gains AI:AO` (IMP_AI_SetGain, IMP_AO_SetGain) | `t40` / `t41,variant=t41nq` | `test-ingenic-audio.sh --soc t40 --gains 12:20`, `--soc t41 --gains 27:28` | the codec's analogue gains: T40 records the tone at -6.0 dB (rms 2903 of 5793) and plays at -6.0 dB (4242 of 8464); T41 records at -6.0 dB (2903) and plays at +6.0 dB (16889 of 8464) |
| OpenIPC `t40`, `audio.ko dmic_enable=1` + T40 SDK `libimp.a` | `t40` | `test-ingenic-audio.sh --soc t40 --dmic` | IMP_DMIC records 16 kHz: the DMIC's 1.5 kHz tone (peak 1500.0 Hz, rms 5793); with QEMU's TLB shadow entries on, 5 of 12 runs failed (see CPUs), without them 24 consecutive runs pass |
| T41NQ SDK image, `audio.ko dmic_enable=1` + T41 SDK `libimp.a` | `t41,variant=t41nq` | `test-ingenic-audio.sh --soc t41 --dmic` | IMP_DMIC records 16 kHz through T41's own audio system DMA: the 1.5 kHz tone (peak 1500.0 Hz, rms 5793) |
| T40N / T41NQ SDK images, their kernels' `ingenic_drawbox` and `ingenic_i2d` | `t40` / `t41` | flash, `test-ingenic-xb2blocks.sh --image` | a client draws a rectangle and a line through `/dev/dbox` and rotates a 320x192 NV12 picture 90 degrees through `/dev/i2d`: every pixel checked in the guest |
| T41NQ SDK image, its kernel's `ingenic_aip` + magik-toolkit `libaip.a` | `t41` | flash, `test-ingenic-aip.sh --image` | the library's own bs_* calls: NV12 640x360 -> 320x180 within 0.8 of the ramps sampled at half-pixel positions, a two-box chain, NV12 -> BGRA / RGBA / ARGB matching BT.601 exactly, affine and perspective warps moving a square where the forward matrix sends it with the pad value outside |
| T40N SDK image, its kernel's `ingenic_bscaler` + magik-toolkit nna1 `libvenus.a` (its BSCALER objects) | `t40` | flash, `test-ingenic-bscaler.sh --image` | the library's own bs_* calls: BGRA 64x32 -> 32x16 and a many-tile 32x16 -> 160x80 within 0.7 of the ramps at half-pixel positions, NV12 256x128 -> 128x64 (both planes) within 0.7, NV12 -> BGRA / RGBA / ARGB resizes and the T unit's NV12 -> BGRA / RGBA matching BT.601 exactly, affine and projective perspective warps moving a square where the forward matrix sends it with the pad value outside |
| OpenIPC `t40` + its `soc-nna.ko`, a client with magik-toolkit nna1 `libvenus.a`'s `dma_func` | `t40` | `-kernel` + NOR root, `test-ingenic-nna.sh` | the version ioctl; libvenus's `w_d2o_orasetrun` moves two blocks DDR -> ORAM (waited with `rdhwr $9`), write channel 0 moves them back (`rdhwr $11`), byte for byte; `rdhwr $4` after an `ll` gives its physical address |
| T41NQ SDK image + SDK `libimp.a`, `--osd` | `t41,sensor=gc4663` | `test-ingenic-avpu.sh --soc t41 --osd` | an IMP_OSD rectangle and line on channel 0, drawn by the DRAWBOX (its interrupt counts) and found in the decoded H.264 at their places and colours |
| T41NQ SDK image + SDK `libimp.a`, `--ivpu` | `t41,sensor=gc4663` | `test-ingenic-avpu.sh --soc t41 --codecs jpeg --ivpu` | JPEG channels with `IMP_ENC_TPYE_IVPU` on the HELIX JPEG engine through `/dev/soc_vpu`: 2560x1440 and 640x360 pictures decode cleanly and show the scene |
| `t41` / `t40` qtest | `t41`, `t40` | `test-ingenic.py` `test_dpu` | the DPU composes an RGB565 background, a half-transparent ARGB8888 square and an NV12 layer scaled 2x onto the display (screendump pixels), raises CMP_START / layer ends / CMP_END / DISP_END on IRQ 20, stops on the descriptor's stop bit, loops a chain and ends it with a general stop |
| `t31` / `t41` qtest | `t31`, `t41`, `t31,sensor=gc2053` | `test-ingenic.py` `test_hash_cpu`, `test_i2c_dma`, `test_sadc_dma` | the HASH engine fed by HSDI gives hashlib's MD5, SHA-1/224/256/384/512 for one- and multi-block messages, with round and all-rounds done; the GC2053's ID read and a register write by PDMA through I2C0 (requests 0x24 / 0x25), the RX request waiting for DMARDLR + 1 bytes; the SADC's continuous scans landing in a PDMA ring as AUX1:AUX0 / AUX3:AUX2 words (request 9) -- CPU mode and the SADC's DMA inferred, see above |
| OpenIPC `t31` + the T31 SDK's `sample_aes`, `sample_hash`, `sample_rsa`, `sample_dtrng`, `sample_des` (built against the same kernel) | `t31` | `-kernel` + NOR root, `test-ingenic-crypto.sh` | AES-128 ECB and CBC (FIPS-197, SP 800-38A) and back; MD5, SHA-1/224/256/384/512 equal hashlib's; RSA-1024/2048 random vectors and the sample's own `test_rsa.h` decryptions; DTRNG words by polling and by interrupt; DES ECB of FIPS 46's example by the CPU (85e813540f0ab405), the same as triple DES with equal keys by the PDMA, and a 4 KiB triple-DES CBC round trip |
| OpenIPC T31 kernel with `CONFIG_SERIAL_JZ47XX_UART0_DMA` + the release's root | `t31`, UART0 on a socket | `-kernel` + NOR root, `test-ingenic-uart-dma.sh --kernel` | 32 KiB of random bytes in on ttyS0 by the PDMA (32 full 1 KiB buffers, 3 of 3 runs) and back out by it, both intact |
| OpenIPC `t21` + `ingenic-isp`, jxf37 | `t21,sensor=jxf37` | NOR root, `test-ingenic-vpu-e2e.sh` | vendor modules, FrameSource -> IMP_Encoder: H.264 CBR and a JPEG snapshot, decoded on the host |

`python3 qemu/tests/test-ingenic.py` checks the models at register
level under qtest (no guest), including a migration round trip of `t31`,
`t41` and a three-sensor `t40`, and the image pipeline: the GC2053's ID on I2C0, the VIC
poll, the MSCA FIFOs and a frame painted into a queued buffer under
`clock_step`, on T31 and through the T21, T23, T40 and T41 layouts,
T40's second pipe on its own and beside the first (VIC1 on INTC 29,
channel 3, its AE banks, its counter 500000 ahead), its VIC MDMA (two
raw frames into buffers 0 and 1, 0x380, the count, status2 bit 0, the
RGGB mosaic) and VIC2 (frame done on INTC 28), and
on T20 and T10 the Apical slots, a DMA writer's bank and a dropped frame;
and sound: a PDMA memory copy with its interrupt, a cyclic record ring
filled with the tone at 16 kHz and a play ring drained at that rate,
the DMIC's packed samples on request 13, its FIFO filling without
DMA and its voice trigger (modes 0 and 2, rearm, silence), the AIC playing at the CPM's 48 kHz speaker clock over the codec's
16 kHz, and the T31 codec's record gain and switches; the SSI (READ ID from a
`w25q80` by PIO, the underrun interrupt, 300 frames looped back through
the PDMA), the UART's DMA mode (two 32-byte bursts by the PDMA, an 8-byte
tail for the receive timeout with IER bit 4 alone, PIO reads of it),
AES / HASH / RSA / DTRNG against FIPS-197, SP 800-38A, hashlib and pow(),
and the TCU's PWM pins.

## Not modelled

The SDIO Wi-Fi chip (`atbm-sdio`) enumerates and the vendor driver probes
it and starts the firmware download, but a full `wlan0` association is not
reached: it needs the MSC's CMD53 data phase modelled as the asynchronous
multi-interrupt sequence the `jzmmc_v12` state machine expects (above).


T30's ISP streaming with OpenIPC's own `tx-isp-t30.ko` (it never sets
its Apical base; above), the left / right joins of T40's joint mode
(past the scaler's limit with these sensors) and the VIC raw frame
channel (above), the 160 values of
T40/T41's WDR statistics at +1368, a WDR mode on T23 (no modelled sensor
has one there), T40's GSM and LCE (read otherwise
than T41's), the Apical DIS's motion vectors (zero: no motion), the
SSI slave (no driver, sample or register map in any tree here; above),
the trigger of T41's audio-system DMIC and the DMIC trigger modes other
than 0 and 2, the DPU's write-back, TLB and simple-read (RDMA) channel,
T40's HLDC, T41's LDC block (below)
and the blocks below read as zero or as storage.

What the remaining video and compute blocks would need (survey of the
T40/T41 SDK kernels, drivers and libimp):

- T41's LDC block (0x13040000, `ingenic,t41-ldc`, core INTC 35):
  `tx-isp-t41.ko` (H20230630a) registers it as a platform device
  (`tx-isp-ldc.c`: resource, clocks `div_ldc` / `gate_ldc`, pads) but no
  code programs it; `IMP_ISP_LDC_INIT` / `IMP_ISP_LDC_SetAttr` and their
  `IMPISPLDCParams` (`LDC_ADDR_RES_SIZE`, `X_CORF`, `BROCF`, `R2_REP`,
  `LEN_COEF`, shift tables, per the SDK's `ldclut.h`) are for an
  "LDC-Beta" ISP build (`sample-Encoder-ldc-video.c`) that no SDK here
  ships, so the block has no driver and no register offsets. The ISP's
  own HLDC (above) is modelled.

- T30's NCU (0x13090000): the denoise front end `jz_nvpu` programs with
  the VPU when libimp asks for it; the VPU raises the completion, so
  storage passes the picture through un-denoised.
- The NNA's compute of T40/T41: libvenus runs a network on the CPU, with
  MXU3 vector instructions and Ingenic's own coprocessor-2 instructions
  that feed the multiply-accumulate array from ORAM. The NNA DMA, the ORAM
  and the hardware registers are modelled (above); those instructions
  are not: the toolchain's libopcodes knows none of them and nothing
  public here documents them -- 57 opcode classes (by the COP2 rs field
  and function), 121881 instructions in the T40 build of libvenus and
  48450 in the T41 one. Until they are, a network's first NNA instruction
  raises Reserved Instruction (SIGILL).
- The RISC-V core of T40/T41 (the "LEP"; T41 CCU at 0x12A00000): no
  driver, firmware or library here starts it; it would need a RISC-V CPU
  beside the MIPS ones and firmware these SDKs do not ship.
