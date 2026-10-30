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
| 0x10030000 + 0x1000 n | UART0..3, 16550 with a 4-byte stride | QEMU `serial-mm` |
| 0x10050000 + 0x1000 n | I2C0..3 (DW_apb_i2c) | `ingenic-i2c` |
| 0x12000000 | system OST → IP4 (XBurst1); global OST (XBurst2) | `ingenic-ost` |
| 0x12100000 + 0x100 n | core OST → IP4 (XBurst2) | `ingenic-ost` layout 2 |
| 0x12200000 | CCU: core reset/entry, mailboxes → IP3 (XBurst2) | `ingenic-ccu` |
| 0x12300000 + 0x100 n | core INTC → IP2 (XBurst2) | `ingenic-ccu` |
| 0x13010000 | DDR PHY (+0x1000) and DDRC APB (+0x2000) | `ingenic-ddr` |
| 0x134F0000 | DDRC | `ingenic-ddr` |
| 0x13440000 | SFC + SPI NOR or SPI NAND | `ingenic-sfc` + m25p80 / `ingenic-spinand` |
| 0x13450000 | T41: SFC1, IRQ 19 (`-drive if=mtd,index=1`) | `ingenic-sfc` |
| 0x13450000 / 0x13460000 | MSC0/1 (XBurst1) | `ingenic-msc` |
| 0x13060000 / 0x13070000 | MSC0/1 (XBurst2): SDHCI 3.0 | QEMU `generic-sdhci` |
| 0x134B0000 | Synopsys GMAC, IP101G PHY | `ingenic-gmac` |
| 0x13500000 | DWC2 USB OTG, host | QEMU `dwc2-usb` |
| 0x13540000 | EFUSE: blank but for the grade code | `ingenic-efuse` |
| 0x13300000 | ISP core (T31: to 0x1338FFFF; T21, T23: 64 KiB; T40, T41: 512 KiB, T41's scaler at 0x133F0000; T10/T20/T30's Apical core at 0x13380000) | `ingenic-isp`, `ingenic-isp-apical` |
| 0x133E0000 | VIC (T21: 0x13310000) | `ingenic-isp` |
| 0x10023000 / 0x10022000 | MIPI CSI-2 host / D-PHY (T23, T31) | `ingenic-isp` |
| 0x13000000 | AHB0 arbiter, the SoC ID at +0x2C | `ingenic-harb` |
| 0x13200000 | video encoder: VPU (T10/T20), HELIX (T21/T30; T23: 0x13100000), IRQ 70 (T23: 48) | `ingenic-vpu` |
| 0x13600000 | RADIX, T30's H.265 encoder, IRQ 53 | `ingenic-vpu` kind 1 |
| 0x13080000 | IPU, libimp's OSD blender (every part), IRQ 14 | `ingenic-ipu` |
| 0x10070000 | SADC: 8 AUX channels, 12 bits, IRQ 18 (`/machine/sadc`) | `ingenic-sadc` |
| 0x10020000 / 0x10021000 | AIC (I2S, IRQ 1) / internal audio codec (`/machine/aic`) | `ingenic-aic` |
| 0x13420000 | PDMA: 32 channels, descriptors, IRQ 10 | `ingenic-pdma` |
| 0x13460000 | T41: PWM controller, 8 channels (`/machine/pwm`) | `ingenic-pwm` |
| 0x13200000 | AVPU video encoder, registers from +0x8000 (T31, T40; T41: 0x13100000 from +0), IRQ 70 | `ingenic-avpu` |
| 0x12660000 | XBurst2 on-chip RAM (64 KiB) | RAM |
| 0x1F000000 | cache-as-RAM window (32 KiB) | RAM |
| 0x1FC00000 | boot ROM (the reset vector, 0xBFC00000) | ROM |

Everything else in 0x10000000-0x13FFFFFF reads as zero (`ingenic.io`, an
unimplemented device, logged with `-d unimp`). Interrupt numbers are the
same on every part: AIC 1, SFC 7, PDMA 10, GPIO A 17 (B 16, C 15, D 14), OTG 21,
SADC 18, TCU 27/26/25, MSC0/1 37/36, UART0..3 51..48, GMAC 55, I2C0..3 60..57,
ISP 29, VIC 30, AVPU 62 (Linux numbers are these + 8: the ISP's 37 and
38, the AVPU's 70).

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
RAM at 0x12660000 and runs core 1 on it through the CCU, so that RAM is
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
runs `app_init.sh` at the test's address (and, with a sensor, without
IVDC's `direct_mode`), puts QEMU's user network on that /24 with host
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
flag without interrupts.

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

`qemu/tests/test-ingenic-vpu.py` plays the driver and libimp at register
level on every SoC (H.264, JPEG, H.265 on T30) and decodes the result
with ffmpeg; `qemu/tests/test-ingenic-vpu-feed.py` runs libimp's own
encoders (`i264e_*`, `i265e_*`, `ijpege_*`; `qemu/tests/ingenic-vpu-feed.c`)
from a picture in memory on the OpenIPC kernel and rootfs of each SoC; `qemu/tests/test-ingenic-vpu-e2e.sh` runs the
SDK samples' FrameSource -> `IMP_Encoder` path on T21 with the ISP model
and its sensor.

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
  I2C0.
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
- `ingenic-pwm`: T41's PWM block (`pwm-ingenic.c`, 4.4): ENS/ENC over EN,
  WCFG taken on UPDATE, BUSYR clear, LVL and OEN; DMA mode is storage.
  `qom-get /machine/pwm chan2` reads a pin as `high/period` cycles or a
  fixed `0`/`1`. The other parts' PWM is the TCU's channels.

### Sound: `ingenic-aic` and `ingenic-pdma`

The vendor `audio.ko` (SDK `drivers/audio/oss2` on T10..T31,
`oss3` on T23, T40 and T41; `/dev/dsp`, which libimp's IMP_AI and IMP_AO
drive) plays and records through the AIC, its FIFOs moved by the PDMA in
cyclic transfers.

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
  7 (record) and 5 (loop record, the echo canceller's reference).
- `ingenic-aic`: AICFR (ENB, RST), AICCR (EREC/ERPL, the flushes, M2S,
  MONOCTR, TDMS/RDMS, ISS/OSS, TLDMS/ETFL, CHANNEL, PACK16), AICSR (levels,
  TUR/ROR), AICDR and AICLR over the hardware's FIFOs (64 to play, 32 to
  record and loop), and the codec at 0x10021000: bytes, one per word, read
  back as written; its CSRR (0x110 bits 2:0) is the sample rate, 8 kHz
  to 96 kHz. While ENB is set, frames move at that rate in virtual time:
  played samples (one per frame with M2S, CHANNEL + 1 otherwise) leave the
  TX FIFO for the machine's audiodev as 16-bit stereo; the RX FIFO gets
  one sample per frame with MONOCTR (T21, T31: the right channel) or two:
  a 1 kHz sine at a quarter of full scale (`-global
  ingenic-aic.tone-hz=N`, 0 for silence) or, with
  `-global ingenic-aic.rx-source=audiodev`, the host's input; the loop
  FIFO gets what is played. Requests: TX while TDMS and the TX FIFO has
  room, RX while RDMS and it holds a sample, loop while TLDMS and it does.

`-audiodev wav,id=snd,path=out.wav -M t31,audiodev=snd` records what the
guest plays. DMIC (0x134D0000) is not modelled: the OpenIPC T10..T31
builds of `audio.ko` have only its stubs, T40's drives one only with
`dmic_enable=1`, and T41's device tree disables it.

### The image pipeline: `sensor=` and `ingenic-isp`

`-M <soc>,sensor=<name>` puts an image sensor on I2C0 (I2C1 on T40) at
the address and with the ID that OpenIPC's `sinfo.ko` table probes:
`gc2053` (0x37, `galaxycore-sensor`), `gc2083` (0x37, 16-bit registers),
`sc2336` and `sc2335` (0x30, `hisi-smartsens`), `jxf37` and `jxf22`
(0x40, `hisi-f37`, `hisi-jxf22`), `ov2735b` and `ov2735` (0x3C,
`ingenic-ov2735`: 8-bit registers in pages, the revision in page 0's
0x04 telling the two apart), `gc4653` (0x29, 16-bit registers; T40, on
I2C1), `gc4663` (T41, the same ID), `jxh62` (0x30, `hisi-jxh62`; T10).
The default is none. T10's kernel bit-bangs the sensor bus (`i2c-gpio`,
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
| AWB zone | 16 bytes: R, G, B, IR sums (21 bits), count (13) | 128 bytes: 8 records of R, G, B, IR (22), count (14), all filled alike |

In both blocks +0x04 is the zone grid (columns [31:28], rows [15:12]:
15 x 15), +0x08-0x14 and +0x18-0x24 the zone widths and heights a byte
each in 2x2-pixel quads, +0x4C the bank counts and +0x50 the bank last
written (AE: zones [5:4], histogram [1:0]); AE +0x28 holds the dark and
bright thresholds ([7:0], [15:8]), +0x2C-0x38 four zone banks and
+0x3C-0x48 four histogram banks (256 21-bit counts of the quads by luma,
twice); AWB +0x3C-0x48 four zone banks. The kernel's `ae0_weight_mean2`
takes a zone's three sums over its width x height as its mean. Zones are
sampled 8 x 6 from the scene, in linear light (8 bits), as a sensor's
statistics are; the sensor's raw colour makes a white
read R `light-r` and B `light-b` per mille of G (550, 600: daylight).
`light`, `light-r` and `light-b` can change while running
(`qom-set /machine/isp light 30`). The kernel's AE starts high and
settles over a second or two -- on T31 a light of 30 % and of 100 % give
the same picture by frame 40 (the navy at Y 39) -- and, metering linear
light with this dark scene, exposes for the navy (Y ~90 on the T40 and
T41 SDK images). The AWB gains follow the light's colour (T31, frame
45: R 2.72 / B 1.27 in daylight, R 2.13 / B 1.75 under a warm light of
R 0.9 / B 0.35). The tests that check pictures take them once the AE has
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
the light at 10 %, 100 % or 1000 %. Its AWB reads the zones but keeps
the gains it starts with (R 2.28 / B 1.16, register 0x604 / 0x608) under
any light colour; why is not known.

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

T40 (`tx-isp-t40.ko` H20220606a, read off the T40 SDK 1.2.0 build with
symbols) is the T31 design again, rearranged: the core at 0x13300000 is
512 KiB with pipe 0's interrupts at 0x40020 / 0x40028 / 0x40024 (enable,
status, clear; IRQ 39 = INTC 31), the MSCA channels at 0x16100 + 0x100 n
with T31's offsets and their enable at 0x16008; VIC0 at 0x13380000 (IRQ
38) is T31's but for its frame start, bit 4 of 0x1E0, which starts the
sensor / AE work; the single sensor's CSI host is the one at 0x10054000.
Nothing is polled. The second pipe, VIC1/VIC2 (0x13390000, 0x133A0000)
and the second CSI (0x10023000) are not modelled.

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
IVDC, the ISP-to-encoder path at 0x13200000, is not modelled, so the
tests that run the vendor application turn it off (frames go through
memory, as on every other part). The core's 0x204 bit 0 follows 0x200
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
| T41NQ SDK 1.2.0 image + its modules, `sensor=gc4663` | `t41,variant=t41nq -m 128M` | `-kernel` + NOR root | 2560x1440 and 640x360 (`test-ingenic-isp.sh --soc t41`) |
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
| OpenIPC `t31` (its `audio.ko`) / T41NQ SDK image (its `audio.ko`) + SDK `libimp.a` | `t31` / `t41,variant=t41nq`, `audiodev=` wav | `-kernel` + NOR root, `test-ingenic-audio.sh [--soc t41]` | IMP_AI records 16 kHz mono: the 1 kHz tone (FFT peak 1000.0 Hz, rms 5793); IMP_AO plays a 440 Hz sine: 2.00 s of it in the wav (peak 440.0 Hz) |
| OpenIPC `t21` + `ingenic-isp`, jxf37 | `t21,sensor=jxf37` | NOR root, `test-ingenic-vpu-e2e.sh` | vendor modules, FrameSource -> IMP_Encoder: H.264 CBR and a JPEG snapshot, decoded on the host |

`python3 qemu/tests/test-ingenic.py` checks the models at register
level under qtest (no guest), including a migration round trip of `t31`
and `t41`, and the image pipeline: the GC2053's ID on I2C0, the VIC
poll, the MSCA FIFOs and a frame painted into a queued buffer under
`clock_step`, on T31 and through the T21, T23, T40 and T41 layouts, and
on T20 and T10 the Apical slots, a DMA writer's bank and a dropped frame;
and sound: a PDMA memory copy with its interrupt, a cyclic record ring
filled with the tone at 16 kHz and a play ring drained at that rate.

## Not modelled

T30's ISP streaming with OpenIPC's own `tx-isp-t30.ko` (it never sets
its Apical base; above), T40's second pipe and VIC1/VIC2, T21's AWB
response (above), AE1, AF, ADR, defog and WDR statistics, the Apical
AF and the zone statistics beyond AE and AWB, the VPU's TLB (libimp
addresses the engine physically), the T30 NCU/PP, the AIP, DMIC, the codec's analogue side (gains,
mutes: storage), PDMA transfers for peripherals other than the AIC, SSI,
the TCU channels' PWM output
and the RISC-V cores of T40/T41 read as zero or as storage.

Also storage or zero, and untouched by every boot tested here: the OTG
PHY registers (0x10060070/78; USB host works without them), the HASH,
AES and RSA engines, T40/T41's DTRNG, and T40/T41's BSCALER, I2D,
MONITOR, DRAWBOX and VO blocks.
