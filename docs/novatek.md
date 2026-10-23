# Novatek NT9856x (`nt98562`, `nt98566`), NT98533 (`nt98533`) and NT98332 (`nt98332`)

Novatek's NT9856x IP-camera SoCs (internal name NA51089): one Cortex-A9
with DDR in the package and Novatek's own peripheral blocks from
`0xF0000000`. One machine file (`hw/arm/novatek.c`) covers the family with a
table entry per part. It boots the OpenIPC `nt98562_lite` / `nt98566_lite`
kernel with its root on the emulated SPI NOR, Ethernet up and an SD card,
and boots a
whole flash image through the vendor U-Boot, on SPI NOR or (the 566 boards)
SPI NAND with a UBIFS root. What is left is in
[Not modelled](#not-modelled). [NT98533](#nt98533-ns02501) (NS02501) is
the same peripherals around two Cortex-A32s; [NT98332](#nt98332-na51103)
(NA51103) is an aarch64 NVR die.

## What the SDKs support

| Archive | Chip | What it has |
|---|---|---|
| `NT9856x-…-002.zip` → `na51089_linux_sdk_glibc_v1.01.002` | NA51089 (NT98560/561/562/566) | Kernel 4.19.91 and U-Boot 2019.04 source, board configs `configs/Linux/cfg_56*/nvt-*.dtsi`, loader control flow |
| `nt9856x_linux_sdk_v1.01.007.rar` | NA51089 | Newer loaders (≥ v1.01.003 for the newer in-package DDR), `NT98560_DataSheet_Register_Map.pdf`, the toolchains (`arm-ca9-linux-{gnueabihf,uclibcgnueabihf}` 8.4, a Linaro 6.4 `arm-eabi` for the loader), and the three tarballs below |
| → `nt9856x_linux_sdk_release_{uclibc,glibc}_v1.01.007` | NA51089, NA51103 | Kernel 4.19.91 and U-Boot 2019.04 source, as in v1.01.002. The U-Boot also carries NA51103: `arch-nvt-na51103_{a32,a64}` headers, `board/novatek/nvt-na51103{,-a64}`, `nvt-na51103_a32_*` defconfigs, and drivers (`eth_na51103.c`, SPI NOR / NAND, SDIO). The kernel has no NA51103 platform |
| → `nt9856x_loader_release_v1.01.007` | NA51089 | The loader's source (`bl_func.c`: FDT, U-Boot and SHMINFO), the DDR tables (`Tools/ConfigRam/560/*.ini`) and `bfc`; the DDR training is a closed library |
| `ns02501_linux_sdk_release_uclibc_v3.07.101` | NS02501 (NT98532 / NT98533) | Kernel 5.10.168 (`mach-nvt-ivot`, `CONFIG_NVT_IVOT_PLAT_NS02501`), U-Boot 2021.10 (`arch-nvt-ns02501_a32`, `nvt-ns02501_a32_{nor,nand,emmc}_defconfig`), board configs `cfg_53{2,3}_*`, HDAL. No toolchain: the SDK names `arm-ca32-linux-gnueabihf-10.4.0` |
| `ns02501_loader_v3.07.101` | NS02501 | The loader's source, as NA51089's (`bl_func.c`, `bl_lowlevel_init.c`) |
| `NT98533.zip` | NS02501 | `NT98533_DataSheet_Register_Map.pdf`, Boot ROM, boot loader and U-Boot guides, pinmux sheets |
| `NT9856x-…-000/001.zip` | NA51089 | Prebuilt NVTPACK images `FW9856{2,6}A.bin` (562 NOR, 562 NAND, 566 NAND) and loaders |
| `NT9832x_SDK_release_v1.02.0{13,24}` | NA51068 (NT98321/323/621/623) | A dual-A9 DVR part with Faraday IP at `0xFExx0000`; a different family, not modelled |
| `NA51023_BSP` | NT96672 | MIPS 24KEc running uITRON/eCos; not Linux, out of scope |

The OpenIPC kernel (branch `novatek-nt9856x` of `openipc/linux`) is the
v1.01.002 SDK kernel. NA51023 and NA51055 use NA51089's register offsets
(NA51023 at `0xC0000000`), so the blocks here are not 51089-only.

## Machines

| Machine | Chip ID (TOP +0xF0 [31:16]) | DDR in the package | Emulated: Linux / HDAL |
|---|---|---|---|
| `nt98562` | `0x7021` | 64 MiB DDR2 (SDK: 30 / 34 MiB) | 128 / 96 MiB |
| `nt98566` | `0x7021` | 128 MiB DDR3 (SDK: 32 / 96 MiB) | 128 / 96 MiB |

The SDK's memory tables (`nvt-mem-tbl.dtsi`) give Linux 30 or 32 MiB and
the HDAL media pool the rest. That is too little to run the HDAL samples
and their files beside the media modules, so both machines get 224 MiB by
default, more than the packages hold: Linux the first 128 MiB, and the
96 MiB media pool (`/hdal-memory`) after it. A larger `-m` grows the
pool. With less (`-m 64M` or `128M`, the packages' sizes) the machines fall
back to the SDK's split. The vendor firmware boots with its own device
tree and its own split.

The CPU is QEMU's Cortex-A9 with its MIDR set to r4p1. QEMU's default is
r0p0, and Linux's `arm_global_timer` refuses anything before r2p0 (a
comparator erratum), which leaves `sched_clock` on 100 Hz jiffies. The part's
real revision is not documented; r4p1 is the last A9 release.

## Memory map

From `nvt-basic.dtsi`, `nvt-peri.dtsi` and U-Boot's
`arch-nvt-na51089_a32/IOAddress.h`; IRQs are GIC SPI numbers.

| Block | Base | IRQ | Model |
|---|---|---|---|
| DDR | `0x00000000` | | RAM |
| DDR controller | `0xF0000000` | 33 | storage |
| TOP (pinmux, chip ID) | `0xF0010000` | | `nvt-top` |
| CG (clocks, PLLs, resets) | `0xF0020000` | | `nvt-cg` |
| PAD | `0xF0030000` | | storage |
| Novatek timer | `0xF0040000` | 0 | `nvt-timer` |
| WDT | `0xF0050000` | 57 | `nvt-wdt` |
| RTC + power button | `0xF0060000` | 56 | `nvt-rtc` |
| GPIO | `0xF0070000` | 24 | `nvt-gpio` |
| UART1 / 2 / 3 | `0xF0290000` / `0xF0300000` / `0xF0310000` | 43 / 44 / 45 | 16550, reg-shift 2 |
| Ethernet (DWC EQoS) + PHY | `0xF02B0000`, `0xF02B3800` | 34 | `nvt-eqos` |
| SPI NOR / NAND controller | `0xF0400000` | 29 | `nvt-flash` (NOR, or a W25N SPI NAND) |
| SDIO 1 / 2 | `0xF0420000` / `0xF0500000` | 30 / 31 | `nvt-sdio`, an SD card from `-drive if=sd,index=0` / `1` |
| SDIO 3 (eMMC host) | `0xF0510000` | 32 | `nvt-sdio`, an eMMC from `-drive if=sd,index=2` |
| PWM | `0xF0210000` | 26 | `nvt-pwm` |
| I2C 1 / 2 / 3 | `0xF0220000` / `0xF0350000` / `0xF03A0000` | 41 / 42 / 60 | `nvt-i2c` (`/dev/i2c-0..2`); `sensor=` puts a sensor on the first |
| USB 2.0 (FOTG200) | `0xF0600000`, alias `0xFF600000` | 27 | `nvt-usb`, host side; devices from `-device usb-*` |
| OTP / eFuse | `0xF0660000` | | `nvt-otp`, blank |
| Crypto (AES, DES, 3DES) | `0xF0620000` | 38 | `nvt-crypto` |
| Hash (SHA-1/256, HMAC) | `0xF0670000` | 28 | `nvt-hash` |
| RSA | `0xF06A0000` | 20 | `nvt-rsa` |
| SIE 1 / 2 / 3 (sensor capture) | `0xF0C00000` / `0xF0D20000` / `0xF0D30000` | 1 / 2 / 3 | `nvt-media` |
| IFE / DCE / IPE / IME | `0xF0C70000` / `0xF0C20000` / `0xF0C30000` / `0xF0C40000` | 8 / 7 / 5 / 6 | `nvt-media` |
| ISE / 2D graphics (2 engines) | `0xF0C90000` / `0xF0C80000`, `0xF0D10000` | 21 / 18, 19 | `nvt-media` |
| H.26x / JPEG codec | `0xF0A10000` / `0xF0A00000` | 16 / 17 | `nvt-media` |
| EAC / DAI (audio) | `0xF0640000` / `0xF0630000` | — / 15 | `nvt-audio` |
| IDE (display engine) | `0xF0800000` | 48 | `nvt-ide`, on QEMU's console |
| MIPI DSI | `0xF0840000` | 50 | `nvt-dsi` |
| AI: NUE / NUE2 / CNN | `0xF0C60000` / `0xF0D50000` / `0xF0CB0000` | 14 / 13 / 11 | `nvt-media`, `nvt-ai` |
| CSI, LVDS, sensor PHY, TGE, SIF | | | `nvt-media`, storage |
| On-chip SRAM | `0xF07C0000` | | — |
| A9 MPCore (SCU, GIC, timers) | `0xFFD00000` | PPI 11 / 13 | `a9mpcore_priv` |
| PL310 | `0xFFE00000` | 96 | `l2x0` |

The rest of `0xF0000000`–`0xF1FFFFFF` reads as zero and is logged under
`-d unimp`. The kernel's `plat-na51089/hardware.h` and U-Boot's
`IOADDR_GIC_REG_BASE` still say the GIC is at `0xF1500000`. That is a
leftover: the device tree and U-Boot's own `SCU_BASE_ADDR` (`na51089_regs.h`)
put the MPCore at `0xFFD00000`, and nothing on the boot path touches
`0xF1500000`.

## The device tree

The OpenIPC uImage has no appended DTB, and on silicon the loader hands
U-Boot one from flash. With `-kernel` the machine builds one from the SDK's
board files, keeping only what is modelled. Things the kernel depends on:

- Root `compatible = "novatek,na51089", "nvt,ca9"`. `mach-nvt-ivot/board-dt.c`
  matches `nvt,ca9`.
- Node names are the vendor's. The clock driver registers each gate under a
  device name such as `f0290000.uart`, and the drivers look their gate up by
  that name.
- `nvt,core_clk` (the CG node) comes before `nvt,periph_clk` in the tree. The
  periph_clk init reads CG through the base the core_clk init maps, and
  `of_clk_init()` runs them in tree order. With the order reversed the kernel
  faults in `get_periph_clk()` before the console is up. `fdt_add_subnode()`
  puts each new node first, so the code creates the two in reverse.
- The `nvt,nvt_top` node has three reg pairs (TOP, PAD, GPIO).
  `nvt_get_chip_id()` reads all six cells.
- The `nvt,nvt_top` node has the board's pinmux children
  (`nvt-top.dtsi`). Without them the pinctrl probe fails with -ENOMEM. The
  driver takes each child's index as its function ID, so the order matters
  too.
- `memory` is the Linux part of DDR, and `hdal-memory/media` is the rest,
  for the vendor modules.

`-dtb` overrides it, for example with a DTB extracted from a vendor
`FW9856xA.bin` (partition 0 of the NVTPACK).

## Clocks and timers

The kernel's clocksource and clockevents are the A9 global timer and TWD, at
`periph_clk = cpu_clk / 8`. The clock driver computes `cpu_clk` from CG:
`SYS_CLK_RATE[1:0] = 1` selects PLL8, whose rate is `12 MHz * ratio /
131072`, with the ratio in bytes at CG `+0x4420/0x4424/0x4428`. QEMU's
a9mpcore runs both timers at a fixed 100 MHz, so `nvt-cg` resets PLL8 to
ratio `0x855555` (800 MHz) instead of the silicon's 960 MHz (`0xA00000`).
With the real ratio, every delay and jiffy would run 20% slow.

The Novatek timer block at `0xF0040000` (`nvt-timer`, SPI 0) is not
Linux's clock. U-Boot and the HDAL use it. It has twenty up-counting
timers, laid out as the HDAL's `timer_reg.h` gives them:

- **Global registers:** STS (`0x10`, write 1 to clear), INT (`0x20`, the
  enables for this core) and CLKDIV (`0x30`).
- **Per timer n at `0x100 + 0x10n`:** CTRL (enable, free run, clock from
  divider 1), TARGET, COUNTER and RELOAD.
- **Clocks:** divider 0 runs at a fixed 1 MHz. Divider 1 runs at
  `3 MHz / ([15:8] + 1)`, which is 1 MHz at reset.
- **Counting:** a timer counts from 0 to TARGET, then sets its STS bit.
  A one-shot timer then stops, with its enable bit reading clear. A
  free-running timer wraps to 0 and goes on.

`timer_cfg()` polls CTRL after every enable and disable, so both take
effect at once. When divider 1's rate changes under a running timer, it
continues from its count at the new rate.

Timer 0 is the system timer. The loader leaves it running free at
1 MHz with the full 32-bit target, and the model resets it that way.
Three things rely on it:

- The vendor U-Boot reads COUNTER 0 (`0xF0040108`) as microseconds since
  power-on, for its `boot time` stamps and `timer2_delay()`.
- `nvt_get_cpu_freq` also reads it.
- The HDAL's `timer_open_sys_timer()` reads it too. Before the model, it
  printed `TIMER_0 is not configured` on the vendor NAND firmware, and
  U-Boot printed `boot time: 0(us)`.

The machine's device tree has the vendor node (`timer@f0040000`,
`nvt,nvt_timer`), which the release's `kdrv_comm.ko` binds. That build has
`CONFIG_NVT_SMALL_HDAL`, so it has no `/proc/nvt_timer_module`.

Three tests cover the timer:

- `test-novatek-boot.sh` checks that timer 0 counts about a million
  across a one-second sleep, and that the driver binds.
- `test-novatek-uboot.sh` checks that the vendor U-Boot's stamps count up.
- `test-novatek.py` checks it at register level: the one-shot and
  free-running timers with SPI 0, divider 1, RELOAD, and reset.

## Device notes

- **`nvt-wdt`.** Every CTRL write must carry the key `0x5A96` in [31:16]. The
  count runs at 12 MHz / 1024 up to `(MSB << 12) | 0xFFF`, and expiry either
  interrupts or resets. A write of 1 to MANUAL (`+0x0C`) resets at once
  while the watchdog is enabled; the kernel's restart hook uses it. The
  driver is a module (`na51055_wdt.ko`).
- **`nvt-rtc`.** Writes to TIMER, DAYKEY and PWRALM (the power-on alarm,
  which the driver's wake alarm uses) are shadowed until a CSET commits
  them. PWRALM and the daily ALARM go off on the RTC clock, setting the alarm
  status and, with CTRL's alarm enable, SPI 56. CSET completion is an
  interrupt the driver sleeps on before every access, so it has to fire. A key other than `0xA`, or an
  oscillator config other than `0x9`, makes the driver treat the RTC as
  power-lost and reset it to 1970. The model presents both, with the host's
  time. Reboot goes through the RTC: the vendor reboot notifier arms the
  power-on alarm three seconds ahead and powers off through PWBC, so a
  power-off with the alarm armed is a reset and one without it is a shutdown
  (`poweroff`). Without an RTC node the notifier dereferences a NULL base
  and the kernel panics on `reboot`.
- **`nvt-gpio`.** Eight 32-pin groups (C, P, S, L, D, H, A, DSI at
  `0x00`–`0xE0`), with DATA/DIR/SET/CLR words per group. Only 40 pins can
  interrupt, through a fixed table (`nvt_gpio_irq_table`), all on SPI 24.
  QOM lines `in`/`out` are numbered by pin, at `/machine/nvt-gpio`. An
  `out` line is the pad driver: the latch while the pin is an output, low
  while it is an input or after a reset.
- **UARTs.** 16550 registers at a stride of 4. `nvt_serial.c` adds DMA
  registers at `+0x80`, which read zero here, so the driver stays in PIO.

## SPI NOR and the flash controller

`nvt-flash` is the controller at `0xF0400000` with the boot NOR behind it,
backed by `-M nt98562,flash-file=img`. Erases and programs are written
through to the file, which must be a power of two from 1 to 32 MiB. The
controller sequences operations: software loads COMMAND, ROWADDR (the flash
address, sent MSB first in `MODULE0.row_addr + 1` bytes) and DATALEN, then
writes an operation code with OPER_EN into CTRL0 (`nvt_flash_spi_cmd.c`).
Command, command+address, dummy clocks, read and write of N bytes (through
the 16-word PIO FIFO or by DMA), status read, auto-status poll and block
erase are modelled. Each operation ends with CTRL_STS bit 12, which raises
SPI 29. The driver sleeps on that interrupt after every operation. CS is
either the controller's, per operation, or software's (SPI_CFG bit 1 for
manual, bit 4 for the level). A read holds CS low across CMD_ADDR, DUMMY_CLOCK
and READ_N.

The kernel's `spinor.c` with `trace-stdtable` matches a flash on the
capacity byte alone. So the default JEDEC ID is a Winbond W25Q of the
image's size, and an 8 MiB part is "STDR64FW": dual-output reads (0x3B), 64
KiB 0xD8 erases. The MTD is named after `nvt-devname`, so OpenIPC's command
line is
`mtdparts=spi_nor.0:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)`.

Writes from busybox `flashcp` go out as 256-byte DMA page programs. It pads a
short last buffer to 4 KiB, as on silicon. Smaller writes are PIO programs.
`test-novatek-boot.sh` checks both kinds against the image file.

## SPI NAND

The 566 boards (`cfg_566_IPCAM1_EVB`) boot from SPI NAND on the same
controller, which `spinand.c` (kernel and U-Boot) switches to NAND mode:
`MODULE0` bit 19 (`nand_type`) set, bit 20 (`spi_flash_type`) clear. A
`flash-file` of 128 or 256 MiB makes the flash a Winbond W25N01GV or
W25N02KV: ID `EF AA 21` / `22`, 2 KiB pages with 64 bytes of OOB, 128 KiB
blocks, both in the drivers' ID tables. The image is either the data alone,
with the OOB kept in memory as erased, or every page followed by its OOB
(2112 bytes a page). The chip is modelled at the SPI byte level. It has a
2112-byte cache register, PAGE DATA READ (`0x13`), the cache reads
(`0x03`/`0x0B`/`0x3B`/`0x6B`: column, dummy, data) and the program loads,
which AND into the page as NAND does. PROGRAM EXECUTE (`0x10`) and BLOCK
ERASE (`0xD8`) need WEL. The feature registers are GET/SET FEATURE at
A0/B0/C0. B0 resets to `0x18` (ECC-E, BUF), so the drivers pick on-die
ECC, and the status never reports an ECC error. Block protection (A0) is
storage only.

In NAND mode the controller changes how some operations go out on the bus,
which the drivers rely on:

- a status read is GET_FEATURE plus the feature address: two command bytes
  for CMD_STS, AUTO_STATUS and BLK_ERSE's poll (`0x0F 0xC0`);
- READ_ID clocks one address byte before the ID;
- `0x16` DATA_RD3 is a whole page read. It sends cmd 1 (PAGE READ) and the
  3-byte row, polls cmd 2 + cmd 3, then sends cmd 4 (cache read x4), the
  2-byte column and a dummy byte. DATALEN bytes go to DMASTART and the
  page's OOB to the spare SRAM at `+0x200`; a column in the OOB fills the
  spare SRAM alone. COMMAND2's cmd 5 + cmd 6 (`+0xB4`) then read the status
  into CTRL_STS[7:0], where the driver looks for ECC bits;
- `0x15` SPI_PROG_CMD is a whole page program. It sends cmd 1 (WEN), then
  cmd 2 (load x4) with the column, DATALEN bytes from DMASTART and the spare
  SRAM as the OOB, then cmd 3 (PROGRAM EXECUTE) with the row, and polls
  cmd 4 + COMMAND2[7:0].

The spare SRAM holds the OOB byte for byte: the drivers copy it word by word
to and from their page buffer (`nand_cmd_write_spare_sram`). Sub-page and
OOB-only writes take the manual-CS path in NOR mode instead (CMD_ADDR,
WRITE_N, then CMD_ADDR `0x10`). It needs nothing NAND-specific from the
controller.

The OpenIPC nt9856x kernels are built without `CONFIG_MTD_SPINAND`, so the
NAND is exercised with the vendor firmware: the SDK's 566_NAND
`FW98566A.bin` through `nvtpack2flash.py`. The vendor U-Boot probes the
W25N01GV, reads the kernel from NAND and boots it. Linux attaches UBI on
the rootfs partition and mounts UBIFS read-write as root, then reaches the
vendor shell. Its writes go back to the image file and are there on the
next boot. The only UBI complaint, "could not find any anchor PEB", comes
from the vendor image, which leaves UBI no free PEB for a fastmap.
`qemu-boot/test-novatek-nand.sh --flash nand.bin` runs the whole check.
U-Boot round-trips 256 KiB through `nand erase/write/read` and `crc32`,
and the image file holds it afterwards. Linux keeps a file on UBIFS across
`reboot -f`. The test takes about 20 s. `test-novatek.py` plays the
drivers' register sequences against both image formats in CI.

## Flash boot

On silicon the mask ROM loads the 32 KiB loader (`LD9856xA.bin`) into SRAM.
The loader trains DDR with closed libraries and then, per `bl_func.c`:

1. copies the FDT from the fixed `fdt` partition to `/nvt_memory_cfg/fdt`:
   flash `0x10000` on NOR, block 2 (`FDT_OFFSET`, `0x40000`) on NAND;
2. loads `partition_uboot` (from the tree's `nor` or `nand` node) to
   `/nvt_memory_cfg/uboot`, unpacking it if it is BCL1. The 566 NAND image
   stores it plain;
3. fills the 328-byte SHMINFO at `/nvt_memory_cfg/shmem`;
4. calls U-Boot's entry.

DDR needs no training here, so with `flash-file=` and no `-kernel` the
machine does steps 1–4 itself and leaves `ldr pc, [pc, #-4]` plus U-Boot's
address at 0 for the CPU to start on. The SHMINFO is what U-Boot's
`nvt_shminfo_init()` hangs without: `"LD_NVT"` at +0 and the FDT address at
+44 (`arch/arm/include/asm/nvt-common/shm_info.h`). `boot.LdStorage` at +42
says 3 (SPI NOR) or 2 (SPI NAND).

BCL1 is Novatek's container, as the SDK's `build/nvt-tools/bfc` writes
it: 16 bytes of header, then the data.

| Offset | Field |
|---|---|
| 0 | `"BCL1"` |
| 4 | checksum, little-endian 16-bit: makes the sum over the container's little-endian 16-bit words, each plus its index, 0 mod 65536 (`nvtpack_calc_nvt_sum`; a last odd byte is not counted) |
| 6 | algorithm, big-endian 16-bit |
| 8 | uncompressed size, big-endian |
| 12 | compressed size (the data after the header), big-endian |

Both algorithms the loader knows are unpacked here:

- **9 (`bfc c lz`).** An LZ77 with a marker byte and 7-bit variable-length
  lengths and offsets (U-Boot's `lz_uncompress`).
- **11 (`bfc c lzma`).** An LZMA "alone" stream: the properties byte
  (`0x5D`), the 32-bit dictionary size and the 64-bit unpacked size, all
  little-endian, then the range-coded data. `bfc` fills in the size; an
  unknown size (all ones) with an end marker unpacks too. The loader
  passes the data after the BCL1 header, with both sizes from the header,
  to the mask ROM's decoder (its address is the word at `0x7FD0`,
  `rom_lzma_inflate`). U-Boot's `lzmaBuffToBuffDecompress()` takes the
  same stream for the SDK's LZMA RTOS images. `python3 -c 'import lzma'`
  with `format=lzma.FORMAT_ALONE` writes it, with the size field left
  unknown.

A plain U-Boot is copied by its own length: the loader reads `HEADINFO`
at image `+0x300` (`BIN_INFO_OFFSET_UBOOT`, U-Boot's `nvt_headinfo.c`)
and copies `BinLength` (`+0x368`) bytes. The SDK's build writes that
field and the checksum after linking (`encrypt_bin SUM u-boot.bin 0x350`);
a raw `u-boot.bin` has `0xFFFFFFFF` there. The stand-in copies the whole
partition instead, so it does not show a missing length.

The loader's normal boot does not check the BCL1 checksum (only its
update path does). It reads the compressed U-Boot from flash as
`ALIGN_CEIL(compressed size, block size)` bytes from the partition's
start, which does not count the 16-byte header: an image whose last 16
bytes fall past that boundary loses them on silicon. The stand-in reads
the whole partition, so it does not show that.

### Xiongmai's layout

Xiongmai's NT98562 / NT98566 cameras (the boards OpenIPC supports) have
no `fdt` partition. Their loader (`loader.bin.img`, flash 0–`0x10000`)
carries the board's device tree itself, plain, behind an `"XMFDT562"` tag
at loader `+0x8000`. U-Boot follows the loader at `0x10000` as
`partition_boot`, whose node has an `lzma` property: the partition is an
LZMA "alone" stream, size unknown, with an end marker. The environment
is at `0x30000`. With no tree at flash `0x10000` the machine looks for
one in the loader (the first FDT there with `/nvt_memory_cfg`), takes
`partition_boot` when there is no `partition_uboot`, and unpacks it as
LZMA when the node says `lzma` (refusing it if it does not unpack).

A Xiongmai upgrade's `*.img` files are uImages whose load address and
entry point are the region's start and end in the flash.
`qemu-boot/xmimg2flash.py` lays them out, optionally with raw files at
given offsets (`FILE@OFFSET`):

```bash
python3 qemu-boot/xmimg2flash.py -o xm.bin u/loader.bin.img \
    u/u-boot.bin.img u/u-boot.env.img u/romfs-x.squashfs.img \
    u/user-x.squashfs.img u/web-x.squashfs.img u/custom-x.squashfs.img
FLASH=xm.bin bash qemu-boot/run-nt98562.sh
```

With the 80N40 board's firmware (`IPC_NT98562_80N40_S38`, U-Boot 2019.04
of December 2021) Xiongmai's U-Boot comes up, probes the W25Q64, reads
its environment, loads `boot/uImage` from the romfs and starts it. Its
kernel then prints nothing, and within two minutes the board resets;
that part is not investigated.

The SDK ships NVTPACK update packages, not flash dumps.
`qemu-boot/nvtpack2flash.py FW98562A.bin nor.bin` lays one out the way the
vendor U-Boot's updater writes it. It places each partition where the
pack's own device tree says (`nvtpack/index/idN` → `partition_<name>`) and
strips the 64-byte CKSM header from the rootfs and app images. Booted with
`FLASH=nor.bin qemu-boot/run-nt98562.sh`, the SDK's `FW98562A.bin` runs
U-Boot 2019.04 (SPI NOR probed, empty SD slot, `nvt_boot`), the kernel and
the vendor init scripts. A NAND pack (`FW98566A.bin`) comes out as the
128 MiB NAND's data, its UBI rootfs as ubinize made it. It boots the same
way with `SOC=nt98566 FLASH=nand.bin`.

## SD card

`nvt-sdio` is the Novatek SD host (`na51055_mmc{,host,reg}` in the kernel,
`nvt_ivot_mmc` in U-Boot) with QEMU's `sd-card` on its bus:
`-drive if=sd,index=0,format=raw,file=sd.img` puts a card in SDIO1 and
`index=1` one in SDIO2. The OpenIPC rootfs carries the driver as modules
(`mmc_core.ko`, `mmc_block.ko`, `mmc_na51055.ko`); vfat is built in.

The drivers set the data phase up first: DATA_LENGTH, FIFO_CTRL (DMA or
PIO, direction), DATA_EN, then FIFO_EN. Then they write the command
with CMD_EN, and the data moves with it. Command completion is CMD_SENT,
plus RSP_CRC_OK with a response. An R2 goes to RSP3..RSP0, with bits
127..96 in RSP3, because the drivers read RSP3 into `resp[0]`. The drivers
send R2 with LONG_RSP alone, not NEED_RSP. A DMA transfer completes at
once, so CMD_SENT, RSP_CRC_OK, DATA_END and DATA_CRC_OK arrive together,
and the kernel's IRQ handler takes them in that order. FIFO_EN and DATA_EN
clear when a transfer ends.

The two drivers DMA differently through the same register at `+0x110`:

- **Linux** (`SDIO_SCATTER_DMA`) gives a table of 3-word descriptors:
  valid/end/act=2, length, address, at most 128.
- **U-Boot** writes `0x1FC` = 1 and the buffer's own address.

Linux never clears `0x1FC`. On silicon the block is reset under it: the
clock framework drops and raises the host's CG_SYS_RESET_REG1 bit (2 for
SDIO1, 3 for SDIO2) when it prepares the clock
(`nvt_composite_gate_clk_prepare`, DO_RESET), and U-Boot does the same by
hand. `nvt-cg` drives a "sys-reset" line for every SYS_RESET bit, and the
machine wires those two into the hosts, so the U-Boot to Linux handover
works as on the chip.

Buffers that are not cache-line aligned, such as the 8-byte SCR, go by
PIO. In its command-done handler the driver drains or fills DATA_PORT, 16
words when FIFO_STS says full (read) or empty (write), the rest when the
count matches. DATA_END follows the last word. BUS_STATUS bit 0 reads
"not busy", which the kernel checks before every data request.

Card detect: the generated device tree gives no `cd_gpio`, so the kernel
takes the slot as full, and an empty one just times out (RSP_TIMEOUT).
The vendor EVB trees use `cd_gpio = <9 0 0>`, C_GPIO 9 low with a card
in. The machine drives that pad low when SDIO1 has a card and high
otherwise.

What runs:

- `test-novatek-boot.sh` (CI) loads the release's modules. It finds the
  card and its MBR, and reads 1 MiB the host put there, matching the
  host's md5. It writes 192 KiB raw, which the host then finds in the card
  image. It also makes a vfat on the partition that keeps a file across a
  remount.
- `test-novatek-uboot.sh` with a vendor image runs `mmc info` (SD 3.0,
  4-bit, high speed), `mmc part`, a 256 KiB `mmc read` whose `crc32`
  matches the host's, and an `mmc write` the host finds in the image.
- The vendor `FW98566A.bin` Linux finds the card and its hotplug mounts
  the partition.

### eMMC

SDIO3 at `0xF0510000` (SPI 32) is the same host as SDIO1/2. It is the
SDK's eMMC board's `mmc@f0510000` (`cfg_562_EMMC_EVB`), compatible
`nvt,nvt_mmc3`, which the same `na51055_mmc` driver serves, and its CG
reset is `CG_SYS_RESET_REG1` bit 14. `-drive if=sd,index=2` puts QEMU's
`emmc` card on it. The generated tree gives it `bus-width = <8>` and no
`cd_gpio`, so the driver runs it 8 bits wide and takes it as always
present.

The SDK's eMMC board wires the bus 4 bits wide and sets `voltage-switch`.
Here the node keeps `voltage-switch = <0>` and the card stays at high
speed (no HS200). With the release's `mmc_na51055.ko` the eMMC comes up as
"new high speed MMC card" on `mmc2`, block device `mmcblk2`.
`test-novatek-boot.sh` (CI) checks that it is an MMC card on an 8-bit
host, reads back 512 KiB the host put there, and writes 192 KiB raw that
the host then finds in the image. `test-novatek.py` runs the eMMC
sequence against the registers: CMD1 until ready, CMD2 / 3 / 7, and the
512-byte EXT_CSD by descriptor DMA. It also checks SDIO3's CG reset bit
and the device-tree node. The shipped vendor firmware is NOR / NAND and
registers only SDIO1, so eMMC boot (the SDK's `nvt-na51089_emmc_defconfig`)
is not exercised.

## I2C and the image sensor

`nvt-i2c` is the controller `i2c-nvt.c` drives, three instances
(`/dev/i2c-0..2`, the device tree's `id`). The node names (`i2c@`,
`i2c2@`, `i2c3@`) are the vendor's, because the driver gets its clock by
device name (`f0350000.i2c2`). A message moves in PIO chunks of up to four
bytes. DATA holds the bytes, and CONFIG gives the chunk's length [18:16]
and, per byte i, START before it (bit 8+i), STOP after it (12+i) and the
master's NACK (4+i). Writing CTRL with TB_EN starts the chunk. A byte
with START is the address byte; it opens or restarts a transfer on the
bus, and the bytes after it are sent or received as its R/W bit says. A
chunk ends with STS.DT (sent), STS.DR (received, data in DATA from byte 1
of the first chunk) or STS.BERR when nothing ACKs. The driver then sends a
lone STOP. The bus always reads free. The DMA, VD-synchronised writes
(`i2c_nvtim_xfer_msg_vd`) are not modelled.

`-M nt98566,sensor=sc401ai` (or `sc501ai`) puts a SmartSens sensor at
0x30 on I2C 0, the repo's `hisi-smartsens` model with that part's chip
ID (0xCD2E or 0xCE1F at 0x3107/0x3108). That is where OpenIPC's `ipcinfo`
looks on Novatek (`ssens_addrs` 0x60, 8-bit). The OpenIPC release
carries drivers for exactly those parts (`nvt_sen_sc401ai.ko`, …). The
default is `sensor=none`. With a sensor, the stock init behaves as on a
camera: `load_novatek` gets `sc401ai` from `ipcinfo --short-sensor`,
loads the HDAL stack and `nvt_sen_sc401ai.ko`, and majestic starts. Boot
still reaches the login prompt. Nothing past the sensor's registers is
modelled: no MIPI receiver or ISP, so no frames.

`test-novatek-boot.sh` runs with `sensor=sc401ai` and checks three
adapters, `i2cdetect` finding 0x30, and the release's own `ipcinfo`
naming the part. `test-novatek.py` replays the driver's chunk sequences:
the chip-ID read (address write, repeated START, read), writes and reads
that span two chunks, a NACK and its STOP, and the `i2cdetect` quick
write.

## PWM

`nvt-pwm` is the controller at `0xF0210000`, laid out as the kernel's
`pwm-na51055.c` and the HDAL's `pwm_reg.h` describe it:

- **Per channel:** CTRL (+8n, a cycle count in [15:0]) and PERIOD (+8n+4:
  rise, fall and base period, one byte each, and invert in bit 28). For
  channels 0–7 the high bytes of rise, fall and base period are at
  0x230+4n.
- **ENABLE (0x100):** writing a 1 starts a channel, and the register reads
  back the running set. The driver polls it.
- **DISABLE (0x104):** stops the channel and raises its bit in INT_STS
  (0xF0) and SPI 26. `nvt_pwm_disable()` waits for that interrupt.
- **Cycle count:** with one set, the channel stops by itself after that
  many periods, with the same interrupt. Enable waits for it, and the
  model times it at a nominal 30 MHz (the 120 MHz source over the
  drivers' default divider of 4).
- **CLKDIV_LOAD (0x10C)** sets CLKDIV_LOAD_STS (0xFC).

There is no output pin, but each channel drives the device's `out` GPIO
line n: high while it runs with a nonzero duty.

Nothing shipped for these SoCs drives the PWM. The OpenIPC and SDK
kernels have `CONFIG_PWM` off. The HDAL's `kdrv_comm.ko` carries a PWM
driver, but it binds only `nvt,nvt_kdrv_pwm` and refuses `nvt,nvt_pwm`,
which is what the vendor device trees (and this machine's) have. So the
test kernel from `build-novatek-test-kernel.sh` (see USB) also builds
`pwm-na51055.c` with sysfs. `qemu-boot/test-novatek-pwm.sh --kernel
testk/uImage.nt9856x-test` drives channel 0 through `/sys/class/pwm`
and checks the registers with `devmem`:

- a 1 ms period at 50 % gives base period 30000 and fall 15000
  (PERIOD 0x00309800, extension 0x00753A00);
- enable sets ENABLE, and disable returns on the stop interrupt and clears
  it;
- the vendor `cycle` attribute runs 20 periods, after which enable
  returns with the channel stopped.

All 7 checks pass. `test-novatek.py` checks the same in CI at register
level, along with the `out` line, the 20 ms cycle-count timing and the
divider-load status.

## USB

The block at `0xF0600000` is a Faraday FOTG200 dual-role controller. Its
host side has an EHCI laid out like Faraday's FUSBH200, which QEMU already
models: capabilities at 0x00, operational registers at 0x10, no
CONFIGFLAG, and the single port's PORTSC at 0x30. The vendor kernel's
`ehci_def.h` uses that layout under `CONFIG_USB_NVTIVOT_HCD`. `nvt-usb` is
`fusbh200-ehci-usb` with the rest of the 64 KiB block behind it, which the
host driver (`ehci-na51089.c`) programs:

- **+0x80 OTG control/status.** Bits [23:22] give the speed of the device
  on the port (0 full, 1 low, 2 high). `ehci_hub_control()` reads them
  instead of PORTSC's (`has_tt`, `ehci_port_speed()`), so a high-speed
  stick shows up as high speed. A_BUS_REQ / A_BUS_DROP are stored.
- +0x100, +0x1C8, +0x30C, +0x400 and the UTMI PHY at +0x1000 are
  storage.

On NA51089 the driver does not use the address in the device tree. Unless
TOP's version word reads 0x48210000 it adds 0x0F000000, so it maps
`0xFF600000`, and the machine maps the block at both addresses. Before
starting the controller, `nvtim_init_usbhc()` reads the PHY trim with
`efuse_readParamOps()`. That oopses unless the OTP driver is bound, and
the OTP driver's code (a shipped object) spins on the eFuse status bits.
So the device tree has the vendor's `nvt_otp@f0660000` node, and `nvt-otp`
models the handshake. Bit 0 of 0x20 powers the array and 0x30 bit 0 reports
it ready. A read sets 0x20 bit 4, which clears when done, with 0x30 bit 1
set (write 1 to clear) and the word at 0x08. The array reads as blank, so
every caller keeps its default trim. The device side (`nvt,fotg200_udc`)
is not modelled.

No shipped firmware for these SoCs has a USB host driver. The OpenIPC
kernels are built without `CONFIG_USB_EHCI_HCD`, and the SDK's kernel and
U-Boot leave USB off. `qemu-boot/build-novatek-test-kernel.sh` builds the
same kernel OpenIPC builds, with EHCI and USB storage added, to test
against:

- the source is `openipc/linux` branch `novatek-nt9856x`;
- the config is OpenIPC's `nt98566.generic.config`;
- the toolchain is OpenIPC's own, and the build runs in-tree like
  Buildroot. An out-of-tree build loses the OTP driver's shipped object.

Then:

```bash
bash qemu-boot/build-novatek-test-kernel.sh --out testk
bash qemu-boot/test-novatek-usb.sh --soc nt98562 --kernel testk/uImage.nt9856x-test
```

`--soc` names the OpenIPC release whose rootfs supplies busybox
(`qemu-boot/rootfs.squashfs.<soc>`); the kernel runs on either machine.

That test attaches a QEMU `usb-storage` stick
(`-device usb-storage,drive=...`). It checks that `ehci-nvtivot` binds at
`0xFF600000` and the stick enumerates at high speed (480 Mb/s) as `sda`.
It reads 1 MiB the host put there, writes 192 KiB raw that the host then
finds in the image, and keeps a vfat file across a remount: 11/11 on
nt98562 and nt98566. `test-novatek.py` covers the model in CI: the
EHCI at both addresses, PORTSC and the OTG speed bits with and without a
device, a port reset, the glue storage, and the OTP handshake.

## Crypto

There are three security engines, each with its vendor driver in
`drivers/crypto/nvt_ivot/` (`na51089_crypto.c`, `na51089_hash.c` and
`na51089_rsa.c`). The device tree has the vendor nodes: `crypto@f0620000`,
`hash@f0670000` and `rsa@f06a0000`, with `mclk = <1>`. U-Boot's
`IOAddress.h` puts the hash engine at `0xF0700000`; the vendor device tree
and the clock driver (`f0670000.hash`) say `0xF0670000`, and that is
where it is.

**`nvt-crypto`** runs AES-128/256, DES and 3DES in ECB, CBC, CFB, OFB and
CTR, and AES-GCM. The driver builds CCM from CBC and CTR passes.

- **PIO:** one ECB block through the KEY, IN and OUT registers.
- **DMA:** a 0x280-byte descriptor holds the key, the IV, the CTR step and
  a header (direction, cipher, mode, key source, "get S0"). It ends in up
  to 32 `{src, dst, length, cfg}` blocks. The blocks form one stream up to
  the block flagged last; in GCM the first such group is the associated
  data.
- **Write-back:** the engine writes three values into the descriptor, and
  the driver depends on all of them:
  - **CV**, the IV that continues the stream. The driver chains requests
    longer than 32 blocks through it, takes CCM's CBC-MAC from it, and
    treats a CV still zero as an engine that has not finished.
  - **S0** = E(K, first counter block).
  - **GHASH** over the associated data, the ciphertext and their lengths.
    The GCM tag is S0 ^ GHASH.
- **Key manager:** with the key-source bit set, the key comes from an OTP
  key set. The OTP is blank, so that key is zero.

**`nvt-hash`** runs SHA-1, SHA-256 and their HMACs by DMA. Requests come
in chunks: the state carries over through the IV registers, the engine
pads the last chunk to the length in PADLEN, and it adds the HMAC's ipad
and opad blocks itself when asked.

**`nvt-rsa`** computes DATA ^ E/D mod N, from 256 to 2048 bits. The driver
writes the operands little-endian into the engine's SRAM through three
auto-incrementing ports, and reads the result back the same way. The CRC32
key-check modes are not modelled: such a request never reports done, and
the driver (which never asks for one) would time it out.

The engines complete each request at once. The ciphers are implemented
in the model itself: AES from QEMU's `crypto/aes.c`, and DES, SHA, GHASH
and a Montgomery bignum in the files. So nothing depends on which crypto
library QEMU was built with.

No shipped kernel enables these drivers. The vendor U-Boot does use the
cipher engine: `nvt_encrypt` and the encrypted-kernel path, keyed from
the OTP. `build-novatek-test-kernel.sh` (see USB) builds all three drivers
with the kernel's crypto self-tests on. testmgr then checks every
algorithm the drivers register, as it registers, against the kernel's
known-answer vectors:

```bash
bash qemu-boot/test-novatek-crypto.sh --kernel testk/uImage.nt9856x-test
```

That test checks `/proc/crypto` and reports 26/26:

- 22 `na51089-*` algorithms, `rsa` among them, report `selftest: passed`;
- testmgr has no vectors for DES and 3DES in CFB and OFB, so those four
  pass without having run.

Breaking the models on purpose (one output bit flipped) makes AES, DES,
GCM, CCM, SHA, HMAC and RSA fail, so the tests do reach the emulated
engines.

`test-novatek-uboot.sh` runs the vendor U-Boot's `nvt_encrypt 0`:
AES-128-ECB of its sample block under the OTP key, then decrypted back.
The ciphertext must be `fb ee 76 ee ...`, AES-128 under a zero key.

`test-novatek.py` checks the engines at register level, in CI:

- **Cipher:**
  - AES by PIO (FIPS-197);
  - AES-CBC and CTR (SP 800-38A), checking CV and a partial last block;
  - GCM (the McGrew and Viega vectors), checking that S0 ^ GHASH gives the
    tag;
  - DES and 3DES, and a key-manager key.
- **Hash:** SHA-1 and SHA-256 padded by the engine, a message hashed in two
  chunks, and HMAC (RFC 2202 and RFC 4231).
- **RSA:** 256, 512 and 2048-bit exponentiation, checked against Python's
  `pow()`.

## HDAL media

OpenIPC's `load_novatek` loads the vendor's HDAL kernel modules:
- kwrap, nvtmem and kdrv_builtin;
- kdrv_comm;
- the video capture, process, codec and gfx drivers and their kflow units;
- audio;
- the ISP, AE, AWB and IQ;
- the sensor driver.

They bind by device tree, so the machine generates the vendor nodes
(`nvt-peri.dtsi`, `nvt-media.dtsi`, `nvt-mem-tbl.dtsi`):
- every engine, with its windows and interrupts;
- the HDAL's software units (`nvtmpp`, `isf_flow`, `isf_vdocap`, ...);
- `/hdal-memory`;
- `/hdal-maxpath-cfg`, the path limits the units size by (`isf_vdoenc`
  fails its probe without it);
- `/pll_preset@0`. `nvt_preset_pll()` sets PLL5 to 297 MHz from it, which
  the sensor master clock divides to 27 MHz. Without it `ctl_sen` fails the
  open with "cannot find pll for MCLK".

Every engine is an `nvt-media` block (`hw/misc/nvt-media.c`): register
storage, plus the handshakes its driver waits on. Video pixels are real
from the sensor to JPEG (`hw/misc/nvt-pixel.c`), and the 2D graphics
engines and the ISE scaler compute their jobs (`hw/misc/nvt-gfx.c`). The
H.26x encoder codes real H.264 / H.265 when QEMU is built with x264 /
x265 (`hw/misc/nvt-h26x.c`). Audio is its own device, `nvt-audio`
(below).

- **SIE (sensor capture).** While `ACT_EN` is set, a frame comes every
  1/30 s, in three steps the way the sensor's lines would arrive:
  1. VD and the window starts;
  2. the line break points, half way;
  3. the window ends and the DRAM outputs' ends.

  An output in single mode clears its enable (`0xA4` bits 28..30) once
  its frame is written. That is how `ctl_sie` knows a buffer is complete
  and hands it on.

  At each frame end the SIE writes the sensor's picture to DRAM output 0
  (address `0x5C`, line offset `0x60` [15:2]), at the crop size (`0x2C`),
  packed as `0x60` [17:16] says (8, 10, 12 or 16 bits, a little-endian
  bit stream). The picture is a test pattern, as RGGB Bayer:
  - 75 % colour bars over the top two thirds;
  - a grey ramp below;
  - a white box that moves 24 pixels a frame.
- **IFE, DCE, IPE, IME, ISE, 2D graphics.** `kdrv_ipp` and the graphics
  driver work in linked-list mode: a list of 64-bit commands in DRAM
  (`kdrv_ipp_int_util.h`, `grph_ll.h`). The engine applies each UPD
  (a register write with byte enables) through its own registers, which
  include its interrupt enables, and follows NXTUPD. At NXTLL or NULL a
  job is done: some 1 ms later it raises frame and job end, plus list end
  after the last. The graphics engine raises list end only. `graph_isr`
  reports DMA idle as an abort.

  When IME finishes a job, the RAW frame the IFE was pointed at becomes
  YUV 4:2:0 semi-planar at IME output path 1:
  - IFE input: address `0x30`, line offset `0x34`, size `0x20`, depth
    CTRL1 [6:5];
  - IME path 1: enable `0x04` bit 2, size `0x84`, Y and UV at `0x94` and
    `0x98`, line offsets `0x8C` and `0x90`;
  - the conversion: 2x2 demosaic, BT.601 limited range, nearest-neighbour
    scaling to the output size.

  The machine links IME to IFE (`ipp-input`). DCE's and IPE's corrections
  are not applied.

  The graphics engines and ISE do each job's work when it completes,
  from the register file as the list left it. The driver re-sends only the
  registers that changed, so the state carries over from job to job.

  The graphics engine's CONFIG (`0x04`) holds:
  - bit 0: AOP (pixel arithmetic) or GOP (geometry);
  - the precision, [6:4];
  - the 16-bit format, [10:8];
  - the GOP, [19:16], and the AOP, [28:24].

  Images A, B and C are at `0x70`, `0x80` and `0x90`, with their line
  offsets in the next word. They share the line count (`0x78`) and the
  bytes a line (`0x7C`). Modelled:
  - **A_COPY.** hd_gfx_memcpy is this engine, not a DMA block.
  - **PLUS / MINUS / MULTIPLY.**
    - At 8 bits, B's shift is its IN2 operator, results saturate, and
      OUT operator 4 takes the magnitude.
    - At 16 bits, the shift is CONST1 [3:0], bit 31 asks for the
      magnitude, and sums wrap.
  - **TEXT_COPY**, the fills the line and rectangle drawing becomes. At
    8 bits, byte k of CONST1 lands where the address is k mod 4.
  - **PACKING**: U and V planes into pairs, W counted in output bytes.
  - **RGBYUV_BLEND**: ARGB4444 / 1555 / 8888 or RGB565 onto NV12.
    - Colours convert with the formula of the vendor's own
      `argb_to_ycbcr`, so engine blends match the driver's fills.
    - Blending is per-pixel alpha.
    - With CONST1 bit 0, U / V are 4:2:0; bit 5 averages each 2x2 block.
  - **GOP flips and the half turn.** For a vertical flip the driver points
    C at the last line.

  The quarter turns and the remaining AOPs log as unimplemented.

  ISE scales Y8, U / V pairs or ARGB8888 from `0x10` / `0x18` / `0x20`
  (size, line offset, address) to `0x38` / `0x90` / `0x98`. It uses the
  method in `0x04` [9:8]:
  - bilinear, corner to corner;
  - nearest;
  - integration, as a box average.

  The blend rounding and the integration filter are this model's choices;
  the driver does not pin them down. The affine engine is not modelled:
  the release ships no `kdrv_affine.ko`.
- **H.26x.** `INT_FLAG` is write-1-to-clear, and software reset and DMA
  disable set their finish flags. A picture finishes with:
  - the picture in the bitstream buffer;
  - its length in `BS_LEN` (report slot 6) and the NAL length list;
  - the slice QP times the macroblocks or CTBs in `QP_SUM` (slot 19).

  A picture the buffer cannot take raises buffer full (bit 12) instead.

  The driver writes every parameter set and slice header in software,
  and the engine only codes slice data. The headers reach the engine
  through the BSDMA list at `0x06C`:
  - a count, then an address and length for each slice;
  - the length's top nibble is the bits used in the last byte;
  - H.265 adds a 1-byte entry after the last slice.

  On an IDR the first entry starts with the sequence header (SPS / PPS,
  plus VPS for H.265). HDAL skips those bytes in the buffer and takes
  them from its own copy. So a decoder always sees the driver's
  parameter sets, and real slices must fit them.

  When pkg-config finds x264 / x265 at build time, each stream (keyed by
  its BSDMA list) gets an encoder set up to match the driver's parameter
  sets in everything the slice data depends on. The encoder's own headers
  are parsed and checked against the driver's before it is used.
  - **H.264:** the driver's slice header goes out unchanged. x264 is made
    to code at its slice QP and deblocking. In CRF mode the forced QP is
    exact; in CQP mode x264 would clamp it.
  - **H.265:** the slice header is built from two sources.
    - From the driver's: NAL type, POC and reference set.
    - From x265's: slice type, QP, SAO, deblocking and merge candidates.
    - x265 signals `cu_qp_delta` only with adaptive quantisation, so it
      runs in CRF with a small AQ strength, the picture QP still forced.

  The source is NV12 at `0x024` / `0x028`, with the line offsets in words
  at `0x084`, padded to the coded size.

  **Tiled H.265.** At 2880x1620 the driver codes H.265 in tile columns:
  - one row of two columns, 1472 and 1408 pixels;
  - one slice per column, the second starting at that column's first CTB;
  - no loop filter across tiles or slices.

  So each column decodes like a picture of its own. Intra prediction,
  CABAC, deblocking and SAO all stop at its edges, as at a picture's
  edges. Each column gets its own x265 at that width, which codes it as
  an intra picture.

  Its slice goes out as an I slice under the driver's slice: same
  address, NAL type, POC and reference set, and no temporal MV
  prediction. Inter coding could not be stitched this way: a motion
  vector near a column edge reaches into the next column, which the
  column's encoder never sees. Tiled streams are therefore all intra.

  Encoding runs in a worker thread (QEMU's thread pool), so a 5 MP
  picture does not hold the main loop:
  - the main loop reads the driver's headers and the source;
  - the worker sets the encoders up and codes;
  - the main loop then writes the bitstream and raises the interrupt.

  A reset while a picture is coding drops it.

  Without the libraries, every slice is a filler NAL unit, which decoders
  skip; on an IDR the sequence header still goes in front. The same
  happens for a stream the encoders cannot match:
  - several slices without tiles, or tiles in rows or not one slice per
    column, or filtered across;
  - a compressed or direct-from-IME source;
  - a reference that is not the previous picture.

  Such a stream stays filler until its next IDR. So does every stream
  after a migration, since the encoders are host state. A real picture
  too big for the buffer raises buffer full, and its stream waits for an
  IDR as well.
- **JPEG.** A baseline encoder (float DCT, the standard Annex K Huffman
  tables) reads the 4:2:0 picture:
  - Y and UV at `0x54` and `0x58`, line offsets at `0x60` and `0x64`;
  - MCU columns and rows at `0x28`;
  - each of these is stored minus one.

  It quantises with the tables the driver loads through the LUT port:
  table 1 holds the luma then the chroma table, zig-zag order, four bytes
  per word. The Huffman tables match the DHT in the driver's own header
  byte for byte. The scan ends in EOI after the software's header.

  With no picture to read, every block is coded flat grey ("DC difference
  0, end of block").
- **Audio: the DAI and the EAC** are one device, `nvt-audio`
  (`hw/misc/nvt-audio.c`), from `dai_reg.h` / `eac_reg.h` as the closed
  `dai.o` and `Audio.o` drive them.

  The DAI's five DMAs are TX1, TX2, RX1, RX2 and TXLB:
  - buffer addresses at `0x40`..`0x50`, sizes in words at `0x60`..`0x70`,
    and where each is now at `0x80`..`0x90`;
  - formats at `0x10` / `0x14` / `0x18`: mode, sample size, mono in DRAM.

  The address and size registers queue the next buffer:
  - enabling a DMA takes them (DMA loaded);
  - each buffer's end raises buffer done and takes them again, so the
    driver re-arms the queue from its interrupt;
  - a channel enable (`CTRL` bits 12..15) falling raises stopped, which
    the driver waits on.

  The driver spins on the current-address registers when it stops, so
  they advance with time.

  Neither block holds the sample rate. The clock tree gives the DAI 256
  times it: PLL7 through the eacdac and DAI dividers at CG `0x38`. The
  kernel's clock setup leaves 48 kHz. The DMAs run at that rate on the
  virtual clock, and buffer ends land on time.
  - **RX** captures a test tone: 1 kHz left, 1.5 kHz right, half scale
    (`-global nvt-audio.tone-hz=`, 0 for silence). With
    `rx-source=audiodev` it captures the host's input instead.
  - **TX1** plays to the machine's audiodev (`-M ...,audiodev=` or
    `-audio`), if one is given, through the EAC's DAC path:
    - its enable;
    - L/R averaging, which the driver sets;
    - its digital gain, 0.5 dB steps from 0xFF.

  The ADC enable and digital gain mute the capture. TX2 and TXLB move
  their buffers but are not heard. The host's audio is a sink and a
  source; it never paces the DMA.
- **CSI, LVDS, the sensor PHY, TGE, SIF, the DDR arbiter.** Storage.

`-global nvt-media.trace=on` logs every access to the media blocks; the
access logs are how the handshakes above were worked out. A run under
`-d unimp` with trace on can grow by gigabytes a minute, so write it to
`/mnt/data` or cap it (`ulimit -f`).

`qemu-boot/test-novatek-hdal.sh` (CI, both machines) boots the OpenIPC
release with `sensor=sc501ai`. It runs the release's own `load_novatek`
in a chroot of its SquashFS root, and checks that all 26 media drivers
and units bound.

With `--samples DIR` it also runs the SDK's HDAL sample programs, with
their files on an emulated SD card. `qemu-boot/build-novatek-hdal-samples.sh`
builds them from the vendor SDK (not in CI). The run checks 75 things
(108 with `--kmods` and an `ai_net` model, 122 with the detectors and
motion detection, below):
- `hd_video_record` (sensor, SIE, the IPP, H.265) and
  `hd_video_encode_only` (H.265 and H.264, from a patterned file) write
  their bitstreams. With x264 / x265 built in, and ffmpeg on the host:
  - `hd_video_record`'s tiled stream decodes without an error to the
    sensor's picture: the bars, the ramp, the box moving;
  - both of `hd_video_encode_only`'s streams decode without an error to
    the input's frames in order, each within 30 dB PSNR;
- both of `hd_video_snapshot`'s JPEGs decode as 2880x1620 pictures;
- `hd_video_capture_only` runs;
- eleven `hd_gfx_only` operations run cleanly, with these results:
  - copy, colour transform, lines, rectangles and a job list: the
    expected green, red, blue and white;
  - rotation flips the picture;
  - 3-plane to 2-plane;
  - scaling up and down from patterned inputs;
  - the 1 MiB DMA copy and 16-bit subtraction pass the sample's own
    comparisons;
- the audio samples, run with the machine's audiodev a wav file:
  - `hd_audio_capture_only` captures the test tone, 1 kHz left and
    1.5 kHz right;
  - `hd_audio_output_only` (playing that capture) and
    `hd_audio_livesound` (capture looped to playback) leave both tones
    in the wav;
  - `hd_audio_record`'s AAC decodes (ffmpeg) to them.
- with the display modules, `hd_video_output_only` (colour bars from a
  file) and `hd_video_liveview` (the sensor) put their pictures on the
  panel: QEMU's screendump of each, taken half way, shows the bars.
- with the AI modules, `ai_op`'s five operators give the results
  [AI engines](#ai-engines) lists, and `ai_net`'s ImageNet classifier
  calls the test picture a sea snake.

None of them reports an HDAL error.

The samples run against the release's own `libhdal.so`. That library is
a uClibc build, and the OpenIPC rootfs answers its `libc.so.0` with musl,
whose `mmap()` takes a 64-bit offset where uClibc's code passes 32 bits.
libhdal's first mmap, of `/dev/log_vg`, then fails, and it writes through
`MAP_FAILED`. So the samples are built with the SDK's uClibc toolchain
and run under its runtime, which the builder bundles.

The SDK's sensor samples name an IMX290 or use 1920x1080 buffers; the
builder sets them to the SC501AI's 2880x1620 mode, the only one its
driver offers.

`hd_video_encode_only` stops its encoder on `q` while its feed thread may
be about to push one more frame. That push then fails (`push_in error`,
-19 or -26), about once in ten runs here, and on silicon too. The
builder therefore joins the feed thread before the encoder stops.

With Python PIL, the run also checks the snapshots' content: the eight
colour bars, the ramp from dark to light, and the box moving between the
two snapshots.

`test-novatek.py` checks each engine's handshake at register level, in CI.

## AI engines

`kdrv_ai` drives three engines, named by the `ai@f0c60000` node's three
windows in this order:

| engine | window | SPI | does |
|---|---|---|---|
| NUE | `0xF0C60000` | 14 | vector layers: fully connected, SVM, ROI pooling, softmax |
| NUE2 | `0xF0D50000` | 13 | preprocessing: YUV to RGB, scaling, mean subtraction |
| CNN | `0xF0CB0000` | 11 | convolution, batch norm, element-wise, activation, pooling, deconvolution |

Their registers are in `cnn_reg.h`, `nue_reg.h` and `nue2_reg.h`. The
SDK's rtos tree has `kdrv_ai` as C source, and the Linux tree has it as
objects. Its `clock-frequency` has one cell an engine, 600, 480 and
600 MHz. The tree's CNN window is `0x22C` long, one register short of
the checksum counter at `0x22C` that the driver reads, so the generated
one is `0x230`.

All three are `nvt-media` IPP-style engines:
- CTRL bit 0 resets.
- Bit 1 runs a job set up in the registers. Its done is status bit 0.
- Bit 28 fires a DRAM linked list. Its status bits are job end 10, list
  end 8 and broken list 9.
- Interrupt enables and status are NUE `0x38`/`0x3C`, NUE2 `0x3C`/`0x40`
  and CNN `0x08`/`0x0C`, write 1 to clear.
- Lists start at NUE `0x024`, NUE2 `0x014` and CNN `0x040`.
- The DMA-disable registers (`0x1AC`, `0x098`, `0x1FC`) read idle in
  bit 15, which the driver polls.

The AI lists add two bits to the IPP's update command:
- **Bit 59** adds the list base register (NUE `0x1CC`, NUE2 `0x09C`,
  CNN `0x204`) to the update's value. A list's addresses are relative to
  it, so the same list can run on another buffer.
- **Bit 60** marks the job's last update. The engine runs the job, then
  carries on with the next command, so one list holds a whole network,
  a layer a job. There is no job-end command between them.

`hw/misc/nvt-ai.c` does the work of a job. The jobs run on a worker
thread with a copy of the registers, because a CNN layer can take
200 ms of host time.
- The work logs its writes. They land in guest memory with the job's
  done interrupt, when the work comes back to the main loop.
- A reset or a loadvm detaches a job in flight, which then leaves
  memory untouched.
- A job running at a migration runs again on the destination, from the
  registers it started with.
- Intermediate values saturate rather than wrap, and a job too large to
  run at once logs as unimplemented.

**NUE, fully connected** (`kdrv_ai_tran_nue_fc_parm`) is mode 0 (SVM)
with kernel 1 in dot mode.
- The inputs (`0x00C`) are 8 or 16 bits, MODE0 bit 12, signed by bit 4.
- The weight rows (`0x014`) are 12-bit, an LSB-first bit stream. With
  MODE1 bit 21 (quantised) they are 8-bit.
- 8-bit operands enter the 12-bit multiplier top-aligned, so each one
  counts x 16. `ai_op`'s 8-bit inputs and weights therefore come out
  x 256, and the classifier's u8 inputs against 12-bit weights x 16.
  16-bit operands (the face network's embedding layer) enter by their
  top 12 bits.
- The weight row length is at `0x040` [13:0], the rows at [28:16], and
  the input vectors at `0x048`.

The kernel's result is

    k1[o][j] = ((sum_k ((x'[o][k] * w'[j][k]) >> fshift)) * gain
                >> gshift) + offset

where gain and its shift are at `0x04C`, and the offset (21 bits,
signed) and fshift, the SDK's weight shift, at `0x050`. fshift applies
to each product before the sum. Output paths 1 and 3 (MODE1 [9:8])
write `k1` as s32. Path 0, with result mode 1 (MODE1 [5:4],
`SUB_RHO`), is the classifier's:

    r = (k1 >> rho_fmt) - rho[j]

- `rho_fmt` is `0x058` [19:16] + 3.
- `rho` is a 12-bit value a row, from the same kind of stream at
  `0x01C`. With MODE1 bit 11 it is the constant at `0x058` [11:0].
- Then comes the ReLU (MODE1 bit 25, its slope at `0x054`), and its
  shift, `0x054` [23:20] (bit 16 makes it a left shift).
- The output is 16-bit (MODE0 bit 16) or 8-bit (r >> 8), signed by
  MODE0 bit 8, one a row, a vector's rows together.

The detectors' heads use two more NUE modes. Both read a planar map: W
and H at `0x040` [13:0] / [28:16], C at `0x044` [27:16], the element
type from MODE0 bits 4 / 12. The input scale-shift at `0x0F0` is
identity in every model.
- **Permute** (mode 4, channel first) writes each pixel's C elements
  together, the pixels `0x034` bytes apart. The converter stripes wide
  maps by channel, so the gap between pixels belongs to other jobs and
  is left alone. The shift after it is `0x1C4`.
- **Softmax** (mode 6) works on each pixel's channels, in groups of
  `0x0EC` [23:16]. The input shift ([7:4], bit 0 left) brings the
  logits to Q.8, at most 12 bits. The probabilities come out in Q.15
  through the output shift ([15:12], bit 8 left), and are written the
  same way as permute: each pixel's channels together. The two permutes
  that follow it in every model only undo that layout.

**NUE2, preprocessing** (the 528 register layout NA51089 uses). FUNC
`0x004` bit 0 turns on YUV to RGB, bit 1 the mean subtraction, bits
[13:12] pick the input (0 NV12, 3 RGB planes), and bit 14 makes the
output signed. The pipeline is:
1. YUV to RGB, full-range BT.601 with chroma replicated;
2. bilinear scaling from `0x044` to `0x058`, corners to corners. The
   step `kdrv` programs is (in − 1) / (out − 1);
3. the mean subtraction: a DC mean a channel (`0x060`), or planar
   (`0x010`, R G B bytes a pixel, its size at `0x05C`, replicated by
   `0x060` [25:24]);
4. the scale-shift at `0x07C`. A scale of 0 means no scale-shift: the
   classifier uses it, and wants plain pixel less mean;
5. a clamp to the output's range.

The outputs go to `0x018..` with line offsets at `0x030..`.

**CNN** (`cnn_reg.h`; the driver's mapping is `kdrv_ai_tran_cnn_parm`).
A job is one layer, or one stripe of a layer, described entirely by its
registers.
- **Input.** `0x010`, with its line and channel pitch at `0x020` /
  `0x024`. It is W x H x C at `0x05C` / `0x060`, 8 or 16 bits (IO
  `0x064` bit 5), signed by bit 4. Bit 0 is image mode, which the 7x7
  stem uses.
- **Convolution** (MODE `0x004` bit 12).
  - The kernel comes from `0x068` [3:0] (11x11, 7x7, 5x5, 3x3, 1x1, the
    1-D shapes, 9x9), the stride from [5:4], the output channels
    ("sets") from [19:8].
  - Each padded side (`0x06C` [3:0]) pads half the kernel with zeros.
  - The weights at `0x030` are 12-bit signed, an LSB-first bit stream
    ordered [set][channel][ky][kx]. `0x0D4` holds their length in bits.
  - The accumulator is wide, and 16-bit inputs lose 8 bits first.
- **Scale-shift stages.** Six stages (`0x0A8..0x0BC`), each
  `(x * scale) >> shift`, or `<<` with bit 0. The scale is [27:12], the
  shift [8:4].
- **The chain.**
  1. conv;
  2. `0x0AC`;
  3. BN (MODE bit 8), or without BN the bias;
  4. element-wise (bit 11);
  5. `0x0B0`;
  6. a clamp to 16 bits and the activation (bit 9; a ReLU with a leaky
     slope at `0x090` / `0x094`, whose bit 16 negates the input first).

  A job without a convolution but with BN or element-wise starts at
  `0x0AC` with its input as it is.
- **Bias.** Without BN the record at `0x038` is 2 bytes a set: a signed
  12-bit bias in the low bits, `<< CONV_SHIFT_B` (`0x06C` [12:8]). It is
  added after `0x0AC`, whatever the output's width.
- **Element-wise.** The second map is at `0x03C` (pitches `0x028` /
  `0x02C`, type `0x084` [1:0]) and goes through `0x0A8`. The result is
  `((a * coef0) >> shift0) + ((b * coef1) >> shift1)`, or the larger of
  the two terms (MODE [3:2] = 2), then `>> outshf`. The coefficients are
  `0x088`, the shifts `0x084` [8:4], [16:12] and [24:20].
- **BN.** The record at `0x038` is 8 bytes an output channel: alpha
  [23:0] signed 24-bit, mean [35:24] and beta [47:36], both signed
  12-bit.
  The stage is

      y = ((x - (mean << m)) * alpha >> a) + (beta << b)

  with m, b and a in `0x078` [4:0], [12:8] and [25:21].
- **Two outputs.**
  - out0 (`0x044`, IO bit 8) is act0, then `0x0B4`, then >> 8 if it is
    8-bit (a 16-bit out0 keeps the fraction bits).
  - out1 (`0x048`, bit 13) is act1, then `0x0B8`, then the pooling
    (bit 10), its shift (`0x09C` [6:4]), then `0x0BC`.

  The datapath carries 8.8 fixed point, which out0 drops at its end and
  out1's scale already folds in. A 16-bit output keeps 8 fraction bits
  more than an 8-bit one on either path. A job without convolution feeds its
  input straight to the outputs: a pooling layer.
- **Pooling.**
  - Local pooling is max or average (`0x098` bit 0), 2x2..5x5
    ([9:8]), stride 1 or 2 (bit 4), pads of half the kernel, ceil or
    floor (bit 1).
  - Global pooling (MODE bit 5) is `sum * mul >> (shift + 10)` (`0x09C`
    [18:8], [24:20]).
- **Output layout.** Outputs are clamped to 8 or 16 bits (IO bits 9 /
  12), signed by IO bits 6 / 7 / 14. They land planar, with the pitches
  at `0x04C..0x058`.
- **Deconvolution and scale-up** (MODE [1:0] 1 / 2) run alone, into
  out0 through `0x0B4`, with no 8.8 drop.
  - Deconvolution spreads the map out with stride − 1 holes between
    pixels and adds pad rows and columns (counts 0..3, `0x0A0`). The
    holes and pads take the pad value. The convolution that completes it
    is jobs of its own.
  - Scale-up repeats each pixel `rate` times each way (`0x0A4`).
- **Stripes and depthwise layers.** A tall layer runs as row stripes
  that overlap and recompute. A depthwise layer runs as a job a channel,
  one input and one output channel each, often in image mode. Each job
  is self-contained, so neither needs anything of its own.
- **PReLU.** The compiler builds it from three jobs. ReLU(−x) is an
  activation with ACT bit 16 set; ReLU(x) is another activation; an
  element-wise job adds them, its BN scaling the first by the negated
  slopes (negative alphas). In a depthwise layer, a job a channel, it is
  instead that job's leaky slope.

Most of this is the register map. The weight, BN and bias layouts, the
8.8 datapath and where the scales sit are not in the SDK. They were
fitted to the SDK's networks:
- **Weights.** The 12-bit weights correlate between neighbouring taps
  in that order only.
- **Ranges.** With these choices every job of the classifier keeps its
  int8 output in range, and the classifier names its test picture.
- **Bias.** Its placement is fitted, not read from the SDK. It goes after
  `0x0AC`: before it, the person and face detectors find nothing, and
  the face recognition network's embeddings stop telling people apart.
  An earlier width-dependent rule (before `0x0AC` for 16-bit outputs) had
  been fitted while BN alpha was read unsigned. With alpha signed, the
  person / vehicle detector finds all three vehicles in the dashcam
  frame, not one, with the bias after `0x0AC` everywhere.

`-global nvt-media.ai-dump=DIR` writes each AI job's registers to
`DIR/jobs.bin`, and snapshots RAM at the first CNN and NUE jobs and at
each softmax (a detector head's logits). The
fitting was done offline against those dumps, and the C model matches
that reference bit for bit.

Not modelled, and logged as unimplemented:
- on the CNN: element-wise multiply, pre-activation, tanh and sigmoid,
  FCD weight compression, CONV_SHIFT_ACC, channel stripes, out0's
  intermediate mode and row offset, batches;
- on the NUE: ROI pooling, reorg, anchors, permute height first, the
  other SVM kernels;
- on NUE2: pad, HSV and rotation.

Some details are guesses that the networks do not pin down:
- rounding, which is half up at each right shift;
- the YUV to RGB matrix;
- NUE2's output channel order (R first);
- the element-wise main input's scale;
- the softmax's exact exp.

**Modules.** As with the display, the OpenIPC release leaves the AI
modules out and its toolchain has them (`sdk/kmod/hdal`). The SDK's init
loads them in this order:
1. `kdrv_ai ai_net_supported_num=5` (the AI turnkey's `ai_ko.sh`; the
   face detector runs as process 2, which the SDK's 2 refuses);
2. `kflow_cnn`, `kflow_nue` and `kflow_nue2`;
3. `kflow_cpu` and `kflow_dsp`;
4. `kflow_ai_net`.

**Samples.** `ai_op` (`samples/ai_op`) runs single operators through
`vendor_ai`:
- operator 0 is a fully connected layer on the NUE;
- operators 1..4 are NUE2's preprocessing of a 512x376 NV12 picture,
  which the builder copies from the SDK's `ai_pattern`.

**Networks.** `ai_net` runs a model through `kflow_ai_net` as one
linked list per engine. A model carries its chip id (`nn_verinfo.h`),
and the release's `kflow_ai_net` accepts only id 4 on chip `0x7021`.
The SDK v1.01.002's `ai_pattern/para/nvt_model.bin` has id 1
(`NN_CHIP_CNN25_A`, the NT98520), so on this chip it is refused:

    nvt_ai_chk_vers() chip(0x00007021) not support id(0x00000001)

SDK v1.01.007 ships the same network built for this chip (id 4). It is
BN-Inception, ImageNet's 1000 classes, 17 MB, and runs as:
- NUE2's preprocessing to 224x224;
- 93 CNN jobs;
- four NUE FC jobs;
- softmax on the CPU.

The v1.01.007 AI turnkey package has id-4 detectors as well (`pdcnn`,
`cddcnn`, `pvdcnn`, `frcnn`). Under QEMU, with the release's modules and
the v1.01.002 `ai_net`, the classifier calls `ai_pattern`'s picture what
it is:

    1. no=65, label=sea snake, score=0.514828
    2. no=58, label=water snake, score=0.286910
    3. no=67, label=diamondback, diamondback rattlesnake, ..., score=0.064323

A run takes about 2.5 s of host CPU.

**Detectors.** The v1.01.007 AI turnkey package has id-4 detector models
(`CNNLib/para`). SDK v1.01.007 has their samples and their closed
post-processing libraries (`vendor/third_party`), whose anchors and NMS
run on the CPU. Its user-space `vendor_ai2` checks for a `kdrv_ai`
version the release modules do not report. So the builder links
v1.01.007's samples and detector libraries with v1.01.002's
`vendor_ai2`, which works.

| sample | model | jobs | finds |
|---|---|---|---|
| `ai2_pdcnn_sample` | `pdcnn` (people), `public` (7 classes, to reject boxes) | 1752 a picture | people |
| `alg_pvdcnn_sample` | `pvdcnn/nvt_model_prune37`, `public` | about 300 | people (class 1), vehicles (2) |
| `alg_fdcnn_ai2_sample` | `fdcnn_method2_light` | about 600 | faces, five landmarks each |
| `cddcnn_sample` (ours) | `cddcnn` (pets), `public` | about 3000 | cats and dogs (category 1) |
| `frcnn_sample` (ours) | `frcnn` (face recognition) | about 3600 a face | a 128-value embedding |

`cddcnn` has a library (`cddcnn_ai2`) but no sample, so
`qemu-boot/novatek-cddcnn-sample.c` is one. It follows the turnkey
guide's flow (2.1.3, 2.2.3) and the other samples' API use: the picture
scaled to the network's 1024x576, `cddcnn_process`, then
`limit_fdet_process` with the `public` network and class 1. It writes
both: the network's proposals to `cdd_raw_results.txt`, those limit_fdet
keeps to `cdd_test_results.txt`. `cddcnn_version_check` needs each
model buffer 32-byte aligned, or it rejects the model (`fmt_main 0`).

`frcnn` has neither library nor sample, in any SDK release: the turnkey
ships the model alone. The model's own tensor descriptors say what it is:
input `data`, 112x112x3; output `BatchNormalization_fc1_Y`, 1x1x128,
s16 with 13 fraction bits. That is the InsightFace / ArcFace layout of a
face embedding (a MobileFaceNet: depthwise layers, PReLU, a global
pooling, a fully connected layer and BN).
`qemu-boot/novatek-frcnn-sample.c` runs it through `vendor_ai_net`
alone, on 112x112 NV12 faces aligned to the ArcFace template by their
five landmarks. It writes each face's 128
values. The turnkey's models start with 32 bytes of their own; the
detector libraries step past them, and the sample does too, or
`kflow_ai_net` reads chip id 0 and refuses the model.
`alg_fdcnn_ai2_sample` asks for a second DDR (`NN_USE_DRAM2`), which the
NT9856x does not have; on silicon it fails as here. The builder turns it
off.

On their pictures under QEMU:
- **pdcnn** finds the two large people in a composite of OpenCV's
  `messi5` and `basketball1`, both again at the smaller size, and the
  facing person in `basketball1`. It misses the one turned away.
- **pvdcnn** finds the large Messi and the facing man (0.95), and in the
  SDK's dashcam frame the truck, the white car ahead and the small car on
  the left (0.97..0.99).
- **fdcnn** finds the astronaut's face, its landmarks on the eyes, nose
  and mouth corners, and the facing man's face in `basketball1`.
- **cddcnn** proposes scikit-image's `chelsea` (a cat, 0.98) and the dog
  cropped from darknet's `dog.jpg` (0.92), and nothing in the dashcam
  frame. In the whole `dog.jpg` (dog, bicycle and truck) it proposes
  nothing. limit_fdet then rejects both pets: its classifier puts the
  cat in class 5 (0.46) and the dog in class 5 (0.81), not class 1 at
  0.7. That classifier confirms pvdcnn's people as class 0 at 0.93..0.98,
  so the rejections are taken as the model's, not the emulation's.
- **frcnn**, on faces from the `face_recognition` project's example
  photos (three of Obama, two of Biden, one each of two other men), gives
  cosine similarities of 0.68..0.76 between Obama's photos and 0.82
  between Biden's; between different people they are −0.05..0.18, and a
  grey square scores below zero against every face. A mirrored photo
  scores 0.96 against its original.

A picture takes 4..11 s of host CPU. The job runs on the vendor's own
code end to end: the graphics engine scales the picture, NUE2 and the
CNN and NUE run the network, and the vendor's library post-processes on
the CPU.

**Tests.** `test-novatek-hdal.sh --kmods DIR` loads the AI modules and
checks that they load and `kdrv_ai` binds the engines. CI runs this.
With `--samples` too, it runs `ai_op`'s five operators and checks them:
- operator 0's 10240 outputs are each 4480, which is
  sum_k (k & 7)² over the sample's inputs and weights;
- operator 2 is 384x282 R, G and B planes;
- operators 3 and 4 equal operator 1's 512x376 planes less the planar
  mean (128) and the DC means (128, 127, 126);
- operator 1's channels are the test picture's colours.

When the samples carry the v1.01.007 model
(`build-novatek-hdal-samples.sh --ai-model`), it also runs `ai_net`.
Its top class must be the sea snake, at 0.3 or more. With the detectors
(`--sdk7` and `--turnkey`), it runs the five samples on their pictures.
Their 30 MB of models and pictures is too large for the initramfs, so
the test packs it as a SquashFS on a second SD card (`-drive
if=sd,index=1`), and the guest copies it to the vfat card the samples
read. It checks that each finds, near where they are:
- **pdcnn**: the three people;
- **pvdcnn**: the two people and the three vehicles;
- **fdcnn**: both faces, with the astronaut's eyes within 12 pixels;
- **cddcnn**: the cat and the dog among the network's proposals, as
  category 1 at 0.8 or more, and no proposal in the dashcam frame;
- **frcnn**: all nine faces embedded, under 1% of the values at ±4. Each
  photo of Obama and Biden is nearest another of the same man; the
  mirrored Obama is nearest its original; and every pair of one man
  scores above every pair of two. The grey square scores below every
  pair of one man.

`test-novatek.py` checks, at register level:
- the idle bits;
- a fully connected job;
- an NV12 picture scaled and mean-subtracted;
- a CNN list of two jobs fired by bit-60 updates, one value based by
  bit 59, whose list end runs neither job again;
- the compute against its own arithmetic:
  - a CNN 3x3 layer (12-bit weights, BN, ReLU, out0);
  - 3x3/2 max pooling;
  - global average pooling;
  - NUE output path 0 (rho, 16 bits), with 12-bit rows of an odd
    length;
  - NUE2's scale 0;
  - scales at their largest saturating;
  - for the detectors: a convolution with a plain bias, deconvolution,
    scale-up, the element-wise add of two s16 maps, and NUE's permute
    and softmax;
  - for face recognition: ACT bit 16, BN with negative alphas, a global
    average into a 16-bit out1, and NUE FC on 16-bit inputs.

## Motion detection

The MDBC (`hw/misc/nvt-md.c`, an `nvt-media` engine) is at `0xF0C10000`,
SPI 46, under `kdrv_md.ko` (`mdbc_reg.h`). Like the display and AI
modules, the release leaves it out and OpenIPC's toolchain has it. Its
node must be named `md@f0c10000`: probe looks its clock up by the
device's name.

A trigger (control bit 1) takes a frame:
- 8-bit Y and NV12 CbCr, 60x60 to 960x1022;
- the background model, in three DRAM buffers that only the engine
  reads. Software passes them back in place, and the emulation lays them
  out its own way.

It writes the foreground as one bit a pixel, row-major, LSB first, 1 for
motion. Then it raises frame end (status bit 0, write 1 to clear).
`kdrv_md` waits for that with no timeout, so a missing interrupt hangs
the process.

The SDK does not describe the algorithm. The parameters are named after
SuBSENSE (LBSP descriptors, a sample model, minimum matches, update
rates, neighbour diffusion, ghost removal). The emulation is a reduced,
deterministic version of it:
- INIT seeds each pixel's samples with its own value.
- Otherwise a pixel is foreground when fewer than `BC_MIN_MATCH` samples
  are within `D_COLOUR` in Y, `D_LBSP` in descriptor Hamming distance and
  `UV_THRES` in chroma. Each ROI has its own thresholds.
- Background pixels refresh a random sample at a rate of 1 / `BC_MIN_T`,
  and with neighbour updates on, a neighbour's sample too.
- With ghost removal on, a pixel that stays foreground for more than
  `BC_MAX_FG_FRM` frames updates as if background.
- The foreground then goes through the four 3x3 stages. A stage's select
  bit adds pixels that have enough neighbours set; a clear bit keeps only
  those. A threshold of 0 skips the stage.

The random numbers are a 15-bit LFSR whose state is kept in the RND
register, hashed for each draw (its successive states are only shifts of
each other). A run therefore always gives the same bitmaps. With
`DBG_RND_EN` every frame starts from the same seed. RND, the frame
counter and the luminance difference are read-only. Control bit 0 resets
the engine: a running job and the status go, and the interrupt enables
(written before it) stay.

The SDK's `alg_md_sample` runs the engine and then `lib_md`'s alarms:
- a global alarm when over half the picture moves;
- a sub-region alarm over rows 0..89.

It reads 160x120 frames from `MDBCP/DI/pic_NNNN`, and the SDK ships none.
`build-novatek-hdal-samples.sh` writes 11 of them: a still textured
scene, with a bright 120x100 block moving through frames 4..7. With
`--kmods` and `--samples`, `test-novatek-hdal.sh` loads `kdrv_md` and
checks that it binds. It then checks that both alarms come in frames
4..7, and in no other frame.

`test-novatek.py` checks at register level:
- INIT reports no motion and raises frame end;
- the still scene then matches its model;
- a bright block is foreground, and nothing 3 pixels off it is;
- the frame counter counts;
- an erosion stage gives exactly the previous bitmap's pixels with all 8
  neighbours set;
- with ghost removal, a lasting change is absorbed (BC_MIN_MATCH 4);
- a reset straight after a start leaves no frame end and no status.

## Display

The display path is the IDE (`hw/display/nvt-ide.c`) at `0xF0800000`,
SPI 48, scanned out to QEMU's graphic console. It drives LCD panels or
MIPI DSI; the chip has no TV or HDMI output. The EVB's panel is a 3.5"
HX8238D on 8-bit serial RGB, 320x240 at about 61 Hz. Each line is 960
clocks, one colour each, and the IDE's windows are 960 wide too. Out of
a serial-RGB device the console therefore shows every third column, at
320x240.

**Modules.** The OpenIPC release leaves the display modules out, but its
toolchain (`toolchain.novatek-nt9856x.tgz`, `sdk/kmod/hdal`) carries
them, built for the release kernel. The SDK's init script loads them in
this order:
1. `nvt_ide`;
2. `kdrv_videoout`;
3. `kflow_videoout`;
4. `nvt_dispdev_panel`;
5. the panel, `disp_if8b_lcd1_pw35p00`.

The generated device tree has their nodes, from the EVB's dtsi files:
- `ide@f0800000`, whose name the clock driver finds its clocks by;
- `dispobj` and `dispdev`;
- `/display`, the panel's GPIO wiring (`lcd_ctrl` 1: its registers are
  bit-banged on D_GPIO 3 / 4 / 5).

**The IDE** (`ide_reg.h`, as the closed `ide2.o` / `ide_idec.o` drive
it; the vendor U-Boot has the same driver as source):
- CTRL `0x000`: layer enables, the device type [10:7], on (bit 27) and
  load (bit 28). DMA idle (bit 2) reads 1 while the DMA is disabled or
  the engine is off.
- Most registers take effect at the next frame. The driver sets load,
  then waits with no timeout for frame end (INT `0x0FC` bit 23, enable
  bit 7, write 1 to clear). Its handler wakes the waiter only if load
  reads 0 by then. So each frame latches the registers, clears load,
  and only then raises frame end.
- Frames run at 60 Hz while the engine is on. With it off, the driver
  sleeps a frame instead of waiting.

Scanout composes the background (`0x0BC`, YCbCr) and then the layers:
video 1, video 2, OSD 1. Each layer's buffer is scaled into its window,
nearest neighbour, with mirroring. The formats:
- video: 9 and 10, YUV 4:2:2 and 4:2:0 with interleaved CbCr;
- OSD 1: the 8-bit palette (256 entries of Y, Cb, Cr and alpha at
  `0x200`), ARGB8888, ARGB4444 and ARGB1555.

OSD 1 blends by its alpha select: opaque, global alpha, or the pixel's.
Its colour key is honoured. Other formats log as unimplemented.

**MIPI DSI** (`hw/display/nvt-dsi.c`, `0xF0840000`, SPI 50) is for the
DSI panels. The toolchain has three:

| panel | lanes | mode | size |
|---|---|---|---|
| nt35510 | 2 | command | 480x800 |
| tb093 | 4 | video | 320x1280 |
| ut35067a0 / ili9488 | 1 | command | 320x480 |

`nvt_dsi.ko` loads before the panel's module; nothing else changes. The
driver is the vendor U-Boot's `nvt-na51055/dsi` (source), closed in the
kernel. Every wait in it is on an interrupt with no timeout: INTSTS
`0x84` with enables at `0x80`, write 1 to clear.

The panels send their set-up commands as low-power data, one byte and
one escape-done interrupt at a time. The host then goes automatic and
streams the IDE's frames until the driver closes, which waits for a
frame end. So the model does this:
- escape triggers (`EN` bits 4..7) and the clock ULPS trigger (bit 9)
  clear themselves and raise their done bits (25..28, 24);
- in manual mode, TX on (bit 0) sends `CMD0..`, clears itself and raises
  TX done (bit 0). A read gets no bytes back; no panel reads;
- in an automatic mode, TX on raises frame end (bit 1) at 60 Hz until
  TX goes off, which raises TX done.

The picture is still the IDE's, which with device 0xD (DSI) is scanned
out 1:1, at the panel's size. `-global nvt-dsi.trace=on` logs the
panel's low-power packets.

`test-novatek-hdal.sh --kmods DIR` loads the modules and checks
that the IDE, dispobj and dispdev bind and the panel driver comes up.
CI runs this, with the kmods extracted from the toolchain. With
`--samples`, `hd_video_output_only` and `hd_video_liveview` run too, and
the screendumps are checked. `--panel nt35510 | tb093 | ili9488` uses a
DSI panel instead, with `nvt_dsi` loaded before it, and expects its size.
`test-novatek.py` checks the frame
handshake and a screendump of a video and an OSD layer, and the DSI's
escape, ULPS, manual and automatic transfers.

## Ethernet

`nvt-eqos` is the 16 KiB block at `0xF02B0000`: a Synopsys DWC Ethernet QoS
(GMAC 4.x) core at 0x0000 (MAC), 0x0C00 (MTL) and 0x1000 (DMA, channel 0 at
0x1100), the Novatek glue at 0x3000 and the embedded PHY's analog block at
0x3800. The kernel driver is the Synopsys reference driver (`DWC_ETH_QOS`,
built in as `ntkimethmac`), and these are the points it depends on:

- **MDIO.** HW_FEATURE0 must advertise it (SMASEL, bit 5); without it the
  driver prints "MDIO is not present" and runs with no PHY. It then scans
  for the first address whose BMSR is neither 0 nor 0xFFFF. The model has a
  generic 100 Mb/s PHY at address 1 with the link up, and phylib binds
  genphy to it.
- **Queues.** HW_FEATURE2 reads 0: one Tx and one Rx queue, so only DMA
  channel 0 is modelled.
- **Rx state.** `DWC_ETH_QOS_yinit()` polls DMA_DEBUG_STATUS0 until RPS0
  reads 3 ("running, waiting for a packet") and re-initializes the core
  until it does.
- **ETH_ISR.** The ISR first reads ETH_ISR, glue register 0x3008, and
  returns IRQ_NONE unless its low 6 bits are set. A model that leaves it 0
  gets an interrupt storm. It reads the core's interrupt line in bit 0.
- **Descriptors.** Descriptors are four words with OWN in DES3 bit 31. Tx
  write-back keeps only FD and LD in TDES3, whose low half is error flags in
  that format. Rx write-back carries the length without the FCS, because the
  driver does not strip it.
- **Rx interrupts.** The driver sets IOC on only some Rx descriptors and
  leaves the rest to the Rx watchdog (RIWT). RI is raised for every frame
  while RIWT is set.

Whether the board is wired for RMII or the embedded PHY comes from the
pinmux (`eth` = 0x22 on the EVB: the embedded PHY), and the driver's
power-on writes to the 0x3800 block are kept as storage.
`test-novatek-boot.sh` takes a DHCP lease from `-nic user` and fetches a
1 MiB file over HTTP, checksummed.

The vendor U-Boot's driver (`eth_na51089.c`) runs on the same model
unchanged: `ping`, `dhcp` and a 1 MiB `tftp` from `-nic user,tftp=dir`,
its `crc32` matching the host's. Reaching its prompt takes a key already
waiting at "Hit any key to stop autoboot", since the SDK build has
`bootdelay` 0 and keeps its environment nowhere, so nothing can be saved.
QEMU's 16550 used to lose that key, which is what
`qemu/patches/0003-serial-fcr-rx-reset-accept-input.patch` fixes. A byte
that arrives before U-Boot sets up the UART sits in RBR and the backend
stops polling. U-Boot then resets the receive FIFO and only reads LSR, and
QEMU never asked the backend for more input: the vCPU spinning on LSR made
no main-loop events, so the line stalled until Linux read RBR.
`qemu-boot/test-novatek-uboot.sh --flash nor.bin` starts QEMU paused
with the console on a socket, queues the key, and runs the network and SD
card checks.
It needs a vendor image, so it is not in CI. `test-novatek.py` covers the
UART fix with a small echo guest instead.

## Running

```bash
cd qemu-boot && curl -sL \
  https://github.com/OpenIPC/firmware/releases/download/latest/openipc.nt98562-nor-lite.tgz \
  | tar xz && cd ..
bash qemu-boot/run-nt98562.sh               # SOC=nt98566 for the 128 MB part
FLASH=nor.bin bash qemu-boot/run-nt98562.sh # a whole flash image
SOC=nt98566 FLASH=nand.bin bash qemu-boot/run-nt98562.sh   # FW98566A.bin, NAND
bash qemu-boot/test-novatek-uboot.sh --flash nor.bin       # vendor U-Boot: net, SD
bash qemu-boot/test-novatek-nand.sh --flash nand.bin       # NAND: U-Boot, UBIFS
bash qemu-boot/test-novatek-boot.sh --soc nt98562
bash qemu-boot/build-novatek-test-kernel.sh --out testk    # see USB, PWM
bash qemu-boot/test-novatek-usb.sh --soc nt98562 --kernel testk/uImage.nt9856x-test
bash qemu-boot/test-novatek-pwm.sh --soc nt98562 --kernel testk/uImage.nt9856x-test
python3 qemu/tests/test-novatek.py
```

The release rootfs is SquashFS, and this kernel has no RAM disk driver, so
the boot matrix's `-initrd rootfs.squashfs root=/dev/ram0` does not apply.
`run-nt98562.sh` lays the release into an 8 MiB NOR image
(`qemu-boot/nor-<soc>.bin`, kept between runs) and mounts root from it.
`test-novatek-boot.sh` boots the kernel into a small initramfs (busybox and
an `/init` script) with the same NOR attached. The script checks the device
tree, the clocksource, the pinctrl, GPIO, RTC and watchdog drivers and the
MTD. It reads, erases and programs the flash, reads and writes an SD card
and a vfat on it, takes a DHCP lease and fetches a file over HTTP, then
runs `reboot -f` under `-no-reboot`. Any run script takes a card the same
way: `bash qemu-boot/run-nt98562.sh -drive if=sd,format=raw,file=sd.img`.

## Not modelled

- The real loader: it trains DDR from its ConfigRAM table with closed
  libraries (`Reset_560.a`). Its NAND path also skips bad blocks; the
  stand-in reads the FDT and U-Boot at their nominal offsets.
- SPI NAND other than the two Winbond parts, 4 KiB pages, bit errors and
  bad blocks, and the controller's own Reed-Solomon ECC (used only when a
  part's on-die ECC is off).
- UHS / 1.8 V signalling and tuning, eMMC HS200 and boot partitions (the
  eMMC has none unless built with `-device emmc,boot-partition-size=`),
  SDIO (non-memory) cards, and card hot-plug.
- The EQoS features the drivers leave off: checksum offload, TSO,
  timestamps, more queues.
- The FOTG200's device (gadget) side, and OTP contents other than blank
  (no trims, keys or secure boot).
- I2C's DMA (VD-synchronised) transfers, and sensors other than the two
  SmartSens IDs, with nothing behind their registers (no MIPI, ISP or
  frames).
- PWM micro-step (motor) mode, the cycle counters (CCNT), and the clock
  generator's real PWM dividers (cycle runs are timed at a nominal 30 MHz).
- The Novatek timer's other cores (its DST routing and their status
  words are storage, and every interrupt goes to SPI 0).
- The RSA engine's CRC32 key checks, the hash engine's PIO mode, and
  OTP key sets other than blank (the key manager hands out zeros).
- The EAC's analog gains, ALC and filters (the capture is the tone or
  the host's input as is), and TX2 / TXLB output. H.26x pictures are real
  only with x264 / x265 built in, and tiled streams are intra only. The
  IPP applies no DCE / IPE corrections and only IME output path 1,
  and the sensor is a test pattern. The graphics engine has no quarter
  turns, palettes, colour keys, blending of two images or LUTs, and there
  is no affine engine. The SIE
  ignores the sensor's real timing (fixed 30 fps). DIS, VPE and IVE
  are not wired. MD's linked lists run as the other engines' do but are
  untested; its checksums read 0, and its background model is not the
  silicon's (below).
- AI: the CNN's tanh / sigmoid, pre-activation, FCD compression,
  channel stripes and batches; NUE's ROI pooling, reorg, anchors and
  other SVM kernels; NUE2's pad, HSV and rotation. Rounding, NUE2's
  colour matrix and the element-wise main input's scale are guesses.
- Display: DSI reads (bus turnaround returns no bytes), the IDE's 1/2/4-bit
  palettes and ARGB8565 / 4565, its colour pipeline (gamma, CST), face
  boxes and lines, and `/dev/fb0`. The release kernel has no
  `CONFIG_FB`, so `nvt_fb.ko` does not load.

## NT98533 (NS02501)

`nt98533` (`qemu-system-arm`) is Novatek's NS02501 camera die, sold as
NT98532 and NT98533: two Cortex-A32s (ARMv8 cores that run only AArch32,
with a 32-bit ARMv7 kernel), a GIC-400, the ARMv8 generic timer, and
256 MB of DDR on the EVBs. Its peripherals are NA51089's, at NA51089's
addresses: U-Boot's `arch-nvt-ns02501_a32/IOAddress.h`, the SDK's
`cfg_53x` board trees and the datasheet agree. So the NT9856x models
serve it unchanged. QEMU has no Cortex-A32; the machine uses a Cortex-A7,
which runs the same code. The chip ID is `0x5521` (`CHIP_NS02501`).

### Memory map

Where it differs from NA51089's (`cfg_532_IPC_NOR_SQUASHFS_BD_EVB`):

| Block | Base | IRQ | Model |
|---|---|---|---|
| GIC-400 | `0xFFF01000` (dist), `0xFFF02000` (CPU) | | `arm_gic` |
| Generic timer | | PPI 13 secure, 14, 11, 10 | the A7's, 12 MHz |
| Core communicator | `0xF0090000` | 59 | storage; `+0x100` releases the second core |
| WDT | `0xF0050000` | 75 | `nvt-wdt` |
| RTC | `0xF0060000` | 76 | `nvt-rtc` |
| I2C 1 / 2 / 3 | `0xF0220000` / `0xF0350000` / `0xF0360000` | 10 / 41 / 42 | `nvt-i2c` |
| USB 2.0 | `0xFF600000` | 27 | `nvt-usb` (the tree's own address) |
| UART 1 / 2 / 3 | as NA51089 | 43 / 44 / 45 | 16550, all 48 MHz |

The timer, GPIO, SPI NOR / NAND controller, SDIO 1–3, Ethernet, PWM,
crypto, hash, RSA and OTP are NA51089's, at the same addresses and
interrupts. The Novatek timer's dividers run from 3 MHz, as on NA51089:
both read 1 MHz at reset. The media engines, display, audio and the
HR timers (`0xF00C0000`, `0xF0150000`–`0xF01E0000`) are not instantiated
and read as zero.

### Boot

The flash boot is NA51089's stand-in. The SHMINFO layout and the
`nvt_memory_cfg` regions are the same (`ns02501_ldr`), and U-Boot is at
`/nvt_memory_cfg/uboot` = `0x0F940000`. The SDK's NOR layout puts the FDT
at `0x10000` and U-Boot at `0x50000`, plain.

- **Secure SVC.** Everything runs there, the kernel included: the CPUs
  have EL3, and `-kernel` enters the kernel in secure SVC as U-Boot
  does. U-Boot's `lowlevel_init` goes to monitor mode, sets `SCR.NS` and
  clears CNTVOFF, which QEMU allows only with EL2 present. So the A7
  keeps its virtualization extensions. The A32 has no EL2, and on
  silicon that write is fine.
- **GIC.** It has no security extensions here; with everything secure,
  they would change nothing the software sees.
- **The second core.** On silicon it waits in the loader until
  `CC_CPU2_CPU1_CMDBUF_REG1` (`0xF0090100`) is non-zero, then jumps there
  (`bl_lowlevel_core2_jump()`). The kernel's `cpu-release-addr` is that
  register; the kernel writes `nvt_secondary_startup` into it, then
  releases the core from its own pen. Here the core stays off until that
  write, which starts it at the address. That serves `-kernel` too.
- **CNTFRQ.** The tree's timer says 12 MHz, which the kernel uses, and
  the model's counter runs at 12 MHz. The loader writes CNTFRQ = 3 MHz
  (`CPU_TIMER_FREQ`) before it jumps to U-Boot, and U-Boot's
  `CONFIG_SYS_ARCH_TIMER` delays read CNTFRQ. On silicon one of the two
  is off by four. The direct Linux boot path (`bl_linux.c`) writes 12 MHz.
  The stand-in leaves CNTFRQ at 12 MHz.
- **BCL1 LZMA.** The loader finds its LZMA decoder at mask ROM `0xBFD0`
  here (`0x7FD0` on NA51089); the format is the same (see Flash boot).

`qemu-boot/build-ns02501-sdk.sh --sdk … --toolchain …` builds the SDK's
U-Boot (`nvt-ns02501_a32_nor_defconfig`), kernel
(`ns02501_ipc_a32_evb_defconfig_nor_release`, with its modules) and the
board tree, and lays them out as a 16 MiB NOR. The NT9856x SDK's
`arm-ca9-linux-gnueabihf-8.4` builds all three; the kernel's version
header needs bash. The SDK ships a `ModelConfig.mk` only for a NAND
board, so the script writes a NOR one. Then:

```bash
SOC=nt98533 FLASH=ns02501/nor.bin bash qemu-boot/run-nt98562.sh
bash qemu-boot/test-nt98533.sh --dir ns02501 --rootfs qemu-boot/rootfs.squashfs.nt98562
```

What runs:

- **Flash boot.** U-Boot 2021.10 comes up and takes its device tree from
  the SHMINFO. It reports 256 MiB, probes the W25Q128, then `nvt_boot`
  loads the uImage from the kernel partition and starts it. Linux 5.10.168
  brings up both cores and the generic timer at 12 MHz, and mounts a
  squashfs root from NOR.
- **`-kernel zImage -dtb nvt-evb.dtb`.** The kernel finds the pinctrl,
  GPIO, I2C and the SPI NOR, which reads, erases and programs correctly.
  The SDK's `nvt-mmc.ko` reads and writes an SD card, and `ntkimethmac.ko`
  takes a DHCP lease and fetches a file over HTTP. Timer 0 counts at
  1 MHz.

`test-nt98533.sh` checks all of it (27 checks, about 30 s).
`test-novatek.py` covers the map and the flash boot in CI.

The SDK kernel reads NOR through `mtdblock` 512 bytes at a time. Each
read is three controller operations, each with its interrupt and DMA
cache maintenance across both cores. That is slow under TCG: about
1.5 MB/s, about 4x better with `-smp 1`. A squashfs root therefore boots
slowly. The OpenIPC nt98562 root used above is a 4.19 userland: it
reaches its init scripts, but its modules do not match this kernel.

Not modelled on NT98533: the media, display and audio engines (the
SDK's HDAL is not set up), the HR timers, the thermal sensor, ADC and
DSP. RTC and watchdog drivers are not in the SDK kernel config, so they
are untested here.

## NT98332 (NA51103)

`nt98332` (`qemu-system-aarch64`) is Novatek's NA51103 NVR die, which the
kernel names NT98331 / NT98332 / NT98631 / NT98632: two Cortex-A53s running
an aarch64 kernel, a GIC-400, the ARMv8 generic timer at 12 MHz, and 1 GB of
DDR. Novatek's blocks sit above 4 GiB, from `0x2_F0000000`; most keep
NA51089's register layout at new offsets, the same offsets as NA51090's
(OpenIPC `u-boot-nt9856x`, `arch-nvt-na51090_a64/IOAddress.h`). The
NT9832x SDK is NA51068, a different die, and does not describe it. The
NT9856x v1.01.007 U-Boot does (`arch-nvt-na51103_a64`). Its chip ID,
`CHIP_NA51103` = `0x8B20`, is TOP's version word here. Its
`IOAddress.h` agrees with the dump's tree on these blocks: TOP, CG, PAD,
GPIO, WDT, the timer, the flash controller, the UARTs, Ethernet, USB,
RTC, OTP, the GIC and the LCD300 (`IDE`). It also lists blocks the NVR's
tree does not use, which are not modelled:

- SDIO 1 / 2 at `0x2_F0190000` / `0x2_F01A0000` (U-Boot's EVB tree has
  `mmc@2,f0190000`);
- a second Ethernet at `0x2_F0450000`;
- I2C 1–4 at `0x2_F01C0000`–`0x2_F01F0000`;
- PWM at `0x2_F0120000`;
- the system counter at `0x2_FFE50000` / `0x2_FFE60000`.

Much of the header is a template copy of NA51089's. Its media and display
lines contradict the dump's tree (HDMI at `0x820000`, the DAI at
`0x630000` on top of the OTP, CSI on the first UART), and the tree wins.

It was brought up on a Xiongmai NBD80N36RA-KL NVR (FW98332A loader,
U-Boot 2019.04, kernel 4.19.148, HDAL 3.03), from a 16 MiB SPI NOR dump.
The map is that firmware's own device tree; the machine builds none.

### Memory map

| Block | Base | IRQ | Model |
|---|---|---|---|
| DDR | `0x0` | | RAM, 1 GB |
| TOP / CG / PAD / GPIO | `0x2_F0010000` / `…20000` / `…30000` / `…40000` | 24 | `nvt-top`, `nvt-cg`, storage, `nvt-gpio` |
| WDT | `0x2_F0060000` | 57 | `nvt-wdt` |
| Novatek timer | `0x2_F0100000` | 0 | `nvt-timer` |
| SPI NOR controller | `0x2_F0180000` | 29 | `nvt-flash` |
| UART 0..3 | `0x2_F0280000` / `…290000` / `…2A0000` / `…2B0000` | 43 / 44 / 45 / 38 | 16550, reg-shift 2, 48 MHz |
| VPE (scaler) | `0x2_F0410000` | 3 | `nvt-engine` |
| Ethernet (DWC EQoS) | `0x2_F0440000` | 12 | `nvt-eqos` |
| USB 2.0 host ×2 | `0x2_F0480000` / `…490000` (PHY `…4E0000` / `…4F0000`) | 27 / 78 | `nvt-usb`, PHY storage |
| RTC | `0x2_F0620000` | 56 | `nvt-rtc` |
| OTP | `0x2_F0630000` | | `nvt-otp` |
| SATA (AHCI) ×2 | `0x2_F0680000` / `…690000` (glue `…6C0000` / `…6D0000`, PHY `…6E0000` / `…6F0000`) | 13 / 14 | `sysbus-ahci`, storage |
| LCD300 | `0x2_F0800000` | 18 | `nvt-lcd300` |
| LCD200 (CVBS) | `0x2_F0820000` | | storage |
| HDMI TX | `0x2_F0880000` | 52 | `nvt-hdmi` |
| DAI (audio) ×5 | `0x2_F0850000`, `…860000`, `…220000`, `…230000`, `…870000` | 15, 34..37 | `nvt-dai` |
| GIC-400 | `0x2_FFF01000` (dist), `0x2_FFF02000` (CPU) | | `arm_gic`, security extensions |

The rest of `0x2_F0000000`–`0x2_FFFFFFFF` reads zero, logged under
`-d unimp`.

### Boot

The flash has the loader (`FW98332A.BIN`, with the DDR training and BL31)
at 0, the device tree at `0x10000` behind an 8-byte `XMFDT32G` tag, LZMA,
and U-Boot LZMA-packed as the whole of `partition_boot` (`0x20000`, Xiongmai's
name for the SDK's `partition_uboot`). With `flash-file` and no `-kernel`
the machine does the loader's part: it unpacks both with
`hw/arm/nvt-lzma.c` (a plain decoder after the public-domain LzmaSpec; QEMU
links no LZMA library), fills the `LD_NVT` SHMINFO as on NA51089, and starts
U-Boot at its link address `0x1E000000` from an AArch64 stub at 0. U-Boot
runs at EL1; QEMU answers PSCI over SMC in place of BL31. U-Boot then does
everything the board does: reads its environment, fills `xm_bootargs`,
reads `boot/uImage` from the romfs squashfs and boots it.

The GIC needs its security extensions. The vendor U-Boot programs every
`IGROUPR` to group 1 (and `GICD_CTLR` = 3). On silicon it runs non-secure
after BL31, so those writes are ignored; on a GIC without the extensions
they land, Linux enables only its own group, and CPU 0 never sees an IPI or
its timer again: the kernel stops right after `CPU1: Booted secondary
processor`. So the GIC has them, the CPUs are non-secure, and the groups
start the way BL31 leaves them (`irq_reset_nonsecure`).

`-kernel Image -dtb fdt.dtb` boots a kernel directly, with the firmware's
device tree (required; it is not built here).

### U-Boot environment

The environment is 64 KiB at flash `0x50000` (CRC32 + `key=value`). A
dump may carry it zeroed; U-Boot then runs on its built-in defaults, which
have `bootcmd`. A valid environment without `bootcmd` does not autoboot
but drops to a password prompt. Xiongmai's `xmuart` turns the console off
unless it is 0; `qemu-boot/uboot-env.py nor.bin xmuart=0` writes U-Boot's
defaults plus that. The settings belong in the environment, not in the
device tree: U-Boot builds `xm_bootargs` (`XmUart=…,XmAuto=…`) from it.

On its first boot the NVR application rewrites `osmem` and reboots once.

### SPI NOR block locks

The Xiongmai kernel turns on the W25Q's per-block protection (SR3's WPS)
and unlocks only what it writes: 256 blocks plus 2 × 15 boundary sectors,
286 lock units on 16 MiB. `nvt-flash` models SR3 (`0x15`/`0x11`, reset
`0x60`) and the individual block lock commands (`0x36`/`0x39`/`0x3D`,
global `0x7E`/`0x98`), lock bits powering up set. Without them SR3 read
`0xFF`, every block read locked, and the services rebooted in a loop
trying to save the environment.

### Audio (`nvt-dai`)

Five DAIs, from the HDAL's `kdrv_audioio/nvt_audio.ko` (not stripped).
After enabling a DMA the record and playback paths poll the "load done"
status (`+0xC4` [19:0]) for a second; when it timed out, the HDAL's own
watchdog (`gs`) took the media stack down. The model takes the queued
buffers on a DMA enable and completes them on a timer at a nominal 8 kHz
16-bit, RX buffers filled with silence. Registers are in the file header.

### HDAL engines (`nvt-engine`)

The HDAL's engines are fired the same way: a job, often a link list of
register writes in DDR, a fire bit, a done interrupt. `nvt-engine` is a
register file that runs the link list (`hi[31:28]` 8 write, 2 next, 0 end)
and reports the job done; it does no pixel work. The list carries the
engine's interrupt enable, so it has to be run. VPE: CTRL bit 31 fires the
list at `+0x300`, status `+0x400`, enable `+0x404`. Without a completion the
job manager's `em_putjob` timed out ("Scaler putjob not response!") and the
HDAL panicked.

### Display (`nvt-lcd300`) and HDMI (`nvt-hdmi`)

The GUI is ARGB1555 in a framebuffer (`/dev/fb0`, `disp0_fb`) that the
LCD300 scans out with the video. Two things the vendor driver depends on:

- **CTRL bit 31 is a register-update request**, not an enable: software
  sets it after a change, the controller clears it at the next frame. The
  driver's tasklet programs nothing while it reads high
  (`dev_check_regupdate_high()`), so a model that keeps it set shows only
  U-Boot's logo setup forever.
- **The frame interrupt** (status `+0x1A4` bit 5, enable `+0x1A8`, clear
  `+0x1AC`): `flcd300-pip` writes a channel's framebuffer address only from
  its handler.

Three input channels, `0x28` apart from `+0x10`: video (YUV 4:2:0, chroma
plane at `+0x04`), the GUI (ARGB1555, alpha bit opaque) and a 64×64 cursor;
control at `+0x24` (bit 31 enable, [3:0] format), sizes at `+0x08`/`+0x1C`.
The output size is `+0x04`; the application picks 1280×1024. The model
composes the enabled channels on the QEMU console;
`-global nvt-lcd300.trace=on` logs the register writes.

`nvt-hdmi` gives the TX what it polls: "clock stable" (`+0x00` bit 12) and
the mute handshake (`+0x88` bit 10 follows `+0x1B0` bit 4). Hotplug reads
0, so the drivers skip EDID; the output shows regardless.

### USB and SATA

Both USB hosts are `nvt-usb`; `-device usb-mouse` drives the GUI.

The SATA hosts are QEMU's one-port AHCI. The Novatek driver
(`nvt_ahci_save_initial_config()`) sets GHC to AE | IE and later writes it
0; on QEMU's AHCI that drops the interrupt enable and every command times
out ("qc timeout (cmd 0xec)"), while the board runs on it. A 4-byte overlay
on GHC keeps AE and IE set. The PHY and glue are storage. The disk is
`-drive if=ide,index=0` (`index=1` for the second port).

### Running

```bash
cp dump.bin nor.bin                       # the image is written to
python3 qemu-boot/uboot-env.py nor.bin xmuart=0
FLASH=nor.bin bash qemu-boot/run-nt98332.sh
```

The GUI comes up about a minute after power-on; the web UI is at
`http://127.0.0.1:8080/`. The board's address is a static 192.168.1.10,
so the run script puts user networking on that subnet.

### Camera video

The NVR shows an IP camera's video: an emulated XM530
(`qemu-boot/run-xm530-camera.sh`, the OpenIPC release with majestic) on a
QEMU socket network with the NVR (`LAN=mcast qemu-boot/run-nt98332.sh`).
The camera's stream is H.264 from the H1 model (`docs/xiongmai.md`): intra
pictures only, DC coefficients only, so the picture is the test pattern at
4x4-block resolution.

The decoder (`nvt-h26xdec`, `0x2_F0600000`, SPI 9) is driven by the
HDAL's `h26xdec.ko`, which parses the parameter sets and slice headers
itself and hands each picture over as a link list of register writes:

- Commands are two words: `(op << 28) | (job << 20) | offset`, then a
  value. Op 0 writes, op 8 waits for the value's bits in the register,
  op 0xC links to the list at the value, or ends the job at -16.
- The wrapper at `+0x3000`: the list's address at `+0x04`, 2 to `+0x00`
  fires it, `+0x18` / `+0x1C` the last command run.
  `IMVQ_VDEC_LLCheckFinalJob()` wants `+0x1C` = -16.
- The job interrupt: `+0x184C` bit 0, acknowledged by writing 1 to
  `+0x1848` (`VPU_ClearInterrupt()`). A line that stays up runs the
  handler again with no list in flight, and it writes through garbage
  indices: a kernel Oops in `update_ll_finish_table()`.
- `+0x1000` (the H.264 core) and `+0x1814` (its CABAC stage): bit 0 is a
  reset that clears itself (`VPU_ResetHW()` polls for that, and its
  read-modify-write carries the start bit along, so a reset wins), bit 1
  starts, bit 16 is done (what the list's waits check). `+0x103C` holds
  the flags (bit 0 done; any of `0x16` is a "BIN error").
- The bitstream is a whole Annex-B access unit, at the `{count, address,
  length}` block `+0x102C` points at. The picture goes to `+0x1040` (Y) and
  `+0x1044` (UV interleaved) at the strides in `+0x1008`. With `+0x1894`
  bit 0 set, a reduced copy follows: 1 / 2^[11:8] of the width and
  1 / 2^[15:12] of the height (half each way here), at `+0x1898`. The
  preview windows read that copy.

H.265 jobs use their own banks, the list writing `+0x1800` = 0 where
H.264's write 2:

- `+0x2000` (AD) and `+0x2800` (BIN) are the control words, with
  H.264's three bits. The list starts AD (`0xE`), then BIN (6).
  `doEngineIsr()` takes bit 16 of both as done and writes it back.
- The interrupt is enabled by `+0x20DC` (and `+0x28DC`), `0x41`. The
  handler turns that off rather than acknowledging `+0x1848`, so an
  interrupt that `+0x1848` gates stays up and the handler Oopses as
  above.
- `+0x211C` is an error register: `check_hw_error()` fails the job
  ("AD has error", `DEC_ERR_ISR`) on anything but 0. `+0x20E4`, which
  the list clears with an op-6 command, is only printed with it.
- `+0x2094` points at the `{count, address, size}` block, but the address
  is the picture's first slice, past the parameter sets the driver has
  parsed, and the size is the buffer's. The access unit is stored whole,
  so the model takes the VPS, SPS, PPS and SEI from just before the slice
  and ends the picture at the next unit that does not belong to it.
- The picture goes to `+0x28B8` (Y) and `+0x28BC` (UV), packed; the
  reduced copy is `+0x29D0` / `+0x29D4`, as `+0x1894` / `+0x1898`. The
  size is `+0x2804` (`[31:16]` height).

The driver's own parser rejects a stream whose VPS declares two temporal
sub-layers once it has parsed one: every later VPS misreads
(`vps_max_layer_id(0x1c)` in the HDAL log) and the channel stops at the
next I frame. Cameras send one sub-layer; the test encoder below has to
be told to.

Built with libavcodec (pkg-config, as `sstar-vpu`), the model decodes each
access unit and writes the picture and the reduced copy, one decoder per
stream size. Without it, jobs complete with nothing written.

The reduced copy's layout is this model's choice: its stride is its width
and its UV follows its Y. The driver's UV offset (`+0x189C`) is too short
for a half-size copy at that stride. Only the VPE model reads the copy, so
the two agree.

The VPE (`nvt-engine` with `scaler=on`) does the window work from its
list's registers:

- the source at `0x308` (size `0x010`, stride `0x800`, its UV right
  after Y), of which `0x01C` is the part shown: 1920×1080 of the
  decoder's 1088-line buffer;
- the destination frame at `0x360` (`0x50C` its size, its UV after its
  Y), and the window in it: `0x550` × `0x508` at x `0x554` `[15:0]`, y
  `0x510` `[15:0]`;
- `0x008` says which: `0xC00` scales a reduced copy into a preview
  window, `0x1000` a full picture into the single-channel view (a
  double-click on a tile, which switches the channel to the main
  stream). With bits 12:10 clear, the job fills the window with the
  background colour `0x260` instead. That is how the NVR clears empty
  tiles to black; without it the frames read as all-zero YUV, i.e. green.

The destinations are the LCD300's triple-buffered 1280×1024 video frames.

Setting the camera up, once:

1. On the camera's console: log in as root, accept the EULA and set a
   password, which its RTSP uses. Then:
   ```sh
   cli -s .video0.codec h264; cli -s .video0.size 1920x1080
   cli -s .video1.enabled true; cli -s .video1.codec h264; cli -s .video1.size 704x576
   /etc/init.d/S95majestic restart
   printf 'auto eth0\niface eth0 inet static\n    address 192.168.1.20\n    netmask 255.255.255.0\n' \
       > /etc/network/interfaces.d/eth0
   ```
   The sub stream matters: the preview windows decode it. With only a
   main stream larger than the NVR's per-window limit (1280×768), the
   application never feeds the decoder.
2. In the NVR's first-boot wizard, at the channel page: edit channel 1 to
   protocol RTSP, template Custom1, user root, the password, main
   `rtsp://192.168.1.20:554/stream=0`, sub `…/stream=1`. Then finish the
   wizard; until it is finished it runs again on every boot and re-adds
   the camera over ONVIF. The NVR's ONVIF client fails `GetStreamUris`
   against majestic, so RTSP is the way.

The camera's tile then shows its live picture.

The XM530 cannot send H.265 (its encoder for that, the Mali VE, is not
modelled), so H.265 is tested with `qemu-boot/fake-ipcam.py` on the host,
looping H.265 files, and QEMU's user networking relaying the camera's
address to it (one `--relay` process per connection). The NVR's channel stays as above:

```sh
python3 qemu-boot/fake-ipcam.py --rtsp 18554 --advertise 192.168.1.20 --fps 25 \
    --file main.hevc --size 1920x1080 --sub-file sub.hevc --sub-size 704x576 &
FLASH=nor.bin LAN=fake bash qemu-boot/run-nt98332.sh
```

The files come from [kvazaar](https://github.com/ultravideo/kvazaar) with
`--gop 0 --period 25 --vps-period 1`. kvazaar writes
`vps_max_sub_layers_minus1` and `sps_max_sub_layers_minus1` as 1, which
the driver does not survive (above): set both to 0 in
`src/encoder_state-bitstream.c` and drop the sub-layer flags and
`reserved_zero_2bits` that `encoder_state_write_bitstream_PTL()` writes
for them.

### Not modelled on NT98332

- The other media engines: OSG, SSCA, JPEG, AGE, the H.26x encoder, the
  DEI, hwcopy and VCAP have no model and read as zero. Their drivers only
  set them up at boot; live view, the single-channel view, recording and
  playback use none of them. The VPE does only the fill and the plain
  scale the NVR uses.
- The decoder's own picture formats beyond what is described above.
- Audio content (RX is silence, TX is dropped), HDMI hotplug / EDID, the
  CVBS output (LCD200, TVE100), the LCD300's cursor and colour pipeline.
- The real loader and BL31 (DDR training, secure world).
