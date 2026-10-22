# Novatek NT9856x (`nt98562`, `nt98566`)

Novatek's NT9856x IP-camera SoCs (internal name NA51089): one Cortex-A9
with DDR in the package and Novatek's own peripheral blocks from
`0xF0000000`. One machine file (`hw/arm/novatek.c`) covers the family with a
table entry per part. It boots the OpenIPC `nt98562_lite` / `nt98566_lite`
kernel with its root on the emulated SPI NOR, Ethernet up and an SD card,
and boots a
whole flash image through the vendor U-Boot, on SPI NOR or (the 566 boards)
SPI NAND with a UBIFS root. What is left is in
[Not modelled](#not-modelled).

## What the SDKs support

| Archive | Chip | What it has |
|---|---|---|
| `NT9856x-…-002.zip` → `na51089_linux_sdk_glibc_v1.01.002` | NA51089 (NT98560/561/562/566) | Kernel 4.19.91 and U-Boot 2019.04 source, board configs `configs/Linux/cfg_56*/nvt-*.dtsi`, loader control flow |
| `nt9856x_linux_sdk_v1.01.007.rar` | NA51089 | Newer loaders (≥ v1.01.003 for the newer in-package DDR), `NT98560_DataSheet_Register_Map.pdf`; no kernel source |
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
| EAC / DAI (audio) | `0xF0640000` / `0xF0630000` | — / 15 | `nvt-media` |
| CSI, LVDS, sensor PHY, TGE, SIF, AI | | | `nvt-media`, storage |
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

BCL1 is Novatek's container: the FourCC, then the algorithm and the
big-endian uncompressed and compressed sizes, then the data. Algorithm 9 is
an LZ77 with a marker byte and 7-bit variable-length lengths and offsets
(U-Boot's `lz_uncompress`), and it is unpacked here. Algorithm 11 is LZMA
through a mask-ROM routine and is refused.

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
storage, plus the handshakes its driver waits on. No pixels, samples or
bitstreams move: the engines accept their setup and report each job
done, and their outputs are placeholders.

- **SIE (sensor capture).** While `ACT_EN` is set, a frame comes every
  1/30 s, in three steps the way the sensor's lines would arrive:
  1. VD and the window starts;
  2. the line break points, half way;
  3. the window ends and the DRAM outputs' ends.

  An output in single mode clears its enable (`0xA4` bits 28..30) once
  its frame is written. That is how `ctl_sie` knows a buffer is complete
  and hands it on.
- **IFE, DCE, IPE, IME, ISE, 2D graphics.** `kdrv_ipp` and the graphics
  driver work in linked-list mode: a list of 64-bit commands in DRAM
  (`kdrv_ipp_int_util.h`, `grph_ll.h`). The engine applies each UPD
  (a register write with byte enables) through its own registers, which
  include its interrupt enables, and follows NXTUPD. At NXTLL or NULL a
  job is done: some 1 ms later it raises frame and job end, plus list end
  after the last. The graphics engine raises list end only. `graph_isr`
  reports DMA idle as an abort.
- **H.26x.** `INT_FLAG` is write-1-to-clear, and software reset and DMA
  disable set their finish flags. A picture finishes with:
  - a filler NAL unit (H.265 FD_NUT) in the bitstream buffer;
  - its length in `BS_LEN` (report slot 6) and the NAL length list.
- **JPEG.** A frame "encodes" as a flat grey picture of the right size:
  every block is "DC difference 0, end of block" in the standard Annex K
  Huffman codes, four bytes per 4:2:0 MCU, then EOI. With the driver's
  header that is a valid JPEG.
- **EAC.** Its two strobes clear themselves: `LOAD`, and the DC-cancel
  initial load.
- **DAI.** The audio DMA is double-buffered:
  - enabling a direction latches its buffer (DMA load);
  - each buffer then finishes at 16-bit mono 16 kHz pace, raising buffer
    full and the next load;
  - capture fills the buffers with silence;
  - disabling a direction raises stop.
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
builds them from the vendor SDK (not in CI). The run checks 48 things:
- `hd_video_record` (sensor, SIE, the IPP, H.265) and
  `hd_video_encode_only` write their bitstreams;
- both of `hd_video_snapshot`'s JPEGs decode as 2880x1620 pictures;
- `hd_video_capture_only` runs;
- nine `hd_gfx_only` operations exit cleanly and write their results;
- `hd_audio_capture_only` captures PCM.

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

`test-novatek.py` checks each engine's handshake at register level, in CI.

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
- The HDAL engines' pixel, sample and bitstream work: frames, encoded
  pictures and audio are placeholders (see HDAL media). The graphics
  samples that check their output (DMA copy, arithmetic) fail. The SIE
  ignores the sensor's real timing (fixed 30 fps), and the DAI paces audio
  at a nominal rate. AI/CNN, MD, DIS, VPE, IVE and the display path are
  not wired.
