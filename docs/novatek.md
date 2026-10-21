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

| Machine | Chip ID (TOP +0xF0 [31:16]) | DDR | Linux / HDAL split |
|---|---|---|---|
| `nt98562` | `0x7021` | 64 MiB DDR2 | 30 / 34 MiB (`cfg_562_IPCAM1_NOR_EVB`) |
| `nt98566` | `0x7021` | 128 MiB DDR3 | 32 / 96 MiB (`FW98566A.bin`, IPCAM1_EVB) |

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
| Novatek timer | `0xF0040000` | 0 | storage (vendor modules only) |
| WDT | `0xF0050000` | 57 | `nvt-wdt` |
| RTC + power button | `0xF0060000` | 56 | `nvt-rtc` |
| GPIO | `0xF0070000` | 24 | `nvt-gpio` |
| UART1 / 2 / 3 | `0xF0290000` / `0xF0300000` / `0xF0310000` | 43 / 44 / 45 | 16550, reg-shift 2 |
| Ethernet (DWC EQoS) + PHY | `0xF02B0000`, `0xF02B3800` | 34 | `nvt-eqos` |
| SPI NOR / NAND controller | `0xF0400000` | 29 | `nvt-flash` (NOR, or a W25N SPI NAND) |
| SDIO 1 / 2 | `0xF0420000` / `0xF0500000` | 30 / 31 | `nvt-sdio`, an SD card from `-drive if=sd,index=0` / `1` |
| PWM | `0xF0210000` | 26 | `nvt-pwm` |
| I2C 1 / 2 / 3 | `0xF0220000` / `0xF0350000` / `0xF03A0000` | 41 / 42 / 60 | `nvt-i2c` (`/dev/i2c-0..2`); `sensor=` puts a sensor on the first |
| USB 2.0 (FOTG200) | `0xF0600000`, alias `0xFF600000` | 27 | `nvt-usb`, host side; devices from `-device usb-*` |
| OTP / eFuse | `0xF0660000` | | `nvt-otp`, blank |
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
- SDIO3 (`0xF0510000`, the eMMC host), UHS / 1.8 V signalling and tuning,
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
- The Novatek timer, crypto, and the HDAL media blocks.
