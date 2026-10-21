# Novatek NT9856x (`nt98562`, `nt98566`)

Novatek's NT9856x IP-camera SoCs (internal name NA51089): one Cortex-A9
with DDR in the package and Novatek's own peripheral blocks from
`0xF0000000`. One machine file (`hw/arm/novatek.c`) covers the family with a
table entry per part. It boots the OpenIPC `nt98562_lite` / `nt98566_lite`
kernel with its root on the emulated SPI NOR and Ethernet up, and boots a
whole flash image through the vendor U-Boot. What is left is in
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
| SPI NOR / NAND controller | `0xF0400000` | 29 | `nvt-flash` (NOR) |
| SDIO 1 / 2 | `0xF0420000` / `0xF0500000` | 30 / 31 | `nvt-sdio` (empty slot) |
| USB 2.0 | `0xF0600000` | 27 | — |
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

## Flash boot

On silicon the mask ROM loads the 32 KiB loader (`LD9856xA.bin`) into SRAM.
The loader trains DDR with closed libraries and then, per `bl_func.c`:

1. copies the FDT from the fixed `fdt` partition (flash `0x10000`) to
   `/nvt_memory_cfg/fdt`;
2. loads `partition_uboot` to `/nvt_memory_cfg/uboot`, unpacking it if it is
   BCL1;
3. fills the 328-byte SHMINFO at `/nvt_memory_cfg/shmem`;
4. calls U-Boot's entry.

DDR needs no training here, so with `flash-file=` and no `-kernel` the
machine does steps 1–4 itself and leaves `ldr pc, [pc, #-4]` plus U-Boot's
address at 0 for the CPU to start on. The SHMINFO is what U-Boot's
`nvt_shminfo_init()` hangs without: `"LD_NVT"` at +0 and the FDT address at
+44 (`arch/arm/include/asm/nvt-common/shm_info.h`).

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
the vendor init scripts.

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
`qemu-boot/test-novatek-uboot-net.sh --flash nor.bin` starts QEMU paused
with the console on a socket, queues the key, and runs the network checks.
It needs a vendor image, so it is not in CI. `test-novatek.py` covers the
UART fix with a small echo guest instead.

## Running

```bash
cd qemu-boot && curl -sL \
  https://github.com/OpenIPC/firmware/releases/download/latest/openipc.nt98562-nor-lite.tgz \
  | tar xz && cd ..
bash qemu-boot/run-nt98562.sh               # SOC=nt98566 for the 128 MB part
FLASH=nor.bin bash qemu-boot/run-nt98562.sh # a whole flash image
bash qemu-boot/test-novatek-uboot-net.sh --flash nor.bin   # vendor U-Boot net
bash qemu-boot/test-novatek-boot.sh --soc nt98562
python3 qemu/tests/test-novatek.py
```

The release rootfs is SquashFS, and this kernel has no RAM disk driver, so
the boot matrix's `-initrd rootfs.squashfs root=/dev/ram0` does not apply.
`run-nt98562.sh` lays the release into an 8 MiB NOR image
(`qemu-boot/nor-<soc>.bin`, kept between runs) and mounts root from it.
`test-novatek-boot.sh` boots the kernel into a small initramfs (busybox and
an `/init` script) with the same NOR attached. The script checks the device
tree, the clocksource, the pinctrl, GPIO, RTC and watchdog drivers and the
MTD. It reads, erases and programs the flash, takes a DHCP lease and
fetches a file over HTTP, then runs `reboot -f` under `-no-reboot`.

## Not modelled

- SPI NAND behind `nvt-flash` (the 566 boards), and the real loader: it
  trains DDR from its ConfigRAM table with closed libraries
  (`Reset_560.a`).
- An SD card: the SDIO hosts answer as empty slots.
- The EQoS features the drivers leave off: checksum offload, TSO,
  timestamps, more queues.
- USB, the Novatek timer, PWM, I2C, crypto/OTP, and the HDAL media
  blocks.
