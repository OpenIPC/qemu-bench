# Fullhan (`fh8852v200`, `fh8856v200`, `fh865x`, `fh885xv100`, `fh8626v100`, `fh8632`)

Fullhan's IP-camera SoCs. Every generation keeps one memory map -- DDR at
`0xA0000000` and mostly DesignWare peripherals behind Fullhan's own PMU --
so one machine file (`hw/arm/fullhan.c`) covers them all with a table entry
per die. The generations differ in the CPU, the DMA engine, the interrupt
controller and numbering, and a few blocks.

## What the SDKs and dumps cover

| Generation | Parts | Source | Kernel | Boot image |
|---|---|---|---|---|
| FH885xV200 | FH8852V200, FH8856V200, FH8858V200 | SDK `FH885XV200_IPC_V1.1.0` (+ FP13), dumps of an FH8852V200 and an FH8856V200 camera, OpenIPC `fh8852v200` | 4.9.129 (board files) | ROM loads U-Boot |
| FH865x | FH8652, FH8656, FH8658 | the FH885xV200 SDK's `#ifdef`s and chip-ID table, an FH8652 camera dump | 4.9.129 (board files, Cortex-A7) | ROM loads a ramboot (DDR sizing), it loads U-Boot |
| FH885x V100 | FH8852, FH8856 | SDK `FH8856_IPC_V1.2.0`, an FH8852 camera dump, OpenIPC `fh8852v100` | 3.0.8 | ROM loads U-Boot (dumps) or a ramboot (SDK images) |
| FH8626V100 | FH8626V100 | a camera dump only (its kernel's kallsyms, IKCONFIG and resource tables) | 3.0.8 | ROM loads U-Boot |
| FH8632 | FH8632, FH8632v2 (mach `FH8833`) | SDK `FH8632_IPC_V1.0` + FP62 patch | 3.0.8 | ROM loads an SPL, it loads U-Boot |

`FH8630D_RT_V1.0` (an RT-Thread SDK) is not covered.

## Machines

| Machine | CPU | DDR | Interrupts | DMA | Chip ID (PMU) | Boot tested |
|---|---|---|---|---|---|---|
| `fh8852v200` | ARM1176JZF-S | 64 MB | DW ICTL | AXI | `0x19112201`, 0x33C = `0x000001` | dump, OpenIPC |
| `fh8856v200` | ARM1176JZF-S | 128 MB | DW ICTL | AXI | `0x19112201`, 0x33C = `0x100001` | dump |
| `fh8858v200` | ARM1176JZF-S | 128 MB | DW ICTL | AXI | `0x19112201`, 0x33C = `0x410001` | —¹ |
| `fh8652` | Cortex-A7 | 64 MB² | GIC-400 at `0x30000000` | AXI | `0x20031601`, 0x33C = `0xC` | dump |
| `fh8656` / `fh8658` | Cortex-A7 | 128 MB² | GIC-400 | AXI | `0x20031601`, `0x8` / `0x4` | —¹ |
| `fh8852v100` | ARM1176JZF-S | 64 MB | DW ICTL | AHB | `0x17092901`, 0x004 = `0xC` | dump, OpenIPC |
| `fh8856v100` | ARM1176JZF-S | 128 MB | DW ICTL | AHB | `0x17092901`, 0x004 = `0x7840D` | dump³, OpenIPC |
| `fh8626v100` | ARM1176JZF-S | 64 MB | DW ICTL | AHB | `0x18112301` | dump |
| `fh8632` | ARM1176JZF-S | 64 MB | DW ICTL | AHB | `0x46488302`, 0x054 = `0x37` | SDK images |

¹ Same die and models as a tested sibling; no image was at hand.

³ A Vatilon PB1 (board IF56N), sold as "FH8856V200": its U-Boot drives the
AHB DMAC and its kernel is 3.0.8 (`FH8856 board init`), so it is this
machine; on `fh8856v200` the U-Boot waits forever on the AXI DMAC's
RawTfr. A PB4 dump of the same vendor is a V200 and boots on
`fh8856v200`.
² Not seen on a dump; `-m` overrides it.

```
qemu-system-arm -M fh8856v200,flash-file=dump.bin -nographic -nic user
```

`flash-file=` is the camera's whole SPI NOR, booted the way the mask ROM
does. Erases and programs are written back to it, so boot a copy. The flash
is QEMU's m25p80 as a part the vendor drivers know, by image size (8 MiB
`w25q64`, 16 MiB `mx25l12805d`; the cameras' Boya BY25Q64AS is in no
vendor table); `flash-type=` overrides it. `-drive if=mtd` works instead of
`flash-file=`. `-kernel` boots a kernel directly with ATAGs for machine
number 9999, which every Fullhan board file uses. `-drive if=sd,index=N`
puts a card in SD slot N.

## Memory map

| Address | Block | Model | Notes |
|---|---|---|---|
| `0x00000000` | mask ROM | synthetic (below) | |
| `0x10000000` | SRAM | RAM | 136 KiB (V200 / FH865x), 32 KiB before |
| `0x30000000` | Cortex-A7 MPCore (GIC-400) | `a15mpcore_priv` | FH865x only |
| `0xA0000000` | DDR | RAM | |
| `0xE0200000` | interrupt controller | `fh-intc` | DW APB ICTL; not on FH865x |
| `0xE0300000` | DMA controller | `fh-axi-dmac` / `fh-ahb-dmac` | |
| `0xE0600000` | Ethernet | `xm-dwmac` | Synopsys GMAC, normal descriptors |
| `0xE0700000` | USB | `dwc2-usb` | DWC2 OTG core, host mode |
| `0xE2000000` / `0xE2200000` | SD0 / SD1 | `hisi-himci` | DW MMC, IDMAC |
| `0xE7000040` | PTS counter | `fh-pts` | FH8632 only |
| `0xE8200000` | AES / DES engine | `fh-aes` | |
| `0xF0000000` | PMU | `fh-pmu` | |
| `0xF0200000` / `0xF0B00000` / `0xF0100000` | I2C0-2 | `fh-i2c` | |
| `0xF0300000` / `0xF4000000` | GPIO0 / GPIO1 | `fh-gpio` | GPIO 0-31 / 32-63 |
| `0xF0500000` / `0xF0600000` / `0xF0640000` | SPI0-2 | `fh-spi` | FH8626V100: SPI0 `0xF0E00000`, SPI1 `0xF0500000` |
| `0xF0700000` / `0xF0800000` / `0xF1300000` | UART0-2 | `fh-uart` | |
| `0xF0C00000` | timers | `fh-timer` | |
| `0xF0D00000` | watchdog | `fh-wdt` | |
| `0xF1200000` | SADC | `fh-sadc` | three register layouts |
| `0xF1500000` | RTC | `fh-rtc` | `v1` on FH8632 |
| `0xF1600000` | eFuse | `fh-efuse` | |

The rest of `0xE0000000-0xF8000000` (ISP, video encoders, JPEG, the ARC
coprocessor's blocks, the DDR controller) reads as zero.

Interrupt numbers (`fh8856v200/chip.h`, the V100 `fh8856.h`, `fh8833.h`,
the FH8626V100 kernel's resources):

| Source | V200 / FH865x | V100, FH8632 | FH8626V100 |
|---|---|---|---|
| WDT, timers | 2, 3 | 2, 3 | 2, 3 |
| I2C0-2 | 11, 12, 46 | 11, 12 | 9, 10, 11 |
| GMAC | 44 | 15 | 13 |
| SD0, SD1 | 42, 43 | 17, 18 | 14, 15 |
| DMAC | 23 | 23 | 21 |
| SPI0-2 | 28, 21, 38 | 28, 21, 22 | 17, 16 |
| UART0-2 | 30, 31, 41 | 30, 31 | 18, 19, 20 |
| GPIO0, GPIO1 | 26, 40 | 26, 40 | 26, 40 |
| RTC | 33 | 33 | 33 |
| USB | 39 | 27 | 27 |
| AES, SADC | 16, 20 | 16, 20 | 24, 25 |

On FH865x the same numbers are GIC SPIs (the timer is interrupt 35).

## Boot

### The mask ROM

The start of the NOR is the ROM's boot header, `2BL*OABD`, version, image
and parameter counts and the header's size at +0x14 (0x180-0x300 on most
cameras, 0x400 on a PB4 that lists ten DDR parts), then 64-byte records from
0x100 up to that size: a name, and at record +0x20 size, +0x24
flash offset, +0x28 load address, +0x2C entry, +0x30 type. The parameter
records (type 0x23 or 0x43, named for the DDR part) point at register
scripts for the fitted DDR (16-byte {address,
value, mask, op} entries for the PMU PLLs and the DDR controller at
`0xED000000`); QEMU's DDR needs no training, so they are skipped. The ROM
loads and enters the image of type 0x10:

- U-Boot itself, at `0xA0800000`, on most cameras;
- FH8632's SPL at `0xA0400000`, which then reads the root header for the
  type-0x12 `Uboot` and its mkimage header and loads it (`spi_spl_load.c`)
  -- run as is;
- on FH865x and in the V100 SDK's images, a ramboot in SRAM, which trains
  the DDR, sizes it by probing for address aliases and loads the U-Boot
  record (type 0x11 or 0x12). QEMU's RAM has no aliases, so the stand-in
  loads that U-Boot itself.

The stand-in puts a `ldr pc, [pc, #-4]` and the entry at 0 and fills the
rest of the ROM with ones: FH8632's U-Boot prints before `serial_init()` has
set its UART pointer, so through a NULL one, polling "USR" at 0x7C.

### U-Boot

All the vendor U-Boots are 2010.06. They keep the environment at `0x40000`
(64 KiB, CRC over 0xFFFC bytes). The V200 ones choose its medium from the
PMU's boot strap, `0xF0000330`: 1 SPI NAND, 2 SPI NOR (the model's), 3 / 4
eMMC on SD0 / SD1 -- with anything else they print `bad CRC, using default
environment`. They read the flash through SPI0 with the AXI or AHB DMAC
(below), probe the Ethernet PHY, and most try a TFTP update (`netupdate`)
before `sf read` and `bootm`. Some stop autoboot on one key only, whatever
they print: the Vatilon PB1's on a capital `H` (its abortboot compares with
0x48).

### OpenIPC

OpenIPC publishes `fh8852v200`, `fh8852v100` and `fh8856v100` lite bundles
but no Fullhan bootloader: on a camera it is installed on the vendor U-Boot,
from its console -- `tftp` each file to `0xA1000000`, `sf erase` and
`sf write` it at 0x50000 (kernel) and 0x250000 (rootfs) -- which runs the
same under QEMU with `-nic user,tftp=<dir>` (OpenIPC/firmware#1610).
`qemu-boot/fh-openipc-flash.sh` does the same -- a dump's boot region, the
OpenIPC kernel and rootfs at OpenIPC's 8 MiB layout
(`256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)`, which
the vendor U-Boot's own matches), and an environment with OpenIPC's
`bootargs`/`bootcmd`:

```
bash qemu-boot/fetch-openipc-lite.sh fh8852v200 qemu-boot
bash qemu-boot/fh-openipc-flash.sh dump.bin fh8852v200 qemu-boot
qemu-system-arm -M fh8852v200,flash-file=qemu-boot/openipc-fh8852v200-nor-8mb.bin \
    -nographic -nic user,restrict=on
```

The MTD device is `spi_flash` under the 3.0.8 kernels and `spi0.0` under
4.9 and FH8626V100's 4.19. OpenIPC's kernels have no RAM disk driver, so
`-initrd` cannot carry the SquashFS; `qemu-boot/test-fullhan-boot.sh` (the
CI's) boots `-kernel` with the rootfs in a synthetic NOR image and checks
the login prompt, a DHCP lease and the JFFS2 overlay written back to the
image. The image's env partition holds a valid U-Boot environment, as a
camera's does: with no `/etc/fw_env.config`, OpenIPC's `fw_printenv` looks
for one by CRC, and when there is none it CRCs 64-256 KiB at every 4 KiB of
the flash -- seconds per call under TCG, a dozen calls before login.
FH8626V100's release reaches login in under 20 s, booted with
`sensor=gc1054`, its DT's sensor; the test checks that `fh8626-3ad` starts
on it (**Image sensor**, below). Its kernel builds no Ethernet driver (the board it was brought up on is Wi-Fi only), so the
DHCP check is skipped there, and it boots with `mem=64M` and `osmem=64M`
in that environment (its DT reserves CMA and the ARC's memory up to the
top of 64 MB, and `load_fullhan` writes `osmem` and reboots until it reads
64M). A second boot there runs a script as PID 1 that starts the ARC and
echoes 64 messages through it (**ARC**, below). Mind the host CPU: that
kernel never issues WFI (`fh8626_idle()` is empty), so a guest at idle
still takes a whole host core. The first login asks the camera's owner to accept majestic's EULA.

## Devices

**PMU** (`fh-pmu`). Mostly storage. The chip-ID registers per die (0x000;
0x004 the V100 package, 0x33C the V200 one; 0x054 FH8632's variant), the
DDR size in Mbit (0x334), the NOR boot strap (0x330). The block resets
(0x040-0x04C, FH8626V100 0x04C-0x058) read back finished; clearing bit 31
of the first is the system reset, bit 18 restarts the timers. PTSLO/PTSHI
(0x22C/0x230) latch a 1 MHz count on a write of 1. WDT_CTRL bit 8 pauses
the watchdog. The timers' clock (`tmr-clk`) is computed the way the
kernels compute it: from the system PLL's P output (PLL2 on V200 / FH865x,
PLL0 on V100; VCO = 24 MHz x M / (N + 1), / (P + 1) / DIVVCO), / (CLK_DIV5
[27:24] + 1) / (CLK_DIV5[23:16] + 1) -- or on FH8632 / FH8626V100 from PLL1
= 24 MHz x M / N / 2^OD, / 10 (FH8632's `tmr0_clk` prediv; FH8626V100's
kernel has none, `tmr-prediv` 1) / (CLK_DIV5[23:16] + 1). The reset values
give 1 MHz, which the U-Boots also program; the V100 and FH865x kernels
set 50 MHz through the same divider. FH8626V100 is the exception: PLL1 / 6
(`dw_100m_clk`) / (CLK_DIV5[7:0] + 1), as OpenIPC's 4.19
`clk-fh8626v100.c` computes it -- the only source that names the field;
no FH8626V100 SDK, and no dump here to read DIV5 from. [7:0] resets to 9:
10 MHz, which the vendor 3.0.8 kernel runs at without touching the
divider, and the 4.19 kernel sets its DT's 50 MHz. Counting [23:16]
instead left that kernel's clock at 10 MHz while it reckoned 50: guest
time ran 5x slow, a 10 s wait took 55 s, and the watchdog it fed on that
time reset the machine after login.

**Interrupt controller** (`fh-intc`). The DW APB ICTL: enable, mask, force,
raw / status / final status in low/high pairs. The 4.9 kernel's handler
calls `find_first_bit()` on FINALSTATUS (0x30), and ARM's loads bytes, so
byte reads at any offset work.

**Timers** (`fh-timer`). DW APB timers, 0x14 apart: a disabled channel reads
0 (both drivers wait for it), periodic and free-running modes, interrupt
mask, EOI, and the block-wide status at 0xA0. The channels share one
interrupt: the V200 kernels tick on channel 0, the older ones on channel 1.

**UART** (`fh-uart`). A 16550 with registers four bytes apart plus the
DesignWare USR (0x7C), which the vendor drivers poll instead of LSR: TX FIFO
empty and not full, RX FIFO not empty / full from the 16550's FIFO.
On FH8626V100 each UART also answers across the rest of its 1 MB APB slot,
the 0x100-byte window repeating (`uart_slot_mirror`). OpenIPC's 4.19
FH8626 kernel needs this. Its `DEBUG_LL` UART is PHYS 0xF0700000 /
VIRT 0xFE020000, which `head.S` maps with a single 1 MB section, so until
`paging_init()` takes over, the early console writes THR and polls LSR at
0xF0720000. Without the mirror that read hit the unimplemented periph
space (0), and the kernel spun before printing a line. The camera boots,
so the hardware answers there, and the FH8626 DTS puts every peripheral
alone in its 1 MB slot.

**ARC** (`fh-arc`, FH8626V100). OpenIPC's 4.19 kernel drives the ARC600
itself (`drivers/remoteproc/fh8626_arc.c`), with a firmware built from
source (OpenIPC/firmware `general/package/fh8626-arc-firmware`,
`/lib/firmware/fh8626-arc.elf`). That firmware announces one RPMsg
endpoint, `rpmsg-arc-echo` at 1024, and echoes what it is sent, polling
split virtqueues in the shared pool at 0xA3FC0000.
The kernel loads the ELF into the reserved 0xA3F00000 and writes four boot
words at PMU 0x2000 (`sr entry, [INT_VECTOR_BASE]`, `j entry`). It then
releases the reset (PMU 0x1C4 bit 18) and sets run (PMU 0x2010 bit 4).
With both set the core starts at 0xF0002000, those words' physical
address, and sees the ARM's physical map. The doorbells are plain PMU
registers: 0x1E4 to the ARC (a write wakes the core at once) and 0x1F0
bits 1:0 to the ARM (vrings, crash), which raise IRQ 32 while set.
The core is an interpreter for ARCompact's 32-bit formats, decoded as the
ARC600 decodes them:
- Bcc/B, BL and BRcc;
- LD and ST, with sizes, sign extension and write-back;
- the ALU operations in all four operand formats, with long immediates,
  conditions and flags;
- the barrel shifter;
- LR/SR, J/JL and SYNC.

That is everything the firmware is built from. A 16-bit instruction, a
delay slot or a reserved opcode takes the instruction-error exception
(vector 0x10); the firmware reports it to the ARM as a crash. `mips`
(default 2) sets its rate; a slice that stores nothing is taken as
polling and backs off to 10 ms until a store or the doorbell.
Run on the release, the kernel's `remoteproc0` comes up and the firmware
announces its channel. Through `rpmsg_char` (bound by `driver_override`;
`qemu-boot/fh-rpmsg-echo.c`), 64 messages of 1 to 496 bytes come back
unchanged, and the firmware's own diagnostics read ECHO_RUNNING with no
error.

**SPI** (`fh-spi`). A DW SSI. SPI0 on V200 has Fullhan's "SPIC" additions:
CCFGR (0xF4) with the data phase's wire count, a byte swap and 32-bit
frames, XIP registers, and a 4 KiB data window at +0x1000 where every word
is DR. Flash reads go RX-only: the command in TX-only mode, then CTRLR1 + 1
frames clocked in on a dummy DR write -- in 16-bit frames with the swap on,
so each frame is the next two bytes in memory order. The chip selects are
GPIOs (GPIO 6 on V200 and FH865x, GPIO 54 before), wired to the flash
through the controller's `cs-gpio` input. QEMU's flash models count
fast-read dummies in cycles, one transfer each, where these drivers send
one dummy byte: the controller follows the command from the select on and
clocks the dummy byte of 0x0B / 0x3B / 0x6B as 8. The RX request line goes
to the DMAC while the RX FIFO holds data.

**AXI DMAC** (`fh-axi-dmac`). DW_axi_dmac in linked-list mode: 64-byte
descriptors (SAR, DAR, BLOCK_TS, LLP, CTL), CHEN with write-enable bits,
per-channel interrupt status / signal enables. U-Boot drains SPI0 on
handshake 4 from the data window, one descriptor per 4 KiB with the source
address reloaded (the kernels' SPI drivers read by PIO). Transfers
run in a bottom half: the kernel counts channels by enabling each with
nothing programmed and reading CHEN back.

**AHB DMAC** (`fh-ahb-dmac`). DW_ahb_dmac (a `dw_dmac` fork): channels 0x58
apart, RAW / STATUS / MASK / CLEAR per interrupt kind, `{sar, dar, llp,
ctllo, ctlhi}` descriptors with DONE written back, and the encoded
parameter words the kernel sizes its channels from (FH_PARAMS bit 28 must
say they are there). V100's U-Boot reads the flash with it on handshake 2.

**I2C** (`fh-i2c`). The DW I2C master: DATA_CMD with read / STOP / RESTART
bits, FIFO depths in COMP_PARAM_1, an absent target aborting with
`7B_ADDR_NOACK`.

**Image sensor** (`sensor=`, on I2C0). Only the chip ID and a register file
that takes the init tables: enough for detection, no pixels. The vendor
apps link their sensor drivers in and probe `/dev/i2c-0` in turn
(FH885xV200: GC2063 at 0x37, GC4653 at 0x10 then 0x29, MIS2006 at 0x30;
the V100 board and FH8626V100 apps GC2053 / GC2093 / GC1054 / JX-F37 /
MIS2006), and write what they found to `sensor.conf` on the custom
partition; OpenIPC's `ipcinfo -s` and majestic probe the same bus.

| `sensor=` | Address | Registers | ID | Default on |
|---|---|---|---|---|
| `gc4653` | 0x29 | 16-bit | 0x03F0/1 = 0x46 0x53 | FH885xV200 |
| `gc2063` | 0x37 | 8-bit | 0xF0/1 = 0x20 0x63 | FH865x |
| `gc2053` | 0x37 | 8-bit | 0xF0/1 = 0x20 0x53 | V100, FH8626V100 |
| `gc2093` | 0x37 | 16-bit | 0x03F0/1 = 0x20 0x93 | |
| `gc1054` | 0x21 | 8-bit | 0xF0/1 = 0x10 0x54 | |
| `mis2006` | 0x30 | 16-bit | 0x3000/1 = 0x20 0x06 | |
| `f37` | 0x40 | 8-bit | 0x0A/B = 0x0F 0x37 | |
| `none` | | | | FH8632 |

The GalaxyCore parts are `galaxycore-sensor`; a 16-bit one answers 0 to a
read after a single address byte, so ipctool's 8-bit probe falls through
to its 16-bit one. Detection is as far as the vendor apps get: the V200
`ipcam` prints `detected sensor gc4653_mipi`, then fails `FH_SYS_Init`
(the media devices need the ARC, below) and reboots the camera.

OpenIPC's FH8626V100 kernel (4.19) has its own open media drivers instead,
and its DT names the sensor: a GC1054 at I2C0 0x21, feeding the CSI
(0xF1100000, wrapper 0xF1000000) and the ISP (0xE8400000). Boot that
release with `sensor=gc1054`; the default GC2053 is the vendor dump's.
With the GC1054 present, `fh8626-capture` registers its media graph:
`fh8626-isp`, `fh8626-csi` and `gc1054 0-0021` subdevices, the camera,
logo and VPU video nodes. Then `fh8626-3ad`, the ISP's 3A daemon, finds
them and starts: it reads the ISP's gain range and tuning controls, then
waits for statistics.
Without the sensor the capture driver's async notifier never completes, no
subdevice appears, and the daemon gives up after 10 s.
What the twin does not do yet: the CSI and ISP registers are the
unimplemented periph space, so no frames and no statistics arrive. On this
release nothing would ask for them: `divinus` needs the vendor SDK's
`libvmm.so`, which this image does not ship, and exits. The encoder's reset
times out, and the JPEG block reads its ID as 0 (0x00038401 expected).

**USB** (`dwc2-usb`). QEMU's DWC2 in host mode; `-device usb-storage`,
`usb-net` and the rest plug into its root port (behind QEMU's hub). The
4.9 kernels run it under mainline dwc2 (`fh_usb`; a module, `dwc2.ko`, in
OpenIPC). The 3.0.8 kernels' `fh_otg` (a dwc_otg fork) checks each of its
parameters against GHWCFG2-4 / HPTXFSIZ and gives up (-22) unless the core
could do them, and always runs the host channels with descriptor DMA: on
V100, FH8626V100 and FH8632 the machine sets those registers to an OTG core
with a UTMI PHY, dedicated FIFOs and DescDMA, and patch 0014 adds host
descriptor DMA to the model. With the FH8626V100 kernel (`cdc_ether`
built in) and `-device usb-net`, `usb0` pings the host; the OpenIPC
kernels have no class drivers built in, but enumerate devices.

**RTC** (`fh-rtc`). The V200 / FH865x / V100 block is a wrapper around a
core reached by command (INT_STATUS bit 5 core idle, CMD, RD_DATA,
WR_DATA); the core's registers go up to the twelve words of the temperature
LUT at index 9, so the register field is bits 8:4. FH8632's `v1` block has
its registers direct, a SYNC register and a set-up magic in USER_REG. Both
pack the time as days since 1970 in 31:17, hours, minutes and seconds, and
run from `-rtc base=`.

**Watchdog** (`fh-wdt`). DW WDT: sticky enable, response mode (interrupt
first, reset on the second expiry), 2^(16 + TOP) clocks, kick 0x76, paused
by the PMU.

**Ethernet** (`xm-dwmac`). The fh_gmac drivers are Synopsys GMAC with
normal descriptors, the MAC block where `xm-dwmac` has it. They accept only
the PHYs on their list (IP101G `0x02430C54`, RTL8201, the internal EPHY
`0x441400`), attach the generic PHY code to MDIO address 0, and take 4 off
every received length; so the PHY is an IP101G at address 0 and the core
keeps the FCS. The V100 U-Boot's driver ("FH EMAC") polls with the CPU's
interrupts off: each poll sets SR, scans all 32 RX descriptors for one it
owns and clears SR again. `xm-dwmac` delays only the first SR after the
ring is set up (which Linux's `stmmac_open()` needs); later starts take a
waiting frame at once, or a U-Boot TFTP download stalls for good.

**AES** (`fh-aes`). The engine `fh_aes.c` registers with the kernel
crypto API (`ecb/cbc/ctr/ofb/cfb(aes)`, DES and 3DES, priority 100) and
as Fullhan's cipher device: control word (bit 0 decrypt, 3:1 the cipher,
6:4 the mode, bit 7 take the IV), key and IV as big-endian words, a DMA
source / destination / length and a done interrupt. The driver feeds a
request in 2 KiB transfers and loads the IV with the first only, so the
chaining value carries over (and reads back from LAST_IV). The ciphers are
QEMU's; keys the eFuse would load are not modelled.

**SADC** (`fh-sadc`). The eight-input ADC behind the light sensor and
buttons. Three layouts: `version` 1 (V100, FH8632: CMD, CONTROL's input
mask, a done bit), 2 (FH8626V100's `CONFIG_FH_SADC_V21`, the same behind a
MODEL register) and 3 (V200 / FH865x: enable, update, a done bit per
input). A conversion takes 100 us and returns the input's level, the
`chN` properties of `/machine/sadc`, which `qom-set` moves at run time
(`cat /proc/driver/sadc` shows them in mV).

**eFuse** (`fh-efuse`). Command register (load, program, read, check) with
a done bit per operation in STATUS0; blank fuses. FH865x's ramboot reads it.

## QEMU patches this family needs

- `0012-arm-ccsidr-missing-cache-reads-l1d.patch`: FH865x's U-Boot writes
  CSSELR from a register still holding a delay loop's count (Level 3)
  before its last D-cache clean, and counted ways from the 0 QEMU returned
  forever. CCSIDR for a cache that is not there now reads the L1 D-cache's.
- `0013-m25p80-clear-wel-after-program-erase.patch`: the 3.0.8 kernels'
  m25p80 waits for SR to read neither WIP nor WEL after every erase and
  program; the model never cleared WEL, so each took the 40 s timeout and
  JFFS2 put every freshly erased block on its bad list.
- `0014-dwc2-ghwcfg-props-host-descriptor-dma.patch`: `ghwcfg2/3/4` and
  `hptxfsiz` properties, and host-mode descriptor DMA for non-isochronous
  channels (descriptor write-back, EOL / last / short-packet halts, BNA,
  silent NAK retries, interrupt channels run in the frames their FrameList
  entry names) -- what `fh_otg` needs.

## Not modelled

The ARC625 coprocessor (Synopsys ARC600 family, ARCompact) and the media
blocks it serves. The ARM boots it through PMU 0x2000-0x2010 and the two
interrupt each other through PMU 0x1E0-0x1F4 (`mach-fh/pmu.c`); everything
past that -- the xbus (V200) / rtvbus (V100) protocols, the firmware's
services, the ISP / VPU / encoder / JPEG register interfaces -- ships only
as binaries (`make_drivers/*.o`, `rtthread_arc_*.bin`, the media `.ko`s),
with no source or register documentation in the SDKs. RT-Thread's
open-source vbus describes the generic ring protocol but not Fullhan's
glue (ring placement, notification, ready flag, block size). So
`xbus_rpc.ko` fails ("XBUS: firmware not ready!") and on V100 / FH8626V100
`media_process.ko` does; `media_process_no_sc.ko`, shipped alongside, loads
without the ARC. Video and audio (the internal codec is the ARC
firmware's) need that material from the vendor.

That is the vendors' ARC firmware. OpenIPC's FH8626V100 ARC runs its own,
and that is modelled (**ARC**, above).

Also: PWM (registers read as zero; the drivers only program it), the IR
receiver, the internal Ethernet PHY's training. The V100 U-Boot reads the
RTC through an older register layout than the kernel and is left with
whatever the v2 block answers.
