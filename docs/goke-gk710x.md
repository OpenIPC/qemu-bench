# Goke GK7101 / GK7102 Hardware Reference

GK7101 and GK7102 are Goke's own first IP-camera SoC, not a rebadged
HiSilicon part like the later GK7205/GK7605 (which `hisilicon.c` models).
One ARM1176JZF-S, DDR at `0xC0000000`, and peripherals that are Ambarella
A5s IP underneath. GK7102 is the GK7101 die with 64 MiB of DDR in the
package (GK7101: 128 MiB, external). The vendor BSP is
`GK710X_LinuxSDK_v2.0.0`: U-Boot 2012.10 (`board/goke/gk7101`), Linux
3.4.43 (`arch/arm/mach-gk710x`, `plat-goke`), no device tree, ATAGs,
machine `GOKE_IPC` (3988).

Machines: `gk7101`, `gk7102` in `qemu/hw/arm/goke.c`; the S die,
`gk7101s` and `gk7102s`, is described in its own section below.

## The HAL

Nothing in the vendor U-Boot or kernel touches a register directly. U-Boot
carries a blob, `gk7101_hal_data.h` in the SDK, copies it to `0xC0012000`
(`CONFIG_GK_HAL_ADDR`) and stores that address at `0xC0000008` for the
kernel. `hal_init()` returns a `struct hw_ops` of `hw_readb/w/l`,
`hw_writeb/w/l`, `flash_read/write` and (from 20151223) USB and SPI
accessors; every `gk_*_readl()` / `gk_*_writel()` in both trees is one of
them (`*_HAL_MODE 1` in `mach-gk710x/include/mach/gk710x.h`).

The drivers are written against a **fictitious** map — AHB at `0x90000000`,
APB at `0xA0000000` (`mach-gk710x/include/mach/hardware.h`, U-Boot
`arch-gk7101/gk7101_reg.h`) — and the HAL:

1. moves each block to its real place (AHB `0x60000000`, APB `0x70000000`,
   at different offsets);
2. shuffles the registers inside a block;
3. permutes, drops or recomputes the bits of some registers (the VIC masks,
   UART IER/LCR/LSR/MCR, timer and watchdog control, I2C control).

Undo all three and the hardware is standard: the UART is a 16550 with
`regshift 2`, the VIC is PL190-shaped, the timers, watchdog and I2C are
Ambarella's. The QEMU models are of that silicon, at the real addresses
the HAL drives.

The map was recovered by executing the HAL rather than reading it:
`qemu-boot/gk710x-halmap.py` runs `hw_writel` / `hw_readl` under Unicorn
for every fictitious address and prints where each lands and what happens
to each bit. Three versions were compared — 20151223 (both camera dumps),
20160324 (the SDK's prebuilt `uboot-gk7102-*.bin`) and 20160913 (the SDK
U-Boot source) — and differ only in the I2S and crypto bit shuffles, which
the RAM-backed stubs absorb.

```
$ gk710x-halmap.py dump.bin a0005000-a0005020
# HAL version 20151223
a0005000 -> 70005004                                  IER/DLH -> 16550 +0x04
a0005004 -> 70005000                                  RBR/THR -> 16550 +0x00
a000500c -> 70005010 W{0:2 1:3 2:0 3:1 4:6 6:4 7-31:-} MCR, bits permuted
a0005014 -> 70005014 W{...} R{...}                    LSR, read recomputed
a0005018 -> 7000500c W{2:3 3:2 4:7 6:4 7:6 8-31:-}    LCR (DLAB is bit 7)
```

### VIC bit wiring

The HAL applies one permutation to every VIC register of both VICs, so it
is wiring: the kernel's IRQ bit *n* (`mach-gk710x/include/mach/irqs.h`) is
silicon input `irq_line[n]`:

| kernel bit | 0–5 | 6–7 | 8–15 | 16–23 | 24–31 |
|---|---|---|---|---|---|
| VIC input | 26–31 | 0–1 | 18–25 | 10–17 | 2–9 |

## Memory map

| Block | Silicon | HAL address | Kernel IRQ | Model |
|---|---|---|---|---|
| DDR | `0xC0000000`, 64 / 128 MiB | | | RAM |
| SD (SDHCI) | `0x60002000` | `0x90000000` | 8 | `generic-sdhci` |
| VIC1 | `0x60003000` | `0x90008000` | | `gk-vic` |
| DDR controller | `0x60004000` | `0x90002000` | | regbank |
| DMA | `0x60005000` | `0x90003000` | 21 | regbank |
| USB (MUSB OTG) | `0x60006000` | `0x90006000` | 26 | RAM, CONFIGDATA preset |
| I2S | `0x6000A000` | `0x9000A000` | | regbank |
| eFuse | `0x6000B000` | `0x90001000` | | regbank |
| GMAC / its DMA | `0x6000E000` / `0x6000F000` | `0x9000E000` / `F000` | 1 | `xm-dwmac` |
| VIC2 | `0x60010000` | `0x90009000` | | `gk-vic` |
| Crypto | `0x60014000` | `0x90005000` | | regbank |
| SPI-NOR controller | `0x60016000` | `0x90016000` | | `gk-sflash` |
| AHB GREG, USB PHY, audio, DDR PHY | `0x60020000` | `0x90020000` | | regbank |
| Ethernet PHY | `0x60022000` | `0x90022000` | | `gk-ephy` |
| PMU, RTC, 8051 code | `0x60080000` | `0x90080000` | | regbank |
| Timers 1–3 | `0x7000B000` | `0xA0000000` | 18, 19, 20 | `gk-timer` |
| TSSI / SSI2 | `0x70001000` / `0x70002000` | same offsets | | regbank |
| I2C 0 (IDC) | `0x70003000` | `0xA0003000` | 9 | `gk-idc` |
| UART0 | `0x70005000` | `0xA0005000` | 31 | 16550 |
| I2C 1 (IDC2, HDMI) | `0x70007000` | `0xA0004000` | 58 | `gk-idc` |
| GPIO | `0x70009000` | `0xA0009000` | 16, 17, 4 | regbank |
| Watchdog | `0x7000C000` | `0xA0006000` | 11 | `gk-wdt` |
| ADC2 | `0x7000D000` | `0xA0007000` | | regbank |
| SSI1 | `0x7000F000` | `0xA0008000` | 10 | regbank |
| PWM 0–3 | `0x70010000`.. | `0xA000A000`.. | | regbank |
| UART2 / UART1 | `0x7001E000` / `0x7001F000` | same offsets | 27 / 15 | 16550 |
| MEM | `0x70150000` | `0xA0150000` | | regbank |
| RCT (clocks) | `0x70170000` | `0xA0170000` | | regbank |

The kernel also maps the fictitious ranges themselves (`AHB: 0x90000000
0xf2000000`), and the HAL passes addresses of blocks it does not know
straight through; `-d unimp` logs anything reaching either window.

## Devices

- **CPU.** QEMU's `arm1176` lacks the TCM region registers (CP15 c9, c1
  and c9, c2); U-Boot's first instructions read them. The board adds them
  reading 0 (no TCM).
- **UART.** 16550, `regshift 2`, 24 MHz (U-Boot: `UART_BASE_FREQ` = 48 MHz
  / 2). DesignWare extras past `+0x20` are storage (the kernel clears the
  software-reset register at `+0x88`). The HAL forces LSR.THRE on read.
- **VIC** (`gk-vic`). PL190 registers at PL190 offsets up to PROTECTION
  (`0x20`), then SENSE `0x24` (1 = level), BOTHEDGE `0x28`, EVENT `0x2C`
  (1 = rising / active high) and EDGECLR `0x38`, which the kernel's
  `irq_ack` writes. Timers are registered rising-edge, UART, Ethernet and
  I2C level-high. The IR receiver (IRQ 62) asks for level-low, so its
  undriven input rests high (`idle-high`); at 0 it storms as soon as its
  driver loads.
- **Timers** (`gk-timer`). Ambarella layout: per timer STATUS (live count),
  RELOAD, MATCH1, MATCH2 at `0x10 * n`; CONTROL at `0x30`, a nibble per
  timer: enable, external clock, overflow interrupt. 69 MHz: U-Boot's
  `APB_CLK`, and the kernel's fallback when the RCT PLL words read 0.
- **Watchdog** (`gk-wdt`). STATUS, RELOAD, RESTART (magic `0x4755`),
  CONTROL (enable, reset, interrupt), TIMEOUT, CLR_TMO, RST_WD. The
  vendor kernel reboots through it, so `reboot` works.
- **SPI-NOR** (`gk-sflash`). No memory window: COMMAND (`+0x04`) describes
  one SPI transaction (opcode, address and dummy byte counts, direction,
  lane widths, 1–4 data bytes per access) and DATA (`+0x00`) carries the
  address and then the data. Bytes are packed MSB first, so every 32-bit
  word of a GK710x flash image is stored byte-swapped: dumps read as
  big-endian, the boot header as "GK7101" in a hex editor. A GD25Q-series
  JEDEC ID of the image's size is reported (the dumps: GD25Q128C,
  GD25Q64C).
- **I2C** (`gk-idc`). Ambarella IDC: ENR, CTRL (ACK, IF, START, STOP),
  DATA, STS (receiver), PSLL/PSLH, FMCTRL/FMDATA (bulk-write FIFO). One
  byte per interrupt, as `i2c-goke_v1_00.c` drives it. `sensor=jxh42`,
  `jxh62` or `f37` puts an SOI sensor model on bus 0.
- **Ethernet.** The GMAC is a Synopsys DWMAC (MAC + DMA at `+0x1000`),
  served by `xm-dwmac` with `regs-size=0x2000` (VIC2 follows at
  `+0x2000`) and `rx-fcs`: the core stores the FCS and the kernel driver
  subtracts 4 from every received length. With `phytype=0` (U-Boot's
  default) the kernel reads the internal PHY's MII registers from memory,
  register *n* at `0x60022000 + 4n` (`gk-ephy`), not over MDIO.
- **SD.** A standard SDHCI underneath (`generic-sdhci`), with two Goke
  quirks the vendor driver (`gk_sdio2_v1_00.c`) depends on. CAP bit 27
  must be set (`SD_CAP_INTMODE`; without it `gk_sd_probe` fails with
  `-ENODEV` and no message). The write-protect pin level reads inverted
  ("1 read only, 0 write": `wp-inverted`). The vendor kernel takes card
  detect from a GPIO, active low, which reads 0 here, so it always sees a
  card; the model's PRESENTSTATE has card-inserted only with `-drive
  if=sd`.
- **USB.** Mentor MUSB. Plain memory, except CONFIGDATA (`+0x1F`) reads
  DYNFIFO | SOFTCONE: with 0 there the vendor driver finds no bulk FIFO,
  fails the probe and oopses in its own error path.
- **Everything else** is a RAM-backed `hisi-regbank` with `sub-word`
  accesses on (the HAL has byte and halfword accessors for every block).

Not modelled: the DSP / video pipeline (the vendor media driver probes the
sensor, then nothing streams), audio, PWM, ADC, GPIO interrupts.

## Boot

### Flash (`flash-file=`)

The mask ROM reads the boot header at flash offset 0 (in CPU words, after
the byte swap above):

| Word | |
|---|---|
| 0–1 | `"GK7101"`, version |
| 2 | U-Boot length in words |
| 3 | load and entry address (`0xC0800000`) |
| 6 | `0x5A5A5A5A` |
| 7.. | `address, value` register writes (DDR PHY, DDR controller, RCT); `0xFFFFFFFE, address, value` = poll until equal; `0` ends the list |

U-Boot follows the terminating 0. `goke.c` does the ROM's job on the host:
it applies the writes at reset, skips the polls (the DDR controller and the
eFuse loader are instant here), copies U-Boot to the load address and
starts the CPU there. Everything after is the camera's own code; the stop
key at "Hit any key to stop autoboot" is Enter (`\r` only, `common/main.c`).

### `-kernel`

The kernel needs what U-Boot leaves behind (`common/cmd_bootm.c`,
`CONFIG_UBOOT_TO_KERNEL`; `mach-gk710x/gk710x.c`, `gk_map_io`):

- the HAL at `0xC0012000`, its address at `0xC0000008`;
- the U2K words at `0xC0000000`: PHY type, ARM clock, kernel memory,
  total memory (MiB), BSB size, `0x7101` "valid";
- `mem=` on the command line: the kernel lives from `PHYS_OFFSET`
  `0xC0200000` and carves the DSP and BSB regions out of what lies above.

The HAL is vendor code and is not shipped: `hal-file=` names any image
holding one — a vendor U-Boot binary or a flash dump, in either byte order
(default: `flash-file=`). The machine finds it by the `hal_init` prologue.
`mem=` defaults to the vendor U-Boot's: 47M on `gk7102` (what the GK7102
cameras pass), 50M on `gk7101` (`include/configs/gk7101.h`). QEMU's boot
stub and ATAGs go to `0xC0100000`, clear of both.

OpenIPC's `gk7102` kernel calls the SPI accessors added in HAL 20160324
and jumps through a NULL pointer with an older one; the cameras' own
kernels run with any.

## Tested

- `fullflash_..._GK7102_lifecontrol.bin` (16 MiB GD25Q128C, U-Boot
  "GK7102 rb ov9712 v1.0", HAL 20151223, 3.4.43 #27): U-Boot (`sf`, `ping`,
  `dhcp`), the kernel, the vendor application and a shell on the console;
  DHCP and ping from Linux.
- `somedump.bin` (8 MiB GD25Q64C, "GK7102 rb jxh42 v2.00"): its U-Boot runs
  and then stops at "Alpu-C Encryption Test Fail" — the board carries an
  ALPU-C anti-cloning chip on I2C that U-Boot challenges, and that chip is
  not emulated. Its kernel and root filesystem boot with `-kernel` (the
  zImage wrapped in a uImage loading at `0xC1000000`, as its `bootm` does)
  and `flash-file=` for the MTD partitions, until the vendor application
  repeats the check and reboots.
- OpenIPC `gk7102` lite (`-kernel` / `-initrd`, HAL from an SDK U-Boot):
  login prompt, DHCP, `reboot`, SPI-NOR erase / program from Linux written
  back to the image, I2C reads of a `sensor=jxh42` ID.

## GK7101S / GK7102S / GK7102C

The S die (`mach-gk710xs`, U-Boot `board/goke/gk7101s`) runs the same
drivers with a different HAL (`gk7101s_hal_data.h`, version 20160804). The
GK7102C cameras are this family: their U-Boot (2012.10, "GK7102C
devkit-v1.1", 2020) carries that HAL byte for byte. Machines `gk7101s`
(128 MiB) and `gk7102s` (64 MiB).

**Where the blocks are.** The kernel does not use the HAL for most
blocks. `mach-gk710xs/include/mach/gk710xs.h` sets `*_HAL_MODE 0` for
all of them except the UARTs and SSI, so the VIC, timers, watchdog, IDC,
GMAC, DMA, SD, GPIO and RCT are plain loads and stores at the
`hardware.h` addresses — AHB `0x90000000`, APB `0xA0000000`, the same
offsets as the GK710x drivers' map — in the drivers' register layout.
The interrupt entry code reads the VIC's IRQ status at `+0x30` directly.
The S HAL's own register shuffles (VIC, eFuse, crypto, UARTs) matter only
for the UARTs, where it makes the same standard 16550 of them as the
GK710x HAL does.

**How they are modelled.** Same drivers, same Ambarella IP: an S block
is a GK710x model with the GK710x HAL applied in front of it. `gk-remap`
(`hw/misc/gk-remap.c`) is that front, for the VIC, the timers, the
watchdog, both IDCs and the GMAC. Its tables were taken from the GK710x
HAL under Unicorn. The S headers define the same offsets and bits as the
GK710x ones for these blocks, with one exception: `timer.h` swaps each
timer's overflow-interrupt and clock-select bits (S: `OFn = 0x40 >> n`,
`CSLn = 0x04 >> n`), so that row is composed with the swap. The VIC wiring
(kernel IRQ *n* = model line `irq_line[n]`) and the IRQ numbers are the
GK710x ones. The UARTs, sflash, internal PHY, GPIO, DDRC and USB sit at
their S addresses as they are.

| Block | S address | S layout (driver offsets) |
|---|---|---|
| VIC1 / VIC2 | `0x90008000` / `0x90009000` | SENSE 0x00, EVENT 0x04, BOTHEDGE 0x08, INT_SEL 0x0C, INTEN 0x10, INTEN_CLR 0x14, RAW 0x18, SOFTEN 0x1C, SOFTEN_CLR 0x20, PROTEN 0x24, IRQ_STA 0x30, FIQ_STA 0x34, EDGE_CLR 0x38 |
| Timers | `0xA0000000` | T1 STATUS 0x00, MATCH1 0x04, MATCH2 0x08, CTR 0x0C, T2 0x14–0x1C, T3 0x20–0x28, RELOAD 0x30 / 0x34 / 0x38; CTR: EN1/2/3 0x400/0x200/0x100, OF1/2/3 0x40/0x20/0x10, CSL1/2/3 0x04/0x02/0x01; 69 MHz |
| Watchdog | `0xA0006000` | CONTROL 0x00 (EN 1, INT 2, RST 4), TIMEOUT 0x04, CLR_TMO 0x08, STATUS 0x0C, RELOAD 0x10, RESTART 0x14 (`0x4755`), RST_WD 0x18 |
| IDC / IDC2 | `0xA0003000` / `0xA0004000` | PSLL 0x00, PSLH 0x04, FMCTRL 0x08, CTRL 0x0C (STOP 1, START 2, ACK 4, IF 8), STS 0x10, ENR 0x14, DATA 0x18, FMDATA 0x1C |
| GMAC | `0x9000E000` | MAC CFG 0x00, GMII_ADDR 0x04, GMII_DATA 0x08, FRAME_FILTER 0x0C, HASH 0x10/0x14, FLOW 0x18, MAC0 0x40/0x44, VERSION 0x58; DMA TX/RX poll 0x1000/0x1004, BUS_MODE 0x1008, RX/TX list 0x100C/0x1010, STATUS 0x1014, INTEN 0x101C, OPMODE 0x1020 |
| DMA | `0x90003000` | INT 0x304, DA(n) 0x308 + 4n, CTR(n) 0x31C + 16n, SRC 0x320 + 16n, DST 0x324 + 16n, STA 0x328 + 16n (`gk-dma`) |
| UART0 / 1 / 2 | `0xA0005000` / `0xA001F000` / `0xA001E000` | 16550, `regshift 2`, 24 MHz |
| sflash | `0x90016000` | as on the GK710x |
| EPHY | `0x90022000` | as on the GK710x |
| DDRC | `0x90002000` | `+0x04` = `0x00100344` (64 MiB) / `0x00100354` (128 MiB), read back by U-Boot's `dram_init` |
| SDIO0 / SDC1 | `0x90000000` / `0x90010000` | the vendor `sd.h` layout, below |
| RCT, GPIO, PMU, ... | `hardware.h` | storage (`hisi-regbank`) |

**DMA.** The GK7102C U-Boot checks the DMA controller before anything
else. It builds one descriptor at `0xC1000000` (from the I2S FIFO at
`0x9000A080`, 32 bytes, EOC), writes it to channel 1's DA (`+0x30C`) and
`EN | D` to its CTR (`+0x32C`), and expects DA to have moved on to the
descriptor's `next` (or one past it). Otherwise it prints `DMA reboot`
and resets through the watchdog. It touches the controller directly, not
through the HAL. `gk-dma` models the controller in the S layout
(`mach-gk710xs/include/mach/dma.h`, descriptors from `plat-goke`
`gk_dma_req_t`). It runs a whole transfer when EN is set: descriptor
chains, report words, and the channel's INT bit and interrupt (DMA_IRQ
21).

**Boot header.** The format is the same, with `w1` `0x30360801` on the
SDK's S images and `0x304308EF` on the GK7102C. The GK7102C table also
uses `0xFFFFFFFF, n` (a delay) between the bytes it sends out of UART0
("\r\n.\r\n": the first console output). Its table first sets the RCT
PLLs and UART, then arms the watchdog (reload `0x2AEA5400`, CONTROL
EN | RST), then does the DDR set-up of the SDK's GK7102S table. That
watchdog does not reset the machine during the boot: a 90 s run reaches
`login:` and stays there.

**SD.** SDIO0 (`0x90000000`, SD_IRQ 8) and SDC1 (`0x90010000`, SD1_IRQ
22) are `generic-sdhci` behind `gk-remap`, in the vendor's register
layout (`mach-gk710xs/include/mach/sd.h`, U-Boot `arch-gk7101s`
`REG_SDIO_*`): halfword and word registers, and byte registers inside
them. The rows come from the GK710x HAL's `hw_writew`/`hw_writel` and
`hw_writeb` with their `hw_read*` counterparts. The byte accessors have
their own map, so the same address means different things by width:
`+0x2C` as a word is CONTROL00 (only bits 0–3 reach SDHCI `0x28`), while
`+0x2C`/`+0x2D` as bytes are host and power control. The S `sd.h` swaps
SD_XFR_MUL_SEL / SD_XFR_CTH_SEL (bits 4, 5) relative to the GK710x one,
and that row is composed with the swap. `-drive if=sd,index=0` puts a
card in SDIO0, `index=1` in SDC1.

| S register (driver offset) | width | SDHCI | bits (driver → SDHCI) |
|---|---|---|---|
| DMA_ADDR `0x00` | 32 | `0x00` | |
| XFR `0x04` | 16 | `0x0C` | 0→1, 1→2, 2→0, 4→5, 5→4 (BLKCNT_EN, AC12_EN, DMA_EN, MUL, CTH) |
| NIXEN `0x06` / EIXEN `0x08` | 16 | `0x38` / `0x3A` | EIXEN: 1→2, 2→3, 3→4, 4→1 |
| BLK_CNT `0x0A` / BLK_SZ `0x0C` | 16 | `0x06` / `0x04` | |
| NISEN `0x0E` / EISEN `0x10` | 16 | `0x34` / `0x36` | EISEN: 1↔2, 4↔5 |
| NIS `0x12` / EIS `0x14` | 16 | `0x30` / `0x32` | NIS: 1→2, 2→3, 3→1, 5↔6 |
| CMD `0x16` | 16 | `0x0E` | 4↔5 (DATA, CHKIDX); bit 2 not passed |
| TMO `0x18` (byte) | 8 | `0x2E` | |
| RESET `0x19` (byte) | 8 | `0x2F` | 0↔1 (CMD 1, ALL 2 → SDHCI ALL 1, CMD 2) |
| CLK `0x1A` | 16 | `0x2C` | |
| RSP0–3 `0x1C`–`0x28` | 32 | `0x10`–`0x1C` | |
| HOST `0x2C` (byte) | 8 | `0x28` | 2↔3 (8-bit, high speed) |
| PWR `0x2D` (byte) | 8 | `0x29` | |
| GAP `0x2E` / WAK `0x2F` (bytes) | 8 | `0x2A` / `0x2B` | GAP 1↔2, 4↔7; WAK 0↔1 |
| STA `0x30` | 32 | `0x24` | 1↔2, 8→9, 9→10, 10→8; 3–7, 12–15, 29–31 not passed |
| ARG `0x34` | 32 | `0x08` | |
| CAP `0x38` | 32 | `0x40` | 22↔23 (SUS_RES, DMA); 6, 14, 15, 19, 20, 28–31 not passed |
| AC12ES `0x3C` | 32 | `0x3C` | 0→1, 1→2, 2→3, 3→0; 5, 6, 8–31 not passed |
| DATA `0x40` | 32 | `0x20` | |

Reads are the exact inverse of the writes, bit for bit.

With a FAT card (MBR, one FAT32 partition) the GK7102C camera's own kernel
finds it: `mmc0: new ultra high speed SDR12 SD card`, `mmcblk0: p1`. It
mounts the partition, reads a file from it, and writes a 320 KiB file
that lands byte for byte in the image.

**Flash images.** As on the GK710x, the chip holds every word byte-swapped.
A dump read out by software (`sf read`, `/dev/mtd`) is in CPU order, and
the machine refuses it with that hint. `qemu-boot/run-gk7102s.sh`
converts one.

**Tested:** `roxiq.bin`, a GK7102C camera dump (7.09 MiB software dump,
GD25Q64C, padded to 8 MiB). U-Boot passes its DMA check, prints its
banner and `HAL: 20160804`, and probes the flash and the internal PHY.
At its prompt, `ping` and `tftpboot` (CRC32 matches the host file) work,
`sf erase/write/read` + `cmp.b` agree, and `reset` goes through the
watchdog. Autoboot loads the 3.4.43 kernel, which lays out
BSB/DSP/USR from the U2K words, takes the GPIO map, calibrates its delay
loop on timer 3 and mounts the squashfs root. Userspace brings `eth0` up
and starts the vendor application (which stops at the unmodelled DSP),
and `login:` appears. The one jffs2 CRC error is past the end of the
dump.

## Sources

- SDK `linux/kernel/3.4.43/arch/arm/mach-gk710x/include/mach/`:
  `hardware.h` (fictitious map), `irqs.h`, `timer.h`, `wdt.h`, `idc.h`,
  `eth.h`, `flash.h`; `gk710x.c` (HAL init, U2K words, restart);
  `irq.c` (VIC); `rct.c` (clocks).
- SDK `drivers/`: `tty/serial/gk_sio_v1_00.c`, `mtd/devices/
  gk_sflash_v1_00.c`, `net/ethernet/goke/gk_eth_v1_00.c`,
  `i2c/busses/i2c-goke_v1_00.c`, `usb/musb/`, `input/misc/gk_ir_v1_00.c`.
- SDK U-Boot: `board/goke/gk7101/gk7101.c` (HAL hand-off, DRAM size from
  DDRC `+0x04`), `arch/arm/cpu/arm1176/gk7101/`, `drivers/mtd/spi/
  gk7101_sflash.c`, `include/configs/gk7101.h`, `common/cmd_bootm.c`.
- The HAL itself, through `qemu-boot/gk710x-halmap.py`.
