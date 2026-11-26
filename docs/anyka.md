# Anyka AK39xx (`ak3918ev200`, `ak391xev300`, `ak391xev330`, `ak3918ev300l`, `ak3918av100`, `ak3918av100x`)

Anyka's IP-camera SoCs: one ARM926EJ-S with DDR2 in the package. The memory
map has not moved from the AK3918EV200 through the AnyCloud39EV300 parts to
the AnyCloud V500 (EV330), so one machine file (`hw/arm/anyka.c`) covers all
three generations with a table entry each. The AK3918EV300L and the
AK3918AV100 family (the vendor's "ASPEN" code, despite the EV300L's name not
an EV300) keep the map with two moves -- SPI0 to `0x21100000`, I2C 2 to
`0x201C0000` -- and a new clock block; a flag in the table entry (`aspen`)
selects those. The AV100 adds blocks on top (`av100`): a second MIPI/CSI
input, an NPU and a video pipe higher in the map (`0x212`--`0x215`).

## What the SDKs support

| Generation | Parts | Source | Kernel | Boot header |
|---|---|---|---|---|
| Cloud39E / EV200 | AK3918EV200 | `ak3918ev200-kernel` (4.4 port), `ak3918ev200-bsp` (U-Boot v2026.07 port), dumps | 3.4.35 vendor (dumps), 4.4.302 port | `AKSH_240` |
| AnyCloud39EV300 | AK3916EV300, AK3918EV300, AK3919EV300 (B and C silicon revisions), AK3916EV301 | `anycloud39ev300` SDK: kernel 3.4.35 board files, U-Boot 2013.10, spiboot | 3.4.35 vendor | `ANYKA_H3` |
| AnyCloud V500 / EV330 | AK3916EV330, AK3918EV330, AK3919EV330, and the EV331 trio (one `dt_compat` list, `mach-ak39ev330.c`) | `ak3918ev200-kernel` (`anycloud_ak39ev330*.dtsi`), dump | 4.4.192 vendor (DT) | `ANYKAH3B` |
| AnyCloud39AV130 SDK V1.03 (one tree for several lines) | AK3918EV300L, AK3918AV100 (N/P/X/XP), AK3918AV130 (A/L/N: kernel `CONFIG_MACH_AK3918AV130`, U-Boot `CONFIG_3918AV130_CODE`) | `ANYKA_130L_SDK`: U-Boot 2019.10 (`arch-ak3918ev300l`, `arch-ak3918av100`), kernel 4.4.302 (`anycloud_ak3918ev300l*.dtsi`), dump | 4.4.282 vendor (DT) | `AK_H322L` / `AKH322LS` (EV300L), `ANYKA322` (AV100, inferred from `ak_ddr_param`) |
| AnyCloud39AV100 SDK V1.12 | AK3918AV100 (N/P/X/XP), KM01 (N/D) | `ANYKA_AV100_SDK`: U-Boot 2019.10.0-V4.0.27 (`arch-ak3918av100`, `ak3918av100_nor_defconfig`), kernel 4.4.282 (`anycloud_ak3918av100*.dtsi`), prebuilt `image/` | 4.4.282 vendor (DT) | `ANYKA322` (confirmed: U-Boot `vectors.S` builds `ANYKAH3D`, the burn tool patches it to `ANYKA322`; `ak_secureboot` header template) |
| AK3918AV130 | AK3918AV130 | the AnyCloud39AV130 SDK above (kernel 4.4.302 drivers `ak_irq_v2.c`, `ak_timer*.c`, `ak_uartv2.c`, `ak_nor.c`; U-Boot `ak_sfcv2.c`, `ak_serialv2.c`), and one flash dump (owner report r-5uwdebux on openipc.org): U-Boot 2019.10.0-V4.0.27 built for it (`EVB_CBD_AK3918AV130_V1.0.0`), its DTs `anyka,ak3918av130` | vendor (DT) | `AKH322LS`, payload from `0x200` to `0x80DFFC00` |
| AK37D, AK37E | — | `#ifdef`s in the 4.4 tree's `map.h`, `irqs.h`, `ak_timer.c` only | — | — |

The `Aimer39_PDK_V1.1.00` archive (AK3918 datasheet and programming guide)
is header-encrypted and was not usable.

## Machines

| Machine | Chip ID (SYSCTRL +0) | DDR | Timers | Notes |
|---|---|---|---|---|
| `ak3918ev200` | `0x20150200` | 64 MiB | 5 | the 4.4 port also accepts `0x20120100` |
| `ak3916ev300` | `0x20160100` | 64 MiB | 5 | the AK3916EV301 dump boots here |
| `ak3918ev300` | `0x20160100` | 64 MiB | 5 | camera input: the HeimVision PE3020-W's JX-F37 (reset on GPIO 79), see below |
| `ak3919ev300` | `0x20160100` | 128 MiB¹ | 5 | |
| `ak3918ev330` | `0x20160101` | 64 MiB | 10 | eFuse reader, I2C v2, flash CS on GPIO 25 |
| `ak3919ev330` | `0x20160101` | 128 MiB | 10 | U-Boot of the dump prints `ramsize: 08000000` |
| `ak3918ev300l` | `0x53543400` | 64 MiB | 10 | "ASPEN": SPI0 at `0x21100000`, clock block v2; `socinfo` prints `0x535434` |
| `ak3918av100` | `0x53533500` | 64 MiB | 10 | ASPEN + AV100 blocks; AK3918AV100N/P, KM01N; boots the AnyCloud39AV100 SDK V1.12 firmware to login |
| `ak3918av100x` | `0x53533500` | 128 MiB | 10 | same die, 128 MiB; AK3918AV100X/XP, KM01D |
| `ak3918av130` | `0x53533500`¹ | 64 MiB | 5 at `0x21110000` | the AV100's map, plus its own interrupt controller and timers at `0x21100000` (`ak-av130-intc`), a FIFO UART (`ak-uart2`) and the SPI flash controller at `0x20120000` (`ak-sfc`); the dump boots to `login:`, see below |

The AV100 family all read chip ID `0x53533500` (`socinfo` `0x535335`,
"AK3918AV100-A"; the C revision is `0x53533501`, "-C"). The vendor tells the
parts apart not by this register but by the eFuse global ID and the DDR bank
count: `libplat_osal.so` (`osal_get_chip_type`) reads the eFuse model code
and `osal_get_mem_bank_num` reads a DDRC bit (`0x21000000`, `0x200` = 8
banks / 128 MiB, else 4 banks / 64 MiB). So KM01N and the 64 MiB AK3918AV100
parts map onto `ak3918av100`, KM01D and the 128 MiB X parts onto
`ak3918av100x`; the two machines differ only in DDR size.

¹ Not seen on a dump; `-m` overrides it.

```
qemu-system-arm -M ak3918ev300,flash-file=dump.bin -nographic -nic none
```

`flash-file=` is the camera's whole SPI NOR, booted the way the mask ROM
does. Erases and programs are written back to it, so boot a copy; for the
same reason a machine with one cannot be migrated or snapshotted (without
one, the flash contents travel in the migration stream).
`flash-jedec=0xEF4017` overrides the JEDEC ID the flash answers with (by
default the one the boot header's SPIP descriptor names, which is what the
vendor drivers check against). `-kernel` boots a kernel directly with ATAGs
for machine number 3892 (`anyka_ak39xx`, every board file uses it) or with an
appended DTB.

## Memory map

| Address | Block | Model |
|---|---|---|
| `0x00000000` | mask ROM, 32 KiB | synthetic ROM (below) |
| `0x08000000` | system control: chip ID, PLLs, clock gates, resets, the interrupt controller, timers, watchdog, RTC link, EV330 eFuse, SAR-ADC1 samples (`ain0` / `ain1` / `bat` in millivolts at the pin, default 3000 on AIN0/AIN1, settable with `qom-set` while the guest runs; see below) | `ak-sysctrl` |
| `0x20000000` | ISP / VI (interrupt 1) | EV200 and AK3918EV300 `ak-isp`, others storage |
| `0x20020000` | video encoder: EV200 a Hantro 8290 (H.264, JPEG; interrupt 2), EV300 an Allegro core (registers from `+0x8000`; interrupt 2) | EV200 `ak-h8290`, EV300 `ingenic-avpu` (`anyka`), others storage |
| `0x20100000` / `0x20108000` | MCI0 / MCI1; `-drive if=sd` is the TF slot: MCI0, but MCI1 on the EV300L and AV100 (their DTs put the card on mmc1, pads 31-36, power GPIO 50) | `ak-mci` |
| `0x20110000` / `0x20118000` | DAC / ADC | storage |
| `0x20120000` / `0x20128000` | SPI0 (boot NOR) / SPI1 | `ak-spi`, `ak-spinor` |
| `0x20130000` / `0x20138000` | UART0 / UART1 (UART2 at `0x201A0000`, interrupt 24, on the EV300L and AV100) | `ak-uart` |
| `0x20140000` | L2 buffer controller | `ak-l2` |
| `0x20150000`, `0x20190000`, `0x201A0000` | I2C 0 (1, 2 on EV330; EV300L: I2C 2 at `0x201C0000`, UART 2 at `0x201A0000`) | `ak-i2c` |
| `0x20170000` | GPIO | `ak-gpio` |
| `0x20200000` | USB 2.0 OTG (MUSB-style) | `ak-usb`: a MUSB host on the AV130 (`hcd-musb`), nothing plugged in elsewhere |
| `0x20300000` | Ethernet: Synopsys GMAC, RMII PHY at MDIO 1 | `xm-dwmac` |
| `0x21000000` | DDR2 controller | `ak-ddrc` |
| `0x21100000` | EV300L / AV100: SPI0 (boot NOR), the old SPI0 register set plus `+0x20` RTIM and `+0x24` WORK_MODE | `ak-spi`, `ak-spinor` |
| `0x201A8000` / `0x58000000` | AV100: second L2 buffer controller / its SRAM (`l2-buffer_1`, interrupt 25; disabled on the single-sensor boards) | unimplemented (reads 0) |
| `0x20400000` / `0x20500000` | AV100: MIPI0 / MIPI1 receiver (the GPIO node's second and third `reg`) | `ak-mipi` storage / unimplemented |
| `0x21200000`--`0x21500000` | AV100: VI/ISP (`0x21200000`), NPU/CNN (`0x21300000`), video encoder (`0x21400000`), encoder bridge (`0x21500000`) | the `ak-apb` catch-all (reads 0) |
| `0x48000000` | L2 memory, 8 KiB, mirrored over 64 KiB | `ak-l2` |
| `0x80000000` | DDR (64 MiB; 128 MiB on `ak3919*`, `ak3918av100x`) | RAM |

Interrupts are SYSCTRL's: module sources 0..23 in `+0x24` (enable) and
`+0x4C` (status), source 3 being the "system" sub-controller, whose sources
(SAR ADC 0, timers 1..5 on bits 5..1, RTC ready 7, EV330 timers 6..10 on bits
15..11) are in `+0x2C` / `+0x30`. Module numbers: 1 ISP, 2 VENC, 4 MCI0, 5
MCI1, 6 ADC, 7 DAC, 8 SPI0, 9 SPI1, 10 UART0, 11 UART1, 12 L2, 13 I2C, 15
GPIO, 16 MAC, 18/19 USB.

## Boot header and the mask ROM

The first 512 bytes of the NOR (U-Boot port `mkaksh.py`, and every dump):

| Offset | Content |
|---|---|
| `+0x00` | branch |
| `+0x04` | magic `AKSH_240` / `ANYKA_H3` / `ANYKAH3B` / `AK_H322L` (EV300L) / `ANYKA322` (AV100) |
| `+0x0C` | payload length (it follows the header) |
| `+0x20` | load address: `0x80E00000` (U-Boot, EV200/EV300/EV300L/AV100) or `0x48000000` (EV330 first stage) |
| `+0x2C` | register script: `{address, value}` pairs; address `0x66668888` = delay, `0x66668881` = three-word poll, `0x66668880` = three-word read-modify-write `{marker, register, mask}` (the AV130 dump clears bit 2 of DDRC `+0x24` with it), `0x66668882` = a ROM operation of one value; all skipped; `0x88888888` = end |
| `+0x18C` | `SPIP` + the SPI NOR descriptor (JEDEC ID, size, page, erase size, partition-table block) |

The AV100's ROM copies from `+0x200`, as the others' do. `vectors.S` under
`CONFIG_3918AV100_CODE` reserves `0x400` bytes of secure-boot space behind
the header (public key at `+0x200`, signature at `+0x300`;
`tools/secureboot/ak_secureboot`), and the vendor's header covers it: load
address `0x80DFFC00`, length from `+0x200`, so U-Boot at flash `0x600` lands
at `0x80E00000` and the ROM enters through the zeroes (`andeq`); the vendor
U-Boot is linked at `0x80DFFA00`, the header's address. That is the one AV100
flash dump seen, owner report r-s9hgnp86 (AK3918AV100N, U-Boot
2019.10.0-V4.0.21), the same as the AV130's below. The model used to copy an
`ANYKA322` payload from `0x600`, inferred from the SDK before any dump: it left
that dump's U-Boot `0x400` below where it runs (no output at all), and booted
an OpenIPC header made to the same inference, load `0x80E00000` with the
length from `0x600`, which on the chip puts U-Boot `0x400` too high and its
last `0x400` bytes uncopied. (Signed images also write `0x84000000` at
`+0x1C`; unsigned ones keep `6`.)

The ROM replays the script (PLLs, DDR controller), copies the payload to the
load address and enters it. The EV300L header has the EV300's fields (its
U-Boot, linked at `0x80DFFE00`, carries it: `vectors.S` writes the generic
`ANYKAH3D` and the burn tools patch magic and script); its payload, 0x49000
bytes, takes in ENV, ENVBK and DTB behind U-Boot; its SPIP descriptor is
empty, so the flash answers as a GD25Q64 unless `flash-jedec=` says
otherwise. It also leaves the header in L2 memory at
`0x48000200`, where the old EV200 `spiboot` loader looks for `SPIP`. The
model is a small program at address 0 that the CPU runs out of reset, because
the script has to land after the device resets: the EV330 script leaves its
first stage the load parameters (SPI setup, length 0x40000, destination
0x80E01000) in timer registers.

Word `0x24` of the stub ROM is a pointer into the ROM itself (`0x7F00`),
and the script table starts at `0x28`. The EV300L vendor U-Boot's
`ak_spi_set_mode()` takes the platdata of the SPI bus's *parent* -- NULL --
and read-modify-writes SPICON at "NULL->regs", the ROM word at `0x24`; on
silicon that is a harmless ROM access, with a script word there it was an
alignment abort and a reset loop at `Flash:`.

EV330 boots in two stages: the first (4 KiB in L2 memory) runs the second
header's DDR script itself, polls DDRC `+0x24` for "done", and streams U-Boot
from flash offset 0x1200 through L2 buffer 0 by DMA.

## Device notes

- **EV300L clock block** (U-Boot `clk-ak3918ev300l.h`): 24 MHz crystal;
  PLL1 (CPU, `+0x04`), PLL2 (`+0x08`, GCLK) and PLL3 (`+0x14`) are each
  24 MHz × (M + 1) / (N + 1) / (OD + 1) with M 12:0, N 18:13, OD 22:19.
  GCLK = PLL2 / (2 × (bits 24:23 + 1)). Self-clearing: `+0x04` bits 30 /
  29 / 28 (apply PLL1 / PLL3 / PLL2), `+0x08` bit 25 (GCLK divider valid),
  `+0x0C` and `+0x10` bits 29 / 19 / 9 (divider valid; `+0x0C` 19 is SPI0's
  high-speed clock, divider in 17:10, enable 18, its reset `+0x20` bit 31),
  `+0x18` bit 9 (MAC OPCLK). The header script leaves PLL1 at `0x24`
  (888 MHz); the model resets PLL2 to 240 MHz / GCLK 60 MHz and the vendor
  U-Boot sets 400 / 200 MHz. Pad registers run to `+0x290`. Gates (`+0x1C`,
  1 = off) and resets (`+0x20`): MMC0 1, MMC1 2, SPI0 5, SPI1 6, UART0 7,
  L2 buffer 0 9, GPIO 12, MAC 13. The interrupt controller, timers (EV330
  layout, 12 MHz), watchdog (`+0xE4` / `+0xE8`), UART and L2 buffer
  numbering are the old ones.
- **Watchdog** (`ak-sysctrl` `+0xE4` / `+0xE8`, 21.333 us a count, at most
  357 s). `ak_top_wdt.c` writes the load to CFG1 (`0x55` key), then CFG2
  enable, then enable + feed (`0xAA` key); magic close writes CFG2 `0xAA`
  with enable clear, which stops it. It is also how the board reboots: the
  kernel's restart hook feeds it a two-count load. That case (under 1 ms)
  resets quietly. A longer load that runs out is an expiry and says so on
  stderr, e.g. `ak-sysctrl: watchdog expired: last fed 300.000 s ago with
  a 300.000 s timeout`, and goes through QEMU's watchdog action, so
  `-action watchdog=pause` stops the machine where it was instead of
  rebooting it. Expiries every 7.5 minutes or so with Majestic's 300 s
  watchdog on are the guest's tick, not this model. The EV300 kernel's
  clock event handler cleared the timer's expired flag after programming
  the next event, so a vCPU stall of over 1 ms lost that event, and with it
  the tick for good. Jiffies stop (`/proc/timer_list`), the `ak_ce_timer`
  count in `/proc/interrupts` stops, `/proc/uptime` falls back by 358 s at
  each wrap of the clocksource, and the 150 s keepalive never comes due.
  The fix is OpenIPC/linux#325 (`ak_timer.c`: clear first).
- **SAR-ADC1** differs between the chips. The EV330 (and the model's
  default) leaves three packed 10-bit samples of the 3.3 V AVCC in `+0x68`
  all the time. The EV300 parts (`sar-ev300`, OpenIPC/builder#234) convert
  one input at a time: `+0x5C` bit 0 starts a conversion of the input
  selected in bits 7:5 (bit 5 AIN0), `+0x6C` bit 0 says it is ready
  (written 0 to clear), and `+0x68` bits 11:0 hold 12 bits against 3.0 V.
  The input is read only when the converter is clocked (`+0x0C` bit 3), out
  of reset (`+0x20` bit 30), powered (`+0xA0` bit 0 clear), and the input
  is enabled (`+0xA0` bit 5, AIN0 only); otherwise the sample is 0. That is
  the 0 mV a kernel with the EV330 reader gets on an EV300, so the twin
  shows it too. A lit PE3020-W photocell (3000 mV) saturates at 4095,
  2999 mV, as the camera does; `qom-set ... ain0 39` is the same photocell
  covered.
- **Fault injection** for the camera's stop paths (QOM properties, `-global`
  at start or `qom-set` while the guest runs; not migrated):
  - `ak-isp` `stuck-busy`: STATUS bit 14 (ISP_UNDER_WORKING) reads set, as
    for a core that never finishes the frame it is writing. The capture
    stop waits 200 ms for it, then gives up: `[ak_isp_vi_stop_capturing]
    reg10 b14 not clear`.
  - `ingenic-avpu` `drop-irqs`: a finished picture raises no interrupt, as
    if the line were lost (counted in `irqs-dropped`). libakv's wait for it
    never returns, and Majestic logs `anyka: the capture thread is stuck in
    the encoder; releasing it` when it stops.
  - `ingenic-avpu` `resets` (read-only) counts the video resets:
    `ak-sysctrl` `+0x20` bit 20 drives the encoder's `reset` GPIO, and
    holding it puts the core back at its power-on state. OpenIPC's
    `uio_vcodec` pulses it for 5 ms on open.
- **UART** (`ak-uart`) has no data register. TX bytes go into L2 memory at
  `0x48001000 + 0x100 n`, CONF2 starts the send; received bytes land in a
  128-byte ring at `+0x80`, and DATA_CONF bits 19:13 say where the next one
  goes. CONF2 bit 19 must be a live "transmitter idle" flag: the 3.4 kernel's
  `start_tx()` only enables the TX-done interrupt, and if `set_termios()`
  had cleared a latched flag, getty would never print.
- **SPI** (`ak-spi`) is a byte-wise PIO master; the drivers shift opcode,
  address and dummy byte out as TX bytes. QEMU's m25p80 counts fast-read
  dummies in clock cycles for some vendors and not at all for GigaDevice, so
  the data came back shifted; `ak-spinor` takes every dummy as one byte and
  answers to any JEDEC ID (the ZB25VQ64 and XM25QH64A have no m25p80 part).
  EV330 kernels drive the chip select on GPIO 25 (`cs-gpios`), the others
  with CON bit 5.
- **L2** (`ak-l2`): eight 512-byte buffers, each a ring of 64-byte blocks
  with a fill counter. The CPU signals a block by touching its last word
  (write fills, read drains), the DMA engine moves whole blocks and a
  "fraction" DMA the tail. Kernels move any SPI transfer of 256 bytes or
  more this way. The memory is a ROM device: RAM to reads (code runs there
  at boot) until a buffer is enabled.
- **GPIO** inputs read 1 (pull-ups): "upgrade button not pressed", "no SD
  card". With 0 there, the EV300 U-Boot goes to SD update.
- **I2C**: two layouts. EV200/EV300 (CMD1 + DATA at `+0x20`, done = CTRL bit
  4, polled with no timeout) and EV330 (`ak3790d-i2c`, address at `+0x08`,
  data at `+0x10`, done = bit 13, NAK = bit 27). Absent sensors read 0xFF.
- **Ethernet** is a Synopsys GMAC under a synopGMAC-derived driver (SDK
  `drivers/net/ethernet/ak-ethernet`; on EV330 the `ak_eth.ko` module from
  the APP partition). It is the `xm-dwmac` model Xiongmai and Goke use,
  with two things the Anyka driver needs: the descriptor skip length
  (`DmaDescriptorSkip2` -- its descriptors carry two software words, 24
  bytes apart) and ANER bit 0, without which it settles on half duplex.
  The driver takes 4 off each received length, so the core keeps the FCS.
  `-nic user` gives the guest a network; the vendor U-Boot (EV300) has the
  driver too, so `ping` works there.
- **MCI** is a PL180-like controller with its own bit layout; STATUS clears
  when read, MASK at `+0x38` gates the interrupt. With no card every
  command times out, which is how U-Boot and the kernel see an empty slot
  (both stall polling the old storage stub). Data goes through the FIFO;
  the kernels' L2 DMA path for SD is not modelled yet.
- **USB**: the vendor scripts load the host driver to look for a USB Wi-Fi
  dongle. As plain storage the block echoed the driver's bits back as
  connect events and the hub layer retried forever.
- **EV200 video encoder** (`ak-h8290`): the Hantro 8290 that OpenIPC's
  in-kernel V4L2 mem2mem driver `ak-venc` drives (`8290 encoder at
  /dev/videoN, irq 2`); the register map is the driver's
  `ak-venc-regs.h`, summarised at the top of `hw/misc/ak-h8290.c`. swreg0
  reads `0x82900760`. Writing swreg14 with bit 0 set starts a job, which
  completes 1 ms of virtual time later (the driver programs the next job
  from its interrupt handler); swreg1 bit 0 is the line, bit 2 frame
  ready, bit 5 buffer full, bit 3 an error. The core continues from the
  last whole 64-bit word of the header the driver wrote (swreg22 / 23,
  bit count in swreg37 [28:23]), and swreg24 reads back the bits written
  from there. H.264: a slice NAL with its own start code, its header from
  the registers (16-bit frame_num, idr_pic_id, slice type, QP against the
  PPS QP, deblocking), I_16x16 / P (skip, inter, intra) macroblocks from
  `hantro-h264.c`, predicted from the reference store the driver names and
  reconstructed into the other, so the driver's store rotation matters as
  on silicon. JPEG: `jpeg-enc.c`'s 4:2:0 scan with the quantisers from
  swreg64..95, Annex K tables, EOI. Input I420 (or NV12), height not a
  multiple of 16 through the bottom overfill. Not modelled: the core's
  own rate control (MAD QP adjustment, checkpoints, ROI: the frame is
  coded at the initial QP), the clock gate / soft reset (the id reads
  regardless), slices, restart markers, rotation, RGB input.
  `-global ak-h8290.dump-dir=DIR` keeps each job. `qemu/tests/test-ak-h8290.py`
  plays the driver under qtest and decodes the result with ffmpeg.
- **EV300 video encoder** (`ingenic-avpu` with `anyka`, reg-base
  `0x8000`): an Allegro core without MCU, the lineage of Ingenic's AVPU.
  The SDK's closed `libakv_encode` (2.2) runs
  Allegro's control software on the CPU and reaches the core through
  OpenIPC's `uio_vcodec` (`/dev/uio0`: the 64 KiB window mapped, one
  interrupt status bit returned per `AKUIO_WAIT_IRQ`). Its contract, read
  off the library's objects: the list address / start of core c at
  `+0x8080 + 16c` / `+0x8084 + 16c` (Enc1 2, Enc2 8, JPEG 0x10), Enc1
  then Enc2 started on the same list with W8 bit 11 (Enc1 feeds Enc2 on
  chip; Enc2 writes the stream after the slice header the library put in
  the buffer); list entries 0x1C0 bytes, the slice status written back at
  0x100-0x16C of each; the core's status at `+0x8300` (busy bits 0 / 2,
  bit 1 overflow) and the picture's bytes at `+0x8304`, which the
  interrupt handler reads to re-code a picture that overflowed; JPEG's
  R0-R10 at `+0x83C0`, its status / bytes / overflow at
  `+0x83F0`-`+0x83F8`, done on interrupt bit 18 (Enc1 / Enc2 on 0 / 2;
  for H.264 on its one core the library unmasks Enc2's alone, so a
  picture ends with its entropy coding). The L2 prefetch (`+0xC004` control: bit 0 on, bits 10 / 11
  bypass; `+0xC008` / `+0xC00C` the source planes, `+0xC014` [18:6] the
  pitch; `+0x9204` / `+0x9208` written with it) is set up per picture
  for the channel the library gives the L2 to, with that picture's own
  source; the model keeps the registers and codes from the planes the
  list names, and logs a guest error if an enabled L2 points elsewhere.
  The library reads no ID register, only `+0x801C` (a debug switch,
  0). Majestic's 1080p25 main and 640x360 sub channels and its JPEG
  snapshots run on it; `qemu/tests/test-ak-allegro.py` plays the library
  under qtest. Not modelled: the reset pulses at `+0x8010` and the clock
  gate at `+0x8020` (registers only), SYSCTRL's encoder reset.
- **ISP2** (`ak-isp`, EV200 and AK3918EV300): the frames of the ISP2 as
  OpenIPC's kernel drives it through the SDK-derived `ak39_isp2.c` -- on
  EV200 from `ak-cam`, on EV300 from the soc_camera host `ak_camera.c` --
  not the image processing. Most of its configuration is a register table
  in DRAM (`+0x18` holds its address), fetched when `REG_UPDATE` (`+0x1C`)
  is written: a table written but not loaded is not seen. The model reads
  the crop window (block 1 words 0 / 1), the buffer addresses (block 5
  words 25..30 for buffer 1, block 1 words 31..48 for 2..4), the main /
  sub sizes and the per-channel frame control from it. `GEN_CTRL` bit 20
  starts capture; every frame period (`-global ak-isp.fps=N`; 15 on EV200,
  25 on EV300) the core takes the crop window out of the sensor's frame,
  scales it to each channel, writes a test pattern -- bars, a moving
  block, the frame number as 16 black/white cells along the bottom -- into
  the buffer it is on, raises `FRAME_ALL_DONE`, and reports the next
  enabled buffer in `STATUS` 23:22, the order `ak_isp_irq_work()` expects.
  The two parts differ (`ev300=on`, the driver's `CONFIG_CPU_AK3918EV200`):

  | | EV200 | EV300 |
  |---|---|---|
  | buffer enables | `GEN_CTRL` 15..18 main, 21..24 sub | `+0x14` 31..28 main, 27..24 sub, 23..20 channel 3, buffer 1 at the top |
  | register table | 3424 bytes, block 4 / 5 at 3152 / 3292 | 3540 bytes, block 4 / 5 at 3188 / 3396 |
  | channel sizes | block 4 words 31 / 33, 12 + 12 bits | block 4 words 48 / 50, 13 + 13 bits |
  | chroma | `UV_MODE` clear: I420 | `UV_MODE` set (`0x7 << 10`): NV12 at the U address |
  | sensor | `sc1345` on I2C 0 | `jxf37` on I2C 0 at 0x40 |

  The model holds the driver to what it programs, as silicon does. An empty
  crop window (the EV300 release kernel sets it only from `VIDIOC_S_CROP`)
  writes nothing, though the frame completes; the sub picture goes where
  the table says, which on EV300 is `main + w * h * 3 / 2`, outside the
  V4L2 buffer, at the size `VIDIOC_S_PARM` raw `{1, w, h}` set (none: not
  written); on EV300 a sensor that has not been set up or is held in reset
  sends no frames. Not modelled: statistics (the 3A reads zeros), 3D-NR,
  OSD, channel 3, the MIPI receiver (frames flow whatever the D-PHY at
  `0x20400000` is set to), the scaler's limits, frame-control values other
  than 0 and `0xFF`. The comment at the top of `hw/misc/ak-isp.c` lists
  what is a guess. `qemu/tests/test-ak-isp.py` drives both parts under
  qtest.
- **The AK3918EV300 camera input** is the HeimVision PE3020-W's
  (`ak3918ev300-heimvision-pe3020w.dts`, OpenIPC/builder#234): the JX-F37
  on I2C 0 at 0x40, its active-low reset on GPIO 79. The sensor model
  (`hisi-f37`) has a `reset-n` input: while the pad is low it answers
  nothing (the ID reads `0xff/0xff`, as it did on the camera with the
  EVB's GPIO 49 in its device tree) and comes back with its registers at
  their defaults. `sensor=` picks another catalogue sensor and
  `sensor-reset=N|none` the GPIO (the TECKIN TC100: `sensor=jxf23,
  sensor-reset=49`). The released PE3020-W image probes it as on the
  camera:

  ```
  f22_set_power_on
  aksensor 0-0030: probe_sensors Probing Sensor ID: 0xf37
  f22_set_power_on
  ak_camera 20000000.isp: AK Camera driver attached to camera 0
  ak_camera 20000000.isp: AK Camera driver detached from camera 0
  ak_camera 20000000.isp: AK Camera driver loaded
  ```

  and when Majestic opens `/dev/video0`, `ak_camera 20000000.isp: ISP
  tuning /etc/sensor/jxf37.conf, 2 styles` (in the kernel log; the console
  is quiet by then). That release's Majestic then stops where the camera
  did: `anyka: sensor 1920x1080@25`, `/proc/self/pagemap reports no frame
  numbers ...`, `Cannot start SDK`, before it starts the ISP.
- **The EV200 camera pipeline end to end**: with both models, OpenIPC's
  ak3918ev200 image boots to `ak-venc` on `/dev/video0` and `ak-cam` on
  `/dev/video1` / `/dev/video2` as on the board, and Majestic's EV200 path
  streams H.264 over RTSP (main and sub) and serves JPEG snapshots. The
  capture pool is the video memory above `mem=` and has no struct pages,
  so the kernel cannot export its buffers as dma-bufs (`failed to get
  scatterlist from DMA API`): hand them to the encoder as USERPTR.

## AK3918AV130

The one AV130 seen is a raw 8 MiB read sent to openipc.org (owner report
r-5uwdebux). It was taken for an AV100 at first -- its U-Boot is the AV100
SDK's 2019.10.0-V4.0.27 and its header magic `AKH322LS` -- but both its DTs
say `anyka,ak3918av130` (board `EVB_CBDM_AK3918av130`; U-Boot finds 64 MiB,
the AV130N), and the
blocks it drives are not the AV100's:

- **The header.** Payload length `0x40000` from flash `0x200` to
  `0x80DFFC00`: the 0x400 bytes of secure-boot space go first, so flash
  `0x600` lands at `0x80E00000`, which is where U-Boot is linked. The ROM
  enters at the load address and runs through the zeroes (`andeq`) to it.
  Its script ends with the three-word `0x66668880` op; read as a pair, it
  became a store of `0x88888888` to `0xFFFFFFFB` and a data abort in the ROM.
- **Interrupt controller and timers** at `0x21100000`
  (`hw/misc/ak-av130-intc.c`; the SDK kernel's `ak_irq_v2.c` and
  `ak_timer_common.h` under `CONFIG_MACH_AK3918AV130`). The system
  controller no longer takes the interrupts:
  - **Top bank:** `+0x00` FIQ mask, `+0x04` IRQ mask (1 = enabled),
    `+0x08` status. Devices keep the old numbers (MCI 4/5, SFC 8, UART0 10,
    GPIO 15, MAC 16).
  - **Sub-banks** behind it, each a mask/status pair: TWI (I2C) at
    `+0x0C`/`+0x10` on top bit 13, SSS `+0x14` (bit 17), NSS `+0x1C`
    (bit 0), ISS `+0x24` (bit 1), ESS `+0x2C` (bit 2). A DT specifier is
    `<bit bank>`. The system controller's sub-bank (bank 1, `0x0800002C`)
    is not modelled.
  - **Five timers** at `+0x10000 + n * 0x1000`, 12 MHz, counting down, on
    top bits 27-31. Registers: `LOAD`, `VALUE`, `CTRL` (0 enable, 2:1
    free / periodic / one-shot, 3 reload now), `INTCLR`, `INT_EN`, `DIV`,
    `INT_STA`. Timer 0 is U-Boot's `get_timer()` and the kernel's
    clocksource, timer 1 its clock event. Without them U-Boot's first
    `udelay` never returns and the kernel never ticks.
- **UART** (`anyka,ak3918av100plus-uart` in the U-Boot DT,
  `anyka,ak3918av130-uart0` in the kernel's, at the old addresses): a FIFO
  UART with a data register, not the L2-buffer UART (`ak-uart2` in
  `hw/char/ak-uart.c`, from the SDK's `ak_serialv2.c` and `ak_uartv2.c`):
  - `CFG` bits 28/29 clear the RX/TX FIFO and clear themselves; the
    kernel waits on 28 when the console opens.
  - Status: TX empty (1) and TX end (17) after a send; RX threshold (8)
    and timeout (10) while bytes wait. `0x20` enables each status bit as
    an interrupt.
- **SPI flash controller** `sfc@20120000` (`hw/ssi/ak-sfc.c`), where the
  older parts had SPI0: a list of command banks (`0x200 + 0x40 n`: opcode,
  address, mode, data, match mask / data, interface, lengths) that it runs
  against the boot NOR on its SSI bus, one chip-select frame each. Data
  moves through a bank's own register (up to 4 bytes: RDID, status), the
  `0x184` FIFO, or L2 common buffer 1 by DMA (BUFASSIGN device 0, 64-byte
  blocks), which is how U-Boot reads the kernel. A bank with bit 27 polls
  (RDSR until WIP is clear). `0x0D0` bit 4 is done (write 1 to clear),
  `0x100` bit 1 acknowledges the XIP stop, bit 4 resets the FIFO. The XIP
  window at `0x30000000` reads with 0x03; this U-Boot never uses it. All
  of it from the U-Boot driver's behaviour; no datasheet.
- **USB** (`usb@20200000`, interrupts 18 and 19): a Mentor MUSB core,
  modelled as a host with one port (`hw/usb/hcd-musb.c`, QEMU v9.0.0's
  model that upstream dropped with the TUSB6010 boards, behind `ak-usb`).
  `-device <usb device>,port=1` plugs into it. The `port=1` matters: without
  it QEMU puts a full-speed `usb-hub` on the only port, and the device is
  enumerated behind it at full speed. The vendor's prebuilt `ak_hcd`
  (`ak_otg_hshcd.c`) drives the core in ways the upstream model did not
  follow, so the model changed in five places:
  - it writes the CSRs a byte at a time, and a write to a CSR's high byte
    does not resend its packet;
  - it runs EP0 with INTRTXE clear and polls INTRTX, so INTRTX/INTRRX
    latch whatever the enables say;
  - it names the device in FADDR, as a core built without multipoint does;
  - every REQPKT is an IN token, so a control transfer's status stage
    reaches the device;
  - FLUSHFIFO is a strobe.

  The DMA block at `+0x200` is storage, and FIFOs are 64 bytes, so this
  covers enumeration, not bulk traffic. With
  `-device atbm-wifi,pid=0x6162,port=1`, the high-speed ATBM6132 as the
  Yoton WS03 has it (OpenIPC/firmware#2962), the vendor's
  `usb-common`/`usbcore`/`ak_hcd` enumerate `007a:6162`.
  OpenIPC's own `usbcore` gives the board's
  `USB_PID_IN(OverFlow): len=18, ep->length=0` / `device descriptor
  read/64, error -75` until `struct usb_device` is laid out the SDK's way.
  The cause: the SDK kernel moves `dev` to the end, and `ak_hcd` reads
  `ep_in[]` at offset 136.
- **Elsewhere:** the GMAC is at `0x20240000` (`ethernet@0x20240000`), and
  the I2C interrupts are on the TWI bank.
- **Result:** U-Boot detects the GD25Q64, loads its environment, reads the
  4.4.302-cip94 kernel (checksum OK) and the DT, and starts it. The kernel
  brings up `ttySAK0`, reads the NOR through the SFC v2 by L2 DMA
  (`Anyka SFCv2 driver register success`), mounts the squashfs root, and
  reaches `AnyCloud-uc1_1 login:`. The vendor modules load. The app's
  `device_id_decryption` step then fails ("Read and decrypt failed",
  eleven tries) and its daemon asks for a reboot. It never touches the
  crypto engine at `0x20180000` (no access, and neither `ak_crypto.ko` nor
  `cryptodev.ko` is loaded); what it misses is its encrypted device ID.
  That is left failing on purpose: making it succeed would recover a
  camera's cloud identity from an owner's dump.
- **Reboot** (`ak_pm.c`, `L2_LINK(standby)` with `param1` set, entered
  from `ak3918av130_restart()` through `l2_exec_buf()`): the kernel copies
  the code into L2 memory, prints `Y` when the copy checks out, and jumps
  to it at its mapping, `AK_VA_L2MEM` = `0xF1000000`.
  `REBOOT_DISABLE_CACHE_MMU()` then turns the MMU off with an `mcr`. The
  next instruction, `mov pc` to the same code at `0x480000xx`, is one the
  ARM926 has already fetched; QEMU fetches it again, at physical
  `0xF1000000`, so the machine maps L2 memory there too.
  `REBOOT_SELF_REFRESH()` then puts the DRAM in self-refresh (`ak-ddrc`:
  `0x21000010` bit 23 reads back done) and starts the watchdog
  (`SYSCTRL +0xE4 = 0x55000009`, `+0xE8 = 0xAA000003`), whose reset
  boots the flash again. On the dump this is a loop: login, the
  device-ID step, a watchdog reset about every ten seconds.
- **Seen, not fixed:** typed console input reaches OpenIPC's AV130
  kernel only as the first byte. UART0's data register holds it, interrupt
  10 stays masked, and the shell never answers. Scripted runs use
  `init=` a script in the rootfs instead.
- **Not modelled:** the crypto engine itself, the system controller's
  interrupt bank, and the XIP windows. The DT also has a data mover
  (`0x21120000`) and a remote processor (`0x21102000`).

¹ The AV100's. The AV130's is at SYSCTRL `+0x3FC` (`0x56573500`), where ipctool reads it.

## Dumps

Vendor firmware, each booted with `flash-file=` on a copy:

| Dump | Machine | Result |
|---|---|---|
| `3918.bin` (AK3918EV300, U-Boot AK_V3.0.08) | `ak3918ev300` | root shell (`root`, no password); `/dev/mtdblock1/4/6` md5 match the image; partition C's squashfs id table is erased in the dump itself |
| `qh64.Bin` (EV300) | `ak3918ev300` | login; same truncated partition C |
| `25Q64JVS.BIN` (V380E, EV300) | `ak3918ev300` | login, vendor app |
| `fullflash_…_ZB25VQ64_…_AK3916EV301` | `ak3916ev300` | login, Tuya app |
| `V380E2_…` (AK3918EV200 + U-Boot) | `ak3918ev200` | login, vendor app with audio decode |
| `fullflash_…_MX25L6405D_…_AK3918EV200` (spiboot) | `ak3918ev200` | login, vendor app |
| `win25q64jvs10_…` (EV200, spiboot) | `ak3918ev200` | login; an old audio library reports `CHIP(106552) unsupported` |
| an AK3918EV201 camera with AliOS Things (4 MiB BY25Q32ES; header `AKSH_240`, U-Boot 2019.10.0-V4.0.25 `S200W_EVB_CBD_AK3918EV201`, DTB `anyka,ak39ev200`) | `ak3918ev200`, `sensor=sc1346` | AOS-R-3.4.3 mounts its romfs and littlefs partitions and starts the vendor app: the `sc1346` driver takes the sensor, the ISP opens at 1280x720, eth0 links. Its clock code sets the CSI SCLK divider in SYSCTRL `+0x18` (bits 15:10, enable 18) and waits for bit 19, the divider's "valid", to clear, as the model now does. No model for its ATBM603x SDIO Wi-Fi (the app retries it), the audio DAC or the RTC |
| `full.bin` (AK3919EV330, U-Boot AK_V3.1.00, 4.4.192) | `ak3919ev330` | login, `anyka_ipc`; its nginx answers HTTP over the emulated Ethernet (static 192.168.1.88) |
| `AK3918_V300s_dumps.zip`, `AK3918_V330L_dumps.zip` (AK3918EV330, U-Boot V3.1.28, 4.4.192; partitions only) | `ak3918ev330` | assembled at the `mtdparts` offsets: login; ENV, CONFIG and APP are not in the zips, so `/usr` (and `ak_eth.ko`) is missing. Flash BY25Q64A (`0x684017`) |
| gzhongshi HVT3610S3 QQ-WiFi v2.0 (AK3918EV300L, U-Boot 2019.10.0-V4.0.25, 4.4.282, XM25QH64C; ipctool backup, rebuilt into a raw image) | `ak3918ev300l` | `GZhongshi login:`, vendor app (no sensor answers on I2C); vendor U-Boot `sf probe/read/erase/write` and `reset` (watchdog) work; matches the board's own boot log up to the PHY (absent on the board) |
| `XM25QH64A@SOIC8.BIN` | — | the dump has bit errors throughout (`consold=ttyS@K0`); not bootable |
| AnyCloud39AV100 SDK V1.12 firmware, assembled into an 8 MiB image (`qemu-boot/anyka-sdk-image.py`): the SDK's own U-Boot 2019.10.0-V4.0.27 (`ak3918av100_nor_defconfig`) under an `ANYKA322` header with the burn tool's 64 MiB DDR script, `image/uImage` (vendor 4.4.282), `EVB_CBDM_AK3918AV100N_V1.0.0.dtb`, `root.sqsh4`, `usr.jffs2`, `usr.sqsh4` at the burn-config offsets | `ak3918av100` | `anyka login:` (`ANYKA CPU AK3918AV100-A (ID 0x535335)`, GD25Q64 NOR, 8 MTD partitions, squashfs root, jffs2 config, APP mounted). The vendor modules load without crashing: `ak_rtc`, `ak_i2c`, `ak_efuse`, `ak_ion`, `ak_pcm`, `ak_isp`, `ak_venc_adapter`, `ak_npu`, `ak_eth`. (`ak_venc_bridge` and `ak_hcd` need symbols from `ak_venc_adapter` / usbcore loaded first, as the vendor scripts do.) |
| the same, `--ram 128` with `EVB_CBDM_AK3918AV100X_V1.0.0.dtb` | `ak3918av100x` | `anyka login:`, `mem=128M memsize=128M` |
| owner report r-5uwdebux (openipc.org): AK3918AV130, U-Boot 2019.10.0-V4.0.27, raw 8 MiB read | `ak3918av130` | `AnyCloud-uc1_1 login:` (4.4.302-cip94, NOR through the SFC by L2 DMA, squashfs root, the vendor modules load); then the app's device-ID decryption fails (left so on purpose, see above), its daemon asks for a reboot, and the watchdog reset boots it again: a loop of about ten seconds |

## Majestic on the nightlies

`qemu-boot/test-anyka-majestic.sh --soc ev200|ev300` boots the latest
nightly as released and checks Majestic end to end; CI's `majestic-anyka`
job runs it for both:

| `--soc` | Image (latest release) | Machine | Checked |
|---|---|---|---|
| `ev200` | OpenIPC/firmware `openipc.ak3918ev200-nor-lite.tgz`, `u-boot-ak3918ev200-nor.bin` | `ak3918ev200` (SC1345, `ak-isp`, `ak-h8290`) | majestic's first frame; 50 pictures of each RTSP stream decode; `/image.jpg` is the main stream's size; no `Cannot start SDK` or stuck encoder |
| `ev300` | OpenIPC/builder `ak3918ev300_lite_heimvision-pe3020w-nor.tgz`, firmware's `u-boot-ak3918ev300-nor.bin` | `ak3918ev300` (JX-F37, `ak-isp ev300`, `ingenic-avpu anyka`) | the same |

The image is written to an 8 MiB NOR at OpenIPC's offsets and boots from its
own U-Boot. The root changes in three ways only: root's password is set (an
unclaimed camera streams nothing), `majestic.yaml` turns on the 640x360
second stream, and the syslog is mirrored to the console. `--firmware`,
`--kernel` and `--majestic` take local files in place of the release's, to
try a fix before it ships. The Anyka twins need `-nic user` without
`restrict=on`, which keeps the forwarded ports' replies from returning.

## Ethernet checks

- EV300 `3918.bin`, U-Boot: `setenv ipaddr 10.0.2.15; setenv serverip
  10.0.2.2; ping 10.0.2.2` -> `host 10.0.2.2 is alive`.
- EV300 `3918.bin`, Linux (built-in driver): `ifconfig eth0 10.0.2.15 up;
  ping -c 3 10.0.2.2` -> 0% loss, PHY ID 0x0007/0xc0f1, 100 Mbit/s.
- EV330 `full.bin`, Linux (`ak_eth.ko`): with `-netdev user,net=192.168.1.0/24,
  hostfwd=tcp::41080-192.168.1.88:80`, the camera's nginx answers from the
  host; a `filter-dump` capture shows ARP and the whole TCP exchange.

## Not modelled

The MCI's L2 DMA data path, the ISP's image processing and statistics
(EV200's and AK3918EV300's frames are `ak-isp`; the other parts' ISP is
storage), the EV330 video encoder (EV200's is `ak-h8290`, EV300's
`ingenic-avpu`), audio
codec, SAR ADC. Only the EV200 and the AK3918EV300 have a sensor on I2C
(`sc1345` / `jxf37` by default). On the AV100 the second MIPI/CSI
input, the NPU (`0x21300000`) and the video pipe (`0x212`--`0x215`) are the
`ak-apb` catch-all (reads 0): the vendor modules probe and map them but no
frames flow. The second L2 buffer (`0x201A8000` / `0x58000000`) and the
third MCI (`mmc2` at `0x201B0000`, SDIO Wi-Fi) are likewise unmodelled and
disabled on the single-sensor boards.

## Building an AV100 flash image

The AnyCloud39AV100 SDK ships no U-Boot binary (the `*.bin` are gitignored).
Build it from the SDK's `os/uboot` (`ak3918av100_nor_defconfig`), then wrap
it with the header and partitions the burn tool writes:

```
python3 qemu-boot/anyka-sdk-image.py <sdk> <u-boot.bin> flash.bin --ram 64
bash qemu-boot/run-ak3918av100.sh flash.bin ak3918av100
```

`--ram 128` with `--dtb EVB_CBDM_AK3918AV100X_V1.0.0.dtb` makes the 128 MiB
image for `ak3918av100x`. The script reads the DDR register script from the
SDK's `tools/burntool/SDK_CPU_DDR2.ini` and the DTB, kernel and rootfs from
the SDK's `image/`.
