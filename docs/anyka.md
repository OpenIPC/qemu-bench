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
| AnyCloud39AV130 SDK V1.03 (one tree for several lines) | AK3918EV300L, AK3918AV100 (N/P/X/XP) | `ANYKA_130L_SDK`: U-Boot 2019.10 (`arch-ak3918ev300l`, `arch-ak3918av100`), kernel 4.4.302 (`anycloud_ak3918ev300l*.dtsi`), dump | 4.4.282 vendor (DT) | `AK_H322L` / `AKH322LS` (EV300L), `ANYKA322` (AV100, inferred from `ak_ddr_param`) |
| AnyCloud39AV100 SDK V1.12 | AK3918AV100 (N/P/X/XP), KM01 (N/D) | `ANYKA_AV100_SDK`: U-Boot 2019.10.0-V4.0.27 (`arch-ak3918av100`, `ak3918av100_nor_defconfig`), kernel 4.4.282 (`anycloud_ak3918av100*.dtsi`), prebuilt `image/` | 4.4.282 vendor (DT) | `ANYKA322` (confirmed: U-Boot `vectors.S` builds `ANYKAH3D`, the burn tool patches it to `ANYKA322`; `ak_secureboot` header template) |
| AK37D, AK37E | — | `#ifdef`s in the 4.4 tree's `map.h`, `irqs.h`, `ak_timer.c` only | — | — |

The `Aimer39_PDK_V1.1.00` archive (AK3918 datasheet and programming guide)
is header-encrypted and was not usable.

## Machines

| Machine | Chip ID (SYSCTRL +0) | DDR | Timers | Notes |
|---|---|---|---|---|
| `ak3918ev200` | `0x20150200` | 64 MiB | 5 | the 4.4 port also accepts `0x20120100` |
| `ak3916ev300` | `0x20160100` | 64 MiB | 5 | the AK3916EV301 dump boots here |
| `ak3918ev300` | `0x20160100` | 64 MiB | 5 | |
| `ak3919ev300` | `0x20160100` | 128 MiB¹ | 5 | |
| `ak3918ev330` | `0x20160101` | 64 MiB | 10 | eFuse reader, I2C v2, flash CS on GPIO 25 |
| `ak3919ev330` | `0x20160101` | 128 MiB | 10 | U-Boot of the dump prints `ramsize: 08000000` |
| `ak3918ev300l` | `0x53543400` | 64 MiB | 10 | "ASPEN": SPI0 at `0x21100000`, clock block v2; `socinfo` prints `0x535434` |
| `ak3918av100` | `0x53533500` | 64 MiB | 10 | ASPEN + AV100 blocks; AK3918AV100N/P, KM01N; boots the AnyCloud39AV100 SDK V1.12 firmware to login |
| `ak3918av100x` | `0x53533500` | 128 MiB | 10 | same die, 128 MiB; AK3918AV100X/XP, KM01D |

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
| `0x08000000` | system control: chip ID, PLLs, clock gates, resets, the interrupt controller, timers, watchdog, RTC link, EV330 eFuse | `ak-sysctrl` |
| `0x20000000` | ISP / VI | storage |
| `0x20020000` | video encoder | storage |
| `0x20100000` / `0x20108000` | MCI0 / MCI1; `-drive if=sd` is the TF slot: MCI0, but MCI1 on the EV300L and AV100 (their DTs put the card on mmc1, pads 31-36, power GPIO 50) | `ak-mci` |
| `0x20110000` / `0x20118000` | DAC / ADC | storage |
| `0x20120000` / `0x20128000` | SPI0 (boot NOR) / SPI1 | `ak-spi`, `ak-spinor` |
| `0x20130000` / `0x20138000` | UART0 / UART1 (UART2 at `0x201A0000`, interrupt 24, on the EV300L and AV100) | `ak-uart` |
| `0x20140000` | L2 buffer controller | `ak-l2` |
| `0x20150000`, `0x20190000`, `0x201A0000` | I2C 0 (1, 2 on EV330; EV300L: I2C 2 at `0x201C0000`, UART 2 at `0x201A0000`) | `ak-i2c` |
| `0x20170000` | GPIO | `ak-gpio` |
| `0x20200000` | USB 2.0 OTG (MUSB-style) | `ak-usb` (nothing plugged in) |
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
| `+0x2C` | register script: `{address, value}` pairs; address `0x66668888` = delay, `0x66668881` = three-word poll, `0x66668880` / `0x66668882` = EV300L / AV100 ROM operations of one value (skipped), `0x88888888` = end |
| `+0x18C` | `SPIP` + the SPI NOR descriptor (JEDEC ID, size, page, erase size, partition-table block) |

The AV100's `ANYKA322` payload does not follow the header at `+0x200` but at
`+0x600`: `vectors.S` under `CONFIG_3918AV100_CODE` reserves `0x400` more
bytes (secure boot keeps the public key at `+0x200` and the signature at
`+0x300`; `tools/secureboot/ak_secureboot`), and the vendor U-Boot is linked
at the load address `- 0x600` (`CONFIG_SYS_TEXT_BASE 0x80DFFA00`). The model
copies an `ANYKA322` payload from flash `0x600`; the others still from
`0x200`. (Signed images also write `0x84000000` at `+0x1C`; unsigned ones
keep `6`.) `0x600` is inferred from the SDK -- no AV100 flash dump has been
seen -- and matches what the OpenIPC U-Boot `ak3918av100` port emits.

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
| `full.bin` (AK3919EV330, U-Boot AK_V3.1.00, 4.4.192) | `ak3919ev330` | login, `anyka_ipc`; its nginx answers HTTP over the emulated Ethernet (static 192.168.1.88) |
| `AK3918_V300s_dumps.zip`, `AK3918_V330L_dumps.zip` (AK3918EV330, U-Boot V3.1.28, 4.4.192; partitions only) | `ak3918ev330` | assembled at the `mtdparts` offsets: login; ENV, CONFIG and APP are not in the zips, so `/usr` (and `ak_eth.ko`) is missing. Flash BY25Q64A (`0x684017`) |
| gzhongshi HVT3610S3 QQ-WiFi v2.0 (AK3918EV300L, U-Boot 2019.10.0-V4.0.25, 4.4.282, XM25QH64C; ipctool backup, rebuilt into a raw image) | `ak3918ev300l` | `GZhongshi login:`, vendor app (no sensor answers on I2C); vendor U-Boot `sf probe/read/erase/write` and `reset` (watchdog) work; matches the board's own boot log up to the PHY (absent on the board) |
| `XM25QH64A@SOIC8.BIN` | — | the dump has bit errors throughout (`consold=ttyS@K0`); not bootable |
| AnyCloud39AV100 SDK V1.12 firmware, assembled into an 8 MiB image (`qemu-boot/anyka-sdk-image.py`): the SDK's own U-Boot 2019.10.0-V4.0.27 (`ak3918av100_nor_defconfig`) under an `ANYKA322` header with the burn tool's 64 MiB DDR script, `image/uImage` (vendor 4.4.282), `EVB_CBDM_AK3918AV100N_V1.0.0.dtb`, `root.sqsh4`, `usr.jffs2`, `usr.sqsh4` at the burn-config offsets | `ak3918av100` | `anyka login:` (`ANYKA CPU AK3918AV100-A (ID 0x535335)`, GD25Q64 NOR, 8 MTD partitions, squashfs root, jffs2 config, APP mounted). The vendor modules load without crashing: `ak_rtc`, `ak_i2c`, `ak_efuse`, `ak_ion`, `ak_pcm`, `ak_isp`, `ak_venc_adapter`, `ak_npu`, `ak_eth`. (`ak_venc_bridge` and `ak_hcd` need symbols from `ak_venc_adapter` / usbcore loaded first, as the vendor scripts do.) |
| the same, `--ram 128` with `EVB_CBDM_AK3918AV100X_V1.0.0.dtb` | `ak3918av100x` | `anyka login:`, `mem=128M memsize=128M` |

## Ethernet checks

- EV300 `3918.bin`, U-Boot: `setenv ipaddr 10.0.2.15; setenv serverip
  10.0.2.2; ping 10.0.2.2` -> `host 10.0.2.2 is alive`.
- EV300 `3918.bin`, Linux (built-in driver): `ifconfig eth0 10.0.2.15 up;
  ping -c 3 10.0.2.2` -> 0% loss, PHY ID 0x0007/0xc0f1, 100 Mbit/s.
- EV330 `full.bin`, Linux (`ak_eth.ko`): with `-netdev user,net=192.168.1.0/24,
  hostfwd=tcp::41080-192.168.1.88:80`, the camera's nginx answers from the
  host; a `filter-dump` capture shows ARP and the whole TCP exchange.

## Not modelled

The MCI's L2 DMA data path, ISP, video encoder, audio codec, SAR ADC. No
sensor models are attached to I2C yet. On the AV100 the second MIPI/CSI
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
