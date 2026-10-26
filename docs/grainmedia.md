# Grain Media (`gm8135s`, `gm8136`, `gm8136s`)

Grain Media's GM813x IP-camera SoCs: a Faraday FA626TE (ARMv5TE) among
Faraday peripherals, DDR at 0 and everything else from `0x90000000`. One
machine file (`hw/arm/grainmedia.c`) covers the packages, which differ in
the PMU's chip ID and where the DRAM sits.

## What the SDK and dumps cover

| Source | What it gave |
|---|---|
| `arm-linux-3.3` SDK: `linux-3.3.tar.gz` + `gm.patch` (`arch/arm/mach-GM/platform-GM8136`, the `ft*` drivers), `module/` (vendor media modules, `adda308` source) | memory map, IRQs, every driver the models are checked against |
| `u-boot-2013.01` (`board/gm/gm8136`, `drivers/mtd/spi/spi020*.c`) | PLL/PCLK formulas, bootargs selection, SPI020 + DMA usage |
| `gm8136_nsboot` (source and `nsboot.bin`) | the boot loader the mask ROM runs: DDR tables, frequency change, NOR system header |
| three Longse camera dumps (MX25L12805D, 16 MiB) | the real boot chain: nsboot at 0x1000, U-Boot at 0x10000, kernel `#44` 2015, vendor application |
| OpenIPC `gm8136` / `gm8135` releases | the kernel booted with `-kernel`, root on the NOR |

The SDK also carries GM8126, GM8139 and GM8287 platforms; they are not
modelled here (GM8139's map is close to GM8136's, GM8126 has an FTINTC010).

## Machines

| Machine | CPU | DDR | Chip ID (PMU 0x00) | Boot tested |
|---|---|---|---|---|
| `gm8135s` | FA626TE | 64 MB in package | `0x08136110` | OpenIPC `gm8135` |
| `gm8136` | FA626TE | 128 MB (external) | `0x08136100` | Longse dumps, OpenIPC `gm8136` |
| `gm8136s` | FA626TE | 128 MB in package | `0x08136150` | —¹ |

PMU 0x00 is `0x0813_61xx`: bits 11:8 the revision (1 = B), bits 6:2 the
package, which the kernel's `pmu_get_chip()` and U-Boot read as 8136 (0),
8136S (0x14, 0x1C) or 8135 (0x4, 0x8, 0x10, 0x18).

The CPU is QEMU's `arm926` with the FA626TE's MIDR (`0x66056261`), which
`proc-fa626te.S` matches (`0x66056260` / `0xff0ffff0`); the FA626TE's
cache operations are all ARMv5 CP15 ones QEMU accepts.

¹ Same die and models as `gm8136`; no image was at hand.

## Memory map

| Address | Block | Model | IRQ |
|---|---|---|---|
| `0x00000000` | DDR (after nsboot's AXI remap) | RAM | |
| `0x90700000` / `0x90800000` / `0x90900000` | UART0-2, FTUART010 (16550, 4-byte stride, 25 MHz) | `serial-mm` | 21 / 22 / 25 |
| `0x90A00000` | SAR ADC (CVBS detect, keys) | `gm-saradc` | 35 |
| `0x90B00000` | ADDA308 audio codec wrapper | `gm-ssp` (SSP0's second region) | |
| `0x90C00000` | FTPMU010 | `gm-pmu` | FCS: 8 |
| `0x90D00000` | FTTMR010, three timers | `gm-timer` | 14 |
| `0x90E00000` | FTWDT010 | `gm-wdt` | 16 |
| `0x91000000` / `0x91900000` | FTGPIO010 GPIO0 / GPIO1 | `gm-gpio` | 13 / 63 |
| `0x91200000` | FTI2C010 | `gm-i2c` | 18 |
| `0x91300000` / `0x91400000` | FTSSP010 I2S (audio) | `gm-ssp` | 6 / 11 |
| `0x91600000` | FTIRDET IR receiver | `gm-irdet` | 48 |
| `0x91B00000` | FTPWM010 PWM | `gm-pwm` | 49 |
| `0x91C00000` | FTRTC011 RTC | `gm-rtc` | 50 |
| `0x91D00000` | pMonitor bandwidth monitor | `gm-pmonitor` | 44 |
| `0x92000000` | SRAM (MCP100's; nsboot runs here) | RAM, 64 KiB; 0x8000-0x8FFF the MCP100 mailbox | |
| `0x92300000` | FTSPI020 flash controller | `gm-spi020` | 54 |
| `0x92500000` | FTGMAC100 | QEMU `ftgmac100` | 3 |
| `0x92600000` | FTDMAC020 AHB DMA | `gm-dmac` | 1 |
| `0x92800000` / `0x93800000` | FTSDC021 SD0 / SD1 | `generic-sdhci` | 15 / 17 |
| `0x93000000` | FOTG210 USB OTG | `gm-otg` | 9 |
| `0x96000000` | FTINTC030 | `gm-intc` | |
| `0x96100000` | VCAP300 capture | `gm-vcap` | 32 |
| `0x96200000` | scaler300 | `gm-scaler` | 12 |
| `0x92C00000` / `0x92D00000` | MCP280 H.264 encoder / its VCACHE | `gm-h264e` | 29 |
| `0x9B700000` | FT3DNR200 | `gm-3dnr` | 42 |
| `0x9BF00000` | LCD200 | `gm-lcd` | 24 |
| `0x92200000` | GM2D (think2d) 2D engine | `gm-2d` | 4 |
| `0x92020000` / `0x92400000` | MCP100 MJPEG codec / its DMA wrapper | `gm-mcp100` | 31 |
| `0x92700000` | FTAES020 AES/DES | `fh-aes` (`advance=on`) | 19 |
| `0x93930000` | ISP328 | `gm-isp` | 43 |
| `0x9A100000` | FTDDRC010 | `gm-ddrc` | |

Everything in `0x90000000`-`0x9FFFFFFF` that is not listed as a model
reads as zero (`grainmedia.periph`, `-d unimp` shows the accesses).

## Boot

### The mask ROM and nsboot

The NOR starts with a 256-byte system header -- `"GM8136"`, `bootm_addr`,
ten `{addr, size, name}` partition records (`UBOOT`, `LINUX`, `FS`,
`USER` on the Longse cameras) and `0x55AA` at 0xFE -- and the nsboot
loader at 0x1000. The mask ROM copies nsboot (24 KiB) into SRAM at
`0x92000000` and runs it; the stand-in does the same and puts
`ldr pc, [pc, #-4]; .word 0x92000000` at address 0.

nsboot's `head.S` remaps DDR to 0 (AXIC 0x130); its `entry.c` then:

1. sets the UART pin mux and divider, and writes its `DDR_arg` table into
   the DDR controller, writes 1 to DDRC 0x04 and polls bit 8;
2. points the IRQ vector (in DDR at 0x18) at its own handler;
3. changes the CPU clock: writes PLL1, routes interrupt 8 to CPU 0 (INTC
   0x45C / 0x04), sets PMU 0x0C bit 2 and waits in WFI; the PMU raises
   interrupt 8 and the handler clears PMU 0x20 bit 17;
4. reads the header through the FTSPI020 (PIO in the dumps' build) and
   loads image 0 -- U-Boot, 0x10000 + 0x50000 -- to address 0 and jumps.

The DDR table's 0x10 holds the DRAM size code (2 = 64 MiB, 3 = 128 MiB,
4 = 256 MiB, 5 = 512 MiB); `gm-ddrc` resets to the machine's RAM size and
nsboot overwrites it with its own.

### U-Boot

U-Boot 2013.01 (`GM #`) probes the NOR through the FTSPI020 and the
FTDMAC020, prints the clocks from the PMU, brings up the FTGMAC100 and
reads the kernel with `sf read 0x02000000`. `commandline_mode()` reads
DDRC 0x10 and picks `cmd1`..`cmd4` from the environment as `bootargs`
(`mem=128M gmmem=90M ... root=/dev/mtdblock4` on the Longse cameras); it
saves that to the environment at 0x50000 on every boot, so a dump boot
erases and programs the NOR (written back to the `flash-file`). It
resets through the watchdog (load 1, restart, enable with reset).

### Kernel

The vendor kernel is 3.3.0 without a device tree: machine 758 (`GM`),
ATAGs at 0x100. `-kernel` takes a uImage or zImage:

- The SDK's uImages are built to run in place (U-Boot's "XIP Kernel
  Image"): `ih_load` is the header's address and `ih_ep` = `ih_load` +
  0x40 is the zImage's. QEMU puts the payload at `ih_load`, so the machine
  hands it a copy whose load address is the entry.
- `fmem_fixup_memory()` needs `mem=` on the command line and hangs before
  the console without it; the machine appends `mem=<RAM>M` when it is
  missing. `gmmem=` sizes the media pool (frammap) out of it.
- The vendor kernel takes its MTD partitions from the NOR's system header
  (`drivers/mtd/devices/spi_flash.c`), not `mtdparts=`; OpenIPC's honours
  `mtdparts=nor-flash:...`.

### OpenIPC

OpenIPC publishes `gm8136` and `gm8135` kernels and root filesystems but
no GM bootloader. `qemu-boot/test-grainmedia-boot.sh` lays them out on an
8 MiB NOR (`256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)`)
and boots with `-kernel`:

```bash
bash qemu-boot/fetch-openipc-lite.sh gm8136 qemu-boot
bash qemu-boot/test-grainmedia-boot.sh --soc gm8136
```

It reaches the login prompt, gets a DHCP lease and writes its JFFS2
overlay back to the image. The OpenIPC `gm8135` release runs on
`gm8135s`.

### A camera dump

```bash
./qemu-src/build/qemu-system-arm -M gm8136,flash-file=dump.bin,sensor=ov2715 \
    -nographic -serial mon:stdio -nic user
```

Work on a copy: U-Boot rewrites its environment and the application its
JFFS2 partition. On the Longse dumps `q` + Enter at "Press q -> ENTER to
exit boot procedure?" gives a shell before the application starts.

The application streams the encoded video over RTSP. It gives itself
192.168.1.209 and wants `admin` / `admin`; the main stream is `/ch0`:

```bash
./qemu-src/build/qemu-system-arm -M gm8136,flash-file=dump.bin,sensor=ov2715 \
    -nographic -serial mon:stdio \
    -nic user,net=192.168.1.0/24,host=192.168.1.2,hostfwd=tcp::8554-192.168.1.209:554
ffplay -rtsp_transport tcp rtsp://admin:admin@127.0.0.1:8554/ch0
```

`qemu-boot/test-grainmedia-rtsp.sh --flash dump.bin` does that unattended:
flash boot, the RTSP server up, a 1920x1080 H.264 stream that ffmpeg
records and decodes without errors, and no unfinished hardware job in the
video graph.

## Devices

### `gm-pmu` (FTPMU010)

0x00 chip ID, 0x04 strap (bit 7: SPI NAND, bit 23: 4-byte NOR), 0x0C
PMODE (bit 2: start a frequency change), 0x20 status (bit 17: change done,
write 1 to clear; the "fcs-irq" output, interrupt 8), 0x30 PLL1 and 0x34
PLL2 / PLL3 (30 MHz × N[10:4] / M[15:11]), PLL1's high half selecting
the clocks: AHB from PLL1 / 3, PLL2 / 3 or PLL2 / 4 (bits 19:18). PCLK =
AHB / 2 is the "pclk" Clock output that `gm-timer` and `gm-wdt` count at;
it follows the PLL registers. The reset values are what nsboot leaves on
one of the Longse cameras (the other two set PLL1 to 885 MHz): PLL1
860 MHz -- the `adda308` audio driver panics unless PLL1 is one of 860,
590, 762, 885, 712.5 or 442.5 MHz, so a `-kernel` boot needs one of them
-- and PLL2 600 MHz, so PCLK is 100 MHz.
Everything else is storage; sub-word stores merge.

### `gm-intc` (FTINTC030)

Groups of 32 interrupts at `g * 0x20` (g < 2) and `(g + 1) * 0x20` after:
SOURCE, ENABLE, CLEAR (edge latches), MODE (1 = edge), LEVEL (1 = active
low / falling), STATUS. The kernel writes ENABLE all ones once and masks
through CPU 0's IRQ target words at 0x45C + 4g (FIQ at 0x420), reading
them back after each write; its IRQ entry reads STATUS (0x14, 0x34) and
ANDs the target. The line to the CPU is STATUS & target.

### `gm-timer` (FTTMR010)

Three channels at 0x00/0x10/0x20 (COUNTER, LOAD, MATCH1, MATCH2), CR 0x30
(enable, external clock, overflow interrupt per channel; count-up bits
9-11), INTR_STATE 0x34 (write 1 to clear), INTR_MASK 0x38, one interrupt
line (14). A channel counts down at PCLK and reloads LOAD past zero; the
kernel's one-shot writes COUNTER with LOAD = 0, so a channel that reloads
0 stops. The clocksource is a free-running channel read as `~COUNTER`.
Matches are modelled only at 0, where both drivers leave them.

### `gm-spi020` (FTSPI020) and `gm-dmac` (FTDMAC020)

A flash command is CMD0 (address), CMD1 (address bytes, dummy cycles,
opcode bytes), CMD2 (data bytes) and CMD3 (opcode, chip select, status
read, write, interrupt), the last write starting it. Reads clock the flash
into a 128-byte RX FIFO and complete when the last byte is in: U-Boot
waits for completion before it drains the 3-byte JEDEC ID. Status-read
commands (hardware-polled or software) land the status in 0x28. Dummies
are clocked one transfer per cycle, which is how QEMU's m25p80 counts
them.

The kernel and U-Boot move data with the FTDMAC020 in hardware-handshake
mode on request line 8; the controller holds "dma-req" while the FIFO has
data (or a write wants data). The DMAC runs channels from a bottom half --
the request rises inside the SPI controller's own CMD3 write -- and
follows linked lists (`{src, dst, next, ctrl, cycles}`, the ctrl word in
the LLD layout), with TC and error/abort status per channel.

### `gm-i2c` (FTI2C010)

Byte at a time: DR then CR with TB_EN, plus START (DR is the address
byte), STOP, NAK. SR reports byte transmitted / received / bus error (no
ACK) a few microseconds later and clears on read; the kernel's interrupt
mode sends the next byte from its handler. `sensor=ov2715` (the Longse
cameras' sensor, at 0x36, ID 0x2715 at 0x300A) or `ov9715` puts a 16-bit
register sensor stub (`galaxycore-sensor`) on the bus, which the ISP's
`fisp_ov2715.ko` probes; the default is none.

### `gm-wdt` (FTWDT010)

Counter, load, restart (`0x5AB9`), control (enable, reset, interrupt,
external 30 MHz clock), status / clear. It counts PCLK unless the control
selects the oscillator, and resets the machine on expiry with the reset
bit -- both reboot paths load 0 or 1 and enable with reset.

### `gm-otg` (FOTG210)

The host block is FUSBH200's, which is QEMU's `fusbh200-ehci-usb` (PORTSC
0x30, bus control 0x40); above it the OTG control/status (0x80: ID pin
and role read A device / host, bits 23:22 the port speed the vendor
`ehci_port_speed()` reads -- high), and the global interrupt status / mask
(0xC0 / 0xC4, bit 2 the host). The camera kernel has no `usb-storage`, so
a `-device usb-storage` enumerates but gets no `sd` node.

### Others

- FTGMAC100: QEMU's model with `aspeed` off; the kernel's `ftgmac100` and
  U-Boot find the PHY by MDIO through PHYCR.
- FTSDC021: an SD 3.0 SDHCI with ADMA2 (`capareg` 0x017834b4); the
  vendor `sdhci-ftsdc021` supplies the base clock.
- `gm-gpio` (FTGPIO010): undriven inputs read high (`input-default`).
- `gm-ddrc`: storage, init done (0x04 bit 8), the size code in 0x10.

## The video pipeline

The vendor video graph (`em.ko`, `vpd_slave.ko`, `gs.ko`, gmlib) runs
the IP camera's graph -- capture -> 3DNR -> H.264 for the 1080p main
stream and the D1 sub-stream, the scaler where capture cannot produce a
size, the LCD for its frame clock -- and declares a hardware job lost (the
"damnit" log dump, `GS is entering panic state`) when one never comes
back. Every engine it waits on is modelled, from the SDK's drivers:

- **`gm-vcap` (VCAP300)** runs the driver's link lists in its SRAM at
  0x40000 -- per path a "next" table pointer in the channel block
  (+0x1C8), tables of STATUS / UPDATE (byte enables) / NEXT_CMD / NEXT_LLI
  / NULL commands -- and at 30 frames a second paints the Allwinner
  capture model's picture (the OpenIPC logo on navy, a dot sliding
  beneath it; `aw-vinc-logo.h`) into each completed table's
  DMA buffer as UYVY, the target crop at the scaler output's scale, then
  fills the STATUS slot (ready, frame mode, frame count, DMA done, the
  driver's channel / path IDs kept) and sets LL done (the cascade's
  0x55B4). The driver links jobs into the lists only from the 60 Hz
  sync-timer interrupt (0x55AC bit 0). 0x55E8 / 0x55EC read the GM8136's
  version (0x20140311) and capabilities (0x0F184874: four scalers that
  can scale -- gmlib refuses the D1 sub-stream otherwise).
- **`gm-3dnr` (FT3DNR200)** executes its command lists in its memory at
  0x400-0xBFF (absolute addresses: status, register write, 16-bit write,
  jump, end of job, stop). The first job is fired; later ones are queued
  by turning the stop command the engine waits on into the next job's
  status slot. Each job copies the source frame to the destination and
  sets its status slot's bit 0 and 0x208 bits 3 / 7.
- **`gm-scaler` (scaler300)** runs the dummy job in its registers, then
  the list at 0x40000 + 0x1A8 (status, byte-enabled register writes, job
  end, continue, chain end), scaling UYVY nearest-neighbour or filling the
  fixed colour per output path with the target crop, and reports each
  job's finished outputs in its status slot and list done in 0x55B0.
- **`gm-h264e` (MCP280)**: the driver packs the SPS, PPS and slice
  headers itself through the header FIFO at 0x400 ({control, value}
  pairs: length, emulation prevention, Exp-Golomb, trailing bits, last);
  the model keeps the bit position and the emulation-prevention state
  across batches and continues the same bitstream with the slice data
  and its trailing bits. I slices are I_16x16 DC macroblocks from the
  packed 4:2:2 source (DIDN_CUR_TOP 0x100) -- in CABAC the 16 luma DCs
  and the chroma DCs (shared with the Xiongmai H1 model,
  `hw/misc/hantro-h264.c`), in CAVLC each macroblock's mean. P slices
  code the macroblocks that changed against the reference as I_PCM and
  skip the rest; the reference is kept, as the driver expects, in its
  REC buffer (0x120 / 0x124) as NV12 and read back through ME_REF
  (0x108 / 0x10C), so the moving dot is followed exactly at about
  2 Mbit/s.
  STS1 reports the byte count the driver uses as the frame size, STS2 the
  slice-done interrupt (write 1 to clear). STS0's idle bits end the
  probe's DMA-stop poll. `-global gm-h264e.dump=DIR` saves the streams,
  one file per picture size.
- **`gm-lcd` (LCD200)**: the display driver moves its jobs on only from
  the vertical interrupt, which is also the graph's reference clock, so
  while the LCD is enabled a 29.97 Hz timer raises INT_STATUS bit 2.
- **`gm-isp` (ISP328)** runs frame start / end (status0 bits 16 / 17)
  at 30 frames a second whether the ISP is on or not -- the driver waits
  for both around every stop and resize -- executes the input DMA tables
  the driver queues its register updates in at frame start, and at frame
  end fills the statistics for a neutral grey scene (`luma`, default 100,
  the AE's target: per-window R / G / B sums, histograms, AWB ratios of
  1.0, a flat AF figure), runs the output DMA table that copies them to
  the AE / AWB / AF threads' buffers, and raises the ready bits of the
  enabled units with frame end. AE settles at once and AWB keeps unity
  gains.
- **`gm-2d` (GM2D)** executes its command lists ({register, value}
  pairs; DRAW_CMD and BLIT_CMD run the operation): lines and filled
  rectangles, blits with colour keys, rotation, nearest-neighbour scaling
  and DirectFB blend factors, between RGB/ARGB 8888, RGB565, RGBA5551,
  RGBA4444, A8, L8, RGB332, BW1 and UYVY (BT.601 limited range). That
  covers the GM8136's uses: OSD bitmaps (RGB565 keyed on 0x0020) into the
  UYVY frames, privacy masks (RGB565 fills of packed U Y / V Y pairs) and
  UYVY to RGB565 for the LCD. Each list completes at once, STATUS 0 and
  interrupt 4.
- **`gm-mcp100` (MCP100)** reads its embedded CPU running (INCTL 0x2007C
  bit 23) and completes its reset, and does the CPU firmware's part of
  MJPEG: the host writes the headers through the bit packer (BADR 0x4C /
  BALR 0x50 to ABADR 0x78), fills the Share_Node_JE mailbox at SRAM
  0x8000 and writes command 8; the model encodes the ROI -- packed UYVY
  through the DMA wrapper at 0x92400000, or I420 planes -- as baseline
  4:2:0 with the DQTs' tables, the standard Huffman tables and RSTn every
  restart_interval MCUs, ends it with EOI, writes bitslen / jpgenc_done /
  command 0x80 and raises INT_FLAG bit 31 (interrupt 31). A JPEG that
  would pass max_bs_thd stops early with stop_x / stop_y, as the firmware
  does. MPEG-4 is not built into the cameras' kernels.

## Audio

`gm-ssp` is an FTSSP010 in I2S mode; SSP0's instance also maps the
ADDA308 codec's registers at 0x90B00000, whose capture / playback rate
codes (reg 0 bits 7:4 / 11:8) clock it. Capture feeds the RX FIFO with a
tone (`tone-hz`, 1 kHz, amplitude 8000; 0 for silence) at the codec's
rate, playback drains the TX FIFO to the machine's `-audiodev`. The
FIFOs report at most 16 entries but keep up to 2048 samples behind them,
so emulation jitter never overruns or underruns -- either makes the
vendor driver tear the stream down. The DMAC moves the samples on the
handshake lines (SSP0 RX 10 / TX 9, SSP1 RX 12 / TX 11), in the driver's
cyclic link lists; channel abort (CCR bit 15) reports through EAISR. The
PMU's ADDA clock divider (0x74 bits 29:24) starts at the codec driver's
value, which `irda.ko` divides by.

The Longse application captures only with audio enabled in its
configuration: the byte at offset 0x98C of `/mnt/mtd/sysenv.cfg` (and
`sysenvcopy.cfg`; struct offset 0x988 after the 4-byte magic, no
checksum) is 0 in the dumps. Set it to 1 from the boot shell (press `q`
at "Press q -> ENTER") and the RTSP stream's PCMU track carries the tone:

    for f in sysenv sysenvcopy; do printf '\001' | \
        dd of=/mnt/mtd/$f.cfg bs=1 seek=2444 conv=notrunc; done; sync

## The small peripherals

- **`gm-saradc`**: three 8-bit channels, `ch0`-`ch2` (default 0x80),
  settable at run time (`qom-set /machine/saradc ch2 0x20`). XGAIN2 is
  the CVBS cable detector: above 0x78 the driver powers the video DAC
  down. No threshold interrupts.
- **`gm-rtc` (FTRTC011)**: SEC / MIN / HOUR / DAY from QEMU's RTC clock
  (`-rtc base=`), loading through WSEC..WDAY and CR bit 6, per-second /
  minute / hour / day and alarm interrupts in INTR_STATE (write 1 to
  clear), never busy. The Longse kernel lacks the RTC class, so its
  `rtc-ftrtc011.ko` does not load.
- **`gm-pwm` (FTPWM010)**: registers; the update bit loads CNTB into
  CNTO and clears. The timers do not count and interrupt 49 stays low
  (the driver's handler indexes by INT_PRIO - 1).
- **`gm-irdet` (FTIRDET)**: codes are pressed from the monitor,
  `qom-set /machine/irdet key 0x45`, delivered as NEC frames from
  `address` with the repeat flag within `repeat-ms`.
- **`fh-aes` (FTAES020)**: the Fullhan model, the same IP: AES-128/192/256
  in ECB / CBC / CTR / CFB / OFB (QEMU's AES) and DES / 3DES (with a
  crypto backend). With `advance=on` the DMA addresses move past each
  transfer and the first-block flag is spent, as the vendor
  `security.c` expects when it feeds a request in chunks.
- **`gm-pmonitor`**: a started window ends after PERIOD APB clocks
  (at least 1 ms) with interrupt 44; the counters read 0.

The Longse application streams this over RTSP; `qemu/tests/
test-grainmedia.py` drives the encoder through the header FIFO as the
driver does, in CAVLC and CABAC, and checks with ffmpeg that the stream
decodes, the I frame's macroblock means match the source and the P frame
follows the changed macroblock exactly. It also runs an MJPEG job through
the mailbox (decoded by ffmpeg, restart markers included), GM2D fills and
blits, the SSP tone, AES against the FIPS-197 vector, the RTC and the
other small peripherals.

## Not modelled

The ISP's image processing (its frames are the capture's OpenIPC logo
picture; the statistics describe a fixed grey scene), the MCP100's
MPEG-4 (not in the cameras' kernels), the LCD's output, and the timing
of the PWM outputs.
