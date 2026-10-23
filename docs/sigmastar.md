# SigmaStar / MStar ARMv7 (`ssr621q`, `ssd202d`, `ssd201`, `ssc375`)

SigmaStar's (formerly MStar's) Cortex-A7 SoCs share one memory map whatever
the family: DRAM (the MIU0 window) at `0x20000000`, the SPI NOR read window at
`0x14000000`, the A7 private region with the GIC at `0x16000000`, IMI SRAM at
`0xA0000000` and every peripheral behind the RIU at `0x1F000000`. One machine
file (`hw/arm/sigmastar.c`) covers them with a table entry per SoC; so far the
entries are the infinity2m dies.

Sources: the vendor infinity2m SDK (`boot_621_release_20200522`: U-Boot
2015.01; `kernel_621_release_20200522`: Linux 4.9.84), the device tree built
into a Xiongmai NBD80S16S-KL NVR's kernel (model `INFINITY2M SSC010A-S01A-S`)
and that NVR's 16 MiB SPI NOR dump, and Daniel Palmer's MStar QEMU work at
[github.com/fifteenhex/qemu](https://github.com/fifteenhex/qemu) (branches
`mstar-clean` and `mstar-miyoo-working`), whose register findings were made on
SSD202D silicon with `contrib/mstarpoker`. The GE and display models are ported
from there; the reset values of the clock, chiptop, efuse and USB banks, the
MIU and l3bridge status bits, the smpctrl mailbox and the timer semantics come
from there too.

## Machines

| Machine | Die | Cores | DRAM | Bond (chiptop +0x120) | Chip ID (`0x1F003C00`) | Boot tested |
|---|---|---|---|---|---|---|
| `ssr621q` | infinity2m | 2× Cortex-A7 | 256 MiB | `0x1E` | `0xF0` | yes: IPL → U-Boot → kernel → Sofia GUI |
| `ssd202d` | infinity2m | 2× Cortex-A7 | 128 MiB | `0x1E` | `0xF0` | — |
| `ssd201` | infinity2m | 2× Cortex-A7 | 64 MiB | `0x1D` | `0xF0` | — |
| `ssc375` | Ifado | 1× Cortex-A7 | 128 MiB | `0x15` | `0xF0`¹ | IPL → U-Boot → DHBoot → kernel → rootfs |

¹ Not checked against silicon: the boot chain to userspace does not look
at it. The SoC is named by the firmware itself (`CHIP=SSC375` in the Dahua
rootfs; kernel compatible `sstar,ifado`).

The SSR621Q is the infinity2m die in Xiongmai's 4/8/16-channel NVRs, with
SATA, HDMI and the video decoder bonded out. Its IPL reads the bond strap to
pick the DRAM profile and reports `256MB` and `BIST0_0001-OK` here.

```
qemu-system-arm -M ssr621q,flash-file=nvr.bin -serial mon:stdio \
    -nic user,net=192.168.1.0/24,host=192.168.1.1 \
    -drive if=ide,index=0,file=hdd.img,format=raw \
    -device usb-tablet,bus=usb-bus.0 -device usb-kbd,bus=usb-bus.1
```

or `FLASH=nvr.bin qemu-boot/run-ssr621q.sh`. `flash-file=` is the board's
whole SPI NOR; erases and programs go back to it, so boot a copy.
`flash-jedec=` overrides the JEDEC ID (default MX25L12835F, `0xC22018`).
`-kernel` boots a kernel directly; the vendor kernel builds its device tree
from the ATAGs (`atags_to_fdt`).

## Boot

The mask ROM checks for `IPL_` at offset 4 of the flash (through the XIU
window), copies the IPL (size at +8) to IMI at `0xA0000000` with BDMA and
enters it with its stack at `0xA000D000` (Daniel Palmer's notes on the SSD202D
ROM). A synthetic ROM does the same: it repeats the ROM's own register writes
(watchdog off, timer 0 free-running, the UART pad mux), sets the stack and
jumps to the IPL QEMU has placed in IMI. `-bios rom.bin` runs a real ROM dump
instead, which loads the IPL itself through the DID strap, the FSP and BDMA.

The IPL brings up the PLLs and the MIU, sizes DRAM by the wrap of the 512 MiB
window, loads `IPL_CUST`, which unpacks U-Boot (XZ) into DRAM. The NVR's U-Boot
reads the romfs partition, takes `/boot/uImage` out of the squashfs and boots
it.

**Xiongmai console.** Xiongmai's U-Boot steers the kernel console by the
`xmuart` environment variable, passed in a custom ATAG: unset (or `1`) and the
kernel prints nothing on UART0; `xmuart=0` puts the console on UART0.
`qemu-boot/sstar-env.py` sets it in a copy of a dump:

```
qemu-boot/sstar-env.py nvr.bin xmuart=0
```

(the environment is at flash `0x40000`, 64 KiB, CRC32 + `name=value` pairs).
U-Boot takes a valid block instead of its built-in defaults, so when the
dump's block is blank the script starts from those defaults, unpacked from
the U-Boot image in the dump.
The U-Boot console is password protected.

## Memory map

| Address | Block | Model |
|---|---|---|
| `0x00000000` | mask ROM, 16 KiB | synthetic ROM, or `-bios` |
| `0x14000000` | SPI NOR read window (XIU) | `sstar-fsp` |
| `0x16000000` | A7 private region: GIC-400 (distributor +0x1000, CPU interface +0x2000) | `a15mpcore_priv` |
| `0x1F000000` | RIU: banks of 128 16-bit registers on a 4-byte stride | `sstar-riu` under the models below |
| `0x1F001000` | ISP (bank `0x0008`): byte-wise SPI master | `sstar-fsp` |
| `0x1F002C00` / `0x1F002E00` | FSP sequencer / QSPI (banks `0x0016`, `0x0017`) | `sstar-fsp` |
| `0x1F006000` | watchdog + 3 PIU timers (bank `0x0030`) | `sstar-timer` |
| `0x1F006200` | EMAC internal PHY (banks `0x0031`-`0x0033`) | RIU storage, read through by `sstar-emac` |
| `0x1F200400` | BDMA, 4 channels (bank `0x1002`) | `sstar-bdma` |
| `0x1F201200` | main interrupt controller (bank `0x1009`) | `sstar-intc` |
| `0x1F204000` | smpctrl: CPU1 boot mailbox (bank `0x1020`) | machine |
| `0x1F220400`, `0x1F221000`, `0x1F221200`, `0x1F221400` | FUART, UART0 (console), UART1, UART2 | `sstar-uart` |
| `0x1F225000` | display top: timing, vsync (bank `0x1128`) | `mstar-disp` |
| `0x1F246800` | GOP 1, the plane the NVR's fbdev uses (banks `0x1234`/`0x1235`) | `mstar-disp` |
| `0x1F280A00` | MOP video plane (bank `0x1405`) | `mstar-disp` |
| `0x1F281200` | GE 2D engine (bank `0x1409`) | `mstar-ge` |
| `0x1F284800`, `0x1F285000`, `0x1F286400` | UHC0-2: EHCI hosts (banks `0x1424`, `0x1428`, `0x1432`) | `sstar-uhc` |
| `0x1F2A2000` | EMAC, RIU view (16-bit halves) | `sstar-emac` |
| `0x1F343C00` | EMAC, X32 view (32-bit) | `sstar-emac` |
| `0x1F345000` | SATA AHCI, one port | `sysbus-ahci` |
| `0x20000000` | DRAM, mirrored through the 512 MiB MIU0 window and the 512 MiB after it (the Xiongmai U-Boot writes its logo at `0x4DD00000`) | RAM |
| `0xA0000000` | IMI SRAM, 64 KiB | RAM |

A register `r` of RIU bank `b` is at `0x1F000000 + b * 0x200 + r * 4`; its
high byte is at `+1`. `sstar-riu` keeps every bank no model claims as storage
(firmware read-modify-writes clock gates, pad muxes and PLL words) and answers
the bits the boot chain waits on: MIU init done and BIST done (bank `0x1012`
regs `0x00`, `0x70`), the l3bridge flush done (`0x1022` reg `0x10`), the
chiptop bond strap, the UTMI calibration done (`0x1421`/`0x1425`/`0x1429` reg
`0x1E`), and the internal PHY's link and IDs. Writing `0x79` to PM_SLEEP reg
`0x2E` resets the machine (U-Boot's `reset_cpu`).

Debugging aids: `-d unimp` reports the first access to each unmodelled bank
and any register read 64k times in a row ("hot poll": firmware waiting on
hardware that is not there); `-global sstar-riu.trace=0x1234,0x1235` logs
every access to the listed banks.

## Interrupts

Peripheral lines are GIC SPIs of the same number as the vendor DTS gives
them; the main interrupt controller only masks them on the way. SPIs 32-95
are its level "IRQ" group (mask at reg `0x54`, polarity `0x58`), SPIs 96-127
its edge "FIQ" group (mask `0x44`, polarity `0x48`, write-1-to-clear status
`0x4C`). Lines come out masked.

| SPI | Device |
|---|---|
| 37 | FSP |
| 50 | SATA |
| 52, 82 | GOP vsync, display top vsync |
| 53 | video decoder (VPU) |
| 57 | GE |
| 58 | EMAC |
| 63, 65, 87 | UHC2, UHC1, UHC0 |
| 66, 67, 71, 79 | UART0, UART1, UART2, FUART |
| 72, 73, 93, 94 | BDMA 0-3 |
| 96, 97, 98, 108 | timer 0, timer 1, watchdog, timer 2 |

The ARM generic timer runs at 6 MHz (the DTS `clock-frequency`); the PIU
timers count the 12 MHz crystal, timer 0 the 432 MHz MPLL divided by
`DIVIDE + 1` once the IPL switches it (clkgen `0x1F207004 = 0x30`, divide 35).
Reading a counter's low half latches its high half: U-Boot takes any step
back for a 32-bit wrap.

## Devices

- **SPI NOR** (`sstar-fsp`). One generic 3-byte-address NOR behind the four
  faces of the controller. The FSP runs up to three commands, each its own
  chip select, from one write buffer into one read buffer; its "outside"
  write buffer takes page data from BDMA (U-Boot's `HAL_FSP_Write_BDMA`:
  WREN; PP addr + 256 bytes; RDSR).
- **BDMA** (`sstar-bdma`). MIU ↔ MIU/IMI, memory fill, SPI → memory (how the
  ROM loads the IPL), MIU → FSP.
- **EMAC** (`sstar-emac`). An AT91RM9200-style EMAC: TX by TAR/TCR, RX on a
  ring of 8-byte descriptors with the length including the FCS, MStar's "RX
  delay" interrupt (J104 bit 7 enables, J108 bit 15 pending) because the
  vendor kernel never enables RCOM. The RIU view splits each register in two
  16-bit words; U-Boot writes the low half first, so commands fire on the
  high half. DMA addresses are MIU bus addresses. The internal PHY answers at
  MDIO address 0 and through its RIU bank, with the link up at 100 Mbit/s full
  duplex.
- **USB** (`sstar-uhc`). Each UHC is QEMU's `fusbh200-ehci-usb` (the same
  register layout: PORTSC at `0x30`, the bus monitor `bmcs` at `0x40`) behind
  the RIU's 16-bit halves. The controllers take physical and MIU bus
  addresses alike. `usb-bus.0` is the vendor's `Sstar-ehci-1` (UHC0),
  `usb-bus.1` `Sstar-ehci-2`, `usb-bus.2` `Sstar-ehci-3`. `usb-tablet`,
  `usb-kbd` and `usb-storage` attach as USB 2.0 devices. Each 16-bit half
  (or byte) is written through at once, merged with the live rest of the
  register minus its write-1-to-clear bits: U-Boot's USB stack writes
  single halves, and finds a `usb-storage` stick on `usb-bus.1`.
- **SATA**. QEMU's `sysbus-ahci`, one port, `-drive if=ide,index=0`. The NVR
  formats a blank disk at boot (its own WFS filesystem, no partition
  table). The vendor driver passes MIU bus addresses, and runs the PRD high words through the
  same subtraction (`0` becomes `0xE0000000`), so the bus masters' address
  space repeats there.
- **Display** (`mstar-disp`). GOP 1 is composited over the MOP video windows
  and scanned out to a graphic console, sized from the GOP stretch window
  (1280×1024 on the NVR); RGB565, ARGB1555, ARGB4444, ARGB8888 and ABGR8888.
  The NVR's Sofia draws its GUI in ARGB1555 through the GE, with the GOP's
  transparent colour (reg `0x00` bit 11, key in regs `0x24`/`0x25`: black)
  keyed so the live view shows through. The MOP has 16 windows, `0x40`
  bytes apart from bank `0x1406`: NV12 address (regs 2-5, 16-byte units),
  the rectangle on screen (6-9), pitch (`0x0A`), source size (`0x0B`,
  `0x0C`) and source/destination ratios in 1/4096ths (`0x0D`, `0x0E`); the
  9-split puts each channel in a 426×340 cell.
- **Video decoder** (`sstar-vpu`). A Chips&Media WAVE511 at `0x1F344800`,
  modelled at its firmware's mailbox (no firmware runs; `chagall.bin` is
  loaded and ignored): boot, decoder instances, the bitstream ring,
  INIT_SEQ and DEC_PIC with their interrupts and result reports, decoded by
  libavcodec (H.264, HEVC). The SigmaStar driver differs from mainline
  Linux's `wave5` in where the completions and the queue status are read
  and in passing an output buffer with each DEC_PIC, which the firmware's
  scaler fills (NV12, at the size the display window wants) and the result
  hands back with the picture's VLC buffer; the file's header comment has
  the register-level detail, read off `mi_vdec.ko`. `qemu/setup.sh` builds
  the decoding in when pkg-config finds libavcodec and libavutil
  (`libavcodec-dev`); without them the device still runs the protocol, but
  finds no sequence in any stream. `-global sstar-vpu.trace=on` logs the
  mailbox and each decoded picture.
- **GE** (`mstar-ge`). Always idle with an empty command queue; executes
  bitblt, rectangle fill and line, and raises its job-done interrupt.

## Live view

The NVR shows an RTSP camera's stream once one is on its network.
`qemu-boot/fake-ipcam.py` is such a camera: ONVIF and RTSP on the host,
streaming a lossless synthetic H.264 test pattern (colour bars, a frame
counter, a moving box) that it encodes itself, with no encoder library.
`CAMERA=1` makes the run script start it and put it at `192.168.1.10`:

```bash
CAMERA=1 FLASH=nvr.bin HDD=disk.img bash qemu-boot/run-ssr621q.sh
```

Then, in the GUI, open the digital channel page (数字通道配置: the first
boot's setup wizard offers it after its device search, as 继续添加; the
dump's login is `ucxe` / `3hfrxa`) → `+` on channel 1 → protocol `RTSP`,
address `192.168.1.10`, port `554` → OK (the on-screen keyboard; the USB
keyboard does not reach these fields). The NVR logs in, plays both
streams, and the first cell of the live view shows the pattern. The
channel is kept in the flash, so later boots reconnect on their own.
Discovery (ONVIF WS-Discovery, Xiongmai's broadcast) needs multicast,
which SLIRP does not carry; the camera answers ONVIF too, for adding it
as an ONVIF device by address.

Sofia does not always start feeding a camera's stream to the decoder
after boot: `/proc/mi_modules/mi_vdec/mi_vdec0` then shows the channel
started but `SendCnt` 0, the cell keeps its "H.265+" placeholder, and
another boot is the cure.

## SSC375 (Ifado)

Source: the 16 MiB NOR of a Dahua IPC-HDW1433V-A-IL2 (kernel DTB model
`IFADO SSC032B-S01A`, Linux 4.9.84, U-Boot 2021.10, Dahua DHBoot 2.0),
read against the newer vendor FSP-QSPI driver in the Iford/Pcupid kernel
(linux-chenxing-vendor-slop, `drivers/sstar/fsp_qspi/hal/iford`), the
infinity2m SDK's AESDMA HAL, and the Maruko (SSC37X / infinity6c) SDK
`Maruko-ILS00_TINY_V1.4.0`, whose I2C and clock drivers are the same
generation. One Cortex-A7, the infinity memory map, and:

```
qemu-system-arm -M ssc375,flash-file=cam.bin -icount shift=0,sleep=on \
    -serial mon:stdio -nic user,restrict=on
```

`-icount shift=0,sleep=on` is needed on anything but a fast, idle host.
Without it the guest clock follows the host's. Linux re-arms U-Boot's
60 s watchdog at 8 s of uptime, and sonia takes it over (120 s, fed
every 6 s) only once its media pipeline is up: at 22 s on an i9-13900,
much later on a slower or busy host, so the camera boot-loops. A fixed
shift ties guest time to the instructions run. shift=0 is a 1 GHz guest,
as the SSC375 is, and sleep=on lets an idle guest's time pass in real
time. Under a host load of about 90, the camera then booted with no
reset and served its web UI within 2 minutes; without it, it booted 5
times in 4 minutes and never started its web server. shift=auto does not help, because it stretches guest
time to keep pace with the host. The machine warns at start when the
shift is not fixed.

`sensor=` picks the image sensor on MIIC1 at `0x30` (the DTB's sensorif
`snr0_mipi_i2c = <1>`): `sc431hai`, the board's and the default (ID
`0xCD6B` in `0x3107`/`0x3108`, as its `sc431hai_mipi.ko` reads it), or
`none`. It is the generic `hisi-smartsens` model.

`uuid=` is the die's unique ID, which the kernel's `msys_read_uuid` reads
from OTP2 (bank `0x0022`, regs `0x7C`-`0x7F` at `0x1F0045F0`, low word
first). The Ifliegen SDK's Ifado header puts OTP2 at bank `0x101F`, but
the shipped kernel does not read it there. Unset, the ID reads as zero,
and sonia logs `sigmastar SSC378Q id is all zero!`. Dahua's analytics
licence (`/mnt/mtd/forever/License.dat`) is signed for the camera's own
ID, so it loads only when given that camera's value, e.g.
`-M ssc375,flash-file=cam.bin,uuid=0x0123456789ABCDEF`.

- **IPL.** It runs from IMI past 64 KiB (stack `0xA00155E0`), so IMI is
  128 KiB here. It picks the DRAM profile from the bond strap, `0x10`-`0x19`
  (QFN88 WB512M/ESMT512M/ETRON512M DDR2, NTC1G/WB1G/PM1G DDR3); `0x15`, WB1G
  DDR3, gives the board's 128 MiB. Its SMF stub sets bank `0x1121` reg
  `0x37` bit 2 and waits for reg `0x38` to read 3 (`smf-ack`).
- **NOR.** The boot chain rewrites the active NRI record at flash `0x8000`
  to match the JEDEC ID it reads (W25Q128 by default), with Quad Page
  Program (`0x32`) and the extended address register (`0xC5`/`0xC8`), and
  resets the flash (`0x66`/`0x99`) before reading its status. The FSP-QSPI
  has a software chip select (QSPI `0x0A`): the opcode and address go in one
  fire and the 256 data bytes from BDMA, through the outside buffer, in the
  next, the sequencer waiting for them (`sstar-fsp` `sw-cs`). U-Boot loads
  the NRI records only when the boot strap at PM clkgen `0x1F001C7C` [14:10]
  says SPI NOR (`0x7800` here).
- **DHBoot** checks the kernel and the device tree with SHA-256 and a
  2048-bit RSA public key through the engines at bank `0x1122`
  (`sstar-crypto`); the signatures verify, so the images are the board's.
- **Kernel.** cpufreq waits for the CPU PLL's LPF glide (bank `0x1032` reg
  `0x5D` bit 0; `0x1032BA` in the Maruko `cpu_dvfs()`), and the clock
  driver divides by the PLLs' LPF words while nothing has set them: the CPU
  PLL's current word (regs `0x60`/`0x61`) resets to `0x374BC6`, 1 GHz, and
  the IPU PLL (bank `0x1035`) to what the kernel's `hal_clk_ipupll_init`
  programs. The NVR's SATA, display, video decoder and GE are not there:
  Ifado's video encoder sits inside the VPU's window at `0x1F34D000`.
- **I2C.** `sstar-i2c` is the MIIC master at `0x1F223000`, `0x1F223200` and
  `0x1F222E00` (SPIs 68, 69, 92), from the Maruko `hal_iic_reg.h`: byte mode
  (START, data, read trigger, STOP, the write-1-to-clear flag at `0x04`)
  and the DMA mode the `dma-enable` nodes use (command bytes, then the
  buffer in DRAM, done flag `0x24`, count `0x2C`). Before each transfer the
  driver reads the SCL/SDA pads through GPIO (PADTOP bank `0x103E`, one
  register per pad, bit 0 the level); pads in their function mode read high
  (`gpio-bank`). The consoles are `dma-enable`: UART0/1 and the FUART move
  bytes through URDMA rings (`sstar-uart` `urdma`, SPIs 93, 94 and 80; on
  this die 93/94 are not BDMA's). The TX interrupt bits differ from
  infinity2m: enable/clear/status are bits 10/11/14 of URDMA `0x0D`.
- **Media.** No source exists for the media drivers, but the camera's
  `sstar_mi.ko` keeps its symbols; the models follow it. `sstar-vif` (bank
  `0x1308`: channel interrupts at regs `0x60`/`0x6A`/`0x70`, SPI 89) and
  `sstar-isp` (bank `0x1302`: seven groups of mask/force/clear/status
  from reg `0x28`, SPI 57) are the frame clock: each frame the VIF raises
  channel 0's events, which hand the ISP its input
  (`MI_ISP_PASS_OnEnqueueInputTask` → `COMMON_ISP_Open`, after which the
  "early CameraOpen" refusals stop), and the ISP raises frame start, frame
  end and the statistics events `HalIspIsr` maps. `sstar-cmdq` runs the
  four command queues (banks `0x1120`, `0x1123`-`0x1125`, SPI 49): 8-byte
  commands, pointers in 16-byte MIU units; a write pointer at the ring's
  end means its start, or the queue replays the whole ring. The scaler's
  tasks complete on its CMDQ fence, so it needs no model of its own.
  `sstar-venc` answers the encoder's RISC-V firmware
  (`0x1F34C800`/`0x1F34CC00`, SPI 53): the boot, the job mailbox, the
  status block whose recon, side-plane and bitstream end pointers
  `HalVencDpbDoneFrame` range-checks, the stop/reboot the driver does to
  switch a stream to H.265, and the output buffer the driver hands over
  after each frame (`HalVencIosAddOutBuffer`, ctrl `0x204`-`0x20C`).
  Frame done is enc `0x60` bit 0, which the ISR turns into MI's
  `FRM_DONE`; bits 5, 6 and 1 are a buffer ready, a slice done and a
  ring drop (the names are in the Ifliegen SDK's debug information), and
  none of them hands the stream to `MI_VENC_GetStream`. The ISR clears
  the status before it reads the shared registers and writes the next
  output buffer, so the model reports the next done 2 ms later; each job
  completes 10 ms after it is fired. Built with libavcodec
  and its libx264/libx265 encoders, it encodes a test pattern (colour
  bars, a moving box, the frame number in binary) at each job's size and
  codec (program registers `0xA0`, `0x1080` bit 1, `0xBC` bit 8 for an I
  frame) as the job is fired, with the virtual clock stopped: libx265
  takes tens of milliseconds a frame, and a guest left running meanwhile
  queues the channel's next frame onto the ring-pool record of the one
  in flight. `-global
  sstar-venc.dump=PREFIX` appends each stream to `PREFIX-ctxN.h264` or
  `.h265` on the host. Without libavcodec a frame is an access unit
  delimiter. `sstar-jpe` (bank `0x1237`, SPI 61) is the JPEG engine.
  The driver writes the whole JFIF header (SOI to SOS, regs `0x1D`/`0x1E`,
  size in `0x1C` [14:8]) to DRAM. The model parses its quantisation
  and Huffman tables and frame size, codes a baseline scan of a test
  picture (bars and a frame counter) after it into the output window
  (`0x51`..`0x54`), and reports the result in `0x63`..`0x6C`. Frame done
  is interrupt bit 0 (`E_JPE_HAL_IRQ_FRAME_DONE`). Bit 1 is
  `BSPOBUF_FULL`, which the driver reports as "output buff is too
  small". The register names come from the Ifliegen SDK, whose
  `sgs_mi.ko` carries debug information (`hal_jpe_ios.c`). That SDK's
  kernel also holds Ifado's own GPIO, IRQ, I2C and watchdog HALs
  (`drivers/sgs_common/*/hal/ifado`).
- **Audio input.** `sstar-bach` (bank `0x1502`, SPI 74) is the BACH
  capture DMA, DMA 0 to the driver. `HalBachReadReg` bank index 0 is RIU
  bank `0x1502`, 1 is `0x1503`, 2 is `0x1504` and 3 is `0x1034`. `0x92`
  holds address [14:3], latch (bit 12), read trigger (a change of bit 13)
  and run (bit 15). `0x94` is address [29:15], `0x96` the size, `0x98`
  the bytes read, `0x9A` the threshold and `0x9E` the latched level, all
  in 8-byte units. The ring fills with a 1 kHz tone at 16 kHz.
  `_AudDmaIrq` acts only on a flag (`0x90`) whose enable (`0x80`) is set:
  bit 5 (level at or above the threshold) with bit 11, or bit 1 (overrun)
  with bit 14. Any other condition leaves the interrupt unmasked, and it
  storms the CPU: an RCU stall at about 43 s in `sstar_urdma_task`.
- **Reset button.** The camera's reset key is `PAD_SAR_GPIO0` (gpio 81,
  `pdc` prints it in hex: `keyScanGpio : 51`). Its level is bit 8 of PM SAR
  register `0x12`, which reads high as an input (`sstar-riu`, with
  `gpio-bank`); read low, the button is held, and sonia does a factory
  reset and reboots.
- **Status.** The rootfs mounts and Dahua's userspace starts (`CHIP=SSC375`);
  `sonia` finds the sensor, the 3A thread runs on the statistics events, and
  the network services come up: the RTSP server on 554 answers `OPTIONS`
  through a `hostfwd`. Frames flow VIF → ISP → scaler → encoder with no
  stalls for minutes: both streams in H.264 and then the H.265 the app
  switches to, and the JPEG snapshots. The encoded streams decode on the
  host (the `dump` files), and sonia takes them from the encoder
  (`MediaApp` stops reporting `bitrate:0` for streams 0 and 1), for its
  RTSP server on port 554; `MI_AI_Read` returns microphone frames. A
  300 s boot encodes about 7500 frames with no `MI ERR`. sonia takes
  JPEG snapshots only on demand, so the snapshot channel times out
  ("process data exceed timeout") after the first frames. Open: the
  on-device analytics, which fail their licence check (`invalidLicense`)
  and leave their YUV queue full. The CMDQ read pointer must equal the
  write pointer when caught up, the ring's end included, or
  `MHal_cmdq_CheckBufferAvailTimeout` sees a full ring. U-Boot starts
  the watchdog with 60 s, so a boot that stalls before Linux services
  it resets.

## Not modelled

- HDMI TX: no EDID, so the NVR falls back to its default output mode.
- The DIVP/DIP scalers (the VPU's own scaler serves the live view) and the
  remaining GOPs (GOP 0 is programmed but not scanned out).
- SAR ADC, RTC (the kernel warns that RTCPWC has no power),
  the MMU block (`HAL_MMU_Reset timeout`), audio output and the
  infinity2m's audio.
- infinity3 (MSC313E), infinity6 and mercury5 parts. Daniel Palmer's
  `mstar-miyoo-working` branch has the infinity3 and mercury5 maps.
