# SigmaStar / MStar ARMv7 (`ssr621q`, `ssd202d`, `ssd201`, `ssc375`, `msc313e`)

SigmaStar's (formerly MStar's) Cortex-A7 SoCs share one memory map whatever
the family: DRAM (the MIU0 window) at `0x20000000`, the SPI NOR read window at
`0x14000000`, the A7 private region with the GIC at `0x16000000`, IMI SRAM at
`0xA0000000` and every peripheral behind the RIU at `0x1F000000`. One machine
file (`hw/arm/sigmastar.c`) covers them with a table entry per SoC: the
infinity2m dies, Ifado and infinity3.

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
| `msc313e` | infinity3 | 1× Cortex-A7 | 64 MiB | — | `0xC2` | IPL → U-Boot → kernel → shell |

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
qemu-system-arm -M ssc375,flash-file=cam.bin -serial mon:stdio \
    -nic user,restrict=on
```

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
  and its libx264/libx265 encoders, it encodes the test pattern of the
  other targets (the OpenIPC logo on navy, a sliding dot) at each job's size and
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
  picture (the same logo pattern) after it into the output window
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

## MSC313E (infinity3)

Sources: the vendor I3 SDK `MSC3XX_SDK_V_1711291828-I3` (U-Boot 2015.01,
Linux 3.18.30, whose `infinity3.dtsi` and `drivers/mstar` give the map,
and its prebuilt `image_QFN64M-MSC313E` flash image), the MSC313E
datasheet, linux-chenxing.org's infinity3 and boot ROM pages, and an
8 MiB W25Q64 dump of a retail MSC313E camera (JX-F22 sensor, U-Boot
2015.01-svn129, a Linux 3.18.30 built two months after the SDK's).

```
qemu-system-arm -M msc313e,flash-file=cam.bin -serial mon:stdio \
    -nic user,restrict=on,net=192.168.1.0/24,host=192.168.1.1,hostfwd=tcp::8554-192.168.1.32:554
```

The SPI NOR's JEDEC ID follows the image's size (W25Q128's maker and
type with the capacity byte of the file: an 8 MiB dump is a W25Q64,
`0xEF4017`); `flash-jedec=` overrides it.

The infinity3 keeps the infinity map: the same GIC SPI numbers
(`drivers/mstar/include/infinity3/irqs.h`: UART0 66, EMAC 58, FSP 37,
timers 96/97/108, watchdog 98), UARTs, FSP, BDMA, EMAC and timers, so
those models serve it unchanged. What differs:

- **Boot.** The mask ROM looks for the IPL at NOR `0x4000`, not 0
  (`ipl_offset`); the first 16 KiB are blank on both images. The IPL
  sizes DRAM by the wrap (`64MB`) and does not read the bond strap. The
  MXP partition table follows it (`0x20000` in the SDK image, `0x3000` in
  the camera's), and `IPL_CUST` finds U-Boot through it.
- **SD host reset.** `Hal_SDMMC_Reset` and the IPL ("[HSD] IP Reset
  Switch", its table at IPL `0x7670`) clear reg `0x3F` bit 0 of the FCIE
  banks `0x1410` and `0x1402` and wait for bits [3:1] (the MIU, MIE and MCU
  reset states) to read `0b111`, then set it and wait for them to clear
  (`sstar-riu` `infinity3`; `sstar-fcie` for `0x1410`). Unanswered, each
  step times out after about 1.2 s with "Low Fail" / "High Fail" and the
  boot goes on.
- **SD card** (`sstar-fcie`, banks `0x1410`-`0x1411` at `0x1F282000`, SPI
  51): SDIO0, the FCIE5 on its SDIO port that the vendor U-Boot and kernel
  use for the slot (`drivers/mstar/sdmmc`, `hal_sdmmc_v5.c`). A job runs on
  the `SD_CTL` write with `JOB_START`: the command from the CMD FIFO (reg
  `0x20`, `0x40 | index` then the argument MSB first) to the card, its
  response back into the FIFO (the index or `0x3F` first), `CMD_END`, or
  `CMD_NORSP` with no answer; the data by DMA between the card and DDR
  (`DMA_ADDR`, MIU addresses) or, with `DEST_R2N`, 64 bytes at a time
  through the CIFD (bank `0x1411`); `DATA_END`. DAT0-3 read high: the card
  is never busy. ADMA is not modelled. The card is `-drive
  if=sd,format=raw,file=sd.img`; PM_SD_CDZ (PM GPIO `0x47` bit 2, active
  low) reads "card in" then, else the drivers report no card at once.
  The vendor U-Boot's `mmc info`, `fatls`, `fatload` and `mmc write`/`read`
  work against it (its FAT wants a partition table: an MBR with one FAT
  partition).
- **RTC** (`sstar-rtc`, bank `0x0012` at `0x1F002400`, SPI 76): the
  infinity PM RTC of `drivers/mstar/rtc/ms_rtc.c`, a seconds counter with
  load and latch requests the hardware clears (the driver spins on them
  with no timeout: the boot hung there) and a match alarm. It starts at
  the host's time. infinity2m and later have the RTCPWC instead.
- **Scaler gamma SRAMs** (`sstar-vip-gamma`, `sstar-rgb-gamma`). The SCL
  driver loads the VIP's YUV gamma curves through an indirect port at
  bank `0x1226` regs `0x40`-`0x4F` and the RGB ones through two ports at
  bank `0x1211` regs `0x7A`-`0x7F` (`Hal_VIP_SRAMDumpGammaYUV` /
  `...GammaRGB` in `scl/infinity3/src/mxlib/hal/vip/halvip.c`). Outside
  CMDQ mode it reads each entry back, and on a mismatch waits up to 2 s
  for a scaler frame end, ten times per entry. The camera's kernel loads
  them that way at probe: without the SRAMs its boot sits in the SCL
  probe for hours. The SDK kernel's boot gets past the probe without them.
- **Image sensor** (`sensor=jxf22`, the default, or `none`): a JX-F22 at
  `0x40` on MIIC1 (`0x1F223200`, SPI 69; MIIC0 at `0x1F223000`, SPI 68 is
  the other bus; the MIIC is Ifado's `sstar-i2c`). The camera's
  `libdrv_ms_cus_JXF22.so` checks `Sensor_id_table`, `0x0A` = `0x0F` and
  `0x0B` = `0x22` (`hisi-jxf22`, the SOI model of `hisi-f37.c`). The ISP
  reads its exposure from the registers that driver writes: rows in
  `0x02`/`0x01` (29.63 us each, at most the frame length `0x23`/`0x22`
  less 4), gain in `0x00` (2^[6:4] x (1 + [3:0]/16), 1x-15.5x), the
  window in `0x24`-`0x26` (1920x1080).
- **ISP and CSI** (`sstar-i3-isp`, `/machine/isp`: ISP0 top bank
  `0x1302`, SPI 57; the menuload bank `0x1306`; the CSI MAC bank `0x1204`,
  SPI 65), from `drivers/mstar/isp/infinity3` and its `infinity3e_reg_isp*`
  field maps. Three interrupt groups (1: `0x28` mask, `0x29` force,
  `0x2A` clear, `0x2B` final, `0x2C` raw; 2: packed in `0x2D`-`0x2F`; 3:
  `0x4A`-`0x4E`), mask 1 = masked. While group 1's vsync is unmasked,
  each frame (30 a second) raises ISPIF_VSYNC (group 1 bit 14); at 60 %
  it makes the picture and the statistics and raises AE/AWB done and the
  AE block row; at 90 % the frame end (group 3 bits 4, 5 and 8), the
  frame-done count in `0x7A` [14:8] (which the scaler driver reads too)
  and the CSI's VC0 report bit. It drives the scaler's frame start and
  end.

  The picture is made from the sensor's. The sensor sees a fixed scene
  (the test pattern of the other targets, the OpenIPC logo and its sliding
  dot in an 80 % white on a navy gradient, as linear reflectances) under
  tungsten light (its R and B 0.82 and 0.40 of G), and
  puts out 10-bit Bayer, black at 64, from its exposure rows and gain.
  The ISP runs the stages the camera application turns on (found by
  tracing its register writes): OBC (ISP1 `0x67`-`0x6D`), the AE digital
  gain (`0x6E`/`0x6F`), the white-balance gains (WBG1, ISP6 bank `0x1308`
  `0x48`-`0x4F`, which the 3A drives; WBG0 is off), the 12-to-10 Bayer
  gamma and the 10-to-10 RGB gamma from the tables the menuload copies
  from DRAM (SRAMs 3-5 and 10-12; `0x07` bit 0 queues a table, bit 1
  starts, `0x00` bit 11 is done), a 2x2 demosaic (one RGB pixel a quad:
  960x540 out of 1920x1080), and the RGB-to-YCbCr matrix in ISP0
  `0x3C`-`0x45`, into which the 3A folds its colour correction. LSC,
  defect and noise filters, false colour and the HSV tables are passed
  through (off, or nothing to do on a noise-free scene). The statistics
  come from the data after OBC, before white balance: per block the mean
  R, G, B (and Y for AE) as 8 bits, in the block grid and at the DMA
  bases userspace sets in ISP3 (bank `0x1305`): 128x80 AE blocks of 4
  bytes and 128x90 AWB blocks of 3. The vendor AE and AWB converge on
  them: on the camera dump the exposure settles at 1686 rows (gain
  1.125x), the white logo at full scale over the dark navy, and
  the WB gains at R 1.21, B 2.53, which make the white neutral (1/0.82,
  1/0.40). `qom-set /machine/isp light|light-r|light-b`
  changes the light while running (level in 1/1000 LSB a row; R and B
  per mille of G): less light brings more exposure and gain, daylight
  (530, 680) moves the gains to R 2.00, B 1.48.
  `pixels=off` (`-global sstar-i3-isp.pixels=off`) skips the picture,
  `trace=on` logs the register writes and a line a frame (exposure,
  gains, mean Y/Cb/Cr). Without a sensor, or with the ISP off, the
  statistics read mid grey and there is no picture. A quad row equal to
  the one above is copied, not recomputed: 1080p costs ~0.75 ms a frame
  on the scene (10.6 ms worked out in full).
- **CMDQ** (`sstar-cmdq` `infinity3`, bank `0x1120`, SPI 49). The scaler
  driver queues its register changes while streaming (a size change:
  the camera application rescales the main picture to 1280x720 for the
  encoder) and starts the engine at the next frame end (`0x03` bit 1);
  done is status `0x44` bit 2, on which the driver signals
  CHANGESIZEDONE. Without it the resize ioctl times out every 100 ms while
  the VSRC component holds its port mutex, and the main picture never
  reaches the encoder: the application reboots after 40 s without a
  frame.
- **H.264 encoder** (`sstar-mfe`, banks `0x1324`/`0x1325` at
  `0x1F264800`, SPI 60): `mfev5/mfe6`. The driver writes SPS, PPS and the
  whole slice header itself and pushes the bits through the FDC packer
  (`0x46`-`0x48`); the model appends the slice data, CAVLC or CABAC as
  `0x03` bit 13 says (the camera app streams Main profile, CABAC). Slice
  QP, deblocking control and chroma QP offset come from the registers
  the driver fills from the same values; `cabac_init_idc` is always 0.
  I frames are Intra16x16 without residual where a V, H or DC prediction
  is exact, I_PCM elsewhere; P frames are P_Skip where the picture is
  within 2 of the reference in every sample (`skip-tolerance`), I_PCM
  elsewhere, up to an eighth of the picture a frame (the rest stay
  P_Skip, and the next frame carries on where this one stopped): without
  that, an exposure or white-balance step of the ISP's 3A changes every
  macroblock and makes a P frame of over 1 MiB, which overflows the
  driver's buffer (`mfe-enc buffer full`) and breaks the RTSP stream. A
  P_Skip macroblock decodes to the reference's samples, and so does the
  model's picture. The model deblocks as a decoder does
  and writes that to the reconstruction buffer, the next P frame's
  reference, so FFmpeg decodes to it bit for bit; `sstar-mfe.dump=DIR`
  appends the frames to `DIR/mfe-<w>x<h>.h264`, a file per size. The CABAC
  coder is `sstar-cabac.h`, shared with `sstar-vhe`.
- **JPEG encoder** (`sstar-i3-jpe`, bank `0x1320` at `0x1F264000`,
  SPI 61): jpe2's `_EncFire()` (`hal_jpe_ios.c`). The Q tables go in
  through a table port that the driver reads back entry by entry,
  transposed (entry `i` is raster `[(i & 7) * 8 + (i >> 3)]`). The first
  `frame_start` of each fire comes before the soft reset, with the last
  picture's settings, so the model encodes on a timer that the reset
  cancels. The engine writes the scan only, ending in `FF D9`; the
  application writes the headers with the Annex K tables, and the model
  codes with those. It reads the size back as the write pointer minus the
  start address, which `0x3B` bit 13 swaps into the same registers. The
  driver waits for the interrupt with no timeout, so every start raises
  one.
- **H.265 encoder** (`sstar-vhe`, a Hantro H2 v3, conf bank `0x1328` at
  `0x1F265000`, swregs from `0x1F265200`, SPI 53): `vhe/h2v3`. Each
  32-bit swreg is two RIU words, at `8 * i` and `8 * i + 4`. The driver
  puts VPS/SPS/PPS in front itself and points `BASE_OUTBS_ADDR` past them;
  the core writes the slice NALs, the size goes to swreg9 and swreg1 to
  `IRQ | FRAME_RDY`. The slice header follows the parameter sets
  `h265_enc.c` writes, and the registers give the rest. The slice data
  comes from a small CABAC encoder in the model: DC intra CUs with a
  coded residual in I frames, and in P frames skip or zero-vector merge
  CUs with a residual. It reconstructs as a decoder does, deblocking
  included (SAO is off in every slice), and writes the result to the
  reconstruction buffer, which the next P frame reads as its reference.
  FFmpeg decodes the streams to the model's reconstruction bit for bit.
- **Audio** (`sstar-i3-bach`, banks `0x1502`/`0x1503`, SPI 74), the
  playback reader and capture writer DMAs of the ALSA driver
  (`drivers/mstar/sound/infinity.c`, `infinity_pcm.c`). Levels,
  thresholds and sizes are 8-byte units; the CPU reports what it wrote or
  read by inverting TRIG (`0x82`/`0x92` bit 13) with the amount in
  `0x88`/`0x98`. The interrupt is level and the ISR works by masking:
  playback's period tick is the live underrun (level under buffer -
  period), capture's the live overrun (over one period), re-armed by the
  writer underrun (under 256 bytes); the reader's EMPTY and the writer's
  FULL latch until their clear and are the XRUNs. Time is the virtual
  clock at the programmed rate (`0x02`: reader 8-48 kHz, writer 8/16/32/48
  kHz; `0xEE` mono). Capture is a 1 kHz tone, delivered a period at a
  time (the driver only re-arms its period interrupt once the level has
  dropped under 256 bytes); its level includes the 64-byte local buffer
  the driver takes off. Playback goes to the machine's `audiodev`
  (`-audiodev pa,id=snd0 -M msc313e,audiodev=snd0`), if any. A raw-ioctl
  `aplay`/`arecord` stand-in on the camera's kernel runs 16 kHz mono and
  48 kHz stereo streams with no XRUN and one (playback) or two
  (capture) interrupts a period.
- **GPIO, SAR, PM-sleep interrupts** (`sstar-i3-gpio`): PADGPIO bank
  `0x103C` (in bit 0, out bit 4, output disabled bit 5), PM GPIO bank
  `0x000F` (in bit 2, out bit 1, output disabled bit 0) and the SAR pads
  (bank `0x0014` regs `0x11`/`0x12`), per `gpio/infinity3/mhal_gpio.c`.
  Outputs read back what they drive; inputs read high (pull-ups), so
  `PM_SD_CDZ` says "no card" (`-global sstar-i3-gpio.sd-card=on` puts one
  in). The PM-sleep controller (PMSLEEP bank `0x000E` regs `0x00`-`0x0F`,
  cascaded into MS_IRQ 2, GIC SPI 34; `irqchip/infinity3/irqchip.c`)
  latches PM pad edges at their polarity (pad register bit 7), masks (bit
  4) and clears (bit 6) them, and is high only while a final status bit
  in `0x04`/`0x05`/`0x0E` is set: the vendor handler `BUG()`s otherwise.
  The SAR ADC converts at once.
- **Day and night** (`-M msc313e,daynight=day|night`, default day;
  `qom-set /machine daynight night` while running). `PAD_PM_IRIN` (gpio
  78, the DTS `ircut` UIO device's input, PM-sleep hwirq 22) reads high by
  day and low by night, and its edges raise the interrupt the driver
  flips between rising and falling. The camera application decides from
  the SAR instead: `jco_server`'s `encode_infrared` reads `/dev/sar`
  channel 2 (register `0x42`) and calls night below `dwTurnOnLux` (50,
  hysteresis 3), so that channel reads `0x3FF` by day and `0x008` by
  night. On the camera dump it logs `eDayOrNight:0 change to night:1` and
  back as the property changes, and drives the IR-cut motor on gpio
  98/99.
- **Not there**: SATA, the display, the VPU and GE are infinity2m NVR
  blocks, and Ifado's own media models sit at other banks. The IVE is not
  modelled, nor the scaler's own crop, filters and VIP stages (the camera
  application leaves its VIP gammas off and its CCM at the identity).

**The camera dump.** Its U-Boot environment has `bootcmd` set and
bootargs with `init=/bin/sh`, so the board, as dumped, boots to a bare
`sh-4.0#`; U-Boot arms the watchdog with 60 s and the kernel's driver
re-arms it, so with nothing feeding `/dev/watchdog` it resets every minute,
as the real one would. `exec /init` at that prompt runs the firmware's own
init (`init.rc`, `rcS`, `/ipc/etc/auto_run.sh`), which starts the camera
application `jco_server`: it finds the sensor, runs its ISP and 3A,
scales and encodes the main picture to 1280x720 H.264, feeds the
watchdog, takes its address from its saved configuration (`192.168.1.32`)
and serves HTTP (thttpd, port 80) and RTSP (live555, port 554,
`stream1`, `stream2`). Use `restrict=on`: without it the application
reaches out to its vendor's cloud servers through SLIRP's NAT. Its EMAC
driver prints `probe of emac failed with error -1` after `register_netdev`
(the board's LED GPIOs); `eth0` works.

The vendor SDK's own `one.bin` (`project/I3/image/image_QFN64M-MSC313E`)
is not a full flash image: the SDK's flashing script writes it and then
saves the U-Boot environment, so on its own it stops at the `MStar #`
prompt, on a board as under QEMU.

## Not modelled

- HDMI TX: no EDID, so the NVR falls back to its default output mode.
- The DIVP/DIP scalers (the VPU's own scaler serves the live view) and the
  remaining GOPs (GOP 0 is programmed but not scanned out).
- SAR ADC, RTC (the kernel warns that RTCPWC has no power),
  the MMU block (`HAL_MMU_Reset timeout`), audio output and the
  infinity2m's audio.
- infinity (MSC313, MSC316 without E), infinity6 and mercury5 parts.
  The I3 SDK's kernel also has `infinity.dtsi`; Daniel Palmer's
  `mstar-miyoo-working` branch has the mercury5 map.
- On infinity3, the media blocks (MFE, IVE, audio), the second SD host
  (FCIE, bank `0x1402`, the NAND pads), ADMA, the SAR ADC and the FUART's
  URDMA.
