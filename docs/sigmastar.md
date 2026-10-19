# SigmaStar / MStar ARMv7 (`ssr621q`, `ssd202d`, `ssd201`)

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
| `0x20000000` | DRAM, mirrored through the 512 MiB MIU0 window | RAM |
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
- **Display** (`mstar-disp`). GOP 1 is composited over the MOP video plane and
  scanned out to a graphic console, sized from the GOP stretch window
  (1280×1024 on the NVR); RGB565, ARGB1555, ARGB4444, ARGB8888 and ABGR8888.
  The NVR's Sofia draws its GUI in ARGB1555 through the GE.
- **GE** (`mstar-ge`). Always idle with an empty command queue; executes
  bitblt, rectangle fill and line, and raises its job-done interrupt.

## Not modelled

- HDMI TX: no EDID, so the NVR falls back to its default output mode.
- The video decoder, the scalers and the remaining GOPs (GOP 0 is programmed
  but not scanned out).
- I2C controllers, SAR ADC, RTC (the kernel warns that RTCPWC has no power),
  the MMU block (`HAL_MMU_Reset timeout`), audio.
- infinity3 (MSC313E), infinity6 and mercury5 parts. Daniel Palmer's
  `mstar-miyoo-working` branch has the infinity3 and mercury5 maps.
