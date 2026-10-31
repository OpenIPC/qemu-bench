# Xiongmai XM510 / XM530 / XM550 Hardware Reference

Every address here is read from the vendor kernels OpenIPC builds for these
SoCs: 3.0.101 for XM510 (`xm510_lite`) and 3.10.103 for XM530/XM550
(`xm530_lite`, `xm550_lite`). Paths below are relative to those kernel trees.
There is no device tree; both kernels boot with ATAGs.

The kernels began from HiSilicon BSP templates (the mach headers still carry
`__HI_CHIP_REGS_H__` guards), but the memory map is not HiSilicon's, and
neither are the flash controller, GPIO or I2C. Hence a family of its own in
`qemu/hw/arm/xiongmai.c`.

## Summary

| | XM510 | XM530 / XM550 |
|---|---|---|
| CPU | 1× Cortex-A5, MIDR 410fc051 — `cortex-a9` stand-in | 2× Cortex-A5 MPCore, A9-layout private region at 0x20200000 |
| IRQ | PL190-layout VIC at 0x20010000, 32 lines | GICv2 (dist +0x1000, CPU +0x100) |
| L2 | none | PL310 at 0x20210000 |
| DDR | 0x80000000, 32 MB | 0x80000000, 64 MB (XM530) / 128 MB (XM550) |
| UART | PL011 0x10030000 / 0x10040000 / 0x10050000, IRQ 0 / 1 / 2 | same, SPI 0 / 1 / 2 |
| Timers | SP804 0x100C0000 (IRQ 4), 0x100D0000 (IRQ 5), 24 MHz | same, 12 MHz, plus TWD |
| SYS_CTRL | 0x20000000; soft reset +0x30 / +0x34 | 0x20000000; soft reset +0x80 / +0x84 |
| Watchdog | SP805 0x20060000 | same |
| DMA | PL330 0x20020000, IRQ 10 | same |
| SPI-NOR | xmsfc v1, regs 0x100F0000, data window at 0x0 (16 MiB) | xmsfc v2, regs 0x100F4000, window 0x60000000 |
| Ethernet | DWMAC 10/100 0x20040000, IRQ 3, RMII PHY at MDIO 1 | DWMAC 0x10010000, SPI 3 |
| SD/MMC | Arasan SDHCI 0x50000000 (IRQ 24), 0x50010000 (IRQ 25) | 0x50000000 / 0x50100000, SPI 41 / 42 |
| GPIO / mux | 0x10020000, banked: eight pins a port, ports 0..8 (mux `+P*4`, data/OE `+0x40 + P*0x10`) | 0x10020000, one register per pad |
| SRAM | 0x04000000 (U-Boot's first stack, sp = 0x04001000) | same |
| DDRC | Synopsys uPCTL 0x100B0000, PHY 0x100B2000 | same block (U-Boot references both pages) |
| Watchdog clock | 93.75 kHz (24 MHz/256) | 46.875 kHz (12 MHz/256) |
| Boot | ATAGs at 0x80000100, machine 8888, `mtdparts=xm_sfc:` | same |

## Sources

- **Memory map, IRQs:** `arch/arm/mach-xm510/include/mach/platform.h`,
  `irqs.h`, `io.h`; `arch/arm/mach-xm530/include/mach/` likewise.
- **Timer clock:** `CFG_TIMER_CLK` in `mach-xm510/include/mach/time.h`. The
  kernel takes it as a constant, not from a PLL register.
- **Soft reset:** `arch_reset()` in `mach-xm510/include/mach/system.h`, and
  `xm530_restart()` in `mach-xm530/core.c`. Both write 1 to
  `REG_SYS_SOFT_RSTEN`, then 0xca110000 to `REG_SYS_SOFT_RST`.
- **Machine number:** `MACH_XM510` / `MACH_XM530` = 8888 in
  `arch/arm/tools/mach-types`.
- **xmsfc v1:** `drivers/mtd/devices/xmsfc/`.
  - You write CMD, ADDR, WRNUM and the byte buffer at +0x400 (one byte per
    word), then write OP. The OP codes are 2 erase, 3 read register, 4 write
    status and 6 page program.
  - The driver polls ST for bit (1 << (op − 1)). Register reads land in SRR,
    MSB first.
- **Ethernet:** `drivers/net/xmmac/`, a fork of stmmac.
  - `xm510_gmac_platdata` selects the 10/100 core with normal descriptors.
  - Xiongmai moved that core's MAC registers, so the offsets come from its
    `dwmac100.h`: CONTROL 0x00, FRAME_FLT 0x04, HASH 0x08/0x0C, MII 0x10/0x14,
    FLOW 0x18, ADDR0 0x40/0x44.
  - The DMA block is the standard one at +0x1000.
  - `stmmac_main.c` is built with `EDIT_ZENGCHUANJIE 1`, so the driver does
    not subtract the FCS: the descriptor's frame length excludes it.
- **SD:** `drivers/mmc/host/arasan.{c,h}`. These use standard SDHCI offsets,
  and `arasan_set_clock()` waits on CLOCK_CTRL's internal-clock-stable bit,
  which upstream `generic-sdhci` provides.

## Booting a flash dump

The CPU starts at 0x0, which is the xmsfc read window, so a full flash image
boots its own U-Boot in place. There is no mask ROM to emulate:

```sh
qemu-system-arm -M xm510,flash-file=dump.bin -nographic -serial mon:stdio
```

This was checked with an XM510 dump (W25Q64CV, U-Boot 2014.04 "Dec 16 2016"):
U-Boot trains DDR, reads the kernel with `sf`, `bootm`s it, and the camera's
own kernel and rootfs reach `login:`. The dump is written back as the guest
erases and programs, so work on a copy.

OpenIPC publishes no XM U-Boot, so CI cannot do this; it boots the release
kernel with `-kernel` against a synthetic NOR instead.

What U-Boot needs before DDR, read from its code since there is no public
source:

- **PLL lock:** each PLL is programmed through SYS and polled until its
  status word reads 1. Those words are SYS +0x08, +0x1C and +0x28, which
  `xm-sysctl` holds locked (`pll-lock-mask`).
- **SRAM:** the first stack is at 0x04001000, in on-chip SRAM. 64 KiB is
  mapped, a guess.
- **DDR controller:** a Synopsys uPCTL at 0x100B0000. The `xm-ddrc` model
  provides:
  - SCTL/STAT, where state requests (CFG, GO, …) are reached at once;
  - MCMD `start_cmd` (bit 31), which self-clears;
  - POWSTAT `power_up_done`;
  - the PHY's training-done word at +0x1038.

  The kernel's commented-out `DDRC_BASE 0x20110000` is not where U-Boot finds
  it.
- **Flash size:** the XM510 kernel's xmsfc chip table knows only 8 MiB parts
  (W25Q64CV 0xef4017, MX25L6405D 0xc22017), so tests use an 8 MiB NOR.

## XM530 / XM550

- **SMP:** `set_scu_boot_addr()` in `mach-xm530/hotplug.c` brings up CPU1 in
  three steps. `xm-sysctl` models each one, and `xiongmai.c` wires them to
  memory and CPU power:
  1. It writes 1 to `REG_SC_REMAP` (SYS+0x204) and waits for it to read back.
     That puts RAM at address 0, where the boot flash is aliased at reset.
  2. It writes `ldr pc, [pc, #-4]; .word secondary_startup` at physical 0.
  3. With RSTEN set, it writes `0xCA11C100`, then 1, to `REG_CPU1_SOFT_RST`
     (SYS+0x8C), holding CPU1 in reset and then releasing it.
- **TWD rate:** `core.c` computes it from the PLL words:
  12 MHz / refdiv × fbdiv, then ÷ (div + 1) by CPUCLK.
  - QEMU's TWD ticks at 100 MHz. Under `-kernel` nothing has programmed the
    PLLs, so they reset to values that give exactly that: PLLA 0x1901 and
    CPUCLK 0x0B.
  - U-Boot programs a 150 MHz TWD instead, so local-timer ticks on a
    flash-booted guest run 1.5× off.
- **PLL lock:** XM530's U-Boot polls the PLL control words themselves
  (SYS +0x08, +0x10, +0x18, +0x20) until bit 31 (LOCK) reads 1
  (`pll-lock31-mask`).
- **xmsfc v2:** see `xm-sfc.c`. With no JEDEC ID given, the flash reports the
  Winbond part of the image's size: W25Q64CV for 8 MiB, W25Q128BV for 16 MiB.
- **Watchdog and `mem=`:** `xm_wdt.ko`'s init starts the SP805 with a 60 s
  timeout, and only majestic stops or feeds it (`watchdog_disabled_platform()`,
  majestic `src/xm/sdk.c`; OpenIPC/firmware#743). majestic gets there only if
  MMZ is sane:
  - `load_xiongmai` takes `mem=` from the first kernel argument (`awk
    '{print $1}'`) to place MMZ, as U-Boot passes it (`mem=32M console=…`).
  - With `console=` first, MMZ is laid over 0x80000000 + all of RAM,
    kernel included.
  - majestic then Oopses in `mmz` (called from `xm530_rgn`), and the
    watchdog resets the guest a minute later.

  With `mem=32M` first, majestic starts and the camera stays up. That is
  how the flash tests run on XM530 (`--cmdline-prefix mem=32M`).
- **#2613 does not apply on XM530:** its kernel has jffs2 xattr support, so
  it mounts an xattr overlay. `test-overlay-keep.sh` runs on XM510, whose
  kernel lacks it.
- **XM550 flash boot:** `-M xm550,flash-file=dump.bin -smp 2` runs a camera's
  own U-Boot 2014.04 ("Mar 15 2019") through to `login:`. The dump tested
  carries an OpenIPC 3.10.103 kernel and rootfs: U-Boot `sf read`s the
  kernel, both CPUs come up, the root is mounted from flash, majestic starts,
  and the jffs2 overlay is written as on a camera. Getting there took five
  things, each read from that U-Boot:
  1. **A flash alias at 0x08000000** (`sfc_text_alias`). U-Boot is linked
     there, walks `init_sequence` through link addresses before relocating,
     and returns into its relocated copy at `here + reloc_off`. That only
     works if it already runs at its link address, so a flash boot starts
     CPU0 at the alias. That points at a mask ROM jumping there; nothing
     documents it.
  2. **PLL lock** on bit 31 of SYS +0x08/+0x10/+0x18/+0x20.
  3. **DDR polls:** uPCTL +0xF4/+0xF8 read 1, and PHY +0xB8 echoes +0xA8
     (`done-offsets`, `mirror-reads`/`mirror-of` on `xm-ddrc`).
  4. **xmsfc v2 data reads:** OP3 with a read opcode (0x6B) fills the 4 KiB
     RAM buffer at the flash address modulo 4 KiB. U-Boot reads 16 × 256
     bytes, then the whole buffer.
  5. **Status writes:** OP4 with opcode 0x01 writes the status register,
     which is how U-Boot sets QE. Treated as a page program, it had been
     ANDing two bytes into flash offset 0.

## XM530V200 / XM533 / XM535

The second generation (#154), each with its own machine. Dumps of XM530V200
(its U-Boot says `XM550V200WX2`), XM533W1, XM535D1, XM535D2 and XM535W1
cameras boot their vendor U-Boot 2014.04 (2023–2025 builds), 3.10.103
kernel and Sofia:

```sh
qemu-system-arm -M xm530v200,flash-file=dump.bin -nographic
qemu-system-arm -M xm533,flash-file=dump.bin -nographic
qemu-system-arm -M xm535,flash-file=dump.bin -nographic
```

The kernel console stays off unless the saved environment (64 KiB at
0x30000) has `xmuart=0`. The kernels name their machines `xm530v200`,
`qiandaohu` (XM533; its modules say `xm530v300`) and `xm535`. They share
the XM530's static I/O map (0x10000000, 0x20000000). Only the XM530V200
kernel is SMP. Six things differ from the XM530, each read from those
U-Boots and kernels:
  1. **EL3.** `start.S` writes NSACR to open the FPU. That is UNDEF on a
     core without the Security Extensions, so a flash boot keeps EL3 (the
     A5 has it; `-kernel` still runs without). Without it the exception
     goes to the unrelocated vectors at 0x60000004, and the serial port
     never prints a byte.
  2. **Link address 0x60000000,** the xmsfc window itself, not the
     0x08000000 alias. Started at the alias, U-Boot returns from
     `relocate_code` into `here + reloc_off` computed from the alias, below
     DDR. These machines start CPU0 at the window (`flash_entry`) and have
     no text alias.
  3. **DDR is a DesignWare uMCTL2** at 0x20050000, with its PHY at +0x8000
     and +0xC000 (`xm-umctl2`); there is no uPCTL at 0x100B0000. U-Boot polls STAT (operating mode normal),
     DFISTAT `dfi_init_complete` and SWSTAT `sw_done_ack`. It polls PHY
     +0x174 for exact values (0x403 after a training request, 0x30000 after
     a bit-4 step), and PHY +0x158 for 1 or 0xC0 after the +0xA4 / +0xB0
     requests (XM530V200 and XM535D2 only). It also waits on SYS +0xDC
     reading back what it wrote (1, then 5).
  4. **A latching SP804.** The XM530V200 kernel's clocksource writes
     TimerControl with bit 8 set and waits for it to clear, then reads a
     snapshot of TimerValue at +0x1C and inverts it. A stock SP804 has
     neither. Without the latch the kernel's clock never advances and
     printk stalls before the banner. A latch request that leaves the
     control bits unchanged must not reach the stock model, which reloads
     the counter on every control write.
  5. **CPU1 in CPU0's security state.** U-Boot leaves CPU0 secure, and
     `arm_set_cpu_on()` at EL1 would release CPU1 non-secure. Their banked
     GIC groups then disagree, the IPIs between them are lost, and the
     kernel stalls right after `Brought up 2 CPUs`. The XM535 kernels are
     uniprocessor, so only XM530V200 shows it.
  6. **The second SD slot became USB.** XM530's SDIO SDHCI at 0x50100000
     moved to 0x50200000 (SPI 42) on XM533, its card slot (the SSV6158 SDIO
     Wi-Fi is on 0x50000000). On XM530V200 and XM535, 0x50300000 (SPI 44) is the DWC2 USB OTG
     core that `xm_usb0.ko` registers as the `dwc2` platform device (IRQ
     76), for their USB Wi-Fi dongles. With an SDHCI there, `dwc2.ko` fails
     (`AHB Idle timeout`, -16). Unmapped, the arasan driver spins with IRQs
     off on the clock-stable bit, which has no timeout.

  **SD card.** `-drive if=sd,index=0,file=card.img,format=raw` puts a card
  in the board's slot. A blank image is fine: Sofia finds `mmcblk0`, formats
  it with its own WFS filesystem (an `XM`-signed MBR, not FAT), and records
  from then on. The recordings are XM's stream format: `00 00 01 FC` (I
  frame, 16-byte header, length at +12), `FD` (P frame, length at +4), `FA`
  (audio) and `F9` (info), both with a 16-bit length at +6, then the
  HEVC/H.264 NAL units. Which controller has the slot comes from
  `rootfs/usr/etc/loadpublic`, whose `sdioN_sd.ko` registers the arasan
  platform device:
  - XM530V200, XM535D1, XM535D2: `sdio0_sd.ko`, 0x50000000 (SPI 41).
  - XM533: `sdio1_sd.ko`, 0x50200000 (SPI 42); its SSV6158 Wi-Fi uses
    0x50000000. `sd_slot` maps drive index 0 there. A card on the Wi-Fi
    controller probes but gets that device's platform data. Platform data
    +0x18, "auto CMD12", is 0, so the driver sets transfer mode 0x33 and
    never sends CMD12 after a multi-block read; the card stays in the data
    state (`timed out sending r/w cmd command`).
  - XM535W1 (X2W-WNG): no slot. Its `loadpublic` comments `sdio1_sd.ko`
    out.

  Sofia comes up, finds no sensor or Wi-Fi card, and after a while reboots
  the camera itself (`xmsfc_shutdown`, `Restarting system.`). The machine
  reset brings it back through U-Boot. XM210, XM212 and XM232 dumps from the
  same issue are RISC-V parts (`auipc gp` / `csrw mtvec` at flash offset 0)
  and are out of scope for `qemu-system-arm`.

### Video (xm-isp2, xm-hevc, xm-osd, xm-nn)

Sofia captures, encodes and serves RTSP (`rtsp://IP:554/user=admin&password=&channel=1&stream=0.sdp?real_stream`,
`stream=1` the sub stream, `channel=N` per camera on XM535D2). The sensor
is not modelled; the capture blocks produce a test picture, the OpenIPC
logo with a moving dot, as the Allwinner models do:
  - **xm-isp2**: the VICUs (capture, a frame interrupt block per VICU), the
    BPU (Bayer processing, online or offline as isp.ko asks), the VPSS
    (XM530V200's vpss.ko runs memory-to-memory jobs; newer XM535 isp.ko
    runs it inline behind each BPU job) and XM533's VI store. The picture
    is drawn where the frame lands. Multi-lens boards draw each lens with
    the dot at its own phase: XM530V200 X6C and XM535D1 splice two lenses
    into one stream, XM535D2 serves three cameras.
  - **xm-hevc**: the HEVC engines (layouts `xm535`, `xm533`, `xm530v200`).
    The vendor library writes the parameter sets and slice header; the
    model writes the slice data (`hevc-enc.c`) under them, so its CABAC
    must follow the library's choices: CTB 16 without SAO (XM535, XM533)
    or CTB 32 with SAO (XM530V200), deblocking with beta offset +5.
    Every CU is 8x8 with a DC-only intra block, or, in a P picture,
    skipped: zero motion into the previous picture, the only one the
    library's RPS names. A CU is skipped where copying the reference costs
    no more than coding it again (squared error plus lambda times the
    intra CU's bits). The encoder runs the decoder's deblocking filter so
    its reconstruction is the decoder's bit for bit, and keeps each
    decoded picture on the host under the reconstruction address the job
    programs (+0x0B0 on XM535, +0x1F0 on the others); a P job's reference
    address (+0x0A8 / +0x1E8) names it. P pictures of the test picture
    are about 1 KB. `cu_qp_delta` caps the QP at 22 whatever the slice
    QP: the rate control raises that to 44 within a GOP, and DC-only
    blocks need the finer steps. `-global xm-hevc.dump-dir=DIR` keeps
    every job as a whole access unit.
  - **xm-osd**: the OSD engine (rgn.ko) blends the clock and title into
    the frame in place (2 bits a pixel, two regions a job). XM535's main
    streams use the HEVC engine's own overlay regions instead.
  - **xm-nn**: nn.ko's accelerator completes its jobs (nothing is
    computed). Without it Sofia times out (`cnn wait for timeout`) and,
    on XM530V200, crashes.

Some boards need flags: XM535W1 `sensor-id=da4d` and
`-global xm-isp2.vicus=1 -global xm-isp2.sensor-width=1280 -global xm-isp2.sensor-lines=728`,
XM533 `-global xm-isp2.vicus=1 -global xm-isp2.sensor-width=1920 -global xm-isp2.sensor-lines=1088`,
XM535D1/D2 `-global xm-isp2.vicus=2` (D2 also `-m 128M`).

With `-smp 2` the XM530V200 pipeline stalls after a while: isp.ko stops
starting BPU jobs (both ISP buffers end up off its free and BPU lists).
Run it uniprocessor.

### Audio (xm-audio)

`i2s.ko` moves 16-bit mono samples by programmed I/O through two I2S
blocks in the 0x10060000 window; `acodec.ko` programs the codec at +0x8000,
which only has to read back. Per block, AI at +0x6000 (Linux IRQ 48, SPI 16)
and AO at +0x7000 (IRQ 49, SPI 17):

| Offset | Register |
|---|---|
| +0x00 | control: bit 0 on, bit 1 transmit (`ao_open` sets 3), bit 2 receive (`ai_open` sets 0x25) |
| +0x10 | transmit status: bit 2 wants a chunk, bit 1 almost empty, bit 0 underrun |
| +0x20 | receive status: bit 4 a chunk is in, bit 3 full, bit 0 overrun |
| +0x38 | FIFO, 16-bit accesses |

The handler moves one chunk per interrupt, 256 samples on XM530V200 and
160 (20 ms at 8 kHz) on XM535 and XM533, then writes the status words back.
The model raises a chunk every chunk / rate of virtual time while a
direction is on. Capture is a 1 kHz sine (`-global xm-audio.tone=N`), so
the RTSP audio track (PCMA, payload type 8) carries a tone; playback is
dropped, or played to an audiodev:

```sh
-audiodev wav,id=a0,path=out.wav -global xm-audio.audiodev=a0
```

With `-global xm-audio.mic=on` capture comes from that audiodev too. Sofia's
prompts (`welcome.pcm`) land in the wav. The tone arrives about 22 dB down,
which is Sofia's own input gain. On a busy guest Sofia's encoder falls
behind: a uniprocessor XM530V200 streaming its main stream sends about 0.3 s
of audio per second.

### JPEG snapshots (xm-jpeg)

XM530V200 has no `hx280enc`: 0x21100000 (SPI 46) is the JPEG engine that
`jpeg.ko` drives for Sofia's snapshots (VENC channel 4, 640x360). With an
SD card in, Sofia snapshots while it records. Without the engine every
snapshot waits out `jpeg.ko`'s one-second timeout (`wl_jpeg_wait_ready error
status=0x80`, `GetJPEGTimeout`), and the live streams starve: 13 frames in
120 s of RTSP.

`jpeg.ko` writes the library's register list and sleeps until +0x3C reads 3
(or +0x40 has an error bit). The registers the library programs:

| Offset | Register |
|---|---|
| +0x000 | 1: start; the interrupt handler writes 0 |
| +0x00C | height << 16 \| width |
| +0x014 | source stride |
| +0x060 / +0x064 | luma / chroma quantisers, a write each: `1 << 24 \| (0x40 + i) << 16 \| 65536 / q`, natural order |
| +0x088 / +0x08C | where the scan goes / the buffer's end |
| +0x090 / +0x094 | source luma / chroma (NV12) |
| +0x03C | 3: done |
| +0x044 | bytes of scan written |

The library writes SOI, DQT, SOF0, DHT (Annex K) and SOS up to +0x088
itself, and EOI after the scan. The model writes only the entropy-coded
scan, with the encoder shared with `xm-h1` (`jpeg-enc.c`).
`-global xm-jpeg.dump-dir=DIR` keeps each job as a whole JPEG. XM535 and
XM533 have no `jpeg.ko`: their snapshots go through the H1 at the same
address.

### USB and Wi-Fi

XM530V200 and XM535 get QEMU's `dwc2-usb` (raspi's model) at 0x50300000.
Its root port takes any QEMU USB device; give `bus=usb-bus.0,port=1` to
plug one in directly instead of behind QEMU's automatic hub.

Their Wi-Fi is an AltoBeam ATBM60xx USB dongle (`atbm-wifi`):

```sh
# X2-WQ2 (XM530V200, xm713g.ko, 007a:8888, ARES_B)
-netdev user,id=w,net=192.168.50.0/24 \
-device atbm-wifi,bus=usb-bus.0,port=1,netdev=w
# XM535W1 / D1 (6012B_x.ko, 007a:888b, AresLite)
-device atbm-wifi,bus=usb-bus.0,port=1,netdev=w,pid=0x888b,chip-version=0x4a
```

The chip runs an LMAC firmware that speaks ST-Ericsson's CW1200 WSM to the
driver (`hal_apollo`, source in OpenIPC's `atbm_60xx`). The model answers
WSM itself:
  - EP0 vendor requests 0/1 and 4/5 read and write chip memory (wValue /
    wIndex the address halves). The firmware download lands in it.
  - The chip version is at 0x0acc017c. Clearing bit 8 of 0x16101000
    releases the CPU; that sends the startup indication, with
    CAPABILITIES_NO_CONFIRM, which the CONFIG_TX_NO_CONFIRM drivers
    insist on.
  - Bulk OUT carries `wsm_hdr_tx` built with USB_BUS_BUG: flag, usb_len
    and id, len and id. Bulk IN carries `wsm_hdr` plus the body. The
    sequence number is in bits 13..15, the interface in bits 6..9.
  - Commands get a confirm with status 0. Configuration carries the MAC,
    read-MIB a zeroed MIB, and join the power levels.
  - TX requests (the old rate policy: the 802.11 frame at +32) get a TX
    confirm only when htTxParameters asks for one (bit 8). The
    others are freed by the driver itself; 0x0420 must not be sent, as
    xm713g.ko's confirm table traps on it.
  - The model is an access point (`ssid`, default `OpenIPC`; `channel`,
    default 6), open unless `psk=` gives it a WPA2-PSK passphrase:
    - a scan gets a probe response, a beacon and the scan-complete
      indication;
    - it answers authentication and (re)association;
    - 802.11 data frames with LLC/SNAP are bridged to and from the
      netdev, as Ethernet.
  - RX indications are status, channel, rate, RSSI, flags and
    channel_type (the driver is built with 40 MHz support), then the
    frame.
  - Bulk IN NAKs while there is nothing to send. Completing it later
    (USB_RET_ASYNC) would be cheaper, but the driver kills its IN URBs on
    recovery, and the DWC2 model does not cancel a halted channel's
    in-flight packet.

The driver binds, wlan comes up as `eth3`, and wpa_supplicant associates
(`key_mgmt=NONE`). The netdev's DHCP leases an address and RTSP runs
over it. Two limits:
  - **Sofia's own Wi-Fi setup.** With no saved Wi-Fi config, wlandaemon
    hops channels in monitor mode waiting for provisioning (QR code /
    smart link). Sofia also strips any address set on `eth3` by hand. A
    test therefore stops wlandaemon, runs wpa_supplicant and `udhcpc -s
    /usr/sbin/udhcpc.script`, and holds the address.
  - **Cost.** The channel hopping costs guest CPU: on XM530V200 the main
    stream's frame rate over Ethernet about halves with the dongle
    plugged in.

**WPA2-PSK.** With `psk=PASSPHRASE` (8..63 characters) the access point
is WPA2-PSK with CCMP, using the authenticator Ingenic's SDIO chip model
shares (`hw/net/atbm-wpa.c`, documented in `docs/ingenic.md`):
  - beacons and probe responses carry the privacy bit and an RSN IE; after
    association the access point runs the 4-way handshake and group key
    (IEEE 802.11-2016 12.7: PBKDF2 PMK, PRF-384 PTK, HMAC-SHA1-128 MICs,
    the GTK wrapped with AES key wrap). A message 2 whose MIC fails -- a
    wrong passphrase -- is dropped; after four unanswered retries the
    station is deauthenticated. Until message 4 only EAPOL passes;
  - the driver does CCMP in the chip. hal_apollo's `atbm_set_key`
    installs the PTK and GTK with `WSM_ADD_KEY` (the model checks they are
    the handshake's), after which mac80211 only inserts the 8-byte CCMP
    header and the driver leaves 8 bytes of MIC room (`txrx.c`) around a
    plaintext body. The model strips both, and sends frames to the
    station Protected with a CCMP header whose PN rises and MIC room,
    flagged decrypted;
  - the `selftest` property runs the PBKDF2, PRF and key-wrap known
    answers.

The dongle takes the same security options as the SDIO chip
(`hw/net/atbm-wpa.c`, see `docs/ingenic.md`): `proto=wpa` (WPA1),
`cipher=tkip`, `pmf=optional|required` and `rekey=N`, and it answers the
driver's power-mode request with the set-PM-mode indication the driver
waits for.

`qemu-boot/test-xm-wifi.sh --image openipc-xm530v200-nor.bin [--psk P
[--wrong-psk]] [--proto wpa] [--cipher tkip] [--pmf optional|required]
[--rekey N]` boots an OpenIPC XM530V200 build with the atbm-wifi
package (`atbm603x_comb_wifi_usb.ko`) with Ethernet unplugged: wlan0 comes
up, `wpa_supplicant` completes (open, or `key_mgmt=WPA2-PSK`,
`pairwise_cipher=CCMP`), udhcpc leases 10.0.2.15 over Wi-Fi and the host
answers 3 of 3 pings, with the data frames going Protected (27 in a run,
against 2 unprotected EAPOL frames). With `--wrong-psk` the access point
rejects message 2's MIC, deauthenticates after four retries, and the
station stays in `4WAY_HANDSHAKE`. `--proto wpa --cipher tkip --rekey 10`
associates over WPA1 / TKIP (`key_mgmt=WPA-PSK`, `pairwise_cipher=TKIP`)
and keeps pinging across four group rekeys; `--pmf required` installs the
IGTK. Without an image it still checks the
device: the self-test, `psk` reaching it, a short passphrase refused. The
stock X6C-WEQ firmware's `6012B_x.ko` loads its firmware into a
`psk=`-configured dongle and runs its WSM exchange as before; Sofia does
not scan without provisioning (above).

Not modelled: WPA3-SAE and 802.1X (the guests' `wpa_supplicant` has
neither; see `docs/ingenic.md`); XM535D2's `6132cu.ko` and XM533's SDIO
SSV6158.

## XM530 media pipeline (in progress)

What majestic's vendor SDK talks to on XM530. Everything here was read from
the vendor modules and the MPI statically linked into majestic.

| Block | Where | Model |
|---|---|---|
| I2C (`xm_i2c.ko`) | 0x10000000 | `xm-i2c`, a byte FIFO in front of the bus |
| Sensor | I2C 0x1A | `hisi-imx335` |
| VI (`xm530_vi.ko`) | 0x30000000, SPI 33–36; ISP SPI 25–26 | `xm-vi`, test-pattern frames and the ISP's frame interrupts |
| Hantro H1 (`xm530_h264.ko`, hx280enc) | 0x20070000, SPI 45 | `xm-h1`: JPEG and H.264 (I slices, CABAC) |
| Mali-V500/V550 (`mve_base`/`mve_rsrc`) | 0x21000000 | none (H.265: out of scope) |

**I2C (`xm-i2c`).** The controller has these registers:

| Offset | Name | Use |
|---|---|---|
| +0x00 | CTRL | 0x0B starts a write, 0x803 starts a read; bit 6 / bit 7 pulse TX / RX FIFO resets |
| +0x0C | ADDR | 7-bit slave address |
| +0x10 / +0x14 | TX / RX data | one byte per access |
| +0x18 / +0x1C | WCNT / RCNT | byte counts |
| +0x44 | CLR | write-1-to-clear for ST |
| +0x48 | ST | bit 4 write done, bit 5 read done |

The start bits clear when a transfer ends. The driver pulses the FIFO
resets as `old | bit` then `old`, so a start command left in CTRL would
launch another transfer.

**Sensor.** `libsns_X50`'s `sensor_get_chip()` tells an IMX335 by reading
0x3302 == 0x32. That is the BLKLEVEL power-on default, which
`hisi-imx335` now has.

**H.264 and JPEG run on the H1, through Hantro's own `H264Enc` and
`JpegEnc`.** Both are linked into majestic and drive the H1 via EWL and
`/dev/h264`. Only H.265 uses the Mali VE (`/dev/mv500`).

For `XM_MPI_VENC_CreateChn` to succeed on the H1, two registers matter:
- swreg0 must carry an H1 product ID (0x8290);
- swreg63 (+0xFC) must advertise the codecs, H.264 and JPEG bits and the
  max width. With it 0, every channel failed `XM_ERR_VENC_NOT_CONFIG`.

**H1 JPEG (`xm-h1`).** The encoder follows the open-source H1 driver in
mainline Linux (`drivers/media/platform/verisilicon`: `hantro_h1_regs.h`,
`hantro_h1_jpeg_enc.c`, `hantro_jpeg.c`), not vendor code:
- `ENC_CTRL` with EN and JPEG mode starts a job: baseline, 4:2:0, the
  Annex K Huffman tables, quantisers from `JPEG_LUMA/CHROMA_QUAT` in
  `hw_reorder` order;
- the scan and EOI go to `ADDR_OUTPUT_STREAM`;
- `STR_BUF_LIMIT` returns the bit count, and `INTERRUPT` gets
  `FRAME_RDY | IRQ`.

`qemu/tests/test-xm-h1-jpeg.py` drives it over qtest with that driver's
register sequence. The output must decode and match libjpeg's own encode
of the frame.

swreg63 puts JPEG at bit 25 (VS 24, VP8 26, H.264 27), as Hantro's EWL
reads it. With JPEG at 26, majestic's JPEG channel failed
`XM_ERR_VENC_NOT_CONFIG`.

`-global xm-h1.dump-dir=DIR` keeps every job:
- JPEG jobs as complete `.jpg` files, the model writing the header the
  driver would;
- H.264 jobs as `.h264`, the bytes the H1 wrote, an IDR job with the
  SPS/PPS the library put ahead of it, so `cat *.h264` plays;
- each job's registers as `.regs`.

**VI (`xm-vi`).** Learnt from the driver's register writes. For channel `c`,
in window `W = 0x84000 + c*0x1000`:
- `W+0x40/0x44` and `W+0xC0/0xC4` are buffer 0/1 luma and chroma, stored
  as `(addr - 0x80000000) >> 3`, NV12;
- `W+0x20/0x28` is the stride ÷ 8, `W+0x24` the width, `W+0x34` the
  enable;
- `0xF0080 + c*0x20` is the interrupt enable, `+4` the height + 1, and
  `+0xC` the status, cleared by writing 0;
- `W+0x08` holds one done bit per buffer.

Enabled channels get a moving test pattern at `fps` (default 10).

The ISP's two frame interrupts sit in the same grid below the channels:
- `0xF0040`: `isp_irq_bayer`, SPI 25;
- `0xF0060`: `isp_irq_lum`, SPI 26;
- enable at +0, status at +0xC.

They keep VI running. `XM_MPI_ISP_MemInit` stops every channel, and the
ISP's per-frame handler is what starts them again. Without these lines,
VI stopped after six frames.

**H1 H.264 (`xm-h1`, `hantro-h264.c`).** Hantro's library writes the SPS and
PPS in software, up to an 8-byte aligned `ADDR_OUTPUT_STREAM`, and leaves
the H1 the bits that didn't fit: `STR_HDR_REM_MSB/LSB`, with
`RLC_CTRL[28:23]` of them. The model:
- emits those bits first;
- parses the SPS/PPS back out of the stream buffer on every IDR. Rate
  control rewrites the PPS's `pic_init_qp` between GOPs;
- writes a slice that fits them: frame_num and POC widths, PPS id,
  deblocking control, `slice_qp_delta` against the PPS's QP.

Each picture is:
- one I slice, an IDR when `ENC_CTRL` has the intra bit;
- every macroblock I_16x16 with DC prediction;
- only DC coefficients sent.

So the frame comes back at 4×4-block resolution, with exact decoder-matching
reconstruction. majestic's XM backend asks for Main profile, so the PPS is
CABAC; the arithmetic coder and residual coding follow x264
(GPL-2.0-or-later). CAVLC PPSs are not handled yet.

`qemu/tests/test-xm-h1-h264.py` feeds it majestic's own SPS/PPS over qtest,
with a QP change at the second IDR. ffmpeg must decode every frame, each
plane within 45 dB of the 4×4-averaged source.

**Status:**
- majestic configures the sensor and VI, and creates VENC chn 1 (H.264
  704×576) and chn 2 (JPEG).
- VI, the ISP interrupts, and H1 H.264 run at the VI frame rate. A 55 s
  run's dump plays as 498 Main-profile 704×576 frames with no decoder
  errors, IDRs every 25 frames.
- RTSP and HTTP stay 401 until the camera is claimed; the dump is how
  the stream is checked without one.
- CI runs it on every change: `qemu-boot/test-xm-stream.sh` boots the
  release from flash, lets majestic start on its own, and requires 60
  dumped jobs that ffmpeg decodes frame for frame, with at least two IDRs.
- H.265 on the Mali VE stays out of scope.

## XM510 media pipeline

The same blocks as XM530, found the same way, from the register traffic of
the vendor modules:

| Block | Where | Model |
|---|---|---|
| I2C (`xm_i2c.ko`) | 0x10000000 | `xm-i2c`, unchanged from XM530 |
| Sensor | I2C 0x30 | `hisi-jxh62`, the SOI JX-H62 on XM510 boards |
| VI and ISP (`xm510_vi.ko`, `xm510_isp.ko`) | 0x40000000, IRQ 16 and 18 | `xm-vi` with `soc=xm510` |
| Hantro H1 (`xm510_h264.ko`, hx280enc) | 0x20070000, IRQ 26 | `xm-h1` |

**Sensor.** ipcinfo and the vendor `libsns` read the SOI product ID from
registers 0x0A/0x0B at 0x30; 0xA062 is the H62.
- `hisi-jxh62` is `hisi-f37` with that ID. Both are RAM behind the ID.
- With no sensor on the bus, majestic stopped at "Not Support This Sensor!".

**Other sensors.** On every Xiongmai machine `sensor=` replaces the
board's sensor (and its twin, where a board has two) with one of any I2C
device type, or with `none`, and `sensor-addr=` sets its 7-bit address
(1-0x7f: 0 is the general call). `sensor-id=` still picks the part of the
board's own SmartSens model. Boards with a second I2C bus get the sensor on
both, as they get their own. `xm-sensor` is a generic one: a register file with 8- or
16-bit register addresses (`addr-bytes`) and power-on values from `regs`,
which logs every access to `log` (a log that can't be written is reported
once and dropped). With 1-byte addresses `regs` takes registers up to 0xff:

```sh
qemu-system-arm -M xm510,flash-file=dump.bin,sensor=xm-sensor,sensor-addr=0x30 \
    -global xm-sensor.addr-bytes=2 \
    -global xm-sensor.regs=0x3107:0x22,0x3108:0x35 \
    -global xm-sensor.log=sensor.log ...
```

With it the XM510 vendor `isp` daemon, which majestic starts, can be shown any
sensor it supports by that sensor's chip ID. It starts, and logs its init,
with each of these:

| Sensor | Address | Registers | ID |
|---|---|---|---|
| SOI JX-H62, JX-H42, JX-H65 | 0x30 | 8-bit, `0x0a`/`0x0b` | `a0 62`, `a0 42`, `0a 65` |
| SmartSens SC1235, SC1245, SC2235 | 0x30 | 16-bit, `0x3107`/`0x3108` | `12 35`, `12 45`, `22 35` |
| onsemi AR0130 | 0x10 | 16-bit, `0x3000` (16-bit data) | `24 02` |

With `sensor=none` it probes, in order: 0x30 (8-bit `0x0a`/`0x0b`), 0x30
(16-bit `0x3107`/`0x3108`, then `0x3008` and `0x3005`), 0x36 (16-bit
`0x300a`/`0x300b`), 0x10 (`0x3000`), 0x32 (8-bit, `0x00`/`0x01` after
`0xf0`/`0x1c` writes), 0x36 (`0x300b`/`0x300c`), 0x34 (8-bit,
`0x04`/`0x05` after `0xfe`/`0xfd` writes) and 0x30 (`0x30`, `0x2f`), then
gives up. The model stores what the guest writes, so it doesn't reset like
a real sensor, and registers the daemon reads back for values it measures
read 0. `qemu/tests/test-xm-sensor.py` checks the properties and the model.

**VI.** It's the XM530 design at other offsets, with up to three buffers and
a single back-end interrupt for every channel. `load_xiongmai` sets
`VI_BUFF=3` for an 18M kernel and 2 otherwise. For channel `c`, in `W = 0x81000 +
c*0x1000`:
- `W+0x00/04/08` and `W+0x10/14/18` are buffer 0/1/2 luma and chroma, as
  `(addr - 0x80000000) >> 3`;
- `W+0x20/0x28` is the stride ÷ 8, `W+0x24` the width, `W+0x34` the enable;
- `W+0x38` is the last buffer index: 2 for three buffers, 1 for two, when
  buffer 2's registers stay 0;
- `W+0x3C` is the buffer the hardware writes next. The driver takes the one
  before it as the finished frame; reading it as 0 gave buffer 2 every time;
- `W+0x48` is 1 for a finished frame. It isn't a per-buffer mask: the
  driver ignores 2 and 4;
- `W+0x4C` is an overflow flag. When it's non-zero, the handler clears it
  and drops the frame.

There's no height register. Chroma follows luma, so the height is
`(chroma - luma) / stride`: 720 and 576 for the two channels majestic sets up.

Interrupt blocks follow XM530's shape, enable at +0 and status at +0xC:
- `0xF0010` is `vi_front_v` and `isp_irq` (IRQ 16);
- `0xF0030` is `vi_back_v` (IRQ 18), with a status bit per channel.

**Status:**
- majestic finds the H62 and creates VENC chn 0, H.264 1280×720 Main.
- Every VI frame is encoded at the VI rate.
- After VENC starts, the driver first hands out two new frames and then
  one it had queued before; from then on the order is strict.
- `test-xm-stream.sh --soc xm510` runs in CI, booting with `mem=18M` as
  U-Boot passes it.
- XM510's other CI boots use `mem=20M` first. Without a `mem=`,
  `load_xiongmai` lays MMZ over the whole kernel, and now that VI and the H1
  really write their buffers, the guest dies. At 18M, `sysupgrade --archive`
  refuses: /tmp has no room to unpack the archive (4170 KB needed, 3656 KB
  free). That would happen on silicon too.

## Model notes

- **xmsfc status registers:** both versions keep SR1, SR2 and SR3.
  - Reads: 0x05, 0x35 and 0x15 (OP3).
  - Writes (OP4): 0x01 writes SR1, or SR1 and SR2 with two bytes; 0x31
    writes SR2; 0x11 writes SR3. On v2 every other OP4 opcode is a page
    program, so before this an SR3 write ANDed the data RAM into flash.
  - SR3 is non-volatile: it survives a machine reset (not a QEMU restart).
- **xmsfc per-block protection (`-global xm-sfc.wps=on`):** the part powers
  up as Xiongmai bootloaders leave it, with SR3.WPS set.
  - Protection then comes from per-block lock bits, all set at power-up, and
    erases and programs are dropped with no error, as on silicon. With
    `-d guest_errors` the log names each one.
  - 0x98 (v2 OP1 with no address) unlocks every block until the next reset;
    clearing WPS with 0x11 ends the per-block mode.
  - Off by default, so existing images behave as before.
  - Not modelled: the BP bits in SR1 are stored but never enforced, so once
    WPS is clear nothing is protected. SR3 survives a machine reset but not a
    QEMU restart: `flash-file` holds the array only, and `wps` sets the
    power-up state of every run.

- **xmsfc SPI NAND (`-global xm-sfc.nand-file=<raw image>`, v2 only):**
  the controller in NAND mode, as the vendor U-Boot's `xmsfc_v2_nand.c`
  drives it.
  - The image is raw pages of 2048 data + 64 spare bytes, 64 to a 128 KiB
    block. The part is a W25N01GV (EF AA 21) unless `nand-id` says otherwise.
  - OP1 runs an addressed command without data: 0x13 page read to the cache,
    0x10 program execute, 0xD8 block erase. ADDR holds the row.
  - OP2 is a bare command (0xFF reset).
  - OP3 reads: 0x9F ID, 0x0F feature at ADDR, and 0x03/0x0B copy the cache
    from column ADDR into the RAM at the same offset.
  - OP4 writes: 0x1F feature, and 0x02/0x84 copy RAM into the cache at
    column ADDR. 0x02 first clears the cache.
  - Programs AND into the page, and are written through to the file.
  - The part powers up with every block protected (feature A0 = 0x7C).
    Until A0 is cleared, as the vendor driver does before each program and
    erase, those fail with P_FAIL/E_FAIL in feature C0.
  - **The boot path is a guess.** Nothing documents how the XM530 boots from
    SPI NAND. The model has the read window serve the NAND's page data
    linearly, spare skipped, so U-Boot runs in place from it as from NOR.
    The vendor tree links a NAND U-Boot at 0x60001000, behind a 4 KiB
    header. This exists so a NAND U-Boot can be exercised at all; it says
    nothing about silicon.
- **xm-dwmac RX start:** receive DMA starts 1 ms (virtual time) after the
  driver sets DMA_CONTROL.SR.
  - The delay exists because the vendor `stmmac_open()` sets SR before
    `napi_enable()`. An RX interrupt in that gap masks DMA interrupts for good:
    `_stmmac_schedule()` disables them, then `napi_schedule()` is a no-op on a
    disabled NAPI.
  - Flushing QEMU's queued frames on the SR write itself hit that gap on every
    TAP boot, where the host's IPv6 and mDNS chatter is always queued. The
    result was RX packets: 0 and no DHCP lease.
  - On silicon the same race exists, but only for a frame arriving within
    microseconds of `ifconfig up`.
- **Pad GPIO (`xm-gpio`), XM530/XM550:** 0x10020000, one register per pad,
  its mux and its GPIO. The bits are:
  - MUX1-3: bits 1-3;
  - drive strength: bits 7:6;
  - output enable: bit 10;
  - output level: bit 11;
  - input enable: bit 12;
  - input level: bit 13, read only.

  A lab XM550 populates pads 0..99; the words after them are control
  registers and stay RAM. Bit 13 reads the driven level while output is
  enabled, otherwise the pad's input line: qdev GPIO input N at
  `/machine/xm-gpio`, for qtest. Every output change is logged under
  `-d unimp` as `xm-gpio: pad N high/low`, which shows who drives what:
  - the stock vendor ISP pulses the IR-cut pair, pads 1/2, at start;
  - the `gpio-xm530` kernel driver (OpenIPC/linux#98) drives them through
    `/sys/class/gpio` instead.

- **GPIO (`xm510-gpio`), XM510:** the same 0x10020000, but banked, eight pins
  a port, ports 0..8:
  - mux word of port P at `+P*4`: pin b is a GPIO while bits b, 8+b and 16+b,
    its three functions, are all clear;
  - data at `+0x40 + P*0x10`: writes latch the output levels, reads give the
    pins, the latch where the output is enabled and the input line elsewhere;
  - output enable at `+0x44 + P*0x10`, 1 = output.

  Pin b of port P is line P*8 + b, qdev GPIO input/output N at
  `/machine/xm-gpio` as on XM530, and every change in what a pin drives is
  logged as `xm510-gpio: gpio N high/low/released`. A muxed pin drives
  nothing. `+8`/`+C` of a port and everything past the ports stay RAM,
  `+0xD0` included: that is the MAC's speed select, which the `xmmac` driver
  sets for 100 Mb/s, not a tenth port. So is the IR-cut block at `+0x100`:
  the stock ISP daemon unlocks it with `0x1ACCE551`, writes the direction to
  `+0x104` and drives `+0x108` for 200 ms, with port 6 pin 7 (line 55) as the
  driver's enable. On a lab XM510 that moves the filter; here it only holds
  what was written.

  The layout is the vendor's `sdio1_sd.c`/`sdio2_wifi.c` helpers and a lab
  XM510's registers at rest. The stock nightly boots to the same rest state:
  mux 0/1/5/7 = `0x3000`/`0x300500`/`0xf`/`0x1`, and port 8 driving pin 1
  (line 65) high.
- **Regbanks:** blocks with no model yet are RAM-backed `hisi-regbank` stubs,
  so drivers and vendor modules can map them: PL330, RTC and (first
  generation) I2S.
