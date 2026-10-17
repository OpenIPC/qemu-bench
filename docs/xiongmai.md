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
| GPIO / mux | 0x10020000, one register per pin | same |
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

## XM530 media pipeline (in progress)

What majestic's vendor SDK talks to on XM530. Everything here was read from
the vendor modules and the MPI statically linked into majestic.

| Block | Where | Model |
|---|---|---|
| I2C (`xm_i2c.ko`) | 0x10000000 | `xm-i2c`, a byte FIFO in front of the bus |
| Sensor | I2C 0x1A | `hisi-imx335` |
| VI (`xm530_vi.ko`) | 0x30000000, SPI 33–36 | `xm-vi`, test-pattern frames |
| Hantro H1 (`xm530_h264.ko`, hx280enc) | 0x20070000, SPI 45 | `xm-h1`: JPEG encodes; H.264 not yet |
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
- other jobs as the bytes written, with what software put ahead of them
  (`.pre`);
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

**H.264 on XM needs CABAC.** majestic's XM backend asks for Main profile.
The Hantro library writes an SPS with profile 77 and a PPS with
`entropy_coding_mode_flag = 1` ahead of the H1's output, which the model
can read from guest memory.

**Status:**
- majestic configures the sensor and VI, and creates VENC chn 1 (H.264
  704×576) and chn 2 (JPEG).
- VI delivers frames: `RecvCnt`/`SendCnt` climb, and VENC chn 1 starts
  with `SucCnt` > 0.
- H.264 jobs aren't encoded yet, so frames back up after a few.
- Next: H.264 (CABAC, I slices).
- H.265 on the Mali VE stays out of scope.

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
- **Regbanks:** blocks with no model yet are RAM-backed `hisi-regbank` stubs,
  so drivers and vendor modules can map them: GPIO, I2C, PL330, Hantro H1,
  RTC, I2S and ISP. `xm510_h264.ko` still reports "Resource busy" at insmod;
  the media blocks are out of scope for now.
