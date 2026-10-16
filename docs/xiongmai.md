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
- **Watchdog:** `xm_wdt.ko`'s init starts the SP805 with a 60 s timeout, and
  only majestic stops or feeds it (`watchdog_disabled_platform()`,
  majestic `src/xm/sdk.c`; OpenIPC/firmware#743). In QEMU majestic Oopses in
  the unmodelled media blocks before it gets there, so the guest resets
  about 60 s after `load_xiongmai`. That is long enough for login and the
  network check, but not for the flash tests.
- **XM550 dump:** booting a full XM550 flash image gets through U-Boot's PLL
  setup but not further yet.

## Model notes

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
