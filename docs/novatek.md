# Novatek NT9856x (`nt98562`, `nt98566`)

Novatek's NT9856x IP-camera SoCs (internal name NA51089): one Cortex-A9
with DDR in the package and Novatek's own peripheral blocks from
`0xF0000000`. One machine file (`hw/arm/novatek.c`) covers the family with a
table entry per part. It boots the OpenIPC `nt98562_lite` / `nt98566_lite`
kernel with `-kernel` to a shell. Flash boot and Ethernet come next; see
[Not modelled](#not-modelled).

## What the SDKs support

| Archive | Chip | What it has |
|---|---|---|
| `NT9856x-…-002.zip` → `na51089_linux_sdk_glibc_v1.01.002` | NA51089 (NT98560/561/562/566) | Kernel 4.19.91 and U-Boot 2019.04 source, board configs `configs/Linux/cfg_56*/nvt-*.dtsi`, loader control flow |
| `nt9856x_linux_sdk_v1.01.007.rar` | NA51089 | Newer loaders (≥ v1.01.003 for the newer in-package DDR), `NT98560_DataSheet_Register_Map.pdf`; no kernel source |
| `NT9856x-…-000/001.zip` | NA51089 | Prebuilt NVTPACK images `FW9856{2,6}A.bin` (562 NOR, 562 NAND, 566 NAND) and loaders |
| `NT9832x_SDK_release_v1.02.0{13,24}` | NA51068 (NT98321/323/621/623) | A dual-A9 DVR part with Faraday IP at `0xFExx0000`; a different family, not modelled |
| `NA51023_BSP` | NT96672 | MIPS 24KEc running uITRON/eCos; not Linux, out of scope |

The OpenIPC kernel (branch `novatek-nt9856x` of `openipc/linux`) is the
v1.01.002 SDK kernel. NA51023 and NA51055 use NA51089's register offsets
(NA51023 at `0xC0000000`), so the blocks here are not 51089-only.

## Machines

| Machine | Chip ID (TOP +0xF0 [31:16]) | DDR | Linux / HDAL split |
|---|---|---|---|
| `nt98562` | `0x7021` | 64 MiB DDR2 | 30 / 34 MiB (`cfg_562_IPCAM1_NOR_EVB`) |
| `nt98566` | `0x7021` | 128 MiB DDR3 | 32 / 96 MiB (`FW98566A.bin`, IPCAM1_EVB) |

The CPU is QEMU's Cortex-A9 with its MIDR set to r4p1. QEMU's default is
r0p0, and Linux's `arm_global_timer` refuses anything before r2p0 (a
comparator erratum), which leaves `sched_clock` on 100 Hz jiffies. The part's
real revision is not documented; r4p1 is the last A9 release.

## Memory map

From `nvt-basic.dtsi`, `nvt-peri.dtsi` and U-Boot's
`arch-nvt-na51089_a32/IOAddress.h`; IRQs are GIC SPI numbers.

| Block | Base | IRQ | Model |
|---|---|---|---|
| DDR | `0x00000000` | | RAM |
| DDR controller | `0xF0000000` | 33 | storage |
| TOP (pinmux, chip ID) | `0xF0010000` | | `nvt-top` |
| CG (clocks, PLLs, resets) | `0xF0020000` | | `nvt-cg` |
| PAD | `0xF0030000` | | storage |
| Novatek timer | `0xF0040000` | 0 | storage (vendor modules only) |
| WDT | `0xF0050000` | 57 | `nvt-wdt` |
| RTC + power button | `0xF0060000` | 56 | `nvt-rtc` |
| GPIO | `0xF0070000` | 24 | `nvt-gpio` |
| UART1 / 2 / 3 | `0xF0290000` / `0xF0300000` / `0xF0310000` | 43 / 44 / 45 | 16550, reg-shift 2 |
| Ethernet (DWC EQoS) + PHY | `0xF02B0000`, `0xF02B3800` | 34 | — |
| SPI NOR / NAND controller | `0xF0400000` | 29 | — |
| SDIO 1 / 2 | `0xF0420000` / `0xF0500000` | 30 / 31 | — |
| USB 2.0 | `0xF0600000` | 27 | — |
| On-chip SRAM | `0xF07C0000` | | — |
| A9 MPCore (SCU, GIC, timers) | `0xFFD00000` | PPI 11 / 13 | `a9mpcore_priv` |
| PL310 | `0xFFE00000` | 96 | `l2x0` |

The rest of `0xF0000000`–`0xF1FFFFFF` reads as zero and is logged under
`-d unimp`. The kernel's `plat-na51089/hardware.h` and U-Boot's
`IOADDR_GIC_REG_BASE` still say the GIC is at `0xF1500000`. That is a
leftover: the device tree and U-Boot's own `SCU_BASE_ADDR` (`na51089_regs.h`)
put the MPCore at `0xFFD00000`, and nothing on the boot path touches
`0xF1500000`.

## The device tree

The OpenIPC uImage has no appended DTB, and on silicon the loader hands
U-Boot one from flash. With `-kernel` the machine builds one from the SDK's
board files, keeping only what is modelled. Things the kernel depends on:

- Root `compatible = "novatek,na51089", "nvt,ca9"`. `mach-nvt-ivot/board-dt.c`
  matches `nvt,ca9`.
- Node names are the vendor's. The clock driver registers each gate under a
  device name such as `f0290000.uart`, and the drivers look their gate up by
  that name.
- `nvt,core_clk` (the CG node) comes before `nvt,periph_clk` in the tree. The
  periph_clk init reads CG through the base the core_clk init maps, and
  `of_clk_init()` runs them in tree order. With the order reversed the kernel
  faults in `get_periph_clk()` before the console is up. `fdt_add_subnode()`
  puts each new node first, so the code creates the two in reverse.
- The `nvt,nvt_top` node has three reg pairs (TOP, PAD, GPIO).
  `nvt_get_chip_id()` reads all six cells.
- The `nvt,nvt_top` node has the board's pinmux children
  (`nvt-top.dtsi`). Without them the pinctrl probe fails with -ENOMEM. The
  driver takes each child's index as its function ID, so the order matters
  too.
- `memory` is the Linux part of DDR, and `hdal-memory/media` is the rest,
  for the vendor modules.

`-dtb` overrides it, for example with a DTB extracted from a vendor
`FW9856xA.bin` (partition 0 of the NVTPACK).

## Clocks and timers

The kernel's clocksource and clockevents are the A9 global timer and TWD, at
`periph_clk = cpu_clk / 8`. The clock driver computes `cpu_clk` from CG:
`SYS_CLK_RATE[1:0] = 1` selects PLL8, whose rate is `12 MHz * ratio /
131072`, with the ratio in bytes at CG `+0x4420/0x4424/0x4428`. QEMU's
a9mpcore runs both timers at a fixed 100 MHz, so `nvt-cg` resets PLL8 to
ratio `0x855555` (800 MHz) instead of the silicon's 960 MHz (`0xA00000`).
With the real ratio, every delay and jiffy would run 20% slow.

## Device notes

- **`nvt-wdt`.** Every CTRL write must carry the key `0x5A96` in [31:16]. The
  count runs at 12 MHz / 1024 up to `(MSB << 12) | 0xFFF`, and expiry either
  interrupts or resets. A write of 1 to MANUAL (`+0x0C`) resets at once
  while the watchdog is enabled; the kernel's restart hook uses it. The
  driver is a module (`na51055_wdt.ko`).
- **`nvt-rtc`.** Writes to TIMER, DAYKEY and PWRALM (the power-on alarm,
  which the driver's wake alarm uses) are shadowed until a CSET commits
  them. PWRALM and the daily ALARM go off on the RTC clock, setting the alarm
  status and, with CTRL's alarm enable, SPI 56. CSET completion is an
  interrupt the driver sleeps on before every access, so it has to fire. A key other than `0xA`, or an
  oscillator config other than `0x9`, makes the driver treat the RTC as
  power-lost and reset it to 1970. The model presents both, with the host's
  time. Reboot goes through the RTC: the vendor reboot notifier arms the
  power-on alarm three seconds ahead and powers off through PWBC, so a
  power-off with the alarm armed is a reset and one without it is a shutdown
  (`poweroff`). Without an RTC node the notifier dereferences a NULL base
  and the kernel panics on `reboot`.
- **`nvt-gpio`.** Eight 32-pin groups (C, P, S, L, D, H, A, DSI at
  `0x00`–`0xE0`), with DATA/DIR/SET/CLR words per group. Only 40 pins can
  interrupt, through a fixed table (`nvt_gpio_irq_table`), all on SPI 24.
  QOM lines `in`/`out` are numbered by pin, at `/machine/nvt-gpio`. An
  `out` line is the pad driver: the latch while the pin is an output, low
  while it is an input or after a reset.
- **UARTs.** 16550 registers at a stride of 4. `nvt_serial.c` adds DMA
  registers at `+0x80`, which read zero here, so the driver stays in PIO.

## Running

```bash
cd qemu-boot && curl -sL \
  https://github.com/OpenIPC/firmware/releases/download/latest/openipc.nt98562-nor-lite.tgz \
  | tar xz && cd ..
bash qemu-boot/run-nt98562.sh               # SOC=nt98566 for the 128 MB part
bash qemu-boot/test-novatek-boot.sh --soc nt98562
python3 qemu/tests/test-novatek.py
```

The release rootfs is SquashFS, and this kernel has no RAM disk driver, so
the boot matrix's `-initrd rootfs.squashfs root=/dev/ram0` does not apply.
`run-nt98562.sh` repacks the rootfs as an initramfs, which needs `mem=`
beyond the device tree's Linux part. `test-novatek-boot.sh` uses a small
initramfs with busybox and an `/init` script instead. That script checks the
device tree, the clocksource, and the pinctrl, GPIO, RTC and watchdog drivers,
then runs `reboot -f` under `-no-reboot`.

## Not modelled

- The SPI NOR/NAND controller at `0xF0400000` (`drivers/mtd/nvt_flash_spi`),
  and so flash boot. The plan is a synthetic boot ROM that, like the
  silicon's loader, puts the FDT at `0x100000` and U-Boot at `0x03900000`
  and fills the `LD_NVT` shared-memory block at `0x200000`. The real 32 KiB
  loader is skipped: it trains DDR from its ConfigRAM table with closed
  libraries (`Reset_560.a`).
- Ethernet: a DWC EQoS (GMAC 4.x) MAC. QEMU has no model of it, and
  `xm-dwmac` is GMAC 3.x with a different descriptor format.
- SDIO, USB, the Novatek timer, PWM, I2C, crypto/OTP, and the HDAL media
  blocks.
