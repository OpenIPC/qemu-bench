# Ambarella S3L

`qemu-system-arm -M s3l`: one Cortex-A9 r4p1 at the addresses of the vendor
kernel's `ambarella-s3l.dtsi`, as in the Amcrest IP4M-1026E / Dahua
IPC-HFW1430SN. The default 512 MiB matches that camera.

The machine is a test bench, not a camera. It boots the vendor kernel to a
shell, gives `ipctool` the same answers the real board does, and runs
`load_ambarella` with the shipped modules. With `flash-file=` it has the NAND of
a Hikvision DS-2CD2423G0-I, so a kernel mounts a camera's own UBI volumes. There
is no video: nothing models the DSP, VIN or the encoder, and blocks with no
model read as zero. With `flash-file=` and no `-kernel` the machine boots a
camera's own `bst` and `bld` from that image; with `-kernel` it loads the kernel
directly and generates the device tree instead.

## Running it

OpenIPC publishes no S3L image, so build the kernel from `OpenIPC/linux`,
branch `ambarella-s3l`, with the firmware's
`br-ext-chip-ambarella/board/s3l/s3l.generic.config`. Any 32-bit ARM gcc does.

```sh
./qemu-src/build/qemu-system-arm -M s3l -nographic -monitor none \
    -kernel arch/arm/boot/zImage -initrd rootfs.cpio.gz \
    -append 'console=ttyS0,115200 rdinit=/init'
```

`-M s3l,sensor=<name>` picks another sensor from the catalogue for I2C0, and
`sensor=none` leaves the bus empty. `sensor-reset=off` is a board with no reset
line, where the sensor answers at once. `-m` sets DRAM: over 120 MiB, for the
DSP, and at most 0xDFFE0000, where the peripherals start.

## NAND

```sh
./qemu-src/build/qemu-system-arm -M s3l,flash-file=flash.bin -nographic \
    -monitor none -kernel arch/arm/boot/zImage -initrd rootfs.cpio.gz \
    -append 'console=ttyS0,115200 rdinit=/init mtdparts=amba_nand:... ubi.mtd=10'
```

Work on a copy: erases and programs are written through to the file, as on the
repo's other flash models. A file that cannot be opened for writing stays as it
is, and the guest's changes last until it exits, so `chmod 444` protects a dump.

- **Image:** 128 MiB of page data, or 132 MiB with 64 spare bytes behind each
  2048 byte page, as `nanddump -o` writes them. Any other size is refused. With
  page data alone the spare area starts blank, and programmed spare bytes last
  only until QEMU exits.
- **Partitions:** none in the generated tree. They are the board's, so they come
  from `mtdparts=amba_nand:...`, with the kernel's `MTD_CMDLINE_PARTS`. The
  camera's own kernel takes them from its device tree.
- **ECC:** BCH is not modelled. Nothing corrupts data here, so the report stays
  clean, and no parity bytes appear in the spare area. An image programmed here
  would not read back on a chip.
- **Bad blocks:** a dump cannot show one, since a block that read as nothing
  looks erased. `bad-blocks=` lists byte offsets, separated by colons. Each gets
  the factory marker in its first spare byte, and fails erase and program. With
  none listed, every block is good. Reading such a block returns what the image
  has there.
- **Bad block table:** the generated tree leaves out `nand-on-flash-bbt`, which
  the real one has. Finding no table, a kernel would write one to the end of the
  chip, and a dump may not have read those blocks. Instead it reads every block's
  marker at boot and writes nothing.
- **ID:** `98 f1 80 15 00`. Only the first two bytes are known from a camera. The
  next two give 2048 + 64 byte pages and 128 KiB blocks through the kernel's
  generic decode. `98 f1 80 15 72` would not do: `nand_ids.c` lists that with 128
  byte spare areas, and the driver then asks for more than the chip has.

## What is modelled

| Block | Address | Model |
|---|---|---|
| VIC, 3 banks | `0xE0003000`, `0xE0010000`, `0xE001C000` | `amb-vic` |
| AHB scratchpad | `0xE001B000` | `amb-vic` |
| Timers | `0xE800B000` | `amb-timer` |
| UART0 | `0xE8005000` | QEMU's 16550, plus a status word at 0x7C |
| I2C0, 1, 2 | `0xE8003000`, `0xE8001000`, `0xE8007000` | `amb-idc` |
| GPIO, 4 banks | `0xE8009000`, `0xE800A000`, `0xE800E000`, `0xE8010000` | `amb-gpio` |
| FIO, FDMA, NAND | `0xE0001000`, `0xE0012000` | `amb-nand`, with `flash-file=` |
| Ethernet | `0xE000E000` | `xm-dwmac`, keeping the FCS: the driver takes 4 off |
| SD0, SD2 | `0xE0002000`, `0xE001F000` | QEMU's SDHCI with capability bit 27 |
| RCT | `0xEC170000` | `amb-rct`: PLL words, free-running timers, chip id at 0x300 |
| L2 cache | `0xF0002000` | QEMU's `l2x0` |

Everything else in the AHB, APB, debug bus, DRAM controller and AXI windows is
an unimplemented device, so `-d unimp` lists what software touched. There is
no pad mux, and GPIO has no interrupts.

## Things that are easy to get wrong

**The scratchpad decides interrupts.** S3L has `VIC_SUPPORT_CPU_OFFLOAD` 2, so
`irq-ambarella.c` reads `AHBSP_PRI_IRQ_C0` (scratchpad + 0x38) and never walks
the banks. All ones there means nothing is pending. S3 uses 0x60 and S2E 0x40,
and either here reads as a real line number and storms.

**The device tree carries the memory split.** The boot loader adds `/iavmem`,
and `ambarella_dt_scan_iavmem()` reads it, so it is the top of Linux's memory
and the bottom of the DSP's. The machine generates it from `-m`.
`/proc/device-tree/iavmem/reg` is also how ipctool reads total RAM on kernels
that have `/proc/device-tree`. Stock Dahua firmware has neither that nor this
node in flash, and ipctool falls back to the boot log there; this machine does
not reproduce that path.

**The device tree has three nodes nothing obviously needs.** A sensor driver
binds to `ambvin0@01` under I2C0: without it the module loads and silently never
probes, so it never releases the sensor's reset. The UART driver
refuses to probe with no default pin state, so `pinctrl-0` is present and
empty. `iav.ko` calls `of_get_child_by_name()` on `/iav` without checking it
exists, so `/iav/vinc0` is needed or `modprobe iav` oopses.

**The sensor starts in reset.** On the IP4M-1026E, XSHUTDN is GPIO 107, active
low, and a pad nobody drives holds it. So does this machine: until GPIO 107 is
an output driven high, I2C0 acknowledges nothing, which is why a probe at boot
finds no sensor. Loading a sensor driver releases it and removing one holds it
again, so the probe that would say which driver to load cannot work without
that driver. The firmware's `load_ambarella` gets round this two ways: `sensor`
in the environment skips the probe, and otherwise it reads
`/iav/vinc0/vinrst-gpios` and releases the line itself. The pin is board data: a
Hikvision DS-2CD2423G0-I has GPIO 39. The flag is the level `iav.ko` resets with,
so 0 means held low, whatever `OF_GPIO_ACTIVE_LOW` suggests. The whole bus goes
quiet, not just the one address, because the sensor is all that is on it here.

**The boot chain uses the hardware differently from the kernel.** With
`flash-file=` and no `-kernel` the machine boots a camera's own loaders: the
chip puts the first NAND page at the high vectors, `0xffff0000`, and the BST
copies itself from those addresses. Both loaders time everything with RCT's
free-running counters, the BST polls `FLASH_INT` and reads whole blocks with one
DMA, and the BLD reads spare areas through DMA descriptors and looks for its
bad-block table in the last blocks. A stock image from a real camera reaches its
main application this way, then reboots: see "The stock app" below. Straps are
the camera's: `SYS_CONFIG` picks NAND, 2K pages and BCH.

**The boot loader needs a chip id.** The BLD reads `RCT + 0x300` to name the
SoC and pick one of the device trees packed in it: `0x4a000000` is an S3Lm
(`hik_chip_name` "S3L11M", e.g. the DS-2CD2423G0-I), `0x40000000` the next part
up, and anything else is "chip id error, default dtb_0" -- a generic tree that
leaves the kernel with the wrong flash, SD and board description. `amb-rct`
returns the S3Lm value, so the BLD runs the real board init and patches the
right tree (`hik_chip_id`, `hik_dram_size`, the NAND partition table from the
`ptb` partition, and the board-identity nodes the kernel reads). Without it the
BLD defaulted everything and the kernel came up on a tree that could not mount
its own volumes.

**PLL rates are read back.** The kernel derives every clock from the RCT words,
so these reset to the rates `/proc/ambarella/clock` shows on the IP4M-1026E. They
are that board's, not the SoC's: the DS-2CD2423G0-I runs its core PLL at 576 MHz.
Note `gclk_ddr` is half its PLL.

**A page moves when FIO DMA is enabled.** The driver sets up FDMA first, so the
whole transfer happens in that write, and `amb-nand` then raises all three
interrupts. The driver waits for all of them. The straps in `SYS_CONFIG` decide
its ECC mode, so `flash-file=` sets them; without them it would pick 1-bit ECC and
a different spare layout.

**The stock app needs a chip this machine does not model.** With the chip id in
place (above) the stock loader boots its own kernel and device tree correctly,
but the application (`davinci`) still reboots within half a minute:
`HIKIO_GET_DEV_HARD_INFO error. errno 14`, then `hwif_getcoreinfo failed` and
`sys app init failed to reboot!`. That ioctl in the vendor `hikio` driver maps
any failure of its `get_dev_info` to `EFAULT`, and `get_dev_info` is gated on a
flag a kernel `__init` sets only after reading a two-byte id from a board
info/security chip that it bit-bangs over GPIO (the same chip behind the early
"Card authentication succeeded" line). This machine models no such chip, so the
read returns nothing, the flag stays clear and the app gives up. It is not the
eMMC: `mmc2` (`sdmmc2`) is the camera's microSD slot, and it finds no card on
the real hardware too. Standing the app up would mean emulating that
Hikvision-specific chip with board-specific responses, and the DSP (ORC
microcode via `/dev/ucode`) cannot run here regardless, so the machine is not
meant to reach the vendor app -- it reaches OpenIPC and ipctool, which is the
point.

**The stock kernel's console is silent without a baud.** Its bootargs are
`console=ttyS0` with no rate, and the vendor `ambarella-uart` driver's console
does not enable against QEMU's plain 16550 then (it sets the rate through
vendor registers this machine does not carry). Userspace still prints, so a
flash boot shows the loaders and busybox but no kernel log. OpenIPC always
passes `console=ttyS0,115200`, so the bench is unaffected; it is only a trap if
you try to read a stock kernel's dmesg here.

**A timer's expiry is an underflow.** Working the count out from the clock at
that moment can come to zero ticks, because the conversions truncate, and the
timer then re-arms at the same instant for ever. The handler just reloads. A
reloading timer repeats no faster than every microsecond, slower than the
silicon: `RELOAD` 0 is the reset value, and 11 ns ticks only spin the host.

## Testing

`qemu/tests/test-ambarella.py` needs only a built QEMU and runs in CI. It drives
the models over qtest the way the kernel does.

`qemu-boot/test-s3l-boot.sh` boots twice. The first boot is the camera as it
is: nothing answers on I2C, driving GPIO 107 high from sysfs makes `ipcinfo -s`
find the sensor, and `load_ambarella` releases it by itself from the device
tree. The second sets `sensor=ov4689` in the environment, so nothing probes and
the real driver releases the line on loading, then checks what the camera in
[ipctool#266](https://github.com/OpenIPC/ipctool/issues/266) reported: `ipctool`
says Ambarella S3L, 512M total, 392M media and an OV4689 at 0x6c on bus 0;
`load_ambarella` loads `iav` and the sensor through `modules.dep`, makes
`/dev/iav` and `/dev/ucode` root-only at 248:0 and 248:1, and reports that no
microcode is present. It takes the kernel, ipctool, and a firmware checkout, so
it does not run in CI.

With `--cc`, it also builds the firmware's `load_ucode` and runs it against the
real `dsp.ko` with made-up microcode: the files land at their offsets, the
version comes back through the driver, and a bad set leaves the window as it
was. That proves the loader and the ABI, not a DSP boot.

What this does not prove: anything about the DSP, the sensor's real timing, or
the Dahua kernel's behaviour. The sensor here is a register file that answers
its chip ID.
