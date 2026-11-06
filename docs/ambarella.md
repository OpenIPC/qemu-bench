# Ambarella S3L

`qemu-system-arm -M s3l`: one Cortex-A9 r4p1 at the addresses of the vendor
kernel's `ambarella-s3l.dtsi`, as in the Amcrest IP4M-1026E / Dahua
IPC-HFW1430SN. The default 512 MiB matches that camera.

The machine is a test bench, not a camera. It boots the vendor kernel to a
shell, gives `ipctool` the same answers the real board does, and runs
`load_ambarella` with the shipped modules. There is no video: nothing models
the DSP, VIN or the encoder, and blocks with no model read as zero.

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

## What is modelled

| Block | Address | Model |
|---|---|---|
| VIC, 3 banks | `0xE0003000`, `0xE0010000`, `0xE001C000` | `amb-vic` |
| AHB scratchpad | `0xE001B000` | `amb-vic` |
| Timers | `0xE800B000` | `amb-timer` |
| UART0 | `0xE8005000` | QEMU's 16550, plus a status word at 0x7C |
| I2C0, 1, 2 | `0xE8003000`, `0xE8001000`, `0xE8007000` | `amb-idc` |
| GPIO, 4 banks | `0xE8009000`, `0xE800A000`, `0xE800E000`, `0xE8010000` | `amb-gpio` |
| RCT | `0xEC170000` | `amb-rct` |
| L2 cache | `0xF0002000` | QEMU's `l2x0` |

Everything else in the AHB, APB, debug bus, DRAM controller and AXI windows is
an unimplemented device, so `-d unimp` lists what software touched. There is
no pad mux, Ethernet or flash, and GPIO has no interrupts.

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

**PLL rates are read back.** The kernel derives every clock from the RCT words,
so these reset to the rates `/proc/ambarella/clock` shows on the camera. Note
`gclk_ddr` is half its PLL.

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
