# Allwinner V831 / V833 (sun8iw19p1)

`hw/arm/allwinner-v83x.c`, machines `v831` (64 MiB DDR2 in the package)
and `v833` (128 MiB DDR3). One Cortex-A7 with TrustZone, a GIC-400, and
the "ncat" peripheral map the H6 introduced. A camera's SPI NOR dump boots
as on silicon: boot0, OP-TEE, U-Boot, Linux 4.9 and the vendor userspace.

```bash
bash qemu/setup.sh
./qemu-src/build/qemu-system-arm -M v831,flash-file=full.bin \
    -drive if=sd,format=raw,file=sd.img -nographic -serial mon:stdio
bash qemu-boot/test-v831-boot.sh --flash full.bin    # milestones + shell
```

Flash erases and programs go back to `flash-file` (the firmware saves its
settings there): boot a copy. `flash-jedec=` sets the ID the NOR answers
with (default GD25Q64, 0xC84017).

## Memory map

From the vendor U-Boot's `arch-sunxi/cpu_ncat.h` and the board DTB in the
boot package.

| Address | Block | Model |
|---------|-------|-------|
| `0x00000000` | BROM | stub: `ldr pc` to boot0 |
| `0x00010000` | SRAM A1/C/A2 (320 KiB) | RAM |
| `0x00100000` | SRAM A2 (boot0's) | RAM |
| `0x03000000` | SRAM controller | storage |
| `0x03001000` | CCU | `aw-ccu` |
| `0x03002000` | DMA, 16 channels | `aw-dma` |
| `0x03006000` | SID / chip ID | storage |
| `0x03009000` | timers | upstream `allwinner-A10-timer` |
| `0x030090A0` | watchdog | `aw-wdt` |
| `0x0300B000` | PIO | `aw-pio`: inputs read high unless pulled down |
| `0x01C0E000` | video engine (cedar VE) | `aw-ve`: H.264 encoder |
| `0x01480000` | G2D (2D engine, RCQ) | `aw-g2d`: scale / convert (IRQ 21) |
| `0x02100000` | ISP521, VIPP scalers | storage |
| `0x06600000` | CSI top / parsers, MIPI | storage |
| `0x06609000` | CSI DMA ("vinc") 0..3 | `aw-vinc`: test-pattern frames |
| `0x030F0000` | IOMMU (VE, CSI, ISP, G2D masters) | `aw-iommu` |
| `0x03021000` | GIC-400 (192 lines) | `arm_gic`, security extensions |
| `0x04002000` | DRAM COM / CTL / PHY | storage, `aw-dramc` at `0x04003000` |
| `0x04020000` | SMHC0 (SD), SMHC1 (SDIO) | upstream `allwinner-sdhost-sun50i-a64` |
| `0x05000000` | UART0..3 | 16550 + DW extras (USR) |
| `0x05090000` | DAUDIO0/1 (I2S) | `aw-audio`: capture of silence |
| `0x05096000` | audio codec | `aw-audio`: ADC capture of silence |
| `0x05002000` | TWI0..3 | upstream `allwinner.i2c-sun6i` + `aw-twi` at +0x200 |
| `0x05010000` | SPI0..2 | `aw-spi`; the NOR is `ak-spinor` on SPI0 CS0 |
| `0x05020000` | EMAC ("sunxi-gmac") + internal PHY | upstream `allwinner-sun8i-emac` |
| `0x07000000` | RTC, PRCM | storage |
| `0x07022000` | R_PIO | `aw-pio` |
| `0x07081400` | R_TWI | upstream `allwinner.i2c-sun6i` |
| `0x40000000` | DRAM, repeated across 2 GiB | RAM + aliases |

Interrupts: UART0 49, SPI0 54, TWI0 41, SMHC0 46, EMAC 27, DMA 10,
timers 60/61,
watchdog 14; the arch timer on PPIs 13/14/11/10.

## Boot chain

The flash starts with eGON.BT0 (boot0, length at +0x10, DRAM parameters
in its private header), then a `sunxi-package` at `0x10000` with OP-TEE
(run at `0x41980000`), U-Boot (`0x4A000000`) and the DTB, then a GPT at
`0x5C000` whose partitions are counted from there (boot, rootfs, ...).

1. The stub ROM copies boot0 to `0x20000` and enters it in secure SVC.
2. boot0 sets the PLLs (`aw-ccu` reports LOCK), probes for a PMU on
   R_TWI (none), trains DRAM (`aw-dramc` finishes every PIR step at
   once), reads the package over SPI0 and enters OP-TEE.
3. OP-TEE configures the GIC and drops to U-Boot in the normal world.
4. U-Boot reads its environment and the GPT over SPI with DMA, reads the
   SD card, loads the uImage from "boot" and starts Linux.
5. Linux mounts the SquashFS root from `mtdblock2`.

## Things the firmware depends on

- **192 GIC lines.** OP-TEE moves the interrupts to group 1
  (non-secure) only on a GIC with more than 179 lines; with fewer it leaves
  them secure and the kernel, in the normal world, never gets one. It
  hangs in WFI right after the timer is set up.
- **DRAM repeats across its window.** U-Boot is linked for `0x4A000000`
  whatever the DRAM size.
- **SPI.** XCH is set before the TX FIFO is filled, MBC counts down to 0
  (boot0 checks), and U-Boot reads 128 KiB in one exchange through DMA,
  so the exchange stalls on an empty TX FIFO and on a full 64-byte RX FIFO,
  as the clock does on silicon.
- **DMA.** Only the IO-mode (FIFO register) side of a channel waits for
  its DRQ; DRAM is port 1 here, not 0. SPI0 is DRQ 22 as source (RX) and
  as destination (TX), so the model has separate source and destination
  DRQ inputs. A rising DRQ runs the channels from a bottom half, because
  it rises inside the SPI's own register access.
- **I2C.** Upstream's TWI needed four fixes (`qemu/patches/0005`..`0008`).
  SRST is a self-clearing pulse. A NACK raises INT_FLAG. Master mode needs
  no BUS_EN, which the vendor kernel never sets. The interrupt line is
  INT_FLAG && INT_EN, because the vendor handler turns INT_EN off first.
- **SD.** The IDMAC soft reset clears itself (`qemu/patches/0004`).
- **Ethernet.** The vendor `sunxi-gmac` is the H3 EMAC register for
  register. Its driver scans all 32 MDIO addresses and keeps the last
  PHY it finds, so an absent PHY must read 0xFFFF and the address and
  register fields must be five bits wide (`qemu/patches/0009`).
  Otherwise it attaches to a phantom PHY at 31, and `ifconfig eth0 up`
  fails with EINVAL.
- The TWI blocks also have a packet engine at +0x200 (DT `twi_drv_used`),
  modelled by `aw-twi`. The kernel of the firmware tested drives the legacy
  registers, so it is untested against real firmware.

## Console on Xiaoyi (YI) firmware

The tested dump is a YI camera (U-Boot strings `xiaoyi-*`). Its U-Boot
leaves the kernel console on `ttyS0` only if the environment has `print=1`
or the SD card holds `/debug_tty` or `/Factory`. Otherwise it passes
`console=ttyS2`, a UART the DT disables, and the kernel stays silent.
That U-Boot has no `setenv`, so boot with a card that carries
`/debug_tty`; `qemu-boot/test-v831-boot.sh` builds one. With the card,
the vendor userspace runs to a root shell and the YI application.

When the kernel is silent, its printk ring can be read from the guest:
`-gdb tcp::1234`, then `dump binary memory lowmem.bin 0xc0000000 0xc4000000`
and look for the records (`log_buf` is near `0xc039e000` in this kernel).

## Ethernet on a Wi-Fi-only camera

The tested camera has no Ethernet jack, but the V831's EMAC and its DT
node are there, and the kernel registers `eth0`. With `-nic user` the
vendor driver brings the link up. A DHCP lease, ping and an HTTP
download (checksum compared) work from the shell:

```
ifconfig eth0 up; udhcpc -i eth0 -n -q
ifconfig eth0 10.0.2.15 netmask 255.255.255.0   # the lease is not applied
ping -c 3 10.0.2.2
```

A bridged tap (`-nic tap,...`) works too. Before patch 0011 it did not:
traffic from the host before `ifconfig eth0 up` (IPv6, mDNS, STP) was
queued while the receiver was off, and the upstream EMAC model never
flushed that queue on the register Linux enables reception with, so the
camera's ARP requests went out and nothing ever came back in.

The YI application itself only configures `wlan0`. With the video input
modelled (below) the media process stays up; the YI supervisor
(`watch_process`) would otherwise reboot the camera when it dies.

## Video input

The vendor media process `rmm` (Allwinner eyesee-MPP, ISP521, CedarC)
starts, stays up, and with a configured camera opens its H.264 encoder.

- **VE identity.** CedarC's VE layer keys a table compiled into `rmm`
  on 0xF0 >> 16 (0x1633 .. 0x1718) or on (0xE0 << 32) | 0xE4 for newer
  parts, and its H.264 encoder takes the LBC-compressed input the video
  input writes only from an IC (0xE4) above 0x2100F. Three entries
  qualify; `aw-ve` answers 0xF0 = 0, 0xE0 = 0x21110, 0xE4 = 0x21210, the
  300 MHz one. CedarC then sets the VE to 300 MHz and the encoder opens
  on LBC input. 0x1707 (an older guess) passes the VE layer but the
  encoder refuses it. The ID is chosen from the firmware's table, not
  read from silicon.
- **Sensor.** The repo's GC2053 (`hisi-gc2053`, ID 0x2053 at 0xF0/0xF1)
  sits at 0x37 on TWI1. The VIN driver registers its video nodes and the
  ISP library finds its `gc2053_mipi_1936_1096_20` tuning.
- **Frames.** `aw-vinc` paints the OpenIPC logo (still, white on navy)
  with a round dot sliding beneath it into each queued buffer at 20 fps
  and raises VSYNC and frame done. The logo stays still because P frames
  send changed macroblocks raw; the dot gives the motion detector
  something to see. The
  scene is drawn in sensor coordinates and timed by the clock, so every
  channel shows the same picture. The firmware uses the single-address
  mode (0x20/0x28); the FIFO mode (`CONFIG_BUF_AUTO_UPDATE`) is modelled
  too. Allwinner's LBC format is not public, so in LBC mode (EN bit 8: the
  encoder inputs) the channel writes a private stand-in, `aw-qlbc.h`:
  the picture as 4x4-block averages, which `aw-ve` reads. FBC frames
  (bit 3) are signalled but not painted.
- **IOMMU.** Buffer addresses are IOMMU device addresses; `aw-iommu`
  walks the vendor page tables (4096 x 1 MiB directory, 256 x 4 KiB
  tables, PDE valid = 1, PTE valid bit 1) for the CSI master.
- **Reset button.** The PIO used to be plain storage, so every input read
  0, and on this camera that is the reset button held down: the firmware
  factory-reset itself on every boot. `aw-pio` reads inputs high unless
  pulled down.

### Unconfigured vs. configured camera

As dumped, the camera was never paired: it starts in pairing mode (QR
scanner on the 640x360 VIPP 2, sound-wave pairing) and does not start its
encoders. `qemu-boot/yi-v831-prepare.sh in.bin out.bin` makes a copy that
behaves as on Wi-Fi:

- the system config (`conf`, mtdblock7 = flash 0x7F0000) gets an SSID
  (+0x1C) and passphrase (+0x5C), and its start_with_reset word (+0x18)
  is cleared;
- the `home` SquashFS gets `eth0` renamed to `wlan0` at boot and a
  `wpa_cli` stand-in that reports an association.

With `-nic user` the camera then takes a DHCP lease, syncs its clock,
says "wifi connected goto venc and aenc" and runs both H.264 streams
(1920x1080 and 640x360, High profile, CABAC, 20 fps) and AAC audio.

### The H.264 encoder

CedarC writes the parameter sets and every slice header itself, bit by
bit through the encoder's bitstream writer (0xB20 / 0xB18 command 1,
ready bit 9 of 0xB1C), into a ring buffer given in 256-byte units
(0xB80 / 0xB84). The hardware appends the macroblock layer on command 8,
then signals done. `aw-ve` codes the input picture (the LBC buffer at
0xA78, `aw-qlbc.h`) as CABAC:

- I slices: every macroblock I_16x16, DC prediction, luma and chroma DC
  coefficients, so each 4x4 block decodes flat at its average (within the
  quantiser step). The decoder's reconstruction is tracked exactly.
- P slices: macroblocks whose input did not change since the last frame
  are skipped, the others are I_PCM.

The reference is kept per stream (per bitstream buffer). The slice QP comes from 0xB08, the slice type and `cabac_init_idc`
from 0xB04 (bits 4 and 11:10), and the size from 0xA00. VLE_LENGTH
(0xB90) is the write position, which the driver moves by setting
VLE_OFFSET; it takes a frame's size as LENGTH - OFFSET.

`-global aw-ve.dump=DIR` writes each frame as the driver takes it to
`DIR/avc-<buffer>.h264`. With the stream's SPS and PPS in front, the
YI streams decode with libavcodec with no errors and no concealment,
and show the logo picture. The coder decodes pixel-exact against its
tracked reconstruction over QP 0..51 (offline harness with libavcodec).

### The 2D engine

The YI application has G2D turn every 640x360 NV21 frame of VIPP 2 into a
320x240 grey (Y8) one for its motion and person detection. The driver
(`g2d_rcq`) lays the mixer registers out in memory and queues 16-byte
headers (block address, length, dirty, register offset) at 0x2C / 0x34,
then starts it with 0x28 bit 0 and waits for the task-end interrupt.
`aw-g2d` loads the blocks, scales the V0 layer into the write-back buffer
(nearest sample; YUV 4:2:0 / 4:2:2, Y8 and packed RGB) and raises it.
Without it every task failed (`G2D_CMD_MIXER_TASK failure!`); with it the
application's motion detection fires on the moving dot
(`motion_detect=3`, `send start motion detect`).

### Audio and the recorder

`aw-audio` feeds the codec ADC and DAUDIO FIFOs silence at the programmed
rate, so ALSA capture runs and the AAC encoder produces frames. The
recorder (`mp4record`) then gets the video and audio configurations and
waits on an SD card. Use a card of 4 GB or more (SDHC): 2 GB is "too
small" for it, and `qemu/patches/0010` lets SDHC cards initialise. It
records on events (motion, people) only; whether it then writes files has
not been checked.

## Not modelled

USB (the atbm Wi-Fi is USB), the real LBC/FBC formats (LBC is a private
stand-in), JPEG, ISP statistics (the 3A loop times out waiting for
events), playback, the NNA (person detection), G2D blending/rotation, and
the crypto engine (CE).
