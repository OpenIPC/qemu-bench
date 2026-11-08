# SigmaStar / MStar ARMv7 (`ssr621q`, `ssd20x`, `ssc375`, `msc313e`, `ssc029`, `ssc034`, infinity6 dies)

SigmaStar's (formerly MStar's) Cortex-A7 SoCs share one memory map whatever
the family: DRAM (the MIU0 window) at `0x20000000`, the SPI NOR read window at
`0x14000000`, the A7 private region with the GIC at `0x16000000`, IMI SRAM at
`0xA0000000` and every peripheral behind the RIU at `0x1F000000`. One machine
file (`hw/arm/sigmastar.c`) covers them with a table entry per SoC: the
infinity2m dies, Ifado, infinity3, Iford and the infinity6 dies.

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
| `msc316dc`, `msc316dm` | infinity3 | 1× Cortex-A7 | 128 MiB | — | `0xC2` | IPL → OpenIPC U-Boot: flash, environment, network, SD |
| `ssc029` | Iford | 2× Cortex-A32 (`max` here)² | 128 MiB | `0`³ | `0xF0`³ | IPL → U-Boot → kernel → shell, SDK `preview` to RTSP H.264, Ethernet, SD, USB; no AE |
| `ssc029a` | Iford | 2× Cortex-A32 (`max` here)² | 512 MiB | `0`³ | `0xF0`³ | SDK SPI NAND: IPL → OP-TEE → U-Boot → kernel → shell |
| `ssc029b` | Iford | 2× Cortex-A32 (`max` here)² | 256 MiB | none: `bond=` needed³ | `0xF0`³ | IPL → U-Boot with `bond=0x10` |
| `ssc034a` | Ifliegen | 2× Cortex-A7 | 128 MiB | `0x09`⁵ | `0xF0`⁵ | SDK IPL → TF-A → U-Boot → SDK kernel → shell from NOR and SPI NAND, 2 CPUs, Ethernet, SD, SDK `venc_demo` H.264/H.265 |
| `ssc034b` | Ifliegen | 2× Cortex-A7 | 64 MiB | `0x00`⁵ | `0xF0`⁵ | the same, from its QFN88 IPL |

| `ssc325` | infinity6 | 1× Cortex-A7 | 128 MiB⁴ | — | `0xEF` | IPL → vendor U-Boot → OpenIPC login, NOR and SPI NAND |
| `ssc325de` | infinity6 | 1× Cortex-A7 | 256 MiB⁴ | — | `0xEF` | IPL → vendor U-Boot → OpenIPC login from SPI NAND |
| `ssc327de` | infinity6 | 1× Cortex-A7 | 128 MiB | — | `0xEF` | a Raysharp camera's SPI NAND dump to login |
| `ssc333`, `ssc335`, `ssc337` | infinity6b0 | 1× Cortex-A7 | 64 MiB | — | `0xF2` | IPL → vendor U-Boot; `ssc337` to OpenIPC login |
| `ssc335de`, `ssc337de` | infinity6b0 | 1× Cortex-A7 | 128 MiB | — | `0xF2` | IPL → vendor U-Boot |
| `ssc30kd`, `ssc30kq` | infinity6e | 2× Cortex-A7 | 128 MiB⁴ | `0x10` | `0xF1` | IPL → vendor U-Boot |
| `ssc338q` | infinity6e | 2× Cortex-A7 | 256 MiB⁴ | `0x10` | `0xF1` | IPL → vendor U-Boot → OpenIPC login |
| `ssc377`, `ssc377d`, `ssc377de`, `ssc378de` | infinity6c | 1× Cortex-A35 (`max` here) | 128 MiB⁴ | — | `0xF9` | IPL → vendor U-Boot; `ssc377` to OpenIPC login |
| `ssc377qe`, `ssc378qe` | infinity6c | 1× Cortex-A35 (`max` here) | 256 MiB⁴ | — | `0xF9` | IPL → vendor U-Boot |

⁴ A starting value, not a known size: see [Part defaults](#part-defaults).
The machine warns unless `-m` chooses.

¹ Not checked against silicon: the boot chain to userspace does not look
at it. The SoC is named by the firmware itself (`CHIP=SSC375` in the Dahua
rootfs; kernel compatible `sstar,ifado`).

² QEMU has no Cortex-A32: the cores are QEMU's ARMv8 `max` in AArch32,
with an A32 MIDR (`0x410FD011`), as the SDK's OP-TEE uses `LDAEX`/`STLEX`,
which an A7 lacks. CPU1 wakes through smpctrl (bank `0x1020`): the
kernel writes its entry to regs `0x13`/`0x14` and sets bit 2 (wake) and bit 3
(Non-secure) of reg `0x15`. Nothing on the flash reads those, so what parks the
core on silicon is unseen; QEMU starts it in place of that firmware and makes
its redistributor's lines Non-secure, as the IPL does for CPU0.

³ Not from the SDK. The bond is the IPL's DRAM profile, 0-5. The kernel shows
the chip ID as `soc_id` (bank `0x1E00` reg 0) but `mdrv_device_id.h` has no
Iford entry, so `0xF0` remains a placeholder.

⁵ The bond is the IPL's DRAM profile, which it prints: the QFN88 IPL takes
0, 1, 4 and 5, the QFN128 one 9, 10, 13 and 14, and bit 2 changes neither.
Each default is the one profile that matches the part's DRAM size in the SDK
(`ssc034a.128.qfn128`, `ssc034b.64.qfn88`). Nothing in the boot chain reads the
chip ID, so `0xF0` is a placeholder.

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

## Iford (`ssc029`)

Iford is a later generation than the other dies here. Linux is 5.10 and
U-Boot 2021.10, and a good deal has moved. The SDK ships prebuilt NOR images
rather than a flash dump; `qemu-boot/sstar-iford-image.py` lays one out as its
TFTP install scripts would:

```
qemu-boot/sstar-iford-image.py \
    Image/ipc_iford.nor.uclibc-9.1.0-squashfs.ssz029c.128.qfn128_ddr3_demo_aov_defconfig/images \
    iford.bin
qemu-system-arm -M ssc029,flash-file=iford.bin -serial mon:stdio -nic user
```

It boots to a shell on the console. `demo.sh` loads the MI modules with
`printk` silenced, so the kernel goes quiet after
`clk: Not disabling unused clocks`; the prompt comes when it is done. Its
`scsi_mod`, `usb-storage` and `sd_mod` and a read-only `/misc` are the image's
own errors. `sstar_emac.ko` is not in `demo.sh`: `insmod` it, and use a static
address: this kernel has no `AF_PACKET`, so `udhcpc` cannot open its socket.

What differs from the other dies:

| | Iford | Older dies |
|---|---|---|
| Interrupt controller | GICv3, redistributors at `0x16040000`; the IPL itself uses `ICC_MSRE` | GIC-400 |
| Interrupts | `sstar-intc`, 128 `MS_IRQ` from SPI 32, `MS_FIQ` from SPI 160. The driver runs the old registers past their ends: masks `0x54`-`0x5B`, polarity `0x58`-`0x5F`, raw `0x5C`-`0x63`, so polarity is ignored | 64 `MS_IRQ` | `MS_FIQ` from 96 |
| Boot flash | FSP banks `0x100D`/`0x100E` | `0x0016`/`0x0017` |
| Mask ROM | leaves the core in Monitor mode⁴ | Supervisor |
| Secure monitor | `SMF` at `0x20000400`: A32 `CPUACTLR`/`CPUECTLR` (RAZ/WI here) and the bank `0x1121` handshake | Ifado has the handshake |
| Unpacking | `sstar-xzdec`, below | IPL_CUST in software |
| URDMA | UART0 `0x1F220E00`, UART1 `0x1F221C00`, FUART `0x1F220600`; SPIs 67, 92, 80 | Ifado: `0x1F220C00`, `0x1F220E00` |
| I2C | `0x1F222800`, `0x1F222A00`, `0x1F007C00` | `0x1F223000`.. |
| EMAC | X32 view `0x1F304200`, internal PHY `0x1F2A3000` (bank `0x1518`), SPI 51 | `0x1F343C00`, bank `0x0031`, SPI 58 |

⁴ An inference: the IPL sets `SCR.NS`, touches `ICC_HSRE` and clears it again,
which only works in Monitor mode. There is no dump of this ROM.

The boot chain's status bits, in `sstar-riu`: the IPL's DRAM test (bank
`0x1668` reg `0x50` bit 0 starts it, reg `0x59` [2:0] reads 4), the MIU MMU's
reset (bank `0x1659` reg 0 bit 4, done in [13:12]), the FCIEs' reset
(banks `0x1413` and `0x0042`, reg `0x3F`). DRAM size is bank `0x1659` reg
`0x0D`, log2 of MiB, which `mi_sys` checks against `LX_MEM`.

**XZ decoder.** IPL_CUST and U-Boot unpack through a hardware decoder
(banks `0x11D7`-`0x11DC`), fed by BDMA destination port 6. The file format is
the vendor's `sz` tool's (`sstar_sz.sh`, `szsplit`, `szrestore`):

- `sstarXZ`, the block count, the chunk size `0x20`, each block's stored and
  unpacked size as XZ VLIs, padded to 16 bytes;
- then the blocks round-robin, a chunk of each in turn, for the four decoder
  cores; a trailing `SZ`;
- each block is an XZ block (header, LZMA2, padding, CRC32).

**The LZMA is not standard**: a literal is eight direct bits of the range
coder, with no context (`szdec`'s `lzma_literal`). No stock decoder reads
these files. Matches and lengths are as usual; `xz` fails a few bytes in.

**The media path.** The SDK's `amigos/preview` runs to RTSP at
`rtsp://<ip>/video0` with its stock `pad0-venc-file.json`, VIF to ISP in frame
mode (the uClibc binary from
`project/release/chip/iford/ipc-rtos/common/uclibc/9.1.0/release/bin/amigos/preview`;
keep its stdin open, or its menu loop fills the ramfs).

That `preview` is the RTOS build and links no 3A, so nothing runs AE beside it.
`qemu-boot/sstar-iford-3a-host.c` is a small dynamic host for the vendor's own
`libcus3a` and `libispalgo`: `MI_SYS_Init`, `CUS3A_Init`,
`MI_ISP_EnableUserspace3A`. Run beside `preview`, the unmodified algorithms
converge, and follow `qom-set /machine/isp light|light-r|light-b`.

| Stage | State |
|---|---|
| OS04D10 on i2c1, `0x3C`: ID, modes, stream on | modelled (`os04d10`, in the catalogue) |
| MIU MMU, bus masters and the CPU's window at `0x60000000` | modelled, from `hal_miu.c` |
| CMDQ, 8 engines, MI's fences | modelled |
| Sensor to Bayer | modelled in `sstar-iford-isp`: scene × light × the sensor's own rows and gain |
| VIF WDMA, banks `0x1358`, `0x130C`, SPI 56 | modelled: FIFO, events, the frame in DRAM. The 10-bit packing is a guess, as nothing reads it back |
| ISP RDMA | **not modelled**: the ISP works from the sensor's frame directly, in either bind mode |
| 3A statistics, bank `0x131B`, into `ISP0_STATS` | modelled from the Bayer data: AE, AWB, histograms |
| WBG, bank `0x1312` | modelled, driven by the vendor's AWB |
| Demosaic | 2×2 quads, half size |
| Gamma, CCM | **stub**: a fixed 1/2.2 and BT.601. The IQ file's are not read |
| Black level, LSC, NR, sharpening | not modelled |
| SCL output, bank `0x1253` | the ISP's picture, written by nearest neighbour. Crop and filters not modelled |
| SCL to VENC ring (`RING_UNIFIED_DMA`) | not modelled: see below |
| VENC at its firmware interface | the interface is real; no firmware runs, and H.264 comes from `sstar-venc` (libavcodec, or the writer below) |
| Suspend to RAM | modelled: `sstar-rtcpwc`, then the stock IPL's resume path |
| IPU | **stub**: the supply ack and `0x19000000`-`0x1938FFFF` as plain RAM. Nothing computes; `ipu_lfw.bin` in the image is empty |
| H.265 without libavcodec | not modelled |

`prog_aov_aov_demo` gets through `MI_IPU` and `DET` init on that stub, which
proves only that they do not touch the block, and starts its pipeline. It stops
next at the encoder: its VENC takes `RING_UNIFIED_DMA` input, the driver sets
the channel up but never writes the start register, so `__AovGetVencStream`
times out. `MI_IPU_Invoke` has not been reached.

**Suspend.** `sram_suspend_imi` ends by clearing `PWR_EN` in the RTCPWC (bank
`0x0034` reg `0x0F`) and then runs off into its literal pool, as the power has
gone. Here that suspends the machine; the RTC alarm resets it with DRAM and the
RTC domain kept, and the PMPOR flag (bank `0x0600` reg 1 bit 3). `echo +4 >
/sys/class/rtc/rtc0/wakealarm; echo mem > /sys/power/state` returns to the
shell through the unmodified IPL, with CPU1 back. It needs the default host RTC
clock: `-rtc clock=vm` stops while suspended, and the alarm never comes.

Without libavcodec `sstar-venc` has a Baseline H.264 writer, which codes the
real source buffer: Intra16x16 with no residual where a prediction is exact,
I_PCM elsewhere, P_Skip for what stands still. No transform or motion: a lossless
picture, not a bitrate. Frames that change everywhere take several P frames.

The buffers are contiguous only as MIU addresses: a bus master must translate
every page. `sstar-fcie` gained ADMA and `BUSY_END`, which Iford's driver uses
and infinity3's does not; SD reads and writes `mmcblk0`. USB is one EHCI at bank
`0x1424`, SPI 63; the image has no `usb-storage.ko`, `scsi_mod.ko` or `sd_mod.ko`,
so a USB disk cannot work, while a `usb-kbd` enumerates and binds `usbhid`.

## Ifliegen (`ssc034a`, `ssc034b`)

Ifliegen is Iford's register map on two Cortex-A7s and a GIC-400, with its
interrupt lines renumbered (vendor Ifliegen SDK ILSSV01C12: U-Boot 2021.10,
Linux 4.9.84, TF-A SP_MIN 2.5 as the secure monitor). The SDK ships no images
for it, so the image is built from its sources and board files:

```
# U-Boot: ifliegen_ssc034a_s01a_defconfig -> u-boot.xz.img.bin
# kernel: ifliegen_ssc034a_s01a_defconfig, uImage.xz (it must fit 0x310000)
qemu-boot/sstar-ifliegen-image.py project/board/ifliegen/boot \
    u-boot.xz.img.bin uImage.xz rootfs.sqfs ifliegen.bin
qemu-system-arm -M ssc034a,flash-file=ifliegen.bin -serial mon:stdio -nic user

# SPI NAND: ..._spinand_defconfig for both -> u-boot_spinand.xz.img.bin and
# uImage.xz; the squashfs in a UBI image as a static volume "rootfs"
ubinize -o rootfs.ubi -p 0x20000 -m 2048 -s 2048 -O 2048 ubi.cfg
qemu-boot/sstar-ifliegen-image.py project/board/ifliegen/boot \
    u-boot_spinand.xz.img.bin uImage.xz rootfs.ubi ifliegen.nand --nand
qemu-system-arm -M ssc034a,flash-file=ifliegen.nand -serial mon:stdio
```

`--package qfn88` and `-M ssc034b` give the 64 MiB part. Both boot the SDK's
IPL (DDR init and BIST), TF-A, IPL_CUST and U-Boot, then the kernel to a shell
with both CPUs, I2C, the RTC and Ethernet: from NOR with a squashfs root, from
SPI NAND (KERNEL, then the root through `ubiblock1_0`, UBI on `ubia` too).

**SPI NAND.** The NAND IPL has no package variants. Its CIS is what the SDK's
`snigenerator` (whose C source the SDK carries; the script does the same, and
matches it byte for byte) leaves: BL0PBA 10 and BL1PBA 25 in the SNIB and every
SNIF, their CRCs. The SNIF must be a listed part's, as U-Boot's probe writes on
a board: the IPL takes its status-poll timeout from the record, and the generic
one and some parts (GD5F1GQ4U, record 17) leave it zero, so the first poll
times out (`load header fail`). Record 57, a W25N01GV, is the default.

**SD.** The FCIE at bank `0x1413` (`-drive if=sd`) and the SDIO slot's at
`0x1416` (`if=sd,index=1`, INT_IRQ_SDIO, SPI 86). Card detect: `PAD_PM_SD_CDZ`
at bank `0x103E` reg `0x16` bit 0 and `PAD_SD1_IO6` at reg `0x06` bit 0
(`hal_gpio_common.c`), low with a card in. The driver (`kdrv_sdmmc.ko`, a
module in the SDK config) writes with `EN_AUTO_TRIG`: `SD_CTL` bit 12
(`R_CMD_END_AUTO_TRIG_EN`) sends a write's data after its command in the same
job, which `sstar-fcie` now does. Reads and writes go through to the image.

**Encoder.** Not Wave5: the DTS `venc` "vpu-bit" `0x1F34D000` lies in Iford's
RISC firmware window at `0x1F34C800` (mi_venc.ko `DRV_VENC_TOP_RiscHostFwInit`),
`sstar-venc` here, on INT_FIQ_HST_2 (SPI 174), with a 32-bit wrapper page at
`0x1F34E000` as storage (+0x404 [28:27] = 2, buffers in MIU, which
`DRV_VENC_DEV_ImiShareCheck` requires). Its firmware ABI is not Iford's, and
`sstar-venc` (`cmd-ring`) stands in for the firmware:

- **Commands.** The first word written to ctrl+0x04 is the ring's MIU
  address; later writes there are the host's index into its six 0x1B40-byte
  entries, and ctrl+0x08 is the firmware's index. ctrl+0x18 bit 0 is a
  synchronous command, which it clears; ctrl+0x14 reads 2, running.
- **An entry.** Header +0x00 the stream, +0x01 its task, +0x10 the command
  (8 encode), +0x4C/+0x58 luma stride and MIU address, +0x64/+0x70 chroma's,
  +0xD0 the bitstream ring's control block (+0x00 base, +0x08 size, +0x40
  the firmware's write offset). Context +0x19C the codec (1 H.264, 2
  H.265), +0xFC4/+0xFC6 width and height.
- **Results.** Blocks of 0x180 after the ring (ring + 7 × 0x1B40), two per
  stream: +0x00 0 (1-3 make MI drop the picture), +0x0B nonzero, +0x10 the
  stream's MIU address, +0x18 its size in bits, +0x160 status 2. Then
  message 9 in ctrl+0x10 [23:16] on the interrupt. Messages 1 and 3 (from
  5) are "output buffer full" and are not sent.
- **Pictures.** Coded by the model's own encoders (H.265 `hevc-enc`, H.264
  a baseline SPS/PPS and `hantro-h264` I slices), not the firmware's.

The SDK's own `venc_demo` sample (built from `sdk/verify/sample_code` with
GCC 10.3 and the toolchain's glibc, run with the prebuilt glibc-9.1.0
`mi_common`, `mi_sys`, `mi_rgn`, `mi_venc` and their libraries) writes
`venc_demo_case_h265.es` and `venc_demo_case_h264.es` from its 1920×1080
NV12 frame, both decoding to that picture. It needed the BDMA fix below:
`msys_dma_copy` uses BDMA channel 2, whose done interrupt was never wired,
and the input buffer copy waited forever.

**BDMA.** All four channels of the first group share INT_IRQ_BDMA (SPI 72)
here (DTS); the second group (`0x1F201600`, INT_IRQ_BDMA2) is not modelled.

The kernel builds with
GCC 10.3. The vendor tree needs `scripts/sgs_coding_style_sanitize.py`,
which is missing from U-Boot and needs `/usr/bin/python` in Linux; an empty
script that exits 0 works in both.

What differs from Iford:

| | Ifliegen | Iford |
|---|---|---|
| Cores, interrupt controller | Cortex-A7, GIC-400 (distributor `0x16001000`, CPU interface `0x16002000`), 224 interrupts | Cortex-A32, GICv3 |
| Interrupts | the same `sstar-intc` groups (128 `MS_IRQ` from SPI 32, `MS_FIQ` from 160); UART1 60, URDMA 69/53/92, MIIC0 98, timers `MS_IRQ` 101-103 and level, watchdog 162 | UART1 69, URDMA 67/92/80, timers `MS_FIQ` |
| Console | IPL and kernel print through the URDMA's TX FIFO (reg `0x17` data, `0x1C` status, `0x1D` enable) | the UART |
| Secure monitor | TF-A at the top of DRAM. PSCI `CPU_ON` writes smpctrl `0x13`/`0x14` and `0x15 = 0x0C`; CPU1 starts Secure at TF-A's warm entry | OP-TEE, or none |
| I2C | `0x1F222800`, `0x1F222A00`, `0x1F222C00` | `0x1F222800`, `0x1F222A00`, `0x1F007C00` |
| SD | FCIE banks `0x1413` and `0x1416`, write data by auto-trigger | `0x0042` |
| Encoder | `sstar-venc`, a command-ring firmware ABI | `sstar-venc`, job slots |
| Crypto | the AES and RSA reset-done bits (`0x7F` bit 4, `0x29` bit 3), which its driver waits for at probe | |

Not done: the rest of the media path (VIF, ISP, scaler): no sensor runs
on this machine, so the encoder is fed from memory only. U-Boot's `DRAM: 48 MiB` is its
build's `CONFIG_UBOOT_RAM_SIZE`; mstar-88 found SP_MIN stays resident at
`0x23FE0000`, so a U-Boot relocating above it overwrites it.

## infinity6 dies

Sources: the vendor Pudding (infinity6, 6b0, 6e) and Maruko (6c) SDKs, chip
IDs from ipctool, and the published `u-boot-<soc>-nor.bin` and
`openipc.<soc>-nor-lite.tgz` of OpenIPC/firmware, the controls every result
here was judged by. Nothing is checked on silicon. The default DRAM of each part
is a convenience, not a datasheet value: use `-m`, the IPL sizes what it finds.

```
qemu-system-arm -M ssc338q,flash-file=full.bin -nographic \
    -nic user,net=192.168.1.0/24,host=192.168.1.107 -drive if=sd,format=raw,file=card.img
```

| | infinity6, 6b0 | infinity6e | infinity6c |
|---|---|---|---|
| CPU, GIC | A7, GIC-400 | 2× A7, GIC-400 | A35 (A7 with its MIDR), GICv3, Monitor at reset |
| IMI | 96 KiB | 184 KiB | 240 KiB |
| Boot strap | PM clkgen `+0x7C` bit 14 | DID key reg `0x70` | DID key reg `0x70` |
| DRAM size the IPL records | MIU `0x1012` reg `0x69` | same | bank `0x1659` reg `0x0D`, log2 MiB; `0x69` is a fixed `0x6000` |
| FSP | `0x0016` | follows `bond=`: `[5:4]` 0 is `0x0016`, else `0x100D` | `0x0016`, software CS, outside size is count + 1 |
| RTC | `sstar-rtc`, bank `0x0012` | RTCPWC with the ISO FSM | RTCPWC, Iford's handshake |
| EMAC PHY bank | `0x0031` | `0x1514` | `0x1518` |
| SD card detect (input pad) | PM GPIO `0x47` bit 2 | PADTOP `0x11` bit 0 | PADGPIO `0x06` bit 0 |
| Console DMA | no | no | URDMA, Iford's bases |

What the boot chain needs, each found by a vendor image failing without it:

- **None of the SSD202D capture.** Its efuse trim at `0x1F004014` sends the
  6b0 IPL into a deliberate `b .`.
- **The MIU BIST writes DRAM.** The IPL runs it on every boot: length in regs
  `0x72`/`0x73`, pattern in `0x74`. IPL_CUST reads a header's compression byte
  from `0x23D0001F` before it has put anything there and takes a zero for a
  chained image, so zeroed DRAM boots into nowhere, and U-Boot's heap does the
  same on a warm reset. The 16-byte length unit is an inference: 63 MiB exactly
  on a 64 MiB part.
- **infinity6c must say Cortex-A35.** `__v7_ca35_setup` in the kernel's
  `proc-v7.S` leaves Monitor mode Non-secure, and it matches MIDR `0x411FD040`
  and no other. In the Secure state the kernel's Group 1 NS interrupts never
  arrive, and it hangs at its first sleep.
- **infinity6e's MMU** (bank `0x1013` regs `0x50`-`0x5B`, 1024 entries, 128 or
  256 KiB pages). `mi_sys` takes the region above DRAM, CPU `0x50000000` on a
  256 MiB part, where QEMU would repeat DRAM from 0, so a window translates it
  and an address with no entry reaches nothing. BDMA goes through it too.
  BDMA's address registers are full words: the 12-bit mask in the IPLs is
  theirs. Masking to 28 bits wraps those clears onto the kernel's page tables.

Not modelled: USB beyond one host and BDMA's second group on infinity6c
(`0x1F201600`). The second SD host is bank `0x1410` on every infinity6 die,
`-drive if=sd,index=1`, interrupt `INT_IRQ_SDIO` or, on infinity6c,
`INT_IRQ_FCIE_TOP`; Linux finds its card on `ssc337de` and `ssc338q`. OpenIPC's kernel
for infinity6e has the bond hard-coded to `0x10` (`halSERFLASH.c`), so with
`bond=0` it finds no flash and panics at root mount: U-Boot follows the strap,
that kernel does not.

### Media on infinity6

`ssc325` follows `ssc337` with these differences, each from its `mhal.ko` and
device tree: VIF at bank `0x1319` with frame end on bit 1, not 5; five ISP
interrupt groups that are not contiguous (regs `0x28`, `0x56`, `0x4A`, `0x5B`,
`0x2D`), with frame start and process done on group 0 bits 14 and 15; the CMDQ's
idle on bit 11 alone, since `0x1700` of that word is an error; and an MFE at
`0x1F264E00` and an MHE at `0x1F264800` in place of the Wave5.

Through the real boot chain its closed modules load, the VPE runs and the scaler
draws frames. `mi_vpe` is woken only by the scaler's interrupt and sets its first
buffer after one, so that interrupt follows the scaler's input with or without a
buffer. MJPEG comes out (see below).

H.264 runs on `sstar-mfe` with `riu` linked, through the vendor driver on a
`mitest` pipeline: 997 frames at baseline CAVLC and 1056 at main profile CABAC,
both 0 decode errors with I frames every 30. There is no vendor source for this
generation, so what is here comes from `mhal.ko` and traces. The core is
infinity3's. The size, source and output moved to the `ven` banks: `0x1329` regs
`0x10` start, `0x11`/`0x12` size, `0x15` pitch, `0x20`-`0x23` source; `0x132B`
`0x03`-`0x06` output. Regs `0x01`/`0x02` hold a 216-byte record the driver ends a
frame from, and `0x39` bit 4 marks an I frame, which every slice header of a run
agreed with. CMDQ1 carries the job, waits for the encoder and forces frame done
through `0x1C`.

Limits of this model:

- The reconstruction stays inside the device, per stream, because the layout of
  the reference buffers is not known. It is not migrated.
- The CMDQ does not hold on a wait, so the frame is encoded inline.
- `disable_deblocking_filter_idc` is taken as 0: the driver always sent that.
- Only `mitest` has driven this, without 3A, so the picture is dark.
- The driver logs a few `mfe ref_ring_EndAddr out of range` lines. The ring
  position alternates between two slots and the record's end runs past the ring
  from the second one. The real wrap needs a ring size the model does not know,
  and the stream is not affected.

H.265 runs on `sstar-mhe`, through the same driver and `mitest`: 871 frames under
rate control and 874 at a fixed QP 33, both 0 decode errors, an I frame every
30. It too comes from `mhal.ko` and traces, and it shares the `ven` banks. The
driver feeds VPS, SPS, PPS and the slice header through a packer (`0x1324` regs
`0x49`-`0x4B`) and the hardware appends the slice data, which is
`hevc_enc_slice_data()` here. `0x48` is the QP, the only register that followed
the slice header at both QPs. The bit count is at `0x1325` regs `0x73`/`0x74`, and
the record is at `0x1326` regs `0x65`-`0x67`. The same limits apply as for the
MFE, and:

- `hevc-enc` codes I pictures at 8x8 block resolution, and CUs that changed in a
  P picture as intra, so the picture is coarser than the H.264 path's.
- The coding parameters are fixed to what the driver's SPS and PPS said (CTB 32,
  transform depth 2, no SAO), not read from them.
- `mhe ref_ring_EndAddr out of range` lines appear for the same reason as the MFE's.
- Under rate control the driver logs a few `uiTotalCnt is 0` lines: the record's
  histograms are zero. Whether that changes rate control is not known; QP stayed at
  10 throughout, and the model's P frames are very small.

Boot with the board's U-Boot for this: under `-kernel`, `insmod mi_sys.ko`
sleeps forever on `ssc325`. The cause is not known.

### H.264 on the Wave5

These dies' drivers ask for it in the instance register's high half, 3 for AVC.
The model answers with its own baseline CAVLC SPS and PPS, in front of every
IDR, and intra pictures from `hantro-h264.c`, so the stream holds no P slices
and is far larger than silicon's. On `ssc337` the vendor drivers stream it and
`ffmpeg` decodes it without errors. The other dies share the path but have not
been run.

### Media on infinity6e, infinity6b0 and infinity6c

All three run the vendor stack from sensor to H.265 over the network: the
OpenIPC kernel and `osdrv` modules and libraries, with
[waybeam](https://github.com/OpenIPC/waybeam) in front on `ssc338q` and
`ssc377`, a small MI test program on `ssc337`. None of these has a first-boot
gate or needs an EULA acceptance. `chagall.bin` has to be the set's own: the
driver compares the build digits in the image.

| Block | infinity6e, infinity6b0 | infinity6c |
|---|---|---|
| Sensor | `sensor=` on MIIC1, as `ipctool` looks for it; `imx415` on infinity6e, `imx307` on infinity6b0 | the same, `imx415` |
| VIF | `sstar-vif` with `i6e`, bank `0x1304`: per channel, mask reg `0x0B`, clear `0x0D`, status `0x0E` | `sstar-vif`, bank `0x1308`, Iford's layout |
| ISP top | `sstar-isp`, bank `0x1302`, five interrupt groups. On infinity6b0 group 0 bit 1 too: `mi_vpe` counts frames on it, and drops a realtime channel's first ten against that count, for ever if it does not move | `sstar-isp`, seven groups |
| Picture | `sstar-iford-isp` with `sensor-window`: the scene through the sensor's exposure | the same |
| Scaler output | `sstar-i6e-scl`: bank `0x1213`, `0x1215`; interrupts bank `0x1218` | `sstar-iford-scl`, bank `0x1253` |
| CMDQ | `sstar-cmdq`, Ifado's four banks | twelve, banks `0x1120`, `0x1123`-`0x1129`, `0x1360`-`0x1363`; idle is status bit 27 alone |
| MIU MMU | infinity6e: bank `0x1013`; infinity6b0 has none | bank `0x1659` as Iford, init done bit 9, regions in regs 5 and 6, and the CPU's window at `0x60000000` |
| Encoder | `sstar-wave5` at `0x1F345200`: host commands, H.265 from `hevc-enc.c` | `sstar-wave5` at `0x1F34D000`: the same, and pictures through a queue |
| CPU | Cortex-A7 | QEMU's ARMv8 `max`, saying A35: userland built for the A35 traps on a plain A7 |

VPE to VENC is a **hardware ring**: on `ssc338q` the port's buffer holds 138
lines of a 1280x720 stream and the MIU MMU maps nothing beyond them, and on
infinity6c the ring is at the sensor's pitch and shorter than the frame, so a
frame is never in memory. The scaler gives the frame straight to the encoder
(`sstar_i6e_scl_ring()`, `sstar_iford_scl_ring()`). infinity6b0's ring is a
whole frame, which the encoder reads like a whole-buffer bind; both bind types
stream there.

infinity6c's driver (`codec40`) queues pictures: sixteen 256-byte pages in
memory, each an image of registers `0x100`-`0x1FF`, named by `0x16C` at
`INIT_VPU`, with the host's index in `0x1F8` and the firmware's in `0x1F4`. A
picture's results go to memory too, in the buffer `CREATE` names in `0x14C`.
Reg `0x1E8` is bit 24 and an instance number there, not a bitmask. Addresses
are relative to 1 MiB below the configured base, except the sources'.

### 3A, JPEG and audio on the infinity6 family

The vendor's own AE and AWB run on `ssc337`, `ssc338q` and `ssc377`, reading
the statistics this model writes. Two things hold them up on real firmware, and
neither is the model's:

- **`iqfile0.bin` and the API `.bin` are different files.** `libcus3a` loads its
  own IQ file from `<isproot>/iqfile0.bin` and checks its size. Putting
  `configs/<sensor>.bin` there fails that check, and AE then starts with a NULL
  configuration and segfaults in `AE_GetParamFromCfg`.
- **OpenIPC's `sigmastar-osdrv-infinity6c` mixes builds.** Its `mi.ko` is from
  2024-06-18 and its `libcus3a`, `libispalgo` and `libmi_isp` from 2022-09-07.
  They disagree on a structure ("Structure size not match (112, 108)"), so the AE
  limits read back as zeros and AE stays at the floor. The 2024-06-18 set in
  `waybeam`'s `vendor-libs/maruko` works.

The model's part: the AE grid is a fixed 32 x 32 here, with no block count
register. `ssc377` needs frame done on ISP group 0 bit 10, armed by bit 9, for
the statistics to change buffer and `/dev/isp_fe` to wake the 3A.

Measured with `waybeam` from a lossless capture at QEMU's network layer:

| | Mean Y | Lines | Gain | Frames decoded |
|---|---|---|---|---|
| `ssc337` | 32 | 1124 | 1024 | 2056 |
| `ssc338q` | 41 | 1620 | 1024 | 1261 |
| `ssc377` | 45 | 949 | 2043 | 827 |

`ssc338q`'s yellow picture under `waybeam` is not a model error. `waybeam` takes
AWB from the ISP there and runs its own grey-world loop, and the scene averages
blue: the block means are 18/36/88, the loop's target 2012/418, the gains settle
at 2011/428 inside its 3% deadband. With `awbFps: 0` the vendor's AWB keeps it
and gives 2077/1759 on the neutral band. Sensor exposure in this model is linear,
so only the statistics, not the picture's look, are comparable with silicon.

JPEG uses `sstar-i3-jpe`: the family keeps jpe2's registers, the mask moved to
`0x22` and addresses widened, and goes through the MIU MMU. `ssc337` streams
MJPEG through the vendor driver with a correct picture. `ssc338q` answers
`waybeam`'s `/api/v1/snapshot.jpg`, 4 of 4 clean: its driver finishes a picture
from a record the engine writes to memory (regs `0x52`/`0x53`), and the JPEG
channel shares scaler port 0's ring with H.265, so the JPE takes that frame from
the scaler. No register marks a ring; the MMU table does, with pages missing or
wrapping onto pages already used.
`ssc325` streams MJPEG through the vendor driver on a `mitest` pipeline: 1017
frames, with `0x1C` still its interrupt mask (`mask-1c`) and rate mode 6 for
fixed-QP JPEG on that SDK. Without 3A running the picture is dark. One early
frame of 1017 was flat. `ssc377` keeps `sstar-jpe`, not run.

Audio capture reaches `waybeam` on `ssc337`, `ssc338q` and `ssc377`: a 1 kHz tone
at amplitude 8000 with no RTP gaps. `mi_ai` reads the ring as two interleaved
channels. `ssc377`'s DMA counts 16-byte units and goes through the MIU MMU; with
8-byte units the tone overwrote the kernel. Playback (AO) is not modelled.

## Part defaults

Where each default comes from. OpenIPC/firmware's defconfigs name the vendor
SKU of each part; the SDK's device trees and board configs give the SKU's
sizes. **Unknown** means no source ties a size to that part. Those machines
print a warning naming the evidence unless `-m` sets the size.

| Part | SKU | DRAM | Status | Source | Package / bond |
|---|---|---|---|---|---|
| `ssr621q` | SSC010A | 256 MiB | sourced | retail U-Boot `LX_MEM=0xFF00000`; vendor `ssc010a-s01a-2Gb` DTS | bond `0x1E`, source not rechecked |
| `ssd202d` | SSC010A | 128 MiB | sourced | kernel `mstar_get_package_type`: SSC010A is `QFN_DDR3_128MB`; DTS `0x07E00000` | QFN, bond `0x1E` from the capture |
| `ssd201` | SSC011A | 64 MiB | partly | DTS `0x03E00000`; that SSD201 is SSC011A is inferred | bond `0x1D` from the capture |
| `ssc375` | SSC032B | 128 MiB | sourced | retail Dahua `mem=128M` | bond `0x15`, WB1G DDR3 in its IPL |
| `msc313e` | | 64 MiB | sourced | SDK `image_QFN64M-MSC313E`; datasheet "up to 512Mb" | QFN, not read |
| `msc316dc` | | 128 MiB | sourced | SDK `image_QFN128M-MSC316DC` | QFN, not read |
| `msc316dm` | | 128 MiB | partly | OpenIPC U-Boot only; the SDK has no MSC316DM image | **unknown** |
| `ssc029` | | 128 MiB | partly | the model's MIU size register; not rechecked against the SDK | bond `0` picks a DRAM profile |
| `ssc029a` | | 512 MiB | sourced | SDK `ssc029a.512.bga12_ddr4`; its IPL sizes it to 512 MiB | BGA, any bond |
| `ssc029b` | | 256 MiB | sourced | SDK `ssc029b.256.bga11_ddr3` | BGA; the IPL takes `0x10`-`0x1D`, `0x34`-`0x35`, `0x50`-`0x5D`: **unknown** which |
| `ssc034a` | | 128 MiB | sourced | SDK `ssc034a.128.qfn128`; the IPL prints `QFN128 DDR3 4x 1G` | QFN128, bond `0x09` (inferred, see ⁵) |
| `ssc034b` | | 64 MiB | sourced | SDK `ssc034b.64.qfn88`; the IPL prints `QFN88 DDR2 4x 512M` | QFN88, bond `0x00` (inferred, see ⁵) |
| `ssc325` | SSC009A | 128 MiB | **unknown** | vendor DTS `0x07E00000`, nothing else | **unknown**, not read |
| `ssc325de` | SSC009B | 256 MiB | **unknown** | vendor DTS `0x0FE00000`, nothing else | **unknown**, not read |
| `ssc333`, `ssc335`, `ssc337` | 6b0 SSC009A | 64 MiB | sourced | vendor DTS `0x04000000` | **unknown**, not read |
| `ssc335de`, `ssc337de` | 6b0 SSC009B | 128 MiB | sourced | vendor DTS `0x08000000` | **unknown**, not read |
| `ssc30kd`, `ssc30kq`, `ssc338q` | SSC012B | 128, 128, 256 MiB | **unknown** | SDK builds 128 and 256 | QFN128, bond `0x10` (`CHIP_BOUND_QFN128`) |
| `ssc377`, `ssc377d` | SSC027A | 128 MiB | **unknown** | SDK builds 64 and 128 | QFN88 in the SDK's names, not read |
| `ssc377de`, `ssc378de` | SSC027D | 128 MiB | **unknown** | SDK builds 128 and 256 | QFN128 in the SDK's names, not read |
| `ssc377qe`, `ssc378qe` | SSC027D | 256 MiB | **unknown** | SDK builds 128 and 256 | QFN128 in the SDK's names, not read |

The 64 MiB `ssc325` used to have was a guess, and the vendor's device trees
disagree with it. Vendor trees state DRAM less 2 MiB (`ssc010a` too), but they
are explicitly "replaced with LX_MEM" and name a SKU, not a part. So `ssc325`
stays unknown at their size.

## SPI NAND

An image starting with an SNI (`MSTARSEMIUSFDCIS`) on an infinity6 family
machine gets a SPI NAND on the FSP in place of the NOR, and the boot straps to
match. `sstar-fsp` holds page + spare for every page in `flash-file`, so
bad-block marks survive. `tools/sstar-nand-image.py` lays out the published
images (`u-boot-*-nand.bin`, `rootfs.ubi.*`), which carry no spares:

    tools/sstar-nand-image.py flash.nand u-boot-ssc337de-nand.bin rootfs.ubi.ssc337de
    qemu-system-arm -M ssc337de,flash-file=flash.nand -nographic

- **Straps.** Each reader has its own, in the vendor's sources. On infinity6
  and 6b0 U-Boot reads PM clkgen `+0x7C` [14:10] (`BIT4|BIT0`) and the kernel
  DID key reg `0x70` (`BIT2`); on infinity6e both read DID key reg `0x70`
  (`BIT4`). The kernel binds `ms-spinand` only if its own strap says NAND.
- **The ROM's part.** Inferred, no dump exists. It finds an SNI at a block
  start, takes the IPL from BL0PBA or BL1PBA past a block marked bad, and leaves
  page 0 at `0xA0010000` and page 1 at `0xA0010A00`: the IPLs of all three dies
  read their geometry from those copies.
- **The chip.** Page read, cache read by RIU or BDMA, load, execute, block
  erase, features. Bits only clear, and BP bits set at power-on refuse writes.
  `flash-nand-id=` overrides the ID; otherwise it is what the SNI names, or a
  GD5F1GQ4U (`0xC8F1`) for a generic SNI, whose ID is `0xEE 0xEE` and a version.
- **Faults for testing.** `-global sstar-fsp.ecc-flip-page=`, `ecc-err-page=`
  and `fail-block=` make the status report a corrected read, an uncorrectable
  read, or a failed erase or program. The data stay as stored: there is no ECC
  computation behind the status field.
- **A held page load.** `HAL_SPINAND_PROGRAM_BY_BDMA` opens a load in one fire
  with the software chip select off, and enables it only for the next. The model
  keeps a load selected until a later fire without it, or the driver's release.
  That is inferred from the driver working on silicon, not from a document.

Tested from the published images: `ssc325`, `ssc325de`, `ssc337de` and
`ssc338q` to login, `ssc30kq` to `OpenIPC #`; `nand` erase, write, read and
`cmp` on `ssc337de`, the bytes checked in the raw file; `markbad` surviving a
reboot; `ecc error` on an uncorrectable page; `MTD Erase failure` on a failing
block.

Geometry comes from the SNI, so a 4 KiB page part works, at register level only.
infinity6c gets the strap (DID key reg `0x70` BIT4, from its U-Boot and kernel).
No published NAND image exists for it; the Maruko SDK's own SPI NAND chain runs
to U-Boot, and a firmware written by a NAND U-Boot to `openipc-ssc377 login:`.

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

## Boards

The `msc313e` machine takes other infinity3 boards through its options. The
firmware expects the board's own sensor and RAM: with the wrong sensor an
app gives up (Anjoy's reboots, Raysharp's dies under its watchdog), and with
too little RAM the kernel's CMA pool wraps onto low memory and corrupts it.
A board's real values are in its dump: the sensor from `Sensor_id_table` in
the `libdrv_ms_cus_*.so` it ships, the RAM from the kernel's `Memory:` line.

| Board | Options | Result |
|---|---|---|
| Jabsco C3001, C301-M15 | `sensor=jxf22` (the default) | login, app runs |
| Anjoy YM200J, YM200J2 | `sensor=imx307` | YM200J: H.264 over RTSP; AE converges |
| Anjoy MC300E10 (its "SC32xx" name notwithstanding) | `sensor=sc2315e` | no reboot, AE converges |
| Raysharp MSC316D (RS-CH228M2NN) | `sensor=sc2310,rtc=ds1338 -m 128M` | video pipeline runs, AE converges |
| MSC316DC | `sensor=imx323 -m 128M` | video pipeline runs; picture saturates, AE does not settle |

`rtc=ds1338` puts a DS1307-family RTC at 0x68 (0xD0 in 8-bit) on the first
I2C bus, where the Raysharp app reads its time. The Anjoy `CRY` driver
(`/dev/cry`) reads "failed" on every board here and the firmware runs on.

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
  the kernel's `IMI_SIZE`, 240 KiB, here. It picks the DRAM profile from the bond strap, `0x10`-`0x19`
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
  is never busy. ADMA is Iford's; this driver does not use it. The card is
  `-drive if=sd,format=raw,file=sd.img`; PM_SD_CDZ (PM GPIO `0x47` bit 2, active
  low) reads "card in" then, else the drivers report no card at once.
  The vendor U-Boot's `mmc info`, `fatls`, `fatload` and `mmc write`/`read`
  work against it (its FAT wants a partition table: an MBR with one FAT
  partition).
- **RTC** (`sstar-rtc`, bank `0x0012` at `0x1F002400`, SPI 76): the
  infinity PM RTC of `drivers/mstar/rtc/ms_rtc.c`, a seconds counter with
  load and latch requests the hardware clears (the driver spins on them
  with no timeout: the boot hung there) and a match alarm. It starts at
  the host's time. infinity2m and later have the RTCPWC instead.
- **RTCPWC on infinity2m** (`sstar-rtcpwc` with `iso-fsm`, bank `0x0034` at
  `0x1F006800`): the flags and data of Iford's RTCPWC behind another
  handshake, from `drivers/sstar/rtc/ms_rtcpwc.c`. The driver steps reg 3
  through 000, 001, 011, 111, 101, 001, 000; reg 8 bit 3 acks each step and
  reads as the state's parity, and the command crosses as the sequence ends.
  The driver retries that sequence forever under a spinlock, so a wrong ack
  hangs the kernel rather than warning. SW0 starts at the driver's magic
  `0xBABE` and the base at the machine's RTC time, as on a board that kept
  its battery. Its alarm and power off are not modelled: this driver has
  the alarm compiled out and takes no interrupt, and what reg `0x0F` and
  the registers at `0x30` do on this die is not known.
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
  bytes and 128x90 AWB blocks of 3, and two luma histograms (ISP3
  `0x60`-`0x74`: each window counts its blocks by Y / 2 into 128 16-bit
  bins, at the histogram base + 240 and + 496). The histograms are not
  optional: libCamAlgo's AE scans window 0's bins down for the top
  non-empty one with an 8-bit index that wraps, so with all of them
  empty `ISP_workqueue` spins there at 100 % CPU for good and the
  pipeline never starts (an OpenIPC image's majestic did; the camera
  dump's application streamed without them). The vendor AE and AWB
  converge on them: on the camera dump the exposure settles at 1686 rows (gain
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
  `0xF5`/`0xF6` report the sum of the macroblocks' qsteps (all at the
  frame's, `0x27`), which the `msb2` rate control averages with when its
  MB rate control is on: read as 0, the next I frame's `msb2_enc_conf`
  divides by zero.
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

### IVE

`sstar-ive`, banks `0x1520` and `0x1521`, interrupt 56, runs all 24 operations of
the I3 SDK's `mdrv_ive_io_st.h` in driver order, from the vendor's `hal_ive.c`.
What each computes comes from `mi_ive_datatype.h`. The sources do not settle the
following, so these are assumptions, listed at the top of `sstar-ive.c`: frame
edges repeat, CSC coefficients are Q8, magnitude is `|Gx| + |Gy|`, angle is 256
steps, integral "both" packs a 28-bit sum under the square sum, and an 8-bit SAD
is the 16-bit one >> 8. No vendor program has driven it: the SDK kernels here
carry no IVE driver, so the checks replay the HAL's register order.

The second SD host is the FCIE at bank `0x1402`, `INT_IRQ_FCIE`,
`-drive if=sd,index=1`. The vendor DTS leaves it unused.

### MSC316

`msc316dc` and `msc316dm` are the same die with 128 MiB. Their IPL ("IPL_EFUSE",
`gd156225`) reads efuse `0x1F004014`, which `msc313e`'s does not. The SSD202D
capture in the default register table holds `0x0809` there and sends it round
the DDR set-up: it prints `D-ff` and never writes the PHY, yet still boots, since
the model needs no training. So these machines take no capture, and it prints
`D-02.` and programs the DDR itself:

| | `msc316dc` | `msc313e` |
|---|---|---|
| Banner | `D-02.` | `D-01.` |
| PHY (bank `0x1010`) writes | 93 | 94 |
| MIU (bank `0x1012`) writes | 141 | 142 |
| DDR PLL (regs `0x18`, `0x19`) | `0x8F5C`, `0x001E` | `0x8000`, `0x0029` |
| Timing (`0x1012` regs `0x01`-`0x05`) | `03A3 000E 1538 20DD 2D76` | `0392 000D 1B28 1E99 2777` |
| BIST length (`0x1012` reg `0x73`) | `0x007E` | `0x003E` |

The size comes from the wrap: `-m 64M` gives `64MB`. U-Boot then caps at the 128
MiB it was built for, so `-m 256M` shows `256MB` from the IPL and 128 MiB from
U-Boot. The model does no DDR training, so none of these values is checked
against anything but the IPL's own tables. `msc313e` itself still takes the
SSD202D capture: its IPL does not read that fuse.

## Not modelled

- HDMI TX: no EDID, so the NVR falls back to its default output mode.
- The DIVP/DIP scalers (the VPU's own scaler serves the live view) and the
  remaining GOPs (GOP 0 is programmed but not scanned out).
- SAR ADC, the MMU block (`HAL_MMU_Reset timeout`), audio output and the
  infinity2m's audio.
- infinity (MSC313, MSC316 without E) and mercury5 parts.
  The I3 SDK's kernel also has `infinity.dtsi`; Daniel Palmer's
  `mstar-miyoo-working` branch has the mercury5 map.
- On infinity3, the SAR ADC and the FUART's URDMA.
