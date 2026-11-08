# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

**qemu-bench** (formerly `qemu-hisilicon`): out-of-tree QEMU machine + device
models for IP-camera, DVR/NVR and STB SoCs, targeting **QEMU v10.2.0**. It
boots unmodified OpenIPC and vendor-SDK firmware to a Linux shell. The core is
the table-driven HiSilicon lineup (and its Goke rebrands); next to it sit
Xiongmai, Goke GK710x, Anyka, SigmaStar and Novatek families, each with its
own machine file. There is no code here except QEMU C sources, boot/test
scripts, and docs.

## Build model — `qemu/` is source of truth, `qemu-src/` is a copy

`qemu/setup.sh` clones vanilla QEMU v10.2.0 into `qemu-src/` (gitignored),
**copies** every `qemu/hw/**` and `qemu/include/**` file into it, appends the
HiSilicon entries to upstream `Kconfig`/`meson.build` files, patches
`hw/sd/sd.c` for 1.8 V signalling, applies `qemu/patches/*.patch`, then
configures `--target-list=arm-softmmu,aarch64-softmmu,mipsel-softmmu` and runs
ninja (mipsel for the Ingenic family).

```bash
bash qemu/setup.sh          # bootstrap AND incremental rebuild — re-run after every edit
```

Re-running `setup.sh` is safe and cheap: it skips the clone, re-copies all
sources, no-ops the already-applied build-system patches, and lets ninja do an
incremental build. **Prefer it over hand-copying single files.**

The build tree silently goes stale if you `cp` only the file you touched — the
classic symptom is a QOM failure (`Property 'hisi-gmac.rx-pkt-offset' not
found` + core dump) on a machine you did not modify. If a *known-good* machine
regresses, suspect staleness, not your diff:

```bash
for f in $(cd qemu && find hw include -name 'hisi*' -o -name 'hisilicon*'); do
    diff -q "qemu/$f" "qemu-src/$f" >/dev/null 2>&1 || echo "DIFFERS: $f"
done
```

Binaries land at `qemu-src/build/qemu-system-arm`,
`qemu-src/build/qemu-system-aarch64` and `qemu-src/build/qemu-system-mipsel`;
every run/test script expects them there.

If the system has no `libfdt-dev`, meson falls back to QEMU's vendored
`subprojects/dtc`, whose `fdt_overlay.c` fails `-Werror=discarded-qualifiers`
under GCC 15 — the build dies with ~270/3379 targets done, in code that is not
ours. CI sidesteps this by installing `libfdt-dev`. Locally, either install it
or reconfigure the build tree once without werror (`setup.sh` then keeps the
existing config and just re-runs ninja):

```bash
cd qemu-src/build && ../configure --target-list=arm-softmmu,aarch64-softmmu,mipsel-softmmu \
    --disable-docs --disable-werror && ninja -j$(nproc)
```

## Testing

Machine smoke test — no kernel or firmware needed, proves device wiring
(no assert, no MMIO region overlap, no bad IRQ):

```bash
timeout 20 ./qemu-src/build/qemu-system-arm -M <machine> -m 64M -nographic \
    -display none -serial null -monitor none -kernel /dev/null
```

On 32-bit machines a clean model runs all device init, then prints
`qemu-system-arm: could not load kernel '/dev/null'` and exits **1** — the
message, not the status, is the pass signal. `aarch64 = true` configs need
`qemu-system-aarch64`, and there the empty file is accepted and QEMU keeps
running, so `timeout` kills it and reports its own **124**; that is also a
pass. Either way what fails is a QOM property error, an assert, or a core
dump — so assert on the output, not on the exit status. The Ingenic
machines (`qemu-system-mipsel -M t31` etc.) print `ingenic: /dev/null is
neither an ELF nor a uImage` and exit 1 the same way.

Device register tests (self-contained, need a built QEMU):

```bash
bash qemu/tests/test-hisi-wdt.sh        # SP805 watchdog
bash qemu/tests/test-hisi-rtc.sh
bash qemu/tests/test-hisi-sdhci-wp.sh    # SDHCI WP polarity per kernel driver
bash qemu/tests/test-hisi-mipi-rx.sh
bash qemu/tests/test-hisi-v1-ddr-init.sh   # V1 DDR needs the U-Boot reg_info table
python3 qemu/tests/test-hisi-vedu.py       # V4 VEDU (H.264/H.265) + JPGE, ffmpeg-checked
python3 qemu/tests/test-hisi-vpss.py       # V4 VI offline + VPSS node list (wide sensors)
python3 qemu/tests/test-hisi-isp.py        # V4 ISP statistics (FE / BE STT, AE / AWB)
python3 qemu/tests/test-hisi-sensor.py     # sensor timing: frame latching, group hold, fps, violations
python3 qemu/tests/test-novatek.py         # one per non-HiSilicon family: test-xm-*.py,
                                           # test-gk710x*.py, test-anyka.py, test-sigmastar.py, test-ambarella.py,
                                           # test-ingenic.py (qemu-system-mipsel)
```

Full-boot tests need firmware, in one of two shapes.

**Split kernel + rootfs** (`-kernel`/`-initrd`, what the `run-*.sh` scripts and
the CI boot matrix use) — from the GitHub release. The release publishes
it as `openipc.<soc>-nor-lite.tgz` (the two files) or, for SoCs moved since
OpenIPC/firmware#2638, `openipc.<soc>-nor-lite-split.tgz` (one
`firmware.bin.<soc>`: uImage, then SquashFS); `fetch-openipc-lite.sh` takes
either and leaves `uImage.<soc>` + `rootfs.squashfs.<soc>`:

```bash
bash qemu-boot/fetch-openipc-lite.sh hi3516ev300 qemu-boot
bash qemu-boot/run-ev300.sh              # one run-*.sh per boot-tested platform
```

**Single full flash image** (bootloader + kernel + rootfs in one `.bin`, for
the `flash-file=` boot-ROM path). The GitHub release has **no** full images —
only the `.tgz` split bundle and `u-boot-<soc>-universal.bin`. Full images are
generated by the openipc.org site: start at
<https://openipc.org/supported-hardware/featured>, pick the SoC, and the SoC
page's "Flash full OpenIPC Firmware image" section links to it. That link is a
plain GET — no form POST, CSRF token or cookie needed:

```bash
# https://openipc.org/cameras/vendors/<vendor>/socs/<soc>/download_full_image
#     ?flash_size=<8|16|32>&flash_type=<nor|nand>&fw_release=<lite|ultimate|neo>
curl -L -o openipc-gk7205v200-nor-lite-8mb.bin \
  "https://openipc.org/cameras/vendors/goke/socs/gk7205v200/download_full_image?flash_size=8&flash_type=nor&fw_release=lite"

./qemu-src/build/qemu-system-arm -M gk7205v200,flash-file=openipc-gk7205v200-nor-lite-8mb.bin \
    -nographic -serial mon:stdio -nic user,restrict=on
```

`<vendor>` is the silicon vendor, so Goke parts are under `/vendors/goke/`, not
`/vendors/hisilicon/`. The image is exactly the nominal flash size and carries
no pre-set U-Boot environment (no MAC/IP), which is fine under QEMU. The bare
SoC page is an installation-guide generator form; POSTing it yields the same
link plus TFTP flashing commands, but the direct GET skips that.

Scripted regression harnesses (these are what CI invokes; run them the same way
locally):

```bash
bash qemu-boot/test-flash-rw.sh --soc hi3516ev300 --machine hi3516ev300,sensor=none \
    --qemu ./qemu-src/build/qemu-system-arm --output-dir /tmp/frw
bash qemu-boot/test-linux-network.sh --soc <soc> --machine <machine> --append '<cmdline>' \
    --qemu ./qemu-src/build/qemu-system-arm --setup-network
bash qemu-boot/sofia-isp-ci-test.sh      # hi3516ev200 + sc2315e ISP regression
python3 qemu-boot/test-fastboot-protocol.py /tmp/hisi-fb   # boot-ROM serial protocol
```

`test-linux-network.sh` needs `--setup-network` when run by hand: without it
the harness assumes the caller already built the whole environment its ping and
curl checks expect — a bridge holding `--host-ip` (default 10.0.10.1), dnsmasq
serving DHCP on it, and a `python3 -m http.server 8080` bound to that address.
CI creates all three inline in its "Setup TAP network" step, which is why the
CI invocation omits the flag. `qemu-boot/net-setup.sh` is *not* a substitute:
it builds a 192.168.0.1/24 bridge with DHCP only and no HTTP server, so the
ping would target the wrong address and the curl would find nothing listening.
That script is for giving the interactive `run-*.sh` scripts a bridged TAP
(they fall back to SLIRP when `tap0` is absent); pair it with
`net-teardown.sh`.

Boot-test gotchas that will otherwise waste a session:

- Pass `-nic user,restrict=on` for unattended boots. OpenIPC's busybox `ntpd`
  has a 0.5 ms `STEP_PEER_MAX` threshold; with real NAT latency it loops
  forever and init never reaches login. `restrict=on` keeps SLIRP DHCP working
  while failing NTP DNS fast.
- Use `-machine <soc>,sensor=none` where the SoC has a `default_sensor` but the
  vendor MPP blob segfaults under TCG (e.g. `hi3516ev300`) — otherwise
  `load_hisilicon` insmods it and boot stalls before login.
- Do **not** build new tests around `expect` driving the getty login: they are
  persistently flaky (getty's 60 s re-prompt desynchronises the match buffer).
  Prefer a static C binary as PID 1 via `-initrd` + `rdinit=/init`, the pattern
  in `qemu-boot/test-ive-ops.c`.

## Architecture

### Table-driven machines

Every HiSilicon (and Goke-rebrand) machine lives in `qemu/hw/arm/hisilicon.c`
(~6.4k lines) and `qemu/include/hw/arm/hisilicon.h`. Each SoC is one
`static const HisiSoCConfig <soc>_soc = {...}` — bases, IRQs, peripheral
counts, boot quirks — and one `DEFINE_HISI_MACHINE("name", tag, name_soc)` line
at the bottom of the file (48 of the 69 machines today). A single
`hisilicon_common_init()` instantiates every device from that struct; there is
no per-machine init function to write.

Family commonalities are C macros (`HISI_V4_COMMON_PERIPH`, …) expanded at the
top of a config struct, with per-die deltas written **after** the macro —
designated initialisers make last-writer-wins. Caveat: array members such as
`.regbanks` / `.uart_bases` cannot be partially overridden; re-list the whole
array when a die differs.

HiSilicon memory-map generations (the axis most differences fall on):
V1/V2 `0x20xxxxxx` with a PL190 VIC, V3/V3A/V4 `0x12xxxxxx`, V5 `0x11xxxxxx`,
and the DVR/NVR + STB outliers. `hardware_reference.md` (CV300/EV300, from SDK DTS) and
`qemu/HISILICON-QEMU-ARCHITECTURE.md` (whole-family map, SDK/U-Boot/kernel
versions, Goke↔HiSilicon equivalences) are the reference tables — new configs
should be justified against a vendor DTS, and the commit message should say
which one.

Non-HiSilicon families have their own table and init instead of
`HisiSoCConfig` slots: Xiongmai XM510/XM530/XM550 (`hw/arm/xiongmai.c`,
`docs/xiongmai.md`) and Goke GK7101/GK7102 (`hw/arm/goke.c`,
`docs/goke-gk710x.md`). On GK710x the vendor code reaches registers only
through a HAL blob that relocates, reorders and bit-permutes them: model the
silicon at the addresses the HAL drives (`qemu-boot/gk710x-halmap.py` prints
them), never the fictitious `0x90000000`/`0xA0000000` map the drivers use. The
GK710xS die (`gk7101s`/`gk7102s`, the GK7102C) is the exception: its HAL
passes most accesses through, so its blocks are modelled at those driver
addresses, behind `gk-remap` where the S layout differs.
Its flash images are byte-swapped per 32-bit word, and `-kernel` boots need
`hal-file=` (the HAL is vendor code; it is not shipped).

Anyka AK39xx (`hw/arm/anyka.c`, `docs/anyka.md`) is a third such family:
ARM926 parts whose UART and SPI move data through the L2 buffer SRAM
(`hw/misc/ak-l2.c`), not FIFOs. Its mask ROM is a synthetic program at
address 0 that replays the boot header's register script on the CPU -- the
script must land after device reset, which a reset handler does not -- and
the boot NOR is `ak-spinor`, not m25p80, because the vendor drivers send
fast-read dummies as ordinary TX bytes.

Allwinner V831/V833 (`hw/arm/allwinner-v83x.c`,
`docs/allwinner-v83x.md`) boots a camera's whole NOR image through boot0,
OP-TEE, U-Boot and Linux. Its own blocks are the `aw-*` files (CCU, DRAM
controller, SPI, DMA, watchdog, TWI engine), built under ALLWINNER_V83X.
Timers, SD, legacy TWI and the EMAC are upstream's Allwinner models;
patches 0004 to 0009 fix what the vendor drivers rely on (self-clearing
resets, NACK interrupts, no BUS_EN in master mode, absent MDIO PHYs
reading 0xffff). The GIC needs more than 179 lines,
or OP-TEE leaves every interrupt secure and Linux hangs after the timer.
`qemu-boot/test-v831-boot.sh --flash dump.bin` checks each stage up to a
shell. Video input: `aw-vinc` paints test frames into buffers behind
`aw-iommu` (the media masters use IOMMU addresses); `aw-ve` answers the
VE ID CedarC's compiled-in table and its LBC check accept (0xE0/0xE4 =
0x21110/0x21210); the sensor is the catalogue's `gc2053` on TWI1; `aw-pio` reads
inputs high (stored zeros were the YI reset button held down).
`qemu-boot/yi-v831-prepare.sh` makes a YI image that runs as if paired.
`aw-ve` encodes H.264: CedarC puts the headers through the bitstream
writer and the model appends a CABAC slice coding the input picture
(I_16x16 DC coefficients / P_Skip + I_PCM), read from the video input's
LBC buffer in a private stand-in format (`aw-qlbc.h`); `-global
aw-ve.dump=DIR` saves the streams for checking with a decoder. `aw-g2d`
runs the 2D engine's register queue (scale + convert, for the app's
motion detection). `aw-audio` captures silence so the AAC encoder and recorder run.

Fullhan (`hw/arm/fullhan.c`, `docs/fullhan.md`) is one memory map of
DesignWare blocks across five generations (FH885xV200, FH865x, FH885x V100,
FH8626V100, FH8632), its own devices the `hw/*/fh-*.c` files under
FULLHAN. The mask ROM stand-in loads the boot header's type-0x10 image (or
U-Boot past FH865x's DDR-sizing ramboot). The U-Boots key their
environment on PMU 0x330 (2 = NOR), else "bad CRC, using default
environment". The 4.9 kernel's irq handler runs find_first_bit() on the
ICTL's FINALSTATUS, which loads bytes: any MMIO a driver reads with byte
loads must extract the byte itself -- with `.impl.min_access_size = 4`
QEMU hands the handler the unaligned offset. Flash reads go through the
SPIC in 16-bit byte-swapped frames, by DMA from a data window whose
address the DMA walks, so window reads return the frame unshifted; QEMU's
m25p80 counts fast-read dummies in cycles, so `fh-spi` clocks the drivers'
one dummy byte as 8. The flash select is a GPIO, not SER. The timers'
rate comes from the PMU's divider as a Clock (U-Boot 1 MHz, V100 / FH865x
kernels 50 MHz, FH8626V100 10 MHz -- PLL1 with no /10, unlike FH8632).
Patches 0012 (CCSIDR of a missing cache) and 0013 (m25p80 clears WEL) are
for its U-Boot and 3.0.8 kernels; 0014 (dwc2 GHWCFG properties + host
descriptor DMA) for those kernels' `fh_otg`, which refuses the probe unless
GHWCFG2-4 allow its parameters and always uses DescDMA -- interrupt
channels must follow the FrameList or a hub's status endpoint storms.
OpenIPC has no Fullhan bootloader: `qemu-boot/fh-openipc-flash.sh` puts a
release on a dump's vendor U-Boot; CI boots it with `-kernel`
(`test-fullhan-boot.sh`).

Ingenic T10/T20/T21/T23/T30/T31/T40/T41 (`hw/mips/ingenic.c`,
`docs/ingenic.md`) are the one MIPS family, in `qemu-system-mipsel`, one
table entry per SoC, devices `hw/*/ingenic-*.c` under INGENIC (Kconfig in
`hw/mips`). QEMU's XBurstR1/R2 do not run these kernels: patch 0015 adds
`XBurst1` (FPU, 32 TLB entries, Config7.WII because the kernel idles in
`wait` with IE=0, R2 ISA for the SPL's `mfc0 $12,2`) and `XBurst2`
(PRid company 0x13, Config4.IE=2 or `tlbinvf` is a no-op). A NOR image
boots through a mask ROM stand-in: SPL header check, SPL into the
cache-as-RAM window (patch 0016: KSEG0 0x80000000+32K goes to 0x1F000000
until the first KSEG0 fetch outside it -- the T41 SPL tests DDR through
KSEG1 underneath itself), sp = 0x80008000 (the SPL never sets it). The
kernels size RAM from `mem=` only, and early printk uses the first UART
with LCR != 0, so `-kernel` boots from a ROM stub that sets UART1's LCR
and passes U-Boot's argc/argv. The SPL/U-Boot timer is the OST inside
the TCU (+0xE0), not the kernel's OST at 0x12000000; reboot is the TCU
watchdog. SFC END means "all data in the FIFO", not "drained" (vendor
U-Boots wait for it after reading part of a word), and SFC DMA addresses
are taken modulo 512 MiB (T41 passes virt_to_phys of a KSEG1 pointer).
PLL encodings differ per SoC (`ingenic-cpm.c`); reset values are U-Boot's
because the kernels oops before the console on a mux of 3. XBurst2's
second core starts powered off and the CCU starts it at RER.
OpenIPC publishes U-Boot and kernel/rootfs but no full image;
`qemu-boot/test-ingenic-boot.sh --mode flash` lays them out and boots
them, `--mode kernel` uses `-kernel` with the root on the NOR.

Grain Media GM8135S/GM8136/GM8136S (`hw/arm/grainmedia.c`,
`docs/grainmedia.md`) put an FA626TE -- QEMU's arm926 with MIDR
0x66056261, which the kernel's proc-fa626te matches -- among Faraday
blocks, its own devices the `hw/*/gm-*.c` files under GRAINMEDIA (the
Ethernet is QEMU's FTGMAC100, the SD hosts generic-sdhci, the USB host
QEMU's FUSBH200 EHCI inside `gm-otg`). A NOR dump boots through the vendor
nsboot (flash 0x1000, run in SRAM), whose frequency change waits in WFI
for the PMU's interrupt 8, then U-Boot and the 3.3 kernel, machine 758 with
ATAGs. The FTINTC030 masks through the CPU 0 target registers (0x45C), not
ENABLE. FTSPI020 reads fill an RX FIFO and complete when the last byte is
in it: U-Boot waits for completion before draining. The kernel moves flash
data with the FTDMAC020 on request line 8, run from a bottom half because
the request rises inside the SPI controller's own register write. U-Boot
picks bootargs by the DDRC 0x10 size code; the kernel's fmem hangs without
`mem=` (the machine appends it), and the adda308 audio driver panics unless
PLL1 is a rate in its table (the PMU resets to nsboot's 860 MHz). The SDK
uImages are XIP-style (load = header address), loaded 64 bytes up.
The vendor video graph declares a job lost ("damnit", GS panic) unless
every engine it waits on calls back, so capture (`gm-vcap`, link lists
and a 60 Hz sync-timer interrupt that the driver links jobs from), 3DNR,
scaler, LCD (its vertical interrupt is the graph's clock), GM2D and the
MCP280 H.264 encoder (`gm-h264e`) are modelled. The encoder's driver packs
SPS / PPS / slice headers through a FIFO and the model appends real slice
data (I_16x16 DC from `hantro-h264.c`; P: changed macroblocks I_PCM, the
rest P_Skip, against the reference kept NV12 in the driver's REC buffer),
so a Longse dump streams decodable H.264 over RTSP
(`qemu-boot/test-grainmedia-rtsp.sh`, admin / admin, /ch0). ffmpeg's
`gray` output rescales limited-range luma: compare against `yuv420p`
planes. The ISP328 (`gm-isp`) must keep frame start / end running even
while disabled (the driver waits on both around stop and resize) and
synthesises statistics that its output DMA table copies to the
algorithms' buffers. MJPEG (`gm-mcp100`) goes through the embedded CPU's
mailbox at SRAM 0x8000 (an MMIO overlay; command byte 8 starts, 0x80
done); the host itself packs the JPEG headers through BADR / BALR, and the
scan comes from `nvt_pixel_jpeg_scan_rst()`. Audio (`gm-ssp`) must never
raise FIFO overrun / underrun -- the driver tears the stream down -- so the
FIFOs keep samples behind the 16 they report; the Longse app captures
only with sysenv.cfg byte 0x98C set. The FTAES020 is `fh-aes` with
`advance=on` (addresses move on between chunks).

SigmaStar/MStar infinity2m, Ifado, infinity3, Iford and infinity6* (`hw/arm/sigmastar.c`,
`docs/sigmastar.md`) are a fourth: Cortex-A7 parts whose blocks sit in
16-bit RIU register banks at `0x1F000000` (register r of bank b at `0x1F000000 + b*0x200 + r*4`, high byte
at +1). `sstar-riu` is mapped under every model as storage plus the status
bits the boot chain polls; `-d unimp` reports hot polls and
`-global sstar-riu.trace=<banks>` logs a bank. The machine sets
`minimum_page_bits = 12`: with QEMU's default 1 KiB ARM pages a region that
starts inside one 1 KiB page and covers the next is registered with a
misaligned offset, and its first access dies in `iotlb_to_section`. Any
machine with 512-byte-aligned MMIO blocks larger than 1 KiB is exposed to
the same assertion. The dies differ by `SsDie`: the infinity3 (`msc313e`)
ROM takes its IPL from NOR `0x4000` (`ipl_offset`), and its scaler driver
reads its gamma SRAMs back entry by entry, waiting 2 s on each mismatch
(`sstar-vip-gamma`). A boot that goes quiet with the CPU idle is often
such a slow poll, which the hot-poll detector does not see;
`-trace 'memory_region_ops_*'` does. Its video decoder (`hw/misc/sstar-vpu.c`) is a WAVE5
modelled at the firmware's mailbox, decoding with libavcodec when pkg-config
finds it at build time (CI builds without it); queued commands complete
from a timer, because the vendor driver arms its wait for the interrupt
after issuing the command and loses a completion raised inside the
doorbell write.

Ambarella S3L (`hw/arm/ambarella.c`, `docs/ambarella.md`) is one Cortex-A9
machine, `s3l`, whose blocks are the `amb-*` files under AMBARELLA. It stops
short of video: it exists so ipctool and the firmware's `load_ambarella` can
be tested without the camera. With `-kernel` the machine generates the device
tree, including `/iavmem`, the DSP's share of DRAM, from `-m`; Linux gets what
is below 0x07800000. On S3L the kernel takes interrupts from one scratchpad
register, not the banks, and all ones there means none (0x60 is S3's value).
`iav.ko` oopses without an `/iav/vinc0` node. The sensor is off I2C until GPIO
107 goes high (`amb-gpio`), as on the camera; `sensor-reset=off` removes that.
With `flash-file=` and no `-kernel` it runs a camera's own BST and BLD: the first
NAND page sits at the high vectors `0xffff0000`, which the BST copies from.
The vendor modules load, but no microcode ships, so the DSP stays down. `flash-file=` adds NAND
(`amb-nand`), page data alone or with spare bytes. BCH is not modelled, partitions
come from `mtdparts=`, and a dump's bad blocks must be listed in `bad-blocks=`. Use
a copy, or `chmod 444` it: writes go through to the file. `-m` must be over 120 MiB, so the smoke
test above needs its default RAM. A reloading `amb-timer` repeats no faster
than 1 us, so `RELOAD` 0 cannot pin the host.

Novatek NT9856x / NA51089 (`hw/arm/novatek.c`, `docs/novatek.md`) is a
fifth: a Cortex-A9 whose MPCore is at `0xFFD00000` (the `0xF1500000` GIC in
the SDK headers is stale) with Novatek blocks from `0xF0000000`. With
`-kernel` the machine generates the device tree, because the OpenIPC uImage
has none. The vendor drivers are particular about it. `fdt_add_subnode()`
prepends, so nodes come out in reverse creation order, and two orders
matter: `nvt,core_clk` must precede `nvt,periph_clk`, and the pinmux
children of the TOP node are indexed by position. The A9 timers run at
`cpu_clk / 8` from CG's PLL8, which `nvt-cg` resets to 800 MHz to match
QEMU's fixed 100 MHz MPCore timers. The MIDR is r4p1, because Linux refuses
the global timer on r0p0. The Novatek timer block (`nvt-timer`) is not
Linux's clock, but the vendor U-Boot reads its timer 0 as microseconds
since power-on, so it comes out of reset running free, as the loader
leaves it. Reboot goes through the RTC (power-on alarm, then
power off), so the RTC node is required. The kernel has no RAM disk driver,
so boot tests use an initramfs (`qemu-boot/test-novatek-boot.sh`), not the
boot matrix. A `flash-file` without `-kernel` boots through a stand-in for
the closed loader: FDT from flash 0x10000, a BCL1 (LZ or LZMA) U-Boot unpacked
to `nvt_memory_cfg/uboot`, and the `LD_NVT` SHMINFO U-Boot hangs without.
Xiongmai NOR images keep the FDT inside the loader (`XMFDT562`) and U-Boot
LZMA-packed in `partition_boot` (`lzma` property); `qemu-boot/xmimg2flash.py`
lays out their upgrade `*.img` files.
A flash that starts with a real loader (header word 1 = the SRAM base, the
entry inside it) boots through it instead: `hw/arm/nvt-bootrom.c` models the
mask ROM (copies the loader to SRAM; stubs for its ROM routines hand their
arguments to a C service window; the ROM image in DRAM at 0 and read-only at
`0x80000000`), and the loader's closed DDR training runs against
`hw/misc/nvt-ddrc.c` (done / self-clearing bits, paged PHYs). `loader=off`
forces the stand-in. On NA51089, `rom-fallback=on` (or a `boot-strap=` that
is not a flash strap) runs the ROM's own fallback when the flash has no
valid loader: SD, Ethernet (its DHCP/TFTP client on `nvt-eqos`), USB and its
UART download engine, which `defib burn` (and `burn --nic`) drive as on silicon.
`-bios re.elf` (OpenIPC/opennovatek's reconstructed NA51089 ROM) runs the ROM
itself instead: at 0x80000000 and in DRAM at 0, CPU from 0, model unused (`qemu-boot/test-novatek-recovery.sh`,
`docs/novatek.md#recovery-boot`). `qemu-boot/build-novatek-loader.sh` builds the SDKs'
loaders; `docs/novatek.md#the-real-first-stage-loader` has what each loader
waits for. Part-number machines `nt98560` / `nt98561` (NA51089), `nt98523` /
`nt98529` (NA51055 / NA51084), `nt98532` (NS02501), `nt98321` (NA51068)
differ from their siblings only in RAM size and split.
SDK `FW*.bin` packages become flash images with `qemu-boot/nvtpack2flash.py`.
A 128/256 MiB `flash-file` is a W25N SPI NAND instead of a NOR (FDT then at
0x40000). The controller's NAND mode (MODULE0 bit 19) sends status reads as
two command bytes and has whole-page ops 0x15/0x16 with the OOB in the spare
SRAM at +0x200. The OpenIPC kernels lack the NAND driver, so it is tested
with the vendor `FW98566A.bin` (`qemu-boot/test-novatek-nand.sh`).
Its I2C (`hw/i2c/nvt-i2c.c`) moves PIO chunks of up to four bytes,
with per-byte START/STOP/NACK flags in CONFIG. `sensor=` puts a
`hisi-smartsens` chip at 0x30 on bus 0 from novatek.c's own table, not
hisilicon's strcmp chain. The default is none.
Its USB host (`hw/usb/nvt-usb.c`) is QEMU's FUSBH200 EHCI plus the
FOTG200 glue. The driver reads the port speed from +0x80 and finds the
block at 0xFF600000, not 0xF0600000. It needs `nvt-otp` and the
`nvt_otp` DT node for its PHY trim. No shipped kernel has the driver;
`qemu-boot/build-novatek-test-kernel.sh` builds one, along with the PWM
driver (`nvt-pwm`; HDAL's kdrv_comm refuses the vendor `nvt,nvt_pwm`
node) and the crypto drivers with the kernel's self-tests on, which
check `nvt-crypto`, `nvt-hash` and `nvt-rsa` against known-answer vectors
(`qemu-boot/test-novatek-crypto.sh`). Those models implement their
ciphers themselves (AES from QEMU's `crypto/aes.c`), because QEMU 10.2 has
no built-in cipher backend and CI may build without one.
Its HDAL media engines (`hw/misc/nvt-media.c`) are one table-driven
device that does each engine's handshakes. Pixels are real from the SIE's
test pattern through IME's YUV to JPEG (`hw/misc/nvt-pixel.c`), and the
graphics and ISE engines compute each job from their register file
(`hw/misc/nvt-gfx.c`). The H.26x driver writes every parameter set
and slice header itself; with x264 / x265 found by pkg-config at build
time (CI has neither) `hw/misc/nvt-h26x.c` codes slices that fit them,
and otherwise writes filler slices. Audio is `hw/misc/nvt-audio.c` (DAI +
EAC). No register in either holds the sample rate; it comes from CG (PLL7
and the 0x38 dividers, 256 x fs). The address / size registers queue the
next buffer, and STOP follows the channel enable (CTRL bits 12..15),
which the driver blocks on. Capture is a test tone, and playback goes to
the machine's `audiodev`. Its display engine is
`hw/display/nvt-ide.c`. The driver sets CTRL bit 28 (load) and waits for
frame end with no timeout, and its handler wakes the waiter only if load
already reads 0. So a frame clears load before raising frame end. The
release has no display modules; OpenIPC's toolchain tarball has them
(`sdk/kmod/hdal`, `test-novatek-hdal.sh --kmods`). MIPI DSI
(`hw/display/nvt-dsi.c`) is interrupt-driven with untimed waits. Its
escape triggers must clear themselves and raise escape done, and
automatic mode raises frame end, which the driver waits for when it
closes. The AI engines (NUE, NUE2, CNN) are IPP-style
engines whose list updates have two more bits: 59 adds the list base
register, 60 ends a job and fires it, so one list holds a network.
`hw/misc/nvt-ai.c` computes the CNN's layers, NUE's fully connected and
NUE2's preprocessing, on a worker thread (a layer can take 200 ms). The
weights are 12-bit LSB-first streams, 8-bit operands count x 16 in the
multiplier, and the CNN datapath is 8.8 fixed point. These were fitted
to the SDK's classifier with `-global nvt-media.ai-dump=DIR` (job
registers and RAM). SDK v1.01.002's models are for chip id 1 (NT98520),
which `kflow_ai_net` refuses on 0x7021; v1.01.007's `ai_pattern` model is
the same network for chip id 4, and under QEMU it names the test
picture (a sea snake). The v1.01.007 AI turnkey's detectors (people,
vehicles, faces) run too, built from v1.01.007's samples and libraries
against v1.01.002's `vendor_ai2`, because v1.01.007's checks for a
`kdrv_ai` version the release modules lack. The pet detector `cddcnn`
has a library but no SDK sample; `qemu-boot/novatek-cddcnn-sample.c` is
ours. Its test checks the network's raw proposals, because limit_fdet's
classifier rejects the test pets (as the model's own call). The face
recognition network `frcnn` ships as a model only;
`qemu-boot/novatek-frcnn-sample.c` runs it through `vendor_ai_net`,
skipping the 32-byte prefix the turnkey's models carry. It fixed
four rules the detectors had hidden: BN alpha is signed, ACT bit 16
negates (PReLU is ReLU(-x), ReLU(x) and an element-wise BN), a 16-bit
out1 keeps 8 fraction bits, and the NUE takes 16-bit operands by their
top 12 bits. A conv bias without BN goes after the conv's scale-shift
(0x0AC) at any output width; the old 16-bit "before" rule had
compensated for the unsigned alpha. Motion detection is the MDBC
(`hw/misc/nvt-md.c`, node `md@f0c10000` -- probe takes the clock by that
name). `kdrv_md` waits for frame end with no timeout. The algorithm is
undocumented, so the background model is a deterministic reduced
SuBSENSE in buffers only the engine reads. The IVE (`hw/misc/nvt-ive.c`,
node `ive@f0d70000`) runs a job at START with that job's registers and
raises its frame end ~1 ms later, one per job. The driver's
nonblocking trigger releases its lock before it leaves RUN, so on a slow
host the other channel's trigger can be refused ("illegal in RUN state")
and the SDK sample then checks an empty buffer: the test reruns it on
that refusal. OpenIPC's
`kdrv_ive.ko` cannot load (it needs `kdrv_ive_waitdone_nonblock`), so
`build-novatek-hdal-samples.sh --kernel` rebuilds it from v1.01.007's
objects. The
IPP and graphics engines execute the drivers' DRAM linked lists, whose
updates set their own interrupt enables. The SIE's single-mode outputs
clear their enables when a frame is written, which is how `ctl_sie` hands
buffers on. The OpenIPC `libhdal.so` is a uClibc build that musl's 64-bit
`off_t` `mmap()` breaks, so SDK samples need the SDK's uClibc runtime
(`qemu-boot/build-novatek-hdal-samples.sh`). `-global nvt-media.trace=on`
logs at gigabytes a minute when an engine is polled: cap it.
Its SD hosts (`hw/sd/nvt-sdio.c`) DMA through a descriptor table under
Linux but a plain address under U-Boot (`0x1FC` = 1). Linux relies on the
clock framework's CG reset pulse to undo that, so `nvt-cg` drives
"sys-reset" lines into them.
Its Ethernet (`hw/net/nvt-eqos.c`) is a DWC EQoS 4.x under the Synopsys
reference driver, which needs HW_FEATURE0's MDIO bit to look for a PHY and
returns IRQ_NONE unless the Novatek glue's ETH_ISR (+0x3008) is non-zero.
Do not run a flash boot under `-d unimp -D` into `/tmp` (a tmpfs): a
firmware polling an unmodelled block fills it in seconds. Its vendor U-Boot
has `bootdelay` 0 and no saved environment, so the only way to its prompt is
a key queued before the CPU runs: start QEMU with `-S` and the serial line
on a socket (`qemu-boot/test-novatek-uboot.sh`). Keyboard input to any
16550 that a guest resets before reading relies on
`qemu/patches/0003-serial-fcr-rx-reset-accept-input.patch`.
`nt98525` / `nt98528` (NA51055 / NA51084, `docs/novatek.md#nt9852x-na51055--na51084`)
run through `novatek_common_init` with `NvtSoCConfig.na51055`: two A9s, the
second off until the kernel writes its entry to `0xF07F8000` (the loader's
release word, the tree's `cpu-release-addr`), and per-die tables for the
extra UARTs / I2C, the moved hash, the 4-window AI and 5-window SIE nodes
(`na51055_media_override`) and the dual-DDR arbiter. The NT9852x kernel
picks the CG PLL table by chip ID, so `nvt-cg` has a `layout` property
(1: `0x4821`, 2: `0x5021`). A missing PLL8 hangs the boot silently in
`read_current_timer()` before the console comes up.
`nt98323` (NA51068, `docs/novatek.md#nt98323-na51068`) is its own file,
`hw/arm/novatek-na51068.c`: Faraday-derived blocks, not NA51089's. It reuses
`nvt-cg` (layout 3), `nvt-top`, `nvt-wdt`, `nvt-otp` and `nvt-flash` (with
`spi020=on`, a FTSPI020 register front end on the same NOR/NAND engine), and
adds `hw/timer/nvt-fttmr010.c`. `-kernel` needs the SDK board tree as
`-dtb`. With `flash-file` alone it calls novatek.c's `nvt_flash_boot()` (exported in
`novatek.h`; it also accepts the `nvt,nvt_spi020_nor/nand` compatibles), and the SDK
U-Boot runs. `nvt-cg` layout 3 drops writes to PLL16, which U-Boot sets
to 1400 MHz, so `periph_clk` stays 100 MHz. `qemu-boot/build-na51068-sdk.sh` builds
U-Boot (writing the missing `ModelConfig.mk`, LZ-packing with the SDK's
`bfc`), the 4.9.118 kernel, the tree and a 32 MiB `nor.bin`; `--nand` the NT98321 NAND EVB and
a 128 MiB `nand.bin`; `--test-drivers` the drivers the SDK leaves off. NA51068-only models:
`hw/gpio/nvt-ftgpio010.c`, `hw/i2c/nvt-ftiic010.c`, `hw/dma/nvt-ftdmac030.c`,
`hw/misc/nvt-na51068-misc.c` (PWM, IR, thermal); reused: `nvt-rtc`, `nvt-usb`, `nvt-crypto`,
`nvt-hash` (with `iv-reversed`, which NA51055 needs too), `nvt-rsa`, `generic-sdhci`.
`nt98533` (NS02501, `docs/novatek.md#nt98533-ns02501`) is NA51089's
peripherals at the same addresses around two Cortex-A32s (a Cortex-A7
here), a GIC-400 at `0xFFF01000` and the generic timer, with its own init
(`novatek_ns02501_init`). Everything runs in secure SVC; the A7 keeps EL2
only because U-Boot clears CNTVOFF from monitor mode. The second core is
off until something writes `0xF0090100` (the loader's core-2 release
register, the kernel's `cpu-release-addr`). No DT is built: `-kernel`
needs the SDK board tree; `qemu-boot/build-ns02501-sdk.sh` builds it, the
SDK U-Boot and kernel, and `test-nt98533.sh` runs them.
The aarch64 `nt98332` (NA51103, `docs/novatek.md#nt98332-na51103`) shares
`novatek.c` but has its own init: blocks at `0x2_F0000000`, a GIC-400 that
must have its security extensions (the vendor U-Boot regroups every IRQ
otherwise), and only a flash boot or `-kernel` with the firmware's `-dtb`.
Its flash keeps the FDT (`XMFDT32G`) and U-Boot LZMA-packed (`nvt-lzma.c`).
By default (`-smp 3`: 2 A53s + boot core) a flash with a loader runs
Xiongmai's AArch32 loader on an extra Cortex-A7 boot core in its own CPU
cluster (IO also at 0xF0000000); its RMR write (`na51103_rmr_write`) stops
it and starts the A53s at EL3 in AArch64 at 0, into the firmware's BL31
(EL2 on, no QEMU PSCI). `loader=off` / `-smp 2` / `-kernel`: the stand-in.
`nt98633` (NA51090, 4 A53s) is the same init with `na51090_map`
(`NvtA64Map`: the blocks that moved); its default `-smp 5` runs its loader
through BL31 (nvt-ddrc `na51090`: psdk, the PHY command 11 pattern echo,
the QSEN level / gate; the trampoline's mailboxes in the core communicator
at 0x2_F0160000); `-smp 4` / `loader=off`: the stand-in.
The ROM's table stubs live at 0x7C00 (loaders scribble on 0x10 / 0x100),
and its lzma_inflate returns the unpacked length.
Do not edit the vendor device tree from machine code; board settings go in
the U-Boot environment in the image (`qemu-boot/uboot-env.py`). Its media
blocks are `nvt-engine` instances (fire, run the link list, report done; no
pixels), `nvt-dai`, `nvt-lcd300` (CTRL bit 31 is an update request the
frame clears; the driver writes nothing while it is set) and `nvt-hdmi`.
Its decoder (`nvt-h26xdec`) runs h26xdec.ko's link lists and decodes H.264
and H.265 with libavcodec when pkg-config finds it (H.265 has its own
banks at 0x2000/0x2800, and 0x211C is an error register, not a done
flag); the VPE instance of `nvt-engine` has
`scaler=on` and scales the decoder's reduced pictures into the preview
windows, and its full ones into the single-channel view. An XM530 camera
on a `-netdev socket,mcast=` LAN is the video source (`docs/novatek.md`,
"Camera video").

### Boot paths

`hisilicon_common_init()` dispatches on four mutually exclusive modes, tested
in this order:

1. **flash boot** — `-machine <soc>,flash-file=img` *and no `-kernel`*. On
   ARMv7/v5 a synthetic boot ROM is written at address 0 that copies U-Boot
   from the flash window to DDR and jumps, like silicon. The V1 quartet
   (`v1_xip_boot`) instead jumps into the flash window and U-Boot runs in
   place: its reg_info table at image offset 0x40 is what brings DDR up, and
   `hisi-ddrc-v1` keeps DDR reading zero until that table starts the DDR PHY,
   so an image built without the table fails as on silicon
   (OpenIPC/firmware#2593). On aarch64 V5 the
   gzip self-extracting `u-boot-z.bin` payload is located inside the image (by
   scanning for the reset branch + `0xdeadbeef` self-descriptor run, so a
   GSL-signed full NOR image works too), loaded at `uboot_load_addr`, and CPU0
   is started there. A `flash-file` over 16 MiB is auto-detected as SPI-NAND
   (`flash-type=1`) and SYSSTAT then reports a NAND boot strap so U-Boot takes
   the NAND/UBI path.
2. **`-bios <mask-rom.elf>`** — a reverse-engineered mask ROM in ROM at
   0x04000000 drives the boot itself, including UART0.
3. **fastboot** — no `-kernel`, no flash, serial present: `hisi-fastboot`
   emulates the boot-ROM serial download protocol for the `defib` recovery
   tool and halts the CPU until firmware is uploaded.
4. **`-kernel` (+ `-initrd`)** — DTB patching (below) plus per-SoC cmdline
   injection from `extra_cmdline`/`kernel_mem_mb`; a NULL trap page is mapped
   at 0 so NULL calls return cleanly.

### Cross-cutting mechanisms worth knowing before editing

- **Appended-DTB auto-patching** (`hisilicon_patch_appended_dtb`): OpenIPC
  uImages carry a DTB appended to the kernel. It is rewritten at load time to
  add `chosen/stdout-path`, pad for `atags_to_fdt()`, unify split SP804 clocks
  (prevents timer storms), and disable NAND controllers whose polling hangs on
  stubs. Firmware works unmodified because of this; if a new SoC hangs early,
  look here before touching device models.
- **Machine-level properties** `sensor=` and `flash-file=` are declared by the
  `DEFINE_HISI_MACHINE` macro. `flash-file` forwards to whichever controller
  the SoC instantiates (`hisi-fmc` or `hisi-sfc350`) so callers need not know;
  the legacy `-global hisi-fmc.flash-file=…` form still works. Other `hisi-fmc`
  knobs: `nand-file` (dual NOR+NAND boards), `nand-jedec`, `nand-oob-size`,
  `nor-wps-locked` (factory write-protected NOR, for recovery testing),
  `flash-jedec`, `variant` (1 = `fmc100`/`lotus,fmc` shuffled register map).
- **Chip ID**: `hisi-sysctl` serves SCSYSID. V4+ return the packed 32-bit
  `soc_id`; V1/V2/V2A/V3/V3A need `chipid_byte_layout = true` plus
  `chip_variant` (and `v1_chip_id_88`/`_8c` for the V1 quartet) or firmware
  prints `Wrong chip-id`/`hiunknown` and `load_hisilicon` bails before loading
  any vendor module. CI greps for exactly this.
- **Sensors** are I2C slave models in `qemu/hw/i2c/`, shared by every family
  and built for every target. `sensor=` names one from the catalogue in
  `hw/i2c/image-sensor.c` (name, model, 7-bit address, chip ID): add a part
  there, not in a machine, and every machine takes it through
  `image_sensor_create()`. A part new to the catalogue usually needs only a
  line: SmartSens and GalaxyCore parts share a model. Take its ID from the
  vendor driver. An ISP reads a sensor through the `image-sensor` interface
  (`image_sensor_exposure()`); a model implements it from its registers. The
  plain Sony models, `hisi-mis2006` and `hisi-sp2305` do not yet: on an ISP
  that needs a picture from the sensor they are found but draw nothing.
  The Xiongmai machines also take a QOM type name (`sensor=hisi-imx335`) plus
  `sensor-addr=`; `xm-sensor` is a generic register-file sensor whose ID comes
  from `-global xm-sensor.regs=` (`docs/xiongmai.md`). The SSI `imx122` stays
  in `hisilicon_common_init()`.
- **Video encoding (V4: EV200/EV300/18EV300/16DV200, GK7205V200 family)** is
  `hisi-vedu` (`docs/hisilicon-venc.md`). The HiMPP driver writes only a few
  VEDU MMIO registers per picture; the rest is a register image in DDR at
  VEDU+0x58 that the engine fetches at START. Output goes into a per-channel
  stream ring of 64-byte-header packets the driver itself parses (type,
  last-slice / slice-type byte, channel id -- it panics on a wrong one --
  PTS); the driver writes the SPS/PPS/VPS/SEI and JFIF header packets, the
  model appends the slice or scan and moves the ring's write-pointer cell.
  H.264 slice headers are the model's own, H.265 ones the driver's bits from
  the image (0x404-0x42c). JPGE is plain MMIO, quantisers as 2^23/q
  reciprocals. Completion is on a virtual-time timer, never inside START.
  `-global hisi-vedu.dump-dir=DIR` keeps each channel's stream,
  `trace=on` logs every access and the image at START.
  `qemu-boot/test-hisi-venc.sh` runs the vendor stack (`VENC_SendFrame`, no
  VI) under the OpenIPC release; its guest program needs a 32-bit-`off_t`
  `mmap()` shim, because the release's MPI libraries are uClibc builds.
  In front of it on the EV200-class machines (Goke rebrands too), `hisi-vi` (the
  frame clock: VI_CAP's write-channel start / early / end bits, VI_PROC's
  start / end) and `hisi-vpss` (paints its channels, then frame start +
  end) run the online pipeline, so majestic and the SDK's own
  `sample_venc` stream from `sensor=sc2315e` (Goke releases have no SC2315E
  config in `/etc/sensors`: use `sensor=sc2335` there)
  (`test-hisi-majestic.sh`, `test-hisi-sample-venc.sh --sdk`). VI runs only
  while VPSS's online video is enabled: an interrupt for a pipe that does
  not exist oopses the driver (or while pipe 0's write channel is on,
  which also starts the clock in offline mode). A sensor wider than 2304
  (`sensor=imx335`, `gc4653` on hi3516ev300) runs VI offline: the driver
  starts the pipe per frame (VI_PROC 0x300), and VPSS, armed by its 0x304,
  loads the node list at its 0x2f8 then (register images in DDR, one per
  vertical strip; the model paints the strips' widths as one picture) and
  raises no interrupt. VEDU's source stride is image 0x1f4, not 0x160
  (the 64-aligned reference stride). Compressed VPSS channels use a private
  half-height stand-in shared with `hisi-vedu`. Use `-icount shift=0`, or
  frames overtake the driver on a slow host. `hisi-vi` also writes the
  ISP's statistics (front-end AE histogram, backend AE histogram, zones
  and AWB, the STT layouts in openhisilicon's `isp_stt_define.h`). They
  come from the scene in front of the camera, `hisi-scene` at
  `/machine/scene` (`docs/hisilicon-digital-twin.md`): grey bars by
  default, or `-global hisi-scene.file=` a PPM / PFM picture. It is lit
  by `light` / `light-r` / `light-b` (settable at runtime with QMP
  `qom-set`) times the sensor's exposure × gain (`image-sensor`, one
  frame late), so AE and AWB converge. VPSS paints the same scene through
  the frame's exposure, WB gains and gamma. The SmartSens and 16-bit
  GalaxyCore models keep a sensor's timing once an ISP drives their frames
  (`sensor-sync.h`, `image_sensor_frame_start()`). Writes to exposure,
  gain and VTS latch at the next frame start, and the SmartSens group hold
  (0x3812) holds them. VI's frame clock is HTS x VTS / `pclk`. Datasheet
  violations (exposure beyond the frame, mode registers written while
  streaming) are counted (QOM `violations`) and logged with
  `-d guest_errors`. `-global hisi-smartsens.trace-i2c=on` logs every write
  with its frame and line. Families without a frame clock see writes apply
  at once, as before. Scenarios: `-global hisi-scene.timeline=FILE` plays
  timed scene changes (`20 light=150`, ramps `a..b prop=x->y`), and
  `-global hisi-vi.record=FILE` writes a CSV line for each frame.
  `qemu-boot/twin-metrics.py` turns the CSV into settle time, overshoot,
  level and AWB residual per segment. `qemu-boot/test-hisi-scenario.sh`
  runs majestic through a scenario (CI on hi3516ev200). It enforces AE
  and the timeline but only reports AWB, since lib_hiawb freezes in some
  runs (#252). An AWB zone's count is every pixel of the zone:
  lib_hiawb leaves its gains alone unless the zones' average count passes
  1024. Offline, each strip's VIPROC node (0x2f8 chain) has its own
  statistics block (S_ISP_STT_REGS_TYPE) at the address in its 0x5d0.
  The ISP driver's per-frame work runs on the front end's
  FSTART / FEND (VI_CAP 0x200f0, SPI 43, shared with capture), and a
  port status (0x10f0) with error bits set makes it skip the frame.
  `-global hisi-vi.stats-trace=on` prints what AE / AWB see.

## Adding a SoC or a device

New machine: add the `HisiSoCConfig` + one `DEFINE_HISI_MACHINE` line (or the
family's own config + `DEFINE_XM_/GK_/AK_/SS_/NVT_MACHINE` line), rebuild,
smoke-test, then add it to the README machine table and to the CI matrix if it
can boot real firmware. `-machine help` on the built binary is the
authoritative list — keep the README tables *and* their totals (the "N total"
heading and the per-target counts at the top) in sync, since those drift
silently otherwise.

New device file: `qemu/setup.sh` must learn about it in **both** branches of
each build-system patch block — the fresh-tree `cat >>` heredoc *and* the
"existing tree" `sed` splice that upgrades an already-configured `qemu-src/`.
Missing the second branch means it builds on a clean checkout and mysteriously
does not on yours. Also add the `cp` line in step 2 and the Kconfig `select`.

## QEMU v10.2.0 API notes

These bite when copying idioms from older QEMU code or from the web:

- `const void *data` in `class_init` (not `void *data`).
- `const Property[]` arrays, no `DEFINE_PROP_END_OF_LIST()`.
- `device_class_set_legacy_reset()`, not `dc->reset = `.
- Headers moved: `system/runstate.h`, `system/address-spaces.h`
  (`get_system_memory()`), `system/system.h` (`serial_hd()`),
  `hw/qdev-properties.h` (`qdev_prop_set_uint32()`).
- The PL190 Kconfig symbol is `PL190`, not `ARM_VIC`.
- Custom machine state uses a named struct with `OBJECT_DECLARE_SIMPLE_TYPE`
  and `DEFINE_MACHINE_EXTENDED` (this repo's `DEFINE_HISI_MACHINE` wraps it).

Follow QEMU coding style (4-space indent, no tabs, braces on every `if`) —
these files are meant to read as in-tree QEMU.

## CI

`.github/workflows/ci.yml` gates every PR: `build` (also asserts ≥21 `hi35*`/
`gk7*` arm machines register, that the aarch64 V5 machines and the Xiongmai,
GK710x and SigmaStar machines exist, and runs the `qemu/tests/*.py`
register tests of the non-HiSilicon families) →
`ive-test` (19 IVE ops, register-level, initramfs PID 1) → `sofia-isp`
(hi3516ev200 + sc2315e) → `maskrom-av300` (builds `re.elf` from the
`openhisilicon` repo) → a `boot` matrix that downloads OpenIPC releases and
boots ~14 SoCs (HiSilicon, Goke, Xiongmai) to a login prompt, plus dedicated
`boot-novatek` (nt98562/nt98566), `boot-hi3516cv610` and
`boot-hi3516ev300-nand` jobs. Boot-matrix rows optionally add the U-Boot ping,
Linux ping+curl, and SPI-NOR erase/program steps. Everything runs under TCG on
GitHub runners, so a boot iteration is minutes — smoke-test locally first.

## Conventions

Commit subjects are `<area>: <imperative summary> (#PR)` where area is a
family (`hisilicon`, `xiongmai`, `goke`, `anyka`, `sigmastar`, `novatek`,
`grainmedia`), a
device (`hisi-fmc`, `hisi-gmac`), a machine (`hi3516cv610`, `nt98332`), `ci` or
`docs`. Bodies are substantial: what the vendor DTS/SDK says, why the old value
was wrong, what boots now that did not, and issue references
(`openhisilicon#104`, `widgetii#115`). Match that depth — the commit log is the
project's design record for register-level decisions.
