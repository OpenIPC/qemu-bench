# Axera camera SoCs (`ax630c`, `ax620e`, `ax620q`, `ax620u`)

Axera's camera/KVM-capture dies are Cortex-A53 (AX630C, AX620E) or
Cortex-A7 (AX620Q, AX620U) parts with ordinary ARM infrastructure: a
GIC-400 and the ARM generic timer, unlike the RIU/HAL-indirected families
elsewhere in this tree. What is Axera-specific is purely the register
map, which is why `hw/arm/axera.c` is small compared to `sigmastar.c` or
`novatek.c`.

All four share one GIC-400 at `0x01850000` (distributor at `0x01851000`,
CPU interface at `0x01852000`) and a 24 MHz ARM generic timer with the
same PPI layout (13 secure, 14 non-secure, 11 virtual, 10 hypervisor,
active-low). They differ in CPU count/type, UART0 base, and SMP release
method:

| SoC | CPU | UART0 | SMP release | DRAM (real board) |
|---|---|---|---|---|
| AX630C | 2x Cortex-A53, AArch64 | `0x04880000` | PSCI/SMC | 1 GiB @ `0x40000000` (NanoKVM Pro) |
| AX620E | 2x Cortex-A53, AArch64 | `0x04880000` | PSCI/SMC | 1 GiB @ `0x40000000` (real Xiongmai IVG-A6 camera) |
| AX620Q | 2x Cortex-A7, AArch32 | `0x04880000` | vendor spin-table, `axera,ax620e-smp` | 256 MiB @ `0x40000000` (NanoKVM-Go) |
| AX620U | 4x Cortex-A7, AArch32 | `0x04990000` | vendor spin-table, `axera,ax620x-smp` | ~1 GiB @ `0x40000000` (real deployed camera) |

AX630C and AX620Q both use the SoC compatible string `axera,ax620e` (one
family, different packages); AX620U's root compatible is `axera,ax620x`
-- a related but distinct chip, confirmed from its own real firmware's
devicetree, not an assumption from the name.

Modelled so far, for every machine: the CPUs (for AX630C/AX620E, PSCI
over SMC as the vendor ATF answers it -- the dtsi's `psci` node is
`method = "smc"`; for AX620U, the real vendor spin-table, all 4 CPUs
come online; AX620Q still only boots CPU0, see below), the GIC-400 (no
virtualization frames yet), and UART0 (DesignWare-style, reg-shift 2;
AX620Q's real console is on a separate, fourth NS16550 instance at
`0x06080000`, not UART0 -- confirmed from its own real vendor U-Boot
config, not assumed).

For AX630C/AX620E, the simple syscon-style blocks -- clock controllers
(`clk_cpu`/`clk_pllc`/`clk_common`/`clk_flash`/`clk_periph`/`clk_vpu`/
`clk_mm`; each one's reset-regmap alias shares its address, so only one
entry is mapped per block), pinctrl (both windows), the four GPIO
banks, the thermal sensor and the small per-block syscons -- are
`hisi-regbank` instances (RAM-backed storage, reused from the
HiSilicon family, not Axera-specific): a driver's write-then-read probe
sees what it wrote, which the unimplemented catch-all underneath
cannot give it. AX620Q is the same die: the Maix SDK's 4.19 AX620Q
devicetrees include the same `AX620E.dtsi` (same addresses and GIC
SPIs), so `ax620q` gets the full AX620E peripheral set
(`axera_ax620e_periphs()`, shared with the AArch64 machines), minus the
IP-camera board's SC450AI sensor. Its console is `ttyS3` at
`0x06080000` (SPI 0x79, `serial_hd(0)`); UART0 is the second serial
port. Because `uboot-file=`/`-kernel` skip SPL, a reset hook writes what
SPL leaves in iram0: `boot_mode_info` at `0x700` (`is_sd_boot` set when
`-drive if=sd,index=1` is given, so U-Boot runs `sd_boot`) and
`misc_info` at `0x740` naming a 256 MiB NanoKVM-Go (AX620QF), plus the
board's released boot key on GPIO3.

Three more blocks needed real device-model behaviour, not just storage,
because real boot code polls them expecting a side effect a passive
regbank can never produce: `clk_pllc`'s CPU-PLL/VPLL0 ready bits
(`hw/misc/axera-pllc.c`, needed for BL1's own PLL bring-up), BL1's
generic delay timer at `0x04830000` (`hw/misc/axera-bl1-timer.c`, a
real free-running DesignWare APB down-counter BL1's `udelay()` polls
for elapsed ticks against), and the SPI-attached boot-logo panel at
`spi2@0x06072000` (`hw/misc/axera-dw-spi.c`, a real DesignWare APB SSI
model -- U-Boot's `designware_spi.c` `poll_transfer()` has no timeout,
so a passive regbank hangs the boot dead silently the moment it tries
to draw the logo, on every machine that has this panel node).

EIP-130 crypto (`hw/misc/axera-eip130.c`) is a real, functional model,
not a stub: it implements the guest driver's actual token protocol
(mailbox 2 doorbell, firmware-bootstrap handshake accepted
unconditionally since there is no real firmware image to validate) and
performs real AES ECB/CBC/CTR via QEMU's qcrypto API on the guest's own
DMA buffers, so Linux's dm-crypt/AF_ALG paths through this driver
produce correct ciphertext, not just a satisfied probe.

VIN/CSI2/TDP (`axera-vin`, `axera-mipi-csi`, `axera-mipi-dphy`,
`axera-tdp`) are `hisi-regbank` storage too, upgraded from plain
unimplemented stubs for the same reason as the clock/pinctrl blocks:
the real open driver (`open_vin_capture.c`, `open_vin_csi2.c` in the
onekvm tree) is an unusually large, hardware-bring-up-derived register
protocol -- per-channel WDMA trigger/sequence/address/enable registers,
a clock-mux sequencing dance with "golden" settle values read back off
real silicon, multiple interrupt groups -- modelling real frame DMA
there is a substantial future project of its own, not attempted here;
the regbank upgrade at least lets its many write-then-poll-for-settle
loops see what they wrote instead of spinning to a timeout against
always-zero reads.

VENC (VC8000E), JENC and VDEC are one shared device type,
`hw/misc/axera-vcmd.c` (`axera-vcmd`, instantiated three times at
their own addresses/IRQs), modelling the real VeriSilicon VCMD
front-end protocol confirmed from the open VC8000E driver
(`eswin/vc8000_vcmd_driver.c`): WORK_STATE/CONTROL/INT_STATUS/INT_CTL,
completing from a timer (not synchronously in the register write) for
the same completion-ordering reason `sstar-vpu.c`/`nvt-h26xdec.c` do.
The handshake is real -- probe, submit, wait and complete all succeed,
no hang, no timeout -- but it does not produce real bitstream content:
the engine's actual job is a guest-built linked list of command-buffer
nodes encoding undocumented VeriSilicon microcode (SPS/PPS, rate
control, frame addresses), not a register map recoverable from the
platform glue driver or a devicetree the way EIP-130's token protocol
was. Decoding that command-buffer format is the concrete next step for
real pixels, not a rewrite of this file.

eMMC/SD/SDIO (`emmc@1b40000`/`sd@104e0000`/`sdio@104d0000`, all
`"axera,sdhc","cdns,sd4hc"`) use QEMU's own generic `cadence.sdhci`
model -- a real, already-upstream device (not Axera-specific) that
implements this exact compatible string's register set, not a stub.
Real media is wired too: `axera_attach_sdhci()` plugs a real `sd-card`
or `emmc` device onto each controller from `-drive if=sd,index=N`
(index 0 is the eMMC, matching U-Boot's own `mmc@1B40000: 0` device
numbering); `hw/sd/sd.c` itself needed a small shared-core fix
(`sd_cmd_APP_CMD` now rejects `ACMD41` for `emmc` devices and the
eMMC's own OCR advertises 1.8V, matching how real eMMC silicon ignores
the SD-specific op-cond handshake instead of answering it) for U-Boot's
and BL1's MMC probes to actually reach `sd_ready_state`.
USB2 (`usb2@8000000`, `"snps,dwc3"`) uses QEMU's generic `usb_dwc3`
model the same way; note that upstream's own model does not expose its
internal XHCI IRQ as a sysbus output at all (confirmed against its only
other in-tree user, `hw/arm/xlnx-zynqmp.c`, which wires none either),
so register-level access works but interrupt-driven transfers do not
-- a limitation of the upstream device, not introduced here.

DWMAC Ethernet (`axera-eqos`, `hw/net/axera-eqos.c`), the DesignWare
I2C controllers (`dw-i2c`), the watchdog (`axera-wdt`), and (AX630C/
AX620E only) the on-chip RTC (`axera-rtc`, `hw/rtc/axera-rtc.c`, a real
free-running clock, not a stub) are real, functional device models
now -- not unimplemented stubs. `axera-eqos` is a full Synopsys DWC EQoS/
dwmac-4.10a model (MAC/MTL/DMA register map, real RX/TX descriptor-ring
DMA, QEMU netdev-backed), confirmed live with real end-to-end traffic
on both platforms (DNS/DHCP/ARP/ICMP on AX620E; ARP/ONVIF WS-Discovery
on AX620U) -- see "What is not here yet" below for the one thing it
still can't do. AX620E has two `dw-i2c` buses and two `axera-wdt`
instances (`/dev/watchdog0`/`watchdog1`, both real DT nodes); AX620U
has ten `dw-i2c` controllers wired (matching its real dtsi), with its
real OmniVision OS04A10 sensor (see sensor models, below) on bus 8.
AX620U additionally has a real `cadence.sdhci` eMMC/SD controller (the
same upstream model AX620E/AX630C use) and a passive `xhci-sysbus`
USB3 instance with `hisi-regbank` OTG/device-mode register windows.

ISP (`axera,proton`) and NPU (`axera,npu`) remain
`create_unimplemented_device` regions at their real dtsi addresses (so
`-d unimp` names the actual block a boot stage polls), with no register
behaviour yet -- they have no public driver source to ground a model in
at all (see the licensing/source-availability notes above) and are not
planned the same way as VENC/VDEC were.

## Sources, and how AX620E/AX620U's addresses were confirmed

AX630C and AX620Q come from the
[onekvm/linux-6.18-ax620e](https://github.com/onekvm/linux-6.18-ax620e)
fork's devicetrees (`arch/arm64/boot/dts/axera/ax620e.dtsi` +
`ax630c-nanokvm-pro.dts`; `arch/arm/boot/dts/axera/ax620e.dtsi` +
`ax620q-nanokvm-go.dts`) and `arch/arm/mach-axera/platsmp.c` for the
spin-table mechanism. That is a third-party, clean-room reimplementation
(unaffiliated with Axera), not vendor source.

AX620E and AX620U's addresses were instead confirmed directly from real
deployed camera firmware dumps (a Xiongmai IVG-A6 for AX620E): each ships a `boot/uImage`
that is actually a U-Boot FIT image (LZMA-wrapped on the AX620E dump,
plain on AX620U's). `unlzma` (if needed) then `dumpimage -l` lists the
FIT's `kernel` and `fdt-1` images; `dumpimage -T flat_dt -p 1` extracts
the devicetree blob and `dtc -I dtb -O dts` decompiles it. The result is
the real board's own devicetree -- which is how the GIC/timer/UART
addresses already modelled here were cross-checked against silicon
rather than only a community reverse-engineering effort, and how AX620E's
`isp@2400000 {compatible="axera,proton"}` / `ax_npu@0x3800000
{compatible="axera,npu"}` and AX620U's equivalents at the same bases were
found. The same dumps' prebuilt kernel modules (`ax_npu.ko`,
`ax_proton.ko`, …) also carry an exact `vermagic=` string identifying the
real deployed kernel: 5.15.73 aarch64 for AX620E, 4.19.125 ARMv7 for
AX620U -- the two chips do not share a kernel baseline despite being in
the same family, so do not assume one Axera kernel version covers both.

## Running

There is no machine-built device tree: `-kernel` needs `-dtb` from the
kernel's own tree, as novatek's nt98332 does and for the same reason --
the Axera drivers read almost everything from it, and duplicating that
map in C here would just be a second, driftable source of truth.

```
qemu-system-aarch64 -M ax630c -m 1G -kernel Image -dtb ax630c.dtb \
    -nographic -append 'console=ttyS0,115200n8'
qemu-system-arm -M ax620q -m 256M -kernel zImage -dtb ax620q.dtb \
    -nographic -append 'console=ttyS0,115200n8'
```

`uboot-file=` boots the real, signed vendor U-Boot binary directly at
its own `CONFIG_SYS_TEXT_BASE` (skipping BL1/SPL/ATF/OP-TEE's own
PLL/DDR bring-up, which real silicon already did and QEMU's RAM is
"trained" by construction either way), verified end-to-end on real
binaries for all three machines that have real vendor U-Boot source
available (AX620U has none in the vendor dump):

```
qemu-system-aarch64 -M ax630c,uboot-file=u-boot.bin -m 2G -nographic
qemu-system-arm -M ax620q,uboot-file=u-boot.bin -nographic
```

Both reach the real interactive `AXERA-UBOOT=>` shell prompt: full
console banner, DRAM detection, relocation, MMC/net device probing.
Add `-drive if=sd,index=0,format=raw,file=emmc.img` for real eMMC
media. `flash-file=` (BL1 included, the real boot-ROM-readable path) is
further along but not yet complete -- see "What is not here yet"
below.

## What is not here yet, and why

- **Flash boot (`flash-file=`, BL1 included) is in progress, not yet
  complete.** The vendor boot chain is BootROM → BL1/SPL → ATF (BL31,
  AX630C/AX620E) or a simpler 32-bit boot (AX620Q/AX620U) → U-Boot →
  kernel. BL1 itself now gets past its own PLL bring-up
  (`hw/misc/axera-pllc.c`) and its generic delay timer
  (`hw/misc/axera-bl1-timer.c`), reaching real application code
  (`flash_init()`/`emmc_init()`), but stalls retrying CMD6/SWITCH in its
  own low-level `sdhci_cdns.c` driver -- a Cadence SDHCI
  command-completion/busy-signalling gap distinct from the `hw/sd/sd.c`
  fix that unblocked U-Boot's higher-level MMC core, not yet
  root-caused. `uboot-file=` (above) is the complete, verified
  alternative that skips BL1 entirely.
- **The camera pipeline runs end to end on the real Xiongmai firmware,
  with a picture flowing through it.** Both AX620E (`flash-file=` of an
  IVG-A6 dump) and AX620U (a JFtech IPC_AX620U_A8 dump) boot the vendor
  App and serve H.265 (or H.264) over RTSP
  (`rtsp://<cam>:554/user=admin&password=&channel=1&stream=0.sdp`, sub
  stream `stream=1`; `qemu-boot/axera-rtsp-pull.py` pulls it). The
  picture is the OpenIPC test scene the other camera models use (the
  logo on navy, a dot sliding beneath it), written as the ISP's output
  and carried by the IVPS and encoder models like a real frame:
  - VIN/ISP (`axera-vin`): there is no sensor or ISP model, so the
    ISP's YUV output is painted with the scene. The driver programs the
    output buffers only through CDMA command lists in memory; the model
    walks each list it is handed and picks out the YUV write-DMA
    addresses (AX620E's main output is compressed, so its picture is
    kept in `axera-fbuf` by address). The same lists arm the range-1
    interrupts on AX620E. Frame events at 30 fps; per-SoC `quiet-*`
    props keep the error/frame-loss events silent, `poll-*` props latch
    the RLTM histogram bit the driver polls.
  - IVPS engines (`axera-ivps-engine`): AX620U's TDP copy-scales NV12
    per job; on AX620E the GDC streams the ISP picture into VPP, which
    fills each output port (compressed ports via `axera-fbuf`). DPU
    (OSD) jobs complete without drawing.
  - NPU (`axera-npu`, AX620U): jobs complete without computing anything,
    which is enough for the AI-ISP denoise stage and the App's detection
    models to keep running.
  - VENC (`axera-vcmd`): the VeriSilicon VCMD front end plus a software
    H.264/HEVC/JPEG encoder, run in a worker thread (`AXERA_VCMD_SYNC=1`
    runs it inline). If a source picture is still empty it paints the
    scene itself.
  - AE statistics: written from the painted scene scaled by the
    sensor's programmed exposure x gain (AX620E: ITP AE grid +
    histogram; AX620U: the IFE raw-downscaler frame its App's CPU 3A
    reads), so auto exposure settles on daylight instead of night mode.
  - Audio (`axera-i2s`, Synopsys DW I2S): capture is a 400 Hz tone
    (PCMA in the RTSP stream), playback is drained and discarded.
    AX620E moves it by AX_DMAPER, AX620U by interrupt.
  Pixel jobs (TDP, GDC->VPP, encoder) run in worker threads, off the
  BQL: inline they starved the vCPUs and the flash DMA interrupts.
  Not modelled: OSD drawing (DPU), real ISP image processing.
  AX620E runs both CPUs (spin-table, CPU1 released from BL1's mailbox).
  Guest time vs. a busy host (patch 0007): the firmware's drivers have
  tight guest-time deadlines (SPI-NAND DMA ~200 ms, ax_venc ~1 s, ax_vo
  draw waits). The axera machines default `QEMU_VCLOCK_STARVE=5`, which
  freezes the virtual clock while a runnable vCPU or worker thread has
  been off-CPU for 5 ms (`=0` turns it off, `QEMU_VCLOCK_STARVE_LOG=1`
  logs pauses). It does not cover every case: on a host loaded far
  beyond its cores AX620U can still miss a flash deadline.
  OpenIPC firmware on these machines: the AX620E SPI-NOR model serves a
  synthetic U-Boot environment by default (what the stock Xiongmai
  firmware needs); `-global axera-spi-nor.env-from-flash=on` serves the
  flash file's own env instead, so OpenIPC's U-Boot keeps what
  `saveenv`/`fw_setenv` wrote.
  `QEMU_TLBI_RELAXED=1` (patch 0006) makes broadcast TLB flushes cheaper
  and roughly doubles AX620U's frame rate on a loaded host.
- **AX620U's spin-table SMP is modelled; AX620Q's is not yet.** Both
  use a vendor spin-table (`secondary-boot-reg`), not PSCI. AX620U
  reuses QEMU's own generic `default_write_secondary()`
  (`hw/arm/boot.c`, the same mechanism `realview.c`/`vexpress.c` use):
  `secondary-boot-reg` at `0x02220300` is a real regbank watched for the
  entry address, confirmed against the real AX620U vendor kernel +
  devicetree -- `-smp 4` brings all 4 CPUs online cleanly. AX620Q's own
  `secondary-boot-reg` address is not yet confirmed from any real
  source, so it still only boots CPU0.
- **Sensor models exist for identification, not real pixel data.**
  AX630C's real board shape is HDMI-capture (NanoKVM Pro); AX620E/AX620U's
  real deployed boards are Bayer-sensor IP cameras. Both real sensors are
  now modelled as I2C ID-register stubs, wired to their real I2C bus/
  address: `hisi-smartsens` for AX620E's SmartSens SC450AI, and
  `axera-ov-sensor` (`hw/i2c/axera-ov-sensor.c`) for AX620U's OmniVision
  OS04A10 (slave address 0x36, chip ID 0x53/0x04/0x41 at register
  0x300A, matching the vendor SDK's own `os04a10.c` exactly). Both let
  the real camera app's sensor-probe/ID-read succeed and load its real
  tuning files, but neither delivers actual pixel data; the video path
  downstream runs on the painted test scene (see above).
