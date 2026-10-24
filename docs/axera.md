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
cannot give it. AX620Q's much smaller dtsi gets the three it actually
has (`clk_cpu`/`clk_pllc`/`clk_common`).

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

The remaining IP blocks -- ISP (`axera,proton`), NPU (`axera,npu`),
I2C/SPI/watchdog, DWMAC Ethernet -- are named
`create_unimplemented_device` regions at their real dtsi addresses (so
`-d unimp` names the actual block a boot stage polls), with no register
behaviour yet. ISP and NPU have no public driver source to ground a
model in at all (see the licensing/source-availability notes above)
and are not planned the same way as VENC/VDEC were.

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
- **No ISP, VENC/VDEC, or NPU device model.** The onekvm kernel fork has
  open drivers for VIN/CSI2 capture, the VC8000E encoder and a Hantro
  decoder path for AX630C; the camera-sensor ISP 3A pipeline and the NPU
  remain closed vendor code on every chip in the family, with no public
  kernel-driver source anywhere (checked directly: the vendor SDKs'
  `osdrv` driver directories hold only a `Makefile` per subsystem, zero
  `.c` files, confirmed independently by a third-party audit in
  `sipeed/maix_ax620e_sdk` issue #2). Emulating their registers usefully
  would need that source or a register-level trace against real
  silicon.
- **AX620U's spin-table SMP is modelled; AX620Q's is not yet.** Both
  use a vendor spin-table (`secondary-boot-reg`), not PSCI. AX620U
  reuses QEMU's own generic `default_write_secondary()`
  (`hw/arm/boot.c`, the same mechanism `realview.c`/`vexpress.c` use):
  `secondary-boot-reg` at `0x02220300` is a real regbank watched for the
  entry address, confirmed against the real AX620U vendor kernel +
  devicetree -- `-smp 4` brings all 4 CPUs online cleanly. AX620Q's own
  `secondary-boot-reg` address is not yet confirmed from any real
  source, so it still only boots CPU0.
- **No sensor models.** AX630C's real board shape is HDMI-capture
  (NanoKVM Pro); AX620E/AX620U's real deployed boards are Bayer-sensor IP
  cameras (confirmed sensors: Smartsens SC450AI on the AX620E dump,
  OmniVision OS04A10 on the AX620U dump) but no `sensor=` property or I2C
  sensor models exist yet the way the HiSilicon machines have them.
