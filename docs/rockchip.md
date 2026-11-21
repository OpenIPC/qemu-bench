# Rockchip RV1103 / RV1106 / RV1126 (`rv1103`, `rv1106`, `rv1106g3`,
`rv1103b`, `rv1126`, `rv1109`)

Rockchip's single Cortex-A7 IP-camera SoCs, all in `hw/arm/rockchip.c`.
The first three machines are one die with different in-package DDR.
`rv1103b` is the newer RV1103B / RV1106B die (see below). They boot a camera's
whole SPI NOR image through the vendor SPL, and on stock firmware through
U-Boot, to the vendor 5.10 kernel and its userspace.

## Sources

| What | Where |
|---|---|
| Memory map, interrupts | `arch/arm/boot/dts/rv1106.dtsi`, rockchip-linux/kernel `develop-5.10` (the Luckfox Pico SDK 1.4 tree has the same map) |
| SFC, crypto v3, decompressor, ATAGs, SPL | rockchip-linux/u-boot `next-dev`: `drivers/spi/rockchip_sfc.c`, `drivers/crypto/rockchip/crypto_v2.c`, `drivers/misc/rockchip_decompress.c`, `arch/arm/include/asm/arch-rockchip/rk_atags.h` |
| I2C, GPIO, TRNG, RTC, mailbox, thunder-boot service | rockchip-linux/kernel `develop-5.10`: `drivers/i2c/busses/i2c-rk3x.c`, `drivers/gpio/gpio-rockchip.c`, `drivers/char/hw_random/rockchip-rng.c`, `drivers/rtc/rtc-rockchip.c`, `drivers/mailbox/rockchip-mailbox.c`, `drivers/soc/rockchip/rockchip_thunderboot_service.c` |
| Sensor chip IDs | the same kernel's `drivers/media/i2c/sc2336p.c`, `sc231hai.c`, `sc3336.c`, `sc530ai.c` |
| PLL layout | `drivers/clk/rockchip/clk_rv1106.c` (pll_rk3328 type) |
| Boot behaviour | two Xiongmai camera dumps (below) |


## Machines

| Machine | DDR | Boot tested |
|---|---|---|
| `rv1103` | 64 MiB | —¹ |
| `rv1106` | 128 MiB (RV1106G2) | XM "R5" (stock U-Boot chain), XM "G2" (thunder boot) |
| `rv1106g3` | 256 MiB | —¹ |
| `rv1103b` | 64 MiB | XM "RV1103" (an RV1103B despite its name; kernel 5.10.209) |
| `rv1109` | 1 GiB, 2 cores | an RV1109 SPI NAND camera (OP-TEE 3.6, U-Boot 2017.09, kernel 4.19.111, UBIFS) |
| `rv1126` | 1 GiB, 4 cores | —¹ |

¹ Same die and models as `rv1106`; no image was at hand. `-m` overrides the
size.

### RV1126 / RV1109 (`rv1126`, `rv1109`)

These are four and two Cortex-A7s with peripherals from `0xFE000000` (the
vendor kernel's `rv1126.dtsi`, develop-4.19). They boot from SPI NAND on
the SFC at `0xFFC90000`, through the repo's generic `ingenic-spinand`
(`gd5f2gq4ub`, 256 MiB). The only dump at hand had lost its first 3 MiB,
the ID block with every loader before U-Boot, and its bad blocks. So:

- **Rebuilt flash:** `qemu-boot/rv1126-nand-image.py dump nand.bin`
  rebuilds a flat image.
  - It puts each partition from the dump's backup GPT back at its offset.
  - It leaves the skipped bad blocks inside a UBI partition erased; UBI
    takes them as free.
  - It writes both GPTs and a protective MBR. U-Boot's EFI probe wants the
    MBR first.
- **SPL stand-in (`rk_fit_setup`):**
  - It finds `uboot.img`, a FIT, and loads U-Boot, U-Boot's dtb right
    after it, and OP-TEE.
  - It writes the ATAGs and the boot source (4, SPI NAND).
  - It enters OP-TEE in the secure world as Rockchip's `spl_optee_entry`
    does, with `r1` = `lr` = U-Boot and `r2` = the dtb.
  - OP-TEE then returns to U-Boot non-secure.
- **Dummy clocks:** U-Boot sends the read-from-cache dummy as a third
  address byte; the kernel's rkflash sends 8 dummy cycles. So on this
  machine the SFC sends dummies as bytes (`rk-sfc,dummy-bytes`), and the
  NAND counts one transfer (`dummy-clocks=1`).

What OP-TEE and Linux need from the SoC, beyond the RV1106's models:

- **CRU:** PMUCRU and CRU PLLs, with the fractional dividers as whole-word
  registers (`rk-syscon` `plain-regs`). With hiword writes the denominator
  stays 0, and U-Boot's clock driver divides by zero.
- **PMU handshakes:** power domains and bus idle (`rk-syscon` `mirror`).
  Status follows request at once, or the kernel panics in
  `panic_on_set_idle`.
- **PMU core power** (`0x90` mode, `0x94` request, `0x9c` status), as
  OP-TEE's PSCI drives it:
  - A core in power-down mode loses power at WFI. Patch 0030 adds a WFI
    hook to QEMU's ARM CPU for this.
  - A core powered up, or woken by an interrupt at the GIC's inputs, comes
    out of reset at OP-TEE's trampoline `0xFF710000` in the SRAM (128 KiB).
  - This is how the second core comes up and how cpuidle's `cpu-sleep`
    (power-down, `local-timer-stop`) returns.
- **`rk-timer`:** the rk3288-type timer at `0xFF660000`, the broadcast
  clockevent of those idle states. Without it, the first idle sleeps
  forever.
- **RNG:** the crypto-v2 RNG at `0xFF500400`, inside the crypto block
  (`rk-trng,crypto-v2`).
- **VOP (`rk-vop`, `0xFFB00000`):** the display controller. Out of
  standby it raises frame start and the line flags at 60 Hz
  (`refresh-hz`), but only the enabled interrupts latch. Window 2, the
  primary plane, is scanned out to a QEMU graphic console (ARGB8888,
  RGB888 or RGB565).
  - The scan-out reads through the VOP's IOMMU (`iommu` link, v1
    two-level tables). `MST` is an IOVA, and the driver's IOVAs start at
    0, so 0 is a valid buffer address.
  - Checked: in the RV1109 guest's root shell,
    `modetest -M rockchip -s 56:800x480` gives the SMPTE colour bars on
    the console.
  - Without the frame interrupt, DRM warned "vblank wait timed out".
- **Board I2C devices (RV1109 camera):**
  - The sensor default is the board's IMX415, at `0x1a` on I2C1. The
    shared IMX415 model now also answers `0x311A` with `0xE0`, the chip ID
    Rockchip's driver checks.
  - A PCF8563-compatible RTC, the board's BM8563, is at `0x51` on I2C0
    (`rk-pcf8563.c`, QOM type `pcf8563`). Linux registers it as `rtc0`.
  - The board's TI SN65DSI8x MIPI DSI → LVDS bridge, for its 800×480
    panel, is at `0x2c` on I2C3. It is a 256-byte register file (an
    AT24C) whose ID reads "58ISD   " at 0x00. `/app/mipi_4lane_800x480.sh`
    programs it with `i2cset` at boot; with nothing there it logged 23
    "Error: Write failed". No LVDS is produced.
- **Audio:** the board's ES8311 codec at `0x18` on I2C4 (`rk-es8311.c`,
  QOM type `es8311`) and the I2S/TDM controller at `0xFF800000`
  (`rk-i2s-tdm`).
  - The I2S FIFOs fill (capture) and drain (playback) at a sample clock
    (`word-rate`).
  - The FIFO levels drive the PL330's per-peripheral busy inputs for its
    DMA requests 20 (TX) and 19 (RX).
  - The sound card registers with the codec. Capture data is silence,
    or with `-global rk-i2s-tdm.rx-tone-hz=1000` a 1 kHz sine at a
    quarter of full scale in every channel, at the rate the clocks give.
  - `arecord -D hw:0,0 -d 1 -r 16000 -c 2 -f S16_LE` records that sine:
    16 samples a period, peaking at ±8191.
  - The DMA's eight event interrupts are ORed onto its first line
    (SPI 1), as on the chip. With only event 0 wired, capture stalled
    because its period interrupts never arrived.
  - The FIFO rate is the one the clock registers give. MCLK comes from the
    CRU's `CLKSEL_CON(27)` parent PLL and divider, the `CON(28/29)`
    fraction and the `CON(30)` mux. The I2S then divides it by `CLKDIV`
    (to BCLK) and `CKR` (to LRCK). A FIFO word is one channel, or two
    packed 16-bit samples.
  - A 2 s, 16 kHz `arecord` takes 2.09 s: MCLK is CPLL × 128/15625 =
    4.096 MHz, then divided by 4 and by 64.
- **IOMMUs (`rk-iommu`):** the register interface, with nothing
  translated. The masters behind them aren't modelled. The probe's
  `DTE_ADDR` test and force reset pass.
- **DMA:** QEMU's PL330 at `0xFF4E0000`, with its two interrupt lines
  (SPI 1, 2). The I2S sound card registers with it.
- **USB host:** QEMU's platform EHCI at `0xFFE00000`, with an OHCI
  companion at `0xFFE10000`. The naneng PHYs report Rterm calibration
  done. Attach devices with `-device usb-storage,drive=...,bus=usb-bus.0`.
  The OTG port's DWC3 (`0xFFD00000`, SPI 85) is QEMU's `usb_dwc3`. It is
  a host-only core, so the driver runs it as host. Its xHCI is
  `usb-bus.1`.
- **OTP:** at `0xFF5C0000`, and the secure one OP-TEE reads at
  `0xFF5D0000` (`rk-otp`). It is unprogrammed (zeros), and programming
  only sets bits. OP-TEE checks the status for exact values: 1 after
  `CEB`, then 0xd after `RSTB`.
- **Parallel NAND controller:** at `0xFFC80000`, answering "ready, no
  chip". Otherwise rkflash waits out 100000 polls per command.
- **NPU (`rk-npu`, `0xFFBC0000`, SPI 107):** the Vivante VIP core that
  the SDK's `galcore.ko` (6.4.3) drives.
  - Its identity registers give the RV1126 entry of galcore's built-in
    feature database (chipModel 0x8000, revision 0x8003, product
    0x05080009, eco 0x04000000, customer 0xb5). With zeros, the probe
    fails with "Feature database is not found" and there's no
    `/dev/galcore`; the SDK's mediaserver then dies in `_OpenDevice`.
  - `0x000` reads IDLE_3D / IDLE_2D set, which galcore's soft reset waits
    for.
  - The command front end walks the stream from `0x654` once `0x658`
    bit 16 is set: LOAD_STATE to AQ_EVENT raises that event's interrupt
    bit, END stops, WAIT rests until the next tick, and LINK / CALL /
    RETURN follow. The jobs' NN / TP work isn't done; outputs are
    unwritten.
- **RAM:** the `rv1109` default is 1 GiB. With 512 MiB (256 MiB of it the
  ISP's CMA), the SDK's mediaserver is OOM-killed.
- **Camera pipeline:** the SDK runs IMX415 → CSI-2 → CIF → (memory) →
  ISP20 → ISPP → mediaserver. Its 3A library (rkaiq) takes each raw frame
  from the CIF and queues it to the ISP's raw read DMA ("read back"). All
  of it is modelled. mediaserver's display preview shows the scene that
  `rk-scene.c` draws (the same one as the RV1106's ISP), and its RTSP
  main stream (`rtsp://<board>/live/mainstream`, 4K H.265 at 25 fps)
  decodes to it on the host. rkaiq's AE closes its loop: it moves the
  IMX415's SHR0 and the picture follows.
  ```
  qemu-system-arm -M rv1109,flash-file=nand.bin -display none \
      -nic user,hostfwd=tcp::5554-:554
  ffmpeg -rtsp_transport tcp -i rtsp://127.0.0.1:5554/live/mainstream ...
  ```
  - **IMX415:** it streams while STANDBY (`0x3000` bit 0) is clear. Its
    gain (`0x3090`) is 0 dB out of reset, as on the part. Rockchip's
    driver writes it only when AE changes it, so with the model's
    old 0xFF the exposure overflowed.
  - **CIF (`rk-cif`, `0xFFAE0000`, SPI 58):** MIPI channels 0..3
    (`IDn_CTRL0` bit 0). It gives frame start / end interrupts in
    `INTSTAT` (write 1 to clear), alternating between the FRAME0 and
    FRAME1 buffers, at the sensor's rate (25 fps) while the sensor
    streams. No raw data is written: what the pictures show is drawn
    later.
  - **ISP20:** `rk-isp32` with `readback`, at `0xFFB50000` (SPIs 52 / 54
    / 55), under its IOMMU at `+0x1a00`. A write to `CSI2RX_CTRL0`
    (`0x1c00`) with bit 0 set runs bits 13:12 + 1 passes (two with
    dehaze / tone mapping), each with V_START, FRAME_IN and FRAME. The
    last pass also ends the bridge's path to the ISPP (MI `MPFBC_FRAME`,
    bit 31). Every pass sets the 3A statistics, in registers as the v2x
    driver reads them: it reads them at every FRAME. With statistics only
    on the last pass, rkaiq got an empty buffer for each first pass and
    reported "no aec stats".
  - **ISPP (`rk-ispp`, `0xFFB60000`, SPIs 63 / 64):** the driver starts
    TNR, NR + SHP and FEC per frame through `CTRL_STRT`. Each stage ends
    with its interrupt (TNR / NR + SHP + each enabled scaler / FEC). The
    main picture (`SHARP_WR`, or `FEC_WR`) and the scalers' outputs (at
    their sizes from the 8192-based factors) are the scene, drawn in their
    formats (NV12 / NV16 / YUYV / UYVY, limited or full range) through
    the ISPP's IOMMU.
  - **RGA2 (`rk-rga2`, `0xFFAF0000`, SPI 62):** the command at `CMD_BASE`
    is run on `CMD_CTRL`: bitblt with nearest-neighbour scaling between
    the RGB and YUV formats, and solid fill, through the RGA's own page
    tables. Rotation, blending and ROPs aren't done (straight copy).
    mediaserver's preview is one of its jobs: NV12 to BGR888 for the
    VOP's window 2.
  - **VEPU541:** `rk-vepu540c` with `vepu541`, at `0xFFBB0000` (SPI 68),
    with the RV1126's register offsets and DMA through its IOMMU
    (`+0xf00`). mediaserver's 4K H.265 and the smaller streams encode.
    Its OSD regions (8, at 16×16 block positions, an 8-bit palette index
    per pixel) are blended into the source by the palette's alpha. The
    palette word is Y 7:0, U 15:8, V 23:16, alpha 31:24, the HAL's order
    and not MPP's public `MppEncOSDPltVal`. The stream carries
    mediaserver's date / time, "Camera 01" and logo in its configured
    #fff799.
- **PWM (`rk-pwm`):** three blocks of four at `0xFF430000`, `0xFF440000`
  and `0xFF550000`, with no interrupt in the RV1126's DT. The registers
  read back. Read as zeros, every channel looked disabled, and the SDK
  re-applied its PWMs over and over.
- **Network:** the board's RMII PHY is at MDIO address 0 (the
  machine's `gmac_phy`; the RV1106's integrated one is at 2). With 2
  here, the GMAC found no PHY.
- **VEPU2 JPEG (`rk-vepu2`, `0xFFB90000`, SPI 74, IOMMU at `+0x800`):**
  the HAL writes the JPEG header before the output address (register
  77). The model takes the DQT from it and writes the scan and EOI with
  the shared `jpeg-enc.c`. Register 53 returns the bits written from the
  8-byte-aligned start, as the HAL reads it. Inputs: NV12, I420, YUYV,
  UYVY.
- **NPU MMU:** galcore runs the command front end on GPU addresses once
  it turns on the Vivante MMUv2 (`0x388` bit 0, master TLB at `0x184`,
  loaded by LOAD_STATE). The model walks those tables. A fetch outside
  RAM stops the front end. Before this, one hit the GIC from a timer, and
  QEMU crashed.

```
python3 qemu-boot/rv1126-nand-image.py full.dump nand.bin
bash qemu-boot/test-rv1106-boot.sh --machine rv1109 --flash nand.bin
```

The test passes eight milestones in about 7 s:

1. OP-TEE runs.
2. U-Boot runs, and talks to OP-TEE.
3. The kernel is loaded from the Android boot partition.
4. The second core comes up through PSCI.
5. UBIFS root is mounted.
6. init and its scripts run.

### RV1103B / RV1106B (`rv1103b`)

A newer die with its peripherals at `0x20000000`, from the vendor kernel's
`rv1103b.dtsi` and the U-Boot devicetree inside the dump. It has the same
IP blocks as the RV1106, at other addresses:

- **GIC:** at `0x20411000`.
- **IRAM:** `0x210F6000`.
- **Console:** UART0 at `0x20540000`, 1.5 Mbaud.
- **Flash:** the "FSPI" at `0x20D40000`, which is the SFC under another
  name.
- **Sensors:** on I2C4 at `0x20940000`.
- **SD / SDIO / eMMC:** `0x20D20000`, `0x20650000`, `0x20D30000`.

What differs:

- **Crypto:** v4 at `0x20680000`, with `CRYPTO_VERSION_NEW` 0x04000000.
- **RNG:** `rockchip,rkrng` at `0x20690000`. Its CTRL at 0x10 is
  hiword-masked: SW_DRNG_REQ, then an ack in STATE, then 32 bytes at 0x70.
- **CRU:** only DPLL and GPLL, at TOPCRU `+0x60000`.
- **DMA:** a Rockchip controller (`rockchip,dma`, `rk-dmac`) instead of
  the PL330, at `0x20740000` (16 logical and 8 physical channels). It runs
  the driver's linked lists: memory-to-memory blocks at `mem-rate`
  bytes/µs, and blocks to or from a peripheral at `dev-rate` bytes/s
  (default 192000, 48 kHz stereo audio), so cyclic audio buffers don't
  spin. Blocks to or from the SAI run at the SAI's own rate instead: the
  peripheral is the side whose address doesn't increment. Each block
  writes its position back into the command entry, which is where the
  driver reads the residue.
- **SAI** (`rk-sai`, `0x20660000`): version 0x24031103, `STATUS` idle,
  `CLR` self-clearing.
  - Its frame rate is mclk / MDIV (CKR 14:3) / FW (FSCR 11:0). The mclk
    is the TOPCRU's `mclk_sai_src`: `CLKSEL_CON(35)` 11:10 picks GPLL /
    (`CON(20)` 4:0 + 1), that times the `CON(21)` fraction, or 12 MHz.
  - The DMA's byte rate is that times the channels (SNB × CSR) times
    2 bytes a sample (S16; 1 for S8, 4 above). The XM App captures 8 kHz
    stereo: mclk 2.048 MHz, MDIV 4, FW 64, 32000 B/s. The fixed 192000
    had run it six times fast.
  - Up to 16 bits, two samples share a FIFO word, low half first. The
    32-bit DMA moves the words into the S16 buffer as they are.
  - `-global rk-sai.rx-file=mic.wav` (16-bit PCM, looped; a mono file
    feeds every channel) is what the camera records. `-global
    rk-sai.tx-file=spk.wav` saves playback at the stream's rate and
    channels; the header is kept current every 100 ms and when playback
    stops. Without them, capture is silence.
- **OTP:** at `0x20820000`. It answers the same way as the RV1106's OTP.
- **Camera:** ISP V33 ("rockchip,rv1103b-rkisp", `0x20D00000`, SPI 60 /
  59 / 58), VICAP (`0x20D10000`, SPI 57) and RGA2 (`0x20640000`, SPI 48).
  These are the RV1106's models (`rk-isp32` with `v33`, `rk-vicap`,
  `rk-rga2`).
  - The V33 driver reads its histogram and enhancement IIR rows through
    a handshake: a row ID with `RD_P` (bit 8) written to `HIST_RW` (0x3ca0)
    or `ENH_IIR_RW` (0x3aa8). The model answers `RDATA_VAL` (bit 9) at
    once. Without it every frame logged "hist read:0 timeout".
  - V33 adds the self path: MI_CTRL bit 1, Y base 0x143c, size 0x1440,
    line length 0x144c, CbCr 0x1450 / 0x1454, shadows 0x14a0 / 0x14a8,
    self-update MI_CTRL2 bit 5, auto-update MI_CTRL bit 14, MI frame
    bit 1. Each path is drawn at its own size, the frame scaled to its
    line length and Y size.
  - The encoder ("rockchip,rkv-encoder-rv1103b", `rkvenc_500`,
    `0x20E80000`, SPI 62) is the RV1106's `rk-vepu540c`. The VEPU500
    keeps the VEPU540C's layout: interrupts at 0x20–0x2c, `ENC_START`
    0x10 = 0x100, status from 0x4000. (Read from the annotated
    `mpp_rkvenc_500` and `hal_h265e_vepu500` in a public RV1103B SDK
    build.)
  - The XM App's main stream is online ("set online mode 2", hardware
    DVBM). The driver never writes `ENC_START` for it: it writes the
    frame's registers, then `DVBM_CFG` (0x60) = 0x4000b. Connecting the
    ISP alone writes 0x15, so bits 1:0 together arm the encoder. The
    ISP's mainpath frame end ("dvbm" GPIO) starts the armed job
    ("dvbm-frame").
  - On the RV1103B that start goes through the HPMCU (`rk-hpmcu`). This
    small RISC-V core runs rockit's `hpmcu_wrap.bin`, loaded with
    `rockit.ko en_mcu=1 mcu_fw_addr=0x210f6000`. The firmware polls the
    ISP's line counter (`ISP_DEBUG2`) and feeds the encoder the lines
    written so far (0x18). It writes `ENC_START` itself for the pipe the
    ISP is on (VEPU 0x308 bits 13:12), and posts events through the
    mailbox at `0x20A10000`: B2A_CMD (+0x18) a sequence 12, 13, 14, and
    B2A_DAT (+0x1c) `pipe << 16 | event`, where 4 is the frame end.
    rockit's thread polls B2A_CMD every millisecond, and each new event
    releases `vvi_mb_get`, which hands the wrap buffer on for the next
    frame.
  - The model takes rockit's commands (A2B_DAT then A2B_CMD: 3 wrap start
    with the pipe, 4 end, 7/8/9 the frame the ISP starts as `pipe |
    frame << 2`). At each mainpath frame end it starts an armed encoder
    for the current pipe and posts event 4.
  - `vmcu_probe` gives up ("mcu status is not reset") unless CRU
    `SOFTRST_CON(10)` (0x20000A28) bit 4 holds the core in reset, so the
    machine sets bits 7:4 there.
  - Without the HPMCU model only the first online frame was ever encoded,
    and the App logged "exist nobit ... Reason = 2(enc_stop)" every
    second; that wasn't the XM low-power MCU's `GetMcuPower timeout`.
    The App now streams 1920x1080 H.265 on the main channel (about 11 KB
    P frames) with no nobit and no encoder timeouts.
  - The board has no Ethernet: `stmmac.ko` ships but the App doesn't load
    it, so this dump can't serve RTSP without Wi-Fi.

The boot ROM contract is the RV1106's: the SPL at DRAM 0, the boot source
at IRAM + 0x10, and the ATAGs at 2M − 8K, with SoC 0x1103.

```
qemu-system-arm -M rv1106,flash-file=dump.bin -nographic -nic user
```

Machine options besides the flash:

- `sensor=`: the image sensor at 0x30 on I2C4, from the shared catalogue
  (`hw/i2c/image-sensor.c`); default `sc2336p`. The two dumps' devicetrees
  list sc2336p and sc231hai there.
- `sensor2=`: a second sensor at 0x32, for two-sensor boards such as the
  G2; default `none`.
- `gpio-high=`: input pins that idle high, as a pull-up on the board
  holds them, `/`-separated: bank, port, pin (`4C0/0A2`). Any other
  input nothing drives reads 0. The G2's reset button is `4C0` (see
  Xiongmai firmware).
- `gc6160=CLK:DATA`: XM's GC6160 pan / tilt motor and IR-cut driver
  (`XmMotor.ko`, `g_useI2c=3`). It is an I2C device bit-banged on two
  GPIOs, given as Linux GPIO numbers (bank × 32 + pin, the module's
  `gpio_pin`).
  - Its main chip is at 0x26 and the sub chips at 0x20 / 0x22 (8-bit
    addresses). Each is a 256-byte register file (QEMU's `gpio_i2c`
    bridge and three AT24C).
  - The driver releases SDA by turning the pin to an input, so an input
    pin drives the line high, the board's pull-up. Both lines idle high.
  - Without it the driver logs "ircut close fail, retry to close": it
    reads back what it wrote. `70:75` (GPIO2_A6, GPIO2_B3) on the XM
    RV1103B.
- `tb-isp=ADDR:CAMERAS[:STRIDE]`: stands in for the MCU's
  thunder-boot result to the ISP driver (`rkisp_chk_tb_over`). It is
  written at reset into the ISP's reserved memory (its devicetree
  `memory-region`), one block per camera, STRIDE bytes apart (default
  11245, XM's RV1106G2 kernel). Each block holds: enable, complete = OK,
  one frame, 1920×1080, the camera count and index, 10 ms at unity gain
  (Q16), and the AWB1 gains of a daylight white in the ISP parameters
  after the 100-byte head. The G2 takes `tb-isp=0x8b0000:2`.
- `-drive if=sd,index=0` is the SD card, `index=1` an SDIO slot and
  `index=2` an eMMC.
  - The vendor kernel writes with BYTCNT 0: an undefined-length transfer
    (DW MSHC databook), with the IDMAC chain setting the length and a
    CMD12 ending it on the card. The shared MSHC model (`hisi-himci`)
    used to take 0 as nothing to move, so every write was lost.
  - Now the XM R5 App formats a card in its own WFS layout on the first
    boot (768 MiB recordings + 256 MiB snapshots on a 1 GiB image) and
    records to it after that. A blank image is enough; it ignores FAT.

`flash-file=` is the camera's whole SPI NOR. Erases and programs are written
back to it, so boot a copy. The flash is QEMU's m25p80, chosen by image size:
`w25q64` for 8 MiB, `mx25l12805d` for 16 MiB (QEMU has no `w25q128`, and the
vendor table accepts the Macronix ID), `w25q256` for 32 MiB.
`flash-type=` overrides the choice.

## Memory map

| Address | Block | Model | Notes |
|---|---|---|---|
| `0x00000000` | DDR | RAM | the SPL is linked at 0 |
| `0xFF000000` | GRF (PERI_GRF) | `rk-syscon` | reboot mode at +0x20200 |
| `0xFF1C0000` | RTC | `rk-rtc` | host time, IRQ 98 |
| `0xFF1F0000` | GIC-400 | `a15mpcore_priv`, 160 IRQs | dist +0x1000, cpu +0x2000; patch 0029 |
| `0xFF300000` | PMU | `rk-syscon` | |
| `0xFF310000`, `0xFF320000`, `0xFF450000`, `0xFF460000`, `0xFF470000` | I2C0..4 | `rk-i2c` | the sensor on I2C4; `/machine/i2cN` |
| `0xFF380000`, `0xFF530000` + n·0x10000 | GPIO0..4 | `rk-gpio` | `/machine/gpioN` |
| `0xFF388000` / `0xFF538000` | PMUIOC / IOC | `rk-syscon` | |
| `0xFF3A0000` | CRU (TOPCRU at +0x10000) | `rk-syscon` | PLLs out of reset at the loader's rates, LOCK set |
| `0xFF3C0000` | SARADC v2 | `rk-syscon` | channels read 0x3ff: no key pressed |
| `0xFF3C8000` | TSADC | `rk-syscon` | code 530: about 40 °C |
| `0xFF3D0000` | OTP (secure OTP +0x8000) | `rk-syscon` | INT_STATUS reads done |
| `0xFF420000` | DMAC | QEMU's `pl330` | 8 channels, ID repeated at +0x3000 |
| `0xFF440000` | crypto v3 | `rk-crypto` | hash engine only |
| `0xFF448000` | TRNG v1 | `rk-trng` | host entropy |
| `0xFF4A0000` + n·0x10000 | UART0..5 | `serial-mm` + DW extras | console UART2, IRQ 27 |
| `0xFF520000` | hardware decompressor | `rk-decom` | gzip, zlib, LZ4 frames |
| `0xFF5A0000` | watchdog | `fh-wdt` (DW APB) | fixed TOPs, 24 MHz |
| `0xFF5C0000` | mailbox to the MCU | `rk-mailbox` | "MCU done" stand-in |
| `0xFF6C0000` | system SRAM, 256 KiB | RAM | boot source id at +0x10 |
| `0xFF9A0000`, `0xFFA90000`, `0xFFAA0000` | SDIO, eMMC, SD (DW MSHC 2.70a) | `hisi-himci` | `-drive if=sd,index=1/2/0` |
| `0xFFA80000` | GMAC (dwmac-4.20a) | `axera-eqos` | integrated PHY 0044.1400 at MDIO 2 |
| `0xFFAC0000` | SFC | `rk-sfc` | IRQ 53 |
| `0xFFA00000` | ISP32 | `rk-isp32` | SPIs 105 / 104 / 103 (ISP, MI, MIPI); a scene lit by the sensor's exposure (`sensor` link), NV12 into the mainpath / bypass buffers; AE means and histograms of that scene, to registers and to the DDR stats buffer |
| `0xFFA10000` | VICAP | `rk-vicap` | SPI 106; the to-ISP frame start / end interrupts (`GLB_INTST` bits 14 / 20) when the ISP's frames run; `toisp-on` gates them; MIPI ID0 DMA capture frame interrupts (`INTSTAT` +0x78) |
| `0xFFA50000` | VEPU540C | `rk-vepu540c` | SPI 107; H.265 through the shared `hevc-enc.c`, under the HAL's own VPS/SPS/PPS; JPEG scan + EOI through `jpeg-enc.c` with the HAL header's DQT, on the thread pool; H.264 I and P pictures through `hantro-h264.c` under the HAL's SPS/PPS |
| `0xFFB00000` | USB OTG (DWC3) | QEMU's `usb_dwc3`, or `rk-dwc3` with `usb-otg=device` | SPI 54; host: a host-only core (the driver runs it as host whatever `dr_mode` says), its xHCI at the base, `-device usb-*,bus=usb-bus.0`; device: see USB gadget |
| `0xFF350000`, `0xFF360000`, `0xFF490000` | PWM v2 ×3 (four channels each) | `rk-pwm` | SPIs 31 / 33 / 35; counters at 24 MHz, one-shot interrupts |
| `0xFFAE0000` | I2S/TDM | `rk-i2s-tdm` | PL330 requests 22 / 21; fixed `word-rate` (its CRU clock isn't followed) |
| `0xFFFF0000` | boot ROM | ROM | |

Everything else from `0xFF000000` is an unimplemented-device catch-all.

## Boot

There is no mask ROM code to run. A stand-in finds the `RKNS` ID block
(header v2, at flash 0x10000; 0, 0x8000, 0x20000 and 0x40000 are also
tried). Image entries are 0x58 bytes from header + 0x78, each with
`size << 16 | offset` in 512-byte sectors. The first entry loaded at
`0xffffffff` is the DDR-init blob, which is closed and has nothing to train
here, so it is skipped. The next one is the SPL, which is loaded at DRAM 0
and run. On the G2 dump, entry 1 is data at 0x40000, not code.

The stand-in recreates what the ROM and DDR blob leave behind:

- **SRAM + 0x10:** boot source 3 (SPI NOR).
- **Preloader ATAGs at `0x1FE000`** (2 MiB − 8 KiB), each with a hash word
  of 0, meaning "accept":
  - `CORE`
  - `SERIAL`: UART2, 115200
  - `DDR_MEM`: base in `bank[i]`, size in `bank[count + i]`. A size in the
    wrong slot gives U-Boot's "Bidram Error".
  - `SOC_INFO`: `0x1106` with `SOC_FLAGS_TDBT`. Both dumps' DDR blobs set
    this flag. A thunder-boot SPL needs it to boot the kernel it has
    preloaded. Without it, the SPL jumps to U-Boot, which the preloaded
    kernel at 0x208000 and the SPL's 64 KiB environment buffer at 0x21E000
    have already overwritten.

From there the firmware runs unmodified:

| Chain | What happens | Models it needs |
|---|---|---|
| Stock (R5) | SPL → U-Boot FIT, each image sha256-checked → U-Boot 2017.09 → kernel FIT (inside SquashFS on R5) → Linux 5.10.160 | SFC, crypto, ADC, OTP, CRU |
| Thunder boot (G2) | SPL → meta partition, MCU firmware, U-Boot and kernel preloaded (gzip, on the decompressor) → "Jumping to Kernel(0x00208000)" → Linux 5.10.160 → erofs ramdisk root | decompressor too |

## Models

**`rk-sfc`.** The FSPI controller:

- **Registers:** CTRL to DATA, the CS1 copy at +0x200, `VER` (property,
  default 8), and `LEN_EXT` for version ≥ 4.
- **Commands:** a command without an address starts at the CMD write;
  otherwise it starts at the ADDR write.
- **Dummies:** one `ssi_transfer` per dummy cycle.
- **Reads:** buffered from the start of the transfer.
- **Writes:** flushed once every byte has arrived.
- **DMA:** `DMA_TRIGGER` copies the whole transfer.
- **Status:** RISR TRAN_FINISH is bit 4 and DMA bit 7.

**`rk-crypto`.** Crypto v3, hash engine only:

- **Hashing:** a descriptor-list DMA (START / RESTART on DMA_CTL, pause and
  last flags), done with QEMU's `qcrypto_hash`.
- **Algorithms:** SHA1, MD5, SHA256, SHA384 and SHA512, as advertised in
  `HASH_VERSION`. There is no SM3 or SHA-224.
- **Digest:** in `DOUT` at 0x3a0, as big-endian words.

**`rk-decom`.** The hardware decompressor:

- **Formats:** gzip, zlib and raw deflate via zlib, and LZ4 frames
  decoded here. LZ4 checks the header, block and content xxHash32 sums
  that CTRL bits 1..3 ask for. Each bad part has its own ISR bit: header
  1, sequence 3, checksums 13 / 14 / 15.
- **Timing:** jobs take virtual time (`out_len / rate` µs, property `rate`,
  default 200 bytes/µs). The thunder-boot SPL starts the kernel's
  decompression, keeps loading, and polls STAT later, as it does on the
  real chip.

**`rk-syscon`.** Register file for CRU, GRF, PMU, IOC, OTP and SARADC:

- **Writes:** hiword-mask writes, unless the `hiword` property is off.
- **`lock-regs`:** registers whose bit 10 (PLL LOCK) always reads set.
- **`init`:** reset values.
- **`const`:** fixed reads, for status registers the drivers clear by
  writing them, such as OTP's.

**`rk-i2c`.** The RK3x controller, version 0 in CON:

- **TX mode:** the first TXBUF byte after START is the address.
- **Register modes:** MRXADDR, then MRXRADDR's valid register bytes, then a
  restart to read: the kernel's combined write/read in one go.
- **Long reads:** a further 32-byte chunk is plain RX on the open transfer.
- **NACK:** NAKRCV, and the bus is let go.
- **CON1 auto stop:** only used from version 5.
- **Timing:** every transfer completes at the count write.

**`rk-gpio`.** GPIO v2:

- **Writes:** each control register is two hiword-masked halves (pins
  0..15 at the offset, 16..31 at +4).
- **Both-edge pins:** `INT_BOTHEDGE` makes a pin edge-triggered whatever
  `INT_TYPE` says. Linux sets only `INT_BOTHEDGE` for both edges; reading
  that pin as a level interrupt storms the G2's power key.
- **Pins:** qdev GPIOs both ways; `input-levels` sets the inputs at reset.

**`rk-trng`.** TRNG v1 with `VERSION` 0x46bc and host entropy. The kernel
refuses other versions, and its hwrng then retries for 10 s.

**`rk-rtc`.** The BCD calendar on the host clock, with an alarm:

- **Writes:** only the low byte counts (the driver ORs in a key).
- **`STATUS1`:** follows `CTRL.START` at once. The driver polls it and
  gives up after 3 s.
- **Trim test:** done at once, with an exact count, so no compensation.

**`rk-mailbox`.** The mailbox's registers, and a stand-in for the RISC-V
MCU's firmware. On thunder-boot boards the MCU brings the sensor up and
then reports "done" on channel 1 (cmd 0xf00d, data 0xdeadbeef). The
kernel's thunder-boot service waits for that report. Until it comes, the
I2C, ISP and CIF drivers it shares with the MCU keep their interrupts off,
and every sensor access times out. The stand-in posts the message at
reset.

**Reused models.** The rest are the same Synopsys cores modelled for other
families:

- **Watchdog:** the DW APB watchdog `fh-wdt`, with `comp-params-1`
  advertising fixed TOPs. Without them `dw_wdt` gives up.
- **GMAC:** the DWC EQOS `axera-eqos`, with `phy-addr` 2 and `snps-ver`
  0x42. The PHY ID is the same as the Axera board's.
- **SD / SDIO / eMMC:** the DW MSHC `hisi-himci`. Its VERID 270a and data
  window at 0x200 are what `dw_mmc` expects.
- **DMA:** QEMU's PL330. The AMBA bus reads the PrimeCell ID at the top of
  the devicetree's 16 KiB, so the register file repeats there.

**UART.** QEMU's 16550 with regshift 2. A DesignWare extras region at
+0x20 makes `USR` (0x7c) read 0x6 plus RFNE, and RFL report the receive
FIFO. The vendor SPL polls `USR`.

**GIC (patch 0029).** The vendor kernel uses `CONFIG_FIQ_GLUE`, so Linux runs
in the secure world with every interrupt in group 1. It acknowledges through
the aliased CPU-interface registers: `GICC_AIAR` (0x20), `AEOIR` (0x24) and
`AHPPIR` (0x28). QEMU's GIC read those as "bad offset", and the kernel hung
right after "Starting kernel".

## Xiongmai firmware

Xiongmai keeps the kernel console off unless its U-Boot environment says
`xmuart=0`. The environment is at flash 0: 64 KiB with a CRC32 first,
calculated over the rest. Without the variable, everything after "Starting
kernel" / "Jumping to Kernel" is silent, with nothing wrong in the model.
`qemu-boot/test-rv1106-boot.sh` adds the variable to its copy of any image
that carries XM's SPI NOR code (`XmSpiNor`).

No dump's root password is known, so the test stops at getty.

```
bash qemu-boot/test-rv1106-boot.sh --flash dump.bin [--log console.log]
bash qemu-boot/test-rv1106-boot.sh --machine rv1103b --flash rv1103b.bin
```

The test checks these milestones in order:

1. SPL banner
2. SPL reading the NOR
3. "Jumping to Kernel" or "Starting kernel"
4. init running `rcS`
5. getty

The R5 reaches getty in about 3 s, the G2 and the RV1103B in about 34 s.
Most of the time on the G2 and the RV1103B is their SDIO Wi-Fi driver
(`hcc`) waiting 30 s for a card that isn't there.

Messages that are expected, because the hardware isn't modelled:

- the XM app reporting an unknown product, when the dump has no hardware
  ID block (below). The network itself works: with `-nic user`, the R5
  sets 192.168.1.10, answers ARP, and resets connections to closed ports.
- the G2's ISP: "wait thunderboot over timeout"
- on the G2, sometimes, `Yuv_getChnFrame` timeouts on the second camera's
  AI channel (VPSS group 1, channel 1, 640x384), about one every 2 s.
  - It showed in 1 boot of 5. In the others both cameras' detectors ran
    about 600 frames each in 180 s, with rockiva reporting 0 objects.
  - When it happens, the channel's frames keep coming (its RGA buffer
    recycles as often as camera 0's), but the one frame rockiva took
    first is never released.
  - rockit's repeated "AVS ... release mb buf_id(207)" lines are normal
    buffer reuse; the fd printed is stale.
  - Without `tb-isp` the kernel asks VICAP for online capture of both
    sensors, which isn't modelled (mpp's "nobit").
- the G2's empty `mtd` partition (all zeros in the dump) failing to mount
  as JFFS2

### Hardware ID

XM's LibCrypto reads a 1 KiB hardware ID block (magic `D2 D4`) from the
NOR 1 KiB below the environment it uses. On the Rockchip NOR dumps that
environment is at 0xA0000, so the block is at 0x9FC00, not just below
the U-Boot environment at flash 0. A dump without one makes the App
report an unknown product and stop. A block from another camera works
once its product ID is patched to the dump's model: the App then prints
`ID(Encr): 0x0241(IPC_LPG-R4C-WEQ)` on the G2.

The product ID (`/usr/jsons/IPC_PRODUCT_TYPES.json` lists each name's)
comes from two HWID fields: XmService's `(hwinfo(0x0d) & 0x1f) << 8 |
(hwinfo(0x21) & 0xff)`, where LibCrypto's `get_dvr_hwinfo` reads field
0x0d as byte 13 >> 3. The R5 (`IPC_R5-WQ`, 0x401) needs field 0x0d = 4
and model byte 0x01. With the block at 0x9FC00 too, its App prints
`ID(Encr): 0x0401(IPC_R5-WQ)`. Its sensor is an SC231HAI
(`sensor=sc231hai`), and its reset button is GPIO4_C0 like the G2's
(`gpio-high=4C0`). On the first boot the App writes `osmem` to the
environment and reboots; after that its video runs: the mainpath
delivers, with no encoder time-outs. Its AI channel waits on the NPU. Its
RTSP stream works over the GMAC with user networking on the App's
subnet, e.g. `-nic user,net=192.168.1.0/24,host=192.168.1.2,dhcpstart=
192.168.1.15,hostfwd=tcp::18554-192.168.1.15:554` and
`rtsp://127.0.0.1:18554/user=admin&password=&channel=1&stream=0.sdp`.
That gives H.265 2880×1616 at about 12 fps with A-law audio. It shows the
model's scene, neutral, with XM's timestamp and title OSD.

The RV1103B dump (`IPC_LPG-R4C-WEQ-N`) is 0x241 like the G2, so the G2's
block works unchanged. Its `/usr` is cramfs (7-Zip extracts it; the
image's own `fsck.cramfs` reports CRC errors). Boot it with
`-M rv1103b,gpio-high=0A0,sensor2=sc2336p,gc6160=70:75`. The reset button
is GPIO0_A0. The RV1103B's GPIO1 is at `0x20D80000` and GPIO2 at
`0x20840000` (the devicetree's aliases); until the machine followed them,
the board's bank 2 pins answered as bank 1.
The board's SC2337P sits at 0x32 and has the SC2336P's chip ID
(0x9b3a). Both cameras run, through VICAP and the ISP (vir0 online, vir1
read back). The mainpath, self path (800×448) and bypass path
(640×360) all deliver frames to the App.

### Reset button

The G2's App reads its reset button on GPIO4_C0 (`"ResetGpio": [4, 16]`
in `/usr/jsons/LIBXMDVR.json`, XMFV-encoded). The button pulls the pin
low, and the model's undriven input reads 0, so the App sees a held
button. About 10 s into its Wi-Fi wait it reboots ("LowPower reset key
reboot"). `gpio-high=4C0` releases it, and the App then keeps running:

```
qemu-system-arm -M rv1106,flash-file=g2.bin,gpio-high=4C0 -nographic -nic user
```

## USB gadget (`usb-otg=device`)

With `-M rv1106,usb-otg=device` (or `rv1103`, `rv1109`, `rv1126`), the
OTG port's DWC3 is `rk-dwc3` (`hw/usb/rk-dwc3.c`). It's a device-only
core, so the kernel's dwc3 driver runs it as a gadget, and a USB host
built into the model enumerates whatever gadget is bound:

- **Registers:** the DWC3's device side: event ring and count
  (`GEVNT*`), `DCTL` run and soft reset, `DSTS`, and the endpoint
  commands (Start / Update / End Transfer, stall). Transfers are TRB
  lists, with chains, links and CSP short packets.
- **Enumeration:** when the driver sets RUN, the host resets the bus
  (reset, then connect done at high speed) and drives endpoint 0 the way
  the driver's control state machine expects: the SETUP packet, the data
  stage, XferNotReady(STATUS), then the status stage. It reads the
  descriptors, sets address 1 and the configuration.
- **Data:** if the configuration has a CDC ECM function, the host selects
  its data interface's alternate setting 1 and bridges its bulk
  endpoints to `-nic ...,model=usb-gadget`. An RNDIS function instead
  gets REMOTE_NDIS_INITIALIZE and a packet filter (SEND_ENCAPSULATED_COMMAND,
  each answer read back with GET_ENCAPSULATED_RESPONSE), which turns its
  carrier on. Its frames travel in REMOTE_NDIS_PACKET_MSGs.
- **Serial:** a CDC ACM function gets SET_CONTROL_LINE_STATE (DTR, RTS),
  and its bulk pair is the chardev `-global rk-dwc3.chardev=ID`. What
  the gadget writes to `/dev/ttyGS0` comes out there, and what comes in
  there the gadget reads. On OpenIPC's `g_cdc` (ECM + ACM), both work at
  once:
  ```
  -chardev socket,id=gs,host=127.0.0.1,port=17777,server=on,wait=off \
  -global rk-dwc3.chardev=gs
  ```

On OpenIPC (built-in `g_cdc`), `usb0` gets 10.0.2.15 by DHCP from QEMU's
user network. Pings answer, and a 1.7 MB file served over the gadget
arrives with the same md5:

```
qemu-system-arm -M
rv1106,flash-file=nand.bin,flash-type=w25n01gv,usb-otg=device \
    -kernel zImage -dtb rv1106.dtb -append "$(cat cmdline)" \
    -nic user,model=usb-gadget,hostfwd=tcp::18080-:8080
```

The RV1109 SDK's configfs gadget enumerates to "configured" as it
ships, with adb over FunctionFS:

- **adb:** an adb function (interface ff/42/01) is a byte stream on
  `-global rk-dwc3.adb-chardev=ID`. adb's TCP transport carries the same
  messages as its USB one, so an adb client talking to a socket chardev
  reaches the camera's adbd (`shell:` commands answer).
- **One transfer per request:** adbd reads each 24-byte message header on
  its own, so the stream goes out as one transfer per request, no larger
  than the request's first TRB. A transfer that ends mid-chain on a
  partial packet completes the chain as a short packet. Before that, the
  payload bytes landed in the driver's bounce buffer. Switched to
RNDIS (`echo usb_rndis_en > /tmp/.usb_config; /etc/init.d/S50usbdevice
restart`), its usb0 runs and answers pings over
`-nic user,model=usb-gadget,net=10.0.3.0/24` (a net apart from the
GMAC's).

A gotcha: the driver chains a 2-byte bounce TRB after each unaligned OUT
request, with the IOC on the bounce. A short packet has to complete the
chain at that last TRB (CSP). Raising the event on the short TRB instead
delivered every frame one frame late.

## OpenIPC firmware (rv1106, SPI NAND, `-kernel`)

OpenIPC's own RV1106 release boots on the `rv1106` machine to its login
prompt, in about 6 s. Unlike the vendor dumps, its root has no password.

```
python3 qemu-boot/rv1106-openipc-image.py \
    openipc.rv1106-nand-lite.tgz openipc.rv1106-nor-lite.tgz img/
bash qemu-boot/test-rv1106-boot.sh --openipc img/
```

- **Image:** the script builds a 128 MiB data-only NAND image with the
  release's UBI at 1 MiB and partitions given by `mtdparts=spi-nand0:…`.
  The kernel and devicetree (Luckfox Pico Pro) come from the NOR
  release's `zboot.img`, an Android boot image whose second stage is an
  RSCE blob.
- **NAND:** `flash-type=w25n01gv` (any `ingenic-spinand` part) puts the
  SFC in NAND mode on a machine whose dumps use NOR. Mainline Linux's
  SPI NAND driver finds it as "Winbond SPI NAND".
- **Secure boot:** `-kernel` on the RV1103/RV1106 machines starts Linux
  in the secure world, as the vendor U-Boot leaves it, since there's no
  secure firmware. Booted non-secure, every interrupt stays in GIC
  group 0 and the first timer tick never comes.
- **U-Boot environment:** the image carries one at 0x40000 (64 KiB,
  CRC32 first). OpenIPC's `fw_printenv` has no `fw_env.config`, so it
  searches the MTD partitions in order for a valid CRC, mapping each
  one whole. Without an environment it maps the 127 MiB UBI partition
  into 96 MiB of RAM, and `rcS` thrashes forever in `check_mac`.
- **Camera pipeline:** with `sensor=sc3336` (the Luckfox board's),
  majestic runs the whole chain. The vendor modules load, the sensor is
  detected, and the Rockchip camera library starts. `rk-isp32` writes
  its test picture to the mainpath and bypass buffers, and
  `rk-vepu540c` encodes it to H.265 (1698 frames, none lost or timed
  out). A dump of the encoder output decodes in ffmpeg with no errors:
  2304×1296, GOP 5.
  - The kernel MPP (kmpp) writes the VPS/SPS/PPS before BSBS, and the
    model parses them. The PPS sets `weighted_pred_flag`, so P slices
    carry a default `pred_weight_table()` (added to `hevc-enc.c`).
  - The reference for a P frame is the reconstruction the previous job
    wrote at the address this one reads (`rfpw` → `rfpr`).
  - Intra CUs code the AC residual too (`intra_ac`): an 8×8 luma and
    two 4×4 chroma transforms, quantised at the job's QP. A CU whose
    variance is below the quantiser step stays DC-only. Edges and OSD
    text come out sharp: on a checkerboard at QP 22 the error is 26 max
    and 8.3 mean, against 64 and 28 for DC alone.
  - The guest clock stands still while a job encodes
    (`cpu_disable_ticks`), so a heavy picture does not trip mpp's
    timeouts; the job costs only its modelled `job-ns`.
  - JPEG (the snapshot channel, from the mainpath) runs alongside, with
    no frames lost on either path. The HAL writes SOI..SOS, and the model
    writes the scan and EOI with that header's DQT. A 2304×1296 scan
    takes about 11 ms of host time, on QEMU's thread pool.
    `jpeg-enc.c`'s DCT is now AAN, 2.2× faster and byte-identical on
    the benchmark picture.
  - H.264 jobs (`ENC_PIC` standard 0) are coded by the shared H.264
    encoder under the HAL's SPS/PPS, through `h264_enc_frame_ref` (new):
    - an IDR when `SYNT_NAL` says so, an I slice when `SYNT_SLI0`'s type
      is 2, else a P slice with inter coding against the previous
      picture's reconstruction (one reference, a default weight table
      when the PPS weights);
    - the reconstruction follows the `RFPW` → `RFPR` addresses.

    The qtest decodes an IDR, an unchanged P and a shifted P in ffmpeg,
    under libx264's baseline (CAVLC) and main (CABAC) headers. The
    shifted P is 16–19 bytes against a 51–63-byte IDR.
  - Sensor → ISP: the frame is a scene: a grey gradient, colour bars, a
    clipping lamp and a moving box. It's lit by the sensor model's
    exposure (integration lines × gain, read through
    `image_sensor_exposure()`), and the frame period follows the
    sensor's frame length. Setting the SC3336's exposure registers over
    I2C changes the picture: mean Y went from 44 to 98 at 0x3e01 = 0x40.
  - Statistics in memory: the RV1106 driver runs as `ISP_V32`
    (`rd_stats_from_ddr`). The ISP writes `struct isp32_stat` to
    `MI_3A_WR_BASE` (0x197c, latched at frame start): AE means at 0x000,
    0x390, 0x720 and 0xab0, histograms at 0xc00 to 0x1800, and AWB at
    0x2b00. AF (0x1c00) isn't written.
  - White balance: like a real Bayer sensor, the raw response isn't
    neutral. White reads R/G 0.606 and B/G 0.588 (`rk-isp32` properties
    `raw-r` and `raw-b`, per mille), a white on the SC3336 IQ's
    illuminant range. The AWB statistics (`isp32_rawawb_meas_stat` at
    0x2b00) give per zone the mean R, G, B and the white points. Per
    illuminant they give the sums of G/R and G/B over the white points,
    and the white-point count. rkaiq takes sum ÷ count as the gain,
    with no fraction bits; with Q8 sums its gains ran away to 0x7ff.
    White points are the scene's grey blocks above 1/64 of full scale:
    at 1/16 the first, dark frames had too few, AWB settled on its
    default, and smart-run never ran it again. rkaiq writes its result
    to the AWB1 gains (`wbgainApplyPosition(1)`). With them applied,
    majestic's grey comes out at Cb 128 and Cr 126–127, where the
    unbalanced raw gives 117 and 116.
  - Colour matrix (CCM, 0x700, on with bit 0): per output row R, G, B
    the registers hold the matrix minus the identity in Q7 (c0 | c1 << 16,
    c2 | offset << 16). majestic's rows sum to 0, e.g. R (113, -93, -20),
    and shrink as the gain rises. It's applied to the 12-bit levels after
    white balance, before gamma.
  - Gamma out (GOC, 0x900): the IQ's 49-point curve maps each 12-bit
    level to the output. majestic programs CTRL 5 (on, log segments,
    the last segment dense): x at 0–8 by 1, then four steps per
    doubling up to 2048, then steps of 256. The curve is 2.2-gamma
    (y(1) = 93 of 4095). Without GOC the output is linear, as in the
    hardware. With it, majestic's grey gradient reads Y 108 / 152 at
    top and bottom (the fixed square-root curve gave 100 / 145).
  - rkaiq's log: with a shell (`init=/bin/sh`, then the inittab
    sysinit lines), restart majestic with
    `persist_camera_engine_log=0xfffffff4` and its output in a file.
    The AWB lines (`WPNo`, `wbgain_s6`, `processing awb_gain_algo`)
    show what it measured and applied.
  - Frame sync: with the sensor behind VICAP, the camera library takes
    its start-of-frame events from the CSI-2 subdevice. rkcif queues
    them from VICAP's to-ISP frame-start interrupt, which `rk-vicap` now
    raises at each ISP frame start.
  - The ISP's frames now run only while VICAP's to-ISP channel is on and
    the sensor streams (`image_sensor_streaming()`, from SmartSens
    0x0100). That keeps the CSI-2 and ISP sequence counters equal
    (checked with gdb: 465 and 465).
  - The vendor AE loop closes. majestic starts at the SC3336 driver's
    initial exposure (mean Y 44) and settles at mean Y 104 within 2 s.
    Exposure is forced down by hand over I2C, and AE restores it within
    a second. What it took, found with gdb in the stripped
    `librkaiq.so` (via Ghidra string cross-references, hardware
    breakpoints at the library's base + offset):
    1. Frame syncs from VICAP's to-ISP FS interrupt (`rk-vicap`).
    2. The ISP's frames gated on VICAP's channel and the sensor's
       stream, so the frame-sync and statistics counters match.
    3. Every enabled statistics block must report done. With only big
       window 1, AE's pre-process found the AE statistics invalid and
       skipped the algorithm. That meant no `AE_PRE_RES` message, and
       every analyzer group behind it waited, which is why no
       parameters were ever queued. Now all three big windows (0x4400,
       0x4600, 0x4700) and all four histograms report done.
    4. The statistics go to memory at `MI_3A_WR_BASE`, at 0x197c.

  - 3A statistics in registers (the lite ISP's path): the driver reads
    them at each frame end. `rk-isp32` sets `MEAS_DONE` (bit 31 of each block's
    control register, write-one-to-clear) and the block's 3A raw-status
    bit, and supplies mid-grey, neutral values. The camera library's
    "AWB stats invalid" messages stopped (from one per frame to 0).
  - `MI_WR_CTRL2`'s self-update bits latch a path's shadows at once. When
    the driver restarts a path that paused for lack of buffers, it
    enables it, self-updates and disables it again, all within one
    frame. With shadows taken only at frame start, the path stayed off
    and its buffers never came back. That was the intermittent mainpath
    or bypass stall, worst under host load. Since the fix, 8 of 8
    four-at-once runs lose no frames, where the batch before lost frames
    in 4 of 4.
- **RV1103:** the same script builds OpenIPC's RV1103 release (Luckfox
  Pico Plus devicetree), which boots on `-M rv1103` with
  `--machine rv1103 --openipc DIR`. In 64 MiB the kernel's 32 MiB CMA
  doesn't fit, so the image adds `rk_dma_heap_cma=24M` (Rockchip's
  DMA-heap reservation, which the vendor U-Boot passes). With it,
  majestic runs the whole camera pipeline with no frames lost.
- **First login:** OpenIPC asks the camera's owner to set a root
  password and to accept the Majestic EULA in person. The test stops at
  the login prompt.

## Not modelled

RV1103 / RV1106:

- **Capture front end:** VICAP (rkcif, `0xFFA10000`) raises its to-ISP
  frame start / end interrupts. Its two MIPI hosts' ID0 DMA capture
  (0x100 and 0x300) runs at 30 fps with frame start / end interrupts,
  ping-pong between the two buffers. A frame whose turn comes while the
  last frame's end is still unacknowledged is dropped. Both ends pending
  at once (the guest a whole frame late, which emulation can be) give
  `frame_phase` 3. The RV1103B kernel's
  `rkcif_assign_check_buffer_update_toisp` then dereferences a NULL
  buffer in its dual update and panics (3 boots in 8 before the drop,
  none in 8 after). Each frame is written: the scene
  at the host's sensor's exposure (host 0 the sensor at 0x30, host 1
  the one at 0x32). It is BGGR through the same raw response as the
  ISP's. RAW10/RAW12 are MIPI-packed when the line stride (VLW) is short
  of two bytes a pixel; otherwise they are 16-bit. RAW8 is one byte. The
  ISP still renders its own picture on read-back. The CSI-2 host and
  D-PHY are storage.
- **Multi-sensor read-back:** a CSI2RX_CTRL0 write with bit 0 set
  runs one ISP32 pass over the frame in memory, with the mainpath, BP
  and statistics written.
  - Without a thunder-boot result, the G2's XM kernel asks VICAP for
    online capture with two sensors, which it can't run.
  - With `tb-isp=0x8b0000:2` the kernel picks read-back itself. rkaiq
    initialises both cameras from the header (`ffexp:0.01,1,1`,
    `ffwb:1.52,1,1,1.71`), and the first frame goes through VICAP → DDR
    → ISP.
  - The header's ISP parameters enable the statistics modules (RAWAE
    0–3, RAWHIST 0–3, RAWAWB, the AWB gains: `module_ens` 0x7fc0020).
    Zeroed, every 3A block was off on the first frame. rkaiq then never
    ran its algorithms, so it never queued the first parameters that
    start the frame's second pass (`first params for fast`).
  - Each read-back pass needs the mainpath's frame end when the
    mainpath is on. When the driver pauses the path, it soft-updates
    the shadows (`MI_WR_INIT` bit 4, 0x1404) and builds its end mask from
    them. The model now latches the MP and BP shadows on that write.
    Before, stale shadows stalled the ISP after a few frames.
  - With those and the RGA, the G2 App's video runs: both cameras about
    30 fps through read-back, the mainpath at 41 ms a frame, the H.265
    encoders started, and no `nobit` or `processStream` time-outs. Run
    for six minutes, it stays up (no reboot or suspend), at about 7000
    mainpath frames a camera. Its
    object detector fails (`rockiva ... run error -1`): the NPU doesn't
    compute.
- **ISP processing:** of the tone stages only gamma out is applied (no
  DRC, dehaze or 3D LUT). The colour matrix (CCM) is applied without its
  high-luma fade (`alp_y`). White balance and colour processing are
  applied too: the AWB gains (gain0 at 0x538 and
  AWB1 at 0x530, multiplied together, when ISP_CTRL0 bit 7 is set) and
  CPROC at `0x800` (contrast, brightness, saturation, hue). Each grey
  block is a white point for every illuminant: the xy ranges rkaiq
  programs aren't applied, so AWB lands a few per cent short of the
  exact gains. AF statistics aren't produced from the scene.
- **Encoder:** H.265 P slices use the shared encoder's simple inter
  coding (skip, or intra where the picture changed). There's no
  link-table mode, no wrap reference buffers and no ROI. Online (DVBM)
  jobs start at the ISP's mainpath frame end, with the whole frame
  already written; the line-by-line handshake (0x18) isn't modelled.
- **HPMCU** (`rk-hpmcu`, RV1103B mailbox `0x20A10000`): its firmware is
  modelled by what it does, not run. It starts armed online jobs for the
  ISP's current pipe and posts each frame end to rockit. It posts no
  quarter-frame events (1–3), and it ignores AVS and the log buffer.
- **RKVENC-PP** (`rk-vepu-pp`, `0xFFA60000`, SPI 108), the encoder's
  pre-processor for the smart features: jobs complete but nothing is
  computed. The kernel (kmpp `mpp_rkvenc_pp.c`) writes the job, then
  `ENC_STRT` (0x10) = 1, and waits for `INT_STA` (0x2c) bit 0. The model
  sets it after `job-ns` (500 µs). The motion and occlusion results stay
  as they were. As storage, every job timed out: about 360
  "rk_vcodec: ... processing time out" per XM R5 boot.
- **Encoder OSD:** blended into the source before coding. The layout is
  kmpp's `vepu540c_osd_reg` (public in ticklab/rockchip-kmpp):
  - eight regions of nine words from 0x3000: cfg0 (on, format, alpha
    swap, R/B swap, background and foreground alpha); left/top and
    right/bottom in pixels, inclusive; the picture's address; the stride;
  - the RGB→YUV matrix at 0x3120.
  ARGB1555, BGRA8888, BGR888 and BGR565 are drawn; the palette formats
  aren't. XM's App draws its timestamp and "CAM01" title this way, in
  ARGB1555 with both swaps on. majestic leaves the block at zero.
- **RGA:** the RV1106's RGA2 (`0xFF980000`, SPI 87, "rockchip,rga2_core0")
  is `rk-rga2`, the RV1109's model: bitblt and colour fill, with scaling
  and YUV/RGB conversion, no rotation or blending. The 5.10 multi-RGA
  driver drives it as it does the RV1109's. The XM G2 App scales its
  sub-streams and AI frames through it. Without it every job timed out,
  and the App's video pipeline stalled (`nobit`).
- **RKNN NPU** (`rk-rknpu`; RV1106 `0xFF660000` SPI 109, RV1103B
  `0x20F00000` SPI 50): jobs complete but nothing is computed. The rknpu
  driver submits a job: the first task's command stream at PC_DATA_ADDR
  (0x10), the last task's interrupt mask at INT_MASK (0x20), the task
  count in PC_TASK_CONTROL 15:0 (0x30), then a pulse on PC_OP_EN (0x08).
  After `task-ns` per task (200 µs) the model sets INT_STATUS (0x28) to
  the mask; that is what the driver's `rknpu_fuzz_status()` compares.
  The XM Apps' AI jobs then finish instead of timing out.
  - Each task is a stream of 64-bit commands, `target << 48 | value << 16
    | register`, that ends with the PC's op enable (target 0x0081,
    register 0x0008). Addresses in it are offsets from PC_DMA_BASE_ADDR
    (0x34). The PC's 0x0010 points to the next task, and the DPU's 0x4020
    is the output.
  - A task writes only the registers that change, so the model keeps the
    DPU's values across tasks.
  - At the end of a job, every DPU output cube is filled with 0x80, the
    INT8 minimum. The cube is width 0x4030 + 1 × height 0x4034 + 1, in
    16-channel surfaces (0x403c + 1 channels). The surface stride is
    0x4024 in 16-byte units; 0 means the surfaces are packed.
  - A detector's scores then come out as low as they go: rockiva reports
    0 objects. With the outputs left as they were, the R5 decoded 128
    phantom objects a frame. Post-processing them took seconds per frame,
    which starved the AI's VPSS channel (group 0, channel 2, 896×512)
    after its first few frames. The AI now runs about 2 frames a second
    instead of one every 7 s, and its `getChnFrame` no longer times out.
- **RVE**: storage.
- **USB gadget functions other than CDC ECM, RNDIS, ACM and adb:** they
  enumerate, but no mass storage or UVC traffic is bridged.
- **PWM** (`rk-pwm`, `/machine/pwmN`): no waveform. Each channel's level
  is the read-only QOM property `outputN`: the permille of time its pin is
  high, from duty / period and the duty polarity (CTRL bit 3), or the
  inactive level (bit 4) when it is stopped. For example
  `qom-get /machine/pwm0 output1` reads an IR LED's brightness.
- **Audio codec** (the analogue acodec): registers only. The I2S/TDM
  sample clock on the RV1106 comes from the TOPCRU, as on the RV1126:
  - the source is `CLKSEL_CON(17)` TX / `(19)` RX (bit 7 GPLL / CPLL,
    bits 6:2 the divider);
  - the fraction is CON(18) / (20);
  - the MCLK mux is CON(19) / (21) bits 1:0;
  - the I2S's own MCLK→BCLK and BCLK→LRCK dividers follow.
  A rate above 2 M words a second (unprogrammed dividers) falls back to
  the fixed `word-rate`. At the fixed 96 k words a second, the XM R5's
  recorder ran fast and rockit logged "timestamp jump" (141 in a boot);
  now it logs none. The FIFO carries the samples: `-global
  rk-i2s-tdm.rx-file=mic.wav` (16-bit
  PCM, looped; a mono file feeds every channel) is what the camera
  records. `-global rk-i2s-tdm.tx-file=spk.wav` saves what it plays, in
  the TX format (16-bit, TXCR's channels, the clock's frame rate). The
  header is kept current every 100 ms and when playback stops. Without
  either, recordings are silence (or the `rx-tone-hz` tone), and played
  audio goes nowhere. QEMU's own audio backends aren't used: their
  real-time pace doesn't match the guest's virtual clock.
- **RISC-V MCU:** the SPL loads its firmware to `0xFF6FF000` and starts
  it; only its final mailbox message is stood in for.
- **XM firmware:** the G2's SDIO Wi-Fi co-processor (`hichannel`) isn't
  there, so the App has no network. Its video runs with `tb-isp` and
  `sensor2` (see Multi-sensor read-back).

RV1109 / RV1126:

- The SARADC (`0xFF5E0000`, SPI 40) is the RK3399's v1, `rk-saradc-v1`:
  - CTRL (0x08): channel in bits 2:0, power-up (bit 3) starts a
    conversion, bit 6 reports it done (and interrupts with bit 5).
    A conversion is done at once.
  - Every channel reads full scale (`channel-level`), so the volume and
    recovery keys read released.
  The RV1106's constant v2 block had been mapped there, and U-Boot
  logged "Key Error: volume up failed to read saradc".
- AWB and AF statistics from the scene (AWB is neutral). The ISPP's
  params buffers run short now and then ("no free ispp params buffer").
- The VEPU2's H.264 (its JPEG is modelled) and the VDPU decoders.
- rkaiq's "no free ispp params buffer" now and then: its threads run late
  under emulation (the CIF's "frame drop to dummy buf" too). That's not a
  missing model.
- The NPU's computation (only its command front end and events run), so
  mediaserver's face detection drops its frames.
- The microphone is a file: the I2S RX path plays `rk-i2s-tdm.rx-file`
  (or silence, or the `rx-tone-hz` tone), and `tx-file` saves playback.
