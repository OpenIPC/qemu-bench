# A digital twin of a HiSilicon V4 IP camera

Design for running a HiSilicon V4 camera (Hi3516EV200/EV300, Hi3518EV300,
Hi3516DV200 and their Goke rebrands) entirely in emulation, with a world
in front of the lens. The goal is to develop and debug sensor drivers and
the 3A algorithms (AE, AWB) without the camera.

## Where we start

Already in place (`docs/hisilicon-venc.md`):

- **Unmodified vendor firmware runs end to end.** majestic and the SDK's
  `sample_venc` drive the sensor over I2C, VI → VPSS → VEDU, and produce
  decodable H.264 / H.265.
- **The ISP's statistics contract is known**, front end and backend,
  online and offline, from openhisilicon's headers and the driver and
  library objects. The closed `lib_hiae` / `lib_hiawb` close the loop:
  AE settles the exposure, and AWB settles gains that neutralise the light.
- **Runs are deterministic** (`-icount shift=0`), and observable
  (`/proc/umap/isp`, `-global hisi-vi.stats-trace=on`).

What is missing is the world and the physics:

- the scene is eight grey bars computed analytically;
- the sensor is a register file plus an exposure formula;
- the ISP writes statistics but no image: the picture VPSS paints does
  not change with exposure or white balance.

## Architecture

```
            QMP / -global                         gdb, record/replay
                 │                                       │
   ┌─────────────▼────────────┐                          │
   │ hisi-scene (the world)   │  scene image, light level and colour
   │  reflectance grid,       │  (later: timeline, flicker, IR)
   │  per-frame camera state  │◄──────────────┐
   └──────┬──────────────┬────┘               │ exposure, WB gains
          │ radiance      │ display pixels     │ of the current frame
   ┌──────▼─────┐  ┌──────▼──────┐     ┌───────┴──────┐
   │ sensor     │  │ hisi-vpss   │     │ hisi-vi      │
   │ (I2C model)│  │ output      │     │ VI + ISP FE/ │──► STT / registers
   │ exposure,  │──┤ channels    │     │ BE statistics│      │
   │ gain, mode │  └──────┬──────┘     └──────────────┘      ▼
   └────────────┘         ▼                          ISP driver, lib_hiae,
                     hisi-vedu ──► H.264 / H.265      lib_hiawb (guest)
```

The guest is untouched. Everything the twin adds lives behind registers
the vendor stack already drives.

### Layer 1: the scene (`hisi-scene`)

One device per machine, `/machine/scene`. It holds what is in front of
the camera and the camera's per-frame state.

- **Source.**
  - The built-in grey bars, the default; this keeps today's tests.
  - A binary PPM (`P6`, sRGB, decoded to linear).
  - A PFM (linear float, so HDR scenes keep their range).

  Both convert from anything with ImageMagick or ffmpeg and need no
  library in QEMU. Video sources (y4m) come later.
- **Light.** Brightness (`light`, 16-bit raw units a row at ×1 gain for a
  white surface) and colour as the sensor sees it (`light-r`, `light-b`,
  per mille of G).
- **Runtime control.** Every property is settable while the guest runs:
  `qom-set /machine/scene light 1200`.
- **Interfaces.**
  - Consumers ask for the reflectance grid at a resolution they choose;
    it is cached per size and source.
  - `hisi-vi` publishes the frame's exposure level and the white balance
    gains the driver programmed.
  - `hisi-vpss` turns reflectance into display pixels with them.

### Layer 2: the sensor

Today's catalogue models (`hw/i2c/image-sensor.c`) report exposure,
gain and window through `image_sensor_exposure()`. The twin needs more:

- **Timing:** modes, VTS/HTS to frame rate, and when a write takes effect
  (frame N+1 or N+2, group hold). `hisi-vi` already applies exposure one
  frame late; per-sensor latency belongs in the sensor.
- **Readout:** black level, saturation, noise (shot, read, dark), fixed
  pattern, rolling shutter.
- **A datasheet checker:** warnings for writes real silicon punishes:
  - exposure past VTS − margin;
  - gain outside the table;
  - registers written mid-frame without group hold;
  - streaming before the PLL locks.
- **An I2C log stamped with frame and line,** so a driver's writes are
  seen against the sensor's frame.

### Layer 3: the ISP

- **Statistics from pixels (step 1).** The front end and backend compute
  their statistics from the rendered grid, not from formulas: per-sample
  clipping, zones that straddle edges, histograms of a real distribution.
- **Next, the hardware's own definitions:**
  - black level subtraction;
  - the AE skip and crop;
  - zone weights;
  - the AWB white region (`CR_MM` / `CB_MM`, which decide a zone's count).
- **Pixels.** The backend's pipeline, driven by the driver's registers
  and LUTs (their DDR addresses are known, VIPROC 0x6fc onward):
  - BLC → LSC → WB gains → demosaic → CCM → gamma → CSC.

  Step 1 starts with the part that makes the loop visible: exposure, WB
  gains, a fixed gamma and BT.601, so the encoded video follows AE and
  AWB.

The rule for this layer: statistics must be right at the bit level, while
the image only has to be plausible.

### Layer 4: debugging and analysis

- **Scenarios:** scripted changes of scene and light over time (QMP from a
  test, later a timeline file): lights off, sunrise, a backlit window,
  mains flicker.
- **A per-frame record:**
  - the scene and light;
  - the sensor registers in effect;
  - the statistics written;
  - AE / AWB decisions;
  - the frame encoded.

  From it, plot convergence, overshoot and steady-state error.
  `stats-trace` is the first piece.
- **Determinism and debuggers:**
  - `-icount` and QEMU record/replay make a 3A oscillation exactly
    reproducible;
  - gdbstub covers the kernel drivers;
  - gdbserver in the guest covers majestic and the libraries (the SDK's
    `.a` files carry symbols).
- **Fault injection:** I2C NACKs, dropped frames, late interrupts, a
  sensor that ignores group hold.

## Validation against hardware

The twin is trusted once it agrees with a camera:

1. On a real camera, capture raw Bayer and the ISP's statistics block
   under a known light (a grey card, a colour checker, two colour
   temperatures).
2. Feed the same raw to the twin as a scene. The statistics must match bit
   for bit; then compare the AE and AWB trajectories.
3. Calibrate the sensor's colour response (`light-r` / `light-b` per
   illuminant) from the same captures.

## Steps

| Step | What | Where |
|---|---|---|
| 1 | Scene device; statistics from the rendered grid; VPSS paints the scene through exposure, WB and gamma; runtime control over QMP | `hisi-scene.c`, `hisi-vi.c`, `hisi-vpss.c` |
| 2 | Register-accurate SmartSens / GalaxyCore models: timing, write latency, datasheet checker, I2C frame log | `hw/i2c/` |
| 3 | Scenario engine and metrics: timelines, per-frame CSV, convergence checks in CI | tests, `hisi-scene.c` |
| 4 | Differential validation against one real camera | `qemu-boot/`, docs |
| 5 | Backend pixel pipeline from the driver's registers and LUTs | new ISP model |

Each step is one PR, landed before the next starts.

## Step 1 in detail

- **`hisi-scene`** (`qemu/hw/misc/hisi-scene.c`, `include/hw/misc/hisi-scene.h`).
  - A bus-less device created by the HiSilicon machine as `/machine/scene`.
  - Properties (all runtime-settable; `-global hisi-scene.*` at start):
    - `file` (PPM/PFM; empty for the bars);
    - `light`, `light-r`, `light-b`.
  - API:
    - `hisi_scene_grid(scene, w, h)` returns linear reflectance, three
      channels, 0..1 (cached);
    - `hisi_scene_set_frame(scene, level, wb_r, wb_b)` records the
      current frame;
    - `hisi_scene_display()` maps a reflectance pixel to display Y / Cb / Cr.
- **`hisi-vi`** renders the grid at an eighth of the sensor's window in
  each direction. Each grid pixel stands for 8 × 8 sensor pixels, i.e. 4
  of the AE's skip samples. All statistics (histograms, AE / AWB zones,
  globals, offline strips) are computed by summing grid pixels. Its
  `light*` properties move to the scene.
- **`hisi-vpss`** paints each channel from the scene at the channel's size:
  - reflectance × the light × the frame's exposure level, scaled to
    0..1 of the sensor's range;
  - × the WB gains, then gamma 1/2.2, then BT.601 limited range;
  - with the frame counter kept in the top twelfth for the stream checks.
- **Tests.**
  - `test-hisi-isp.py` keeps its expected values: with the bars on a grid
    whose bars and zones align they are unchanged.
  - It also loads a PPM scene and changes the light over QMP while the
    machine runs.
  - majestic's end-to-end tests run unchanged.

## Open questions

- **Which camera to twin and validate first:** an EV300 or GK7205V200
  board on the desk makes step 4 possible early.
- **Speed and fidelity:** statistics at zone resolution and pictures at
  channel size keep TCG near real time; full-resolution Bayer rendering is
  for later and optional.
- **The focus of driver debugging:** kernel-side sensor control or
  user-side 3A tuning. It decides whether step 2 or step 5 comes next.
