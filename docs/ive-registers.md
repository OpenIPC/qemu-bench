# HiSilicon IVE Register Map (Hi3516EV300)

Register dump captured from live hardware during motion detection.
Board: EV300 (`IVG85HG50PYA-S.dlab.doty.ru`), OpenIPC firmware.
Method: `/dev/mem` mmap of 0x11320000 (64KB), polling during `HI_IVS_MD_Process()`.

This capture reads the `0x0100` parameter window during motion detection. Note
that the working clean-room driver (`OpenIPC/openhisilicon` `kernel/ive_neo`)
configures operations through **DRAM task-node descriptors** kicked at `0x00`
(fire) / `0x10` (node chain), not by writing this `0x0100` block — see
[`xnn-registers.md`](xnn-registers.md), which covers the XNN dispatch model and
where the full conv/fc decode lives.

## Hardware Details

| Property | Value |
|----------|-------|
| Base address (EV300) | 0x11320000 |
| Base address (CV500) | 0x11230000 |
| Region size | 0x10000 (64 KB) |
| IRQ (EV300) | GIC SPI 51 |
| IRQ (CV500) | GIC SPI 37 |
| DT compatible | `hisilicon,hisi-ive` |
| Kernel module | `hi3516ev200_ive.ko` (261 KB) |
| Access width | 32-bit |

## Register Map (from live capture, 554 changes over 5 seconds)

### Control/Status Registers (0x0000-0x00FF)

| Offset | Name | Bits | Description |
|--------|------|------|-------------|
| 0x0004 | CTRL | 32 | Control register (idle value: 0x06) |
| 0x0008 | **SW_FIRE** | bit 0 | **Write 1 to start operation, auto-clears to 0** |
| 0x0010 | OP_DESC | 32 | Operation descriptor (alternates between ops) |
| 0x0014 | CMD_READY | 32 | Set to 0x03 before fire, cleared after completion |
| 0x0018 | **CMD_DONE** | bit 0 | **Completion flag: 0=busy, 1=done** |
| 0x001C | OP_PARAMS | 32 | Packed operation parameters (changes per op) |
| 0x002C | CFG0 | 32 | Configuration (changes between DMA/SAD/CCL) |
| 0x0030 | PERF_CNT0 | 32 | Performance counter (always incrementing) |
| 0x0034 | STATIC_CFG | 32 | Static config (idle: 0x00313307) |
| 0x0038 | PERF_CNT1 | 32 | Performance counter |
| 0x003C | STATUS | 32 | Status (idle: 0x0000F000, active: varies) |
| 0x0040 | FRAME_CNT | 32 | Frame counter (increments 0→1→2→0→1→2...) |
| 0x0044 | TOTAL_OPS | 32 | Total operations counter (monotonic) |
| 0x0048 | OP_ACTIVE | bit 0 | 1 when operation in progress, 0 when idle |
| 0x0054 | IRQ_CFG | 32 | Interrupt config (idle: 0x00003F07) |
| 0x005C | BLEND_WEIGHTS | 32 | Weighted add X/Y (e.g., 0x00400040 = 50%/50%) |
| 0x0060 | IRQ_MASK | 32 | Interrupt mask (idle: 0xFFFFFFFF = all masked) |
| 0x0064 | SAD_PARAMS | 32 | SAD threshold/mode packed value |
| 0x0068-0x007C | PERF[0-5] | 32×6 | Performance/timing counters (always changing) |
| 0x0080 | HW_ID | 32 | Hardware ID (0x11E1A300) |
| 0x0084 | HW_VER0 | 32 | Version (0x01) |
| 0x0088 | HW_VER1 | 32 | Version (0x01) |
| 0x0090 | HW_CAP | 32 | Capability register (0x01AB5159) |

### Command Parameter Registers (0x0100-0x01FF)

Written before each fire; define the operation to execute.

| Offset | Name | Bits | Description |
|--------|------|------|-------------|
| 0x0104 | **OP_TYPE** | 8 | **Operation type: 0=DMA, 1=SAD, 2=CCL** |
| 0x0108 | DIMENSIONS | 32 | Packed width/height (e.g., 0x00170000, 0x002E0400) |
| 0x010C | BLOCK_CFG | 32 | Block size / CCL config (e.g., 0xC8 for CCL) |
| 0x0110 | **SRC1_ADDR** | 32 | Source image 1 physical address |
| 0x0114 | **DST1_ADDR** | 32 | Destination buffer 1 physical address |
| 0x0118 | SRC2_ADDR | 32 | Source image 2 (for SAD: reference frame) |
| 0x011C | DST2_ADDR | 32 | Destination buffer 2 |
| 0x0128 | STRIDES_0 | 32 | Packed src1_stride / dst1_stride |
| 0x012C | STRIDES_1 | 32 | Packed src2_stride or dst_stride |
| 0x0130 | STRIDES_2 | 32 | Additional stride |
| 0x0134 | STRIDES_3 | 32 | Additional stride |
| 0x0138 | THRESH_VAL | 32 | Threshold value (e.g., 0x0000FF00 for binary) |
| 0x0160 | CCL_CFG0 | 32 | CCL area threshold (e.g., 0x00100000) |
| 0x0164 | CCL_CFG1 | 32 | CCL mode (e.g., 0x04 = 4-connected) |

### Interrupt Registers (0x0258-0x025C)

| Offset | Name | Bits | Description |
|--------|------|------|-------------|
| 0x0258 | IRQ_STATIC | 32 | Static IRQ config (0x70000000) |
| 0x025C | **IRQ_STATUS** | bit 24 | **Frame done interrupt: pulses 0x01000000** |

### Result Registers (0x0300-0x0308)

Written by hardware after operation completes.

| Offset | Name | Bits | Description |
|--------|------|------|-------------|
| 0x0300 | RESULT0 | 32 | Output result word 0 (varies per op type) |
| 0x0304 | RESULT1 | 32 | Output result word 1 |
| 0x0308 | RESULT2 | 32 | Output result word 2 |

### Extended Operation Registers (0x0400-0x0484)

Used for SAD coefficients, CCL parameters, and filter masks.

| Offset | Name | Bits | Description |
|--------|------|------|-------------|
| 0x0400 | EXT_PARAM0 | 32 | Extended parameter 0 |
| 0x0404 | EXT_PARAM1 | 32 | Extended parameter 1 |
| 0x040C | EXT_CTRL | 32 | Extended control (0x08 when active) |
| 0x0414 | EXT_COEFF0 | 32 | Coefficient / threshold packed |
| 0x0418 | EXT_COEFF1 | 32 | Additional coefficients |
| 0x0484 | EXT_MODE | 32 | Mode / sub-operation config |

## Operation Patterns (from captured sequences)

### Pattern 1: DMA Copy (op_type=0)

```
1. Write 0x0104 = 0x00000000        (OP_TYPE = DMA)
2. Write 0x0108 = 0x00170000        (dimensions)
3. Write 0x0110 = src_phys_addr     (source)
4. Write 0x0114 = dst_phys_addr     (destination)
5. Write 0x011C = dst2_phys_addr    (destination 2, for planar)
6. Write 0x0128 = strides packed
7. Write 0x0160 = 0x00100000        (DMA config)
8. Write 0x0164 = 0x00000004
9. Write 0x0008 = 0x00000001        (SW_FIRE)
   → HW clears 0x0008, sets 0x0018=1, pulses 0x025C
```

### Pattern 2: SAD (op_type=1)

```
1. Write 0x0104 = 0x00000001        (OP_TYPE = SAD)
2. Write 0x0108 = 0x002E0400        (width=0x2E0=736, height=0x400? or packed)
3. Write 0x010C = 0x000000C8        (threshold = 200)
4. Write 0x0110 = src1_addr         (current frame)
5. Write 0x0114 = dst_addr          (SAD output)
6. Write 0x0118 = src2_addr         (reference frame)
7. Write 0x0128 = strides
8. Write 0x012C = strides
9. Write 0x0138 = 0x0000FF00        (binary threshold output)
10. Write 0x0008 = 0x00000001       (SW_FIRE)
    → HW processes SAD, writes result to dst_addr
```

### Pattern 3: CCL (op_type=2)

```
1. Write 0x0104 = 0x00000002        (OP_TYPE = CCL)
2. Write 0x0108 = 0x002E0400        (same dimensions as SAD output)
3. Write 0x0110 = src_addr          (binary image from SAD)
4. Write 0x0114 = dst_addr          (IVE_CCBLOB_S output)
5. Write 0x040C = 0x00000008        (CCL enable)
6. Write 0x0414 = coeff             (area threshold, connectivity)
7. Write 0x0484 = mode              (CCL mode config)
8. Write 0x0008 = 0x00000001        (SW_FIRE)
   → HW runs CCL, writes IVE_CCBLOB_S to dst_addr
```

### Pattern 4: Weighted Add (for background model update)

Observed between SAD and CCL — blends current frame into background model:
```
Write 0x005C = 0x00400040           (weight X=64/256, Y=64/256 → ~25% blend)
```

## MD Processing Cycle (one complete frame)

Each motion detection frame triggers this sequence of IVE operations:

1. **DMA** — copy video frame (YUV→grayscale) to IVE buffer
2. **Weighted Add** — blend current into background model (25%/75%)
3. **SAD** — compute 4×4 block differences between current and reference
4. **Threshold** — binarize SAD output (part of SAD operation with `0x0138=0xFF00`)
5. **CCL** — find connected components in binary mask → `IVE_CCBLOB_S`

Operations 3+4 appear combined: the SAD hardware does threshold internally
when `0x0138` is set (SAD_OUT_CTRL_THRESH mode from the SDK).

## Cross-Reference with Sigmastar

| HiSilicon Offset | Sigmastar Concept | Notes |
|---|---|---|
| 0x0008 | Bank0 reg00 (sw_fire) | Same concept, different offset |
| 0x0018 | Bank0 reg13? (completion) | HiSi uses dedicated done flag |
| 0x025C | Bank0 reg15 (irq_clr)? | HiSi uses bit 24 for frame_done |
| 0x0104 | Bank1 reg04 (op_type) | Same concept at different base |
| 0x0110-0x011C | Bank1 reg08-0F (addresses) | Same layout shifted |
| 0x0128-0x0134 | Bank1 reg14-19 (strides) | Same concept |
| 0x0138 | Bank1 reg28 (threshold) | Threshold value |

**Key difference:** HiSilicon uses a flat register space (no separate bank bases),
with command params starting at 0x0100 instead of Sigmastar's separate bank1 base.

## Idle State Registers (power-on defaults)

```
[0x0004] = 0x00000006    CTRL
[0x0034] = 0x00313307    STATIC_CFG (hardware version?)
[0x0054] = 0x00003F07    IRQ_CFG
[0x0060] = 0xFFFFFFFF    IRQ_MASK (all interrupts masked)
[0x0080] = 0x11E1A300    HW_ID
[0x0084] = 0x00000001    HW_VER0
[0x0088] = 0x00000001    HW_VER1
[0x0090] = 0x01AB5159    HW_CAP
[0x0300] = 0x00100000    RESULT0 (default)
```

## V5 (Hi3516CV608 / CV610): IVE as SVP NPU tasks

On V5 the IVE is not a block of its own. Each IVE op is a 576-byte task
that `ive.ko` builds in DDR and hands to the SVP NPU's hardware task
scheduler (HWTS). The scheduler runs it on the AI core with an NPU model
compiled into `ive.ko`. `hisi-ive-npu` models the scheduler's side and
computes each op in C. Source: SDK V1.0.2.0 `out/obj/mod_ive.o`
(unstripped) and `hi3516cv610.dtsi`.

### Hardware

| Property | Value |
|---|---|
| Base | 0x14000000; the driver maps 64 KiB |
| DT | `ive@0x14000000`, SPI 76, sharing the 8 MiB window with `npu@0x14000000` (SPI 75) |
| Interrupt groups | group 1 is IVE's (SPI 76), group 0 `svp_npu`'s (SPI 75) |
| Locks | the driver's "ear lock" is a call into the sys module, not hardware |

### Registers

| Offset | Meaning |
|---|---|
| 0x0100 / 0x0104 | AI core interrupt status / clear (the driver acts only if `& 0x100001`) |
| 0x80a0 bit 10 | AI core working |
| 0x8108 bit 0 / 0x8110 bit 0 | AI core soft reset / its ack |
| 0x8200 / 0x82b0 | interrupt mask, group 0 / 1; a set bit masks (`ive.ko` writes 0x332, finish unmasked; 0x1ff at close) |
| 0x8204 / 0x82b4 | interrupt status; bit 0: a stream finished tasks |
| 0x8210 / 0x82c0 | finished streams, one bit each |
| 0x8218 / 0x82c8, 0x821c / 0x82cc | timed-out / failed streams |
| 0x8220 / 0x82d0 | stream stopped, one bit each |
| 0x8228 / 0x82d8 | stream pre-stopped, one bit each |
| 0x8280 | read: allocate a stream |
| 0x8500 + 0x40·s | stream config; bit 5 selects the interrupt group |
| +0x08 | task ring base \| 1 |
| +0x10 | control: [11:8] tasks pushed (at most 15 per write), bit 2 pre-stop, bit 1 stop, bit 0 free |
| +0x14 | write: clear the stream's finish (`ive.ko` writes 31) |
| +0x18 | SQE count [11:0], CQE count [27:16] |
| +0x20 | the last finished task's id [15:0] |

Stopping a stream (`hal_ive_npu_reset_stream`) is a pre-stop polled on
0x82d8, then a stop polled on 0x82d0. Both fail as `ive stream[n] pre stop
failure` / `stop failure` when the poll times out.

### Tasks

The ring has 20-512 slots (a module parameter); `ive.ko` keeps two, ping
and pong. The tasks link to each other, so the scheduler follows the links
and never needs the ring's size:

| Offset | Field |
|---|---|
| +0 | header (0x4013 flags) |
| +6 | the task's id (its slot) |
| +8 | the next task's address; the last slot points at the first |
| +64.. | image addresses |
| +104.. | their strides |
| +288 | the op code |
| +292 / +296 | width / height |
| +300.. | the op's own fields |

Op codes at +288 (`ive_fill_npu_*_task`); the NPU models are shared
between ops, so dispatch is on this code alone:

| Code | Op | Code | Op | Code | Op |
|---|---|---|---|---|---|
| 0 | dma | 12 | hist | 23 | lbp |
| 1 | filter | 13 / 14 | thresh_s16 / _u16 | 24 | norm_grad |
| 2 | csc | 15 | 16bit_to_8bit | 25 | st_box_flt_and_eigen |
| 3 | sobel | 16 | ord_stat_filter | 26 | candidate_corner |
| 4 | mag_and_ang | 17 | map | 27 | sad |
| 5 / 6 | dilate / erode | 18 | add | 28 | lk_optical_flow_pyr |
| 7 | thresh | 19 | xor | 30 | gmm2 |
| 8 | and | 20 | ncc | 32 | acc |
| 9 | sub | 21 | ccl | 33 | image_dot |
| 10 | or | 22 | canny_hys_edge | 34 | alpha_blending |
| 11 | integ | | | | |

Modelled so far, with their fields. Their pixel work is in
`hw/misc/hisi-ive-ops.c`, which V4's `hisi-ive` shares where its
semantics are the same:

| Op | Fields |
|---|---|
| dma | +80 / +104 source and stride, +96 / +112 destination; mode +308 (direct, interval with +336 / +340 / +344, set 3 / 8 bytes from +320) |
| sad | +64 / +72 sources, +104 / +108 strides; +88 / +96 outputs, +116 / +120 byte strides; block +308 (4, 8, 16), output control +312, threshold +316, min / max +320 / +324 |
| ccl | +64 image (labelled in place), +88 stride, +80 `ot_ive_ccblob`; 8-connected if +300 is 1, area threshold +304, step +308 |
| add / sub | +80 / +88 sources, +104 destination, +112 / +116 / +124 strides; add's Q0.16 weights at +300 / +304, sub's mode at +300 |
| hist | +80 source, +104 stride, +96 the 256 u32 bins |
| and / or / xor | add's layout |
| thresh | +80 / +104 source, +96 / +112 destination; mode +300 (8 modes), low / high +304 / +308, min / mid / max +312 / +316 / +320 |
| thresh_s16 / _u16 | the same, a 16-bit source (stride in bytes); s16 modes 0 / 1 give S8 |
| 16bit_to_8bit | +80 / +104 source, +96 / +112 destination; mode +300, bias +304, num / den as the driver's rounded Q16 ratio +308 |
| map | +80 / +112 source, +88 the table (256 U8, S16 or U16), +104 / +124 destination; mode +300 |
| integ | +80 / +104 source, +96 / +112 destination; out_ctrl +300: 1 u32 sums, 2 u64 squares, 0 both in a u64 (sum [27:0]) |
| ncc | add's sources, +104 `ot_ive_ncc_dst_mem` (Σab, Σa², Σb²) |
| dilate / erode | +64 / +96 source, +80 / +104 destination; the 5x5 mask as 25 bytes of 0 / 1 at +300 |
| filter | +64 / +104 source Y, +88 / +116 destination Y (UV at +72 / +96 when +300 is set); norm +304, kernel w / h +308 / +312, the S8 mask at +332 |
| sobel | +64 / +96 source; 5x5 S8 mask at +300, its transpose for the vertical; out_ctrl +328: both at +80 / +88 (+104 / +108), or one at +80 |
| ord_stat_filter | +80 / +104 source, +96 / +112 destination; mode +300: median, max, min |
| csc | source planes +80 / +88 / +96 and strides +136 / +144, destination +112 / +120 / +128 and +152 / +160; source and destination format codes +300 / +304, mode +308 |
| mag_and_ang | +64 / +96 source; magnitude (U16) +80 / +104, angle +88 / +108 when out_ctrl (+328) is 1; 5x5 S8 mask +300, threshold +332, the mask's range code +336 |
| norm_grad | +64 / +96 source, 5x5 S8 mask +300, norm +336; out_ctrl +328: 0 both at +80 / +88, 1 or 2 one at +80, 3 both interleaved as S8C2 |
| lbp | +80 / +104 source, +96 / +112 destination; mode +300, threshold +304 (S8 in normal mode) |
| acc | +80 / +88 sources, +104 destination, strides +112 / +116 / +124; mode +300 |
| image_dot | acc's layout with S32 pictures, so the strides are already in bytes |
| alpha_blending | +80 / +88 / +96 the sources and the alpha, +112 destination, strides +120 / +124 / +128 / +136 |
| resize | the only op with no fields in the task: see below |
| canny_hys_edge | +64 the magnitude (U16, byte stride +96) with the octant angle right after it, +80 the edge picture (+104), +88 the stack of seeds; low / high at +300 / +304 |
| lk_optical_flow_pyr | the pyramids at +64.. (previous) and +96.. (next), level 0 first, strides at +176.. and +192..; +128 the points in, +136 out, +160 the status, +168 the error. The size fields hold 2 and the maximum level, and the level-0 size is at +320 / +324; minimum eigenvalue, eps, iterations and point count at +304, +308, +312, +316 |
| gmm2 | +72 the picture (+132), +64 the per-pixel factor when either mode is per-pixel (+128; else the model's address again), +80 / +120 the model, +96 the match info (+144), +104 the foreground (+148), +112 the background (+152); the control struct's fields in order from +312, the two byte ones at +348 / +352 |

Edges are replicated.

### The ops' arithmetic, from a board

The engine's rounding is not in the driver, so these were fitted against
a Hi3516CV608 running the same SDK (`qemu-boot/ive-v5-capture.c` runs
every op on fixed inputs; the references are in
`qemu/tests/data/ive-v5-cv608-ref.tar.xz` and `--qemu` runs the same
binary under QEMU and compares file by file):

- **mag_and_ang**: the magnitude is `|gh| + |gv|` saturated to U16 and
  zeroed below the threshold; the angle is the gradient's octant --
  `atan2` rounded to the nearest 45 degrees, 0 for +x -- and the
  threshold does not apply to it.
- **norm_grad**: `(g + (1 << (norm - 1))) >> norm` saturated to S8, so
  halves round up.
- **lbp**: bit 7 is the top-left neighbour, then clockwise to bit 0
  (left). Normal mode sets a bit where `neighbour - centre >= thr` with
  `thr` as S8, abs mode where `|neighbour - centre| >= thr`.
- **acc**: mode 0 `a + b`, mode 1 `a - b`, mode 2 `a + (b & 1)`, each
  saturated to U8.
- **image_dot**: `(a * b + 2^29) >> 30` of two S17Q15 pictures.
- **alpha_blending**: `(a * alpha + b * (255 - alpha) + 128) >> 8`.
- **csc**: a Q10 matrix and a per-channel constant, clamped to 16..235
  for the video-range modes (0 / 1 out, 10 / 11 in) and 0..255 for the
  full-range ones (2 / 3, 8 / 9); BT.601 is each pair's even mode. RGB,
  packed or planar, is B, G, R, and YUV420SP's chroma plane is V then U.
  Out of 420 the chroma is replicated; into it each 2x2 block takes its
  top-left pixel's chroma. The matrices are in `hisi-ive-ops.c`; they are
  the standard coefficients divided by 1.164 where the output is video
  range. Modes 4..7 (HSV, LAB) do not exist on V5 -- the MPI refuses
  them with "mode must be in [0, 3] or [8, 12)".
- **canny_hys_edge** takes no picture: the driver runs a mag_and_ang
  task into the ctrl's own `mem` first, so the op reads that buffer --
  the U16 magnitude, then the octant angle immediately after it -- and
  writes the edge classes plus a stack of seeds. The classes are
  **1 for not an edge, 0 for a possible edge and 2 for an edge**, which
  is the opposite way round from what the numbers suggest.
  Suppression keeps a pixel whose magnitude beats its two neighbours
  along the gradient's octant, reading a neighbour outside the picture
  as 0 (not replicated, so border pixels survive): strictly on the side
  that comes earlier in raster order for the axis-aligned octants, and
  strictly on both sides for the diagonals. The low threshold is strict
  (`mag > low`), the high one too. A kept pixel over the high threshold
  is a seed unless the pixel to its left is itself an edge or the pixel
  above it is a seed -- the engine thins the seeds, so a straight edge
  alternates 2, 0, 2, 0 -- and the seeds go on the stack in raster
  order as `ot_svp_point_u16`, with `ot_ive_canny_stack_size` after a
  picture's worth of them. The thinning rule fitted here still puts
  about one seed in a thousand on the other pixel of a pair, which
  changes nothing: `ss_mpi_ive_canny_edge` walks 8-connected from every
  seed, and the edge picture it produces is byte-identical to the
  board's in all 18 captured cases.
- **resize**: `ive_fill_npu_resize_task` puts the parameters in a block
  of its own in DDR, whose address is at task +64 and whose first word is
  the op code 29 and second the number of records. The task's +288, +292
  and +296 stay zero, which is how a resize is told from a dma. Each
  record is 64 bytes, its fields from +16: source and destination
  addresses, their strides, source and destination width and height, and
  a Q27 ratio and start fraction per axis (`(ratio - 2^27) / 2`, negative
  when upscaling, its integer part folded into the source address).
  Per pixel: `p = offset + i * ratio`, the index `p >> 27` clamped to the
  source, the low 27 bits taken to a Q12 weight; the two rows are
  interpolated into a 7-bit intermediate (`+ 16 >> 5`) and the column
  weights applied to that (`>> 17`, summed, `+ 2 >> 2`). Both the
  intermediate width and the asymmetric rounding matter: nothing simpler
  matches the silicon.

- **gmm2** keeps `model_num` Gaussians of 8 bytes per pixel, pixel by
  pixel: the mean and the variance as U9Q7, then the frequency and the
  life as U16. A sample matches the first Gaussian whose squared
  distance passes the sensitivity gate,
  `(sample - mean)^2 <= var * sns_factor << 7` — with the variance in
  U9Q7 that is `|d| <= 2.83 sigma` for the default factor of 8. The
  matched Gaussian then walks towards the sample, and both updates are
  `x -= (x - target) >> 7`, which rounds towards +infinity; the
  variance's target is the squared distance **in whole grey levels,
  rounded up** (`dg = -((-d) >> 7)`, `target = dg * dg << 7`), and it is
  clamped to the control's min and max. Its frequency becomes
  `((freq * freq_reduce_factor) >> 16) + freq_add_factor` and its life
  rises by the life factor, saturating; every other Gaussian only loses
  frequency, `(freq * freq_reduce_factor) >> 16`. With no match, a new
  Gaussian takes the **last** free slot — the weakest if none is free —
  as (sample, `max_var`, `freq_init_val`, 1). A slot with no frequency
  reads its life back as 0xffff.
  The outputs: the foreground flag is 255 while the matched Gaussian's
  life is below `life_threshold`, the background is the mean of the
  frequency leader *before* the update, and the match info is that
  leader's index in the high 7 bits with the match flag in the low one
  (which is what `ss_mpi_ive_gmm2`'s comment means by "Low-1bit is match
  flag, and high-7bits is max freq index"). `freq_threshold` did not
  enter any of the captured cases, and the shift of 7 is `var_rate`'s
  default of 1; the captures do not cover another rate.

- **lk_optical_flow_pyr** is the one op modelled to a tolerance rather
  than exactly. Each level iterates over a **7x7** window: the gradients
  are the Sobel of the previous picture over 8 (the same normalised
  gradient norm_grad writes), the pictures are sampled bilinearly at the
  Q7 position, the 2x2 system is solved by Cramer's rule and the step
  truncated towards minus infinity, and an iteration stops when a step
  moves less than `eps`. The error output is the mean absolute residual
  over the window in Q7. Against a board that places the tracked points
  within **0.0043 pixels on average and 0.03 at worst** (768 components
  over 24 runs; the status matches everywhere, and half the captured
  files are byte-identical), which is the engine's own accumulation and
  division rounding, not a difference in the algorithm. The window size,
  the gradient operator and the solve were each fitted: a 7x7 window
  tracks to 0.016 px where 5x5 and 9x9 are out by 0.4, and the gradient
  scale has to be the Sobel over 8 or the step comes out 8 times too
  small. `min_eig_val_threshold` is compared against the lesser
  eigenvalue of the window's gradient tensor at that scale; no captured
  point is ever rejected, so that scale is plausible rather than
  measured.

All 34 ops are modelled. The last two to arrive were
st_box_flt_and_eigen and candidate_corner:

- **st_cand_corner is a sequence**, not one op: norm_grad (norm 3, both
  gradients interleaved as S8C2) into the ctrl's mem, then
  st_box_flt_and_eigen (op 25) over those gradients, writing a U16 map
  of the corner measure and, right after it, that map's maximum; then
  16bit_to_8bit, a threshold at `quality_level`, a 3x3 maximum filter
  (ord_stat_filter), and finally candidate_corner (op 26). **Op 26 is
  `(a == b && a > 0) ? a : 0`** of the thresholded map and its maximum
  filter -- plain non-maximum suppression -- which reproduces the
  captured candidate picture exactly.

  **The 16bit_to_8bit step is the sequence's hinge, and the driver, not
  the engine, sets it up.** A task's +300 is an *ext work* code naming
  what the driver must do when the task finishes; 36 is op 25's
  (`ive_int_ext_work`, `mod_ive.o` 0x858). So the driver pushes only the
  tasks up to and including op 25 -- the push helper stops counting at
  the first one with an ext-work code -- having mapped the maximum's word
  nocache and kept that pointer in the task at +308. On the finish
  interrupt it reads the maximum and computes 16bit_to_8bit's Q16 ratio,
  the same `(num << 16 | den / 2) / den` clamped to `0xffff` that
  `ive_fill_npu_16bit_to_8bit_task` computes, but with `num` 255 and
  `den` the maximum; it writes that into the *following* task's +308 and
  pushes the remaining four. (Ext work 35 is the other one: it builds
  equalize_hist's cumulative table and rewrites the next task's op code
  to 17.)

  So the scale is **255 / maximum, clamped at 1.0**, and the clamp is
  what decides whether a picture has any corners: a measure peaking at
  255 or less is passed through unstretched, so with `quality_level`
  above that peak nothing survives the threshold. On the CV608 that is
  six of the twelve captured pictures -- maxima 1, 1, 5, 7, 12 and 19
  against a `quality_level` of 25 -- and every one of them gives an empty
  candidate map. Taking the board's own eigen maps as the input, this
  rule reproduces all twelve candidate pictures **to the pixel**, which
  leaves op 25's own last count (below) as the sequence's only residual.
  Scaling by a plain 255 / maximum instead invents corners in those six;
  scaling by 1 instead over-produces in the other six.

  **Op 25 is**

      t1 = det // (16 * trace)
      measure = t1 + 16 * t1^2 // trace

  over the 3x3 box sums of the gradient products (edges replicated),
  clamped to U16, with the picture's maximum written after the map --
  the first iterate of `l <- det / (trace - l)` with its division
  expanded to two terms, over 16. The measure depends on the structure
  tensor through its **trace and determinant only** (exactly rotation
  invariant) and is homogeneous of degree one to within half a percent:
  the slope of `log measure` against `log trace` at a fixed
  `det / trace^2` is 1.004 to 1.008 over trace spans of 3x to 9x, which
  is what a sum of two separately rounded terms looks like. Against a
  CV608 it is **exact on 92.5% of pixels and within one count on 98.0%**
  (138384 pixels of the `strec_*` trace sweep), with a symmetric
  residual -- so the last count is a quantisation inside the engine that
  is still open (#300), not a wrong shape. `lmin` itself is 13.4% out
  and `det / trace` 5.1%, so the engine does neither; nor does it use a
  truncating reciprocal of the trace (for 258 of the 356 traces carrying
  four determinants or more, no single `R` fits
  `floor(det * R * (1 + det * R^2) / 16)`).

  What the captures *do* establish, and what they rule out:

  - The measure is a **function of that 3x3 tensor alone**. The probe
    pictures (cases 10 and 11) are built backwards from a wanted gradient
    sequence -- with `src = F(x) + G(y)` the Sobel's gx is
    `(F(x+1) - F(x-1)) / 2`, so F follows from any gx sequence, odd and
    even columns being independent chains -- and they place triples with
    the same sum and sum of squares next to each other, (1,5,6) against
    (2,3,7) and so on. Each triple is followed by three columns that walk
    the ramp back to where it started, which takes some care because F's
    odd and even columns are separate chains: the plain negation cancels
    neither, and the drift accumulates until the clamp at 0 and 255
    changes the later triples' gradients, so a pair meant to be matched
    is not one. `(t0, t1, t2, -t1, -(t0 + t2), 0)` returns both chains to
    their start while the window at the triple still reads it, and the
    rows are built the same way; the pictures then stay inside 90..134,
    nowhere near a clamp. The tensors are still read back from the
    captured picture rather than assumed, and all 64 compared windows are
    tensor-equal and agree on the measure.
    Without them the point is nearly unevidenced: of the 6144 distinct
    tensors in the other pictures, exactly one with a non-zero measure is
    ever reached by two different gradient patterns.
  - It is **not** a scalar multiple of det/trace, of the lesser
    eigenvalue, of min(A, C), of sqrt(det) or of det/min(A, C): a tensor
    of (89, 54, 40) gives 1 while (180, 137, 150) gives 0, though
    det/trace (4.99 against 24.94) and the lesser eigenvalue (5.20
    against 27.18) both rank the second higher, and both pixels are well
    inside their picture.
  - It is not **any** ratio of a quadratic in (A, C, |B|, the
    discriminant) to a linear form in them, nor a square, square root or
    cube root of one. That is a linear programme over the inequalities
    `lambda <= P/Q < lambda + 1`, which is infeasible for every
    normalisation; the same programme recovers `floor(det/trace/13)` from
    synthetic data, so it is the family that is refuted and not the
    method.
  - Nor is it the lesser eigenvalue computed with a square root of
    reduced precision, a block-floating-point det/trace, a truncated
    reciprocal of the trace, or one to three steps of the secant
    iteration `x <- det / (trace - x)` (that last gets 26% of tensors
    wrong and every one of the 579 with a measure over 100). Nor an
    asymmetric window (1x3 for A and 3x1 for C and so on -- only the
    plain 3x3 is free of contradictions), a discriminant approximated as
    `alpha max(|A - C|, 2|B|) + beta min(...)` for any alpha and beta in
    thirty-seconds, arithmetic with a limited mantissa (5 to 24 bits), a
    linear blend of the two eigenvalues, or the trace times a small
    lookup table in the anisotropy.

  **Why a global fit cannot work, and what to fit instead.** Write
  `a = sqrt((A - C)^2 + 4B^2) / trace` for the anisotropy. Binning the
  measure by `a` shows its spread within a bin growing from 2.6% at
  `a < 0.1` to 85% at `a > 0.9` -- because the lesser eigenvalue there is
  a small difference of large numbers, so the engine's own rounding, in
  whatever form, swamps the result. No closed formula can match those
  samples, which is why every exact fit over all of them is infeasible by
  construction. Fit the well-conditioned ones instead.

  On those (`a <= 0.1`, 3309 samples with a measure of 5 or more), the
  measure is **det/trace times about 0.07805** -- that is 1/12.814, and
  not 5/64 = 0.078125, which is a percent too large and costs 20% of the
  matches. The best scale gets **79%** of them exactly right, and no
  choice of scale, of rounding, or of truncating the division before or
  after the scale does better than about 80%: a second effect of a few
  percent per sample remains. It is also established that the measure
  vanishes as the tensor becomes singular (no sample with
  `det/(AC) < 0.1` ever reaches 5), that it depends on *both*
  eigenvalues -- `(11289, 0, 10800)` gives 431 and `(10809, 0, 10800)`
  gives 421, the same lesser eigenvalue -- and that it is
  rotation-invariant to within the quantisation where it is well
  conditioned.


V5's `ot_ive_ccblob` differs from V4's. It starts with a word of
`cur_area_threshold` [22:0], `label_status` [23] and `rgn_num` [31:24],
then 254 regions of area (u32) and left, right, top, bottom (u16).

### Tests

- `qemu/tests/test-hisi-ive-npu.py` (CI): register level, one ring of
  every modelled op.
Of the 343 files in the capture's reference set, 246 come out of QEMU
byte-identical and 97 do not. What differs, measured against a CV608 on
2026-11-14 (the board itself reproduces the stored set exactly, all 343
files):

- 48 LK files, by design: that op is modelled to a tolerance, within
  0.03 pixels, so its flow and error words differ by up to 4 Q7 units.
- 18 Shi-Tomasi eigen maps (`st_s*_eig`, `strec_tr*_eig`), op 25's own
  residual (#300): 7.5% of pixels, by one count on 5.5% of them and by
  two on the remaining 2%. One `max_eig` follows from it, and so do
  four candidate maps -- but only just: 3, 4, 10 and 31 pixels of 4096,
  and the six pictures whose measure peaks below `quality_level` come
  out empty on both sides, as the clamped ratio above says they must.
- 29 canny_hys_edge files (#308), 2 to 21 pixels of 4096 on the edge
  maps and more of the seed stack, which the thinning's raster order
  amplifies. Not yet characterised.

- `qemu-boot/ive-v5-capture.sh --sdk DIR --board HOST --qemu` (local,
  since the capture links the SDK's MPI libraries): every op through the
  MPI on a board and under QEMU, compared file by file. Without
  `--board` it compares against the stored CV608 outputs. CI cannot run
  it, but it does not guard anything the register test above does not:
  that test computes each op's expected output from the rules this
  capture established, so an op that regresses fails CI, and the capture
  is what establishes or revises a rule.
- `qemu-boot/test-hisi-v5-sample-ive.sh --sdk DIR` (local): the SDK's
  memory test (dma, sub, hist) and its sobel through `ive.ko`; sobel's
  four gradient files must equal numpy's correlation of the same input.
