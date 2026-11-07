# HiSilicon V4 video encoder (VEDU + JPGE)

`qemu/hw/misc/hisi-vedu.c` models the encoder of the HiMPP V4 parts:
Hi3516EV200, Hi3516EV300, Hi3518EV300, Hi3516DV200 and their Goke rebrands
(GK7205V200 family).

| Block | Base | SPI | Does |
|---|---|---|---|
| VEDU | `0x11410000` | 47 | H.264 and H.265 |
| JPGE | `0x11420000` | 48 | JPEG and MJPEG |

The vendor VENC stack runs on it unmodified. That covers the OpenIPC
release's `open_*.ko` / `gk7205v200_*.ko` modules and `libmpi.so` /
`libgk_api.so`. Pictures sent with `VENC_SendFrame` come back from
`VENC_GetStream` as streams ffmpeg decodes.

On the EV200-class machines (hi3516ev200, hi3516ev300, hi3518ev300,
hi3516dv200 and the Goke rebrands gk7205v200, gk7205v300, gk7202v300,
gk7605v100, gk7201v200), `hisi-vi` and `hisi-vpss` add the camera pipeline
in front of it (see [VI and VPSS](#vi-and-vpss) below). With a 1080p
sensor on the bus, two unmodified programs run from sensor to stream:

- the SDK's own `sample_venc` (mode 1: 1080p H.265 and 640x480 H.264);
- the OpenIPC release's majestic, with HiSilicon's `open_*.ko` or Goke's
  `gk7205v200_*.ko`.

The sensor has to be one the firmware has a config for in `/etc/sensors`.
`sensor=sc2315e` suits HiSilicon's releases. Goke's releases have no
SC2315E config, so use `sensor=sc2335` there. With an unlisted sensor,
majestic stops at "Cannot start SDK", in syslog, before it touches VI.

The contract below was taken from the SDK's driver objects
(`Hi3516EV200_SDK_V1.0.1.2/mpp/obj/hi_vedu.o`, `hi_h264e.o`, `hi_h265e.o`;
the `VEDU_HAL_*`, `JPGE_DRV_*`, `h264e_drv_write_regs`,
`h264e_get_bit_stream`, `h264e_wr_str_buf_sps_pps`,
`h265e_encode_slice_header` functions), and from tracing that driver under
the model with `-global hisi-vedu.trace=on`.

## VEDU

Per picture the driver writes only a few MMIO registers:

| Offset | Register | |
|---|---|---|
| `0x0000` | INTMASK | bit 0: end of picture |
| `0x0004` | INTCLR | write 1s to clear |
| `0x0008` | START | 1 starts the picture |
| `0x0010` | mode | `0x40071116` H.264, `0x400103d2` H.265 (not needed by the model) |
| `0x0014` | PIC_SIZE | `(width - 1) \| (height - 1) << 16` |
| `0x004C` | RESET | reset request |
| `0x0058` | REG_IMAGE | physical address of the register image |
| `0x3000` | INTSTAT | masked: 0 end of picture, 2 buffer full, 3 bits over, 6 timeout |
| `0x3004` | RAWINT | raw; 11 ring full, 12 ring back |
| `0x3010` | BUS_IDLE | bit 0 idle; polled around a reset |

The rest of the configuration is a **register image in DDR**, laid out like
the MMIO space. `h264e_drv_cfg_address` and its kin fill it, and the engine
fetches it at START. Of it:

| Image offset | |
|---|---|
| `0x104` / `0x108` | source luma / chroma; NV21 (`PIXEL_FORMAT_YVU_SEMIPLANAR_420`, V first) |
| `0x160` | source strides, luma \| chroma << 16 |
| `0x12c` | address of the stream ring's write-pointer cell |
| `0x130` | address of its read-pointer cell |
| `0x16c` | stream ring base |
| `0x4a0` | stream ring size |
| `0x4a4` / `0x4a8` | PTS |
| `0x61c` | slice QP (H.264) |
| `0x194`..`0x1a4` | reconstruction / reference (unused: the model keeps its own) |
| `0x404` | H.265 slice header: part-1 bits \| part-2 bits << 16 |
| `0x408` | part 1, left-aligned in the low 16 bits: `no_output_of_prior_pics_flag` (IRAP only) and `slice_pic_parameter_set_id` |
| `0x40c`.. | part 2 as big-endian words: `slice_type` through the QP and deblocking syntax |

The H.265 header leaves out `first_slice_segment_in_pic_flag` and
`byte_alignment()`; the engine adds them, as the model does.

### The stream ring

Each channel has its own ring. It is a sequence of packets, each a 64-byte
header followed by the payload padded to 64 bytes:

| Offset | Size | |
|---|---|---|
| `+0` | u32 | packet length, header included |
| `+4` | u32 | padding bytes at the end of the payload |
| `+8` | u8 | type: `H264E_NALU_*` (1 P, 5 IDR, 6 SEI, 7 SPS, 8 PPS), `H265E_NALU_*` (1, 19, 32-34, 39), `JPEGE_PACK_*` (5 scan, 8 header) |
| `+11` | u8 | bit 0 last slice of the picture; bits 7:6 slice type (0 I, 1 P, 2 B) |
| `+12` | u32 | channel; `h264e_get_bit_stream` panics on a mismatch |
| `+16` | u64 | PTS |

The pointers in the cells are byte offsets into the ring.

1. The driver writes its own packets at the write pointer and moves it on:
   - SPS / PPS / SEI ahead of an H.264 IDR;
   - VPS / SPS / PPS / SEI ahead of an H.265 IDR;
   - the whole JFIF header, SOI to SOS, ahead of every JPEG.
2. The engine appends the picture's packet and leaves the write pointer at
   its end.
3. `*_get_bit_stream` walks packets from the read pointer to the last slice
   of the picture.

The model reads the driver's parameter-set packets back out of the ring
(from where it stopped last time up to the write pointer). For H.264 it
writes a slice header that fits them. For H.265 it uses the driver's header
bits. It then codes the slice data:

- **H.264** (`hantro-h264.c`): I_16x16 intra, P with motion search,
  CAVLC or CABAC as the PPS says.
- **H.265** (`hevc-enc.c`): CTB size, SAO, cu_qp_delta depth, TMVP,
  cabac_init and merge candidates from the SPS / PPS and the slice header.

Completion comes 1 ms later in virtual time, never inside the START write.
A picture that does not fit between the write and read pointers is not
written. VEDU then raises end of picture with buffer full (`0x3004` bit 2),
which `h264e_drv_read_regs` reports. JPGE raises error and ring full
(bits 1 and 5), never done.

## JPGE

JPGE is programmed directly over MMIO; there is no register image.

| Offset | |
|---|---|
| `0x000` | status: 0 done, 1 error, 5 ring full, 7 ring back |
| `0x004` | interrupt enable (3) |
| `0x008` | raw status |
| `0x00c` | clear (the driver writes -1) |
| `0x02c` | START: 0, then 1 |
| `0x034` | `(width - 1) \| (height - 1) << 16` |
| `0x060` | stream ring size |
| `0x068` | reset request (bit 0) |
| `0x164` | strides, luma \| chroma << 16 |
| `0x200` / `0x204` | source luma / chroma (NV21) |
| `0x210` | stream ring base |
| `0x214` / `0x218` | addresses of the read / write pointer cells |
| `0x800` / `0x900` / `0xa00` | Y / Cb / Cr quantisers as `2^23 / q`; register i is row `i % 8`, column `i / 8` |
| `0xc0c` | bus idle (bit 0) |

The driver's header packet carries Annex K Huffman tables. The model
appends the entropy-coded scan, ending with EOI, as one `JPEGE_PACK_ECS`
(type 5) packet with the header packet's channel and PTS.

## VI and VPSS

The models cover the online pipeline: VI-online into VPSS-online, which is
what the drivers choose for a sensor up to 2304 pixels wide.

- **The sensor's frames never exist.** VI (`hisi-vi`) is the frame clock.
  VPSS (`hisi-vpss`) paints a test picture into each channel's buffers:
  moving bars, and the frame number as 16 binary squares across the top
  twelfth.
- **The contract** comes from `hi_vi.o` and `hi_vpss.o`, in particular
  `VI_COMM_CapOnLineFrameEarlyEndIrqRoute`, `VI_COMM_ProcIrqRoute`,
  `VPSS_COMM_OnLineIrqProc` and the `VPSS_HAL_*` setters.

**VI_CAP (`0x11000000`, SPI 43)**

| Offset | |
|---|---|
| `0xf0` | status. Bit 8 (pipe 0) stands while the pipe's write channel has anything pending; the driver clears only the write channel. |
| `0xf8` | mask. The driver sets all of it when the module loads, so it says nothing about the pipe running. |
| `0x10000 + pipe × 0x1000 + 0xf0` | the pipe's write-channel status, write 1s to clear. Bit 0: frame start. Bit 15: "early", where VPSS's early handler programs this frame's buffers. Bit 1: frame end, where VI sends the frame on. |

**VI_PROC (`0x11200000`, SPI 44)**

| Offset | |
|---|---|
| `0x310` | status, write 1s to clear: bit 10 frame start, bit 1 frame end. `VI_COMM_ProcIrqRoute` panics on a zero status. |
| `0x318` | cleared together with `0x310`. |

The rest of both windows, the ISP's registers at `0x11220000` included,
is storage.

**VPSS (`0x11400000`, SPI 46)**

| Offset | |
|---|---|
| `0x000` | bit 31: online video enable. |
| `0x010` | REGUP. |
| `0x0f0` | interrupt mask, `0x7401`. |
| `0x310` | raw status, write 1s to clear: bit 10 frame start, bit 0 end of frame; `0xadc` clears the errors. |
| `0x314` | masked status. |
| channel size | `0x44c` / `0x4a0` / `0x4e8` |
| channel control | `0x450` / `0x4a4` / `0x4ec`; bit 31 enable, toggled per frame by frame-rate control. |
| channel addresses | `0x468` / `0x4b0` / `0x4f4`: header Y, header C, Y, C, 8 bytes apart. |
| channel stride | `0x488` / `0x4d0` / `0x514` |
| `0x700` | bit 0: channel 0 compression. |

**Each frame**

1. VI raises the write channel's start, early and end bits and the
   pipe's start and end. The driver's handlers program VPSS for this
   frame and write REGUP.
2. An eighth of a frame later, VPSS takes the configuration (if REGUP was
   written since the last frame), paints every enabled channel and raises
   frame start and end. The driver then sends the buffers to VENC.

**When frames run.** VI runs while VPSS's online video is enabled and its
frame interrupt unmasked, from the group's start to its stop. An interrupt
for a pipe that does not exist yet, or any more, dereferences a NULL
context in the driver.

**Compressed channels.** Channel 0 is compressed (`COMPRESS_MODE_SEG`) in
both programs. Its buffers are smaller than a linear picture, and the
real format is not modelled. Instead the two models share a stand-in:
VPSS stores every other row at the channel's Y and C addresses, with the
width as the stride. VEDU sees compression headers in its register image
(`0x1ec`) and doubles the rows back.

**icount.** Run with `-icount shift=0`. Guest time then follows the
guest's instructions, so its interrupt handlers always finish within a
frame. Without it, on a slow host frames can overtake the driver, and an
encoded picture may repeat or reorder (the streams still decode).

Running it:

```bash
bash qemu-boot/test-hisi-majestic.sh --soc hi3516ev200        # the release's majestic (CI; also gk7205v200)
bash qemu-boot/test-hisi-sample-venc.sh --sdk <Hi3516EV200_SDK_V1.0.1.2>   # the SDK's sample_venc
```

## Running it

```bash
bash qemu-boot/test-hisi-venc.sh --soc hi3516ev200 --frames 10   # or gk7205v200, hi3516ev300
python3 qemu/tests/test-hisi-vedu.py                             # register level, no firmware
```

`test-hisi-venc.sh` builds an initramfs from the OpenIPC release with three
pieces:

- busybox;
- the MPI library and the encoder half of `load_hisilicon`'s modules;
- `qemu-boot/hisi-venc-userframe.c`, built with OpenIPC's musl toolchain
  against the OpenIPC/openhisilicon headers. The script fetches both when
  they are missing.

The program sends frames to an H.264, an H.265 and an MJPEG channel. Each
frame shows moving bars plus the frame number as 16 binary squares along
the top. `check-hisi-venc.py` then decodes each channel's dump-dir stream
with `-err_detect explode` and reads the counter back from every frame.

The release's MPI libraries are uClibc builds. Under musl they need:

- an `mmap()` that takes a 32-bit `off_t`. Without it CreateChn fails with
  `mmap err, page addr` and `0xa008800c`;
- stand-ins for the audio VQE / voice symbols they import.

## Not yet

- **VI offline:** a sensor wider than 2304 (EV300's 5 MP IMX335) makes the
  drivers write VI's frames to DDR for VPSS to read, which is not
  modelled. ISP statistics are not produced either: the 3A algorithms run,
  but on nothing.
- **Multi-slice pictures and B frames:** the model codes one slice per
  picture and has no B frames.
- **Other generations:** V1 (CV100), CV300, AV100 and V5 have their own
  encoder layouts and are still register stubs.
