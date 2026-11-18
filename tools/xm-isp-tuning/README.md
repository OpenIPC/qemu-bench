# xm-isp-tuning: a Xiongmai camera's ISP tuning, as an SDK PQ bin

Xiongmai's firmware for XMedia "Lotus" cameras (GK7206V1: `xmorca`) tunes the
ISP from tables of its own, compiled into its `/usr/bin/App` and read through
`LibXmVideoParam_<module>_getParam()` — one entry per sensor, with day and
night halves. The XMedia SDK publishes no tuning for most sensors (SC465SL
included), so a firmware that runs the SDK on its own gets the ISP library's
generic defaults: on an IVG-K8 (GK7206V11 + SC465SL) in low light, a picture
far noisier and bluer than the stock firmware's on the same board.

These tools take the vendor's tuning out of a firmware dump and turn it into
the SDK's own format, a PQ bin (what the SDK's `sample/pq_bin` and its PQ
tool export, and `xmedia_isp_import()` loads):

1. `xm-isp-capture.py` runs the App's ISP monitor thread under Unicorn — its
   init, its tables, its first monitor loops — with libc stubbed, the
   statically linked SDK reduced to "return 0", and the SDK's attribute
   setters and getters caught. What the App sets is recorded block by block,
   as the SDK's structures. Getters answer with the last value set, or with
   the camera's own block (`attrtool dump`), so a get-modify-set lands on
   what the App had put there. Decoding the tables themselves would mean
   mapping each of Xiongmai's packed layouts onto the SDK's fields; running
   the code that does it gets the same answer.
2. `xm-isp-finish` drops what the App sets at run time rather than from tables
   and fixes the few fields the capture cannot carry (see its header).
3. On a camera running the same SDK: `attrtool apply` the set, `ae-ainr` for
   the AE curve and AINR switch points, then `pqexport` the result.

## Use

```sh
SDK_INC=<shimetapi_pico_gx>/source/gmp/include \
TOOLCHAIN=<toolchain.goke-gk7206v100> tools/xm-isp-tuning/build.sh
pip install unicorn pyelftools        # 2.1.4 / 0.32 used

# the App, from a stock dump: the second squashfs (bin/App)
# on the camera, with the streamer up (the SDK's defaults in force):
./attrtool dump base/                 # the baseline blocks

tools/xm-isp-tuning/xm-isp-capture.py --app App \
    --profile tools/xm-isp-tuning/profiles/ipc_gk7206v11_k8-v1.10.t02.20261103.json \
    --layout tools/xm-isp-tuning/build/layout.json --base base \
    --sensor SC465SL --iso 25600 --ct 3500 --out capture
tools/xm-isp-tuning/build/xm-isp-finish capture base final
tools/xm-isp-tuning/build/xm-isp-decode final     # what changed, readable

# on the camera:
./attrtool apply final $(ls final | sed 's/.bin//')
./ae-ainr 100:35,400:42,1600:36,6400:28,12800:25,25600:23,102400:25,409600:25 \
    168 8 2048000 1024 10000 11000
./pqexport sc465sl.bin                # -> /etc/sensors/iq/sc465sl.bin
```

The capture takes well under a second. The camera tools link the SDK's
`libxmedia_*.so` from the toolchain sysroot and run beside the process that
owns the ISP (the blocks live in the ISP's shared register image).

## What the App does, measured for SC465SL

From the capture and the stock camera's `/proc/umap/isp`:

- Auto tables (ISO-indexed) for sharpen (edge 100→128, texture 64→0 as ISO
  rises; the stock camera showed edge 128 / texture 11 at ISO ~26000), STNR,
  BNR, demosaic, LCE, CA, AWB saturation (90→70) and 3 CCMs (2650-6350 K);
  DPC soft threshold 6 / slope 21; DRC on a user tone curve; dehaze on a user
  LUT; its own gamma.
- AE: compensation by ISO (100:35, 400:42, 1600:36, 6400:28, 12800:25,
  25600:23, ≥102400:25 — the stock camera ran target 28-29 at ISO ~21000),
  speed 168, black delay 8, total gain capped at 2000x (2048000 Q10), gain
  threshold 1x, and the AE's slow-shutter mode at 125 ms (8 fps from 25).
  The limits and the mode come from its AE control code, not the tables.
- AINR switching on at ISO 11000, off below 10000, with the SDK's network.
- White balance is the App's own algorithm (AWB bypass 1, CCM rewritten
  every loop from the SDK's statistics); the capture keeps its calibration
  and lets the SDK's AWB run on it.

Applied to an OpenIPC IVG-K8 beside a stock one (dark storeroom, ISO ~20000,
125 ms): the same picture — neutral colour, deep blacks, low noise; the
OpenIPC one a little sharper. Re-running the capture reproduces the set the
shipped `sc465sl.bin` was exported from byte for byte.

## The twin route, and why it wins

`twin/xm-twin-export.py` boots the whole stock firmware in the `gk7206v11a`
machine, lets the App bring its ISP up, and runs `attrtool dump` and
`pqexport` inside it (static musl builds of the camera tools, linked against
the SDK's `.a` libraries; `camera/hexout.c` carries the results out over the
console). About ten minutes, nearly all of it typing the tools in.

It reads what the App actually left in the ISP, including what it rewrites
after the code path the Unicorn capture runs. For SC465SL that matters: the
capture's STNR, BNR temporal, sharpen and LCE tables are not the App's live
ones. Interpolated to a stock camera's ISO (21010, 25867), the twin's
tables give the values that camera reports in `/proc/umap/isp` (TNR motion
thresholds 32/65, chroma strength 248, sharpen texture 16, overshoot 86);
the capture's give 22/43, 224, 13, 113. The static blocks (gamma, CA,
demosaic, DPC, LCAC, BLC, shading) and the AWB calibration agree byte for
byte between the two.

So the shipped `sc465sl.bin` takes its ISP blocks from the twin and its AWB
calibration from the capture (the App's own AWB is not in the ISP to read),
through `xm-isp-finish` as before. The twin's scene is synthetic and bright,
so scene-driven scalars (DRC strength, dehaze, the AWB gains) are the
twin's daylight values; the ISO-indexed tables do not depend on that.

## Not covered

- The App's night (IR) half: `--then 0x362958:1` applies it after the
  monitor (the profile's `apply_day_night`), but nothing here has been
  measured at night with IR.
- Daylight: the tables are ISO-indexed, but only low light was compared.
- Other sensors and App builds: the sensor is an argument; the addresses are
  this App build's (`app_md5` in the profile). Another build needs its own
  profile — the setters are named by the `xmedia_*_attr ISP_Failed` strings
  logged after each call, and the monitor entry is the thread `IspCtrl_init`
  starts.
