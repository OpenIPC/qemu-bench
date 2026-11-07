#!/usr/bin/env python3
"""Check the streams hisi-vedu's dump-dir kept.

  check-hisi-venc.py --frames N DUMP_DIR [h264|h265|jpeg]...
  check-hisi-venc.py --min-frames N --rising DUMP_DIR [h264|h265|jpeg]...

For each codec asked for, the dump directory must hold one stream
(vedu-<n>.h264 / .h265 / .mjpeg) that ffmpeg decodes with
-err_detect explode, and every frame must carry the binary counter along
its top (16 squares, most significant first, the top twelfth of the
picture high), painted by hisi-venc-userframe or by hisi-vpss.

--frames: exactly N frames, frame i carrying i (hisi-venc-userframe).
--min-frames / --rising: at least N frames, the counter rising from frame
to frame (an application encoding hisi-vpss's frames may skip some, but
never repeats or reverses one: a stale buffer fails).
"""
import argparse
import glob
import os
import subprocess
import sys

EXT = {"h264": "h264", "h265": "h265", "jpeg": "mjpeg"}
FMT = {"h264": "h264", "h265": "hevc", "jpeg": "mjpeg"}


def probe(path, fmt):
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-f", fmt, "-select_streams", "v:0",
         "-show_entries", "stream=width,height", "-of", "csv=p=0", path],
        check=True, capture_output=True, text=True).stdout.strip()
    w, h = out.split(",")[:2]
    return int(w), int(h)


def check(path, codec, frames, min_frames=0, rising=False):
    fmt = FMT[codec]
    err = subprocess.run(
        ["ffmpeg", "-v", "error", "-err_detect", "explode", "-f", fmt,
         "-i", path, "-f", "null", "-"], capture_output=True, text=True)
    if err.returncode or err.stderr.strip():
        return "does not decode cleanly: " + err.stderr.strip()[:300]

    w, h = probe(path, fmt)
    raw = subprocess.run(
        ["ffmpeg", "-v", "error", "-f", fmt, "-i", path, "-f", "rawvideo",
         "-pix_fmt", "gray", "-"], check=True, capture_output=True).stdout
    n = len(raw) // (w * h)
    if rising:
        if n < min_frames:
            return "%d frames, expected at least %d" % (n, min_frames)
    elif n != frames:
        return "%d frames, expected %d" % (n, frames)
    last = -1
    for i in range(n):
        y = raw[i * w * h:(i + 1) * w * h]
        row = min(16, h // 24)
        value = 0
        for bit in range(16):
            x = bit * (w // 16) + w // 32
            value = value << 1 | (y[row * w + x] > 128)
        if rising and value <= last:
            return "frame %d carries counter %d after %d" % (i, value, last)
        if not rising and value != i:
            return "frame %d carries counter %d" % (i, value)
        last = value
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--min-frames", type=int, default=1)
    ap.add_argument("--rising", action="store_true")
    ap.add_argument("dump")
    ap.add_argument("codecs", nargs="*", default=["h264", "h265", "jpeg"])
    args = ap.parse_args()

    fails = 0
    for codec in args.codecs:
        files = glob.glob(os.path.join(args.dump, "vedu-*." + EXT[codec]))
        if len(files) != 1:
            print("FAIL %s: %d streams in %s" % (codec, len(files), args.dump))
            fails += 1
            continue
        why = check(files[0], codec, args.frames, args.min_frames,
                    args.rising)
        if why:
            print("FAIL %s: %s: %s" % (codec, os.path.basename(files[0]), why))
            fails += 1
        elif args.rising:
            print("PASS %s: %s, counter rising" %
                  (codec, os.path.basename(files[0])))
        else:
            print("PASS %s: %d frames, counter 0..%d" %
                  (codec, args.frames, args.frames - 1))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
