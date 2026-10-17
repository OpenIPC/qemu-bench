#!/bin/bash
#
# majestic's H.264 on the emulated XM530, end to end.
#
# Boots the release kernel and rootfs from a synthetic NOR the way the flash
# tests do (mem= first, as OpenIPC's U-Boot passes it), leaves majestic to
# start on its own, and has the H1 model keep every job it encodes
# (-global xm-h1.dump-dir).  No login, no claim: RTSP and HTTP stay 401 on an
# unclaimed camera, so the stream is judged from the encoder's side.  The
# host then checks:
#
#   1. majestic encoded at least --frames H.264 pictures
#   2. the jobs concatenate into a stream ffmpeg decodes with no errors,
#      frame for frame, at majestic's main-channel size (--size)
#   3. every IDR job carries its own SPS and PPS, and there is more than
#      one, so the headers majestic rewrites per GOP are followed
#   4. the pictures are live: xm-vi stamps its frame number into the top
#      left as 16 binary blocks, and it must rise from frame to frame, so
#      an encoder fed a stale buffer fails
#
# Usage:
#   test-xm530-stream.sh --qemu ./qemu-src/build/qemu-system-arm \
#       --output-dir <dir> [--machine xm530] [--kernel <uImage>] \
#       [--rootfs <squashfs>] [--frames 60] [--size 704x576] [--timeout 240]
#
set -u

MACHINE=xm530
QEMU="./qemu-src/build/qemu-system-arm"
OUTDIR="."
KERNEL="qemu-boot/uImage.xm530"
ROOTFS="qemu-boot/rootfs.squashfs.xm530"
FRAMES=60
SIZE=704x576
TIMEOUT=240

while [ $# -gt 0 ]; do
    case "$1" in
        --machine)    MACHINE="$2"; shift 2 ;;
        --qemu)       QEMU="$2"; shift 2 ;;
        --output-dir) OUTDIR="$2"; shift 2 ;;
        --kernel)     KERNEL="$2"; shift 2 ;;
        --rootfs)     ROOTFS="$2"; shift 2 ;;
        --frames)     FRAMES="$2"; shift 2 ;;
        --size)       SIZE="$2"; shift 2 ;;
        --timeout)    TIMEOUT="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

command -v ffmpeg >/dev/null || { echo "=== FAIL: needs ffmpeg ==="; exit 1; }
for f in "$KERNEL" "$ROOTFS"; do
    [ -f "$f" ] || { echo "=== FAIL: missing $f ==="; exit 1; }
done
mkdir -p "$OUTDIR"
DUMP="$OUTDIR/h1-jobs"
rm -rf "$DUMP"; mkdir -p "$DUMP"

BOOT_KB=256; ENV_KB=64; KERN_KB=2048
ROOTFS_KB=$(( ( ($(stat -c %s "$ROOTFS") + 65535) / 65536 ) * 64 ))
FLASH="$OUTDIR/flash.bin"
head -c $((8 * 1024 * 1024)) /dev/zero | tr '\000' '\377' > "$FLASH"
dd if="$ROOTFS" of="$FLASH" bs=1k seek=$((BOOT_KB + ENV_KB + KERN_KB)) conv=notrunc 2>/dev/null
MTDPARTS="xm_sfc:${BOOT_KB}k(boot),${ENV_KB}k(env),${KERN_KB}k(kernel),${ROOTFS_KB}k(rootfs),-(rootfs_data)"
APPEND="mem=32M console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs init=/init mtdparts=${MTDPARTS}"

CONSOLE="$OUTDIR/console.txt"; rm -f "$CONSOLE"
echo "=== booting $MACHINE, waiting for $FRAMES H.264 jobs (<= ${TIMEOUT}s) ==="
"$QEMU" -M "$MACHINE",flash-file="$FLASH" -smp 2 -m 64M -kernel "$KERNEL" \
    -global xm-h1.dump-dir="$DUMP" \
    -nographic -serial file:"$CONSOLE" -monitor none \
    -nic user,restrict=on -append "$APPEND" >/dev/null 2>"$OUTDIR/qemu.err" &
pid=$!
t=0
while kill -0 "$pid" 2>/dev/null && [ "$t" -lt "$TIMEOUT" ]; do
    [ "$(find "$DUMP" -name '*.h264' | wc -l)" -ge "$FRAMES" ] && break
    sleep 1; t=$((t + 1))
done
kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null

fail=0
jobs=$(find "$DUMP" -name '*.h264' | wc -l)
if [ "$jobs" -ge "$FRAMES" ]; then
    echo "ok   majestic encoded $jobs H.264 jobs in ${t}s"
else
    echo "FAIL: $jobs H.264 jobs in ${t}s, wanted $FRAMES"; fail=1
fi

STREAM="$OUTDIR/stream.h264"
find "$DUMP" -name '*.h264' | sort | xargs -r cat > "$STREAM"
if [ "$jobs" -gt 0 ]; then
    python3 - "$DUMP" "$STREAM" "$SIZE" <<'PY' || fail=1
import os, subprocess, sys

dump, stream, size = sys.argv[1:]
w, h = (int(v) for v in size.split('x'))
fail = 0

def check(ok, what):
    global fail
    print(('ok   ' if ok else 'FAIL: ') + what)
    fail |= not ok

def nal_types(data):
    i, out = 0, []
    while True:
        i = data.find(b'\x00\x00\x01', i)
        if i < 0 or i + 3 >= len(data):
            return out
        out.append(data[i + 3] & 0x1f)
        i += 3

# 3. every IDR job carries the SPS and PPS ahead of its slice
idrs = bad = 0
for name in sorted(os.listdir(dump)):
    if not name.endswith('.h264'):
        continue
    t = nal_types(open(os.path.join(dump, name), 'rb').read())
    if 5 in t:
        idrs += 1
        ahead = t[:t.index(5)]
        bad += not (7 in ahead and 8 in ahead)
check(idrs >= 2 and not bad,
      '%d IDR jobs, %d without their own SPS/PPS (want >= 2 and 0)' % (idrs, bad))

# 2. a clean decode, one frame per job, at the expected size
jobs = sum(n.endswith('.h264') for n in os.listdir(dump))
r = subprocess.run(['ffmpeg', '-v', 'error', '-err_detect', 'explode', '-i', stream,
                    '-f', 'rawvideo', '-pix_fmt', 'yuv420p', '-'], capture_output=True)
check(r.returncode == 0 and not r.stderr,
      'ffmpeg decodes the stream (exit %d)%s' % (r.returncode,
      ': ' + r.stderr.decode(errors='replace')[:300] if r.stderr else ''))
info = dict(l.split('=', 1) for l in subprocess.run(
    ['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
     'stream=width,height,profile', '-of', 'default=nw=1', stream],
    capture_output=True, text=True).stdout.split())
check((info.get('width'), info.get('height')) == (str(w), str(h)),
      'picture %sx%s, want %s (%s)' % (info.get('width'), info.get('height'),
                                       size, info.get('profile')))
fs = w * h * 3 // 2
frames = len(r.stdout) // fs if r.returncode == 0 else 0
check(frames == jobs, '%d frames decoded from %d jobs' % (frames, jobs))

# 4. xm-vi's frame number, 16 blocks of 16 px across the top left, MSB first
counts = []
for f in range(frames):
    y = r.stdout[f * fs:f * fs + w * h]
    v = 0
    for b in range(16):
        v = (v << 1) | (y[8 * w + b * 16 + 8] > 128)
    counts.append(v)
rising = all(b > a for a, b in zip(counts, counts[1:]))
check(frames > 1 and rising,
      'the VI frame number rises through every picture (%s .. %s)' %
      (counts[:3], counts[-3:]))
sys.exit(fail)
PY
fi

if [ "$fail" = "0" ]; then
    echo "=== PASS: majestic streams H.264 on the emulated $MACHINE ==="
    exit 0
fi
echo "--- console, majestic lines ---"
tr -d '\r' < "$CONSOLE" | grep -aiE 'majestic|venc|error' | tail -30
exit 1
