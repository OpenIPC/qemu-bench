#!/bin/bash
#
# majestic's H.264 on an emulated Xiongmai camera, end to end.
#
# Boots the release kernel and rootfs from a synthetic NOR the way the flash
# tests do (mem= first, as OpenIPC's U-Boot passes it; it is what sizes MMZ
# and the VI buffer count in load_xiongmai), leaves majestic to
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
#      an encoder fed a stale buffer fails.  --settle frames at the start
#      are exempt: XM510's VI driver hands out a frame it queued before
#      VENC started after the first two new ones.
#
# --soc picks the release, the board's mem= and RAM, the CPUs, majestic's
# main-channel size and the settle count; the other options override it.
#
#   xm530  xm530 release, mem=32M of 64M, 2 CPUs, 704x576 (the D1
#          feeder; since the 2026-11-17 release majestic also encodes its
#          1920x1080 main channel as H.264)
#
# The H1 serves every H.264 channel majestic runs, so the jobs are split by
# the picture size in their ENC_CTRL (the .regs dump beside each job) and
# each channel is checked on its own; --size must be one of them.
#   xm510  xm510 release, mem=18M of 32M as U-Boot passes it (three VI
#          buffers), 1 CPU, 1280x720
#
# Usage:
#   test-xm-stream.sh --soc xm530|xm510 --output-dir <dir> \
#       [--qemu PATH] [--machine M] [--kernel <uImage>] [--rootfs <squashfs>] \
#       [--frames 60] [--size WxH] [--settle N] [--kernel-mem 18M] \
#       [--timeout 240]
#
set -u

SOC=xm530
MACHINE=""
QEMU="./qemu-src/build/qemu-system-arm"
OUTDIR="."
KERNEL=""
ROOTFS=""
FRAMES=60
SIZE=""
SETTLE=""
KMEM=""
TIMEOUT=240

while [ $# -gt 0 ]; do
    case "$1" in
        --soc)        SOC="$2"; shift 2 ;;
        --machine)    MACHINE="$2"; shift 2 ;;
        --qemu)       QEMU="$2"; shift 2 ;;
        --output-dir) OUTDIR="$2"; shift 2 ;;
        --kernel)     KERNEL="$2"; shift 2 ;;
        --rootfs)     ROOTFS="$2"; shift 2 ;;
        --frames)     FRAMES="$2"; shift 2 ;;
        --size)       SIZE="$2"; shift 2 ;;
        --settle)     SETTLE="$2"; shift 2 ;;
        --kernel-mem) KMEM="$2"; shift 2 ;;
        --timeout)    TIMEOUT="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

case "$SOC" in
    xm530) P_MEM=32M; P_RAM=64M; P_SMP=2; P_SIZE=704x576;  P_SETTLE=0 ;;
    xm510) P_MEM=18M; P_RAM=32M; P_SMP=1; P_SIZE=1280x720; P_SETTLE=3 ;;
    *) echo "unknown --soc $SOC" >&2; exit 2 ;;
esac
: "${MACHINE:=$SOC}" "${SIZE:=$P_SIZE}" "${SETTLE:=$P_SETTLE}" "${KMEM:=$P_MEM}"
: "${KERNEL:=qemu-boot/uImage.$SOC}" "${ROOTFS:=qemu-boot/rootfs.squashfs.$SOC}"

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
APPEND="mem=$KMEM console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs init=/init mtdparts=${MTDPARTS}"

CONSOLE="$OUTDIR/console.txt"; rm -f "$CONSOLE"
echo "=== booting $MACHINE, waiting for $FRAMES H.264 jobs (<= ${TIMEOUT}s) ==="
"$QEMU" -M "$MACHINE",flash-file="$FLASH" -smp "$P_SMP" -m "$P_RAM" -kernel "$KERNEL" \
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
    python3 - "$DUMP" "$STREAM" "$SIZE" "$SETTLE" <<'PY' || fail=1
import os, subprocess, sys

dump, stream, size, settle = sys.argv[1:]
settle = int(settle)
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

# 2./4. per channel: the jobs grouped by ENC_CTRL's picture size
def enc_size(name):
    for l in open(os.path.join(dump, name[:-len('.h264')] + '.regs')):
        a, v = l.split()
        if int(a, 16) == 0x38:
            v = int(v, 16)
            return ((v >> 19) & 0x1ff) * 16, ((v >> 10) & 0x1ff) * 16
    return 0, 0

channels = {}
for name in sorted(os.listdir(dump)):
    if name.endswith('.h264'):
        channels.setdefault(enc_size(name), []).append(name)
# coded heights round up to 16: 1080 is coded as 1088
check(any(cw == w and h <= ch < h + 16 for cw, ch in channels),
      'a channel at %s among %s' % (size, ', '.join(
          '%dx%d (%d jobs)' % (cw, ch, len(n)) for (cw, ch), n in channels.items())))

for (cw, ch), names in sorted(channels.items()):
    tag = '%dx%d' % (cw, ch)
    stream_c = '%s.%s' % (stream, tag)
    with open(stream_c, 'wb') as out:
        for n in names:
            out.write(open(os.path.join(dump, n), 'rb').read())
    r = subprocess.run(['ffmpeg', '-v', 'error', '-err_detect', 'explode', '-i',
                        stream_c, '-f', 'rawvideo', '-pix_fmt', 'yuv420p', '-'],
                       capture_output=True)
    check(r.returncode == 0 and not r.stderr,
          '%s: ffmpeg decodes the stream (exit %d)%s' % (tag, r.returncode,
          ': ' + r.stderr.decode(errors='replace')[:300] if r.stderr else ''))
    info = dict(l.split('=', 1) for l in subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
         'stream=width,height', '-of', 'default=nw=1', stream_c],
        capture_output=True, text=True).stdout.split())
    pw, ph = int(info.get('width', 0)), int(info.get('height', 0))
    fs = pw * ph * 3 // 2
    frames = len(r.stdout) // fs if r.returncode == 0 and fs else 0
    check(frames == len(names), '%s: %d frames decoded from %d jobs' %
          (tag, frames, len(names)))

    # 4. xm-vi's frame number, 16 blocks of 16 px across the top left, MSB
    # first -- at the main size; xm-vi scales it into a smaller channel
    if pw != w:
        continue
    counts = []
    for f in range(frames):
        y = r.stdout[f * fs:f * fs + pw * ph]
        v = 0
        for b in range(16):
            v = (v << 1) | (y[8 * pw + b * 16 + 8] > 128)
        counts.append(v)
    live = counts[settle:]
    rising = all(b > a for a, b in zip(live, live[1:]))
    check(len(live) > 1 and rising,
          '%s: the VI frame number rises through every picture%s (%s .. %s)' %
          (tag, ' after the first %d' % settle if settle else '',
           live[:3], live[-3:]))
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
