#!/bin/bash
# The T30 media path end to end, over the network: the Ingenic T30 SDK
# 1.0.5's own full flash image (T30X_gcc472_uclibc_8M_201907251730.img)
# boots on -M t30,variant=t30x,sensor=jxf22 with its vendor kernel,
# tx-isp-t30.ko (H20190725b) and sample-RTSPServer (LIVE555, "carrier-
# server"), which streams H.265 1920x1080 (/main) and 640x360 (/second)
# with libimp's timestamp OSD blended in by the IPU (hw/misc/ingenic-ipu.c).
# The host pulls both streams with ffmpeg -rtsp_transport tcp through a
# user-mode network port forward, decodes N seconds of each with
# -err_detect explode and checks:
#   - frames decoded, no decoder message;
#   - the ISP's scene (qemu-boot/ingenic-isp-check.py's logo, navy
#     background and dot; on /main also the frame counter) in the first,
#     middle and last picture, the counter advancing and the dot moving;
#   - the OSD, on /main: the timestamp box (libimp's region at 5,16,
#     320x34; rows 16-37, clear of the counter below) is brighter than the
#     same rows further right -- the navy background without the IPU (a
#     timed-out IPU leaves the frame untouched) -- and its text changes
#     over the run.  On /second the region (5,8, 160x18) lies over the
#     ISP's frame counter, so it is only reported there.
#
#   bash qemu-boot/test-ingenic-t30-rtsp.sh [IMAGE] [--seconds N]
#        [--port P] [--qemu PATH] [--ffmpeg CMD] [--out DIR]
#
# IMAGE defaults to the SDK's res/image_t30 image; absent -> SKIP.  The
# image sets eth0 to 193.169.4.151 with gateway 193.169.4.1 (its
# app_init.sh), so the user network is put on that subnet.  --ffmpeg
# defaults to ffmpeg on PATH, else the jrottenberg/ffmpeg:6.1-alpine
# image with --network host (the forward listens on the host's
# 127.0.0.1).  The flash is copied; the image is never written.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
IMG=/home/ai/dimerr/local/ingenic/extract/t30-sdk/Ingenic-SDK-T30-1.0.5-20190725/sdk/Ingenic-SDK-T30-1.0.5-20190725/res/image_t30/T30X_gcc472_uclibc_8M_201907251730.img
SECS=6
PORT=
FFMPEG=
OUT=
BOOT_TIMEOUT=300

while [ $# -gt 0 ]; do
    case $1 in
    --seconds) SECS=$2; shift ;;
    --port) PORT=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --ffmpeg) FFMPEG=$2; shift ;;
    --out) OUT=$2; shift ;;
    -*) echo "unknown option $1" >&2; exit 2 ;;
    *) IMG=$1 ;;
    esac
    shift
done

if [ ! -f "$IMG" ]; then
    echo "SKIP: no T30 SDK image ($IMG)"
    exit 0
fi
[ -x "$QEMU" ] || { echo "FAIL: no $QEMU (bash qemu/setup.sh)"; exit 1; }

WORK=${OUT:-$(mktemp -d "${TMPDIR:-/tmp}/t30-rtsp.XXXXXX")}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
case "$FFMPEG" in
'') if command -v ffmpeg >/dev/null; then FFMPEG=ffmpeg
    elif command -v docker >/dev/null; then
        FFMPEG="docker run --rm --network host -u $(id -u):$(id -g) -v $WORK:$WORK jrottenberg/ffmpeg:6.1-alpine"
    else
        echo "SKIP: no ffmpeg"; exit 0
    fi ;;
esac
if [ -z "$PORT" ]; then
    PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
fi

cp "$IMG" "$WORK/flash.bin"
LOG=$WORK/serial.log
: > "$LOG"
"$QEMU" -M t30,variant=t30x,sensor=jxf22,flash-file="$WORK/flash.bin" \
    -m 128M -display none -monitor none -serial file:"$LOG" \
    -nic user,net=193.169.4.0/24,host=193.169.4.1,hostfwd=tcp:127.0.0.1:$PORT-193.169.4.151:8554 \
    > "$WORK/qemu.out" 2>&1 &
QPID=$!
trap 'kill $QPID 2>/dev/null; wait $QPID 2>/dev/null || true' EXIT

echo "booting $IMG (port $PORT, work $WORK)"
t=0
until grep -q "rtsp://.*:8554/second" "$LOG"; do
    if ! kill -0 $QPID 2>/dev/null; then
        echo "FAIL: QEMU exited"; tail -20 "$LOG" "$WORK/qemu.out"; exit 1
    fi
    if grep -a -q -E "Kernel panic|Oops|Unhandled kernel" "$LOG"; then
        echo "FAIL: kernel"; tail -30 "$LOG"; exit 1
    fi
    if [ $t -ge $BOOT_TIMEOUT ]; then
        echo "FAIL: no RTSP server after ${BOOT_TIMEOUT}s"; tail -20 "$LOG"
        exit 1
    fi
    sleep 2; t=$((t + 2))
done
grep -a -E "tx-isp-probe|chip found|i265e\[info\]|rtsp://" "$LOG" | sed 's/^/  /'
sleep 5                             # the OSD thread's first updates

rc=0
for s in main second; do
    case $s in main) W=1920; H=1080 ;; second) W=640; H=360 ;; esac
    rm -f "$WORK/$s.nv12"
    # no -t: a timed read stops at the wall clock; -frames counts pictures
    $FFMPEG -hide_banner -v error -rtsp_transport tcp \
        -i "rtsp://127.0.0.1:$PORT/$s" -err_detect explode \
        -frames:v $((SECS * 25)) -f rawvideo -pix_fmt nv12 \
        "$WORK/$s.nv12" > "$WORK/$s.ffmpeg.log" 2>&1 || true
    grep -v "^docker: bind sources" "$WORK/$s.ffmpeg.log" > "$WORK/$s.err" || true
    python3 - "$WORK" "$s" "$W" "$H" "$SECS" "$HERE/ingenic-isp-check.py" <<'EOF' || rc=1
import importlib.util, os, sys
import numpy as np
work, s, w, h, secs, chk = sys.argv[1], sys.argv[2], *map(int, sys.argv[3:6]), sys.argv[6]
spec = importlib.util.spec_from_file_location('chk', chk)
c = importlib.util.module_from_spec(spec); spec.loader.exec_module(c)
fs = w * h * 3 // 2
path = os.path.join(work, s + '.nv12')
n = os.path.getsize(path) // fs if os.path.exists(path) else 0
err = open(os.path.join(work, s + '.err')).read().strip()
ok = True
print('%s: %d %dx%d pictures decoded (%d asked), decoder messages: %s'
      % (s, n, w, h, secs * 25, 'none' if not err else err.splitlines()[0]))
if n < secs * 25 or err:
    print('  FAIL: stream'); sys.exit(1)
d = np.memmap(path, dtype=np.uint8, mode='r')
def frame(i):
    y = np.array(d[i * fs:i * fs + w * h]).reshape(h, w)
    uv = np.array(d[i * fs + w * h:(i + 1) * fs]).reshape(h // 2, w)
    return y, uv
# the OSD box (libimp's timestamp region) and a reference box on the same
# rows, right of the counter, where the scene is plain background
if s == 'main':
    box, ref = (16, 38, 5, 325), (16, 38, 600, 920)
else:
    box, ref = (8, 26, 5, 165), (8, 26, 300, 460)
prev = None
for i in sorted({0, n // 2, n - 1}):
    y, uv = frame(i)
    cnt = c.counter(y, w, h) if s == 'main' else None
    bgm, bgs = c.background(y, w, h)
    bright = c.logo_ok(y, w, h, bgm)
    dx = c.dot_x(y, w, h, bgm)
    ob = y[box[0]:box[1], box[2]:box[3]].astype(float)
    rb = y[ref[0]:ref[1], ref[2]:ref[3]].astype(float)
    osd = ob.mean() - rb.mean()
    good = (bright > 0.15 and 20 < bgm < 170 and bgs < 8 and dx is not None
            and (s != 'main' or (cnt is not None and osd > 15)))
    print('  picture %3d: counter %s, logo %.0f%% bright, background Y %.1f, '
          'dot at x %s, OSD box Y %.1f vs %.1f beside it: %s'
          % (i, cnt, bright * 100, bgm, dx, ob.mean(), rb.mean(),
             'ok' if good else 'BAD'))
    ok &= good
    if prev is not None:
        moved = dx is not None and prev[1] is not None and dx != prev[1]
        if s == 'main':
            moved &= cnt is not None and prev[0] is not None and cnt > prev[0]
        print('    advanced: %s' % ('yes' if moved else 'NO'))
        ok &= moved
    prev = (cnt, dx)
# the timestamp ticks once a second: its box must change over the run
y0, _ = frame(0)
y1, _ = frame(n - 1)
diff = np.abs(y0[box[0]:box[1], box[2]:box[3]].astype(int) -
              y1[box[0]:box[1], box[2]:box[3]].astype(int)).max()
print('  OSD box changed over the run: %s (max |dY| %d)%s'
      % ('yes' if diff > 20 else 'NO', diff,
         '' if s == 'main' else ', over the counter: not checked'))
if s == 'main':
    ok &= bool(diff > 20)
try:
    from PIL import Image
    y, uv = frame(n // 2)
    Image.fromarray(y).save(os.path.join(work, s + '.png'))
except ImportError:
    pass
print('  %s' % ('ok' if ok else 'FAIL'))
sys.exit(0 if ok else 1)
EOF
    rm -f "$WORK/$s.nv12"
done

if grep -a -q -E "ipu: done_ipu|ipu start ret" "$LOG"; then
    echo "FAIL: IPU timeouts in the guest log"; grep -a -m3 "ipu" "$LOG"; rc=1
fi
[ $rc -eq 0 ] && echo PASS || echo FAIL
exit $rc
