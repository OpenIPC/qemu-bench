#!/bin/bash
# JPEG decoding end to end (T23, T30): OpenIPC's kernel and rootfs with its
# libimp, qemu-boot/ingenic-jpegd-test.c running the SDK's
# sample-Decoder-jpeg sequence (IMP_Decoder) on JPEG files it carries
# in its root; libimp parses each file and hands the scan to HELIX's JPGC
# (hw/misc/ingenic-vpu.c).  The host saves each decoded NV12 picture from
# guest memory through the monitor (the parts' kernels have no SD card
# here) and compares it with ffmpeg's decode of the same file.
#
#   bash qemu-boot/test-ingenic-jpegd.sh [--soc t23|t30] [--jpeg FILE]...
#        [--fw DIR] [--qemu PATH] [--out DIR]
#
# Without --jpeg, ffmpeg makes two test pictures (640x480 and 1280x720,
# 4:2:0).  Needs the Ingenic uClibc toolchain, the SDK's libimp.a (T23,
# T30 SDKs) and ffmpeg (or FFMPEG=...).
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
TC=/home/ai/toolchains/ingenic/mips-gcc540-uclibc0.9.33.2-64bit-r3.3.0.smaller/bin
SDK23=/home/ai/dimerr/local/ingenic/extract/Ingenic-SDK-T23-1.1.0-20230726/sdk
SDK30=/home/ai/dimerr/local/ingenic/extract/t30-sdk/Ingenic-SDK-T30-1.0.5-20190725/sdk/Ingenic-SDK-T30-1.0.5-20190725
FFMPEG=${FFMPEG:-ffmpeg}
SOC=t23
FW=
OUT=
JPEGS=()
TIMEOUT=${TIMEOUT:-600}

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --jpeg) JPEGS+=("$2"); shift ;;
    --fw) FW=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
case $SOC in
t23|t30) ;;
*) echo "IMP_Decoder is in T23's and T30's libimp only" >&2; exit 2 ;;
esac
FW=${FW:-/home/ai/dimerr/local/ingenic/fw/$SOC}
# the client is static, against the SDK's own uClibc libimp
case $SOC in
t23) INC=$SDK23/include; L=$SDK23/lib/uclibc
     LIBS="$L/libimp.a $L/libalog.a $L/libsysutils.a" ;;
t30) INC=$SDK30/include; L=$SDK30/lib-5.4.0/uclibc
     LIBS="$L/libimp.a $L/libalog.a $L/libsysutils.a" ;;
esac
for f in "$FW/uImage.$SOC" "$FW/rootfs.squashfs.$SOC" "$QEMU" \
         "$TC/mips-linux-uclibc-gcc" "$INC/imp/imp_decoder.h" $LIBS; do
    [ -e "$f" ] || { echo "SKIP: missing $f"; exit 0; }
done
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

if [ ${#JPEGS[@]} = 0 ]; then
    # the pictures come back on stdout: a docker ffmpeg need not see this
    # host's files
    for s in 640x480 1280x720; do
        $FFMPEG -v error -f lavfi -i testsrc=size=$s -frames:v 1 \
            -pix_fmt yuvj420p -q:v 3 -f mjpeg - > "$OUT/test-$s.jpg"
        JPEGS+=("$OUT/test-$s.jpg")
    done
fi

echo "== rootfs with the decoder client"
unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.$SOC" >/dev/null
CC=$TC/mips-linux-uclibc-gcc
$CC -O2 -Wall -march=mips32r2 -I"$INC" -static -s \
    -o "$WORK/rfs/usr/bin/jpegd-test" "$HERE/ingenic-jpegd-test.c" \
    $LIBS -lpthread -lm -lrt -lstdc++
cat > "$WORK/rfs/usr/bin/jpegd-init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t tmpfs tmp /var
mkdir -p /dev/shm && mount -t tmpfs shm /dev/shm
mkdir -p /var/lock
echo JPEGDTEST-START
touch /tmp/closesimd
jpegd-test 1920 1088 /usr/share/jpegd/*.jpg
echo "JPEGDTEST rc=\$?"
sync
echo JPEGDTEST-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/jpegd-init"
mkdir -p "$WORK/rfs/usr/share/jpegd"
n=0
for j in "${JPEGS[@]}"; do
    cp "$j" "$WORK/rfs/usr/share/jpegd/$n.jpg"
    n=$((n + 1))
done
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
python3 - "$WORK/nor.bin" "$FW/uImage.$SOC" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    img[off:off + len(data)] = data
open(sys.argv[1], 'wb').write(img)
PY

# A boot now and then stops before init prints anything (as in the other
# harnesses); such a boot is retried.
for try in 1 2 3; do
echo "== boot ($SOC)${try#1}"
rm -f "$OUT/console.log" "$OUT"/dec*
# shellcheck disable=SC2086  # QEMU_EXTRA: extra options, word-split
"$QEMU" -M $SOC,flash-file="$WORK/nor.bin" -m 64M $QEMU_EXTRA \
    -display none -serial file:"$OUT/console.log" \
    -monitor unix:"$WORK/mon.sock",server,nowait \
    -nic user,restrict=on -kernel "$FW/uImage.$SOC" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/jpegd-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=40M rmem=24M@0x2800000" &
QPID=$!
saved=0
started=
for i in $(seq "$TIMEOUT"); do
    grep -q JPEGDTEST-END "$OUT/console.log" 2>/dev/null && break
    # each decoded picture, held by the client: save it from guest memory
    grep -a "^JPEGDFRAME" "$OUT/console.log" 2>/dev/null | tail -n +$((saved + 1)) |
    while read -r _ n addr w h size; do
        python3 - "$WORK/mon.sock" "$addr" "$size" "$OUT/dec$n.nv12" <<'PY'
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
s.sendall(b'pmemsave %d %d "%s"\n' % (int(sys.argv[2], 16), int(sys.argv[3]),
                                       sys.argv[4].encode()))
time.sleep(1)
s.close()
PY
        echo "$w $h" > "$OUT/dec$n.size"
    done
    saved=$(grep -a -c "^JPEGDFRAME" "$OUT/console.log" 2>/dev/null || true)
    grep -q JPEGDTEST-START "$OUT/console.log" 2>/dev/null && started=1
    [ -z "$started" ] && [ "$i" -ge 90 ] && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 2
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true
[ -n "$started" ] && break
echo "   no init output in 90 s, retrying"
done
grep -a -E "JPEGDTEST|jpegd-test" "$OUT/console.log" || true
grep -q "JPEGDTEST rc=0" "$OUT/console.log" || {
    echo "FAIL: the client did not decode its JPEGs"; exit 1; }

echo "== pictures"
fail=0
n=0
for j in "${JPEGS[@]}"; do
    $FFMPEG -v error -i - -f rawvideo -pix_fmt yuvj420p - < "$j" \
        > "$OUT/ref$n.yuv" 2> /dev/null || true
    python3 - "$OUT" $n "$(basename "$j")" <<'PY' || fail=1
import math, os, sys
out, n, name = sys.argv[1], int(sys.argv[2]), sys.argv[3]
if not os.path.exists('%s/dec%d.nv12' % (out, n)):
    print('FAIL   %s: no picture saved' % name)
    sys.exit(1)
w, h = map(int, open('%s/dec%d.size' % (out, n)).read().split())
pic = open('%s/dec%d.nv12' % (out, n), 'rb').read()
ref = open('%s/ref%d.yuv' % (out, n), 'rb').read()
# libimp turns the JPGC's 16x16 tiles into NV12 itself and leaves the
# rows of a partial last macroblock row zero (whatever the tiles hold:
# they are checked in qemu/tests/test-ingenic-vpu.py); compare above them
rows = h // 16 * 16 if h % 16 else h
n = w * rows
ok = len(ref) >= n and len(pic) >= n
se = sum((a - b) ** 2 for a, b in zip(pic[:n], ref[:n])) if ok else 1
p = 99.0 if not se else 10 * math.log10(255 * 255 * n / se)
good = ok and p >= 40
print('%s   %s: %dx%d, Y PSNR %.1f dB against ffmpeg (>= 40)%s'
      % ('ok' if good else 'FAIL', name, w, h, p,
         ', rows 0-%d' % (rows - 1) if rows != h else ''))
sys.exit(0 if good else 1)
PY
    n=$((n + 1))
done
[ "$fail" = 0 ] && echo PASS || { echo FAIL; exit 1; }
