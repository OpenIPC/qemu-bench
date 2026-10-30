#!/bin/bash
# The vendor encoder path end to end on an emulated camera: OpenIPC's
# kernel and rootfs, the vendor modules from the rootfs's own load_ingenic
# (sinfo, tx-isp-<soc>, sensor_<sensor>_<soc>), the machine's sensor= and
# the ingenic-isp model for the frames, and qemu/tests/ingenic-vpu-enc.c:
# the SDK samples' FrameSource -> IMP_Encoder sequence, H.264 (CBR) and a
# JPEG snapshot through libimp onto the ingenic-vpu model.  The streams
# come back on an SD card image and ffmpeg decodes them on the host
# (with -err_detect explode).
#
#   bash qemu/tests/test-ingenic-vpu-e2e.sh [--soc t21] [--sensor NAME]
#        [--fw DIR] [--qemu PATH] [--frames N] [--out DIR]
#
# Needs: hw/misc/ingenic-isp.c and the machine's sensor= (the ISP model),
# the Ingenic uClibc toolchain, the T31 SDK headers (ISP, FrameSource) and
# the T23 SDK's imp_encoder.h, ffmpeg (or FFMPEG=...).  T21's jxf37 (DVP)
# is the sensor the ISP model has a T21 preset for.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$(dirname "$HERE")")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=/home/ai/dimerr/local/ingenic/fw/t21
TC=/home/ai/toolchains/ingenic/mips-gcc540-uclibc0.9.33.2-64bit-r3.3.0.smaller/bin
SDK=/home/ai/dimerr/local/ingenic/extract/isvp-t31/software/sdk/Ingenic-SDK-T31-1.1.5-20220514/sdk/5.4.0
ENCH=/home/ai/dimerr/local/ingenic/extract/Ingenic-SDK-T23-1.1.0-20230726/sdk/include/imp/imp_encoder.h
FFMPEG=${FFMPEG:-ffmpeg}
SOC=t21
SENSOR=
FRAMES=20
OUT=
TIMEOUT=900

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --sensor) SENSOR=$2; shift ;;
    --fw) FW=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --frames) FRAMES=$2; shift ;;
    --out) OUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
case $SOC in
t21) SENSOR=${SENSOR:-jxf37} ;;
*) echo "no end-to-end encoder test for $SOC (the ISP model has T21/T31)" >&2
   exit 2 ;;
esac
case $SENSOR in
jxf37) ADDR=0x40 ;;
gc2053) ADDR=0x37 ;;
*) echo "unknown sensor $SENSOR" >&2; exit 2 ;;
esac
for f in "$FW/uImage.$SOC" "$FW/rootfs.squashfs.$SOC" "$QEMU" "$TC/mips-linux-uclibc-gcc" \
         "$SDK/include/imp/imp_framesource.h" "$ENCH"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

echo "== rootfs with the encoder client"
unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.$SOC" >/dev/null
read W H <<< "$(sed -n 's/^ *\(width\|height\): *\([0-9]*\).*/\2/p' \
    "$WORK/rfs/etc/sensor/$SENSOR.yaml" | tr '\n' ' ')"
W=${W:-1920}
H=${H:-1080}
CC=$TC/mips-linux-uclibc-gcc
S=$($CC -print-sysroot)/usr/lib
$CC -O2 -Wall -march=mips32r2 -I"$SDK/include" "-DIMP_ENCODER_H=\"$ENCH\"" \
    -c -o "$WORK/enc.o" "$HERE/ingenic-vpu-enc.c"
$CC -nostdlib -o "$WORK/rfs/usr/bin/vpu-enc" \
    "$S/crt1.o" "$S/crti.o" "$($CC -print-file-name=crtbegin.o)" \
    "$WORK/enc.o" -L"$WORK/rfs/usr/lib" -L"$WORK/rfs/lib" \
    -limp -lalog -l:libc.so "$($CC -print-libgcc-file-name)" \
    "$($CC -print-file-name=crtend.o)" "$S/crtn.o" \
    -Wl,--dynamic-linker=/lib/ld-musl-mipsel.so.1 \
    -Wl,--export-dynamic -Wl,--allow-shlib-undefined
"$TC/mips-linux-uclibc-strip" "$WORK/rfs/usr/bin/vpu-enc"
cat > "$WORK/rfs/usr/bin/vpu-test" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t tmpfs tmp /var
mkdir -p /dev/shm && mount -t tmpfs shm /dev/shm
mkdir -p /var/lock
echo "VPUTEST sinfo: \$(modprobe sinfo; echo 1 > /proc/jz/sinfo/info; cat /proc/jz/sinfo/info)"
load_ingenic -i
touch /tmp/closesimd
vpu-enc $FRAMES /dev/mmcblk0 $SENSOR $ADDR $W $H
echo "VPUTEST rc=\$?"
sync
echo VPUTEST-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/vpu-test"
echo "/dev/mtd1 0x0 0x10000 0x8000" > "$WORK/rfs/etc/fw_env.config"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
truncate -s 32M "$WORK/sd.img"
python3 - "$WORK/nor.bin" "$FW/uImage.$SOC" "$WORK/rootfs.squashfs" \
    "sensor=$SENSOR" "soc=$SOC" <<'PY'
import struct, sys, zlib
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    img[off:off + len(data)] = data
env = b''.join(kv.encode() + b'\0' for kv in sys.argv[4:]) + b'\0'
env += b'\0' * (0x10000 - 4 - len(env))
img[0x40000:0x50000] = struct.pack('<I', zlib.crc32(env)) + env
open(sys.argv[1], 'wb').write(img)
PY

for try in 1 2 3; do
    echo "== boot ($SOC, sensor=$SENSOR)${try#1}"
    rm -f "$OUT/console.log"
    "$QEMU" -M $SOC,sensor=$SENSOR,flash-file="$WORK/nor.bin" -m 64M \
        -display none -monitor none -serial file:"$OUT/console.log" \
        -nic user,restrict=on -kernel "$FW/uImage.$SOC" \
        -drive if=sd,index=0,file="$WORK/sd.img",format=raw \
        -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/vpu-test mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=40M rmem=24M@0x2800000" &
    QPID=$!
    started=
    for i in $(seq "$TIMEOUT"); do
        grep -q VPUTEST-END "$OUT/console.log" 2>/dev/null && break
        grep -q "VPUTEST sinfo" "$OUT/console.log" 2>/dev/null && started=1
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
grep -a -E "VPUTEST|^chn[0-9]|ingenic-vpu-enc|failed" "$OUT/console.log" | tail -30 || true
grep -q "VPUTEST rc=0" "$OUT/console.log" || { echo "FAIL: the client did not get its streams"; exit 1; }

echo "== streams"
python3 - "$WORK/sd.img" "$OUT" <<'PY'
import struct, sys
img = open(sys.argv[1], 'rb').read()
for off, tag, name in ((0, b'H264', 'out.h264'), (8 << 20, b'JPEG', 'snap.jpg')):
    assert img[off:off + 4] == tag, 'no %s on the card' % tag
    n = struct.unpack_from('<I', img, off + 4)[0]
    open(sys.argv[2] + '/' + name, 'wb').write(img[off + 512:off + 512 + n])
    print('%s: %d bytes' % (name, n))
PY
fail=0
for f in out.h264 snap.jpg; do
    if r=$($FFMPEG -v error -err_detect explode -i "$OUT/$f" -f null - 2>&1 | grep -v "^docker:"); [ -z "$r" ]; then
        echo "ok   ffmpeg decodes $f cleanly"
    else
        echo "FAIL ffmpeg on $f: $r"; fail=1
    fi
done
n=$($FFMPEG -v error -i "$OUT/out.h264" -f framecrc - 2>/dev/null | grep -c "^0,")
echo "$([ "$n" -eq "$FRAMES" ] && echo ok || echo FAIL)   $n of $FRAMES H.264 frames"
[ "$n" -eq "$FRAMES" ] || fail=1
$FFMPEG -v error -y -i "$OUT/out.h264" -vf "select='eq(n\,0)+eq(n\,$((FRAMES - 1)))',tile=1x2" \
    -frames:v 1 -update 1 "$OUT/out.png" 2>/dev/null || true
$FFMPEG -v error -y -i "$OUT/snap.jpg" -update 1 "$OUT/snap.png" 2>/dev/null || true
[ "$fail" = 0 ] && echo PASS || { echo FAIL; exit 1; }
