#!/bin/bash
# T23's encoder path end to end, with the IVDC (direct_mode) and a second
# sensor, on the HELIX model: OpenIPC's T23 kernel and rootfs, the
# machine's sensor= / sensor1= and the ingenic-isp model, and an
# IMP_Encoder client (qemu-boot/ingenic-t23-enc.c).  With one sensor the
# rootfs's own tx-isp-t23.ko, sensor module and libimp.so; with two the
# T23 SDK's tx-isp-t23-double firmware (OpenIPC builds none), linked here
# into a module the way the SDK's Makefile does, its sensor modules and
# the client linked statically against its libimp.a.  The H.264 streams
# come back through RAM (pmemsave) and are decoded on the host with
# ffmpeg -err_detect explode: every picture, no decoder message, and the
# first and last pictures of each stream are what the ISP painted
# (qemu-boot/ingenic-isp-check.py) -- the second sensor's with its frame
# counter 500000 ahead, so that a picture from the wrong pipe fails.
#
#   bash qemu-boot/test-ingenic-t23-enc.sh [--direct 0|1|2]
#        [--sensor1 gc2053s1] [--frames N] [--fw DIR] [--qemu PATH]
#        [--ffmpeg CMD] [--out DIR] [--keep]
#
# --direct 1 asks the main stream's encoder channel for the IVDC (and
# loads tx-isp-t23 with direct_mode=1).  --sdk uses the T23 SDK's own
# tx-isp-t23.ko, sensor module and libimp.a instead of OpenIPC's.
# --sensor1 gc2053s1 loads the SDK's dual-sensor firmware; its raw pipe
# (the VIC caching both sensors' raw frames for one ISP) is not modelled,
# so that run does not stream yet (docs/ingenic.md); with it --direct 2
# would be the SDK's "double direct" mode.
# The double firmware needs a 2.10 tuning file: OpenIPC's first T23
# gc2053-t23.bin, fetched from its firmware repository (or BIN210=FILE).
# QEMU_EXTRA in the environment adds QEMU options (its stderr goes to
# <out>/qemu.err).  The SDK is not public: the test skips (exit 0)
# without it.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=/home/ai/dimerr/local/ingenic/fw/t23
TC=/home/ai/toolchains/ingenic/mips-gcc540-uclibc0.9.33.2-64bit-r3.3.0.smaller/bin
SDK23=/home/ai/dimerr/local/ingenic/extract/Ingenic-SDK-T23-1.1.0-20230726
FFMPEG=${FFMPEG:-}
DIRECT=0
SENSOR=gc2053
SENSOR1=
SDKSTACK=
FRAMES=10
OUT=
KEEP=
TIMEOUT=900

while [ $# -gt 0 ]; do
    case $1 in
    --direct) DIRECT=$2; shift ;;
    --sensor1) SENSOR1=$2; shift ;;
    --sdk) SDKSTACK=1 ;;
    --frames) FRAMES=$2; shift ;;
    --fw) FW=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --ffmpeg) FFMPEG=$2; shift ;;
    --out) OUT=$2; shift ;;
    --keep) KEEP=1 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
DRV=$SDK23/opensource/drivers
ISP1=$DRV/isp-t23/tx-isp-t23
ISP2=$DRV/isp-t23/tx-isp-t23-double
SEN=$DRV/sensors-t23/gc2053
SEN2=$DRV/sensors-t23/gc2053-double
for f in "$SDK23/sdk/include/imp/imp_encoder.h" \
         ${SENSOR1:+"$ISP2/libt23-firmware.a"} \
         ${SDKSTACK:+"$ISP1/tx-isp-t23.ko" "$ISP1/tx-isp-t23.mod.o" "$SDK23/sdk/lib/uclibc/libimp.a"}; do
    [ -e "$f" ] || { echo "SKIP: missing $f"; exit 0; }
done
for f in "$FW/uImage.t23" "$FW/rootfs.squashfs.t23" "$QEMU" \
         "$TC/mips-linux-uclibc-gcc"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
case $SENSOR1 in
'') NSTREAM=2 ;;
gc2053s1) NSTREAM=4 ;;
*) echo "--sensor1: gc2053s1 only (the SDK's second-sensor driver)" >&2
   exit 2 ;;
esac
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; [ -n "$KEEP" ] && echo "work dir kept: $WORK" || rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

echo "== rootfs with the encoder client"
unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.t23" >/dev/null
MOD=$(dirname "$(find "$WORK/rfs/lib/modules" -name tx-isp-t23.ko | head -1)")
mkdir -p "$WORK/rfs/lib/sdk"
TCC="$TC/mips-linux-uclibc-gcc"
[ -n "$SENSOR1" ] && SDKSTACK=1
if [ -n "$SDKSTACK" ]; then
    # Two sensors: the SDK's double firmware (OpenIPC builds none); one
    # sensor with --sdk: the SDK's own tx-isp-t23.ko.
    # tx-isp-t23.o is the firmware object plus the SDK's tx-isp-debug.o /
    # tx-isp-module.o, the module its .mod.o (the same in both builds: no
    # symbol versions, the same name), with the SDK's sensor modules and
    # libimp.a
    if [ -n "$SENSOR1" ]; then
        "$TC/mips-linux-uclibc-ld" -m elf32ltsmip -r -o "$WORK/dbl.o" \
            "$ISP2/libt23-firmware.a" "$ISP1/tx-isp-debug.o" "$ISP1/tx-isp-module.o"
        "$TC/mips-linux-uclibc-ld" -m elf32ltsmip -r \
            -T "$SDK23/opensource/kernel/scripts/module-common.lds" --build-id \
            -o "$WORK/rfs/lib/sdk/tx-isp-t23.ko" "$WORK/dbl.o" "$ISP1/tx-isp-t23.mod.o"
        cp "$SEN2/sensor_gc2053_t23.ko" "$SEN2/sensor_gc2053s1_t23.ko" "$WORK/rfs/lib/sdk/"
    else
        cp "$ISP1/tx-isp-t23.ko" "$SEN/sensor_gc2053_t23.ko" "$WORK/rfs/lib/sdk/"
    fi
    # The SDK's firmware (H20230720a, H20230726b-double) reads tuning
    # files of format 2.10 and fails on the rootfs's 2.20 ones ("The version of bin doesn't
    # match with driver!", then a kfree BUG in tiziano_load_parameters):
    # OpenIPC's first T23 package carried a 2.10 gc2053-t23.bin.  The
    # second sensor's tuning goes by the s1 driver's name.
    BIN210=${BIN210:-$WORK/gc2053-t23-2.10.bin}
    [ -s "$BIN210" ] || curl -fsSL --retry 3 -o "$BIN210" \
        "https://raw.githubusercontent.com/OpenIPC/firmware/006ba82bdd/general/package/ingenic-osdrv-t23/files/sensor/params/gc2053-t23.bin" ||
        { echo "SKIP: no 2.10 gc2053-t23.bin"; exit 0; }
    cp "$BIN210" "$WORK/rfs/etc/sensor/gc2053-t23.bin"
    cp "$BIN210" "$WORK/rfs/etc/sensor/gc2053s1-t23.bin"
    rm -f "$MOD"/sensor_*_t23.ko     # room for the client in the 8 MiB root
    $TCC -O2 -Wall -I"$SDK23/sdk/include" -static -s -o "$WORK/rfs/usr/bin/t23-enc" \
        "$HERE/ingenic-t23-enc.c" "$SDK23/sdk/lib/uclibc/libimp.a" \
        "$SDK23/sdk/lib/uclibc/libalog.a" "$SDK23/sdk/lib/uclibc/libsysutils.a" \
        -lpthread -lm -lrt -ldl -lstdc++ 2>&1 | grep -v -i warning || true
else
    # One sensor: OpenIPC's own tx-isp-t23.ko, sensor module and libimp
    # (the SDK 1.1.0 module cannot read OpenIPC's 2.20 tuning files)
    cp "$MOD/tx-isp-t23.ko" "$MOD/sensor_gc2053_t23.ko" "$WORK/rfs/lib/sdk/"
    S=$($TCC -print-sysroot)/usr/lib
    $TCC -O2 -Wall -march=mips32r2 -I"$SDK23/sdk/include" -DSNAP_MUSL -c \
        -o "$WORK/enc.o" "$HERE/ingenic-t23-enc.c"
    $TCC -nostdlib -o "$WORK/rfs/usr/bin/t23-enc" \
        "$S/crt1.o" "$S/crti.o" "$($TCC -print-file-name=crtbegin.o)" \
        "$WORK/enc.o" -L"$WORK/rfs/usr/lib" -L"$WORK/rfs/lib" \
        -limp -lalog -l:libc.so "$($TCC -print-libgcc-file-name)" \
        "$($TCC -print-file-name=crtend.o)" "$S/crtn.o" \
        -Wl,--dynamic-linker=/lib/ld-musl-mipsel.so.1 \
        -Wl,--export-dynamic -Wl,--allow-shlib-undefined
    "$TC/mips-linux-uclibc-strip" "$WORK/rfs/usr/bin/t23-enc"
fi
# libimp's buffers in rmem, the streams' 4 MiB slots in the top 16 MiB,
# outside both (the encoder runs on after the client is done)
RAM=128
MEM="mem=64M rmem=48M@0x4000000"
cat > "$WORK/rfs/usr/bin/t23-test" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t tmpfs tmp /var
mkdir -p /dev/shm && mount -t tmpfs shm /dev/shm
mkdir -p /var/lock
echo "T23TEST sinfo: \$(modprobe sinfo; echo 1 > /proc/jz/sinfo/info; cat /proc/jz/sinfo/info)"
cd /lib/sdk
insmod tx-isp-t23.ko isp_clk=200000000 direct_mode=$DIRECT
insmod sensor_gc2053_t23.ko data_interface=1 sensor_max_fps=25
${SENSOR1:+insmod sensor_gc2053s1_t23.ko data_interface=1 sensor_max_fps=25}
cd /tmp
echo "T23TEST modules: \$(lsmod | sed -n 's/^\\(tx_isp[^ ]*\\|sensor_[^ ]*\\).*/\\1/p' | tr '\\n' ' ')"
touch /tmp/closesimd
export SNAP_DEVMEM=$(( (RAM - 16) << 20 ))
t23-enc $FRAMES $DIRECT $SENSOR 0x37 ${SENSOR1:+$SENSOR1 0x3f} &
ENC=\$!
for i in \$(seq 900); do [ -e /tmp/t23-enc.done ] && break; kill -0 \$ENC 2>/dev/null || break; sleep 1; done
echo "T23TEST ivdc: \$(grep -E "Frame Done|Overflow" /proc/jz/isp/isp-ivdc 2>/dev/null | tr -s " " | tr "\\n" ";")"
echo "T23TEST interrupts: \$(grep -E "isp|ivdc|vpu|helix" /proc/interrupts | tr -s " " | tr "\\n" ";")"
echo T23TEST-PMEM
sleep 20        # the host saves the streams meanwhile
echo "T23TEST rc=\$([ -e /tmp/t23-enc.done ] && echo 0 || echo 1)"
echo T23TEST-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/t23-test"
echo "/dev/mtd1 0x0 0x10000 0x8000" > "$WORK/rfs/etc/fw_env.config"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
python3 - "$WORK/nor.bin" "$FW/uImage.t23" "$WORK/rootfs.squashfs" \
    "sensor=$SENSOR" "soc=t23" <<'PY'
import struct, sys, zlib
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    assert off != 0x250000 or len(data) <= 0x800000, 'the root does not fit'
    img[off:off + len(data)] = data
env = b''.join(kv.encode() + b'\0' for kv in sys.argv[4:]) + b'\0'
env += b'\0' * (0x10000 - 4 - len(env))
img[0x40000:0x50000] = struct.pack('<I', zlib.crc32(env)) + env
open(sys.argv[1], 'wb').write(img)
PY

case "$FFMPEG" in
'') if command -v ffmpeg >/dev/null; then FFMPEG=ffmpeg
    elif docker image inspect jrottenberg/ffmpeg:6.1-alpine >/dev/null 2>&1; then
        FFMPEG="docker run --rm -i jrottenberg/ffmpeg:6.1-alpine"
    fi ;;
esac
for try in 1 2 3; do
    echo "== boot (t23, sensor=$SENSOR${SENSOR1:+, sensor1=$SENSOR1}, direct_mode=$DIRECT)${try#1}"
    rm -f "$OUT/console.log" "$WORK"/pmem*.bin
    "$QEMU" -M t23,sensor=$SENSOR${SENSOR1:+,sensor1=$SENSOR1},flash-file="$WORK/nor.bin" \
        -m ${RAM}M -display none -serial file:"$OUT/console.log" \
        -monitor unix:"$WORK/mon.sock",server,nowait \
        -nic user,restrict=on -kernel "$FW/uImage.t23" $QEMU_EXTRA \
        -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/t23-test mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) $MEM" \
        2> "$OUT/qemu.err" &
    QPID=$!
    started=
    saved=
    for i in $(seq "$TIMEOUT"); do
        end=
        grep -q T23TEST-END "$OUT/console.log" 2>/dev/null && end=1
        # the streams sit in RAM from T23TEST-PMEM on; QEMU runs on for the
        # console's drain after T23TEST-END, so a late look still finds them
        if [ -z "$saved" ] && { [ -n "$end" ] ||
                grep -q T23TEST-PMEM "$OUT/console.log" 2>/dev/null; }; then
            saved=1
            python3 - "$WORK/mon.sock" "$WORK" $RAM $NSTREAM <<'PY'
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
for slot in range(int(sys.argv[4])):
    s.sendall(b'pmemsave %d %d "%s/pmem%d.bin"\n' %
              ((int(sys.argv[3]) - 16 + 4 * slot) << 20, 4 << 20,
               sys.argv[2].encode(), slot))
    time.sleep(2)
s.close()
PY
        fi
        grep -q "T23TEST sinfo" "$OUT/console.log" 2>/dev/null && started=1
        [ -n "$end" ] && break
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
grep -a -E "T23TEST|^stream[0-3] picture [0-9]*:|t23-enc:|failed" "$OUT/console.log" | grep -v PMEM | tail -40 || true
if ! grep -a "t23-enc: PASS" "$OUT/console.log" >/dev/null; then
    note=
    [ -n "$SENSOR1" ] && note=" (the T23 dual-sensor raw pipe is not modelled)"
    echo "FAIL: the client did not get its streams$note"
    exit 1
fi
[ -n "$FFMPEG" ] || { echo "SKIP: no ffmpeg to decode with"; exit 0; }
fails=0
for slot in $(seq 0 $((NSTREAM - 1))); do
    f=$OUT/stream$slot.h264
    python3 -c "import struct,sys; d=open(sys.argv[1],'rb').read(); n=struct.unpack('<I',d[:4])[0]; open(sys.argv[2],'wb').write(d[4:4+n])" "$WORK/pmem$slot.bin" "$f"
    if [ $((slot % 2)) = 0 ]; then sz=1920x1080; else sz=640x360; fi
    # the second sensor's pictures carry their counter 500000 ahead
    base=0; [ $slot -ge 2 ] && base=500000
    rm -f "$WORK/dec.nv12"
    $FFMPEG -v error -err_detect explode -f h264 -i - -f rawvideo \
        -pix_fmt nv12 - < "$f" > "$WORK/dec.nv12" 2> "$WORK/ff.log" || true
    errs=$(grep -v -c '^docker:' "$WORK/ff.log" || true)
    fs=$(( ${sz%x*} * ${sz#*x} * 3 / 2 ))
    got=$(( $(stat -c %s "$WORK/dec.nv12" 2>/dev/null || echo 0) / fs ))
    echo "stream$slot ($sz): $(stat -c %s "$f") bytes, $got of $FRAMES pictures decoded, $errs decoder messages"
    if [ "$errs" != 0 ] || [ "$got" != "$FRAMES" ]; then
        fails=$((fails + 1)); grep -v '^docker:' "$WORK/ff.log" | head -3; continue
    fi
    python3 - "$WORK/dec.nv12" $fs $got "$WORK" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read(); fs = int(sys.argv[2]); n = int(sys.argv[3])
open(sys.argv[4] + '/first.nv12', 'wb').write(d[:fs])
open(sys.argv[4] + '/last.nv12', 'wb').write(d[(n - 1) * fs:n * fs])
PY
    if [ $sz = 640x360 ]; then
        # the 640x360 stream's counter cells are 6 pixels: legible at QP 30
        python3 "$HERE/ingenic-isp-check.py" --advance $sz:"$WORK/first.nv12" \
            $sz:"$WORK/last.nv12" > "$WORK/chk.log" || true
    else
        python3 "$HERE/ingenic-isp-check.py" --advance \
            $sz:"$WORK/first.nv12@$base" $sz:"$WORK/last.nv12@$base" \
            > "$WORK/chk.log" || true
    fi
    sed 's/^/    /' "$WORK/chk.log"
    grep -q '^PASS' "$WORK/chk.log" || fails=$((fails + 1))
done
if [ "$DIRECT" != 0 ]; then
    # the IVDC took the main stream(s): it counted the ISP's pictures into
    # its store
    n=$(grep -a -o "ISP Frame Done *: *[0-9]*" "$OUT/console.log" | grep -o "[0-9]*$" | head -1)
    if [ "${n:-0}" -gt 0 ]; then
        echo "ok   the IVDC counted $n ISP frames done"
    else
        echo "FAIL the IVDC counted ${n:-no} ISP frames done"; fails=$((fails + 1))
    fi
fi
[ $fails = 0 ] && echo PASS || { echo "FAIL ($fails)"; exit 1; }
