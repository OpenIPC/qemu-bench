#!/bin/bash
# The AVPU encoder model (hw/misc/ingenic-avpu.c) end to end: OpenIPC's
# kernel and rootfs, the vendor modules loaded by the rootfs's own
# load_ingenic (avpu.ko among them), the machine's sensor= and the
# ingenic-isp model, and an IMP_Encoder client (qemu-boot/ingenic-avpu-enc.c,
# built against the rootfs's own libimp.so) that encodes the sensor's
# frames at its size and at 640x360 -- H.264, H.265 and JPEG, one boot
# each.  The streams come back through rmem (pmemsave) and are decoded on
# the host with ffmpeg -err_detect explode: every picture, no decoder
# message, and the first and last decoded pictures' frame counter, logo
# and dot are what the ISP painted (qemu-boot/ingenic-isp-check.py).
#
#   bash qemu-boot/test-ingenic-avpu.sh [--soc t40|t31|t41] [--sensor NAME]
#        [--codecs "h264 h265 jpeg"] [--frames N] [--fw DIR] [--qemu PATH]
#        [--image SDK.img] [--ffmpeg CMD] [--out DIR] [--osd]
#
# --osd (T41): channel 0 goes through a libimp OSD group with a red
# rectangle and a green line, which libimp draws with the DRAWBOX
# (hw/misc/ingenic-drawbox.c); the decoded pictures must show both.  Not
# T40 here: OpenIPC's T40 kernel has no drawbox driver, so libimp draws
# no boxes (test-ingenic-xb2blocks.sh covers the T40 SDK kernel's).
#
# The same machinery as test-ingenic-isp.sh (its options, sensors and
# toolchains); T40 is the default here.  --ffmpeg defaults to ffmpeg on
# PATH, else the jrottenberg/ffmpeg:6.1-alpine docker image.
#
# T41 (OpenIPC has no T41 release) boots the T41 SDK's own flash image as
# test-ingenic-isp.sh does: its kernel by -kernel, the vendor modules from
# its JFFS2, a rebuilt root as a fifth partition, and the client linked
# statically against the SDK's libimp.a; the image is not public, so the
# test skips (exit 0) without it.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=$HERE
TC=/home/ai/toolchains/ingenic/mips-gcc540-uclibc0.9.33.2-64bit-r3.3.0.smaller/bin
# T40's libimp is built for a 64-bit FPU and NaN2008: the GCC 7.2 one
TC40=/home/ai/toolchains/ingenic/mips-linux-gnu-ingenic-gcc7.2.0-glibc2.29-fp64-r5.1.8/bin
CROSS=mips-linux-uclibc-
CCFLAGS=
SDK=/home/ai/dimerr/local/ingenic/extract/isvp-t31/software/sdk/Ingenic-SDK-T31-1.1.5-20220514/sdk/5.4.0
# the IMP headers of the client: the SoC's own SDK where their structs
# differ (T23's IMPSensorInfo has a sensor ID and an I2C adapter)
INC23=/home/ai/dimerr/local/ingenic/extract/Ingenic-SDK-T23-1.1.0-20230726/sdk/include
INC40=/home/ai/dimerr/tmp-scratch/isp-re-t40/sdk/Ingenic-SDK-T40-1.2.0-20220609/sdk/include
# T41: the SDK's own flash image and IMP (OpenIPC has no T41 release)
SDK41=/home/ai/dimerr/tmp-scratch/t41sdk/Ingenic-SDK-T41-1.2.0-20230713/sdk
IMAGE41=/home/ai/dimerr/local/ingenic/extract/t41-sdk-full/Ingenic-SDK-T41-1.2.0-20230713/resource/image_t41/T41NQ_gcc720_uclibc_16M_202307061021.img
IMAGE=
VARIANT=
INC=
DEFS=
RAM=64M
MEM="mem=40M rmem=24M@0x2800000"
RMEM_MB=40
LIBIMP=rootfs
SOC=t40
CODECS="h264 h265 jpeg"
FFMPEG=
SENSOR=
FRAMES=10
OUT=
PNG=
TIMEOUT=900
OSD=

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --sensor) SENSOR=$2; shift ;;
    --fw) FW=$2; shift ;;
    --image) IMAGE=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --toolchain) TC=$2; shift ;;
    --sdk) SDK=$2; shift ;;
    --include) INC=$2; shift ;;
    --libimp) LIBIMP=$2; shift ;;
    --frames) FRAMES=$2; shift ;;
    --out) OUT=$2; shift ;;
    --png) PNG=1 ;;
    --osd) OSD=1 ;;
    --timeout) TIMEOUT=$2; shift ;;
    --codecs) CODECS=$2; shift ;;
    --ffmpeg) FFMPEG=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done

case $SOC in
t31) SENSOR=${SENSOR:-gc2053} ;;
t21) SENSOR=${SENSOR:-jxf37} ;;
t23) SENSOR=${SENSOR:-gc2053}; INC=${INC:-$INC23} ;;
t20) SENSOR=${SENSOR:-jxf22} ;;
t30) SENSOR=${SENSOR:-ov2735b} ;;
t40) SENSOR=${SENSOR:-gc4653}; INC=${INC:-$INC40}; DEFS=-DSNAP_T40
     TC=$TC40; CROSS=mips-linux-gnu-; CCFLAGS="-muclibc -fno-use-linker-plugin"
     RAM=128M; MEM="mem=64M rmem=64M@0x4000000"; RMEM_MB=64 ;;
t41) SENSOR=${SENSOR:-gc4663}; INC=${INC:-$SDK41/include}; LIBIMP=sdk41
     DEFS="-DSNAP_T40 -DSNAP_I2C_ADAPTER=0 -DSNAP_MCLK=IMPISP_SENSOR_MCLK0"
     TC=$TC40; CROSS=mips-linux-gnu-; CCFLAGS="-muclibc -fno-use-linker-plugin"
     IMAGE=${IMAGE:-$IMAGE41}
     RAM=128M; MEM="mem=48M@0x0 rmem=64M@0x3000000 nmem=16M@0x7000000"
     RMEM_MB=48; VARIANT=,variant=t41nq ;;
*) echo "no AVPU on $SOC (T31, T40, T41 here)" >&2; exit 2 ;;
esac
case $SENSOR in
gc2053|gc2083) ADDR=0x37 ;;
sc2336|sc2335) ADDR=0x30 ;;
jxf37|jxf22) ADDR=0x40 ;;
ov2735b|ov2735) ADDR=0x3c ;;
gc4653|gc4663) ADDR=0x29 ;;
*) echo "unknown sensor $SENSOR" >&2; exit 2 ;;
esac
if [ $SOC = t41 ]; then
    # vendor material that is not public: skip, not fail, without it
    for f in "$IMAGE" "$SDK41/lib/uclibc/libimp.a" "$TC/${CROSS}gcc"; do
        [ -e "$f" ] || { echo "SKIP: missing $f"; exit 0; }
    done
    NEED=
else
    NEED="$FW/uImage.$SOC $FW/rootfs.squashfs.$SOC $SDK/include/imp/imp_framesource.h"
fi
for f in $NEED "$QEMU" "$TC/${CROSS}gcc"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; [ -n "$KEEP" ] && echo "work dir kept: $WORK" || rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

echo "== rootfs with the capture client ($LIBIMP libimp)"
if [ $SOC = t41 ]; then
    # the image: SPL + U-Boot, the uImage at 0x40000, the root SquashFS at
    # 0x2C0000 and the vendor modules in the JFFS2 at 0x4C0000 (/system)
    python3 - "$IMAGE" "$WORK" <<'PY'
import struct, sys
d = open(sys.argv[1], 'rb').read()
n = struct.unpack('>I', d[0x40000 + 12:0x40000 + 16])[0]
open(sys.argv[2] + '/uImage', 'wb').write(d[0x40000:0x40000 + 64 + n])
open(sys.argv[2] + '/root.sq', 'wb').write(d[0x2C0000:0x4C0000])
PY
    unsquashfs -q -d "$WORK/rfs" "$WORK/root.sq" >/dev/null
    W=2560; H=1440
else
    unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.$SOC" >/dev/null
    read W H <<< "$(sed -n 's/^ *\(width\|height\): *\([0-9]*\).*/\2/p' \
        "$WORK/rfs/etc/sensor/$SENSOR.yaml" | tr '\n' ' ')"
fi
W=${W:-1920}
H=${H:-1080}
CC="$TC/${CROSS}gcc $CCFLAGS"
SRC=$HERE/ingenic-avpu-enc.c
if [ -n "$OSD" ]; then
    case $SOC in t41) DEFS="$DEFS -DENC_OSD" ;;
    *) echo "--osd is for T41 (OpenIPC's T40 kernel has no drawbox driver)" >&2
       exit 2 ;; esac
fi
if [ "$LIBIMP" = sdk41 ]; then
    $CC -O2 -Wall -I"$INC" $DEFS -static -s -o "$WORK/rfs/usr/bin/avpu-enc" \
        "$SRC" "$SDK41/lib/uclibc/libimp.a" "$SDK41/lib/uclibc/libalog.a" \
        "$SDK41/lib/uclibc/libsysutils.a" -lpthread -lm -lrt -ldl -lstdc++ \
        2>&1 | grep -v -i warning || true
elif [ "$LIBIMP" = sdk ]; then
    $CC -O2 -Wall -march=mips32r2 -muclibc -I"$SDK/include" -static -s \
        -o "$WORK/rfs/usr/bin/avpu-enc" "$SRC" "$SDK/lib/uclibc/libimp.a" \
        "$SDK/lib/uclibc/libalog.a" -lpthread -lm -lrt -ldl -lstdc++
else
    S=$($CC -print-sysroot)/usr/lib
    # T30's libimp is partly C++: the rootfs has musl's libstdc++
    LIBCXX=
    [ -e "$WORK/rfs/usr/lib/libstdc++.so.6" ] && \
        LIBCXX="-l:libstdc++.so.6 -l:libgcc_s.so.1"
    $CC -O2 -Wall -march=mips32r2 -I"${INC:-$SDK/include}" -DSNAP_MUSL $DEFS -c \
        -o "$WORK/snap.o" "$SRC"
    $CC -nostdlib -o "$WORK/rfs/usr/bin/avpu-enc" \
        "$S/crt1.o" "$S/crti.o" "$($CC -print-file-name=crtbegin.o)" \
        "$WORK/snap.o" -L"$WORK/rfs/usr/lib" -L"$WORK/rfs/lib" \
        -limp -lalog $LIBCXX -l:libc.so "$($CC -print-libgcc-file-name)" \
        "$($CC -print-file-name=crtend.o)" "$S/crtn.o" \
        -Wl,--dynamic-linker=/lib/ld-musl-mipsel.so.1 \
        -Wl,--export-dynamic -Wl,--allow-shlib-undefined
    "$TC/${CROSS}strip" "$WORK/rfs/usr/bin/avpu-enc"
fi
if [ $SOC = t41 ]; then
    # the SDK image's start.sh, without its application; the ISP without
    # direct_mode (IVDC, the ISP-to-encoder path, is not modelled)
    LOADER='mount -t jffs2 /dev/mtdblock3 /system
cd /system/lib/modules
insmod avpu.ko
insmod sinfo.ko
echo "AVPUTEST sinfo: $(echo 1 > /proc/jz/sinfo/info; cat /proc/jz/sinfo/info)"
insmod tx-isp-t41.ko isp_clk=300000000
insmod sensor_'$SENSOR'_t41.ko
cd /tmp'
else
    LOADER='echo "AVPUTEST sinfo: $(modprobe sinfo; echo 1 > /proc/jz/sinfo/info; cat /proc/jz/sinfo/info)"
load_ingenic -i'
fi
cat > "$WORK/rfs/usr/bin/avpu-test" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t tmpfs tmp /var
mkdir -p /dev/shm && mount -t tmpfs shm /dev/shm
mkdir -p /var/lock
CODEC=\$(sed -n 's/.*avpu_codec=\([a-z0-9]*\).*/\1/p' /proc/cmdline)
$LOADER
echo "AVPUTEST modules: \$(lsmod | sed -n 's/^\\(avpu[^ ]*\\|tx_isp[^ ]*\\|sensor_[^ ]*\\).*/\\1/p' | tr '\\n' ' ')"
touch /tmp/closesimd
${OSD:+export AVPU_OSD=1}
# T40's SDHCI finds no card: the streams go to the top 8 MiB of RAM too,
# above libimp's buffers, for the host's pmemsave
[ -b /dev/mmcblk0 ] || export SNAP_DEVMEM=$(( (${RAM%M} - 8) << 20 ))
avpu-enc \$CODEC $FRAMES /tmp $SENSOR $ADDR $W $H &
ENC=\$!
for i in \$(seq 900); do [ -e /tmp/avpu-enc.done ] && break; kill -0 \$ENC 2>/dev/null || break; sleep 1; done
echo "AVPUTEST interrupts: \$(grep -E "avpu|dbox|drawbox|i2d" /proc/interrupts | tr -s " " | tr "\\n" ";")"
if [ -b /dev/mmcblk0 ]; then
    for n in 0 1; do dd if=/tmp/slot\$n.bin of=/dev/mmcblk0 bs=4M seek=\$n conv=sync 2>/dev/null; done
    sync
else
    echo AVPUTEST-PMEM
    sleep 20        # the host saves the streams meanwhile
fi
echo "AVPUTEST rc=\$([ -e /tmp/avpu-enc.done ] && echo 0 || echo 1)"
echo AVPUTEST-END
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/avpu-test"
# fw_printenv writes this on first use, but the root is read-only here
echo "/dev/mtd1 0x0 0x10000 0x8000" > "$WORK/rfs/etc/fw_env.config"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
truncate -s 64M "$WORK/sd.img"
KERNEL=$FW/uImage.$SOC
ROOTDEV=/dev/mtdblock3
MTDPARTS="jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data)"
if [ $SOC = t41 ]; then
    # the image with this root at 12 MiB, a partition of its own (the
    # JFFS2 ends below 8 MiB)
    KERNEL=$WORK/uImage
    ROOTDEV=/dev/mtdblock4
    MTDPARTS="sfc0_nor:256k(boot),2560k(kernel),2048k(root),7424k(appfs),-(test)"
    python3 - "$WORK/nor.bin" "$IMAGE" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(open(sys.argv[2], 'rb').read())
r = open(sys.argv[3], 'rb').read()
assert len(r) <= 0x400000, 'the root does not fit in 4 MiB'
img[0xC00000:0xC00000 + len(r)] = r
open(sys.argv[1], 'wb').write(img)
PY
else
# the NOR: OpenIPC's layout, a U-Boot environment that names the sensor
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
fi

case "$FFMPEG" in
'') if command -v ffmpeg >/dev/null; then FFMPEG=ffmpeg
    elif docker image inspect jrottenberg/ffmpeg:6.1-alpine >/dev/null 2>&1; then
        FFMPEG="docker run --rm -v $WORK:$WORK jrottenberg/ffmpeg:6.1-alpine"
    fi ;;
esac
fails=0
for CODEC in $CODECS; do
# A boot now and then stops dead after the SD card is detected, before
# init prints anything; such a boot is retried.
for try in 1 2 3; do
    echo "== boot ($SOC, sensor=$SENSOR, $CODEC)${try#1}"
    rm -f "$OUT/console-$CODEC.log" "$WORK"/pmem*.bin
    truncate -s 0 "$WORK/sd.img"; truncate -s 64M "$WORK/sd.img"
    "$QEMU" -M $SOC,sensor=$SENSOR$VARIANT,flash-file="$WORK/nor.bin" -m $RAM \
        -display none -serial file:"$OUT/console-$CODEC.log" \
        -monitor unix:"$WORK/mon.sock",server,nowait \
        -nic user,restrict=on -kernel "$KERNEL" \
        -drive if=sd,index=0,file="$WORK/sd.img",format=raw $QEXTRA \
        -append "console=ttyS1,115200n8 root=$ROOTDEV rootfstype=squashfs init=/usr/bin/avpu-test avpu_codec=$CODEC mtdparts=$MTDPARTS $MEM" &
    QPID=$!
    started=
    saved=
    for i in $(seq "$TIMEOUT"); do
        grep -q AVPUTEST-END "$OUT/console-$CODEC.log" 2>/dev/null && break
        if [ -z "$saved" ] && grep -q AVPUTEST-PMEM "$OUT/console-$CODEC.log" 2>/dev/null; then
            saved=1
            python3 - "$WORK/mon.sock" "$WORK" ${RAM%M} <<'PY'
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
for slot in range(2):
    s.sendall(b'pmemsave %d %d "%s/pmem%d.bin"\n' %
              ((int(sys.argv[3]) - 8 + 4 * slot) << 20, 4 << 20,
               sys.argv[2].encode(), slot))
    time.sleep(2)
s.close()
PY
        fi
        grep -q "AVPUTEST sinfo" "$OUT/console-$CODEC.log" 2>/dev/null && started=1
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
grep -a -E "AVPUTEST|^enc[01] picture|avpu-enc:|^osd:" "$OUT/console-$CODEC.log" | grep -v PMEM || true
if [ ! -e "$WORK/pmem1.bin" ]; then
    # the SD card's slots instead
    for slot in 0 1; do
        dd if="$WORK/sd.img" of="$WORK/pmem$slot.bin" bs=4M skip=$slot count=1 status=none
    done
fi
if ! grep -q "AVPUTEST rc=0" "$OUT/console-$CODEC.log"; then
    echo "FAIL: $CODEC: no streams from the client"; fails=$((fails + 1)); continue
fi
case $CODEC in jpeg) EXT=jpg; N=1 ;; h265) EXT=h265; N=$FRAMES ;; *) EXT=h264; N=$FRAMES ;; esac
for slot in 0 1; do
    f=$OUT/$CODEC-ch$slot.$EXT
    python3 -c "import struct,sys; d=open(sys.argv[1],'rb').read(); n=struct.unpack('<I',d[:4])[0]; open(sys.argv[2],'wb').write(d[4:4+n])" "$WORK/pmem$slot.bin" "$f"
    if [ $slot = 0 ]; then sz=${W}x$H; else sz=640x360; fi
    if [ -z "$FFMPEG" ]; then echo "SKIP: no ffmpeg for $f"; continue; fi
    cp "$f" "$WORK/in.$EXT"
    rm -f "$WORK/dec.nv12"
    $FFMPEG -v error -err_detect explode -i "$WORK/in.$EXT" -f rawvideo \
        -pix_fmt nv12 -y "$WORK/dec.nv12" > "$WORK/ff.log" 2>&1 || true
    errs=$(grep -v -c '^docker:' "$WORK/ff.log" || true)
    fs=$(( ${sz%x*} * ${sz#*x} * 3 / 2 ))
    got=$(( $(stat -c %s "$WORK/dec.nv12" 2>/dev/null || echo 0) / fs ))
    echo "$CODEC ch$slot: $(stat -c %s "$f") bytes, $got of $N pictures decoded, $errs decoder messages"
    if [ "$errs" != 0 ] || [ "$got" != "$N" ]; then
        fails=$((fails + 1)); grep -v '^docker:' "$WORK/ff.log" | head -3; continue
    fi
    # the first and the last picture against the ISP's scene
    python3 - "$WORK/dec.nv12" $fs $got "$WORK" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read(); fs = int(sys.argv[2]); n = int(sys.argv[3])
open(sys.argv[4] + '/first.nv12', 'wb').write(d[:fs])
open(sys.argv[4] + '/last.nv12', 'wb').write(d[(n - 1) * fs:n * fs])
PY
    if [ -n "$OSD" ] && [ $slot = 0 ]; then
        # the OSD's boxes, in the middle of their 6-pixel lines
        python3 - "$WORK/last.nv12" ${sz%x*} ${sz#*x} \
            "$(grep -a -m1 '^osd: ' "$OUT/console-$CODEC.log" | tr -d '\r')" <<'PY' ||
import sys
d = open(sys.argv[1], 'rb').read()
w, h = int(sys.argv[2]), int(sys.argv[3])
f = sys.argv[4].split()
if len(f) < 8:
    print('    OSD: the client reported no boxes: FAIL')
    sys.exit(1)
x, y, bw, bh, lx, ly, ll = map(int, f[1:8])
uv = w * h
def at(px, py):
    c = uv + (py // 2) * w + (px // 2) * 2
    return d[py * w + px], d[c], d[c + 1]
def avg(pts):
    v = [at(px, py) for px, py in pts]
    return tuple(sum(c[i] for c in v) / len(v) for i in range(3))
edge = avg([(x + i, y + 3) for i in range(16, bw - 16, 8)] +
           [(x + 3, y + j) for j in range(16, bh - 16, 8)])
inside = avg([(x + bw // 2 + i, y + bh // 2) for i in range(-8, 9, 4)])
line = avg([(lx + i, ly + 3) for i in range(16, ll - 16, 8)])
red = edge[2] > 200 and edge[1] < 120
green = line[1] < 80 and line[2] < 80
untouched = inside[2] < 180
print('    OSD rectangle edge YUV %.0f/%.0f/%.0f (red: %s), line %.0f/%.0f/%.0f '
      '(green: %s), inside Cr %.0f: %s' % (edge + (red,) + line + (green,) +
      (inside[2], 'PASS' if red and green and untouched else 'FAIL')))
sys.exit(0 if red and green and untouched else 1)
PY
        fails=$((fails + 1))
    fi
    if [ "$N" = 1 ]; then
        python3 "$HERE/ingenic-isp-check.py" ${PNG:+--png "$OUT/$CODEC-ch$slot"} \
            $sz:"$WORK/first.nv12" > "$WORK/chk.log" || true
    else
        python3 "$HERE/ingenic-isp-check.py" --advance ${PNG:+--png "$OUT/$CODEC-ch$slot"} \
            $sz:"$WORK/first.nv12" $sz:"$WORK/last.nv12" > "$WORK/chk.log" || true
    fi
    if [ "$CODEC" = h265 ] && [ $slot = 1 ]; then
        grep -v '^PASS$\|^FAIL$' "$WORK/chk.log" | sed 's/^/    /'
        # hevc-enc codes 8x8 DC blocks: at 640x360 the counter's 6-pixel
        # cells are below that, so only the logo and the moving dot count
        python3 - "$WORK/chk.log" <<'PY' || fails=$((fails + 1))
import re, sys
xs = []
for l in open(sys.argv[1]):
    m = re.search(r'logo (\d+)% bright, background Y ([\d.]+).*dot at x (\d+)', l)
    if m:
        if int(m.group(1)) < 15 or not 20 < float(m.group(2)) < 170:
            sys.exit(1)
        xs.append(int(m.group(3)))
ok = len(xs) == 2 and xs[0] != xs[1]
print('    logo and dot only (8x8 blocks at 640x360): %s' % ('PASS' if ok else 'FAIL'))
sys.exit(0 if ok else 1)
PY
    else
        sed 's/^/    /' "$WORK/chk.log"
        grep -q '^PASS' "$WORK/chk.log" || fails=$((fails + 1))
    fi
done
done
[ $fails = 0 ] && echo PASS || { echo "FAIL ($fails)"; exit 1; }
