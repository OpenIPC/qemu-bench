#!/bin/bash
# Frames from the emulated Ingenic image pipeline to userspace, end to
# end: OpenIPC's kernel and rootfs, the vendor modules loaded by the
# rootfs's own load_ingenic (sinfo, tx-isp-<soc>, sensor_<sensor>_<soc>;
# the sensor preset in the U-Boot environment, as the second boot of a
# camera has it), the machine's sensor= on I2C0 and the ingenic-isp model,
# and an IMP FrameSource client (qemu-boot/ingenic-isp-snap.c) that takes
# NV12 frames at the sensor's size and at 640x360 through libimp and
# writes four of them to an SD card image.  On the host,
# qemu-boot/ingenic-isp-check.py reads their frame counters, finds the
# logo and the moving dot, and checks that both channels advance.
#
#   bash qemu-boot/test-ingenic-isp.sh [--soc t31|t21|t23|t20|t10|t30|t40|t41] [--sensor NAME]
#        [--fw DIR] [--qemu PATH] [--toolchain BINDIR] [--sdk DIR]
#        [--libimp rootfs|sdk] [--frames N] [--out DIR] [--png]
#        [--isp-ko FILE]
#
# Sensors: t31 gc2053 (MIPI, the default), sc2336, gc2083; t21 jxf37 (DVP);
# t23 gc2053, sc2336; t20 jxf22; t10 jxh62 (720p, bit-banged I2C); t30 ov2735b; t40 gc4653 (MIPI, I2C1);
# t41 gc4663, from the T41 SDK's own flash image (--image).
# --include overrides the IMP header directory.  --isp-ko puts FILE in the
# rootfs in place of its tx-isp-<soc>.ko: OpenIPC's tx-isp-t30.ko oopses
# at libimp's first call (docs/ingenic.md), and the T30 SDK 1.0.5's
# prebuilt one (opensource/drivers/tx-isp-t30/tx-isp-t30.ko, the same
# H20190725b) streams with the release kernel, libimp and sensor module.
# --fw holds uImage.<soc> and rootfs.squashfs.<soc> (fetch-openipc-lite.sh
# <soc> DIR).  The client is built with the Ingenic uClibc toolchain
# (mips-gcc540-uclibc0.9.33.2) against the T31 SDK headers, and by
# default linked against the rootfs's own libimp.so (IMP 1.1.6, the
# build that goes with its tx-isp-t31.ko) and musl; --libimp sdk links
# the SDK's static uClibc libimp.a (1.1.5) instead.  The rootfs is
# rebuilt with the client and an init script and put on a 16 MiB NOR as
# the root (a RAM disk would take the 5 MiB a 1080p capture needs); RAM
# is OpenIPC's for 64 MiB (mem=40M rmem=24M@0x2800000), and libimp's MXU2
# paths are off (/tmp/closesimd), which the CPU model does not decode.
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
# T41: the SDK's own flash image (OpenIPC has no T41 release) and IMP
SDK41=/home/ai/dimerr/tmp-scratch/t41sdk/Ingenic-SDK-T41-1.2.0-20230713/sdk
IMAGE41=/home/ai/dimerr/local/ingenic/extract/t41-sdk-full/Ingenic-SDK-T41-1.2.0-20230713/resource/image_t41/T41NQ_gcc720_uclibc_16M_202307061021.img
IMAGE=
ISP_KO=
INC=
DEFS=
RAM=64M
MEM="mem=40M rmem=24M@0x2800000"
RMEM_MB=40
VARIANT=
LIBIMP=rootfs
SOC=t31
SENSOR=
FRAMES=40
OUT=
PNG=
TIMEOUT=900

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --sensor) SENSOR=$2; shift ;;
    --fw) FW=$2; shift ;;
    --image) IMAGE=$2; shift ;;
    --isp-ko) ISP_KO=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --toolchain) TC=$2; shift ;;
    --sdk) SDK=$2; shift ;;
    --include) INC=$2; shift ;;
    --libimp) LIBIMP=$2; shift ;;
    --frames) FRAMES=$2; shift ;;
    --out) OUT=$2; shift ;;
    --png) PNG=1 ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done

case $SOC in
t31) SENSOR=${SENSOR:-gc2053} ;;
t21) SENSOR=${SENSOR:-jxf37} ;;
t23) SENSOR=${SENSOR:-gc2053}; INC=${INC:-$INC23} ;;
t20) SENSOR=${SENSOR:-jxf22} ;;
t10) SENSOR=${SENSOR:-jxh62} ;;     # T10's ISP takes at most 1280x960
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
*) echo "no ISP test for $SOC" >&2; exit 2 ;;
esac
case $SENSOR in
gc2053|gc2083) ADDR=0x37 ;;
sc2336|sc2335) ADDR=0x30 ;;
jxf37|jxf22) ADDR=0x40 ;;
jxh62) ADDR=0x30 ;;
ov2735b|ov2735) ADDR=0x3c ;;
gc4653|gc4663) ADDR=0x29 ;;
*) echo "unknown sensor $SENSOR" >&2; exit 2 ;;
esac
if [ $SOC = t41 ]; then
    NEED="$IMAGE $SDK41/lib/uclibc/libimp.a"
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
if [ -n "$ISP_KO" ]; then
    KO=$(find "$WORK/rfs/lib/modules" -name "tx-isp-$SOC.ko" | head -1)
    [ -n "$KO" ] || { echo "no tx-isp-$SOC.ko in the rootfs" >&2; exit 1; }
    cp "$ISP_KO" "$KO"
    echo "   $KO <- $ISP_KO"
fi
CC="$TC/${CROSS}gcc $CCFLAGS"
SRC=$HERE/ingenic-isp-snap.c
if [ "$LIBIMP" = sdk41 ]; then
    $CC -O2 -Wall -I"$INC" $DEFS -static -s -o "$WORK/rfs/usr/bin/isp-snap" \
        "$SRC" "$SDK41/lib/uclibc/libimp.a" "$SDK41/lib/uclibc/libalog.a" \
        "$SDK41/lib/uclibc/libsysutils.a" -lpthread -lm -lrt -ldl -lstdc++ \
        2>&1 | grep -v -i warning || true
elif [ "$LIBIMP" = sdk ]; then
    $CC -O2 -Wall -march=mips32r2 -muclibc -I"$SDK/include" -static -s \
        -o "$WORK/rfs/usr/bin/isp-snap" "$SRC" "$SDK/lib/uclibc/libimp.a" \
        "$SDK/lib/uclibc/libalog.a" -lpthread -lm -lrt -ldl -lstdc++
else
    S=$($CC -print-sysroot)/usr/lib
    # T30's libimp is partly C++: the rootfs has musl's libstdc++
    LIBCXX=
    [ -e "$WORK/rfs/usr/lib/libstdc++.so.6" ] && \
        LIBCXX="-l:libstdc++.so.6 -l:libgcc_s.so.1"
    $CC -O2 -Wall -march=mips32r2 -I"${INC:-$SDK/include}" -DSNAP_MUSL $DEFS -c \
        -o "$WORK/snap.o" "$SRC"
    $CC -nostdlib -o "$WORK/rfs/usr/bin/isp-snap" \
        "$S/crt1.o" "$S/crti.o" "$($CC -print-file-name=crtbegin.o)" \
        "$WORK/snap.o" -L"$WORK/rfs/usr/lib" -L"$WORK/rfs/lib" \
        -limp -lalog $LIBCXX -l:libc.so "$($CC -print-libgcc-file-name)" \
        "$($CC -print-file-name=crtend.o)" "$S/crtn.o" \
        -Wl,--dynamic-linker=/lib/ld-musl-mipsel.so.1 \
        -Wl,--export-dynamic -Wl,--allow-shlib-undefined
    "$TC/${CROSS}strip" "$WORK/rfs/usr/bin/isp-snap"
fi
if [ $SOC = t41 ]; then
    # the SDK image's start.sh, without its application
    LOADER='mount -t jffs2 /dev/mtdblock3 /system
cd /system/lib/modules
insmod avpu.ko
insmod sinfo.ko
echo "ISPTEST sinfo: $(echo 1 > /proc/jz/sinfo/info; cat /proc/jz/sinfo/info)"
insmod tx-isp-t41.ko isp_clk=300000000
insmod sensor_'$SENSOR'_t41.ko
cd /tmp'
else
    LOADER='echo "ISPTEST sinfo: $(modprobe sinfo; echo 1 > /proc/jz/sinfo/info; cat /proc/jz/sinfo/info)"
load_ingenic -i'
fi
cat > "$WORK/rfs/usr/bin/isp-test" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t tmpfs tmp /var
mkdir -p /dev/shm && mount -t tmpfs shm /dev/shm
mkdir -p /var/lock
$LOADER
echo "ISPTEST modules: \$(lsmod | sed -n 's/^\\(tx_isp[^ ]*\\|sensor_[^ ]*\\).*/\\1/p' | tr '\\n' ' ')"
touch /tmp/closesimd
# T23's MSC driver is one of the modules load_ingenic loads
for i in 1 2 3 4 5 6 7 8 9 10; do [ -b /dev/mmcblk0 ] && break; sleep 1; done
OUTDEV=/dev/mmcblk0
[ -b \$OUTDEV ] || OUTDEV=/tmp
[ "\$OUTDEV" = /tmp ] && export SNAP_DEVMEM=$(( (RMEM_MB + 32) << 20 ))
isp-snap $FRAMES \$OUTDEV $SENSOR $ADDR $W $H &
SNAP=\$!
# wait for the frames, not for the teardown (T40's can hang)
for i in \$(seq 600); do [ -e /tmp/isp-snap.done ] && break; kill -0 \$SNAP 2>/dev/null || break; sleep 1; done
sleep 1
if [ "\$OUTDEV" = /tmp ]; then
    # no SD card (T40's SDHCI sees none): the client put the frames in
    # rmem's top half, which the host reads with pmemsave
    echo ISPTEST-PMEM
    sleep 30            # the host saves them meanwhile
fi
echo "ISPTEST rc=\$(grep -q PASS /tmp/isp-snap.done 2>/dev/null && echo 0 || echo 1)"
echo "ISPTEST vic: \$(head -1 /proc/jz/isp/isp-w02 2>/dev/null || cat /proc/jz/vic/isp_vic_frd)"
sync
echo ISPTEST-END
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/isp-test"
# fw_printenv writes this on first use, but the root is read-only here
echo "/dev/mtd1 0x0 0x10000 0x8000" > "$WORK/rfs/etc/fw_env.config"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
truncate -s 64M "$WORK/sd.img"
KERNEL=$FW/uImage.$SOC
ROOTDEV=/dev/mtdblock3
MTDPARTS="jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data)"
if [ $SOC = t41 ]; then
    # the image with this root at 14 MiB, a partition of its own
    KERNEL=$WORK/uImage
    ROOTDEV=/dev/mtdblock4
    MTDPARTS="sfc0_nor:256k(boot),2560k(kernel),2048k(root),9472k(appfs),-(test)"
    python3 - "$WORK/nor.bin" "$IMAGE" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(open(sys.argv[2], 'rb').read())
r = open(sys.argv[3], 'rb').read()
assert len(r) <= 0x200000, 'the root does not fit in 2 MiB'
img[0xE00000:0xE00000 + len(r)] = r
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

# A boot now and then stops dead after the SD card is detected, before
# init prints anything (not the ISP: no vendor module is loaded yet);
# such a boot is retried.
for try in 1 2 3; do
    echo "== boot ($SOC, sensor=$SENSOR)${try#1}"
    rm -f "$OUT/console.log"
    # shellcheck disable=SC2086  # QEMU_EXTRA: extra options, word-split
    "$QEMU" -M $SOC,sensor=$SENSOR$VARIANT,flash-file="$WORK/nor.bin" -m $RAM \
        $QEMU_EXTRA -display none -serial file:"$OUT/console.log" \
        -monitor unix:"$WORK/mon.sock",server,nowait \
        -nic user,restrict=on -kernel "$KERNEL" \
        -drive if=sd,index=0,file="$WORK/sd.img",format=raw \
        -append "console=ttyS1,115200n8 root=$ROOTDEV rootfstype=squashfs init=/usr/bin/isp-test mtdparts=$MTDPARTS $MEM" &
    QPID=$!
    started=
    saved=
    for i in $(seq "$TIMEOUT"); do
        grep -q ISPTEST-END "$OUT/console.log" 2>/dev/null && break
        if [ -z "$saved" ] && grep -q ISPTEST-PMEM "$OUT/console.log"; then
            saved=1
            # the frames in rmem, 8 MiB apart
            python3 - "$WORK/mon.sock" "$WORK" $RMEM_MB <<'PY'
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
for slot in range(4):
    s.sendall(b'pmemsave %d %d "%s/pmem%d.nv12"\n' %
              ((int(sys.argv[3]) + 32 + 8 * slot) << 20, 8 << 20,
               sys.argv[2].encode(), slot))
    time.sleep(1)
s.close()
PY
        fi
        grep -q "ISPTEST sinfo" "$OUT/console.log" 2>/dev/null && started=1
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
grep -a -E "ISPTEST|^ch[01] frame|isp-snap:|sinfo: Success" "$OUT/console.log" || true
grep -q "ISPTEST rc=0" "$OUT/console.log" || { echo "FAIL: the client did not get its frames"; exit 1; }

echo "== frames"
last=$((FRAMES - 1))
first=$((FRAMES - 6))
for slot in 0 1 2 3; do
    if [ -e "$WORK/pmem$slot.nv12" ]; then
        mv "$WORK/pmem$slot.nv12" "$OUT/slot$slot.nv12"
    else
        dd if="$WORK/sd.img" of="$OUT/slot$slot.nv12" bs=8M skip=$slot count=1 status=none
    fi
done
mv "$OUT/slot0.nv12" "$OUT/ch0-$first.nv12"
mv "$OUT/slot1.nv12" "$OUT/ch0-$last.nv12"
mv "$OUT/slot2.nv12" "$OUT/ch1-$first.nv12"
mv "$OUT/slot3.nv12" "$OUT/ch1-$last.nv12"
python3 "$HERE/ingenic-isp-check.py" --advance ${PNG:+--png "$OUT"} \
    ${W}x$H:"$OUT/ch0-$first.nv12" ${W}x$H:"$OUT/ch0-$last.nv12" \
    640x360:"$OUT/ch1-$first.nv12" 640x360:"$OUT/ch1-$last.nv12"
