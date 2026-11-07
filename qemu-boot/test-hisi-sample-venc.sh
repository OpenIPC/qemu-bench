#!/bin/bash
# The vendor SDK's own sample_venc on an emulated Hi3516EV200, end to end.
#
# Builds mpp/sample/venc statically, as the SDK's makefile does (himix100,
# the SDK's libmpi.a and sensor libraries), and runs it unmodified from an
# initramfs cut from the OpenIPC release (busybox and the release's
# open_*.ko, the whole load_hisilicon chain) with an SC2315E on the sensor
# bus.  Mode 1 (H265_H264): VI online into VPSS, a 1080p H.265 channel from
# VPSS channel 0 (compressed, see hisi-vpss.c) and a 640x480 H.264 one from
# channel 1.  The rate control and GOP prompts get CBR and NORMALP; the
# sample runs --seconds, then exits on the Enter it waits for.  Both
# streams must decode cleanly, with hisi-vpss's frame counter rising.
#
# The SDK is not public, so this runs where it is (CI runs
# test-hisi-majestic.sh instead):
#
#   test-hisi-sample-venc.sh --sdk ~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2
#                            [--qemu Q] [--work DIR] [--seconds 20]
#
# Needs the himix100 toolchain (arm-himix100-linux-gcc on PATH, or under
# /opt/hisi-linux/x86-arm/arm-himix100-linux), unsquashfs, cpio, ffmpeg.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SDK=
QEMU=$HERE/../qemu-src/build/qemu-system-arm
WORK=
SECS=20
SOC=hi3516ev200

while [ $# -gt 0 ]; do
    case $1 in
    --sdk) SDK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --work) WORK=$2; shift ;;
    --seconds) SECS=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
[ -n "$SDK" ] || { echo "--sdk is required" >&2; exit 2; }
MPP=$SDK/mpp
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
command -v arm-himix100-linux-gcc >/dev/null ||
    PATH=/opt/hisi-linux/x86-arm/arm-himix100-linux/bin:$PATH

# --- sample_venc, as mpp/sample/venc/Makefile builds it ----------------
# (its flags, -fno-aggressive-loop-optimizations included: without it GCC 6
# turns the samples' loops into crashes)
# The SDK's hi_sns_ctrl.h declares only some sensor objects (not the
# SC2315E's, its default), so declare every one the libraries define.
OBJ=$WORK/obj
mkdir -p "$OBJ"
{
    echo '#include "hi_sns_ctrl.h"'
    for a in "$MPP"/lib/libsns_*.a; do
        nm "$a" | awk '$2 == "D" && $3 ~ /^stSns.*Obj$/ {
            print "extern ISP_SNS_OBJ_S " $3 ";" }'
    done | sort -u
} > "$OBJ/sns_externs.h"
CFLAGS="-Wall -O2 -I$MPP/include -I$MPP/sample/common
    -I$MPP/sample/audio/adp -Dhi3516ev200
    -DHI_XXXX -DISP_V2 -DHI_RELEASE -DUSER_BIT_32 -DKERNEL_BIT_32
    -D_GNU_SOURCE -DSENSOR0_TYPE=SMART_SC2315E_MIPI_2M_25FPS_10BIT
    -DHI_ACODEC_TYPE_INNER -mcpu=cortex-a7 -mfloat-abi=softfp
    -mfpu=neon-vfpv4 -ffunction-sections -fdata-sections
    -fno-aggressive-loop-optimizations -DVER_X=1 -DVER_Y=0 -DVER_Z=0
    -DVER_P=0 -DVER_B=10 -include $OBJ/sns_externs.h"
objs=()
for f in "$MPP"/sample/common/*.c "$MPP"/sample/venc/sample_venc.c; do
    o=$OBJ/$(basename "${f%.c}").o
    # shellcheck disable=SC2086
    arm-himix100-linux-gcc $CFLAGS -c -o "$o" "$f"
    objs+=("$o")
done
L=$MPP/lib
arm-himix100-linux-gcc -static -o "$WORK/sample_venc" "${objs[@]}" \
    -Wl,--start-group "$L"/libmpi.a "$L"/lib_hiae.a "$L"/libisp.a \
    "$L"/lib_hidehaze.a "$L"/lib_hidrc.a "$L"/lib_hildci.a "$L"/lib_hiawb.a \
    "$L"/lib_hiir_auto.a "$L"/libsns_*.a "$L"/libsecurec.a \
    "$L"/libVoiceEngine.a "$L"/libupvqe.a "$L"/libdnvqe.a "$L"/libive.a \
    "$L"/libmd.a "$L"/libtde.a "$L"/libhive_*.a "$L"/libaac*.a \
    -Wl,--end-group -lpthread -lm -ldl -lstdc++

# --- initramfs: the release's modules, the sample as PID 1's job ---------
if [ ! -f "$WORK/uImage.$SOC" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$SOC" "$WORK"
fi
ROOTFS=$WORK/rootfs
[ -d "$ROOTFS" ] ||
    unsquashfs -q -d "$ROOTFS" "$WORK/rootfs.squashfs.$SOC" >/dev/null
KO=$(ls -d "$ROOTFS"/lib/modules/*/extra)
INITRD=$WORK/initrd
rm -rf "$INITRD"
mkdir -p "$INITRD"/{bin,lib,proc,sys,dev,tmp,ko,usr/bin}
cp -a "$ROOTFS"/bin/busybox "$INITRD/bin/"
for l in sh mount insmod sleep echo; do ln -sf busybox "$INITRD/bin/$l"; done
cp -a "$ROOTFS"/lib/ld-* "$ROOTFS"/lib/libc.so* "$INITRD/lib/"
MODS="sys_config osal base sys rgn vgs vi openipc_frame_ts isp vpss chnl
      vedu rc venc h264e h265e jpege sensor_i2c mipi_rx"
for m in $MODS; do cp "$KO/open_$m.ko" "$INITRD/ko/"; done
cp "$WORK/sample_venc" "$INITRD/usr/bin/"
cat > "$INITRD/init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev
mount -t tmpfs tmp /tmp
cd /ko
insmod open_sys_config.ko chip=$SOC sensors=sc2315e board=demo
insmod open_osal.ko anony=1 mmz_allocator=hisi mmz=anonymous,0,0x42000000,32M
for m in base sys rgn; do insmod open_\$m.ko; done
insmod open_vgs.ko max_vgs_job=20 max_vgs_node=20 max_vgs_task=20
for m in vi openipc_frame_ts isp vpss chnl vedu rc; do insmod open_\$m.ko; done
insmod open_venc.ko VencMaxChnNum=3
for m in h264e h265e jpege sensor_i2c mipi_rx; do insmod open_\$m.ko; done
cd /tmp
echo "=== sample_venc"
(printf "c\\n0\\n"; sleep $SECS; echo; echo) | sample_venc 1
echo "=== done"
echo b > /proc/sysrq-trigger
EOF
chmod +x "$INITRD/init"
(cd "$INITRD" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/sample.cpio"

DUMP=$WORK/dump
rm -rf "$DUMP"
mkdir -p "$DUMP"
timeout $((SECS * 20 + 300)) "$QEMU" -M "$SOC,sensor=sc2315e" \
    -kernel "$WORK/uImage.$SOC" -initrd "$WORK/sample.cpio" \
    -append "console=ttyAMA0,115200 rdinit=/init" -icount shift=0 \
    -global hisi-vedu.dump-dir="$DUMP" -nographic -serial mon:stdio \
    -nic user,restrict=on -no-reboot > "$WORK/console.log" 2>&1 || true

if ! grep -q "program exit normally" "$WORK/console.log"; then
    echo "sample_venc did not finish; console in $WORK/console.log" >&2
    exit 1
fi
python3 "$HERE/check-hisi-venc.py" --min-frames 25 --rising "$DUMP" h265 h264
