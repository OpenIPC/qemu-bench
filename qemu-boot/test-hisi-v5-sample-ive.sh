#!/bin/bash
# The vendor SDK's IVE memory test on an emulated Hi3516CV610, end to end
# through ive.ko and the SVP NPU's task scheduler (hisi-ive-npu).
#
# Builds the memory test of mpp/sample/svp/ive (sample_ive_test_memory.c:
# DMA a picture, subtract the copy from the original, histogram the
# difference) statically from the SDK's sources and static MPI libraries,
# with a main that runs that case alone, and runs it from an initramfs made
# of the OpenIPC hi3516cv6xx rootfs, whose own load_hisilicon loads every
# module.  The sample writes the difference to test_mem_out.yuv and prints
# "Test mem error" when the histogram is not all in bin 0, yet exits 0
# either way, so the host checks both: no such line, and the difference
# all zeros.
#
# The SDK is not public, so this runs locally, not in CI:
#
#   test-hisi-v5-sample-ive.sh --sdk ~/projects/cameras/sdk/Hi3516CV610_SDK_V1.0.2.0
#                              [--qemu Q] [--work DIR]
#
# Needs arm-openipc-linux-musleabi-gcc (or CC), unsquashfs, cpio.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SDK=
QEMU=$HERE/../qemu-src/build/qemu-system-arm
WORK=
CC=${CC:-arm-openipc-linux-musleabi-gcc}

while [ $# -gt 0 ]; do
    case $1 in
    --sdk) SDK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --work) WORK=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
SRC=$SDK/smp/a7_linux/source
OUT=$SRC/out
SMP=$SRC/mpp/sample
SVP=$SMP/svp
if [ ! -f "$OUT/lib/libss_mpi_ive.a" ] || [ ! -d "$SVP/ive" ]; then
    echo "--sdk: no Hi3516CV610 SDK (out/lib, mpp/sample/svp) in '$SDK'" >&2
    exit 2
fi
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
REL=hi3516cv6xx

# --- the memory test, from the SDK's sources ---------------------------
OBJ=$WORK/obj
rm -rf "$OBJ"
mkdir -p "$OBJ"
cat > "$OBJ/main.c" <<'EOF'
#include "sample_ive_main.h"

int main(void)
{
    sample_ive_test_memory();
    return 0;
}
EOF
SNS=SC4336P_MIPI_4M_30FPS_10BIT
CFLAGS="-O2 -I$OUT/include -I$SMP/common -I$SVP/common -I$SVP/ive/sample
    -Dhi3516cv610 -DOT_XXXX -DISP_V2
    -DSENSOR0_TYPE=$SNS -DSENSOR1_TYPE=$SNS -D${SNS}_SELECT=y
    -DBOARD_TYPE=DMEB_QFN -fsigned-char -D_GNU_SOURCE
    -DVER_X=1 -DVER_Y=0 -DVER_Z=0 -DVER_P=0 -DVER_B=10
    -mcpu=cortex-a7 -mfloat-abi=softfp -mfpu=neon-vfpv4"
objs=()
for f in "$SMP"/common/{sample_comm_sys,sample_comm_isp,sample_comm_vi,sample_comm_vpss,loadbmp,ot_confaccess,sample_comm_venc,sample_comm_region,sample_comm_osd_drawline,sample_ipc}.c \
         "$SVP"/common/{sample_common_svp,sample_common_ive}.c \
         "$SVP"/ive/sample/sample_ive_test_memory.c "$OBJ/main.c"; do
    o=$OBJ/$(basename "$f" .c).o
    # shellcheck disable=SC2086
    "$CC" $CFLAGS -c -o "$o" "$f"
    objs+=("$o")
done
L=$OUT/lib
"$CC" -static -o "$WORK/sample_ive" "${objs[@]}" \
    -Wl,--start-group "$L"/libss_mpi.a "$L"/libss_mpi_ive.a \
    "$L"/libss_mpi_sysbind.a "$L"/libss_mpi_sysmem.a "$L"/libss_mpi_ae.a \
    "$L"/libss_mpi_isp.a "$L"/libot_mpi_isp.a "$L"/libss_mpi_awb.a \
    "$L"/libdehaze.a "$L"/libextend_stats.a "$L"/libdrc.a "$L"/libldci.a \
    "$L"/libbnr.a "$L"/libcalcflicker.a "$L"/libir_auto.a "$L"/libacs.a \
    "$L"/libsns_sc4336p.a "$L"/libot_osal.a "$L"/libss_mpi_syskol.a \
    "$L"/libsecurec.a -Wl,--end-group -lpthread -lm -ldl -lstdc++

# --- initramfs: the release's rootfs, the sample as /init's job ---------
if [ ! -f "$WORK/uImage.$REL" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$REL" "$WORK"
fi
INITRD=$WORK/initrd
rm -rf "$INITRD"
unsquashfs -q -d "$INITRD" "$WORK/rootfs.squashfs.$REL" >/dev/null
cp "$WORK/sample_ive" "$INITRD/usr/bin/"
mkdir -p "$INITRD/root/data/input/testmem" "$INITRD/root/data/output/testmem"
cp "$SVP/ive/data/input/testmem/test_mem_in.yuv" \
    "$INITRD/root/data/input/testmem/"
cat > "$INITRD/init" <<'EOF'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev
load_hisilicon -i -sensor0 sc4336p
cd /root
echo "=== sample_ive"
sample_ive
echo "=== done rc=$?"
f=data/output/testmem/test_mem_out.yuv
echo "=== out $(wc -c < $f) bytes, $(tr -d '\000' < $f | wc -c) nonzero"
echo b > /proc/sysrq-trigger
EOF
chmod +x "$INITRD/init"
(cd "$INITRD" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/sample.cpio"

timeout 600 "$QEMU" -M hi3516cv610,sensor=sc4336p -m 128M \
    -kernel "$WORK/uImage.$REL" -initrd "$WORK/sample.cpio" \
    -append "console=ttyAMA0,115200 rdinit=/init mem=64M mmz_allocator=hisi mmz=anonymous,0,0x44000000,64M" \
    -icount shift=0 -nographic -serial mon:stdio -nic user,restrict=on \
    -no-reboot > "$WORK/console.log" 2>&1 || true

fail() {
    echo "$1; console in $WORK/console.log" >&2
    exit 1
}
grep -aq "=== done rc=0" "$WORK/console.log" || fail "sample_ive did not finish"
# the sample's own errors, and ive.ko's (a stream that would not stop)
if sed -n '/^=== sample_ive/,$p' "$WORK/console.log" |
        grep -aE "Test mem error|termination abnormally|\[level\]:Error|failure"; then
    fail "sample_ive reported errors"
fi
out=$(grep -a "^=== out " "$WORK/console.log" | tail -1 | tr -d '\r')
echo "sample_ive: ${out#=== }"
case $out in
"=== out 0 bytes"*) fail "no output picture" ;;
*" 0 nonzero") echo "PASS ive: DMA, SUB and HIST through ive.ko" ;;
*) fail "the difference of a picture and its copy is not zero" ;;
esac
