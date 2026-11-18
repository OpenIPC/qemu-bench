#!/bin/bash
# The vendor SDK's own sample_venc on an emulated Hi3516CV610, end to end
# -- the V5 counterpart of test-hisi-sample-venc.sh.
#
# Builds mpp/sample/venc statically from the SDK's sources and static MPI
# libraries (with OpenIPC's musl toolchain), and runs it from an initramfs
# made of the OpenIPC hi3516cv6xx rootfs, whose own load_hisilicon loads
# every module, with an SC4336P on the sensor bus.  sample_venc's case 0
# runs VI and VPSS online, its group's frame interrupt EARLY_END (the
# driver prepares each frame on VPSS's early interrupt, 0x100).  It is
# built without its VPSS -> VENC wrap (a low-delay ring the model does
# not run), with a common pool large enough for channel 0's 2560x1440 in
# the wrap buffer's place (the sample sizes none for it otherwise).
# --offline builds it with VI_OFFLINE_VPSS_ONLINE (the SDK's aiisp / kol
# mode): VI runs its pipe per frame from a node list, VPSS its node lists,
# the ISP's statistics go through the offline blocks, and the pool holds
# the raw frames too.  The sample must exit cleanly, its H.265 and H.264
# streams decode with hisi-vpss's frame counter rising, and the ISP's
# AE / AWB settle.
#
# The SDK is not public, so this runs locally, not in CI:
#
#   test-hisi-v5-sample-venc.sh --sdk ~/projects/cameras/sdk/Hi3516CV610_SDK_V1.0.2.0
#                               [--offline] [--qemu Q] [--work DIR] [--seconds 30]
#
# Needs arm-openipc-linux-musleabi-gcc (or CC), unsquashfs, cpio, ffmpeg.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SDK=
QEMU=$HERE/../qemu-src/build/qemu-system-arm
WORK=
SECS=30
OFFLINE=0
CC=${CC:-arm-openipc-linux-musleabi-gcc}

while [ $# -gt 0 ]; do
    case $1 in
    --sdk) SDK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --work) WORK=$2; shift ;;
    --seconds) SECS=$2; shift ;;
    --offline) OFFLINE=1 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
SRC=$SDK/smp/a7_linux/source
OUT=$SRC/out
SMP=$SRC/mpp/sample
if [ ! -f "$OUT/lib/libss_mpi.a" ] || [ ! -d "$SMP/venc" ]; then
    echo "--sdk: no Hi3516CV610 SDK (out/lib, mpp/sample) in '$SDK'" >&2
    exit 2
fi
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
REL=hi3516cv6xx

# --- sample_venc, from the SDK's sources -------------------------------
OBJ=$WORK/obj
rm -rf "$OBJ"
mkdir -p "$OBJ"
cp "$SMP/venc/sample_venc.c" "$OBJ/"
# no VPSS -> VENC wrap (the low-delay ring the model does not run;
# majestic does not use it)
sed -i -e 's/sys_cfg->vpss_wrap_en = TD_TRUE;/sys_cfg->vpss_wrap_en = TD_FALSE;/' \
    -e 's/vpss_cfg->wrap_attr\[0\].enable = TD_TRUE;/vpss_cfg->wrap_attr[0].enable = TD_FALSE;/' \
    "$OBJ/sample_venc.c"
# a common pool for channel 0 (and offline, the raw frames) where the
# disabled wrap buffer's was: 2560x1440 at 12 bits a pixel covers both
perl -0pi -e 's/vb_cfg->common_pool\[i\]\.blk_cnt = 1;\n\s*vb_cfg->common_pool\[i\]\.blk_size = wrap_size;/{ ot_pic_buf_attr raw = { .width = 2560, .height = 1440, .align = OT_DEFAULT_ALIGN, .bit_width = OT_DATA_BIT_WIDTH_12, .pixel_format = OT_PIXEL_FORMAT_RGB_BAYER_12BPP, .compress_mode = OT_COMPRESS_MODE_NONE, .video_format = OT_VIDEO_FORMAT_LINEAR };\n        (void)wrap_size; ot_common_get_pic_buf_cfg(&raw, &calc_cfg);\n        vb_cfg->common_pool[i].blk_size = calc_cfg.vb_size;\n        vb_cfg->common_pool[i].blk_cnt = 4; }/' \
    "$OBJ/sample_venc.c"
grep -q OT_PIXEL_FORMAT_RGB_BAYER_12BPP "$OBJ/sample_venc.c"
if [ "$OFFLINE" = 1 ]; then
    sed -i 's/sys_cfg->mode_type = OT_VI_ONLINE_VPSS_ONLINE;/sys_cfg->mode_type = OT_VI_OFFLINE_VPSS_ONLINE;/' \
        "$OBJ/sample_venc.c"
    grep -q OT_VI_OFFLINE_VPSS_ONLINE "$OBJ/sample_venc.c"
fi
MEM="mem=64M mmz_allocator=hisi mmz=anonymous,0,0x44000000,64M"
SNS=SC4336P_MIPI_4M_30FPS_10BIT
CFLAGS="-O2 -I$OUT/include -I$SMP/common -Dhi3516cv610 -DOT_XXXX -DISP_V2
    -DSENSOR0_TYPE=$SNS -DSENSOR1_TYPE=$SNS -D${SNS}_SELECT=y
    -DBOARD_TYPE=DMEB_QFN -fsigned-char -D_GNU_SOURCE
    -DVER_X=1 -DVER_Y=0 -DVER_Z=0 -DVER_P=0 -DVER_B=10
    -mcpu=cortex-a7 -mfloat-abi=softfp -mfpu=neon-vfpv4"
objs=()
for f in sample_comm_sys sample_comm_isp sample_comm_vi sample_comm_vpss \
         loadbmp ot_confaccess sample_comm_venc sample_comm_region \
         sample_comm_osd_drawline sample_ipc; do
    # shellcheck disable=SC2086
    "$CC" $CFLAGS -c -o "$OBJ/$f.o" "$SMP/common/$f.c"
    objs+=("$OBJ/$f.o")
done
# shellcheck disable=SC2086
"$CC" $CFLAGS -c -o "$OBJ/sample_venc.o" "$OBJ/sample_venc.c"
L=$OUT/lib
"$CC" -static -o "$WORK/sample_venc" "${objs[@]}" "$OBJ/sample_venc.o" \
    -Wl,--start-group "$L"/libss_mpi.a "$L"/libss_mpi_sysbind.a \
    "$L"/libss_mpi_sysmem.a "$L"/libss_mpi_ae.a "$L"/libss_mpi_isp.a \
    "$L"/libot_mpi_isp.a "$L"/libss_mpi_awb.a "$L"/libdehaze.a \
    "$L"/libextend_stats.a "$L"/libdrc.a "$L"/libldci.a "$L"/libbnr.a \
    "$L"/libcalcflicker.a "$L"/libir_auto.a "$L"/libacs.a \
    "$L"/libsns_sc4336p.a "$L"/libot_osal.a "$L"/libss_mpi_syskol.a \
    "$L"/libsecurec.a -Wl,--end-group -lpthread -lm -ldl -lstdc++

# --- initramfs: the release's rootfs, the sample as /init's job ---------
if [ ! -f "$WORK/uImage.$REL" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$REL" "$WORK"
fi
INITRD=$WORK/initrd
rm -rf "$INITRD"
unsquashfs -q -d "$INITRD" "$WORK/rootfs.squashfs.$REL" >/dev/null
cp "$WORK/sample_venc" "$INITRD/usr/bin/"
cat > "$INITRD/init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev
mount -t tmpfs tmp /tmp
load_hisilicon -i -sensor0 sc4336p
cd /tmp
echo "=== sample_venc"
(printf "0\\nc\\n"; sleep $SECS; echo; echo) | sample_venc 0
echo "=== done rc=\$?"
echo b > /proc/sysrq-trigger
EOF
chmod +x "$INITRD/init"
(cd "$INITRD" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/sample.cpio"

DUMP=$WORK/dump
rm -rf "$DUMP"
mkdir -p "$DUMP"
# the initramfs is the whole rootfs: the OS gets more than the camera's
# 32 MiB, 64, and MMZ the other 64 (the 2560x1440 pool)
timeout $((SECS * 20 + 300)) "$QEMU" -M hi3516cv610,sensor=sc4336p -m 128M \
    -kernel "$WORK/uImage.$REL" -initrd "$WORK/sample.cpio" \
    -append "console=ttyAMA0,115200 rdinit=/init $MEM" \
    -icount shift=0 -global hisi-vedu.dump-dir="$DUMP" \
    -global hisi-vi.stats-trace=on \
    -global hisi-scene.light-r=625 -global hisi-scene.light-b=565 \
    -nographic -serial mon:stdio -nic user,restrict=on -no-reboot \
    > "$WORK/console.log" 2>&1 || true

if ! grep -aq "=== done rc=0" "$WORK/console.log"; then
    echo "sample_venc did not finish cleanly; console in $WORK/console.log" >&2
    exit 1
fi
# AE and AWB on the ISP statistics hisi-vi wrote (offline: the offline
# blocks): the scene settled between 10 % and 70 % of full scale, and the
# light's R and B, times the driver's WB gains, within 15 % of G
last=$(grep -a "^isp stats" "$WORK/console.log" | tail -1)
echo "ISP: $last"
if [ "$OFFLINE" = 1 ] && ! echo "$last" | grep -q ", offline,"; then
    echo "no offline statistics; console in $WORK/console.log" >&2
    exit 1
fi
if ! echo "$last" | awk '
    function hex(s,   n, i) {
        n = 0; s = tolower(s); sub(/^0x/, "", s); sub(/,$/, "", s)
        for (i = 1; i <= length(s); i++)
            n = n * 16 + index("0123456789abcdef", substr(s, i, 1)) - 1
        return n
    }
    { for (i = 1; i <= NF; i++) {
        if ($i == "R" && $(i + 1) ~ /^[0-9]+$/) r = $(i + 1)
        if ($i == "B" && $(i + 1) ~ /^[0-9]+,?$/) b = $(i + 1) + 0
        if ($i == "mean") g = $(i + 1) + 0
        if ($i == "gains") { gr = hex($(i + 2)); gb = hex($(i + 4)) }
    }
    if (g < 6553 || g > 45875) { print "AE: G mean " g " not settled"; exit 1 }
    if (!gr || !gb) { print "AWB: no gains"; exit 1 }
    er = r * gr / 256 / g - 1; eb = b * gb / 256 / g - 1
    printf "AE: G mean %d; AWB: %+.0f%% / %+.0f%% of G\n", g, er * 100, eb * 100
    exit (er < -0.15 || er > 0.15 || eb < -0.15 || eb > 0.15) }'; then
    echo "AE / AWB did not settle; console in $WORK/console.log" >&2
    exit 1
fi
# both channels: the H.265 one at 2560x1440, the H.264 one VPSS scales
for codec in h265 h264; do
    python3 "$HERE/check-hisi-venc.py" --min-frames 25 --rising "$DUMP" $codec
done
