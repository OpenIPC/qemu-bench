#!/bin/bash
# The HDAL media engines of emulated NT9856x under the OpenIPC release's
# own vendor modules.  The release boots from an 8 MiB NOR laid out the
# OpenIPC way; a PID-1 script mounts its SquashFS root, runs the release's
# load_novatek in a chroot -- kwrap, nvtmem, kdrv_builtin, kdrv_comm, the
# video capture / process / codec / gfx / audio drivers and flows, the ISP
# and the sensor driver (sensor=sc501ai) -- and checks that every media
# driver bound its device-tree node (docs/novatek.md, HDAL media).
#
# With --samples DIR (qemu-boot/build-novatek-hdal-samples.sh, from the
# vendor SDK: not in CI) it then runs the SDK's HDAL samples with their
# files on an emulated SD card, and checks what they leave there:
#
#   - hd_video_record: sensor -> SIE -> IFE / DCE / IPE / IME -> H.265,
#     the bitstream file written, no HDAL error;
#   - hd_video_snapshot: two JPEG snapshots, each a whole JPEG (SOI ...
#     EOI) that a decoder (Python PIL) reads as a 2880x1620 picture;
#   - hd_video_capture_only, hd_video_encode_only (H.264 / H.265 from a
#     file of frames): no HDAL error;
#   - hd_gfx_only: copy, scale (ISE), rotation, colour transform, lines,
#     rectangles, job lists, YUV420 3P -> 2P and 3P scaling each exit
#     without an error and leave their result file;
#   - hd_audio_capture_only: PCM captured through EAC and the DAI DMA.
#
# The SIE captures a moving test pattern; the IPP turns its RAW into YUV and
# the JPEG codec encodes it, so the snapshots show colour bars, a grey ramp
# and a white box that moves between them (checked with Python PIL).  H.26x
# output, graphics results and audio are still placeholders
# (docs/novatek.md).  Sample keys go in on stdin.
#
#   bash qemu-boot/test-novatek-hdal.sh [--soc nt98566] [--machine nt98566] \
#       [--samples DIR] [--dir qemu-boot] [--qemu ./qemu-src/build/qemu-system-arm]
set -euo pipefail

SOC=nt98566
MACHINE=
SAMPLES=
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
SENSOR=sc501ai
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --samples) SAMPLES=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
MACHINE=${MACHINE:-$SOC}
KERNEL=$DIR/uImage.$SOC
ROOTFS=$DIR/rootfs.squashfs.$SOC
if [ ! -f "$KERNEL" ] || [ ! -f "$ROOTFS" ]; then
    echo "need $KERNEL and $ROOTFS: unpack openipc.$SOC-nor-lite.tgz in $DIR" >&2
    exit 2
fi
if [ -n "$SAMPLES" ] && [ ! -d "$SAMPLES/smp" ]; then
    echo "--samples: no smp/ in $SAMPLES" \
         "(qemu-boot/build-novatek-hdal-samples.sh)" >&2
    exit 2
fi

WORK=$(mktemp -d)
trap '[ -n "${KEEP:-}" ] || rm -rf "$WORK"' EXIT

MTDPARTS="spi_nor.0:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"
python3 - "$KERNEL" "$ROOTFS" "$WORK/nor.bin" <<'PY'
import sys
kernel, rootfs, nor = sys.argv[1:]
img = bytearray(b'\xff' * (8 << 20))
k = open(kernel, 'rb').read()
r = open(rootfs, 'rb').read()
img[0x50000:0x50000 + len(k)] = k
img[0x250000:0x250000 + len(r)] = r
open(nor, 'wb').write(img)
PY
truncate -s 64M "$WORK/sd.img"

unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" bin/busybox lib/libc.so \
    lib/ld-musl-armhf.so.1 >/dev/null
mkdir -p "$WORK/initramfs"/{bin,lib,proc,sys,dev,mnt}
cp "$WORK/rootfs/bin/busybox" "$WORK/initramfs/bin/"
cp -a "$WORK/rootfs/lib/libc.so" "$WORK/rootfs/lib/ld-musl-armhf.so.1" \
    "$WORK/initramfs/lib/"
ln -s busybox "$WORK/initramfs/bin/sh"
if [ -n "$SAMPLES" ]; then
    cp -a "$SAMPLES/smp" "$SAMPLES/uc" "$WORK/initramfs/"
fi

cat > "$WORK/initramfs/init" <<INIT
#!/bin/sh
bb=/bin/busybox
\$bb mount -t proc proc /proc
\$bb mount -t sysfs sys /sys
\$bb mount -t devtmpfs dev /dev
\$bb mount -t squashfs -o ro /dev/mtdblock3 /mnt
for fs in proc sys dev; do \$bb mount --bind /\$fs /mnt/\$fs; done
\$bb mount -t tmpfs tmp /mnt/tmp
\$bb chroot /mnt /bin/sh -c "SENSOR=$SENSOR /usr/bin/load_novatek" >/dev/null 2>&1
echo "NVT-HDAL-MODULES: \$(\$bb lsmod | \$bb wc -l)"
for d in /sys/bus/platform/devices/*; do
    [ -e \$d/driver ] && echo "NVT-HDAL-BOUND: \${d##*/} \$(\$bb basename \$(\$bb readlink \$d/driver))"
done
[ -d /smp ] && \$bb cp -a /smp /uc /samples.sh /mnt/tmp/ && \$bb chroot /mnt /bin/sh /tmp/samples.sh
echo "NVT-HDAL-OK"
\$bb reboot -f
INIT
chmod +x "$WORK/initramfs/init"

# Run in the chroot: the SD card at /mnt/sd, where the samples keep files.
cat > "$WORK/initramfs/samples.sh" <<'SMP'
M=/lib/modules/4.19.91/kernel/drivers/mmc
for ko in core/mmc_core core/mmc_block host/mmc_na51055; do insmod $M/$ko.ko; done
for i in 1 2 3 4 5 6 7 8 9 10; do [ -b /dev/mmcblk0 ] && break; sleep 1; done
mount -t tmpfs -o size=1m sd /mnt && mkdir /mnt/sd
mkfs.vfat /dev/mmcblk0 >/dev/null && mount -t vfat /dev/mmcblk0 /mnt/sd
cd /tmp/smp
run() {         # run <sample> [keys]: the keys 4 s apart, then q
    n=$1
    shift
    ( sleep 6; for k in "$@"; do echo "$k"; sleep 4; done; echo q ) |
        timeout 60 ./$n > /tmp/$n.out 2>&1
    RC=$?
    ERRS=$(grep -c -i 'err\|fail' /tmp/$n.out)
    echo "NVT-HDAL-RUN: $n rc=$RC errors=$ERRS"
}
size() {
    [ -f "$1" ] && wc -c < "$1" || echo 0
}
run hd_video_record
echo "NVT-HDAL-RECORD-BYTES: $(size /mnt/sd/dump_bs_main.dat)"
run hd_video_snapshot s s
for j in /mnt/sd/dump_bs_snap0.jpg /mnt/sd/dump_bs_snap1.jpg; do
    echo "NVT-HDAL-JPEG: ${j##*/} $(head -c 2 $j | hexdump -e '2/1 "%02x"') $(tail -c 2 $j | hexdump -e '2/1 "%02x"') $(size $j)"
done
for n in 0 1; do
    echo "NVT-HDAL-JPEG-B64-BEGIN $n"
    base64 /mnt/sd/dump_bs_snap$n.jpg
    echo "NVT-HDAL-JPEG-B64-END $n"
done
rm -f /mnt/sd/*
run hd_video_capture_only
dd if=/dev/zero of=/mnt/sd/video_frm_352_288_30_yuv420.dat bs=1k count=4455 2>/dev/null
run hd_video_encode_only
echo "NVT-HDAL-ENCODE-BYTES: $(size /mnt/sd/dump_bs_main.dat)"
rm -f /mnt/sd/*
dd if=/dev/zero of=/mnt/sd/video_frm_1000_200_1_argb4444.dat bs=1k count=391 2>/dev/null
dd if=/dev/zero of=/mnt/sd/video_frm_1920_1080_1_yuv420.dat bs=3110400 count=1 2>/dev/null
dd if=/dev/zero of=/mnt/sd/video_frm_1920_1080_1_yuv4203P.dat bs=1M count=4 2>/dev/null
for k in c s o t l r j p d; do
    run hd_gfx_only $k
    f=$(sed -n 's/^result is //p' /tmp/hd_gfx_only.out | head -1)
    echo "NVT-HDAL-GFX: $k rc=$RC errors=$ERRS bytes=$(size "$f")"
    rm -f /mnt/sd/hd_gfx* /mnt/sd/affine*
done
rm -f /mnt/sd/*
run hd_audio_capture_only
echo "NVT-HDAL-AUDIO-BYTES: $(size /mnt/sd/audio_bs_16_2_48000_pcm.dat)"
SMP

(cd "$WORK/initramfs" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

echo "Booting $KERNEL on -M $MACHINE,sensor=$SENSOR${SAMPLES:+, with the HDAL samples}"
set +e
timeout 900 "$QEMU" -M "$MACHINE,sensor=$SENSOR,flash-file=$WORK/nor.bin" \
    -kernel "$KERNEL" -initrd "$WORK/initramfs.cpio" \
    -append "console=ttyS0,115200 rdinit=/init mtdparts=$MTDPARTS" \
    -drive "if=sd,index=0,format=raw,file=$WORK/sd.img" \
    -nographic -monitor none -serial stdio -nic none -no-reboot \
    > "$WORK/console.log" 2>&1
rc=$?
set -e
grep -a -v '^[A-Za-z0-9+/=]*$' "$WORK/console.log" || true

fail=0
pass() { echo "PASS: $1"; }
bad() { echo "FAIL: $1"; fail=1; }
expect() {
    if grep -a -q "$1" "$WORK/console.log"; then pass "$2"; else bad "$2"; fi
}
for b in "f0c00000.sie drv_sie" "f0cc0000.tge kdrv_tge" \
         "f0280000.csi nvt_csi" "f0270000.lvds nvt_lvds" \
         "f0240000.sif nvt_sif" "f0c70000.ife kdrv_ife" \
         "f0c20000.dce kdrv_dce" "f0c30000.ipe kdrv_ipe" \
         "f0c40000.ime ime_builtin" "f0c90000.ise kdrv_ise" \
         "f0c80000.graphic nvt_graphic" "f0a10000.h26x nvt_h26x" \
         "f0a00000.jpg nvt_jpg" "f0640000.eac nvt_eac" \
         "f0630000.dai nvt_dai" "f0000000.nvt_arb nvt_arb" \
         "nvtmpp nvtmpp" "isf_flow isf_flow" "isf_vdocap isf_vdocap" \
         "isf_vdoprc isf_vdoprc" "isf_vdoenc nvt_vdoenc" \
         "isf_audcap isf_audcap" "isf_audout isf_audout" \
         "isf_audenc isf_audenc" "isf_auddec isf_auddec" \
         "audio nvt_audio"; do
    expect "NVT-HDAL-BOUND: $b[[:space:]]*\$" "${b% *} bound by ${b#* }"
done
expect 'NVT-HDAL-OK' 'PID 1 ran to the end'

if [ -n "$SAMPLES" ]; then
    for s in hd_video_record hd_video_snapshot hd_video_capture_only \
             hd_video_encode_only hd_audio_capture_only; do
        expect "NVT-HDAL-RUN: $s rc=0 errors=0" "$s runs without an HDAL error"
    done
    if grep -a -q 'NVT-HDAL-RECORD-BYTES: [1-9]' "$WORK/console.log"; then
        pass "hd_video_record writes its bitstream"
    else
        bad "hd_video_record writes its bitstream"
    fi
    if grep -a -q 'NVT-HDAL-ENCODE-BYTES: [1-9]' "$WORK/console.log"; then
        pass "hd_video_encode_only writes its bitstream"
    else
        bad "hd_video_encode_only writes its bitstream"
    fi
    for n in 0 1; do
        expect "NVT-HDAL-JPEG: dump_bs_snap$n.jpg ffd8 ffd9 [1-9]" \
            "snapshot $n: SOI .. EOI"
    done
    for n in 0 1; do
        sed -n "/NVT-HDAL-JPEG-B64-BEGIN $n/,/NVT-HDAL-JPEG-B64-END $n/p" \
            "$WORK/console.log" | grep -a -v 'NVT-HDAL' | tr -d '\r' |
            base64 -d > "$WORK/snap$n.jpg" 2>/dev/null || true
        dim=$(python3 -c "
from PIL import Image
im = Image.open('$WORK/snap$n.jpg'); im.load(); print('%dx%d' % im.size)
" 2>/dev/null || true)
        if [ "$dim" = 2880x1620 ]; then
            pass "snapshot $n decodes: a 2880x1620 JPEG"
        elif [ -z "$dim" ] && ! python3 -c 'import PIL' 2>/dev/null; then
            echo "SKIP: snapshot $n decode (no Python PIL)"
        else
            bad "snapshot $n decodes: a 2880x1620 JPEG (got '$dim')"
        fi
    done
    # The sensor's test pattern through SIE, IPP and the JPEG encoder:
    # colour bars over a grey ramp, a white box that moves frame to frame.
    look=$(python3 - "$WORK/snap0.jpg" "$WORK/snap1.jpg" <<'PY' 2>/dev/null || true
import sys
from PIL import Image
ims = [Image.open(f).convert('RGB') for f in sys.argv[1:]]
w, h = ims[0].size
want = [(1, 1, 1), (1, 1, 0), (0, 1, 1), (0, 1, 0), (1, 0, 1), (1, 0, 0),
        (0, 0, 1), (0, 0, 0)]
bars = all(tuple(int(c > 96) for c in ims[0].getpixel((w * (2 * i + 1) // 16,
                                                         h // 6))) == want[i]
           for i in range(8))
ramp = [ims[0].getpixel((x, h * 5 // 6))[1] for x in (8, w // 2, w - 8)]
def box(im):
    xs = [x for x in range(0, w, 4) if min(im.getpixel((x, h // 3))) > 220]
    return xs[0] if xs else -1
print('bars=%d ramp=%d moved=%d' % (bars, ramp[0] < ramp[1] < ramp[2],
                                     box(ims[0]) >= 0 and box(ims[1]) >= 0 and
                                     box(ims[0]) != box(ims[1])))
PY
)
    if [ -z "$look" ] && ! python3 -c 'import PIL' 2>/dev/null; then
        echo "SKIP: snapshot content checks (no Python PIL)"
    elif [ -z "$look" ]; then
        bad "snapshot content: the check did not run"
    else
        case "$look" in *bars=1*) pass "snapshot: the eight colour bars";;
            *) bad "snapshot: the eight colour bars ($look)";; esac
        case "$look" in *ramp=1*) pass "snapshot: the grey ramp, dark to light";;
            *) bad "snapshot: the grey ramp ($look)";; esac
        case "$look" in *moved=1*) pass "the white box moves between the snapshots";;
            *) bad "the white box moves between the snapshots ($look)";; esac
    fi
    for k in c s o t l r j p d; do
        expect "NVT-HDAL-GFX: $k rc=0 errors=0 bytes=[1-9]" \
            "hd_gfx_only '$k': no error, its result file written"
    done
    if grep -a -q 'NVT-HDAL-AUDIO-BYTES: [1-9]' "$WORK/console.log"; then
        pass "hd_audio_capture_only captures PCM"
    else
        bad "hd_audio_capture_only captures PCM"
    fi
fi
if [ $rc -ne 0 ]; then
    echo "FAIL: QEMU exited $rc (124: a wait never ended)"
    fail=1
fi
exit $fail
