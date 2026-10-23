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
#     the bitstream file written, no HDAL error; with x265 built into QEMU
#     and ffmpeg on the host, the tiled stream decodes without an error
#     to the sensor's moving picture;
#   - hd_video_snapshot: two JPEG snapshots, each a whole JPEG (SOI ...
#     EOI) that a decoder (Python PIL) reads as a 2880x1620 picture;
#   - hd_video_capture_only, hd_video_encode_only (H.265 and H.264 from a
#     patterned file of frames): no HDAL error; with x264 / x265 built
#     into QEMU and ffmpeg on the host, both bitstreams decode without an
#     error to pictures of the input (PSNR 30 dB or better);
#   - hd_gfx_only: copy, scale (ISE), rotation, colour transform, lines,
#     rectangles, job lists, YUV420 3P -> 2P and 3P scaling, from inputs
#     patterned here, leave the pixels the engines should compute; the DMA
#     copy and 16-bit arithmetic pass the sample's own comparisons;
#   - hd_audio_capture_only: the test tone captured through the EAC and
#     the DAI DMA, 1 kHz left and 1.5 kHz right (checked on the host);
#   - hd_audio_output_only and hd_audio_livesound: the capture played
#     back, to the machine's audiodev, a wav file here, where both tones
#     must be;
#   - hd_audio_record: AAC of the tone, which ffmpeg decodes to it.
#
# The SIE captures a moving test pattern; the IPP turns its RAW into YUV and
# the JPEG codec encodes it, so the snapshots show colour bars, a grey ramp
# and a white box that moves between them (checked with Python PIL).  Audio
# is a test tone or the host's (docs/novatek.md).  Sample keys go in on
# stdin.
#
# With --kmods DIR (the HDAL modules of OpenIPC's novatek-nt9856x
# toolchain, toolchain/sdk/kmod/hdal, which the release leaves out;
# --display-kos is the old name) it also loads the AI engines' modules
# (kdrv_ai and the kflow units) and checks they bind, and the display
# path the SDK's way -- nvt_ide, kdrv_videoout,
# kflow_videoout, nvt_dispdev_panel and the EVB's HX8238D panel -- and
# checks that it binds the IDE.  With --samples as well it runs
# hd_video_output_only (colour bars from a file) and hd_video_liveview
# (the sensor), and checks QEMU's screendump of the display mid-run, and
# ai_op's five operators: NUE's fully connected layer against the result
# host arithmetic gives, NUE2's preprocessing (YUV to RGB, scaling, mean
# subtraction) for its sizes and for the subtractions' consistency; and,
# when the samples carry a model built for this chip
# (build-novatek-hdal-samples.sh --ai-model), ai_net, the SDK's ImageNet
# classifier (NUE2, 93 CNN jobs, NUE's FC), which must call ai_pattern's
# picture a sea snake.  With the AI turnkey's detectors in the samples
# (--sdk7 / --turnkey) it runs them on their pictures -- people, people
# and vehicles, faces -- from a SquashFS on a second SD card, and checks
# what they find.
# --panel picks the panel: pw35p00 (the EVB's, serial RGB, 320x240) or a
# MIPI DSI one -- nt35510 (480x800), tb093 (320x1280) or ili9488
# (320x480) -- for which nvt_dsi loads first.
#
#   bash qemu-boot/test-novatek-hdal.sh [--soc nt98566] [--machine nt98566] \
#       [--samples DIR] [--kmods DIR] [--panel pw35p00] [--dir qemu-boot] \
#       [--qemu ./qemu-src/build/qemu-system-arm]
set -euo pipefail

SOC=nt98566
MACHINE=
SAMPLES=
DISPLAY_KOS=
PANEL=pw35p00
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
SENSOR=sc501ai
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --samples) SAMPLES=$2; shift 2 ;;
        --kmods|--display-kos) DISPLAY_KOS=$2; shift 2 ;;
        --panel) PANEL=$2; shift 2 ;;
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
# the panel: its module (display_panel/<dir>/<module>.ko), its size on
# QEMU's console, and whether it is on MIPI DSI
case $PANEL in
    pw35p00) PANEL_KO=disp_if8b_lcd1_pw35p00_hx8238d/disp_if8b_lcd1_pw35p00
             PANEL_SIZE=320x240 PANEL_DSI= PANEL_HELLO=PW35P00 ;;
    nt35510) PANEL_KO=disp_ifdsi_lcd1_nt35510/disp_ifdsi_lcd1_nt35510
             PANEL_SIZE=480x800 PANEL_DSI=nvt_dsi PANEL_HELLO=NT35510 ;;
    tb093) PANEL_KO=disp_ifdsi_lcd1_tb093/disp_ifdsi_lcd1_tb093
           PANEL_SIZE=320x1280 PANEL_DSI=nvt_dsi PANEL_HELLO=tb093 ;;
    ili9488) PANEL_KO=disp_ifdsi_lcd1_ut35067a0_ili9488/disp_ifdsi_lcd1_ut35067a0_ili9488
             PANEL_SIZE=320x480 PANEL_DSI=nvt_dsi PANEL_HELLO=UT35067A0 ;;
    *) echo "--panel: pw35p00, nt35510, tb093 or ili9488" >&2; exit 2 ;;
esac
PANEL_MOD=${PANEL_KO##*/}
mkdir -p "$WORK/initramfs/dispin"
if [ -n "$DISPLAY_KOS" ]; then
    mkdir -p "$WORK/initramfs/dko"
    for k in kdrv_videoout/ide/nvt_ide ${PANEL_DSI:+kdrv_videoout/dsi/nvt_dsi} \
             kdrv_videoout/display_obj/kdrv_videoout \
             kflow_videoout/unit/kflow_videoout display_panel/nvt_dispdev_panel \
             display_panel/$PANEL_KO; do
        cp "$DISPLAY_KOS/$k.ko" "$WORK/initramfs/dko/"
    done
    for k in kdrv_ai kflow_cnn kflow_nue kflow_nue2 kflow_cpu kflow_dsp \
             kflow_ai_net; do
        cp "$DISPLAY_KOS/$k/$k.ko" "$WORK/initramfs/dko/"
    done
    # hd_video_output_only's 3 frames of 320x240 YUV 4:2:2, Y then CbCr
    # interleaved: the eight 75 % colour bars (BT.601)
    python3 - "$WORK/initramfs/dispin/video_frm_320_240_3_yuv422.dat" <<'PY'
import sys
w, h = 320, 240
bars = [(180, 128, 128), (162, 44, 142), (131, 156, 44), (112, 72, 58),
        (84, 184, 198), (65, 100, 212), (35, 212, 114), (16, 128, 128)]
y = bytes(bars[x * 8 // w][0] for r in range(h) for x in range(w))
uv = bytes(v for r in range(h) for x in range(0, w, 2)
           for v in bars[x * 8 // w][1:])
open(sys.argv[1], 'wb').write((y + uv) * 3)
PY
fi
if [ -n "$SAMPLES" ]; then
    cp -a "$SAMPLES/smp" "$SAMPLES/uc" "$WORK/initramfs/"
    # the turnkey detectors' models and pictures (--sdk7 / --turnkey):
    # too large for the initramfs, so a SquashFS on a second SD card,
    # its size a power of two as QEMU's SD cards want
    if [ -d "$SAMPLES/sd" ]; then
        mksquashfs "$SAMPLES/sd" "$WORK/det.img" -noappend -quiet >/dev/null
        n=$(stat -c %s "$WORK/det.img"); p=1048576
        while [ $p -lt "$n" ]; do p=$((p * 2)); done
        truncate -s $p "$WORK/det.img"
        DETDRIVE="-drive if=sd,index=1,format=raw,file=$WORK/det.img"
        touch "$WORK/initramfs/smp/.detcard"     # samples.sh waits for it
    fi
    # hd_gfx_only's inputs, patterned so its results can be checked: Y is
    # (x + y) / 16, U / V ramps over the pair index, the ARGB4444 logo is
    # opaque green
    # hd_video_encode_only's 30 frames of 352x288 NV12: a ramp scrolling
    # 4 pixels a frame, chroma ramps (the host compares the decoded
    # bitstreams with them)
    mkdir -p "$WORK/initramfs/encin"
    python3 - "$WORK/initramfs/encin/video_frm_352_288_30_yuv420.dat" <<'PY'
import sys
w, h = 352, 288
out = bytearray()
for f in range(30):
    out += bytes(16 + ((x + 4 * f) % w) * 200 // w
                 for y in range(h) for x in range(w))
    out += bytes(v for y in range(h // 2) for x in range(w // 2)
                 for v in (98 + y * 60 // 144,
                           98 + ((x + 2 * f) % (w // 2)) * 60 // (w // 2)))
open(sys.argv[1], 'wb').write(out)
PY
    mkdir -p "$WORK/initramfs/gfxin"
    python3 - "$WORK/initramfs/gfxin" <<'PY'
import sys
out = sys.argv[1]
w, h = 1920, 1080
y = bytes((x + r) // 16 for r in range(h) for x in range(w))
def uv(c):
    return bytes((64 + (i + r) // 8) if c == 'u' else (192 - (i + r) // 8)
                 for r in range(h // 2) for i in range(w // 2))
u, v = uv('u'), uv('v')
nv12 = bytearray(y)
for r in range(h // 2):
    row = bytearray(w)
    row[0::2] = u[r * 960:(r + 1) * 960]
    row[1::2] = v[r * 960:(r + 1) * 960]
    nv12 += row
open(out + '/video_frm_1920_1080_1_yuv420.dat', 'wb').write(nv12)
pad = bytes(w // 2)
p3 = bytearray(y)
for plane in (u, v):
    for r in range(h // 2):
        p3 += plane[r * 960:(r + 1) * 960] + pad
p3 += bytes((4 << 20) - len(p3))
open(out + '/video_frm_1920_1080_1_yuv4203P.dat', 'wb').write(p3)
open(out + '/video_frm_1000_200_1_argb4444.dat', 'wb').write(
    b'\xf0' * 400000)
PY
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
# the display path, in the SDK's order (S10_SysInit2, IPCAM1 EVB)
if [ -d /dko ]; then
    \$bb cp -a /dko /mnt/tmp/
    for m in nvt_ide kdrv_videoout kflow_videoout nvt_dispdev_panel \\
             $PANEL_DSI $PANEL_MOD; do
        \$bb chroot /mnt /bin/sh -c "insmod /tmp/dko/\$m.ko" >/dev/null 2>&1
        echo "NVT-DISP-INSMOD: \$m rc=\$?"
    done
    \$bb dmesg | \$bb grep -a "Hello, panel" | \$bb sed 's/^.*Hello/NVT-DISP-PANEL: Hello/'
    # the AI engines (S10_SysInit2's order; ai_op runs a second process)
    for m in "kdrv_ai ai_net_supported_num=5" kflow_cnn kflow_nue kflow_nue2 \\
             kflow_cpu kflow_dsp kflow_ai_net; do
        \$bb chroot /mnt /bin/sh -c "insmod /tmp/dko/\${m%% *}.ko \$(echo \$m | \$bb cut -s -d' ' -f2-)" >/dev/null 2>&1
        echo "NVT-AI-INSMOD: \${m%% *} rc=\$?"
    done
fi
for d in /sys/bus/platform/devices/*; do
    [ -e \$d/driver ] && echo "NVT-HDAL-BOUND: \${d##*/} \$(\$bb basename \$(\$bb readlink \$d/driver))"
done
[ -d /smp ] && \$bb cp -a /smp /uc /gfxin /encin /dispin /samples.sh /mnt/tmp/ && \$bb chroot /mnt /bin/sh /tmp/samples.sh
echo "NVT-HDAL-OK"
\$bb reboot -f
INIT
chmod +x "$WORK/initramfs/init"

# Run in the chroot: the SD card at /mnt/sd, where the samples keep files.
cat > "$WORK/initramfs/samples.sh" <<'SMP'
M=/lib/modules/4.19.91/kernel/drivers/mmc
for ko in core/mmc_core core/mmc_block host/mmc_na51055; do insmod $M/$ko.ko; done
for i in 1 2 3 4 5 6 7 8 9 10; do [ -b /dev/mmcblk0 ] && break; sleep 1; done
mount -t tmpfs -o size=1m sd /mnt && mkdir /mnt/sd /mnt/det
# a second card with the detectors' SquashFS: which mmcblk is which
SDCARD=/dev/mmcblk0
if [ -f /tmp/smp/.detcard ]; then
    for i in 1 2 3 4 5; do [ -b /dev/mmcblk1 ] && break; sleep 1; done
fi
for b in /dev/mmcblk0 /dev/mmcblk1; do
    if [ -b $b ] && [ "$(head -c 4 $b)" = hsqs ]; then
        mount -t squashfs -o ro $b /mnt/det
        [ $b = /dev/mmcblk0 ] && SDCARD=/dev/mmcblk1
    fi
done
mkfs.vfat $SDCARD >/dev/null && mount -t vfat $SDCARD /mnt/sd
cd /tmp/smp
run() {         # run <sample> [keys]: the keys 4 s apart, then q ($ARGS)
    n=$1
    shift
    ( sleep 6; for k in "$@"; do echo "$k"; sleep 4; done; echo q ) |
        timeout 60 ./$n $ARGS > /tmp/$n.out 2>&1
    RC=$?
    ERRS=$(grep -c -i 'err\|fail' /tmp/$n.out)
    echo "NVT-HDAL-RUN: $n${ARGS:+ $ARGS} rc=$RC errors=$ERRS"
    grep -i 'err\|fail' /tmp/$n.out | head -3 | sed 's/^/NVT-HDAL-ERR: /'
}
size() {
    [ -f "$1" ] && wc -c < "$1" || echo 0
}
run hd_video_record
echo "NVT-HDAL-RECORD-BYTES: $(size /mnt/sd/dump_bs_main.dat)"
echo "NVT-HDAL-REC-B64-BEGIN"
head -c 6000000 /mnt/sd/dump_bs_main.dat | base64
echo "NVT-HDAL-REC-B64-END"
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
cp /tmp/encin/video_frm_352_288_30_yuv420.dat /mnt/sd/
for t in 0 1; do                # H.265, H.264
    ARGS=$t run hd_video_encode_only
    echo "NVT-HDAL-ENCODE-BYTES: $t $(size /mnt/sd/dump_bs_main.dat)"
    echo "NVT-HDAL-ENC-B64-BEGIN $t"
    head -c 4000000 /mnt/sd/dump_bs_main.dat | base64
    echo "NVT-HDAL-ENC-B64-END $t"
    rm -f /mnt/sd/dump_bs_main.dat
done
rm -f /mnt/sd/*
cp /tmp/gfxin/* /mnt/sd/
px() {          # px <file> <offset> <bytes>: those bytes in hex
    hexdump -s $2 -n $3 -e '/1 "%02x"' "$1"
}
for k in c s o t l r j p d; do
    run hd_gfx_only $k
    f=$(sed -n 's/^result is //p' /tmp/hd_gfx_only.out | head -1)
    echo "NVT-HDAL-GFX: $k rc=$RC errors=$ERRS bytes=$(size "$f")"
    # the result's pixels at points of the picture (offsets for 1920x1080
    # NV12 unless noted)
    case $k in
    c) at="979510 1 979490 1 2563510 2" ;;      # logo, background, logo UV
    t) at="0 1 200000 2" ;;                     # 1000x200
    l) at="960510 1 960400 1 2554110 2" ;;
    r) at="1153200 1 2650800 2" ;;
    j) at="193200 1 577200 1 1153200 1 1537200 1 1844400 1" ;;
    o) at="0 1 2071680 1 2073600 2" ;;
    p) at="384100 1 2112020 2" ;;
    s) at="0 1 8294399 1 8294400 2 12441598 2" ;;   # 3840x2160
    d) at="4810 1 48100 1 134410 1 199210 1" ;;     # 480x270, 3 planes
    esac
    set -- $at
    v=
    while [ $# -gt 1 ]; do v="$v $(px "$f" $1 $2)"; shift 2; done
    echo "NVT-HDAL-GFX-PX: $k$v"
    rm -f /mnt/sd/hd_gfx* /mnt/sd/affine*
done
# the two with checks of their own: the DMA copy and 16-bit arithmetic
for k in m a; do
    run hd_gfx_only $k
    echo "NVT-HDAL-GFX: $k rc=$RC errors=$ERRS"
    grep -a "dma manages\|operation of two" /tmp/hd_gfx_only.out
done
# the display: colour bars from a file, then the sensor, each with a
# screendump of the panel half way
if [ -e /dev/nvt_ide0 ] && [ -x ./hd_video_output_only ]; then
    cp /tmp/dispin/video_frm_320_240_3_yuv422.dat /mnt/sd/
    snaprun() {     # snaprun <snapshot> <sample> [args]
        nm=$1 n=$2
        shift 2
        ( sleep 10; echo "NVT-DISP-SNAP $nm" > /dev/console; sleep 5; echo q ) |
            timeout 60 ./$n "$@" > /tmp/$n.out 2>&1
        RC=$?
        ERRS=$(grep -c -i 'err\|fail' /tmp/$n.out)
        echo "NVT-HDAL-RUN: $n${*:+ $*} rc=$RC errors=$ERRS"
        grep -i 'err\|fail' /tmp/$n.out | head -3 | sed 's/^/NVT-HDAL-ERR: /'
    }
    snaprun bars hd_video_output_only 1
    snaprun live hd_video_liveview
fi
# the AI engines: ai_op's operators -- 0 a fully connected layer on NUE,
# 1..4 NUE2 preprocessing -- each run with "r", its result dumped
if [ -e /dev/nvt_ai_module0 ] && [ -x ./ai_op ]; then
    for op in 0 1 2 3 4; do
        rm -f op_user_out.bin
        ( sleep 6; echo r; sleep 14; echo q ) | timeout 60 ./ai_op $op > /tmp/ai_op.out 2>&1
        RC=$?
        ERRS=$(grep -c -i 'err\|fail' /tmp/ai_op.out)
        echo "NVT-HDAL-RUN: ai_op $op rc=$RC errors=$ERRS"
        grep -i 'err\|fail' /tmp/ai_op.out | head -3 | sed 's/^/NVT-HDAL-ERR: /'
        echo "NVT-AI-OUT: $op $(size op_user_out.bin) $(md5sum < op_user_out.bin | cut -c1-32)"
        case $op in 1|3|4)
            echo "NVT-AI-B64-BEGIN $op"
            base64 op_user_out.bin
            echo "NVT-AI-B64-END $op";;
        esac
    done
    # a network: the SDK's ImageNet classifier on ai_pattern's picture
    if [ -f para/nvt_model.bin ] && [ -x ./ai_net ]; then
        ( sleep 8; echo r; sleep 40; echo q ) | timeout 120 ./ai_net 0 -1 0 > /tmp/ai_net.out 2>&1
        RC=$?
        ERRS=$(grep -c -i 'err\|fail' /tmp/ai_net.out)
        echo "NVT-HDAL-RUN: ai_net rc=$RC errors=$ERRS"
        grep -i 'err\|fail' /tmp/ai_net.out | head -3 | sed 's/^/NVT-HDAL-ERR: /'
        grep '^[1-5]\. no=' /tmp/ai_net.out | sed 's/^/NVT-AI-NET: /'
    fi
    # the turnkey's detectors on their pictures, from the SD card
    if [ -d /mnt/det/CNNLib ] && [ -x ./ai2_pdcnn_sample ]; then
        rm -f /mnt/sd/*
        cp -a /mnt/det/. /mnt/sd/
        mkdir -p /mnt/sd/det_results /mnt/sd/FDCNN/result/fd /mnt/sd/save_bmp
        for d in ai2_pdcnn_sample alg_pvdcnn_sample "alg_fdcnn_ai2_sample faces 0 1" \
                cddcnn_sample; do
            ( sleep 3; echo q ) | timeout 600 ./$d > /tmp/det.out 2>&1
            echo "NVT-HDAL-RUN: ${d%% *} rc=$?"
            grep -i 'err\|fail' /tmp/det.out | head -5 | sed 's/^/NVT-HDAL-ERR: /'
        done
        for f in /mnt/sd/det_results/*.txt /mnt/sd/FDCNN/result/fd/*.txt; do
            [ -f "$f" ] && tr -d '\r' < "$f" | sed "s|^|NVT-AI-DET: ${f##*/} |"
        done
        rm -rf /mnt/sd/CNNLib /mnt/sd/jpg /mnt/sd/FDCNN /mnt/sd/det_results \
            /mnt/sd/save_bmp
    fi
fi
rm -f /mnt/sd/*
run hd_audio_capture_only
echo "NVT-HDAL-AUDIO-BYTES: $(size /mnt/sd/audio_bs_16_2_48000_pcm.dat)"
echo "NVT-HDAL-PCM-B64-BEGIN"
head -c 384000 /mnt/sd/audio_bs_16_2_48000_pcm.dat | base64
echo "NVT-HDAL-PCM-B64-END"
# capture_only's file, played to the machine's audiodev (a wav file here)
run hd_audio_output_only
run hd_audio_livesound
rm -f /mnt/sd/*
run hd_audio_record             # AAC
echo "NVT-HDAL-AAC-BYTES: $(size /mnt/sd/dump_bs_16_2_48000_aac.dat)"
echo "NVT-HDAL-AAC-B64-BEGIN"
head -c 200000 /mnt/sd/dump_bs_16_2_48000_aac.dat | base64
echo "NVT-HDAL-AAC-B64-END"
SMP

(cd "$WORK/initramfs" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

echo "Booting $KERNEL on -M $MACHINE,sensor=$SENSOR${SAMPLES:+, with the HDAL samples}"
# Screendumps: the guest prints NVT-DISP-SNAP <name> while a picture is
# up, and this takes QEMU's console as $WORK/<name>.ppm.
(
    seen=
    while sleep 1; do
        [ -S "$WORK/mon.sock" ] || continue
        for n in $(grep -a -o 'NVT-DISP-SNAP [a-z0-9_]*' "$WORK/console.log" \
                   2>/dev/null | cut -d' ' -f2); do
            case " $seen " in *" $n "*) continue;; esac
            seen="$seen $n"
            python3 - "$WORK/mon.sock" "screendump $WORK/$n.ppm" <<'PY' || true
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
s.settimeout(5)
time.sleep(0.2)
s.sendall(sys.argv[2].encode() + b'\n')
time.sleep(1)
s.close()
PY
        done
    done
) &
snapper=$!
set +e
timeout 1500 "$QEMU" -M "$MACHINE,sensor=$SENSOR,flash-file=$WORK/nor.bin,audiodev=snd0" \
    -audiodev "wav,id=snd0,path=$WORK/out.wav" \
    -kernel "$KERNEL" -initrd "$WORK/initramfs.cpio" \
    -append "console=ttyS0,115200 rdinit=/init mtdparts=$MTDPARTS" \
    -drive "if=sd,index=0,format=raw,file=$WORK/sd.img" ${DETDRIVE:-} \
    -nographic -monitor "unix:$WORK/mon.sock,server=on,wait=off" \
    -serial stdio -nic none -no-reboot \
    > "$WORK/console.log" 2>&1
rc=$?
kill $snapper 2>/dev/null || true
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
if [ -n "$DISPLAY_KOS" ]; then
    for m in nvt_ide kdrv_videoout kflow_videoout nvt_dispdev_panel \
             $PANEL_DSI $PANEL_MOD; do
        expect "NVT-DISP-INSMOD: $m rc=0" "display: $m loads"
    done
    for b in "f0800000.ide nvt_ide" "dispobj nvt_dispobj" \
             "dispdev nvt_dispdev" ${PANEL_DSI:+"f0840000.dsi nvt_dsi"}; do
        expect "NVT-HDAL-BOUND: $b[[:space:]]*\$" "display: ${b% *} bound by ${b#* }"
    done
    expect "NVT-DISP-PANEL: Hello, panel: $PANEL_HELLO" "display: the $PANEL panel driver is up"
    for m in kdrv_ai kflow_cnn kflow_nue kflow_nue2 kflow_cpu kflow_dsp \
             kflow_ai_net; do
        expect "NVT-AI-INSMOD: $m rc=0" "AI: $m loads"
    done
    expect "NVT-HDAL-BOUND: f0c60000.ai kdrv_ai[[:space:]]*\$" "AI: the engines bound by kdrv_ai"
fi

if [ -n "$SAMPLES" ]; then
    for s in hd_video_record hd_video_snapshot hd_video_capture_only \
             "hd_video_encode_only 0" "hd_video_encode_only 1" \
             hd_audio_capture_only hd_audio_output_only hd_audio_livesound \
             hd_audio_record; do
        expect "NVT-HDAL-RUN: $s rc=0 errors=0" "$s runs without an HDAL error"
    done
    if grep -a -q 'NVT-HDAL-RECORD-BYTES: [1-9]' "$WORK/console.log"; then
        pass "hd_video_record writes its bitstream"
    else
        bad "hd_video_record writes its bitstream"
    fi
    for t in "0 H.265" "1 H.264"; do
        expect "NVT-HDAL-ENCODE-BYTES: ${t% *} [1-9]" \
            "hd_video_encode_only writes its ${t#* } bitstream"
    done
    # The record stream is H.265 in two tile columns: with x265 each is
    # an intra slice, and the stream decodes to the sensor's picture
    if strings "$QEMU" | grep -c x265_encoder_open >/dev/null &&
            command -v ffmpeg >/dev/null && python3 -c 'import PIL' 2>/dev/null; then
        sed -n "/NVT-HDAL-REC-B64-BEGIN/,/NVT-HDAL-REC-B64-END/p" \
            "$WORK/console.log" | grep -a -v 'NVT-HDAL' | tr -d '\r' |
            base64 -d > "$WORK/rec.bin" 2>/dev/null || true
        python3 - "$WORK/rec.bin" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read()
cut = d.rfind(b'\x00\x00\x00\x01')
open(sys.argv[1], 'wb').write(d[:cut] if len(d) >= 6000000 and cut > 0 else d)
PY
        ffmpeg -v error -xerror -f hevc -i "$WORK/rec.bin" -vf \
            "select=eq(n\,0)+eq(n\,4)" -vsync 0 "$WORK/rec%d.png" \
            > "$WORK/rec.log" 2>&1 || true
        look=$(python3 - "$WORK/rec1.png" "$WORK/rec2.png" <<'PY' 2>/dev/null || true
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
b0, b1 = box(ims[0]), box(ims[1])
print('size=%dx%d bars=%d ramp=%d moved=%d' % (w, h, bars,
      ramp[0] < ramp[1] < ramp[2], b0 >= 0 and b1 >= 0 and b0 != b1))
PY
)
        if [ ! -s "$WORK/rec.log" ] && [ "$look" = "size=2880x1620 bars=1 ramp=1 moved=1" ]; then
            pass "hd_video_record's tiled H.265 decodes to the sensor's moving picture ($look)"
        else
            bad "hd_video_record's tiled H.265 decodes to the sensor's moving picture ($look; $(head -c 200 "$WORK/rec.log"))"
        fi
    else
        echo "SKIP: decoding the record stream (QEMU without x265, or no ffmpeg / PIL)"
    fi
    # With x264 / x265 in QEMU the bitstreams are real: they decode
    # (ffmpeg, stopping at any error) to the input's pictures, in order
    for t in "0 hevc H.265 x265" "1 h264 H.264 x264"; do
        set -- $t
        if ! strings "$QEMU" | grep -c "$4_encoder_open" >/dev/null ||
                ! command -v ffmpeg >/dev/null; then
            echo "SKIP: decoding the $3 bitstream (QEMU without $4, or no ffmpeg)"
            continue
        fi
        sed -n "/NVT-HDAL-ENC-B64-BEGIN $1/,/NVT-HDAL-ENC-B64-END $1/p" \
            "$WORK/console.log" | grep -a -v 'NVT-HDAL' | tr -d '\r' |
            base64 -d > "$WORK/enc$1.bin" 2>/dev/null || true
        # the last NAL may be cut short by the transfer: drop it
        python3 - "$WORK/enc$1.bin" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read()
cut = d.rfind(b'\x00\x00\x00\x01')
open(sys.argv[1], 'wb').write(d[:cut] if len(d) >= 4000000 and cut > 0 else d)
PY
        ffmpeg -v error -xerror -f $2 -i "$WORK/enc$1.bin" \
            -f rawvideo -pix_fmt nv12 "$WORK/dec$1.yuv" \
            > "$WORK/dec$1.log" 2>&1 || true
        # the sample feeds the file's 30 frames round and round: find
        # where the stream starts in it, then every decoded picture must
        # be the next frame (PSNR), and the pictures must move
        look=$(python3 - "$WORK/initramfs/encin/video_frm_352_288_30_yuv420.dat" \
            "$WORK/dec$1.yuv" <<'PY' 2>/dev/null || true
import math, sys
fs = 352 * 288 * 3 // 2
src = open(sys.argv[1], 'rb').read()
dec = open(sys.argv[2], 'rb').read()
ins = [src[i:i + fs] for i in range(0, len(src), fs)]
outs = [dec[i:i + fs] for i in range(0, len(dec) - fs + 1, fs)][:60]
def psnr(a, b):
    se = sum((x - y) ** 2 for x, y in zip(a[::7], b[::7]))
    return 99.0 if not se else 10 * math.log10(255 ** 2 * len(a[::7]) / se)
if not outs:
    print('frames=0 psnr=0 moving=0')
    sys.exit()
k = max(range(len(ins)), key=lambda i: psnr(outs[0], ins[i]))
worst = min(psnr(o, ins[(k + i) % len(ins)]) for i, o in enumerate(outs))
moving = sum(a != b for a, b in zip(outs, outs[1:]))
print('frames=%d psnr=%.1f moving=%d' % (len(dec) // fs, worst, moving))
PY
)
        frames=$(echo "$look" | sed -n 's/.*frames=\([0-9]*\).*/\1/p')
        psnr=$(echo "$look" | sed -n 's/.*psnr=\([0-9]*\).*/\1/p')
        moving=$(echo "$look" | sed -n 's/.*moving=\([0-9]*\).*/\1/p')
        if [ -s "$WORK/dec$1.log" ] || [ "${frames:-0}" -lt 5 ] ||
                [ "${psnr:-0}" -lt 30 ] || [ "${moving:-0}" -lt 4 ]; then
            bad "$3 from hd_video_encode_only decodes to the input, in order ($look; $(head -c 200 "$WORK/dec$1.log"))"
        else
            pass "$3 from hd_video_encode_only decodes to the input, in order ($look)"
        fi
    done
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
    # what the engines computed: green is Y 93, U / V 2a 14 (the vendor's
    # argb_to_ycbcr), the background (x + y) / 16
    for e in "c 93 32 2a14:copy blends the opaque green logo over the picture" \
             "t 93 2a14:colour transform turns the logo into green NV12" \
             "l 93 38 2a14:a green line drawn, the picture beside it kept" \
             "r 93 2a14:a green rectangle drawn" \
             "j 93 4e 1c ff 87:a job list draws green, red, blue, white" \
             "o 43 00 837d:rotate flips the picture top to bottom" \
             "p 12 43bd:3-plane to 2-plane keeps Y and interleaves U / V" \
             "s 00 bb 40c0 fb05:scaling to 3840x2160 keeps the corners" \
             "d 05 32 4a b6:scaling to 480x270 averages 4x4 blocks"; do
        expect "NVT-HDAL-GFX-PX: ${e%%:*}[[:space:]]*\$" "hd_gfx_only '${e%% *}': ${e#*:}"
    done
    expect "NVT-HDAL-GFX: m rc=0 errors=0" "hd_gfx_only 'm' runs without an error"
    expect "dma manages to copy 1048576 bytes" \
        "hd_gfx_only 'm': the DMA copy of 1 MiB compares equal"
    expect "NVT-HDAL-GFX: a rc=0 errors=0" "hd_gfx_only 'a' runs without an error"
    expect "minus operation of two 16bits arrays ok" \
        "hd_gfx_only 'a': 16-bit subtraction gives the sample's answer"
    if grep -a -q 'NVT-HDAL-AUDIO-BYTES: [1-9]' "$WORK/console.log"; then
        pass "hd_audio_capture_only captures PCM"
    else
        bad "hd_audio_capture_only captures PCM"
    fi
    # The display: the bars hd_video_output_only shows from its file, and
    # the sensor's picture through hd_video_liveview
    if [ -n "$DISPLAY_KOS" ]; then
        for s in "hd_video_output_only 1" hd_video_liveview; do
            expect "NVT-HDAL-RUN: $s rc=0 errors=0" "$s runs without an HDAL error"
        done
        look=$(python3 - "$WORK/bars.ppm" "$WORK/live.ppm" <<'PY' 2>/dev/null || true
import sys
from PIL import Image
want = [(1, 1, 1), (1, 1, 0), (0, 1, 1), (0, 1, 0), (1, 0, 1), (1, 0, 0),
        (0, 0, 1), (0, 0, 0)]
def bars(path):
    im = Image.open(path).convert('RGB')
    w, h = im.size
    ok = all(tuple(int(c > 96) for c in im.getpixel((w * (2 * i + 1) // 16,
                                                     h // 6))) == want[i]
             for i in range(8))
    return '%dx%d:%d' % (w, h, ok)
print('bars=%s live=%s' % (bars(sys.argv[1]), bars(sys.argv[2])))
PY
)
        if [ -z "$look" ] && ! python3 -c 'import PIL' 2>/dev/null; then
            echo "SKIP: display screendump checks (no Python PIL)"
        else
            case "$look" in *bars=$PANEL_SIZE:1*) pass "the panel shows hd_video_output_only's colour bars ($PANEL_SIZE)";;
                *) bad "the panel shows hd_video_output_only's colour bars ($look)";; esac
            case "$look" in *live=$PANEL_SIZE:1*) pass "the panel shows the sensor's picture through hd_video_liveview";;
                *) bad "the panel shows the sensor's picture through hd_video_liveview ($look)";; esac
        fi
    fi
    # The AI engines: ai_op 0 fills inputs and weights with i & 7, so all
    # 10240 outputs are sum((k & 7)^2) = 4480; ops 1..4 are the test
    # picture in RGB planes -- 512x376, scaled to 384x282, less a planar
    # mean of 128, less DC means 128 / 127 / 126 -- which must agree.
    if [ -n "$DISPLAY_KOS" ]; then
        for op in 0 1 2 3 4; do
            expect "NVT-HDAL-RUN: ai_op $op rc=0 errors=0" "ai_op $op runs without an error"
        done
        expect "NVT-AI-OUT: 0 40960 $(python3 -c "
import hashlib, struct
print(hashlib.md5(struct.pack('<i', 4480) * 10240).hexdigest())")" \
            "ai_op 0: NUE's fully connected layer gives the host's 10240 x 4480"
        expect "NVT-AI-OUT: 2 324864 " "ai_op 2: NUE2 scales to 384x282 (R, G, B planes)"
        for op in 1 3 4; do
            sed -n "/NVT-AI-B64-BEGIN $op/,/NVT-AI-B64-END $op/p" "$WORK/console.log" |
                grep -a -v 'NVT-AI' | tr -d '\r' | base64 -d > "$WORK/ai$op.bin" 2>/dev/null || true
        done
        look=$(python3 - "$WORK" <<'PY' 2>/dev/null || true
import sys
w = sys.argv[1]
a1, a3, a4 = (open('%s/ai%d.bin' % (w, n), 'rb').read() for n in (1, 3, 4))
n = 512 * 376
ok = len(a1) == len(a3) == len(a4) == 3 * n
planar = ok and all(a3[i] == max(a1[i] - 128, 0) for i in range(0, 3 * n, 7))
dc = ok and all(a4[i] == max(a1[i] - (128, 127, 126)[i // n], 0)
                for i in range(0, 3 * n, 7))
means = [sum(a1[c * n:(c + 1) * n:13]) / len(a1[c * n:(c + 1) * n:13])
         for c in range(3)] if ok else []
print('sizes=%d planar=%d dc=%d picture=%d' % (ok, planar, dc,
      ok and all(120 < m < 240 for m in means)))
PY
)
        case "$look" in *sizes=1*) pass "ai_op 1 / 3 / 4: NUE2's 512x376 RGB planes";;
            *) bad "ai_op 1 / 3 / 4: NUE2's 512x376 RGB planes ($look)";; esac
        case "$look" in *planar=1*dc=1*) pass "ai_op 3 / 4: the planar and DC mean subtractions agree with op 1's picture";;
            *) bad "ai_op 3 / 4: the mean subtractions agree with op 1's picture ($look)";; esac
        case "$look" in *picture=1*) pass "ai_op 1: the test picture's colours (a sunlit scene)";;
            *) bad "ai_op 1: the test picture's colours ($look)";; esac
        # ai_net, when the samples carry a model for this chip: NUE2, the
        # CNN's 93 jobs and NUE's FC must make it a sea snake (ImageNet 65)
        if [ -f "$SAMPLES/smp/para/nvt_model.bin" ]; then
            expect "NVT-HDAL-RUN: ai_net rc=0 errors=0" "ai_net runs without an error"
            top=$(grep -a -m1 'NVT-AI-NET: 1\. no=' "$WORK/console.log" | tr -d '\r' || true)
            score=$(echo "$top" | sed -n 's/.*score=\([0-9.]*\).*/\1/p')
            case "$top" in
            *"no=65, label=sea snake"*)
                if awk "BEGIN { exit !(${score:-0} >= 0.3) }"; then
                    pass "ai_net: the picture is a sea snake ($score)"
                else
                    bad "ai_net: a sea snake, but only at $score"
                fi;;
            *) bad "ai_net: the picture is a sea snake (${top:-no result})";;
            esac
        else
            echo "SKIP: ai_net (the samples carry no model for this chip)"
        fi
        # the turnkey detectors: each must find what the pictures hold
        if [ -d "$SAMPLES/sd" ]; then
            for d in ai2_pdcnn_sample alg_pvdcnn_sample alg_fdcnn_ai2_sample \
                    cddcnn_sample; do
                expect "NVT-HDAL-RUN: $d rc=0" "$d runs to its end"
            done
            look=$(grep -a '^NVT-AI-DET: ' "$WORK/console.log" | tr -d '\r' |
                   python3 -c '
import sys
# NVT-AI-DET: <result file> <its line>
det = [l.split()[1:] for l in sys.stdin]

def near(c, want, tol):
    return abs(c[0] - want[0]) <= tol and abs(c[1] - want[1]) <= tol

def centre(x0, y0, x1, y1):
    return ((x0 + x1) / 2, (y0 + y1) / 2)

# pdcnn: picture score x y w h (1920x1080); the people in its pictures
pd = [(r[1], float(r[2]), centre(float(r[3]), float(r[4]),
                                 float(r[3]) + float(r[5]),
                                 float(r[4]) + float(r[6])))
      for r in det if r[0].startswith("pd_") and len(r) >= 7]
people = [("comp.yuv", (383, 320)), ("comp.yuv", (848, 332)),
          ("bball.yuv", (453, 615))]
pdok = sum(any(n == want_n and s >= 0.8 and near(c, want, 90)
               for n, s, c in pd) for want_n, want in people)
# pvdcnn: picture class score x0 y0 x1 y1; class 1 people, 2 vehicles
pv = [(r[1], int(r[2]), float(r[3]), centre(*map(float, r[4:8])))
      for r in det if r[0].startswith("PVD") and len(r) >= 8]
pvok = sum(any(n == "comp.yuv" and k == 1 and s >= 0.8 and near(c, want, 90)
               for n, k, s, c in pv)
           for want in ((459, 345), (1233, 298), (498, 768)))
car = any(n == "huang.yuv" and k == 2 and s >= 0.8 and near(c, (110, 466), 60)
          for n, k, s, c in pv)
# fdcnn, a file a frame: x y w h score, then five landmarks, eyes first
faces = {}
for r in det:
    if r[0][0].isdigit() and len(r) > 6:
        faces.setdefault(r[0], []).append([int(v) for v in r[1:]])
eyes = any(f[4] >= 14000 and abs(f[5] - 207) <= 12 and abs(f[6] - 96) <= 12
           and abs(f[7] - 243) <= 12 and abs(f[8] - 100) <= 12
           for f in faces.get("000000.txt", []))
bface = any(near(centre(f[0], f[1], f[0] + f[2], f[1] + f[3]), (86, 115), 15)
            for f in faces.get("000001.txt", []))
# cddcnn: picture category score x1 y1 x2 y2 (1920x1080), what the
# network proposes (cdd_raw_results.txt) -- category 1, the pets -- and
# what limit_fdet confirms (cdd_test_results.txt, not checked: its
# classifier puts these pets below its threshold). The sample exits 0
# only if every picture went through both, so with "rc=0" no row for
# the dashcam frame means no proposal there.
cdd = [(r[1], int(r[2]), float(r[3]), centre(*map(float, r[4:8])))
       for r in det if r[0] == "cdd_raw_results.txt" and len(r) >= 8]
pets = sum(any(n == want_n and k == 1 and s >= 0.8 and near(c, want, 200)
               for n, k, s, c in cdd)
           for want_n, want in (("cat.yuv", (636, 540)), ("dog.yuv", (958, 563))))
nopet = not any(n == "huang.yuv" for n, k, s, c in cdd)
print("pd=%d/3 pvd=%d/3 car=%d eyes=%d bface=%d pets=%d/2 nopet=%d"
      % (pdok, pvok, car, eyes, bface, pets, nopet))
' 2>/dev/null || true)
            case "$look" in *pd=3/3*) pass "ai2_pdcnn_sample: the three people ($look)";;
                *) bad "ai2_pdcnn_sample: the three people (${look:-no result})";; esac
            case "$look" in *pvd=3/3*car=1*) pass "alg_pvdcnn_sample: three people and the car ($look)";;
                *) bad "alg_pvdcnn_sample: three people and the car (${look:-no result})";; esac
            case "$look" in *eyes=1*bface=1*) pass "alg_fdcnn_ai2_sample: both faces, the astronaut's eyes in place ($look)";;
                *) bad "alg_fdcnn_ai2_sample: both faces (${look:-no result})";; esac
            case "$look" in *pets=2/2*nopet=1*) pass "cddcnn_sample: the cat and the dog, no pet in the street ($look)";;
                *) bad "cddcnn_sample: the cat and the dog (${look:-no result})";; esac
        fi
    fi
    # The audio: the captured test tone (1 kHz left, 1.5 kHz right), what
    # output_only and livesound played to the wav audiodev, and the AAC
    # record encoded from it, each measured on the host
    for b in PCM AAC; do
        sed -n "/NVT-HDAL-$b-B64-BEGIN/,/NVT-HDAL-$b-B64-END/p" \
            "$WORK/console.log" | grep -a -v 'NVT-HDAL' | tr -d '\r' |
            base64 -d > "$WORK/audio.$b" 2>/dev/null || true
    done
    if command -v ffmpeg >/dev/null; then
        ffmpeg -v error -f aac -i "$WORK/audio.AAC" -f s16le -ac 2 \
            -ar 48000 "$WORK/aac.pcm" > /dev/null 2>&1 || true
    fi
    tones=$(python3 - "$WORK" <<'PY' 2>/dev/null || true
import math, os, struct, sys
w = sys.argv[1]
def pcm(path, skip=0):
    try:
        d = open(path, 'rb').read()[skip:]
    except OSError:
        return [], []
    v = struct.unpack('<%dh' % (len(d) // 4 * 2), d[:len(d) // 4 * 4])
    return v[0::2], v[1::2]
def power(x, f, rate):          # Goertzel
    c = 2 * math.cos(2 * math.pi * f / rate)
    s1 = s2 = 0.0
    for v in x:
        s1, s2 = v + c * s1 - s2, s1
    return s1 * s1 + s2 * s2 - c * s1 * s2
def tone(x, f, rate=48000):     # f dominant in the loudest 0.1 s
    x = list(x)
    n = rate // 10
    if len(x) < 2 * n:
        return 0
    best = max(range(0, len(x) - n, n // 2),
               key=lambda i: sum(abs(v) for v in x[i:i + n:8]))
    x = x[best:best + n]
    p = power(x, f, rate)
    others = [power(x, g, rate) for g in (300, 700, 2500, 4000)]
    return int(p > 100 * max(others) and max(abs(v) for v in x) > 300)
l, r = pcm(w + '/audio.PCM')
cap = tone(l, 1000) and tone(r, 1500) and max(l) > 15000
o, _ = pcm(w + '/out.wav', 44)
try:                            # the backend's rate, not the guest's
    wr = struct.unpack('<I', open(w + '/out.wav', 'rb').read()[24:28])[0]
except (OSError, struct.error):
    wr = 48000
out = tone(o, 1000, wr) and tone(o, 1500, wr)
a, b = pcm(w + '/aac.pcm')
aac = tone(a, 1000) and tone(b, 1500)
print('capture=%d played=%d aac=%d' % (cap, out, aac))
PY
)
    case "$tones" in *capture=1*) pass "hd_audio_capture_only captures the test tone: 1 kHz left, 1.5 kHz right";;
        *) bad "hd_audio_capture_only captures the test tone ($tones)";; esac
    case "$tones" in *played=1*) pass "output_only / livesound play it to the audiodev (wav)";;
        *) bad "output_only / livesound play it to the audiodev ($tones)";; esac
    if command -v ffmpeg >/dev/null; then
        case "$tones" in *aac=1*) pass "hd_audio_record's AAC decodes to the tone";;
            *) bad "hd_audio_record's AAC decodes to the tone ($tones)";; esac
    else
        echo "SKIP: decoding hd_audio_record's AAC (no ffmpeg)"
    fi
fi
if [ $rc -ne 0 ]; then
    echo "FAIL: QEMU exited $rc (124: a wait never ended)"
    fail=1
fi
exit $fail
