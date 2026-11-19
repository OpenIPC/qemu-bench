#!/bin/bash
# Boot an OpenIPC Fullhan release (uImage.<soc> + rootfs.squashfs.<soc>, as
# fetch-openipc-lite.sh leaves them) to its login prompt and check what it
# did on the way.
#
# OpenIPC publishes no Fullhan bootloader, and its kernels have no RAM disk
# driver, so the kernel is booted with -kernel and its root is the SquashFS
# on the SPI NOR: a synthetic 8 MiB image laid out the OpenIPC way,
#
#   256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)
#
# env holds a valid U-Boot environment, as on a camera. OpenIPC's
# fw_printenv finds it by CRC when there is no /etc/fw_env.config, and
# with none to find it CRCs 64-256 KiB at every 4 KiB of the whole flash:
# seconds per call, a dozen calls on the way to login.
#
# The checks:
#   - "Welcome to OpenIPC" and the login prompt: the kernel booted on the
#     timers, interrupt controller and UART, and read its root over SPI0
#     (the FH SPIC on V200, through the AXI DMAC) from the NOR;
#   - a DHCP lease from QEMU's user network: the GMAC, its PHY and the
#     interrupts that carry its frames (not on fh8626v100, whose kernel
#     builds no Ethernet driver: the board it was brought up on is Wi-Fi
#     only);
#   - JFFS2 nodes in rootfs_data afterwards: the overlay was erased and
#     programmed through the SPI controller and written back to the image
#     (the 3.0.8 kernels' m25p80 also waits for WEL to clear after every
#     erase and program);
#   - fh8626v100 only: its 3A daemon starts. OpenIPC's DT puts a GC1054 at
#     I2C0 0x21 behind the CSI and ISP, so the machine gets sensor=gc1054
#     (its default, gc2053, is the vendor camera dump's); with no sensor
#     the capture driver never registers the ISP subdevice fh8626-3ad
#     waits for, and the daemon gives up after 10 s;
#   - fh8626v100 only, in a second boot with a script as PID 1: its kernel
#     starts the ARC (fh-arc) with OpenIPC's firmware from the image, the
#     firmware announces rpmsg-arc-echo, and fh-rpmsg-echo.c -- copied out
#     of rootfs_data, where the test writes it -- gets 64 messages back.
#     The client is built with arm-linux-gnueabi-gcc unless --echo-client
#     names one;
#   - fh8626v100 only, in the same boot: the CSI -> ISP -> VPU frame path
#     (fh-csi, fh-isp). With fh8626-3ad started by its own init script,
#     fh-v4l2-capture.c (built the same way, or --capture-client) streams
#     100 frames from /dev/video0 and checks them, and the daemon's AE and
#     AWB act on the model's statistics: its white balance lands on the
#     gains the model's light calls for (R/G 0.739, B/G 0.399 -> 693 and
#     1282, Q9), within 3%;
#   - fh8626v100 only, in the same boot: the PAE H.264 and JPEG encoders
#     (fh-pae, fh-jpeg). The client feeds 20 frames through OpenIPC's
#     fh8626-encoder (GOP 10) and one through fh8626-jpeg and writes both
#     streams to rootfs_data, which ffmpeg then decodes here: 20 frames at
#     1280x720, 2 IDR and 18 P, each frame's mean luma within 8 of the
#     captured picture's; the JPEG at 1280x720. Without ffprobe on the
#     host the streams are only found and their sizes checked.
#
#   bash qemu-boot/test-fullhan-boot.sh --soc fh8852v200 \
#       [--machine fh8852v200] [--qemu ./qemu-src/build/qemu-system-arm] \
#       [--dir qemu-boot] [--timeout 120] [--echo-client PATH]
#       [--capture-client PATH]
set -euo pipefail

SOC=fh8852v200
MACHINE=
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
TIMEOUT=120
CLIENT=
CAPTURE=
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        --echo-client) CLIENT=$2; shift 2 ;;
        --capture-client) CAPTURE=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
MACHINE=${MACHINE:-$SOC}
[ "$SOC" = fh8626v100 ] && [ "$MACHINE" = fh8626v100 ] &&
    MACHINE=fh8626v100,sensor=gc1054
KERNEL=$DIR/uImage.$SOC
ROOTFS=$DIR/rootfs.squashfs.$SOC

MEM=40M
ENV=
GMAC=1
ARC=0
case $SOC in
    fh8626v100)
        MTDID=spi0.0                   # 4.19, despite the suffix
        # Its DT reserves CMA and the ARC's memory up to the top of 64 MB,
        # and load_fullhan writes osmem and reboots until it reads 64M.
        MEM=64M
        ENV=osmem=64M
        GMAC=0
        ARC=1
        ;;
    *v100) MTDID=spi_flash ;;          # the 3.0.8 m25p80 platform data's
    *)     MTDID=spi0.0 ;;
esac
APPEND="mem=$MEM console=ttyS0,115200 panic=20 root=/dev/mtdblock3"
APPEND="$APPEND rootfstype=squashfs init=/init"
APPEND="$APPEND mtdparts=$MTDID:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
IMG=$WORK/flash.bin
LOG=$WORK/serial.log

# $1: the image, laid out as above
make_image() {
    head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$1"
    python3 - "$1" "$ENV" <<'EOF'
import struct, sys, zlib
data = b''.join(v.encode() + b'\0' for v in sys.argv[2].split()) + b'\0'
data = data.ljust(0x10000 - 4, b'\0')
with open(sys.argv[1], 'r+b') as f:
    f.seek(0x40000)
    f.write(struct.pack('<I', zlib.crc32(data)) + data)
EOF
    dd if="$KERNEL" of="$1" bs=64k seek=5 conv=notrunc status=none
    dd if="$ROOTFS" of="$1" bs=64k seek=37 conv=notrunc status=none
}
make_image "$IMG"

timeout "$TIMEOUT" "$QEMU" -M "$MACHINE,flash-file=$IMG" \
    -kernel "$KERNEL" -append "$APPEND" \
    -display none -monitor none -serial "file:$LOG" \
    -nic user,restrict=on &
QPID=$!

fail=0
# $1: what; fh8626-3ad's init script's verdict in $LOG is OK.
check_3ad() {
    # "OK" or "FAIL (...)", after the "Starting fh8626-3ad: " prefix or on
    # a line of its own once the daemon has printed, and before the next
    # script starts. rcS runs the scripts in turn, so it is in the log
    # before login:.
    if awk '/Starting fh8626-3ad:/ { f = 1 }
            f && /(^|fh8626-3ad: )FAIL \(/ { exit 1 }
            f && /(^|fh8626-3ad: )OK\r?$/ { ok = 1; exit 0 }
            f && /^(Starting |capture: )/ && !/fh8626-3ad:/ { exit 1 }
            END { exit !ok }' "$LOG"; then
        echo "  PASS: $1"
    else
        echo "  FAIL: $1"
        fail=1
    fi
}

# The streams fh-v4l2-capture wrote to rootfs_data, decoded by ffmpeg.
check_streams() {
    local ymean
    if ! python3 - "$IMG" "$WORK" <<'EOF'
import struct, sys
d = open(sys.argv[1], 'rb').read()[0x750000 + 0x30000:]
if d[:4] != b'FHEN':
    sys.exit(1)
jl, hl, n = struct.unpack('<III', d[4:16])
at = 16 + ((jl + 3) & ~3)
open(sys.argv[2] + '/out.jpg', 'wb').write(d[16:16 + jl])
open(sys.argv[2] + '/out.h264', 'wb').write(d[at:at + hl])
EOF
    then
        echo "  FAIL: the streams in rootfs_data"
        fail=1
        return
    fi
    echo "  PASS: the streams in rootfs_data"
    if ! command -v ffprobe >/dev/null; then
        echo "  SKIP: decoding them (no ffprobe on this host)"
        return
    fi
    check_ffprobe "H.264: 20 frames, 1280x720" "$WORK/out.h264" \
        stream=width,height,nb_read_frames '1280,720,20'
    check_ffprobe "H.264: IDR every 10 frames, P between" "$WORK/out.h264" \
        frame=pict_type 'IPPPPPPPPPIPPPPPPPPP'
    check_ffprobe "JPEG: 1280x720" "$WORK/out.jpg" stream=width,height \
        '1280,720'
    ymean=$(sed -n 's/^capture: .*Y mean [0-9]* -> \([0-9]*\).*/\1/p' \
            "$ARCLOG" | head -1)
    if ffmpeg -v error -i "$WORK/out.h264" \
            -vf signalstats,metadata=print:key=lavfi.signalstats.YAVG:file=- \
            -f null - 2>"$WORK/ffmpeg.err" | awk -F= -v y="${ymean:-0}" '
            /YAVG/ { n++; d = $2 - y; if (d < -8 || d > 8) bad++ }
            END { exit !(n == 20 && !bad) }' && [ ! -s "$WORK/ffmpeg.err" ]
    then
        echo "  PASS: H.264 decodes clean, each frame's mean luma near $ymean"
    else
        echo "  FAIL: H.264 decodes clean, each frame's mean luma near $ymean"
        head -5 "$WORK/ffmpeg.err"
        fail=1
    fi
}

# $1: what; $2: file; $3: ffprobe entries; $4: their values, joined
check_ffprobe() {
    local got
    got=$(ffprobe -v error -count_frames -select_streams v:0 \
          -show_entries "$3" -of csv=p=0 "$2" 2>&1 | tr -d '\n' | tr -d ' ')
    if [ "$got" = "$4" ]; then
        echo "  PASS: $1"
    else
        echo "  FAIL: $1 ($got)"
        fail=1
    fi
}

check() {
    if grep -aq "$2" "$LOG"; then
        echo "  PASS: $1"
    else
        echo "  FAIL: $1"
        fail=1
    fi
}

# Wait for the login prompt, then a little longer for the DHCP client.
for _ in $(seq "$TIMEOUT"); do
    grep -aq 'login:' "$LOG" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
for _ in $(seq $((GMAC * 20))); do
    grep -aq 'lease of' "$LOG" 2>/dev/null && break
    sleep 1
done
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true

echo "$SOC on $MACHINE:"
check "kernel up" 'Linux version'
check "OpenIPC userland" 'Welcome to OpenIPC'
check "login prompt" 'login:'
if [ $GMAC -eq 1 ]; then
    check "DHCP lease over the GMAC" 'lease of 10.0.2.15'
fi
if [ $ARC -eq 1 ]; then
    check_3ad "fh8626-3ad (ISP 3A daemon) started"
fi
# A JFFS2 node starts 0x1985 (little-endian) at a 4-byte boundary.
if python3 - "$IMG" <<'EOF'
import sys
d = open(sys.argv[1], 'rb').read()[0x750000:]
sys.exit(0 if any(d[i:i + 2] == b'\x85\x19' for i in range(0, len(d), 4))
         else 1)
EOF
then
    echo "  PASS: JFFS2 overlay written back to rootfs_data"
else
    echo "  FAIL: JFFS2 overlay written back to rootfs_data"
    fail=1
fi
if [ $fail -ne 0 ]; then
    echo "--- serial log (tail) ---"
    tail -40 "$LOG"
fi

if [ $ARC -eq 1 ]; then
    if [ -z "$CLIENT" ]; then
        CLIENT=$WORK/fh-rpmsg-echo
        arm-linux-gnueabi-gcc -static -nostdlib -ffreestanding -Os -marm \
            -o "$CLIENT" "$(dirname "$0")/fh-rpmsg-echo.c" -lgcc
    fi
    if [ -z "$CAPTURE" ]; then
        CAPTURE=$WORK/fh-v4l2-capture
        arm-linux-gnueabi-gcc -static -nostdlib -ffreestanding -Os -marm \
            -o "$CAPTURE" "$(dirname "$0")/fh-v4l2-capture.c" -lgcc
    fi
    ARCLOG=$WORK/arc.log
    for f in "$CLIENT" "$CAPTURE"; do
        if [ "$(stat -c %s "$f")" -gt 65536 ]; then
            echo "$f: over the 64 KiB slot it is given" >&2
            exit 2
        fi
    done
    make_image "$IMG"
    # rootfs_data (0x750000) carries the boot's script, then the clients,
    # 64 KiB apart: the kernel command line holds 1024 bytes, not a script.
    # The kernel prints that line, and the script echoes as it runs, so
    # the markers below are built at run time.
    rp=/sys/class/remoteproc/remoteproc0/state
    cat > "$WORK/test.sh" <<EOF
mount -t proc p /proc; mount -t sysfs s /sys; mount -t tmpfs r /run
cp() { dd if=/dev/mtdblock4 of=\$2 bs=65536 skip=\$1 count=1 2>/dev/null; chmod 755 \$2; }
cp 1 /tmp/e; cp 2 /tmp/c
modprobe virtio_rpmsg_bus; modprobe rpmsg_char
# the channel appears when the ARC's announcement is taken
echo start > $rp; i=0
while [ \$i -lt 20 ] && ! ls /sys/bus/rpmsg/devices/*arc-echo* >/dev/null 2>&1; do
    sleep 1; i=\$((i + 1))
done
# rpmsg_chrdev binds only by driver_override
for d in /sys/bus/rpmsg/devices/*arc-echo*; do
    echo rpmsg_chrdev > \$d/driver_override
    echo \${d##*/} > /sys/bus/rpmsg/drivers/rpmsg_chrdev/bind
done
/tmp/e
# the firmware's diagnostics block: state, error code
echo arc-diag \$(devmem 0xa3f07000) \$(devmem 0xa3f07004)
echo stop > $rp
# the media graph, the 3A daemon as its init script starts it, frames,
# and the H.264 and JPEG encoders; every node found by its name
for m in phy_fh8626_mipi gc1054 fh8626_capture fh8626_encoder fh8626_jpeg; do
    modprobe \$m
done
sleep 2
for f in /sys/class/video4linux/*/name; do
    d=\${f%/name}; d=/dev/\${d##*/}
    case \$(cat \$f) in
    gc1054*) sd=\$d ;;
    fh8626-isp) id=\$d ;;
    fh8626-camera) cam=\$d ;;
    fh8626-encoder) enc=\$d ;;
    fh8626-jpeg) jpg=\$d ;;
    esac
done
/etc/init.d/S80fh8626-3ad start
/tmp/c \$cam \$sd \$id 100 \$enc \$jpg /dev/mtdblock4
echo arc-test-\$((6 * 7))
sleep 600
EOF
    dd if="$WORK/test.sh" of="$IMG" bs=64k seek=117 conv=notrunc status=none
    dd if="$CLIENT" of="$IMG" bs=64k seek=118 conv=notrunc status=none
    dd if="$CAPTURE" of="$IMG" bs=64k seek=119 conv=notrunc status=none
    SCRIPT="mount -t tmpfs t /tmp"
    SCRIPT="$SCRIPT; dd if=/dev/mtdblock4 of=/tmp/t.sh bs=$(stat -c %s "$WORK/test.sh") count=1"
    SCRIPT="$SCRIPT; sh /tmp/t.sh"
    APPEND="mem=$MEM console=ttyS0,115200 root=/dev/mtdblock3"
    APPEND="$APPEND rootfstype=squashfs"
    APPEND="$APPEND mtdparts=$MTDID:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"
    timeout "$TIMEOUT" "$QEMU" -M "$MACHINE,flash-file=$IMG" \
        -kernel "$KERNEL" -append "$APPEND init=/bin/sh -- -c \"$SCRIPT\"" \
        -display none -monitor none -serial "file:$ARCLOG" -nic none \
        </dev/null &
    QPID=$!
    for _ in $(seq "$TIMEOUT"); do
        grep -aq 'arc-test-42' "$ARCLOG" 2>/dev/null && break
        kill -0 $QPID 2>/dev/null || break
        sleep 1
    done
    kill $QPID 2>/dev/null || true
    wait $QPID 2>/dev/null || true
    LOG=$ARCLOG
    before=$fail
    fail=0
    echo "$SOC ARC (fh-arc, OpenIPC's echo firmware):"
    check "remoteproc up" 'remote processor soc:remoteproc is now up'
    check "firmware announces rpmsg-arc-echo" 'creating channel rpmsg-arc-echo addr 0x400'
    check "64 messages echoed through the ARC" 'rpmsg echo: 64 ok'
    check "firmware state ECHO_RUNNING, no error" 'arc-diag 0x00000004 0x00000000'
    check "stopped" 'stopped remote processor soc:remoteproc'
    echo "$SOC frames (fh-csi, fh-isp, fh8626-3ad on the statistics):"
    check_3ad "fh8626-3ad started"
    check "100 NV12 frames from /dev/video0, changing" 'capture: ok'
    grep -a '^capture: [0-9]' "$ARCLOG" | head -1 | sed 's/^/  /'
    if awk '/^capture: [0-9]/ {
                seen = 1
                for (i = 1; i <= NF; i++) {
                    if ($i == "exposure") { e0 = $(i + 1); e1 = $(i + 3) }
                    if ($i == "gain") { g0 = $(i + 1); g1 = $(i + 3) }
                    if ($i == "wb") { split($(i + 3), w, "/") }
                }
                sub(",", "", e1); sub(",", "", g1)
                ok = (e1 != e0 || g1 != g0) &&
                     w[1] >= 672 && w[1] <= 714 && w[2] >= 1243 && w[2] <= 1321
                exit !ok
            }
            END { if (!seen) exit 1 }' "$ARCLOG"; then
        echo "  PASS: AE moved the sensor, AWB at the light's gains"
    else
        echo "  FAIL: AE moved the sensor, AWB at the light's gains"
        fail=1
    fi
    echo "$SOC encoders (fh-pae, fh-jpeg; OpenIPC's fh8626-encoder, -jpeg):"
    check "20 frames through the H.264 encoder, one through the JPEG one" \
        'encode: 20 frames'
    grep -a '^encode: ' "$ARCLOG" | head -1 | sed 's/^/  /'
    check_streams
    if [ $fail -ne 0 ]; then
        echo "--- ARC / frames serial log (tail) ---"
        tail -40 "$ARCLOG"
    fi
    fail=$((fail | before))
fi
exit $fail
