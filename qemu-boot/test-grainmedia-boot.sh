#!/bin/bash
# Boot an OpenIPC Grain Media release (uImage.<soc> + rootfs.squashfs.<soc>,
# as fetch-openipc-lite.sh leaves them) to its login prompt and check what
# it did on the way.
#
# OpenIPC publishes no GM bootloader (the cameras' nsboot + U-Boot are
# vendor code), so the kernel is booted with -kernel and its root is the
# SquashFS on the SPI NOR: a synthetic 8 MiB image laid out the OpenIPC way,
#
#   256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)
#
# The OpenIPC kernel takes those from mtdparts= (the vendor one only reads
# the NOR's system header), on "nor-flash".  mem= / gmmem= are what the
# vendor U-Boot passes for the RAM size: the media pool comes out of it.
#
# The checks:
#   - "Welcome to OpenIPC" and the login prompt: the kernel booted on the
#     FTTMR010, the FTINTC030 and the UART, and read its root over the
#     FTSPI020 through the FTDMAC020 from the NOR;
#   - the audio codec driver loaded: it panics unless the PMU's PLL1 is
#     one of the rates the cameras run at;
#   - a DHCP lease from QEMU's user network: the FTGMAC100, its PHY and
#     the interrupts that carry its frames;
#   - JFFS2 nodes in rootfs_data afterwards: the overlay was erased and
#     programmed through the SPI controller and written back to the image.
#
#   bash qemu-boot/test-grainmedia-boot.sh --soc gm8136 \
#       [--machine gm8136] [--qemu ./qemu-src/build/qemu-system-arm] \
#       [--dir qemu-boot] [--timeout 150]
set -euo pipefail

SOC=gm8136
MACHINE=
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
TIMEOUT=150
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
case $SOC in
    gm8135) MACHINE=${MACHINE:-gm8135s}; MEM="mem=64M gmmem=34M" ;;
    *)      MACHINE=${MACHINE:-$SOC};    MEM="mem=128M gmmem=90M" ;;
esac
KERNEL=$DIR/uImage.$SOC
ROOTFS=$DIR/rootfs.squashfs.$SOC

APPEND="$MEM console=ttyS0,115200 panic=20 root=/dev/mtdblock3"
APPEND="$APPEND rootfstype=squashfs init=/init"
APPEND="$APPEND mtdparts=nor-flash:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
IMG=$WORK/flash.bin
LOG=$WORK/serial.log

head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$IMG"
dd if="$KERNEL" of="$IMG" bs=64k seek=5 conv=notrunc status=none
dd if="$ROOTFS" of="$IMG" bs=64k seek=37 conv=notrunc status=none

timeout "$TIMEOUT" "$QEMU" -M "$MACHINE,flash-file=$IMG" \
    -kernel "$KERNEL" -append "$APPEND" \
    -display none -monitor none -serial "file:$LOG" \
    -nic user,restrict=on &
QPID=$!

fail=0
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
for _ in $(seq 20); do
    grep -aq 'lease of' "$LOG" 2>/dev/null && break
    sleep 1
done
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true

echo "$SOC on $MACHINE:"
check "kernel up" 'Linux version'
check "root from the NOR" 'VFS: Mounted root (squashfs'
check "OpenIPC userland" 'Welcome to OpenIPC'
check "login prompt" 'login:'
check "DHCP lease over the FTGMAC100" 'lease of 10.0.2.15'
if grep -aq 'Kernel panic' "$LOG"; then
    echo "  FAIL: no kernel panic"
    fail=1
else
    echo "  PASS: no kernel panic"
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
exit $fail
