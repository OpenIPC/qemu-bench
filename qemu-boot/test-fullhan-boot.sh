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
#     erase and program).
#
#   bash qemu-boot/test-fullhan-boot.sh --soc fh8852v200 \
#       [--machine fh8852v200] [--qemu ./qemu-src/build/qemu-system-arm] \
#       [--dir qemu-boot] [--timeout 120]
set -euo pipefail

SOC=fh8852v200
MACHINE=
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
TIMEOUT=120
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
MACHINE=${MACHINE:-$SOC}
KERNEL=$DIR/uImage.$SOC
ROOTFS=$DIR/rootfs.squashfs.$SOC

MEM=40M
ENV=
GMAC=1
case $SOC in
    fh8626v100)
        MTDID=spi0.0                   # 4.19, despite the suffix
        # Its DT reserves CMA and the ARC's memory up to the top of 64 MB,
        # and load_fullhan writes osmem and reboots until it reads 64M.
        MEM=64M
        ENV=osmem=64M
        GMAC=0
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

head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$IMG"
python3 - "$IMG" "$ENV" <<'EOF'
import struct, sys, zlib
data = b''.join(v.encode() + b'\0' for v in sys.argv[2].split()) + b'\0'
data = data.ljust(0x10000 - 4, b'\0')
with open(sys.argv[1], 'r+b') as f:
    f.seek(0x40000)
    f.write(struct.pack('<I', zlib.crc32(data)) + data)
EOF
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
