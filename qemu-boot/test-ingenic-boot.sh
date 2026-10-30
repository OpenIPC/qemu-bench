#!/bin/bash
# Boot an OpenIPC Ingenic release on its machine and check what it did on
# the way (docs/ingenic.md).  Two modes, both on an 8 MiB NOR image laid
# out the OpenIPC way,
#
#   256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)
#
# with uImage.<soc> at 0x50000 and rootfs.squashfs.<soc> at 0x250000 (as
# fetch-openipc-lite.sh leaves them in DIR):
#
#   --mode kernel  -kernel uImage, root=/dev/mtdblock3 on the NOR: the
#                  kernel's SFC driver (DMA, interrupts) reads the root;
#   --mode flash   the release's U-Boot at 0 (u-boot-<soc>-nor.bin, fetched
#                  into DIR when missing) and no -kernel: the mask ROM
#                  stand-in runs the SPL, which brings up the PLLs and DDR
#                  and loads U-Boot, which saves its environment on first
#                  boot (erase + program) and boots the kernel from flash.
#
# The checks:
#   - kernel up, root mounted from the NOR, "Welcome to OpenIPC", the login
#     prompt, no kernel panic;
#   - flash mode: the SPL and U-Boot banners, and an environment with a
#     valid CRC32 at 0x40000 afterwards (written back to the image);
#   - a JFFS2 node in rootfs_data afterwards: the overlay was erased and
#     programmed through the SFC and written back to the image;
#   - with --net: a DHCP lease from QEMU's user network over the GMAC.
#
#   bash qemu-boot/test-ingenic-boot.sh --soc t31 [--mode kernel|flash]
#       [--qemu ./qemu-src/build/qemu-system-mipsel] [--dir qemu-boot]
#       [--timeout 240] [--net]
set -euo pipefail

SOC=t31
MODE=kernel
QEMU=./qemu-src/build/qemu-system-mipsel
DIR=qemu-boot
TIMEOUT=240
NET=0
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --mode) MODE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        --net) NET=1; shift ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
case $SOC in
    t10|t20) UBOOT=u-boot-$SOC-nor.bin ;;
    t21|t23|t30|t31) UBOOT=u-boot-${SOC}n-nor.bin ;;
    t40) UBOOT=u-boot-t40n-universal.bin ;;
    *) echo "no OpenIPC release for $SOC" >&2; exit 2 ;;
esac
case $SOC in
    t40) MEM="mem=64M rmem=64M@0x4000000"; RAM=128M ;;
    *)   MEM="mem=40M rmem=24M@0x2800000"; RAM=64M ;;
esac
KERNEL=$DIR/uImage.$SOC
ROOTFS=$DIR/rootfs.squashfs.$SOC
URL=https://github.com/OpenIPC/firmware/releases/download/latest

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
IMG=$WORK/flash.bin
LOG=$WORK/serial.log

head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$IMG"
if [ "$MODE" = flash ]; then
    [ -f "$DIR/$UBOOT" ] || curl -fsSL --retry 3 -o "$DIR/$UBOOT" "$URL/$UBOOT"
    dd if="$DIR/$UBOOT" of="$IMG" conv=notrunc status=none
fi
dd if="$KERNEL" of="$IMG" bs=64k seek=5 conv=notrunc status=none
dd if="$ROOTFS" of="$IMG" bs=64k seek=37 conv=notrunc status=none

if [ $NET -eq 1 ]; then
    NIC="-nic user,restrict=on"
else
    NIC="-nic none"
fi
if [ "$MODE" = flash ]; then
    BOOT=()
else
    APPEND="$MEM console=ttyS1,115200n8 panic=20 root=/dev/mtdblock3"
    APPEND="$APPEND rootfstype=squashfs init=/init"
    APPEND="$APPEND mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"
    BOOT=(-kernel "$KERNEL" -append "$APPEND")
fi

timeout "$TIMEOUT" "$QEMU" -M "$SOC,flash-file=$IMG" -m $RAM "${BOOT[@]}" \
    -display none -monitor none -serial "file:$LOG" $NIC &
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

# Wait for the login prompt, then for the overlay to be written (and the
# DHCP client, with --net).
for _ in $(seq "$TIMEOUT"); do
    grep -aq 'login:' "$LOG" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
for _ in $(seq 20); do
    [ $NET -eq 0 ] && { sleep 5; break; }
    grep -aq 'lease of' "$LOG" 2>/dev/null && break
    sleep 1
done
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true

echo "$SOC, $MODE boot:"
if [ "$MODE" = flash ]; then
    check "SPL" 'U-Boot SPL'
    check "U-Boot boots the kernel" 'Starting kernel'
fi
check "kernel up" 'Linux version'
check "root from the NOR" 'VFS: Mounted root (squashfs'
check "OpenIPC userland" 'Welcome to OpenIPC'
check "login prompt" 'login:'
[ $NET -eq 1 ] && check "DHCP lease over the GMAC" 'lease of 10.0.2.15'
if grep -aq 'Kernel panic' "$LOG"; then
    echo "  FAIL: no kernel panic"
    fail=1
else
    echo "  PASS: no kernel panic"
fi
if python3 - "$IMG" "$MODE" <<'EOF'
import sys, zlib
d = open(sys.argv[1], 'rb').read()
ok = any(d[i:i + 2] == b'\x85\x19' for i in range(0x750000, len(d), 4))
print('  %s: JFFS2 overlay written back to rootfs_data'
      % ('PASS' if ok else 'FAIL'))
if sys.argv[2] == 'flash':
    env = d[0x40000:0x50000]
    good = int.from_bytes(env[:4], 'little') == zlib.crc32(env[4:])
    print('  %s: U-Boot saved its environment at 0x40000'
          % ('PASS' if good else 'FAIL'))
    ok = ok and good
sys.exit(0 if ok else 1)
EOF
then :; else fail=1; fi
if [ $fail -ne 0 ]; then
    echo "--- serial log (tail) ---"
    tail -40 "$LOG"
fi
exit $fail
