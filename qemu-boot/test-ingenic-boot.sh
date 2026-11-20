#!/bin/bash
# Boot an OpenIPC Ingenic release on its machine and check what it did on
# the way (docs/ingenic.md).  Three modes, all on an 8 MiB NOR image.  Two
# lay it out in the fixed slots,
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
# The third is the split layout (OpenIPC/firmware#2912), the release's
# firmware.bin.<soc> -- its uImage, then its squashfs at the next 64 KiB --
# written whole at 0x50000, which the kernel divides at boot into kernel,
# rootfs and rootfs_data (OpenIPC/linux split_layout.c):
#
#   --mode split   the release's U-Boot at 0, as flash, and firmware.bin.<soc>
#                  from openipc.<soc>-nor-lite-split.tgz (fetched into DIR
#                  when missing; a release without one is a SKIP).
#
# The checks:
#   - kernel up, root mounted from the NOR, "Welcome to OpenIPC", the login
#     prompt, no kernel panic;
#   - flash and split modes: the SPL and U-Boot banners, and an environment
#     with a valid CRC32 at 0x40000 afterwards (written back to the image);
#   - split mode: the kernel's "split NOR layout", and the root on the rootfs
#     it split off (mtdblock4) rather than the slot the command line names;
#   - a JFFS2 node in rootfs_data afterwards (from 0x750000, or on the split
#     layout from the end of firmware.bin): the overlay was erased and
#     programmed through the SFC and written back to the image;
#   - with --net: a DHCP lease from QEMU's user network over the GMAC;
#   - with --reset: a system_reset from the monitor once the first boot is
#     at its login prompt, and a second boot in the same QEMU that gets
#     there too, without an RCU stall, and mounts the overlay about as
#     soon as the first did (guest time: translations kept from the first
#     boot had a T40's second boot mount it at 32 s instead of 3 s).
#
#   bash qemu-boot/test-ingenic-boot.sh --soc t31 [--mode kernel|flash|split]
#       [--qemu ./qemu-src/build/qemu-system-mipsel] [--dir qemu-boot]
#       [--timeout 240] [--net] [--reset]
set -euo pipefail

SOC=t31
MODE=kernel
QEMU=./qemu-src/build/qemu-system-mipsel
DIR=qemu-boot
TIMEOUT=240
NET=0
RESET=0
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --mode) MODE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        --net) NET=1; shift ;;
        --reset) RESET=1; shift ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
case $MODE in
    kernel|flash|split) ;;
    *) echo "unknown mode $MODE" >&2; exit 2 ;;
esac
URL=https://github.com/OpenIPC/firmware/releases/download/latest
UBOOT_URL=$URL
MACHINE=$SOC
MTDID=jz_sfc
case $SOC in
    t10|t20) UBOOT=u-boot-$SOC-nor.bin ;;
    t21|t23|t30|t31) UBOOT=u-boot-${SOC}n-nor.bin ;;
    t40) UBOOT=u-boot-t40n-universal.bin ;;
    # the firmware release has no T41 U-Boot; OpenIPC/u-boot publishes it
    t41) UBOOT=u-boot-t41nq-nor.bin
         UBOOT_URL=https://github.com/OpenIPC/u-boot/releases/download/ingenic
         MACHINE=t41,variant=t41nq MTDID=sfc0_nor ;;
    *) echo "no OpenIPC release for $SOC" >&2; exit 2 ;;
esac
case $SOC in
    t40) MEM="mem=64M rmem=64M@0x4000000"; RAM=128M ;;
    t41) MEM="mem=48M rmem=64M@0x3000000"; RAM=128M ;;
    *)   MEM="mem=40M rmem=24M@0x2800000"; RAM=64M ;;
esac
KERNEL=$DIR/uImage.$SOC
ROOTFS=$DIR/rootfs.squashfs.$SOC
FIRMWARE=$DIR/firmware.bin.$SOC
# The split package, when the release has one; only a 404 means it has not
if [ "$MODE" = split ] && [ ! -f "$FIRMWARE" ]; then
    code=$(curl -sL --retry 3 -o "$DIR/split.$SOC.tgz" -w '%{http_code}' \
        "$URL/openipc.$SOC-nor-lite-split.tgz") || code=000
    case $code in
        200) tar -xzf "$DIR/split.$SOC.tgz" -C "$DIR" "firmware.bin.$SOC"
             rm -f "$DIR/split.$SOC.tgz" ;;
        404) rm -f "$DIR/split.$SOC.tgz"
             echo "$SOC, split boot:"
             echo "  SKIP: the release has no openipc.$SOC-nor-lite-split.tgz"
             exit 0 ;;
        *) echo "fetching openipc.$SOC-nor-lite-split.tgz: HTTP $code" >&2
           exit 1 ;;
    esac
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
IMG=$WORK/flash.bin
LOG=$WORK/serial.log

head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$IMG"
if [ "$MODE" != kernel ]; then
    [ -f "$DIR/$UBOOT" ] ||
        curl -fsSL --retry 3 -o "$DIR/$UBOOT" "$UBOOT_URL/$UBOOT"
    dd if="$DIR/$UBOOT" of="$IMG" conv=notrunc status=none
fi
if [ "$MODE" = split ]; then
    dd if="$FIRMWARE" of="$IMG" bs=64k seek=5 conv=notrunc status=none
    # rootfs_data: where firmware.bin ends, at a 64 KiB block
    DATA=$(( 0x50000 + ($(stat -c %s "$FIRMWARE") + 0xffff) / 0x10000 * 0x10000 ))
else
    dd if="$KERNEL" of="$IMG" bs=64k seek=5 conv=notrunc status=none
    dd if="$ROOTFS" of="$IMG" bs=64k seek=37 conv=notrunc status=none
    DATA=$((0x750000))
fi

if [ $NET -eq 1 ]; then
    NIC="-nic user,restrict=on"
else
    NIC="-nic none"
fi
if [ "$MODE" != kernel ]; then
    BOOT=()
else
    APPEND="$MEM console=ttyS1,115200n8 panic=20 root=/dev/mtdblock3"
    APPEND="$APPEND rootfstype=squashfs init=/init"
    APPEND="$APPEND mtdparts=$MTDID:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"
    BOOT=(-kernel "$KERNEL" -append "$APPEND")
fi

MON=$WORK/monitor.sock
if [ $RESET -eq 1 ]; then
    MONITOR="unix:$MON,server,nowait"
else
    MONITOR=none
fi
timeout $((TIMEOUT * (RESET + 1))) "$QEMU" -M "$MACHINE,flash-file=$IMG" \
    -m $RAM "${BOOT[@]}" -display none -monitor "$MONITOR" \
    -serial "file:$LOG" $NIC &
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

# Wait for the Nth login prompt, then for the overlay to be written (and
# the DHCP client, with --net).
count() { grep -ao "$1" "$LOG" 2>/dev/null | wc -l; }
wait_boot() {
    for _ in $(seq "$TIMEOUT"); do
        [ "$(count 'login:')" -ge "$1" ] && break
        kill -0 $QPID 2>/dev/null || break
        sleep 1
    done
    for _ in $(seq 20); do
        [ $NET -eq 0 ] && { sleep 5; break; }
        [ "$(count 'lease of')" -ge "$1" ] && break
        sleep 1
    done
}
wait_boot 1
if [ $RESET -eq 1 ] && kill -0 $QPID 2>/dev/null; then
    python3 - "$MON" <<'PY'
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
time.sleep(0.5)
s.sendall(b'system_reset\n')
time.sleep(0.5)
PY
    wait_boot 2
fi
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true

echo "$SOC, $MODE boot:"
if [ "$MODE" != kernel ]; then
    check "SPL" 'U-Boot SPL'
    # "Starting kernel" from the vendor U-Boot 2013.07, "Uncompressing
    # Kernel Image" from mainline (OpenIPC/u-boot openipc-ingenic)
    check "U-Boot boots the kernel" \
        'Starting kernel\|Uncompressing Kernel Image'
fi
check "kernel up" 'Linux version'
check "root from the NOR" 'VFS: Mounted root (squashfs'
if [ "$MODE" = split ]; then
    check "the kernel splits firmware.bin" 'split NOR layout, kernel'
    check "root on the rootfs it split off" \
        'Mounted root (squashfs filesystem) readonly on device 31:4'
fi
check "OpenIPC userland" 'Welcome to OpenIPC'
check "login prompt" 'login:'
[ $NET -eq 1 ] && check "DHCP lease over the GMAC" 'lease of 10.0.2.15'
if grep -aq 'Kernel panic' "$LOG"; then
    echo "  FAIL: no kernel panic"
    fail=1
else
    echo "  PASS: no kernel panic"
fi
if [ $RESET -eq 1 ]; then
    # the overlay's mount notice per boot, in whole guest seconds
    MOUNT=($(grep -a 'jffs2: notice' "$LOG" |
             sed -n 's/^\[ *\([0-9]*\)\.[0-9]*\].*/\1/p'))
    if [ "$(count 'login:')" -ge 2 ]; then
        echo "  PASS: second boot after system_reset reaches the login prompt"
    else
        echo "  FAIL: second boot after system_reset reaches the login prompt"
        fail=1
    fi
    if [ ${#MOUNT[@]} -ge 2 ] &&
       [ "${MOUNT[1]}" -le $((MOUNT[0] + 10)) ]; then
        echo "  PASS: second boot mounts the overlay as soon as the first" \
             "(${MOUNT[0]} s, ${MOUNT[1]} s)"
    else
        echo "  FAIL: second boot mounts the overlay as soon as the first" \
             "(${MOUNT[*]:-no mount} s)"
        fail=1
    fi
    if grep -aq 'detected stall' "$LOG"; then
        echo "  FAIL: no RCU stall"
        fail=1
    else
        echo "  PASS: no RCU stall"
    fi
fi
if python3 - "$IMG" "$MODE" "$DATA" <<'EOF'
import sys, zlib
d = open(sys.argv[1], 'rb').read()
ok = any(d[i:i + 2] == b'\x85\x19' for i in range(int(sys.argv[3]), len(d), 4))
print('  %s: JFFS2 overlay written back to rootfs_data'
      % ('PASS' if ok else 'FAIL'))
if sys.argv[2] != 'kernel':
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
