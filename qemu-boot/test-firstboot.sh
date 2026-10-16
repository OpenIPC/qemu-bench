#!/bin/bash
#
# End-to-end `firstboot` (sysupgrade -n) on emulated flash.
#
# rootfs_data starts as a mountable jffs2 overlay holding a saved setting,
# etc/majestic.yaml, written on the host.  The firmware's unattended SD hook
# (autostart.sh, run as root by lib/mdev/automount.sh) runs `firstboot`, which
# wipes the overlay and reboots; -no-reboot turns that into QEMU exiting.  The
# host then checks:
#
#   1. the first boot really mounted the planted overlay -- the setting was
#      visible in /etc, or the wipe below proves nothing
#   2. after firstboot, rootfs_data no longer holds the setting's name
#   3. a second boot from that flash reaches login
#
# No login and no claim, as in test-flash-rw.sh.  Needs sudo for one
# loop-mount (the FAT SD card).
#
# Usage:
#   test-firstboot.sh --soc xm510 --machine xm510 --mtd-name xm_sfc \
#       --flash-mb 8 --mem 32M --qemu ./qemu-src/build/qemu-system-arm \
#       --output-dir <dir> [--kernel <uImage>] [--rootfs <squashfs>] \
#       [--timeout 240]
#
set -u

SOC=""
MACHINE=""
QEMU="./qemu-src/build/qemu-system-arm"
OUTDIR="."
MEM="128M"
TIMEOUT=240
KERNEL=""
ROOTFS=""
MTD_NAME="hi_sfc"
CMDLINE_PREFIX=""
FLASH_MB=16

while [ $# -gt 0 ]; do
    case "$1" in
        --soc)        SOC="$2"; shift 2 ;;
        --machine)    MACHINE="$2"; shift 2 ;;
        --qemu)       QEMU="$2"; shift 2 ;;
        --output-dir) OUTDIR="$2"; shift 2 ;;
        --mem)        MEM="$2"; shift 2 ;;
        --timeout)    TIMEOUT="$2"; shift 2 ;;
        --kernel)     KERNEL="$2"; shift 2 ;;
        --rootfs)     ROOTFS="$2"; shift 2 ;;
        --mtd-name)   MTD_NAME="$2"; shift 2 ;;
        --cmdline-prefix) CMDLINE_PREFIX="$2"; shift 2 ;;
        --flash-mb)   FLASH_MB="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

[ -n "$SOC" ] && [ -n "$MACHINE" ] || { echo "--soc and --machine required" >&2; exit 2; }
command -v mkfs.jffs2 >/dev/null || { echo "=== FAIL: needs mkfs.jffs2 (mtd-utils) ==="; exit 1; }
mkdir -p "$OUTDIR"

: "${KERNEL:=qemu-boot/uImage.${SOC}}"
: "${ROOTFS:=qemu-boot/rootfs.squashfs.${SOC}}"
for f in "$KERNEL" "$ROOTFS"; do
    [ -f "$f" ] || { echo "=== FAIL: missing $f ==="; exit 1; }
done

BOOT_KB=256; ENV_KB=64; KERN_KB=2048
ROOTFS_KB=$(( ( ($(stat -c %s "$ROOTFS") + 65535) / 65536 ) * 64 ))
DATA_OFF_KB=$((BOOT_KB + ENV_KB + KERN_KB + ROOTFS_KB))
DATA_KB=$((FLASH_MB * 1024 - DATA_OFF_KB))
[ "$DATA_KB" -ge 256 ] || { echo "=== FAIL: no room for rootfs_data in ${FLASH_MB} MiB ==="; exit 1; }

# A unique marker in the setting's contents, so the host can tell the planted
# file from anything init or the hook write later.
MARK="firstboot-test-$(head -c 6 /dev/urandom | od -An -tx1 | tr -d ' \n')"
DATA_DIR="$OUTDIR/overlay-src"
rm -rf "$DATA_DIR"; mkdir -p "$DATA_DIR/etc"
echo "# $MARK" > "$DATA_DIR/etc/majestic.yaml"
DATA_IMG="$OUTDIR/rootfs_data.jffs2"
mkfs.jffs2 -l -e 0x10000 -p $((DATA_KB * 1024)) -d "$DATA_DIR" -o "$DATA_IMG" ||
    { echo "=== FAIL: mkfs.jffs2 ==="; exit 1; }

FLASH="$OUTDIR/flash.bin"
head -c $((FLASH_MB * 1024 * 1024)) /dev/zero | tr '\000' '\377' > "$FLASH"
dd if="$ROOTFS" of="$FLASH" bs=1k seek=$((BOOT_KB + ENV_KB + KERN_KB)) conv=notrunc 2>/dev/null
dd if="$DATA_IMG" of="$FLASH" bs=1k seek="$DATA_OFF_KB" conv=notrunc 2>/dev/null
MTDPARTS="${MTD_NAME}:${BOOT_KB}k(boot),${ENV_KB}k(env),${KERN_KB}k(kernel)"
MTDPARTS="${MTDPARTS},${ROOTFS_KB}k(rootfs),-(rootfs_data)"
APPEND="${CMDLINE_PREFIX:+$CMDLINE_PREFIX }console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs"
APPEND="${APPEND} init=/init mtdparts=${MTDPARTS}"

# Detached and off the card: firstboot is sysupgrade -n, which unmounts /mnt
# before its RAM pivot and refuses while this script holds the card.
cat > "$OUTDIR/autostart.sh" <<'AUTOSTART'
#!/bin/sh
# Once per boot, and straight to the console.  The 3.10 XM530 kernel can
# deliver the card's add event twice, so automount may run this twice; and
# automount's logger pipe dies with everything else in sysupgrade's pivot.
[ -e /tmp/.firstboot-ran ] && exit 0
touch /tmp/.firstboot-ran
exec >/dev/console 2>&1
# Only the planted setting triggers the wipe: after it, /etc/majestic.yaml is
# the rootfs's stock file, and boot 2 must not wipe and reboot again.
if ! grep -q firstboot-test- /etc/majestic.yaml 2>/dev/null; then
	echo "FB_SEEN_SETTING=0"
	exit 0
fi
echo "FB_SEEN_SETTING=1"
cd / && ( sleep 3; firstboot; echo "FB_RC=$?" ) </dev/null &
AUTOSTART

SD="$OUTDIR/sd.img"
rm -f "$SD"; truncate -s 64M "$SD"
sfdisk -q "$SD" <<'PARTS'
label: dos
start=2048, type=b
PARTS
LOOP=$(sudo losetup --find --show --partscan "$SD") || {
    echo "=== FAIL: could not attach $SD to a loop device ==="; exit 1; }
MNT="$OUTDIR/sdmnt"; mkdir -p "$MNT"
sudo mkfs.vfat -F 16 -n SDTEST "${LOOP}p1" >/dev/null
sudo mount "${LOOP}p1" "$MNT"
sudo cp "$OUTDIR/autostart.sh" "$MNT/"
sudo sync; sudo umount "$MNT"; sudo losetup -d "$LOOP"

run_guest() {   # $1 console file, $2 seconds, [$3 stop on this string]
    "$QEMU" -M "$MACHINE",flash-file="$FLASH" -m "$MEM" -kernel "$KERNEL" \
        -drive file="$SD",if=sd,format=raw \
        -nographic -serial file:"$1" -monitor none -no-reboot \
        -nic user,restrict=on -append "$APPEND" &
    local pid=$! t=0
    while kill -0 "$pid" 2>/dev/null && [ "$t" -lt "$2" ]; do
        [ -n "${3:-}" ] && grep -aq "$3" "$1" 2>/dev/null && break
        sleep 1; t=$((t + 1))
    done
    kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
    [ "$t" -lt "$2" ]
}

C1="$OUTDIR/firstboot-console.txt"; rm -f "$C1"
echo "=== boot 1: firstboot from the SD hook (<= ${TIMEOUT}s) ==="
run_guest "$C1" "$TIMEOUT"; exited=$?

fail=0
seen=$(tr -d '\r' < "$C1" | sed -n 's/.*FB_SEEN_SETTING=\([0-9]\+\).*/\1/p' | head -1)
if [ "$seen" = "1" ]; then
    echo "ok   the planted overlay was mounted (the setting was in /etc)"
else
    echo "FAIL: the planted setting was not visible in /etc; nothing to wipe"; fail=1
fi
rc=$(tr -d '\r' < "$C1" | sed -n 's/.*FB_RC=\([0-9]\+\).*/\1/p' | head -1)
if [ -n "$rc" ]; then
    echo "FAIL: firstboot returned $rc without rebooting"; fail=1
elif [ "$exited" != "0" ]; then
    echo "FAIL: the guest neither rebooted nor finished within ${TIMEOUT}s"; fail=1
else
    echo "ok   firstboot rebooted the guest"
fi

if python3 - "$FLASH" "$((DATA_OFF_KB * 1024))" "$MARK" <<'PY'
import sys
fw = open(sys.argv[1], 'rb').read()
sys.exit(1 if sys.argv[3].encode() in fw[int(sys.argv[2]):] else 0)
PY
then
    echo "ok   rootfs_data no longer holds the setting"
else
    echo "FAIL: the setting is still in rootfs_data after firstboot"; fail=1
fi

C2="$OUTDIR/second-boot-console.txt"; rm -f "$C2"
echo "=== boot 2: after the wipe ==="
if run_guest "$C2" 180 'login:' && grep -aq 'login:' "$C2"; then
    echo "ok   the wiped camera boots to login"
else
    echo "FAIL: the wiped camera does not reach login"; fail=1
fi
if grep -aq 'FB_SEEN_SETTING=1' "$C2"; then
    echo "FAIL: boot 2 still saw the planted setting in /etc"; fail=1
fi

if [ "$fail" = "0" ]; then
    echo "=== PASS: firstboot wiped the overlay and the camera boots ==="
    exit 0
fi
echo "--- firstboot lines ---"
tr -d '\r' < "$C1" | grep -naiE 'firstboot|sysupgrade|overlay|erase|FB_' | tail -30
exit 1
