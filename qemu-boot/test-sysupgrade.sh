#!/bin/bash
#
# End-to-end sysupgrade from a local archive, on emulated flash.
#
# Boots the OpenIPC image from a synthetic NOR and lets the firmware's own
# unattended hook -- autostart.sh on the SD card, run as root by
# lib/mdev/automount.sh -- run
#
#     sysupgrade --archive=/tmp/<archive>.tgz -z
#
# (copied off the card first: sysupgrade unmounts /mnt before its pivot, and
# refuses to while the hook that started it still holds the card)
#
# sysupgrade then validates the archive, pivots into RAM, flashcp's kernel and
# rootfs, and reboots; -no-reboot turns that reboot into QEMU exiting.  The
# host then checks what the guest cannot check for itself:
#
#   1. the kernel and rootfs partitions hold the archive's images byte for byte
#   2. a saved setting planted in rootfs_data survives the upgrade (no -n)
#   3. a second boot from that flash reaches login
#
# The archive's rootfs must differ from the one on flash, or (1) proves
# nothing -- and sysupgrade skips a rootfs whose GITHUB_VERSION matches the
# running one ("Same version, nothing to update").  Without --new-rootfs the
# script makes one: the same rootfs with its version stamp changed.  The kernel partition starts
# erased, since -kernel is what boots.  No login and no claim, as in
# test-flash-rw.sh.  Needs sudo for one loop-mount (the FAT SD card).
#
# Usage:
#   test-sysupgrade.sh --soc xm510 --machine xm510 --mtd-name xm_sfc \
#       --flash-mb 8 --mem 32M [--new-rootfs <squashfs>] \
#       --qemu ./qemu-src/build/qemu-system-arm --output-dir <dir> \
#       [--kernel <uImage>] [--rootfs <squashfs>] [--timeout 300]
#
set -u

SOC=""
MACHINE=""
QEMU="./qemu-src/build/qemu-system-arm"
OUTDIR="."
MEM="128M"
TIMEOUT=300
KERNEL=""
ROOTFS=""
NEW_ROOTFS=""
MTD_NAME="hi_sfc"
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
        --new-rootfs) NEW_ROOTFS="$2"; shift 2 ;;
        --mtd-name)   MTD_NAME="$2"; shift 2 ;;
        --flash-mb)   FLASH_MB="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

[ -n "$SOC" ] && [ -n "$MACHINE" ] || { echo "--soc and --machine required" >&2; exit 2; }
command -v mkfs.jffs2 >/dev/null || { echo "=== FAIL: needs mkfs.jffs2 (mtd-utils) ==="; exit 1; }
mkdir -p "$OUTDIR"

: "${KERNEL:=qemu-boot/uImage.${SOC}}"
: "${ROOTFS:=qemu-boot/rootfs.squashfs.${SOC}}"
[ -f "$ROOTFS" ] || { echo "=== FAIL: missing $ROOTFS ==="; exit 1; }
if [ -z "$NEW_ROOTFS" ]; then
    command -v mksquashfs >/dev/null ||
        { echo "=== FAIL: needs squashfs-tools to restamp the rootfs ==="; exit 1; }
    TREE="$OUTDIR/new-rootfs-tree"; rm -rf "$TREE"
    unsquashfs -q -d "$TREE" "$ROOTFS" >/dev/null ||
        { echo "=== FAIL: unsquashfs $ROOTFS ==="; exit 1; }
    REL=$(sed -n 's/^GITHUB_VERSION="\{0,1\}\([^"]*\)"\{0,1\}$/\1/p' "$TREE/usr/lib/os-release")
    sed -i "s/^GITHUB_VERSION=.*/GITHUB_VERSION=\"${REL}+sysupgrade-test\"/" "$TREE/usr/lib/os-release"
    COMP=$(unsquashfs -s "$ROOTFS" | sed -n 's/^Compression //p')
    BLK=$(unsquashfs -s "$ROOTFS" | sed -n 's/^Block size //p')
    NEW_ROOTFS="$OUTDIR/rootfs.new.${SOC}"; rm -f "$NEW_ROOTFS"
    mksquashfs "$TREE" "$NEW_ROOTFS" -comp "${COMP:-xz}" -b "${BLK:-131072}" \
        -all-root -noappend -quiet >/dev/null ||
        { echo "=== FAIL: mksquashfs ==="; exit 1; }
    rm -rf "$TREE"
fi
for f in "$KERNEL" "$ROOTFS" "$NEW_ROOTFS"; do
    [ -f "$f" ] || { echo "=== FAIL: missing $f ==="; exit 1; }
done
cmp -s "$ROOTFS" "$NEW_ROOTFS" &&
    { echo "=== FAIL: --new-rootfs is the rootfs already on flash ==="; exit 1; }

kb() { echo $(( ( ($(stat -c %s "$1") + 65535) / 65536 ) * 64 )); }
BOOT_KB=256; ENV_KB=64; KERN_KB=2048
ROOTFS_KB=$(kb "$ROOTFS")
[ "$(kb "$NEW_ROOTFS")" -gt "$ROOTFS_KB" ] && ROOTFS_KB=$(kb "$NEW_ROOTFS")
KERN_OFF=$(( (BOOT_KB + ENV_KB) * 1024 ))
ROOTFS_OFF=$(( (BOOT_KB + ENV_KB + KERN_KB) * 1024 ))
DATA_OFF=$(( ROOTFS_OFF + ROOTFS_KB * 1024 ))
FLASH_BYTES=$(( FLASH_MB * 1024 * 1024 ))
[ $(( FLASH_BYTES - DATA_OFF )) -ge $((256 * 1024)) ] ||
    { echo "=== FAIL: no room for rootfs_data in ${FLASH_MB} MiB ==="; exit 1; }

# The camera's settings: a mountable overlay with a uniquely marked
# majestic.yaml, which the upgrade must leave in place.
MARK="sysupgrade-test-$(head -c 6 /dev/urandom | od -An -tx1 | tr -d ' \n')"
DATA_DIR="$OUTDIR/overlay-src"
rm -rf "$DATA_DIR"; mkdir -p "$DATA_DIR/etc"
echo "# $MARK" > "$DATA_DIR/etc/majestic.yaml"
DATA_IMG="$OUTDIR/rootfs_data.jffs2"
mkfs.jffs2 -l -e 0x10000 -p $(( FLASH_BYTES - DATA_OFF )) -d "$DATA_DIR" -o "$DATA_IMG" ||
    { echo "=== FAIL: mkfs.jffs2 ==="; exit 1; }

FLASH="$OUTDIR/flash.bin"
head -c "$FLASH_BYTES" /dev/zero | tr '\000' '\377' > "$FLASH"
dd if="$ROOTFS" of="$FLASH" bs=1k seek=$((ROOTFS_OFF / 1024)) conv=notrunc 2>/dev/null
dd if="$DATA_IMG" of="$FLASH" bs=1k seek=$((DATA_OFF / 1024)) conv=notrunc 2>/dev/null
MTDPARTS="${MTD_NAME}:${BOOT_KB}k(boot),${ENV_KB}k(env),${KERN_KB}k(kernel)"
MTDPARTS="${MTDPARTS},${ROOTFS_KB}k(rootfs),-(rootfs_data)"
APPEND="console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs"
APPEND="${APPEND} init=/init mtdparts=${MTDPARTS}"

# The archive sysupgrade is pointed at: the release bundle's shape, with the
# md5sums it checks.
ARCH_DIR="$OUTDIR/archive"
rm -rf "$ARCH_DIR"; mkdir -p "$ARCH_DIR"
cp "$KERNEL" "$ARCH_DIR/uImage.${SOC}"
cp "$NEW_ROOTFS" "$ARCH_DIR/rootfs.squashfs.${SOC}"
( cd "$ARCH_DIR" &&
  md5sum "uImage.${SOC}" > "uImage.${SOC}.md5sum" &&
  md5sum "rootfs.squashfs.${SOC}" > "rootfs.squashfs.${SOC}.md5sum" )
ARCHIVE="openipc.${SOC}-nor-lite.tgz"
tar czf "$OUTDIR/$ARCHIVE" -C "$ARCH_DIR" .

cat > "$OUTDIR/autostart.sh" <<AUTOSTART
#!/bin/sh
# Once only: the upgrade reboots, and the second boot mounts this card again.
[ -e /overlay/.sysupgrade-test-ran ] && exit 0
touch /overlay/.sysupgrade-test-ran
echo "SU_SEEN_SETTING=\$(grep -c sysupgrade-test- /etc/majestic.yaml 2>/dev/null)"
echo "SU_START_MARK"
# sysupgrade unmounts /mnt before its RAM pivot and refuses while this very
# script holds the card open, so run it from /tmp, detached, once we are gone.
cp /mnt/mmcblk0p1/${ARCHIVE} /tmp/ && cd / &&
	( sleep 3; sysupgrade --archive=/tmp/${ARCHIVE} -z; echo "SU_RC=\$?" ) \
		</dev/null >/dev/console 2>&1 &
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
sudo cp "$OUTDIR/autostart.sh" "$OUTDIR/$ARCHIVE" "$MNT/"
sudo sync; sudo umount "$MNT"; sudo losetup -d "$LOOP"

run_guest() {   # $1 console file; returns when QEMU exits or $2 s pass
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

C1="$OUTDIR/sysupgrade-console.txt"; rm -f "$C1"
echo "=== boot 1: sysupgrade from the SD card (<= ${TIMEOUT}s) ==="
cp "$FLASH" "$OUTDIR/flash.before"
run_guest "$C1" "$TIMEOUT"; exited=$?

fail=0
grep -aq SU_START_MARK "$C1" || { echo "FAIL: the SD hook never ran sysupgrade"; fail=1; }
if grep -aq 'SU_SEEN_SETTING=1' "$C1"; then
    echo "ok   the planted overlay was mounted (the setting was in /etc)"
else
    echo "FAIL: the planted setting was not visible in /etc; nothing to preserve"; fail=1
fi
rc=$(tr -d '\r' < "$C1" | sed -n 's/.*SU_RC=\([0-9]\+\).*/\1/p' | head -1)
if [ -n "$rc" ]; then
    echo "FAIL: sysupgrade returned $rc without rebooting"; fail=1
elif [ "$exited" != "0" ]; then
    echo "FAIL: the guest neither rebooted nor finished within ${TIMEOUT}s"; fail=1
else
    echo "ok   sysupgrade rebooted the guest"
fi

python3 - "$FLASH" "$OUTDIR/flash.before" "$KERNEL" "$NEW_ROOTFS" \
    "$KERN_OFF" "$ROOTFS_OFF" "$DATA_OFF" "$MARK" <<'PY' || fail=1
import sys
fw, before = (open(p, 'rb').read() for p in sys.argv[1:3])
kern, rootfs = (open(p, 'rb').read() for p in sys.argv[3:5])
koff, roff, doff = map(int, sys.argv[5:8])
ok = True
for name, off, img in (('kernel', koff, kern), ('rootfs', roff, rootfs)):
    if fw[off:off + len(img)] == img:
        print(f'ok   {name} partition holds the archive\'s image')
    else:
        print(f'FAIL: {name} partition does not hold the archive\'s image'); ok = False
# The hook itself writes to the overlay, so compare the setting, not bytes.
if sys.argv[8].encode() in fw[doff:]:
    print('ok   the saved setting is still in rootfs_data')
else:
    print('FAIL: the upgrade lost the saved setting in rootfs_data'); ok = False
sys.exit(0 if ok else 1)
PY

C2="$OUTDIR/second-boot-console.txt"; rm -f "$C2"
echo "=== boot 2: the upgraded flash ==="
if run_guest "$C2" 180 'login:' && grep -aq 'login:' "$C2"; then
    echo "ok   the upgraded flash boots to login"
else
    echo "FAIL: the upgraded flash does not reach login"; fail=1
fi

if [ "$fail" = "0" ]; then
    echo "=== PASS: sysupgrade --archive wrote kernel + rootfs and the result boots ==="
    exit 0
fi
echo "--- sysupgrade lines ---"
tr -d '\r' < "$C1" | grep -naiE 'sysupgrade|flash|error|fail|SU_' | tail -30
exit 1
