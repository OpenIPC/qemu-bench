#!/bin/bash
#
# Regression test for OpenIPC/firmware#2613: init must never erase an overlay
# it cannot mount when that overlay holds the camera's settings.
#
# The failure, measured on an XM510:
#   1. The camera moved from a kernel built with CONFIG_JFFS2_FS_XATTR to one
#      without it.
#   2. jffs2 refused the old overlay as a whole: an xattr node is
#      JFFS2_FEATURE_INCOMPAT.
#   3. init took the refusal for a foreign partition and ran flash_eraseall,
#      taking every setting and the root password.
#
# Reproduced here without a camera:
#   * rootfs_data is a jffs2 image built on the host with --with-xattr. It
#     holds an xattr node and etc/majestic.yaml, the setting that marks an
#     overlay as OpenIPC's own.
#   * The guest boots from flash with a kernel that has no jffs2 xattr
#     support, which is what OpenIPC's xm510 kernel is.
#   * Once the guest has reached login, the host compares rootfs_data
#     byte-for-byte with what it wrote. An init with the fix leaves it
#     untouched and runs on tmpfs; one without it erases it.
#
# Asserted from outside the guest, so no login and no claim (see
# test-flash-rw.sh for why that matters).
#
# Usage:
#   test-overlay-keep.sh --soc xm510 --machine xm510 --mtd-name xm_sfc \
#       --flash-mb 8 --mem 32M --qemu ./qemu-src/build/qemu-system-arm \
#       --output-dir <dir> [--kernel <uImage>] [--rootfs <squashfs>] \
#       [--timeout 180]
#
set -u

SOC=""
MACHINE=""
QEMU="./qemu-src/build/qemu-system-arm"
OUTDIR="."
MEM="128M"
TIMEOUT=180
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
command -v setfattr >/dev/null || { echo "=== FAIL: needs setfattr (attr) ==="; exit 1; }
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

# The overlay a camera that was set up under an xattr-capable kernel has: a
# saved majestic setting, carrying an xattr as the overlayfs upper dir does.
DATA_DIR="$OUTDIR/overlay-src"
rm -rf "$DATA_DIR"; mkdir -p "$DATA_DIR/etc"
echo "# saved by the camera's owner" > "$DATA_DIR/etc/majestic.yaml"
if ! setfattr -n user.openipc -v owner "$DATA_DIR/etc/majestic.yaml" 2>/dev/null; then
    echo "=== FAIL: $OUTDIR does not support user xattrs; use another --output-dir ==="
    exit 1
fi
DATA_IMG="$OUTDIR/rootfs_data.jffs2"
mkfs.jffs2 -l -e 0x10000 -p $((DATA_KB * 1024)) --with-xattr \
    -d "$DATA_DIR" -o "$DATA_IMG" || { echo "=== FAIL: mkfs.jffs2 ==="; exit 1; }
python3 - "$DATA_IMG" <<'PY' || { echo "=== FAIL: the overlay image has no xattr node to refuse ==="; exit 1; }
import struct, sys
d = open(sys.argv[1], 'rb').read()
types = {struct.unpack('<H', d[i + 2:i + 4])[0]
         for i in range(0, len(d) - 4, 4) if d[i:i + 2] == b'\x85\x19'}
sys.exit(0 if 0xe008 in types else 1)    # JFFS2_NODETYPE_XATTR
PY

FLASH="$OUTDIR/flash.bin"
head -c $((FLASH_MB * 1024 * 1024)) /dev/zero | tr '\000' '\377' > "$FLASH"
dd if="$ROOTFS" of="$FLASH" bs=1k seek=$((BOOT_KB + ENV_KB + KERN_KB)) conv=notrunc 2>/dev/null
dd if="$DATA_IMG" of="$FLASH" bs=1k seek="$DATA_OFF_KB" conv=notrunc 2>/dev/null
MTDPARTS="${MTD_NAME}:${BOOT_KB}k(boot),${ENV_KB}k(env),${KERN_KB}k(kernel)"
MTDPARTS="${MTDPARTS},${ROOTFS_KB}k(rootfs),-(rootfs_data)"
APPEND="${CMDLINE_PREFIX:+$CMDLINE_PREFIX }console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs"
APPEND="${APPEND} init=/init mtdparts=${MTDPARTS}"

CONSOLE="$OUTDIR/overlay-keep-console.txt"
rm -f "$CONSOLE"
"$QEMU" \
    -M "$MACHINE",flash-file="$FLASH" -m "$MEM" \
    -kernel "$KERNEL" \
    -nographic -serial file:"$CONSOLE" -monitor none -no-reboot \
    -nic user,restrict=on \
    -append "$APPEND" &
QEMU_PID=$!
cleanup() { kill "$QEMU_PID" 2>/dev/null; wait "$QEMU_PID" 2>/dev/null; }
trap cleanup EXIT

echo "=== waiting for login (<= ${TIMEOUT}s) ==="
login=0
for _ in $(seq 1 "$TIMEOUT"); do
    if grep -aq 'login:' "$CONSOLE" 2>/dev/null; then login=1; break; fi
    kill -0 "$QEMU_PID" 2>/dev/null || break
    sleep 1
done
cleanup
trap - EXIT

fail=0
[ "$login" = "1" ] || { echo "FAIL: no login prompt"; fail=1; }

# The kernel must really have refused it, or this test proves nothing.  Its
# own "Incompatible feature node (0xe008)" is below console loglevel; what
# reaches the console is busybox mount's failure.
refused=0
if tr -d '\r' < "$CONSOLE" | grep -aqE 'mounting /dev/mtdblock[0-9]+ on /overlay failed'; then
    echo "ok   the kernel refused the overlay (no jffs2 xattr support)"
    refused=1
else
    # A kernel with jffs2 xattrs mounts the overlay and writes to it, as it
    # should: nothing here can then say whether init would have kept it.
    echo "FAIL: the kernel mounted the xattr overlay, so the scenario was not reproduced;" \
         "pass a kernel built without CONFIG_JFFS2_FS_XATTR (--kernel)"
    fail=1
fi

if [ "$refused" = "0" ]; then
    :
elif python3 - "$FLASH" "$DATA_IMG" "$((DATA_OFF_KB * 1024))" <<'PY'
import sys
fw = open(sys.argv[1], 'rb').read()
img = open(sys.argv[2], 'rb').read()
off = int(sys.argv[3])
sys.exit(0 if fw[off:off + len(img)] == img else 1)
PY
then
    echo "ok   rootfs_data is byte-for-byte what the camera had"
else
    echo "FAIL: rootfs_data was rewritten: init erased an overlay holding settings (#2613)"
    fail=1
fi

if [ "$fail" = "0" ]; then
    echo "=== PASS: an unmountable overlay holding settings survives boot ==="
    exit 0
fi
echo "--- overlay / jffs2 lines ---"
tr -d '\r' < "$CONSOLE" | grep -naiE 'jffs2|overlay|rootfs_data|erase' | head -20
exit 1
