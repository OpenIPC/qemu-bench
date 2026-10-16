#!/bin/bash
#
# SPI-NOR erase + program round-trip test (regression guard for the HiFMC
# erase-address / WEL fix that makes OpenIPC `firstboot` and `sysupgrade`
# work — see "hisi-fmc: fix NOR erase/program ...").
#
# Boots OpenIPC from a writable synthetic SPI-NOR attached via -machine
# flash-file=, then, in the guest:
#   1. flash_eraseall a multi-block partition  -> must read back 0xFF
#   2. flashcp -v a payload over it            -> must verify (exit 0)
#   3. the host compares the NOR image byte-for-byte with what it handed over
#
# The first two fail with the pre-fix model: the erase address was read from
# the stale FMC_ADDRL instead of the IO buffer, so every block after the first
# erased the wrong offset (partition stayed unerased; flashcp mismatched at the
# first un-erased block).  A multi-block payload is required to catch it — a
# single-block write happens to land on whatever addrl held.
#
# NO LOGIN, AND A STOCK IMAGE.  Root's login shell is `openipc-claim`, which
# hands out no shell until the camera has an owner, and on a majestic image
# claiming shows the Majestic EULA first — acceptance that belongs to the
# camera's human owner and never to CI (OpenIPC/firmware CLAUDE.md, "Policy,
# for AI assistants").  Rather than work around that with a substitute image,
# this does not log in at all: the commands are delivered the way the firmware
# itself supports unattended work.  `lib/mdev/automount.sh` runs `autostart.sh`
# from the SD card as root at mdev coldplug and pipes it through `logger -s`,
# which puts the results on the console we already capture.  So the image under
# test is the one OpenIPC ships, unmodified, and the run never reaches the
# claim prompt at all.
#
# Two consequences worth knowing:
#
#   * The guest must boot from flash.  `/init` only builds the writable overlay
#     when the root is flash — `root=ram|mmcblk|nfs` opts straight out — and
#     without it automount cannot even create its mountpoint
#     ("mkdir: can't create directory '/mnt/mmcblk0p1': Read-only file system"),
#     which is what a `-initrd` boot gives.  So the rootfs goes into the
#     synthetic NOR and the guest boots from it.  First boot then jffs2-formats
#     rootfs_data, exercising the erase path the way firstboot does before the
#     round-trip below even starts.
#
#   * The payload is generated on the host and carried in on the same SD card,
#     so afterwards the host can check the bytes that actually landed in the
#     NOR instead of trusting the guest's own exit codes.
#
# Needs sudo for one loop-mount, to put two files on a FAT image.
#
# Usage:
#   test-flash-rw.sh --soc hi3516ev300 --machine hi3516ev300,sensor=none \
#       --qemu ./qemu-src/build/qemu-system-arm --output-dir <dir> \
#       [--mem 128M] [--timeout 240] [--kernel <uImage>] [--rootfs <squashfs>]
#       [--mtd-name hi_sfc] [--flash-mb 16] [--cmdline-prefix "mem=32M"]
#
# --cmdline-prefix goes first on the kernel command line.  Xiongmai's
# load_xiongmai sizes MMZ from mem= as the FIRST argument, the way U-Boot
# passes it; without it MMZ is laid over the whole of RAM, kernel included,
# and majestic Oopses in mmz (all four flash scripts take it).
#
# --mtd-name is the controller name mtdparts= keys on (xm_sfc for Xiongmai),
# --flash-mb the synthetic NOR's size.  The XM510 kernel only knows 8 MiB
# chips, so there rootfs_data shrinks to whatever the 8 MiB leaves.
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
        --soc)           SOC="$2"; shift 2 ;;
        --machine)       MACHINE="$2"; shift 2 ;;
        --qemu)          QEMU="$2"; shift 2 ;;
        --output-dir)    OUTDIR="$2"; shift 2 ;;
        --mem)           MEM="$2"; shift 2 ;;
        --timeout|--login-timeout) TIMEOUT="$2"; shift 2 ;;
        --kernel)        KERNEL="$2"; shift 2 ;;
        --rootfs)        ROOTFS="$2"; shift 2 ;;
        --mtd-name)      MTD_NAME="$2"; shift 2 ;;
        --cmdline-prefix) CMDLINE_PREFIX="$2"; shift 2 ;;
        --flash-mb)      FLASH_MB="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

[ -n "$SOC" ] && [ -n "$MACHINE" ] || { echo "--soc and --machine required" >&2; exit 2; }
mkdir -p "$OUTDIR"

: "${KERNEL:=qemu-boot/uImage.${SOC}}"
: "${ROOTFS:=qemu-boot/rootfs.squashfs.${SOC}}"
for f in "$KERNEL" "$ROOTFS"; do
    [ -f "$f" ] || { echo "=== FAIL: missing $f ==="; exit 1; }
done

# Synthetic NOR (16 MiB by default): the rootfs the guest boots from, a 1 MiB scratch "test"
# partition that is NOT the rootfs so erasing and rewriting it is safe, and a
# rootfs_data for the overlay.  The kernel still arrives via -kernel; nothing
# writes mtd2.
BOOT_KB=256; ENV_KB=64; KERN_KB=2048; TEST_KB=1024; DATA_KB=2048
ROOTFS_KB=$(( ( ($(stat -c %s "$ROOTFS") + 65535) / 65536 ) * 64 ))
ROOTFS_OFF_KB=$((BOOT_KB + ENV_KB + KERN_KB))
TEST_OFF=$(( (ROOTFS_OFF_KB + ROOTFS_KB) * 1024 ))
TEST_MTD=4
LEFT_KB=$(( FLASH_MB * 1024 - (ROOTFS_OFF_KB + ROOTFS_KB + TEST_KB) ))
[ "$LEFT_KB" -ge 256 ] || {
    echo "=== FAIL: rootfs does not fit a ${FLASH_MB} MiB flash with room for an overlay ==="
    exit 1; }

FLASH="$OUTDIR/flash.bin"
dd if=/dev/zero of="$FLASH" bs=1M count="$FLASH_MB" 2>/dev/null
dd if="$ROOTFS" of="$FLASH" bs=1k seek="$ROOTFS_OFF_KB" conv=notrunc 2>/dev/null
MTDPARTS="${MTD_NAME}:${BOOT_KB}k(boot),${ENV_KB}k(env),${KERN_KB}k(kernel)"
MTDPARTS="${MTDPARTS},${ROOTFS_KB}k(rootfs),${TEST_KB}k(test)"
if [ "$LEFT_KB" -gt "$DATA_KB" ]; then
    MTDPARTS="${MTDPARTS},${DATA_KB}k(rootfs_data),-(spare)"
else
    MTDPARTS="${MTDPARTS},-(rootfs_data)"
fi
APPEND="${CMDLINE_PREFIX:+$CMDLINE_PREFIX }console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs"
APPEND="${APPEND} init=/init mtdparts=${MTDPARTS}"

# The SD card carries the firmware's unattended hook and the payload.  Results
# come back as NAME=<digit>, read with a digit-anchored sed so the script text
# echoed into the log cannot be mistaken for a result.
PAYLOAD="$OUTDIR/payload.bin"
head -c 524288 /dev/urandom > "$PAYLOAD"

cat > "$OUTDIR/autostart.sh" <<AUTOSTART
#!/bin/sh
# Once per boot, and straight to the console.  The 3.10 XM530 kernel can
# deliver the card's add event twice, so automount may run this twice; and
# automount's logger pipe dies with everything else in sysupgrade's pivot.
[ -e /tmp/.flash-rw-ran ] && exit 0
touch /tmp/.flash-rw-ran
exec >/dev/console 2>&1
M=/dev/mtd${TEST_MTD}
P=/mnt/mmcblk0p1/payload.bin
flash_eraseall \$M >/dev/null 2>&1; echo "RW_ERASE_RC=\$?"
xxd -l 16 \$M | grep -q "ffff ffff ffff ffff ffff ffff ffff ffff"; echo "RW_EFF_RC=\$?"
flashcp -v \$P \$M >/dev/null 2>&1; echo "RW_FLASHCP_RC=\$?"
echo "RW_DONE_MARK"
AUTOSTART

# QEMU's SD card model only accepts a power-of-two image.
SD="$OUTDIR/sd.img"
truncate -s 64M "$SD"
sfdisk -q "$SD" <<'PARTS'
label: dos
start=2048, type=b
PARTS
LOOP=$(sudo losetup --find --show --partscan "$SD") || {
    echo "=== FAIL: could not attach $SD to a loop device ==="; exit 1; }
MNT="$OUTDIR/sdmnt"; mkdir -p "$MNT"
sudo mkfs.vfat -F 16 -n SDTEST "${LOOP}p1" >/dev/null
sudo mount "${LOOP}p1" "$MNT"
sudo cp "$OUTDIR/autostart.sh" "$PAYLOAD" "$MNT/"
sudo sync
sudo umount "$MNT"
sudo losetup -d "$LOOP"

CONSOLE="$OUTDIR/flash-rw-console.txt"
rm -f "$CONSOLE"

# restrict=on: keep SLIRP DHCP reachable but block outbound (busybox ntpd,
# see openhisilicon#104) so init runs deterministically.
"$QEMU" \
    -M "$MACHINE",flash-file="$FLASH" -m "$MEM" \
    -kernel "$KERNEL" \
    -drive file="$SD",if=sd,format=raw \
    -nographic -serial file:"$CONSOLE" -monitor none -no-reboot \
    -nic user,restrict=on \
    -append "$APPEND" &
QEMU_PID=$!
cleanup() { kill "$QEMU_PID" 2>/dev/null; wait "$QEMU_PID" 2>/dev/null; }
trap cleanup EXIT

echo "=== waiting for the SD hook to report (<= ${TIMEOUT}s) ==="
done_ok=0
for _ in $(seq 1 "$TIMEOUT"); do
    if grep -aq 'RW_DONE_MARK' "$CONSOLE" 2>/dev/null; then done_ok=1; break; fi
    kill -0 "$QEMU_PID" 2>/dev/null || break
    sleep 1
done
cleanup
trap - EXIT

if [ "$done_ok" != "1" ]; then
    echo "=== FAIL: the SD hook never reported ==="
    echo "--- mount / hook lines ---"
    tr -d '\r' < "$CONSOLE" | grep -nE 'mmcblk|/mnt|autostart|Read-only' | head -10
    echo "--- last 30 console lines ---"
    tr -d '\r' < "$CONSOLE" | tail -30
    exit 1
fi

get_rc() { tr -d '\r' < "$CONSOLE" | sed -n "s/.*$1=\([0-9]\+\).*/\1/p" | head -1; }
erase_rc=$(get_rc RW_ERASE_RC)      # flash_eraseall exit status
eff_rc=$(get_rc RW_EFF_RC)          # 0 => test mtd reads all-0xFF after erase
flashcp_rc=$(get_rc RW_FLASHCP_RC)  # 0 => flashcp -v erase+write+verify OK

# The assertion the guest cannot get wrong on our behalf: what is in the NOR.
if python3 - "$FLASH" "$PAYLOAD" "$TEST_OFF" <<'PY'
import sys
fw = open(sys.argv[1], 'rb').read()
payload = open(sys.argv[2], 'rb').read()
off = int(sys.argv[3])
sys.exit(0 if fw[off:off + len(payload)] == payload else 1)
PY
then bytes_rc=0; else bytes_rc=1; fi

echo "=== results: ERASE_RC=$erase_rc EFF_RC=$eff_rc FLASHCP_RC=$flashcp_rc" \
     "HOST_VERIFY_RC=$bytes_rc ==="

fail=0
[ "$erase_rc" = "0" ]   || { echo "FAIL: flash_eraseall on the test mtd returned '$erase_rc'"; fail=1; }
[ "$eff_rc" = "0" ]     || { echo "FAIL: test mtd not 0xFF after erase (erase hit wrong address)"; fail=1; }
[ "$flashcp_rc" = "0" ] || { echo "FAIL: flashcp -v returned '$flashcp_rc' (verify mismatch)"; fail=1; }
[ "$bytes_rc" = "0" ]   || { echo "FAIL: NOR at $(printf 0x%x "$TEST_OFF") does not match the payload"; fail=1; }

if [ "$fail" = "0" ]; then
    echo "=== PASS: SPI-NOR erase + program round-trip OK ==="
    exit 0
fi
echo "--- last 40 console lines ---"; tr -d '\r' < "$CONSOLE" | tail -40
exit 1
