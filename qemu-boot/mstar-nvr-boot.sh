#!/bin/sh
# SPDX-License-Identifier: MIT
#
# Boot a Xiongmai MStar (SigmaStar infinity2m / ssr621q) NVR flash dump to its
# running XM NVR services (DVRIP, ONVIF, the mobile daemon) in the qemu-bench
# twin.
#
# Built for OpenIPC/majestic#379, where the reporter's recorder is an MStar
# NBD80S16S (V4.03.R11.7601): the camera pushes a wsnt:Notify that the NVR
# ignores. To watch the NVR's ONVIF client against a majestic camera twin we
# first want the recorder booting with a console.
#
# The recorder already boots as shipped: `qemu-boot/run-ssr621q.sh FLASH=dump`
# runs its own IPL and vendor U-Boot to the full XM NVR userspace (DVRIP, ONVIF,
# the mobile daemon; eth0 at the vendor static 192.168.1.9). What that path does
# NOT give is a kernel console: a Xiongmai NVR prints on UART0 only with xmuart=0
# in the U-Boot environment, which means editing the image's env (sstar-env.py).
#
# This script is the alternative when the image must stay byte-for-byte as
# shipped: it boots the vendor uImage directly with the exact bootargs U-Boot
# would have passed, recovered READ-ONLY from the dump's own U-Boot default
# environment -- never stopping, editing or reconfiguring U-Boot. The NOR is
# still attached via flash-file, so the kernel's MTD, the squashfs partitions and
# the XM services' flash reads all work, and the full stack comes up with a live
# serial console. No device-model change is involved; this is an invocation
# recipe for observability, not a silicon fix.
#
#   bash qemu-boot/mstar-nvr-boot.sh --flash ~/dumps/nbd80s16s.bin
#   bash qemu-boot/mstar-nvr-boot.sh --flash ~/dumps/nbd80s16s.bin --check
#
# --check boots headless, waits for the NVR services to come up (the mobile
# daemon's "Running" banner and a listening DVRIP port), prints PASS/FAIL and
# exits -- a self-test for the recipe. Without it the guest's serial console is
# left on stdio so you can watch and interact.
#
# The dump is never written through: the script works on a copy. The firmware
# is the user's; nothing from it is committed.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$HERE")

QEMU=${QEMU:-"$REPO/qemu-src/build/qemu-system-arm"}
MACHINE=${MACHINE:-ssr621q}
FLASH=
MEM=${MEM:-256M}
WORK=${WORK:-}
CHECK=0
CHECK_TIMEOUT=${CHECK_TIMEOUT:-300}

while [ $# -gt 0 ]; do
    case $1 in
        --flash)        FLASH=$2; shift 2 ;;
        --machine)      MACHINE=$2; shift 2 ;;
        --qemu)         QEMU=$2; shift 2 ;;
        --mem)          MEM=$2; shift 2 ;;
        --work)         WORK=$2; shift 2 ;;
        --check)        CHECK=1; shift ;;
        --check-timeout) CHECK_TIMEOUT=$2; shift 2 ;;
        -h|--help)
            sed -n '2,40p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

[ -n "$FLASH" ] || { echo "need --flash FILE (an MStar NVR NOR dump)" >&2; exit 2; }
[ -r "$FLASH" ] || { echo "cannot read flash dump: $FLASH" >&2; exit 2; }
[ -x "$QEMU" ]  || { echo "qemu not built at $QEMU (run: bash qemu/setup.sh)" >&2; exit 2; }
command -v unsquashfs >/dev/null 2>&1 || {
    echo "need unsquashfs (squashfs-tools) to extract the vendor uImage" >&2; exit 2; }

if [ -z "$WORK" ]; then
    WORK=$(mktemp -d "${TMPDIR:-/tmp}/mstar-nvr.XXXXXX")
    trap 'rm -rf "$WORK"' EXIT
fi
mkdir -p "$WORK"

# Work on a copy so the dump is never written through (U-Boot would rewrite a
# blank env, and the NVR services write their config back to the NOR).
cp -f "$FLASH" "$WORK/flash.bin"

# Recover the romfs offset and the exact (expanded) kernel command line from the
# dump itself: locate the root squashfs, find and XZ-decompress the U-Boot image
# carried in the boot partition, read its default-env `bootargs` and expand the
# ${mmasize}/${heapsize...} MMA-heap variables from the same environment. This
# is read-only introspection of the image; it does not run or stop U-Boot.
META=$(python3 - "$WORK/flash.bin" <<'PY'
import lzma, re, struct, sys
d = open(sys.argv[1], 'rb').read()

# Root filesystem: the first squashfs after the boot area. Its byte offset is
# also the romfs partition start the uImage is read from.
sq = d.find(b'hsqs', 0x40000)
if sq < 0:
    sys.exit("no root squashfs (hsqs magic) found in dump")

# U-Boot: an XZ stream in the boot partition [0x10000, romfs). Decompress it and
# read the compiled-in default environment.
ub = None
for m in re.finditer(b'\xfd7zXZ\x00', d):
    if 0x10000 <= m.start() < sq:
        try:
            ub = lzma.LZMADecompressor(format=lzma.FORMAT_XZ).decompress(d[m.start():sq])
            break
        except Exception:
            ub = None
if ub is None:
    sys.exit("no XZ U-Boot image found in boot partition")

def env(k):
    # U-Boot default env is a run of key=val\0 pairs; match a whole entry. The
    # name carries '=', so e.g. `setenv bootargs ${bootargs}` elsewhere in the
    # env (no '=') is not matched.
    m = re.search((re.escape(k) + r'=([ -~]*?)\x00').encode(), ub)
    return m.group(1).decode() if m else None

args = env('bootargs')
if not args:
    sys.exit("no bootargs in U-Boot default environment")
for _ in range(5):
    todo = set(re.findall(r'\$\{(\w+)\}', args))
    if not todo:
        break
    for v in todo:
        args = args.replace('${%s}' % v, env(v) or '')

print("ROMFS_OFF=0x%x" % sq)
print("BOOTARGS=%s" % args)
PY
)
ROMFS_OFF=$(printf '%s\n' "$META" | sed -n 's/^ROMFS_OFF=//p')
BOOTARGS=$(printf '%s\n' "$META" | sed -n 's/^BOOTARGS=//p')
[ -n "$ROMFS_OFF" ] && [ -n "$BOOTARGS" ] || { echo "failed to parse dump" >&2; exit 1; }

# Carve the root squashfs and pull the kernel image out of it. dd takes no hex,
# so resolve the offsets to decimal first.
ROMFS_SZ=${ROMFS_SZ:-0x400000}
romfs_off_dec=$(( ROMFS_OFF ))
romfs_sz_dec=$(( ROMFS_SZ ))
dd if="$WORK/flash.bin" of="$WORK/romfs.sqfs" bs=4096 \
   iflag=skip_bytes,count_bytes skip="$romfs_off_dec" count="$romfs_sz_dec" status=none
rm -rf "$WORK/romfs"
unsquashfs -d "$WORK/romfs" -q "$WORK/romfs.sqfs" boot/uImage >/dev/null 2>&1 || true
UIMAGE="$WORK/romfs/boot/uImage"
[ -f "$UIMAGE" ] || { echo "no boot/uImage in root squashfs at $ROMFS_OFF" >&2; exit 1; }

echo "machine  : $MACHINE"
echo "flash    : $FLASH ($(stat -c%s "$FLASH" 2>/dev/null || echo '?') bytes, booting a copy)"
echo "uImage   : romfs@$ROMFS_OFF -> $(stat -c%s "$UIMAGE") bytes"
echo "bootargs : $BOOTARGS"
echo

if [ "$CHECK" = 1 ]; then
    # Headless self-test: boot, watch the console for the NVR services, PASS when
    # the mobile daemon reports running and DVRIP is up; FAIL on a panic/oops or
    # when the deadline passes.
    SER="$WORK/serial.log"
    : > "$SER"
    "$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin" -m "$MEM" -nographic \
        -kernel "$UIMAGE" -append "$BOOTARGS" \
        -icount shift=0 -serial "file:$SER" -monitor none \
        -nic user,restrict=on >"$WORK/qemu.log" 2>&1 &
    QPID=$!
    # shellcheck disable=SC2064
    trap "kill $QPID 2>/dev/null || true; rm -rf \"$WORK\"" EXIT
    rc=2
    end=$(( $(date +%s) + CHECK_TIMEOUT ))
    while [ "$(date +%s)" -lt "$end" ]; do
        if ! kill -0 "$QPID" 2>/dev/null; then
            echo "FAIL: qemu exited early"; rc=1; break
        fi
        if grep -qE 'Kernel panic|Unable to handle kernel|Oops' "$SER" 2>/dev/null; then
            echo "FAIL: kernel crash"; rc=1; break
        fi
        if grep -qE 'Mobile Device is Running' "$SER" 2>/dev/null; then
            echo "PASS: NVR userspace up (XM services + mobile daemon running)"
            grep -aoE 'LibXmDvr: Compile date: [^,]*' "$SER" | head -1 || true
            rc=0; break
        fi
        sleep 3
    done
    [ "$rc" = 2 ] && echo "FAIL: services did not come up within ${CHECK_TIMEOUT}s"
    kill "$QPID" 2>/dev/null || true
    exit "$rc"
fi

# Interactive: console on stdio, SLIRP with DVRIP (34567) and ONVIF/HTTP (80)
# forwarded to localhost so a host-side probe can reach the recorder. The vendor
# firmware sets a static 192.168.1.9 on eth0, so for a two-guest ONVIF lab put
# the bridge on 192.168.1.0/24 (see docs/sigmastar.md).
exec "$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin" -m "$MEM" -nographic \
    -kernel "$UIMAGE" -append "$BOOTARGS" \
    -icount shift=0 -serial mon:stdio \
    -nic user,restrict=on,hostfwd=tcp:127.0.0.1:34567-:34567,hostfwd=tcp:127.0.0.1:8080-:80 \
    "$@"
