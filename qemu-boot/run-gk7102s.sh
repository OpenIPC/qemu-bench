#!/bin/bash
#
# Boot an emulated Goke GK7102S / GK7102C (the S die, 64 MiB in package)
# from a camera's SPI-NOR image, through its own U-Boot:
#
#   FLASH=dump.bin bash qemu-boot/run-gk7102s.sh
#
# The chip holds every 32-bit word byte-swapped (docs/goke-gk710x.md).  A
# programmer dump is in that order and is used as it is; erases and programs
# are written back to it, so boot a copy.  A dump read out by software (sf
# read, /dev/mtd) is in CPU order: it is converted, padded to a whole MiB,
# into $FLASH.chip, and that file is used instead.
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

if [ -z "$FLASH" ]; then
    echo "Set FLASH=<GK7102S/C flash image> (see the top of $0)" >&2
    exit 1
fi

if [ "$(head -c 4 "$FLASH")" = "17KG" ]; then
    python3 - "$FLASH" "$FLASH.chip" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read()
d += b'\xff' * (-len(d) % (1 << 20))
open(sys.argv[2], 'wb').write(
    b''.join(d[i:i + 4][::-1] for i in range(0, len(d), 4)))
PY
    echo "CPU-order dump: booting the converted $FLASH.chip"
    FLASH="$FLASH.chip"
fi

TAP="${TAP:-tap0}"
if ip link show "$TAP" &>/dev/null 2>&1; then
    NIC_ARGS="-nic tap,ifname=$TAP,script=no,downscript=no"
else
    NIC_ARGS="-nic user"
fi

exec "$QEMU" -M gk7102s,flash-file="$FLASH" \
    -nographic -serial mon:stdio \
    $NIC_ARGS \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-gk7102s.log" \
    "$@"
