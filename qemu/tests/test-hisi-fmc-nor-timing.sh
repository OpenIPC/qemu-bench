#!/bin/bash
#
# Test the opt-in SPI-NOR erase/program timing in hisi-fmc.
#
# Usage:  bash qemu/tests/test-hisi-fmc-nor-timing.sh
#
# By default every NOR erase and program completes at once. With
# hisi-fmc.erase-64k-us / page-program-us set, SR1 reports WIP until the
# operation would have finished on the chip, so a guest flashing 7 MB waits
# the minute it waits on silicon -- long enough for a process paging from the
# flash being rewritten to fault, which is how fixed-slot to split sysupgrade
# migrations bricked cameras in the field. This drives the controller through
# qtest: a 64 KiB sector erase, then the status op before and after the erase
# time has passed on the virtual clock, with the timing off and on.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

if [ ! -x "$QEMU" ]; then
    echo "FAIL: QEMU binary not found at $QEMU"
    echo "      Run 'bash qemu/setup.sh' first."
    exit 1
fi

IMG="$(mktemp /tmp/hisi-fmc-nor.XXXXXX.img)"
trap 'rm -f "$IMG"' EXIT
head -c 16777216 /dev/zero > "$IMG"

# hi3516ev300: FMC at 0x10000000; a 16 MiB flash-file is SPI-NOR.
# WREN, then sector erase (D8h) at 0x10000: FMC_OP CMD1_EN|ADDR_EN|START.
# Status op: FMC_OP READ_STATUS_EN|START latches SR1 into FMC_STATUS (0xac).
# Read it at once, then 100 ms and 200 ms of virtual time later.
probe() {
    printf '%s\n' \
        "writel 0x10000024 0x06" "writel 0x1000003c 0x81" \
        "writel 0x10000024 0xd8" "writel 0x1000002c 0x10000" \
        "writel 0x1000003c 0xc1" \
        "writel 0x1000003c 0x3" "readl 0x100000ac" \
        "clock_step 100000000" \
        "writel 0x1000003c 0x3" "readl 0x100000ac" \
        "clock_step 100000000" \
        "writel 0x1000003c 0x3" "readl 0x100000ac" |
        timeout 5 "$QEMU" -M hi3516ev300,sensor=none,flash-file="$IMG" \
            -accel qtest -qtest stdio -display none -serial null \
            -monitor none "$@" 2>/dev/null |
        grep -E '^OK 0x' | awk '{ printf "%d ", strtonum($2) % 2 }' || true
}

fail=0
check() {
    if [ "$2" = "$3" ]; then echo "  PASS: $1"
    else echo "  FAIL: $1 (WIP sequence '$2', want '$3')"; fail=1; fi
}

check "timing off: erase done at once" "$(probe)" "0 0 0 "
check "erase-64k-us=150000: WIP at 0 and 100 ms, clear at 200 ms" \
    "$(probe -global hisi-fmc.erase-64k-us=150000)" "1 1 0 "

echo "Result: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
