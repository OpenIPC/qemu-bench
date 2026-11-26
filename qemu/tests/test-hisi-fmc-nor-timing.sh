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
# qtest and reads the status latch against the virtual clock after each kind
# of operation, with the timing off and on.
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

# hi3516ev300: FMC at 0x10000000, DDR at 0x40000000; a 16 MiB flash-file is
# SPI-NOR. FMC_OP (0x3c): CMD1_EN 0x80 | ADDR_EN 0x40 | WRITE_DATA_EN 0x20 |
# START 0x1; READ_STATUS_EN|START (0x3) latches SR1 into FMC_STATUS (0xac).
WREN='writel 0x10000024 0x06
writel 0x1000003c 0x81'
STATUS='writel 0x1000003c 0x3
readl 0x100000ac'
erase64k() { printf '%s\n' "$WREN" "writel 0x10000024 0xd8" \
    "writel 0x1000002c $1" "writel 0x1000003c 0xc1"; }
# Register page program: 256 bytes from the IO buffer.
regprog() { printf '%s\n' "$WREN" "writel 0x10000024 0x02" \
    "writel 0x1000002c $1" "writel 0x10000038 0x100" "writel 0x1000003c 0xe1"; }
# DMA program of $2 bytes of 0x00 from DDR: FMC_CFG normal mode, then
# FMC_OP_CTRL DMA_OP_READY | RW_OP (write).
dmaprog() { printf '%s\n' "memset 0x41000000 $2 0x00" "writel 0x10000000 0x1" \
    "writel 0x1000002c $1" "writel 0x1000004c 0x41000000" \
    "writel 0x10000040 $2" "writel 0x10000068 0x3"; }
step() { echo "clock_step $1"; }

# Run qtest commands from stdin; print each status read's WIP bit.
qtest() {
    head -c 16777216 /dev/zero > "$IMG"
    timeout 5 "$QEMU" -M hi3516ev300,sensor=none,flash-file="$IMG" \
        -accel qtest -qtest stdio -display none -serial null \
        -monitor none "$@" 2>/dev/null |
        grep -E '^OK 0x' |
        while read -r _ v; do printf '%d ' $(( v & 1 )); done
}
# The byte at $1 in the image the last qtest() left behind, as two hex digits.
byte() { od -An -tx1 -j "$(($1))" -N1 "$IMG" | tr -d ' \n'; }

E='-global hisi-fmc.erase-64k-us=150000'
P='-global hisi-fmc.page-program-us=700'

fail=0
check() {
    if [ "$2" = "$3" ]; then echo "  PASS: $1"
    else echo "  FAIL: $1 (got '$2', want '$3')"; fail=1; fi
}

check "timing off: erase done at once" \
    "$( { erase64k 0x10000; echo "$STATUS"; step 100000000; echo "$STATUS"; } | qtest)" "0 0 "
check "64 KiB erase: WIP at 0 and 100 ms, clear at 200 ms" \
    "$( { erase64k 0x10000; echo "$STATUS"; step 100000000; echo "$STATUS"
          step 100000000; echo "$STATUS"; } | qtest $E)" "1 1 0 "
check "register page program: WIP at 0, clear at 1 ms" \
    "$( { regprog 0x20000; echo "$STATUS"; step 1000000; echo "$STATUS"; } | qtest $P)" "1 0 "
check "4 KiB DMA program (16 pages): WIP at 10 ms, clear at 12 ms" \
    "$( { dmaprog 0x20000 4096; step 10000000; echo "$STATUS"; step 2000000
          echo "$STATUS"; } | qtest $P)" "1 0 "
check "chip erase: WIP at 30 s, clear at 40 s (256 x 150 ms)" \
    "$( { printf '%s\n' "$WREN" "writel 0x10000024 0xc7" "writel 0x1000003c 0x81"
          step 30000000000; echo "$STATUS"; step 10000000000; echo "$STATUS"; } |
        qtest $E)" "1 0 "

# A chip that is busy ignores a program, and the deadline does not move.
check "a DMA program sent mid-erase is dropped, the erase ends on time" \
    "$( { erase64k 0x10000; dmaprog 0x10000 256; step 160000000
          echo "$STATUS"; } | qtest $E $P)" "0 "
check "...and the erased sector stays erased" "$(byte 0x10000)" "ff"
check "timing off: the same program lands" \
    "$( { erase64k 0x10000; dmaprog 0x10000 256; } | qtest; byte 0x10000)" "00"

# FMC_CFG flash select = NAND (bits 2:1 = 1): the NOR's WIP is not NAND's.
check "status with the NAND selected ignores a busy NOR" \
    "$( { erase64k 0x10000; echo "writel 0x10000000 0x3"; echo "$STATUS"; } |
        qtest $E)" "0 "

echo "Result: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
