#!/bin/bash
#
# SDHCI write-protect polarity on the HiSilicon/Goke machines (qemu-bench#209).
#
# Usage:  bash qemu/tests/test-hisi-sdhci-wp.sh
#
# With a writable card in slot 0, reads PRNSTS (+0x24) and checks bit 19,
# the write-protect switch level, against what the machine's kernel driver
# needs in order to see the card as writable:
#
#   - sdhci-hisi.c (hi3516ev200 kernel), sdhci-goke.c (gk7205v200),
#     sdhci-xmedia-common.c (gk7205v500) and sdhci-lotus-common.c (gk7206v100)
#     set SDHCI_QUIRK_INVERTED_WRITE_PROTECT, so a writable card must read 0.
#     QEMU's default of 1 left every card on these machines mounted read-only.
#   - nebula (hi3516cv6xx, hi3519dv500 kernels) uses the spec polarity: 1.
#
# A read-only card cannot be tested: QEMU refuses a read-only drive as an SD
# card ("Cannot use read-only drive as SD card").
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
BUILD="$REPO_ROOT/qemu-src/build"

for b in qemu-system-arm qemu-system-aarch64; do
    if [ ! -x "$BUILD/$b" ]; then
        echo "FAIL: $BUILD/$b not found"
        echo "      Run 'bash qemu/setup.sh' first."
        exit 1
    fi
done

CARD=$(mktemp)
trap 'rm -f "$CARD"' EXIT
truncate -s 64M "$CARD"     # QEMU's SD model wants a power-of-two image

PASS=0
FAIL=0

# check <binary> <machine> <sdhci base> <expected bit 19> <driver>
check() {
    local bin="$1" machine="$2" base="$3" want="$4" driver="$5"
    local prnsts bit

    prnsts=$(printf 'xp/1xw 0x%x\nquit\n' $((base + 0x24)) \
        | timeout 15 "$BUILD/$bin" -M "$machine" -nodefaults -nographic -S \
            -drive if=sd,file="$CARD",format=raw \
            -chardev stdio,id=mon,mux=off -mon chardev=mon,mode=readline \
            2>/dev/null \
        | tr -d '\r' | grep -oP '[0-9a-f]+:\s+\K0x[0-9a-fA-F]+' | head -1) || true

    if [ -z "$prnsts" ]; then
        echo "  FAIL: $machine ($driver): PRNSTS unreadable"
        FAIL=$((FAIL + 1))
        return
    fi
    bit=$(( (prnsts >> 19) & 1 ))
    if [ "$bit" = "$want" ]; then
        echo "  PASS: $machine ($driver): PRNSTS $prnsts, WP bit $bit"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $machine ($driver): PRNSTS $prnsts, WP bit $bit, want $want"
        FAIL=$((FAIL + 1))
    fi
}

echo "--- inverted (writable card reads WP bit 0) ---"
check qemu-system-arm     hi3516ev300 0x10010000 0 sdhci-hisi
check qemu-system-arm     hi3516ev200 0x10010000 0 sdhci-hisi
check qemu-system-arm     hi3518ev300 0x10010000 0 sdhci-hisi
check qemu-system-arm     hi3516dv200 0x10010000 0 sdhci-hisi
check qemu-system-arm     gk7205v200  0x10010000 0 sdhci-goke
check qemu-system-arm     gk7205v500  0x10010000 0 sdhci-xmedia-common
check qemu-system-arm     gk7202v330  0x10010000 0 sdhci-xmedia-common
check qemu-system-arm     gk7206v10   0x10010000 0 sdhci-lotus-common
echo "--- spec polarity (writable card reads WP bit 1) ---"
check qemu-system-arm     hi3516cv610 0x10030000 1 nebula
check qemu-system-aarch64 hi3519dv500 0x10030000 1 nebula

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" = 0 ]
