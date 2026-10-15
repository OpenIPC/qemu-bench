#!/bin/bash
#
# Test that V1 DDR only works once a reg_info table has brought it up.
#
# Usage:  bash qemu/tests/test-hisi-v1-ddr-init.sh
#
# The V1 U-Boot (OpenIPC/u-boot-hi3516cv100) runs in place from SPI NOR
# and walks the register table at image offset 0x40 before it copies
# itself to DDR at 0x80800000.  That table programs the PLLs, the DDR
# controller and the DDR PHY.  The u-boot-hi3516cv100 / hi3518[ace]v100
# release assets were published with that region all zero, and flashing
# them left boards with no UART output at all (OpenIPC/firmware#2593),
# while this emulator booted them, because it copied U-Boot to DDR
# itself and DDR always worked.
#
# The flash image is generated here, no cross toolchain needed: a vector
# branch, the table at 0x40, and at 0x200 a walker for the two record
# kinds the DDR sequence uses (write: attr bit 2; poll until a field
# equals the value: attr bit 18, field at attr[31:27], width
# attr[23:19] + 1).  After the table it writes a magic word to 0x80800000,
# reads it back, and prints DDR-OK or DDR-DEAD on UART0.  Source (GNU as,
# -march=armv5te), assembled at 0x200:
#
#   reset: adr r0, table
#   walk:  ldr r1,[r0]; ldr r2,[r0,#4]; ldr r3,[r0,#12]
#          cmp r1,#0; beq done
#          tst r3,#4; strne r2,[r1]; bne next
#          tst r3,#0x40000; beq next
#          mov r4,r3,lsr #27; mov r5,r3,lsr #19; and r5,r5,#0x1f
#          add r5,r5,#1; mov r6,#1; mov r6,r6,lsl r5; sub r6,r6,#1
#   1:     ldr r7,[r1]; mov r7,r7,lsr r4; and r7,r7,r6; cmp r7,r2; bne 1b
#   next:  add r0,r0,#16; b walk
#   done:  ldr r1,=0x80800000; ldr r2,=0x600dd00d; str r2,[r1]; ldr r3,[r1]
#          ldr r8,=0x20080000; ldr r9,=0x301; str r9,[r8,#0x30]
#          cmp r3,r2; adreq r9,ok; adrne r9,dead
#   print: ldrb r10,[r9],#1; cmp r10,#0; beq hang
#   2:     ldr r11,[r8,#0x18]; tst r11,#0x20; bne 2b
#          str r10,[r8]; b print
#   hang:  b hang
#   ok:    .asciz "DDR-OK\r\n"
#   dead:  .asciz "DDR-DEAD\r\n"
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

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# One 8 MiB NOR image per case.  The DDR records are the sequence every
# V1 table we have shares (the OpenIPC cv100 one and the vendor DS-I203
# one from OpenIPC/u-boot-hi3516cv100#6), attr words included.
python3 - "$TMP" <<'PY'
import struct, sys
out = sys.argv[1]
code = [
    0xe24f0f72, 0xe5901000, 0xe5902004, 0xe590300c, 0xe3510000, 0x0a000012,
    0xe3130004, 0x15812000, 0x1a00000d, 0xe3130701, 0x0a00000b, 0xe1a04da3,
    0xe1a059a3, 0xe205501f, 0xe2855001, 0xe3a06001, 0xe1a06516, 0xe2466001,
    0xe5917000, 0xe1a07437, 0xe0077006, 0xe1570002, 0x1afffffa, 0xe2800010,
    0xeaffffe7, 0xe59f1058, 0xe59f2058, 0xe5812000, 0xe5913000, 0xe59f8050,
    0xe59f9050, 0xe5889030, 0xe1530002, 0x028f9024, 0x128f9029, 0xe4d9a001,
    0xe35a0000, 0x0a000004, 0xe598b018, 0xe31b0020, 0x1afffffc, 0xe588a000,
    0xeafffff7, 0xeafffffe, 0x2d524444, 0x0a0d4b4f, 0x52444400, 0x4145442d,
    0x000a0d44, 0x80800000, 0x600dd00d, 0x20080000, 0x00000301,
]
pll = [
    (0x20030010, 0x11000000, 0, 0xfd),      # BPLL post dividers
    (0x20030014, 0x0068306e, 0, 0xfd),      # BPLL refdiv/fbdiv
    (0x200300e8, 0xf, 0, 0x001d0000),       # poll: all four PLLs locked
]
ddrc = [
    (0x2011001c, 0x80000600, 0, 0xfd),
    (0x2011002c, 0x00000132, 0, 0xfd),
    (0x20110004, 0x00000000, 0, 0xfd),
    (0x20110000, 0x0, 0, 0x10050000),       # poll: DDRC bit 2 == 0
]
phy_reset = [
    (0x20120404, 0x80000000, 0, 0xfd),
    (0x2012040c, 0x1, 0, 0x00050000),       # poll: PHY bit 0 == 1
]
phy_init = [
    (0x20120404, 0x000000ff, 0, 0xfd),      # bit 0: DRAM init + training
    (0x2012040c, 0x1, 0, 0x00050000),
]
cases = {
    'table': pll + ddrc + phy_reset + phy_init,
    'empty': [],
    'noinit': pll + ddrc + phy_reset,
}
for name, recs in cases.items():
    img = bytearray(b'\xff' * (8 << 20))
    img[0:4] = struct.pack('<I', 0xea00007e)            # b reset (0x200)
    for i in range(1, 16):
        img[i * 4:i * 4 + 4] = struct.pack('<I', 0xeafffffe)
    img[0x40:0x200] = bytes(0x1c0)
    for i, r in enumerate(recs):
        img[0x40 + i * 16:0x50 + i * 16] = struct.pack('<4I', *r)
    blob = struct.pack('<%dI' % len(code), *code)
    img[0x200:0x200 + len(blob)] = blob
    open('%s/%s.bin' % (out, name), 'wb').write(img)
PY

PASS=0
FAIL=0

check() {
    local desc="$1" expected="$2" actual="$3"
    if [ "$actual" = "$expected" ]; then
        echo "  PASS: $desc ($actual)"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $desc (expected $expected, got '$actual')"
        FAIL=$((FAIL + 1))
    fi
}

# run <machine> <case> [extra args...] -> UART text on stdout.  The guest
# ends in a loop, so timeout's 124 is the only good exit; anything else
# (QEMU refused the machine, crashed) is reported instead of the UART, and
# the UART file is removed first so a run that never opened it cannot
# hand back the previous case's output.
run() {
    local machine="$1" case="$2" rc=0
    shift 2
    rm -f "$TMP/uart"
    : > "$TMP/log"
    timeout 5 "$QEMU" -M "$machine,flash-file=$TMP/$case.bin" "$@" \
        -nodefaults -display none -monitor none \
        -serial "file:$TMP/uart" -d guest_errors -D "$TMP/log" \
        >"$TMP/qemu.out" 2>&1 || rc=$?
    if [ "$rc" -ne 124 ]; then
        echo "qemu exited $rc: $(head -c 200 "$TMP/qemu.out")"
        return
    fi
    tr -d '\r\n' < "$TMP/uart"
}

untrained_logged() {
    grep -q 'hisi-ddrc-v1: DDR write at +0x800000 before the DDR PHY' \
        "$TMP/log" && echo yes || echo no
}

for m in hi3516cv100 hi3518av100 hi3518cv100 hi3518ev100; do
    echo "== $m"
    check "table brings DDR up" "DDR-OK" "$(run $m table)"
    check "empty table: DDR stays dead" "DDR-DEAD" "$(run $m empty)"
    check "empty table: DDR access logged" "yes" "$(untrained_logged)"
    check "PHY never started: DDR stays dead" "DDR-DEAD" "$(run $m noinit)"
done

echo ""
echo "$PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
