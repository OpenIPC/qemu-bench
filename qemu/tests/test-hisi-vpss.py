#!/usr/bin/env python3
"""Drive hisi-vi and hisi-vpss through VI offline (a sensor wider than
VPSS's 2304-pixel line buffer) the way the HiMPP V4 driver does, as
traced from majestic with an IMX335 on hi3516ev300.

Per frame the driver arms VPSS (0x304 bit 0, the first node at 0x2f8) and
starts VI's pipe (VI_PROC 0x300).  The nodes are VPSS register images in
DDR, chained through their own 0x2f8, one per vertical strip; the model
paints the strips' widths together as one picture.  The pipe's end comes
on VI_PROC 0x310 a quarter frame later.

    python3 qemu/tests/test-hisi-vpss.py [--qemu PATH] [--machine M]
"""
import argparse
import os
import struct
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
QTest = import_module('test-xm-h1-jpeg').QTest

VI_CAP, VI_PROC, VPSS = 0x11000000, 0x11200000, 0x11400000
WCH_CTRL = VI_CAP + 0x10000
NODES = 0x42200000
Y, C = 0x42400000, 0x42c00000
STRIPS = (1536, 1056)           # IMX335's: 2592 wide together
W, H = sum(STRIPS), 240
FRAME_NS = 40 * 1000 * 1000     # hisi-vi's 25 fps


def node(w, nxt, cmp):
    """One strip's VPSS register image: channel 0 only."""
    img = bytearray(0x800)

    def put(off, v):
        struct.pack_into('<I', img, off, v)
    put(0x2f8, nxt)
    put(0x44c, (w - 1) | (H - 1) << 16)
    put(0x450, 1 << 31)
    put(0x468 + 0x10, Y)
    put(0x468 + 0x18, C)
    put(0x488, W | W << 16)
    put(0x700, 1 if cmp else 0)
    return bytes(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine', default='hi3516ev300')
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    q = QTest(a.qemu, a.machine)

    def offline_frame(cmp):
        q.write(Y, b'\0' * (W * H))
        q.write(NODES, node(STRIPS[0], NODES + 0x800, cmp))
        q.write(NODES + 0x800, node(STRIPS[1], 0, cmp))
        q.writel(VI_PROC + 0x310, 0xffffffff)
        q.writel(VPSS + 0x304, 0x0900647f)
        q.writel(VPSS + 0x2f8, NODES)
        q.readl(VPSS + 0x2f8)
        q.writel(VI_PROC + 0x2f8, 0x44455000)
        q.writel(VI_PROC + 0x0f0, 6)
        q.writel(VI_PROC + 0x300, 1)
        q.cmd('clock_step %d' % (FRAME_NS // 2))

    q.writel(WCH_CTRL, 1 << 31)                     # VI_HAL_SetWchEn

    for cmp in (False, True):
        what = 'compressed' if cmp else 'linear'
        offline_frame(cmp)
        st = q.readl(VI_PROC + 0x310)
        check(st & 0x402 == 0x402,
              '%s: the pipe reports start and end (0x%x)' % (what, st))
        # the stand-in of a compressed channel: every other row, the
        # width as the stride
        row = (H // 2) // (2 if cmp else 1)
        line = q.read(Y + row * W, W)
        check(all(line[x] for x in (8, STRIPS[0] - 8, STRIPS[0] + 8, W - 8)),
              '%s: both strips painted, %d wide' % (what, W))

    # the pipe stopped within the quarter frame: no late end for it
    q.writel(VI_PROC + 0x310, 0xffffffff)
    q.writel(VPSS + 0x304, 0x0900647f)
    q.writel(VPSS + 0x2f8, NODES)
    q.writel(VI_PROC + 0x300, 1)
    q.writel(WCH_CTRL, 0)
    q.cmd('clock_step %d' % FRAME_NS)
    st = q.readl(VI_PROC + 0x310)
    check(st == 0, 'stopped pipe: no late interrupt (0x%x)' % st)

    # rebuilt online: VPSS's online video and frame interrupt, no 0x300;
    # every frame raises the pipe's start and end again
    q.writel(VI_PROC + 0x310, 0xffffffff)
    q.writel(WCH_CTRL, 1 << 31)
    q.writel(VPSS + 0x000, 1 << 31)
    q.writel(VPSS + 0x0f0, 0x7401)
    q.cmd('clock_step %d' % (FRAME_NS * 2))
    st = q.readl(VI_PROC + 0x310)
    check(st & 0x402 == 0x402,
          'online after offline: the pipe follows capture (0x%x)' % st)
    q.close()

    print('%d failure(s)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
