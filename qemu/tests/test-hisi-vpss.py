#!/usr/bin/env python3
"""Drive hisi-vi and hisi-vpss through VI offline (a sensor wider than
VPSS's 2304-pixel line buffer) the way the HiMPP V4 driver does, as
traced from majestic with an IMX335 on hi3516ev300.

Per frame the driver arms VPSS (0x304 bit 0, the first node at 0x2f8) and
starts VI's pipe (VI_PROC 0x300).  The nodes are VPSS register images in
DDR, chained through their own 0x2f8, one per vertical strip; the model
paints the strips' widths together as one picture.  The pipe's end comes
on VI_PROC 0x310 a quarter frame later.

--gen 5 drives the V5 (Hi3516CV610) online pipeline instead, as
mod_vi.o / mod_vpss.o program it: the CRG's reset handshake for VPSS, VI's
top-level status derived from the write channel and the ISP front end,
VI_PROC's start and end behind its mask, and VPSS painting through the
V5 channel registers.

    python3 qemu/tests/test-hisi-vpss.py [--qemu PATH] [--machine M] [--gen 4|5]
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


def gen5(q, check):
    """V5 online: VI capture -> VI_PROC -> VPSS channel 0."""
    CRG = 0x11010000
    CAP, PROC, VP = 0x17400000, 0x17800000, 0x17900000
    W5, H5 = 640, 360

    # sys_hal_vpss_reset_sel: the status follows the request at once
    q.writel(CRG + 0x9b40, 1)
    on = q.readl(CRG + 0x9b44) & 1
    q.writel(CRG + 0x9b40, 0)
    check(on == 1 and q.readl(CRG + 0x9b44) & 1 == 0,
          'CRG: VPSS reset status follows its request')

    # what vi_drv_enable_pipe_interrupt and vpss_online_set_mask_and_enable
    # program: masks, channel 0, online video
    q.writel(CAP + 0xf8, 0x300 | 1 << 16)
    q.writel(CAP + 0x10000 + 0xf8, 0x200001)
    q.writel(CAP + 0x20000 + 0xf8, 0x128)
    q.writel(PROC + 0xf0, 0x0c00)
    q.writel(CAP + 0x10000, 1 << 31)                # write channel 0 on
    q.write(Y, b'\0' * (W5 * H5))
    q.writel(VP + 0x428, (W5 - 1) | (H5 - 1) << 16)
    q.writel(VP + 0x42c, 1 << 31)
    q.writel(VP + 0x444 + 8, Y)                     # header Y, C, then Y, C
    q.writel(VP + 0x444 + 12, C)
    q.writel(VP + 0x454, W5 | W5 << 16)
    q.writel(VP + 0xf0, 0x7ed9)
    q.writel(VP + 0x000, 1 << 31)
    q.writel(VP + 0x010, 1)                         # REGUP
    q.cmd('clock_step %d' % (FRAME_NS * 2))

    top, wch = q.readl(CAP + 0xf4), q.readl(CAP + 0x10000 + 0xf0)
    fe = q.readl(CAP + 0x20000 + 0xf0)
    check(top & 0x10100 == 0x10100 and q.readl(CAP + 0xe4) == top,
          'VI: top status has the write channel and the FE (0x%x)' % top)
    check(wch & 1 and fe & 0x101 == 0x101,
          'VI: write channel frame start (0x%x), FE start / end (0x%x)'
          % (wch, fe))
    raw, msk = q.readl(PROC + 0x310), q.readl(PROC + 0x314)
    check(raw & 0x401 == 0x401 and msk == raw & 0xc00,
          'VI_PROC: start and end (0x%x), masked 0x%x' % (raw, msk))
    vraw = q.readl(VP + 0x310)
    check(vraw & 0x401 == 0x401, 'VPSS: start and end of frame (0x%x)' % vraw)
    line = q.read(Y + (H5 // 2) * W5, W5)
    check(any(line), 'VPSS: channel 0 painted through the V5 registers')

    q.writel(CAP + 0x10000 + 0xf0, 0xffffffff)
    q.writel(CAP + 0x20000 + 0xf0, 0xffffffff)
    check(q.readl(CAP + 0xf4) == 0,
          'VI: top status clears with the blocks (0x%x)' % q.readl(CAP + 0xf4))
    q.writel(PROC + 0x310, 0xffffffff & ~0x10000)
    check(q.readl(PROC + 0x314) == 0, 'VI_PROC: cleared')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine')
    ap.add_argument('--gen', type=int, default=4, choices=(4, 5))
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    if a.gen == 5:
        q = QTest(a.qemu, a.machine or 'hi3516cv610')
        gen5(q, check)
        q.close()
        print('%d failure(s)' % fails)
        return 1 if fails else 0
    q = QTest(a.qemu, a.machine or 'hi3516ev300')

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
