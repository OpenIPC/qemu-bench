#!/usr/bin/env python3
"""The ISP statistics hisi-vi writes at the end of each frame, read the
way the HiMPP V4 ISP driver reads them (ISP_DRV_FE_STT_StatisticsRead,
ISP_DRV_BE_STT_StatisticsRead, ISP_DRV_BE_APB_StatisticsRead), and the
front end's interrupt its per-frame work runs from (ISP_ISR).

No sensor here, so the scene's white is half scale (65535 / 2): eight grey
bars under a light of R 0.55, G 1, B 0.7, the brightest bar white.

    python3 qemu/tests/test-hisi-isp.py [--qemu PATH] [--machine M]
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
FE_STT, BE_STT = 0x42900000, 0x42940000     # as the driver's buffers
FRAME_NS = 40 * 1000 * 1000                 # hisi-vi's 25 fps
WHITE = 65535 / 2
R, G, B = int(WHITE * 0.55), int(WHITE), int(WHITE * 0.7)
SAMPLES = (1920 // 4) * (1080 // 4)         # every 4th row and column


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine', default='hi3516ev200,sensor=none')
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    def words(addr, n):
        return struct.unpack('<%dI' % n, q.read(addr, n * 4))

    q = QTest(a.qemu, a.machine)
    # what the driver sets up: the STT buffers, the zone grids, the front
    # end's interrupt mask (ISP_DRV_SetIntEnable)
    q.writel(VI_CAP + 0x10208, FE_STT)              # FE AE histogram
    q.writel(VI_CAP + 0x22810, 0xf11)               # FE AE zones 17 x 15
    q.writel(VI_PROC + 0x668, BE_STT)               # BE AE histogram
    q.writel(VI_PROC + 0x670, BE_STT + 0x1000)      # AE R << 16 | Gr
    q.writel(VI_PROC + 0x678, BE_STT + 0x1400)      # AE Gb << 16 | B
    q.writel(VI_PROC + 0x680, BE_STT + 0x2800)      # AWB zones
    q.writel(VI_PROC + 0x22010, 0xf11)
    q.writel(VI_PROC + 0x22110, 0x2020)             # AWB zones 32 x 32
    q.writel(VI_CAP + 0x200f0, 0xff)
    q.writel(VI_CAP + 0x200f8, 0x11)                # FSTART | FEND
    # the online pipeline runs: the write channel, VPSS's online video
    q.writel(VI_CAP + 0x10000, 1 << 31)
    q.writel(VPSS + 0x000, 1 << 31)
    q.writel(VPSS + 0x0f0, 0x7401)
    q.cmd('clock_step %d' % (FRAME_NS + FRAME_NS // 2))

    st = q.readl(VI_CAP + 0x200f0)
    check(st == 0x11, 'front end: frame start and end (0x%x)' % st)
    q.writel(VI_CAP + 0x200f0, 0xff)
    st = q.readl(VI_CAP + 0x200f0)
    check(st == 0, 'front end: write 1s to clear (0x%x)' % st)
    q.writel(VI_CAP + 0x10f0, 0xf)
    st = q.readl(VI_CAP + 0x10f0)
    check(st == 0, 'port status: no errors after a clear (0x%x)' % st)

    fe = words(FE_STT, 256)
    check(sum(fe) == SAMPLES // 8 * 8 * 4 and fe[G >> 8] == SAMPLES // 8 * 2,
          'front end histogram: %d samples, white G in bin %d' %
          (sum(fe), G >> 8))
    be = words(BE_STT, 1024)
    check(sum(be) == SAMPLES // 8 * 8 * 4 and be[G >> 6] == SAMPLES // 8 * 2
          and be[R >> 6] == SAMPLES // 8,
          'backend histogram: white R / G in bins %d / %d' % (R >> 6, G >> 6))

    rgr, gbb = words(BE_STT + 0x1000, 1)[0], words(BE_STT + 0x1400, 1)[0]
    check(rgr == (R << 16 | G) and gbb == (G << 16 | B),
          'AE zone 0 (the white bar): R %d Gr %d Gb %d B %d' %
          (rgr >> 16, rgr & 0xffff, gbb >> 16, gbb & 0xffff))
    last = words(BE_STT + 0x1000 + (17 * 15 - 1) * 4, 1)[0]
    check(last >> 16 < R // 10,
          'AE zone 254 (the black bar): R %d' % (last >> 16))

    w0, w1 = words(BE_STT + 0x2800, 2)
    n = SAMPLES // 1024
    check(w0 == (G << 16 | R) and w1 == (n << 16 | B),
          'AWB zone 0: R %d G %d B %d count %d' %
          (w0 & 0xffff, w0 >> 16, w1 & 0xffff, w1 >> 16))

    r, g, b, cnt = (q.readl(VI_PROC + 0x22140 + 4 * i) for i in range(4))
    check(0 < r < g and 0 < b < g and cnt == min(SAMPLES, 0xffff),
          'AWB global: R %d G %d B %d count %d' % (r, g, b, cnt))
    tot, aver_r = q.readl(VI_PROC + 0x22018), q.readl(VI_PROC + 0x22020)
    check(tot == SAMPLES * 4 and aver_r == r,
          'AE global: %d pixels, R mean %d' % (tot, aver_r))
    q.close()

    print('%d failure(s)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
