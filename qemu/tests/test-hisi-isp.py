#!/usr/bin/env python3
"""The ISP statistics hisi-vi writes at the end of each frame, read the
way the HiMPP V4 ISP driver reads them (ISP_DRV_FE_STT_StatisticsRead,
ISP_DRV_BE_STT_StatisticsRead, ISP_DRV_BE_APB_StatisticsRead), and the
front end's interrupt its per-frame work runs from (ISP_ISR).

No sensor here, so the scene's white is half scale (65535 / 2): eight grey
bars under a light of R 0.55, G 1, B 0.7, the brightest bar white.

The scene (hisi-scene) can be a picture and its light can change while
the machine runs (QMP qom-set /machine/scene ...).

Offline, the pipe's strips each write a block at the address in their
VIPROC register image (VI_HAL_SetIspSttAddr, 0x5d0).

    python3 qemu/tests/test-hisi-isp.py [--qemu PATH] [--machine M]
"""
import argparse
import json
import os
import socket
import struct
import sys
import tempfile
import time

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
    n = 1920 * 1080 // 1024                 # every pixel of the zone
    check(w0 == (G << 16 | R) and w1 == (n << 16 | B),
          'AWB zone 0: R %d G %d B %d count %d' %
          (w0 & 0xffff, w0 >> 16, w1 & 0xffff, w1 >> 16))

    r, g, b, cnt = (q.readl(VI_PROC + 0x22140 + 4 * i) for i in range(4))
    check(0 < r < g and 0 < b < g and cnt == n,
          'AWB global: R %d G %d B %d count %d' % (r, g, b, cnt))
    tot, aver_r = q.readl(VI_PROC + 0x22018), q.readl(VI_PROC + 0x22020)
    check(tot == SAMPLES * 4 and aver_r == r,
          'AE global: %d pixels, R mean %d' % (tot, aver_r))

    # offline (a sensor wider than 2304): the pipe runs two strips, VIPROC
    # register images chained through 0x2f8, each writing its statistics
    # block (S_ISP_STT_REGS_TYPE) to the address at its 0x5d0
    NODE0, NODE1 = 0x42a00000, 0x42a20000
    STT0, STT1 = 0x42b00000, 0x42b10000
    for node, nxt, stt in ((NODE0, NODE1, STT0), (NODE1, 0, STT1)):
        q.write(node, b'\0' * 0x800)
        q.writel(node + 0x2f8, nxt)
        q.writel(node + 0x5d0, stt)
    q.write(STT0, b'\0' * 0x4000)
    q.write(STT1, b'\0' * 0x4000)
    q.writel(VI_PROC + 0x2f8, NODE0)
    q.writel(VI_PROC + 0x300, 1)                    # VI_HAL_Start
    q.cmd('clock_step %d' % (FRAME_NS // 2))

    hist = [a + b for a, b in zip(words(STT0 + 0x1c, 1024),
                                  words(STT1 + 0x1c, 1024))]
    check(abs(sum(hist) - SAMPLES * 4) <= 16,
          'offline: the strips\' histograms add up to the picture (%d)' %
          sum(hist))
    # strip 1 starts at AWB column 16 of 32: the fifth bar (luma 106)
    bar4 = int(WHITE * (106 - 16) / 219)
    z0 = words(STT0 + 0x182c, 2)
    z1 = words(STT1 + 0x182c, 2)
    check(z0[0] == (G << 16 | R) and z1[0] >> 16 == bar4 and
          z1[1] >> 16 == n,
          'offline: AWB zone 0 of each strip: G %d, G %d (bar 4), count %d'
          % (z0[0] >> 16, z1[0] >> 16, z1[1] >> 16))
    # strip 0 has AE columns 0..8 (row width 9), strip 1 9..16 (width 8)
    ae1 = words(STT1 + 0x101c + 8 * 4, 1)[0]        # row 1, column 9
    ae1_ref = words(STT1 + 0x101c, 1)[0]            # row 0, column 9
    check(ae1 == ae1_ref and words(STT0 + 0x1c, 1)[0] == 0,
          'offline: AE zones row by row at the strip\'s width')
    q.close()

    # the scene (hisi-scene): a picture instead of the bars, and the light
    # changed over QMP while the machine runs
    tmp = tempfile.mkdtemp()
    ppm = os.path.join(tmp, 'half.ppm')
    with open(ppm, 'wb') as f:                      # white left, black right
        f.write(b'P6\n64 36\n255\n' +
                (b'\xff' * 96 + b'\x00' * 96) * 36)
    qmp_path = os.path.join(tmp, 'qmp.sock')
    record = os.path.join(tmp, 'record.csv')
    q = QTest(a.qemu, a.machine, ('-global', 'hisi-scene.file=' + ppm,
                                  '-global', 'hisi-vi.record=' + record,
                                  '-qmp', 'unix:%s,server=on,wait=off' %
                                  qmp_path))
    qmp = socket.socket(socket.AF_UNIX)
    for _ in range(50):
        try:
            qmp.connect(qmp_path)
            break
        except OSError:
            time.sleep(0.1)
    qf = qmp.makefile('rw')
    qf.readline()                                   # the greeting

    def qmp_cmd(cmd, **args):
        qf.write(json.dumps({'execute': cmd, 'arguments': args}) + '\n')
        qf.flush()
        while True:
            r = json.loads(qf.readline())
            if 'return' in r or 'error' in r:
                return r
    qmp_cmd('qmp_capabilities')

    def frame():
        q.writel(VI_PROC + 0x680, BE_STT + 0x2800)
        q.writel(VI_PROC + 0x22110, 0x2020)
        q.writel(VI_CAP + 0x10000, 1 << 31)
        q.writel(VPSS + 0x000, 1 << 31)
        q.writel(VPSS + 0x0f0, 0x7401)
        q.cmd('clock_step %d' % FRAME_NS)
        z0 = words(BE_STT + 0x2800, 2)
        z31 = words(BE_STT + 0x2800 + 31 * 8, 2)
        return z0, z31

    z0, z31 = frame()
    check(z0[0] >> 16 == G and z31[0] >> 16 < 64,
          'scene file: white on the left (G %d), black on the right (G %d)'
          % (z0[0] >> 16, z31[0] >> 16))
    r = qmp_cmd('qom-set', path='/machine/scene', property='light-r',
                value=1000)
    z0, _ = frame()
    check('return' in r and z0[0] & 0xffff == G,
          'qom-set light-r 1000: the white zone\'s R follows (%d)' %
          (z0[0] & 0xffff))
    r = qmp_cmd('qom-get', path='/machine/scene', property='file')
    check(r.get('return') == ppm, 'qom-get file: %s' % r.get('return'))
    # broken files are refused and leave the scene as it was: a header
    # that ends at the end of the file, a huge picture with no pixels
    for name, data in (('eof.ppm', b'P6\n64 36\n255'),
                       ('huge.ppm', b'P6\n8192 8192\n255\n')):
        bad = os.path.join(tmp, name)
        with open(bad, 'wb') as f:
            f.write(data)
        r = qmp_cmd('qom-set', path='/machine/scene', property='file',
                    value=bad)
        z0, _ = frame()
        check('error' in r and z0[0] >> 16 == G,
              'scene file %s refused, the picture kept' % name)

    # a timeline (step 3): its clock starts at the next frame; a step at
    # 0.1 s and a ramp over 0.2..0.4 s
    timeline = os.path.join(tmp, 'timeline')
    with open(timeline, 'w') as f:
        f.write('# test\n0.1 light-r=300\n0.2..0.4 light-r=300->900\n')
    r = qmp_cmd('qom-set', path='/machine/scene', property='timeline',
                value=timeline)
    check('return' in r, 'timeline loaded over QMP')
    z0, _ = frame()                                 # t = 0: unchanged
    before = z0[0] & 0xffff
    for _ in range(3):                              # t = 0.12 s
        z0, _ = frame()
    step = z0[0] & 0xffff
    for _ in range(9):                              # past the ramp
        z0, _ = frame()
    ramp = z0[0] & 0xffff
    check(before == G and step == int(WHITE * 0.3) and
          ramp == int(WHITE * 0.9),
          'timeline: R %d, after the step %d, after the ramp %d' %
          (before, step, ramp))

    # an integer ramp (0..1: '%lf' alone would eat the first dot)
    with open(timeline, 'w') as f:
        f.write('0..1 light-b=100->1000\n')
    qmp_cmd('qom-set', path='/machine/scene', property='timeline',
            value=timeline)
    for _ in range(32):                             # 1.3 s
        z0, _ = frame()
    check(z0[1] & 0xffff == int(WHITE),
          'integer ramp 0..1: B at its end, %d' % (z0[1] & 0xffff))
    # timelines that are refused, leaving the one loaded
    for name, text in (('nested', '1 timeline=/tmp/x\n'),
                       ('nan', '1..2 light=oops->600\n'),
                       ('reversed', '2..1 light=100->600\n'),
                       ('garbage', '10xyz light=100\n')):
        bad = os.path.join(tmp, name)
        with open(bad, 'w') as f:
            f.write(text)
        r = qmp_cmd('qom-set', path='/machine/scene', property='timeline',
                    value=bad)
        cur = qmp_cmd('qom-get', path='/machine/scene',
                      property='timeline').get('return')
        check('error' in r and cur == timeline,
              'timeline "%s" refused' % text.strip())

    rows = open(record).read().splitlines()
    head = rows[0].split(',')
    lr = [int(float(x.split(',')[head.index('light_r')])) for x in rows[1:]]
    check(head[:3] == ['t', 'frame', 'light'] and len(rows) > 10 and
          300 in lr and 900 in lr and any(300 < v < 900 for v in lr),
          'record: %d frames, light-r through %s' %
          (len(rows) - 1, sorted(set(lr))[:3] + ['...'] +
           sorted(set(lr))[-1:]))
    qmp.close()
    q.close()

    print('%d failure(s)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
