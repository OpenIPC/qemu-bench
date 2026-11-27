#!/usr/bin/env python3
"""XM_REGTRACE, the Xiongmai machines' register write log, under qtest.

XM_REGTRACE=<file> makes the second-generation media blocks append one line
per guest write: virtual time in seconds, the CPU mode ('U' user, 'K'
kernel, '-' none, as under qtest), the device, the offset and the value,
both in hex. openxiongmai's capture tools (tools/xmv200-capture) read it.

  - unset, nothing is appended to an earlier log;
  - set, writes to XM232's VI/ISP window (xm-vimu), its encoder (xm-hevc,
    layout xm232) and the system controller come out in order, one line
    each, in that format;
  - on the XM530V200, the xm-isp2 blocks log under "isp2.<block>", the
    OSD engine under "osd" and the video output under "vo";
  - on the XM533, the FFT engine logs under "fft".

    python3 qemu/tests/test-xm-regtrace.py [--qemu PATH]
"""
import argparse, os, re, sys, tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from qtest_harness import QTest

LINE = re.compile(r'^(\d+\.\d{6}) ([UK-]) (\S+) ([0-9a-f]{8}) ([0-9a-f]{8})$')


def run(qemu, machine, trace, writes):
    """Start the machine under qtest, XM_REGTRACE=trace if not None."""
    env = os.environ.pop('XM_REGTRACE', None)
    if trace is not None:
        os.environ['XM_REGTRACE'] = trace
    try:
        q = QTest(qemu, machine)
        try:
            for a, v in writes:
                q.writel(a, v)
            q.cmd('clock_step 1000000')
        finally:
            q.close()
    finally:
        os.environ.pop('XM_REGTRACE', None)
        if env is not None:
            os.environ['XM_REGTRACE'] = env


def lines(path):
    with open(path) as f:
        return [l.rstrip('\n') for l in f]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='./qemu-src/build/qemu-system-arm')
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    d = tempfile.mkdtemp()

    # XM232: VI/ISP window, encoder, system controller
    xm232 = [(0x30010024, 0x3554515),      # vi_sel enable mask
             (0x21000000 + 0x10c, 0x92300000),  # encoder source luma
             (0x20000124, 0x1)]            # NPU reset (sysctl)
    t = os.path.join(d, 'xm232.trace')
    run(a.qemu, 'xm232', t, xm232)
    n = len(lines(t))
    run(a.qemu, 'xm232', None, xm232)
    check(n and len(lines(t)) == n, 'XM_REGTRACE unset: nothing logged')
    got = [LINE.match(l) for l in lines(t)]
    check(got and all(got), 'every line is "time mode device offset value"')
    rec = [(m.group(3), int(m.group(4), 16), int(m.group(5), 16))
           for m in got if m]
    want = [('vimu', 0x10024, 0x3554515), ('hevc', 0x10c, 0x92300000),
            ('sysctl', 0x124, 0x1)]
    check([r for r in rec if r in want] == want,
          'xm232: vimu, hevc and sysctl writes logged in order')
    check(all(m.group(2) == '-' for m in got if m),
          'qtest writes come from no CPU (mode -)')

    # XM530V200: xm-isp2's blocks and the OSD engine
    t = os.path.join(d, 'v200.trace')
    run(a.qemu, 'xm530v200', t, [(0x30000100, 0x11), (0x32000100, 0x22),
                                 (0x21300100, 0x33), (0x31000100, 0x44)])
    rec = {(m.group(3), int(m.group(4), 16), int(m.group(5), 16))
           for m in map(LINE.match, lines(t)) if m}
    check(('isp2.vi', 0x100, 0x11) in rec, 'xm530v200: VICU window as isp2.vi')
    check(('isp2.bpu', 0x100, 0x22) in rec, 'xm530v200: BPU as isp2.bpu')
    check(('osd', 0x100, 0x33) in rec, 'xm530v200: OSD engine as osd')
    check(('vo', 0x100, 0x44) in rec, 'xm530v200: video output as vo')

    # XM533: the FFT engine (ive.ko)
    t = os.path.join(d, 'xm533.trace')
    run(a.qemu, 'xm533', t, [(0x21300000, 0x55)])
    rec = {(m.group(3), int(m.group(4), 16), int(m.group(5), 16))
           for m in map(LINE.match, lines(t)) if m}
    check(('fft', 0x0, 0x55) in rec, 'xm533: FFT engine as fft')

    print('%s: %d failure(s)' % (os.path.basename(__file__), fails))
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
