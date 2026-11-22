#!/usr/bin/env python3
"""hisi-pwm: the PWM controller of every HiSilicon IPC machine, over qtest.

For each machine, what a driver can get wrong on silicon:

  - gated: with the CRG word not ungating it, the block takes the write and
    reads back zero (an hi3516av300 at its stock 0x120101BC does exactly
    this);
  - clocked: once the word is right, a period reads back, on the last channel
    the block decodes and on no channel past it;
  - rate: a bounded run's remaining-periods counter falls at the clock the
    mux selects -- the bench method of OpenIPC/majestic#324 (sample the count
    twice, a known time apart), against the model's clock;
  - latch: a duty reaches the output on a CTRL write, and on V1 only on the
    0->1 edge of the enable bit;
  - output: the period and high time QMP reports for the channel, with V5's
    minus-one encoding, the classic two-tick floor and the invert bit.

    python3 qemu/tests/test-hisi-pwm.py [--qemu Q] [machine ...]
"""
import argparse, json, os, socket, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from qtest_harness import QTest  # noqa: E402

M = 1000000
# machine: (pwm base, CRG base, family)
FAM = {
    #        layout   ch  edge  crg off gate  reset mux(shift,width) rates
    'V1':   ('classic', 3, True,  0x38,  0x2,  0,    None,   [3 * M]),
    'V2':   ('classic', 4, False, 0x38,  0x2,  0,    (2, 1), [3 * M, 50 * M]),
    'V2A':  ('split',   8, False, 0x38,  0x2,  0,    (2, 1), [3 * M, 50 * M]),
    'V3A':  ('classic', 8, False, 0xdc,  0x2,  0,    (2, 1), [3 * M, 50 * M]),
    'V4A':  ('classic', 2, False, 0x1bc, 0x80, 0x40, (8, 2),
             [3 * M, 50 * M, 24 * M, 24 * M]),
    'V4':   ('classic', 4, False, 0x1bc, 0x80, 0x40, (8, 2),
             [3 * M, 50 * M, 24 * M, 24 * M]),
    'GK500': ('classic', 12, False, 0x1bc, 0x80, 0x40, (8, 2),
              [3 * M, 50 * M, 24 * M, 24 * M]),
    'V5':   ('v5',      8, False, 0x4598, 0x10, 0x1, (12, 2),
             [1 * M, 24 * M, 198 * M]),
}
MACHINES = {
    'hi3516cv100': (0x20130000, 0x20030000, 'V1'),
    'hi3518ev100': (0x20130000, 0x20030000, 'V1'),
    'hi3516cv200': (0x20130000, 0x20030000, 'V2'),
    'hi3518ev200': (0x20130000, 0x20030000, 'V2'),
    'hi3516av100': (0x20130000, 0x20030000, 'V2A'),
    'hi3516dv100': (0x20130000, 0x20030000, 'V2A'),
    'hi3516cv300': (0x12130000, 0x12010000, 'V2'),
    'hi3516ev100': (0x12130000, 0x12010000, 'V2'),
    'hi3519v101':  (0x12130000, 0x12010000, 'V3A'),
    'hi3516av200': (0x12130000, 0x12010000, 'V3A'),
    'hi3516cv500': (0x12070000, 0x12010000, 'V4A'),
    'hi3516av300': (0x12070000, 0x12010000, 'V4A'),
    'hi3516dv300': (0x12070000, 0x12010000, 'V4A'),
    'hi3516ev200': (0x12080000, 0x12010000, 'V4'),
    'hi3516ev300': (0x12080000, 0x12010000, 'V4'),
    'gk7205v200':  (0x12080000, 0x12010000, 'V4'),
    'gk7205v500':  (0x12080000, 0x12010000, 'GK500'),
    'hi3516cv608': (0x11080000, 0x11010000, 'V5'),
    'hi3516cv610': (0x11080000, 0x11010000, 'V5'),
    'hi3516cv613': (0x11080000, 0x11010000, 'V5'),
}


class QMP:
    def __init__(self, path):
        self.s = socket.socket(socket.AF_UNIX)
        for _ in range(200):
            try:
                self.s.connect(path)
                break
            except OSError:
                import time
                time.sleep(0.05)
        self.f = self.s.makefile('rw', buffering=1)
        json.loads(self.f.readline())
        self.call('qmp_capabilities')

    def call(self, cmd, **args):
        self.f.write(json.dumps({'execute': cmd, 'arguments': args}) + '\n')
        while True:
            r = json.loads(self.f.readline())
            if 'return' in r:
                return r['return']
            if 'error' in r:
                raise RuntimeError(r['error'])

    def get(self, prop):
        return self.call('qom-get', path='/machine/pwm', property=prop)


class Block:
    def __init__(self, qt, base, crg, fam):
        (self.layout, self.channels, self.edge, off, self.gate, self.reset,
         self.mux, self.rates) = fam
        self.qt, self.base, self.crg = qt, base, crg + off
        v5 = self.layout == 'v5'
        self.CFG0, self.CFG1 = 0x0, 0x4
        self.CFG2, self.CTRL = (0x10, 0x14) if v5 else (0x8, 0xc)
        self.REM = 0x50 if v5 else 0x18

    def ch(self, n):
        if self.layout == 'v5':
            return self.base + n * 0x100
        if self.layout == 'split' and n >= 4:
            return self.base + 0x1000 + (n - 4) * 0x20
        return self.base + n * 0x20

    def clock(self, mux=0, on=True):
        w = self.qt.readl(self.crg)
        w &= ~(self.gate | self.reset)
        if self.mux:
            sh, wd = self.mux
            w = (w & ~(((1 << wd) - 1) << sh)) | (mux << sh)
        if on:
            w |= self.gate
        self.qt.writel(self.crg, w)

    def remaining(self, n):
        v = self.qt.readl(self.ch(n) + self.REM)
        return v if self.layout == 'v5' else v >> 12

    def ticks(self, v):
        return v + 1 if self.layout == 'v5' else v


def check(name, cond, what, fails):
    if not cond:
        fails.append('%s: %s' % (name, what))


def run(qemu, name):
    base, crg, fam = MACHINES[name]
    tmp = os.path.join('/tmp', 'hisi-pwm-qmp-%d-%s.sock' % (os.getpid(), name))
    qt = QTest(qemu, name, extra=['-qmp', 'unix:%s,server=on,wait=off' % tmp])
    fails = []
    try:
        qmp = QMP(tmp)
        b = Block(qt, base, crg, FAM[fam])
        last = b.channels - 1

        # gated: takes the write, keeps nothing
        b.clock(on=False)
        qt.writel(b.ch(0) + b.CFG0, 0x1234)
        check(name, qt.readl(b.ch(0) + b.CFG0) == 0, 'gated block read back',
              fails)
        if b.reset:
            qt.writel(b.crg, qt.readl(b.crg) | b.gate | b.reset)
            qt.writel(b.ch(0) + b.CFG0, 0x1234)
            check(name, qt.readl(b.ch(0) + b.CFG0) == 0,
                  'block in reset read back', fails)

        # clocked: the last channel decodes, the one after it does not
        b.clock(0)
        for n in (0, last):
            qt.writel(b.ch(n) + b.CFG0, 0x1000 + n)
        check(name, qt.readl(b.ch(0) + b.CFG0) == 0x1000,
              'ch0 period did not read back', fails)
        check(name, qt.readl(b.ch(last) + b.CFG0) == 0x1000 + last,
              'ch%d period did not read back' % last, fails)
        if b.layout != 'split':
            qt.writel(b.ch(b.channels) + b.CFG0, 0x55)
            check(name, qt.readl(b.ch(b.channels) + b.CFG0) == 0,
                  'ch%d decodes but should not' % b.channels, fails)

        # rate per mux value: a bounded run, sampled a known time apart
        for mux, hz in enumerate(b.rates):
            if not hz or (mux and not b.mux):
                continue
            b.clock(mux)
            check(name, qmp.get('clock-hz') == hz, 'clock-hz at mux %d' % mux,
                  fails)
            period = 0x4000 - (1 if b.layout == 'v5' else 0)
            ticks = b.ticks(period)
            n = 0xfffff if b.layout == 'v5' else 0xfffff
            c = b.ch(1 if b.channels > 1 else 0)
            qt.writel(c + b.CTRL, 0)
            qt.writel(c + b.CFG0, period)
            qt.writel(c + b.CFG1, period // 2)
            qt.writel(c + b.CFG2, n)
            qt.writel(c + b.CTRL, 0x1)       # enable, NOT keep: bounded
            r0 = b.remaining(1 if b.channels > 1 else 0)
            step = 200 * ticks * 1000000000 // hz   # 200 periods
            qt.cmd('clock_step %d' % step)
            r1 = b.remaining(1 if b.channels > 1 else 0)
            got = (r0 - r1) * ticks * 1e9 / step if r0 > r1 else 0
            check(name, abs(got - hz) <= hz * 0.01,
                  'mux %d measured %.0f Hz, modelled %d' % (mux, got, hz),
                  fails)
            qt.writel(c + b.CTRL, 0)

        # output, latch and polarity on channel 0, at mux 0
        b.clock(0)
        hz = b.rates[0]
        c = b.ch(0)
        qt.writel(c + b.CTRL, 0)
        qt.writel(c + b.CFG0, 2499 if b.layout == 'v5' else 7500)
        qt.writel(c + b.CFG1, 624 if b.layout == 'v5' else 1875)
        qt.writel(c + b.CTRL, 0x5)
        per = 2500 if b.layout == 'v5' else 7500
        hi = 625 if b.layout == 'v5' else 1875
        ns = lambda t: t * 1000000000 // hz
        check(name, qmp.get('ch0-enabled') is True, 'ch0 not running', fails)
        check(name, qmp.get('ch0-period-ns') == ns(per),
              'period %s ns, want %d' % (qmp.get('ch0-period-ns'), ns(per)),
              fails)
        check(name, qmp.get('ch0-high-ns') == ns(hi),
              'high %s ns, want %d' % (qmp.get('ch0-high-ns'), ns(hi)), fails)

        # a new duty without a CTRL write
        qt.writel(c + b.CFG1, 3750 if b.layout != 'v5' else 1249)
        hi2 = 1250 if b.layout == 'v5' else 3750
        want = hi2 if b.layout == 'v5' else hi   # v5 rewrites a running duty
        check(name, qmp.get('ch0-high-ns') == ns(want),
              'CFG1 alone: high %s ns, want %d' %
              (qmp.get('ch0-high-ns'), ns(want)), fails)
        # a redundant CTRL write: loads everywhere but V1
        qt.writel(c + b.CTRL, 0x5)
        want = hi if b.edge else hi2
        check(name, qmp.get('ch0-high-ns') == ns(want),
              'redundant CTRL: high %s ns, want %d' %
              (qmp.get('ch0-high-ns'), ns(want)), fails)
        # the enable edge loads everywhere
        qt.writel(c + b.CTRL, 0)
        check(name, qmp.get('ch0-enabled') is False, 'CTRL=0 still running',
              fails)
        qt.writel(c + b.CTRL, 0x5)
        check(name, qmp.get('ch0-high-ns') == ns(hi2),
              'enable edge: high %s ns, want %d' %
              (qmp.get('ch0-high-ns'), ns(hi2)), fails)
        # invert
        qt.writel(c + b.CTRL, 0x7)
        check(name, qmp.get('ch0-high-ns') == ns(per - hi2),
              'inverted: high %s ns, want %d' %
              (qmp.get('ch0-high-ns'), ns(per - hi2)), fails)
        # a zero duty is not dark: the floor
        qt.writel(c + b.CTRL, 0)
        qt.writel(c + b.CFG1, 0)
        qt.writel(c + b.CTRL, 0x5)
        floor = 1 if b.layout == 'v5' else 2
        check(name, qmp.get('ch0-high-ns') == ns(floor),
              'zero duty: high %s ns, want the %d-tick floor' %
              (qmp.get('ch0-high-ns'), floor), fails)
        if b.layout != 'v5':
            check(name, qt.readl(c + 0x14) == 2, 'ST1 does not read 2', fails)

        # gating a running block stops the output
        b.clock(on=False)
        check(name, qmp.get('ch0-enabled') is False,
              'output survives the clock gate', fails)
    finally:
        qt.close()
        try:
            os.unlink(tmp)
        except OSError:
            pass
    return fails


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(
        here, '../../qemu-src/build/qemu-system-arm'))
    ap.add_argument('machines', nargs='*')
    a = ap.parse_args()
    fails = []
    for m in a.machines or MACHINES:
        f = run(a.qemu, m)
        print('%-12s %s' % (m, 'ok' if not f else 'FAIL'))
        fails += f
    for f in fails:
        print('  ' + f)
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
