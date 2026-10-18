#!/usr/bin/env python3
"""The XM530/XM550 pad controller (xm-gpio) the way the gpio-xm530 kernel
driver and the vendor ISP use it: one register per pad, output enable and
level in bits 10/11, input enable 12, the pad's level read back in bit 13.

No guest: QEMU runs under qtest and this script writes the registers.

    python3 qemu/tests/test-xm-gpio.py [--qemu PATH] [--machine xm530]
"""
import argparse, os, sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

BASE = 0x10020000
OUT_EN, OUT_HIGH, IN_EN, IN_LEVEL = 1 << 10, 1 << 11, 1 << 12, 1 << 13


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine', default='xm530')
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    q = jt.QTest(a.qemu, a.machine, ('-d', 'unimp'))
    pad = lambda n: BASE + 4 * n
    try:
        # output: bit 13 follows the driven level
        q.writel(pad(1), OUT_EN | OUT_HIGH)
        check(q.readl(pad(1)) & IN_LEVEL, 'pad 1 driven high reads back high')
        q.writel(pad(1), OUT_EN)
        check(not q.readl(pad(1)) & IN_LEVEL, 'pad 1 driven low reads back low')
        # input: bit 13 is the line, whatever is written to it
        q.writel(pad(49), IN_EN | IN_LEVEL)
        check(not q.readl(pad(49)) & IN_LEVEL, 'pad 49 input: bit 13 is read only')
        q.cmd('set_irq_in /machine/xm-gpio unnamed-gpio-in 49 1')
        check(q.readl(pad(49)) & IN_LEVEL, 'pad 49 input follows its line high')
        q.cmd('set_irq_in /machine/xm-gpio unnamed-gpio-in 49 0')
        check(not q.readl(pad(49)) & IN_LEVEL, 'pad 49 input follows its line low')
        # mux and drive bits are kept as written
        q.writel(pad(15), 0x82)
        check(q.readl(pad(15)) == 0x82, 'pad 15 keeps its mux and drive bits')
        # past the pads: plain RAM (control words, the light ADC)
        q.writel(BASE + 0x3e0, 0x80)
        check(q.readl(BASE + 0x3e0) == 0x80, 'a control word past the pads is RAM')
    finally:
        q.close()

    log = open(os.path.join(q.dir, 'qemu.err')).read()
    check('xm-gpio: pad 1 high' in log and 'xm-gpio: pad 1 low' in log,
          'output changes are logged under -d unimp')

    print('=== %s: xm-gpio ===' % ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
