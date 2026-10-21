#!/usr/bin/env python3
"""The XM510 GPIO block (xm510-gpio) the way a kernel driver uses it: banked,
eight pins a port, a mux word per port at +P*4, and per port at
+0x40 + P*0x10 the data word (+0) and the output enable (+4).

No guest: QEMU runs under qtest and this script writes the registers.

    python3 qemu/tests/test-xm510-gpio.py [--qemu PATH]
"""
import argparse, os, re, sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

BASE = 0x10020000
SYS = 0x20000000
mux = lambda p: BASE + 4 * p
data = lambda p: BASE + 0x40 + 0x10 * p
oe = lambda p: BASE + 0x44 + 0x10 * p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    q = jt.QTest(a.qemu, 'xm510', ('-d', 'unimp'))
    try:
        # output: the data word reads back what an enabled output drives
        q.writel(oe(8), 0x02)
        q.writel(data(8), 0x02)
        check(q.readl(data(8)) & 0x02, 'port 8 pin 1 driven high reads back high')
        q.writel(data(8), 0x00)
        check(not q.readl(data(8)) & 0x02, 'port 8 pin 1 driven low reads back low')

        # input: the data bit is the line, whatever was written to it
        q.writel(oe(2), 0x00)
        q.writel(data(2), 0x10)
        check(not q.readl(data(2)) & 0x10, 'port 2 pin 4 input ignores the latch')
        q.cmd('set_irq_in /machine/xm-gpio unnamed-gpio-in 20 1')
        check(q.readl(data(2)) & 0x10, 'port 2 pin 4 input follows line 20 high')
        q.cmd('set_irq_in /machine/xm-gpio unnamed-gpio-in 20 0')
        check(not q.readl(data(2)) & 0x10, 'port 2 pin 4 input follows line 20 low')

        # mux words and the words past the ports are kept as written
        q.writel(mux(1), 0x300500)
        check(q.readl(mux(1)) == 0x300500, 'port 1 keeps its mux word')
        q.writel(BASE + 0x48, 0x2)
        check(q.readl(BASE + 0x48) == 0x2, 'a port word past data/OE is RAM')
        # +0xD0 is the MAC's speed select, not a port: bit 0 reads back as set
        q.writel(BASE + 0xd0, 0x1)
        check(q.readl(BASE + 0xd0) == 0x1, '+0xD0 (mac_speed_o) is RAM, not a port')
        q.writel(BASE + 0x200, 0x55)
        check(q.readl(BASE + 0x200) == 0x55, 'a word past the ports is RAM')

        def lines(cmd):
            q.f.write(cmd + '\n'); q.f.flush()
            seen = []
            while True:
                r = q.f.readline().strip()
                seen.append(r)
                if r.startswith(('OK', 'FAIL', 'ERR')):
                    return seen
        lines('irq_intercept_out /machine/xm-gpio')

        # a muxed pin does not drive, and lets go when it is muxed later
        lines('writel 0x%x 0x%x' % (mux(3), 1 << 13))      # pin 5's 2nd function
        lines('writel 0x%x 0x20' % oe(3))
        seen = lines('writel 0x%x 0x20' % data(3))
        check('IRQ raise 29' not in seen, 'a muxed pin (port 3 pin 5) drives nothing')
        seen = lines('writel 0x%x 0' % mux(3))
        check('IRQ raise 29' in seen, 'unmuxed, it drives its latched level')
        seen = lines('writel 0x%x 0x20' % mux(3))
        check('IRQ lower 29' in seen, 'muxed again, it lets go of its line')

        # reset: every pin comes back an undriven input, its line lowered
        lines('writel 0x%x 0' % mux(3))
        lines('writel 0x%x 0x08' % oe(0))
        seen = lines('writel 0x%x 0x08' % data(0))
        check('IRQ raise 3' in seen, 'port 0 pin 3 driven high raises line 3')
        # XM510's soft reset is SYS +0x30/+0x34 (XM530's is +0x80/+0x84)
        lines('writel 0x%x 1' % (SYS + 0x30))
        seen = lines('writel 0x%x 0xca110000' % (SYS + 0x34))
        for _ in range(3):
            seen += lines('readl 0x%x' % data(0))
        check('IRQ lower 3' in seen, 'a machine reset lowers line 3')
        check(q.readl(oe(0)) == 0, 'and clears the output enable')
    finally:
        q.close()

    log = q.err
    check('xm510-gpio: gpio 65 high' in log and 'xm510-gpio: gpio 65 low' in log,
          'output changes are logged under -d unimp')
    check('xm510-gpio: gpio 3 released' in log,
          'the reset logs the release of a pin it was driving')
    # 29 was let go when muxed again; 3 and 65 were still driven at reset.
    released = sorted({int(n) for n in re.findall(r'gpio (\d+) released', log)})
    check(released == [3, 29, 65], 'and of no pin that was not driven')

    print('=== %s: xm510-gpio ===' % ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
