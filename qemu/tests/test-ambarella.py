#!/usr/bin/env python3
"""Register-level checks of the Ambarella S3L models, no guest needed.

QEMU runs the s3l machine under qtest and this script plays the vendor
kernel (docs/ambarella.md):

  - amb-rct: the PLL words decode to the rates /proc/ambarella/clock shows
    on an IP4M-1026E, PLL lock reads set;
  - amb-timer: a one-shot count raises its line through the VIC, the
    free-running clocksource reloads, a disabled timer stands still;
  - amb-vic: the scratchpad names the pending line and all ones when there
    is none, which is the only way irq-ambarella.c looks on S3L; indexed and
    mask registers agree, an edge stays latched until cleared;
  - amb-idc: the driver's CTRL conversation reads the OV4689's chip ID from
    I2C0, and an address nobody answers comes back not acknowledged;
  - sensor=none leaves the bus empty, too little RAM for the DSP is refused,
    and machine state migrates to a file and back.

    python3 qemu/tests/test-ambarella.py [--qemu PATH]
"""
import argparse
import atexit
import os
import shutil
import socket
import subprocess
import sys
import tempfile

VIC0 = 0xE0003000
VIC1 = 0xE0010000
VIC2 = 0xE001C000
SCRATCHPAD = 0xE001B000
IDC0 = 0xE8003000
TIMER = 0xE800B000
RCT = 0xEC170000

PRI_IRQ = SCRATCHPAD + 0x38
NO_IRQ = 0xFFFFFFFF

IRQ_STA = 0x00
INTEN = 0x10
INTEN_CLR = 0x14
INT_EN = 0x44
INT_EN_CLR = 0x48
INT_SENSE = 0x54
INT_EVT = 0x64
INT_EDGE_CLR = 0x98

TIMER8 = TIMER + 0x74
TIMER7 = TIMER + 0x64
TIMER_CTRL = TIMER + 0x30
T7_EN, T7_OF = 1 << 24, 4 << 24
T8_EN, T8_OF = 1 << 28, 4 << 28
LINE_TIMER8 = 63

CTRL = 0x04
DATA = 0x08
ENR = 0x00
C_STOP, C_START, C_IF, C_ACK = 0x08, 0x04, 0x02, 0x01

OV4689 = 0x36


class QTest:
    def __init__(self, qemu, machine, extra=()):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        srv = socket.socket(socket.AF_UNIX)
        srv.bind(path)
        srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest',
             '-display', 'none', '-nodefaults', '-qtest', 'unix:' + path] +
            list(extra),
            stdout=subprocess.DEVNULL,
            stderr=open(os.path.join(self.dir, 'qemu.err'), 'w'))
        srv.settimeout(20)
        self.sock, _ = srv.accept()
        self.f = self.sock.makefile('rw', buffering=1)

    def cmd(self, line):
        self.f.write(line + '\n')
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith(('OK', 'FAIL', 'ERR')):
                return r

    def writel(self, a, v):
        self.cmd('writel 0x%x 0x%x' % (a, v))

    def readl(self, a):
        return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def step(self, ns):
        self.cmd('clock_step %d' % ns)

    def close(self):
        self.proc.kill()
        self.proc.wait()
        shutil.rmtree(self.dir, ignore_errors=True)


failures = 0


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what,
                             hex(got) if isinstance(got, int)
                             and not isinstance(got, bool) else got))


def pll_rate(word):
    """ref * (intp + 1) * (sdiv + 1) / (sout + 1), as the kernel's clk.c."""
    intp = ((word >> 24) & 0x7F) + 1
    sdiv = ((word >> 12) & 0xF) + 1
    sout = ((word >> 16) & 0xF) + 1
    return 24000000 * intp * sdiv // sout


def test_rct(qt):
    print('amb-rct')
    core = pll_rate(qt.readl(RCT + 0x000))
    check('pll_out_core', core, 720000000)
    check('gclk_apb, core / 8', core // 8, 90000000)
    check('gclk_cortex', pll_rate(qt.readl(RCT + 0x264)), 816000000)
    check('gclk_ddr is half the PLL', pll_rate(qt.readl(RCT + 0xDC)) // 2,
          840000000)
    check('gclk_idsp', pll_rate(qt.readl(RCT + 0xE4)), 348000000)
    check('gclk_uart is the reference', qt.readl(RCT + 0x38), 1)
    check('PLLs read locked', qt.readl(RCT + 0x2C), 0xFFFFFFFF)
    qt.writel(RCT + 0x34, 0x1234)
    check('other registers are storage', qt.readl(RCT + 0x34), 0x1234)


def test_timer_and_vic(qt):
    print('amb-timer through amb-vic')
    check('nothing pending names no line', qt.readl(PRI_IRQ), NO_IRQ)

    bank = VIC1
    bit = LINE_TIMER8 - 32
    # irq_set_type() for rising edge, then irq_unmask()
    qt.writel(bank + 0x58, bit)
    qt.writel(bank + 0x60, bit)
    qt.writel(bank + INT_EVT, bit)
    qt.writel(bank + INT_EN, bit)
    check('indexed enable shows in the mask', qt.readl(bank + INTEN),
          1 << bit)

    qt.writel(TIMER8 + 0x0, 900)        # 10 us at 90 MHz
    qt.writel(TIMER8 + 0x4, 0xFFFFFFFF)
    qt.writel(TIMER_CTRL, T8_EN | T8_OF)
    qt.step(5000)
    left = qt.readl(TIMER8)
    check('the count runs at 90 MHz', 440 <= left <= 460, True)
    check('nothing pending half way', qt.readl(PRI_IRQ), NO_IRQ)
    qt.step(6000)
    check('expiry names the line', qt.readl(PRI_IRQ), LINE_TIMER8)
    check('and shows in the bank', qt.readl(bank + IRQ_STA), 1 << bit)
    check('the count reloaded', qt.readl(TIMER8) > 0xFFFF0000, True)
    qt.step(1000)
    check('an edge stays latched', qt.readl(PRI_IRQ), LINE_TIMER8)
    qt.writel(bank + INT_EDGE_CLR, bit)
    check('until it is cleared', qt.readl(PRI_IRQ), NO_IRQ)

    qt.writel(TIMER_CTRL, 0)
    frozen = qt.readl(TIMER8)
    qt.step(100000)
    check('a disabled timer stands still', qt.readl(TIMER8), frozen)
    qt.writel(bank + INT_EN_CLR, bit)
    check('indexed disable clears the mask', qt.readl(bank + INTEN), 0)

    qt.writel(TIMER7 + 0x0, 0xFFFFFFFF)
    qt.writel(TIMER7 + 0x4, 0xFFFFFFFF)
    qt.writel(TIMER_CTRL, T7_EN)
    a = qt.readl(TIMER7)
    qt.step(1000)
    check('the clocksource counts down', a - qt.readl(TIMER7), 90)
    qt.writel(TIMER_CTRL, 0)


def test_lines(qt):
    print('amb-vic trigger setup')
    # A software interrupt on bank 2 goes through the same scratchpad
    qt.writel(VIC2 + 0x18, 1 << 3)
    check('nothing while masked', qt.readl(PRI_IRQ), NO_IRQ)
    qt.writel(VIC2 + INTEN, 1 << 3)
    check('bank 2 numbers from 64', qt.readl(PRI_IRQ), 64 + 3)
    qt.writel(VIC0 + 0x18, 1 << 5)
    qt.writel(VIC0 + INTEN, 1 << 5)
    check('the lowest line goes first', qt.readl(PRI_IRQ), 5)
    qt.writel(VIC0 + 0x1C, 1 << 5)
    qt.writel(VIC2 + 0x1C, 1 << 3)
    qt.writel(VIC0 + INTEN_CLR, 0xFFFFFFFF)
    qt.writel(VIC2 + INTEN_CLR, 0xFFFFFFFF)
    check('and none left', qt.readl(PRI_IRQ), NO_IRQ)


def idc_ctrl(qt, val):
    """One CTRL write; what the interrupt handler would read back."""
    qt.writel(IDC0 + CTRL, val)
    return qt.readl(IDC0 + CTRL)


def read_reg16(qt, addr, reg):
    """write(fd, reg, 2) then read(fd, buf, 1), as ipctool does."""
    qt.writel(IDC0 + DATA, addr << 1)
    if idc_ctrl(qt, C_START) & C_ACK:
        idc_ctrl(qt, C_STOP | C_ACK)
        return None
    for b in (reg >> 8, reg & 0xFF):
        qt.writel(IDC0 + DATA, b)
        check_ack = idc_ctrl(qt, 0)
        if check_ack & C_ACK:
            idc_ctrl(qt, C_STOP | C_ACK)
            return None
    idc_ctrl(qt, C_STOP)
    qt.writel(IDC0 + DATA, addr << 1 | 1)
    if idc_ctrl(qt, C_START) & C_ACK:
        idc_ctrl(qt, C_STOP | C_ACK)
        return None
    idc_ctrl(qt, C_ACK)                 # one byte: no acknowledge
    val = qt.readl(IDC0 + DATA) & 0xFF
    idc_ctrl(qt, C_STOP)
    return val


def test_i2c(qt):
    print('amb-idc')
    # irq_set_type() for level high, as the device tree asks on line 83
    qt.writel(VIC2 + INT_SENSE, 19)
    qt.writel(VIC2 + INT_EVT, 19)
    qt.writel(IDC0 + ENR, 1)
    check('OV4689 ID high', read_reg16(qt, OV4689, 0x300A), 0x46)
    check('OV4689 ID low', read_reg16(qt, OV4689, 0x300B), 0x88)
    check('the line is low between transfers',
          qt.readl(VIC2 + 0x08) & (1 << 19), 0)
    qt.writel(IDC0 + DATA, 0x50 << 1)
    check('nobody at 0x50: IF and no ACK', idc_ctrl(qt, C_START),
          C_IF | C_ACK)
    check('IDC0 interrupts on line 83',
          qt.readl(VIC2 + 0x08) & (1 << 19), 1 << 19)
    idc_ctrl(qt, C_STOP | C_ACK)
    check('STOP lowers the line', qt.readl(VIC2 + 0x08) & (1 << 19), 0)


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


def test_no_sensor(qemu):
    print('sensor=none')
    qt = QTest(qemu, 's3l,sensor=none')
    try:
        qt.writel(IDC0 + ENR, 1)
        check('the bus is empty', read_reg16(qt, OV4689, 0x300A), None)
    finally:
        qt.close()


def test_rejections_and_migration(qemu, tmp):
    print('rejections and migration')
    r = subprocess.run([qemu, '-M', 's3l', '-m', '64M', '-display', 'none',
                        '-nodefaults', '-kernel', '/dev/null'],
                       capture_output=True, text=True, timeout=20)
    check('no room for the DSP is refused', 'room for the DSP' in r.stderr,
          True)

    mig = os.path.join(tmp, 's3l.mig')
    out = hmp(qemu, ['-M', 's3l', '-S'],
              ['migrate file:' + mig, 'info migrate'])
    check('state saved', 'completed' in out and 'Assertion' not in out and
          os.path.getsize(mig) > 0, True)
    out = hmp(qemu, ['-M', 's3l', '-S', '-incoming', 'file:' + mig],
              ['info status'])
    check('and loaded back', 'error' not in out.lower() and
          'Assertion' not in out, True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '../../qemu-src/build/qemu-system-arm'))
    args = ap.parse_args()

    tmp = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, tmp, True)
    qt = QTest(args.qemu, 's3l')
    try:
        test_rct(qt)
        test_timer_and_vic(qt)
        test_lines(qt)
        test_i2c(qt)
    finally:
        qt.close()
    test_no_sensor(args.qemu)
    test_rejections_and_migration(args.qemu, tmp)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
