#!/usr/bin/env python3
"""Register-level checks of the Goke GK7102S / GK7102C machine, no guest.

QEMU runs the gk7102s machine under qtest and this script plays the vendor
drivers, which on the S die reach the blocks directly at the AHB 0x90000000
/ APB 0xA0000000 addresses, in the register layouts of the SDK's
mach-gk710xs headers (docs/goke-gk710x.md):

  - the boot ROM: an S header (its delay entry included) applies its
    register writes and stages U-Boot;
  - gk-remap over gk-timer and gk-vic: timer 3 programmed with the S
    timer.h bits (EN3 0x100, OF3 0x10) raises kernel IRQ 20 in the VIC's
    IRQ status at +0x30 (what entry-macro.S reads) until EDGE_CLR;
  - gk-remap over gk-wdt: the S wdt.h layout, the count runs into TIMEOUT;
  - gk-remap over gk-idc: start / data / read against sensor=jxh42 with the
    S idc.h offsets and CTRL bits;
  - gk-dma: one descriptor run as the GK7102C U-Boot checks it at start --
    DA moves on to the descriptor's next, the data and the report land, the
    channel's INT bit is set until STA is cleared.

    python3 qemu/tests/test-gk710xs.py [--qemu PATH]
"""
import argparse
import os
import socket
import struct
import subprocess
import sys
import tempfile

VIC1 = 0x90008000
TIMER = 0xA0000000
WDT = 0xA0006000
IDC = 0xA0003000
DDRC = 0x90002000
DMAC = 0x90003000
RAM = 0xC1000000

# mach-gk710xs/include/mach/irqs.h: VIC register offsets and TIMER3_IRQ.
VIC_SENSE, VIC_EVENT, VIC_INTEN, VIC_IRQ_STA, VIC_EDGE_CLR = \
    0x00, 0x04, 0x10, 0x30, 0x38
TIMER3_IRQ = 20


class QTest:
    def __init__(self, qemu, machine):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        srv = socket.socket(socket.AF_UNIX)
        srv.bind(path)
        srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest',
             '-display', 'none', '-nodefaults', '-qtest', 'unix:' + path,
             '-serial', 'null'],
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


failures = 0


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    shown = got if isinstance(got, (bool, str)) else hex(got)
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown))


def make_image(path):
    """1 MiB, as the chip holds it (every word byte-swapped): a GK7102C-style
    boot header and a two-word "U-Boot"."""
    code = [0xE1A00000, 0xEAFFFFFE]         # nop; b .
    header = [0x474B3731, 0x304308EF, len(code), 0xC0800000, 0x30200000,
              0xC1000000, 0x5A5A5A5A,
              0xFFFFFFFF, 0x180,                    # a delay
              DDRC + 4, 0x00100344,                 # a register write
              0xFFFFFFFE, DDRC + 0x14, 0,           # a poll
              0]
    img = bytearray(b'\xFF' * (1 << 20))
    img[:4 * (len(header) + len(code))] = b''.join(
        struct.pack('>I', w) for w in header + code)
    open(path, 'wb').write(img)
    return code


def test_boot_rom(qt, code):
    print('boot ROM')
    check('write after the delay entry applied', qt.readl(DDRC + 4),
          0x00100344)
    check('U-Boot word 0 at load address', qt.readl(0xC0800000), code[0])
    check('U-Boot word 1 at load address', qt.readl(0xC0800004), code[1])


def test_timer_vic(qt):
    print('gk-remap: gk-timer -> gk-vic, S layout')
    bit = 1 << TIMER3_IRQ
    qt.writel(VIC1 + VIC_SENSE, 0)              # edge
    qt.writel(VIC1 + VIC_EVENT, bit)            # rising
    qt.writel(VIC1 + VIC_EDGE_CLR, 0xFFFFFFFF)
    qt.writel(VIC1 + VIC_INTEN, bit)
    qt.writel(TIMER + 0x20, 690)                # T3 STATUS: 10 us at 69 MHz
    qt.writel(TIMER + 0x38, 69000)              # T3 RELOAD: 1 ms
    qt.writel(TIMER + 0x0C, 0x100 | 0x10)       # EN3 | OF3 (S timer.h)
    check('no interrupt before underflow', qt.readl(VIC1 + VIC_IRQ_STA), 0)
    qt.step(20000)
    check('TIMER3_IRQ pending', qt.readl(VIC1 + VIC_IRQ_STA), bit)
    check('reloaded from RELOAD', 0 < qt.readl(TIMER + 0x20) <= 69000, True)
    check('control reads back in S bits', qt.readl(TIMER + 0x0C), 0x110)
    qt.writel(VIC1 + VIC_EDGE_CLR, bit)
    check('EDGE_CLR clears it', qt.readl(VIC1 + VIC_IRQ_STA), 0)
    qt.writel(TIMER + 0x0C, 0x100 | 0x01)       # EN3 | CSL3: no interrupt
    qt.step(2000000)
    check('clock select alone raises nothing', qt.readl(VIC1 + VIC_IRQ_STA),
          0)
    qt.writel(TIMER + 0x0C, 0)


def test_wdt(qt):
    print('gk-remap: gk-wdt, S layout')
    qt.writel(WDT + 0x10, 6900)                 # RELOAD: 100 us
    qt.writel(WDT + 0x00, 0x2 | 0x1)            # INT_EN | EN
    qt.writel(WDT + 0x14, 0x4755)               # RESTART
    check('STATUS counting', 0 < qt.readl(WDT + 0x0C) <= 6900, True)
    qt.step(200000)
    check('TIMEOUT after the count', qt.readl(WDT + 0x04), 1)
    qt.writel(WDT + 0x08, 1)                    # CLR_TMO
    check('CLR_TMO', qt.readl(WDT + 0x04), 0)
    qt.writel(WDT + 0x00, 0)


def idc_ctrl(qt, val):
    qt.writel(IDC + 0x0C, val)
    return qt.readl(IDC + 0x0C)


def test_idc(qt):
    print('gk-remap: gk-idc + sensor=jxh42, S layout')
    STOP, START, ACK, IF = 1, 2, 4, 8           # S idc.h
    qt.writel(IDC + 0x14, 1)                    # ENR
    qt.writel(IDC + 0x18, 0x30 << 1)            # DATA: address, write
    check('address acknowledged', idc_ctrl(qt, START), IF | ACK)
    qt.writel(IDC + 0x18, 0x0A)                 # register: PIDH
    check('register byte acknowledged', idc_ctrl(qt, 0), IF | ACK)
    qt.writel(IDC + 0x18, (0x30 << 1) | 1)      # repeated start, read
    check('read address acknowledged', idc_ctrl(qt, START), IF | ACK)
    check('STS: master receiver', qt.readl(IDC + 0x10) & 1, 1)
    idc_ctrl(qt, ACK)                           # last byte: NACK it
    check('JX-H42 PIDH', qt.readl(IDC + 0x18), 0xA0)
    idc_ctrl(qt, STOP)


def test_dma(qt):
    print('gk-dma')
    desc, src, dst, rpt = RAM, RAM + 0x1000, RAM + 0x2000, RAM + 0x3000
    for i in range(8):
        qt.writel(src + 4 * i, 0x11111111 * (i + 1))
        qt.writel(dst + 4 * i, 0)
    # src, dst, next, rpt, count, attr: EOC | WM | RM | 4-byte units | ID
    for i, w in enumerate([src, dst, desc + 0x18, rpt, 0x20,
                           (1 << 24) | (1 << 23) | (1 << 22) | (2 << 19) |
                           0x7]):
        qt.writel(desc + 4 * i, w)
    qt.writel(DMAC + 0x30C, desc)               # DA(1)
    qt.writel(DMAC + 0x32C, 0xC0000000)         # CTR(1): EN | D
    check('DA moved on to next', qt.readl(DMAC + 0x30C), desc + 0x18)
    check('data copied', [qt.readl(dst + 4 * i) for i in range(8)] ==
          [0x11111111 * (i + 1) for i in range(8)], True)
    check('report written', qt.readl(rpt) & (1 << 22), 1 << 22)
    check('EN dropped', qt.readl(DMAC + 0x32C) & 0x80000000, 0)
    check('INT: channel 1', qt.readl(DMAC + 0x304), 0x2)
    qt.writel(DMAC + 0x338, 0)                  # STA(1)
    check('STA cleared, INT drops', qt.readl(DMAC + 0x304), 0)


def test_rejection(qemu, path):
    print('rejections')
    cpu = path + '.cpu'
    img = open(path, 'rb').read()
    open(cpu, 'wb').write(b''.join(img[i:i + 4][::-1]
                                   for i in range(0, len(img), 4)))
    r = subprocess.run([qemu, '-M', 'gk7102s,flash-file=' + cpu,
                        '-display', 'none', '-nodefaults'],
                       capture_output=True, text=True, timeout=20)
    check('CPU-order image refused with a reason',
          'CPU byte order' in r.stderr, True)


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    ap.add_argument('--qemu', default=os.path.join(
        root, 'qemu-src/build/qemu-system-arm'))
    args = ap.parse_args()

    path = os.path.join(tempfile.mkdtemp(), 'flash.bin')
    code = make_image(path)
    qt = QTest(args.qemu, 'gk7102s,flash-file=%s,sensor=jxh42' % path)
    try:
        test_boot_rom(qt, code)
        test_timer_vic(qt)
        test_wdt(qt)
        test_idc(qt)
        test_dma(qt)
    finally:
        qt.close()
    test_rejection(args.qemu, path)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
