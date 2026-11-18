#!/usr/bin/env python3
"""The Xiongmai machines' sensor= and sensor-addr= properties and the generic
xm-sensor model, driven through xm-i2c the way xm_i2c.ko does: the register
address goes out as a write, the data comes back as a read.

No guest: QEMU runs under qtest and this script plays the driver.  It checks
that a preset chip ID reads back at both register widths, that a write lands
in the register file and in the log, that sensor=none leaves the bus empty
(the read is NACKed and never completes), and that a type which is not an
I2C device is refused.

    python3 qemu/tests/test-xm-sensor.py [--qemu PATH]
"""
import argparse, os, shutil, socket, subprocess, sys, tempfile

I2C = 0x10000000
CTRL, ADDR, TX, RX, WCNT, RCNT, CLR, ST = 0x00, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x44, 0x48
START_WR, START_RD = 0x00B, 0x803
WR_DONE, RD_DONE = 1 << 4, 1 << 5

failures = 0


def check(ok, what):
    global failures
    print(('ok   ' if ok else 'FAIL ') + what)
    failures += not ok


class QTest:
    def __init__(self, qemu, machine, extra=()):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        srv = socket.socket(socket.AF_UNIX); srv.bind(path); srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest', '-display', 'none',
             '-nodefaults', '-qtest', 'unix:' + path, '-qtest-log', '/dev/null',
             '-serial', 'null', *extra],
            stdout=subprocess.DEVNULL,
            stderr=open(os.path.join(self.dir, 'qemu.err'), 'w'))
        srv.settimeout(20)
        self.sock, _ = srv.accept()
        self.f = self.sock.makefile('rw', buffering=1)

    def cmd(self, line):
        self.f.write(line + '\n'); self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith(('OK', 'FAIL', 'ERR')):
                return r

    def writel(self, a, v): self.cmd('writel 0x%x 0x%x' % (a, v))
    def readl(self, a): return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def close(self, expect=None):
        """Stops QEMU. Anything it printed on stderr fails the test (an
        assert, an MMIO overlap, a bad IRQ, a core dump), unless it is the
        one message expected."""
        self.proc.kill(); self.proc.wait()
        err = open(os.path.join(self.dir, 'qemu.err')).read().strip()
        shutil.rmtree(self.dir, ignore_errors=True)
        if expect:
            check(expect in err and len(err.splitlines()) == 1,
                  'QEMU stderr has only: %s (%r)' % (expect, err))
        else:
            check(not err, 'QEMU stderr is empty' + ('' if not err else ':\n' + err))

    # xm_i2c.ko's two transfers
    def send(self, dev, data):
        self.writel(I2C + CLR, WR_DONE | RD_DONE)
        self.writel(I2C + ADDR, dev)
        for b in data:
            self.writel(I2C + TX, b)
        self.writel(I2C + WCNT, len(data))
        self.writel(I2C + CTRL, START_WR)
        return bool(self.readl(I2C + ST) & WR_DONE)

    def recv(self, dev, n):
        self.writel(I2C + CLR, WR_DONE | RD_DONE)
        self.writel(I2C + ADDR, dev)
        self.writel(I2C + RCNT, n)
        self.writel(I2C + CTRL, START_RD)
        if not self.readl(I2C + ST) & RD_DONE:
            return None
        return [self.readl(I2C + RX) for _ in range(n)]


def reg(r, width):
    return list(r.to_bytes(width, 'big'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='./qemu-src/build/qemu-system-arm')
    qemu = ap.parse_args().qemu
    tmp = tempfile.mkdtemp()
    log = os.path.join(tmp, 'sensor.log')

    # SmartSens layout: 16-bit registers, ID at 0x3107/0x3108, on XM530.
    q = QTest(qemu, 'xm530,sensor=xm-sensor,sensor-addr=0x30',
              ['-global', 'xm-sensor.addr-bytes=2',
               '-global', 'xm-sensor.regs=0x3107:0x22,0x3108:0x35',
               '-global', 'xm-sensor.log=' + log])
    try:
        check(q.send(0x30, reg(0x3107, 2)), '16-bit: register address sent')
        check(q.recv(0x30, 2) == [0x22, 0x35], '16-bit: chip ID 0x2235 reads back')
        check(q.send(0x30, reg(0x3e01, 2) + [0x5a]), '16-bit: register write')
        q.send(0x30, reg(0x3e01, 2))
        check(q.recv(0x30, 1) == [0x5a], '16-bit: the write reads back')
        check(q.recv(0x31, 1) is None, 'nothing answers at another address')
    finally:
        q.close()
    lines = open(log).read().split('\n')
    check(lines[:3] == ['R 3107 22', 'R 3108 35', 'W 3e01 5a'],
          'log records reads and writes: %r' % lines[:3])

    # SOI layout: 8-bit registers, the xm510 default address.
    q = QTest(qemu, 'xm510,sensor=xm-sensor',
              ['-global', 'xm-sensor.regs=0x0a:0xa0,0x0b:0x62'])
    try:
        q.send(0x30, [0x0a])
        check(q.recv(0x30, 2) == [0xa0, 0x62], '8-bit: chip ID 0xa062 at 0x30')
    finally:
        q.close()

    # The SoC's own sensor is still there by default, and none removes it.
    q = QTest(qemu, 'xm510')
    try:
        q.send(0x30, [0x0a])
        check(q.recv(0x30, 1) is not None, 'default: the JX-H62 answers at 0x30')
    finally:
        q.close()
    q = QTest(qemu, 'xm510,sensor=none')
    try:
        check(not q.send(0x30, [0x0a]), 'sensor=none: the write is NACKed')
        check(q.recv(0x30, 1) is None, 'sensor=none: the read never completes')
    finally:
        q.close()

    r = subprocess.run([qemu, '-M', 'xm510,sensor=pl011', '-display', 'none',
                        '-nodefaults', '-machine', 'accel=qtest', '-qtest',
                        'stdio'], input='', capture_output=True, text=True,
                       timeout=20)
    check(r.returncode != 0 and 'not an I2C device model' in r.stderr,
          'a non-I2C type is refused')
    r = subprocess.run([qemu, '-M', 'xm510,sensor=xm-sensor', '-display', 'none',
                        '-nodefaults', '-machine', 'accel=qtest', '-qtest',
                        'stdio', '-global', 'xm-sensor.regs=0x10000:1',
                       '-global', 'xm-sensor.addr-bytes=2'],
                       input='', capture_output=True, text=True, timeout=20)
    check(r.returncode != 0 and 'bad regs entry' in r.stderr,
          'a register outside the file is refused')
    r = subprocess.run([qemu, '-M', 'xm510,sensor=xm-sensor', '-display', 'none',
                        '-nodefaults', '-machine', 'accel=qtest', '-qtest',
                        'stdio', '-global', 'xm-sensor.regs=0x3107:0x22'],
                       input='', capture_output=True, text=True, timeout=20)
    check(r.returncode != 0 and 'bad regs entry' in r.stderr,
          'a 16-bit register with 8-bit addressing is refused')
    r = subprocess.run([qemu, '-M', 'xm510,sensor=xm-sensor,sensor-addr=0',
                        '-display', 'none', '-nodefaults', '-machine',
                        'accel=qtest', '-qtest', 'stdio'],
                       input='', capture_output=True, text=True, timeout=20)
    check(r.returncode != 0 and '1-0x7f' in r.stderr,
          'sensor-addr=0, the general call address, is refused')
    q = QTest(qemu, 'xm510,sensor=xm-sensor,sensor-addr=0x36',
              ['-global', 'xm-sensor.regs=0x0a:0x5a'])
    try:
        q.send(0x36, [0x0a])
        check(q.recv(0x36, 1) == [0x5a], 'sensor-addr=0x36 moves the sensor')
        check(not q.send(0x30, [0x0a]), 'nothing left at the default 0x30')
    finally:
        q.close()

    # A log that can't be written: reported once, and the sensor works on.
    q = QTest(qemu, 'xm510,sensor=xm-sensor',
              ['-global', 'xm-sensor.regs=0x0a:0xa0',
               '-global', 'xm-sensor.log=/dev/full'])
    try:
        for _ in range(3):
            q.send(0x30, [0x0a])
            check(q.recv(0x30, 1) == [0xa0], 'log on a full disk: the ID still reads')
    finally:
        q.close(expect='no more log')

    shutil.rmtree(tmp, ignore_errors=True)
    print('%d failed' % failures if failures else 'all passed')
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
