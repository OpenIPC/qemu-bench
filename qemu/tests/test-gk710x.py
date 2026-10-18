#!/usr/bin/env python3
"""Register-level checks of the Goke GK7101/GK7102 models, no guest needed.

QEMU runs the gk7102 machine under qtest and this script plays the vendor
drivers, at the silicon addresses (docs/goke-gk710x.md):

  - the boot ROM: a synthetic flash image's header register writes land and
    its U-Boot is staged at the load address, in CPU byte order;
  - gk-sflash: JEDEC ID, reads packed MSB first, erase and page program
    written back to the image file, the write-enable latch;
  - gk-timer into gk-vic: an underflow latches a rising edge on the timer's
    VIC input until EDGECLR;
  - gk-idc: start / data / repeated start / read against sensor=jxh42, and
    an unanswered address raising nothing more after the handler's CTRL=0;
  - gk-wdt: the count runs out into TIMEOUT.

    python3 qemu/tests/test-gk710x.py [--qemu PATH]
"""
import argparse
import os
import socket
import struct
import subprocess
import sys
import tempfile

SFLASH = 0x60016000
VIC1 = 0x60003000
TIMER = 0x7000B000
WDT = 0x7000C000
IDC = 0x70003000
DDRC = 0x60004000

# gk-sflash COMMAND fields (U-Boot include/gk7101_sflash.h)
SEND_CMD = 1 << 8
ADDR3 = 3 << 9
RWN_WRITE = 1 << 15
RWN_NONE = 3 << 15
def NBYTES(n): return (n & 0x1F) << 23
WORD = NBYTES(0x1F)

# The HAL's VIC wiring: kernel TIMER3_IRQ (20) is silicon input 14.
TIMER3_LINE = 14


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


def be_words(words):
    """CPU words as the flash stores them: each one byte-swapped."""
    return b''.join(struct.pack('>I', w) for w in words)


def make_image(path):
    """1 MiB: a boot header writing one DDRC register, a two-word "U-Boot",
    and a recognisable pattern from 0x10000 on."""
    code = [0xE1A00000, 0xEAFFFFFE]         # nop; b .
    header = [0x474B3731, 0x30310801, len(code), 0xC0800000, 0, 0,
              0x5A5A5A5A,
              DDRC + 4, 0x00100344,                 # a register write
              0xFFFFFFFE, DDRC + 0x14, 0,           # a poll, skipped
              0]
    img = bytearray(b'\xFF' * (1 << 20))
    img[:4 * (len(header) + len(code))] = be_words(header + code)
    for i in range(0x10000, 0x20000):
        img[i] = i * 7 & 0xFF
    open(path, 'wb').write(img)
    return code


def test_boot_rom(qt, code):
    print('boot ROM')
    check('header register write applied', qt.readl(DDRC + 4), 0x00100344)
    check('U-Boot word 0 at load address', qt.readl(0xC0800000), code[0])
    check('U-Boot word 1 at load address', qt.readl(0xC0800004), code[1])


def sf(qt, cmd, data=None):
    qt.writel(SFLASH + 4, cmd)
    if data is not None:
        qt.writel(SFLASH, data)


def test_sflash(qt, path):
    print('gk-sflash')
    sf(qt, 0x9F | SEND_CMD | NBYTES(3))
    check('RDID: GD25Q of 1 MiB', qt.readl(SFLASH), 0xC84014)
    sf(qt, 0x9F | SEND_CMD | NBYTES(1))
    check('RDID, 1 byte: manufacturer first', qt.readl(SFLASH), 0xC8)
    sf(qt, 0x9F | SEND_CMD | NBYTES(2))
    check('RDID, 2 bytes', qt.readl(SFLASH), 0xC840)

    img = open(path, 'rb').read()
    sf(qt, 0x03 | SEND_CMD | ADDR3 | WORD, 0x10010)
    check('read packs MSB first', qt.readl(SFLASH),
          int.from_bytes(img[0x10010:0x10014], 'big'))
    check('read streams on', qt.readl(SFLASH),
          int.from_bytes(img[0x10014:0x10018], 'big'))

    sf(qt, 0x20 | SEND_CMD | ADDR3 | RWN_NONE, 0x10000)      # no WREN
    check('erase without WREN ignored', open(path, 'rb').read()[0x10010],
          img[0x10010])

    sf(qt, 0xC5 | SEND_CMD | RWN_WRITE | NBYTES(1), 1)      # no WREN
    sf(qt, 0xC8 | SEND_CMD | NBYTES(1))
    check('WREAR without WREN ignored', qt.readl(SFLASH), 0)

    sf(qt, 0x06 | SEND_CMD | RWN_NONE)
    sf(qt, 0x05 | SEND_CMD | NBYTES(1))
    check('WREN sets WEL', qt.readl(SFLASH) & 2, 2)
    sf(qt, 0x20 | SEND_CMD | ADDR3 | RWN_NONE, 0x10000)
    after = open(path, 'rb').read()
    check('4 KiB erase written back',
          after[0x10000:0x11000] == b'\xFF' * 0x1000, True)
    check('next sector untouched', after[0x11000], img[0x11000])

    sf(qt, 0x06 | SEND_CMD | RWN_NONE)
    sf(qt, 0x02 | SEND_CMD | ADDR3 | RWN_WRITE | WORD, 0x10100)
    qt.writel(SFLASH, 0x11223344)
    qt.writel(SFLASH, 0x55667788)
    sf(qt, 0x05 | SEND_CMD | NBYTES(1))
    check('program ends, WEL drops', qt.readl(SFLASH) & 2, 0)
    check('program written back MSB first',
          open(path, 'rb').read()[0x10100:0x10108].hex(), '1122334455667788')


def test_timer_vic(qt):
    print('gk-timer -> gk-vic')
    bit = 1 << TIMER3_LINE
    qt.writel(VIC1 + 0x24, 0)                   # SENSE: edge
    qt.writel(VIC1 + 0x2C, bit)                 # EVENT: rising
    qt.writel(VIC1 + 0x38, 0xFFFFFFFF)          # EDGECLR
    qt.writel(VIC1 + 0x10, bit)                 # INTENABLE
    qt.writel(TIMER + 0x20, 690)                # T3 STATUS: 10 us at 69 MHz
    qt.writel(TIMER + 0x24, 69000)              # T3 RELOAD: 1 ms
    qt.writel(TIMER + 0x30, 0x5 << 8)           # T3 enable | overflow irq
    check('no edge before underflow', qt.readl(VIC1) & bit, 0)
    qt.step(20000)
    check('underflow latched the edge', qt.readl(VIC1) & bit, bit)
    count = qt.readl(TIMER + 0x20)
    check('reloaded from RELOAD', 0 < count <= 69000, True)
    qt.step(1000)
    check('edge stays latched (level gone)', qt.readl(VIC1) & bit, bit)
    qt.writel(VIC1 + 0x38, bit)
    check('EDGECLR clears it', qt.readl(VIC1) & bit, 0)
    qt.writel(TIMER + 0x30, 0)


def idc_ctrl(qt, val):
    qt.writel(IDC + 4, val)
    return qt.readl(IDC + 4)


def test_idc(qt):
    print('gk-idc + sensor=jxh42')
    ACK, IF, START, STOP = 1, 2, 4, 8
    qt.writel(IDC, 1)                           # ENR
    qt.writel(IDC + 8, 0x30 << 1)               # address, write
    check('address acknowledged', idc_ctrl(qt, START), IF | ACK)
    qt.writel(IDC + 8, 0x0A)                    # register: PIDH
    check('register byte acknowledged', idc_ctrl(qt, 0), IF | ACK)
    qt.writel(IDC + 8, (0x30 << 1) | 1)         # repeated start, read
    check('read address acknowledged', idc_ctrl(qt, START), IF | ACK)
    check('STS: master receiver', qt.readl(IDC + 0xC) & 1, 1)
    idc_ctrl(qt, ACK)                           # last byte: NACK it
    check('JX-H42 PIDH', qt.readl(IDC + 8), 0xA0)
    idc_ctrl(qt, STOP)

    qt.writel(IDC + 8, 0x36 << 1)               # nobody at 0x36
    check('unanswered address: IF, no ACK', idc_ctrl(qt, START), IF)
    check('handler CTRL=0 raises nothing', idc_ctrl(qt, 0), 0)
    idc_ctrl(qt, STOP | ACK)


def test_wdt(qt):
    print('gk-wdt')
    qt.writel(WDT + 0x04, 6900)                 # RELOAD: 100 us
    qt.writel(WDT + 0x0C, 0x5)                  # interrupt | enable
    qt.writel(WDT + 0x08, 0x4755)               # RESTART
    check('counting', 0 < qt.readl(WDT) <= 6900, True)
    qt.step(200000)
    check('TIMEOUT after the count', qt.readl(WDT + 0x10), 1)
    qt.writel(WDT + 0x14, 1)
    check('CLR_TMO', qt.readl(WDT + 0x10), 0)
    qt.writel(WDT + 0x0C, 0)


def test_rejections(qemu, path):
    """Images and command lines the machine must refuse, with a reason."""
    print('rejections')
    bad = path + '.short'
    img = bytearray(open(path, 'rb').read()[:0x10000])
    # The write list's last pair cut in half by the end of the image.
    img[:0x10000] = be_words([0x474B3731, 0x30310801, 1, 0xC0800000, 0, 0,
                              0x5A5A5A5A] + [DDRC, 0] * 8188 + [DDRC])
    assert len(img) == 0x10000
    open(bad, 'wb').write(img)
    r = subprocess.run([qemu, '-M', 'gk7102,flash-file=' + bad,
                        '-display', 'none', '-nodefaults'],
                       capture_output=True, text=True, timeout=20)
    check('truncated boot header refused',
          'runs past the image' in r.stderr, True)
    r = subprocess.run([qemu, '-M', 'gk7102', '-display', 'none',
                        '-nodefaults', '-kernel', '/dev/null',
                        '-append', 'mem=100M'],
                       capture_output=True, text=True, timeout=20)
    check('mem= beyond DDR refused', 'does not fit' in r.stderr, True)


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    ap.add_argument('--qemu', default=os.path.join(
        root, 'qemu-src/build/qemu-system-arm'))
    args = ap.parse_args()

    path = os.path.join(tempfile.mkdtemp(), 'flash.bin')
    code = make_image(path)
    qt = QTest(args.qemu, 'gk7102,flash-file=%s,sensor=jxh42' % path)
    try:
        test_boot_rom(qt, code)
        test_sflash(qt, path)
        test_timer_vic(qt)
        test_idc(qt)
        test_wdt(qt)
    finally:
        qt.close()
    test_rejections(args.qemu, path)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
