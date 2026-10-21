#!/usr/bin/env python3
"""Register-level checks of the Anyka AK39xx models, no guest needed.

QEMU runs the ak3918ev300 (and ak3919ev330) machine under qtest and this
script plays the vendor drivers (docs/anyka.md):

  - the mask ROM: a synthetic boot header's register script and payload are
    staged where the ROM puts them (stub + table at 0, U-Boot at its load
    address, the header behind it and in L2 memory at 0x48000200);
  - ak-sysctrl: chip ID, a one-shot timer expiring into the timer's
    sub-interrupt and the cascaded module interrupt, the RTC link;
  - ak-uart: a byte put in L2 memory and started from CONF2 reaches the
    serial backend, TX-done reads as idle;
  - ak-spi + ak-spinor: JEDEC ID by PIO, a fast read whose dummy byte is a
    plain TX byte, a 256-byte read streamed through L2 buffer 1 by DMA,
    sector erase and page program written back to the image;
  - ak-gpio pull-ups, the DDRC init-done flag, both I2C layouts against an
    empty bus;
  - the GMAC (xm-dwmac): the PHY over MDIO, and a TX ring of the driver's
    24-byte descriptors (descriptor skip length 2) walked to its end;
  - ak-mci with no card: a command times out, interrupting through MASK,
    and reading STATUS clears it;
  - a full-duplex PIO transfer that fills the RX FIFO loses no TX byte, and
    an L2 read whose DMA ends in a short fraction drains the buffer;
  - outside qtest: a flash image smaller than an erase block is refused,
    machine state migrates to a file and back on five- and ten-timer
    parts, and a file-backed flash blocks migration.

    python3 qemu/tests/test-anyka.py [--qemu PATH]
"""
import argparse
import atexit
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

SYSCTRL = 0x08000000
SPI0 = 0x20120000
UART0 = 0x20130000
L2 = 0x20140000
L2MEM = 0x48000000
I2C0 = 0x20150000
GPIO = 0x20170000
DDRC = 0x21000000
DDR = 0x80000000
MAC = 0x20300000
MCI0 = 0x20100000

LOAD = 0x80E00000
JEDEC = 0xC84017                    # GD25Q64, as the SPIP descriptor says


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


def shown(v):
    if isinstance(v, (bool, str)):
        return v
    if isinstance(v, bytes):
        return v[:8].hex() + ('...' if len(v) > 8 else '')
    return hex(v)


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown(got)))


def make_image(path, magic=b'ANYKA_H3'):
    """8 MiB: a boot header with a two-pair register script and an SPIP
    descriptor, a two-word "U-Boot" after it, a pattern from 0x10000 on."""
    code = [0xE1A00000, 0xEAFFFFFE]         # nop; b .
    img = bytearray(b'\xFF' * (8 << 20))
    hdr = bytearray(0x200)
    struct.pack_into('<I', hdr, 0, 0xEA00007E)
    hdr[4:12] = magic
    struct.pack_into('<I', hdr, 0x0C, 4 * len(code))
    struct.pack_into('<I', hdr, 0x20, LOAD)
    script = [DDRC + 0x04, 0x0D493315,      # a DDR timing register
              0x66668888, 0xC8,             # a delay, skipped
              DDRC + 0x0C, 0x00000A8B,
              0x88888888, 0x88888888]
    struct.pack_into('<%dI' % len(script), hdr, 0x2C, *script)
    hdr[0x18C:0x190] = b'SPIP'
    struct.pack_into('<I', hdr, 0x190, 0x001740C8)
    img[:0x200] = hdr
    struct.pack_into('<%dI' % len(code), img, 0x200, *code)
    for i in range(0x10000, 0x20000):
        img[i] = i * 7 & 0xFF
    open(path, 'wb').write(img)
    return code


def test_boot_rom(qt, code):
    print('mask ROM')
    stub = [0xE28F2010, 0xE8B20003, 0xE3500000, 0x15801000, 0x1AFFFFFB,
            0xE592F000]
    check('ROM stub at 0', [qt.readl(4 * i) for i in range(6)] == stub, True)
    table = [qt.readl(0x18 + 4 * i) for i in range(7)]
    check('script table, delay dropped, then {0, 0} and the entry',
          table == [DDRC + 0x04, 0x0D493315, DDRC + 0x0C, 0x00000A8B,
                    0, 0, LOAD], True)
    check('U-Boot word 0 at the load address', qt.readl(LOAD), code[0])
    check('U-Boot word 1', qt.readl(LOAD + 4), code[1])
    check('header in front of U-Boot (TEXT_BASE)', qt.readl(LOAD - 0x200 + 4),
          0x4B594E41)
    check('header left in L2 memory: SPIP at 0x4800038C',
          qt.readl(L2MEM + 0x200 + 0x18C), 0x50495053)


def test_sysctrl(qt):
    print('ak-sysctrl')
    check('chip ID', qt.readl(SYSCTRL), 0x20160100)
    # Timer 1 (CTRL1/2 at 0xB4/0xB8) one-shot, 1 ms; its sub-interrupt is
    # bit 5, the sub-controller is module input 3.
    qt.writel(SYSCTRL + 0x2C, 1 << 5)
    qt.writel(SYSCTRL + 0x24, 1 << 3)
    qt.writel(SYSCTRL + 0xB4, 12000)
    qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24) | (1 << 29))
    check('not expired yet', (qt.readl(SYSCTRL + 0xB8) >> 27) & 1, 0)
    qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24) | (1 << 26))
    live = qt.readl(SYSCTRL + 0xB4)
    check('READ_SEL: CTRL1 is the live count', 0 < live <= 12000, True)
    qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24))
    qt.step(2000000)
    check('expired flag', (qt.readl(SYSCTRL + 0xB8) >> 27) & 1, 1)
    check('timer 1 sub-interrupt', qt.readl(SYSCTRL + 0x30) & (1 << 5),
          1 << 5)
    check('module interrupt 3', qt.readl(SYSCTRL + 0x4C) & (1 << 3), 1 << 3)
    qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24) | (1 << 30))
    check('CLEAR drops it', qt.readl(SYSCTRL + 0x4C) & (1 << 3), 0)
    # RTC: write register 3, read it back through the serial link.
    qt.writel(SYSCTRL + 0x50, (1 << 21) | (2 << 18) | (3 << 14) | 0x1234)
    qt.writel(SYSCTRL + 0x50, (1 << 21) | (2 << 18) | (1 << 17) | (3 << 14))
    check('RTC register round trip', qt.readl(SYSCTRL + 0x54) & 0x3FFF,
          0x1234)
    check('RTC ready', (qt.readl(SYSCTRL + 0x30) >> 7) & 1, 1)


def test_uart(qt, out):
    print('ak-uart')
    qt.writel(L2MEM + 0x1000, ord('K'))
    qt.writel(UART0 + 0x00, 1 << 28)                # clear the TX state
    qt.writel(UART0 + 0x04, (1 << 4) | (1 << 16))   # 1 byte, go
    check('TX done reads as idle', (qt.readl(UART0 + 0x04) >> 19) & 1, 1)
    time.sleep(0.2)
    check('byte reached the backend', open(out, 'rb').read(), b'K')


def spi_tx(qt, data):
    """The drivers' PIO write: ARRM, CNT, then OUT words while half empty."""
    qt.writel(SPI0, 0x272)
    qt.writel(SPI0 + 0x0C, len(data))
    for i in range(0, len(data), 4):
        qt.writel(SPI0 + 0x18, int.from_bytes(data[i:i + 4], 'little'))
    return qt.readl(SPI0 + 0x04) & (1 << 8)


def spi_rx(qt, n):
    qt.writel(SPI0, 0x271)
    qt.writel(SPI0 + 0x0C, n)
    got = b''
    while len(got) < n:
        got += qt.readl(SPI0 + 0x1C).to_bytes(4, 'little')
    return got[:n]


def spi_end(qt):
    qt.writel(SPI0, 0x250)


def test_spi(qt, path):
    print('ak-spi + ak-spinor')
    spi_tx(qt, b'\x9F')
    check('RDID', int.from_bytes(spi_rx(qt, 3), 'big'), JEDEC)
    spi_end(qt)
    img = open(path, 'rb').read()
    check('fast read: TRANSF after the command', spi_tx(
        qt, b'\x0B\x01\x00\x10\x00'), 1 << 8)
    check('fast read, dummy as a TX byte', spi_rx(qt, 12),
          img[0x10010:0x1001C])
    spi_end(qt)

    # 256 bytes through L2 buffer 1: buffer and its DMA on, SPI0 RX
    # (device 7) assigned to it, then the transfer and a 4-block DMA.
    qt.writel(L2 + 0x88, (1 << 1) | (1 << 17))
    qt.writel(L2 + 0x90, 1 << 21)
    qt.writel(L2 + 0x88, (1 << 1) | (1 << 17) | (1 << 25))
    spi_tx(qt, b'\x03\x01\x80\x00')
    qt.writel(SPI0, 0x271)
    qt.writel(SPI0 + 0x14, 0x10001)
    qt.writel(SPI0 + 0x0C, 256)
    check('L2 buffer full, SPI stalled', (qt.readl(L2 + 0xA0) >> 4) & 0xF,
          4)
    qt.writel(L2 + 0x04, DDR + 0x100000)
    qt.writel(L2 + 0x44, 4)
    qt.writel(L2 + 0x80, (1 << 25) | 1)
    check('DMA done', qt.readl(L2 + 0x80) & (1 << 25), 0)
    check('SPI count drained', qt.readl(SPI0 + 0x0C), 0)
    got = b''.join(qt.readl(DDR + 0x100000 + i).to_bytes(4, 'little')
                   for i in range(0, 256, 4))
    check('256 bytes via L2 DMA', got, img[0x18000:0x18100])
    qt.writel(SPI0 + 0x14, 0)
    qt.writel(L2 + 0x88, 0)
    spi_end(qt)

    # Erase the 4 KiB sector at 0x11000, program 4 bytes into it.
    spi_tx(qt, b'\x06')
    spi_end(qt)
    spi_tx(qt, b'\x20\x01\x10\x00')
    spi_end(qt)
    spi_tx(qt, b'\x06')
    spi_end(qt)
    spi_tx(qt, b'\x02\x01\x10\x00' + b'\x11\x22\x33\x44')
    spi_end(qt)
    spi_tx(qt, b'\x05')
    check('WEL cleared after the program', spi_rx(qt, 1)[0] & 2, 0)
    spi_end(qt)
    img = open(path, 'rb').read()
    check('programmed bytes in the image file', img[0x11000:0x11004],
          b'\x11\x22\x33\x44')
    check('rest of the sector erased', img[0x11004:0x12000],
          b'\xFF' * 0xFFC)
    check('next sector untouched', img[0x12000:0x12004],
          bytes(i * 7 & 0xFF for i in range(0x12000, 0x12004)))


def test_spi_more(qt, path):
    print('ak-spi: duplex with a full RX FIFO, L2 fraction tail')
    img = open(path, 'rb').read()
    # RDID shifted out with 11 filler bytes, both directions, OUT written
    # three words ahead of IN: the third waits for FIFO room, intact.
    qt.writel(SPI0, 0x270)
    qt.writel(SPI0 + 0x0C, 12)
    for w in (0x9F, 0, 0):
        qt.writel(SPI0 + 0x18, w)
    got = b''.join(qt.readl(SPI0 + 0x1C).to_bytes(4, 'little')
                   for _ in range(3))
    check('12 bytes in step with the 12 sent', got,
          b'\xFF' + bytes.fromhex('c84017') * 3 + bytes.fromhex('c840'))
    spi_end(qt)

    # 100 bytes: one 64-byte block by DMA, the 36-byte tail by fraction
    # DMA out of block 1 of buffer 1.
    dst = DDR + 0x300000
    qt.writel(L2 + 0x88, (1 << 1) | (1 << 17))
    qt.writel(L2 + 0x90, 1 << 21)
    qt.writel(L2 + 0x88, (1 << 1) | (1 << 17) | (1 << 25))
    spi_tx(qt, b'\x03\x01\x90\x00')
    qt.writel(SPI0, 0x271)
    qt.writel(SPI0 + 0x14, 0x10001)
    qt.writel(SPI0 + 0x0C, 100)
    qt.writel(L2 + 0x04, dst)
    qt.writel(L2 + 0x44, 1)
    qt.writel(L2 + 0x80, (1 << 25) | 1)
    qt.writel(L2 + 0x84, (3 << 28) | ((dst + 64) & 0x0FFFFFFF))
    qt.writel(L2 + 0x80, 1 | (1 << 9) | (35 << 10) | (((1 << 3) | 1) << 1))
    check('fraction DMA done', qt.readl(L2 + 0x80) & (1 << 9), 0)
    check('buffer drained', (qt.readl(L2 + 0xA0) >> 4) & 0xF, 0)
    got = b''.join(qt.readl(dst + i).to_bytes(4, 'little')
                   for i in range(0, 100, 4))
    check('100 bytes via block + fraction DMA', got, img[0x19000:0x19064])
    qt.writel(SPI0 + 0x14, 0)
    qt.writel(L2 + 0x88, 0)
    spi_end(qt)


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


def test_rejections_and_migration(qemu, tmp, path):
    print('flash size, migration')
    small = os.path.join(tmp, 'small.bin')
    open(small, 'wb').write(b'\xFF' * 0x8000)
    out = subprocess.run([qemu, '-M', 'ak3918ev300,flash-file=' + small,
                          '-display', 'none', '-nodefaults', '-S'],
                         capture_output=True, text=True, timeout=30)
    check('a 32 KiB flash image is refused',
          out.returncode != 0 and '64 KiB' in out.stderr, True)
    for m in ('ak3918ev300', 'ak3919ev330'):
        mig = os.path.join(tmp, m + '.mig')
        out = hmp(qemu, ['-M', m, '-S'],
                  ['migrate file:' + mig, 'info migrate'])
        check('%s: state saved' % m, 'completed' in out and
              'Assertion' not in out and os.path.getsize(mig) > 0, True)
        out = hmp(qemu, ['-M', m, '-S', '-incoming', 'file:' + mig],
                  ['info status'])
        check('%s: and loaded back' % m, 'error' not in out.lower() and
              'Assertion' not in out, True)
    out = hmp(qemu, ['-M', 'ak3918ev300,flash-file=' + path, '-S'],
              ['migrate file:' + os.path.join(tmp, 'blocked.mig')])
    check('file-backed flash blocks migration', 'written through' in out,
          True)


def test_misc(qt):
    print('ak-gpio, ak-ddrc, ak-i2c')
    check('GPIO inputs pulled up', qt.readl(GPIO + 0x18), 0xFFFFFFFF)
    qt.writel(GPIO + 0x00, 1 << 5)
    qt.writel(GPIO + 0x0C, 0)
    check('an output pin reads its level', qt.readl(GPIO + 0x18),
          0xFFFFFFDF)
    check('DDR init done', qt.readl(DDRC + 0x24) & 6, 2)
    # Read 2 bytes from 0x30, which is not there.
    qt.writel(I2C0 + 0x10, (1 << 18) | (1 << 17) | (0x30 << 1) | 1)
    qt.writel(I2C0, (1 << 14) | (1 << 15) | (1 << 13) | (1 << 9))
    check('I2C done', qt.readl(I2C0) & (1 << 4), 1 << 4)
    check('absent device reads 0xFF', qt.readl(I2C0 + 0x20) & 0xFFFF,
          0xFFFF)


def mdio_read(qt, reg):
    qt.writel(MAC + 0x10, (1 << 11) | (reg << 6) | 1)
    return qt.readl(MAC + 0x14)


def test_mac(qt):
    print('GMAC (xm-dwmac)')
    check('PHY ID1', mdio_read(qt, 2), 0x0007)
    check('BMSR: link up, autonegotiation done',
          mdio_read(qt, 1) & 0x24, 0x24)
    check('ANER: partner negotiates', mdio_read(qt, 6) & 1, 1)
    # Two TX descriptors, 4 words + 2 software words each, the second
    # ending the ring; a 60-byte frame in each.
    ring = DDR + 0x200000
    for i in range(2):
        d = ring + 24 * i
        qt.writel(d, 1 << 31)
        qt.writel(d + 4, (1 << 29) | (1 << 30) | (1 << 31) | 60 |
                  ((1 << 25) if i else 0))
        qt.writel(d + 8, DDR + 0x210000)
        qt.writel(d + 12, 0)
    qt.writel(MAC + 0x1000, 2 << 2)                 # DSL = 2
    qt.writel(MAC + 0x1010, ring)
    qt.writel(MAC + 0x00, 1 << 3)                   # TE
    qt.writel(MAC + 0x1018, 1 << 13)                # ST
    check('descriptor 0 handed back', qt.readl(ring) >> 31, 0)
    check('descriptor 1 (24 bytes on) handed back', qt.readl(ring + 24) >> 31,
          0)
    check('back at the ring base', qt.readl(MAC + 0x1048), ring)
    check('TX interrupt status', qt.readl(MAC + 0x1014) & 1, 1)


def test_mci(qt):
    print('ak-mci, no card')
    qt.writel(MCI0 + 0x38, 1 << 2)                  # RESPTIMEOUT
    qt.writel(MCI0 + 0x08, 0x1AA)
    qt.writel(MCI0 + 0x0C, 1 | (8 << 1) | (1 << 7)) # CMD8, short response
    check('module interrupt 4 raised', qt.readl(SYSCTRL + 0x4C) & (1 << 4),
          1 << 4)
    check('response timeout', qt.readl(MCI0 + 0x34) & 0x14, 1 << 2)
    check('STATUS read cleared it', qt.readl(MCI0 + 0x34) & 0x14, 0)
    check('and the interrupt', qt.readl(SYSCTRL + 0x4C) & (1 << 4), 0)


def test_ev330(qemu, path):
    print('ak3919ev330: I2C v2, eFuse')
    qt = QTest(qemu, 'ak3919ev330,flash-file=%s' % path)
    try:
        check('chip ID', qt.readl(SYSCTRL), 0x20160101)
        qt.writel(SYSCTRL + 0x8C, 0xA0000000)
        check('eFuse read done', qt.readl(SYSCTRL + 0x8C) >> 30, 0)
        qt.writel(I2C0 + 0x08, (0x30 << 1) | 1 | 0x300)
        qt.writel(I2C0, (2 << 17) | (7 << 22) | (3 << 15) | (1 << 26))
        st = qt.readl(I2C0)
        check('I2C v2 done', st & (1 << 13), 1 << 13)
        check('I2C v2 NAK from an absent device', st & (1 << 27), 1 << 27)
    finally:
        qt.close()


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    ap.add_argument('--qemu', default=os.path.join(
        root, 'qemu-src/build/qemu-system-arm'))
    args = ap.parse_args()

    tmp = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, tmp, True)
    path = os.path.join(tmp, 'flash.bin')
    out = os.path.join(tmp, 'uart0.txt')
    code = make_image(path)
    qt = QTest(args.qemu, 'ak3918ev300,flash-file=%s' % path,
               ['-serial', 'file:' + out])
    try:
        test_boot_rom(qt, code)
        test_sysctrl(qt)
        test_uart(qt, out)
        test_spi(qt, path)
        test_spi_more(qt, path)
        test_misc(qt)
        test_mac(qt)
        test_mci(qt)
    finally:
        qt.close()
    path330 = os.path.join(tmp, 'flash330.bin')
    make_image(path330, b'ANYKAH3B')
    test_ev330(args.qemu, path330)
    test_rejections_and_migration(args.qemu, tmp, path)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
