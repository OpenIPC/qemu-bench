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
  - amb-gpio: the sensor is off I2C until GPIO 107 goes high, as on the
    camera, and comes off again when it goes low, or back to an input;
  - amb-idc: the driver's CTRL conversation reads the OV4689's chip ID from
    I2C0, and an address nobody answers comes back not acknowledged;
  - amb-nand, on made-up images: the ID, a page and its spare through both
    DMA engines and all three interrupts, erase and program written through
    to the file, a page program only clearing bits, write protection, bad
    blocks, images with and without spare bytes, read-only files;
  - the path the camera's own loaders take: the first page at the high
    vectors, RCT's free-running counters, a polled run of pages and
    descriptor DMA reporting into memory;
  - a guest reset clears what was pending, sensor=none leaves the bus
    empty, RAM that leaves no room for the DSP or runs into the peripherals
    is refused, and machine state migrates to a file and back.

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
GPIO3 = 0xE8010000
XSHUTDN = 1 << 11                   # GPIO 107, bit 11 of the fourth bank
TIMER = 0xE800B000
RCT = 0xEC170000
FIO = 0xE0001000
SD0 = 0xE0002000
SD2 = 0xE001F000
ETH = 0xE000E000
FDMA = 0xE0012000
FIFO = 0xE0000000

NAND_PAGE = 2048
NAND_SPARE = 64
NAND_RAW = NAND_PAGE + NAND_SPARE
NAND_PAGES = 65536
NAND_BLOCK = 64 * NAND_PAGE
BUF = 0x01000000                    # in guest RAM
SPARE_BUF = BUF + NAND_PAGE
CMD_RESET, CMD_ERASE, CMD_READID, CMD_READSTATUS = 0x2, 0x9, 0xA, 0xC
FIO_DMA_EN, FIO_DMA_RM = 0x80000000, 0x40000000
FDMA_EN, FDMA_RM, FDMA_WM = 0x80000000, 0x10000000, 0x20000000
FDMA_DESC = 0x40000000
BOOT = 0xFFFF0000
DMASTA_DN, DMASTA_AE = 0x01000000, 0x02000000
FDMASTA_DN = 0x00400000
LINE_FDMA, LINE_CMD, LINE_DMA = 6, 8, 9     # bits of bank 2: 70, 72, 73

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

    def read(self, a, n):
        return bytes.fromhex(self.cmd('read 0x%x 0x%x' % (a, n)).split()[1][2:])

    def write(self, a, data):
        self.cmd('write 0x%x 0x%x 0x%s' % (a, len(data), data.hex()))

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

    # A guest polling or rewriting a register more often than one tick
    # (11 ns) must not hold the expiry off.
    qt.writel(bank + INT_EN, bit)
    qt.writel(TIMER8 + 0x0, 9)          # 100 ns
    qt.writel(TIMER8 + 0x4, 0xFFFFFFFF)
    qt.writel(TIMER_CTRL, T8_EN | T8_OF)
    for _ in range(25):
        qt.writel(TIMER8 + 0x8, 0)
        qt.step(5)
    check('writes under a tick apart do not postpone it', qt.readl(PRI_IRQ),
          LINE_TIMER8)
    qt.writel(TIMER_CTRL, 0)
    qt.writel(bank + INT_EDGE_CLR, bit)

    # RELOAD 0 is the reset value: this has to return, not spin
    qt.writel(TIMER8 + 0x0, 0)
    qt.writel(TIMER8 + 0x4, 0)
    qt.writel(TIMER_CTRL, T8_EN | T8_OF)
    qt.step(100000)
    check('RELOAD 0 fires without pinning the host', qt.readl(PRI_IRQ),
          LINE_TIMER8)
    qt.writel(TIMER_CTRL, 0)
    qt.writel(bank + INT_EDGE_CLR, bit)
    qt.writel(bank + INT_EN_CLR, bit)


def mdio_read(qt, phy, reg):
    """The driver's MII read: address, wait for busy to fall, data."""
    qt.writel(ETH + 0x10, phy << 11 | reg << 6 | 1)
    check('MDIO finishes', qt.readl(ETH + 0x10) & 1, 0)
    return qt.readl(ETH + 0x14)


def test_peripherals(qemu):
    print('Ethernet and SD')
    qt = QTest(qemu, 's3l')
    try:
        check('the Ethernet strap', qt.readl(RCT + 0x34) & 1, 1)
        check('PHY ID high', mdio_read(qt, 0, 2), 0x001C)
        check('PHY ID low', mdio_read(qt, 0, 3), 0xC816)
        for name, base in (('SD0', SD0), ('SD2', SD2)):
            check(name + ' offers interrupt mode',
                  qt.readl(base + 0x40) >> 27 & 1, 1)
    finally:
        qt.close()
    path = tempfile.mktemp()
    make_nand(path)
    qt = QTest(qemu, 's3l,flash-file=' + path)
    try:
        check('a NAND board has its own PHY', mdio_read(qt, 1, 3), 0x0C54)
    finally:
        qt.close()
        os.unlink(path)


def test_reset(qt):
    print('guest reset')
    qt.writel(VIC2 + 0x18, 1 << 3)
    qt.writel(VIC2 + INTEN, 1 << 3)
    check('pending before the reset', qt.readl(PRI_IRQ), 64 + 3)
    qt.writel(RCT + 0x68, 0x2)
    qt.writel(RCT + 0x68, 0x3)
    qt.step(1000)
    check('nothing pending after it', qt.readl(PRI_IRQ), NO_IRQ)
    check('the mask went with it', qt.readl(VIC2 + INTEN), 0)


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


def xshutdn(qt, high):
    """gpio_direction_output(107, high), as gpio-ambarella.c does it."""
    qt.writel(GPIO3 + 0x18, qt.readl(GPIO3 + 0x18) & ~XSHUTDN)  # not AFSEL
    qt.writel(GPIO3 + 0x04, qt.readl(GPIO3 + 0x04) | XSHUTDN)   # output
    qt.writel(GPIO3 + 0x28, XSHUTDN)                            # MASK
    qt.writel(GPIO3 + 0x00, XSHUTDN if high else 0)             # DATA


def test_sensor_reset(qt):
    print('amb-gpio: the sensor reset')
    qt.writel(IDC0 + ENR, 1)
    check('held at power-on', read_reg16(qt, OV4689, 0x300A), None)
    xshutdn(qt, True)
    check('released by GPIO 107', read_reg16(qt, OV4689, 0x300A), 0x46)
    check('the pad reads back', qt.readl(GPIO3) & XSHUTDN, XSHUTDN)
    xshutdn(qt, False)
    check('held again when it goes low', read_reg16(qt, OV4689, 0x300A), None)
    xshutdn(qt, True)
    qt.writel(GPIO3 + 0x04, qt.readl(GPIO3 + 0x04) & ~XSHUTDN)
    check('an input does not drive it: held',
          read_reg16(qt, OV4689, 0x300A), None)
    qt.writel(GPIO3 + 0x04, qt.readl(GPIO3 + 0x04) | XSHUTDN)
    qt.writel(GPIO3 + 0x18, qt.readl(GPIO3 + 0x18) | XSHUTDN)
    check('nor does a peripheral function',
          read_reg16(qt, OV4689, 0x300A), None)
    qt.writel(GPIO3 + 0x18, qt.readl(GPIO3 + 0x18) & ~XSHUTDN)
    check('released once it is back', read_reg16(qt, OV4689, 0x300A), 0x46)


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


def make_nand(path, spare=False):
    """A sparse image, zero but for the two pages the checks read."""
    unit = NAND_RAW if spare else NAND_PAGE
    with open(path, 'wb') as f:
        f.truncate(NAND_PAGES * unit)
        for page in (5, 300):
            f.seek(page * unit)
            f.write(bytes((page + i) & 0xFF for i in range(unit)))


def nand_cmd(qt, cmd, addr=0, ctr=0):
    qt.writel(FIO + 0x120, ctr)
    qt.writel(FIO + 0x124, addr | cmd)


def nand_status(qt):
    nand_cmd(qt, CMD_READSTATUS)
    return qt.readl(FIO + 0x140)


def nand_ack(qt):
    """What the three interrupt handlers do."""
    qt.writel(FIO + 0x150, 0)
    qt.writel(FIO + 0x08C, 0)
    qt.writel(FDMA + 0x30C, 0)
    qt.writel(FDMA + 0x20C, 0)


def nand_dma(qt, page, write, ctr=0):
    """nand_amb_setup_dma_devmem() / _memdev() in dual-space mode."""
    qt.writel(FDMA + 0x30C, 0)
    qt.writel(FDMA + 0x304, BUF if write else FIFO)
    qt.writel(FDMA + 0x308, FIFO if write else BUF)
    qt.writel(FDMA + 0x20C, 0)
    qt.writel(FDMA + 0x204, SPARE_BUF if write else FIFO)
    qt.writel(FDMA + 0x208, FIFO if write else SPARE_BUF)
    qt.writel(FDMA + 0x200, NAND_SPARE)
    qt.writel(FDMA + 0x300, FDMA_EN | (FDMA_RM if write else FDMA_WM) |
              NAND_PAGE)
    qt.writel(FIO + 0x120, ctr)
    qt.writel(FIO + 0x084, page * NAND_PAGE)
    qt.writel(FIO + 0x080, FIO_DMA_EN | (FIO_DMA_RM if write else 0) |
              (NAND_PAGE + NAND_SPARE))
    return qt.readl(FIO + 0x08C)


def nand_read(qt, page):
    sta = nand_dma(qt, page, False)
    nand_ack(qt)
    return sta, qt.read(BUF, NAND_PAGE), qt.read(SPARE_BUF, NAND_SPARE)


def nand_program(qt, page, data, spare=b'\xFF' * NAND_SPARE, ctr=0):
    qt.write(BUF, data)
    qt.write(SPARE_BUF, spare)
    sta = nand_dma(qt, page, True, ctr)
    nand_ack(qt)
    return sta


def test_boot_chain(qemu, tmp):
    """What the camera's own loaders use that the kernel does not.  Their
    images are vendor code, so this drives the same paths by hand."""
    print('amb-nand and amb-rct, as the boot loaders use them')
    path = os.path.join(tmp, 'boot.bin')
    make_nand(path)
    with open(path, 'rb') as f:
        first = f.read(NAND_PAGE)
        f.seek(5 * NAND_PAGE)
        pages = f.read(2 * NAND_PAGE)
    qt = QTest(qemu, 's3l,flash-file=' + path)
    try:
        check('the first page is at the high vectors',
              qt.read(BOOT, 2048), first)
        check('the reset vector jumps there', qt.readl(0), 0xE51FF004)
        check('to that address', qt.readl(4), BOOT)

        a = qt.readl(RCT + 0x254)
        qt.step(1000)
        check('RCT timer runs at 24 MHz', qt.readl(RCT + 0x254) - a, 24)
        qt.step(100000)
        qt.writel(RCT + 0x498, 1)
        qt.writel(RCT + 0x498, 0)
        b = qt.readl(RCT + 0x494)
        qt.step(1000)
        check('timer 2 restarts from zero', b, 0)
        check('and counts', qt.readl(RCT + 0x494), 24)

        # bst: a run of pages with their spare bytes, polling, no interrupts
        qt.writel(FDMA + 0x30C, 0)
        qt.writel(FDMA + 0x304, FIFO)
        qt.writel(FDMA + 0x308, BUF)
        qt.writel(FDMA + 0x20C, 0)
        qt.writel(FDMA + 0x204, FIFO)
        qt.writel(FDMA + 0x208, BUF + 0x10000)
        qt.writel(FDMA + 0x200, 2 * NAND_SPARE)
        qt.writel(FDMA + 0x300, FDMA_EN | FDMA_WM | 2 * NAND_PAGE)
        qt.writel(FIO + 0x084, 5 * NAND_PAGE)
        qt.writel(FIO + 0x080, FIO_DMA_EN | (2 * NAND_RAW))
        check('polled done flag', qt.readl(FIO + 0x150) & 1, 1)
        check('polled DMA done', qt.readl(FIO + 0x08C), DMASTA_DN)
        check('two pages at once', qt.read(BUF, 2 * NAND_PAGE), pages)
        nand_ack(qt)

        # bld: spare bytes through descriptors, reporting into memory
        desc, spare = BUF + 0x20000, BUF + 0x20100
        report = BUF + 0x20200
        for addr, dst, count, rpt in ((desc, BUF + 0x30000, NAND_PAGE, report),
                                      (spare, BUF + 0x40000, NAND_SPARE,
                                       report + 4)):
            for i, word in enumerate((FIFO, dst, 0, rpt, count)):
                qt.writel(addr + i * 4, word)
        qt.writel(report, 0)
        qt.writel(report + 4, 0)
        qt.writel(FDMA + 0x380, desc)
        qt.writel(FDMA + 0x30C, 0)
        qt.writel(FDMA + 0x280, spare)
        qt.writel(FDMA + 0x20C, 0)
        qt.writel(FDMA + 0x300, FDMA_EN | FDMA_DESC)
        qt.writel(FIO + 0x084, 5 * NAND_PAGE)
        qt.writel(FIO + 0x080, FIO_DMA_EN | NAND_RAW)
        check('a descriptor reads a page', qt.read(BUF + 0x30000, NAND_PAGE),
              pages[:NAND_PAGE])
        check('blank spare through its descriptor',
              qt.read(BUF + 0x40000, NAND_SPARE), b'\xff' * NAND_SPARE)
        check('the report lands in memory', qt.readl(report), FDMASTA_DN)
        check('for both', qt.readl(report + 4), FDMASTA_DN)
    finally:
        qt.close()


def test_nand(qemu, tmp):
    print('amb-nand')
    path = os.path.join(tmp, 'nand.bin')
    make_nand(path)
    qt = QTest(qemu, 's3l,flash-file=%s,bad-blocks=0x20000:0x60000' % path)
    try:
        check('the straps say BCH on 2K pages',
              qt.readl(RCT + 0x34) & 0x78000, 0x70000)

        nand_cmd(qt, CMD_RESET)
        check('a command sets FI', qt.readl(FIO + 0x04) & 1, 1)
        qt.writel(FIO + 0x150, 0)
        check('FLASH_INT clears it', qt.readl(FIO + 0x04) & 1, 0)

        nand_cmd(qt, CMD_READID)
        check('the ID', qt.readl(FIO + 0x144), 0x98F18015)
        qt.writel(FIO + 0x150, 0)
        check('ready, not protected', nand_status(qt), 0xC0)
        qt.writel(FIO + 0x150, 0)

        for line in (LINE_FDMA, LINE_CMD, LINE_DMA):
            qt.writel(VIC2 + INT_SENSE, line)
            qt.writel(VIC2 + INT_EVT, line)
        check('quiet between transfers', qt.readl(VIC2 + 0x08) & 0x340, 0)

        sta = nand_dma(qt, 5, False)
        check('FIO DMA done', sta, DMASTA_DN)
        check('FDMA done', qt.readl(FDMA + 0x30C), FDMASTA_DN)
        check('FDMA channel 0 pending', qt.readl(FDMA + 0x3F0) & 1, 1)
        check('all three lines up', qt.readl(VIC2 + 0x08) & 0x340, 0x340)
        check('the enable falls', qt.readl(FIO + 0x080) >> 31, 0)
        check('ECC reports nothing', qt.readl(FIO + 0x0A4), 0)
        nand_ack(qt)
        check('each handler lowers its own', qt.readl(VIC2 + 0x08) & 0x340, 0)
        check('the page arrived', qt.read(BUF, NAND_PAGE),
              bytes((5 + i) & 0xFF for i in range(NAND_PAGE)))
        check('with no spare bytes in the file, they are blank',
              qt.read(SPARE_BUF, NAND_SPARE), b'\xFF' * NAND_SPARE)

        nand_cmd(qt, CMD_ERASE, 8 * NAND_BLOCK)
        qt.writel(FIO + 0x150, 0)
        check('erased', nand_read(qt, 8 * 64)[1], b'\xFF' * NAND_PAGE)
        data = bytes(i & 0xF0 for i in range(NAND_PAGE))
        tag = bytes(range(NAND_SPARE))
        check('program', nand_program(qt, 8 * 64, data, tag), DMASTA_DN)
        check('reads back', nand_read(qt, 8 * 64)[1], data)
        check('and its spare', nand_read(qt, 8 * 64)[2], tag)
        with open(path, 'rb') as f:
            f.seek(8 * NAND_BLOCK)
            check('written through', f.read(NAND_PAGE), data)
        nand_program(qt, 8 * 64, b'\x0F' * NAND_PAGE)
        check('programming only clears bits', nand_read(qt, 8 * 64)[1],
              bytes(NAND_PAGE))

        sta = nand_program(qt, 9 * 64, b'\x00' * NAND_PAGE, ctr=0x200)
        check('write protect: the DMA itself completes', sta, DMASTA_DN)
        check('but the chip reports failure', nand_status(qt) & 1, 1)
        qt.writel(FIO + 0x150, 0)
        nand_cmd(qt, CMD_READSTATUS, ctr=0x200)
        check('and reads as protected', qt.readl(FIO + 0x140) & 0x80, 0)
        qt.writel(FIO + 0x150, 0)
        nand_cmd(qt, CMD_RESET)
        qt.writel(FIO + 0x150, 0)
        check('nothing was written', nand_read(qt, 9 * 64)[1][:4],
              bytes(4))

        check('a bad block has the factory marker',
              nand_read(qt, 64)[2][0], 0)
        check('the next block has not', nand_read(qt, 128)[2][0], 0xFF)
        check('and the listed third has', nand_read(qt, 192)[2][0], 0)
        nand_cmd(qt, CMD_ERASE, NAND_BLOCK)
        qt.writel(FIO + 0x150, 0)
        check('erasing it fails', nand_status(qt) & 1, 1)
        qt.writel(FIO + 0x150, 0)

        sta = nand_dma(qt, NAND_PAGES, False, 0)
        check('past the chip: a DMA error', sta, DMASTA_AE)
        nand_ack(qt)
    finally:
        qt.close()

    # The file holds what it was given, and survives into a new run
    qt = QTest(qemu, 's3l,flash-file=%s' % path)
    try:
        check('a new run reads it back', nand_read(qt, 8 * 64)[1],
              bytes(NAND_PAGE))
        check('with no bad-blocks, no marker', nand_read(qt, 64)[2][0], 0xFF)
    finally:
        qt.close()

    raw = os.path.join(tmp, 'raw.bin')
    make_nand(raw, spare=True)
    qt = QTest(qemu, 's3l,flash-file=%s' % raw)
    try:
        sta, page, spare = nand_read(qt, 5)
        check('an image with spare bytes: the page',
              page, bytes((5 + i) & 0xFF for i in range(NAND_PAGE)))
        check('and its spare', spare,
              bytes((5 + NAND_PAGE + i) & 0xFF for i in range(NAND_SPARE)))
        nand_cmd(qt, CMD_ERASE, 8 * NAND_BLOCK)
        qt.writel(FIO + 0x150, 0)
        nand_program(qt, 8 * 64, bytes(NAND_PAGE), bytes(range(NAND_SPARE)))
    finally:
        qt.close()
    with open(raw, 'rb') as f:
        f.seek(8 * 64 * NAND_RAW + NAND_PAGE)
        check('spare bytes reach such a file', f.read(NAND_SPARE),
              bytes(range(NAND_SPARE)))

    ro = os.path.join(tmp, 'ro.bin')
    make_nand(ro)
    os.chmod(ro, 0o444)
    before = os.path.getmtime(ro)
    qt = QTest(qemu, 's3l,flash-file=%s' % ro)
    try:
        nand_cmd(qt, CMD_ERASE, 8 * NAND_BLOCK)
        qt.writel(FIO + 0x150, 0)
        check('a read-only file: the guest still erases',
              nand_read(qt, 8 * 64)[1], b'\xFF' * NAND_PAGE)
    finally:
        qt.close()
    with open(ro, 'rb') as f:
        f.seek(8 * NAND_BLOCK)
        check('and the file does not change', f.read(16), bytes(16))
    check('not even its time', os.path.getmtime(ro), before)

    small = os.path.join(tmp, 'small.bin')
    with open(small, 'wb') as f:
        f.truncate(1 << 20)
    out = subprocess.run([qemu, '-M', 's3l,flash-file=' + small, '-display',
                          'none', '-nodefaults', '-kernel', '/dev/null'],
                         capture_output=True, text=True, timeout=20)
    check('the wrong size is refused', 'expected' in out.stderr, True)
    out = subprocess.run([qemu, '-M',
                          's3l,flash-file=%s,bad-blocks=0x1234' % path,
                          '-display', 'none', '-nodefaults', '-kernel',
                          '/dev/null'],
                         capture_output=True, text=True, timeout=20)
    check('so is a bad block that is not a block',
          'not the offset of a block' in out.stderr, True)
    out = hmp(qemu, ['-M', 's3l,flash-file=' + path, '-S'],
              ['migrate file:' + os.path.join(tmp, 'blocked.mig')])
    check('a file-backed NAND blocks migration', 'written through' in out,
          True)

    qt = QTest(qemu, 's3l')
    try:
        check('no flash-file, no NAND straps',
              qt.readl(RCT + 0x34) & 0x78000, 0)
    finally:
        qt.close()


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


def test_no_sensor(qemu):
    print('sensor=none, sensor-reset=off')
    qt = QTest(qemu, 's3l,sensor=none')
    try:
        qt.writel(IDC0 + ENR, 1)
        xshutdn(qt, True)
        check('the bus is empty', read_reg16(qt, OV4689, 0x300A), None)
    finally:
        qt.close()
    qt = QTest(qemu, 's3l,sensor-reset=off')
    try:
        qt.writel(IDC0 + ENR, 1)
        check('no reset line: it answers at once',
              read_reg16(qt, OV4689, 0x300A), 0x46)
    finally:
        qt.close()


def test_rejections_and_migration(qemu, tmp):
    print('rejections and migration')
    r = subprocess.run([qemu, '-M', 's3l', '-m', '64M', '-display', 'none',
                        '-nodefaults', '-kernel', '/dev/null'],
                       capture_output=True, text=True, timeout=20)
    check('no room for the DSP is refused', 'room for the DSP' in r.stderr,
          True)

    r = subprocess.run([qemu, '-M', 's3l', '-m', '3584M', '-display', 'none',
                        '-nodefaults', '-kernel', '/dev/null'],
                       capture_output=True, text=True, timeout=20)
    check('RAM over the peripherals is refused',
          'end below the peripherals' in r.stderr, True)

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
        test_sensor_reset(qt)
        test_i2c(qt)
        test_reset(qt)
    finally:
        qt.close()
    test_nand(args.qemu, tmp)
    test_boot_chain(args.qemu, tmp)
    test_peripherals(args.qemu)
    test_no_sensor(args.qemu)
    test_rejections_and_migration(args.qemu, tmp)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
