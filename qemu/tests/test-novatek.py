#!/usr/bin/env python3
"""Register-level checks of the Novatek NT9856x models, no guest needed.

QEMU runs the nt98562 (and nt98566) machine under qtest and this script
plays the vendor drivers (docs/novatek.md):

  - nvt-top: the chip ID nvt_get_chip_id() and U-Boot read, pinmux
    storage;
  - nvt-cg: the PLL8 ratio and CPU clock select the clock driver turns
    into periph_clk, which must be the 100 MHz QEMU's A9 timers tick at;
    PLL lock status following PLL_EN;
  - nvt-wdt: a CTRL write without the key is lost; an interrupt-mode
    expiry raises SPI 57 and the status write-back acknowledges it; a
    reset-mode expiry and the kernel's manual reset both reset the machine;
  - nvt-gpio: SET / CLR / DIR / DATA, input pads, a rising-edge interrupt
    on C_GPIO 3 and a level-low one on D_GPIO 0 through SPI 24; the "out"
    lines follow the pad drivers through direction changes and a reset;
  - nvt-rtc: a kept-alive RTC (key, oscillator) reading the host's year;
    a CSET commits the time and raises SPI 56; the daily alarm and the
    driver's wake alarm (PWRALM, committed by CSET) go off; powering off with the
    power-on alarm armed is the vendor reboot, and resets the machine;
  - nvt-flash, the way nvt_flash_spi_cmd.c drives it: the JEDEC ID by
    PIO, a manual-CS read (CMD_ADDR then READ_N), a DMA read into RAM, a
    block erase with its RDSR poll and a PIO page program, both written
    back to the image file, and the completion interrupt on SPI 29;
  - nvt-flash in SPI NAND mode, the way spinand.c drives a W25N01GV, on
    a 128 MiB image of data alone and on one with each page's OOB: the
    ID after its address byte, GET / SET_FEATURE, DATA_RD3 page reads (by
    DMA, the OOB into the spare SRAM, and the OOB alone), a block erase
    and an SPI_PROG_CMD page program polled through GET_FEATURE C0, the
    image file afterwards; a malformed image size is refused;
  - the flash boot ROM: a synthetic NVT image (device tree at 0x10000,
    BCL1/LZ-packed "U-Boot") leaves U-Boot unpacked at
    /nvt_memory_cfg/uboot, the device tree and the "LD_NVT" SHMINFO where
    the tree says, and a branch at 0; an image with no tree is refused;
    a NAND image (device tree in block 2, a plain U-Boot) boots the same
    way, its SHMINFO naming SPI NAND;
  - nvt-eqos: the embedded PHY over MDIO, the HW feature bits the driver
    keys on (MDIO present, one queue), the software reset, the Rx DMA
    state yinit() polls for, a frame sent through a Tx descriptor (seen in
    a filter-dump pcap) with its write-back and interrupt, and the
    Novatek ETH_ISR summary the ISR checks first; a frame split across two
    tail writes, an over-long one dropped with ES / JT; frames injected
    through a socket netdev: unicast filtering, a frame spanning 64-byte
    buffers, Rx stopping at its tail pointer until it moves;
  - the 16550 UART at its reg-shift-2 offsets, and input that arrived
    before the guest set the UART up still flowing after it resets the
    receive FIFO (qemu/patches/0003, the vendor U-Boot's autoboot stop);
  - the generated device tree: compatible, the clock nodes in the order
    of_clk_init() needs, the pinmux children in the driver's order; the
    flash node spinand.c binds when the image is a NAND;
  - outside qtest: machine state migrates to a file and back.

    python3 qemu/tests/test-novatek.py [--qemu PATH]
"""
import argparse
import os
import random
import re
import socket
import struct
import subprocess
import sys
import tempfile
import time

TOP = 0xF0010000
CG = 0xF0020000
WDT = 0xF0050000
RTC = 0xF0060000
GPIO = 0xF0070000
UART0 = 0xF0290000
FLASH = 0xF0400000
ETH = 0xF02B0000
GICD = 0xFFD01000

SPI_GPIO = 24
SPI_FLASH = 29
SPI_ETH = 34
SPI_RTC = 56
SPI_WDT = 57

WDT_KEY = 0x5A96 << 16


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
        self.irqs = {}

    def cmd(self, line):
        self.f.write(line + '\n')
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith('IRQ '):
                _, how, n = r.split()
                self.irqs[int(n)] = how == 'raise'
            elif r.startswith(('OK', 'FAIL', 'ERR')):
                return r

    def writel(self, a, v):
        self.cmd('writel 0x%x 0x%x' % (a, v))

    def readl(self, a):
        return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def read(self, a, n):
        return bytes.fromhex(self.cmd('read 0x%x %d' % (a, n)).split()[1][2:])

    def write(self, a, data):
        self.cmd('write 0x%x %d 0x%s' % (a, len(data), data.hex()))

    def step(self, ns):
        self.cmd('clock_step %d' % ns)

    def pin(self, n, level):
        self.cmd('set_irq_in /machine/nvt-gpio in %d %d' % (n, level))

    def close(self):
        self.proc.kill()
        self.proc.wait()


failures = 0


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    if isinstance(got, bytes):
        shown = got[:8].hex() + ('...' if len(got) > 8 else '')
    else:
        shown = got if isinstance(got, (bool, str)) else hex(got)
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown))


def gic_enable(qt, spi):
    n = 32 + spi
    qt.writel(GICD + 0x100 + (n // 32) * 4, 1 << (n % 32))


def gic_pending(qt, spi):
    n = 32 + spi
    return bool(qt.readl(GICD + 0x200 + (n // 32) * 4) & (1 << (n % 32)))


def test_top_cg(qt):
    print('nvt-top, nvt-cg')
    check('chip ID 0x7021 in TOP_VERSION[31:16]', qt.readl(TOP + 0xF0) >> 16,
          0x7021)
    qt.writel(TOP + 0xF0, 0)
    check('TOP_VERSION is read-only', qt.readl(TOP + 0xF0) >> 16, 0x7021)
    qt.writel(TOP + 0x04, 0x12345678)
    check('pinmux word keeps its value', qt.readl(TOP + 0x04), 0x12345678)

    check('CPU clock from PLL8', qt.readl(CG + 0x10) & 3, 1)
    ratio = (qt.readl(CG + 0x4420) & 0xFF) | \
        ((qt.readl(CG + 0x4424) & 0xFF) << 8) | \
        ((qt.readl(CG + 0x4428) & 0xFF) << 16)
    periph = 12000000 * ratio // 131072 // 8
    check('periph_clk = PLL8 / 8 within 1 kHz of 100 MHz',
          abs(periph - 100000000) < 1000, True)
    qt.writel(CG + 0x00, 0x00000100)
    check('PLL_STAT follows PLL_EN', qt.readl(CG + 0x04), 0x00000100)


def test_wdt(qt):
    print('nvt-wdt')
    gic_enable(qt, SPI_WDT)
    qt.writel(WDT + 0x00, 0x00000001)
    check('CTRL without the key is ignored', qt.readl(WDT + 0x00), 0)
    # MSB 0: 0xFFF ticks of 12 MHz / 1024, 0.35 s.  Interrupt mode.
    qt.writel(WDT + 0x00, WDT_KEY | 0x01)
    check('STS reports enabled', bool(qt.readl(WDT + 0x04) & (1 << 30)),
          True)
    qt.step(200 * 1000 * 1000)
    check('counting up', 0 < (qt.readl(WDT + 0x04) & 0xFFFFF) < 0xFFF, True)
    qt.writel(WDT + 0x08, 1)
    check('TRIG restarts the count', (qt.readl(WDT + 0x04) & 0xFFFFF) < 2,
          True)
    qt.step(400 * 1000 * 1000)
    check('expired', bool(qt.readl(WDT + 0x04) & (1 << 31)), True)
    check('SPI 57 pending', gic_pending(qt, SPI_WDT), True)
    qt.writel(WDT + 0x04, qt.readl(WDT + 0x04))
    check('status write-back acknowledges', gic_pending(qt, SPI_WDT), False)
    qt.writel(WDT + 0x00, WDT_KEY)

    # A reset puts CG back to its reset value: use that as the witness.
    qt.writel(CG + 0x10, 0)
    qt.writel(WDT + 0x84, 0xCAFE)
    qt.writel(WDT + 0x00, WDT_KEY | 0x03)
    qt.step(400 * 1000 * 1000)
    check('reset-mode expiry resets the machine', qt.readl(CG + 0x10) & 3, 1)
    check('watchdog off after the reset', qt.readl(WDT + 0x00), 0)
    check('UDATA survives it', qt.readl(WDT + 0x84), 0xCAFE)

    # plat-na51089/reset.c: nvt_trigger_wdt_external()
    qt.writel(CG + 0x10, 0)
    qt.writel(WDT + 0x00, 0x5A960112)
    qt.writel(WDT + 0x00, 0x5A960113)
    qt.writel(WDT + 0x0C, 1)
    qt.step(1000)
    check('manual reset resets the machine', qt.readl(CG + 0x10) & 3, 1)


def test_gpio(qt):
    print('nvt-gpio')
    gic_enable(qt, SPI_GPIO)
    qt.cmd('irq_intercept_out /machine/nvt-gpio out')
    # P_GPIO 5 (bank 1) as an output
    qt.writel(GPIO + 0x24, 1 << 5)
    qt.writel(GPIO + 0x44, 1 << 5)
    check('SET drives the pad high', qt.readl(GPIO + 0x04) & (1 << 5),
          1 << 5)
    qt.writel(GPIO + 0x64, 1 << 5)
    check('CLR drives it low', qt.readl(GPIO + 0x04) & (1 << 5), 0)
    check('DIR reads back', qt.readl(GPIO + 0x24), 1 << 5)
    qt.pin(0x20 + 6, 1)
    check('an input pad reads its level', qt.readl(GPIO + 0x04) & (1 << 6),
          1 << 6)
    qt.pin(0x20 + 5, 1)
    check('an output pad ignores its input', qt.readl(GPIO + 0x04) &
          (1 << 5), 0)
    # out line 37 = P_GPIO 5: high output, to input, latch cleared, output
    qt.writel(GPIO + 0x44, 1 << 5)
    check('out line follows a high output', qt.irqs.get(0x25), True)
    qt.pin(0x20 + 5, 0)
    qt.writel(GPIO + 0x24, 0)
    check('released as an input', qt.irqs.get(0x25), False)
    qt.writel(GPIO + 0x64, 1 << 5)
    qt.writel(GPIO + 0x24, 1 << 5)
    check('a low output again stays low', qt.irqs.get(0x25), False)
    check('DATA agrees', qt.readl(GPIO + 0x04) & (1 << 5), 0)
    qt.writel(GPIO + 0x44, 1 << 5)

    # Interrupt 0 is C_GPIO 3: rising edge
    qt.writel(GPIO + 0xA0, 0)
    qt.writel(GPIO + 0xA4, 0)
    qt.writel(GPIO + 0xA8, 0)
    qt.writel(GPIO + 0x90, 1)
    qt.pin(3, 1)
    check('rising edge on C_GPIO 3 latches STS bit 0',
          qt.readl(GPIO + 0x80) & 1, 1)
    check('SPI 24 pending', gic_pending(qt, SPI_GPIO), True)
    qt.writel(GPIO + 0x80, 1)
    check('write-1-to-clear', qt.readl(GPIO + 0x80) & 1, 0)
    check('SPI 24 released', gic_pending(qt, SPI_GPIO), False)
    qt.pin(3, 0)
    check('the falling edge does not fire', qt.readl(GPIO + 0x80) & 1, 0)
    qt.writel(GPIO + 0x90, 0)

    # Interrupt 32 is D_GPIO 0 (DGPIO bit 0): level low, stays pending
    # while the pad is low
    qt.writel(GPIO + 0xE0, 1)
    qt.writel(GPIO + 0xE4, 1)
    qt.writel(GPIO + 0xD0, 1)
    check('level-low D_GPIO 0 latches DGPIO STS bit 0',
          qt.readl(GPIO + 0xC0) & 1, 1)
    qt.writel(GPIO + 0xC0, 1)
    check('and re-latches while the level holds', qt.readl(GPIO + 0xC0) & 1,
          1)
    qt.pin(0x80, 1)
    qt.writel(GPIO + 0xC0, 1)
    check('cleared once the pad goes high', qt.readl(GPIO + 0xC0) & 1, 0)
    check('SPI 24 idle', gic_pending(qt, SPI_GPIO), False)

    # A reset makes every pin an input again (manual watchdog reset)
    check('P_GPIO 5 still driven high', qt.irqs.get(0x25), True)
    qt.writel(WDT + 0x00, WDT_KEY | 0x01)
    qt.writel(WDT + 0x0C, 1)
    qt.step(1000)
    check('reset releases it', qt.irqs.get(0x25), False)


def rtc_tod(qt):
    t = qt.readl(RTC + 0x00)
    return (t & 0x3F) + ((t >> 6) & 0x3F) * 60 + ((t >> 12) & 0x1F) * 3600


def rtc_year(qt):
    days = qt.readl(RTC + 0x04) & 0xFFFF
    return 1900 + int(days / 365.2425)


def test_rtc(qt):
    print('nvt-rtc')
    check('key 0xA: no power loss', (qt.readl(RTC + 0x04) >> 16) & 0xF, 0xA)
    check('oscillator setting 0x9', qt.readl(RTC + 0x24) & 0xFF, 0x9)
    check('host year', rtc_year(qt), time.gmtime().tm_year)
    gic_enable(qt, SPI_RTC)
    # rtc-na51055.c set_time: select, shadow writes, CSET with interrupt
    qt.writel(RTC + 0x10, (1 << 3) | (1 << 4))
    qt.writel(RTC + 0x00, 5 | (4 << 6) | (3 << 12))
    qt.writel(RTC + 0x04, (0xA << 16) | 36524)      # 2000-01-01
    qt.writel(RTC + 0x10, qt.readl(RTC + 0x10) | (1 << 2) | (1 << 8))
    check('CSET done', bool(qt.readl(RTC + 0x0C) & (1 << 8)), True)
    check('CSET bit self-clears', qt.readl(RTC + 0x10) & (1 << 2), 0)
    check('SPI 56 pending', gic_pending(qt, SPI_RTC), True)
    qt.writel(RTC + 0x0C, 1 << 8)
    check('acknowledged', gic_pending(qt, SPI_RTC), False)
    check('time committed', qt.readl(RTC + 0x00) & 0x1FFC0,
          (4 << 6) | (3 << 12))
    check('day committed', qt.readl(RTC + 0x04) & 0xFFFF, 36524)
    # The daily alarm, 3 s ahead, with the alarm interrupt on
    tod = rtc_tod(qt) + 3
    qt.writel(RTC + 0x08, (tod % 60) | ((tod // 60 % 60) << 6) |
              ((tod // 3600 % 24) << 12))
    qt.writel(RTC + 0x10, 1)
    qt.step(1500 * 1000 * 1000)
    check('daily alarm not yet', qt.readl(RTC + 0x0C) & 1, 0)
    qt.step(3000 * 1000 * 1000)
    check('daily alarm goes off', qt.readl(RTC + 0x0C) & 1, 1)
    check('SPI 56 pending for it', gic_pending(qt, SPI_RTC), True)
    qt.writel(RTC + 0x0C, 1)
    check('acknowledged', gic_pending(qt, SPI_RTC), False)
    qt.writel(RTC + 0x08, 0x1FFFF)

    # rtc-na51055.c set_alarm: the wake alarm in PWRALM, through a CSET
    day = qt.readl(RTC + 0x04) & 0xFFFF
    tod = rtc_tod(qt) + 3
    qt.writel(RTC + 0x10, (1 << 6) | (1 << 7) | 1)
    qt.writel(RTC + 0x20, (tod % 60) | ((tod // 60 % 60) << 6) |
              ((tod // 3600 % 24) << 12) | ((day & 0x1F) << 20))
    check('PWRALM is shadowed until the CSET', qt.readl(RTC + 0x20), 0)
    qt.writel(RTC + 0x10, qt.readl(RTC + 0x10) | (1 << 2))
    qt.writel(RTC + 0x0C, 1 << 8)
    qt.step(4500 * 1000 * 1000)
    check('wake alarm goes off', qt.readl(RTC + 0x0C) & 1, 1)
    qt.writel(RTC + 0x0C, 1)

    # plat-na51089/reset.c reboot: arm the power-on alarm, power off
    qt.writel(CG + 0x10, 0)
    qt.writel(RTC + 0x18, 1 << 3)
    check('alarm armed', qt.readl(RTC + 0x1C) & 1, 1)
    check('enable bit self-clears', qt.readl(RTC + 0x18) & (1 << 3), 0)
    qt.writel(RTC + 0x18, 1 << 1)
    qt.step(1000)
    check('power off with the alarm armed resets', qt.readl(CG + 0x10) & 3, 1)
    check('the RTC keeps its day across it',
          qt.readl(RTC + 0x04) & 0xFFFF, 36524)


def test_uart(qt):
    print('UART (16550, reg-shift 2)')
    check('LSR: transmitter empty', qt.readl(UART0 + 0x14) & 0x60, 0x60)
    qt.writel(UART0 + 0x0C, 0x83)
    qt.writel(UART0 + 0x00, 13)
    qt.writel(UART0 + 0x0C, 0x03)
    check('LCR reads back', qt.readl(UART0 + 0x0C), 0x03)


# ---- nvt-flash ---------------------------------------------------------

OP_READ_ID, OP_CMD, OP_CMD_ADDR, OP_BLK_ERSE = 0x06, 0x07, 0x0C, 0x05
OP_READ_N, OP_WRITE_N = 0x18, 0x19


def flash_op(qt, op, cmd, row=0, length=0, dma=False, write=False):
    qt.writel(FLASH + 0x48, 1 << 12)            # clear COMP
    qt.writel(FLASH + 0x34, cmd)
    qt.writel(FLASH + 0x3C, row)
    qt.writel(FLASH + 0x104, length)
    qt.writel(FLASH + 0x10C, 1 | (2 if dma else 0) | (4 if write else 0))
    qt.writel(FLASH + 0x20, op | (1 << 12))


def flash_done(qt):
    return bool(qt.readl(FLASH + 0x48) & (1 << 12))


def test_flash(qt, path):
    print('nvt-flash')
    img = open(path, 'rb').read()
    gic_enable(qt, SPI_FLASH)
    qt.writel(FLASH + 0x44, 1 << 12)            # completion interrupt
    qt.writel(FLASH + 0x00, 2 << 2)             # 3 row-address bytes

    flash_op(qt, OP_READ_ID, 0x9F, length=4)
    check('READ_ID completes', flash_done(qt), True)
    check('SPI 29 pending', gic_pending(qt, SPI_FLASH), True)
    check('JEDEC ID through the FIFO (W25Q64 for 8 MiB)',
          qt.readl(FLASH + 0x100) & 0xFFFFFF, 0x1740EF)
    qt.writel(FLASH + 0x48, 1 << 12)
    check('status write-back acknowledges', gic_pending(qt, SPI_FLASH),
          False)

    # spiflash_read_data: CS manual and low across CMD_ADDR + READ_N
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x03, row=0x1234)
    flash_op(qt, OP_READ_N, 0x03, length=16)
    words = b''.join(qt.readl(FLASH + 0x100).to_bytes(4, 'little')
                     for _ in range(4))
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    check('PIO read under manual CS', words, img[0x1234:0x1244])

    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x03, row=0x20000)
    qt.writel(FLASH + 0x110, 0x01000000)
    flash_op(qt, OP_READ_N, 0x03, length=256, dma=True)
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    got = b''.join(qt.readl(0x01000000 + i).to_bytes(4, 'little')
                   for i in range(0, 256, 4))
    check('DMA read into RAM', got, img[0x20000:0x20100])

    # spinor_erase_sector: WREN, then BLK_ERSE (cmd, address, RDSR polled)
    flash_op(qt, OP_CMD, 0x06)
    qt.writel(FLASH + 0xBC, (1 << 8) | 0)       # mask WIP, want 0
    flash_op(qt, OP_BLK_ERSE, 0xD8 | (0x05 << 8), row=0x30000)
    check('block erase completes', flash_done(qt), True)
    # spiflash_page_program, PIO: WREN, CMD_ADDR, WRITE_N, data port
    flash_op(qt, OP_CMD, 0x06)
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x02, row=0x30010)
    flash_op(qt, OP_WRITE_N, 0x02, length=8, write=True)
    check('a PIO write waits for its data', flash_done(qt), False)
    qt.writel(FLASH + 0x100, 0x44332211)
    qt.writel(FLASH + 0x100, 0x88776655)
    check('and completes with it', flash_done(qt), True)
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    img = open(path, 'rb').read()
    check('erase reached the image file',
          img[0x30000:0x30010] + img[0x30018:0x40000] ==
          b'\xff' * (0x10000 - 8), True)
    check('program reached the image file', img[0x30010:0x30018],
          bytes(range(0x11, 0x99, 0x11)))

    check('past the registers reads 0', qt.readl(FLASH + 0x300), 0)
    flash_op(qt, OP_WRITE_N, 0x02, length=0, write=True)
    check('a zero-length PIO write completes', flash_done(qt), True)
    check('and interrupts', gic_pending(qt, SPI_FLASH), True)
    qt.writel(FLASH + 0x48, 1 << 12)

    # A PIO read longer than the 4 KiB buffer streams through it
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x03, row=0x50000)
    flash_op(qt, OP_READ_N, 0x03, length=0x2000)
    check('an 8 KiB PIO read is not complete before it is drained',
          flash_done(qt), False)
    got = b''.join(qt.readl(FLASH + 0x100).to_bytes(4, 'little')
                   for _ in range(0x800))
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    check('8 KiB PIO read', got, img[0x50000:0x52000])
    check('and completes with its last word', flash_done(qt), True)

    # FAST_READ with a dummy setting under a byte of clocks
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x0B, row=0x60000)
    qt.writel(FLASH + 0xC0, 3)
    flash_op(qt, 0x1E, 0x0B)
    flash_op(qt, OP_READ_N, 0x0B, length=16)
    got = b''.join(qt.readl(FLASH + 0x100).to_bytes(4, 'little')
                   for _ in range(4))
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    check('FAST_READ after four dummy clocks', got, img[0x60000:0x60010])


# ---- nvt-flash, SPI NAND --------------------------------------------------

OP_CMD_STS, OP_SPI_PROG_CMD, OP_DATA_RD3 = 0x0B, 0x15, 0x16
NAND_MODE = 1 << 19                 # MODULE0.nand_type, spi_flash_type 0
PAGE, OOB, RAW = 2048, 64, 2112
NAND_SIZE = 128 << 20


def make_nand(path, with_oob):
    """A 128 MiB NAND: pages 0..191 numbered, page 5's OOB (if kept) set."""
    data = bytearray(b'\xff' * NAND_SIZE)
    for p in range(192):
        data[p * PAGE:(p + 1) * PAGE] = (struct.pack('<I', p) * 512)
    if not with_oob:
        open(path, 'wb').write(data)
        return
    raw = bytearray(b'\xff' * (NAND_SIZE // PAGE * RAW))
    for p in range(192):
        raw[p * RAW:p * RAW + PAGE] = data[p * PAGE:(p + 1) * PAGE]
    raw[5 * RAW + PAGE:6 * RAW] = bytes(range(0x40, 0x80))
    open(path, 'wb').write(raw)


def nand_page(path, with_oob, p):
    """(data, oob) of page p in the image file."""
    with open(path, 'rb') as f:
        if with_oob:
            f.seek(p * RAW)
            b = f.read(RAW)
            return b[:PAGE], b[PAGE:]
        f.seek(p * PAGE)
        return f.read(PAGE), None


def nand_module0(qt, row, col=2):
    qt.writel(FLASH + 0x00, NAND_MODE | ((row - 1) << 2) | (col - 1))


def nand_read_page(qt, page, col=0, length=PAGE, dma=True):
    """spinand_read_page: PAGE_READ, poll, CACHE_READX4, status."""
    nand_module0(qt, 3)
    qt.writel(FLASH + 0x38, col)
    qt.writel(FLASH + 0xB4, 0x0F | (0xC0 << 8))     # COMMAND2: cmd 5, 6
    qt.writel(FLASH + 0xBC, 1 << 8)                 # OIP == 0
    qt.writel(FLASH + 0x110, 0x01000000)
    flash_op(qt, OP_DATA_RD3, 0x13 | (0x0F << 8) | (0xC0 << 16) | (0x6B << 24),
             row=page, length=length, dma=dma)


def nand_feature(qt, addr):
    nand_module0(qt, 1)
    flash_op(qt, OP_CMD_STS, 0x0F | (addr << 8), length=1)
    return qt.readl(FLASH + 0x48) & 0xFF


def test_nand(qt, path, with_oob):
    print('nvt-flash, SPI NAND (%s)' % ('with OOB' if with_oob else
                                        'data only'))
    gic_enable(qt, SPI_FLASH)
    qt.writel(FLASH + 0x44, 1 << 12)

    nand_module0(qt, 1)
    flash_op(qt, OP_READ_ID, 0x9F, length=4)
    check('READ_ID: W25N01GV after the address byte',
          qt.readl(FLASH + 0x100), 0x0021AAEF)
    check('configuration B0: on-die ECC, buffer mode', nand_feature(qt, 0xB0),
          0x18)

    # spi_nand_unlock_bp / nand_cmd_write_status: CMD_ADDR, 2 "row" bytes
    nand_module0(qt, 2)
    flash_op(qt, OP_CMD_ADDR, 0x1F, row=(0xA0 << 8) | 0x38)
    check('SET_FEATURE A0', nand_feature(qt, 0xA0), 0x38)
    nand_module0(qt, 2)
    flash_op(qt, OP_CMD_ADDR, 0x1F, row=(0xA0 << 8) | 0x00)
    check('and back to unlocked', nand_feature(qt, 0xA0), 0)

    data, oob = nand_page(path, with_oob, 5)
    nand_read_page(qt, 5)
    check('DATA_RD3 completes', flash_done(qt), True)
    check('and interrupts', gic_pending(qt, SPI_FLASH), True)
    check('page 5 by DMA', qt.read(0x01000000, PAGE), data)
    check('its OOB in the spare SRAM', qt.read(FLASH + 0x200, OOB),
          oob or b'\xff' * OOB)
    check('status (cmd 5 + 6) in CTRL_STS: ready, no ECC error',
          qt.readl(FLASH + 0x48) & 0xFF, 0)
    qt.writel(FLASH + 0x48, 1 << 12)

    qt.write(FLASH + 0x200, b'\0' * OOB)
    nand_read_page(qt, 5, col=PAGE, length=OOB, dma=False)
    check('a spare-only read completes undrained', flash_done(qt), True)
    check('with the OOB in the spare SRAM', qt.read(FLASH + 0x200, OOB),
          oob or b'\xff' * OOB)

    # nand_cmd_erase_block: WEN, then BLK_ERSE polling GET_FEATURE C0
    nand_module0(qt, 1)
    flash_op(qt, OP_CMD, 0x06)
    check('WEN sets WEL', nand_feature(qt, 0xC0), 0x02)
    nand_module0(qt, 3)
    qt.writel(FLASH + 0xBC, 1 << 8)
    flash_op(qt, OP_BLK_ERSE, 0xD8 | (0x0F << 8) | (0xC0 << 16), row=64)
    check('block erase completes', flash_done(qt), True)
    check('with E_FAIL clear and WEL dropped',
          qt.readl(FLASH + 0x48) & 0xFF, 0)
    check('block 1 erased in the file',
          all(nand_page(path, with_oob, p)[0] == b'\xff' * PAGE
              for p in (64, 100, 127)), True)
    check('block 2 untouched', nand_page(path, with_oob, 128)[0],
          struct.pack('<I', 128) * 512)

    # spinand_program_page: spare SRAM, then SPI_PROG_CMD by DMA
    payload = random.Random(5).randbytes(PAGE)
    spare = bytes(range(0xC0, 0x100))
    qt.write(0x01000000, payload)
    qt.write(FLASH + 0x200, spare)
    nand_module0(qt, 3)
    qt.writel(FLASH + 0x38, 0)
    qt.writel(FLASH + 0xB4, 0xC0)                   # COMMAND2: cmd 5
    qt.writel(FLASH + 0xBC, 1 << 8)
    qt.writel(FLASH + 0x110, 0x01000000)
    flash_op(qt, OP_SPI_PROG_CMD, 0x06 | (0x32 << 8) | (0x10 << 16) |
             (0x0F << 24), row=67, length=PAGE, dma=True, write=True)
    check('SPI_PROG_CMD completes', flash_done(qt), True)
    check('with P_FAIL clear', qt.readl(FLASH + 0x48) & 0x08, 0)
    d, o = nand_page(path, with_oob, 67)
    check('page 67 programmed in the file', d, payload)
    if with_oob:
        check('its OOB from the spare SRAM', o, spare)
    qt.write(0x01000000, b'\0' * PAGE)
    nand_read_page(qt, 67)
    check('and reads back', qt.read(0x01000000, PAGE), payload)

    # PROGRAM EXECUTE without WEN programs nothing (W25N)
    nand_module0(qt, 3)
    flash_op(qt, OP_CMD_ADDR, 0x10, row=68)
    check('no program without WEL', nand_page(path, with_oob, 68)[0],
          b'\xff' * PAGE)


# ---- flash boot ROM ----------------------------------------------------

def fdt_blob(tree):
    """Flatten {name: {prop: bytes | {child...}}} into a DTB."""
    strings = bytearray()
    struct_ = bytearray()

    def string(name):
        i = strings.find(name.encode() + b'\0')
        if i < 0:
            i = len(strings)
            strings.extend(name.encode() + b'\0')
        return i

    def node(name, body):
        struct_.extend(struct.pack('>I', 1) + name.encode() + b'\0')
        while len(struct_) % 4:
            struct_.append(0)
        for k, v in body.items():
            if isinstance(v, bytes):
                struct_.extend(struct.pack('>III', 3, len(v), string(k)) + v)
                while len(struct_) % 4:
                    struct_.append(0)
        for k, v in body.items():
            if isinstance(v, dict):
                node(k, v)
        struct_.extend(struct.pack('>I', 2))

    node('', tree)
    struct_.extend(struct.pack('>I', 9))
    rsv = b'\0' * 16
    off_rsv = 40
    off_struct = off_rsv + len(rsv)
    off_strings = off_struct + len(struct_)
    total = off_strings + len(strings)
    hdr = struct.pack('>10I', 0xD00DFEED, total, off_struct, off_strings,
                      off_rsv, 17, 16, 0, len(strings), len(struct_))
    return hdr + rsv + bytes(struct_) + bytes(strings)


def cells(*v):
    return struct.pack('>%dI' % len(v), *v)


def lz_pack(data, marker=0xF7):
    """BCL's LZ77: literals, marker escapes, (length, offset) references."""
    def var(v):
        out = [v & 0x7F]
        v >>= 7
        while v:
            out.append(0x80 | (v & 0x7F))
            v >>= 7
        return bytes(reversed(out))
    out = bytearray([marker])
    i = 0
    while i < len(data):
        # a reference back to a 64-byte-aligned repeat, when there is one
        if i >= 64 and data[i:i + 64] == data[i - 64:i]:
            n = 64
            while i + n < len(data) and data[i + n] == data[i + n - 64]:
                n += 1
            out += bytes([marker]) + var(n) + var(64)
            i += n
            continue
        out.append(data[i])
        if data[i] == marker:
            out.append(0)
        i += 1
    return bytes(out)


def make_boot_image(path, part_size=0x40000, bcl=None, nand=False):
    rng = random.Random(7)
    head = bytes(rng.randrange(256) for _ in range(300)) + b'\xf7' * 3
    payload = head + head[-64:] * 40 + bytes(range(256))
    packed = lz_pack(payload)
    if bcl is None:
        bcl = b'BCL1' + struct.pack('>III', 9, len(payload), len(packed)) + \
            packed
    tree = {
        'nvt_memory_cfg': {
            '#address-cells': cells(1), '#size-cells': cells(1),
            'fdt': {'reg': cells(0x100000, 0x100000)},
            'shmem': {'reg': cells(0x200000, 0x100000)},
            'uboot': {'reg': cells(0x3900000, 0x6C0000)},
        },
        ('nand@f0400000' if nand else 'nor@f0400000'): {
            'compatible': b'nvt,nvt_spinand\0' if nand else
                          b'nvt,nvt_spinor\0',
            '#address-cells': cells(2), '#size-cells': cells(2),
            'partition_uboot': {'reg': cells(0, 0xC0000 if nand else 0x50000,
                                             0, part_size)},
        },
    }
    dtb = fdt_blob(tree)
    if nand:
        # cfg_566_IPCAM1_EVB: the tree in block 2, U-Boot as built
        img = bytearray(b'\xff' * NAND_SIZE)
        img[0x40000:0x40000 + len(dtb)] = dtb
        img[0xC0000:0xC0000 + len(payload)] = payload
    else:
        img = bytearray(b'\xff' * (8 << 20))
        img[0x10000:0x10000 + len(dtb)] = dtb
        img[0x50000:0x50000 + len(bcl)] = bcl
    open(path, 'wb').write(img)
    return payload, dtb


def test_boot_rom(qemu, tmp):
    print('flash boot ROM')
    path = os.path.join(tmp, 'boot.bin')
    payload, dtb = make_boot_image(path)
    qt = QTest(qemu, 'nt98562,flash-file=' + path)
    try:
        def ram(a, n):
            return b''.join(qt.readl(a + i).to_bytes(4, 'little')
                            for i in range(0, n, 4))[:n]
        check('U-Boot unpacked from BCL1 at /nvt_memory_cfg/uboot',
              ram(0x3900000, len(payload)), payload)
        check('device tree copied to /nvt_memory_cfg/fdt',
              ram(0x100000, len(dtb)), dtb)
        shm = ram(0x200000, 48)
        check('SHMINFO says LD_NVT', shm[:6], b'LD_NVT')
        check('and points at the device tree',
              struct.unpack_from('<I', shm, 44)[0], 0x100000)
        check('a branch to U-Boot at 0',
              (qt.readl(0), qt.readl(4)) == (0xE51FF004, 0x3900000), True)
    finally:
        qt.close()
    def refused(img, what):
        out = subprocess.run([qemu, '-M', 'nt98562,flash-file=' + img,
                              '-display', 'none', '-nodefaults', '-S'],
                             capture_output=True, text=True, timeout=30)
        # exit(1) with a message, not a signal
        check(what, out.returncode == 1 and
              ('novatek:' in out.stderr or 'nvt-flash:' in out.stderr), True)

    blank = os.path.join(tmp, 'blank.bin')
    open(blank, 'wb').write(b'\xff' * (8 << 20))
    refused(blank, 'an image without a device tree is refused')
    bad = os.path.join(tmp, 'bad.bin')
    make_boot_image(bad, part_size=0)
    refused(bad, 'a zero-length uboot partition is refused')
    make_boot_image(bad, part_size=8, bcl=b'BCL1\0\0\0\x09')
    refused(bad, 'a BCL1 partition shorter than its header is refused')
    make_boot_image(bad, bcl=b'BCL1' + struct.pack('>III', 9, 0xF0000000, 8) +
                    b'\xf7' * 8)
    refused(bad, 'a BCL1 header declaring 3.75 GiB is refused')
    open(bad, 'wb').write(b'\xff' * (100 << 20))
    refused(bad, 'a 100 MiB image, neither NOR nor NAND, is refused')

    nand = os.path.join(tmp, 'boot-nand.bin')
    payload, dtb = make_boot_image(nand, nand=True)
    qt = QTest(qemu, 'nt98566,flash-file=' + nand)
    try:
        check('NAND: the plain U-Boot copied to /nvt_memory_cfg/uboot',
              qt.read(0x3900000, len(payload)), payload)
        check('the device tree from block 2', qt.read(0x100000, len(dtb)),
              dtb)
        check('SHMINFO boot.LdStorage: SPI NAND',
              struct.unpack_from('<H', qt.read(0x200000 + 42, 2))[0], 2)
    finally:
        qt.close()
    os.unlink(nand)


# ---- nvt-eqos ----------------------------------------------------------

def mdio_read(qt, phy, reg):
    qt.writel(ETH + 0x200, (phy << 21) | (reg << 16) | (3 << 2) | 1)
    check('MDIO read %d completes' % reg, qt.readl(ETH + 0x200) & 1, 0)
    return qt.readl(ETH + 0x204) & 0xFFFF


def test_eqos(qt, pcap):
    print('nvt-eqos')
    gic_enable(qt, SPI_ETH)
    check('HW_FEATURE0: MII and MDIO (SMASEL)',
          qt.readl(ETH + 0x11C) & 0x21, 0x21)
    check('HW_FEATURE2: one Tx and one Rx queue', qt.readl(ETH + 0x124), 0)
    check('BMSR at PHY 1: link up, autoneg done',
          mdio_read(qt, 1, 1) & 0x24, 0x24)
    check('no PHY at address 0', mdio_read(qt, 0, 1), 0xFFFF)

    qt.writel(ETH + 0x1000, 1)
    check('DMA software reset is immediate', qt.readl(ETH + 0x1000) & 1, 0)

    # One Tx descriptor at 0x00100000, a 60-byte frame at 0x00101000
    frame = bytes.fromhex('ffffffffffff525400123456') + b'\x08\x06' + \
        bytes(range(46))
    for i in range(0, len(frame), 4):
        qt.writel(0x00101000 + i,
                  int.from_bytes(frame[i:i + 4].ljust(4, b'\0'), 'little'))
    qt.writel(0x00100000, 0x00101000)                       # TDES0
    qt.writel(0x00100004, 0)                                # TDES1
    qt.writel(0x00100008, (1 << 31) | len(frame))           # IOC, length
    qt.writel(0x0010000C, (1 << 31) | (1 << 29) | (1 << 28) | len(frame))
    qt.writel(ETH + 0x1100 + 0x14, 0x00100000)              # TDLAR
    qt.writel(ETH + 0x1100 + 0x2C, 3)                       # 4 descriptors
    qt.writel(ETH + 0x1100 + 0x34, (1 << 15) | 1)           # NIE, TIE
    qt.writel(ETH + 0x0000, 3)                              # TE, RE
    qt.writel(ETH + 0x1100 + 0x04, 1)                       # ST
    qt.writel(ETH + 0x1100 + 0x08, 1)                       # SR
    check('Rx DMA running, waiting for a packet (RPS0 = 3)',
          (qt.readl(ETH + 0x100C) >> 8) & 0xF, 3)
    qt.writel(ETH + 0x1100 + 0x20, 0x00100010)              # tail: 1 desc
    tdes3 = qt.readl(0x0010000C)
    check('Tx write-back: OWN clear, FD/LD, no error bits',
          tdes3, (1 << 29) | (1 << 28))
    check('TI in the channel status', qt.readl(ETH + 0x1160) & 1, 1)
    check('SPI 34 pending', gic_pending(qt, SPI_ETH), True)
    check('ETH_ISR summarises it',
          (qt.readl(ETH + 0x3008) & 0x3F) != 0, True)
    qt.writel(ETH + 0x1160, 1)
    check('write-1-to-clear releases it', gic_pending(qt, SPI_ETH), False)
    check('and ETH_ISR with it', qt.readl(ETH + 0x3008), 0)
    qt.cmd('clock_step 1000')
    data = open(pcap, 'rb').read()
    check('the frame went out on the wire', frame in data, True)

    # A frame in two descriptors, the tail moved past only the first
    part1, part2 = frame[:20], bytes(reversed(frame[20:]))
    write_bytes(qt, 0x00102000, part1)
    write_bytes(qt, 0x00103000, part2)
    tx_desc(qt, 1, 0x00102000, len(part1), (1 << 29))              # FD
    tx_desc(qt, 2, 0x00103000, len(part2), (1 << 28))              # LD
    qt.writel(ETH + 0x1100 + 0x20, 0x00100020)
    qt.writel(ETH + 0x1100 + 0x20, 0x00100030)
    qt.cmd('clock_step 1000')
    data = open(pcap, 'rb').read()
    check('a frame split across tail writes goes out whole',
          (part1 + part2) in data, True)

    # Over-long: 2 x 8000 bytes, more than the MAC takes
    tx_desc(qt, 3, 0x00110000, 8000, (1 << 29))
    tx_desc(qt, 0, 0x00110000, 8000, (1 << 28))
    qt.writel(ETH + 0x1100 + 0x20, 0x00100010)
    check('an over-long frame is dropped with ES / JT',
          qt.readl(0x0010000C) & 0xC000, 0xC000)


def write_bytes(qt, addr, data):
    for i in range(0, len(data), 4):
        qt.writel(addr + i,
                  int.from_bytes(data[i:i + 4].ljust(4, b'\0'), 'little'))


def tx_desc(qt, idx, buf, length, flags):
    a = 0x00100000 + 16 * idx
    qt.writel(a, buf)
    qt.writel(a + 4, 0)
    qt.writel(a + 8, length)
    qt.writel(a + 12, (1 << 31) | flags | length)


MAC = bytes.fromhex('525400123456')


def test_eqos_rx(qt, sock, port):
    print('nvt-eqos receive')
    ring, bufs = 0x00200000, 0x00210000
    qt.writel(ETH + 0x300, 0x80000000 | MAC[4] | (MAC[5] << 8))
    qt.writel(ETH + 0x304, int.from_bytes(MAC[:4], 'little'))
    for i in range(4):
        qt.writel(ring + 16 * i, bufs + 0x100 * i)
        qt.writel(ring + 16 * i + 12, (1 << 31) | (1 << 30) | (1 << 24))
    qt.writel(ETH + 0x1100 + 0x1C, ring)                    # RDLAR
    qt.writel(ETH + 0x1100 + 0x30, 3)                       # 4 descriptors
    qt.writel(ETH + 0x1100 + 0x28, ring + 0x30)             # tail: desc 3
    qt.writel(ETH + 0x0000, 3)
    qt.writel(ETH + 0x1100 + 0x08, 1 | (64 << 1))           # SR, RBSZ 64

    def send(frame):
        sock.sendto(frame, ('127.0.0.1', port))
        for _ in range(20):
            qt.cmd('clock_step 1000000')

    def rdes3(i):
        return qt.readl(ring + 16 * i + 12)

    other = bytes.fromhex('020000000001') + MAC + b'\x08\x00' + bytes(50)
    send(other)
    check('a frame for another station is filtered out',
          rdes3(0) >> 31, 1)
    big = MAC + bytes.fromhex('020000000002') + b'\x08\x00' + \
        bytes(range(86))
    send(big)
    check('a 100-byte frame spans two 64-byte buffers: FD on the first',
          rdes3(0) & 0xB0000000, 0x20000000)
    check('LD and the frame length on the second',
          rdes3(1) & 0xB0007FFF, 0x10000000 | len(big))
    got = b''.join(qt.readl(bufs + i).to_bytes(4, 'little')
                   for i in range(0, 64, 4)) + \
        b''.join(qt.readl(bufs + 0x100 + i).to_bytes(4, 'little')
                 for i in range(0, 36, 4))
    check('and the bytes are the frame', got[:len(big)], big)
    small = MAC + bytes.fromhex('020000000003') + b'\x08\x00' + bytes(30)
    send(small)
    check('the next frame takes descriptor 2', rdes3(2) >> 28 & 3, 3)
    send(small)
    check('descriptor 3 is the tail: the DMA stops short of it',
          rdes3(3) >> 31, 1)
    qt.writel(ETH + 0x1100 + 0x28, ring)                    # tail: desc 0
    for _ in range(20):
        qt.cmd('clock_step 1000000')
    check('and fills it once the tail moves on', rdes3(3) >> 28 & 3, 3)


# ldr r0,=0xF0290000; FCR = 7; 1: wait for LSR.DR; RBR -> THR; b 1b
UART_ECHO = [0xE59F001C, 0xE3A01007, 0xE5801008, 0xE5901014, 0xE3110001,
             0x0AFFFFFC, 0xE5902000, 0xE5802000, 0xEAFFFFF9, 0xF0290000]


def test_uart_rx_after_fifo_reset(qemu, tmp):
    """Bytes queued on the backend before the guest initialises its UART:
    one lands in RBR (FIFO off) and the backend stops polling.  The guest
    then resets the receive FIFO and spins on LSR, never reading RBR -- as
    U-Boot's ns16550 tstc() does at "Hit any key to stop autoboot".  A
    spinning vCPU makes no main-loop events, so without
    qemu/patches/0003 nothing more ever arrives.  Needs a real guest:
    qtest's own commands wake the main loop and hide the stall."""
    print('UART input across a receive-FIFO reset')
    kernel = os.path.join(tmp, 'echo.bin')
    open(kernel, 'wb').write(struct.pack('<%dI' % len(UART_ECHO), *UART_ECHO))
    ser_path = os.path.join(tmp, 'echo-ser.sock')
    mon_path = os.path.join(tmp, 'echo-mon.sock')
    proc = subprocess.Popen(
        [qemu, '-M', 'nt98562', '-display', 'none', '-nodefaults', '-S',
         '-kernel', kernel,
         '-serial', 'unix:%s,server=on,wait=off' % ser_path,
         '-monitor', 'unix:%s,server=on,wait=off' % mon_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(100):
            if os.path.exists(ser_path) and os.path.exists(mon_path):
                break
            time.sleep(0.05)
        ser = socket.socket(socket.AF_UNIX)
        ser.connect(ser_path)
        mon = socket.socket(socket.AF_UNIX)
        mon.connect(mon_path)
        ser.sendall(b'abc')
        # Still paused: wait until 'a' really sits in RBR (LSR.DR, read
        # through the monitor, which does not touch RBR) -- the state the
        # stall needs -- before the guest runs.
        mon.settimeout(0.2)
        dr = False
        for _ in range(100):
            mon.sendall(b'xp /1wx 0x%x\n' % (UART0 + 0x14))
            time.sleep(0.05)
            try:
                if re.search(rb': 0x[0-9a-f]*[13579bdf]\b', mon.recv(4096)):
                    dr = True
                    break
            except socket.timeout:
                pass
        check('a byte waits in RBR before the guest runs', dr, True)
        mon.sendall(b'cont\n')
        got = b''
        ser.settimeout(0.2)
        end = time.time() + 3
        while time.time() < end and b'c' not in got:
            try:
                got += ser.recv(64)
            except socket.timeout:
                pass
        # 'a' sat in RBR and went with the FIFO reset, as on silicon
        check('input queued before UART init is echoed after its FIFO '
              'reset', got, b'bc')
        mon.sendall(b'quit\n')
        proc.wait(timeout=10)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def find_nodes(dtb, names):
    """Offsets of FDT_BEGIN_NODE tokens naming these nodes, -1 if absent."""
    return [dtb.find(b'\0\0\0\x01' + n.encode() + b'\0') for n in names]


def test_fdt(qemu, tmp):
    print('generated device tree')
    kernel = os.path.join(tmp, 'kernel.bin')
    open(kernel, 'wb').write(b'\xfe\xff\xff\xea')       # b .
    dtb = os.path.join(tmp, 'nt98562.dtb')
    subprocess.run([qemu, '-M', 'nt98562', '-machine', 'dumpdtb=' + dtb,
                    '-kernel', kernel, '-display', 'none', '-nodefaults'],
                   capture_output=True, timeout=30)
    data = open(dtb, 'rb').read() if os.path.exists(dtb) else b''
    check('dumped', data[:4] == b'\xd0\x0d\xfe\xed', True)
    check('root compatible matches mach-nvt-ivot',
          b'novatek,na51089\0nvt,ca9\0' in data, True)
    cg, pclk = find_nodes(data, ['cg@f0020000', 'periph_clk'])
    check('nvt,core_clk before nvt,periph_clk', 0 < cg < pclk, True)
    order = find_nodes(data, ['sdio', 'sdio2', 'uart', 'eth', 'misc'])
    check('pinmux children in PIN_FUNC order',
          all(0 < a < b for a, b in zip(order, order[1:])), True)
    check('chip-ID node for nvt_get_chip_id()', b'nvt,nvt_top' in data, True)
    check('the NOR under spinor.c', b'nvt,nvt_spinor\0' in data and
          b'spi_nor.0\0' in data, True)

    nand = os.path.join(tmp, 'fdt-nand.bin')
    with open(nand, 'wb') as f:
        f.truncate(NAND_SIZE)
    subprocess.run([qemu, '-M', 'nt98566,flash-file=' + nand,
                    '-machine', 'dumpdtb=' + dtb, '-kernel', kernel,
                    '-display', 'none', '-nodefaults'],
                   capture_output=True, timeout=30)
    data = open(dtb, 'rb').read()
    check('a NAND image gets the node spinand.c binds',
          b'nvt,nvt_spinand\0' in data and b'spi_nand.0\0' in data and
          b'nvt,nvt_spinor' not in data, True)
    os.unlink(nand)


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


def test_migration(qemu, tmp):
    print('migration')
    for m in ('nt98562', 'nt98566'):
        mig = os.path.join(tmp, m + '.mig')
        out = hmp(qemu, ['-M', m, '-S'],
                  ['migrate file:' + mig, 'info migrate'])
        check('%s: state saved' % m, 'completed' in out and
              'Assertion' not in out and os.path.getsize(mig) > 0, True)
        out = hmp(qemu, ['-M', m, '-S', '-incoming', 'file:' + mig],
                  ['info status'])
        check('%s: and loaded back' % m, 'error' not in out.lower() and
              'Assertion' not in out, True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='./qemu-src/build/qemu-system-arm')
    args = ap.parse_args()
    tmp = tempfile.mkdtemp()

    # The RTC on the virtual clock, so clock_step moves it.
    qt = QTest(args.qemu, 'nt98562', ['-rtc', 'clock=vm'])
    try:
        test_top_cg(qt)
        test_wdt(qt)
        test_gpio(qt)
        test_rtc(qt)
        test_uart(qt)
    finally:
        qt.close()

    flash = os.path.join(tmp, 'flash.bin')
    open(flash, 'wb').write(random.Random(3).randbytes(8 << 20))
    # -kernel keeps the boot ROM, which wants an NVT image, out of it
    stub = os.path.join(tmp, 'stub.bin')
    open(stub, 'wb').write(b'\xfe\xff\xff\xea')     # b .
    qt = QTest(args.qemu, 'nt98562,flash-file=' + flash, ['-kernel', stub])
    try:
        test_flash(qt, flash)
    finally:
        qt.close()
    test_boot_rom(args.qemu, tmp)

    for with_oob in (False, True):
        nand = os.path.join(tmp, 'nand.bin')
        make_nand(nand, with_oob)
        qt = QTest(args.qemu, 'nt98566,flash-file=' + nand, ['-kernel', stub])
        try:
            test_nand(qt, nand, with_oob)
        finally:
            qt.close()
        os.unlink(nand)

    pcap = os.path.join(tmp, 'eth.pcap')
    qt = QTest(args.qemu, 'nt98562',
               ['-netdev', 'user,id=n0,restrict=on',
                '-nic', 'none',
                '-global', 'nvt-eqos.netdev=n0',
                '-object', 'filter-dump,id=d0,netdev=n0,file=' + pcap])
    try:
        test_eqos(qt, pcap)
    finally:
        qt.close()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    peer = sock.getsockname()[1]
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
    probe.close()
    qt = QTest(args.qemu, 'nt98562',
               ['-netdev', 'socket,id=n0,udp=127.0.0.1:%d,'
                'localaddr=127.0.0.1:%d' % (peer, port),
                '-nic', 'none', '-global', 'nvt-eqos.netdev=n0'])
    try:
        test_eqos_rx(qt, sock, port)
    finally:
        qt.close()
        sock.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        print('nt98566')
        check('same die, same chip ID', qt.readl(TOP + 0xF0) >> 16, 0x7021)
    finally:
        qt.close()

    test_uart_rx_after_fifo_reset(args.qemu, tmp)
    test_fdt(args.qemu, tmp)
    test_migration(args.qemu, tmp)

    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
