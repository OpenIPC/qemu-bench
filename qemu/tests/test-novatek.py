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
  - nvt-sdio with an SD card, the way na51055_mmc.c and U-Boot drive it:
    card init (CMD0/8/55/41/2/3/7, R2 in RSP3..0 as the drivers read
    it), single- and multi-block reads by descriptor-table DMA (Linux)
    and by plain DMA with 0x1FC set (U-Boot), a DMA write, PIO reads and
    writes through the data port with FIFO_STS full / count / empty and
    DATA_END after the last word, the 8-byte SCR by PIO, the status
    interrupt on SPI 30; the CG reset pulse (0x84 bit 2) returning the
    host to descriptor mode; card detect on C_GPIO 9; an empty slot
    timing out; SDIO3 with an eMMC: CMD1 / 2 / 3 / 7, the 512-byte
    EXT_CSD by descriptor DMA, and its CG reset bit (0x84 bit 14);
  - nvt-usb, the FOTG200 host as ehci-na51089.c sees it: EHCI
    capabilities (CAPLENGTH 0x10, one port) at 0xF0600000 and at the
    0xFF600000 alias the driver uses on NA51089, PORTSC at 0x30 showing a
    usb-storage stick, the OTG register's [23:22] speed (high), the glue
    registers and PHY as storage, a port reset enabling the port; with no
    device, no connect and speed 0;
  - nvt-i2c, the way i2c-nvt.c drives it, against an SC401AI on I2C 0
    (sensor=sc401ai): the chip ID ipcinfo reads (a register-address write,
    then a repeated-START read), a write and a read that each span two
    four-byte chunks, the DT / DR / BERR status and SPI 41, a NACK from an
    empty address followed by the driver's lone STOP, an i2cdetect-style
    quick write, and the empty second and third controllers;
  - nvt-pwm, the way pwm-na51055.c drives it: PERIOD and its extension,
    ENABLE starting a channel (and its "out" line, high with a nonzero
    duty), DISABLE stopping it with INT_STS and SPI 26, a cycle-count run
    ending on its own after cycles x base period at 30 MHz, a divider load
    flagging CLKDIV_LOAD_STS;
  - nvt-timer, the way timer_cfg() and U-Boot drive it: timer 0
    running free at 1 MHz out of reset, a one-shot timer counting to its
    target, flagging STS and SPI 0 and stopping, a free-running one on
    divider 1 wrapping and going on, a divider change mid-period (and
    repeated ones at 600 kHz keeping every tick), 3 MHz periods without
    drift, a switch to one shot mid-period, RELOAD,
    a disabled timer holding its count, STS without INT raising nothing,
    and a machine reset restoring timer 0;
  - nvt-crypto, the way na51089_crypto.c and U-Boot drive it: a PIO
    AES block (FIPS-197), and by descriptor DMA AES-CBC and CTR
    (SP 800-38A; CV written back, a partial last block, a block split
    across two descriptor entries), GCM (the McGrew
    and Viega cases; the tag from S0 and GHASH), DES and 3DES, the OTP
    key manager's (blank, zero) key and "get S0", with SPI 38;
  - nvt-hash: SHA-1 / SHA-256 padded by the engine, a message in two
    chunks chained through the IV registers, HMAC-SHA-1 / -256 (RFC 2202,
    4231), an unreadable chunk leaving no digest, SPI 28;
  - nvt-rsa: 256, 512 and 2048-bit modular exponentiation through the
    SRAM ports, against Python's pow(), SPI 20; a CRC32 key check never
    reporting done;
  - nvt-media, the HDAL engines' handshakes: SIE frames in three steps
    (VD, break points, ends) with single-mode outputs completing; an IME
    linked list of two jobs, its updates applied with their byte enables,
    frame / job / list end and SPI 6; the graphics engine's list end (no
    DMA idle), single job and reset; ISE's list; H.26x flags, reset and a
    picture (BS_LEN, a filler NAL); a JPEG of four grey MCUs;
  - nvt-ide: DMA idle with the engine off; load pending to the frame, then
    frame end and SPI 48, write 1 to clear; a screendump of an NV12 video
    layer and an ARGB4444 OSD box blended over it;
  - nvt-dsi: escape and ULPS triggers clearing themselves with their
    done bits, manual TX dropping with TX done, automatic mode's frame
    ends until TX goes off, write 1 to clear, SPI 50;
  - the AI engines: idle bits; NUE fully connected (dot products, shifts,
    offset, s32) with frame end and SPI 14; NUE2 NV12 to RGB, scaled,
    less a DC mean; a CNN list of two jobs fired by bit-60 updates, with
    a bit-59 update adding the list base;
  - nvt-audio: the EAC strobes; capture DMA latching its buffer and the
    queued next, the test tone in it (1 kHz left, 1.5 kHz right), the
    current-address register, stop on the channel enable, the ADC enable
    muting it, the global enable gating it, the rate from the CG, also
    when it changes mid-stream; playback DMA stopping on its channel
    enable, and playing to a wav audiodev;
  - nvt-otp: power-up and ready, a read starting, finishing and reading a
    blank word, the done bit cleared by writing it back;
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
import hashlib
import lzma
import os
import random
import re
import shutil
import socket
import math
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

    def close(self, graceful=False):
        if graceful:                    # let QEMU clean up (audio files)
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        else:
            self.proc.kill()
        self.proc.wait()
        self.sock.close()
        shutil.rmtree(self.dir, ignore_errors=True)


failures = 0


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    if isinstance(got, bytes):
        shown = got[:8].hex() + ('...' if len(got) > 8 else '')
    else:
        shown = got if isinstance(got, (bool, str, tuple, list)) else hex(got)
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


def nor_cmd(qt, cmd, row=None, read=0, data=None):
    """One SPI NOR command under manual CS: cmd [address] [data | read]."""
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    if row is None:
        flash_op(qt, OP_CMD, cmd)
    else:
        flash_op(qt, OP_CMD_ADDR, cmd, row=row)
    got = None
    if data is not None:
        flash_op(qt, OP_WRITE_N, cmd, length=1, write=True)
        qt.writel(FLASH + 0x100, data)
    elif read:
        flash_op(qt, OP_READ_N, cmd, length=read)
        got = qt.readl(FLASH + 0x100) & 0xFF
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    return got


def test_flash_locks(qt, path):
    """W25Q per-block protection, as the NT98332 (Xiongmai) kernel uses it:
    SR3's WPS on, lock bits powering up set, 0x39 to unlock what it writes."""
    print('nvt-flash: W25Q block locks')
    qt.writel(FLASH + 0x00, 2 << 2)             # 3 row-address bytes

    def erase(row):
        flash_op(qt, OP_CMD, 0x06)
        qt.writel(FLASH + 0xBC, (1 << 8) | 0)
        flash_op(qt, OP_BLK_ERSE, 0xD8 | (0x05 << 8), row=row)

    def erased(row):
        return open(path, 'rb').read()[row:row + 0x10000] == \
            b'\xff' * 0x10000

    def locked(row):
        return nor_cmd(qt, 0x3D, row=row, read=1) & 1

    check('SR3 out of reset: 0x60, WPS clear', nor_cmd(qt, 0x15, read=1),
          0x60)
    erase(0x30000)
    check('WPS clear: the locks do not apply', erased(0x30000), True)

    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x11, data=0x64)
    check('WRSR3 sets WPS', nor_cmd(qt, 0x15, read=1), 0x64)
    check('the lock bits power up set', locked(0x40000), 1)
    erase(0x40000)
    check('an erase of a locked block is dropped', erased(0x40000), False)

    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x39, row=0x40000)
    check('0x39 unlocks the block', locked(0x40000), 0)
    check('all of its 64 KiB', locked(0x4F000), 0)
    check('and no more', locked(0x50000), 1)
    erase(0x40000)
    check('then the erase lands', erased(0x40000), True)

    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x39, row=0x7F0000)
    check('the top block unlocks per 4 KiB sector', locked(0x7F0000), 0)
    check('its next sector stays locked', locked(0x7F1000), 1)

    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x36, row=0x40000)
    check('0x36 locks it again', locked(0x40000), 1)
    nor_cmd(qt, 0x39, row=0x50000)
    check('an unlock without WREN is ignored', locked(0x50000), 1)
    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x98)
    check('0x98 unlocks everything', (locked(0x50000), locked(0x7F1000)),
          (0, 0))


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

    # A column past the OOB reads nothing, whatever DATALEN says
    qt.write(FLASH + 0x200, b'\x5a' * 0x80)
    nand_read_page(qt, 5, col=0xFFF, length=0x100000, dma=False)
    check('a column past the page completes', flash_done(qt), True)
    check('and leaves the spare SRAM alone', qt.read(FLASH + 0x200, 0x80),
          b'\x5a' * 0x80)
    check('the controller still answers', nand_feature(qt, 0xB0), 0x18)

    # RESET returns B0 to its power-on value
    nand_module0(qt, 2)
    flash_op(qt, OP_CMD_ADDR, 0x1F, row=(0xB0 << 8) | 0x08)
    check('SET_FEATURE B0: ECC off', nand_feature(qt, 0xB0), 0x08)
    flash_op(qt, OP_CMD, 0xFF)
    check('RESET restores ECC-E', nand_feature(qt, 0xB0), 0x18)

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


# ---- nvt-sdio --------------------------------------------------------------

SDIO = 0xF0420000
SDIO2 = 0xF0500000
SPI_SDIO = 30
SD_CMD_SENT, SD_RSP_OK, SD_RSP_TIMEOUT = 0x40, 0x10, 0x04
SD_DATA_END, SD_DATA_OK = 0x80, 0x20


def sd_cmd(qt, idx, arg=0, rsp='short', base=SDIO):
    """sdiohost_sendcmd: clear the status, ARG, CMD, then CMD_EN."""
    qt.writel(base + 0x28, 0xFFFFFFFF)
    qt.writel(base + 0x04, arg)
    v = idx | {'none': 0, 'short': 1 << 6, 'long': 1 << 7}[rsp]
    qt.writel(base, v)
    qt.writel(base, v | (1 << 9))
    return qt.readl(base + 0x28)


def sd_data(qt, length, blksz=512, dma=True, write=False, addr=0,
            base=SDIO):
    """sdiohost_setupdatatransfer{dma,pio}: before the command."""
    qt.writel(base + 0x1C, (blksz << 16))
    qt.writel(base + 0x110, addr)
    qt.writel(base + 0x104, length)
    fifo = (2 if dma else 0) | (4 if write else 0)
    qt.writel(base + 0x10C, fifo)
    qt.writel(base + 0x1C, (blksz << 16) | (1 << 6))
    qt.writel(base + 0x10C, fifo | 1)


SDIO3 = 0xF0510000


def test_emmc(qt):
    print('nvt-sdio: SDIO3 with an eMMC')
    qt.writel(SDIO3 + 0x30, 0xFF)
    sd_cmd(qt, 0, rsp='none', base=SDIO3)
    for _ in range(10):                             # CMD1 until not busy
        sd_cmd(qt, 1, 0x40FF8080, base=SDIO3)
        if qt.readl(SDIO3 + 0x08) >> 31:
            break
    check('CMD1 (SEND_OP_COND): powered up (OCR bit 31)',
          qt.readl(SDIO3 + 0x08) >> 31, 1)
    sd_cmd(qt, 2, rsp='long', base=SDIO3)
    sd_cmd(qt, 3, 1 << 16, base=SDIO3)              # the host picks the RCA
    sd_cmd(qt, 7, 1 << 16, base=SDIO3)
    qt.write(0x00800000, struct.pack('<3I', 1 | 2 | (2 << 4), 512, 0x01000000))
    sd_data(qt, 512, addr=0x00800000, base=SDIO3)
    sts = sd_cmd(qt, 8, 0, base=SDIO3)
    ext = qt.read(0x01000000, 512)
    check('CMD8 (SEND_EXT_CSD) by descriptor DMA', sts & SD_DATA_END,
          SD_DATA_END)
    check('EXT_CSD: a revision, and SEC_COUNT for 4 MiB',
          (ext[192] > 0, struct.unpack_from('<I', ext, 212)[0]),
          (True, (4 << 20) // 512))
    qt.writel(SDIO3 + 0x1FC, 1)
    qt.writel(CG + 0x84, qt.readl(CG + 0x84) & ~(1 << 14))
    qt.writel(CG + 0x84, qt.readl(CG + 0x84) | (1 << 14))
    check('CG 0x84 bit 14 resets SDIO3', qt.readl(SDIO3 + 0x1FC), 0)


def test_sdio(qt, card):
    print('nvt-sdio')
    img = open(card, 'rb').read()
    gic_enable(qt, SPI_SDIO)
    check('BUS_STATUS: card not busy', qt.readl(SDIO + 0x40) & 1, 1)
    check('card detect: C_GPIO 9 low with a card in',
          qt.readl(GPIO + 0x00) >> 9 & 1, 0)
    qt.writel(SDIO, 1 << 10)
    check('SDC_RST clears itself', qt.readl(SDIO) & (1 << 10), 0)
    qt.writel(SDIO + 0x30, 0xFF)

    check('CMD0: CMD_SENT', sd_cmd(qt, 0, rsp='none'), SD_CMD_SENT)
    check('and SPI 30', gic_pending(qt, SPI_SDIO), True)
    check('CMD8: R7 echoes the check pattern',
          (sd_cmd(qt, 8, 0x1AA), qt.readl(SDIO + 0x08) & 0xFFF),
          (SD_CMD_SENT | SD_RSP_OK, 0x1AA))
    sd_cmd(qt, 55)
    sd_cmd(qt, 41, 0x40FF8000)
    check('ACMD41: powered up', qt.readl(SDIO + 0x08) >> 31, 1)
    sd_cmd(qt, 2, rsp='long')
    check('CMD2: CID MID in RSP3[31:24] (resp[0])',
          qt.readl(SDIO + 0x14) >> 24, 0xAA)
    check('and the end bit in RSP0[0]', qt.readl(SDIO + 0x08) & 1, 1)
    sd_cmd(qt, 3)
    rca = qt.readl(SDIO + 0x08) >> 16
    check('CMD3: an RCA', rca != 0, True)
    sd_cmd(qt, 7, rca << 16)
    sd_cmd(qt, 16, 512)

    # Linux: a descriptor table, here two 256-byte segments
    qt.write(0x00800000, struct.pack('<6I', 1 | (2 << 4), 256, 0x01000000,
                                     1 | 2 | (2 << 4), 256, 0x01100000))
    sd_data(qt, 512, addr=0x00800000)
    sts = sd_cmd(qt, 17, 3 * 512)
    check('CMD17 by descriptor DMA: command and data done together', sts,
          SD_CMD_SENT | SD_RSP_OK | SD_DATA_END | SD_DATA_OK)
    check('block 3 in the two segments',
          qt.read(0x01000000, 256) + qt.read(0x01100000, 256),
          img[3 * 512:4 * 512])
    check('FIFO_EN clears with the transfer', qt.readl(SDIO + 0x10C) & 1, 0)

    # U-Boot: 0x1FC = 1, the buffer address itself
    qt.writel(SDIO + 0x1FC, 1)
    sd_data(qt, 4 * 512, addr=0x01200000)
    sts = sd_cmd(qt, 18, 20 * 512)
    sd_cmd(qt, 12)
    check('CMD18 by plain DMA: four blocks', qt.read(0x01200000, 2048),
          img[20 * 512:24 * 512])
    check('DATA_END', sts & SD_DATA_END, SD_DATA_END)
    payload = random.Random(11).randbytes(512)
    qt.write(0x01300000, payload)
    sd_data(qt, 512, addr=0x01300000, write=True)
    sts = sd_cmd(qt, 24, 10 * 512)
    check('CMD24 by DMA completes', sts & SD_DATA_END, SD_DATA_END)
    check('block 10 written to the card image',
          open(card, 'rb').read()[10 * 512:11 * 512], payload)

    # A descriptor table shorter than DATA_LENGTH fails, not succeeds
    qt.writel(SDIO + 0x1FC, 0)
    qt.write(0x00800000, struct.pack('<3I', 1 | 2 | (2 << 4), 256, 0x01000000))
    sd_data(qt, 512, addr=0x00800000)
    sts = sd_cmd(qt, 17, 3 * 512)
    check('descriptors 256 bytes short: DATA_TIMEOUT | DMA_ERROR, no DATA_END',
          sts & (0x08 | (1 << 19) | SD_DATA_END), 0x08 | (1 << 19))
    sd_cmd(qt, 12)
    # A command the card does not answer leaves no data phase armed
    sd_data(qt, 512, addr=0x00800000)
    qt.writel(SDIO + 0x28, 0xFFFFFFFF)
    qt.writel(SDIO + 0x04, 0)
    qt.writel(SDIO, 5 | (1 << 6) | (1 << 9))          # CMD5: no SDIO
    check('a timed-out command disarms its data phase',
          (qt.readl(SDIO + 0x28) & 0x0C, qt.readl(SDIO + 0x10C) & 1,
           qt.readl(SDIO + 0x1C) >> 6 & 1), (0x04, 0, 0))
    qt.writel(SDIO + 0x1FC, 1)

    # PIO, as for unaligned buffers
    sd_data(qt, 512, dma=False)
    sts = sd_cmd(qt, 17, 3 * 512)
    check('PIO CMD17: command done, data waiting', sts & SD_DATA_END, 0)
    check('FIFO_STS full, 16 words', qt.readl(SDIO + 0x108), 0x200 | 16)
    got = b''.join(qt.readl(SDIO + 0x100).to_bytes(4, 'little')
                   for _ in range(128))
    check('block 3 through the data port', got, img[3 * 512:4 * 512])
    check('DATA_END after the last word',
          qt.readl(SDIO + 0x28) & SD_DATA_END, SD_DATA_END)
    check('FIFO_STS empty', qt.readl(SDIO + 0x108), 0x100)
    payload = random.Random(12).randbytes(512)
    sd_data(qt, 512, dma=False, write=True)
    sd_cmd(qt, 24, 11 * 512)
    for i in range(0, 512, 4):
        qt.writel(SDIO + 0x100, int.from_bytes(payload[i:i + 4], 'little'))
    check('PIO CMD24: DATA_END after the last word',
          qt.readl(SDIO + 0x28) & SD_DATA_END, SD_DATA_END)
    check('block 11 written to the card image',
          open(card, 'rb').read()[11 * 512:12 * 512], payload)
    sd_cmd(qt, 55, rca << 16)
    sd_data(qt, 8, blksz=8, dma=False)
    sd_cmd(qt, 51)
    check('ACMD51: the 8-byte SCR waits as two words',
          qt.readl(SDIO + 0x108) & 0x3F, 2)
    scr = qt.readl(SDIO + 0x100).to_bytes(4, 'little')
    qt.readl(SDIO + 0x100)
    check('SCR structure version 0 (byte 0 high nibble)', scr[0] >> 4, 0)
    check('and DATA_END', qt.readl(SDIO + 0x28) & SD_DATA_END, SD_DATA_END)

    # nvt_composite_gate_clk_prepare: SYS_RESET1 bit 2 low, then high
    qt.writel(CG + 0x84, qt.readl(CG + 0x84) & ~(1 << 2))
    qt.writel(CG + 0x84, qt.readl(CG + 0x84) | (1 << 2))
    check('the CG reset pulse returns it to descriptor DMA',
          (qt.readl(SDIO + 0x1FC), qt.readl(SDIO + 0x30)), (0, 0))
    check('and drops the interrupt', gic_pending(qt, SPI_SDIO), False)

    check('the empty SDIO2 slot times out',
          sd_cmd(qt, 8, 0x1AA, base=SDIO2), SD_CMD_SENT | SD_RSP_TIMEOUT)


# ---- nvt-i2c -----------------------------------------------------------------

I2C0, I2C1, I2C2 = 0xF0220000, 0xF0350000, 0xF03A0000
SPI_I2C0 = 41
I2C_DT, I2C_DR, I2C_BERR = 1 << 4, 1 << 5, 1 << 6
I2C_INTR_ALL = 0x1473                       # i2c_nvtim_init()


def i2c_chunk(qt, n, data=None, start=False, stop=False, nack=False,
              base=I2C0):
    """nvtim_set_data / nvtim_set_rxconfig: one chunk of n bytes."""
    cfg = n << 16
    if start:
        cfg |= 1 << 8
    if stop:
        cfg |= 1 << (12 + n - 1)
    if nack:
        cfg |= 1 << (4 + n - 1)
    qt.writel(base + 0x04, 0xFFFFFFFF)              # clear status
    qt.writel(base + 0x08, cfg)
    if data is not None:
        qt.writel(base + 0x10, data)
    qt.writel(base + 0x00, qt.readl(base + 0x00) | (1 << 3))
    return qt.readl(base + 0x04)


def i2c_write(qt, addr, payload, stop=True):
    """i2c_nvtim_xfer_msg() for a write: the address, then four-byte
    chunks; returns the last chunk's status."""
    stream = bytes([addr << 1]) + bytes(payload)
    for i in range(0, len(stream), 4):
        part = stream[i:i + 4]
        last = i + 4 >= len(stream)
        sts = i2c_chunk(qt, len(part), int.from_bytes(part, 'little'),
                        start=(i == 0), stop=last and stop)
        if sts & I2C_BERR:
            return sts
    return sts


def i2c_read(qt, addr, length):
    """The read side: the first chunk carries the address byte."""
    out = b''
    remain = length + 1
    first = True
    while remain:
        n = min(remain, 4)
        remain -= n
        sts = i2c_chunk(qt, n, (addr << 1 | 1) if first else None,
                        start=first, stop=not remain, nack=not remain)
        if sts & I2C_BERR:
            return sts, None
        data = qt.readl(I2C0 + 0x10).to_bytes(4, 'little')
        out += data[1 if first else 0:n]
        first = False
    return sts, out


def test_i2c(qt):
    print('nvt-i2c (sensor=sc401ai)')
    gic_enable(qt, SPI_I2C0)
    qt.writel(I2C0 + 0x00, 2)                       # I2C_CTRL_REG_DEFAULT
    check('idle: BUSFREE set, BUSBUSY clear',
          qt.readl(I2C0 + 0x04) & ((1 << 28) | (1 << 3)), 1 << 28)
    qt.writel(I2C0 + 0x00, I2C_INTR_ALL)
    sts = i2c_write(qt, 0x30, [0x31, 0x07], stop=False)
    check('register address 0x3107: DT', sts & (I2C_DT | I2C_DR | I2C_BERR),
          I2C_DT)
    check('and SPI 41', gic_pending(qt, SPI_I2C0), True)
    sts, data = i2c_read(qt, 0x30, 2)
    check('repeated START, read two: DR', sts & (I2C_DR | I2C_BERR), I2C_DR)
    check('chip ID 0xCD2E, what ipcinfo calls SC401AI', data, b'\xcd\x2e')

    i2c_write(qt, 0x30, [0x30, 0x00, 0x11, 0x22, 0x33, 0x44])
    i2c_write(qt, 0x30, [0x30, 0x00], stop=False)
    sts, data = i2c_read(qt, 0x30, 4)
    check('six bytes written and four read, two chunks each', data,
          b'\x11\x22\x33\x44')

    sts = i2c_write(qt, 0x31, [0x00])
    check('an empty address NACKs: BERR', sts & I2C_BERR, I2C_BERR)
    # i2c_nvtim_handle_nack(): a lone STOP
    sts = i2c_chunk(qt, 1, stop=True)
    check('the STOP after it completes cleanly', sts & I2C_BERR, 0)
    # i2cdetect's quick write: address, a 0 byte, STOP after it
    sts = i2c_chunk(qt, 2, 0x30 << 1, start=True, stop=True)
    check('i2cdetect quick write finds 0x30', sts & (I2C_DT | I2C_BERR),
          I2C_DT)
    qt.writel(I2C0 + 0x04, 0xFFFFFFFF)
    check('status write-back drops SPI 41', gic_pending(qt, SPI_I2C0), False)

    for base, name in ((I2C1, 'I2C 1'), (I2C2, 'I2C 2')):
        sts = i2c_chunk(qt, 1, 0x30 << 1, start=True, stop=True, base=base)
        check('%s is empty' % name, sts & I2C_BERR, I2C_BERR)


# ---- nvt-pwm -----------------------------------------------------------------

PWM = 0xF0210000
SPI_PWM = 26


def test_pwm(qt):
    print('nvt-pwm')
    gic_enable(qt, SPI_PWM)
    qt.cmd('irq_intercept_out /machine/nvt-pwm out')
    # nvt_pwm_config(): 1 ms at 50 %, base 30000 / fall 15000
    qt.writel(PWM + 0x04, (0x30 << 16) | (0x98 << 8))
    qt.writel(PWM + 0x230, (0x75 << 16) | (0x3A << 8))
    qt.writel(PWM + 0xE0, 1)                        # INT_ENABLE channel 0
    qt.writel(PWM + 0x100, 1)
    check('ENABLE reads channel 0 running', qt.readl(PWM + 0x100), 1)
    check('its out line is high', qt.irqs.get(0), True)
    qt.writel(PWM + 0x104, 1)
    check('DISABLE stops it', qt.readl(PWM + 0x100), 0)
    check('INT_STS bit 0: stopped', qt.readl(PWM + 0xF0) & 1, 1)
    check('SPI 26', gic_pending(qt, SPI_PWM), True)
    check('out low again', qt.irqs.get(0), False)
    qt.writel(PWM + 0xF0, 1)
    check('INT_STS write-back drops it', gic_pending(qt, SPI_PWM), False)

    # nvt_pwm_set_cycle(): 20 cycles x 30000 ticks at 30 MHz = 20 ms
    qt.writel(PWM + 0x00, 20)
    qt.writel(PWM + 0x100, 1)
    qt.step(19 * 1000 * 1000)
    check('a 20-cycle run: still running at 19 ms', qt.readl(PWM + 0x100), 1)
    qt.step(2 * 1000 * 1000)
    check('stopped by 21 ms', qt.readl(PWM + 0x100), 0)
    check('with INT_STS and SPI 26',
          (qt.readl(PWM + 0xF0) & 1, gic_pending(qt, SPI_PWM)), (1, True))
    qt.writel(PWM + 0xF0, 1)

    qt.writel(PWM + 0x04, 0x30 << 16)               # duty 0: fall 0 in
    qt.writel(PWM + 0x230, 0x75 << 16)              # both bytes
    qt.writel(PWM + 0x00, 0)
    qt.writel(PWM + 0x100, 1)
    check('a zero duty keeps out low', qt.irqs.get(0), False)
    qt.writel(PWM + 0x104, 1)
    qt.writel(PWM + 0x10C, 0x2)
    check('CLKDIV_LOAD flags CLKDIV_LOAD_STS', qt.readl(PWM + 0xFC), 0x2)
    qt.writel(PWM + 0xF4, 0x55)
    check('the micro-step status word is storage', qt.readl(PWM + 0xF4), 0x55)


# ---- nvt-timer ---------------------------------------------------------------

TIMER = 0xF0040000
SPI_TIMER = 0


def tmr(n, off):
    return TIMER + 0x100 + n * 0x10 + off


def timer_cfg(qt, n, target, ctrl):
    """timer_cfg(): stop, clear status, enable the interrupt, start."""
    qt.writel(tmr(n, 0), 0)
    qt.writel(TIMER + 0x10, 1 << n)
    qt.writel(TIMER + 0x20, qt.readl(TIMER + 0x20) | 1 << n)
    qt.writel(tmr(n, 4), target)
    qt.writel(tmr(n, 0), ctrl)


def test_timer(qt):
    print('nvt-timer')
    gic_enable(qt, SPI_TIMER)
    check('timer 0 runs free out of reset (U-Boot\'s microsecond clock)',
          qt.readl(tmr(0, 0)), 3)
    t0 = qt.readl(tmr(0, 8))
    qt.step(5000 * 1000)
    check('COUNTER 0 advances 5000 in 5 ms: 1 MHz',
          qt.readl(tmr(0, 8)) - t0, 5000)

    # one shot, divider 0: 1000 ticks
    timer_cfg(qt, 3, 999, 1)
    check('enable reads back at once (timer_cfg() polls it)',
          qt.readl(tmr(3, 0)) & 1, 1)
    qt.step(400 * 1000)
    check('one shot: 400 counted at 400 us', qt.readl(tmr(3, 8)), 400)
    qt.step(599 * 1000)
    check('no status before the target', qt.readl(TIMER + 0x10) & 1 << 3, 0)
    qt.step(1000)
    check('STS bit 3 at 1 ms, and SPI 0',
          (qt.readl(TIMER + 0x10) & 1 << 3, gic_pending(qt, SPI_TIMER)),
          (1 << 3, True))
    check('a one-shot timer stops, enable clear and COUNTER at its target',
          (qt.readl(tmr(3, 0)) & 1, qt.readl(tmr(3, 8))), (0, 999))
    qt.writel(TIMER + 0x10, 1 << 3)
    check('STS write-back drops SPI 0', gic_pending(qt, SPI_TIMER), False)

    # free run, divider 1 at 3 MHz / (5 + 1) = 500 kHz: a 2 ms period
    qt.writel(TIMER + 0x30, (5 << 8) | 1 << 17)
    timer_cfg(qt, 5, 999, 7)
    qt.step(2000 * 1000)
    check('free run on divider 1: STS at 2 ms (500 kHz)',
          qt.readl(TIMER + 0x10) & 1 << 5, 1 << 5)
    qt.writel(TIMER + 0x10, 1 << 5)
    check('it goes on', qt.readl(tmr(5, 0)) & 1, 1)
    qt.step(1000 * 1000)
    check('wrapped: 500 counted 1 ms into the next period',
          qt.readl(tmr(5, 8)), 500)
    qt.writel(TIMER + 0x30, (2 << 8) | 1 << 17)     # 1 MHz
    qt.step(250 * 1000)
    check('a divider change carries the count on at the new rate',
          qt.readl(tmr(5, 8)), 750)
    qt.writel(tmr(5, 0xC), 1)
    qt.step(100 * 1000)
    check('RELOAD starts a new period', qt.readl(tmr(5, 8)), 100)
    qt.writel(tmr(5, 0), 6)
    held = qt.readl(tmr(5, 8))
    qt.step(1000 * 1000)
    check('disabled, the count holds', qt.readl(tmr(5, 8)), held)
    qt.step(2000 * 1000)
    check('and no more status', qt.readl(TIMER + 0x10) & 1 << 5, 0)
    qt.writel(TIMER + 0x20, 0)
    qt.writel(tmr(7, 4), 9)
    qt.writel(tmr(7, 0), 1)
    qt.step(20 * 1000)
    check('a status bit without its INT bit raises nothing',
          (qt.readl(TIMER + 0x10) & 1 << 7, gic_pending(qt, SPI_TIMER)),
          (1 << 7, False))
    qt.writel(TIMER + 0x10, 1 << 7)
    qt.writel(tmr(5, 8), 1234)
    check('COUNTER is read-only', qt.readl(tmr(5, 8)), held)
    # divider 1 at 3 MHz / (4 + 1) = 600 kHz: a tick is 1666.7 ns, so a
    # rebase that went through nanoseconds and back would lose it
    qt.writel(TIMER + 0x30, (4 << 8) | 1 << 17)
    timer_cfg(qt, 11, 0xFFFFFF, 5)
    qt.step(1700)
    counts = []
    for _ in range(5):
        qt.writel(TIMER + 0x30, (4 << 8) | 1 << 17)
        counts.append(qt.readl(tmr(11, 8)))
    check('divider rewrites at 600 kHz keep the count (1, not back to 0)',
          counts, [1] * 5)
    qt.writel(tmr(11, 0), 0)
    # 3 MHz, a 1 us period: no drift from rounding each period to 333 ns
    qt.writel(TIMER + 0x30, 1 << 17)
    timer_cfg(qt, 12, 2, 7)
    qt.writel(TIMER + 0x20, 0)
    qt.step(10 * 1000 * 1000 + 334)
    check('3 MHz, period 3 ticks: 10 ms and 334 ns in, count 1',
          qt.readl(tmr(12, 8)), 1)
    qt.writel(tmr(12, 0), 5)                        # free run -> one shot
    qt.step(1000)
    check('switched to one shot mid-period: it ends there and stops',
          (qt.readl(tmr(12, 0)) & 1, qt.readl(tmr(12, 8))), (0, 2))
    qt.writel(TIMER + 0x10, 0xFFFFF)
    # The machine reset (the kernel's manual watchdog reset) restarts
    # timer 0 as the loader leaves it, and stops the rest.
    qt.writel(tmr(9, 4), 0xFFFF)
    qt.writel(tmr(9, 0), 1)
    qt.writel(WDT + 0x00, 0x5A960112)
    qt.writel(WDT + 0x00, 0x5A960113)
    qt.writel(WDT + 0x0C, 1)
    qt.step(1000)
    check('a machine reset: timer 0 free-running again, timer 9 stopped',
          (qt.readl(tmr(0, 0)), qt.readl(tmr(9, 0))), (3, 0))


# ---- nvt-crypto, nvt-hash, nvt-rsa -------------------------------------------

CRYPTO = 0xF0620000
HASH = 0xF0670000
RSA = 0xF06A0000
SPI_CRYPTO, SPI_HASH, SPI_RSA = 38, 28, 20
DESC, SRC, DST = 0x01000000, 0x01001000, 0x01002000


def crypto_dma(qt, cipher, op, data, key, iv=b'', decrypt=False, aad=None,
               get_s0=False, step=True, keymgr=False):
    """One na51089_crypto_start() request; returns the output, CV, S0
    and GHASH."""
    hdr = (int(decrypt) | cipher << 4 | op << 8 | int(keymgr) << 12 |
           int(get_s0) << 16)
    desc = bytearray(0x80 + 32 * 16)
    desc[0:len(key)] = key
    desc[0x20:0x20 + len(iv)] = iv
    if op == 4 and step:                    # CTR: counter step 1
        struct.pack_into('<I', desc, 0x3C if cipher >= 2 else 0x34, 1)
    struct.pack_into('<I', desc, 0x40, hdr)
    blk = 0
    if op == 5:                             # GCM: the AAD group first
        a = aad or b''
        qt.write(SRC + 0x800, a or b'\0')
        struct.pack_into('<IIII', desc, 0x80, SRC + 0x800, 0, len(a), 1)
        blk = 1
    qt.write(SRC, data)
    struct.pack_into('<IIII', desc, 0x80 + 16 * blk, SRC, DST, len(data), 1)
    qt.write(DESC, bytes(desc))
    qt.writel(CRYPTO + 0x00, 0x2)
    qt.writel(CRYPTO + 0x08, 0x10)
    qt.writel(CRYPTO + 0x0C, 0x10)
    qt.writel(CRYPTO + 0x50, DESC)
    qt.writel(CRYPTO + 0x04, 0x10)
    out = qt.read(DST, len(data))
    return (out, qt.read(DESC + 0x50, 16), qt.read(DESC + 0x60, 16),
            qt.read(DESC + 0x70, 16))


def test_crypto(qt):
    print('nvt-crypto')
    gic_enable(qt, SPI_CRYPTO)
    # PIO: FIPS-197 C.1, AES-128 of 00112233..ff
    key = bytes(range(16))
    pt = bytes.fromhex('00112233445566778899aabbccddeeff')
    for i in range(4):
        qt.writel(CRYPTO + 0x10 + 4 * i, struct.unpack_from('<I', key, 4 * i)[0])
        qt.writel(CRYPTO + 0x30 + 4 * i, struct.unpack_from('<I', pt, 4 * i)[0])
    qt.writel(CRYPTO + 0x00, 0x2 | 2 << 4)
    qt.writel(CRYPTO + 0x04, 1)
    out = b''.join(struct.pack('<I', qt.readl(CRYPTO + 0x40 + 4 * i))
                   for i in range(4))
    check('PIO AES-128 (FIPS-197 C.1)', out.hex(),
          '69c4e0d86a7b0430d8cdb78070b4c55a')
    check('PIO done in INT_STS bit 0, CTRL idle',
          (qt.readl(CRYPTO + 0x0C) & 1, qt.readl(CRYPTO + 0x04) & 1), (1, 0))
    qt.writel(CRYPTO + 0x0C, 1)

    k = bytes.fromhex('2b7e151628aed2a6abf7158809cf4f3c')
    p = bytes.fromhex('6bc1bee22e409f96e93d7e117393172a'
                      'ae2d8a571e03ac9c9eb76fac45af8e51')
    out, cv, _, _ = crypto_dma(qt, 2, 1, p, k, bytes(range(16)))
    check('DMA AES-128-CBC (SP 800-38A F.2.1)', out.hex(),
          '7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2')
    check('CV: the last cipher block, the next request\'s IV', cv, out[16:])
    check('DMA channel 0 done: INT_STS bit 4 and SPI 38',
          (qt.readl(CRYPTO + 0x0C) & 0x10, gic_pending(qt, SPI_CRYPTO)),
          (0x10, True))
    qt.writel(CRYPTO + 0x0C, 0x10)
    check('the write-back drops SPI 38', gic_pending(qt, SPI_CRYPTO), False)
    back, _, _, _ = crypto_dma(qt, 2, 1, out, k, bytes(range(16)), True)
    check('and decrypted back', back, p)

    # The same two blocks in two entries split 7 bytes in, each with its
    # own destination: the block across the split still encrypts.
    desc = bytearray(0x80 + 32 * 16)
    desc[0:16] = k
    desc[0x20:0x30] = bytes(range(16))
    struct.pack_into('<I', desc, 0x40, 2 << 4 | 1 << 8)
    struct.pack_into('<IIII', desc, 0x80, SRC, DST, 7, 0)
    struct.pack_into('<IIII', desc, 0x90, SRC + 0x100, DST + 0x100, 25, 1)
    qt.write(SRC, p[:7])
    qt.write(SRC + 0x100, p[7:])
    qt.write(DESC, bytes(desc))
    qt.writel(CRYPTO + 0x50, DESC)
    qt.writel(CRYPTO + 0x04, 0x10)
    check('AES-CBC, a block split across two entries',
          (qt.read(DST, 7) + qt.read(DST + 0x100, 25)).hex(),
          '7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2')
    qt.writel(CRYPTO + 0x0C, 0x10)

    ctr = bytes.fromhex('f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff')
    out, cv, _, _ = crypto_dma(qt, 2, 4, p[:16] + p[16:21], k, ctr)
    check('AES-128-CTR (F.5.1), 21 bytes: a partial last block',
          out.hex(), '874d6191b620e3261bef6864990db6ce9806f66b79')
    check('CV: the counter after both blocks, carried across bytes',
          cv.hex(), 'f0f1f2f3f4f5f6f7f8f9fafbfcfdff01')

    # GCM test case 2 (McGrew & Viega): K, IV, P all zero
    out, _, s0, gh = crypto_dma(qt, 2, 5, bytes(16), bytes(16), bytes(12))
    check('AES-GCM: the ciphertext', out.hex(),
          '0388dace60b6a392f328c2b971b2fe78')
    check('tag = S0 ^ GHASH, as the driver forms it',
          bytes(a ^ b for a, b in zip(s0, gh)).hex(),
          'ab6e47d42cec13bdf53a67b21257bddf')
    # test case 4: a partial last block and associated data
    k4 = bytes.fromhex('feffe9928665731c6d6a8f9467308308')
    iv4 = bytes.fromhex('cafebabefacedbaddecaf888')
    p4 = bytes.fromhex('d9313225f88406e5a55909c5aff5269a86a7a9531534f7da'
                       '2e4c303d8a318a721c3c0c95956809532fcf0e2449a6b525'
                       'b16aedf5aa0de657ba637b39')
    a4 = bytes.fromhex('feedfacedeadbeeffeedfacedeadbeefabaddad2')
    out, _, s0, gh = crypto_dma(qt, 2, 5, p4, k4, iv4, aad=a4)
    check('AES-GCM test case 4: 60 bytes, 20 of AAD',
          (out[:8].hex(), bytes(a ^ b for a, b in zip(s0, gh)).hex()),
          ('42831ec2217774244b7221b784d0d49c'[:16],
           '5bc94fbc3221a5db94fae95ae7121a47'))
    back, _, s0, gh = crypto_dma(qt, 2, 5, out, k4, iv4, True, aad=a4)
    check('decrypting it: the plaintext and the same tag (GHASH over the input)',
          (back == p4, bytes(a ^ b for a, b in zip(s0, gh)).hex()),
          (True, '5bc94fbc3221a5db94fae95ae7121a47'))

    out, _, _, _ = crypto_dma(qt, 0, 0, bytes.fromhex('0123456789abcdef'),
                              bytes.fromhex('133457799bbcdff1'))
    check('DES-ECB (the textbook example)', out.hex(), '85e813540f0ab405')
    k3 = bytes.fromhex('0123456789abcdef23456789abcdef01456789abcdef0123')
    out, _, _, _ = crypto_dma(qt, 1, 0, b'The quic', k3)
    # openssl enc -des-ede3 -K <k3> -nopad
    check('3DES-EDE-ECB, three keys', out.hex(), '1ccf23869d09333e')
    # The key from the OTP key manager, not the descriptor: blank, so zero
    out, _, s0, _ = crypto_dma(qt, 2, 4, bytes(16), b'\x55' * 16, bytes(16),
                               get_s0=True, keymgr=True)
    check('key manager key (zero); "get S0": S0 = E(K, IV), the text '
          'from the next counter',
          (s0.hex(), out.hex()),
          ('66e94bd4ef8a2c3b884cfa59ca342b2e',
           '58e2fccefa7e3061367f1d57a4e7455a'))


def hash_run(qt, mode, data, cfg_extra, padlen=None, key=None):
    if key is not None:
        k = key.ljust(64, b'\0')
        for i in range(16):
            qt.writel(HASH + 0x10 + 4 * i, struct.unpack_from('<I', k, 4 * i)[0])
    qt.write(SRC, data or b'\0')
    qt.writel(HASH + 0x04, len(data) if padlen is None else padlen)
    qt.writel(HASH + 0x94, SRC)
    qt.writel(HASH + 0x9C, len(data))
    qt.writel(HASH + 0x0C, 1)
    qt.writel(HASH + 0x08, 1)
    qt.writel(HASH + 0x00, 0x2 | mode << 4 | 1 << 12 | cfg_extra)
    return b''.join(struct.pack('<I', qt.readl(HASH + 0x70 + 4 * i))
                    for i in range(8))


def test_hash(qt):
    print('nvt-hash')
    gic_enable(qt, SPI_HASH)
    d = hash_run(qt, 1, b'abc', 1 << 13)
    check('SHA-256("abc"), padded by the engine', d.hex(),
          'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad')
    check('done: INT_STS bit 0 and SPI 28',
          (qt.readl(HASH + 0x0C) & 1, gic_pending(qt, SPI_HASH)), (1, True))
    qt.writel(HASH + 0x0C, 1)
    d = hash_run(qt, 0, b'abc', 1 << 13)
    check('SHA-1("abc")', d[:20].hex(),
          'a9993e364706816aba3e25717850c26c9cd0d89d')
    # two chunks: a whole block, then the rest from the IV registers
    msg = b'abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq' * 2
    mid = hash_run(qt, 1, msg[:64], 0)
    for i in range(8):
        qt.writel(HASH + 0x50 + 4 * i, struct.unpack_from('<I', mid, 4 * i)[0])
    d = hash_run(qt, 1, msg[64:], 1 << 8 | 1 << 13, padlen=len(msg))
    check('SHA-256 in two chunks, chained through the IV registers', d.hex(),
          hashlib.sha256(msg).hexdigest())
    d = hash_run(qt, 2, b'Hi There', 1 << 13 | 1 << 21 | 1 << 22,
                 key=b'\x0b' * 20)
    check('HMAC-SHA-1 (RFC 2202 case 1): ipad and opad by the engine',
          d[:20].hex(), 'b617318655057264e28bc0b6fb378c8ef146be00')
    d = hash_run(qt, 3, b'what do ya want for nothing?',
                 1 << 13 | 1 << 21 | 1 << 22, key=b'Jefe')
    check('HMAC-SHA-256 (RFC 4231 case 2)', d.hex(),
          '5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843')
    qt.writel(HASH + 0x0C, 1)
    qt.writel(HASH + 0x94, 0xE0000000)          # nothing there
    qt.writel(HASH + 0x9C, 64)
    qt.writel(HASH + 0x00, 0x2 | 1 << 4 | 1 << 12 | 1 << 13)
    check('a chunk it cannot read: done, and no digest left behind',
          (qt.readl(HASH + 0x0C) & 1, qt.readl(HASH + 0x70)), (1, 0))


def rsa_load(qt, port, value, words):
    qt.writel(RSA + port + 4, 0)
    for i in range(words):
        qt.writel(RSA + port, (value >> (32 * i)) & 0xFFFFFFFF)


def test_rsa(qt):
    print('nvt-rsa')
    gic_enable(qt, SPI_RSA)
    rng = random.Random(5)
    for width, bits in ((0, 256), (1, 512), (3, 2048)):
        n = rng.getrandbits(bits) | 1 << (bits - 1) | 1
        e = 65537 if bits < 2048 else rng.getrandbits(bits) % n
        m = rng.getrandbits(bits) % n
        words = 64                                  # what the driver writes
        rsa_load(qt, 0x20, m, words)
        rsa_load(qt, 0x10, n, words)
        rsa_load(qt, 0x18, e, words)
        qt.writel(RSA + 0x00, width << 1)
        qt.writel(RSA + 0x0C, 1)
        qt.writel(RSA + 0x08, 1)
        qt.writel(RSA + 0x04, 1)
        qt.writel(RSA + 0x24, 0)
        got = sum(qt.readl(RSA + 0x20) << (32 * i) for i in range(bits // 32))
        check('%d-bit m^e mod n through the SRAM ports' % bits, got == pow(m, e, n),
              True)
    check('done: INT_STS bit 0 and SPI 20',
          (qt.readl(RSA + 0x0C) & 1, gic_pending(qt, SPI_RSA)), (1, True))
    qt.writel(RSA + 0x0C, 1)
    check('the write-back drops SPI 20', gic_pending(qt, SPI_RSA), False)
    qt.writel(RSA + 0x00, 1 << 4 | 3 << 1)          # CRC32 of N
    qt.writel(RSA + 0x04, 1)
    check('a CRC32 key check (not modelled) never reports done',
          qt.readl(RSA + 0x0C) & 1, 0)


# ---- nvt-media: the HDAL engines ---------------------------------------------

SIE = 0xF0C00000
IME = 0xF0C40000
GRPH = 0xF0C80000
ISE = 0xF0C90000
H26X = 0xF0A10000
JPG = 0xF0A00000
EAC = 0xF0640000
DAI = 0xF0630000
LL = 0x01100000
FRAME_NS = 1000 * 1000 * 1000 // 30


def ll_upd(off, val, byte_en=0xF):
    return struct.pack('<Q', 4 << 61 | byte_en << 44 | off << 32 | val)


def ll_nxtll(addr):
    return struct.pack('<Q', 1 << 61 | addr << 8)


def test_media(qt):
    print('nvt-media')
    # SIE: a frame in three steps, single-mode outputs complete
    gic_enable(qt, 1)
    qt.writel(SIE + 0x08, 0x2008F)                  # VD, BP3, OUT0 end, CRPEND
    qt.writel(SIE + 0xA4, 0x70000000)               # three outputs, single
    qt.writel(SIE + 0x00, 0x6)                      # ACT_EN | LOAD
    check('SIE: LOAD clears itself, DMA idle reads set',
          qt.readl(SIE + 0x00) & 0x02000006, 0x02000004)
    qt.step(FRAME_NS // 3 + 1000)
    check('SIE frame start: VD and the window starts, SPI 1',
          (qt.readl(SIE + 0x0C), gic_pending(qt, 1)), (0x31, True))
    qt.writel(SIE + 0x0C, 0xFFFFFFFF)
    qt.step(FRAME_NS // 3)
    check('half way: the line break points', qt.readl(SIE + 0x0C), 0xE)
    qt.writel(SIE + 0x0C, 0xFFFFFFFF)
    qt.step(FRAME_NS // 3)
    check('frame end: the window ends and the DRAM outputs\' ends',
          qt.readl(SIE + 0x0C), 0x31F80)
    check('a single-mode output clears its enable once written',
          qt.readl(SIE + 0xA4), 0)
    qt.writel(SIE + 0x00, 0)
    qt.writel(SIE + 0x0C, 0xFFFFFFFF)
    qt.step(FRAME_NS)
    check('stopped: no more frames', qt.readl(SIE + 0x0C), 0)

    # IME: a linked list of two jobs; its updates set the enables
    gic_enable(qt, 6)
    second = LL + 0x100
    qt.write(LL, ll_upd(0x018, 0xA0000009) + ll_upd(0x100, 0xBEEF1234, 0x3) +
             ll_nxtll(second))
    qt.write(second, ll_upd(0x104, 0x5A5A5A5A) + struct.pack('<Q', 0))
    qt.writel(IME + 0x100, 0xFFFF0000)
    qt.writel(IME + 0x14, LL)
    qt.writel(IME + 0x00, 1 << 28)
    check('IME: the fire bit does not stay', qt.readl(IME + 0x00), 0)
    qt.step(1000 * 1000)
    check('job 1: its updates applied, bytes as enabled',
          (qt.readl(IME + 0x18), qt.readl(IME + 0x100)),
          (0xA0000009, 0xFFFF1234))
    check('frame start / end and job end in status, SPI 6',
          (qt.readl(IME + 0x1C), gic_pending(qt, 6)), (0xA0000008, True))
    qt.writel(IME + 0x1C, 0xFFFFFFFF)
    check('status written back: SPI 6 drops', gic_pending(qt, 6), False)
    qt.step(1000 * 1000)
    check('job 2, the last: list end too',
          (qt.readl(IME + 0x104), qt.readl(IME + 0x1C)),
          (0x5A5A5A5A, 0xA0000009))
    qt.writel(IME + 0x1C, 0xFFFFFFFF)

    # 2D graphics: list end without DMA idle (an abort's); single jobs
    gic_enable(qt, 18)
    qt.write(LL, ll_upd(0x008, 0x16) + struct.pack('<Q', 0))
    qt.writel(GRPH + 0xD0, LL)
    qt.writel(GRPH + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    check('graphics list: list end only, SPI 18',
          (qt.readl(GRPH + 0x0C), gic_pending(qt, 18)), (0x2, True))
    qt.writel(GRPH + 0x0C, 0xFF)
    qt.writel(GRPH + 0x00, 0x2)
    check('software reset takes effect at once', qt.readl(GRPH + 0x00), 0)
    qt.writel(GRPH + 0x08, 0x1)
    qt.writel(GRPH + 0x00, 0x1)
    qt.step(1000 * 1000)
    check('a single job: done in status bit 0, its trigger dropped',
          (qt.readl(GRPH + 0x0C), qt.readl(GRPH + 0x00)), (0x1, 0))
    qt.writel(GRPH + 0x0C, 0xFF)

    # ISE: list end and frame end
    qt.write(LL, struct.pack('<Q', 0))
    qt.writel(ISE + 0x2C, LL)
    qt.writel(ISE + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    check('ISE list: frame end and list end', qt.readl(ISE + 0x0C),
          0x80000001)

    # H.26x: INT_FLAG write 1 to clear, reset, a picture
    gic_enable(qt, 16)
    qt.writel(H26X + 0x004, 0x01101711)
    check('H.26x: INT_FLAG clears by writing ones', qt.readl(H26X + 0x004), 0)
    qt.writel(H26X + 0x000, 0x10)
    check('software reset: its finish flag', qt.readl(H26X + 0x004),
          0x10000000)
    qt.writel(H26X + 0x004, 0xFFFFFFFF)
    qt.writel(H26X + 0x070, DST)
    qt.writel(H26X + 0x07C, 0x10000)
    qt.writel(H26X + 0x01C, 0x1)
    qt.writel(H26X + 0x000, 0x100)
    qt.step(1000 * 1000)
    check('a picture: finish flag and SPI 16, start dropped',
          (qt.readl(H26X + 0x004), gic_pending(qt, 16),
           qt.readl(H26X + 0x000)), (1, True, 0))
    nal = qt.read(DST, 64)
    check('BS_LEN 64, a filler NAL unit in the buffer',
          (qt.readl(H26X + 0x268), nal[:6].hex(), nal[63]),
          (64, '000000014c01', 0x80))
    qt.writel(H26X + 0x004, 0xFFFFFFFF)
    qt.write(DST, b'\x55' * 8)
    qt.writel(H26X + 0x07C, 0)
    qt.writel(H26X + 0x000, 0x100)
    qt.step(1000 * 1000)
    check('no bitstream buffer: buffer full (bit 12), not finish; BS_LEN 0, '
          'nothing written',
          (qt.readl(H26X + 0x004) & 0x1001, qt.readl(H26X + 0x268),
           qt.read(DST, 8)), (0x1000, 0, b'\x55' * 8))
    qt.writel(H26X + 0x004, 0xFFFFFFFF)

    # JPEG: reset, DMA idle, a grey 4-MCU picture
    gic_enable(qt, 17)
    qt.writel(JPG + 0x00, 0x8)
    check('JPEG: reset clears itself, DMA idle reads set',
          qt.readl(JPG + 0x00), 0x20)
    qt.writel(JPG + 0x2C, 3)                        # four MCUs, minus one
    qt.writel(JPG + 0x40, DST)
    qt.writel(JPG + 0x44, 0x1000)
    qt.writel(JPG + 0x18, 1)
    qt.writel(JPG + 0x00, 1)
    qt.step(1000 * 1000)
    check('frame end, SPI 17, act dropped',
          (qt.readl(JPG + 0x1C), gic_pending(qt, 17),
           qt.readl(JPG + 0x00) & 1), (1, True, 0))
    check('no picture to read: flat grey, DC 0 / EOB per block, 4 bytes '
          'per MCU, then EOI',
          (qt.readl(JPG + 0x4C), qt.read(DST, 18).hex()),
          (18, '28a28a00' * 4 + 'ffd9'))
    qt.writel(JPG + 0x1C, 1)



AUD_A, AUD_B = 0x01600000, 0x01601000


def test_audio(qt, wav):
    print('nvt-audio')
    # EAC: strobes the driver waits on
    qt.writel(EAC + 0x04, 1)
    qt.writel(EAC + 0x10, 0x17000)
    check('EAC: LOAD and the DC-cancel load clear themselves',
          (qt.readl(EAC + 0x04), qt.readl(EAC + 0x10)), (0, 0x15000))
    qt.writel(EAC + 0x00, 3)                        # ADC and DAC on
    qt.writel(EAC + 0x20, 0x73730E0E)               # ADC gains 0 dB
    qt.writel(EAC + 0xAC, 0xFFFF)                   # DAC gain 0 dB

    # capture: 16-bit stereo, 1 KiB buffers, 48 kHz (192000 B/s)
    gic_enable(qt, 15)
    qt.write(AUD_A, b'\xAA' * 1024)
    qt.write(AUD_B, b'\xAA' * 1024)
    qt.writel(DAI + 0x14, 0x0101)
    qt.writel(DAI + 0x48, AUD_A)
    qt.writel(DAI + 0x68, 256)                      # words
    qt.writel(DAI + 0x20, 0x40)
    qt.writel(DAI + 0x00, 0x41)
    check('RX DMA on takes the buffer (DMA loaded), SPI 15',
          (qt.readl(DAI + 0x28), gic_pending(qt, 15)), (0x40, True))
    qt.writel(DAI + 0x28, 0xFFFFFFFF)
    qt.writel(DAI + 0x48, AUD_B)                    # queue the next
    qt.writel(DAI + 0x20, 0x10C00C)
    qt.writel(DAI + 0x00, 0x4041)
    qt.step(3 * 1000 * 1000)
    cur = qt.readl(DAI + 0x88)
    check('3 ms in: the current address moves through the first buffer',
          (AUD_A < cur < AUD_A + 1024, qt.readl(DAI + 0x28) & 4), (True, 0))
    qt.step(3 * 1000 * 1000)
    check('1 KiB at 192000 B/s: buffer done, into the queued one',
          (qt.readl(DAI + 0x28) & 4, AUD_B <= qt.readl(DAI + 0x88) <
           AUD_B + 1024), (4, True))
    fr = struct.unpack('<8h', qt.read(AUD_A, 16))
    want = [round(16384 * math.sin(2 * math.pi * f * n / 48000))
            for n in range(4) for f in (1000, 1500)]
    check('the first buffer holds the tone: 1 kHz left, 1.5 kHz right',
          all(abs(a - b) <= 1 for a, b in zip(fr, want)), True)
    qt.writel(DAI + 0x28, 0xFFFFFFFF)
    qt.writel(DAI + 0x00, 0x41)
    check('RX channel off: stopped', qt.readl(DAI + 0x28) & 0x400, 0x400)
    qt.writel(DAI + 0x00, 0x1)
    qt.writel(DAI + 0x28, 0xFFFFFFFF)

    # the ADC off: silence
    qt.writel(EAC + 0x00, 2)
    qt.write(AUD_A, b'\xAA' * 1024)
    qt.writel(DAI + 0x48, AUD_A)
    qt.writel(DAI + 0x00, 0x4041)
    qt.step(6 * 1000 * 1000)
    check('with the ADC disabled the capture is silence',
          qt.read(AUD_A, 1024), bytes(1024))
    qt.writel(DAI + 0x00, 0x1)
    qt.writel(DAI + 0x28, 0xFFFFFFFF)
    qt.writel(EAC + 0x00, 3)

    # the rate is the CG's: 16 kHz is eacdac / 21, DAI / 4
    qt.writel(CG + 0x38, 0x14030000)
    qt.writel(DAI + 0x48, AUD_A)
    qt.writel(DAI + 0x00, 0x4041)
    qt.step(10 * 1000 * 1000)
    early = qt.readl(DAI + 0x28) & 4
    qt.step(7 * 1000 * 1000)
    check('at 16 kHz a 1 KiB buffer takes 16 ms, not 5.3',
          (early, qt.readl(DAI + 0x28) & 4), (0, 4))
    qt.writel(DAI + 0x00, 0x1)
    qt.writel(DAI + 0x28, 0xFFFFFFFF)
    qt.writel(CG + 0x38, 0x1B000100)                # 48 kHz again

    # the DAI's global enable gates the DMAs too
    qt.writel(DAI + 0x48, AUD_A)
    qt.writel(DAI + 0x00, 0x4040)                   # DAIEN off
    qt.step(10 * 1000 * 1000)
    check('with DAIEN clear nothing moves', qt.readl(DAI + 0x28) & 4, 0)
    qt.writel(DAI + 0x00, 0x0)
    qt.writel(DAI + 0x28, 0xFFFFFFFF)

    # a rate change under a running stream takes effect within a tick
    qt.writel(DAI + 0x48, AUD_A)
    qt.writel(DAI + 0x00, 0x4041)                   # 48 kHz: 5.3 ms a buffer
    qt.writel(CG + 0x38, 0x14030000)                # now 16 kHz: 16 ms
    qt.step(6 * 1000 * 1000)
    qt.writel(DAI + 0x28, 0xFFFFFFFF)               # the first buffer, at 48k
    qt.step(10 * 1000 * 1000)
    early = qt.readl(DAI + 0x28) & 4
    qt.step(8 * 1000 * 1000)
    check('the CG dividers changing mid-stream change its rate',
          (early, qt.readl(DAI + 0x28) & 4), (0, 4))
    qt.writel(DAI + 0x00, 0x1)
    qt.writel(DAI + 0x28, 0xFFFFFFFF)
    qt.writel(CG + 0x38, 0x1B000100)

    # playback: 1 kHz, 16-bit stereo, to the wav audiodev
    tone = b''.join(struct.pack('<2h', *(
        [round(16384 * math.sin(2 * math.pi * 1000 * n / 48000))] * 2))
        for n in range(256))
    qt.write(AUD_A, tone)
    qt.writel(DAI + 0x10, 0x0101)
    qt.writel(DAI + 0x40, AUD_A)
    qt.writel(DAI + 0x60, 256)
    qt.writel(DAI + 0x20, 0x10)
    qt.writel(DAI + 0x00, 0x11)
    check('TX1 DMA on: loaded, from 0x40 / 0x60',
          qt.readl(DAI + 0x28) & 0x10, 0x10)
    qt.writel(DAI + 0x28, 0xFFFFFFFF)
    qt.writel(DAI + 0x20, 0x1001)
    qt.writel(DAI + 0x00, 0x1011)
    for i in range(40):                             # 200 ms, the buffer looping
        qt.step(5 * 1000 * 1000)
    check('TX1 buffers complete', qt.readl(DAI + 0x28) & 1, 1)
    qt.writel(DAI + 0x20, 0x1101)
    qt.writel(DAI + 0x00, 0x11)
    check('TX1 channel off: stopped (what the driver waits on)',
          qt.readl(DAI + 0x28) & 0x100, 0x100)
    qt.writel(DAI + 0x00, 0x1)
    qt.close(graceful=True)
    data = open(wav, 'rb').read()[44:]
    pcm = struct.unpack('<%dh' % (len(data) // 4 * 2),
                        data[:len(data) // 4 * 4])
    left = pcm[0::2]
    loud = [i for i, v in enumerate(left) if abs(v) > 4000]
    hz = 0
    if loud:
        part = left[loud[0]:loud[-1]]
        cross = sum(1 for a, b in zip(part, part[1:]) if a < 0 <= b)
        hz = cross * 48000 / max(len(part), 1)
    check('the wav audiodev plays the 1 kHz tone at full level',
          (max((abs(v) for v in left), default=0) > 15000,
           950 < hz < 1050), (True, True))


IDE = 0xF0800000
DISP_Y, DISP_UV, DISP_OSD = 0x01700000, 0x01720000, 0x01740000


def test_display(qt, mon):
    print('nvt-ide')
    gic_enable(qt, 48)
    check('DMA idle while the engine is off', qt.readl(IDE) & 4, 4)
    # 320x240 out of a parallel panel (device 9): no serial-RGB squeeze
    qt.writel(IDE + 0x0D0, 319 << 12)
    qt.writel(IDE + 0x0D8, 239 << 12)
    qt.writel(IDE + 0x0BC, 0x808010)                # background black
    # video 1: NV12 320x240, the left half red, the right half blue
    red, blue = (81, 90, 240), (41, 240, 110)
    qt.write(DISP_Y, bytes((red if x < 160 else blue)[0]
                           for y in range(240) for x in range(320)))
    qt.write(DISP_UV, bytes(v for y in range(120) for x in range(0, 320, 2)
                            for v in (red if x < 160 else blue)[1:]))
    qt.writel(IDE + 0xD00, DISP_Y)
    qt.writel(IDE + 0xD10, DISP_UV)
    qt.writel(IDE + 0x028, 240 << 16 | 320)
    qt.writel(IDE + 0x02C, 80)                      # words
    qt.writel(IDE + 0x0A8, 10 << 28 | 239 << 16 | 319)
    qt.writel(IDE + 0x0AC, 0)
    # OSD 1: a 32x32 ARGB4444 white box at (100, 100), global alpha 0x80
    qt.write(DISP_OSD, struct.pack('<H', 0xFFFF) * 32 * 32)
    qt.writel(IDE + 0x05C, DISP_OSD)
    qt.writel(IDE + 0x060, 32 << 16 | 32)
    qt.writel(IDE + 0x064, 16)
    qt.writel(IDE + 0x090, 12 << 28 | 31 << 16 | 31)
    qt.writel(IDE + 0x094, 100 << 16 | 100)
    qt.writel(IDE + 0x1B4, 0x80 << 24 | 1 << 4)
    qt.writel(IDE + 0x000, 9 << 7 | 1 << 27 | 1 << 3 | 1 << 1)
    check('the engine on: DMA busy', qt.readl(IDE) & 4, 0)
    qt.writel(IDE + 0x0FC, 0x80)                    # frame end enabled
    qt.writel(IDE + 0x000, qt.readl(IDE) | 1 << 28) # load
    check('load pending until the frame', qt.readl(IDE) >> 28 & 1, 1)
    qt.step(20 * 1000 * 1000)
    check('at the frame: load clear, frame end (bit 23), SPI 48',
          (qt.readl(IDE) >> 28 & 1, qt.readl(IDE + 0x0FC) >> 23 & 1,
           gic_pending(qt, 48)), (0, 1, True))
    qt.writel(IDE + 0x0FC, 0x80 | 1 << 23)
    check('frame end is write 1 to clear', (qt.readl(IDE + 0x0FC) >> 23 & 1,
          gic_pending(qt, 48)), (0, False))
    ppm = os.path.join(os.path.dirname(mon), 'ide.ppm')
    c = socket.socket(socket.AF_UNIX)
    c.connect(mon)
    c.sendall(('screendump %s\n' % ppm).encode())
    for i in range(50):
        if os.path.exists(ppm) and os.path.getsize(ppm) >= 320 * 240 * 3:
            break
        time.sleep(0.1)
    c.close()
    data = open(ppm, 'rb').read()
    hdr = data.split(b'\n', 3)
    px = hdr[3]
    def pixel(x, y):
        return tuple(px[(y * 320 + x) * 3:(y * 320 + x) * 3 + 3])
    check('screendump: 320x240', hdr[1], b'320 240')
    left, right, box = pixel(40, 40), pixel(280, 40), pixel(110, 110)
    check('video 1 scanned out: red left, blue right',
          (left[0] > 200 and left[2] < 60, right[2] > 200 and right[0] < 60),
          (True, True))
    check('OSD 1 over it at half alpha: a light red box',
          (180 < box[0], 100 < box[1] < 160, 100 < box[2] < 160),
          (True, True, True))


DSI = 0xF0840000


def test_dsi(qt):
    print('nvt-dsi')
    gic_enable(qt, 50)
    qt.writel(DSI + 0x80, 0x1F0FFFF3)               # the driver's enables
    # an LP data byte on lane 0: the trigger clears, escape done (25)
    qt.writel(DSI + 0x28, 0x05)
    qt.writel(DSI + 0x00, 1 << 1 | 1 << 4)
    check('escape trigger: done at once', qt.readl(DSI) & 1 << 4, 0)
    qt.step(10 * 1000)
    check('lane 0 escape done (bit 25), SPI 50',
          (qt.readl(DSI + 0x84) >> 25 & 1, gic_pending(qt, 50)), (1, True))
    qt.writel(DSI + 0x84, 1 << 25)
    check('status is write 1 to clear', (qt.readl(DSI + 0x84),
                                         gic_pending(qt, 50)), (0, False))
    # clock lane ULPS
    qt.writel(DSI + 0x00, 1 << 1 | 1 << 9)
    qt.step(10 * 1000)
    check('ULPS trigger: clears, ULPS done (bit 24)',
          (qt.readl(DSI) >> 9 & 1, qt.readl(DSI + 0x84) >> 24 & 1), (0, 1))
    qt.writel(DSI + 0x84, 0xFFFFFFFF)
    # manual mode: TX sends the commands and drops
    qt.writel(DSI + 0x04, 0)
    qt.writel(DSI + 0x00, 1 << 1 | 1)
    qt.step(10 * 1000)
    check('manual TX: TX on clears, TX done (bit 0)',
          (qt.readl(DSI) & 1, qt.readl(DSI + 0x84) & 1), (0, 1))
    qt.writel(DSI + 0x84, 0xFFFFFFFF)
    # automatic mode 2: frames until TX goes off
    qt.writel(DSI + 0x04, 2)
    qt.writel(DSI + 0x00, 1 << 1 | 1)
    qt.step(20 * 1000 * 1000)
    check('auto mode: TX stays on, frame end (bit 1) each frame',
          (qt.readl(DSI) & 1, qt.readl(DSI + 0x84) >> 1 & 1), (1, 1))
    qt.writel(DSI + 0x84, 0xFFFFFFFF)
    qt.writel(DSI + 0x00, 1 << 1)
    qt.step(10 * 1000)
    check('TX off: TX done', qt.readl(DSI + 0x84) & 1, 1)
    qt.step(40 * 1000 * 1000)
    check('and no more frames', qt.readl(DSI + 0x84) >> 1 & 1, 0)


NUE, NUE2, CNN = 0xF0C60000, 0xF0D50000, 0xF0CB0000
AI_IN, AI_W, AI_OUT, AI_LL = 0x01800000, 0x01810000, 0x01820000, 0x01830000


def ai_settle(qt, addr, mask, tries=500):
    """Step the clock until an AI engine's status has a bit of mask: the
    engines compute on a worker thread, done when the work comes back."""
    for _ in range(tries):
        qt.step(1000 * 1000)
        if qt.readl(addr) & mask:
            return
        time.sleep(0.005)


def test_ai(qt):
    print('AI engines (NUE / NUE2 / CNN)')
    gic_enable(qt, 14)
    check('idle (bit 15) on all three',
          [qt.readl(b + o) >> 15 & 1 for b, o in
           ((NUE, 0x1AC), (NUE2, 0x098), (CNN, 0x1FC))], [1, 1, 1])
    # NUE, fully connected: 8 inputs (s8), 3 rows of weights, s32 out
    x = [3, -2, 5, 0, 7, -8, 1, 4]
    w = [[1, 2, 3, 4, 5, 6, 7, 8], [-1, 0, 1, 0, -1, 0, 1, 0],
         [2, 2, 2, 2, -3, -3, -3, -3]]
    qt.write(AI_IN, bytes(v & 0xFF for v in x))
    qt.write(AI_W, bytes(v & 0xFF for r in w for v in r))
    qt.writel(NUE + 0x004, 0x110)                   # SVM, signed s8
    qt.writel(NUE + 0x008, 0x200350)                # dot, path 3 (s32), 8-bit w
    qt.writel(NUE + 0x00C, AI_IN)
    qt.writel(NUE + 0x014, AI_W)
    qt.writel(NUE + 0x02C, AI_OUT)
    qt.writel(NUE + 0x040, 3 << 16 | 8)
    qt.writel(NUE + 0x044, 8 << 16 | 1)
    qt.writel(NUE + 0x048, 1 << 16 | 1)
    qt.writel(NUE + 0x04C, 3 << 16 | 1)             # gain 1, >> 3
    qt.writel(NUE + 0x050, 5 << 24 | 7)             # each product >> 5, + 7
    qt.writel(NUE + 0x038, 1)
    qt.writel(NUE + 0x000, 2)
    ai_settle(qt, NUE + 0x03C, 1)
    want = [(sum((a * 256 * b) >> 5 for a, b in zip(x, r)) >> 3) + 7
            for r in w]
    check('NUE FC: frame end and SPI 14, START dropped',
          (qt.readl(NUE + 0x03C) & 1, gic_pending(qt, 14),
           qt.readl(NUE) & 2), (1, True, 0))
    check('NUE FC: each row\'s dot product, shifted, offset (s32)',
          list(struct.unpack('<3i', qt.read(AI_OUT, 12))), want)
    qt.writel(NUE + 0x03C, 0xFFFFFFFF)
    check('status is write 1 to clear', gic_pending(qt, 14), False)
    # the same job from a list, fired by its last update and writing over
    # its own input: run twice, it would compute from its own results
    def upd(off, val, flags=0):
        return struct.pack('<Q', 8 << 60 | flags | 0xF << 44 | off << 32 | val)
    qt.write(AI_LL, upd(0x02C, AI_IN, 1 << 60) + struct.pack('<Q', 0))
    qt.writel(NUE + 0x024, AI_LL)
    qt.writel(NUE + 0x038, 0x500)
    qt.writel(NUE + 0x000, 1 << 28)
    ai_settle(qt, NUE + 0x03C, 0x100)
    check('NUE list: the fired job runs once, then the list ends',
          (list(struct.unpack('<3i', qt.read(AI_IN, 12))),
           qt.readl(NUE + 0x03C) & 0x500), (want, 0x500))
    qt.writel(NUE + 0x03C, 0xFFFFFFFF)
    # NUE2: 4x2 NV12 grey / white to RGB, scaled to 2x2, DC mean 16
    qt.write(AI_IN, bytes([128, 128, 255, 255, 128, 128, 255, 255]))
    qt.write(AI_IN + 0x100, bytes([128, 128, 128, 128]))
    qt.writel(NUE2 + 0x004, 0x3)                    # YUV to RGB, DC mean
    qt.writel(NUE2 + 0x008, AI_IN)
    qt.writel(NUE2 + 0x00C, AI_IN + 0x100)
    for c in range(3):
        qt.writel(NUE2 + 0x018 + 4 * c, AI_OUT + 0x100 * c)
        qt.writel(NUE2 + 0x030 + 4 * c, 2)
    qt.writel(NUE2 + 0x024, 4)
    qt.writel(NUE2 + 0x028, 4)
    qt.writel(NUE2 + 0x044, 2 << 16 | 4)
    qt.writel(NUE2 + 0x058, 2 << 16 | 2)
    qt.writel(NUE2 + 0x060, 0x101010)
    qt.writel(NUE2 + 0x07C, 1 << 12)                # mean scale 1, shift 0
    qt.writel(NUE2 + 0x000, 2)
    ai_settle(qt, NUE2 + 0x040, 1)
    check('NUE2: grey and white, scaled 4 to 2 columns (corners), less 16',
          [list(qt.read(AI_OUT + 0x100 * c, 2)) for c in range(3)],
          [[112, 239]] * 3)
    check('NUE2: frame end', qt.readl(NUE2 + 0x040) & 1, 1)
    qt.writel(NUE2 + 0x040, 0xFFFFFFFF)
    # CNN: a list of two jobs, each ending in a firing update (bit 60);
    # a bit-59 update adds the list base
    qt.writel(CNN + 0x204, 0x10000000)              # list base
    qt.write(AI_LL, upd(0x010, 0x1234, 1 << 59) + upd(0x0C0, 0, 1 << 60) +
             upd(0x014, 0x5678) + upd(0x0C0, 0, 1 << 60) +
             struct.pack('<Q', 0))
    qt.writel(CNN + 0x040, AI_LL)
    qt.writel(CNN + 0x008, 0x500)
    qt.writel(CNN + 0x000, 1 << 28)
    ai_settle(qt, CNN + 0x00C, 0x100)
    check('CNN list: both jobs, job end (bit 10) and list end (bit 8)',
          (qt.readl(CNN + 0x00C) & 0x500, qt.readl(CNN + 0x014)),
          (0x500, 0x5678))
    check('a bit-59 update adds the list base', qt.readl(CNN + 0x010),
          0x10001234)



def pack12(vals):
    """12-bit two's-complement values as an LSB-first bit stream."""
    bits = 0
    for i, v in enumerate(vals):
        bits |= (v & 0xFFF) << (12 * i)
    return bits.to_bytes((12 * len(vals) + 7) // 8 + 1, 'little')


def rshr(v, sh):
    """The engines' right shift, rounding half up."""
    return (v + (1 << (sh - 1))) >> sh if sh else v


def test_ai_compute(qt):
    print('AI engines: a CNN layer, pooling, NUE FC with rho')
    rnd = random.Random(98566)
    # CNN: a 3x3 conv of a 5x4x2 s8 map into 2 sets, all sides padded,
    # then BN, ReLU and out0 (s8)
    W, H, C, S = 5, 4, 2, 2
    x = [[[rnd.randint(-100, 100) for _ in range(W)] for _ in range(H)]
         for _ in range(C)]
    w = [[[[rnd.randint(-300, 300) for _ in range(3)] for _ in range(3)]
          for _ in range(C)] for _ in range(S)]
    bn = [(600, -20, 30), (400, 15, -40)]              # alpha, mean, beta
    qt.write(AI_IN, bytes(v & 0xFF for c in x for r in c for v in r))
    qt.write(AI_W, pack12([v for st in w for c in st for r in c for v in r]))
    qt.write(AI_W + 0x1000, b''.join(
        struct.pack('<Q', a | (m & 0xFFF) << 24 | (b & 0xFFF) << 36)
        for a, m, b in bn))
    regs = {0x004: 0x1300, 0x010: AI_IN, 0x020: W, 0x024: W * H,
            0x030: AI_W, 0x038: AI_W + 0x1000, 0x044: AI_OUT,
            0x04C: W, 0x050: W * H, 0x05C: W | H << 12, 0x060: C,
            0x064: 0x150, 0x068: 3 | S << 8, 0x06C: 0xF,
            0x078: 1 | 2 << 8 | 10 << 21,                  # mean <<1, beta <<2
            0x0AC: 3 << 12 | 4 << 4, 0x0B0: 1 << 12,       # x3 >>4, x1
            0x0B4: 255 << 12 | 7 << 4, 0x008: 1}
    for off, val in regs.items():
        qt.writel(CNN + off, val)
    qt.writel(CNN + 0x000, 2)
    ai_settle(qt, CNN + 0x00C, 1)
    want = []
    for st in range(S):
        a, m, b = bn[st]
        for oy in range(H):
            for ox in range(W):
                acc = sum(w[st][c][ky][kx] * x[c][oy + ky - 1][ox + kx - 1]
                          for c in range(C) for ky in range(3)
                          for kx in range(3)
                          if 0 <= oy + ky - 1 < H and 0 <= ox + kx - 1 < W)
                v = rshr(acc * 3, 4)
                v = rshr((v - (m << 1)) * a, 10) + (b << 2)
                v = max(min(v, 32767), 0)                  # 16 bits, ReLU
                want.append(max(min(rshr(v * 255, 7 + 8), 127), -128))
    check('CNN 3x3 conv, BN, ReLU: out0 (s8), and frame end',
          (list(struct.unpack('<%db' % (S * H * W),
                              qt.read(AI_OUT, S * H * W))),
           qt.readl(CNN + 0x00C) & 1), (want, 1))
    qt.writel(CNN + 0x00C, 0xFFFFFFFF)
    # the same layer with every scale at its largest, shifted left 31:
    # the stages saturate (positive out, ReLU's zeros stay)
    for off in (0x0AC, 0x0B0, 0x0B4):
        qt.writel(CNN + off, 0xFFFF << 12 | 31 << 4 | 1)
    qt.writel(CNN + 0x000, 2)
    ai_settle(qt, CNN + 0x00C, 1)
    got = list(struct.unpack('<%db' % (S * H * W), qt.read(AI_OUT, S * H * W)))
    check('CNN scales at their largest saturate, not wrap',
          sorted(set(got)) in ([0, 127], [127], [0]), True)
    qt.writel(CNN + 0x00C, 0xFFFFFFFF)
    # pooling alone: 3x3 max, stride 2, ceil, of a 5x5 u8 map (out1)
    px = [rnd.randint(0, 255) for _ in range(25)]
    qt.write(AI_IN, bytes(px))
    for off, val in {0x004: 0x400, 0x020: 5, 0x024: 25, 0x048: AI_OUT,
                     0x054: 2, 0x058: 4, 0x05C: 5 | 5 << 12, 0x060: 1,
                     0x064: 0x2000, 0x098: 0x110, 0x09C: 0,
                     0x0B8: 1 << 12, 0x0BC: 1 << 12}.items():
        qt.writel(CNN + off, val)
    qt.writel(CNN + 0x000, 2)
    ai_settle(qt, CNN + 0x00C, 1)
    want = [max(px[(y + dy) * 5 + x + dx] for dy in range(3)
                for dx in range(3)) for y in (0, 2) for x in (0, 2)]
    check('CNN 3x3/2 max pooling (out1, u8)', list(qt.read(AI_OUT, 4)), want)
    qt.writel(CNN + 0x00C, 0xFFFFFFFF)
    # global average of three 4x4 maps: sum * 64 >> 10
    px = [rnd.randint(0, 255) for _ in range(48)]
    qt.write(AI_IN, bytes(px))
    for off, val in {0x004: 0x420, 0x05C: 4 | 4 << 12, 0x060: 3,
                     0x020: 4, 0x024: 16, 0x054: 1, 0x058: 1,
                     0x098: 1, 0x09C: 64 << 8}.items():
        qt.writel(CNN + off, val)
    qt.writel(CNN + 0x000, 2)
    ai_settle(qt, CNN + 0x00C, 1)
    check('CNN global average pooling',
          list(qt.read(AI_OUT, 3)),
          [rshr(sum(px[16 * c:16 * c + 16]) * 64, 10) for c in range(3)])
    qt.writel(CNN + 0x00C, 0xFFFFFFFF)
    # NUE, output path 0: u8 inputs (x 16) by 12-bit weights, per-product
    # >> 8, x 3 >> 2, >> rho_fmt 6, less a 12-bit rho per row, s16 out
    xv = [rnd.randint(0, 255) for _ in range(4)]
    wv = [[rnd.randint(-2048, 2047) for _ in range(4)] for _ in range(3)]
    rho = [rnd.randint(-500, 500) for _ in range(3)]
    qt.write(AI_IN, bytes(xv))
    qt.write(AI_W, pack12([v for r in wv for v in r]))
    qt.write(AI_W + 0x100, pack12(rho))
    for off, val in {0x004: 0x00010100, 0x008: 0x50, 0x00C: AI_IN,
                     0x014: AI_W, 0x01C: AI_W + 0x100, 0x02C: AI_OUT,
                     0x040: 4 | 3 << 16, 0x048: 1, 0x04C: 3 | 2 << 16,
                     0x050: 8 << 24, 0x054: 0, 0x058: 3 << 16,
                     0x038: 1}.items():
        qt.writel(NUE + off, val)
    qt.writel(NUE + 0x000, 2)
    ai_settle(qt, NUE + 0x03C, 1)
    want = [max(min(((sum((16 * xv[k] * wv[j][k]) >> 8 for k in range(4))
                      * 3 >> 2) >> 6) - rho[j], 32767), -32768)
            for j in range(3)]
    check('NUE FC path 0: 12-bit weights, rho, s16 out',
          list(struct.unpack('<3h', qt.read(AI_OUT, 6))), want)
    qt.writel(NUE + 0x03C, 0xFFFFFFFF)
    # rows of an odd length: the next row starts mid-byte
    wv = [[rnd.randint(-2048, 2047) for _ in range(3)] for _ in range(3)]
    qt.write(AI_W, pack12([v for r in wv for v in r]))
    qt.writel(NUE + 0x040, 3 | 3 << 16)
    qt.writel(NUE + 0x000, 2)
    ai_settle(qt, NUE + 0x03C, 1)
    want = [max(min(((sum((16 * xv[k] * wv[j][k]) >> 8 for k in range(3))
                      * 3 >> 2) >> 6) - rho[j], 32767), -32768)
            for j in range(3)]
    check('NUE FC: 12-bit rows of 3, the second from mid-byte',
          list(struct.unpack('<3h', qt.read(AI_OUT, 6))), want)
    qt.writel(NUE + 0x03C, 0xFFFFFFFF)
    # NUE2: mean scale 0 is no scale-shift, not a scale of nought
    qt.write(AI_IN, bytes([128, 128, 255, 255, 128, 128, 255, 255]))
    qt.write(AI_IN + 0x100, bytes([128, 128, 128, 128]))
    qt.writel(NUE2 + 0x07C, 0)
    qt.writel(NUE2 + 0x000, 2)
    ai_settle(qt, NUE2 + 0x040, 1)
    check('NUE2: mean scale 0 leaves the difference as it is',
          [list(qt.read(AI_OUT + 0x100 * c, 2)) for c in range(3)],
          [[112, 239]] * 3)



def test_ai_face(qt):
    print('AI engines: the face recognition network\'s layers')
    rnd = random.Random(98563)
    W, H, C = 4, 4, 2
    x = [rnd.randint(-120, 120) for _ in range(W * H * C)]
    qt.write(AI_IN, bytes(v & 0xFF for v in x))
    # activation alone, ACT bit 16: ReLU of the input negated (PReLU's
    # negative half), u8 out0 (x 256 by its scale-shift, the 8.8 drop)
    cnn_job(qt, {0x004: 0x200, 0x010: AI_IN, 0x020: W, 0x024: W * H,
                 0x044: AI_OUT, 0x04C: W, 0x050: W * H,
                 0x05C: W | H << 12, 0x060: C, 0x064: 0x110,
                 0x090: 1 << 16, 0x094: 1 << 16, 0x0B4: 1 << 12 | 8 << 4 | 1})
    check('CNN act bit 16: ReLU(-x)', list(qt.read(AI_OUT, W * H * C)),
          [max(-v, 0) for v in x])
    # BN alone with negative alphas (PReLU's slopes are), s8 out0
    bn = [(-300, 10, 5), (-77, -20, -3)]                # alpha, mean, beta
    qt.write(AI_W, b''.join(
        struct.pack('<Q', (a & 0xFFFFFF) | (m & 0xFFF) << 24 |
                    (b & 0xFFF) << 36) for a, m, b in bn))
    cnn_job(qt, {0x004: 0x100, 0x010: AI_IN, 0x020: W, 0x024: W * H,
                 0x038: AI_W, 0x044: AI_OUT, 0x04C: W, 0x050: W * H,
                 0x05C: W | H << 12, 0x060: C, 0x064: 0x150,
                 0x078: 10 << 21, 0x0AC: 1 << 12, 0x0B0: 1 << 12,
                 0x0B4: 1 << 12 | 8 << 4 | 1})
    want = []
    for c, (a, m, b) in enumerate(bn):
        for v in x[c * W * H:(c + 1) * W * H]:
            want.append(max(min(rshr((v - m) * a, 10) + b, 127), -128))
    check('CNN BN: alpha is signed',
          list(struct.unpack('<%db' % (W * H * C), qt.read(AI_OUT, W * H * C))),
          want)
    # global average into a 16-bit out1: 8 fraction bits more than an
    # 8-bit one
    px = [rnd.randint(0, 120) for _ in range(48)]
    qt.write(AI_IN, bytes(px))
    cnn_job(qt, {0x004: 0x420, 0x010: AI_IN, 0x020: 4, 0x024: 16,
                 0x048: AI_OUT, 0x054: 2, 0x058: 2, 0x05C: 4 | 4 << 12,
                 0x060: 3, 0x064: 0x3080, 0x098: 1, 0x09C: 64 << 8,
                 0x0B8: 1 << 12, 0x0BC: 1 << 12})
    check('CNN global average into a 16-bit out1 keeps 8 fraction bits',
          list(struct.unpack('<3h', qt.read(AI_OUT, 6))),
          [rshr(sum(px[16 * c:16 * c + 16]) * 256 * 64, 10)
           for c in range(3)])
    # NUE FC on s16 inputs: their top 12 bits enter the multiplier
    xv = [rnd.randint(-32768, 32767) for _ in range(4)]
    wv = [[rnd.randint(-2048, 2047) for _ in range(4)] for _ in range(3)]
    rho = [rnd.randint(-500, 500) for _ in range(3)]
    qt.write(AI_IN, struct.pack('<4h', *xv))
    qt.write(AI_W, pack12([v for r in wv for v in r]))
    qt.write(AI_W + 0x100, pack12(rho))
    for off, val in {0x004: 0x00011110, 0x008: 0x50, 0x00C: AI_IN,
                     0x014: AI_W, 0x01C: AI_W + 0x100, 0x02C: AI_OUT,
                     0x040: 4 | 3 << 16, 0x048: 1, 0x04C: 3 | 2 << 16,
                     0x050: 8 << 24, 0x054: 0, 0x058: 3 << 16,
                     0x038: 1}.items():
        qt.writel(NUE + off, val)
    qt.writel(NUE + 0x000, 2)
    ai_settle(qt, NUE + 0x03C, 1)
    want = [max(min(((sum(((xv[k] >> 4) * wv[j][k]) >> 8 for k in range(4))
                      * 3 >> 2) >> 6) - rho[j], 32767), -32768)
            for j in range(3)]
    check('NUE FC: 16-bit inputs by their top 12 bits',
          list(struct.unpack('<3h', qt.read(AI_OUT, 6))), want)
    qt.writel(NUE + 0x03C, 0xFFFFFFFF)


MD = 0xF0C10000
MD_MODEL, MD_VAR1 = 0x01900000, 0x01A00000     # 40 and 6 bytes a pixel


def md_frame(qt, y, w, h, mode, morph=0, regs=None):
    """One MDBC trigger on the w x h luma y (grey chroma); the bitmap."""
    qt.write(AI_IN, bytes(y))
    qt.write(AI_IN + w * h, bytes([128]) * (w * h // 2))
    for off, val in {0x004: mode, 0x010: AI_IN, 0x014: AI_IN + w * h,
                     0x018: w, 0x01C: w, 0x024: MD_MODEL, 0x028: MD_VAR1,
                     0x034: AI_LL, 0x038: MD_MODEL, 0x03C: MD_VAR1,
                     0x048: w | h << 12,
                     0x04C: 10 << 8 | 30 << 16 | 4 << 24 | 8 << 28,
                     0x050: 8, 0x054: 2, 0x058: 20, 0x064: morph,
                     0x068: 4 | 64 << 8 | 128 << 16,
                     0x078: 1 << 15 | 0x1234, 0x008: 0x31,
                     **(regs or {})}.items():
        qt.writel(MD + off, val)
    qt.writel(MD + 0x000, 2)
    ai_settle(qt, MD + 0x00C, 1)
    bits = qt.read(AI_LL, (w * h + 7) // 8)
    qt.writel(MD + 0x00C, 0xFFFFFFFF)
    return [bits[p >> 3] >> (p & 7) & 1 for p in range(w * h)]


def test_md(qt):
    print('MDBC: motion detection')
    w, h = 64, 48
    bg = [60 + x // 2 + 20 * ((x // 8 + y // 8) % 2)
          for y in range(h) for x in range(w)]
    gic_enable(qt, 46)
    check('MDBC: idle (bit 15)', qt.readl(MD + 0x110) >> 15 & 1, 1)
    fg = md_frame(qt, bg, w, h, 0)
    check('MDBC INIT: no motion, frame end and SPI 46',
          (sum(fg), qt.readl(MD + 0x000) & 2), (0, 0))
    check('MDBC: the still scene matches its model',
          sum(md_frame(qt, bg, w, h, 1)), 0)
    blk = list(bg)
    for y in range(12, 36):
        for x in range(16, 48):
            blk[y * w + x] = 230
    fg = md_frame(qt, blk, w, h, 1)
    inside = all(fg[y * w + x] for y in range(12, 36) for x in range(16, 48))
    far = any(fg[y * w + x] for y in range(h) for x in range(w)
              if not (10 <= y < 38 and 14 <= x < 50))
    check('MDBC: a bright block is foreground, nothing 3 pixels off it',
          (inside, far), (True, False))
    check('MDBC: the frame counter counts triggers',
          qt.readl(MD + 0x074) >> 12 & 0xFF, 3)
    # the same frame through one erosion stage (select 0, threshold 8):
    # the last bitmap's pixels with all 8 neighbours set
    want = [fg[y * w + x] and all(
                0 <= y + dy < h and 0 <= x + dx < w and fg[(y + dy) * w + x + dx]
                for dy in (-1, 0, 1) for dx in (-1, 0, 1))
            for y in range(h) for x in range(w)]
    check('MDBC morphology: an erosion stage',
          md_frame(qt, blk, w, h, 1, morph=8), [int(v) for v in want])
    # a scene change that stays: with ghost removal (foreground for more
    # than 2 frames updates anyway), updates every frame and 4 matches
    # needed, the block joins the background.  The random state runs
    # free (DBG_RND would draw the same numbers every frame).
    mode = 1 | 1 << 4 | 1 << 8
    regs = {0x054: 4, 0x068: 1 | 64 << 8 | 2 << 16, 0x078: 0}
    md_frame(qt, bg, w, h, 0, regs=regs)
    first = sum(md_frame(qt, blk, w, h, mode, regs=regs))
    for _ in range(40):
        last = sum(md_frame(qt, blk, w, h, mode, regs=regs))
    # (neighbour diffusion every frame keeps a few pixels flickering where
    # it copies values across the pattern's edges)
    check('MDBC: ghost removal absorbs a lasting change',
          (first > 700, last < first // 50), (True, True))
    # a reset straight after a start: no frame end, no status
    qt.writel(MD + 0x000, 2)
    qt.writel(MD + 0x000, 1)
    qt.writel(MD + 0x000, 0)
    qt.step(10 * 1000 * 1000)
    check('MDBC reset: the job and its status go, the enables stay',
          (qt.readl(MD + 0x00C), qt.readl(MD + 0x008),
           gic_pending(qt, 46)), (0, 0x31, False))


def cnn_job(qt, regs, done=1):
    """Run one CNN job set up in regs (offset: value) and wait for it."""
    for off in range(0x004, 0x0C0, 4):
        qt.writel(CNN + off, regs.get(off, 0))
    qt.writel(CNN + 0x008, 1)
    qt.writel(CNN + 0x000, 2)
    ai_settle(qt, CNN + 0x00C, done)
    qt.writel(CNN + 0x00C, 0xFFFFFFFF)


def test_ai_detect(qt):
    print('AI engines: the detectors\' layers')
    rnd = random.Random(98562)
    # conv with a plain bias (no BN): 3x3 over 4x4x2 u8, 2 sets; the bias,
    # 12 bits in 16, << CONV_SHIFT_B 3, comes after the conv's scale-shift
    W, H, C, S = 4, 4, 2, 2
    x = [[[rnd.randint(0, 200) for _ in range(W)] for _ in range(H)]
         for _ in range(C)]
    w = [[[[rnd.randint(-200, 200) for _ in range(3)] for _ in range(3)]
          for _ in range(C)] for _ in range(S)]
    bias = [-37, 52]
    qt.write(AI_IN, bytes(v for c in x for r in c for v in r))
    qt.write(AI_W, pack12([v for st in w for c in st for r in c for v in r]))
    qt.write(AI_W + 0x1000, struct.pack('<2H', *(b & 0xFFF for b in bias)))
    cnn_job(qt, {0x004: 0x1200, 0x010: AI_IN, 0x020: W, 0x024: W * H,
                 0x030: AI_W, 0x038: AI_W + 0x1000, 0x044: AI_OUT,
                 0x04C: W, 0x050: W * H, 0x05C: W | H << 12, 0x060: C,
                 0x064: 0x100, 0x068: 3 | S << 8, 0x06C: 0xF | 3 << 8,
                 0x0AC: 5 << 12 | 6 << 4, 0x0B0: 1 << 12,
                 0x0B4: 255 << 12 | 7 << 4})
    want = []
    for st in range(S):
        for oy in range(H):
            for ox in range(W):
                acc = sum(w[st][c][ky][kx] * x[c][oy + ky - 1][ox + kx - 1]
                          for c in range(C) for ky in range(3)
                          for kx in range(3)
                          if 0 <= oy + ky - 1 < H and 0 <= ox + kx - 1 < W)
                v = rshr(acc * 5, 6) + (bias[st] << 3)
                v = max(min(v, 32767), 0)
                want.append(min(rshr(v * 255, 7 + 8), 255))
    check('CNN conv with a plain bias, after its scale-shift (u8 out)',
          list(qt.read(AI_OUT, S * H * W)), want)
    # the same into a 16-bit out0 (a detector's head): the bias still
    # after the scale-shift, and out0 keeps the fraction bits
    cnn_job(qt, {0x004: 0x1000, 0x010: AI_IN, 0x020: W, 0x024: W * H,
                 0x030: AI_W, 0x038: AI_W + 0x1000, 0x044: AI_OUT,
                 0x04C: 2 * W, 0x050: 2 * W * H, 0x05C: W | H << 12,
                 0x060: C, 0x064: 0x340, 0x068: 3 | S << 8,
                 0x06C: 0xF | 3 << 8, 0x0AC: 5 << 12 | 6 << 4,
                 0x0B0: 1 << 12 | 4 << 4, 0x0B4: 1 << 12})
    want = []
    for st in range(S):
        for oy in range(H):
            for ox in range(W):
                acc = sum(w[st][c][ky][kx] * x[c][oy + ky - 1][ox + kx - 1]
                          for c in range(C) for ky in range(3)
                          for kx in range(3)
                          if 0 <= oy + ky - 1 < H and 0 <= ox + kx - 1 < W)
                v = rshr(rshr(acc * 5, 6) + (bias[st] << 3), 4)
                want.append(max(min(v, 32767), -32768))
    check('CNN conv into a 16-bit out0: the bias after the scale-shift',
          list(struct.unpack('<%dh' % (S * H * W), qt.read(AI_OUT, 2 * S * H * W))),
          want)
    # deconvolution: 3x2 x 2 channels spread out by 2, a pad row and
    # column after, the holes 0 (out0 raw, no 8.8 drop)
    px = [rnd.randint(1, 255) for _ in range(12)]
    qt.write(AI_IN, bytes(px))
    cnn_job(qt, {0x004: 1, 0x010: AI_IN, 0x020: 3, 0x024: 6,
                 0x044: AI_OUT, 0x04C: 6, 0x050: 24, 0x05C: 3 | 2 << 12,
                 0x060: 2, 0x064: 0x100, 0x0A0: 1 << 24 | 1 << 2 | 1 << 6,
                 0x0B4: 1 << 12})
    want = [0] * 48
    for c in range(2):
        for y in range(2):
            for xx in range(3):
                want[c * 24 + 2 * y * 6 + 2 * xx] = px[c * 6 + y * 3 + xx]
    check('CNN deconvolution: stride 2, holes and pads 0', list(qt.read(AI_OUT, 48)),
          want)
    # scale-up x 2: each pixel 2x2 times
    cnn_job(qt, {0x004: 2, 0x010: AI_IN, 0x020: 3, 0x024: 6,
                 0x044: AI_OUT, 0x04C: 6, 0x050: 24, 0x05C: 3 | 2 << 12,
                 0x060: 2, 0x064: 0x100, 0x0A4: 1, 0x0B4: 1 << 12})
    want = [px[c * 6 + (y // 2) * 3 + xx // 2] for c in range(2)
            for y in range(4) for xx in range(6)]
    check('CNN scale-up x 2 (nearest)', list(qt.read(AI_OUT, 48)), want)
    # element-wise alone: s16 a + s16 b, ((a * 527) >> 9) + ((b * 1) >> 0)
    # >> 9, then x 256 >> 8 into s8
    a = [rnd.randint(-20000, 20000) for _ in range(8)]
    b = [rnd.randint(-20000, 20000) for _ in range(8)]
    qt.write(AI_IN, struct.pack('<8h', *a))
    qt.write(AI_W, struct.pack('<8h', *b))
    cnn_job(qt, {0x004: 0x800, 0x010: AI_IN, 0x020: 8, 0x024: 16,
                 0x028: 8, 0x02C: 16, 0x03C: AI_W, 0x044: AI_OUT,
                 0x04C: 4, 0x050: 8, 0x05C: 4 | 2 << 12, 0x060: 1,
                 0x064: 0x170, 0x084: 3 | 9 << 4 | 9 << 20,
                 0x088: 527 | 1 << 16, 0x0A8: 1 << 12, 0x0AC: 1 << 12,
                 0x0B0: 1 << 12, 0x0B4: 1 << 12 | 8 << 4 | 1})
    want = [max(min(rshr(rshr(a[i] * 527, 9) + b[i], 9), 127), -128)
            for i in range(8)]
    check('CNN element-wise add of two s16 maps (s8 out)',
          list(struct.unpack('<8b', qt.read(AI_OUT, 8))), want)
    # NUE permute: a planar 3 x 2 x 2 s16 map to each pixel's 3 channels
    # together, pixels 8 bytes apart
    pm = [rnd.randint(-30000, 30000) for _ in range(12)]
    qt.write(AI_IN, struct.pack('<12h', *pm))
    qt.write(AI_OUT, b'\xee' * 32)
    for off, val in {0x004: 0x11114, 0x008: 0, 0x00C: AI_IN, 0x02C: AI_OUT,
                     0x034: 8, 0x040: 2 | 2 << 16, 0x044: 3 << 16,
                     0x0F0: 1 << 12, 0x1C4: 0, 0x038: 1}.items():
        qt.writel(NUE + off, val)
    qt.writel(NUE + 0x000, 2)
    ai_settle(qt, NUE + 0x03C, 1)
    got = qt.read(AI_OUT, 32)
    check('NUE permute: CHW to each pixel\'s channels, 8 bytes a pixel',
          [list(struct.unpack('<3h', got[8 * p:8 * p + 6])) for p in range(4)],
          [[pm[c * 4 + p] for c in range(3)] for p in range(4)])
    check('and the gap after each pixel kept', got[6:8], b'\xee\xee')
    qt.writel(NUE + 0x03C, 0xFFFFFFFF)
    # NUE softmax: 2 classes at 3 pixels, Q.12 logits >> 4 to Q.8, Q.15 out,
    # each pixel's classes together
    lg = [rnd.randint(-12000, 12000) for _ in range(6)]
    qt.write(AI_IN, struct.pack('<6h', *lg))
    for off, val in {0x004: 0x11116, 0x00C: AI_IN, 0x02C: AI_OUT,
                     0x040: 3 | 1 << 16, 0x044: 2 << 16,
                     0x0EC: 4 << 4 | 2 << 16 | 1 << 24}.items():
        qt.writel(NUE + off, val)
    qt.writel(NUE + 0x000, 2)
    ai_settle(qt, NUE + 0x03C, 1)
    want = []
    for p in range(3):
        q = [max(min(lg[c * 3 + p] >> 4, 2047), -2048) / 256 for c in range(2)]
        e = [math.exp(v - max(q)) for v in q]
        want += [min(round(v / sum(e) * 32768), 32767) for v in e]
    got = list(struct.unpack('<6h', qt.read(AI_OUT, 12)))
    check('NUE softmax: Q.15 per pixel, classes together (within 1)',
          all(abs(g - v) <= 1 for g, v in zip(got, want)), True)
    qt.writel(NUE + 0x03C, 0xFFFFFFFF)


PX_RAW, PX_YUV = 0x01200000, 0x01400000


GRPH2 = 0xF0D10000
GA, GB, GC = 0x01500000, 0x01510000, 0x01520000


def gfx_job(qt, base, regs, lladdr=0xD0):
    """One job through the engine's linked list; regs {offset: value}."""
    qt.write(LL, b''.join(ll_upd(o, v) for o, v in regs.items()) +
             struct.pack('<Q', 0))
    qt.writel(base + lladdr, LL)
    qt.writel(base + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    qt.writel(base + 0x0C, 0xFFFFFFFF)


def grph(qt, cfg, w, h, a=GA, alofs=0, b=GB, blofs=0, c=GC, clofs=0,
         const=(0, 0), base=GRPH):
    gfx_job(qt, base, {0x04: cfg, 0x20: 0, 0x24: 0, 0x28: 0,
                       0x60: const[0], 0x64: const[1],
                       0x70: a, 0x74: alofs or w, 0x80: b, 0x84: blofs or w,
                       0x90: c, 0x94: clofs or w, 0x78: h, 0x7C: w})


def test_gfx(qt):
    print('nvt-media graphics and ISE')
    # A_COPY, 8-bit: 16 bytes x 2 lines between line offsets 32 and 16
    src = bytes(range(64))
    qt.write(GA, src)
    grph(qt, 0x00000011, 16, 2, alofs=32, clofs=16)
    check('A_COPY: two lines of 16 bytes, line offset 32 to 16',
          qt.read(GC, 32).hex(), (src[:16] + src[32:48]).hex())
    # MINUS_SHF, 16-bit, as hd_gfx_arithmetic: (6000 + i) - i
    qt.write(GA, struct.pack('<8H', *[6000 + i for i in range(8)]))
    qt.write(GB, struct.pack('<8H', *range(8)))
    grph(qt, 0x02000021, 16, 1)
    check('MINUS_SHF 16-bit: (6000 + i) - i',
          struct.unpack('<8H', qt.read(GC, 16)), (6000,) * 8)
    qt.write(GA, struct.pack('<2H', 5, 9))
    qt.write(GB, struct.pack('<2H', 9, 5))
    grph(qt, 0x02000021, 4, 1, const=(1 << 31, 0))
    check('and its magnitude (CONST1 bit 31): |5 - 9|, |9 - 5|',
          struct.unpack('<2H', qt.read(GC, 4)), (4, 4))
    qt.write(GA, bytes([200, 10]))
    qt.write(GB, bytes([100, 20]))
    grph(qt, 0x01000011, 2, 1)
    check('PLUS_SHF 8-bit saturates: 200 + 100, 10 + 20',
          list(qt.read(GC, 2)), [255, 30])
    # TEXT_COPY, 8-bit: an NV12 green U / V pair laid over the words
    grph(qt, 0x08000011, 6, 2, c=GC + 2, clofs=8, const=(0x142A142A, 0))
    check('TEXT_COPY 8-bit: U V pairs from an address 2 mod 4, two lines',
          (qt.read(GC + 2, 6).hex(), qt.read(GC + 10, 6).hex()),
          ('2a142a142a14', '2a142a142a14'))
    # PACKING: U and V planes into pairs
    qt.write(GA, bytes([1, 2, 3, 4]))
    qt.write(GB, bytes([5, 6, 7, 8]))
    grph(qt, 0x10000011, 8, 1)
    check('PACKING: U from A, V from B, interleaved',
          list(qt.read(GC, 8)), [1, 5, 2, 6, 3, 7, 4, 8])
    # RGBYUV_BLEND, ARGB4444 over NV12: two opaque green pixels, two
    # transparent ones, two lines
    qt.write(GA, struct.pack('<8H', *([0xF0F0] * 2 + [0x0F00] * 2) * 2))
    qt.write(GB, bytes([0x10]) * 8)
    qt.write(GC, bytes([0x80]) * 4)
    grph(qt, 0x18000421, 8, 2, blofs=4, clofs=4, const=(0x21, 0))
    check('RGBYUV_BLEND ARGB4444: opaque green over Y, transparent leaves it',
          (qt.read(GB, 4).hex(), qt.read(GB + 4, 4).hex()),
          ('93931010', '93931010'))
    check('and over the 4:2:0 U / V pairs', qt.read(GC, 4).hex(), '2a148080')
    # with mixed alpha in a 2x2 block the colour is alpha-weighted: an
    # opaque green beside transparent red pixels stays green
    qt.write(GA, struct.pack('<4H', 0xF0F0, 0x0F00, 0x0F00, 0x0F00))
    qt.write(GB, bytes([0x10]) * 4)
    qt.write(GC, bytes([0x80]) * 2)
    grph(qt, 0x18000421, 4, 2, alofs=4, blofs=2, clofs=2, const=(0x21, 0))
    check('RGBYUV_BLEND: transparent pixels add none of their colour '
          '(U / V of a quarter green over grey)', qt.read(GC, 2).hex(),
          '6b65')
    # 16-bit fill of an odd width: the last byte is left as it was
    qt.write(GC, bytes([0xEE]) * 4)
    grph(qt, 0x08000021, 3, 1, const=(0x1234, 0))
    check('TEXT_COPY 16-bit, odd width: whole pixels filled, the odd byte '
          'kept', qt.read(GC, 4).hex(), '3412eeee')
    # a broken list: no work, list error (status bit 2)
    qt.write(GC, bytes([0x55]) * 4)
    qt.write(LL, ll_upd(0x04, 0x08000011) + ll_upd(0x90, GC) +
             struct.pack('<Q', 7 << 61))
    qt.writel(GRPH + 0xD0, LL)
    qt.writel(GRPH + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    check('a malformed list: list error, no fill done',
          (qt.readl(GRPH + 0x0C), qt.read(GC, 4).hex()), (0x4, '55555555'))
    qt.writel(GRPH + 0x0C, 0xFFFFFFFF)
    # GOP on engine 2: a vertical flip (C at the last line), then a
    # horizontal one at 16 bits
    qt.write(GA, bytes([1, 1, 2, 2, 3, 3]))
    grph(qt, 0x00040010, 2, 3, c=GC + 4, base=GRPH2)
    check('VTC_FLIP: line y of A lands y lines above C',
          list(qt.read(GC, 6)), [3, 3, 2, 2, 1, 1])
    qt.write(GA, struct.pack('<4H', 1, 2, 3, 4))
    grph(qt, 0x00030020, 8, 1, base=GRPH2)
    check('HRZ_FLIP 16-bit: a line reversed in 16-bit units',
          struct.unpack('<4H', qt.read(GC, 8)), (4, 3, 2, 1))

    # ISE: a 4x4 Y8 ramp (10 x) up to 7x7, bilinear; and down to 2x2,
    # integration; U / V pairs, nearest
    qt.write(GA, bytes(10 * x + y for y in range(4) for x in range(4)))
    def ise(ctrl, iw, ih, ow, oh, ilofs, olofs):
        gfx_job(qt, ISE, {0x04: ctrl, 0x10: ih << 16 | iw, 0x18: ilofs,
                          0x20: GA, 0x38: oh << 16 | ow, 0x90: olofs,
                          0x98: GC}, lladdr=0x2C)
    ise(0x100, 4, 4, 7, 7, 4, 8)
    check('ISE bilinear 4x4 to 7x7: corners kept, half steps between',
          (list(qt.read(GC, 7)), qt.read(GC + 6 * 8 + 6, 1)[0]),
          ([0, 5, 10, 15, 20, 25, 30], 33))
    ise(0x300, 4, 4, 2, 2, 4, 4)
    check('ISE integration 4x4 to 2x2: each output the average it covers',
          (list(qt.read(GC, 2)), list(qt.read(GC + 4, 2))),
          ([6, 26], [8, 28]))
    qt.write(GA, bytes([10, 20, 30, 40, 50, 60, 70, 80]))
    ise(0x203, 4, 1, 2, 1, 8, 4)
    check('ISE nearest, U / V pairs: every other pair',
          list(qt.read(GC, 4)), [10, 20, 50, 60])


def test_pixels(qt):
    print('nvt-media pixels')
    # SIE: the test pattern, RGGB, 12-bit packed, at DRAM output 0
    w, h = 64, 48
    qt.writel(SIE + 0x2C, h << 16 | w)
    qt.writel(SIE + 0x5C, PX_RAW)
    qt.writel(SIE + 0x60, 2 << 16 | 96)             # 12 bits, 96 bytes a line
    qt.writel(SIE + 0x00, 0x4)
    qt.step(FRAME_NS + 1000)
    qt.writel(SIE + 0x00, 0)
    def raw12(line, x):
        b = 3 * (x // 2)
        v = line[b] | line[b + 1] << 8 | line[b + 2] << 16
        return v >> (12 * (x & 1)) & 0xFFF

    line = qt.read(PX_RAW, 96)
    check('SIE writes packed 12-bit RAW: the white bar, R G R G',
          [raw12(line, x) for x in range(4)], [3071] * 4)
    line = qt.read(PX_RAW + 96, 96)
    check('line 1 is G B: G in the white bar, B in the blue bar (x 49), '
          'none in the red (x 41)',
          (raw12(line, 0), raw12(line, 49), raw12(line, 41)), (3071, 3071, 0))
    # IME: that RAW through IFE / IME to YUV 4:2:0, 32x24 (scaled by half)
    qt.writel(0xF0C70000 + 0x20, h << 16 | w)       # IFE input
    qt.writel(0xF0C70000 + 0x30, PX_RAW)
    qt.writel(0xF0C70000 + 0x34, 96)
    qt.writel(0xF0C70000 + 0x04, 2 << 5)            # 12 bits
    qt.writel(IME + 0x04, 1 << 2)                   # path 1
    qt.writel(IME + 0x84, 24 << 16 | 32)
    qt.writel(IME + 0x94, PX_YUV)
    qt.writel(IME + 0x98, PX_YUV + 0x1000)
    qt.writel(IME + 0x8C, 32)
    qt.writel(IME + 0x90, 32)
    qt.write(LL, struct.pack('<Q', 0))
    qt.writel(IME + 0x14, LL)
    qt.writel(IME + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    y = qt.read(PX_YUV, 32)
    uv = qt.read(PX_YUV + 0x1000, 32)
    check('IME path 1: Y of the bars, white to black (BT.601, limited)',
          [y[2], y[6], y[30]], [180, 161, 16])
    check('UV of the white bar neutral, of the blue bar blue-shifted',
          (uv[0], uv[1], uv[24] > 200), (128, 128, True))
    qt.writel(IME + 0x1C, 0xFFFFFFFF)
    # 16-bit RAW spans the whole range, and IME reads it back as 12-bit;
    # an odd output size (31x23) still writes every byte of its extent
    qt.writel(SIE + 0x60, 3 << 16 | 128)            # 16 bits, 128 bytes a line
    qt.writel(SIE + 0x00, 0x4)
    qt.step(FRAME_NS + 1000)
    qt.writel(SIE + 0x00, 0)
    line = qt.read(PX_RAW, 128)
    check('SIE writes 16-bit RAW scaled to the range: white bar 3071 << 4',
          struct.unpack('<2H', line[:4]), (3071 << 4, 3071 << 4))
    qt.write(PX_YUV, bytes([0xAA]) * 0x2000)
    qt.writel(0xF0C70000 + 0x34, 128)
    qt.writel(0xF0C70000 + 0x04, 3 << 5)            # 16 bits
    qt.writel(IME + 0x84, 23 << 16 | 31)
    qt.write(LL, struct.pack('<Q', 0))
    qt.writel(IME + 0x14, LL)
    qt.writel(IME + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    y = qt.read(PX_YUV, 32 * 23)
    uv = qt.read(PX_YUV + 0x1000, 32 * 11)
    check('IME from 16-bit RAW: the same Y of the bars',
          [y[2], y[6], y[29]], [180, 161, 16])
    check('odd output: the last column and row written, the stride gap not',
          (y[30] != 0xAA, y[22 * 32 + 30] != 0xAA, y[31], uv[30] != 0xAA,
           uv[31]),
          (True, True, 0xAA, True, 0xAA))
    qt.writel(IME + 0x1C, 0xFFFFFFFF)
    # JPEG: a 16x16 picture, the driver's quantisation via the LUT port
    pic = bytes((x * 16) & 0xFF for yy in range(16) for x in range(16))
    qt.write(PX_YUV, pic)
    qt.write(PX_YUV + 0x1000, bytes([128]) * 128)
    qt.writel(JPG + 0x10, 1 << 12 | 1 << 16)        # table 1, auto-increment
    for i in range(32):
        qt.writel(JPG + 0x14, 0x01010101)           # all ones: lossless-ish
    qt.writel(JPG + 0x28, 0)                        # one MCU
    qt.writel(JPG + 0x54, PX_YUV)
    qt.writel(JPG + 0x58, PX_YUV + 0x1000)
    qt.writel(JPG + 0x60, 15)
    qt.writel(JPG + 0x64, 15)
    qt.writel(JPG + 0x40, DST)
    qt.writel(JPG + 0x44, 0x1000)
    qt.writel(JPG + 0x00, 1)
    qt.step(1000 * 1000)
    n = qt.readl(JPG + 0x4C)
    scan = qt.read(DST, n)
    check('JPEG of a real picture: a scan ending in EOI', scan[-2:].hex(),
          'ffd9')
    try:
        from PIL import Image
        import io
        hdr = bytearray(b'\xff\xd8')
        hdr += b'\xff\xdb\x00\x84\x00' + bytes([1] * 64) + b'\x01' + \
            bytes([1] * 64)
        hdr += b'\xff\xc0\x00\x11\x08\x00\x10\x00\x10\x03' + \
            b'\x01\x22\x00\x02\x11\x01\x03\x11\x01'
        hdr += jpeg_std_dht()
        hdr += b'\xff\xda\x00\x0c\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00'
        im = Image.open(io.BytesIO(bytes(hdr) + scan)).convert('L')
        got = [im.getpixel((x, 8)) for x in (0, 7, 15)]
        check('decodes (PIL) to the picture: the ramp, within 2',
              all(abs(g - e) <= 2 for g, e in zip(got, (0, 112, 240))), True)
    except ImportError:
        print('  SKIP: decoding the JPEG (no Python PIL)')
    qt.writel(JPG + 0x1C, 1)


def jpeg_std_dht():
    """The Annex K Huffman tables, as the driver's JPEG header has them."""
    return bytes.fromhex(
        'ffc401a20000010501010101010100000000000000000102030405060708090a'
        '0b100002010303020403050504040000017d0102030004110512213141061351'
        '6107227114328191a1082342b1c11552d1f02433627282090a161718191a2526'
        '2728292a3435363738393a434445464748494a535455565758595a6364656667'
        '68696a737475767778797a838485868788898a92939495969798999aa2a3a4a5'
        'a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9da'
        'e1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa010003010101010101010101'
        '0000000000000102030405060708090a0b110002010204040304070504040001'
        '0277000102031104052131061241510761711322328108144291a1b1c1092333'
        '52f0156272d10a162434e125f11718191a262728292a35363738393a43444546'
        '4748494a535455565758595a636465666768696a737475767778797a82838485'
        '868788898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9ba'
        'c2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2e3e4e5e6e7e8e9eaf2f3f4f5f6'
        'f7f8f9fa')


# ---- nvt-usb, nvt-otp -------------------------------------------------------

USB = 0xF0600000
USB_ALIAS = 0xFF600000
OTP = 0xF0660000


def test_usb(qt, stick):
    print('nvt-usb' + (' (a usb-storage stick)' if stick else ' (no device)'))
    if not stick:
        check('no device: PORTSC connect clear', qt.readl(USB + 0x30) & 1, 0)
        check('and OTG speed 0', qt.readl(USB + 0x80) >> 22 & 3, 0)
        return
    for base, where in ((USB, '0xF0600000'), (USB_ALIAS, '0xFF600000')):
        caps = qt.readl(base)
        check('%s: CAPLENGTH 0x10, EHCI 1.00' % where,
              (caps & 0xFF, caps >> 16), (0x10, 0x0100))
        check('%s: HCSPARAMS one port' % where, qt.readl(base + 4) & 0xF, 1)
    check('PORTSC at 0x30: the stick is connected',
          qt.readl(USB + 0x30) & 3, 3)
    check('OTG register [23:22]: high speed (ehci_port_speed)',
          qt.readl(USB + 0x80) >> 22 & 3, 2)
    qt.writel(USB + 0x80, 1 << 4)                       # A_BUS_REQ
    check('OTG register keeps what is written', qt.readl(USB + 0x80),
          (1 << 4) | (2 << 22))
    for off in (0x100, 0x1C8, 0x30C, 0x400, 0x1000 + 0x51 * 4):
        qt.writel(USB + off, 0x5A5A0000 | off)
    check('glue and PHY registers are storage',
          [qt.readl(USB + off) for off in (0x100, 0x400, 0x1144)],
          [0x5A5A0100, 0x5A5A0400, 0x5A5A1144])
    check('the alias reaches the same glue', qt.readl(USB_ALIAS + 0x400),
          0x5A5A0400)
    # ehci_hub_control SetPortFeature(RESET), then clear it
    qt.writel(USB + 0x30, (1 << 8))
    qt.writel(USB + 0x30, 0)
    check('a port reset enables the port', qt.readl(USB + 0x30) >> 2 & 1, 1)
    # A machine reset (the kernel's manual watchdog reset) clears the glue
    # as well as the EHCI.
    qt.writel(WDT + 0x00, 0x5A960112)
    qt.writel(WDT + 0x00, 0x5A960113)
    qt.writel(WDT + 0x0C, 1)
    qt.step(1000)
    check('a machine reset clears the glue, PHY and OTG bits',
          (qt.readl(USB + 0x400), qt.readl(USB + 0x1144),
           qt.readl(USB + 0x80) & 0x30), (0, 0, 0))
    check('and the EHCI: port disabled again',
          qt.readl(USB + 0x30) >> 2 & 1, 0)


def test_otp(qt):
    print('nvt-otp')
    check('powered down: not ready', qt.readl(OTP + 0x30) & 1, 0)
    qt.writel(OTP + 0x20, 1)
    check('efuse_open: CTRL bit 0, then STATUS bit 0',
          qt.readl(OTP + 0x30) & 1, 1)
    qt.writel(OTP + 0x04, 5)
    qt.writel(OTP + 0x20, 1 | (1 << 4))
    check('efuse_read: the start bit clears',
          qt.readl(OTP + 0x20) & (1 << 4), 0)
    check('STATUS bit 1: read done', qt.readl(OTP + 0x30) & 2, 2)
    check('a blank array reads 0', qt.readl(OTP + 0x08), 0)
    qt.writel(OTP + 0x30, 2)
    check('writing bit 1 back clears it', qt.readl(OTP + 0x30) & 2, 0)
    qt.writel(OTP + 0x20, 0)
    check('efuse_close: not ready again', qt.readl(OTP + 0x30) & 1, 0)


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

    # BCL1 algorithm 11, as "bfc c lzma" writes it: an LZMA "alone"
    # stream with the unpacked size filled in.
    lz = os.path.join(tmp, 'boot-lzma.bin')
    rng = random.Random(11)
    payload = bytes(rng.randrange(256) for _ in range(5000)) * 3
    alone = bytearray(lzma.compress(payload, format=lzma.FORMAT_ALONE))
    alone[5:13] = struct.pack('<Q', len(payload))
    make_boot_image(lz, bcl=b'BCL1' + struct.pack('>III', 11, len(payload),
                                                  len(alone)) + alone)
    qt = QTest(qemu, 'nt98562,flash-file=' + lz)
    try:
        check('U-Boot unpacked from BCL1 algorithm 11 (LZMA)',
              qt.read(0x3900000, len(payload)), payload)
    finally:
        qt.close()
    alone[-200:] = bytes(200)
    make_boot_image(bad, bcl=b'BCL1' + struct.pack('>III', 11, len(payload),
                                                   len(alone)) + alone)
    refused(bad, 'a BCL1 LZMA U-Boot that does not unpack is refused')
    os.unlink(lz)

    # Xiongmai's NA51089 layout: no fdt partition, the tree inside the
    # loader behind "XMFDT562", U-Boot LZMA-packed in partition_boot.
    xm = os.path.join(tmp, 'boot-xm.bin')
    payload = bytes(rng.randrange(256) for _ in range(3000)) * 2
    dtb = fdt_blob({
        'nvt_memory_cfg': {
            '#address-cells': cells(1), '#size-cells': cells(1),
            'fdt': {'reg': cells(0x100000, 0x100000)},
            'shmem': {'reg': cells(0x200000, 0x100000)},
            'uboot': {'reg': cells(0x3900000, 0x6C0000)},
        },
        'nor@f0400000': {
            'compatible': b'nvt,nvt_spinor\0',
            '#address-cells': cells(2), '#size-cells': cells(2),
            'partition_loader': {'reg': cells(0, 0, 0, 0x10000)},
            'partition_boot': {'reg': cells(0, 0x10000, 0, 0x30000),
                               'lzma': b''},
        },
    })
    img = bytearray(b'\xff' * (8 << 20))
    img[0:0x100] = bytes(rng.randrange(256) for _ in range(0x100))
    img[0x8000:0x8008] = b'XMFDT562'
    img[0x8008:0x8008 + len(dtb)] = dtb
    packed = lzma.compress(payload, format=lzma.FORMAT_ALONE)
    img[0x10000:0x10000 + len(packed)] = packed
    open(xm, 'wb').write(img)
    qt = QTest(qemu, 'nt98562,flash-file=' + xm)
    try:
        check('Xiongmai: the device tree found in the loader',
              qt.read(0x100000, len(dtb)), dtb)
        check('and the LZMA partition_boot unpacked as U-Boot',
              qt.read(0x3900000, len(payload)), payload)
    finally:
        qt.close()
    img[0x10000:0x10000 + len(packed)] = payload[:len(packed)]
    open(bad, 'wb').write(img)
    refused(bad, 'a partition marked lzma that is not LZMA is refused')
    os.unlink(xm)

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


# ---- nt98533: NS02501 ----------------------------------------------------

def test_nt98533(qemu, tmp):
    """NA51089's blocks at NA51089's addresses around a GIC-400, and the
    SDK NOR layout's flash boot (FDT at 0x10000, plain U-Boot)."""
    print('nt98533 (NS02501)')
    path = os.path.join(tmp, 'ns02501.bin')
    payload, dtb = make_boot_image(path, bcl=b'')
    img = bytearray(open(path, 'rb').read())
    rng = random.Random(533)
    payload = bytes(rng.randrange(256) for _ in range(4096))
    img[0x50000:0x50000 + len(payload)] = payload
    open(path, 'wb').write(img)
    qt = QTest(qemu, 'nt98533,flash-file=' + path)
    try:
        check('chip ID 0x5521 (CHIP_NS02501) in TOP_VERSION[31:16]',
              qt.readl(TOP + 0xF0) >> 16, 0x5521)
        typer = qt.readl(0xFFF01004)
        check('GIC-400 distributor at 0xFFF01000: 128 SPIs, two CPUs',
              (typer & 0x1F, (typer >> 5) & 7), (4, 1))
        qt.writel(0xF0090104, 0x12345678)
        check('core communicator command buffer keeps its value',
              qt.readl(0xF0090104), 0x12345678)
        check('UART 0 at 0xF0290000 (16550 LSR: THR empty)',
              qt.readl(0xF0290014) & 0x60, 0x60)
        check('plain U-Boot copied to /nvt_memory_cfg/uboot',
              qt.read(0x3900000, len(payload)), payload)
        check('SHMINFO at /nvt_memory_cfg/shmem says LD_NVT',
              qt.read(0x200000, 6), b'LD_NVT')
        check('a branch to U-Boot at 0',
              (qt.readl(0), qt.readl(4)) == (0xE51FF004, 0x3900000), True)
    finally:
        qt.close()
    out = subprocess.run([qemu, '-M', 'nt98533', '-display', 'none',
                          '-nodefaults', '-kernel', path],
                         capture_output=True, text=True, timeout=30)
    check('-kernel without a board tree is refused',
          out.returncode == 1 and 'needs a board device tree' in out.stderr,
          True)
    os.unlink(path)


# ---- nt98332: the NA51103 flash boot ----------------------------------

NA51103_UBOOT = 0x1E000000


def make_nt98332_image(path, uboot_part):
    """A 16 MiB NOR laid out as Xiongmai's NA51103 images are: the device
    tree LZMA-packed behind "XMFDT32G" at 0x10000, U-Boot in
    partition_boot at 0x20000."""
    tree = {
        '#address-cells': cells(2), '#size-cells': cells(2),
        'nvt_memory_cfg': {
            '#address-cells': cells(1), '#size-cells': cells(1),
            'fdt': {'reg': cells(0x100000, 0x100000)},
            'shmem': {'reg': cells(0xA00000, 0x100000)},
            'uboot': {'reg': cells(NA51103_UBOOT, 0x2000000)},
        },
        'nor@2,f0180000': {
            'compatible': b'nvt,nvt_spinor\0',
            '#address-cells': cells(2), '#size-cells': cells(2),
            'partition_boot': {'reg': cells(0, 0x20000, 0, 0x40000)},
        },
    }
    dtb = fdt_blob(tree)
    packed = lzma.compress(dtb, format=lzma.FORMAT_ALONE)
    img = bytearray(b'\xff' * (16 << 20))
    img[0x10000:0x10008] = b'XMFDT32G'
    img[0x10008:0x10008 + len(packed)] = packed
    img[0x20000:0x20000 + len(uboot_part)] = uboot_part
    open(path, 'wb').write(img)
    return dtb


def test_nt98332_boot(qemu, tmp):
    print('nt98332 flash boot')
    path = os.path.join(tmp, 'nt98332.bin')
    rng = random.Random(11)
    # an AArch64 U-Boot's head: b reset, nop, then _TEXT_BASE as a quad
    head = struct.pack('<IIQ', 0x1400000A, 0xD503201F, NA51103_UBOOT)
    payload = head + bytes(rng.randrange(256) for _ in range(4096)) * 4

    def boot(part, what, want):
        dtb = make_nt98332_image(path, part)
        qt = QTest(qemu, 'nt98332,flash-file=' + path)
        try:
            check(what, qt.read(NA51103_UBOOT, len(want)), want)
            check('  the device tree unpacked to /nvt_memory_cfg/fdt',
                  qt.read(0x100000, len(dtb)), dtb)
            check('  SHMINFO says LD_NVT', qt.read(0xA00000, 6), b'LD_NVT')
            check('  an AArch64 branch to U-Boot at 0',
                  struct.unpack('<IIQ', qt.read(0, 16)),
                  (0x58000041, 0xD61F0020, NA51103_UBOOT))
        finally:
            qt.close()

    boot(lzma.compress(payload, format=lzma.FORMAT_ALONE),
         'U-Boot unpacked from LZMA (end marker, size unknown)', payload)
    boot(lzma.compress(payload, format=lzma.FORMAT_ALONE,
                       filters=[{'id': lzma.FILTER_LZMA1,
                                 'dict_size': 3 << 20}]),
         'LZMA with a 3 MiB dictionary', payload)
    boot(payload, 'a plain AArch64 U-Boot is not taken for LZMA', payload)
    bad = bytearray(lzma.compress(payload, format=lzma.FORMAT_ALONE))
    bad[64:96] = bytes(32)
    boot(bytes(bad), 'LZMA that does not unpack falls back to plain',
         bytes(bad[:4096]))
    os.unlink(path)


# ---- nt98332: decoder and VPE link lists ------------------------------

NA51103_DEC = 0x2F0600000
NA51103_VPE = 0x2F0410000
NA51103_GICD = 0x2FFF01000


def na51103_pending(qt, spi):
    n = 32 + spi
    return bool(qt.readl(NA51103_GICD + 0x200 + (n // 32) * 4) &
                (1 << (n % 32)))


def test_nt98332_dec(qt):
    """The H.26x decoder's link-list protocol, as h26xdec.ko drives it."""
    print('nt98332: H.26x decoder link list')
    ll = 0x01000000

    def cmd(op, off, val):
        return struct.pack('<II', (op << 28) | off, val)
    lst = (cmd(0xC, 0, ll + 8) +                 # link to the next entry
           cmd(0, 0x1814, 1) +                   # BIN reset
           cmd(8, 0x1814, 0x10000) +             # wait for it
           cmd(0, 0x103C, 0xFF) +                # clear the flags
           cmd(0, 0x1040, 0x02000000) +          # picture Y
           cmd(0, 0x1000, 2) +                   # decode
           cmd(8, 0x1000, 0x10000) +             # wait for done
           cmd(0xC, 0, 0xFFFFFFF0))              # end of the job
    qt.write(ll, lst)
    qt.writel(NA51103_DEC + 0x1000, 0x10003)
    check('a reset clears itself (and wins over the start bit)',
          qt.readl(NA51103_DEC + 0x1000) & 3, 0)
    check('the control word reads done', qt.readl(NA51103_DEC + 0x1000) >> 16
          & 1, 1)
    qt.writel(NA51103_DEC + 0x3004, ll)
    qt.writel(NA51103_DEC + 0x3000, 2)
    qt.step(5 * 1000 * 1000)
    check('the list ran to its end', qt.readl(NA51103_DEC + 0x301C),
          0xFFFFFFF0)
    check('the register writes landed', qt.readl(NA51103_DEC + 0x1040),
          0x02000000)
    check('decode done in 0x103C (errors 0x16 clear)',
          qt.readl(NA51103_DEC + 0x103C), 1)
    check('job interrupt status', qt.readl(NA51103_DEC + 0x184C) & 1, 1)
    check('SPI 9 raised', na51103_pending(qt, 9), True)
    qt.writel(NA51103_DEC + 0x1848, 1)
    check('acknowledged at 0x1848', qt.readl(NA51103_DEC + 0x184C) & 1, 0)
    check('and the line drops', na51103_pending(qt, 9), False)

    # H.265: AD (0x2000) and BIN (0x2800), done taken by writing bit 16
    # back, the interrupt behind 0x20DC; 0x211C is an error register
    lst = (cmd(0xC, 0, ll + 8) +
           cmd(0, 0x1800, 0) +                   # H.265
           cmd(0, 0x20DC, 0x41) +                # interrupt enable
           cmd(0, 0x28B8, 0x02000000) +          # picture Y
           cmd(0, 0x2000, 0xE) +                 # AD
           cmd(0, 0x2800, 6) +                   # BIN
           cmd(0xC, 0, 0xFFFFFFF0))
    qt.write(ll, lst)
    qt.writel(NA51103_DEC + 0x2000, 1)
    check('an H.265 AD reset clears itself',
          qt.readl(NA51103_DEC + 0x2000) & 1, 0)
    qt.writel(NA51103_DEC + 0x3000, 2)
    qt.step(5 * 1000 * 1000)
    check('H.265: the list ran to its end', qt.readl(NA51103_DEC + 0x301C),
          0xFFFFFFF0)
    check('AD and BIN read done',
          (qt.readl(NA51103_DEC + 0x2000) >> 16 & 1,
           qt.readl(NA51103_DEC + 0x2800) >> 16 & 1), (1, 1))
    check('no error in 0x211C', qt.readl(NA51103_DEC + 0x211C), 0)
    check('nothing at 0x184C', qt.readl(NA51103_DEC + 0x184C) & 1, 0)
    check('SPI 9 raised', na51103_pending(qt, 9), True)
    qt.writel(NA51103_DEC + 0x2000, 0x10000)
    check('taken by writing done back to AD', na51103_pending(qt, 9), False)
    qt.writel(NA51103_DEC + 0x3000, 2)
    qt.step(5 * 1000 * 1000)
    qt.writel(NA51103_DEC + 0x20DC, 0)
    check('or by turning its enable off', na51103_pending(qt, 9), False)


def test_nt98332_vpe(qt):
    """The VPE's fill and scale jobs, nvt-engine with scaler=on."""
    print('nt98332: VPE fill and scale')
    ll, src, dst = 0x01100000, 0x01200000, 0x01400000
    fw, fh = 64, 32

    def job(regs):
        lst = b''.join(struct.pack('<II', v, 0x8F000000 | off)
                       for off, v in regs)
        lst += struct.pack('<II', 0, 0)
        qt.write(ll, lst)
        qt.writel(NA51103_VPE + 0x300, ll)
        qt.writel(NA51103_VPE + 0x000, 0x80000000)
        qt.step(5 * 1000 * 1000)

    frame = [(0x050C, fh << 16 | fw), (0x0360, dst)]
    job(frame + [(0x0008, 0), (0x0260, 0x00804010),
                 (0x0550, fw), (0x0508, fh)])
    check('a fill job paints the background colour, Y',
          qt.read(dst, 8), b'\x10' * 8)
    check('and UV', qt.read(dst + fw * fh, 4), b'\x40\x80\x40\x80')
    check('it reports done (frame end, list end)',
          qt.readl(NA51103_VPE + 0x400) & 5, 5)
    qt.writel(NA51103_VPE + 0x400, 5)

    # an 8x4 source: Y counts across, UV a fixed pair
    qt.write(src, bytes(range(32)))
    qt.write(src + 32, b'\x11\x22' * 8)
    job(frame + [(0x0008, 0xC00), (0x0010, 4 << 16 | 8), (0x0308, src),
                 (0x0800, 8), (0x0550, 4), (0x0508, 2)])
    check('a scale job halves the source into the window, Y',
          qt.read(dst, 4), bytes([0, 2, 4, 6]))
    check('the next window row', qt.read(dst + fw, 4),
          bytes([16, 18, 20, 22]))
    check('outside the window stays filled', qt.read(dst + 4, 2),
          b'\x10\x10')
    check('UV, one row of pairs', qt.read(dst + fw * fh, 4),
          b'\x11\x22\x11\x22')

    # a full picture (0x1000), of which 0x01C shows the top 8x2: two rows
    # of padding below it, then UV
    qt.write(src, bytes(range(32)))
    qt.write(src + 32, b'\x33\x44' * 8)
    job(frame + [(0x0008, 0x1000), (0x0010, 4 << 16 | 8), (0x001C, 8 << 16 | 2),
                 (0x0308, src), (0x0800, 8), (0x0550, 4), (0x0508, 2)])
    check('a full-picture job scales the part shown, Y',
          qt.read(dst, 4), bytes([0, 2, 4, 6]))
    check('and leaves out the padding rows', qt.read(dst + fw, 4),
          bytes([8, 10, 12, 14]))
    check('its UV follows the whole buffer', qt.read(dst + fw * fh, 4),
          b'\x33\x44\x33\x44')

    # a window placed at (8, 4) in the frame: Y 4 rows and 8 columns in,
    # UV 2 rows and 8 columns into the frame's UV
    job(frame + [(0x0008, 0), (0x0260, 0x00804010),
                 (0x0550, fw), (0x0508, fh)])
    job(frame + [(0x0008, 0), (0x0260, 0x00AABB20), (0x0554, 8),
                 (0x0510, 4), (0x0550, 4), (0x0508, 2)])
    check('a placed window starts at its x / y, Y',
          qt.read(dst + 4 * fw + 6, 6), b'\x10\x10\x20\x20\x20\x20')
    check('and not at the frame origin', qt.read(dst, 2), b'\x10\x10')
    check('its UV at half the rows', qt.read(dst + fw * fh + 2 * fw + 8, 4),
          b'\xbb\xaa\xbb\xaa')
    check('the row above it untouched', qt.read(dst + fw * fh + fw + 8, 2),
          b'\x40\x80')


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
    check('the USB host and OTP nodes ehci-na51089.c needs',
          b'nvt,ehci-nvtivot\0' in data and b'nvt,nvt_otp\0' in data, True)
    check('the PWM node pwm-na51055.c binds', b'nvt,nvt_pwm\0' in data, True)
    check('the HDAL media nodes, units and limits',
          find_nodes(data, ['sie@f0c00000', 'ime@f0c40000', 'h26x@f0a10000',
                            'jpg@f0a00000', 'dai@f0630000', 'nvtmpp',
                            'isf_vdoenc', 'hdal-maxpath-cfg', 'hdal-memory',
                            'pll_preset@0']).count(-1), 0)
    check('the crypto, hash and RSA nodes the na51089 drivers bind',
          find_nodes(data, ['crypto@f0620000', 'hash@f0670000',
                            'rsa@f06a0000']).count(-1) == 0 and
          all(b'nvt,nvt_%s\0' % n in data for n in (b'crypto', b'hash', b'rsa')),
          True)
    check('the timer node kdrv_comm.ko binds: timer@f0040000, nvt,nvt_timer',
          find_nodes(data, ['timer@f0040000'])[0] > 0 and
          b'nvt,nvt_timer\0' in data, True)
    check('the eMMC host node: mmc@f0510000, nvt,nvt_mmc3',
          find_nodes(data, ['mmc@f0510000'])[0] > 0 and
          b'nvt,nvt_mmc3\0' in data, True)
    check('three I2C nodes under the names i2c-nvt.c clocks by',
          find_nodes(data, ['i2c@f0220000', 'i2c2@f0350000',
                            'i2c3@f03a0000']).count(-1), 0)
    check('the NOR under spinor.c', b'nvt,nvt_spinor\0' in data and
          b'spi_nor.0\0' in data, True)

    subprocess.run([qemu, '-M', 'nt98562', '-m', '64M',
                    '-machine', 'dumpdtb=' + dtb, '-kernel', kernel,
                    '-display', 'none', '-nodefaults'],
                   capture_output=True, timeout=30)
    small = open(dtb, 'rb').read()
    check('-m 64: the SDK split, Linux 30 MB, the media pool the rest',
          (find_nodes(small, ['hdal-memory'])[0] > 0,
           cells(0, 0x01E00000) in small,
           cells(0x01E00000, 0x02200000) in small), (True, True, True))
    check('224 MB: Linux 128 MB, the pool 96 MB after it',
          (cells(0, 0x08000000) in data, cells(0x08000000, 0x06000000) in data),
          (True, True))

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
    for m in ('nt98562', 'nt98566', 'nt98566,sensor=sc401ai'):
        mig = os.path.join(tmp, m.replace(',', '_') + '.mig')
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
    ap.add_argument('--qemu-aarch64',
                    default='./qemu-src/build/qemu-system-aarch64')
    args = ap.parse_args()
    # Images up to 128 MiB go here, under /tmp: always remove them.
    with tempfile.TemporaryDirectory() as tmp:
        run(args, tmp)


def run(args, tmp):

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

    card = os.path.join(tmp, 'sd.img')
    open(card, 'wb').write(random.Random(9).randbytes(4 << 20))
    qt = QTest(args.qemu, 'nt98562',
               ['-drive', 'if=sd,index=0,format=raw,file=' + card])
    try:
        test_sdio(qt, card)
    finally:
        qt.close()

    emmc = os.path.join(tmp, 'emmc.img')
    open(emmc, 'wb').write(bytes(4 << 20))
    qt = QTest(args.qemu, 'nt98566',
               ['-drive', 'if=sd,index=2,format=raw,file=' + emmc])
    try:
        test_emmc(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566,sensor=sc401ai')
    try:
        test_i2c(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_pwm(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98562')
    try:
        test_timer(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_media(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_pixels(qt)
    finally:
        qt.close()

    mon = os.path.join(tmp, 'mon.sock')
    qt = QTest(args.qemu, 'nt98566',
               ['-monitor', 'unix:%s,server=on,wait=off' % mon])
    try:
        test_display(qt, mon)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_dsi(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_ai(qt)
        test_ai_compute(qt)
        test_ai_detect(qt)
        test_ai_face(qt)
        test_md(qt)
    finally:
        qt.close()

    wav = os.path.join(tmp, 'audio.wav')
    qt = QTest(args.qemu, 'nt98566,audiodev=snd0',
               ['-audiodev', 'wav,id=snd0,path=' + wav])
    try:
        test_audio(qt, wav)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_gfx(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_crypto(qt)
        test_hash(qt)
        test_rsa(qt)
    finally:
        qt.close()

    stick = os.path.join(tmp, 'stick.img')
    open(stick, 'wb').write(bytes(1 << 20))
    qt = QTest(args.qemu, 'nt98566',
               ['-drive', 'if=none,id=s,format=raw,file=' + stick,
                '-device', 'usb-storage,drive=s'])
    try:
        test_usb(qt, True)
        test_otp(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_usb(qt, False)
        print('nt98566')
        check('same die, same chip ID', qt.readl(TOP + 0xF0) >> 16, 0x7021)
        check('no card: C_GPIO 9 high', qt.readl(GPIO + 0x00) >> 9 & 1, 1)
    finally:
        qt.close()

    locks = os.path.join(tmp, 'locks.bin')
    open(locks, 'wb').write(random.Random(13).randbytes(8 << 20))
    qt = QTest(args.qemu, 'nt98562,flash-file=' + locks, ['-kernel', stub])
    try:
        test_flash_locks(qt, locks)
    finally:
        qt.close()
    os.unlink(locks)

    test_nt98533(args.qemu, tmp)

    if os.path.exists(args.qemu_aarch64):
        test_nt98332_boot(args.qemu_aarch64, tmp)
        qt = QTest(args.qemu_aarch64, 'nt98332')
        try:
            test_nt98332_dec(qt)
            test_nt98332_vpe(qt)
        finally:
            qt.close()
    else:
        print('nt98332: %s not built, skipped' % args.qemu_aarch64)

    test_uart_rx_after_fifo_reset(args.qemu, tmp)
    test_fdt(args.qemu, tmp)
    test_migration(args.qemu, tmp)

    print('%d failure(s)' % failures)
    if failures:
        sys.exit(1)


if __name__ == '__main__':
    main()
