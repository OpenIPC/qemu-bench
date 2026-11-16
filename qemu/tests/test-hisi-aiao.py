#!/usr/bin/env python3
"""The V4 AIAO (hisi-aiao) at 0x100e0000: capture and playback rings, as
aio.ko programs them, with the values a lab GK7201V200 reads under majestic
(8 kHz mono: NCO 0x95218, dividers 0xC119, a 1024-byte capture ring, 320
bytes and 20 ms a transfer; playback 2560 bytes, 640 a transfer, 40 ms).

No guest: QEMU runs under qtest and this script writes the registers.

    python3 qemu/tests/test-hisi-aiao.py [--qemu PATH]
"""
import argparse, math, os, struct, sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

AIAO = 0x100e0000
INT_ENA, INT_STA = 0x0000, 0x0004
RX, TX = 0x1000, 0x2000
RX_CRG, TX_CRG = 0x0100, 0x0140
ATTR, CTRL, ADDR, SIZE, WPTR, RPTR, TRANS = 0x00, 0x04, 0x80, 0x84, 0x88, 0x8c, 0x94
MASK, RAW, STA, CLR = 0xa0, 0xa4, 0xa8, 0xac
START, STOPPED = 1 << 28, 1 << 29
MONO16 = 0xe4800004             # what aio.ko writes for 16-bit mono
RXBUF, TXBUF = 0x43000000, 0x43010000
# rate: (NCO, dividers, capture ring, capture transfer), from the board
RATES = {8000: (0x95218, 0xC119, 0x400, 0x140),
         16000: (0x95218, 0xC117, 0x780, 0x280),
         48000: (0xDFB24, 0xC115, 0xB80, 0x3C0)}


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

    for mach in ('hi3516ev200', 'gk7201v200'):
        q = jt.QTest(a.qemu, mach)
        r = lambda off: q.readl(AIAO + off)
        w = lambda off, v: q.writel(AIAO + off, v)
        step = lambda ms: q.cmd('clock_step %d' % int(ms * 1e6))
        try:
            check(r(RX + CTRL) & STOPPED and r(TX + CTRL) & STOPPED,
                  '%s: both directions read stopped at reset' % mach)
            # Pointers and statuses read before anything started (no rate)
            vals = [r(o) for o in (RX + WPTR, RX + RPTR, RX + STA, RX + RAW,
                                   TX + WPTR, TX + RPTR, TX + STA, INT_STA)]
            check(vals == [0] * 8,
                  '%s: pointers and statuses read 0 before a start' % mach)

            def capture(rate):
                nco, div, size, trans = RATES[rate]
                w(RX + CTRL, 0)
                w(RX_CRG, nco)
                w(RX_CRG + 4, div)
                w(RX + ATTR, MONO16)
                w(RX + ADDR, RXBUF)
                w(RX + SIZE, size)
                w(RX + TRANS, trans)
                w(RX + WPTR, 0)
                w(RX + RPTR, 0)
                w(RX + CLR, 0x3f)
                w(RX + MASK, 0xb)
                w(INT_ENA, 1)
                w(RX + CTRL, START)
                return size, trans

            # 8 kHz: one transfer each 20 ms, a 1 kHz sine in it
            _, trans = capture(8000)
            check(not r(RX + CTRL) & STOPPED, '%s: capture runs' % mach)
            step(19)
            check(r(RX + WPTR) == 0 and not r(INT_STA),
                  '%s: nothing before 20 ms' % mach)
            step(2)
            check(r(RX + WPTR) == trans and r(RX + STA) == 1 and
                  r(INT_STA) == 1,
                  '%s: 320 bytes and a transfer interrupt at 20 ms '
                  '(wptr 0x%x, status 0x%x, top 0x%x)'
                  % (mach, r(RX + WPTR), r(RX + STA), r(INT_STA)))
            pcm = struct.unpack('<160h', q.read(RXBUF, trans))
            want = [round(16000 * math.sin(2 * math.pi * 1000 * n / 8000))
                    for n in range(160)]
            check(all(abs(x - y) <= 1 for x, y in zip(pcm, want)),
                  '%s: the transfer is a 1 kHz sine at 8 kHz' % mach)
            w(RX + CLR, 1)
            check(r(RX + STA) == 0 and r(INT_STA) == 0,
                  '%s: writing 1 clears it' % mach)

            # Nobody reads: three transfers fit in 1024 bytes, the fourth
            # waits, and a second later the ring says it was full.
            step(40)
            check(r(RX + WPTR) == 3 * trans, '%s: three transfers in the '
                  'ring at 60 ms (wptr 0x%x)' % (mach, r(RX + WPTR)))
            step(40)
            check(r(RX + WPTR) == 3 * trans and not r(RX + RAW) & 2,
                  '%s: the fourth waits for room' % mach)
            # The driver takes two: what was due follows at once.
            w(RX + RPTR, 2 * trans)
            check(r(RX + WPTR) == (5 * trans) % 0x400,
                  '%s: moving the read pointer lets the backlog in '
                  '(wptr 0x%x)' % (mach, r(RX + WPTR)))
            step(1100)
            check(r(RX + RAW) & 2, '%s: a second unread raises full' % mach)

            # The rate comes from the clock words, as the board programs them
            for rate in (16000, 48000):
                # read as the driver does, every 10 ms, for a second
                size, trans = capture(rate)
                per_s = rate * 2 // trans
                got = 0
                for _ in range(100):
                    step(10)
                    wp = r(RX + WPTR)
                    got += (wp - r(RX + RPTR)) % size // trans
                    w(RX + RPTR, wp)
                check(abs(got - per_s) <= 1, '%s: %d Hz gives %d transfers '
                      'in a second (want %d)' % (mach, rate, got, per_s))
            w(RX + CTRL, 0)
            check(r(RX + CTRL) & STOPPED, '%s: capture stops' % mach)
            w(RX + CLR, 0x3f)

            # Transfers that are not whole frames move nothing, and do not
            # spin: zero bytes, then three bytes of 16-bit mono
            for bad in (0, 3):
                capture(8000)
                w(RX + CTRL, 0)
                w(RX + TRANS, bad)
                w(RX + CTRL, START)
                step(100)
                check(r(RX + WPTR) == 0 and not r(RX + RAW),
                      '%s: a %d-byte transfer moves nothing' % (mach, bad))
                w(RX + CTRL, 0)

            def count(size, trans, secs):
                got = 0
                for _ in range(int(secs * 100)):
                    step(10)
                    wp = r(RX + WPTR)
                    got += (wp - r(RX + RPTR)) % size // trans
                    w(RX + RPTR, wp)
                return got

            # 16 kHz's dividers written while 8 kHz runs: twice the transfers
            size, trans = capture(8000)
            first = count(size, trans, 1)
            w(RX_CRG + 4, RATES[16000][1])
            second = count(size, trans, 1)
            check(abs(first - 50) <= 1 and abs(second - 100) <= 1,
                  '%s: a live clock change takes effect (%d then %d '
                  'transfers a second)' % (mach, first, second))
            w(RX + CTRL, 0)

            # 11025 Hz: MCLK 1024 x 11025 is NCO 841814; 220 samples a
            # transfer is 50.1 a second, 501 in ten (11000 Hz would be 500)
            w(RX_CRG, 841814)
            w(RX_CRG + 4, 0xC119)
            w(RX + TRANS, 440)
            w(RX + WPTR, 0)
            w(RX + RPTR, 0)
            w(RX + CTRL, START)
            got = count(0x400, 440, 10)
            check(got == 501, '%s: 11025 Hz keeps its rate (%d transfers in '
                  '10 s, want 501)' % (mach, got))
            w(RX + CTRL, 0)
            w(RX + CLR, 0x3f)

            # A playback ring aimed at the device's own registers: the DMA
            # reads them, and that must not start another transfer
            w(TX_CRG, RATES[8000][0])
            w(TX_CRG + 4, RATES[8000][1])
            w(TX + ATTR, 0xe4000044)
            w(TX + ADDR, AIAO + TX)
            w(TX + SIZE, 0x100)
            w(TX + TRANS, 0x40)
            w(TX + RPTR, 0)
            w(TX + WPTR, 0x80)
            w(TX + CTRL, START)
            step(50)
            check(r(TX + CTRL) & START,
                  '%s: a ring over its own registers does not recurse' % mach)
            w(TX + CTRL, 0)
            w(TX + CLR, 0x1f)

            # Playback, 8 kHz: 640 bytes leave every 40 ms
            nco, div = RATES[8000][:2]
            ttrans = 0x280
            w(INT_ENA, 1 << 16)
            w(TX_CRG, nco)
            w(TX_CRG + 4, div)
            w(TX + ATTR, 0xe4000044)
            w(TX + ADDR, TXBUF)
            w(TX + SIZE, 0xA00)
            w(TX + TRANS, ttrans)
            w(TX + RPTR, 0)
            w(TX + WPTR, 0x500)
            w(TX + MASK, 0x1b)
            w(TX + CTRL, START)
            step(41)
            check(r(TX + RPTR) == ttrans and r(TX + STA) == 1 and
                  r(INT_STA) == 1 << 16,
                  '%s: playback takes 640 bytes at 40 ms (rptr 0x%x, '
                  'status 0x%x, top 0x%x)'
                  % (mach, r(TX + RPTR), r(TX + STA), r(INT_STA)))
            w(TX + CLR, 0x1f)
            step(80)
            check(r(TX + RPTR) == 0x500 and r(TX + RAW) & 2,
                  '%s: and runs empty when the driver stops writing '
                  '(rptr 0x%x raw 0x%x)' % (mach, r(TX + RPTR), r(TX + RAW)))
            w(TX + CTRL, 0)
            check(r(TX + CTRL) & STOPPED, '%s: playback stops' % mach)
        finally:
            q.close()

    print('%d failure(s)' % fails)
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
