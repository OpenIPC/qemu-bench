#!/usr/bin/env python3
"""The AK3918EV300's Allegro encoder (hw/misc/ingenic-avpu.c with "anyka")
under qtest.

No guest: QEMU runs the ak3918ev300 machine and this script plays the
SDK's libakv_encode 2.2 (CoreManager.c, SchedulerCpu.c, ChannelMngr.c
StartEncodingProcess, EncCmdRegs.c), with the encoder's window at
0x20020000 and the Allegro registers from +0x8000:

  - the core init: 0x8010 / 0x8018 / the clock gate at 0x8020 / 0x822C /
    0x8034 / 0x8054, for H.264 on one core Enc2's done bit alone enabled
    in the mask at 0x8014 (the picture ends with its entropy coding);
    the core's status at 0x8300 reads idle, the start at 0x8084 reads 0;
  - the L2 prefetch at 0xC004-0xC014 (and 0x9204 / 0x9208) programmed per
    picture with the source planes and pitch the way StartEncodingProcess
    does: plain registers, read back; a picture whose L2 addresses are not
    its own is reported (-d guest_errors), one with L2 bypassed is not;
  - H.264 CABAC as libakv runs it: the command list address at 0x8080 and
    start 2 (Enc1) then 8 (Enc2) at 0x8084 on the same list; W8 bit 11 set
    with no Enc1 stream buffer: Enc2 codes the slice into WF0-WFC after
    the header libakv wrote (no tail bits); an IDR, a P picture of two
    slices (list entries 0x1C0 bytes apart) and another P picture;
  - while a picture is in flight 0x8300 has its busy bits (0 Enc1, 2
    Enc2), and the done bits (0, 2 at 0x8018) are not up inside the start
    write; afterwards module interrupt 2 at the system controller, the
    W1C ack drops it;
  - the results: 0x8304 the picture's bytes (the slices' sum), the
    entries' entropy status at 0x100 (bit 3 overflow) / 0x104 (bytes) and
    Enc1's at 0x130-0x16C (QP min / max and LCUs at 0x138, 64-bit
    counters high word first at 0x158 / 0x15C), nothing past 0x16C;
  - a stream buffer too small: 0x8300 bit 1 and the entry's 0x100 bit 3,
    cleared again by the next picture that fits;
  - JPEG: R0-R10 at 0x83C0, start 0x10, done bit 18, bytes at 0x83F4, no
    overflow at 0x83F8; the T31 layout's JPEG start (0x83E4 = R9 here)
    starts nothing.

The stream is decoded with ffmpeg if there is one: no errors, the I
picture's macroblock means within 3 of the source's, the P pictures'
luma PSNR at least 30 dB; the JPEG's luma PSNR above 30 dB.

    python3 qemu/tests/test-ak-allegro.py [--qemu PATH] [--ffmpeg CMD]
"""
import argparse
import importlib.util
import math
import os
import shutil
import sys
import tempfile
import time

sys.dont_write_bytecode = True
_spec = importlib.util.spec_from_file_location(
    'avpu', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'test-ingenic-avpu.py'))
avpu = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(avpu)

VENC = 0x20020000
R = VENC + 0x8000                   # the Allegro registers
SYSCTRL = 0x08000000
MOD_STATUS = SYSCTRL + 0x4C         # module interrupt status, bit 2 VENC
DDR = 0x80000000
A = DDR + 0x01000000                # the test's buffers
W, H, PITCH = avpu.W, avpu.H, avpu.PITCH
MBW, MBH = W // 16, H // 16
ENTRY = 0x1C0
QP, CQP, IDC = 30, 2, 0

failures = 0


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what,
                             hex(got) if isinstance(got, int) and
                             not isinstance(got, bool) else got))
    return ok


def words(e):
    return b''.join(v.to_bytes(4, 'little') for v in e)


def wait_done(qt, bits):
    """Virtual time until the done bits are up: the coding runs on a host
    thread of its own, so a loaded host can take longer than one step."""
    for _ in range(400):
        qt.step(1000000)
        st = qt.readl(R + 0x18)
        if st & bits == bits:
            return st
        time.sleep(0.02)
    return st


def core_init(qt):
    """AL_EncCore_Init, AL_EncCore_SetInterrupts for one core."""
    print('core init')
    qt.writel(R + 0x10, 0xFFFFFFFF)
    qt.writel(R + 0x18, 0x00FFFFFF)
    qt.writel(R + 0x20, qt.readl(R + 0x20) | 1)
    qt.writel(R + 0x22C, 0)
    qt.writel(R + 0x20, qt.readl(R + 0x20) & ~1)
    qt.writel(R + 0x34, 0)
    qt.writel(R + 0x54, 0x70)
    # StartEncodingProcess for H.264 on one core: Enc2's done bit (2) on,
    # Enc1's (0) off -- the picture ends with its entropy coding
    qt.writel(R + 0x14, (qt.readl(R + 0x14) | 4) & ~1)
    check('interrupt mask: Enc2 of core 0', qt.readl(R + 0x14), 4)
    check('core 0 status (0x8300) idle', qt.readl(R + 0x300), 0)
    check('the start register (0x8084) reads 0', qt.readl(R + 0x84), 0)
    check('0x801C (libakv polls it each picture) reads 0',
          qt.readl(R + 0x1C), 0)
    check('nothing pending', qt.readl(R + 0x18), 0)


def l2_program(qt, src, pitch, w4, bypass=False):
    """StartEncodingProcess: the L2 prefetch for one picture (AVC)."""
    qt.writel(R + 0x1208, 0xABCDABCD)
    qt.writel(R + 0x1204, 0x5000)
    if bypass:
        qt.writel(R + 0x4004, (qt.readl(R + 0x4004) & ~0x8000) | 0xE00)
        return
    qt.writel(R + 0x4004, (qt.readl(R + 0x4004) & ~0x8000 & ~0xC00) | 0x210)
    qt.writel(R + 0x4014, qt.readl(R + 0x4014) | 0x80000)
    qt.writel(R + 0x4008, src)
    qt.writel(R + 0x400C, src + pitch * H)
    qt.writel(R + 0x204, w4)
    v = qt.readl(R + 0x4014)
    qt.writel(R + 0x4014, (v & ~0x7FFC0) | ((pitch << 6) & 0x7FFC0))
    qt.writel(R + 0x4004, qt.readl(R + 0x4004) | 1)


def slice_header(idr, k, first_mb):
    b = avpu.Bits()
    b.ue(first_mb)
    b.ue(7 if idr else 5)
    b.ue(0)
    b.u(k & 15, 4)
    if idr:
        b.ue(0)
    b.u((2 * k) & 15, 4)
    if not idr:
        b.u(0, 1); b.u(0, 1)
    if idr:
        b.u(0, 1); b.u(0, 1)
    else:
        b.u(0, 1)
    if not idr:
        b.ue(IDC)
    b.se(QP - 26); b.ue(0); b.se(0); b.se(0)
    while len(b.b) % 8:
        b.u(1, 1)                       # cabac_alignment_one_bit
    return b


def picture(qt, k, y, uv, slices, room=None, l2_wrong=False,
            l2_bypass=False, lost_irq=False):
    """One picture of len(slices) slices (their first MB rows), coded the
    way libakv starts it.  Returns ([(region, header length, bytes)],
    status of 0x8018, core status, core bytes)."""
    src = A + 0x10000 * (k & 1)
    rec = A + 0x20000 + 0x10000 * (k & 1)
    prev = A + 0x20000 + 0x10000 * ((k - 1) & 1)
    cl, bs, bs_size = A + 0x48000, A + 0x50000, 0x80000
    idr = k == 0
    qt.write(src, y)
    qt.write(src + PITCH * H, uv)
    w4 = (W // 8 - 1) | ((H // 8 - 1) << 12)
    stype = 2 if idr else 1
    entries, regions = [], []
    for n, row in enumerate(slices):
        last = n == len(slices) - 1
        rows = (slices[n + 1] if not last else MBH) - row
        region = 0x20000 * n
        start = b''
        if idr and n == 0:
            start = avpu.sps(True, MBW, MBH) + avpu.pps(True, CQP)
        before = start + b'\x00\x00\x00\x01' + \
            bytes([0x65 if idr else 0x41]) + \
            avpu.escape(slice_header(idr, k, row * MBW).bytes())
        qt.write(bs + region, before)
        e = [0] * (ENTRY // 4)
        e[0] = (1 << 31) if last else 0
        e[1] = w4
        # CABAC, Enc1 hands the slice to Enc2 (bit 11), bit 13 as libakv
        e[2] = (IDC << 8) | (1 << 10) | (1 << 11) | (1 << 13)
        e[3] = (CQP & 31) | ((CQP & 31) << 8) | (QP << 16) | (stype << 28)
        e[5] = row << 12
        e[0x80 // 4], e[0x84 // 4] = src, src + PITCH * H
        e[0x88 // 4] = PITCH
        e[0x90 // 4], e[0x94 // 4] = rec, rec + W * H
        if not idr:
            e[0xA0 // 4], e[0xA4 // 4] = prev, prev + W * H
        e[0x70 // 4] = (IDC << 8) | (QP << 16) | (stype << 28)
        e[0x78 // 4] = rows * MBW - 1
        e[0xF0 // 4], e[0xF4 // 4] = bs, bs_size
        e[0xF8 // 4] = region + len(before)
        e[0xFC // 4] = room if room else 0x10000
        e[0x180 // 4] = 0x5A5A0000 | n      # a command word past the status
        entries.append(e)
        regions.append((region, len(before)))
    qt.write(cl, b''.join(words(e) for e in entries))
    l2_program(qt, src + (0x1000 if l2_wrong else 0), PITCH, w4,
               bypass=l2_bypass)
    # AL_EncCore_Encode1 with an Enc2 list: Enc1, then Enc2, same list
    qt.writel(R + 0x80, cl)
    qt.writel(R + 0x84, 2)
    qt.writel(R + 0x80, cl)
    qt.writel(R + 0x84, 8)
    early, busy = qt.readl(R + 0x18), qt.readl(R + 0x300)
    what = 'IDR' if idr else 'P%d' % k
    if lost_irq:
        # drop-irqs: the core finishes, the interrupt never comes
        for _ in range(400):
            qt.step(1000000)
            if qt.readl(R + 0x300) & 5 == 0:
                break
            time.sleep(0.02)
        check('%s, interrupt dropped: the core finishes (0x8300 idle)'
              % what, qt.readl(R + 0x300) & 5, 0)
        check('%s, interrupt dropped: no done bits, no module interrupt'
              % what, (qt.readl(R + 0x18) & 5,
                       qt.readl(MOD_STATUS) >> 2 & 1), (0, 0))
        return None
    st = wait_done(qt, 5)
    check('%s: not done inside the start write' % what, early & 5, 0)
    check('%s: Enc1 and Enc2 busy meanwhile (0x8300 bits 0, 2)' % what,
          busy & 5, 5)
    check('%s: Enc1 and Enc2 done (0x8018 bits 0, 2)' % what, st & 5, 5)
    check('%s: module interrupt 2 up' % what,
          qt.readl(MOD_STATUS) >> 2 & 1, 1)
    qt.writel(R + 0x18, st)
    check('%s: acked, interrupt down' % what,
          (qt.readl(R + 0x18), qt.readl(MOD_STATUS) >> 2 & 1), (0, 0))
    core, core_bytes = qt.readl(R + 0x300), qt.readl(R + 0x304)
    out = []
    for n, (region, hlen) in enumerate(regions):
        ea = cl + ENTRY * n
        flags, count = qt.readl(ea + 0x100), qt.readl(ea + 0x104)
        out.append((region, hlen, count, flags))
        c138 = qt.readl(ea + 0x138)
        rows = (slices[n + 1] if n + 1 < len(slices) else MBH) - slices[n]
        check('%s slice %d: QP min / max %d, LCUs at 0x138' % (what, n, QP),
              (c138 & 0xFF, (c138 >> 8) & 0xFF, c138 >> 16),
              (QP, QP, rows * MBW))
        check('%s slice %d: 64-bit counters high word first (0x158 0, '
              '0x15C the sum)' % (what, n),
              (qt.readl(ea + 0x158), qt.readl(ea + 0x15C) != 0), (0, True))
        check('%s slice %d: the word at 0x180 untouched' % (what, n),
              qt.readl(ea + 0x180), 0x5A5A0000 | n)
    return out, core, core_bytes


def run(qt, ff, tmp):
    core_init(qt)
    bs = A + 0x50000
    pics = [avpu.picture(), avpu.picture(True), avpu.shifted(4, 2)]
    plan = [[0], [0, 3], [0]]           # the P picture has two slices
    stream = b''
    print('H.264 CABAC, Enc1 + Enc2 as libakv runs them')
    for k, ((y, uv), slices) in enumerate(zip(pics, plan)):
        res, core, core_bytes = picture(qt, k, y, uv, slices)
        what = 'IDR' if k == 0 else 'P%d' % k
        total = 0
        for n, (region, hlen, count, flags) in enumerate(res):
            check('%s slice %d: bytes at 0x104, no overflow (0x100 bit 3)'
                  % (what, n), (count > 0, flags & 8), (True, 0))
            stream += qt.read(bs + region, hlen + count)
            total += count
        check('%s: 0x8304 is the picture\'s bytes (%d)' % (what, total),
              core_bytes, total)
        check('%s: 0x8300 idle, no overflow' % what, core, 0)
    check('the L2 registers read back',
          (qt.readl(R + 0x4004) & 0xC11, qt.readl(R + 0x4014) >> 6 & 0x1FFF,
           qt.readl(R + 0x1208)), (0x011, PITCH, 0xABCDABCD))

    # the T31 layout's JPEG start is JPEG's R9 here: nothing starts
    qt.writel(R + 0x3E4, 1)
    qt.step(1000000)
    check('0x83E4 (R9) starts nothing', qt.readl(R + 0x18), 0)

    print('a stream buffer too small')
    y, uv = avpu.shifted(12, 6)
    res, core, _ = picture(qt, 3, y, uv, [0], room=4)
    check('overflow: 0x8300 bit 1, the entry\'s 0x100 bit 3',
          (core & 2, res[0][3] & 8), (2, 8))
    y, uv = avpu.shifted(12, 6)
    res, core, _ = picture(qt, 4, y, uv, [0])
    check('the next picture that fits clears it', (core & 2, res[0][3] & 8),
          (0, 0))

    print('L2 prefetch addresses that are not the picture\'s')
    picture(qt, 5, y, uv, [0], l2_wrong=True)
    err = open(os.path.join(qt.dir, 'qemu.err')).read()
    check('reported (guest_errors)', 'L2 prefetch is on' in err, True)
    picture(qt, 6, y, uv, [0], l2_wrong=True, l2_bypass=True)
    err = open(os.path.join(qt.dir, 'qemu.err')).read()
    check('not when the L2 is bypassed (0xC004 bits 10, 11)',
          err.count('L2 prefetch is on'), 1)

    print('JPEG')
    a = A + 0x200000
    src, tables, jbs, size, off = a, a + 0x20000, a + 0x30000, 0x40000, 0x800
    y, uv = avpu.picture()
    qt.write(src, y)
    qt.write(src + PITCH * H, uv)
    zz = bytes(avpu.QL[avpu.ZIGZAG[i]] for i in range(64)) + \
        bytes(avpu.QC[avpu.ZIGZAG[i]] for i in range(64))
    qt.write(tables, zz + bytes(400))
    qt.writel(R + 0x14, qt.readl(R + 0x14) | (1 << 18))
    qt.writel(R + 0x4004, (qt.readl(R + 0x4004) & ~0x8000) | 0xE00)
    qt.writel(R + 0x10, 0x10000000)
    regs = [0x130, (H - 1) | ((W - 1) << 16), 0x00100009, PITCH, src,
            src + PITCH * H, tables, jbs, size, off, size - off]
    for i, v in enumerate(regs):
        qt.writel(R + 0x3C0 + 4 * i, v)
    qt.writel(R + 0x84, 0x10)
    st = wait_done(qt, 1 << 18)
    check('JPEG done: bit 18 alone', st, 1 << 18)
    check('module interrupt 2 up', qt.readl(MOD_STATUS) >> 2 & 1, 1)
    qt.writel(R + 0x18, st)
    n = qt.readl(R + 0x3F4)
    check('no overflow (0x83F8 bit 1)', qt.readl(R + 0x3F8) & 2, 0)
    data = qt.read(jbs + off, n)
    check('SOI ... EOI at R9', (data[:2], data[-2:]),
          (b'\xff\xd8', b'\xff\xd9'))

    if not ff:
        print('  SKIP: no ffmpeg to decode')
        return
    path = os.path.join(tmp, 'ak-allegro.h264')
    open(path, 'wb').write(stream)
    rc, raw, errs = avpu.decode(ff, path)
    fs = W * H * 3 // 2
    check('H.264: three pictures decode without errors',
          (rc, len(raw) // fs, errs), (0, 3, []))
    for l in errs[:5]:
        print('    ' + l)
    if len(raw) >= 3 * fs:
        ref = avpu.luma(pics[0][0])
        worst = 0
        for my in range(MBH):
            for mx in range(MBW):
                px = [(my * 16 + j) * W + mx * 16 + x
                      for j in range(16) for x in range(16)]
                worst = max(worst, abs(sum(ref[q] for q in px) -
                                       sum(raw[q] for q in px)) // 256)
        check('IDR macroblock means within 3 of the source (worst %d)' %
              worst, worst <= 3, True)
        for k in (1, 2):
            ref = avpu.luma(pics[k][0])
            p = raw[k * fs:k * fs + W * H]
            e = sum((p[i] - ref[i]) ** 2 for i in range(W * H)) / (W * H)
            psnr = 10 * math.log10(255 * 255 / max(e, 1e-9))
            check('P%d: luma PSNR %.1f dB (>= 30)' % (k, psnr), psnr >= 30,
                  True)
    path = os.path.join(tmp, 'ak-allegro.jpg')
    open(path, 'wb').write(data)
    rc, raw, errs = avpu.decode(ff, path, 'gray')
    check('JPEG decodes without errors', (rc, errs), (0, []))
    if len(raw) == W * H:
        ref = avpu.luma(y)
        mse = sum((a - b) ** 2 for a, b in zip(ref, raw)) / (W * H)
        psnr = 99 if not mse else 10 * math.log10(255 * 255 / mse)
        check('JPEG luma PSNR %.1f dB above 30' % psnr, psnr > 30, True)
    else:
        check('JPEG %dx%d' % (W, H), len(raw), W * H)


def faults(qemu):
    """drop-irqs and the video reset: what OpenIPC's uio_vcodec does when
    a picture never reports back (AKUIO_UNBLOCK_CHANNEL, then a reset of
    the core at the last close: MODULE_RESET_CON1 bit 20 for 5 ms)."""
    print('faults: a lost interrupt, then the video reset')
    qt = avpu.QTest(qemu, 'ak3918ev300',
                    ['-global', 'ingenic-avpu.drop-irqs=on'])
    try:
        core_init(qt)
        y, uv = avpu.picture()
        picture(qt, 0, y, uv, [0], lost_irq=True)
        check('before the reset: the interrupt mask as set',
              qt.readl(R + 0x14), 4)
        qt.writel(SYSCTRL + 0x20, qt.readl(SYSCTRL + 0x20) | 1 << 20)
        qt.step(5000000)
        qt.writel(SYSCTRL + 0x20, qt.readl(SYSCTRL + 0x20) & ~(1 << 20))
        check('video reset (0x08000020 bit 20): the core is back at its '
              'reset state (mask 0, nothing pending, idle)',
              (qt.readl(R + 0x14), qt.readl(R + 0x18),
               qt.readl(R + 0x300) & 5), (0, 0, 0))
        check('and its ID is still there (0x8004)',
              qt.readl(R + 0x04), 0x72000460)
    finally:
        qt.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', 'qemu-src',
        'build', 'qemu-system-arm'))
    ap.add_argument('--ffmpeg', default=None)
    ap.add_argument('--keep', help='directory to leave the streams in')
    args = ap.parse_args()
    tmp = args.keep or tempfile.mkdtemp()
    os.makedirs(tmp, exist_ok=True)
    ff = avpu.ffmpeg_cmd(args.ffmpeg, tmp)
    try:
        qt = avpu.QTest(args.qemu, 'ak3918ev300', ['-d', 'guest_errors'])
        try:
            run(qt, ff, tmp)
        finally:
            qt.close()
        faults(args.qemu)
    finally:
        if not args.keep:
            shutil.rmtree(tmp, ignore_errors=True)
    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
