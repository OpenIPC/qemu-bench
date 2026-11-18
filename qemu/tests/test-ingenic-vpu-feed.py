#!/usr/bin/env python3
"""libimp's own encoders on the ingenic-vpu model, per SoC: OpenIPC's kernel
and rootfs (T20, T21, T23, T30), qemu/tests/ingenic-vpu-feed.c in the rootfs
feeding i264e_* (H.264, fixed QP and CBR), ijpege_* (JPEG) and on T30
i265e_* (H.265) with pictures from memory -- no ISP needed.  The streams
come back over the console (base64) and ffmpeg decodes them on the host
(-err_detect explode); their pictures are compared with what was fed.

    python3 qemu/tests/test-ingenic-vpu-feed.py [--soc t21 ...] [--fw DIR]
            [--qemu PATH] [--frames N] [--keep DIR]

--fw holds <soc>/uImage.<soc> and <soc>/rootfs.squashfs.<soc>.  Needs the
Ingenic uClibc toolchain, unsquashfs/mksquashfs, pexpect and ffmpeg (PATH
or FFMPEG=...).
"""
import argparse, base64, os, shutil, subprocess, sys, tempfile, time
import pexpect

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
tv = import_module('test-ingenic-vpu')

HERE = os.path.dirname(os.path.abspath(__file__))
TC = '/home/ai/toolchains/ingenic/mips-gcc540-uclibc0.9.33.2-64bit-r3.3.0.smaller/bin/mips-linux-uclibc-'


def build(soc, fw, work):
    """the feed program against the rootfs's libimp and musl, in a copy of
    the rootfs"""
    rfs = os.path.join(work, 'rfs-' + soc)
    subprocess.run(['unsquashfs', '-q', '-d', rfs,
                    os.path.join(fw, soc, 'rootfs.squashfs.' + soc)],
                   check=True, stdout=subprocess.DEVNULL)
    subprocess.run([TC + 'gcc', '-O2', '-fno-strict-aliasing', '-Wall', '-o',
                    os.path.join(rfs, 'usr/bin/vpu-feed'),
                    os.path.join(HERE, 'ingenic-vpu-feed.c'), '-nodefaultlibs',
                    '-Wl,-dynamic-linker,/lib/ld-musl-mipsel.so.1',
                    '-Wl,--allow-shlib-undefined',
                    '-L' + os.path.join(rfs, 'usr/lib'),
                    '-L' + os.path.join(rfs, 'lib'),
                    '-l:libimp.so', '-l:libc.so', '-lgcc'], check=True)
    rd = os.path.join(work, 'rd-%s.sqfs' % soc)
    subprocess.run(['mksquashfs', rfs, rd, '-comp', 'xz', '-all-root',
                    '-noappend', '-quiet'], check=True,
                   stdout=subprocess.DEVNULL)
    shutil.rmtree(rfs)
    return rd


def run(qemu, soc, fw, rd, cmd, log):
    """boot with init=/bin/sh, run cmd, fetch /tmp/out.bin"""
    p = pexpect.spawn(qemu, [
        '-M', soc, '-m', '64M', '-nographic', '-nic', 'user,restrict=on',
        '-kernel', os.path.join(fw, soc, 'uImage.' + soc), '-initrd', rd,
        '-append', 'console=ttyS1,115200n8 root=/dev/ram0 init=/bin/sh '
        'mem=40M rmem=24M@0x2800000'], encoding='latin-1', timeout=300)
    p.logfile_read = open(log, 'a')
    try:
        p.expect('# ')
        time.sleep(2)
        for c in ['true', 'mount -t proc proc /proc; mount -t sysfs sys /sys;'
                  ' mount -t tmpfs tmp /tmp', 'touch /tmp/closesimd', cmd]:
            p.sendline(c)
            p.expect('# ', timeout=600)
        p.sendline('echo BEGIN""64; base64 /tmp/out.bin; echo END""64')
        p.expect('BEGIN64\r+\n', timeout=60)
        p.expect('END64', timeout=600)
        body = p.before.replace('\r', '')
        return base64.b64decode(''.join(l.strip() for l in body.splitlines()
                                        if l.strip()))
    except (pexpect.TIMEOUT, pexpect.EOF, ValueError):
        return b''
    finally:
        p.terminate(force=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--soc', nargs='*', default=['t20', 't21', 't23', 't30'])
    ap.add_argument('--fw', default='/home/ai/dimerr/local/ingenic/fw')
    ap.add_argument('--qemu', default=os.path.join(HERE, '..', '..', 'qemu-src',
                                                   'build', 'qemu-system-mipsel'))
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--keep')
    a = ap.parse_args()
    ffmpeg = os.environ.get('FFMPEG') or shutil.which('ffmpeg')
    work = tempfile.mkdtemp()
    out = a.keep or work
    os.makedirs(out, exist_ok=True)
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    w, h, n = 640, 360, a.frames
    for soc in a.soc:
        print('== %s' % soc)
        rd = build(soc, a.fw, work)
        log = os.path.join(out, '%s-console.log' % soc)
        jobs = [('h264', 'FEED_LIB=%s vpu-feed /tmp/out.bin %d' % (soc, n), n,
                 'yuv420p', 45),
                ('jpg', 'FEED_LIB=%s FEED_JPEG=1 vpu-feed /tmp/out.bin 1' % soc,
                 1, 'yuvj420p', 45)]
        if soc != 't20':        # the T20 library's CBR fields are not mapped
            jobs.insert(1, ('cbr.h264', 'FEED_LIB=%s FEED_CBR=600 vpu-feed '
                            '/tmp/out.bin %d' % (soc, n), n, 'yuv420p', 40))
        if soc == 't30':
            jobs.append(('h265', 'FEED_LIB=t30 FEED_H265=1 vpu-feed '
                         '/tmp/out.bin %d' % n, n, 'yuv420p', 25))
        for ext, cmd, nf, pix, need in jobs:
            data = run(a.qemu, soc, a.fw, rd, cmd, log)
            name = '%s.%s' % (soc, ext)
            check(len(data) > 0, '%s: %d bytes from libimp' % (name, len(data)))
            if not data or not ffmpeg:
                continue
            path = os.path.join(out, name)
            open(path, 'wb').write(data)
            raw, err = tv.decode(ffmpeg, path, pix)
            check(not err, 'ffmpeg decodes %s cleanly%s' % (
                name, ': ' + err[-300:] if err else ''))
            fs = w * h * 3 // 2
            check(len(raw) == nf * fs, '%d of %d pictures' % (len(raw) // fs, nf))
            worst = min([tv.psnr_y(raw[f * fs:(f + 1) * fs], w, h, f)
                         for f in range(min(nf, len(raw) // fs))] or [0])
            check(worst >= need, '%s: Y PSNR >= %.1f dB against the fed '
                  'pictures (>= %d)' % (name, worst, need))
    shutil.rmtree(work, ignore_errors=True)
    print('PASS' if not fails else 'FAIL (%d)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
