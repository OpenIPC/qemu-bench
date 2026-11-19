#!/usr/bin/env python3
"""Boot the stock XM firmware on the gk7206v11a twin, let the App run, then
dump its ISP blocks and export a PQ bin, carried out over the console.

    xm-twin-export.py FLASH TOOLDIR WAIT_S OUTDIR [LOG]

FLASH: the stock dump. It is booted from a copy (OUTDIR/flash.img) whose
saved U-Boot environment says xmuart=0: Xiongmai's kernels keep the
console on only then. TOOLDIR: build/twin from build.sh with SDK_STATIC set --
static musl builds, since the stock rootfs is uClibc. OUTDIR gets the ISP
blocks as <block>.bin (what xm-isp-finish reads), twin.bin (the PQ export),
umap_isp.txt and the tools' logs.

Neither way in is the obvious one: the busybox has no tftp/nc/wget, and an
SD card is claimed by the App's recorder, which reboots on an unknown card
and then formats it. Tools go in as printf octal escapes typed at the
shell, results come out as hex on the console.
"""
import gzip
import os
import re
import struct
import sys
import time
import zlib

import pexpect

ENV_AT, ENV_SIZE = 0x40000, 0x10000     # this board's saved environment


def with_xmuart0(src, dst):
    """Copy the flash, setting xmuart=0 in its saved environment (CRC32,
    then NUL-separated key=value pairs), every other variable kept."""
    b = bytearray(open(src, 'rb').read())
    env = bytes(b[ENV_AT + 4:ENV_AT + ENV_SIZE])
    if zlib.crc32(env) != struct.unpack_from('<I', b, ENV_AT)[0]:
        sys.exit('%s: no valid U-Boot environment at %#x' % (src, ENV_AT))
    kv = [v for v in env[:env.find(b'\0\0')].split(b'\0') if not v.startswith(b'xmuart=')]
    new = b'\0'.join(kv + [b'xmuart=0']) + b'\0\0'
    new += bytes(ENV_SIZE - 4 - len(new))
    struct.pack_into('<I', b, ENV_AT, zlib.crc32(new))
    b[ENV_AT + 4:ENV_AT + ENV_SIZE] = new
    open(dst, 'wb').write(b)


src, tooldir, wait, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
os.makedirs(out, exist_ok=True)
flash = os.path.join(out, 'flash.img')
with_xmuart0(src, flash)
log = open(sys.argv[5] if len(sys.argv) > 5 else os.path.join(out, 'console.log'), 'w')
qemu = os.environ.get('QEMU', 'qemu-src/build/qemu-system-arm')
args = ['-M', 'gk7206v11a,flash-file=' + flash, '-nographic', '-display', 'none',
        '-serial', 'mon:stdio', '-monitor', 'none',
        '-nic', 'user,restrict=on,net=192.168.1.0/24']
p = pexpect.spawn(qemu, args, encoding='utf-8', codec_errors='replace',
                  timeout=60, logfile=log, maxread=1 << 16)

p.expect('login:', timeout=400)
p.sendline('root')
p.expect('assword:')
p.sendline('Hp#!8CocD_g')
p.expect('# ', timeout=60)
MARK = '__DONE__'


def run(cmd, t=300, check=True):
    """Run cmd at the guest shell; its output, and its status checked."""
    p.sendline(cmd + '; echo ' + MARK + '$?')
    p.expect(MARK + r'(\d+)', timeout=t)
    out, rc = p.before, int(p.match.group(1))
    if check and rc:
        sys.exit('%r exited %d:\n%s' % (cmd, rc, out[-2000:]))
    return out


def rsum(data):
    """hexout's checksum: s = s * 31 + byte, mod 2^32."""
    s = 0
    for c in data:
        s = (s * 31 + c) & 0xffffffff
    return s


def received(text):
    """Files hexout printed, as ({name: bytes, or None for @@MISSING},
    [names whose @@END length or checksum did not match]). The App and the
    kernel log to the same console, so a line of theirs can land in front of
    a hex line or cut one: the hex is looked for anywhere in a line, and
    what was damaged anyway is caught by the checksum."""
    files, damaged, name, data = {}, [], None, bytearray()
    for line in text.splitlines():
        m = re.search(r'@@MISSING (\S+)', line)
        if m:
            files[m.group(1)] = None
            continue
        m = re.search(r'@@BEGIN (\S+)', line)
        if m:
            name, data = m.group(1), bytearray()
            continue
        m = re.search(r'@@END (\S+) (\d+) ([0-9a-f]{8})', line)
        if m:
            ok = m.group(1) == name and len(data) == int(m.group(2)) \
                and rsum(data) == int(m.group(3), 16)
            if not ok:
                print('%s: damaged on the console (%d bytes, sum %08x; '
                      'expected %s %s)' % (m.group(1), len(data), rsum(data),
                                           m.group(2), m.group(3)), flush=True)
            if ok:
                files[name] = bytes(data)
            else:
                damaged.append(m.group(1))
            name = None
            continue
        m = re.search(r'@@([0-9a-f]+)\s*$', line)
        if name and m and len(m.group(1)) % 2 == 0:
            data += bytes.fromhex(m.group(1))
    if name:
        damaged.append(name)        # its @@END never arrived
    return files, damaged


def fetch(names, tries=4):
    """hexout the guest files, again for any that arrive damaged."""
    got, want = {}, list(names)
    for _ in range(tries):
        files, damaged = received(run('./hexout.static ' + ' '.join(want), 900))
        got.update(files)
        # a name with neither a good copy nor an answer was lost whole
        want = [n for n in want if n in damaged or n not in files]
        if not want:
            return got
    sys.exit('still damaged after %d tries: %s' % (tries, ' '.join(want)))


print('logged in; waiting for the ISP', flush=True)
p.expect(r'isp_init\[\d+\]: isp\[0\] init success', timeout=600)
print('ISP up; App running for %d s' % wait, flush=True)
time.sleep(wait)
p.sendline('')
run('cd /var/tmp')


def send(local, remote):
    """printf octal escapes, 200 bytes a line, one line at a time: the
    tty holds 4 KiB of input, and a burst of lines overflows it."""
    data = open(local, 'rb').read()
    run(': > ' + remote)
    for i in range(0, len(data), 200):
        run("printf '%s' >> %s" % (''.join('\\%03o' % c for c in data[i:i + 200]), remote))
    print('sent', local, len(data), flush=True)


p.setecho(False)
for t in ('hexout.static', 'attrtool.static', 'pqexport.static'):
    send(os.path.join(tooldir, t + '.gz'), t + '.gz')
    run('gunzip -f %s.gz && chmod +x %s' % (t, t))
    want = gzip.decompress(open(os.path.join(tooldir, t + '.gz'), 'rb').read())
    got = run('./hexout.static -s ' + t)
    if '@@END %s %d %08x' % (t, len(want), rsum(want)) not in got:
        sys.exit('%s arrived damaged:\n%s' % (t, got))

# attrtool's exit status is not checked: the App's own AE and AWB are not
# registered in this process, so their getters fail (and one may crash);
# the ISP blocks before them are what is wanted, and its log says which.
run('cat /proc/umap/isp > umap_isp.txt; rm -rf dump; '
    './attrtool.static dump dump > attr.log 2>&1', check=False)
run('./pqexport.static twin.bin > pq.log 2>&1')
# a shell loop, not ls: this busybox colours ls's names even off a tty
names = re.findall(r'@@NAME (dump/\w+\.bin)',
                   run('for f in dump/*.bin; do echo "@@NAME $f"; done'))
files = fetch(['attr.log', 'pq.log', 'umap_isp.txt', 'twin.bin'] + names)
missing = [n for n in ('twin.bin', 'dump/bnr.bin', 'dump/stnr.bin', 'dump/sharpen.bin')
           if files.get(n) is None]
if missing:
    sys.exit('not exported: ' + ' '.join(missing))
os.makedirs(out, exist_ok=True)
for name, data in files.items():
    if data is None or name == 'flash.img':
        continue
    # flat, as xm-isp-finish reads <dir>/<block>.bin
    open(os.path.join(out, os.path.basename(name)), 'wb').write(data)
    print('%s %d bytes' % (name, len(data)))
p.sendcontrol('a')
p.send('x')
p.expect(pexpect.EOF, timeout=30)
