#!/usr/bin/env python3
"""Boot the stock XM firmware on the gk7206v11a twin, let the App run, then
dump its ISP blocks and export a PQ bin, carried out over the console.

    xm-twin-export.py FLASH TOOLDIR WAIT_S OUTDIR [LOG]

FLASH: the stock dump with xmuart=0 in its U-Boot environment (Xiongmai's
kernels keep the console on only then; qemu-boot/uboot-env.py, env at
0x40000 here). TOOLDIR: hexout.static.gz, attrtool.static.gz,
pqexport.static.gz -- static musl builds, since the stock rootfs is uClibc.

Neither way in is the obvious one: the busybox has no tftp/nc/wget, and an
SD card is claimed by the App's recorder, which reboots on an unknown card
and then formats it. Tools go in as printf octal escapes typed at the
shell, results come out as hex on the console.
"""
import os
import re
import sys
import time

import pexpect

flash, tooldir, wait, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
log = open(sys.argv[5] if len(sys.argv) > 5 else 'twin.log', 'w')
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


def run(cmd, t=300):
    p.sendline(cmd + '; echo ' + MARK + '$?')
    p.expect(MARK + r'(\d+)', timeout=t)
    return p.before


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
print(run('ls -l /var/tmp/*.static; ./hexout.static -s /var/tmp/*.static'))
run('cat /proc/umap/isp > umap_isp.txt; mkdir -p dump; ./attrtool.static dump dump > attr.log 2>&1; ./pqexport.static twin.bin > pq.log 2>&1')
text = run('./hexout.static attr.log pq.log umap_isp.txt twin.bin dump/*.bin', 900)
os.makedirs(out, exist_ok=True)
name, data = None, bytearray()
for line in text.splitlines():
    line = line.strip()
    m = re.match(r'@@BEGIN (\S+)', line)
    if m:
        name, data = m.group(1), bytearray()
        continue
    m = re.match(r'@@END (\S+) (\d+) ([0-9a-f]+)', line)
    if m and name:
        assert len(data) == int(m.group(2)), (name, len(data), m.group(2))
        dst = os.path.join(out, name.replace('/', '__'))
        open(dst, 'wb').write(data)
        print('%s %d bytes' % (name, len(data)))
        name = None
        continue
    if name and line.startswith('@@'):
        data += bytes.fromhex(line[2:])
p.sendcontrol('a')
p.send('x')
p.expect(pexpect.EOF, timeout=30)
