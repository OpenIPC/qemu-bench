#!/usr/bin/env python3
"""Run shell commands on a vendor Ingenic SDK image over its telnetd.

The SDK root (busybox telnetd from rcS, root without a password) is
reached through QEMU's user network and a hostfwd, so this also proves
TCP through the emulated GMAC both ways.  A minimal telnet client (the
stdlib one is gone from Python 3.13): refuse every option, log in as
root, send each command and collect its output up to a marker.

    ingenic-telnet.py PORT [--timeout S] [--host H] -- CMD [CMD...]

Prints "### <cmd>" and the command's output for each command; exit 0 when
every command came back.  A command that does not come back (reboot)
ends the session and is not an error if it is the last.

Copyright (c) 2026 OpenIPC.
SPDX-License-Identifier: GPL-2.0-or-later
"""
import socket
import sys
import time

IAC, DONT, DO, WONT, WILL, SB, SE = 255, 254, 253, 252, 251, 250, 240


class Telnet:
    def __init__(self, host, port, timeout):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.settimeout(1)
        self.buf = b''

    def _recv(self):
        try:
            d = self.s.recv(4096)
        except socket.timeout:
            return True
        if not d:
            return False
        out, i = bytearray(), 0
        while i < len(d):
            c = d[i]
            if c == IAC and i + 1 < len(d):
                op = d[i + 1]
                if op in (DO, DONT, WILL, WONT) and i + 2 < len(d):
                    reply = WONT if op in (DO, DONT) else DONT
                    self.s.sendall(bytes([IAC, reply, d[i + 2]]))
                    i += 3
                    continue
                if op == SB:
                    j = d.find(bytes([IAC, SE]), i)
                    i = len(d) if j < 0 else j + 2
                    continue
                i += 2
                continue
            out.append(c)
            i += 1
        self.buf += bytes(out)
        return True

    def expect(self, token, timeout):
        end = time.time() + timeout
        while time.time() < end:
            k = self.buf.find(token)
            if k >= 0:
                got, self.buf = self.buf[:k], self.buf[k + len(token):]
                return got
            if not self._recv():
                return None
        return None

    def send(self, line):
        self.s.sendall(line.encode() + b'\r\n')


def main():
    args = sys.argv[1:]
    host, timeout = '127.0.0.1', 30
    port = int(args.pop(0))
    while args and args[0] != '--':
        if args[0] == '--timeout':
            timeout = float(args[1])
            args = args[2:]
        elif args[0] == '--host':
            host = args[1]
            args = args[2:]
        else:
            sys.exit('unknown option ' + args[0])
    cmds = args[1:]
    t = Telnet(host, port, timeout)
    if t.expect(b'login:', timeout) is None:
        sys.exit('telnet: no login prompt')
    t.send('root')
    # no password on the SDK root; a prompt ends in "# "
    if t.expect(b'# ', timeout) is None:
        sys.exit('telnet: no shell prompt')
    for n, cmd in enumerate(cmds):
        mark = 'TNEND%d' % n
        # the markers are split so that the echo of the line (which the
        # shell wraps at 80 columns) does not match them
        t.send('echo TNB""EG%d; %s; echo TNE""ND%d' % (n, cmd, n))
        print('### ' + cmd)
        if t.expect(b'TNBEG%d\r\n' % n, timeout) is None:
            sys.exit('telnet: no reply to ' + cmd)
        got = t.expect(mark.encode() + b'\r\n', timeout)
        if got is None:
            if n == len(cmds) - 1:
                print('(no reply: session ended)')
                return 0
            sys.exit('telnet: no end of ' + cmd)
        print(got.decode('latin-1').replace('\r', ''), end='')
    return 0


if __name__ == '__main__':
    sys.exit(main())
