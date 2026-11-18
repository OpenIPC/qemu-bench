#!/usr/bin/env python3
"""Minimal RTSP-over-TCP puller: OPTIONS, DESCRIBE, SETUP video, PLAY, then
depacketise H.265 (RFC 7798: single NAL, FU) from interleaved channel 0
into an Annex-B file. Prints every RTSP response and RTP statistics.

usage: rtsp-pull.py <host> <port> <path> <seconds> <out.h265>
"""
import socket, sys, time, re, base64, struct

host, port, path, secs, out = sys.argv[1], int(sys.argv[2]), sys.argv[3], float(sys.argv[4]), sys.argv[5]
url = f"rtsp://192.168.1.10:554/{path}"
s = socket.create_connection((host, port), timeout=60)
buf = b''
cseq = 0

def req(method, u, extra=''):
    global cseq, buf
    cseq += 1
    s.sendall(f"{method} {u} RTSP/1.0\r\nCSeq: {cseq}\r\nUser-Agent: pull\r\n{extra}\r\n".encode())
    while b'\r\n\r\n' not in buf:
        d = s.recv(65536)
        if not d:
            raise SystemExit('closed')
        buf += d
    hdr, buf = buf.split(b'\r\n\r\n', 1)
    m = re.search(rb'Content-Length:\s*(\d+)', hdr, re.I)
    body = b''
    if m:
        n = int(m.group(1))
        while len(buf) < n:
            buf += s.recv(65536)
        body, buf = buf[:n], buf[n:]
    print(f"--- {method}\n{hdr.decode(errors='replace')}\n")
    return hdr.decode(errors='replace'), body.decode(errors='replace')

req('OPTIONS', url)
_, sdp = req('DESCRIBE', url, 'Accept: application/sdp\r\n')
ps = []
for k in ('sprop-vps', 'sprop-sps', 'sprop-pps'):
    m = re.search(k + r'=([^;\s]+)', sdp)
    if m:
        ps.append(base64.b64decode(m.group(1)))
h, _ = req('SETUP', url + '/trackID=3', 'Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n')
sess = re.search(r'Session:\s*([^;\r\n]+)', h)
sess = sess.group(1).strip() if sess else ''
req('PLAY', url, f'Session: {sess}\r\nRange: npt=0.000-\r\n')

f = open(out, 'wb')
for p in ps:
    f.write(b'\x00\x00\x00\x01' + p)
pkts = byts = nals = 0
fu = None
t0 = time.time()
s.settimeout(5)
while time.time() - t0 < secs:
    try:
        d = s.recv(65536)
    except socket.timeout:
        continue
    if not d:
        break
    buf += d
    while len(buf) >= 4 and buf[0] == 0x24:
        ch, ln = buf[1], struct.unpack('>H', buf[2:4])[0]
        if len(buf) < 4 + ln:
            break
        pkt, buf = buf[4:4 + ln], buf[4 + ln:]
        if ch != 0 or len(pkt) < 14:
            continue
        pkts += 1
        byts += len(pkt)
        cc = pkt[0] & 0xf
        pl = pkt[12 + 4 * cc:]
        ntype = (pl[0] >> 1) & 0x3f
        if ntype == 49:                      # fragmentation unit
            fuh = pl[2]
            t = fuh & 0x3f
            if fuh & 0x80:
                fu = bytes([(pl[0] & 0x81) | (t << 1), pl[1]]) + pl[3:]
            elif fu is not None:
                fu += pl[3:]
            if fuh & 0x40 and fu is not None:
                f.write(b'\x00\x00\x00\x01' + fu); nals += 1; fu = None
        else:
            f.write(b'\x00\x00\x00\x01' + pl); nals += 1
    if buf and buf[0] != 0x24:              # stray RTSP reply/keepalive
        i = buf.find(b'\x24')
        buf = buf[i:] if i >= 0 else b''
f.close()
print(f"RTP packets={pkts} bytes={byts} nals={nals} in {time.time()-t0:.1f}s -> {out}")
