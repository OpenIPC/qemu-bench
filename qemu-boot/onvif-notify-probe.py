#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Subscribe to a camera's ONVIF event service and dump the wsnt:Notify it
pushes — a self-contained WS-BaseNotification consumer, stdlib only.

This reproduces the client half of OpenIPC/majestic#379: an NVR subscribes
with a ConsumerReference and the camera POSTs Notify to it. Point it at a
camera (real or an emulated twin) to capture exactly what majestic sends and
to A/B a fix.

    python3 onvif-notify-probe.py --host 192.168.50.148 \
        --user admin --password 123456 --seconds 20 --out /tmp/notify

HTTP Digest authentication is used (what majestic accepts once onvif.password
is set; it sidesteps the clock-skew sensitivity of WSSE PasswordDigest on a
freshly booted twin with no RTC). No third-party packages — standard library
only.
"""
import argparse, os, secrets, socket, sys, threading, time
import urllib.request
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DEV = "/onvif/device_service"
EVT = "/onvif/event_service"
WSNT = "http://docs.oasis-open.org/wsn/b-2"

MAX_BODY = 1 << 20      # a Notify is a few KiB; nothing legitimate nears this
READ_TIMEOUT_S = 5.0    # a stalled sender must not pin a handler thread


def opener_for(host, port, user, password):
    handlers = [urllib.request.ProxyHandler({})]  # lab LAN: never via a proxy
    if user:
        mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        # Scope the credential to the exact authority soap() requests, port
        # included, or Digest lookup misses when --port is not 80.
        mgr.add_password(None, f"http://{host}:{port}/", user, password)
        handlers.append(urllib.request.HTTPDigestAuthHandler(mgr))
    return urllib.request.build_opener(*handlers)


def soap(opener, host, port, path, body, action):
    env = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<SOAP-ENV:Envelope xmlns:SOAP-ENV="http://www.w3.org/2003/05/soap-envelope"'
        ' xmlns:wsa="http://www.w3.org/2005/08/addressing"'
        ' xmlns:wsnt="http://docs.oasis-open.org/wsn/b-2"'
        ' xmlns:tev="http://www.onvif.org/ver10/events/wsdl">'
        f"<SOAP-ENV:Body>{body}</SOAP-ENV:Body></SOAP-ENV:Envelope>"
    )
    req = urllib.request.Request(
        f"http://{host}:{port}{path}", data=env.encode(),
        headers={"Content-Type": f'application/soap+xml; charset=utf-8; action="{action}"'})
    with opener.open(req, timeout=10) as r:
        return r.read()


def local_ip_towards(host):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((socket.gethostbyname(host), 80))
        return s.getsockname()[0]
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--user", default="")
    ap.add_argument("--password", default="")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    bodies, lock = [], threading.Lock()
    dut_ip = socket.gethostbyname(a.host)
    path = "/notify/" + secrets.token_urlsafe(12)

    class H(BaseHTTPRequestHandler):
        timeout = READ_TIMEOUT_S

        def _reply(self, code):
            self.send_response(code)
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_POST(self):
            # The listener binds a reachable interface, so trust nothing:
            # reject an unexpected sender or path before reading the body,
            # and cap the length, so a stranger cannot pin a handler thread.
            if self.client_address[0] != dut_ip or self.path != path:
                self._reply(404)
                return
            try:
                n = int(self.headers.get("Content-Length", "0") or "0")
            except ValueError:
                n = -1
            if n < 0 or n > MAX_BODY:
                self._reply(413 if n > MAX_BODY else 411)
                return
            data = self.rfile.read(n) if n else b""
            if len(data) != n:
                return  # sender went away mid-body
            with lock:
                bodies.append(data)
            self._reply(202)

        def log_message(self, *_):
            pass

    host_ip = local_ip_towards(a.host)
    srv = ThreadingHTTPServer((host_ip, 0), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    consumer = f"http://{host_ip}:{srv.server_port}{path}"
    print(f"consumer listening at {consumer}", file=sys.stderr)

    opener = opener_for(a.host, a.port, a.user, a.password)
    sub_body = (
        "<wsnt:Subscribe><wsnt:ConsumerReference>"
        f"<wsa:Address>{consumer}</wsa:Address>"
        "</wsnt:ConsumerReference>"
        "<wsnt:InitialTerminationTime>PT300S</wsnt:InitialTerminationTime>"
        "</wsnt:Subscribe>")
    resp = soap(opener, a.host, a.port, EVT, sub_body,
                "http://docs.oasis-open.org/wsn/bw-2/NotificationProducer/Subscribe")
    print("Subscribe accepted" if b"SubscribeResponse" in resp
          else "Subscribe FAILED:\n" + resp.decode(errors="replace")[:800],
          file=sys.stderr)

    deadline = time.time() + a.seconds
    while time.time() < deadline:
        time.sleep(0.5)
    srv.shutdown()

    with lock:
        caught = list(bodies)
    print(f"\n{len(caught)} Notify POST(s) received")
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        for i, b in enumerate(caught):
            open(os.path.join(a.out, f"notify-{i:03d}.xml"), "wb").write(b)
        print(f"bodies written to {a.out}")
    for i, b in enumerate(caught):
        try:
            root = ET.fromstring(b)
            msgs = root.iter(f"{{{WSNT}}}NotificationMessage")
            n = sum(1 for _ in msgs)
            changed = any(
                m.get("PropertyOperation") == "Changed"
                for m in root.iter("{http://www.onvif.org/ver10/schema}Message"))
        except ET.ParseError:
            n, changed = -1, False
        label = f"{n} NotificationMessage(s)" if n >= 0 else "unparseable body"
        print(f"  POST {i}: {label}{'  [has Changed]' if changed else ''}")


if __name__ == "__main__":
    main()
