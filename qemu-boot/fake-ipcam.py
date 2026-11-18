#!/usr/bin/env python3
"""
A fake IP camera for emulated NVRs: ONVIF (Profile S subset) + RTSP with
RTP interleaved over TCP, serving a synthetic H.264 test pattern, or
H.264 / H.265 files.

The H.264 stream needs no encoder library.  Every picture is lossless:
the IDR codes each macroblock as I_PCM, and the P pictures that follow
code the macroblocks that changed as I_PCM and skip the rest (P_Skip with
every motion vector zero, deblocking off), so what the NVR decodes is
exactly the pattern drawn here: colour bars, a moving box and a frame
counter.

ONVIF offers two profiles, the main stream (Profile_1) and the sub
stream (Profile_2), and GetSnapshotUri's /snapshot.jpg is the main
stream's current picture as a baseline JPEG, encoded here too.

Usage (the NVR at 192.168.1.9, the camera at 192.168.1.10):

    python3 qemu-boot/fake-ipcam.py --http 18080 --rtsp 18554 \
        --advertise 192.168.1.10 &
    qemu-system-arm ... -nic user,net=192.168.1.0/24,host=192.168.1.1,\
guestfwd=tcp:192.168.1.10:80-cmd:"fake-ipcam.py --relay 18080",\
guestfwd=tcp:192.168.1.10:554-cmd:"fake-ipcam.py --relay 18554"

A guestfwd to a tcp: chardev is a single host connection that every guest
connection would share; the cmd: form starts one --relay process (stdio to
the local port) per guest connection.

    python3 qemu-boot/fake-ipcam.py --write out.h264 --frames 50

--file / --sub-file loop raw Annex B files instead, H.265 when they end
in .hevc / .h265 (e.g. from kvazaar; see docs/novatek.md for what the
NT98332 needs of them).

Copyright (c) 2026 OpenIPC.
SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import base64
import datetime
import math
import os
import random
import select
import socket
import socketserver
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------
# H.264 bitstream


class BitWriter:
    def __init__(self):
        self.out = bytearray()
        self.acc = 0
        self.n = 0

    def u(self, bits, v):
        for i in range(bits - 1, -1, -1):
            self.acc = (self.acc << 1) | ((v >> i) & 1)
            self.n += 1
            if self.n == 8:
                self.out.append(self.acc)
                self.acc = 0
                self.n = 0

    def ue(self, v):
        v += 1
        length = v.bit_length()
        self.u(length - 1, 0)
        self.u(length, v)

    def se(self, v):
        self.ue(2 * v - 1 if v > 0 else -2 * v)

    def align_zero(self):
        if self.n:
            self.u(8 - self.n, 0)

    def raw(self, data):
        assert self.n == 0
        self.out += data

    def trailing(self):
        self.u(1, 1)
        self.align_zero()
        return bytes(self.out)


def nal(ref_idc, nal_type, rbsp):
    """NAL unit with start code and emulation prevention."""
    out = bytearray(b"\x00\x00\x00\x01")
    out.append((ref_idc << 5) | nal_type)
    zeros = 0
    for b in rbsp:
        if zeros >= 2 and b <= 3:
            out.append(3)
            zeros = 0
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
    return bytes(out)


class H264Pattern:
    """Synthetic lossless H.264 Baseline stream."""

    def __init__(self, width=640, height=360, fps=10, gop=20):
        self.w, self.h = width, height
        self.mbw = (width + 15) // 16
        self.mbh = (height + 15) // 16
        self.fps, self.gop = fps, gop
        self.n = 0
        self.prev = None
        self.idr_id = 0

    def sps(self):
        b = BitWriter()
        b.u(8, 66)                  # profile_idc: Baseline
        b.u(8, 0xC0)                # constraint_set0/1
        b.u(8, 31)                  # level 3.1
        b.ue(0)                     # seq_parameter_set_id
        b.ue(0)                     # log2_max_frame_num_minus4
        b.ue(2)                     # pic_order_cnt_type
        b.ue(1)                     # max_num_ref_frames
        b.u(1, 0)                   # gaps_in_frame_num_allowed
        b.ue(self.mbw - 1)
        b.ue(self.mbh - 1)
        b.u(1, 1)                   # frame_mbs_only
        b.u(1, 1)                   # direct_8x8_inference
        crop_r = (self.mbw * 16 - self.w) // 2
        crop_b = (self.mbh * 16 - self.h) // 2
        if crop_r or crop_b:
            b.u(1, 1)
            b.ue(0)
            b.ue(crop_r)
            b.ue(0)
            b.ue(crop_b)
        else:
            b.u(1, 0)
        b.u(1, 1)                   # vui_parameters_present
        b.u(1, 0)                   # aspect_ratio_info
        b.u(1, 0)                   # overscan_info
        b.u(1, 0)                   # video_signal_type
        b.u(1, 0)                   # chroma_loc_info
        b.u(1, 1)                   # timing_info_present
        b.u(32, 1)                  # num_units_in_tick
        b.u(32, 2 * self.fps)       # time_scale
        b.u(1, 1)                   # fixed_frame_rate
        b.u(1, 0)                   # nal_hrd
        b.u(1, 0)                   # vcl_hrd
        b.u(1, 0)                   # pic_struct_present
        b.u(1, 1)                   # bitstream_restriction
        b.u(1, 1)                   # motion_vectors_over_pic_boundaries
        b.ue(0)                     # max_bytes_per_pic_denom
        b.ue(0)                     # max_bits_per_mb_denom
        b.ue(16)                    # log2_max_mv_length_horizontal
        b.ue(16)                    # log2_max_mv_length_vertical
        b.ue(0)                     # max_num_reorder_frames
        b.ue(1)                     # max_dec_frame_buffering
        return nal(3, 7, b.trailing())

    def pps(self):
        b = BitWriter()
        b.ue(0)                     # pic_parameter_set_id
        b.ue(0)                     # seq_parameter_set_id
        b.u(1, 0)                   # entropy_coding_mode: CAVLC
        b.u(1, 0)                   # bottom_field_pic_order_in_frame
        b.ue(0)                     # num_slice_groups_minus1
        b.ue(0)                     # num_ref_idx_l0_default_minus1
        b.ue(0)                     # num_ref_idx_l1_default_minus1
        b.u(1, 0)                   # weighted_pred
        b.u(2, 0)                   # weighted_bipred_idc
        b.se(0)                     # pic_init_qp_minus26
        b.se(0)                     # pic_init_qs_minus26
        b.se(0)                     # chroma_qp_index_offset
        b.u(1, 1)                   # deblocking_filter_control_present
        b.u(1, 0)                   # constrained_intra_pred
        b.u(1, 0)                   # redundant_pic_cnt_present
        return nal(3, 8, b.trailing())

    # -- the picture ------------------------------------------------------

    BARS = [(235, 128, 128), (210, 16, 146), (170, 166, 16), (145, 54, 34),
            (106, 202, 222), (81, 90, 240), (41, 240, 110), (16, 128, 128)]

    FONT = {
        "0": "111101101101111", "1": "010110010010111",
        "2": "111001111100111", "3": "111001111001111",
        "4": "101101111001001", "5": "111100111001111",
        "6": "111100111101111", "7": "111001001001001",
        "8": "111101111101111", "9": "111101111001111",
        ":": "000010000010000", " ": "000000000000000",
    }

    def draw(self, n):
        """YUV 4:2:0 planes (bytearrays) of picture n."""
        W, H = self.mbw * 16, self.mbh * 16
        y = bytearray(W * H)
        u = bytearray(W * H // 4)
        v = bytearray(W * H // 4)
        bw = max(1, self.w // len(self.BARS))
        rowy = bytearray(W)
        rowu = bytearray(W // 2)
        rowv = bytearray(W // 2)
        for x in range(W):
            c = self.BARS[min(x // bw, len(self.BARS) - 1)]
            rowy[x] = c[0]
            if not x & 1:
                rowu[x // 2] = c[1]
                rowv[x // 2] = c[2]
        for r in range(H):
            y[r * W:(r + 1) * W] = rowy
        for r in range(H // 2):
            u[r * W // 2:(r + 1) * W // 2] = rowu
            v[r * W // 2:(r + 1) * W // 2] = rowv

        def rect(x0, y0, rw, rh, c):
            x0, y0 = max(0, x0), max(0, y0)
            x1, y1 = min(W, x0 + rw), min(H, y0 + rh)
            for r in range(y0, y1):
                y[r * W + x0:r * W + x1] = bytes([c[0]]) * (x1 - x0)
            for r in range(y0 // 2, y1 // 2):
                o = r * W // 2
                u[o + x0 // 2:o + x1 // 2] = bytes([c[1]]) * (x1 // 2 - x0 // 2)
                v[o + x0 // 2:o + x1 // 2] = bytes([c[2]]) * (x1 // 2 - x0 // 2)

        # a box bouncing across the lower third
        span = max(1, self.w - 64)
        t = (n * 8) % (2 * span)
        bx = t if t < span else 2 * span - t
        # the areas that may differ from the previous picture
        self.dirty = [(bx - 8, self.h * 2 // 3, 80, 48),
                      (8, 8, 8 + len("%06d %s" % (n, "00:00:00")) * 16, 36)]
        rect(bx, self.h * 2 // 3, 64, 48, (81, 90, 240))
        rect(bx + 16, self.h * 2 // 3 + 12, 32, 24, (235, 128, 128))
        # the frame counter and wall-clock time, top left
        text = "%06d %s" % (n, time.strftime("%H:%M:%S"))
        rect(8, 8, 8 + len(text) * 16, 36, (16, 128, 128))
        for i, ch in enumerate(text):
            g = self.FONT.get(ch, self.FONT[" "])
            for gy in range(5):
                for gx in range(3):
                    if g[gy * 3 + gx] == "1":
                        rect(16 + i * 16 + gx * 4, 14 + gy * 5, 4, 5,
                             (235, 128, 128))
        return y, u, v

    def mb_pcm(self, planes, mx, my):
        y, u, v = planes
        W = self.mbw * 16
        out = bytearray()
        for r in range(16):
            o = (my * 16 + r) * W + mx * 16
            out += y[o:o + 16]
        for p in (u, v):
            for r in range(8):
                o = (my * 8 + r) * (W // 2) + mx * 8
                out += p[o:o + 8]
        # pcm samples of 0 are not allowed by the first edition of H.264
        return bytes(max(1, s) for s in out)

    def vertical(self, planes, mx, my):
        """Does the macroblock repeat the sample row just above it?"""
        y, u, v = planes
        W = self.mbw * 16
        above = y[(my * 16 - 1) * W + mx * 16:(my * 16 - 1) * W + mx * 16 + 16]
        for r in range(16):
            o = (my * 16 + r) * W + mx * 16
            if y[o:o + 16] != above:
                return False
        for p in (u, v):
            o = (my * 8 - 1) * (W // 2) + mx * 8
            above = p[o:o + 8]
            for r in range(8):
                o = (my * 8 + r) * (W // 2) + mx * 8
                if p[o:o + 8] != above:
                    return False
        return True

    def picture(self):
        """The next access unit (bytes) and whether it is an IDR."""
        planes = self.draw(self.n)
        self.last = planes                      # for the snapshot
        idr = self.n % self.gop == 0 or self.prev is None
        b = BitWriter()
        b.ue(0)                                 # first_mb_in_slice
        b.ue(7 if idr else 5)                   # slice_type, all I / P
        b.ue(0)                                 # pic_parameter_set_id
        b.u(4, (self.n % self.gop) % 16)        # frame_num
        if idr:
            b.ue(self.idr_id)                   # idr_pic_id
            self.idr_id = (self.idr_id + 1) % 16
            b.u(1, 0)                           # no_output_of_prior_pics
            b.u(1, 0)                           # long_term_reference_flag
        else:
            b.u(1, 0)                           # num_ref_idx_active_override
            b.u(1, 0)                           # ref_pic_list_modification
            b.u(1, 0)                           # adaptive_ref_pic_marking
        b.se(0)                                 # slice_qp_delta
        b.ue(1)                                 # disable_deblocking_filter
        skip = 0
        kind = {}                   # macroblock -> total_coeff, for nC
        maybe = None
        if not idr:
            # only the macroblocks the moving parts touch can differ
            maybe = set()
            for x, y, w, h in self.dirty:
                for my in range(max(0, y // 16), min(self.mbh,
                                                     (y + h + 15) // 16)):
                    for mx in range(max(0, x // 16),
                                    min(self.mbw, (x + w + 15) // 16)):
                        maybe.add((mx, my))
        for my in range(self.mbh):
            for mx in range(self.mbw):
                if maybe is not None and (mx, my) not in maybe:
                    skip += 1
                    kind[mx, my] = 0
                    continue
                pcm = self.mb_pcm(planes, mx, my)
                if not idr and pcm == self.prev[my * self.mbw + mx]:
                    skip += 1
                    kind[mx, my] = 0
                    continue
                if not idr:
                    b.ue(skip)                  # mb_skip_run
                    skip = 0
                if my and self.vertical(planes, mx, my):
                    # I_16x16, vertical prediction, no residual: an exact
                    # copy of the row above, a few bits instead of 384 bytes
                    b.ue(1 if idr else 6)       # mb_type I_16x16_0_0_0
                    b.ue(2)                     # intra_chroma_pred: vertical
                    b.se(0)                     # mb_qp_delta
                    na = kind.get((mx - 1, my))
                    nb = kind[mx, my - 1]
                    nc = nb if na is None else (na + nb + 1) >> 1
                    # Intra16x16DCLevel coeff_token, TotalCoeff 0
                    if nc < 2:
                        b.u(1, 1)
                    elif nc < 4:
                        b.u(2, 3)
                    elif nc < 8:
                        b.u(4, 15)
                    else:
                        b.u(6, 3)
                    kind[mx, my] = 0
                else:
                    b.ue(25 if idr else 30)     # mb_type I_PCM
                    b.align_zero()
                    b.raw(pcm)
                    kind[mx, my] = 16
                if self.prev is None:
                    self.prev = [None] * (self.mbw * self.mbh)
                self.prev[my * self.mbw + mx] = pcm
        if skip:
            b.ue(skip)
        au = bytearray()
        if idr:
            au += self.sps() + self.pps()
        au += nal(3 if idr else 2, 5 if idr else 1, b.trailing())
        self.n += 1
        return bytes(au), idr


def split_nals(au):
    """NAL unit payloads (no start codes) of an Annex B access unit."""
    out = []
    i = au.find(b"\x00\x00\x01")
    while i >= 0:
        j = au.find(b"\x00\x00\x01", i + 3)
        end = len(au) if j < 0 else j
        n = au[i + 3:end]
        if j >= 0 and n.endswith(b"\x00"):
            n = n[:-1]
        out.append(n)
        i = j
    return out


# ---------------------------------------------------------------------------
# RTSP / RTP


class H264File:
    """Replay a raw H.264 stream (Annex B), looping, like H264Pattern.

    For reproducing what a real camera sends: capture one with e.g.
    ffmpeg -rtsp_transport tcp -i rtsp://CAMERA/... -c copy -an -t 20
    -f h264 cam.h264
    """

    def __init__(self, path, width, height, fps):
        data = open(path, "rb").read()
        self.w, self.h, self.fps, self.gop = width, height, fps, 0
        self.aus = []
        self.sps_nal = self.pps_nal = None
        au, vcl = [], False
        for n in split_nals(data):
            if not n:
                continue
            t = n[0] & 0x1F
            first_mb0 = t in (1, 5) and len(n) > 1 and n[1] & 0x80
            # a new access unit: an AUD/SPS/PPS/SEI, or a picture's first
            # slice, after the previous picture's slices
            if vcl and (t in (6, 7, 8, 9) or first_mb0):
                self.aus.append(au)
                au, vcl = [], False
            if t == 7 and self.sps_nal is None:
                self.sps_nal = n
            if t == 8 and self.pps_nal is None:
                self.pps_nal = n
            au.append(n)
            vcl |= t in (1, 5)
        if vcl:
            self.aus.append(au)
        # start, and loop, at the first IDR
        idrs = [i for i, a in enumerate(self.aus)
                if any(x[0] & 0x1F == 5 for x in a)]
        if not idrs or not self.sps_nal or not self.pps_nal:
            raise SystemExit("%s: no SPS/PPS/IDR in it" % path)
        self.aus = self.aus[idrs[0]:]
        self.n = 0

    def sps(self):
        return b"\x00\x00\x00\x01" + self.sps_nal

    def pps(self):
        return b"\x00\x00\x00\x01" + self.pps_nal

    def picture(self):
        au = self.aus[self.n % len(self.aus)]
        self.n += 1
        idr = any(x[0] & 0x1F == 5 for x in au)
        return b"".join(b"\x00\x00\x00\x01" + x for x in au), idr


class H265File(H264File):
    """Replay a raw H.265 stream (Annex B), looping, starting at an IRAP."""

    hevc = True

    def __init__(self, path, width, height, fps):
        data = open(path, "rb").read()
        self.w, self.h, self.fps, self.gop = width, height, fps, 0
        self.aus = []
        self.ps = {}
        au, vcl = [], False
        for n in split_nals(data):
            if len(n) < 3:
                continue
            t = (n[0] >> 1) & 0x3F
            first = t < 32 and n[2] & 0x80
            # a new access unit: a VPS/SPS/PPS/AUD/prefix SEI, or a
            # picture's first slice, after the previous picture's slices
            if vcl and (t in (32, 33, 34, 35, 39) or first):
                self.aus.append(au)
                au, vcl = [], False
            if t in (32, 33, 34):
                self.ps.setdefault(t, n)
            au.append(n)
            vcl |= t < 32
        if vcl:
            self.aus.append(au)
        iraps = [i for i, a in enumerate(self.aus) if self.irap(a)]
        if not iraps or len(self.ps) < 3:
            raise SystemExit("%s: no VPS/SPS/PPS/IRAP in it" % path)
        self.aus = self.aus[iraps[0]:]
        self.n = 0

    @staticmethod
    def irap(au):
        return any(16 <= (x[0] >> 1) & 0x3F <= 23 for x in au)

    def vps(self):
        return b"\x00\x00\x00\x01" + self.ps[32]

    def sps(self):
        return b"\x00\x00\x00\x01" + self.ps[33]

    def pps(self):
        return b"\x00\x00\x00\x01" + self.ps[34]

    def picture(self):
        au = self.aus[self.n % len(self.aus)]
        self.n += 1
        return b"".join(b"\x00\x00\x00\x01" + x for x in au), self.irap(au)


class Stream:
    """One shared encoder; clients read the latest access units."""

    def __init__(self, pattern):
        self.pat = pattern
        self.cond = threading.Condition()
        self.au = None
        self.seq = 0
        self.sps, self.pps = split_nals(pattern.sps())[0], \
            split_nals(pattern.pps())[0]
        self.hevc = getattr(pattern, "hevc", False)
        self.vps = split_nals(pattern.vps())[0] if self.hevc else None
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        period = 1.0 / self.pat.fps
        nxt = time.monotonic()
        while True:
            au, idr = self.pat.picture()
            with self.cond:
                self.au = (au, idr, self.seq)
                self.seq += 1
                self.cond.notify_all()
            nxt += period
            time.sleep(max(0, nxt - time.monotonic()))

    def wait(self, after):
        with self.cond:
            while self.au is None or self.au[2] <= after:
                self.cond.wait()
            return self.au


def rtp_packets(nalu, seq, ts, ssrc, marker_last, mtu=1400, hevc=False):
    hdr = lambda m, s: struct.pack(">BBHII", 0x80, (m << 7) | 96, s & 0xFFFF,
                                   ts & 0xFFFFFFFF, ssrc)
    if len(nalu) <= mtu:
        return [hdr(marker_last, seq) + nalu], seq + 1
    out = []
    if hevc:                        # RFC 7798 fragmentation unit, type 49
        ind = bytes([(nalu[0] & 0x81) | (49 << 1), nalu[1]])
        typ = (nalu[0] >> 1) & 0x3F
        body = nalu[2:]
    else:
        ind = bytes([(nalu[0] & 0xE0) | 28])
        typ = nalu[0] & 0x1F
        body = nalu[1:]
    first = True
    while body:
        chunk, body = body[:mtu], body[mtu:]
        fu = typ | (0x80 if first else 0) | (0x40 if not body else 0)
        m = marker_last if not body else 0
        out.append(hdr(m, seq) + ind + bytes([fu]) + chunk)
        seq += 1
        first = False
    return out, seq


def rtcp_sr(ssrc, ts, packets, octets):
    """RTCP sender report + SDES CNAME (a compound packet)."""
    now = time.time() + 2208988800             # NTP epoch
    sr = struct.pack(">BBHIIIIII", 0x80, 200, 6, ssrc, int(now),
                     int((now % 1) * (1 << 32)), ts & 0xFFFFFFFF, packets,
                     octets & 0xFFFFFFFF)
    cname = b"fake-ipcam"
    item = bytes([1, len(cname)]) + cname + b"\x00"
    item += b"\x00" * (-(len(item) + 4) % 4)
    sdes = struct.pack(">BBHI", 0x81, 202, (len(item) + 4) // 4, ssrc) + item
    return sr + sdes


class RTSPHandler(socketserver.StreamRequestHandler):
    def setup(self):
        super().setup()
        self.session = "%08X" % random.getrandbits(32)
        self.channel = 0
        self.pumper = None
        self.playing = False
        self.lock = threading.Lock()        # RTSP replies vs RTP writes
        self.ssrc = random.getrandbits(32)
        self.seq = random.getrandbits(16)
        self.track = ""
        self.stream = self.server.streams["main"]

    def reply(self, cseq, code="200 OK", headers=(), body=b""):
        msg = "RTSP/1.0 %s\r\nCSeq: %s\r\n" % (code, cseq)
        for h in headers:
            msg += h + "\r\n"
        if body:
            msg += "Content-Length: %d\r\n" % len(body)
        msg += "\r\n"
        with self.lock:
            self.wfile.write(msg.encode() + body)

    def pick_stream(self, url):
        """The sub stream for URLs that ask for it, as cameras name it."""
        path = url.lower()
        sub = any(k in path for k in ("sub", "stream2", "subtype=1",
                                      "channel=2", "/102", "/12",
                                      "stream=1"))
        self.stream = self.server.streams["sub" if sub else "main"]

    def sdp(self):
        st = self.stream
        b64 = lambda n: base64.b64encode(n).decode()
        if st.hevc:
            fmt = ("a=rtpmap:96 H265/90000\r\n"
                   "a=fmtp:96 sprop-vps=%s;sprop-sps=%s;sprop-pps=%s\r\n"
                   % (b64(st.vps), b64(st.sps), b64(st.pps)))
        else:
            fmt = ("a=rtpmap:96 H264/90000\r\n"
                   "a=fmtp:96 packetization-mode=1;profile-level-id=%s;"
                   "sprop-parameter-sets=%s,%s\r\n"
                   % (st.sps[1:4].hex(), b64(st.sps), b64(st.pps)))
        ip = self.server.advertise
        return ("v=0\r\no=- 0 0 IN IP4 %s\r\ns=fake-ipcam\r\n"
                "c=IN IP4 0.0.0.0\r\nt=0 0\r\na=control:*\r\n"
                "a=range:npt=0-\r\n"
                "m=video 0 RTP/AVP 96\r\na=recvonly\r\n%s"
                "a=framerate:%d\r\na=control:track1\r\n"
                % (ip, fmt, st.pat.fps)).encode()

    def handle(self):
        while True:
            first = self.rfile.read(1)
            if not first:
                return
            if first == b"$":               # interleaved RTCP from client
                rest = self.rfile.read(3)
                if len(rest) < 3:
                    return
                self.rfile.read(struct.unpack(">H", rest[1:3])[0])
                continue
            line = first + self.rfile.readline()
            req = line.decode(errors="replace").strip()
            if not req:
                continue
            hdrs = {}
            while True:
                h = self.rfile.readline().decode(errors="replace").strip()
                if not h:
                    break
                k, _, v = h.partition(":")
                hdrs[k.strip().lower()] = v.strip()
            if "content-length" in hdrs:
                self.rfile.read(int(hdrs["content-length"]))
            method = req.split()[0]
            cseq = hdrs.get("cseq", "0")
            log("rtsp: %s" % req)
            if method == "OPTIONS":
                self.reply(cseq, headers=["Public: OPTIONS, DESCRIBE, SETUP, "
                                          "PLAY, PAUSE, TEARDOWN, "
                                          "GET_PARAMETER, SET_PARAMETER"])
            elif method == "DESCRIBE":
                url = req.split()[1]
                self.pick_stream(url)
                self.reply(cseq, headers=["Content-Type: application/sdp",
                                          "Content-Base: %s/" % url],
                           body=self.sdp())
            elif method == "SETUP":
                tr = hdrs.get("transport", "")
                if "interleaved" not in tr and "TCP" not in tr:
                    # RTP over UDP would go to the --relay process, not
                    # the guest: clients retry with TCP interleaving.
                    self.reply(cseq, "461 Unsupported Transport")
                    continue
                ch = 0
                for part in tr.split(";"):
                    if part.startswith("interleaved="):
                        ch = int(part[12:].split("-")[0])
                self.channel = ch
                rtr = "RTP/AVP/TCP;unicast;interleaved=%d-%d" % (ch, ch + 1)
                rtr += ';ssrc=%08X;mode="play"' % self.ssrc
                self.track = req.split()[1]
                self.pick_stream(self.track)
                self.reply(cseq, headers=["Transport: " + rtr,
                                          "Session: %s;timeout=60"
                                          % self.session])
            elif method == "PLAY":
                st = self.stream
                nxt = st.au[2] + 1 if st.au else 0
                self.reply(cseq, headers=[
                    "Session: " + self.session, "Range: npt=0.000-",
                    "RTP-Info: url=%s;seq=%d;rtptime=%d"
                    % (self.track, self.seq,
                       int(nxt * 90000 / st.pat.fps) & 0xFFFFFFFF)])
                if not self.playing:
                    if self.pumper:
                        self.pumper.join()      # a PAUSEd one, stopping
                    self.playing = True
                    self.pumper = threading.Thread(target=self.pump,
                                                   daemon=True)
                    self.pumper.start()
            elif method == "PAUSE":
                self.playing = False            # the pump stops
                self.reply(cseq, headers=["Session: " + self.session])
            elif method in ("GET_PARAMETER", "SET_PARAMETER"):
                self.reply(cseq, headers=["Session: " + self.session])
            elif method == "TEARDOWN":
                self.reply(cseq, headers=["Session: " + self.session])
                self.playing = False
                return
            else:
                self.reply(cseq, "501 Not Implemented")

    def send_rtp(self, rtcp, p):
        """RTP (or RTCP) interleaved on the RTSP connection."""
        with self.lock:
            self.wfile.write(struct.pack(">BBH", 0x24, self.channel + rtcp,
                                         len(p)) + p)

    def pump(self):
        st = self.stream
        ssrc = self.ssrc
        seq = self.seq
        last = -1
        started = False
        packets = octets = 0
        last_sr = 0
        try:
            while self.playing:
                au, idr, last = st.wait(last)
                if not started and not idr:
                    continue                    # start on an IDR
                started = True
                ts = int(last * 90000 / st.pat.fps)
                nals = split_nals(au)
                for i, n in enumerate(nals):
                    pkts, seq = rtp_packets(n, seq, ts, ssrc,
                                            i == len(nals) - 1,
                                            hevc=st.hevc)
                    for p in pkts:
                        packets += 1
                        octets += len(p) - 12
                        self.send_rtp(0, p)
                if time.monotonic() - last_sr >= 1:
                    last_sr = time.monotonic()
                    self.send_rtp(1, rtcp_sr(ssrc, ts, packets, octets))
        except OSError:
            pass
        self.seq = seq                  # a PLAY after PAUSE carries on
        self.playing = False


class TServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


# ---------------------------------------------------------------------------
# ONVIF


NS = ('xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
      'xmlns:tds="http://www.onvif.org/ver10/device/wsdl" '
      'xmlns:trt="http://www.onvif.org/ver10/media/wsdl" '
      'xmlns:tt="http://www.onvif.org/ver10/schema"')


def envelope(body):
    return ('<?xml version="1.0" encoding="UTF-8"?>\n<s:Envelope %s>'
            '<s:Body>%s</s:Body></s:Envelope>' % (NS, body)).encode()


def onvif_body(action, srv, body=b""):
    """The response to an ONVIF action.  Profile_1 is the main stream,
    Profile_2 the sub stream (NVRs live-view their split screens from it);
    a request naming Profile_2 or VideoEncoder_2 gets the sub stream's."""
    ip, port = srv.advertise, srv.advertise_http
    sub = b"Profile_2" in body or b"VideoEncoder_2" in body
    pat = srv.streams["sub" if sub else "main"].pat
    w, h, fps = pat.w, pat.h, pat.fps
    dev = "http://%s:%d/onvif/device_service" % (ip, port)
    med = "http://%s:%d/onvif/media_service" % (ip, port)
    now = datetime.datetime.now(datetime.timezone.utc)
    def encoder(n, w, h):
        return ('<tt:Name>VideoEncoder_%d</tt:Name><tt:UseCount>1</tt:UseCount>'
            '<tt:Encoding>H264</tt:Encoding><tt:Resolution><tt:Width>%d'
            '</tt:Width><tt:Height>%d</tt:Height></tt:Resolution>'
            '<tt:Quality>5</tt:Quality><tt:RateControl><tt:FrameRateLimit>%d'
            '</tt:FrameRateLimit><tt:EncodingInterval>1</tt:EncodingInterval>'
            '<tt:BitrateLimit>4096</tt:BitrateLimit></tt:RateControl>'
            '<tt:H264><tt:GovLength>%d</tt:GovLength><tt:H264Profile>Baseline'
            '</tt:H264Profile></tt:H264><tt:Multicast><tt:Address><tt:Type>'
            'IPv4</tt:Type><tt:IPv4Address>0.0.0.0</tt:IPv4Address>'
            '</tt:Address><tt:Port>0</tt:Port><tt:TTL>0</tt:TTL>'
            '<tt:AutoStart>false</tt:AutoStart></tt:Multicast>'
            '<tt:SessionTimeout>PT60S</tt:SessionTimeout>'
            % (n, w, h, fps, pat.gop))

    venc = encoder(2 if sub else 1, w, h)
    vsrc = ('<tt:Name>VideoSource_1</tt:Name><tt:UseCount>1</tt:UseCount>'
            '<tt:SourceToken>VideoSource_1</tt:SourceToken><tt:Bounds x="0" '
            'y="0" width="%d" height="%d"/>' % (w, h))
    def profile(n, name, p):
        return ('<trt:Profiles token="Profile_%d" fixed="true"><tt:Name>'
                '%s</tt:Name><tt:VideoSourceConfiguration '
                'token="VideoSourceConfig_1">%s</tt:VideoSourceConfiguration>'
                '<tt:VideoEncoderConfiguration token="VideoEncoder_%d">%s'
                '</tt:VideoEncoderConfiguration></trt:Profiles>'
                % (n, name, vsrc, n, encoder(n, p.w, p.h)))

    profiles = (profile(1, "mainStream", srv.streams["main"].pat) +
                profile(2, "subStream", srv.streams["sub"].pat))
    one = profile(2, "subStream", pat) if sub else \
        profile(1, "mainStream", pat)
    bodies = {
        "GetSystemDateAndTime":
            '<tds:GetSystemDateAndTimeResponse><tds:SystemDateAndTime>'
            '<tt:DateTimeType>Manual</tt:DateTimeType><tt:DaylightSavings>'
            'false</tt:DaylightSavings><tt:TimeZone><tt:TZ>GMT0</tt:TZ>'
            '</tt:TimeZone><tt:UTCDateTime><tt:Time><tt:Hour>%d</tt:Hour>'
            '<tt:Minute>%d</tt:Minute><tt:Second>%d</tt:Second></tt:Time>'
            '<tt:Date><tt:Year>%d</tt:Year><tt:Month>%d</tt:Month><tt:Day>%d'
            '</tt:Day></tt:Date></tt:UTCDateTime></tds:SystemDateAndTime>'
            '</tds:GetSystemDateAndTimeResponse>'
            % (now.hour, now.minute, now.second, now.year, now.month,
               now.day),
        "GetDeviceInformation":
            '<tds:GetDeviceInformationResponse><tds:Manufacturer>OpenIPC'
            '</tds:Manufacturer><tds:Model>fake-ipcam</tds:Model>'
            '<tds:FirmwareVersion>1.0</tds:FirmwareVersion><tds:SerialNumber>'
            '0001</tds:SerialNumber><tds:HardwareId>qemu</tds:HardwareId>'
            '</tds:GetDeviceInformationResponse>',
        "GetCapabilities":
            '<tds:GetCapabilitiesResponse><tds:Capabilities><tt:Device>'
            '<tt:XAddr>%s</tt:XAddr></tt:Device><tt:Media><tt:XAddr>%s'
            '</tt:XAddr><tt:StreamingCapabilities><tt:RTPMulticast>false'
            '</tt:RTPMulticast><tt:RTP_TCP>true</tt:RTP_TCP>'
            '<tt:RTP_RTSP_TCP>true</tt:RTP_RTSP_TCP></tt:StreamingCapabilities>'
            '</tt:Media></tds:Capabilities></tds:GetCapabilitiesResponse>'
            % (dev, med),
        "GetServices":
            '<tds:GetServicesResponse><tds:Service><tds:Namespace>'
            'http://www.onvif.org/ver10/device/wsdl</tds:Namespace>'
            '<tds:XAddr>%s</tds:XAddr><tds:Version><tt:Major>2</tt:Major>'
            '<tt:Minor>40</tt:Minor></tds:Version></tds:Service><tds:Service>'
            '<tds:Namespace>http://www.onvif.org/ver10/media/wsdl'
            '</tds:Namespace><tds:XAddr>%s</tds:XAddr><tds:Version><tt:Major>'
            '2</tt:Major><tt:Minor>40</tt:Minor></tds:Version></tds:Service>'
            '</tds:GetServicesResponse>' % (dev, med),
        "GetScopes":
            '<tds:GetScopesResponse><tds:Scopes><tt:ScopeDef>Fixed'
            '</tt:ScopeDef><tt:ScopeItem>onvif://www.onvif.org/name/'
            'fake-ipcam</tt:ScopeItem></tds:Scopes></tds:GetScopesResponse>',
        "GetNetworkInterfaces":
            '<tds:GetNetworkInterfacesResponse><tds:NetworkInterfaces '
            'token="eth0"><tt:Enabled>true</tt:Enabled><tt:Info><tt:Name>eth0'
            '</tt:Name><tt:HwAddress>00:12:34:56:78:9a</tt:HwAddress>'
            '<tt:MTU>1500</tt:MTU></tt:Info><tt:IPv4><tt:Enabled>true'
            '</tt:Enabled><tt:Config><tt:Manual><tt:Address>%s</tt:Address>'
            '<tt:PrefixLength>24</tt:PrefixLength></tt:Manual><tt:DHCP>false'
            '</tt:DHCP></tt:Config></tt:IPv4></tds:NetworkInterfaces>'
            '</tds:GetNetworkInterfacesResponse>' % ip,
        "GetProfiles": '<trt:GetProfilesResponse>%s</trt:GetProfilesResponse>'
            % profiles,
        "GetProfile": '<trt:GetProfileResponse>%s</trt:GetProfileResponse>'
            % one.replace("trt:Profiles", "trt:Profile"),
        "GetVideoSources":
            '<trt:GetVideoSourcesResponse><trt:VideoSources '
            'token="VideoSource_1"><tt:Framerate>%d</tt:Framerate>'
            '<tt:Resolution><tt:Width>%d</tt:Width><tt:Height>%d</tt:Height>'
            '</tt:Resolution></trt:VideoSources></trt:GetVideoSourcesResponse>'
            % (fps, w, h),
        "GetVideoSourceConfigurations":
            '<trt:GetVideoSourceConfigurationsResponse><trt:Configurations '
            'token="VideoSourceConfig_1">%s</trt:Configurations>'
            '</trt:GetVideoSourceConfigurationsResponse>' % vsrc,
        "GetVideoEncoderConfigurations":
            '<trt:GetVideoEncoderConfigurationsResponse><trt:Configurations '
            'token="VideoEncoder_1">%s</trt:Configurations><trt:Configurations '
            'token="VideoEncoder_2">%s</trt:Configurations>'
            '</trt:GetVideoEncoderConfigurationsResponse>'
            % (encoder(1, srv.streams["main"].pat.w, srv.streams["main"].pat.h),
               encoder(2, srv.streams["sub"].pat.w, srv.streams["sub"].pat.h)),
        "GetVideoEncoderConfiguration":
            '<trt:GetVideoEncoderConfigurationResponse><trt:Configuration '
            'token="VideoEncoder_%d">%s</trt:Configuration>'
            '</trt:GetVideoEncoderConfigurationResponse>'
            % (2 if sub else 1, venc),
        "GetStreamUri":
            '<trt:GetStreamUriResponse><trt:MediaUri><tt:Uri>'
            'rtsp://%s:%d/%s</tt:Uri><tt:InvalidAfterConnect>false'
            '</tt:InvalidAfterConnect><tt:InvalidAfterReboot>false'
            '</tt:InvalidAfterReboot><tt:Timeout>PT0S</tt:Timeout>'
            '</trt:MediaUri></trt:GetStreamUriResponse>'
            % (ip, srv.advertise_rtsp, "sub" if sub else "stream1"),
        "GetSnapshotUri":
            '<trt:GetSnapshotUriResponse><trt:MediaUri><tt:Uri>'
            'http://%s:%d/snapshot.jpg</tt:Uri></trt:MediaUri>'
            '</trt:GetSnapshotUriResponse>' % (ip, port),
        "GetAudioSources": '<trt:GetAudioSourcesResponse/>',
        "GetAudioEncoderConfigurations":
            '<trt:GetAudioEncoderConfigurationsResponse/>',
        "SetSystemDateAndTime": '<tds:SetSystemDateAndTimeResponse/>',
        "SetVideoEncoderConfiguration":
            '<trt:SetVideoEncoderConfigurationResponse/>',
        "GetVideoEncoderConfigurationOptions":
            '<trt:GetVideoEncoderConfigurationOptionsResponse><trt:Options>'
            '<tt:QualityRange><tt:Min>1</tt:Min><tt:Max>6</tt:Max>'
            '</tt:QualityRange><tt:H264><tt:ResolutionsAvailable><tt:Width>%d'
            '</tt:Width><tt:Height>%d</tt:Height></tt:ResolutionsAvailable>'
            '<tt:GovLengthRange><tt:Min>1</tt:Min><tt:Max>%d</tt:Max>'
            '</tt:GovLengthRange><tt:FrameRateRange><tt:Min>1</tt:Min>'
            '<tt:Max>%d</tt:Max></tt:FrameRateRange><tt:EncodingIntervalRange>'
            '<tt:Min>1</tt:Min><tt:Max>1</tt:Max></tt:EncodingIntervalRange>'
            '<tt:H264ProfilesSupported>Baseline</tt:H264ProfilesSupported>'
            '</tt:H264></trt:Options></trt:GetVideoEncoderConfigurationOptionsResponse>'
            % (w, h, max(1, pat.gop), fps),
        "GetNetworkProtocols":
            '<tds:GetNetworkProtocolsResponse><tds:NetworkProtocols><tt:Name>'
            'HTTP</tt:Name><tt:Enabled>true</tt:Enabled><tt:Port>%d'
            '</tt:Port></tds:NetworkProtocols><tds:NetworkProtocols><tt:Name>'
            'RTSP</tt:Name><tt:Enabled>true</tt:Enabled><tt:Port>%d'
            '</tt:Port></tds:NetworkProtocols></tds:GetNetworkProtocolsResponse>'
            % (port, srv.advertise_rtsp),
        "GetHostname":
            '<tds:GetHostnameResponse><tds:HostnameInformation><tt:FromDHCP>'
            'false</tt:FromDHCP><tt:Name>fake-ipcam</tt:Name>'
            '</tds:HostnameInformation></tds:GetHostnameResponse>',
    }
    return bodies.get(action)


def soap_action(body):
    """The first element of the SOAP Body."""
    i = body.find(b"Body")
    if i < 0:
        return None
    i = body.find(b"<", i)
    while i >= 0 and body[i + 1:i + 2] in (b"/", b"!", b"?"):
        i = body.find(b"<", i + 1)
    if i < 0:
        return None
    j = i + 1
    while j < len(body) and body[j:j + 1] not in b" />\r\n\t":
        j += 1
    return body[i + 1:j].decode(errors="replace").split(":")[-1]


class HTTPHandler(socketserver.StreamRequestHandler):
    def snapshot(self):
        """The main stream's current picture as a JPEG (the pattern only:
        a file stream has no pictures to give)."""
        pat = self.server.streams["main"].pat
        planes = getattr(pat, "last", None)
        if planes:
            code, ctype = "200 OK", "image/jpeg"
            data = jpeg_encode(pat.w, pat.h, *planes, pat.mbw * 16)
        else:
            code, ctype, data = "404 Not Found", "text/plain", b"no picture"
        log("http: GET /snapshot.jpg -> %s, %d bytes" % (code, len(data)))
        self.wfile.write(("HTTP/1.1 %s\r\nContent-Type: %s\r\n"
                          "Content-Length: %d\r\n\r\n"
                          % (code, ctype, len(data))).encode() + data)

    def handle(self):
        while True:
            line = self.rfile.readline()
            if not line:
                return
            req = line.decode(errors="replace").strip()
            if not req:
                continue
            hdrs = {}
            while True:
                h = self.rfile.readline().decode(errors="replace").strip()
                if not h:
                    break
                k, _, v = h.partition(":")
                hdrs[k.strip().lower()] = v.strip()
            body = self.rfile.read(int(hdrs.get("content-length", "0")))
            if req.split(" ")[:2] == ["GET", "/snapshot.jpg"]:
                self.snapshot()
                if hdrs.get("connection", "").lower() == "close":
                    return
                continue
            action = soap_action(body)
            resp = onvif_body(action, self.server, body) if action else None
            log("onvif: %s %s -> %s" % (req, action,
                                        "ok" if resp else "fault"))
            if resp:
                code, data = "200 OK", envelope(resp)
            else:
                code = "500 Internal Server Error"
                data = envelope(
                    '<s:Fault><s:Code><s:Value>s:Receiver</s:Value>'
                    '<s:Subcode><s:Value>ter:ActionNotSupported</s:Value>'
                    '</s:Subcode></s:Code><s:Reason><s:Text xml:lang="en">'
                    'not supported</s:Text></s:Reason></s:Fault>')
            self.wfile.write(("HTTP/1.1 %s\r\nContent-Type: application/"
                              "soap+xml; charset=utf-8\r\nContent-Length: "
                              "%d\r\n\r\n" % (code, len(data))).encode()
                             + data)
            if hdrs.get("connection", "").lower() == "close":
                return


# ---------------------------------------------------------------------------
# JPEG, for the ONVIF snapshot (GetSnapshotUri)

# ITU T.81 Annex K: the typical Huffman tables (code counts by length, then
# the symbols)
JPEG_DC_L = ([0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0], range(12))
JPEG_DC_C = ([0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0], range(12))
JPEG_AC_L = ([0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7D], bytes.fromhex(
    "01020300041105122131410613516107227114328191a1082342b1c11552d1f0"
    "2433627282090a161718191a25262728292a3435363738393a434445464748494a"
    "535455565758595a636465666768696a737475767778797a838485868788898a"
    "92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6"
    "c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa"))
JPEG_AC_C = ([0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77], bytes.fromhex(
    "000102031104052131061241510761711322328108144291a1b1c109233352f0"
    "156272d10a162434e125f11718191a262728292a35363738393a434445464748"
    "494a535455565758595a636465666768696a737475767778797a828384858687"
    "88898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3"
    "c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8f9fa"))


def huff_codes(table):
    """{symbol: (code, length)} of a (counts, symbols) table."""
    codes, code, k, syms = {}, 0, 0, list(table[1])
    for length, count in enumerate(table[0], 1):
        for _ in range(count):
            codes[syms[k]] = (code, length)
            code += 1
            k += 1
        code <<= 1
    return codes


JPEG_COS = [[(math.sqrt(0.5) if u == 0 else 1.0) *
             math.cos((2 * x + 1) * u * math.pi / 16) / 2 for x in range(8)]
            for u in range(8)]
JPEG_ZIGZAG = sorted(range(64), key=lambda i: (
    i // 8 + i % 8, (i % 8) if (i // 8 + i % 8) & 1 == 0 else i // 8))


def jpeg_encode(w, h, y, u, v, stride):
    """Baseline 4:2:0 JPEG of the planes (Y at stride, U / V at stride / 2,
    padded to 16 lines), unquantised (every step 1).  A flat block -- most
    of the pattern -- codes its DC alone; the others get the full DCT."""
    dcl, dcc = huff_codes(JPEG_DC_L), huff_codes(JPEG_DC_C)
    acl, acc_c = huff_codes(JPEG_AC_L), huff_codes(JPEG_AC_C)
    out = bytearray()
    acc = [0, 0]                                # bits, count

    def put(code, n):
        acc[0] = (acc[0] << n) | code
        acc[1] += n
        while acc[1] >= 8:
            acc[1] -= 8
            b = (acc[0] >> acc[1]) & 0xFF
            out.append(b)
            if b == 0xFF:
                out.append(0)                   # byte stuffing

    def coef(val, codes, run=None):
        cat = abs(val).bit_length()
        put(*codes[cat if run is None else (run << 4) | cat])
        if cat:
            put(val if val > 0 else val + (1 << cat) - 1, cat)

    def block(plane, pstride, x0, y0, pred, dc, ac):
        rows = [plane[r * pstride + x0:r * pstride + x0 + 8]
                for r in range(y0, y0 + 8)]
        first = rows[0][0]
        if all(p == first for row in rows for p in row):
            val = (first - 128) * 8             # the FDCT's DC, step 1
            coef(val - pred, dc)
            put(*ac[0])                         # EOB
            return val
        f = [[p - 128 for p in row] for row in rows]
        t = [[sum(JPEG_COS[k][x] * f[r][x] for x in range(8))
              for k in range(8)] for r in range(8)]
        F = [round(sum(JPEG_COS[k][r] * t[r][j] for r in range(8)))
             for k in range(8) for j in range(8)]
        F = [max(-1023, min(1023, c)) for c in F]
        coef(F[0] - pred, dc)
        run = 0
        for i in JPEG_ZIGZAG[1:]:
            if F[i] == 0:
                run += 1
                continue
            while run > 15:
                put(*ac[0xF0])                  # ZRL
                run -= 16
            coef(F[i], ac, run)
            run = 0
        if run:
            put(*ac[0])
        return F[0]

    cs = stride // 2
    py = pu = pv = 0
    for my in range(0, (h + 15) // 16 * 16, 16):
        for mx in range(0, (w + 15) // 16 * 16, 16):
            for by, bx in ((0, 0), (0, 8), (8, 0), (8, 8)):
                py = block(y, stride, mx + bx, my + by, py, dcl, acl)
            pu = block(u, cs, mx // 2, my // 2, pu, dcc, acc_c)
            pv = block(v, cs, mx // 2, my // 2, pv, dcc, acc_c)
    if acc[1]:
        put((1 << (8 - acc[1])) - 1, 8 - acc[1])   # pad with ones

    def seg(marker, body):
        return struct.pack(">BBH", 0xFF, marker, len(body) + 2) + body

    def dht(cls_id, table):
        return bytes([cls_id]) + bytes(table[0]) + bytes(table[1])

    hdr = b"\xff\xd8"
    hdr += seg(0xE0, b"JFIF\0\x01\x01\0\0\x01\0\x01\0\0")
    hdr += seg(0xDB, b"\x00" + b"\x01" * 64 + b"\x01" + b"\x01" * 64)
    hdr += seg(0xC0, struct.pack(">BHHB", 8, h, w, 3) +
               b"\x01\x22\x00\x02\x11\x01\x03\x11\x01")
    hdr += seg(0xC4, dht(0x00, JPEG_DC_L) + dht(0x10, JPEG_AC_L) +
               dht(0x01, JPEG_DC_C) + dht(0x11, JPEG_AC_C))
    hdr += seg(0xDA, b"\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00")
    return hdr + bytes(out) + b"\xff\xd9"


# ---------------------------------------------------------------------------


def log(msg):
    if ARGS.verbose:
        print(time.strftime("%H:%M:%S ") + msg, file=sys.stderr, flush=True)


def relay(host, port):
    sock = socket.create_connection((host, port))

    def up():
        while True:
            data = os.read(0, 65536)
            if not data:
                sock.shutdown(socket.SHUT_WR)
                return
            sock.sendall(data)

    threading.Thread(target=up, daemon=True).start()
    out = os.fdopen(1, "wb", buffering=0)
    while True:
        data = sock.recv(65536)
        if not data:
            return
        view = memoryview(data)
        while view:
            n = out.write(view)
            if n is None:           # non-blocking stdout, full: wait
                select.select([], [1], [])
                continue
            view = view[n:]


def main():
    global ARGS
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--http", type=int, default=18080,
                    help="ONVIF HTTP port to listen on")
    ap.add_argument("--rtsp", type=int, default=18554,
                    help="RTSP port to listen on")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--advertise", default="192.168.1.10",
                    help="the camera's address as the NVR sees it")
    ap.add_argument("--advertise-http", type=int, default=80)
    ap.add_argument("--advertise-rtsp", type=int, default=554)
    ap.add_argument("--size", default="640x360",
                    help="main stream size")
    ap.add_argument("--sub-size", default="640x360",
                    help="sub stream size (rtsp URLs with 'sub' in them):"
                    " NVRs live-view their split screens from it")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--gop", type=int, default=20)
    ap.add_argument("--file", help="stream this raw H.264 file (looped) "
                    "instead of the pattern, H.265 if it ends in .hevc or "
                    ".h265; --size/--fps describe it")
    ap.add_argument("--sub-file", help="the sub stream's file, with "
                    "--file; --sub-size describes it")
    ap.add_argument("--write", help="write an Annex B file and exit")
    ap.add_argument("--frames", type=int, default=50)
    ap.add_argument("--relay", type=int, metavar="PORT",
                    help="copy stdin/stdout to and from a local port "
                    "(for QEMU guestfwd=...-cmd:)")
    ap.add_argument("-v", "--verbose", action="store_true")
    ARGS = ap.parse_args()
    if ARGS.relay:
        relay(ARGS.bind, ARGS.relay)
        return
    w, h = (int(x) for x in ARGS.size.split("x"))

    def from_file(path, w, h):
        hevc = path.lower().endswith((".hevc", ".h265", ".265"))
        return (H265File if hevc else H264File)(path, w, h, ARGS.fps)

    if ARGS.file:
        pat = from_file(ARGS.file, w, h)
    else:
        pat = H264Pattern(w, h, ARGS.fps, ARGS.gop)

    if ARGS.write:
        with open(ARGS.write, "wb") as f:
            for _ in range(ARGS.frames):
                f.write(pat.picture()[0])
        return

    streams = {"main": Stream(pat)}
    if ARGS.file and ARGS.sub_file:
        sw, sh = (int(x) for x in ARGS.sub_size.split("x"))
        streams["sub"] = Stream(from_file(ARGS.sub_file, sw, sh))
    elif ARGS.sub_size == ARGS.size or ARGS.file:
        streams["sub"] = streams["main"]
    else:
        sw, sh = (int(x) for x in ARGS.sub_size.split("x"))
        streams["sub"] = Stream(H264Pattern(sw, sh, ARGS.fps, ARGS.gop))
    for port, handler, extra in ((ARGS.rtsp, RTSPHandler, None),
                                 (ARGS.http, HTTPHandler, None)):
        srv = TServer((ARGS.bind, port), handler)
        srv.streams = streams
        srv.stream = streams["main"]
        srv.advertise = ARGS.advertise
        srv.advertise_http = ARGS.advertise_http
        srv.advertise_rtsp = ARGS.advertise_rtsp
        threading.Thread(target=srv.serve_forever, daemon=True).start()
    print("fake-ipcam: ONVIF on %s:%d, RTSP on %s:%d (rtsp://%s:%d/stream1)"
          % (ARGS.bind, ARGS.http, ARGS.bind, ARGS.rtsp, ARGS.advertise,
             ARGS.advertise_rtsp), flush=True)
    while True:
        time.sleep(3600)


ARGS = None

if __name__ == "__main__":
    main()
