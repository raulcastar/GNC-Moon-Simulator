"""
TCP receiver for the UnrealStereoCam plugin's frame stream (NAVF packets).

Input/output only: sockets, threads, packet parsing and the per-camera frame
rings the stereo node reads from. No ROS.
"""

import collections
import socket
import struct
import threading
import time
from typing import Dict, Optional

import cv2
import numpy as np


class FrameStreamReceiver:
    """
    TCP server for the UnrealStereoCam plugin (the Unreal project's Plugins/UnrealStereoCam/).

    Unreal renders the navcams with each frame and pushes them here over one
    connection, so nothing is rendered on demand and the game thread never
    stalls. Packets are 24-byte-header BGRA frames tagged with
    Unreal's frame counter; captures enqueued in one tick share a tag, which
    is what lets left/right be paired as a rigid stereo view.

    Runs a background thread; survives Play/Stop cycles by re-accepting.
    Several UnrealStereoCam components may connect at once (e.g. the navcam
    rig and a separate rear camera), each on its own connection and reader
    thread; their frames land in the same per-cam-id rings.
    """

    _HEADER = struct.Struct('<4sBBBBHHQI')
    _MAGIC = b'NAVF'
    _RING = 6                      # frames kept per camera for pairing

    def __init__(self, bind_ip: str, port: int, logger):
        self._addr = (bind_ip, port)
        self._log = logger
        self._frames: Dict[int, collections.deque] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._server: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self.connections = 0                    # Unreal streamers connected now
        self.known_ids: set = set()             # camera ids the node uses (set after configure)
        self._unknown_ids: set = set()          # ids already warned about

    def start(self) -> None:
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(self._addr)
        self._server.listen(4)
        self._server.settimeout(0.5)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._log.info(f'Frame stream: listening on {self._addr[0]}:{self._addr[1]} '
                       f'for the UnrealStereoCam plugin.')

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._server:
            self._server.close()
            self._server = None

    def latest_pair(self, left_id: int, right_id: int, center_id: int):
        """
        Newest frame id present for BOTH eyes, with the nearest centre frame.

        Returns (left, right, center, frame_id, age_s) or None. The eyes must
        match exactly — that is the rigid-pair guarantee. The centre tolerates
        a small offset: it is not matched against anything, only overlaid.
        """
        with self._lock:
            eyes_l = {e[0]: e for e in self._frames.get(left_id, ())}
            eyes_r = {e[0]: e[1] for e in self._frames.get(right_id, ())}
            common = sorted(set(eyes_l) & set(eyes_r))
            if not common:
                return None
            fid = common[-1]
            centers = list(self._frames.get(center_id, ()))
            if not centers:
                return None
            c_fid, center, _, _ = min(centers, key=lambda e: abs(e[0] - fid))
            if abs(c_fid - fid) > 60:        # a second of frames apart: stale
                return None
            _, left, t, stamp_ns = eyes_l[fid]
            age = time.monotonic() - t
            return left, eyes_r[fid], center, fid, age, stamp_ns

    def frame_near(self, cam_id: int, fid: int, max_gap: int = 60):
        """
        The frame for cam_id closest to frame id `fid`, or None.

        Used for the object-mask capture, which is streamed on its own cam_id
        but must overlay the central image: take the mask rendered nearest the
        same world tick. `max_gap` matches the centre-pairing tolerance in
        latest_pair — a second of frames apart is treated as stale.
        """
        with self._lock:
            frames = list(self._frames.get(cam_id, ()))
        if not frames:
            return None
        f_fid, img, _, _ = min(frames, key=lambda e: abs(e[0] - fid))
        if abs(f_fid - fid) > max_gap:
            return None
        return img

    def latest_with(self, cam_id: int, other_id: int):
        """
        (frame_id, image, stamp_ns, other_image) of the newest frame of cam_id
        whose frame id other_id also has (a capture and its mask, rendered in
        the same tick), or None while no such pair has arrived.
        """
        with self._lock:
            others = {e[0]: e[1] for e in self._frames.get(other_id, ())}
            for fid, img, _, stamp_ns in reversed(self._frames.get(cam_id, ())):
                if fid in others:
                    return fid, img, stamp_ns, others[fid]
        return None

    def latest(self, cam_id: int):
        """(frame_id, image, stamp_ns) of the newest frame for cam_id, or None."""
        with self._lock:
            frames = self._frames.get(cam_id)
            if not frames:
                return None
            fid, img, _, stamp_ns = frames[-1]
            return fid, img, stamp_ns

    # ------------------------------------------------------------------ #

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self._serve, args=(conn, addr), daemon=True).start()

    def _serve(self, conn: socket.socket, addr) -> None:
        self._log.info(f'Frame stream: Unreal connected from {addr[0]}:{addr[1]}.')
        with self._lock:
            self.connections += 1
        try:
            self._read_stream(conn)
        finally:
            conn.close()
            with self._lock:
                self.connections -= 1
            if not self._stop.is_set():
                self._log.warn(f'Frame stream: Unreal disconnected ({addr[0]}:{addr[1]}); '
                               'waiting for the next Play.')

    def _read_stream(self, conn: socket.socket) -> None:
        conn.settimeout(0.5)
        # A bytearray grows in place: with bytes, every read copied the whole
        # partial frame again (a 1.2 MB frame arrives in many reads).
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = conn.recv(1 << 20)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            self._drain(buf)

    def _drain(self, buf) -> bytearray:
        """
        Extract every complete frame from `buf`, consuming them in place (a
        bytearray; bytes are copied into one), and return what is left: the
        trailing fragment of the next frame.
        """
        if not isinstance(buf, bytearray):
            buf = bytearray(buf)
        while True:
            start = buf.find(self._MAGIC)
            if start < 0:
                del buf[:-3]                 # keep a possible partial magic
                return buf
            if start > 0:
                del buf[:start]              # resync past garbage
            if len(buf) < self._HEADER.size:
                return buf
            (_, version, cam_id, fmt, _, w, h, fid,
             size) = self._HEADER.unpack_from(buf)
            if version not in (1, 2) or fmt not in (1, 2) or size != w * h * 4 or w == 0:
                self._log.warn(f'Frame stream: bad header (v{version} fmt{fmt} '
                               f'{w}x{h} size {size}); resyncing.',
                               throttle_duration_sec=10.0)
                del buf[:4]                  # skip this magic, scan again
                continue
            # Version 2 adds the capture time (simulation clock, ns) after the
            # 24-byte header; version 1 frames get stamped on arrival (0 here).
            start = self._HEADER.size + (8 if version == 2 else 0)
            end = start + size
            if len(buf) < end:
                return buf                   # payload still in flight
            stamp_ns = struct.unpack_from('<Q', buf, self._HEADER.size)[0] if version == 2 else 0
            # bytes() copies the payload out, so no numpy view pins the
            # bytearray (it could not be resized while one exists).
            payload = bytes(buf[start:end])
            if fmt == 2:                     # R32F depth, centimetres
                image = np.frombuffer(payload, dtype='<f4').reshape(h, w).copy()
            else:                            # BGRA8 colour -> RGB (cv2: ~30x faster than indexing)
                image = cv2.cvtColor(np.frombuffer(payload, dtype=np.uint8).reshape(h, w, 4),
                                     cv2.COLOR_BGRA2RGB)
            if self.known_ids and cam_id not in self.known_ids and cam_id not in self._unknown_ids:
                self._unknown_ids.add(cam_id)
                self._log.warn(
                    f'Frame stream: camera id {cam_id} ({w}x{h}) is not one this node '
                    f'uses (configured: {sorted(self.known_ids)}); its frames are ignored. '
                    'Check the camera ids in the UnrealStereoCam components against '
                    'config/params.yaml.')
            with self._lock:
                ring = self._frames.setdefault(
                    cam_id, collections.deque(maxlen=self._RING))
                ring.append((fid, image, time.monotonic(), stamp_ns))
            del buf[:end]
