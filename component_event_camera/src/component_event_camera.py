"""
ROS 2 Lifecycle Node for the simulated event (DVS) camera.

Receives the event stream pushed by the UnrealEventCam plugin
(unreal/UnrealEventCam/) over TCP and turns it into something the rest of the
stack can look at: a rolling **event-frame visualisation** (colour by polarity)
on `event_camera/image_events`, plus the instantaneous event rate on
`event_camera/event_rate`.

The plugin sends one packet per rendered frame that produced events. Wire format
(little-endian), matching unreal/UnrealEventCam/README.md:

    header  'EVT0' | ver u8 | cam_id u8 | width u16 | height u16
            | t0_ns u64 | count u32 | pad u16                        (24 bytes)
    event   x u16 | y u16 | t_rel_ns u32 | pol u8                    (9 bytes each)

Events are asynchronous and have no frame; the visualisation simply accumulates
every event seen in the last `accumulation_window_s` and renders it, ON events
one colour and OFF events another, on a dark background — the standard way to
make an event stream legible.
"""

import socket
import struct
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

# 24-byte packet header and the 9-byte per-event record (see module docstring).
_HEADER = struct.Struct('<4sBBHHQIH')
_MAGIC = b'EVT0'
_EVENT_DTYPE = np.dtype([('x', '<u2'), ('y', '<u2'),
                         ('t', '<u4'), ('pol', 'u1')])
assert _HEADER.size == 24
assert _EVENT_DTYPE.itemsize == 9

# Visualisation colours (RGB). ON = brightness increase, OFF = decrease.
_COLOR_ON = (60, 140, 255)     # blue
_COLOR_OFF = (255, 70, 40)     # red
_COLOR_BG = (10, 10, 14)       # near-black


class EventStreamReceiver:
    """
    TCP server for the UnrealEventCam plugin.

    Unreal connects and streams event packets; this accepts one connection at a
    time and survives Play/Stop cycles by re-accepting. Parsed events are kept
    in a short time-ordered ring (by receive time), which the node renders on a
    timer — decoupling the wire from the publish rate.
    """

    def __init__(self, bind_ip: str, port: int, cam_id: int,
                 window_s: float, logger):
        self._addr = (bind_ip, port)
        self._cam_id = cam_id
        self._window_s = window_s
        self._log = logger

        self._lock = threading.Lock()
        self._batches: deque = deque()        # (recv_monotonic, np event array)
        self._width = 0
        self._height = 0
        self._total_events = 0                # lifetime counter, for the rate
        self.last_event_ns = 0                # newest event time (simulation clock, ns)

        self._stop = threading.Event()
        self._server: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self.connected = False

    def start(self) -> None:
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(self._addr)
        self._server.listen(1)
        self._server.settimeout(0.5)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._log.info(f'Event stream: listening on {self._addr[0]}:{self._addr[1]} '
                       f'for the UnrealEventCam plugin.')

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._server:
            self._server.close()
            self._server = None

    def size(self):
        with self._lock:
            return self._width, self._height

    def drain_window(self):
        """
        Return (events, n_recent, dt) for the current window: a single stacked
        event array within `window_s`, how many that is, and the span they cover.
        Old batches are dropped here so memory stays bounded when nobody reads.
        """
        now = time.monotonic()
        cutoff = now - self._window_s
        with self._lock:
            while self._batches and self._batches[0][0] < cutoff:
                self._batches.popleft()
            if not self._batches:
                return None, 0, self._window_s
            arrays = [b for _, b in self._batches]
            oldest = self._batches[0][0]
        events = np.concatenate(arrays) if len(arrays) > 1 else arrays[0]
        return events, len(events), max(now - oldest, 1e-3)

    # ------------------------------------------------------------------ #

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self._log.info(f'Event stream: Unreal connected from {addr[0]}:{addr[1]}.')
            self.connected = True
            try:
                self._read_stream(conn)
            finally:
                conn.close()
                self.connected = False
                if not self._stop.is_set():
                    self._log.warn('Event stream: Unreal disconnected; waiting for '
                                   'the next Play.')

    def _read_stream(self, conn: socket.socket) -> None:
        conn.settimeout(0.5)
        buf = b''
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
            buf = self._drain(buf)

    def _drain(self, buf: bytes) -> bytes:
        """Parse every complete packet in buf; return the unparsed tail."""
        while len(buf) >= _HEADER.size:
            magic, ver, cam_id, w, h, t0, count, _pad = _HEADER.unpack_from(buf)
            if magic != _MAGIC:
                # Desynced: drop one byte and resynchronise on the next magic.
                buf = buf[1:]
                continue
            total = _HEADER.size + count * _EVENT_DTYPE.itemsize
            if len(buf) < total:
                break                      # wait for the rest of this packet
            if cam_id == self._cam_id and count > 0:
                payload = buf[_HEADER.size:total]
                events = np.frombuffer(payload, dtype=_EVENT_DTYPE)
                with self._lock:
                    self._width, self._height = w, h
                    self._batches.append((time.monotonic(), events))
                    self._total_events += count
                    self.last_event_ns = max(self.last_event_ns, t0 + int(events['t'].max()))
            buf = buf[total:]
        return buf


class LifecycleEventCamera(rclpy.lifecycle.Node):
    """Receives UnrealEventCam packets and publishes an event-frame view."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self.bridge = CvBridge()
        self._stream: Optional[EventStreamReceiver] = None
        self._timer = None
        self._is_active = False

        self._pub_image: Optional[Publisher] = None
        self._pub_rate: Optional[Publisher] = None

        self._last_rate_t = 0.0
        self._last_rate_count = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('stream_bind_ip', rclpy.Parameter.Type.STRING),
                ('stream_port', rclpy.Parameter.Type.INTEGER),
                ('cam_id', rclpy.Parameter.Type.INTEGER),
                # Events accumulated into each rendered frame, seconds. Longer =
                # brighter, more persistent trails; shorter = crisper motion.
                ('accumulation_window_s', rclpy.Parameter.Type.DOUBLE),
                ('publish_rate_hz', rclpy.Parameter.Type.DOUBLE),
                ('image_topic', rclpy.Parameter.Type.STRING),
                ('frame_id', rclpy.Parameter.Type.STRING),
            ])

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle: configuring event camera.')

        self._image_topic = self.get_parameter('image_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._window_s = float(self.get_parameter('accumulation_window_s').value)
        hz = float(self.get_parameter('publish_rate_hz').value)

        self._pub_image = self.create_lifecycle_publisher(
            Image, self._image_topic, qos_profile_sensor_data)
        self._pub_rate = self.create_lifecycle_publisher(
            Float32, 'event_camera/event_rate', 10)

        self._stream = EventStreamReceiver(
            self.get_parameter('stream_bind_ip').value,
            self.get_parameter('stream_port').value,
            self.get_parameter('cam_id').value,
            self._window_s,
            self.get_logger())
        self._stream.start()

        self._last_rate_t = time.monotonic()
        self._last_rate_count = 0
        self._timer = self.create_timer(1.0 / max(hz, 1.0), self._render_and_publish)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle: event camera ACTIVE.')
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle: event camera deactivated.')
        self._is_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def _release(self) -> None:
        self._is_active = False
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._stream:
            self._stream.stop()
            self._stream = None
        for pub in (self._pub_image, self._pub_rate):
            if pub:
                self.destroy_publisher(pub)
        self._pub_image = self._pub_rate = None

    # =========================================================================
    # Rendering
    # =========================================================================

    def _render_and_publish(self) -> None:
        if not self._is_active or self._stream is None:
            return

        w, h = self._stream.size()
        if w == 0 or h == 0:
            return                          # no packets yet

        events, n_recent, dt = self._stream.drain_window()

        img = np.empty((h, w, 3), dtype=np.uint8)
        img[:] = _COLOR_BG
        if events is not None and n_recent > 0:
            xs = np.clip(events['x'].astype(np.int32), 0, w - 1)
            ys = np.clip(events['y'].astype(np.int32), 0, h - 1)
            on = events['pol'] != 0
            img[ys[on], xs[on]] = _COLOR_ON
            img[ys[~on], xs[~on]] = _COLOR_OFF

        # The window ends at the newest event: stamp it with that event's time
        # (simulation clock, ns from Unreal); arrival time if none yet.
        t_ns = self._stream.last_event_ns
        stamp = (RosTime(sec=int(t_ns // 1_000_000_000), nanosec=int(t_ns % 1_000_000_000))
                 if t_ns else self.get_clock().now().to_msg())
        msg = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        self._pub_image.publish(msg)

        # Event rate over the interval since the last publish (events/second).
        now = time.monotonic()
        total = self._stream._total_events
        elapsed = now - self._last_rate_t
        if elapsed >= 0.25:
            rate = (total - self._last_rate_count) / elapsed
            self._pub_rate.publish(Float32(data=float(rate)))
            self._last_rate_t = now
            self._last_rate_count = total


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleEventCamera('lc_component_event_camera')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
