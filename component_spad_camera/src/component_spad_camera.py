"""
ROS 2 Lifecycle Node for the simulated single-photon (SPAD) camera.

Receives 8-bit intensity frames pushed by the UnrealSPADCam plugin
(unreal/UnrealSPADCam/) over TCP, runs the spadSim photon model on each frame
(see spad_model.py — ported from spadSim, MIT, (c) 2025 David Rodriguez-Martinez),
and publishes:

  * spad/binary — the raw single-photon binary frame (mono8, 0/255)
  * spad/image  — a digitised reconstruction: the last `digitize_frames` binary
                  frames summed and scaled to grayscale, which is how a SPAD's
                  usable image is built up from sparse photon detections.

Wire format from the plugin (little-endian), matching unreal/UnrealSPADCam:

    header  'SPD0' | ver u8 | cam_id u8 | width u16 | height u16
            | t_ns u64 | payload u32 (= width*height) | pad u16    (24 bytes)
    payload width*height bytes, 8-bit grayscale intensity, row-major
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
from cv_bridge import CvBridge

from .spad_model import SPADModel

# 24-byte packet header (see module docstring).
_HEADER = struct.Struct('<4sBBHHQIH')
_MAGIC = b'SPD0'
assert _HEADER.size == 24


class FrameReceiver:
    """
    TCP server for the UnrealSPADCam plugin. Accepts one connection at a time
    and survives Play/Stop cycles by re-accepting. Keeps only the latest
    intensity frame (the model runs on a timer, not per packet).
    """

    def __init__(self, bind_ip: str, port: int, cam_id: int, logger):
        self._addr = (bind_ip, port)
        self._cam_id = cam_id
        self._log = logger

        self._lock = threading.Lock()
        self._latest = None          # (np.uint8 HxW, t_ns)
        self._seq = 0                # increments per received frame

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
        self._log.info(f'SPAD stream: listening on {self._addr[0]}:{self._addr[1]} '
                       f'for the UnrealSPADCam plugin.')

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._server:
            self._server.close()
            self._server = None

    def latest(self):
        """Return (frame, t_ns, seq) or (None, 0, seq)."""
        with self._lock:
            if self._latest is None:
                return None, 0, self._seq
            frame, t_ns = self._latest
            return frame, t_ns, self._seq

    # ------------------------------------------------------------------ #

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self._log.info(f'SPAD stream: Unreal connected from {addr[0]}:{addr[1]}.')
            self.connected = True
            try:
                self._read_stream(conn)
            finally:
                conn.close()
                self.connected = False
                if not self._stop.is_set():
                    self._log.warn('SPAD stream: Unreal disconnected; waiting for '
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
        while len(buf) >= _HEADER.size:
            magic, ver, cam_id, w, h, t_ns, payload, _pad = _HEADER.unpack_from(buf)
            if magic != _MAGIC:
                buf = buf[1:]            # resync on the next magic
                continue
            total = _HEADER.size + payload
            if len(buf) < total:
                break                    # wait for the rest of this frame
            if cam_id == self._cam_id and payload == w * h and w > 0 and h > 0:
                frame = np.frombuffer(buf[_HEADER.size:total],
                                      dtype=np.uint8).reshape(h, w).copy()
                with self._lock:
                    self._latest = (frame, t_ns)
                    self._seq += 1
            buf = buf[total:]
        return buf


class LifecycleSpadCamera(rclpy.lifecycle.Node):
    """Receives UnrealSPADCam frames, runs the spadSim model, publishes frames."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self.bridge = CvBridge()
        self._stream: Optional[FrameReceiver] = None
        self._model: Optional[SPADModel] = None
        self._timer = None
        self._is_active = False

        self._pub_binary: Optional[Publisher] = None
        self._pub_image: Optional[Publisher] = None

        # Rolling-window state (used only when digitize_mode == 'rolling').
        self._accum: deque = deque()
        self._accum_sum = None

        self.declare_parameters(
            namespace='',
            parameters=[
                ('stream_bind_ip', rclpy.Parameter.Type.STRING),
                ('stream_port', rclpy.Parameter.Type.INTEGER),
                ('cam_id', rclpy.Parameter.Type.INTEGER),
                # ── spadSim photon model ──
                ('rgb_photons', rclpy.Parameter.Type.DOUBLE),
                ('quantum_efficiency', rclpy.Parameter.Type.DOUBLE),
                ('spad_fps', rclpy.Parameter.Type.DOUBLE),
                ('rgb_fps', rclpy.Parameter.Type.DOUBLE),
                ('dcr', rclpy.Parameter.Type.DOUBLE),
                ('include_dcr', rclpy.Parameter.Type.BOOL),
                ('detection_threshold', rclpy.Parameter.Type.INTEGER),
                # ── output ──
                # How the digitised reconstruction is built from binary frames:
                #   'subframes' — each readout integrates digitize_frames fresh
                #                 exposures of the current intensity (rate-
                #                 independent; a real SPAD integrating between
                #                 readouts).
                #   'rolling'   — one binary frame per readout, summed over a
                #                 rolling window of the last digitize_frames
                #                 readouts (temporal, reacts over ~N readouts).
                ('digitize_mode', rclpy.Parameter.Type.STRING),
                # Number of binary frames summed into the digitised reconstruction
                # (per-readout exposures in 'subframes', window length in 'rolling').
                ('digitize_frames', rclpy.Parameter.Type.INTEGER),
                ('publish_rate_hz', rclpy.Parameter.Type.DOUBLE),
                ('binary_topic', rclpy.Parameter.Type.STRING),
                ('image_topic', rclpy.Parameter.Type.STRING),
                ('frame_id', rclpy.Parameter.Type.STRING),
            ])

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle: configuring SPAD camera.')

        self._frame_id = self.get_parameter('frame_id').value
        self._digitize_frames = max(1, int(self.get_parameter('digitize_frames').value))
        self._digitize_mode = str(self.get_parameter('digitize_mode').value).strip().lower()
        if self._digitize_mode not in ('subframes', 'rolling'):
            self.get_logger().warn(
                f"digitize_mode '{self._digitize_mode}' unknown; using 'subframes'.")
            self._digitize_mode = 'subframes'
        self._accum.clear()
        self._accum_sum = None
        hz = float(self.get_parameter('publish_rate_hz').value)

        self._model = SPADModel(
            rgb_photons=self.get_parameter('rgb_photons').value,
            quantum_efficiency=self.get_parameter('quantum_efficiency').value,
            spad_fps=self.get_parameter('spad_fps').value,
            rgb_fps=self.get_parameter('rgb_fps').value,
            dcr=self.get_parameter('dcr').value,
            include_dcr=self.get_parameter('include_dcr').value,
            detection_threshold=self.get_parameter('detection_threshold').value)

        self._pub_binary = self.create_lifecycle_publisher(
            Image, self.get_parameter('binary_topic').value, qos_profile_sensor_data)
        self._pub_image = self.create_lifecycle_publisher(
            Image, self.get_parameter('image_topic').value, qos_profile_sensor_data)

        self._stream = FrameReceiver(
            self.get_parameter('stream_bind_ip').value,
            self.get_parameter('stream_port').value,
            self.get_parameter('cam_id').value,
            self.get_logger())
        self._stream.start()

        self._timer = self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            f'SPAD model: rgb_photons={self._model.rgb_photons} QE={self._model.qe} '
            f'spad_fps={self._model.spad_fps} rgb_fps={self._model.rgb_fps} '
            f'exposure_ratio={self._model.t_spad / self._model.t_rgb:.4g} '
            f'| digitising {self._digitize_frames} frames.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle: SPAD camera ACTIVE.')
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle: SPAD camera deactivated.')
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
        for pub in (self._pub_binary, self._pub_image):
            if pub:
                self.destroy_publisher(pub)
        self._pub_binary = self._pub_image = None
        self._model = None

    # =========================================================================
    # Processing
    # =========================================================================

    def _tick(self) -> None:
        if not self._is_active or self._stream is None or self._model is None:
            return

        frame, t_ns, _seq = self._stream.latest()
        if frame is None:
            return

        if self._digitize_mode == 'rolling':
            # One binary frame per readout, summed over a rolling window of the
            # last digitize_frames readouts (spadSim's spad_digitization).
            bits = self._model.sample(frame)
            if self._accum_sum is None or self._accum_sum.shape != bits.shape:
                self._accum.clear()
                self._accum_sum = np.zeros(bits.shape, dtype=np.int32)
            self._accum.append(bits)
            self._accum_sum += bits
            while len(self._accum) > self._digitize_frames:
                self._accum_sum -= self._accum.popleft()
            acc, denom = self._accum_sum, len(self._accum)
        else:
            # 'subframes': integrate digitize_frames fresh exposures of the
            # current intensity per readout (a SPAD integrating between readouts).
            # Rich reconstruction even at a low, navcam-matched readout rate.
            acc, bits = self._model.sample_sum(frame, self._digitize_frames)
            denom = self._digitize_frames

        # Frame time from Unreal (simulation clock, ns); 0 from an older plugin.
        stamp = (RosTime(sec=int(t_ns // 1_000_000_000), nanosec=int(t_ns % 1_000_000_000))
                 if t_ns else self.get_clock().now().to_msg())

        # Raw binary single-photon frame (0/255 for visibility).
        bin_msg = self.bridge.cv2_to_imgmsg((bits * 255).astype(np.uint8),
                                            encoding='mono8')
        bin_msg.header.stamp = stamp
        bin_msg.header.frame_id = self._frame_id
        self._pub_binary.publish(bin_msg)

        # Digitised reconstruction: integrated photon counts, scaled to
        # full-range grayscale (spadSim's digitisation step).
        recon = np.clip(acc.astype(np.float32) * (255.0 / max(denom, 1)),
                        0, 255).astype(np.uint8)
        img_msg = self.bridge.cv2_to_imgmsg(recon, encoding='mono8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self._frame_id
        self._pub_image.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleSpadCamera('lc_component_spad_camera')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
