"""
ROS 2 lifecycle node for the UnrealLidarCam lidar.

Serves the UnrealLidarCam plugin: a lidar with a configurable horizontal and
vertical aperture (GPU depth captures or line traces), mounted on a camera or
anywhere on the rover, that streams whole scans over TCP (`port`, default 3007).

Each scan is published as an organized sensor_msgs/PointCloud2 on `topic`
(default lidar/points, frame `frame_id`, default lidar): height = channels,
width = beams per channel, fields x, y, z, intensity (float32), ring, column
(uint16), in the sensor frame with the ROS convention (x forward, y left, z up),
NaN where a beam returns nothing. The wire layout of a point is the PointCloud2
layout, so a scan is republished without per-point work.

A daemon thread runs the TCP server (blocking I/O off the executor) and
re-accepts after every Unreal Play/Stop.
"""

import socket
import struct
import threading
from typing import Optional

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn

from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import PointCloud2, PointField


class LidarCamReceiver:
    """
    TCP server for the UnrealLidarCam plugin: one packet per scan, a 48-byte
    header then point_count x 20-byte points (the plugin's docs/wire-format.md).
    Calls `on_scan(header, payload)` for every complete scan. Re-accepts after
    each Play/Stop; runs on a daemon thread.
    """

    HEADER = struct.Struct('<4sBBHQdffIIII')
    MAGIC = b'LIDR'
    POINT_STEP = 20

    def __init__(self, bind_ip: str, port: int, on_scan, logger):
        self._addr = (bind_ip, port)
        self._on_scan = on_scan
        self._log = logger
        self._stop = threading.Event()
        self._server: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(self._addr)
        self._server.listen(1)
        self._server.settimeout(0.5)
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._log.info(f'UnrealLidarCam: listening on {self._addr[0]}:{self._addr[1]}.')

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._server:
            self._server.close()
            self._server = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self._log.info(f'UnrealLidarCam: Unreal connected from {addr[0]}:{addr[1]}.')
            try:
                self._read(conn)
            finally:
                conn.close()
                if not self._stop.is_set():
                    self._log.warn('UnrealLidarCam: Unreal disconnected; waiting for the next Play.')

    def _read(self, conn: socket.socket) -> None:
        conn.settimeout(0.5)
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
            while True:
                start = buf.find(self.MAGIC)
                if start < 0:
                    del buf[:-3]                    # Keep a possible partial magic
                    break
                if start:
                    del buf[:start]                 # Resync past garbage
                if len(buf) < self.HEADER.size:
                    break
                header = self.HEADER.unpack_from(buf)
                (_, version, _, _, _, _, _, _, cols, rows, count, step) = header
                if version != 1 or step != self.POINT_STEP or count != rows * cols or count == 0:
                    self._log.warn(f'UnrealLidarCam: bad header (v{version}, {cols}x{rows}, '
                                   f'{count} points, step {step}); resyncing.',
                                   throttle_duration_sec=10.0)
                    del buf[:4]
                    continue
                end = self.HEADER.size + count * step
                if len(buf) < end:
                    break                           # Scan still in flight
                payload = bytes(buf[self.HEADER.size:end])
                del buf[:end]
                try:
                    self._on_scan(header, payload)
                except Exception as exc:            # Never let a bad scan kill the link
                    self._log.error(f'UnrealLidarCam: publishing a scan failed: {exc}',
                                    throttle_duration_sec=10.0)


class LifecycleUnrealLidarCam(rclpy.lifecycle.Node):
    """Lifecycle node: UnrealLidarCam scans over TCP -> PointCloud2."""

    FIELDS = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
        PointField(name='column', offset=18, datatype=PointField.UINT16, count=1),
    ]

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self._pub_cloud: Optional[Publisher] = None
        self._receiver: Optional[LidarCamReceiver] = None
        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('port', 3007)
        self.declare_parameter('topic', 'lidar/points')
        self.declare_parameter('frame_id', 'lidar')

    def _publish_scan(self, header: tuple, payload: bytes) -> None:
        """One scan -> PointCloud2 (organized when the plugin says so)."""
        if not self._pub_cloud or not self._pub_cloud.is_activated:
            return
        (_, _, _, flags, _, stamp, _, _, cols, rows, count, step) = header
        msg = PointCloud2()
        # Scan time from Unreal (simulation clock, s since the Unix epoch); a
        # zero stamp (older plugin) is stamped on arrival.
        msg.header.stamp = (RosTime(sec=int(stamp), nanosec=int(round((stamp % 1.0) * 1e9)) % 1_000_000_000)
                            if stamp > 0 else self.get_clock().now().to_msg())
        msg.header.frame_id = self.frame_id
        if flags & 1:
            msg.height, msg.width = rows, cols
        else:
            msg.height, msg.width = 1, count
        msg.fields = self.FIELDS
        msg.is_bigendian = False
        msg.point_step = step
        msg.row_step = step * msg.width
        msg.is_dense = False                        # NaN where a beam has no return
        msg.data = payload
        self._pub_cloud.publish(msg)

    # -------------------------------------------------------------------------
    # Lifecycle transitions
    # -------------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Executing Lifecycle Transition: CONFIGURED.')
        self.frame_id = self.get_parameter('frame_id').value
        self._pub_cloud = self.create_lifecycle_publisher(
            PointCloud2, self.get_parameter('topic').value, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Executing Lifecycle Transition: ACTIVE.')
        if self._receiver is None:
            try:
                self._receiver = LidarCamReceiver(
                    self.get_parameter('bind_ip').value, self.get_parameter('port').value,
                    self._publish_scan, self.get_logger())
                self._receiver.start()
            except OSError as exc:
                self.get_logger().error(f'UnrealLidarCam: cannot listen ({exc}).')
                self._receiver = None
                return TransitionCallbackReturn.FAILURE
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Executing Lifecycle Transition: DEACTIVATED.')
        if self._receiver:
            self._receiver.stop()
            self._receiver = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Executing Lifecycle Transition: CLEANING UP.')
        if self._receiver:
            self._receiver.stop()
            self._receiver = None
        if self._pub_cloud:
            self.destroy_publisher(self._pub_cloud)
            self._pub_cloud = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Executing Lifecycle Transition: SHUTTING DOWN.')
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()
    # Single-threaded is enough: the blocking socket I/O runs on its own thread.
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LifecycleUnrealLidarCam('lc_unreal_lidar')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
