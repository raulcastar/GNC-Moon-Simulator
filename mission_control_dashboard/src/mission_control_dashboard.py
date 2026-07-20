"""
ROS 2 Lifecycle Web Mission Control.

Flask + SocketIO server presenting the whole stack in one browser page:
every navcam feed, stereo depth, object segmentation, a live lidar sweep,
the elevation map with click-to-goal, odometry telemetry, and the ROS log
stream (/rosout) as a mission console.

All imagery is re-encoded to JPEG and pushed over the websocket; the browser
renders lidar and map client-side on canvases.
"""

import os
import json
import time
import threading
import base64

import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import Bool, Float32MultiArray
from rcl_interfaces.msg import Log
from lifecycle_msgs.srv import ChangeState
from cv_bridge import CvBridge

from custom_map_interfaces.srv import GetMatrices

from flask import Flask, render_template
from flask_socketio import SocketIO

pkg_share = get_package_share_directory('mission_control_dashboard')
template_dir = os.path.join(pkg_share, 'templates')

app = Flask(__name__, template_folder=template_dir)
app.config['SECRET_KEY'] = 'lunar_mission_secure'

socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*",
                    max_http_buffer_size=10000000)

ros_node = None

# /rosout severity bytes -> console labels
_LOG_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

# Push rates deliberately low: the browser renders the lidar and DEM in WebGL,
# which competes with Unreal for the same (laptop) GPU. Higher rates here buy
# nothing the operator can perceive and risk stalling the desktop compositor.
_MAX_LIDAR_POINTS = 4000       # per frame pushed to the browser
_LIDAR_MIN_PERIOD = 1.0        # seconds between lidar frames
_ODOM_MIN_PERIOD = 0.15        # seconds between telemetry emits
_GRID_MIN_PERIOD = 2.0         # seconds between local DEM frames


def _cloud_xyz(msg: PointCloud2, max_points: int) -> np.ndarray:
    """Extract finite XYZ from a PointCloud2, strided down to max_points."""
    offsets = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0 or not all(k in offsets for k in ('x', 'y', 'z')):
        return np.empty((0, 3), np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    raw = raw[:n * msg.point_step].reshape(n, msg.point_step)

    def col(name):
        off = offsets[name]
        return raw[:, off:off + 4].copy().view(np.float32).ravel()

    xyz = np.column_stack((col('x'), col('y'), col('z')))
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) > max_points:
        xyz = xyz[::max(1, len(xyz) // max_points)][:max_points]
    return xyz


class LifecycleMissionControlUI(rclpy.lifecycle.Node):
    """Bridges the ROS graph to the browser over SocketIO."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self.bridge = CvBridge()
        self._subs = []
        self._pub_goal = None
        self._pub_lights = None
        self._pub_cmdvel = None
        self._cli_get_maps = None
        self._cli_stereo_lc = None
        self._restart_busy = False
        self._raw_dem = None        # (W, H, np.ndarray metres) awaiting its grid

        self._last_lidar = 0.0
        self._last_odom_emit = 0.0
        self._last_grid = 0.0
        self._prev_odom = None      # (t, x, y) for speed estimation

        self.declare_parameter('global_res', 1.0)
        self.declare_parameter('camera_namespace', 'navcam')
        self.map_resolution = self.get_parameter('global_res').value

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: configuring mission control interfaces.")
        ns = self.get_parameter('camera_namespace').value

        self._pub_goal = self.create_lifecycle_publisher(PoseStamped, 'goal_pose', 10)
        self._pub_lights = self.create_lifecycle_publisher(Bool, 'headlights_cmd', 10)
        self._pub_cmdvel = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)
        self._cli_get_maps = self.create_client(GetMatrices, 'get_matrices')
        self._cli_stereo_lc = self.create_client(
            ChangeState, '/lc_component_stereo_depth/change_state')

        sub = self.create_subscription
        self._subs = [
            sub(Odometry, 'odom', self.odom_callback, 10),
            sub(Path, 'global_path', self.global_path_callback, 10),
            sub(Path, 'driven_trajectory', self.trajectory_callback, 10),
            sub(Log, '/rosout', self.rosout_callback, 50),
            sub(PointCloud2, 'velodyne_points', self.lidar_callback,
                qos_profile_sensor_data),
            sub(OccupancyGrid, 'local_mapping/elevation_grid',
                self.local_grid_callback, 10),
            sub(Float32MultiArray, 'local_mapping/elevation_map_raw',
                self.raw_dem_callback, 10),
        ]

        # Compressed colour passes straight through: it is already JPEG.
        for cam in ('left', 'right', 'central'):
            self._subs.append(sub(
                CompressedImage, f'{ns}/{cam}/image_color/compressed',
                self._make_jpeg_relay(f'cam_{cam}'), qos_profile_sensor_data))

        # Already-colourised products only need JPEG encoding.
        self._subs.append(sub(
            Image, f'{ns}/central/image_depth/visualization',
            self._make_image_relay('depth'), qos_profile_sensor_data))
        self._subs.append(sub(
            Image, f'{ns}/central/object_mask',
            self._make_image_relay('mask'), qos_profile_sensor_data))

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: mission control ACTIVE.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: mission control suspended.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        for s in self._subs:
            self.destroy_subscription(s)
        self._subs = []
        for p in (self._pub_goal, self._pub_lights, self._pub_cmdvel):
            if p:
                self.destroy_publisher(p)
        for c in (self._cli_get_maps, self._cli_stereo_lc):
            if c:
                self.destroy_client(c)
        return TransitionCallbackReturn.SUCCESS

    def _is_active(self) -> bool:
        return self._state_machine.current_state[1] == 'active'

    # =========================================================================
    # Imagery
    # =========================================================================

    def _make_jpeg_relay(self, feed_id: str):
        """Relay a CompressedImage (already JPEG) to the browser untouched."""
        def cb(msg: CompressedImage):
            if not self._is_active():
                return
            socketio.emit('feed', {
                'id': feed_id,
                'image': base64.b64encode(bytes(msg.data)).decode('ascii')})
        return cb

    def _make_image_relay(self, feed_id: str):
        """Relay a raw colour Image, JPEG-encoding it first."""
        def cb(msg: Image):
            if not self._is_active():
                return
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                ok, buf = cv2.imencode('.jpg', img,
                                       [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    socketio.emit('feed', {
                        'id': feed_id,
                        'image': base64.b64encode(buf).decode('ascii')})
            except Exception as e:
                self.get_logger().warn(f"{feed_id} relay fault: {e}",
                                       throttle_duration_sec=10.0)
        return cb

    # =========================================================================
    # Lidar
    # =========================================================================

    def lidar_callback(self, msg: PointCloud2) -> None:
        if not self._is_active():
            return
        now = time.monotonic()
        if now - self._last_lidar < _LIDAR_MIN_PERIOD:
            return
        self._last_lidar = now

        xyz = _cloud_xyz(msg, _MAX_LIDAR_POINTS)
        if len(xyz) == 0:
            return
        socketio.emit('lidar', {
            'points': np.round(xyz, 2).ravel().tolist(),
            'count': int(len(xyz)),
            'frame': msg.header.frame_id,
        })

    # =========================================================================
    # Local DEM
    # =========================================================================

    def raw_dem_callback(self, msg: Float32MultiArray) -> None:
        """
        Cache the raw DEM (metres). local_mapping publishes it immediately
        before the matching OccupancyGrid, whose values are normalised 0-100;
        joining the two recovers true elevations for the 3D terrain.
        """
        if len(msg.layout.dim) == 2:
            self._raw_dem = (msg.layout.dim[0].size, msg.layout.dim[1].size,
                             np.asarray(msg.data, dtype=np.float32))

    def local_grid_callback(self, msg: OccupancyGrid) -> None:
        if not self._is_active():
            return
        now = time.monotonic()
        if now - self._last_grid < _GRID_MIN_PERIOD:
            return
        self._last_grid = now

        data = np.asarray(msg.data, dtype=np.int8)

        # True height span from the raw DEM. Raw is DEM[x, y] flattened
        # (index x*H + y); the grid is data[y*W + x] — transpose to align.
        hmin = hmax = 0.0
        if (self._raw_dem is not None
                and self._raw_dem[0] == msg.info.width
                and self._raw_dem[1] == msg.info.height):
            W, H, raw = self._raw_dem
            aligned = raw.reshape(W, H).T.ravel()
            known = aligned[data >= 0]
            if known.size:
                hmin, hmax = float(known.min()), float(known.max())

        socketio.emit('local_grid', {
            'w': msg.info.width,
            'h': msg.info.height,
            'res': round(msg.info.resolution, 4),
            'ox': round(msg.info.origin.position.x, 3),
            'oy': round(msg.info.origin.position.y, 3),
            'hmin': round(hmin, 3),
            'hmax': round(hmax, 3),
            'data': data.tolist(),           # int8: -1 unknown, 0-100 elevation
        })

    # =========================================================================
    # Telemetry, paths, logs
    # =========================================================================

    def odom_callback(self, msg: Odometry) -> None:
        if not self._is_active():
            return
        now = time.monotonic()
        if now - self._last_odom_emit < _ODOM_MIN_PERIOD:
            return
        self._last_odom_emit = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = np.degrees(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

        speed = 0.0
        if self._prev_odom is not None:
            t0, x0, y0 = self._prev_odom
            dt = now - t0
            if dt > 1e-3:
                speed = float(np.hypot(p.x - x0, p.y - y0) / dt)
        self._prev_odom = (now, p.x, p.y)

        socketio.emit('odom_update', {
            'x': round(p.x, 3), 'y': round(p.y, 3), 'z': round(p.z, 3),
            'yaw': round(float(yaw), 1), 'speed': round(speed, 3),
        })

    def global_path_callback(self, msg: Path) -> None:
        if not self._is_active():
            return
        coords = [{'x': p.pose.position.x, 'y': p.pose.position.y}
                  for p in msg.poses]
        socketio.emit('global_path_update', {'path': coords})

    def trajectory_callback(self, msg: Path) -> None:
        if not self._is_active():
            return
        coords = [{'x': p.pose.position.x, 'y': p.pose.position.y}
                  for p in msg.poses]
        socketio.emit('trajectory_update', {'path': coords})

    def rosout_callback(self, msg: Log) -> None:
        if not self._is_active():
            return
        if msg.level < 20:                      # drop DEBUG chatter
            return
        socketio.emit('log', {
            'level': _LOG_LEVELS.get(msg.level, str(msg.level)),
            'node': msg.name,
            'text': msg.msg,
            'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
        })

    # =========================================================================
    # Map service + goal dispatch
    # =========================================================================

    def fetch_maps(self) -> None:
        if self._cli_get_maps is None:
            return
        if not self._cli_get_maps.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Planner map service is not available yet.")
            socketio.emit('log', {'level': 'WARN', 'node': 'dashboard',
                                  'text': 'Planner map service unavailable.',
                                  'stamp': time.time()})
            return
        self.get_logger().info("Requesting map from planner...")
        future = self._cli_get_maps.call_async(GetMatrices.Request())
        future.add_done_callback(self.map_response_callback)

    def map_response_callback(self, future) -> None:
        try:
            response = future.result()
            socketio.emit('map_update', {
                'data': list(response.elevation_map.data),
                'width': response.elevation_map.layout.dim[1].size,
                'height': response.elevation_map.layout.dim[0].size,
                'resolution': self.map_resolution,
            })
            self.get_logger().info("Map matrices delivered to the browser.")
        except Exception as e:
            self.get_logger().error(f"Map service call failed: {e}")

    def restart_cameras(self) -> None:
        """Cycle the stereo node's lifecycle from a background thread."""
        if self._restart_busy:
            self._console('WARN', 'Camera restart already in progress.')
            return
        self._restart_busy = True
        threading.Thread(target=self._restart_cameras_impl, daemon=True).start()

    def _console(self, level: str, text: str) -> None:
        """Log to ROS and mirror onto the operator console."""
        {'INFO': self.get_logger().info, 'WARN': self.get_logger().warn,
         'ERROR': self.get_logger().error}.get(level, self.get_logger().info)(text)
        socketio.emit('log', {'level': level, 'node': 'dashboard',
                              'text': text, 'stamp': time.time()})

    def _restart_cameras_impl(self) -> None:
        """
        deactivate -> cleanup -> configure -> activate on the stereo node.

        Equivalent to restarting its container: cleanup releases the UnrealCV
        socket and configure reconnects and re-solves the rig. The first two
        steps may legitimately fail if the node is already stuck inactive, so
        only configure and activate are load-bearing.
        """
        try:
            if not self._cli_stereo_lc.wait_for_service(timeout_sec=3.0):
                self._console('ERROR',
                              'Stereo node lifecycle service unreachable — is '
                              'component_stereo_depth running at all?')
                return
            self._console('INFO', 'Restarting the camera node...')
            steps = [(4, 'deactivate', False), (2, 'cleanup', False),
                     (1, 'configure', True), (3, 'activate', True)]
            for tid, label, required in steps:
                req = ChangeState.Request()
                req.transition.id = tid
                fut = self._cli_stereo_lc.call_async(req)
                t0 = time.monotonic()
                while not fut.done() and time.monotonic() - t0 < 90.0:
                    time.sleep(0.1)
                ok = fut.done() and fut.result() is not None and fut.result().success
                if ok:
                    self._console('INFO', f'Camera node: {label} ok.')
                elif required:
                    self._console('ERROR',
                                  f'Camera node: {label} FAILED — check the '
                                  f'stereo node log; is Unreal playing?')
                    return
                else:
                    self._console('WARN',
                                  f'Camera node: {label} skipped (already past '
                                  f'that state).')
            self._console('INFO', 'Camera node restarted; feeds should return '
                                  'within a few seconds.')
        finally:
            self._restart_busy = False

    def set_headlights(self, on: bool) -> None:
        if self._pub_lights and self._pub_lights.is_activated:
            self._pub_lights.publish(Bool(data=bool(on)))
            self.get_logger().info(
                f"Headlights commanded {'ON' if on else 'OFF'} from the console.")

    def stop_rover(self) -> None:
        """Zero twist, sent thrice for luck — the rover holds the last command."""
        if self._pub_cmdvel and self._pub_cmdvel.is_activated:
            for _ in range(3):
                self._pub_cmdvel.publish(Twist())
            self.get_logger().warn("ALL STOP commanded from the console.")

    def dispatch_goal(self, x: float, y: float) -> None:
        if self._pub_goal and self._pub_goal.is_activated:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = float(x)
            goal.pose.position.y = float(y)
            self._pub_goal.publish(goal)
            self.get_logger().info(
                f"Goal dispatched: ({x:.1f}, {y:.1f}) m.")
        else:
            self.get_logger().warn("Goal rejected: node not ACTIVE.")


# =========================================================================
# Flask routing
# =========================================================================

@app.route('/')
def index():
    return render_template('index.html')


@socketio.on('connect')
def handle_connect(auth=None):
    if ros_node is not None:
        ros_node.get_logger().info("Operator console connected.")
        ros_node.fetch_maps()


@socketio.on('send_goal')
def handle_goal(json_data):
    if ros_node is not None:
        ros_node.dispatch_goal(json_data['x'], json_data['y'])


@socketio.on('refresh_map')
def handle_refresh(json_data=None):
    if ros_node is not None:
        ros_node.fetch_maps()


@socketio.on('headlights')
def handle_headlights(json_data):
    if ros_node is not None:
        ros_node.set_headlights(json_data.get('on', False))


@socketio.on('stop_rover')
def handle_stop(json_data=None):
    if ros_node is not None:
        ros_node.stop_rover()


@socketio.on('restart_cameras')
def handle_restart_cameras(json_data=None):
    if ros_node is not None:
        ros_node.restart_cameras()


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = LifecycleMissionControlUI('mission_control_dashboard')
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
