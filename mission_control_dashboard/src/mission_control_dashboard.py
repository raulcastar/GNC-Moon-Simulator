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
import math
import time
import threading
import base64
import warnings

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
from std_msgs.msg import Bool, Empty, Float32MultiArray
from rcl_interfaces.msg import Log
from lifecycle_msgs.srv import ChangeState
from cv_bridge import CvBridge

from custom_map_interfaces.srv import GetMatrices

from flask import Flask, render_template, request, send_from_directory
from flask_socketio import SocketIO

pkg_share = get_package_share_directory('mission_control_dashboard')
template_dir = os.path.join(pkg_share, 'templates')

# Four independent web servers, one ROS node behind them all:
#   5000  hub      — a menu that links to the other screens
#   5001  overview — the full GNC mission-control dashboard
#   5002  teleop   — manual drive-by-wire, bypassing the GNC planner
#   5003  cameras  — a camera wall with per-feed show/hide toggles
#   5005  map3d    — LiDAR 3D + local DEM side by side, with labelled axes
# network_mode: host means each port is reachable on the host directly, so no
# compose port mapping is needed. Each Flask app gets its own SocketIO server;
# the ROS node fans shared feeds (cameras, telemetry) out to whichever servers
# need them via emit_over() / emit_all() / emit_feeds().

app = Flask('overview', template_folder=template_dir)
app.config['SECRET_KEY'] = 'lunar_mission_secure'
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*",
                    max_http_buffer_size=10000000)

app_hub = Flask('hub', template_folder=template_dir)
app_hub.config['SECRET_KEY'] = 'lunar_hub'
sio_hub = SocketIO(app_hub, async_mode='threading', cors_allowed_origins="*")

app_teleop = Flask('teleop', template_folder=template_dir)
app_teleop.config['SECRET_KEY'] = 'lunar_teleop'
sio_teleop = SocketIO(app_teleop, async_mode='threading', cors_allowed_origins="*",
                      max_http_buffer_size=10000000)

app_cams = Flask('cameras', template_folder=template_dir)
app_cams.config['SECRET_KEY'] = 'lunar_cameras'
sio_cams = SocketIO(app_cams, async_mode='threading', cors_allowed_origins="*",
                    max_http_buffer_size=10000000)

app_map = Flask('map3d', template_folder=template_dir)
app_map.config['SECRET_KEY'] = 'lunar_map3d'
sio_map = SocketIO(app_map, async_mode='threading', cors_allowed_origins="*",
                   max_http_buffer_size=10000000)


def emit_over(event, data):
    """Emit only to the overview dashboard (5001)."""
    socketio.emit(event, data)


def emit_all(event, data):
    """Emit telemetry/log to the screens that show it (overview + teleop)."""
    socketio.emit(event, data)
    sio_teleop.emit(event, data)


def emit_map(event, data):
    """Fan a spatial feed (lidar / DEM / pose) out to the overview and the
    dedicated 3D map window (5005)."""
    socketio.emit(event, data)
    sio_map.emit(event, data)


def emit_feeds(event, data, teleop=True):
    """Fan a camera feed out to the screens that show it: overview and the
    camera wall always; teleop too for the driving-relevant views."""
    socketio.emit(event, data)
    sio_cams.emit(event, data)
    if teleop:
        sio_teleop.emit(event, data)


def _ground_plane(dem: np.ndarray, res: float) -> np.ndarray:
    """
    Robust plane fit z = a*x + b*y + c to a rover-aligned DEM (rows forward,
    columns lateral), iteratively excluding cells above the fit (rocks).
    Returns (a, b, c), or zeros when too few cells are observed.
    """
    rows, cols = np.nonzero(np.isfinite(dem))
    if rows.size < 50:
        return np.zeros(3)
    x = (cols - (dem.shape[1] - 1) / 2.0) * res
    y = (rows + 0.5) * res
    z = dem[rows, cols].astype(np.float64)
    keep = np.ones(z.size, dtype=bool)
    coef = np.zeros(3)
    for _ in range(3):
        A = np.c_[x[keep], y[keep], np.ones(keep.sum())]
        coef, *_ = np.linalg.lstsq(A, z[keep], rcond=None)
        resid = z - (coef[0] * x + coef[1] * y + coef[2])
        keep = resid < 0.05
        if keep.sum() < 50:
            break
    return coef


def _local_dem_js():
    """Serves the local DEM view shared by the overview and 3D map pages."""
    return send_from_directory(template_dir, 'local_dem.js', mimetype='application/javascript')


for _app in (app, app_map):
    _app.add_url_rule('/local_dem.js', 'local_dem_js', _local_dem_js)


ros_node = None

# /rosout severity bytes -> console labels
_LOG_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

# Push rates deliberately low: the browser renders the lidar and DEM in WebGL,
# which competes with Unreal for the same (laptop) GPU. Higher rates here buy
# nothing the operator can perceive and risk stalling the desktop compositor.
_MAX_LIDAR_POINTS = 9000       # per frame pushed to the browser
_LIDAR_MIN_PERIOD = 1.0        # seconds between lidar frames
_ODOM_MIN_PERIOD = 0.15        # seconds between telemetry emits
_GRID_MIN_PERIOD = 1.0         # seconds between local DEM frames
_DEM_MAX_CELLS = 160           # max DEM cells per side sent to the browser
# The driven-trajectory Path grows without bound as the rover moves. Relaying
# every pose meant an ever-larger payload each update, and the browser re-drew
# the whole polyline on every odom tick — O(N) with N climbing, which is what
# made the machine crawl over a long run. Decimate to a fixed budget (the trail
# shape is preserved) and throttle. Bounds payload and client redraw cost.
_TRAJ_MAX_POINTS = 1500        # points per driven-path push (decimated)
_TRAJ_MIN_PERIOD = 0.5         # seconds between driven-path frames
# The event and SPAD cameras publish at ~30 Hz. Relaying every frame to the
# browser as a base64 JPEG floods the websocket and stalls the UI, so cap their
# browser-facing rate. 0.16 s ~= 6 fps, plenty for these visualisations.
_CAM_PRODUCT_MIN_PERIOD = 0.16


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
        self._pub_cancel = None
        self._cli_get_maps = None
        self._cli_stereo_lc = None
        self._restart_busy = False

        self._last_lidar = 0.0
        self._last_odom_emit = 0.0
        self._last_grid = 0.0
        self._last_traj = 0.0
        self._prev_odom = None      # (t, x, y) for speed estimation

        # Manual teleop (5002). The rover holds its last cmd_vel, so a dropped
        # browser or a stuck key must not leave it driving: engage on any
        # non-zero command and let a watchdog zero it if commands stop arriving.
        self._teleop_watchdog = None
        self._last_teleop = 0.0
        self._teleop_engaged = False

        self.declare_parameter('global_res', 1.0)
        self.declare_parameter('camera_namespace', 'navcam')
        # Relief (m) shown as an obstacle in the local DEM view. Display only.
        self.declare_parameter('dem_obstacle_height', 0.15)
        self._dem_res = 0.05        # local DEM cell size, updated from the grid
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
        self._pub_cancel = self.create_lifecycle_publisher(Empty, 'cancel_goal', 10)
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
            self._make_image_relay('depth', both=True), qos_profile_sensor_data))
        self._subs.append(sub(
            Image, f'{ns}/central/object_mask',
            self._make_image_relay('mask'), qos_profile_sensor_data))

        # Event (DVS) and SPAD cameras publish at ~30 Hz; throttle their
        # browser-facing rate so the websocket / UI does not stall (~6 fps).
        self._subs.append(sub(
            Image, 'event_camera/image_events',
            self._make_image_relay('events', min_period=_CAM_PRODUCT_MIN_PERIOD),
            qos_profile_sensor_data))

        # SPAD camera digitised reconstruction from component_spad_camera.
        self._subs.append(sub(
            Image, 'spad/image',
            self._make_image_relay('spad', min_period=_CAM_PRODUCT_MIN_PERIOD),
            qos_profile_sensor_data))

        # 5 Hz teleop deadman: stop the rover if manual commands stop arriving.
        self._teleop_watchdog = self.create_timer(0.2, self._teleop_watchdog_cb)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: mission control ACTIVE.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: mission control suspended.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._teleop_watchdog is not None:
            self.destroy_timer(self._teleop_watchdog)
            self._teleop_watchdog = None
        for s in self._subs:
            self.destroy_subscription(s)
        self._subs = []
        for p in (self._pub_goal, self._pub_lights, self._pub_cmdvel, self._pub_cancel):
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
            emit_feeds('feed', {
                'id': feed_id,
                'image': base64.b64encode(bytes(msg.data)).decode('ascii')})
        return cb

    def _make_image_relay(self, feed_id: str, both: bool = False,
                          min_period: float = 0.0):
        """Relay a raw colour Image, JPEG-encoding it first.

        Always goes to the overview and the camera wall (5003). both=True also
        mirrors it to the teleop screen (used for the stereo depth view).
        min_period caps the browser-facing rate (seconds between frames): the
        event and SPAD cameras publish at ~30 Hz, and pushing every one as a
        base64 JPEG over the websocket floods the browser and stalls the UI.
        The throttle is checked before the JPEG encode, so it also saves CPU.
        """
        def emit(event, data):
            emit_feeds(event, data, teleop=both)

        last = [0.0]        # mutable cell for the per-feed last-emit time

        def cb(msg: Image):
            if not self._is_active():
                return
            if min_period > 0.0:
                now = time.monotonic()
                if now - last[0] < min_period:
                    return
                last[0] = now
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                ok, buf = cv2.imencode('.jpg', img,
                                       [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    emit('feed', {
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
        emit_map('lidar', {
            'points': np.round(xyz, 2).ravel().tolist(),
            'count': int(len(xyz)),
            'frame': msg.header.frame_id,
        })

    # =========================================================================
    # Local DEM
    # =========================================================================

    def raw_dem_callback(self, msg: Float32MultiArray) -> None:
        """
        Relays the local DEM to the browser as 'local_dem'.

        Input: DEM[x, y] in metres above the ground under the rover (x lateral,
        right +; y forward from the camera plane; NaN = unobserved). Output:
        centimetres, row-major by forward row, None for unobserved cells, since
        NaN is not valid JSON.
        """
        if not self._is_active() or len(msg.layout.dim) != 2:
            return
        now = time.monotonic()
        if now - self._last_grid < _GRID_MIN_PERIOD:
            return
        self._last_grid = now

        W, H = msg.layout.dim[0].size, msg.layout.dim[1].size
        dem = np.asarray(msg.data, dtype=np.float32).reshape(W, H).T   # (forward, lateral)
        res = self._dem_res

        # Bound the browser mesh; max-pool so obstacles survive the reduction.
        f = int(math.ceil(max(W, H) / _DEM_MAX_CELLS))
        if f > 1:
            h2, w2 = -(-dem.shape[0] // f), -(-dem.shape[1] // f)
            padded = np.full((h2 * f, w2 * f), np.nan, dtype=np.float32)
            padded[:dem.shape[0], :dem.shape[1]] = dem
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)   # all-NaN blocks
                dem = np.nanmax(padded.reshape(h2, f, w2, f), axis=(1, 3))
            res *= f

        known = np.isfinite(dem)
        threshold = self.get_parameter('dem_obstacle_height').value
        plane = _ground_plane(dem, res)
        rows, cols = np.indices(dem.shape)
        relief = dem - (plane[0] * (cols - (dem.shape[1] - 1) / 2.0) * res
                        + plane[1] * (rows + 0.5) * res + plane[2])
        z_cm = np.where(known, np.round(dem * 100.0), 0).astype(np.int32)
        emit_map('local_dem', {
            'w': int(dem.shape[1]),
            'h': int(dem.shape[0]),
            'res': round(float(res), 4),
            'z': [int(v) if k else None for v, k in zip(z_cm.ravel(), known.ravel())],
            # Local ground z = a*x + b*y + c (x right, y forward, m); the browser
            # colours by relief above it so slopes are not shown as obstacles.
            'plane': [round(float(v), 5) for v in plane],
            'observed': round(float(known.mean()), 3),
            'max_relief_m': round(float(relief[known].max()), 3) if known.any() else None,
            'obstacles': int((relief[known] >= threshold).sum()),
            'obstacle_m': float(threshold),
        })

    def local_grid_callback(self, msg: OccupancyGrid) -> None:
        """Tracks the DEM cell size; the grid itself is not relayed."""
        self._dem_res = float(msg.info.resolution)

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

        odom_payload = {
            'x': round(p.x, 3), 'y': round(p.y, 3), 'z': round(p.z, 3),
            'yaw': round(float(yaw), 1), 'speed': round(speed, 3),
        }
        emit_all('odom_update', odom_payload)
        sio_map.emit('odom_update', odom_payload)

    def global_path_callback(self, msg: Path) -> None:
        if not self._is_active():
            return
        coords = [{'x': p.pose.position.x, 'y': p.pose.position.y}
                  for p in msg.poses]
        socketio.emit('global_path_update', {'path': coords})

    def trajectory_callback(self, msg: Path) -> None:
        if not self._is_active():
            return
        # An empty path is a clear signal (session reset / Unreal Play-Stop):
        # always forward it so the browser wipes the old trail, bypassing the
        # rate limit that would otherwise drop it.
        if len(msg.poses) == 0:
            socketio.emit('trajectory_update', {'path': []})
            return
        now = time.monotonic()
        if now - self._last_traj < _TRAJ_MIN_PERIOD:
            return
        self._last_traj = now
        # Decimate the (unbounded, ever-growing) driven path to a fixed budget:
        # stride keeps the whole trail's shape while capping payload and the
        # browser's per-redraw cost. The last pose is always included so the
        # trail reaches the rover's current position.
        poses = msg.poses
        n = len(poses)
        step = max(1, (n + _TRAJ_MAX_POINTS - 1) // _TRAJ_MAX_POINTS)
        idx = list(range(0, n, step))
        if idx and idx[-1] != n - 1:
            idx.append(n - 1)
        coords = [{'x': poses[i].pose.position.x, 'y': poses[i].pose.position.y}
                  for i in idx]
        socketio.emit('trajectory_update', {'path': coords})

    def rosout_callback(self, msg: Log) -> None:
        if not self._is_active():
            return
        if msg.level < 20:                      # drop DEBUG chatter
            return
        # Suppress the stereo node's per-frame coverage report ("matched X% of
        # the left frame | Y% of the central frame received depth"): it fires
        # every frame and floods the operator console. Filtered here so it still
        # appears in the raw ROS log, just not on any dashboard.
        if 'received depth' in msg.msg and 'matched' in msg.msg:
            return
        emit_all('log', {
            'level': _LOG_LEVELS.get(msg.level, str(msg.level)),
            'node': msg.name,
            'text': msg.msg,
            'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
        })

    # =========================================================================
    # Map service + goal dispatch
    # =========================================================================

    def fetch_maps(self, sid=None) -> None:
        """Request the elevation map and send it to `sid` (all clients if None)."""
        if self._cli_get_maps is None:
            return
        if not self._cli_get_maps.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Planner map service is not available yet.")
            socketio.emit('log', {'level': 'WARN', 'node': 'dashboard',
                                  'text': 'Planner map service unavailable.',
                                  'stamp': time.time()}, to=sid)
            return
        self.get_logger().info("Requesting map from planner...")
        future = self._cli_get_maps.call_async(GetMatrices.Request())
        future.add_done_callback(lambda f: self.map_response_callback(f, sid))

    def map_response_callback(self, future, sid=None) -> None:
        try:
            response = future.result()
            # Quantise to uint16 over the valid range and send as base64: the
            # JSON float list for a 1000x1000 map is ~19 MB and stalls the
            # socket. 65535 marks no-data (planner sentinel >= 9999).
            elev = np.asarray(response.elevation_map.data, dtype=np.float32)
            valid = np.isfinite(elev) & (elev < 9999.0)
            lo = float(elev[valid].min()) if valid.any() else 0.0
            hi = float(elev[valid].max()) if valid.any() else 0.0
            scale = (hi - lo) / 65534.0 if hi > lo else 1.0
            q = np.full(elev.shape, 65535, dtype='<u2')
            q[valid] = np.round((elev[valid] - lo) / scale).astype('<u2')
            socketio.emit('map_update', {
                'data_u16': base64.b64encode(q.tobytes()).decode('ascii'),
                'lo': lo,
                'scale': scale,
                'width': response.elevation_map.layout.dim[1].size,
                'height': response.elevation_map.layout.dim[0].size,
                'resolution': self.map_resolution,
            }, to=sid)
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
        """
        Cancels the planner's goal, which would otherwise override the stop on its
        next cycle, then sends a zero twist three times (the rover holds the last
        command).
        """
        self._teleop_engaged = False
        if self._pub_cancel and self._pub_cancel.is_activated:
            self._pub_cancel.publish(Empty())
        if self._pub_cmdvel and self._pub_cmdvel.is_activated:
            for _ in range(3):
                self._pub_cmdvel.publish(Twist())
            self.get_logger().warn("ALL STOP commanded from the console.")

    def teleop_drive(self, lin: float, ang: float) -> None:
        """
        Publish a manual cmd_vel from the teleop screen (5002).

        This shares the cmd_vel bus with the GNC planner, which only commands
        while it is chasing a goal — so manual driving is unobstructed as long as
        no goal is active. Engaging arms the watchdog so a dropped connection
        cannot leave the rover running.
        """
        if not (self._pub_cmdvel and self._pub_cmdvel.is_activated):
            return
        lin = float(lin)
        ang = float(ang)
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self._pub_cmdvel.publish(t)
        self._last_teleop = time.monotonic()
        self._teleop_engaged = self._teleop_engaged or lin != 0.0 or ang != 0.0

    def _teleop_watchdog_cb(self) -> None:
        """Zero the rover if a teleop session goes quiet (dropped tab, stuck key)."""
        if not self._teleop_engaged:
            return
        if time.monotonic() - self._last_teleop <= 0.6:
            return
        self._teleop_engaged = False
        if self._pub_cmdvel and self._pub_cmdvel.is_activated:
            self._pub_cmdvel.publish(Twist())
            self.get_logger().warn("Teleop watchdog: manual commands stopped — "
                                   "rover halted.")

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
# Flask routing — hub (5000)
# =========================================================================

@app_hub.route('/')
def hub():
    return render_template('hub.html')


# =========================================================================
# Flask routing — teleop (5002)
# =========================================================================

@app_teleop.route('/')
def teleop():
    return render_template('teleop.html')


@sio_teleop.on('teleop_cmd')
def handle_teleop_cmd(json_data):
    if ros_node is not None:
        ros_node.teleop_drive(json_data.get('lin', 0.0), json_data.get('ang', 0.0))


@sio_teleop.on('headlights')
def handle_teleop_headlights(json_data):
    if ros_node is not None:
        ros_node.set_headlights(json_data.get('on', False))


@sio_teleop.on('stop_rover')
def handle_teleop_stop(json_data=None):
    if ros_node is not None:
        ros_node.stop_rover()


# =========================================================================
# Flask routing — cameras (5003)
# =========================================================================

@app_cams.route('/')
def cameras():
    return render_template('cameras.html')


# =========================================================================
# Flask routing — map3d (5004): LiDAR 3D + local DEM
# =========================================================================

@app_map.route('/')
def map3d():
    return render_template('map3d.html')


# =========================================================================
# Flask routing — overview (5001)
# =========================================================================

@app.route('/')
def index():
    return render_template('index.html')


@socketio.on('connect')
def handle_connect(auth=None):
    if ros_node is not None:
        ros_node.get_logger().info("Operator console connected.")
        ros_node.fetch_maps(request.sid)


@socketio.on('send_goal')
def handle_goal(json_data):
    if ros_node is not None:
        ros_node.dispatch_goal(json_data['x'], json_data['y'])


@socketio.on('refresh_map')
def handle_refresh(json_data=None):
    if ros_node is not None:
        ros_node.fetch_maps(request.sid)


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

    # Hub (5000), teleop (5002), cameras (5003) and map3d (5005) run in background
    # threads; the overview (5001) holds the main thread. All share ros_node via
    # the globals.
    #
    # The map window uses 5005, not the sequential 5004: under this machine's WSL2
    # mirrored networking, port 5004 specifically would not mirror to the Windows
    # host — a listener bound there was live inside the container's netns but
    # localhost:5004 refused from both Windows and WSL loopback, while every
    # neighbouring port (5005, 5006, 5010) mirrored instantly. Most likely a stale
    # Windows-side reservation on 5004. 5005 sidesteps it.
    def _serve(sio, flask_app, port):
        sio.run(flask_app, host='0.0.0.0', port=port, allow_unsafe_werkzeug=True)

    threading.Thread(target=_serve, args=(sio_hub, app_hub, 5000),
                     daemon=True).start()
    threading.Thread(target=_serve, args=(sio_teleop, app_teleop, 5002),
                     daemon=True).start()
    threading.Thread(target=_serve, args=(sio_cams, app_cams, 5003),
                     daemon=True).start()
    threading.Thread(target=_serve, args=(sio_map, app_map, 5005),
                     daemon=True).start()
    ros_node.get_logger().info(
        "Mission control up: hub :5000  overview :5001  teleop :5002  "
        "cameras :5003  map3d :5005")

    try:
        socketio.run(app, host='0.0.0.0', port=5001, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
