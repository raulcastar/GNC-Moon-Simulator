"""
ROS 2 Lifecycle Web Mission Control.

Flask + SocketIO server presenting the whole stack in one browser page:
every navcam feed, stereo depth, object segmentation, a live lidar sweep,
the elevation map with click-to-goal, odometry telemetry, and the ROS log
stream (/rosout) as a mission console.

All imagery is re-encoded to JPEG and pushed over the websocket; the browser
renders lidar and map client-side on canvases.
"""

import collections
import io
import os
import json
import math
import signal
import subprocess
import zipfile
from datetime import datetime, timezone
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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2
from std_msgs.msg import Bool, Empty, Float32MultiArray, String
from rcl_interfaces.msg import Log
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import GetParameters
from cv_bridge import CvBridge

from custom_map_interfaces.srv import GetMatrices

from flask import Flask, Response, render_template, request, send_from_directory
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


def _level(xyz: np.ndarray, roll: float, pitch: float) -> np.ndarray:
    """Removes body roll and pitch (radians, REP-103) from rover-frame points."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return xyz @ (ry @ rx).T


def _local_dem_js():
    """Serves the local DEM view shared by the overview and 3D map pages."""
    return send_from_directory(template_dir, 'local_dem.js', mimetype='application/javascript')


def _feed_legends_js():
    """Serves the depth / segmentation legends shared by the camera pages."""
    return send_from_directory(template_dir, 'feed_legends.js', mimetype='application/javascript')


def _snapshot():
    if ros_node is None:
        return Response('ROS node not ready', status=503)
    name = datetime.now().strftime('gnc_snapshot_%Y%m%d_%H%M%S.zip')
    return Response(ros_node.snapshot_zip(), mimetype='application/zip',
                    headers={'Content-Disposition': f'attachment; filename={name}'})


for _app in (app, app_map):
    _app.add_url_rule('/local_dem.js', 'local_dem_js', _local_dem_js)
def _nav_overlay_js():
    """Serves the planned-path overlay drawn on the navcam feeds."""
    return send_from_directory(template_dir, 'nav_overlay.js', mimetype='application/javascript')


for _app in (app, app_cams, app_teleop):
    _app.add_url_rule('/feed_legends.js', 'feed_legends_js', _feed_legends_js)
    _app.add_url_rule('/nav_overlay.js', 'nav_overlay_js', _nav_overlay_js)
app.add_url_rule('/snapshot.zip', 'snapshot', _snapshot)


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

_HEALTH_PERIOD = 1.0           # seconds between feed-health / node-state pushes
_HEALTH_WINDOW = 5.0           # seconds of arrivals used for the rate estimate
_NODE_POLL_TICKS = 3           # health ticks between lifecycle state polls
_DEM_LIDAR_MAX_POINTS = 4000   # lidar points overlaid on the local DEM
_OBSTACLE_MAX_POINTS = 800     # local obstacle cells drawn on the global map
_LIDAR_SELF_RADIUS = 0.6       # m; lidar returns closer than this hit the rover

# Lifecycle nodes shown in the SYSTEM panel (state + restart).
_LIFECYCLE_NODES = (
    'lc_component_stereo_depth', 'lc_component_local_mapping',
    'lc_component_path_planner', 'lc_component_rover_leo_raph',
    'lc_component_lidar', 'lc_component_imu',
    'lc_component_event_camera', 'lc_component_spad_camera',
)
# Parameters read from other nodes: name -> (node, parameter).
_REMOTE_PARAMS = {
    'max_tilt_deg': ('lc_component_path_planner', 'max_tilt_deg'),
    'depth_max_distance': ('lc_component_stereo_depth', 'depth_max_distance'),
    # Central navcam mounting, as used by the local mapper (for the path overlay).
    'camera_angle': ('lc_component_local_mapping', 'camera_angle'),
    'camera_height_m': ('lc_component_local_mapping', 'camera_height_m'),
    'camera_offset_forward_m': ('lc_component_local_mapping', 'camera_offset_forward_m'),
    'camera_offset_up_m': ('lc_component_local_mapping', 'camera_offset_up_m'),
}
# Fallbacks until the mapper's parameters have been read.
_CAMERA_DEFAULTS = {'camera_angle': 20.0, 'camera_height_m': 0.577,
                    'camera_offset_forward_m': 0.171, 'camera_offset_up_m': 0.284}

_PATH_OVERLAY_AHEAD = 30.0     # m of planned path projected onto the navcam
_PATH_OVERLAY_STEP = 0.25      # m between projected path samples
_ROVER_HALF_WIDTH = 0.23       # m; half the ribbon width drawn along the path
_MIN_CAMERA_DEPTH = 0.3        # m; samples closer than this are not drawn
# Topics recorded by the dashboard's rosbag button.
_BAG_TOPICS = [
    '/odom', '/imu/data', '/cmd_vel', '/goal_pose', '/cancel_goal',
    '/global_path', '/driven_trajectory', '/mission_status/state',
    '/mission_status/goal_reached', '/lidar/points',
    '/local_mapping/elevation_map_raw', '/local_mapping/elevation_grid',
    '/navcam/central/image_color/compressed', '/navcam/central/image_depth',
    '/rear_cam/image_color/compressed',
    '/navcam/central/depth_ground_truth',
    '/navcam/central/camera_info', '/navcam/central/object_mask',
    '/navcam/central/shadow_mask', '/navcam/central/shadow_label',
    '/navcam/central/shadow_label_mapping', '/navcam/central/object_mask_mapping',
]


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
        self._restart_busy = set()          # nodes with a restart in progress

        # Feed health: recent arrival times per feed, for rate and age.
        self._arrivals = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self._health_timer = None
        self._health_ticks = 0

        # Latest data, for the snapshot export and for browsers that join late.
        self._last_jpeg = {}            # feed id -> JPEG bytes
        self._last_dem = None           # (DEM metres [forward, lateral], cell size)
        self._last_cloud = None         # lidar xyz, rover frame
        self._last_odom = None          # pose and attitude
        self._mission = {'state': 'unknown', 'goal': None, 'reason': ''}
        self._grid_pose = None          # local grid origin in odom: (x, y, yaw)
        self._hfov = None               # central camera horizontal FOV (rad)
        self._cam_info = None           # (fx, fy, cx, cy, width, height)
        self._ground = None             # local ground plane (a, b, c) of the latest DEM, DEM frame
        self._dem_half_width = 3.0      # m; lateral half-extent of the latest DEM
        self._odom_pose = None          # (x, y, z, qx, qy, qz, qw), latest odom
        self._global_path = None        # planned path (N, 2), odom frame
        self._elev = None               # planner's global elevation map [row = y, col = x], metres
        self._elev_offset = collections.deque(maxlen=30)   # odom z - map z, sampled at the rover
        self._elev_requested = 0.0
        self._overlay_shown = False     # a non-empty path overlay is on screen
        self._mask_legend = {}          # stencil id -> class name
        self._remote = {k: None for k in _REMOTE_PARAMS}
        self._node_states = {n: 'unknown' for n in _LIFECYCLE_NODES}
        self._cli_get_state, self._cli_change_state, self._cli_params = {}, {}, {}
        self._pending = set()           # outstanding async service calls
        self._bag_proc = None
        self._bag_path = None
        self._bag_t0 = 0.0

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
        # Lidar mounting (UnrealLidarCam, lidar/points in the sensor frame): yaw
        # and pitch of the sensor's X axis from the rover's forward axis (Unreal
        # convention: negative pitch looks down), and its height above the
        # ground. Defaults: following the central navcam (pitched 20 deg down).
        self.declare_parameter('lidar_yaw_deg', 0.0)
        self.declare_parameter('lidar_pitch_deg', -20.0)
        self.declare_parameter('lidar_height_m', 0.55)  # Navcam_Center above the ground (ground-plane fit while driving)
        self.declare_parameter('bag_directory', '/opt/ros/jazzy/src/mission_control_dashboard/bags')
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
        for n in _LIFECYCLE_NODES:
            self._cli_get_state[n] = self.create_client(GetState, f'/{n}/get_state')
            self._cli_change_state[n] = self.create_client(ChangeState, f'/{n}/change_state')
        for n in {node for node, _ in _REMOTE_PARAMS.values()}:
            self._cli_params[n] = self.create_client(GetParameters, f'/{n}/get_parameters')

        sub = self.create_subscription
        self._subs = [
            sub(Odometry, 'odom', self.odom_callback, 10),
            sub(Path, 'global_path', self.global_path_callback, 10),
            sub(Path, 'driven_trajectory', self.trajectory_callback, 10),
            sub(Log, '/rosout', self.rosout_callback, 50),
            sub(PointCloud2, 'lidar/points', self.lidar_callback,
                qos_profile_sensor_data),
            sub(OccupancyGrid, 'local_mapping/elevation_grid',
                self.local_grid_callback, 10),
            sub(Float32MultiArray, 'local_mapping/elevation_map_raw',
                self.raw_dem_callback, 10),
            sub(Imu, 'imu/data', lambda _m: self._mark('imu'), qos_profile_sensor_data),
            sub(String, 'mission_status/state', self.mission_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            sub(CameraInfo, f'{ns}/central/camera_info', self.camera_info_callback, 10),
            sub(String, f'{ns}/central/object_mask_mapping', self.mask_legend_callback, 10),
        ]

        # Compressed colour passes straight through: it is already JPEG.
        for cam in ('left', 'right', 'central'):
            self._subs.append(sub(
                CompressedImage, f'{ns}/{cam}/image_color/compressed',
                self._make_jpeg_relay(f'cam_{cam}'), qos_profile_sensor_data))

        # Rear camera (an extra mono camera of the stereo node).
        self._subs.append(sub(
            CompressedImage, 'rear_cam/image_color/compressed',
            self._make_jpeg_relay('cam_rear'), qos_profile_sensor_data))

        # Already-colourised products only need JPEG encoding.
        self._subs.append(sub(
            Image, f'{ns}/central/image_depth/visualization',
            self._make_image_relay('depth', both=True), qos_profile_sensor_data))
        self._subs.append(sub(
            Image, f'{ns}/central/depth_ground_truth/visualization',
            self._make_image_relay('depth_gt'), qos_profile_sensor_data))
        self._subs.append(sub(
            Image, f'{ns}/central/object_mask',
            self._make_image_relay('mask'), qos_profile_sensor_data))
        self._subs.append(sub(
            Image, f'{ns}/central/shadow_mask',
            self._make_image_relay('shadow'), qos_profile_sensor_data))

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
        self._health_timer = self.create_timer(_HEALTH_PERIOD, self._health_cb)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: mission control ACTIVE.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: mission control suspended.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.stop_bag()
        for attr in ('_teleop_watchdog', '_health_timer'):
            if getattr(self, attr) is not None:
                self.destroy_timer(getattr(self, attr))
                setattr(self, attr, None)
        for s in self._subs:
            self.destroy_subscription(s)
        self._subs = []
        for p in (self._pub_goal, self._pub_lights, self._pub_cmdvel, self._pub_cancel):
            if p:
                self.destroy_publisher(p)
        clients = [self._cli_get_maps, *self._cli_get_state.values(),
                   *self._cli_change_state.values(), *self._cli_params.values()]
        for c in clients:
            if c:
                self.destroy_client(c)
        self._cli_get_state, self._cli_change_state, self._cli_params = {}, {}, {}
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
            self._mark(feed_id)
            jpeg = bytes(msg.data)
            self._last_jpeg[feed_id] = jpeg
            emit_feeds('feed', {
                'id': feed_id,
                'image': base64.b64encode(jpeg).decode('ascii')})
            if feed_id == 'cam_central':
                self._emit_path_overlay()   # drawn on the frame just sent
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
            self._mark(feed_id)
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
                    self._last_jpeg[feed_id] = buf.tobytes()
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
        """Relays the cloud in the rover frame and overlays it on the local DEM."""
        if not self._is_active():
            return
        self._mark('lidar')
        now = time.monotonic()
        if now - self._last_lidar < _LIDAR_MIN_PERIOD:
            return
        self._last_lidar = now

        xyz = _cloud_xyz(msg, _MAX_LIDAR_POINTS * 4)
        if len(xyz) == 0:
            return
        # Sensor frame -> rover frame (X forward, Y left, Z up): the mount's
        # pitch (about Y; a sensor looking down has its X below the horizon),
        # then its yaw (about Z).
        a = math.radians(self.get_parameter('lidar_yaw_deg').value)
        p = -math.radians(self.get_parameter('lidar_pitch_deg').value)
        yaw = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        pitch = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
        rover = xyz @ (yaw @ pitch).astype(np.float32).T
        self._last_cloud = rover
        shown = rover[::max(1, len(rover) // _MAX_LIDAR_POINTS)][:_MAX_LIDAR_POINTS]
        emit_map('lidar', {
            'points': np.round(shown, 2).ravel().tolist(),
            'count': int(len(shown)),
            'frame': 'rover',
        })
        self._emit_dem_lidar(rover)

    def _emit_dem_lidar(self, rover: np.ndarray) -> None:
        """
        Lidar points in the local DEM frame (x right, y forward from the camera
        plane, z above the ground under the rover), levelled with the odom
        attitude like the DEM itself.
        """
        if self._last_odom is None:
            return
        att = self._last_odom
        lev = _level(rover.astype(np.float64), math.radians(att['roll']), math.radians(att['pitch']))
        h, w = (self._last_dem[0].shape if self._last_dem else (61, 121))
        res = self._last_dem[1] if self._last_dem else 0.05
        x, y = -lev[:, 1], lev[:, 0]
        z = lev[:, 2] + self.get_parameter('lidar_height_m').value
        keep = ((y >= 0) & (y <= h * res) & (np.abs(x) <= w * res / 2)
                & (np.hypot(lev[:, 0], lev[:, 1]) > _LIDAR_SELF_RADIUS))
        pts = np.column_stack((x[keep], y[keep], z[keep]))
        if len(pts) > _DEM_LIDAR_MAX_POINTS:
            pts = pts[::len(pts) // _DEM_LIDAR_MAX_POINTS + 1]
        emit_map('local_dem_lidar', {'xyz': np.round(pts, 3).ravel().tolist()})

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
        self._mark('local_dem')
        now = time.monotonic()
        if now - self._last_grid < _GRID_MIN_PERIOD:
            return
        self._last_grid = now

        W, H = msg.layout.dim[0].size, msg.layout.dim[1].size
        dem = np.asarray(msg.data, dtype=np.float32).reshape(W, H).T   # (forward, lateral)
        res = self._dem_res
        self._last_dem = (dem, res)

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
        self._ground = plane
        self._dem_half_width = dem.shape[1] * res / 2.0
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
        self._emit_obstacles(known & (relief >= threshold), res)

    def _emit_obstacles(self, obstacle: np.ndarray, res: float) -> None:
        """
        Obstacle cells and the camera's field of view in the odom frame, for the
        global map. Uses the pose of the latest local grid (x axis = rover right,
        y axis = forward from the camera plane).
        """
        if self._grid_pose is None:
            return
        ox, oy, gyaw = self._grid_pose
        c, s = math.cos(gyaw), math.sin(gyaw)
        rows, cols = np.nonzero(obstacle)
        if len(rows) > _OBSTACLE_MAX_POINTS:
            step = len(rows) // _OBSTACLE_MAX_POINTS + 1
            rows, cols = rows[::step], cols[::step]
        gx, gy = (cols + 0.5) * res, (rows + 0.5) * res
        pts = np.column_stack((ox + c * gx - s * gy, oy + s * gx + c * gy))
        half_w = obstacle.shape[1] * res / 2.0
        socketio.emit('local_obstacles', {
            'pts': np.round(pts, 2).ravel().tolist(),
            'fov': {'x': round(ox + c * half_w, 3), 'y': round(oy + s * half_w, 3),
                    'yaw': round(gyaw + math.pi / 2, 4),
                    'hfov': round(self._hfov or math.radians(90.0), 4),
                    'range': round(obstacle.shape[0] * res, 2)},
        })

    def local_grid_callback(self, msg: OccupancyGrid) -> None:
        """Tracks the DEM cell size and the grid pose; the grid itself is not relayed."""
        self._dem_res = float(msg.info.resolution)
        o = msg.info.origin
        q = o.orientation
        self._grid_pose = (o.position.x, o.position.y,
                           math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    # =========================================================================
    # Planned path projected onto the central navcam
    # =========================================================================

    def _cam_param(self, key: str) -> float:
        value = self._remote.get(key)
        return _CAMERA_DEFAULTS[key] if value is None else value

    def _map_height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Bilinear height from the global elevation map (NaN outside it)."""
        res = self.map_resolution
        r, c = np.asarray(y) / res, np.asarray(x) / res
        h, w = self._elev.shape
        inside = (r >= 0) & (c >= 0) & (r < h - 1) & (c < w - 1)
        r0 = np.clip(np.floor(r).astype(int), 0, h - 2)
        c0 = np.clip(np.floor(c).astype(int), 0, w - 2)
        fr, fc = r - r0, c - c0
        e = self._elev
        z = (e[r0, c0] * (1 - fr) * (1 - fc) + e[r0 + 1, c0] * fr * (1 - fc)
             + e[r0, c0 + 1] * (1 - fr) * fc + e[r0 + 1, c0 + 1] * fr * fc)
        return np.where(inside, z, np.nan)

    def _ground_z(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Terrain height at odom (x, y). Primary source: the planner's global
        elevation map, offset so it matches the ground under the rover (median
        over recent frames). Fallback: the local DEM's ground plane (x right,
        y forward from the camera plane), extrapolated.
        """
        z_ground = self._odom_pose[2] - (self._cam_param('camera_height_m')
                                         - self._cam_param('camera_offset_up_m'))
        z = np.full_like(x, np.nan, dtype=float)
        if self._elev is not None:
            here = self._map_height(np.array([self._odom_pose[0]]), np.array([self._odom_pose[1]]))[0]
            if np.isfinite(here):
                self._elev_offset.append(z_ground - here)
            if self._elev_offset:
                z = self._map_height(x, y) + float(np.median(self._elev_offset))
        missing = ~np.isfinite(z)
        if not missing.any():
            return z
        z[missing] = self._plane_z(x[missing], y[missing], z_ground)
        return z

    def _plane_z(self, x: np.ndarray, y: np.ndarray, z_ground: float) -> np.ndarray:
        """Height from the local DEM's ground plane, extrapolated (flat if none)."""
        if self._ground is None or self._grid_pose is None:
            return np.full_like(x, z_ground)
        ox, oy, g = self._grid_pose
        rx, ry = x - ox, y - oy
        gx = math.cos(g) * rx + math.sin(g) * ry - self._dem_half_width
        gy = -math.sin(g) * rx + math.cos(g) * ry
        a, b, c = self._ground
        return z_ground + a * gx + b * gy + c

    def _project(self, xyz: np.ndarray) -> np.ndarray:
        """
        Projects odom-frame points into the central navcam image. Returns
        (N, 2) normalised (u, v) in [0, 1] image coordinates, NaN behind or
        too close to the camera.
        """
        fx, fy, cx, cy, w, h = self._cam_info
        px, py, pz, qx, qy, qz, qw = self._odom_pose
        r_bw = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        a = math.radians(self._cam_param('camera_angle'))         # pitched down
        r_mount = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
        cam = np.array([px, py, pz]) + r_bw @ np.array(
            [self._cam_param('camera_offset_forward_m'), 0.0, self._cam_param('camera_offset_up_m')])
        p = (xyz - cam) @ (r_bw @ r_mount)            # camera axes: forward, left, up
        with np.errstate(divide='ignore', invalid='ignore'):
            u = (cx - fx * p[:, 1] / p[:, 0]) / w
            v = (cy - fy * p[:, 2] / p[:, 0]) / h
        uv = np.column_stack((u, v))
        uv[p[:, 0] < _MIN_CAMERA_DEPTH] = np.nan
        return uv

    def _emit_path_overlay(self) -> None:
        """
        Sends the planned path ahead of the rover, projected onto the central
        navcam, as a centre line, a rover-width ribbon and metre marks.
        """
        active = self._mission.get('state') in ('planning', 'driving')
        path = self._global_path
        if (not active or path is None or len(path) < 2
                or self._cam_info is None or self._odom_pose is None):
            if self._overlay_shown:
                emit_feeds('path_overlay', {'center': []})
                self._overlay_shown = False
            return

        # Path ahead: from the rover to the closest vertex onwards, resampled.
        rx, ry = self._odom_pose[0], self._odom_pose[1]
        i0 = int(np.argmin(np.hypot(path[:, 0] - rx, path[:, 1] - ry)))
        pts = np.vstack(([rx, ry], path[i0 + 1:]))
        if len(pts) < 2:
            return
        cum = np.r_[0.0, np.cumsum(np.hypot(*np.diff(pts, axis=0).T))]
        d = np.arange(0.0, min(cum[-1], _PATH_OVERLAY_AHEAD) + 1e-9, _PATH_OVERLAY_STEP)
        if len(d) < 2:
            return
        x, y = np.interp(d, cum, pts[:, 0]), np.interp(d, cum, pts[:, 1])
        tx, ty = np.gradient(x), np.gradient(y)
        norm = np.hypot(tx, ty) + 1e-9
        nx, ny = -ty / norm * _ROVER_HALF_WIDTH, tx / norm * _ROVER_HALF_WIDTH

        def project(px, py):
            uv = self._project(np.column_stack((px, py, self._ground_z(px, py))))
            return [None if not np.isfinite(r).all() else [round(float(r[0]), 4), round(float(r[1]), 4)]
                    for r in uv]

        emit_feeds('path_overlay', {
            'center': project(x, y),
            'left': project(x + nx, y + ny),
            'right': project(x - nx, y - ny),
            'dist': [round(float(v), 2) for v in d],
            'pose': [round(float(v), 4) for v in self._odom_pose],   # pose used, for checks
        })
        self._overlay_shown = True

    def camera_info_callback(self, msg: CameraInfo) -> None:
        fx = msg.k[0]
        if fx > 0 and msg.width > 0:
            self._hfov = 2.0 * math.atan(msg.width / (2.0 * fx))
            self._cam_info = (fx, msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)

    def mask_legend_callback(self, msg: String) -> None:
        """Stencil id -> class name legend of the object mask, from component_stereo_depth."""
        try:
            legend = {str(k): str(v) for k, v in json.loads(msg.data).items()}
        except (ValueError, AttributeError):
            return
        if legend != self._mask_legend:
            self._mask_legend = legend
            emit_feeds('mask_legend', legend)

    def mission_callback(self, msg: String) -> None:
        """Planner mission state (idle / planning / driving / reached / aborted / cancelled)."""
        try:
            mission = json.loads(msg.data)
        except ValueError:
            return
        mission['t'] = time.time()
        self._mission = mission
        emit_all('mission', mission)

    # =========================================================================
    # Telemetry, paths, logs
    # =========================================================================

    def odom_callback(self, msg: Odometry) -> None:
        if not self._is_active():
            return
        self._mark('odom')
        pp, qq = msg.pose.pose.position, msg.pose.pose.orientation
        self._odom_pose = (pp.x, pp.y, pp.z, qq.x, qq.y, qq.z, qq.w)
        now = time.monotonic()
        if now - self._last_odom_emit < _ODOM_MIN_PERIOD:
            return
        self._last_odom_emit = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = np.degrees(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        roll = math.degrees(math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                                       1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))))
        # Combined tilt as the planner computes it (angle of the body z axis).
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))))

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
            'roll': round(roll, 1), 'pitch': round(pitch, 1), 'tilt': round(tilt, 1),
            'tilt_limit': self._remote['max_tilt_deg'],
        }
        self._last_odom = odom_payload
        emit_all('odom_update', odom_payload)
        sio_map.emit('odom_update', odom_payload)

    def global_path_callback(self, msg: Path) -> None:
        if not self._is_active():
            return
        coords = [{'x': p.pose.position.x, 'y': p.pose.position.y}
                  for p in msg.poses]
        self._global_path = (np.array([[c['x'], c['y']] for c in coords])
                             if coords else None)
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

    def _store_elevation(self, response, elev, valid) -> None:
        """Keeps the global elevation map for the path overlay's ground heights."""
        dims = response.elevation_map.layout.dim
        self._elev = np.where(valid, elev, np.nan).reshape(dims[0].size, dims[1].size)

    def _request_elevation(self) -> None:
        """Fetches the global map once for the backend if no browser has requested it."""
        now = time.monotonic()
        if (self._elev is not None or self._cli_get_maps is None
                or not self._cli_get_maps.service_is_ready() or now - self._elev_requested < 10.0):
            return
        self._elev_requested = now

        def done(fut):
            try:
                r = fut.result()
                e = np.asarray(r.elevation_map.data, dtype=np.float32)
                self._store_elevation(r, e, np.isfinite(e) & (e < 9999.0))
            except Exception as exc:
                self.get_logger().warn(f"Elevation map for the path overlay unavailable: {exc}")
        self._cli_get_maps.call_async(GetMatrices.Request()).add_done_callback(done)

    def map_response_callback(self, future, sid=None) -> None:
        try:
            response = future.result()
            # Quantise to uint16 over the valid range and send as base64: the
            # JSON float list for a 1000x1000 map is ~19 MB and stalls the
            # socket. 65535 marks no-data (planner sentinel >= 9999).
            elev = np.asarray(response.elevation_map.data, dtype=np.float32)
            valid = np.isfinite(elev) & (elev < 9999.0)
            self._store_elevation(response, elev, valid)
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
        self.restart_node('lc_component_stereo_depth')

    def restart_node(self, name: str) -> None:
        """Cycles a lifecycle node from a background thread."""
        if name not in self._cli_change_state:
            self._console('WARN', f'Unknown node {name}.')
            return
        if name in self._restart_busy:
            self._console('WARN', f'Restart of {name} already in progress.')
            return
        self._restart_busy.add(name)
        threading.Thread(target=self._restart_node_impl, args=(name,), daemon=True).start()

    def _console(self, level: str, text: str) -> None:
        """Log to ROS and mirror onto the operator console."""
        {'INFO': self.get_logger().info, 'WARN': self.get_logger().warn,
         'ERROR': self.get_logger().error}.get(level, self.get_logger().info)(text)
        socketio.emit('log', {'level': level, 'node': 'dashboard',
                              'text': text, 'stamp': time.time()})

    def _restart_node_impl(self, name: str) -> None:
        """
        deactivate -> cleanup -> configure -> activate on `name`.

        Equivalent to restarting its container (for the stereo node, cleanup
        releases the UnrealCV socket and configure reconnects). The first two
        steps may legitimately fail if the node is already inactive, so only
        configure and activate are load-bearing.
        """
        client = self._cli_change_state[name]
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                self._console('ERROR', f'{name}: lifecycle service unreachable — '
                                       f'is its container running?')
                return
            self._console('INFO', f'Restarting {name}...')
            steps = [(4, 'deactivate', False), (2, 'cleanup', False),
                     (1, 'configure', True), (3, 'activate', True)]
            for tid, label, required in steps:
                req = ChangeState.Request()
                req.transition.id = tid
                fut = client.call_async(req)
                t0 = time.monotonic()
                while not fut.done() and time.monotonic() - t0 < 90.0:
                    time.sleep(0.1)
                ok = fut.done() and fut.result() is not None and fut.result().success
                if ok:
                    self._console('INFO', f'{name}: {label} ok.')
                elif required:
                    self._console('ERROR', f'{name}: {label} FAILED — check its log; '
                                           f'is Unreal playing?')
                    return
                else:
                    self._console('WARN', f'{name}: {label} skipped (already past '
                                          f'that state).')
            self._console('INFO', f'{name} restarted.')
        finally:
            self._restart_busy.discard(name)

    # =========================================================================
    # Health: feed rates, node states, remote parameters
    # =========================================================================

    def _mark(self, feed: str) -> None:
        self._arrivals[feed].append(time.monotonic())

    def _health_cb(self) -> None:
        if not self._is_active():
            return
        now = time.monotonic()
        feeds = {}
        for name, times in self._arrivals.items():
            recent = sum(1 for t in times if now - t <= _HEALTH_WINDOW)
            feeds[name] = {'hz': round(recent / _HEALTH_WINDOW, 1),
                           'age': round(now - times[-1], 1) if times else None}
        payload = {'feeds': feeds, 'nodes': dict(self._node_states), 'bag': self._bag_status()}
        for server in (socketio, sio_cams, sio_teleop, sio_map):
            server.emit('health', payload)

        self._health_ticks += 1
        self._request_elevation()
        if self._health_ticks % _NODE_POLL_TICKS == 1:
            self._poll_nodes()
        self._fetch_remote_params()

    def _poll_nodes(self) -> None:
        """Queries each node's lifecycle state; replies arrive asynchronously."""
        for name, client in self._cli_get_state.items():
            if not client.service_is_ready():
                self._node_states[name] = 'offline'
                continue
            key = ('state', name)
            if key in self._pending:
                continue
            self._pending.add(key)
            fut = client.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=name, k=key: self._on_node_state(n, k, f))

    def _on_node_state(self, name: str, key, fut) -> None:
        self._pending.discard(key)
        try:
            self._node_states[name] = fut.result().current_state.label
        except Exception:
            self._node_states[name] = 'error'

    def _fetch_remote_params(self) -> None:
        """Reads the planner's tilt limit and the depth colour range once available."""
        for key, (node, param) in _REMOTE_PARAMS.items():
            client = self._cli_params.get(node)
            if self._remote[key] is not None or client is None or not client.service_is_ready():
                continue
            pkey = ('param', key)
            if pkey in self._pending:
                continue
            self._pending.add(pkey)
            fut = client.call_async(GetParameters.Request(names=[param]))
            fut.add_done_callback(lambda f, k=key, pk=pkey: self._on_remote_param(k, pk, f))

    def _on_remote_param(self, key: str, pkey, fut) -> None:
        self._pending.discard(pkey)
        try:
            v = fut.result().values[0]
            value = v.double_value if v.type == 3 else float(v.integer_value) if v.type == 2 else None
        except Exception:
            value = None
        if value is not None:
            self._remote[key] = value
            for server in (socketio, sio_cams, sio_teleop, sio_map):
                server.emit('config', self._remote)

    # =========================================================================
    # Recording: rosbag + snapshot
    # =========================================================================

    def _bag_status(self) -> dict:
        running = self._bag_proc is not None and self._bag_proc.poll() is None
        return {'recording': running,
                'name': os.path.basename(self._bag_path) if self._bag_path else None,
                'elapsed': round(time.time() - self._bag_t0) if running else 0}

    def start_bag(self) -> None:
        if self._bag_proc is not None and self._bag_proc.poll() is None:
            return
        directory = self.get_parameter('bag_directory').value
        os.makedirs(directory, exist_ok=True)
        self._bag_path = os.path.join(directory, datetime.now().strftime('gnc_%Y%m%d_%H%M%S'))
        self._bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'mcap', '-o', self._bag_path] + _BAG_TOPICS,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        self._bag_t0 = time.time()
        self._console('INFO', f'Recording rosbag {os.path.basename(self._bag_path)} '
                              f'({len(_BAG_TOPICS)} topics).')

    def stop_bag(self) -> None:
        proc = self._bag_proc
        if proc is None or proc.poll() is not None:
            return
        os.killpg(proc.pid, signal.SIGINT)      # lets rosbag close the file cleanly
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        self._console('INFO', f'Rosbag saved: mission_control_dashboard/bags/'
                              f'{os.path.basename(self._bag_path)}')

    def snapshot_zip(self) -> bytes:
        """Latest camera frames, local DEM, lidar cloud and state, as a zip."""
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
            for feed_id, jpeg in list(self._last_jpeg.items()):
                z.writestr(f'{feed_id}.jpg', jpeg)
            if self._last_dem is not None:
                dem, res = self._last_dem
                b = io.BytesIO()
                np.savez_compressed(b, dem=dem, cell_size_m=res,
                                    axes='rows: forward from the camera plane; '
                                         'columns: lateral, right +; metres above '
                                         'the ground under the rover; NaN = unobserved')
                z.writestr('local_dem.npz', b.getvalue())
            if self._last_cloud is not None:
                b = io.BytesIO()
                np.savez_compressed(b, xyz=self._last_cloud, frame='rover: x forward, y left, z up')
                z.writestr('lidar.npz', b.getvalue())
            z.writestr('metadata.json', json.dumps({
                'utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
                'odom': self._last_odom, 'mission': self._mission,
                'nodes': self._node_states, 'feeds': sorted(self._last_jpeg),
            }, indent=2))
        return buf.getvalue()

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

        This shares the cmd_vel bus with the GNC planner, so engaging cancels
        an active goal. Engaging also arms the watchdog so a dropped connection
        cannot leave the rover running.
        """
        if not (self._pub_cmdvel and self._pub_cmdvel.is_activated):
            return
        lin = float(lin)
        ang = float(ang)
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        moving = lin != 0.0 or ang != 0.0
        if (moving and not self._teleop_engaged
                and self._mission.get('state') in ('planning', 'driving')
                and self._pub_cancel and self._pub_cancel.is_activated):
            # Manual driving takes over: cancel the GNC goal so the planner
            # does not fight the operator on the shared cmd_vel bus.
            self._pub_cancel.publish(Empty())
            self._console('WARN', 'Manual drive took over: GNC goal cancelled.')
        self._pub_cmdvel.publish(t)
        self._last_teleop = time.monotonic()
        self._teleop_engaged = self._teleop_engaged or moving

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
        _send_cached(socketio, request.sid)


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


@socketio.on('restart_node')
def handle_restart_node(json_data):
    if ros_node is not None:
        ros_node.restart_node(str(json_data.get('name', '')))


@socketio.on('bag')
def handle_bag(json_data):
    if ros_node is None:
        return
    if json_data.get('action') == 'start':
        ros_node.start_bag()
    else:
        threading.Thread(target=ros_node.stop_bag, daemon=True).start()


def _send_cached(server, sid):
    """State a newly connected browser needs before the next update arrives."""
    if ros_node is None:
        return
    server.emit('config', ros_node._remote, to=sid)
    if ros_node._mask_legend:
        server.emit('mask_legend', ros_node._mask_legend, to=sid)
    server.emit('mission', ros_node._mission, to=sid)


@sio_cams.on('connect')
def handle_cams_connect(auth=None):
    _send_cached(sio_cams, request.sid)


@sio_teleop.on('connect')
def handle_teleop_connect(auth=None):
    _send_cached(sio_teleop, request.sid)


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
