"""
ROS 2 Lifecycle Web Mission Control.

Flask + SocketIO server presenting the whole stack in one browser page:
every navcam feed, stereo depth, object segmentation, a live lidar sweep,
the elevation map with click-to-goal, odometry telemetry, and the ROS log
stream (/rosout) as a mission console.

All imagery is re-encoded to JPEG and pushed over the websocket; the browser
renders lidar and map client-side on canvases.

Layout of this package:
  mission_control_dashboard  the node (construction, lifecycle) and main()
  web                        Flask apps / SocketIO servers per screen, emit helpers
  routes                     page routes and SocketIO handlers of every screen
  imagery                    camera feed relays, planned path drawn on the navcam
  mapping                    lidar, local DEM and obstacles, global elevation map
  telemetry                  odometry, paths, mission state, /rosout console
  topics                     System window: ROS 2 graph, topic inspection
  health                     feed rates, lifecycle nodes and restarts, remote params
  recording                  rosbag recording (presets, notes, rename / delete) and snapshot
  replay                     BAGS window: replaying the recorded rosbags
  commands                   headlights, stop, teleop drive, goals
  geometry, settings         pure helpers and tuning constants
"""

import collections
import threading

import rclpy
from custom_map_interfaces.srv import GetMatrices
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2
from std_msgs.msg import Bool, Empty, Float32MultiArray, String

from . import web
from .commands import CommandsMixin
from .health import HealthMixin
from .imagery import ImageryMixin
from .mapping import MappingMixin
from .recording import RecordingMixin
from .replay import ReplayMixin
from .settings import (CAM_PRODUCT_MIN_PERIOD, DEFAULT_BAG_PRESET, HEALTH_PERIOD, INSPECT_PERIOD,
                       LIFECYCLE_NODES, REMOTE_PARAMS, TIMELINE_MAX)
from .telemetry import TelemetryMixin
from .topics import TopicsMixin
from .web import (app, app_cams, app_hub, app_map, app_replay, app_sys, app_teleop, sio_cams, sio_hub,
                  sio_map, sio_replay, sio_sys, sio_teleop, socketio)
from . import routes  # noqa: F401  (registers the page / socket handlers)


class LifecycleMissionControlUI(ImageryMixin, MappingMixin, TelemetryMixin, TopicsMixin,
                                HealthMixin, RecordingMixin, ReplayMixin, CommandsMixin, rclpy.lifecycle.Node):
    """Bridges the ROS graph to the browser over SocketIO. The work is split over
    the mixins (one module each); this class owns construction and the lifecycle."""

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

        # System window: the topic each browser is inspecting (sid -> state).
        self._inspect = {}
        self._inspect_lock = threading.Lock()
        self._inspect_timer = None
        self._init_replay()

        # Latest data, for the snapshot export and for browsers that join late.
        self._last_jpeg = {}            # feed id -> JPEG bytes
        self._last_dem = None           # (DEM metres [forward, lateral], cell size)
        self._last_cloud = None         # lidar xyz, rover frame
        self._last_odom = None          # pose and attitude
        self._mission = {'state': 'unknown', 'goal': None, 'reason': ''}
        self._timeline = collections.deque(maxlen=TIMELINE_MAX)   # mission events, oldest first
        self._feeds_seen = set()        # feeds that delivered data in this Play session
        self._nodes_seen = set()        # lifecycle nodes seen active
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
        self._remote = {k: None for k in REMOTE_PARAMS}
        self._node_states = {n: 'unknown' for n in LIFECYCLE_NODES}
        self._cli_get_state, self._cli_change_state, self._cli_params = {}, {}, {}
        self._pending = set()           # outstanding async service calls
        self._bag_proc = None
        self._bag_path = None
        self._bag_t0 = 0.0
        self._bag_size = (0, 0.0)            # (bytes, time) of the last size reading, for the rate
        self._bag_meta = {}
        self._bag_choice = {'preset': DEFAULT_BAG_PRESET, 'topics': []}   # REC uses the last choice

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

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: configuring mission control interfaces.")
        ns = self.get_parameter('camera_namespace').value

        self._pub_goal = self.create_lifecycle_publisher(PoseStamped, 'goal_pose', 10)
        self._pub_lights = self.create_lifecycle_publisher(Bool, 'headlights_cmd', 10)
        self._pub_cmdvel = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)
        self._pub_cancel = self.create_lifecycle_publisher(Empty, 'cancel_goal', 10)
        self._cli_get_maps = self.create_client(GetMatrices, 'get_matrices')
        for n in LIFECYCLE_NODES:
            self._cli_get_state[n] = self.create_client(GetState, f'/{n}/get_state')
            self._cli_change_state[n] = self.create_client(ChangeState, f'/{n}/change_state')
        for n in {node for node, _ in REMOTE_PARAMS.values()}:
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
            self._make_image_relay('events', min_period=CAM_PRODUCT_MIN_PERIOD),
            qos_profile_sensor_data))

        # SPAD camera digitised reconstruction from component_spad_camera.
        self._subs.append(sub(
            Image, 'spad/image',
            self._make_image_relay('spad', min_period=CAM_PRODUCT_MIN_PERIOD),
            qos_profile_sensor_data))

        # 5 Hz teleop deadman: stop the rover if manual commands stop arriving.
        self._teleop_watchdog = self.create_timer(0.2, self._teleop_watchdog_cb)
        self._health_timer = self.create_timer(HEALTH_PERIOD, self._health_cb)
        self._inspect_timer = self.create_timer(INSPECT_PERIOD, self._inspect_cb)
        self._configure_replay()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: mission control ACTIVE.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: mission control suspended.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.stop_bag()
        for sid in list(self._inspect):
            self.stop_inspect(sid)
        self._cleanup_replay()
        for attr in ('_teleop_watchdog', '_health_timer', '_inspect_timer'):
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


def main(args=None):
    rclpy.init(args=args)
    web.node = LifecycleMissionControlUI('mission_control_dashboard')
    executor = SingleThreadedExecutor()
    executor.add_node(web.node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Hub (5000), teleop (5002), cameras (5003), map3d (5005) and system (5006) run in background
    # threads; the overview (5001) holds the main thread. All reach the node through
    # web.node.
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
    threading.Thread(target=_serve, args=(sio_sys, app_sys, 5006),
                     daemon=True).start()
    threading.Thread(target=_serve, args=(sio_replay, app_replay, 5007),
                     daemon=True).start()
    web.node.get_logger().info(
        "Mission control up: hub :5000  overview :5001  teleop :5002  "
        "cameras :5003  map3d :5005  system :5006  replay :5007")

    try:
        socketio.run(app, host='0.0.0.0', port=5001, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        web.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
