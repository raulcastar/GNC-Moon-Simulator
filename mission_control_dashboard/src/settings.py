"""Tuning constants of the mission-control dashboard."""


# /rosout severity bytes -> console labels
LOG_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

# Push rates deliberately low: the browser renders the lidar and DEM in WebGL,
# which competes with Unreal for the same (laptop) GPU. Higher rates here buy
# nothing the operator can perceive and risk stalling the desktop compositor.
MAX_LIDAR_POINTS = 9000       # per frame pushed to the browser
LIDAR_MIN_PERIOD = 1.0        # seconds between lidar frames
ODOM_MIN_PERIOD = 0.15        # seconds between telemetry emits
GRID_MIN_PERIOD = 1.0         # seconds between local DEM frames
DEM_MAX_CELLS = 160           # max DEM cells per side sent to the browser
# The driven-trajectory Path grows without bound as the rover moves. Relaying
# every pose meant an ever-larger payload each update, and the browser re-drew
# the whole polyline on every odom tick — O(N) with N climbing, which is what
# made the machine crawl over a long run. Decimate to a fixed budget (the trail
# shape is preserved) and throttle. Bounds payload and client redraw cost.
TRAJ_MAX_POINTS = 1500        # points per driven-path push (decimated)
TRAJ_MIN_PERIOD = 0.5         # seconds between driven-path frames
# The event and SPAD cameras publish at ~30 Hz. Relaying every frame to the
# browser as a JPEG floods the websocket and stalls the UI, so cap their
# browser-facing rate. 0.16 s ~= 6 fps, plenty for these visualisations.
CAM_PRODUCT_MIN_PERIOD = 0.16

HEALTH_PERIOD = 1.0           # seconds between feed-health / node-state pushes
HEALTH_WINDOW = 5.0           # seconds of arrivals used for the rate estimate
INSPECT_PERIOD = 1.0          # seconds between TOPICS-panel updates of the inspected topic
INSPECT_WINDOW = 5.0          # seconds of arrivals for its rate and bandwidth
INSPECT_TRUNCATE = 12         # array elements shown in its message preview
INSPECT_MAX_CHARS = 6000      # preview length cap
INSPECT_MAX_STRING = 2000     # characters of a string field shown in the preview
REPLAY_PERIOD = 0.5           # seconds between replay-window updates (state, position)
REPLAY_CLOCK_HZ = 100         # /clock generated for bags recorded without one
# Recorded topics a live node recomputes from the replayed sensors (or commands
# that would re-drive the system), left out of a replay by default: replaying
# them too would put two publishers on one topic. topic -> who regenerates it.
REPLAY_LIVE_OUTPUTS = {
    '/imu/data': 'component_imu, from /odom',
    '/local_mapping/elevation_map_raw': 'component_local_mapping, from the depth',
    '/local_mapping/elevation_grid': 'component_local_mapping, from the depth',
    '/global_path': 'component_path_planner',
    '/planner/obstacles': 'component_path_planner (lidar obstacles)',
    '/driven_trajectory': 'component_path_planner',
    '/mission_status/state': 'component_path_planner',
    '/mission_status/goal_reached': 'component_path_planner',
    '/goal_pose': 'a command: would send the recorded goals again',
    '/cancel_goal': 'a command',
    '/cmd_vel': 'a command: would drive the rover',
    '/headlights_cmd': 'a command',
}
NODE_POLL_TICKS = 3           # health ticks between lifecycle state polls
# Alerts (the banner in every window): a feed that delivered data while Unreal
# is in Play and then goes quiet this long; a lifecycle node that was active and
# left that state.
FEED_STALE_S = 5.0
FEED_NAMES = {
    'odom': 'Odometry', 'imu': 'IMU', 'cam_central': 'Central navcam', 'cam_left': 'Left navcam',
    'cam_right': 'Right navcam', 'cam_rear': 'Rear camera', 'depth': 'Stereo depth',
    'depth_gt': 'Ground-truth depth', 'mask': 'Segmentation', 'shadow': 'Shadow mask',
    'events': 'Event camera', 'spad': 'SPAD camera', 'lidar': 'Lidar', 'local_dem': 'Local elevation map',
}
TIMELINE_MAX = 400            # mission events kept for the overview's timeline
DEM_LIDAR_MAX_POINTS = 4000   # lidar points overlaid on the local DEM
OBSTACLE_MAX_POINTS = 800     # local obstacle cells drawn on the global map
LIDAR_SELF_RADIUS = 0.6       # m; lidar returns closer than this hit the rover

# Lifecycle nodes shown in the SYSTEM panel (state + restart).
LIFECYCLE_NODES = (
    'lc_component_stereo_depth', 'lc_component_local_mapping',
    'lc_component_path_planner', 'lc_component_rover_leo_raph',
    'lc_component_lidar', 'lc_component_imu',
    'lc_component_event_camera', 'lc_component_spad_camera',
    'lc_component_sim_clock',
)
# Parameters read from other nodes: name -> (node, parameter).
# The planner, whose lidar obstacle avoidance the overview switches.
PLANNER_NODE = 'lc_component_path_planner'
AVOIDANCE_PARAM = 'lidar_obstacle_avoidance'
OBSTACLES_MIN_PERIOD = 0.5      # s between obstacle updates sent to the overview

REMOTE_PARAMS = {
    'max_tilt_deg': ('lc_component_path_planner', 'max_tilt_deg'),
    'depth_max_distance': ('lc_component_stereo_depth', 'depth_max_distance'),
    # Central navcam mounting, as used by the local mapper (for the path overlay).
    'camera_angle': ('lc_component_local_mapping', 'camera_angle'),
    'camera_height_m': ('lc_component_local_mapping', 'camera_height_m'),
    'camera_offset_forward_m': ('lc_component_local_mapping', 'camera_offset_forward_m'),
    'camera_offset_up_m': ('lc_component_local_mapping', 'camera_offset_up_m'),
}
# Fallbacks until the mapper's parameters have been read.
CAMERA_DEFAULTS = {'camera_angle': 20.0, 'camera_height_m': 0.577,
                    'camera_offset_forward_m': 0.171, 'camera_offset_up_m': 0.284}

PATH_OVERLAY_AHEAD = 30.0     # m of planned path projected onto the navcam
PATH_OVERLAY_STEP = 0.25      # m between projected path samples
ROVER_HALF_WIDTH = 0.23       # m; half the ribbon width drawn along the path
MIN_CAMERA_DEPTH = 0.3        # m; samples closer than this are not drawn
# Rosbag recording presets (BAGS window and the overview's REC). Sizes measured
# with the simulator at its default rates.
_NAVIGATION = [
    '/clock',                                   # simulation time: a replay runs on it
    '/odom', '/imu/data', '/cmd_vel', '/goal_pose', '/cancel_goal', '/headlights_cmd',
    '/global_path', '/driven_trajectory', '/mission_status/state', '/mission_status/goal_reached',
    '/local_mapping/elevation_map_raw', '/local_mapping/elevation_grid',
    '/navcam/central/image_color/compressed', '/navcam/central/camera_info',
    '/navcam/central/image_depth',              # lets local mapping re-run on a replay
    '/rear_cam/image_color/compressed', '/rear_cam/camera_info',
]
BAG_PRESETS = {
    'navigation': {
        'label': 'NAVIGATION',
        'about': 'pose, IMU, commands, goals, mission, paths, local maps, navcam depth and the '
                 'compressed navcam / rear images: replays a drive and re-runs local mapping '
                 '(~2 MB/s, ~6 GB/h)',
        'topics': _NAVIGATION,
    },
    'perception': {
        'label': 'FULL PERCEPTION',
        'about': 'navigation plus lidar, ground-truth depth, object and shadow masks, the left / '
                 'right navcams, event and SPAD cameras: for perception work offline '
                 '(~8 MB/s, ~30 GB/h)',
        'topics': _NAVIGATION + [
            '/lidar/points', '/navcam/central/depth_ground_truth',
            '/navcam/central/object_mask', '/navcam/central/object_mask_mapping',
            '/navcam/central/object_mask_classes',
            '/navcam/central/shadow_mask', '/navcam/central/shadow_label',
            '/navcam/central/shadow_label_mapping',
            '/navcam/left/image_color/compressed', '/navcam/left/camera_info',
            '/navcam/right/image_color/compressed', '/navcam/right/camera_info',
            '/event_camera/image_events', '/event_camera/event_rate', '/spad/image',
        ],
    },
}
DEFAULT_BAG_PRESET = 'navigation'
BAG_LOW_DISK_BYTES = 5e9      # warn below this much free space where bags are written
