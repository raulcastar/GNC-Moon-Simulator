"""
ROS 2 Lifecycle Node — Local DEM Mapping from Depth Camera.

  projectCameraCoord  — pixel (u, v) + depth d → camera frame (x_p, y_p, d)
  rotateRefSystem     — camera frame → local world (x_w, y_w, z_w), accounting
                        for camera tilt angle and physical mounting height
  computeDEM          — bin (x_w, y_w) → 2-D grid, elevation = max z_w per cell

Published topics
----------------
  local_mapping/elevation_map_raw  (std_msgs/Float32MultiArray) — raw metres
  local_mapping/elevation_grid     (nav_msgs/OccupancyGrid)     — scaled 0-100
  local_mapping/pointcloud         (sensor_msgs/PointCloud2)    — in odom frame
"""

import math
import threading
from typing import Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn

from sensor_msgs.msg import Image as RosImage, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from cv_bridge import CvBridge

# BEST_EFFORT + depth=1: a new depth frame replaces any queued one so the map
# always uses the most recent data rather than playing catch-up.
_DEPTH_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Unit quaternion → 3x3 rotation matrix  (body → world frame)."""
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n < 1e-10:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    return np.array([
        [1 - s*(qy*qy + qz*qz),     s*(qx*qy - qz*qw),     s*(qx*qz + qy*qw)],
        [    s*(qx*qy + qz*qw), 1 - s*(qx*qx + qz*qz),     s*(qy*qz - qx*qw)],
        [    s*(qx*qz - qy*qw),     s*(qy*qz + qx*qw), 1 - s*(qx*qx + qy*qy)],
    ], dtype=np.float64)


class LifecycleLocalMapping(rclpy.lifecycle.Node):
    """
    Local DEM built from a single RealSense-style depth camera.

    Pipeline per depth frame
    ------------------------
    1. projectCameraCoord (vectorised)
       The depth topic carries planar Z-depth in metres.  z_cam = d
       x_cam = px · z_cam   [lateral,  right +]   px = (u - cx)/fx
       y_cam = py · z_cam   [vertical, down  +]   py = (v - cy)/fy

    2. rotateRefSystem (vectorised)  — ref: CameraTransform.rotateRefSystem
       total = π/2 + camera_angle_rad
       x_w   = x_cam                                    [lateral, unchanged]
       y_w   = y_cam·cos(total) + z_cam·sin(total)      [horizontal forward]
       z_w   = -y_cam·sin(total) + z_cam·cos(total) + camera_height_m  [elevation]

    3. computeDEM (vectorised)       — ref: MergedImage.computeDEM
       Filter by distance_limit on both x_w and y_w.
       x_offset = |min(x_w)|  (shift so all lateral indices ≥ 0)
       DEM[x_idx, y_idx] = max z_w in that cell (obstacle-safe aggregation)

    4. Global frame for PointCloud2
       Map (x_w, y_w, z_w) → body frame (REP-103: X=forward, Y=left, Z=up):
         body_x =  y_w                          (camera forward → rover forward)
         body_y = -x_w                          (camera right  → rover -left)
         body_z =  z_w - camera_height_m        (elevation rel. to rover base)
       Then rotate by rover quaternion from /odom and translate by rover position.
    """

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self._pub_elevation_raw:  Optional[Publisher] = None
        self._pub_elevation_grid: Optional[Publisher] = None
        self._pub_pointcloud:     Optional[Publisher] = None
        self._sub_depth = None
        self._sub_color = None
        self._sub_odom  = None
        self._sub_info  = None
        self._is_active: bool = False

        # Latest colour image — written by _color_callback, read by _depth_callback.
        self._color_lock = threading.Lock()
        self._latest_color_img: Optional[np.ndarray] = None  # (H, W, 3) float32 [0,1]

        # Pre-computed pixel direction grids — allocated on the first depth frame
        # so that image dimensions are known without a separate config parameter.
        self._pixel_x: Optional[np.ndarray] = None   # (H, W): (u − cx) / fx
        self._pixel_y: Optional[np.ndarray] = None   # (H, W): (v − cy) / fy
        self._odom_lock = threading.Lock()
        self._latest_pose: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        self.cv_bridge = CvBridge()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic',     rclpy.Parameter.Type.STRING),
                # camera_fx/fy/cx/cy are fallbacks, used until camera_info
                # arrives, or for ever when use_camera_info is false.
                ('use_camera_info',  rclpy.Parameter.Type.BOOL),
                ('camera_fx',        rclpy.Parameter.Type.DOUBLE),
                ('camera_fy',        rclpy.Parameter.Type.DOUBLE),
                ('camera_cx',        rclpy.Parameter.Type.DOUBLE),  # = Cu in reference
                ('camera_cy',        rclpy.Parameter.Type.DOUBLE),  # = Cv in reference
                # Physical mounting
                ('camera_height_m',  rclpy.Parameter.Type.DOUBLE),  # = phys_h in reference
                ('camera_angle',     rclpy.Parameter.Type.DOUBLE),  # degrees; 0 = horizontal
                ('camera_roll',      rclpy.Parameter.Type.DOUBLE),  # degrees; lateral mounting tilt (usually 0)
                # DEM parameters
                ('map_resolution',   rclpy.Parameter.Type.DOUBLE),  # metres per cell
                ('distance_limit',   rclpy.Parameter.Type.DOUBLE),  # metres; mirrors reference
                # Depth validity window (max is distance_limit — single source of truth)
                ('min_valid_depth',  rclpy.Parameter.Type.DOUBLE),
                ('depth_scale',      rclpy.Parameter.Type.DOUBLE),
                # Elevation sanity window (metres, relative to the rover base at
                # z_w=0). Rejects physically impossible heights from stereo-depth
                # outliers before they poison the max-per-cell DEM.
                ('min_elevation',    rclpy.Parameter.Type.DOUBLE),
                ('max_elevation',    rclpy.Parameter.Type.DOUBLE),
                # Apply live rover roll/pitch (from /odom) to the DEM. OFF by
                # default: the odom orientation is in a non-standard frame, so the
                # extracted roll is garbage (~-84° on flat ground) and warps the
                # DEM. The fixed camera tilt is handled by camera_angle instead.
                ('apply_rover_tilt', rclpy.Parameter.Type.BOOL),
                ('debug_mode',       rclpy.Parameter.Type.BOOL),
                ('debug_save_path',  rclpy.Parameter.Type.STRING),
            ]
        )

    # =========================================================================
    # Odometry callback
    # =========================================================================

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._odom_lock:
            self._latest_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def _color_callback(self, msg: RosImage) -> None:
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            with self._color_lock:
                self._latest_color_img = img.astype(np.float32) / 255.0
        except Exception as exc:
            self.get_logger().warn(f"Colour decode failed: {exc}", throttle_duration_sec=5.0)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """
        Take intrinsics from the publisher rather than a YAML file.

        A DEM built with the wrong fx is uniformly stretched or squashed in
        range, and nothing in the output announces it.
        """
        fx, cx, cy = msg.k[0], msg.k[2], msg.k[5]
        fy = msg.k[4] or fx
        if fx <= 0.0:
            return
        changed = (fx, fy, cx, cy) != (self.camera_fx, self.camera_fy,
                                       self.camera_cx, self.camera_cy)
        if changed:
            self.camera_fx, self.camera_fy = fx, fy
            self.camera_cx, self.camera_cy = cx, cy
            self._pixel_x = None          # force the direction grids to rebuild
            self._pixel_y = None
            self.get_logger().info(
                f"Intrinsics from camera_info: fx={fx:.2f} fy={fy:.2f} "
                f"cx={cx:.2f} cy={cy:.2f}")

    # =========================================================================
    # Pixel direction grids
    # =========================================================================

    def _init_pixel_coords(self, height: int, width: int) -> None:
        """
        Pre-compute per-pixel normalised direction vectors (pinhole model).

        Vectorised equivalent of CameraTransform.projectCameraCoord:
            pixel_x[v, u] = (u - cx) / fx   →   x_cam = pixel_x * d
            pixel_y[v, u] = (v - cy) / fy   →   y_cam = pixel_y * d
        """
        u = np.arange(width,  dtype=np.float64)
        v = np.arange(height, dtype=np.float64)
        uu, vv = np.meshgrid(u, v)
        self._pixel_x = (uu - self.camera_cx) / self.camera_fx   # (H, W)
        self._pixel_y = (vv - self.camera_cy) / self.camera_fy   # (H, W)
        self.get_logger().info(
            f"Pixel direction grids ready: {width}x{height} | "
            f"fx={self.camera_fx} fy={self.camera_fy} "
            f"cx={self.camera_cx} cy={self.camera_cy}"
        )

    # =========================================================================
    # DEM computation — vectorised port of MergedImage.computeDEM (RealSense)
    # =========================================================================

    def _compute_dem(self, depth_m: np.ndarray,
                     color_img: Optional[np.ndarray] = None,
                     rover_pitch: float = 0.0,
                     rover_roll:  float = 0.0):
        """
        Full DEM pipeline for one depth frame.

        Parameters
        ----------
        depth_m   : (H, W) float32 depth image in metres.
        color_img : optional (H, W, 3) float32 RGB image in [0, 1].
                    When provided, colours are returned for every mapped point.

        Returns
        -------
        DEM      : ndarray (DEM_width, DEM_height) — max elevation per cell (m),
                   NaN where no valid point landed.
        x_w_v    : ndarray (N,) — lateral world coord of each mapped point
        y_w_v    : ndarray (N,) — forward world coord of each mapped point
        z_w_v    : ndarray (N,) — elevation of each mapped point
        x_offset : float — shift applied to x_w so all x grid indices are ≥ 0;
                   matches `x_offset` in reference's computeDEM.
        colors   : ndarray (N, 3) float32 RGB in [0, 1], or None.
        Returns (None, …) when there are no valid points this frame.
        """
        h, w = depth_m.shape
        if self._pixel_x is None:
            self._init_pixel_coords(h, w)

        # ── 1. Filter valid depth pixels ─────────────────────────────────────
        valid = (
            (depth_m > self.min_valid_depth) &
            (depth_m < self.distance_limit) &
            np.isfinite(depth_m)
        )
        # Track flat pixel indices so we can look up colour later
        pixel_idx = np.where(valid.ravel())[0]        # (N,) flat indices

        d  = depth_m[valid].astype(np.float64) * self.depth_scale  # (N,) metres
        px = self._pixel_x[valid]                                   # (N,) (u − cx)/fx
        py = self._pixel_y[valid]                                   # (N,) (v − cy)/fy

        if self.debug_mode:
            self.get_logger().info(
                f"[DEM diag] valid pixels: {d.size}/{depth_m.size} | "
                f"depth range: [{d.min():.3f}, {d.max():.3f}] m  (depth_scale={self.depth_scale})"
                if d.size > 0 else
                f"[DEM diag] NO valid pixels (all filtered by depth window "
                f"{self.min_valid_depth}-{self.distance_limit} m)"
            )

        if d.size == 0:
            return None, None, None, None, None, None

        # ── 2. projectCameraCoord (vectorised) ───────────────────────────────
        # The depth topic carries planar Z depth: component_stereo_depth already
        # converted UnrealCV's radial buffer. Do not reintroduce a
        # `/ sqrt(1 + px² + py²)` divide — on already-planar depth it pulls
        # off-centre points toward the camera by 60% in the corners at 90° HFOV.
        z_cam = d               # planar Z-depth  (forward along optical axis)
        x_cam = px * z_cam      # lateral  (right +)
        y_cam = py * z_cam      # vertical (down  +)

        # ── 3. rotateRefSystem (vectorised) ──────────────────────────────────
        #   ref:
        #     angle_radians = radians(camera_angle)
        #     ref_rotation  = π/2
        #     total         = ref_rotation + angle_radians
        #     x_r =   x_p
        #     y_r =   y_p * cos(total) + d * sin(total)
        #     z_r = - y_p * sin(total) + d * cos(total) + phys_h
        #
        # rover_pitch tilts the combined sensor forward/backward:
        #   total pitch = fixed camera mounting angle + live rover pitch from odom
        total = math.pi / 2.0 + self._camera_angle_rad + rover_pitch
        cos_t = math.cos(total)
        sin_t = math.sin(total)

        x_w = x_cam                                # lateral (unchanged)
        y_w = y_cam * cos_t + z_cam * sin_t        # horizontal forward distance
        z_w = -y_cam * sin_t + z_cam * cos_t + self.camera_height_m  # elevation

        # ── 3b. Roll correction ───────────────────────────────────────────────
        # rover_roll (+ camera_roll_rad for fixed mounting offset) rotates in the
        # lateral/elevation plane: a positive roll lifts the rover's right side,
        # which shifts map points laterally.  We rotate around the camera origin
        # (z_rel = 0 at the camera elevation), then restore the height offset.
        total_roll = rover_roll + self._camera_roll_rad
        if total_roll != 0.0:
            z_rel = z_w - self.camera_height_m
            cr = math.cos(total_roll)
            sr = math.sin(total_roll)
            x_w_new =  x_w  * cr - z_rel * sr
            z_rel_new= x_w  * sr + z_rel * cr
            x_w = x_w_new
            z_w = z_rel_new + self.camera_height_m

        if self.debug_mode:
            self.get_logger().info(
                f"[DEM diag] world coords — "
                f"x_w:[{x_w.min():.2f}, {x_w.max():.2f}] "
                f"y_w:[{y_w.min():.2f}, {y_w.max():.2f}] "
                f"z_w:[{z_w.min():.2f}, {z_w.max():.2f}] m | "
                f"distance_limit={self.distance_limit} m"
            )

        # ── 4. Distance limit filter ──────────────────────────────────────────
        #   ref: if(x_w < distance_limit and y_w < distance_limit)
        #   We extend to both sides of the lateral axis via abs(x_w).
        in_range = (
            (np.abs(x_w) < self.distance_limit) &
            (y_w > 0.0) &
            (y_w < self.distance_limit) &
            # Elevation sanity gate: stereo depth degrades as Z²/(fx·B) and, on a
            # near-horizontal camera, a grazing ground pixel or a bad above-horizon
            # match projects to a wildly wrong height. Those outliers otherwise win
            # the max-per-cell reduction and spike the DEM. Keep only plausible
            # ground/rock elevations relative to the rover base (z_w = 0).
            (z_w >= self.min_elevation) &
            (z_w <= self.max_elevation)
        )
        x_w = x_w[in_range]
        y_w = y_w[in_range]
        z_w = z_w[in_range]

        pixel_idx = pixel_idx[in_range]

        if self.debug_mode:
            self.get_logger().info(
                f"[DEM diag] after distance filter: {x_w.size} points survive "
                f"(|x_w|<{self.distance_limit}, 0<y_w<{self.distance_limit})"
            )

        if x_w.size == 0:
            return None, None, None, None, None, None

        # ── 5. computeDEM grid building ───────────────────────────────────────
        # Fixed-size grid bounded by distance_limit so that changing the limit
        # directly controls the area that gets mapped — not just the plot view.
        #
        #   lateral extent: -distance_limit … +distance_limit  (width  = 2*dl/res + 1)
        #   forward extent:  0              … +distance_limit  (height =   dl/res + 1)
        #   x_offset = distance_limit  →  x=0 (camera centre) maps to column DEM_width//2
        res_mult   = 1.0 / self.map_resolution
        dl         = self.distance_limit
        x_offset   = dl                                         # fixed, frame-invariant
        DEM_width  = int(2.0 * dl * res_mult) + 1              # lateral cells
        DEM_height = int(      dl * res_mult) + 1              # forward cells

        x_idx = np.floor((x_w + x_offset) * res_mult).astype(np.int32)
        y_idx = np.floor( y_w              * res_mult).astype(np.int32)
        # Clamp: any residual floating-point overshoot is quietly discarded
        in_cell = (
            (x_idx >= 0) & (x_idx < DEM_width) &
            (y_idx >= 0) & (y_idx < DEM_height)
        )
        pixel_idx = pixel_idx[in_cell]
        x_idx = x_idx[in_cell];  x_w_v = x_w[in_cell]
        y_idx = y_idx[in_cell];  y_w_v = y_w[in_cell]
        z_w_v = z_w[in_cell]

        # Maximum height per cell — rocks must not be averaged away
        flat = np.full(DEM_width * DEM_height, -np.inf, dtype=np.float32)
        np.maximum.at(flat, x_idx * DEM_height + y_idx, z_w_v.astype(np.float32))

        # Cells that received no points remain at -inf → NaN (unknown)
        flat[np.isinf(flat)] = np.nan
        DEM = flat.reshape(DEM_width, DEM_height)

        # ── 6. Colour lookup (optional) ───────────────────────────────────────
        colors = None
        if color_img is not None and color_img.shape[:2] == (h, w):
            rows = pixel_idx // w
            cols = pixel_idx % w
            colors = color_img[rows, cols]   # (N, 3) float32 RGB [0, 1]

        return DEM, x_w_v, y_w_v, z_w_v, x_offset, colors

    # =========================================================================
    # Debug visualisation
    # =========================================================================

    def _show_dem(
        self,
        DEM: np.ndarray,
        x_offset: float,
        x_w: np.ndarray,
        y_w: np.ndarray,
        z_w: np.ndarray,
        colors: Optional[np.ndarray] = None,
        rover_pitch: float = 0.0,
        rover_roll:  float = 0.0,
    ) -> None:
        """
        Two-panel live debug plot refreshed every depth frame.

        Left  — Top-down scatter: raw world-frame points (x_w vs y_w)
                coloured by elevation z_w.  Camera sits at the origin,
                forward = up, lateral = left/right.
        Right — Gridded DEM imshow: same data binned into cells,
                max-elevation per cell, terrain colormap.

        Safe to call from the depth callback because SingleThreadedExecutor
        keeps all callbacks on the main (OS) thread.
        """
        fig = plt.gcf()
        fig.clf()
        fig.canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        ax_scatter, ax_dem = fig.subplots(1, 2)
        fig.suptitle(
            f'Local Mapping — cam_angle={math.degrees(self._camera_angle_rad):.1f}°  '
            f'rover_pitch={math.degrees(rover_pitch):.1f}°  '
            f'rover_roll={math.degrees(rover_roll):.1f}°  '
            f'dist_limit={self.distance_limit} m  pts={x_w.size}',
            fontsize=9,
        )

        # ── Left: top-down scatter ────────────────────────────────────────────
        xs, ys, zs = x_w, y_w, z_w
        cs = colors

        use_rgb = cs is not None
        sc = ax_scatter.scatter(
            xs, ys,
            c=cs if use_rgb else zs,
            cmap=None if use_rgb else 'terrain',
            s=1,
            linewidths=0,
            alpha=0.9 if use_rgb else 0.7,
        )
        if not use_rgb:
            fig.colorbar(sc, ax=ax_scatter, fraction=0.046, pad=0.04,
                         label='Elevation (m)')

        ax_scatter.set_xlim(-self.distance_limit, self.distance_limit)
        ax_scatter.set_ylim(0, self.distance_limit)
        ax_scatter.set_xlabel('Lateral x  (m)')
        ax_scatter.set_ylabel('Forward y  (m)')
        scatter_title = 'Top-down point cloud  (RGB)' if use_rgb else 'Top-down point cloud  (elevation)'
        ax_scatter.set_title(scatter_title)
        ax_scatter.set_aspect('equal')
        ax_scatter.plot(0, 0, 'r^', markersize=8, label='Camera')
        ax_scatter.legend(loc='lower right', fontsize=7)
        ax_scatter.grid(True, alpha=0.3)

        # ── Right: flat 2D DEM imshow ─────────────────────────────────────────
        DEM_w, DEM_h = DEM.shape
        res = self.map_resolution

        finite = DEM[np.isfinite(DEM)]
        vmin = float(finite.min()) if finite.size else 0.0
        vmax = float(finite.max()) if finite.size else 1.0

        # DEM[x, y]: dim-0 = lateral (x), dim-1 = forward (y).
        # Transpose for imshow so rows = forward (y↑), cols = lateral (x→).
        cmap_dem = plt.get_cmap('terrain').copy()
        cmap_dem.set_bad('#404040')   # dark grey for unobserved cells

        im = ax_dem.imshow(
            np.ma.masked_invalid(DEM.T),
            origin='lower',
            aspect='auto',
            cmap=cmap_dem,
            vmin=vmin,
            vmax=vmax,
            interpolation='nearest',
        )
        fig.colorbar(im, ax=ax_dem, fraction=0.046, pad=0.04,
                     label='Elevation (m)')

        # Clip view to ±distance_limit laterally and 0→distance_limit forward.
        # Cell coords: col = (x_m + x_offset) / res,  row = y_m / res
        dl = self.distance_limit
        x_cell_lo = (-dl + x_offset) / res          # left  edge in cell space
        x_cell_hi = ( dl + x_offset) / res          # right edge in cell space
        y_cell_hi = dl / res                        # far   edge in cell space

        ax_dem.set_xlim(x_cell_lo - 0.5, x_cell_hi + 0.5)
        ax_dem.set_ylim(-0.5, y_cell_hi + 0.5)

        # Tick labels in metres over the clipped range
        n_x = 7;  n_y = 7
        ax_dem.set_xticks(np.linspace(x_cell_lo, x_cell_hi, n_x))
        ax_dem.set_xticklabels(
            [f'{v:.1f}' for v in np.linspace(-dl, dl, n_x)],
            fontsize=7,
        )
        ax_dem.set_yticks(np.linspace(0, y_cell_hi, n_y))
        ax_dem.set_yticklabels(
            [f'{v:.1f}' for v in np.linspace(0.0, dl, n_y)],
            fontsize=7,
        )
        ax_dem.set_xlabel('Lateral x  (m)')
        ax_dem.set_ylabel('Forward y  (m)')
        ax_dem.set_title(
            f'DEM  {DEM_w}×{DEM_h} @ {res*100:.0f} cm/cell  '
            f'[{vmin:.2f}, {vmax:.2f}] m',
            fontsize=8,
        )

        fig.tight_layout()
        plt.pause(0.001)

    # =========================================================================
    # Depth callback — orchestrates DEM computation and publishing
    # =========================================================================

    def _depth_callback(self, msg: RosImage) -> None:
        if not self._is_active:
            return

        try:
            depth_m = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().error(f"Depth decode failed: {exc}")
            return

        # ── Grab latest colour frame (non-blocking) ───────────────────────────
        with self._color_lock:
            color_img = self._latest_color_img  # None until first colour message

        # ── Rover pose snapshot — used for tilt correction AND global frame ───
        with self._odom_lock:
            tx, ty, tz, qx, qy, qz, qw = self._latest_pose

        # Decompose quaternion into roll (around X) and pitch (around Y).
        # Standard ZYX Tait-Bryan convention, same as ROS REP-103.
        #   roll  — rotation around X (right axis): lifts one side of the rover
        #   pitch — rotation around Y (left axis):  tilts rover forward/backward
        if self.apply_rover_tilt:
            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            rover_roll  = math.atan2(sinr_cosp, cosr_cosp)

            sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
            rover_pitch = math.asin(sinp)
        else:
            # The odom orientation is in a non-standard frame (same convention
            # that mirrors the dashboard heading), so the roll extracted here is
            # spurious (~-84° on flat ground) and rotates the whole DEM into a
            # narrow warped sliver. Ignore it; camera_angle carries the fixed tilt.
            rover_roll = 0.0
            rover_pitch = 0.0

        # ── DEM computation (reference code logic, vectorised) ────────────────
        DEM, x_w, y_w, z_w, x_offset, colors = self._compute_dem(
            depth_m, color_img, rover_pitch, rover_roll)
        if DEM is None:
            return   # no valid points this frame

        DEM_width, DEM_height = DEM.shape

        # ── Local world → global odom frame ──────────────────────────────────
        # rotateRefSystem produces a left-handed-ish local frame:
        #   x_w = lateral (right +),  y_w = forward,  z_w = elevation
        #
        # Map to body frame (REP-103: X=forward, Y=left, Z=up):
        #   body_x =  y_w                        forward
        #   body_y = -x_w                        right of camera → rover -left
        #   body_z =  z_w − camera_height_m      height rel. to rover base origin
        #
        # Then: p_odom = R_body→world @ p_body + rover_position
        R_bw   = _quat_to_rot(qx, qy, qz, qw)              # (3, 3)
        p_body = np.stack([ y_w,
                            -x_w,
                             z_w - self.camera_height_m], axis=0)   # (3, N)
        p_odom = R_bw @ p_body + np.array([[tx], [ty], [tz]])       # (3, N)

        Xw_g = p_odom[0].astype(np.float32)
        Yw_g = p_odom[1].astype(np.float32)
        Zw_g = p_odom[2].astype(np.float32)

        # ── Publish ───────────────────────────────────────────────────────────
        stamp = msg.header.stamp
        self._publish_elevation_raw(DEM, DEM_width, DEM_height, stamp)
        self._publish_elevation_grid(DEM, DEM_width, DEM_height, stamp,
                                     x_offset, tx, ty, tz, qx, qy, qz, qw)
        if Xw_g.size > 0:
            pts = np.stack([Xw_g, Yw_g, Zw_g], axis=1)
            self._publish_pointcloud(pts, stamp, 'odom')

        if self.debug_mode:
            self._show_dem(DEM, x_offset, x_w, y_w, z_w, colors,
                           rover_pitch=rover_pitch, rover_roll=rover_roll)

    # =========================================================================
    # Publisher helpers
    # =========================================================================

    def _publish_elevation_raw(
        self,
        DEM: np.ndarray,
        DEM_width: int,
        DEM_height: int,
        stamp,
    ) -> None:
        """
        Float32MultiArray where DEM[x_idx, y_idx] = elevation in metres.
        Dim 0 (lateral_x) = DEM_width cells, Dim 1 (forward_y) = DEM_height cells.
        NaN cells (no observed points) are emitted as 0.0.
        """
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(label='lateral_x', size=DEM_width,
                                    stride=DEM_width * DEM_height),
                MultiArrayDimension(label='forward_y',  size=DEM_height,
                                    stride=DEM_height),
            ],
            data_offset=0,
        )
        msg.data = np.nan_to_num(DEM, nan=0.0).flatten().tolist()
        self._pub_elevation_raw.publish(msg)

    def _publish_elevation_grid(
        self,
        DEM: np.ndarray,
        DEM_width: int,
        DEM_height: int,
        stamp,
        x_offset: float,
        tx: float, ty: float, tz: float,
        qx: float, qy: float, qz: float, qw: float,
    ) -> None:
        """
        OccupancyGrid in 'odom' frame.

        DEM[x_idx, y_idx]  — lateral (x) first, forward (y) second.
        OccupancyGrid data is row-major with row = Y (forward), col = X (lateral),
        so DEM.T is flattened: grid.data[y * DEM_width + x] = DEM[x, y].

        The grid origin (bottom-left corner, x_w = -x_offset, y_w = 0) is
        translated to the odom frame using the full rover pose.
        """
        known = np.isfinite(DEM)
        norm  = np.full((DEM_width, DEM_height), -1, dtype=np.int8)  # -1 = unobserved
        if known.any():
            h_min = float(DEM[known].min())
            h_max = float(DEM[known].max())
            span  = h_max - h_min if h_max > h_min else 1.0
            # Normalise only the known cells — avoids NaN→int8 cast warning
            norm[known] = np.clip(
                ((DEM[known] - h_min) / span * 100), 0, 100
            ).astype(np.int8)

        # Grid origin in local world frame: leftmost-lateral, y=0 (camera plane)
        # In body frame:  forward = 0,  lateral = +x_offset (left of camera)
        origin_body  = np.array([0.0, x_offset, -self.camera_height_m])
        R_bw         = _quat_to_rot(qx, qy, qz, qw)
        origin_odom  = R_bw @ origin_body + np.array([tx, ty, tz])

        grid_msg = OccupancyGrid()
        grid_msg.header              = Header(stamp=stamp, frame_id='odom')
        grid_msg.info.resolution     = self.map_resolution
        grid_msg.info.width          = DEM_width    # lateral cells  (X direction)
        grid_msg.info.height         = DEM_height   # forward cells  (Y direction)
        grid_msg.info.origin = Pose(
            position=Point(
                x=float(origin_odom[0]),
                y=float(origin_odom[1]),
                z=float(origin_odom[2]),
            ),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        # DEM[x, y] → OccupancyGrid row-major [y, x] → DEM.T.flatten()
        grid_msg.data = norm.T.flatten().tolist()
        self._pub_elevation_grid.publish(grid_msg)

    def _publish_pointcloud(
        self,
        points: np.ndarray,
        stamp,
        frame_id: str,
    ) -> None:
        header = Header(stamp=stamp, frame_id=frame_id)
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        pc2_msg = point_cloud2.create_cloud(header, fields, points.tolist())
        self._pub_pointcloud.publish(pc2_msg)

    # =========================================================================
    # Lifecycle callbacks
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: CONFIGURED.")

        self.camera_topic    = self.get_parameter('camera_topic').value
        self.use_camera_info = self.get_parameter('use_camera_info').value
        self.camera_fx       = self.get_parameter('camera_fx').value
        self.camera_fy       = self.get_parameter('camera_fy').value
        self.camera_cx       = self.get_parameter('camera_cx').value
        self.camera_cy       = self.get_parameter('camera_cy').value
        self.camera_height_m = self.get_parameter('camera_height_m').value
        camera_angle_deg     = self.get_parameter('camera_angle').value
        self._camera_angle_rad = math.radians(camera_angle_deg)
        camera_roll_deg      = self.get_parameter('camera_roll').value
        self._camera_roll_rad  = math.radians(camera_roll_deg)
        self.map_resolution  = self.get_parameter('map_resolution').value
        self.distance_limit  = self.get_parameter('distance_limit').value
        self.min_valid_depth = self.get_parameter('min_valid_depth').value
        self.depth_scale     = self.get_parameter('depth_scale').value
        self.min_elevation   = self.get_parameter('min_elevation').value
        self.max_elevation   = self.get_parameter('max_elevation').value
        self.apply_rover_tilt = self.get_parameter('apply_rover_tilt').value
        # max valid depth == distance_limit: single parameter controls both the
        # pixel-level cut and the world-frame distance filter.
        self.debug_mode      = self.get_parameter('debug_mode').value
        self.debug_save_path = self.get_parameter('debug_save_path').value

        if self.debug_mode:
            plt.ion()

        depth_topic = f'{self.camera_topic}/image_depth'
        color_topic = f'{self.camera_topic}/image_color'
        info_topic  = f'{self.camera_topic}/camera_info'
        self.get_logger().info(
            f"Subscribing to '{depth_topic}' and '{color_topic}' | "
            f"camera_angle={camera_angle_deg}° camera_roll={camera_roll_deg}° "
            f"camera_height={self.camera_height_m} m | "
            f"resolution={self.map_resolution} m/cell distance_limit={self.distance_limit} m"
        )
        # Stereo depth: sparse, with occlusion holes and range-dependent
        # quantisation. Cells with no valid point stay unknown in the DEM.
        self.get_logger().info(
            f"Depth source is stereo, not ground truth — expect holes."
            + (f" Intrinsics will follow '{info_topic}'." if self.use_camera_info
               else f" Using YAML intrinsics fx={self.camera_fx}.")
        )

        self._pub_elevation_raw  = self.create_lifecycle_publisher(
            Float32MultiArray, 'local_mapping/elevation_map_raw', 10)
        self._pub_elevation_grid = self.create_lifecycle_publisher(
            OccupancyGrid, 'local_mapping/elevation_grid', 10)
        self._pub_pointcloud     = self.create_lifecycle_publisher(
            PointCloud2, 'local_mapping/pointcloud', 10)

        self._sub_odom  = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self._sub_depth = self.create_subscription(
            RosImage, depth_topic, self._depth_callback, _DEPTH_QOS)
        self._sub_color = self.create_subscription(
            RosImage, color_topic, self._color_callback, _DEPTH_QOS)
        if self.use_camera_info:
            self._sub_info = self.create_subscription(
                CameraInfo, info_topic, self._camera_info_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: ACTIVE.")
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: DEACTIVATED.")
        self._is_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: CLEANING UP.")
        self._is_active = False
        self._pixel_x = None
        self._pixel_y = None

        for sub in (self._sub_depth, self._sub_color, self._sub_odom, self._sub_info):
            if sub:
                self.destroy_subscription(sub)
        self._sub_depth = None
        self._sub_color = None
        self._sub_odom  = None
        self._sub_info  = None
        with self._color_lock:
            self._latest_color_img = None

        for pub in (self._pub_elevation_raw, self._pub_elevation_grid, self._pub_pointcloud):
            if pub:
                self.destroy_publisher(pub)
        self._pub_elevation_raw  = None
        self._pub_elevation_grid = None
        self._pub_pointcloud     = None

        plt.close('all')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle: SHUTTING DOWN.")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LifecycleLocalMapping('lc_component_local_mapping')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
