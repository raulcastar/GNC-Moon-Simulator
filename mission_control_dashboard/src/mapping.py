"""Lidar, the local DEM and its obstacles, and the global elevation map."""

import base64
import math
import time
import warnings

import numpy as np

from custom_map_interfaces.srv import GetMatrices
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray

from .geometry import cloud_xyz, ground_plane, level_points
from .settings import DEM_LIDAR_MAX_POINTS, DEM_MAX_CELLS, GRID_MIN_PERIOD, LIDAR_MIN_PERIOD, LIDAR_SELF_RADIUS, MAX_LIDAR_POINTS, OBSTACLE_MAX_POINTS
from .web import emit_map, socketio


class MappingMixin:
    """Lidar, the local DEM and its obstacles, and the global elevation map. Mixed into LifecycleMissionControlUI."""

    def lidar_callback(self, msg: PointCloud2) -> None:
        """Relays the cloud in the rover frame and overlays it on the local DEM."""
        if not self._is_active():
            return
        self._mark('lidar')
        now = time.monotonic()
        if now - self._last_lidar < LIDAR_MIN_PERIOD:
            return
        self._last_lidar = now

        xyz = cloud_xyz(msg, MAX_LIDAR_POINTS * 4)
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
        shown = rover[::max(1, len(rover) // MAX_LIDAR_POINTS)][:MAX_LIDAR_POINTS]
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
        lev = level_points(rover.astype(np.float64), math.radians(att['roll']), math.radians(att['pitch']))
        h, w = (self._last_dem[0].shape if self._last_dem else (61, 121))
        res = self._last_dem[1] if self._last_dem else 0.05
        x, y = -lev[:, 1], lev[:, 0]
        z = lev[:, 2] + self.get_parameter('lidar_height_m').value
        keep = ((y >= 0) & (y <= h * res) & (np.abs(x) <= w * res / 2)
                & (np.hypot(lev[:, 0], lev[:, 1]) > LIDAR_SELF_RADIUS))
        pts = np.column_stack((x[keep], y[keep], z[keep]))
        if len(pts) > DEM_LIDAR_MAX_POINTS:
            pts = pts[::len(pts) // DEM_LIDAR_MAX_POINTS + 1]
        emit_map('local_dem_lidar', {'xyz': np.round(pts, 3).ravel().tolist()})

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
        if now - self._last_grid < GRID_MIN_PERIOD:
            return
        self._last_grid = now

        W, H = msg.layout.dim[0].size, msg.layout.dim[1].size
        dem = np.asarray(msg.data, dtype=np.float32).reshape(W, H).T   # (forward, lateral)
        res = self._dem_res
        self._last_dem = (dem, res)

        # Bound the browser mesh; max-pool so obstacles survive the reduction.
        f = int(math.ceil(max(W, H) / DEM_MAX_CELLS))
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
        plane = ground_plane(dem, res)
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
        if len(rows) > OBSTACLE_MAX_POINTS:
            step = len(rows) // OBSTACLE_MAX_POINTS + 1
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
