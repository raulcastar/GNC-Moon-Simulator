"""Camera feeds relayed to the browser, and the planned path drawn on the central navcam."""

import base64
import json
import math
import time

import cv2
import numpy as np

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from .settings import CAMERA_DEFAULTS, MIN_CAMERA_DEPTH, PATH_OVERLAY_AHEAD, PATH_OVERLAY_STEP, ROVER_HALF_WIDTH
from .web import emit_feeds


class ImageryMixin:
    """Camera feeds relayed to the browser, and the planned path drawn on the central navcam. Mixed into LifecycleMissionControlUI."""

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

    def _cam_param(self, key: str) -> float:
        value = self._remote.get(key)
        return CAMERA_DEFAULTS[key] if value is None else value

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
        uv[p[:, 0] < MIN_CAMERA_DEPTH] = np.nan
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
        d = np.arange(0.0, min(cum[-1], PATH_OVERLAY_AHEAD) + 1e-9, PATH_OVERLAY_STEP)
        if len(d) < 2:
            return
        x, y = np.interp(d, cum, pts[:, 0]), np.interp(d, cum, pts[:, 1])
        tx, ty = np.gradient(x), np.gradient(y)
        norm = np.hypot(tx, ty) + 1e-9
        nx, ny = -ty / norm * ROVER_HALF_WIDTH, tx / norm * ROVER_HALF_WIDTH

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
