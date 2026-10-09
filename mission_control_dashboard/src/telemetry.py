"""Odometry, planned and driven paths, mission state and the /rosout console."""

import json
import math
import time

import numpy as np

from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from std_msgs.msg import String

from .settings import LOG_LEVELS, ODOM_MIN_PERIOD, TRAJ_MAX_POINTS, TRAJ_MIN_PERIOD
from .web import emit_all, sio_map, socketio


class TelemetryMixin:
    """Odometry, planned and driven paths, mission state and the /rosout console. Mixed into LifecycleMissionControlUI."""

    def mission_callback(self, msg: String) -> None:
        """Planner mission state (idle / planning / driving / reached / aborted / cancelled)."""
        try:
            mission = json.loads(msg.data)
        except ValueError:
            return
        mission['t'] = time.time()
        self._mission = mission
        emit_all('mission', mission)

    def odom_callback(self, msg: Odometry) -> None:
        if not self._is_active():
            return
        self._mark('odom')
        pp, qq = msg.pose.pose.position, msg.pose.pose.orientation
        self._odom_pose = (pp.x, pp.y, pp.z, qq.x, qq.y, qq.z, qq.w)
        now = time.monotonic()
        if now - self._last_odom_emit < ODOM_MIN_PERIOD:
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
        if now - self._last_traj < TRAJ_MIN_PERIOD:
            return
        self._last_traj = now
        # Decimate the (unbounded, ever-growing) driven path to a fixed budget:
        # stride keeps the whole trail's shape while capping payload and the
        # browser's per-redraw cost. The last pose is always included so the
        # trail reaches the rover's current position.
        poses = msg.poses
        n = len(poses)
        step = max(1, (n + TRAJ_MAX_POINTS - 1) // TRAJ_MAX_POINTS)
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
            'level': LOG_LEVELS.get(msg.level, str(msg.level)),
            'node': msg.name,
            'text': msg.msg,
            'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
        })
