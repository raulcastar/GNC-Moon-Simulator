"""
Obstacles from the lidar, by height above the local ground. No ROS.

A scan is levelled (the lidar is pitched down), a ground plane is fitted to
its near field from the lowest points (absorbing terrain slope and rover
tilt), and every point higher than `min_height` above that plane is an
obstacle. Detections are thinned on a grid, moved to the odom frame with the
rover's pose, and remembered for a while: the lidar looks forward, so a rock
passed while turning leaves its view but must not be forgotten.

The planner uses them twice: the local planner avoids the remembered points,
and the map cells under them (grid_cells) go to the global planner, which
routes around them and re-plans when a new one blocks the route.
"""

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class DetectionSettings:
    pitch_deg: float = -20.0       # lidar pitch, negative = looking down
    x_offset_m: float = 0.0        # lidar ahead of the rover's odom point
    min_height_m: float = 0.15     # obstacle: this far above the local ground
    max_range_m: float = 6.0       # ignore points farther ahead than this
    half_width_m: float = 3.0      # ... or farther to the side than this
    self_radius_m: float = 0.45    # ignore points this close (the rover itself)
    fit_range_m: float = 8.0       # ground plane fitted on points nearer than this
    voxel_m: float = 0.15          # detections thinned on this grid


def level_points(points: np.ndarray, pitch_deg: float) -> np.ndarray:
    """
    Points (N, 3) in the lidar frame (x forward, y left, z up) rotated into a
    level frame: undoes the lidar's pitch (negative = looking down).
    """
    p = -math.radians(pitch_deg)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    return np.column_stack((x * math.cos(p) + z * math.sin(p), y,
                            -x * math.sin(p) + z * math.cos(p)))


def ground_heights(level: np.ndarray, fit_range_m: float, half_width_m: float,
                   iterations: int = 4, tolerance_m: float = 0.08):
    """
    Height of every point above a plane fitted to the near-field ground, or
    None when there are too few ground points. The fit keeps the points within
    `tolerance_m` of (or below) the previous plane, so rocks drop out of it.
    """
    x, y, z = level[:, 0], level[:, 1], level[:, 2]
    near = (x > 0.0) & (x < fit_range_m) & (np.abs(y) < half_width_m)
    keep = near.copy()
    heights = None
    for _ in range(iterations):
        if keep.sum() < 20:
            return None
        A = np.column_stack((np.ones(keep.sum()), x[keep], y[keep]))
        coef = np.linalg.lstsq(A, z[keep], rcond=None)[0]
        heights = z - (coef[0] + coef[1] * x + coef[2] * y)
        keep = near & (heights < tolerance_m)
    return heights


def detect(points: np.ndarray, settings: DetectionSettings) -> np.ndarray:
    """
    Obstacle positions (M, 2) in the rover frame (x forward, y left, metres
    from the odom point), thinned to one per `voxel_m` cell; points (N, 3)
    are a scan in the lidar frame (NaN rows allowed).
    """
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 20:
        return np.empty((0, 2))
    level = level_points(points, settings.pitch_deg)
    heights = ground_heights(level, settings.fit_range_m, settings.half_width_m)
    if heights is None:
        return np.empty((0, 2))
    x = level[:, 0] + settings.x_offset_m
    y = level[:, 1]
    hit = ((heights > settings.min_height_m) & (x > 0.0) & (x < settings.max_range_m)
           & (np.abs(y) < settings.half_width_m) & (np.hypot(x, y) > settings.self_radius_m))
    return thin(np.column_stack((x[hit], y[hit])), settings.voxel_m)


def thin(xy: np.ndarray, voxel_m: float) -> np.ndarray:
    """One point (the cell centre) per occupied `voxel_m` grid cell."""
    if len(xy) == 0:
        return np.empty((0, 2))
    cells = np.unique(np.floor(xy / voxel_m).astype(np.int64), axis=0)
    return (cells + 0.5) * voxel_m


def to_odom(xy_rover: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """Rover-frame points (x forward, y left) at pose (x, y, yaw) -> odom frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + c * xy_rover[:, 0] - s * xy_rover[:, 1],
                            y + s * xy_rover[:, 0] + c * xy_rover[:, 1]))


@dataclass
class ObstacleMemory:
    """Obstacle points in the odom frame, each kept `max_age_s` after it was last seen."""
    voxel_m: float = 0.15
    max_age_s: float = 20.0
    max_distance_m: float = 15.0
    _seen: dict = field(default_factory=dict)        # cell (i, j) -> last time seen

    def add(self, xy_odom: np.ndarray, t: float) -> None:
        for cell in map(tuple, np.floor(xy_odom / self.voxel_m).astype(np.int64)):
            self._seen[cell] = t

    def prune(self, t: float, x: float, y: float) -> None:
        """Forgets points older than max_age_s or farther than max_distance_m from (x, y)."""
        limit = (self.max_distance_m / self.voxel_m) ** 2
        cx, cy = x / self.voxel_m, y / self.voxel_m
        self._seen = {c: seen for c, seen in self._seen.items()
                      if t - seen <= self.max_age_s and (c[0] + 0.5 - cx) ** 2 + (c[1] + 0.5 - cy) ** 2 <= limit}

    def clear(self) -> None:
        self._seen.clear()

    def points(self) -> np.ndarray:
        if not self._seen:
            return np.empty((0, 2))
        return (np.array(list(self._seen), dtype=float) + 0.5) * self.voxel_m

    def __len__(self) -> int:
        return len(self._seen)


def grid_cells(xy_odom: np.ndarray, resolution: float, shape) -> set:
    """The global-map cells (col, row) under odom points (map cell = odom metres / resolution)."""
    if len(xy_odom) == 0:
        return set()
    cells = np.rint(xy_odom / resolution).astype(np.int64)
    rows, cols = shape
    inside = (cells[:, 0] >= 0) & (cells[:, 0] < cols) & (cells[:, 1] >= 0) & (cells[:, 1] < rows)
    return set(map(tuple, cells[inside]))
