"""Pure helpers on point clouds and DEMs (no ROS node state)."""

import math

import numpy as np

from sensor_msgs.msg import PointCloud2


def cloud_xyz(msg: PointCloud2, max_points: int) -> np.ndarray:
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


def ground_plane(dem: np.ndarray, res: float) -> np.ndarray:
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


def level_points(xyz: np.ndarray, roll: float, pitch: float) -> np.ndarray:
    """Removes body roll and pitch (radians, REP-103) from rover-frame points."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return xyz @ (ry @ rx).T
