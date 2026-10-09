"""Dashboard helpers without node state: the topic preview, cloud parsing, ground plane, levelling."""
import math

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String

from src.geometry import cloud_xyz, ground_plane, level_points
from src.settings import INSPECT_MAX_STRING, INSPECT_TRUNCATE
from src.topics import message_preview


# ---------------------------------------------------------------- message preview

def test_preview_nested_message():
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = 1.5
    out = message_preview(msg)
    assert out["header"]["frame_id"] == "map"
    assert out["pose"]["position"] == {"x": 1.5, "y": 0.0, "z": 0.0}
    assert out["pose"]["orientation"]["w"] == 1.0


def test_preview_truncates_large_arrays():
    img = Image(height=100, width=100, encoding="rgb8", step=300)
    img.data = bytes(range(256)) * 117                     # ~30 kB of pixels
    out = message_preview(img)["data"]
    assert len(out) == INSPECT_TRUNCATE + 1
    assert out[:3] == [0, 1, 2]
    assert out[-1] == f"... {len(img.data)} elements in all"


def test_preview_keeps_short_arrays_whole():
    img = Image()
    img.data = bytes([5, 6, 7])
    assert message_preview(img)["data"] == [5, 6, 7]


def test_preview_long_strings_are_cut():
    out = message_preview(String(data="x" * (INSPECT_MAX_STRING + 50)))["data"]
    assert out == "x" * INSPECT_MAX_STRING + "..."
    assert message_preview(String(data="short"))["data"] == "short"


def test_preview_numpy_scalars_become_plain_numbers():
    assert message_preview(np.float32(2.5)) == 2.5 and isinstance(message_preview(np.int16(3)), int)


# ---------------------------------------------------------------- point clouds

def cloud(xyz, extra=4):
    """PointCloud2 with x, y, z float32 fields and `extra` padding bytes per point."""
    step = 12 + extra
    data = np.zeros((len(xyz), step), np.uint8)
    data[:, :12] = np.asarray(xyz, "<f4").view(np.uint8).reshape(len(xyz), 12)
    msg = PointCloud2(height=1, width=len(xyz), point_step=step, row_step=step * len(xyz))
    msg.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                  for k, n in enumerate("xyz")]
    msg.data = data.tobytes()
    return msg


def test_cloud_xyz_drops_non_finite_points():
    xyz = np.array([[1, 2, 3], [np.nan, 0, 0], [4, 5, 6], [np.inf, 1, 1]], np.float32)
    out = cloud_xyz(cloud(xyz), 100)
    assert np.array_equal(out, xyz[[0, 2]])


def test_cloud_xyz_strides_down_to_the_budget():
    xyz = np.arange(3000, dtype=np.float32).reshape(1000, 3)
    out = cloud_xyz(cloud(xyz), 100)
    assert len(out) == 100 and np.array_equal(out[1], xyz[10])


def test_cloud_xyz_without_xyz_fields():
    msg = cloud(np.zeros((3, 3), np.float32))
    msg.fields = msg.fields[:2]
    assert cloud_xyz(msg, 10).shape == (0, 3)


# ---------------------------------------------------------------- ground plane

def plane_dem(a=0.05, b=-0.02, c=-0.5, res=0.05, shape=(120, 60), seed=0):
    rows, cols = np.mgrid[0:shape[0], 0:shape[1]]
    x = (cols - (shape[1] - 1) / 2.0) * res
    y = (rows + 0.5) * res
    rng = np.random.default_rng(seed)
    return a * x + b * y + c + rng.normal(0, 0.002, shape)


def test_ground_plane_recovers_the_slope():
    coef = ground_plane(plane_dem(), 0.05)
    assert coef == pytest.approx([0.05, -0.02, -0.5], abs=0.003)


def test_ground_plane_ignores_rocks_and_gaps():
    dem = plane_dem()
    dem[40:55, 20:35] += 0.6                    # a rock
    dem[90:, :] = np.nan                        # unobserved
    coef = ground_plane(dem, 0.05)
    assert coef == pytest.approx([0.05, -0.02, -0.5], abs=0.005)


def test_ground_plane_with_too_few_cells():
    dem = np.full((50, 50), np.nan)
    dem[0, :10] = 0.0
    assert np.array_equal(ground_plane(dem, 0.05), np.zeros(3))


# ---------------------------------------------------------------- levelling

def test_level_points_undoes_roll_and_pitch():
    """Points on a tilted plane through the rover come out level."""
    roll, pitch = math.radians(7.0), math.radians(-12.0)
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    level = np.random.default_rng(1).uniform(-5, 5, (200, 3))
    level[:, 2] = -0.55                         # ground 0.55 m below the sensor
    body = level @ np.linalg.inv(ry @ rx).T     # as the tilted rover sees it
    assert np.allclose(level_points(body, roll, pitch), level, atol=1e-9)


def test_level_points_identity_when_level():
    xyz = np.random.default_rng(2).normal(size=(10, 3))
    assert np.allclose(level_points(xyz, 0.0, 0.0), xyz)
