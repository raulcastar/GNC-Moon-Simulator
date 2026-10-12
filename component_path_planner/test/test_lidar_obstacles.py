"""Obstacles from the lidar by height (lidar_obstacles), and the DWA with and without them."""

import math
import types

import numpy as np
import pytest

from src import lidar_obstacles as lo
from src.local_trajectory_controller import Local_Trajectory_Controller

PITCH = -20.0
LIDAR_HEIGHT = 0.56


def scan(slope_deg=0.0, rocks=(), pitch_deg=PITCH, spacing=0.1, roll_noise=0.0):
    """
    A synthetic scan in the lidar frame: ground (rising `slope_deg` ahead)
    sampled every `spacing` m up to 10 m ahead and 4 m to each side, plus
    rocks given as (x, y, radius, height) boxes, seen from LIDAR_HEIGHT.
    """
    xs, ys = np.meshgrid(np.arange(0.3, 10.0, spacing), np.arange(-4.0, 4.0, spacing))
    xs, ys = xs.ravel(), ys.ravel()
    zs = xs * math.tan(math.radians(slope_deg)) - LIDAR_HEIGHT
    for x0, y0, radius, height in rocks:
        inside = np.hypot(xs - x0, ys - y0) <= radius
        zs[inside] += height
    rng = np.random.default_rng(0)
    zs = zs + rng.normal(scale=0.005, size=zs.shape)
    level = np.column_stack((xs, ys, zs))
    # into the lidar frame: rotate by the pitch (inverse of level_points)
    p = math.radians(pitch_deg)
    x, y, z = level[:, 0], level[:, 1], level[:, 2]
    return np.column_stack((x * math.cos(p) + z * math.sin(p), y, -x * math.sin(p) + z * math.cos(p)))


def test_levelling_undoes_the_pitch():
    level = np.array([[2.0, 0.5, -0.56]])
    back = lo.level_points(scan(rocks=())[:0].reshape(0, 3), PITCH)       # empty input is fine
    assert back.shape == (0, 3)
    p = math.radians(PITCH)
    lidar = np.array([[2.0 * math.cos(p) - 0.56 * math.sin(p), 0.5, -2.0 * math.sin(p) - 0.56 * math.cos(p)]])
    np.testing.assert_allclose(lo.level_points(lidar, PITCH), level, atol=1e-9)


@pytest.mark.parametrize("slope", [0.0, 8.0, -10.0])
def test_flat_or_sloped_ground_has_no_obstacles(slope):
    assert len(lo.detect(scan(slope_deg=slope), lo.DetectionSettings())) == 0


@pytest.mark.parametrize("slope", [0.0, 8.0])
def test_a_rock_is_found_where_it_is(slope):
    found = lo.detect(scan(slope_deg=slope, rocks=[(3.0, 0.5, 0.3, 0.4)]), lo.DetectionSettings())
    assert len(found) > 3
    assert np.allclose(found.mean(axis=0), [3.0, 0.5], atol=0.15)
    assert np.hypot(found[:, 0] - 3.0, found[:, 1] - 0.5).max() < 0.45


def test_low_rocks_and_far_rocks_are_ignored():
    settings = lo.DetectionSettings(min_height_m=0.15, max_range_m=6.0)
    assert len(lo.detect(scan(rocks=[(3.0, 0.0, 0.3, 0.08)]), settings)) == 0       # under 15 cm
    assert len(lo.detect(scan(rocks=[(8.0, 0.0, 0.4, 0.5)]), settings)) == 0        # beyond 6 m


def test_nan_rows_and_tiny_scans():
    points = scan(rocks=[(3.0, 0.0, 0.3, 0.4)])
    points[::7] = np.nan
    assert len(lo.detect(points, lo.DetectionSettings())) > 0
    assert len(lo.detect(np.full((5, 3), np.nan), lo.DetectionSettings())) == 0


def test_thin_and_odom_transform():
    thinned = lo.thin(np.array([[0.01, 0.01], [0.02, 0.03], [1.0, 1.0]]), 0.15)
    assert len(thinned) == 2
    odom = lo.to_odom(np.array([[2.0, 1.0]]), 10.0, 20.0, math.pi / 2)        # facing +y
    np.testing.assert_allclose(odom, [[9.0, 22.0]], atol=1e-9)


def test_memory_keeps_recent_nearby_points():
    memory = lo.ObstacleMemory(voxel_m=0.5, max_age_s=10.0, max_distance_m=5.0)
    memory.add(np.array([[1.0, 1.0], [1.1, 1.1], [3.0, 0.0]]), t=0.0)
    assert len(memory) == 2
    memory.add(np.array([[3.0, 0.0]]), t=8.0)
    memory.prune(12.0, 0.0, 0.0)                     # the first point is 12 s old
    np.testing.assert_allclose(memory.points(), [[3.25, 0.25]])
    memory.prune(12.0, 20.0, 0.0)                    # the rover moved away
    assert len(memory) == 0 and memory.points().shape == (0, 2)


def test_cells_keep_clear_and_blocked_route():
    cells = lo.grid_cells(np.array([[5.2, 3.9], [-1.0, 2.0], [100.0, 1.0]]), 1.0, (50, 50))
    assert cells == {(5, 4)}
    assert lo.keep_clear({(5, 4), (20, 20)}, [(5, 5)], 1.5) == {(20, 20)}
    path = np.column_stack((np.arange(0, 20, dtype=float), np.full(20, 4.0)))
    assert lo.path_blocked(path, {(5, 4)}, (0.0, 4.0), 6.0)
    assert not lo.path_blocked(path, {(5, 4)}, (0.0, 4.0), 4.0)        # beyond the look-ahead
    assert not lo.path_blocked(path, {(5, 4)}, (8.0, 4.0), 6.0)        # already passed
    assert not lo.path_blocked(path, set(), (0.0, 4.0), 6.0)


def test_mark_blocked_copies_the_maps():
    cost, obstacles = np.ones((5, 5)), np.zeros((5, 5))
    new_cost, new_obstacles = lo.mark_blocked(cost, obstacles, {(1, 3)}, 30.0)
    assert new_obstacles[3, 1] == 1 and new_cost[3, 1] == 30.0
    assert obstacles[3, 1] == 0 and cost[3, 1] == 1.0                   # originals untouched


def dwa_config():
    return types.SimpleNamespace(
        max_speed=0.5, min_speed=0.0, max_accel=0.5, max_yaw_rate=math.radians(40),
        max_delta_yaw_rate=math.radians(40), yaw_rate_resolution=math.radians(1.0),
        v_resolution=0.05, dt=0.1, predict_time=2.0, to_goal_cost_gain=1.0, speed_cost_gain=1.0,
        obstacle_cost_gain=1.0, robot_stuck_flag_cons=0.001, robot_type=1,
        robot_width=0.35, robot_length=0.35, robot_radius=0.35)


def test_dwa_without_obstacles_drives_straight_at_the_goal():
    dwa = Local_Trajectory_Controller()
    u, _trajectory = dwa.dwa_control(np.array([0.0, 0.0, 0.0, 0.3, 0.0]), dwa_config(),
                                     np.array([5.0, 0.0]), np.empty((0, 2)))
    assert u[0] > 0.0 and abs(u[1]) < math.radians(2)


def rock_at(x0, y0=0.0, half=0.15):
    return lo.thin(np.array([[x0 + dx, y0 + dy] for dx in np.arange(-half, half, 0.05)
                             for dy in np.arange(-half, half, 0.05)]), 0.1)


def test_dwa_steers_away_from_a_rock_ahead():
    dwa = Local_Trajectory_Controller()
    config = dwa_config()
    # A turn window wide enough to clear it (+-30 deg/s reachable this step)
    config.predict_time, config.max_delta_yaw_rate = 4.0, math.radians(300)
    rock = rock_at(1.15)
    state = np.array([0.0, 0.0, 0.0, 0.3, 0.0])
    u, trajectory = dwa.dwa_control(state, config, np.array([5.0, 0.0]), rock)
    clearance = np.min(np.hypot(trajectory[:, 0, None] - rock[:, 0], trajectory[:, 1, None] - rock[:, 1]))
    assert abs(u[1]) > math.radians(2)               # turns, where it drove straight without the rock
    assert clearance > 0.175                         # the chosen path keeps the footprint off the rock


def test_dwa_stops_when_every_path_hits():
    dwa = Local_Trajectory_Controller()
    wall = np.array([[0.5, y] for y in np.arange(-3.0, 3.0, 0.1)])
    u, trajectory = dwa.dwa_control(np.array([0.0, 0.0, 0.0, 0.3, 0.0]), dwa_config(),
                                    np.array([5.0, 0.0]), wall)
    assert u == [0.0, 0.0] and len(trajectory) == 1
