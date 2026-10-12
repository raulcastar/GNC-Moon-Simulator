"""The navcam overlay's path-ahead resampling and goal bearing (imagery.path_ahead, goal_bearing)."""

import math

import numpy as np

from src.imagery import goal_bearing, path_ahead

LEVEL = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)          # at the origin, facing +x


def yawed(deg):
    h = math.radians(deg) / 2
    return (0.0, 0.0, 0.0, 0.0, 0.0, math.sin(h), math.cos(h))


def test_path_ahead_starts_at_the_rover_and_skips_passed_vertices():
    path = np.array([[-5.0, 0.0], [0.0, 0.0], [10.0, 0.0]])
    x, y, d = path_ahead(path, 1.0, 0.5, ahead=30.0, step=1.0)
    assert (x[0], y[0]) == (1.0, 0.5)
    assert d[0] == 0.0 and np.all(np.diff(d) == 1.0)
    assert x.min() >= 1.0                      # nothing behind the rover


def test_path_ahead_is_cut_at_the_look_ahead():
    path = np.array([[0.0, 0.0], [100.0, 0.0]])
    _, _, d = path_ahead(path, 0.0, 0.0, ahead=30.0, step=0.25)
    assert math.isclose(d[-1], 30.0)


def test_path_ahead_none_at_the_end_or_without_a_path():
    assert path_ahead(None, 0.0, 0.0) is None
    assert path_ahead(np.array([[0.0, 0.0], [5.0, 0.0]]), 5.0, 0.0) is None


def test_goal_bearing_left_right_behind():
    assert goal_bearing(LEVEL, (10.0, 0.0)) == (10.0, 0.0)
    dist, left = goal_bearing(LEVEL, (0.0, 5.0))
    assert dist == 5.0 and math.isclose(left, 90.0)
    _, right = goal_bearing(LEVEL, (3.0, -3.0))
    assert math.isclose(right, -45.0)
    _, behind = goal_bearing(yawed(90.0), (0.0, -4.0))
    assert math.isclose(abs(behind), 180.0)
