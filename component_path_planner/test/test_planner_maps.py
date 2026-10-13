"""The planner's maps (planner_maps) and ROS messages (ros_conversions)."""

import bisect

import numpy as np

from src.planner_maps import SLOPE_COST_MAX, build_maps, slope_cost
from src.ros_conversions import finite_cost_map, matrix_msg, path_msg


def bordered_soil(shape):
    soil = np.ones(shape)
    soil[0, :] = soil[-1, :] = soil[:, 0] = soil[:, -1] = 0
    return soil


def original_cost(slope):
    """DyMu's getCost, one slope at a time: the reference for slope_cost."""
    lut = [[0, 5, 10, 15, 20], [10.0, 12.0, 15.0, 20.0, 40.0]]
    if slope >= lut[0][-1]:
        return 50.0
    i = bisect.bisect_left(lut[0], slope)
    fraction = (slope - lut[0][i - 1]) / (lut[0][i] - lut[0][i - 1])
    return fraction * (lut[1][i] - lut[1][i - 1]) + lut[1][i - 1]


def test_slope_cost_matches_the_original():
    slopes = np.concatenate(([0.0, 5.0, 10.0, 15.0, 19.999, 20.0, 35.0], np.random.default_rng(0).uniform(0, 30, 500)))
    expected = np.array([original_cost(s) for s in slopes])
    assert np.array_equal(slope_cost(slopes), expected)
    assert slope_cost(np.array([25.0]))[0] == SLOPE_COST_MAX


def test_maps_regions_follow_the_obstacles():
    elevation = np.zeros((20, 20))
    elevation[:, 10] = 50.0                          # a cliff splits the map
    maps = build_maps(elevation, bordered_soil((20, 20)), 1.0, 15.0)
    left, right = maps.free_regions[5, 2], maps.free_regions[5, 17]
    assert left and right and left != right
    assert (maps.obstacles[:, 9:12] == 1).any()
    assert (maps.obstacles[0, :] == 1).all()         # no-go soil border
    assert np.isfinite(maps.cost).all()


def test_messages():
    path = path_msg(np.array([[1.0, 2.0], [3.0, 4.0]]), None)
    assert path.header.frame_id == 'map' and [p.pose.position.y for p in path.poses] == [2.0, 4.0]
    assert path_msg([], None).poses == []
    m = matrix_msg(np.arange(6, dtype=float).reshape(2, 3))
    assert [d.size for d in m.layout.dim] == [2, 3] and list(m.data) == [0, 1, 2, 3, 4, 5]
    assert finite_cost_map(np.array([1.0, np.inf])).tolist() == [1.0, 9999.0]
