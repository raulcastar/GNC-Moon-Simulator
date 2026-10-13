"""Mission logic (mission), map building (planner_maps) and messages (ros_conversions)."""

import json
import math
import types

import numpy as np
import pytest

from src import global_path_planner as gpp
from src import mission
from src.planner_maps import build_maps
from src.ros_conversions import finite_cost_map, matrix_msg, path_msg


def bordered_soil(shape):
    """Drivable soil (1) with the no-go border (0) the real soil maps have: the
    planner's search does not check the map bounds and relies on it."""
    soil = np.ones(shape)
    soil[0, :] = soil[-1, :] = soil[:, 0] = soil[:, -1] = 0
    return soil


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return types.SimpleNamespace(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                                 z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


def test_yaw_and_tilt_from_the_attitude():
    assert mission.yaw_from_quaternion(quat(yaw=1.2)) == pytest.approx(1.2)
    assert mission.tilt_deg(quat(yaw=2.0)) == pytest.approx(0.0, abs=1e-6)
    assert mission.tilt_deg(quat(pitch=math.radians(20))) == pytest.approx(20.0)
    assert mission.tilt_deg(quat(roll=math.radians(10), yaw=0.5)) == pytest.approx(10.0)


def test_session_jump():
    assert not mission.is_session_jump(None, 5.0, 5.0, 2.0)
    assert not mission.is_session_jump([0.0, 0.0], 1.0, 1.0, 2.0)
    assert mission.is_session_jump([0.0, 0.0], 3.0, 0.0, 2.0)


def test_trajectory_points_are_spaced():
    history = []
    for x in np.arange(0.0, 1.0, 0.03):
        mission.record_point(history, float(x), 0.0)
    gaps = np.diff([p[0] for p in history])
    assert len(history) > 5 and (gaps > mission.TRAJECTORY_SPACING_M).all()


def test_state_json():
    assert json.loads(mission.state_json('driving', [1.234, 5.678])) == \
        {'state': 'driving', 'goal': [1.23, 5.68], 'reason': ''}
    assert json.loads(mission.state_json('idle', None, 'new session'))['goal'] is None


def test_goal_snapping_on_a_small_map():
    obstacles = np.zeros((20, 20), int)
    obstacles[8:13, 8:13] = 1                        # a block
    obstacles[:, 16] = 1                             # a wall cutting off columns 17+
    regions = np.where(obstacles == 1, 0, 1)
    regions[:, 17:] = 2
    assert mission.drivable_goal(obstacles, regions, [2, 2], [0, 0]) == [2, 2]
    moved = mission.drivable_goal(obstacles, regions, [10, 10], [0, 0])
    assert moved is not None and obstacles[moved[1], moved[0]] == 0
    assert math.dist(moved, [10, 10]) <= mission.GOAL_SNAP_RADIUS_CELLS
    moved = mission.drivable_goal(obstacles, regions, [16, 5], [0, 0])
    assert moved == [15, 5]                          # not [17, 5]: the far side is unreachable
    assert mission.drivable_goal(obstacles, regions, [25, 5], [0, 0]) is None


def test_plan_route_on_a_small_map():
    elevation = np.zeros((40, 40))
    soil = bordered_soil((40, 40))
    planner = gpp.Global_Path_Planner()
    maps = build_maps(planner, elevation, soil, 1.0, 15.0)
    state, path, goal, reason, _message = mission.plan_route(
        planner, maps.cost, maps.obstacles, maps.free_regions, [5, 5], [30, 20], 15.0)
    assert state == 'driving' and reason == '' and goal == [30, 20]
    assert len(path) >= 2 and math.dist(path[-1], [30, 20]) < 2.0


def test_plan_route_refuses_an_unreachable_goal():
    planner = gpp.Global_Path_Planner()
    maps = build_maps(planner, np.zeros((30, 30)), bordered_soil((30, 30)), 1.0, 15.0)
    state, path, _goal, reason, _message = mission.plan_route(
        planner, maps.cost, maps.obstacles, maps.free_regions, [5, 5], [50, 5], 15.0)
    assert state == 'aborted' and path is None and reason == 'goal on impassable ground'


@pytest.mark.parametrize("dist, expected", [(10.0, 0.5), (4.5, 0.5), (2.25, 0.25), (0.5, 0.2)])
def test_approach_speed(dist, expected):
    assert mission.approach_speed(0.5, dist, 4.5) == pytest.approx(expected)


def test_approach_yaw_rate():
    assert mission.approach_yaw_rate(0.2, 10.0, 4.5, 0.7) == 0.2       # far: unchanged
    assert mission.approach_yaw_rate(0.2, 3.0, 4.5, 0.7) == pytest.approx(0.2 * 3.0 / 4.5)
    assert mission.approach_yaw_rate(0.2, 1.5, 4.5, 0.7) == 0.0       # final metre: locked
    assert mission.stop_tolerance(0.35, 0.7) == 0.7 and mission.stop_tolerance(0.9, 0.7) == 0.9


def test_robot_config_from_parameters():
    params = dict(max_speed=0.5, min_speed=0.0, max_accel=0.1, max_yaw_rate_deg=8.0,
                  max_delta_yaw_rate_deg=17.0, yaw_rate_resolution_deg=0.5, v_resolution=0.005,
                  dt=0.1, predict_time=1.5, to_goal_cost_gain=1.0, speed_cost_gain=1.0,
                  obstacle_cost_gain=1.0, robot_stuck_flag_cons=0.001, robot_width=0.35,
                  robot_length=0.35, robot_radius=0.35, obstacle_safety_m=0.15)
    conf = mission.robot_config(params.__getitem__)
    assert conf.max_yaw_rate == pytest.approx(math.radians(8.0))
    assert conf.v_resolution == 0.005 and conf.robot_type == 1


def test_planner_maps_regions_follow_the_obstacles():
    planner = gpp.Global_Path_Planner()
    soil = bordered_soil((20, 20))
    elevation = np.zeros((20, 20))
    elevation[:, 10] = 50.0                          # a cliff splits the map
    maps = build_maps(planner, elevation, soil, 1.0, 15.0)
    left, right = maps.free_regions[5, 2], maps.free_regions[5, 17]
    assert left and right and left != right
    assert (maps.obstacles[:, 9:12] == 1).any()


def test_messages():
    path = path_msg(np.array([[1.0, 2.0], [3.0, 4.0]]), None)
    assert path.header.frame_id == 'map' and [p.pose.position.y for p in path.poses] == [2.0, 4.0]
    assert path_msg([], None).poses == []
    m = matrix_msg(np.arange(6, dtype=float).reshape(2, 3))
    assert [d.size for d in m.layout.dim] == [2, 3] and list(m.data) == [0, 1, 2, 3, 4, 5]
    assert finite_cost_map(np.array([1.0, np.inf])).tolist() == [1.0, 9999.0]
