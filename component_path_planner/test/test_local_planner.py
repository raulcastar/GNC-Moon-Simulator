"""The local planner (local_planner): the DWA, route following and the final approach."""

import dataclasses
import math

import numpy as np
import pytest

from src import local_planner as lp

# The rover's configuration (config/params.yaml)
ROVER = lp.DWAConfig(
    max_speed=0.5, min_speed=0.0, max_accel=0.1, max_yaw_rate=math.radians(8.0),
    max_delta_yaw_rate=math.radians(17.0), v_resolution=0.005, yaw_rate_resolution=math.radians(0.5),
    dt=0.1, predict_time=1.5, to_goal_cost_gain=1.0, speed_cost_gain=1.0, robot_stuck_flag_cons=0.001,
    robot_length=0.65, robot_width=0.65, obstacle_safety_m=0.15,
    lookahead_dist=4.0, braking_distance=4.5, stop_tolerance=0.7)

# Agile and with a small footprint: the DWA's choices show within one step.
AGILE = dataclasses.replace(
    ROVER, max_accel=0.5, max_yaw_rate=math.radians(40), max_delta_yaw_rate=math.radians(40),
    yaw_rate_resolution=math.radians(1.0), v_resolution=0.05, predict_time=2.0,
    robot_length=0.35, robot_width=0.35, obstacle_safety_m=0.0)


def rock_at(x0, y0=0.0, half=0.15):
    return np.array([[x0 + dx, y0 + dy] for dx in np.arange(-half, half, 0.1) for dy in np.arange(-half, half, 0.1)])


# --- the DWA ----------------------------------------------------------------

def test_without_obstacles_it_drives_straight_at_the_goal():
    v, w, _ = lp.dwa(np.array([0.0, 0.0, 0.0, 0.3, 0.0]), np.array([5.0, 0.0]), np.empty((0, 2)), AGILE, 0.5)
    assert v > 0.0 and abs(w) < math.radians(2)


def test_it_steers_away_from_a_rock_ahead():
    config = dataclasses.replace(AGILE, predict_time=4.0, max_delta_yaw_rate=math.radians(300))
    rock = rock_at(1.15)
    v, w, trajectory = lp.dwa(np.array([0.0, 0.0, 0.0, 0.3, 0.0]), np.array([5.0, 0.0]), rock, config, 0.5)
    clearance = np.min(np.hypot(trajectory[:, 0, None] - rock[:, 0], trajectory[:, 1, None] - rock[:, 1]))
    assert abs(w) > math.radians(2)                  # turns, where it drove straight without the rock
    assert clearance > 0.175                         # the footprint stays off the rock


def test_it_stops_when_every_trajectory_hits():
    wall = np.array([[0.5, y] for y in np.arange(-3.0, 3.0, 0.1)])
    v, w, trajectory = lp.dwa(np.array([0.0, 0.0, 0.0, 0.3, 0.0]), np.array([5.0, 0.0]), wall, AGILE, 0.5)
    assert (v, w) == (0.0, 0.0) and len(trajectory) == 1


def test_it_moves_off_from_rest_beside_a_rock():
    """The stall seen in Unreal: at rest 0.85 m from a rock with the route
    leading past it. A cost growing near rocks made standing still cheapest."""
    rock = np.array([[0.6 + dx, 0.6 + dy] for dx in (0.0, 0.1, 0.2) for dy in (0.0, 0.1, 0.2)])
    v, _, _ = lp.dwa(np.zeros(5), np.array([4.0, -0.5]), rock, ROVER, 0.5)
    assert v > 0.0


def test_the_window_is_one_cycle_of_acceleration():
    v_min, v_max, w_min, w_max = lp.dynamic_window(np.array([0, 0, 0, 0.3, 0.0]), ROVER, 0.5)
    assert (v_min, v_max) == pytest.approx((0.29, 0.31))
    assert w_max == pytest.approx(ROVER.max_delta_yaw_rate * ROVER.dt)
    assert lp.dynamic_window(np.array([0, 0, 0, 0.5, 0.0]), ROVER, 0.2)[1] == 0.2   # the approach limit


def test_obstacle_cost_is_refuse_or_allow():
    straight = lp.predict_trajectory(np.zeros(5), 0.5, 0.0, ROVER)                   # 0.8 m ahead
    assert lp.obstacle_cost(straight, np.empty((0, 2)), ROVER) == 0.0
    assert lp.obstacle_cost(straight, np.array([[1.2, 0.0]]), ROVER) == math.inf     # within the safety
    assert lp.obstacle_cost(straight, np.array([[1.4, 0.0]]), ROVER) == 0.0
    assert lp.obstacle_cost(straight, np.array([[0.4, 0.6]]), ROVER) == 0.0          # beside, clear


# --- following the route ----------------------------------------------------

def test_lookahead_point():
    route = np.column_stack((np.arange(0.0, 10.0), np.zeros(10)))
    assert list(lp.lookahead_point((2.2, 0.3), route, 4.0)) == [7.0, 0.0]   # the first point 4 m away
    assert list(lp.lookahead_point((8.5, 0.0), route, 4.0)) == [9.0, 0.0]   # near the end: the last point


def test_command_follows_the_route():
    route = np.column_stack((np.arange(0.0, 20.0), np.zeros(20)))
    cmd = lp.LocalPlanner(ROVER).command((0.0, 0.0, 0.0), (0.3, 0.0), route, (19.0, 0.0), np.empty((0, 2)))
    assert cmd.v >= 0.29 and abs(cmd.w) < 0.02 and cmd.trajectory.shape[1] == 5


def test_config_from_parameters():
    params = dict(max_speed=0.5, min_speed=0.0, max_accel=0.1, max_yaw_rate_deg=8.0,
                  max_delta_yaw_rate_deg=17.0, yaw_rate_resolution_deg=0.5, v_resolution=0.005,
                  dt=0.1, predict_time=1.5, to_goal_cost_gain=1.0, speed_cost_gain=1.0,
                  robot_stuck_flag_cons=0.001, robot_width=0.65, robot_length=0.65,
                  obstacle_safety_m=0.15, lookahead_dist=4.0, braking_distance=4.5)
    config = lp.DWAConfig.from_params(params.__getitem__, 0.7)
    assert config == ROVER


# --- the final approach -----------------------------------------------------

@pytest.mark.parametrize("dist, expected", [(10.0, 0.5), (4.5, 0.5), (2.25, 0.25), (0.5, 0.2)])
def test_approach_speed(dist, expected):
    assert lp.approach_speed(0.5, dist, 4.5) == pytest.approx(expected)


def test_approach_yaw_rate():
    assert lp.approach_yaw_rate(0.2, 10.0, 4.5, 0.7) == 0.2           # far: unchanged
    assert lp.approach_yaw_rate(0.2, 3.0, 4.5, 0.7) == pytest.approx(0.2 * 3.0 / 4.5)
    assert lp.approach_yaw_rate(0.2, 1.5, 4.5, 0.7) == 0.0            # final metre: locked


def test_approach_lets_an_off_course_rover_turn_to_the_goal():
    assert lp.approach_yaw_rate(0.1, 1.5, 4.5, 0.7, math.radians(5)) == 0.0
    # arriving 60 deg off course: the turn is kept, so it does not circle the goal
    assert lp.approach_yaw_rate(0.1, 1.5, 4.5, 0.7, math.radians(60)) == 0.1
