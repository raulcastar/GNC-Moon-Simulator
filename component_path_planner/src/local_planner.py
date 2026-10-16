"""
Local planner: the velocity command that follows the route, by the Dynamic
Window Approach (DWA). No ROS.

Based on the PythonRobotics DWA sample by Atsushi Sakai and Goktug Karakasli
(https://github.com/AtsushiSakai/PythonRobotics, PathPlanning/DynamicWindowApproach),
used under its MIT License: Copyright (c) 2016 - now Atsushi Sakai and other
contributors. The full licence text is reproduced at the end of this module.

Each control cycle, command():
 1. The local goal is the first route point lookahead_dist from the rover.
 2. Every (v, w) the rover can reach within one cycle (the dynamic window) is
    simulated for predict_time. A trajectory that brings the footprint within
    obstacle_safety_m of an obstacle is refused; of the rest, the one heading
    best at the local goal at the highest speed is chosen.
 3. Near the goal (final approach) the speed limit falls over the braking
    distance, and the turn rate is damped while the rover heads at the goal,
    so it parks instead of overshooting or spinning over it.

Keeping a distance from rocks is the global route's job (planned with a
clearance); the DWA only refuses to hit them. A cost growing as the rover
nears a rock would outweigh the speed the window offers from rest
(max_accel * dt) and leave the rover stopped beside rocks.
"""

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

# Within this heading error the final approach damps the turn rate.
APPROACH_ALIGNED_RAD = math.radians(20.0)
# The final approach's speed limit does not drop below this (m/s).
APPROACH_MIN_SPEED = 0.2


@dataclass
class DWAConfig:
    # Speed and turn limits
    max_speed: float            # m/s
    min_speed: float            # m/s
    max_accel: float            # m/s^2
    max_yaw_rate: float         # rad/s
    max_delta_yaw_rate: float   # rad/s^2
    # Sampling and prediction
    v_resolution: float         # m/s; below max_accel * dt or only v = 0 is sampled
    yaw_rate_resolution: float  # rad/s
    dt: float                   # s
    predict_time: float         # s
    # Costs
    to_goal_cost_gain: float
    speed_cost_gain: float
    robot_stuck_flag_cons: float
    # Footprint (a rectangle around the odometry point)
    robot_length: float         # m
    robot_width: float          # m
    obstacle_safety_m: float    # m around the footprint
    # Route following and the final approach
    lookahead_dist: float       # m
    braking_distance: float     # m
    stop_tolerance: float       # m

    @classmethod
    def from_params(cls, param: Callable[[str], object], stop_tolerance: float) -> 'DWAConfig':
        """From the node's parameters (`param(name)` -> value)."""
        return cls(
            max_speed=param('max_speed'), min_speed=param('min_speed'), max_accel=param('max_accel'),
            max_yaw_rate=math.radians(param('max_yaw_rate_deg')),
            max_delta_yaw_rate=math.radians(param('max_delta_yaw_rate_deg')),
            v_resolution=param('v_resolution'),
            yaw_rate_resolution=math.radians(param('yaw_rate_resolution_deg')),
            dt=param('dt'), predict_time=param('predict_time'),
            to_goal_cost_gain=param('to_goal_cost_gain'), speed_cost_gain=param('speed_cost_gain'),
            robot_stuck_flag_cons=param('robot_stuck_flag_cons'),
            robot_length=param('robot_length'), robot_width=param('robot_width'),
            obstacle_safety_m=param('obstacle_safety_m'),
            lookahead_dist=param('lookahead_dist'), braking_distance=param('braking_distance'),
            stop_tolerance=stop_tolerance)


@dataclass
class Command:
    v: float                    # m/s
    w: float                    # rad/s
    trajectory: np.ndarray      # the predicted trajectory (N, 5: x, y, yaw, v, w)


class LocalPlanner:
    def __init__(self, config: DWAConfig):
        self.config = config

    def command(self, pose, velocity, route: np.ndarray, goal, obstacles: np.ndarray) -> Command:
        """
        The command at `pose` (x, y, yaw) moving at `velocity` (v, w), following
        `route` (N, 2) to `goal` (x, y), not hitting `obstacles` (M, 2); metres.
        """
        c = self.config
        x, y, yaw = pose
        dist_to_goal = math.hypot(goal[0] - x, goal[1] - y)
        local_goal = lookahead_point((x, y), route, c.lookahead_dist)
        max_speed = approach_speed(c.max_speed, dist_to_goal, c.braking_distance)

        state = np.array([x, y, yaw, velocity[0], velocity[1]], dtype=float)
        v, w, trajectory = dwa(state, np.asarray(local_goal), obstacles, c, max_speed)

        heading_error = wrap_angle(math.atan2(goal[1] - y, goal[0] - x) - yaw)
        w = approach_yaw_rate(w, dist_to_goal, c.braking_distance, c.stop_tolerance, heading_error)
        return Command(float(v), float(w), trajectory)


# =============================================================================
# Following the route
# =============================================================================

def lookahead_point(rover_xy, route: np.ndarray, lookahead_dist: float):
    """From the route point nearest the rover, the first one at least lookahead_dist from it (else the last)."""
    distances = np.hypot(route[:, 0] - rover_xy[0], route[:, 1] - rover_xy[1])
    i = int(np.argmin(distances))
    while i < len(route) - 1 and distances[i] < lookahead_dist:
        i += 1
    return route[i]


# =============================================================================
# The Dynamic Window Approach
# =============================================================================

def dwa(state: np.ndarray, local_goal: np.ndarray, obstacles: np.ndarray, c: DWAConfig,
        max_speed: float):
    """
    The best (v, w) and its predicted trajectory, from `state` [x, y, yaw, v, w];
    (0, 0) when every reachable trajectory would hit an obstacle.
    """
    v_min, v_max, w_min, w_max = dynamic_window(state, c, max_speed)
    best_cost, best_u, best_trajectory = math.inf, (0.0, 0.0), np.array([state])
    for v in np.arange(v_min, v_max, c.v_resolution):
        for w in np.arange(w_min, w_max, c.yaw_rate_resolution):
            trajectory = predict_trajectory(state, v, w, c)
            heading = c.to_goal_cost_gain * heading_cost(trajectory, local_goal)
            target_speed = max_speed * math.exp(-heading)
            speed = c.speed_cost_gain * abs(target_speed - trajectory[-1, 3])
            cost = heading + speed + obstacle_cost(trajectory, obstacles, c)
            if best_cost >= cost:
                best_cost, best_u, best_trajectory = cost, (v, w), trajectory

    if not math.isfinite(best_cost):
        return 0.0, 0.0, np.array([state])
    v, w = best_u
    if abs(v) < c.robot_stuck_flag_cons and abs(state[3]) < c.robot_stuck_flag_cons:
        # Standing still with nowhere better to go (an obstacle ahead, or
        # already facing the goal): turn, so the rover does not stay stuck.
        w = -c.max_delta_yaw_rate
    return v, w, best_trajectory


def dynamic_window(state: np.ndarray, c: DWAConfig, max_speed: float):
    """(v_min, v_max, w_min, w_max): the limits, narrowed to what one cycle's acceleration reaches."""
    v, w = state[3], state[4]
    return (max(c.min_speed, v - c.max_accel * c.dt), min(max_speed, v + c.max_accel * c.dt),
            max(-c.max_yaw_rate, w - c.max_delta_yaw_rate * c.dt),
            min(c.max_yaw_rate, w + c.max_delta_yaw_rate * c.dt))


def predict_trajectory(state: np.ndarray, v: float, w: float, c: DWAConfig) -> np.ndarray:
    """The trajectory (N, 5) driving (v, w) from `state` for predict_time, every dt."""
    dt = c.dt
    n = _steps(c.predict_time, dt)
    trajectory = np.empty((n + 1, 5))
    trajectory[0] = state
    yaw = state[2] + w * dt * np.arange(1, n + 1)
    trajectory[1:, 0] = state[0] + np.cumsum(v * np.cos(yaw) * dt)
    trajectory[1:, 1] = state[1] + np.cumsum(v * np.sin(yaw) * dt)
    trajectory[1:, 2] = yaw
    trajectory[1:, 3] = v
    trajectory[1:, 4] = w
    return trajectory


def _steps(predict_time: float, dt: float) -> int:
    """The number of steps a `time += dt` loop takes up to predict_time."""
    n, time = 0, 0
    while time <= predict_time:
        n += 1
        time += dt
    return n


def heading_cost(trajectory: np.ndarray, local_goal) -> float:
    """The angle (rad) between the trajectory's final heading and the direction to the local goal."""
    x, y, yaw = trajectory[-1, :3]
    return abs(wrap_angle(math.atan2(local_goal[1] - y, local_goal[0] - x) - yaw))


def obstacle_cost(trajectory: np.ndarray, obstacles: np.ndarray, c: DWAConfig) -> float:
    """inf if the trajectory brings the footprint within obstacle_safety_m of an obstacle, else 0."""
    if len(obstacles) == 0:
        return 0.0
    half_l = c.robot_length / 2 + c.obstacle_safety_m
    half_w = c.robot_width / 2 + c.obstacle_safety_m
    distances = np.hypot(trajectory[:, 0] - obstacles[:, 0, None], trajectory[:, 1] - obstacles[:, 1, None])
    if distances.min() > math.hypot(half_l, half_w):
        return 0.0                              # beyond the footprint's corners everywhere
    # Obstacles in the footprint's frame at every pose of the trajectory
    yaw = trajectory[:, 2]
    rotations = np.transpose(np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]), [2, 0, 1])
    relative = (obstacles[:, None] - trajectory[:, 0:2]).reshape(-1, 2)
    local = np.array([relative @ r for r in rotations]).reshape(-1, 2)
    inside = (np.abs(local[:, 0]) <= half_l) & (np.abs(local[:, 1]) <= half_w)
    return math.inf if inside.any() else 0.0


# =============================================================================
# The final approach
# =============================================================================

def approach_speed(max_speed: float, dist_to_goal: float, braking_distance: float) -> float:
    """The speed limit: max_speed, falling linearly over the braking distance (to APPROACH_MIN_SPEED)."""
    if dist_to_goal < braking_distance:
        return max(max_speed * (dist_to_goal / braking_distance), APPROACH_MIN_SPEED)
    return max_speed


def approach_yaw_rate(w: float, dist_to_goal: float, braking_distance: float,
                      stop_tolerance: float, heading_error: float = 0.0) -> float:
    """
    The turn rate near the goal: damped over the braking distance and zero
    within a metre of the stop tolerance, so the rover does not spin over the
    goal (its bearing turns unstable there). Only while it heads at the goal
    (within APPROACH_ALIGNED_RAD): arriving off course, it must still turn, or
    it circles the goal at crawling speed.
    """
    if abs(heading_error) > APPROACH_ALIGNED_RAD or dist_to_goal >= braking_distance:
        return w
    if dist_to_goal < stop_tolerance + 1.0:
        return 0.0
    return w * dist_to_goal / braking_distance


def wrap_angle(a: float) -> float:
    """The angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# -----------------------------------------------------------------------------
# PythonRobotics licence (the DWA sample this planner is based on)
# -----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2016 - now Atsushi Sakai and other contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
