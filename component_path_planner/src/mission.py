"""
Mission logic of the planner node: goals, routes, the final approach and the
rover's attitude. No ROS.
"""

import json
import math
from types import SimpleNamespace
from typing import Callable, List, Optional

import numpy as np

from src import global_path_planner

# A goal on impassable ground is moved to the nearest drivable cell within this
# many grid cells (metres at 1 m/px), if the rover can reach it.
GOAL_SNAP_RADIUS_CELLS = 5

# Driven trajectory points are kept this far apart (metres).
TRAJECTORY_SPACING_M = 0.1


# =========================================================================
# Attitude
# =========================================================================

def yaw_from_quaternion(q) -> float:
    """Yaw (rad) of a geometry_msgs-like quaternion (x, y, z, w attributes)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def tilt_deg(q) -> float:
    """Combined roll/pitch tilt (deg): the angle between the body z axis and vertical."""
    cz = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
    return math.degrees(math.acos(cz))


def is_session_jump(previous_pose, x: float, y: float, limit_m: float) -> bool:
    """
    Whether the rover moved more than limit_m in one odometry step: a Play/Stop
    in Unreal teleports it back to its start pose.
    """
    return (previous_pose is not None
            and math.hypot(x - previous_pose[0], y - previous_pose[1]) > limit_m)


def record_point(history: list, x: float, y: float, spacing: float = TRAJECTORY_SPACING_M) -> bool:
    """Appends (x, y) to the driven trajectory if it is `spacing` from the last point."""
    if not history or math.hypot(x - history[-1][0], y - history[-1][1]) > spacing:
        history.append([x, y])
        return True
    return False


# =========================================================================
# Goals and routes
# =========================================================================

def state_json(state: str, goal, reason: str = '') -> str:
    """
    The mission state as published on mission_status/state:
    {"state": idle|planning|driving|reached|aborted|cancelled,
     "goal": [x, y] or null, "reason": str}.
    """
    goal = [round(float(v), 2) for v in goal] if goal else None
    return json.dumps({'state': state, 'goal': goal, 'reason': reason})


def drivable_goal(obstacle_map: np.ndarray, free_regions: Optional[np.ndarray],
                  goal: list, start: list, radius: int = GOAL_SNAP_RADIUS_CELLS) -> Optional[list]:
    """
    `goal` (map cells) if it is drivable, else the nearest drivable cell within
    `radius` that lies in the rover's drivable region (so a path can exist);
    None if there is none.
    """
    rows, cols = obstacle_map.shape
    gx, gy = goal
    if not (0 <= gx < cols and 0 <= gy < rows):
        return None
    if obstacle_map[gy, gx] != 1:
        return goal
    r = radius
    y0, y1, x0, x1 = max(0, gy - r), min(rows, gy + r + 1), max(0, gx - r), min(cols, gx + r + 1)
    free = obstacle_map[y0:y1, x0:x1] != 1
    sx, sy = start
    if free_regions is not None and 0 <= sx < cols and 0 <= sy < rows and free_regions[sy, sx]:
        free &= free_regions[y0:y1, x0:x1] == free_regions[sy, sx]
    ys, xs = np.nonzero(free)
    if ys.size == 0:
        return None
    d2 = (ys + y0 - gy) ** 2 + (xs + x0 - gx) ** 2
    k = int(np.argmin(d2))
    if d2[k] > r * r:
        return None
    return [int(xs[k] + x0), int(ys[k] + y0)]


def plan_route(planner: global_path_planner.Global_Path_Planner, cost_map: np.ndarray,
               obstacle_map: np.ndarray, free_regions: Optional[np.ndarray],
               start_pos: list, goal: list, slope_threshold: float,
               log_info: Callable[[str], None] = lambda message: None):
    """
    The route from `start_pos` to `goal` (map cells). Returns (state, path,
    goal, reason, message): state 'driving' with the path and the goal actually
    used (moved off impassable ground if need be), or 'aborted' with the reason.
    Raises global_path_planner.PlanningCancelled if the planner is told to stop.
    """
    # Ensure target coordinates are integers to prevent NumPy matrix indexing faults
    grid_goal = [int(round(goal[0])), int(round(goal[1]))]

    # A goal on impassable ground (too steep, or no-go soil) would leave the
    # whole search stuck at the goal: move it to the nearest drivable cell
    # the rover can reach, or refuse it.
    snapped = drivable_goal(obstacle_map, free_regions, grid_goal, start_pos)
    if snapped is None:
        return ('aborted', None, None, 'goal on impassable ground',
                f"Goal {grid_goal} is on impassable ground (slope over "
                f"{slope_threshold:.0f} deg or no-go soil) with no "
                f"reachable drivable cell within {GOAL_SNAP_RADIUS_CELLS} cells. Pick another point.")
    used_goal = list(goal)
    if snapped != grid_goal:
        log_info(f"Goal {grid_goal} is on impassable ground; moved to the "
                 f"nearest drivable cell {snapped}.")
        grid_goal = snapped
        used_goal = [float(snapped[0]), float(snapped[1])]

    log_info(f"Computing global trajectory from {start_pos} to {grid_goal}...")
    Tmap = planner.computeTmap(cost_map, obstacle_map, grid_goal, start_pos)
    if not np.isfinite(Tmap[start_pos[1], start_pos[0]]):
        # The search from the goal never reached the rover: steep ground
        # or no-go soil separates them.
        return ('aborted', None, None, 'goal not reachable',
                f"Goal {grid_goal} is not reachable from the rover "
                "(cut off by steep ground or no-go soil).")
    path = planner.getPathGDM(Tmap, np.asarray(start_pos), np.asarray(grid_goal), 0.4)

    # --- Path Validation Safety Check ---
    if path is None or np.isnan(path).any() or len(path) < 2:
        return ('aborted', None, None, 'no path to the goal',
                f"Pathfinding failed! Coordinates {grid_goal} are unreachable or inside an obstacle.")
    return ('driving', path, used_goal, '',
            f"Global trajectory resolved with {len(path)} waypoints. Resuming kinematic control.")


# =========================================================================
# Final approach
# =========================================================================

class ProgressWatch:
    """
    Notices a rover that should be driving but makes no progress: it has
    neither moved min_move_m nor turned min_turn_deg (a spot turn is progress)
    for window_s. `stalls` counts stalls in a row; only moving resets it, so a
    rover that turns back and forth on the spot still adds up stalls.
    """

    def __init__(self, window_s: float = 15.0, min_move_m: float = 0.25, min_turn_deg: float = 20.0):
        self.window_s, self.min_move_m, self.min_turn_deg = window_s, min_move_m, min_turn_deg
        self.stalls = 0
        self._anchor = None                 # (t, x, y, yaw) of the last progress

    def reset(self) -> None:
        """A new goal: forget everything."""
        self.stalls, self._anchor = 0, None

    def pause(self) -> None:
        """Not driving (planning): the window starts again when driving resumes."""
        self._anchor = None

    def update(self, t: float, x: float, y: float, yaw: float) -> bool:
        """True once per window without progress."""
        if self._anchor is None:
            self._anchor = (t, x, y, yaw)
            return False
        t0, x0, y0, yaw0 = self._anchor
        moved = math.hypot(x - x0, y - y0) >= self.min_move_m
        turned = abs(math.degrees(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))) >= self.min_turn_deg
        if moved or turned:
            self._anchor = (t, x, y, yaw)
            if moved:
                self.stalls = 0
            return False
        if t - t0 >= self.window_s:
            self._anchor = (t, x, y, yaw)
            self.stalls += 1
            return True
        return False


def stop_tolerance(robot_radius: float, base_tolerance: float) -> float:
    """Distance to the goal at which the rover parks."""
    return max(robot_radius, base_tolerance)


def approach_speed(base_max_speed: float, dist_to_goal: float, braking_distance: float) -> float:
    """
    The DWA's speed limit: full speed, then linearly lower within the braking
    distance (at least 0.2 m/s) so the rover does not overshoot the goal.
    """
    if dist_to_goal < braking_distance:
        return max(base_max_speed * (dist_to_goal / braking_distance), 0.2)
    return base_max_speed


APPROACH_ALIGNED_RAD = math.radians(20.0)


def approach_yaw_rate(yaw_rate: float, dist_to_goal: float, braking_distance: float,
                      tolerance: float, heading_error: float = 0.0) -> float:
    """
    The commanded yaw rate near the goal: damped within the braking distance
    and zero on the final metre, so the DWA does not spin over the goal (atan2
    turns unstable there). Only while the rover heads at the goal (within
    APPROACH_ALIGNED_RAD): arriving off course, it could not turn to the goal
    and circled it at crawling speed, never within the tolerance.
    """
    if abs(heading_error) > APPROACH_ALIGNED_RAD:
        return yaw_rate
    if dist_to_goal < braking_distance:
        yaw_rate *= dist_to_goal / braking_distance
        if dist_to_goal < tolerance + 1.0:
            yaw_rate = 0.0
    return yaw_rate


# =========================================================================
# Configuration
# =========================================================================

def robot_config(param: Callable[[str], object]) -> SimpleNamespace:
    """The DWA configuration from the node's parameters (`param(name)` -> value)."""
    conf = SimpleNamespace()
    conf.max_speed = param('max_speed')
    conf.min_speed = param('min_speed')
    conf.max_accel = param('max_accel')
    conf.max_yaw_rate = math.radians(param('max_yaw_rate_deg'))
    conf.max_delta_yaw_rate = math.radians(param('max_delta_yaw_rate_deg'))
    conf.yaw_rate_resolution = math.radians(param('yaw_rate_resolution_deg'))
    conf.v_resolution = param('v_resolution')
    conf.dt = param('dt')
    conf.predict_time = param('predict_time')
    conf.to_goal_cost_gain = param('to_goal_cost_gain')
    conf.speed_cost_gain = param('speed_cost_gain')
    conf.obstacle_cost_gain = param('obstacle_cost_gain')
    conf.robot_stuck_flag_cons = param('robot_stuck_flag_cons')
    conf.robot_type = 1
    conf.robot_width = param('robot_width')
    conf.robot_length = param('robot_length')
    conf.robot_radius = param('robot_radius')
    conf.obstacle_safety_m = param('obstacle_safety_m')
    return conf


def path_points(path) -> List[List[float]]:
    """A path (array or list of points) as [[x, y], ...] floats."""
    return [[float(p[0]), float(p[1])] for p in path]
