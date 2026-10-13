"""
Global planner: the route from the rover to the goal over the map, around
the obstacles the lidar has found. No ROS.

    plan(rover, goal, obstacle_cells)          -> Route
    blocks_route(route, rover, goal, cells)    -> bool, for re-planning

A plan:
 1. The goal is moved to the nearest drivable cell the rover can reach when
    it lies on impassable ground (too steep, no-go soil), or refused.
 2. Lidar obstacle cells are made impassable (cells_to_avoid), with a margin
    of `clearance.margin_m` around them so the route keeps its distance from
    rocks (the local planner only refuses to hit them).
 3. The fast marching search runs from the goal to the rover, and the route
    is traced back down its travel-time map.
 4. If no route keeps the margin, the search is repeated without it.

Obstacle cells near the rover and the goal are ignored (cells_to_avoid): the
rover sits right beside what its lidar sees, and a goal beside a rock must
stay reachable. The re-planning check uses the same rule.

Units: positions and routes in metres (odom frame); the maps and obstacle
cells in cells of `resolution` metres, (x, y) = (column, row).
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from src.fast_marching import FastMarching
from src.planner_maps import PlannerMaps

# A goal on impassable ground is moved to the nearest drivable cell within
# this many cells, if the rover can reach it.
GOAL_SNAP_RADIUS_CELLS = 5


@dataclass
class Route:
    """The outcome of a plan."""
    ok: bool
    path: Optional[np.ndarray] = None   # (N, 2) metres, rover first, goal last
    goal: Optional[list] = None         # the goal used (moved off impassable ground if need be)
    reason: str = ''                    # why not, for the mission state
    message: str = ''                   # for the log


@dataclass
class Clearance:
    """Distances (m) for the lidar's obstacle cells."""
    rover_m: float      # cells this close to the rover are ignored
    goal_m: float       # cells this close to the goal are ignored
    margin_m: float     # routes keep this far from obstacle cells, when they can


class GlobalPlanner:
    def __init__(self, maps: PlannerMaps, resolution: float, slope_threshold: float,
                 clearance: Clearance, log=None):
        self.maps = maps
        self.resolution = resolution
        self.slope_threshold = slope_threshold
        self.clearance = clearance
        self.search = FastMarching(log)
        self.log = self.search.log

    def plan(self, rover_xy, goal_xy, obstacle_cells: set = frozenset()) -> Route:
        """The route from the rover to the goal (metres), around `obstacle_cells`."""
        res = self.resolution
        rover = (rover_xy[0] / res, rover_xy[1] / res)
        goal = (goal_xy[0] / res, goal_xy[1] / res)
        if not obstacle_cells:
            return self._search(self.maps.cost, self.maps.obstacles, rover, goal_xy)

        c = self.clearance
        blocked = closed_cells(obstacle_cells, rover, goal, c.rover_m / res, c.goal_m / res, c.margin_m / res)
        route = self._search(*mark_impassable(self.maps, blocked, self._blocked_cost), rover, goal_xy)
        if route.ok or c.margin_m <= 0:
            return route
        self.log.info("No route keeps the obstacle clearance; planning without it.")
        blocked = closed_cells(obstacle_cells, rover, goal, c.rover_m / res, c.goal_m / res, 0.0)
        return self._search(*mark_impassable(self.maps, blocked, self._blocked_cost), rover, goal_xy)

    def blocks_route(self, path: np.ndarray, rover_xy, goal_xy, obstacle_cells: set,
                     lookahead_m: float) -> bool:
        """Whether `obstacle_cells` cross the route within lookahead_m ahead of the rover."""
        if not obstacle_cells or path is None:
            return False
        res = self.resolution
        rover = (rover_xy[0] / res, rover_xy[1] / res)
        goal = None if goal_xy is None else (goal_xy[0] / res, goal_xy[1] / res)
        c = self.clearance
        cells = cells_to_avoid(obstacle_cells, rover, goal, c.rover_m / res, c.goal_m / res)
        return route_crosses(np.asarray(path) / res, cells, rover, lookahead_m / res)

    @property
    def _blocked_cost(self) -> float:
        return 2 * self.slope_threshold          # what impassable cells cost in the maps

    def _search(self, cost: np.ndarray, obstacles: np.ndarray, rover, goal_xy) -> Route:
        """Snap the goal, search, trace (rover and goal in cells; the route in metres)."""
        res = self.resolution
        start = [int(round(rover[0])), int(round(rover[1]))]
        goal = [int(round(goal_xy[0] / res)), int(round(goal_xy[1] / res))]

        snapped = drivable_goal(obstacles, self.maps.free_regions, goal, start)
        if snapped is None:
            return Route(False, reason='goal on impassable ground', message=(
                f"Goal {goal} is on impassable ground (slope over {self.slope_threshold:.0f} deg "
                f"or no-go soil) with no reachable drivable cell within {GOAL_SNAP_RADIUS_CELLS} "
                f"cells. Pick another point."))
        used_goal = list(goal_xy)
        if snapped != goal:
            self.log.info(f"Goal {goal} is on impassable ground; moved to the nearest drivable cell {snapped}.")
            goal = snapped
            used_goal = [goal[0] * res, goal[1] * res]

        self.log.info(f"Computing global trajectory from {start} to {goal}...")
        T = self.search.travel_time(cost, obstacles, goal, start)
        if not np.isfinite(T[start[1], start[0]]):
            return Route(False, reason='goal not reachable', message=(
                f"Goal {goal} is not reachable from the rover (cut off by steep ground or no-go soil)."))
        path = self.search.trace_route(T, start, goal)
        if path is None or np.isnan(path).any() or len(path) < 2:
            return Route(False, reason='no path to the goal', message=(
                f"Pathfinding failed: no route to {goal} could be traced."))
        return Route(True, path * res, used_goal, message=(
            f"Global trajectory resolved with {len(path)} waypoints. Resuming kinematic control."))


# =============================================================================
# The goal
# =============================================================================

def drivable_goal(obstacles: np.ndarray, free_regions: Optional[np.ndarray], goal: list,
                  start: list, radius: int = GOAL_SNAP_RADIUS_CELLS) -> Optional[list]:
    """
    `goal` (cells) if it is drivable, else the nearest drivable cell within
    `radius` in the rover's drivable region (so a route can exist); None if none.
    """
    rows, cols = obstacles.shape
    gx, gy = goal
    if not (0 <= gx < cols and 0 <= gy < rows):
        return None
    if obstacles[gy, gx] != 1:
        return goal
    r = radius
    y0, y1, x0, x1 = max(0, gy - r), min(rows, gy + r + 1), max(0, gx - r), min(cols, gx + r + 1)
    free = obstacles[y0:y1, x0:x1] != 1
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


# =============================================================================
# Obstacle cells (col, row)
# =============================================================================

def cells_to_avoid(cells: set, rover, goal, rover_clear: float, goal_clear: float) -> set:
    """
    The obstacle cells a route may not cross: all of `cells` except the one
    under the rover and those within rover_clear of it, and those within
    goal_clear of the goal (rover and goal in cells, may be fractional).
    """
    under = (int(round(rover[0])), int(round(rover[1])))
    return {c for c in cells
            if c != under
            and math.hypot(c[0] - rover[0], c[1] - rover[1]) > rover_clear
            and (goal is None or math.hypot(c[0] - goal[0], c[1] - goal[1]) > goal_clear)}


def closed_cells(cells: set, rover, goal, rover_clear: float, goal_clear: float,
                 margin: float) -> set:
    """
    The cells closed to the search: cells_to_avoid, plus a margin around every
    obstacle cell. The margin is left open within margin + 1 of the rover and
    the goal, so the rover can leave or reach a spot close to a rock.
    """
    avoid = cells_to_avoid(cells, rover, goal, rover_clear, goal_clear)
    if margin <= 0:
        return avoid
    open_radius = margin + 1.0
    ring = {c for c in inflate(cells, margin) - cells
            if math.hypot(c[0] - rover[0], c[1] - rover[1]) > open_radius
            and math.hypot(c[0] - goal[0], c[1] - goal[1]) > open_radius}
    return avoid | ring


def inflate(cells: set, radius: float) -> set:
    """`cells` grown by every cell whose centre is within `radius` (cells) of one of them."""
    r = int(math.floor(radius))
    offsets = [(dc, dr) for dc in range(-r, r + 1) for dr in range(-r, r + 1)
               if math.hypot(dc, dr) <= radius]
    return {(c + dc, w + dr) for c, w in cells for dc, dr in offsets}


def mark_impassable(maps: PlannerMaps, cells: set, blocked_cost: float):
    """Copies of the cost and obstacle maps with `cells` impassable (cells off the map ignored)."""
    cost, obstacles = maps.cost.copy(), maps.obstacles.copy()
    rows, cols = obstacles.shape
    for col, row in cells:
        if 0 <= col < cols and 0 <= row < rows:
            obstacles[row, col] = 1
            cost[row, col] = blocked_cost
    return cost, obstacles


def route_crosses(path: np.ndarray, cells: set, rover, lookahead: float) -> bool:
    """
    Whether the route (N, 2 cells, start first) passes through one of `cells`
    within `lookahead` (cells, along the route) of its point nearest the rover.
    """
    if not cells or path is None or len(path) == 0:
        return False
    path = np.asarray(path, dtype=float)
    nearest = int(np.argmin(np.hypot(path[:, 0] - rover[0], path[:, 1] - rover[1])))
    ahead = path[nearest:]
    along = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(ahead, axis=0).T))))
    return any((int(round(x)), int(round(y))) in cells for x, y in ahead[along <= lookahead])
