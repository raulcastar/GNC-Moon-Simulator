"""
Fast marching search over a cost map, and route tracing from its result. No ROS.

From the DyMu (Dynamic Multilayered) path planner library by J. Ricardo
Sanchez Ibanez (ricardosan@uma.es) and Gonzalo Paz Delgado.

1. travel_time(): a front spreads from the goal over the cost map, solving
   the Eikonal equation on the 4-connected grid. T[cell] is the least total
   cost from the cell to the goal. Impassable cells are never entered, so T
   stays infinite there and wherever the front cannot reach.
2. trace_route(): from the rover, small steps down the gradient of T until
   the goal. When that continuous descent stalls (T stops dropping, e.g. in a
   one-cell-wide passage) or has no direction, trace_cells() walks down T
   cell by cell instead, which always reaches the goal.

Cells are (x, y) = (column, row); maps are indexed [row, column]. The search
does not check the map bounds: the maps must have an impassable border (the
soil maps do).
"""

import bisect
import logging
import math

import numpy as np

# The 4 neighbours, in the order the search visits them.
NEIGHBOURS = ((0, -1), (0, 1), (-1, 0), (1, 0))

# Route tracing: the step length (cells), the steps without a drop in T before
# falling back to the cell walk, and the distance at which the goal is reached.
TRACE_STEP_CELLS = 0.4
TRACE_STALL_STEPS = 40
TRACE_GOAL_CELLS = 1.5
TRACE_MAX_CELLS = 15000

# Loop iterations between checks of FastMarching.should_stop.
STOP_CHECK_EVERY = 256


class PlanningCancelled(Exception):
    """Raised inside a search when should_stop() turns true (the route is no longer wanted)."""


class FastMarching:
    """The search and the route tracing; `should_stop` lets another thread cancel them."""

    def __init__(self, log=None):
        self.log = log or logging.getLogger('fast_marching')
        # fn() -> bool, polled by the long loops: true raises PlanningCancelled.
        self.should_stop = None

    # -------------------------------------------------------------------------
    # Travel time
    # -------------------------------------------------------------------------

    def travel_time(self, cost: np.ndarray, obstacles: np.ndarray, goal, start=None) -> np.ndarray:
        """
        T map: the least total cost from each cell to `goal` (inf where it
        cannot be reached). The front stops early once it reaches `start`.
        """
        closed = obstacles == 1
        T = np.full(cost.shape, np.inf)
        band_t, band_cells = [], []          # the front, sorted by T

        goal = (int(goal[0]), int(goal[1]))
        start = None if start is None else (int(start[0]), int(start[1]))
        T[goal[1], goal[0]] = 0.0
        closed[goal[1], goal[0]] = True
        self._update_neighbours(goal, cost, T, band_t, band_cells, closed)

        steps = 0
        while band_t:
            steps += 1
            self._check_stop(steps)
            cell = band_cells.pop(0)
            del band_t[0]
            closed[cell[1], cell[0]] = True
            self._update_neighbours(cell, cost, T, band_t, band_cells, closed)
            if cell == start:
                break
        return T

    def _update_neighbours(self, cell, cost, T, band_t, band_cells, closed) -> None:
        """New or lower T for the open neighbours of `cell`, kept sorted in the front."""
        x, y = cell
        for dx, dy in NEIGHBOURS:
            nx, ny = x + dx, y + dy
            if closed[ny, nx]:
                continue
            t_horizontal = min(T[ny, nx + 1], T[ny, nx - 1])
            t_vertical = min(T[ny + 1, nx], T[ny - 1, nx])
            t = eikonal(t_horizontal, t_vertical, cost[ny, nx])
            old = T[ny, nx]
            if np.isinf(old):
                self._insert(band_t, band_cells, t, (nx, ny))
                T[ny, nx] = t
            elif t < old:
                self._remove(band_t, band_cells, old, (nx, ny))
                self._insert(band_t, band_cells, t, (nx, ny))
                T[ny, nx] = t

    @staticmethod
    def _insert(band_t, band_cells, t, cell) -> None:
        i = bisect.bisect_left(band_t, t)
        band_t.insert(i, t)
        band_cells.insert(i, cell)

    def _remove(self, band_t, band_cells, t, cell) -> None:
        i = bisect.bisect_left(band_t, t)
        for j in range(i, len(band_cells)):
            if band_cells[j] == cell:
                del band_t[j]
                del band_cells[j]
                return
        self.log.warning(f'Fast marching: cell {list(cell)} with T {t:.2f} missing from the front')

    # -------------------------------------------------------------------------
    # Route tracing
    # -------------------------------------------------------------------------

    def trace_route(self, T: np.ndarray, start, goal, step: float = TRACE_STEP_CELLS):
        """
        The route (N, 2 cells, start first, goal last) down the gradient of T,
        or the cell walk when that stalls; None if the start cannot reach the goal.
        """
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        rows, cols = T.shape
        path = [start]
        best_t, since_best = np.inf, 0

        for k in range(round(TRACE_MAX_CELLS / step)):
            self._check_stop(k + 1)
            here = path[-1]
            cx, cy = int(round(here[0])), int(round(here[1]))
            if 0 <= cy < rows and 0 <= cx < cols and T[cy, cx] < best_t:
                best_t, since_best = T[cy, cx], 0
            else:
                since_best += 1
                if since_best > TRACE_STALL_STEPS:
                    return self.trace_cells(T, start, goal)

            gx, gy = gradient_at(T, here)
            norm = math.hypot(gx, gy)
            if not norm > 0:                   # flat, or NaN: no direction to step in
                return self.trace_cells(T, start, goal)
            path.append(here - step * np.array([gx, gy]) / norm)
            if math.hypot(*(path[-1] - goal)) < TRACE_GOAL_CELLS:
                break
        else:
            return self.trace_cells(T, start, goal)   # out of steps

        path.append(goal)
        path = np.array(path)
        if enters_unreachable(path, T):
            # The descent stepped over an impassable cell (the gradient on its
            # two sides can point straight across it): walk the cells instead.
            return self.trace_cells(T, start, goal)
        return path

    def trace_cells(self, T: np.ndarray, start, goal):
        """
        The route by steepest descent of T over the 4 neighbours, cell by cell.
        T was computed 4-connected from the goal, so every reached cell has a
        lower neighbour and the walk ends at the goal without entering an
        impassable cell. None if the start was not reached by the front.
        """
        rows, cols = T.shape
        x, y = int(round(start[0])), int(round(start[1]))
        gx, gy = int(round(goal[0])), int(round(goal[1]))
        path = [[x, y]]
        for step in range(rows * cols):
            self._check_stop(step + 1)
            if (x, y) == (gx, gy):
                return np.array(path, dtype=float)
            best = None
            for dx, dy in NEIGHBOURS:
                nx, ny = x + dx, y + dy
                if 0 <= nx < cols and 0 <= ny < rows and T[ny, nx] < T[y, x] \
                        and (best is None or T[ny, nx] < T[best[1], best[0]]):
                    best = (nx, ny)
            if best is None:
                return None
            x, y = best
            path.append([x, y])
        return None

    def _check_stop(self, step: int) -> None:
        """Every STOP_CHECK_EVERY steps: PlanningCancelled if should_stop() says so."""
        if step % STOP_CHECK_EVERY == 0 and self.should_stop is not None and self.should_stop():
            raise PlanningCancelled()


def enters_unreachable(path: np.ndarray, T: np.ndarray) -> bool:
    """Whether a route point after the start lies in a cell the front never reached (T inf)."""
    cells = np.rint(path[1:]).astype(int)
    rows, cols = T.shape
    inside = (cells[:, 0] >= 0) & (cells[:, 0] < cols) & (cells[:, 1] >= 0) & (cells[:, 1] < rows)
    if not inside.all():
        return True
    return bool(np.isinf(T[cells[:, 1], cells[:, 0]]).any())


# =============================================================================
# The Eikonal update and the gradient of T
# =============================================================================

def eikonal(t_horizontal: float, t_vertical: float, cost: float) -> float:
    """T of a cell from its lowest horizontal and vertical neighbours' T and its own cost."""
    if np.isinf(t_horizontal):
        return np.inf if np.isinf(t_vertical) else t_vertical + cost
    if np.isinf(t_vertical):
        return t_horizontal + cost
    if cost < abs(t_horizontal - t_vertical):
        return min(t_horizontal, t_vertical) + cost
    return 0.5 * (t_horizontal + t_vertical
                  + math.sqrt(2 * cost ** 2 - (t_horizontal - t_vertical) ** 2))


def cell_gradient(T: np.ndarray, row: int, col: int):
    """
    The unit gradient (gx, gy) of T at a cell; one-sided next to infinite
    cells (T jumps there) and at the map edge, (0, 0) when undefined.
    """
    rows, cols = T.shape
    j, i = row, col
    if j == 0:
        gy = T[1, i] - T[0, i]
    elif j == rows - 1:
        gy = T[j, i] - T[j - 1, i]
    elif np.isinf(T[j + 1, i]):
        gy = 0 if np.isinf(T[j - 1, i]) else T[j, i] - T[j - 1, i]
    elif np.isinf(T[j - 1, i]):
        gy = T[j + 1, i] - T[j, i]
    else:
        gy = (T[j + 1, i] - T[j - 1, i]) / 2
    if i == 0:
        gx = T[j, 1] - T[j, 0]
    elif i == cols - 1:
        gx = T[j, i] - T[j, i - 1]
    elif np.isinf(T[j, i + 1]):
        gx = 0 if np.isinf(T[j, i - 1]) else T[j, i] - T[j, i - 1]
    elif np.isinf(T[j, i - 1]):
        gx = T[j, i + 1] - T[j, i]
    else:
        gx = (T[j, i + 1] - T[j, i - 1]) / 2
    gx, gy = np.float64(gx), np.float64(gy)
    magnitude = math.sqrt(gx ** 2 + gy ** 2)
    if magnitude == 0 or math.isinf(magnitude) or math.isnan(magnitude):
        return np.float64(0.0), np.float64(0.0)
    return gx / magnitude, gy / magnitude


def gradient_at(T: np.ndarray, point):
    """The unit gradient of T at a point (x, y), bilinear between the cell gradients around it."""
    cache = {}

    def component(k):
        def value(row, col):
            key = (int(row), int(col))
            if key not in cache:
                cache[key] = cell_gradient(T, key[0], key[1])
            return cache[key][k]
        return value

    return interpolate(point, T.shape, component(0)), interpolate(point, T.shape, component(1))


def interpolate(point, shape, value_at):
    """Bilinear value at `point` (x, y) of a map given by value_at(row, col)."""
    i = np.uint32(np.fix(point[0]))
    j = np.uint32(np.fix(point[1]))
    a = point[0] - i
    b = point[1] - j
    m, n = np.uint32(shape)

    if i == n:
        if j == m:
            return value_at(j, i)
        return b * value_at(j + 1, i) + (1 - b) * value_at(j, i)
    if j == m:
        return a * value_at(j, i + 1) + (1 - a) * value_at(j, i)

    a00 = value_at(j, i)
    a10 = value_at(j, i + 1) - value_at(j, i)
    a01 = value_at(j + 1, i) - value_at(j, i)
    a11 = value_at(j + 1, i + 1) + value_at(j, i) - value_at(j, i + 1) - value_at(j + 1, i)
    if a == 0:
        return a00 if b == 0 else a00 + a01 * b
    if b == 0:
        return a00 + a10 * a
    return a00 + a10 * a + a01 * b + a11 * a * b
