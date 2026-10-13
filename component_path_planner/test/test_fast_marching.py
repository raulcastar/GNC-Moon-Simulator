"""
The search (fast_marching) on the shipped procedural map: the goal that used
to hang the route tracing, seeded random goals, unreachable goals, the cell
walk, the gradient, and the cooperative stop used by the node's planning thread.
"""
import math
import os
import time

import numpy as np
import pytest

from conftest import PACKAGE
from src import fast_marching as fm
from src.planner_maps import load_planner_maps

MAPS = os.path.join(PACKAGE, "maps")
MAP_FILES = ("procedural_lunar_terrain_1_elevationMap_2.csv", "procedural_lunar_terrain_1_soilMap_edges_0_2.csv")
# The maps are git-ignored (large CSVs): a checkout without them skips these tests.
pytestmark = pytest.mark.skipif(not all(os.path.isfile(os.path.join(MAPS, f)) for f in MAP_FILES),
                                reason="planner maps not in component_path_planner/maps")
START = (615, 549)                      # the rover's usual start cell on this map
HANGING_GOAL = (645, 547)               # oscillated for minutes before the stall guard


@pytest.fixture(scope="module")
def search():
    return fm.FastMarching()


@pytest.fixture(scope="module")
def maps():
    return load_planner_maps(*(os.path.join(MAPS, f) for f in MAP_FILES), 1.0, 15.0)


def plan(search, maps, goal, start=START):
    T = search.travel_time(maps.cost, maps.obstacles, goal, start)
    assert np.isfinite(T[start[1], start[0]]), f"goal {goal} not reachable"
    t = time.perf_counter()
    path = search.trace_route(T, start, goal)
    return path, time.perf_counter() - t


def check_path(path, maps, goal, start=START):
    assert path is not None and len(path) >= 2
    assert np.isfinite(path).all()
    assert math.dist(path[0], start) < 1.0, "path does not start at the rover"
    assert math.dist(path[-1], goal) <= 1.5, "path does not end at the goal"
    cells = np.round(path).astype(int)
    assert not (maps.obstacles[cells[:, 1], cells[:, 0]] == 1).any(), "a waypoint is on an obstacle cell"
    length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    assert length <= 3.0 * math.dist(goal, start) + 5, "path wanders"


def test_hanging_goal_finishes(search, maps):
    path, seconds = plan(search, maps, HANGING_GOAL)
    check_path(path, maps, HANGING_GOAL)
    assert seconds < 5.0, f"route tracing took {seconds:.1f} s"


def reachable_goals(maps, count=8, seed=4, reach=80):
    rng = np.random.default_rng(seed)
    goals = []
    while len(goals) < count:
        g = (int(START[0] + rng.integers(-reach, reach + 1)), int(START[1] + rng.integers(-reach, reach + 1)))
        if maps.obstacles[g[1], g[0]] != 1 and maps.free_regions[g[1], g[0]] == maps.free_regions[START[1], START[0]]:
            goals.append(g)
    return goals


def test_random_goals(search, maps):
    for goal in reachable_goals(maps):
        path, seconds = plan(search, maps, goal)
        check_path(path, maps, goal)
        assert seconds < 5.0, f"route tracing to {goal} took {seconds:.1f} s"


def test_goal_in_another_region_is_unreachable(search, maps):
    regions = maps.free_regions
    sizes = np.bincount(regions.ravel())
    other = np.argwhere((regions != regions[START[1], START[0]]) & (regions > 0) & (sizes[regions] > 50))
    gy, gx = other[len(other) // 2]
    T = search.travel_time(maps.cost, maps.obstacles, (int(gx), int(gy)), START)
    assert not np.isfinite(T[START[1], START[0]])


def test_cell_walk_reaches_the_goal(search, maps):
    T = search.travel_time(maps.cost, maps.obstacles, HANGING_GOAL, START)
    path = search.trace_cells(T, START, HANGING_GOAL)
    assert path is not None and tuple(path[-1]) == HANGING_GOAL
    assert np.all(np.abs(np.diff(path, axis=0)).sum(axis=1) == 1), "steps are not 4-connected"


def full_gradient(cost, point):
    """The original full-map gradient (DyMu's computeGradient), the reference for gradient_at."""
    m, n = cost.shape
    jmax, imax = min(m, int(point[1]) + 3), min(n, int(point[0]) + 3)
    jmin, imin = max(0, int(point[1] - 3)), max(0, int(point[0] - 3))
    Gx, Gy = np.zeros_like(cost), np.zeros_like(cost)
    Gnx, Gny = np.zeros_like(cost), np.zeros_like(cost)
    for i in range(imin, imax):
        for j in range(jmin, jmax):
            if j == 0:
                Gy[0, i] = cost[1, i] - cost[0, i]
            elif j == m - 1:
                Gy[j, i] = cost[j, i] - cost[j - 1, i]
            elif np.isinf(cost[j + 1, i]):
                Gy[j, i] = 0 if np.isinf(cost[j - 1, i]) else cost[j, i] - cost[j - 1, i]
            elif np.isinf(cost[j - 1, i]):
                Gy[j, i] = cost[j + 1, i] - cost[j, i]
            else:
                Gy[j, i] = (cost[j + 1, i] - cost[j - 1, i]) / 2
            if i == 0:
                Gx[j, 0] = cost[j, 1] - cost[j, 0]
            elif i == n - 1:
                Gx[j, i] = cost[j, i] - cost[j, i - 1]
            elif np.isinf(cost[j, i + 1]):
                Gx[j, i] = 0 if np.isinf(cost[j, i - 1]) else cost[j, i] - cost[j, i - 1]
            elif np.isinf(cost[j, i - 1]):
                Gx[j, i] = cost[j, i + 1] - cost[j, i]
            else:
                Gx[j, i] = (cost[j, i + 1] - cost[j, i - 1]) / 2
            magnitude = math.sqrt(Gx[j, i] ** 2 + Gy[j, i] ** 2)
            if magnitude == 0 or math.isinf(magnitude) or math.isnan(magnitude):
                Gnx[j, i] = Gny[j, i] = 0
            else:
                Gnx[j, i], Gny[j, i] = Gx[j, i] / magnitude, Gy[j, i] / magnitude
    return Gnx, Gny


def test_gradient_at_matches_the_full_gradient(search, maps):
    """gradient_at (4 cells) equals interpolating the full gradient maps, bit for bit."""
    T = search.travel_time(maps.cost, maps.obstacles, HANGING_GOAL, START)
    rng = np.random.default_rng(1)
    for _ in range(40):
        p = np.array([START[0] + rng.uniform(-20, 20), START[1] + rng.uniform(-20, 20)])
        g1, g2 = full_gradient(T, p)
        expected = (fm.interpolate(p, T.shape, lambda j, i: g1[j, i]),
                    fm.interpolate(p, T.shape, lambda j, i: g2[j, i]))
        assert np.array_equal(np.array(fm.gradient_at(T, p)), np.array(expected), equal_nan=True), p


def test_should_stop_cancels_a_search(search, maps):
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] > 2

    search.should_stop = stop
    try:
        with pytest.raises(fm.PlanningCancelled):
            search.travel_time(maps.cost, maps.obstacles, (200, 200), START)
    finally:
        search.should_stop = None


def test_eikonal():
    assert fm.eikonal(np.inf, np.inf, 1.0) == np.inf
    assert fm.eikonal(2.0, np.inf, 1.0) == 3.0                   # from one side
    assert fm.eikonal(2.0, 5.0, 1.0) == 3.0                      # the other side too far behind
    assert fm.eikonal(2.0, 2.0, 1.0) == pytest.approx(2.0 + math.sqrt(2) / 2)
