"""
Global planner on the shipped procedural map: the goal that used to hang the
gradient descent, seeded random goals, unreachable goals, goal snapping, and
the cooperative stop used by the node's planning thread.
"""
import os
import time

import cv2
import numpy as np
import pytest

from conftest import PACKAGE
from src import global_path_planner as gpp

MAPS = os.path.join(PACKAGE, "maps")
MAP_FILES = ("procedural_lunar_terrain_1_elevationMap_2.csv", "procedural_lunar_terrain_1_soilMap_edges_0_2.csv")
# The maps are git-ignored (large CSVs): a checkout without them skips these tests.
pytestmark = pytest.mark.skipif(not all(os.path.isfile(os.path.join(MAPS, f)) for f in MAP_FILES),
                                reason="planner maps not in component_path_planner/maps")
START = [615, 549]                      # the rover's usual start cell on this map
HANGING_GOAL = [645, 547]               # oscillated for minutes before the stall guard


@pytest.fixture(scope="module")
def planner():
    return gpp.Global_Path_Planner()


@pytest.fixture(scope="module")
def maps(planner):
    elev = np.loadtxt(os.path.join(MAPS, "procedural_lunar_terrain_1_elevationMap_2.csv"))
    soil = np.loadtxt(os.path.join(MAPS, "procedural_lunar_terrain_1_soilMap_edges_0_2.csv"))
    cost, obstacles, _ = planner.getCostMap(elev, soil, 1.0, 15.0)
    _, regions = cv2.connectedComponents((obstacles != 1).astype(np.uint8), connectivity=4)
    return cost, obstacles, regions


def plan(planner, maps, goal, start=START):
    cost, obstacles, _ = maps
    T = planner.computeTmap(cost, obstacles, goal, start)
    assert np.isfinite(T[start[1], start[0]]), f"goal {goal} not reachable"
    t = time.perf_counter()
    path = planner.getPathGDM(T, np.asarray(start), np.asarray(goal), 0.4)
    return path, time.perf_counter() - t


def check_path(path, maps, goal, start=START):
    _, obstacles, _ = maps
    assert path is not None and len(path) >= 2
    assert np.isfinite(path).all()
    assert np.hypot(*(path[0] - start)) < 1.0, "path does not start at the rover"
    assert np.hypot(*(path[-1] - goal)) <= 1.5, "path does not end at the goal"
    cells = np.round(path).astype(int)
    assert not (obstacles[cells[:, 1], cells[:, 0]] == 1).any(), "a waypoint is on an obstacle cell"
    length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    assert length <= 3.0 * np.hypot(goal[0] - start[0], goal[1] - start[1]) + 5, "path wanders"


def test_hanging_goal_finishes(planner, maps):
    path, seconds = plan(planner, maps, HANGING_GOAL)
    check_path(path, maps, HANGING_GOAL)
    assert seconds < 5.0, f"path tracing took {seconds:.1f} s"


def reachable_goals(maps, count=8, seed=4, reach=80):
    _, obstacles, regions = maps
    rng = np.random.default_rng(seed)
    goals = []
    while len(goals) < count:
        g = [int(START[0] + rng.integers(-reach, reach + 1)), int(START[1] + rng.integers(-reach, reach + 1))]
        if obstacles[g[1], g[0]] != 1 and regions[g[1], g[0]] == regions[START[1], START[0]]:
            goals.append(g)
    return goals


def test_random_goals(planner, maps):
    for goal in reachable_goals(maps):
        path, seconds = plan(planner, maps, goal)
        check_path(path, maps, goal)
        assert seconds < 5.0, f"path tracing to {goal} took {seconds:.1f} s"


def test_goal_in_another_region_is_unreachable(planner, maps):
    cost, obstacles, regions = maps
    sizes = np.bincount(regions.ravel())
    other = np.argwhere((regions != regions[START[1], START[0]]) & (regions > 0) & (sizes[regions] > 50))
    gy, gx = other[len(other) // 2]
    T = planner.computeTmap(cost, obstacles, [int(gx), int(gy)], START)
    assert not np.isfinite(T[START[1], START[0]])


def test_discrete_descent_reaches_the_goal(planner, maps):
    cost, obstacles, _ = maps
    T = planner.computeTmap(cost, obstacles, HANGING_GOAL, START)
    path = planner.getPathDiscrete(T, np.asarray(START), np.asarray(HANGING_GOAL))
    assert path is not None and tuple(path[-1]) == tuple(HANGING_GOAL)
    assert np.all(np.abs(np.diff(path, axis=0)).sum(axis=1) == 1), "steps are not 4-connected"


def test_gradient_at_matches_the_full_gradient(planner, maps):
    """gradientAt (4 cells) equals interpolating computeGradient (full map), bit for bit."""
    cost, obstacles, _ = maps
    T = planner.computeTmap(cost, obstacles, HANGING_GOAL, START)
    rng = np.random.default_rng(1)
    for _ in range(40):
        p = np.array([START[0] + rng.uniform(-20, 20), START[1] + rng.uniform(-20, 20)])
        g1, g2 = planner.computeGradient(T, p)
        expected = (planner.interpolatePoint(p, g1), planner.interpolatePoint(p, g2))
        got = planner.gradientAt(T, p)
        assert np.array_equal(np.array(got), np.array(expected), equal_nan=True), p


def test_should_stop_cancels_a_search(planner, maps):
    cost, obstacles, _ = maps
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] > 2

    planner.should_stop = stop
    try:
        with pytest.raises(gpp.PlanningCancelled):
            planner.computeTmap(cost, obstacles, [200, 200], START)
    finally:
        planner.should_stop = None


def test_goal_snapping(maps):
    """drivable_goal: drivable goals stay, goals on obstacles move to a reachable cell nearby."""
    from src.mission import GOAL_SNAP_RADIUS_CELLS, drivable_goal
    _, obstacles, regions = maps
    snap = lambda goal: drivable_goal(obstacles, regions, goal, START)
    assert snap([640, 560]) == [640, 560]
    blocked = np.argwhere(obstacles == 1)
    near = blocked[np.argmin(np.hypot(blocked[:, 1] - START[0], blocked[:, 0] - START[1]))]
    goal = [int(near[1]), int(near[0])]
    moved = snap(goal)
    assert moved is not None and obstacles[moved[1], moved[0]] != 1
    assert np.hypot(moved[0] - goal[0], moved[1] - goal[1]) <= GOAL_SNAP_RADIUS_CELLS
    assert regions[moved[1], moved[0]] == regions[START[1], START[0]]
    assert snap([-5, 10]) is None                       # off the map
