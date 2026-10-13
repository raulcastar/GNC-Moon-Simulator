"""The global planner (global_planner): goal snapping, routes, obstacle cells and the re-planning check."""

import math

import numpy as np

from src import global_planner as gp
from src.planner_maps import build_maps

CLEARANCE = gp.Clearance(rover_m=0.35, goal_m=1.35, margin_m=1.5)


def bordered_soil(shape):
    """Drivable soil (1) with the no-go border (0) the real soil maps have: the
    search does not check the map bounds and relies on it."""
    soil = np.ones(shape)
    soil[0, :] = soil[-1, :] = soil[:, 0] = soil[:, -1] = 0
    return soil


def planner(size=40, resolution=1.0, clearance=CLEARANCE):
    maps = build_maps(np.zeros((size, size)), bordered_soil((size, size)), resolution, 15.0)
    return gp.GlobalPlanner(maps, resolution, 15.0, clearance)


# --- the goal ---------------------------------------------------------------

def test_goal_snapping_on_a_small_map():
    obstacles = np.zeros((20, 20), int)
    obstacles[8:13, 8:13] = 1                        # a block
    obstacles[:, 16] = 1                             # a wall cutting off columns 17+
    regions = np.where(obstacles == 1, 0, 1)
    regions[:, 17:] = 2
    assert gp.drivable_goal(obstacles, regions, [2, 2], [0, 0]) == [2, 2]
    moved = gp.drivable_goal(obstacles, regions, [10, 10], [0, 0])
    assert moved is not None and obstacles[moved[1], moved[0]] == 0
    assert math.dist(moved, [10, 10]) <= gp.GOAL_SNAP_RADIUS_CELLS
    moved = gp.drivable_goal(obstacles, regions, [16, 5], [0, 0])
    assert moved == [15, 5]                          # not [17, 5]: the far side is unreachable
    assert gp.drivable_goal(obstacles, regions, [25, 5], [0, 0]) is None


# --- routes -----------------------------------------------------------------

def test_a_route_on_a_small_map():
    route = planner().plan((5.0, 5.0), (30.0, 20.0))
    assert route.ok and route.reason == '' and route.goal == [30.0, 20.0]
    assert len(route.path) >= 2 and math.dist(route.path[0], (5, 5)) < 1.0
    assert math.dist(route.path[-1], (30, 20)) < 1e-9


def test_an_off_map_goal_is_refused():
    route = planner(30).plan((5.0, 5.0), (50.0, 5.0))
    assert not route.ok and route.path is None and route.reason == 'goal on impassable ground'


def test_routes_are_in_metres_at_any_resolution():
    route = planner(40, resolution=0.5).plan((2.5, 2.5), (15.0, 10.0))
    assert route.ok and math.dist(route.path[0], (2.5, 2.5)) < 0.5
    assert math.dist(route.path[-1], (15.0, 10.0)) < 1e-9


def test_the_route_keeps_its_distance_from_a_rock_on_the_way():
    rock = {(20, 20), (20, 21), (21, 20)}
    route = planner().plan((5.0, 20.0), (35.0, 20.0), rock)
    assert route.ok
    gap = min(math.dist(p, c) for p in route.path for c in rock)
    assert gap >= 1.4, gap


def test_without_room_for_the_clearance_the_route_skirts_the_rock():
    # A 3-cell-wide corridor with a rock in it: no route keeps 1.5 cells away.
    soil = bordered_soil((40, 40))
    soil[:19, :] = 0
    soil[22:, :] = 0                                 # free rows 19-21 only
    maps = build_maps(np.zeros((40, 40)), soil, 1.0, 15.0)
    route = gp.GlobalPlanner(maps, 1.0, 15.0, CLEARANCE).plan((3.0, 20.0), (36.0, 20.0), {(20, 20)})
    assert route.ok
    assert all((int(round(x)), int(round(y))) != (20, 20) for x, y in route.path)


# --- obstacle cells ---------------------------------------------------------

def test_inflate_grows_by_the_radius():
    assert gp.inflate({(5, 5)}, 0.0) == {(5, 5)}
    assert gp.inflate({(5, 5)}, 1.0) == {(5, 5), (4, 5), (6, 5), (5, 4), (5, 6)}
    assert len(gp.inflate({(5, 5)}, 1.5)) == 9


def test_cells_to_avoid_free_the_rover_cell_and_around_the_goal():
    cells = {(10, 10), (11, 10), (14, 10), (20, 10), (22, 10)}
    avoid = gp.cells_to_avoid(cells, (10.3, 10.2), (21.0, 10.0), 0.35, 1.35)
    assert (10, 10) not in avoid                     # under the rover
    assert (11, 10) in avoid and (14, 10) in avoid
    assert (20, 10) not in avoid and (22, 10) not in avoid   # within 1.35 of the goal


def test_closed_cells_add_a_margin_but_not_near_the_rover_or_goal():
    rock = {(20, 20)}
    far = gp.closed_cells(rock, (5.0, 5.0), (35.0, 35.0), 0.35, 1.35, 1.5)
    assert far == gp.inflate(rock, 1.5)              # the rock and its 3 x 3 margin
    near = gp.closed_cells(rock, (18.0, 20.0), (35.0, 35.0), 0.35, 1.35, 1.5)
    assert (20, 20) in near and (19, 20) not in near  # the rock stays; the margin by the rover opens
    assert gp.cells_to_avoid(rock, (18.0, 20.0), (35.0, 35.0), 0.35, 1.35) <= near


def test_mark_impassable_copies_and_ignores_cells_off_the_map():
    maps = build_maps(np.zeros((5, 5)), np.ones((5, 5)), 1.0, 15.0)
    before = maps.obstacles.copy()
    cost, obstacles = gp.mark_impassable(maps, {(-1, 2), (2, 7), (1, 3)}, 30.0)
    assert obstacles[3, 1] == 1 and cost[3, 1] == 30.0 and obstacles.sum() == 1
    assert np.array_equal(maps.obstacles, before)    # the originals untouched


def test_route_crosses():
    path = np.column_stack((np.arange(0, 20, dtype=float), np.full(20, 4.0)))
    assert gp.route_crosses(path, {(5, 4)}, (0.0, 4.0), 6.0)
    assert not gp.route_crosses(path, {(5, 4)}, (0.0, 4.0), 4.0)    # beyond the look-ahead
    assert not gp.route_crosses(path, {(5, 4)}, (8.0, 4.0), 6.0)    # already passed
    assert not gp.route_crosses(path, set(), (0.0, 4.0), 6.0)


# --- re-planning ------------------------------------------------------------

def test_a_rock_close_ahead_gives_a_route_the_check_accepts():
    """A rock 0.7 m ahead of the rover (the re-plan loop seen in Unreal): the
    new route must not be 'blocked' by the same cells at once."""
    rover, goal = (10.0, 20.3), (30.0, 20.0)
    rock = {(11, 19), (11, 20), (11, 21), (12, 20)}
    for margin in (1.5, 0.0):
        p = planner(clearance=gp.Clearance(0.35, 1.35, margin))
        route = p.plan(rover, goal, rock)
        assert route.ok
        avoid = gp.cells_to_avoid(rock, rover, goal, 0.35, 1.35)
        assert not any((int(round(x)), int(round(y))) in avoid for x, y in route.path)


def test_blocks_route_sees_a_new_rock_ahead_only():
    p = planner()
    route = p.plan((5.0, 20.0), (35.0, 20.0))
    assert route.ok
    on_route = {(int(round(x)), int(round(y))) for x, y in route.path[10:12]}
    assert p.blocks_route(route.path, (5.0, 20.0), (35.0, 20.0), on_route, 20.0)
    assert not p.blocks_route(route.path, (5.0, 20.0), (35.0, 20.0), on_route, 2.0)   # too far ahead
    assert not p.blocks_route(route.path, (5.0, 20.0), (35.0, 20.0), {(20, 35)}, 40.0)  # off the route
    assert not p.blocks_route(route.path, (5.0, 20.0), (35.0, 20.0), set(), 40.0)


def test_a_route_never_steps_over_a_single_rock_cell():
    """Straight on the line from the rover to the goal, the gradient on both
    sides of a one-cell obstacle points straight across it."""
    route = planner(clearance=gp.Clearance(0.35, 1.35, 0.0)).plan((5.0, 20.0), (35.0, 20.0), {(20, 20)})
    assert route.ok
    assert all((int(round(x)), int(round(y))) != (20, 20) for x, y in route.path)
