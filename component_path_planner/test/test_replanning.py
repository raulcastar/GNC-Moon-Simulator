"""Re-planning around lidar obstacles: shared blocking rules, clearance, and the no-progress watch."""

import math

import numpy as np

from src import global_path_planner as gpp
from src import lidar_obstacles as lo
from src import mission
from src.planner_maps import build_maps

ROVER_CLEAR, GOAL_CLEAR = 0.35, 1.35          # cells, at 1 m per cell (robot_radius 0.35 m)


def bordered_soil(shape):
    soil = np.ones(shape)
    soil[0, :] = soil[-1, :] = soil[:, 0] = soil[:, -1] = 0
    return soil


def test_inflate_grows_by_the_radius():
    assert lo.inflate({(5, 5)}, 0.0) == {(5, 5)}
    assert lo.inflate({(5, 5)}, 1.0) == {(5, 5), (4, 5), (6, 5), (5, 4), (5, 6)}
    assert len(lo.inflate({(5, 5)}, 1.5)) == 9


def test_route_blocking_frees_the_rover_cell_and_around_the_goal():
    cells = {(10, 10), (11, 10), (14, 10), (20, 10), (22, 10)}
    blocked = lo.route_blocking(cells, (10.3, 10.2), (21.0, 10.0), ROVER_CLEAR, GOAL_CLEAR)
    assert (10, 10) not in blocked                   # under the rover
    assert (11, 10) in blocked and (14, 10) in blocked
    assert (20, 10) not in blocked and (22, 10) not in blocked   # within 1.35 of the goal


def test_planning_blocks_a_margin_but_not_near_the_rover_or_goal():
    rock = {(20, 20)}
    far = lo.planning_blocked(rock, (5.0, 5.0), (35.0, 35.0), ROVER_CLEAR, GOAL_CLEAR, 1.5)
    assert far == lo.inflate(rock, 1.5)              # the rock and its 3x3 margin
    near = lo.planning_blocked(rock, (18.0, 20.0), (35.0, 35.0), ROVER_CLEAR, GOAL_CLEAR, 1.5)
    assert (20, 20) in near and (19, 20) not in near  # the rock stays; the margin by the rover opens
    assert lo.route_blocking(rock, (18.0, 20.0), (35.0, 35.0), ROVER_CLEAR, GOAL_CLEAR) <= near


def plan_with(cells, rover, goal, clearance):
    planner = gpp.Global_Path_Planner()
    maps = build_maps(planner, np.zeros((40, 40)), bordered_soil((40, 40)), 1.0, 15.0)
    blocked = lo.planning_blocked(cells, rover, goal, ROVER_CLEAR, GOAL_CLEAR, clearance)
    cost, obstacles = lo.mark_blocked(maps.cost, maps.obstacles, blocked, 30.0)
    start = [int(round(rover[0])), int(round(rover[1]))]
    state, path, _goal, _reason, _msg = mission.plan_route(
        planner, cost, obstacles, maps.free_regions, start, list(goal), 15.0)
    assert state == 'driving'
    return path


def test_a_rock_close_ahead_gives_a_route_the_check_accepts():
    """The re-plan loop seen in Unreal: a rock 0.7 m ahead of the rover. Planning
    used to clear blocked cells within 1.35 cells of the rover but the check
    only within 0.35, so every new route was 'blocked' at once."""
    rover, goal = (10.0, 20.3), (30.0, 20.0)
    rock = {(11, 19), (11, 20), (11, 21), (12, 20)}
    for clearance in (1.5, 0.0):
        path = plan_with(rock, rover, goal, clearance)
        assert not lo.path_blocked(path, lo.route_blocking(rock, rover, goal, ROVER_CLEAR, GOAL_CLEAR),
                                   rover, 5.0, 1.0)


def test_the_route_keeps_its_distance_from_a_rock_on_the_way():
    rover, goal = (5.0, 20.0), (35.0, 20.0)
    rock = {(20, 20), (20, 21), (21, 20)}
    path = plan_with(rock, rover, goal, 1.5)
    gap = min(math.dist(p, c) for p in path for c in rock)
    assert gap >= 1.4, gap
    assert not lo.path_blocked(path, lo.route_blocking(rock, rover, goal, ROVER_CLEAR, GOAL_CLEAR),
                               rover, 40.0, 1.0)


def test_mark_blocked_ignores_cells_outside_the_map():
    cost, obstacles = lo.mark_blocked(np.zeros((5, 5)), np.zeros((5, 5)), {(-1, 2), (2, 7), (2, 2)}, 9.0)
    assert obstacles.sum() == 1 and obstacles[2, 2] == 1 and obstacles[-1, 2] == 0


def test_progress_watch_counts_stalls_and_resets_on_moving():
    w = mission.ProgressWatch(window_s=10.0, min_move_m=0.25, min_turn_deg=20.0)
    assert not w.update(0.0, 0.0, 0.0, 0.0)
    assert not w.update(9.0, 0.1, 0.0, 0.0)
    assert w.update(10.5, 0.1, 0.0, 0.0) and w.stalls == 1
    assert w.update(21.0, 0.1, 0.0, 0.0) and w.stalls == 2
    assert not w.update(25.0, 0.6, 0.0, 0.0) and w.stalls == 0      # it moved


def test_progress_watch_spot_turns_are_progress_but_do_not_clear_stalls():
    w = mission.ProgressWatch(window_s=10.0)
    w.update(0.0, 0.0, 0.0, 0.0)
    assert w.update(11.0, 0.0, 0.0, 0.0) and w.stalls == 1
    assert not w.update(20.0, 0.0, 0.0, math.radians(30))           # a spot turn: progress
    assert w.stalls == 1
    w.pause()                                                         # planning: the window restarts
    assert not w.update(100.0, 0.0, 0.0, math.radians(30))
    assert not w.update(105.0, 0.0, 0.0, math.radians(30))
    w.reset()
    assert w.stalls == 0


def test_dwa_moves_off_from_rest_beside_a_rock():
    """The stall seen in Unreal: at rest 0.85 m from a rock with the route
    leading past it, the old 1/distance cost made standing still cheapest."""
    import types
    from src.local_trajectory_controller import Local_Trajectory_Controller
    conf = types.SimpleNamespace(
        max_speed=0.5, min_speed=0.0, max_accel=0.1, max_yaw_rate=math.radians(8.0),
        max_delta_yaw_rate=math.radians(17.0), yaw_rate_resolution=math.radians(0.5),
        v_resolution=0.005, dt=0.1, predict_time=1.5, to_goal_cost_gain=1.0, speed_cost_gain=1.0,
        obstacle_cost_gain=1.0, robot_stuck_flag_cons=0.001, robot_type=1,
        robot_width=0.65, robot_length=0.65, robot_radius=0.35, obstacle_safety_m=0.15)
    rock = np.array([[0.6 + dx, 0.6 + dy] for dx in (0.0, 0.1, 0.2) for dy in (0.0, 0.1, 0.2)])
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    u, _ = Local_Trajectory_Controller().dwa_control(state, conf, np.array([4.0, -0.5]), rock)
    assert u[0] > 0.0


def test_approach_lets_an_off_course_rover_turn_to_the_goal():
    # heading at the goal: damped, and zero on the final metre (no spinning over it)
    assert mission.approach_yaw_rate(0.1, 1.5, 4.5, 0.7, math.radians(5)) == 0.0
    # arriving 60 deg off course: the DWA's turn is kept, so it does not circle the goal
    assert mission.approach_yaw_rate(0.1, 1.5, 4.5, 0.7, math.radians(60)) == 0.1
