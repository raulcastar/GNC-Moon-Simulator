"""The mission logic (mission): states and decisions, cycle by cycle."""

import json
import math

import numpy as np
import pytest

from src import mission as m
from src.global_planner import Route

SETTINGS = m.MissionSettings(stop_tolerance=0.7, max_tilt_deg=25.0, replan_lookahead_m=5.0,
                             stall_window_s=15.0, max_stalls=3)
ROUTE = np.column_stack((np.arange(0.0, 21.0), np.zeros(21)))
GOAL = [20.0, 0.0]


class Checker:
    """A stand-in for GlobalPlanner.blocks_route: blocked when told, and records the cells it saw."""
    def __init__(self):
        self.blocked, self.seen = False, []

    def __call__(self, path, rover_xy, goal, cells, lookahead_m):
        self.seen.append(set(cells))
        return self.blocked


def driving(cells=frozenset()):
    """A mission driving ROUTE to GOAL, planned with `cells`."""
    checker = Checker()
    mission = m.Mission(SETTINGS, checker)
    mission.set_goal(GOAL)
    goal, generation = mission.planning_request()
    event = mission.route_planned(generation, Route(True, ROUTE, goal, message='ok'), cells)
    assert event.state == m.DRIVING and mission.state == m.DRIVING
    return mission, checker


# --- goals and plans --------------------------------------------------------

def test_no_goal_does_nothing():
    assert m.Mission(SETTINGS, Checker()).step((0, 0, 0), 0.0, 0.0).action == m.DO_NOTHING


def test_a_goal_is_planned_then_driven():
    mission = m.Mission(SETTINGS, Checker())
    event = mission.set_goal((20, 0))
    assert event.state == m.PLANNING and event.goal == GOAL
    assert mission.step((0, 0, 0), 0.0, 0.0).action == m.PLAN
    goal, generation = mission.planning_request()
    mission.route_planned(generation, Route(True, ROUTE, goal), frozenset())
    assert mission.step((0, 0, 0), 0.0, 1.0).action == m.DRIVE


def test_a_plan_for_an_old_goal_is_dropped():
    mission = m.Mission(SETTINGS, Checker())
    mission.set_goal((20, 0))
    _, old = mission.planning_request()
    mission.set_goal((5, 5))
    assert mission.route_planned(old, Route(True, ROUTE, GOAL), frozenset()) is None
    assert mission.route is None and mission.goal == [5.0, 5.0]


def test_an_impossible_goal_is_aborted():
    mission = m.Mission(SETTINGS, Checker())
    mission.set_goal((20, 0))
    _, generation = mission.planning_request()
    event = mission.route_planned(generation, Route(False, reason='goal not reachable', message='no'), frozenset())
    assert event.state == m.ABORTED and event.reason == 'goal not reachable' and event.goal == GOAL
    assert mission.goal is None and mission.step((0, 0, 0), 0.0, 0.0).action == m.DO_NOTHING


def test_the_snapped_goal_is_the_one_driven_to():
    mission = m.Mission(SETTINGS, Checker())
    mission.set_goal((20.4, 0.3))
    _, generation = mission.planning_request()
    mission.route_planned(generation, Route(True, ROUTE, [20.0, 0.0]), frozenset())
    assert mission.goal == [20.0, 0.0]


# --- the control cycle ------------------------------------------------------

def test_arrival():
    mission, _ = driving()
    step = mission.step((19.5, 0.2, 0.0), 0.0, 0.0)
    assert step.action == m.STOP and step.event.state == m.REACHED and step.event.goal == GOAL
    assert mission.goal is None


def test_tilt_aborts_before_anything_else():
    mission, _ = driving()
    step = mission.step((19.9, 0.0, 0.0), 30.0, 0.0)       # even at the goal
    assert step.action == m.STOP and step.event.state == m.ABORTED and step.event.level == 'error'
    assert 'tilt' in step.event.reason and mission.goal is None


def test_a_stall_replans_then_aborts():
    mission, _ = driving()
    t = 0.0
    for attempt in (1, 2):
        assert mission.step((1.0, 0.0, 0.0), 0.0, t).action == m.DRIVE
        step = mission.step((1.0, 0.0, 0.0), 0.0, t + 15.0)
        assert step.action == m.PLAN and step.event.reason == 'no progress'
        assert mission.step((1.0, 0.0, 0.0), 0.0, t + 15.1).action == m.PLAN    # route gone: plan
        goal, generation = mission.planning_request()
        mission.route_planned(generation, Route(True, ROUTE, goal), frozenset())
        t += 20.0
    mission.step((1.0, 0.0, 0.0), 0.0, t)
    step = mission.step((1.0, 0.0, 0.0), 0.0, t + 15.0)
    assert step.action == m.STOP and step.event.state == m.ABORTED and 'no progress' in step.event.reason


def test_moving_clears_the_stalls():
    mission, _ = driving()
    mission.step((1.0, 0.0, 0.0), 0.0, 0.0)
    mission.step((1.0, 0.0, 0.0), 0.0, 15.0)                # stall 1: re-plan
    goal, generation = mission.planning_request()
    mission.route_planned(generation, Route(True, ROUTE, goal), frozenset())
    mission.step((1.0, 0.0, 0.0), 0.0, 16.0)
    mission.step((3.0, 0.0, 0.0), 0.0, 20.0)                # it moved
    assert mission.progress.stalls == 0


def test_only_new_obstacle_cells_can_block_the_route():
    """Re-planning for the cells the route was planned with gives the same route: a loop."""
    known = frozenset({(5, 1), (6, 1)})
    mission, checker = driving(known)
    checker.blocked = True
    assert mission.step((1.0, 0.0, 0.0), 0.0, 0.0, known).action == m.DRIVE
    assert checker.seen == []                               # nothing new: not even checked
    step = mission.step((1.0, 0.0, 0.0), 0.0, 1.0, known | {(4, 0)})
    assert step.action == m.PLAN and step.event.reason == 'obstacle on the route'
    assert checker.seen == [{(4, 0)}]


def test_a_new_cell_off_the_route_keeps_driving():
    mission, checker = driving()
    checker.blocked = False
    assert mission.step((1.0, 0.0, 0.0), 0.0, 0.0, {(4, 9)}).action == m.DRIVE


# --- the operator and sessions ----------------------------------------------

def test_cancel():
    mission, _ = driving()
    event = mission.cancel()
    assert event.state == m.CANCELLED and event.goal == GOAL and event.level == 'warn'
    assert mission.goal is None and mission.cancel() is None      # nothing left to cancel


def test_a_new_session_drops_everything():
    mission, _ = driving()
    _, generation = mission.planning_request()
    event = mission.new_session()
    assert event.state == m.IDLE and event.reason == 'new session' and event.goal is None
    assert mission.route_planned(generation, Route(True, ROUTE, GOAL), frozenset()) is None


# --- progress and messages --------------------------------------------------

def test_progress_watch_counts_stalls_and_resets_on_moving():
    w = m.ProgressWatch(window_s=10.0, min_move_m=0.25, min_turn_deg=20.0)
    assert not w.update(0.0, 0.0, 0.0, 0.0)
    assert not w.update(9.0, 0.1, 0.0, 0.0)
    assert w.update(10.5, 0.1, 0.0, 0.0) and w.stalls == 1
    assert w.update(21.0, 0.1, 0.0, 0.0) and w.stalls == 2
    assert not w.update(25.0, 0.6, 0.0, 0.0) and w.stalls == 0      # it moved


def test_progress_watch_spot_turns_are_progress_but_do_not_clear_stalls():
    w = m.ProgressWatch(window_s=10.0)
    w.update(0.0, 0.0, 0.0, 0.0)
    assert w.update(11.0, 0.0, 0.0, 0.0) and w.stalls == 1
    assert not w.update(20.0, 0.0, 0.0, math.radians(30))           # a spot turn: progress
    assert w.stalls == 1
    w.pause()                                                         # planning: the window restarts
    assert not w.update(100.0, 0.0, 0.0, math.radians(30))
    assert not w.update(105.0, 0.0, 0.0, math.radians(30))
    w.reset()
    assert w.stalls == 0


def test_state_json():
    assert json.loads(m.state_json('driving', [1.234, 5.678])) == \
        {'state': 'driving', 'goal': [1.23, 5.68], 'reason': ''}
    assert json.loads(m.state_json('idle', None, 'new session'))['goal'] is None


@pytest.mark.parametrize("state", [m.IDLE, m.PLANNING, m.DRIVING, m.REACHED, m.ABORTED, m.CANCELLED])
def test_states_are_the_published_names(state):
    assert state in ('idle', 'planning', 'driving', 'reached', 'aborted', 'cancelled')
