"""
Mission logic: what the planner does with a goal, cycle by cycle. No ROS.

States (published on mission_status/state as state_json):
    idle       no goal
    planning   a route is being computed; the rover is held still
    driving    following the route with the local planner
    reached    parked within the stop tolerance of the goal
    aborted    given up (unreachable goal, tilt, no progress); `reason` says why
    cancelled  stopped by the operator

Every control cycle, Mission.step() decides, in this order:
    1. no goal                                    -> idle
    2. body tilt over the limit                   -> abort the goal
    3. no route                                   -> plan
    4. within the stop tolerance                  -> reached
    5. no progress for stall_window_s             -> plan again (abort after max_stalls in a row)
    6. a newly found obstacle on the route ahead  -> plan again
    7. otherwise                                  -> drive

Re-planning around obstacles: a route is planned with the obstacle cells
known at that moment, and only cells found since can make it blocked (step
6). Planning again with the same cells would give the same route, so
re-planning for them loops.

Routes are computed outside (a worker thread in the node): the node asks
planning_request() for the goal, plans, and hands the result to
route_planned(). `generation` changes whenever the wanted route does (new
goal, cancel, abort, new session), so a route finished for an older one is
dropped and a running search can stop early.
"""

import json
import math
from dataclasses import dataclass
from typing import Callable, Optional

from src.global_planner import Route

IDLE, PLANNING, DRIVING, REACHED, ABORTED, CANCELLED = (
    'idle', 'planning', 'driving', 'reached', 'aborted', 'cancelled')

# What the node does this cycle
DO_NOTHING, PLAN, DRIVE, STOP = 'nothing', 'plan', 'drive', 'stop'


@dataclass
class MissionSettings:
    stop_tolerance: float       # m from the goal to park
    max_tilt_deg: float         # body tilt that aborts the goal
    replan_lookahead_m: float   # route checked for new obstacles this far ahead
    stall_window_s: float       # no progress for this long -> plan again
    max_stalls: int             # ... this many times in a row -> abort


@dataclass
class Event:
    """A state change, for the node to publish and log."""
    state: str
    goal: Optional[list]
    reason: str = ''
    message: str = ''
    level: str = 'info'         # 'info' | 'warn' | 'error'


@dataclass
class Step:
    action: str                 # DO_NOTHING | PLAN | DRIVE | STOP
    event: Optional[Event] = None


# route_blocked(path, rover_xy, goal_xy, new_obstacle_cells, lookahead_m) -> bool
RouteCheck = Callable[..., bool]


class Mission:
    """The mission state machine; see the module docstring for the cycle."""

    def __init__(self, settings: MissionSettings, route_blocked: RouteCheck):
        self.settings = settings
        self.route_blocked = route_blocked
        self.state = IDLE
        self.goal: Optional[list] = None
        self.route = None                   # (N, 2) metres
        self.route_cells: frozenset = frozenset()   # obstacle cells the route was planned with
        self.generation = 0
        self.progress = ProgressWatch()

    # -------------------------------------------------------------------------
    # Requests
    # -------------------------------------------------------------------------

    def set_goal(self, goal) -> Event:
        self._new_generation(goal=[float(goal[0]), float(goal[1])])
        self.progress.reset()
        return self._event(PLANNING)

    def cancel(self) -> Optional[Event]:
        """Operator ALL STOP: drop the goal (an event only if there was one)."""
        had_goal = self.goal is not None
        event = self._event(CANCELLED, 'by the operator (ALL STOP or manual drive)',
                            f"Goal {self.goal} cancelled by the operator.", 'warn')
        self._new_generation(goal=None)
        return event if had_goal else None

    def new_session(self) -> Event:
        """Unreal Play/Stop: everything from the previous run is stale."""
        self._new_generation(goal=None)
        self.progress.reset()
        return self._event(IDLE, 'new session')

    def invalidate(self) -> None:
        """Deactivation: a route being planned is no longer wanted."""
        self.generation += 1

    # -------------------------------------------------------------------------
    # Planning (the node's worker thread)
    # -------------------------------------------------------------------------

    def planning_request(self):
        """(goal, generation) to plan for, or None without a goal."""
        return None if self.goal is None else (list(self.goal), self.generation)

    def route_planned(self, generation: int, route: Route, obstacle_cells) -> Optional[Event]:
        """Applies a finished plan; None (dropped) if the wanted route changed meanwhile."""
        if generation != self.generation:
            return None
        if not route.ok:
            event = self._event(ABORTED, route.reason, route.message, 'warn')
            self._new_generation(goal=None)
            return event
        self.goal, self.route = route.goal, route.path
        self.route_cells = frozenset(obstacle_cells)
        return self._event(DRIVING, message=route.message)

    # -------------------------------------------------------------------------
    # The control cycle
    # -------------------------------------------------------------------------

    def step(self, pose, tilt_deg: float, now: float, obstacle_cells=frozenset()) -> Step:
        """What to do this cycle, at `pose` (x, y, yaw); `now` in seconds."""
        s = self.settings
        if self.goal is None:
            return Step(DO_NOTHING)

        if tilt_deg > s.max_tilt_deg:
            return self._abort(f'tilt {tilt_deg:.1f} deg > {s.max_tilt_deg:.1f} deg',
                               f"Tilt safety stop: body tilt {tilt_deg:.1f} deg exceeds "
                               f"{s.max_tilt_deg:.1f} deg. Goal {self.goal} aborted.", 'error')

        if self.route is None:
            self.progress.pause()
            return Step(PLAN)

        x, y, yaw = pose
        if math.hypot(x - self.goal[0], y - self.goal[1]) <= s.stop_tolerance:
            event = self._event(REACHED, message=f"Goal {self.goal} reached.")
            self._new_generation(goal=None)
            return Step(STOP, event)

        self.progress.window_s = s.stall_window_s
        if self.progress.update(now, x, y, yaw):
            stalls = self.progress.stalls
            if stalls >= s.max_stalls:
                return self._abort(f'no progress ({stalls} stalls): blocked near an obstacle?',
                                   f"No progress towards {self.goal} after {stalls} attempts: goal aborted.",
                                   'warn')
            return self._replan('no progress',
                                f"No progress for {s.stall_window_s:.0f} s ({stalls}/{s.max_stalls}): "
                                f"planning again.", 'warn')

        new_cells = set(obstacle_cells) - self.route_cells
        if new_cells and self.route_blocked(self.route, (x, y), self.goal, new_cells, s.replan_lookahead_m):
            return self._replan('obstacle on the route', "Obstacle on the route ahead (lidar): planning around it.")

        return Step(DRIVE)

    # -------------------------------------------------------------------------

    def _replan(self, reason: str, message: str, level: str = 'info') -> Step:
        self.route = None
        return Step(PLAN, self._event(PLANNING, reason, message, level))

    def _abort(self, reason: str, message: str, level: str) -> Step:
        event = self._event(ABORTED, reason, message, level)
        self._new_generation(goal=None)
        return Step(STOP, event)

    def _new_generation(self, goal) -> None:
        """A different route is wanted from now on (or none)."""
        self.generation += 1
        self.goal, self.route, self.route_cells = goal, None, frozenset()

    def _event(self, state: str, reason: str = '', message: str = '', level: str = 'info') -> Event:
        self.state = state
        return Event(state, None if self.goal is None else list(self.goal), reason, message, level)


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


def state_json(state: str, goal, reason: str = '') -> str:
    """
    The mission state as published on mission_status/state:
    {"state": idle|planning|driving|reached|aborted|cancelled,
     "goal": [x, y] or null, "reason": str}.
    """
    goal = [round(float(v), 2) for v in goal] if goal else None
    return json.dumps({'state': state, 'goal': goal, 'reason': reason})
