"""
Rover cameras for the Play viewport: the rover's own follow camera, an
orbit camera circling the rover, or a top-down camera above it. Runs inside the Unreal Editor during Play
(PIE); install by copying this file to <Project>/Content/Python/.

In the editor's Output Log console (Cmd box set to "Python"):

    import rover_cameras as rc
    rc.orbit()              # the viewport circles the rover
    rc.orbit(radius=6, height=2.5, period=45, direction=-1)   # wider, higher, slower, reversed
    rc.top()                # straight down from 8 m, the rover pointing up the image
    rc.top(height=15, heading_up=False)       # higher, fixed like the dashboard map (+y up)
    rc.follow()             # back to the rover's follow camera
    rc.install()            # once, with Play stopped: places the orbit camera in the level

or as console commands:  py rover_cameras.py orbit | top | follow

From the mission control dashboard: start_dashboard_link() (run at editor
startup by rover_cameras_startup.py, a project startup script) polls the
dashboard's GET /viewport_camera twice a second and applies the mode chosen
there (the overview's VIEW button) in every Play session, reporting back the
mode it shows.

The orbit camera is its own actor, moved every frame by an editor tick: it
circles the rover at `radius` m and `height` m above its base_link, one turn
every `period` simulated seconds, always looking at the rover with a level
horizon. It follows the rover smoothly (hops do not shake it), stays above the
ground, and pulls in when terrain or a rock gets between it and the rover. The
top-down camera (the same actor) hangs `height` m above the rover looking
straight down, following its position and, with heading_up, turning with it so
the rover points to the top of the image. Both only read the rover: nothing on
the rover is changed.
"""

import builtins
import json
import math
import sys
import threading
import time
import urllib.request

import unreal

ROVER_PREFIXES = ("Leo_Raph", "Yutu_Rover")   # the rover Blueprints' class names
TARGET_COMPONENT = "base_link"     # what the camera looks at (+ LOOK_UP_CM)
LOOK_UP_CM = 45.0
FOLLOW_SMOOTHING = 3.0             # 1/s: how fast the orbit centre catches up with the rover
YAW_SMOOTHING = 2.0                # 1/s: how fast the top-down view turns with the rover
MIN_GROUND_CLEARANCE_CM = 60.0
BLEND_S = 1.5                      # viewport blend between cameras
STATE = "_rover_cameras_state"     # kept in builtins across imports
CAMERA_TAG = "RoverOrbitCamera"    # the level's camera this script moves (install())
DASHBOARD_URL = "http://127.0.0.1:5001/viewport_camera"   # the overview's port
POLL_S = 0.5                       # dashboard polling period
RETRY_S = 1.0                      # retry an orbit that could not start (Play still starting)
LINK = "_rover_cameras_link"       # the dashboard link, in builtins


def _state():
    return getattr(builtins, STATE, None)


def _world():
    world = unreal.EditorLevelLibrary.get_game_world()
    if world is None:
        unreal.log_warning("[rover cameras] Unreal is not playing: press Play first.")
    return world


def _orbit_camera(world):
    for actor in unreal.GameplayStatics.get_all_actors_with_tag(world, CAMERA_TAG):
        if isinstance(actor, unreal.CameraActor):
            return actor
    return None


def install():
    """
    Places the orbit camera in the current level (Play stopped) and saves the
    level. Python cannot spawn actors in a running game, so the camera lives in
    the level and Play copies it; orbit() moves it.
    """
    if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
        unreal.log_warning("[rover cameras] stop Play first.")
        return
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    existing = [a for a in actors.get_all_level_actors() if CAMERA_TAG in [str(t) for t in a.tags]]
    if existing:
        unreal.log(f"[rover cameras] {existing[0].get_actor_label()} is already in the level.")
        return
    camera = actors.spawn_actor_from_class(unreal.CameraActor, unreal.Vector(0.0, 0.0, 0.0))
    camera.set_actor_label(CAMERA_TAG)
    camera.tags = [unreal.Name(CAMERA_TAG)]
    camera.get_editor_property("camera_component").set_editor_property("constrain_aspect_ratio", False)
    unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).save_current_level()
    unreal.log(f"[rover cameras] {CAMERA_TAG} placed in the level and saved.")


def _rover(world):
    pawns = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Pawn)
    rovers = [p for p in pawns if p.get_class().get_name().startswith(ROVER_PREFIXES)]
    return rovers[0] if rovers else None


def _target(rover):
    for c in rover.get_components_by_class(unreal.SceneComponent):
        if c.get_name() == TARGET_COMPONENT:
            return c.get_world_location() + unreal.Vector(0.0, 0.0, LOOK_UP_CM)
    return rover.get_actor_location()


def _trace(world, start, end, ignore):
    hit = unreal.SystemLibrary.line_trace_single(
        world, start, end, unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, ignore,
        unreal.DrawDebugTrace.NONE, True)
    if hit is None:
        return None
    parts = hit.to_tuple()
    return parts[5] if parts[0] else None          # impact point, or None


def _place(state, delta_seconds):
    """Moves the camera for this frame: the centre follows the rover smoothly, then orbit or top."""
    target = _target(state["rover"])
    k = 1.0 - math.exp(-FOLLOW_SMOOTHING * max(delta_seconds, 0.0))
    c = state["centre"]
    state["centre"] = unreal.Vector(c.x + (target.x - c.x) * k, c.y + (target.y - c.y) * k,
                                    c.z + (target.z - c.z) * k)
    if state["kind"] == "top":
        _place_top(state, delta_seconds)
    else:
        _place_orbit(state)


def _rover_yaw(rover):
    """The rover's heading in Unreal degrees: its base_link's forward axis."""
    for comp in rover.get_components_by_class(unreal.SceneComponent):
        if comp.get_name() == TARGET_COMPONENT:
            return comp.get_world_rotation().yaw
    return rover.get_actor_rotation().yaw + 90.0


def _place_top(state, delta_seconds):
    c, camera = state["centre"], state["camera"]
    if state["heading_up"]:
        # Turn with the rover, smoothly, by the shortest way round.
        k = 1.0 - math.exp(-YAW_SMOOTHING * max(delta_seconds, 0.0))
        error = (_rover_yaw(state["rover"]) - state["yaw"] + 180.0) % 360.0 - 180.0
        state["yaw"] += error * k
        yaw = state["yaw"]
    else:
        yaw = 90.0          # fixed: Unreal +Y (odom +y) up, as the dashboard's global map
    pos = unreal.Vector(c.x, c.y, c.z + state["top_height"] * 100.0)
    # Pitch just short of -90: straight down without a degenerate yaw.
    camera.set_actor_location_and_rotation(pos, unreal.Rotator(roll=0.0, pitch=-89.9, yaw=yaw), False, False)


def _place_orbit(state):
    world, rover, camera, c = state["world"], state["rover"], state["camera"], state["centre"]

    # Angle from simulation time, so a recording's speed does not depend on the frame rate.
    t = unreal.GameplayStatics.get_time_seconds(world)
    angle = state["phase"] + state["direction"] * 2.0 * math.pi * t / state["period"]
    r, h = state["radius"] * 100.0, state["height"] * 100.0
    pos = unreal.Vector(c.x + r * math.cos(angle), c.y + r * math.sin(angle), c.z + h)

    ignore = [rover, camera]
    ground = _trace(world, unreal.Vector(pos.x, pos.y, pos.z + 5000.0), unreal.Vector(pos.x, pos.y, pos.z - 5000.0), ignore)
    if ground is not None and pos.z < ground.z + MIN_GROUND_CLEARANCE_CM:
        pos.z = ground.z + MIN_GROUND_CLEARANCE_CM
    blocked = _trace(world, c, pos, ignore)            # terrain or a rock between rover and camera
    if blocked is not None:
        d = unreal.Vector(pos.x - c.x, pos.y - c.y, pos.z - c.z)
        length = math.sqrt(d.x * d.x + d.y * d.y + d.z * d.z) or 1.0
        back = 30.0 / length                           # stop 30 cm short of the obstacle
        pos = unreal.Vector(blocked.x - d.x * back, blocked.y - d.y * back, blocked.z - d.z * back)

    look = unreal.MathLibrary.find_look_at_rotation(pos, c)
    camera.set_actor_location_and_rotation(pos, unreal.Rotator(roll=0.0, pitch=look.pitch, yaw=look.yaw), False, False)


def _tick(delta_seconds):
    state = _state()
    if not state:
        return
    try:
        world = unreal.EditorLevelLibrary.get_game_world()
        if world is None or world != state["world"] or not unreal.SystemLibrary.is_valid(state["camera"]):
            _stop_tick("Play ended")
            return
        _place(state, delta_seconds)
    except Exception as error:                          # never let the tick raise every frame
        _stop_tick(f"stopped: {error}")


def _stop_tick(reason):
    state = _state()
    if state and state.get("handle") is not None:
        unreal.unregister_slate_post_tick_callback(state["handle"])
    setattr(builtins, STATE, None)
    unreal.log(f"[rover cameras] moving camera off ({reason}).")


def _session(fov):
    """The moving camera's state for this Play session (created on first use); None if impossible."""
    world = _world()
    if world is None:
        return None
    rover = _rover(world)
    if rover is None:
        unreal.log_warning("[rover cameras] no rover in the game.")
        return None
    state = _state()
    if state is None or state["world"] != world or not unreal.SystemLibrary.is_valid(state.get("camera")):
        if state:
            _stop_tick("restarted")
        camera = _orbit_camera(world)
        if camera is None:
            unreal.log_warning(f"[rover cameras] no camera tagged {CAMERA_TAG} in the level: "
                               "stop Play and run rover_cameras.install() once.")
            return None
        # Orbit starts behind the rover, where the follow camera is, so the blend is short.
        state = {"world": world, "rover": rover, "camera": camera, "centre": _target(rover), "handle": None,
                 "phase": math.radians(rover.get_actor_rotation().yaw + 90.0 + 180.0),
                 "yaw": _rover_yaw(rover), "kind": None}
        setattr(builtins, STATE, state)
        state["handle"] = unreal.register_slate_post_tick_callback(_tick)
    component = state["camera"].get_editor_property("camera_component")
    component.set_editor_property("field_of_view", float(fov))
    component.set_editor_property("constrain_aspect_ratio", False)
    return state


def _show(state):
    """The viewport blends to the moving camera."""
    _place(state, 10.0)                                # jump to this mode's first position
    controller = unreal.GameplayStatics.get_player_controller(state["world"], 0)
    controller.set_view_target_with_blend(state["camera"], BLEND_S)


def top(height=8.0, heading_up=True, fov=60.0):
    """The viewport looks straight down at the rover from height m; heading_up turns with it."""
    state = _session(fov)
    if state is None:
        return False
    state.update(kind="top", top_height=float(height), heading_up=bool(heading_up))
    _show(state)
    unreal.log(f"[rover cameras] top-down on: {height} m up, {'heading up' if heading_up else 'fixed, map up'}.")
    return True


def orbit(radius=3.5, height=1.2, period=36.0, direction=1, fov=60.0):
    """The viewport circles the rover: radius and height in m, one turn every period s."""
    state = _session(fov)
    if state is None:
        return False
    world = state["world"]
    state["kind"] = "orbit"
    t = unreal.GameplayStatics.get_time_seconds(world)
    # Keep the camera where it is when the speed or direction changes.
    if "period" in state:
        current = state["phase"] + state["direction"] * 2.0 * math.pi * t / state["period"]
        state["phase"] = current - direction * 2.0 * math.pi * t / period
    else:
        state["phase"] -= direction * 2.0 * math.pi * t / period
    state.update(radius=float(radius), height=float(height), period=float(period),
                 direction=1 if direction >= 0 else -1)
    _show(state)
    unreal.log(f"[rover cameras] orbit on: {radius} m, {height} m up, a turn every {period} s.")
    return True


def follow():
    """Back to the rover's own follow camera."""
    world = _world()
    if world is None:
        return
    rover = _rover(world)
    controller = unreal.GameplayStatics.get_player_controller(world, 0)
    if rover is not None:
        controller.set_view_target_with_blend(rover, BLEND_S)
    state = _state()
    if state:
        _stop_tick("back to the follow camera")


# =============================================================================
# The dashboard link
# =============================================================================

def _link():
    return getattr(builtins, LINK, None)


def _poll_loop(link):
    """Background thread: asks the dashboard which mode it wants, tells it what is shown."""
    while not link["stop"].is_set():
        try:
            with urllib.request.urlopen(f"{link['url']}?applied={link['applied']}", timeout=1.0) as reply:
                mode = json.loads(reply.read().decode()).get("mode")
            link["wanted"] = mode if mode in ("follow", "orbit", "top") else None
        except Exception:
            link["wanted"] = None                        # dashboard not running: leave the view alone
        link["stop"].wait(POLL_S)


def _link_tick(delta_seconds):
    """Editor tick: applies the dashboard's mode once per Play session, and on every change."""
    link = _link()
    if not link:
        return
    try:
        world = unreal.EditorLevelLibrary.get_game_world()
        if world is None:
            link["applied"], link["world"] = "idle", None
            return
        wanted = link["wanted"]
        if wanted is None:
            return
        new_session = world != link["world"]
        if not new_session and wanted == link["applied"]:
            return
        if wanted == "follow":
            if not new_session:                          # a new session starts on the follow camera
                follow()
            link["applied"], link["world"] = "follow", world
        elif time.monotonic() - link["tried"] >= RETRY_S:
            link["tried"] = time.monotonic()
            if {"orbit": orbit, "top": top}[wanted]():     # may fail while Play is still starting
                link["applied"], link["world"] = wanted, world
    except Exception as error:                           # never let the tick raise every frame
        if not link.get("warned"):
            link["warned"] = True
            unreal.log_warning(f"[rover cameras] dashboard link: {error}")


def start_dashboard_link(url=DASHBOARD_URL):
    """Follows the dashboard's viewport camera choice (idempotent)."""
    if _link():
        return
    link = {"url": url, "wanted": None, "applied": "idle", "world": None, "tried": 0.0,
            "stop": threading.Event(), "handle": None}
    setattr(builtins, LINK, link)
    threading.Thread(target=_poll_loop, args=(link,), name="rover_cameras_link", daemon=True).start()
    link["handle"] = unreal.register_slate_post_tick_callback(_link_tick)
    unreal.log(f"[rover cameras] following the dashboard's viewport camera ({url}).")


def stop_dashboard_link():
    link = _link()
    if link:
        link["stop"].set()
        unreal.unregister_slate_post_tick_callback(link["handle"])
        setattr(builtins, LINK, None)
        unreal.log("[rover cameras] dashboard link stopped.")


if __name__ == "__main__":
    command = sys.argv[1] if len(sys.argv) > 1 else "orbit"
    {"orbit": orbit, "top": top, "follow": follow, "link": start_dashboard_link,
     "unlink": stop_dashboard_link}.get(command, orbit)()
