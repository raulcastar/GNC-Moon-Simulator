"""
MCP server for the GNC Moon Simulator: lets an AI agent watch and drive the
rover through the mission-control dashboard's JSON API (/api/ on port 5001,
mission_control_dashboard/src/api.py).

Run (stdio):  py mcp/gnc_mcp.py
Claude Code:  claude mcp add gnc -- py "<repo>/mcp/gnc_mcp.py"
Dashboard URL: GNC_DASHBOARD_URL (default http://localhost:5001).

Needs the Docker stack up (docker compose up -d) and, for anything that moves
the rover, Unreal in Play on the active rover's level. Playing / stopping
Unreal and building terrains are in the Terrain Studio MCP server.
"""

import json
import os
import time
import urllib.error
import urllib.request

from mcp.server.mcpserver import Image, MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import ToolAnnotations

DASHBOARD = os.environ.get('GNC_DASHBOARD_URL', 'http://localhost:5001').rstrip('/')
FINAL_STATES = ('reached', 'aborted', 'cancelled', 'idle')

READ = ToolAnnotations(read_only_hint=True, open_world_hint=False)
ACT = ToolAnnotations(read_only_hint=False, destructive_hint=False, open_world_hint=False)

server = MCPServer(
    name='gnc-moon-simulator',
    instructions=(
        "Controls a simulated lunar rover (ROS 2 GNC stack + Unreal Engine). Start with "
        "get_status: it says whether Unreal is in Play, where the rover is, the mission "
        "state, the active rover and node health. Positions are metres in the planner's "
        "map frame (the same frame as get_status pose); angles are degrees. Prefer "
        "send_goal (autonomous, avoids obstacles) over drive (open loop). After a goal, "
        "call wait_for_mission. Use stop if anything looks wrong."),
)


class DashboardError(ToolError):
    """A failed dashboard call; its message reaches the agent (other exceptions' text does not)."""


def _request(method, path, body=None, timeout=30.0, raw=False):
    """Calls the dashboard API; returns JSON (or (bytes, headers) when raw). Raises DashboardError."""
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f'{DASHBOARD}{path}', data=data, method=method,
                                 headers={'Content-Type': 'application/json'} if data else {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as reply:
            payload = reply.read()
            return (payload, dict(reply.headers)) if raw else json.loads(payload)
    except urllib.error.HTTPError as exc:
        try:
            text = json.loads(exc.read()).get('error', str(exc))
        except Exception:
            text = str(exc)
        raise DashboardError(text) from None
    except (urllib.error.URLError, TimeoutError, ConnectionError) as exc:
        raise DashboardError(
            f'Dashboard not reachable at {DASHBOARD} ({exc}). Start the stack: '
            'docker compose up -d (repo root), then retry.') from None


def _get(path, **kw):
    return _request('GET', path, **kw)


def _post(path, body=None, **kw):
    return _request('POST', path, body or {}, **kw)


@server.tool(annotations=READ)
def get_status() -> dict:
    """
    The whole picture: simulation_running (Unreal in Play), pose (x, y, z m; yaw,
    roll, pitch, tilt deg; speed m/s), mission (state: idle / planning / driving /
    reached / aborted / cancelled, goal, reason), route, active rover and the one
    Unreal plays, obstacle avoidance, headlights, lifecycle node states, feed rates
    (hz, age_s), alerts and recording.
    """
    return _get('/api/status')


@server.tool(annotations=READ)
def get_log(lines: int = 40, min_level: str = 'INFO') -> dict:
    """Recent ROS log lines from every node (newest last). min_level: INFO, WARN or ERROR."""
    return _get(f'/api/log?n={int(lines)}&level={min_level}')


@server.tool(annotations=READ)
def get_camera_image(feed: str = 'cam_central') -> Image:
    """
    Newest frame of a camera feed, as a JPEG. Feeds: cam_central, cam_left,
    cam_right, cam_rear (colour navcams); depth, depth_gt (stereo / ground-truth
    depth, colourised); mask, rear_mask (segmentation); shadow, rear_shadow;
    events (event camera); spad (SPAD camera). Needs Unreal in Play.
    """
    data, _headers = _get(f'/api/camera/{feed}', raw=True)
    return Image(data=data, format='jpeg')


@server.tool(annotations=ACT)
def send_goal(x: float, y: float) -> dict:
    """
    Sends the rover autonomously to map point (x, y) metres. The planner routes
    around steep slopes and lidar obstacles. Returns at once: follow it with
    wait_for_mission or get_status. Replaces any current goal.
    """
    return _post('/api/goal', {'x': x, 'y': y})


@server.tool(annotations=READ)
def wait_for_mission(timeout_s: float = 120.0) -> dict:
    """
    Waits until the current goal ends (reached, aborted or cancelled) or timeout_s
    passes; returns the final mission, pose and how long it took. Aborted means
    the planner gave up (tilt limit, stalled, no route): see mission.reason.
    """
    t0 = time.monotonic()
    status = _get('/api/status')
    while time.monotonic() - t0 < timeout_s:
        state = (status.get('mission') or {}).get('state')
        if state in FINAL_STATES and time.monotonic() - t0 > 1.0:
            break
        time.sleep(1.0)
        status = _get('/api/status')
    state = (status.get('mission') or {}).get('state')
    return {'finished': state in FINAL_STATES, 'waited_s': round(time.monotonic() - t0, 1),
            'mission': status.get('mission'), 'pose': status.get('pose')}


@server.tool(annotations=ACT)
def stop() -> dict:
    """Emergency stop: cancels the goal and stops the rover."""
    return _post('/api/stop')


@server.tool(annotations=ACT)
def drive(linear: float, angular: float = 0.0, seconds: float = 2.0) -> dict:
    """
    Open-loop manual drive for `seconds` (max 15), then stops; blocks until done.
    linear m/s (forward +, max 0.25), angular rad/s (left +; max 0.35 Leo_Raph,
    0.15 Yutu). Values are clamped. Cancels an autonomous goal. Returns the pose
    before and after.
    """
    return _post('/api/drive', {'linear': linear, 'angular': angular, 'seconds': seconds},
                 timeout=float(min(max(seconds, 0.0), 15.0)) + 15.0)


@server.tool(annotations=ACT)
def set_headlights(on: bool | None = None, level: float | None = None) -> dict:
    """Headlights on/off and/or brightness level 0..1 (either or both)."""
    body = {k: v for k, v in (('on', on), ('level', level)) if v is not None}
    return _post('/api/headlights', body)


@server.tool(annotations=ACT)
def set_obstacle_avoidance(on: bool) -> dict:
    """Turns the planner's lidar obstacle avoidance on or off (on is the default)."""
    return _post('/api/avoidance', {'on': on})


@server.tool(annotations=ACT)
def set_viewport_camera(mode: str) -> dict:
    """Unreal's viewport camera: follow (behind the rover), orbit (circles it) or top (from above)."""
    return _post('/api/viewport', {'mode': mode})


@server.tool(annotations=READ)
def get_rover() -> dict:
    """The rover the stack drives (active), the rovers available, and the one Unreal plays (match)."""
    return _get('/api/rover')


@server.tool(annotations=ToolAnnotations(read_only_hint=False, destructive_hint=True, open_world_hint=False))
def switch_rover(rover: str) -> dict:
    """
    Switches the whole stack to another rover ('leo_raph' or 'yutu'): stops the
    rover, drops the goal, applies its calibration, footprint and URDF, and
    re-configures the planner and local mapping (~10-20 s; then get_rover busy is
    false). Unreal must play that rover's level (get_rover lists them).
    """
    _post('/api/rover', {'rover': rover})
    t0 = time.monotonic()
    state = _get('/api/rover')
    while state.get('busy') and time.monotonic() - t0 < 120.0:
        time.sleep(1.0)
        state = _get('/api/rover')
    return state


@server.tool(annotations=ToolAnnotations(read_only_hint=False, destructive_hint=True, open_world_hint=False))
def restart_node(name: str) -> dict:
    """
    Re-configures a lifecycle node (deactivate, cleanup, configure, activate), as
    the dashboard's System window does. For a node stuck or in error. Names are
    the keys of get_status nodes (e.g. lc_component_stereo_depth).
    """
    return _post('/api/restart_node', {'name': name})


@server.tool(annotations=ACT)
def record_bag(action: str, preset: str | None = None, label: str = '', notes: str = '') -> dict:
    """
    Starts ('start') or stops ('stop') a rosbag recording. preset: a BAGS-window
    preset (navigation or perception); default the last one used. Bags are in
    mission_control_dashboard/bags.
    """
    body = {'action': action, 'label': label, 'notes': notes}
    if preset:
        body['preset'] = preset
    return _post('/api/record', body)


if __name__ == '__main__':
    server.run()
