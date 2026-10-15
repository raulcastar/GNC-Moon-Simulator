"""
JSON API on the overview's port (5001, under /api/), for scripts and AI agents
(the MCP server in mcp/gnc_mcp.py wraps it). The browsers use SocketIO; this
gives the same state and commands over plain HTTP:

  GET  /api/status                 pose, mission, rover, nodes, feeds, alerts, recording
  GET  /api/log?n=50&level=WARN    recent ROS log lines (/rosout)
  GET  /api/camera/<feed>          newest JPEG of a feed (see FEED_NAMES), with its age
  POST /api/goal {x, y}            drive autonomously to a map point (metres)
  POST /api/stop                   cancel the goal and stop
  POST /api/drive {linear, angular, seconds}   manual drive, clamped, then stops
  POST /api/headlights {on?, level?}
  POST /api/avoidance {on}         the planner's lidar obstacle avoidance
  POST /api/viewport {mode}        Unreal's viewport camera: follow / orbit / top
  GET|POST /api/rover {rover?}     the active rover / switch the stack (rovers.py)
  POST /api/restart_node {name}    re-configure a lifecycle node
  POST /api/record {action: start|stop, preset?, label?, notes?}

Commands that move the rover are refused while Unreal is not in Play. Errors
come back as {"error": text} with a 4xx/5xx status.
"""

import threading
import time

from flask import Response, jsonify, request

from . import web
from .settings import BAG_PRESETS, FEED_NAMES, LIFECYCLE_NODES
from .viewport_camera import MODES, VIEWPORT
from .web import app

DRIVE_MAX_LINEAR = 0.25         # m/s: the teleop window's limit
DRIVE_MAX_SECONDS = 15.0
DRIVE_PERIOD = 0.2              # s between commands (the teleop watchdog stops after 0.6 s)
CAMERA_WAIT_S = 4.0             # s to wait for a fresh frame of a feed nobody was watching
CAMERA_LEASE_S = 20.0           # s the raw feeds stay subscribed after an API request

_drive_lock = threading.Lock()
_drive = {'stop': None}         # the running timed drive's stop event


def _node():
    return web.node


def _error(text, status=400):
    return jsonify({'error': text}), status


def _body():
    return request.get_json(silent=True) or {}


def _number(body, key, default=None):
    value = body.get(key, default)
    if value is None:
        raise ValueError(f'{key} is required')
    value = float(value)
    if value != value:
        raise ValueError(f'{key} is not a number')
    return value


def _require_play(node):
    if not node._live_simulation():
        return _error('Unreal is not in Play (no simulation clock): press Play first.', 409)
    if not node._is_active():
        return _error('The dashboard node is not active.', 503)
    return None


def status_payload(node) -> dict:
    """Everything an agent needs to know at a glance."""
    now = time.monotonic()
    feeds = {}
    for name, times in list(node._arrivals.items()):
        if times:
            recent = sum(1 for t in times if now - t <= 3.0)
            feeds[name] = {'hz': round(recent / 3.0, 1), 'age_s': round(now - times[-1], 1)}
    route = node._route
    return {
        'simulation_running': node._live_simulation(),
        'pose': node._last_odom,          # x, y, z (m, odom/map), yaw/roll/pitch/tilt (deg), speed (m/s)
        'mission': {k: v for k, v in node._mission.items() if k != 't'},
        'route': {'goal': route.get('goal'), 'waypoints': len(route.get('current') or []),
                  'replans': route.get('replans', 0)},
        'rover': node.rover_status(),
        'obstacle_avoidance': node._avoidance,
        'headlight_level': node._light_level,
        'viewport_camera': VIEWPORT.status(),
        'nodes': dict(node._node_states),
        'feeds': feeds,
        'alerts': list(node._last_alerts),
        'recording': node._bag_status(),
    }


@app.route('/api/status')
def api_status():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    return jsonify(status_payload(node))


@app.route('/api/log')
def api_log():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    order = ['INFO', 'WARN', 'ERROR', 'FATAL']
    minimum = request.args.get('level', 'INFO').upper()
    keep = set(order[order.index(minimum):]) if minimum in order else set(order)
    n = max(1, min(int(request.args.get('n', 50)), 500))
    lines = [e for e in list(node._log_lines) if e['level'] in keep]
    return jsonify({'lines': lines[-n:]})


@app.route('/api/camera/<feed>')
def api_camera(feed):
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    if feed not in FEED_NAMES or feed in ('odom', 'imu', 'lidar', 'local_dem'):
        cams = [k for k in FEED_NAMES if k not in ('odom', 'imu', 'lidar', 'local_dem')]
        return _error(f'Unknown camera feed {feed!r}; one of {cams}.', 404)
    # Raw feeds are subscribed only while something watches them: hold them for a while.
    web.api_watch(CAMERA_LEASE_S)
    t0 = time.monotonic()
    while True:
        times = node._arrivals.get(feed)
        age = time.monotonic() - times[-1] if times else None
        if (age is not None and age < 1.0) or time.monotonic() - t0 > CAMERA_WAIT_S:
            break
        time.sleep(0.1)
    jpeg = node.latest_jpegs().get(feed)
    if jpeg is None:
        return _error(f'No frame of {feed} yet: is Unreal in Play and the camera node running?', 404)
    return Response(jpeg, mimetype='image/jpeg',
                    headers={'X-Frame-Age-S': f'{age:.1f}' if age is not None else 'unknown'})


@app.route('/api/goal', methods=['POST'])
def api_goal():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    refused = _require_play(node)
    if refused:
        return refused
    try:
        x, y = _number(_body(), 'x'), _number(_body(), 'y')
    except (TypeError, ValueError) as exc:
        return _error(str(exc))
    _stop_drive()
    node.dispatch_goal(x, y)
    return jsonify({'ok': True, 'goal': [x, y],
                    'note': 'Follow progress in /api/status mission.state (planning, driving, reached, aborted).'})


@app.route('/api/stop', methods=['POST'])
def api_stop():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    _stop_drive()
    node.stop_rover()
    return jsonify({'ok': True})


def _stop_drive():
    with _drive_lock:
        if _drive['stop'] is not None:
            _drive['stop'].set()
            _drive['stop'] = None


@app.route('/api/drive', methods=['POST'])
def api_drive():
    """Drives at (linear, angular) for `seconds`, then stops. Blocks until done."""
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    refused = _require_play(node)
    if refused:
        return refused
    body = _body()
    try:
        linear = _number(body, 'linear', 0.0)
        angular = _number(body, 'angular', 0.0)
        seconds = _number(body, 'seconds')
    except (TypeError, ValueError) as exc:
        return _error(str(exc))
    max_angular = float(node._remote.get('teleop_max_angular') or 0.2)
    clamped = (max(-DRIVE_MAX_LINEAR, min(DRIVE_MAX_LINEAR, linear)),
               max(-max_angular, min(max_angular, angular)),
               max(0.0, min(DRIVE_MAX_SECONDS, seconds)))
    stop = threading.Event()
    _stop_drive()
    with _drive_lock:
        _drive['stop'] = stop
    start = node._last_odom
    t0 = time.monotonic()
    while time.monotonic() - t0 < clamped[2] and not stop.is_set():
        node.teleop_drive(clamped[0], clamped[1])
        stop.wait(DRIVE_PERIOD)
    node.teleop_drive(0.0, 0.0)
    node._teleop_engaged = False
    with _drive_lock:
        if _drive['stop'] is stop:
            _drive['stop'] = None
    return jsonify({'ok': True, 'interrupted': stop.is_set(),
                    'commanded': {'linear': clamped[0], 'angular': clamped[1], 'seconds': clamped[2]},
                    'limits': {'linear': DRIVE_MAX_LINEAR, 'angular': max_angular, 'seconds': DRIVE_MAX_SECONDS},
                    'pose_before': start, 'pose_after': node._last_odom})


@app.route('/api/headlights', methods=['POST'])
def api_headlights():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    body = _body()
    if 'on' not in body and 'level' not in body:
        return _error('Give on (true/false) and/or level (0..1).')
    if 'on' in body:
        node.set_headlights(bool(body['on']))
    if 'level' in body:
        try:
            node.set_light_level(min(max(_number(body, 'level'), 0.0), 1.0))
        except (TypeError, ValueError) as exc:
            return _error(str(exc))
    return jsonify({'ok': True})


@app.route('/api/avoidance', methods=['POST'])
def api_avoidance():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    if 'on' not in _body():
        return _error('Give on (true/false).')
    node.set_obstacle_avoidance(bool(_body()['on']))
    return jsonify({'ok': True, 'note': 'Applied asynchronously; /api/status obstacle_avoidance shows it.'})


@app.route('/api/viewport', methods=['POST'])
def api_viewport():
    mode = str(_body().get('mode', ''))
    if not VIEWPORT.request(mode):
        return _error(f'mode must be one of {list(MODES)}')
    web.emit_all('viewport_camera', VIEWPORT.status())
    return jsonify({'ok': True, 'viewport_camera': VIEWPORT.status()})


@app.route('/api/rover', methods=['GET', 'POST'])
def api_rover():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    if request.method == 'POST':
        rover = str(_body().get('rover', ''))
        known = [r['key'] for r in node.rover_status()['rovers']]
        if rover not in known:
            return _error(f'rover must be one of {known}')
        _stop_drive()
        node.select_rover(rover)
    return jsonify(node.rover_status())


@app.route('/api/restart_node', methods=['POST'])
def api_restart_node():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    name = str(_body().get('name', ''))
    if name not in LIFECYCLE_NODES:
        return _error(f'name must be one of {list(LIFECYCLE_NODES)}')
    node.restart_node(name)
    return jsonify({'ok': True, 'note': 'Runs in the background (~5-30 s); /api/status nodes shows the state.'})


@app.route('/api/record', methods=['POST'])
def api_record():
    node = _node()
    if node is None:
        return _error('Dashboard node not running.', 503)
    body = _body()
    action = body.get('action')
    if action == 'start':
        if body.get('preset') is not None and body['preset'] not in BAG_PRESETS:
            return _error(f'preset must be one of {list(BAG_PRESETS)}')
        error = node.start_bag(preset=body.get('preset'), label=str(body.get('label', '')),
                               notes=str(body.get('notes', '')))
        if error:
            return _error(error, 409)
    elif action == 'stop':
        node.stop_bag()
    else:
        return _error('action must be start or stop')
    return jsonify({'ok': True, 'recording': node._bag_status()})
