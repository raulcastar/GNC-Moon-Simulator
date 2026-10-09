"""Web side of the dashboard: one Flask app + SocketIO server per screen, the
helpers that fan events out to them, and the static files they share."""

import json
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, jsonify, request, send_from_directory
from flask_socketio import SocketIO


pkg_share = get_package_share_directory('mission_control_dashboard')
template_dir = os.path.join(pkg_share, 'templates')

# Independent web servers, one ROS node behind them all:
#   5000  hub      — a menu that links to the other screens
#   5001  overview — the full GNC mission-control dashboard
#   5002  teleop   — manual drive-by-wire, bypassing the GNC planner
#   5003  cameras  — a camera wall with per-feed show/hide toggles
#   5005  map3d    — LiDAR 3D + local DEM side by side, with labelled axes
#   5006  system   — feed rates, lifecycle nodes, recording and the ROS 2 topics
#   5007  bags     — record rosbags (presets, names, notes), replay and manage them
# network_mode: host means each port is reachable on the host directly, so no
# compose port mapping is needed. Each Flask app gets its own SocketIO server;
# the ROS node fans shared feeds (cameras, telemetry) out to whichever servers
# need them via emit_over() / emit_all() / emit_feeds().

app = Flask('overview', template_folder=template_dir)
app.config['SECRET_KEY'] = 'lunar_mission_secure'
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*",
                    max_http_buffer_size=10000000)

app_hub = Flask('hub', template_folder=template_dir)
app_hub.config['SECRET_KEY'] = 'lunar_hub'
sio_hub = SocketIO(app_hub, async_mode='threading', cors_allowed_origins="*")

app_teleop = Flask('teleop', template_folder=template_dir)
app_teleop.config['SECRET_KEY'] = 'lunar_teleop'
sio_teleop = SocketIO(app_teleop, async_mode='threading', cors_allowed_origins="*",
                      max_http_buffer_size=10000000)

app_cams = Flask('cameras', template_folder=template_dir)
app_cams.config['SECRET_KEY'] = 'lunar_cameras'
sio_cams = SocketIO(app_cams, async_mode='threading', cors_allowed_origins="*",
                    max_http_buffer_size=10000000)

app_map = Flask('map3d', template_folder=template_dir)
app_map.config['SECRET_KEY'] = 'lunar_map3d'
sio_map = SocketIO(app_map, async_mode='threading', cors_allowed_origins="*",
                   max_http_buffer_size=10000000)

app_sys = Flask('system', template_folder=template_dir)
app_sys.config['SECRET_KEY'] = 'lunar_system'
sio_sys = SocketIO(app_sys, async_mode='threading', cors_allowed_origins="*")

app_replay = Flask('replay', template_folder=template_dir)
app_replay.config['SECRET_KEY'] = 'lunar_replay'
sio_replay = SocketIO(app_replay, async_mode='threading', cors_allowed_origins="*")


def emit_over(event, data):
    """Emit only to the overview dashboard (5001)."""
    socketio.emit(event, data)


def emit_all(event, data):
    """Emit telemetry/log to the screens that show it (overview + teleop)."""
    socketio.emit(event, data)
    sio_teleop.emit(event, data)


def emit_map(event, data):
    """Fan a spatial feed (lidar / DEM / pose) out to the overview and the
    dedicated 3D map window (5005)."""
    socketio.emit(event, data)
    sio_map.emit(event, data)


def emit_feeds(event, data, teleop=True):
    """Fan a camera feed out to the screens that show it: overview and the
    camera wall always; teleop too for the driving-relevant views."""
    socketio.emit(event, data)
    sio_cams.emit(event, data)
    if teleop:
        sio_teleop.emit(event, data)


# The ROS node behind every server; set by main().
node = None


def _local_dem_js():
    """Serves the local DEM view shared by the overview and 3D map pages."""
    return send_from_directory(template_dir, 'local_dem.js', mimetype='application/javascript')


def _feed_legends_js():
    """Serves the depth / segmentation legends shared by the camera pages."""
    return send_from_directory(template_dir, 'feed_legends.js', mimetype='application/javascript')


def _dash_common_js():
    """Serves the helpers shared by the overview, teleop and system pages."""
    return send_from_directory(template_dir, 'dash_common.js', mimetype='application/javascript')


def _lidar_view_js():
    """Serves the lidar 3D view shared by the overview and 3D map pages."""
    return send_from_directory(template_dir, 'lidar_view.js', mimetype='application/javascript')


def _nav_overlay_js():
    """Serves the planned-path overlay drawn on the navcam feeds."""
    return send_from_directory(template_dir, 'nav_overlay.js', mimetype='application/javascript')


def _snapshot():
    if node is None:
        return Response('ROS node not ready', status=503)
    name = datetime.now().strftime('gnc_snapshot_%Y%m%d_%H%M%S.zip')
    return Response(node.snapshot_zip(), mimetype='application/zip',
                    headers={'Content-Disposition': f'attachment; filename={name}'})


for _app in (app, app_cams, app_teleop):
    _app.add_url_rule('/feed_legends.js', 'feed_legends_js', _feed_legends_js)
    _app.add_url_rule('/nav_overlay.js', 'nav_overlay_js', _nav_overlay_js)
app.add_url_rule('/snapshot.zip', 'snapshot', _snapshot)
for _app in (app, app_map):
    _app.add_url_rule('/local_dem.js', 'local_dem_js', _local_dem_js)
    _app.add_url_rule('/lidar_view.js', 'lidar_view_js', _lidar_view_js)
for _app in (app, app_teleop, app_sys, app_replay):
    _app.add_url_rule('/dash_common.js', 'dash_common_js', _dash_common_js)


# ---- colour theme, shared by every window (chosen in the hub) ----
THEMES = ('mission', 'midnight', 'night', 'daylight', 'slate', 'solarized', 'amber', 'contrast', 'paper')
FONTS = ('small', 'normal', 'large', 'xlarge')
# Kept next to the package sources (mounted from the repository), so it survives
# container rebuilds; git-ignored.
UI_SETTINGS = os.environ.get('GNC_UI_SETTINGS',
                             '/opt/ros/jazzy/src/mission_control_dashboard/ui_settings.json')


def _load_ui_settings() -> tuple:
    try:
        with open(UI_SETTINGS, encoding='utf-8') as f:
            saved = json.load(f)
    except (OSError, ValueError):
        saved = {}
    theme = saved.get('theme') if saved.get('theme') in THEMES else THEMES[0]
    font = saved.get('font') if saved.get('font') in FONTS else 'normal'
    return theme, font


theme, font = _load_ui_settings()


def _theme_js():
    """theme.js with the current theme filled in, so it applies before the page paints."""
    with open(os.path.join(template_dir, 'theme.js'), encoding='utf-8') as f:
        body = f.read().replace('__THEME__', theme).replace('__FONT__', font)
    return Response(body, mimetype='application/javascript', headers={'Cache-Control': 'no-store'})


def _theme_json():
    return Response(json.dumps({'theme': theme, 'themes': THEMES, 'font': font, 'fonts': FONTS}),
                    mimetype='application/json',
                    headers={'Cache-Control': 'no-store'})


def _set_theme():
    """POST {theme} and/or {font}: the hub's selectors. Saved, and picked up by every window."""
    global theme, font
    body = request.get_json(silent=True) or {}
    if 'theme' not in body and 'font' not in body:
        return jsonify(error='expected theme or font'), 400
    if 'theme' in body and body['theme'] not in THEMES:
        return jsonify(error=f"unknown theme {body['theme']!r}"), 400
    if 'font' in body and body['font'] not in FONTS:
        return jsonify(error=f"unknown font size {body['font']!r}"), 400
    theme, font = body.get('theme', theme), body.get('font', font)
    try:
        with open(UI_SETTINGS, 'w', encoding='utf-8') as f:
            json.dump({'theme': theme, 'font': font}, f)
    except OSError as exc:
        return jsonify(theme=theme, font=font, warning=f'not saved: {exc}')
    return jsonify(theme=theme, font=font)


for _app in (app, app_hub, app_teleop, app_cams, app_map, app_sys, app_replay):
    _app.add_url_rule('/theme.js', 'theme_js', _theme_js)
    _app.add_url_rule('/theme.json', 'theme_json', _theme_json)
app_hub.add_url_rule('/theme', 'set_theme', _set_theme, methods=['POST'])
