"""Page routes and SocketIO handlers of every dashboard screen. They act on the
ROS node in web.node (set by main())."""

import threading

from flask import render_template, request

from . import web
from .web import (app, app_cams, app_hub, app_map, app_replay, app_sys, app_teleop, sio_cams, sio_replay,
                  sio_sys, sio_teleop, socketio)


# =========================================================================
# Flask routing — hub (5000)
# =========================================================================

@app_hub.route('/')
def hub():
    return render_template('hub.html')


# =========================================================================
# Flask routing — teleop (5002)
# =========================================================================

@app_teleop.route('/')
def teleop():
    return render_template('teleop.html')


@sio_teleop.on('teleop_cmd')
def handle_teleop_cmd(json_data):
    if web.node is not None:
        web.node.teleop_drive(json_data.get('lin', 0.0), json_data.get('ang', 0.0))


@sio_teleop.on('headlights')
def handle_teleop_headlights(json_data):
    if web.node is not None:
        web.node.set_headlights(json_data.get('on', False))


@sio_teleop.on('stop_rover')
def handle_teleop_stop(json_data=None):
    if web.node is not None:
        web.node.stop_rover()


# =========================================================================
# Flask routing — cameras (5003)
# =========================================================================

@app_cams.route('/')
def cameras():
    return render_template('cameras.html')


# =========================================================================
# Flask routing — map3d (5005): LiDAR 3D + local DEM
# =========================================================================

@app_map.route('/')
def map3d():
    return render_template('map3d.html')


# =========================================================================
# Flask routing — system (5006): feeds, lifecycle nodes, ROS 2 topics
# =========================================================================

@app_sys.route('/')
def system():
    return render_template('system.html')


@sio_sys.on('restart_node')
def handle_restart_node(json_data):
    if web.node is not None:
        web.node.restart_node(str(json_data.get('name', '')))


@sio_sys.on('topics_list')
def handle_topics_list(json_data=None):
    if web.node is not None:
        web.node.list_topics(request.sid)


@sio_sys.on('topic_inspect')
def handle_topic_inspect(json_data):
    if web.node is not None:
        web.node.inspect_topic(request.sid, str(json_data.get('name', '')))


@sio_sys.on('topic_inspect_stop')
def handle_topic_inspect_stop(json_data=None):
    if web.node is not None:
        web.node.stop_inspect(request.sid)


@sio_sys.on('disconnect')
def handle_sys_disconnect(*args):
    if web.node is not None:
        web.node.stop_inspect(request.sid)


# =========================================================================
# Flask routing — replay (5007): recorded rosbags
# =========================================================================

@app_replay.route('/')
def replay():
    return render_template('replay.html')


@sio_replay.on('bags')
def handle_bags(json_data=None):
    if web.node is not None:
        web.node.list_bags(request.sid)
        sio_replay.emit('recorder', web.node.recorder_info(), to=request.sid)


def _bag_result(error, action):
    """Reports a recorder / management action to the BAGS window and refreshes the bag list."""
    sio_replay.emit('bag_result', {'action': action, 'error': error}, to=request.sid)
    web.node.list_bags()


@sio_replay.on('record_start')
def handle_record_start(json_data):
    if web.node is not None:
        err = web.node.start_bag(str(json_data.get('preset', 'navigation')),
                                 [str(t) for t in json_data.get('topics') or []],
                                 str(json_data.get('label', '')), str(json_data.get('notes', '')))
        sio_replay.emit('bag_result', {'action': 'record', 'error': err}, to=request.sid)


@sio_replay.on('record_stop')
def handle_record_stop(json_data=None):
    if web.node is not None:
        sid = request.sid

        def stop():
            web.node.stop_bag()
            web.node.list_bags()
            sio_replay.emit('bag_result', {'action': 'stopped', 'error': None}, to=sid)
        threading.Thread(target=stop, daemon=True).start()


@sio_replay.on('bag_rename')
def handle_bag_rename(json_data):
    if web.node is not None:
        _bag_result(web.node.rename_bag(json_data.get('bag', ''), json_data.get('name', '')), 'rename')


@sio_replay.on('bag_notes')
def handle_bag_notes(json_data):
    if web.node is not None:
        _bag_result(web.node.set_bag_notes(json_data.get('bag', ''), json_data.get('notes', '')), 'notes')


@sio_replay.on('bag_delete')
def handle_bag_delete(json_data):
    if web.node is not None:
        _bag_result(web.node.delete_bag(json_data.get('bag', '')), 'delete')


@sio_replay.on('replay_start')
def handle_replay_start(json_data):
    if web.node is not None:
        web.node.start_replay(str(json_data.get('bag', '')), float(json_data.get('rate', 1.0)),
                              bool(json_data.get('loop', False)), float(json_data.get('offset', 0.0)),
                              [str(t) for t in json_data.get('topics') or []])


@sio_replay.on('replay_stop')
def handle_replay_stop(json_data=None):
    if web.node is not None:
        threading.Thread(target=web.node.stop_replay, daemon=True).start()


@sio_replay.on('replay_pause')
def handle_replay_pause(json_data):
    if web.node is not None:
        web.node.pause_replay(bool(json_data.get('paused', True)))


@sio_replay.on('replay_rate')
def handle_replay_rate(json_data):
    if web.node is not None:
        web.node.set_replay_rate(float(json_data.get('rate', 1.0)))


@sio_replay.on('replay_seek')
def handle_replay_seek(json_data):
    if web.node is not None:
        web.node.seek_replay(float(json_data.get('fraction', 0.0)))


# =========================================================================
# Flask routing — overview (5001)
# =========================================================================

@app.route('/')
def index():
    return render_template('index.html')


@socketio.on('connect')
def handle_connect(auth=None):
    if web.node is not None:
        web.node.get_logger().info("Operator console connected.")
        web.node.fetch_maps(request.sid)
        _send_cached(socketio, request.sid)
        socketio.emit('timeline', list(web.node._timeline), to=request.sid)


@socketio.on('timeline_clear')
def handle_timeline_clear(json_data=None):
    if web.node is not None:
        web.node.clear_timeline()


@socketio.on('send_goal')
def handle_goal(json_data):
    if web.node is not None:
        web.node.dispatch_goal(json_data['x'], json_data['y'])


@socketio.on('refresh_map')
def handle_refresh(json_data=None):
    if web.node is not None:
        web.node.fetch_maps(request.sid)


@socketio.on('headlights')
def handle_headlights(json_data):
    if web.node is not None:
        web.node.set_headlights(json_data.get('on', False))


@socketio.on('obstacle_avoidance')
def handle_obstacle_avoidance(json_data):
    if web.node is not None:
        web.node.set_obstacle_avoidance(bool(json_data.get('on', True)))


@socketio.on('stop_rover')
def handle_stop(json_data=None):
    if web.node is not None:
        web.node.stop_rover()


@socketio.on('restart_cameras')
def handle_restart_cameras(json_data=None):
    if web.node is not None:
        web.node.restart_cameras()


@socketio.on('bag')
def handle_bag(json_data):
    if web.node is None:
        return
    if json_data.get('action') == 'start':
        web.node.start_bag()
    else:
        threading.Thread(target=web.node.stop_bag, daemon=True).start()


def _send_cached(server, sid):
    """State a newly connected browser needs before the next update arrives."""
    if web.node is None:
        return
    server.emit('config', web.node._remote, to=sid)
    if web.node._mask_legend:
        server.emit('mask_legend', web.node._mask_legend, to=sid)
    if web.node._mask_classes:
        server.emit('mask_classes', web.node._mask_classes, to=sid)
    server.emit('mission', web.node._mission, to=sid)
    server.emit('avoidance', {'on': web.node._avoidance}, to=sid)


@sio_cams.on('connect')
def handle_cams_connect(auth=None):
    _send_cached(sio_cams, request.sid)


@sio_teleop.on('connect')
def handle_teleop_connect(auth=None):
    _send_cached(sio_teleop, request.sid)
