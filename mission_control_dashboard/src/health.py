"""Feed rates, lifecycle node states and restarts, remote parameters."""

import threading
import time

from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

from .settings import (AVOIDANCE_PARAM, FEED_NAMES, FEED_STALE_S, HEALTH_WINDOW, NODE_POLL_TICKS,
                       PLANNER_NODE, REMOTE_PARAMS)
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

from .viewport_camera import VIEWPORT

AUTO_ACTIVATE_WAIT_S = 5.0      # s for a node's launch file to activate it after configure
from .web import emit_all, feed_watchers, sio_cams, sio_map, sio_replay, sio_sys, sio_teleop, socketio


class HealthMixin:
    """Feed rates, lifecycle node states and restarts, remote parameters. Mixed into LifecycleMissionControlUI."""

    def restart_cameras(self) -> None:
        self.restart_node('lc_component_stereo_depth')

    def restart_node(self, name: str) -> None:
        """Cycles a lifecycle node from a background thread."""
        if name not in self._cli_change_state:
            self._console('WARN', f'Unknown node {name}.')
            return
        if name in self._restart_busy:
            self._console('WARN', f'Restart of {name} already in progress.')
            return
        self._restart_busy.add(name)
        threading.Thread(target=self._restart_node_impl, args=(name,), daemon=True).start()

    def _console(self, level: str, text: str) -> None:
        """
        Log to ROS. The consoles show it through /rosout like every other node's
        message (sending it to them here as well printed it twice).
        """
        # One call site per severity: rclpy refuses a call site whose severity changes
        # between calls ("Logger severity cannot be changed between calls").
        logger = self.get_logger()
        if level == 'ERROR':
            logger.error(text)
        elif level == 'WARN':
            logger.warn(text)
        else:
            logger.info(text)

    def _restart_node_impl(self, name: str) -> bool:
        """
        deactivate -> cleanup -> configure -> activate on `name`; True once it is active again.

        Equivalent to restarting its container (for the stereo node, cleanup
        closes the frame-stream server and configure opens it again). The first two
        steps may legitimately fail if the node is already inactive, so only
        configure and activate are load-bearing.

        Every node's launch file activates it whenever it reaches inactive
        (OnStateTransition configuring -> inactive), so after configure this
        waits for that and sends activate only if it does not come: a second
        activate is refused inside the node, and the error raised there ends
        its executor (the node stops answering).
        """
        client = self._cli_change_state[name]
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                self._console('ERROR', f'{name}: lifecycle service unreachable — '
                                       f'is its container running?')
                return False
            self._console('INFO', f'Restarting {name}...')
            steps = [(4, 'deactivate', False), (2, 'cleanup', False),
                     (1, 'configure', True), (3, 'activate', True)]
            for tid, label, required in steps:
                if label == 'activate' and self._await_state(name, 'active', AUTO_ACTIVATE_WAIT_S):
                    self._console('INFO', f'{name}: activated (by its launch file).')
                    continue
                req = ChangeState.Request()
                req.transition.id = tid
                fut = client.call_async(req)
                t0 = time.monotonic()
                while not fut.done() and time.monotonic() - t0 < 90.0:
                    time.sleep(0.1)
                ok = fut.done() and fut.result() is not None and fut.result().success
                if ok:
                    self._console('INFO', f'{name}: {label} ok.')
                elif required:
                    self._console('ERROR', f'{name}: {label} FAILED — check its log; '
                                           f'is Unreal playing?')
                    return False
                else:
                    self._console('WARN', f'{name}: {label} skipped (already past '
                                          f'that state).')
            self._console('INFO', f'{name} restarted.')
            return True
        finally:
            self._restart_busy.discard(name)

    def _await_state(self, name: str, label: str, timeout: float) -> bool:
        """Polls the node's lifecycle state until it is `label` (True) or `timeout` passes."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._lifecycle_state(name) == label:
                return True
            time.sleep(0.2)
        return False

    def _lifecycle_state(self, name: str, timeout: float = 3.0):
        """The node's lifecycle state label (blocking; call from a worker thread), or None."""
        client = self._cli_get_state.get(name)
        if client is None or not client.service_is_ready():
            return None
        fut = client.call_async(GetState.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        try:
            return fut.result().current_state.label if fut.done() else None
        except Exception:
            return None

    def _mark(self, feed: str) -> None:
        self._arrivals[feed].append(time.monotonic())

    def _sync_viewer_subscriptions(self) -> None:
        """
        Subscribes to the raw image feeds while a window shows them or their
        rates, and drops the subscriptions when the last such window closes.
        """
        wanted = feed_watchers()
        if wanted and not self._viewer_subs:
            self._viewer_subs = [self.create_subscription(Image, topic, cb, qos_profile_sensor_data)
                                 for _feed, topic, cb in self._viewer_feeds]
            self.get_logger().info('Camera windows open: raw image feeds subscribed.')
        elif not wanted and self._viewer_subs:
            for s in self._viewer_subs:
                self.destroy_subscription(s)
            self._viewer_subs = []
            for feed, _topic, _cb in self._viewer_feeds:
                self._arrivals.pop(feed, None)          # not watched: no rate, no alert
            self.get_logger().info('No camera window open: raw image feeds unsubscribed.')

    def _health_cb(self) -> None:
        if not self._is_active():
            return
        self._sync_viewer_subscriptions()
        self._poll_avoidance()
        viewport = VIEWPORT.status()               # notices Unreal's link stopping
        if viewport != getattr(self, '_viewport_sent', None):
            self._viewport_sent = viewport
            emit_all('viewport_camera', viewport)
        self._emit_rover()                         # Unreal's rover, as its link reports it
        now = time.monotonic()
        feeds = {}
        for name, times in self._arrivals.items():
            recent = sum(1 for t in times if now - t <= HEALTH_WINDOW)
            feeds[name] = {'hz': round(recent / HEALTH_WINDOW, 1),
                           'age': round(now - times[-1], 1) if times else None}
        payload = {'feeds': feeds, 'nodes': dict(self._node_states), 'bag': self._bag_status(),
                   'alerts': self._alerts(feeds)}
        for server in (socketio, sio_cams, sio_teleop, sio_map, sio_sys, sio_replay):
            server.emit('health', payload)

        self._health_ticks += 1
        self._request_elevation()
        if self._health_ticks % NODE_POLL_TICKS == 1:
            self._poll_nodes()
        self._fetch_remote_params()

    def _alerts(self, feeds: dict) -> list:
        """
        Problems worth a banner in every window, as [{key, level, text}].

        Only something that was working counts: a feed that delivered data in the
        current Play session and then went quiet, or a lifecycle node that was
        active and left that state. A stopped simulation (or a replay) clears the
        feeds' history, so stopping Play or a feed that is switched off raises
        nothing.
        """
        alerts = []
        if not self._live_simulation():
            self._feeds_seen.clear()
        else:
            for name, f in sorted(feeds.items()):
                if f['age'] is not None and f['age'] <= FEED_STALE_S:
                    self._feeds_seen.add(name)
                elif name in self._feeds_seen and f['age'] is not None:
                    alerts.append({'key': f'feed:{name}', 'level': 'warn',
                                   'text': f"{FEED_NAMES.get(name, name)}: no data for {f['age']:.0f} s"})
        for name, state in sorted(self._node_states.items()):
            short = name.replace('lc_component_', '')
            if state == 'active':
                self._nodes_seen.add(name)
            elif name in self._nodes_seen and state != 'unknown':
                if name in self._restart_busy:
                    alerts.append({'key': f'node:{name}', 'level': 'info', 'text': f'{short}: restarting'})
                else:
                    alerts.append({'key': f'node:{name}', 'level': 'error', 'text': f'{short} node is {state}'})
        return alerts

    def _poll_nodes(self) -> None:
        """Queries each node's lifecycle state; replies arrive asynchronously."""
        for name, client in self._cli_get_state.items():
            if not client.service_is_ready():
                self._node_states[name] = 'offline'
                continue
            key = ('state', name)
            if key in self._pending:
                continue
            self._pending.add(key)
            fut = client.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=name, k=key: self._on_node_state(n, k, f))

    def _on_node_state(self, name: str, key, fut) -> None:
        self._pending.discard(key)
        try:
            self._node_states[name] = fut.result().current_state.label
        except Exception as exc:
            self._node_states[name] = 'error'
            self.get_logger().warn(f'Lifecycle state of {name} unreadable: {exc}',
                                   throttle_duration_sec=30.0)

    def _poll_avoidance(self) -> None:
        """Reads the planner's lidar_obstacle_avoidance, so the switch shows changes made elsewhere."""
        client = self._cli_params.get(PLANNER_NODE)
        if client is None or not client.service_is_ready() or ('param', AVOIDANCE_PARAM) in self._pending:
            return
        self._pending.add(('param', AVOIDANCE_PARAM))
        fut = client.call_async(GetParameters.Request(names=[AVOIDANCE_PARAM]))
        fut.add_done_callback(self._on_avoidance)

    def _on_avoidance(self, fut) -> None:
        self._pending.discard(('param', AVOIDANCE_PARAM))
        try:
            value = fut.result().values[0]
            on = value.bool_value if value.type == ParameterType.PARAMETER_BOOL else None
        except Exception:
            on = None
        if on != self._avoidance:
            self._avoidance = on
            emit_all('avoidance', {'on': on})

    def _fetch_remote_params(self) -> None:
        """Reads the planner's tilt limit and the depth colour range once available."""
        for key, (node, param) in REMOTE_PARAMS.items():
            client = self._cli_params.get(node)
            if self._remote[key] is not None or client is None or not client.service_is_ready():
                continue
            pkey = ('param', key)
            if pkey in self._pending:
                continue
            self._pending.add(pkey)
            fut = client.call_async(GetParameters.Request(names=[param]))
            fut.add_done_callback(lambda f, k=key, pk=pkey: self._on_remote_param(k, pk, f))

    def _on_remote_param(self, key: str, pkey, fut) -> None:
        self._pending.discard(pkey)
        try:
            v = fut.result().values[0]
            value = v.double_value if v.type == 3 else float(v.integer_value) if v.type == 2 else None
        except Exception as exc:     # asked again on the next health tick
            value = None
            self.get_logger().warn(f'Parameter {key} unreadable: {exc}', throttle_duration_sec=30.0)
        if value is not None:
            self._remote[key] = value
            for server in (socketio, sio_cams, sio_teleop, sio_map):
                server.emit('config', self._remote)
