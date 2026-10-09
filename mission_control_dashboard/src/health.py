"""Feed rates, lifecycle node states and restarts, remote parameters."""

import threading
import time

from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import GetParameters

from .settings import HEALTH_WINDOW, NODE_POLL_TICKS, REMOTE_PARAMS
from .web import sio_cams, sio_map, sio_replay, sio_sys, sio_teleop, socketio


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

    def _restart_node_impl(self, name: str) -> None:
        """
        deactivate -> cleanup -> configure -> activate on `name`.

        Equivalent to restarting its container (for the stereo node, cleanup
        releases the UnrealCV socket and configure reconnects). The first two
        steps may legitimately fail if the node is already inactive, so only
        configure and activate are load-bearing.
        """
        client = self._cli_change_state[name]
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                self._console('ERROR', f'{name}: lifecycle service unreachable — '
                                       f'is its container running?')
                return
            self._console('INFO', f'Restarting {name}...')
            steps = [(4, 'deactivate', False), (2, 'cleanup', False),
                     (1, 'configure', True), (3, 'activate', True)]
            for tid, label, required in steps:
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
                    return
                else:
                    self._console('WARN', f'{name}: {label} skipped (already past '
                                          f'that state).')
            self._console('INFO', f'{name} restarted.')
        finally:
            self._restart_busy.discard(name)

    def _mark(self, feed: str) -> None:
        self._arrivals[feed].append(time.monotonic())

    def _health_cb(self) -> None:
        if not self._is_active():
            return
        now = time.monotonic()
        feeds = {}
        for name, times in self._arrivals.items():
            recent = sum(1 for t in times if now - t <= HEALTH_WINDOW)
            feeds[name] = {'hz': round(recent / HEALTH_WINDOW, 1),
                           'age': round(now - times[-1], 1) if times else None}
        payload = {'feeds': feeds, 'nodes': dict(self._node_states), 'bag': self._bag_status()}
        for server in (socketio, sio_cams, sio_teleop, sio_map, sio_sys, sio_replay):
            server.emit('health', payload)

        self._health_ticks += 1
        self._request_elevation()
        if self._health_ticks % NODE_POLL_TICKS == 1:
            self._poll_nodes()
        self._fetch_remote_params()

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
        except Exception:
            self._node_states[name] = 'error'

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
        except Exception:
            value = None
        if value is not None:
            self._remote[key] = value
            for server in (socketio, sio_cams, sio_teleop, sio_map):
                server.emit('config', self._remote)
