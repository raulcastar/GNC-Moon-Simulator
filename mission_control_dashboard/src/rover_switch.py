"""The menu's ROVER selector: switches the stack to another rover, live (see rovers.py)."""

import os
import threading
import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter as NodeParameter

from . import rovers
from .settings import REMOTE_PARAMS
from .viewport_camera import VIEWPORT
from .web import emit_all

RSP_NODE = 'robot_state_publisher'
# Re-configured after the switch: they read their sensor mounts from TF and
# build their limits when configured.
RESTART_AFTER = ('lc_component_path_planner', 'lc_component_local_mapping')
TF_SETTLE_S = 2.0           # robot_state_publisher republishing /tf_static
CALL_TIMEOUT_S = 5.0


def _value(v) -> ParameterValue:
    if isinstance(v, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    if isinstance(v, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    if isinstance(v, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(v))


class RoverSwitchMixin:
    """The active rover and the switch to another. Mixed into LifecycleMissionControlUI."""

    def _init_rover(self) -> None:
        rover = os.environ.get('GNC_ROVER', rovers.DEFAULT_ROVER)
        self._rover = rover if rover in rovers.ROVERS else rovers.DEFAULT_ROVER
        self._rover_busy = False
        self._rover_sent = None
        self._cli_set = {}

    def _configure_rover(self) -> None:
        nodes = [n for n in rovers.PACKAGES.values() if n != self.get_name()] + [RSP_NODE]
        self._cli_set = {n: self.create_client(SetParameters, f'/{n}/set_parameters') for n in nodes}

    def rover_status(self) -> dict:
        viewport = VIEWPORT.status()
        return rovers.status(self._rover, viewport.get('rover'), viewport['linked'], self._rover_busy)

    def _emit_rover(self, force: bool = False) -> None:
        status = self.rover_status()
        if force or status != self._rover_sent:
            self._rover_sent = status
            emit_all('rover', status)

    def select_rover(self, rover: str) -> None:
        """Switches the stack to `rover` from a background thread."""
        if rover not in rovers.ROVERS:
            self._console('WARN', f'Unknown rover {rover}.')
            return
        if self._rover_busy:
            self._console('WARN', 'A rover switch is already in progress.')
            return
        self._rover_busy = True
        self.stop_rover()                   # the planner is re-configured: no goal, no motion
        self._emit_rover()
        threading.Thread(target=self._select_rover_impl, args=(rover,), daemon=True).start()

    def _set_remote(self, node: str, values: dict) -> bool:
        """SetParameters on another node; logs and returns whether all were accepted."""
        client = self._cli_set.get(node)
        if client is None or not client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
            self._console('WARN', f'{node}: not running, its parameters were not changed.')
            return False
        request = SetParameters.Request(parameters=[Parameter(name=k, value=_value(v)) for k, v in values.items()])
        fut = client.call_async(request)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < CALL_TIMEOUT_S:
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            self._console('WARN', f'{node}: no answer to the parameter change.')
            return False
        refused = [f'{k} ({r.reason or "refused"})' for k, r in zip(values, fut.result().results) if not r.successful]
        if refused:
            self._console('WARN', f'{node}: refused {", ".join(refused)}.')
        return not refused

    def _select_rover_impl(self, rover: str) -> None:
        label = rovers.ROVERS[rover]['label']
        try:
            self._console('INFO', f'Switching the stack to {label}...')
            profile = rovers.profile(rover)
            ok = True
            for node, values in profile.items():
                if node == self.get_name():
                    self.set_parameters([NodeParameter(k, value=v) for k, v in values.items()])
                    for key in ('rover_width_m', 'teleop_max_angular'):
                        self._remote[key] = self.get_parameter(key).value
                else:
                    ok = self._set_remote(node, values) and ok
            description = rovers.urdf(rover)
            if description is None:
                self._console('WARN', f'No URDF for {label}: the frames on /tf_static are unchanged.')
                ok = False
            else:
                ok = self._set_remote(RSP_NODE, {'robot_description': description}) and ok
            self._rover = rover
            time.sleep(TF_SETTLE_S)
            for node in RESTART_AFTER:
                if node in self._restart_busy:
                    continue
                self._restart_busy.add(node)
                ok = self._restart_node_impl(node) and ok
            # The mapper's camera mount (from TF) and the planner's values, read again.
            for key, (node, _param) in REMOTE_PARAMS.items():
                if node in RESTART_AFTER:
                    self._remote[key] = None
            emit_all('config', self._remote)
            self._console('INFO' if ok else 'WARN',
                          f'Stack switched to {label}{"" if ok else " (with warnings, see above)"}. '
                          f'Unreal must play {rovers.ROVERS[rover]["level"]}; '
                          f'GNC_ROVER in .env decides at the next start.')
        except Exception as exc:
            self._console('ERROR', f'Rover switch to {label} failed: {exc}')
        finally:
            self._rover_busy = False
            self._emit_rover(force=True)
