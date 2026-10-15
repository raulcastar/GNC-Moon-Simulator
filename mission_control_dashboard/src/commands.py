"""Operator commands: headlights, stop, teleop drive (with its deadman), goals, obstacle avoidance."""

import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from .settings import AVOIDANCE_PARAM, PLANNER_NODE
from .web import emit_all

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Empty, Float32


class CommandsMixin:
    """Operator commands: headlights, stop, teleop drive (with its deadman), goals. Mixed into LifecycleMissionControlUI."""

    def set_obstacle_avoidance(self, on: bool) -> None:
        """Switches the planner's lidar obstacle avoidance (its parameter, live)."""
        client = self._cli_planner_set
        if client is None or not client.service_is_ready():
            self._console('WARN', f'Obstacle avoidance not changed: {PLANNER_NODE} is not running.')
            emit_all('avoidance', {'on': self._avoidance})
            return
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(on))
        request = SetParameters.Request(parameters=[Parameter(name=AVOIDANCE_PARAM, value=value)])
        client.call_async(request).add_done_callback(lambda f: self._avoidance_set(bool(on), f))

    def _avoidance_set(self, on: bool, fut) -> None:
        try:
            result = fut.result().results[0]
            ok, reason = result.successful, result.reason
        except Exception as exc:
            ok, reason = False, str(exc)
        if ok:
            self._avoidance = on
            self._console('INFO', f"Lidar obstacle avoidance {'ON' if on else 'OFF'} (from the console).")
        else:
            self._console('WARN', f'Obstacle avoidance not changed: {reason or "refused"}.')
        emit_all('avoidance', {'on': self._avoidance})

    def set_headlights(self, on: bool) -> None:
        if self._pub_lights and self._pub_lights.is_activated:
            self._pub_lights.publish(Bool(data=bool(on)))
            self.get_logger().info(
                f"Headlights commanded {'ON' if on else 'OFF'} from the console.")

    def set_light_level(self, level) -> None:
        """Headlight brightness (0..1) on headlights_intensity; the rover node clamps it too."""
        try:
            level = min(max(float(level), 0.0), 1.0)
        except (TypeError, ValueError):
            return
        if level != level:                  # NaN
            return
        if self._pub_light_level and self._pub_light_level.is_activated:
            self._pub_light_level.publish(Float32(data=level))

    def light_level_callback(self, msg: Float32) -> None:
        """The brightness on the topic (ours or anyone's): every window's slider follows it."""
        self._light_level = float(msg.data)
        emit_all('light_level', {'level': self._light_level})

    def stop_rover(self) -> None:
        """
        Cancels the planner's goal, which would otherwise override the stop on its
        next cycle, then sends a zero twist three times (the rover holds the last
        command).
        """
        self._teleop_engaged = False
        if self._pub_cancel and self._pub_cancel.is_activated:
            self._pub_cancel.publish(Empty())
        if self._pub_cmdvel and self._pub_cmdvel.is_activated:
            for _ in range(3):
                self._pub_cmdvel.publish(Twist())
            self.get_logger().warn("ALL STOP commanded from the console.")

    def teleop_drive(self, lin: float, ang: float) -> None:
        """
        Publish a manual cmd_vel from the teleop screen (5002).

        This shares the cmd_vel bus with the GNC planner, so engaging cancels
        an active goal. Engaging also arms the watchdog so a dropped connection
        cannot leave the rover running.
        """
        if not (self._pub_cmdvel and self._pub_cmdvel.is_activated):
            return
        lin = float(lin)
        ang = float(ang)
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        moving = lin != 0.0 or ang != 0.0
        if (moving and not self._teleop_engaged
                and self._mission.get('state') in ('planning', 'driving')
                and self._pub_cancel and self._pub_cancel.is_activated):
            # Manual driving takes over: cancel the GNC goal so the planner
            # does not fight the operator on the shared cmd_vel bus.
            self._pub_cancel.publish(Empty())
            self._console('WARN', 'Manual drive took over: GNC goal cancelled.')
        self._pub_cmdvel.publish(t)
        self._last_teleop = time.monotonic()
        self._teleop_engaged = self._teleop_engaged or moving

    def _teleop_watchdog_cb(self) -> None:
        """Zero the rover if a teleop session goes quiet (dropped tab, stuck key)."""
        if not self._teleop_engaged:
            return
        if time.monotonic() - self._last_teleop <= 0.6:
            return
        self._teleop_engaged = False
        if self._pub_cmdvel and self._pub_cmdvel.is_activated:
            self._pub_cmdvel.publish(Twist())
            self.get_logger().warn("Teleop watchdog: manual commands stopped — "
                                   "rover halted.")

    def dispatch_goal(self, x: float, y: float) -> None:
        if self._pub_goal and self._pub_goal.is_activated:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = float(x)
            goal.pose.position.y = float(y)
            self._pub_goal.publish(goal)
            self.get_logger().info(
                f"Goal dispatched: ({x:.1f}, {y:.1f}) m.")
        else:
            self.get_logger().warn("Goal rejected: node not ACTIVE.")
