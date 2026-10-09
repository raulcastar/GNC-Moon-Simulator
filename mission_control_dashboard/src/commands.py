"""Operator commands: headlights, stop, teleop drive (with its deadman), goals."""

import time

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Empty


class CommandsMixin:
    """Operator commands: headlights, stop, teleop drive (with its deadman), goals. Mixed into LifecycleMissionControlUI."""

    def set_headlights(self, on: bool) -> None:
        if self._pub_lights and self._pub_lights.is_activated:
            self._pub_lights.publish(Bool(data=bool(on)))
            self.get_logger().info(
                f"Headlights commanded {'ON' if on else 'OFF'} from the console.")

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
