"""
ROS 2 Lifecycle Node for Unreal Engine Rover Control.

This module acts as the bridge between standard ROS 2 navigation stacks 
and the Unreal Engine simulator. It subscribes to standard Twist messages, 
performs inverse kinematics to calculate individual wheel velocities, 
and streams the commands to the simulator via low-latency UDP JSON packets.
"""

from typing import Optional
import json
import socket

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.subscription import Subscription

# Standard ROS velocity command message
from geometry_msgs.msg import Twist

import warnings
warnings.filterwarnings("ignore")


class LifecycleRoverController(rclpy.lifecycle.Node):
    """
    Lifecycle node responsible for converting Twist commands to UDP wheel commands.

    Subscribes to `/cmd_vel`, computes differential drive inverse kinematics, 
    and dispatches UDP packets to the Unreal Engine host. The commands are strictly 
    gated by the node's lifecycle state to prevent runaway robots during startup/shutdown.
    """

    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the LifecycleRoverController node and declare ROS 2 parameters.

        Args:
            node_name (str): The designated name of the ROS 2 node.
        """
        super().__init__(node_name, **kwargs)
        
        # ROS 2 Interfaces
        self._sub_cmd_vel: Optional[Subscription] = None
        
        # Network Utilities
        self.udp_socket: Optional[socket.socket] = None
        
        # Runtime configuration variables
        self.target_ip: str = "127.0.0.1"
        self.target_port: int = 3002
        self.track_width: float = 1.0

        # Declare ROS 2 Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip',          rclpy.Parameter.Type.STRING),
                ('port',        rclpy.Parameter.Type.INTEGER),
                ('track_width', rclpy.Parameter.Type.DOUBLE),
            ])

    def twist_to_wheels(self, v: float, w: float) -> dict:
        """
        Calculates Inverse Kinematics for a 4-wheel skid-steer/differential rover.

        Converts desired linear and angular velocity into individual wheel speed 
        commands. Note: In this specific simulator configuration, negative motor 
        commands correspond to positive forward movement.

        Args:
            v (float): Desired linear velocity in meters per second.
            w (float): Desired angular velocity in radians per second.

        Returns:
            dict: JSON-serializable dictionary of the 4 motor commands.
        """
        # Calculate actual physical velocity needed for each side
        vel_left  = v - (w * self.track_width / 2.0)
        vel_right = v + (w * self.track_width / 2.0)
        
        # Invert the physical velocity to get the motor command
        cmd_left = -vel_left
        cmd_right = -vel_right
        
        return {
            "FL": cmd_left,
            "BL": cmd_left,
            "FR": cmd_right,
            "BR": cmd_right
        }

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Subscription callback for incoming velocity commands.

        Executes the inverse kinematics and transmits the UDP payload.
        Execution is halted if the node is not in the ACTIVE lifecycle state.
        
        Args:
            msg (Twist): The incoming linear and angular velocity command.
        """
        if not self.udp_socket:
            return

        try:
            # Extract standard linear (x) and angular (z) velocities
            linear_x = msg.linear.x
            angular_z = msg.angular.z

            # Process inverse kinematics
            wheel_commands = self.twist_to_wheels(linear_x, angular_z)

            # Serialize and transmit via UDP
            json_string = json.dumps(wheel_commands)
            self.udp_socket.sendto(json_string.encode('utf-8'), (self.target_ip, self.target_port))

        except Exception as e:
            self.get_logger().error(f"Failed to process and send cmd_vel packet: {e}")

    # =========================================================================
    # Lifecycle Transition Callbacks
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node configuration.
        Initializes parameters, the UDP socket, and the ROS 2 subscription.
        """
        self.get_logger().info("Lifecycle state transition: CONFIGURED.")

        # Read Parameters
        self.target_ip   = self.get_parameter('ip').value
        self.target_port = self.get_parameter('port').value
        self.track_width = self.get_parameter('track_width').value

        # Initialize UDP Datagram Socket
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.get_logger().info(f"UDP Socket initialized. Target: {self.target_ip}:{self.target_port}")
        except socket.error as e:
            self.get_logger().error(f"Failed to create UDP socket: {e}")
            return TransitionCallbackReturn.FAILURE

        # Initialize Subscription
        self._sub_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # QoS profile depth
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Transition callback for node activation."""
        self.get_logger().info("Lifecycle state transition: ACTIVE. Listening to /cmd_vel...")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node deactivation.
        Safely halts the rover by sending a zero-velocity command before pausing.
        """
        self.get_logger().info("Lifecycle state transition: DEACTIVATED. Halting rover...")
        
        # Transmit zero velocities to stop the rover safely
        if self.udp_socket:
            stop_commands = {"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0}
            json_string = json.dumps(stop_commands)
            self.udp_socket.sendto(json_string.encode('utf-8'), (self.target_ip, self.target_port))

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node cleanup.
        Destroys subscriptions and closes the UDP socket.
        """
        self.get_logger().info('Lifecycle state transition: CLEANING UP.')
        
        if self._sub_cmd_vel:
            self.destroy_subscription(self._sub_cmd_vel)
            
        if self.udp_socket:
            self.udp_socket.close()
            self.udp_socket = None
            
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node shutdown.
        Ensures graceful network termination.
        """
        self.get_logger().info('Lifecycle state transition: SHUTTING DOWN.')
        
        if self._sub_cmd_vel:
            self.destroy_subscription(self._sub_cmd_vel)
            
        if self.udp_socket:
            # Attempt final halt before closing
            stop_commands = {"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0}
            try:
                self.udp_socket.sendto(json.dumps(stop_commands).encode('utf-8'), (self.target_ip, self.target_port))
            except:
                pass
            self.udp_socket.close()
            self.udp_socket = None
            
        return TransitionCallbackReturn.SUCCESS


def main():
    """
    Entry point for the executable. Initializes the ROS 2 context, 
    allocates the executor, and spins the node.
    """
    rclpy.init()

    # A SingleThreadedExecutor is sufficient here since UDP sends are non-blocking
    # and we aren't handling heavy image arrays.
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node  = LifecycleRoverController('lc_component_rover_leo_raph')
    executor.add_node(lc_node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

if __name__ == '__main__':
    main()