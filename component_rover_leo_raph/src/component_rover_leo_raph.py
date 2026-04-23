"""
ROS 2 Lifecycle Node for Unreal Engine Rover Control and Odometry.

This module provides a bi-directional bridge between standard ROS 2 navigation stacks 
and the Unreal Engine simulator. It subscribes to geometry_msgs/Twist messages to 
dispatch low-latency UDP motor commands, and utilizes a dedicated background thread 
to ingest UDP telemetry, translating Unreal Engine's coordinate system into standard 
ROS 2 nav_msgs/Odometry (Meters, Right-Handed, Quaternions).
"""

from typing import Optional
import json
import socket
import threading
import math
import re

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription

# Standard ROS 2 messages for kinematics and state estimation
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import warnings
warnings.filterwarnings("ignore")


class LifecycleRoverController(rclpy.lifecycle.Node):
    """
    Lifecycle node handling asynchronous UDP communication with Unreal Engine.

    Manages a non-blocking transmission socket for motor commands and a dedicated 
    background thread for receiving high-frequency telemetry. Strictly enforces 
    ROS 2 REP-103 standards for units and coordinate frames.
    """

    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the LifecycleRoverController node and declare configurable parameters.

        Args:
            node_name (str): The designated name of the ROS 2 node.
        """
        super().__init__(node_name, **kwargs)
        
        # ROS 2 Interfaces
        self._sub_cmd_vel: Optional[Subscription] = None
        self._sub_headlights: Optional[Subscription] = None 
        self._pub_odom:    Optional[Publisher] = None
        
        # Rover State
        self.headlights_on: bool = False
        
        # Network Utilities: Transmission
        self.udp_send_socket: Optional[socket.socket] = None
        
        # Network Utilities: Reception & Threading
        self.udp_recv_socket: Optional[socket.socket] = None
        self._listen_thread:  Optional[threading.Thread] = None
        self._keep_listening: bool = False
        
        # Runtime configuration defaults
        self.target_ip: str = "127.0.0.1"
        self.target_port: int = 3002
        self.listen_ip: str = "0.0.0.0"
        self.listen_port: int = 3001
        self.rover_width: float = 1.0

        # Declare ROS 2 Parameters for dynamic configuration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sending_ip',   rclpy.Parameter.Type.STRING),
                ('sending_port', rclpy.Parameter.Type.INTEGER),
                ('listen_ip',    rclpy.Parameter.Type.STRING),
                ('listen_port',  rclpy.Parameter.Type.INTEGER),
                ('rover_width',  rclpy.Parameter.Type.DOUBLE),
                ('headlights_on', rclpy.Parameter.Type.BOOL),
            ])

    # =========================================================================
    # Kinematics & Coordinate Transformation Helpers
    # =========================================================================

    def euler_to_quaternion(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
        """
        Translates Euler angles (degrees) into a ROS 2 compliant Quaternion.
        """
        # Convert degrees to radians to satisfy standard trigonometric functions
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        # Construct quaternion using standard aerospace sequence (Z-Y-X)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def twist_to_wheels(self, v: float, w: float) -> dict:
        """
        Computes Inverse Kinematics for a 4-wheel skid-steer/differential vehicle.
        """
        # Compute individual side velocities based on chassis track width
        vel_left  = v - (w * self.rover_width / 2.0)
        vel_right = v + (w * self.rover_width / 2.0)
        
        # Apply mechanical inversion (negative input = forward motion in UE blueprint)
        return {
            "FL": -vel_left,
            "BL": -vel_left,
            "FR": -vel_right,
            "BR": -vel_right,
            "headlights_on": self.headlights_on
        }

    # =========================================================================
    # Core Callbacks & Background Threads
    # =========================================================================

    def headlights_callback(self, msg: Bool) -> None:
        """
        Updates the internal state of the headlights based on incoming ROS 2 commands.
        """
        self.headlights_on = msg.data
        headlights_commands = {"headlights_on": self.headlights_on}
        self.udp_send_socket.sendto(json.dumps(headlights_commands).encode('utf-8'), (self.target_ip, self.target_port))
        self.get_logger().info(f"Headlights toggled: {'ON' if self.headlights_on else 'OFF'}")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Processes incoming linear/angular velocity commands and dispatches them via UDP.
        """
        try:
            wheel_commands = self.twist_to_wheels(msg.linear.x, msg.angular.z)
            json_string = json.dumps(wheel_commands)
            self.udp_send_socket.sendto(json_string.encode('utf-8'), (self.target_ip, self.target_port))
        except Exception as e:
            self.get_logger().error(f"Failed to transmit cmd_vel UDP packet: {e}")

    def _udp_receive_loop(self) -> None:
        """
        Background thread loop for handling incoming UDP telemetry asynchronously.
        """
        self.get_logger().info(f"Telemetry listener thread initialized on port {self.listen_port}.")
        
        while self._keep_listening:
            try:
                # 0.5s timeout prevents blocking indefinitely, allowing the thread 
                # to regularly check the `_keep_listening` termination flag.
                self.udp_recv_socket.settimeout(0.5)
                data, _ = self.udp_recv_socket.recvfrom(1024)
                                    
                # Decode and strip whitespace/null characters
                message = data.decode('utf-8').strip().rstrip('\x00')
                
                # This regex looks for a digit, a comma, and another digit, and removes the comma.
                message = re.sub(r'(\d),(\d)', r'\1\2', message)
                transform = json.loads(message)

                # Initialize Odometry payload
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"

                # ---------------------------------------------------------
                # Position Translation: Unreal Engine (cm) -> ROS 2 (m)
                odom_msg.pose.pose.position.x = -transform['x'] / 100.0
                odom_msg.pose.pose.position.y = transform['y'] / 100.0
                odom_msg.pose.pose.position.z = transform['z'] / 100.0

                q = self.euler_to_quaternion(transform['roll'], transform['pitch'], transform['yaw'])
                odom_msg.pose.pose.orientation = q

                self._pub_odom.publish(odom_msg)

            except socket.timeout:
                # Expected timeout; loop repeats to evaluate `_keep_listening`
                continue
            except json.JSONDecodeError:
                self.get_logger().warn(f"Received non-JSON data: {repr(message)}")
            except Exception as e:
                if self._keep_listening:
                    self.get_logger().error(f"Telemetry thread fault: {e}")
                    
        self.get_logger().info("Telemetry listener thread cleanly terminated.")

    # =========================================================================
    # Lifecycle Transition Callbacks
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node configuration.
        """
        self.get_logger().info("Lifecycle state transition: CONFIGURED.")

        self.target_ip   = self.get_parameter('sending_ip').value
        self.target_port = self.get_parameter('sending_port').value
        self.listen_ip   = self.get_parameter('listen_ip').value
        self.listen_port = self.get_parameter('listen_port').value
        self.rover_width = self.get_parameter('rover_width').value
        self.headlights_on = self.get_parameter('headlights_on').value
        
        # Initialize Transmission Socket (Unbound UDP Datagram)
        try:
            self.udp_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as e:
            self.get_logger().error(f"Failed to allocate UDP transmission socket: {e}")
            return TransitionCallbackReturn.FAILURE

        # Initialize Reception Socket (Bound to targeted listening port)
        try:
            self.udp_recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_recv_socket.bind((self.listen_ip, self.listen_port))
        except socket.error as e:
            self.get_logger().error(f"Failed to bind UDP reception socket on port {self.listen_port}: {e}")
            return TransitionCallbackReturn.FAILURE
        
        # Send initial commands
        if self.udp_send_socket:
            stop_commands = {"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0, "headlights_on": self.headlights_on}
            self.udp_send_socket.sendto(json.dumps(stop_commands).encode('utf-8'), (self.target_ip, self.target_port))

        # Allocate standard pub/sub endpoints
        self._pub_odom    = self.create_lifecycle_publisher(Odometry, 'odom', 10)
        self._sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Allocate headlights subscription
        self._sub_headlights = self.create_subscription(Bool, 'headlights_cmd', self.headlights_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node activation.
        """
        self.get_logger().info("Lifecycle state transition: ACTIVE.")
        
        # Ignite the background listening thread natively
        self._keep_listening = True
        self._listen_thread = threading.Thread(target=self._udp_receive_loop, daemon=True)
        self._listen_thread.start()
        
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node deactivation.
        """
        self.get_logger().info("Lifecycle state transition: DEACTIVATED.")
        
        # 1. Dispatch absolute zero velocities to enforce a hardware/simulation halt
        if self.udp_send_socket:
            stop_commands = {"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0, "headlights_on": False} # Turn off lights on stop
            self.udp_send_socket.sendto(json.dumps(stop_commands).encode('utf-8'), (self.target_ip, self.target_port))

        # 2. Assert thread termination and await its graceful exit (max 1 second block)
        self._keep_listening = False
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=1.0)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node cleanup.
        """
        self.get_logger().info('Lifecycle state transition: CLEANING UP.')
        
        if self._sub_cmd_vel:
            self.destroy_subscription(self._sub_cmd_vel)
            
        if self._sub_headlights:
            self.destroy_subscription(self._sub_headlights)
            
        if self._pub_odom:
            self.destroy_publisher(self._pub_odom)
            
        if self.udp_send_socket:
            self.udp_send_socket.close()
            self.udp_send_socket = None
            
        if self.udp_recv_socket:
            self.udp_recv_socket.close()
            self.udp_recv_socket = None
            
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node shutdown.
        """
        self.get_logger().info('Lifecycle state transition: SHUTTING DOWN.')
        
        # Terminate active threads if bypassing the 'deactivate' step
        self._keep_listening = False
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=1.0)
            
        if self._sub_cmd_vel:
            self.destroy_subscription(self._sub_cmd_vel)
            
        if self._sub_headlights:
            self.destroy_subscription(self._sub_headlights)
            
        if self._pub_odom:
            self.destroy_publisher(self._pub_odom)
            
        if self.udp_send_socket:
            self.udp_send_socket.close()
        if self.udp_recv_socket:
            self.udp_recv_socket.close()
            
        return TransitionCallbackReturn.SUCCESS


def main():
    """
    Entry point for the executable. 
    """
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node  = LifecycleRoverController('lc_component_rover_leo_raph')
    executor.add_node(lc_node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

if __name__ == '__main__':
    main()