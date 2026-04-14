"""
ROS 2 Lifecycle Node for Global Path Planning and DWA Local Trajectory Control.

This module provides a ROS 2 wrapper around the standalone global path planner and 
local trajectory controller. It loads map geometries via parameters, computes the 
global path on configuration, and utilizes a timer-based control loop to dispatch 
local velocity commands (Twist) based on incoming Odometry.
"""
import os
from ament_index_python.packages import get_package_share_directory

from typing import Optional
import math
import numpy as np

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Custom Planner Imports
from src import global_path_planner
from src import local_trajectory_controller
from src import robot_config

import warnings
warnings.filterwarnings("ignore")


class LifecyclePlannerController(rclpy.lifecycle.Node):
    """
    Lifecycle node handling Path Planning and Local Control execution.

    Manages map loading, costmap generation, and execution of the DWA local planner.
    Strictly interfaces via standard ROS 2 geometry_msgs/Twist and nav_msgs/Odometry.
    """

    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the LifecyclePlannerController node and declare configurable parameters.
        """
        super().__init__(node_name, **kwargs)
        
        # ROS 2 Interfaces
        self._pub_cmd_vel: Optional[Publisher] = None
        self._sub_odom:    Optional[Subscription] = None
        self._control_timer: Optional[Timer] = None
        
        # Planner Instances
        self.path_planner  = global_path_planner.Global_Path_Planner()
        self.local_planner = local_trajectory_controller.Local_Trajectory_Controller()
        self.robot_conf    = robot_config.Robot()
        self.robot_conf._robot_type = 1 # rectangle

        # Internal State Variables
        self.robot_pose = None  # [x, y, yaw, v, omega]
        self.global_path = None
        self.obstacles = np.array([])
        self.goal_reached = False

        # Declare ROS 2 Parameters for maps and planning configuration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('elevation_map_path', 'maps/generated_elevationMap_40.csv'),
                ('soil_map_path',      'maps/generated_soilMap_1.csv'),
                ('global_res',         1.0),
                ('slope_threshold',    30.0),
                ('lookahead_dist',     7.0),
                ('start_pos',          [110, 56]), # [x, y]
                ('goal_pos',           [39, 99]),  # [x, y]
                ('control_rate_hz',    10.0),      # Control loop frequency
            ])

    # =========================================================================
    # Kinematics & Coordinate Transformation Helpers
    # =========================================================================

    def yaw_from_quaternion(self, q) -> float:
        """
        Extracts the yaw (Z-axis rotation) in radians from a ROS 2 Quaternion.
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # =========================================================================
    # Core Callbacks & Control Loop
    # =========================================================================

    def odom_callback(self, msg: Odometry) -> None:
        """
        Updates the internal robot state using standard Odometry.
        Translates nav_msgs/Odometry into the [x, y, yaw, v, omega] array required by DWA.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        self.robot_pose = np.array([x, y, yaw, v, omega])

    def control_loop_callback(self) -> None:
        """
        Timer callback executing the local planner at a fixed rate.
        Extracts lookahead targets and publishes velocity commands.
        """
        if self.robot_pose is None or self.global_path is None or self.goal_reached:
            return # Wait until we have odometry and a valid path

        # Check if the goal is reached
        goal = self.get_parameter('goal_pos').value
        dist_to_goal = math.hypot(self.robot_pose[0] - goal[0], self.robot_pose[1] - goal[1])
        
        if dist_to_goal <= self.robot_conf.robot_radius:
            if not self.goal_reached:
                self.get_logger().info("Goal Reached! Stopping robot.")
                self.goal_reached = True
                self._stop_robot()
            return

        try:
            # 1. Pick a "lookahead" point on the path
            lookahead = self.get_parameter('lookahead_dist').value
            local_goal = self.local_planner.get_lookahead_point(self.robot_pose, self.global_path, lookahead)
            
            # 2. Compute best velocity [v, w] and predicted trajectory using DWA
            # Note: We pass self.obstacles which is populated during on_configure
            cmd_vel_arr, predicted_trajectory = self.local_planner.dwa_control(
                self.robot_pose, self.robot_conf, np.array(local_goal), self.obstacles
            )

            # 3. Publish to standard ROS Twist
            twist_msg = Twist()
            twist_msg.linear.x = float(cmd_vel_arr[0])
            twist_msg.angular.z = float(cmd_vel_arr[1])
            self._pub_cmd_vel.publish(twist_msg)

        except Exception as e:
            self.get_logger().error(f"Error in local control loop: {e}")
            self._stop_robot()

    def _stop_robot(self):
        """Helper to publish zero velocities"""
        if self._pub_cmd_vel and self._pub_cmd_vel.is_activated:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self._pub_cmd_vel.publish(msg)

    # =========================================================================
    # Lifecycle Transition Callbacks
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node configuration.
        Reads maps from parameters, computes Costmaps and Global Paths.
        """
        self.get_logger().info("Lifecycle state transition: CONFIGURED. Loading Maps...")

        # 1. Get the absolute path to this package's shared directory
        pkg_share_dir = get_package_share_directory('component_path_planner')

        # 2. Get the relative paths from the params file (e.g. "maps/generated_elevationMap_40.csv")
        elev_rel_path = self.get_parameter('elevation_map_path').value
        soil_rel_path = self.get_parameter('soil_map_path').value
        
        # 3. Combine them to get the absolute path!
        elev_path = os.path.join(pkg_share_dir, elev_rel_path)
        soil_path = os.path.join(pkg_share_dir, soil_rel_path)

        global_res = self.get_parameter('global_res').value
        slope_thresh = self.get_parameter('slope_threshold').value
        start = self.get_parameter('start_pos').value
        goal = self.get_parameter('goal_pos').value

        try:
            # Now load using the absolute paths
            elevationMap = np.loadtxt(open(elev_path, "rb"), delimiter=" ", skiprows=0)
            soilMap      = np.loadtxt(open(soil_path, "rb"), delimiter=" ", skiprows=0)
            
            # 2. Generate Costmap 
            costMap, obstacleMap, slopeMap = self.path_planner.getCostMap(
                elevationMap, soilMap, global_res, slope_thresh
            )
            
            # 3. Generate Global Path
            Tmap = self.path_planner.computeTmap(costMap, obstacleMap, goal, start)
            self.global_path = self.path_planner.getPathGDM(Tmap, np.asarray(start), np.asarray(goal), 0.4)

            # 4. Define static obstacles (Hardcoded wall for demonstration purposes)
            self.obstacles = np.array([
                [90, y] for y in range(66, 86)
            ])

        except Exception as e:
            self.get_logger().error(f"Failed to generate Global Path from Maps: {e}")
            return TransitionCallbackReturn.FAILURE

        # Allocate standard pub/sub endpoints
        self._pub_cmd_vel = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)
        self._sub_odom    = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.get_logger().info("Global Path Calculated. Node successfully configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node activation.
        Ignites the control loop timer to begin publishing velocity commands.
        """
        self.get_logger().info("Lifecycle state transition: ACTIVE.")
        
        rate_hz = self.get_parameter('control_rate_hz').value
        timer_period = 1.0 / rate_hz
        
        # Start the execution loop
        self.goal_reached = False
        self._control_timer = self.create_timer(timer_period, self.control_loop_callback)
        
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node deactivation.
        Halts the physical robot and stops the control timer.
        """
        self.get_logger().info("Lifecycle state transition: DEACTIVATED.")
        
        if self._control_timer:
            self._control_timer.cancel()
            self.destroy_timer(self._control_timer)
            self._control_timer = None

        self._stop_robot()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node cleanup.
        Deallocates ROS 2 interfaces and memory structures.
        """
        self.get_logger().info('Lifecycle state transition: CLEANING UP.')
        
        if self._sub_odom:
            self.destroy_subscription(self._sub_odom)
        if self._pub_cmd_vel:
            self.destroy_publisher(self._pub_cmd_vel)
            
        self.global_path = None
        self.robot_pose = None
        self.obstacles = np.array([])
            
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node shutdown.
        Ensures hardware halt regardless of current execution state.
        """
        self.get_logger().info('Lifecycle state transition: SHUTTING DOWN.')
        
        if self._control_timer:
            self._control_timer.cancel()
        
        self._stop_robot()
        
        if self._sub_odom:
            self.destroy_subscription(self._sub_odom)
        if self._pub_cmd_vel:
            self.destroy_publisher(self._pub_cmd_vel)
            
        return TransitionCallbackReturn.SUCCESS


def main():
    """
    Entry point for the executable. 
    """
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node  = LifecyclePlannerController('lc_component_path_planner')
    executor.add_node(lc_node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

if __name__ == '__main__':
    main()