"""
ROS 2 Lifecycle Node: Global Path Planning & DWA Local Trajectory Control
-------------------------------------------------------------------------
This module implements a managed lifecycle node for autonomous rover navigation 
on procedural lunar terrain. It bridges global A*/Dijkstra pathfinding with a 
Dynamic Window Approach (DWA) local controller.

Key Features:
- Matrix-to-Cartesian coordinate frame transformations.
- Live Matplotlib telemetry visualization.
- Deterministic ROS 2 Lifecycle state management (Configure, Activate, Deactivate).
- System-wide mission status broadcasting (Goal Reached).
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Optional
import warnings

from ament_index_python.packages import get_package_share_directory

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool  # For broadcasting mission status

# Custom Planner Imports
from src import global_path_planner
from src import local_trajectory_controller
from src import robot_config

# Suppress Matplotlib GUI warnings during live ROS 2 execution
warnings.filterwarnings("ignore")


class LifecyclePlannerController(rclpy.lifecycle.Node):
    """
    Lifecycle node managing the high-level routing and low-level kinematics 
    of the lunar rover.
    """
    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        
        # --- ROS 2 Interfaces ---
        self._pub_cmd_vel: Optional[Publisher] = None
        self._pub_goal_reached: Optional[Publisher] = None
        self._sub_odom:    Optional[Subscription] = None
        self._control_timer: Optional[Timer] = None
        
        # --- Planner Instances ---
        self.path_planner  = global_path_planner.Global_Path_Planner()
        self.local_planner = local_trajectory_controller.Local_Trajectory_Controller()
        
        # Define robot physical footprint
        self.robot_conf    = robot_config.Robot()
        self.robot_conf._robot_type = 1  # 1 = Rectangular Footprint
        
        # Store the absolute max speed so we can restore it after braking
        self._base_max_speed = self.robot_conf.max_speed

        # --- Internal State Tracking ---
        self.robot_pose = None  # [x, y, yaw, linear_vel, angular_vel]
        self.global_path = None
        self.obstacles = np.array([])
        self.trajectory_history = []
        self.goal_reached = False
        
        # --- Environmental Data ---
        self.elevationMap = None
        self.costMap = None
        self.obstacleMap = None

        # --- ROS 2 Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('elevation_map_path', 'maps/generated_elevationMap_40.csv'),
                ('soil_map_path',      'maps/generated_soilMap_1.csv'),
                ('global_res',         1.0),
                ('slope_threshold',    30.0),
                ('lookahead_dist',     7.0),
                ('goal_pos',           [39, 99]),
                ('control_rate_hz',    10.0),
                ('enable_path_visualization', True),
                ('braking_distance',   3.0),
                ('stop_tolerance',     0.5)
            ])

        # Initialize interactive plotting if visualization is enabled
        if self.get_parameter('enable_path_visualization').value:
            plt.ion()

    # =========================================================================
    # KINEMATICS & SENSORY CALLBACKS
    # =========================================================================

    def yaw_from_quaternion(self, q) -> float:
        """Converts a ROS geometry_msgs Quaternion into a Euler Yaw angle (radians)."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry) -> None:
        """
        Processes incoming odometry data and aligns the robot's coordinate 
        frame with the global Cartesian map frame.
        """
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        raw_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        
        # Transform logic: Map specific simulator odometry to the algorithm's global frame
        # (Note: These offsets depend strictly on how the robot spawns in the simulator)
        x = raw_x
        y = raw_y
        yaw_offset = 90.0 * math.pi / 180.0  # Rotate 90 degrees to align North = 0 rad
        yaw = -raw_yaw - yaw_offset
        
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        self.robot_pose = np.array([x, y, yaw, v, omega])
        
        # Record trajectory history for plotting (downsampled to 0.1m resolution)
        if len(self.trajectory_history) == 0 or \
           math.hypot(x - self.trajectory_history[-1][0], y - self.trajectory_history[-1][1]) > 0.1:
            self.trajectory_history.append([x, y])

    # =========================================================================
    # CORE CONTROL LOOP & PATHING
    # =========================================================================

    def _compute_global_path(self, start_pos: list):
        """Calculates the optimal global route using the generated costmaps."""
        goal_pos = self.get_parameter('goal_pos').value
        self.get_logger().info(f"Computing Global Path from {start_pos} to {goal_pos}...")

        try:
            Tmap = self.path_planner.computeTmap(self.costMap, self.obstacleMap, goal_pos, start_pos)
            self.global_path = self.path_planner.getPathGDM(Tmap, np.asarray(start_pos), np.asarray(goal_pos), 0.4)
            self.get_logger().info("Global Path successfully computed!")
        except Exception as e:
            self.get_logger().error(f"Failed to generate Global Path: {e}")

    def control_loop_callback(self) -> None:
        """
        High-frequency timer callback managing the DWA local controller.
        Calculates optimal command velocities to follow the global path,
        manages smooth deceleration, and prevents goal-state singularities.
        """
        # Abort if sensors haven't provided a valid pose yet
        if self.robot_pose is None:
            return 

        # Generate the initial route if one does not exist
        if self.global_path is None:
            start_x = int(round(self.robot_pose[0]))
            start_y = int(round(self.robot_pose[1]))
            self._compute_global_path([start_x, start_y])
            return 

        # ---------------------------------------------------------
        # 1. STATE LATCH: KEEP IT PARKED
        # If we already reached the goal, continually apply the brakes 
        # and completely bypass the DWA math.
        # ---------------------------------------------------------
        if self.goal_reached:
            self._stop_robot()
            return

        goal = self.get_parameter('goal_pos').value
        dist_to_goal = math.hypot(self.robot_pose[0] - goal[0], self.robot_pose[1] - goal[1])
        
        # ---------------------------------------------------------
        # 2. HARD STOP & MISSION STATUS
        # Ensure minimum tolerance of 0.5m to prevent high-speed overshoots
        # ---------------------------------------------------------
        base_tolerance = self.get_parameter('stop_tolerance').value
        stop_tolerance = max(self.robot_conf.robot_radius, base_tolerance) 

        if dist_to_goal <= stop_tolerance:
            self.get_logger().info(f"Goal {goal} Reached! Securing rover.")
            self.goal_reached = True
            
            # Broadcast to the rest of the ROS 2 system that the rover is parked
            if self._pub_goal_reached and self._pub_goal_reached.is_activated:
                status_msg = Bool()
                status_msg.data = True
                self._pub_goal_reached.publish(status_msg)
            
            self._stop_robot()
            
            # One final UI update to show the robot resting on the target
            if self.get_parameter('enable_path_visualization').value:
                self._update_live_plot(np.array([self.robot_pose]), goal)
            return

        # ---------------------------------------------------------
        # 3. SMOOTH BRAKING ZONE
        # Dynamically clamp DWA max linear speed as we approach
        # ---------------------------------------------------------
        braking_distance = self.get_parameter('braking_distance').value
          
        if dist_to_goal < braking_distance:
            # Linearly scale speed down, but maintain a minimum crawl (0.2 m/s)
            scaled_speed = self._base_max_speed * (dist_to_goal / braking_distance)
            self.robot_conf.max_speed = max(scaled_speed, 0.2)
        else:
            self.robot_conf.max_speed = self._base_max_speed

        # ---------------------------------------------------------
        # 4. EXECUTE DWA AND APPLY YAW DAMPING
        # ---------------------------------------------------------
        try:
            lookahead = self.get_parameter('lookahead_dist').value
            local_goal = self.local_planner.get_lookahead_point(self.robot_pose, self.global_path, lookahead)
            
            cmd_vel_arr, predicted_trajectory = self.local_planner.dwa_control(
                self.robot_pose, self.robot_conf, np.array(local_goal), self.obstacles
            )

            # Prevent the "Death Spiral": As the rover brakes near the target, 
            # dampen the steering authority so it doesn't panic and spin in place.
            if dist_to_goal < braking_distance:
                # Smoothly reduce maximum steering authority
                yaw_damping_factor = dist_to_goal / braking_distance
                cmd_vel_arr[1] *= yaw_damping_factor

                # Lock the steering straight for the final 1-meter approach
                if dist_to_goal < (stop_tolerance + 1.0):
                    cmd_vel_arr[1] = 0.0

            # Package and publish velocity commands
            twist_msg = Twist()
            twist_msg.linear.x = float(cmd_vel_arr[0])
            twist_msg.angular.z = float(cmd_vel_arr[1])
            self._pub_cmd_vel.publish(twist_msg)

            # Refresh live UI
            if self.get_parameter('enable_path_visualization').value:
                self._update_live_plot(predicted_trajectory, goal)

        except Exception as e:
            self.get_logger().error(f"Error in local control loop: {e}")
            self._stop_robot()

    def _stop_robot(self):
        """Publishes zero-velocity commands to halt the rover."""
        if self._pub_cmd_vel and self._pub_cmd_vel.is_activated:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self._pub_cmd_vel.publish(msg)

    # =========================================================================
    # VISUALIZATION TELEMETRY
    # =========================================================================

    def _preview_full_maps(self):
        """
        Renders a static, blocking preview of the parsed cost and elevation maps
        to verify matrix alignment before engaging the control loops.
        """
        self.get_logger().info("Rendering full map alignment preview...")
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Elevation Map
        im1 = ax1.imshow(self.elevationMap, origin='lower', cmap='viridis')
        ax1.set_title("Full Elevation Map (Cartesian Aligned)")
        ax1.set_xlabel("X (Cols)")
        ax1.set_ylabel("Y (Rows)")
        fig.colorbar(im1, ax=ax1, label="Elevation (Meters)")
        
        # Cost Map
        im2 = ax2.imshow(self.costMap, origin='lower', cmap='hot')
        ax2.set_title("Full Cost Map (Obstacles & Slopes)")
        ax2.set_xlabel("X (Cols)")
        ax2.set_ylabel("Y (Rows)")
        fig.colorbar(im2, ax=ax2, label="Cost Penalty")
        
        goal = self.get_parameter('goal_pos').value
        ax1.plot(goal[0], goal[1], 'xb', markersize=12, markeredgewidth=3, label="Goal")
        ax2.plot(goal[0], goal[1], 'xb', markersize=12, markeredgewidth=3, label="Goal")
        
        ax1.legend()
        ax2.legend()
        
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(3.0)  # Hold on screen momentarily to allow visual inspection

    def _update_live_plot(self, predicted_trajectory, goal):
        """Refreshes the live matplotlib window with the current environment state."""
        plt.cla()
        
        # Bind ESC key to cleanly kill the simulation
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        # Plot Environmental Matrix
        if self.elevationMap is not None:
            plt.contourf(self.elevationMap, levels=20, cmap='viridis', alpha=0.5, origin='lower')

        # Plot Global & Local Routes
        if self.global_path is not None and len(self.global_path) > 0:
             plt.plot(self.global_path[:,0], self.global_path[:,1], "-r", linewidth=2, label="Global Path")

        if predicted_trajectory is not None and len(predicted_trajectory) > 0:
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g", linewidth=2, label="Local DWA Path")
            
        if len(self.trajectory_history) > 0:
            hist = np.array(self.trajectory_history)
            plt.plot(hist[:, 0], hist[:, 1], "-b", label="Actual Trajectory")

        # Plot Entity Markers
        plt.plot(self.robot_pose[0], self.robot_pose[1], "xr")
        plt.plot(goal[0], goal[1], "xb", markersize=10, label="Target Goal")
        
        if len(self.obstacles) > 0:
            plt.plot(self.obstacles[:, 0], self.obstacles[:, 1], "sk")

        self.local_planner.plot_robot(self.robot_pose[0], self.robot_pose[1], self.robot_pose[2], self.robot_conf)
        self.local_planner.plot_arrow(self.robot_pose[0], self.robot_pose[1], self.robot_pose[2])

        # Enforce full-map viewing constraints (zoom disabled)
        if self.elevationMap is not None:
            plt.xlim(0, self.elevationMap.shape[1])
            plt.ylim(0, self.elevationMap.shape[0])
        
        plt.grid(True)
        plt.legend(loc='upper right')
        plt.pause(0.001)

    # =========================================================================
    # ROS 2 LIFECYCLE MANAGEMENT
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Loads files, transforms matrices, and initializes publishers."""
        self.get_logger().info("Lifecycle state transition: CONFIGURED.")

        pkg_share_dir = get_package_share_directory('component_path_planner')
        elev_path = os.path.join(pkg_share_dir, self.get_parameter('elevation_map_path').value)
        soil_path = os.path.join(pkg_share_dir, self.get_parameter('soil_map_path').value)

        global_res = self.get_parameter('global_res').value
        slope_thresh = self.get_parameter('slope_threshold').value

        try:
            self.elevationMap = np.loadtxt(open(elev_path, "rb"), delimiter=" ", skiprows=0)
            soilMap = np.loadtxt(open(soil_path, "rb"), delimiter=" ", skiprows=0)
            
            # Generate navigational costmaps using the properly aligned matrices
            self.costMap, self.obstacleMap, _ = self.path_planner.getCostMap(
                self.elevationMap, soilMap, global_res, slope_thresh
            )
            
            # Hardcoded test obstacles
            self.obstacles = np.array([
                [90, y] for y in range(66, 86)
            ])

            # Trigger visual validation
            if self.get_parameter('enable_path_visualization').value:
                self._preview_full_maps()

        except Exception as e:
            self.get_logger().error(f"Failed to load Maps/Costmaps: {e}")
            return TransitionCallbackReturn.FAILURE

        self._pub_cmd_vel = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)
        self._pub_goal_reached = self.create_lifecycle_publisher(Bool, 'mission_status/goal_reached', 10)
        self._sub_odom    = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Engages the control loop timer."""
        self.get_logger().info("Lifecycle state transition: ACTIVE.")
        rate_hz = self.get_parameter('control_rate_hz').value
        
        # Reset goal tracking for new activation
        self.goal_reached = False
        if self._pub_goal_reached and self._pub_goal_reached.is_activated:
            msg = Bool()
            msg.data = False
            self._pub_goal_reached.publish(msg)
            
        self._control_timer = self.create_timer(1.0 / rate_hz, self.control_loop_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Halts the control loop and stops the rover."""
        if self._control_timer:
            self.destroy_timer(self._control_timer)
            self._control_timer = None
        self._stop_robot()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Tears down interfaces and closes open windows."""
        if self._sub_odom: self.destroy_subscription(self._sub_odom)
        if self._pub_cmd_vel: self.destroy_publisher(self._pub_cmd_vel)
        if self._pub_goal_reached: self.destroy_publisher(self._pub_goal_reached)
        plt.close('all')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Emergency node termination protocol."""
        if self._control_timer: self._control_timer.cancel()
        self._stop_robot()
        
        if self._sub_odom: self.destroy_subscription(self._sub_odom)
        if self._pub_cmd_vel: self.destroy_publisher(self._pub_cmd_vel)
        if self._pub_goal_reached: self.destroy_publisher(self._pub_goal_reached)
        plt.close('all')
        return TransitionCallbackReturn.SUCCESS


def main():
    """Entry point for the ROS 2 executable."""
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node  = LifecyclePlannerController('lc_component_path_planner')
    executor.add_node(lc_node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        lc_node.destroy_node()

if __name__ == '__main__':
    main()