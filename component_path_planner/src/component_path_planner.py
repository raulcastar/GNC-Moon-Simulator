"""
ROS 2 Lifecycle Node for Global Path Planning and DWA Local Trajectory Control.
"""
import os
from ament_index_python.packages import get_package_share_directory

from typing import Optional
import math
import numpy as np

# Use standard interactive plotting
import matplotlib.pyplot as plt

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
    def __init__(self, node_name: str, **kwargs):
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
        self.trajectory_history = []
        self.goal_reached = False
        
        # Store maps
        self.elevationMap = None
        self.costMap = None
        self.obstacleMap = None

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
                ('enable_path_visualization', True)
            ])

        # Enable interactive mode for live plotting
        if self.get_parameter('enable_path_visualization').value:
            plt.ion()

    def yaw_from_quaternion(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry) -> None:
        """Raw Odometry without translations/offsets."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        print(f"Raw Odometry: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg")
        
        # Modify the odometry to match the coordinate system of the maps and planner
        x = - x
        yaw_offset = 90 * math.pi / 180.0  # To make heading to the north the 0 degree reference
        yaw  = -yaw - yaw_offset
        
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        self.robot_pose = np.array([x, y, yaw, v, omega])
        
        # Store for plotting trail
        if len(self.trajectory_history) == 0 or \
           math.hypot(x - self.trajectory_history[-1][0], y - self.trajectory_history[-1][1]) > 0.1:
            self.trajectory_history.append([x, y])

    def _compute_global_path(self, start_pos: list):
        goal_pos = self.get_parameter('goal_pos').value
        self.get_logger().info(f"Computing Global Path from {start_pos} to {goal_pos}...")

        try:
            Tmap = self.path_planner.computeTmap(self.costMap, self.obstacleMap, goal_pos, start_pos)
            self.global_path = self.path_planner.getPathGDM(Tmap, np.asarray(start_pos), np.asarray(goal_pos), 0.4)
            self.get_logger().info("Global Path successfully computed!")
        except Exception as e:
            self.get_logger().error(f"Failed to generate Global Path: {e}")

    def control_loop_callback(self) -> None:
        if self.robot_pose is None:
            return 

        if self.global_path is None:
            start_x = int(round(self.robot_pose[0]))
            start_y = int(round(self.robot_pose[1]))
            self._compute_global_path([start_x, start_y])
            return 

        if self.goal_reached:
            return

        goal = self.get_parameter('goal_pos').value
        dist_to_goal = math.hypot(self.robot_pose[0] - goal[0], self.robot_pose[1] - goal[1])
        
        if dist_to_goal <= self.robot_conf.robot_radius:
            if not self.goal_reached:
                self.get_logger().info("Goal Reached!")
                self.goal_reached = True
                self._stop_robot()
            return

        try:
            lookahead = self.get_parameter('lookahead_dist').value
            local_goal = self.local_planner.get_lookahead_point(self.robot_pose, self.global_path, lookahead)
            
            cmd_vel_arr, predicted_trajectory = self.local_planner.dwa_control(
                self.robot_pose, self.robot_conf, np.array(local_goal), self.obstacles
            )

            twist_msg = Twist()
            twist_msg.linear.x = float(cmd_vel_arr[0])
            twist_msg.angular.z = float(cmd_vel_arr[1])
            self._pub_cmd_vel.publish(twist_msg)

            # Live Update Plot
            if self.get_parameter('enable_path_visualization').value:
                self._update_live_plot(predicted_trajectory, goal)

        except Exception as e:
            self.get_logger().error(f"Error in local control loop: {e}")
            self._stop_robot()

    def _update_live_plot(self, predicted_trajectory, goal):
        """Refreshes the live matplotlib window with the current environment state."""
        plt.cla()
        
        # Stop simulation with ESC
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        # Plot Background Map
        if self.elevationMap is not None:
            plt.contourf(self.elevationMap, levels=20, cmap='viridis', alpha=0.5, origin='lower')

        # Plot Global Path
        if self.global_path is not None and len(self.global_path) > 0:
             plt.plot(self.global_path[:,0], self.global_path[:,1], "-r", linewidth=2, label="Global Path")

        # Plot Local Path (DWA)
        if predicted_trajectory is not None and len(predicted_trajectory) > 0:
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g", linewidth=2, label="Local Path")
            
        # Plot Trajectory History
        if len(self.trajectory_history) > 0:
            hist = np.array(self.trajectory_history)
            plt.plot(hist[:, 0], hist[:, 1], "-b", label="Actual Trajectory")

        # Plot current position, goal, and obstacles
        plt.plot(self.robot_pose[0], self.robot_pose[1], "xr")
        plt.plot(goal[0], goal[1], "xb", markersize=10)
        
        if len(self.obstacles) > 0:
            plt.plot(self.obstacles[:, 0], self.obstacles[:, 1], "sk")

        # Plot Robot footprint and orientation
        self.local_planner.plot_robot(self.robot_pose[0], self.robot_pose[1], self.robot_pose[2], self.robot_conf)
        self.local_planner.plot_arrow(self.robot_pose[0], self.robot_pose[1], self.robot_pose[2])

        padding = 15.0 
        plt.xlim(self.robot_pose[0] - padding, self.robot_pose[0] + padding)
        plt.ylim(self.robot_pose[1] - padding, self.robot_pose[1] + padding)
        
        plt.grid(True)
        plt.legend(loc='upper right')
        
        # This is the command that forces the window to refresh live
        plt.pause(0.001)

    def _stop_robot(self):
        if self._pub_cmd_vel and self._pub_cmd_vel.is_activated:
            msg = Twist()
            self._pub_cmd_vel.publish(msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle state transition: CONFIGURED.")

        pkg_share_dir = get_package_share_directory('component_path_planner')
        elev_path = os.path.join(pkg_share_dir, self.get_parameter('elevation_map_path').value)
        soil_path = os.path.join(pkg_share_dir, self.get_parameter('soil_map_path').value)

        global_res = self.get_parameter('global_res').value
        slope_thresh = self.get_parameter('slope_threshold').value

        try:
            self.elevationMap = np.loadtxt(open(elev_path, "rb"), delimiter=" ", skiprows=0)
            soilMap      = np.loadtxt(open(soil_path, "rb"), delimiter=" ", skiprows=0)
            
            self.costMap, self.obstacleMap, _ = self.path_planner.getCostMap(
                self.elevationMap, soilMap, global_res, slope_thresh
            )
            
            self.obstacles = np.array([
                [90, y] for y in range(66, 86)
            ])

        except Exception as e:
            self.get_logger().error(f"Failed to load Maps/Costmaps: {e}")
            return TransitionCallbackReturn.FAILURE

        self._pub_cmd_vel = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)
        self._sub_odom    = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle state transition: ACTIVE.")
        rate_hz = self.get_parameter('control_rate_hz').value
        self.goal_reached = False
        self._control_timer = self.create_timer(1.0 / rate_hz, self.control_loop_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._control_timer:
            self.destroy_timer(self._control_timer)
            self._control_timer = None
        self._stop_robot()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._sub_odom: self.destroy_subscription(self._sub_odom)
        if self._pub_cmd_vel: self.destroy_publisher(self._pub_cmd_vel)
        plt.close('all')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if self._control_timer: self._control_timer.cancel()
        self._stop_robot()
        plt.close('all')
        return TransitionCallbackReturn.SUCCESS

def main():
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