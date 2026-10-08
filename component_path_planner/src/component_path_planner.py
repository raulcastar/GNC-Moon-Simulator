"""
ROS 2 Lifecycle Node: Global Path Planning & DWA Local Trajectory Control

This module implements a managed lifecycle node for autonomous rover navigation 
on procedural lunar terrain. It bridges global A*/Dijkstra pathfinding with a 
Dynamic Window Approach (DWA) local kinematic controller.

Key Capabilities:
- Matrix-to-Cartesian coordinate frame transformations.
- Deterministic ROS 2 Lifecycle state management.
- System-wide mission status broadcasting via state latches.
- Dynamic asynchronous goal allocation via standard ROS 2 PoseStamped topics.
- High-fidelity Network Visualization (ROS 2 Services for Map UI).
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Optional
from types import SimpleNamespace
import warnings

from ament_index_python.packages import get_package_share_directory

import json

import rclpy
import rclpy.lifecycle
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Empty, Float32MultiArray, MultiArrayDimension, String

# --- Custom Service Import ---
from custom_map_interfaces.srv import GetMatrices

# Custom Planner Imports
from src import global_path_planner
from src import local_trajectory_controller

# Suppress Matplotlib GUI warnings during live ROS 2 execution
warnings.filterwarnings("ignore")

# A goal on impassable ground is moved to the nearest drivable cell within this
# many grid cells (metres at 1 m/px), if the rover can reach it.
GOAL_SNAP_RADIUS_CELLS = 5


class LifecyclePlannerController(rclpy.lifecycle.Node):
    """
    Lifecycle node managing the high-level routing and low-level kinematics 
    of the rover platform.
    
    This class handles:
        1. Ingesting topological and cost matrices.
        2. Listening to kinematic telemetry (Odometry).
        3. Listening to asynchronous goal dispatches (PoseStamped).
        4. Transmitting local velocity commands (Twist).
        5. Acting as a Service Server to provide raw map arrays for UIs.
    """
    def __init__(self, node_name: str, **kwargs):
        """
        Initializes the LifecyclePlannerController node and declares dynamic parameters.

        Args:
            node_name (str): The designated identifier for the ROS 2 node.
            **kwargs: Additional keyword arguments passed to the parent LifecycleNode.
        """
        super().__init__(node_name, **kwargs)
        
        # --- ROS 2 Interfaces: Actuators & Status ---
        self._pub_cmd_vel: Optional[Publisher] = None
        self._pub_goal_reached: Optional[Publisher] = None
        self._pub_state: Optional[Publisher] = None
        self._sub_odom:    Optional[Subscription] = None
        self._sub_goal:    Optional[Subscription] = None
        self._sub_cancel:  Optional[Subscription] = None
        
        # --- ROS 2 Interfaces: Telemetry & Services ---
        self._pub_global_path: Optional[Publisher] = None
        self._pub_trajectory: Optional[Publisher] = None
        self._srv_get_maps = None
        
        self._control_timer: Optional[Timer] = None
        
        # --- Planner Instances ---
        self.path_planner  = global_path_planner.Global_Path_Planner()
        self.local_planner = local_trajectory_controller.Local_Trajectory_Controller()
        
        # Dynamic configuration namespace populated during the on_configure transition
        self.robot_conf = SimpleNamespace()
        self._base_max_speed = 0.0

        # --- Internal State Tracking ---
        self.robot_pose = None      # Vector: [x, y, yaw, linear_vel, angular_vel]
        # Last commanded (v, omega), used as the DWA velocity state
        self._last_cmd = (0.0, 0.0)
        self.current_goal = None    # Vector: [x, y]
        self.global_path = None
        self.obstacles = np.array([])
        self.trajectory_history = []
        self.goal_reached = False
        # A Play/Stop in Unreal teleports the rover back to its start pose, which
        # shows up as a one-step odometry jump far larger than any real motion
        # between frames. Above this many metres in one step, treat it as a new
        # session and wipe the stale trajectory/goal so the map reflects the run.
        self._session_reset_dist = 2.0
        self._tilt_deg = 0.0        # Angle between body z axis and vertical [deg]
        
        # --- Environmental Matrices ---
        self.elevationMap = None
        self.costMap = None
        self.obstacleMap = None
        self.freeRegions = None     # Connected drivable regions (label per cell)

        # --- Parameter Declarations for Runtime Mutability ---
        self.declare_parameters(
            namespace='',
            parameters=[
                # Filepaths and Map configuration
                ('elevation_map_path', 'maps/generated_elevationMap_40.csv'),
                ('soil_map_path',      'maps/generated_soilMap_1.csv'),
                ('global_res',         1.0),
                ('slope_threshold',    15.0),
                ('max_tilt_deg',       15.0),
                ('lookahead_dist',     7.0),
                
                # Execution and UI
                ('control_rate_hz',    10.0),
                ('enable_path_visualization', True),
                ('braking_distance',   3.0),
                ('stop_tolerance',     0.5),
                
                # DWA and Kinematic Constraints
                ('max_speed', 2.5),
                ('min_speed', 0.0),
                ('max_accel', 2.5),
                ('max_yaw_rate_deg', 1.5),
                ('max_delta_yaw_rate_deg', 1.5),
                ('v_resolution', 0.01),
                ('yaw_rate_resolution_deg', 0.1),
                ('dt', 0.1),
                ('predict_time', 10.0),
                ('to_goal_cost_gain', 1.0),
                ('speed_cost_gain', 1.0),
                ('obstacle_cost_gain', 1.0),
                ('robot_stuck_flag_cons', 0.001),
                
                # Physical Footprint
                ('robot_width', 0.35),
                ('robot_length', 0.35),
                ('robot_radius', 0.35)
            ])
            
        if self.get_parameter('enable_path_visualization').value:
            plt.ion()

    # =========================================================================
    # KINEMATICS & SENSORY CALLBACKS
    # =========================================================================

    def yaw_from_quaternion(self, q) -> float:
        """
        Extracts the Euler yaw angle from a geometry_msgs Quaternion.
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry) -> None:
        """
        Processes telemetry data and maps the local odometry frame to the 
        global Cartesian environment frame.
        """
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        raw_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        
        # Odom now publishes a clean REP-103 heading (the rover node bakes in the
        # Unreal-frame correction), so the quaternion yaw IS the heading — the
        # old `-raw_yaw - 90` compensation is no longer needed.
        x = raw_x
        y = raw_y
        yaw = raw_yaw
        
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # New session (Unreal Play/Stop): the rover teleports to its start pose.
        # Detect the resulting large one-step jump and clear the previous run's
        # trajectory, goal and path — then push empty paths so the map clears too.
        if self.robot_pose is not None:
            jump = math.hypot(x - self.robot_pose[0], y - self.robot_pose[1])
            if jump > self._session_reset_dist:
                self.get_logger().info(
                    f"Odometry jumped {jump:.1f} m — treating as an Unreal "
                    f"Play/Stop. Clearing trajectory, goal and map overlays.")
                self.trajectory_history = []
                self.current_goal = None
                self.global_path = None
                self.goal_reached = False
                self._stop_robot()
                self._publish_clear()
                self._set_state('idle', 'new session')

        self.robot_pose = np.array([x, y, yaw, v, omega])

        # Combined roll/pitch tilt: world-z component of the body z axis.
        q = msg.pose.pose.orientation
        cz = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
        self._tilt_deg = math.degrees(math.acos(cz))

        # Downsample trajectory recording to 0.1m intervals to prevent memory saturation
        if len(self.trajectory_history) == 0 or \
           math.hypot(x - self.trajectory_history[-1][0], y - self.trajectory_history[-1][1]) > 0.1:
            self.trajectory_history.append([x, y])

    def _set_state(self, state: str, reason: str = '') -> None:
        """
        Publishes the mission state on mission_status/state as JSON:
        {"state": idle|planning|driving|reached|aborted|cancelled,
         "goal": [x, y] or null, "reason": str}.
        """
        if self._pub_state is None or not self._pub_state.is_activated:
            return
        goal = [round(float(v), 2) for v in self.current_goal] if self.current_goal else None
        self._pub_state.publish(String(data=json.dumps(
            {'state': state, 'goal': goal, 'reason': reason})))

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """
        Asynchronous callback for dynamic waypoint assignment via external GUIs.
        Interrupts the active control loop and forces a global path recalculation.
        """
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        
        self.get_logger().info(f"Waypoint update received: [{new_x:.2f}, {new_y:.2f}]. Suspending kinematics.")
        
        # Enforce zero-velocity state to prevent drifting during computational blocking
        self._stop_robot()
        
        # Reset tracking states to trigger the routing subroutines
        self.current_goal = [new_x, new_y]
        self._set_state('planning')
        self.global_path = None
        self.goal_reached = False 
        
        # Invalidate the previous mission completion state across the ROS 2 network
        if self._pub_goal_reached and self._pub_goal_reached.is_activated:
            status_msg = Bool()
            status_msg.data = False
            self._pub_goal_reached.publish(status_msg)

    def cancel_goal_callback(self, msg: Empty) -> None:
        """
        Aborts the active goal on an operator ALL STOP and halts the rover, so
        the control loop does not override the stop on its next cycle.
        """
        if self.current_goal is not None:
            self.get_logger().warn(f"Goal {self.current_goal} cancelled by the operator.")
            self._set_state('cancelled', 'by the operator (ALL STOP or manual drive)')
        self.current_goal = None
        self.global_path = None
        self._stop_robot()

        # Clear the abandoned route from the UI; the driven trajectory is kept.
        if self._pub_global_path and self._pub_global_path.is_activated:
            empty = Path()
            empty.header.frame_id = 'map'
            empty.header.stamp = self.get_clock().now().to_msg()
            self._pub_global_path.publish(empty)

    # =========================================================================
    # CORE CONTROL LOOP & PATHING
    # =========================================================================

    def _drivable_goal(self, goal: list, start: list) -> Optional[list]:
        """
        `goal` if it is drivable, else the nearest drivable cell within
        GOAL_SNAP_RADIUS_CELLS that lies in the rover's drivable region (so a path
        can exist); None if there is none.
        """
        rows, cols = self.obstacleMap.shape
        gx, gy = goal
        if not (0 <= gx < cols and 0 <= gy < rows):
            return None
        if self.obstacleMap[gy, gx] != 1:
            return goal
        r = GOAL_SNAP_RADIUS_CELLS
        y0, y1, x0, x1 = max(0, gy - r), min(rows, gy + r + 1), max(0, gx - r), min(cols, gx + r + 1)
        free = self.obstacleMap[y0:y1, x0:x1] != 1
        sx, sy = start
        if self.freeRegions is not None and 0 <= sx < cols and 0 <= sy < rows and self.freeRegions[sy, sx]:
            free &= self.freeRegions[y0:y1, x0:x1] == self.freeRegions[sy, sx]
        ys, xs = np.nonzero(free)
        if ys.size == 0:
            return None
        d2 = (ys + y0 - gy) ** 2 + (xs + x0 - gx) ** 2
        k = int(np.argmin(d2))
        if d2[k] > r * r:
            return None
        return [int(xs[k] + x0), int(ys[k] + y0)]

    def _compute_global_path(self, start_pos: list) -> None:
        """
        Generates the optimal global route bridging the starting coordinate 
        to the target coordinate using the processed costmap matrices.
        """
        if self.current_goal is None:
            return

        # Ensure target coordinates are integers to prevent NumPy matrix indexing faults
        goal_x = int(round(self.current_goal[0]))
        goal_y = int(round(self.current_goal[1]))
        grid_goal = [goal_x, goal_y]

        # A goal on impassable ground (too steep, or no-go soil) would leave the
        # whole search stuck at the goal: move it to the nearest drivable cell
        # the rover can reach, or refuse it.
        snapped = self._drivable_goal(grid_goal, start_pos)
        if snapped is None:
            self.get_logger().warn(
                f"Goal {grid_goal} is on impassable ground (slope over "
                f"{self.get_parameter('slope_threshold').value:.0f} deg or no-go soil) with no "
                f"reachable drivable cell within {GOAL_SNAP_RADIUS_CELLS} cells. Pick another point.")
            self._set_state('aborted', 'goal on impassable ground')
            self.current_goal = None
            self.global_path = None
            return
        if snapped != grid_goal:
            self.get_logger().info(f"Goal {grid_goal} is on impassable ground; moved to the "
                                   f"nearest drivable cell {snapped}.")
            grid_goal = snapped
            self.current_goal = [float(snapped[0]), float(snapped[1])]

        self.get_logger().info(f"Computing global trajectory from {start_pos} to {grid_goal}...")

        try:
            Tmap = self.path_planner.computeTmap(self.costMap, self.obstacleMap, grid_goal, start_pos)
            if not np.isfinite(Tmap[start_pos[1], start_pos[0]]):
                # The search from the goal never reached the rover: steep ground
                # or no-go soil separates them.
                self.get_logger().warn(f"Goal {grid_goal} is not reachable from the rover "
                                       "(cut off by steep ground or no-go soil).")
                self._set_state('aborted', 'goal not reachable')
                self.current_goal = None
                self.global_path = None
                return
            self.global_path = self.path_planner.getPathGDM(Tmap, np.asarray(start_pos), np.asarray(grid_goal), 0.4)

            # --- Path Validation Safety Check ---
            if self.global_path is not None and np.isnan(self.global_path).any():
                self.global_path = None
            if self.global_path is None or len(self.global_path) < 2:
                self.get_logger().warn(f"Pathfinding failed! Coordinates {grid_goal} are unreachable or inside an obstacle.")
                self._set_state('aborted', 'no path to the goal')
                self.current_goal = None
                self.global_path = None
                return
                
            self.get_logger().info(f"Global trajectory resolved with {len(self.global_path)} waypoints. Resuming kinematic control.")
            self._set_state('driving')
            self._broadcast_paths()
            
        except Exception as e:
            self.get_logger().error(f"Global path generation fault: {e}")
            self._set_state('aborted', 'path planning error')
            self.current_goal = None
            self.global_path = None

    def control_loop_callback(self) -> None:
        """
        High-frequency timer callback executing the DWA local controller.
        Calculates actuator commands, manages terminal deceleration, and handles 
        singularity avoidance near the goal coordinate.
        """
        # Suspend execution if spatial context is unavailable OR if we have no active mission
        if self.robot_pose is None or self.current_goal is None:
            return

        # Tilt safety: stop and abort the goal on excessive body tilt.
        max_tilt = self.get_parameter('max_tilt_deg').value
        if self._tilt_deg > max_tilt:
            self.get_logger().error(
                f"Tilt safety stop: body tilt {self._tilt_deg:.1f} deg exceeds "
                f"{max_tilt:.1f} deg. Goal {self.current_goal} aborted.")
            self._set_state('aborted', f'tilt {self._tilt_deg:.1f} deg > {max_tilt:.1f} deg')
            self._stop_robot()
            self.current_goal = None
            self.global_path = None
            return

        # Halt kinematics and defer to the global planner if no valid route exists
        if self.global_path is None:
            self._stop_robot()
            start_x = int(round(self.robot_pose[0]))
            start_y = int(round(self.robot_pose[1]))
            self._compute_global_path([start_x, start_y])
            return 

        # State Latch: Prevent odometry drift from re-triggering the controller post-arrival
        if self.goal_reached:
            self._stop_robot()
            return

        dist_to_goal = math.hypot(self.robot_pose[0] - self.current_goal[0], self.robot_pose[1] - self.current_goal[1])
        
        # ---------------------------------------------------------
        # Terminal Stopping Condition & Network Broadcasting
        # ---------------------------------------------------------
        base_tolerance = self.get_parameter('stop_tolerance').value
        stop_tolerance = max(self.robot_conf.robot_radius, base_tolerance) 

        if dist_to_goal <= stop_tolerance:
            self.get_logger().info(f"Target {self.current_goal} achieved. Engaging parking brake.")
            self._set_state('reached')
            self.goal_reached = True
            
            if self._pub_goal_reached and self._pub_goal_reached.is_activated:
                status_msg = Bool()
                status_msg.data = True
                self._pub_goal_reached.publish(status_msg)
            
            self._stop_robot()
            self._broadcast_paths()  # Ensure UI reflects the final resting position
            
            if self.get_parameter('enable_path_visualization').value:
                self._update_live_plot(np.array([self.robot_pose]), self.current_goal)
            
            self.current_goal = None
            return

        # ---------------------------------------------------------
        # Kinematic Deceleration Profile
        # ---------------------------------------------------------
        braking_distance = self.get_parameter('braking_distance').value
          
        if dist_to_goal < braking_distance:
            # Linearly attenuate maximum velocity to prevent target overshoot
            scaled_speed = self._base_max_speed * (dist_to_goal / braking_distance)
            self.robot_conf.max_speed = max(scaled_speed, 0.2)
        else:
            self.robot_conf.max_speed = self._base_max_speed

        # ---------------------------------------------------------
        # Trajectory Optimization & Singularity Avoidance
        # ---------------------------------------------------------
        try:
            lookahead = self.get_parameter('lookahead_dist').value
            local_goal = self.local_planner.get_lookahead_point(self.robot_pose, self.global_path, lookahead)
            
            state = np.array([*self.robot_pose[:3], *self._last_cmd])
            cmd_vel_arr, predicted_trajectory = self.local_planner.dwa_control(
                state, self.robot_conf, np.array(local_goal), self.obstacles
            )

            # Prevent angular oscillation ("Death Spiral") caused by DWA resolving
            # atan2 boundaries when directly over the target coordinate.
            if dist_to_goal < braking_distance:
                yaw_damping_factor = dist_to_goal / braking_distance
                cmd_vel_arr[1] *= yaw_damping_factor
                
                # Lock steering actuator vector for the final approach segment
                if dist_to_goal < (stop_tolerance + 1.0):
                    cmd_vel_arr[1] = 0.0

            twist_msg = Twist()
            twist_msg.linear.x = float(cmd_vel_arr[0])
            twist_msg.angular.z = float(cmd_vel_arr[1])
            self._pub_cmd_vel.publish(twist_msg)
            self._last_cmd = (twist_msg.linear.x, twist_msg.angular.z)

            # Periodically stream the actual driven trajectory to the UI
            self._broadcast_paths()
            if self.get_parameter('enable_path_visualization').value:
                self._update_live_plot(predicted_trajectory, self.current_goal)

        except Exception as e:
            self.get_logger().error(f"DWA algorithmic fault: {e}")
            self._stop_robot()

    def _stop_robot(self) -> None:
        """Transmits absolute zero-velocity limits to the hardware/simulation actuators."""
        self._last_cmd = (0.0, 0.0)
        if self._pub_cmd_vel and self._pub_cmd_vel.is_activated:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self._pub_cmd_vel.publish(msg)

    # =========================================================================
    # SERVICE HANDLERS & NETWORK BROADCASTING
    # =========================================================================

    def _pack_numpy_to_multiarray(self, matrix: np.ndarray) -> Float32MultiArray:
        """
        Helper function to package a 2D NumPy array into a Float32MultiArray 
        while preserving dimensional data.
        """
        msg = Float32MultiArray()
        
        # Define matrix rows (height)
        dim_height = MultiArrayDimension()
        dim_height.label = "height"
        dim_height.size = matrix.shape[0]
        dim_height.stride = matrix.shape[0] * matrix.shape[1]
        
        # Define matrix columns (width)
        dim_width = MultiArrayDimension()
        dim_width.label = "width"
        dim_width.size = matrix.shape[1]
        dim_width.stride = matrix.shape[1]
        
        msg.layout.dim = [dim_height, dim_width]
        msg.data = matrix.flatten().tolist()
        return msg

    def get_matrices_callback(self, request, response):
        """
        Answers the service call by packing and returning the current matrices.
        This guarantees the UI only receives the map when it explicitly asks for it.
        """
        self.get_logger().info("Map Service called. Packaging matrices for UI.")
        
        if self.costMap is not None:
            # Safely serialize infinite obstacles for IEEE float compliance
            safe_cost_map = np.copy(self.costMap)
            safe_cost_map[np.isinf(safe_cost_map)] = 9999.0 
            response.cost_map = self._pack_numpy_to_multiarray(safe_cost_map)
            
        if self.elevationMap is not None:
            response.elevation_map = self._pack_numpy_to_multiarray(self.elevationMap)
            
        return response

    def _publish_clear(self) -> None:
        """
        Push empty Global Path and Trajectory messages so the UI wipes the
        previous run's overlays. _broadcast_paths skips an empty trajectory (it
        guards on len > 0), so on a session reset the stale trail would otherwise
        never be cleared — this sends the empty paths explicitly, once.
        """
        timestamp = self.get_clock().now().to_msg()
        empty = Path()
        empty.header.frame_id = 'map'
        empty.header.stamp = timestamp
        if self._pub_global_path and self._pub_global_path.is_activated:
            self._pub_global_path.publish(empty)
        if self._pub_trajectory and self._pub_trajectory.is_activated:
            self._pub_trajectory.publish(empty)

    def _broadcast_paths(self) -> None:
        """
        Translates the Global Path and Historical Trajectory arrays into
        standard ROS 2 Path messages for visualizer consumption.
        """
        timestamp = self.get_clock().now().to_msg()
        
        # Broadcast the intended Global Path
        if self._pub_global_path and self._pub_global_path.is_activated and self.global_path is not None:
            global_msg = Path()
            global_msg.header.frame_id = 'map'
            global_msg.header.stamp = timestamp
            
            for pt in self.global_path:
                ps = PoseStamped()
                ps.header = global_msg.header
                ps.pose.position.x = float(pt[0])
                ps.pose.position.y = float(pt[1])
                global_msg.poses.append(ps)
                
            self._pub_global_path.publish(global_msg)
            
        # Broadcast the driven Trajectory History
        if self._pub_trajectory and self._pub_trajectory.is_activated and len(self.trajectory_history) > 0:
            hist_msg = Path()
            hist_msg.header.frame_id = 'map'
            hist_msg.header.stamp = timestamp
            
            for pt in self.trajectory_history:
                ps = PoseStamped()
                ps.header = hist_msg.header
                ps.pose.position.x = float(pt[0])
                ps.pose.position.y = float(pt[1])
                hist_msg.poses.append(ps)
                
            self._pub_trajectory.publish(hist_msg)

    # =========================================================================
    # DEBUG MATPLOTLIB
    # =========================================================================

    def _preview_full_maps(self) -> None:
        """
        Renders a static topological preview of the parsed cost and elevation maps
        to verify matrix alignment prior to kinematic activation.
        """
        self.get_logger().info("Rendering topological matrix alignment preview...")
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        im1 = ax1.imshow(self.elevationMap, origin='lower', cmap='viridis')
        ax1.set_title("Full Elevation Map (Cartesian Aligned)")
        ax1.set_xlabel("X (Cols)")
        ax1.set_ylabel("Y (Rows)")
        fig.colorbar(im1, ax=ax1, label="Elevation (Meters)")
        
        im2 = ax2.imshow(self.costMap, origin='lower', cmap='hot')
        ax2.set_title("Full Cost Map (Obstacles & Slopes)")
        ax2.set_xlabel("X (Cols)")
        ax2.set_ylabel("Y (Rows)")
        fig.colorbar(im2, ax=ax2, label="Cost Penalty")
        
        if self.current_goal is not None:
            ax1.plot(self.current_goal[0], self.current_goal[1], 'xb', markersize=12, markeredgewidth=3, label="Goal")
            ax2.plot(self.current_goal[0], self.current_goal[1], 'xb', markersize=12, markeredgewidth=3, label="Goal")
        
        ax1.legend()
        ax2.legend()
        
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(3.0)

    def _update_live_plot(self, predicted_trajectory: np.ndarray, goal: list) -> None:
        """
        Refreshes the live matplotlib UI with localized trajectory telemetry.
        """
        plt.cla()
        
        # Bind ESC key to allow clean termination of the blocking UI thread
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        if self.elevationMap is not None:
            plt.contourf(self.elevationMap, levels=20, cmap='viridis', alpha=0.5, origin='lower')

        if self.global_path is not None and len(self.global_path) > 0:
             plt.plot(self.global_path[:,0], self.global_path[:,1], "-r", linewidth=2, label="Global Path")

        if predicted_trajectory is not None and len(predicted_trajectory) > 0:
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g", linewidth=2, label="Local DWA Path")
            
        if len(self.trajectory_history) > 0:
            hist = np.array(self.trajectory_history)
            plt.plot(hist[:, 0], hist[:, 1], "-b", label="Actual Trajectory")

        plt.plot(self.robot_pose[0], self.robot_pose[1], "xr")
        if goal is not None:
            plt.plot(goal[0], goal[1], "xb", markersize=10, label="Target Goal")
        
        if len(self.obstacles) > 0:
            plt.plot(self.obstacles[:, 0], self.obstacles[:, 1], "sk")

        self.local_planner.plot_robot(self.robot_pose[0], self.robot_pose[1], self.robot_pose[2], self.robot_conf)
        self.local_planner.plot_arrow(self.robot_pose[0], self.robot_pose[1], self.robot_pose[2])

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
        """
        Handles the Unconfigured -> Inactive transition.
        Resolves parameters, instantiates the configuration namespace, and generates matrices.
        """
        self.get_logger().info("Lifecycle transition: CONFIGURED. Standing by for UI coordinates.")

        # --- Enforce Idle Start State ---
        self.current_goal = None

        # Build kinematic configuration structure from dynamic parameters
        self.robot_conf.max_speed = self.get_parameter('max_speed').value
        self.robot_conf.min_speed = self.get_parameter('min_speed').value
        self.robot_conf.max_accel = self.get_parameter('max_accel').value
        self.robot_conf.max_yaw_rate = math.radians(self.get_parameter('max_yaw_rate_deg').value)
        self.robot_conf.max_delta_yaw_rate = math.radians(self.get_parameter('max_delta_yaw_rate_deg').value)
        self.robot_conf.yaw_rate_resolution = math.radians(self.get_parameter('yaw_rate_resolution_deg').value)
        self.robot_conf.v_resolution = self.get_parameter('v_resolution').value
        self.robot_conf.dt = self.get_parameter('dt').value
        self.robot_conf.predict_time = self.get_parameter('predict_time').value
        self.robot_conf.to_goal_cost_gain = self.get_parameter('to_goal_cost_gain').value
        self.robot_conf.speed_cost_gain = self.get_parameter('speed_cost_gain').value
        self.robot_conf.obstacle_cost_gain = self.get_parameter('obstacle_cost_gain').value
        self.robot_conf.robot_stuck_flag_cons = self.get_parameter('robot_stuck_flag_cons').value
        self.robot_conf.robot_type = 1 
        self.robot_conf.robot_width = self.get_parameter('robot_width').value
        self.robot_conf.robot_length = self.get_parameter('robot_length').value
        self.robot_conf.robot_radius = self.get_parameter('robot_radius').value

        self._base_max_speed = self.robot_conf.max_speed

        pkg_share_dir = get_package_share_directory('component_path_planner')
        elev_path = os.path.join(pkg_share_dir, self.get_parameter('elevation_map_path').value)
        soil_path = os.path.join(pkg_share_dir, self.get_parameter('soil_map_path').value)

        global_res = self.get_parameter('global_res').value
        slope_thresh = self.get_parameter('slope_threshold').value

        try:
            self.elevationMap = np.loadtxt(open(elev_path, "rb"), delimiter=" ", skiprows=0)
            soilMap = np.loadtxt(open(soil_path, "rb"), delimiter=" ", skiprows=0)
            
            self.costMap, self.obstacleMap, _ = self.path_planner.getCostMap(
                self.elevationMap, soilMap, global_res, slope_thresh
            )
            # Drivable regions, 4-connected like the planner's search.
            import cv2
            _, self.freeRegions = cv2.connectedComponents(
                (self.obstacleMap != 1).astype(np.uint8), connectivity=4)
            
            # Temporary static boundary testing array
            self.obstacles = np.array([
                [90, y] for y in range(66, 86)
            ])
            
            if self.get_parameter('enable_path_visualization').value:
                self._preview_full_maps()
            
        except Exception as e:
            self.get_logger().error(f"Matrix parsing failure: {e}")
            return TransitionCallbackReturn.FAILURE

        self._pub_cmd_vel = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)
        self._pub_goal_reached = self.create_lifecycle_publisher(Bool, 'mission_status/goal_reached', 10)
        # Latched, so a dashboard that connects later still receives the current state.
        self._pub_state = self.create_lifecycle_publisher(
            String, 'mission_status/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        self._pub_global_path = self.create_lifecycle_publisher(Path, 'global_path', 10)
        self._pub_trajectory = self.create_lifecycle_publisher(Path, 'driven_trajectory', 10)
        
        # --- Create Service Server ---
        self._srv_get_maps = self.create_service(GetMatrices, 'get_matrices', self.get_matrices_callback)
        
        self._sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self._sub_goal = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self._sub_cancel = self.create_subscription(Empty, 'cancel_goal', self.cancel_goal_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Handles the Inactive -> Active transition.
        Activates interfaces and ignites the high-frequency control loop.
        """
        self.get_logger().info("Lifecycle transition: ACTIVE. Listening for UI goals.")
        rate_hz = self.get_parameter('control_rate_hz').value
        
        self.goal_reached = False
        if self._pub_goal_reached and self._pub_goal_reached.is_activated:
            msg = Bool()
            msg.data = False
            self._pub_goal_reached.publish(msg)
            
        self._control_timer = self.create_timer(1.0 / rate_hz, self.control_loop_callback)
        result = super().on_activate(state)
        self._set_state('idle')
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Handles the Active -> Inactive transition.
        Suspends the control loop and transmits halt commands.
        """
        if self._control_timer:
            self.destroy_timer(self._control_timer)
            self._control_timer = None
        self._stop_robot()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Handles the Inactive -> Unconfigured transition.
        Deallocates UI topics and ROS 2 interfaces.
        """
        if self._sub_odom: self.destroy_subscription(self._sub_odom)
        if self._sub_goal: self.destroy_subscription(self._sub_goal)
        if self._sub_cancel: self.destroy_subscription(self._sub_cancel)
        if self._pub_cmd_vel: self.destroy_publisher(self._pub_cmd_vel)
        if self._pub_goal_reached: self.destroy_publisher(self._pub_goal_reached)
        if self._pub_state: self.destroy_publisher(self._pub_state)
        if self._pub_global_path: self.destroy_publisher(self._pub_global_path)
        if self._pub_trajectory: self.destroy_publisher(self._pub_trajectory)
        if self._srv_get_maps: self.destroy_service(self._srv_get_maps)
        plt.close('all')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Emergency node termination protocol."""
        if self._control_timer: self._control_timer.cancel()
        self._stop_robot()
        self.on_cleanup(state)
        plt.close('all')
        return TransitionCallbackReturn.SUCCESS


def main():
    """Application entry point."""
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