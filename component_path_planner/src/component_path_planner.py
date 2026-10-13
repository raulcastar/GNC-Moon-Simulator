"""
ROS 2 Lifecycle Node: Global Path Planning & DWA Local Trajectory Control

A managed lifecycle node for autonomous rover navigation on procedural lunar
terrain: a global travel-time search over the cost map (global_path_planner),
followed by a Dynamic Window Approach (DWA) local controller
(local_trajectory_controller).

- Goals arrive on goal_pose (PoseStamped), cancels on cancel_goal.
- The mission state is latched on mission_status/state.
- The route and the driven trajectory are published for the UIs, and the maps
  are served on the get_matrices service.
- With lidar_obstacle_avoidance on, obstacles seen by the lidar (by height
  above the local ground, lidar_obstacles) are avoided by the DWA, published on
  planner/obstacles, and marked in the global map: a route running into one is
  planned again around it.

This file is the ROS wiring: parameters, lifecycle, callbacks, the planning
thread and the control loop. The parts:
    mission           goals, routes, final approach, attitude (logic)
    lidar_obstacles   obstacles from the lidar, their memory and map cells (logic)
    planner_maps      map loading, cost/obstacle maps, drivable regions
    ros_conversions   Path and Float32MultiArray messages
    debug_plots       optional matplotlib windows
"""

import math
import os
import threading
import warnings
from typing import Optional

import numpy as np
from ament_index_python.packages import get_package_share_directory

import rclpy
import rclpy.lifecycle
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, Header, String
from rclpy.qos import qos_profile_sensor_data

from custom_map_interfaces.srv import GetMatrices

from src import debug_plots
from src import global_path_planner
from src import lidar_obstacles
from src import local_trajectory_controller
from src import mission
from src.planner_maps import load_planner_maps
from src.ros_conversions import finite_cost_map, matrix_msg, path_msg

# Suppress Matplotlib GUI warnings during live ROS 2 execution
warnings.filterwarnings("ignore")


class LifecyclePlannerController(rclpy.lifecycle.Node):
    """
    Lifecycle node managing the rover's routing and kinematics: it loads the
    maps, follows odometry, takes goals, plans routes in a worker thread,
    drives them with the DWA and serves the maps to the UIs.
    """

    def __init__(self, node_name: str, **kwargs):
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
        self.path_planner = global_path_planner.Global_Path_Planner()
        self.path_planner.log = self.get_logger()

        # Global planning runs in a worker thread, so goals, cancels and odometry
        # keep arriving while it searches. _plan_generation changes whenever the
        # wanted plan does (new goal, cancel, new session, deactivation): a plan
        # finished for an older generation is dropped, and a running search
        # stops early (Global_Path_Planner.should_stop).
        self._plan_lock = threading.Lock()
        self._plan_generation = 0
        self._plan_thread: Optional[threading.Thread] = None
        self.local_planner = local_trajectory_controller.Local_Trajectory_Controller()

        # DWA configuration, built from the parameters at configure time
        self.robot_conf = None
        self._base_max_speed = 0.0

        # --- Internal State Tracking ---
        self.robot_pose = None      # Vector: [x, y, yaw, linear_vel, angular_vel]
        # Last commanded (v, omega), used as the DWA velocity state
        self._last_cmd = (0.0, 0.0)
        self.current_goal = None    # Vector: [x, y]
        self.global_path = None
        self.obstacles = np.empty((0, 2))     # Obstacles the DWA avoids (odom, metres)
        # Lidar obstacles: remembered points (odom frame) and the global-map
        # cells they block (col, row). Rocks do not move, so cells stay blocked
        # until a new session.
        self._obstacle_memory = lidar_obstacles.ObstacleMemory()
        self._blocked_cells = set()
        # The blocked cells the current route was planned with. Only cells
        # found since then can make the route ahead blocked: planning again
        # with the same cells gives the same route (which may clip a known
        # cell's corner), so re-planning for them would loop.
        self._route_known_cells = set()
        # No progress while driving (stalled beside a rock, stuck on terrain):
        # re-plan, and after max_stalls in a row abort instead of waiting forever.
        self._progress = mission.ProgressWatch()
        self._sub_lidar: Optional[Subscription] = None
        self._pub_obstacles: Optional[Publisher] = None
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
                ('robot_width', 0.65),
                ('robot_length', 0.65),
                ('robot_radius', 0.35),
                ('obstacle_safety_m', 0.15),

                # Lidar obstacle avoidance (read every cycle: can be switched live)
                ('lidar_obstacle_avoidance', True),
                ('lidar_topic', 'lidar/points'),
                ('lidar_pitch_deg', -20.0),
                ('lidar_x_offset_m', 0.0),
                ('obstacle_min_height_m', 0.15),
                ('obstacle_max_range_m', 6.0),
                ('obstacle_memory_s', 20.0),
                ('obstacle_replan_lookahead_m', 5.0),
                ('obstacle_clearance_m', 1.5),

                # No-progress watchdog
                ('stall_window_s', 15.0),
                ('max_stalls', 3),
            ])

        if self._param('enable_path_visualization'):
            debug_plots.interactive()

    def _param(self, name: str):
        return self.get_parameter(name).value

    # =========================================================================
    # KINEMATICS & SENSORY CALLBACKS
    # =========================================================================

    def odom_callback(self, msg: Odometry) -> None:
        """Follows the rover: pose, velocities, tilt and the driven trajectory."""
        # Odom carries a REP-103 heading (the rover node bakes in the Unreal-frame
        # correction), so the quaternion yaw is the heading.
        q = msg.pose.pose.orientation
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = mission.yaw_from_quaternion(q)
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # New session (Unreal Play/Stop): the rover teleports to its start pose.
        # Clear the previous run's trajectory, goal and path, and push empty
        # paths so the map clears too.
        if mission.is_session_jump(self.robot_pose, x, y, self._session_reset_dist):
            jump = math.hypot(x - self.robot_pose[0], y - self.robot_pose[1])
            self.get_logger().info(
                f"Odometry jumped {jump:.1f} m — treating as an Unreal "
                f"Play/Stop. Clearing trajectory, goal and map overlays.")
            with self._plan_lock:
                self._plan_generation += 1
                self.trajectory_history = []
                self.current_goal = None
                self.global_path = None
                self.goal_reached = False
                self._blocked_cells = set()
            self._obstacle_memory.clear()
            self._progress.reset()
            self._stop_robot()
            self._publish_clear()
            self._set_state('idle', 'new session')

        self.robot_pose = np.array([x, y, yaw, v, omega])
        self._tilt_deg = mission.tilt_deg(q)
        mission.record_point(self.trajectory_history, x, y)

    def _lidar_enabled(self) -> bool:
        return bool(self._param('lidar_obstacle_avoidance'))

    def lidar_callback(self, msg: PointCloud2) -> None:
        """Obstacles in a scan, by height: remembered, published, and their cells blocked."""
        if not self._lidar_enabled() or self.robot_pose is None:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        settings = lidar_obstacles.DetectionSettings(
            pitch_deg=self._param('lidar_pitch_deg'), x_offset_m=self._param('lidar_x_offset_m'),
            min_height_m=self._param('obstacle_min_height_m'),
            max_range_m=self._param('obstacle_max_range_m'))
        found = lidar_obstacles.detect(np.asarray(points, dtype=float).reshape(-1, 3), settings)
        x, y, yaw = self.robot_pose[:3]
        now = self.get_clock().now().nanoseconds * 1e-9
        memory = self._obstacle_memory
        memory.max_age_s = self._param('obstacle_memory_s')
        if len(found):
            odom = lidar_obstacles.to_odom(found, x, y, yaw)
            memory.add(odom, now)
            if self.obstacleMap is not None:
                cells = lidar_obstacles.grid_cells(odom, self._param('global_res'), self.obstacleMap.shape)
                with self._plan_lock:
                    self._blocked_cells |= cells
        memory.prune(now, x, y)
        if self._pub_obstacles is not None and self._pub_obstacles.is_activated:
            xy = memory.points()
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id='odom'),
                np.column_stack((xy, np.zeros(len(xy)))).tolist())
            self._pub_obstacles.publish(cloud)

    def _cell_radii(self):
        """(map resolution, rover clear radius, goal clear radius) in cells: see route_blocking."""
        res = self._param('global_res')
        radius = self.robot_conf.robot_radius
        return res, radius / res, (radius + 1.0) / res

    def _route_cells(self, rover_xy, goal) -> set:
        """The lidar cells a route may not cross (lidar_obstacles.route_blocking), rover at rover_xy (m)."""
        res, rover_clear, goal_clear = self._cell_radii()
        with self._plan_lock:
            cells = set(self._blocked_cells)
        return lidar_obstacles.route_blocking(
            cells, (rover_xy[0] / res, rover_xy[1] / res),
            None if goal is None else (goal[0] / res, goal[1] / res), rover_clear, goal_clear)

    def _planning_maps(self, rover_xy, goal: list, cells: set, clearance: bool = True):
        """
        The cost and obstacle maps for planning: the lidar's obstacle cells
        impassable (the same cells the route-ahead check uses, so a new route
        is never blocked at once), with a clearance margin around them unless
        clearance is False. None when there are no lidar obstacles to add.
        """
        if not self._lidar_enabled() or not cells:
            return None
        res, rover_clear, goal_clear = self._cell_radii()
        margin = self._param('obstacle_clearance_m') / res if clearance else 0.0
        blocked = lidar_obstacles.planning_blocked(
            cells, (rover_xy[0] / res, rover_xy[1] / res), (goal[0] / res, goal[1] / res),
            rover_clear, goal_clear, margin)
        return lidar_obstacles.mark_blocked(self.costMap, self.obstacleMap, blocked,
                                            2 * self._param('slope_threshold'))

    def _set_state(self, state: str, reason: str = '') -> None:
        """Publishes the mission state on mission_status/state (mission.state_json)."""
        if self._pub_state is None or not self._pub_state.is_activated:
            return
        self._pub_state.publish(String(data=mission.state_json(state, self.current_goal, reason)))

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """A new goal: stop, drop the current route and plan to the new goal."""
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y

        self.get_logger().info(f"Waypoint update received: [{new_x:.2f}, {new_y:.2f}]. Suspending kinematics.")

        # Enforce zero-velocity state to prevent drifting during computational blocking
        self._stop_robot()

        # Reset tracking states to trigger the routing subroutines
        with self._plan_lock:
            self._plan_generation += 1
            self.current_goal = [new_x, new_y]
            self.global_path = None
            self.goal_reached = False
        self._progress.reset()
        self._set_state('planning')

        # Invalidate the previous mission completion state across the ROS 2 network
        if self._pub_goal_reached and self._pub_goal_reached.is_activated:
            self._pub_goal_reached.publish(Bool(data=False))

    def cancel_goal_callback(self, msg: Empty) -> None:
        """
        Aborts the active goal on an operator ALL STOP and halts the rover, so
        the control loop does not override the stop on its next cycle.
        """
        with self._plan_lock:
            self._plan_generation += 1
            if self.current_goal is not None:
                self.get_logger().warn(f"Goal {self.current_goal} cancelled by the operator.")
                self._set_state('cancelled', 'by the operator (ALL STOP or manual drive)')
            self.current_goal = None
            self.global_path = None
        self._stop_robot()

        # Clear the abandoned route from the UI; the driven trajectory is kept.
        if self._pub_global_path and self._pub_global_path.is_activated:
            self._pub_global_path.publish(path_msg([], self.get_clock().now().to_msg()))

    # =========================================================================
    # CORE CONTROL LOOP & PATHING
    # =========================================================================

    def _start_planning(self, start_pos: list) -> None:
        """
        Plans the route to the current goal in a worker thread (one at a time).
        The control loop keeps the rover stopped until the route is applied.
        """
        if self._plan_thread is not None and self._plan_thread.is_alive():
            return                              # a search is running; its result decides
        with self._plan_lock:
            if self.current_goal is None:
                return
            goal, generation = list(self.current_goal), self._plan_generation
        self._plan_thread = threading.Thread(
            target=self._plan_worker, args=(list(start_pos), goal, generation),
            name='global_planner', daemon=True)
        self._plan_thread.start()

    def _plan_worker(self, start_pos: list, goal: list, generation: int) -> None:
        """Computes the route (worker thread), then applies it if still wanted."""
        planner = self.path_planner
        planner.should_stop = lambda: generation != self._plan_generation
        rover_xy = (float(self.robot_pose[0]), float(self.robot_pose[1]))
        with self._plan_lock:
            known = set(self._blocked_cells)        # what this route is planned with
        try:
            plan = lambda maps: mission.plan_route(
                planner, *maps, self.freeRegions, start_pos, goal,
                self._param('slope_threshold'), self.get_logger().info)
            maps = self._planning_maps(rover_xy, goal, known)
            if maps is None:
                outcome = plan((self.costMap, self.obstacleMap))
            else:
                outcome = plan(maps)
                if outcome[0] != 'driving':
                    # No route keeps the clearance from the rocks: try without it.
                    self.get_logger().info("No route keeps the obstacle clearance; planning without it.")
                    outcome = plan(self._planning_maps(rover_xy, goal, known, clearance=False))
        except global_path_planner.PlanningCancelled:
            self.get_logger().info(f"Planning to {goal} stopped: the goal changed.")
            return
        except Exception as e:
            outcome = ('aborted', None, None, 'path planning error',
                       f"Global path generation fault: {e}")
        finally:
            planner.should_stop = None
        state, path, new_goal, reason, message = outcome
        with self._plan_lock:
            if generation != self._plan_generation:
                self.get_logger().info(f"Discarding the route to {goal}: the goal changed while planning.")
                return
            # One logging call per severity: rclpy refuses a call site whose
            # severity changes between calls.
            if state == 'driving':
                self.get_logger().info(message)
                self.current_goal = new_goal
                self.global_path = path
                self._route_known_cells = known
            else:
                self.get_logger().warn(message)
                self.current_goal = None
                self.global_path = None
            self._set_state(state, reason)
        if state == 'driving':
            try:
                self._broadcast_paths()
            except Exception as e:
                self.get_logger().error(f"Publishing the new route failed: {e}")

    def control_loop_callback(self) -> None:
        """
        The DWA control cycle: tilt safety, planning when there is no route,
        arrival, the final approach and the velocity command.
        """
        # Suspend execution if spatial context is unavailable OR if we have no active mission
        if self.robot_pose is None or self.current_goal is None:
            return

        # Tilt safety: stop and abort the goal on excessive body tilt.
        max_tilt = self._param('max_tilt_deg')
        if self._tilt_deg > max_tilt:
            self.get_logger().error(
                f"Tilt safety stop: body tilt {self._tilt_deg:.1f} deg exceeds "
                f"{max_tilt:.1f} deg. Goal {self.current_goal} aborted.")
            with self._plan_lock:
                self._plan_generation += 1      # a route being planned must not restart the drive
                self._set_state('aborted', f'tilt {self._tilt_deg:.1f} deg > {max_tilt:.1f} deg')
                self.current_goal = None
                self.global_path = None
            self._stop_robot()
            return

        # Halt kinematics and defer to the global planner if no valid route exists
        # (planned in a worker thread: this callback returns at once).
        if self.global_path is None:
            self._stop_robot()
            self._progress.pause()
            self._start_planning([int(round(self.robot_pose[0])), int(round(self.robot_pose[1]))])
            return

        # State Latch: Prevent odometry drift from re-triggering the controller post-arrival
        if self.goal_reached:
            self._stop_robot()
            return

        dist_to_goal = math.hypot(self.robot_pose[0] - self.current_goal[0],
                                  self.robot_pose[1] - self.current_goal[1])
        tolerance = mission.stop_tolerance(self.robot_conf.robot_radius, self._param('stop_tolerance'))

        if dist_to_goal <= tolerance:
            self.get_logger().info(f"Target {self.current_goal} achieved. Engaging parking brake.")
            self._set_state('reached')
            self.goal_reached = True

            if self._pub_goal_reached and self._pub_goal_reached.is_activated:
                self._pub_goal_reached.publish(Bool(data=True))

            self._stop_robot()
            self._broadcast_paths()  # Ensure UI reflects the final resting position

            if self._param('enable_path_visualization'):
                self._update_live_plot(np.array([self.robot_pose]), self.current_goal)

            self.current_goal = None
            return

        # No progress for a while (stalled beside a rock, stuck on terrain):
        # plan again; after max_stalls in a row, give the goal up.
        self._progress.window_s = self._param('stall_window_s')
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._progress.update(now, *self.robot_pose[:3]):
            stalls, limit = self._progress.stalls, self._param('max_stalls')
            if stalls >= limit:
                self.get_logger().warn(
                    f"No progress towards {self.current_goal} after {stalls} attempts: goal aborted.")
                with self._plan_lock:
                    self._plan_generation += 1
                    self._set_state('aborted', f'no progress ({stalls} stalls): blocked near an obstacle?')
                    self.current_goal = None
                    self.global_path = None
                self._stop_robot()
                return
            self.get_logger().warn(
                f"No progress for {self._progress.window_s:.0f} s ({stalls}/{limit}): planning again.")
            with self._plan_lock:
                self.global_path = None
            self._set_state('planning', 'no progress')
            self._stop_robot()
            return

        # A lidar obstacle on the route ahead: plan the route again around it.
        if self._lidar_enabled():
            with self._plan_lock:
                known = self._route_known_cells
            cells = self._route_cells(self.robot_pose[:2], self.current_goal) - known
            if lidar_obstacles.path_blocked(self.global_path, cells, self.robot_pose[:2],
                                            self._param('obstacle_replan_lookahead_m'),
                                            self._param('global_res')):
                self.get_logger().info("Obstacle on the route ahead (lidar): planning around it.")
                with self._plan_lock:
                    self.global_path = None
                self._set_state('planning', 'obstacle on the route')
                self._stop_robot()
                return
            self.obstacles = self._obstacle_memory.points()
        else:
            self.obstacles = np.empty((0, 2))

        # Slow down over the braking distance so the rover does not overshoot.
        braking_distance = self._param('braking_distance')
        self.robot_conf.max_speed = mission.approach_speed(self._base_max_speed, dist_to_goal,
                                                           braking_distance)

        try:
            local_goal = self.local_planner.get_lookahead_point(
                self.robot_pose, self.global_path, self._param('lookahead_dist'))

            state = np.array([*self.robot_pose[:3], *self._last_cmd])
            cmd_vel_arr, predicted_trajectory = self.local_planner.dwa_control(
                state, self.robot_conf, np.array(local_goal), self.obstacles
            )
            heading_error = math.atan2(self.current_goal[1] - self.robot_pose[1],
                                       self.current_goal[0] - self.robot_pose[0]) - self.robot_pose[2]
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            cmd_vel_arr[1] = mission.approach_yaw_rate(cmd_vel_arr[1], dist_to_goal,
                                                       braking_distance, tolerance, heading_error)

            twist_msg = Twist()
            twist_msg.linear.x = float(cmd_vel_arr[0])
            twist_msg.angular.z = float(cmd_vel_arr[1])
            self._pub_cmd_vel.publish(twist_msg)
            self._last_cmd = (twist_msg.linear.x, twist_msg.angular.z)

            # Periodically stream the actual driven trajectory to the UI
            self._broadcast_paths()
            if self._param('enable_path_visualization'):
                self._update_live_plot(predicted_trajectory, self.current_goal)

        except Exception as e:
            self.get_logger().error(f"DWA algorithmic fault: {e}")
            self._stop_robot()

    def _stop_robot(self) -> None:
        """Transmits absolute zero-velocity limits to the hardware/simulation actuators."""
        self._last_cmd = (0.0, 0.0)
        if self._pub_cmd_vel and self._pub_cmd_vel.is_activated:
            self._pub_cmd_vel.publish(Twist())

    # =========================================================================
    # SERVICE HANDLERS & NETWORK BROADCASTING
    # =========================================================================

    def get_matrices_callback(self, request, response):
        """The cost and elevation maps, for a UI that asks for them."""
        self.get_logger().info("Map Service called. Packaging matrices for UI.")
        if self.costMap is not None:
            response.cost_map = matrix_msg(finite_cost_map(self.costMap))
        if self.elevationMap is not None:
            response.elevation_map = matrix_msg(self.elevationMap)
        return response

    def _publish_clear(self) -> None:
        """
        Push empty Global Path and Trajectory messages so the UI wipes the
        previous run's overlays. _broadcast_paths skips an empty trajectory, so
        on a session reset the stale trail would otherwise never be cleared.
        """
        empty = path_msg([], self.get_clock().now().to_msg())
        if self._pub_global_path and self._pub_global_path.is_activated:
            self._pub_global_path.publish(empty)
        if self._pub_trajectory and self._pub_trajectory.is_activated:
            self._pub_trajectory.publish(empty)

    def _broadcast_paths(self) -> None:
        """Publishes the planned route and the driven trajectory as nav_msgs/Path."""
        timestamp = self.get_clock().now().to_msg()
        if self._pub_global_path and self._pub_global_path.is_activated and self.global_path is not None:
            self._pub_global_path.publish(path_msg(self.global_path, timestamp))
        if self._pub_trajectory and self._pub_trajectory.is_activated and len(self.trajectory_history) > 0:
            self._pub_trajectory.publish(path_msg(self.trajectory_history, timestamp))

    def _update_live_plot(self, predicted_trajectory: np.ndarray, goal: list) -> None:
        debug_plots.update_live_plot(
            self.local_planner, self.robot_conf, self.robot_pose, self.elevationMap,
            self.global_path, predicted_trajectory, self.trajectory_history, self.obstacles, goal)

    # =========================================================================
    # ROS 2 LIFECYCLE MANAGEMENT
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Reads the parameters, loads the maps and creates the interfaces."""
        self.get_logger().info("Lifecycle transition: CONFIGURED. Standing by for UI coordinates.")

        # --- Enforce Idle Start State ---
        self.current_goal = None

        self.robot_conf = mission.robot_config(self._param)
        self._base_max_speed = self.robot_conf.max_speed

        pkg_share_dir = get_package_share_directory('component_path_planner')
        try:
            maps = load_planner_maps(
                self.path_planner,
                os.path.join(pkg_share_dir, self._param('elevation_map_path')),
                os.path.join(pkg_share_dir, self._param('soil_map_path')),
                self._param('global_res'), self._param('slope_threshold'))
            self.elevationMap, self.costMap = maps.elevation, maps.cost
            self.obstacleMap, self.freeRegions = maps.obstacles, maps.free_regions
            self.obstacles = np.empty((0, 2))

            if self._param('enable_path_visualization'):
                self.get_logger().info("Rendering topological matrix alignment preview...")
                debug_plots.preview_maps(self.elevationMap, self.costMap, self.current_goal)

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

        self._srv_get_maps = self.create_service(GetMatrices, 'get_matrices', self.get_matrices_callback)

        self._sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self._sub_goal = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self._sub_cancel = self.create_subscription(Empty, 'cancel_goal', self.cancel_goal_callback, 10)
        self._sub_lidar = self.create_subscription(
            PointCloud2, self._param('lidar_topic'), self.lidar_callback, qos_profile_sensor_data)
        self._pub_obstacles = self.create_lifecycle_publisher(PointCloud2, 'planner/obstacles', 10)
        self.get_logger().info(
            f"Lidar obstacle avoidance {'ON' if self._lidar_enabled() else 'OFF'} "
            f"(obstacles: {self._param('obstacle_min_height_m'):g} m above the ground, "
            f"within {self._param('obstacle_max_range_m'):g} m).")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Starts the control loop and announces an idle mission."""
        self.get_logger().info("Lifecycle transition: ACTIVE. Listening for UI goals.")
        rate_hz = self._param('control_rate_hz')

        self.goal_reached = False
        if self._pub_goal_reached and self._pub_goal_reached.is_activated:
            self._pub_goal_reached.publish(Bool(data=False))

        self._control_timer = self.create_timer(1.0 / rate_hz, self.control_loop_callback)
        result = super().on_activate(state)
        self._set_state('idle')
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stops the control loop, drops any route being planned and halts the rover."""
        if self._control_timer:
            self.destroy_timer(self._control_timer)
            self._control_timer = None
        with self._plan_lock:
            self._plan_generation += 1          # a running search stops, its result is dropped
        self._stop_robot()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Releases the topics, the service and the plot windows."""
        if self._sub_odom: self.destroy_subscription(self._sub_odom)
        if self._sub_goal: self.destroy_subscription(self._sub_goal)
        if self._sub_cancel: self.destroy_subscription(self._sub_cancel)
        if self._sub_lidar: self.destroy_subscription(self._sub_lidar)
        if self._pub_obstacles: self.destroy_publisher(self._pub_obstacles)
        if self._pub_cmd_vel: self.destroy_publisher(self._pub_cmd_vel)
        if self._pub_goal_reached: self.destroy_publisher(self._pub_goal_reached)
        if self._pub_state: self.destroy_publisher(self._pub_state)
        if self._pub_global_path: self.destroy_publisher(self._pub_global_path)
        if self._pub_trajectory: self.destroy_publisher(self._pub_trajectory)
        if self._srv_get_maps: self.destroy_service(self._srv_get_maps)
        debug_plots.close_all()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Emergency node termination protocol."""
        if self._control_timer: self._control_timer.cancel()
        self._stop_robot()
        self.on_cleanup(state)
        debug_plots.close_all()
        return TransitionCallbackReturn.SUCCESS


def main():
    """Application entry point."""
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LifecyclePlannerController('lc_component_path_planner')
    executor.add_node(lc_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
