"""
ROS 2 lifecycle node of the path planner: the ROS wiring around

    mission          the mission logic: states and decisions, cycle by cycle
    global_planner   the route to the goal (fast_marching over planner_maps)
    local_planner    the velocity command along the route (DWA)
    lidar_obstacles  obstacles from the lidar, by height above the ground
    rover_state      attitude, Unreal session jumps, the driven trajectory

Inputs:  odom, goal_pose (PoseStamped), cancel_goal (Empty), the lidar cloud.
Outputs: cmd_vel; mission_status/state (latched JSON) and goal_reached;
         global_path and driven_trajectory (nav_msgs/Path); planner/obstacles;
         the get_matrices service (cost and elevation maps for the UIs).

Every control cycle the node asks the mission what to do (Mission.step) and
does it: plan (in a worker thread, so goals, cancels and odometry keep
arriving during a long search), drive (the local planner's command), or stop.
With lidar_obstacle_avoidance on (read every cycle: it can be switched live),
obstacles are avoided by the local planner, routed around by the global
planner, and a route a new one blocks is planned again.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, Header, String
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

from custom_map_interfaces.srv import GetMatrices

from src import debug_plots
from src import lidar_obstacles
from src import mission as mission_logic
from src import rover_state
from src.fast_marching import PlanningCancelled
from src.global_planner import Clearance, GlobalPlanner, Route
from src.local_planner import DWAConfig, LocalPlanner
from src.mission import Mission, MissionSettings
from src.planner_maps import load_planner_maps
from src.ros_conversions import finite_cost_map, matrix_msg, path_msg

# Suppress Matplotlib GUI warnings during live ROS 2 execution
warnings.filterwarnings("ignore")

# A Play/Stop in Unreal teleports the rover to its start pose: an odometry
# step longer than this (m) starts a new session.
SESSION_JUMP_M = 2.0


class LifecyclePlannerController(rclpy.lifecycle.Node):

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # --- ROS interfaces (created at configure) ---
        self._pub_cmd_vel: Optional[Publisher] = None
        self._pub_goal_reached: Optional[Publisher] = None
        self._pub_state: Optional[Publisher] = None
        self._pub_global_path: Optional[Publisher] = None
        self._pub_trajectory: Optional[Publisher] = None
        self._pub_obstacles: Optional[Publisher] = None
        self._sub_odom: Optional[Subscription] = None
        self._sub_goal: Optional[Subscription] = None
        self._sub_cancel: Optional[Subscription] = None
        self._sub_lidar: Optional[Subscription] = None
        self._srv_get_maps = None
        self._control_timer: Optional[Timer] = None

        # --- The parts (built at configure) ---
        self.maps = None
        self.mission: Optional[Mission] = None
        self.global_planner: Optional[GlobalPlanner] = None
        self.local_planner: Optional[LocalPlanner] = None

        # The mission and the obstacle cells are shared by the callbacks, the
        # control loop and the planning thread.
        self._lock = threading.Lock()
        self._plan_thread: Optional[threading.Thread] = None

        # --- The rover ---
        self._pose = None                   # (x, y, yaw), odom frame
        self._tilt_deg = 0.0
        self._last_cmd = (0.0, 0.0)         # (v, w) last commanded: the local planner's velocity state
        self._trajectory = []               # driven trajectory, [[x, y], ...]

        # --- Lidar obstacles: points remembered for the local planner, and the
        # map cells under them for the global planner (kept for the session:
        # rocks do not move) ---
        self._obstacle_memory = lidar_obstacles.ObstacleMemory()
        self._obstacle_cells = set()

        self.declare_parameters(
            namespace='',
            parameters=[
                # Maps
                ('elevation_map_path', 'maps/generated_elevationMap_40.csv'),
                ('soil_map_path',      'maps/generated_soilMap_1.csv'),
                ('global_res',         1.0),
                ('slope_threshold',    15.0),

                # Mission
                ('control_rate_hz',    10.0),
                ('max_tilt_deg',       15.0),
                ('stop_tolerance',     0.5),
                ('stall_window_s',     15.0),
                ('max_stalls',         3),

                # Local planner (DWA)
                ('lookahead_dist',     7.0),
                ('braking_distance',   3.0),
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
                ('robot_stuck_flag_cons', 0.001),

                # The rover
                ('robot_width', 0.65),
                ('robot_length', 0.65),
                ('robot_radius', 0.35),
                ('obstacle_safety_m', 0.15),

                # Lidar obstacle avoidance
                ('lidar_obstacle_avoidance', True),
                ('lidar_topic', 'lidar/points'),
                ('lidar_pitch_deg', -20.0),
                ('lidar_x_offset_m', 0.0),
                ('lidar_mount_from_tf', True),   # pitch and offset from TF (base_link -> lidar_frame)
                ('base_frame', 'base_link'),
                ('lidar_frame', 'lidar'),
                ('obstacle_min_height_m', 0.15),
                ('obstacle_max_range_m', 6.0),
                ('obstacle_memory_s', 20.0),
                ('obstacle_replan_lookahead_m', 5.0),
                ('obstacle_clearance_m', 1.5),

                # Debug
                ('enable_path_visualization', True),
            ])

        if self._param('enable_path_visualization'):
            debug_plots.interactive()

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _avoidance_on(self) -> bool:
        return bool(self._param('lidar_obstacle_avoidance'))

    # =========================================================================
    # Settings read live (the mission's and the obstacle clearances)
    # =========================================================================

    def _stop_tolerance(self) -> float:
        return max(self._param('robot_radius'), self._param('stop_tolerance'))

    def _mission_settings(self) -> MissionSettings:
        return MissionSettings(
            stop_tolerance=self._stop_tolerance(), max_tilt_deg=self._param('max_tilt_deg'),
            replan_lookahead_m=self._param('obstacle_replan_lookahead_m'),
            stall_window_s=self._param('stall_window_s'), max_stalls=self._param('max_stalls'))

    def _clearance(self) -> Clearance:
        radius = self._param('robot_radius')
        return Clearance(rover_m=radius, goal_m=radius + 1.0, margin_m=self._param('obstacle_clearance_m'))

    # =========================================================================
    # Inputs
    # =========================================================================

    def odom_callback(self, msg: Odometry) -> None:
        """The rover's pose, tilt and driven trajectory; a new session on a teleport."""
        q = msg.pose.pose.orientation
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        if rover_state.is_session_jump(self._pose, x, y, SESSION_JUMP_M):
            self.get_logger().info("Odometry jumped: treating it as an Unreal Play/Stop. "
                                   "Clearing the goal, the obstacles and the map overlays.")
            with self._lock:
                event = self.mission.new_session()
                self._obstacle_cells = set()
                self._trajectory = []
            self._obstacle_memory.clear()
            self._stop_robot()
            self._publish_clear()
            self._publish_event(event)

        self._pose = (x, y, rover_state.yaw_from_quaternion(q))
        self._tilt_deg = rover_state.tilt_deg(q)
        rover_state.record_point(self._trajectory, x, y)

    def lidar_callback(self, msg: PointCloud2) -> None:
        """Obstacles in a scan, by height: remembered, published, and their map cells recorded."""
        if not self._avoidance_on() or self._pose is None:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        settings = lidar_obstacles.DetectionSettings(
            pitch_deg=self._param('lidar_pitch_deg'), x_offset_m=self._param('lidar_x_offset_m'),
            min_height_m=self._param('obstacle_min_height_m'),
            max_range_m=self._param('obstacle_max_range_m'))
        found = lidar_obstacles.detect(np.asarray(points, dtype=float).reshape(-1, 3), settings)

        x, y, yaw = self._pose
        now = self.get_clock().now().nanoseconds * 1e-9
        memory = self._obstacle_memory
        memory.max_age_s = self._param('obstacle_memory_s')
        if len(found):
            odom = lidar_obstacles.to_odom(found, x, y, yaw)
            memory.add(odom, now)
            cells = lidar_obstacles.grid_cells(odom, self._param('global_res'), self.maps.obstacles.shape)
            with self._lock:
                self._obstacle_cells |= cells
        memory.prune(now, x, y)

        if self._pub_obstacles is not None and self._pub_obstacles.is_activated:
            xy = memory.points()
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id='odom'),
                np.column_stack((xy, np.zeros(len(xy)))).tolist())
            self._pub_obstacles.publish(cloud)

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """A new goal: stop and plan to it."""
        goal = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f"Waypoint update received: [{goal[0]:.2f}, {goal[1]:.2f}]. Suspending kinematics.")
        self._stop_robot()
        with self._lock:
            event = self.mission.set_goal(goal)
        self._publish_event(event)
        if self._pub_goal_reached and self._pub_goal_reached.is_activated:
            self._pub_goal_reached.publish(Bool(data=False))

    def cancel_goal_callback(self, msg: Empty) -> None:
        """Operator ALL STOP: drop the goal and halt (the control loop does not override it)."""
        with self._lock:
            event = self.mission.cancel()
        self._stop_robot()
        if event is not None:
            self._publish_event(event)
        # Clear the abandoned route from the UI; the driven trajectory is kept.
        if self._pub_global_path and self._pub_global_path.is_activated:
            self._pub_global_path.publish(path_msg([], self.get_clock().now().to_msg()))

    # =========================================================================
    # The control cycle
    # =========================================================================

    def control_loop_callback(self) -> None:
        """Asks the mission what to do this cycle, and does it."""
        if self._pose is None or self.mission is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self.mission.settings = self._mission_settings()
            self.global_planner.clearance = self._clearance()
            cells = frozenset(self._obstacle_cells) if self._avoidance_on() else frozenset()
            step = self.mission.step(self._pose, self._tilt_deg, now, cells)
            route, goal = self.mission.route, self.mission.goal

        if step.event is not None:
            self._publish_event(step.event)

        if step.action == mission_logic.PLAN:
            self._stop_robot()
            self._start_planning()
        elif step.action == mission_logic.STOP:
            self._stop_robot()
            if step.event is not None and step.event.state == mission_logic.REACHED:
                self._arrived(step.event.goal)
        elif step.action == mission_logic.DRIVE:
            self._drive(route, goal)

    def _drive(self, route: np.ndarray, goal) -> None:
        """One local planner command along the route."""
        obstacles = self._obstacle_memory.points() if self._avoidance_on() else np.empty((0, 2))
        try:
            cmd = self.local_planner.command(self._pose, self._last_cmd, route, goal, obstacles)
        except Exception as e:
            self.get_logger().error(f"Local planner fault: {e}")
            self._stop_robot()
            return
        twist = Twist()
        twist.linear.x, twist.angular.z = cmd.v, cmd.w
        self._pub_cmd_vel.publish(twist)
        self._last_cmd = (cmd.v, cmd.w)

        self._broadcast_paths(route)
        if self._param('enable_path_visualization'):
            self._update_live_plot(route, cmd.trajectory, goal, obstacles)

    def _arrived(self, goal) -> None:
        if self._pub_goal_reached and self._pub_goal_reached.is_activated:
            self._pub_goal_reached.publish(Bool(data=True))
        self._broadcast_paths(None)                    # the final resting position
        if self._param('enable_path_visualization'):
            self._update_live_plot(None, np.array([[*self._pose, 0.0, 0.0]]), goal, np.empty((0, 2)))

    def _stop_robot(self) -> None:
        self._last_cmd = (0.0, 0.0)
        if self._pub_cmd_vel and self._pub_cmd_vel.is_activated:
            self._pub_cmd_vel.publish(Twist())

    # =========================================================================
    # Planning (worker thread)
    # =========================================================================

    def _start_planning(self) -> None:
        """Plans the route to the mission's goal in a worker thread (one at a time)."""
        if self._plan_thread is not None and self._plan_thread.is_alive():
            return                              # a search is running; its result decides
        with self._lock:
            request = self.mission.planning_request()
            cells = frozenset(self._obstacle_cells) if self._avoidance_on() else frozenset()
        if request is None:
            return
        goal, generation = request
        rover_xy = (self._pose[0], self._pose[1])
        self._plan_thread = threading.Thread(
            target=self._plan_worker, args=(rover_xy, goal, generation, cells),
            name='global_planner', daemon=True)
        self._plan_thread.start()

    def _plan_worker(self, rover_xy, goal, generation: int, cells: frozenset) -> None:
        """Computes the route, then hands it to the mission (dropped if no longer wanted)."""
        search = self.global_planner.search
        search.should_stop = lambda: generation != self.mission.generation
        try:
            route = self.global_planner.plan(rover_xy, goal, cells)
        except PlanningCancelled:
            self.get_logger().info(f"Planning to {goal} stopped: the goal changed.")
            return
        except Exception as e:
            route = Route(False, reason='path planning error', message=f"Global path generation fault: {e}")
        finally:
            search.should_stop = None

        with self._lock:
            event = self.mission.route_planned(generation, route, cells)
        if event is None:
            self.get_logger().info(f"Discarding the route to {goal}: the goal changed while planning.")
            return
        self._publish_event(event)
        if route.ok:
            try:
                self._broadcast_paths(route.path)
            except Exception as e:
                self.get_logger().error(f"Publishing the new route failed: {e}")

    # =========================================================================
    # Outputs
    # =========================================================================

    def _publish_event(self, event: mission_logic.Event) -> None:
        """A mission state change: published on mission_status/state, and logged."""
        if self._pub_state is not None and self._pub_state.is_activated:
            self._pub_state.publish(String(data=mission_logic.state_json(event.state, event.goal, event.reason)))
        if event.message:
            self._log(event.level, event.message)

    def _log(self, level: str, message: str) -> None:
        # One call site per severity: rclpy refuses a call site whose severity changes.
        if level == 'error':
            self.get_logger().error(message)
        elif level == 'warn':
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def get_matrices_callback(self, request, response):
        """The cost and elevation maps, for a UI that asks for them."""
        self.get_logger().info("Map Service called. Packaging matrices for UI.")
        if self.maps is not None:
            response.cost_map = matrix_msg(finite_cost_map(self.maps.cost))
            response.elevation_map = matrix_msg(self.maps.elevation)
        return response

    def _publish_clear(self) -> None:
        """
        Empty route and trajectory, so the UIs wipe the previous run's overlays
        (_broadcast_paths skips an empty trajectory).
        """
        empty = path_msg([], self.get_clock().now().to_msg())
        if self._pub_global_path and self._pub_global_path.is_activated:
            self._pub_global_path.publish(empty)
        if self._pub_trajectory and self._pub_trajectory.is_activated:
            self._pub_trajectory.publish(empty)

    def _broadcast_paths(self, route) -> None:
        """The route (if any) and the driven trajectory, as nav_msgs/Path."""
        stamp = self.get_clock().now().to_msg()
        if route is not None and self._pub_global_path and self._pub_global_path.is_activated:
            self._pub_global_path.publish(path_msg(route, stamp))
        if self._trajectory and self._pub_trajectory and self._pub_trajectory.is_activated:
            self._pub_trajectory.publish(path_msg(self._trajectory, stamp))

    def _update_live_plot(self, route, predicted_trajectory, goal, obstacles) -> None:
        debug_plots.update_live_plot(
            self.local_planner.config, self._pose, self.maps.elevation, route,
            predicted_trajectory, self._trajectory, obstacles, goal)

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def _lidar_mount_from_tf(self) -> None:
        """
        The lidar's pitch and forward offset from TF (component_robot_description:
        the URDF exported from the Unreal Blueprint), written into lidar_pitch_deg
        and lidar_x_offset_m. Without TF within 5 s the YAML values stay.
        """
        base, lidar = self._param('base_frame'), self._param('lidar_frame')
        buffer = Buffer()
        listener = TransformListener(buffer, self, spin_thread=True)
        try:
            mount = buffer.lookup_transform(base, lidar, rclpy.time.Time(), timeout=Duration(seconds=5.0))
        except Exception as exc:
            self.get_logger().warn(f"Lidar mount not in TF ({base} -> {lidar}: {exc}); using the YAML values.")
            return
        finally:
            listener.unregister()
        q = mount.transform.rotation
        view_z = 2.0 * (q.x * q.z - q.w * q.y)          # z of the lidar's x axis in base_link
        pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, view_z))))     # negative = looking down
        x_offset = mount.transform.translation.x
        self.set_parameters([Parameter('lidar_pitch_deg', value=float(pitch_deg)),
                             Parameter('lidar_x_offset_m', value=float(x_offset))])
        self.get_logger().info(f"Lidar mount from TF ({base} -> {lidar}): pitch {pitch_deg:.1f} deg, "
                               f"{x_offset:.3f} m ahead of {base}.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Loads the maps, builds the planners and the mission, creates the interfaces."""
        self.get_logger().info("Lifecycle transition: CONFIGURED. Standing by for UI coordinates.")
        if self._param('lidar_mount_from_tf'):
            self._lidar_mount_from_tf()
        share = get_package_share_directory('component_path_planner')
        try:
            self.maps = load_planner_maps(
                os.path.join(share, self._param('elevation_map_path')),
                os.path.join(share, self._param('soil_map_path')),
                self._param('global_res'), self._param('slope_threshold'))
        except Exception as e:
            self.get_logger().error(f"Matrix parsing failure: {e}")
            return TransitionCallbackReturn.FAILURE
        if self._param('enable_path_visualization'):
            debug_plots.preview_maps(self.maps.elevation, self.maps.cost)

        self.global_planner = GlobalPlanner(self.maps, self._param('global_res'),
                                            self._param('slope_threshold'), self._clearance(),
                                            log=self.get_logger())
        self.local_planner = LocalPlanner(DWAConfig.from_params(self._param, self._stop_tolerance()))
        self.mission = Mission(self._mission_settings(), self.global_planner.blocks_route)

        self._pub_cmd_vel = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)
        self._pub_goal_reached = self.create_lifecycle_publisher(Bool, 'mission_status/goal_reached', 10)
        # Latched, so a dashboard that connects later still receives the current state.
        self._pub_state = self.create_lifecycle_publisher(
            String, 'mission_status/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._pub_global_path = self.create_lifecycle_publisher(Path, 'global_path', 10)
        self._pub_trajectory = self.create_lifecycle_publisher(Path, 'driven_trajectory', 10)
        self._pub_obstacles = self.create_lifecycle_publisher(PointCloud2, 'planner/obstacles', 10)
        self._srv_get_maps = self.create_service(GetMatrices, 'get_matrices', self.get_matrices_callback)

        self._sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self._sub_goal = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self._sub_cancel = self.create_subscription(Empty, 'cancel_goal', self.cancel_goal_callback, 10)
        self._sub_lidar = self.create_subscription(
            PointCloud2, self._param('lidar_topic'), self.lidar_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f"Lidar obstacle avoidance {'ON' if self._avoidance_on() else 'OFF'} "
            f"(obstacles: {self._param('obstacle_min_height_m'):g} m above the ground, "
            f"within {self._param('obstacle_max_range_m'):g} m).")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Starts the control loop and announces an idle mission."""
        self.get_logger().info("Lifecycle transition: ACTIVE. Listening for UI goals.")
        if self._pub_goal_reached and self._pub_goal_reached.is_activated:
            self._pub_goal_reached.publish(Bool(data=False))
        self._control_timer = self.create_timer(1.0 / self._param('control_rate_hz'), self.control_loop_callback)
        result = super().on_activate(state)
        self._publish_event(mission_logic.Event(mission_logic.IDLE, None))
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stops the control loop, drops any route being planned and halts the rover."""
        if self._control_timer:
            self.destroy_timer(self._control_timer)
            self._control_timer = None
        with self._lock:
            self.mission.invalidate()
        self._stop_robot()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Releases the topics, the service and the plot windows."""
        for sub in (self._sub_odom, self._sub_goal, self._sub_cancel, self._sub_lidar):
            if sub:
                self.destroy_subscription(sub)
        for pub in (self._pub_obstacles, self._pub_cmd_vel, self._pub_goal_reached, self._pub_state,
                    self._pub_global_path, self._pub_trajectory):
            if pub:
                self.destroy_publisher(pub)
        if self._srv_get_maps:
            self.destroy_service(self._srv_get_maps)
        debug_plots.close_all()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Halts the rover and releases everything."""
        if self._control_timer:
            self._control_timer.cancel()
        self._stop_robot()
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    node = LifecyclePlannerController('lc_component_path_planner')
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
