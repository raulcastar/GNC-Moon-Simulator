# Path planner

`component_path_planner` drives the rover to goals on `goal_pose` (map frame,
metres; the dashboard sends them). The node is ROS wiring around plain-Python
modules in `src/`:

| Module | Role |
|---|---|
| `mission.py` | The mission state machine, cycle by cycle (below). |
| `global_planner.py` | The route: fast marching over the cost map, around lidar obstacle cells kept at `obstacle_clearance_m`; moves a goal on impassable ground to the nearest drivable cell. |
| `fast_marching.py` | Eikonal travel-time search from the goal and route tracing down its gradient (from the DyMu planner, J. R. Sánchez Ibáñez and G. Paz Delgado). |
| `planner_maps.py` | Slope, cost and impassable cells from the elevation and soil maps (slope cost from DyMu). |
| `local_planner.py` | DWA velocity command along the route (from PythonRobotics): refuses trajectories that bring the footprint within `obstacle_safety_m` of an obstacle, prefers heading and speed; the final approach slows over the braking distance. |
| `lidar_obstacles.py` | Obstacles from each lidar scan: levelled, ground plane fitted, points `obstacle_min_height_m` above it; remembered `obstacle_memory_s`. |
| `rover_state.py` | Attitude from odometry, Play/Stop teleports, driven trajectory. |

## Mission states

`mission_status/state` (latched JSON): `idle`, `planning`, `driving`,
`reached`, `aborted` (with a reason), `cancelled`. Each control cycle
(`control_rate_hz`, 10 Hz):

1. no goal → idle;
2. body tilt over `max_tilt_deg` → abort;
3. no route → plan (in a worker thread; a new goal cancels a running search);
4. within `stop_tolerance` → reached;
5. no progress (0.25 m moved or 20° turned) for `stall_window_s` → plan again,
   abort after `max_stalls` in a row;
6. an obstacle found since the route was planned blocks the route within
   `obstacle_replan_lookahead_m` → plan again;
7. otherwise → drive.

Re-planning only reacts to obstacle cells that are new since the route was
planned: planning again for the same cells would give the same route and loop.
An odometry jump over 2 m (Unreal Play/Stop) clears the goal, the obstacles and
the overlays. `cancel_goal` (the dashboard's ALL STOP) halts and drops the goal.

## Lidar obstacle avoidance

`lidar_obstacle_avoidance` (on by default) can be switched live, from the
dashboard (⛰ AVOID) or with
`ros2 param set /lc_component_path_planner lidar_obstacle_avoidance false`.
Obstacles are published on `planner/obstacles` (odom frame).

## Maps

`elevation_map_path` and `soil_map_path` name CSVs in
`component_path_planner/maps/` (git-ignored; installed with the package, so
rebuild after adding one). They are Terrain Studio's `<name>_elevationMap.csv`
and `<name>_soilMap.csv` for the terrain in the level, at `global_res` metres
per cell; cells steeper than `slope_threshold` or with soil 0 are impassable.

## Outputs

`cmd_vel`, `global_path`, `driven_trajectory` (nav_msgs/Path),
`mission_status/goal_reached`, `mission_status/state`, `planner/obstacles` and
the `get_matrices` service (cost and elevation maps for the dashboard,
`custom_map_interfaces/GetMatrices`).
