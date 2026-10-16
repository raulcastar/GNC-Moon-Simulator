# Testing

## Unit tests

Unit tests run inside each package's container, on the mounted source (start
the stack first):

```bash
./run_tests.sh                          # all packages
./run_tests.sh component_path_planner   # one package (extra args go to pytest)
py -m pytest mcp                        # the MCP server, on the host
```

Covered: `component_path_planner`, `component_stereo_depth`, `component_lidar`,
`component_rover_leo_raph`, `component_imu`, `component_robot_description` and
`mission_control_dashboard`. The tests stub Unreal and the ROS graph: they never
reach a running simulator.

## Scenarios (Unreal in Play)

Checks that need the simulation running are in `tools/scenarios/` (see
[its README](../tools/scenarios/README.md)): a traverse to a goal with a drive
log, lidar against the stereo DEM, and lidar against Unreal's ground-truth
depth.

## Commanding the rover by hand

Use the dashboard, its JSON API or the MCP tools ([Mission
control](mission-control.md#json-api-and-ai-agents-mcp)), or publish directly
from a container (`docker compose exec debug_cli bash`):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 300.0, y: 350.0}}}"
ros2 topic pub --once /headlights_intensity std_msgs/msg/Float32 "{data: 0.3}"
```

A single `/cmd_vel` message keeps the rover driving: the rover node holds the
last command. Send a zero twist to stop it, or use the dashboard.
