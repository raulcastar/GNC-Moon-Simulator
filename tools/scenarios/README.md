# Scenario checks

Checks that need the whole system running: Unreal in Play and the containers
up. Unlike the unit tests (`./run_tests.sh`), they drive the rover and read
live topics, so run them deliberately and watch the simulation.

| Command | What it checks | Runs in |
|---|---|---|
| `tools/scenarios/run.sh traverse <x> <y> [s]` | Sends a goal (map cells) and logs the drive every 0.5 s: pose, roll/pitch, commands, mission state, lidar obstacles in the rover's lane. Stops on arrival, planner abort, an obstacle under 1.5 m ahead, no progress for 25 s, or the time limit; always cancels the goal and stops the rover. | `component_path_planner` |
| `tools/scenarios/run.sh overlay [s] [period]` | Every period, compares the lidar points (as the dashboard draws them on the local elevation map) with the stereo DEM: height differences and obstacle agreement, plus a picture per snapshot. Drive the rover meanwhile (e.g. with `traverse` in another terminal). | `mission_control_dashboard` |
| `tools/scenarios/run.sh lidar_gt [s]` | Projects one lidar scan into the central navcam and compares it with Unreal's ground-truth depth; reports the scan rate, coverage and depth error. Needs the lidar to follow the central navcam. | `component_stereo_depth` |

Output (logs, overlay pictures, `stats.json`) goes to `tools/scenarios/results/`,
which git ignores.
