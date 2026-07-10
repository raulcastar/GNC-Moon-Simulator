# GNC-Moon-Simulator

ROS 2 (Jazzy) guidance, navigation and control stack for a lunar rover simulated
in Unreal Engine. Sensor data crosses from Unreal into ROS over
[UnrealCV](https://unrealcv.org/); every component runs in its own container and
talks over Zenoh.

## Quick start

```bash
./docker_build.sh          # build the base image, then all components, then start
docker compose logs -f component_stereo_depth
```

`docker_build.sh` pulls `ros:jazzy`, so it needs network access. Once the images
exist, code changes land on a plain restart — each component mounts its source as
a volume and runs `colcon build` in its entrypoint:

```bash
docker compose restart component_stereo_depth
```

Three services drop to an interactive shell instead of launching automatically:
`component_local_mapping`, `mission_control_dashboard` and `debug_cli`. Bring
them up by hand:

```bash
docker compose exec component_local_mapping \
  ros2 launch component_local_mapping lc_component_local_mapping.launch.py
```

## Components

| Package | Role |
|---|---|
| `component_stereo_depth` | Owns the UnrealCV connection. Serves all three navcams and computes stereo depth. |
| `component_lidar` | Velodyne point cloud from Unreal. |
| `component_local_mapping` | Builds a local DEM and occupancy grid from the central navcam's depth. |
| `component_path_planner` | Global path over the elevation grid. |
| `component_rover_leo_raph` | Rover base: `/cmd_vel` in, `/odom` out. |
| `mission_control_dashboard` | Operator UI. |
| `custom_map_interfaces` | Shared message definitions. |

**Only one process may hold the UnrealCV socket.** The server accepts a single
client connection, so `component_stereo_depth` serves every camera itself. Stop
it before running anything else that talks to Unreal, including the scripts in
`utils/`.

## The navcam rig

Three cameras on one mount: a stereo pair either side of a central camera.
`component_stereo_depth` matches the pair with SGBM, then reprojects the result
onto the central camera's pixel grid, so depth, disparity, object mask and RGB
all overlay one another.

```
navcam/left/image_color              navcam/central/image_color
navcam/left/image_color/compressed   navcam/central/image_color/compressed
navcam/left/camera_info              navcam/central/camera_info
                                     navcam/central/image_depth
navcam/right/image_color             navcam/central/image_depth/visualization
navcam/right/image_color/compressed  navcam/central/disparity
navcam/right/camera_info             navcam/central/depth_gt
                                     navcam/central/object_mask
                                     navcam/central/object_mask_mapping
```

TF frame names flatten the namespace: `navcam_left`, `navcam_right`,
`navcam_central`.

`image_depth` is **planar Z depth in metres**, `NaN` where no surface was
recovered. It is a stereo product, with the occlusion holes and range-dependent
quantisation of one. `depth_gt` carries Unreal's exact depth buffer, converted to
the same units and frame, for scoring the stereo result — nothing in the sensor
path reads it.

### Rig requirements

SGBM assumes epipolar lines are image rows. That holds only when the two eyes
share a rotation and are offset purely along the left camera's right axis. At
configure time the node reads all three poses from Unreal inside one
`vset /action/game/pause` window, and checks:

- left and right rotations match, and the baseline is purely lateral
- the central camera lies on the line between the eyes, near its midpoint
- left and right FOVs agree

Violations are logged as errors; a central camera far off the rig aborts
configuration. To inspect the poses yourself, stop the node and run:

```bash
docker compose stop component_stereo_depth
python utils/list_unreal_cameras.py
```

It prints every camera's location, rotation, FOV and resolution, then evaluates
each ordered pair as a stereo candidate.

### Geometry that governs the result

With focal length `fx` (px) and baseline `B` (m):

| quantity | formula | at `fx·B = 64` |
|---|---|---|
| nearest resolvable surface | `fx·B / num_disparities` | 1.00 m at 64 disparities |
| depth resolution | `dZ = Z² / (fx·B)` | 14 cm at 3 m, 56 cm at 6 m |
| unmatched left border | `num_disparities` columns | 10% of a 640-px image |

`num_disparities` is the only knob here: the near limit, the border cost and the
matching time all follow from it. Raising it resolves nearer surfaces at the cost
of a wider border.

Anything nearer than the near limit has a disparity beyond the search window.
SGBM cannot find it and locks onto whatever correlates best inside the range,
producing a speckled band along the bottom of the depth image. At startup the
node measures how much of the view this affects and logs the `num_disparities`
that would resolve it.

`fill_left_border` recovers most of the dead band: OpenCV blanks every column
`u < num_disparities`, but a pixel at column `u` with disparity `d` is matchable
whenever `d ≤ u`, and distant surfaces have small `d`.

### Glare study

`saturation_level` marks clipped pixels in the central image. With
`validate_against_ground_truth`, every frame logs the matching hole rate and
median/p95 `|dZ|/Z` **inside versus outside** the saturated region:

```
saturated 1.20% | glare: holes 61.3% med|dZ|/Z 4.11% p95 22.7%
                | clean: holes  9.8% med|dZ|/Z 0.72% p95  3.1%
```

Lock the cameras' exposure in Unreal before trusting these numbers. If the eyes
auto-expose independently they diverge most when a bright source is in one field
of view, and the resulting photometric mismatch looks exactly like a glare
effect.

## Local mapping

`component_local_mapping` subscribes to `navcam/central/{image_depth,
image_color, camera_info}` and publishes:

- `local_mapping/elevation_map_raw` — `Float32MultiArray`, metres
- `local_mapping/elevation_grid` — `OccupancyGrid` in `odom`
- `local_mapping/pointcloud` — `PointCloud2` in `odom`

Intrinsics come from `camera_info` rather than YAML, so changing a camera's FOV
in Unreal cannot silently stretch the DEM in range. `camera_height_m` and
`camera_angle` still describe the physical mount and must match reality: they set
where the ground plane lands.

## Utilities

Run these with `component_stereo_depth` stopped — it holds the UnrealCV socket.

| Script | Purpose |
|---|---|
| `utils/list_unreal_cameras.py` | Enumerate camera poses; flag usable stereo pairs. |
| `utils/stereo_depth_test.py` | One-shot SGBM + glare comparison, plotted with matplotlib. |
| `utils/connect_unrealcv.py` | Minimal capture and display. |

## ⚙️ WSL2 System Configuration (`.wslconfig`)

Here is the exact file path:
C:\Users\<YourUsername>\.wslconfig

(Make sure to replace <YourUsername> with your actual Windows login name).

```ini
[wsl2]
# Merges the WSL network with the Windows network
networkingMode=mirrored

# Ensures localhost ports are shared between Windows and WSL
localhostForwarding=true

# (Optional) Forces WSL to use the exact same DNS as Windows
dnsTunneling=true
```
