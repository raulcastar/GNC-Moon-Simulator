# GNC-Moon-Simulator

ROS 2 (Jazzy) guidance, navigation and control stack for a lunar rover (Leo
"Raph") simulated in Unreal Engine 5. Sensor data crosses from Unreal into ROS
over dedicated Unreal plugins (stereo navcams, lidar, event and SPAD cameras,
simulation clock) and a TCP link for the rover itself; every component runs in
its own container and talks over Zenoh. The terrain, rocks, ground-truth masks
and sun come from Terrain Studio, in the sibling repository
`Unreal-Moon-Terrain-Generation`.

## Quick start

```bash
./docker_build.sh          # build the base image, then all components, then start
docker compose logs -f component_stereo_depth
```

Then press Play in Unreal: the plugins and the rover dial the containers on
`127.0.0.1`, and the dashboard menu is at **http://localhost:5000**.

`docker_build.sh` pulls `ros:jazzy`, so it needs network access. Each component
mounts its source as a volume and builds it once, in the entrypoint all
components share (`docker/entrypoint.sh`, baked into the base image; a
component's Dockerfile only adds its dependencies and sets `GNC_PACKAGE`, the
package to launch). To keep a container restart (and a WSL/Docker reboot) from
re-running `colcon build` — a storm of simultaneous builds that can destabilise
the WSL VM — the entrypoint **skips the build when the workspace is already
built**. So a plain restart is near-instant but does **not** pick up code
changes. To apply source changes, either rebuild inside the running container
and restart it, or recreate it (a fresh container has no build yet):

```bash
# rebuild one package in place, then restart it:
docker compose exec component_path_planner bash -lc \
  'cd /opt/ros/jazzy/src && source /opt/ros/jazzy/setup.bash && colcon build --packages-select component_path_planner'
docker compose restart component_path_planner

# or recreate the container (rebuilds its whole workspace):
docker compose up -d --force-recreate component_path_planner
```

(The entrypoint also honours `FORCE_BUILD=1` in the container's environment.)

Restarting `component_rover_leo_raph` drops the rover's TCP link, and Unreal
only dials it when Play begins: **press Play again** after restarting it.

`debug_cli` is an interactive shell with the ROS 2 tools (rviz2, rqt) on the
host display, instead of a component:

```bash
docker compose exec debug_cli bash
```

Only `component_path_planner` and `component_local_mapping` get the host display,
for their optional matplotlib debug windows (`enable_path_visualization` and
`debug_mode` in their `config/params.yaml`); the other nodes are headless.

## Components

| Package | Role |
|---|---|
| `component_rover_leo_raph` | Rover base: `/cmd_vel` in (ramped to acceleration limits, converted to skid-steer wheel speeds), `/odom` and TF `odom → base_link` out. See [The rover](#the-rover). |
| `component_robot_description` | The rover's fixed frames as static TF (`robot_state_publisher`): `urdf/<GNC_ROVER>.urdf` (`leo_raph`, `yutu`), exported from the Unreal Blueprint. See [TF tree](#tf-tree). |
| `component_sim_clock` | Publishes `/clock` (simulation time) from the UnrealSimClock plugin. |
| `component_stereo_depth` | Receives the three navcams from the UnrealStereoCam plugin and computes SGBM stereo depth on the central camera; also Unreal's ground-truth depth, the object and shadow masks, and extra mono cameras (the rear camera, `rear_cam/image_color`). |
| `component_lidar` | Lidar point cloud from the UnrealLidarCam plugin (`lidar/points`, organized, frame `lidar`). |
| `component_imu` | Simulated IMU (`imu/data`) from `/odom`: attitude, body rates and specific force (gravity included). |
| `component_event_camera` | Event (DVS) camera from the UnrealEventCam plugin: an event-frame visualisation (`event_camera/image_events`) and the event rate. |
| `component_spad_camera` | Single-photon (SPAD) camera from the UnrealSPADCam plugin, through a port of spadSim's photon model: binary frames (`spad/binary`) and a digitised reconstruction (`spad/image`). |
| `component_local_mapping` | Local DEM, occupancy grid and point cloud from the central navcam's stereo depth. |
| `component_path_planner` | Mission logic, global route (fast marching over the elevation and soil maps) and local planner (DWA), with lidar obstacle avoidance and re-planning. See [Path planner](#path-planner). |
| `mission_control_dashboard` | Web mission control. See [Mission control](#mission-control). |
| `custom_map_interfaces` | Shared interfaces: the `GetMatrices` service (the planner's cost and elevation maps). |

## The simulator (Unreal)

The Unreal project (`Moon_Simulator`, Unreal 5.7) holds the level, the rover
Blueprint `Leo_Raph` and the sensor plugins in `unreal/`, copied into
`<Project>/Plugins/` (each has a README with its setup):

| Plugin | Role |
|---|---|
| `UnrealSimClock` | The simulation time (`GncSimTime.h`) streamed to `/clock`; Blueprint nodes *Get Sim Time Ns String* / *Seconds* |
| `UnrealStereoCam` | Navcam captures (left, right, centre, mask, ground-truth depth, rear camera) streamed as frames |
| `UnrealLidarCam` | Ray-traced lidar with a configurable aperture |
| `UnrealEventCam` | Event camera: brightness-change events from rendered frames |
| `UnrealSPADCam` | Intensity frames for the SPAD model |

The project also uses the `TCP-Unreal` plugin for the rover link, and editor
Python (*Python Editor Script Plugin*, *Remote Execution* on) for Terrain
Studio and the [viewport cameras](#viewport-cameras).

### Project settings

`Config/DefaultEngine.ini` settings the simulation depends on:

| Setting | Value | Why |
|---|---|---|
| `bUseFixedFrameRate` / `FixedFrameRate` | True / 30 | Every frame advances game time by 1/30 s, so physics, `/clock` and the sensors stay consistent even when the editor renders slower than real time (the simulation then runs slower than the wall clock, never inconsistently). |
| `bSubstepping`, `MaxSubstepDeltaTime`, `MaxSubsteps` | True, 0.008333, 8 | Physics at 120 Hz in 4 substeps per frame. Without substepping, `MaxPhysicsDeltaTime` 0.033 alone ran physics at ~0.43× the clock at 13 fps. |
| `bEnableEnhancedDeterminism`, `RestitutionCombineMode` | True, Min | Repeatable runs; wheels do not bounce off the ground. |
| `DefaultGravityZ` | -980 (Earth) | See [Gravity](#gravity). |
| `+StartupScripts` | `rover_cameras_startup.py` | The viewport cameras follow the dashboard. |

All sensor plugins gate their rates on game time (`GetWorld()->GetTimeSeconds()`),
so their rates are per simulated second, like `/clock`.

## Simulation time

Everything runs on Unreal's clock: one simulation time for the whole run —
Unreal's game time, so it stops when Play is paused and follows time dilation —
in nanoseconds since the Unix epoch, never running backwards across Play
sessions. Every sensor plugin (stereo cameras, lidar, event and SPAD cameras)
stamps its data with it at capture through its own copy of `GncSimTime.h`, so
the plugins stay independent of each other. The **UnrealSimClock** plugin
(`unreal/UnrealSimClock/`, no Blueprint setup: a world subsystem) streams that
time to `component_sim_clock`, which publishes `/clock`. The rover Blueprint
puts the same time in its telemetry (`t_ns`, from *Get Sim Time Ns String*), so
odometry is stamped with the moment Unreal measured the pose.

The GNC nodes run with `use_sim_time: true` (in each `config/params.yaml`), so a
message's stamp is when the sensor captured it in the simulation, and topics can
be matched by stamp: `component_local_mapping` places each depth frame at the
rover pose **at its capture time**, interpolated from odometry, rather than at
the newest pose (0.5 s of processing at 0.5 m/s would otherwise misplace it by
25 cm). The rover node's acceleration ramp and the planner's control loop run
on it too. When Unreal is not playing, simulation time stands still and the
timers of those nodes pause with it. The dashboard stays on the wall clock (it
is a UI), and its rosbags record `/clock`, so a replay runs on the recorded
time: the **BAGS** window (port 5007) plays a bag back into the live system —
stop Play in Unreal first — with the sensors replayed and the GNC nodes
recomputing their outputs from them.

## Talking to Unreal

Independent links, and they fail differently. In every one **Unreal dials and
the container listens**, on `127.0.0.1`:

| Link | Port | Carries |
|---|---|---|
| Rover (`TCP-Unreal` in the Blueprint) | TCP 3003 | wheel commands and headlights out, pose telemetry in |
| UnrealStereoCam | TCP 3004 | navcam frames, object mask, ground-truth depth, rear camera |
| UnrealEventCam | TCP 3005 | event packets |
| UnrealSPADCam | TCP 3006 | intensity frames |
| UnrealLidarCam | TCP 3007 | lidar scans |
| UnrealSimClock | TCP 3008 | simulation time → `/clock` |

The plugins redial when their node is not there; the rover Blueprint connects
once, at BeginPlay.

The containers run `network_mode: host`, so they sit on WSL2's network stack
while Unreal runs on Windows. Crossing that boundary is where links go wrong,
and **only `127.0.0.1` works** — the host's LAN address is ambiguous under
mirrored networking and silently drops traffic.

A wildcard is worse: `0.0.0.0` is a bind address, not a destination. Linux
rewrites a datagram sent to it into one sent to `127.0.0.1` (the sender's own
loopback), and Windows rejects it outright. Anything that sends to `0.0.0.0` is
talking to itself. The rover node's legacy UDP path (`transport: udp`,
`sending_ip` / `sending_port` / `listen_port`) is entirely subject to this; TCP
sidesteps it by never needing a destination at all.

### The rover link

`transport: tcp` (the default) is **one bidirectional TCP connection** on
`tcp_port` (3003), carrying wheel commands out and pose back. Neither side needs
the other's address: replies go back the way they came.

**Unreal dials, the node listens** (`tcp_role: server`). Set `tcp_role: client`
for the reverse, if the Unreal component is switched to listen instead.

Unreal delimits nothing, so the reader **scans for JSON objects rather than for a
separator**. Bare, split across reads, coalesced, or newline-delimited all parse,
as do Unreal's locale numbers (`1,205`) even when a read boundary lands inside
one.

- **Out:** `{"FL", "BL", "FR", "BR", "headlights_on", "headlights_intensity"}`,
  the wheel inputs (negative = forward in the Blueprint) and the lights.
  `headlights_intensity` is 0–1 of the Blueprint's own light intensity
  (`HeadLight_L` / `HeadLight_R`, 20 000 cd each: brighter burnt out up to 44 % of the navcam's ground view); the node takes it from the
  `headlights_intensity` topic (`std_msgs/Float32`, clamped) or the parameter
  of that name at start, and the dashboard has a slider for it (overview and
  teleop). On/off stays on `headlights_cmd`. The Blueprint reads it with *Get
  Field* (Json Blueprint Utilities), which treats an exact 0 as a missing field,
  so 0 % goes out as `1e-9` (dark).
- **In:** the world transform of the Blueprint's `base_link` component
  (`x, y, z` cm, `roll, pitch, yaw` degrees), optionally the physics body's
  velocities (`vx, vy, vz` cm/s, `wx, wy, wz` rad/s) and `t_ns`.

Unreal **connects once, at BeginPlay, and does not redial**: restarting
`component_rover_leo_raph` drops the listener, so Unreal must be re-played. The
node handles the reverse fine — it keeps listening and re-accepts across a
Play/Stop cycle. Every new connection is a new Play session: the node resets
its acceleration ramp (a ramp cut short by Stop would otherwise spin the wheels
while the rover lands at spawn, and roll it over) and sends the headlight state.

If the rover does not move, check that the game is not **paused**: that
freezes physics, so no command has any effect.

## Two rovers: Leo_Raph and Yutu

The stack drives either rover; one runs at a time (they use the same ports):

| | Leo_Raph | Yutu |
|---|---|---|
| Unreal level | `Lunar_Terrain_Raph` | `Lunar_Terrain_Yutu` |
| Blueprint | `/Game/Blueprints/Leo_Raph` | `/Game/Blueprints/Yutu_Rover` |
| Meshes, materials | `/Game/RoverAssets/Leo_Raph` | `/Game/RoverAssets/Yutu` |
| Wheels | 4, 0.18 m | 6, 0.34 m (rigid chassis, no suspension yet) |
| Size, mass | ~0.5 m, 210 kg | 1.85 x 1.9 m with the solar wings out, 1.2 m tall, 140 kg |
| Navcams, lidar | 0.55 m up, on the body | 1.13 m up, on the mast |
| `GNC_ROVER` | `leo_raph` (default) | `yutu` |

**To switch from the dashboard:** the menu (port 5000) has a **ROVER** selector.
Open the rover's level in Unreal and pick the rover there; no container restarts.
The rover stops first.
The selector:

- applies the rover node's drive calibration live (Unreal's TCP link stays up);
- sets the planner's footprint and limits;
- gives `robot_state_publisher` the rover's URDF, so `/tf_static` changes;
- sets the dashboard's own values;
- re-configures the planner and local mapping, which read their sensor mounts
  from TF. A goal in progress is dropped.

The values come from the same config files, mounted read-only into the
dashboard's container (`mission_control_dashboard/src/rovers.py`). Next to the
selector, a line shows the rover Unreal is playing (reported by the viewport
camera link, `rover_cameras.py`), in amber when it differs. The overview's header
shows the active rover and the same check (`Unreal: Yutu ✓`, or amber `≠`). The choice lasts until the containers restart; then `.env`
decides again.

**To switch for good:** open the rover's level in Unreal; set `GNC_ROVER` in `.env` (repo
root); `docker compose up -d` (recreates, and rebuilds, the four containers that
use it: rover node, planner, robot description, dashboard); press Play.

Each of those launch files loads its `config/params.yaml` and then, when it
exists, `config/params_<GNC_ROVER>.yaml` with the rover's overrides; the robot
description loads `urdf/<GNC_ROVER>.urdf`. Yutu's overrides:

- **rover node:** `wheel_speed_scale` 0.96 and `rover_width` 330 (calibrated:
  0.2 m/s -> 0.198, 0.2 rad/s -> 0.197-0.199).
- **planner:** footprint 1.15 x 1.55 m, `robot_radius` 0.8, `lidar_self_radius_m`
  1.0 (the mast lidar sees the wings and arm), and `max_yaw_rate_deg` 8: spot turns
  rock the rigid six-wheel chassis (pitch/roll swing 2-4 deg at 0.1 rad/s, 5-8 at
  0.15, 8-10 at 0.2, 18-24 at 0.3). A rocker-bogie suspension would allow faster
  turns.
- **dashboard:** `lidar_height_m` 1.13, `lidar_self_radius_m` 1.0, `rover_width_m`
  1.11 (the map icon), `teleop_max_angular` 0.15 (the teleop turn slider's
  limit; Leo_Raph 0.35).

**How Yutu was built** (to repeat for another model): the glTF was prepared in
Blender (`Content/RoverAssets/Yutu/Source/prepare_yutu.py` in the Unreal project: turned to
face +X, scaled x2 to real size, the six wheels separated with their pivots on
their axles), imported to `/Game/RoverAssets/Yutu`, and `Yutu_Rover` made from a **copy of
Leo_Raph**, so it keeps its graph (TCP link, telemetry, lights, sensors) and
component names; the meshes, wheel and joint positions, masses, sensor mounts and
collision (a chassis box, sphere wheels) were changed, and two middle wheels and
joints added and driven like the front ones. Its URDF comes from the same
exporter (`export_rover_urdf.py`, run with the Yutu level open). For its wheel
tracks and rover-shadow tag, send the masks from Terrain Studio once with the
level open (Find lists the level's cameras, and the mask camera defaults to its
rover's `Navcam_Mask`).

**Credit (CC-BY-4.0, required):** This work is based on "Yutu"
(https://sketchfab.com/3d-models/yutu-d9564c9a60c14b85a880cc0bfcf1a6b5) by tashtego
(https://sketchfab.com/tashtego) licensed under CC-BY-4.0
(http://creativecommons.org/licenses/by/4.0/). The original `license.txt` and a
`CREDITS.txt` (with the changes made) are in the Unreal project's `Content/RoverAssets/Yutu/`,
and every Yutu asset carries the author, license and source as metadata.

## The rover

`component_rover_leo_raph` turns `/cmd_vel` into wheel inputs and Unreal's
telemetry into `/odom`.

- **Commands:** a velocity step is ramped to `max_linear_accel` /
  `max_angular_accel` (and the decel limits) per simulated second, then split
  into skid-steer side speeds, `v ∓ ω · rover_width / 2`, times
  `wheel_speed_scale` (m/s → Unreal wheel units). The ramp completes even if
  `/cmd_vel` stops arriving.
- **Odometry:** `pose_from_telemetry` converts the Blueprint's `base_link`
  transform from Unreal (left-handed, cm, degrees) to REP-103 (x forward, y
  left, z up, metres), through the rotation's axes, so roll, pitch and heading
  are exact at any attitude. The twist comes from the physics body when the
  Blueprint sends it (marked by a non-zero twist covariance). Each message is
  also broadcast as TF `odom → base_link` (`publish_tf`).

`wheel_speed_scale` and `rover_width` (the *effective* skid-steer track: slip
makes it wider than the physical 38 cm) are calibrated on level ground; the
comments in `config/params.yaml` give the measurements. Recalibrate after
changing gravity, wheel friction, damping or the drive strength: drive at 0.2 m/s
and spot-turn at ±0.3 rad/s, and compare commanded with driven.

### Gravity

The simulation runs under **Earth gravity** (`DefaultGravityZ` -980 cm/s²): the
rover is much easier to control than under the Moon's -162. What changed
together with it, and must change together again if gravity changes:

| | Earth (now) | Moon |
|---|---|---|
| Unreal `DefaultGravityZ` | -980 | -162 |
| Wheel drive strength (`*_Constraint` twist velocity drive) | 60000 | 10000 |
| Rover accel / decel (`component_rover_leo_raph`) | 0.4 / 0.4 m/s², 1.0 / 1.5 rad/s² | 0.1 m/s² |
| `rover_width` | 89 cm | 96 cm |
| Planner `max_accel`, `v_resolution`, `max_yaw_rate_deg` | 0.4, 0.02, 12 | 0.1, 0.005, 8 |
| IMU `gravity` | 9.81 | 1.62 |
| Teleop turn cap (default / max) | 0.20 / 0.35 rad/s | 0.10 / 0.15 |

Notes from tuning the rover's physics (`Leo_Raph` Blueprint):

- **Damping:** linear damping 0.05 on every part and angular 0.05 on the wheels
  (body angular stays 2.0). Damping 2.0 acted as a speed-proportional drag the
  wheel motors fought, which flipped the light body at ~0.2 m/s on the Moon.
- **Centre of mass:** the body's COM is nudged to (-2.4, -1.2, -18.2) cm: over
  the wheel centre and 12 cm high, so the static tip-over angle is ~58°
  (it was 24°). Mass is unchanged: 150 kg body, 10 kg per wheel.
- **Drive strength:** the wheel drives are velocity servos whose torque is
  strength × speed error, so the strength sets how far the rover slows under
  load. It scales with the rover's weight: at 10000 under Earth gravity the
  rover stalled on 11° slopes and in tight arcs beside rocks.
- `wheel_speed_scale` 1.82 holds under both gravities (the damping change moved
  it, not gravity).
- **Spot turns scrub** (isotropic wheel friction) and rock the rover. Under
  Earth gravity the pitch swings ~2° at 0.1 rad/s, ~4.5° at 0.2, 7–9° at 0.35
  and 13–15° at 0.5; under the Moon's it hopped above 0.15 rad/s. Hence the
  planner's 12°/s and the teleop cap.
- Unreal's Python can set Blueprint component properties but not graph nodes,
  and the placed rover keeps its own copy of physics values: set both the
  Blueprint and the level instance, with Play stopped. Changing a simulating
  part's properties during Play rebuilds its physics and breaks the wheels.

## TF tree

```
odom ──► base_link                       (component_rover_leo_raph, every odometry message)
          ├── base_footprint              on the ground under base_link
          ├── wheel_{fl,fr,rl,rr}_link    wheel centres
          ├── navcam_{left,central,right}_link ──► navcam_{left,central,right}   (optical)
          ├── lidar                       at the component it follows (the central navcam)
          ├── rear_cam_link ──► rear_cam
          ├── event_camera_link ──► event_camera
          └── spad_link ──► spad
```

`base_link` is the odometry point: the Blueprint's `base_link` component,
centred between the wheels at axle height, x forward, z up. Its world transform
is what the telemetry sends. Everything below it is static, from
`component_robot_description/urdf/leo_raph.urdf`, written by
`component_robot_description/unreal/export_rover_urdf.py` run in the Unreal
editor (`exec(open(r"<repo>/component_robot_description/unreal/export_rover_urdf.py").read())`).
**Re-run it after moving a sensor or a wheel in the Blueprint**, then rebuild
`component_robot_description`. Camera `*_link` frames have x along the view;
the optical frames (z along the view, x right, y down) are the frame ids the
images are stamped with.

`component_local_mapping` (camera height, offset and pitch) and
`component_path_planner` (lidar pitch and offset) read their sensor mounts from
TF at configure (`mount_from_tf`, `lidar_mount_from_tf`); the YAML values are
the fallback when TF does not answer within 5 s.

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
navcam/right/camera_info             navcam/central/depth_ground_truth
                                     navcam/central/depth_ground_truth/visualization
                                     navcam/central/object_mask
                                     navcam/central/object_mask_mapping
                                     navcam/central/object_mask_classes
                                     navcam/central/shadow_mask
                                     navcam/central/shadow_label
                                     navcam/central/shadow_label_mapping
```

`image_depth` is **planar Z depth in metres**, `NaN` where no surface was
recovered. It is a stereo product, with the occlusion holes and range-dependent
quantisation of one. `depth_ground_truth` is Unreal's exact depth for the same
view (`+Inf` for the sky).

The object mask's classes (id, name, colour, group) are in
`component_stereo_depth/config/mask_classes.json`, a copy of Terrain Studio's
`config/mask_classes.json`, which bakes them into the Unreal mask material:
**edit both together** (Terrain Studio's ground-truth page reports when they
differ). The node publishes them as JSON on `object_mask_mapping` and
`object_mask_classes`; the dashboard draws its legends from them. The shadow
feeds come from flag bits in the same mask frame (see Terrain Studio's README).

**The rear camera has its masks too:** `rear_cam/object_mask` (+ `_mapping`,
`_classes`), `rear_cam/shadow_mask` and `rear_cam/shadow_label` (+ `_mapping`),
from `RearCam_Mask` in the Blueprint (a copy of `Navcam_Mask`'s render settings
at `RearCam`'s pose), streamed by `RearCam_Stream` on cam id 13. The option is
the fourth field of its `extra_cameras` entry (`"10:rear_cam:90:13"`): drop it to
publish the colour only. Each mask is paired with the rear image of the same
Unreal frame, so both carry the same stamp. `enable_object_mask` /
`enable_shadow_mask` switch them with the central ones. The dashboard shows them
as REAR SEGMENTATION and REAR SHADOW MASK (camera wall; hidden by default in the
overview's VIEWS), and the FULL PERCEPTION bag preset records them.

### How frames arrive

The **UnrealStereoCam** plugin (`unreal/UnrealStereoCam/` — build
instructions in its README) renders each navcam **with the game frame** on the
render thread, reads it back asynchronously, and pushes it to the node over TCP
port 3004 — no render-on-demand, no game-thread stall. `component_stereo_depth`
runs a TCP server there and processes frames as they land.

Every capture enqueued in one Unreal tick carries the same frame counter, and
the node pairs the left/right eyes **only when their ids match exactly** — that
is the simultaneity guarantee, with no pausing and no round-trip probing. The
centre frame is taken from the nearest tick; it is overlaid, not matched, so a
frame or two of slack is fine.

### Rig requirements

SGBM assumes epipolar lines are image rows. That holds only when the two eyes
share a rotation and are offset purely along the left camera's right axis, so the
UnrealStereoCam captures must be **co-rotated and offset laterally** on the
mount, with the central capture between them.

A stream cannot be measured, so the geometry is **declared from parameters** in
`component_stereo_depth/config/params.yaml`: `baseline_m` (eye separation),
`hfov_deg` (the captures' FOV) and `center_offset_m` (the centre's position along
the baseline). The image size is taken from the first frames, and the node logs
the rig it solved at configure time:

```
Rig declared (stream mode): 640x480 | baseline 20.00 cm | hfov 90.00 deg
-> fx 320.00 px | centre 10.00 cm from the left eye
```

Set these to the real capture geometry — wrong values shift absolute depth.

### Geometry that governs the result

With focal length `fx` (px) and baseline `B` (m):

| quantity | formula | at `fx·B = 64` |
|---|---|---|
| nearest resolvable surface | `fx·B / num_disparities` | 0.50 m at 128 disparities |
| depth resolution | `dZ = Z² / (fx·B)` | 14 cm at 3 m, 56 cm at 6 m |
| unmatched left border | `num_disparities` columns | 20% of a 640-px image at 128 |

`num_disparities` (128) is the only knob here: the near limit, the border cost
and the matching time all follow from it. Raising it resolves nearer surfaces at
the cost of a wider border.

Anything nearer than the near limit has a disparity beyond the search window.
SGBM cannot find it and locks onto whatever correlates best inside the range,
producing a speckled band along the bottom of the depth image. Depth nearer than
the near limit is rejected, and the node logs that limit at configure time; raise
`num_disparities` if the bottom rows drop out.

`fill_left_border` recovers most of the dead band: OpenCV blanks every column
`u < num_disparities`, but a pixel at column `u` with disparity `d` is matchable
whenever `d ≤ u`, and distant surfaces have small `d`.

Exposure matters for matching: if the two eyes auto-expose independently they
diverge most when a bright source is in one field of view, and the photometric
mismatch looks exactly like a glare effect. Clamp or lock the captures' exposure
in Unreal — see the UnrealStereoCam README's **Exposure** section (Min/Max
EV100, or a full manual lock).

## Local mapping

`component_local_mapping` subscribes to `navcam/central/{image_depth,
image_color, camera_info}` and `/odom`, and publishes:

- `local_mapping/elevation_map_raw` — `Float32MultiArray`, metres
- `local_mapping/elevation_grid` — `OccupancyGrid` in `odom`
- `local_mapping/pointcloud` — `PointCloud2` in `odom`

Each depth frame is projected to the ground frame with the camera mount (from
TF) and the rover's roll and pitch at the frame's capture time
(`apply_rover_tilt`), then binned at `map_resolution` (5 cm) within
`distance_limit` (3 m). A cell takes its second-highest point
(`drop_highest`), so single stereo outliers do not spike it, and needs
`min_points_per_cell` points. Intrinsics come from `camera_info` rather than
YAML, so changing a camera's FOV in Unreal cannot silently stretch the DEM in
range.

## Path planner

`component_path_planner` drives the rover to goals on `goal_pose` (map frame,
metres; the dashboard sends them). The node is ROS wiring around plain-Python
modules in `src/`:

| Module | Role |
|---|---|
| `mission.py` | The mission state machine, cycle by cycle (below). |
| `global_planner.py` | The route: fast marching over the cost map, around lidar obstacle cells kept at `obstacle_clearance_m`; moves a goal on impassable ground to the nearest drivable cell. |
| `fast_marching.py` | Eikonal travel-time search from the goal and route tracing down its gradient (from the DyMu planner, J. R. Sánchez Ibáñez and G. Paz Delgado). |
| `planner_maps.py` | Slope, cost and impassable cells from the elevation and soil maps. |
| `local_planner.py` | DWA velocity command along the route (from PythonRobotics): refuses trajectories that bring the footprint within `obstacle_safety_m` of an obstacle, prefers heading and speed; final approach slows over the braking distance. |
| `lidar_obstacles.py` | Obstacles from each lidar scan: levelled, ground plane fitted, points `obstacle_min_height_m` above it; remembered `obstacle_memory_s`. |
| `rover_state.py` | Attitude from odometry, Play/Stop teleports, driven trajectory. |

**Mission states** (`mission_status/state`, latched JSON): `idle`, `planning`,
`driving`, `reached`, `aborted` (with a reason), `cancelled`. Each control cycle
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

**Lidar obstacle avoidance** (`lidar_obstacle_avoidance`, on by default) can be
switched live — from the dashboard (⛰ AVOID) or
`ros2 param set /lc_component_path_planner lidar_obstacle_avoidance false`.
Obstacles are published on `planner/obstacles` (odom frame).

**Maps:** `elevation_map_path` and `soil_map_path` name CSVs in
`component_path_planner/maps/` (git-ignored; installed with the package, so
rebuild after adding one). They are Terrain Studio's `<name>_elevationMap.csv`
and `<name>_soilMap.csv` for the terrain in the level, at `global_res` metres
per cell; cells steeper than `slope_threshold` or with soil 0 are impassable.

Outputs: `cmd_vel`, `global_path`, `driven_trajectory` (nav_msgs/Path),
`mission_status/goal_reached`, `planner/obstacles` and the `get_matrices`
service (cost and elevation maps for the dashboard).

## Mission control

`mission_control_dashboard`: one ROS node behind several web servers (Flask +
SocketIO, wall clock):

| Port | Screen |
|---|---|
| 5000 | Menu linking to the others |
| 5001 | **Overview:** navcam feeds with the route, goal, re-plans and obstacles drawn on them; lidar; local DEM; the global map with click-to-goal, the route, re-plans, obstacles and the driven path (FOLLOW ROVER zooms in on the rover); telemetry; mission timeline; `/rosout` console; lights, ALL STOP, 🎥 VIEW (Unreal's viewport camera), ⛰ AVOID, snapshot and record |
| 5002 | Teleop: drive-by-wire, bypassing the planner, with a deadman (a node-side watchdog stops the rover if commands stop) |
| 5003 | Camera wall with per-feed toggles |
| 5005 | 3D map: lidar and local DEM side by side |
| 5006 | System: feed rates, lifecycle nodes (restart), remote parameters, ROS 2 topics |
| 5007 | Bags: record with presets, names and notes; replay into the live system; rename and delete |

Bags are written to `mission_control_dashboard/bags/` (git-ignored).

### Viewport cameras

`unreal/python/rover_cameras.py` sets Unreal's Play viewport to the rover's
follow camera (the Blueprint's `FollowArm` spring arm, a three-quarter rear view
that keeps the wheel tracks in frame), an **orbit** around the rover, or a
**top-down** view above it. The orbit and top-down views move a level actor
tagged `RoverOrbitCamera` every frame; they only read the rover.

Install: copy `rover_cameras.py` and `rover_cameras_startup.py` to
`<Project>/Content/Python/`, add `rover_cameras_startup.py` to *Project Settings
→ Python → Startup Scripts*, and run `rover_cameras.install()` once with Play
stopped (it places the camera actor; save the level). The startup script polls
the dashboard's `GET /viewport_camera` (port 5001), so the overview's 🎥 VIEW
button cycles follow → orbit → top in every Play session. From the editor's
Python console: `rc.orbit(radius=6, height=2.5, period=45)`, `rc.top(height=15)`,
`rc.follow()` (`import rover_cameras as rc`).

## AI agents (MCP)

`mcp/gnc_mcp.py` is an MCP server that lets an AI agent (Claude Code, Claude
Desktop or any MCP client) use the running simulator:

- **Read:** status, log, camera images, the rover.
- **Drive:** goals (and waiting for them), timed manual drives, stop.
- **Settings and maintenance:** headlights, obstacle avoidance, viewport
  camera, rover switch, node restarts, rosbag recording.

It wraps the dashboard's JSON API (`mission_control_dashboard/src/api.py`,
`http://localhost:5001/api/...`), which scripts can also call directly:

```bash
py -m pip install -r mcp/requirements.txt         # once, on the host
claude mcp add gnc -- py "<repo>/mcp/gnc_mcp.py"  # or use the repo's .mcp.json
curl localhost:5001/api/status                    # the API without MCP
```

Commands that move the rover are refused while Unreal is not in Play. Manual
drives are clamped to the teleop limits and always end stopped. Terrain Studio
has its own server for building terrain and starting or stopping Play.
`AGENTS.md` has guidance for coding agents working in this repo.

## Tests

Unit tests run inside each package's container, on the mounted source (start the
stack first):

```bash
./run_tests.sh                          # all packages
./run_tests.sh component_path_planner   # one package (extra args go to pytest)
```

Covered: `component_path_planner`, `component_stereo_depth`, `component_lidar`,
`component_rover_leo_raph`, `component_imu`, `component_robot_description` and
`mission_control_dashboard`.

Checks that need the simulation running (Unreal in Play) are in
`tools/scenarios/` (see its README): a traverse to a goal with a drive log,
lidar against the stereo DEM, and lidar against Unreal's ground-truth depth.

To command the rover by hand, use the dashboard, its JSON API or the MCP tools
(see [AI agents (MCP)](#ai-agents-mcp)). Or publish directly from a container
(`docker compose exec debug_cli bash`):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 300.0, y: 350.0}}}"
ros2 topic pub --once /headlights_intensity std_msgs/msg/Float32 "{data: 0.3}"
```

A single `/cmd_vel` message keeps the rover driving: the rover node holds the
last command. Send a zero twist to stop it, or use the dashboard.

## ⚙️ WSL2 System Configuration (`.wslconfig`)

WSL reads its settings only from your Windows profile, `%UserProfile%\.wslconfig`.
Put this there (or merge it into an existing one), then run `wsl --shutdown` for
it to take effect:

```ini
[wsl2]
# Required. Windows and WSL share one network, so Unreal on Windows reaches
# the containers on 127.0.0.1 (and the browser reaches the dashboard).
networkingMode=mirrored

# Resource caps. Every component container runs `colcon build` on its first
# start, so a full `docker compose up` runs ~10 builds at once; an unbounded
# WSL VM can take all the host's RAM and CPU and destabilise it. Leave enough
# for Windows and Unreal (these suit a 32 GB, 22-thread machine).
memory=10GB
processors=6
swap=4GB
```

`localhostForwarding` is not needed: mirrored networking ignores it. DNS
tunnelling is on by default in current WSL.

### ⚠️ Keep a WSL terminal open while running

WSL2 shuts a distro down once nothing is holding it open, and a systemd distro
running dockerd can still be caught by this — when the distro stops, systemd
stops `docker.service`, every container goes down with it, and the dashboard
becomes unreachable ("site cannot be reached"). Touching the distro again boots
it, restarts docker and re-runs every container.

**Keep an interactive WSL session open the whole time the stack is running:**

    wsl -d <distro>      # the distribution Docker runs in (wsl -l -v lists them)

Leave that window open in the background. As long as it is, the distro (and so
docker + all containers) stays alive and the dashboard stays up. If drops
persist, run `wsl --update` then `wsl --shutdown` — premature systemd-distro
shutdowns are a known WSL bug fixed in newer builds.

## License

[MIT](LICENSE), © 2026 Raul Castilla Arquillo, except:

- `component_spad_camera/src/spad_model.py` is ported from
  [spadSim](https://github.com/drodriguezSRL/spadSim), MIT, © 2025 David
  Rodriguez-Martinez (credited in the file).
- `component_path_planner/src/local_planner.py` (the DWA local planner) is
  based on the [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)
  DWA sample by Atsushi Sakai and Goktug Karakasli, MIT, © 2016 - now Atsushi
  Sakai and other contributors (licence reproduced in the file).
- `unreal/UnrealEventCam`'s event model (`EventSimulator.h/.cpp`) is ported
  from ESIM ([rpg_esim](https://github.com/uzh-rpg/rpg_esim)), MIT, Robotics
  and Perception Group, University of Zurich (see its `LICENSE`).
- The Unreal project ([Unreal-Moon-Simulator](https://github.com/raulcastar/Unreal-Moon-Simulator))
  lists its third-party content in its `THIRD_PARTY_NOTICES.md`: the Yutu model
  (CC-BY-4.0 by tashtego, see [Two rovers](#two-rovers-leo_raph-and-yutu)),
  Poly Haven's moon rocks and textures (CC0), TCP-Unreal (MIT) and TAPython.
