# Rovers

## Leo_Raph and Yutu

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
Open the rover's level in Unreal and pick the rover there; no container
restarts. The rover stops first. The selector:

- applies the rover node's drive calibration live (Unreal's TCP link stays up);
- sets the planner's footprint and limits;
- gives `robot_state_publisher` the rover's URDF, so `/tf_static` changes;
- sets the dashboard's own values;
- re-configures the planner and local mapping, which read their sensor mounts
  from TF. A goal in progress is dropped.

The values come from the same config files, mounted read-only into the
dashboard's container (`mission_control_dashboard/src/rovers.py`). Next to the
selector, a line shows the rover Unreal is playing (reported by the viewport
camera link, `rover_cameras.py`), in amber when it differs; the overview's
header shows the same check (`Unreal: Yutu ✓`, or amber `≠`). The choice lasts
until the containers restart; then `.env` decides again.

**To switch for good:** open the rover's level in Unreal; set `GNC_ROVER` in
`.env` (repo root); `docker compose up -d` (recreates, and rebuilds, the four
containers that use it: rover node, planner, robot description, dashboard);
press Play.

Each of those launch files loads its `config/params.yaml` and then, when it
exists, `config/params_<GNC_ROVER>.yaml` with the rover's overrides; the robot
description loads `urdf/<GNC_ROVER>.urdf`. Yutu's overrides:

- **rover node:** `wheel_speed_scale` 0.96 and `rover_width` 330 (calibrated:
  0.2 m/s -> 0.198, 0.2 rad/s -> 0.197-0.199).
- **planner:** footprint 1.15 x 1.55 m, `robot_radius` 0.8, `lidar_self_radius_m`
  1.0 (the mast lidar sees the wings and arm), and `max_yaw_rate_deg` 8: spot
  turns rock the rigid six-wheel chassis (pitch/roll swing 2-4 deg at 0.1 rad/s,
  5-8 at 0.15, 8-10 at 0.2, 18-24 at 0.3).
- **dashboard:** `lidar_height_m` 1.13, `lidar_self_radius_m` 1.0, `rover_width_m`
  1.11 (the map icon), `teleop_max_angular` 0.15 (Leo_Raph 0.35).

### Adding a rover (how Yutu was built)

The glTF was prepared in Blender
(`Content/RoverAssets/Yutu/Source/prepare_yutu.py` in the Unreal project:
turned to face +X, scaled x2 to real size, the six wheels separated with their
pivots on their axles) and imported to `/Game/RoverAssets/Yutu`. `Yutu_Rover`
is a **copy of Leo_Raph**, so it keeps its graph (TCP link, telemetry, lights,
sensors) and component names; the meshes, wheel and joint positions, masses,
sensor mounts and collision (a chassis box, sphere wheels) were changed, and
two middle wheels and joints added and driven like the front ones. Its URDF
comes from the same exporter (`export_rover_urdf.py`, run with the Yutu level
open). For its wheel tracks and rover-shadow tag, send the masks from Terrain
Studio once with the level open.

Yutu is based on ["Yutu"](https://sketchfab.com/3d-models/yutu-d9564c9a60c14b85a880cc0bfcf1a6b5)
by [tashtego](https://sketchfab.com/tashtego), licensed under
[CC-BY-4.0](http://creativecommons.org/licenses/by/4.0/). The original
`license.txt` and a `CREDITS.txt` with the changes made are in the Unreal
project's `Content/RoverAssets/Yutu/`.

## The rover node

`component_rover_leo_raph` turns `/cmd_vel` into wheel inputs and Unreal's
telemetry into `/odom` (it drives either rover).

- **Commands:** a velocity step is ramped to `max_linear_accel` /
  `max_angular_accel` (and the decel limits) per simulated second, then split
  into skid-steer side speeds, `v ∓ ω · rover_width / 2`, times
  `wheel_speed_scale` (m/s → Unreal wheel units). The ramp completes even if
  `/cmd_vel` stops arriving, and the node holds the last command: send a zero
  twist to stop.
- **Odometry:** `pose_from_telemetry` converts the Blueprint's `base_link`
  transform from Unreal (left-handed, cm, degrees) to REP-103 (x forward, y
  left, z up, metres), through the rotation's axes, so roll, pitch and heading
  are exact at any attitude. The twist comes from the physics body when the
  Blueprint sends it (marked by a non-zero twist covariance). Each message is
  also broadcast as TF `odom → base_link` (`publish_tf`).

`wheel_speed_scale` and `rover_width` (the *effective* skid-steer track: slip
makes it wider than the physical 38 cm) are calibrated on level ground; the
comments in `config/params.yaml` give the measurements. Recalibrate after
changing gravity, wheel friction, damping or the drive strength: drive at
0.2 m/s and spot-turn at ±0.3 rad/s, and compare commanded with driven.

## Gravity

The simulation runs under **Earth gravity** (`DefaultGravityZ` -980 cm/s²): the
rover is much easier to control than under the Moon's -162. What changed with
it, and must change together again if gravity changes:

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
- **Centre of mass:** nudged to (-2.4, -1.2, -18.2) cm: over the wheel centre
  and 12 cm high, so the static tip-over angle is ~58°. Mass: 150 kg body,
  10 kg per wheel.
- **Drive strength:** the wheel drives are velocity servos whose torque is
  strength × speed error, so the strength sets how far the rover slows under
  load. It scales with the rover's weight: at 10000 under Earth gravity the
  rover stalled on 11° slopes and in tight arcs beside rocks.
- `wheel_speed_scale` 1.82 holds under both gravities.
- **Spot turns scrub** (isotropic wheel friction) and rock the rover: under
  Earth gravity the pitch swings ~2° at 0.1 rad/s, ~4.5° at 0.2, 7–9° at 0.35
  and 13–15° at 0.5. Hence the planner's 12°/s and the teleop cap.
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
centred between the wheels at axle height, x forward, z up. Everything below it
is static, from `component_robot_description/urdf/<GNC_ROVER>.urdf`, written by
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
