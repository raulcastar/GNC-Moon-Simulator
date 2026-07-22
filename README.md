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

The mission control web interface starts automatically — open
**http://localhost:5000**. It shows all three navcam feeds, stereo depth,
object segmentation, a live top-down lidar sweep, rover telemetry (position,
heading, speed), the elevation map with click-to-dispatch goals, and a system
console streaming `/rosout`.

Two services drop to an interactive shell instead of launching automatically:
`component_local_mapping` and `debug_cli`. Bring them up by hand:

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
| `mission_control_dashboard` | Web mission control on port 5000: camera wall, lidar, telemetry, map with goal dispatch, /rosout console. |
| `custom_map_interfaces` | Shared message definitions. |

**Only one process may hold the UnrealCV socket.** The server accepts a single
client connection, so `component_stereo_depth` serves every camera itself. Stop
it before running anything else that talks to Unreal, including the scripts in
`utils/`.

## Talking to Unreal

Two independent links, and they fail differently:

| Link | Transport | Carries |
|---|---|---|
| UnrealCV | TCP 9000 | cameras, poses, object masks |
| Rover plugin | TCP 3003 | wheel commands out, odometry in |

The containers run `network_mode: host`, so they sit on WSL2's network stack
while Unreal runs on Windows. Crossing that boundary is where this link goes
wrong, and **only `127.0.0.1` works** — the host's LAN address is ambiguous under
mirrored networking and silently drops traffic.

A wildcard is worse: `0.0.0.0` is a bind address, not a destination. Linux
rewrites a datagram sent to it into one sent to `127.0.0.1` (the sender's own
loopback), and Windows rejects it outright. Anything that sends to `0.0.0.0` is
talking to itself. The legacy UDP path (`sending_ip` / `sending_port` /
`listen_port`, kept behind `transport: udp`) is entirely subject to this; TCP
sidesteps it by never needing a destination at all.

### The rover link is TCP (default)

`transport: tcp` replaces the two UDP sockets with **one bidirectional TCP
connection** on `tcp_port` (3003), carrying wheel commands out and pose back.

This is the arrangement that works, because **neither side needs the other's
address**. Every failure on this link came from an address being wrong or
ambiguous — a wildcard used as a destination, or a mirrored-networking address
that could mean Windows or WSL. A connection has no such question: replies go
back the way they came, and Unreal's `Send IP` leaves the picture entirely.

**Unreal dials, the node listens** (`tcp_role: server`). Unreal's TCP component
connects out to `127.0.0.1:3003` — the only address that crosses into WSL; the
host's LAN address does not. Set `tcp_role: client` for the reverse, if the
Unreal component is switched to listen instead.

Unreal delimits nothing, so the reader **scans for JSON objects rather than for a
separator**. Bare, split across reads, coalesced, or newline-delimited all parse,
as do Unreal's locale numbers (`1,205`) even when a read boundary lands inside
one. Payloads are the same as UDP: `FL`/`BL`/`FR`/`BR` + `headlights_on` inbound,
the pose object outbound.

One operational catch: **Unreal connects once, at BeginPlay, and does not
redial.** Restarting `component_rover_leo_raph` drops the listener, so Unreal
must be re-played to reconnect. The node itself handles the reverse fine — it
keeps listening and re-accepts across a Play/Stop cycle without a restart.

To test the link without ROS in the way:

```bash
docker compose stop component_rover_leo_raph   # it holds the telemetry port
python utils/udp_probe.py
```

It waits for telemetry, reports where Unreal is, drives the rover from that
address and tells you whether the pose changed. Silence in the first step means
Unreal is not sending — the plugin is off, aimed elsewhere, or **the game is
paused**, which freezes physics so no command has any effect.

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
configure time the node reads all three poses and checks:

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

### Streaming frames instead of polling UnrealCV

UnrealCV renders on demand: every `vget` blocks the game thread for a full
render + synchronous GPU readback — **~300 ms per camera, measured**, freezing
the simulation ~40% of wall time at 1 Hz with three cameras. Switching the
image encoding does not help (BMP took as long as PNG); the stall *is* the
synchronous readback.

`image_source: stream` in `component_stereo_depth/config/params.yaml` removes
it. The **StereoCamStreamer** component (`unreal/StereoCamStreamer/` — build
instructions in its README) renders the navcams with each frame on the render
thread, reads them back asynchronously, and pushes them to the node over TCP
port 3004. Left/right frames carry Unreal's frame counter and are paired only
when the ids match exactly, preserving the same-tick guarantee the batched
UnrealCV request provided. The centre frame may trail by a frame or two — it is
overlaid, not matched.

UnrealCV stays connected for the one-time rig geometry solve and, if enabled,
`depth_gt` / `object_mask` — each of those still costs a stall and only aligns
with streamed frames while the rover is stationary, so leave them off outside
glare studies. The default remains `unrealcv` until the plugin side is built.

### Simultaneity without pausing

A stereo pair is only rigid if both eyes see the same world state. The node sends
all its grabs as **one batched UnrealCV request**, so the server answers them
back to back and normally within a single game tick. The world advances on ticks
rather than wall-clock, so that is simultaneous however long the wire took.

Every batch is bracketed by a `vget /camera/N/location` probe. If the two replies
differ, the batch spanned ticks and the node says by how much, as a fraction of
the baseline:

```
Capture: the rig moved 5.00 cm mid-batch (25% of the baseline) — the grabs span
game ticks and are not simultaneous.
```

`pause_game_during_capture` guarantees simultaneity but freezes the simulation
for every other component, so it is **off by default**. Turn it on only if the
drift warning shows the batch spanning ticks and an exact baseline matters more
than uninterrupted rover dynamics.

The configure-time geometry snapshot uses the same batch and probe, retrying a
few times to catch the rig within one tick. Measure the rig with the rover
stationary if you want an exact baseline.

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
| `utils/list_unreal_cameras.py` | Enumerate camera poses; flag usable stereo pairs. `--resume` clears a stale pause. |
| `utils/stereo_depth_test.py` | One-shot SGBM + glare comparison, plotted with matplotlib. |
| `utils/connect_unrealcv.py` | Minimal capture and display. |
| `utils/udp_probe.py` | Prove the rover UDP link both ways. Needs `component_rover_leo_raph` stopped, not the stereo node. |

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
