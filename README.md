# GNC-Moon-Simulator

ROS 2 (Jazzy) guidance, navigation and control stack for a lunar rover simulated
in Unreal Engine. Sensor data crosses from Unreal into ROS over dedicated Unreal
plugins (StereoCamStreamer for the cameras, MetaLidar for the lidar, a TCP link
for the rover); every component runs in its own container and talks over Zenoh.

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
| `component_stereo_depth` | Receives the three navcams from the StereoCamStreamer plugin and computes stereo depth. |
| `component_lidar` | Velodyne point cloud from Unreal. |
| `component_local_mapping` | Builds a local DEM and occupancy grid from the central navcam's depth. |
| `component_path_planner` | Global path over the elevation grid. |
| `component_rover_leo_raph` | Rover base: `/cmd_vel` in, `/odom` out. |
| `mission_control_dashboard` | Web mission control on port 5000: camera wall, lidar, telemetry, map with goal dispatch, /rosout console. |
| `custom_map_interfaces` | Shared message definitions. |

## Talking to Unreal

Three independent links, and they fail differently:

| Link | Transport | Carries |
|---|---|---|
| StereoCamStreamer | TCP 3004 | navcam frames + object mask (Unreal → ROS) |
| MetaLidar | UDP | Velodyne point cloud (Unreal → ROS) |
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
navcam/right/camera_info             navcam/central/object_mask
                                     navcam/central/object_mask_mapping
```

TF frame names flatten the namespace: `navcam_left`, `navcam_right`,
`navcam_central`.

`image_depth` is **planar Z depth in metres**, `NaN` where no surface was
recovered. It is a stereo product, with the occlusion holes and range-dependent
quantisation of one.

### How frames arrive

The **StereoCamStreamer** plugin (`unreal/StereoCamStreamer/` — build
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
StereoCamStreamer captures must be **co-rotated and offset laterally** on the
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
| nearest resolvable surface | `fx·B / num_disparities` | 1.00 m at 64 disparities |
| depth resolution | `dZ = Z² / (fx·B)` | 14 cm at 3 m, 56 cm at 6 m |
| unmatched left border | `num_disparities` columns | 10% of a 640-px image |

`num_disparities` is the only knob here: the near limit, the border cost and the
matching time all follow from it. Raising it resolves nearer surfaces at the cost
of a wider border.

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
in Unreal — see the StereoCamStreamer README's **Exposure** section (Min/Max
EV100, or a full manual lock).

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

| Script | Purpose |
|---|---|
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
