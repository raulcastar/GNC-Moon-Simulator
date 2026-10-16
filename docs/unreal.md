# The Unreal side

## Project and plugins

The Unreal project (`Moon_Simulator`, Unreal Engine 5.7) is its own repository,
[MARE-Lunar-Simulator](https://github.com/raulcastar/MARE-Lunar-Simulator).
It holds the levels and the rover Blueprints (`Leo_Raph`, `Yutu_Rover`), with a
copy of the sensor plugins in its `Plugins/`. Each plugin also has its own
repository, with its setup, wire format and prebuilt releases:

| Plugin | Role | ROS node |
|---|---|---|
| [UnrealSimClock](https://github.com/raulcastar/UnrealSimClock) | The simulation time, streamed to `/clock`; Blueprint nodes *Get Sim Time Ns String* / *Seconds* | `component_sim_clock` |
| [UnrealStereoCam](https://github.com/raulcastar/UnrealStereoCam) | Navcams (left, right, central), object mask, ground-truth depth, rear camera | `component_stereo_depth` |
| [UnrealLidarCam](https://github.com/raulcastar/UnrealLidarCam) | Lidar with a configurable aperture (GPU depth captures or line traces) | `component_lidar` |
| [UnrealEventCam](https://github.com/raulcastar/UnrealEventCam) | Event camera: brightness-change events from rendered frames (ESIM) | `component_event_camera` |
| [UnrealSPADCam](https://github.com/raulcastar/UnrealSPADCam) | Intensity frames for the SPAD photon model | `component_spad_camera` |

The project also uses the `TCP-Unreal` plugin for the rover link, the
`TAPython` plugin (Terrain Studio builds Landscapes with it), and editor Python
(*Python Editor Script Plugin*, *Remote Execution* on) for Terrain Studio and
the [viewport cameras](mission-control.md#viewport-cameras).

## Links and ports

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
while Unreal runs on Windows. **Only `127.0.0.1` works** across that boundary:
the host's LAN address is ambiguous under mirrored networking and silently
drops traffic. `0.0.0.0` is a bind address, not a destination: Linux rewrites a
datagram sent to it into one to its own loopback, and Windows rejects it. The
rover node's legacy UDP path (`transport: udp`, `sending_ip` / `sending_port` /
`listen_port`) is subject to this; TCP never needs a destination address.

### The rover link

`transport: tcp` (the default) is **one bidirectional TCP connection** on
`tcp_port` (3003), carrying wheel commands out and pose back. **Unreal dials,
the node listens** (`tcp_role: server`); set `tcp_role: client` if the Unreal
component is switched to listen instead.

Unreal delimits nothing, so the reader **scans for JSON objects rather than for
a separator**. Bare, split across reads, coalesced, or newline-delimited all
parse, as do Unreal's locale numbers (`1,205`) even when a read boundary lands
inside one.

- **Out:** `{"FL", "BL", "FR", "BR", "headlights_on", "headlights_intensity"}`,
  the wheel inputs (negative = forward in the Blueprint) and the lights.
  `headlights_intensity` is 0–1 of the Blueprint's own light intensity
  (`HeadLight_L` / `HeadLight_R`, 20 000 cd each; brighter burnt out up to 44 %
  of the navcam's ground view). The node takes it from the
  `headlights_intensity` topic (`std_msgs/Float32`, clamped) or the parameter
  of that name at start; the dashboard has a slider for it. On/off stays on
  `headlights_cmd`. The Blueprint reads it with *Get Field* (Json Blueprint
  Utilities), which treats an exact 0 as a missing field, so 0 % goes out as
  `1e-9`.
- **In:** the world transform of the Blueprint's `base_link` component
  (`x, y, z` cm, `roll, pitch, yaw` degrees), optionally the physics body's
  velocities (`vx, vy, vz` cm/s, `wx, wy, wz` rad/s) and `t_ns`.

Unreal **connects once, at BeginPlay, and does not redial**: restarting
`component_rover_leo_raph` drops the listener, so Unreal must be re-played. The
node keeps listening and re-accepts across a Play/Stop cycle. Every new
connection is a new Play session: the node resets its acceleration ramp (a ramp
cut short by Stop would otherwise spin the wheels while the rover lands at
spawn) and sends the headlight state.

If the rover does not move, check that the game is not **paused**: that
freezes physics.

## Simulation time

Everything runs on Unreal's clock: Unreal's game time (it stops when Play is
paused and follows time dilation), in nanoseconds since the Unix epoch, never
running backwards across Play sessions. Every sensor plugin stamps its data
with it at capture, through its own copy of `GncSimTime.h`, so the plugins stay
independent of each other. UnrealSimClock (a world subsystem, no Blueprint
setup) streams that time to `component_sim_clock`, which publishes `/clock`.
The rover Blueprint puts the same time in its telemetry (`t_ns`, from *Get Sim
Time Ns String*), so odometry is stamped with the moment Unreal measured the
pose.

The GNC nodes run with `use_sim_time: true`, so a message's stamp is when the
sensor captured it, and topics can be matched by stamp: `component_local_mapping`
places each depth frame at the rover pose **at its capture time**, interpolated
from odometry (0.5 s of processing at 0.5 m/s would otherwise misplace it by
25 cm). When Unreal is not playing, simulation time stands still and the nodes'
timers pause with it. The dashboard stays on the wall clock (it is a UI); its
rosbags record `/clock`, so a replay runs on the recorded time.

## Project settings

`Config/DefaultEngine.ini` settings the simulation depends on:

| Setting | Value | Why |
|---|---|---|
| `bUseFixedFrameRate` / `FixedFrameRate` | True / 30 | Every frame advances game time by 1/30 s, so physics, `/clock` and the sensors stay consistent even when the editor renders slower than real time (the simulation then runs slower than the wall clock, never inconsistently). |
| `bSubstepping`, `MaxSubstepDeltaTime`, `MaxSubsteps` | True, 0.008333, 8 | Physics at 120 Hz in 4 substeps per frame. Without substepping, `MaxPhysicsDeltaTime` 0.033 alone ran physics at ~0.43× the clock at 13 fps. |
| `bEnableEnhancedDeterminism`, `RestitutionCombineMode` | True, Min | Repeatable runs; wheels do not bounce off the ground. |
| `DefaultGravityZ` | -980 (Earth) | See [Gravity](rovers.md#gravity). |
| `+StartupScripts` | `rover_cameras_startup.py` | The viewport cameras follow the dashboard. |

All sensor plugins gate their rates on game time, so their rates are per
simulated second, like `/clock`.
