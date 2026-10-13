# UnrealEventCam — a simulated event (DVS) camera for Unreal

Renders an intensity image with each game frame, generates brightness-change
**events** from it with the ESIM model, and streams them to a ROS node over TCP.
Same no-stall design as the sibling `UnrealStereoCam` plugin: the SceneCapture
renders on the render thread, the readback is asynchronous, and a worker thread
runs the event model and drains to the socket. The game thread never blocks.


## How an event camera works (and the one caveat here)

A DVS pixel fires an **event** `(x, y, t, polarity)` whenever the log intensity
at that pixel has changed by a contrast threshold `C` since it last fired —
positive polarity for a brightness increase, negative for a decrease. There is
no frame: the output is a sparse, asynchronous, microsecond-timestamped stream.

ESIM reconstructs this from rendered frames by looking at how far the log
intensity moved between two consecutive frames and emitting one event per
threshold crossing, with the crossing time interpolated linearly across the
frame interval.

**Caveat — sampling rate.** This is only accurate when frames are rendered far
faster than the scene changes. Here the sampling rate *is* the render frame
rate, so fast motion between frames is linearly interpolated and can alias, and
undersampling *loses* events (unlike an RGB stream, where it just drops frames).
For the cleanest stream: run the editor at a high, steady FPS, keep the event
resolution modest, and avoid extremely small contrast thresholds.

## Installing as a plugin

This folder **is** the plugin — copy it whole.

1. **Prerequisite (one-time):** Visual Studio 2022 with the *Game development
   with C++* workload (code plugins are compiled against your engine).
2. Close the Unreal editor. In your Unreal **project** folder (next to the
   `.uproject`), create `Plugins/` if needed and copy this folder in:

   ```
   <YourProject>/Plugins/UnrealEventCam/
   ├── UnrealEventCam.uplugin
   ├── LICENSE
   └── Source/UnrealEventCam/
       ├── UnrealEventCam.Build.cs
       ├── Public/{UnrealEventCam.h, EventSimulator.h}
       └── Private/{UnrealEventCam.cpp, EventSimulator.cpp, UnrealEventCamModule.cpp}
   ```

3. Open the `.uproject`. Answer **Yes** to rebuild `UnrealEventCam`. It compiles
   in a minute or two. (Build log on failure:
   `<YourProject>/Saved/Logs/UnrealBuildTool/`.)
4. Confirm it under **Edit → Plugins → Project → Other → Unreal Event Cam**
   (enabled by default).
5. Add a **SceneCaptureComponent2D** named **`EventCam`** where the sensor should
   sit (the rover pawn is fine — it moves with the actor). Give it a
   **TextureRenderTarget2D** (or leave it empty and one is auto-created at
   `CaptureWidth`×`CaptureHeight`, default 346×260 — a DAVIS-346 resolution).
6. Add the **UnrealEventCam** component to any actor. If you used the name above
   nothing needs configuring; otherwise set **CaptureName** to your component.
   At Play it logs the capture it found and the address it streams to.

## Component parameters

**EventCam | Network**
- `ServerIp` / `ServerPort` — where the ROS event node listens (default
  `127.0.0.1:3005`; only `127.0.0.1` crosses into WSL under mirrored networking).
- `CamId` — id stamped in every packet, so several event cameras share a consumer.

**EventCam | Camera**
- `CaptureName` — the SceneCapture2D to render from (dropdown).
- `CaptureWidth` / `CaptureHeight` — render-target size if none is assigned. Keep
  modest; the model is O(W·H) per frame on the CPU.
- `MaxSampleHz` — cap on how often frames are sampled for events (0 = every
  rendered frame). Higher is better for event fidelity. Per simulated (game)
  second, so it holds when the editor runs below real time.

**EventCam | Model** (ESIM)
- `ContrastThresholdPos` / `ContrastThresholdNeg` — C+ / C- in log-intensity
  units. Real DVS pixels sit around 0.1–0.5; larger = fewer events.
- `SigmaThresholdPos` / `SigmaThresholdNeg` *(advanced)* — Gaussian noise on the
  thresholds (pixel mismatch). 0 disables.
- `RefractoryPeriodMs` *(advanced)* — per-pixel dead time after firing. 0 disables.
- `bUseLogImage` *(advanced)* — operate on log intensity (the physical model);
  leave on outside debugging.

## Wire format

TCP, little-endian, one packet per frame **that produced events** (frames with
no events send nothing). A 24-byte header, then `count` 9-byte records:

```
header  'EVT0' | ver u8 | cam_id u8 | width u16 | height u16
        | t0_ns u64 | count u32 | pad u16
event   x u16 | y u16 | t_rel_ns u32 | pol u8      (pol: 1 = +, 0 = -)
```

Each event's absolute timestamp is `t0_ns + t_rel_ns`. Events within a packet
are sorted by time.

## Consuming the events in ROS

There is no ROS node for this yet. A consumer opens a TCP server on `ServerPort`,
reads the 24-byte header, then `count × 9` bytes, and republishes — e.g. as
[`dvs_msgs/EventArray`](https://github.com/uzh-rpg/rpg_dvs_ros) or a custom
message. Mirror the `FrameStreamReceiver` in
`component_stereo_depth/src/component_stereo_depth.py`, which already does exactly
this handshake for the RGB stream. Ask and this can be scaffolded as a
`component_event_camera` package.


## Acknowledgments

The event-generation core (`EventSimulator.h` / `.cpp`) is a port of the
**EventSimulator** from **ESIM (rpg_esim)** by the Robotics and Perception Group,
University of Zurich — used under MIT License (see [`LICENSE`](LICENSE)).

The port keeps ESIM's algorithm — per-pixel log-intensity threshold crossings,
linear timestamp interpolation between frames, per-polarity contrast thresholds,
optional Gaussian threshold noise and a per-pixel refractory period — and only
swaps the OpenCV / glog / ze types for Unreal primitives so it has no external
dependencies.

Upstream: https://github.com/uzh-rpg/rpg_esim