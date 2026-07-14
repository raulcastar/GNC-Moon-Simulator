# NavcamStreamer — camera frames without game-thread stalls

UnrealCV renders on demand: each `vget` blocks the game thread for a full
render + synchronous GPU readback (~300 ms per camera, measured — the sim was
frozen ~40% of wall time at five captures per second). This component streams
the navcams instead: SceneCaptures render **with the frame** on the render
thread, readback is **asynchronous** (polled, never flushed mid-render), and a
worker thread ships completed frames over TCP. The game thread only enqueues
copies.

The ROS side is already merged: `component_stereo_depth` with
`image_source: stream` listens on port 3004 and pairs left/right frames by
frame id — every capture enqueued in one tick carries the same `GFrameCounter`,
preserving the same-tick guarantee the batched UnrealCV request used to give.

## Installing as a plugin

This folder **is** the plugin — copy it whole.

1. **Prerequisite (one-time):** a C++ toolchain. Install **Visual Studio 2022
   Community** with the *Game development with C++* workload. Needed even for
   a Blueprint-only project: code plugins are compiled against your engine
   when the editor first loads them.
2. Close the Unreal editor. In your Unreal **project** folder (next to the
   `.uproject`), create a `Plugins` directory if it does not exist, and copy
   this folder into it:

   ```
   <YourProject>/Plugins/NavcamStreamer/
   ├── NavcamStreamer.uplugin
   └── Source/NavcamStreamer/
       ├── NavcamStreamer.Build.cs
       ├── Public/NavcamStreamer.h
       └── Private/{NavcamStreamer.cpp, NavcamStreamerModule.cpp}
   ```

3. Open the `.uproject`. The editor reports *"The following modules are
   missing or built with a different engine version: NavcamStreamer. Would
   you like to rebuild them now?"* — answer **Yes**. It compiles in a minute
   or two and the editor opens. (If it fails, the build log is at
   `<YourProject>/Saved/Logs/UnrealBuildTool/`; the usual cause is the C++
   workload missing from Visual Studio.)
4. Confirm it's active under **Edit → Plugins → Project → Other →
   Navcam Streamer** (enabled by default).
5. Add three **SceneCaptureComponent2D** components named exactly
   **NavcamLeft**, **NavcamRight**, **NavcamCenter**, posed like the three
   navcams (same transforms UnrealCV camera ids 1, 2, 0 have). Give each a
   **TextureRenderTarget2D** (640×480, RTF RGBA8). They can live on any actor
   — the rover pawn is the natural home so they move with it.
6. Add the **NavcamStreamer** component to any actor (the rover pawn is fine;
   it appears in the *Add Component* search once the plugin is built). The
   captures are selected **by component name** — no pickers, no actor linking.
   Its own actor is searched first, then the whole world. If you used the
   names above, nothing needs configuring; otherwise set
   `Left/Right/CenterCaptureName` to match your components tree. At Play, the
   Output Log prints one line per camera saying what was found and on which
   actor — or lists every SceneCapture2D in the world if a name matches
   nothing.

   The cam ids sent to ROS default to left 1, right 2, centre 0, matching
   `left/right/center_camera_id` in
   `component_stereo_depth/config/params.yaml`; they are under the details
   panel's *Advanced* arrow if you ever change those.
7. Leave `ServerIp` at `127.0.0.1`, `ServerPort` at 3004. `SendHz` 5 is ample
   for the 1 Hz stereo pipeline.
8. On the ROS side set `image_source: stream` in the stereo node's params and
   restart it. Start order does not matter — the node listens and Unreal
   connects at BeginPlay (re-Play after restarting the node, as with the rover
   link).

UnrealCV stays connected for what it is still good at: the one-time rig
geometry solve at configure, and the optional `object_mask` / `depth_gt`
products if enabled (each still costs a stall — leave them off outside glare
studies).

## Wire format

One packet per frame, little-endian, 24-byte header:

```
'NAVF' | ver u8 | cam_id u8 | format u8 (1 = BGRA8) | pad u8
width u16 | height u16 | frame_id u64 | payload u32 | pixels (w*h*4)
```

## Version notes

Written against the UE 4.26+ / UE5 RHI API (`FRHIGPUTextureReadback`). If your
engine version complains about `Lock(int32&)`, the signature varies slightly
across versions (`void* Lock(int32& OutRowPitchInPixels)` vs a `uint32`
variant) — adjust the one call in `PollReadbacks`.
