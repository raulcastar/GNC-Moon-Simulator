# StereoCamStreamer — camera frames without game-thread stalls

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
   <YourProject>/Plugins/StereoCamStreamer/
   ├── StereoCamStreamer.uplugin
   └── Source/StereoCamStreamer/
       ├── StereoCamStreamer.Build.cs
       ├── Public/StereoCamStreamer.h
       └── Private/{StereoCamStreamer.cpp, StereoCamStreamerModule.cpp}
   ```

3. Open the `.uproject`. The editor reports *"The following modules are
   missing or built with a different engine version: StereoCamStreamer. Would
   you like to rebuild them now?"* — answer **Yes**. It compiles in a minute
   or two and the editor opens. (If it fails, the build log is at
   `<YourProject>/Saved/Logs/UnrealBuildTool/`; the usual cause is the C++
   workload missing from Visual Studio.)
4. Confirm it's active under **Edit → Plugins → Project → Other →
   Stereo Cam Streamer** (enabled by default).
5. Add three **SceneCaptureComponent2D** components named exactly
   **NavcamLeft**, **NavcamRight**, **NavcamCenter**, posed like the three
   navcams (same transforms UnrealCV camera ids 1, 2, 0 have). Give each a
   **TextureRenderTarget2D** (640×480, RTF RGBA8). They can live on any actor
   — the rover pawn is the natural home so they move with it.
6. Add the **StereoCamStreamer** component to any actor (the rover pawn is fine;
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

In stream mode the ROS node needs **no** UnrealCV: the rig geometry comes from
`baseline_m` / `hfov_deg` params, colour and the object mask come from the
captures below. The legacy UnrealCV plugin can be disabled outright — and
should be, because its `AnnotationComponent` fights the Custom Depth-Stencil
buffer the object mask uses (see the crash note below).

## Object segmentation mask (optional)

Reproduces UnrealCV's `object_mask` — each object a flat colour — **without**
UnrealCV, by reading Unreal's **Custom Depth-Stencil** buffer and colouring it
in a post-process material. It streams as a fourth capture on the same path as
the RGB cameras.

1. **Enable the buffer.** Project Settings → Rendering → Postprocessing →
   **Custom Depth-Stencil Pass = `Enabled with Stencil`**, then restart the
   editor.
   > Do **not** enable it with the `r.CustomDepth 3` *console* command while the
   > old UnrealCV plugin is still in the project — its `AnnotationComponent`
   > asserts on the forced proxy rebuild and the editor crashes
   > (`PrimitiveSceneProxy.cpp:438`). Set it in Project Settings, or just
   > disable/remove the UnrealCV plugin (it is unused in stream mode).

2. **Tag every actor you want segmented — this is the required per-object step.**
   On each mesh: *Details → Rendering →* set **Render CustomDepth Pass = true**
   **and give it a nonzero CustomDepth Stencil Value (an id, 1–255)**. Same id →
   same colour (e.g. all boulders share one id); unique ids → per-object masks.
   Id 0 is "untagged" and renders black. Special cases:
   - **Nanite meshes** (typical for Megascans rocks/terrain) also need Nanite's
     custom-depth path on, or they silently ignore the stencil value: add
     ```ini
     [SystemSettings]
     r.Nanite.CustomDepth=1
     ```
     to `Config/DefaultEngine.ini`.
   - **Landscape:** set the same two properties on the Landscape actor — and on
     **each** `LandscapeStreamingProxy` if World Partition split the terrain.
   - **Painted foliage:** set them on the **Foliage Type asset**, not on the
     placed instances.

3. **Colouriser material `PPM_ObjectMask`.** A Material with *Material Domain =
   Post Process*, *Blendable Location = After Tonemapping*, that reads a
   `SceneTexture: CustomStencil` node (R channel) and maps the value to a flat
   colour into **Emissive Color** (e.g. a `Custom` HLSL node hashing the id to
   RGB, black when the value is 0).

4. **Mask capture.** A fourth `SceneCaptureComponent2D` (e.g. **`Navcam_Mask`**),
   posed and FOV-matched to `NavcamCenter`, with `PPM_ObjectMask` added to its
   *Post Process Settings → Post Process Materials* (weight 1). Leave its render
   target empty (auto-created). On the **StereoCamStreamer** component set
   **MaskCaptureName** to it; its **MaskCamId** (default 3, under *Advanced*)
   must equal `mask_camera_id` in the stereo node's params.

5. **Name the ids (what each number means).** The mask image carries only
   colours, so the id→name legend is declared on the ROS side in
   `component_stereo_depth/config/params.yaml → mask_labels` as `"<id>:<name>"`
   entries — **keep these ids in sync with the CustomDepth Stencil Values from
   step 2**. The stereo node republishes them as JSON on
   `navcam/central/object_mask_mapping`; `ros2 topic echo` it to read the legend,
   e.g. `{"10": "landscape", "20": "boulder", "30": "crater"}`.

## Exposure

By default the captures auto-expose like any Unreal camera. Two optional
controls on the **StereoCamStreamer** component (Details → *StereoCam | Cameras*)
override that — pick **at most one**:

- **Limit Auto Exposure** (`bLimitAutoExposure`) — keeps auto-exposure but clamps
  the EV100 range it may adapt into, via **Min EV100** and **Max EV100**. This is
  the usual fix for the two failure modes on a dim lunar surface: raising
  **Min EV100** stops the shadows being amplified to grey noise; lowering
  **Max EV100** stops bright regolith being blown to white. Set **Min EV100 ==
  Max EV100** to pin a single fixed exposure — identical on both stereo eyes, so
  their brightness matches (mismatched eye exposure poisons stereo matching)
  without turning metering off. These map to the capture's PostProcess
  *Auto Exposure → Min/Max Brightness*, which carry EV100 under UE5's extended
  luminance range.
- **Lock Exposure** (`bLockExposure`) — switches to fully manual metering
  (`AEM_Manual`, no adapting range at all) with **Exposure Compensation EV** as a
  plain brightness dial. Heavier-handed; use it only when you want to remove the
  scene-luminance feedback loop entirely (e.g. a glare study).

Both are off by default — verify the base image looks right before enabling
either. Changing them takes effect at **Play** (they are applied when the
captures are resolved at BeginPlay).

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
