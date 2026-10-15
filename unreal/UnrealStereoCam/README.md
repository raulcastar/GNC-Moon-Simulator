# UnrealStereoCam — camera frames without game-thread stalls

This component streams the navcams to ROS without stalling the game:
SceneCaptures render on the render thread **only when a frame is sent**
(`SendHz`, with `bCaptureOnDemand`, the default), readback is **asynchronous**
(polled, never flushed mid-render), and a worker thread ships completed frames
over TCP. The game thread only queues the renders and copies.

Rendering on demand doubled the editor's frame rate (6.7 -> 14.2 frames/s with
all camera outputs read) with identical images: each capture is a full scene
render, and rendering every engine frame threw most of them away. The captures
keep their rendering state between renders (exposure, temporal AA, Lumen). The
ground-truth depth capture also skips all shading (lighting, shadows, GI,
reflections, decals, post-processing). Untick `bCaptureOnDemand` to render
every frame as before.

On the ROS side, `component_stereo_depth` listens on port 3004 and pairs
left/right frames by frame id: every capture enqueued in one tick carries the
same `GFrameCounter`, so a pair always shows the same world state.

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
   <YourProject>/Plugins/UnrealStereoCam/
   ├── UnrealStereoCam.uplugin
   └── Source/UnrealStereoCam/
       ├── UnrealStereoCam.Build.cs
       ├── Public/UnrealStereoCam.h
       └── Private/{UnrealStereoCam.cpp, UnrealStereoCamModule.cpp}
   ```

3. Open the `.uproject`. The editor reports *"The following modules are
   missing or built with a different engine version: UnrealStereoCam. Would
   you like to rebuild them now?"* — answer **Yes**. It compiles in a minute
   or two and the editor opens. (If it fails, the build log is at
   `<YourProject>/Saved/Logs/UnrealBuildTool/`; the usual cause is the C++
   workload missing from Visual Studio.)
4. Confirm it's active under **Edit → Plugins → Project → Other →
   Unreal Stereo Cam** (enabled by default).
5. Add three **SceneCaptureComponent2D** components named exactly
   **NavcamLeft**, **NavcamRight**, **NavcamCenter**, posed like the three
   navcams (left, right and centre of the stereo rig). Give each a
   **TextureRenderTarget2D** (640×480, RTF RGBA8). They can live on any actor
   — the rover pawn is the natural home so they move with it.
6. Add the **UnrealStereoCam** component to any actor (the rover pawn is fine;
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
   for the 1 Hz stereo pipeline. Rates are per simulated (game) second, so they
   hold relative to physics and `/clock` when the editor runs below real time.
8. On the ROS side set `image_source: stream` in the stereo node's params and
   restart it. Start order does not matter — the node listens and Unreal
   connects at BeginPlay (re-Play after restarting the node, as with the rover
   link).

The rig geometry comes from the node's `baseline_m` / `hfov_deg` params;
colour and the object mask come from the captures below.

## Object segmentation mask (optional)

An object mask, each object a flat colour, made by reading Unreal's
**Custom Depth-Stencil** buffer and colouring it in a post-process material. It streams as a fourth capture on the same path as
the RGB cameras.

1. **Enable the buffer.** Project Settings → Rendering → Postprocessing →
   **Custom Depth-Stencil Pass = `Enabled with Stencil`**, then restart the
   editor.

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

3. **Colouriser material `PPM_GroundTruthMask`** (in `/Game/GroundTruth_Masks/Mask`,
   where Terrain Studio looks for it). A Material with *Material Domain =
   Post Process*, *Blendable Location = After Tonemapping*, that reads a
   `SceneTexture: CustomStencil` node (R channel) and maps the value to a flat
   colour into **Emissive Color** (e.g. a `Custom` HLSL node hashing the id to
   RGB, black when the value is 0).

4. **Mask capture.** A fourth `SceneCaptureComponent2D` (e.g. **`Navcam_Mask`**),
   posed and FOV-matched to `NavcamCenter`, with `PPM_GroundTruthMask` added to its
   *Post Process Settings → Post Process Materials* (weight 1). Leave its render
   target empty (auto-created). On the **UnrealStereoCam** component set
   **MaskCaptureName** to it; its **MaskCamId** (default 3, under *Advanced*)
   must equal `mask_camera_id` in the stereo node's params.

5. **Name the ids (what each number means).** The mask image carries only
   colours, so the classes (id, name, colour, group) are listed in
   `component_stereo_depth/config/mask_classes.json` — the same file as Terrain
   Studio's `config/mask_classes.json`, which sets the CustomDepth Stencil Values
   of step 2 and the mask colours. **Edit both copies together**; Terrain
   Studio's ground-truth page says when they differ. The stereo node republishes
   it as JSON on `navcam/central/object_mask_mapping` (id → name, e.g.
   `{"1": "sky", "5": "landscape", "9": "boulder"}`) and
   `navcam/central/object_mask_classes` (with colours and groups).

## Ground-truth depth (on by default)

With **Stream Center Depth** (`bStreamCenterDepth`, Details → *StereoCam |
Cameras*), the component also streams Unreal's own depth for the central camera.
At Play it adds a depth-only `SceneCaptureComponent2D` attached to the centre
capture, with its projection, FOV, resolution and hidden / show-only actors, so
it sees exactly the same view; nothing has to be added to the blueprint. Each
pixel is the scene depth in centimetres along the camera axis (planar depth),
sent as float on **DepthCamId** (default 4, under *Advanced*; must match
`depth_ground_truth_camera_id` in `component_stereo_depth/config/params.yaml`).

The stereo node publishes it as `navcam/central/depth_ground_truth` (32FC1,
metres, +Inf for the sky) and `.../visualization`, with the same stamp and frame
as `image_color` and `image_depth`, so the stereo depth can be checked against
it pixel for pixel.

## Extra mono cameras (e.g. the rear camera)

Any other SceneCapture2D can ride the same link as a plain colour camera: add
a second UnrealStereoCam component that streams only that capture — Center =
the capture, CenterCamId = a free id (10 and up), Left / Right blank,
`bStreamCenterDepth` off. The ROS node accepts several streamer connections on
port 3004 at once, and publishes the camera as listed in `extra_cameras`
(`"<cam_id>:<topic>[:<hfov_deg>[:<mask_cam_id>]]"`, rate `extra_cameras_hz`) in
`component_stereo_depth/config/params.yaml`.

**Its own object mask (optional):** a mask capture beside it — the same
`PPM_GroundTruthMask` material, sun-only show flags and manual exposure as the
navcam's mask capture, at the camera's pose and FOV — set as this streamer's
**MaskCaptureName** with a free **MaskCamId**, and that id as the entry's fourth
field. The node then publishes `<topic>/object_mask`, `shadow_mask` and
`shadow_label` (with their mappings), each paired with the image of the same
frame. Terrain Studio sets up the navcam's mask camera only; copy its settings
(a mask capture's render settings live on the placed rover, where Terrain Studio
writes them, and on the Blueprint).

Leo_Raph does this for its rear camera: `RearCam` on the body (behind it,
looking back, pitched -20°, 90° FOV) streamed by `RearCam_Stream` on cam id 10
-> `rear_cam/image_color` (+ `/compressed`, `/camera_info`), shown as REAR on
the dashboard; and `RearCam_Mask` on cam id 13 -> `rear_cam/object_mask`,
`shadow_mask`, `shadow_label`, shown as REAR SEGMENTATION / REAR SHADOW MASK.

## Exposure

By default the captures auto-expose like any Unreal camera. Two optional
controls on the **UnrealStereoCam** component (Details → *StereoCam | Cameras*)
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

One packet per frame, little-endian, 32-byte header (version 2):

```
'NAVF' | ver u8 = 2 | cam_id u8 | format u8 | pad u8
width u16 | height u16 | frame_id u64 | payload u32 | stamp_ns u64 | pixels (w*h*4)
```

`stamp_ns` is the capture time on the simulation clock (UnrealSimClock: ns since
the Unix epoch), the same for every capture of one tick; the stereo node stamps
everything it publishes from it. Version 1 (24-byte header, no `stamp_ns`) is
still read; its frames are stamped on arrival. The time comes from the plugin's
own `GncSimTime.h` (see the UnrealSimClock README); no other plugin is needed.

`format` 1 is BGRA8 colour; 2 is R32F depth (float32 centimetres of planar
depth, the ground-truth depth above). Both use 4 bytes per pixel.

The socket is non-blocking: when a pump's frames exceed the send buffer, the
sender waits for room and continues; only a real socket error drops the link
(it then redials every 3 s).

## Version notes

Written against the UE 4.26+ / UE5 RHI API (`FRHIGPUTextureReadback`). If your
engine version complains about `Lock(int32&)`, the signature varies slightly
across versions (`void* Lock(int32& OutRowPitchInPixels)` vs a `uint32`
variant) — adjust the one call in `PollReadbacks`.
