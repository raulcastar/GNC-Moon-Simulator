# UnrealSPADCam — a simulated single-photon (SPAD) camera front-end

Renders an intensity image with each game frame and streams it to a ROS node
over TCP, where the **spadSim** photon model turns it into single-photon binary
frames. Same no-stall design as the sibling `UnrealEventCam` / `UnrealStereoCam`
plugins: the SceneCapture renders on the render thread, the readback is
asynchronous, and a worker thread converts to 8-bit grayscale and drains to the
socket. The game thread never blocks.

## Where the model lives

This plugin is **only the front-end** — it delivers intensity frames. The SPAD
sensor model (photon-flux estimation, Poisson photon arrivals, binary
thresholding, digitisation) runs in the ROS component **`component_spad_camera`**,
which ports **spadSim** by David Rodriguez-Martinez (MIT). That split keeps the
physics in Python/numpy — exactly spadSim's implementation — and matches how the
stereo pipeline works (plugin streams pixels, ROS does the processing).

> Upstream model: https://github.com/drodriguezSRL/spadSim (MIT, © 2025 David
> Rodriguez-Martinez). Related: *"Fast Vision in the Dark: A Case for
> Single-Photon Imaging in Planetary Navigation"*, arXiv:2510.10597.

## What a SPAD camera is

A Single-Photon Avalanche Diode pixel fires when it detects **one photon**. In a
very short exposure a SPAD produces a **binary** frame — 1 where a photon was
detected, 0 elsewhere — governed by Poisson photon statistics. Summing many
binary frames reconstructs a grayscale image, which is what makes SPADs strong in
extreme low light (relevant to lunar permanently-shadowed regions). The ROS node
does the photon sampling and the reconstruction; this plugin feeds it the scene.

## Installing as a plugin

This folder **is** the plugin — copy it whole.

1. **Prerequisite (one-time):** Visual Studio 2022 with the *Game development
   with C++* workload.
2. Close the Unreal editor. In your Unreal **project** folder (next to the
   `.uproject`), create `Plugins/` if needed and copy this folder in:

   ```
   <YourProject>/Plugins/UnrealSPADCam/
   ├── UnrealSPADCam.uplugin
   ├── LICENSE
   └── Source/UnrealSPADCam/
       ├── UnrealSPADCam.Build.cs
       ├── Public/UnrealSPADCam.h
       └── Private/{UnrealSPADCam.cpp, UnrealSPADCamModule.cpp}
   ```

3. Open the `.uproject`. Answer **Yes** to rebuild `UnrealSPADCam`.
4. Confirm it under **Edit → Plugins → Project → Other → Unreal SPAD Cam**.
5. Add a **SceneCaptureComponent2D** named **`SPADCam`** where the sensor should
   sit. Give it a **TextureRenderTarget2D** (or leave it empty and one is
   auto-created at `CaptureWidth`×`CaptureHeight`, default 256×256).
6. Add the **UnrealSPADCam** component to any actor. If you used the name above
   nothing needs configuring; otherwise set **CaptureName** to your component. At
   Play it logs the capture it found and the address it streams to.

## Component parameters

**SPADCam | Network**
- `ServerIp` / `ServerPort` — where `component_spad_camera` listens (default
  `127.0.0.1:3006`; only `127.0.0.1` crosses into WSL under mirrored networking).
- `CamId` — id stamped in every packet.

**SPADCam | Camera**
- `CaptureName` — the SceneCapture2D to render from (dropdown).
- `CaptureWidth` / `CaptureHeight` — render-target size if none is assigned.
- `MaxSampleHz` — cap on how often frames are sampled and sent (0 = every
  rendered frame), per simulated (game) second. The SPAD exposure / photon
  parameters are set on the ROS side.

## Wire format

TCP, little-endian, one packet per sampled frame: a 24-byte header then the
8-bit grayscale image (row-major, `width*height` bytes).

```
'SPD0' | ver u8 | cam_id u8 | width u16 | height u16
       | t_ns u64 | payload u32 (= width*height) | pad u16
```

`component_spad_camera` reads this, runs the spadSim photon model per frame, and
publishes the single-photon binary frames plus a digitised reconstruction.
