# UnrealLidarCam — a configurable lidar streamed to ROS over TCP

A lidar component for Unreal Engine 5 with a free beam pattern: you choose the
**horizontal and vertical aperture in degrees** and how many beams cover each.
The beams are measured on the **GPU** by default (depth captures), or on the
**CPU** with one line trace per beam (MetaLidar's approach). Every scan goes over TCP to
`component_lidar`, which publishes it as an organized `sensor_msgs/PointCloud2`
on `lidar/points`.

## Installing

1. Copy this folder to `<Project>/Plugins/UnrealLidarCam`.
2. Open the project; Unreal asks to build the missing `UnrealLidarCam` module: accept
   (needs the Visual Studio C++ tools). Or build from the command line with the
   editor closed.

## Adding it to the rover

Add a **Unreal Lidar Cam** component to the rover blueprint (Add → *Unreal Lidar Cam*). Then
either:

- **attach it under a camera** in the component tree (e.g. drag it onto
  `Navcam_Center`): it scans from where it sits, its +X is the scan's centre; or
- leave it anywhere and pick the camera in **Follow Component Name** (dropdown of
  the actor's cameras and scene captures): it scans from that camera's pose every
  frame, so it shares the camera's view. With the same aperture as the camera
  (90 x 73.7 degrees for the 640 x 480, 90-degree navcams) its points cover the
  image.

It ignores the actor carrying it (the rover) by default.

## Settings (Details → *Lidar*)

| Setting | Default | Meaning |
| --- | --- | --- |
| Mode | GPU | **GPU**: depth captures, read back asynchronously; **CPU**: one line trace per beam |
| Horizontal FOV | 90 deg | Horizontal aperture, centred on forward; 360 = spinning lidar |
| Vertical FOV | 30 deg | Vertical aperture, centred on forward |
| Horizontal Samples | 512 | Beams across the horizontal aperture (columns) |
| Channels | 32 | Beams across the vertical aperture (rows) |
| Min Range / Max Range | 0.2 / 100 m | Returns outside are dropped |
| Scan Rate Hz | 10 | Full scans per second |
| Range Noise Std Dev | 0 m | Gaussian range noise |
| Server Ip / Server Port | 127.0.0.1 / 3007 | Where `component_lidar` listens (`port`) |
| Sensor Id | 0 | Tags the packets |
| Gpu Oversampling | 1.5 | Advanced, GPU mode: depth-map pixels per beam spacing |
| Trace Channel, Trace Complex | Visibility, on | Advanced, CPU mode |
| Ignore Owner | on | Advanced: the carrying actor (the rover) is not seen |
| Draw Points | off | Draws the returns in the viewport (debugging) |

Beams are evenly spaced, edge to edge (for 360 degrees, around the full circle
without repeating). It only scans while `component_lidar` is connected (or Draw
Points is on).

### GPU mode (default)

When a scan is due, depth-only scene captures of the aperture are rendered
(one per horizontal sector of at most 90 degrees, so 360 degrees takes four),
sized from the beam spacing times Gpu Oversampling, and read back
asynchronously; each beam reads its range from the depth map. The cost barely
grows with the number of beams, and the game thread never waits for the GPU (a
scan reaches ROS a frame or two after it is rendered). Vertical apertures up to
160 degrees; beyond that the component falls back to CPU mode.

Intensity, 0-255: the default reflectivity (20) times the cosine of the
incidence angle, with the surface normal taken from the depth map.

### CPU mode

One line trace per beam, spread over worker threads: rows x columns x scan rate
traces per second (512 x 32 at 10 Hz is about 164,000). Traces the collision
geometry (complex by default).

Intensity, 0-255: the hit material's reflectivity times the cosine of the
incidence angle. Name a physical material `PM_Reflectivity_<0-255>` (as in
MetaLidar) to set it; otherwise 20.

## In ROS

`component_lidar` (`config/params.yaml`: `bind_ip`, `port`, `topic`, `frame_id`) publishes each scan on **`lidar/points`**, frame `lidar`:

- organized: height = channels, width = columns; row 0 is the top channel,
  column 0 the leftmost beam;
- fields `x`, `y`, `z`, `intensity` (float32) and `ring`, `column` (uint16);
- sensor frame with the ROS convention (x forward, y left, z up), metres;
- NaN coordinates where a beam returns nothing (`is_dense` = false).

It replaces MetaLidar (the Velodyne VLP-16 over UDP), which the simulator no
longer uses.

## Wire format

One packet per scan, little-endian:

```
'LIDR' | version u8 = 1 | sensor_id u8 | flags u16 (bit 0: organized)
frame_id u64 | stamp f64 (world seconds) | horizontal_fov f32 | vertical_fov f32
columns u32 | rows u32 | point_count u32 | point_step u32 = 20
points: x f32 | y f32 | z f32 | intensity f32 | ring u16 | column u16
```

The point layout is the PointCloud2 layout, so the ROS side republishes a scan
without touching each point. The socket is non-blocking; a full send buffer
waits for room, only a real error drops the link (redialled every 3 s). If the
link is slower than the scans, scans are dropped rather than queued.
