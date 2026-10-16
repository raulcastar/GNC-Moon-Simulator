# Perception

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
navcam/right/camera_info             navcam/central/depth_ground_truth
                                     navcam/central/depth_ground_truth/visualization
                                     navcam/central/object_mask
                                     navcam/central/object_mask_mapping
                                     navcam/central/object_mask_classes
                                     navcam/central/shadow_mask
                                     navcam/central/shadow_label
                                     navcam/central/shadow_label_mapping
```

`image_depth` is **planar Z depth in metres**, `NaN` where no surface was
recovered: a stereo product, with the occlusion holes and range-dependent
quantisation of one. `depth_ground_truth` is Unreal's exact depth for the same
view (`+Inf` for the sky). The `visualization` images are colour-mapped up to
`depth_max_distance` (6 m).

### Object and shadow masks

The object mask's classes (id, name, colour, group) are in
`component_stereo_depth/config/mask_classes.json`, a copy of Terrain Studio's
`config/mask_classes.json`, which bakes them into the Unreal mask material:
**edit both together** (Terrain Studio's ground-truth page reports when they
differ). The node publishes them as JSON on `object_mask_mapping` and
`object_mask_classes`; the dashboard draws its legends from them. The shadow
feeds come from flag bits in the same mask frame (see Terrain Studio's
[masks documentation](https://github.com/raulcastar/Unreal-Moon-Terrain-Generation/blob/main/docs/masks.md#how-the-shadows-travel)).

The rear camera has its masks too: `rear_cam/object_mask` (+ `_mapping`,
`_classes`), `rear_cam/shadow_mask` and `rear_cam/shadow_label` (+ `_mapping`),
from `RearCam_Mask` in the Blueprint, streamed on cam id 13. The option is the
fourth field of its `extra_cameras` entry (`"10:rear_cam:90:13"`): drop it to
publish the colour only. Each mask is paired with the rear image of the same
Unreal frame. `enable_object_mask` / `enable_shadow_mask` switch them with the
central ones.

### How frames arrive

UnrealStereoCam renders the navcams when a frame is due (`SendHz`), reads them
back asynchronously, and pushes them to the node over TCP port 3004, without
stalling the game thread. Every capture of one Unreal tick carries the same
frame counter, and the node pairs the left/right eyes **only when their ids
match exactly**: that is the simultaneity guarantee. The central frame is taken
from the nearest tick; it is overlaid, not matched, so a frame or two of slack
is fine.

### Rig requirements

SGBM assumes epipolar lines are image rows. That holds only when the two eyes
share a rotation and are offset purely along the left camera's right axis, so
the captures must be **co-rotated and offset laterally** on the mount, with the
central capture between them.

The geometry is **declared from parameters** in
`component_stereo_depth/config/params.yaml`: `baseline_m` (eye separation),
`hfov_deg` (the captures' FOV) and `center_offset_m` (the central camera's
position along the baseline). The image size is taken from the first frames,
and the node logs the rig it solved at configure time:

```
Rig declared (stream mode): 640x480 | baseline 20.00 cm | hfov 90.00 deg
-> fx 320.00 px | centre 10.00 cm from the left eye
```

Set these to the real capture geometry: wrong values shift absolute depth.

### Geometry that governs the result

With focal length `fx` (px) and baseline `B` (m):

| quantity | formula | at `fx·B = 64` |
|---|---|---|
| nearest resolvable surface | `fx·B / num_disparities` | 0.50 m at 128 disparities |
| depth resolution | `dZ = Z² / (fx·B)` | 14 cm at 3 m, 56 cm at 6 m |
| unmatched left border | `num_disparities` columns | 20% of a 640-px image at 128 |

`num_disparities` (128) is the only knob here: the near limit, the border cost
and the matching time all follow from it. Anything nearer than the near limit
has a disparity beyond the search window; SGBM locks onto whatever correlates
best inside the range, producing a speckled band along the bottom of the image.
Depth nearer than the limit is rejected (the node logs it at configure time);
raise `num_disparities` if the bottom rows drop out.

`fill_left_border` recovers most of the dead band: OpenCV blanks every column
`u < num_disparities`, but a pixel at column `u` with disparity `d` is matchable
whenever `d ≤ u`, and distant surfaces have small `d`.

If the two eyes auto-expose independently, they diverge most when a bright
source is in one field of view, and the mismatch looks like glare. Clamp or
lock the captures' exposure in Unreal: see UnrealStereoCam's
[exposure options](https://github.com/raulcastar/UnrealStereoCam/blob/main/docs/options.md#exposure).

## Local mapping

`component_local_mapping` subscribes to `navcam/central/{image_depth,
image_color, camera_info}` and `/odom`, and publishes:

- `local_mapping/elevation_map_raw`: `Float32MultiArray`, metres
- `local_mapping/elevation_grid`: `OccupancyGrid` in `odom`
- `local_mapping/pointcloud`: `PointCloud2` in `odom`

Each depth frame is projected to the ground frame with the camera mount (from
TF) and the rover's roll and pitch at the frame's capture time
(`apply_rover_tilt`), then binned at `map_resolution` (5 cm) within
`distance_limit` (3 m). A cell takes its second-highest point
(`drop_highest`), so single stereo outliers do not spike it, and needs
`min_points_per_cell` points. Intrinsics come from `camera_info` rather than
YAML, so changing a camera's FOV in Unreal cannot silently stretch the DEM.

## Other sensors

| Node | Publishes |
|---|---|
| `component_lidar` | `lidar/points`: organized `PointCloud2` in frame `lidar` |
| `component_imu` | `imu/data`, simulated from `/odom`: attitude, body rates and specific force (gravity included) |
| `component_event_camera` | `event_camera/image_events` (an event-frame visualisation: blue brighter, red darker) and `event_camera/event_rate` |
| `component_spad_camera` | `spad/binary` (single-photon binary frames, through a port of spadSim's photon model) and `spad/image` (a digitised reconstruction) |
