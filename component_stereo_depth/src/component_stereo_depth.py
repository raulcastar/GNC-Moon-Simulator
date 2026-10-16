"""
ROS 2 lifecycle node for the simulated stereo navcam rig (UnrealStereoCam).

Serves three cameras under one hierarchical namespace. The left and right eyes
publish colour; the central camera publishes colour plus the full depth product,
because depth and disparity are products of the *pair* and belong in the frame a
person actually looks at:

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

TF frame names flatten the namespace: navcam_left, navcam_right, navcam_central.

Extra mono cameras (extra_cameras, e.g. rear_cam) publish image_color (+
compressed, camera_info) and, when their entry names a mask capture, the same
object_mask / shadow_mask / shadow_label topics under their own namespace.

navcam/central/image_depth is the SGBM result reprojected onto the central
camera's pixel grid: planar Z in metres, NaN where no surface was recovered.
navcam/central/depth_ground_truth is Unreal's own depth for the same view (the
plugin's depth capture on the central camera): planar Z in metres, exact, +Inf
where there is no surface (sky).
Disparity has no separate visualisation: it is a monotonic function of depth, so
it would just render image_depth/visualization inverted.

Frames arrive over TCP from the UnrealStereoCam plugin: each SceneCapture
renders when a frame is due and is pushed here, with no game-thread stall. Every capture enqueued in one Unreal
tick carries the same frame id, which is what pairs the left/right eyes as one
rigid stereo view.

Rig geometry (baseline, focal length) is declared from parameters — stream mode
has nothing to measure it against — and the image size is taken from the first
frames.

This file is the ROS wiring: parameters, lifecycle, publishers and the cycle
that ties the parts together. The parts:
    frame_stream      TCP receiver for the plugin's frames (input/output)
    stereo_geometry   rig, SGBM matching, depth, reprojection (logic)
    masks             object and shadow masks, mask classes (logic)
    stereo_config     parameter parsing (logic)
    ros_messages      CameraInfo and DisparityImage construction
"""

import json
import os
import traceback
from typing import Dict, Optional

import cv2
import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer

from sensor_msgs.msg import Image as RosImage, CompressedImage, CameraInfo
from std_msgs.msg import String
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

from .cycle_timer import CycleTimer
from .frame_stream import FrameStreamReceiver
from . import masks
from . import stereo_geometry as geo
from .ros_messages import camera_info, disparity_msg, ros_time
from .stereo_config import frame_id, parse_extra_camera

# An extra camera's newest frame with its mask may trail its newest frame by
# this many Unreal frames (a mask still in flight); further behind, the mask
# stream has stopped and the colour is published alone.
MASK_PAIR_MAX_LAG = 30


class LifecycleStereoDepth(rclpy.lifecycle.Node):
    """Stereo matching node fed by the UnrealStereoCam plugin over TCP."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self._pubs: Dict[str, Publisher] = {}   # keyed by short name
        self._cached_color_map_json: str = "{}"
        # Rate and per-stage time of the read cycle, logged every 30 s.
        self._cycle_timer = CycleTimer(self.get_logger(), 'Stereo cycle')
        self._mask_classes_json: str = "[]"

        self._timer: Optional[Timer] = None
        self._extra_timer: Optional[Timer] = None
        # Extra mono cameras (extra_cameras), stereo_config.ExtraCamera each
        self._extras: list = []
        self._extra_last_fid: Dict[int, int] = {}
        self._extra_masks_on = False      # enable_object_mask: extra cameras' masks too
        self._shadow_ids_json = "{}"
        self.cv_bridge = CvBridge()
        self._is_active = False

        # Frames are pushed by the UnrealStereoCam plugin over TCP.
        self._stream: Optional[FrameStreamReceiver] = None
        self._last_stream_fid: int = -1

        # Rig and matcher, built on the first frame (the image size is needed).
        self._rig: Optional[geo.StereoRig] = None
        self._matcher: Optional[geo.StereoMatcher] = None

        self.declare_parameters(
            namespace='',
            parameters=[
                # The UnrealStereoCam plugin pushes frames to this TCP server.
                ('stream_bind_ip',          rclpy.Parameter.Type.STRING),
                ('stream_port',             rclpy.Parameter.Type.INTEGER),
                ('left_camera_id',          rclpy.Parameter.Type.INTEGER),
                ('right_camera_id',         rclpy.Parameter.Type.INTEGER),
                ('center_camera_id',        rclpy.Parameter.Type.INTEGER),
                # Stream cam id of the object-segmentation capture (UnrealStereoCam
                # MaskCamId). < 0 disables the mask.
                ('mask_camera_id',          rclpy.Parameter.Type.INTEGER),
                # Stream cam id of the plugin's ground-truth depth capture on the
                # central camera (UnrealStereoCam DepthCamId). < 0 disables it.
                ('depth_ground_truth_camera_id', rclpy.Parameter.Type.INTEGER),
                # Extra mono cameras streamed as they are (no stereo), e.g. a rear
                # camera: "<cam_id>:<topic>[:<hfov_deg>]" each, published on
                # <topic>/image_color (+ /compressed, + /camera_info when hfov_deg is
                # given) at extra_cameras_hz, independently of the stereo pair.
                ('extra_cameras',           rclpy.Parameter.Type.STRING_ARRAY),
                ('extra_cameras_hz',        rclpy.Parameter.Type.DOUBLE),
                # Object-mask classes (id, name, colour, group): a JSON file,
                # the same as Terrain Studio's config/mask_classes.json. Empty:
                # config/mask_classes.json of this package.
                ('mask_classes_file',       rclpy.Parameter.Type.STRING),
                # Topic namespaces, one per physical camera
                ('left_topic',              rclpy.Parameter.Type.STRING),
                ('right_topic',             rclpy.Parameter.Type.STRING),
                ('center_topic',            rclpy.Parameter.Type.STRING),
                ('image_reading_hz_rate',   rclpy.Parameter.Type.DOUBLE),
                # Rig geometry, declared (a stream cannot measure it); both
                # are required.
                ('baseline_m',              rclpy.Parameter.Type.DOUBLE),
                ('hfov_deg',                rclpy.Parameter.Type.DOUBLE),
                # Centre camera's distance from the left eye along the baseline
                # (metres). < 0 means the midpoint.
                ('center_offset_m',         rclpy.Parameter.Type.DOUBLE),
                # Matching
                ('num_disparities',         rclpy.Parameter.Type.INTEGER),
                ('max_depth_m',             rclpy.Parameter.Type.DOUBLE),
                ('block_size',              rclpy.Parameter.Type.INTEGER),
                ('uniqueness_ratio',        rclpy.Parameter.Type.INTEGER),
                ('speckle_window_size',     rclpy.Parameter.Type.INTEGER),
                ('speckle_range',           rclpy.Parameter.Type.INTEGER),
                ('equalize_for_glare',      rclpy.Parameter.Type.BOOL),
                ('fill_left_border',        rclpy.Parameter.Type.BOOL),
                ('debug_mode',              rclpy.Parameter.Type.BOOL),
                # Central camera products
                ('depth_max_distance',      rclpy.Parameter.Type.DOUBLE),
                ('enable_object_mask',      rclpy.Parameter.Type.BOOL),
                ('enable_shadow_mask',      rclpy.Parameter.Type.BOOL),
            ])

    # =========================================================================
    # Object-mask legend
    # =========================================================================

    def _load_mask_classes(self):
        """
        The object-mask classes from mask_classes_file, as the two JSON strings
        published with the mask (masks.mask_class_messages). The mask itself
        only carries colours, so this file is the record of what each id in the
        Unreal scene means; Terrain Studio bakes the same file into the mask
        material. Malformed entries are skipped with a warning.
        """
        path = self.get_parameter('mask_classes_file').value or os.path.join(
            get_package_share_directory('component_stereo_depth'), 'config', 'mask_classes.json')
        try:
            with open(path, encoding='utf-8') as handle:
                entries = json.load(handle).get('classes') or []
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Object-mask classes unreadable ({path}): {exc}. '
                                    'object_mask_mapping will publish "{}".')
            return '{}', '[]'
        classes, skipped = masks.parse_mask_classes(entries)
        for entry in skipped:
            self.get_logger().warn(f'{path}: ignoring malformed class {entry}.')
        if classes:
            self.get_logger().info(
                f'Object mask classes ({len(classes)}, {path}): '
                + ', '.join(f'{c["id"]}={c["name"]}' for c in classes))
        else:
            self.get_logger().warn(f'{path} lists no classes; object_mask_mapping will publish "{{}}".')
        return masks.mask_class_messages(classes)

    # =========================================================================
    # Capture
    # =========================================================================

    def _solve_rig(self, eye_shape, center_shape) -> None:
        """Rig and matcher from parameters and the first streamed frames."""
        self._rig = geo.solve_rig(eye_shape, center_shape, self.hfov_deg_param,
                                  self.baseline_m_param, self.center_offset_m)
        self._matcher = geo.StereoMatcher(
            self._rig, self.num_disparities_param, self.block_size, self.uniqueness_ratio,
            self.speckle_window_size, self.speckle_range, self.max_depth_m,
            self.fill_left_border, self.equalize_for_glare)
        log = self.get_logger()
        for level, message in self._matcher.report():
            (log.warn if level == 'warn' else log.info)(message)
        rig = self._rig
        log.info(f'Rig declared (stream mode): {rig.width}x{rig.height} | '
                 f'baseline {rig.baseline_m * 100:.2f} cm | '
                 f'hfov {self.hfov_deg_param:.2f} deg -> fx {rig.fx:.2f} px | '
                 f'centre {rig.c_offset_m * 100:.2f} cm from the left eye')

    def _capture_from_stream(self):
        """
        Take the newest same-tick pair from the UnrealStereoCam plugin.

        Returns None when there is nothing new to process; the cycle simply
        skips. Geometry is solved from parameters on the first frame, when the
        image size becomes known.
        """
        got = self._stream.latest_pair(self.left_camera_id,
                                       self.right_camera_id,
                                       self.center_camera_id)
        if got is None:
            self.get_logger().warn(
                'No streamed frames yet. Is the UnrealStereoCam component in '
                'the level, and the game playing?', throttle_duration_sec=15.0)
            return None
        left, right, center, fid, age, stamp_ns = got

        if fid == self._last_stream_fid:
            return None                      # nothing new this cycle
        if age > 3.0:
            self.get_logger().warn(
                f'Streamed frames are {age:.1f} s old — the stream has '
                f'stalled. Skipping.', throttle_duration_sec=15.0)
            return None

        if self._matcher is None:
            self._solve_rig(left.shape, center.shape)
        elif left.shape[:2] != (self._rig.height, self._rig.width):
            self.get_logger().error(
                f'Streamed frame is {left.shape[1]}x{left.shape[0]} but the '
                f'rig was solved at {self._rig.width}x{self._rig.height}. Restart the '
                f'node after changing the render target size.',
                throttle_duration_sec=15.0)
            return None
        self._last_stream_fid = fid

        # Object mask (optional): rendered by a separate SceneCapture whose
        # post-process material colours the Custom Depth-Stencil buffer, and
        # streamed on its own cam id. It is already RGB (the receiver swaps
        # BGRA->RGB) and flat-coloured, so it publishes as-is.
        mask = None
        if self.enable_object_mask and self.mask_camera_id >= 0:
            mask = self._stream.frame_near(self.mask_camera_id, fid)
        # Ground-truth depth (optional): the plugin's depth capture on the
        # central camera, centimetres of planar depth, on its own cam id.
        depth_gt = None
        if self.depth_gt_camera_id >= 0:
            depth_gt = self._stream.frame_near(self.depth_gt_camera_id, fid)
        return left, right, center, mask, depth_gt, stamp_ns

    # =========================================================================
    # Publishing
    # =========================================================================

    def _wanted(self, key: str) -> bool:
        """
        Whether anything subscribes to output `key` now. Outputs nobody reads
        (raw eye images, disparity, raw ground truth...) are not built or
        published; a subscriber (a viewer, a bag recording) brings one back on
        the next frame.
        """
        pub = self._pubs.get(key)
        return pub is not None and pub.get_subscription_count() > 0

    def _publish_image(self, key: str, img: np.ndarray, encoding: str,
                       stamp, frame: str) -> None:
        if not self._wanted(key):
            return
        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        self._pubs[key].publish(msg)

    def _publish_color(self, key: str, rgb: np.ndarray, stamp, frame: str) -> None:
        """Raw rgb8 plus a JPEG-compressed copy, on <key> and <key>_compressed."""
        self._publish_image(key, rgb, 'rgb8', stamp, frame)
        if not self._wanted(key + '_compressed'):
            return

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        msg = self.cv_bridge.cv2_to_compressed_imgmsg(bgr, dst_format='jpg')
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        self._pubs[key + '_compressed'].publish(msg)

    def _publish_masks(self, prefix: str, frame_rgb: np.ndarray, stamp, frame: str) -> None:
        """
        One mask capture frame as the object mask (flag bits cleared) and, with
        enable_shadow_mask, the shadow feeds split from its flag bits; with the
        legends. Outputs `<prefix>_object_mask`, ... (see _mask_publishers).
        """
        if self.enable_shadow_mask:
            shadow, labels, frame_rgb = masks.split_shadow_mask(frame_rgb)
            self._publish_image(f'{prefix}_shadow_mask', shadow, 'rgb8', stamp, frame)
            self._publish_image(f'{prefix}_shadow_label', labels, 'mono8', stamp, frame)
            self._pubs[f'{prefix}_shadow_label_mapping'].publish(String(data=self._shadow_ids_json))
        self._publish_image(f'{prefix}_object_mask', frame_rgb, 'rgb8', stamp, frame)
        self._pubs[f'{prefix}_object_mask_mapping'].publish(String(data=self._cached_color_map_json))
        self._pubs[f'{prefix}_object_mask_classes'].publish(String(data=self._mask_classes_json))

    # =========================================================================
    # Timer callbacks
    # =========================================================================

    def read_stereo_pair(self) -> None:
        if not self._is_active:
            return

        timer = self._cycle_timer
        try:
            stamp = self.get_clock().now().to_msg()
            f_left, f_right, f_center = (self.left_frame, self.right_frame,
                                         self.center_frame)

            timer.start()
            captured = self._capture_from_stream()
            if captured is None:
                timer.start()                # nothing new: not a cycle
                return
            left, right, center, center_mask, depth_gt_cm, stamp_ns = captured
            if stamp_ns:                     # capture time from Unreal (simulation clock)
                stamp = ros_time(stamp_ns)
            rig, matcher = self._rig, self._matcher
            timer.mark('frames')
            disp = matcher.disparity(left, right)
            depth_left = matcher.depth(disp)
            timer.mark('matching')

            # The eyes carry colour only: depth is a product of the pair.
            eye = (rig.fx, rig.cx, rig.cy, rig.width, rig.height)
            self._publish_color('left_color', left, stamp, f_left)
            self._publish_color('right_color', right, stamp, f_right)
            self._pubs['left_info'].publish(camera_info(stamp, f_left, eye))
            self._pubs['right_info'].publish(camera_info(stamp, f_right, eye, rig.baseline_m))

            # The central camera is the RGB-D sensor this rig simulates.
            self._publish_color('center_color', center, stamp, f_center)
            self._pubs['center_info'].publish(camera_info(stamp, f_center, rig.center_intrinsics))
            timer.mark('colour')

            # Warped onto the central pixel grid, so depth, disparity and RGB
            # all overlay one another.
            depth_c = geo.reproject_to_center(depth_left, rig, matcher.near_limit_m)
            if depth_c is not None:
                depth_pub = geo.clip_depth(depth_c, self.depth_max_distance)
                self._publish_image('center_depth', depth_pub, '32FC1', stamp, f_center)
                if self._wanted('center_depth_vis'):
                    self._publish_image('center_depth_vis',
                                        geo.colourise(depth_pub, self.depth_max_distance),
                                        'bgr8', stamp, f_center)
                if self._wanted('center_disparity'):
                    # With the border recovered, and after reprojection, validity
                    # is per pixel (d <= u): the valid window is the whole image.
                    self._pubs['center_disparity'].publish(disparity_msg(
                        self.cv_bridge, geo.center_disparity(depth_c, rig), stamp, f_center,
                        rig.c_fx, rig.baseline_m, matcher.num_disparities,
                        rig.c_width, rig.c_height))
            timer.mark('depth')

            if depth_gt_cm is not None and (self._wanted('center_depth_gt')
                                            or self._wanted('center_depth_gt_vis')):
                if depth_gt_cm.shape != center.shape[:2]:
                    self.get_logger().warn(
                        f'Ground-truth depth is {depth_gt_cm.shape[1]}x{depth_gt_cm.shape[0]}, '
                        f'the central image {center.shape[1]}x{center.shape[0]}: skipped.',
                        throttle_duration_sec=15.0)
                else:
                    depth_gt = geo.ground_truth_depth_m(depth_gt_cm)
                    self._publish_image('center_depth_gt', depth_gt, '32FC1', stamp, f_center)
                    if self._wanted('center_depth_gt_vis'):
                        shown = np.where(np.isfinite(depth_gt),
                                         np.clip(depth_gt, 0.0, self.depth_max_distance), np.nan)
                        self._publish_image('center_depth_gt_vis',
                                            geo.colourise(shown, self.depth_max_distance),
                                            'bgr8', stamp, f_center)
            timer.mark('ground truth')

            if center_mask is not None:
                self._publish_masks('center', center_mask, stamp, f_center)
            timer.mark('masks')
            timer.end()

            if self.debug_mode:
                matched = np.isfinite(disp).mean() * 100
                filled = (np.isfinite(depth_c).mean() * 100
                          if depth_c is not None else 0.0)
                self.get_logger().info(
                    f'matched {matched:.1f}% of the left frame | '
                    f'{filled:.1f}% of the central frame received depth')

        except Exception as exc:
            self.get_logger().error(f'Stereo read/publish cycle failed: {exc}')
            return

    def publish_extra_cameras(self) -> None:
        """Newest frame of each extra mono camera, published once per new frame."""
        if not self._is_active or self._stream is None:
            return
        stamp = self.get_clock().now().to_msg()
        for cam, frame in self._extras:
            try:
                got = self._stream.latest(cam.cam_id)
                if got is None:
                    continue
                fid, rgb, stamp_ns = got
                mask = None
                if self._extra_masks_on and cam.mask_cam_id is not None:
                    # Its mask capture renders in the same tick (same frame id) but
                    # may land a moment later: take the newest frame both have.
                    # Without a recent pair (no mask stream), the colour goes alone.
                    pair = self._stream.latest_with(cam.cam_id, cam.mask_cam_id)
                    if pair is not None and fid - pair[0] <= MASK_PAIR_MAX_LAG:
                        fid, rgb, stamp_ns, mask = pair
                if fid == self._extra_last_fid.get(cam.cam_id):
                    continue
                if rgb.ndim != 3:                   # a depth (R32F) stream: not a colour camera
                    continue
                self._extra_last_fid[cam.cam_id] = fid
                shot = ros_time(stamp_ns) if stamp_ns else stamp      # capture time when known
                self._publish_color(f'{cam.key}_color', rgb, shot, frame)
                if mask is not None and mask.ndim == 3:
                    self._publish_masks(cam.key, mask, shot, frame)
                if cam.hfov_deg:
                    h, w = rgb.shape[:2]
                    fx = geo.focal_from_hfov(w, cam.hfov_deg)
                    self._pubs[f'{cam.key}_info'].publish(
                        camera_info(shot, frame, (fx, w / 2.0, h / 2.0, w, h)))
            except Exception as exc:
                self.get_logger().error(f'Extra camera {cam.cam_id} publish failed: {exc}',
                                        throttle_duration_sec=10.0)

    # =========================================================================
    # Lifecycle transitions
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        rclpy reports a raising on_configure as a bare 'Failed to make
        transition', discarding the exception. Catch it so the reason reaches the
        log, and release the stream server so a failed configure does not leave
        its listening socket bound and block the retry.
        """
        try:
            return self._configure()
        except Exception as exc:
            self.get_logger().error(f'Configuration failed: {exc}')
            self.get_logger().error(traceback.format_exc())
            self._release()
            return TransitionCallbackReturn.FAILURE

    def _configure(self) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle state transition: CONFIGURED.')

        self.left_camera_id   = self.get_parameter('left_camera_id').value
        self.right_camera_id  = self.get_parameter('right_camera_id').value
        self.center_camera_id = self.get_parameter('center_camera_id').value
        self.mask_camera_id   = self.get_parameter('mask_camera_id').value
        self.depth_gt_camera_id = self.get_parameter('depth_ground_truth_camera_id').value
        if self.depth_gt_camera_id is None:
            self.depth_gt_camera_id = -1
        self.left_topic       = self.get_parameter('left_topic').value
        self.right_topic      = self.get_parameter('right_topic').value
        self.center_topic     = self.get_parameter('center_topic').value
        self.baseline_m_param = self.get_parameter('baseline_m').value
        self.hfov_deg_param   = self.get_parameter('hfov_deg').value
        self.center_offset_m  = self.get_parameter('center_offset_m').value
        self.num_disparities_param = self.get_parameter('num_disparities').value
        self.max_depth_m      = self.get_parameter('max_depth_m').value
        self.block_size       = self.get_parameter('block_size').value
        self.uniqueness_ratio = self.get_parameter('uniqueness_ratio').value
        self.speckle_window_size = self.get_parameter('speckle_window_size').value
        self.speckle_range    = self.get_parameter('speckle_range').value
        self.equalize_for_glare = self.get_parameter('equalize_for_glare').value
        self.fill_left_border = self.get_parameter('fill_left_border').value
        self.debug_mode       = self.get_parameter('debug_mode').value
        self.depth_max_distance = self.get_parameter('depth_max_distance').value
        self.enable_object_mask = self.get_parameter('enable_object_mask').value
        self.enable_shadow_mask = (self.enable_object_mask
                                   and self.get_parameter('enable_shadow_mask').value)
        # The switches cover the extra cameras' masks too (extra_cameras with a
        # mask cam id); mask_camera_id < 0 turns off only the central one.
        self._extra_masks_on = bool(self.enable_object_mask)
        hz                    = self.get_parameter('image_reading_hz_rate').value

        # Frames come from the UnrealStereoCam plugin and the rig geometry from
        # parameters. Nothing can be measured, so the two geometry inputs are
        # mandatory.
        if self.baseline_m_param <= 0.0 or self.hfov_deg_param <= 0.0:
            raise ValueError(
                'the rig cannot be measured from a stream: set baseline_m and '
                'hfov_deg to the real rig values (they are required).')
        # The object mask is served by a dedicated UnrealStereoCam capture
        # (post-process material over the Custom Depth-Stencil buffer). It works
        # as long as mask_camera_id names that capture.
        if self.enable_object_mask:
            if self.mask_camera_id is None or self.mask_camera_id < 0:
                self.get_logger().warn(
                    'enable_object_mask is on but mask_camera_id is unset '
                    '(< 0): the mask needs the UnrealStereoCam segmentation '
                    'capture. Disabling the object mask.')
                self.enable_object_mask = False
            else:
                self.get_logger().info(
                    f'Object mask served from stream cam id '
                    f'{self.mask_camera_id} (UnrealStereoCam segmentation '
                    f'capture).')
        # Intrinsics need the frame size; solved when the first frame lands.
        self._rig = self._matcher = None

        L, R, C = self.left_topic, self.right_topic, self.center_topic

        # Topic namespaces are hierarchical; TF frame names cannot contain '/'.
        self.left_frame = frame_id(L)
        self.right_frame = frame_id(R)
        self.center_frame = frame_id(C)

        def pub(key, msg_type, topic):
            self._pubs[key] = self.create_lifecycle_publisher(msg_type, topic, 10)

        for key, ns in (('left', L), ('right', R), ('center', C)):
            pub(f'{key}_color', RosImage, f'{ns}/image_color')
            pub(f'{key}_color_compressed', CompressedImage, f'{ns}/image_color/compressed')
            pub(f'{key}_info', CameraInfo, f'{ns}/camera_info')

        # The central camera is the RGB-D sensor this rig simulates.
        pub('center_depth', RosImage, f'{C}/image_depth')
        pub('center_depth_vis', RosImage, f'{C}/image_depth/visualization')
        pub('center_disparity', DisparityImage, f'{C}/disparity')
        if self.depth_gt_camera_id >= 0:
            pub('center_depth_gt', RosImage, f'{C}/depth_ground_truth')
            pub('center_depth_gt_vis', RosImage, f'{C}/depth_ground_truth/visualization')

        def mask_publishers(prefix, ns):
            """The object mask (and shadow feeds) of one camera namespace."""
            pub(f'{prefix}_object_mask', RosImage, f'{ns}/object_mask')
            pub(f'{prefix}_object_mask_mapping', String, f'{ns}/object_mask_mapping')
            pub(f'{prefix}_object_mask_classes', String, f'{ns}/object_mask_classes')
            if self.enable_shadow_mask:
                pub(f'{prefix}_shadow_mask', RosImage, f'{ns}/shadow_mask')
                pub(f'{prefix}_shadow_label', RosImage, f'{ns}/shadow_label')
                pub(f'{prefix}_shadow_label_mapping', String, f'{ns}/shadow_label_mapping')

        extra_entries = [parse_extra_camera(e) for e in self.get_parameter('extra_cameras').value or []]
        if self.enable_object_mask or (self._extra_masks_on and any(
                c is not None and c.mask_cam_id is not None for c in extra_entries)):
            # The colours are baked into the Unreal post-process material from the
            # same class file, so publishing it tells a consumer what each id and
            # colour in the mask represents.
            self._cached_color_map_json, self._mask_classes_json = self._load_mask_classes()
            self._shadow_ids_json = masks.shadow_ids_json()
        if self.enable_object_mask:
            mask_publishers('center', C)

        self.get_logger().info(
            f'Publishing {L}/*, {R}/* (colour) and {C}/* '
            f'(colour + PLANAR depth + disparity'
            f'{" + ground-truth depth" if self.depth_gt_camera_id >= 0 else ""}'
            f'{" + object mask" if self.enable_object_mask else ""}'
            f'{" + shadow mask" if self.enable_shadow_mask else ""}). '
            f'Frames: {self.left_frame}, {self.right_frame}, {self.center_frame}.')

        # Extra mono cameras: "<cam_id>:<topic>[:<hfov_deg>[:<mask_cam_id>]]".
        self._extras, self._extra_last_fid = [], {}
        for entry, cam in zip(self.get_parameter('extra_cameras').value or [], extra_entries):
            if cam is None:
                self.get_logger().warn(f'extra_cameras: ignored "{entry}" '
                                       '(expected "<cam_id>:<topic>[:<hfov_deg>[:<mask_cam_id>]]").')
                continue
            pub(f'{cam.key}_color', RosImage, f'{cam.topic}/image_color')
            pub(f'{cam.key}_color_compressed', CompressedImage, f'{cam.topic}/image_color/compressed')
            if cam.hfov_deg:
                pub(f'{cam.key}_info', CameraInfo, f'{cam.topic}/camera_info')
            masked = self._extra_masks_on and cam.mask_cam_id is not None
            if masked:
                mask_publishers(cam.key, cam.topic)
            self._extras.append((cam, frame_id(cam.topic)))
            self.get_logger().info(
                f'Extra camera: stream cam id {cam.cam_id} -> {cam.topic}/image_color'
                f'{f" (hfov {cam.hfov_deg:g} deg)" if cam.hfov_deg else ""}'
                f'{f"; object mask from cam id {cam.mask_cam_id} -> {cam.topic}/object_mask" if masked else ""}'
                f'{" + shadow mask" if masked and self.enable_shadow_mask else ""}.')

        self._stream = FrameStreamReceiver(
            self.get_parameter('stream_bind_ip').value,
            self.get_parameter('stream_port').value,
            self.get_logger())
        # Every camera id a parameter names, so a frame on another id (an Unreal
        # component configured with a different id) is reported once.
        self._stream.known_ids = {i for i in (
            self.left_camera_id, self.right_camera_id, self.center_camera_id,
            self.mask_camera_id if self.enable_object_mask else -1,
            self.depth_gt_camera_id) if i is not None and i >= 0} | {c.cam_id for c, _ in self._extras} | {
            c.mask_cam_id for c, _ in self._extras if c.mask_cam_id is not None}
        self._stream.start()

        self._timer = self.create_timer(1.0 / hz, self.read_stereo_pair)
        if self._extras:
            extra_hz = self.get_parameter('extra_cameras_hz').value or hz
            self._extra_timer = self.create_timer(1.0 / extra_hz, self.publish_extra_cameras)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle state transition: ACTIVE.')
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle state transition: DEACTIVATED.')
        self._is_active = False
        return super().on_deactivate(state)

    def _release(self) -> None:
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._extra_timer:
            self.destroy_timer(self._extra_timer)
            self._extra_timer = None

        if self._stream:
            self._stream.stop()
            self._stream = None

        # getattr: _release also runs from a failed on_configure, which may not
        # have created any publisher.
        for pub in getattr(self, '_pubs', {}).values():
            self.destroy_publisher(pub)
        if hasattr(self, '_pubs'):
            self._pubs.clear()

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle state transition: CLEANING UP.')
        self._is_active = False
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle state transition: SHUTTING DOWN.')
        self._is_active = False
        self._release()
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = LifecycleStereoDepth('lc_component_stereo_depth')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
