"""
ROS 2 Lifecycle Node for the simulated stereo navcam rig (UnrealCV).

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
    navcam/right/camera_info             navcam/central/depth_gt
                                         navcam/central/object_mask
                                         navcam/central/object_mask_mapping

TF frame names flatten the namespace: navcam_left, navcam_right, navcam_central.

navcam/central/image_depth is the SGBM result reprojected onto the central
camera's pixel grid: planar Z in metres, NaN where no surface was recovered.
Unreal's exact depth buffer is published on depth_gt, for scoring it. Disparity
has no separate visualisation: it is a monotonic function of depth, so it would
just render image_depth/visualization inverted.

This node must own the UnrealCV socket and serve every camera:

  * The server accepts one client connection at a time.
  * Each vget is a separate TCP round-trip. The rover moves between the left and
    right grabs unless the process issuing them brackets the whole capture with
    "vset /action/game/pause". All three cameras are read inside one pause
    window, so the sensor suite shares a single instant of world state.

Rig geometry (baseline, focal length, rectification) is read from the running
Unreal instance at configure time and validated.
"""

from typing import Dict, Optional, Tuple
import math
import time
import re
import json
import traceback

import numpy as np
import cv2

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer

from sensor_msgs.msg import Image as RosImage, CompressedImage, CameraInfo, RegionOfInterest
from std_msgs.msg import String
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

from unrealcv import Client
from unrealcv.util import read_png, read_npy

# float16 max — UnrealCV writes this into the depth buffer for sky/background.
_FLOAT16_MAX = 65504.0


def _frame_id(topic: str) -> str:
    """
    Topic namespace -> TF frame name: 'navcam/central' -> 'navcam_central'.

    tf2 reserves '/' in frame names for the deprecated tf_prefix syntax.
    """
    return topic.strip('/').replace('/', '_')


def unreal_axes(pitch_deg: float, yaw_deg: float,
                roll_deg: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Forward / right / up unit vectors for an Unreal FRotator, in world space.

    Mirrors FRotationMatrix: X = forward, Y = right, Z = up, left-handed.
    """
    p, y, r = map(math.radians, (pitch_deg, yaw_deg, roll_deg))
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    cr, sr = math.cos(r), math.sin(r)

    forward = np.array([cp * cy, cp * sy, sp])
    right = np.array([sr * sp * cy - cr * sy,
                      sr * sp * sy + cr * cy,
                      -sr * cp])
    up = np.array([-(cr * sp * cy + sr * sy),
                   cy * sr - cr * sp * sy,
                   cr * cp])
    return forward, right, up


def radial_to_planar(point_depth: np.ndarray, fx: float, fy: float,
                     cx: float, cy: float) -> np.ndarray:
    """
    UnrealCV point (radial) depth -> planar Z depth along the optical axis.

    The buffer stores straight-line distance from the camera centre; stereo
    triangulation yields the Z component. At 90 deg HFOV they differ by 60% in
    the image corners.

    unrealcv.util.convert2planedepth does this but calls np.float, which NumPy
    removed in 1.24.
    """
    h, w = point_depth.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float64),
                       np.arange(h, dtype=np.float64))
    px = (u - cx) / fx
    py = (v - cy) / fy
    return point_depth / np.sqrt(1.0 + px ** 2 + py ** 2)


class LifecycleStereoDepth(rclpy.lifecycle.Node):
    """Stereo matching node for a simulated UnrealCV stereo rig."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self._pubs: Dict[str, Publisher] = {}   # keyed by short name
        self._cached_color_map_json: str = "{}"

        self._timer: Optional[Timer] = None
        self.unreal_client: Optional[Client] = None
        self.cv_bridge = CvBridge()
        self._is_active = False

        # Rig solution, filled in by _solve_rig during on_configure.
        self._baseline_m: float = 0.0
        self._fx: float = 0.0
        self._cx: float = 0.0
        self._cy: float = 0.0
        self._width: int = 0
        self._height: int = 0
        self._swap_lr: bool = False

        self._matcher: Optional[cv2.StereoSGBM] = None
        self._num_disparities: int = 0
        self._near_limit_m: float = 0.0     # fx*B/num_disparities, set in _build_matcher
        self._clahe = None

        # {camera_id: (location, rotation)}, sampled at one instant
        self._geom: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip',                      rclpy.Parameter.Type.STRING),
                ('port',                    rclpy.Parameter.Type.INTEGER),
                ('left_camera_id',          rclpy.Parameter.Type.INTEGER),
                ('right_camera_id',         rclpy.Parameter.Type.INTEGER),
                ('center_camera_id',        rclpy.Parameter.Type.INTEGER),
                # Topic namespaces, one per physical camera
                ('left_topic',              rclpy.Parameter.Type.STRING),
                ('right_topic',             rclpy.Parameter.Type.STRING),
                ('center_topic',            rclpy.Parameter.Type.STRING),
                ('image_reading_hz_rate',   rclpy.Parameter.Type.DOUBLE),
                ('pause_game_during_capture', rclpy.Parameter.Type.BOOL),
                # Rig geometry. Both are auto-solved from Unreal when <= 0.
                ('baseline_m',              rclpy.Parameter.Type.DOUBLE),
                ('hfov_deg',                rclpy.Parameter.Type.DOUBLE),
                ('uu_per_metre',            rclpy.Parameter.Type.DOUBLE),
                # Matching
                ('num_disparities',         rclpy.Parameter.Type.INTEGER),
                ('max_depth_m',             rclpy.Parameter.Type.DOUBLE),
                ('block_size',              rclpy.Parameter.Type.INTEGER),
                ('uniqueness_ratio',        rclpy.Parameter.Type.INTEGER),
                ('speckle_window_size',     rclpy.Parameter.Type.INTEGER),
                ('speckle_range',           rclpy.Parameter.Type.INTEGER),
                ('equalize_for_glare',      rclpy.Parameter.Type.BOOL),
                ('fill_left_border',        rclpy.Parameter.Type.BOOL),
                # Glare study
                ('saturation_level',        rclpy.Parameter.Type.INTEGER),
                ('validate_against_ground_truth', rclpy.Parameter.Type.BOOL),
                ('debug_mode',              rclpy.Parameter.Type.BOOL),
                # Central camera products
                ('depth_max_distance',      rclpy.Parameter.Type.DOUBLE),
                ('enable_object_mask',      rclpy.Parameter.Type.BOOL),
            ])

    # =========================================================================
    # UnrealCV connection and rig solving
    # =========================================================================

    def _connect_unrealcv(self, ip: str, port: int) -> None:
        self.get_logger().info(f'Attempting connection to UnrealCV at {ip}:{port}...')
        self.unreal_client = Client((ip, port), 'inet')
        while not self.unreal_client.isconnected():
            self.unreal_client.connect()
            time.sleep(1.0)
        self.get_logger().info('UnrealCV TCP connection established.')

    def _request_raw(self, cmd: str, retries: int = 3):
        """
        Issue a UnrealCV command and insist on a usable reply, preserving type.

        Client.request returns None on a dropped reply, str for text commands,
        and raw bytes for image payloads. Both failure modes otherwise surface
        far away as a NoneType crash.

        The reply is returned untouched: decoding PNG bytes as UTF-8 corrupts
        them into something read_png parses as None.
        """
        for _ in range(retries):
            res = self.unreal_client.request(cmd)
            if res is None:
                continue
            err = (res.lower().startswith('error') if isinstance(res, str)
                   else res[:5].lower() == b'error')
            if err:
                raise RuntimeError(f'UnrealCV rejected "{cmd}": {res[:120]!r}')
            return res
        raise RuntimeError(f'UnrealCV gave no reply to "{cmd}" after {retries} tries')

    def _request(self, cmd: str, retries: int = 3) -> str:
        """Text-valued command. Use _request_raw for png/npy payloads."""
        res = self._request_raw(cmd, retries)
        if isinstance(res, bytes):
            res = res.decode('utf-8', 'replace')
        return res

    def _get_vec3(self, cam_id: int, what: str) -> np.ndarray:
        cmd = f'vget /camera/{cam_id}/{what}'
        res = self._request(cmd)
        try:
            vals = [float(v) for v in res.split()]
        except ValueError:
            raise RuntimeError(f'Could not parse "{cmd}" reply as 3 floats: {res!r}')
        if len(vals) != 3:
            raise RuntimeError(f'"{cmd}" returned {len(vals)} values, expected 3: {res!r}')
        return np.array(vals)

    def _build_color_mapping(self) -> str:
        """
        Map every scene object name to its segmentation colour.

        One TCP round-trip per object, so it runs once at configure time. The
        JSON is republished verbatim with every object_mask frame. Objects whose
        colour cannot be read are skipped and counted.
        """
        self.get_logger().info('Downloading object colour mapping from Unreal...')

        res_objects = self._request('vget /objects')
        if not res_objects:
            self.get_logger().warn('No objects found; semantic mapping will be empty.')
            return "{}"

        # split() not split(' '): the reply carries trailing whitespace, and an
        # empty name would ask for "vget /object//color".
        object_ids = res_objects.split()

        color_regex = re.compile(r'\(R=(.*),G=(.*),B=(.*),A=(.*)\)')
        color_mapping = {}
        failed = 0
        for obj_id in object_ids:
            try:
                color_str = self._request(f'vget /object/{obj_id}/color', retries=1)
            except RuntimeError:
                failed += 1
                continue
            match = color_regex.match(color_str) if color_str else None
            if match:
                color_mapping[obj_id] = [int(match.group(1)),
                                         int(match.group(2)),
                                         int(match.group(3))]
            else:
                failed += 1

        self.get_logger().info(
            f'Mapped {len(color_mapping)} of {len(object_ids)} objects'
            + (f' ({failed} gave no colour).' if failed else '.'))
        return json.dumps(color_mapping)

    def _check_cameras_exist(self) -> None:
        """
        Verify the configured camera ids exist, before a missing one surfaces as
        an unreadable PNG or an unparseable float.
        """
        n_cams = len(self._request('vget /cameras').split())
        needed = {self.left_camera_id, self.right_camera_id, self.center_camera_id}
        missing = sorted(c for c in needed if c >= n_cams)
        if missing:
            raise RuntimeError(
                f'Unreal reports {n_cams} camera(s), valid ids 0..{n_cams - 1}, '
                f'but this node is configured to use {missing}. Add the cameras '
                f'in Unreal, or fix left/right/center_camera_id.')
        self.get_logger().info(
            f'Unreal reports {n_cams} cameras; using left={self.left_camera_id} '
            f'right={self.right_camera_id} center={self.center_camera_id}')

    def _snapshot_geometry(self) -> None:
        """
        Read all three camera poses inside one pause window.

        Each vget is a separate TCP round-trip and the rover keeps driving
        between them. Sampling the left eye and the centre camera 0.3 s apart
        subtracts two different world states: cameras bolted to the same bar
        come back non-collinear and mutually rotated.
        """
        cams = [self.left_camera_id, self.right_camera_id, self.center_camera_id]
        paused = False
        if self.pause_game_during_capture:
            paused = self.unreal_client.request('vset /action/game/pause') == 'ok'
        if not paused:
            self.get_logger().warn(
                'Geometry snapshot taken with the game running: the measured '
                'baseline and camera offsets will include the rover\'s own '
                'motion between queries.')
        try:
            self._geom = {c: (self._get_vec3(c, 'location'),
                              self._get_vec3(c, 'rotation')) for c in cams}
        finally:
            if paused:
                self.unreal_client.request('vset /action/game/resume')

    def _solve_rig(self) -> None:
        """
        Verify the pair is rectified and derive the intrinsics.

        SGBM assumes epipolar lines are image rows, which holds only when both
        cameras share a rotation and are offset purely along the left camera's
        right axis. Violations are logged as errors.
        """
        probe = self._grab_rgb(self.left_camera_id)
        self._height, self._width = probe.shape[:2]
        self._cx = self._width / 2.0
        self._cy = self._height / 2.0

        loc_l, rot_l = self._geom[self.left_camera_id]
        loc_r, rot_r = self._geom[self.right_camera_id]

        rot_err = float(np.abs(rot_r - rot_l).max())
        if rot_err > 1e-3:
            self.get_logger().error(
                f'Left/right rotations differ by up to {rot_err:.4f} deg '
                f'({rot_l} vs {rot_r}). The pair is NOT rectified: epipolar '
                f'lines are not image rows and the disparity will be wrong.')

        _, right_vec, _ = unreal_axes(*rot_l)
        baseline_vec = loc_r - loc_l                       # Unreal Units
        lateral = float(np.dot(baseline_vec, right_vec))   # signed, along +right
        residual = float(np.linalg.norm(baseline_vec - lateral * right_vec))
        if abs(lateral) < 1e-6:
            raise RuntimeError(
                f'Cameras {self.left_camera_id} and {self.right_camera_id} are '
                f'co-located along the lateral axis; there is no stereo baseline.')
        if residual > 0.01 * abs(lateral):
            self.get_logger().error(
                f'Baseline is not purely lateral: {residual:.2f} UU of the '
                f'{np.linalg.norm(baseline_vec):.2f} UU offset is vertical or '
                f'forward. Offset the right camera along the left camera\'s '
                f'right axis only.')

        self._swap_lr = lateral < 0
        if self._swap_lr:
            self.get_logger().warn(
                f'Camera {self.right_camera_id} sits to the LEFT of camera '
                f'{self.left_camera_id}; the images will be swapped before matching.')

        # A configured baseline overrides the measurement; useful when the rig
        # is parented to a moving actor and the query returns world coordinates
        # that momentarily disagree.
        if self.baseline_m_param > 0.0:
            self._baseline_m = self.baseline_m_param
        else:
            self._baseline_m = abs(lateral) / self.uu_per_metre

        hfov = self.hfov_deg_param
        if hfov <= 0.0:
            try:
                hfov = float(self._request(f'vget /camera/{self.left_camera_id}/fov'))
            except (RuntimeError, TypeError, ValueError) as exc:
                raise RuntimeError(
                    f'Could not read the camera FOV from Unreal ({exc}). Older '
                    f'UnrealCV server plugins lack "vget /camera/N/fov"; set the '
                    f'hfov_deg parameter explicitly to skip this query.')

            try:
                fov_r = float(self._request(f'vget /camera/{self.right_camera_id}/fov'))
                if abs(fov_r - hfov) > 1e-3:
                    self.get_logger().error(
                        f'Left FOV {hfov} deg != right FOV {fov_r} deg. The pair '
                        f'has different intrinsics and is not a stereo rig.')
            except (RuntimeError, TypeError, ValueError):
                pass

        self._fx = (self._width / 2.0) / math.tan(math.radians(hfov) / 2.0)

        self.get_logger().info(
            f'Rig solved: {self._width}x{self._height} | '
            f'baseline {self._baseline_m * 100:.2f} cm | '
            f'hfov {hfov:.2f} deg -> fx {self._fx:.2f} px')

    def _solve_center_transform(self) -> None:
        """
        Rigid transform and intrinsics for reprojecting the left-frame depth
        into the centre camera.

        Unreal world axes are X=forward, Y=right, Z=up; a camera optical frame
        is x=right, y=down, z=forward, so a camera's optical basis in world
        coordinates is M = [right, -up, forward]. For a point P_l in left-optical
        coordinates:

            P_c = R (P_l - t),   R = M_c^T M_l,   t = M_l^T (loc_c - loc_l)

        R is the identity when both cameras share a rotation; t is the centre
        camera's position in the left camera's optical axes.
        """
        probe = self._grab_rgb(self.center_camera_id)
        self._c_h, self._c_w = probe.shape[:2]
        hfov_c = float(self._request(f'vget /camera/{self.center_camera_id}/fov'))
        self._c_fx = (self._c_w / 2.0) / math.tan(math.radians(hfov_c) / 2.0)
        self._c_cx, self._c_cy = self._c_w / 2.0, self._c_h / 2.0

        loc_l, rot_l = self._geom[self.left_camera_id]
        loc_r, _     = self._geom[self.right_camera_id]
        loc_c, rot_c = self._geom[self.center_camera_id]

        fwd_l, right_l, up_l = unreal_axes(*rot_l)
        fwd_c, right_c, up_c = unreal_axes(*rot_c)
        M_l = np.column_stack([right_l, -up_l, fwd_l])   # left-optical  -> world
        M_c = np.column_stack([right_c, -up_c, fwd_c])   # centre-optical -> world

        self._c_R = M_c.T @ M_l
        self._c_t = M_l.T @ ((loc_c - loc_l) / self.uu_per_metre)

        self.get_logger().info(
            f'Centre camera {self.center_camera_id}: {self._c_w}x{self._c_h} '
            f'hfov {hfov_c:.2f} deg -> fx {self._c_fx:.2f} px')
        self.get_logger().info(
            f'  offset from left eye (optical x=right, y=down, z=forward): '
            f'[{self._c_t[0]*100:.2f}, {self._c_t[1]*100:.2f}, '
            f'{self._c_t[2]*100:.2f}] cm')

        # Frame-independent placement checks:
        #   along = fraction of the baseline (0 = left eye, 1 = right eye)
        #   perp  = distance from the line through the eyes
        # A camera off that line, or outside the eyes, is not the centre camera.
        eye_axis = loc_r - loc_l
        eye_len = float(np.linalg.norm(eye_axis))
        if eye_len > 1e-9:
            u = eye_axis / eye_len
            v = loc_c - loc_l
            perp = float(np.linalg.norm(v - np.dot(v, u) * u)) / self.uu_per_metre
            along = float(np.dot(v, u)) / eye_len
            self.get_logger().info(
                f'  centre sits {along * 100:.1f}% of the way along the baseline, '
                f'{perp * 100:.2f} cm off the line through the eyes')

            baseline = eye_len / self.uu_per_metre
            # Far off the rig: the warp would still be correct, but the result
            # is not a centre view. Refuse rather than mislead.
            if perp > 0.5 * baseline or not (-0.5 <= along <= 1.5):
                raise RuntimeError(
                    f'Camera {self.center_camera_id} is not the centre of this '
                    f'rig: it sits {along * 100:.0f}% along the baseline and '
                    f'{perp * 100:.1f} cm off the line through cameras '
                    f'{self.left_camera_id} and {self.right_camera_id} '
                    f'(baseline {baseline * 100:.1f} cm). Check center_camera_id '
                    f'— run utils/list_unreal_cameras.py to see every pose.')
            # Slightly off the line is legitimate: the warp stays exact, but
            # disocclusion holes grow with the offset.
            if perp > 0.05 * baseline:
                self.get_logger().warn(
                    f'Centre camera is {perp * 100:.1f} cm off the eye-to-eye '
                    f'line; the warp is exact but disocclusion holes grow with '
                    f'that offset.')

        if np.abs(self._c_R - np.eye(3)).max() > 1e-6:
            self.get_logger().warn(
                'Centre camera is rotated relative to the left eye; the '
                'reprojection handles it, but a co-rotated rig warps cleaner.')
        if abs(self._c_t[2]) > 0.01:
            self.get_logger().warn(
                f'Centre camera is {self._c_t[2]*100:.1f} cm forward/behind the '
                f'left eye. The warp is still correct, but a centre camera on '
                f'the stereo baseline produces far fewer occlusion holes.')

    def _audit_near_field(self) -> None:
        """
        Report how much of the scene sits closer than the rig can resolve.

        A surface nearer than fx*B/num_disparities has a disparity beyond the
        search window. SGBM then locks onto whatever correlates best inside the
        range, producing the speckled band along the bottom edge of the depth
        image: plausible values with no relation to the scene.

        Widening num_disparities is the only fix. This measures how wide it has
        to be, from the central camera's ground-truth buffer. Diagnostic only;
        the ground truth never touches the sensor path.
        """
        try:
            gt = self._center_depth_gt(self._grab_depth(self.center_camera_id))
        except Exception as exc:
            self.get_logger().warn(f'Could not audit the near field ({exc}).')
            return

        valid = np.isfinite(gt)
        if not valid.any():
            return
        too_near = valid & (gt < self._near_limit_m)
        frac = too_near.mean() * 100

        if not too_near.any():
            self.get_logger().info(
                f'Near-field audit: nothing in view is closer than the '
                f'{self._near_limit_m:.2f} m near limit.')
            return

        # 1st percentile, not the minimum: one stray pixel must not size the rig.
        closest = float(np.percentile(gt[too_near], 1))
        fxb = self._fx * self._baseline_m
        needed = int(math.ceil((fxb / closest) / 16.0)) * 16

        msg = (f'Near-field audit: {frac:.0f}% of the view is closer than the '
               f'{self._near_limit_m:.2f} m near limit (down to {closest:.2f} m). '
               f'Depth there is aliasing, not geometry.')
        if needed <= 256:
            msg += (f' num_disparities: {needed} would resolve it, at a '
                    f'{needed / self._width * 100:.0f}% left border '
                    f'(currently {self._num_disparities}).')
        else:
            msg += (f' Resolving it needs num_disparities {needed}, beyond the '
                    f'256 cap: widen the baseline or accept the band.')
        self.get_logger().warn(msg)

    def _build_matcher(self) -> None:
        """
        Configure SGBM and derive the near limit from the disparity search.

        num_disparities is the width of the search window, and therefore of the
        unmatched left border. The nearest resolvable surface follows from it:

            near = fx * B / num_disparities

        Depth resolution follows dZ = Z^2 / (fx * B): a short baseline and a
        wide FOV leave the far field coarsely quantised.
        """
        fxb = self._fx * self._baseline_m

        nd = self.num_disparities_param
        if nd % 16 != 0 or nd <= 0:
            nd = max(16, min(int(math.ceil(nd / 16.0)) * 16, 256))
            self.get_logger().warn(
                f'num_disparities must be a positive multiple of 16; using {nd}.')
        self._num_disparities = min(nd, 256)
        self._near_limit_m = fxb / self._num_disparities

        block = self.block_size | 1   # SGBM requires an odd block size
        self._matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self._num_disparities,
            blockSize=block,
            P1=8 * block * block,
            P2=32 * block * block,
            disp12MaxDiff=1,
            uniquenessRatio=self.uniqueness_ratio,
            speckleWindowSize=self.speckle_window_size,
            speckleRange=self.speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        far = fxb / (1.0 / 16.0)   # one sub-pixel step of disparity
        dz_at_max = self.max_depth_m ** 2 / fxb
        self.get_logger().info(
            f'SGBM: numDisparities {self._num_disparities}, blockSize {block} | '
            f'near limit fx*B/nd = {self._near_limit_m:.2f} m .. {far:.0f} m')
        if self.fill_left_border:
            # A surface at Z is recoverable from column u >= fx*B/Z.
            recover_at = fxb / self.max_depth_m
            self.get_logger().info(
                f'Left border recovered per-pixel (d <= u): a surface at '
                f'{self.max_depth_m:.0f} m is matchable from column '
                f'{recover_at:.0f} onward, not column {self._num_disparities}')
        else:
            self.get_logger().warn(
                f'fill_left_border is off: the leftmost {self._num_disparities} '
                f'columns ({self._num_disparities / self._width * 100:.0f}% of the '
                f'image) will be blank regardless of scene depth')
        self.get_logger().info(
            f'Depth resolution dZ = Z^2/(fx*B), fx*B = {fxb:.2f} m*px -> '
            f'{dz_at_max * 100:.1f} cm per disparity pixel at {self.max_depth_m:.1f} m')
        if dz_at_max > 0.1 * self.max_depth_m:
            self.get_logger().warn(
                f'At max_depth_m={self.max_depth_m:.1f} m one pixel of matching '
                f'error is a {dz_at_max / self.max_depth_m * 100:.0f}% depth error. '
                f'Widen the baseline or narrow the FOV before trusting far-field depth.')

        if self.equalize_for_glare:
            self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    # =========================================================================
    # Capture
    # =========================================================================

    def _grab_rgb(self, cam_id: int, viewmode: str = 'lit') -> np.ndarray:
        """PNG payload -> contiguous uint8 RGB, alpha dropped."""
        cmd = f'vget /camera/{cam_id}/{viewmode} png'
        img = read_png(self._request_raw(cmd))
        if img is None:
            raise RuntimeError(
                f'read_png could not decode the reply to "{cmd}". The payload '
                f'must be passed through as raw bytes, not decoded text.')
        img = img.astype(np.uint8)
        if img.shape[2] == 4:
            img = img[:, :, :3]
        return np.ascontiguousarray(img)

    def _grab_depth(self, cam_id: int) -> np.ndarray:
        """NPY payload -> float depth buffer, still in Unreal's radial units."""
        cmd = f'vget /camera/{cam_id}/depth npy'
        depth = read_npy(self._request_raw(cmd))
        if depth is None:
            raise RuntimeError(f'read_npy could not decode the reply to "{cmd}".')
        return depth

    def _capture_all(self):
        """
        Grab all three navcams as one instant, pausing the game so the TCP
        round-trips describe the same world state.
        """
        paused = False
        if self.pause_game_during_capture:
            paused = self.unreal_client.request('vset /action/game/pause') == 'ok'
            if not paused:
                self.get_logger().warn(
                    'Could not pause the game; the stereo pair will be captured '
                    'at slightly different times and the baseline will not hold.',
                    throttle_duration_sec=10.0)
        try:
            left = self._grab_rgb(self.left_camera_id)
            right = self._grab_rgb(self.right_camera_id)
            center = self._grab_rgb(self.center_camera_id)

            center_depth_gt = None
            if self.validate_against_ground_truth:
                center_depth_gt = self._grab_depth(self.center_camera_id)

            center_mask = None
            if self.enable_object_mask:
                center_mask = self._grab_rgb(self.center_camera_id,
                                             viewmode='object_mask')
        finally:
            if paused:
                self.unreal_client.request('vset /action/game/resume')

        if self._swap_lr:
            left, right = right, left
        return left, right, center, center_depth_gt, center_mask

    # =========================================================================
    # Stereo matching
    # =========================================================================

    def _compute_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """SGBM disparity in the left frame, sub-pixel, NaN where unmatched."""
        gray_l = cv2.cvtColor(left, cv2.COLOR_RGB2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_RGB2GRAY)

        # CLAHE pulls texture back out of near-saturated pixels. It cannot
        # recover a fully clipped highlight, where the texture is simply gone.
        if self._clahe is not None:
            gray_l = self._clahe.apply(gray_l)
            gray_r = self._clahe.apply(gray_r)

        if not self.fill_left_border:
            disp = self._matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
            disp[disp <= 0.0] = np.nan
            return disp

        # OpenCV blanks every column u < numDisparities, where the widest search
        # window does not fit. The true constraint is per-pixel: a left pixel at
        # column u with disparity d matches right pixel u - d, so it is
        # recoverable whenever d <= u. Distant surfaces have small d and match
        # well inside that border.
        #
        # Pad both images by numDisparities so the window always fits, crop back,
        # then enforce u - d >= 0. Matches reaching into the padding violate it
        # and are dropped: those points lie outside the right camera's FOV.
        d_max = self._num_disparities
        pad_l = cv2.copyMakeBorder(gray_l, 0, 0, d_max, 0, cv2.BORDER_REPLICATE)
        pad_r = cv2.copyMakeBorder(gray_r, 0, 0, d_max, 0, cv2.BORDER_REPLICATE)

        disp = self._matcher.compute(pad_l, pad_r).astype(np.float32) / 16.0
        disp = disp[:, d_max:]                       # back to the left frame

        u = np.arange(disp.shape[1], dtype=np.float32)[None, :]
        disp[u - disp < 0.0] = -1.0                  # outside the right FOV
        disp[disp <= 0.0] = np.nan
        return np.ascontiguousarray(disp)

    def _depth_from_disparity(self, disp: np.ndarray) -> np.ndarray:
        """Z = fx * B / d, clamped to the configured depth window."""
        with np.errstate(divide='ignore', invalid='ignore'):
            depth = (self._fx * self._baseline_m) / disp
        depth[~np.isfinite(depth)] = np.nan
        depth[(depth < self._near_limit_m) | (depth > self.max_depth_m)] = np.nan
        return depth.astype(np.float32)

    def _reproject_to_center(self, depth_left: np.ndarray) -> Optional[np.ndarray]:
        """
        Forward-warp the left-frame planar depth into the centre camera.

        Each valid left pixel is back-projected to 3D, moved into the centre
        camera's optical frame and projected through its intrinsics. Several may
        land on one centre pixel, so a z-buffer keeps the nearest; otherwise
        background bleeds through foreground at depth discontinuities.

        Two passes. Pass 1 rounds each point to its nearest centre pixel. The
        shift fx*t/Z varies with depth, so adjacent source pixels can land two
        columns apart, leaving single-pixel gaps. Pass 2 splats each point over
        its 2x2 footprint and is consulted only where pass 1 left a hole, so a
        foreground surface grows by at most one pixel and only into a gap.

        Occlusion holes, wider than one pixel, survive as NaN.
        """
        valid = np.isfinite(depth_left)
        if not valid.any():
            return None

        vv, uu = np.nonzero(valid)
        Z = depth_left[valid].astype(np.float64)
        X = (uu - self._cx) * Z / self._fx
        Y = (vv - self._cy) * Z / self._fx        # fy == fx for a square pixel

        P_c = self._c_R @ (np.stack([X, Y, Z]) - self._c_t[:, None])   # (3, N)
        Zc = P_c[2]

        front = Zc > self._near_limit_m           # behind the centre camera: drop
        if not front.any():
            return None
        Zc = Zc[front]
        uf = self._c_fx * P_c[0][front] / Zc + self._c_cx     # sub-pixel
        vf = self._c_fx * P_c[1][front] / Zc + self._c_cy

        w, h = self._c_w, self._c_h

        def zbuffer(u_i: np.ndarray, v_i: np.ndarray, z: np.ndarray) -> np.ndarray:
            inb = (u_i >= 0) & (u_i < w) & (v_i >= 0) & (v_i < h)
            buf = np.full(h * w, np.inf, dtype=np.float64)
            if inb.any():
                np.minimum.at(buf, v_i[inb] * w + u_i[inb], z[inb])
            return buf

        # Pass 1: nearest pixel, no edge growth.
        primary = zbuffer(np.rint(uf).astype(np.int32),
                          np.rint(vf).astype(np.int32), Zc)

        # Pass 2: 2x2 splat, consulted only where pass 1 left a hole.
        holes = np.isinf(primary)
        if holes.any():
            u0 = np.floor(uf).astype(np.int32)
            v0 = np.floor(vf).astype(np.int32)
            splat = np.full(h * w, np.inf, dtype=np.float64)
            for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):
                np.minimum(splat, zbuffer(u0 + du, v0 + dv, Zc), out=splat)
            primary[holes] = splat[holes]

        primary[np.isinf(primary)] = np.nan
        return primary.reshape(h, w).astype(np.float32)

    def _saturation_mask(self, rgb: np.ndarray) -> np.ndarray:
        """Pixels where any channel has clipped — the blown-out core of a glare."""
        return rgb.max(axis=2) >= self.saturation_level

    def _center_depth_gt(self, depth_gt_raw: np.ndarray) -> np.ndarray:
        """
        Central camera's Unreal depth buffer -> planar Z metres, sky as NaN.

        The buffer is radial. Skipping the conversion produces a smooth corner
        error that resembles lens distortion (60% at 90 deg HFOV).
        """
        raw = depth_gt_raw.astype(np.float64)
        gt = radial_to_planar(raw, self._c_fx, self._c_fx, self._c_cx, self._c_cy)
        gt = gt / self.uu_per_metre
        gt[raw >= _FLOAT16_MAX - 4.0] = np.nan     # sky / background
        return gt.astype(np.float32)

    def _log_glare_stats(self, depth_center: np.ndarray, gt: np.ndarray,
                         sat: np.ndarray) -> None:
        """
        Score the stereo depth against ground truth, inside vs outside the glare.

        All three arguments live in the central camera's frame: depth_center is
        the reprojected stereo result, gt that camera's own depth buffer, sat the
        saturation mask of its image. Differencing frames would compare different
        3D points per pixel.
        """
        if gt.shape != depth_center.shape:
            self.get_logger().warn(
                f'Central camera depth {gt.shape} differs from the reprojected '
                f'stereo depth {depth_center.shape}; skipping validation.',
                throttle_duration_sec=10.0)
            return
        depth = depth_center

        # A blown highlight destroys matching across the whole correlation
        # window, not just the clipped pixels, so grow the zone by the block.
        k = max(3, self.block_size * 2 + 1)
        sat_zone = cv2.dilate(sat.astype(np.uint8),
                              np.ones((k, k), np.uint8)).astype(bool)

        gt_ok = np.isfinite(gt) & (gt > self._near_limit_m) & (gt < self.max_depth_m)
        matched = np.isfinite(depth)

        parts = [f'saturated {sat.mean() * 100:.2f}%']
        for name, zone in (('glare', sat_zone & gt_ok), ('clean', ~sat_zone & gt_ok)):
            if not zone.any():
                continue
            holes = (~matched[zone]).mean() * 100
            both = zone & matched
            if both.any():
                rel = np.abs(depth[both] - gt[both]) / gt[both]
                parts.append(f'{name}: holes {holes:.1f}% '
                             f'med|dZ|/Z {np.median(rel) * 100:.2f}% '
                             f'p95 {np.percentile(rel, 95) * 100:.2f}%')
            else:
                parts.append(f'{name}: holes {holes:.1f}% (no matches)')
        self.get_logger().info(' | '.join(parts))

    # =========================================================================
    # Message construction
    # =========================================================================

    def _camera_info(self, stamp, frame_id: str, is_right: bool,
                     intrinsics: Optional[Tuple[float, float, float, int, int]] = None
                     ) -> CameraInfo:
        """
        Pinhole CameraInfo for an already-rectified pair.

        The right camera carries the baseline in P[3] = -fx * B, which is how the
        standard ROS stereo pipeline recovers depth from disparity. Pass
        `intrinsics` as (fx, cx, cy, width, height) for the centre camera, whose
        FOV may differ from the pair's.
        """
        if intrinsics is None:
            fx, cx, cy = self._fx, self._cx, self._cy
            width, height = self._width, self._height
        else:
            fx, cx, cy, width, height = intrinsics
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx,
                  0.0, fx, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        tx = -fx * self._baseline_m if is_right else 0.0
        info.p = [fx, 0.0, cx, tx,
                  0.0, fx, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def _disparity_msg(self, disp: np.ndarray, stamp, frame_id: str,
                       fx: Optional[float] = None,
                       width: Optional[int] = None,
                       height: Optional[int] = None,
                       valid_window: bool = True) -> DisparityImage:
        fx = self._fx if fx is None else fx
        width = self._width if width is None else width
        height = self._height if height is None else height

        msg = DisparityImage()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        # Unmatched pixels go out as 0.0: consumers treat anything
        # <= min_disparity as invalid, and NaN is not portable here.
        msg.image = self.cv_bridge.cv2_to_imgmsg(
            np.nan_to_num(disp, nan=0.0), encoding='32FC1')
        msg.image.header.stamp = stamp
        msg.image.header.frame_id = frame_id
        msg.f = float(fx)
        msg.t = float(self._baseline_m)
        msg.min_disparity = 0.0
        msg.max_disparity = float(self._num_disparities)
        msg.delta_d = 1.0 / 16.0    # SGBM's sub-pixel quantum
        if valid_window and not self.fill_left_border:
            # The leftmost numDisparities columns have no search window in the
            # right image and can never be matched.
            msg.valid_window = RegionOfInterest(
                x_offset=self._num_disparities, y_offset=0,
                width=max(0, width - self._num_disparities),
                height=height, do_rectify=False)
        else:
            # With the border recovered, and after reprojection, validity is
            # per-pixel (d <= u), not a column range. Test the value, not this
            # rectangle.
            msg.valid_window = RegionOfInterest(
                x_offset=0, y_offset=0, width=width, height=height,
                do_rectify=False)
        return msg

    def _publish_image(self, key: str, img: np.ndarray, encoding: str,
                       stamp, frame_id: str) -> None:
        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        self._pubs[key].publish(msg)

    def _publish_color(self, key: str, rgb: np.ndarray, stamp, frame_id: str) -> None:
        """Raw rgb8 plus a JPEG-compressed copy, on <key> and <key>_compressed."""
        self._publish_image(key, rgb, 'rgb8', stamp, frame_id)

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        msg = self.cv_bridge.cv2_to_compressed_imgmsg(bgr, dst_format='jpg')
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        self._pubs[key + '_compressed'].publish(msg)

    def _colourise(self, values: np.ndarray, vmax: float) -> np.ndarray:
        """Scalar field -> JET bgr8, invalid pixels black."""
        norm = np.nan_to_num(values, nan=0.0) / max(vmax, 1e-6)
        vis = cv2.applyColorMap((np.clip(norm, 0.0, 1.0) * 255).astype(np.uint8),
                                cv2.COLORMAP_JET)
        vis[~np.isfinite(values)] = 0
        return np.ascontiguousarray(vis)

    # =========================================================================
    # Timer callback
    # =========================================================================

    def read_stereo_pair(self) -> None:
        if not self._is_active:
            return
        if not self.unreal_client or not self.unreal_client.isconnected():
            self.get_logger().warn('UnrealCV connection lost. Bypassing read cycle.',
                                   throttle_duration_sec=5.0)
            return

        try:
            stamp = self.get_clock().now().to_msg()
            f_left, f_right, f_center = (self.left_frame, self.right_frame,
                                         self.center_frame)

            left, right, center, gt_raw, center_mask = self._capture_all()
            disp = self._compute_disparity(left, right)
            depth_left = self._depth_from_disparity(disp)

            # The eyes carry colour only: depth is a product of the pair.
            self._publish_color('left_color', left, stamp, f_left)
            self._publish_color('right_color', right, stamp, f_right)
            self._pubs['left_info'].publish(self._camera_info(stamp, f_left, False))
            self._pubs['right_info'].publish(self._camera_info(stamp, f_right, True))

            # The central camera is the RGB-D sensor this rig simulates.
            self._publish_color('center_color', center, stamp, f_center)
            c_intr = (self._c_fx, self._c_cx, self._c_cy, self._c_w, self._c_h)
            self._pubs['center_info'].publish(
                self._camera_info(stamp, f_center, False, intrinsics=c_intr))

            # Warped onto the central pixel grid, so depth, disparity and RGB
            # all overlay one another.
            depth_c = self._reproject_to_center(depth_left)
            if depth_c is not None:
                depth_pub = np.clip(depth_c, 0.0, self.depth_max_distance)
                depth_pub[~np.isfinite(depth_c)] = np.nan
                self._publish_image('center_depth', depth_pub, '32FC1', stamp, f_center)
                self._publish_image('center_depth_vis',
                                    self._colourise(depth_pub, self.depth_max_distance),
                                    'bgr8', stamp, f_center)

                # Disparity a virtual pair would see from the centre, using the
                # central focal length: d = fx_c * B / Z.
                with np.errstate(divide='ignore', invalid='ignore'):
                    disp_c = (self._c_fx * self._baseline_m) / depth_c
                disp_c[~np.isfinite(depth_c)] = np.nan
                self._pubs['center_disparity'].publish(
                    self._disparity_msg(disp_c, stamp, f_center, fx=self._c_fx,
                                        width=self._c_w, height=self._c_h,
                                        valid_window=False))

            if center_mask is not None:
                self._publish_image('center_object_mask', center_mask, 'rgb8',
                                    stamp, f_center)
                msg_map = String()
                msg_map.data = self._cached_color_map_json
                self._pubs['center_object_mask_mapping'].publish(msg_map)

            if gt_raw is not None:
                gt = self._center_depth_gt(gt_raw)
                self._publish_image('center_depth_gt', gt, '32FC1', stamp, f_center)
                if depth_c is not None:
                    # Saturation is measured on the central image so it aligns
                    # with the depth it explains. Clipping in the eyes destroys
                    # matching; the damage surfaces here as holes.
                    self._log_glare_stats(depth_c, gt,
                                          self._saturation_mask(center))

            if self.debug_mode:
                matched = np.isfinite(disp).mean() * 100
                filled = (np.isfinite(depth_c).mean() * 100
                          if depth_c is not None else 0.0)
                self.get_logger().info(
                    f'matched {matched:.1f}% of the left frame | '
                    f'{filled:.1f}% of the central frame received depth')

        except Exception as exc:
            self.get_logger().error(f'Stereo read/publish cycle failed: {exc}')

    # =========================================================================
    # Lifecycle transitions
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        rclpy reports a raising on_configure as a bare 'Failed to make
        transition', discarding the exception. Catch it so the reason reaches the
        log, and release the UnrealCV socket: otherwise a failed configure holds
        the server's only client slot and locks out every retry.
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
        self.left_topic       = self.get_parameter('left_topic').value
        self.right_topic      = self.get_parameter('right_topic').value
        self.center_topic     = self.get_parameter('center_topic').value
        self.pause_game_during_capture = self.get_parameter('pause_game_during_capture').value
        self.baseline_m_param = self.get_parameter('baseline_m').value
        self.hfov_deg_param   = self.get_parameter('hfov_deg').value
        self.uu_per_metre     = self.get_parameter('uu_per_metre').value
        self.num_disparities_param = self.get_parameter('num_disparities').value
        self.max_depth_m      = self.get_parameter('max_depth_m').value
        self.block_size       = self.get_parameter('block_size').value
        self.uniqueness_ratio = self.get_parameter('uniqueness_ratio').value
        self.speckle_window_size = self.get_parameter('speckle_window_size').value
        self.speckle_range    = self.get_parameter('speckle_range').value
        self.equalize_for_glare = self.get_parameter('equalize_for_glare').value
        self.fill_left_border = self.get_parameter('fill_left_border').value
        self.saturation_level = self.get_parameter('saturation_level').value
        self.validate_against_ground_truth = self.get_parameter('validate_against_ground_truth').value
        self.debug_mode       = self.get_parameter('debug_mode').value
        self.depth_max_distance = self.get_parameter('depth_max_distance').value
        self.enable_object_mask = self.get_parameter('enable_object_mask').value
        hz                    = self.get_parameter('image_reading_hz_rate').value

        self._connect_unrealcv(self.get_parameter('ip').value,
                               self.get_parameter('port').value)
        self._check_cameras_exist()
        # All pose queries in one pause window: the rig must be measured at a
        # single instant, or the rover's motion is misread as rig geometry.
        self._snapshot_geometry()
        self._solve_rig()
        self._build_matcher()
        self._solve_center_transform()
        self._audit_near_field()

        L, R, C = self.left_topic, self.right_topic, self.center_topic

        # Topic namespaces are hierarchical; TF frame names cannot contain '/'.
        self.left_frame = _frame_id(L)
        self.right_frame = _frame_id(R)
        self.center_frame = _frame_id(C)

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

        if self.validate_against_ground_truth:
            pub('center_depth_gt', RosImage, f'{C}/depth_gt')
        if self.enable_object_mask:
            pub('center_object_mask', RosImage, f'{C}/object_mask')
            pub('center_object_mask_mapping', String, f'{C}/object_mask_mapping')
            try:
                self._cached_color_map_json = self._build_color_mapping()
            except Exception as exc:
                # The mask images are usable without a name->colour legend.
                self.get_logger().error(
                    f'Object colour mapping failed ({exc}); publishing an empty '
                    f'map. The object_mask images are unaffected.')
                self._cached_color_map_json = "{}"

        self.get_logger().info(
            f'Publishing {L}/*, {R}/* (colour) and {C}/* '
            f'(colour + PLANAR depth + disparity'
            f'{" + object mask" if self.enable_object_mask else ""}). '
            f'Frames: {self.left_frame}, {self.right_frame}, {self.center_frame}.')

        self._timer = self.create_timer(1.0 / hz, self.read_stereo_pair)
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

        # getattr: _release also runs from a failed on_configure, which may not
        # have created any publisher.
        for pub in getattr(self, '_pubs', {}).values():
            self.destroy_publisher(pub)
        if hasattr(self, '_pubs'):
            self._pubs.clear()

        if self.unreal_client:
            # Leaving the game paused would freeze the simulation for every other
            # component.
            if getattr(self, 'pause_game_during_capture', False):
                try:
                    self.unreal_client.request('vset /action/game/resume')
                except Exception:
                    pass
            self.unreal_client.disconnect()
            self.unreal_client = None

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
