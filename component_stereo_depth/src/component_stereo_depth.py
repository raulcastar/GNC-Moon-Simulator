"""
ROS 2 Lifecycle Node for the simulated stereo navcam rig (UnrealStereoCam).

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

navcam/central/image_depth is the SGBM result reprojected onto the central
camera's pixel grid: planar Z in metres, NaN where no surface was recovered.
navcam/central/depth_ground_truth is Unreal's own depth for the same view (the
plugin's depth capture on the central camera): planar Z in metres, exact, +Inf
where there is no surface (sky).
Disparity has no separate visualisation: it is a monotonic function of depth, so
it would just render image_depth/visualization inverted.

Frames arrive over TCP from the UnrealStereoCam Unreal plugin
(unreal/UnrealStereoCam/): each SceneCapture renders with the game frame and is
pushed here, so there is no render-on-demand and no game-thread stall. Every
capture enqueued in one Unreal tick carries the same frame id, which is what
pairs the left/right eyes as one rigid stereo view.

Rig geometry (baseline, focal length) is declared from parameters — stream mode
has nothing to measure it against — and the image size is taken from the first
frames.
"""

from typing import Dict, Optional, Tuple
import collections
import math
import os
import socket
import struct
import threading
import time
import json
import traceback

import numpy as np
import cv2

import rclpy
from ament_index_python.packages import get_package_share_directory
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer

from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import Image as RosImage, CompressedImage, CameraInfo, RegionOfInterest
from std_msgs.msg import String
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge


def _ros_time(ns: int) -> RosTime:
    """Simulation time in ns (UnrealSimClock) -> builtin_interfaces/Time."""
    return RosTime(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def _frame_id(topic: str) -> str:
    """
    Topic namespace -> TF frame name: 'navcam/central' -> 'navcam_central'.

    tf2 reserves '/' in frame names for the deprecated tf_prefix syntax.
    """
    return topic.strip('/').replace('/', '_')


class CycleTimer:
    """
    Time spent per processing cycle, by stage, summarised in one log line every
    `period` seconds (wall clock): cycles per second and the mean milliseconds
    of each stage. `start()` opens a cycle, `mark(stage)` closes the stage
    running since the previous mark, `end()` closes the cycle.
    """

    def __init__(self, logger, label: str, period: float = 30.0):
        self._log, self._label, self._period = logger, label, period
        self._sums = collections.OrderedDict()
        self._cycles, self._total, self._worst = 0, 0.0, 0.0
        self._since = time.monotonic()
        self._t0 = self._last = None

    def start(self) -> None:
        self._t0 = self._last = time.perf_counter()

    def mark(self, stage: str) -> None:
        if self._last is None:
            return
        now = time.perf_counter()
        self._sums[stage] = self._sums.get(stage, 0.0) + (now - self._last)
        self._last = now

    def end(self) -> None:
        if self._t0 is None:
            return
        spent = time.perf_counter() - self._t0
        self._t0 = self._last = None
        self._cycles += 1
        self._total += spent
        self._worst = max(self._worst, spent)
        elapsed = time.monotonic() - self._since
        if elapsed >= self._period:
            n = self._cycles
            stages = ', '.join(f'{k} {1000 * v / n:.0f}' for k, v in self._sums.items())
            self._log.info(f'{self._label}: {n / elapsed:.2f} cycles/s, {1000 * self._total / n:.0f} ms per '
                           f'cycle (worst {1000 * self._worst:.0f})' + (f' | ms: {stages}' if stages else ''))
            self._sums.clear()
            self._cycles, self._total, self._worst = 0, 0.0, 0.0
            self._since = time.monotonic()


class FrameStreamReceiver:
    """
    TCP server for the UnrealStereoCam plugin (unreal/UnrealStereoCam/).

    Unreal renders the navcams with each frame and pushes them here over one
    connection, so nothing is rendered on demand and the game thread never
    stalls. Packets are 24-byte-header BGRA frames tagged with
    Unreal's frame counter; captures enqueued in one tick share a tag, which
    is what lets left/right be paired as a rigid stereo view.

    Runs a background thread; survives Play/Stop cycles by re-accepting.
    Several UnrealStereoCam components may connect at once (e.g. the navcam
    rig and a separate rear camera), each on its own connection and reader
    thread; their frames land in the same per-cam-id rings.
    """

    _HEADER = struct.Struct('<4sBBBBHHQI')
    _MAGIC = b'NAVF'
    _RING = 6                      # frames kept per camera for pairing

    def __init__(self, bind_ip: str, port: int, logger):
        self._addr = (bind_ip, port)
        self._log = logger
        self._frames: Dict[int, collections.deque] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._server: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self.connections = 0                    # Unreal streamers connected now
        self.known_ids: set = set()             # camera ids the node uses (set after configure)
        self._unknown_ids: set = set()          # ids already warned about

    def start(self) -> None:
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(self._addr)
        self._server.listen(4)
        self._server.settimeout(0.5)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._log.info(f'Frame stream: listening on {self._addr[0]}:{self._addr[1]} '
                       f'for the UnrealStereoCam plugin.')

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._server:
            self._server.close()
            self._server = None

    def latest_pair(self, left_id: int, right_id: int, center_id: int):
        """
        Newest frame id present for BOTH eyes, with the nearest centre frame.

        Returns (left, right, center, frame_id, age_s) or None. The eyes must
        match exactly — that is the rigid-pair guarantee. The centre tolerates
        a small offset: it is not matched against anything, only overlaid.
        """
        with self._lock:
            eyes_l = {e[0]: e for e in self._frames.get(left_id, ())}
            eyes_r = {e[0]: e[1] for e in self._frames.get(right_id, ())}
            common = sorted(set(eyes_l) & set(eyes_r))
            if not common:
                return None
            fid = common[-1]
            centers = list(self._frames.get(center_id, ()))
            if not centers:
                return None
            c_fid, center, _, _ = min(centers, key=lambda e: abs(e[0] - fid))
            if abs(c_fid - fid) > 60:        # a second of frames apart: stale
                return None
            _, left, t, stamp_ns = eyes_l[fid]
            age = time.monotonic() - t
            return left, eyes_r[fid], center, fid, age, stamp_ns

    def frame_near(self, cam_id: int, fid: int, max_gap: int = 60):
        """
        The frame for cam_id closest to frame id `fid`, or None.

        Used for the object-mask capture, which is streamed on its own cam_id
        but must overlay the central image: take the mask rendered nearest the
        same world tick. `max_gap` matches the centre-pairing tolerance in
        latest_pair — a second of frames apart is treated as stale.
        """
        with self._lock:
            frames = list(self._frames.get(cam_id, ()))
        if not frames:
            return None
        f_fid, img, _, _ = min(frames, key=lambda e: abs(e[0] - fid))
        if abs(f_fid - fid) > max_gap:
            return None
        return img

    def latest(self, cam_id: int):
        """(frame_id, image, stamp_ns) of the newest frame for cam_id, or None."""
        with self._lock:
            frames = self._frames.get(cam_id)
            if not frames:
                return None
            fid, img, _, stamp_ns = frames[-1]
            return fid, img, stamp_ns

    # ------------------------------------------------------------------ #

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self._serve, args=(conn, addr), daemon=True).start()

    def _serve(self, conn: socket.socket, addr) -> None:
        self._log.info(f'Frame stream: Unreal connected from {addr[0]}:{addr[1]}.')
        with self._lock:
            self.connections += 1
        try:
            self._read_stream(conn)
        finally:
            conn.close()
            with self._lock:
                self.connections -= 1
            if not self._stop.is_set():
                self._log.warn(f'Frame stream: Unreal disconnected ({addr[0]}:{addr[1]}); '
                               'waiting for the next Play.')

    def _read_stream(self, conn: socket.socket) -> None:
        conn.settimeout(0.5)
        # A bytearray grows in place: with bytes, every read copied the whole
        # partial frame again (a 1.2 MB frame arrives in many reads).
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = conn.recv(1 << 20)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            self._drain(buf)

    def _drain(self, buf) -> bytearray:
        """
        Extract every complete frame from `buf`, consuming them in place (a
        bytearray; bytes are copied into one), and return what is left: the
        trailing fragment of the next frame.
        """
        if not isinstance(buf, bytearray):
            buf = bytearray(buf)
        while True:
            start = buf.find(self._MAGIC)
            if start < 0:
                del buf[:-3]                 # keep a possible partial magic
                return buf
            if start > 0:
                del buf[:start]              # resync past garbage
            if len(buf) < self._HEADER.size:
                return buf
            (_, version, cam_id, fmt, _, w, h, fid,
             size) = self._HEADER.unpack_from(buf)
            if version not in (1, 2) or fmt not in (1, 2) or size != w * h * 4 or w == 0:
                self._log.warn(f'Frame stream: bad header (v{version} fmt{fmt} '
                               f'{w}x{h} size {size}); resyncing.',
                               throttle_duration_sec=10.0)
                del buf[:4]                  # skip this magic, scan again
                continue
            # Version 2 adds the capture time (simulation clock, ns) after the
            # 24-byte header; version 1 frames get stamped on arrival (0 here).
            start = self._HEADER.size + (8 if version == 2 else 0)
            end = start + size
            if len(buf) < end:
                return buf                   # payload still in flight
            stamp_ns = struct.unpack_from('<Q', buf, self._HEADER.size)[0] if version == 2 else 0
            # bytes() copies the payload out, so no numpy view pins the
            # bytearray (it could not be resized while one exists).
            payload = bytes(buf[start:end])
            if fmt == 2:                     # R32F depth, centimetres
                image = np.frombuffer(payload, dtype='<f4').reshape(h, w).copy()
            else:                            # BGRA8 colour -> RGB (cv2: ~30x faster than indexing)
                image = cv2.cvtColor(np.frombuffer(payload, dtype=np.uint8).reshape(h, w, 4),
                                     cv2.COLOR_BGRA2RGB)
            if self.known_ids and cam_id not in self.known_ids and cam_id not in self._unknown_ids:
                self._unknown_ids.add(cam_id)
                self._log.warn(
                    f'Frame stream: camera id {cam_id} ({w}x{h}) is not one this node '
                    f'uses (configured: {sorted(self.known_ids)}); its frames are ignored. '
                    'Check the camera ids in the UnrealStereoCam components against '
                    'config/params.yaml.')
            with self._lock:
                ring = self._frames.setdefault(
                    cam_id, collections.deque(maxlen=self._RING))
                ring.append((fid, image, time.monotonic(), stamp_ns))
            del buf[:end]


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
        # Extra mono cameras (extra_cameras): (cam_id, key, frame, intrinsics or None)
        self._extras: list = []
        self._extra_last_fid: Dict[int, int] = {}
        self.cv_bridge = CvBridge()
        self._is_active = False

        # Frames are pushed by the UnrealStereoCam plugin over TCP.
        self._stream: Optional[FrameStreamReceiver] = None
        self._last_stream_fid: int = -1

        # Rig solution, filled in by _solve_stream_rig on the first frame.
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

        self.declare_parameters(
            namespace='',
            parameters=[
                # The UnrealStereoCam plugin (unreal/UnrealStereoCam/) pushes
                # frames to this TCP server: each SceneCapture renders with the
                # game frame, so there is no render-on-demand and no game stall.
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
                # Rig geometry, declared (stream mode cannot measure it): both
                # baseline_m and hfov_deg are REQUIRED.
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

    def _load_mask_classes(self) -> Tuple[str, str]:
        """
        The object-mask classes from mask_classes_file, as the two JSON strings
        published with the mask: the id -> name legend (object_mask_mapping,
        e.g. {"1": "sky", "5": "landscape"}) and the full class list with
        colours and groups (object_mask_classes).

        The mask itself only carries colours, so this file is the record of
        what each id in the Unreal scene means; Terrain Studio bakes the same
        file into the mask material. Malformed entries are skipped with a
        warning rather than aborting configuration.
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
        classes = []
        for entry in entries:
            cid, name, rgb = entry.get('id'), entry.get('name'), entry.get('rgb')
            if (not isinstance(cid, int) or not isinstance(name, str) or not name
                    or not isinstance(rgb, list) or len(rgb) != 3):
                self.get_logger().warn(f'{path}: ignoring malformed class {entry}.')
                continue
            classes.append({'id': cid, 'name': name, 'group': entry.get('group', 'other'),
                            'rgb': [int(v) for v in rgb]})
        classes.sort(key=lambda c: c['id'])
        legend = {str(c['id']): c['name'] for c in classes}
        if classes:
            self.get_logger().info(
                f'Object mask classes ({len(classes)}, {path}): '
                + ', '.join(f'{c["id"]}={c["name"]}' for c in classes))
        else:
            self.get_logger().warn(f'{path} lists no classes; object_mask_mapping will publish "{{}}".')
        return json.dumps(legend), json.dumps(classes)

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

    def _solve_stream_rig(self, eye_shape, center_shape) -> None:
        """
        Rig geometry from parameters and the first streamed frames.

        The stream carries no camera poses, so the geometry is declared:
        baseline_m and hfov_deg from parameters, image size from the frames,
        the eyes co-rotated (SGBM requires it anyway — the UnrealStereoCam
        captures must share a rotation), and the centre camera on the baseline
        at center_offset_m from the left eye (midpoint when negative).
        """
        self._height, self._width = eye_shape[:2]
        self._cx = self._width / 2.0
        self._cy = self._height / 2.0
        self._fx = (self._width / 2.0) / math.tan(
            math.radians(self.hfov_deg_param) / 2.0)

        self._c_h, self._c_w = center_shape[:2]
        self._c_fx = (self._c_w / 2.0) / math.tan(
            math.radians(self.hfov_deg_param) / 2.0)
        self._c_cx, self._c_cy = self._c_w / 2.0, self._c_h / 2.0

        offset = (self.center_offset_m if self.center_offset_m >= 0.0
                  else self._baseline_m / 2.0)
        self._c_R = np.eye(3)
        # Optical axes: x = right. The centre sits `offset` to the right of
        # the left eye, on the baseline.
        self._c_t = np.array([offset, 0.0, 0.0])

        self._build_matcher()
        self.get_logger().info(
            f'Rig declared (stream mode): {self._width}x{self._height} | '
            f'baseline {self._baseline_m * 100:.2f} cm | '
            f'hfov {self.hfov_deg_param:.2f} deg -> fx {self._fx:.2f} px | '
            f'centre {offset * 100:.2f} cm from the left eye')

    def _capture_from_stream(self):
        """
        Take the newest same-tick pair from the UnrealStereoCam plugin.

        Frames were rendered with the game frame and pushed here, with no
        game-thread stall. Returns None when there is nothing new to
        process; the cycle simply skips. Geometry is solved from parameters on
        the first frame, when the image size becomes known.
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
            self._solve_stream_rig(left.shape, center.shape)
        elif left.shape[:2] != (self._height, self._width):
            self.get_logger().error(
                f'Streamed frame is {left.shape[1]}x{left.shape[0]} but the '
                f'rig was solved at {self._width}x{self._height}. Restart the '
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

    # Shadow-mask colours.
    SHADOW_RGB = (0, 0, 0)          # Any surface the sun does not reach
    SUNLIT_RGB = (40, 120, 255)     # Ground in sunlight
    SKY_RGB = (0, 0, 0)             # Untagged (sky)
    OTHER_RGB = (128, 128, 128)     # Rocks, rover in sunlight
    ROVER_SHADOW_RGB = (255, 140, 0)  # The rover's own cast shadow

    # Ground-truth depth beyond this (metres) has no surface: the sky (+Inf).
    GROUND_TRUTH_FAR_M = 1.0e5

    # Shadow class ids (shadow_label, mono8), for training.
    SHADOW_IDS = {0: 'sky', 1: 'sunlit_ground', 2: 'shadowed_ground',
                  3: 'sunlit_object', 4: 'shadowed_object', 5: 'rover_shadow'}

    def _split_shadow_mask(self, frame: np.ndarray
                           ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Splits a mask frame from Terrain Studio's mask material into the shadow
        feed (colours), the shadow label image (SHADOW_IDS) and the object
        mask. The lowest bit of green flags ground, the lowest bit of blue any
        surface without sunlight (ground, rocks, rover), the lowest bit of red
        the rover's own shadow (with blue) or the sky (without blue: sky, sun's
        disc, glare); all
        are cleared from the object mask. The sun stays sky in the shadow feeds.
        """
        # One 4-bit code per pixel (ground | shadow | red flag | sky), then
        # two table lookups: ~3x faster than masked writes per class.
        objects = frame & np.uint8(0xFE)
        r, g, b = frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
        sky = (objects[:, :, 0] | objects[:, :, 1] | objects[:, :, 2]) == 0
        code = (g & 1) | ((b & 1) << 1) | ((r & 1) << 2) | (sky.view(np.uint8) << 3)
        return (np.take(self._SHADOW_PALETTE, code, axis=0),
                np.take(self._SHADOW_LABELS, code), objects)

    @classmethod
    def _shadow_tables(cls) -> Tuple[np.ndarray, np.ndarray]:
        """Colour and SHADOW_IDS label for each code of _split_shadow_mask."""
        palette = np.empty((16, 3), np.uint8)
        labels = np.empty(16, np.uint8)
        for code in range(16):
            ground, shadow, red, sky = (code >> bit & 1 for bit in range(4))
            rover, sun = red & shadow, red & (1 - shadow)
            sky = sky | sun                 # Red without blue: the sky (sun included)
            colour = cls.SKY_RGB if sky else cls.OTHER_RGB
            colour = cls.SUNLIT_RGB if ground else colour
            colour = cls.SHADOW_RGB if shadow else colour
            palette[code] = cls.ROVER_SHADOW_RGB if rover else colour
            label = 0 if sky else (1 if ground else 3) + shadow
            labels[code] = 5 if rover else label
        return palette, labels

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
                       stamp, frame_id: str) -> None:
        if not self._wanted(key):
            return
        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        self._pubs[key].publish(msg)

    def _publish_color(self, key: str, rgb: np.ndarray, stamp, frame_id: str) -> None:
        """Raw rgb8 plus a JPEG-compressed copy, on <key> and <key>_compressed."""
        self._publish_image(key, rgb, 'rgb8', stamp, frame_id)
        if not self._wanted(key + '_compressed'):
            return

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
                stamp = _ros_time(stamp_ns)
            timer.mark('frames')
            disp = self._compute_disparity(left, right)
            depth_left = self._depth_from_disparity(disp)
            timer.mark('matching')

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
            timer.mark('colour')

            # Warped onto the central pixel grid, so depth, disparity and RGB
            # all overlay one another.
            depth_c = self._reproject_to_center(depth_left)
            if depth_c is not None:
                depth_pub = np.clip(depth_c, 0.0, self.depth_max_distance)
                depth_pub[~np.isfinite(depth_c)] = np.nan
                self._publish_image('center_depth', depth_pub, '32FC1', stamp, f_center)
                if self._wanted('center_depth_vis'):
                    self._publish_image('center_depth_vis',
                                        self._colourise(depth_pub, self.depth_max_distance),
                                        'bgr8', stamp, f_center)

                # Disparity a virtual pair would see from the centre, using the
                # central focal length: d = fx_c * B / Z.
                if self._wanted('center_disparity'):
                    with np.errstate(divide='ignore', invalid='ignore'):
                        disp_c = (self._c_fx * self._baseline_m) / depth_c
                    disp_c[~np.isfinite(depth_c)] = np.nan
                    self._pubs['center_disparity'].publish(
                        self._disparity_msg(disp_c, stamp, f_center, fx=self._c_fx,
                                            width=self._c_w, height=self._c_h,
                                            valid_window=False))
            timer.mark('depth')

            if depth_gt_cm is not None and (self._wanted('center_depth_gt')
                                            or self._wanted('center_depth_gt_vis')):
                if depth_gt_cm.shape != center.shape[:2]:
                    self.get_logger().warn(
                        f'Ground-truth depth is {depth_gt_cm.shape[1]}x{depth_gt_cm.shape[0]}, '
                        f'the central image {center.shape[1]}x{center.shape[0]}: skipped.',
                        throttle_duration_sec=15.0)
                else:
                    depth_gt = depth_gt_cm / 100.0                       # metres
                    depth_gt[~(depth_gt < self.GROUND_TRUTH_FAR_M)] = np.inf  # sky
                    self._publish_image('center_depth_gt', depth_gt.astype(np.float32),
                                        '32FC1', stamp, f_center)
                    if self._wanted('center_depth_gt_vis'):
                        shown = np.where(np.isfinite(depth_gt),
                                         np.clip(depth_gt, 0.0, self.depth_max_distance), np.nan)
                        self._publish_image('center_depth_gt_vis',
                                            self._colourise(shown, self.depth_max_distance),
                                            'bgr8', stamp, f_center)
            timer.mark('ground truth')

            if center_mask is not None:
                if self.enable_shadow_mask:
                    shadow, labels, center_mask = self._split_shadow_mask(center_mask)
                    self._publish_image('center_shadow_mask', shadow, 'rgb8',
                                        stamp, f_center)
                    self._publish_image('center_shadow_label', labels, 'mono8',
                                        stamp, f_center)
                    msg_ids = String()
                    msg_ids.data = self._shadow_ids_json
                    self._pubs['center_shadow_label_mapping'].publish(msg_ids)
                self._publish_image('center_object_mask', center_mask, 'rgb8',
                                    stamp, f_center)
                msg_map = String()
                msg_map.data = self._cached_color_map_json
                self._pubs['center_object_mask_mapping'].publish(msg_map)
                self._pubs['center_object_mask_classes'].publish(String(data=self._mask_classes_json))
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
        for cam_id, key, frame, hfov in self._extras:
            try:
                got = self._stream.latest(cam_id)
                if got is None or got[0] == self._extra_last_fid.get(cam_id):
                    continue
                fid, rgb, stamp_ns = got
                if rgb.ndim != 3:                   # a depth (R32F) stream: not a colour camera
                    continue
                self._extra_last_fid[cam_id] = fid
                shot = _ros_time(stamp_ns) if stamp_ns else stamp      # capture time when known
                self._publish_color(f'{key}_color', rgb, shot, frame)
                if hfov:
                    h, w = rgb.shape[:2]
                    fx = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
                    self._pubs[f'{key}_info'].publish(self._camera_info(
                        shot, frame, False, intrinsics=(fx, w / 2.0, h / 2.0, w, h)))
            except Exception as exc:
                self.get_logger().error(f'Extra camera {cam_id} publish failed: {exc}',
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
        self._baseline_m = self.baseline_m_param
        self._swap_lr = False
        # Intrinsics need the frame size; solved when the first frame lands.
        self._matcher = None
        self._width = self._height = 0

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
        if self.depth_gt_camera_id >= 0:
            pub('center_depth_gt', RosImage, f'{C}/depth_ground_truth')
            pub('center_depth_gt_vis', RosImage, f'{C}/depth_ground_truth/visualization')

        if self.enable_object_mask:
            pub('center_object_mask', RosImage, f'{C}/object_mask')
            pub('center_object_mask_mapping', String, f'{C}/object_mask_mapping')
            pub('center_object_mask_classes', String, f'{C}/object_mask_classes')
            if self.enable_shadow_mask:
                pub('center_shadow_mask', RosImage, f'{C}/shadow_mask')
                pub('center_shadow_label', RosImage, f'{C}/shadow_label')
                pub('center_shadow_label_mapping', String, f'{C}/shadow_label_mapping')
                self._shadow_ids_json = json.dumps(
                    {str(k): v for k, v in self.SHADOW_IDS.items()})
            # The colours are baked into the Unreal post-process material from the
            # same class file, so publishing it tells a consumer what each id and
            # colour in the mask represents.
            self._cached_color_map_json, self._mask_classes_json = self._load_mask_classes()

        self.get_logger().info(
            f'Publishing {L}/*, {R}/* (colour) and {C}/* '
            f'(colour + PLANAR depth + disparity'
            f'{" + ground-truth depth" if self.depth_gt_camera_id >= 0 else ""}'
            f'{" + object mask" if self.enable_object_mask else ""}'
            f'{" + shadow mask" if self.enable_shadow_mask else ""}). '
            f'Frames: {self.left_frame}, {self.right_frame}, {self.center_frame}.')

        # Extra mono cameras: "<cam_id>:<topic>[:<hfov_deg>]".
        self._extras, self._extra_last_fid = [], {}
        for entry in self.get_parameter('extra_cameras').value or []:
            parts = [x.strip() for x in str(entry).split(':')]
            if len(parts) not in (2, 3) or not parts[0].lstrip('-').isdigit() or not parts[1]:
                self.get_logger().warn(f'extra_cameras: ignored "{entry}" '
                                       '(expected "<cam_id>:<topic>[:<hfov_deg>]").')
                continue
            cam_id, topic = int(parts[0]), parts[1].strip('/')
            hfov = float(parts[2]) if len(parts) == 3 else None
            key = f'extra_{cam_id}'
            pub(f'{key}_color', RosImage, f'{topic}/image_color')
            pub(f'{key}_color_compressed', CompressedImage, f'{topic}/image_color/compressed')
            if hfov:
                pub(f'{key}_info', CameraInfo, f'{topic}/camera_info')
            self._extras.append((cam_id, key, _frame_id(topic), hfov))
            self.get_logger().info(f'Extra camera: stream cam id {cam_id} -> {topic}/image_color'
                                   f'{f" (hfov {hfov:g} deg)" if hfov else ""}.')

        self._stream = FrameStreamReceiver(
            self.get_parameter('stream_bind_ip').value,
            self.get_parameter('stream_port').value,
            self.get_logger())
        # Every camera id a parameter names, so a frame on another id (an Unreal
        # component configured with a different id) is reported once.
        self._stream.known_ids = {i for i in (
            self.left_camera_id, self.right_camera_id, self.center_camera_id,
            self.mask_camera_id if self.enable_object_mask else -1,
            self.depth_gt_camera_id) if i is not None and i >= 0} | {e[0] for e in self._extras}
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


# Shadow-split lookup tables, built once from the class colours.
LifecycleStereoDepth._SHADOW_PALETTE, LifecycleStereoDepth._SHADOW_LABELS = \
    LifecycleStereoDepth._shadow_tables()


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
