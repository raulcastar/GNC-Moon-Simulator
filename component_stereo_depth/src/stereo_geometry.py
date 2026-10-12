"""
Stereo geometry and matching for the navcam rig. No ROS, no I/O.

The rig is declared, not measured: the stream carries no camera poses, so the
baseline and horizontal FOV come from parameters and the image size from the
first frames. The eyes are co-rotated (SGBM needs it), and the central camera
sits on the baseline.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

# Ground-truth depth beyond this (metres) has no surface: the sky (+Inf).
GROUND_TRUTH_FAR_M = 1.0e5


def focal_from_hfov(width: int, hfov_deg: float) -> float:
    """Focal length in pixels of a pinhole camera `width` px wide with this FOV."""
    return (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


@dataclass
class StereoRig:
    """Intrinsics of the eyes and of the central camera, and their geometry."""
    width: int
    height: int
    fx: float
    cx: float
    cy: float
    baseline_m: float
    c_width: int
    c_height: int
    c_fx: float
    c_cx: float
    c_cy: float
    c_offset_m: float          # centre camera, metres right of the left eye

    @property
    def c_t(self) -> np.ndarray:
        """Centre camera position in the left eye's optical frame (x = right)."""
        return np.array([self.c_offset_m, 0.0, 0.0])

    @property
    def c_R(self) -> np.ndarray:
        """Centre camera rotation relative to the left eye (co-rotated)."""
        return np.eye(3)

    @property
    def center_intrinsics(self) -> Tuple[float, float, float, int, int]:
        """(fx, cx, cy, width, height) of the central camera."""
        return self.c_fx, self.c_cx, self.c_cy, self.c_width, self.c_height


def solve_rig(eye_shape, center_shape, hfov_deg: float, baseline_m: float,
              center_offset_m: float) -> StereoRig:
    """
    Rig geometry from parameters and the first streamed frames: image sizes
    from the frames, focal lengths from the FOV, the centre camera on the
    baseline at center_offset_m from the left eye (the midpoint when negative).
    """
    height, width = eye_shape[:2]
    c_height, c_width = center_shape[:2]
    offset = center_offset_m if center_offset_m >= 0.0 else baseline_m / 2.0
    return StereoRig(
        width=width, height=height, fx=focal_from_hfov(width, hfov_deg),
        cx=width / 2.0, cy=height / 2.0, baseline_m=baseline_m,
        c_width=c_width, c_height=c_height, c_fx=focal_from_hfov(c_width, hfov_deg),
        c_cx=c_width / 2.0, c_cy=c_height / 2.0, c_offset_m=offset)


def normalise_num_disparities(requested: int) -> int:
    """A positive multiple of 16 (SGBM requires it), at most 256."""
    nd = requested
    if nd % 16 != 0 or nd <= 0:
        nd = max(16, min(int(math.ceil(nd / 16.0)) * 16, 256))
    return min(nd, 256)


class StereoMatcher:
    """
    SGBM on a rectified pair, with the near limit that follows from the
    disparity search width:

        near = fx * B / num_disparities

    Depth resolution follows dZ = Z^2 / (fx * B): a short baseline and a wide
    FOV leave the far field coarsely quantised.
    """

    def __init__(self, rig: StereoRig, num_disparities: int, block_size: int,
                 uniqueness_ratio: int, speckle_window_size: int, speckle_range: int,
                 max_depth_m: float, fill_left_border: bool, equalize_for_glare: bool):
        self.rig = rig
        self.requested_disparities = num_disparities
        self.num_disparities = normalise_num_disparities(num_disparities)
        self.max_depth_m = max_depth_m
        self.fill_left_border = fill_left_border
        self.fxb = rig.fx * rig.baseline_m
        self.near_limit_m = self.fxb / self.num_disparities
        self.block = block_size | 1   # SGBM requires an odd block size
        self._sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self.num_disparities,
            blockSize=self.block,
            P1=8 * self.block * self.block,
            P2=32 * self.block * self.block,
            disp12MaxDiff=1,
            uniquenessRatio=uniqueness_ratio,
            speckleWindowSize=speckle_window_size,
            speckleRange=speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self._clahe = (cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
                       if equalize_for_glare else None)

    def report(self) -> List[Tuple[str, str]]:
        """(level, message) lines describing the matcher's limits, for the log."""
        lines = []
        if self.num_disparities != self.requested_disparities:
            lines.append(('warn', 'num_disparities must be a positive multiple of 16; '
                                  f'using {self.num_disparities}.'))
        far = self.fxb / (1.0 / 16.0)   # one sub-pixel step of disparity
        dz_at_max = self.max_depth_m ** 2 / self.fxb
        lines.append(('info', f'SGBM: numDisparities {self.num_disparities}, blockSize {self.block} | '
                              f'near limit fx*B/nd = {self.near_limit_m:.2f} m .. {far:.0f} m'))
        if self.fill_left_border:
            # A surface at Z is recoverable from column u >= fx*B/Z.
            recover_at = self.fxb / self.max_depth_m
            lines.append(('info', f'Left border recovered per-pixel (d <= u): a surface at '
                                  f'{self.max_depth_m:.0f} m is matchable from column '
                                  f'{recover_at:.0f} onward, not column {self.num_disparities}'))
        else:
            lines.append(('warn', f'fill_left_border is off: the leftmost {self.num_disparities} '
                                  f'columns ({self.num_disparities / self.rig.width * 100:.0f}% of the '
                                  f'image) will be blank regardless of scene depth'))
        lines.append(('info', f'Depth resolution dZ = Z^2/(fx*B), fx*B = {self.fxb:.2f} m*px -> '
                              f'{dz_at_max * 100:.1f} cm per disparity pixel at {self.max_depth_m:.1f} m'))
        if dz_at_max > 0.1 * self.max_depth_m:
            lines.append(('warn', f'At max_depth_m={self.max_depth_m:.1f} m one pixel of matching '
                                  f'error is a {dz_at_max / self.max_depth_m * 100:.0f}% depth error. '
                                  f'Widen the baseline or narrow the FOV before trusting far-field depth.'))
        return lines

    def disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """SGBM disparity in the left frame (RGB inputs), sub-pixel, NaN where unmatched."""
        gray_l = cv2.cvtColor(left, cv2.COLOR_RGB2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_RGB2GRAY)

        # CLAHE pulls texture back out of near-saturated pixels. It cannot
        # recover a fully clipped highlight, where the texture is simply gone.
        if self._clahe is not None:
            gray_l = self._clahe.apply(gray_l)
            gray_r = self._clahe.apply(gray_r)

        if not self.fill_left_border:
            disp = self._sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0
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
        d_max = self.num_disparities
        pad_l = cv2.copyMakeBorder(gray_l, 0, 0, d_max, 0, cv2.BORDER_REPLICATE)
        pad_r = cv2.copyMakeBorder(gray_r, 0, 0, d_max, 0, cv2.BORDER_REPLICATE)

        disp = self._sgbm.compute(pad_l, pad_r).astype(np.float32) / 16.0
        disp = disp[:, d_max:]                       # back to the left frame

        u = np.arange(disp.shape[1], dtype=np.float32)[None, :]
        disp[u - disp < 0.0] = -1.0                  # outside the right FOV
        disp[disp <= 0.0] = np.nan
        return np.ascontiguousarray(disp)

    def depth(self, disp: np.ndarray) -> np.ndarray:
        """Z = fx * B / d, clamped to the matcher's depth window, NaN outside it."""
        with np.errstate(divide='ignore', invalid='ignore'):
            depth = self.fxb / disp
        depth[~np.isfinite(depth)] = np.nan
        depth[(depth < self.near_limit_m) | (depth > self.max_depth_m)] = np.nan
        return depth.astype(np.float32)


def reproject_to_center(depth_left: np.ndarray, rig: StereoRig,
                        near_limit_m: float) -> Optional[np.ndarray]:
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
    X = (uu - rig.cx) * Z / rig.fx
    Y = (vv - rig.cy) * Z / rig.fx        # fy == fx for a square pixel

    P_c = rig.c_R @ (np.stack([X, Y, Z]) - rig.c_t[:, None])   # (3, N)
    Zc = P_c[2]

    front = Zc > near_limit_m             # behind the centre camera: drop
    if not front.any():
        return None
    Zc = Zc[front]
    uf = rig.c_fx * P_c[0][front] / Zc + rig.c_cx     # sub-pixel
    vf = rig.c_fx * P_c[1][front] / Zc + rig.c_cy

    w, h = rig.c_width, rig.c_height

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


def clip_depth(depth: np.ndarray, max_distance: float) -> np.ndarray:
    """Depth clipped to [0, max_distance], NaN kept where there is none."""
    clipped = np.clip(depth, 0.0, max_distance)
    clipped[~np.isfinite(depth)] = np.nan
    return clipped


def center_disparity(depth_c: np.ndarray, rig: StereoRig) -> np.ndarray:
    """Disparity a virtual pair would see from the centre: d = fx_c * B / Z."""
    with np.errstate(divide='ignore', invalid='ignore'):
        disp_c = (rig.c_fx * rig.baseline_m) / depth_c
    disp_c[~np.isfinite(depth_c)] = np.nan
    return disp_c


def ground_truth_depth_m(depth_cm: np.ndarray) -> np.ndarray:
    """Unreal's planar depth in cm -> metres, +Inf where there is no surface (sky)."""
    depth = depth_cm / 100.0
    depth[~(depth < GROUND_TRUTH_FAR_M)] = np.inf
    return depth.astype(np.float32)


def colourise(values: np.ndarray, vmax: float) -> np.ndarray:
    """Scalar field -> JET bgr8, invalid pixels black."""
    norm = np.nan_to_num(values, nan=0.0) / max(vmax, 1e-6)
    vis = cv2.applyColorMap((np.clip(norm, 0.0, 1.0) * 255).astype(np.uint8),
                            cv2.COLORMAP_JET)
    vis[~np.isfinite(values)] = 0
    return np.ascontiguousarray(vis)
