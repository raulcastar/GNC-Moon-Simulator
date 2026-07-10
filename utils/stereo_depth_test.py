"""
Stereo disparity / depth from the simulated UnrealCV stereo rig.

Camera 1 = left RGB, camera 2 = right RGB, camera 3 = centre RGB + ground-truth
depth.  Computes an SGBM disparity map, back-projects it to metric depth, and
compares against the ground-truth depth buffer to quantify how much the depth
degrades inside blown-out (glare) regions.

Rig geometry (baseline, relative rotation, HFOV) is read from the running
Unreal instance rather than hard-coded, and the rectification assumptions are
asserted before any matching happens.
"""

import math
import os
import time

import numpy as np
import cv2
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from unrealcv import Client
from unrealcv.util import read_png, read_npy


# ── Configuration ─────────────────────────────────────────────────────────────
CAM_LEFT, CAM_RIGHT, CAM_CENTER = 1, 2, 3

# Unreal Units per metre. Stock UE is 1 UU = 1 cm; this scene uses 1 UU = 1 mm.
# Keep in sync with component_stereo_depth/config/params.yaml. Scales both the
# baseline and the ground-truth depth, so a wrong value corrupts the absolute
# metres but not the stereo/ground-truth ratio.
UU_PER_METRE = 1000.0

MIN_DEPTH_M = 0.30          # closest surface the matcher must resolve
FALLBACK_HFOV_DEG = 90.0    # used only if the server has no fieldofview getter
SATURATION_LEVEL = 250      # a channel at/above this counts as blown out


# ── Unreal geometry helpers ───────────────────────────────────────────────────
def unreal_axes(pitch_deg: float, yaw_deg: float, roll_deg: float):
    """
    Forward / right / up unit vectors for an Unreal FRotator, in world space.

    Mirrors FRotationMatrix: X = forward, Y = right, Z = up, left-handed with
    Z pointing up.
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


def get_vec3(client: Client, cam_id: int, what: str) -> np.ndarray:
    res = client.request(f'vget /camera/{cam_id}/{what}')
    return np.array([float(v) for v in res.split()])


def get_hfov_deg(client: Client, cam_id: int) -> float:
    res = client.request(f'vget /camera/{cam_id}/fov')
    try:
        return float(res)
    except (TypeError, ValueError):
        print(f"  ! server did not return an HFOV ({res!r}); "
              f"assuming {FALLBACK_HFOV_DEG}deg")
        return FALLBACK_HFOV_DEG


def to_planar_depth(point_depth: np.ndarray, fx: float, fy: float,
                    cx: float, cy: float) -> np.ndarray:
    """
    Radial (point) depth -> planar Z depth along the optical axis.

    UnrealCV stores straight-line distance from the camera centre; stereo
    triangulation produces the Z component. unrealcv.util.convert2planedepth
    does this but calls np.float, removed in NumPy 1.24.
    """
    h, w = point_depth.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float64),
                       np.arange(h, dtype=np.float64))
    px = (u - cx) / fx
    py = (v - cy) / fy
    return point_depth / np.sqrt(1.0 + px ** 2 + py ** 2)


# ── Capture ───────────────────────────────────────────────────────────────────
def capture(client: Client):
    """
    Grab the stereo pair, the centre view and the centre depth as one snapshot.

    Each vget is a separate round-trip; pausing makes the four reads describe a
    single instant, so the rover cannot move between the left and right grabs.
    """
    paused = client.request('vset /action/game/pause')
    if paused != 'ok':
        print(f"  ! could not pause the game ({paused!r}); the stereo pair will "
              f"be captured at slightly different times")
    try:
        left = read_png(client.request(f'vget /camera/{CAM_LEFT}/lit png'))
        right = read_png(client.request(f'vget /camera/{CAM_RIGHT}/lit png'))
        center = read_png(client.request(f'vget /camera/{CAM_CENTER}/lit png'))
        depth_raw = read_npy(client.request(f'vget /camera/{CAM_CENTER}/depth npy'))
    finally:
        if paused == 'ok':
            client.request('vset /action/game/resume')

    drop_alpha = lambda im: im[:, :, :3] if im.shape[2] == 4 else im
    return (drop_alpha(left), drop_alpha(right), drop_alpha(center),
            depth_raw.astype(np.float64))


# ── Rig geometry ──────────────────────────────────────────────────────────────
def solve_rig(client: Client, width: int):
    """
    Read the rig out of Unreal and check it is a rectified stereo pair.

    Returns (baseline_m, fx_px, swap) where swap is True when camera 2 sits to
    the *left* of camera 1 and the images must be exchanged before matching.
    """
    loc_l = get_vec3(client, CAM_LEFT, 'location')
    loc_r = get_vec3(client, CAM_RIGHT, 'location')
    rot_l = get_vec3(client, CAM_LEFT, 'rotation')
    rot_r = get_vec3(client, CAM_RIGHT, 'rotation')

    rot_err = np.abs(rot_r - rot_l).max()
    if rot_err > 1e-3:
        print(f"  ! left/right rotations differ by up to {rot_err:.4f}deg "
              f"(pitch/yaw/roll {rot_l} vs {rot_r}).\n"
              f"    The pair is NOT rectified; epipolar lines are not image rows "
              f"and SGBM results will be wrong.")

    _, right_vec, up_vec = unreal_axes(*rot_l)
    baseline_vec = loc_r - loc_l                      # Unreal Units
    lateral = float(np.dot(baseline_vec, right_vec))  # signed, along +right
    residual = float(np.linalg.norm(baseline_vec - lateral * right_vec))
    if residual > 0.01 * abs(lateral):
        print(f"  ! baseline is not purely lateral: {residual:.2f} UU of the "
              f"{np.linalg.norm(baseline_vec):.2f} UU offset is vertical/forward.\n"
              f"    Move camera {CAM_RIGHT} along camera {CAM_LEFT}'s right axis only.")

    baseline_m = abs(lateral) / UU_PER_METRE
    hfov = get_hfov_deg(client, CAM_LEFT)
    fx = (width / 2.0) / math.tan(math.radians(hfov) / 2.0)

    print(f"  baseline   {abs(lateral):.2f} UU = {baseline_m * 100:.2f} cm")
    print(f"  hfov       {hfov:.2f}deg  ->  fx = {fx:.2f} px")
    return baseline_m, fx, lateral < 0


# ── Stereo matching ───────────────────────────────────────────────────────────
def compute_disparity(left_rgb: np.ndarray, right_rgb: np.ndarray,
                      fx: float, baseline_m: float) -> np.ndarray:
    """
    SGBM disparity in the left image frame.  Invalid pixels come back as NaN.
    """
    gray_l = cv2.cvtColor(left_rgb, cv2.COLOR_RGB2GRAY)
    gray_r = cv2.cvtColor(right_rgb, cv2.COLOR_RGB2GRAY)

    # Enough disparity levels to reach MIN_DEPTH_M; SGBM needs a multiple of 16.
    max_disp = fx * baseline_m / MIN_DEPTH_M
    num_disp = min(int(math.ceil(max_disp / 16.0)) * 16, 256)

    block = 5
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block,
        P1=8 * block * block,
        P2=32 * block * block,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    print(f"  numDisparities {num_disp} (resolves down to "
          f"{fx * baseline_m / num_disp:.2f} m)")

    disp = matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
    disp[disp <= 0] = np.nan
    return disp


# ── Glare analysis ────────────────────────────────────────────────────────────
def saturation_mask(rgb: np.ndarray) -> np.ndarray:
    """Pixels where any channel has clipped — the blown-out core of the glare."""
    return rgb.max(axis=2) >= SATURATION_LEVEL


def report(depth_stereo, depth_gt, sat, disp):
    """Print how depth quality differs inside vs outside the glare."""
    # Dilate: the damage from a blown highlight bleeds into the matching window.
    sat_zone = cv2.dilate(sat.astype(np.uint8),
                          np.ones((11, 11), np.uint8)).astype(bool)

    gt_ok = np.isfinite(depth_gt) & (depth_gt > MIN_DEPTH_M)
    matched = np.isfinite(disp)

    print(f"\n  saturated pixels         {sat.mean() * 100:6.2f} %")
    print(f"  unmatched (no disparity) {(~matched).mean() * 100:6.2f} % overall")
    for name, zone in (("inside glare", sat_zone & gt_ok),
                       ("outside glare", ~sat_zone & gt_ok)):
        if not zone.any():
            continue
        hole_rate = (~matched[zone]).mean() * 100
        both = zone & matched
        if both.any():
            rel = np.abs(depth_stereo[both] - depth_gt[both]) / depth_gt[both]
            print(f"  {name:14s}  holes {hole_rate:5.1f} %   "
                  f"median |dZ|/Z {np.median(rel) * 100:5.2f} %   "
                  f"p95 {np.percentile(rel, 95) * 100:6.2f} %")
        else:
            print(f"  {name:14s}  holes {hole_rate:5.1f} %   (no matched pixels)")


def show(left, right, disp, depth_stereo, depth_gt, sat):
    fig, ax = plt.subplots(2, 3, figsize=(16, 9))

    overlay = left.copy()
    overlay[sat] = [255, 0, 255]
    ax[0, 0].imshow(overlay)
    ax[0, 0].set_title(f'Left (cam {CAM_LEFT}) — saturated pixels in magenta')

    ax[0, 1].imshow(right)
    ax[0, 1].set_title(f'Right (cam {CAM_RIGHT})')

    im = ax[0, 2].imshow(disp, cmap='jet')
    ax[0, 2].set_title('SGBM disparity (px)')
    fig.colorbar(im, ax=ax[0, 2], fraction=0.046)

    vmax = np.nanpercentile(depth_gt, 99)
    im = ax[1, 0].imshow(depth_stereo, cmap='jet', vmin=0, vmax=vmax)
    ax[1, 0].set_title('Stereo depth (m)')
    fig.colorbar(im, ax=ax[1, 0], fraction=0.046)

    im = ax[1, 1].imshow(depth_gt, cmap='jet', vmin=0, vmax=vmax)
    ax[1, 1].set_title('Ground-truth planar depth (m)')
    fig.colorbar(im, ax=ax[1, 1], fraction=0.046)

    rel = np.abs(depth_stereo - depth_gt) / depth_gt
    im = ax[1, 2].imshow(rel * 100, cmap='inferno', vmin=0, vmax=20)
    ax[1, 2].set_title('|dZ| / Z  (%)')
    fig.colorbar(im, ax=ax[1, 2], fraction=0.046)

    for a in ax.ravel():
        a.axis('off')
    plt.tight_layout()
    plt.show()


def main():
    ip = "127.0.0.1"
    if os.path.exists("ip.txt"):
        with open("ip.txt") as f:
            ip = f.readline().strip()

    client = Client((ip, 9000), 'inet')
    while not client.isconnected():
        print('Trying to connect to UnrealCV...')
        client.connect()
        time.sleep(1)
    print('Connection established.\n')

    try:
        left, right, _center, depth_raw = capture(client)
        h, w = depth_raw.shape
        print(f"Captured {w}x{h}")
        print(f"  raw depth range [{depth_raw.min():.1f}, "
              f"{depth_raw[np.isfinite(depth_raw)].max():.1f}] "
              f"(sky/background sits at the float16 max, 65504)")

        baseline_m, fx, swap = solve_rig(client, w)
        if swap:
            print(f"  camera {CAM_RIGHT} is left of camera {CAM_LEFT}; swapping")
            left, right = right, left
    finally:
        client.disconnect()

    fy, cx, cy = fx, w / 2.0, h / 2.0

    disp = compute_disparity(left, right, fx, baseline_m)
    depth_stereo = fx * baseline_m / disp          # NaN propagates from disp

    # The depth buffer is radial; triangulation is planar.  Convert before any
    # comparison: at 90deg HFOV the two disagree by 60% in the image corners,
    # which on its own looks exactly like barrel distortion.
    depth_gt = to_planar_depth(depth_raw, fx, fy, cx, cy) / UU_PER_METRE
    depth_gt[depth_raw >= 65500.0] = np.nan        # float16 max = sky/background

    sat = saturation_mask(left)
    report(depth_stereo, depth_gt, sat, disp)

    print(f"\n  Absolute metres assume 1 m = {UU_PER_METRE} UU. This constant "
          f"scales the baseline and the\n  ground truth identically, so "
          f"|dZ|/Z above is unaffected by it — only the axis labels are.")
    show(left, right, disp, depth_stereo, depth_gt, sat)


if __name__ == '__main__':
    main()
