"""Rig geometry, SGBM matching, depth and reprojection (stereo_geometry)."""

import math

import numpy as np
import pytest

from src import stereo_geometry as geo


def rig(width=320, height=240, hfov=90.0, baseline=0.2, offset=-1.0, center=None):
    return geo.solve_rig((height, width, 3), center or (height, width, 3), hfov, baseline, offset)


def matcher(r, nd=64, fill=True, max_depth=50.0):
    return geo.StereoMatcher(r, nd, 7, 10, 100, 2, max_depth, fill, False)


def test_focal_length_from_fov():
    assert geo.focal_from_hfov(640, 90.0) == pytest.approx(320.0)
    assert geo.focal_from_hfov(640, 60.0) == pytest.approx(320.0 / math.tan(math.radians(30.0)))


def test_rig_from_frames_and_parameters():
    r = rig(320, 240, 90.0, 0.2, offset=-1.0, center=(480, 640, 3))
    assert (r.width, r.height, r.cx, r.cy) == (320, 240, 160.0, 120.0)
    assert r.fx == pytest.approx(160.0)
    assert (r.c_width, r.c_height, r.c_fx) == (640, 480, pytest.approx(320.0))
    assert r.c_offset_m == pytest.approx(0.1)                     # negative: the midpoint
    assert rig(offset=0.05).c_offset_m == pytest.approx(0.05)
    np.testing.assert_allclose(r.c_t, [0.1, 0.0, 0.0])


@pytest.mark.parametrize("requested, used", [(64, 64), (60, 64), (0, 16), (-5, 16), (500, 256), (256, 256)])
def test_num_disparities_multiple_of_16(requested, used):
    assert geo.normalise_num_disparities(requested) == used


def test_near_limit_and_report():
    m = matcher(rig(), nd=60)
    assert m.num_disparities == 64
    assert m.near_limit_m == pytest.approx(160.0 * 0.2 / 64)
    levels = [level for level, _ in m.report()]
    assert levels[0] == 'warn'                                    # 60 was adjusted


def test_depth_from_disparity_and_window():
    m = matcher(rig(), max_depth=10.0)
    disp = np.array([[32.0, 4.0, 1.0, np.nan, 100.0]], dtype=np.float32)
    depth = m.depth(disp)
    assert depth[0, 0] == pytest.approx(1.0)                      # fx*B/d = 32/32
    assert depth[0, 1] == pytest.approx(8.0)
    assert np.isnan(depth[0, 2])                                  # 32 m > max_depth
    assert np.isnan(depth[0, 3])
    assert np.isnan(depth[0, 4])                                  # 0.32 m < near limit 0.5 m


def test_sgbm_recovers_a_known_shift():
    rng = np.random.default_rng(0)
    texture = (rng.random((240, 400)) * 255).astype(np.uint8)
    texture = np.repeat(texture[:, :, None], 3, axis=2)
    shift = 16                                   # a point at left column u is at u - 16 in the right image
    left = np.ascontiguousarray(texture[:, :320])
    right = np.ascontiguousarray(texture[:, shift:shift + 320])
    disp = matcher(rig()).disparity(left, right)
    centre = disp[40:200, 120:280]
    assert np.nanmedian(centre) == pytest.approx(shift, abs=0.25)
    assert np.isfinite(centre).mean() > 0.9


def test_reprojection_is_identity_without_offset():
    r = rig(offset=0.0)
    depth = np.full((240, 320), np.nan, np.float32)
    depth[50:150, 60:200] = 4.0
    out = geo.reproject_to_center(depth, r, near_limit_m=0.5)
    np.testing.assert_allclose(out[50:150, 60:200], 4.0)
    assert np.isnan(out[:40]).all()


def test_reprojection_shifts_by_fx_t_over_z():
    r = rig(offset=0.1)                                           # centre 0.1 m right of the left eye
    depth = np.full((240, 320), np.nan, np.float32)
    depth[100:140, 100:140] = 2.0
    out = geo.reproject_to_center(depth, r, near_limit_m=0.5)
    shift = r.fx * 0.1 / 2.0                                      # 8 px to the left in the centre view
    cols = np.nonzero(np.isfinite(out[120]))[0]
    assert cols.min() == pytest.approx(100 - shift, abs=1) and cols.max() == pytest.approx(139 - shift, abs=1)


def test_reprojection_of_nothing():
    assert geo.reproject_to_center(np.full((10, 10), np.nan), rig(10, 10), 0.5) is None


def test_center_disparity_and_clip():
    r = rig()
    depth = np.array([[2.0, np.nan, 100.0]])
    disp = geo.center_disparity(depth, r)
    assert disp[0, 0] == pytest.approx(r.c_fx * 0.2 / 2.0) and np.isnan(disp[0, 1])
    clipped = geo.clip_depth(depth, 10.0)
    assert clipped[0, 0] == 2.0 and np.isnan(clipped[0, 1]) and clipped[0, 2] == 10.0


def test_ground_truth_metres_and_sky():
    gt = geo.ground_truth_depth_m(np.array([[250.0, 1e9, np.inf]], np.float32))
    assert gt.dtype == np.float32
    assert gt[0, 0] == pytest.approx(2.5) and np.isinf(gt[0, 1]) and np.isinf(gt[0, 2])


def test_colourise_black_where_invalid():
    vis = geo.colourise(np.array([[0.0, 5.0, np.nan]]), 10.0)
    assert vis.shape == (1, 3, 3) and vis.dtype == np.uint8
    assert (vis[0, 2] == 0).all() and vis[0, 1].any()
