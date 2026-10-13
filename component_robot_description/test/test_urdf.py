"""
The exported URDF (urdf/leo_raph.urdf): every frame the nodes stamp with is in
it, and the geometry they rely on holds (stereo baseline, optical frames,
lidar mount, ground under base_link).
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

URDF = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'urdf', 'leo_raph.urdf')

# frame_id of each publisher (component configs and the stereo node's naming)
FRAMES = ['base_link', 'base_footprint', 'navcam_left', 'navcam_central', 'navcam_right',
          'rear_cam', 'event_camera', 'spad', 'lidar']


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


@pytest.fixture(scope='module')
def poses():
    """Every link's pose in base_link: {name: (R 3x3, t 3)}."""
    joints = {}
    for j in ET.parse(URDF).getroot().iter('joint'):
        o = j.find('origin')
        xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
        joints[j.find('child').get('link')] = (j.find('parent').get('link'), rotation(*rpy), np.array(xyz))
    out = {'base_link': (np.eye(3), np.zeros(3))}

    def resolve(link):
        if link not in out:
            parent, R, t = joints[link]
            Rp, tp = resolve(parent)
            out[link] = (Rp @ R, Rp @ t + tp)
        return out[link]

    for link in joints:
        resolve(link)
    return out


def test_every_published_frame_is_in_the_tree(poses):
    assert set(FRAMES) <= set(poses)


def test_stereo_pair_is_a_lateral_baseline(poses):
    left, right, centre = poses['navcam_left'][1], poses['navcam_right'][1], poses['navcam_central'][1]
    assert np.linalg.norm(left - right) == pytest.approx(0.20, abs=0.005)
    assert left[1] > centre[1] > right[1]                  # left eye on the left (+y)
    np.testing.assert_allclose(left[[0, 2]], right[[0, 2]], atol=1e-3)


def test_optical_frames_look_along_the_camera(poses):
    for cam in ('navcam_central', 'rear_cam'):
        R_link, _ = poses[f'{cam}_link']
        R_opt, _ = poses[cam]
        np.testing.assert_allclose(R_opt[:, 2], R_link[:, 0], atol=1e-5)    # optical z = view
        np.testing.assert_allclose(R_opt[:, 1], -R_link[:, 2], atol=1e-5)   # optical y = down


def test_navcams_look_forward_and_down_the_rear_cam_back(poses):
    view = poses['navcam_central_link'][0][:, 0]
    assert view[0] > 0.9 and view[2] < -0.3                # forward, pitched down (20 deg)
    assert poses['rear_cam_link'][0][:, 0][0] < -0.9


def test_lidar_rides_on_the_central_navcam(poses):
    np.testing.assert_allclose(poses['lidar'][1], poses['navcam_central_link'][1], atol=1e-4)
    np.testing.assert_allclose(poses['lidar'][0], poses['navcam_central_link'][0], atol=1e-4)


def test_ground_is_a_wheel_radius_below_the_wheels(poses):
    ground = poses['base_footprint'][1][2]
    wheels = [poses[w][1][2] for w in ('wheel_fl_link', 'wheel_fr_link', 'wheel_rl_link', 'wheel_rr_link')]
    radius = float(ET.parse(URDF).getroot().find(".//link[@name='wheel_fl_link']//cylinder").get('radius'))
    assert ground == pytest.approx(np.mean(wheels) - radius, abs=1e-3)
    assert 0.2 < -ground < 0.35
