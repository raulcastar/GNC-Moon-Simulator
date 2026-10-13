"""
base_link's pose from Unreal's telemetry (pose_from_telemetry): the world
transform of the Blueprint's base_link (X forward, Z up) in Unreal's
left-handed centimetres, into ROS odom (REP-103: x forward, y left, z up).
"""
import math

import pytest

from src.component_rover_leo_raph import pose_from_telemetry


def ypr(q):
    x, y, z, w = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return pose_from_telemetry({'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw})


def test_position_is_mirrored_in_x_and_in_metres():
    position, _ = pose(x=-61015.8, y=8023.0, z=2019.5)
    assert position == pytest.approx((610.158, 80.230, 20.195))


@pytest.mark.parametrize('yaw_ue, heading', [(90.0, 90.0), (0.0, 180.0), (180.0, 0.0), (-90.0, -90.0), (30.0, 150.0)])
def test_heading(yaw_ue, heading):
    # Facing Unreal +Y is ROS +y (heading 90); +X is ROS -x (heading 180).
    h, p, r = ypr(pose(yaw=yaw_ue)[1])
    assert math.cos(math.radians(h - heading)) == pytest.approx(1.0)
    assert p == pytest.approx(0.0, abs=1e-9) and r == pytest.approx(0.0, abs=1e-9)


def test_nose_up_is_negative_pitch():
    # Unreal pitch +5: nose up. REP-103: pitch about y (left) is positive nose down.
    _h, p, _r = ypr(pose(yaw=90.0, pitch=5.0)[1])
    assert p == pytest.approx(-5.0)


def test_roll_keeps_its_sense():
    # Unreal roll +10 tips the right side down; REP-103 roll +10 too (left side up).
    _h, _p, r = ypr(pose(yaw=90.0, roll=10.0)[1])
    assert r == pytest.approx(10.0)


def test_same_heading_as_the_old_payload_sensor_formula():
    # The old telemetry came from payload_sensor, yawed 180 from the rover, with
    # heading = -yaw_payload - 90. base_link is yawed +90 from the rover.
    for yaw_root in (0.0, 37.0, -120.0):
        h, _p, _r = ypr(pose(yaw=yaw_root + 90.0)[1])
        old = -(yaw_root + 180.0) - 90.0
        assert math.cos(math.radians(h - old)) == pytest.approx(1.0)
