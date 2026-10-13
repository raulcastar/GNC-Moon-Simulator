"""The rover's state from odometry (rover_state)."""

import math
import types

import numpy as np
import pytest

from src import rover_state as rs


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return types.SimpleNamespace(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                                 z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


def test_yaw_and_tilt_from_the_attitude():
    assert rs.yaw_from_quaternion(quat(yaw=1.2)) == pytest.approx(1.2)
    assert rs.tilt_deg(quat(yaw=2.0)) == pytest.approx(0.0, abs=1e-6)
    assert rs.tilt_deg(quat(pitch=math.radians(20))) == pytest.approx(20.0)
    assert rs.tilt_deg(quat(roll=math.radians(10), yaw=0.5)) == pytest.approx(10.0)


def test_session_jump():
    assert not rs.is_session_jump(None, 5.0, 5.0, 2.0)
    assert not rs.is_session_jump((0.0, 0.0, 0.0), 1.0, 1.0, 2.0)
    assert rs.is_session_jump((0.0, 0.0, 0.0), 3.0, 0.0, 2.0)


def test_trajectory_points_are_spaced():
    history = []
    for x in np.arange(0.0, 1.0, 0.03):
        rs.record_point(history, float(x), 0.0)
    gaps = np.diff([p[0] for p in history])
    assert len(history) > 5 and (gaps > rs.TRAJECTORY_SPACING_M).all()
