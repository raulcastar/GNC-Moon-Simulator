"""
The optional velocity fields of Unreal's telemetry (vx, vy, vz in cm/s and
wx, wy, wz in rad/s, world frame, Unreal's axes) as a body-frame twist.

Positions map to ROS as (-x, y, z) / 100 and the heading is -yaw - 90, so the
velocities must follow the same mapping.
"""
import math

import pytest

from src.component_rover_leo_raph import _q_from_euler, twist_from_telemetry


def telemetry(vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
    return {'vx': vx, 'vy': vy, 'vz': vz, 'wx': wx, 'wy': wy, 'wz': wz}


@pytest.mark.parametrize('yaw_ue', [0.0, 30.0, -120.0, 175.0])
def test_driving_forward_is_plus_x_in_base_link(yaw_ue):
    heading = math.radians(-yaw_ue - 90.0)
    # 1 m/s along the heading in the ROS world, written in Unreal's axes.
    t = telemetry(vx=-100.0 * math.cos(heading), vy=100.0 * math.sin(heading))
    lin, ang = twist_from_telemetry(t, _q_from_euler(0.0, 0.0, math.degrees(heading)))
    assert lin == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    assert ang == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_yaw_rate_has_the_heading_sign():
    # Unreal's yaw grows with +wz, and heading = -yaw - 90 falls: a negative
    # (clockwise) ROS yaw rate.
    _lin, ang = twist_from_telemetry(telemetry(wz=0.5), _q_from_euler(0.0, 0.0, 40.0))
    assert ang == pytest.approx((0.0, 0.0, -0.5), abs=1e-9)


def test_climbing_is_plus_z_and_units_are_m_s():
    lin, _ang = twist_from_telemetry(telemetry(vz=25.0), _q_from_euler(0.0, 0.0, 0.0))
    assert lin == pytest.approx((0.0, 0.0, 0.25), abs=1e-12)


def test_pitched_body_sees_world_velocity_in_its_own_axes():
    # Nose 10 deg down (positive pitch), driving horizontally along +x:
    # in the body frame part of that velocity points along body z.
    p = math.radians(10.0)
    lin, _ang = twist_from_telemetry(telemetry(vx=-100.0), _q_from_euler(0.0, 10.0, 0.0))
    assert lin == pytest.approx((math.cos(p), 0.0, math.sin(p)), abs=1e-9)
