"""
The IMU from /odom: with a measured twist (velocities from Unreal's physics,
marked by a non-zero twist covariance) and without (the pose differentiated).
"""
import math

import numpy as np
import pytest
import rclpy

from nav_msgs.msg import Odometry

from src.component_imu import LifecycleImu

G = 1.62


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def imu(ros):
    node = LifecycleImu('test_imu')
    node.gravity, node.frame_id, node.alpha = G, 'base_link', 0.0
    node.accel_noise = node.gyro_noise = 0.0
    node.max_dt = 0.5
    node._is_active = True
    out = []
    node._pub = type('Pub', (), {'publish': lambda self, m: out.append(m)})()
    node.out = out
    yield node
    node.destroy_node()


def odom(t, x=0.0, yaw=0.0, vx=None, wz=None):
    m = Odometry()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    m.pose.pose.position.x = x
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    if vx is not None:
        m.twist.twist.linear.x, m.twist.twist.angular.z = vx, wz
        m.twist.covariance = [1e-4 if i % 7 == 0 else 0.0 for i in range(36)]
    return m


def accel(m):
    a = m.linear_acceleration
    return np.array([a.x, a.y, a.z])


def test_measured_twist_gives_gyro_at_once_and_accel_from_one_difference(imu):
    imu._odom_cb(odom(10.0, vx=0.0, wz=0.3))
    assert imu.out[-1].angular_velocity.z == pytest.approx(0.3)   # no wait for a second pose
    imu._odom_cb(odom(10.1, x=0.0, vx=0.2, wz=0.3))                 # position not used
    np.testing.assert_allclose(accel(imu.out[-1]), [2.0, 0.0, G], atol=1e-6)


def test_measured_velocity_is_rotated_with_the_heading(imu):
    yaw = math.radians(90.0)
    imu._odom_cb(odom(10.0, yaw=yaw, vx=0.0, wz=0.0))
    imu._odom_cb(odom(10.1, yaw=yaw, vx=0.2, wz=0.0))
    np.testing.assert_allclose(accel(imu.out[-1]), [2.0, 0.0, G], atol=1e-6)   # body frame


def test_without_twist_the_pose_is_differentiated_twice(imu):
    for i, x in enumerate([0.0, 0.0, 0.01]):              # v: 0, then 0.1 m/s over 0.1 s
        imu._odom_cb(odom(10.0 + 0.1 * i, x=x))
    np.testing.assert_allclose(accel(imu.out[-1]), [1.0, 0.0, G], atol=1e-6)
