#!/usr/bin/env python3
"""
ROS 2 Lifecycle Node — Simulated IMU from Unreal telemetry (via /odom).

The rover node (component_rover_leo_raph) turns Unreal Engine's pose telemetry
into nav_msgs/Odometry. This node turns that pose stream into a sensor_msgs/Imu
the way a strap-down IMU would report it:

  orientation          — the rover attitude, straight from odom (REP-103 frame).
                         Whatever orientation odom carries flows through, so once
                         the rover publishes real roll/pitch this IMU gets them
                         for free — no change here.
  angular_velocity     — body-frame rates from the quaternion time-derivative.
  linear_acceleration  — body-frame specific force: proper acceleration minus
                         gravity, i.e. what an accelerometer physically reads.
                         At rest and level this is (0, 0, +g).

When the rover Blueprint also sends the physics body's velocities, the rover
node fills odom's twist (marked by a non-zero twist covariance). The gyro is
then that angular velocity, and the acceleration is one difference of that
velocity rather than two of the position. Without them, everything is a finite
difference of the pose. Either way an EMA smoothing factor and optional
Gaussian noise make the signal behave like hardware instead of a perfect
analytic value. A pose gap (e.g. an Unreal Play/Stop teleport) resets the
differentiator so it does not emit a huge spurious spike.

Publishes: imu/data (sensor_msgs/Imu) in frame `imu_frame_id`.
"""
import math
from typing import Optional

import numpy as np

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


# ── Quaternion helpers (x, y, z, w) ──────────────────────────────────────────
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def q_rot(q, v):
    """Rotate 3-vector v by quaternion q (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class LifecycleImu(rclpy.lifecycle.Node):
    """Simulated IMU: differentiates /odom into a sensor_msgs/Imu."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self._pub: Optional[Publisher] = None
        self._sub: Optional[Subscription] = None
        self._is_active = False

        # Finite-difference state.
        self._t_prev: Optional[float] = None
        self._pos_prev: Optional[np.ndarray] = None
        self._q_prev = None
        self._v_prev: Optional[np.ndarray] = None
        self._accel_ema = np.zeros(3)
        self._gyro_ema = np.zeros(3)
        self._measured = None              # odom twist measured? (logged on change)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('gravity',          9.81),        # m/s^2 (Earth, as Unreal; 1.62 for the Moon)
                ('imu_frame_id',     'base_link'),
                ('odom_topic',       'odom'),
                ('imu_topic',        'imu/data'),
                ('smoothing',        0.3),         # EMA alpha 0..1 (0 = raw, →1 = heavy)
                ('accel_noise_std',  0.0),         # m/s^2, added to accelerometer
                ('gyro_noise_std',   0.0),         # rad/s, added to gyro
                ('max_dt',           0.5),         # s; a larger pose gap resets the diff
            ])

    # ── Lifecycle ────────────────────────────────────────────────────────────
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.gravity     = float(self.get_parameter('gravity').value)
        self.frame_id    = self.get_parameter('imu_frame_id').value
        self.odom_topic  = self.get_parameter('odom_topic').value
        self.imu_topic   = self.get_parameter('imu_topic').value
        self.alpha       = float(min(max(self.get_parameter('smoothing').value, 0.0), 0.99))
        self.accel_noise = float(self.get_parameter('accel_noise_std').value)
        self.gyro_noise  = float(self.get_parameter('gyro_noise_std').value)
        self.max_dt      = float(self.get_parameter('max_dt').value)

        self._pub = self.create_lifecycle_publisher(Imu, self.imu_topic, qos_profile_sensor_data)
        self._sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"IMU sim CONFIGURED: '{self.odom_topic}' -> '{self.imu_topic}' | "
            f"gravity={self.gravity} m/s^2 frame='{self.frame_id}' "
            f"smoothing={self.alpha} noise(a={self.accel_noise}, g={self.gyro_noise}).")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("IMU sim ACTIVE.")
        self._is_active = True
        self._reset()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("IMU sim DEACTIVATED.")
        self._is_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def _teardown(self) -> None:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None

    def _reset(self) -> None:
        self._t_prev = None
        self._pos_prev = None
        self._q_prev = None
        self._v_prev = None
        self._accel_ema = np.zeros(3)
        self._gyro_ema = np.zeros(3)

    # ── Core ─────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        if not self._is_active or self._pub is None:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = np.array([msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z])
        o = msg.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)

        # A twist with a non-zero covariance was measured by Unreal's physics
        # (the rover node fills it when the telemetry carries velocities): the
        # gyro is read from it, and the velocity is differentiated only once.
        tw = msg.twist.twist
        measured = any(msg.twist.covariance)
        if measured != self._measured:
            self._measured = measured
            self.get_logger().info(
                'IMU from the velocities of Unreal\'s physics (odom twist).' if measured else
                'IMU from the odom pose (no velocities in the telemetry): differentiated twice.')
        v_measured = (np.array(q_rot(q, (tw.linear.x, tw.linear.y, tw.linear.z)))
                      if measured else None)

        gyro = np.array([tw.angular.x, tw.angular.y, tw.angular.z]) if measured else np.zeros(3)
        accel_body = None

        if self._t_prev is not None:
            dt = t - self._t_prev
            if dt <= 1e-4 or dt > self.max_dt:
                # Duplicate stamp or a gap/teleport (Unreal Play-Stop): drop the
                # differentiator so we never emit a giant spurious spike.
                self._reset()
            else:
                if not measured:
                    # Angular velocity: body-frame rates from the quaternion delta.
                    dq = q_mul(q_conj(self._q_prev), q)
                    if dq[3] < 0.0:
                        dq = tuple(-c for c in dq)          # shortest arc
                    gyro = np.array([2.0 * dq[0] / dt, 2.0 * dq[1] / dt, 2.0 * dq[2] / dt])

                # World-frame velocity, then acceleration.
                v = v_measured if measured else (p - self._pos_prev) / dt
                if self._v_prev is not None:
                    a_world = (v - self._v_prev) / dt
                    # Accelerometer measures specific force = proper accel − gravity.
                    g_world = np.array([0.0, 0.0, -self.gravity])
                    f_world = a_world - g_world
                    accel_body = np.array(q_rot(q_conj(q), (f_world[0], f_world[1], f_world[2])))
                self._v_prev = v
        if measured:
            self._v_prev = v_measured          # valid from the first sample, even after a gap

        self._t_prev = t
        self._pos_prev = p
        self._q_prev = q

        # Until two motion samples exist, report gravity only (a stationary IMU).
        if accel_body is None:
            accel_body = np.array(q_rot(q_conj(q), (0.0, 0.0, self.gravity)))

        # EMA smoothing (double-differentiated position is noisy).
        self._gyro_ema = self.alpha * self._gyro_ema + (1.0 - self.alpha) * gyro
        self._accel_ema = self.alpha * self._accel_ema + (1.0 - self.alpha) * accel_body

        g_out = self._gyro_ema.copy()
        a_out = self._accel_ema.copy()
        if self.gyro_noise > 0.0:
            g_out += np.random.normal(0.0, self.gyro_noise, 3)
        if self.accel_noise > 0.0:
            a_out += np.random.normal(0.0, self.accel_noise, 3)

        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self.frame_id
        imu.orientation = o
        imu.angular_velocity = Vector3(x=float(g_out[0]), y=float(g_out[1]), z=float(g_out[2]))
        imu.linear_acceleration = Vector3(x=float(a_out[0]), y=float(a_out[1]), z=float(a_out[2]))

        # Diagonal covariances. Orientation roll/pitch are only as good as odom's
        # (yaw-only until the rover publishes real tilt), so they are marked loose.
        imu.orientation_covariance = [0.05, 0.0, 0.0,
                                      0.0, 0.05, 0.0,
                                      0.0, 0.0, 0.01]
        av = max(self.gyro_noise ** 2, 1e-4)
        imu.angular_velocity_covariance = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        la = max(self.accel_noise ** 2, 1e-2)
        imu.linear_acceleration_covariance = [la, 0.0, 0.0, 0.0, la, 0.0, 0.0, 0.0, la]
        self._pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleImu('lc_component_imu')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
