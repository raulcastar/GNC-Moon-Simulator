"""
ROS 2 lifecycle node: rover control and odometry for the Unreal rover.

A bidirectional bridge between ROS 2 and the rover Blueprint in Unreal Engine.
geometry_msgs/Twist commands on cmd_vel are ramped to the acceleration limits
and sent as skid-steer wheel inputs; a background thread reads Unreal's pose
telemetry and publishes it as nav_msgs/Odometry (REP-103: metres, right-handed,
quaternions) and TF odom -> base_link.

Transport: one bidirectional TCP connection (the default, port 3003: Unreal
dials, this node listens), or the legacy pair of UDP sockets.
"""

from typing import Optional
import json
import socket
import threading
import time
import math
import re
from decimal import Decimal, InvalidOperation

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription

from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

import warnings
warnings.filterwarnings("ignore")


# ── Quaternion helpers (tuples, x, y, z, w) ─────────────────────────────────
def _q_from_euler(roll_deg, pitch_deg, yaw_deg):
    """Degrees, ZYX aerospace (roll about x, pitch about y, yaw about z)."""
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _q_inv(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _q_unrotate(q, v):
    """The world vector v expressed in the frame q (rotates v by q's inverse)."""
    x, y, z, w = _q_mul(_q_mul(_q_inv(q), (v[0], v[1], v[2], 0.0)), q)
    return (x, y, z)


def pose_from_telemetry(transform: dict):
    """
    The telemetry's pose in the ROS odom frame: (x, y, z) metres and the
    quaternion (x, y, z, w) of base_link.

    The telemetry is the world transform of the Blueprint's base_link component
    (centred between the wheels at axle height, its X forward and Z up), in
    Unreal's world: centimetres, left-handed, rotation as roll/pitch/yaw degrees.
    Positions map as (-x, y, z) / 100. The frame's forward and up axes (Unreal's
    rotation matrix) map the same way; the mirror makes Unreal's left-handed
    axes right-handed, and left = up x forward completes base_link (REP-103:
    x forward, y left, z up). No offsets: whatever the rover's attitude, this
    is its heading, roll and pitch.
    """
    position = (-transform['x'] / 100.0, transform['y'] / 100.0, transform['z'] / 100.0)
    r, p, y = (math.radians(transform[k]) for k in ('roll', 'pitch', 'yaw'))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    fwd_ue = (cp * cy, cp * sy, sp)                                  # FRotationMatrix X axis
    up_ue = (-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp)   # Z axis
    fwd = (-fwd_ue[0], fwd_ue[1], fwd_ue[2])
    up = (-up_ue[0], up_ue[1], up_ue[2])
    left = (up[1] * fwd[2] - up[2] * fwd[1], up[2] * fwd[0] - up[0] * fwd[2], up[0] * fwd[1] - up[1] * fwd[0])
    return position, _q_from_axes(fwd, left, up)


def _q_from_axes(x, y, z):
    """Quaternion (x, y, z, w) of the rotation whose matrix columns are x, y, z."""
    m00, m10, m20 = x
    m01, m11, m21 = y
    m02, m12, m22 = z
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        q = ((m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, 0.25 / s)
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


# Optional telemetry fields: the physics body's velocity, world frame, Unreal
# axes. vx, vy, vz in cm/s (Get Physics Linear Velocity); wx, wy, wz in rad/s
# (Get Physics Angular Velocity In Radians).
_TWIST_KEYS = ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')


def twist_from_telemetry(transform: dict, orientation):
    """
    Body-frame (base_link) linear and angular velocity in ROS units from the
    telemetry's world velocities, with orientation the odom attitude (x, y, z,
    w). The world axes map as the position does (x negated). Unreal is
    left-handed, so an angular velocity also flips sign with that mirror:
    (wx, -wy, -wz). A yaw rate turning Unreal's X towards +Y is a negative
    heading rate: the mirror that maps Unreal's x to -x reverses rotation senses.
    """
    v_world = (-transform['vx'] / 100.0, transform['vy'] / 100.0, transform['vz'] / 100.0)
    w_world = (transform['wx'], -transform['wy'], -transform['wz'])
    return _q_unrotate(orientation, v_world), _q_unrotate(orientation, w_world)


# Optional telemetry field: the simulation time Unreal measured the pose at, in
# ns since the Unix epoch, as text or an integer (UnrealSimClock's Get Sim Time
# Ns String in the rover Blueprint). The clock every sensor plugin stamps with.
_STAMP_KEY = 't_ns'


def stamp_ns_from_telemetry(transform: dict) -> Optional[int]:
    """
    The telemetry's capture time (ns since the Unix epoch), or None when it has
    none or it is unreadable. Without it, odometry is stamped on arrival.
    """
    value = transform.get(_STAMP_KEY)
    if value is None or isinstance(value, bool):
        return None
    try:
        # Decimal, not float: 19 digits are more than a double holds exactly.
        ns = int(Decimal(str(value).strip()).to_integral_value())
    except (InvalidOperation, ValueError):
        return None
    return ns if ns > 0 else None


# Unreal's Json Blueprint Utilities 'Get Field' reports an exact 0 (0, 0.0,
# -0.0) as not found, so the Blueprint would ignore "lights at 0 %". Zero goes
# out as this instead: 1e-9 x 20000 cd is dark.
HEADLIGHT_ZERO_ON_WIRE = 1e-9


def headlight_wire_value(level: float) -> float:
    """The brightness as sent to Unreal: 4 decimals, and never an exact zero."""
    return round(level, 4) or HEADLIGHT_ZERO_ON_WIRE


def headlight_level(value) -> Optional[float]:
    """
    A headlights_intensity command as the fraction sent to Unreal: clamped to
    0..1 (1 = the Blueprint's own light intensity), or None when it is not a
    finite number (ignored).
    """
    try:
        level = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(level):
        return None
    return min(max(level, 0.0), 1.0)


class LifecycleRoverController(rclpy.lifecycle.Node):
    """
    The link with Unreal (TCP, or legacy UDP): wheel commands out, telemetry in
    on a background thread, published in REP-103 units and frames.
    """

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # ROS 2 interfaces
        self._sub_cmd_vel: Optional[Subscription] = None
        self._sub_headlights: Optional[Subscription] = None
        self._sub_intensity: Optional[Subscription] = None
        self._pub_odom:    Optional[Publisher] = None
        self._tf: Optional[TransformBroadcaster] = None    # odom -> base_link

        # Rover state
        self.headlights_on: bool = False
        self.headlights_intensity: float = 1.0      # fraction of the Blueprint's intensity
        self._last_udp_telemetry: float = float("-inf")   # Detects a new Play session (UDP)
        # Last wheel command, so a headlights toggle can be resent in the full
        # drive-packet shape Unreal parses without disturbing motion.
        self._last_wheels: dict = {"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0}
        # Acceleration limiter state: a velocity step (e.g. a manual
        # `ros2 topic pub`) would otherwise jump the wheel speeds and can flip
        # the rover, so v and w are ramped.
        self._v_cmd: float = 0.0
        self._w_cmd: float = 0.0
        self._last_cmd_t: Optional[float] = None
        # Latest commanded twist; _ramp_tick keeps ramping toward it between messages.
        self._v_tgt: float = 0.0
        self._w_tgt: float = 0.0
        self._ramp_timer = None
        self._warned_no_stamp = False

        # UDP transport: send and receive sockets, and the receive thread.
        self.udp_send_socket: Optional[socket.socket] = None

        self.udp_recv_socket: Optional[socket.socket] = None
        self._listen_thread:  Optional[threading.Thread] = None
        self._keep_listening: bool = False

        # TCP transport: one bidirectional connection, so no address to guess.
        self.transport: str = "udp"
        self.tcp_role: str = "server"
        self.tcp_socket: Optional[socket.socket] = None
        self.tcp_server: Optional[socket.socket] = None
        self.tcp_port: int = 3003
        self._decoder = json.JSONDecoder()

        # Runtime configuration defaults
        self.target_ip: str = "127.0.0.1"
        self.target_port: int = 3002
        self.listen_ip: str = "0.0.0.0"
        self.listen_port: int = 3001
        self.rover_width: float = 1.0

        # When sending_ip is a wildcard, Unreal's address is learned from the
        # source of its telemetry instead. See _resolve_target.
        self._autodetect_target: bool = False
        self._target_learned: bool = False

        self.declare_parameters(
            namespace='',
            parameters=[
                ('sending_ip',   rclpy.Parameter.Type.STRING),
                ('sending_port', rclpy.Parameter.Type.INTEGER),
                ('listen_ip',    rclpy.Parameter.Type.STRING),
                ('listen_port',  rclpy.Parameter.Type.INTEGER),
                ('rover_width',  rclpy.Parameter.Type.DOUBLE),
                ('headlights_on', rclpy.Parameter.Type.BOOL),
                ('transport',    rclpy.Parameter.Type.STRING),
                ('tcp_role',     rclpy.Parameter.Type.STRING),
                ('tcp_port',     rclpy.Parameter.Type.INTEGER),
            ])

        # Acceleration limits: a step command is ramped to these rates, so a hard
        # launch cannot tip the rover. The planner ramps on its own; these are
        # the safety net for any other command source.
        self.declare_parameter('max_linear_accel', 0.6)    # m/s^2
        self.declare_parameter('max_angular_accel', 1.5)   # rad/s^2
        self.declare_parameter('max_linear_decel', 0.4)    # m/s^2
        self.declare_parameter('max_angular_decel', 3.0)   # rad/s^2
        self.declare_parameter('wheel_speed_scale', 1.0)   # m/s -> Unreal wheel units
        # Broadcast odom -> base_link on /tf with every odometry message.
        self.declare_parameter('publish_tf', True)
        # Headlight brightness at start, 0..1 of the Blueprint's intensity; the
        # headlights_intensity topic changes it.
        self.declare_parameter('headlights_intensity', 1.0)
        self.wheel_speed_scale: float = 1.0
        self.max_linear_accel = self.max_angular_accel = 0.0
        self.max_linear_decel = self.max_angular_decel = 0.0
        # The drive calibration applies live (the dashboard's rover switch sets it):
        # reconfiguring would drop Unreal's TCP link, which Unreal does not redial.
        self.add_on_set_parameters_callback(self._on_parameters)

    # Parameters applied as soon as they are set: name -> (attribute, factor).
    _LIVE = {'wheel_speed_scale': ('wheel_speed_scale', 1.0), 'rover_width': ('rover_width', 0.01),
             'max_linear_accel': ('max_linear_accel', 1.0), 'max_angular_accel': ('max_angular_accel', 1.0),
             'max_linear_decel': ('max_linear_decel', 1.0), 'max_angular_decel': ('max_angular_decel', 1.0)}

    def _on_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in self._LIVE:
                attr, factor = self._LIVE[p.name]
                try:
                    value = float(p.value) * factor
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason=f'{p.name} must be a number')
                if value <= 0.0:
                    return SetParametersResult(successful=False, reason=f'{p.name} must be positive')
                setattr(self, attr, value)
                self.get_logger().info(f'{p.name} set to {p.value} (live).')
        return SetParametersResult(successful=True)


    # =========================================================================
    # Kinematics
    # =========================================================================

    @staticmethod
    def _slew(cur: float, target: float, up_step: float, down_step: float) -> float:
        """
        Step cur toward target by at most up_step when speeding up, or
        down_step when slowing down, stopping or reversing.
        """
        speeding_up = abs(target) > abs(cur) and (cur == 0.0 or (cur > 0) == (target > 0))
        step = up_step if speeding_up else down_step
        if abs(target - cur) <= step:
            return target       # exact, so _ramp_tick can detect completion
        return cur + step if target > cur else cur - step

    def _now_s(self) -> float:
        """The node's clock (simulation time with use_sim_time), in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _ramp_step(self) -> None:
        """
        Advance (v, w) toward the latest target by one rate-limited step and send it.
        On the node's clock, so the limits are per simulated second: the planner
        and the physics run on simulation time, which falls behind the wall
        clock when Unreal renders below its fixed frame rate.
        """
        now = self._now_s()
        dt = 0.1 if self._last_cmd_t is None else max(0.0, min(now - self._last_cmd_t, 0.2))
        self._last_cmd_t = now

        self._v_cmd = self._slew(self._v_cmd, self._v_tgt,
                                 self.max_linear_accel * dt, self.max_linear_decel * dt)
        self._w_cmd = self._slew(self._w_cmd, self._w_tgt,
                                 self.max_angular_accel * dt, self.max_angular_decel * dt)

        wheels = self.twist_to_wheels(self._v_cmd, self._w_cmd)
        self._last_wheels = {k: wheels[k] for k in ("FL", "BL", "FR", "BR")}
        self._send(wheels, "cmd_vel")

    def _ramp_tick(self) -> None:
        """Complete an unfinished ramp when cmd_vel is not arriving (e.g. a single stop)."""
        if self._v_cmd == self._v_tgt and self._w_cmd == self._w_tgt:
            return
        if self._last_cmd_t is not None and self._now_s() - self._last_cmd_t < 0.05:
            return      # cmd_vel is already stepping the ramp
        self._ramp_step()

    def twist_to_wheels(self, v: float, w: float) -> dict:
        """
        Skid-steer inverse kinematics: v [m/s], w [rad/s] -> Unreal wheel inputs. rover_width is the effective
        (slip-calibrated) track in metres; wheel_speed_scale converts m/s to
        wheel input units.
        """
        k = self.wheel_speed_scale
        vel_left  = k * (v - (w * self.rover_width / 2.0))
        vel_right = k * (v + (w * self.rover_width / 2.0))

        # Negative input = forward in the Blueprint.
        return {
            "FL": -vel_left,
            "BL": -vel_left,
            "FR": -vel_right,
            "BR": -vel_right,
            **self._lights(),
        }

    def _lights(self) -> dict:
        """The light fields every packet carries: Unreal reads them from the drive packet."""
        return {"headlights_on": self.headlights_on,
                "headlights_intensity": headlight_wire_value(self.headlights_intensity)}

    # =========================================================================
    # Sending
    # =========================================================================

    def _resolve_target(self, sending_ip: str) -> None:
        """
        Decide where drive commands are sent.

        A wildcard address is a valid bind target but not a valid destination:
        the kernel rewrites a datagram addressed to 0.0.0.0 into one addressed to
        127.0.0.1, so it stops on this container's own loopback and never reaches
        Unreal on the Windows host. WSL2's localhost relay hides this for TCP
        (it forwards TCP to Windows listeners) but does not carry UDP.

        So a wildcard means autodetect: Unreal's address is taken from the source
        of the telemetry it sends us, which is by definition reachable. An
        explicit address is honoured as given.
        """
        self._autodetect_target = sending_ip in ('', '0.0.0.0', '::')
        self._target_learned = False

        if self._autodetect_target:
            self.target_ip = "127.0.0.1"
            self.get_logger().info(
                f"sending_ip is '{sending_ip}', which is a bind address, not a destination. "
                f"Unreal's address will be taken from its telemetry on port {self.listen_port}; "
                f"until the first packet arrives, commands go to 127.0.0.1.")
        else:
            self.target_ip = sending_ip

    def _send_light_state(self) -> None:
        """
        A new Play session (Unreal (re)connected): the wheels start stopped,
        and Unreal is told the headlight state (it only learns it from drive
        packets, and an idle rover sends none, so a new Play session would
        otherwise keep its own default).

        The ramp is reset first. It runs on simulation time, which stands still
        between Play sessions, so a ramp cut short by Stop would otherwise carry
        on in the next session and spin the wheels while the rover lands at
        spawn, which can roll it over.
        """
        self._v_cmd = self._w_cmd = 0.0
        self._v_tgt = self._w_tgt = 0.0
        self._last_cmd_t = None
        self._last_wheels = {"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0}
        self._send({"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0, **self._lights()}, "headlight state")
        self.get_logger().info(
            f"Unreal connected: headlights {'ON' if self.headlights_on else 'OFF'}, "
            f"intensity {self.headlights_intensity:.0%}.")

    def _send(self, payload: dict, what: str) -> None:
        """Dispatch one JSON command to Unreal over the configured transport."""
        if self.transport == 'tcp':
            if self.tcp_socket is None:
                self.get_logger().warn(
                    f"Dropped {what}: no TCP connection to Unreal.",
                    throttle_duration_sec=10.0)
                return
            try:
                # Newline-delimited so Unreal can frame the stream.
                self.tcp_socket.sendall(json.dumps(payload).encode('utf-8') + b'\n')
            except OSError as e:
                self.get_logger().error(f"Failed to transmit {what} over TCP: {e}")
                self._close_tcp()
            return

        if not self.udp_send_socket:
            return
        try:
            self.udp_send_socket.sendto(
                json.dumps(payload).encode('utf-8'), (self.target_ip, self.target_port))
        except Exception as e:
            self.get_logger().error(f"Failed to transmit {what} UDP packet: {e}")
            return

        if self._autodetect_target and not self._target_learned:
            self.get_logger().warn(
                f"Sent {what} to 127.0.0.1:{self.target_port} — no telemetry has arrived yet, "
                f"so Unreal's address is still unknown. If the rover does not move, Unreal is "
                f"not sending on port {self.listen_port}; set sending_ip to the host's address.",
                throttle_duration_sec=10.0)

    # =========================================================================
    # Callbacks and the receive threads
    # =========================================================================

    def headlights_callback(self, msg: Bool) -> None:
        """Headlights on/off from headlights_cmd."""
        self.headlights_on = msg.data
        # Unreal reads the light state from the drive packet (FL/BL/FR/BR +
        # headlights_on), not from a standalone {"headlights_on": ...}, so send
        # the full shape. Reuse the last wheel command so the toggle alone does
        # not change motion (zeros when the rover is idle).
        self._send({**self._last_wheels, **self._lights()}, "headlights")
        self.get_logger().info(f"Headlights toggled: {'ON' if self.headlights_on else 'OFF'}")

    def headlights_intensity_callback(self, msg: Float32) -> None:
        """
        Headlight brightness, 0..1 of the Blueprint's intensity (clamped); kept
        and sent with every packet, so it survives an off/on and a new Play
        session. Like the toggle, it rides a full drive packet (the last wheel
        command), so motion is not disturbed.
        """
        level = headlight_level(msg.data)
        if level is None:
            self.get_logger().warn(f"Headlight intensity {msg.data!r} ignored: not a number.",
                                   throttle_duration_sec=10.0)
            return
        if level == self.headlights_intensity:
            return
        self.headlights_intensity = level
        self._send({**self._last_wheels, **self._lights()}, "headlight intensity")
        self.get_logger().info(f"Headlight intensity: {level:.0%}.")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """A cmd_vel twist becomes the ramp's target; _ramp_step applies the limits."""
        self._v_tgt = msg.linear.x
        self._w_tgt = msg.angular.z
        self._ramp_step()

    def _udp_receive_loop(self) -> None:
        """Background thread: receives UDP telemetry and publishes it."""
        self.get_logger().info(f"Telemetry listener thread initialized on port {self.listen_port}.")

        while self._keep_listening:
            try:
                # The timeout lets the loop check _keep_listening.
                self.udp_recv_socket.settimeout(0.5)
                data, addr = self.udp_recv_socket.recvfrom(1024)

                # Telemetry proves where Unreal is and that the route back is
                # open. Its source port is ephemeral, so keep the configured one.
                if addr[0] != self.target_ip:
                    if self._autodetect_target:
                        self.get_logger().info(
                            f"Unreal is at {addr[0]}; drive commands now go to "
                            f"{addr[0]}:{self.target_port}.")
                        self.target_ip = addr[0]
                    else:
                        self.get_logger().warn(
                            f"Telemetry arrives from {addr[0]} but sending_ip is "
                            f"{self.target_ip}. Commands are going somewhere else than "
                            f"Unreal is.", throttle_duration_sec=30.0)
                self._target_learned = True
                # First telemetry after a pause: a new Play session.
                now = time.monotonic()
                if now - self._last_udp_telemetry > 2.0:
                    self._send_light_state()
                self._last_udp_telemetry = now
                self._publish_telemetry(data)

            except socket.timeout:
                continue
            except Exception as e:
                if self._keep_listening:
                    self.get_logger().error(f"Telemetry thread fault: {e}")

        self.get_logger().info("Telemetry listener thread cleanly terminated.")

    def _drain_frames(self, buf: str) -> str:
        """
        Pull every complete JSON object out of a TCP byte stream.

        TCP has no message boundaries and Unreal delimits nothing: objects arrive
        bare, and may be split across reads or coalesced into one. So scan for
        objects rather than for a separator, publishing each and keeping whatever
        trailing fragment is still incomplete. Whitespace and NULs between objects
        are skipped; a newline delimiter, if present, costs nothing.
        """
        while True:
            buf = buf.lstrip(' \r\n\t\x00')
            if not buf:
                return ''
            try:
                transform, end = self._decoder.raw_decode(buf)
            except ValueError:
                # An incomplete object: keep it and wait for the rest. A stream
                # that never parses would grow without bound, so cap it.
                if len(buf) > 65536:
                    self.get_logger().warn(
                        "Discarding 64 KB of unparseable telemetry; is Unreal "
                        "sending JSON?", throttle_duration_sec=30.0)
                    return ''
                return buf
            self._publish_transform(transform)
            buf = buf[end:]

    def _publish_telemetry(self, data: bytes) -> None:
        """Turn one raw UDP telemetry datagram into Odometry."""
        message = self._clean(data)
        if not message:
            return
        try:
            self._publish_transform(json.loads(message))
        except json.JSONDecodeError:
            self.get_logger().warn(f"Received non-JSON data: {repr(message)}")

    @staticmethod
    def _clean(data: bytes) -> str:
        """Decode a payload and repair Unreal's locale-formatted numbers."""
        text = data.decode('utf-8', 'replace').strip().rstrip('\x00')
        # Unreal writes thousands separators; strip the comma between two digits.
        return re.sub(r'(\d),(\d)', r'\1\2', text)

    def _publish_transform(self, transform: dict) -> None:
        """Publish one Unreal pose as Odometry. Transport agnostic."""
        odom_msg = Odometry()
        # The time Unreal measured the pose at, when the telemetry carries it;
        # else its arrival (rounded to the last /clock tick, so consecutive
        # poses can share a stamp or skip one).
        stamp_ns = stamp_ns_from_telemetry(transform)
        if stamp_ns is not None:
            odom_msg.header.stamp.sec, odom_msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        else:
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            if not self._warned_no_stamp:
                self._warned_no_stamp = True
                self.get_logger().info(
                    f"Telemetry has no '{_STAMP_KEY}' field: odometry is stamped on arrival. "
                    f"Add it in the rover Blueprint (UnrealSimClock: Get Sim Time Ns String) "
                    f"to stamp it with the time Unreal measured the pose.")
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # base_link's pose (see pose_from_telemetry).
        position, q = pose_from_telemetry(transform)
        p = odom_msg.pose.pose.position
        p.x, p.y, p.z = position
        o = odom_msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q

        # Velocities, when the Blueprint sends them: straight from the physics
        # body, so the IMU differentiates them once instead of the pose twice.
        # A non-zero twist covariance marks the twist as measured; without the
        # fields it stays all zero (unknown).
        if all(k in transform for k in _TWIST_KEYS):
            o = odom_msg.pose.pose.orientation
            lin, ang = twist_from_telemetry(transform, (o.x, o.y, o.z, o.w))
            t = odom_msg.twist.twist
            t.linear.x, t.linear.y, t.linear.z = lin
            t.angular.x, t.angular.y, t.angular.z = ang
            odom_msg.twist.covariance = [1e-4 if i % 7 == 0 else 0.0 for i in range(36)]

        self._pub_odom.publish(odom_msg)
        self._broadcast_tf(odom_msg)

    def _broadcast_tf(self, odom_msg: Odometry) -> None:
        """odom -> base_link, the dynamic link of the TF tree (the rest: component_robot_description)."""
        if self._tf is None:
            return
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        p = odom_msg.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        self._tf.sendTransform(t)

    # =========================================================================
    # TCP transport
    # =========================================================================

    def _tcp_accept(self) -> None:
        """
        Wait for Unreal to connect. Used when tcp_role is 'server'.

        Unreal's TCP component dials out, so this side listens. Whoever connects
        owns the link until it drops; the listening socket stays up to catch the
        reconnection after a Play/Stop cycle.
        """
        if self.tcp_server is None:
            self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.tcp_server.bind((self.listen_ip, self.tcp_port))
            self.tcp_server.listen(1)
            self.tcp_server.settimeout(0.5)
            self.get_logger().info(
                f"Listening for Unreal on TCP {self.listen_ip}:{self.tcp_port}.")
        try:
            conn, addr = self.tcp_server.accept()
            conn.settimeout(0.5)
            self.tcp_socket = conn
            self.get_logger().info(f"Unreal connected from {addr[0]}:{addr[1]}.")
            self._send_light_state()
        except socket.timeout:
            pass

    def _tcp_connect(self) -> None:
        """Dial Unreal. Used when tcp_role is 'client' (Unreal listens)."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((self.target_ip, self.tcp_port))
            s.settimeout(0.5)
            self.tcp_socket = s
            self.get_logger().info(
                f"TCP connected to Unreal at {self.target_ip}:{self.tcp_port}.")
            self._send_light_state()
        except OSError as e:
            self.get_logger().warn(
                f"TCP connect to {self.target_ip}:{self.tcp_port} failed ({e}). "
                f"Is the Unreal TCP component listening?",
                throttle_duration_sec=10.0)
            time.sleep(2.0)

    def _tcp_loop(self) -> None:
        """
        Hold one TCP link to Unreal, carrying commands out and telemetry back.

        TCP needs no send address: replies come back on the established
        connection, so neither side has to know the other's IP (the failure the
        UDP path suffers under WSL2 mirrored networking). Commands go out as
        newline-delimited JSON; telemetry is parsed object by object
        (_drain_frames), since Unreal sends no delimiter.

        tcp_role decides who dials: 'server' (we listen, Unreal connects) or
        'client' (we connect to a listening Unreal).
        """
        buf = ''
        while self._keep_listening:
            if self.tcp_socket is None:
                buf = ''
                if self.tcp_role == 'server':
                    self._tcp_accept()
                else:
                    self._tcp_connect()
                continue

            try:
                chunk = self.tcp_socket.recv(4096)
                if not chunk:
                    raise ConnectionError("Unreal closed the connection")
                # Repair the whole buffer, not the chunk: a number like 1,205 can
                # straddle a read boundary.
                buf = self._drain_frames(
                    self._clean(buf.encode() + chunk))
            except socket.timeout:
                continue
            except OSError as e:
                if self._keep_listening:
                    self.get_logger().warn(
                        f"TCP link lost ({e}); waiting for Unreal to come back."
                        if self.tcp_role == 'server' else
                        f"TCP link lost ({e}); reconnecting.")
                self._close_tcp()
                time.sleep(1.0)

        self._close_tcp()
        self._close_tcp_server()
        self.get_logger().info("TCP telemetry thread cleanly terminated.")

    def _close_tcp(self) -> None:
        """Drop the link, keeping the listening socket so Unreal can reconnect."""
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except OSError:
                pass
            self.tcp_socket = None

    def _close_tcp_server(self) -> None:
        if self.tcp_server:
            try:
                self.tcp_server.close()
            except OSError:
                pass
            self.tcp_server = None

    # =========================================================================
    # Lifecycle transitions
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle state transition: CONFIGURED.")

        self.target_port = self.get_parameter('sending_port').value
        self.listen_ip   = self.get_parameter('listen_ip').value
        self.listen_port = self.get_parameter('listen_port').value
        self.rover_width = self.get_parameter('rover_width').value / 100.0  # cm -> m
        self.wheel_speed_scale = self.get_parameter('wheel_speed_scale').value
        self.headlights_on = self.get_parameter('headlights_on').value
        self.headlights_intensity = headlight_level(self.get_parameter('headlights_intensity').value)
        if self.headlights_intensity is None:
            self.headlights_intensity = 1.0
        self.max_linear_accel  = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_linear_decel  = self.get_parameter('max_linear_decel').value
        self.max_angular_decel = self.get_parameter('max_angular_decel').value
        self.transport   = self.get_parameter('transport').value.strip().lower()
        self.tcp_role    = self.get_parameter('tcp_role').value.strip().lower()
        self.tcp_port    = self.get_parameter('tcp_port').value
        self._resolve_target(self.get_parameter('sending_ip').value)

        if self.transport not in ('udp', 'tcp'):
            self.get_logger().error(
                f"transport must be 'udp' or 'tcp', not '{self.transport}'.")
            return TransitionCallbackReturn.FAILURE
        if self.tcp_role not in ('server', 'client'):
            self.get_logger().error(
                f"tcp_role must be 'server' or 'client', not '{self.tcp_role}'.")
            return TransitionCallbackReturn.FAILURE

        if self.transport == 'tcp':
            # One connection carries both directions. The link is established by
            # the thread started on activate.
            where = (f"listening on {self.listen_ip}:{self.tcp_port} for Unreal"
                     if self.tcp_role == 'server'
                     else f"dialling Unreal at {self.target_ip}:{self.tcp_port}")
            self.get_logger().info(
                f"Transport: TCP, {where}. Telemetry returns on the same "
                f"connection, so no send address is needed.")
        else:
            # Send socket (unbound).
            try:
                self.udp_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except socket.error as e:
                self.get_logger().error(f"Failed to allocate UDP transmission socket: {e}")
                return TransitionCallbackReturn.FAILURE

            # Receive socket, bound to listen_port.
            try:
                self.udp_recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_recv_socket.bind((self.listen_ip, self.listen_port))
            except socket.error as e:
                self.get_logger().error(
                    f"Failed to bind UDP reception socket on port {self.listen_port}: {e}")
                return TransitionCallbackReturn.FAILURE

        # Start stopped, with the configured lights.
        self._send({"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0, **self._lights()}, "initial stop")

        self._pub_odom    = self.create_lifecycle_publisher(Odometry, 'odom', 10)
        if self.get_parameter('publish_tf').value:
            self._tf = TransformBroadcaster(self)
        self._sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self._ramp_timer  = self.create_timer(0.05, self._ramp_tick)

        self._sub_headlights = self.create_subscription(Bool, 'headlights_cmd', self.headlights_callback, 10)
        self._sub_intensity = self.create_subscription(
            Float32, 'headlights_intensity', self.headlights_intensity_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle state transition: ACTIVE.")

        # The receive thread (TCP link or UDP telemetry).
        self._keep_listening = True
        loop = self._tcp_loop if self.transport == 'tcp' else self._udp_receive_loop
        self._listen_thread = threading.Thread(target=loop, daemon=True)
        self._listen_thread.start()

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle state transition: DEACTIVATED.")

        # Stop the rover.
        self._send({"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0,
                    "headlights_on": False,
                    "headlights_intensity": headlight_wire_value(self.headlights_intensity)},
                   "stop")
        # The zeros bypassed the ramp: reset it so the next command ramps from
        # a standstill.
        self._v_cmd = self._w_cmd = 0.0
        self._v_tgt = self._w_tgt = 0.0
        self._last_cmd_t = None

        # Stop the receive thread (waits at most 1 s).
        self._keep_listening = False
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=1.0)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle state transition: CLEANING UP.')

        if self._ramp_timer:
            self.destroy_timer(self._ramp_timer)
            self._ramp_timer = None

        if self._sub_cmd_vel:
            self.destroy_subscription(self._sub_cmd_vel)

        if self._sub_headlights:
            self.destroy_subscription(self._sub_headlights)

        if self._sub_intensity:
            self.destroy_subscription(self._sub_intensity)

        if self._pub_odom:
            self.destroy_publisher(self._pub_odom)

        if self.udp_send_socket:
            self.udp_send_socket.close()
            self.udp_send_socket = None

        if self.udp_recv_socket:
            self.udp_recv_socket.close()
            self.udp_recv_socket = None

        self._close_tcp()

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Lifecycle state transition: SHUTTING DOWN.')

        # Stop the receive thread, in case deactivate was skipped.
        self._keep_listening = False
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=1.0)

        if self._ramp_timer:
            self.destroy_timer(self._ramp_timer)
            self._ramp_timer = None

        if self._sub_cmd_vel:
            self.destroy_subscription(self._sub_cmd_vel)

        if self._sub_headlights:
            self.destroy_subscription(self._sub_headlights)

        if self._sub_intensity:
            self.destroy_subscription(self._sub_intensity)

        if self._pub_odom:
            self.destroy_publisher(self._pub_odom)

        if self.udp_send_socket:
            self.udp_send_socket.close()
        if self.udp_recv_socket:
            self.udp_recv_socket.close()
        self._close_tcp()

        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node  = LifecycleRoverController('lc_component_rover_leo_raph')
    executor.add_node(lc_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

if __name__ == '__main__':
    main()