"""
ROS 2 Lifecycle Node for Unreal Engine Rover Control and Odometry.

This module provides a bi-directional bridge between standard ROS 2 navigation stacks 
and the Unreal Engine simulator. It subscribes to geometry_msgs/Twist messages to 
dispatch low-latency UDP motor commands, and utilizes a dedicated background thread 
to ingest UDP telemetry, translating Unreal Engine's coordinate system into standard 
ROS 2 nav_msgs/Odometry (Meters, Right-Handed, Quaternions).
"""

from typing import Optional
import json
import socket
import threading
import time
import math
import re

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import Publisher, State, TransitionCallbackReturn
from rclpy.subscription import Subscription

# Standard ROS 2 messages for kinematics and state estimation
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import warnings
warnings.filterwarnings("ignore")


# ── Quaternion helpers (tuples, x, y, z, w), for the body-tilt recovery ──────
def _q_from_euler(roll_deg, pitch_deg, yaw_deg):
    """Degrees, ZYX aerospace — identical convention to euler_to_quaternion."""
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


def _q_to_euler(q):
    """Return (roll, pitch, yaw) in degrees, ZYX aerospace."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class LifecycleRoverController(rclpy.lifecycle.Node):
    """
    Lifecycle node handling asynchronous UDP communication with Unreal Engine.

    Manages a non-blocking transmission socket for motor commands and a dedicated 
    background thread for receiving high-frequency telemetry. Strictly enforces 
    ROS 2 REP-103 standards for units and coordinate frames.
    """

    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the LifecycleRoverController node and declare configurable parameters.

        Args:
            node_name (str): The designated name of the ROS 2 node.
        """
        super().__init__(node_name, **kwargs)
        
        # ROS 2 Interfaces
        self._sub_cmd_vel: Optional[Subscription] = None
        self._sub_headlights: Optional[Subscription] = None 
        self._pub_odom:    Optional[Publisher] = None
        
        # Rover State
        self.headlights_on: bool = False
        self._last_udp_telemetry: float = float("-inf")   # Detects a new Play session (UDP)
        # Last wheel command, so a headlights toggle can be resent in the full
        # drive-packet shape Unreal parses without disturbing motion.
        self._last_wheels: dict = {"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0}
        # Slew-rate (acceleration) limiter state. A commanded velocity STEP
        # (e.g. a manual `ros2 topic pub`, or any glitch) would otherwise become
        # an instant wheel-speed jump and flip the rover; these ramp v and w.
        self._v_cmd: float = 0.0
        self._w_cmd: float = 0.0
        self._last_cmd_t: Optional[float] = None
        # Latest commanded twist; _ramp_tick keeps ramping toward it between messages.
        self._v_tgt: float = 0.0
        self._w_tgt: float = 0.0
        self._ramp_timer = None
        
        # Network Utilities: Transmission
        self.udp_send_socket: Optional[socket.socket] = None
        
        # Network Utilities: Reception & Threading
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

        # Declare ROS 2 Parameters for dynamic configuration
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

        # Acceleration limits (declared with defaults so an older params.yaml
        # without them still starts). A step command is ramped to these rates
        # instead of jumping, which is what stops the rover tipping on a hard
        # launch. The path planner already ramps at ~0.4 m/s^2, so a slightly
        # higher cap here is a safety net that leaves normal driving untouched.
        self.declare_parameter('max_linear_accel', 0.6)    # m/s^2
        self.declare_parameter('max_angular_accel', 1.5)   # rad/s^2
        self.declare_parameter('max_linear_decel', 0.4)    # m/s^2
        self.declare_parameter('max_angular_decel', 3.0)   # rad/s^2
        self.declare_parameter('wheel_speed_scale', 1.0)   # m/s -> Unreal wheel units
        self.wheel_speed_scale: float = 1.0

        # Body-tilt recovery from Unreal telemetry (see _publish_transform).
        # publish_body_tilt: off -> pure-yaw odom (flat-ground only); on ->
        # recover real roll/pitch relative to the level baseline.
        # tilt_*_offset: the telemetry roll/pitch when the rover is LEVEL
        # (measured: roll=-90, pitch=0). tilt_*_sign: flip if a known tilt comes
        # out inverted during verification.
        self.declare_parameter('publish_body_tilt', True)
        self.declare_parameter('tilt_roll_offset', -90.0)   # deg, telemetry roll when level
        self.declare_parameter('tilt_pitch_offset', 0.0)    # deg, telemetry pitch when level
        self.declare_parameter('tilt_roll_sign', 1.0)
        self.declare_parameter('tilt_pitch_sign', -1.0)   # nose-down -> +DEM pitch

    # =========================================================================
    # Kinematics & Coordinate Transformation Helpers
    # =========================================================================

    def euler_to_quaternion(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
        """
        Translates Euler angles (degrees) into a ROS 2 compliant Quaternion.
        """
        # Convert degrees to radians to satisfy standard trigonometric functions
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        # Construct quaternion using standard aerospace sequence (Z-Y-X)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

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

    def _ramp_step(self) -> None:
        """Advance (v, w) toward the latest target by one rate-limited step and send it."""
        now = time.monotonic()
        dt = 0.1 if self._last_cmd_t is None else min(now - self._last_cmd_t, 0.2)
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
        if self._last_cmd_t is not None and time.monotonic() - self._last_cmd_t < 0.05:
            return      # cmd_vel is already stepping the ramp
        self._ramp_step()

    def twist_to_wheels(self, v: float, w: float) -> dict:
        """
        Computes Inverse Kinematics for a 4-wheel skid-steer/differential vehicle.

        v [m/s], w [rad/s] -> Unreal wheel inputs. rover_width is the effective
        (slip-calibrated) track in metres; wheel_speed_scale converts m/s to
        wheel input units.
        """
        # Compute individual side velocities based on chassis track width
        k = self.wheel_speed_scale
        vel_left  = k * (v - (w * self.rover_width / 2.0))
        vel_right = k * (v + (w * self.rover_width / 2.0))
        
        # Apply mechanical inversion (negative input = forward motion in UE blueprint)
        return {
            "FL": -vel_left,
            "BL": -vel_left,
            "FR": -vel_right,
            "BR": -vel_right,
            "headlights_on": self.headlights_on
        }

    # =========================================================================
    # UDP Transmission
    # =========================================================================

    def _resolve_target(self, sending_ip: str) -> None:
        """
        Decide where drive commands are sent.

        A wildcard address is a valid bind target but not a valid destination:
        the kernel rewrites a datagram addressed to 0.0.0.0 into one addressed to
        127.0.0.1, so it stops on this container's own loopback and never reaches
        Unreal on the Windows host. WSL2 masks this for UnrealCV, whose localhost
        relay forwards TCP to Windows listeners but does not carry UDP.

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
        Tells a freshly (re)connected Unreal the headlight state, with the wheels
        stopped: Unreal only learns it from drive packets, and an idle rover
        sends none, so a new Play session would otherwise keep its own default.
        """
        self._send({"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0,
                    "headlights_on": self.headlights_on}, "headlight state")
        self.get_logger().info(
            f"Unreal connected: headlights {'ON' if self.headlights_on else 'OFF'}.")

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
    # Core Callbacks & Background Threads
    # =========================================================================

    def headlights_callback(self, msg: Bool) -> None:
        """
        Updates the internal state of the headlights based on incoming ROS 2 commands.
        """
        self.headlights_on = msg.data
        # Unreal reads the light state from the drive packet (FL/BL/FR/BR +
        # headlights_on), not from a standalone {"headlights_on": ...}, so send
        # the full shape. Reuse the last wheel command so the toggle alone does
        # not change motion (zeros when the rover is idle).
        self._send({**self._last_wheels, "headlights_on": self.headlights_on},
                   "headlights")
        self.get_logger().info(f"Headlights toggled: {'ON' if self.headlights_on else 'OFF'}")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Processes incoming linear/angular velocity commands and dispatches them via UDP.
        """
        # Set the target; _ramp_step applies the accel/decel limits.
        self._v_tgt = msg.linear.x
        self._w_tgt = msg.angular.z
        self._ramp_step()

    def _udp_receive_loop(self) -> None:
        """
        Background thread loop for handling incoming UDP telemetry asynchronously.
        """
        self.get_logger().info(f"Telemetry listener thread initialized on port {self.listen_port}.")
        
        while self._keep_listening:
            try:
                # 0.5s timeout prevents blocking indefinitely, allowing the thread 
                # to regularly check the `_keep_listening` termination flag.
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
                # Expected timeout; loop repeats to evaluate `_keep_listening`
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
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position Translation: Unreal Engine (cm) -> ROS 2 (m)
        odom_msg.pose.pose.position.x = -transform['x'] / 100.0
        odom_msg.pose.pose.position.y = transform['y'] / 100.0
        odom_msg.pose.pose.position.z = transform['z'] / 100.0

        # Heading is reliable from Unreal's yaw (measured by driving).
        heading_deg = -transform['yaw'] - 90.0

        # Body tilt: Unreal's telemetry attitude is in a non-standard left-handed
        # frame (reads roll=-90 when the rover is level), so the numbers cannot be
        # used directly. But the rotation RELATIVE to that level baseline is
        # convention-independent and recovers true tilt (verified against 10 deg
        # and 20 deg reference tilts). q_rel = q_level^-1 * q_current, with q_level
        # = the level attitude at the current heading. The -90 roll offset lands
        # the pitch component on q_rel's yaw axis, so body_pitch = q_rel.yaw.
        # Signs/axis mapping are parameterised for one-shot verification.
        if self.publish_body_tilt:
            q_level = _q_from_euler(self.tilt_roll_offset, self.tilt_pitch_offset,
                                    transform['yaw'])
            q_cur = _q_from_euler(transform['roll'], transform['pitch'], transform['yaw'])
            rr, _resid, yy = _q_to_euler(_q_mul(_q_inv(q_level), q_cur))
            # Verified against a known nose-down tilt: the -90 telemetry roll
            # offset maps body PITCH onto q_rel.roll and body ROLL onto q_rel.yaw
            # (a nose-down 15 deg showed up on q_rel.roll and rammed the DEM).
            body_roll  = self.tilt_roll_sign  * yy
            body_pitch = self.tilt_pitch_sign * rr
            odom_msg.pose.pose.orientation = self.euler_to_quaternion(
                body_roll, body_pitch, heading_deg)
        else:
            # Pure-yaw fallback: flat-ground-correct, ignores body tilt.
            odom_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, heading_deg)

        self._pub_odom.publish(odom_msg)

    # =========================================================================
    # TCP Transport
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
        connection, so neither side has to know the other's IP. That removes the
        whole class of failure the UDP path suffers under WSL2 mirrored
        networking. Frames are newline-delimited JSON.

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
    # Lifecycle Transition Callbacks
    # =========================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node configuration.
        """
        self.get_logger().info("Lifecycle state transition: CONFIGURED.")

        self.target_port = self.get_parameter('sending_port').value
        self.listen_ip   = self.get_parameter('listen_ip').value
        self.listen_port = self.get_parameter('listen_port').value
        self.rover_width = self.get_parameter('rover_width').value / 100.0  # cm -> m
        self.wheel_speed_scale = self.get_parameter('wheel_speed_scale').value
        self.headlights_on = self.get_parameter('headlights_on').value
        self.max_linear_accel  = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_linear_decel  = self.get_parameter('max_linear_decel').value
        self.max_angular_decel = self.get_parameter('max_angular_decel').value
        self.publish_body_tilt = self.get_parameter('publish_body_tilt').value
        self.tilt_roll_offset  = self.get_parameter('tilt_roll_offset').value
        self.tilt_pitch_offset = self.get_parameter('tilt_pitch_offset').value
        self.tilt_roll_sign    = self.get_parameter('tilt_roll_sign').value
        self.tilt_pitch_sign   = self.get_parameter('tilt_pitch_sign').value
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
            # Initialize Transmission Socket (Unbound UDP Datagram)
            try:
                self.udp_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except socket.error as e:
                self.get_logger().error(f"Failed to allocate UDP transmission socket: {e}")
                return TransitionCallbackReturn.FAILURE

            # Initialize Reception Socket (Bound to targeted listening port)
            try:
                self.udp_recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_recv_socket.bind((self.listen_ip, self.listen_port))
            except socket.error as e:
                self.get_logger().error(
                    f"Failed to bind UDP reception socket on port {self.listen_port}: {e}")
                return TransitionCallbackReturn.FAILURE

        # Send initial commands
        self._send({"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0,
                    "headlights_on": self.headlights_on}, "initial stop")

        # Allocate standard pub/sub endpoints
        self._pub_odom    = self.create_lifecycle_publisher(Odometry, 'odom', 10)
        self._sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self._ramp_timer  = self.create_timer(0.05, self._ramp_tick)

        # Allocate headlights subscription
        self._sub_headlights = self.create_subscription(Bool, 'headlights_cmd', self.headlights_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node activation.
        """
        self.get_logger().info("Lifecycle state transition: ACTIVE.")
        
        # Ignite the background listening thread natively
        self._keep_listening = True
        loop = self._tcp_loop if self.transport == 'tcp' else self._udp_receive_loop
        self._listen_thread = threading.Thread(target=loop, daemon=True)
        self._listen_thread.start()

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node deactivation.
        """
        self.get_logger().info("Lifecycle state transition: DEACTIVATED.")
        
        # 1. Dispatch absolute zero velocities to enforce a hardware/simulation halt
        self._send({"FL": 0.0, "BL": 0.0, "FR": 0.0, "BR": 0.0,
                    "headlights_on": False}, "stop")
        # Sent zeros directly, bypassing the ramp — zero its state too so the
        # next command ramps from a standstill instead of a phantom velocity.
        self._v_cmd = self._w_cmd = 0.0
        self._v_tgt = self._w_tgt = 0.0
        self._last_cmd_t = None

        # 2. Assert thread termination and await its graceful exit (max 1 second block)
        self._keep_listening = False
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=1.0)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Transition callback for node cleanup.
        """
        self.get_logger().info('Lifecycle state transition: CLEANING UP.')

        if self._ramp_timer:
            self.destroy_timer(self._ramp_timer)
            self._ramp_timer = None

        if self._sub_cmd_vel:
            self.destroy_subscription(self._sub_cmd_vel)
            
        if self._sub_headlights:
            self.destroy_subscription(self._sub_headlights)
            
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
        """
        Transition callback for node shutdown.
        """
        self.get_logger().info('Lifecycle state transition: SHUTTING DOWN.')
        
        # Terminate active threads if bypassing the 'deactivate' step
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

        if self._pub_odom:
            self.destroy_publisher(self._pub_odom)

        if self.udp_send_socket:
            self.udp_send_socket.close()
        if self.udp_recv_socket:
            self.udp_recv_socket.close()
        self._close_tcp()

        return TransitionCallbackReturn.SUCCESS


def main():
    """
    Entry point for the executable. 
    """
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