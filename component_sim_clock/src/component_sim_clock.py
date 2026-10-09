#!/usr/bin/env python3
"""
ROS 2 Lifecycle Node — the simulation clock (/clock) from Unreal.

The UnrealSimClock plugin (unreal/UnrealSimClock/) keeps one clock for the
whole simulation: Unreal's game time (it stops when Play is paused, follows
time dilation), offset so it reads as nanoseconds since the Unix epoch and
never runs backwards across Play sessions. Every sensor plugin stamps its data
with it, and every world tick it sends the time here over TCP (the plugin
dials this node when Play begins, and redials every 2 s) as one record:

  'SCLK' | version u8 = 1 | 3 reserved | sim time ns u64 | frame u64   (24 bytes)

This node publishes each one as rosgraph_msgs/Clock on /clock. Nodes started
with use_sim_time: true then run on simulation time, so a message's stamp is
when the sensor captured it in the simulation, and odometry, images, depth and
lidar can be matched by stamp. When Unreal is not playing no record arrives
and simulation time stands still: timers of use_sim_time nodes pause with it.

The clock itself must not run on simulation time (use_sim_time: false here).
Times not newer than the last one published are dropped, so /clock is
monotonic.
"""
import socket
import struct
import threading
import time
from typing import Optional

import rclpy
import rclpy.lifecycle
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Time as RosTime
from rosgraph_msgs.msg import Clock

_RECORD = struct.Struct('<4sB3xQQ')
_MAGIC = b'SCLK'


class LifecycleSimClock(rclpy.lifecycle.Node):
    """TCP records from UnrealSimClock -> /clock."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('port', 3008)
        # Seconds without a record before the log says the simulation stopped.
        self.declare_parameter('stall_s', 2.0)

        self._pub = None
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._is_active = False
        self._last_ns = 0

    # ---- lifecycle ----------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        if self.get_parameter('use_sim_time').value:
            self.get_logger().error('The clock source cannot run on simulation time: set use_sim_time false.')
            return TransitionCallbackReturn.FAILURE
        self._pub = self.create_lifecycle_publisher(Clock, '/clock', QoSProfile(depth=1))
        bind = (self.get_parameter('bind_ip').value, self.get_parameter('port').value)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(bind)
        self._sock.listen(1)
        self._sock.settimeout(0.5)
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'Listening for the UnrealSimClock plugin on TCP {bind[0]}:{bind[1]}.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        self.get_logger().info('Lifecycle: ACTIVE — publishing /clock.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def _release(self) -> None:
        self._is_active = False
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock:
            self._sock.close()
            self._sock = None
        if self._pub:
            self.destroy_publisher(self._pub)
            self._pub = None

    # ---- receive ------------------------------------------------------------

    def _loop(self) -> None:
        """Accept the plugin's connection (one per Play session) and read it."""
        while not self._stop.is_set():
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.get_logger().info(f'UnrealSimClock connected from {addr[0]}:{addr[1]}.')
            try:
                self._read(conn)
            finally:
                conn.close()
                if not self._stop.is_set():
                    self.get_logger().info('UnrealSimClock disconnected (Play stopped): '
                                           'simulation time is standing still.')

    def _read(self, conn: socket.socket) -> None:
        stall = self.get_parameter('stall_s').value
        conn.settimeout(0.5)
        buf, running, last_rx = b'', False, time.monotonic()
        while not self._stop.is_set():
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                if running and time.monotonic() - last_rx > stall:
                    self.get_logger().info('Simulation clock paused (no record from Unreal).')
                    running = False
                continue
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while len(buf) >= _RECORD.size:
                start = buf.find(_MAGIC)
                if start < 0:
                    buf = buf[-3:]           # keep a possible partial magic
                    break
                buf = buf[start:]
                if len(buf) < _RECORD.size:
                    break
                magic, version, sim_ns, _frame = _RECORD.unpack_from(buf)
                buf = buf[_RECORD.size:]
                if version != 1:
                    continue
                last_rx = time.monotonic()
                if not running:
                    running = True
                    self.get_logger().info(f'Simulation clock running: sim time {sim_ns * 1e-9:.3f} s.')
                self._publish(sim_ns)

    def _publish(self, sim_ns: int) -> None:
        if sim_ns <= self._last_ns:         # repeated or older: keep /clock monotonic
            return
        self._last_ns = sim_ns
        if self._is_active and self._pub is not None:
            msg = Clock()
            msg.clock = RosTime(sec=sim_ns // 1_000_000_000, nanosec=sim_ns % 1_000_000_000)
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleSimClock('lc_component_sim_clock')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
