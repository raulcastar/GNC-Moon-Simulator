"""Replay window (5007): browse the recorded rosbags and play one back."""

import os
import signal
import subprocess
import threading
import time

from builtin_interfaces.msg import Time as RosTime
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rosbag2_interfaces.srv import Pause, Resume, Seek, SetRate

from .settings import REPLAY_CLOCK_HZ, REPLAY_LIVE_OUTPUTS, REPLAY_PERIOD
from .web import sio_replay

_PLAYER = 'rosbag2_player'               # ros2 bag play's node name (its services live under it)
_PLAYER_LOG = '/tmp/rosbag2_player.log'  # its output (read back only when it fails)


def _seconds(value) -> float:
    """Bag metadata times/durations: rclpy Time/Duration (Jazzy) or datetime/timedelta."""
    if hasattr(value, 'nanoseconds'):
        return value.nanoseconds * 1e-9
    return value.timestamp() if hasattr(value, 'timestamp') else value.total_seconds()


class ReplayMixin:
    """
    Plays one bag at a time with `ros2 bag play` and drives it through the
    player's services (pause / resume / rate / seek). The replayed topics reach
    every node as if live: the overview shows them, and nodes on simulation time
    follow the bag's /clock (recorded since the simulation clock exists; older bags
    get one generated from their timestamps). Replay is refused while Unreal is
    playing: two sources would fight over the same topics. Mixed into
    LifecycleMissionControlUI.
    """

    def _init_replay(self) -> None:
        self._replay_proc = None
        self._replay = {'state': 'idle', 'bag': None, 'rate': 1.0, 'loop': False, 'start_ns': 0,
                        'duration': 0.0, 'paused': False, 'error': None}
        self._replay_lock = threading.Lock()
        self._clock_ns = 0                   # newest /clock seen, ns
        self._clock_rx = 0.0                 # monotonic time it arrived
        self._seek_pos = None                # (position s, /clock at the seek): shown until the clock moves
        self._replay_clients = {}
        self._replay_timer = None
        self._sub_clock = None

    def _configure_replay(self) -> None:
        self._sub_clock = self.create_subscription(
            Clock, '/clock', self._on_clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        for name, srv in (('pause', Pause), ('resume', Resume), ('set_rate', SetRate), ('seek', Seek)):
            self._replay_clients[name] = self.create_client(srv, f'/{_PLAYER}/{name}')
        self._replay_timer = self.create_timer(REPLAY_PERIOD, self._replay_tick)

    def _cleanup_replay(self) -> None:
        self.stop_replay()
        if self._replay_timer is not None:
            self.destroy_timer(self._replay_timer)
            self._replay_timer = None
        if self._sub_clock is not None:
            self.destroy_subscription(self._sub_clock)
            self._sub_clock = None
        for client in self._replay_clients.values():
            self.destroy_client(client)
        self._replay_clients = {}

    def _on_clock(self, msg: Clock) -> None:
        self._clock_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        self._clock_rx = time.monotonic()

    # ---- bags ---------------------------------------------------------------

    def _bag_directory(self) -> str:
        return self.get_parameter('bag_directory').value

    def _replaying(self) -> bool:
        return self._replay_proc is not None and self._replay_proc.poll() is None

    def list_bags(self, sid=None) -> None:
        """Every bag in the bag directory, newest first, with its metadata."""
        import rosbag2_py                    # heavy import, only when the window asks
        bags, directory = [], self._bag_directory()
        names = sorted(os.listdir(directory), reverse=True) if os.path.isdir(directory) else []
        for name in names:
            path = os.path.join(directory, name)
            if not os.path.isfile(os.path.join(path, 'metadata.yaml')):
                continue
            notes = self.read_notes(path)
            entry = {'name': name, 'size': sum(os.path.getsize(os.path.join(path, f)) for f in os.listdir(path)),
                     'notes': notes.get('notes', ''), 'preset': notes.get('preset'),
                     'recording': self._recording() and os.path.realpath(path) == os.path.realpath(self._bag_path or '')}
            try:
                meta = rosbag2_py.Info().read_metadata(path, '')
                entry.update(
                    start=_seconds(meta.starting_time),
                    duration=_seconds(meta.duration),
                    messages=meta.message_count,
                    topics=sorted(({'name': t.topic_metadata.name, 'type': t.topic_metadata.type,
                                    'count': t.message_count} for t in meta.topics_with_message_count),
                                  key=lambda t: t['name']))
            except Exception as exc:
                entry['error'] = f'unreadable: {exc}'
            bags.append(entry)
        sio_replay.emit('bags', {'bags': bags, 'directory': 'mission_control_dashboard/bags',
                                 'live_outputs': REPLAY_LIVE_OUTPUTS}, to=sid)

    # ---- player -------------------------------------------------------------

    def _unreal_playing(self) -> bool:
        """True when a live simulation clock is ticking (Unreal in Play) and it is not ours."""
        live = time.monotonic() - self._clock_rx < 1.5
        return live and not (self._replay_proc is not None and self._replay_proc.poll() is None)

    def start_replay(self, name: str, rate: float, loop: bool, offset: float, topics) -> None:
        import rosbag2_py
        self.stop_replay()
        path = os.path.join(self._bag_directory(), os.path.basename(name))
        if not os.path.isfile(os.path.join(path, 'metadata.yaml')):
            self._replay_error(f'Recording {name} not found.')
            return
        if self._unreal_playing():
            self._replay_error('Simulation running: stop Play in Unreal before playback.')
            return
        if self._recording():
            self._replay_error('Recording in progress: stop it before playback.')
            return
        meta = rosbag2_py.Info().read_metadata(path, '')
        in_bag = {t.topic_metadata.name for t in meta.topics_with_message_count}
        cmd = ['ros2', 'bag', 'play', path, '--rate', f'{max(0.05, min(rate, 10.0)):g}',
               '--disable-keyboard-controls']
        if loop:
            cmd.append('--loop')
        if offset > 0:
            cmd += ['--start-offset', f'{offset:g}']
        chosen = [t for t in (topics or []) if t in in_bag]
        if chosen and len(chosen) < len(in_bag):
            cmd += ['--topics'] + chosen
        clock_recorded = '/clock' in in_bag and (not chosen or '/clock' in chosen)
        if not clock_recorded:
            cmd += ['--clock', str(REPLAY_CLOCK_HZ)]    # older bag: a clock from its timestamps
        with open(_PLAYER_LOG, 'w') as log:           # a file, never a pipe: a full pipe would stall the player
            self._replay_proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                                 start_new_session=True)
        with self._replay_lock:
            self._replay.update(state='playing', bag=os.path.basename(path), rate=rate, loop=loop,
                                start_ns=int(round(_seconds(meta.starting_time) * 1e9)),
                                duration=_seconds(meta.duration), paused=False, error=None)
        self._console('INFO', f'Replaying rosbag {os.path.basename(path)} at {rate:g}x'
                              f'{" (looping)" if loop else ""}'
                              f'{"" if clock_recorded else " with a generated /clock"}.')
        self._replay_tick()

    def stop_replay(self) -> None:
        proc = self._replay_proc
        self._replay_proc = None
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
            self._console('INFO', 'Replay stopped.')
            self._clock_rx = 0.0             # that replay's own /clock was not Unreal playing
        with self._replay_lock:
            if self._replay['state'] in ('playing', 'paused'):
                self._replay.update(state='idle', paused=False)

    def _call(self, name: str, request) -> None:
        client = self._replay_clients.get(name)
        if client is None or not client.service_is_ready():
            self._replay_error(f'Player not ready ({name}); try again in a moment.')
            return
        client.call_async(request)

    def pause_replay(self, paused: bool) -> None:
        if self._replay_proc is None or self._replay_proc.poll() is not None:
            return
        self._call('pause' if paused else 'resume', Pause.Request() if paused else Resume.Request())
        with self._replay_lock:
            self._replay.update(paused=paused, state='paused' if paused else 'playing')
        self._replay_tick()

    def set_replay_rate(self, rate: float) -> None:
        rate = max(0.05, min(float(rate), 10.0))
        with self._replay_lock:
            self._replay['rate'] = rate
        if self._replay_proc is not None and self._replay_proc.poll() is None:
            req = SetRate.Request()
            req.rate = rate
            self._call('set_rate', req)
        self._replay_tick()

    def seek_replay(self, fraction: float) -> None:
        if self._replay_proc is None or self._replay_proc.poll() is not None:
            return
        with self._replay_lock:
            t_ns = self._replay['start_ns'] + int(max(0.0, min(fraction, 1.0)) * self._replay['duration'] * 1e9)
        req = Seek.Request()
        req.time = RosTime(sec=t_ns // 1_000_000_000, nanosec=t_ns % 1_000_000_000)
        self._call('seek', req)
        # A paused player publishes no /clock: show where it now is until the clock moves.
        self._seek_pos = ((t_ns - self._replay['start_ns']) * 1e-9, self._clock_ns)
        self._replay_tick()

    def _replay_error(self, text: str) -> None:
        with self._replay_lock:
            self._replay['error'] = text
            self._replay['error_t'] = time.monotonic()
        self._console('WARN', f'Replay: {text}')
        self._replay_tick()

    def _replay_tick(self) -> None:
        """Pushes the player state (and the position, from /clock) to the replay window."""
        proc = self._replay_proc
        with self._replay_lock:
            if (self._replay.get('error') and self._replay['state'] not in ('playing', 'paused')
                    and time.monotonic() - self._replay.get('error_t', 0.0) > 8.0):
                self._replay['error'] = None          # clear a refusal from earlier
            r = dict(self._replay)
            if proc is not None and proc.poll() is not None:     # the player exited
                err = []
                if proc.returncode not in (0, -2):
                    try:
                        err = [l for l in open(_PLAYER_LOG).read().splitlines() if l.strip()]
                    except OSError:
                        pass
                self._replay_proc = None
                self._replay.update(state='finished' if proc.returncode in (0, -2) else 'failed',
                                    error=err[-1][:300] if err else None, error_t=time.monotonic())
                r = dict(self._replay)
        position = None
        if self._seek_pos is not None and self._seek_pos[1] == self._clock_ns and r['state'] in ('playing', 'paused'):
            position = self._seek_pos[0]
        elif r['state'] in ('playing', 'paused') and r['start_ns'] and self._clock_ns >= r['start_ns']:
            self._seek_pos = None
            position = min(r['duration'], (self._clock_ns - r['start_ns']) * 1e-9)
        r.update(position=position, unreal_playing=self._unreal_playing())
        sio_replay.emit('replay', r)
