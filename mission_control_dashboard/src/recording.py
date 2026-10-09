"""Recording: rosbags (presets, names, notes, management) and the snapshot zip."""

import io
import json
import os
import re
import shutil
import signal
import subprocess
import time
import zipfile
from datetime import datetime, timezone

import numpy as np

from .settings import BAG_LOW_DISK_BYTES, BAG_PRESETS, DEFAULT_BAG_PRESET

NOTES_FILE = 'notes.json'        # Per bag: label, notes, preset, topics (written by the dashboard)


class RecordingMixin:
    """Recording: rosbag and the snapshot zip. Mixed into LifecycleMissionControlUI."""

    # ---- recording ------------------------------------------------------------

    def _recording(self) -> bool:
        return self._bag_proc is not None and self._bag_proc.poll() is None

    def _bag_status(self) -> dict:
        """Recorder state for the overview's REC button and the BAGS window (once a second)."""
        running = self._recording()
        directory = self.get_parameter('bag_directory').value
        size = 0
        if running and self._bag_path and os.path.isdir(self._bag_path):
            size = sum(e.stat().st_size for e in os.scandir(self._bag_path) if e.is_file())
        now = time.time()
        rate = (size - self._bag_size[0]) / max(1e-3, now - self._bag_size[1]) if running and self._bag_size[1] else 0.0
        self._bag_size = (size, now) if running else (0, 0.0)
        try:
            free = shutil.disk_usage(directory if os.path.isdir(directory) else '/').free
        except OSError:
            free = None
        live = self._live_simulation()
        warnings = []
        if running and not live:
            warnings.append('Simulation not running: little or no data is being recorded.')
        if free is not None and free < BAG_LOW_DISK_BYTES:
            warnings.append(f'Low disk space: {free / 1e9:.1f} GB free.')
        choice = self._bag_choice
        return {'recording': running,
                'name': os.path.basename(self._bag_path) if self._bag_path else None,
                'elapsed': round(now - self._bag_t0) if running else 0,
                'size': size, 'rate': max(0.0, rate), 'free': free,
                'preset': choice['preset'], 'topic_count': len(self._bag_topics(choice)),
                'live': live, 'replaying': self._replaying(), 'warnings': warnings}

    def _live_simulation(self) -> bool:
        """Unreal is in Play: its simulation clock is ticking (and it is not a replay's)."""
        return time.monotonic() - self._clock_rx < 1.5 and not self._replaying()

    @staticmethod
    def _bag_topics(choice: dict) -> list:
        if choice.get('preset') == 'custom':
            return sorted(set(choice.get('topics') or []))
        return list(BAG_PRESETS.get(choice.get('preset'), BAG_PRESETS[DEFAULT_BAG_PRESET])['topics'])

    def recorder_info(self) -> dict:
        """Presets and the last choice, for the BAGS window's recorder."""
        return {'presets': BAG_PRESETS, 'choice': self._bag_choice,
                'live_topics': sorted(n for n, _ in self.get_topic_names_and_types())}

    def start_bag(self, preset: str = None, topics=None, label: str = '', notes: str = ''):
        """
        Starts recording. Without arguments (the overview's REC) it uses the last
        preset chosen in the BAGS window. Returns an error text, or None.
        """
        if self._recording():
            return None
        if self._replaying():
            return self._bag_error('Playback in progress: stop it before recording.')
        if preset is not None:
            self._bag_choice = {'preset': preset if preset in BAG_PRESETS or preset == 'custom'
                                else DEFAULT_BAG_PRESET, 'topics': list(topics or [])}
        chosen = self._bag_topics(self._bag_choice)
        if not chosen:
            return self._bag_error('No topics selected.')
        directory = self.get_parameter('bag_directory').value
        os.makedirs(directory, exist_ok=True)
        tag = re.sub(r'[^A-Za-z0-9_-]+', '_', (label or '').strip()).strip('_')[:40]
        name = datetime.now().strftime('gnc_%Y%m%d_%H%M%S') + (f'_{tag}' if tag else '')
        self._bag_path = os.path.join(directory, name)
        self._bag_meta = {'label': (label or '').strip(), 'notes': (notes or '').strip(),
                          'preset': self._bag_choice['preset'], 'topics': chosen,
                          'started_utc': datetime.now(timezone.utc).isoformat(timespec='seconds')}
        self._bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'mcap', '-o', self._bag_path] + chosen,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        self._bag_t0 = time.time()
        self._bag_size = (0, 0.0)
        live = '' if self._live_simulation() else ' (simulation not running)'
        self._console('INFO', f'Recording rosbag {name}: {self._bag_choice["preset"]}, '
                              f'{len(chosen)} topics{live}.')
        return None

    def stop_bag(self) -> None:
        proc = self._bag_proc
        if proc is None or proc.poll() is not None:
            return
        os.killpg(proc.pid, signal.SIGINT)      # lets rosbag close the file cleanly
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        if os.path.isdir(self._bag_path):
            self._bag_meta['stopped_utc'] = datetime.now(timezone.utc).isoformat(timespec='seconds')
            self._write_notes(self._bag_path, self._bag_meta)
        self._console('INFO', f'Rosbag saved: mission_control_dashboard/bags/'
                              f'{os.path.basename(self._bag_path)}')

    def _bag_error(self, text: str) -> str:
        self._console('WARN', f'Recording: {text}')
        return text

    # ---- bag management ---------------------------------------------------------

    @staticmethod
    def _write_notes(path: str, meta: dict) -> None:
        with open(os.path.join(path, NOTES_FILE), 'w', encoding='utf-8') as f:
            json.dump(meta, f, indent=2)

    @staticmethod
    def read_notes(path: str) -> dict:
        try:
            with open(os.path.join(path, NOTES_FILE), encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _bag_dir(self, name: str):
        """Path of bag `name` inside the bag directory, or None (also for in-use bags)."""
        directory = os.path.realpath(self.get_parameter('bag_directory').value)
        path = os.path.realpath(os.path.join(directory, os.path.basename(str(name))))
        if os.path.dirname(path) != directory or not os.path.isdir(path):
            return None
        return path

    def _bag_in_use(self, path: str) -> bool:
        busy = []
        if self._recording() and self._bag_path:
            busy.append(os.path.realpath(self._bag_path))
        if self._replaying() and self._replay.get('bag'):
            busy.append(os.path.realpath(os.path.join(self.get_parameter('bag_directory').value,
                                                      self._replay['bag'])))
        return path in busy

    def rename_bag(self, name: str, new_name: str):
        path = self._bag_dir(name)
        new = re.sub(r'[^A-Za-z0-9_-]+', '_', str(new_name).strip()).strip('_')[:80]
        if path is None:
            return 'Recording not found.'
        if not new:
            return 'The new name is empty.'
        if self._bag_in_use(path):
            return 'The recording is in use (recording or playback).'
        target = os.path.join(os.path.dirname(path), new)
        if os.path.exists(target):
            return f'A recording named {new} already exists.'
        os.rename(path, target)
        self._console('INFO', f'Bag renamed: {os.path.basename(path)} -> {new}')
        return None

    def set_bag_notes(self, name: str, notes: str):
        path = self._bag_dir(name)
        if path is None:
            return 'Recording not found.'
        meta = self.read_notes(path)
        meta['notes'] = str(notes).strip()
        self._write_notes(path, meta)
        return None

    def delete_bag(self, name: str):
        path = self._bag_dir(name)
        if path is None:
            return 'Recording not found.'
        if self._bag_in_use(path):
            return 'The recording is in use (recording or playback).'
        shutil.rmtree(path)
        self._console('INFO', f'Bag deleted: {os.path.basename(path)}')
        return None

    def snapshot_zip(self) -> bytes:
        """Latest camera frames, local DEM, lidar cloud and state, as a zip."""
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
            for feed_id, jpeg in list(self._last_jpeg.items()):
                z.writestr(f'{feed_id}.jpg', jpeg)
            if self._last_dem is not None:
                dem, res = self._last_dem
                b = io.BytesIO()
                np.savez_compressed(b, dem=dem, cell_size_m=res,
                                    axes='rows: forward from the camera plane; '
                                         'columns: lateral, right +; metres above '
                                         'the ground under the rover; NaN = unobserved')
                z.writestr('local_dem.npz', b.getvalue())
            if self._last_cloud is not None:
                b = io.BytesIO()
                np.savez_compressed(b, xyz=self._last_cloud, frame='rover: x forward, y left, z up')
                z.writestr('lidar.npz', b.getvalue())
            z.writestr('metadata.json', json.dumps({
                'utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
                'odom': self._last_odom, 'mission': self._mission,
                'nodes': self._node_states, 'feeds': sorted(self._last_jpeg),
            }, indent=2))
        return buf.getvalue()
