"""Recording: rosbag and the snapshot zip."""

import io
import json
import os
import signal
import subprocess
import time
import zipfile
from datetime import datetime, timezone

import numpy as np

from .settings import BAG_TOPICS


class RecordingMixin:
    """Recording: rosbag and the snapshot zip. Mixed into LifecycleMissionControlUI."""

    def _bag_status(self) -> dict:
        running = self._bag_proc is not None and self._bag_proc.poll() is None
        return {'recording': running,
                'name': os.path.basename(self._bag_path) if self._bag_path else None,
                'elapsed': round(time.time() - self._bag_t0) if running else 0}

    def start_bag(self) -> None:
        if self._bag_proc is not None and self._bag_proc.poll() is None:
            return
        directory = self.get_parameter('bag_directory').value
        os.makedirs(directory, exist_ok=True)
        self._bag_path = os.path.join(directory, datetime.now().strftime('gnc_%Y%m%d_%H%M%S'))
        self._bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'mcap', '-o', self._bag_path] + BAG_TOPICS,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        self._bag_t0 = time.time()
        self._console('INFO', f'Recording rosbag {os.path.basename(self._bag_path)} '
                              f'({len(BAG_TOPICS)} topics).')

    def stop_bag(self) -> None:
        proc = self._bag_proc
        if proc is None or proc.poll() is not None:
            return
        os.killpg(proc.pid, signal.SIGINT)      # lets rosbag close the file cleanly
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        self._console('INFO', f'Rosbag saved: mission_control_dashboard/bags/'
                              f'{os.path.basename(self._bag_path)}')

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
