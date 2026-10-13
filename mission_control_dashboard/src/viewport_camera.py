"""
The Unreal viewport's camera, chosen on the dashboard: the rover's follow
camera or an orbit around it. Unreal applies it: its rover_cameras.py
(unreal/python) polls GET /viewport_camera on the overview's port about twice
a second, passing the mode it has applied (?applied=), and switches the Play
viewport when the requested mode differs.
"""

import threading
import time

MODES = ('follow', 'orbit')
LINK_TIMEOUT_S = 3.0          # no poll for this long: Unreal's link is not running


class ViewportCamera:
    def __init__(self):
        self._lock = threading.Lock()
        self.mode = 'follow'          # requested on the dashboard
        self.applied = None           # what Unreal reports: 'follow', 'orbit', 'idle' (no Play)
        self._last_poll = 0.0

    def request(self, mode: str) -> bool:
        """Requests a mode; False if unknown."""
        if mode not in MODES:
            return False
        with self._lock:
            self.mode = mode
        return True

    def poll(self, applied, now: float = None) -> dict:
        """Unreal's poll: records what it applied, answers what is wanted."""
        with self._lock:
            self._last_poll = time.monotonic() if now is None else now
            if applied:
                self.applied = str(applied)
            return {'mode': self.mode}

    def status(self, now: float = None) -> dict:
        """For the browser: requested mode, Unreal's applied one, and whether Unreal polls."""
        now = time.monotonic() if now is None else now
        with self._lock:
            linked = now - self._last_poll < LINK_TIMEOUT_S
            return {'mode': self.mode, 'applied': self.applied if linked else None, 'linked': linked}


VIEWPORT = ViewportCamera()
