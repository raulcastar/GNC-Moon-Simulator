"""Parameter parsing for the stereo node. No ROS."""

from typing import NamedTuple, Optional


class ExtraCamera(NamedTuple):
    """A mono camera streamed as it is, besides the stereo rig."""
    cam_id: int
    topic: str
    hfov_deg: Optional[float]
    mask_cam_id: Optional[int] = None    # its object-mask capture's cam id, if streamed

    @property
    def key(self) -> str:
        return f'extra_{self.cam_id}'


def frame_id(topic: str) -> str:
    """
    Topic namespace -> TF frame name: 'navcam/central' -> 'navcam_central'.

    tf2 reserves '/' in frame names for the deprecated tf_prefix syntax.
    """
    return topic.strip('/').replace('/', '_')


def _int(text: str) -> Optional[int]:
    return int(text) if text.lstrip('-').isdigit() else None


def parse_extra_camera(entry) -> Optional[ExtraCamera]:
    """
    One extra_cameras entry, "<cam_id>:<topic>[:<hfov_deg>[:<mask_cam_id>]]"
    (hfov may be left empty, "10:rear_cam::13"), or None if malformed.
    """
    parts = [x.strip() for x in str(entry).split(':')]
    if not 2 <= len(parts) <= 4 or _int(parts[0]) is None or not parts[1]:
        return None
    try:
        hfov = float(parts[2]) if len(parts) > 2 and parts[2] else None
    except ValueError:
        return None
    mask = None
    if len(parts) == 4:
        mask = _int(parts[3])
        if mask is None or mask < 0:
            return None
    return ExtraCamera(int(parts[0]), parts[1].strip('/'), hfov, mask)
