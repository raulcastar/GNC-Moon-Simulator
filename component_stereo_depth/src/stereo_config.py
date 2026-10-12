"""Parameter parsing for the stereo node. No ROS."""

from typing import NamedTuple, Optional


class ExtraCamera(NamedTuple):
    """A mono camera streamed as it is, besides the stereo rig."""
    cam_id: int
    topic: str
    hfov_deg: Optional[float]

    @property
    def key(self) -> str:
        return f'extra_{self.cam_id}'


def frame_id(topic: str) -> str:
    """
    Topic namespace -> TF frame name: 'navcam/central' -> 'navcam_central'.

    tf2 reserves '/' in frame names for the deprecated tf_prefix syntax.
    """
    return topic.strip('/').replace('/', '_')


def parse_extra_camera(entry) -> Optional[ExtraCamera]:
    """One extra_cameras entry, "<cam_id>:<topic>[:<hfov_deg>]", or None if malformed."""
    parts = [x.strip() for x in str(entry).split(':')]
    if len(parts) not in (2, 3) or not parts[0].lstrip('-').isdigit() or not parts[1]:
        return None
    hfov = float(parts[2]) if len(parts) == 3 else None
    return ExtraCamera(int(parts[0]), parts[1].strip('/'), hfov)
