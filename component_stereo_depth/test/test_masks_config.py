"""Object/shadow mask split, mask classes, parameter parsing and camera messages."""

import json

import numpy as np
import pytest

from src import masks
from src.ros_messages import camera_info, ros_time
from src.stereo_config import ExtraCamera, frame_id, parse_extra_camera


def pixel(rgb, ground=0, shadow=0, red=0):
    """A mask pixel of class colour `rgb` with the flag bits set."""
    r, g, b = (v & 0xFE for v in rgb)
    return [r | red, g | ground, b | shadow]


def split(*pixels):
    frame = np.array([pixels], dtype=np.uint8)
    shadow, labels, objects = masks.split_shadow_mask(frame)
    return shadow[0], labels[0], objects[0]


def test_shadow_labels():
    landscape, rock = (10, 96, 216), (238, 244, 242)
    _shadow, labels, _objects = split(
        pixel((0, 0, 0)),                                  # sky
        pixel(landscape, ground=1),                        # sunlit ground
        pixel(landscape, ground=1, shadow=1),              # shadowed ground
        pixel(rock),                                       # sunlit object
        pixel(rock, shadow=1),                             # shadowed object
        pixel(landscape, ground=1, shadow=1, red=1),       # the rover's shadow
        pixel((36, 40, 96), red=1))                        # sun's disc: sky
    assert labels.tolist() == [0, 1, 2, 3, 4, 5, 0]
    assert set(labels.tolist()) == set(masks.SHADOW_IDS)


def test_shadow_colours_and_object_mask():
    landscape = (10, 96, 216)
    shadow, _labels, objects = split(pixel(landscape, ground=1), pixel(landscape, ground=1, shadow=1, red=1))
    assert tuple(shadow[0]) == masks.SUNLIT_RGB and tuple(shadow[1]) == masks.ROVER_SHADOW_RGB
    assert (objects & 1).sum() == 0                        # flag bits cleared
    assert tuple(objects[0]) == landscape


def test_shadow_ids_json():
    assert json.loads(masks.shadow_ids_json())["5"] == "rover_shadow"


def test_mask_classes_parsed_sorted_and_malformed_skipped():
    entries = [{"id": 9, "name": "boulder", "rgb": [238, 244, 242], "group": "rock"},
               {"id": 1, "name": "sky", "rgb": [36, 40, 96]},
               {"id": "x", "name": "bad", "rgb": [0, 0, 0]},
               {"id": 3, "name": "", "rgb": [0, 0, 0]},
               {"id": 4, "name": "short", "rgb": [0, 0]}]
    classes, skipped = masks.parse_mask_classes(entries)
    assert [c["id"] for c in classes] == [1, 9] and len(skipped) == 3
    assert classes[0]["group"] == "other"
    legend, full = masks.mask_class_messages(classes)
    assert json.loads(legend) == {"1": "sky", "9": "boulder"}
    assert json.loads(full)[1]["rgb"] == [238, 244, 242]


def test_mask_classes_file_of_the_package():
    with open("config/mask_classes.json", encoding="utf-8") as handle:
        classes, skipped = masks.parse_mask_classes(json.load(handle)["classes"])
    assert classes and not skipped
    assert all(c["rgb"][i] % 2 == 0 for c in classes for i in range(3))   # low bits carry the flags


@pytest.mark.parametrize("entry, expected", [
    ("7:rear_cam", ExtraCamera(7, "rear_cam", None)),
    (" 7 : /rear_cam/ : 90 ", ExtraCamera(7, "rear_cam", 90.0)),
    ("-1:x", ExtraCamera(-1, "x", None)),
    ("10:rear_cam:90:13", ExtraCamera(10, "rear_cam", 90.0, 13)),     # with its object mask
    ("10:rear_cam::13", ExtraCamera(10, "rear_cam", None, 13)),       # mask, no hfov
    ("rear_cam", None), ("a:b", None), ("7:", None), ("7:x:wide", None),
    ("10:rear_cam:90:", None), ("10:rear_cam:90:-1", None), ("1:2:3:4:5", None)])
def test_extra_camera_entries(entry, expected):
    assert parse_extra_camera(entry) == expected


def test_extra_camera_key_and_frame_names():
    assert ExtraCamera(7, "rear_cam", None).key == "extra_7"
    assert frame_id("/navcam/central/") == "navcam_central"


def test_camera_info_right_eye_carries_the_baseline():
    left = camera_info(ros_time(0), "navcam_left", (160.0, 160.0, 120.0, 320, 240))
    right = camera_info(ros_time(0), "navcam_right", (160.0, 160.0, 120.0, 320, 240), 0.2)
    assert right.p[3] == pytest.approx(-32.0)
    assert left.p[3] == 0.0 and str(left.p[3]) == "0.0"   # not -0.0
    assert (left.width, left.height, left.k[0], left.k[2]) == (320, 240, 160.0, 160.0)


def test_ros_time_from_ns():
    t = ros_time(1_700_000_000_123_456_789)
    assert (t.sec, t.nanosec) == (1_700_000_000, 123_456_789)
