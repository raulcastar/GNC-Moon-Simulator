"""
Object and shadow masks from Terrain Studio's mask material. No ROS.

The mask capture colours each surface by its class (config/mask_classes.json)
and uses the lowest bit of each channel as a flag: green = ground, blue = no
sunlight, red = the rover's own shadow (with blue) or the sky (without blue:
sky, sun's disc, glare).
"""

import json
from typing import List, Tuple

import numpy as np

# Shadow-mask colours.
SHADOW_RGB = (0, 0, 0)            # Any surface the sun does not reach
SUNLIT_RGB = (40, 120, 255)       # Ground in sunlight
SKY_RGB = (0, 0, 0)               # Untagged (sky)
OTHER_RGB = (128, 128, 128)       # Rocks, rover in sunlight
ROVER_SHADOW_RGB = (255, 140, 0)  # The rover's own cast shadow

# Shadow class ids (shadow_label, mono8), for training.
SHADOW_IDS = {0: 'sky', 1: 'sunlit_ground', 2: 'shadowed_ground',
              3: 'sunlit_object', 4: 'shadowed_object', 5: 'rover_shadow'}


def shadow_tables() -> Tuple[np.ndarray, np.ndarray]:
    """Colour and SHADOW_IDS label for each 4-bit code of split_shadow_mask."""
    palette = np.empty((16, 3), np.uint8)
    labels = np.empty(16, np.uint8)
    for code in range(16):
        ground, shadow, red, sky = (code >> bit & 1 for bit in range(4))
        rover, sun = red & shadow, red & (1 - shadow)
        sky = sky | sun                 # Red without blue: the sky (sun included)
        colour = SKY_RGB if sky else OTHER_RGB
        colour = SUNLIT_RGB if ground else colour
        colour = SHADOW_RGB if shadow else colour
        palette[code] = ROVER_SHADOW_RGB if rover else colour
        label = 0 if sky else (1 if ground else 3) + shadow
        labels[code] = 5 if rover else label
    return palette, labels


_PALETTE, _LABELS = shadow_tables()


def split_shadow_mask(frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Splits an RGB mask frame into the shadow feed (colours), the shadow label
    image (SHADOW_IDS) and the object mask (the frame with the flag bits
    cleared). The sun stays sky in the shadow feeds.
    """
    # One 4-bit code per pixel (ground | shadow | red flag | sky), then
    # two table lookups: ~3x faster than masked writes per class.
    objects = frame & np.uint8(0xFE)
    r, g, b = frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
    sky = (objects[:, :, 0] | objects[:, :, 1] | objects[:, :, 2]) == 0
    code = (g & 1) | ((b & 1) << 1) | ((r & 1) << 2) | (sky.view(np.uint8) << 3)
    return np.take(_PALETTE, code, axis=0), np.take(_LABELS, code), objects


def shadow_ids_json() -> str:
    """The shadow_label legend, as published on shadow_label_mapping."""
    return json.dumps({str(k): v for k, v in SHADOW_IDS.items()})


def parse_mask_classes(entries) -> Tuple[List[dict], List[dict]]:
    """
    The well-formed classes of a mask_classes.json 'classes' list, sorted by id,
    and the malformed entries that were skipped.
    """
    classes, skipped = [], []
    for entry in entries or []:
        cid, name, rgb = entry.get('id'), entry.get('name'), entry.get('rgb')
        if (not isinstance(cid, int) or not isinstance(name, str) or not name
                or not isinstance(rgb, list) or len(rgb) != 3):
            skipped.append(entry)
            continue
        classes.append({'id': cid, 'name': name, 'group': entry.get('group', 'other'),
                        'rgb': [int(v) for v in rgb]})
    classes.sort(key=lambda c: c['id'])
    return classes, skipped


def mask_class_messages(classes: List[dict]) -> Tuple[str, str]:
    """
    The two JSON strings published with the mask: the id -> name legend
    (object_mask_mapping, e.g. {"1": "sky", "5": "landscape"}) and the full
    class list with colours and groups (object_mask_classes).
    """
    return json.dumps({str(c['id']): c['name'] for c in classes}), json.dumps(classes)
