"""
The NAVF frame stream from the UnrealStereoCam plugin, parsed by FrameStreamReceiver.

Packets are assembled byte by byte at the offsets documented in the
UnrealStereoCam plugin (Source/UnrealStereoCam/Public/UnrealStereoCam.h),
not with the parser's own struct,, so a change to the wire format on either
side fails here instead of in a Play session:

    offset size  field
    0      4     magic 'NAVF'
    4      1     version = 2
    5      1     cam_id
    6      1     format            1 = BGRA8 colour, 2 = R32F depth (cm)
    7      1     reserved
    8      2     width  (u16)
    10     2     height (u16)
    12     8     frame_id (u64)
    20     4     payload size (u32) = width * height * 4
    24     8     stamp_ns (u64)    (version 2 only)
    32     ...   pixels            (version 1: from offset 24)
"""
import numpy as np
import pytest

from src.frame_stream import FrameStreamReceiver


class Log:
    def __init__(self):
        self.warnings = []

    def warn(self, msg, **_):
        self.warnings.append(msg)

    def info(self, msg, **_):
        pass

    error = warn


def navf(cam_id, frame_id, pixels, fmt=1, version=2, stamp_ns=0):
    """One packet, little-endian, at the documented offsets."""
    h, w = pixels.shape[:2]
    payload = pixels.astype("<f4" if fmt == 2 else np.uint8).tobytes()
    head = bytearray(32 if version == 2 else 24)
    head[0:4] = b"NAVF"
    head[4] = version
    head[5] = cam_id
    head[6] = fmt
    head[7] = 0
    head[8:10] = w.to_bytes(2, "little")
    head[10:12] = h.to_bytes(2, "little")
    head[12:20] = frame_id.to_bytes(8, "little")
    head[20:24] = len(payload).to_bytes(4, "little")
    if version == 2:
        head[24:32] = stamp_ns.to_bytes(8, "little")
    return bytes(head) + payload


def bgra(h=4, w=6, seed=0):
    return np.random.default_rng(seed).integers(0, 256, (h, w, 4), dtype=np.uint8)


@pytest.fixture
def rx():
    receiver = FrameStreamReceiver("127.0.0.1", 0, Log())
    return receiver


def frames(rx, cam):
    return list(rx._frames.get(cam, ()))


def test_colour_frame_v2(rx):
    img = bgra()
    rest = rx._drain(navf(1, 77, img, stamp_ns=1_791_000_000_123_456_789))
    assert rest == b""
    (fid, rgb, _t, stamp), = frames(rx, 1)
    assert fid == 77 and stamp == 1_791_000_000_123_456_789
    assert rgb.shape == (4, 6, 3)
    assert np.array_equal(rgb, img[:, :, [2, 1, 0]]), "BGRA must become RGB"


def test_colour_frame_v1_has_no_stamp(rx):
    img = bgra(seed=1)
    rx._drain(navf(2, 5, img, version=1))
    (fid, rgb, _t, stamp), = frames(rx, 2)
    assert (fid, stamp) == (5, 0)
    assert np.array_equal(rgb, img[:, :, [2, 1, 0]])


def test_depth_frame_is_float_centimetres(rx):
    depth = np.array([[100.0, 250.5], [np.inf, 1e7]], dtype=np.float32)
    rx._drain(navf(4, 9, depth, fmt=2))
    (_, out, _, _), = frames(rx, 4)
    assert out.dtype == np.float32 and np.array_equal(out, depth)


def test_packet_split_across_reads(rx):
    packet = navf(1, 3, bgra())
    buf = b""
    for i in range(0, len(packet), 7):          # arrives 7 bytes at a time
        buf = rx._drain(buf + packet[i:i + 7])
    assert buf == b"" and len(frames(rx, 1)) == 1


def test_two_packets_in_one_read(rx):
    rx._drain(navf(1, 1, bgra()) + navf(2, 1, bgra(seed=3)))
    assert len(frames(rx, 1)) == len(frames(rx, 2)) == 1


def test_resync_after_garbage(rx):
    rx._drain(b"\x00junk\xffNAV" + navf(1, 4, bgra()))
    assert [f[0] for f in frames(rx, 1)] == [4]


def test_bad_header_is_skipped(rx):
    bad = bytearray(navf(1, 1, bgra()))
    bad[4] = 9                                   # unknown version
    rx._drain(bytes(bad) + navf(1, 2, bgra()))
    assert [f[0] for f in frames(rx, 1)] == [2]
    assert any("bad header" in w for w in rx._log.warnings)


def test_size_mismatch_is_rejected(rx):
    bad = bytearray(navf(1, 1, bgra()))
    bad[20:24] = (10).to_bytes(4, "little")      # payload size != w * h * 4
    rx._drain(bytes(bad) + navf(1, 2, bgra()))
    assert [f[0] for f in frames(rx, 1)] == [2]


def test_unknown_camera_id_warned_once(rx):
    rx.known_ids = {0, 1, 2}
    rx._drain(navf(77, 1, bgra()) + navf(77, 2, bgra()) + navf(1, 2, bgra()))
    unknown = [w for w in rx._log.warnings if "camera id 77" in w]
    assert len(unknown) == 1
    assert not any("camera id 1 " in w for w in rx._log.warnings)


def test_ring_keeps_the_newest_frames(rx):
    for fid in range(20):
        rx._drain(navf(1, fid, bgra(h=2, w=2)))
    kept = [f[0] for f in frames(rx, 1)]
    assert kept == list(range(20 - FrameStreamReceiver._RING, 20))


def test_latest_pair_matches_eyes_by_frame_id(rx):
    left, right, centre = bgra(seed=1), bgra(seed=2), bgra(seed=3)
    rx._drain(navf(1, 10, left) + navf(2, 10, right) + navf(0, 10, centre))
    rx._drain(navf(1, 11, left))                 # left ahead; no right for 11 yet
    got = rx.latest_pair(1, 2, 0)
    assert got is not None
    l_img, r_img, c_img, fid, _age, _stamp = got
    assert fid == 10, "eyes must come from the same frame"
    assert np.array_equal(l_img, left[:, :, [2, 1, 0]])
    assert np.array_equal(r_img, right[:, :, [2, 1, 0]])


def test_latest_pair_none_without_both_eyes(rx):
    rx._drain(navf(1, 10, bgra()) + navf(0, 10, bgra()))
    assert rx.latest_pair(1, 2, 0) is None


def test_frame_near_and_latest(rx):
    rx._drain(navf(3, 100, bgra(seed=1)) + navf(3, 104, bgra(seed=2)))
    assert np.array_equal(rx.frame_near(3, 103), bgra(seed=2)[:, :, [2, 1, 0]])
    assert rx.frame_near(3, 1000) is None        # too far apart: stale
    fid, _img, _stamp = rx.latest(3)
    assert fid == 104
    assert rx.latest(9) is None


def test_latest_with_pairs_a_camera_and_its_mask_by_frame_id(rx):
    cam, mask = bgra(seed=1), bgra(seed=2)
    rx._drain(navf(10, 50, cam) + navf(13, 50, mask))
    rx._drain(navf(10, 56, bgra(seed=3)))        # newer colour; its mask still in flight
    fid, img, _stamp, got_mask = rx.latest_with(10, 13)
    assert fid == 50, "the mask must come from the same frame"
    assert np.array_equal(img, cam[:, :, [2, 1, 0]])
    assert np.array_equal(got_mask, mask[:, :, [2, 1, 0]])
    rx._drain(navf(13, 56, bgra(seed=4)))        # it lands: the newer pair wins
    assert rx.latest_with(10, 13)[0] == 56
    assert rx.latest_with(10, 14) is None        # no mask stream
