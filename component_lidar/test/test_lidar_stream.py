"""
The LIDR scan stream from the UnrealLidarCam plugin, parsed by LidarCamReceiver,
and its conversion to PointCloud2.

Packets are assembled byte by byte at the offsets documented in
the Unreal project's Plugins/UnrealLidarCam/Source/UnrealLidarCam/Public/UnrealLidarCam.h, so a
change to the wire format on either side fails here:

    offset size  field
    0      4     magic 'LIDR'
    4      1     version = 1
    5      1     sensor_id
    6      2     flags            bit 0: organized (rows x columns)
    8      8     frame_id (u64)
    16     8     stamp (f64)      simulation time, s since the Unix epoch
    24     4     horizontal_fov (f32, degrees)
    28     4     vertical_fov   (f32, degrees)
    32     4     columns (u32)
    36     4     rows    (u32)
    40     4     point_count (u32) = rows * columns
    44     4     point_step  (u32) = 20
    48     ...   points: x, y, z, intensity (f32), ring (u16), column (u16)
"""
import struct
import types

import numpy as np
import pytest

from src.component_lidar import LidarCamReceiver, LifecycleUnrealLidarCam

POINT = np.dtype({"names": ["x", "y", "z", "i", "ring", "col"],
                  "formats": ["<f4"] * 4 + ["<u2"] * 2,
                  "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 20})


class Log:
    def __init__(self):
        self.warnings, self.errors = [], []

    def warn(self, msg, **_):
        self.warnings.append(msg)

    def error(self, msg, **_):
        self.errors.append(msg)

    def info(self, msg, **_):
        pass


class Conn:
    """A socket that hands over the given chunks, then closes."""

    def __init__(self, chunks):
        self.chunks = list(chunks)

    def settimeout(self, _):
        pass

    def recv(self, _):
        return self.chunks.pop(0) if self.chunks else b""


def points(rows, cols, seed=0):
    rng = np.random.default_rng(seed)
    a = np.zeros(rows * cols, POINT)
    a["x"], a["y"], a["z"] = rng.uniform(1, 40, a.size), rng.uniform(-5, 5, a.size), rng.uniform(-1, 1, a.size)
    a["i"] = rng.uniform(0, 255, a.size)
    a["ring"] = np.repeat(np.arange(rows), cols)
    a["col"] = np.tile(np.arange(cols), rows)
    if a.size > 3:
        a["x"][3] = np.nan                                   # a beam without a return
    return a


def lidr(pts, rows, cols, flags=1, version=1, step=20, stamp=1_791_000_000.25, frame_id=42, count=None):
    head = bytearray(48)
    head[0:4] = b"LIDR"
    head[4] = version
    head[5] = 0                                              # sensor_id
    head[6:8] = flags.to_bytes(2, "little")
    head[8:16] = frame_id.to_bytes(8, "little")
    head[16:24] = struct.pack("<d", stamp)
    head[24:28] = struct.pack("<f", 90.0)
    head[28:32] = struct.pack("<f", 30.0)
    head[32:36] = cols.to_bytes(4, "little")
    head[36:40] = rows.to_bytes(4, "little")
    head[40:44] = (rows * cols if count is None else count).to_bytes(4, "little")
    head[44:48] = step.to_bytes(4, "little")
    return bytes(head) + pts.tobytes()


def receive(chunks):
    scans, log = [], Log()
    rx = LidarCamReceiver("127.0.0.1", 0, lambda h, p: scans.append((h, p)), log)
    rx._read(Conn(chunks))
    return scans, log


def test_one_scan():
    pts = points(4, 8)
    (header, payload), = receive([lidr(pts, 4, 8)])[0]
    magic, version, sensor, flags, frame_id, stamp, hfov, vfov, cols, rows, count, step = header
    assert (magic, version, flags, frame_id) == (b"LIDR", 1, 1, 42)
    assert stamp == pytest.approx(1_791_000_000.25)
    assert (hfov, vfov, cols, rows, count, step) == (90.0, 30.0, 8, 4, 32, 20)
    assert payload == pts.tobytes()


def test_scan_split_across_reads_and_two_in_one():
    a, b = lidr(points(2, 4, 1), 2, 4, frame_id=1), lidr(points(2, 4, 2), 2, 4, frame_id=2)
    data = a + b
    chunks = [data[i:i + 13] for i in range(0, len(data), 13)]
    scans, _ = receive(chunks)
    assert [s[0][4] for s in scans] == [1, 2]
    scans, _ = receive([data])
    assert [s[0][4] for s in scans] == [1, 2]


def test_resync_after_garbage_and_bad_header():
    good = lidr(points(2, 4), 2, 4, frame_id=7)
    bad = lidr(points(2, 4), 2, 4, step=16)                  # wrong point step
    scans, log = receive([b"\x00\x01LID" + bad + good])
    assert [s[0][4] for s in scans] == [7]
    assert any("bad header" in w for w in log.warnings)


@pytest.mark.parametrize("kwargs", [dict(version=2), dict(count=5), dict(count=0)])
def test_invalid_headers_are_rejected(kwargs):
    rows, cols = (2, 4) if kwargs.get("count") != 0 else (0, 0)
    scans, log = receive([lidr(points(max(rows, 1), max(cols, 1)), rows, cols, **kwargs)])
    assert scans == [] and log.warnings


def test_a_failing_handler_does_not_stop_the_stream():
    log = Log()
    calls = []

    def on_scan(header, payload):
        calls.append(header[4])
        if header[4] == 1:
            raise ValueError("boom")

    rx = LidarCamReceiver("127.0.0.1", 0, on_scan, log)
    rx._read(Conn([lidr(points(2, 2), 2, 2, frame_id=1) + lidr(points(2, 2), 2, 2, frame_id=2)]))
    assert calls == [1, 2] and any("boom" in e for e in log.errors)


# ---------------------------------------------------------------- PointCloud2

class Publisher:
    is_activated = True

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def to_cloud(pts, rows, cols, flags=1, stamp=1_791_000_000.25):
    (header, payload), = receive([lidr(pts, rows, cols, flags=flags, stamp=stamp)])[0]
    pub = Publisher()
    node = types.SimpleNamespace(_pub_cloud=pub, frame_id="lidar", FIELDS=LifecycleUnrealLidarCam.FIELDS,
                                 get_clock=lambda: types.SimpleNamespace(
                                     now=lambda: types.SimpleNamespace(to_msg=lambda: "now")))
    LifecycleUnrealLidarCam._publish_scan(node, header, payload)
    return pub.sent[0]


def test_organized_cloud():
    pts = points(4, 8)
    msg = to_cloud(pts, 4, 8)
    assert (msg.height, msg.width, msg.point_step, msg.row_step) == (4, 8, 20, 160)
    assert msg.header.frame_id == "lidar" and not msg.is_dense
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1_791_000_000, 250_000_000)
    back = np.frombuffer(bytes(msg.data), POINT)
    assert np.array_equal(back["ring"], pts["ring"]) and np.isnan(back["x"][3])
    names = [(f.name, f.offset) for f in msg.fields]
    assert names == [("x", 0), ("y", 4), ("z", 8), ("intensity", 12), ("ring", 16), ("column", 18)]


def test_unorganized_cloud_and_arrival_stamp():
    msg = to_cloud(points(2, 4), 2, 4, flags=0, stamp=0.0)
    assert (msg.height, msg.width) == (1, 8)
    assert msg.header.stamp == "now"                       # older plugin: stamped on arrival
