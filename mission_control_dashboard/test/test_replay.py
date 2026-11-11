"""Replay position: measured along the bag's own /clock, which can lag its timestamps."""
import rosbag2_py
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock

from src import replay

SEC = 1_000_000_000


def _write_bag(path, clocks_and_stamps):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/clock', type='rosgraph_msgs/msg/Clock', serialization_format='cdr'))
    for clock_ns, stamp_ns in clocks_and_stamps:
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(clock_ns, SEC)
        writer.write('/clock', serialize_message(msg), stamp_ns)
    del writer


def test_bag_clock_range_reads_the_recorded_clock(tmp_path):
    # A simulation at 0.7x real time, 265 s behind the wall clock the bag is stamped with.
    stamps = [1_000_000 * SEC + i * SEC for i in range(11)]
    clocks = [stamps[0] - 265 * SEC + int(i * 0.7 * SEC) for i in range(11)]
    _write_bag(tmp_path / 'bag', zip(clocks, stamps))
    assert replay._bag_clock_range(str(tmp_path / 'bag'), 'mcap') == (clocks[0], clocks[-1])


def test_position_follows_the_bag_clock(monkeypatch):
    emitted = []
    monkeypatch.setattr(replay.sio_replay, 'emit', lambda event, data: emitted.append(data))
    node = replay.ReplayMixin()
    node._init_replay()
    first, last = 735 * SEC, 742 * SEC                # 7 s of simulation over a 10 s recording
    node._replay.update(state='playing', start_ns=1_000_000 * SEC, duration=10.0, clock_range=(first, last))
    node._clock_ns = first + int(3.5 * SEC)           # half way through the bag's clock
    node._replay_tick()
    assert emitted[-1]['position'] == 5.0
    assert 'clock_range' not in emitted[-1]
