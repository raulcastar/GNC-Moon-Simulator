"""System window (5006): the ROS 2 graph, and one topic inspected on demand."""

import collections
import time

import numpy as np
import yaml

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .settings import INSPECT_MAX_CHARS, INSPECT_MAX_STRING, INSPECT_TRUNCATE, INSPECT_WINDOW
from .web import sio_sys


def message_preview(value):
    """
    A ROS message as plain data for the TOPICS panel: arrays (images, clouds...)
    cut to their first INSPECT_TRUNCATE elements plus their length; strings kept
    whole up to INSPECT_MAX_STRING characters.
    """
    if hasattr(value, 'get_fields_and_field_types'):
        return {f: message_preview(getattr(value, f)) for f in value.get_fields_and_field_types()}
    if isinstance(value, str):
        return value if len(value) <= INSPECT_MAX_STRING else value[:INSPECT_MAX_STRING] + '...'
    if isinstance(value, (bytes, bytearray)) or hasattr(value, '__len__'):
        n = len(value)
        head = [message_preview(v) for v in list(value[:INSPECT_TRUNCATE])]
        return head if n <= INSPECT_TRUNCATE else head + [f'... {n} elements in all']
    if isinstance(value, np.generic):
        return value.item()
    return value


class TopicsMixin:
    """System window (5006): the ROS 2 graph, and one topic inspected on demand. Mixed into LifecycleMissionControlUI."""

    @staticmethod
    def _endpoint_nodes(infos) -> list:
        return sorted({f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in infos})

    def list_topics(self, sid) -> None:
        """Every topic in the graph with its type(s), publishers and subscribers."""
        rows = []
        for name, types in sorted(self.get_topic_names_and_types()):
            rows.append({'name': name, 'type': ', '.join(types),
                         'pubs': self._endpoint_nodes(self.get_publishers_info_by_topic(name)),
                         'subs': self._endpoint_nodes(self.get_subscriptions_info_by_topic(name))})
        sio_sys.emit('topics_list', {'topics': rows, 'self': self.get_fully_qualified_name()}, to=sid)

    def inspect_topic(self, sid, name: str) -> None:
        """
        Subscribes to `name` (serialized, so nothing is decoded per message) until
        stop_inspect; _inspect_cb reports its rate, bandwidth and latest message.
        The QoS follows the publishers: best effort if any is, transient local if
        all are (so a latched topic shows its last message).
        """
        self.stop_inspect(sid)
        types = dict(self.get_topic_names_and_types()).get(name)
        if not types:
            sio_sys.emit('topic_info', {'name': name, 'error': 'topic not found'}, to=sid)
            return
        try:
            cls = get_message(types[0])
        except Exception as exc:
            sio_sys.emit('topic_info', {'name': name, 'error': f'unknown type {types[0]}: {exc}'}, to=sid)
            return
        pubs = self.get_publishers_info_by_topic(name)
        best_effort = any(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for i in pubs)
        latched = bool(pubs) and all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                                     for i in pubs)
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
        st = {'name': name, 'type': types[0], 'cls': cls, 'arrivals': collections.deque(maxlen=1000),
              'last': None, 'last_t': None, 'shown': None, 'preview': '',
              'qos': ('best effort' if best_effort else 'reliable') + (', transient local' if latched else '')}

        def cb(raw, st=st):
            now = time.monotonic()
            st['arrivals'].append((now, len(raw)))
            st['last'], st['last_t'] = raw, now
        st['sub'] = self.create_subscription(cls, name, cb, qos, raw=True)
        with self._inspect_lock:
            self._inspect[sid] = st
        self._inspect_cb(only=sid)

    def stop_inspect(self, sid) -> None:
        with self._inspect_lock:
            st = self._inspect.pop(sid, None)
        if st is not None:
            self.destroy_subscription(st['sub'])

    def _inspect_cb(self, only=None) -> None:
        """Rate, bandwidth and latest message of each inspected topic, to its browser."""
        with self._inspect_lock:
            items = [(sid, st) for sid, st in self._inspect.items() if only is None or sid == only]
        now = time.monotonic()
        for sid, st in items:
            recent = [(t, n) for t, n in list(st['arrivals']) if now - t <= INSPECT_WINDOW]
            span = recent[-1][0] - recent[0][0] if len(recent) > 1 else 0.0
            hz = (len(recent) - 1) / span if span > 0 else 0.0
            bw = sum(n for _, n in recent) / INSPECT_WINDOW
            if st['last'] is not None and st['last'] is not st['shown']:
                st['shown'] = st['last']
                try:
                    text = yaml.safe_dump(message_preview(deserialize_message(st['last'], st['cls'])),
                                          sort_keys=False, default_flow_style=None, width=110,
                                          allow_unicode=True)
                except Exception as exc:
                    text = f'(could not decode: {exc})'
                if len(text) > INSPECT_MAX_CHARS:
                    text = text[:INSPECT_MAX_CHARS] + f'\n... ({len(text) - INSPECT_MAX_CHARS} more characters)'
                st['preview'] = text
            sio_sys.emit('topic_info', {
                'name': st['name'], 'type': st['type'], 'qos': st['qos'],
                'hz': round(hz, 2), 'bw': round(bw), 'count': len(st['arrivals']),
                'size': st['arrivals'][-1][1] if st['arrivals'] else None,
                'age': round(now - st['last_t'], 1) if st['last_t'] else None,
                'pubs': self._endpoint_nodes(self.get_publishers_info_by_topic(st['name'])),
                'subs': self._endpoint_nodes(self.get_subscriptions_info_by_topic(st['name'])),
                'preview': st['preview']}, to=sid)
