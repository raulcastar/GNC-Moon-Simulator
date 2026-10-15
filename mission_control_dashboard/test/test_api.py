"""The JSON API (/api/ on 5001) against a stub node: nothing reaches ROS."""
import collections

import pytest

from src import api, routes, web  # noqa: F401  (every route registered before the first request)


class StubNode:
    def __init__(self, playing=True):
        self.playing = playing
        self.goals, self.drives, self.stops = [], [], 0
        self._remote = {'teleop_max_angular': 0.15}
        self._last_odom = {'x': 1.0, 'y': 2.0}
        self._mission = {'state': 'idle'}
        self._route = {'goal': None, 'current': [], 'replans': 0}
        self._arrivals = collections.defaultdict(collections.deque)
        self._log_lines = collections.deque([{'level': 'INFO', 'node': 'a', 'text': 'hello', 'stamp': 0.0},
                                             {'level': 'WARN', 'node': 'b', 'text': 'careful', 'stamp': 1.0}])
        self._last_alerts, self._node_states, self._avoidance, self._light_level = [], {}, True, 0.5
        self._teleop_engaged = False

    def _live_simulation(self):
        return self.playing

    def _is_active(self):
        return True

    def dispatch_goal(self, x, y):
        self.goals.append((x, y))

    def teleop_drive(self, lin, ang):
        self.drives.append((lin, ang))

    def stop_rover(self):
        self.stops += 1

    def rover_status(self):
        return {'active': 'yutu', 'rovers': [{'key': 'leo_raph'}, {'key': 'yutu'}], 'busy': False}

    def _bag_status(self):
        return {'recording': False}

    def latest_jpegs(self):
        return {}


@pytest.fixture()
def client():
    saved = web.node
    web.node = StubNode()
    yield web.app.test_client()
    web.node = saved


def test_status_and_log(client):
    status = client.get('/api/status').get_json()
    assert status['simulation_running'] and status['pose'] == {'x': 1.0, 'y': 2.0}
    assert status['rover']['active'] == 'yutu'
    assert [e['text'] for e in client.get('/api/log?level=WARN').get_json()['lines']] == ['careful']


def test_goals_need_play_and_numbers(client):
    assert client.post('/api/goal', json={'x': 3, 'y': 4}).get_json()['ok']
    assert web.node.goals == [(3.0, 4.0)]
    assert client.post('/api/goal', json={'x': 3}).status_code == 400
    web.node.playing = False
    reply = client.post('/api/goal', json={'x': 3, 'y': 4})
    assert reply.status_code == 409 and 'Play' in reply.get_json()['error']


def test_drive_is_clamped_and_ends_stopped(client):
    reply = client.post('/api/drive', json={'linear': 9.0, 'angular': -1.0, 'seconds': 0.3}).get_json()
    assert reply['commanded'] == {'linear': 0.25, 'angular': -0.15, 'seconds': 0.3}
    assert web.node.drives[0] == (0.25, -0.15) and web.node.drives[-1] == (0.0, 0.0)


def test_unknown_names_are_refused_with_the_valid_ones(client):
    assert 'cam_central' in client.get('/api/camera/nope').get_json()['error']
    assert 'yutu' in client.post('/api/rover', json={'rover': 'apollo'}).get_json()['error']
    assert 'follow' in client.post('/api/viewport', json={'mode': 'drone'}).get_json()['error']
    assert 'navigation' in client.post('/api/record', json={'action': 'start', 'preset': 'x'}).get_json()['error']
