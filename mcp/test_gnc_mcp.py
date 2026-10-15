"""
The GNC MCP server against a fake dashboard API (a local HTTP server): nothing
reaches ROS or Unreal. Run on the host: py -m pytest mcp
"""
import asyncio
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class FakeDashboard(BaseHTTPRequestHandler):
    calls = []
    mission = {'state': 'reached', 'goal': [3.0, 4.0]}

    def log_message(self, *args):
        pass

    def _reply(self, code, payload, kind='application/json'):
        body = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self.calls.append(('GET', self.path, None))
        if self.path == '/api/status':
            self._reply(200, {'simulation_running': True, 'pose': {'x': 3.0, 'y': 4.0},
                              'mission': self.mission})
        elif self.path == '/api/rover':
            self._reply(200, {'active': 'yutu', 'busy': False})
        elif self.path.startswith('/api/camera/cam_central'):
            self._reply(200, b'\xff\xd8fakejpeg\xff\xd9', 'image/jpeg')
        else:
            self._reply(404, {'error': f'Unknown camera feed {self.path}'})

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
        self.calls.append(('POST', self.path, body))
        if self.path == '/api/goal' and not isinstance(body.get('x'), (int, float)):
            self._reply(400, {'error': 'x is required'})
        else:
            self._reply(200, {'ok': True, 'echo': body})


@pytest.fixture()
def gnc(monkeypatch):
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), FakeDashboard)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    import gnc_mcp
    monkeypatch.setattr(gnc_mcp, 'DASHBOARD', f'http://127.0.0.1:{httpd.server_port}')
    FakeDashboard.calls = []
    yield gnc_mcp
    httpd.shutdown()


def test_the_tools_are_listed_with_descriptions(gnc):
    tools = {t.name: t for t in asyncio.run(gnc.server.list_tools())}
    for name in ('get_status', 'send_goal', 'wait_for_mission', 'stop', 'drive', 'get_camera_image',
                 'switch_rover', 'set_headlights', 'record_bag'):
        assert name in tools and tools[name].description
    assert tools['get_status'].annotations.read_only_hint
    assert tools['switch_rover'].annotations.destructive_hint


def test_commands_reach_the_dashboard_api(gnc):
    assert gnc.send_goal(3.0, 4.0)['ok']
    assert gnc.drive(0.2, 0.0, 1.0)['ok']
    gnc.set_headlights(level=0.5)
    posts = [(p, b) for m, p, b in FakeDashboard.calls if m == 'POST']
    assert posts == [('/api/goal', {'x': 3.0, 'y': 4.0}),
                     ('/api/drive', {'linear': 0.2, 'angular': 0.0, 'seconds': 1.0}),
                     ('/api/headlights', {'level': 0.5})]


def test_wait_for_mission_returns_the_final_state(gnc):
    result = gnc.wait_for_mission(timeout_s=5.0)
    assert result['finished'] and result['mission']['state'] == 'reached'


def test_camera_image_and_errors_come_back_readable(gnc):
    image = gnc.get_camera_image('cam_central')
    assert image.data.startswith(b'\xff\xd8')
    with pytest.raises(gnc.DashboardError, match='Unknown camera feed'):
        gnc.get_camera_image('nope')


def test_an_unreachable_dashboard_says_how_to_fix_it(gnc, monkeypatch):
    monkeypatch.setattr(gnc, 'DASHBOARD', 'http://127.0.0.1:9')
    with pytest.raises(gnc.DashboardError, match='docker compose up'):
        gnc.get_status()


def test_errors_reach_the_agent_over_mcp(gnc):
    """Only a ToolError's text is passed to the agent; other exceptions become a generic error."""
    from mcp.server.mcpserver.exceptions import ToolError
    with pytest.raises(ToolError, match='Unknown camera feed'):
        asyncio.run(gnc.server.call_tool('get_camera_image', {'feed': 'nope'}))
