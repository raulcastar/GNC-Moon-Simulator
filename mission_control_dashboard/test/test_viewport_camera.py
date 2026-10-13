"""The Unreal viewport camera requested on the dashboard (viewport_camera)."""

from src.viewport_camera import LINK_TIMEOUT_S, ViewportCamera


def test_requests_only_known_modes():
    v = ViewportCamera()
    assert v.request('orbit') and v.mode == 'orbit'
    assert v.request('top') and v.mode == 'top'
    assert not v.request('drone') and v.mode == 'top'


def test_unreal_polls_link_it_and_report_what_it_shows():
    v = ViewportCamera()
    assert v.status(now=100.0) == {'mode': 'follow', 'applied': None, 'linked': False}
    v.request('orbit')
    assert v.poll('follow', now=100.0) == {'mode': 'orbit'}
    assert v.status(now=100.5) == {'mode': 'orbit', 'applied': 'follow', 'linked': True}
    v.poll('orbit', now=101.0)
    assert v.status(now=101.2)['applied'] == 'orbit'


def test_link_is_lost_without_polls():
    v = ViewportCamera()
    v.poll('orbit', now=10.0)
    assert not v.status(now=10.0 + LINK_TIMEOUT_S + 0.1)['linked']
    assert v.status(now=10.0 + LINK_TIMEOUT_S + 0.1)['applied'] is None
