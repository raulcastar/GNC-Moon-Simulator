"""The overview's original and re-planned routes (telemetry.track_route)."""

from src.telemetry import track_route

EMPTY = {'goal': None, 'first': [], 'current': [], 'replans': 0}
A = [{'x': 0.0, 'y': 0.0}, {'x': 5.0, 'y': 0.0}]
B = [{'x': 0.0, 'y': 0.0}, {'x': 2.0, 'y': 2.0}, {'x': 5.0, 'y': 0.0}]
C = [{'x': 1.0, 'y': 0.0}, {'x': 5.0, 'y': 1.0}]


def test_first_route_for_a_goal():
    route = track_route(EMPTY, A, [5.0, 0.0])
    assert route == {'goal': [5.0, 0.0], 'first': A, 'current': A, 'replans': 0}


def test_republished_route_is_not_a_replan():
    route = track_route(track_route(EMPTY, A, [5.0, 0.0]), A, [5.0, 0.0])
    assert route['replans'] == 0


def test_a_different_route_for_the_same_goal_is_a_replan():
    route = track_route(EMPTY, A, [5.0, 0.0])
    route = track_route(route, B, [5.0, 0.0])
    assert route['replans'] == 1 and route['first'] == A and route['current'] == B
    route = track_route(route, B, [5.0, 0.0])
    assert route['replans'] == 1
    route = track_route(route, C, [5.0, 0.0])
    assert route['replans'] == 2 and route['first'] == A


def test_a_new_goal_starts_over():
    route = track_route(track_route(EMPTY, A, [5.0, 0.0]), B, [5.0, 0.0])
    route = track_route(route, C, [5.0, 1.0])
    assert route == {'goal': [5.0, 1.0], 'first': C, 'current': C, 'replans': 0}


def test_an_empty_route_clears():
    route = track_route(track_route(EMPTY, A, [5.0, 0.0]), B, [5.0, 0.0])
    assert track_route(route, [], [5.0, 0.0]) == EMPTY
