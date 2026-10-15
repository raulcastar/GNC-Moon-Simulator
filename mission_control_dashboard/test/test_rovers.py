"""The rover switch's profiles: each rover's values come from the packages' own config files."""
import os

import yaml

from src.rovers import profile, rover_of_class, status, urdf


def write(path, node, params):
    with open(path, 'w', encoding='utf-8') as handle:
        yaml.safe_dump({node: {'ros__parameters': params}}, handle)


def configs(tmp_path):
    rover_dir, planner_dir = tmp_path / 'rover', tmp_path / 'planner'
    rover_dir.mkdir()
    planner_dir.mkdir()
    write(rover_dir / 'params.yaml', 'lc_rover', {'use_sim_time': True, 'wheel_speed_scale': 1.82,
                                                 'rover_width': 89.0, 'tcp_port': 3003})
    write(rover_dir / 'params_yutu.yaml', 'lc_rover', {'wheel_speed_scale': 0.96, 'rover_width': 330.0})
    write(planner_dir / 'params.yaml', 'lc_planner', {'robot_width': 0.65, 'max_yaw_rate_deg': 12.0})
    write(planner_dir / 'params_yutu.yaml', 'lc_planner', {'robot_width': 1.15, 'max_yaw_rate_deg': 8.0})
    return ({'rover_pkg': str(rover_dir), 'planner_pkg': str(planner_dir)},
            {'rover_pkg': 'lc_rover', 'planner_pkg': 'lc_planner'})


def test_another_rover_gets_its_overrides(tmp_path):
    dirs, packages = configs(tmp_path)
    p = profile('yutu', dirs, packages)
    assert p == {'lc_rover': {'rover_width': 330.0, 'wheel_speed_scale': 0.96},
                 'lc_planner': {'max_yaw_rate_deg': 8.0, 'robot_width': 1.15}}


def test_the_default_rover_gets_back_the_base_values_of_the_overridden_keys_only(tmp_path):
    dirs, packages = configs(tmp_path)
    p = profile('leo_raph', dirs, packages)
    assert p == {'lc_rover': {'rover_width': 89.0, 'wheel_speed_scale': 1.82},
                 'lc_planner': {'max_yaw_rate_deg': 12.0, 'robot_width': 0.65}}
    assert 'tcp_port' not in p['lc_rover'] and 'use_sim_time' not in p['lc_rover']


def test_urdf_and_unreal_class_names(tmp_path):
    (tmp_path / 'yutu.urdf').write_text('<robot name="yutu_unreal"/>')
    assert urdf('yutu', str(tmp_path)) == '<robot name="yutu_unreal"/>'
    assert urdf('nope', str(tmp_path)) is None
    assert rover_of_class('Yutu_Rover_C') == 'yutu'
    assert rover_of_class('Leo_Raph_C') == 'leo_raph'
    assert rover_of_class('none') is None


def test_status_flags_a_mismatch_with_unreal_only_when_linked():
    assert status('leo_raph', 'Yutu_Rover_C', linked=True, busy=False)['match'] is False
    assert status('yutu', 'Yutu_Rover_C', linked=True, busy=False)['match'] is True
    assert status('leo_raph', 'Yutu_Rover_C', linked=False, busy=False)['match'] is True


def test_the_menu_reads_and_switches_the_rover():
    """The hub's /rover: GET the status, POST {rover} to switch (a stub node, nothing reaches ROS)."""
    from src import routes, web  # noqa: F401  (registers the routes)

    class Node:
        def __init__(self):
            self.active, self.selected = 'yutu', []

        def rover_status(self):
            return status(self.active, None, linked=False, busy=False)

        def select_rover(self, rover):
            self.selected.append(rover)

    saved, web.node = web.node, Node()
    try:
        client = web.app_hub.test_client()
        assert client.get('/rover').get_json()['active'] == 'yutu'
        client.post('/rover', json={'rover': 'leo_raph'})
        assert web.node.selected == ['leo_raph']
    finally:
        web.node = saved
