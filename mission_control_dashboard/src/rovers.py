"""
The rover the stack drives, chosen on the menu's (hub, 5000) ROVER selector.

Each rover-dependent package keeps its values in its own config/params.yaml
(Leo_Raph, the default) and config/params_<rover>.yaml (that rover's
overrides); GNC_ROVER (.env) picks one when the containers start. A switch from
the dashboard applies another rover live, from the same files (mounted
read-only into the dashboard's container, see docker-compose.yml):

  1. the rover node's drive calibration (applied live: re-configuring it would
     drop Unreal's TCP link, which Unreal does not redial);
  2. the planner's footprint and limits;
  3. robot_state_publisher's robot_description (urdf/<rover>.urdf), so /tf_static
     carries the new rover's frames;
  4. the dashboard's own values (lidar height, self radius, map icon, teleop limit);
  5. a restart (re-configure) of the planner and local mapping, which read
     their sensor mounts from TF and build their limits when configured.

Unreal must run the rover's level: the viewport-camera link (rover_cameras.py)
reports the rover in Play, and the menu and the overview warn when they differ. The switch
lasts until the containers restart; .env decides at start.
"""

import glob
import os

import yaml

# Rover -> display name, Unreal level and Blueprint class (the class name the
# Unreal link reports starts with it).
ROVERS = {
    'leo_raph': {'label': 'Leo_Raph', 'level': 'Lunar_Terrain_Raph', 'blueprint': 'Leo_Raph',
                 'summary': '4 wheels · 0.6 m · sensors 0.55 m up · max turn 0.35 rad/s'},
    'yutu': {'label': 'Yutu', 'level': 'Lunar_Terrain_Yutu', 'blueprint': 'Yutu_Rover',
             'summary': '6 wheels · 1.9 m · mast sensors 1.13 m up · max turn 0.15 rad/s'},
}
DEFAULT_ROVER = 'leo_raph'

# Rover-dependent packages: package -> node whose parameters it configures.
PACKAGES = {
    'component_rover_leo_raph': 'lc_component_rover_leo_raph',
    'component_path_planner': 'lc_component_path_planner',
    'mission_control_dashboard': 'mission_control_dashboard',
}
CONFIG_ROOT = '/opt/gnc/rovers'                       # read-only mounts (docker-compose.yml)
CONFIG_DIRS = {
    'component_rover_leo_raph': f'{CONFIG_ROOT}/component_rover_leo_raph',
    'component_path_planner': f'{CONFIG_ROOT}/component_path_planner',
    'mission_control_dashboard': '/opt/ros/jazzy/src/mission_control_dashboard/config',
}
URDF_DIR = f'{CONFIG_ROOT}/urdf'
SKIP = {'use_sim_time'}


def _parameters(path: str, node: str) -> dict:
    try:
        with open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
    except OSError:
        return {}
    return dict((data.get(node) or {}).get('ros__parameters') or {})


def profile(rover: str, config_dirs: dict = None, packages: dict = None) -> dict:
    """
    {node: {parameter: value}} that make `rover` the active one: every parameter
    any rover overrides, with this rover's value (its override, else params.yaml).
    """
    config_dirs = CONFIG_DIRS if config_dirs is None else config_dirs
    packages = PACKAGES if packages is None else packages
    out = {}
    for package, node in packages.items():
        directory = config_dirs.get(package)
        if not directory:
            continue
        base = _parameters(os.path.join(directory, 'params.yaml'), node)
        keys = set()
        for path in glob.glob(os.path.join(directory, 'params_*.yaml')):
            keys |= set(_parameters(path, node))
        own = _parameters(os.path.join(directory, f'params_{rover}.yaml'), node)
        values = {k: own.get(k, base.get(k)) for k in sorted(keys - SKIP)}
        values = {k: v for k, v in values.items() if v is not None}
        if values:
            out[node] = values
    return out


def urdf(rover: str, urdf_dir: str = URDF_DIR):
    """The rover's URDF text, or None if it has none."""
    try:
        with open(os.path.join(urdf_dir, f'{rover}.urdf'), encoding='utf-8') as handle:
            return handle.read()
    except OSError:
        return None


def rover_of_class(class_name):
    """The rover key of an Unreal rover class name ('Yutu_Rover_C' -> 'yutu'), or None."""
    if not class_name:
        return None
    for key, info in ROVERS.items():
        if str(class_name).startswith(info['blueprint']):
            return key
    return None


def status(active: str, unreal_class, linked: bool, busy: bool) -> dict:
    """What the overview shows: the active rover, Unreal's, and whether they match."""
    unreal = rover_of_class(unreal_class) if linked else None
    return {
        'active': active, 'label': ROVERS.get(active, {}).get('label', active),
        'level': ROVERS.get(active, {}).get('level'),
        'rovers': [{'key': k, 'label': v['label'], 'level': v['level'], 'summary': v.get('summary', '')}
                   for k, v in ROVERS.items()],
        'unreal': unreal, 'unreal_label': ROVERS.get(unreal, {}).get('label') if unreal else None,
        'unreal_linked': linked, 'match': unreal is None or unreal == active, 'busy': busy,
    }
