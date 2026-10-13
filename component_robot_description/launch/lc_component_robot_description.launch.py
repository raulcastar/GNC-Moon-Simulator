"""
The rover's frames: robot_state_publisher publishes every fixed joint of
urdf/leo_raph.urdf (exported from the Unreal Blueprint) on /tf_static.
odom -> base_link comes from component_rover_leo_raph.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('component_robot_description'), 'urdf', 'leo_raph.urdf')
    with open(urdf) as handle:
        description = handle.read()

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[{'robot_description': description, 'use_sim_time': True}],
        ),
    ])
