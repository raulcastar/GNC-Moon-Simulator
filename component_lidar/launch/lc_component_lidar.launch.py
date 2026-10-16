import os
import launch
import launch_ros
import lifecycle_msgs
import launch.actions
import launch.events
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    yaml_config = os.path.join(
        get_package_share_directory('component_lidar'),
        'config',
        'params.yaml'
    )

    lidar_node = launch_ros.actions.LifecycleNode(package='component_lidar',
        executable='lc_component_lidar', name='lc_component_lidar', namespace='', output='screen', emulate_tty=True, respawn=True,
        arguments=['--ros-args', '--log-level', "info"], parameters = [yaml_config])

    # Configure the node at launch
    lidar_node_configure_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(lidar_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate it once configured
    lidar_node_activate_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(lidar_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    active_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = lidar_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [lidar_node_activate_event],
        )
    )


    return LaunchDescription([
        # Coloured console output.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        lidar_node,

        lidar_node_configure_event,
        active_state_handler,
    ])