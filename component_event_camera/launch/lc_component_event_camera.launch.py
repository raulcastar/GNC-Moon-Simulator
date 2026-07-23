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
        get_package_share_directory('component_event_camera'),
        'config',
        'params.yaml'
    )

    event_node = launch_ros.actions.LifecycleNode(package='component_event_camera',
        executable='lc_component_event_camera', name='lc_component_event_camera', namespace='', output='screen', emulate_tty=True, respawn=True,
        arguments=['--ros-args', '--log-level', "info"], parameters=[yaml_config])

    # Make the node take the 'configure' transition
    event_node_configure_event = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(event_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the node take the 'activate' transition
    event_node_activate_event = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(event_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    active_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=event_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[event_node_activate_event],
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        event_node,
        event_node_configure_event,
        active_state_handler,
    ])
