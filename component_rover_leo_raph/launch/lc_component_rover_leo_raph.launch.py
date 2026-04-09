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
        get_package_share_directory('component_rover_leo_raph'),
        'config',
        'params.yaml'
    )

    rover_node = launch_ros.actions.LifecycleNode(package='component_rover_leo_raph', 
        executable='lc_component_rover_leo_raph', name='lc_component_rover_leo_raph', namespace='', output='screen', emulate_tty=True, respawn=True,
        arguments=['--ros-args', '--log-level', "info"], parameters = [yaml_config])
    
    # Make the cameras node take the 'configure' transition
    rover_node_configure_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(rover_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the cameras node node take the 'activate' transition
    rover_node_trans_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(rover_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    active_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = rover_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [rover_node_trans_event],
        )
    )

    
    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Start Nodes
        rover_node,

        #activate system
        rover_node_configure_event,
        active_state_handler,
    ])