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
        get_package_share_directory('component_path_planner'),
        'config',
        'params.yaml'
    )

    path_planner_node = launch_ros.actions.LifecycleNode(package='component_path_planner', 
        executable='lc_component_path_planner', name='lc_component_path_planner', namespace='', output='screen', emulate_tty=True, respawn=True,
        arguments=['--ros-args', '--log-level', "info"], parameters = [yaml_config])
    
    # Make the path planner node take the 'configure' transition
    path_planner_node_configure_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(path_planner_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the path planner node take the 'activate' transition
    path_planner_node_trans_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(path_planner_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    active_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = path_planner_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [path_planner_node_trans_event],
        )
    )

    
    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Start Nodes
        path_planner_node,

        #activate system
        path_planner_node_configure_event,
        active_state_handler,
    ])