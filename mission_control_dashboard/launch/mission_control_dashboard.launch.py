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

def rover_parameters(package):
    """params.yaml, then params_<GNC_ROVER>.yaml (the rover's overrides) when it exists."""
    share = get_package_share_directory(package)
    files = [os.path.join(share, 'config', 'params.yaml')]
    override = os.path.join(share, 'config', f"params_{os.environ.get('GNC_ROVER', 'leo_raph')}.yaml")
    if os.path.exists(override):
        files.append(override)
    return files

def generate_launch_description():

    dashboard_node = launch_ros.actions.LifecycleNode(package='mission_control_dashboard', 
        executable='mission_control_dashboard', name='mission_control_dashboard', namespace='', output='screen', emulate_tty=True, respawn=True,
        arguments=['--ros-args', '--log-level', "info"], parameters=rover_parameters('mission_control_dashboard'))
    
    # Make the cameras node take the 'configure' transition
    dashboard_node_configure_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(dashboard_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the cameras node node take the 'activate' transition
    dashboard_node_trans_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(dashboard_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    active_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = dashboard_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [dashboard_node_trans_event],
        )
    )

    
    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Start Nodes
        dashboard_node,

        #activate system
        dashboard_node_configure_event,
        active_state_handler,
    ])