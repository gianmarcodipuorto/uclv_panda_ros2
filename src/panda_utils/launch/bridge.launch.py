"""Starts one bridge without the GUI: it configures it and then activates it.

    ros2 launch panda_utils bridge.launch.py bridge:=joint_velocities
    ros2 launch panda_utils bridge.launch.py bridge:=cartesian_position params_file:=/path/to/my.yaml
    ros2 launch panda_utils bridge.launch.py bridge:=joint_positions robot_ip:=172.16.0.2 mode:=internal

Arguments: bridge (joint_positions, joint_velocities, joint_torques, cartesian_position,
cartesian_velocity), params_file (YAML of parameters, default config/bridge.yaml), activate (false:
configure only), robot_ip and mode (override the file). Ctrl+C stops the robot smoothly, then the
bridge exits.
"""
import os

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

BRIDGES = ['joint_positions', 'joint_velocities', 'joint_torques', 'cartesian_position', 'cartesian_velocity']


def change_state(node, transition_id):
    return EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node), transition_id=transition_id))


def bridge_actions(context):
    # The executable and the node have the same name, as the GUI expects
    name = f"panda_{LaunchConfiguration('bridge').perform(context)}_bridge"
    overrides = {key: LaunchConfiguration(key).perform(context) for key in ('robot_ip', 'mode')}
    overrides = {key: value for key, value in overrides.items() if value}
    node = LifecycleNode(package='panda_utils', executable=name, name=name, namespace='',
                         parameters=[LaunchConfiguration('params_file').perform(context), overrides],
                         output='screen', emulate_tty=True)
    actions = [node, change_state(node, lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE)]
    if LaunchConfiguration('activate').perform(context).lower() in ('true', '1', 'yes'):
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
            entities=[change_state(node, lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE)])))
    return actions


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('panda_utils'), 'config', 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('bridge', default_value='joint_positions', choices=BRIDGES,
                              description='Which bridge to start'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML file with the parameters of the bridge'),
        DeclareLaunchArgument('activate', default_value='true',
                              description='Activate after configuring (false: configure only)'),
        DeclareLaunchArgument('robot_ip', default_value='', description='Robot address, overrides the file'),
        DeclareLaunchArgument('mode', default_value='', description='external or internal, overrides the file'),
        OpaqueFunction(function=bridge_actions),
    ])
