"""RViz with the robot, the Franka Hand and, optionally, the rest of the cell.

    ros2 launch panda_utils rviz.launch.py
    ros2 launch panda_utils rviz.launch.py scene:=/path/to/my_cell.urdf.xacro rviz_config:=/path/to/my.rviz

The joints come from the running bridge (/joint_states) and from the gripper (/panda/gripper_state).
The scene is a xacro (or URDF) whose root link is fer_link0, the base of the robot: every object hangs
from it with a fixed joint (see config/scene_example.urdf.xacro). The robot model comes from
franka_description, cloned into the workspace (see the README). The nodes live in the namespace
/panda_view and never publish on the topics of the driver.
"""
import os

import xacro
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'panda_view'
# Without a scene: only the base of the robot, nothing hangs from it
EMPTY_SCENE = '<?xml version="1.0"?><robot name="cell"><link name="fer_link0"/></robot>'


def view_actions(context):
    try:
        franka_description = get_package_share_directory('franka_description')
    except PackageNotFoundError:
        raise RuntimeError('RViz needs franka_description: clone it into the src folder of the workspace and '
                           'build it (see the README)') from None
    robot = xacro.process_file(os.path.join(franka_description, 'robots', 'fer', 'fer.urdf.xacro'),
                               mappings={'hand': LaunchConfiguration('hand').perform(context)}).toxml()
    scene_file = os.path.expanduser(LaunchConfiguration('scene').perform(context))
    scene = xacro.process_file(scene_file).toxml() if scene_file else EMPTY_SCENE
    rviz_config = os.path.expanduser(LaunchConfiguration('rviz_config').perform(context)) or os.path.join(
        get_package_share_directory('panda_utils'), 'config', 'panda.rviz')
    return [
        # Arm (from the bridge) and fingers (from the gripper) in one stream, /panda_view/joint_states
        Node(package='joint_state_publisher', executable='joint_state_publisher', namespace=NAMESPACE,
             parameters=[{'source_list': ['/joint_states', '/panda/gripper_state'], 'rate': 50}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=NAMESPACE,
             parameters=[{'robot_description': robot}]),
        # The rest of the cell: fixed joints only, published once as static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='scene_state_publisher',
             namespace=NAMESPACE, parameters=[{'robot_description': scene}],
             remappings=[('robot_description', 'scene_description')]),
        Node(package='rviz2', executable='rviz2', namespace=NAMESPACE, arguments=['-d', rviz_config]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='',
                              description='xacro or URDF of the rest of the cell, rooted at fer_link0 (empty: none)'),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='RViz configuration (empty: config/panda.rviz of the package)'),
        DeclareLaunchArgument('hand', default_value='true', description='Draw the Franka Hand'),
        OpaqueFunction(function=view_actions),
    ])
