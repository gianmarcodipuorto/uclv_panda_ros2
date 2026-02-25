from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='panda_utils',
            executable='panda_gui_py',  # Note: .py extension for scripts
            name='panda_gui',
            output='screen',
            emulate_tty=True
        )
    ])