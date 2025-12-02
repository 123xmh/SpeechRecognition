from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_display',
            executable='controller_display_node',
            name='controller_display',
            output='screen',
            emulate_tty=True
        )
    ])