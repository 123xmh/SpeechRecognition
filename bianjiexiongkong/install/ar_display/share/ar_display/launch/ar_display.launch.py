from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_display',
            executable='ar_display_node',
            name='ar_display_node',
            output='screen'
        )
    ])