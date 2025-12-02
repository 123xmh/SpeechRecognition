from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='web_monitor',
            executable='/home/jinshuxin/miniconda3/envs/ros2env/bin/python',
            arguments=['-m', 'web_monitor.web_monitor_node'],
            name='web_monitor_node',
            output='screen'
        ),
    ])