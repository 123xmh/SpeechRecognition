from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'button_simulator', 'button_simulator_node'],
            output='screen'
        )
    ])