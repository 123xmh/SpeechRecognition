from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug output'
        ),
        DeclareLaunchArgument(
            'simulate_microphone',
            default_value='true',
            description='Simulate microphone input'
        ),
        Node(
            package='voice_processing',
            executable='voice_processing_node',
            name='voice_processing',
            output='screen',
            parameters=[{
                'simulate_microphone': LaunchConfiguration('simulate_microphone'),
                'debug': LaunchConfiguration('debug')
            }]
        )
    ])