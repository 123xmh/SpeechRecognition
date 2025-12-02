# /home/jinshuxin/TK30V2/tk_chest_controller_ws/src/video_source_loader/launch/video_source_loader.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('chest_interfaces'),
        'config',
        'video_sources.yaml'
    ])

    return LaunchDescription([
        Node(
            package='video_source_loader',
            executable='video_source_loader_node',
            name='video_source_loader',
            parameters=[{
                'config_file': config_file,
                'debug_output': True  # 设置为False可关闭调试输出
            }],
            output='screen'
        )
    ])