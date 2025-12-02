from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(get_package_share_directory('communication'), 'config')
    config_file = os.path.join(config_dir, 'communication_config.yaml')
    
    return LaunchDescription([
        Node(
            package='communication',
            executable='communication_node',
            name='communication_node',
            output='screen',
            parameters=[config_file]
        )
    ])