from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='head_tracking',
            executable='head_tracking_node',
            name='head_tracking',
            output='screen',
            parameters=[{
                'simulate_imu': True  # 启用IMU模拟
            }]
        )
    ])