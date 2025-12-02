from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 头动解算模块
        Node(
            package='head_tracking',
            executable='head_tracking_node',
            output='screen',
            parameters=[{'simulate_imu': True}]
        ),
        
        # 语音处理模块
        Node(
            package='voice_processing',
            executable='voice_processing_node',
            output='screen',
            parameters=[{'simulate_microphone': True}]
        ),
        
        # AR显示模块
        Node(
            package='ar_display',
            executable='ar_display_node',
            output='screen'
        ),
        
        # 通讯模块
        Node(
            package='communication',
            executable='communication_node',
            output='screen',
            parameters=[
                {"remote_ip": "127.0.0.1"},
                {"remote_port": 5000},
                {"local_port": 5001}
            ]
        ),
        
        # 按键模拟器
        Node(
            package='button_simulator',
            executable='button_simulator_node',
            output='screen'
        ),
        
        # 按键控制器
        Node(
            package='button_controller',
            executable='button_controller_node',
            output='screen'
        )
    ])


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # 头动解算模块
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('head_tracking'),
                    'launch',
                    'head_tracking.launch.py'
                ])
            ])
        ),
        
        # 语音处理模块
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('voice_processing'),
                    'launch',
                    'voice_processing.launch.py'
                ])
            ])
        ),
        
        # AR显示模块
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ar_display'),
                    'launch',
                    'ar_display.launch.py'
                ])
            ])
        ),
        
        # 通讯模块
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('communication'),
                    'launch',
                    'communication.launch.py'
                ])
            ])
        ),
        
        # 按键模拟器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('button_simulator'),
                    'launch',
                    'button_simulator.launch.py'
                ])
            ])
        ),
        
        # 视频源加载器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('video_source_loader'),
                    'launch',
                    'video_source_loader.launch.py'
                ])
            ])
        ),
    ])