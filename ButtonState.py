from chest_interfaces.msg import ButtonState
from rclpy.node import Node
import time

class ButtonStateSubscriber(Node):
    def __init__(self, recording_flag):
        super().__init__('button_state_subscriber')
        # 订阅正确的话题名称
        self.subscription = self.create_subscription(
            ButtonState,
            '/tk_chest/button/state',  # 改为正确的话题名称
            self.listener_callback,
            10)
        self.subscription
        self.recording_flag = recording_flag
        
        self.get_logger().info('已启动按钮状态订阅节点，等待侧按键2的状态...')
        
    def listener_callback(self, msg):
        try:
            # 使用正确的字段名：voice_command_enabled
            voice_command_enabled = msg.voice_command_enabled
            
            # 更新录音标志
            self.recording_flag[0] = voice_command_enabled
                
        except Exception as e:
            self.get_logger().error(f'处理按钮状态消息时出错: {e}')