from rclpy.node import Node
from chest_interfaces.msg import VoiceText, VoiceCommand
from std_msgs.msg import Header

class VoiceResultPublisher(Node):
    def __init__(self):
        super().__init__('voice_result_publisher')
        # 创建发布者 - 使用现有的消息类型
        self.voice_text_pub = self.create_publisher(
            VoiceText, 
            '/voice_processing_text', 
            10
        )
        self.voice_command_pub = self.create_publisher(
            VoiceCommand, 
            '/voice_processing_command', 
            10
        )
        self.get_logger().info('已启动语音识别结果发布节点')

    def publish_text_result(self, raw_text, is_final=False):
        """发布语音识别文本结果"""
        # 创建消息头
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "voice_recognition"
        
        # 创建语音文本消息
        msg = VoiceText()
        msg.header = header
        msg.raw_text = raw_text
        msg.is_final = is_final
        
        # 发布消息
        self.voice_text_pub.publish(msg)
        status = "最终" if is_final else "中间"
        self.get_logger().info(f'发布语音文本: "{raw_text}" ({status}结果)')

    def publish_command_result(self, category, operation, command_id, param1=0, param2=0, param3=0, is_valid=True, command_text=""):
        """发布语音指令结果 - 符合C++消息格式"""
        # 创建消息头
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "voice_command"
        
        # 创建语音指令消息
        msg = VoiceCommand()
        msg.header = header
        
        # 按照C++消息格式设置字段
        msg.frame_header = (category << 8) | operation  # 前8位分类码，后8位操作码
        msg.command_id = command_id
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.is_valid = is_valid
        
        # 发布消息
        self.voice_command_pub.publish(msg)
        
        status = "有效" if is_valid else "无效"
        param_info = f" 参数: p1={param1}, p2={param2}, p3={param3}" if any([param1, param2, param3]) else ""
        self.get_logger().info(f'发布语音指令: 分类={category:02X}, 操作={operation:02X}, ID={command_id} ({status}){param_info}')
        
        return msg