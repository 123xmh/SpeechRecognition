#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from chest_interfaces.msg import ButtonState
import time

class ButtonDebugSubscriber(Node):
    def __init__(self):
        super().__init__('button_debug_subscriber')
        self.subscription = self.create_subscription(
            ButtonState,
            '/button_state',
            self.listener_callback,
            10)
        self.message_count = 0
        self.get_logger().info('按钮调试订阅器已启动...')
        
    def listener_callback(self, msg):
        self.message_count += 1
        print(f"\n=== 收到第 {self.message_count} 条按钮消息 ===")
        print(f"时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        print(f"语音开关 (voice_command_enabled): {msg.voice_command_enabled}")
        print(f"头动开关 (head_tracking_enabled): {msg.head_tracking_enabled}")
        print(f"正面屏开关 (front_screen_enabled): {msg.front_screen_enabled}")
        print(f"头显开关 (hmd_enabled): {msg.hmd_enabled}")
        print("=== 消息结束 ===\n")

def main():
    rclpy.init()
    node = ButtonDebugSubscriber()
    
    print("等待按钮状态消息...")
    print("按 Ctrl+C 退出")
    
    try:
        # 运行60秒或直到收到Ctrl+C
        start_time = time.time()
        while time.time() - start_time < 60:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f"总共收到 {node.message_count} 条消息")

if __name__ == '__main__':
    main()