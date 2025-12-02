#!/usr/bin/env python3
"""
监控UDP通信流量
"""
import socket
import struct
import time
import sys
import os

# 添加src目录到路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from vehicle_simulator import ProtocolHandler

def monitor_udp_traffic():
    """监控UDP流量"""
    print("=== UDP流量监控 ===")
    
    # 创建协议处理器
    protocol = ProtocolHandler()
    
    # 创建UDP socket监听8887端口
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 8887))
    sock.settimeout(1.0)
    
    print("监听8887端口，等待来自communication模块的数据...")
    print("按Ctrl+C停止监控")
    
    try:
        while True:
            try:
                # 接收数据
                data, addr = sock.recvfrom(1024)
                
                # 解析数据包类型
                if len(data) >= 2:
                    header = struct.unpack('>H', data[0:2])[0]
                    
                    if header == 0x55AB:  # 头动跟踪数据
                        parsed_data = protocol.parse_head_tracking_packet(data)
                        if parsed_data:
                            print(f"[{time.strftime('%H:%M:%S')}] 接收到头动跟踪数据:")
                            print(f"  来源: {addr}")
                            print(f"  偏航: {parsed_data['yaw']:.2f}°")
                            print(f"  俯仰: {parsed_data['pitch']:.2f}°")
                            print(f"  跟踪状态: {parsed_data['tracking_status']}")
                            print(f"  置信度: {parsed_data['confidence']}%")
                            print(f"  数据包: {data.hex()}")
                            print("-" * 50)
                    
                    elif header == 0x55AC:  # 语音文本数据
                        print(f"[{time.strftime('%H:%M:%S')}] 接收到语音文本数据: {data.hex()}")
                    
                    elif header == 0x55AA:  # 语音指令数据
                        print(f"[{time.strftime('%H:%M:%S')}] 接收到语音指令数据: {data.hex()}")
                    
                    else:
                        print(f"[{time.strftime('%H:%M:%S')}] 接收到未知数据包: {data.hex()}")
                
            except socket.timeout:
                # 超时，继续等待
                continue
            except Exception as e:
                print(f"接收数据错误: {e}")
                break
                
    except KeyboardInterrupt:
        print("\n监控停止")
    finally:
        sock.close()

def main():
    """主函数"""
    print("UDP通信流量监控工具")
    print("=" * 50)
    monitor_udp_traffic()

if __name__ == '__main__':
    main()
