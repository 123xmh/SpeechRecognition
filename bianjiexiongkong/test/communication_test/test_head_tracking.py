#!/usr/bin/env python3
"""
测试头动跟踪数据发送
"""
import socket
import struct
import time
import sys
import os

# 添加src目录到路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from vehicle_simulator import ProtocolHandler

def send_head_tracking_data():
    """发送头动跟踪数据到communication模块"""
    print("=== 发送头动跟踪数据测试 ===")
    
    # 创建协议处理器
    protocol = ProtocolHandler()
    
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5.0)
    
    try:
        # 创建模拟的头动跟踪数据包
        packet = bytearray(16)
        struct.pack_into('>H', packet, 0, 0x55AB)  # 帧头
        struct.pack_into('>H', packet, 2, 0x000A)  # 数据长度 (10字节)
        struct.pack_into('>i', packet, 4, 4500)    # 偏航角 * 100 (45.00度)
        struct.pack_into('>i', packet, 8, -1200)   # 俯仰角 * 100 (-12.00度)
        packet[12] = 1  # 跟踪状态
        packet[13] = 95  # 置信度
        
        # 计算CRC
        crc = protocol.crc.calculate(packet[:14])
        struct.pack_into('>H', packet, 14, crc)
        
        print(f"发送头动跟踪数据包到 127.0.0.1:8887")
        print(f"数据包: {packet.hex()}")
        
        # 发送到vehicle_simulator (8887端口)
        sock.sendto(bytes(packet), ('127.0.0.1', 8887))
        print("✓ 头动跟踪数据包发送成功")
        
        # 等待一下让vehicle_simulator处理
        time.sleep(1)
        
        return True
        
    except Exception as e:
        print(f"✗ 发送头动跟踪数据失败: {e}")
        return False
    finally:
        sock.close()

def main():
    """主函数"""
    print("头动跟踪数据发送测试")
    print("=" * 50)
    
    if send_head_tracking_data():
        print("🎉 头动跟踪数据发送测试成功！")
        print("请检查vehicle_simulator的日志输出和Web界面")
    else:
        print("⚠️  头动跟踪数据发送测试失败")

if __name__ == '__main__':
    main()
