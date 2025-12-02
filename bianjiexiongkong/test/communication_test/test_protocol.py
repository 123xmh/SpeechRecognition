#!/usr/bin/env python3
"""
测试头盔通讯协议
"""
import socket
import time
import struct
import sys
import os

# 添加src目录到路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from vehicle_simulator import ProtocolHandler

def test_vehicle_status_packet():
    """测试车辆状态数据包创建和解析"""
    print("=== 测试车辆状态数据包 ===")
    
    # 创建协议处理器
    protocol = ProtocolHandler()
    
    # 创建车辆状态数据包
    packet = protocol.create_vehicle_status_packet(platform_id=0)
    print(f"车辆状态数据包长度: {len(packet)} 字节")
    print(f"数据包内容: {packet.hex()}")
    
    # 验证数据包结构
    if len(packet) == 59:
        print("✓ 数据包长度正确 (59字节)")
    else:
        print(f"✗ 数据包长度错误: {len(packet)} 字节")
        return False
    
    # 验证帧头
    header = struct.unpack('>H', packet[0:2])[0]
    if header == 0x55AE:
        print("✓ 帧头正确 (0x55AE)")
    else:
        print(f"✗ 帧头错误: 0x{header:04X}")
        return False
    
    # 验证数据长度
    length = struct.unpack('>H', packet[2:4])[0]
    if length == 0x0037:
        print("✓ 数据长度正确 (0x0037)")
    else:
        print(f"✗ 数据长度错误: 0x{length:04X}")
        return False
    
    # 验证CRC
    calculated_crc = protocol.crc.calculate(packet[:57])
    received_crc = struct.unpack('>H', packet[57:59])[0]
    if calculated_crc == received_crc:
        print("✓ CRC校验正确")
    else:
        print(f"✗ CRC校验错误: 计算值=0x{calculated_crc:04X}, 接收值=0x{received_crc:04X}")
        return False
    
    return True

def test_head_tracking_packet():
    """测试头动跟踪数据包解析"""
    print("\n=== 测试头动跟踪数据包 ===")
    
    protocol = ProtocolHandler()
    
    # 创建模拟的头动跟踪数据包
    packet = bytearray(16)
    struct.pack_into('>H', packet, 0, 0x55AB)  # 帧头
    struct.pack_into('>H', packet, 2, 0x000C)  # 数据长度
    struct.pack_into('>i', packet, 4, 4500)    # 偏航角 * 100
    struct.pack_into('>i', packet, 8, -1200)   # 俯仰角 * 100
    packet[12] = 1  # 跟踪状态
    packet[13] = 95  # 置信度
    
    # 计算CRC
    crc = protocol.crc.calculate(packet[:14])
    struct.pack_into('>H', packet, 14, crc)
    
    print(f"头动跟踪数据包: {packet.hex()}")
    
    # 解析数据包
    result = protocol.parse_head_tracking_packet(bytes(packet))
    if result:
        print("✓ 头动跟踪数据包解析成功")
        print(f"  偏航角: {result['yaw']:.2f}°")
        print(f"  俯仰角: {result['pitch']:.2f}°")
        print(f"  跟踪状态: {result['tracking_status']}")
        print(f"  置信度: {result['confidence']}%")
        return True
    else:
        print("✗ 头动跟踪数据包解析失败")
        return False

def test_voice_command_packet():
    """测试语音指令数据包解析"""
    print("\n=== 测试语音指令数据包 ===")
    
    protocol = ProtocolHandler()
    
    # 创建模拟的语音指令数据包 (按照协议格式)
    packet = bytearray(21)
    struct.pack_into('>H', packet, 0, 0x55AA)  # 帧头
    struct.pack_into('>H', packet, 2, 0x000F)  # 数据长度 (15字节)
    packet[4] = 1  # 类别
    packet[5] = 2  # 操作
    struct.pack_into('>I', packet, 6, 12345)   # 命令ID
    struct.pack_into('>i', packet, 10, 100)    # 参数1 (int32_t)
    struct.pack_into('>i', packet, 14, 200)    # 参数2 (int32_t)
    packet[18] = 1  # 参数3
    
    # 计算CRC
    crc = protocol.crc.calculate(packet[:19])
    struct.pack_into('>H', packet, 19, crc)
    
    print(f"语音指令数据包: {packet.hex()}")
    
    # 解析数据包
    result = protocol.parse_voice_command_packet(bytes(packet))
    if result:
        print("✓ 语音指令数据包解析成功")
        print(f"  命令ID: {result['command_id']}")
        print(f"  类别: {result['category']}")
        print(f"  操作: {result['operation']}")
        print(f"  参数1: {result['param1']}")
        print(f"  参数2: {result['param2']}")
        print(f"  参数3: {result['param3']}")
        
        # 测试确认包创建
        ack_packet = protocol.create_ack_packet(result['command_id'], 0x01)
        print(f"确认包: {ack_packet.hex()}")
        print("✓ 确认包创建成功")
        return True
    else:
        print("✗ 语音指令数据包解析失败")
        return False

def test_udp_communication():
    """测试UDP通信"""
    print("\n=== 测试UDP通信 ===")
    
    try:
        # 创建UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(5.0)
        
        # 发送车辆状态数据包到communication模块
        protocol = ProtocolHandler()
        packet = protocol.create_vehicle_status_packet()
        
        print(f"发送车辆状态数据包到 127.0.0.1:8888")
        sock.sendto(packet, ('127.0.0.1', 8888))
        print("✓ 车辆状态数据包发送成功")
        
        # 尝试接收响应
        try:
            data, addr = sock.recvfrom(1024)
            print(f"✓ 接收到响应: {len(data)} 字节来自 {addr}")
            print(f"响应内容: {data.hex()}")
        except socket.timeout:
            print("! 未接收到响应 (超时)")
        
        sock.close()
        return True
        
    except Exception as e:
        print(f"✗ UDP通信测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("头盔通讯协议测试")
    print("=" * 50)
    
    tests = [
        test_vehicle_status_packet,
        test_head_tracking_packet,
        test_voice_command_packet,
        test_udp_communication
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"✗ 测试异常: {e}")
    
    print("\n" + "=" * 50)
    print(f"测试结果: {passed}/{total} 通过")
    
    if passed == total:
        print("🎉 所有测试通过！")
    else:
        print("⚠️  部分测试失败")

if __name__ == '__main__':
    main()
