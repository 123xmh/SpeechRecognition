#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
车辆节点模拟器
模拟车辆节点，向头盔节点发送车辆状态数据，接收头盔节点的各种数据并发送确认
用于测试头盔节点的communication模块
"""

import asyncio
import json
import socket
import struct
import time
import threading
import yaml
import os
import random
from datetime import datetime
from typing import Dict, Any, Optional
import aiohttp
from aiohttp import web, WSMsgType
from aiohttp.web import WebSocketResponse
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)

class CRC16MODBUS:
    """CRC-16/MODBUS校验算法实现"""
    
    def __init__(self):
        self.poly = 0x8005
        self.init_value = 0xFFFF
        self.table = self._generate_table()
    
    def _generate_table(self):
        """生成CRC查找表"""
        table = []
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ self.poly
                else:
                    crc >>= 1
            table.append(crc & 0xFFFF)
        return table
    
    def calculate(self, data):
        """计算CRC-16/MODBUS校验值"""
        crc = self.init_value
        for byte in data:
            crc = self.table[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFF

class ProtocolHandler:
    """协议处理器，负责按照头盔通讯协议格式处理数据"""
    
    def __init__(self):
        self.crc = CRC16MODBUS()
        self.command_id_counter = 1
        self.pending_commands = {}  # 待确认的命令池
    
    def create_vehicle_status_packet(self, platform_id=0):
        """创建车辆状态数据包 (Vehicle to Helmet)"""
        # 生成模拟数据
        timestamp = int(time.time())
        longitude = int(116.3974 * 1e7)  # 北京经度
        latitude = int(39.9093 * 1e7)    # 北京纬度
        altitude = int(50 * 100)         # 50米海拔
        ground_altitude = 0              # 车辆对地高度为0
        heading = int(90 * 100)          # 90度航向
        roll = int(0 * 100)              # 0度横滚
        pitch = int(0 * 100)             # 0度俯仰
        speed = int(60 * 10)             # 60 km/h
        ground_speed = 0                 # 车辆对地速度为0
        fuel_level = random.randint(80, 100)  # 80-100%油量
        battery_level = random.randint(85, 100)  # 85-100%电量
        gimbal_pitch = int(0 * 100)      # 云台俯仰
        gimbal_yaw = int(0 * 100)        # 云台方位
        gimbal_active = 1                # 云台激活
        ammo_type1 = random.randint(80, 100)  # 弹药类型1
        ammo_type2 = random.randint(70, 90)   # 弹药类型2
        ammo_type3 = random.randint(60, 80)   # 弹药类型3
        warnings = 0                     # 无警告
        
        # 构建数据包 (59字节)
        packet = struct.pack('>H', 0x55AE)  # 帧头
        packet += struct.pack('>H', 0x0037)  # 数据长度 (55字节)
        packet += struct.pack('>I', timestamp)  # 时间戳
        packet += struct.pack('B', platform_id)  # 平台标识
        packet += struct.pack('>i', longitude)   # 经度
        packet += struct.pack('>i', latitude)    # 纬度
        packet += struct.pack('>i', altitude)    # 海拔高度
        packet += struct.pack('>i', ground_altitude)  # 对地高度
        packet += struct.pack('>i', heading)     # 航向角
        packet += struct.pack('>i', roll)        # 横滚角
        packet += struct.pack('>i', pitch)       # 俯仰角
        packet += struct.pack('>h', speed)       # 速度
        packet += struct.pack('>h', ground_speed)  # 对地速度
        packet += struct.pack('B', fuel_level)   # 油量
        packet += struct.pack('B', battery_level)  # 电量
        packet += struct.pack('>i', gimbal_pitch)  # 云台俯仰
        packet += struct.pack('>i', gimbal_yaw)    # 云台方位
        packet += struct.pack('B', gimbal_active)  # 云台激活
        packet += struct.pack('B', ammo_type1)     # 弹药类型1
        packet += struct.pack('B', ammo_type2)     # 弹药类型2
        packet += struct.pack('B', ammo_type3)     # 弹药类型3
        packet += struct.pack('>H', warnings)      # 警告
        
        # 计算校验和 (前57字节)
        checksum = self.crc.calculate(packet)
        packet += struct.pack('>H', checksum)
        
        return packet
    
    def parse_head_tracking_packet(self, data):
        """解析头动跟踪数据包 (Helmet to Vehicle)"""
        if len(data) != 16:
            return None
        
        try:
            header, length, yaw, pitch, tracking_status, confidence, checksum = struct.unpack('>HHiiBBH', data)
            
            if header != 0x55AB:
                return None
            
            # 验证校验和
            calculated_checksum = self.crc.calculate(data[:-2])
            if calculated_checksum != checksum:
                return None
            
            return {
                'type': 'head_tracking',
                'yaw': yaw / 100.0,  # 转换为度
                'pitch': pitch / 100.0,  # 转换为度
                'tracking_status': tracking_status,
                'confidence': confidence
            }
        except struct.error:
            return None
    
    def parse_voice_text_packet(self, data):
        """解析语音文本数据包 (Helmet to Vehicle)"""
        if len(data) < 8:
            return None
        
        try:
            header, length = struct.unpack('>HH', data[:4])
            
            if header != 0x55AC:
                return None
            
            if len(data) != 6 + length:
                return None
            
            operation = data[4]
            packet_info = data[5]
            text_data = data[6:-2]
            checksum = struct.unpack('>H', data[-2:])[0]
            
            # 验证校验和
            calculated_checksum = self.crc.calculate(data[:-2])
            if calculated_checksum != checksum:
                return None
            
            return {
                'type': 'voice_text',
                'operation': operation,
                'packet_info': packet_info,
                'text': text_data.decode('utf-8', errors='ignore')
            }
        except (struct.error, UnicodeDecodeError):
            return None
    
    def parse_voice_command_packet(self, data):
        """解析语音指令数据包 (Helmet to Vehicle)"""
        if len(data) != 21:
            return None
        
        try:
            header, length, category, operation, command_id, param1, param2, param3, checksum = struct.unpack('>HHBBIIiBH', data)
            
            if header != 0x55AA:
                return None
            
            # 验证校验和
            calculated_checksum = self.crc.calculate(data[:-2])
            if calculated_checksum != checksum:
                return None
            
            return {
                'type': 'voice_command',
                'category': category,
                'operation': operation,
                'command_id': command_id,
                'param1': param1,
                'param2': param2,
                'param3': param3
            }
        except struct.error:
            return None
    
    def create_ack_packet(self, command_id, status):
        """创建确认包 (Vehicle to Helmet)"""
        packet = struct.pack('>H', 0x55AD)  # 帧头
        packet += struct.pack('>I', command_id)  # 命令ID
        packet += struct.pack('B', status)  # 状态 (0=失败, 1=成功)
        
        # 计算校验和 (前7字节)
        checksum = self.crc.calculate(packet)
        packet += struct.pack('>H', checksum)
        
        return packet

logger = logging.getLogger(__name__)

class VehicleSimulator:
    """车辆节点模拟器"""
    
    def __init__(self, config_file: str = None):
        # 加载配置
        self.config = self._load_config(config_file)
        
        # 从配置中获取参数
        self.vehicle_port = self.config['vehicle_simulator']['network']['vehicle_port']
        self.helmet_ip = self.config['vehicle_simulator']['network']['helmet']['ip']
        self.helmet_port = self.config['vehicle_simulator']['network']['helmet']['port']
        self.web_port = self.config['vehicle_simulator']['network']['web']['port']
        self.web_host = self.config['vehicle_simulator']['network']['web']['host']
        
        # 初始化协议处理器
        self.protocol_handler = ProtocolHandler()
        
        self.vehicle_socket = None
        self.websockets = set()
        
        # 控制状态
        self.vehicle_status_enabled = False
        self.vehicle_status_interval = 0.05  # 20Hz
        
        # 接收到的数据
        self.received_data = {
            'head_tracking': [],
            'voice_text': [],
            'voice_commands': []
        }
        
        # 发送的确认包
        self.sent_acks = []
    
    def _load_config(self, config_file: str = None) -> Dict[str, Any]:
        """加载配置文件"""
        if config_file is None:
            config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'vehicle_simulator_config.yaml')
        
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            logger.info(f"配置文件加载成功: {config_file}")
            return config
        except Exception as e:
            logger.error(f"配置文件加载失败: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            'vehicle_simulator': {
                'network': {
                    'vehicle_port': 8887,
                    'helmet': {
                        'ip': '127.0.0.1',
                        'port': 8888
                    },
                    'web': {
                        'port': 9091,
                        'host': '0.0.0.0'
                    }
                },
                'data': {
                    'vehicle_status': {
                        'enabled': False,
                        'interval': 0.05
                    }
                }
            }
        }
    
    async def start(self):
        """启动车辆模拟器"""
        logger.info("启动车辆节点模拟器...")
        
        # 启动UDP服务器
        await self.start_vehicle_server()
        
        # 启动Web服务器
        await self.start_web_server()
        
        logger.info("车辆节点模拟器启动完成")
    
    async def start_vehicle_server(self):
        """启动车辆UDP服务器"""
        self.vehicle_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vehicle_socket.bind(('0.0.0.0', self.vehicle_port))
        self.vehicle_socket.setblocking(False)
        
        logger.info(f"车辆节点UDP服务器启动，监听端口: {self.vehicle_port}")
        logger.info(f"头盔节点配置: {self.helmet_ip}:{self.helmet_port}")
        
        # 启动数据接收循环
        asyncio.create_task(self._receive_data_loop())
    
    async def _receive_data_loop(self):
        """接收来自头盔节点的数据"""
        loop = asyncio.get_event_loop()
        
        while True:
            try:
                # 接收数据
                data, addr = await loop.run_in_executor(None, self.vehicle_socket.recvfrom, 1024)
                
                # 解析数据包类型
                if len(data) >= 2:
                    header = struct.unpack('>H', data[0:2])[0]
                    
                    if header == 0x55AB:  # 头动跟踪数据
                        parsed_data = self.protocol_handler.parse_head_tracking_packet(data)
                        if parsed_data:
                            self.received_data['head_tracking'].append(parsed_data)
                            # 保持最近100条记录
                            if len(self.received_data['head_tracking']) > 100:
                                self.received_data['head_tracking'] = self.received_data['head_tracking'][-100:]
                            
                            logger.info(f"接收到头动跟踪数据: 偏航={parsed_data['yaw']:.2f}°, 俯仰={parsed_data['pitch']:.2f}°")
                            
                            # 通知Web界面
                            await self._notify_web_clients('head_tracking', parsed_data)
                    
                    elif header == 0x55AC:  # 语音文本数据
                        parsed_data = self.protocol_handler.parse_voice_text_packet(data)
                        if parsed_data:
                            self.received_data['voice_text'].append(parsed_data)
                            # 保持最近50条记录
                            if len(self.received_data['voice_text']) > 50:
                                self.received_data['voice_text'] = self.received_data['voice_text'][-50:]
                            
                            logger.info(f"接收到语音文本: {parsed_data['text']}")
                            
                            # 通知Web界面
                            await self._notify_web_clients('voice_text', parsed_data)
                    
                    elif header == 0x55AA:  # 语音指令数据
                        parsed_data = self.protocol_handler.parse_voice_command_packet(data)
                        if parsed_data:
                            self.received_data['voice_commands'].append(parsed_data)
                            # 保持最近50条记录
                            if len(self.received_data['voice_commands']) > 50:
                                self.received_data['voice_commands'] = self.received_data['voice_commands'][-50:]
                            
                            logger.info(f"接收到语音指令: 类别=0x{parsed_data['category']:02X}, 操作=0x{parsed_data['operation']:02X}, ID={parsed_data['command_id']}")
                            
                            # 发送确认包
                            ack_packet = self.protocol_handler.create_ack_packet(
                                parsed_data['command_id'], 0x01  # 成功
                            )
                            
                            # 发送确认包给头盔节点
                            await loop.run_in_executor(None, self.vehicle_socket.sendto, ack_packet, addr)
                            
                            # 记录发送的确认包
                            self.sent_acks.append({
                                'command_id': parsed_data['command_id'],
                                'status': 0x01,
                                'timestamp': time.time()
                            })
                            if len(self.sent_acks) > 50:
                                self.sent_acks = self.sent_acks[-50:]
                            
                            logger.info(f"发送语音指令确认: ID={parsed_data['command_id']}, 状态=成功")
                            
                            # 通知Web界面
                            await self._notify_web_clients('voice_command', parsed_data)
                            await self._notify_web_clients('voice_command_ack', {
                                'command_id': parsed_data['command_id'],
                                'status': 0x01,
                                'timestamp': time.time()
                            })
                
            except Exception as e:
                if "Resource temporarily unavailable" not in str(e):
                    logger.error(f"接收数据失败: {e}")
                await asyncio.sleep(0.001)  # 短暂等待
    
    async def _send_vehicle_status_loop(self):
        """车辆状态发送循环"""
        while self.vehicle_status_enabled:
            try:
                # 使用协议处理器创建车辆状态数据包
                packet = self.protocol_handler.create_vehicle_status_packet(platform_id=0)
                
                # 发送到头盔节点（communication模块）
                target_address = (self.helmet_ip, self.helmet_port)
                
                loop = asyncio.get_event_loop()
                await loop.run_in_executor(None, self.vehicle_socket.sendto, packet, target_address)
                
                # 通知Web界面
                await self._notify_web_clients('vehicle_status', {
                    'timestamp': time.time(),
                    'packet_size': len(packet),
                    'platform_id': 0
                })
                
            except Exception as e:
                logger.error(f"发送车辆状态数据失败: {e}")
            
            await asyncio.sleep(self.vehicle_status_interval)
    
    async def _notify_web_clients(self, data_type: str, data: Dict[str, Any]):
        """通知Web客户端数据更新"""
        message = {
            'type': 'data_update',
            'data_type': data_type,
            'data': data
        }
        
        # 发送给所有连接的WebSocket客户端
        disconnected = set()
        for ws in self.websockets:
            try:
                await ws.send_str(json.dumps(message))
            except Exception as e:
                logger.error(f"发送WebSocket消息失败: {e}")
                disconnected.add(ws)
        
        # 移除断开的连接
        self.websockets -= disconnected
    
    async def start_web_server(self):
        """启动Web服务器"""
        app = web.Application()

        # 添加中间件设置正确的Content-Type
        @web.middleware
        async def charset_middleware(request, handler):
            response = await handler(request)
            if response.content_type == 'text/html':
                response.charset = 'utf-8'
            return response

        app.middlewares.append(charset_middleware)

        # 静态文件服务
        static_path = '/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static'
        app.router.add_static('/static', static_path)
        
        # 路由
        app.router.add_get('/', self.index_handler)
        app.router.add_get('/ws', self.websocket_handler)
        app.router.add_get('/test_ws.html', self.test_ws_handler)
        app.router.add_get('/debug_ws.html', self.debug_ws_handler)
        app.router.add_get('/simple_test.html', self.simple_test_handler)
        app.router.add_get('/force_refresh.html', self.force_refresh_handler)
        app.router.add_get('/debug_connection.html', self.debug_connection_handler)
        app.router.add_get('/simple_connection_test.html', self.simple_connection_test_handler)
        app.router.add_get('/debug_main.html', self.debug_main_handler)
        app.router.add_get('/fixed_main.html', self.fixed_main_handler)
        
        # 启动服务器
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.web_host, self.web_port)
        await site.start()
        
        logger.info(f"Web服务器启动，访问地址: http://{self.web_host}:{self.web_port}")
    
    async def index_handler(self, request):
        """主页面处理器"""
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/vehicle_index.html')
    
    async def websocket_handler(self, request):
        """WebSocket处理器"""
        ws = WebSocketResponse()
        await ws.prepare(request)
        
        self.websockets.add(ws)
        logger.info(f"WebSocket客户端连接，当前连接数: {len(self.websockets)}")
        
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        await self._handle_websocket_message(ws, data)
                    except json.JSONDecodeError:
                        logger.error(f"无效的JSON消息: {msg.data}")
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"WebSocket错误: {ws.exception()}")
        finally:
            self.websockets.discard(ws)
            logger.info(f"WebSocket客户端断开，当前连接数: {len(self.websockets)}")
        
        return ws
    
    async def _handle_websocket_message(self, ws, data):
        """处理WebSocket消息"""
        message_type = data.get('type')
        
        if message_type == 'toggle_vehicle_status':
            self.vehicle_status_enabled = data.get('enabled', False)
            if self.vehicle_status_enabled:
                asyncio.create_task(self._send_vehicle_status_loop())
            
            # 发送状态更新
            await ws.send_str(json.dumps({
                'type': 'status',
                'data': {
                    'vehicle_status_enabled': self.vehicle_status_enabled
                }
            }))
            
        elif message_type == 'set_vehicle_status_interval':
            interval = data.get('interval', 0.05)
            self.vehicle_status_interval = interval
            
        elif message_type == 'get_status':
            await ws.send_str(json.dumps({
                'type': 'status',
                'data': {
                    'vehicle_status_enabled': self.vehicle_status_enabled,
                    'vehicle_status_interval': self.vehicle_status_interval,
                    'helmet_ip': self.helmet_ip,
                    'helmet_port': self.helmet_port
                }
            }))
            
        elif message_type == 'get_received_data':
            await self._send_received_data(ws)
    
    async def _send_received_data(self, ws):
        """发送接收到的数据"""
        data = {
            'type': 'received_data',
            'data': {
                'head_tracking': self.received_data['head_tracking'][-10:],  # 最近10条
                'voice_text': self.received_data['voice_text'][-10:],        # 最近10条
                'voice_commands': self.received_data['voice_commands'][-10:], # 最近10条
                'sent_acks': self.sent_acks[-10:]  # 最近10条确认包
            }
        }
        await ws.send_str(json.dumps(data))
    
    # 其他页面处理器
    async def test_ws_handler(self, request):
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/test_ws.html')
    
    async def debug_ws_handler(self, request):
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/debug_ws.html')
    
    async def simple_test_handler(self, request):
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/simple_test.html')
    
    async def force_refresh_handler(self, request):
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/force_refresh.html')
    
    async def debug_connection_handler(self, request):
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/debug_connection.html')
    
    async def simple_connection_test_handler(self, request):
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/simple_connection_test.html')
    
    async def debug_main_handler(self, request):
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/debug_main.html')
    
    async def fixed_main_handler(self, request):
        return web.FileResponse('/home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test/web/static/fixed_main.html')

async def main():
    """主函数"""
    import sys
    
    config_file = None
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
    
    simulator = VehicleSimulator(config_file)
    await simulator.start()
    
    # 保持运行
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("收到中断信号，正在关闭服务器...")
    finally:
        if simulator.vehicle_socket:
            simulator.vehicle_socket.close()

if __name__ == '__main__':
    asyncio.run(main())
