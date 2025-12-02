import rclpy
from rclpy.node import Node
import json
import asyncio
import os
from threading import Thread
import traceback
import concurrent.futures

# 导入 aiohttp 相关模块
import aiohttp
from aiohttp import web

from chest_interfaces.msg import (
    ButtonState, HeadTracking, VoiceCommand, VoiceText, 
    VideoSource, EquipmentState
)
from std_msgs.msg import String
from sensor_msgs.msg import Imu

class WebMonitorNode(Node):
    def __init__(self):
        super().__init__('web_monitor_node')

        # 存储最新数据
        self.data = {
            'button_state': None,
            'head_tracking': None,
            'voice_command': None,
            'voice_text': None,
            'video_source': None,
            'equipment_state': None,
            'imu_data': None
        }

        # WebSocket服务器设置
        self.ws_clients = set()
        self.loop = None
        self.server_thread = None

        # 获取静态文件目录路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.static_dir = os.path.join(current_dir, 'static')
        
        self.get_logger().info(f'Static directory: {self.static_dir}')

        # 启动WebSocket服务器
        self.start_websocket_server()

        # 订阅所有话题
        self.create_subscriptions()

        self.get_logger().info('Web Monitor Node started')

    def create_subscriptions(self):
        """创建所有ROS2订阅"""
        # 使用正确的消息类型和话题名称
        self.button_sub = self.create_subscription(
            ButtonState, '/tk_chest/button/state',
            self.button_callback, 10
        )
        self.head_tracking_sub = self.create_subscription(
            HeadTracking, '/tk_chest/head_tracking/state',
            self.head_tracking_callback, 10
        )
        self.voice_command_sub = self.create_subscription(
            VoiceCommand, '/tk_chest/voice_processing/command',
            self.voice_command_callback, 10
        )
        self.voice_text_sub = self.create_subscription(
            VoiceText, '/tk_chest/voice_processing/raw_text',
            self.voice_text_callback, 10
        )
        self.video_source_sub = self.create_subscription(
            VideoSource, '/tk_chest/video/source/selected',
            self.video_source_callback, 10
        )
        self.equipment_state_sub = self.create_subscription(
            EquipmentState, '/tk_chest/equipment/status',
            self.equipment_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/tk_chest/sensors/imu',
            self.imu_callback, 10
        )
        
        self.get_logger().info("All subscriptions created")

    async def websocket_handler(self, request):
        """处理WebSocket连接"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.ws_clients.add(ws)
        self.get_logger().info('New WebSocket client connected')
        
        try:
            # 发送初始数据
            await ws.send_json({
                'type': 'initial_data',
                'data': self.data
            })
            
            # 保持连接，监听消息
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    # 处理客户端消息（如果有）
                    try:
                        data = json.loads(msg.data)
                        if data.get('type') == 'ping':
                            await ws.send_json({'type': 'pong'})
                    except:
                        pass
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    self.get_logger().error(f'WebSocket connection closed with exception {ws.exception()}')
                    break
                    
        except Exception as e:
            self.get_logger().error(f'WebSocket error: {e}')
        finally:
            if ws in self.ws_clients:
                self.ws_clients.remove(ws)
            self.get_logger().info('WebSocket client disconnected')
            
        return ws

    async def handle_index(self, request):
        """处理根路径请求，返回HTML页面"""
        index_path = os.path.join(self.static_dir, 'index.html')
        if os.path.exists(index_path):
            return web.FileResponse(index_path)
        else:
            self.get_logger().error(f"Index file not found at {index_path}")
            return web.Response(text="Web interface not found", status=404)

    def start_websocket_server(self):
        """使用 aiohttp 启动 WebSocket 服务器和静态文件服务"""
        def run_server():
            try:
                # 创建新的事件循环
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
                
                # 创建 aiohttp 应用
                app = web.Application()
                
                # 添加静态文件路由
                if os.path.exists(os.path.join(self.static_dir, 'css')):
                    app.router.add_static('/css/', path=os.path.join(self.static_dir, 'css'))
                if os.path.exists(os.path.join(self.static_dir, 'js')):
                    app.router.add_static('/js/', path=os.path.join(self.static_dir, 'js'))
                
                # 添加WebSocket路由
                app.router.add_get('/ws', self.websocket_handler)
                
                # 添加首页路由
                app.router.add_get('/', self.handle_index)
                app.router.add_static('/', path=self.static_dir)
                
                # 创建运行器
                runner = web.AppRunner(app)
                self.loop.run_until_complete(runner.setup())
                
                # 检测WSL2环境并配置网络
                host = '0.0.0.0'  # 默认绑定所有接口
                port = 9090
                
                # 检测是否在WSL2环境中
                if os.path.exists('/proc/version') and 'microsoft' in open('/proc/version').read():
                    self.get_logger().info('检测到WSL2环境')
                    try:
                        # 获取WSL2的IP地址
                        import subprocess
                        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
                        if result.returncode == 0:
                            wsl_ip = result.stdout.strip().split()[0]
                            self.get_logger().info(f'WSL2 IP地址: {wsl_ip}')
                            self.get_logger().info(f'在Windows中可以通过 http://localhost:{port} 访问')
                            self.get_logger().info(f'在WSL2中可以通过 http://{wsl_ip}:{port} 访问')
                    except Exception as e:
                        self.get_logger().warning(f'无法获取WSL2 IP地址: {e}')
                
                # 创建 TCP 站点
                site = web.TCPSite(runner, host, port)
                self.loop.run_until_complete(site.start())
                
                self.get_logger().info(f'aiohttp WebSocket server started on {host}:{port}')
                self.get_logger().info('Serving static files from: ' + self.static_dir)
                
                # 运行事件循环
                self.loop.run_forever()
                
            except Exception as e:
                self.get_logger().error(f'Failed to start WebSocket server: {e}')
                self.get_logger().error(traceback.format_exc())
        
        self.server_thread = Thread(target=run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info('WebSocket server thread started')

    def broadcast_data(self):
        """向所有客户端广播数据"""
        if self.loop and self.ws_clients:
            try:
                # 确保数据是可序列化的
                serializable_data = {}
                for key, value in self.data.items():
                    if value is not None:
                        serializable_data[key] = value
                
                message = json.dumps({
                    'type': 'update',
                    'data': serializable_data
                })
                
                # 使用线程池执行异步函数
                future = asyncio.run_coroutine_threadsafe(self._async_broadcast(message), self.loop)
                future.result(timeout=2.0)  # 等待2秒
            except concurrent.futures.TimeoutError:
                self.get_logger().warn('Broadcast timeout')
            except Exception as e:
                self.get_logger().error(f'Broadcast error: {e}')

    async def _async_broadcast(self, message):
        """异步广播数据"""
        disconnected_clients = []
        for client in self.ws_clients:
            try:
                await client.send_str(message)
            except Exception as e:
                self.get_logger().error(f'Error sending to client: {e}')
                disconnected_clients.append(client)
        
        # 移除断开连接的客户端
        for client in disconnected_clients:
            if client in self.ws_clients:
                self.ws_clients.remove(client)

    # 所有回调函数修改为调用 broadcast_data() 而不是异步方法
    def button_callback(self, msg):
        """处理按钮状态消息"""
        try:
            self.data['button_state'] = {
                'head_tracking_enabled': msg.head_tracking_enabled,
                'voice_command_enabled': msg.voice_command_enabled,
                'front_screen_enabled': msg.front_screen_enabled,
                'front_screen_mode': msg.front_screen_mode,
                'hmd_video_source': msg.hmd_video_source,
                'hmd_overlay_mode': msg.hmd_overlay_mode,
                'hmd_enabled': msg.hmd_enabled,
                'timestamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                }
            }
            
            self.broadcast_data()
        except Exception as e:
            self.get_logger().error(f'Error in button_callback: {e}')

    def head_tracking_callback(self, msg):
        """处理头动跟踪消息"""
        try:
            self.data['head_tracking'] = {
                'pitch': msg.pitch,
                'yaw': msg.yaw,
                'is_tracking': msg.is_tracking,
                'confidence': msg.confidence,
                'timestamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                }
            }
            
            self.broadcast_data()
        except Exception as e:
            self.get_logger().error(f'Error in head_tracking_callback: {e}')

    def voice_command_callback(self, msg):
        """处理语音命令消息"""
        try:
            self.data['voice_command'] = {
                'frame_header': msg.frame_header,
                'command_id': msg.command_id,
                'param1': msg.param1,
                'param2': msg.param2,
                'param3': msg.param3,
                'is_valid': msg.is_valid,
                'timestamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                }
            }
            
            self.broadcast_data()
        except Exception as e:
            self.get_logger().error(f'Error in voice_command_callback: {e}')

    def voice_text_callback(self, msg):
        """处理语音文本消息"""
        try:
            self.data['voice_text'] = {
                'raw_text': msg.raw_text,
                'is_final': msg.is_final,
                'confidence': msg.confidence,
                'timestamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                }
            }
            
            self.broadcast_data()
        except Exception as e:
            self.get_logger().error(f'Error in voice_text_callback: {e}')

    def video_source_callback(self, msg):
        """处理视频源消息"""
        try:
            self.data['video_source'] = {
                'id': msg.id,
                'ip_address': msg.ip_address,
                'source_name': msg.source_name,
                'fps': msg.fps,
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'is_available': msg.is_available,
                'timestamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                }
            }
            
            self.broadcast_data()
        except Exception as e:
            self.get_logger().error(f'Error in video_source_callback: {e}')

    def equipment_state_callback(self, msg):
        """处理设备状态消息"""
        try:
            self.data['equipment_state'] = {
                'longitude': msg.longitude,
                'latitude': msg.latitude,
                'speed': msg.speed,
                'fuel_level': msg.fuel_level,
                'battery_level': msg.battery_level,
                'roll': msg.roll,
                'pitch': msg.pitch,
                'yaw': msg.yaw,
                'gimbal_pitch': msg.gimbal_pitch,
                'gimbal_yaw': msg.gimbal_yaw,
                'gimbal_is_active': msg.gimbal_is_active,
                'ammo_types': list(msg.ammo_types),
                'ammo_counts': list(msg.ammo_counts),
                'warnings': list(msg.warnings),
                'timestamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                }
            }
            
            self.broadcast_data()
        except Exception as e:
            self.get_logger().error(f'Error in equipment_state_callback: {e}')

    def imu_callback(self, msg):
        """处理IMU数据消息"""
        try:
            self.data['imu_data'] = {
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'timestamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                }
            }
            
            self.broadcast_data()
        except Exception as e:
            self.get_logger().error(f'Error in imu_callback: {e}')

    def __del__(self):
        """清理资源"""
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)

def main(args=None):
    rclpy.init(args=args)
    node = WebMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error in main: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()