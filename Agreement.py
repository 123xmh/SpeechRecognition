# [file name]: Agreement.py
# [file content begin]
#!/usr/bin/env python3

import struct
import crcmod
from enum import Enum
import time
import threading
from collections import deque
import re 

class CommandCategory(Enum):
    ENVIRONMENT = 0x01
    LIGHTS = 0x02
    VEHICLE_MODE = 0x03
    INFO_SYSTEM = 0x04
    COMM_NAV = 0x05
    STATUS_QUERY = 0x06
    UAV = 0x07
    UGV = 0x08
    EMERGENCY = 0x09

class CommandManager:
    def __init__(self, udp_comm):
        self.udp_comm = udp_comm
        self.command_id_counter = 1
        self.pending_commands = {}  # command_id -> (data, timestamp, retry_count)
        self.crc16_func = crcmod.mkCrcFun(0x18005, initCrc=0xFFFF, rev=False)
        
        # 启动重传定时器
        self.retry_timer = threading.Timer(0.1, self._check_retries)
        self.retry_timer.daemon = True
        self.retry_timer.start()
    
    def _check_retries(self):
        """检查需要重传的命令"""
        current_time = time.time()
        to_remove = []
        
        for cmd_id, (data, timestamp, retry_count) in self.pending_commands.items():
            if current_time - timestamp > 0.2:  # 200ms超时
                if retry_count < 3:  # 最大重试3次
                    # 重发命令
                    self.udp_comm.send_data(data)
                    self.pending_commands[cmd_id] = (data, current_time, retry_count + 1)
                    print(f"重发命令 {cmd_id}, 重试次数: {retry_count + 1}")
                else:
                    # 重试次数超限，移除命令
                    to_remove.append(cmd_id)
                    print(f"命令 {cmd_id} 重试次数超限，移除")
        
        # 移除超限的命令
        for cmd_id in to_remove:
            self.pending_commands.pop(cmd_id, None)
        
        # 重新设置定时器
        self.retry_timer = threading.Timer(0.1, self._check_retries)
        self.retry_timer.daemon = True
        self.retry_timer.start()
    
    def _create_command_packet(self, category, operation, param1=0, param2=0, param3=0):
        """创建命令数据包"""
        command_id = self.command_id_counter
        self.command_id_counter += 1
        
        # 构建数据包（不包括校验和）
        header = 0x55AA
        length = 0x000F  # 固定15字节
        
        # 转换为网络字节序（大端）
        data = struct.pack('>HHBBIIIB', 
                          header, length, category, operation, 
                          command_id, param1, param2, param3)
        
        # 计算校验和（从header到param3）
        checksum = self.crc16_func(data)
        
        # 添加校验和
        full_packet = data + struct.pack('>H', checksum)
        
        return command_id, full_packet
    
    def send_command(self, category, operation, param1=0, param2=0, param3=0):
        """发送命令并添加到待确认列表"""
        command_id, packet = self._create_command_packet(category, operation, param1, param2, param3)
        
        # 添加到待确认列表
        self.pending_commands[command_id] = (packet, time.time(), 0)
        
        # 发送命令
        self.udp_comm.send_data(packet)
        print(f"发送命令: ID={command_id}, 类别={category:02X}, 操作={operation:02X}")
        
        return command_id, category, operation, param1, param2, param3
    
    def handle_ack(self, ack_data):
        """处理确认包"""
        if len(ack_data) < 9:
            return False
        
        try:
            # 解析确认包
            header, cmd_id, status = struct.unpack('>HIB', ack_data[:7])
            
            if header == 0x55AD:  # 确认包帧头
                if cmd_id in self.pending_commands:
                    # 移除已确认的命令
                    self.pending_commands.pop(cmd_id)
                    print(f"命令 {cmd_id} 已确认，状态: {'成功' if status == 0x01 else '失败'}")
                    return True
        except:
            pass
        
        return False

    # 1. 车辆环境控制系统
    def set_ac_temperature(self, temperature_celsius):
        """设置空调温度 (1.1)"""
        param1 = int(temperature_celsius * 10)  # 温度 × 10
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x01, param1)
    
    def increase_temperature(self):
        """调高温度 (1.2)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x02)
    
    def decrease_temperature(self):
        """调低温度 (1.3)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x03)
    
    def set_fan_speed(self, level):
        """设置风扇速度档位 (1.4)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x04, param3=level)
    
    def set_ac_auto_mode(self, enabled):
        """空调自动模式 (1.5)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x05, param3=1 if enabled else 0)
    
    def set_ac_power(self, enabled):
        """空调开关 (1.6)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x06, param3=1 if enabled else 0)
    
    def set_dust_pump(self, enabled):
        """抽尘泵运行 (1.7)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x07, param3=1 if enabled else 0)
    
    def set_fan_force(self, enabled):
        """风扇强制运行 (1.8)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x08, param3=1 if enabled else 0)
    
    def set_water_pump(self, enabled):
        """水泵强制运行 (1.9)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x09, param3=1 if enabled else 0)
    
    def set_low_temp_start(self, enabled):
        """低温启动 (1.10)"""
        return self.send_command(CommandCategory.ENVIRONMENT.value, 0x0A, param3=1 if enabled else 0)

    # 2. 车辆灯光与信号系统
    def set_left_turn_signal(self, enabled):
        """左转向灯 (2.1)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x01, param3=1 if enabled else 0)
    
    def set_right_turn_signal(self, enabled):
        """右转向灯 (2.2)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x02, param3=1 if enabled else 0)
    
    def set_left_low_beam(self, enabled):
        """左前大灯近光 (2.3)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x03, param3=1 if enabled else 0)
    
    def set_right_low_beam(self, enabled):
        """右前大灯近光 (2.4)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x04, param3=1 if enabled else 0)
    
    def set_left_high_beam(self, enabled):
        """左前大灯远光 (2.5)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x05, param3=1 if enabled else 0)
    
    def set_right_high_beam(self, enabled):
        """右前大灯远光 (2.6)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x06, param3=1 if enabled else 0)
    
    def set_position_lights(self, enabled):
        """示廓灯 (2.7)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x07, param3=1 if enabled else 0)
    
    def set_hazard_lights(self, enabled):
        """危险告警灯 (2.8)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x08, param3=1 if enabled else 0)
    
    def set_horn(self, enabled):
        """喇叭 (2.9)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x09, param3=1 if enabled else 0)
    
    def set_air_defense(self, enabled):
        """防空 (2.10)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x0A, param3=1 if enabled else 0)
    
    def set_water_spray(self, enabled):
        """喷水 (2.11)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x0B, param3=1 if enabled else 0)
    
    def set_left_screen_switch(self, enabled):
        """左切屏 (2.12)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x0C, param3=1 if enabled else 0)
    
    def set_right_screen_switch(self, enabled):
        """右切屏 (2.13)"""
        return self.send_command(CommandCategory.LIGHTS.value, 0x0D, param3=1 if enabled else 0)

    # 3. 车辆行驶模式
    def set_vehicle_mode(self, mode):
        """设置车辆模式 (3.1)"""
        return self.send_command(CommandCategory.VEHICLE_MODE.value, 0x01, param3=mode)

    # 4. 车辆信息系统
    def show_front_video(self):
        """显示前方视频 (4.1)"""
        return self.send_command(CommandCategory.INFO_SYSTEM.value, 0x01)
    
    def show_rear_video(self):
        """显示后方视频 (4.2)"""
        return self.send_command(CommandCategory.INFO_SYSTEM.value, 0x02)
    
    def show_status_info(self):
        """显示状态信息 (4.3)"""
        return self.send_command(CommandCategory.INFO_SYSTEM.value, 0x03)
    
    def show_map(self):
        """显示地图 (4.4)"""
        return self.send_command(CommandCategory.INFO_SYSTEM.value, 0x04)
    
    def show_thermal(self):
        """显示热成像 (4.5)"""
        return self.send_command(CommandCategory.INFO_SYSTEM.value, 0x05)
    
    def switch_video_source(self, source_id):
        """切换视频源 (4.6)"""
        return self.send_command(CommandCategory.INFO_SYSTEM.value, 0x06, param3=source_id)

    # 5. 通信与导航
    def establish_comm_link(self):
        """建立通信链路 (5.1)"""
        return self.send_command(CommandCategory.COMM_NAV.value, 0x01)
    
    def terminate_comm_link(self):
        """终止通信链路 (5.2)"""
        return self.send_command(CommandCategory.COMM_NAV.value, 0x02)
    
    def set_waypoint(self, latitude, longitude):
        """设置航点 (5.3)"""
        param1 = int(latitude * 1e7)  # 纬度 × 1e7
        param2 = int(longitude * 1e7)  # 经度 × 1e7
        return self.send_command(CommandCategory.COMM_NAV.value, 0x03, param1, param2)
    
    def clear_all_waypoints(self):
        """清除所有航点 (5.4)"""
        return self.send_command(CommandCategory.COMM_NAV.value, 0x04)
    
    def request_return(self):
        """请求返航 (5.5)"""
        return self.send_command(CommandCategory.COMM_NAV.value, 0x05)
    
    def start_mission(self):
        """开始执行任务 (5.6)"""
        return self.send_command(CommandCategory.COMM_NAV.value, 0x06)

    # 6. 系统状态查询
    def query_battery_status(self):
        """查询电池状态 (6.1)"""
        return self.send_command(CommandCategory.STATUS_QUERY.value, 0x01)
    
    def query_fuel_status(self):
        """查询燃油状态 (6.2)"""
        return self.send_command(CommandCategory.STATUS_QUERY.value, 0x02)
    
    def query_ammo_status(self):
        """查询弹药状态 (6.3)"""
        return self.send_command(CommandCategory.STATUS_QUERY.value, 0x03)
    
    def query_position_info(self):
        """查询位置信息 (6.4)"""
        return self.send_command(CommandCategory.STATUS_QUERY.value, 0x04)
    
    def query_all_system_status(self):
        """查询所有系统状态 (6.5)"""
        return self.send_command(CommandCategory.STATUS_QUERY.value, 0x05)
    
    def query_comm_status(self):
        """查询通信状态 (6.6)"""
        return self.send_command(CommandCategory.STATUS_QUERY.value, 0x06)
    
    def query_sensor_status(self):
        """查询传感器状态 (6.7)"""
        return self.send_command(CommandCategory.STATUS_QUERY.value, 0x07)

    # 7. 无人机控制指令
    def uav_takeoff(self):
        """起飞 (7.1)"""
        return self.send_command(CommandCategory.UAV.value, 0x01)
    
    def uav_land(self):
        """降落 (7.2)"""
        return self.send_command(CommandCategory.UAV.value, 0x02)
    
    def uav_return_home(self):
        """返航 (7.3)"""
        return self.send_command(CommandCategory.UAV.value, 0x03)
    
    def uav_hover(self):
        """悬停 (7.4)"""
        return self.send_command(CommandCategory.UAV.value, 0x04)
    
    def uav_ascend_descend(self, distance_cm):
        """上升/下降 (7.5)"""
        param1 = int(distance_cm * 100)  # 距离 × 100 (厘米)
        return self.send_command(CommandCategory.UAV.value, 0x05, param1)
    
    def uav_forward_backward(self, distance_cm):
        """前进/后退 (7.6)"""
        param1 = int(distance_cm * 100)  # 距离 × 100 (厘米)
        return self.send_command(CommandCategory.UAV.value, 0x06, param1)
    
    def uav_left_right(self, distance_cm):
        """左移/右移 (7.7)"""
        param1 = int(distance_cm * 100)  # 距离 × 100 (厘米)
        return self.send_command(CommandCategory.UAV.value, 0x07, param1)
    
    def uav_zoom_in(self):
        """光电吊舱变焦（放大） (7.8)"""
        return self.send_command(CommandCategory.UAV.value, 0x08)
    
    def uav_zoom_out(self):
        """光电吊舱变焦（缩小） (7.9)"""
        return self.send_command(CommandCategory.UAV.value, 0x09)
    
    def uav_gimbal_left(self, angle_deg):
        """光电吊舱云台左转 (7.10)"""
        param1 = int(angle_deg * 100)  # 角度 × 100 (0.01度)
        return self.send_command(CommandCategory.UAV.value, 0x0A, param1)
    
    def uav_gimbal_right(self, angle_deg):
        """光电吊舱云台右转 (7.11)"""
        param1 = int(angle_deg * 100)  # 角度 × 100 (0.01度)
        return self.send_command(CommandCategory.UAV.value, 0x0B, param1)
    
    def uav_gimbal_up(self, angle_deg):
        """光电吊舱云台上仰 (7.12)"""
        param1 = int(angle_deg * 100)  # 角度 × 100 (0.01度)
        return self.send_command(CommandCategory.UAV.value, 0x0C, param1)
    
    def uav_gimbal_down(self, angle_deg):
        """光电吊舱云台下俯 (7.13)"""
        param1 = int(angle_deg * 100)  # 角度 × 100 (0.01度)
        return self.send_command(CommandCategory.UAV.value, 0x0D, param1)

    # 8. 无人车控制指令
    def ugv_work_mode(self, mode):
        """工作模式 (8.1)"""
        return self.send_command(CommandCategory.UGV.value, 0x01, param3=mode)
    
    def ugv_emergency_stop(self):
        """紧急停止 (8.2)"""
        return self.send_command(CommandCategory.UGV.value, 0x02)
    
    def ugv_forward(self):
        """UGV前进 (8.3)"""
        return self.send_command(CommandCategory.UGV.value, 0x03)
    
    def ugv_backward(self):
        """UGV后退 (8.4)"""
        return self.send_command(CommandCategory.UGV.value, 0x04)
    
    def ugv_turn_left(self):
        """UGV左转 (8.5)"""
        return self.send_command(CommandCategory.UGV.value, 0x05)
    
    def ugv_turn_right(self):
        """UGV右转 (8.6)"""
        return self.send_command(CommandCategory.UGV.value, 0x06)
    
    def ugv_stop(self):
        """UGV停止 (8.7)"""
        return self.send_command(CommandCategory.UGV.value, 0x07)
    
    def ugv_set_speed(self, speed_kmh):
        """设置UGV速度 (8.8)"""
        param1 = int(speed_kmh * 10)  # 速度 × 10 (0.1 km/h)
        return self.send_command(CommandCategory.UGV.value, 0x08, param1)
    
    def ugv_forward_distance(self, distance_cm):
        """UGV前进N米 (8.9)"""
        param1 = int(distance_cm * 100)  # 距离 × 100 (厘米)
        return self.send_command(CommandCategory.UGV.value, 0x09, param1)
    
    def ugv_backward_distance(self, distance_cm):
        """UGV后退N米 (8.10)"""
        param1 = int(distance_cm * 100)  # 距离 × 100 (厘米)
        return self.send_command(CommandCategory.UGV.value, 0x0A, param1)
    
    def ugv_turn_left_angle(self, angle_deg):
        """UGV左转N度 (8.11)"""
        param1 = int(angle_deg * 100)  # 角度 × 100 (0.01度)
        return self.send_command(CommandCategory.UGV.value, 0x0B, param1)
    
    def ugv_turn_right_angle(self, angle_deg):
        """UGV右转N度 (8.12)"""
        param1 = int(angle_deg * 100)  # 角度 × 100 (0.01度)
        return self.send_command(CommandCategory.UGV.value, 0x0C, param1)
    
    def ugv_follow(self):
        """UGV跟随 (8.13)"""
        return self.send_command(CommandCategory.UGV.value, 0x0D)
    
    def ugv_auto_mode(self):
        """UGV切换至自主模式 (8.14)"""
        return self.send_command(CommandCategory.UGV.value, 0x0E)
    
    def ugv_manual_mode(self):
        """UGV切换至手动模式 (8.15)"""
        return self.send_command(CommandCategory.UGV.value, 0x0F)

    # 9. 应急指令
    def emergency_brake(self):
        """紧急制动 (急停) (9.1)"""
        return self.send_command(CommandCategory.EMERGENCY.value, 0x01)
    
    def emergency_recover(self):
        """紧急恢复 (缓停) (9.2)"""
        return self.send_command(CommandCategory.EMERGENCY.value, 0x02)
    
    def release_smoke_decoy(self):
        """释放烟雾/诱饵 (9.3)"""
        return self.send_command(CommandCategory.EMERGENCY.value, 0x03)
    
    def send_sos(self):
        """发送求救信号 (9.4)"""
        return self.send_command(CommandCategory.EMERGENCY.value, 0x04)
    
    def enter_safe_mode(self):
        """进入安全模式 (9.5)"""
        return self.send_command(CommandCategory.EMERGENCY.value, 0x05)

def process_voice_command(text, cmd_manager, voice_publisher):
    """处理语音指令并发送相应的命令"""
    text_lower = text.lower()
    command_info = None
    
    # 环境控制系统指令
    if "温度调高" in text_lower or "调高温度" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.increase_temperature()
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "调高温度")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "温度调低" in text_lower or "调低温度" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.decrease_temperature()
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "调低温度")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "度" in text_lower and ("设置" in text_lower or "调到" in text_lower):
        # 提取温度数字
        import re
        temp_match = re.search(r'(\d+)度', text_lower)
        if temp_match:
            temperature = int(temp_match.group(1))
            if 16 <= temperature <= 30:
                command_id, category, operation, param1, param2, param3 = cmd_manager.set_ac_temperature(temperature)
                voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, f"设置温度{temperature}度")
                command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "打开空调" in text_lower or "开启空调" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.set_ac_power(True)
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "打开空调")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "关闭空调" in text_lower or "关掉空调" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.set_ac_power(False)
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "关闭空调")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    # 灯光系统指令
    elif "左转向灯" in text_lower:
        if "打开" in text_lower or "开启" in text_lower:
            command_id, category, operation, param1, param2, param3 = cmd_manager.set_left_turn_signal(True)
            voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "打开左转向灯")
            command_info = (command_id, category, operation, param1, param2, param3)
        elif "关闭" in text_lower or "关掉" in text_lower:
            command_id, category, operation, param1, param2, param3 = cmd_manager.set_left_turn_signal(False)
            voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "关闭左转向灯")
            command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "右转向灯" in text_lower:
        if "打开" in text_lower or "开启" in text_lower:
            command_id, category, operation, param1, param2, param3 = cmd_manager.set_right_turn_signal(True)
            voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "打开右转向灯")
            command_info = (command_id, category, operation, param1, param2, param3)
        elif "关闭" in text_lower or "关掉" in text_lower:
            command_id, category, operation, param1, param2, param3 = cmd_manager.set_right_turn_signal(False)
            voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "关闭右转向灯")
            command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "危险灯" in text_lower or "双闪" in text_lower:
        if "打开" in text_lower or "开启" in text_lower:
            command_id, category, operation, param1, param2, param3 = cmd_manager.set_hazard_lights(True)
            voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "打开危险灯")
            command_info = (command_id, category, operation, param1, param2, param3)
        elif "关闭" in text_lower or "关掉" in text_lower:
            command_id, category, operation, param1, param2, param3 = cmd_manager.set_hazard_lights(False)
            voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "关闭危险灯")
            command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "喇叭" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.set_horn(True)
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "鸣喇叭")
        command_info = (command_id, category, operation, param1, param2, param3)
        # 喇叭通常是瞬时操作，不需要关闭
    
    # 车辆模式指令
    elif "静默模式" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.set_vehicle_mode(1)
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "设置静默模式")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "混动模式" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.set_vehicle_mode(2)
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "设置混动模式")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "动力模式" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.set_vehicle_mode(4)
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "设置动力模式")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    # 无人机指令
    elif "起飞" in text_lower and ("无人机" in text_lower or "飞机" in text_lower):
        command_id, category, operation, param1, param2, param3 = cmd_manager.uav_takeoff()
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "无人机起飞")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "降落" in text_lower and ("无人机" in text_lower or "飞机" in text_lower):
        command_id, category, operation, param1, param2, param3 = cmd_manager.uav_land()
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "无人机降落")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "返航" in text_lower and ("无人机" in text_lower or "飞机" in text_lower):
        command_id, category, operation, param1, param2, param3 = cmd_manager.uav_return_home()
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "无人机返航")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    # 应急指令
    elif "紧急制动" in text_lower or "急停" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.emergency_brake()
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "紧急制动")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    elif "求救" in text_lower or "sos" in text_lower:
        command_id, category, operation, param1, param2, param3 = cmd_manager.send_sos()
        voice_publisher.publish_command_result(category, operation, command_id, param1, param2, param3, True, "发送求救信号")
        command_info = (command_id, category, operation, param1, param2, param3)
    
    # 如果没有匹配到指令，发布无效指令
    if command_info is None:
        voice_publisher.publish_command_result(0, 0, 0, 0, 0, 0, False, text)
        return 0
    
    return command_info[0]  # 返回command_id


# 使用示例
# if __name__ == "__main__":
#     # 假设已经有一个UDP通信对象
#     class MockUDPComm:
#         def send_data(self, data):
#             print(f"发送数据: {data.hex()}")
    
#     udp_comm = MockUDPComm()
#     cmd_manager = CommandManager(udp_comm)
    
#     # 测试发送一些命令
#     cmd_manager.set_ac_temperature(25.0)  # 设置空调温度25度
#     cmd_manager.set_left_turn_signal(True)  # 开启左转向灯
#     cmd_manager.set_vehicle_mode(2)  # 设置为混动模式
#     cmd_manager.uav_takeoff()  # 无人机起飞
# [file content end]