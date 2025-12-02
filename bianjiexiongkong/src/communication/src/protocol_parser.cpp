#include "communication/protocol_parser.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

namespace communication {

ProtocolParser::ProtocolParser() : next_command_id_(1) {
}

ProtocolParser::~ProtocolParser() {
}

bool ProtocolParser::parseUdpData(const std::vector<uint8_t>& data, std::string& message_type) {
    if (data.size() < 4) {
        return false;
    }
    
    // 检查帧头
    uint16_t header = (data[0] << 8) | data[1];
    
    switch (header) {
        case 0x55AE: // 车辆状态
            message_type = "vehicle_state";
            return parseVehicleState(data);
            
        case 0x55AB: // 头动跟踪
            message_type = "head_tracking";
            return parseHeadTracking(data);
            
        case 0x55AC: // 语音文本
            message_type = "voice_text";
            return parseVoiceText(data);
            
        case 0x55AA: // 语音指令
            message_type = "voice_command";
            return parseVoiceCommand(data);
            
        case 0x55AD: // 确认包
            message_type = "ack";
            return true;
            
        default:
            return false;
    }
}

bool ProtocolParser::parseVehicleState(const std::vector<uint8_t>& data) {
    if (data.size() != 59) {
        return false;
    }
    
    // 验证CRC
    if (!verifyCrc(data, 0, 56)) {
        return false;
    }
    
    // 解析字段（EquipmentState 无 header 字段）
    
    // 平台标识
    uint8_t platform_id = data[8];
    
    // 经纬度 (WGS84坐标 × 1e7)
    int32_t longitude_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[9]));
    int32_t latitude_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[13]));
    equipment_state_.longitude = longitude_raw / 1e7;
    equipment_state_.latitude = latitude_raw / 1e7;
    
    // 海拔高度 (×100)
    int32_t altitude_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[17]));
    equipment_state_.altitude = altitude_raw / 100.0;
    
    // 对地高度 (×100)
    int32_t ground_altitude_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[21]));
    equipment_state_.ground_altitude = ground_altitude_raw / 100.0;
    
    // 航向角 (角度 × 100)
    int32_t heading_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[25]));
    equipment_state_.heading = heading_raw / 100.0 * M_PI / 180.0; // 转换为弧度
    
    // 横滚角 (角度 × 100)
    int32_t roll_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[29]));
    equipment_state_.roll = roll_raw / 100.0 * M_PI / 180.0; // 转换为弧度
    
    // 俯仰角 (角度 × 100)
    int32_t pitch_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[33]));
    equipment_state_.pitch = pitch_raw / 100.0 * M_PI / 180.0; // 转换为弧度
    
    // 速度 (实际速度 × 10)
    int16_t speed_raw = swapInt16(*reinterpret_cast<const int16_t*>(&data[37]));
    equipment_state_.speed = speed_raw / 10.0;
    
    // 对地速度 (×10)，仅无人机有效
    
    // 油量百分比
    uint8_t fuel_level = data[41];
    equipment_state_.fuel_level = (fuel_level == 255) ? NAN : fuel_level / 100.0;
    
    // 电量百分比
    uint8_t battery_level = data[42];
    equipment_state_.battery_level = (battery_level == 255) ? NAN : battery_level / 100.0;
    
    // 云台/吊舱俯仰 (×100)
    int32_t gimbal_pitch_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[43]));
    equipment_state_.gimbal_pitch = gimbal_pitch_raw / 100.0 * M_PI / 180.0; // 转换为弧度
    
    // 云台/吊舱方位 (×100)
    int32_t gimbal_yaw_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[47]));
    equipment_state_.gimbal_yaw = gimbal_yaw_raw / 100.0 * M_PI / 180.0; // 转换为弧度
    
    // 云台/吊舱激活状态
    uint8_t gimbal_active = data[51];
    equipment_state_.gimbal_is_active = (gimbal_active == 1);
    
    // 弹药类型
    equipment_state_.ammo_types = {"Type1", "Type2", "Type3"};
    equipment_state_.ammo_counts = {
        data[52], // 弹药类型1数量
        data[53], // 弹药类型2数量
        data[54]  // 弹药类型3数量
    };
    
    // 警告位掩码
    uint16_t warnings_raw = swapUint16(*reinterpret_cast<const uint16_t*>(&data[55]));
    equipment_state_.warnings.clear();
    
    // 根据协议解析警告位
    if (warnings_raw & 0x0001) equipment_state_.warnings.push_back("通信故障");
    if (warnings_raw & 0x0002) equipment_state_.warnings.push_back("传感器异常");
    if (warnings_raw & 0x0004) equipment_state_.warnings.push_back("电池低电量");
    if (warnings_raw & 0x0008) equipment_state_.warnings.push_back("动力系统故障");
    if (warnings_raw & 0x0010) equipment_state_.warnings.push_back("燃油不足");
    if (warnings_raw & 0x0020) equipment_state_.warnings.push_back("武器系统故障");
    if (warnings_raw & 0x0040) equipment_state_.warnings.push_back("悬挂系统异常");
    if (warnings_raw & 0x0080) equipment_state_.warnings.push_back("制动系统异常");
    if (warnings_raw & 0x0100) equipment_state_.warnings.push_back("传动系统异常");
    if (warnings_raw & 0x0200) equipment_state_.warnings.push_back("北斗信号弱");
    if (warnings_raw & 0x0400) equipment_state_.warnings.push_back("视觉定位异常");
    if (warnings_raw & 0x0800) equipment_state_.warnings.push_back("避障系统异常");
    if (warnings_raw & 0x1000) equipment_state_.warnings.push_back("图传信号弱");
    
    return true;
}

bool ProtocolParser::parseHeadTracking(const std::vector<uint8_t>& data) {
    if (data.size() != 16) {
        return false;
    }
    
    // 验证CRC
    if (!verifyCrc(data, 0, 13)) {
        return false;
    }
    
    // 解析字段
    head_tracking_.header.stamp = rclcpp::Clock().now();
    head_tracking_.header.frame_id = "head_tracking";
    
    // 偏航角 (角度 × 100)
    int32_t yaw_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[4]));
    head_tracking_.yaw = yaw_raw / 100.0 * M_PI / 180.0; // 转换为弧度
    
    // 俯仰角 (角度 × 100)
    int32_t pitch_raw = swapInt32(*reinterpret_cast<const int32_t*>(&data[8]));
    head_tracking_.pitch = pitch_raw / 100.0 * M_PI / 180.0; // 转换为弧度
    
    // 跟踪状态
    uint8_t tracking_status = data[12];
    head_tracking_.is_tracking = (tracking_status == 1);
    
    // 置信度
    uint8_t confidence = data[13];
    head_tracking_.confidence = confidence / 100.0;
    
    return true;
}

bool ProtocolParser::parseVoiceText(const std::vector<uint8_t>& data) {
    if (data.size() < 8) {
        return false;
    }
    
    // 获取数据长度
    uint16_t length = swapUint16(*reinterpret_cast<const uint16_t*>(&data[2]));
    
    if (data.size() != 8 + length) {
        return false;
    }
    
    // 验证CRC
    if (!verifyCrc(data, 0, 5 + length)) {
        return false;
    }
    
    // 解析字段
    voice_text_.header.stamp = rclcpp::Clock().now();
    voice_text_.header.frame_id = "voice_text";
    
    // 操作码
    uint8_t operation = data[4];
    voice_text_.is_final = (operation == 0x02);
    
    // 包信息 (跳过)
    
    // 文本数据 (UTF-8编码)
    size_t text_length = length - 2; // 减去Operation和PacketInfo
    voice_text_.raw_text = std::string(reinterpret_cast<const char*>(&data[6]), text_length);
    
    // 置信度暂时设为1.0，实际可能需要从其他字段获取
    voice_text_.confidence = 1.0;
    
    return true;
}

bool ProtocolParser::parseVoiceCommand(const std::vector<uint8_t>& data) {
    if (data.size() != 21) {
        return false;
    }
    
    // 验证CRC
    if (!verifyCrc(data, 0, 19)) {
        return false;
    }
    
    // 解析字段
    voice_command_.header.stamp = rclcpp::Clock().now();
    voice_command_.header.frame_id = "voice_command";
    
    // 数据类别和操作码
    uint8_t category = data[4];
    uint8_t operation = data[5];
    voice_command_.frame_header = (category << 8) | operation;
    
    // 命令ID
    voice_command_.command_id = swapUint32(*reinterpret_cast<const uint32_t*>(&data[6]));
    
    // 参数
    voice_command_.param1 = swapInt32(*reinterpret_cast<const int32_t*>(&data[10]));
    voice_command_.param2 = swapInt32(*reinterpret_cast<const int32_t*>(&data[14]));
    voice_command_.param3 = data[18];
    
    // 默认有效
    voice_command_.is_valid = true;
    
    return true;
}

chest_interfaces::msg::EquipmentState ProtocolParser::getEquipmentState() const {
    return equipment_state_;
}

chest_interfaces::msg::HeadTracking ProtocolParser::getHeadTracking() const {
    return head_tracking_;
}

chest_interfaces::msg::VoiceText ProtocolParser::getVoiceText() const {
    return voice_text_;
}

chest_interfaces::msg::VoiceCommand ProtocolParser::getVoiceCommand() const {
    return voice_command_;
}

std::vector<uint8_t> ProtocolParser::createHeadTrackingPacket(const chest_interfaces::msg::HeadTracking& msg) {
    std::vector<uint8_t> packet(16, 0);
    
    // 帧头
    packet[0] = 0x55;
    packet[1] = 0xAB;
    
    // 数据长度
    uint16_t length = 10;
    uint16_t length_net = swapUint16(length);
    memcpy(&packet[2], &length_net, 2);
    
    // 偏航角 (转换为0.01度)
    int32_t yaw_raw = static_cast<int32_t>(msg.yaw * 180.0 / M_PI * 100.0);
    int32_t yaw_net = swapInt32(yaw_raw);
    memcpy(&packet[4], &yaw_net, 4);
    
    // 俯仰角 (转换为0.01度)
    int32_t pitch_raw = static_cast<int32_t>(msg.pitch * 180.0 / M_PI * 100.0);
    int32_t pitch_net = swapInt32(pitch_raw);
    memcpy(&packet[8], &pitch_net, 4);
    
    // 跟踪状态
    packet[12] = msg.is_tracking ? 1 : 0;
    
    // 置信度
    packet[13] = static_cast<uint8_t>(msg.confidence * 100.0);
    
    // 计算CRC
    uint16_t crc = calculateCrc(packet, 0, 13);
    uint16_t crc_net = swapUint16(crc);
    memcpy(&packet[14], &crc_net, 2);
    
    return packet;
}

std::vector<uint8_t> ProtocolParser::createVoiceTextPacket(const chest_interfaces::msg::VoiceText& msg) {
    // 计算文本长度
    size_t text_length = msg.raw_text.size();
    uint16_t length = 2 + text_length; // Operation + PacketInfo + TextData
    
    std::vector<uint8_t> packet(6 + text_length + 2, 0); // Header + Length + Operation + PacketInfo + TextData + CRC
    
    // 帧头
    packet[0] = 0x55;
    packet[1] = 0xAC;
    
    // 数据长度
    uint16_t length_net = swapUint16(length);
    memcpy(&packet[2], &length_net, 2);
    
    // 操作码
    packet[4] = msg.is_final ? 0x02 : 0x01;
    
    // 包信息
    packet[5] = 0x11; // 单包传输
    
    // 文本数据
    memcpy(&packet[6], msg.raw_text.c_str(), text_length);
    
    // 计算CRC
    uint16_t crc = calculateCrc(packet, 0, 5 + text_length);
    uint16_t crc_net = swapUint16(crc);
    memcpy(&packet[6 + text_length], &crc_net, 2);
    
    return packet;
}

std::vector<uint8_t> ProtocolParser::createVoiceCommandPacket(const chest_interfaces::msg::VoiceCommand& msg) {
    std::vector<uint8_t> packet(21, 0);
    
    // 帧头
    packet[0] = 0x55;
    packet[1] = 0xAA;
    
    // 数据长度
    uint16_t length = 15;
    uint16_t length_net = swapUint16(length);
    memcpy(&packet[2], &length_net, 2);
    
    // 数据类别和操作码
    uint8_t category = (msg.frame_header >> 8) & 0xFF;
    uint8_t operation = msg.frame_header & 0xFF;
    packet[4] = category;
    packet[5] = operation;
    
    // 命令ID
    uint32_t command_id = (msg.command_id == 0) ? next_command_id_++ : msg.command_id;
    uint32_t command_id_net = swapUint32(command_id);
    memcpy(&packet[6], &command_id_net, 4);
    
    // 参数
    int32_t param1_net = swapInt32(msg.param1);
    memcpy(&packet[10], &param1_net, 4);
    
    int32_t param2_net = swapInt32(msg.param2);
    memcpy(&packet[14], &param2_net, 4);
    
    packet[18] = msg.param3;
    
    // 计算CRC
    uint16_t crc = calculateCrc(packet, 0, 19);
    uint16_t crc_net = swapUint16(crc);
    memcpy(&packet[19], &crc_net, 2);
    
    return packet;
}

uint32_t ProtocolParser::getNextCommandId() const {
    if (next_command_id_ == 0) {
        return 0;
    }
    return next_command_id_ - 1;
}

bool ProtocolParser::processAckPacket(const std::vector<uint8_t>& data, uint32_t& command_id, uint8_t& status) {
    if (data.size() != 9) {
        return false;
    }
    
    // 验证帧头
    if (data[0] != 0x55 || data[1] != 0xAD) {
        return false;
    }
    
    // 验证CRC
    if (!verifyCrc(data, 0, 7)) {
        return false;
    }
    
    // 提取命令ID和状态
    command_id = swapUint32(*reinterpret_cast<const uint32_t*>(&data[2]));
    status = data[6];
    
    return true;
}

uint16_t ProtocolParser::calculateCrc(const std::vector<uint8_t>& data, size_t start, size_t end) const {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = start; i < end; i++) {
        crc ^= data[i];
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

bool ProtocolParser::verifyCrc(const std::vector<uint8_t>& data, size_t start, size_t end) const {
    if (end + 2 > data.size()) {
        return false;
    }
    
    uint16_t calculated_crc = calculateCrc(data, start, end);
    uint16_t received_crc = (data[end] << 8) | data[end + 1];
    
    return calculated_crc == received_crc;
}

uint16_t ProtocolParser::swapUint16(uint16_t value) const {
    return (value >> 8) | (value << 8);
}

uint32_t ProtocolParser::swapUint32(uint32_t value) const {
    return ((value >> 24) & 0xFF) |
           ((value >> 8) & 0xFF00) |
           ((value << 8) & 0xFF0000) |
           ((value << 24) & 0xFF000000);
}

int32_t ProtocolParser::swapInt32(int32_t value) const {
    return static_cast<int32_t>(swapUint32(static_cast<uint32_t>(value)));
}

int16_t ProtocolParser::swapInt16(int16_t value) const {
    return static_cast<int16_t>(swapUint16(static_cast<uint16_t>(value)));
}

} // namespace communication