#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <memory>
#include "chest_interfaces/msg/equipment_state.hpp"
#include "chest_interfaces/msg/head_tracking.hpp"
#include "chest_interfaces/msg/voice_text.hpp"
#include "chest_interfaces/msg/voice_command.hpp"

namespace communication {

class ProtocolParser {
public:
    ProtocolParser();
    ~ProtocolParser();

    // 解析接收到的UDP数据
    bool parseUdpData(const std::vector<uint8_t>& data, std::string& message_type);
    
    // 获取解析后的消息
    chest_interfaces::msg::EquipmentState getEquipmentState() const;
    chest_interfaces::msg::HeadTracking getHeadTracking() const;
    chest_interfaces::msg::VoiceText getVoiceText() const;
    chest_interfaces::msg::VoiceCommand getVoiceCommand() const;
    uint32_t getNextCommandId() const;
    
    // 创建要发送的UDP数据包
    std::vector<uint8_t> createHeadTrackingPacket(const chest_interfaces::msg::HeadTracking& msg);
    std::vector<uint8_t> createVoiceTextPacket(const chest_interfaces::msg::VoiceText& msg);
    std::vector<uint8_t> createVoiceCommandPacket(const chest_interfaces::msg::VoiceCommand& msg);
    
    // 处理确认包
    bool processAckPacket(const std::vector<uint8_t>& data, uint32_t& command_id, uint8_t& status);

private:
    // CRC计算
    uint16_t calculateCrc(const std::vector<uint8_t>& data, size_t start, size_t end) const;
    bool verifyCrc(const std::vector<uint8_t>& data, size_t start, size_t end) const;
    
    // 数据类型解析
    bool parseVehicleState(const std::vector<uint8_t>& data);
    bool parseHeadTracking(const std::vector<uint8_t>& data);
    bool parseVoiceText(const std::vector<uint8_t>& data);
    bool parseVoiceCommand(const std::vector<uint8_t>& data);
    
    // 字节序转换
    uint16_t swapUint16(uint16_t value) const;
    uint32_t swapUint32(uint32_t value) const;
    int32_t swapInt32(int32_t value) const;
    int16_t swapInt16(int16_t value) const;
    
    // 内部状态
    chest_interfaces::msg::EquipmentState equipment_state_;
    chest_interfaces::msg::HeadTracking head_tracking_;
    chest_interfaces::msg::VoiceText voice_text_;
    chest_interfaces::msg::VoiceCommand voice_command_;
    
    // 命令ID管理
    uint32_t next_command_id_;
};

} // namespace communication