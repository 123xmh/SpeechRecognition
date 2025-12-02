#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "chest_interfaces/msg/equipment_state.hpp"
#include "chest_interfaces/msg/head_tracking.hpp"
#include "chest_interfaces/msg/voice_text.hpp"
#include "chest_interfaces/msg/voice_command.hpp"
#include "communication/protocol_parser.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <map>
#include <functional>

using namespace std::chrono_literals;

namespace communication {

class CommunicationNode : public rclcpp::Node {
public:
    CommunicationNode() : Node("communication_node") {
        // 声明参数
        this->declare_parameter("communication.udp.listen_port", 8888);
        this->declare_parameter("communication.udp.vehicle.ip", "127.0.0.1");
        this->declare_parameter("communication.udp.vehicle.port", 8887);
        this->declare_parameter("communication.command.timeout_ms", 200);
        this->declare_parameter("communication.command.max_retries", 3);
        
        // 获取参数
        int listen_port = this->get_parameter("communication.udp.listen_port").as_int();
        target_ip_ = this->get_parameter("communication.udp.vehicle.ip").as_string();
        int target_port = this->get_parameter("communication.udp.vehicle.port").as_int();
        command_timeout_ms_ = this->get_parameter("communication.command.timeout_ms").as_int();
        max_retries_ = this->get_parameter("communication.command.max_retries").as_int();
        
        // 创建UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
            return;
        }
        
        // 绑定到本地端口
        sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(listen_port);
        
        if (bind(udp_socket_, (sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket to port %d", listen_port);
            close(udp_socket_);
            udp_socket_ = -1;
            return;
        }
        
        // 设置目标地址
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(target_port);
        inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr);
        
        RCLCPP_INFO(this->get_logger(), "UDP communication initialized. Listening on port %d, sending to %s:%d", 
                   listen_port, target_ip_.c_str(), target_port);
        
        // 创建发布者
        equipment_state_pub_ = this->create_publisher<chest_interfaces::msg::EquipmentState>(
            "/tk_chest/equipment/status", 10);
        head_tracking_pub_ = this->create_publisher<chest_interfaces::msg::HeadTracking>(
            "/tk_chest/head_tracking/state", 10);
        voice_text_pub_ = this->create_publisher<chest_interfaces::msg::VoiceText>(
            "/tk_chest/voice_processing/raw_text", 10);
        voice_command_pub_ = this->create_publisher<chest_interfaces::msg::VoiceCommand>(
            "/tk_chest/voice_processing/command", 10);
        
        // 创建订阅者
        head_tracking_sub_ = this->create_subscription<chest_interfaces::msg::HeadTracking>(
            "/tk_chest/head_tracking/state", 10,
            std::bind(&CommunicationNode::headTrackingCallback, this, std::placeholders::_1));
        
        voice_text_sub_ = this->create_subscription<chest_interfaces::msg::VoiceText>(
            "/tk_chest/voice_processing/raw_text", 10,
            std::bind(&CommunicationNode::voiceTextCallback, this, std::placeholders::_1));
        
        voice_command_sub_ = this->create_subscription<chest_interfaces::msg::VoiceCommand>(
            "/tk_chest/voice_processing/command", 10,
            std::bind(&CommunicationNode::voiceCommandCallback, this, std::placeholders::_1));
        
        // 启动接收线程
        receive_thread_ = std::thread(&CommunicationNode::receiveLoop, this);
        
        // 启动命令超时检查定时器
        command_timer_ = this->create_wall_timer(
            100ms, std::bind(&CommunicationNode::checkCommandTimeouts, this));
    }
    
    ~CommunicationNode() {
        if (udp_socket_ >= 0) {
            close(udp_socket_);
        }
        
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    void headTrackingCallback(const chest_interfaces::msg::HeadTracking::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received head tracking data: yaw=%.2f, pitch=%.2f, tracking=%d", 
                    msg->yaw, msg->pitch, msg->is_tracking);
        std::vector<uint8_t> packet = parser_.createHeadTrackingPacket(*msg);
        sendUdpPacket(packet);
    }
    
    void voiceTextCallback(const chest_interfaces::msg::VoiceText::SharedPtr msg) {
        std::vector<uint8_t> packet = parser_.createVoiceTextPacket(*msg);
        sendUdpPacket(packet);
    }
    
    void voiceCommandCallback(const chest_interfaces::msg::VoiceCommand::SharedPtr msg) {
        std::vector<uint8_t> packet = parser_.createVoiceCommandPacket(*msg);
        
        // 提取命令ID
        uint32_t command_id = (msg->command_id == 0) ? parser_.getNextCommandId() : msg->command_id;
        
        // 添加到待确认命令列表
        PendingCommand pending_cmd;
        pending_cmd.packet = packet;
        pending_cmd.send_time = std::chrono::steady_clock::now();
        pending_cmd.retry_count = 0;
        
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_commands_[command_id] = pending_cmd;
        
        // 发送命令
        sendUdpPacket(packet);
    }
    
    void receiveLoop() {
        std::vector<uint8_t> buffer(1024);
        
        while (rclcpp::ok() && udp_socket_ >= 0) {
            sockaddr_in from_addr{};
            socklen_t from_len = sizeof(from_addr);
            
            ssize_t received = recvfrom(udp_socket_, buffer.data(), buffer.size(), 0,
                                      (sockaddr*)&from_addr, &from_len);
            
            if (received > 0) {
                std::vector<uint8_t> data(buffer.begin(), buffer.begin() + received);
                processReceivedData(data);
            } else {
                std::this_thread::sleep_for(10ms);
            }
        }
    }
    
    void processReceivedData(const std::vector<uint8_t>& data) {
        std::string message_type;
        
        if (parser_.parseUdpData(data, message_type)) {
            if (message_type == "vehicle_state") {
                auto msg = parser_.getEquipmentState();
                equipment_state_pub_->publish(msg);
            } else if (message_type == "head_tracking") {
                auto msg = parser_.getHeadTracking();
                head_tracking_pub_->publish(msg);
            } else if (message_type == "voice_text") {
                auto msg = parser_.getVoiceText();
                voice_text_pub_->publish(msg);
            } else if (message_type == "voice_command") {
                auto msg = parser_.getVoiceCommand();
                voice_command_pub_->publish(msg);
            } else if (message_type == "ack") {
                uint32_t command_id;
                uint8_t status;
                
                if (parser_.processAckPacket(data, command_id, status)) {
                    handleAck(command_id, status);
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to parse UDP data");
        }
    }
    
    void handleAck(uint32_t command_id, uint8_t status) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        
        auto it = pending_commands_.find(command_id);
        if (it != pending_commands_.end()) {
            RCLCPP_INFO(this->get_logger(), "Command %u acknowledged with status %u", command_id, status);
            pending_commands_.erase(it);
        }
    }
    
    void checkCommandTimeouts() {
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(command_mutex_);
        
        for (auto it = pending_commands_.begin(); it != pending_commands_.end(); ) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - it->second.send_time).count();
            
            if (elapsed > command_timeout_ms_) {
                if (it->second.retry_count < max_retries_) {
                    // 重发命令
                    it->second.retry_count++;
                    it->second.send_time = now;
                    sendUdpPacket(it->second.packet);
                    RCLCPP_WARN(this->get_logger(), "Retrying command %u (attempt %d/%d)", 
                               it->first, it->second.retry_count, max_retries_);
                    ++it;
                } else {
                    // 超过最大重试次数，移除命令
                    RCLCPP_ERROR(this->get_logger(), "Command %u timed out after %d retries", 
                                it->first, max_retries_);
                    it = pending_commands_.erase(it);
                }
            } else {
                ++it;
            }
        }
    }
    
    void sendUdpPacket(const std::vector<uint8_t>& packet) {
        if (udp_socket_ < 0) {
            RCLCPP_WARN(this->get_logger(), "UDP socket not initialized");
            return;
        }
        
        ssize_t sent = sendto(udp_socket_, packet.data(), packet.size(), 0,
                             (sockaddr*)&target_addr_, sizeof(target_addr_));
        
        if (sent != static_cast<ssize_t>(packet.size())) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send UDP packet: sent %zd/%zu bytes", 
                        sent, packet.size());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "UDP packet sent successfully: %zu bytes to %s:%d", 
                        packet.size(), target_ip_.c_str(), ntohs(target_addr_.sin_port));
        }
    }
    
    struct PendingCommand {
        std::vector<uint8_t> packet;
        std::chrono::steady_clock::time_point send_time;
        int retry_count;
    };
    
    int udp_socket_{-1};
    sockaddr_in target_addr_{};
    std::string target_ip_;
    int command_timeout_ms_{200};
    int max_retries_{3};
    
    ProtocolParser parser_;
    std::thread receive_thread_;
    
    rclcpp::Publisher<chest_interfaces::msg::EquipmentState>::SharedPtr equipment_state_pub_;
    rclcpp::Publisher<chest_interfaces::msg::HeadTracking>::SharedPtr head_tracking_pub_;
    rclcpp::Publisher<chest_interfaces::msg::VoiceText>::SharedPtr voice_text_pub_;
    rclcpp::Publisher<chest_interfaces::msg::VoiceCommand>::SharedPtr voice_command_pub_;
    
    rclcpp::Subscription<chest_interfaces::msg::HeadTracking>::SharedPtr head_tracking_sub_;
    rclcpp::Subscription<chest_interfaces::msg::VoiceText>::SharedPtr voice_text_sub_;
    rclcpp::Subscription<chest_interfaces::msg::VoiceCommand>::SharedPtr voice_command_sub_;
    
    rclcpp::TimerBase::SharedPtr command_timer_;
    std::map<uint32_t, PendingCommand> pending_commands_;
    std::mutex command_mutex_;
};

} // namespace communication

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<communication::CommunicationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}