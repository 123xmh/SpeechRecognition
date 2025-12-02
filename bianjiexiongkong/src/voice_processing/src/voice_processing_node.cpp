#include "rclcpp/rclcpp.hpp"
#include "voice_processing/keyword_matcher.hpp"
#include "chest_interfaces/msg/voice_command.hpp"
#include "chest_interfaces/msg/voice_text.hpp"
#include "chest_interfaces/msg/button_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <random>
#include <map>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

class VoiceProcessingNode : public rclcpp::Node
{
public:
    VoiceProcessingNode() : Node("voice_processing_node"), is_active_(false), debug_(false), last_button_state_(false)
    {
        // 声明调试参数
        this->declare_parameter("debug", false);
        debug_ = this->get_parameter("debug").as_bool();
        
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
        }

        // 订阅麦克风数据
        mic_sub_ = create_subscription<std_msgs::msg::String>(
            "/tk_chest/microphone/transcript", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (is_active_)
                {
                    process_speech(msg->data);
                }
                else if (debug_) {
                    RCLCPP_DEBUG(this->get_logger(), "Received speech but voice processing is inactive: %s", msg->data.c_str());
                }
            });

        // 订阅按键状态
        button_sub_ = create_subscription<chest_interfaces::msg::ButtonState>(
            "/tk_chest/button/state", 10,
            [this](const chest_interfaces::msg::ButtonState::SharedPtr msg)
            {
                // 侧按键2控制语音开关
                bool new_state = msg->voice_command_enabled;
                if (new_state != last_button_state_) {
                    is_active_ = new_state;
                    last_button_state_ = new_state;
                    RCLCPP_INFO(this->get_logger(), "Voice processing %s", is_active_ ? "enabled" : "disabled");
                }
            });

        // 发布语音指令
        command_pub_ = create_publisher<chest_interfaces::msg::VoiceCommand>(
            "/tk_chest/voice_processing/command", 10);

        // 发布语音文本
        text_pub_ = create_publisher<chest_interfaces::msg::VoiceText>(
            "/tk_chest/voice_processing/raw_text", 10);

        // 初始化命令映射
        init_command_mapping();

        // 模拟麦克风输入
        this->declare_parameter("simulate_microphone", true);
        if (this->get_parameter("simulate_microphone").as_bool())
        {
            sim_mic_timer_ = create_wall_timer(5s, [this]()
                                               {
                if (!is_active_) return;
                
                auto mic_msg = std_msgs::msg::String();
                mic_msg.data = sim_sentences_[dist_(gen_) % sim_sentences_.size()];
                if (debug_) {
                    RCLCPP_DEBUG(this->get_logger(), "Simulating speech: %s", mic_msg.data.c_str());
                }
                mic_pub_->publish(mic_msg); });
        }

        mic_pub_ = this->create_publisher<std_msgs::msg::String>("/tk_chest/microphone/transcript", 10);
        RCLCPP_INFO(this->get_logger(), "Voice Processing Node initialized");
    }

private:
    void init_command_mapping() {
        // 初始化命令到frame_header的映射
        command_mapping_ = {
            {"start",     {(1 << 8) | 1, 0, 0, 0}},      // 分类码1，操作码1
            {"stop",      {(1 << 8) | 2, 0, 0, 0}},      // 分类码1，操作码2
            {"on",        {(2 << 8) | 1, 0, 0, 0}},      // 分类码2，操作码1
            {"off",       {(2 << 8) | 2, 0, 0, 0}},      // 分类码2，操作码2
            {"switch",    {(3 << 8) | 1, 0, 0, 0}},      // 分类码3，操作码1
            {"select",    {(3 << 8) | 2, 1, 0, 0}},      // 分类码3，操作码2，param1=1
            {"confirm",   {(4 << 8) | 1, 0, 0, 0}},      // 分类码4，操作码1
            {"cancel",    {(4 << 8) | 2, 0, 0, 0}},      // 分类码4，操作码2
            {"forward",   {(5 << 8) | 1, 0, 0, 0}},      // 分类码5，操作码1
            {"backward",  {(5 << 8) | 2, 0, 0, 0}},      // 分类码5，操作码2
            {"turn_left", {(6 << 8) | 1, 0, 0, 0}},      // 分类码6，操作码1
            {"turn_right",{(6 << 8) | 2, 0, 0, 0}},      // 分类码6，操作码2
            {"accelerate",{(7 << 8) | 1, 0, 0, 0}},      // 分类码7，操作码1
            {"decelerate",{(7 << 8) | 2, 0, 0, 0}},      // 分类码7，操作码2
            {"status",    {(8 << 8) | 1, 0, 0, 0}},      // 分类码8，操作码1
            {"report",    {(8 << 8) | 2, 0, 0, 0}},      // 分类码8，操作码2
            {"target",    {(9 << 8) | 1, 0, 0, 0}},      // 分类码9，操作码1
            {"attack",    {(10 << 8) | 1, 0, 0, 0}},     // 分类码10，操作码1
            {"defend",    {(10 << 8) | 2, 0, 0, 0}},     // 分类码10，操作码2
            {"recon",     {(11 << 8) | 1, 0, 0, 0}}      // 分类码11，操作码1
        };
    }

    std::string format_voice_text(const chest_interfaces::msg::VoiceText& msg) {
        std::stringstream ss;
        ss << "VoiceText{"
           << "stamp:" << msg.header.stamp.sec << "." 
           << std::setw(9) << std::setfill('0') << msg.header.stamp.nanosec
           << ", text:'" << msg.raw_text
           << "', final:" << (msg.is_final ? "true" : "false")
           << ", confidence:" << std::fixed << std::setprecision(2) << msg.confidence
           << "}";
        return ss.str();
    }

    std::string format_voice_command(const chest_interfaces::msg::VoiceCommand& msg) {
        std::stringstream ss;
        ss << "VoiceCommand{"
           << "stamp:" << msg.header.stamp.sec << "." 
           << std::setw(9) << std::setfill('0') << msg.header.stamp.nanosec
           << ", frame_header:0x" << std::hex << std::setw(4) << std::setfill('0') << msg.frame_header << std::dec
           << ", id:" << msg.command_id
           << ", params:[" << msg.param1 << ", " << msg.param2 << ", " << static_cast<int>(msg.param3)
           << "], valid:" << (msg.is_valid ? "true" : "false")
           << "}";
        return ss.str();
    }

    void process_speech(const std::string &text)
    {
        if (debug_) {
            RCLCPP_DEBUG(this->get_logger(), "Processing speech: %s", text.c_str());
        }

        // 发布VoiceText消息
        auto voice_text_msg = chest_interfaces::msg::VoiceText();
        voice_text_msg.header.stamp = this->now();
        voice_text_msg.raw_text = text;
        voice_text_msg.is_final = true;
        voice_text_msg.confidence = 0.8 + (dist_(gen_) % 20) / 100.0; // 随机置信度0.8-1.0
        text_pub_->publish(voice_text_msg);

        // 输出完整的VoiceText消息
        RCLCPP_INFO(this->get_logger(), "Published: %s", format_voice_text(voice_text_msg).c_str());

        // 匹配命令并发布VoiceCommand消息
        std::string command_str = matcher_.match_command(text, debug_);
        auto command_msg = chest_interfaces::msg::VoiceCommand();
        command_msg.header.stamp = this->now();

        // 根据匹配结果设置命令参数
        if (!command_str.empty())
        {
            auto it = command_mapping_.find(command_str);
            if (it != command_mapping_.end()) {
                const auto& mapping = it->second;
                command_msg.frame_header = mapping.frame_header;
                command_msg.param1 = mapping.param1;
                command_msg.param2 = mapping.param2;
                command_msg.param3 = mapping.param3;
                command_msg.is_valid = true;
                
                // 生成唯一命令ID
                static uint32_t command_id_counter = 0;
                command_msg.command_id = command_id_counter++;
            } else {
                command_msg.is_valid = false;
                if (debug_) {
                    RCLCPP_DEBUG(this->get_logger(), "Command '%s' not found in mapping", command_str.c_str());
                }
            }
        }
        else
        {
            command_msg.is_valid = false;
            if (debug_) {
                RCLCPP_DEBUG(this->get_logger(), "No command matched from speech: %s", text.c_str());
            }
        }

        // 发布VoiceCommand消息
        command_pub_->publish(command_msg);
        
        // 输出完整的VoiceCommand消息
        RCLCPP_INFO(this->get_logger(), "Published: %s", format_voice_command(command_msg).c_str());
        
        // 额外输出命令摘要（便于快速查看）
        if (command_msg.is_valid) {
            RCLCPP_INFO(this->get_logger(), "Command recognized: %s", command_str.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "No valid command recognized");
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mic_sub_;
    rclcpp::Subscription<chest_interfaces::msg::ButtonState>::SharedPtr button_sub_;
    rclcpp::Publisher<chest_interfaces::msg::VoiceCommand>::SharedPtr command_pub_;
    rclcpp::Publisher<chest_interfaces::msg::VoiceText>::SharedPtr text_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mic_pub_;
    rclcpp::TimerBase::SharedPtr sim_mic_timer_;

    bool is_active_;
    bool debug_;
    bool last_button_state_; // 记录上一次的按钮状态
    KeywordMatcher matcher_;

    // 命令映射结构
    struct CommandMapping {
        uint16_t frame_header;
        int32_t param1;
        int32_t param2;
        uint8_t param3;
    };
    std::map<std::string, CommandMapping> command_mapping_;

    // 更丰富的仿真句子
    std::vector<std::string> sim_sentences_ = {
        "启动系统",
        "停止运行",
        "开启设备",
        "关闭电源",
        "切换模式",
        "选择一号目标",
        "确认操作",
        "取消任务",
        "前进",
        "后退",
        "左转三十度",
        "右转四十五度",
        "加速前进",
        "减速慢行",
        "报告状态",
        "请求状态报告",
        "锁定目标",
        "发起攻击",
        "进入防御模式",
        "执行侦察任务",
        "今天的天气不错",  // 无效指令
        "你好世界"        // 无效指令
    };
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    std::uniform_int_distribution<> dist_{0, 100};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoiceProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}