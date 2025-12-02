#include "ar_display/video_processor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "chest_interfaces/msg/head_tracking.hpp"
#include "chest_interfaces/msg/button_state.hpp"
#include "chest_interfaces/msg/equipment_state.hpp"
#include "chest_interfaces/msg/video_source.hpp"
#include "chest_interfaces/msg/voice_text.hpp"
#include "chest_interfaces/msg/voice_command.hpp"
#include <ncursesw/ncurses.h>
#include <locale.h>
#include <unistd.h>
#include <codecvt>
#include <string>
#include <sstream>

using namespace std::chrono_literals;

class ARDisplayNode : public rclcpp::Node
{
public:
    ARDisplayNode() : Node("ar_display_node")
    {
        // 设置locale以支持宽字符
        setlocale(LC_ALL, "");

        // 初始化视频处理器
        video_processor_ = std::make_unique<VideoProcessor>();
        video_processor_->start();

        // 订阅头动信息
        head_tracking_sub_ = create_subscription<chest_interfaces::msg::HeadTracking>(
            "/tk_chest/head_tracking/state", 10,
            [this](const chest_interfaces::msg::HeadTracking::SharedPtr msg)
            {
                // 使用弧度值
                video_processor_->update_head_motion(msg->pitch, msg->yaw);
                last_head_tracking_ = *msg;
            });

        // 订阅按钮状态
        button_sub_ = create_subscription<chest_interfaces::msg::ButtonState>(
            "/tk_chest/button/state", 10,
            [this](const chest_interfaces::msg::ButtonState::SharedPtr msg)
            {
                // 头显开关
                is_active_ = msg->hmd_enabled;

                // 设置视频源
                video_processor_->set_current_source(msg->hmd_video_source);

                // 设置叠加模式
                video_processor_->set_overlay_mode(msg->hmd_overlay_mode);
                
                button_state_ = *msg;
            });

        // 订阅语音文本
        voice_text_sub_ = create_subscription<chest_interfaces::msg::VoiceText>(
            "/tk_chest/voice_processing/raw_text", 10,
            [this](const chest_interfaces::msg::VoiceText::SharedPtr msg)
            {
                last_voice_text_ = *msg;
                // 限制文本长度，防止显示溢出
                if (last_voice_text_.raw_text.length() > 50) {
                    last_voice_text_.raw_text = last_voice_text_.raw_text.substr(0, 47) + "...";
                }
            });

        // 订阅语音指令
        voice_command_sub_ = create_subscription<chest_interfaces::msg::VoiceCommand>(
            "/tk_chest/voice_processing/command", 10,
            [this](const chest_interfaces::msg::VoiceCommand::SharedPtr msg)
            {
                last_voice_command_ = *msg;
            });

        // 订阅视频源
        video_source_sub_ = create_subscription<chest_interfaces::msg::VideoSource>(
            "/tk_chest/video/source/selected", 10,
            [this](const chest_interfaces::msg::VideoSource::SharedPtr msg)
            {
                current_video_source_ = *msg;
                video_processor_->update_video_source(*msg);
            });

        // 订阅车辆状态
        equipment_state_sub_ = create_subscription<chest_interfaces::msg::EquipmentState>(
            "/tk_chest/equipment/status", 10,
            [this](const chest_interfaces::msg::EquipmentState::SharedPtr msg)
            {
                equipment_state_ = *msg;
                video_processor_->update_equipment_state(*msg);
            });

        // 初始化ncurses界面
        init_ncurses();

        // 创建定时器更新UI
        ui_timer_ = create_wall_timer(100ms, [this]()
                                      { update_ui(); });
    }

    ~ARDisplayNode()
    {
        video_processor_->stop();
        endwin(); // 关闭ncurses
    }

private:
    void init_ncurses()
    {
        // 初始化ncurses
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        curs_set(0); // 隐藏光标

        // 检查颜色支持
        if (has_colors())
        {
            start_color();
            init_pair(1, COLOR_GREEN, COLOR_BLACK);
            init_pair(2, COLOR_YELLOW, COLOR_BLACK);
            init_pair(3, COLOR_CYAN, COLOR_BLACK);
            init_pair(4, COLOR_RED, COLOR_BLACK);
            init_pair(5, COLOR_MAGENTA, COLOR_BLACK);
            init_pair(6, COLOR_BLUE, COLOR_BLACK);
        }

        // 创建状态窗口
        status_win = newwin(24, 80, 0, 0);
        box(status_win, 0, 0);
        wrefresh(status_win);
    }

    std::wstring to_wide_string(const std::string &str)
    {
        try {
            std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
            return converter.from_bytes(str);
        } catch (...) {
            return L"转换错误";
        }
    }

    void update_ui()
    {
        // 获取当前状态
        VideoProcessor::DisplayStatus status = video_processor_->get_current_status();

        // 清空状态窗口
        wclear(status_win);
        box(status_win, 0, 0);

        // 显示标题
        wattron(status_win, COLOR_PAIR(1) | A_BOLD);
        mvwprintw(status_win, 0, 2, " AR显示系统状态 ");
        wattroff(status_win, COLOR_PAIR(1) | A_BOLD);

        int line = 1;
        
        // 显示系统状态
        wattron(status_win, COLOR_PAIR(3));
        mvwprintw(status_win, line++, 2, "■ 系统状态: %s", is_active_ ? "激活" : "待机");
        wattroff(status_win, COLOR_PAIR(3));

        // 显示视频源信息
        wattron(status_win, COLOR_PAIR(2));
        mvwprintw(status_win, line++, 2, "■ 当前视频源: %s", current_video_source_.source_name.c_str());
        mvwprintw(status_win, line++, 2, "■ 视频源详情: %s %dx%d %dfps %s", 
                 current_video_source_.ip_address.c_str(),
                 current_video_source_.width,
                 current_video_source_.height,
                 current_video_source_.fps,
                 current_video_source_.is_available ? "可用" : "不可用");
        wattroff(status_win, COLOR_PAIR(2));

        // 显示叠加层模式
        const char *overlay_modes[] = {"无", "简洁", "完整"};
        mvwprintw(status_win, line++, 2, "■ 叠加模式: %s", overlay_modes[button_state_.hmd_overlay_mode]);

        // 显示头部姿态信息（弧度值）
        // 将弧度转换为角度
        double pitch_deg = last_head_tracking_.pitch * (180.0 / M_PI);
        double yaw_deg = last_head_tracking_.yaw * (180.0 / M_PI);
        mvwprintw(status_win, line++, 2, "■ 头部姿态 - 俯仰角: %.2f° (%.2f rad)", pitch_deg, last_head_tracking_.pitch);
        mvwprintw(status_win, line++, 2, "■ 头部姿态 - 偏航角: %.2f° (%.2f rad)", yaw_deg, last_head_tracking_.yaw);
        mvwprintw(status_win, line++, 2, "■ 跟踪状态: %s 置信度: %.2f", 
                 last_head_tracking_.is_tracking ? "开启" : "关闭", 
                 last_head_tracking_.confidence);

        // 显示语音文本
        wattron(status_win, COLOR_PAIR(5));
        mvwprintw(status_win, line++, 2, "■ 语音文本: %s", last_voice_text_.raw_text.c_str());
        mvwprintw(status_win, line++, 2, "■ 文本状态: %s 置信度: %.2f", 
                 last_voice_text_.is_final ? "最终" : "中间", 
                 last_voice_text_.confidence);
        wattroff(status_win, COLOR_PAIR(5));

        // 显示语音指令
        if (last_voice_command_.is_valid) {
            wattron(status_win, COLOR_PAIR(1) | A_BOLD);
            mvwprintw(status_win, line++, 2, "■ 语音指令: 有效 (ID: %u)", last_voice_command_.command_id);
            mvwprintw(status_win, line++, 2, "■ 指令详情: 分类:%d 操作:%d 参数:%d,%d,%d", 
                     last_voice_command_.frame_header >> 8,
                     last_voice_command_.frame_header & 0xFF,
                     last_voice_command_.param1,
                     last_voice_command_.param2,
                     last_voice_command_.param3);
            wattroff(status_win, COLOR_PAIR(1) | A_BOLD);
        } else {
            wattron(status_win, COLOR_PAIR(4));
            mvwprintw(status_win, line++, 2, "■ 语音指令: 无效");
            wattroff(status_win, COLOR_PAIR(4));
        }

        // 显示车辆状态
        wattron(status_win, COLOR_PAIR(6));
        mvwprintw(status_win, line++, 2, "■ 车辆位置: 经度: %.6f 纬度: %.6f", 
                 equipment_state_.longitude, equipment_state_.latitude);
        mvwprintw(status_win, line++, 2, "■ 车速/油量/电量: %.1f km/h  %.1f%%  %.1f%%",
                 equipment_state_.speed,
                 equipment_state_.fuel_level * 100,
                 equipment_state_.battery_level * 100);
        
        // 车辆姿态
        mvwprintw(status_win, line++, 2, "■ 车辆姿态: Roll: %.2f° Pitch: %.2f° Yaw: %.2f°",
                 equipment_state_.roll * (180.0 / M_PI),
                 equipment_state_.pitch * (180.0 / M_PI),
                 equipment_state_.yaw * (180.0 / M_PI));
        
        // 光电转台
        mvwprintw(status_win, line++, 2, "■ 光电转台: Pitch: %.2f° Yaw: %.2f° %s",
                 equipment_state_.gimbal_pitch * (180.0 / M_PI),
                 equipment_state_.gimbal_yaw * (180.0 / M_PI),
                 equipment_state_.gimbal_is_active ? "激活" : "未激活");
        
        // 弹药状态
        std::string ammo_str;
        if (equipment_state_.ammo_types.empty()) {
            ammo_str = "无";
        } else {
            for (size_t i = 0; i < equipment_state_.ammo_types.size() && i < equipment_state_.ammo_counts.size(); ++i) {
                if (i != 0) ammo_str += ", ";
                ammo_str += equipment_state_.ammo_types[i] + ":" + std::to_string(equipment_state_.ammo_counts[i]);
            }
        }
        // 限制弹药状态显示长度
        if (ammo_str.length() > 50) {
            ammo_str = ammo_str.substr(0, 47) + "...";
        }
        mvwprintw(status_win, line++, 2, "■ 弹药状态: %s", ammo_str.c_str());
        
        // 告警信息
        if (equipment_state_.warnings.empty()) {
            mvwprintw(status_win, line++, 2, "■ 告警信息: 无");
        } else {
            // 只显示第一个告警
            std::string warning = equipment_state_.warnings[0];
            if (warning.length() > 50) {
                warning = warning.substr(0, 47) + "...";
            }
            wattron(status_win, COLOR_PAIR(4) | A_BOLD);
            mvwprintw(status_win, line++, 2, "■ 告警信息: %s", warning.c_str());
            wattroff(status_win, COLOR_PAIR(4) | A_BOLD);
        }
        wattroff(status_win, COLOR_PAIR(6));

        // 刷新窗口
        wrefresh(status_win);
    }

    rclcpp::Subscription<chest_interfaces::msg::HeadTracking>::SharedPtr head_tracking_sub_;
    rclcpp::Subscription<chest_interfaces::msg::ButtonState>::SharedPtr button_sub_;
    rclcpp::Subscription<chest_interfaces::msg::VoiceText>::SharedPtr voice_text_sub_;
    rclcpp::Subscription<chest_interfaces::msg::VoiceCommand>::SharedPtr voice_command_sub_;
    rclcpp::Subscription<chest_interfaces::msg::VideoSource>::SharedPtr video_source_sub_;
    rclcpp::Subscription<chest_interfaces::msg::EquipmentState>::SharedPtr equipment_state_sub_;
    rclcpp::TimerBase::SharedPtr ui_timer_;

    std::unique_ptr<VideoProcessor> video_processor_;
    bool is_active_ = true;

    // 存储最新消息
    chest_interfaces::msg::HeadTracking last_head_tracking_;
    chest_interfaces::msg::ButtonState button_state_;
    chest_interfaces::msg::VoiceText last_voice_text_;
    chest_interfaces::msg::VoiceCommand last_voice_command_;
    chest_interfaces::msg::VideoSource current_video_source_;
    chest_interfaces::msg::EquipmentState equipment_state_;

    WINDOW *status_win;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ARDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}