#include "rclcpp/rclcpp.hpp"
#include "chest_interfaces/msg/head_tracking.hpp"
#include "chest_interfaces/msg/voice_command.hpp"
#include "chest_interfaces/msg/voice_text.hpp"
#include "chest_interfaces/msg/button_state.hpp"
#include "chest_interfaces/msg/equipment_state.hpp"
#include "chest_interfaces/msg/video_source.hpp"
#include "std_msgs/msg/string.hpp"

#include <ncursesw/ncurses.h>
#include <locale>
#include <vector>
#include <string>
#include <codecvt>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class ControllerDisplayNode : public rclcpp::Node
{
public:
    ControllerDisplayNode() : Node("controller_display_node"), 
                             head_tracking_msg_count_(0), 
                             head_tracking_freq_(0.0)
    {
        std::setlocale(LC_ALL, "");
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
        curs_set(0);
        start_color();
        init_pair(1, COLOR_GREEN, COLOR_BLACK);
        init_pair(2, COLOR_YELLOW, COLOR_BLACK);
        init_pair(3, COLOR_CYAN, COLOR_BLACK);
        init_pair(4, COLOR_RED, COLOR_BLACK);
        init_pair(5, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(6, COLOR_BLUE, COLOR_BLACK);

        // 初始化头动数据
        last_head_tracking_.pitch = 0.0;
        last_head_tracking_.yaw = 0.0;
        last_head_tracking_.is_tracking = false;
        last_head_tracking_.confidence = 0.0;

        // 初始化视频源数据
        current_video_source_.id = 0;
        current_video_source_.ip_address = "未知";
        current_video_source_.source_name = "未知";
        current_video_source_.fps = 0;
        current_video_source_.width = 0;
        current_video_source_.height = 0;
        current_video_source_.encoding = "未知";
        current_video_source_.is_available = false;

        // 订阅语音文本
        voice_text_sub_ = create_subscription<chest_interfaces::msg::VoiceText>(
            "/tk_chest/voice_processing/raw_text", 10,
            [this](const chest_interfaces::msg::VoiceText::SharedPtr msg)
            {
                last_speech_ = msg->raw_text;
                // 限制文本长度，防止显示溢出
                if (last_speech_.length() > 50) {
                    last_speech_ = last_speech_.substr(0, 47) + "...";
                }
                update_ui();
            });

        // 订阅语音指令
        voice_command_sub_ = create_subscription<chest_interfaces::msg::VoiceCommand>(
            "/tk_chest/voice_processing/command", 10,
            [this](const chest_interfaces::msg::VoiceCommand::SharedPtr msg)
            {
                if (msg->is_valid) {
                    last_command_ = "已识别指令";
                    command_id_ = msg->command_id;
                } else {
                    last_command_ = "未识别";
                    command_id_ = 0;
                }
                update_ui();
            });

        // 订阅按键状态
        button_sub_ = create_subscription<chest_interfaces::msg::ButtonState>(
            "/tk_chest/button/state", 10,
            [this](const chest_interfaces::msg::ButtonState::SharedPtr msg)
            {
                // 更新按钮状态
                button_states_.head_tracking_enabled = msg->head_tracking_enabled;
                button_states_.voice_command_enabled = msg->voice_command_enabled;
                button_states_.front_screen_enabled = msg->front_screen_enabled;
                button_states_.front_screen_mode = msg->front_screen_mode;
                button_states_.hmd_video_source = msg->hmd_video_source;
                button_states_.hmd_overlay_mode = msg->hmd_overlay_mode;
                button_states_.hmd_enabled = msg->hmd_enabled;
                update_ui();
            });

        // 订阅头动信息
        head_motion_sub_ = create_subscription<chest_interfaces::msg::HeadTracking>(
            "/tk_chest/head_tracking/state", 10,
            [this](const chest_interfaces::msg::HeadTracking::SharedPtr msg)
            {
                last_head_tracking_ = *msg;

                // 更新频率计数器
                head_tracking_msg_count_++;
                update_ui();
            });

        // 订阅视频源 - 修改为正确的主题名称
        video_source_sub_ = create_subscription<chest_interfaces::msg::VideoSource>(
            "/tk_chest/video/source/selected", 10,
            [this](const chest_interfaces::msg::VideoSource::SharedPtr msg)
            {
                // 直接存储视频源信息
                current_video_source_ = *msg;
                update_ui();
            });

        // 订阅系统状态
        equipment_state_sub_ = create_subscription<chest_interfaces::msg::EquipmentState>(
            "/tk_chest/equipment/status", 10,
            [this](const chest_interfaces::msg::EquipmentState::SharedPtr msg)
            {
                equipment_state_ = *msg;
                update_ui();
            });

        // 创建UI更新定时器
        ui_timer_ = create_wall_timer(100ms, [this]()
                                      { update_ui(); });

        // 创建频率计算定时器
        freq_calc_timer_ = create_wall_timer(1s, [this]()
                                             {
            // 计算接收频率 (Hz)
            head_tracking_freq_ = head_tracking_msg_count_;
            head_tracking_msg_count_ = 0;
            update_ui(); });

        init_ui();
        RCLCPP_INFO(this->get_logger(), "Controller Display Node initialized");
    }

    ~ControllerDisplayNode()
    {
        endwin();
    }

private:
    std::wstring to_wide_string(const std::string &str)
    {
        try {
            std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
            return converter.from_bytes(str);
        } catch (...) {
            return L"转换错误";
        }
    }

    void init_ui()
    {
        clear();

        attron(COLOR_PAIR(2) | A_BOLD);
        mvaddwstr(0, 0, to_wide_string("===== 控制器显示模块 =====").c_str());
        attroff(COLOR_PAIR(2) | A_BOLD);

        // 初始化状态行
        status_lines_ = {
            "侧按键1 (头动开关): ",
            "侧按键2 (语音开关): ",
            "正按键1 (正面屏开关): ",
            "正按键2 (正面屏模式): ",
            "正按键3 (视频源选择): ",
            "正按键4 (头显叠加模式): ",
            "正按键5 (头显开关): "};

        last_command_ = "无";
        last_speech_ = "等待语音输入...";

        update_ui();
        refresh();
    }

    void update_ui()
    {
        clear();

        // 显示标题
        attron(COLOR_PAIR(2) | A_BOLD);
        mvaddwstr(0, 0, to_wide_string("===== 控制器显示模块 =====").c_str());
        attroff(COLOR_PAIR(2) | A_BOLD);

        // 头部姿态显示
        mvaddwstr(2, 0, to_wide_string("头部姿态:").c_str());
        attron(COLOR_PAIR(1));

        // 将弧度转换为角度
        double pitch_deg = last_head_tracking_.pitch * (180.0 / M_PI);
        double yaw_deg = last_head_tracking_.yaw * (180.0 / M_PI);

        mvprintw(2, 15, "Pitch: %-6.2f°   Yaw: %-6.2f°  跟踪: %s  置信度: %.2f",
                 pitch_deg,
                 yaw_deg,
                 last_head_tracking_.is_tracking ? "是" : "否",
                 last_head_tracking_.confidence);
        attroff(COLOR_PAIR(1));

        // 显示接收频率
        mvaddwstr(3, 0, to_wide_string("接收频率:").c_str());
        attron(COLOR_PAIR(3));
        mvprintw(3, 15, "%.1f Hz", head_tracking_freq_);
        attroff(COLOR_PAIR(3));

        // 按键状态
        mvaddwstr(5, 0, to_wide_string("按键状态:").c_str());

        // 侧按键1: 头动开关
        mvaddwstr(6, 0, to_wide_string(status_lines_[0]).c_str());
        attron(COLOR_PAIR(button_states_.head_tracking_enabled ? 1 : 4));
        addstr(button_states_.head_tracking_enabled ? "开" : "关");
        attroff(COLOR_PAIR(button_states_.head_tracking_enabled ? 1 : 4));

        // 侧按键2: 语音开关
        mvaddwstr(7, 0, to_wide_string(status_lines_[1]).c_str());
        attron(COLOR_PAIR(button_states_.voice_command_enabled ? 1 : 4));
        addstr(button_states_.voice_command_enabled ? "开" : "关");
        attroff(COLOR_PAIR(button_states_.voice_command_enabled ? 1 : 4));

        // 正按键1: 正面屏开关
        mvaddwstr(8, 0, to_wide_string(status_lines_[2]).c_str());
        attron(COLOR_PAIR(button_states_.front_screen_enabled ? 1 : 4));
        addstr(button_states_.front_screen_enabled ? "开" : "关");
        attroff(COLOR_PAIR(button_states_.front_screen_enabled ? 1 : 4));

        // 正按键2: 正面屏模式
        mvaddwstr(9, 0, to_wide_string(status_lines_[3]).c_str());
        attron(COLOR_PAIR(1));
        switch (button_states_.front_screen_mode)
        {
        case 0:
            addstr("默认");
            break;
        case 1:
            addstr("模式A");
            break;
        case 2:
            addstr("模式B");
            break;
        case 3:
            addstr("模式C");
            break;
        default:
            addstr("未知");
        }
        attroff(COLOR_PAIR(1));

        // 正按键3: 视频源选择
        mvaddwstr(10, 0, to_wide_string(status_lines_[4]).c_str());
        attron(COLOR_PAIR(1));
        printw("%d", button_states_.hmd_video_source);
        attroff(COLOR_PAIR(1));

        // 正按键4: 头显叠加模式
        mvaddwstr(11, 0, to_wide_string(status_lines_[5]).c_str());
        attron(COLOR_PAIR(1));
        switch (button_states_.hmd_overlay_mode)
        {
        case 0:
            addstr("无叠加");
            break;
        case 1:
            addstr("最小叠加");
            break;
        case 2:
            addstr("全叠加");
            break;
        default:
            addstr("未知");
        }
        attroff(COLOR_PAIR(1));

        // 正按键5: 头显开关
        mvaddwstr(12, 0, to_wide_string(status_lines_[6]).c_str());
        attron(COLOR_PAIR(button_states_.hmd_enabled ? 1 : 4));
        addstr(button_states_.hmd_enabled ? "开" : "关");
        attroff(COLOR_PAIR(button_states_.hmd_enabled ? 1 : 4));

        // 视频源显示 - 简化版本
        mvaddwstr(14, 0, to_wide_string("视频源信息:").c_str());
        attron(COLOR_PAIR(5) | A_BOLD);
        mvprintw(14, 15, "ID:%d %s (%s) %dx%d %dfps %s %s",
                 current_video_source_.id,
                 current_video_source_.source_name.c_str(),
                 current_video_source_.ip_address.c_str(),
                 current_video_source_.width,
                 current_video_source_.height,
                 current_video_source_.fps,
                 current_video_source_.encoding.c_str(),
                 current_video_source_.is_available ? "可用" : "不可用");
        attroff(COLOR_PAIR(5) | A_BOLD);

        // 语音识别文本
        mvaddwstr(16, 0, to_wide_string("语音识别文本:").c_str());
        attron(COLOR_PAIR(3));
        mvaddwstr(16, 15, to_wide_string(last_speech_).c_str());
        attroff(COLOR_PAIR(3));

        // 语音指令
        mvaddwstr(18, 0, to_wide_string("语音指令:").c_str());
        if (last_command_ == "无" || last_command_.empty())
        {
            attron(COLOR_PAIR(4));
            mvaddwstr(18, 15, to_wide_string("未识别").c_str());
            attroff(COLOR_PAIR(4));
        }
        else
        {
            attron(COLOR_PAIR(1) | A_BOLD);
            mvprintw(18, 15, "%s (ID: %u)", last_command_.c_str(), command_id_);
            attroff(COLOR_PAIR(1) | A_BOLD);
        }

        // 激活状态
        mvaddwstr(20, 0, to_wide_string("语音激活状态:").c_str());
        if (button_states_.voice_command_enabled)
        {
            attron(COLOR_PAIR(1) | A_BOLD);
            mvaddwstr(20, 15, to_wide_string("已激活").c_str());
            attroff(COLOR_PAIR(1) | A_BOLD);
        }
        else
        {
            attron(COLOR_PAIR(4));
            mvaddwstr(20, 15, to_wide_string("未激活").c_str());
            attroff(COLOR_PAIR(4));
        }

        // 分隔线
        mvhline(21, 0, ACS_HLINE, COLS);

        // 装备状态显示
        mvaddwstr(22, 0, to_wide_string("装备状态:").c_str());
        attron(COLOR_PAIR(6) | A_BOLD);
        mvaddwstr(22, 15, to_wide_string("================").c_str());
        attroff(COLOR_PAIR(6) | A_BOLD);

        // 车辆位置
        mvaddwstr(23, 0, to_wide_string("车辆位置:").c_str());
        attron(COLOR_PAIR(3));
        mvprintw(23, 15, "经度: %.6f  纬度: %.6f", equipment_state_.longitude, equipment_state_.latitude);
        attroff(COLOR_PAIR(3));

        // 车速/油量/电量
        mvaddwstr(24, 0, to_wide_string("车速/油量/电量:").c_str());
        attron(COLOR_PAIR(3));
        mvprintw(24, 15, "%.1f km/h  %.1f%%  %.1f%%",
                 equipment_state_.speed,
                 equipment_state_.fuel_level * 100,
                 equipment_state_.battery_level * 100);
        attroff(COLOR_PAIR(3));

        // 车辆姿态
        mvaddwstr(25, 0, to_wide_string("车辆姿态:").c_str());
        attron(COLOR_PAIR(3));
        mvprintw(25, 15, "Roll: %.2f°  Pitch: %.2f°  Yaw: %.2f°",
                 equipment_state_.roll * (180.0 / M_PI),
                 equipment_state_.pitch * (180.0 / M_PI),
                 equipment_state_.yaw * (180.0 / M_PI));
        attroff(COLOR_PAIR(3));

        // 光电转台
        mvaddwstr(26, 0, to_wide_string("光电转台:").c_str());
        attron(COLOR_PAIR(3));
        mvprintw(26, 15, "Pitch: %.2f°  Yaw: %.2f°  %s",
                 equipment_state_.gimbal_pitch * (180.0 / M_PI),
                 equipment_state_.gimbal_yaw * (180.0 / M_PI),
                 equipment_state_.gimbal_is_active ? "激活" : "未激活");
        attroff(COLOR_PAIR(3));

        // 弹药状态
        mvaddwstr(27, 0, to_wide_string("弹药状态:").c_str());
        attron(COLOR_PAIR(3));
        std::string ammo_str;
        if (equipment_state_.ammo_types.empty())
        {
            ammo_str = "无";
        }
        else
        {
            for (size_t i = 0; i < equipment_state_.ammo_types.size() && i < equipment_state_.ammo_counts.size(); ++i)
            {
                if (i != 0)
                    ammo_str += ", ";
                ammo_str += equipment_state_.ammo_types[i] + ":" + std::to_string(equipment_state_.ammo_counts[i]);
            }
        }
        // 限制弹药状态显示长度
        if (ammo_str.length() > 50) {
            ammo_str = ammo_str.substr(0, 47) + "...";
        }
        mvaddwstr(27, 15, to_wide_string(ammo_str).c_str());
        attroff(COLOR_PAIR(3));

        // 告警信息
        mvaddwstr(28, 0, to_wide_string("告警信息:").c_str());
        if (equipment_state_.warnings.empty())
        {
            attron(COLOR_PAIR(2));
            mvaddwstr(28, 15, to_wide_string("无").c_str());
            attroff(COLOR_PAIR(2));
        }
        else
        {
            attron(COLOR_PAIR(4) | A_BOLD);
            // 只显示第一个告警
            std::string warning = equipment_state_.warnings[0];
            if (warning.length() > 50) {
                warning = warning.substr(0, 47) + "...";
            }
            mvaddwstr(28, 15, to_wide_string(warning).c_str());
            attroff(COLOR_PAIR(4) | A_BOLD);
        }

        // 底部状态栏
        mvhline(LINES-2, 0, ACS_HLINE, COLS);
        attron(COLOR_PAIR(2));
        mvprintw(LINES-1, 0, "按Ctrl+C退出 | 控制器显示模块 v1.0");
        attroff(COLOR_PAIR(2));

        refresh();
    }

    // 按钮状态结构体
    struct
    {
        bool head_tracking_enabled;
        bool voice_command_enabled;
        bool front_screen_enabled;
        uint8_t front_screen_mode;
        uint8_t hmd_video_source;
        uint8_t hmd_overlay_mode;
        bool hmd_enabled;
    } button_states_;

    rclcpp::Subscription<chest_interfaces::msg::VoiceText>::SharedPtr voice_text_sub_;
    rclcpp::Subscription<chest_interfaces::msg::VoiceCommand>::SharedPtr voice_command_sub_;
    rclcpp::Subscription<chest_interfaces::msg::ButtonState>::SharedPtr button_sub_;
    rclcpp::Subscription<chest_interfaces::msg::HeadTracking>::SharedPtr head_motion_sub_;
    rclcpp::Subscription<chest_interfaces::msg::EquipmentState>::SharedPtr equipment_state_sub_;
    rclcpp::Subscription<chest_interfaces::msg::VideoSource>::SharedPtr video_source_sub_;

    rclcpp::TimerBase::SharedPtr ui_timer_;
    rclcpp::TimerBase::SharedPtr freq_calc_timer_;

    chest_interfaces::msg::HeadTracking last_head_tracking_;
    chest_interfaces::msg::EquipmentState equipment_state_;
    chest_interfaces::msg::VideoSource current_video_source_; // 直接存储视频源
    std::vector<std::string> status_lines_;
    std::string last_speech_;
    std::string last_command_;
    uint32_t command_id_ = 0;

    // 频率跟踪变量
    int head_tracking_msg_count_;
    double head_tracking_freq_;
};

int main(int argc, char *argv[])
{
    std::setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}