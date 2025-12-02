#include "rclcpp/rclcpp.hpp"
#include "chest_interfaces/msg/button_state.hpp"
#include <ncursesw/ncurses.h>
#include <locale>
#include <array>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class ButtonSimulatorNode : public rclcpp::Node
{
public:
    ButtonSimulatorNode() : Node("button_simulator")
    {
        // 确保正确的locale设置
        std::setlocale(LC_ALL, "zh_CN.UTF-8");

        // 初始化按键状态
        button_state_.head_tracking_enabled = false;
        button_state_.voice_command_enabled = false;
        button_state_.front_screen_enabled = false;
        button_state_.front_screen_mode = 0;
        button_state_.hmd_video_source = 1; // 修改为从1开始
        button_state_.hmd_overlay_mode = 0;
        button_state_.hmd_enabled = false;

        // 创建发布者
        pub_ = create_publisher<chest_interfaces::msg::ButtonState>("/tk_chest/button/state", 10);

        // 初始化ncurses宽字符模式
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
        curs_set(0);      // 隐藏光标
        set_escdelay(25); // 解决ESC键延迟问题

        // 检查终端大小
        getmaxyx(stdscr, term_height_, term_width_);
        if (term_height_ < MIN_HEIGHT || term_width_ < MIN_WIDTH)
        {
            endwin();
            RCLCPP_ERROR(get_logger(), "终端大小至少需要 %dx%d (当前: %dx%d)",
                         MIN_WIDTH, MIN_HEIGHT, term_width_, term_height_);
            rclcpp::shutdown();
            return;
        }

        // 启用颜色支持
        start_color();
        use_default_colors();
        init_pair(TITLE_PAIR, COLOR_WHITE, COLOR_BLUE);
        init_pair(HEADER_PAIR, COLOR_CYAN, COLOR_BLACK);
        init_pair(STATUS_PAIR, COLOR_GREEN, COLOR_BLACK);
        init_pair(MESSAGE_PAIR, COLOR_YELLOW, COLOR_BLACK);
        init_pair(DIVIDER_PAIR, COLOR_CYAN, COLOR_BLACK);

        // 初始化界面
        init_ui();

        // 创建输入处理定时器
        timer_ = create_wall_timer(100ms, [this]()
                                   {
            process_input();
            publish_state(); });
    }

    ~ButtonSimulatorNode()
    {
        RCLCPP_INFO(get_logger(), "关闭按钮模拟器");
        if (timer_)
        {
            timer_->cancel();
        }
        if (stdscr)
        {
            endwin();
        }
    }

private:
    void init_ui()
    {
        clear();

        if (!stdscr)
        {
            RCLCPP_ERROR(get_logger(), "无法初始化ncurses");
            rclcpp::shutdown();
            return;
        }

        // 绘制标题栏
        // attron(COLOR_PAIR(TITLE_PAIR));
        for (int i = 0; i < term_width_; i++)
        {
            mvaddch(0, i, ' ');
        }
        std::wstring title = L"✨ 按钮模拟器 ✨";
        int title_pos = (80 - title.length()) / 2;
        mvaddwstr(0, title_pos, title.c_str());
        // attroff(COLOR_PAIR(TITLE_PAIR));

        // 绘制分隔线
        attron(COLOR_PAIR(DIVIDER_PAIR));
        mvaddwstr(1, 0, L"──────────────────────────────────────────────────────────────────────────────────────────");
        attroff(COLOR_PAIR(DIVIDER_PAIR));

        // 按键映射 - 使用表格格式对齐，考虑宽体字符
        attron(COLOR_PAIR(HEADER_PAIR));
        mvaddwstr(2, 0, L"按键:  q(头动)    w(语音)    1(正面屏)  2(模式)    3(视频源)  4(叠加)    5(头显)    x(退出)");
        attroff(COLOR_PAIR(HEADER_PAIR));

        // 状态显示 - 使用表格格式对齐
        attron(COLOR_PAIR(HEADER_PAIR));
        mvaddwstr(3, 0, L"状态: ");
        attroff(COLOR_PAIR(HEADER_PAIR));
        
        // 显示初始状态
        update_status_display();

        // 绘制分隔线
        attron(COLOR_PAIR(DIVIDER_PAIR));
        mvaddwstr(4, 0, L"──────────────────────────────────────────────────────────────────────────────────────────");
        attroff(COLOR_PAIR(DIVIDER_PAIR));

        // 消息区域
        attron(COLOR_PAIR(HEADER_PAIR));
        mvaddwstr(5, 0, L"消息: ");
        attroff(COLOR_PAIR(HEADER_PAIR));
        
        // 初始消息
        last_message_ = L"按对应键切换状态，x键退出";
        update_message_display();
        
        refresh();
    }

    void update_status_display()
    {
        // 清除状态行
        move(3, 7);
        clrtoeol();
        
        // 定义列位置，与按键行完美对齐（考虑宽体字符）
        // 按键行: "按键: q(头动)  w(语音)  1(正面屏) 2(模式)  3(视频源) 4(叠加)  5(头显)  x(退出)"
        // 状态行: "状态: 头动:关  语音:关   正面屏:关 模式:默认 视频源:1  叠加:无   头显:关"
        
        // 头动状态 - 对齐到 q(头动) 位置
        move(3, 7);
        addwstr(L"头动:");
        attron(COLOR_PAIR(STATUS_PAIR));
        addwstr(button_state_.head_tracking_enabled ? L"开 " : L"关 ");
        attroff(COLOR_PAIR(STATUS_PAIR));
        
        // 语音状态 - 对齐到 w(语音) 位置
        move(3, 18);
        addwstr(L"语音:");
        attron(COLOR_PAIR(STATUS_PAIR));
        addwstr(button_state_.voice_command_enabled ? L"开 " : L"关 ");
        attroff(COLOR_PAIR(STATUS_PAIR));
        
        // 正面屏状态 - 对齐到 1(正面屏) 位置
        move(3, 29);
        addwstr(L"正面屏:");
        attron(COLOR_PAIR(STATUS_PAIR));
        addwstr(button_state_.front_screen_enabled ? L"开 " : L"关 ");
        attroff(COLOR_PAIR(STATUS_PAIR));
        
        // 模式状态 - 对齐到 2(模式) 位置
        move(3, 40);
        addwstr(L"模式:");
        std::wstring mode_str;
        switch (button_state_.front_screen_mode)
        {
        case 0:
            mode_str = L"默认";
            break;
        case 1:
            mode_str = L"A   ";
            break;
        case 2:
            mode_str = L"B   ";
            break;
        case 3:
            mode_str = L"C   ";
            break;
        default:
            mode_str = L"未知";
        }
        attron(COLOR_PAIR(STATUS_PAIR));
        addwstr(mode_str.c_str());
        attroff(COLOR_PAIR(STATUS_PAIR));
        
        // 视频源状态 - 对齐到 3(视频源) 位置
        move(3, 51);
        addwstr(L"视频源:");
        attron(COLOR_PAIR(STATUS_PAIR));
        addwstr(std::to_wstring(button_state_.hmd_video_source).c_str());
        addwstr(L"  ");
        attroff(COLOR_PAIR(STATUS_PAIR));
        
        // 叠加状态 - 对齐到 4(叠加) 位置
        move(3, 62);
        addwstr(L"叠加:");
        std::wstring overlay_str;
        switch (button_state_.hmd_overlay_mode)
        {
        case 0:
            overlay_str = L"无   ";
            break;
        case 1:
            overlay_str = L"最小 ";
            break;
        case 2:
            overlay_str = L"完全 ";
            break;
        default:
            overlay_str = L"未知";
        }
        attron(COLOR_PAIR(STATUS_PAIR));
        addwstr(overlay_str.c_str());
        attroff(COLOR_PAIR(STATUS_PAIR));
        
        // 头显状态 - 对齐到 5(头显) 位置
        move(3, 73);
        addwstr(L"头显:");
        attron(COLOR_PAIR(STATUS_PAIR));
        addwstr(button_state_.hmd_enabled ? L"开 " : L"关 ");
        attroff(COLOR_PAIR(STATUS_PAIR));
    }

    void update_message_display()
    {
        // 更新消息行
        move(5, 7);
        clrtoeol();

        if (!last_message_.empty())
        {
            attron(COLOR_PAIR(MESSAGE_PAIR));
            addwstr(last_message_.c_str());
            attroff(COLOR_PAIR(MESSAGE_PAIR));
        }
    }

    void process_input()
    {
        int ch = getch();
        if (ch == ERR)
            return;

        last_message_ = L"";

        switch (ch)
        {
        case 'q':
            button_state_.head_tracking_enabled = !button_state_.head_tracking_enabled;
            last_message_ = L"侧按键1 (头动开关): " +
                            std::wstring(button_state_.head_tracking_enabled ? L"开" : L"关");
            break;
        case 'w':
            button_state_.voice_command_enabled = !button_state_.voice_command_enabled;
            last_message_ = L"侧按键2 (语音开关): " +
                            std::wstring(button_state_.voice_command_enabled ? L"开" : L"关");
            break;
        case '1':
            button_state_.front_screen_enabled = !button_state_.front_screen_enabled;
            last_message_ = L"正按键1 (正面屏开关): " +
                            std::wstring(button_state_.front_screen_enabled ? L"开" : L"关");
            break;
        case '2':
            button_state_.front_screen_mode = (button_state_.front_screen_mode + 1) % 4;
            last_message_ = L"正按键2: 显示模式切换到 " +
                            std::to_wstring(button_state_.front_screen_mode);
            break;
        case '3':
            // 修改视频源切换逻辑，从1-4循环
            button_state_.hmd_video_source = (button_state_.hmd_video_source - 1 + 1) % 4 + 1;
            last_message_ = L"正按键3: 视频源切换到 " +
                            std::to_wstring(button_state_.hmd_video_source);
            break;
        case '4':
            button_state_.hmd_overlay_mode = (button_state_.hmd_overlay_mode + 1) % 3;
            last_message_ = L"正按键4: 叠加模式切换到 " +
                            std::to_wstring(button_state_.hmd_overlay_mode);
            break;
        case '5':
            button_state_.hmd_enabled = !button_state_.hmd_enabled;
            last_message_ = L"正按键5 (头显开关): " +
                            std::wstring(button_state_.hmd_enabled ? L"开" : L"关");
            break;
        case 'x':
            last_message_ = L"关闭节点...";
            rclcpp::shutdown();
            break;
        default:
            last_message_ = L"未映射按键: " + std::wstring(1, ch) + L" (有效按键 q,w,1-5, x退出)";
        }

        update_status_display();
        update_message_display();
    }

    void publish_state()
    {
        if (!pub_)
        {
            return;
        }
        auto msg = chest_interfaces::msg::ButtonState();
        msg.header.stamp = now();
        msg.header.frame_id = "button_panel";

        msg.head_tracking_enabled = button_state_.head_tracking_enabled;
        msg.voice_command_enabled = button_state_.voice_command_enabled;
        msg.front_screen_enabled = button_state_.front_screen_enabled;
        msg.front_screen_mode = button_state_.front_screen_mode;
        msg.hmd_video_source = button_state_.hmd_video_source;
        msg.hmd_overlay_mode = button_state_.hmd_overlay_mode;
        msg.hmd_enabled = button_state_.hmd_enabled;

        pub_->publish(msg);
    }

    // ROS 发布者
    rclcpp::Publisher<chest_interfaces::msg::ButtonState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 终端尺寸
    int term_height_;
    int term_width_;

    // 按钮状态
    struct
    {
        bool head_tracking_enabled;
        bool voice_command_enabled;
        bool front_screen_enabled;
        int front_screen_mode;
        int hmd_video_source;
        int hmd_overlay_mode;
        bool hmd_enabled;
    } button_state_;

    // UI 状态
    std::vector<std::wstring> status_lines_;
    std::wstring last_message_;

    // 常量定义
    static constexpr int MIN_HEIGHT = 6;   // 增加最小高度以适应新布局
    static constexpr int MIN_WIDTH = 80;   // 增加最小宽度以适应新布局
    static constexpr int TITLE_PAIR = 1;
    static constexpr int HEADER_PAIR = 2;  // 新增标题颜色对
    static constexpr int STATUS_PAIR = 3;
    static constexpr int MESSAGE_PAIR = 4;
    static constexpr int DIVIDER_PAIR = 5;
};

int main(int argc, char **argv)
{
    // 确保正确的locale设置
    std::setlocale(LC_ALL, "zh_CN.UTF-8");
    std::locale::global(std::locale("zh_CN.UTF-8"));

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ButtonSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}