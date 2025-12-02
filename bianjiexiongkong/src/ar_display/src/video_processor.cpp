// video_processor.cpp
#include "ar_display/video_processor.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

VideoProcessor::VideoProcessor()
    : current_source_(0),
      overlay_mode_(1),
      head_pitch_(0.0f),
      head_yaw_(0.0f)
{
    // 视频源列表
    video_sources_ = {
        "中央网络摄像头1",
        "中央网络摄像头2",
        "本地摄像头3",
        "备用摄像头4"};

    // 初始化FreeType渲染器
    ft2_ = cv::freetype::createFreeType2();

    // 使用文泉驿微米黑字体
    const std::string font_path = "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc";

    try
    {
        ft2_->loadFontData(font_path, 0);
        std::cout << "成功加载字体: " << font_path << std::endl;
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "字体加载失败: " << e.what() << std::endl;
        std::cerr << "将使用默认字体 (中文可能无法显示)" << std::endl;
    }
}

VideoProcessor::~VideoProcessor()
{
    stop();
}

void VideoProcessor::start()
{
    processing_ = true;
    thread_ = std::thread(&VideoProcessor::process_loop, this);
}

void VideoProcessor::stop()
{
    processing_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void VideoProcessor::set_current_source(uint8_t source_idx)
{
    if (source_idx < video_sources_.size())
    {
        current_source_ = source_idx;
    }
}

void VideoProcessor::set_overlay_mode(uint8_t mode)
{
    overlay_mode_ = mode;
}

void VideoProcessor::update_head_motion(float pitch, float yaw)
{
    std::lock_guard<std::mutex> lock(mutex_);
    head_pitch_ = pitch;
    head_yaw_ = yaw;
}

void VideoProcessor::update_equipment_state(const chest_interfaces::msg::EquipmentState& state)
{
    std::lock_guard<std::mutex> lock(mutex_);
    equipment_state_ = state;
}

void VideoProcessor::update_video_source(const chest_interfaces::msg::VideoSource& source)
{
    std::lock_guard<std::mutex> lock(mutex_);
    video_source_ = source;
}

VideoProcessor::DisplayStatus VideoProcessor::get_current_status() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return {
        video_sources_[current_source_],
        overlay_mode_,
        head_pitch_,
        head_yaw_,
        equipment_state_,
        video_source_};
}

void VideoProcessor::process_loop()
{
    cv::namedWindow("AR Display", cv::WINDOW_NORMAL);
    cv::resizeWindow("AR Display", 800, 600);

    // 姿态指示器平滑处理变量
    float smoothed_pitch = 0.0f;
    float smoothed_yaw = 0.0f;
    const float smoothing_factor = 0.2f;

    // 定义字体参数
    const int main_font_height = 24;
    const int small_font_height = 18;

    while (processing_)
    {
        // 创建模拟视频帧
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

        // 获取当前状态（线程安全）
        DisplayStatus status = get_current_status();

        // 使用FreeType渲染文本
        try
        {
            // 添加视频源标识
            ft2_->putText(frame,
                          "视频源: " + status.video_source.source_name,
                          cv::Point(10, 40),
                          main_font_height,
                          cv::Scalar(0, 255, 0),
                          -1, // 厚度
                          cv::LINE_AA,
                          false); // 不使用左下角原点

            // 添加设备状态
            std::string status_str = "车速: " + std::to_string(static_cast<int>(status.equipment_state.speed)) + " km/h";
            status_str += " 油量: " + std::to_string(static_cast<int>(status.equipment_state.fuel_level * 100)) + "%";
            status_str += " 电量: " + std::to_string(static_cast<int>(status.equipment_state.battery_level * 100)) + "%";
            
            ft2_->putText(frame,
                          status_str,
                          cv::Point(10, 75),
                          small_font_height,
                          cv::Scalar(0, 255, 255),
                          -1,
                          cv::LINE_AA,
                          false);
        }
        catch (const cv::Exception &e)
        {
            std::cerr << "文本渲染错误: " << e.what() << std::endl;
        }

        // 姿态平滑处理
        smoothed_pitch = smoothed_pitch * (1 - smoothing_factor) + status.head_pitch * smoothing_factor;
        smoothed_yaw = smoothed_yaw * (1 - smoothing_factor) + status.head_yaw * smoothing_factor;

        // 根据叠加模式显示不同内容
        if (status.overlay_mode > 0)
        { // MINIMAL或FULL模式
            int center_x = frame.cols / 2;
            int center_y = frame.rows / 2;
            int radius = 50;

            // 绘制指示圈
            cv::circle(frame, cv::Point(center_x, center_y), radius, cv::Scalar(0, 0, 255), 2);

            // 计算指针位置
            int pointer_x = center_x + static_cast<int>(radius * sin(smoothed_yaw) * cos(smoothed_pitch));
            int pointer_y = center_y + static_cast<int>(radius * sin(smoothed_pitch));

            // 绘制指针
            cv::line(frame, cv::Point(center_x, center_y), cv::Point(pointer_x, pointer_y),
                     cv::Scalar(0, 0, 255), 3);

            // FULL模式显示更多信息
            if (status.overlay_mode == 2)
            { // FULL模式
                // 将弧度转换为角度
                double pitch_deg = smoothed_pitch * (180.0 / M_PI);
                double yaw_deg = smoothed_yaw * (180.0 / M_PI);
                
                std::ostringstream angle_text;
                angle_text << "俯仰角: " << std::fixed << std::setprecision(2) << pitch_deg
                           << "°  偏航角: " << yaw_deg << "°";

                try
                {
                    ft2_->putText(frame,
                                  angle_text.str(),
                                  cv::Point(center_x - 150, center_y - 70),
                                  small_font_height,
                                  cv::Scalar(0, 255, 255),
                                  -1,
                                  cv::LINE_AA,
                                  false);
                }
                catch (const cv::Exception &e)
                {
                    std::cerr << "角度文本渲染错误: " << e.what() << std::endl;
                }
            }
        }

        // 显示帧
        cv::imshow("AR Display", frame);

        // 检查ESC键退出
        if (cv::waitKey(30) == 27)
        {
            processing_ = false;
        }
    }

    cv::destroyWindow("AR Display");
}