// video_processor.hpp
#ifndef AR_DISPLAY_VIDEO_PROCESSOR_HPP
#define AR_DISPLAY_VIDEO_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include "chest_interfaces/msg/equipment_state.hpp"
#include "chest_interfaces/msg/video_source.hpp"

class VideoProcessor {
public:
    VideoProcessor();
    ~VideoProcessor();

    void start();
    void stop();
    void set_current_source(uint8_t source_idx);
    void set_overlay_mode(uint8_t mode);
    void update_head_motion(float pitch, float yaw);
    void update_equipment_state(const chest_interfaces::msg::EquipmentState& state);
    void update_video_source(const chest_interfaces::msg::VideoSource& source);

    struct DisplayStatus {
        std::string current_source;
        uint8_t overlay_mode;
        float head_pitch;
        float head_yaw;
        chest_interfaces::msg::EquipmentState equipment_state;
        chest_interfaces::msg::VideoSource video_source;
    };

    DisplayStatus get_current_status() const;

private:
    void process_loop();

    std::vector<std::string> video_sources_;
    uint8_t current_source_ = 0;
    uint8_t overlay_mode_ = 1;  // 默认MINIMAL模式
    float head_pitch_ = 0.0f;
    float head_yaw_ = 0.0f;
    chest_interfaces::msg::EquipmentState equipment_state_;
    chest_interfaces::msg::VideoSource video_source_;

    bool processing_ = false;
    std::thread thread_;
    mutable std::mutex mutex_;
    
    cv::Ptr<cv::freetype::FreeType2> ft2_;
};

#endif // AR_DISPLAY_VIDEO_PROCESSOR_HPP