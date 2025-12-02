#ifndef VIDEO_SOURCE_LOADER__VIDEO_SOURCE_LOADER_HPP_
#define VIDEO_SOURCE_LOADER__VIDEO_SOURCE_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chest_interfaces/msg/video_source.hpp>
#include <chest_interfaces/msg/button_state.hpp>

#include <string>
#include <vector>
#include <memory>

namespace video_source_loader
{

    class VideoSourceLoader : public rclcpp::Node
    {
    public:
        explicit VideoSourceLoader(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        virtual ~VideoSourceLoader();

    private:
        void load_video_sources();
        void button_state_callback(const chest_interfaces::msg::ButtonState::SharedPtr msg);
        void publish_selected_video_source(uint8_t source_id);

        rclcpp::Subscription<chest_interfaces::msg::ButtonState>::SharedPtr button_state_sub_;
        rclcpp::Publisher<chest_interfaces::msg::VideoSource>::SharedPtr video_source_pub_;

        std::vector<chest_interfaces::msg::VideoSource> video_sources_;
        std::string config_file_path_;
        uint8_t last_selected_source_id_;
    };

} // namespace video_source_loader

#endif // VIDEO_SOURCE_LOADER__VIDEO_SOURCE_LOADER_HPP_