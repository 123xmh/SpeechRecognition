#include "video_source_loader/video_source_loader.hpp"
#include "video_source_loader/video_config_parser.hpp"
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace video_source_loader
{

    VideoSourceLoader::VideoSourceLoader(const rclcpp::NodeOptions &options)
        : Node("video_source_loader", options),
          last_selected_source_id_(0)
    {
        // Declare parameters
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<bool>("debug_output", true);

        // Get parameter values
        config_file_path_ = this->get_parameter("config_file").as_string();
        bool debug_output = this->get_parameter("debug_output").as_bool();

        if (config_file_path_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No config file path specified");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Config file path: %s", config_file_path_.c_str());

        // Load video sources from YAML
        load_video_sources();

        // Create publisher
        video_source_pub_ = this->create_publisher<chest_interfaces::msg::VideoSource>(
            "/tk_chest/video/source/selected", 10);
        RCLCPP_INFO(this->get_logger(), "Created publisher on topic: selected_video_source");

        // Create subscriber
        button_state_sub_ = this->create_subscription<chest_interfaces::msg::ButtonState>(
            "/tk_chest/button/state", 10,
            std::bind(&VideoSourceLoader::button_state_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Created subscriber on topic: button_state");

        RCLCPP_INFO(this->get_logger(), "Video Source Loader started");
        if (debug_output)
        {
            RCLCPP_INFO(this->get_logger(), "Debug output enabled");
        }
    }

    VideoSourceLoader::~VideoSourceLoader()
    {
    }

    void VideoSourceLoader::load_video_sources()
    {
        // 直接调用解析函数
        if (!parse_video_sources_config(config_file_path_, video_sources_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load video sources from config");
        }
        else
        {
            bool debug_output = this->get_parameter("debug_output").as_bool();

            RCLCPP_INFO(this->get_logger(), "Loaded %zu video sources", video_sources_.size());
            if (debug_output)
            {
                for (const auto &source : video_sources_)
                {
                    RCLCPP_INFO(this->get_logger(), "Source %d: %s (%s), Available: %s",
                                source.id, source.source_name.c_str(), source.ip_address.c_str(),
                                source.is_available ? "Yes" : "No");
                }
            }
        }
    }

    void VideoSourceLoader::button_state_callback(const chest_interfaces::msg::ButtonState::SharedPtr msg)
    {
        bool debug_output = this->get_parameter("debug_output").as_bool();
        uint8_t selected_source_id = msg->hmd_video_source;

        if (debug_output)
        {
            RCLCPP_INFO(this->get_logger(), "Received ButtonState message, hmd_video_source: %d", selected_source_id);
        }

        // Check if the selected source has changed
        if (selected_source_id != last_selected_source_id_)
        {
            if (debug_output)
            {
                RCLCPP_INFO(this->get_logger(), "Video source changed from %d to %d",
                            last_selected_source_id_, selected_source_id);
            }

            last_selected_source_id_ = selected_source_id;
            publish_selected_video_source(selected_source_id);
        }
    }

    void VideoSourceLoader::publish_selected_video_source(uint8_t source_id)
    {
        bool debug_output = this->get_parameter("debug_output").as_bool();

        // Find the video source with the given ID
        for (const auto &source : video_sources_)
        {
            if (source.id == source_id)
            {
                if (source.is_available)
                {
                    // Create a new message with updated header
                    auto selected_source = source;
                    selected_source.header.stamp = this->now();
                    selected_source.header.frame_id = "selected_video_source";

                    // Publish the selected video source
                    video_source_pub_->publish(selected_source);

                    if (debug_output)
                    {
                        RCLCPP_INFO(this->get_logger(), "Published video source: ID=%d, Name=%s, IP=%s",
                                    source.id, source.source_name.c_str(), source.ip_address.c_str());
                    }
                }
                else
                {
                    if (debug_output)
                    {
                        RCLCPP_WARN(this->get_logger(), "Requested video source ID %d is not available", source_id);
                    }
                }
                return;
            }
        }

        // If no matching source found
        if (debug_output)
        {
            RCLCPP_WARN(this->get_logger(), "Requested video source ID %d not found", source_id);
        }
    }

} // namespace video_source_loader

// 添加main函数
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<video_source_loader::VideoSourceLoader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}