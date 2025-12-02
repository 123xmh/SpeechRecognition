#include "video_source_loader/video_config_parser.hpp"
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace video_source_loader
{

    bool parse_video_sources_config(const std::string &config_file_path,
                                    std::vector<chest_interfaces::msg::VideoSource> &video_sources)
    {
        try
        {
            // Load YAML file
            YAML::Node config = YAML::LoadFile(config_file_path);

            // Get video_sources array
            YAML::Node video_sources_node = config["video_sources"];

            if (!video_sources_node || !video_sources_node.IsSequence())
            {
                RCLCPP_ERROR(rclcpp::get_logger("video_config_parser"),
                             "Invalid YAML structure: 'video_sources' not found or not a sequence");
                return false;
            }

            // Clear existing video sources
            video_sources.clear();

            // Parse each video source
            for (const auto &source_node : video_sources_node)
            {
                chest_interfaces::msg::VideoSource source;

                source.id = source_node["id"].as<uint8_t>();
                source.ip_address = source_node["ip_address"].as<std::string>();
                source.source_name = source_node["source_name"].as<std::string>();
                source.fps = source_node["fps"].as<uint32_t>();
                source.width = source_node["width"].as<uint32_t>();
                source.height = source_node["height"].as<uint32_t>();
                source.encoding = source_node["encoding"].as<std::string>();
                source.is_available = source_node["is_available"].as<bool>();

                video_sources.push_back(source);
            }

            RCLCPP_INFO(rclcpp::get_logger("video_config_parser"),
                        "Loaded %zu video sources from config", video_sources.size());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("video_config_parser"),
                         "YAML parsing error: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("video_config_parser"),
                         "Error loading video sources: %s", e.what());
            return false;
        }
    }

} // namespace video_source_loader