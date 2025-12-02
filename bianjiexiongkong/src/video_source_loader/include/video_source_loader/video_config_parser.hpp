#ifndef VIDEO_SOURCE_LOADER__VIDEO_CONFIG_PARSER_HPP_
#define VIDEO_SOURCE_LOADER__VIDEO_CONFIG_PARSER_HPP_

#include <chest_interfaces/msg/video_source.hpp>
#include <string>
#include <vector>

namespace video_source_loader
{

bool parse_video_sources_config(const std::string& config_file_path, 
                               std::vector<chest_interfaces::msg::VideoSource>& video_sources);

}  // namespace video_source_loader

#endif  // VIDEO_SOURCE_LOADER__VIDEO_CONFIG_PARSER_HPP_