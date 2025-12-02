// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chest_interfaces:msg/VideoSource.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__BUILDER_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__BUILDER_HPP_

#include "chest_interfaces/msg/detail/video_source__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace chest_interfaces
{

namespace msg
{

namespace builder
{

class Init_VideoSource_is_available
{
public:
  explicit Init_VideoSource_is_available(::chest_interfaces::msg::VideoSource & msg)
  : msg_(msg)
  {}
  ::chest_interfaces::msg::VideoSource is_available(::chest_interfaces::msg::VideoSource::_is_available_type arg)
  {
    msg_.is_available = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

class Init_VideoSource_encoding
{
public:
  explicit Init_VideoSource_encoding(::chest_interfaces::msg::VideoSource & msg)
  : msg_(msg)
  {}
  Init_VideoSource_is_available encoding(::chest_interfaces::msg::VideoSource::_encoding_type arg)
  {
    msg_.encoding = std::move(arg);
    return Init_VideoSource_is_available(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

class Init_VideoSource_height
{
public:
  explicit Init_VideoSource_height(::chest_interfaces::msg::VideoSource & msg)
  : msg_(msg)
  {}
  Init_VideoSource_encoding height(::chest_interfaces::msg::VideoSource::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_VideoSource_encoding(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

class Init_VideoSource_width
{
public:
  explicit Init_VideoSource_width(::chest_interfaces::msg::VideoSource & msg)
  : msg_(msg)
  {}
  Init_VideoSource_height width(::chest_interfaces::msg::VideoSource::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_VideoSource_height(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

class Init_VideoSource_fps
{
public:
  explicit Init_VideoSource_fps(::chest_interfaces::msg::VideoSource & msg)
  : msg_(msg)
  {}
  Init_VideoSource_width fps(::chest_interfaces::msg::VideoSource::_fps_type arg)
  {
    msg_.fps = std::move(arg);
    return Init_VideoSource_width(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

class Init_VideoSource_source_name
{
public:
  explicit Init_VideoSource_source_name(::chest_interfaces::msg::VideoSource & msg)
  : msg_(msg)
  {}
  Init_VideoSource_fps source_name(::chest_interfaces::msg::VideoSource::_source_name_type arg)
  {
    msg_.source_name = std::move(arg);
    return Init_VideoSource_fps(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

class Init_VideoSource_ip_address
{
public:
  explicit Init_VideoSource_ip_address(::chest_interfaces::msg::VideoSource & msg)
  : msg_(msg)
  {}
  Init_VideoSource_source_name ip_address(::chest_interfaces::msg::VideoSource::_ip_address_type arg)
  {
    msg_.ip_address = std::move(arg);
    return Init_VideoSource_source_name(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

class Init_VideoSource_id
{
public:
  explicit Init_VideoSource_id(::chest_interfaces::msg::VideoSource & msg)
  : msg_(msg)
  {}
  Init_VideoSource_ip_address id(::chest_interfaces::msg::VideoSource::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_VideoSource_ip_address(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

class Init_VideoSource_header
{
public:
  Init_VideoSource_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VideoSource_id header(::chest_interfaces::msg::VideoSource::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VideoSource_id(msg_);
  }

private:
  ::chest_interfaces::msg::VideoSource msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chest_interfaces::msg::VideoSource>()
{
  return chest_interfaces::msg::builder::Init_VideoSource_header();
}

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__BUILDER_HPP_
