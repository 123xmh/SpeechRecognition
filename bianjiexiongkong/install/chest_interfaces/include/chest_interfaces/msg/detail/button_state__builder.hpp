// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chest_interfaces:msg/ButtonState.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_

#include "chest_interfaces/msg/detail/button_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace chest_interfaces
{

namespace msg
{

namespace builder
{

class Init_ButtonState_hmd_enabled
{
public:
  explicit Init_ButtonState_hmd_enabled(::chest_interfaces::msg::ButtonState & msg)
  : msg_(msg)
  {}
  ::chest_interfaces::msg::ButtonState hmd_enabled(::chest_interfaces::msg::ButtonState::_hmd_enabled_type arg)
  {
    msg_.hmd_enabled = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chest_interfaces::msg::ButtonState msg_;
};

class Init_ButtonState_hmd_overlay_mode
{
public:
  explicit Init_ButtonState_hmd_overlay_mode(::chest_interfaces::msg::ButtonState & msg)
  : msg_(msg)
  {}
  Init_ButtonState_hmd_enabled hmd_overlay_mode(::chest_interfaces::msg::ButtonState::_hmd_overlay_mode_type arg)
  {
    msg_.hmd_overlay_mode = std::move(arg);
    return Init_ButtonState_hmd_enabled(msg_);
  }

private:
  ::chest_interfaces::msg::ButtonState msg_;
};

class Init_ButtonState_hmd_video_source
{
public:
  explicit Init_ButtonState_hmd_video_source(::chest_interfaces::msg::ButtonState & msg)
  : msg_(msg)
  {}
  Init_ButtonState_hmd_overlay_mode hmd_video_source(::chest_interfaces::msg::ButtonState::_hmd_video_source_type arg)
  {
    msg_.hmd_video_source = std::move(arg);
    return Init_ButtonState_hmd_overlay_mode(msg_);
  }

private:
  ::chest_interfaces::msg::ButtonState msg_;
};

class Init_ButtonState_front_screen_mode
{
public:
  explicit Init_ButtonState_front_screen_mode(::chest_interfaces::msg::ButtonState & msg)
  : msg_(msg)
  {}
  Init_ButtonState_hmd_video_source front_screen_mode(::chest_interfaces::msg::ButtonState::_front_screen_mode_type arg)
  {
    msg_.front_screen_mode = std::move(arg);
    return Init_ButtonState_hmd_video_source(msg_);
  }

private:
  ::chest_interfaces::msg::ButtonState msg_;
};

class Init_ButtonState_front_screen_enabled
{
public:
  explicit Init_ButtonState_front_screen_enabled(::chest_interfaces::msg::ButtonState & msg)
  : msg_(msg)
  {}
  Init_ButtonState_front_screen_mode front_screen_enabled(::chest_interfaces::msg::ButtonState::_front_screen_enabled_type arg)
  {
    msg_.front_screen_enabled = std::move(arg);
    return Init_ButtonState_front_screen_mode(msg_);
  }

private:
  ::chest_interfaces::msg::ButtonState msg_;
};

class Init_ButtonState_voice_command_enabled
{
public:
  explicit Init_ButtonState_voice_command_enabled(::chest_interfaces::msg::ButtonState & msg)
  : msg_(msg)
  {}
  Init_ButtonState_front_screen_enabled voice_command_enabled(::chest_interfaces::msg::ButtonState::_voice_command_enabled_type arg)
  {
    msg_.voice_command_enabled = std::move(arg);
    return Init_ButtonState_front_screen_enabled(msg_);
  }

private:
  ::chest_interfaces::msg::ButtonState msg_;
};

class Init_ButtonState_head_tracking_enabled
{
public:
  explicit Init_ButtonState_head_tracking_enabled(::chest_interfaces::msg::ButtonState & msg)
  : msg_(msg)
  {}
  Init_ButtonState_voice_command_enabled head_tracking_enabled(::chest_interfaces::msg::ButtonState::_head_tracking_enabled_type arg)
  {
    msg_.head_tracking_enabled = std::move(arg);
    return Init_ButtonState_voice_command_enabled(msg_);
  }

private:
  ::chest_interfaces::msg::ButtonState msg_;
};

class Init_ButtonState_header
{
public:
  Init_ButtonState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ButtonState_head_tracking_enabled header(::chest_interfaces::msg::ButtonState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ButtonState_head_tracking_enabled(msg_);
  }

private:
  ::chest_interfaces::msg::ButtonState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chest_interfaces::msg::ButtonState>()
{
  return chest_interfaces::msg::builder::Init_ButtonState_header();
}

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_
