// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chest_interfaces:msg/VoiceCommand.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__BUILDER_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__BUILDER_HPP_

#include "chest_interfaces/msg/detail/voice_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace chest_interfaces
{

namespace msg
{

namespace builder
{

class Init_VoiceCommand_is_valid
{
public:
  explicit Init_VoiceCommand_is_valid(::chest_interfaces::msg::VoiceCommand & msg)
  : msg_(msg)
  {}
  ::chest_interfaces::msg::VoiceCommand is_valid(::chest_interfaces::msg::VoiceCommand::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceCommand msg_;
};

class Init_VoiceCommand_param3
{
public:
  explicit Init_VoiceCommand_param3(::chest_interfaces::msg::VoiceCommand & msg)
  : msg_(msg)
  {}
  Init_VoiceCommand_is_valid param3(::chest_interfaces::msg::VoiceCommand::_param3_type arg)
  {
    msg_.param3 = std::move(arg);
    return Init_VoiceCommand_is_valid(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceCommand msg_;
};

class Init_VoiceCommand_param2
{
public:
  explicit Init_VoiceCommand_param2(::chest_interfaces::msg::VoiceCommand & msg)
  : msg_(msg)
  {}
  Init_VoiceCommand_param3 param2(::chest_interfaces::msg::VoiceCommand::_param2_type arg)
  {
    msg_.param2 = std::move(arg);
    return Init_VoiceCommand_param3(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceCommand msg_;
};

class Init_VoiceCommand_param1
{
public:
  explicit Init_VoiceCommand_param1(::chest_interfaces::msg::VoiceCommand & msg)
  : msg_(msg)
  {}
  Init_VoiceCommand_param2 param1(::chest_interfaces::msg::VoiceCommand::_param1_type arg)
  {
    msg_.param1 = std::move(arg);
    return Init_VoiceCommand_param2(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceCommand msg_;
};

class Init_VoiceCommand_command_id
{
public:
  explicit Init_VoiceCommand_command_id(::chest_interfaces::msg::VoiceCommand & msg)
  : msg_(msg)
  {}
  Init_VoiceCommand_param1 command_id(::chest_interfaces::msg::VoiceCommand::_command_id_type arg)
  {
    msg_.command_id = std::move(arg);
    return Init_VoiceCommand_param1(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceCommand msg_;
};

class Init_VoiceCommand_frame_header
{
public:
  explicit Init_VoiceCommand_frame_header(::chest_interfaces::msg::VoiceCommand & msg)
  : msg_(msg)
  {}
  Init_VoiceCommand_command_id frame_header(::chest_interfaces::msg::VoiceCommand::_frame_header_type arg)
  {
    msg_.frame_header = std::move(arg);
    return Init_VoiceCommand_command_id(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceCommand msg_;
};

class Init_VoiceCommand_header
{
public:
  Init_VoiceCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VoiceCommand_frame_header header(::chest_interfaces::msg::VoiceCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VoiceCommand_frame_header(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chest_interfaces::msg::VoiceCommand>()
{
  return chest_interfaces::msg::builder::Init_VoiceCommand_header();
}

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__BUILDER_HPP_
