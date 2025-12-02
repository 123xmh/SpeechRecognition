// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chest_interfaces:msg/VoiceText.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__BUILDER_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__BUILDER_HPP_

#include "chest_interfaces/msg/detail/voice_text__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace chest_interfaces
{

namespace msg
{

namespace builder
{

class Init_VoiceText_confidence
{
public:
  explicit Init_VoiceText_confidence(::chest_interfaces::msg::VoiceText & msg)
  : msg_(msg)
  {}
  ::chest_interfaces::msg::VoiceText confidence(::chest_interfaces::msg::VoiceText::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceText msg_;
};

class Init_VoiceText_is_final
{
public:
  explicit Init_VoiceText_is_final(::chest_interfaces::msg::VoiceText & msg)
  : msg_(msg)
  {}
  Init_VoiceText_confidence is_final(::chest_interfaces::msg::VoiceText::_is_final_type arg)
  {
    msg_.is_final = std::move(arg);
    return Init_VoiceText_confidence(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceText msg_;
};

class Init_VoiceText_raw_text
{
public:
  explicit Init_VoiceText_raw_text(::chest_interfaces::msg::VoiceText & msg)
  : msg_(msg)
  {}
  Init_VoiceText_is_final raw_text(::chest_interfaces::msg::VoiceText::_raw_text_type arg)
  {
    msg_.raw_text = std::move(arg);
    return Init_VoiceText_is_final(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceText msg_;
};

class Init_VoiceText_header
{
public:
  Init_VoiceText_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VoiceText_raw_text header(::chest_interfaces::msg::VoiceText::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VoiceText_raw_text(msg_);
  }

private:
  ::chest_interfaces::msg::VoiceText msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chest_interfaces::msg::VoiceText>()
{
  return chest_interfaces::msg::builder::Init_VoiceText_header();
}

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__BUILDER_HPP_
