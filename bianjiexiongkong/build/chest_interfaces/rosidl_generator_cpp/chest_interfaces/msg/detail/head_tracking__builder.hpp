// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chest_interfaces:msg/HeadTracking.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__BUILDER_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__BUILDER_HPP_

#include "chest_interfaces/msg/detail/head_tracking__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace chest_interfaces
{

namespace msg
{

namespace builder
{

class Init_HeadTracking_confidence
{
public:
  explicit Init_HeadTracking_confidence(::chest_interfaces::msg::HeadTracking & msg)
  : msg_(msg)
  {}
  ::chest_interfaces::msg::HeadTracking confidence(::chest_interfaces::msg::HeadTracking::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chest_interfaces::msg::HeadTracking msg_;
};

class Init_HeadTracking_is_tracking
{
public:
  explicit Init_HeadTracking_is_tracking(::chest_interfaces::msg::HeadTracking & msg)
  : msg_(msg)
  {}
  Init_HeadTracking_confidence is_tracking(::chest_interfaces::msg::HeadTracking::_is_tracking_type arg)
  {
    msg_.is_tracking = std::move(arg);
    return Init_HeadTracking_confidence(msg_);
  }

private:
  ::chest_interfaces::msg::HeadTracking msg_;
};

class Init_HeadTracking_pitch
{
public:
  explicit Init_HeadTracking_pitch(::chest_interfaces::msg::HeadTracking & msg)
  : msg_(msg)
  {}
  Init_HeadTracking_is_tracking pitch(::chest_interfaces::msg::HeadTracking::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_HeadTracking_is_tracking(msg_);
  }

private:
  ::chest_interfaces::msg::HeadTracking msg_;
};

class Init_HeadTracking_yaw
{
public:
  explicit Init_HeadTracking_yaw(::chest_interfaces::msg::HeadTracking & msg)
  : msg_(msg)
  {}
  Init_HeadTracking_pitch yaw(::chest_interfaces::msg::HeadTracking::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_HeadTracking_pitch(msg_);
  }

private:
  ::chest_interfaces::msg::HeadTracking msg_;
};

class Init_HeadTracking_header
{
public:
  Init_HeadTracking_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeadTracking_yaw header(::chest_interfaces::msg::HeadTracking::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HeadTracking_yaw(msg_);
  }

private:
  ::chest_interfaces::msg::HeadTracking msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chest_interfaces::msg::HeadTracking>()
{
  return chest_interfaces::msg::builder::Init_HeadTracking_header();
}

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__BUILDER_HPP_
