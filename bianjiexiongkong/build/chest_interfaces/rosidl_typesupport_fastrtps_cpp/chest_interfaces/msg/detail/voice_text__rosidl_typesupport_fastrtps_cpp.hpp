// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from chest_interfaces:msg/VoiceText.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "chest_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "chest_interfaces/msg/detail/voice_text__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace chest_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
cdr_serialize(
  const chest_interfaces::msg::VoiceText & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  chest_interfaces::msg::VoiceText & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
get_serialized_size(
  const chest_interfaces::msg::VoiceText & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
max_serialized_size_VoiceText(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace chest_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, chest_interfaces, msg, VoiceText)();

#ifdef __cplusplus
}
#endif

#endif  // CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
