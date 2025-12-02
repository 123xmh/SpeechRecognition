// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from chest_interfaces:msg/ButtonState.idl
// generated code does not contain a copyright notice
#include "chest_interfaces/msg/detail/button_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "chest_interfaces/msg/detail/button_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace chest_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
cdr_serialize(
  const chest_interfaces::msg::ButtonState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: head_tracking_enabled
  cdr << (ros_message.head_tracking_enabled ? true : false);
  // Member: voice_command_enabled
  cdr << (ros_message.voice_command_enabled ? true : false);
  // Member: front_screen_enabled
  cdr << (ros_message.front_screen_enabled ? true : false);
  // Member: front_screen_mode
  cdr << ros_message.front_screen_mode;
  // Member: hmd_video_source
  cdr << ros_message.hmd_video_source;
  // Member: hmd_overlay_mode
  cdr << ros_message.hmd_overlay_mode;
  // Member: hmd_enabled
  cdr << (ros_message.hmd_enabled ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  chest_interfaces::msg::ButtonState & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: head_tracking_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.head_tracking_enabled = tmp ? true : false;
  }

  // Member: voice_command_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.voice_command_enabled = tmp ? true : false;
  }

  // Member: front_screen_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.front_screen_enabled = tmp ? true : false;
  }

  // Member: front_screen_mode
  cdr >> ros_message.front_screen_mode;

  // Member: hmd_video_source
  cdr >> ros_message.hmd_video_source;

  // Member: hmd_overlay_mode
  cdr >> ros_message.hmd_overlay_mode;

  // Member: hmd_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.hmd_enabled = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
get_serialized_size(
  const chest_interfaces::msg::ButtonState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: head_tracking_enabled
  {
    size_t item_size = sizeof(ros_message.head_tracking_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: voice_command_enabled
  {
    size_t item_size = sizeof(ros_message.voice_command_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_screen_enabled
  {
    size_t item_size = sizeof(ros_message.front_screen_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_screen_mode
  {
    size_t item_size = sizeof(ros_message.front_screen_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hmd_video_source
  {
    size_t item_size = sizeof(ros_message.hmd_video_source);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hmd_overlay_mode
  {
    size_t item_size = sizeof(ros_message.hmd_overlay_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hmd_enabled
  {
    size_t item_size = sizeof(ros_message.hmd_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chest_interfaces
max_serialized_size_ButtonState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: head_tracking_enabled
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: voice_command_enabled
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: front_screen_enabled
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: front_screen_mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: hmd_video_source
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: hmd_overlay_mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: hmd_enabled
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _ButtonState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const chest_interfaces::msg::ButtonState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ButtonState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<chest_interfaces::msg::ButtonState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ButtonState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const chest_interfaces::msg::ButtonState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ButtonState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ButtonState(full_bounded, 0);
}

static message_type_support_callbacks_t _ButtonState__callbacks = {
  "chest_interfaces::msg",
  "ButtonState",
  _ButtonState__cdr_serialize,
  _ButtonState__cdr_deserialize,
  _ButtonState__get_serialized_size,
  _ButtonState__max_serialized_size
};

static rosidl_message_type_support_t _ButtonState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ButtonState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace chest_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_chest_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<chest_interfaces::msg::ButtonState>()
{
  return &chest_interfaces::msg::typesupport_fastrtps_cpp::_ButtonState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, chest_interfaces, msg, ButtonState)() {
  return &chest_interfaces::msg::typesupport_fastrtps_cpp::_ButtonState__handle;
}

#ifdef __cplusplus
}
#endif
