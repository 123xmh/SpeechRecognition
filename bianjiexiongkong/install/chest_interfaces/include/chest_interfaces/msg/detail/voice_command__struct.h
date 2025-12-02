// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chest_interfaces:msg/VoiceCommand.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__STRUCT_H_
#define CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/VoiceCommand in the package chest_interfaces.
typedef struct chest_interfaces__msg__VoiceCommand
{
  std_msgs__msg__Header header;
  uint16_t frame_header;
  uint32_t command_id;
  int32_t param1;
  int32_t param2;
  uint8_t param3;
  bool is_valid;
} chest_interfaces__msg__VoiceCommand;

// Struct for a sequence of chest_interfaces__msg__VoiceCommand.
typedef struct chest_interfaces__msg__VoiceCommand__Sequence
{
  chest_interfaces__msg__VoiceCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chest_interfaces__msg__VoiceCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__STRUCT_H_
