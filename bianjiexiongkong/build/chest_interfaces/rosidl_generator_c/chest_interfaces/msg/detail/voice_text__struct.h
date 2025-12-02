// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chest_interfaces:msg/VoiceText.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__STRUCT_H_
#define CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__STRUCT_H_

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
// Member 'raw_text'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/VoiceText in the package chest_interfaces.
typedef struct chest_interfaces__msg__VoiceText
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String raw_text;
  bool is_final;
  float confidence;
} chest_interfaces__msg__VoiceText;

// Struct for a sequence of chest_interfaces__msg__VoiceText.
typedef struct chest_interfaces__msg__VoiceText__Sequence
{
  chest_interfaces__msg__VoiceText * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chest_interfaces__msg__VoiceText__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__STRUCT_H_
