// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chest_interfaces:msg/VideoSource.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__STRUCT_H_
#define CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__STRUCT_H_

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
// Member 'ip_address'
// Member 'source_name'
// Member 'encoding'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/VideoSource in the package chest_interfaces.
typedef struct chest_interfaces__msg__VideoSource
{
  std_msgs__msg__Header header;
  uint8_t id;
  rosidl_runtime_c__String ip_address;
  rosidl_runtime_c__String source_name;
  uint32_t fps;
  uint32_t width;
  uint32_t height;
  rosidl_runtime_c__String encoding;
  bool is_available;
} chest_interfaces__msg__VideoSource;

// Struct for a sequence of chest_interfaces__msg__VideoSource.
typedef struct chest_interfaces__msg__VideoSource__Sequence
{
  chest_interfaces__msg__VideoSource * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chest_interfaces__msg__VideoSource__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__STRUCT_H_
