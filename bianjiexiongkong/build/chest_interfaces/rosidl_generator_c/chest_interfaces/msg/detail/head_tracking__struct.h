// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chest_interfaces:msg/HeadTracking.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__STRUCT_H_
#define CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__STRUCT_H_

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

// Struct defined in msg/HeadTracking in the package chest_interfaces.
typedef struct chest_interfaces__msg__HeadTracking
{
  std_msgs__msg__Header header;
  float yaw;
  float pitch;
  bool is_tracking;
  float confidence;
} chest_interfaces__msg__HeadTracking;

// Struct for a sequence of chest_interfaces__msg__HeadTracking.
typedef struct chest_interfaces__msg__HeadTracking__Sequence
{
  chest_interfaces__msg__HeadTracking * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chest_interfaces__msg__HeadTracking__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__STRUCT_H_
