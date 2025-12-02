// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chest_interfaces:msg/ButtonState.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__STRUCT_H_
#define CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FRONT_SCREEN_DEFAULT'.
enum
{
  chest_interfaces__msg__ButtonState__FRONT_SCREEN_DEFAULT = 0
};

/// Constant 'FRONT_SCREEN_MODE_A'.
enum
{
  chest_interfaces__msg__ButtonState__FRONT_SCREEN_MODE_A = 1
};

/// Constant 'FRONT_SCREEN_MODE_B'.
enum
{
  chest_interfaces__msg__ButtonState__FRONT_SCREEN_MODE_B = 2
};

/// Constant 'FRONT_SCREEN_MODE_C'.
enum
{
  chest_interfaces__msg__ButtonState__FRONT_SCREEN_MODE_C = 3
};

/// Constant 'HMD_OVERLAY_NONE'.
enum
{
  chest_interfaces__msg__ButtonState__HMD_OVERLAY_NONE = 0
};

/// Constant 'HMD_OVERLAY_MINIMAL'.
enum
{
  chest_interfaces__msg__ButtonState__HMD_OVERLAY_MINIMAL = 1
};

/// Constant 'HMD_OVERLAY_FULL'.
enum
{
  chest_interfaces__msg__ButtonState__HMD_OVERLAY_FULL = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/ButtonState in the package chest_interfaces.
typedef struct chest_interfaces__msg__ButtonState
{
  std_msgs__msg__Header header;
  bool head_tracking_enabled;
  bool voice_command_enabled;
  bool front_screen_enabled;
  uint8_t front_screen_mode;
  uint8_t hmd_video_source;
  uint8_t hmd_overlay_mode;
  bool hmd_enabled;
} chest_interfaces__msg__ButtonState;

// Struct for a sequence of chest_interfaces__msg__ButtonState.
typedef struct chest_interfaces__msg__ButtonState__Sequence
{
  chest_interfaces__msg__ButtonState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chest_interfaces__msg__ButtonState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__STRUCT_H_
