// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chest_interfaces:msg/EquipmentState.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__STRUCT_H_
#define CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ammo_types'
// Member 'warnings'
// Member 'state_names'
#include "rosidl_runtime_c/string.h"
// Member 'ammo_counts'
// Member 'state_values'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/EquipmentState in the package chest_interfaces.
typedef struct chest_interfaces__msg__EquipmentState
{
  rosidl_runtime_c__String__Sequence ammo_types;
  rosidl_runtime_c__uint8__Sequence ammo_counts;
  float gimbal_pitch;
  float gimbal_yaw;
  bool gimbal_is_active;
  double longitude;
  double latitude;
  double altitude;
  double ground_altitude;
  float heading;
  float speed;
  float fuel_level;
  float battery_level;
  float roll;
  float pitch;
  float yaw;
  rosidl_runtime_c__String__Sequence warnings;
  rosidl_runtime_c__String__Sequence state_names;
  rosidl_runtime_c__float__Sequence state_values;
} chest_interfaces__msg__EquipmentState;

// Struct for a sequence of chest_interfaces__msg__EquipmentState.
typedef struct chest_interfaces__msg__EquipmentState__Sequence
{
  chest_interfaces__msg__EquipmentState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chest_interfaces__msg__EquipmentState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__STRUCT_H_
