// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chest_interfaces:msg/EquipmentState.idl
// generated code does not contain a copyright notice
#include "chest_interfaces/msg/detail/equipment_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ammo_types`
// Member `warnings`
// Member `state_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `ammo_counts`
// Member `state_values`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
chest_interfaces__msg__EquipmentState__init(chest_interfaces__msg__EquipmentState * msg)
{
  if (!msg) {
    return false;
  }
  // ammo_types
  if (!rosidl_runtime_c__String__Sequence__init(&msg->ammo_types, 0)) {
    chest_interfaces__msg__EquipmentState__fini(msg);
    return false;
  }
  // ammo_counts
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->ammo_counts, 0)) {
    chest_interfaces__msg__EquipmentState__fini(msg);
    return false;
  }
  // gimbal_pitch
  // gimbal_yaw
  // gimbal_is_active
  // longitude
  // latitude
  // altitude
  // ground_altitude
  // heading
  // speed
  // fuel_level
  // battery_level
  // roll
  // pitch
  // yaw
  // warnings
  if (!rosidl_runtime_c__String__Sequence__init(&msg->warnings, 0)) {
    chest_interfaces__msg__EquipmentState__fini(msg);
    return false;
  }
  // state_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->state_names, 0)) {
    chest_interfaces__msg__EquipmentState__fini(msg);
    return false;
  }
  // state_values
  if (!rosidl_runtime_c__float__Sequence__init(&msg->state_values, 0)) {
    chest_interfaces__msg__EquipmentState__fini(msg);
    return false;
  }
  return true;
}

void
chest_interfaces__msg__EquipmentState__fini(chest_interfaces__msg__EquipmentState * msg)
{
  if (!msg) {
    return;
  }
  // ammo_types
  rosidl_runtime_c__String__Sequence__fini(&msg->ammo_types);
  // ammo_counts
  rosidl_runtime_c__uint8__Sequence__fini(&msg->ammo_counts);
  // gimbal_pitch
  // gimbal_yaw
  // gimbal_is_active
  // longitude
  // latitude
  // altitude
  // ground_altitude
  // heading
  // speed
  // fuel_level
  // battery_level
  // roll
  // pitch
  // yaw
  // warnings
  rosidl_runtime_c__String__Sequence__fini(&msg->warnings);
  // state_names
  rosidl_runtime_c__String__Sequence__fini(&msg->state_names);
  // state_values
  rosidl_runtime_c__float__Sequence__fini(&msg->state_values);
}

bool
chest_interfaces__msg__EquipmentState__are_equal(const chest_interfaces__msg__EquipmentState * lhs, const chest_interfaces__msg__EquipmentState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ammo_types
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->ammo_types), &(rhs->ammo_types)))
  {
    return false;
  }
  // ammo_counts
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->ammo_counts), &(rhs->ammo_counts)))
  {
    return false;
  }
  // gimbal_pitch
  if (lhs->gimbal_pitch != rhs->gimbal_pitch) {
    return false;
  }
  // gimbal_yaw
  if (lhs->gimbal_yaw != rhs->gimbal_yaw) {
    return false;
  }
  // gimbal_is_active
  if (lhs->gimbal_is_active != rhs->gimbal_is_active) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // ground_altitude
  if (lhs->ground_altitude != rhs->ground_altitude) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // fuel_level
  if (lhs->fuel_level != rhs->fuel_level) {
    return false;
  }
  // battery_level
  if (lhs->battery_level != rhs->battery_level) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->warnings), &(rhs->warnings)))
  {
    return false;
  }
  // state_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->state_names), &(rhs->state_names)))
  {
    return false;
  }
  // state_values
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->state_values), &(rhs->state_values)))
  {
    return false;
  }
  return true;
}

bool
chest_interfaces__msg__EquipmentState__copy(
  const chest_interfaces__msg__EquipmentState * input,
  chest_interfaces__msg__EquipmentState * output)
{
  if (!input || !output) {
    return false;
  }
  // ammo_types
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->ammo_types), &(output->ammo_types)))
  {
    return false;
  }
  // ammo_counts
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->ammo_counts), &(output->ammo_counts)))
  {
    return false;
  }
  // gimbal_pitch
  output->gimbal_pitch = input->gimbal_pitch;
  // gimbal_yaw
  output->gimbal_yaw = input->gimbal_yaw;
  // gimbal_is_active
  output->gimbal_is_active = input->gimbal_is_active;
  // longitude
  output->longitude = input->longitude;
  // latitude
  output->latitude = input->latitude;
  // altitude
  output->altitude = input->altitude;
  // ground_altitude
  output->ground_altitude = input->ground_altitude;
  // heading
  output->heading = input->heading;
  // speed
  output->speed = input->speed;
  // fuel_level
  output->fuel_level = input->fuel_level;
  // battery_level
  output->battery_level = input->battery_level;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // warnings
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->warnings), &(output->warnings)))
  {
    return false;
  }
  // state_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->state_names), &(output->state_names)))
  {
    return false;
  }
  // state_values
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->state_values), &(output->state_values)))
  {
    return false;
  }
  return true;
}

chest_interfaces__msg__EquipmentState *
chest_interfaces__msg__EquipmentState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__EquipmentState * msg = (chest_interfaces__msg__EquipmentState *)allocator.allocate(sizeof(chest_interfaces__msg__EquipmentState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chest_interfaces__msg__EquipmentState));
  bool success = chest_interfaces__msg__EquipmentState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chest_interfaces__msg__EquipmentState__destroy(chest_interfaces__msg__EquipmentState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chest_interfaces__msg__EquipmentState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chest_interfaces__msg__EquipmentState__Sequence__init(chest_interfaces__msg__EquipmentState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__EquipmentState * data = NULL;

  if (size) {
    data = (chest_interfaces__msg__EquipmentState *)allocator.zero_allocate(size, sizeof(chest_interfaces__msg__EquipmentState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chest_interfaces__msg__EquipmentState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chest_interfaces__msg__EquipmentState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
chest_interfaces__msg__EquipmentState__Sequence__fini(chest_interfaces__msg__EquipmentState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      chest_interfaces__msg__EquipmentState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

chest_interfaces__msg__EquipmentState__Sequence *
chest_interfaces__msg__EquipmentState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__EquipmentState__Sequence * array = (chest_interfaces__msg__EquipmentState__Sequence *)allocator.allocate(sizeof(chest_interfaces__msg__EquipmentState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chest_interfaces__msg__EquipmentState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chest_interfaces__msg__EquipmentState__Sequence__destroy(chest_interfaces__msg__EquipmentState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chest_interfaces__msg__EquipmentState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chest_interfaces__msg__EquipmentState__Sequence__are_equal(const chest_interfaces__msg__EquipmentState__Sequence * lhs, const chest_interfaces__msg__EquipmentState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chest_interfaces__msg__EquipmentState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chest_interfaces__msg__EquipmentState__Sequence__copy(
  const chest_interfaces__msg__EquipmentState__Sequence * input,
  chest_interfaces__msg__EquipmentState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chest_interfaces__msg__EquipmentState);
    chest_interfaces__msg__EquipmentState * data =
      (chest_interfaces__msg__EquipmentState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chest_interfaces__msg__EquipmentState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          chest_interfaces__msg__EquipmentState__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chest_interfaces__msg__EquipmentState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
