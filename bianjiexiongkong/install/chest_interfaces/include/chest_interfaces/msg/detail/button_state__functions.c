// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chest_interfaces:msg/ButtonState.idl
// generated code does not contain a copyright notice
#include "chest_interfaces/msg/detail/button_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
chest_interfaces__msg__ButtonState__init(chest_interfaces__msg__ButtonState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    chest_interfaces__msg__ButtonState__fini(msg);
    return false;
  }
  // head_tracking_enabled
  // voice_command_enabled
  // front_screen_enabled
  // front_screen_mode
  // hmd_video_source
  // hmd_overlay_mode
  // hmd_enabled
  return true;
}

void
chest_interfaces__msg__ButtonState__fini(chest_interfaces__msg__ButtonState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // head_tracking_enabled
  // voice_command_enabled
  // front_screen_enabled
  // front_screen_mode
  // hmd_video_source
  // hmd_overlay_mode
  // hmd_enabled
}

bool
chest_interfaces__msg__ButtonState__are_equal(const chest_interfaces__msg__ButtonState * lhs, const chest_interfaces__msg__ButtonState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // head_tracking_enabled
  if (lhs->head_tracking_enabled != rhs->head_tracking_enabled) {
    return false;
  }
  // voice_command_enabled
  if (lhs->voice_command_enabled != rhs->voice_command_enabled) {
    return false;
  }
  // front_screen_enabled
  if (lhs->front_screen_enabled != rhs->front_screen_enabled) {
    return false;
  }
  // front_screen_mode
  if (lhs->front_screen_mode != rhs->front_screen_mode) {
    return false;
  }
  // hmd_video_source
  if (lhs->hmd_video_source != rhs->hmd_video_source) {
    return false;
  }
  // hmd_overlay_mode
  if (lhs->hmd_overlay_mode != rhs->hmd_overlay_mode) {
    return false;
  }
  // hmd_enabled
  if (lhs->hmd_enabled != rhs->hmd_enabled) {
    return false;
  }
  return true;
}

bool
chest_interfaces__msg__ButtonState__copy(
  const chest_interfaces__msg__ButtonState * input,
  chest_interfaces__msg__ButtonState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // head_tracking_enabled
  output->head_tracking_enabled = input->head_tracking_enabled;
  // voice_command_enabled
  output->voice_command_enabled = input->voice_command_enabled;
  // front_screen_enabled
  output->front_screen_enabled = input->front_screen_enabled;
  // front_screen_mode
  output->front_screen_mode = input->front_screen_mode;
  // hmd_video_source
  output->hmd_video_source = input->hmd_video_source;
  // hmd_overlay_mode
  output->hmd_overlay_mode = input->hmd_overlay_mode;
  // hmd_enabled
  output->hmd_enabled = input->hmd_enabled;
  return true;
}

chest_interfaces__msg__ButtonState *
chest_interfaces__msg__ButtonState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__ButtonState * msg = (chest_interfaces__msg__ButtonState *)allocator.allocate(sizeof(chest_interfaces__msg__ButtonState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chest_interfaces__msg__ButtonState));
  bool success = chest_interfaces__msg__ButtonState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chest_interfaces__msg__ButtonState__destroy(chest_interfaces__msg__ButtonState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chest_interfaces__msg__ButtonState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chest_interfaces__msg__ButtonState__Sequence__init(chest_interfaces__msg__ButtonState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__ButtonState * data = NULL;

  if (size) {
    data = (chest_interfaces__msg__ButtonState *)allocator.zero_allocate(size, sizeof(chest_interfaces__msg__ButtonState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chest_interfaces__msg__ButtonState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chest_interfaces__msg__ButtonState__fini(&data[i - 1]);
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
chest_interfaces__msg__ButtonState__Sequence__fini(chest_interfaces__msg__ButtonState__Sequence * array)
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
      chest_interfaces__msg__ButtonState__fini(&array->data[i]);
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

chest_interfaces__msg__ButtonState__Sequence *
chest_interfaces__msg__ButtonState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__ButtonState__Sequence * array = (chest_interfaces__msg__ButtonState__Sequence *)allocator.allocate(sizeof(chest_interfaces__msg__ButtonState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chest_interfaces__msg__ButtonState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chest_interfaces__msg__ButtonState__Sequence__destroy(chest_interfaces__msg__ButtonState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chest_interfaces__msg__ButtonState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chest_interfaces__msg__ButtonState__Sequence__are_equal(const chest_interfaces__msg__ButtonState__Sequence * lhs, const chest_interfaces__msg__ButtonState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chest_interfaces__msg__ButtonState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chest_interfaces__msg__ButtonState__Sequence__copy(
  const chest_interfaces__msg__ButtonState__Sequence * input,
  chest_interfaces__msg__ButtonState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chest_interfaces__msg__ButtonState);
    chest_interfaces__msg__ButtonState * data =
      (chest_interfaces__msg__ButtonState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chest_interfaces__msg__ButtonState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          chest_interfaces__msg__ButtonState__fini(&data[i]);
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
    if (!chest_interfaces__msg__ButtonState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
