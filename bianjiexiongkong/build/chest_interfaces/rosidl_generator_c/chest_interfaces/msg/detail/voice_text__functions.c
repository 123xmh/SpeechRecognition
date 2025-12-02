// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chest_interfaces:msg/VoiceText.idl
// generated code does not contain a copyright notice
#include "chest_interfaces/msg/detail/voice_text__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `raw_text`
#include "rosidl_runtime_c/string_functions.h"

bool
chest_interfaces__msg__VoiceText__init(chest_interfaces__msg__VoiceText * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    chest_interfaces__msg__VoiceText__fini(msg);
    return false;
  }
  // raw_text
  if (!rosidl_runtime_c__String__init(&msg->raw_text)) {
    chest_interfaces__msg__VoiceText__fini(msg);
    return false;
  }
  // is_final
  // confidence
  return true;
}

void
chest_interfaces__msg__VoiceText__fini(chest_interfaces__msg__VoiceText * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // raw_text
  rosidl_runtime_c__String__fini(&msg->raw_text);
  // is_final
  // confidence
}

bool
chest_interfaces__msg__VoiceText__are_equal(const chest_interfaces__msg__VoiceText * lhs, const chest_interfaces__msg__VoiceText * rhs)
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
  // raw_text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->raw_text), &(rhs->raw_text)))
  {
    return false;
  }
  // is_final
  if (lhs->is_final != rhs->is_final) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  return true;
}

bool
chest_interfaces__msg__VoiceText__copy(
  const chest_interfaces__msg__VoiceText * input,
  chest_interfaces__msg__VoiceText * output)
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
  // raw_text
  if (!rosidl_runtime_c__String__copy(
      &(input->raw_text), &(output->raw_text)))
  {
    return false;
  }
  // is_final
  output->is_final = input->is_final;
  // confidence
  output->confidence = input->confidence;
  return true;
}

chest_interfaces__msg__VoiceText *
chest_interfaces__msg__VoiceText__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__VoiceText * msg = (chest_interfaces__msg__VoiceText *)allocator.allocate(sizeof(chest_interfaces__msg__VoiceText), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chest_interfaces__msg__VoiceText));
  bool success = chest_interfaces__msg__VoiceText__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chest_interfaces__msg__VoiceText__destroy(chest_interfaces__msg__VoiceText * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chest_interfaces__msg__VoiceText__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chest_interfaces__msg__VoiceText__Sequence__init(chest_interfaces__msg__VoiceText__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__VoiceText * data = NULL;

  if (size) {
    data = (chest_interfaces__msg__VoiceText *)allocator.zero_allocate(size, sizeof(chest_interfaces__msg__VoiceText), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chest_interfaces__msg__VoiceText__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chest_interfaces__msg__VoiceText__fini(&data[i - 1]);
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
chest_interfaces__msg__VoiceText__Sequence__fini(chest_interfaces__msg__VoiceText__Sequence * array)
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
      chest_interfaces__msg__VoiceText__fini(&array->data[i]);
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

chest_interfaces__msg__VoiceText__Sequence *
chest_interfaces__msg__VoiceText__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__VoiceText__Sequence * array = (chest_interfaces__msg__VoiceText__Sequence *)allocator.allocate(sizeof(chest_interfaces__msg__VoiceText__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chest_interfaces__msg__VoiceText__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chest_interfaces__msg__VoiceText__Sequence__destroy(chest_interfaces__msg__VoiceText__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chest_interfaces__msg__VoiceText__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chest_interfaces__msg__VoiceText__Sequence__are_equal(const chest_interfaces__msg__VoiceText__Sequence * lhs, const chest_interfaces__msg__VoiceText__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chest_interfaces__msg__VoiceText__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chest_interfaces__msg__VoiceText__Sequence__copy(
  const chest_interfaces__msg__VoiceText__Sequence * input,
  chest_interfaces__msg__VoiceText__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chest_interfaces__msg__VoiceText);
    chest_interfaces__msg__VoiceText * data =
      (chest_interfaces__msg__VoiceText *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chest_interfaces__msg__VoiceText__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          chest_interfaces__msg__VoiceText__fini(&data[i]);
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
    if (!chest_interfaces__msg__VoiceText__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
