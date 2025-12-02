// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chest_interfaces:msg/VideoSource.idl
// generated code does not contain a copyright notice
#include "chest_interfaces/msg/detail/video_source__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `ip_address`
// Member `source_name`
// Member `encoding`
#include "rosidl_runtime_c/string_functions.h"

bool
chest_interfaces__msg__VideoSource__init(chest_interfaces__msg__VideoSource * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    chest_interfaces__msg__VideoSource__fini(msg);
    return false;
  }
  // id
  // ip_address
  if (!rosidl_runtime_c__String__init(&msg->ip_address)) {
    chest_interfaces__msg__VideoSource__fini(msg);
    return false;
  }
  // source_name
  if (!rosidl_runtime_c__String__init(&msg->source_name)) {
    chest_interfaces__msg__VideoSource__fini(msg);
    return false;
  }
  // fps
  // width
  // height
  // encoding
  if (!rosidl_runtime_c__String__init(&msg->encoding)) {
    chest_interfaces__msg__VideoSource__fini(msg);
    return false;
  }
  // is_available
  return true;
}

void
chest_interfaces__msg__VideoSource__fini(chest_interfaces__msg__VideoSource * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // ip_address
  rosidl_runtime_c__String__fini(&msg->ip_address);
  // source_name
  rosidl_runtime_c__String__fini(&msg->source_name);
  // fps
  // width
  // height
  // encoding
  rosidl_runtime_c__String__fini(&msg->encoding);
  // is_available
}

bool
chest_interfaces__msg__VideoSource__are_equal(const chest_interfaces__msg__VideoSource * lhs, const chest_interfaces__msg__VideoSource * rhs)
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
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // ip_address
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->ip_address), &(rhs->ip_address)))
  {
    return false;
  }
  // source_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->source_name), &(rhs->source_name)))
  {
    return false;
  }
  // fps
  if (lhs->fps != rhs->fps) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // encoding
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->encoding), &(rhs->encoding)))
  {
    return false;
  }
  // is_available
  if (lhs->is_available != rhs->is_available) {
    return false;
  }
  return true;
}

bool
chest_interfaces__msg__VideoSource__copy(
  const chest_interfaces__msg__VideoSource * input,
  chest_interfaces__msg__VideoSource * output)
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
  // id
  output->id = input->id;
  // ip_address
  if (!rosidl_runtime_c__String__copy(
      &(input->ip_address), &(output->ip_address)))
  {
    return false;
  }
  // source_name
  if (!rosidl_runtime_c__String__copy(
      &(input->source_name), &(output->source_name)))
  {
    return false;
  }
  // fps
  output->fps = input->fps;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // encoding
  if (!rosidl_runtime_c__String__copy(
      &(input->encoding), &(output->encoding)))
  {
    return false;
  }
  // is_available
  output->is_available = input->is_available;
  return true;
}

chest_interfaces__msg__VideoSource *
chest_interfaces__msg__VideoSource__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__VideoSource * msg = (chest_interfaces__msg__VideoSource *)allocator.allocate(sizeof(chest_interfaces__msg__VideoSource), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chest_interfaces__msg__VideoSource));
  bool success = chest_interfaces__msg__VideoSource__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chest_interfaces__msg__VideoSource__destroy(chest_interfaces__msg__VideoSource * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chest_interfaces__msg__VideoSource__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chest_interfaces__msg__VideoSource__Sequence__init(chest_interfaces__msg__VideoSource__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__VideoSource * data = NULL;

  if (size) {
    data = (chest_interfaces__msg__VideoSource *)allocator.zero_allocate(size, sizeof(chest_interfaces__msg__VideoSource), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chest_interfaces__msg__VideoSource__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chest_interfaces__msg__VideoSource__fini(&data[i - 1]);
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
chest_interfaces__msg__VideoSource__Sequence__fini(chest_interfaces__msg__VideoSource__Sequence * array)
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
      chest_interfaces__msg__VideoSource__fini(&array->data[i]);
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

chest_interfaces__msg__VideoSource__Sequence *
chest_interfaces__msg__VideoSource__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__VideoSource__Sequence * array = (chest_interfaces__msg__VideoSource__Sequence *)allocator.allocate(sizeof(chest_interfaces__msg__VideoSource__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chest_interfaces__msg__VideoSource__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chest_interfaces__msg__VideoSource__Sequence__destroy(chest_interfaces__msg__VideoSource__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chest_interfaces__msg__VideoSource__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chest_interfaces__msg__VideoSource__Sequence__are_equal(const chest_interfaces__msg__VideoSource__Sequence * lhs, const chest_interfaces__msg__VideoSource__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chest_interfaces__msg__VideoSource__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chest_interfaces__msg__VideoSource__Sequence__copy(
  const chest_interfaces__msg__VideoSource__Sequence * input,
  chest_interfaces__msg__VideoSource__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chest_interfaces__msg__VideoSource);
    chest_interfaces__msg__VideoSource * data =
      (chest_interfaces__msg__VideoSource *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chest_interfaces__msg__VideoSource__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          chest_interfaces__msg__VideoSource__fini(&data[i]);
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
    if (!chest_interfaces__msg__VideoSource__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
