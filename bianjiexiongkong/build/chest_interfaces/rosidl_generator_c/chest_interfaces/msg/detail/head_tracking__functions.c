// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chest_interfaces:msg/HeadTracking.idl
// generated code does not contain a copyright notice
#include "chest_interfaces/msg/detail/head_tracking__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
chest_interfaces__msg__HeadTracking__init(chest_interfaces__msg__HeadTracking * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    chest_interfaces__msg__HeadTracking__fini(msg);
    return false;
  }
  // yaw
  // pitch
  // is_tracking
  // confidence
  return true;
}

void
chest_interfaces__msg__HeadTracking__fini(chest_interfaces__msg__HeadTracking * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // yaw
  // pitch
  // is_tracking
  // confidence
}

bool
chest_interfaces__msg__HeadTracking__are_equal(const chest_interfaces__msg__HeadTracking * lhs, const chest_interfaces__msg__HeadTracking * rhs)
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
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // is_tracking
  if (lhs->is_tracking != rhs->is_tracking) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  return true;
}

bool
chest_interfaces__msg__HeadTracking__copy(
  const chest_interfaces__msg__HeadTracking * input,
  chest_interfaces__msg__HeadTracking * output)
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
  // yaw
  output->yaw = input->yaw;
  // pitch
  output->pitch = input->pitch;
  // is_tracking
  output->is_tracking = input->is_tracking;
  // confidence
  output->confidence = input->confidence;
  return true;
}

chest_interfaces__msg__HeadTracking *
chest_interfaces__msg__HeadTracking__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__HeadTracking * msg = (chest_interfaces__msg__HeadTracking *)allocator.allocate(sizeof(chest_interfaces__msg__HeadTracking), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chest_interfaces__msg__HeadTracking));
  bool success = chest_interfaces__msg__HeadTracking__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chest_interfaces__msg__HeadTracking__destroy(chest_interfaces__msg__HeadTracking * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chest_interfaces__msg__HeadTracking__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chest_interfaces__msg__HeadTracking__Sequence__init(chest_interfaces__msg__HeadTracking__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__HeadTracking * data = NULL;

  if (size) {
    data = (chest_interfaces__msg__HeadTracking *)allocator.zero_allocate(size, sizeof(chest_interfaces__msg__HeadTracking), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chest_interfaces__msg__HeadTracking__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chest_interfaces__msg__HeadTracking__fini(&data[i - 1]);
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
chest_interfaces__msg__HeadTracking__Sequence__fini(chest_interfaces__msg__HeadTracking__Sequence * array)
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
      chest_interfaces__msg__HeadTracking__fini(&array->data[i]);
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

chest_interfaces__msg__HeadTracking__Sequence *
chest_interfaces__msg__HeadTracking__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chest_interfaces__msg__HeadTracking__Sequence * array = (chest_interfaces__msg__HeadTracking__Sequence *)allocator.allocate(sizeof(chest_interfaces__msg__HeadTracking__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chest_interfaces__msg__HeadTracking__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chest_interfaces__msg__HeadTracking__Sequence__destroy(chest_interfaces__msg__HeadTracking__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chest_interfaces__msg__HeadTracking__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chest_interfaces__msg__HeadTracking__Sequence__are_equal(const chest_interfaces__msg__HeadTracking__Sequence * lhs, const chest_interfaces__msg__HeadTracking__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chest_interfaces__msg__HeadTracking__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chest_interfaces__msg__HeadTracking__Sequence__copy(
  const chest_interfaces__msg__HeadTracking__Sequence * input,
  chest_interfaces__msg__HeadTracking__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chest_interfaces__msg__HeadTracking);
    chest_interfaces__msg__HeadTracking * data =
      (chest_interfaces__msg__HeadTracking *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chest_interfaces__msg__HeadTracking__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          chest_interfaces__msg__HeadTracking__fini(&data[i]);
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
    if (!chest_interfaces__msg__HeadTracking__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
