// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from chest_interfaces:msg/HeadTracking.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__FUNCTIONS_H_
#define CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "chest_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "chest_interfaces/msg/detail/head_tracking__struct.h"

/// Initialize msg/HeadTracking message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * chest_interfaces__msg__HeadTracking
 * )) before or use
 * chest_interfaces__msg__HeadTracking__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
bool
chest_interfaces__msg__HeadTracking__init(chest_interfaces__msg__HeadTracking * msg);

/// Finalize msg/HeadTracking message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
void
chest_interfaces__msg__HeadTracking__fini(chest_interfaces__msg__HeadTracking * msg);

/// Create msg/HeadTracking message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * chest_interfaces__msg__HeadTracking__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
chest_interfaces__msg__HeadTracking *
chest_interfaces__msg__HeadTracking__create();

/// Destroy msg/HeadTracking message.
/**
 * It calls
 * chest_interfaces__msg__HeadTracking__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
void
chest_interfaces__msg__HeadTracking__destroy(chest_interfaces__msg__HeadTracking * msg);

/// Check for msg/HeadTracking message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
bool
chest_interfaces__msg__HeadTracking__are_equal(const chest_interfaces__msg__HeadTracking * lhs, const chest_interfaces__msg__HeadTracking * rhs);

/// Copy a msg/HeadTracking message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
bool
chest_interfaces__msg__HeadTracking__copy(
  const chest_interfaces__msg__HeadTracking * input,
  chest_interfaces__msg__HeadTracking * output);

/// Initialize array of msg/HeadTracking messages.
/**
 * It allocates the memory for the number of elements and calls
 * chest_interfaces__msg__HeadTracking__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
bool
chest_interfaces__msg__HeadTracking__Sequence__init(chest_interfaces__msg__HeadTracking__Sequence * array, size_t size);

/// Finalize array of msg/HeadTracking messages.
/**
 * It calls
 * chest_interfaces__msg__HeadTracking__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
void
chest_interfaces__msg__HeadTracking__Sequence__fini(chest_interfaces__msg__HeadTracking__Sequence * array);

/// Create array of msg/HeadTracking messages.
/**
 * It allocates the memory for the array and calls
 * chest_interfaces__msg__HeadTracking__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
chest_interfaces__msg__HeadTracking__Sequence *
chest_interfaces__msg__HeadTracking__Sequence__create(size_t size);

/// Destroy array of msg/HeadTracking messages.
/**
 * It calls
 * chest_interfaces__msg__HeadTracking__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
void
chest_interfaces__msg__HeadTracking__Sequence__destroy(chest_interfaces__msg__HeadTracking__Sequence * array);

/// Check for msg/HeadTracking message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
bool
chest_interfaces__msg__HeadTracking__Sequence__are_equal(const chest_interfaces__msg__HeadTracking__Sequence * lhs, const chest_interfaces__msg__HeadTracking__Sequence * rhs);

/// Copy an array of msg/HeadTracking messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_chest_interfaces
bool
chest_interfaces__msg__HeadTracking__Sequence__copy(
  const chest_interfaces__msg__HeadTracking__Sequence * input,
  chest_interfaces__msg__HeadTracking__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CHEST_INTERFACES__MSG__DETAIL__HEAD_TRACKING__FUNCTIONS_H_
