// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from geometry_msgs:msg/AccelWithCovarianceStamped.idl
// generated code does not contain a copyright notice

#ifndef GEOMETRY_MSGS__MSG__DETAIL__ACCEL_WITH_COVARIANCE_STAMPED__FUNCTIONS_H_
#define GEOMETRY_MSGS__MSG__DETAIL__ACCEL_WITH_COVARIANCE_STAMPED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "geometry_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "geometry_msgs/msg/detail/accel_with_covariance_stamped__struct.h"

/// Initialize msg/AccelWithCovarianceStamped message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * geometry_msgs__msg__AccelWithCovarianceStamped
 * )) before or use
 * geometry_msgs__msg__AccelWithCovarianceStamped__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
bool
geometry_msgs__msg__AccelWithCovarianceStamped__init(geometry_msgs__msg__AccelWithCovarianceStamped * msg);

/// Finalize msg/AccelWithCovarianceStamped message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
void
geometry_msgs__msg__AccelWithCovarianceStamped__fini(geometry_msgs__msg__AccelWithCovarianceStamped * msg);

/// Create msg/AccelWithCovarianceStamped message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * geometry_msgs__msg__AccelWithCovarianceStamped__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
geometry_msgs__msg__AccelWithCovarianceStamped *
geometry_msgs__msg__AccelWithCovarianceStamped__create();

/// Destroy msg/AccelWithCovarianceStamped message.
/**
 * It calls
 * geometry_msgs__msg__AccelWithCovarianceStamped__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
void
geometry_msgs__msg__AccelWithCovarianceStamped__destroy(geometry_msgs__msg__AccelWithCovarianceStamped * msg);

/// Check for msg/AccelWithCovarianceStamped message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
bool
geometry_msgs__msg__AccelWithCovarianceStamped__are_equal(const geometry_msgs__msg__AccelWithCovarianceStamped * lhs, const geometry_msgs__msg__AccelWithCovarianceStamped * rhs);

/// Copy a msg/AccelWithCovarianceStamped message.
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
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
bool
geometry_msgs__msg__AccelWithCovarianceStamped__copy(
  const geometry_msgs__msg__AccelWithCovarianceStamped * input,
  geometry_msgs__msg__AccelWithCovarianceStamped * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
const rosidl_type_hash_t *
geometry_msgs__msg__AccelWithCovarianceStamped__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
const rosidl_runtime_c__type_description__TypeDescription *
geometry_msgs__msg__AccelWithCovarianceStamped__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
const rosidl_runtime_c__type_description__TypeSource *
geometry_msgs__msg__AccelWithCovarianceStamped__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
geometry_msgs__msg__AccelWithCovarianceStamped__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/AccelWithCovarianceStamped messages.
/**
 * It allocates the memory for the number of elements and calls
 * geometry_msgs__msg__AccelWithCovarianceStamped__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
bool
geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__init(geometry_msgs__msg__AccelWithCovarianceStamped__Sequence * array, size_t size);

/// Finalize array of msg/AccelWithCovarianceStamped messages.
/**
 * It calls
 * geometry_msgs__msg__AccelWithCovarianceStamped__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
void
geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__fini(geometry_msgs__msg__AccelWithCovarianceStamped__Sequence * array);

/// Create array of msg/AccelWithCovarianceStamped messages.
/**
 * It allocates the memory for the array and calls
 * geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
geometry_msgs__msg__AccelWithCovarianceStamped__Sequence *
geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__create(size_t size);

/// Destroy array of msg/AccelWithCovarianceStamped messages.
/**
 * It calls
 * geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
void
geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__destroy(geometry_msgs__msg__AccelWithCovarianceStamped__Sequence * array);

/// Check for msg/AccelWithCovarianceStamped message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
bool
geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__are_equal(const geometry_msgs__msg__AccelWithCovarianceStamped__Sequence * lhs, const geometry_msgs__msg__AccelWithCovarianceStamped__Sequence * rhs);

/// Copy an array of msg/AccelWithCovarianceStamped messages.
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
ROSIDL_GENERATOR_C_PUBLIC_geometry_msgs
bool
geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__copy(
  const geometry_msgs__msg__AccelWithCovarianceStamped__Sequence * input,
  geometry_msgs__msg__AccelWithCovarianceStamped__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GEOMETRY_MSGS__MSG__DETAIL__ACCEL_WITH_COVARIANCE_STAMPED__FUNCTIONS_H_
