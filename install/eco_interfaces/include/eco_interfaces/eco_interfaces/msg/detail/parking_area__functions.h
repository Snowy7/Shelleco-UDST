// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from eco_interfaces:msg/ParkingArea.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__FUNCTIONS_H_
#define ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "eco_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "eco_interfaces/msg/detail/parking_area__struct.h"

/// Initialize msg/ParkingArea message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eco_interfaces__msg__ParkingArea
 * )) before or use
 * eco_interfaces__msg__ParkingArea__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
bool
eco_interfaces__msg__ParkingArea__init(eco_interfaces__msg__ParkingArea * msg);

/// Finalize msg/ParkingArea message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
void
eco_interfaces__msg__ParkingArea__fini(eco_interfaces__msg__ParkingArea * msg);

/// Create msg/ParkingArea message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eco_interfaces__msg__ParkingArea__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
eco_interfaces__msg__ParkingArea *
eco_interfaces__msg__ParkingArea__create();

/// Destroy msg/ParkingArea message.
/**
 * It calls
 * eco_interfaces__msg__ParkingArea__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
void
eco_interfaces__msg__ParkingArea__destroy(eco_interfaces__msg__ParkingArea * msg);

/// Check for msg/ParkingArea message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
bool
eco_interfaces__msg__ParkingArea__are_equal(const eco_interfaces__msg__ParkingArea * lhs, const eco_interfaces__msg__ParkingArea * rhs);

/// Copy a msg/ParkingArea message.
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
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
bool
eco_interfaces__msg__ParkingArea__copy(
  const eco_interfaces__msg__ParkingArea * input,
  eco_interfaces__msg__ParkingArea * output);

/// Initialize array of msg/ParkingArea messages.
/**
 * It allocates the memory for the number of elements and calls
 * eco_interfaces__msg__ParkingArea__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
bool
eco_interfaces__msg__ParkingArea__Sequence__init(eco_interfaces__msg__ParkingArea__Sequence * array, size_t size);

/// Finalize array of msg/ParkingArea messages.
/**
 * It calls
 * eco_interfaces__msg__ParkingArea__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
void
eco_interfaces__msg__ParkingArea__Sequence__fini(eco_interfaces__msg__ParkingArea__Sequence * array);

/// Create array of msg/ParkingArea messages.
/**
 * It allocates the memory for the array and calls
 * eco_interfaces__msg__ParkingArea__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
eco_interfaces__msg__ParkingArea__Sequence *
eco_interfaces__msg__ParkingArea__Sequence__create(size_t size);

/// Destroy array of msg/ParkingArea messages.
/**
 * It calls
 * eco_interfaces__msg__ParkingArea__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
void
eco_interfaces__msg__ParkingArea__Sequence__destroy(eco_interfaces__msg__ParkingArea__Sequence * array);

/// Check for msg/ParkingArea message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
bool
eco_interfaces__msg__ParkingArea__Sequence__are_equal(const eco_interfaces__msg__ParkingArea__Sequence * lhs, const eco_interfaces__msg__ParkingArea__Sequence * rhs);

/// Copy an array of msg/ParkingArea messages.
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
ROSIDL_GENERATOR_C_PUBLIC_eco_interfaces
bool
eco_interfaces__msg__ParkingArea__Sequence__copy(
  const eco_interfaces__msg__ParkingArea__Sequence * input,
  eco_interfaces__msg__ParkingArea__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__FUNCTIONS_H_
