// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eco_interfaces:msg/Obstacles.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_H_
#define ECO_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'obstacles'
#include "eco_interfaces/msg/detail/obstacle__struct.h"

/// Struct defined in msg/Obstacles in the package eco_interfaces.
typedef struct eco_interfaces__msg__Obstacles
{
  std_msgs__msg__Header header;
  eco_interfaces__msg__Obstacle__Sequence obstacles;
} eco_interfaces__msg__Obstacles;

// Struct for a sequence of eco_interfaces__msg__Obstacles.
typedef struct eco_interfaces__msg__Obstacles__Sequence
{
  eco_interfaces__msg__Obstacles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eco_interfaces__msg__Obstacles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ECO_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_H_
