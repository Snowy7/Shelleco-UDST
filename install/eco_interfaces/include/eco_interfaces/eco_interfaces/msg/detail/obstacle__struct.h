// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eco_interfaces:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__OBSTACLE__STRUCT_H_
#define ECO_INTERFACES__MSG__DETAIL__OBSTACLE__STRUCT_H_

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
// Member 'points'
// Member 'center'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Obstacle in the package eco_interfaces.
typedef struct eco_interfaces__msg__Obstacle
{
  std_msgs__msg__Header header;
  /// Points in the obstacle cluster
  geometry_msgs__msg__Point__Sequence points;
  /// Center of the obstacle
  geometry_msgs__msg__Point center;
  /// Approximate radius of the obstacle
  float radius;
} eco_interfaces__msg__Obstacle;

// Struct for a sequence of eco_interfaces__msg__Obstacle.
typedef struct eco_interfaces__msg__Obstacle__Sequence
{
  eco_interfaces__msg__Obstacle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eco_interfaces__msg__Obstacle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ECO_INTERFACES__MSG__DETAIL__OBSTACLE__STRUCT_H_
