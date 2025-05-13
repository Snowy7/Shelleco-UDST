// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eco_interfaces:msg/ParkingArea.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__STRUCT_H_
#define ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__STRUCT_H_

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
// Member 'parking_spot'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/ParkingArea in the package eco_interfaces.
typedef struct eco_interfaces__msg__ParkingArea
{
  std_msgs__msg__Header header;
  bool detected;
  /// Position and orientation of parking spot
  geometry_msgs__msg__Pose parking_spot;
} eco_interfaces__msg__ParkingArea;

// Struct for a sequence of eco_interfaces__msg__ParkingArea.
typedef struct eco_interfaces__msg__ParkingArea__Sequence
{
  eco_interfaces__msg__ParkingArea * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eco_interfaces__msg__ParkingArea__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__STRUCT_H_
