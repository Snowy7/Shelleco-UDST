// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eco_interfaces:msg/LaneDetection.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__STRUCT_H_
#define ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__STRUCT_H_

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
// Member 'left_line_x'
// Member 'left_line_y'
// Member 'right_line_x'
// Member 'right_line_y'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/LaneDetection in the package eco_interfaces.
typedef struct eco_interfaces__msg__LaneDetection
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__float__Sequence left_line_x;
  rosidl_runtime_c__float__Sequence left_line_y;
  rosidl_runtime_c__float__Sequence right_line_x;
  rosidl_runtime_c__float__Sequence right_line_y;
  /// Offset from vehicle center to lane center
  float lane_center_offset;
  /// Heading error relative to lane
  float lane_heading_error;
} eco_interfaces__msg__LaneDetection;

// Struct for a sequence of eco_interfaces__msg__LaneDetection.
typedef struct eco_interfaces__msg__LaneDetection__Sequence
{
  eco_interfaces__msg__LaneDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eco_interfaces__msg__LaneDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__STRUCT_H_
