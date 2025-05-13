// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eco_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_
#define ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IDLE'.
enum
{
  eco_interfaces__msg__SystemState__IDLE = 0
};

/// Constant 'SECTION1_LANE_FOLLOWING'.
enum
{
  eco_interfaces__msg__SystemState__SECTION1_LANE_FOLLOWING = 1
};

/// Constant 'APPROACHING_STOP_SIGN_SECTION1'.
enum
{
  eco_interfaces__msg__SystemState__APPROACHING_STOP_SIGN_SECTION1 = 2
};

/// Constant 'STOPPED_SECTION1'.
enum
{
  eco_interfaces__msg__SystemState__STOPPED_SECTION1 = 3
};

/// Constant 'SECTION2_OBSTACLE_AVOIDANCE'.
enum
{
  eco_interfaces__msg__SystemState__SECTION2_OBSTACLE_AVOIDANCE = 4
};

/// Constant 'APPROACHING_STOP_SIGN_SECTION2'.
enum
{
  eco_interfaces__msg__SystemState__APPROACHING_STOP_SIGN_SECTION2 = 5
};

/// Constant 'STOPPED_SECTION2'.
enum
{
  eco_interfaces__msg__SystemState__STOPPED_SECTION2 = 6
};

/// Constant 'SECTION3_PARKING'.
enum
{
  eco_interfaces__msg__SystemState__SECTION3_PARKING = 7
};

/// Constant 'PARKED'.
enum
{
  eco_interfaces__msg__SystemState__PARKED = 8
};

/// Constant 'EMERGENCY_STOP'.
enum
{
  eco_interfaces__msg__SystemState__EMERGENCY_STOP = 9
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SystemState in the package eco_interfaces.
typedef struct eco_interfaces__msg__SystemState
{
  std_msgs__msg__Header header;
  uint8_t state;
} eco_interfaces__msg__SystemState;

// Struct for a sequence of eco_interfaces__msg__SystemState.
typedef struct eco_interfaces__msg__SystemState__Sequence
{
  eco_interfaces__msg__SystemState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eco_interfaces__msg__SystemState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_
