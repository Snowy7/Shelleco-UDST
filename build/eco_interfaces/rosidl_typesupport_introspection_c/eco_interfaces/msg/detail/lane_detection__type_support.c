// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from eco_interfaces:msg/LaneDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "eco_interfaces/msg/detail/lane_detection__rosidl_typesupport_introspection_c.h"
#include "eco_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "eco_interfaces/msg/detail/lane_detection__functions.h"
#include "eco_interfaces/msg/detail/lane_detection__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `left_line_x`
// Member `left_line_y`
// Member `right_line_x`
// Member `right_line_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  eco_interfaces__msg__LaneDetection__init(message_memory);
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_fini_function(void * message_memory)
{
  eco_interfaces__msg__LaneDetection__fini(message_memory);
}

size_t eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__size_function__LaneDetection__left_line_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__left_line_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__left_line_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__fetch_function__LaneDetection__left_line_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__left_line_x(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__assign_function__LaneDetection__left_line_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__left_line_x(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__resize_function__LaneDetection__left_line_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__size_function__LaneDetection__left_line_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__left_line_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__left_line_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__fetch_function__LaneDetection__left_line_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__left_line_y(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__assign_function__LaneDetection__left_line_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__left_line_y(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__resize_function__LaneDetection__left_line_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__size_function__LaneDetection__right_line_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__right_line_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__right_line_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__fetch_function__LaneDetection__right_line_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__right_line_x(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__assign_function__LaneDetection__right_line_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__right_line_x(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__resize_function__LaneDetection__right_line_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__size_function__LaneDetection__right_line_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__right_line_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__right_line_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__fetch_function__LaneDetection__right_line_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__right_line_y(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__assign_function__LaneDetection__right_line_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__right_line_y(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__resize_function__LaneDetection__right_line_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces__msg__LaneDetection, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_line_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces__msg__LaneDetection, left_line_x),  // bytes offset in struct
    NULL,  // default value
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__size_function__LaneDetection__left_line_x,  // size() function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__left_line_x,  // get_const(index) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__left_line_x,  // get(index) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__fetch_function__LaneDetection__left_line_x,  // fetch(index, &value) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__assign_function__LaneDetection__left_line_x,  // assign(index, value) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__resize_function__LaneDetection__left_line_x  // resize(index) function pointer
  },
  {
    "left_line_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces__msg__LaneDetection, left_line_y),  // bytes offset in struct
    NULL,  // default value
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__size_function__LaneDetection__left_line_y,  // size() function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__left_line_y,  // get_const(index) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__left_line_y,  // get(index) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__fetch_function__LaneDetection__left_line_y,  // fetch(index, &value) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__assign_function__LaneDetection__left_line_y,  // assign(index, value) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__resize_function__LaneDetection__left_line_y  // resize(index) function pointer
  },
  {
    "right_line_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces__msg__LaneDetection, right_line_x),  // bytes offset in struct
    NULL,  // default value
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__size_function__LaneDetection__right_line_x,  // size() function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__right_line_x,  // get_const(index) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__right_line_x,  // get(index) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__fetch_function__LaneDetection__right_line_x,  // fetch(index, &value) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__assign_function__LaneDetection__right_line_x,  // assign(index, value) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__resize_function__LaneDetection__right_line_x  // resize(index) function pointer
  },
  {
    "right_line_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces__msg__LaneDetection, right_line_y),  // bytes offset in struct
    NULL,  // default value
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__size_function__LaneDetection__right_line_y,  // size() function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_const_function__LaneDetection__right_line_y,  // get_const(index) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__get_function__LaneDetection__right_line_y,  // get(index) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__fetch_function__LaneDetection__right_line_y,  // fetch(index, &value) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__assign_function__LaneDetection__right_line_y,  // assign(index, value) function pointer
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__resize_function__LaneDetection__right_line_y  // resize(index) function pointer
  },
  {
    "lane_center_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces__msg__LaneDetection, lane_center_offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lane_heading_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces__msg__LaneDetection, lane_heading_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_members = {
  "eco_interfaces__msg",  // message namespace
  "LaneDetection",  // message name
  7,  // number of fields
  sizeof(eco_interfaces__msg__LaneDetection),
  eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_member_array,  // message members
  eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_type_support_handle = {
  0,
  &eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_eco_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, eco_interfaces, msg, LaneDetection)() {
  eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_type_support_handle.typesupport_identifier) {
    eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &eco_interfaces__msg__LaneDetection__rosidl_typesupport_introspection_c__LaneDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
