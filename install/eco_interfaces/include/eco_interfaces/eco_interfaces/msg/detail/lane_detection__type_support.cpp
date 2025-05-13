// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from eco_interfaces:msg/LaneDetection.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "eco_interfaces/msg/detail/lane_detection__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace eco_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LaneDetection_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) eco_interfaces::msg::LaneDetection(_init);
}

void LaneDetection_fini_function(void * message_memory)
{
  auto typed_message = static_cast<eco_interfaces::msg::LaneDetection *>(message_memory);
  typed_message->~LaneDetection();
}

size_t size_function__LaneDetection__left_line_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LaneDetection__left_line_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LaneDetection__left_line_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__LaneDetection__left_line_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__LaneDetection__left_line_x(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__LaneDetection__left_line_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__LaneDetection__left_line_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__LaneDetection__left_line_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LaneDetection__left_line_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LaneDetection__left_line_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LaneDetection__left_line_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__LaneDetection__left_line_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__LaneDetection__left_line_y(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__LaneDetection__left_line_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__LaneDetection__left_line_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__LaneDetection__left_line_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LaneDetection__right_line_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LaneDetection__right_line_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LaneDetection__right_line_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__LaneDetection__right_line_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__LaneDetection__right_line_x(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__LaneDetection__right_line_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__LaneDetection__right_line_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__LaneDetection__right_line_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LaneDetection__right_line_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LaneDetection__right_line_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LaneDetection__right_line_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__LaneDetection__right_line_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__LaneDetection__right_line_y(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__LaneDetection__right_line_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__LaneDetection__right_line_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__LaneDetection__right_line_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LaneDetection_message_member_array[7] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::LaneDetection, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_line_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::LaneDetection, left_line_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__LaneDetection__left_line_x,  // size() function pointer
    get_const_function__LaneDetection__left_line_x,  // get_const(index) function pointer
    get_function__LaneDetection__left_line_x,  // get(index) function pointer
    fetch_function__LaneDetection__left_line_x,  // fetch(index, &value) function pointer
    assign_function__LaneDetection__left_line_x,  // assign(index, value) function pointer
    resize_function__LaneDetection__left_line_x  // resize(index) function pointer
  },
  {
    "left_line_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::LaneDetection, left_line_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__LaneDetection__left_line_y,  // size() function pointer
    get_const_function__LaneDetection__left_line_y,  // get_const(index) function pointer
    get_function__LaneDetection__left_line_y,  // get(index) function pointer
    fetch_function__LaneDetection__left_line_y,  // fetch(index, &value) function pointer
    assign_function__LaneDetection__left_line_y,  // assign(index, value) function pointer
    resize_function__LaneDetection__left_line_y  // resize(index) function pointer
  },
  {
    "right_line_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::LaneDetection, right_line_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__LaneDetection__right_line_x,  // size() function pointer
    get_const_function__LaneDetection__right_line_x,  // get_const(index) function pointer
    get_function__LaneDetection__right_line_x,  // get(index) function pointer
    fetch_function__LaneDetection__right_line_x,  // fetch(index, &value) function pointer
    assign_function__LaneDetection__right_line_x,  // assign(index, value) function pointer
    resize_function__LaneDetection__right_line_x  // resize(index) function pointer
  },
  {
    "right_line_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::LaneDetection, right_line_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__LaneDetection__right_line_y,  // size() function pointer
    get_const_function__LaneDetection__right_line_y,  // get_const(index) function pointer
    get_function__LaneDetection__right_line_y,  // get(index) function pointer
    fetch_function__LaneDetection__right_line_y,  // fetch(index, &value) function pointer
    assign_function__LaneDetection__right_line_y,  // assign(index, value) function pointer
    resize_function__LaneDetection__right_line_y  // resize(index) function pointer
  },
  {
    "lane_center_offset",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::LaneDetection, lane_center_offset),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lane_heading_error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::LaneDetection, lane_heading_error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LaneDetection_message_members = {
  "eco_interfaces::msg",  // message namespace
  "LaneDetection",  // message name
  7,  // number of fields
  sizeof(eco_interfaces::msg::LaneDetection),
  LaneDetection_message_member_array,  // message members
  LaneDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  LaneDetection_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LaneDetection_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LaneDetection_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace eco_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<eco_interfaces::msg::LaneDetection>()
{
  return &::eco_interfaces::msg::rosidl_typesupport_introspection_cpp::LaneDetection_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, eco_interfaces, msg, LaneDetection)() {
  return &::eco_interfaces::msg::rosidl_typesupport_introspection_cpp::LaneDetection_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
