// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from eco_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "eco_interfaces/msg/detail/system_state__struct.hpp"
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

void SystemState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) eco_interfaces::msg::SystemState(_init);
}

void SystemState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<eco_interfaces::msg::SystemState *>(message_memory);
  typed_message->~SystemState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SystemState_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::SystemState, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(eco_interfaces::msg::SystemState, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SystemState_message_members = {
  "eco_interfaces::msg",  // message namespace
  "SystemState",  // message name
  2,  // number of fields
  sizeof(eco_interfaces::msg::SystemState),
  SystemState_message_member_array,  // message members
  SystemState_init_function,  // function to initialize message memory (memory has to be allocated)
  SystemState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SystemState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SystemState_message_members,
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
get_message_type_support_handle<eco_interfaces::msg::SystemState>()
{
  return &::eco_interfaces::msg::rosidl_typesupport_introspection_cpp::SystemState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, eco_interfaces, msg, SystemState)() {
  return &::eco_interfaces::msg::rosidl_typesupport_introspection_cpp::SystemState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
