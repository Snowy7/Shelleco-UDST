// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eco_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_
#define ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eco_interfaces/msg/detail/system_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eco_interfaces
{

namespace msg
{

namespace builder
{

class Init_SystemState_state
{
public:
  explicit Init_SystemState_state(::eco_interfaces::msg::SystemState & msg)
  : msg_(msg)
  {}
  ::eco_interfaces::msg::SystemState state(::eco_interfaces::msg::SystemState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eco_interfaces::msg::SystemState msg_;
};

class Init_SystemState_header
{
public:
  Init_SystemState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SystemState_state header(::eco_interfaces::msg::SystemState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SystemState_state(msg_);
  }

private:
  ::eco_interfaces::msg::SystemState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eco_interfaces::msg::SystemState>()
{
  return eco_interfaces::msg::builder::Init_SystemState_header();
}

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_
