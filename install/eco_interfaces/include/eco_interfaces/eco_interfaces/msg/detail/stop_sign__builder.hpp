// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eco_interfaces:msg/StopSign.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__BUILDER_HPP_
#define ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eco_interfaces/msg/detail/stop_sign__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eco_interfaces
{

namespace msg
{

namespace builder
{

class Init_StopSign_position
{
public:
  explicit Init_StopSign_position(::eco_interfaces::msg::StopSign & msg)
  : msg_(msg)
  {}
  ::eco_interfaces::msg::StopSign position(::eco_interfaces::msg::StopSign::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eco_interfaces::msg::StopSign msg_;
};

class Init_StopSign_distance
{
public:
  explicit Init_StopSign_distance(::eco_interfaces::msg::StopSign & msg)
  : msg_(msg)
  {}
  Init_StopSign_position distance(::eco_interfaces::msg::StopSign::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_StopSign_position(msg_);
  }

private:
  ::eco_interfaces::msg::StopSign msg_;
};

class Init_StopSign_detected
{
public:
  explicit Init_StopSign_detected(::eco_interfaces::msg::StopSign & msg)
  : msg_(msg)
  {}
  Init_StopSign_distance detected(::eco_interfaces::msg::StopSign::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return Init_StopSign_distance(msg_);
  }

private:
  ::eco_interfaces::msg::StopSign msg_;
};

class Init_StopSign_header
{
public:
  Init_StopSign_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StopSign_detected header(::eco_interfaces::msg::StopSign::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StopSign_detected(msg_);
  }

private:
  ::eco_interfaces::msg::StopSign msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eco_interfaces::msg::StopSign>()
{
  return eco_interfaces::msg::builder::Init_StopSign_header();
}

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__BUILDER_HPP_
