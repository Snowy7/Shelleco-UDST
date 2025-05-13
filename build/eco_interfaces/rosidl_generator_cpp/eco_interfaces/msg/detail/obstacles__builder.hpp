// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eco_interfaces:msg/Obstacles.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__OBSTACLES__BUILDER_HPP_
#define ECO_INTERFACES__MSG__DETAIL__OBSTACLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eco_interfaces/msg/detail/obstacles__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eco_interfaces
{

namespace msg
{

namespace builder
{

class Init_Obstacles_obstacles
{
public:
  explicit Init_Obstacles_obstacles(::eco_interfaces::msg::Obstacles & msg)
  : msg_(msg)
  {}
  ::eco_interfaces::msg::Obstacles obstacles(::eco_interfaces::msg::Obstacles::_obstacles_type arg)
  {
    msg_.obstacles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eco_interfaces::msg::Obstacles msg_;
};

class Init_Obstacles_header
{
public:
  Init_Obstacles_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Obstacles_obstacles header(::eco_interfaces::msg::Obstacles::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Obstacles_obstacles(msg_);
  }

private:
  ::eco_interfaces::msg::Obstacles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eco_interfaces::msg::Obstacles>()
{
  return eco_interfaces::msg::builder::Init_Obstacles_header();
}

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__OBSTACLES__BUILDER_HPP_
