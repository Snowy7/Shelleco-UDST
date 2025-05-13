// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eco_interfaces:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
#define ECO_INTERFACES__MSG__DETAIL__OBSTACLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eco_interfaces/msg/detail/obstacle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eco_interfaces
{

namespace msg
{

namespace builder
{

class Init_Obstacle_radius
{
public:
  explicit Init_Obstacle_radius(::eco_interfaces::msg::Obstacle & msg)
  : msg_(msg)
  {}
  ::eco_interfaces::msg::Obstacle radius(::eco_interfaces::msg::Obstacle::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eco_interfaces::msg::Obstacle msg_;
};

class Init_Obstacle_center
{
public:
  explicit Init_Obstacle_center(::eco_interfaces::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_radius center(::eco_interfaces::msg::Obstacle::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_Obstacle_radius(msg_);
  }

private:
  ::eco_interfaces::msg::Obstacle msg_;
};

class Init_Obstacle_points
{
public:
  explicit Init_Obstacle_points(::eco_interfaces::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_center points(::eco_interfaces::msg::Obstacle::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_Obstacle_center(msg_);
  }

private:
  ::eco_interfaces::msg::Obstacle msg_;
};

class Init_Obstacle_header
{
public:
  Init_Obstacle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Obstacle_points header(::eco_interfaces::msg::Obstacle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Obstacle_points(msg_);
  }

private:
  ::eco_interfaces::msg::Obstacle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eco_interfaces::msg::Obstacle>()
{
  return eco_interfaces::msg::builder::Init_Obstacle_header();
}

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
