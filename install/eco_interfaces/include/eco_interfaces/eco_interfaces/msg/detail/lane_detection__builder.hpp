// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eco_interfaces:msg/LaneDetection.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__BUILDER_HPP_
#define ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eco_interfaces/msg/detail/lane_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eco_interfaces
{

namespace msg
{

namespace builder
{

class Init_LaneDetection_lane_heading_error
{
public:
  explicit Init_LaneDetection_lane_heading_error(::eco_interfaces::msg::LaneDetection & msg)
  : msg_(msg)
  {}
  ::eco_interfaces::msg::LaneDetection lane_heading_error(::eco_interfaces::msg::LaneDetection::_lane_heading_error_type arg)
  {
    msg_.lane_heading_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eco_interfaces::msg::LaneDetection msg_;
};

class Init_LaneDetection_lane_center_offset
{
public:
  explicit Init_LaneDetection_lane_center_offset(::eco_interfaces::msg::LaneDetection & msg)
  : msg_(msg)
  {}
  Init_LaneDetection_lane_heading_error lane_center_offset(::eco_interfaces::msg::LaneDetection::_lane_center_offset_type arg)
  {
    msg_.lane_center_offset = std::move(arg);
    return Init_LaneDetection_lane_heading_error(msg_);
  }

private:
  ::eco_interfaces::msg::LaneDetection msg_;
};

class Init_LaneDetection_right_line_y
{
public:
  explicit Init_LaneDetection_right_line_y(::eco_interfaces::msg::LaneDetection & msg)
  : msg_(msg)
  {}
  Init_LaneDetection_lane_center_offset right_line_y(::eco_interfaces::msg::LaneDetection::_right_line_y_type arg)
  {
    msg_.right_line_y = std::move(arg);
    return Init_LaneDetection_lane_center_offset(msg_);
  }

private:
  ::eco_interfaces::msg::LaneDetection msg_;
};

class Init_LaneDetection_right_line_x
{
public:
  explicit Init_LaneDetection_right_line_x(::eco_interfaces::msg::LaneDetection & msg)
  : msg_(msg)
  {}
  Init_LaneDetection_right_line_y right_line_x(::eco_interfaces::msg::LaneDetection::_right_line_x_type arg)
  {
    msg_.right_line_x = std::move(arg);
    return Init_LaneDetection_right_line_y(msg_);
  }

private:
  ::eco_interfaces::msg::LaneDetection msg_;
};

class Init_LaneDetection_left_line_y
{
public:
  explicit Init_LaneDetection_left_line_y(::eco_interfaces::msg::LaneDetection & msg)
  : msg_(msg)
  {}
  Init_LaneDetection_right_line_x left_line_y(::eco_interfaces::msg::LaneDetection::_left_line_y_type arg)
  {
    msg_.left_line_y = std::move(arg);
    return Init_LaneDetection_right_line_x(msg_);
  }

private:
  ::eco_interfaces::msg::LaneDetection msg_;
};

class Init_LaneDetection_left_line_x
{
public:
  explicit Init_LaneDetection_left_line_x(::eco_interfaces::msg::LaneDetection & msg)
  : msg_(msg)
  {}
  Init_LaneDetection_left_line_y left_line_x(::eco_interfaces::msg::LaneDetection::_left_line_x_type arg)
  {
    msg_.left_line_x = std::move(arg);
    return Init_LaneDetection_left_line_y(msg_);
  }

private:
  ::eco_interfaces::msg::LaneDetection msg_;
};

class Init_LaneDetection_header
{
public:
  Init_LaneDetection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaneDetection_left_line_x header(::eco_interfaces::msg::LaneDetection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LaneDetection_left_line_x(msg_);
  }

private:
  ::eco_interfaces::msg::LaneDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eco_interfaces::msg::LaneDetection>()
{
  return eco_interfaces::msg::builder::Init_LaneDetection_header();
}

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__BUILDER_HPP_
