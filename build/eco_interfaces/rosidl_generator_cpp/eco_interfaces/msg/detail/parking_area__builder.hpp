// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eco_interfaces:msg/ParkingArea.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__BUILDER_HPP_
#define ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eco_interfaces/msg/detail/parking_area__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eco_interfaces
{

namespace msg
{

namespace builder
{

class Init_ParkingArea_parking_spot
{
public:
  explicit Init_ParkingArea_parking_spot(::eco_interfaces::msg::ParkingArea & msg)
  : msg_(msg)
  {}
  ::eco_interfaces::msg::ParkingArea parking_spot(::eco_interfaces::msg::ParkingArea::_parking_spot_type arg)
  {
    msg_.parking_spot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eco_interfaces::msg::ParkingArea msg_;
};

class Init_ParkingArea_detected
{
public:
  explicit Init_ParkingArea_detected(::eco_interfaces::msg::ParkingArea & msg)
  : msg_(msg)
  {}
  Init_ParkingArea_parking_spot detected(::eco_interfaces::msg::ParkingArea::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return Init_ParkingArea_parking_spot(msg_);
  }

private:
  ::eco_interfaces::msg::ParkingArea msg_;
};

class Init_ParkingArea_header
{
public:
  Init_ParkingArea_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParkingArea_detected header(::eco_interfaces::msg::ParkingArea::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ParkingArea_detected(msg_);
  }

private:
  ::eco_interfaces::msg::ParkingArea msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eco_interfaces::msg::ParkingArea>()
{
  return eco_interfaces::msg::builder::Init_ParkingArea_header();
}

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__BUILDER_HPP_
