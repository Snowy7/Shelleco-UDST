// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eco_interfaces:msg/ParkingArea.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__TRAITS_HPP_
#define ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "eco_interfaces/msg/detail/parking_area__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'parking_spot'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace eco_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ParkingArea & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: detected
  {
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << ", ";
  }

  // member: parking_spot
  {
    out << "parking_spot: ";
    to_flow_style_yaml(msg.parking_spot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParkingArea & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << "\n";
  }

  // member: parking_spot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parking_spot:\n";
    to_block_style_yaml(msg.parking_spot, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParkingArea & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace eco_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use eco_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const eco_interfaces::msg::ParkingArea & msg,
  std::ostream & out, size_t indentation = 0)
{
  eco_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use eco_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const eco_interfaces::msg::ParkingArea & msg)
{
  return eco_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<eco_interfaces::msg::ParkingArea>()
{
  return "eco_interfaces::msg::ParkingArea";
}

template<>
inline const char * name<eco_interfaces::msg::ParkingArea>()
{
  return "eco_interfaces/msg/ParkingArea";
}

template<>
struct has_fixed_size<eco_interfaces::msg::ParkingArea>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<eco_interfaces::msg::ParkingArea>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<eco_interfaces::msg::ParkingArea>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__TRAITS_HPP_
