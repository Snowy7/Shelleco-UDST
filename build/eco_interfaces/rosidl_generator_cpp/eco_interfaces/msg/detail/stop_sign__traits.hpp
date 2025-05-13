// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eco_interfaces:msg/StopSign.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__TRAITS_HPP_
#define ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "eco_interfaces/msg/detail/stop_sign__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace eco_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const StopSign & msg,
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

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StopSign & msg,
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

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StopSign & msg, bool use_flow_style = false)
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
  const eco_interfaces::msg::StopSign & msg,
  std::ostream & out, size_t indentation = 0)
{
  eco_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use eco_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const eco_interfaces::msg::StopSign & msg)
{
  return eco_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<eco_interfaces::msg::StopSign>()
{
  return "eco_interfaces::msg::StopSign";
}

template<>
inline const char * name<eco_interfaces::msg::StopSign>()
{
  return "eco_interfaces/msg/StopSign";
}

template<>
struct has_fixed_size<eco_interfaces::msg::StopSign>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<eco_interfaces::msg::StopSign>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<eco_interfaces::msg::StopSign>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__TRAITS_HPP_
