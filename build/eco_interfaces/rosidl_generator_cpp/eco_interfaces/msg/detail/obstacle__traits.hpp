// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eco_interfaces:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__OBSTACLE__TRAITS_HPP_
#define ECO_INTERFACES__MSG__DETAIL__OBSTACLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "eco_interfaces/msg/detail/obstacle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'points'
// Member 'center'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace eco_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Obstacle & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: center
  {
    out << "center: ";
    to_flow_style_yaml(msg.center, out);
    out << ", ";
  }

  // member: radius
  {
    out << "radius: ";
    rosidl_generator_traits::value_to_yaml(msg.radius, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Obstacle & msg,
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

  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center:\n";
    to_block_style_yaml(msg.center, out, indentation + 2);
  }

  // member: radius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radius: ";
    rosidl_generator_traits::value_to_yaml(msg.radius, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Obstacle & msg, bool use_flow_style = false)
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
  const eco_interfaces::msg::Obstacle & msg,
  std::ostream & out, size_t indentation = 0)
{
  eco_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use eco_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const eco_interfaces::msg::Obstacle & msg)
{
  return eco_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<eco_interfaces::msg::Obstacle>()
{
  return "eco_interfaces::msg::Obstacle";
}

template<>
inline const char * name<eco_interfaces::msg::Obstacle>()
{
  return "eco_interfaces/msg/Obstacle";
}

template<>
struct has_fixed_size<eco_interfaces::msg::Obstacle>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<eco_interfaces::msg::Obstacle>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<eco_interfaces::msg::Obstacle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ECO_INTERFACES__MSG__DETAIL__OBSTACLE__TRAITS_HPP_
