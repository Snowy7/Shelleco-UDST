// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eco_interfaces:msg/LaneDetection.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__TRAITS_HPP_
#define ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "eco_interfaces/msg/detail/lane_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace eco_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const LaneDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: left_line_x
  {
    if (msg.left_line_x.size() == 0) {
      out << "left_line_x: []";
    } else {
      out << "left_line_x: [";
      size_t pending_items = msg.left_line_x.size();
      for (auto item : msg.left_line_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: left_line_y
  {
    if (msg.left_line_y.size() == 0) {
      out << "left_line_y: []";
    } else {
      out << "left_line_y: [";
      size_t pending_items = msg.left_line_y.size();
      for (auto item : msg.left_line_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: right_line_x
  {
    if (msg.right_line_x.size() == 0) {
      out << "right_line_x: []";
    } else {
      out << "right_line_x: [";
      size_t pending_items = msg.right_line_x.size();
      for (auto item : msg.right_line_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: right_line_y
  {
    if (msg.right_line_y.size() == 0) {
      out << "right_line_y: []";
    } else {
      out << "right_line_y: [";
      size_t pending_items = msg.right_line_y.size();
      for (auto item : msg.right_line_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: lane_center_offset
  {
    out << "lane_center_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.lane_center_offset, out);
    out << ", ";
  }

  // member: lane_heading_error
  {
    out << "lane_heading_error: ";
    rosidl_generator_traits::value_to_yaml(msg.lane_heading_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaneDetection & msg,
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

  // member: left_line_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.left_line_x.size() == 0) {
      out << "left_line_x: []\n";
    } else {
      out << "left_line_x:\n";
      for (auto item : msg.left_line_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: left_line_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.left_line_y.size() == 0) {
      out << "left_line_y: []\n";
    } else {
      out << "left_line_y:\n";
      for (auto item : msg.left_line_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: right_line_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.right_line_x.size() == 0) {
      out << "right_line_x: []\n";
    } else {
      out << "right_line_x:\n";
      for (auto item : msg.right_line_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: right_line_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.right_line_y.size() == 0) {
      out << "right_line_y: []\n";
    } else {
      out << "right_line_y:\n";
      for (auto item : msg.right_line_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: lane_center_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lane_center_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.lane_center_offset, out);
    out << "\n";
  }

  // member: lane_heading_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lane_heading_error: ";
    rosidl_generator_traits::value_to_yaml(msg.lane_heading_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaneDetection & msg, bool use_flow_style = false)
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
  const eco_interfaces::msg::LaneDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  eco_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use eco_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const eco_interfaces::msg::LaneDetection & msg)
{
  return eco_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<eco_interfaces::msg::LaneDetection>()
{
  return "eco_interfaces::msg::LaneDetection";
}

template<>
inline const char * name<eco_interfaces::msg::LaneDetection>()
{
  return "eco_interfaces/msg/LaneDetection";
}

template<>
struct has_fixed_size<eco_interfaces::msg::LaneDetection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<eco_interfaces::msg::LaneDetection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<eco_interfaces::msg::LaneDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__TRAITS_HPP_
