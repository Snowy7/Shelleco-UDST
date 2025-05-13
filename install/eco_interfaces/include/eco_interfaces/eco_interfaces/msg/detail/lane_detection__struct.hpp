// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eco_interfaces:msg/LaneDetection.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__STRUCT_HPP_
#define ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__eco_interfaces__msg__LaneDetection __attribute__((deprecated))
#else
# define DEPRECATED__eco_interfaces__msg__LaneDetection __declspec(deprecated)
#endif

namespace eco_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LaneDetection_
{
  using Type = LaneDetection_<ContainerAllocator>;

  explicit LaneDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lane_center_offset = 0.0f;
      this->lane_heading_error = 0.0f;
    }
  }

  explicit LaneDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lane_center_offset = 0.0f;
      this->lane_heading_error = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _left_line_x_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _left_line_x_type left_line_x;
  using _left_line_y_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _left_line_y_type left_line_y;
  using _right_line_x_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _right_line_x_type right_line_x;
  using _right_line_y_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _right_line_y_type right_line_y;
  using _lane_center_offset_type =
    float;
  _lane_center_offset_type lane_center_offset;
  using _lane_heading_error_type =
    float;
  _lane_heading_error_type lane_heading_error;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__left_line_x(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->left_line_x = _arg;
    return *this;
  }
  Type & set__left_line_y(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->left_line_y = _arg;
    return *this;
  }
  Type & set__right_line_x(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->right_line_x = _arg;
    return *this;
  }
  Type & set__right_line_y(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->right_line_y = _arg;
    return *this;
  }
  Type & set__lane_center_offset(
    const float & _arg)
  {
    this->lane_center_offset = _arg;
    return *this;
  }
  Type & set__lane_heading_error(
    const float & _arg)
  {
    this->lane_heading_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eco_interfaces::msg::LaneDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const eco_interfaces::msg::LaneDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eco_interfaces::msg::LaneDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eco_interfaces::msg::LaneDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::LaneDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::LaneDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::LaneDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::LaneDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eco_interfaces::msg::LaneDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eco_interfaces::msg::LaneDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eco_interfaces__msg__LaneDetection
    std::shared_ptr<eco_interfaces::msg::LaneDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eco_interfaces__msg__LaneDetection
    std::shared_ptr<eco_interfaces::msg::LaneDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaneDetection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->left_line_x != other.left_line_x) {
      return false;
    }
    if (this->left_line_y != other.left_line_y) {
      return false;
    }
    if (this->right_line_x != other.right_line_x) {
      return false;
    }
    if (this->right_line_y != other.right_line_y) {
      return false;
    }
    if (this->lane_center_offset != other.lane_center_offset) {
      return false;
    }
    if (this->lane_heading_error != other.lane_heading_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaneDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaneDetection_

// alias to use template instance with default allocator
using LaneDetection =
  eco_interfaces::msg::LaneDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__LANE_DETECTION__STRUCT_HPP_
