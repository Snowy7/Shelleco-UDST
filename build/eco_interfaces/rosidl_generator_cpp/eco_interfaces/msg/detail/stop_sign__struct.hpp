// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eco_interfaces:msg/StopSign.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__STRUCT_HPP_
#define ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__eco_interfaces__msg__StopSign __attribute__((deprecated))
#else
# define DEPRECATED__eco_interfaces__msg__StopSign __declspec(deprecated)
#endif

namespace eco_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StopSign_
{
  using Type = StopSign_<ContainerAllocator>;

  explicit StopSign_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detected = false;
      this->distance = 0.0f;
    }
  }

  explicit StopSign_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detected = false;
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detected_type =
    bool;
  _detected_type detected;
  using _distance_type =
    float;
  _distance_type distance;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detected(
    const bool & _arg)
  {
    this->detected = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eco_interfaces::msg::StopSign_<ContainerAllocator> *;
  using ConstRawPtr =
    const eco_interfaces::msg::StopSign_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eco_interfaces::msg::StopSign_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eco_interfaces::msg::StopSign_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::StopSign_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::StopSign_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::StopSign_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::StopSign_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eco_interfaces::msg::StopSign_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eco_interfaces::msg::StopSign_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eco_interfaces__msg__StopSign
    std::shared_ptr<eco_interfaces::msg::StopSign_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eco_interfaces__msg__StopSign
    std::shared_ptr<eco_interfaces::msg::StopSign_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StopSign_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detected != other.detected) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const StopSign_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StopSign_

// alias to use template instance with default allocator
using StopSign =
  eco_interfaces::msg::StopSign_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__STOP_SIGN__STRUCT_HPP_
