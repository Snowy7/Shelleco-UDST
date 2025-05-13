// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eco_interfaces:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__OBSTACLE__STRUCT_HPP_
#define ECO_INTERFACES__MSG__DETAIL__OBSTACLE__STRUCT_HPP_

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
// Member 'points'
// Member 'center'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__eco_interfaces__msg__Obstacle __attribute__((deprecated))
#else
# define DEPRECATED__eco_interfaces__msg__Obstacle __declspec(deprecated)
#endif

namespace eco_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Obstacle_
{
  using Type = Obstacle_<ContainerAllocator>;

  explicit Obstacle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    center(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->radius = 0.0f;
    }
  }

  explicit Obstacle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    center(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->radius = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;
  using _center_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _center_type center;
  using _radius_type =
    float;
  _radius_type radius;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__center(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->center = _arg;
    return *this;
  }
  Type & set__radius(
    const float & _arg)
  {
    this->radius = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eco_interfaces::msg::Obstacle_<ContainerAllocator> *;
  using ConstRawPtr =
    const eco_interfaces::msg::Obstacle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eco_interfaces::msg::Obstacle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eco_interfaces::msg::Obstacle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::Obstacle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::Obstacle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::Obstacle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::Obstacle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eco_interfaces::msg::Obstacle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eco_interfaces::msg::Obstacle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eco_interfaces__msg__Obstacle
    std::shared_ptr<eco_interfaces::msg::Obstacle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eco_interfaces__msg__Obstacle
    std::shared_ptr<eco_interfaces::msg::Obstacle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Obstacle_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->center != other.center) {
      return false;
    }
    if (this->radius != other.radius) {
      return false;
    }
    return true;
  }
  bool operator!=(const Obstacle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Obstacle_

// alias to use template instance with default allocator
using Obstacle =
  eco_interfaces::msg::Obstacle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__OBSTACLE__STRUCT_HPP_
