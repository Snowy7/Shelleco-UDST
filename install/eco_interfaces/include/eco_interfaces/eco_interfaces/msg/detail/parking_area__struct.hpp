// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eco_interfaces:msg/ParkingArea.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__STRUCT_HPP_
#define ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__STRUCT_HPP_

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
// Member 'parking_spot'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__eco_interfaces__msg__ParkingArea __attribute__((deprecated))
#else
# define DEPRECATED__eco_interfaces__msg__ParkingArea __declspec(deprecated)
#endif

namespace eco_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ParkingArea_
{
  using Type = ParkingArea_<ContainerAllocator>;

  explicit ParkingArea_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    parking_spot(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detected = false;
    }
  }

  explicit ParkingArea_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    parking_spot(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detected = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detected_type =
    bool;
  _detected_type detected;
  using _parking_spot_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _parking_spot_type parking_spot;

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
  Type & set__parking_spot(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->parking_spot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eco_interfaces::msg::ParkingArea_<ContainerAllocator> *;
  using ConstRawPtr =
    const eco_interfaces::msg::ParkingArea_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eco_interfaces::msg::ParkingArea_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eco_interfaces::msg::ParkingArea_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::ParkingArea_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::ParkingArea_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::ParkingArea_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::ParkingArea_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eco_interfaces::msg::ParkingArea_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eco_interfaces::msg::ParkingArea_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eco_interfaces__msg__ParkingArea
    std::shared_ptr<eco_interfaces::msg::ParkingArea_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eco_interfaces__msg__ParkingArea
    std::shared_ptr<eco_interfaces::msg::ParkingArea_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParkingArea_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detected != other.detected) {
      return false;
    }
    if (this->parking_spot != other.parking_spot) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParkingArea_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParkingArea_

// alias to use template instance with default allocator
using ParkingArea =
  eco_interfaces::msg::ParkingArea_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__PARKING_AREA__STRUCT_HPP_
