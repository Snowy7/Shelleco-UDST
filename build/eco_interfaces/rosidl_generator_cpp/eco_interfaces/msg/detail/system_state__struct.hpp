// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eco_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_HPP_
#define ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_HPP_

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
# define DEPRECATED__eco_interfaces__msg__SystemState __attribute__((deprecated))
#else
# define DEPRECATED__eco_interfaces__msg__SystemState __declspec(deprecated)
#endif

namespace eco_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SystemState_
{
  using Type = SystemState_<ContainerAllocator>;

  explicit SystemState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit SystemState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t IDLE =
    0u;
  static constexpr uint8_t SECTION1_LANE_FOLLOWING =
    1u;
  static constexpr uint8_t APPROACHING_STOP_SIGN_SECTION1 =
    2u;
  static constexpr uint8_t STOPPED_SECTION1 =
    3u;
  static constexpr uint8_t SECTION2_OBSTACLE_AVOIDANCE =
    4u;
  static constexpr uint8_t APPROACHING_STOP_SIGN_SECTION2 =
    5u;
  static constexpr uint8_t STOPPED_SECTION2 =
    6u;
  static constexpr uint8_t SECTION3_PARKING =
    7u;
  static constexpr uint8_t PARKED =
    8u;
  static constexpr uint8_t EMERGENCY_STOP =
    9u;

  // pointer types
  using RawPtr =
    eco_interfaces::msg::SystemState_<ContainerAllocator> *;
  using ConstRawPtr =
    const eco_interfaces::msg::SystemState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eco_interfaces::msg::SystemState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eco_interfaces::msg::SystemState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::SystemState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::SystemState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eco_interfaces::msg::SystemState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eco_interfaces::msg::SystemState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eco_interfaces::msg::SystemState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eco_interfaces::msg::SystemState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eco_interfaces__msg__SystemState
    std::shared_ptr<eco_interfaces::msg::SystemState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eco_interfaces__msg__SystemState
    std::shared_ptr<eco_interfaces::msg::SystemState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SystemState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SystemState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SystemState_

// alias to use template instance with default allocator
using SystemState =
  eco_interfaces::msg::SystemState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::IDLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::SECTION1_LANE_FOLLOWING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::APPROACHING_STOP_SIGN_SECTION1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::STOPPED_SECTION1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::SECTION2_OBSTACLE_AVOIDANCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::APPROACHING_STOP_SIGN_SECTION2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::STOPPED_SECTION2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::SECTION3_PARKING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::PARKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemState_<ContainerAllocator>::EMERGENCY_STOP;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace eco_interfaces

#endif  // ECO_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_HPP_
