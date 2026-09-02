// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/ZedHeading.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zed_interfaces/msg/zed_heading.hpp"


#ifndef ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__ZedHeading __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__ZedHeading __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ZedHeading_
{
  using Type = ZedHeading_<ContainerAllocator>;

  explicit ZedHeading_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->raw_x = 0.0f;
      this->raw_z = 0.0f;
      this->corrected_x = 0.0f;
      this->corrected_z = 0.0f;
      this->magnetic_heading_deg = 0.0f;
      this->robot_yaw_deg = 0.0f;
      this->robot_yaw_rad = 0.0f;
      this->valid = false;
    }
  }

  explicit ZedHeading_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->raw_x = 0.0f;
      this->raw_z = 0.0f;
      this->corrected_x = 0.0f;
      this->corrected_z = 0.0f;
      this->magnetic_heading_deg = 0.0f;
      this->robot_yaw_deg = 0.0f;
      this->robot_yaw_rad = 0.0f;
      this->valid = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _raw_x_type =
    float;
  _raw_x_type raw_x;
  using _raw_z_type =
    float;
  _raw_z_type raw_z;
  using _corrected_x_type =
    float;
  _corrected_x_type corrected_x;
  using _corrected_z_type =
    float;
  _corrected_z_type corrected_z;
  using _magnetic_heading_deg_type =
    float;
  _magnetic_heading_deg_type magnetic_heading_deg;
  using _robot_yaw_deg_type =
    float;
  _robot_yaw_deg_type robot_yaw_deg;
  using _robot_yaw_rad_type =
    float;
  _robot_yaw_rad_type robot_yaw_rad;
  using _valid_type =
    bool;
  _valid_type valid;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__raw_x(
    const float & _arg)
  {
    this->raw_x = _arg;
    return *this;
  }
  Type & set__raw_z(
    const float & _arg)
  {
    this->raw_z = _arg;
    return *this;
  }
  Type & set__corrected_x(
    const float & _arg)
  {
    this->corrected_x = _arg;
    return *this;
  }
  Type & set__corrected_z(
    const float & _arg)
  {
    this->corrected_z = _arg;
    return *this;
  }
  Type & set__magnetic_heading_deg(
    const float & _arg)
  {
    this->magnetic_heading_deg = _arg;
    return *this;
  }
  Type & set__robot_yaw_deg(
    const float & _arg)
  {
    this->robot_yaw_deg = _arg;
    return *this;
  }
  Type & set__robot_yaw_rad(
    const float & _arg)
  {
    this->robot_yaw_rad = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::msg::ZedHeading_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::ZedHeading_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::ZedHeading_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::ZedHeading_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::ZedHeading_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::ZedHeading_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::ZedHeading_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::ZedHeading_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::ZedHeading_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::ZedHeading_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__ZedHeading
    std::shared_ptr<zed_interfaces::msg::ZedHeading_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__ZedHeading
    std::shared_ptr<zed_interfaces::msg::ZedHeading_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZedHeading_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->raw_x != other.raw_x) {
      return false;
    }
    if (this->raw_z != other.raw_z) {
      return false;
    }
    if (this->corrected_x != other.corrected_x) {
      return false;
    }
    if (this->corrected_z != other.corrected_z) {
      return false;
    }
    if (this->magnetic_heading_deg != other.magnetic_heading_deg) {
      return false;
    }
    if (this->robot_yaw_deg != other.robot_yaw_deg) {
      return false;
    }
    if (this->robot_yaw_rad != other.robot_yaw_rad) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZedHeading_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZedHeading_

// alias to use template instance with default allocator
using ZedHeading =
  zed_interfaces::msg::ZedHeading_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__STRUCT_HPP_
