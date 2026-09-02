// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:msg/ZedHeading.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zed_interfaces/msg/zed_heading.hpp"


#ifndef ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__BUILDER_HPP_
#define ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zed_interfaces/msg/detail/zed_heading__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zed_interfaces
{

namespace msg
{

namespace builder
{

class Init_ZedHeading_valid
{
public:
  explicit Init_ZedHeading_valid(::zed_interfaces::msg::ZedHeading & msg)
  : msg_(msg)
  {}
  ::zed_interfaces::msg::ZedHeading valid(::zed_interfaces::msg::ZedHeading::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

class Init_ZedHeading_robot_yaw_rad
{
public:
  explicit Init_ZedHeading_robot_yaw_rad(::zed_interfaces::msg::ZedHeading & msg)
  : msg_(msg)
  {}
  Init_ZedHeading_valid robot_yaw_rad(::zed_interfaces::msg::ZedHeading::_robot_yaw_rad_type arg)
  {
    msg_.robot_yaw_rad = std::move(arg);
    return Init_ZedHeading_valid(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

class Init_ZedHeading_robot_yaw_deg
{
public:
  explicit Init_ZedHeading_robot_yaw_deg(::zed_interfaces::msg::ZedHeading & msg)
  : msg_(msg)
  {}
  Init_ZedHeading_robot_yaw_rad robot_yaw_deg(::zed_interfaces::msg::ZedHeading::_robot_yaw_deg_type arg)
  {
    msg_.robot_yaw_deg = std::move(arg);
    return Init_ZedHeading_robot_yaw_rad(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

class Init_ZedHeading_magnetic_heading_deg
{
public:
  explicit Init_ZedHeading_magnetic_heading_deg(::zed_interfaces::msg::ZedHeading & msg)
  : msg_(msg)
  {}
  Init_ZedHeading_robot_yaw_deg magnetic_heading_deg(::zed_interfaces::msg::ZedHeading::_magnetic_heading_deg_type arg)
  {
    msg_.magnetic_heading_deg = std::move(arg);
    return Init_ZedHeading_robot_yaw_deg(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

class Init_ZedHeading_corrected_z
{
public:
  explicit Init_ZedHeading_corrected_z(::zed_interfaces::msg::ZedHeading & msg)
  : msg_(msg)
  {}
  Init_ZedHeading_magnetic_heading_deg corrected_z(::zed_interfaces::msg::ZedHeading::_corrected_z_type arg)
  {
    msg_.corrected_z = std::move(arg);
    return Init_ZedHeading_magnetic_heading_deg(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

class Init_ZedHeading_corrected_x
{
public:
  explicit Init_ZedHeading_corrected_x(::zed_interfaces::msg::ZedHeading & msg)
  : msg_(msg)
  {}
  Init_ZedHeading_corrected_z corrected_x(::zed_interfaces::msg::ZedHeading::_corrected_x_type arg)
  {
    msg_.corrected_x = std::move(arg);
    return Init_ZedHeading_corrected_z(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

class Init_ZedHeading_raw_z
{
public:
  explicit Init_ZedHeading_raw_z(::zed_interfaces::msg::ZedHeading & msg)
  : msg_(msg)
  {}
  Init_ZedHeading_corrected_x raw_z(::zed_interfaces::msg::ZedHeading::_raw_z_type arg)
  {
    msg_.raw_z = std::move(arg);
    return Init_ZedHeading_corrected_x(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

class Init_ZedHeading_raw_x
{
public:
  explicit Init_ZedHeading_raw_x(::zed_interfaces::msg::ZedHeading & msg)
  : msg_(msg)
  {}
  Init_ZedHeading_raw_z raw_x(::zed_interfaces::msg::ZedHeading::_raw_x_type arg)
  {
    msg_.raw_x = std::move(arg);
    return Init_ZedHeading_raw_z(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

class Init_ZedHeading_header
{
public:
  Init_ZedHeading_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZedHeading_raw_x header(::zed_interfaces::msg::ZedHeading::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ZedHeading_raw_x(msg_);
  }

private:
  ::zed_interfaces::msg::ZedHeading msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::msg::ZedHeading>()
{
  return zed_interfaces::msg::builder::Init_ZedHeading_header();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__BUILDER_HPP_
