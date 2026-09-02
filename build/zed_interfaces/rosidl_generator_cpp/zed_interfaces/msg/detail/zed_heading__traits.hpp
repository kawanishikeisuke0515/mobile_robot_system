// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:msg/ZedHeading.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zed_interfaces/msg/zed_heading.hpp"


#ifndef ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__TRAITS_HPP_
#define ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zed_interfaces/msg/detail/zed_heading__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace zed_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ZedHeading & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: raw_x
  {
    out << "raw_x: ";
    rosidl_generator_traits::value_to_yaml(msg.raw_x, out);
    out << ", ";
  }

  // member: raw_z
  {
    out << "raw_z: ";
    rosidl_generator_traits::value_to_yaml(msg.raw_z, out);
    out << ", ";
  }

  // member: corrected_x
  {
    out << "corrected_x: ";
    rosidl_generator_traits::value_to_yaml(msg.corrected_x, out);
    out << ", ";
  }

  // member: corrected_z
  {
    out << "corrected_z: ";
    rosidl_generator_traits::value_to_yaml(msg.corrected_z, out);
    out << ", ";
  }

  // member: magnetic_heading_deg
  {
    out << "magnetic_heading_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetic_heading_deg, out);
    out << ", ";
  }

  // member: robot_yaw_deg
  {
    out << "robot_yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_yaw_deg, out);
    out << ", ";
  }

  // member: robot_yaw_rad
  {
    out << "robot_yaw_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_yaw_rad, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ZedHeading & msg,
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

  // member: raw_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "raw_x: ";
    rosidl_generator_traits::value_to_yaml(msg.raw_x, out);
    out << "\n";
  }

  // member: raw_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "raw_z: ";
    rosidl_generator_traits::value_to_yaml(msg.raw_z, out);
    out << "\n";
  }

  // member: corrected_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "corrected_x: ";
    rosidl_generator_traits::value_to_yaml(msg.corrected_x, out);
    out << "\n";
  }

  // member: corrected_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "corrected_z: ";
    rosidl_generator_traits::value_to_yaml(msg.corrected_z, out);
    out << "\n";
  }

  // member: magnetic_heading_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetic_heading_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetic_heading_deg, out);
    out << "\n";
  }

  // member: robot_yaw_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_yaw_deg, out);
    out << "\n";
  }

  // member: robot_yaw_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_yaw_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_yaw_rad, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ZedHeading & msg, bool use_flow_style = false)
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

}  // namespace zed_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zed_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zed_interfaces::msg::ZedHeading & msg,
  std::ostream & out, size_t indentation = 0)
{
  zed_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zed_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const zed_interfaces::msg::ZedHeading & msg)
{
  return zed_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<zed_interfaces::msg::ZedHeading>()
{
  return "zed_interfaces::msg::ZedHeading";
}

template<>
inline const char * name<zed_interfaces::msg::ZedHeading>()
{
  return "zed_interfaces/msg/ZedHeading";
}

template<>
struct has_fixed_size<zed_interfaces::msg::ZedHeading>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<zed_interfaces::msg::ZedHeading>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<zed_interfaces::msg::ZedHeading>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__TRAITS_HPP_
