// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:msg/ZedHeading.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zed_interfaces/msg/zed_heading.h"


#ifndef ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__STRUCT_H_
#define ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ZedHeading in the package zed_interfaces.
typedef struct zed_interfaces__msg__ZedHeading
{
  std_msgs__msg__Header header;
  float raw_x;
  float raw_z;
  float corrected_x;
  float corrected_z;
  float magnetic_heading_deg;
  float robot_yaw_deg;
  float robot_yaw_rad;
  bool valid;
} zed_interfaces__msg__ZedHeading;

// Struct for a sequence of zed_interfaces__msg__ZedHeading.
typedef struct zed_interfaces__msg__ZedHeading__Sequence
{
  zed_interfaces__msg__ZedHeading * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__msg__ZedHeading__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__MSG__DETAIL__ZED_HEADING__STRUCT_H_
