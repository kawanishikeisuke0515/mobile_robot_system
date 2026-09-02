// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zed_interfaces:msg/ZedHeading.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/zed_heading__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
zed_interfaces__msg__ZedHeading__init(zed_interfaces__msg__ZedHeading * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    zed_interfaces__msg__ZedHeading__fini(msg);
    return false;
  }
  // raw_x
  // raw_z
  // corrected_x
  // corrected_z
  // magnetic_heading_deg
  // robot_yaw_deg
  // robot_yaw_rad
  // valid
  return true;
}

void
zed_interfaces__msg__ZedHeading__fini(zed_interfaces__msg__ZedHeading * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // raw_x
  // raw_z
  // corrected_x
  // corrected_z
  // magnetic_heading_deg
  // robot_yaw_deg
  // robot_yaw_rad
  // valid
}

bool
zed_interfaces__msg__ZedHeading__are_equal(const zed_interfaces__msg__ZedHeading * lhs, const zed_interfaces__msg__ZedHeading * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // raw_x
  if (lhs->raw_x != rhs->raw_x) {
    return false;
  }
  // raw_z
  if (lhs->raw_z != rhs->raw_z) {
    return false;
  }
  // corrected_x
  if (lhs->corrected_x != rhs->corrected_x) {
    return false;
  }
  // corrected_z
  if (lhs->corrected_z != rhs->corrected_z) {
    return false;
  }
  // magnetic_heading_deg
  if (lhs->magnetic_heading_deg != rhs->magnetic_heading_deg) {
    return false;
  }
  // robot_yaw_deg
  if (lhs->robot_yaw_deg != rhs->robot_yaw_deg) {
    return false;
  }
  // robot_yaw_rad
  if (lhs->robot_yaw_rad != rhs->robot_yaw_rad) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  return true;
}

bool
zed_interfaces__msg__ZedHeading__copy(
  const zed_interfaces__msg__ZedHeading * input,
  zed_interfaces__msg__ZedHeading * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // raw_x
  output->raw_x = input->raw_x;
  // raw_z
  output->raw_z = input->raw_z;
  // corrected_x
  output->corrected_x = input->corrected_x;
  // corrected_z
  output->corrected_z = input->corrected_z;
  // magnetic_heading_deg
  output->magnetic_heading_deg = input->magnetic_heading_deg;
  // robot_yaw_deg
  output->robot_yaw_deg = input->robot_yaw_deg;
  // robot_yaw_rad
  output->robot_yaw_rad = input->robot_yaw_rad;
  // valid
  output->valid = input->valid;
  return true;
}

zed_interfaces__msg__ZedHeading *
zed_interfaces__msg__ZedHeading__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__ZedHeading * msg = (zed_interfaces__msg__ZedHeading *)allocator.allocate(sizeof(zed_interfaces__msg__ZedHeading), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zed_interfaces__msg__ZedHeading));
  bool success = zed_interfaces__msg__ZedHeading__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zed_interfaces__msg__ZedHeading__destroy(zed_interfaces__msg__ZedHeading * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zed_interfaces__msg__ZedHeading__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zed_interfaces__msg__ZedHeading__Sequence__init(zed_interfaces__msg__ZedHeading__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__ZedHeading * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(zed_interfaces__msg__ZedHeading)) {
      return false;
    }
    data = (zed_interfaces__msg__ZedHeading *)allocator.zero_allocate(size, sizeof(zed_interfaces__msg__ZedHeading), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zed_interfaces__msg__ZedHeading__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zed_interfaces__msg__ZedHeading__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
zed_interfaces__msg__ZedHeading__Sequence__fini(zed_interfaces__msg__ZedHeading__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      zed_interfaces__msg__ZedHeading__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

zed_interfaces__msg__ZedHeading__Sequence *
zed_interfaces__msg__ZedHeading__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__ZedHeading__Sequence * array = (zed_interfaces__msg__ZedHeading__Sequence *)allocator.allocate(sizeof(zed_interfaces__msg__ZedHeading__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zed_interfaces__msg__ZedHeading__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zed_interfaces__msg__ZedHeading__Sequence__destroy(zed_interfaces__msg__ZedHeading__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zed_interfaces__msg__ZedHeading__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zed_interfaces__msg__ZedHeading__Sequence__are_equal(const zed_interfaces__msg__ZedHeading__Sequence * lhs, const zed_interfaces__msg__ZedHeading__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zed_interfaces__msg__ZedHeading__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__ZedHeading__Sequence__copy(
  const zed_interfaces__msg__ZedHeading__Sequence * input,
  zed_interfaces__msg__ZedHeading__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(zed_interfaces__msg__ZedHeading)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(zed_interfaces__msg__ZedHeading);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zed_interfaces__msg__ZedHeading * data =
      (zed_interfaces__msg__ZedHeading *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zed_interfaces__msg__ZedHeading__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zed_interfaces__msg__ZedHeading__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zed_interfaces__msg__ZedHeading__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
