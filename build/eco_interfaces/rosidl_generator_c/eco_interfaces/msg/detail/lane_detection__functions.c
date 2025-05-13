// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eco_interfaces:msg/LaneDetection.idl
// generated code does not contain a copyright notice
#include "eco_interfaces/msg/detail/lane_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `left_line_x`
// Member `left_line_y`
// Member `right_line_x`
// Member `right_line_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
eco_interfaces__msg__LaneDetection__init(eco_interfaces__msg__LaneDetection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    eco_interfaces__msg__LaneDetection__fini(msg);
    return false;
  }
  // left_line_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->left_line_x, 0)) {
    eco_interfaces__msg__LaneDetection__fini(msg);
    return false;
  }
  // left_line_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->left_line_y, 0)) {
    eco_interfaces__msg__LaneDetection__fini(msg);
    return false;
  }
  // right_line_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->right_line_x, 0)) {
    eco_interfaces__msg__LaneDetection__fini(msg);
    return false;
  }
  // right_line_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->right_line_y, 0)) {
    eco_interfaces__msg__LaneDetection__fini(msg);
    return false;
  }
  // lane_center_offset
  // lane_heading_error
  return true;
}

void
eco_interfaces__msg__LaneDetection__fini(eco_interfaces__msg__LaneDetection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // left_line_x
  rosidl_runtime_c__float__Sequence__fini(&msg->left_line_x);
  // left_line_y
  rosidl_runtime_c__float__Sequence__fini(&msg->left_line_y);
  // right_line_x
  rosidl_runtime_c__float__Sequence__fini(&msg->right_line_x);
  // right_line_y
  rosidl_runtime_c__float__Sequence__fini(&msg->right_line_y);
  // lane_center_offset
  // lane_heading_error
}

bool
eco_interfaces__msg__LaneDetection__are_equal(const eco_interfaces__msg__LaneDetection * lhs, const eco_interfaces__msg__LaneDetection * rhs)
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
  // left_line_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->left_line_x), &(rhs->left_line_x)))
  {
    return false;
  }
  // left_line_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->left_line_y), &(rhs->left_line_y)))
  {
    return false;
  }
  // right_line_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->right_line_x), &(rhs->right_line_x)))
  {
    return false;
  }
  // right_line_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->right_line_y), &(rhs->right_line_y)))
  {
    return false;
  }
  // lane_center_offset
  if (lhs->lane_center_offset != rhs->lane_center_offset) {
    return false;
  }
  // lane_heading_error
  if (lhs->lane_heading_error != rhs->lane_heading_error) {
    return false;
  }
  return true;
}

bool
eco_interfaces__msg__LaneDetection__copy(
  const eco_interfaces__msg__LaneDetection * input,
  eco_interfaces__msg__LaneDetection * output)
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
  // left_line_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->left_line_x), &(output->left_line_x)))
  {
    return false;
  }
  // left_line_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->left_line_y), &(output->left_line_y)))
  {
    return false;
  }
  // right_line_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->right_line_x), &(output->right_line_x)))
  {
    return false;
  }
  // right_line_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->right_line_y), &(output->right_line_y)))
  {
    return false;
  }
  // lane_center_offset
  output->lane_center_offset = input->lane_center_offset;
  // lane_heading_error
  output->lane_heading_error = input->lane_heading_error;
  return true;
}

eco_interfaces__msg__LaneDetection *
eco_interfaces__msg__LaneDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__LaneDetection * msg = (eco_interfaces__msg__LaneDetection *)allocator.allocate(sizeof(eco_interfaces__msg__LaneDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eco_interfaces__msg__LaneDetection));
  bool success = eco_interfaces__msg__LaneDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
eco_interfaces__msg__LaneDetection__destroy(eco_interfaces__msg__LaneDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    eco_interfaces__msg__LaneDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
eco_interfaces__msg__LaneDetection__Sequence__init(eco_interfaces__msg__LaneDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__LaneDetection * data = NULL;

  if (size) {
    data = (eco_interfaces__msg__LaneDetection *)allocator.zero_allocate(size, sizeof(eco_interfaces__msg__LaneDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eco_interfaces__msg__LaneDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eco_interfaces__msg__LaneDetection__fini(&data[i - 1]);
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
eco_interfaces__msg__LaneDetection__Sequence__fini(eco_interfaces__msg__LaneDetection__Sequence * array)
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
      eco_interfaces__msg__LaneDetection__fini(&array->data[i]);
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

eco_interfaces__msg__LaneDetection__Sequence *
eco_interfaces__msg__LaneDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__LaneDetection__Sequence * array = (eco_interfaces__msg__LaneDetection__Sequence *)allocator.allocate(sizeof(eco_interfaces__msg__LaneDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = eco_interfaces__msg__LaneDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
eco_interfaces__msg__LaneDetection__Sequence__destroy(eco_interfaces__msg__LaneDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    eco_interfaces__msg__LaneDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
eco_interfaces__msg__LaneDetection__Sequence__are_equal(const eco_interfaces__msg__LaneDetection__Sequence * lhs, const eco_interfaces__msg__LaneDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eco_interfaces__msg__LaneDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eco_interfaces__msg__LaneDetection__Sequence__copy(
  const eco_interfaces__msg__LaneDetection__Sequence * input,
  eco_interfaces__msg__LaneDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eco_interfaces__msg__LaneDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    eco_interfaces__msg__LaneDetection * data =
      (eco_interfaces__msg__LaneDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eco_interfaces__msg__LaneDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          eco_interfaces__msg__LaneDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!eco_interfaces__msg__LaneDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
