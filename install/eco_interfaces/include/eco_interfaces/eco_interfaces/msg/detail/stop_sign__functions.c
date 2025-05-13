// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eco_interfaces:msg/StopSign.idl
// generated code does not contain a copyright notice
#include "eco_interfaces/msg/detail/stop_sign__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
eco_interfaces__msg__StopSign__init(eco_interfaces__msg__StopSign * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    eco_interfaces__msg__StopSign__fini(msg);
    return false;
  }
  // detected
  // distance
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    eco_interfaces__msg__StopSign__fini(msg);
    return false;
  }
  return true;
}

void
eco_interfaces__msg__StopSign__fini(eco_interfaces__msg__StopSign * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detected
  // distance
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
}

bool
eco_interfaces__msg__StopSign__are_equal(const eco_interfaces__msg__StopSign * lhs, const eco_interfaces__msg__StopSign * rhs)
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
  // detected
  if (lhs->detected != rhs->detected) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  return true;
}

bool
eco_interfaces__msg__StopSign__copy(
  const eco_interfaces__msg__StopSign * input,
  eco_interfaces__msg__StopSign * output)
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
  // detected
  output->detected = input->detected;
  // distance
  output->distance = input->distance;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  return true;
}

eco_interfaces__msg__StopSign *
eco_interfaces__msg__StopSign__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__StopSign * msg = (eco_interfaces__msg__StopSign *)allocator.allocate(sizeof(eco_interfaces__msg__StopSign), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eco_interfaces__msg__StopSign));
  bool success = eco_interfaces__msg__StopSign__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
eco_interfaces__msg__StopSign__destroy(eco_interfaces__msg__StopSign * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    eco_interfaces__msg__StopSign__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
eco_interfaces__msg__StopSign__Sequence__init(eco_interfaces__msg__StopSign__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__StopSign * data = NULL;

  if (size) {
    data = (eco_interfaces__msg__StopSign *)allocator.zero_allocate(size, sizeof(eco_interfaces__msg__StopSign), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eco_interfaces__msg__StopSign__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eco_interfaces__msg__StopSign__fini(&data[i - 1]);
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
eco_interfaces__msg__StopSign__Sequence__fini(eco_interfaces__msg__StopSign__Sequence * array)
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
      eco_interfaces__msg__StopSign__fini(&array->data[i]);
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

eco_interfaces__msg__StopSign__Sequence *
eco_interfaces__msg__StopSign__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__StopSign__Sequence * array = (eco_interfaces__msg__StopSign__Sequence *)allocator.allocate(sizeof(eco_interfaces__msg__StopSign__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = eco_interfaces__msg__StopSign__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
eco_interfaces__msg__StopSign__Sequence__destroy(eco_interfaces__msg__StopSign__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    eco_interfaces__msg__StopSign__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
eco_interfaces__msg__StopSign__Sequence__are_equal(const eco_interfaces__msg__StopSign__Sequence * lhs, const eco_interfaces__msg__StopSign__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eco_interfaces__msg__StopSign__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eco_interfaces__msg__StopSign__Sequence__copy(
  const eco_interfaces__msg__StopSign__Sequence * input,
  eco_interfaces__msg__StopSign__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eco_interfaces__msg__StopSign);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    eco_interfaces__msg__StopSign * data =
      (eco_interfaces__msg__StopSign *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eco_interfaces__msg__StopSign__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          eco_interfaces__msg__StopSign__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!eco_interfaces__msg__StopSign__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
