// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eco_interfaces:msg/Obstacle.idl
// generated code does not contain a copyright notice
#include "eco_interfaces/msg/detail/obstacle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `points`
// Member `center`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
eco_interfaces__msg__Obstacle__init(eco_interfaces__msg__Obstacle * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    eco_interfaces__msg__Obstacle__fini(msg);
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->points, 0)) {
    eco_interfaces__msg__Obstacle__fini(msg);
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__init(&msg->center)) {
    eco_interfaces__msg__Obstacle__fini(msg);
    return false;
  }
  // radius
  return true;
}

void
eco_interfaces__msg__Obstacle__fini(eco_interfaces__msg__Obstacle * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // points
  geometry_msgs__msg__Point__Sequence__fini(&msg->points);
  // center
  geometry_msgs__msg__Point__fini(&msg->center);
  // radius
}

bool
eco_interfaces__msg__Obstacle__are_equal(const eco_interfaces__msg__Obstacle * lhs, const eco_interfaces__msg__Obstacle * rhs)
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
  // points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->center), &(rhs->center)))
  {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  return true;
}

bool
eco_interfaces__msg__Obstacle__copy(
  const eco_interfaces__msg__Obstacle * input,
  eco_interfaces__msg__Obstacle * output)
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
  // points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__copy(
      &(input->center), &(output->center)))
  {
    return false;
  }
  // radius
  output->radius = input->radius;
  return true;
}

eco_interfaces__msg__Obstacle *
eco_interfaces__msg__Obstacle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__Obstacle * msg = (eco_interfaces__msg__Obstacle *)allocator.allocate(sizeof(eco_interfaces__msg__Obstacle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eco_interfaces__msg__Obstacle));
  bool success = eco_interfaces__msg__Obstacle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
eco_interfaces__msg__Obstacle__destroy(eco_interfaces__msg__Obstacle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    eco_interfaces__msg__Obstacle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
eco_interfaces__msg__Obstacle__Sequence__init(eco_interfaces__msg__Obstacle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__Obstacle * data = NULL;

  if (size) {
    data = (eco_interfaces__msg__Obstacle *)allocator.zero_allocate(size, sizeof(eco_interfaces__msg__Obstacle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eco_interfaces__msg__Obstacle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eco_interfaces__msg__Obstacle__fini(&data[i - 1]);
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
eco_interfaces__msg__Obstacle__Sequence__fini(eco_interfaces__msg__Obstacle__Sequence * array)
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
      eco_interfaces__msg__Obstacle__fini(&array->data[i]);
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

eco_interfaces__msg__Obstacle__Sequence *
eco_interfaces__msg__Obstacle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eco_interfaces__msg__Obstacle__Sequence * array = (eco_interfaces__msg__Obstacle__Sequence *)allocator.allocate(sizeof(eco_interfaces__msg__Obstacle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = eco_interfaces__msg__Obstacle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
eco_interfaces__msg__Obstacle__Sequence__destroy(eco_interfaces__msg__Obstacle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    eco_interfaces__msg__Obstacle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
eco_interfaces__msg__Obstacle__Sequence__are_equal(const eco_interfaces__msg__Obstacle__Sequence * lhs, const eco_interfaces__msg__Obstacle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eco_interfaces__msg__Obstacle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eco_interfaces__msg__Obstacle__Sequence__copy(
  const eco_interfaces__msg__Obstacle__Sequence * input,
  eco_interfaces__msg__Obstacle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eco_interfaces__msg__Obstacle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    eco_interfaces__msg__Obstacle * data =
      (eco_interfaces__msg__Obstacle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eco_interfaces__msg__Obstacle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          eco_interfaces__msg__Obstacle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!eco_interfaces__msg__Obstacle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
