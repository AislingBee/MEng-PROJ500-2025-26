// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_test:msg/MotorFeedback.idl
// generated code does not contain a copyright notice
#include "motor_test/msg/detail/motor_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `motors`
#include "motor_test/msg/detail/motor_feedback_entry__functions.h"

bool
motor_test__msg__MotorFeedback__init(motor_test__msg__MotorFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // motors
  if (!motor_test__msg__MotorFeedbackEntry__Sequence__init(&msg->motors, 0)) {
    motor_test__msg__MotorFeedback__fini(msg);
    return false;
  }
  return true;
}

void
motor_test__msg__MotorFeedback__fini(motor_test__msg__MotorFeedback * msg)
{
  if (!msg) {
    return;
  }
  // motors
  motor_test__msg__MotorFeedbackEntry__Sequence__fini(&msg->motors);
}

bool
motor_test__msg__MotorFeedback__are_equal(const motor_test__msg__MotorFeedback * lhs, const motor_test__msg__MotorFeedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motors
  if (!motor_test__msg__MotorFeedbackEntry__Sequence__are_equal(
      &(lhs->motors), &(rhs->motors)))
  {
    return false;
  }
  return true;
}

bool
motor_test__msg__MotorFeedback__copy(
  const motor_test__msg__MotorFeedback * input,
  motor_test__msg__MotorFeedback * output)
{
  if (!input || !output) {
    return false;
  }
  // motors
  if (!motor_test__msg__MotorFeedbackEntry__Sequence__copy(
      &(input->motors), &(output->motors)))
  {
    return false;
  }
  return true;
}

motor_test__msg__MotorFeedback *
motor_test__msg__MotorFeedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorFeedback * msg = (motor_test__msg__MotorFeedback *)allocator.allocate(sizeof(motor_test__msg__MotorFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_test__msg__MotorFeedback));
  bool success = motor_test__msg__MotorFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_test__msg__MotorFeedback__destroy(motor_test__msg__MotorFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_test__msg__MotorFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_test__msg__MotorFeedback__Sequence__init(motor_test__msg__MotorFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorFeedback * data = NULL;

  if (size) {
    data = (motor_test__msg__MotorFeedback *)allocator.zero_allocate(size, sizeof(motor_test__msg__MotorFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_test__msg__MotorFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_test__msg__MotorFeedback__fini(&data[i - 1]);
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
motor_test__msg__MotorFeedback__Sequence__fini(motor_test__msg__MotorFeedback__Sequence * array)
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
      motor_test__msg__MotorFeedback__fini(&array->data[i]);
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

motor_test__msg__MotorFeedback__Sequence *
motor_test__msg__MotorFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorFeedback__Sequence * array = (motor_test__msg__MotorFeedback__Sequence *)allocator.allocate(sizeof(motor_test__msg__MotorFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_test__msg__MotorFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_test__msg__MotorFeedback__Sequence__destroy(motor_test__msg__MotorFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_test__msg__MotorFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_test__msg__MotorFeedback__Sequence__are_equal(const motor_test__msg__MotorFeedback__Sequence * lhs, const motor_test__msg__MotorFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_test__msg__MotorFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_test__msg__MotorFeedback__Sequence__copy(
  const motor_test__msg__MotorFeedback__Sequence * input,
  motor_test__msg__MotorFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_test__msg__MotorFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_test__msg__MotorFeedback * data =
      (motor_test__msg__MotorFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_test__msg__MotorFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_test__msg__MotorFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_test__msg__MotorFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
