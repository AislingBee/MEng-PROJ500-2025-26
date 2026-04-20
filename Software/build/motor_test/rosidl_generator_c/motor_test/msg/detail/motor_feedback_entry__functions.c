// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_test:msg/MotorFeedbackEntry.idl
// generated code does not contain a copyright notice
#include "motor_test/msg/detail/motor_feedback_entry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
motor_test__msg__MotorFeedbackEntry__init(motor_test__msg__MotorFeedbackEntry * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    motor_test__msg__MotorFeedbackEntry__fini(msg);
    return false;
  }
  // q
  // q_dot
  return true;
}

void
motor_test__msg__MotorFeedbackEntry__fini(motor_test__msg__MotorFeedbackEntry * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // q
  // q_dot
}

bool
motor_test__msg__MotorFeedbackEntry__are_equal(const motor_test__msg__MotorFeedbackEntry * lhs, const motor_test__msg__MotorFeedbackEntry * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // q
  if (lhs->q != rhs->q) {
    return false;
  }
  // q_dot
  if (lhs->q_dot != rhs->q_dot) {
    return false;
  }
  return true;
}

bool
motor_test__msg__MotorFeedbackEntry__copy(
  const motor_test__msg__MotorFeedbackEntry * input,
  motor_test__msg__MotorFeedbackEntry * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // q
  output->q = input->q;
  // q_dot
  output->q_dot = input->q_dot;
  return true;
}

motor_test__msg__MotorFeedbackEntry *
motor_test__msg__MotorFeedbackEntry__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorFeedbackEntry * msg = (motor_test__msg__MotorFeedbackEntry *)allocator.allocate(sizeof(motor_test__msg__MotorFeedbackEntry), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_test__msg__MotorFeedbackEntry));
  bool success = motor_test__msg__MotorFeedbackEntry__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_test__msg__MotorFeedbackEntry__destroy(motor_test__msg__MotorFeedbackEntry * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_test__msg__MotorFeedbackEntry__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_test__msg__MotorFeedbackEntry__Sequence__init(motor_test__msg__MotorFeedbackEntry__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorFeedbackEntry * data = NULL;

  if (size) {
    data = (motor_test__msg__MotorFeedbackEntry *)allocator.zero_allocate(size, sizeof(motor_test__msg__MotorFeedbackEntry), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_test__msg__MotorFeedbackEntry__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_test__msg__MotorFeedbackEntry__fini(&data[i - 1]);
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
motor_test__msg__MotorFeedbackEntry__Sequence__fini(motor_test__msg__MotorFeedbackEntry__Sequence * array)
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
      motor_test__msg__MotorFeedbackEntry__fini(&array->data[i]);
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

motor_test__msg__MotorFeedbackEntry__Sequence *
motor_test__msg__MotorFeedbackEntry__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorFeedbackEntry__Sequence * array = (motor_test__msg__MotorFeedbackEntry__Sequence *)allocator.allocate(sizeof(motor_test__msg__MotorFeedbackEntry__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_test__msg__MotorFeedbackEntry__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_test__msg__MotorFeedbackEntry__Sequence__destroy(motor_test__msg__MotorFeedbackEntry__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_test__msg__MotorFeedbackEntry__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_test__msg__MotorFeedbackEntry__Sequence__are_equal(const motor_test__msg__MotorFeedbackEntry__Sequence * lhs, const motor_test__msg__MotorFeedbackEntry__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_test__msg__MotorFeedbackEntry__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_test__msg__MotorFeedbackEntry__Sequence__copy(
  const motor_test__msg__MotorFeedbackEntry__Sequence * input,
  motor_test__msg__MotorFeedbackEntry__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_test__msg__MotorFeedbackEntry);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_test__msg__MotorFeedbackEntry * data =
      (motor_test__msg__MotorFeedbackEntry *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_test__msg__MotorFeedbackEntry__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_test__msg__MotorFeedbackEntry__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_test__msg__MotorFeedbackEntry__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
