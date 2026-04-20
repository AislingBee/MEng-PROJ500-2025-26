// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_test:msg/MotorParam.idl
// generated code does not contain a copyright notice
#include "motor_test/msg/detail/motor_param__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
motor_test__msg__MotorParam__init(motor_test__msg__MotorParam * msg)
{
  if (!msg) {
    return false;
  }
  // q
  // kp
  // kd
  // tau
  return true;
}

void
motor_test__msg__MotorParam__fini(motor_test__msg__MotorParam * msg)
{
  if (!msg) {
    return;
  }
  // q
  // kp
  // kd
  // tau
}

bool
motor_test__msg__MotorParam__are_equal(const motor_test__msg__MotorParam * lhs, const motor_test__msg__MotorParam * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // q
  if (lhs->q != rhs->q) {
    return false;
  }
  // kp
  if (lhs->kp != rhs->kp) {
    return false;
  }
  // kd
  if (lhs->kd != rhs->kd) {
    return false;
  }
  // tau
  if (lhs->tau != rhs->tau) {
    return false;
  }
  return true;
}

bool
motor_test__msg__MotorParam__copy(
  const motor_test__msg__MotorParam * input,
  motor_test__msg__MotorParam * output)
{
  if (!input || !output) {
    return false;
  }
  // q
  output->q = input->q;
  // kp
  output->kp = input->kp;
  // kd
  output->kd = input->kd;
  // tau
  output->tau = input->tau;
  return true;
}

motor_test__msg__MotorParam *
motor_test__msg__MotorParam__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorParam * msg = (motor_test__msg__MotorParam *)allocator.allocate(sizeof(motor_test__msg__MotorParam), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_test__msg__MotorParam));
  bool success = motor_test__msg__MotorParam__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_test__msg__MotorParam__destroy(motor_test__msg__MotorParam * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_test__msg__MotorParam__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_test__msg__MotorParam__Sequence__init(motor_test__msg__MotorParam__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorParam * data = NULL;

  if (size) {
    data = (motor_test__msg__MotorParam *)allocator.zero_allocate(size, sizeof(motor_test__msg__MotorParam), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_test__msg__MotorParam__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_test__msg__MotorParam__fini(&data[i - 1]);
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
motor_test__msg__MotorParam__Sequence__fini(motor_test__msg__MotorParam__Sequence * array)
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
      motor_test__msg__MotorParam__fini(&array->data[i]);
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

motor_test__msg__MotorParam__Sequence *
motor_test__msg__MotorParam__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_test__msg__MotorParam__Sequence * array = (motor_test__msg__MotorParam__Sequence *)allocator.allocate(sizeof(motor_test__msg__MotorParam__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_test__msg__MotorParam__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_test__msg__MotorParam__Sequence__destroy(motor_test__msg__MotorParam__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_test__msg__MotorParam__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_test__msg__MotorParam__Sequence__are_equal(const motor_test__msg__MotorParam__Sequence * lhs, const motor_test__msg__MotorParam__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_test__msg__MotorParam__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_test__msg__MotorParam__Sequence__copy(
  const motor_test__msg__MotorParam__Sequence * input,
  motor_test__msg__MotorParam__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_test__msg__MotorParam);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_test__msg__MotorParam * data =
      (motor_test__msg__MotorParam *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_test__msg__MotorParam__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_test__msg__MotorParam__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_test__msg__MotorParam__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
