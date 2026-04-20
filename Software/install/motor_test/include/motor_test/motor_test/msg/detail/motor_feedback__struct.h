// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_test:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_feedback.h"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__STRUCT_H_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'motors'
#include "motor_test/msg/detail/motor_feedback_entry__struct.h"

/// Struct defined in msg/MotorFeedback in the package motor_test.
typedef struct motor_test__msg__MotorFeedback
{
  motor_test__msg__MotorFeedbackEntry__Sequence motors;
} motor_test__msg__MotorFeedback;

// Struct for a sequence of motor_test__msg__MotorFeedback.
typedef struct motor_test__msg__MotorFeedback__Sequence
{
  motor_test__msg__MotorFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_test__msg__MotorFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__STRUCT_H_
